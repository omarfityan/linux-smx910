// SPDX-License-Identifier: GPL-2.0-only
/*
 * Per-plane colour operations for the DPU's video (VIG) source pipes.
 *
 * WHAT THIS IS FOR
 *
 * A compositor can only hand a YCbCr buffer straight to a display plane if the
 * plane can do everything the compositor's colour pipeline asks for.  For SDR
 * video that pipeline is, in order:
 *
 *     YCbCr -> RGB conversion        a 3x4 matrix
 *     the source's EOTF              a curve
 *     a clamp                        pushed unconditionally for limited range
 *     the output's inverse EOTF      a curve
 *
 * The three curve-shaped operations collapse into one sampled table, so the
 * minimum a plane must advertise is a matrix followed by a 1D LUT.  This device
 * has both, per pipe: its device tree declares
 *
 *     qcom,sde-vig-csc-off = <0x1a00>
 *     qcom,sde-vig-igc     = <0x1d00 0x60000>
 *
 * for each of its four VIG pipes.  The CSC is plain MMIO this driver already
 * programs; the inverse-gamma table is written through the register-DMA engine.
 *
 * ORDER IS NOT A DETAIL.  The chain is declared CSC first, then the table, and
 * a compositor walks it forwards only.  It is also the order the silicon
 * implements: the table's three channels are c0/c1/c2 in the RGB domain, and
 * "inverse gamma" is not a meaningful operation on Y/Cb/Cr, so the conversion
 * must precede it.  A chain declared in an order the hardware does not have
 * produces a WRONG PICTURE rather than an error.
 *
 * BYPASS IS MANDATORY, NOT DECORATION.  A compositor that has to step over one
 * of these blocks -- and one that finds blocks left over after it has placed
 * every operation -- gives up on the whole plane unless each of them can be
 * bypassed.  Both ops are therefore created with DRM_COLOROP_FLAG_ALLOW_BYPASS.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_colorop.h>
#include <drm/drm_crtc.h>
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_property.h>

#include "dpu_colorop.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_reg_dma.h"
#include "dpu_hw_sspp.h"
#include "dpu_colorop_igc.h"
#include "dpu_kms.h"
#include "dpu_plane.h"

/*
 * The inverse-gamma table is 256 entries per channel on this part
 * (IGC_TBL_LEN in the vendor's own uapi header), three channels, packed two
 * twelve-bit entries to a dword.
 */
#define DPU_IGC_LUT_LEN		256
#define DPU_IGC_CHANNELS	3
#define DPU_IGC_ENTRY_BITS	12
#define DPU_IGC_ENTRY_MASK	((1u << DPU_IGC_ENTRY_BITS) - 1)

/*
 * The colour-matrix block works in the pipe's 10-bit domain, and the bias
 * registers count in that domain's code values.  The scale is not a guess: the
 * vendor's own shipped tables put -16/255 and -0.5 of full scale in as 0xffc0
 * (-64) and 0xfe00 (-512), which is exactly x1024.
 */
#define DPU_CSC_BIAS_SCALE_SHIFT	10

/*
 * dpu_kms tracks which pipes have inverse gamma switched on as a bit per pipe
 * in a u32.  Seventeen today; say so out loud rather than discover it on a part
 * with more, where the overflow would silently stop pipes being turned off.
 */
static_assert(SSPP_MAX <= 32, "the inverse-gamma on-mask is a u32, one bit a pipe");

/**
 * dpu_ctm_entry_to_s15_16 - decode one DRM colour-matrix coefficient
 * @v: the coefficient, S31.32 in SIGN-MAGNITUDE form
 *
 * DRM does not store these in two's complement: bit 63 is a sign flag and the
 * remaining bits are a magnitude.  A plain cast would read every negative
 * coefficient as an enormous positive one.
 *
 * Return: the same value as S15.16, two's complement, which is the form
 *         struct dpu_csc_cfg uses.
 */
static s32 dpu_ctm_entry_to_s15_16(u64 v)
{
	s64 mag = (s64)((v & ~(1ULL << 63)) >> 16);

	return (v & (1ULL << 63)) ? (s32)-mag : (s32)mag;
}

/* a * b, both S15.16, result S15.16 */
static s64 dpu_fx_mul(s64 a, s64 b)
{
	return (a * b) >> 16;
}

/**
 * dpu_fx_shift_round - shift right, rounding to nearest rather than flooring
 * @v:    the value
 * @bits: how far to shift
 *
 * A plain arithmetic shift FLOORS, so -64.25 becomes -65 rather than -64.
 * Every bias here is negative -- the chroma centres and the black level -- so
 * flooring is wrong on all of them, by one code value each.  It is worth a
 * helper because it is the difference between reproducing this device's own
 * shipped constants and missing them by one.
 */
static s64 dpu_fx_shift_round(s64 v, int bits)
{
	s64 half = 1LL << (bits - 1);

	return (v >= 0) ? ((v + half) >> bits) : -(((-v) + half) >> bits);
}

static s64 dpu_fx_det3(const s64 m[3][3])
{
	return dpu_fx_mul(m[0][0], dpu_fx_mul(m[1][1], m[2][2]) - dpu_fx_mul(m[1][2], m[2][1]))
	     - dpu_fx_mul(m[0][1], dpu_fx_mul(m[1][0], m[2][2]) - dpu_fx_mul(m[1][2], m[2][0]))
	     + dpu_fx_mul(m[0][2], dpu_fx_mul(m[1][0], m[2][1]) - dpu_fx_mul(m[1][1], m[2][0]));
}

/**
 * dpu_csc_solve_pre_bias - turn a post-matrix offset into a pre-matrix one
 * @m:   the 3x3 matrix, S15.16
 * @t:   the offset DRM puts AFTER the matrix, S15.16, normalised
 * @pre: where to write the offset to apply BEFORE it, S15.16, normalised
 *
 * DRM's matrix computes out = M*in + t.  This block computes
 * out = M*(in + pre) + post, and only the PRE register has units this device
 * has ever demonstrated -- every shipped table leaves POST at zero.  So the
 * offset is moved across the matrix by solving M*pre = t, by Cramer's rule.
 *
 * Return: 0, or -EINVAL if the matrix is too near singular to invert, in which
 *         case the caller must refuse the pipeline rather than program a
 *         plausible-looking wrong one.
 */
static int dpu_csc_solve_pre_bias(const s64 m[3][3], const s64 t[3], s64 pre[3])
{
	s64 det = dpu_fx_det3(m);
	int c, r, cc;

	/*
	 * 1/256 in S15.16.  Below this the division below stops being
	 * meaningful at this word length, and a YCbCr conversion matrix is
	 * nowhere near it -- the determinants seen here are of order 1.
	 */
	if (det > -256 && det < 256)
		return -EINVAL;

	for (c = 0; c < 3; c++) {
		s64 mc[3][3];

		for (r = 0; r < 3; r++)
			for (cc = 0; cc < 3; cc++)
				mc[r][cc] = (cc == c) ? t[r] : m[r][cc];

		pre[c] = div64_s64(dpu_fx_det3(mc) << 16, det);
	}

	return 0;
}

/**
 * dpu_colorop_ctm_to_csc - convert a DRM 3x4 colour matrix into a pipe CSC config
 * @blob: the DATA blob of a DRM_COLOROP_CTM_3X4 colorop
 * @cfg:  the config to fill in
 *
 * Return: 0 on success, -EINVAL if the blob is the wrong size or the matrix
 *         cannot be expressed by this block.
 */
int dpu_colorop_ctm_to_csc(const struct drm_property_blob *blob,
			   struct dpu_csc_cfg *cfg)
{
	const struct drm_color_ctm_3x4 *ctm;
	s64 m[3][3], t[3], pre[3];
	int r, c, ret;

	if (!blob || blob->length != sizeof(struct drm_color_ctm_3x4))
		return -EINVAL;

	ctm = blob->data;

	for (r = 0; r < 3; r++) {
		for (c = 0; c < 3; c++)
			m[r][c] = dpu_ctm_entry_to_s15_16(ctm->matrix[r * 4 + c]);
		t[r] = dpu_ctm_entry_to_s15_16(ctm->matrix[r * 4 + 3]);
	}

	ret = dpu_csc_solve_pre_bias(m, t, pre);
	if (ret)
		return ret;

	memset(cfg, 0, sizeof(*cfg));

	for (r = 0; r < 3; r++)
		for (c = 0; c < 3; c++)
			cfg->csc_mv[r * 3 + c] = (u32)(s32)m[r][c];

	/*
	 * S15.16 normalised -> 10-bit code values, kept as a sixteen-bit signed
	 * quantity because that is the width the vendor's own values use.
	 */
	for (r = 0; r < 3; r++) {
		s64 code = dpu_fx_shift_round(pre[r] * (1 << DPU_CSC_BIAS_SCALE_SHIFT), 16);

		cfg->csc_pre_bv[r] = (u32)(code & 0xffff);
	}

	/*
	 * Clamps: the input is whatever the compositor says it is, so the pre
	 * clamp is left wide; the output is a 10-bit RGB value and is clamped to
	 * the full range, matching what the driver's own full-range entries do.
	 */
	cfg->csc_pre_lv[0] = 0x00; cfg->csc_pre_lv[1] = 0x3ff;
	cfg->csc_pre_lv[2] = 0x00; cfg->csc_pre_lv[3] = 0x3ff;
	cfg->csc_pre_lv[4] = 0x00; cfg->csc_pre_lv[5] = 0x3ff;
	cfg->csc_post_lv[0] = 0x00; cfg->csc_post_lv[1] = 0x3ff;
	cfg->csc_post_lv[2] = 0x00; cfg->csc_post_lv[3] = 0x3ff;
	cfg->csc_post_lv[4] = 0x00; cfg->csc_post_lv[5] = 0x3ff;

	return 0;
}

/**
 * dpu_colorop_lut_to_igc - convert a DRM 1D LUT into the pipe's packed table
 * @blob: the DATA blob of a DRM_COLOROP_1D_LUT colorop
 * @out:  DPU_IGC_CHANNELS * (DPU_IGC_LUT_LEN / 2) dwords to fill
 * @last: DPU_IGC_CHANNELS endpoint values to fill
 *
 * DRM sends drm_color_lut32 -- 32 bits per channel, entries spread uniformly
 * over the input range.  The block stores twelve bits, two entries to a dword,
 * one table per channel, and reads slot i at ten-bit input 4i with a separate
 * endpoint past the last slot: so each channel is RESAMPLED onto the block's
 * grid (dpu_colorop_igc.h, host-tested against the block's measured
 * behaviour), never copied slot for slot -- copied, every curve was read 0.3 %
 * compressed, several codes near a PQ curve's knee.
 *
 * Return: 0 on success, -EINVAL if the blob is the wrong size.
 */
int dpu_colorop_lut_to_igc(const struct drm_property_blob *blob, u32 *out, u32 *last)
{
	const struct drm_color_lut32 *lut;
	u16 slots[DPU_IGC_SLOTS], endpoint;
	u32 ch, i;

	if (!blob || blob->length != DPU_IGC_LUT_LEN * sizeof(struct drm_color_lut32))
		return -EINVAL;

	lut = blob->data;

	for (ch = 0; ch < DPU_IGC_CHANNELS; ch++) {
		u32 *tbl = out + ch * (DPU_IGC_LUT_LEN / 2);

		/* red, green and blue are consecutive u32s of each drm_color_lut32 */
		dpu_igc_convert_channel(&lut[0].red + ch,
					sizeof(struct drm_color_lut32) / sizeof(u32),
					DPU_IGC_LUT_LEN, slots, &endpoint);
		for (i = 0; i < DPU_IGC_LUT_LEN / 2; i++)
			tbl[i] = (slots[2 * i] & DPU_IGC_ENTRY_MASK) |
				 ((slots[2 * i + 1] & DPU_IGC_ENTRY_MASK) << 16);
		last[ch] = endpoint & DPU_IGC_ENTRY_MASK;
	}

	return 0;
}

/*
 * Whether the colour pipeline is advertised at all.
 *
 * 🔴 THIS DEFAULTS OFF ON PURPOSE, AND THE REASON IS NOT CAUTION.
 *
 * A compositor that sees COLOR_PIPELINE on a plane stops using the plane's
 * COLOR_ENCODING/COLOR_RANGE path for it.  KWin up to and including 6.6.6
 * goes further: it refuses EVERY non-identity YCbCr buffer on any plane that
 * merely ADVERTISES the property, because the check is on the property
 * existing, not on a pipeline having matched.  So on an unpatched compositor,
 * turning this on does not add a capability -- it REMOVES the one that works
 * today, and nothing replaces it.
 *
 * It becomes useful only alongside a compositor that carries upstream's
 * fc67ed23c7, which moves the YCbCr conversion into the pipeline and deletes
 * that refusal.  The two land together or neither, and this parameter is what
 * makes "together" expressible without shipping a regression in between.
 *
 * 🔴 IT IS READ WHEN THE PLANES ARE CREATED, which is once, at probe.  Writing
 * /sys/module/msm/parameters/plane_colorops on a running system therefore
 * SUCCEEDS AND DOES NOTHING -- the planes already exist.  Turning it on means
 * msm.plane_colorops=1 on the kernel command line, which on this device lives in
 * the boot image and needs a flash.  The file is left writable so its value can
 * be READ BACK to confirm what the running kernel was told.
 */
static bool dpu_plane_colorops;
module_param_named(plane_colorops, dpu_plane_colorops, bool, 0600);
MODULE_PARM_DESC(plane_colorops,
	"advertise per-plane colour operations on the video pipes; needs msm.plane_colorops=1 on the COMMAND LINE because it is read at plane creation, and needs a compositor that converts YCbCr inside the pipeline -- off by default because advertising it to one that does not LOSES the working YCbCr path");

/*
 * A TEST SWITCH, off by default: leave a pipe's inverse gamma ON when no plane
 * wants it any more, instead of switching it off.
 *
 * It exists to answer one question.  A table load that follows a switch-off has
 * been measured NOT to reach the pixels -- the pipeline keeps the previous
 * curve until the display power-collapses -- and every such load was also not
 * the first enable since the power-up.  Whether a load lands when the block is
 * still on (the device's own driver reloads that way, flipping the bank bit)
 * separates the two, and this switch is how to produce that load.
 *
 * 🔴 IT IS NOT SAFE IN GENERAL: a pipe left on shows the next surface it
 * carries through the previous curve.  It is runtime-writable only so an arm
 * can turn it on around one measurement and back off.
 */
static bool dpu_colorop_igc_keep_on;
module_param_named(colorop_igc_keep_on, dpu_colorop_igc_keep_on, bool, 0644);
MODULE_PARM_DESC(colorop_igc_keep_on,
	"TEST ONLY, default off: leave a video pipe's inverse gamma on when no plane wants it, instead of switching it off");

/*
 * A TEST SWITCH, off by default: load every video pipe's inverse-gamma table
 * INVERTED, each twelve-bit entry v as 4095 - v.
 *
 * It exists because a measurement of this block needs a THIRD table that
 * differs from two others in the table and in nothing else.  The block has two
 * banks, so with only two tables a bank often still holds the "new" one from an
 * earlier load, and a load that wrote nothing can pass for one that landed.  The
 * colour matrix is programmed fresh every commit and lives in no bank, so the
 * third table must come with the SAME matrix -- and the compositor could not
 * supply one: the transfer tags that survive this decode path give two such
 * tables, and the ones that would give a third either move a luminance scale
 * into the matrix or leave the plane path altogether.  Inverting what the
 * compositor sent is deterministic, keeps its matrix by construction, and
 * reproduces exactly every time.
 *
 * The table's id is tagged while it is on, so an inverted table is never taken
 * for the one it was made from and a toggle is always a reload.  🔴 NOT SAFE IN
 * GENERAL: the video is a negative while it is on.  Toggle it only while nothing
 * is on the plane path, and turn it back off.
 */
static bool dpu_colorop_igc_test_invert;
module_param_named(colorop_igc_test_invert, dpu_colorop_igc_test_invert, bool, 0644);
MODULE_PARM_DESC(colorop_igc_test_invert,
	"TEST ONLY, default off: load every video pipe's inverse-gamma table inverted, each entry v as 4095 - v -- a third table with the compositor's own colour matrix; the video is a negative while it is on");

/* DRM object ids are positive ints, so the top bit is free to mark a transformed table */
#define DPU_COLOROP_IGC_ID_INVERTED	BIT(31)

static_assert(DPU_IGC_CHANNELS * (DPU_IGC_LUT_LEN / 2) == DPU_PLANE_IGC_DWORDS,
	      "the inversion walks exactly the plane state's packed table");
static_assert(DPU_IGC_CHANNELS == DPU_PLANE_IGC_CHANNELS, "one endpoint per channel");
static_assert(DPU_IGC_LUT_LEN == DPU_IGC_SLOTS, "the conversion fills exactly the block's slots");

static void dpu_colorop_igc_invert(u32 *tbl, u32 *last)
{
	u32 i;

	for (i = 0; i < DPU_IGC_CHANNELS * (DPU_IGC_LUT_LEN / 2); i++) {
		u32 lo = tbl[i] & DPU_IGC_ENTRY_MASK;
		u32 hi = (tbl[i] >> 16) & DPU_IGC_ENTRY_MASK;

		tbl[i] = (DPU_IGC_ENTRY_MASK - lo) | ((DPU_IGC_ENTRY_MASK - hi) << 16);
	}
	/* the endpoint is part of the curve: inverted the same way */
	for (i = 0; i < DPU_IGC_CHANNELS; i++)
		last[i] = DPU_IGC_ENTRY_MASK - (last[i] & DPU_IGC_ENTRY_MASK);
}

/**
 * dpu_plane_colorops_enabled - is the colour-operation path advertised at all?
 *
 * The register-DMA engine the table needs is a device-lifetime resource, and
 * allocating it on a machine that will never advertise a pipeline is a page and
 * a register mapping for nothing.
 */
bool dpu_plane_colorops_enabled(void)
{
	return dpu_plane_colorops;
}

/**
 * dpu_plane_init_colorops - advertise this pipe's colour blocks as a DRM pipeline
 * @plane:      the plane
 * @has_blocks: whether the pipe (or, for a virtual plane, the largest pipe)
 *              carries both a colour matrix and an inverse-gamma table
 *
 * Builds the two-element chain the compositor needs -- the colour matrix, then
 * the inverse-gamma table -- and attaches it as the plane's one colour
 * pipeline.  Does nothing unless the pipe actually has both blocks.
 *
 * Return: 0 on success or when there is nothing to advertise, negative on error.
 */
int dpu_plane_init_colorops(struct drm_plane *plane, bool has_blocks)
{
	struct drm_prop_enum_list pipeline = {};
	struct drm_colorop *csc_op = NULL, *igc_op = NULL;
	struct drm_device *dev = plane->dev;
	int ret;

	if (!dpu_plane_colorops)
		return 0;

	/*
	 * Both blocks or neither, decided by the caller -- a virtual plane has
	 * no pipe until check time, so it advertises what the largest pipe can
	 * do, the same rule COLOR_ENCODING follows.  Advertising a pipeline
	 * that cannot do the whole job is worse than advertising none: the
	 * compositor gives up on the plane, having already stopped using the
	 * path that worked.
	 */
	if (!has_blocks)
		return 0;

	csc_op = kzalloc(sizeof(*csc_op), GFP_KERNEL);
	igc_op = kzalloc(sizeof(*igc_op), GFP_KERNEL);
	if (!csc_op || !igc_op) {
		ret = -ENOMEM;
		goto err;
	}

	/* the matrix comes FIRST: the table's channels are RGB, and inverse
	 * gamma is not a meaningful operation on Y/Cb/Cr */
	ret = drm_plane_colorop_ctm_3x4_init(dev, csc_op, plane,
					     DRM_COLOROP_FLAG_ALLOW_BYPASS);
	if (ret)
		goto err;

	pipeline.type = csc_op->base.id;

	ret = drm_plane_colorop_curve_1d_lut_init(dev, igc_op, plane,
						  DPU_IGC_LUT_LEN,
						  DRM_COLOROP_LUT1D_INTERPOLATION_LINEAR,
						  DRM_COLOROP_FLAG_ALLOW_BYPASS);
	if (ret)
		goto err;

	drm_colorop_set_next_property(csc_op, igc_op);

	pipeline.name = kasprintf(GFP_KERNEL, "Color Pipeline %d", csc_op->base.id);
	if (!pipeline.name) {
		ret = -ENOMEM;
		goto err;
	}

	ret = drm_plane_create_color_pipeline_property(plane, &pipeline, 1);
	kfree(pipeline.name);
	if (ret)
		goto err;

	/*
	 * 🔴 THE POSITIVE CONTROL FOR EVERYTHING BELOW.  A clean compile says
	 * nothing about whether code is reached, and on this device finding that
	 * out costs a build, a flash and a reboot.  This line establishes that
	 * the property exists on a real plane, so that a run with no further
	 * colorop output means the COMPOSITOR did not ask -- not that the driver
	 * never offered.
	 */
	DRM_INFO("colorop: %s advertises a colour pipeline: 3x4 matrix (id %u) -> %u-entry 1D LUT (id %u)\n",
		 plane->name, csc_op->base.id, DPU_IGC_LUT_LEN, igc_op->base.id);

	return 0;

err:
	if (igc_op) {
		drm_colorop_cleanup(igc_op);
		kfree(igc_op);
	}
	if (csc_op) {
		drm_colorop_cleanup(csc_op);
		kfree(csc_op);
	}
	DPU_ERROR("failed to advertise colour operations on %s: %d\n",
		  plane->name, ret);
	return ret;
}

/**
 * dpu_colorop_effective_state - the state this colour operation is actually in
 *
 * A colorop the client did not touch is not in the commit, and its COMMITTED state
 * is what the hardware is left with.  Refusing then would turn "this client only
 * set COLOR_PIPELINE" into a rejection indistinguishable from the hardware saying
 * no -- and on this device telling those two apart costs a build, a flash and a
 * reboot.  KWin does touch every operation in a chain it matches, so this is the
 * defensive branch rather than the expected one.
 */
static struct drm_colorop_state *
dpu_colorop_effective_state(struct drm_atomic_state *state,
			    struct drm_colorop *colorop)
{
	struct drm_colorop_state *cs = drm_atomic_get_new_colorop_state(state, colorop);

	return cs ? cs : colorop->state;
}

/**
 * dpu_plane_colorop_csc - the colour matrix this plane's pipeline asks for
 * @state: the plane state being checked or committed
 * @out:   filled in when a matrix is in force
 *
 * Return: true when @out was filled in, false when the plane has no pipeline,
 *         the matrix operation is bypassed, or it carries no data -- in which
 *         case the caller keeps whatever it would have done otherwise.
 */
bool dpu_plane_colorop_csc(struct drm_atomic_state *state,
			   struct drm_plane_state *plane_state,
			   struct dpu_csc_cfg *out)
{
	struct drm_colorop *colorop = plane_state->color_pipeline;
	struct drm_colorop_state *cs;

	if (!colorop)
		return false;

	cs = dpu_colorop_effective_state(state, colorop);
	if (!cs || cs->bypass || !cs->data)
		return false;

	return dpu_colorop_ctm_to_csc(cs->data, out) == 0;
}

/* what the compositor placed, as booleans -- reported when it changes, never per frame */
#define DPU_COLOROP_SHAPE_PIPELINE	BIT(0)
#define DPU_COLOROP_SHAPE_CSC_BYPASS	BIT(1)
#define DPU_COLOROP_SHAPE_CSC_DATA	BIT(2)
#define DPU_COLOROP_SHAPE_IGC_BYPASS	BIT(3)
#define DPU_COLOROP_SHAPE_IGC_DATA	BIT(4)

/**
 * dpu_plane_colorop_check - can this plane's colour pipeline be programmed?
 * @state:       the atomic state
 * @plane_state: the plane state being checked
 * @reports:     what this plane's colour-pipeline reports last said (logging only)
 *
 * Every operation the pipeline asks for is converted here, at check time, and
 * the commit is refused if any of them cannot be expressed.  Refusing makes the
 * compositor fall back to composing the frame, which is slower and CORRECT.
 * Accepting and then programming something approximate would be fast and wrong,
 * and would look like a working feature.
 *
 * The inverse-gamma table is converted INTO THE PLANE STATE rather than into a
 * scratch buffer, because the flush path has only plane->state and cannot reach
 * a colorop's atomic state -- the same reason the colour matrix is kept there.
 *
 * Return: 0 if the pipeline can be programmed, -EINVAL otherwise.
 */
int dpu_plane_colorop_check(struct drm_atomic_state *state,
			    struct drm_plane_state *plane_state,
			    struct dpu_colorop_reports *reports)
{
	struct dpu_plane_state *pstate = to_dpu_plane_state(plane_state);
	struct drm_colorop *colorop = plane_state->color_pipeline;
	struct drm_plane_state *old_plane_state;
	struct drm_colorop_state *cs;
	struct dpu_csc_cfg csc;
	bool changed, report;
	u32 shape = 0;

	/*
	 * Reached at all?  This check was once hooked into a function VIRTUAL
	 * PLANES NEVER REACH, and it compiled perfectly.  One line a boot is the
	 * price of never having to wonder again.
	 */
	DRM_INFO_ONCE("colorop: the plane colour-pipeline check is running\n");

	pstate->has_pipeline_igc = false;
	pstate->pipeline_igc_id = 0;

	if (colorop) {
		shape |= DPU_COLOROP_SHAPE_PIPELINE;

		/* first: the colour matrix */
		cs = dpu_colorop_effective_state(state, colorop);
		if (!cs) {
			if (dpu_colorop_report_refusal(reports, DPU_COLOROP_REFUSAL_NO_STATE))
				DPU_ERROR("a colour operation has no state at all\n");
			return -EINVAL;
		}
		if (cs->bypass)
			shape |= DPU_COLOROP_SHAPE_CSC_BYPASS;
		if (cs->data)
			shape |= DPU_COLOROP_SHAPE_CSC_DATA;
		if (!cs->bypass && cs->data &&
		    dpu_colorop_ctm_to_csc(cs->data, &csc)) {
			if (dpu_colorop_report_refusal(reports, DPU_COLOROP_REFUSAL_MATRIX))
				DPU_ERROR("colour matrix cannot be expressed by this pipe\n");
			return -EINVAL;
		}

		/* second: the inverse-gamma table */
		colorop = colorop->next;
		if (!colorop)
			return -EINVAL;
		cs = dpu_colorop_effective_state(state, colorop);
		if (!cs) {
			if (dpu_colorop_report_refusal(reports, DPU_COLOROP_REFUSAL_NO_STATE))
				DPU_ERROR("a colour operation has no state at all\n");
			return -EINVAL;
		}
		if (cs->bypass)
			shape |= DPU_COLOROP_SHAPE_IGC_BYPASS;
		if (cs->data)
			shape |= DPU_COLOROP_SHAPE_IGC_DATA;
		if (!cs->bypass && cs->data) {
			if (dpu_colorop_lut_to_igc(cs->data, pstate->pipeline_igc,
						   pstate->pipeline_igc_last)) {
				if (dpu_colorop_report_refusal(reports,
							       DPU_COLOROP_REFUSAL_TABLE_SIZE))
					DPU_ERROR("inverse-gamma table is not the size this pipe takes\n");
				return -EINVAL;
			}
			pstate->has_pipeline_igc = true;
			pstate->pipeline_igc_id = cs->data->base.id;
			if (dpu_colorop_igc_test_invert) {
				dpu_colorop_igc_invert(pstate->pipeline_igc,
						       pstate->pipeline_igc_last);
				pstate->pipeline_igc_id |= DPU_COLOROP_IGC_ID_INVERTED;
			}
		}
	}

	pstate->pipeline_shape = shape;

	/*
	 * 🔴 SAY WHAT THE COMPOSITOR PLACED, not merely that it placed something.
	 *
	 * Without this line, "the compositor never asked for a pipeline" and "it
	 * asked and the hardware refused" are indistinguishable afterwards, and
	 * on this device finding that out costs a build, a flash and a reboot.
	 * "A pipeline arrived with both operations bypassed" and "a pipeline
	 * arrived carrying a real table" are also different worlds, so both
	 * flags are named rather than summarised.
	 *
	 * ⚠️ THIS IS CHECK TIME.  It fires on TEST_ONLY probes that never commit,
	 * which is exactly what makes it useful -- a compositor trying and being
	 * refused shows up here -- but it is not evidence that a frame was
	 * scanned out.  Only the flush path's report is that.
	 *
	 * It is emitted when the shape CHANGES, against the old plane state, so
	 * a clip playing at 120 Hz produces one line and not seven thousand.
	 *
	 * And not while a refused pipeline and the compositor's fall-back
	 * alternate: a refused check never becomes the old state and the
	 * fall-back commits shape 0, so against the old state alone a refused
	 * pipeline was re-reported on every retry.  The plane's own memory
	 * (dpu_report_state_note) silences the second and later returns within
	 * a second, and a shape this plane checked again within a second.  It is
	 * noted on EVERY check -- also when the old-state test fails, or a held
	 * shape's last sighting would go stale -- and both must agree, so this
	 * never prints a line the old-state test alone would not.
	 *
	 * So "no line" for a shape means it matched the committed one, or this
	 * plane checked it in the preceding second.  Unlike the fb report's, the
	 * memory's current shape EXPIRES after that second: on a plane whose
	 * fall-back never reaches this check -- switched off, or left out of the
	 * commit -- the memory keeps a refused offer's shape, and without the
	 * expiry an offer accepted later with that shape would print no line at
	 * any age.  An idle plane stays quiet because its shape is the committed
	 * one.
	 */
	old_plane_state = drm_atomic_get_old_plane_state(state, plane_state->plane);
	changed = !old_plane_state ||
		  to_dpu_plane_state(old_plane_state)->pipeline_shape != shape;
	report = dpu_report_state_note(&reports->shape, shape, jiffies,
				       msecs_to_jiffies(DPU_REPORT_WINDOW_MS));
	if (report && changed)
		DRM_INFO("colorop: %s at check: %s | matrix %s%s | table %s%s\n",
			 plane_state->plane->name,
			 (shape & DPU_COLOROP_SHAPE_PIPELINE)
				? "A COLOUR PIPELINE WAS SELECTED"
				: "no colour pipeline",
			 (shape & DPU_COLOROP_SHAPE_CSC_BYPASS) ? "bypassed" : "in force",
			 (shape & DPU_COLOROP_SHAPE_CSC_DATA) ? ", has data" : ", NO DATA",
			 (shape & DPU_COLOROP_SHAPE_IGC_BYPASS) ? "bypassed" : "in force",
			 (shape & DPU_COLOROP_SHAPE_IGC_DATA) ? ", has data" : ", NO DATA");

	return 0;
}

/**
 * dpu_colorop_crtc_flush - put every pipe's inverse gamma where this frame wants it
 * @crtc: the CRTC being flushed
 * @ctl:  its control block, which is what triggers the engine's transfer
 *
 * Called from the CRTC's flush, immediately after each plane has written its
 * colour matrix, so the two halves of a pipeline land in the same window.
 *
 * 🔴 WHY THIS WALKS PIPES AND NOT PLANES.  A source pipe is not owned by a
 * plane: a virtual plane is assigned one at check time and can be given a
 * different one next frame.  A pipe left behind still has inverse gamma on and
 * the previous clip's curve loaded, and the next surface to land on it comes
 * out visibly wrong with nothing in the frame to explain why.  So the pipes
 * this frame wants are collected first and everything else the driver has ever
 * turned on is turned back off.
 *
 * It walks EVERY plane on the device rather than this CRTC's, because the new
 * states are already swapped in by the time a flush runs -- so plane->state is
 * current for all of them -- and a pipe another CRTC is using has to be counted
 * as wanted or this sweep would switch it off underneath that CRTC.  Only
 * planes on THIS CRTC are programmed, because @ctl is this CRTC's.
 */
void dpu_colorop_crtc_flush(struct drm_crtc *crtc, struct dpu_hw_ctl *ctl)
{
	struct msm_drm_private *priv = crtc->dev->dev_private;
	struct dpu_kms *dpu_kms = to_dpu_kms(priv->kms);
	struct dpu_reg_dma *rdma = &dpu_kms->colorop_rdma;
	struct drm_plane *plane;
	u32 wanted = 0;
	int i, s;

	if (!rdma->ready || !ctl)
		return;

	DRM_INFO_ONCE("colorop: the inverse-gamma flush path is live on ctl%d\n",
		      ctl->idx - CTL_0);

	drm_for_each_plane(plane, crtc->dev) {
		struct dpu_plane_state *pstate;

		if (!plane->state || !plane->state->crtc)
			continue;
		pstate = to_dpu_plane_state(plane->state);
		if (!pstate->has_pipeline_igc)
			continue;

		for (i = 0; i < PIPES_PER_PLANE; i++) {
			struct dpu_hw_sspp *sspp = pstate->pipe[i].sspp;
			bool same;
			u32 bit;
			int ret;

			/*
			 * A plane that is not visible has had its pipes
			 * cleared by dpu_plane_virtual_atomic_check, which is
			 * what makes this the whole invisibility test.
			 */
			if (!sspp || !sspp->cap->sblk->igc_blk.len)
				continue;
			bit = BIT(sspp->idx);

			/*
			 * Two rectangles of a multirect plane share one pipe,
			 * so the second time round is the same hardware.
			 */
			if (wanted & bit)
				continue;

			/* a pipe on another CRTC is that CRTC's to program */
			if (plane->state->crtc != crtc) {
				wanted |= bit;
				continue;
			}

			/*
			 * Nothing to do when the hardware still HAS this exact
			 * table.  Asked of the register rather than of a
			 * remembered bit, because the display controller
			 * power-collapses whenever the panel blanks and takes
			 * the curve with it -- see dpu_reg_dma_sspp_igc_is_on.
			 *
			 * Skipping is not only a saving: the engine is waited
			 * on synchronously with a 1 ms bound, and a 384-dword
			 * transfer inside every commit would be a confound in
			 * any measurement taken afterwards.
			 */
			same = dpu_kms->colorop_igc_lut_id[sspp->idx] ==
					pstate->pipeline_igc_id;
			if (same && dpu_reg_dma_sspp_igc_is_on(sspp)) {
				wanted |= bit;
				continue;
			}

			/*
			 * 🔴 AND DO NOT RETRY A TABLE THAT ALREADY FAILED.
			 * A failing transfer costs up to a millisecond of a
			 * 8.3 ms frame plus a read-back and a log line, and it
			 * would fail again for the same reason every frame --
			 * which is both a flood and a confound, in exactly the
			 * outcome a first attempt is most likely to produce.
			 * The latch lifts when the compositor sends a
			 * different curve.
			 */
			if (same && (dpu_kms->colorop_igc_failed & bit))
				continue;

			ret = dpu_reg_dma_program_sspp_igc(dpu_kms, rdma, ctl, sspp,
							   pstate->pipeline_igc,
							   pstate->pipeline_igc_last, true);
			dpu_kms->colorop_igc_lut_id[sspp->idx] = pstate->pipeline_igc_id;
			if (ret) {
				/*
				 * The sequence turns the block on before its
				 * last operation, so a failure part-way leaves
				 * inverse gamma enabled over a half-written
				 * curve.  Off is the only safe state to fail to.
				 */
				dpu_kms->colorop_igc_failed |= bit;
				dpu_reg_dma_disable_sspp_igc(dpu_kms, rdma, ctl, sspp);
				continue;
			}

			dpu_kms->colorop_igc_failed &= ~bit;
			wanted |= bit;
		}
	}

	for (s = SSPP_NONE + 1; s < SSPP_MAX; s++) {
		struct dpu_hw_sspp *sspp;

		if (!(dpu_kms->colorop_igc_on & BIT(s)) || (wanted & BIT(s)))
			continue;

		/* the test switch above: still on, so still tracked as on */
		if (dpu_colorop_igc_keep_on) {
			wanted |= BIT(s);
			continue;
		}

		sspp = dpu_rm_get_sspp(&dpu_kms->rm, s);
		if (sspp)
			dpu_reg_dma_disable_sspp_igc(dpu_kms, rdma, ctl, sspp);
	}

	dpu_kms->colorop_igc_on = wanted;
}
