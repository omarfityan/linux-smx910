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
#include <drm/drm_plane.h>
#include <drm/drm_print.h>
#include <drm/drm_property.h>

#include "dpu_colorop.h"
#include "dpu_hw_catalog.h"
#include "dpu_hw_sspp.h"
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
 *
 * DRM sends drm_color_lut32 -- 32 bits per channel.  The block stores twelve,
 * two entries to a dword, one table per channel.
 *
 * Return: 0 on success, -EINVAL if the blob is the wrong size.
 */
int dpu_colorop_lut_to_igc(const struct drm_property_blob *blob, u32 *out)
{
	const struct drm_color_lut32 *lut;
	u32 ch, i;

	if (!blob || blob->length != DPU_IGC_LUT_LEN * sizeof(struct drm_color_lut32))
		return -EINVAL;

	lut = blob->data;

	for (ch = 0; ch < DPU_IGC_CHANNELS; ch++) {
		u32 *tbl = out + ch * (DPU_IGC_LUT_LEN / 2);

		for (i = 0; i < DPU_IGC_LUT_LEN / 2; i++) {
			u32 lo, hi;

			switch (ch) {
			case 0:
				lo = lut[2 * i].red;
				hi = lut[2 * i + 1].red;
				break;
			case 1:
				lo = lut[2 * i].green;
				hi = lut[2 * i + 1].green;
				break;
			default:
				lo = lut[2 * i].blue;
				hi = lut[2 * i + 1].blue;
				break;
			}

			lo >>= 32 - DPU_IGC_ENTRY_BITS;
			hi >>= 32 - DPU_IGC_ENTRY_BITS;
			tbl[i] = (lo & DPU_IGC_ENTRY_MASK) |
				 ((hi & DPU_IGC_ENTRY_MASK) << 16);
		}
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
 */
static bool dpu_plane_colorops;
module_param_named(plane_colorops, dpu_plane_colorops, bool, 0600);
MODULE_PARM_DESC(plane_colorops,
	"advertise per-plane colour operations on the video pipes (needs a compositor that converts YCbCr inside the pipeline; off by default because advertising it to one that does not LOSES the working YCbCr path)");

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

	cs = drm_atomic_get_new_colorop_state(state, colorop);
	if (!cs || cs->bypass || !cs->data)
		return false;

	return dpu_colorop_ctm_to_csc(cs->data, out) == 0;
}

/**
 * dpu_plane_colorop_check - can this plane's colour pipeline be programmed?
 * @state:       the atomic state
 * @plane_state: the plane state being checked
 *
 * Every operation the pipeline asks for is converted here, at check time, and
 * the commit is refused if any of them cannot be expressed.  Refusing makes the
 * compositor fall back to composing the frame, which is slower and CORRECT.
 * Accepting and then programming something approximate would be fast and wrong,
 * and would look like a working feature.
 *
 * Return: 0 if the pipeline can be programmed, -EINVAL otherwise.
 */
int dpu_plane_colorop_check(struct drm_atomic_state *state,
			    struct drm_plane_state *plane_state)
{
	struct drm_colorop *colorop = plane_state->color_pipeline;
	struct drm_colorop_state *cs;
	struct dpu_csc_cfg csc;
	u32 *lut;
	int ret = 0;

	if (!colorop)
		return 0;

	/* first: the colour matrix */
	cs = drm_atomic_get_new_colorop_state(state, colorop);
	if (!cs)
		return -EINVAL;
	if (!cs->bypass && cs->data && dpu_colorop_ctm_to_csc(cs->data, &csc)) {
		DPU_ERROR("colour matrix cannot be expressed by this pipe\n");
		return -EINVAL;
	}

	/* second: the inverse-gamma table */
	colorop = colorop->next;
	if (!colorop)
		return -EINVAL;
	cs = drm_atomic_get_new_colorop_state(state, colorop);
	if (!cs)
		return -EINVAL;
	if (!cs->bypass && cs->data) {
		lut = kmalloc_array(DPU_IGC_CHANNELS * (DPU_IGC_LUT_LEN / 2),
				    sizeof(*lut), GFP_KERNEL);
		if (!lut)
			return -ENOMEM;
		if (dpu_colorop_lut_to_igc(cs->data, lut)) {
			DPU_ERROR("inverse-gamma table is not the size this pipe takes\n");
			ret = -EINVAL;
		}
		kfree(lut);
	}

	return ret;
}
