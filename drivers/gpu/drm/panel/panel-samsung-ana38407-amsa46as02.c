// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung AMSA46AS02 MIPI-DSI command-mode AMOLED panel (Anapass ANA38407 DDIC)
 * as fitted to the Samsung Galaxy Tab S9 Ultra (SM-X910 / gts9u).
 *
 * 2960x1848, 4-lane, DSC v1.1 (driver-side PPS), hardware TE. The DDIC is
 * Anapass ANA38407; the module is Samsung AMSA46AS02. Structural pattern from
 * panel-novatek-nt37801.c (command-mode + DSC + driver-side
 * mipi_dsi_picture_parameter_set) and the Samsung level-key idiom from
 * panel-samsung-s6e3ha8.c. The power-on sequence is transcribed from this
 * device's own downstream Panel Data File (GTS9U_ANA38407_AMSA46AS02); the
 * sub-table names below (VBP_SETTING etc.) cross-reference that source.
 *
 * Copyright (c) 2026, ubuntu-tab project.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

/*
 * SESS-141 brightness-sweep probe: WRDISBV override (DDIC display brightness,
 * 0..2047). Default 0x07ff = bl_lvl 255 (panel-on max, the original behaviour).
 * Lowering it engages the DDIC's low-brightness analog dimming (AID/ELVSS) —
 * the hypothesis being that OneUI's low-brightness dark-region tuning is what
 * suppresses the row-154 dark-seam flicker that mainline (fixed max) shows.
 * Set at module load:  insmod panel.ko bl_override=320
 */
static unsigned short bl_override = 0x07ff;
module_param(bl_override, ushort, 0644);
MODULE_PARM_DESC(bl_override, "WRDISBV display brightness 0..2047 (default 2047=max)");

/*
 * SESS-142 TCON pre-emphasis probe. The device's own downstream source
 * (GTS9U_ANA38407_AMSA46AS02.dat: TCON_PE_ON) sets pre-emphasis on six TCON
 * channels (registers 0x6A..0x6F) AFTER panel-on, driven from userspace -- so
 * a bare kernel (TWRP) never emits it, yet the downstream stack flickers under
 * TWRP and is clean under OneUI. Pre-emphasis conditions the signal edge at the
 * source-driver/slice-channel boundaries; a sess-142 candidate for the row-154
 * dark-seam-flicker suppressor (mechanism uncertain -- see SESS142-INTERIM
 * notes). tcon_pe < 0 (default) = do NOT send the block (original behaviour,
 * the flickering control); tcon_pe >= 0 = send the block with that value as the
 * C1 pre-emphasis byte (device's own ON value = 0x04; .dat OFF value = 0x00).
 * Set at module load:  insmod panel.ko tcon_pe=4
 */
static short tcon_pe = -1;
module_param(tcon_pe, short, 0644);
MODULE_PARM_DESC(tcon_pe, "TCON pre-emphasis level on regs 0x6A-0x6F (<0=off/don't-send, 0x04=device ON)");

/*
 * SESS-142 GLUT wrong-table probe. The panel-on GLUT_SETTING below transcribes
 * the device's downstream glut_offset_60HS[bl255] table (non-zero). But the
 * downstream selects the GLUT table by VRR mode base (ss_get_vrr_mode_base):
 * for the 60PHS regime this panel runs (120HS scan + LFD-to-60, our VRR_SETTING
 * below), vrr_base = VRR_120HS -> the 120HS GLUT table, which OneUI emits as
 * ALL ZEROS at every brightness (live capture on rooted OneUI, sess-142). I.e.
 * mainline sends the 60HS gamma offset to a 120HS-base-driven panel -- the wrong
 * table. glut_zero=1 sends an all-zero GLUT (matching OneUI's live 120HS-base
 * output) as a single-variable capture-and-match test of the row-154 flicker.
 * Set at module load:  insmod panel.ko glut_zero=1
 */
static bool glut_zero;
module_param(glut_zero, bool, 0644);
MODULE_PARM_DESC(glut_zero, "send all-zero GLUT (EB/EC) to match OneUI's 120HS-base output (default 0 = original 60HS table)");

/*
 * SESS-143 mDNIe probe. OneUI programs the DDIC's mDNIe image-quality engine
 * (Samsung "Mobile DNIe": ASCR adaptive contrast/colour + AOLCE local-contrast
 * + a tone LUT) at panel-on as a PERSISTENT state; mainline and TWRP send NONE
 * of it. The sess-142 discriminator localized the row-154 dark-seam-flicker
 * suppressor to exactly such a OneUI boot-persistent DDIC command that mainline
 * omits, and a global low-gray contrast/tone transform is the coherent
 * threshold-raising mechanism (the per-frame DSC-slice-row transient is real in
 * both stacks; mDNIe would push it sub-visible). FFC (the prior #1 lead) was
 * refuted: it is an EMPTY command set on this wifi-only panel. The three writes
 * below (registers 0xDF/0xDE/0xDD) are the device's own UI_DYNAMIC scenario,
 * byte-verified against both GTS9U_ANA38407_AMSA46AS02_mdnie.h AND the live
 * rooted-OneUI capture (the runtime value 0xDD[17]=0x00 trans_on, not the
 * static table's 0xFF, is used -- it is what actually drove the clean panel).
 * mdnie=1 sends the UI_DYNAMIC ("Vivid") block; mdnie=2 sends the GRAYSCALE
 * scenario instead -- a DECISIVE engagement probe: if the mDNIe engine actually
 * engages, grayscale desaturates ALL colour to black-and-white (binary, no
 * subjective saturation judgment). Vivid (1) vs ordinary colour is ambiguous on
 * dark content; grayscale (2) is unmissable. Both share the identical DE LUT and
 * the DD enable; only DF's SCR colour matrix differs (grayscale = Rec.601 luma
 * weights). default 0 = do NOT send (original behaviour, the flickering control).
 * Set at module load:  insmod panel.ko mdnie=1   (or mdnie=2 for the gray probe)
 */
static int mdnie;
module_param(mdnie, int, 0644);
MODULE_PARM_DESC(mdnie, "send OneUI mDNIe at panel-on: 0=off (default), 1=UI_DYNAMIC/Vivid, 2=GRAYSCALE (engagement probe)");

/*
 * acl -- ACL (Automatic Current Limiting) level written to WRACL (0x55) at
 * panel-on. default 0 = ACL OFF (original behaviour, the flickering control).
 * The sess-156 DDIC-internal capture (OneUI-clean vs bare-TWRP read_mtp diff)
 * found RDACL (0x56) reads 0x02 on the no-flicker OneUI stack but 0x01 (bare
 * TWRP) / 0x00 (mainline, ACL-off) on the two flickering stacks -- ACL is per-
 * frame and content-adaptive (modulates emission current/ELVSS), so ACL>=2 is
 * the candidate seam-flicker suppressor. Set at module load:  insmod panel.ko
 * acl=2   (sweep acl=1/2/3 to bracket the level).
 */
static int acl;
module_param(acl, int, 0644);
MODULE_PARM_DESC(acl, "ACL level at panel-on via WRACL 0x55: 0=off (default/flickering control), 1/2/3=ACL levels (>=2 = the candidate suppressor)");

struct ana38407 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
};

/*
 * The panel power on this board is a single GPIO-gated rail (panel_ldo_en,
 * &tlmm 187), which enables the discrete panel-power IC that produces the
 * panel's VCI/VDDIO/VSP/VSN. The downstream device tree declares only this
 * one software-controlled supply for the panel.
 */
static const struct regulator_bulk_data ana38407_supplies[] = {
	{ .supply = "power" },
};

static inline struct ana38407 *to_ana38407(struct drm_panel *panel)
{
	return container_of(panel, struct ana38407, panel);
}

/* Samsung register-access level keys. */
#define ana38407_unlock_lvl0(ctx) \
	mipi_dsi_dcs_write_seq_multi((ctx), 0xf0, 0x5a, 0x5a)
#define ana38407_lock_lvl0(ctx) \
	mipi_dsi_dcs_write_seq_multi((ctx), 0xf0, 0xa5, 0xa5)
#define ana38407_unlock_lvl1(ctx) \
	mipi_dsi_dcs_write_seq_multi((ctx), 0xf1, 0x5a, 0x5a)
#define ana38407_lock_lvl1(ctx) \
	mipi_dsi_dcs_write_seq_multi((ctx), 0xf1, 0xa5, 0xa5)

static void ana38407_reset(struct ana38407 *ctx)
{
	/* reset-gpios is active-low: value 1 = asserted (line low). */
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static int ana38407_on(struct ana38407 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* VBP_SETTING_FOR_SDC_IP */
	ana38407_unlock_lvl0(&dsi_ctx);
	ana38407_unlock_lvl1(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x0f, 0x00, 0x00, 0x00, 0x09, 0xfd, 0x81);
	ana38407_lock_lvl0(&dsi_ctx);
	ana38407_lock_lvl1(&dsi_ctx);

	/* DISPLAY_ON_DELAY_SETTING */
	ana38407_unlock_lvl0(&dsi_ctx);
	ana38407_unlock_lvl1(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x0f, 0x00, 0x00, 0x00, 0x01, 0x04, 0x81);
	ana38407_lock_lvl0(&dsi_ctx);
	ana38407_lock_lvl1(&dsi_ctx);

	/* Sleep-out, then the mandatory >=120 ms settle. */
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	/* MX_IP_ENABLE */
	ana38407_unlock_lvl0(&dsi_ctx);
	ana38407_unlock_lvl1(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x0f, 0x00, 0x00, 0x00, 0x09, 0xb2, 0x81);
	ana38407_lock_lvl0(&dsi_ctx);
	ana38407_lock_lvl1(&dsi_ctx);

	/* TCON_INTR_SETTING (0xc1 0x02 = TE active-low, per the device source). */
	ana38407_unlock_lvl0(&dsi_ctx);
	ana38407_unlock_lvl1(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x0f, 0x00, 0x00, 0x00, 0x14, 0x46, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x0f, 0x00, 0x00, 0x00, 0x08, 0xcf, 0x81);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
				     0x0f, 0x00, 0x00, 0x00, 0x09, 0xcd, 0x81);
	ana38407_lock_lvl0(&dsi_ctx);
	ana38407_lock_lvl1(&dsi_ctx);

	/* UPIM_SSCG_SETTING is rev-BtoB only; this module is rev A. */

	/* TE_ON (hardware TE, VBLANK). */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	ana38407_lock_lvl0(&dsi_ctx);

	/*
	 * TSP_SYNC_SETTING (rev C/D). This panel is rev D (lcd_id 0x800004 ->
	 * id 0x04, confirmed live); the device's own downstream source applies
	 * the "CtoZ" TSP_SYNC_SETTING, which writes only the first two 0xB9
	 * fields and OMITS the rev-A 0xB0 0x10 0xB9 / 0xB9 0x88 x4 write. The
	 * 0xB9[0x10] frame-control field is instead set in VRR_SETTING below
	 * (device-ground-truth value 0x80 0x00 0x00 0x00, not the rev-A 0x88 x4).
	 */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0e, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x15);
	ana38407_lock_lvl0(&dsi_ctx);

	/*
	 * DSC_SETTING: enable compression, then send the PPS (the device order;
	 * standard DSI 0x07 + 0x0A transport). The msm host fills ctx->dsc
	 * before prepare() (prepare_prev_first), so the packed PPS is complete.
	 * Bring-up: dump it to verify it matches the device's 88-byte PPS.
	 */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	print_hex_dump(KERN_INFO, "ana38407 pps: ", DUMP_PREFIX_OFFSET, 16, 1,
		       &pps, sizeof(pps), false);
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	ana38407_lock_lvl0(&dsi_ctx);

	/* DIA_SETTING (DIA on). */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0x02);

	/*
	 * BRIGHTNESS_SETTING (factory panel-on gamma/GLUT + ACL + ELVSS +
	 * smooth-dimming), transcribed from the device's own downstream source
	 * GTS9U_ANA38407_AMSA46AS02.dat. Order: GLUT -> ACL -> ELVSS -> DIMMING.
	 * GLUT is per-brightness (downstream update_glut, glut_offset_60hs[bl_lvl]);
	 * we send the bl_lvl=255 row (max-normal, the downstream's own default
	 * bl_level), 60HS table. No byte here is MTP/OTP-derived: GLUT is parsed
	 * verbatim from the .dat, TSET is ambient temperature, ACL is OFF at on.
	 */

	/* GLUT_SETTING, bl_lvl 255 row. */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x91, 0xeb);
	{
		/* EB/EC GLUT payload (60HS bl255 table). glut_zero zeroes the
		 * data (opcode kept), matching OneUI's live 120HS-base output. */
		u8 eb[] = { 0xeb,
			0x3f, 0x00, 0x3f, 0x88, 0x3f, 0xb8, 0x3f, 0xcc, 0x3f, 0xdc, 0x3f, 0xe6,
			0x3f, 0xf0, 0x3f, 0xf3, 0x3f, 0xf8, 0x3f, 0xfa, 0x3f, 0xfe, 0x3f, 0xff,
			0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x06, 0x00, 0x08, 0x00, 0x08,
			0x00, 0x0a, 0x00, 0x0c, 0x00, 0x0d, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x13,
			0x00, 0x11, 0x00, 0x15, 0x00, 0x15, 0x00, 0x15, 0x00, 0x16, 0x00, 0x1a,
			0x00, 0x18, 0x00, 0x1a, 0x3e, 0xca, 0x3f, 0x61, 0x3f, 0x9f, 0x3f, 0xba,
			0x3f, 0xcd, 0x3f, 0xda, 0x3f, 0xe6, 0x3f, 0xeb, 0x3f, 0xf1, 0x3f, 0xf4,
			0x3f, 0xf7, 0x3f, 0xfb, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x02, 0x00, 0x04, 0x00, 0x07, 0x00, 0x09, 0x00, 0x0a, 0x00, 0x0c,
			0x00, 0x0c };
		u8 ec[] = { 0xec,
			0x00, 0x0e, 0x00, 0x10, 0x00, 0x10, 0x00, 0x12, 0x00, 0x13, 0x00, 0x15,
			0x00, 0x15, 0x00, 0x19, 0x00, 0x17, 0x3e, 0x75, 0x3f, 0x2f, 0x3f, 0x85,
			0x3f, 0xad, 0x3f, 0xc8, 0x3f, 0xd7, 0x3f, 0xe4, 0x3f, 0xec, 0x3f, 0xf2,
			0x3f, 0xf7, 0x3f, 0xfa, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x03, 0x00, 0x05,
			0x00, 0x06, 0x00, 0x09, 0x00, 0x0a, 0x00, 0x0d, 0x00, 0x0e, 0x00, 0x10,
			0x00, 0x11, 0x00, 0x12, 0x00, 0x13, 0x00, 0x14, 0x00, 0x16, 0x00, 0x19,
			0x00, 0x18, 0x00, 0x18, 0x00, 0x1a, 0x00, 0x20, 0x00, 0x1c };

		if (glut_zero) {
			memset(eb + 1, 0, sizeof(eb) - 1);
			memset(ec + 1, 0, sizeof(ec) - 1);
		}
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, eb, sizeof(eb));
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, ec, sizeof(ec));
	}
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xeb, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x26, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x93, 0x20);
	ana38407_lock_lvl0(&dsi_ctx);

	/* ACL (WRACL 0x55 = `acl` module_param, default 0 = OFF; + static C9 tuning). */
	ana38407_unlock_lvl0(&dsi_ctx);
	{
		u8 wracl[2] = { 0x55, (u8)acl };

		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, wracl, sizeof(wracl));
	}
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x5f, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9, 0x00, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x51, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9, 0x40, 0xff, 0x26);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x62, 0xc9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc9, 0x3e, 0x42);
	ana38407_lock_lvl0(&dsi_ctx);

	/* ELVSS_TEMP_COMPENSATION (TSET=+25C reference + static C0). */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc1, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0,
		0x0f, 0x00, 0x00, 0x00, 0x06, 0xb1, 0x81);
	ana38407_lock_lvl0(&dsi_ctx);

	/* BRIGHTNESS_DIMMING_SETTING (smooth-dim ON + WRDISBV). bl_override
	 * (default 0x07ff = bl255 max) is a sess-141 sweep knob — see the
	 * module_param note at the top. */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x28);
	{
		u8 wrdisbv[3] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				  (bl_override >> 8) & 0xff, bl_override & 0xff };
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, wrdisbv, sizeof(wrdisbv));
	}
	ana38407_lock_lvl0(&dsi_ctx);

	/*
	 * HBM_FlatZ_SETTING (IRC / "Flat Gamma Z"), transcribed byte-exact from
	 * this device's own downstream data file
	 * (GTS9U_ANA38407_AMSA46AS02.dat:140-141). The .dat applies it inside
	 * BRIGHTNESS_DIMMING_SETTING under "IF REVISION BtoZ" (.dat:106-107);
	 * our panel is rev D (id 0x04 -> index 3, within B..Z), so stock sends
	 * this on every normal panel-on. Our driver omitted it; it is the sole
	 * register-level DCS command in the stock normal power-on path that we
	 * were not sending. IRC is a DDIC low-gray gamma/luminance correction.
	 */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0a, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0,
		0x3c, 0xfd, 0xff, 0x15, 0x00, 0x00, 0x66, 0xcc, 0x00, 0xff, 0x12);
	ana38407_lock_lvl0(&dsi_ctx);

	/* SP_SETTING */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x02);
	ana38407_lock_lvl0(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 50);

	/*
	 * VRR_SETTING (rev D), transcribed from this device's own stock capture
	 * (sess-133 live ss_print_cmd_desc): stock runs 120HS scan + Low-
	 * Frequency-Driving at every refresh rate, programming 0x60=0x00 (120HS
	 * scan select), 0xDD[0x13]=0x01 (LFD divisor = 60PHS: stock drives the
	 * 120HS-scan DDIC at 60 fps via low-frequency-driving -- observed in the
	 * forced-60 capture -- coherent with our fixed-60 command-mode delivery),
	 * and 0xB9[0x10]=0x80 0x00 0x00 0x00 (the rev-D/"CtoZ" 120HS frame-control
	 * value -- NOT the rev-A 0x88 x4 nor the 60HS 0xAA x4). The rev-A/B
	 * 0xF7 0x07 latch is not sent on rev D.
	 */
	ana38407_unlock_lvl0(&dsi_ctx);
	ana38407_unlock_lvl1(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x13, 0xdd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdd, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x10, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x80, 0x00, 0x00, 0x00);
	ana38407_lock_lvl0(&dsi_ctx);
	ana38407_lock_lvl1(&dsi_ctx);

	/*
	 * TCON_PE_ON (sess-142 probe; off by default). Transcribed byte-exact
	 * from GTS9U_ANA38407_AMSA46AS02.dat: for each TCON channel 0x6A..0x6F,
	 * the indirect-register-write envelope sets the pre-emphasis byte (0xC1)
	 * via offset 0xB0 0x03 and the 0xC0 address payload. See the tcon_pe
	 * module_param note above.
	 */
	if (tcon_pe >= 0) {
		u8 pe = tcon_pe & 0xff;
		int reg;

		ana38407_unlock_lvl0(&dsi_ctx);
		ana38407_unlock_lvl1(&dsi_ctx);
		for (reg = 0x6a; reg <= 0x6f; reg++) {
			u8 c1[] = { 0xc1, pe };
			u8 b0[] = { 0xb0, 0x03 };
			u8 c0[] = { 0xc0, 0x0f, 0x00, 0x00, 0x00,
				    0x03, (u8)reg, 0x81 };

			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, c1, sizeof(c1));
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, b0, sizeof(b0));
			mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, c0, sizeof(c0));
		}
		ana38407_lock_lvl1(&dsi_ctx);
		ana38407_lock_lvl0(&dsi_ctx);
	}

	/*
	 * mDNIe (sess-143 probe; off by default). OneUI's UI_DYNAMIC scenario,
	 * registers 0xDF (ASCR/skin) / 0xDE (tone LUT) / 0xDD (mDNIe master enable
	 * + AOLCE/gamut). Bytes byte-verified against the device's own
	 * GTS9U_ANA38407_AMSA46AS02_mdnie.h AND the live rooted-OneUI capture;
	 * 0xDD[17]=0x00 (trans_on) is the runtime-captured value, not the static
	 * table's 0xFF. Sent last, matching OneUI's post-brightness order, with its
	 * exact F0+F1-unlock / F0-only-lock asymmetry. See the mdnie param note.
	 */
	if (mdnie) {
		static const u8 mdnie_df[] = {
			0xdf, 0x11, 0x6a, 0x9a, 0x25, 0x1a, 0x16, 0x2a, 0x00, 0x37, 0x5a, 0x00,
			0x4e, 0xc5, 0x00, 0x5d, 0x17, 0x00, 0x30, 0xc3, 0xe3, 0x00, 0x00, 0xf2,
			0xef, 0x00, 0xf0, 0x24, 0xf0, 0xff, 0xfc, 0xf6, 0x63, 0xe8, 0xfd, 0x16,
			0xf7, 0x00, 0xef, 0x51, 0x0f, 0xfe, 0xec, 0x12, 0xfc, 0x3a, 0xfc, 0x09,
			0x15, 0xe9, 0xff, 0x00, 0xfc, 0x00, 0xf6, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x80, 0x01, 0x00, 0x01, 0x80, 0x02, 0x00, 0x02, 0x80, 0x03, 0x00, 0x03,
			0x80, 0x04, 0x00, 0x04, 0x80, 0x05, 0x00, 0x05, 0x80, 0x06, 0x00, 0x06,
			0x80, 0x07, 0x00, 0x07, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01,
			0x00, 0x01, 0x80, 0x02, 0x00, 0x02, 0x80, 0x03, 0x00, 0x03, 0x80, 0x04,
			0x00, 0x04, 0x80, 0x05, 0x00, 0x05, 0x80, 0x06, 0x00, 0x06, 0x80, 0x07,
			0x00, 0x07, 0x80, 0x08, 0x00
		};
		static const u8 mdnie_de[] = {
			0xde, 0x00, 0x40, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x30,
			0x01, 0x00, 0x70, 0x01, 0x34, 0x01, 0x40, 0x66, 0x03, 0x01, 0x11, 0x00,
			0x20, 0x00, 0x08, 0x08, 0x00, 0x0a, 0x0a, 0x06, 0xae, 0x00, 0x00, 0x10,
			0x20, 0x40, 0x60, 0x00, 0x40, 0x00, 0x50, 0x00, 0x60, 0x00, 0x70, 0x00,
			0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x20, 0x40, 0x60, 0x01, 0x40,
			0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x40, 0x03, 0xb6, 0x23, 0x24, 0x00, 0x72, 0x01, 0x00,
			0x0d, 0x00, 0x04, 0x07, 0x40, 0x00, 0x6e, 0x82, 0xe6, 0xff, 0x00, 0xff,
			0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x14, 0x00, 0x01, 0x00, 0x01, 0x00,
			0x10, 0x00, 0x00, 0x28, 0x3c, 0x04, 0x0f, 0x01, 0x00, 0x00, 0x08, 0x10,
			0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70,
			0x78, 0x80, 0x88, 0x90, 0x98, 0xa0, 0xa8, 0xb0, 0xb8, 0xc0, 0xc8, 0xd0,
			0xd8, 0xe0, 0xe8, 0xf0, 0xf8, 0x01, 0x00, 0x00, 0x08, 0x10, 0x18, 0x20,
			0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68, 0x70, 0x78, 0x80,
			0x88, 0x90, 0x98, 0xa0, 0xa8, 0xb0, 0xb8, 0xc0, 0xc8, 0xd0, 0xd8, 0xe0,
			0xe8, 0xf0, 0xf8, 0x01, 0x00
		};
		static const u8 mdnie_dd[] = {
			0xdd, 0x01, 0x00, 0xf0, 0x07, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01
		};
		/*
		 * GRAYSCALE scenario (mdnie=2 engagement probe): identical DE LUT + DD
		 * enable; DF carries the Rec.601 luma SCR matrix that desaturates all
		 * colour. If mDNIe engages, the screen goes black-and-white.
		 */
		static const u8 mdnie_gray_df[] = {
			0xdf, 0x01, 0x6a, 0x9a, 0x25, 0x1a, 0x16, 0x2a, 0x00, 0x37, 0x5a, 0x00,
			0x4e, 0xc5, 0x00, 0x5d, 0x17, 0x00, 0x30, 0xc3, 0x4c, 0x4c, 0x4c, 0xe2,
			0xe2, 0xe2, 0x69, 0x69, 0x69, 0xff, 0xff, 0xff, 0xb3, 0x4c, 0xb3, 0x4c,
			0xb3, 0x4c, 0x69, 0x96, 0x69, 0x96, 0x69, 0x96, 0xe2, 0x1d, 0xe2, 0x1d,
			0xe2, 0x1d, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x80, 0x01, 0x00, 0x01, 0x80, 0x02, 0x00, 0x02, 0x80, 0x03, 0x00, 0x03,
			0x80, 0x04, 0x00, 0x04, 0x80, 0x05, 0x00, 0x05, 0x80, 0x06, 0x00, 0x06,
			0x80, 0x07, 0x00, 0x07, 0x80, 0x08, 0x00, 0x00, 0x00, 0x00, 0x80, 0x01,
			0x00, 0x01, 0x80, 0x02, 0x00, 0x02, 0x80, 0x03, 0x00, 0x03, 0x80, 0x04,
			0x00, 0x04, 0x80, 0x05, 0x00, 0x05, 0x80, 0x06, 0x00, 0x06, 0x80, 0x07,
			0x00, 0x07, 0x80, 0x08, 0x00
		};
		static const u8 mdnie_gray_dd[] = {
			0xdd, 0x01, 0x00, 0xf0, 0x07, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x04, 0x01
		};
		const u8 *df = (mdnie == 2) ? mdnie_gray_df : mdnie_df;
		const u8 *dd = (mdnie == 2) ? mdnie_gray_dd : mdnie_dd;

		ana38407_unlock_lvl0(&dsi_ctx);
		ana38407_unlock_lvl1(&dsi_ctx);
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, df, sizeof(mdnie_df));
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, mdnie_de, sizeof(mdnie_de));
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, dd, sizeof(mdnie_dd));
		ana38407_lock_lvl0(&dsi_ctx);	/* OneUI locks F0 only (asymmetric) */
	}

	return dsi_ctx.accum_err;
}

static int ana38407_prepare(struct drm_panel *panel)
{
	struct ana38407 *ctx = to_ana38407(panel);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ana38407_supplies),
				    ctx->supplies);
	if (ret < 0)
		return ret;

	ana38407_reset(ctx);

	ret = ana38407_on(ctx);
	if (ret < 0) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(ana38407_supplies),
				       ctx->supplies);
		return ret;
	}

	return 0;
}

static int ana38407_enable(struct drm_panel *panel)
{
	struct ana38407 *ctx = to_ana38407(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* POWER_ON_POST_SETTING: display-on behind the level-2 key. */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	ana38407_lock_lvl0(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int ana38407_disable(struct drm_panel *panel)
{
	struct ana38407 *ctx = to_ana38407(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int ana38407_unprepare(struct drm_panel *panel)
{
	struct ana38407 *ctx = to_ana38407(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ana38407_supplies), ctx->supplies);

	return dsi_ctx.accum_err;
}

/*
 * V0 fixed 60 Hz mode (the downstream wqxga60hs timing). Native 2960x1848
 * landscape; physical 313x196 mm.
 *
 * Horizontal blanking is a DSI TRANSPORT-RATE knob on this panel, not panel
 * timing. In command mode the host never programs the porches into hardware --
 * dsi_timing_setup() writes only STREAM0_CTRL (word count from the DSC
 * slice_chunk_size) and STREAM0_TOTAL (hdisplay/vdisplay); REG_DSI_ACTIVE_H,
 * REG_DSI_TOTAL and REG_DSI_ACTIVE_HSYNC are not written at all. The blanking
 * survives only inside the pixel-clock computation, which msm documents as
 * "the overhead to the image data transfer".
 *
 * Working the algebra through dsi_adjust_pclk_for_compression(), htotal cancels
 * and the link rate reduces to a function of the blanking alone:
 *
 *     pclk = vtotal * 60 * (hblank + hdisplay * bpp / (bpc * 3))
 *          = 2368 * 60 * (hblank + 987)
 *
 * so hblank sets how fast one frame is pushed into the DDIC's GRAM, at a fixed
 * 60 Hz frame cadence and with the transmitted payload unchanged.
 *
 * This knob is EMPIRICALLY LOAD-BEARING: the display artifact's row position
 * tracks it. Measured, single-variable, with everything else held identical:
 *
 *     hblank 767  ->  bit clk 1495.2 Mbps/lane  ->  artifact at panel row ~146
 *     hblank 200  ->  bit clk 1011.9 Mbps/lane  ->  artifact at panel row 66.3
 *
 * (43 firings, sd 1.12; an ~80-row shift against a 7.6-row instrument floor.)
 * The occurrence rate did not change -- 17.9% of frames both before and after --
 * so the link rate sets WHERE the artifact lands, not WHETHER it happens.
 *
 * A third point at hblank 1644 (bit clk 2243 Mbps) put the row back at 144.45,
 * i.e. essentially where it started -- so the position is NOT a monotonic
 * function of the link rate, and every continuous model of it is wrong:
 *
 *     bit clk 1495.2 -> row ~146      bit clk 1011.9 -> row 66.29 +- 1.12
 *     bit clk 2242.9 -> row 144.45 +- 0.50
 *
 * 144.45 - 66.29 = 78.16, and slice_height is 77: the band appears to move by
 * exactly ONE SLICE ROW and back, which would make the position QUANTISED to
 * the DSC slice grid rather than continuous in the rate.
 *
 * hblank 400 answered that: bit clock 1182 Mbps, deliberately BETWEEN the two
 * rates that gave different rows -- and the band landed on 144, NOT between.
 * The position is QUANTISED. The link rate SELECTS which slice boundary the
 * artifact occupies; it does not position it continuously:
 *
 *     bit clk  846 -> ?          bit clk 1011.9 -> row 66.3   (index N=1)
 *     bit clk 1182 -> row 144    bit clk 1495.2 -> row ~146   (index N=2)
 *     bit clk 2243 -> row 144.45                              (index N=2)
 *
 * The discrete positions are ~78 rows apart, one slice_height. This also
 * explains the old slice_height result: 77->66 moved the band 154->132, both
 * 2*slice_height -- N stayed at 2 while the grid rescaled. Grid spacing and
 * grid index are two separate knobs onto the same quantity.
 *
 * DO NOT slow the link below ~1 Gbps/lane. hblank 6 (846.5 Mbps, 78% transfer)
 * made the display unusably slow -- that starves real bandwidth, which is NOT
 * what the vendor does. Stock runs a FAST link and paces the DPU's output on
 * top of it; qcom,mdss-mdp-transfer-time-us is an MDP pacing target, not a link
 * rate. Lengthening the transfer by slowing the link is the wrong mechanism.
 *
 * This value is THE DEVICE'S OWN. The stock panel node declares
 * qcom,mdss-dsi-panel-clockrate = <0x5ad66500> = 1,524,000,000 Hz, identical
 * across all five of its modes. hblank 801 is the closest an integer blanking
 * value reaches it:
 *
 *     pclk = 2368 * 60 * (801 + 987) = 254,039,040 Hz
 *     bit clock = 1,524,234,240 Hz   -- 0.0154% above the declared value
 *
 * Frame transfer ~7.2 ms = ~43% of the period, matching the vendor's 7533 us
 * 60 Hz mode. Everything derived here previously ran at 1,495,249,365 Hz, which
 * is 2% low: a value that fell out of our own htotal arithmetic rather than one
 * the device specifies.
 */
static const struct drm_display_mode ana38407_mode = {
	.clock = (2960 + 267 + 267 + 267) * (1848 + 127 + 256 + 137) * 60 / 1000,
	.hdisplay = 2960,
	.hsync_start = 2960 + 267,
	.hsync_end = 2960 + 267 + 267,
	.htotal = 2960 + 267 + 267 + 267,
	.vdisplay = 1848,
	.vsync_start = 1848 + 127,
	.vsync_end = 1848 + 127 + 256,
	.vtotal = 1848 + 127 + 256 + 137,
	.width_mm = 313,
	.height_mm = 196,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int ana38407_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ana38407_mode);
}

static const struct drm_panel_funcs ana38407_panel_funcs = {
	.prepare = ana38407_prepare,
	.enable = ana38407_enable,
	.disable = ana38407_disable,
	.unprepare = ana38407_unprepare,
	.get_modes = ana38407_get_modes,
};

static int ana38407_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ana38407 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ana38407, panel,
				   &ana38407_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(ana38407_supplies),
					    ana38407_supplies, &ctx->supplies);
	if (ret < 0)
		return ret;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;

	drm_panel_add(&ctx->panel);

	/* This panel only operates with DSC; configure it unconditionally. */
	dsi->dsc = &ctx->dsc;
	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.slice_height = 77;
	ctx->dsc.slice_width = 1480;
	ctx->dsc.slice_count = 2960 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits => 8.0 bpp */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void ana38407_remove(struct mipi_dsi_device *dsi)
{
	struct ana38407 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ana38407_of_match[] = {
	{ .compatible = "samsung,ana38407-amsa46as02" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ana38407_of_match);

static struct mipi_dsi_driver ana38407_driver = {
	.probe = ana38407_probe,
	.remove = ana38407_remove,
	.driver = {
		.name = "panel-samsung-ana38407-amsa46as02",
		.of_match_table = ana38407_of_match,
	},
};
module_mipi_dsi_driver(ana38407_driver);

MODULE_DESCRIPTION("DRM driver for the Samsung AMSA46AS02 (Anapass ANA38407) panel");
MODULE_LICENSE("GPL");
