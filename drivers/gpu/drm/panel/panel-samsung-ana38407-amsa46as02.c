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

	/* TSP_SYNC_ON (rev A). */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0b, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0xcc);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x0e, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x10, 0xb9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9, 0x88, 0x88, 0x88, 0x88);
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
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xeb,
		0x3f, 0x00, 0x3f, 0x88, 0x3f, 0xb8, 0x3f, 0xcc, 0x3f, 0xdc, 0x3f, 0xe6,
		0x3f, 0xf0, 0x3f, 0xf3, 0x3f, 0xf8, 0x3f, 0xfa, 0x3f, 0xfe, 0x3f, 0xff,
		0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x06, 0x00, 0x08, 0x00, 0x08,
		0x00, 0x0a, 0x00, 0x0c, 0x00, 0x0d, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x13,
		0x00, 0x11, 0x00, 0x15, 0x00, 0x15, 0x00, 0x15, 0x00, 0x16, 0x00, 0x1a,
		0x00, 0x18, 0x00, 0x1a, 0x3e, 0xca, 0x3f, 0x61, 0x3f, 0x9f, 0x3f, 0xba,
		0x3f, 0xcd, 0x3f, 0xda, 0x3f, 0xe6, 0x3f, 0xeb, 0x3f, 0xf1, 0x3f, 0xf4,
		0x3f, 0xf7, 0x3f, 0xfb, 0x3f, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x02, 0x00, 0x04, 0x00, 0x07, 0x00, 0x09, 0x00, 0x0a, 0x00, 0x0c,
		0x00, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xec,
		0x00, 0x0e, 0x00, 0x10, 0x00, 0x10, 0x00, 0x12, 0x00, 0x13, 0x00, 0x15,
		0x00, 0x15, 0x00, 0x19, 0x00, 0x17, 0x3e, 0x75, 0x3f, 0x2f, 0x3f, 0x85,
		0x3f, 0xad, 0x3f, 0xc8, 0x3f, 0xd7, 0x3f, 0xe4, 0x3f, 0xec, 0x3f, 0xf2,
		0x3f, 0xf7, 0x3f, 0xfa, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x03, 0x00, 0x05,
		0x00, 0x06, 0x00, 0x09, 0x00, 0x0a, 0x00, 0x0d, 0x00, 0x0e, 0x00, 0x10,
		0x00, 0x11, 0x00, 0x12, 0x00, 0x13, 0x00, 0x14, 0x00, 0x16, 0x00, 0x19,
		0x00, 0x18, 0x00, 0x18, 0x00, 0x1a, 0x00, 0x20, 0x00, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xeb, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x26, 0x93);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x93, 0x20);
	ana38407_lock_lvl0(&dsi_ctx);

	/* ACL (ACL OFF + static C9 tuning). */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x55, 0x00);
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

	/* BRIGHTNESS_DIMMING_SETTING (smooth-dim ON + WRDISBV = bl255 = 0x07ff). */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x28);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				     0x07, 0xff);
	ana38407_lock_lvl0(&dsi_ctx);

	/* SP_SETTING */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x02);
	ana38407_lock_lvl0(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 50);

	/* VRR_SETTING deferred: fixed 60 Hz uses the DDIC default rate. */

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
 */
static const struct drm_display_mode ana38407_mode = {
	.clock = (2960 + 256 + 256 + 255) * (1848 + 127 + 256 + 137) * 60 / 1000,
	.hdisplay = 2960,
	.hsync_start = 2960 + 256,
	.hsync_end = 2960 + 256 + 256,
	.htotal = 2960 + 256 + 256 + 255,
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
