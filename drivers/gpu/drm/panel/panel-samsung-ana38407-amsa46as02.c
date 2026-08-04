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

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

/*
 * Platform brightness level (0..255) -> WRDISBV (the DDIC's own display
 * brightness register, 0..2047), transcribed column-for-column from this
 * device's own downstream Panel Data File: the NORMAL_CANDELA_MAP table in
 * GTS9U_ANA38407_AMSA46AS02.dat, whose header reads
 *
 *	<Platform_Level> <WRDISBV> <Candela>
 *
 * Level 0 is 10 -> 2 cd/m2 (the panel's dimmest, NOT off) and level 255 is
 * 2047 -> 420 cd/m2. The curve is the vendor's own: its steps are finer at the
 * bottom (level 127 lands at 171 cd/m2, 41% of peak, where a luminance-linear
 * ramp would sit at 50%), which is why the backlight below advertises a
 * non-linear scale.
 *
 * Levels 256 and up continue into high-brightness mode; see
 * ana38407_wrdisbv_hbm[] below.
 */
static const u16 ana38407_wrdisbv[256] = {
	  10,   12,   14,   16,   19,   22,   26,   29,
	  33,   36,   40,   44,   48,   53,   57,   61,
	  66,   70,   75,   80,   84,   89,   94,   99,
	 104,  110,  115,  120,  125,  131,  136,  142,
	 147,  153,  158,  164,  170,  176,  182,  187,
	 193,  199,  205,  211,  218,  224,  230,  236,
	 242,  249,  255,  261,  268,  274,  281,  287,
	 294,  301,  307,  314,  321,  327,  334,  341,
	 348,  355,  362,  369,  376,  383,  390,  397,
	 404,  411,  418,  425,  432,  440,  447,  454,
	 462,  469,  476,  484,  491,  499,  506,  514,
	 521,  529,  536,  544,  551,  559,  567,  575,
	 582,  590,  598,  606,  613,  621,  629,  637,
	 645,  653,  661,  669,  677,  685,  693,  701,
	 709,  717,  726,  734,  742,  750,  758,  767,
	 775,  783,  791,  800,  808,  817,  825,  833,
	 842,  850,  859,  867,  876,  884,  893,  901,
	 910,  919,  927,  936,  944,  953,  962,  971,
	 979,  988,  997, 1006, 1014, 1023, 1032, 1041,
	1050, 1059, 1068, 1077, 1086, 1095, 1104, 1113,
	1122, 1131, 1140, 1149, 1158, 1167, 1176, 1185,
	1194, 1203, 1213, 1222, 1231, 1240, 1250, 1259,
	1268, 1277, 1287, 1296, 1305, 1315, 1324, 1334,
	1343, 1352, 1362, 1371, 1381, 1390, 1400, 1409,
	1419, 1428, 1438, 1447, 1457, 1467, 1476, 1486,
	1495, 1505, 1515, 1524, 1534, 1544, 1554, 1563,
	1573, 1583, 1593, 1603, 1612, 1622, 1632, 1642,
	1652, 1662, 1672, 1681, 1691, 1701, 1711, 1721,
	1731, 1741, 1751, 1761, 1771, 1781, 1791, 1801,
	1812, 1822, 1832, 1842, 1852, 1862, 1872, 1882,
	1893, 1903, 1913, 1923, 1934, 1944, 1954, 1964,
	1975, 1985, 1995, 2006, 2016, 2026, 2037, 2047,
};

/*
 * High-brightness mode: platform level 256..547 -> WRDISBV, transcribed from
 * the HBM_CANDELA_MAP table in the same data file. Level 256 is 5 -> 422 cd/m2
 * and level 547 is 2047 -> 900 cd/m2.
 *
 * WRDISBV RESTARTS at 5 here rather than continuing from the 2047 that ends the
 * normal table. That is not a transcription error: HBM is a different DDIC
 * state (WRCTRLD 0xE8 instead of 0x28) which reinterprets the same 0..2047
 * register range against a higher peak. Luminance is what stays continuous
 * across the boundary -- 420 cd/m2 at level 255, 422 at level 256 -- so the
 * combined ramp is smooth to the eye while the register value is not.
 *
 * The slope changes at level 395 (650 cd/m2): below it WRDISBV climbs ~12.5 per
 * level, above it ~2. That is where the panel's own rev-A variant stops -- the
 * device tree's ss_hbm_candela_map_table_revA pins 2047/650 for every level from
 * 395 to 547 -- so 650 cd/m2 is a real inflection in the DDIC's transfer curve
 * rather than an arbitrary cap, and rev B onwards simply keeps going past it.
 *
 * Which of the two tables applies is selected by panel revision. The device tree
 * declares only revA and revB; the vendor's parser falls back to the previous
 * revision for any letter it does not find (ss_wrapper_common.c: "If there is no
 * data for the panel rev, copy previous panel rev data pointer"), so revC and
 * revD both resolve to revB. This unit is revision D -- its own stock boot
 * cmdline records msm_drm.lcd_id=800004, and id2 0x04 maps to 'D' -- which makes
 * this 900 cd/m2 table the correct one. Anything other than rev A lands here.
 */
static const u16 ana38407_wrdisbv_hbm[292] = {
	   5,   17,   30,   43,   55,   68,   80,   93,
	 105,  118,  130,  143,  155,  168,  181,  193,
	 206,  218,  231,  243,  256,  268,  281,  293,
	 306,  319,  331,  344,  356,  369,  381,  394,
	 406,  419,  431,  444,  456,  469,  482,  494,
	 507,  519,  532,  544,  557,  569,  582,  594,
	 607,  620,  632,  645,  657,  670,  682,  695,
	 707,  720,  732,  745,  758,  770,  783,  795,
	 808,  820,  833,  845,  858,  870,  883,  896,
	 908,  921,  933,  946,  958,  971,  983,  996,
	1008, 1021, 1033, 1046, 1059, 1071, 1084, 1096,
	1109, 1121, 1134, 1146, 1159, 1171, 1184, 1197,
	1209, 1222, 1234, 1247, 1259, 1272, 1284, 1297,
	1309, 1322, 1335, 1347, 1360, 1372, 1385, 1397,
	1410, 1422, 1435, 1447, 1460, 1473, 1485, 1498,
	1510, 1523, 1535, 1548, 1560, 1573, 1585, 1598,
	1611, 1623, 1636, 1648, 1661, 1673, 1686, 1698,
	1711, 1723, 1736, 1744, 1747, 1749, 1751, 1753,
	1755, 1757, 1759, 1761, 1763, 1765, 1767, 1769,
	1771, 1773, 1775, 1777, 1779, 1781, 1783, 1785,
	1787, 1789, 1791, 1793, 1795, 1797, 1799, 1801,
	1803, 1805, 1807, 1809, 1811, 1813, 1815, 1817,
	1819, 1821, 1823, 1825, 1827, 1829, 1831, 1833,
	1835, 1837, 1839, 1841, 1843, 1845, 1847, 1849,
	1851, 1853, 1855, 1857, 1858, 1860, 1862, 1864,
	1866, 1868, 1870, 1872, 1874, 1876, 1878, 1880,
	1882, 1884, 1886, 1888, 1890, 1892, 1894, 1896,
	1898, 1900, 1902, 1904, 1906, 1908, 1910, 1912,
	1914, 1916, 1918, 1920, 1922, 1924, 1926, 1928,
	1930, 1932, 1934, 1936, 1938, 1940, 1942, 1944,
	1946, 1948, 1950, 1952, 1954, 1956, 1958, 1960,
	1962, 1964, 1966, 1968, 1970, 1972, 1974, 1976,
	1978, 1980, 1982, 1984, 1986, 1988, 1990, 1992,
	1994, 1996, 1998, 2000, 2002, 2004, 2006, 2008,
	2010, 2012, 2014, 2016, 2018, 2020, 2022, 2024,
	2026, 2028, 2030, 2032, 2034, 2036, 2038, 2040,
	2042, 2044, 2046, 2047,
};

/* First platform level served by the HBM table rather than the normal one. */
#define ANA38407_BL_HBM_FIRST_LEVEL	ARRAY_SIZE(ana38407_wrdisbv)

#define ANA38407_BL_MAX_LEVEL		(ANA38407_BL_HBM_FIRST_LEVEL + \
					 ARRAY_SIZE(ana38407_wrdisbv_hbm) - 1)

/*
 * Where the backlight starts. Deliberately the top of the NORMAL range and not
 * ANA38407_BL_MAX_LEVEL: WRDISBV then works out to 2047 in normal mode, which is
 * what the panel-on sequence sent before a backlight device existed, so a boot
 * that never touches sysfs still produces the identical DSI byte stream it
 * always did. Following the maximum instead would silently boot the panel into
 * HBM at 900 cd/m2 and hold it there.
 */
#define ANA38407_BL_DEFAULT_LEVEL	(ARRAY_SIZE(ana38407_wrdisbv) - 1)

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
 * panel-on. This is the DDIC's own emission-current limiter: per-frame and
 * content-adaptive, it trims drive as average picture level rises.
 *
 * DEFAULT 2, WHICH IS WHAT THE FACTORY SHIPS. A DDIC-internal read of RDACL
 * (0x56) across three live stacks gave OneUI = 0x02, bare TWRP = 0x01,
 * mainline = 0x00 -- so mainline was the only one of the three running with
 * the limiter switched off, purely because this knob was added as a flicker
 * probe and left at its "off" control value.
 *
 * The register was confirmed to actually latch here, which had never been
 * established: at maximum brightness, acl=2 draws 216.6 mA less than acl=0
 * (3.33 sigma, against a 0.48 sigma panel-cycle control), or 9.3% of total
 * system current. That is a trim rather than a protective clamp, and it is
 * free.
 *
 * The magnitude is content-conditional -- ACL trims by average picture level,
 * so that is the one quantity that necessarily varies with what is on screen.
 * It was measured against a mostly-white test field, not a full-screen white,
 * and should not be quoted as a fixed figure.
 *
 * The reason it was previously 0 has expired. ACL was a candidate suppressor
 * for the row-154 dark-seam flicker and the A/B came back ambiguous -- but
 * that flicker is now fixed by the panel's mode configuration, so the old
 * verdict is uninformative in both directions. Because ACL is per-frame and
 * content-adaptive, changing it is a display-path change and is re-verified
 * by eye at both refresh rates rather than assumed harmless.
 *
 * 0 = off, 1 = 8%, 2 = 15%, 3 = 20% (the levels named in the panel's own
 * data file). Settable at runtime, but read only during panel-on.
 */
static int acl = 2;
module_param(acl, int, 0644);
MODULE_PARM_DESC(acl, "ACL level at panel-on via WRACL 0x55: 0=off, 1=8%, 2=15% (default, matches stock), 3=20%");

struct ana38407 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	struct backlight_device *bl;

	/*
	 * drm_panel_funcs hands the panel no mode: prepare/enable/disable/
	 * unprepare all take only the drm_panel, and struct drm_panel carries
	 * neither a connector nor a mode. The DDIC bytes below are per-mode, so
	 * the connector is stashed in get_modes() and the active mode read back
	 * out of its atomic state at prepare time. This is the idiom upstream's
	 * panel-novatek-nt35950.c uses for the same problem.
	 */
	struct drm_connector *connector;
	int cur_mode;
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

/*
 * BRIGHTNESS_DIMMING_SETTING, from this device's own downstream data file
 * (GTS9U_ANA38407_AMSA46AS02.dat:93):
 *
 *	W 0x53 0xXX			WRCTRLD: 0x28 normal, 0xE8 HBM
 *	W 0x51 0xXX 0xXX		WRDISBV from the matching candela map
 *	${HBM_FlatZ_SETTING}		"IF REVISION BtoZ" -- our rev D qualifies
 *
 * Those two bytes are the WHOLE of high-brightness mode on this board. The
 * data file carries no HBM command block at all, and the vendor's common driver
 * couples HBM to more machinery -- ACL backup/restore, a vsync-gated entry,
 * separate ELVSS/IRC flash offsets -- none of which applies here: the ACL dance
 * sits inside "if (vdd->support_optical_fingerprint)" and this tablet reads
 * fingerprints from the power button, while gamma_mode2_hbm_tx_cmds,
 * hbm_irc_tx_cmds, flash_table_hbm_elvss_offset, flash_table_hbm_irc_offset and
 * panel_hbm_entry_delay are declared by zero files across the entire vendor
 * device tree. HBM_FlatZ is gated on revision rather than on mode, so it is sent
 * in both and needs no branch.
 *
 * The WRCTRLD encoding corroborates that 0xE8 is complete rather than merely
 * what stock happens to send. Across the data file's four branches -- 0x20 and
 * 0x28 normal, 0xE0 and 0xE8 HBM -- bit 3 is the standard DCS display-dimming
 * bit (cleared only for the fingerprint-flash cases, which need an instant
 * transition) and bit 5 is the brightness-control block. Normal to HBM changes
 * the top two bits and nothing else.
 *
 * Both the panel-on sequence and every runtime backlight update emit this one
 * block, so the DDIC only ever has a single writer telling it the level.
 *
 * The .dat nests HBM_FlatZ inside the outer level-key window; we keep it as its
 * own unlock/lock pair, which is the byte stream already validated on this
 * hardware. The register writes land identically either way.
 */
static void ana38407_send_brightness(struct mipi_dsi_multi_context *dsi_ctx,
				     unsigned int level)
{
	u8 wrctrld;
	u16 wrdisbv;
	u8 wrdisbv_cmd[3];

	level = min_t(unsigned int, level, ANA38407_BL_MAX_LEVEL);

	if (level >= ANA38407_BL_HBM_FIRST_LEVEL) {
		wrdisbv = ana38407_wrdisbv_hbm[level - ANA38407_BL_HBM_FIRST_LEVEL];
		wrctrld = 0xe8;
	} else {
		wrdisbv = ana38407_wrdisbv[level];
		wrctrld = 0x28;
	}

	/*
	 * The two must agree: each table's values are only meaningful under its
	 * own WRCTRLD. Sending an HBM WRDISBV in normal mode would land the
	 * bottom of the HBM table (5) below the dimmest normal level (10), i.e.
	 * a near-black panel where peak brightness was asked for.
	 *
	 * WRDISBV takes its high byte FIRST. That is what
	 * mipi_dsi_dcs_set_display_brightness_large() does; the plain
	 * mipi_dsi_dcs_set_display_brightness() and its _multi wrapper pack the
	 * payload the other way round and would send the two bytes swapped.
	 * There is no _large_multi, so the buffer is packed by hand here rather
	 * than through a helper that could silently reorder it.
	 */
	wrdisbv_cmd[0] = MIPI_DCS_SET_DISPLAY_BRIGHTNESS;
	wrdisbv_cmd[1] = (wrdisbv >> 8) & 0xff;
	wrdisbv_cmd[2] = wrdisbv & 0xff;

	/*
	 * _var_seq_ and not _seq_: the latter builds a "static const" array, so
	 * it takes compile-time constants only and will not accept a WRCTRLD
	 * chosen at runtime. The two macros are otherwise the same and pack
	 * their arguments in the order given -- unlike the DCS brightness
	 * helpers noted above, neither reorders anything.
	 */
	ana38407_unlock_lvl0(dsi_ctx);
	mipi_dsi_dcs_write_var_seq_multi(dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
					 wrctrld);
	mipi_dsi_dcs_write_buffer_multi(dsi_ctx, wrdisbv_cmd,
					sizeof(wrdisbv_cmd));
	ana38407_lock_lvl0(dsi_ctx);

	/*
	 * HBM_FlatZ_SETTING (IRC / "Flat Gamma Z"), transcribed byte-exact from
	 * this device's own downstream data file
	 * (GTS9U_ANA38407_AMSA46AS02.dat:140-141). The .dat applies it inside
	 * BRIGHTNESS_DIMMING_SETTING under "IF REVISION BtoZ" (.dat:106-107);
	 * our panel is rev D (id 0x04 -> index 3, within B..Z), so stock sends
	 * this on every brightness update. IRC is a DDIC low-gray gamma /
	 * luminance correction.
	 */
	ana38407_unlock_lvl0(dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xb0, 0x0a, 0xe0);
	mipi_dsi_dcs_write_seq_multi(dsi_ctx, 0xe0,
		0x3c, 0xfd, 0xff, 0x15, 0x00, 0x00, 0x66, 0xcc, 0x00, 0xff, 0x12);
	ana38407_lock_lvl0(dsi_ctx);
}

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

/*
 * ===========================================================================
 * MODE TABLE
 * ===========================================================================
 *
 * Two modes, both measured free of the dark-seam banding artifact that this
 * port fought for many months. Native 2960x1848 landscape; physical 313x196 mm.
 *
 * The artifact was a per-frame transient at a fixed panel row, and it turned
 * out to be governed by the relationship between the DDIC's scan rate and the
 * host's transfer cadence -- not by any register value. The DDIC can scan at
 * either 120 Hz or 60 Hz, selected by the VRR bytes stored per mode below;
 * TE follows the SCAN. When the host's frame rate equals the DDIC's scan rate
 * the host transmits on EVERY TE and the panel is clean. When the host runs at
 * half the scan rate (a 60 Hz host against a 120 Hz scan -- the "PHS"
 * low-frequency-driving regime, which every earlier revision of this driver
 * ran) it transmits on every OTHER TE, and the artifact appears.
 *
 * Both entries here therefore pair a host frame rate with the DDIC byte set
 * that makes the DDIC scan at that same rate:
 *
 *     120 Hz host + VRR_120HS scan   0 detections / 46 fiducial-verified
 *                                    transmitted frames (p = 1.2e-27)
 *      60 Hz host + VRR_60HS  scan   0 detections / 722 video frames
 *
 * The 60 Hz arm is the stronger evidence about MECHANISM: the frame transfer
 * still occupies only 43 % of the frame period there -- the exact duty cycle
 * that bands with p = 1.0 in the 120HS-scan regime -- so the duty cycle is
 * exonerated and the TE:transfer ratio is what matters. (Its denominator is
 * video frames rather than fiducial-verified transmitted frames: the 60HS
 * gamma regime lifts mid-tones enough to collapse the fiducial's ON/OFF
 * separation. The screen was demonstrably animating throughout.)
 *
 * ---------------------------------------------------------------------------
 * WHY HORIZONTAL BLANKING IS A CLOCK DIAL AND NOT PANEL TIMING
 * ---------------------------------------------------------------------------
 *
 * In command mode the host never programs the porches into hardware --
 * dsi_timing_setup() writes only STREAM0_CTRL (word count from the DSC
 * slice_chunk_size) and STREAM0_TOTAL (hdisplay/vdisplay); REG_DSI_ACTIVE_H,
 * REG_DSI_TOTAL and REG_DSI_ACTIVE_HSYNC are not written at all. The blanking
 * survives only inside the pixel-clock computation, which msm documents as
 * "the overhead to the image data transfer".
 *
 * Working the algebra through dsi_adjust_pclk_for_compression(), htotal cancels
 * and the link rate reduces to a function of the blanking alone:
 *
 *     pclk = vtotal * refresh * (hblank + hdisplay * bpp / (bpc * 3))
 *          = vtotal * refresh * (hblank + 987)
 *     bit clock = 6 * pclk
 *
 * so hblank sets how fast one frame is pushed into the DDIC's GRAM, at a fixed
 * frame cadence and with the transmitted payload unchanged. vtotal, by
 * contrast, IS hardware-visible in command mode -- it sets the tear-check
 * vsync_count and sync_cfg_height -- so vtotal must be the panel's real value
 * while hblank is free.
 *
 * This knob is EMPIRICALLY LOAD-BEARING: the artifact's row position tracked
 * it. Measured, single-variable, with everything else held identical:
 *
 *     bit clk 1495.2 -> row ~146      bit clk 1011.9 -> row 66.29 +- 1.12
 *     bit clk 2242.9 -> row 144.45 +- 0.50
 *
 * 144.45 - 66.29 = 78.16, and slice_height is 77: the band moved by exactly ONE
 * SLICE ROW and back. A fourth point at bit clk 1182, deliberately BETWEEN the
 * two rates that gave different rows, landed on 144 and NOT between -- so the
 * position is QUANTISED to the DSC slice grid. The link rate SELECTS which
 * slice boundary the artifact occupies; it does not position it continuously.
 * The occurrence rate did not change across any of it (17.9 % of frames before
 * and after), so the link rate set WHERE the artifact landed, never WHETHER.
 *
 * DO NOT slow the link below ~1 Gbps/lane. hblank 6 (846.5 Mbps, 78 % transfer)
 * made the display unusably slow -- that starves real bandwidth.
 *
 * ---------------------------------------------------------------------------
 * THE LINK RATE IS A DECLARED CONSTANT, NOT A DERIVED ONE
 * ---------------------------------------------------------------------------
 *
 * Measured on live stock across a 4x framerate change: 1,524,097,904 Hz at
 * 30 fps and 1,524,052,192 Hz at 120 fps -- 0.003 % apart, and both equal to
 * the panel node's qcom,mdss-dsi-panel-clockrate = <0x5ad66500> =
 * 1,524,000,000 Hz. Stock programs the PLL from that constant regardless of
 * mode. msm instead DERIVES it from the timings, so hblank has to be chosen to
 * land on it, and has to be re-chosen whenever vtotal or the framerate changes.
 * That is why neither mode below uses its vendor node's own blanking value.
 *
 * Both modes land within 0.015 % of the declared constant, and within 0.0023 %
 * of EACH OTHER. That second property is deliberate: the link rate is the knob
 * that moves the artifact's row position, so holding it equal across the two
 * modes keeps the mode switch a single-variable change.
 *
 * (An earlier revision of this comment claimed qcom,mdss-mdp-transfer-time-us
 * is "an MDP pacing target" and that stock "paces the DPU's output". Read in
 * the device's own downstream driver, that property has exactly two consumers
 * -- the MDP core-clock vote (_sde_crtc_reserve_resource) and the plane QoS
 * watermark (sde_crtc_update_line_time) -- both fetch-side. It does not pace
 * output. Measured on live stock it is simply how long the transfer TAKES: one
 * 2960x1848 8bpp-DSC frame over 4 lanes at the declared rate is 7.178 ms,
 * against the DT's 7533 us.)
 */

enum ana38407_mode_id {
	/*
	 * Index 0 is both the PREFERRED mode and the fallback returned by
	 * ana38407_get_current_mode() when the connector has no usable state.
	 * Keeping those the same means a fallback lands on the mode DRM is
	 * actually about to program, rather than on an untested pairing.
	 */
	ANA38407_MODE_120HS,
	ANA38407_MODE_60HS,
	ANA38407_NUM_MODES
};

struct ana38407_mode_data {
	struct drm_display_mode mode;
	/*
	 * The DDIC bytes that select the VRR regime; see VRR_SETTING in
	 * ana38407_on(). Register 0xDD is 0x00 in both regimes, so it is not
	 * stored per-mode.
	 */
	u8 vrr_60;
	u8 vrr_b9[4];
};

static const struct ana38407_mode_data ana38407_modes[ANA38407_NUM_MODES] = {
	/*
	 * 120 Hz -- the mode stock actually runs under load. Live capture of
	 * OneUI: idle it sits in the DT's 30 Hz node, under load it selects the
	 * 120 Hz node (2960(30|16|36|0)x1848(32|16|32|0)@120fps, measured
	 * 118.7). It never chose 60 Hz. The vertical timing here is that node's
	 * exactly: vfp 16, vpulse 32, vbp 32 -> vtotal 1928.
	 *
	 * hblank 111 rather than the node's own 82:
	 *     pclk = 1928 * 120 * (111 + 987) = 254,033,280 Hz
	 *     bit clock = 1,524,199,680 Hz   (0.013 % from the declared value)
	 * The node's own 82 would give 1,483,943,040 Hz (2.6 % low).
	 *
	 * Frame transfer is ~7.18 ms in both modes, but the frame period halves
	 * here, so the transfer occupies ~86 % of it -- which is what stock runs.
	 */
	[ANA38407_MODE_120HS] = {
		.mode = {
			.clock = (2960 + 45 + 36 + 30) * (1848 + 16 + 32 + 32) * 120 / 1000,
			.hdisplay = 2960,
			.hsync_start = 2960 + 45,
			.hsync_end = 2960 + 45 + 36,
			.htotal = 2960 + 45 + 36 + 30,
			.vdisplay = 1848,
			.vsync_start = 1848 + 16,
			.vsync_end = 1848 + 16 + 32,
			.vtotal = 1848 + 16 + 32 + 32,
			.width_mm = 313,
			.height_mm = 196,
			.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
		},
		.vrr_60 = 0x00,
		.vrr_b9 = { 0x80, 0x00, 0x00, 0x00 },
	},
	/*
	 * 60 Hz -- the vendor's wqxga60hs node. Vertical timing verbatim from
	 * it: vfp 127, vpulse 256, vbp 137 -> vtotal 2368.
	 *
	 * The vendor's DT carries the two 60 Hz regimes as separate timing nodes
	 * with IDENTICAL host timing: wqxga60hs and wqxga60phs are both vtotal
	 * 2368 and hblank 767, differing only by one line moved between vfp and
	 * vbp (60HS is 127/256/137, 60PHS is 128/256/136). The 60HS<->60PHS
	 * distinction is carried ENTIRELY by the DDIC bytes below.
	 *
	 * hblank 801 rather than the node's own 767:
	 *     pclk = 2368 * 60 * (801 + 987) = 254,039,040 Hz
	 *     bit clock = 1,524,234,240 Hz   (0.015 % from the declared value)
	 * The node's own 767 would give 1,495,249,920 Hz (1.9 % low).
	 */
	[ANA38407_MODE_60HS] = {
		.mode = {
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
			.type = DRM_MODE_TYPE_DRIVER,
		},
		.vrr_60 = 0x10,
		.vrr_b9 = { 0xaa, 0xaa, 0xaa, 0xaa },
	},
};

/*
 * Resolve the mode DRM is about to program, so ana38407_on() can send the
 * matching DDIC byte set.
 *
 * Every path logs. Driving one mode's host timing with the other mode's DDIC
 * bytes is precisely the untested pairing that produces the banding artifact,
 * and it looks entirely plausible on a static desktop -- so a silent fallback
 * here would be indistinguishable from a working mode switch. The journal, not
 * the eye, is what proves which branch ran.
 */
static int ana38407_get_current_mode(struct ana38407 *ctx)
{
	struct drm_connector *connector = ctx->connector;
	struct drm_crtc_state *crtc_state;
	int i;

	if (!connector || !connector->state || !connector->state->crtc) {
		dev_info(&ctx->dsi->dev,
			 "no connector state yet; using preferred mode (%d Hz)\n",
			 drm_mode_vrefresh(&ana38407_modes[0].mode));
		return 0;
	}

	crtc_state = connector->state->crtc->state;

	for (i = 0; i < ANA38407_NUM_MODES; i++) {
		if (drm_mode_match(&crtc_state->mode, &ana38407_modes[i].mode,
				   DRM_MODE_MATCH_TIMINGS | DRM_MODE_MATCH_CLOCK)) {
			dev_info(&ctx->dsi->dev,
				 "mode %d selected: %d Hz, vtotal %d, htotal %d\n",
				 i, drm_mode_vrefresh(&ana38407_modes[i].mode),
				 ana38407_modes[i].mode.vtotal,
				 ana38407_modes[i].mode.htotal);
			return i;
		}
	}

	dev_warn(&ctx->dsi->dev,
		 "active mode " DRM_MODE_FMT " matches no table entry; falling back to mode 0 (%d Hz) -- DDIC scan rate will NOT match the host frame rate\n",
		 DRM_MODE_ARG(&crtc_state->mode),
		 drm_mode_vrefresh(&ana38407_modes[0].mode));

	return 0;
}

static int ana38407_on(struct ana38407 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;
	const struct ana38407_mode_data *md;

	ctx->cur_mode = ana38407_get_current_mode(ctx);
	md = &ana38407_modes[ctx->cur_mode];

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

	/*
	 * BRIGHTNESS_DIMMING_SETTING + HBM_FlatZ (see ana38407_send_brightness).
	 *
	 * The level comes from props.brightness, NOT backlight_get_brightness():
	 * this runs from prepare(), before drm_panel_enable() has called
	 * backlight_enable(), so the backlight device is still blanked and the
	 * helper would report 0 -- lighting the panel at its 2 cd/m2 minimum for
	 * one transaction before the real level arrived.
	 */
	ana38407_send_brightness(&dsi_ctx, ctx->bl->props.brightness);

	/* SP_SETTING */
	ana38407_unlock_lvl0(&dsi_ctx);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x02);
	ana38407_lock_lvl0(&dsi_ctx);

	mipi_dsi_msleep(&dsi_ctx, 50);

	/*
	 * VRR_SETTING (rev D), transcribed from this device's own runtime-parsed
	 * panel_data_file/GTS9U_ANA38407_AMSA46AS02.dat (the VRR_SETTING table),
	 * cross-checked against the sess-133 live ss_print_cmd_desc capture. The
	 * DDIC always SCANS at 120HS; the refresh rate the host actually delivers
	 * is selected by the Low-Frequency-Driving divisor, not by the scan mode:
	 *
	 *     W 0x60 0xXX   120HS | 60PHS | 30PHS -> 0x00 ;  60HS | 30HS -> 0x10
	 *     W 0xB0 0x13 0xDD
	 *     W 0xDD 0xXX   120HS | 60HS  -> 0x00 ;  60PHS | 30HS -> 0x01
	 *                   30PHS -> 0x03 ; 24PHS -> 0x04 ; 10PHS -> 0x0B
	 *     W 0xB9 0xXX*4 60HS | 30HS -> 0xAA x4 ; ELSE -> 0x80 0x00 0x00 0x00
	 *                   (the rev-D/"CtoZ" branch; rev-A/B's 0xF7 0x07 latch is
	 *                   not sent on rev D)
	 *
	 * *** THE BYTES BELOW ARE PER-MODE. *** Each mode in ana38407_modes[]
	 * stores the byte set that makes the DDIC SCAN at that mode's host frame
	 * rate, so the two always agree:
	 *
	 *     120 Hz host -> VRR_120HS   0x60 = 0x00, 0xB9 = 0x80 0x00 0x00 0x00
	 *      60 Hz host -> VRR_60HS    0x60 = 0x10, 0xB9 = 0xAA x4
	 *
	 * TE follows the SCAN. When the two agree the host transmits on EVERY TE.
	 * Every revision of this driver before the two modes existed sent the
	 * VRR_120HS set unconditionally, so a 60 Hz host transmitted on every OTHER
	 * TE -- that is the "PHS" low-frequency-driving regime, and it banded.
	 *
	 * 0xDD does not move: the table gives 0x00 for 120HS and 60HS alike, and a
	 * 60 Hz build with 0xDD forced to 0x00 was separately shown to keep the
	 * band. The DDIC's emission rate is not the variable. (Note that this is
	 * also what makes the pre-two-mode byte set a native VRR_120HS set rather
	 * than a 60PHS one: 60PHS wants 0xDD = 0x01, which was never sent.)
	 *
	 * GLUT NOTE -- a KNOWN, DELIBERATE DIVERGENCE FROM THE VENDOR.
	 * update_glut() in the device's own downstream driver selects the gamma
	 * offset table by VRR base: ss_get_vrr_mode_base() maps 120 Hz -> VRR_120HS
	 * and 60 Hz + !phs -> VRR_60HS, and the table is then glut_offset_120hs or
	 * glut_offset_60hs respectively. This driver sends the 60HS table in BOTH
	 * modes, so at 120 Hz it sends a table the vendor would not.
	 *
	 * That is intentional and is NOT an oversight:
	 *   - it is exactly what the measured-clean 120 Hz build sent, so keeping it
	 *     leaves that arm byte-identical to its verified configuration;
	 *   - the contents of glut_offset_120hs have not been established from the
	 *     device's own data file. It is a runtime-parsed table, and the belief
	 *     that it is all zeros rests on a live capture, not on the source.
	 * Switching it is therefore not yet actionable. glut_zero=1 already forces
	 * an all-zero GLUT if a comparison is wanted, with no code change.
	 */
	ana38407_unlock_lvl0(&dsi_ctx);
	ana38407_unlock_lvl1(&dsi_ctx);
	{
		/*
		 * Both writes carry a runtime value, so they use
		 * write_buffer_multi: the _seq_multi macro builds a
		 * `static const u8[]` and only accepts literals.
		 */
		u8 c60[] = { 0x60, md->vrr_60 };

		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, c60, sizeof(c60));
	}
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x13, 0xdd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xdd, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x10, 0xb9);
	{
		u8 b9[] = { 0xb9, md->vrr_b9[0], md->vrr_b9[1],
			    md->vrr_b9[2], md->vrr_b9[3] };

		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, b9, sizeof(b9));
	}
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

static int ana38407_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	struct ana38407 *ctx = to_ana38407(panel);
	int i;

	for (i = 0; i < ANA38407_NUM_MODES; i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev,
					  &ana38407_modes[i].mode);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = ana38407_modes[0].mode.width_mm;
	connector->display_info.height_mm = ana38407_modes[0].mode.height_mm;

	/*
	 * The only place DRM hands this panel its connector. Stash it so
	 * ana38407_get_current_mode() can read the active mode back out of the
	 * atomic state at prepare time; drm_panel_funcs offers no other route.
	 */
	ctx->connector = connector;

	return ANA38407_NUM_MODES;
}

static int ana38407_bl_update_status(struct backlight_device *bl)
{
	struct ana38407 *ctx = bl_get_data(bl);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/*
	 * A sysfs write can arrive at any time, including with the panel
	 * powered down, where DCS would go out over a dead link. drm_panel's
	 * enable/disable ordering covers the normal paths; this covers the rest.
	 *
	 * The blank check matters separately: drm_panel_disable() calls
	 * backlight_disable() BEFORE funcs->disable(), so without it every
	 * screen-off would first ramp the panel down to its 2 cd/m2 minimum --
	 * with smooth dimming enabled, a visible fade -- and only then blank.
	 */
	if (backlight_is_blank(bl) || !ctx->panel.enabled)
		return 0;

	ana38407_send_brightness(&dsi_ctx, backlight_get_brightness(bl));

	return dsi_ctx.accum_err;
}

static const struct backlight_ops ana38407_bl_ops = {
	.update_status = ana38407_bl_update_status,
};

static int ana38407_backlight_init(struct ana38407 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		/*
		 * The vendor curve puts its finer steps at the bottom (level
		 * 127 = 41% of peak luminance, not 50%), so tell userspace not
		 * to apply a perceptual correction of its own on top.
		 */
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
		/*
		 * The range runs past the normal table's 420 cd/m2 into
		 * high-brightness mode, so the slider reaches the top of the
		 * vendor's HBM table, whose last entry is labelled 900 cd/m2.
		 *
		 * That 900 is the DATA FILE'S figure, not a measured delivered
		 * luminance, and OLED peak specifications are usually quoted for
		 * a small window at low average picture level rather than a full
		 * white screen. What has actually been established here is that
		 * the top of the range is much brighter than the old 420 cd/m2
		 * maximum; whether the panel sustains 900 at 100% APL is open.
		 * The luminance-versus-power calibration that validated the
		 * normal curve does not settle it either -- it was measured
		 * entirely below this range, and the transfer curve visibly
		 * changes slope at the 650 cd/m2 inflection noted above.
		 *
		 * This arrangement also exceeds stock, which keeps HBM behind an
		 * "Extra brightness" toggle and, under auto-brightness, gates it
		 * on the ambient light sensor and thermal state. There is no
		 * ambient gate, thermal governor or duration limit here: the top
		 * of this slider parks the panel at peak indefinitely. Measured
		 * at the top of the range, battery temperature rose from 37.1 to
		 * 46.5 degrees C within minutes and the charger throttled itself
		 * from 2022 to 1334 mA, recovering immediately when brightness
		 * came down -- so the thermal cost is real and nothing in this
		 * driver will act on it. That is a deliberate choice by the owner
		 * of this device, who manages it directly.
		 */
		.max_brightness = ANA38407_BL_MAX_LEVEL,
		.brightness = ANA38407_BL_DEFAULT_LEVEL,
	};

	ctx->bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
						 &ana38407_bl_ops, &props);
	if (IS_ERR(ctx->bl))
		return dev_err_probe(dev, PTR_ERR(ctx->bl),
				     "Failed to register backlight\n");

	/*
	 * Hand it to drm_panel, which enables the backlight after the panel and
	 * disables it before -- the ordering this driver would otherwise have to
	 * open-code.
	 */
	ctx->panel.backlight = ctx->bl;

	return 0;
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

	/*
	 * Before drm_panel_add(): that call publishes the panel, after which a
	 * consumer may prepare it, and the panel-on sequence reads ctx->bl.
	 */
	ret = ana38407_backlight_init(ctx);
	if (ret < 0)
		return ret;

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
