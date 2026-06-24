// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5440 switched-capacitor 2:1 charge-pump - telemetry driver.
 *
 * The SM5440 is the cell-side 2:1 charge pump on the Samsung Galaxy Tab S9
 * Ultra (SM-X910 / gts9u), used for high-power (45 W) fast charging in
 * series with the SM5714 primary charger.  This driver is the register /
 * ADC telemetry layer: it probes the chip, gates on its DEVICEID, enables
 * the on-chip ADC, and exposes input voltage / input current / die
 * temperature as a power_supply.
 *
 * It deliberately does NOT control charging.  The pump's power FETs are
 * gated solely by the op-mode field (CNTL5, bits[3:2]); this driver never
 * writes that register, so op-mode stays at its power-on CHG_OFF default and
 * the pump never switches.  The fast-charge control loop lands separately.
 *
 * Register addresses, ADC scaling and the DEVICEID gate are transcribed
 * verbatim from the device's own downstream driver,
 * drivers/battery/charger/sm5440_charger/sm5440_charger.c (silicon rev "WE2").
 *
 * Copyright (C) 2026 omar fityan <me@omarfityan.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/*
 * Cross-driver hooks for the buck<->pump handoff and PPS voltage stepping.  The
 * SM5714 Type-C/PD role driver (PPS contract) and the SM5714 VBUS/charger driver
 * (the buck) live in drivers/usb/misc; both expose no-op fallbacks when not
 * built, so the telemetry-only path still builds without them.  Included by
 * relative path for this bring-up (a shared include/linux/ header is a planned
 * cleanup before upstreaming).
 */
#include "../../usb/misc/sm5714-typec.h"
#include "../../usb/misc/sm5714-usb-vbus.h"

/* The ported sm_dc CC/CV direct-charging engine (sm5440_direct_charger.c). */
#include "sm5440_direct_charger.h"

/* Register map (device-own sm5440_charger.h). */
#define SM5440_REG_INT1		0x00
#define SM5440_REG_INT2		0x01
#define SM5440_REG_INT3		0x02
#define SM5440_REG_INT4		0x03
#define SM5440_REG_STATUS1	0x08
#define SM5440_REG_STATUS2	0x09
#define SM5440_REG_STATUS3	0x0a
#define SM5440_REG_STATUS4	0x0b
#define SM5440_REG_CNTL1	0x0c
#define SM5440_REG_CNTL2	0x0d
#define SM5440_REG_CNTL3	0x0e
#define SM5440_REG_CNTL4	0x0f
#define SM5440_REG_CNTL5	0x10
#define SM5440_REG_CNTL6	0x11
#define SM5440_REG_CNTL7	0x12
#define SM5440_REG_VBUSCNTL	0x13
#define SM5440_REG_VBATCNTL	0x14
#define SM5440_REG_VOUTCNTL	0x15
#define SM5440_REG_IBUSCNTL	0x16
#define SM5440_REG_PRTNCNTL	0x19
#define SM5440_REG_THEMCNTL1	0x1a
#define SM5440_REG_ADCCNTL1	0x1c
#define SM5440_REG_ADCCNTL2	0x1d
#define SM5440_REG_ADC_VBUS1	0x1e
#define SM5440_REG_ADC_VBUS2	0x1f
#define SM5440_REG_ADC_VOUT1	0x20
#define SM5440_REG_ADC_VOUT2	0x21
#define SM5440_REG_ADC_IBUS1	0x22
#define SM5440_REG_ADC_IBUS2	0x23
#define SM5440_REG_ADC_THEM1	0x24
#define SM5440_REG_ADC_THEM2	0x25
#define SM5440_REG_ADC_DIETEMP	0x26
#define SM5440_REG_ADC_VBAT1	0x27
#define SM5440_REG_ADC_VBAT2	0x28
#define SM5440_REG_DEVICEID	0x2b

/* STATUS3: VBUS power-OK (device-own psy_chg_get_online: bit5). */
#define SM5440_STATUS3_VBUS_POK	BIT(5)

/* DEVICEID: low nibble == 1 means the chip is present; high nibble is rev. */
#define SM5440_DEVICEID_MASK	GENMASK(3, 0)
#define SM5440_DEVICEID_PRESENT	0x1
#define SM5440_DEVICEID_REV(x)	((x) >> 4)

/*
 * ADC enable, telemetry-only (device-own sm5440_init_reg_param +
 * enable_adc / set_adc_rate):
 *   ADCCNTL2 = 0xdf  - per-channel enable (device-own value, verbatim)
 *   ADCCNTL1 = 0x0b  - bit0 ADC enable | bit1 continuous rate | bit3 32-avg
 */
#define SM5440_ADCCNTL2_CHANNELS	0xdf
#define SM5440_ADCCNTL1_ENABLE		0x0b

/*
 * op-mode field (CNTL5 bits[3:2], device-own enum sm5440_op_mode).  CHG_OFF is
 * also the power-on/idle state; CHG_ON switches the 2:1 pump for forward
 * charging.  REV_* are reverse/D2D boost modes and are not used here.
 */
#define SM5440_OPMODE_SHIFT	2
#define SM5440_OPMODE_MASK	0x3
#define SM5440_OPMODE_CHG_OFF	0x0
#define SM5440_OPMODE_CHG_ON	0x1

/* WDT timer field (CNTL1 bits[6:4]); 0x4 == 30 s (device-own WDT_TIMER_S_30). */
#define SM5440_WDT_TIMER_30S	0x4

/*
 * Fault / status bits.  The INT1-4 registers are clear-on-read; their STATUS1-4
 * level mirrors carry the SAME bit layout and are re-readable, so a POLLED
 * driver (this board has no SM5440 irq-gpio) MUST read STATUS, never INT.
 */
#define SM5440_STATUS1_VOUTOVP	BIT(4)
#define SM5440_STATUS1_VBATOVP	BIT(3)
#define SM5440_STATUS2_IBUSLIM	BIT(7)	/* CC input-current loop active */
#define SM5440_STATUS2_VBATREG	BIT(3)	/* CV loop active */
#define SM5440_STATUS2_THEM_REG	BIT(1)	/* thermal regulation active */
#define SM5440_STATUS3_VBUS_POK	BIT(5)
#define SM5440_STATUS3_VBUSOVP	BIT(7)
#define SM5440_STATUS3_VBUSUVLO	BIT(6)
#define SM5440_STATUS3_THEMSHDN_ALM	BIT(4)	/* thermal-shutdown alarm (SW-OCP) */
#define SM5440_STATUS3_STUP_FAIL	BIT(2)
#define SM5440_STATUS3_REVBLK	BIT(1)
#define SM5440_STATUS3_CFLY_SHORT	BIT(0)
#define SM5440_STATUS4_MIDVBUS2VOUT	BIT(5)
#define SM5440_STATUS4_SW_RDY	BIT(4)

/* The five forced-cutoff faults to poll for during a pump engage. */
#define SM5440_FAULT_S1		(SM5440_STATUS1_VOUTOVP)
#define SM5440_FAULT_S3		(SM5440_STATUS3_VBUSOVP | SM5440_STATUS3_STUP_FAIL | \
				 SM5440_STATUS3_REVBLK | SM5440_STATUS3_CFLY_SHORT)

/*
 * init_reg_param values (device-own sm5440_init_reg_param), transcribed
 * verbatim.  CNTL2 disables HW IBUS/IBAT-OCP + THEM (SW OCP used instead);
 * CNTL6 forces PWM and clears ENHIZ; VBUSCNTL[2:0]=7 is the 11 V VBUS-OVP;
 * CNTL3 disables the charge-timer and enables VBATREG.
 */
#define SM5440_CNTL2_INIT	0xf2
#define SM5440_CNTL4_INIT	0xff
#define SM5440_CNTL6_INIT	0x09
#define SM5440_VOUTCNTL_INIT	0x3f
#define SM5440_PRTNCNTL_INIT	0xfe
#define SM5440_THEMCNTL1_INIT	0x0c
#define SM5440_CNTL3_INIT	0xb8
#define SM5440_VBUSCNTL_OVP_11V	0x7	/* VBUSCNTL bits[2:0] */

/*
 * Per-register encoders (device-own).  VBATREG: (mV-3800)*10/125 into bits[5:0];
 * IBUSLIM: mA/50 (full byte); FREQ: (kHz-250)/50 into bits[4:0].
 */
#define SM5440_VBATREG_REG(mv)	((((mv) - 3800) * 10) / 125)
#define SM5440_VBATREG_MASK	0x3f
#define SM5440_IBUSLIM_REG(ma)	((ma) / 50)
#define SM5440_FREQ_REG(khz)	((((khz) - 250) / 50) & 0x1f)
#define SM5440_FREQ_MASK	0x1f

/*
 * gts9u board values (device-own sm5440_charger.dtsi): switching freq 450 kHz,
 * total path resistance 500 mOhm for the PPS-voltage IR offset, en_vbatreg=0
 * (so the chip VBATREG ceiling sits CV_OFFSET=50 mV above the cell-V target).
 */
#define SM5440_GTS9U_FREQ_KHZ	450
#define SM5440_GTS9U_R_TTL_UOHM	500000
#define SM5440_CV_OFFSET	50
#define SM5440_TA_MIN_VOLTAGE	8200	/* PPS request floor (device-own config) */
#define SM5440_DC_VBUS_OVP_TH	11000	/* PPS ceiling = this - 500 = 10500 mV */

/*
 * Increment-1 guarded manual-engage parameters (NOT the production step table).
 * A conservative 2000 mA input limit -> ~4000 mA into the cell via the 2:1 pump,
 * a clear ~2x over the SM5714 buck's ~2100 mA baseline yet well under step-0's
 * 4500 mA; the cell-V ceiling is step-0's 4250 mV.  Auto-off backstop at 30 s.
 */
#define SM5440_ENGAGE_IBUS_MA	2000
#define SM5440_ENGAGE_VBAT_MV	4250
#define SM5440_ENGAGE_MAX_TICKS	30	/* monitor ticks (1 s each) before auto-off */

/*
 * Increment-2 sm_dc engine config + step-0 test targets (device-own values,
 * sm5440_charger.{c,h}).  IBUSLIM = ci_gl + CI_OFFSET (the chip targets the
 * input-current goal plus headroom); SIOP freq de-rating is unused in our test
 * range (ci_gl >= 2000 > SIOP_LEV2), so set_charging_config uses the single
 * 450 kHz.  TOPOFF/CHG_FLOAT drive the CV done-event only when target == float
 * (step-0's 4250 mV != 4440, so DONE does not fire -- disengage is by WDT/stop).
 */
#define SM5440_CI_OFFSET	300
#define SM5440_TA_MIN_CURRENT	1000
#define SM5440_SIOP_LEV2	1700
#define SM5440_DC_TOPOFF_MA	1000
#define SM5440_DC_CHG_FLOAT_MV	4440
#define SM5440_DC_STEP0_VBAT_MV	4250	/* step-0 (<=62% SoC) cell-V ceiling */
#define SM5440_VBAT_MIN		3300	/* device-own SM5440_VBAT_MIN / dc_min_vbat */

struct sm5440 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *fg;	/* sm5714-fuelgauge, for the engage cross-check */
	struct delayed_work pump_monitor;
	/*
	 * engage_lock serialises BOTH manual bring-up triggers (the Inc-1 pump_test
	 * one-shot engage AND the Inc-2 dc_test engine run) + their monitors.  They
	 * are mutually exclusive: each path checks the other's pump_engaged/dc_running
	 * flag under this one lock before touching the shared pump / FG handle (a
	 * single lock, not two disjoint ones, is what makes that check race-free).
	 */
	struct mutex engage_lock;
	bool pump_engaged;
	int monitor_ticks;
	u8 rev_id;

	/*
	 * Increment-2: the ported sm_dc CC/CV engine drives the closed-loop ramp.
	 * dc is the engine instance (its ops are this driver's sm5440_dc_ops);
	 * ocp_check_work is the device-faithful SW-OCP backstop; dc_monitor is a
	 * LOG-ONLY telemetry logger for the engine run (it does NOT auto-disengage --
	 * disengage is owned by the engine / chip WDT / dc_test=0).
	 */
	struct sm_dc_info *dc;
	struct delayed_work ocp_check_work;
	struct delayed_work dc_monitor;
	bool dc_running;
	int dc_ticks;
};

/*
 * The SM5440 ADC packs a 13-bit sample across two registers as
 * (hi << 5) | (lo >> 3) (device-own sm5440_convert_adc).
 */
static int sm5440_read_adc_raw(struct sm5440 *chip, u8 hi_reg, u8 lo_reg, u32 *raw)
{
	unsigned int hi, lo;
	int ret;

	ret = regmap_read(chip->regmap, hi_reg, &hi);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap, lo_reg, &lo);
	if (ret)
		return ret;

	*raw = (hi << 5) | (lo >> 3);
	return 0;
}

/* VBUS input voltage, microvolts.  Device-own: mV = 4096 + raw. */
static int sm5440_get_vbus_uv(struct sm5440 *chip, int *uv)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_VBUS1,
				  SM5440_REG_ADC_VBUS2, &raw);
	if (ret)
		return ret;

	*uv = (4096 + raw) * 1000;
	return 0;
}

/* IBUS input current, microamps.  Device-own: mA = raw * 625 / 1000. */
static int sm5440_get_ibus_ua(struct sm5440 *chip, int *ua)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_IBUS1,
				  SM5440_REG_ADC_IBUS2, &raw);
	if (ret)
		return ret;

	*ua = raw * 625;
	return 0;
}

/*
 * Die temperature, tenths of a degree C (power_supply TEMP unit).
 * Device-own: deci-C = 225 + reg * 5 (single register, "C x 10").
 */
static int sm5440_get_dietemp_dc(struct sm5440 *chip, int *dc)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(chip->regmap, SM5440_REG_ADC_DIETEMP, &reg);
	if (ret)
		return ret;

	*dc = 225 + reg * 5;
	return 0;
}

/* VBAT (pump output = cell side) and VOUT, millivolts.  Device-own: 2048 + raw*500/1000. */
static int sm5440_get_vbat_mv(struct sm5440 *chip, int *mv)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_VBAT1,
				  SM5440_REG_ADC_VBAT2, &raw);
	if (ret)
		return ret;

	*mv = 2048 + (raw * 500) / 1000;
	return 0;
}

static int sm5440_get_vout_mv(struct sm5440 *chip, int *mv)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_VOUT1,
				  SM5440_REG_ADC_VOUT2, &raw);
	if (ret)
		return ret;

	*mv = 2048 + (raw * 500) / 1000;
	return 0;
}

/*
 * Charge-control register helpers.  update_field is a read-modify-write of a
 * bit field (device-own sm5440_update_reg(reg, val, mask, pos)).
 */
static int sm5440_update_field(struct sm5440 *chip, u8 reg, u8 val, u8 mask, u8 pos)
{
	return regmap_update_bits(chip->regmap, reg, mask << pos,
				  (val & mask) << pos);
}

static int sm5440_set_op_mode(struct sm5440 *chip, u8 op_mode)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL5, op_mode,
				   SM5440_OPMODE_MASK, SM5440_OPMODE_SHIFT);
}

static int sm5440_get_op_mode(struct sm5440 *chip)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(chip->regmap, SM5440_REG_CNTL5, &reg);
	if (ret)
		return ret;
	return (reg >> SM5440_OPMODE_SHIFT) & SM5440_OPMODE_MASK;
}

static int sm5440_set_vbatreg(struct sm5440 *chip, u32 mv)
{
	u8 reg = (mv < 3800) ? 0 : SM5440_VBATREG_REG(mv);

	return sm5440_update_field(chip, SM5440_REG_VBATCNTL, reg,
				   SM5440_VBATREG_MASK, 0);
}

static int sm5440_set_ibuslim(struct sm5440 *chip, u32 ma)
{
	return regmap_write(chip->regmap, SM5440_REG_IBUSCNTL,
			    SM5440_IBUSLIM_REG(ma));
}

static int sm5440_set_freq(struct sm5440 *chip, u32 khz)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL7,
				   SM5440_FREQ_REG(khz), SM5440_FREQ_MASK, 0);
}

static int sm5440_set_wdt_timer(struct sm5440 *chip, u8 tmr)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL1, tmr, 0x7, 4);
}

static int sm5440_enable_wdt(struct sm5440 *chip, bool enable)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL1, enable, 0x1, 7);
}

static int sm5440_enable_chg_timer(struct sm5440 *chip, bool enable)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL3, enable, 0x1, 2);
}

/* SW reset: write CNTL1 bit0, then poll it clear (device-own sm5440_sw_reset). */
static int sm5440_sw_reset(struct sm5440 *chip)
{
	unsigned int reg;
	int i, ret;

	ret = regmap_write(chip->regmap, SM5440_REG_CNTL1, 0x1);
	if (ret)
		return ret;

	for (i = 0; i < 0xff; i++) {
		usleep_range(1000, 2000);
		if (!regmap_read(chip->regmap, SM5440_REG_CNTL1, &reg) &&
		    !(reg & 0x1))
			return 0;
	}

	dev_err(chip->dev, "sw reset bit did not clear\n");
	return -EBUSY;
}

/*
 * One-time chip init before op-mode -> CHG_ON (device-own sm5440_init_reg_param),
 * transcribed verbatim in the same order.  Best-effort like the vendor (the
 * individual writes are not checked); the op-mode flip later is the gate.
 */
static void sm5440_init_reg_param(struct sm5440 *chip)
{
	sm5440_set_wdt_timer(chip, SM5440_WDT_TIMER_30S);
	sm5440_set_freq(chip, SM5440_GTS9U_FREQ_KHZ);
	regmap_write(chip->regmap, SM5440_REG_CNTL2, SM5440_CNTL2_INIT);
	regmap_write(chip->regmap, SM5440_REG_CNTL4, SM5440_CNTL4_INIT);
	regmap_write(chip->regmap, SM5440_REG_CNTL6, SM5440_CNTL6_INIT);
	sm5440_update_field(chip, SM5440_REG_VBUSCNTL, SM5440_VBUSCNTL_OVP_11V, 0x7, 0);
	regmap_write(chip->regmap, SM5440_REG_VOUTCNTL, SM5440_VOUTCNTL_INIT);
	regmap_write(chip->regmap, SM5440_REG_PRTNCNTL, SM5440_PRTNCNTL_INIT);
	sm5440_update_field(chip, SM5440_REG_ADCCNTL1, 1, 0x1, 3);  /* 32-sample avg */
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL2, SM5440_ADCCNTL2_CHANNELS);
	regmap_write(chip->regmap, SM5440_REG_THEMCNTL1, SM5440_THEMCNTL1_INIT);
	regmap_write(chip->regmap, SM5440_REG_CNTL3, SM5440_CNTL3_INIT);
}

/*
 * Poll the STATUS1-4 level mirrors for the forced-cutoff faults.  Returns the
 * raw STATUS bytes via st[] and a bool "a hard fault is asserted".  Polled (no
 * irq-gpio): read STATUS, never the clear-on-read INT registers.
 */
static bool sm5440_fault_present(struct sm5440 *chip, u8 st[4])
{
	int ret;

	ret = regmap_bulk_read(chip->regmap, SM5440_REG_STATUS1, st, 4);
	if (ret) {
		dev_warn(chip->dev, "STATUS read failed (%d)\n", ret);
		return false;
	}
	return (st[0] & SM5440_FAULT_S1) || (st[2] & SM5440_FAULT_S3);
}

/*
 * Chip-side pump engage (device-faithful order: sw_reset -> init -> ADC on ->
 * IBUSLIM/VBATREG -> op-mode CHG_ON -> WDT).  The PPS input must already be
 * commanded to ~2x Vcell and the buck disarmed by the caller.  sw_reset wipes
 * the telemetry ADC config, so it is re-enabled here and restored on disengage.
 */
static int sm5440_pump_engage_chip(struct sm5440 *chip, u32 ibus_ma, u32 vbat_mv)
{
	int ret;

	ret = sm5440_sw_reset(chip);
	if (ret)
		return ret;

	sm5440_init_reg_param(chip);
	/* re-enable continuous telemetry ADC (sw_reset cleared ADCCNTL1). */
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);

	sm5440_set_ibuslim(chip, ibus_ma);
	/* en_vbatreg=0 on gts9u: the chip CV ceiling sits CV_OFFSET above target. */
	sm5440_set_vbatreg(chip, vbat_mv + SM5440_CV_OFFSET);
	sm5440_set_wdt_timer(chip, SM5440_WDT_TIMER_30S);
	sm5440_enable_chg_timer(chip, false);

	ret = sm5440_set_op_mode(chip, SM5440_OPMODE_CHG_ON);  /* the pump-on write */
	if (ret)
		return ret;
	sm5440_enable_wdt(chip, true);
	return 0;
}

/* Chip-side disengage: op-mode CHG_OFF, WDT off, restore telemetry ADC config. */
static void sm5440_pump_disengage_chip(struct sm5440 *chip)
{
	sm5440_set_op_mode(chip, SM5440_OPMODE_CHG_OFF);
	sm5440_enable_wdt(chip, false);
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL2, SM5440_ADCCNTL2_CHANNELS);
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);
}

/*
 * Increment-1 guarded manual pump-engage orchestration.  This is NOT the
 * production CC/CV loop (that is the ported sm_dc engine) -- it is a one-shot,
 * conservative, instrumented engage to prove on hardware that the pump switches
 * (2:1, VBUS ~= 2*VOUT), draws input current, and charges the cell faster than
 * the SM5714 buck, before the ~850 LOC engine is ported.  It coordinates all
 * three chips: disarm the SM5714 buck, step the SM5714 PPS contract down to
 * ~2*Vcell, then flip the SM5440 op-mode to CHG_ON; a 1 s monitor logs the
 * conversion + currents + faults and auto-disengages on any fault or after a
 * short backstop window.
 */

/*
 * Tear down (device-own order: pump OFF first to block reverse current into the
 * TA, then restore the buck + PPS).  Caller holds engage_lock; the caller must
 * NOT be inside a monitor sync-cancel while holding it.
 */
static void sm5440_pump_disengage_locked(struct sm5440 *chip)
{
	sm5440_pump_disengage_chip(chip);		/* op-mode -> CHG_OFF first */
	chip->pump_engaged = false;
	sm5714_pd_request_voltage(10000, 3000);		/* restore the 10 V sustain baseline */
	sm5714_charger_inhibit_buck(false);		/* re-allow the buck to charge */
	if (chip->fg) {
		power_supply_put(chip->fg);
		chip->fg = NULL;
	}
	dev_info(chip->dev, "pump DISENGAGED (buck re-allowed, PPS restored to 10 V)\n");
}

static void sm5440_pump_monitor_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, pump_monitor.work);
	int vbus_uv = 0, ibus_ua = 0, dietemp = 0, vout_mv = 0, vbat_mv = 0;
	int fg_uv = 0, fg_ua = 0, opmode;
	union power_supply_propval pv;
	bool fault;
	u8 st[4] = { };

	mutex_lock(&chip->engage_lock);
	if (!chip->pump_engaged) {
		mutex_unlock(&chip->engage_lock);
		return;
	}

	sm5440_get_vbus_uv(chip, &vbus_uv);
	sm5440_get_ibus_ua(chip, &ibus_ua);
	sm5440_get_dietemp_dc(chip, &dietemp);
	sm5440_get_vout_mv(chip, &vout_mv);
	sm5440_get_vbat_mv(chip, &vbat_mv);
	opmode = sm5440_get_op_mode(chip);
	fault = sm5440_fault_present(chip, st);

	/*
	 * FG cross-check: the en_vbatreg=0 vnow source decision for Increment 2 is
	 * "does the SM5440 VBAT ADC agree with the fuel-gauge?" -- log both so the
	 * data, not an inference, picks the CC/CV regulation source.
	 */
	if (chip->fg) {
		if (!power_supply_get_property(chip->fg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pv))
			fg_uv = pv.intval;
		if (!power_supply_get_property(chip->fg,
				POWER_SUPPLY_PROP_CURRENT_NOW, &pv))
			fg_ua = pv.intval;
	}

	dev_info(chip->dev,
		 "pump[%2d]: opmode=%d VBUS=%dmV 2xVOUT=%dmV IBUS=%dmA die=%d.%dC | cell SM5440-VBAT=%dmV FG-V=%dmV FG-I=%dmA | STATUS=%02x:%02x:%02x:%02x%s\n",
		 chip->monitor_ticks, opmode, vbus_uv / 1000, vout_mv * 2,
		 ibus_ua / 1000, dietemp / 10, dietemp % 10, vbat_mv,
		 fg_uv / 1000, fg_ua / 1000, st[0], st[1], st[2], st[3],
		 fault ? "  *** FAULT ***" : "");

	if (fault || opmode != SM5440_OPMODE_CHG_ON ||
	    ++chip->monitor_ticks >= SM5440_ENGAGE_MAX_TICKS) {
		dev_warn(chip->dev, "pump auto-disengage: %s\n",
			 fault ? "hard fault" :
			 opmode != SM5440_OPMODE_CHG_ON ? "op-mode dropped (WDT/STUP?)" :
			 "max window reached");
		sm5440_pump_disengage_locked(chip);
		mutex_unlock(&chip->engage_lock);
		return;
	}

	schedule_delayed_work(&chip->pump_monitor, msecs_to_jiffies(1000));
	mutex_unlock(&chip->engage_lock);
}

/* Engage sequence.  Caller holds engage_lock and has checked !pump_engaged. */
static int sm5440_pump_engage(struct sm5440 *chip)
{
	int vbat_mv = 0, target_mv, off_mv, ret;

	if (!sm5714_pd_contract_active()) {
		dev_warn(chip->dev,
			 "engage: no PPS contract active -- arm pd_request + plug first\n");
		return -ENOTCONN;
	}

	/* device-own handoff: buck OFF before pump ON. */
	ret = sm5714_charger_inhibit_buck(true);
	if (ret) {
		dev_warn(chip->dev, "engage: buck inhibit failed (%d)\n", ret);
		return ret;
	}

	/*
	 * Command the PPS input to ~2*Vcell + IR offset (device-own
	 * _calc_pps_v_init_offset = Ibus*r_ttl/1e6 + 200 mV), clamped to
	 * [ta_min_voltage, dc_vbus_ovp_th - 500].  Engaging at the 10 V keepalive
	 * would be ~2.5:1 and trip the pump's startup precondition.  The SM5440 VBAT
	 * ADC reads live because the contract already supplies VBUS.
	 */
	ret = sm5440_get_vbat_mv(chip, &vbat_mv);
	if (ret || vbat_mv < 2500 || vbat_mv > 4500) {
		dev_warn(chip->dev, "engage: implausible VBAT %d mV (ret=%d) -- aborting\n",
			 vbat_mv, ret);
		ret = ret ? ret : -EIO;
		goto err_uninhibit;
	}
	off_mv = (SM5440_ENGAGE_IBUS_MA * (SM5440_GTS9U_R_TTL_UOHM / 1000)) / 1000 + 200;
	target_mv = 2 * vbat_mv + off_mv;
	target_mv = min(target_mv, SM5440_DC_VBUS_OVP_TH - 500);
	target_mv = max(target_mv, SM5440_TA_MIN_VOLTAGE);

	ret = sm5714_pd_request_voltage(target_mv, 5000);
	if (ret) {
		dev_warn(chip->dev, "engage: PPS request %d mV failed (%d)\n",
			 target_mv, ret);
		goto err_uninhibit;
	}
	dev_info(chip->dev,
		 "engage: VBAT=%dmV -> PPS target=%dmV (offset %dmV); settling 300 ms\n",
		 vbat_mv, target_mv, off_mv);
	msleep(300);

	/*
	 * Confirm the PPS actually transitioned before flipping op-mode.  The
	 * keepalive re-Request passes through the SinkTxOk collision gate (unverified
	 * on this hardware); if it deferred, VBUS may still be ~10 V, which would
	 * engage at ~2.5:1 into the cell and trip STUP_FAIL.  Verifying VBUS here
	 * turns that into a clean "PPS did not transition" abort instead.
	 */
	{
		int vbus_uv = 0, vbus_mv, diff;

		sm5440_get_vbus_uv(chip, &vbus_uv);
		vbus_mv = vbus_uv / 1000;
		diff = vbus_mv - target_mv;
		if (diff < 0)
			diff = -diff;
		if (diff > target_mv / 10) {
			dev_warn(chip->dev,
				 "engage: VBUS=%dmV not within 10%% of target %dmV (PPS did not transition) -- aborting\n",
				 vbus_mv, target_mv);
			ret = -EIO;
			goto err_uninhibit;
		}
		dev_info(chip->dev,
			 "engage: VBUS settled to %dmV (target %dmV) -- flipping op-mode\n",
			 vbus_mv, target_mv);
	}

	ret = sm5440_pump_engage_chip(chip, SM5440_ENGAGE_IBUS_MA, SM5440_ENGAGE_VBAT_MV);
	if (ret) {
		dev_err(chip->dev, "engage: chip engage failed (%d)\n", ret);
		goto err_uninhibit;
	}

	chip->fg = power_supply_get_by_name("sm5714-fuelgauge");
	chip->pump_engaged = true;
	chip->monitor_ticks = 0;
	schedule_delayed_work(&chip->pump_monitor, msecs_to_jiffies(1000));
	dev_info(chip->dev,
		 "pump ENGAGED: IBUSLIM=%dmA VBATREG=%dmV -- monitoring (max %d s)\n",
		 SM5440_ENGAGE_IBUS_MA, SM5440_ENGAGE_VBAT_MV, SM5440_ENGAGE_MAX_TICKS);
	return 0;

err_uninhibit:
	sm5714_charger_inhibit_buck(false);
	return ret;
}

/*
 * Write-only sysfs bring-up trigger: "1" engages the pump (guarded), "0"
 * disengages.  Per-execution gated; never auto-engages.
 */
static ssize_t pump_test_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sm5440 *chip = dev_get_drvdata(dev);
	bool engage;
	int ret = 0;

	if (kstrtobool(buf, &engage))
		return -EINVAL;

	if (engage) {
		mutex_lock(&chip->engage_lock);
		/* mutually exclusive with the Inc-2 dc_test engine run (shared pump/FG). */
		ret = (chip->pump_engaged || chip->dc_running) ? -EBUSY :
		      sm5440_pump_engage(chip);
		mutex_unlock(&chip->engage_lock);
		return ret ? ret : count;
	}

	/*
	 * Disengage: sync-cancel the monitor WITHOUT engage_lock (the monitor takes
	 * it -- holding it here would deadlock the sync-cancel), then tear down.
	 */
	cancel_delayed_work_sync(&chip->pump_monitor);
	mutex_lock(&chip->engage_lock);
	if (chip->pump_engaged)
		sm5440_pump_disengage_locked(chip);
	mutex_unlock(&chip->engage_lock);
	return count;
}
static DEVICE_ATTR_WO(pump_test);

/* ===================================================================== *
 * Increment-2: the sm_dc CC/CV engine integration.
 *
 * The 9 sm_dc_ops are thin wrappers over the Inc-1 register helpers above.
 * The engine (sm5440_direct_charger.c) is hardware-agnostic and drives the
 * SM5440 only through this vtable + the single send_power_source_msg seam,
 * which routes the engine's stepped PPS target to the SM5714 sustained PD
 * contract via sm5714_pd_request_voltage().
 * ===================================================================== */

/* op 1: ADC read.  Engine units are mV (voltages) / mA (currents). */
static int sm5440_dc_get_adc_value(struct i2c_client *i2c, u8 adc_ch)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	int val = 0;

	switch (adc_ch) {
	case SM_DC_ADC_VBAT:
		sm5440_get_vbat_mv(chip, &val);
		return val;			/* mV */
	case SM_DC_ADC_VBUS:
		sm5440_get_vbus_uv(chip, &val);
		return val / 1000;		/* uV -> mV */
	case SM_DC_ADC_IBUS:
		sm5440_get_ibus_ua(chip, &val);
		return val / 1000;		/* uA -> mA */
	case SM_DC_ADC_VOUT:
		sm5440_get_vout_mv(chip, &val);
		return val;			/* mV */
	case SM_DC_ADC_DIETEMP:
		sm5440_get_dietemp_dc(chip, &val);
		return val;			/* deci-C (logged only) */
	case SM_DC_ADC_THEM:
	case SM_DC_ADC_IBAT:		/* no IBAT ADC on this chip (device-own: 0) */
	default:
		return 0;
	}
}

/*
 * op 2: ADC mode.  The device-own ONESHOT needs a 200 ms re-poll for freshness
 * that we drop; alias ONESHOT->CONTINUOUS (every read fresh, zero cost) and keep
 * the ADC continuously enabled even on OFF so the telemetry psy + monitor keep
 * reading.  All modes -> ADCCNTL1 = enable|continuous|32-avg.
 */
static int sm5440_dc_set_adc_mode(struct i2c_client *i2c, u8 mode)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	return regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);
}

/* op 3: is the pump switching? */
static int sm5440_dc_get_charging_enable(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	return sm5440_get_op_mode(chip) == SM5440_OPMODE_CHG_ON ? 1 : 0;
}

/*
 * op 4: enable/disable the pump (op-mode + WDT).  ENHIZ stays cleared by init
 * (CNTL6=0x09).  The device-own set_charging_enable also high-Zs the VBUS input
 * (ENHIZ=1) on a disable-with-VBUS-present; we deliberately omit that here --
 * Inc-1's pump_test disengage proved op-mode CHG_OFF alone is a clean teardown
 * on this hardware, and the 11 V VBUS-OVP plus the buck-restore handoff protect
 * the path, so we do not add an untested input-isolation write on the disable.
 */
static int sm5440_dc_set_charging_enable(struct i2c_client *i2c, bool enable)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	sm5440_set_op_mode(chip, enable ? SM5440_OPMODE_CHG_ON : SM5440_OPMODE_CHG_OFF);
	sm5440_enable_wdt(chip, enable);
	return 0;
}

/*
 * op 5: program the regulation targets.  en_vbatreg=0 on gts9u -> the chip
 * VBATREG ceiling sits CV_OFFSET above the cell-V goal; IBUSLIM = ci_gl +
 * CI_OFFSET (the input-current goal plus headroom).  freq is the single 450 kHz
 * (SIOP de-rating unused in our ci_gl >= 2000 range).
 */
static int sm5440_dc_set_charging_config(struct i2c_client *i2c, u32 cv_gl, u32 ci_gl, u32 cc_gl)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	u32 vbatreg, ibuslim;

	vbatreg = cv_gl + SM5440_CV_OFFSET;
	if (ci_gl <= SM5440_TA_MIN_CURRENT)
		ibuslim = ci_gl + SM5440_CI_OFFSET * 2;
	else
		ibuslim = ci_gl + SM5440_CI_OFFSET;

	sm5440_set_ibuslim(chip, ibuslim);
	sm5440_set_vbatreg(chip, vbatreg);
	sm5440_set_freq(chip, SM5440_GTS9U_FREQ_KHZ);

	dev_info(chip->dev, "dc config: vbatreg=%umV ibuslim=%umA freq=%dkHz (cv_gl=%u ci_gl=%u)\n",
		 vbatreg, ibuslim, SM5440_GTS9U_FREQ_KHZ, cv_gl, ci_gl);
	return 0;
}

/*
 * op 6: error status (polled adaptation -- this board has no SM5440 irq-gpio,
 * so the 5 forced-cutoff faults the device-own checks in its IRQ are folded in
 * here, read from the STATUS level mirrors, not the clear-on-read INT regs).
 */
static u32 sm5440_dc_get_dc_error_status(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	u32 err = SM_DC_ERR_NONE;
	int op_mode, vbat = 0;
	u8 st[4] = { };

	if (regmap_bulk_read(chip->regmap, SM5440_REG_STATUS1, st, 4)) {
		dev_warn(chip->dev, "dc err: STATUS read failed -> retry\n");
		return SM_DC_ERR_RETRY;
	}
	op_mode = sm5440_get_op_mode(chip);

	if (op_mode == SM5440_OPMODE_CHG_OFF) {
		/* the chip force-cut charging off: identify which fault latched. */
		if (st[0] & SM5440_STATUS1_VOUTOVP)	err |= SM_DC_ERR_VOUTOVP;
		if (st[2] & SM5440_STATUS3_VBUSOVP)	err |= SM_DC_ERR_VBUSOVP;
		if (st[2] & SM5440_STATUS3_STUP_FAIL)	err |= SM_DC_ERR_STUP_FAIL;
		if (st[2] & SM5440_STATUS3_REVBLK)	err |= SM_DC_ERR_REVBLK;
		if (st[2] & SM5440_STATUS3_CFLY_SHORT)	err |= SM_DC_ERR_CFLY_SHORT;
		if (st[2] & SM5440_STATUS3_VBUSUVLO)	err |= SM_DC_ERR_VBUSUVLO;
		if (err == SM_DC_ERR_NONE)
			err = SM_DC_ERR_UNKNOWN;
		dev_err(chip->dev, "dc err: op-mode CHG_OFF, STATUS %02x:%02x:%02x:%02x -> 0x%x\n",
			st[0], st[1], st[2], st[3], err);
		return err;
	}

	/* charging active: VBUSUVLO is a transient worth a retry, not a stop. */
	if (st[2] & SM5440_STATUS3_VBUSUVLO)
		return SM_DC_ERR_RETRY;

	sm5440_get_vbat_mv(chip, &vbat);
	if (vbat < SM5440_VBAT_MIN) {
		dev_err(chip->dev, "dc err: abnormal vbat=%dmV\n", vbat);
		return SM_DC_ERR_INVAL_VBAT;
	}
	return SM_DC_ERR_NONE;
}

/*
 * op 7: which regulation loop is active.  The sm_dc_charging_loop enum values
 * equal the STATUS2 bit positions (IBUSLIM=0x80, VBATREG=0x08, THEMREG=0x02),
 * so the masked STATUS2 bits ARE the loop status.  The target_vbat<=vnow
 * override forces CV when the cell has reached the goal regardless of the bit.
 */
static int sm5440_dc_get_dc_loop_status(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	unsigned int reg = 0;
	int loop = LOOP_INACTIVE, vnow = 0;

	regmap_read(chip->regmap, SM5440_REG_STATUS2, &reg);
	sm5440_get_vbat_mv(chip, &vnow);

	if ((reg & SM5440_STATUS2_VBATREG) ||
	    (chip->dc && chip->dc->target_vbat && chip->dc->target_vbat <= vnow))
		loop = LOOP_VBATREG;
	else if (reg & SM5440_STATUS2_IBUSLIM)
		loop = (reg & SM5440_STATUS2_THEM_REG) ? LOOP_THEMREG : LOOP_IBUSLIM;

	return loop;
}

/*
 * op 8: THE SEAM.  Route the engine's stepped PPS target to the SM5714
 * sustained contract.  sm5714_pd_request_voltage() sets the commandable target
 * and kicks the keepalive (the sole PD requester); it returns <0 only on
 * genuine contract loss, so a SinkTxOk deferral never surfaces here as a fatal
 * SM_DC_ERR_SEND_PD_MSG -- exactly the behaviour the engine needs.
 */
static int sm5440_dc_send_power_source_msg(struct i2c_client *i2c,
					   struct sm_dc_power_source_info *ta)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	int ret;

	ret = sm5714_pd_request_voltage(ta->v, ta->c);
	if (ret)
		dev_warn(chip->dev, "dc send: PPS %umV/%umA failed (%d)\n",
			 ta->v, ta->c, ret);
	return ret;
}

/* op 9: SW-OCP trigger (async, device-faithful -- must return 0, see below). */
static int sm5440_dc_check_sw_ocp(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	schedule_delayed_work(&chip->ocp_check_work, msecs_to_jiffies(1000));
	return 0;
}

/*
 * Device-faithful SW-OCP backstop (the SM5440 has HW OCP disabled in CNTL2).
 * A sustained IBUSLIM + thermal-shutdown alarm over 3 s == a real over-current;
 * report it out-of-band (NOT synchronously from check_sw_ocp, whose <0 return
 * the engine treats as "the op itself failed" and bails WITHOUT stopping the
 * pump).  sm_dc_report_error_status stops the engine; the dc_monitor then sees
 * the engine left its active states and restores the buck.
 */
static void sm5440_ocp_check_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, ocp_check_work.work);
	unsigned int s2, s3;
	int i;

	for (i = 0; i < 3; i++) {
		if (regmap_read(chip->regmap, SM5440_REG_STATUS2, &s2) ||
		    regmap_read(chip->regmap, SM5440_REG_STATUS3, &s3))
			return;
		if ((s2 & SM5440_STATUS2_IBUSLIM) && (s3 & SM5440_STATUS3_THEMSHDN_ALM)) {
			dev_err(chip->dev, "SW-OCP: IBUSLIM+THEMSHDN (i=%d)\n", i);
			msleep(1000);
		} else {
			return;
		}
	}
	dev_err(chip->dev, "SW-OCP confirmed -> stopping engine (IBUSOCP)\n");
	if (chip->dc)
		sm_dc_report_error_status(chip->dc, SM_DC_ERR_IBUSOCP);
}

static const struct sm_dc_ops sm5440_dc_ops = {
	.get_adc_value		= sm5440_dc_get_adc_value,
	.set_adc_mode		= sm5440_dc_set_adc_mode,
	.get_charging_enable	= sm5440_dc_get_charging_enable,
	.set_charging_enable	= sm5440_dc_set_charging_enable,
	.set_charging_config	= sm5440_dc_set_charging_config,
	.get_dc_error_status	= sm5440_dc_get_dc_error_status,
	.get_dc_loop_status	= sm5440_dc_get_dc_loop_status,
	.send_power_source_msg	= sm5440_dc_send_power_source_msg,
	.check_sw_ocp		= sm5440_dc_check_sw_ocp,
};

/*
 * Common teardown (idempotent): stop the engine, restore the 10 V sustain
 * baseline, re-allow the buck, release the FG handle.  Called from dc_test=0,
 * the done callback, and the monitor's auto-restore -- so a topoff/fault/WDT
 * stop can never leave the buck inhibited + pump off = cell not charging.
 */
static void sm5440_dc_teardown(struct sm5440 *chip)
{
	if (chip->dc)
		sm_dc_stop_charging(chip->dc);
	sm5714_pd_request_voltage(10000, 3000);	/* restore the 10 V sustain baseline */
	sm5714_charger_inhibit_buck(false);	/* re-allow the buck to charge */
	if (chip->fg) {
		power_supply_put(chip->fg);
		chip->fg = NULL;
	}
	chip->dc_running = false;
}

/*
 * LOG-ONLY engine monitor.  Unlike Inc-1's pump_monitor it does NOT auto-
 * disengage on a tick count (that would rip control from the engine); it only
 * logs telemetry (incl. the headline FG cell-current) and, if the engine has
 * left its active states (topoff/fault/WDT-drop), runs the unconditional
 * restore.  Takes engage_lock; dc_test=0 sync-cancels it WITHOUT the lock.
 */
static void sm5440_dc_monitor_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, dc_monitor.work);
	int state, vbus = 0, ibus = 0, vout = 0, vbat = 0, die = 0, fg_ua = 0, opmode;
	union power_supply_propval pv;

	mutex_lock(&chip->engage_lock);
	if (!chip->dc_running) {
		mutex_unlock(&chip->engage_lock);
		return;
	}

	state = chip->dc ? sm_dc_get_current_state(chip->dc) : SM_DC_CHG_OFF;
	opmode = sm5440_get_op_mode(chip);
	sm5440_get_vbus_uv(chip, &vbus);
	sm5440_get_ibus_ua(chip, &ibus);
	sm5440_get_vout_mv(chip, &vout);
	sm5440_get_vbat_mv(chip, &vbat);
	sm5440_get_dietemp_dc(chip, &die);
	if (chip->fg && !power_supply_get_property(chip->fg,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pv))
		fg_ua = pv.intval;

	dev_info(chip->dev,
		 "dc[%3d] st=%d op=%d VBUS=%dmV 2xVOUT=%dmV IBUS=%dmA | cell-VBAT=%dmV FG-I=%dmA | die=%d.%dC\n",
		 chip->dc_ticks++, state, opmode, vbus / 1000, vout * 2, ibus / 1000,
		 vbat, fg_ua / 1000, die / 10, die % 10);

	/* engine left the active states (CHG_OFF/ERR/EOC) -> restore the buck. */
	if (state < SM_DC_CHECK_VBAT) {
		dev_warn(chip->dev, "dc engine stopped (state=%d) -> restoring buck\n", state);
		sm5440_dc_teardown(chip);
		mutex_unlock(&chip->engage_lock);
		return;
	}

	schedule_delayed_work(&chip->dc_monitor, msecs_to_jiffies(1000));
	mutex_unlock(&chip->engage_lock);
}

/* The engine's topoff "done" event (CV, target==float).  Stop the engine; the
 * monitor's next tick sees CHG_OFF and restores the buck. */
static void sm5440_dc_done_cb(void *ctx)
{
	struct sm5440 *chip = ctx;

	dev_info(chip->dev, "dc: charging done (topoff) -> stopping engine\n");
	if (chip->dc)
		sm_dc_stop_charging(chip->dc);
}

/*
 * Engine START.  Caller holds engage_lock and has checked !dc_running.  Reuses the
 * Inc-1 proven precondition (buck OFF -> chip sw_reset+init -> step PPS to
 * ~2*Vcell and VERIFY VBUS settled) so the engine's preset re-send is a near
 * no-op against an already-2:1-ready source, THEN hands control to the engine.
 */
static int sm5440_dc_start(struct sm5440 *chip, u32 target_ibus_ma)
{
	struct sm_dc_power_source_info ta = { };
	int vbat_mv = 0, target_mv, off_mv, vbus_mv, diff, ret;

	if (!sm5714_pd_contract_active()) {
		dev_warn(chip->dev, "dc start: no PPS contract -- arm pd_request + plug first\n");
		return -ENOTCONN;
	}

	ret = sm5714_charger_inhibit_buck(true);	/* buck OFF before pump ON */
	if (ret) {
		dev_warn(chip->dev, "dc start: buck inhibit failed (%d)\n", ret);
		return ret;
	}

	/* chip prep: sw_reset + device-faithful init + continuous ADC, NO op-mode
	 * (the engine flips op-mode itself in preset's set_charging_enable). */
	ret = sm5440_sw_reset(chip);
	if (ret)
		goto err_restore;
	sm5440_init_reg_param(chip);
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);

	/* precondition the PPS input to ~2*Vcell + IR offset, clamped to
	 * [ta_min_voltage, dc_vbus_ovp_th - 500], and verify VBUS transitioned. */
	ret = sm5440_get_vbat_mv(chip, &vbat_mv);
	if (ret || vbat_mv < 2500 || vbat_mv > 4500) {
		dev_warn(chip->dev, "dc start: implausible VBAT %dmV -- abort\n", vbat_mv);
		ret = ret ? ret : -EIO;
		goto err_restore;
	}
	off_mv = (target_ibus_ma * (SM5440_GTS9U_R_TTL_UOHM / 1000)) / 1000 + 200;
	target_mv = 2 * vbat_mv + off_mv;
	target_mv = min(target_mv, SM5440_DC_VBUS_OVP_TH - 500);
	target_mv = max(target_mv, SM5440_TA_MIN_VOLTAGE);

	ret = sm5714_pd_request_voltage(target_mv, target_ibus_ma);
	if (ret) {
		dev_warn(chip->dev, "dc start: PPS request %dmV failed (%d)\n", target_mv, ret);
		goto err_restore;
	}
	dev_info(chip->dev, "dc start: VBAT=%dmV -> PPS target=%dmV; settling 300 ms\n",
		 vbat_mv, target_mv);
	msleep(300);

	vbus_mv = 0;
	sm5440_get_vbus_uv(chip, &vbus_mv);
	vbus_mv /= 1000;
	diff = vbus_mv - target_mv;
	if (diff < 0)
		diff = -diff;
	if (diff > target_mv / 10) {
		dev_warn(chip->dev, "dc start: VBUS=%dmV not within 10%% of %dmV (PPS not transitioned) -- abort\n",
			 vbus_mv, target_mv);
		ret = -EIO;
		goto err_restore;
	}
	dev_info(chip->dev, "dc start: VBUS settled to %dmV -- handing to the engine\n", vbus_mv);

	/* TA descriptor: our 10 V APDO (pos 5); the engine ramps within it.  c_max =
	 * the step target (the PD layer clamps to the APDO's advertised max current);
	 * p_max in uW (= v_max * c_max) is a generous bound -- IBUSLIM + the APDO
	 * clamp are the real limiters. */
	ta.pdo_pos = 5;
	ta.v_max = SM5440_DC_VBUS_OVP_TH - 500;		/* 10500 mV */
	ta.c_max = target_ibus_ma;
	ta.p_max = ta.v_max * ta.c_max;

	sm_dc_set_target_vbat(chip->dc, SM5440_DC_STEP0_VBAT_MV);
	sm_dc_set_target_ibus(chip->dc, target_ibus_ma);

	chip->fg = power_supply_get_by_name("sm5714-fuelgauge");
	chip->dc_running = true;
	chip->dc_ticks = 0;

	ret = sm_dc_start_charging(chip->dc, &ta);
	if (ret) {
		dev_err(chip->dev, "dc start: sm_dc_start_charging failed (%d)\n", ret);
		sm5440_dc_teardown(chip);	/* clears dc_running, restores buck */
		return ret;
	}

	schedule_delayed_work(&chip->dc_monitor, msecs_to_jiffies(1000));
	dev_info(chip->dev, "dc ENGINE STARTED: target_vbat=%dmV target_ibus=%umA -- monitoring\n",
		 SM5440_DC_STEP0_VBAT_MV, target_ibus_ma);
	return 0;

err_restore:
	sm5714_charger_inhibit_buck(false);
	sm5714_pd_request_voltage(10000, 3000);
	return ret;
}

/*
 * Engine test trigger: "<mA>" starts the CC/CV engine at that input-current
 * target (e.g. 3000 then 4500); "0" stops + restores.  Per-execution gated;
 * never auto-starts.
 */
static ssize_t dc_test_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sm5440 *chip = dev_get_drvdata(dev);
	u32 target_ibus;
	int ret;

	if (kstrtou32(buf, 0, &target_ibus))
		return -EINVAL;

	if (target_ibus == 0) {		/* stop + unconditional restore */
		cancel_delayed_work_sync(&chip->dc_monitor);
		mutex_lock(&chip->engage_lock);
		if (chip->dc_running)
			sm5440_dc_teardown(chip);
		mutex_unlock(&chip->engage_lock);
		return count;
	}

	/*
	 * Lower bound is SIOP_LEV2 (1700), not ta_min_current: below it the
	 * device-own set_charging_config de-rates the switching frequency
	 * (freq_siop[]), which this port does not model -- so keep test targets in
	 * the single-450 kHz range (the documented steps are 3000 / 4500 anyway).
	 */
	if (target_ibus <= SM5440_SIOP_LEV2 || target_ibus > 5000)
		return -EINVAL;

	mutex_lock(&chip->engage_lock);
	/* mutually exclusive with the Inc-1 pump_test manual engage (shared pump/FG). */
	ret = (chip->dc_running || chip->pump_engaged) ? -EBUSY :
	      sm5440_dc_start(chip, target_ibus);
	mutex_unlock(&chip->engage_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(dc_test);

static int sm5440_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sm5440 *chip = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SM5440";
		return 0;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Silicon Mitus";
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chip->regmap, SM5440_REG_STATUS3, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SM5440_STATUS3_VBUS_POK);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return sm5440_get_vbus_uv(chip, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return sm5440_get_ibus_ua(chip, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return sm5440_get_dietemp_dc(chip, &val->intval);
	default:
		return -EINVAL;
	}
}

static enum power_supply_property sm5440_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	/* VBUS input voltage */
	POWER_SUPPLY_PROP_CURRENT_NOW,	/* IBUS input current */
	POWER_SUPPLY_PROP_TEMP,		/* die temperature */
};

static const struct power_supply_desc sm5440_psy_desc = {
	.name		= "sm5440-charge-pump",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= sm5440_props,
	.num_properties	= ARRAY_SIZE(sm5440_props),
	.get_property	= sm5440_get_property,
};

static const struct regmap_config sm5440_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SM5440_REG_DEVICEID,
	/*
	 * INT registers (0x00-0x03) are clear-on-read and the STATUS / ADC
	 * registers are live, so never cache: every read hits the hardware.
	 */
	.cache_type	= REGCACHE_NONE,
};

static int sm5440_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };
	struct sm5440 *chip;
	unsigned int devid = 0;
	int ret, i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->engage_lock);
	INIT_DELAYED_WORK(&chip->pump_monitor, sm5440_pump_monitor_work);
	INIT_DELAYED_WORK(&chip->ocp_check_work, sm5440_ocp_check_work);
	INIT_DELAYED_WORK(&chip->dc_monitor, sm5440_dc_monitor_work);

	chip->regmap = devm_regmap_init_i2c(client, &sm5440_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	/*
	 * DEVICEID gate (device-own sm5440_hw_init_config): the low nibble
	 * must read 1, retried up to 3 times, else the chip is absent.
	 */
	for (i = 0; i < 3; i++) {
		ret = regmap_read(chip->regmap, SM5440_REG_DEVICEID, &devid);
		if (!ret && (devid & SM5440_DEVICEID_MASK) == SM5440_DEVICEID_PRESENT)
			break;
		usleep_range(1000, 2000);
	}
	if (i == 3)
		return dev_err_probe(dev, -ENODEV,
				     "SM5440 not found (DEVICEID=0x%02x)\n", devid);

	chip->rev_id = SM5440_DEVICEID_REV(devid);

	/*
	 * Enable the on-chip ADC for telemetry only.  We intentionally skip
	 * the vendor's full init (protection thresholds / switching freq /
	 * op-mode) - that belongs to the charge-control path.  op-mode (CNTL5)
	 * is left untouched at its power-on CHG_OFF default, so the pump never
	 * switches.
	 */
	ret = regmap_write(chip->regmap, SM5440_REG_ADCCNTL2,
			   SM5440_ADCCNTL2_CHANNELS);
	if (ret)
		return dev_err_probe(dev, ret, "failed to select ADC channels\n");

	ret = regmap_write(chip->regmap, SM5440_REG_ADCCNTL1,
			   SM5440_ADCCNTL1_ENABLE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable ADC\n");

	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(dev);
	chip->psy = devm_power_supply_register(dev, &sm5440_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy),
				     "failed to register power supply\n");

	/* Guarded manual-engage bring-up trigger (non-fatal if it fails). */
	if (device_create_file(dev, &dev_attr_pump_test))
		dev_warn(dev, "could not create pump_test sysfs attribute\n");

	/*
	 * Increment-2: create the sm_dc CC/CV engine instance, wire its ops to this
	 * driver, and populate the device-own config (PD path).  Non-fatal: if the
	 * engine fails to come up, telemetry + the Inc-1 pump_test still work; only
	 * the closed-loop dc_test is unavailable.
	 */
	chip->dc = sm_dc_create_pd_instance("SM5440-PD-DC", client);
	if (IS_ERR(chip->dc)) {
		dev_warn(dev, "could not create sm_dc engine (%ld) -- dc_test disabled\n",
			 PTR_ERR(chip->dc));
		chip->dc = NULL;
	} else {
		chip->dc->ops = &sm5440_dc_ops;
		chip->dc->config.ta_min_current = SM5440_TA_MIN_CURRENT;
		chip->dc->config.ta_min_voltage = SM5440_TA_MIN_VOLTAGE;
		chip->dc->config.dc_min_vbat = SM5440_VBAT_MIN;
		chip->dc->config.dc_vbus_ovp_th = SM5440_DC_VBUS_OVP_TH;
		chip->dc->config.r_ttl = SM5440_GTS9U_R_TTL_UOHM;
		chip->dc->config.topoff_current = SM5440_DC_TOPOFF_MA;
		chip->dc->config.need_to_sw_ocp = 1;	/* SM5440 has no HW OCP */
		chip->dc->config.support_pd_remain = 1;	/* engine sustains the contract */
		chip->dc->config.chg_float_voltage = SM5440_DC_CHG_FLOAT_MV;
		chip->dc->config.sec_dc_name = "sm5440-charge-pump";

		if (sm_dc_verify_configuration(chip->dc)) {
			dev_warn(dev, "sm_dc verify failed -- dc_test disabled\n");
			sm_dc_destroy_instance(chip->dc);
			chip->dc = NULL;
		} else {
			sm_dc_set_done_notify(chip->dc, sm5440_dc_done_cb, chip);
			if (device_create_file(dev, &dev_attr_dc_test))
				dev_warn(dev, "could not create dc_test sysfs attribute\n");
		}
	}

	dev_info(dev, "SM5440 charge-pump telemetry ready (rev 0x%x, DEVICEID 0x%02x)%s\n",
		 chip->rev_id, devid, chip->dc ? " + sm_dc engine" : "");
	return 0;
}

static void sm5440_remove(struct i2c_client *client)
{
	struct sm5440 *chip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_pump_test);
	if (chip->dc)
		device_remove_file(&client->dev, &dev_attr_dc_test);

	/*
	 * Sync-cancel every monitor/backstop WITHOUT engage_lock (the monitors take
	 * it; holding it here would deadlock the sync-cancel).  ocp_check_work and
	 * dc_monitor are INIT'd unconditionally in probe, so cancelling them is safe
	 * even when the engine never came up.  Then one locked region tears down
	 * whichever path was live (mutually exclusive), and the engine is freed last.
	 */
	cancel_delayed_work_sync(&chip->dc_monitor);
	cancel_delayed_work_sync(&chip->ocp_check_work);
	cancel_delayed_work_sync(&chip->pump_monitor);

	mutex_lock(&chip->engage_lock);
	if (chip->dc_running)
		sm5440_dc_teardown(chip);
	if (chip->pump_engaged)
		sm5440_pump_disengage_locked(chip);
	mutex_unlock(&chip->engage_lock);

	if (chip->dc) {
		sm_dc_destroy_instance(chip->dc);
		chip->dc = NULL;
	}
}

static const struct i2c_device_id sm5440_i2c_id[] = {
	{ "sm5440" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5440_i2c_id);

static const struct of_device_id sm5440_of_match[] = {
	{ .compatible = "siliconmitus,sm5440" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5440_of_match);

static struct i2c_driver sm5440_driver = {
	.driver = {
		.name		= "sm5440-charge-pump",
		.of_match_table	= sm5440_of_match,
	},
	.probe		= sm5440_probe,
	.remove		= sm5440_remove,
	.id_table	= sm5440_i2c_id,
};
module_i2c_driver(sm5440_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5440 charge-pump telemetry driver");
MODULE_AUTHOR("omar fityan <me@omarfityan.com>");
MODULE_LICENSE("GPL");
