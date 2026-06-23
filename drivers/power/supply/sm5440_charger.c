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

struct sm5440 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *fg;	/* sm5714-fuelgauge, for the engage cross-check */
	struct delayed_work pump_monitor;
	struct mutex engage_lock;	/* serialises the sysfs engage/disengage path */
	bool pump_engaged;
	int monitor_ticks;
	u8 rev_id;
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
		ret = chip->pump_engaged ? -EBUSY : sm5440_pump_engage(chip);
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

	dev_info(dev, "SM5440 charge-pump telemetry ready (rev 0x%x, DEVICEID 0x%02x)\n",
		 chip->rev_id, devid);
	return 0;
}

static void sm5440_remove(struct i2c_client *client)
{
	struct sm5440 *chip = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_pump_test);
	/* Sync-cancel the monitor without engage_lock (it takes the lock). */
	cancel_delayed_work_sync(&chip->pump_monitor);
	mutex_lock(&chip->engage_lock);
	if (chip->pump_engaged)
		sm5440_pump_disengage_locked(chip);
	mutex_unlock(&chip->engage_lock);
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
