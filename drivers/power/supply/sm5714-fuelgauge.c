// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5714 battery fuel-gauge (read-only)
 *
 * The SM5714 combined PMIC on the Samsung Galaxy Tab S9 Ultra (SM-X910) carries
 * a battery fuel-gauge sub-block at its own i2c 7-bit address 0x71, on the same
 * I2C-master-hub bus (mainline &i2c_hub_8) as the charger (0x49) and MUIC (0x25).
 *
 * Mainline has no SM5714 MFD/fuel-gauge driver, so this minimal i2c driver binds
 * the fuel-gauge node directly and exposes a read-only POWER_SUPPLY_TYPE_BATTERY
 * so the desktop gets a real battery indicator (capacity / voltage / current /
 * temperature).  Every register address and conversion below is transcribed from
 * the device's OWN downstream driver (kernel_platform/msm-kernel/drivers/battery/
 * fuelgauge/sm5714_fuelgauge/sm5714_fuelgauge.c @ b55c881), not inferred from a
 * sister chip.
 *
 * Register access has two flavours:
 *   - direct word registers (device-id 0x00, system-status 0x10, aux-status 0x22)
 *   - measurement values live in SRAM, reached indirectly: write the target SRAM
 *     address to SRAM_RADDR (0x8C), then read the 16-bit value from SRAM_RDATA
 *     (0x8D).
 *
 * The driver is deliberately read-only.  The fuel-gauge IC is powered
 * continuously from the cell, so it retains the battery-model parameter table
 * programmed by the last vendor boot; system-status bit 12 (INIT_CHECK) reports
 * whether that table is loaded.  We therefore do not push the parameter table
 * ourselves -- we only read the measurements the already-initialised IC computes.
 * If bit 12 is clear (a cell that was fully disconnected and never re-initialised
 * by a vendor boot) the readings may be invalid; the driver warns but still
 * registers, since pushing the model table is a separate, larger task.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/power/sm5714-battery-age.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>

/*
 * Early-boot probe retry.
 *
 * The fuel gauge sits on the I2C-master-hub bus (&i2c_hub_8 / 9a0000.i2c)
 * alongside the SM5714 charger (0x49) and MUIC (0x25).  That controller
 * intermittently loses the bus while the boot is still busy:
 *
 *   geni_i2c 9a0000.i2c: Bus arbitration lost, clock line undriveable
 *   sm5714-fuelgauge 0-0071: error -ETIMEDOUT: not responding at 0x71
 *
 * The controller could not drive SCL, so the chip was never addressed -- a
 * bus-level transient, not an absent or sleeping device.  The IC is powered
 * continuously from the cell (see the file comment above) and answers normally
 * once boot settles; the same stall is ridden out on the charger's NTC read
 * (sm5714-usb-vbus.c, SM5714_TEMP_READ_FAIL_MAX).
 *
 * Treating one such failure as fatal costs far more than a missing battery
 * icon.  With no fuel-gauge power_supply registered, sm5440_read_soc() returns
 * -1 and the charge-pump auto-engage gate (sm5440_charger.c, "soc < 0")
 * declines to engage, so the device silently charges on the ~16 W buck path
 * instead of ~32 W direct charging -- for the rest of that boot.
 *
 * Retry a few times to ride out a short stall, then hand back -EPROBE_DEFER so
 * the driver core re-runs us later for a longer one.
 */
#define SM5714_FG_PROBE_RETRIES		5
#define SM5714_FG_PROBE_RETRY_MS	20

/* Direct (non-SRAM) registers */
#define SM5714_FG_REG_DEVICE_ID		0x00
#define SM5714_FG_REG_SYSTEM_STATUS	0x10
#define SM5714_FG_SYSTEM_STATUS_INIT	BIT(12)	/* model table loaded */
#define SM5714_FG_REG_AUX_STATUS	0x22
#define SM5714_FG_AUX_STATUS_SHUNT	BIT(10)	/* large current-sense shunt */
#define SM5714_FG_REG_SRAM_RADDR	0x8C	/* write target SRAM address */
#define SM5714_FG_REG_SRAM_RDATA	0x8D	/* read SRAM value */

/* SRAM addresses (read via RADDR/RDATA) */
#define SM5714_FG_SRAM_SOC		0x00	/* state of charge */
#define SM5714_FG_SRAM_OCV		0x01	/* open-circuit voltage */
#define SM5714_FG_SRAM_VBAT		0x03	/* terminal voltage */
#define SM5714_FG_SRAM_CURRENT		0x05	/* instantaneous current */
#define SM5714_FG_SRAM_CURRENT_AVG	0x09	/* averaged current (device-own
						 * SM5714_FG_ADDR_SRAM_CURRENT_AVG) */
#define SM5714_FG_SRAM_TEMPERATURE	0x07	/* die/battery temperature */
#define SM5714_FG_SRAM_AGING_RATE_FILT	0x46	/* hardware aging estimate */
#define SM5714_FG_SRAM_SOC_CYCLE	0x87	/* charge cycles, low byte */
#define SM5714_FG_SRAM_USER_RESERV_2	0x8b	/* vendor's persisted SoH ratchet */
#define SM5714_FG_SOH_MASK		0x7f	/* bit 7 is the cycle flag */
#define SM5714_FG_CYCLE_MASK		0x00ff	/* sm5714_fuelgauge.c:476 */

/*
 * Full-charge detection, transcribed from the vendor's supervisor.
 *
 * The vendor puts this in sec_battery, a supervisor mainline does not have, so
 * it lands here: this driver owns the power_supply node userspace reads, which
 * is the same role sec_battery plays downstream.  The mechanism is taken from
 * the device's own configuration and code, not chosen:
 *
 *   full_check_type = full_check_type_2nd = 2 = SEC_BATTERY_FULLCHARGED_FG_CURRENT
 *       -> the test is on fuel-gauge CURRENT           (dts:131-132)
 *   cable-info/full_check_current_1st = 0x44c = 1100 mA
 *       -> the threshold in force from probe onwards   (sec_battery.c:8571)
 *   full_check_count = 1                               (dts:133)
 *       -> one qualifying sample is enough, and any failing sample resets
 *   full_condition_type = 0x09 = NOTIMEFULL | VCELL
 *       -> 🔴 SOC (bit 2) is NOT set, so the vendor's full_condition_soc is
 *          never consulted on this device.  The gate is VOLTAGE.  (dts:171)
 *
 * The vendor's current test (sec_battery.c:2603-2605) requires the current to
 * be strictly POSITIVE and below the threshold -- the pack must still be taking
 * charge.  That is what makes this a taper test rather than a termination test,
 * and it is transcribed as-is: a terminated pack reads about -3 mA here and
 * deliberately does not qualify.
 *
 * The vendor's second stage only lowers the top-off target to
 * full_check_current_2nd and repeats the same test.  It is not transcribed
 * because this port does not drive the top-off vote: the SM5714 terminates in
 * hardware at its own 350 mA top-off, which is already below the vendor's
 * 550 mA software second stage.  What matters for what userspace sees is that
 * the vendor sets POWER_SUPPLY_STATUS_FULL at :2804 -- BEFORE the first/second
 * stage branch -- so the vendor also reports full at the 1100 mA first stage.
 */
#define SM5714_FG_FULL_CHECK_CURRENT_UA	1100000
#define SM5714_FG_FULL_CHECK_COUNT	1

/*
 * Poll interval while a charger is attached.  The vendor polls at 30 s in the
 * charging state (dts:95, battery,polling_time = <10 30 30 30 3600>, indexed by
 * charging state).  Matching it keeps the i2c cost of this check at the vendor's
 * own cadence rather than a number picked here.
 */
#define SM5714_FG_MONITOR_MS		30000

struct sm5714_fg {
	struct i2c_client *client;
	struct power_supply *psy;
	int charge_full_design_uah;	/* from monitored-battery; <=0 if absent */
	int soh_pct;			/* 1..100, or 100 if unavailable */
	struct delayed_work monitor_work;

	/*
	 * Full-charge latch.
	 *
	 * 🔴 Deliberately NOT protected by a mutex, and that is a correctness
	 * decision rather than an omission.  Clearing the latch requires
	 * power_supply_am_i_supplied(), which calls into the charger driver and
	 * takes its sv->lock; the charger's own poll worker reads this gauge's
	 * cycle count.  A lock here that were held across am_i_supplied() would
	 * complete an AB-BA cycle between the two drivers.
	 *
	 * The state is safe without one: full_check_cnt is touched only by the
	 * monitor work (a single-threaded delayed work), and full_latched is a
	 * bool whose only concurrent transitions are "worker sets it while
	 * supplied" and "reader clears it while not supplied" -- conditions that
	 * cannot both hold, so the two writers cannot fight.
	 */
	bool full_latched;
	int full_check_cnt;
};

/* Indirect SRAM read: point RADDR at the target, then read RDATA. */
static int sm5714_fg_read_sram(struct sm5714_fg *fg, u8 sram_addr)
{
	int ret;

	ret = i2c_smbus_write_word_data(fg->client, SM5714_FG_REG_SRAM_RADDR,
					sram_addr);
	if (ret < 0)
		return ret;

	return i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_SRAM_RDATA);
}

/* State of charge, in whole percent (0..100). */
static int sm5714_fg_get_capacity(struct sm5714_fg *fg, int *pct)
{
	int raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC);

	if (raw < 0)
		return raw;

	/* raw is 0.1% units after (raw * 10) >> 8; integer percent = /10 */
	*pct = clamp(((raw * 10) >> 8) / 10, 0, 100);
	return 0;
}

/* Voltage in microvolts (power_supply convention). */
static int sm5714_fg_get_voltage(struct sm5714_fg *fg, u8 sram_addr, int *uv)
{
	int raw = sm5714_fg_read_sram(fg, sram_addr);
	int mv;

	if (raw < 0)
		return raw;

	if (sram_addr == SM5714_FG_SRAM_OCV) {
		mv = (raw * 1000) >> 11;
	} else if (raw & 0x8000) {
		mv = 2700 - (((raw & 0x7fff) * 10) / 109);
	} else {
		mv = ((raw * 10) / 109) + 2700;
	}

	*uv = mv * 1000;
	return 0;
}

/*
 * Current in microamps from a given SRAM register; positive = charging.
 * The gauge exposes two: an INSTANTANEOUS register (0x05) and an AVERAGED one
 * (0x09).  Both use the identical scaling and sign convention, so they share
 * this reader -- which is exactly how the device's own sm5714_get_curr() reads
 * them (one function, both addresses, one set of shunt/sign rules).
 */
static int sm5714_fg_read_current_at(struct sm5714_fg *fg, u8 sram_addr, int *ua)
{
	int raw = sm5714_fg_read_sram(fg, sram_addr);
	int aux, ma;

	if (raw < 0)
		return raw;

	ma = ((raw & 0x7fff) * 1000) / 2044;

	aux = i2c_smbus_read_word_data(fg->client, SM5714_FG_REG_AUX_STATUS);
	if (aux >= 0 && (aux & SM5714_FG_AUX_STATUS_SHUNT))
		ma = ma * 25 / 10;	/* larger shunt -> 2.5x scale */

	if (raw & 0x8000)
		ma = -ma;

	*ua = ma * 1000;
	return 0;
}

/* Current in microamps; positive = charging, negative = discharging. */
static int sm5714_fg_get_current(struct sm5714_fg *fg, int *ua)
{
	return sm5714_fg_read_current_at(fg, SM5714_FG_SRAM_CURRENT, ua);
}

/*
 * The gauge's own AVERAGED current.  Used ONLY by the full-charge test, which
 * the device's own sec_bat_check_full() gates on current_now AND current_avg
 * together -- see the full-charge comment in sm5714_fg_monitor_work().
 */
static int sm5714_fg_get_current_avg(struct sm5714_fg *fg, int *ua)
{
	return sm5714_fg_read_current_at(fg, SM5714_FG_SRAM_CURRENT_AVG, ua);
}

/* Temperature in tenths of a degree Celsius (power_supply convention). */
static int sm5714_fg_get_temp(struct sm5714_fg *fg, int *decidegc)
{
	int raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_TEMPERATURE);
	int t;

	if (raw < 0)
		return raw;

	t = (((raw & 0x7fff) * 10) * 2989) >> 19;
	if (raw & 0x8000)
		t = -t;

	*decidegc = t;
	return 0;
}

/*
 * State of health, in whole percent.
 *
 * 🔴 THE VENDOR DOES NOT REPORT THE HARDWARE AGING REGISTER, SO NEITHER DO WE.
 * It is tempting to read AGING_RATE_FILT (0x46) and scale it -- downstream
 * sm5714_fuelgauge.c:483-514 does exactly that, `soh = soh * 100 / 2048` -- but
 * the very next lines throw the result away and the function ends with
 *
 *	soh = pre_soh;
 *
 * `pre_soh` is a SOFTWARE ratchet persisted in USER_RESERV_2 (0x8b, bits 0..6).
 * The hardware estimate is used only as a trigger to decrement it, by one, and
 * only when a cycle-count flag has toggled AND the cell is at or above a
 * reference temperature. So the vendor's health figure is deliberately
 * monotonic and temperature-gated; the raw register wanders with temperature
 * and is a number stock never displays.
 *
 * We therefore read the persisted value. It lives in cell-powered SRAM and
 * survives reboots -- the same property that lets this device keep its
 * vendor-programmed model table across a reflash -- so on hardware that has run
 * stock it holds stock's own figure.
 *
 * We deliberately do NOT maintain the ratchet. That needs SRAM writes and this
 * driver is read-only by design; a health figure that only decays while Linux
 * is running would also diverge from the vendor's across dual-boot.
 */
static int sm5714_fg_read_soh(struct sm5714_fg *fg)
{
	int raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_USER_RESERV_2);
	int soh;

	if (raw < 0)
		return raw;

	soh = raw & SM5714_FG_SOH_MASK;

	/*
	 * Guard the range rather than trust it. An uninitialised or
	 * never-written scratch word reads as 0 or 0x7f, and either would be
	 * published as a confident, wrong Battery Health. Out of range means
	 * "no usable figure", NOT "a bad battery".
	 */
	if (soh < 1 || soh > 100)
		return -ERANGE;

	return soh;
}

/*
 * Charge cycles, as the vendor counts them (sm5714_fuelgauge.c:466-480: read
 * SOC_CYCLE and keep the low byte).  This is the input that selects the aging
 * row, so it decides the charge ceiling the charger programs as well as the
 * voltages this driver calls full.
 */
static int sm5714_fg_get_cycle_count(struct sm5714_fg *fg, int *cycles)
{
	int raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_SOC_CYCLE);

	if (raw < 0)
		return raw;

	*cycles = raw & SM5714_FG_CYCLE_MASK;
	return 0;
}

/*
 * Is the full-charge latch currently in force?
 *
 * Entering the latch is the monitor work's job.  LEAVING it is done here, on
 * the read path, so that unplugging is reflected immediately rather than up to
 * one poll interval later -- a stale "Fully charged" on a pack that is visibly
 * draining is exactly the kind of confidently wrong reading this driver exists
 * to avoid.
 *
 * Loss of external power is the ONLY exit, which is the vendor's behaviour and
 * the whole point of the latch.  The vendor's recharge path (sec_battery.c:2182-2189)
 * sets charging_mode and is_recharging and pointedly does NOT clear
 * battery->status, so a vendor device holds POWER_SUPPLY_STATUS_FULL -- and
 * therefore a displayed 100% -- across the whole terminate -> relax -> recharge
 * cycle this pack performs near the top of its range.
 */
static bool sm5714_fg_full_active(struct sm5714_fg *fg)
{
	if (!READ_ONCE(fg->full_latched))
		return false;

	if (power_supply_am_i_supplied(fg->psy) > 0)
		return true;

	WRITE_ONCE(fg->full_latched, false);
	return false;
}

/*
 * The vendor's full-charge check, on the vendor's cadence.
 *
 * This exists as a poll rather than as logic inside get_property because the
 * check is stateful: it counts qualifying samples and then latches.  Driving
 * that from the read path would tie the state machine to however often
 * userspace happens to look, so a burst of reads could satisfy the count in
 * milliseconds and a quiet desktop could miss the taper window entirely.
 */
static void sm5714_fg_monitor_work(struct work_struct *work)
{
	struct sm5714_fg *fg = container_of(work, struct sm5714_fg,
					    monitor_work.work);
	const struct sm5714_age_step *age;
	int uv, ua, ua_avg, cycles = 0;

	/* No external power: the latch cannot be entered and must not persist. */
	if (power_supply_am_i_supplied(fg->psy) <= 0) {
		if (READ_ONCE(fg->full_latched)) {
			WRITE_ONCE(fg->full_latched, false);
			power_supply_changed(fg->psy);
		}
		fg->full_check_cnt = 0;
		goto resched;
	}

	if (READ_ONCE(fg->full_latched))
		goto resched;		/* already full; nothing left to decide */

	if (sm5714_fg_get_voltage(fg, SM5714_FG_SRAM_VBAT, &uv) ||
	    sm5714_fg_get_current(fg, &ua) ||
	    sm5714_fg_get_current_avg(fg, &ua_avg))
		goto resched;		/* a bad read is not evidence of anything */

	/*
	 * An unreadable cycle count leaves `cycles` at 0, which selects the
	 * new-pack row -- the highest full threshold in the table.  That is the
	 * safe direction for this test: it can only make the pack harder to call
	 * full, never easier.
	 */
	sm5714_fg_get_cycle_count(fg, &cycles);
	age = sm5714_age_step_for(cycles);

	/*
	 * The taper test needs BOTH the instantaneous and the AVERAGED current
	 * below the threshold, which is exactly what the device's own
	 * sec_bat_check_full() requires for SEC_BATTERY_FULLCHARGED_FG_CURRENT:
	 *
	 *   if ((battery->current_now > 0 && current_now < topoff_condition) &&
	 *       (battery->current_avg > 0 && current_avg < topoff_condition))
	 *
	 * 🔴 Why the average is load-bearing, and why omitting it was a real defect:
	 * SM5714_FG_FULL_CHECK_COUNT is 1 -- which is FAITHFUL, the device's own DT
	 * sets battery,full_check_count = <0x01> -- so a single qualifying sample
	 * latches.  Stock can afford that only because the averaged register is an
	 * independent, glitch-immune second witness; the average IS the debounce.
	 * With the instantaneous reading alone, one bad sample is sufficient.
	 *
	 * Measured on-device before this change: the latch fired on a "4 mA taper"
	 * at an instant when the very same function returned 2505 mA, and when six
	 * consecutive reads afterwards returned 2031 mA identically.  A raw SRAM
	 * value of ~8 where ~5000 was expected -- a read landing mid-update.  The
	 * pack was declared Full at 100 % while absorbing 2 A, and the pump was
	 * disengaged with its input current still well above the topoff threshold.
	 *
	 * So the fix is NOT to raise the count away from the vendor's 1; it is to
	 * supply the second condition the vendor's test always had.
	 */
	if (uv >= age->full_mv * 1000 &&
	    ua > 0 && ua < SM5714_FG_FULL_CHECK_CURRENT_UA &&
	    ua_avg > 0 && ua_avg < SM5714_FG_FULL_CHECK_CURRENT_UA)
		fg->full_check_cnt++;
	else
		fg->full_check_cnt = 0;

	if (fg->full_check_cnt >= SM5714_FG_FULL_CHECK_COUNT) {
		fg->full_check_cnt = 0;
		WRITE_ONCE(fg->full_latched, true);
		dev_info(&fg->client->dev,
			 "battery full: %d mV >= %d mV, %d mA taper (avg %d mA) (cycles %d)\n",
			 uv / 1000, age->full_mv, ua / 1000, ua_avg / 1000, cycles);
		power_supply_changed(fg->psy);
	}

resched:
	schedule_delayed_work(&fg->monitor_work,
			      msecs_to_jiffies(SM5714_FG_MONITOR_MS));
}

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int ret, ua, pct, full_uah;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		/*
		 * While the full latch is in force the vendor reports 100 and
		 * nothing else: sec_battery.c:6381-6386 overwrites the gauge's
		 * value whenever status is FULL, with the sole exception of an
		 * EU eco-recharge mode this device does not configure
		 * (`eu_eco_rechg` is absent from its device tree).
		 *
		 * This is what the raw SoC cannot do on its own.  The gauge's
		 * number keeps moving as the pack relaxes and recharges near the
		 * top of its range, so a device that publishes it directly walks
		 * back down from 100 while sitting on the charger.  The vendor
		 * holds the displayed value because the vendor holds the STATE.
		 */
		if (sm5714_fg_full_active(fg)) {
			val->intval = 100;
			return 0;
		}
		return sm5714_fg_get_capacity(fg, &val->intval);
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		return sm5714_fg_get_cycle_count(fg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return sm5714_fg_get_voltage(fg, SM5714_FG_SRAM_VBAT,
					     &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		return sm5714_fg_get_voltage(fg, SM5714_FG_SRAM_OCV,
					     &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return sm5714_fg_get_current(fg, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return sm5714_fg_get_temp(fg, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		/*
		 * The latch outranks the current reading, which is the whole
		 * mechanism: the vendor keeps reporting FULL while the pack
		 * relaxes and takes recharge current near the top of its range,
		 * because it never clears the state (sec_battery.c:2182-2189).
		 * Testing the current first would report CHARGING every time the
		 * hardware topped the pack back up, and the desktop would
		 * oscillate between "Fully charged" and "Charging" for as long as
		 * it stayed plugged in.
		 */
		if (sm5714_fg_full_active(fg)) {
			val->intval = POWER_SUPPLY_STATUS_FULL;
			return 0;
		}
		ret = sm5714_fg_get_current(fg, &ua);
		if (ret)
			return ret;
		if (ua > 30000) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
		if (ua < -30000) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return 0;
		}
		/*
		 * Current has settled near zero with no full latch in force.
		 * That is either a pack the charger has stopped pushing into
		 * before the latch was earned, or simply an idle pack.
		 *
		 * 🔴 This branch used to declare FULL here on a hard-coded
		 * `pct >= 95`.  That constant was invented rather than extracted,
		 * and it was wrong twice over: the pack terminated at SoC 94 so
		 * it could not fire, and the termination SoC is not even stable
		 * (94 one cycle, 97-98 the one before).  Full is now decided by
		 * the vendor's own voltage-and-taper test in the monitor work,
		 * and this branch reports what it can actually justify.
		 */
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;

	/*
	 * The three capacity properties below are what upower needs to produce a
	 * Battery Health figure (energy-full / energy-full-design) and a
	 * time-to-empty estimate (energy / energy-rate). Without them it reports
	 * "energy-full-design: 0 Wh", health 0 %, and "Estimating..." forever,
	 * even though percentage and charge rate are perfectly good -- which is
	 * what made this look like a desktop bug for several sessions.
	 *
	 * They are reported only when the design capacity is actually known.
	 * -ENODATA leaves the sysfs attribute failing, which upower reads as
	 * absent; publishing a zero instead would look like a dead battery.
	 */
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (fg->charge_full_design_uah <= 0)
			return -ENODATA;
		val->intval = fg->charge_full_design_uah;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (fg->charge_full_design_uah <= 0)
			return -ENODATA;
		val->intval = fg->charge_full_design_uah / 100 * fg->soh_pct;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (fg->charge_full_design_uah <= 0)
			return -ENODATA;
		/*
		 * Derived from the same percentage the CAPACITY property reports,
		 * latch included.  upower publishes both -- percentage from
		 * CAPACITY, energy from CHARGE_NOW -- and a desktop showing
		 * "100%" beside an energy figure that says 96% of full is the
		 * kind of internal disagreement that reads as a broken gauge.
		 */
		if (sm5714_fg_full_active(fg)) {
			pct = 100;
		} else {
			ret = sm5714_fg_get_capacity(fg, &pct);
			if (ret)
				return ret;
		}
		full_uah = fg->charge_full_design_uah / 100 * fg->soh_pct;
		val->intval = full_uah / 100 * pct;
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property sm5714_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
};

static const struct power_supply_desc sm5714_fg_desc = {
	.name		= "sm5714-fuelgauge",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= sm5714_fg_props,
	.num_properties	= ARRAY_SIZE(sm5714_fg_props),
	.get_property	= sm5714_fg_get_property,
};

static int sm5714_fg_probe(struct i2c_client *client)
{
	struct power_supply_battery_info *info;
	struct power_supply_config cfg = { };
	struct sm5714_fg *fg;
	int id, status, try, soh, aging, reserv;

	fg = devm_kzalloc(&client->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->client = client;

	/*
	 * Confirm the chip answers at this address before registering, riding
	 * out an early-boot bus stall (see SM5714_FG_PROBE_RETRIES above).
	 */
	for (try = 0; ; try++) {
		id = i2c_smbus_read_word_data(client, SM5714_FG_REG_DEVICE_ID);
		if (id >= 0)
			break;
		if (try == SM5714_FG_PROBE_RETRIES)
			return dev_err_probe(&client->dev, -EPROBE_DEFER,
					     "not responding at 0x%02x after %d tries (%pe); deferring\n",
					     client->addr, try + 1, ERR_PTR(id));
		msleep(SM5714_FG_PROBE_RETRY_MS);
	}
	if (try)
		dev_info(&client->dev,
			 "answered at 0x%02x after %d retry(s) -- early-boot bus stall\n",
			 client->addr, try);

	status = i2c_smbus_read_word_data(client, SM5714_FG_REG_SYSTEM_STATUS);
	if (status >= 0 && !(status & SM5714_FG_SYSTEM_STATUS_INIT))
		dev_warn(&client->dev,
			 "fuel-gauge model table not initialised (status=0x%04x); readings may be invalid\n",
			 status);

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(&client->dev);

	fg->psy = devm_power_supply_register(&client->dev, &sm5714_fg_desc,
					     &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(&client->dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	/*
	 * Design capacity comes from the board's monitored-battery node, not
	 * from a constant here: the SM5714 ships in several Samsung devices with
	 * different packs, so the pack value belongs to the board DT. If the
	 * node is missing, the three capacity properties simply report -ENODATA
	 * and everything else keeps working.
	 */
	fg->charge_full_design_uah = 0;
	fg->soh_pct = 100;
	if (!power_supply_get_battery_info(fg->psy, &info)) {
		if (info->charge_full_design_uah > 0)
			fg->charge_full_design_uah = info->charge_full_design_uah;
		power_supply_put_battery_info(fg->psy, info);
	}
	if (!fg->charge_full_design_uah)
		dev_info(&client->dev,
			 "no design capacity in DT; charge/health properties disabled\n");

	/*
	 * Read the vendor's persisted state-of-health once. It only changes over
	 * months, so re-reading it per sysfs poll would buy nothing and cost an
	 * i2c round trip on every desktop refresh.
	 *
	 * Both raw words are logged deliberately. This is the first time this
	 * project has looked at either, and a later session refining the health
	 * model should be able to see what the hardware estimate and the vendor's
	 * ratchet actually said on a known-good boot -- rather than re-deriving
	 * it from a device whose values have since moved.
	 */
	soh = sm5714_fg_read_soh(fg);
	aging = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_AGING_RATE_FILT);
	reserv = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_USER_RESERV_2);
	if (soh > 0) {
		fg->soh_pct = soh;
	} else {
		dev_info(&client->dev,
			 "no usable stored state-of-health (%pe); reporting full health\n",
			 ERR_PTR(soh));
	}
	dev_info(&client->dev,
		 "state-of-health %d%% (stored 0x%04x, hw aging estimate 0x%04x -> %d%%)\n",
		 fg->soh_pct, reserv, aging,
		 aging >= 0 ? aging * 100 / 2048 : -1);

	i2c_set_clientdata(client, fg);

	/*
	 * Start the full-charge monitor.  Scheduled immediately rather than after
	 * one interval so a device that boots already plugged in and tapering
	 * gets its first look at the charge state now, not 30 s later.
	 *
	 * ⚠️ A pack that boots already TERMINATED will not latch on this first
	 * look, and that is the vendor's behaviour rather than a gap being
	 * papered over: the test requires positive taper current, which a
	 * terminated pack does not have.  The latch is earned on the next
	 * recharge, which this pack performs on its own near the top of its
	 * range -- the hardware tops it back up whenever the cell falls to the
	 * charger's recharge threshold, so the taper window recurs without any
	 * user action.  Until then the reported capacity is the gauge's own,
	 * which at the vendor's charge ceiling already sits at or near 100.
	 */
	INIT_DELAYED_WORK(&fg->monitor_work, sm5714_fg_monitor_work);
	schedule_delayed_work(&fg->monitor_work, 0);

	dev_info(&client->dev,
		 "SM5714 fuel-gauge ready (id=0x%04x, status=0x%04x)\n",
		 id, status);
	return 0;
}

static void sm5714_fg_remove(struct i2c_client *client)
{
	struct sm5714_fg *fg = i2c_get_clientdata(client);

	/*
	 * Before devm unwinds the power_supply the work dereferences. A remove
	 * callback runs ahead of devm cleanup, which is why this is not itself
	 * a devm action.
	 */
	cancel_delayed_work_sync(&fg->monitor_work);
}

static const struct of_device_id sm5714_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_fg_of_match);

static struct i2c_driver sm5714_fg_driver = {
	.driver = {
		.name = "sm5714-fuelgauge",
		.of_match_table = sm5714_fg_of_match,
	},
	.probe = sm5714_fg_probe,
	.remove = sm5714_fg_remove,
};
module_i2c_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("SM5714 battery fuel-gauge (read-only)");
MODULE_AUTHOR("ubuntu-tab project");
MODULE_LICENSE("GPL");
