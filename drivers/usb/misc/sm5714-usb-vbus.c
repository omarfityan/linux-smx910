// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5714 USB host (OTG) VBUS + data-path enabler
 *
 * The SM5714 combined PMIC owns both halves of the USB-C host data path on
 * boards that wire VBUS through it (e.g. Samsung Galaxy Tab S9 Ultra, SM-X910):
 *
 *   - charger sub-block @ i2c 0x49 : the 5 V OTG boost that *sources* VBUS
 *   - MUIC    sub-block @ i2c 0x25 : the manual com switch that routes the
 *                                    USB2 D+/D- lines to the controller
 *
 * Mainline has no SM5714 MFD/charger/MUIC driver, so this minimal i2c driver
 * binds the charger node and drives both sub-blocks directly to bring up host
 * mode. Enabling host mode performs two register sequences (both transcribed
 * from the device's own downstream driver, not inferred from a sister chip):
 *
 *   MUIC  0x06 (MANUAL_SW) = 0x89  manual mode (bit7) + D+/D- routed to USB (0x09)
 *   CHG   0x23 (BSTCNTL1)  = 0x46  OTG boost: 5.1 V out / 900 mA current limit
 *   CHG   0x14 (CNTL2)     = 0x07  OP_MODE = USB_OTG (engages the boost, VBUS on)
 *
 * Host mode is OFF by default: the SM5714 cannot source VBUS and charge the
 * battery simultaneously (single VBUS path), so leaving the boost on at all
 * times would prevent the device from charging while running. Enable on demand
 * with the "host_vbus" sysfs attribute, or set "siliconmitus,vbus-always-on"
 * in DT to bring host mode up automatically at probe.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "sm5714-usb-vbus.h"

/* MUIC sub-block: secondary i2c address + the manual-switch register */
#define SM5714_MUIC_I2C_ADDR		0x25
#define SM5714_MUIC_REG_MANUAL_SW	0x06
#define SM5714_MUIC_MANSW_USB		0x89	/* bit7 manual | DM/DP -> USB */
#define SM5714_MUIC_MANSW_OPEN		0x00	/* bit7=0 -> automatic mode: the
						 * MUIC autonomously routes D+/D-
						 * (USB data for SDP/CDP -> gadget,
						 * charging otherwise) */

/* Charger sub-block: this driver binds its i2c client (0x49) */
#define SM5714_CHG_REG_CNTL1		0x13
#define SM5714_CHG_CNTL1_ENQ4FET	BIT(3)	/* battery charge FET (Q4) enable */
#define SM5714_CHG_REG_CNTL2		0x14
#define SM5714_CHG_CNTL2_OPMODE_MASK	0x0f	/* OP_MODE field (bits[3:0]) */
#define SM5714_CHG_CNTL2_OTG		0x07	/* OP_MODE = USB_OTG */
#define SM5714_CHG_CNTL2_OFF		0x05	/* OP_MODE = CHG_ON_VBUS (boost off) */
#define SM5714_CHG_REG_BSTCNTL1		0x23
#define SM5714_CHG_BSTCNTL1_OTG		0x46	/* 5.1 V / 900 mA OTG boost */
#define SM5714_CHG_REG_STATUS3		0x0f
#define SM5714_CHG_STATUS3_OTGFAIL	BIT(2)
#define SM5714_CHG_REG_STATUS1		0x0d
#define SM5714_CHG_STATUS1_VBUSPOK	BIT(0)	/* valid charger VBUS present */
#define SM5714_CHG_REG_VBUSCNTL		0x15	/* input-current limit */
#define SM5714_CHG_VBUSCNTL_MASK	0x7f	/* offset = (mA - 100) / 25 */
#define SM5714_CHG_VBUSCNTL_500MA	16	/* (500 - 100) / 25: inrush floor */
#define SM5714_CHG_REG_FACTORY1		0x25
#define SM5714_CHG_FACTORY1_FACMODE	BIT(0)	/* factory mode: leave VBUSCNTL alone */

/* MUIC BC1.2 attached-charger classification (DEVICETYPE1) */
#define SM5714_MUIC_REG_DEVTYPE1	0x07
#define SM5714_MUIC_DEVTYPE1_LO_TA	BIT(7)	/* low-current TA */
#define SM5714_MUIC_DEVTYPE1_QC20	BIT(6)	/* QC2.0 TA (DCP-class at 5V) */
#define SM5714_MUIC_DEVTYPE1_AFC	BIT(5)	/* Samsung AFC TA (DCP-class at 5V) */
#define SM5714_MUIC_DEVTYPE1_U200	BIT(4)	/* unofficial 200K TA */
#define SM5714_MUIC_DEVTYPE1_CDP	BIT(3)	/* charging downstream port */
#define SM5714_MUIC_DEVTYPE1_DCP	BIT(2)	/* dedicated wall charger */
#define SM5714_MUIC_DEVTYPE1_SDP	BIT(1)	/* PC USB port */

/*
 * MUIC AFC (Samsung Adaptive Fast Charging) high-voltage negotiation block.  AFC
 * is a Samsung-proprietary protocol the MUIC speaks over the D+/D- data lines: it
 * sends a serial "ping" requesting a VBUS level and an AFC-capable charger steps
 * VBUS up in response.  Stepping 5 V -> 9 V is the rung-2 fast-charge lever: the
 * SM5714 is a buck charger, and from a 5 V input the cell cannot reach its full
 * charge current (cable IR-drop pulls VBUS toward the AICL fold-back reference); at
 * 9 V the same cell power draws far less input current, so the cell-side limit
 * becomes the only cap.  All register addresses/bits/the request byte are
 * transcribed from the device's own downstream sm5714-muic-afc.c, and the exact
 * 5V->8.8V negotiation was validated by hand on this charger before this code.
 *
 * The HV-capable TRIGGER is DEVICETYPE2 bit6 (AFC_TA_ATTACHED), which the MUIC sets
 * on attach (matching downstream sm5714_hv_muic_init_detect's `dev2 & 0x40` gate).
 * DEVTYPE1 bit5 (AFC) only sets AFTER a successful negotiation, so it is NOT the
 * right trigger for the first request.
 */
#define SM5714_MUIC_REG_INT2		0x02	/* AFC event status (read clears it) */
#define SM5714_MUIC_INT2_AFC_ACCEPTED	BIT(1)	/* TA accepted the AFC request */
#define SM5714_MUIC_INT2_VBUS_UPDATE	BIT(2)	/* a VBUS measurement has latched */
#define SM5714_MUIC_INT2_AFC_ERROR	BIT(5)	/* AFC ping errored (non-AFC TA / parity) */
#define SM5714_MUIC_REG_DEVTYPE2	0x08
#define SM5714_MUIC_DEVTYPE2_AFC_TA	BIT(6)	/* HV-capable TA attached (the AFC trigger) */
#define SM5714_MUIC_REG_AFCCNTL		0x09
#define SM5714_MUIC_AFCCNTL_ENAFC	BIT(0)	/* 1 -> send the AFC ping */
#define SM5714_MUIC_AFCCNTL_VBUS_READ	BIT(3)	/* 1 -> latch a VBUS measurement */
#define SM5714_MUIC_REG_AFCTXD		0x0a	/* request byte: hi nibble = V-5, lo = current */
#define SM5714_MUIC_AFCTXD_9V_1_65A	0x46	/* request 9 V / 1.65 A */
#define SM5714_MUIC_REG_VBUS_VOLTAGE	0x0c	/* measured VBUS, x100 mV (after VBUS_READ) */
#define SM5714_MUIC_REG_VBUS		0x3b
#define SM5714_MUIC_VBUS_VALID		BIT(2)

/*
 * Device-own input-current limits (gts9u sec_battery cable-info table): SDP must
 * stay <= the USB-spec 500 mA; CDP = 1500 mA; every wall-charger class (DCP /
 * unofficial TA / AFC / QC at 5V) = the 1800 mA TA default.  These are ceilings
 * -- the SM5714 hardware AICL (AICLEN, CNTL1 0x13 bit6) folds the draw back to
 * what the adapter actually supplies, so a too-optimistic ceiling is safe.
 */
#define SM5714_INPUT_CURRENT_SDP	500
#define SM5714_INPUT_CURRENT_CDP	1500
#define SM5714_INPUT_CURRENT_TA		1800

/*
 * Cell charge current (CHGCNTL2 reg 0x18, fast-charge field, whole byte):
 * value = 7 + (uA - 109375) / 15625.  The device-own default_charging_current
 * for a wall charger (gts9u cable-info) is 2100 mA; the hardware power-on
 * default is 500 mA.  We do NOT run stock's full thermal step-charging, so the
 * 2100 mA value is applied only with a battery-temperature read and is
 * throttled back to the 500 mA floor when the cell runs hot (hysteresis).
 */
#define SM5714_CHG_REG_CHGCNTL2		0x18
#define SM5714_CHARGE_CURRENT_MIN	500	/* safe floor (hw default) */
#define SM5714_CHARGE_CURRENT_TA	2100	/* device-own (0x834) */
#define SM5714_CHGCNTL2_MAX_OFFSET	134	/* register cap == 2100 mA */

/*
 * Cell temperature window for the raised charge current (units: 0.1 C from the
 * fuel gauge).  Full current only inside ~15..40 C, with hysteresis out to
 * 10..45 C; throttle to the 500 mA floor when the cell is HOT (thermal stress)
 * or COLD (fast charge below ~10 C risks lithium plating -- permanent damage).
 * Readings outside the plausible range are treated as a failed read (-> floor).
 */
#define SM5714_BATT_TEMP_HOT		450	/* >=45.0 C: throttle current (enter) */
#define SM5714_BATT_TEMP_HOT_OK		400	/* <=40.0 C: current throttle cleared */
#define SM5714_BATT_TEMP_COLD		100	/* <=10.0 C: throttle current (enter) */
#define SM5714_BATT_TEMP_COLD_OK	150	/* >=15.0 C: current throttle cleared */
#define SM5714_BATT_TEMP_VALID_LO	-300	/* < -30.0 C: implausible read */
#define SM5714_BATT_TEMP_VALID_HI	800	/* >  80.0 C: implausible read */

/*
 * Hard charge-cut stop-lines.  Outside these the FET is opened, not merely
 * throttled.  Transcribed from the device's own battery layer, which cuts the
 * charge (BUCK_OFF) at the overheat (50.0 C, wire_warm_overheat_thresh) and
 * cold (0.0 C, wire_cold_cool3_thresh) thresholds; below 0 C fast charge plates
 * lithium (permanent cell damage).  A wide re-arm dead-band prevents the cut
 * from oscillating against the inner current-throttle band above.
 */
#define SM5714_BATT_TEMP_HOT_CUT	500	/* >=50.0 C: cut charging (enter) */
#define SM5714_BATT_TEMP_HOT_CUT_OK	450	/* <=45.0 C: overheat cut cleared */
#define SM5714_BATT_TEMP_COLD_CUT	0	/* <=0.0 C: cut charging (enter) */
#define SM5714_BATT_TEMP_COLD_CUT_OK	50	/* >=5.0 C: cold cut cleared */

/* poll period: no IRQ wired, so re-evaluate the attached charger periodically */
#define SM5714_CHG_POLL_MS		3000

/*
 * AFC 9V negotiation parameters.  The handshake is attempted once per charger
 * attach when the MUIC flags an HV-capable TA, with a few retries to cover a
 * transient ping error before giving up (and staying at 5 V -- never a regression).
 * Each attempt: drop the input current, request 9 V, poll INT2 for ACCEPTED/ERROR,
 * then confirm VBUS read back inside [7, 10] V -- the device-own +1/-2 V window
 * around the 9 V request (real AFC chargers settle ~8.8 V under cable drop, as this
 * one did).  The ~1 s handshake runs under the poll worker's lock, which correctly
 * serialises it against the Type-C role switch rather than racing it.
 */
#define SM5714_AFC_9V_MIN_MV		7000
#define SM5714_AFC_9V_MAX_MV		10000
#define SM5714_AFC_PREPARE_MS		300	/* settle after the input-current drop */
#define SM5714_AFC_POLL_TRIES		10	/* INT2 polls for ACCEPTED/ERROR */
#define SM5714_AFC_POLL_MS		50
#define SM5714_AFC_VBUS_TRIES		5	/* INT2 polls for the VBUS measurement */
#define SM5714_AFC_MAX_TRIES		3	/* negotiation attempts before giving up */

/* AFC 9V negotiation state -- one-shot per charger attach, reset on detach. */
enum sm5714_afc_state {
	SM5714_AFC_IDLE,	/* not yet attempted (also the reset-on-detach state) */
	SM5714_AFC_DONE,	/* 9 V negotiated and confirmed */
	SM5714_AFC_FAILED,	/* gave up after retries; stay at 5 V */
};

struct sm5714_vbus {
	struct i2c_client *chg;		/* 0x49 - the bound device */
	struct i2c_client *muic;	/* 0x25 - secondary client */
	struct iio_channel *batt_therm;	/* cell NTC on the gen3 PMIC ADC */
	struct mutex lock;
	bool enabled;
	bool charge_throttled;		/* hot/cold current-throttle hysteresis */
	bool charge_cut;		/* hot/cold FET-cut hysteresis */
	enum sm5714_afc_state afc_state;	/* AFC 9V negotiation (per attach) */
	int afc_tries;			/* negotiation attempts this attach */
	bool afc_inhibited;		/* PD-RX up: skip AFC (set by the role/tcpm driver) */
	bool afc_inhibited_pump;	/* pump owns the contract: skip AFC (set by the SM5440 driver) */
	bool buck_inhibited;		/* pump engaged: keep the cell FET open (set by the SM5440 driver) */
	struct power_supply *psy;	/* charger online indicator */
	struct delayed_work charger_work;	/* input-current management */
};

/*
 * Arm (close) or disarm (open) the battery charge FET, ENQ4FET = CNTL1 bit3.
 * The SM5714 charger only delivers current into the cell when this FET is
 * closed.  The bootloader closes it once at power-on, but in USB-C dual-role
 * mode nothing re-arms it after the Type-C role driver selects the sink role,
 * so the charger sits in CHG_ON op_mode delivering nothing while the battery
 * drains -- this RMW is what actually turns charging on in device mode.
 *
 * On the open->closed transition we hold the input-current limit at the 500 mA
 * floor while the FET closes, to damp the inrush peak into a depleted cell, as
 * the device-own chg_set_enq4fet() does.  In the normal poll-worker path the
 * FET is armed BEFORE the input limit is raised, so VBUSCNTL is already at its
 * 500 mA default and the cap/settle is a no-op; it only does real work on a
 * re-detect where the limit was already raised.  Caller must hold sv->lock.
 *
 * The charge watchdog (WDTCNTL 0x22) is deliberately left untouched: it is
 * disabled at power-on (read back as 0x22=0x04 on this device) and stock only
 * enables it to guard its active SW step-charging loop, which this driver does
 * not run.  The real charge terminator is the charger's own hardware auto-stop,
 * confirmed programmed by the bootloader on this device: CHGCNTL4 (0x1A) bit6
 * (autostop) = 1 with batreg <= the device-own 4.44 V float (read back 4.38 V;
 * it may vary across boots, e.g. battery-care, but stays cell-safe well under
 * the 4.62 V cap), and CHGCNTL5 (0x1B) top-off = 350 mA.  So no SW dead-man is
 * needed; those termination registers persist from the bootloader and are not
 * touched here (programming them in the driver is a noted robustness follow-up).
 */
static void sm5714_set_charge_fet(struct sm5714_vbus *sv, bool arm)
{
	int cntl1, lim, off;

	cntl1 = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_CNTL1);
	if (cntl1 < 0)
		return;
	if (arm == !!(cntl1 & SM5714_CHG_CNTL1_ENQ4FET))
		return;				/* already in the requested state */

	if (!arm) {
		i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CNTL1,
					  cntl1 & ~SM5714_CHG_CNTL1_ENQ4FET);
		dev_info(&sv->chg->dev, "charge FET disarmed\n");
		return;
	}

	/* Hold the input limit at 500 mA across FET closure (inrush damping). */
	lim = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_VBUSCNTL);
	if (lim >= 0) {
		off = lim & SM5714_CHG_VBUSCNTL_MASK;
		if (off > SM5714_CHG_VBUSCNTL_500MA) {
			i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_VBUSCNTL,
				(lim & ~SM5714_CHG_VBUSCNTL_MASK) |
				SM5714_CHG_VBUSCNTL_500MA);
			/* device-own settle: (limit_mA - 500) / 250 ms */
			msleep((off - SM5714_CHG_VBUSCNTL_500MA) * 25 / 250);
		}
	}
	i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CNTL1,
				  cntl1 | SM5714_CHG_CNTL1_ENQ4FET);
	dev_info(&sv->chg->dev, "charge FET armed (charging enabled)\n");
}

static int sm5714_vbus_enable(struct sm5714_vbus *sv)
{
	int ret, status;

	/*
	 * Entering host mode: open the battery charge FET before sourcing VBUS,
	 * so the OTG boost can never coincide with a closed charge path.  The
	 * op_mode write below (CHG_ON -> USB_OTG) already excludes charging, but
	 * keeping the FET strictly host-off is the invariant the role-switch
	 * audit requires.  The poll worker, which is the only other caller that
	 * arms the FET, early-outs while sv->enabled, so host mode stays clean.
	 */
	sm5714_set_charge_fet(sv, false);

	/* Route D+/D- to USB first, then source VBUS. */
	ret = i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_MANUAL_SW,
					SM5714_MUIC_MANSW_USB);
	if (ret)
		return ret;

	ret = i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_BSTCNTL1,
					SM5714_CHG_BSTCNTL1_OTG);
	if (ret)
		goto unroute;

	ret = i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CNTL2,
					SM5714_CHG_CNTL2_OTG);
	if (ret)
		goto unroute;

	/*
	 * The boost is current-limited to 900 mA (BSTCNTL1 above).  That hardware
	 * cap -- with the SM5714's own back-feed/OVP protection -- is the safety
	 * net for the brief source-vs-source contention if a peripheral is swapped
	 * for a charger faster than the role driver cuts VBUS.  STATUS3 OTGFAIL is
	 * advisory only (warn, not abort).  NOTE: the downstream driver's
	 * check_usb_killer() D+/D- probe before energizing is deliberately NOT
	 * ported here -- acceptable for a controlled development target.
	 */
	status = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_STATUS3);
	if (status >= 0 && (status & SM5714_CHG_STATUS3_OTGFAIL))
		dev_warn(&sv->chg->dev,
			 "OTG boost fault (STATUS3=0x%02x) - check the charger is unplugged\n",
			 status);

	return 0;

unroute:
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_MANUAL_SW,
				  SM5714_MUIC_MANSW_OPEN);
	return ret;
}

static int sm5714_vbus_disable(struct sm5714_vbus *sv)
{
	int ret;

	/*
	 * Cut VBUS first -- this is the safety-critical write; a silent failure
	 * here would leave the 5 V boost sourcing.  Then return the MUIC to
	 * automatic.  The CNTL2 result is propagated so the caller does not record
	 * the boost as off when it may still be live.
	 */
	ret = i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CNTL2,
					SM5714_CHG_CNTL2_OFF);
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_MANUAL_SW,
				  SM5714_MUIC_MANSW_OPEN);
	return ret;
}

/*
 * Single-instance hook: the SM5714 Type-C role driver (sm5714-typec, bound to
 * the PDIC at i2c 0x33 on the sibling bus) detects host vs device from CC_STATUS
 * and calls in here to source / cut the OTG VBUS + data routing.  Host VBUS lives
 * in the charger sub-block this driver owns, so the role driver reaches it through
 * this exported helper rather than poking 0x49 itself.
 */
static DEFINE_MUTEX(sm5714_vbus_instance_lock);
static struct sm5714_vbus *sm5714_vbus_instance;

/*
 * SM5440 direct-charge notify (see sm5714-usb-vbus.h).  Stored module-static and
 * guarded by instance_lock: the charging worker calls it at its common exit AFTER
 * dropping sv->lock, so the only lock held across the callback is instance_lock --
 * no inversion against inhibit_buck (instance_lock -> sv->lock), since the callback
 * is non-blocking (stores a flag + schedules work) and never re-enters this driver.
 */
static void (*sm5714_dc_notify)(void *ctx, bool dc_intent);
static void *sm5714_dc_notify_ctx;

void sm5714_charger_set_dc_notify(void (*notify)(void *ctx, bool dc_intent),
				  void *ctx)
{
	mutex_lock(&sm5714_vbus_instance_lock);
	sm5714_dc_notify = notify;
	sm5714_dc_notify_ctx = ctx;
	mutex_unlock(&sm5714_vbus_instance_lock);
}
EXPORT_SYMBOL_GPL(sm5714_charger_set_dc_notify);

static int sm5714_vbus_set(struct sm5714_vbus *sv, bool on)
{
	bool changed = false;
	int ret = 0;

	mutex_lock(&sv->lock);
	if (on == sv->enabled)
		goto out;

	if (on)
		ret = sm5714_vbus_enable(sv);
	else
		ret = sm5714_vbus_disable(sv);

	/*
	 * Commit the new state only when the hardware write succeeded.  On a
	 * failed cutoff sv->enabled stays true, so the early-out above does not
	 * fire next time and the caller's resync retries the cutoff instead of
	 * trusting a still-live boost to be off.
	 */
	if (!ret) {
		sv->enabled = on;
		changed = true;
		/*
		 * Role changed -> re-arm AFC.  A charger present on the return to
		 * sink mode then re-negotiates 9 V from 5 V rather than trusting a
		 * stale DONE/FAILED from before the host excursion.
		 */
		sv->afc_state = SM5714_AFC_IDLE;
		sv->afc_tries = 0;
	}
out:
	mutex_unlock(&sv->lock);

	/* host mode gates the charger-online report; refresh it on a change */
	if (changed && sv->psy)
		power_supply_changed(sv->psy);
	return ret;
}

/*
 * Source (host) or cut (device / disconnected) the OTG VBUS + data routing on
 * behalf of the Type-C role driver.  HOST -> 5 V boost on + D+/D- routed to the
 * controller; DEVICE/NONE -> boost off + MUIC back to automatic (which routes a
 * PC's data for gadget mode and charges a wall charger).  Returns -ENODEV until
 * the VBUS driver has probed.
 */
int sm5714_usb_vbus_set_host(bool on)
{
	int ret;

	mutex_lock(&sm5714_vbus_instance_lock);
	ret = sm5714_vbus_instance ? sm5714_vbus_set(sm5714_vbus_instance, on)
				   : -ENODEV;
	mutex_unlock(&sm5714_vbus_instance_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(sm5714_usb_vbus_set_host);

/*
 * Inhibit / re-allow the AFC handshake (see the header).  The charging worker
 * reads afc_inhibited under its own lock; a single WRITE_ONCE here pairs with a
 * READ_ONCE there, so no lock ordering against the worker is introduced.  The
 * worker polls every few seconds and PD comes up at the attach edge, so the flag
 * is always set well before the next AFC attempt.
 */
void sm5714_usb_vbus_inhibit_afc(bool inhibit)
{
	mutex_lock(&sm5714_vbus_instance_lock);
	if (sm5714_vbus_instance)
		WRITE_ONCE(sm5714_vbus_instance->afc_inhibited, inhibit);
	mutex_unlock(&sm5714_vbus_instance_lock);
}
EXPORT_SYMBOL_GPL(sm5714_usb_vbus_inhibit_afc);

/*
 * Independent AFC inhibit asserted by the SM5440 charge-pump for the whole
 * engaged window (see the header).  This is a SECOND, separate bit from
 * afc_inhibited (which the tcpm/role driver toggles edge-driven on PD-RX
 * up/down): the worker skips AFC if EITHER bit is set.  Keeping them disjoint
 * means a tcpm PD-RX drop on a mid-charge Hard-Reset -- which clears
 * afc_inhibited -- cannot re-arm AFC while the pump still owns the contract and
 * is mid-step.  Idempotent WRITE_ONCE; no refcount (the role driver's calls are
 * edge-driven and unbalanced, so a shared count would corrupt).
 */
void sm5714_usb_vbus_inhibit_afc_pump(bool inhibit)
{
	mutex_lock(&sm5714_vbus_instance_lock);
	if (sm5714_vbus_instance)
		WRITE_ONCE(sm5714_vbus_instance->afc_inhibited_pump, inhibit);
	mutex_unlock(&sm5714_vbus_instance_lock);
}
EXPORT_SYMBOL_GPL(sm5714_usb_vbus_inhibit_afc_pump);

/*
 * Inhibit / re-allow the buck charge path (see the header).  Unlike AFC-inhibit
 * this also acts immediately: on inhibit it disarms the cell FET so the SM5440
 * pump can take the cell without the buck fighting it (the buck-OFF-before-
 * pump-ON handoff), and the worker's FET-arm gate (which reads buck_inhibited)
 * keeps it open until re-allowed.  Lock order is instance_lock -> sv->lock; the
 * worker only ever takes sv->lock, so no inversion is introduced.
 */
int sm5714_charger_inhibit_buck(bool inhibit)
{
	struct sm5714_vbus *sv;

	mutex_lock(&sm5714_vbus_instance_lock);
	sv = sm5714_vbus_instance;
	if (sv) {
		WRITE_ONCE(sv->buck_inhibited, inhibit);
		if (inhibit) {
			mutex_lock(&sv->lock);
			sm5714_set_charge_fet(sv, false);
			mutex_unlock(&sv->lock);
		}
	}
	mutex_unlock(&sm5714_vbus_instance_lock);
	return sv ? 0 : -ENODEV;
}
EXPORT_SYMBOL_GPL(sm5714_charger_inhibit_buck);

/* Map the MUIC BC1.2 classification to its device-own input-current ceiling. */
static int sm5714_input_current_ma(u8 dev_type1)
{
	if (dev_type1 & (SM5714_MUIC_DEVTYPE1_LO_TA | SM5714_MUIC_DEVTYPE1_U200))
		return SM5714_INPUT_CURRENT_TA;		/* unofficial TA -> TA */
	if (dev_type1 & SM5714_MUIC_DEVTYPE1_CDP)
		return SM5714_INPUT_CURRENT_CDP;
	if (dev_type1 & (SM5714_MUIC_DEVTYPE1_DCP | SM5714_MUIC_DEVTYPE1_AFC |
			 SM5714_MUIC_DEVTYPE1_QC20))
		return SM5714_INPUT_CURRENT_TA;
	return SM5714_INPUT_CURRENT_SDP;		/* SDP / undetected -> spec-safe */
}

/* CHGCNTL2 fast-charge field: value = 7 + (uA - 109375) / 15625, capped. */
static u8 sm5714_chgcurr_offset(int ma)
{
	int off;

	if (ma < 110)
		return 7;
	off = 7 + (ma * 1000 - 109375) / 15625;
	if (off > SM5714_CHGCNTL2_MAX_OFFSET)
		off = SM5714_CHGCNTL2_MAX_OFFSET;	/* never exceed 2100 mA */
	return off;
}

/*
 * Read the cell temperature (0.1 C) from the external battery NTC on the gen3
 * PMIC ADC (PM8550 AMUX1, virtual channel 0x144).  This is the real cell
 * thermistor the device's own battery layer gates charging on (its DT sets
 * battery,thermal_source = ADC), NOT the SM5714 fuel-gauge die temperature: the
 * fuel-gauge IC sits in the charge-current path and self-heats well above the
 * cell while charging (measured 38-45 C on a cell flat at ~23 C), which spuriously
 * tripped the thermal throttle on a perfectly cool cell and chopped a clean 2 A
 * charge to the 500 mA floor.
 *
 * iio_read_channel_processed() returns the NTC temperature in millidegrees C (the
 * ADC driver applies the 100k-pullup thermistor curve); convert to the 0.1 C the
 * threshold logic uses.  The channel is resolved at probe so it is always valid
 * here; a transient SPMI read error fails SAFE -- the caller cuts charging for one
 * poll cycle and self-heals, exactly as with an implausible reading.
 */
static bool sm5714_read_batt_temp(struct sm5714_vbus *sv, int *temp)
{
	int milli, deci;

	/*
	 * iio_read_channel_processed() returns the channel's IIO val-type on
	 * success (IIO_VAL_INT == 1 for this temperature channel, i.e. a POSITIVE
	 * value) and a negative errno on failure -- test for the error with < 0,
	 * NOT for non-zero (a successful read is non-zero and must not be rejected).
	 */
	if (iio_read_channel_processed(sv->batt_therm, &milli) < 0)
		return false;
	deci = milli / 100;			/* millidegrees C -> 0.1 C */
	/* Reject implausible (open / shorted NTC) readings -> caller uses the floor. */
	if (deci < SM5714_BATT_TEMP_VALID_LO || deci > SM5714_BATT_TEMP_VALID_HI)
		return false;
	*temp = deci;
	return true;
}

/*
 * Read VBUS through the MUIC's measurement path: trigger a conversion (AFCCNTL
 * VBUS_READ), wait for the INT2 VBUS_UPDATE latch, read VBUS_VOLTAGE (units of
 * 100 mV), then drop the trigger.  Returns VBUS in mV, or negative on error.
 * Caller holds sv->lock.
 */
static int sm5714_afc_read_vbus_mv(struct sm5714_vbus *sv)
{
	int i, int2, vbus, raw;

	vbus = i2c_smbus_read_byte_data(sv->muic, SM5714_MUIC_REG_VBUS);
	if (vbus < 0)
		return vbus;
	if (!(vbus & SM5714_MUIC_VBUS_VALID))
		return -ENODEV;

	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_AFCCNTL,
				  SM5714_MUIC_AFCCNTL_VBUS_READ);
	for (i = 0; i < SM5714_AFC_VBUS_TRIES; i++) {
		usleep_range(5000, 5500);
		int2 = i2c_smbus_read_byte_data(sv->muic, SM5714_MUIC_REG_INT2);
		if (int2 >= 0 && (int2 & SM5714_MUIC_INT2_VBUS_UPDATE))
			break;
	}
	raw = i2c_smbus_read_byte_data(sv->muic, SM5714_MUIC_REG_VBUS_VOLTAGE);
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_AFCCNTL, 0);
	if (raw < 0)
		return raw;
	return raw * 100;
}

/*
 * Run one AFC 9V negotiation.  Drop the input current to the 500 mA floor (the
 * device-own "PREPARE" step -- the D+/D- handshake briefly disturbs VBUS, so we
 * must not be drawing heavily across it), request 9 V (AFCTXD), fire the ping
 * (ENAFC), and poll INT2 for ACCEPTED or ERROR.  On ACCEPTED, confirm VBUS actually
 * read back at ~9 V before declaring success -- an accepted-but-not-9V result is
 * treated as failure so the caller retries / stays at 5 V.  Returns 0 with VBUS at
 * 9 V, or negative.  Caller holds sv->lock; only valid host-off.
 *
 * This only ever RAISES VBUS.  The SM5714's VBUS-input OVP is a fixed silicon
 * default this driver never lowers, and stock fast-charges AFC 9V/12V through this
 * same charger, so 9 V is inside the hardware's tolerated input window.  The input
 * limit is left at the 500 mA floor on return; the poll worker raises it to the
 * charger-class ceiling immediately afterwards (now at 9 V), with hardware AICL and
 * the cell-side current cap unchanged.
 */
static int sm5714_afc_request_9v(struct sm5714_vbus *sv)
{
	int i, int2, lim, vbus_mv;
	bool accepted = false;

	/* PREPARE: hold the input current at the 500 mA floor across the ping. */
	lim = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_VBUSCNTL);
	if (lim >= 0 && (lim & SM5714_CHG_VBUSCNTL_MASK) > SM5714_CHG_VBUSCNTL_500MA)
		i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_VBUSCNTL,
			(lim & ~SM5714_CHG_VBUSCNTL_MASK) | SM5714_CHG_VBUSCNTL_500MA);
	msleep(SM5714_AFC_PREPARE_MS);

	/* Clear any stale INT2, request 9 V, then fire the ping. */
	i2c_smbus_read_byte_data(sv->muic, SM5714_MUIC_REG_INT2);
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_AFCTXD,
				  SM5714_MUIC_AFCTXD_9V_1_65A);
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_AFCCNTL,
				  SM5714_MUIC_AFCCNTL_ENAFC);

	for (i = 0; i < SM5714_AFC_POLL_TRIES; i++) {
		msleep(SM5714_AFC_POLL_MS);
		int2 = i2c_smbus_read_byte_data(sv->muic, SM5714_MUIC_REG_INT2);
		if (int2 < 0)
			continue;
		if (int2 & SM5714_MUIC_INT2_AFC_ACCEPTED) {
			accepted = true;
			break;
		}
		if (int2 & SM5714_MUIC_INT2_AFC_ERROR)
			break;
	}

	/* Stop driving the ping regardless of outcome. */
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_AFCCNTL, 0);
	if (!accepted)
		return -EIO;

	vbus_mv = sm5714_afc_read_vbus_mv(sv);
	if (vbus_mv < 0)
		return vbus_mv;
	if (vbus_mv < SM5714_AFC_9V_MIN_MV || vbus_mv > SM5714_AFC_9V_MAX_MV) {
		dev_warn(&sv->chg->dev,
			 "AFC accepted but VBUS=%d mV not ~9 V; staying low\n",
			 vbus_mv);
		return -ERANGE;
	}

	dev_info(&sv->chg->dev, "AFC negotiated 9 V (VBUS=%d mV)\n", vbus_mv);
	return 0;
}

/*
 * Charger management poll (no IRQ wired).  When host-off and an input source is
 * present, this: (a) gates charging on cell temperature -- cut the FET outside
 * the device-own stop-lines (>=50 C / <=0 C / unreadable), throttle current in
 * an inner caution band; (b) arms the charge FET (the dual-role device-mode fix)
 * in CHG_ON_VBUS op_mode; (c) raises the input-current limit (VBUSCNTL) to the
 * device-own ceiling for the MUIC-classified charger class; (d) raises the cell
 * charge current (CHGCNTL2) toward the device-own 2100 mA TA value for a real
 * charger.  When nothing is attached the FET is opened and both limits return to
 * the 500 mA floor.  Hardware AICL caps the input draw to the adapter; the
 * float-voltage / top-off / OVP charge termination is all in hardware.
 */
static void sm5714_vbus_charger_work(struct work_struct *work)
{
	struct sm5714_vbus *sv = container_of(work, struct sm5714_vbus,
					      charger_work.work);
	int status1, dev_type1, factory, cur, temp;
	int input_ma = SM5714_INPUT_CURRENT_SDP;
	int charge_ma = SM5714_CHARGE_CURRENT_MIN;
	bool charger = false;
	bool vbus = false;
	bool dc_intent = false;		/* publish to the SM5440 supervisor at the
					 * common exit; false on every early-out */
	u8 offset;

	mutex_lock(&sv->lock);

	/* In host mode we source VBUS; charger management does not apply. */
	if (sv->enabled)
		goto out;

	/* Factory mode owns the charger registers; do not fight it. */
	factory = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_FACTORY1);
	if (factory >= 0 && (factory & SM5714_CHG_FACTORY1_FACMODE))
		goto out;

	status1 = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_STATUS1);
	if (status1 < 0)
		goto out;

	vbus = !!(status1 & SM5714_CHG_STATUS1_VBUSPOK);
	if (vbus) {
		dev_type1 = i2c_smbus_read_byte_data(sv->muic,
						     SM5714_MUIC_REG_DEVTYPE1);
		if (dev_type1 >= 0) {
			input_ma = sm5714_input_current_ma(dev_type1);
			/* a source above SDP can charge faster than the default */
			charger = input_ma > SM5714_INPUT_CURRENT_SDP;
		}
	} else {
		/* input source gone -> re-arm AFC for the next charger attach */
		sv->afc_state = SM5714_AFC_IDLE;
		sv->afc_tries = 0;
	}

	/*
	 * Temperature gate.  The cell temperature decides whether we charge at
	 * all, not just how fast.  The device's own battery layer hard-cuts the
	 * charge (BUCK_OFF) past 50.0 C (overheat) and below 0.0 C (fast charge
	 * there plates lithium -- permanent damage), and refuses to charge on an
	 * unreadable temperature.  Mirror that: a missing/implausible read fails
	 * SAFE to a cut, and an inner caution band throttles current to the floor
	 * before the hard cut.  Dead-bands on both keep the state from oscillating.
	 */
	if (vbus) {
		if (!sm5714_read_batt_temp(sv, &temp)) {
			sv->charge_cut = true;		/* no temp -> do not charge */
		} else {
			if (temp >= SM5714_BATT_TEMP_HOT_CUT ||
			    temp <= SM5714_BATT_TEMP_COLD_CUT)
				sv->charge_cut = true;
			else if (temp <= SM5714_BATT_TEMP_HOT_CUT_OK &&
				 temp >= SM5714_BATT_TEMP_COLD_CUT_OK)
				sv->charge_cut = false;

			if (temp >= SM5714_BATT_TEMP_HOT ||
			    temp <= SM5714_BATT_TEMP_COLD)
				sv->charge_throttled = true;
			else if (temp <= SM5714_BATT_TEMP_HOT_OK &&
				 temp >= SM5714_BATT_TEMP_COLD_OK)
				sv->charge_throttled = false;

			if (charger)
				charge_ma = sv->charge_throttled ?
						SM5714_CHARGE_CURRENT_MIN :
						SM5714_CHARGE_CURRENT_TA;
		}
	}

	/*
	 * AFC 9V negotiation (rung-2 fast charge).  Once per attach, when the MUIC
	 * flags an HV-capable TA (DEVTYPE2 AFC_TA) and we intend to charge (source
	 * present, cell inside the safe window so not cut), ask the charger to step
	 * VBUS 5 V -> 9 V.  The ~1 s handshake runs here under sv->lock; on success the
	 * input-/cell-current writes below apply at 9 V.  A handful of failures latch
	 * AFC_FAILED and we stay at 5 V -- never a regression.  Skipped in host mode
	 * (the worker has already returned) and while charge-cut (no point at 9 V if we
	 * will not charge; a cut that arrives AFTER 9 V just opens the FET, which is
	 * safe with 9 V on VBUS).
	 */
	if (vbus && !sv->charge_cut && !READ_ONCE(sv->afc_inhibited) &&
	    !READ_ONCE(sv->afc_inhibited_pump) &&
	    sv->afc_state == SM5714_AFC_IDLE) {
		int dev_type2 = i2c_smbus_read_byte_data(sv->muic,
							 SM5714_MUIC_REG_DEVTYPE2);

		if (dev_type2 >= 0 && (dev_type2 & SM5714_MUIC_DEVTYPE2_AFC_TA)) {
			sv->afc_tries++;
			if (sm5714_afc_request_9v(sv) == 0)
				sv->afc_state = SM5714_AFC_DONE;
			else if (sv->afc_tries >= SM5714_AFC_MAX_TRIES)
				sv->afc_state = SM5714_AFC_FAILED;
		}
	}

	/*
	 * Arm the charge FET only with a source present AND the cell in the safe
	 * temperature window; otherwise open it.  This is the dual-role device-mode
	 * fix: the bootloader-armed FET is lost when the role driver selects sink
	 * and nothing re-arms it.  Arming happens BEFORE the input limit is raised
	 * below, so the FET closes while VBUSCNTL is still at the 500 mA default
	 * (inrush damping).  Even an SDP (500 mA) charges, so arm on VBUS_POK, not
	 * only when the class beats the SDP ceiling.
	 *
	 * Before closing the FET, force the charger op_mode to CHG_ON_VBUS.  On a
	 * charger-attached boot the poll can run (probe schedules it at delay 0)
	 * before the Type-C role driver has set op_mode for the sink role, with
	 * CNTL2 still at the bootloader default; closing the FET while op_mode is
	 * USB_OTG would source the boost into a closed charge path (back-feed).  We
	 * are host-off here (sv->enabled is false), so this never fights the role
	 * driver's OTG selection.  Disarm on no-source / out-of-window is a driver
	 * policy choice (VBUS_POK + temperature as the arm signal), not a stock
	 * transcription; opening the FET with no input path is harmless.
	 */
	if (vbus && !sv->charge_cut && !READ_ONCE(sv->buck_inhibited)) {
		int cntl2 = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_CNTL2);

		if (cntl2 >= 0 && (cntl2 & SM5714_CHG_CNTL2_OPMODE_MASK) !=
		    SM5714_CHG_CNTL2_OFF)
			i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CNTL2,
				(cntl2 & ~SM5714_CHG_CNTL2_OPMODE_MASK) |
				SM5714_CHG_CNTL2_OFF);
		sm5714_set_charge_fet(sv, true);
	} else {
		sm5714_set_charge_fet(sv, false);
	}

	/* 1) input-current limit (VBUSCNTL); clamp (not wrap) for safety */
	offset = min((input_ma - 100) / 25, (int)SM5714_CHG_VBUSCNTL_MASK);
	cur = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_VBUSCNTL);
	if (cur >= 0 && (cur & SM5714_CHG_VBUSCNTL_MASK) != offset) {
		i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_VBUSCNTL,
					  (cur & ~SM5714_CHG_VBUSCNTL_MASK) | offset);
		dev_info(&sv->chg->dev, "input-current limit set to %d mA\n",
			 input_ma);
	}

	/* 2) cell charge current (CHGCNTL2); 500 mA floor unless the temp gate
	 * above raised it to the device-own TA value */
	offset = sm5714_chgcurr_offset(charge_ma);
	cur = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_CHGCNTL2);
	if (cur >= 0 && cur != offset) {
		i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CHGCNTL2,
					  offset);
		dev_info(&sv->chg->dev, "cell charge current set to %d mA\n",
			 charge_ma);
	}

	/*
	 * Direct-charge intent for the SM5440 supervisor: a source is present AND the
	 * cell is in the safe temperature window -- the same gate (vbus && !charge_cut)
	 * that arms the buck FET above.  Deliberately NOT gated on `charger` (the BC1.2
	 * DCP classification): a pure USB-C PD charger need not assert a BC1.2 DCP
	 * signature, and the executor's pd_contract_active() check is the authoritative
	 * "is this a fast (PPS) charger" filter -- mirroring the device-own
	 * is_pd_apdo_wire_type gate.  A non-PPS source (PC/DCP) leaves dc_intent true but
	 * the executor declines (no contract) and the buck handles it.  Computed on the
	 * normal path only; host-mode / factory-mode / status-read-fail early-outs leave
	 * it false, so a running pump disengages in those states (fail-safe).
	 */
	dc_intent = vbus && !sv->charge_cut;

out:
	mutex_unlock(&sv->lock);

	/*
	 * Publish dc_intent to the SM5440 supervisor.  Done AFTER dropping sv->lock so
	 * the only lock held is instance_lock; the callback is non-blocking (it stores
	 * a flag + schedules the pump's executor work), so no inversion against
	 * inhibit_buck's instance_lock -> sv->lock ordering.
	 */
	mutex_lock(&sm5714_vbus_instance_lock);
	if (sm5714_dc_notify)
		sm5714_dc_notify(sm5714_dc_notify_ctx, dc_intent);
	mutex_unlock(&sm5714_vbus_instance_lock);

	schedule_delayed_work(&sv->charger_work,
			      msecs_to_jiffies(SM5714_CHG_POLL_MS));
}

/*
 * Read-only host-VBUS state indicator.  The SM5714 Type-C role driver
 * (sm5714-typec) owns VBUS now -- it asserts host mode from CC_STATUS -- so this
 * attribute only reports whether the boost is currently sourcing.  It is NOT
 * writable: a manual override would desync the role driver's state (it would cut
 * VBUS while the role driver still believes the port is HOST and, being
 * idempotent, would never re-assert it).
 */
static ssize_t host_vbus_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sm5714_vbus *sv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", sv->enabled);
}
static DEVICE_ATTR_RO(host_vbus);

static struct attribute *sm5714_vbus_attrs[] = {
	&dev_attr_host_vbus.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sm5714_vbus);

/*
 * Charger presence indicator.  The SM5714 charger autonomously regulates the
 * charge (current / float-voltage / top-off) once its charge FET is closed; the
 * poll worker arms that FET when an input source is present (see
 * sm5714_set_charge_fet()), and this attribute exposes whether a charger is
 * online so the desktop shows a line-power source.  VBUS_POK reads set when our
 * own OTG boost sources 5 V, so the report is gated on host mode being off.
 */
static int sm5714_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct sm5714_vbus *sv = power_supply_get_drvdata(psy);
	int status1;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		mutex_lock(&sv->lock);
		if (sv->enabled) {
			val->intval = 0;	/* sourcing VBUS, not charging */
			mutex_unlock(&sv->lock);
			return 0;
		}
		status1 = i2c_smbus_read_byte_data(sv->chg,
						   SM5714_CHG_REG_STATUS1);
		mutex_unlock(&sv->lock);
		if (status1 < 0)
			return status1;
		val->intval = !!(status1 & SM5714_CHG_STATUS1_VBUSPOK);
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property sm5714_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static const struct power_supply_desc sm5714_chg_desc = {
	.name		= "sm5714-charger",
	.type		= POWER_SUPPLY_TYPE_USB,
	.properties	= sm5714_chg_props,
	.num_properties	= ARRAY_SIZE(sm5714_chg_props),
	.get_property	= sm5714_chg_get_property,
};

static int sm5714_vbus_probe(struct i2c_client *client)
{
	struct sm5714_vbus *sv;
	int ret;

	sv = devm_kzalloc(&client->dev, sizeof(*sv), GFP_KERNEL);
	if (!sv)
		return -ENOMEM;

	sv->chg = client;
	mutex_init(&sv->lock);

	/*
	 * Resolve the cell-temperature NTC channel (gen3 PMIC ADC) up front: the
	 * charge thermal gate depends on it, so defer the whole driver until the
	 * ADC provider has probed rather than coming up unable to read the cell.
	 * The ADC is a self-contained pmk8550 SPMI child (it consumes nothing from
	 * this charger), so there is no probe-ordering cycle -- the deferral resolves.
	 */
	sv->batt_therm = devm_iio_channel_get(&client->dev, "batt-therm");
	if (IS_ERR(sv->batt_therm))
		return dev_err_probe(&client->dev, PTR_ERR(sv->batt_therm),
				     "failed to get batt-therm (cell NTC) IIO channel\n");

	sv->muic = devm_i2c_new_dummy_device(&client->dev, client->adapter,
					     SM5714_MUIC_I2C_ADDR);
	if (IS_ERR(sv->muic))
		return dev_err_probe(&client->dev, PTR_ERR(sv->muic),
				     "failed to claim MUIC i2c address 0x%02x\n",
				     SM5714_MUIC_I2C_ADDR);

	dev_set_drvdata(&client->dev, sv);
	i2c_set_clientdata(client, sv);

	{
		struct power_supply_config cfg = { };

		cfg.drv_data = sv;
		cfg.fwnode = dev_fwnode(&client->dev);
		sv->psy = devm_power_supply_register(&client->dev,
						     &sm5714_chg_desc, &cfg);
		if (IS_ERR(sv->psy))
			return dev_err_probe(&client->dev, PTR_ERR(sv->psy),
					     "failed to register charger power supply\n");
	}

	if (of_property_read_bool(client->dev.of_node,
				  "siliconmitus,vbus-always-on")) {
		ret = sm5714_vbus_set(sv, true);
		if (ret)
			dev_warn(&client->dev,
				 "vbus-always-on requested but enable failed: %d\n",
				 ret);
	}

	INIT_DELAYED_WORK(&sv->charger_work, sm5714_vbus_charger_work);
	schedule_delayed_work(&sv->charger_work, 0);

	mutex_lock(&sm5714_vbus_instance_lock);
	sm5714_vbus_instance = sv;
	mutex_unlock(&sm5714_vbus_instance_lock);

	dev_info(&client->dev,
		 "SM5714 USB host VBUS controller ready (host_vbus=%d)\n",
		 sv->enabled);
	return 0;
}

static void sm5714_vbus_remove(struct i2c_client *client)
{
	struct sm5714_vbus *sv = i2c_get_clientdata(client);

	mutex_lock(&sm5714_vbus_instance_lock);
	if (sm5714_vbus_instance == sv)
		sm5714_vbus_instance = NULL;
	mutex_unlock(&sm5714_vbus_instance_lock);

	cancel_delayed_work_sync(&sv->charger_work);
	sm5714_vbus_set(sv, false);
}

static const struct of_device_id sm5714_vbus_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-usb-vbus" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_vbus_of_match);

static struct i2c_driver sm5714_vbus_driver = {
	.driver = {
		.name = "sm5714-usb-vbus",
		.of_match_table = sm5714_vbus_of_match,
		.dev_groups = sm5714_vbus_groups,
	},
	.probe = sm5714_vbus_probe,
	.remove = sm5714_vbus_remove,
};
module_i2c_driver(sm5714_vbus_driver);

MODULE_DESCRIPTION("SM5714 USB host (OTG) VBUS + data-path enabler");
MODULE_AUTHOR("ubuntu-tab project");
MODULE_LICENSE("GPL");
