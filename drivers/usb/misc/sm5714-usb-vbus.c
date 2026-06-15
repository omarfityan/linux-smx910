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
#include <linux/i2c.h>
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
#define SM5714_CHG_REG_CNTL2		0x14
#define SM5714_CHG_CNTL2_OTG		0x07	/* OP_MODE = USB_OTG */
#define SM5714_CHG_CNTL2_OFF		0x05	/* OP_MODE default (boost off) */
#define SM5714_CHG_REG_BSTCNTL1		0x23
#define SM5714_CHG_BSTCNTL1_OTG		0x46	/* 5.1 V / 900 mA OTG boost */
#define SM5714_CHG_REG_STATUS3		0x0f
#define SM5714_CHG_STATUS3_OTGFAIL	BIT(2)
#define SM5714_CHG_REG_STATUS1		0x0d
#define SM5714_CHG_STATUS1_VBUSPOK	BIT(0)	/* valid charger VBUS present */
#define SM5714_CHG_REG_VBUSCNTL		0x15	/* input-current limit */
#define SM5714_CHG_VBUSCNTL_MASK	0x7f	/* offset = (mA - 100) / 25 */
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
#define SM5714_BATT_TEMP_HOT		450	/* >=45.0 C: throttle (enter) */
#define SM5714_BATT_TEMP_HOT_OK		400	/* <=40.0 C: hot cleared (exit) */
#define SM5714_BATT_TEMP_COLD		100	/* <=10.0 C: throttle (enter) */
#define SM5714_BATT_TEMP_COLD_OK	150	/* >=15.0 C: cold cleared (exit) */
#define SM5714_BATT_TEMP_VALID_LO	-300	/* < -30.0 C: implausible read */
#define SM5714_BATT_TEMP_VALID_HI	800	/* >  80.0 C: implausible read */

/* poll period: no IRQ wired, so re-evaluate the attached charger periodically */
#define SM5714_CHG_POLL_MS		3000

struct sm5714_vbus {
	struct i2c_client *chg;		/* 0x49 - the bound device */
	struct i2c_client *muic;	/* 0x25 - secondary client */
	struct mutex lock;
	bool enabled;
	bool charge_throttled;		/* hot/cold throttle hysteresis state */
	struct power_supply *psy;	/* charger online indicator */
	struct delayed_work charger_work;	/* input-current management */
};

static int sm5714_vbus_enable(struct sm5714_vbus *sv)
{
	int ret, status;

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

/* Read the cell temperature (0.1 C) from the SM5714 fuel-gauge power supply. */
static bool sm5714_read_batt_temp(int *temp)
{
	struct power_supply *fg;
	union power_supply_propval val;
	bool ok = false;

	fg = power_supply_get_by_name("sm5714-fuelgauge");
	if (!fg)
		return false;
	/* Reject implausible (garbage / stale) readings -> caller uses the floor. */
	if (power_supply_get_property(fg, POWER_SUPPLY_PROP_TEMP, &val) == 0 &&
	    val.intval >= SM5714_BATT_TEMP_VALID_LO &&
	    val.intval <= SM5714_BATT_TEMP_VALID_HI) {
		*temp = val.intval;
		ok = true;
	}
	power_supply_put(fg);
	return ok;
}

/*
 * The SM5714 charger autonomously charges, but its input-current limit defaults
 * to 500 mA -- too low to charge under load -- and its cell charge current to a
 * conservative 500 mA.  With no IRQ wired, poll: when a charger is present (and
 * we are not sourcing VBUS for host mode), read the MUIC classification, raise
 * the input limit (VBUSCNTL) to the device-own ceiling for that charger class,
 * and raise the cell charge current (CHGCNTL2) toward the device-own 2100 mA TA
 * value -- but only with a real charger AND a temperature read, throttling back
 * to 500 mA when the cell runs hot (we do not run stock's full step-charging).
 * When nothing is attached, both return to 500 mA.  Hardware AICL caps the input
 * draw to the adapter's capability; charge/float-voltage/OVP are hardware.
 */
static void sm5714_vbus_charger_work(struct work_struct *work)
{
	struct sm5714_vbus *sv = container_of(work, struct sm5714_vbus,
					      charger_work.work);
	int status1, dev_type1, factory, cur, temp;
	int input_ma = SM5714_INPUT_CURRENT_SDP;
	int charge_ma = SM5714_CHARGE_CURRENT_MIN;
	bool charger = false;
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

	if (status1 & SM5714_CHG_STATUS1_VBUSPOK) {
		dev_type1 = i2c_smbus_read_byte_data(sv->muic,
						     SM5714_MUIC_REG_DEVTYPE1);
		if (dev_type1 >= 0) {
			input_ma = sm5714_input_current_ma(dev_type1);
			/* a source above SDP can charge faster than the default */
			charger = input_ma > SM5714_INPUT_CURRENT_SDP;
		}
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

	/*
	 * 2) cell charge current (CHGCNTL2): raise to the device-own TA value
	 * only inside a safe temperature window.  Throttle to the floor when the
	 * cell is HOT (>=45 C) or COLD (<=10 C -- fast charge there risks lithium
	 * plating), with a dead-band so the state cannot oscillate.  Any failed or
	 * implausible temperature read leaves charge_ma at the 500 mA floor.
	 */
	if (charger && sm5714_read_batt_temp(&temp)) {
		if (temp >= SM5714_BATT_TEMP_HOT || temp <= SM5714_BATT_TEMP_COLD)
			sv->charge_throttled = true;
		else if (temp <= SM5714_BATT_TEMP_HOT_OK &&
			 temp >= SM5714_BATT_TEMP_COLD_OK)
			sv->charge_throttled = false;
		charge_ma = sv->charge_throttled ? SM5714_CHARGE_CURRENT_MIN
						 : SM5714_CHARGE_CURRENT_TA;
	}

	offset = sm5714_chgcurr_offset(charge_ma);
	cur = i2c_smbus_read_byte_data(sv->chg, SM5714_CHG_REG_CHGCNTL2);
	if (cur >= 0 && cur != offset) {
		i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CHGCNTL2,
					  offset);
		dev_info(&sv->chg->dev, "cell charge current set to %d mA\n",
			 charge_ma);
	}

out:
	mutex_unlock(&sv->lock);
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
 * Report-only charger presence.  The SM5714 charger autonomously charges the
 * battery whenever valid VBUS is present (the bootloader leaves it in
 * CHG_ON_VBUS mode with the charge FET armed), so this driver does not touch
 * the charge path -- it only exposes whether a charger is online so the desktop
 * shows a line-power source.  VBUS_POK reads set when our own OTG boost sources
 * 5 V, so the report is gated on host mode being off.
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
