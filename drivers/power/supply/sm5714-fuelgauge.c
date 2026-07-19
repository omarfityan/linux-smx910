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
#include <linux/power_supply.h>

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
#define SM5714_FG_SRAM_TEMPERATURE	0x07	/* die/battery temperature */

struct sm5714_fg {
	struct i2c_client *client;
	struct power_supply *psy;
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

/* Current in microamps; positive = charging, negative = discharging. */
static int sm5714_fg_get_current(struct sm5714_fg *fg, int *ua)
{
	int raw = sm5714_fg_read_sram(fg, SM5714_FG_SRAM_CURRENT);
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

static int sm5714_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5714_fg *fg = power_supply_get_drvdata(psy);
	int ret, ua;

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
		return sm5714_fg_get_capacity(fg, &val->intval);
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
		ret = sm5714_fg_get_current(fg, &ua);
		if (ret)
			return ret;
		if (ua > 30000)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (ua < -30000)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
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
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
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
	struct power_supply_config cfg = { };
	struct sm5714_fg *fg;
	int id, status, try;

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

	i2c_set_clientdata(client, fg);
	dev_info(&client->dev,
		 "SM5714 fuel-gauge ready (id=0x%04x, status=0x%04x)\n",
		 id, status);
	return 0;
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
};
module_i2c_driver(sm5714_fg_driver);

MODULE_DESCRIPTION("SM5714 battery fuel-gauge (read-only)");
MODULE_AUTHOR("ubuntu-tab project");
MODULE_LICENSE("GPL");
