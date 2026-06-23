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
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

/* Register map (device-own sm5440_charger.h). */
#define SM5440_REG_STATUS3	0x0a
#define SM5440_REG_ADCCNTL1	0x1c
#define SM5440_REG_ADCCNTL2	0x1d
#define SM5440_REG_ADC_VBUS1	0x1e
#define SM5440_REG_ADC_VBUS2	0x1f
#define SM5440_REG_ADC_IBUS1	0x22
#define SM5440_REG_ADC_IBUS2	0x23
#define SM5440_REG_ADC_DIETEMP	0x26
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

struct sm5440 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
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

	dev_info(dev, "SM5440 charge-pump telemetry ready (rev 0x%x, DEVICEID 0x%02x)\n",
		 chip->rev_id, devid);
	return 0;
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
	.id_table	= sm5440_i2c_id,
};
module_i2c_driver(sm5440_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5440 charge-pump telemetry driver");
MODULE_AUTHOR("omar fityan <me@omarfityan.com>");
MODULE_LICENSE("GPL");
