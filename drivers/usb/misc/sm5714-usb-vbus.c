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
#include <linux/sysfs.h>

/* MUIC sub-block: secondary i2c address + the manual-switch register */
#define SM5714_MUIC_I2C_ADDR		0x25
#define SM5714_MUIC_REG_MANUAL_SW	0x06
#define SM5714_MUIC_MANSW_USB		0x89	/* bit7 manual | DM/DP -> USB */
#define SM5714_MUIC_MANSW_OPEN		0x00	/* open all (D+/D-/VBUS detached) */

/* Charger sub-block: this driver binds its i2c client (0x49) */
#define SM5714_CHG_REG_CNTL2		0x14
#define SM5714_CHG_CNTL2_OTG		0x07	/* OP_MODE = USB_OTG */
#define SM5714_CHG_CNTL2_OFF		0x05	/* OP_MODE default (boost off) */
#define SM5714_CHG_REG_BSTCNTL1		0x23
#define SM5714_CHG_BSTCNTL1_OTG		0x46	/* 5.1 V / 900 mA OTG boost */
#define SM5714_CHG_REG_STATUS3		0x0f
#define SM5714_CHG_STATUS3_OTGFAIL	BIT(2)

struct sm5714_vbus {
	struct i2c_client *chg;		/* 0x49 - the bound device */
	struct i2c_client *muic;	/* 0x25 - secondary client */
	struct mutex lock;
	bool enabled;
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

static void sm5714_vbus_disable(struct sm5714_vbus *sv)
{
	/* Cut VBUS first, then open the data switch. */
	i2c_smbus_write_byte_data(sv->chg, SM5714_CHG_REG_CNTL2,
				  SM5714_CHG_CNTL2_OFF);
	i2c_smbus_write_byte_data(sv->muic, SM5714_MUIC_REG_MANUAL_SW,
				  SM5714_MUIC_MANSW_OPEN);
}

static int sm5714_vbus_set(struct sm5714_vbus *sv, bool on)
{
	int ret = 0;

	mutex_lock(&sv->lock);
	if (on == sv->enabled)
		goto out;

	if (on)
		ret = sm5714_vbus_enable(sv);
	else
		sm5714_vbus_disable(sv);

	if (!ret)
		sv->enabled = on;
out:
	mutex_unlock(&sv->lock);
	return ret;
}

static ssize_t host_vbus_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct sm5714_vbus *sv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", sv->enabled);
}

static ssize_t host_vbus_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sm5714_vbus *sv = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	ret = sm5714_vbus_set(sv, on);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(host_vbus);

static struct attribute *sm5714_vbus_attrs[] = {
	&dev_attr_host_vbus.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sm5714_vbus);

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

	if (of_property_read_bool(client->dev.of_node,
				  "siliconmitus,vbus-always-on")) {
		ret = sm5714_vbus_set(sv, true);
		if (ret)
			dev_warn(&client->dev,
				 "vbus-always-on requested but enable failed: %d\n",
				 ret);
	}

	dev_info(&client->dev,
		 "SM5714 USB host VBUS controller ready (host_vbus=%d)\n",
		 sv->enabled);
	return 0;
}

static void sm5714_vbus_remove(struct i2c_client *client)
{
	struct sm5714_vbus *sv = i2c_get_clientdata(client);

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
