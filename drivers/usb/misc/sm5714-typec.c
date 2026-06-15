// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5714 USB Type-C role (PDIC) driver
 *
 * The SM5714 combined PMIC includes a Type-C/PD controller (the "PDIC") reached
 * over I2C at 7-bit address 0x33 -- on a separate I2C-master-hub serial engine
 * from the charger/fuel-gauge (e.g. Samsung Galaxy Tab S9 Ultra, SM-X910, where
 * the PDIC sits on i2c_hub_9 and the charger 0x49 / fuel-gauge 0x71 on
 * i2c_hub_8).  This driver makes USB-C role detection automatic:
 *
 *   - it programs the controller for autonomous Dual-Role (DRP) toggling;
 *   - on each attach/detach it reads CC_STATUS and decodes the cable: a source
 *     on the cable (a charger or host PC) means we are the DEVICE/sink; a sink
 *     on the cable (a USB peripheral) means we are the HOST/source;
 *   - it drives the standard mainline usb_role_switch so dwc3 flips its data
 *     role (host <-> gadget) accordingly, and asks the SM5714 VBUS driver to
 *     source 5 V (host) or stay off (device/disconnected).
 *
 * This replaces the manual "host_vbus" sysfs toggle with automatic, cable-driven
 * role switching.  All register values are transcribed from the device's own
 * downstream driver (drivers/usb/typec/sm/sm5714/sm5714_typec.c) and confirmed
 * live on the hardware (CC_STATUS reads 0x01 with a charger, 0x02 with a
 * peripheral, 0x00 unplugged).
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include "sm5714-usb-vbus.h"

/* Interrupt / status registers (latched INT read-to-clear; STATUS is live). */
#define SM5714_TYPEC_REG_INT1		0x01
#define SM5714_TYPEC_REG_INT_MASK1	0x06
#define SM5714_TYPEC_REG_STATUS1	0x0b
#define SM5714_TYPEC_INT1_ATTACH	BIT(3)
#define SM5714_TYPEC_INT1_DETACH	BIT(4)
/* STATUS1 is the live mirror of INT1: same bit layout. */
#define SM5714_TYPEC_STATUS1_ATTACH	BIT(3)
/* INT_MASK1: a set bit masks (disables) the source; clear to enable.  Enable
 * only ATTACH+DETACH (0xff & ~(BIT(3)|BIT(4)) == 0xe7). */
#define SM5714_TYPEC_INT_MASK1_ATTDET	0xe7

/* CC status + role control. */
#define SM5714_TYPEC_REG_CC_STATUS	0x28
#define SM5714_TYPEC_CC_ATTACH_MASK	0x07
#define SM5714_TYPEC_CC_ATTACH_SRC	0x01	/* source on cable -> we DEVICE */
#define SM5714_TYPEC_CC_ATTACH_SNK	0x02	/* sink on cable   -> we HOST   */
#define SM5714_TYPEC_CC_ATTACH_AUDIO	0x03	/* audio accessory             */
#define SM5714_TYPEC_REG_CC_CNTL1	0x29
#define SM5714_TYPEC_CC_CNTL1_DRP	0x40	/* autonomous Dual-Role toggling */

/* No edge can be missed without leaving VBUS mis-sourced, so re-sync slowly. */
#define SM5714_TYPEC_POLL_MS		2000

struct sm5714_typec {
	struct i2c_client *client;
	struct usb_role_switch *role_sw;
	struct mutex lock;
	struct delayed_work resync;
	enum usb_role role;
};

/* Apply a decoded role: VBUS first toward the fail-safe, then the data role. */
static void sm5714_typec_apply(struct sm5714_typec *t, enum usb_role role)
{
	bool host = (role == USB_ROLE_HOST);
	int ret;

	if (role == t->role)
		return;

	/*
	 * VBUS is sourced ONLY for HOST.  Drive it before the data role so that
	 * on any transition away from HOST the 5 V boost is cut first (fail-safe
	 * against back-feeding a newly attached source).
	 */
	ret = sm5714_usb_vbus_set_host(host);

	/*
	 * A failed VBUS *cut* is safety-critical: the boost may still be sourcing
	 * 5 V.  Do not commit the new role, so the periodic resync re-enters here
	 * (role still != t->role) and retries the cut instead of early-outing on an
	 * unchanged role.  -ENODEV means the VBUS driver has not probed, so there is
	 * no boost to cut -- not a failure.  A failed *enable* sources no VBUS, so
	 * it does not block adopting the (non-host) role.
	 */
	if (ret && ret != -ENODEV && !host) {
		dev_err(&t->client->dev,
			"failed to cut OTG VBUS (%d); will retry\n", ret);
		return;
	}

	if (t->role_sw)
		usb_role_switch_set_role(t->role_sw, role);

	t->role = role;
	dev_info(&t->client->dev, "USB-C role -> %s\n", usb_role_string(role));
}

/* Read CC_STATUS and map the cable to a USB role; fail safe to NONE on error. */
static void sm5714_typec_update(struct sm5714_typec *t)
{
	enum usb_role role;
	int status, cc;

	/*
	 * Trust CC_STATUS only when the controller reports a resolved attachment
	 * (STATUS1 ATTACH, bit3).  CC_STATUS can read a transient orientation while
	 * the DRP state machine toggles Rp/Rd with nothing attached, and this path
	 * also runs from the blind periodic resync -- so without this gate a
	 * spurious 0x02 could wrongly source host VBUS.  Not attached -> NONE.
	 */
	status = i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_STATUS1);
	if (status < 0) {
		dev_warn(&t->client->dev, "STATUS1 read failed (%d); role->none\n",
			 status);
		sm5714_typec_apply(t, USB_ROLE_NONE);
		return;
	}
	if (!(status & SM5714_TYPEC_STATUS1_ATTACH)) {
		sm5714_typec_apply(t, USB_ROLE_NONE);
		return;
	}

	cc = i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_CC_STATUS);
	if (cc < 0) {
		dev_warn(&t->client->dev, "CC_STATUS read failed (%d); role->none\n",
			 cc);
		sm5714_typec_apply(t, USB_ROLE_NONE);
		return;
	}

	switch (cc & SM5714_TYPEC_CC_ATTACH_MASK) {
	case SM5714_TYPEC_CC_ATTACH_SNK:
		role = USB_ROLE_HOST;		/* a peripheral is attached */
		break;
	case SM5714_TYPEC_CC_ATTACH_SRC:
		role = USB_ROLE_DEVICE;		/* a charger / host PC is attached */
		break;
	default:				/* 0x03 audio accessory */
		role = USB_ROLE_NONE;
		break;
	}

	sm5714_typec_apply(t, role);
}

static irqreturn_t sm5714_typec_irq(int irq, void *data)
{
	struct sm5714_typec *t = data;

	mutex_lock(&t->lock);
	/* Read-to-clear the latched attach/detach interrupt, then re-evaluate. */
	i2c_smbus_read_byte_data(t->client, SM5714_TYPEC_REG_INT1);
	sm5714_typec_update(t);
	mutex_unlock(&t->lock);

	return IRQ_HANDLED;
}

/* Slow safety re-sync in case a transition's edge was ever missed. */
static void sm5714_typec_resync_work(struct work_struct *work)
{
	struct sm5714_typec *t = container_of(work, struct sm5714_typec,
					      resync.work);

	mutex_lock(&t->lock);
	sm5714_typec_update(t);
	mutex_unlock(&t->lock);

	schedule_delayed_work(&t->resync, msecs_to_jiffies(SM5714_TYPEC_POLL_MS));
}

static struct usb_role_switch *sm5714_typec_get_role_sw(struct device *dev)
{
	struct fwnode_handle *connector;
	struct usb_role_switch *sw;

	/*
	 * dwc3 (dr_mode=otg, usb-role-switch) registers the role switch; we are
	 * the provider that drives it.  The link is the OF graph from our
	 * usb-c-connector child node to the dwc3 HS port.  Fall back to a direct
	 * lookup from our own node if no connector child is present.
	 */
	connector = device_get_named_child_node(dev, "connector");
	if (connector) {
		sw = fwnode_usb_role_switch_get(connector);
		fwnode_handle_put(connector);
	} else {
		sw = usb_role_switch_get(dev);
	}

	return sw;
}

static int sm5714_typec_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sm5714_typec *t;
	int ret;

	t = devm_kzalloc(dev, sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->client = client;
	t->role = USB_ROLE_NONE;
	mutex_init(&t->lock);
	INIT_DELAYED_WORK(&t->resync, sm5714_typec_resync_work);
	i2c_set_clientdata(client, t);

	t->role_sw = sm5714_typec_get_role_sw(dev);
	if (IS_ERR(t->role_sw))
		return dev_err_probe(dev, PTR_ERR(t->role_sw),
				     "failed to get usb_role_switch\n");

	/* Program autonomous DRP so the controller toggles Rp/Rd on its own. */
	ret = i2c_smbus_write_byte_data(client, SM5714_TYPEC_REG_CC_CNTL1,
					SM5714_TYPEC_CC_CNTL1_DRP);
	if (ret) {
		dev_err_probe(dev, ret, "failed to enable DRP\n");
		goto err_put;
	}

	/* Unmask only attach/detach, then clear any latched state. */
	i2c_smbus_write_byte_data(client, SM5714_TYPEC_REG_INT_MASK1,
				  SM5714_TYPEC_INT_MASK1_ATTDET);
	i2c_smbus_read_byte_data(client, SM5714_TYPEC_REG_INT1);

	/* Reflect the cable state present at boot before arming the IRQ. */
	mutex_lock(&t->lock);
	sm5714_typec_update(t);
	mutex_unlock(&t->lock);

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sm5714_typec_irq,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"sm5714-typec", t);
		if (ret) {
			dev_err_probe(dev, ret, "failed to request IRQ\n");
			goto err_put;
		}
	} else {
		dev_warn(dev, "no IRQ; relying on %d ms poll\n",
			 SM5714_TYPEC_POLL_MS);
	}

	/* Slow re-sync backstop regardless of IRQ. */
	schedule_delayed_work(&t->resync, msecs_to_jiffies(SM5714_TYPEC_POLL_MS));

	dev_info(dev, "SM5714 Type-C role controller ready (role=%s)\n",
		 usb_role_string(t->role));
	return 0;

err_put:
	usb_role_switch_put(t->role_sw);
	return ret;
}

static void sm5714_typec_remove(struct i2c_client *client)
{
	struct sm5714_typec *t = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&t->resync);
	/* Leave the port disconnected + VBUS off. */
	mutex_lock(&t->lock);
	sm5714_typec_apply(t, USB_ROLE_NONE);
	mutex_unlock(&t->lock);
	usb_role_switch_put(t->role_sw);
}

static const struct of_device_id sm5714_typec_of_match[] = {
	{ .compatible = "siliconmitus,sm5714-typec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5714_typec_of_match);

static struct i2c_driver sm5714_typec_driver = {
	.driver = {
		.name = "sm5714-typec",
		.of_match_table = sm5714_typec_of_match,
	},
	.probe = sm5714_typec_probe,
	.remove = sm5714_typec_remove,
};
module_i2c_driver(sm5714_typec_driver);

MODULE_DESCRIPTION("SM5714 USB Type-C role (PDIC) driver");
MODULE_AUTHOR("ubuntu-tab project");
MODULE_LICENSE("GPL");
