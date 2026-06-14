// SPDX-License-Identifier: GPL-2.0
/*
 * sec_class compatibility shim for the ubuntu-tab stm32_pogo_v3 keyboard-cover
 * port.
 *
 * Samsung's downstream tree exposes a private device class at /sys/class/sec and
 * a sec_device_create()/sec_device_destroy() pair (drivers/sec/sec_class.c) that
 * the stm32_pogo_v3 driver uses to publish its "sec_keypad" device (factory
 * sysfs group + the CONNECT= uevent on cover attach/detach). Mainline has no
 * such class, so this file provides a minimal, faithful equivalent: a class
 * named "sec" created lazily on first use, with device_create()-backed
 * create/destroy. Behaviour matches the downstream wrapper closely enough that
 * the driver's cmd/notifier code compiles and runs verbatim.
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include "../sec_input.h"

static struct class *sec_class;
static DEFINE_MUTEX(sec_class_lock);
static dev_t sec_devt_next = MKDEV(0, 1);

static int sec_class_ensure(void)
{
	int ret = 0;

	mutex_lock(&sec_class_lock);
	if (!sec_class) {
		/* kernel 6.4+: class_create() takes a single name argument */
		sec_class = class_create("sec");
		if (IS_ERR(sec_class)) {
			ret = PTR_ERR(sec_class);
			sec_class = NULL;
		}
	}
	mutex_unlock(&sec_class_lock);
	return ret;
}

struct device *sec_device_create(void *drvdata, const char *fmt)
{
	struct device *dev;
	dev_t devt;
	int ret;

	ret = sec_class_ensure();
	if (ret)
		return ERR_PTR(ret);

	mutex_lock(&sec_class_lock);
	devt = sec_devt_next;
	sec_devt_next = MKDEV(MAJOR(sec_devt_next), MINOR(sec_devt_next) + 1);
	mutex_unlock(&sec_class_lock);

	dev = device_create(sec_class, NULL, devt, drvdata, "%s", fmt);
	return dev;
}
EXPORT_SYMBOL(sec_device_create);

void sec_device_destroy(dev_t devt)
{
	if (sec_class)
		device_destroy(sec_class, devt);
}
EXPORT_SYMBOL(sec_device_destroy);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Minimal sec_class shim for stm32_pogo_v3 keyboard-cover port");
