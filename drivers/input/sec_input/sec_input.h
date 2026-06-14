/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Minimal sec_input compatibility shim for the mainline ubuntu-tab port of the
 * Samsung stm32_pogo_v3 keyboard-cover driver.
 *
 * The Samsung downstream driver tree depends on a large in-house "sec_input"
 * framework (factory-test sysfs, tsp logging, a private /sys/class/sec device
 * class, an input-event notifier). The stm32_pogo_v3 bundle only touches a tiny
 * slice of it:
 *
 *   - the input_dbg/input_info/input_err logging macros + SECLOG tag
 *   - sec_delay() (a millisecond sleep helper)
 *   - sec_device_create()/sec_device_destroy() (the /sys/class/sec device)
 *   - sec_input_register_notify()/unregister (only under CONFIG_INPUT_SEC_NOTIFIER)
 *
 * This header provides exactly that slice, faithful to the downstream semantics
 * (the logging macros mirror the non-CONFIG_SEC_DEBUG_TSP_LOG variant in the
 * original sec_input.h), so the driver source compiles near-verbatim against a
 * mainline kernel without dragging in the rest of the framework.
 */
#ifndef __SEC_INPUT_SHIM_H__
#define __SEC_INPUT_SHIM_H__

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/notifier.h>

#define SECLOG				"[sec_input]"
#define INPUT_LOG_BUF_SIZE		512

/* sec_input return codes + wakelock hold time (downstream sec_input.h). */
enum sec_input_ret {
	SEC_SUCCESS = 0,
	SEC_ERROR = -1,
};
#define SEC_TS_WAKE_LOCK_TIME		500	/* msec */

/*
 * Logging macros — map onto the standard dev_* helpers, prefixed with SECLOG.
 * The leading "mode" argument selected the factory tsp-log sink downstream; with
 * no tsp log here it is simply ignored, exactly like the downstream #else path.
 */
#define input_dbg(mode, dev, fmt, ...)					\
({									\
	dev_dbg(dev, SECLOG " " fmt, ## __VA_ARGS__);			\
})
#define input_info(mode, dev, fmt, ...)					\
({									\
	dev_info(dev, SECLOG " " fmt, ## __VA_ARGS__);			\
})
#define input_err(mode, dev, fmt, ...)					\
({									\
	dev_err(dev, SECLOG " " fmt, ## __VA_ARGS__);			\
})
#define input_raw_info(mode, dev, fmt, ...) input_info(mode, dev, fmt, ## __VA_ARGS__)
#define input_log_fix()		do {} while (0)
#define input_raw_data_clear()	do {} while (0)

/* Millisecond sleep helper (downstream sec_common_fn.c). */
static inline void sec_delay(unsigned int ms)
{
	if (ms >= 20U)
		msleep(ms);
	else if (ms)
		usleep_range(ms * 1000, ms * 1000 + 100);
}

/* /sys/class/sec device helpers — implemented in sec_input_shim.c. */
struct device *sec_device_create(void *drvdata, const char *fmt);
void sec_device_destroy(dev_t devt);

#if IS_ENABLED(CONFIG_INPUT_SEC_NOTIFIER)
void sec_input_register_notify(struct notifier_block *nb, notifier_fn_t notifier_call, int priority);
void sec_input_unregister_notify(struct notifier_block *nb);
#endif

#endif /* __SEC_INPUT_SHIM_H__ */
