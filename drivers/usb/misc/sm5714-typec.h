/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cross-driver interface exported by the SM5714 Type-C role/PD driver (PDIC at
 * I2C 0x33) for the SM5440 charge-pump driver.
 *
 * The PDIC driver negotiates and sustains a USB-PD PPS contract (a fixed 10 V
 * keepalive by default).  When the SM5440 2:1 charge-pump engages for fast
 * charging, its CC/CV control loop must step the PPS input voltage to track the
 * cell; sm5714_pd_request_voltage() commands the sustained contract to a new
 * target, and sm5714_pd_contract_active() lets the pump supervisor gate on a
 * live contract before engaging.
 */
#ifndef __SM5714_TYPEC_H
#define __SM5714_TYPEC_H

#include <linux/errno.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_USB_SM5714_TYPEC)
int sm5714_pd_request_voltage(unsigned int mv, unsigned int ma);
bool sm5714_pd_contract_active(void);
#else
static inline int sm5714_pd_request_voltage(unsigned int mv, unsigned int ma)
{
	return -ENODEV;
}
static inline bool sm5714_pd_contract_active(void) { return false; }
#endif

#endif /* __SM5714_TYPEC_H */
