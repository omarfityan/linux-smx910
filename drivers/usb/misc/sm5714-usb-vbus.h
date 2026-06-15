/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared interface between the SM5714 USB host VBUS driver (charger 0x49) and
 * the SM5714 Type-C role driver (PDIC 0x33).  The role driver detects host vs
 * device from CC_STATUS and calls sm5714_usb_vbus_set_host() to source or cut
 * the OTG VBUS + data routing that the VBUS driver owns.
 */
#ifndef __SM5714_USB_VBUS_H
#define __SM5714_USB_VBUS_H

#include <linux/errno.h>
#include <linux/types.h>

#if IS_ENABLED(CONFIG_USB_SM5714_VBUS)
int sm5714_usb_vbus_set_host(bool on);
#else
static inline int sm5714_usb_vbus_set_host(bool on) { return -ENODEV; }
#endif

#endif /* __SM5714_USB_VBUS_H */
