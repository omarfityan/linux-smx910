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
/*
 * Inhibit (or re-allow) the charging worker's Samsung-AFC high-voltage
 * handshake.  The role driver calls this with true while a USB-PD contract is
 * active so the AFC handshake (which perturbs D+/D- and VBUS) does not race and
 * break the PD contract on the shared cable; false on PD teardown re-allows AFC
 * as the fallback high-voltage path.  Mirrors the downstream pdic_afc_state
 * PDIC->MUIC inhibit signal.
 */
void sm5714_usb_vbus_inhibit_afc(bool inhibit);
/*
 * Inhibit (or re-allow) the SM5714 buck charge path.  The SM5440 charge-pump
 * driver calls this with true before engaging the pump: it disarms the cell
 * charge FET immediately and stops the charging worker re-arming it, so the
 * pump owns the cell while engaged (the device-own buck-OFF-before-pump-ON
 * handoff).  false on disengage re-allows the worker to resume buck charging.
 */
int sm5714_charger_inhibit_buck(bool inhibit);
#else
static inline int sm5714_usb_vbus_set_host(bool on) { return -ENODEV; }
static inline void sm5714_usb_vbus_inhibit_afc(bool inhibit) { }
static inline int sm5714_charger_inhibit_buck(bool inhibit) { return -ENODEV; }
#endif

#endif /* __SM5714_USB_VBUS_H */
