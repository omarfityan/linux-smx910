/* SPDX-License-Identifier: GPL-2.0 */
/*
 * framebuffer fault codec -- emits register
 * values as nibble-colored stripes on the bootloader splash FB (PA 0xb8000000)
 * so an early-boot panic/abort reports its FAR/ESR/PC visually, decoded offline
 * by pixel-scanning the panel photo. Impl in arch/arm64/kernel/setup.c.
 */
#ifndef _LINUX_FBDBG_H
#define _LINUX_FBDBG_H
#include <linux/types.h>
void fbdbg_stripe(unsigned long off, u32 color);
void fbdbg_emit_octal(unsigned long base, u64 val, int n);
void fbdbg_emit_fault(unsigned long elr);
void fbdbg_emit_panic(unsigned long retaddr);
void fbdbg_emit_warn(unsigned long caller);
#endif /* _LINUX_FBDBG_H */
