# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the Samsung STM32 POGO keyboard-cover driver (V3 bundle).
#
# ubuntu-tab port: V3-only active path + a minimal sec_class shim. The downstream
# V1/V2 monolithic files (stm32_pogo_i2c.c, stm32_pogo_keyboard.c,
# stm32_pogo_keyboard_v2.c, stm32_pogo_touchpad.c) are intentionally omitted.
#
TARGETV3 = stm32_pogo_v3

$(TARGETV3)-objs := stm32_pogo_i2c_v3.o stm32_pogo_fw.o stm32_pogo_keyboard_v3.o \
		    stm32_pogo_keyboard_v3_common.o stm32_pogo_core_v3.o \
		    stm32_pogo_interrupt_v3.o stm32_pogo_notifier_v3.o \
		    stm32_pogo_voting_v3.o stm32_pogo_fn_v3.o stm32_pogo_cmd_v3.o \
		    stm32_pogo_keyboard_v3_row.o stm32_pogo_keyboard_v3_bypass.o \
		    stm32_pogo_touchpad_v3.o kbd_max77816_i2c.o sec_input_shim.o

obj-$(CONFIG_KEYBOARD_STM32_POGO_V3) += $(TARGETV3).o

ccflags-y += -Wformat
