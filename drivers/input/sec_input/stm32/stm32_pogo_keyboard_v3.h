
#ifndef __STM32_POGO_KEYBOARD_V3_H__
#define __STM32_POGO_KEYBOARD_V3_H__

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_OF)
#include <linux/of_gpio.h>
#endif
#include <linux/input/matrix_keypad.h>
#include "stm32_pogo_v3.h"

/*
 * Samsung-custom keycodes absent from mainline include/uapi/linux/input-event-codes.h.
 *
 * KEY_DEX_ON: downstream value 0x2bd — numerically identical to mainline
 *   KEY_PERFORMANCE (0x2bd), so the code is consistent across both names.
 * KEY_AI_HOT: NOT defined in the available SM-X910 downstream mirror (a header gap
 *   in that tree). It only gates a per-model capability bit for the AI keyboard SKUs
 *   (model 0xd1-0xd6 / 0x03 / 0x51); inert for standard covers. Placed at a free
 *   vendor slot below KEY_MAX (0x2ff) pending the canonical value.
 *   TODO: replace with the real Samsung value once a complete source tree is available.
 */
#ifndef KEY_DEX_ON
#define KEY_DEX_ON				0x2bd
#endif
#ifndef KEY_AI_HOT
#define KEY_AI_HOT				0x2c7
#endif

#define STM32_KEY_MAX				762

#define STM32KPD_DRV_NAME			"stm32_kpd"

#define INPUT_VENDOR_ID_SAMSUNG			0x04E8
#define INPUT_PRODUCT_ID_POGO_KEYBOARD		0xA035

#define STM32_KPC_ROW_SHIFT			5
#define STM32_KPC_DATA_PRESS			1

#define STM32_GAMEPAD_KEY_START			304
#define STM32_GAMEPAD_KEY_END			318

#define STM32_CHECK_LED(STM32, IS_LED_ENABLED, LED)		\
	do {							\
		if (test_bit(LED, (STM32)->input_dev->led))	\
			IS_LED_ENABLED = true;			\
		else						\
			IS_LED_ENABLED = false;			\
	} while (0)

struct stm32_keyevent_data_row {
	union {
		struct {
			u16 press:1;
			u16 col:5;
			u16 row:3;
			u16 reserved:7;
		} __packed;
		u16 data[1];
	};
};

struct stm32_keyevent_data {
	union {
		struct {
			u16 key_value:15;
			u16 press:1;
		} __packed;
		u16 data;
	};
};

struct stm32_keypad_dtdata_row {
	struct matrix_keymap_data *keymap_data1;
	struct matrix_keymap_data *keymap_data2;
	int num_row;
	int num_column;
	const char *input_name[3];
};

struct stm32_keypad_dtdata {
	const char *input_name[KDB_SUPPORT_MODEL_CNT];
};

struct stm32_keypad_dev {
	struct platform_device *pdev;
	struct input_dev *input_dev;
	struct mutex dev_lock;

	struct stm32_keypad_dtdata *dtdata;
	struct stm32_keypad_dtdata_row *dtdata_row;
	int key_state[STM32_KEY_MAX];
	struct notifier_block pogo_nb;
	char key_name[716][10];
	unsigned short *keycode;
	u32 support_keyboard_model[KDB_SUPPORT_MODEL_CNT];
	u32 keyboard_model;
	void (*release_all_key)(struct stm32_keypad_dev *stm32);
	int (*keypad_set_input_dev)(struct stm32_keypad_dev *stm32, struct pogo_data_struct pogo_data);
	void (*pogo_kpd_event)(struct stm32_keypad_dev *stm32, u16 *key_values, int len);
};

struct stm32_keypad_dev_function {
	void (*release_all_key_func)(struct stm32_keypad_dev *stm32);
	int (*keypad_set_input_dev_func)(struct stm32_keypad_dev *stm32, struct pogo_data_struct pogo_data);
	void (*pogo_kpd_event_func)(struct stm32_keypad_dev *stm32, u16 *key_values, int len);
};

struct stm32_keypad_dev_function *setup_stm32_keypad_dev_row(void);
struct stm32_keypad_dev_function *setup_stm32_keypad_dev_bypass(void);
int stm32_caps_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
void stm32_setup_probe_keypad(struct stm32_keypad_dev *stm32);
int stm32_setup_attach_keypad(struct stm32_keypad_dev *stm32, struct pogo_data_struct pogo_data);

#endif
