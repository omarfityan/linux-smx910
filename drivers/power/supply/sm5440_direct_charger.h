/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Silicon Mitus sm_dc direct-charging engine - the PD/PPS CC/CV control loop
 * that steps the PPS input voltage to drive the SM5440 2:1 charge-pump for
 * 45 W fast charging.
 *
 * Ported verbatim (PD path only) from the device's own downstream
 * drivers/battery/charger/sm5440_charger/sm5440_direct_charger.h.  The engine
 * is hardware-agnostic: it touches the SM5440 only through the sm_dc_ops vtable
 * and commands the PPS source through the single send_power_source_msg op.  The
 * downstream wireless (WPC) path is dropped (no wireless charging on gts9u).
 *
 * Copyright (C) 2020 Silicon Mitus Co.Ltd
 * Copyright (C) 2026 omar fityan <me@omarfityan.com> (mainline PD-only port)
 */
#ifndef __SM5440_DIRECT_CHARGER_H__
#define __SM5440_DIRECT_CHARGER_H__

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#define PPS_V_STEP		20	/* PPS request voltage granularity, mV */
#define PPS_C_STEP		50	/* PPS request current granularity, mA */

#define PRE_CC_ST_IBUS_OFFSET	150
#define CC_ST_IBUS_OFFSET	100

enum sm_dc_charging_loop {
	LOOP_IBUSLIM		= (0x1 << 7),	/* CC input-current loop active */
	LOOP_VBATREG		= (0x1 << 3),	/* CV loop active */
	LOOP_THEMREG		= (0x1 << 1),	/* thermal regulation active */
	LOOP_INACTIVE		= (0x0),
};

enum sm_dc_work_delay_type {
	DELAY_NONE		= 0,
	DELAY_PPS_UPDATE	= 250,
	DELAY_ADC_UPDATE	= 1100,
	DELAY_RETRY		= 2000,
	DELAY_CHG_LOOP		= 2500,
};

enum sm_dc_state {
	SM_DC_CHG_OFF		= 0x0,
	SM_DC_ERR,
	SM_DC_EOC,		/* vestigial in the PD path (never entered) */
	SM_DC_CHECK_VBAT,
	SM_DC_PRESET,
	SM_DC_PRE_CC,
	SM_DC_UPDAT_BAT,
	SM_DC_CC,
	SM_DC_CV,
};

enum sm_dc_err_index {
	SM_DC_ERR_NONE		= (0x0),
	SM_DC_ERR_VBATOVP	= (0x1 << 3),
	SM_DC_ERR_VOUTOVP	= (0x1 << 4),
	SM_DC_ERR_IBATOCP	= (0x1 << 6),
	SM_DC_ERR_IBUSOCP	= (0x1 << 7),
	SM_DC_ERR_CFLY_SHORT	= (0x1 << 8),
	SM_DC_ERR_REVBLK	= (0x1 << 9),
	SM_DC_ERR_STUP_FAIL	= (0x1 << 10),
	SM_DC_ERR_VBUSUVLO	= (0x1 << 14),
	SM_DC_ERR_VBUSOVP	= (0x1 << 15),
	SM_DC_ERR_INVAL_VBAT	= (0x1 << 16),
	SM_DC_ERR_SEND_PD_MSG	= (0x1 << 17),
	SM_DC_ERR_FAIL_ADJUST	= (0x1 << 18),
	SM_DC_ERR_RETRY		= (0x1 << 30),
	SM_DC_ERR_UNKNOWN	= (0x1 << 31),
};

enum sm_dc_interrupt_index {
	SM_DC_INT_VBATREG	= (0x1 << 0),
	SM_DC_INT_WDTOFF	= (0x1 << 1),
};

enum sm_dc_adc_channel {
	SM_DC_ADC_THEM		= 0x0,
	SM_DC_ADC_DIETEMP,
	SM_DC_ADC_VBAT,
	SM_DC_ADC_IBAT,
	SM_DC_ADC_VOUT,
	SM_DC_ADC_VBUS,
	SM_DC_ADC_IBUS,
};

enum sm_dc_adc_mode {
	SM_DC_ADC_MODE_ONESHOT		= 0x0,
	SM_DC_ADC_MODE_CONTINUOUS	= 0x1,
	SM_DC_ADC_MODE_OFF		= 0x2,
};

struct sm_dc_power_source_info {
	u32 pdo_pos;
	u32 v_max;
	u32 c_max;
	u32 p_max;
	u32 v;
	u32 c;
};

struct sm_dc_ops {
	int (*get_adc_value)(struct i2c_client *i2c, u8 adc_ch);
	int (*set_adc_mode)(struct i2c_client *i2c, u8 mode);
	int (*get_charging_enable)(struct i2c_client *i2c);
	int (*set_charging_enable)(struct i2c_client *i2c, bool enable);
	int (*set_charging_config)(struct i2c_client *i2c, u32 cv_gl, u32 ci_gl, u32 cc_gl);
	u32 (*get_dc_error_status)(struct i2c_client *i2c);
	int (*get_dc_loop_status)(struct i2c_client *i2c);
	int (*send_power_source_msg)(struct i2c_client *i2c, struct sm_dc_power_source_info *ta);
	int (*check_sw_ocp)(struct i2c_client *i2c);
};

struct sm_dc_info {
	const char *name;
	struct i2c_client *i2c;
	struct mutex st_lock;
	const struct sm_dc_ops *ops;

	/* direct-charging state machine */
	struct workqueue_struct *dc_wqueue;
	struct delayed_work check_vbat_work;
	struct delayed_work preset_dc_work;
	struct delayed_work pre_cc_work;
	struct delayed_work cc_work;
	struct delayed_work cv_work;
	struct delayed_work update_bat_work;
	struct delayed_work error_work;
	struct delayed_work done_event_work;

	u8 state;
	u32 err;
	bool req_update_vbat;
	bool req_update_ibus;
	bool req_update_ibat;
	u32 target_vbat;
	u32 target_ibat;
	u32 target_ibus;

	struct sm_dc_power_source_info ta;

	/* state-machine control scratch */
	struct {
		bool pps_cl;
		bool c_up;
		bool c_down;
		bool v_up;
		bool v_down;
		int v_offset;
		int c_offset;
		u16 prev_adc_ibus;
		u16 prev_adc_vbus;
		bool cc_limit;
		int cc_cnt;
		int cv_cnt;
		u32 cv_gl;
		u32 ci_gl;
		u32 cc_gl;
	} wq;

	struct {
		u32 ta_min_current;
		u32 ta_min_voltage;
		u32 dc_min_vbat;
		u32 dc_vbus_ovp_th;
		u32 r_ttl;
		u32 topoff_current;
		bool need_to_sw_ocp;
		bool support_pd_remain;
		u32 chg_float_voltage;
		char *sec_dc_name;
	} config;

	/*
	 * "charging done" notify.  The downstream engine reports topoff to the
	 * sec-battery supervisor via psy_do_property(POWER_SUPPLY_EXT_PROP_DIRECT_DONE);
	 * mainline has no such supervisor, so the pump driver registers a callback
	 * here (sm_dc_set_done_notify) that the CV topoff event invokes instead.
	 */
	void (*done_cb)(void *ctx);
	void *done_ctx;
};

extern struct sm_dc_info *sm_dc_create_pd_instance(const char *name, struct i2c_client *i2c);
extern int sm_dc_verify_configuration(struct sm_dc_info *sm_dc);
extern void sm_dc_destroy_instance(struct sm_dc_info *sm_dc);

extern int sm_dc_report_error_status(struct sm_dc_info *sm_dc, u32 err);
extern int sm_dc_report_interrupt_event(struct sm_dc_info *sm_dc, u32 interrupt);

extern int sm_dc_get_current_state(struct sm_dc_info *sm_dc);
extern int sm_dc_start_charging(struct sm_dc_info *sm_dc, struct sm_dc_power_source_info *ta);
extern int sm_dc_stop_charging(struct sm_dc_info *sm_dc);
extern int sm_dc_set_target_vbat(struct sm_dc_info *sm_dc, u32 target_vbat);
extern int sm_dc_set_target_ibus(struct sm_dc_info *sm_dc, u32 target_ibus);

/* The "done" callback the topoff event invokes (supervisor stops the engine). */
extern void sm_dc_set_done_notify(struct sm_dc_info *sm_dc,
				  void (*done)(void *ctx), void *ctx);

#endif /* __SM5440_DIRECT_CHARGER_H__ */
