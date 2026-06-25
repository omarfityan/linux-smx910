// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus sm_dc direct-charging engine - the PD/PPS CC/CV control loop
 * that steps the PPS input voltage to drive the SM5440 2:1 charge-pump for
 * ~45 W fast charging.
 *
 * Ported (PD path only) from the device's own downstream
 * drivers/battery/charger/sm5440_charger/sm5440_direct_charger.c.  The
 * control-law math and state machine are transcribed verbatim; the downstream
 * Samsung sec-battery glue (psy_do_property / POWER_SUPPLY_EXT_PROP_* /
 * SEC_DIRECT_CHG_MODE_*) does not exist in mainline and is replaced at three
 * seams:
 *   - report_dc_state():     a dev/pr log of the state transition (no supervisor)
 *   - pd_check_vbat_work():  go straight to PRESET (the caller has already
 *                            inhibited the buck and confirmed the PD contract)
 *   - sec_done_event_work(): invoke a registered done_cb instead of the
 *                            POWER_SUPPLY_EXT_PROP_DIRECT_DONE property
 * The wireless (WPC) path, the manual/bypass CV mode, and set_ta_volt_by_soc are
 * dropped (no wireless charging on gts9u).
 *
 * The engine is hardware-agnostic: it touches the SM5440 only through the
 * sm_dc_ops vtable and commands the PPS source through the single
 * send_power_source_msg op (which the chip driver routes to the SM5714 PD
 * contract).  It carries no dependency on any specific charger/PD driver.
 *
 * Copyright (C) 2020 Silicon Mitus Co.Ltd
 * Copyright (C) 2026 omar fityan <me@omarfityan.com> (mainline PD-only port)
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "sm5440_direct_charger.h"

/*
 * The transcribed control law uses loose MAX()/MIN() (the device-own header's
 * macros).  The kernel's <linux/minmax.h> already provides uppercase MAX()/MIN()
 * with identical loose-ternary semantics (no type checking), so use those rather
 * than redefining them.
 */

/**
 *  Internal support functions for Direct-charging (PD3.0 PPS).
 */
static u32 pps_v(u32 vol)
{
	if ((vol % PPS_V_STEP) >= (PPS_V_STEP / 2))
		vol += PPS_V_STEP;

	return (vol / PPS_V_STEP) * PPS_V_STEP;
}

static u32 pps_c(u32 cur)
{
	if ((cur % PPS_C_STEP) >= (PPS_C_STEP / 2))
		cur += PPS_C_STEP;

	return (cur / PPS_C_STEP) * PPS_C_STEP;
}

/* Seam #1: no sec-battery supervisor in mainline - log the state transition. */
static const char *sm_dc_state_str(u8 state)
{
	switch (state) {
	case SM_DC_CHG_OFF:	return "CHG_OFF";
	case SM_DC_ERR:		return "ERR";
	case SM_DC_EOC:		return "EOC";
	case SM_DC_CHECK_VBAT:	return "CHECK_VBAT";
	case SM_DC_PRESET:	return "PRESET";
	case SM_DC_PRE_CC:	return "PRE_CC";
	case SM_DC_UPDAT_BAT:	return "UPDAT_BAT";
	case SM_DC_CC:		return "CC";
	case SM_DC_CV:		return "CV";
	default:		return "?";
	}
}

static int report_dc_state(struct sm_dc_info *sm_dc)
{
	static u8 prev_state = SM_DC_CHG_OFF;

	if (prev_state != sm_dc->state) {
		pr_info("%s %s: state %s -> %s\n", sm_dc->name, __func__,
			sm_dc_state_str(prev_state), sm_dc_state_str(sm_dc->state));
		prev_state = sm_dc->state;
	}

	return 0;
}

static int request_state_work(struct sm_dc_info *sm_dc, u8 state, u32 delay)
{
	int ret = 0;

	mutex_lock(&sm_dc->st_lock);

	switch (state) {
	case SM_DC_CHECK_VBAT:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->check_vbat_work,
				msecs_to_jiffies(delay));
		break;
	case SM_DC_PRESET:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->preset_dc_work,
				msecs_to_jiffies(delay));
		break;
	case SM_DC_PRE_CC:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->pre_cc_work,
				msecs_to_jiffies(delay));
		break;
	case SM_DC_CC:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->cc_work,
				msecs_to_jiffies(delay));
		break;
	case SM_DC_CV:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->cv_work,
				msecs_to_jiffies(delay));
		break;
	case SM_DC_UPDAT_BAT:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->update_bat_work,
				msecs_to_jiffies(delay));
		break;
	case SM_DC_ERR:
		queue_delayed_work(sm_dc->dc_wqueue, &sm_dc->error_work,
				msecs_to_jiffies(delay));
		break;
	default:
		pr_err("%s %s: invalid state(%d)\n", sm_dc->name, __func__, state);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&sm_dc->st_lock);

	return ret;
}

static int update_work_state(struct sm_dc_info *sm_dc, u8 state)
{
	int ret = 0;

	if (sm_dc->state == SM_DC_CHG_OFF) {
		pr_err("%s %s: detected chg_off, terminate work\n", sm_dc->name, __func__);
		return -EBUSY;
	}

	pr_info("%s %s: sm_dc->state=%d, state=%d, update(%d,%d,%d)\n",
		sm_dc->name, __func__, sm_dc->state, state, sm_dc->req_update_vbat,
		sm_dc->req_update_ibus, sm_dc->req_update_ibat);

	if (sm_dc->state > SM_DC_PRESET &&
			(sm_dc->req_update_vbat | sm_dc->req_update_ibus | sm_dc->req_update_ibat)) {
		pr_info("%s %s: changed chg param, request: update_bat\n", sm_dc->name, __func__);
		request_state_work(sm_dc, SM_DC_UPDAT_BAT, DELAY_NONE);
		ret = -EINVAL;
	}

	if (sm_dc->state != state) {
		mutex_lock(&sm_dc->st_lock);
		sm_dc->state = state;
		mutex_unlock(&sm_dc->st_lock);
		report_dc_state(sm_dc);
	}

	return ret;
}

static int send_power_source_msg(struct sm_dc_info *sm_dc)
{
	int ret;

	if (sm_dc->state < SM_DC_CHECK_VBAT)
		return -EINVAL;

	if (sm_dc->ta.v > sm_dc->ta.v_max || sm_dc->ta.c > sm_dc->ta.c_max) {
		pr_err("%s %s: ERROR: out of bounce v=%dmV(max=%dmV) c=%dmA(max=%dmA)\n",
			sm_dc->name, __func__, sm_dc->ta.v, sm_dc->ta.v_max,
			sm_dc->ta.c, sm_dc->ta.c_max);

		sm_dc->err = SM_DC_ERR_SEND_PD_MSG;
		request_state_work(sm_dc, SM_DC_ERR, DELAY_NONE);
		return -EINVAL;
	}

	pr_info("%s %s: [send PWR_MSG] pdo=%d, v=%dmV(max=%dmV) c=%dmA(max=%dmA)\n",
			sm_dc->name, __func__, sm_dc->ta.pdo_pos, sm_dc->ta.v,
			sm_dc->ta.v_max, sm_dc->ta.c, sm_dc->ta.c_max);

	ret = sm_dc->ops->send_power_source_msg(sm_dc->i2c, &sm_dc->ta);
	if (ret < 0) {
		pr_err("%s %s: fail to send msg(ret=%d)\n", sm_dc->name, __func__, ret);
		sm_dc->err = SM_DC_ERR_SEND_PD_MSG;
		request_state_work(sm_dc, SM_DC_ERR, DELAY_NONE);
	}

	return ret;
}

static int setup_direct_charging_work_config(struct sm_dc_info *sm_dc)
{
	sm_dc->wq.pps_cl = 0;
	sm_dc->wq.c_down = 0;
	sm_dc->wq.c_up = 0;
	sm_dc->wq.v_down = 0;
	sm_dc->wq.v_up = 0;
	sm_dc->wq.prev_adc_ibus = 0;
	sm_dc->wq.prev_adc_vbus = 0;
	sm_dc->wq.cc_limit = 0;
	sm_dc->wq.cc_cnt = 0;
	sm_dc->wq.cv_cnt = 0;
	sm_dc->wq.cv_gl = sm_dc->target_vbat;
	sm_dc->wq.ci_gl = MIN(sm_dc->ta.c_max, ((sm_dc->target_ibus * 100) / 100));
	sm_dc->wq.cc_gl = sm_dc->wq.ci_gl * 2;

	pr_info("%s %s: CV_GL=%dmV, CI_GL=%dmA, CC_GL=%dmA\n", sm_dc->name, __func__,
			sm_dc->wq.cv_gl, sm_dc->wq.ci_gl, sm_dc->wq.cc_gl);
	sm_dc->ops->set_charging_config(sm_dc->i2c, sm_dc->wq.cv_gl, sm_dc->wq.ci_gl, sm_dc->wq.cc_gl);

	return 0;
}

static int check_error_state(struct sm_dc_info *sm_dc, u8 retry_state)
{
	int adc_vbat;

	if (sm_dc->state == SM_DC_ERR) {
		pr_err("%s %s: already occurred error (err=0x%x)\n", sm_dc->name, __func__, sm_dc->err);
		return -EINVAL;
	}

	sm_dc->err = sm_dc->ops->get_dc_error_status(sm_dc->i2c);
	if (sm_dc->err == SM_DC_ERR_RETRY) {
		pr_err("%s %s: error status retry, wait 2sec\n", sm_dc->name, __func__);
		request_state_work(sm_dc, retry_state, DELAY_RETRY);
		return -EAGAIN;
	} else if (sm_dc->err > SM_DC_ERR_NONE) {
		pr_err("%s %s: error status:0x%x\n", sm_dc->name, __func__, sm_dc->err);
		request_state_work(sm_dc, SM_DC_ERR, DELAY_NONE);
		return -EPERM;
	}

	adc_vbat = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBAT);
	if (adc_vbat <= sm_dc->config.dc_min_vbat) {
		pr_err("%s %s: abnormal adc_vbat(%d)\n", sm_dc->name, __func__, adc_vbat);
		sm_dc->err = SM_DC_ERR_INVAL_VBAT;
		request_state_work(sm_dc, SM_DC_ERR, DELAY_NONE);
		return -ERANGE;
	}

	return 0;
}

static int get_adc_values(struct sm_dc_info *sm_dc, const char *str, int *vbus, int *ibus, int *vout,
		int *vbat, int *ibat, int *them, int *dietemp)
{
	int adc_vbus, adc_ibus, adc_vout, adc_vbat, adc_ibat, adc_them, adc_dietemp;

	adc_vbus = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBUS);
	adc_ibus = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_IBUS);
	adc_vout = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VOUT);
	adc_vbat = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBAT);
	adc_ibat = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_IBAT);
	adc_them = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_THEM);
	adc_dietemp = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_DIETEMP);

	pr_info("%s %s:vbus:%d:ibus:%d:vout:%d:vbat:%d:ibat:%d:them:%d:dietemp:%d\n",
		sm_dc->name, str, adc_vbus, adc_ibus, adc_vout,
		adc_vbat, adc_ibat, adc_them, adc_dietemp);

	if (vbus)
		*vbus = adc_vbus;
	if (ibus)
		*ibus = adc_ibus;
	if (vout)
		*vout = adc_vout;
	if (vbat)
		*vbat = adc_vbat;
	if (ibat)
		*ibat = adc_ibat;
	if (them)
		*them = adc_them;
	if (dietemp)
		*dietemp = adc_dietemp;

	return 0;
}

static int terminate_charging_work(struct sm_dc_info *sm_dc)
{
	flush_workqueue(sm_dc->dc_wqueue);

	cancel_delayed_work_sync(&sm_dc->check_vbat_work);
	cancel_delayed_work_sync(&sm_dc->preset_dc_work);
	cancel_delayed_work_sync(&sm_dc->pre_cc_work);
	cancel_delayed_work_sync(&sm_dc->cc_work);
	cancel_delayed_work_sync(&sm_dc->cv_work);
	cancel_delayed_work_sync(&sm_dc->update_bat_work);
	cancel_delayed_work_sync(&sm_dc->error_work);

	sm_dc->ops->set_charging_enable(sm_dc->i2c, 0);

	return 0;
}

/**
 *  PD3.0 PPS Direct-charging work functions
 */
static inline u32 _calc_pps_v_init_offset(struct sm_dc_info *sm_dc)
{
	u32 offset;

	offset = (sm_dc->ta.c * sm_dc->config.r_ttl) / 1000000;
	offset += 200;   /* add to extra initial offset */
	pr_info("%s %s: pps_c=%dmA, v_init_offset=%dmV\n", sm_dc->name, __func__, sm_dc->ta.c, offset);

	return offset;
}

static inline int _adjust_pps_v(struct sm_dc_info *sm_dc, int pps_v_original)
{
	int adc_vbus;

	adc_vbus = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBUS);

	pr_info("%s %s: adc_vbus=%dmV, pps_v_original=%dmV\n",
		sm_dc->name, __func__, adc_vbus, pps_v_original);

	sm_dc->wq.v_offset = 0;
	return 0;
}

static inline int _pd_pre_cc_check_limitation(struct sm_dc_info *sm_dc, int adc_ibus, int adc_vbus)
{
	u32 calc_pps_v, calc_reg_v, calc_pps_v_min, calc_reg_v_min;
	int ret = 0;

	if (adc_ibus == sm_dc->wq.prev_adc_ibus && adc_vbus == sm_dc->wq.prev_adc_vbus) {
		pr_info("%s %s: adc didn't update yet\n", sm_dc->name, __func__);
		/* if continuos adc didn't update yet */
		return 0;
	}

	if ((adc_vbus < sm_dc->wq.prev_adc_vbus + PPS_V_STEP) &&
			(adc_ibus < sm_dc->wq.prev_adc_ibus + PPS_C_STEP) &&
			(sm_dc->wq.v_down == 0 && sm_dc->wq.pps_cl == 0)) {
		ret = 1;
	} else if (sm_dc->wq.ci_gl == sm_dc->config.ta_min_current) {     /* Case: didn't reduce PPS_C than TA_MIN_C */
		ret = 1;
	}

	if (ret) {
		calc_reg_v = (sm_dc->wq.ci_gl * sm_dc->config.r_ttl) / 1000000;
		calc_pps_v = (sm_dc->target_vbat * 2) + calc_reg_v + sm_dc->wq.v_offset;
		if ((pps_v(calc_pps_v) * sm_dc->wq.ci_gl) > sm_dc->ta.p_max) {
			pr_info("%s %s: calc_pps_v(%dmV) will be reduced\n", sm_dc->name, __func__, calc_pps_v);
			calc_reg_v_min = (sm_dc->config.ta_min_current * sm_dc->config.r_ttl) / 1000000;
			calc_pps_v_min = (sm_dc->target_vbat * 2) + calc_reg_v_min + sm_dc->wq.v_offset;
			calc_pps_v = MAX((sm_dc->ta.p_max / sm_dc->wq.ci_gl) - PPS_V_STEP, calc_pps_v_min);
		}
		/*
		 * Mainline addition (NOT in the device-own engine): also clamp the
		 * limitation-path request to ta.v_max.  Downstream this path clamps only
		 * to the OVP ceiling because that engine never throttles via ta.v_max (it
		 * equals the APDO max).  Our adaptive supervisor repurposes ta.v_max as a
		 * runtime throttle, so the request must honour it HERE too -- otherwise a
		 * low ta.v_max stalls the PRE_CC ramp, which triggers this very jump to the
		 * ci_gl-derived target (which exceeds the throttle), and
		 * send_power_source_msg rejects it as out-of-bounds (SM_DC_ERR_SEND_PD_MSG).
		 * Clamping is conservative (only ever LOWERS the request) and a no-op when
		 * ta.v_max is the APDO max, so the device-own behaviour is unchanged.
		 */
		sm_dc->ta.v = pps_v(MIN(MIN(calc_pps_v, sm_dc->config.dc_vbus_ovp_th - 500),
					sm_dc->ta.v_max));
		pr_info("%s %s: R_TTL=%d, calc_reg_v=%dmV, calc_pps_v=%dmV, v_max=%dmV\n",
			sm_dc->name, __func__, sm_dc->config.r_ttl, calc_reg_v, calc_pps_v,
			sm_dc->ta.v_max);
		sm_dc->ta.c = sm_dc->ta.c;
		sm_dc->wq.pps_cl = 1;
	}

	return ret;
}

static inline int _try_to_adjust_cc_up(struct sm_dc_info *sm_dc)
{
	sm_dc->wq.cc_cnt += 1;

	if ((sm_dc->wq.cc_cnt % 2) && (sm_dc->ta.c <= sm_dc->wq.ci_gl + (PPS_C_STEP * 4))
		&& (sm_dc->ta.c != sm_dc->ta.c_max)) {
		if ((sm_dc->ta.v * (sm_dc->ta.c + PPS_C_STEP) <= (sm_dc->ta.p_max / 100) * 107)) {
			sm_dc->ta.c += PPS_C_STEP;
			if (sm_dc->ta.c > sm_dc->ta.c_max)
				sm_dc->ta.c = sm_dc->ta.c_max;
		}
	} else {
		/* TA P_MAX + 7% */
		if ((sm_dc->ta.v + (PPS_V_STEP * 2)) * sm_dc->ta.c <= (sm_dc->ta.p_max / 100) * 107) {
			sm_dc->ta.v += PPS_V_STEP * 2;
			if (sm_dc->ta.v > MIN(sm_dc->ta.v_max, sm_dc->config.dc_vbus_ovp_th - 500))
				sm_dc->ta.v = pps_v(MIN(sm_dc->ta.v_max, sm_dc->config.dc_vbus_ovp_th - 500));
		} else {
			pr_info("%s %s: PPS-TA has been reached limitation(v=%dmV, c=%dmA)\n",
			sm_dc->name, __func__, sm_dc->ta.v, sm_dc->ta.c);
			sm_dc->wq.cc_limit = 1;
			return -EINVAL;
		}
	}

	return 0;
}

static inline void _try_to_adjust_cc_down(struct sm_dc_info *sm_dc)
{
	sm_dc->wq.cc_cnt += 1;

	if ((sm_dc->wq.cc_cnt % 2) && (sm_dc->ta.c >= sm_dc->wq.ci_gl - (PPS_C_STEP * 4))) {
		if (sm_dc->ta.c - PPS_C_STEP >= sm_dc->config.ta_min_current)
			sm_dc->ta.c -= PPS_C_STEP;
	} else {
		if (sm_dc->ta.v > sm_dc->config.ta_min_voltage)
			sm_dc->ta.v -= PPS_V_STEP;
	}
}

/*
 * Seam #2: the downstream check_vbat asks the sec-battery supervisor whether the
 * switching (buck) charger is disabled yet, retrying until it is.  In this port
 * the caller (the pump driver's engage path) has already inhibited the buck and
 * confirmed a live PD contract BEFORE sm_dc_start_charging(), so this collapses
 * to a vbat read + an immediate transition to PRESET (the downstream code also
 * always falls through to PRESET, having force-cleared the supervisor reply).
 */
static void pd_check_vbat_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, check_vbat_work.work);
	int adc_vbat;
	int ret;

	ret = update_work_state(sm_dc, SM_DC_CHECK_VBAT);
	if (ret < 0)
		return;

	adc_vbat = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBAT);
	pr_info("%s %s: adc:vbat=%dmV [request] check_vbat -> preset\n",
		sm_dc->name, __func__, adc_vbat);

	request_state_work(sm_dc, SM_DC_PRESET, DELAY_NONE);
}

static void pd_preset_dc_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, preset_dc_work.work);
	int adc_vbat, ret, delay = DELAY_PPS_UPDATE;

	ret = update_work_state(sm_dc, SM_DC_PRESET);
	if (ret < 0)
		return;

	adc_vbat = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBAT);
	if (adc_vbat > sm_dc->target_vbat + PPS_V_STEP) {     /* Debounce ADC accuracy */
		pr_err("%s %s: adc_vbat(%dmV) > target_v, can't start dc\n", sm_dc->name, __func__, adc_vbat);
		sm_dc->err = SM_DC_ERR_INVAL_VBAT;
		request_state_work(sm_dc, SM_DC_ERR, DELAY_NONE);
		return;
	}
	pr_info("%s %s: adc_vbat=%dmV, ta_min_v=%dmV, v_max=%dmA, c_max=%dmA, target_ibus=%dmA\n",
			sm_dc->name, __func__, adc_vbat, sm_dc->config.ta_min_voltage, sm_dc->ta.v_max,
			sm_dc->ta.c_max, sm_dc->target_ibus);

	sm_dc->ta.c = MIN(sm_dc->ta.c_max, ((sm_dc->target_ibus * 50) / 100));
	sm_dc->ta.c = pps_c(MAX(sm_dc->ta.c, sm_dc->config.ta_min_current));
	sm_dc->ta.v = (2 * adc_vbat) + _calc_pps_v_init_offset(sm_dc);
	sm_dc->ta.v = pps_v(MAX(sm_dc->ta.v, sm_dc->config.ta_min_voltage));
	ret = send_power_source_msg(sm_dc);
	if (ret < 0)
		return;

	msleep(DELAY_PPS_UPDATE);

	if (sm_dc->ops->get_charging_enable(sm_dc->i2c) == 0x0)
		_adjust_pps_v(sm_dc, sm_dc->ta.v);

	if (sm_dc->target_ibus < sm_dc->wq.ci_gl)
		delay = DELAY_ADC_UPDATE;

	setup_direct_charging_work_config(sm_dc);
	sm_dc->ops->set_charging_enable(sm_dc->i2c, 1);
	pr_info("%s %s: enable Direct-charging\n", sm_dc->name, __func__);
	/* Pre-update PRE_CC state. for check to charging initial error case */
	ret = update_work_state(sm_dc, SM_DC_PRE_CC);
	if (ret < 0)
		return;

	pr_info("%s %s: [request] preset -> pre_cc\n", sm_dc->name, __func__);
	request_state_work(sm_dc, SM_DC_PRE_CC, delay);
}

static void pd_pre_cc_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, pre_cc_work.work);
	int ret, adc_ibus, adc_vbus, adc_vbat;
	int delay_time = DELAY_PPS_UPDATE;
	u8 loop_status;

	pr_info("%s %s: (CI_GL=%dmA)\n", sm_dc->name, __func__, sm_dc->wq.ci_gl);
	ret = check_error_state(sm_dc, SM_DC_PRE_CC);
	if (ret < 0)
		return;

	ret = update_work_state(sm_dc, SM_DC_PRE_CC);
	if (ret < 0)
		return;

	get_adc_values(sm_dc, "[adc-values]:pre_cc_work", &adc_vbus, &adc_ibus, NULL,
			&adc_vbat, NULL, NULL, NULL);
	loop_status = sm_dc->ops->get_dc_loop_status(sm_dc->i2c);
	switch (loop_status) {
	case LOOP_VBATREG:
		sm_dc->wq.cv_cnt = 0;
		pr_info("%s %s: [request] pre-cc -> cv\n", sm_dc->name, __func__);
		sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_CONTINUOUS);
		request_state_work(sm_dc, SM_DC_CV, DELAY_ADC_UPDATE);
		return;
	case LOOP_IBUSLIM:
		if (sm_dc->config.need_to_sw_ocp && sm_dc->wq.v_down == 1) {
			pr_info("%s %s: call check_sw_ocp\n", sm_dc->name, __func__);
			ret = sm_dc->ops->check_sw_ocp(sm_dc->i2c);
			if (ret < 0)
				return;
		}
		sm_dc->wq.c_offset = 0;
		if (sm_dc->ta.v - (PPS_V_STEP * 2) >= sm_dc->config.ta_min_voltage) {
			sm_dc->ta.v -= PPS_V_STEP * 2;
			sm_dc->wq.v_down = 1;
			sm_dc->wq.v_up = 0;
		} else {
			sm_dc->ta.v = sm_dc->config.ta_min_voltage;
			pr_info("%s %s: can't use less then ta_min_voltage\n", sm_dc->name, __func__);
			pr_info("%s %s: [request] pre_cc -> cc\n", sm_dc->name, __func__);
			sm_dc->wq.v_down = 0;
			sm_dc->wq.pps_cl = 0;
			sm_dc->wq.cc_limit = 0;
			sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_CONTINUOUS);
			request_state_work(sm_dc, SM_DC_CC, DELAY_ADC_UPDATE);
			return;
		}
		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;
		sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_CONTINUOUS);
		request_state_work(sm_dc, SM_DC_PRE_CC, DELAY_PPS_UPDATE);
		return;
	}

	_pd_pre_cc_check_limitation(sm_dc, adc_ibus, adc_vbus);

	if (adc_ibus > sm_dc->wq.ci_gl) {
		sm_dc->wq.cc_limit = 0;
		if (sm_dc->wq.ci_gl > sm_dc->ta.c)
			sm_dc->wq.c_offset = sm_dc->wq.ci_gl - sm_dc->ta.c;
		else
			sm_dc->wq.c_offset = 0;

		pr_info("%s %s: [request] pre_cc -> cc (c_offset=%d, pps_cl=%d)\n", sm_dc->name,
				__func__, sm_dc->wq.c_offset, sm_dc->wq.pps_cl);
		sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_CONTINUOUS);
		request_state_work(sm_dc, SM_DC_CC, DELAY_ADC_UPDATE);
		return;
	}

	if ((sm_dc->wq.pps_cl) &&
		((sm_dc->ta.v * (sm_dc->ta.c + PPS_C_STEP) > sm_dc->ta.p_max) ||
		((sm_dc->ta.c + PPS_C_STEP) > sm_dc->ta.c_max) ||
		(sm_dc->ta.c > sm_dc->wq.ci_gl + PRE_CC_ST_IBUS_OFFSET))) {
		sm_dc->wq.c_offset = 0;
		sm_dc->wq.cc_limit = 0;
		pr_info("%s %s: [request] pre_cc -> cc\n", sm_dc->name, __func__);
		sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_CONTINUOUS);
		request_state_work(sm_dc, SM_DC_CC, DELAY_ADC_UPDATE);
		return;
	}

	if (sm_dc->wq.pps_cl) {
		if ((adc_ibus < sm_dc->wq.ci_gl - (PPS_C_STEP * 6)) &&
				(sm_dc->ta.c < ((sm_dc->wq.ci_gl * 85) / 100)))
			sm_dc->ta.c += (PPS_C_STEP * 3);
		else
			sm_dc->ta.c += PPS_C_STEP;

		if (sm_dc->ta.c > sm_dc->ta.c_max)
			sm_dc->ta.c = sm_dc->ta.c_max;
		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;

		sm_dc->wq.c_up = 1;
		sm_dc->wq.c_down = 0;
	} else {
		sm_dc->ta.v += PPS_V_STEP;
		if (sm_dc->ta.v > MIN(sm_dc->ta.v_max, sm_dc->config.dc_vbus_ovp_th - 500)) {
			pr_info("%s %s: can't increase voltage(v:%d, v_max:%d)\n",
				sm_dc->name, __func__, sm_dc->ta.v, sm_dc->ta.v_max);
			sm_dc->ta.v = pps_v(MIN(sm_dc->ta.v_max, sm_dc->config.dc_vbus_ovp_th - 500));
			sm_dc->wq.pps_cl = 1;
		}

		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;

		sm_dc->wq.v_up = 1;
		sm_dc->wq.v_down = 0;
	}

	sm_dc->wq.prev_adc_vbus = adc_vbus;
	sm_dc->wq.prev_adc_ibus = adc_ibus;
	request_state_work(sm_dc, SM_DC_PRE_CC, delay_time);
}

static void pd_cc_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, cc_work.work);
	int ret, adc_ibus, adc_vbus, adc_vbat;
	u8 loop_status;

	pr_info("%s %s\n", sm_dc->name, __func__);

	ret = check_error_state(sm_dc, SM_DC_CC);
	if (ret < 0)
		return;

	ret = update_work_state(sm_dc, SM_DC_CC);
	if (ret < 0)
		return;

	get_adc_values(sm_dc, "[adc-values]:cc_work", &adc_vbus, &adc_ibus, NULL,
			&adc_vbat, NULL, NULL, NULL);

	loop_status = sm_dc->ops->get_dc_loop_status(sm_dc->i2c);
	switch (loop_status) {
	case LOOP_VBATREG:
		sm_dc->wq.cv_cnt = 0;
		pr_info("%s %s: [request] cc -> cv\n", sm_dc->name, __func__);
		request_state_work(sm_dc, SM_DC_CV, DELAY_ADC_UPDATE);
		return;
	case LOOP_IBUSLIM:
		if (sm_dc->config.need_to_sw_ocp) {
			pr_info("%s %s: call check_sw_ocp\n", sm_dc->name, __func__);
			ret = sm_dc->ops->check_sw_ocp(sm_dc->i2c);
			if (ret < 0)
				return;
		}
		break;
	}

	/* CC_STEP_DOWN */
	if (sm_dc->wq.ci_gl < adc_ibus) {
		_try_to_adjust_cc_down(sm_dc);
		if (sm_dc->config.support_pd_remain) {
			ret = send_power_source_msg(sm_dc);
			if (ret < 0)
				return;
		}
		request_state_work(sm_dc, SM_DC_CC, DELAY_ADC_UPDATE);
		return;
	}

	if (adc_ibus >= sm_dc->wq.ci_gl - CC_ST_IBUS_OFFSET || sm_dc->wq.cc_limit) {
		if (sm_dc->config.support_pd_remain) {
			ret = send_power_source_msg(sm_dc);
			if (ret < 0)
				return;
		}
		request_state_work(sm_dc, SM_DC_CC, DELAY_CHG_LOOP);
		return;
	}

	/* CC_STEP_UP */
	ret = _try_to_adjust_cc_up(sm_dc);
	if (ret < 0) {
		if (sm_dc->config.support_pd_remain) {
			ret = send_power_source_msg(sm_dc);
			if (ret < 0)
				return;
		}
		request_state_work(sm_dc, SM_DC_CC, DELAY_CHG_LOOP);
	} else {
		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;

		request_state_work(sm_dc, SM_DC_CC, DELAY_ADC_UPDATE);
	}
}

static void pd_cv_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, cv_work.work);
	int ret, adc_ibus, adc_vbus, adc_vbat, delay = DELAY_CHG_LOOP;
	u8 loop_status;

	pr_info("%s %s\n", sm_dc->name, __func__);

	ret = check_error_state(sm_dc, SM_DC_CV);
	if (ret < 0)
		return;

	ret = update_work_state(sm_dc, SM_DC_CV);
	if (ret < 0)
		return;

	get_adc_values(sm_dc, "[adc-values]:cv_work", &adc_vbus, &adc_ibus, NULL,
			&adc_vbat, NULL, NULL, NULL);

	loop_status = sm_dc->ops->get_dc_loop_status(sm_dc->i2c);
	if ((sm_dc->wq.cv_cnt == 0) && (loop_status & LOOP_VBATREG))
		sm_dc->wq.cv_cnt = 1;
	else if ((sm_dc->wq.cv_cnt == 1) && (loop_status == LOOP_INACTIVE))
		sm_dc->wq.cv_cnt = 2;

	if (loop_status & LOOP_VBATREG) {
		if (sm_dc->wq.cv_cnt == 1)
			/* fast decrease PPS_V during on the first vbatreg loop */
			sm_dc->ta.v -= PPS_V_STEP * 2;
		else
			sm_dc->ta.v -= PPS_V_STEP;

		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;

		delay = DELAY_PPS_UPDATE;
	} else if (loop_status & LOOP_THEMREG) {
		if (sm_dc->config.support_pd_remain) {
			ret = send_power_source_msg(sm_dc);
			if (ret < 0)
				return;
		}
		delay = DELAY_PPS_UPDATE;
	}

	/* occurred abnormal CV status */
	if ((adc_vbat < sm_dc->target_vbat - 100) || (loop_status & LOOP_IBUSLIM)) {
		pr_info("%s %s: adnormal cv, [request] cv -> pre_cc\n", sm_dc->name, __func__);
		sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_ONESHOT);
		request_state_work(sm_dc, SM_DC_PRE_CC, DELAY_ADC_UPDATE);
		return;
	}

	/* Support to "DIRECT_DONE" used ADC_IBUS */
	if (sm_dc->config.topoff_current > 0) {
		if ((sm_dc->target_vbat == sm_dc->config.chg_float_voltage) &&
			(adc_ibus < sm_dc->config.topoff_current)) {
			pr_info("%s : dc done!!\n", __func__);
			schedule_delayed_work(&sm_dc->done_event_work, msecs_to_jiffies(50));
		}
	}

	if (loop_status == LOOP_INACTIVE && sm_dc->config.support_pd_remain) {
		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;
	}
	request_state_work(sm_dc, SM_DC_CV, delay);
}

static void pd_update_bat_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, update_bat_work.work);
	int index, ret, cnt;
	bool need_to_preset = 1;

	pr_info("%s %s\n", sm_dc->name, __func__);

	ret = check_error_state(sm_dc, SM_DC_UPDAT_BAT);
	if (ret < 0)
		return;

	/* waiting for step change event */
	for (cnt = 0; cnt < 1; ++cnt) {
		if (sm_dc->req_update_vbat && sm_dc->req_update_ibus)
			break;

		pr_info("%s %s: wait 1sec for step changed\n", sm_dc->name, __func__);
		msleep(1000);
	}

	mutex_lock(&sm_dc->st_lock);
	index = (sm_dc->req_update_vbat << 2) | (sm_dc->req_update_ibus << 1) | sm_dc->req_update_ibat;
	sm_dc->req_update_vbat = 0;
	sm_dc->req_update_ibus = 0;
	sm_dc->req_update_ibat = 0;
	mutex_unlock(&sm_dc->st_lock);

	ret = update_work_state(sm_dc, SM_DC_UPDAT_BAT);
	if (ret < 0)
		return;

	if (index & (0x1 << 2))
		pr_info("%s %s: vbat changed (%dmV)\n", sm_dc->name, __func__, sm_dc->target_vbat);

	if (index & (0x1 << 1))
		pr_info("%s %s: ibus changed (%dmA)\n", sm_dc->name, __func__, sm_dc->target_ibus);

	if (index & 0x1)
		pr_info("%s %s: ibat changed (%dmA)\n", sm_dc->name, __func__, sm_dc->target_ibat);

	/* check step change event */
	if ((index & (0x1 << 2)) && (index & (0x1 << 1))) {
		if ((sm_dc->target_vbat > sm_dc->wq.cv_gl) && (sm_dc->target_ibus < sm_dc->wq.ci_gl))
			need_to_preset = 0;
	}

	sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_ONESHOT);

	if (need_to_preset) {
		pr_info("%s %s: [request] update_bat -> preset\n", sm_dc->name, __func__);
		request_state_work(sm_dc, SM_DC_PRESET, DELAY_NONE);
	} else {
		setup_direct_charging_work_config(sm_dc);
		sm_dc->ta.c = pps_c(sm_dc->wq.ci_gl - 200 - sm_dc->wq.c_offset);
		ret = send_power_source_msg(sm_dc);
		if (ret < 0)
			return;

		pr_info("%s %s: [request] update_bat -> pre_cc\n", sm_dc->name,  __func__);
		request_state_work(sm_dc, SM_DC_PRE_CC, DELAY_ADC_UPDATE);
	}
}

static void pd_error_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, error_work.work);
	int ret;

	pr_info("%s %s: err=0x%x\n", sm_dc->name, __func__, sm_dc->err);

	ret = update_work_state(sm_dc, SM_DC_ERR);
	if (ret < 0)
		return;

	sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_OFF);
	sm_dc->ops->set_charging_enable(sm_dc->i2c, 0);
}

/*
 * Seam #3: the downstream reports topoff to the sec-battery supervisor via
 * psy_do_property(POWER_SUPPLY_EXT_PROP_DIRECT_DONE).  Mainline has no such
 * supervisor; invoke the registered done_cb instead (the pump driver / Inc-3
 * supervisor uses it to stop the engine and hand back to the buck).
 */
static void sec_done_event_work(struct work_struct *work)
{
	struct sm_dc_info *sm_dc = container_of(work, struct sm_dc_info, done_event_work.work);

	pr_info("%s %s: charging done\n", sm_dc->name, __func__);

	if (sm_dc->done_cb)
		sm_dc->done_cb(sm_dc->done_ctx);
}

/**
 * SM Direct-charging module management APIs
 */
struct sm_dc_info *sm_dc_create_pd_instance(const char *name, struct i2c_client *i2c)
{
	struct sm_dc_info *sm_dc;
	int ret;

	sm_dc = kzalloc(sizeof(struct sm_dc_info), GFP_KERNEL);
	if (!sm_dc)
		return ERR_PTR(-ENOMEM);

	sm_dc->name = name;
	sm_dc->i2c = i2c;
	mutex_init(&sm_dc->st_lock);

	/* create work queue */
	sm_dc->state = SM_DC_CHG_OFF;
	sm_dc->dc_wqueue = create_singlethread_workqueue(name);
	if (!sm_dc->dc_wqueue) {
		pr_err("%s %s: fail to crearte workqueue\n", name, __func__);
		ret = -ENOMEM;
		goto err_kmem;
	}
	INIT_DELAYED_WORK(&sm_dc->check_vbat_work,  pd_check_vbat_work);
	INIT_DELAYED_WORK(&sm_dc->preset_dc_work,   pd_preset_dc_work);
	INIT_DELAYED_WORK(&sm_dc->pre_cc_work,      pd_pre_cc_work);
	INIT_DELAYED_WORK(&sm_dc->cc_work,          pd_cc_work);
	INIT_DELAYED_WORK(&sm_dc->cv_work,          pd_cv_work);
	INIT_DELAYED_WORK(&sm_dc->update_bat_work,  pd_update_bat_work);
	INIT_DELAYED_WORK(&sm_dc->error_work,       pd_error_work);
	INIT_DELAYED_WORK(&sm_dc->done_event_work,  sec_done_event_work);
	pr_info("%s %s: done.\n", name, __func__);

	return sm_dc;

err_kmem:
	mutex_destroy(&sm_dc->st_lock);
	kfree(sm_dc);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(sm_dc_create_pd_instance);

int sm_dc_verify_configuration(struct sm_dc_info *sm_dc)
{
	if (sm_dc == NULL)
		return -EINVAL;

	if (sm_dc->ops == NULL)
		return -EINVAL;

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_verify_configuration);

void sm_dc_destroy_instance(struct sm_dc_info *sm_dc)
{
	if (sm_dc != NULL) {
		destroy_workqueue(sm_dc->dc_wqueue);
		mutex_destroy(&sm_dc->st_lock);
		kfree(sm_dc);
	}
}
EXPORT_SYMBOL_GPL(sm_dc_destroy_instance);

void sm_dc_set_done_notify(struct sm_dc_info *sm_dc,
			   void (*done)(void *ctx), void *ctx)
{
	sm_dc->done_cb = done;
	sm_dc->done_ctx = ctx;
}
EXPORT_SYMBOL_GPL(sm_dc_set_done_notify);

int sm_dc_report_error_status(struct sm_dc_info *sm_dc, u32 err)
{
	terminate_charging_work(sm_dc);
	sm_dc->state = SM_DC_ERR;
	sm_dc->err = err;
	report_dc_state(sm_dc);

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_report_error_status);

int sm_dc_report_interrupt_event(struct sm_dc_info *sm_dc, u32 interrupt)
{
	if ((sm_dc->state == SM_DC_CC) && (interrupt == SM_DC_INT_VBATREG)) {
		if (delayed_work_pending(&sm_dc->cc_work)) {
			cancel_delayed_work(&sm_dc->cc_work);
			pr_info("%s %s: cancel CC_work, direct request work\n", sm_dc->name, __func__);
			request_state_work(sm_dc, SM_DC_CC, DELAY_NONE);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_report_interrupt_event);

int sm_dc_get_current_state(struct sm_dc_info *sm_dc)
{
	return sm_dc->state;
}
EXPORT_SYMBOL_GPL(sm_dc_get_current_state);

int sm_dc_start_charging(struct sm_dc_info *sm_dc, struct sm_dc_power_source_info *ta)
{
	if (sm_dc->state >= SM_DC_CHECK_VBAT) {
		pr_err("%s %s: already work on dc (state=%d)\n", sm_dc->name, __func__, sm_dc->state);
		return -EBUSY;
	}

	sm_dc->ta.pdo_pos = ta->pdo_pos;
	sm_dc->ta.v_max = ta->v_max;
	sm_dc->ta.c_max = ta->c_max;
	sm_dc->ta.p_max = ta->p_max;

	sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_ONESHOT);
	mutex_lock(&sm_dc->st_lock);
	sm_dc->state = SM_DC_CHECK_VBAT;   /* Pre-update chg.state */
	mutex_unlock(&sm_dc->st_lock);
	request_state_work(sm_dc, SM_DC_CHECK_VBAT, DELAY_PPS_UPDATE);

	pr_info("%s %s: done\n", sm_dc->name, __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_start_charging);

int sm_dc_stop_charging(struct sm_dc_info *sm_dc)
{
	mutex_lock(&sm_dc->st_lock);
	sm_dc->state = SM_DC_CHG_OFF;
	mutex_unlock(&sm_dc->st_lock);
	terminate_charging_work(sm_dc);
	sm_dc->ops->set_adc_mode(sm_dc->i2c, SM_DC_ADC_MODE_OFF);

	report_dc_state(sm_dc);

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_stop_charging);

int sm_dc_set_target_vbat(struct sm_dc_info *sm_dc, u32 target_vbat)
{
	int ret = 0;
	int adc_vbat;

	pr_info("%s %s: [%dmV] to [%dmV]\n", sm_dc->name, __func__, sm_dc->target_vbat, target_vbat);

	sm_dc->target_vbat = target_vbat;
	if (sm_dc->state > SM_DC_CHECK_VBAT) {
		adc_vbat = sm_dc->ops->get_adc_value(sm_dc->i2c, SM_DC_ADC_VBAT);
		if (sm_dc->target_vbat > adc_vbat) {
			mutex_lock(&sm_dc->st_lock);
			sm_dc->req_update_vbat = 1;
			mutex_unlock(&sm_dc->st_lock);
			pr_info("%s %s: request VBAT update on DC work\n", sm_dc->name, __func__);
		} else {
			pr_err("%s %s: target_vbat(%dmV) less then adc_vbat(%dmV)\n", sm_dc->name, __func__,
					sm_dc->target_vbat, adc_vbat);
			ret = -EINVAL;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sm_dc_set_target_vbat);

int sm_dc_set_target_ibus(struct sm_dc_info *sm_dc, u32 target_ibus)
{
	pr_info("%s %s: [%dmA] to [%dmA]\n", sm_dc->name, __func__, sm_dc->target_ibus, target_ibus);

	sm_dc->target_ibus = target_ibus;
	if (sm_dc->state > SM_DC_CHECK_VBAT) {
		mutex_lock(&sm_dc->st_lock);
		sm_dc->req_update_ibus = 1;
		mutex_unlock(&sm_dc->st_lock);
		pr_info("%s %s: request IBUS update on DC work\n", sm_dc->name, __func__);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_set_target_ibus);

int sm_dc_set_ta_vmax(struct sm_dc_info *sm_dc, u32 v_max)
{
	pr_info("%s %s: [%dmV] to [%dmV]\n", sm_dc->name, __func__, sm_dc->ta.v_max, v_max);

	/*
	 * A bare ceiling write -- NO req_update flag, NO state change (cf.
	 * sm_dc_set_target_ibus, whose req_update_ibus diverts the machine to a full
	 * PRE_CC re-ramp).  st_lock for consistency with the other setters; ta.v_max
	 * is a single aligned word the CC reader (_try_to_adjust_cc_up) sees
	 * atomically, so the worst a concurrent read sees is the previous ceiling for
	 * one iteration, corrected on the next.
	 */
	mutex_lock(&sm_dc->st_lock);
	sm_dc->ta.v_max = v_max;
	mutex_unlock(&sm_dc->st_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(sm_dc_set_ta_vmax);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Silicon Mitus; mainline PD-only port by omar fityan <me@omarfityan.com>");
MODULE_DESCRIPTION("Silicon Mitus sm_dc direct-charging (PD/PPS CC/CV) engine");
