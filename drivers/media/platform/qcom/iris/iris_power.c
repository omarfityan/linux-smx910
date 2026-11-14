// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/moduleparam.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-mem2mem.h>

#include "iris_buffer.h"
#include "iris_instance.h"
#include "iris_power.h"
#include "iris_resources.h"
#include "iris_vpu_common.h"

/* TEST-ONLY: clock-bound dose-response switches. 0 = unchanged behaviour. */
static unsigned int test_freq_floor_hz;
module_param(test_freq_floor_hz, uint, 0644);
MODULE_PARM_DESC(test_freq_floor_hz, "TEST: floor (Hz) for the summed VPU core clock request; 0 = off");

static unsigned int test_bw_fps;
module_param(test_bw_fps, uint, 0644);
MODULE_PARM_DESC(test_bw_fps, "TEST: fps used in the DDR bandwidth vote instead of DEFAULT_FPS; 0 = off");

static unsigned int test_freq_floor_logged;

/* TEST-ONLY: start at the formula row instead of descending from the top row */
static bool test_start_floor;
module_param(test_start_floor, bool, 0644);
MODULE_PARM_DESC(test_start_floor, "TEST: skip the start-at-max descent, start at the formula row (positive control)");

/*
 * Decoder clock scaling. From the device's downstream msm_vidc driver
 * (vendor/qcom/opensource/video-driver: vidc/src/msm_vidc_power.c,
 * vidc/inc/msm_vidc_internal.h, vidc/inc/msm_vidc_buffer.h): the input-rate
 * fps, the firmware-held capture-buffer condition, one-row steps and
 * start-at-max. Everything else (the trial step-up below) is ours.
 */
static bool dcvs = true;
module_param(dcvs, bool, 0644);
MODULE_PARM_DESC(dcvs, "Decoder clock scaling (downstream input rate + buffer condition, trial step-up); 0 = upstream behaviour");

#define IRIS_DCVS_DEC_EXTRA_OUTPUT_BUFFERS	4	/* msm_vidc_buffer.h:18 */

/*
 * Trial step-up: ours, pinned 2026-11-14, not downstream values.
 */
#define IRIS_DCVS_WINDOW_INPUTS			16	/* one window */
#define IRIS_DCVS_UTIL_UP_PCT			80	/* trigger: U >= 80 % ... */
#define IRIS_DCVS_UTIL_UP_WINDOWS		3	/* ... in 3 consecutive windows */
#define IRIS_DCVS_MIN_DWELL_INPUTS		128	/* inputs since last step/reset */
#define IRIS_DCVS_TRIAL_SKIP_INPUTS		16	/* skipped after a trial step */
#define IRIS_DCVS_TRIAL_KEEP_PCT		103	/* keep if R_trial >= 1.03 R_base */
#define IRIS_DCVS_BACKOFF_MIN			512	/* inputs */
#define IRIS_DCVS_BACKOFF_MAX			16384	/* inputs */
#define IRIS_DCVS_KEEP_SURVIVE_INPUTS		256	/* keep survives -> back-off reset */
#define IRIS_DCVS_DOWN_PCT			90	/* L x f_cur / f_lower < 0.90 P_in */
#define IRIS_DCVS_DOWN_WINDOWS			8
#define IRIS_DCVS_DESCENT_WINDOWS		2	/* start-up descent dwell */
#define IRIS_DCVS_INPUT_GAP_NS			(500ULL * NSEC_PER_MSEC)

enum iris_dcvs_state {
	IRIS_DCVS_DESCENT,
	IRIS_DCVS_STEADY,
	IRIS_DCVS_TRIAL,
};

static u64 iris_max_freq(struct iris_core *core);
static u64 iris_ceil_freq(struct iris_core *core, u64 freq);

/* one table row above / below freq (0 if none) */
static u64 iris_row_up(struct iris_core *core, u64 freq)
{
	unsigned long f = (unsigned long)min_t(u64, freq, ULONG_MAX - 1) + 1;
	struct dev_pm_opp *opp;

	opp = dev_pm_opp_find_freq_ceil(core->dev, &f);
	if (IS_ERR(opp))
		return 0;
	dev_pm_opp_put(opp);

	return f;
}

static u64 iris_row_down(struct iris_core *core, u64 freq)
{
	unsigned long f;
	struct dev_pm_opp *opp;

	if (!freq)
		return 0;
	f = (unsigned long)min_t(u64, freq, ULONG_MAX) - 1;
	opp = dev_pm_opp_find_freq_floor(core->dev, &f);
	if (IS_ERR(opp))
		return 0;
	dev_pm_opp_put(opp);

	return f;
}

/* clear the measurement windows (seek / flush / DRC / drain / gap) */
static void iris_dcvs_clear_windows(struct iris_inst_power *p, u64 now)
{
	p->busy_ns = 0;
	p->last_close_ns = now;
	p->win_start_ns = now;
	p->win_inputs = 0;
	p->up_wins = 0;
	p->down_wins = 0;
	p->descent_down_wins = 0;
	p->descent_hold_wins = 0;
	p->arr_cnt = 0;
	p->arr_idx = 0;
	p->rate_x10 = 0;
	p->svc_n = 0;
	p->last_capture_ns = 0;
	p->inputs_since_step = 0;
}

/* resets: clear windows, hold the row, abort a trial without penalty */
static void iris_dcvs_soft_reset(struct iris_inst_power *p, u64 now)
{
	if (p->state == IRIS_DCVS_TRIAL) {
		p->row_freq = p->pre_trial_row;
		p->state = IRIS_DCVS_STEADY;
	}
	iris_dcvs_clear_windows(p, now);
	p->pend_valid = 0;
	p->in_flight = 0;
}

/* stream start / end: everything back to the start-up state */
void iris_power_data_reset(struct iris_inst *inst)
{
	struct iris_inst_power *p = &inst->power;
	u64 now = ktime_get_ns();

	p->dcvs_flags = 0;
	p->buffer_counter = 0;
	p->input_rate = 0;
	p->last_input_ns = 0;
	p->input_idx = 0;
	p->input_count = 0;
	p->in_flight = 0;
	p->pend_valid = 0;
	p->last_lat_us = 0;
	p->unmatched = 0;
	p->util_pct = 0;
	p->l_p75_ns = 0;
	p->backoff_len = IRIS_DCVS_BACKOFF_MIN;
	p->backoff_left = 0;
	p->keep_recent = false;
	p->keep_age = 0;
	p->trials = 0;
	p->keeps = 0;
	p->reverts = 0;
	p->buf_full = false;
	p->inputs_since_descent = 0;
	p->unmeasured_wins = 0;
	p->fallback_logged = false;
	iris_dcvs_clear_windows(p, now);

	if (READ_ONCE(test_start_floor)) {
		/* row 0 = take the formula row at the first scale_clocks */
		p->row_freq = 0;
		p->state = IRIS_DCVS_STEADY;
		p->descent_done = true;
	} else {
		p->row_freq = iris_max_freq(inst->core);
		p->state = IRIS_DCVS_DESCENT;
		p->descent_done = false;
	}
	p->started = true;
}

void iris_power_streamon(struct iris_inst *inst, bool src)
{
	if (src)
		WRITE_ONCE(inst->power.src_streaming, true);
	if (!inst->power.started)
		iris_power_data_reset(inst);
	else
		iris_dcvs_soft_reset(&inst->power, ktime_get_ns());
}

void iris_power_streamoff(struct iris_inst *inst, bool capture)
{
	if (capture) {
		/*
		 * stream end: vote nothing until the next streamon / first input
		 * re-seeds the row (a stopped instance stays on core->instances)
		 */
		iris_power_data_reset(inst);
		WRITE_ONCE(inst->power.row_freq, 0);
		WRITE_ONCE(inst->power.started, false);
	} else {
		/*
		 * input (src) streamoff: this instance stops voting; a stale row
		 * from a paused or dying instance must not hold the clock up
		 */
		iris_dcvs_soft_reset(&inst->power, ktime_get_ns());
		WRITE_ONCE(inst->power.src_streaming, false);
		WRITE_ONCE(inst->power.row_freq, 0);
		WRITE_ONCE(inst->power.started, false);
	}
}

/*
 * Record the input picture just queued (its timestamp was stored by
 * iris_set_ts_metadata() immediately before, in iris_vdec_qbuf()); used as
 * a picture cookie to match its capture return.
 */
static void iris_dcvs_pend_add(struct iris_inst *inst, u64 now)
{
	struct iris_inst_power *p = &inst->power;
	u32 idx = inst->metadata_idx ? inst->metadata_idx - 1 : ARRAY_SIZE(inst->tss) - 1;
	u32 slot;

	/* age-out: a picture pending > 500 ms will never be matched */
	for (slot = 0; slot < IRIS_DCVS_PENDING; slot++) {
		if ((p->pend_valid & BIT(slot)) && now > p->pend_qbuf_ns[slot] &&
		    now - p->pend_qbuf_ns[slot] > IRIS_DCVS_INPUT_GAP_NS) {
			p->pend_valid &= ~BIT(slot);
			p->unmatched++;
		}
	}

	if (p->pend_valid == U32_MAX) {
		p->pend_valid = 0;
		p->unmatched++;
	}
	slot = ffz(p->pend_valid);
	p->pend_ts_us[slot] = inst->tss[idx].ts_us;
	p->pend_ts_ns[slot] = inst->tss[idx].ts_ns;
	p->pend_qbuf_ns[slot] = now;
	p->pend_valid |= BIT(slot);
	p->in_flight = hweight32(p->pend_valid);
}

/* R = (n - 1) / span over the rolling arrival ring, x10 per second */
static void iris_dcvs_arrival(struct iris_inst_power *p, u64 now)
{
	u64 oldest, span;

	p->arr_ns[p->arr_idx] = now;
	p->arr_idx = (p->arr_idx + 1) % IRIS_DCVS_ARRIVALS;
	if (p->arr_cnt < IRIS_DCVS_ARRIVALS)
		p->arr_cnt++;
	if (p->arr_cnt < 2) {
		p->rate_x10 = 0;
		return;
	}
	oldest = p->arr_ns[(p->arr_idx + IRIS_DCVS_ARRIVALS - p->arr_cnt) % IRIS_DCVS_ARRIVALS];
	span = now > oldest ? now - oldest : 0;
	p->rate_x10 = span ? (u32)min_t(u64, div64_u64((u64)(p->arr_cnt - 1) * 10 * NSEC_PER_SEC,
						       span), U32_MAX) : 0;
}

/* p75 of the window's service-time samples (insertion sort, <= 16 entries) */
static u64 iris_dcvs_svc_p75(struct iris_inst_power *p)
{
	u64 s[IRIS_DCVS_SVC_SAMPLES], v;
	u32 n = p->svc_n, i, j;

	if (!n)
		return 0;
	for (i = 0; i < n; i++) {
		v = p->svc_ns[i];
		for (j = i; j > 0 && s[j - 1] > v; j--)
			s[j] = s[j - 1];
		s[j] = v;
	}

	return s[(n * 3) / 4 < n ? (n * 3) / 4 : n - 1];
}

/*
 * DOWN test: L x f_cur / f_lower < 0.90 P_in, with P_in = 1 / R.
 * False when there is no lower row, the lower row is under the formula row,
 * or R / L are not measured yet.
 */
static bool iris_dcvs_down_ok(struct iris_inst *inst)
{
	struct iris_inst_power *p = &inst->power;
	u64 f_cur = p->row_freq, f_lower, f_formula, l_scaled, p_in_ns;

	if (!f_cur || !p->rate_x10 || !p->l_p75_ns)
		return false;
	f_lower = iris_row_down(inst->core, f_cur);
	if (!f_lower)
		return false;
	f_formula = p->base_freq ? iris_ceil_freq(inst->core, p->base_freq) : 0;
	if (f_lower < f_formula)
		return false;

	p_in_ns = div64_u64(10ULL * NSEC_PER_SEC, p->rate_x10);
	l_scaled = mul_u64_u64_div_u64(p->l_p75_ns, f_cur, f_lower);

	return l_scaled * 100 < (u64)IRIS_DCVS_DOWN_PCT * p_in_ns;
}

static void iris_dcvs_revert_penalty(struct iris_inst_power *p)
{
	p->reverts++;
	p->backoff_left = p->backoff_len;
	p->backoff_len = min_t(u32, p->backoff_len * 2, IRIS_DCVS_BACKOFF_MAX);
	p->keep_recent = false;
}

/* the state machine, run once per closed window (every 16 input QBUFs) */
static void iris_dcvs_decide(struct iris_inst *inst)
{
	struct iris_inst_power *p = &inst->power;
	struct iris_core *core = inst->core;
	bool measured = p->rate_x10 && p->l_p75_ns;
	bool down = iris_dcvs_down_ok(inst);
	u64 up_row;

	/*
	 * R or L unmeasured for 8 windows: fall back to the formula row
	 * (what upstream votes) instead of holding whatever row we are on.
	 */
	if (p->state != IRIS_DCVS_TRIAL) {
		if (measured) {
			p->unmeasured_wins = 0;
		} else if (++p->unmeasured_wins >= IRIS_DCVS_DOWN_WINDOWS) {
			p->unmeasured_wins = 0;
			if (p->base_freq)
				p->row_freq = iris_ceil_freq(core, p->base_freq);
			p->state = IRIS_DCVS_STEADY;
			if (!p->descent_done) {
				p->descent_done = true;
				p->inputs_since_descent = 0;
			}
			p->inputs_since_step = 0;
			if (!p->fallback_logged) {
				p->fallback_logged = true;
				dev_info(core->dev,
					 "dcvs: no service-time samples, using formula row\n");
			}
			return;
		}
	}

	if (p->arr_cnt < IRIS_DCVS_WINDOW_INPUTS && p->state != IRIS_DCVS_TRIAL)
		return;

	switch (p->state) {
	case IRIS_DCVS_DESCENT:
		/* a hold counts only when DOWN was evaluated (R and L measured) */
		if (!measured)
			break;
		if (down) {
			p->descent_hold_wins = 0;
			if (++p->descent_down_wins >= IRIS_DCVS_DESCENT_WINDOWS) {
				p->row_freq = iris_row_down(core, p->row_freq);
				p->descent_down_wins = 0;
				p->inputs_since_step = 0;
			}
		} else {
			p->descent_down_wins = 0;
			if (++p->descent_hold_wins >= IRIS_DCVS_DESCENT_WINDOWS) {
				p->state = IRIS_DCVS_STEADY;
				p->descent_done = true;
				p->inputs_since_descent = 0;
			}
		}
		break;

	case IRIS_DCVS_STEADY:
		p->down_wins = down ? p->down_wins + 1 : 0;
		if (p->down_wins >= IRIS_DCVS_DOWN_WINDOWS &&
		    p->inputs_since_step >= IRIS_DCVS_MIN_DWELL_INPUTS) {
			p->row_freq = iris_row_down(core, p->row_freq);
			p->down_wins = 0;
			p->up_wins = 0;
			p->inputs_since_step = 0;
			/* a DOWN within 256 inputs of a keep counts as a revert */
			if (p->keep_recent)
				iris_dcvs_revert_penalty(p);
			break;
		}

		p->up_wins = p->util_pct >= IRIS_DCVS_UTIL_UP_PCT ? p->up_wins + 1 : 0;
		up_row = iris_row_up(core, p->row_freq);
		if (p->up_wins >= IRIS_DCVS_UTIL_UP_WINDOWS &&
		    p->inputs_since_step >= IRIS_DCVS_MIN_DWELL_INPUTS &&
		    !p->backoff_left && p->descent_done &&
		    p->inputs_since_descent >= IRIS_DCVS_MIN_DWELL_INPUTS &&
		    up_row && p->buf_full) {
			p->r_base_x10 = p->rate_x10;
			p->pre_trial_row = p->row_freq;
			p->row_freq = up_row;
			p->state = IRIS_DCVS_TRIAL;
			p->trial_skip = IRIS_DCVS_TRIAL_SKIP_INPUTS;
			p->trials++;
			p->up_wins = 0;
			p->down_wins = 0;
			p->inputs_since_step = 0;
		}
		break;

	case IRIS_DCVS_TRIAL:
		/* decided per input in iris_dcvs_trial_input() */
		break;
	}
}

/* per input while in TRIAL: skip 16, then R_trial over the next 128 */
static void iris_dcvs_trial_input(struct iris_inst_power *p)
{
	if (p->trial_skip) {
		if (!--p->trial_skip) {
			p->arr_cnt = 0;
			p->arr_idx = 0;
			p->rate_x10 = 0;
		}
		return;
	}
	if (p->arr_cnt < IRIS_DCVS_ARRIVALS)
		return;

	if ((u64)p->rate_x10 * 100 >= (u64)p->r_base_x10 * IRIS_DCVS_TRIAL_KEEP_PCT) {
		p->keeps++;
		p->keep_recent = true;
		p->keep_age = 0;
	} else {
		p->row_freq = p->pre_trial_row;
		iris_dcvs_revert_penalty(p);
	}
	p->state = IRIS_DCVS_STEADY;
	p->inputs_since_step = 0;
	p->up_wins = 0;
	p->down_wins = 0;
}

/*
 * Close a window of 16 input QBUFs: U = busy_ns * f_cur / f_base / wall_ns
 * (percent), L = p75 of the window's service times, then decide.
 */
static void iris_dcvs_close_window(struct iris_inst *inst, u64 now)
{
	struct iris_inst_power *p = &inst->power;
	u64 wall_ns = now > p->win_start_ns ? now - p->win_start_ns : 0;
	/*
	 * core->power.clk_freq holds the vote, not necessarily a table row;
	 * round it up to the table row actually run.
	 */
	u64 f_req = READ_ONCE(inst->core->power.clk_freq);
	u64 f_cur = f_req ? iris_ceil_freq(inst->core, f_req) : 0;
	u64 f_base = p->base_freq ? iris_ceil_freq(inst->core, p->base_freq) : 0;
	u64 scaled;

	if (wall_ns && f_base && f_cur) {
		scaled = mul_u64_u64_div_u64(p->busy_ns, f_cur, f_base);
		p->util_pct = (u32)min_t(u64, div64_u64(scaled * 100, wall_ns), U32_MAX);
	}
	p->l_p75_ns = iris_dcvs_svc_p75(p);
	p->svc_n = 0;

	iris_dcvs_decide(inst);

	p->busy_ns = 0;
	p->last_close_ns = now;
	p->win_start_ns = now;
	p->win_inputs = 0;
}

/*
 * Input (bitstream) QBUF. Downstream's input rate over IRIS_INPUT_RATE_WINDOW
 * intervals (msm_vidc_update_input_rate(), msm_vidc_driver.c:2757-2798) for
 * the formula fps; plus our arrival ring, picture cookie and window clock.
 * Called with inst->lock held.
 */
void iris_power_update_input_rate(struct iris_inst *inst)
{
	struct iris_inst_power *p = &inst->power;
	u64 now = ktime_get_ns();
	u64 sum = 0;
	u32 i;

	if (p->buffer_counter < U32_MAX)
		p->buffer_counter++;

	if (!READ_ONCE(dcvs)) {
		p->input_rate = 0;
		p->input_count = 0;
		p->last_input_ns = now;
		return;
	}

	if (!p->started)
		iris_power_data_reset(inst);

	/* an input gap > 500 ms: clear windows, hold the row */
	if (p->last_input_ns && now > p->last_input_ns &&
	    now - p->last_input_ns > IRIS_DCVS_INPUT_GAP_NS)
		iris_dcvs_soft_reset(p, now);

	iris_dcvs_pend_add(inst, now);
	iris_dcvs_arrival(p, now);

	if (p->inputs_since_step < U32_MAX)
		p->inputs_since_step++;
	if (p->descent_done && p->inputs_since_descent < U32_MAX)
		p->inputs_since_descent++;
	if (p->backoff_left)
		p->backoff_left--;
	if (p->keep_recent && ++p->keep_age >= IRIS_DCVS_KEEP_SURVIVE_INPUTS) {
		/* a keep that survived 256 inputs resets the back-off */
		p->keep_recent = false;
		p->backoff_len = IRIS_DCVS_BACKOFF_MIN;
	}

	if (p->state == IRIS_DCVS_TRIAL)
		iris_dcvs_trial_input(p);

	if (++p->win_inputs >= IRIS_DCVS_WINDOW_INPUTS)
		iris_dcvs_close_window(inst, now);

	if (p->last_input_ns && now > p->last_input_ns) {
		p->input_intervals_us[p->input_idx] =
			(u32)min_t(u64, div_u64(now - p->last_input_ns, NSEC_PER_USEC), U32_MAX);
		p->input_idx = (p->input_idx + 1) % IRIS_INPUT_RATE_WINDOW;
		if (p->input_count < IRIS_INPUT_RATE_WINDOW)
			p->input_count++;
	}
	p->last_input_ns = now;

	if (p->input_count < IRIS_INPUT_RATE_WINDOW)
		return;

	for (i = 0; i < IRIS_INPUT_RATE_WINDOW; i++)
		sum += p->input_intervals_us[i];
	/*
	 * Clamp: a client that bursts its input pool (start-up, seek, priming)
	 * would otherwise publish a rate that wraps the u32 cycle maths in
	 * calculate_frequency. 240 is a bound chosen here, not a downstream value.
	 */
	if (sum)
		p->input_rate = (u32)min_t(u64, 240,
					   DIV64_U64_ROUND_CLOSEST((u64)IRIS_INPUT_RATE_WINDOW *
								   USEC_PER_SEC, sum));
}

/*
 * A filled CAPTURE buffer came back from firmware. Called from the gen2
 * response handler with inst->lock held (iris_hfi_gen2_handle_session_response).
 */
void iris_power_capture_done(struct iris_inst *inst, u64 fw_timestamp)
{
	struct iris_inst_power *p = &inst->power;
	u64 now = ktime_get_ns();
	u64 start, lat_ns, svc_start;
	u32 slot;

	if (!READ_ONCE(dcvs))
		return;

	for (slot = 0; slot < IRIS_DCVS_PENDING; slot++) {
		if (!(p->pend_valid & BIT(slot)))
			continue;
		if (p->pend_ts_us[slot] == fw_timestamp ||
		    p->pend_ts_ns[slot] == fw_timestamp)
			break;
	}
	if (slot == IRIS_DCVS_PENDING) {
		p->unmatched++;
		return;
	}

	lat_ns = now > p->pend_qbuf_ns[slot] ? now - p->pend_qbuf_ns[slot] : 0;

	/* busy += this picture's interval, clamped so intervals never overlap */
	start = max(p->pend_qbuf_ns[slot], p->last_close_ns);
	if (now > start)
		p->busy_ns += min(lat_ns, now - start);
	p->last_close_ns = now;

	/* service time: from max(input QBUF, previous capture done) */
	svc_start = max(p->pend_qbuf_ns[slot], p->last_capture_ns);
	if (now > svc_start && p->svc_n < IRIS_DCVS_SVC_SAMPLES)
		p->svc_ns[p->svc_n++] = now - svc_start;
	p->last_capture_ns = now;

	p->last_lat_us = (u32)min_t(u64, div_u64(lat_ns, NSEC_PER_USEC), U32_MAX);
	p->pend_valid &= ~BIT(slot);
	p->in_flight = hweight32(p->pend_valid);
}

/* downstream msm_vidc_max_freq(): the top row of the clock table */
static u64 iris_max_freq(struct iris_core *core)
{
	unsigned long f = ULONG_MAX;
	struct dev_pm_opp *opp;

	opp = dev_pm_opp_find_freq_floor(core->dev, &f);
	if (IS_ERR(opp))
		return 0;
	dev_pm_opp_put(opp);

	return f;
}

/* lowest table rate >= freq (the max rate if freq is above the table) */
static u64 iris_ceil_freq(struct iris_core *core, u64 freq)
{
	unsigned long f = (unsigned long)min_t(u64, freq, ULONG_MAX);
	struct dev_pm_opp *opp;

	opp = dev_pm_opp_find_freq_ceil(core->dev, &f);
	if (IS_ERR(opp))
		return iris_max_freq(core);
	dev_pm_opp_put(opp);

	return f;
}

/*
 * Downstream's buffer condition (msm_vidc_apply_dcvs(), msm_vidc_power.c:
 * 398-473; thresholds msm_vidc_dcvs_data_reset(), :620-633), now only a
 * gate ANDed onto any up-step. Called with inst->lock held, before the
 * buffer being queued is marked BUF_ATTR_QUEUED (hence the +1).
 */
static void iris_apply_dcvs(struct iris_inst *inst)
{
	struct iris_inst_power *p = &inst->power;
	struct v4l2_m2m_buffer *buffer, *n;
	struct iris_buffer *buf;
	u32 min_th, max_th, actual, bufs_with_fw = 0;

	actual = vb2_get_num_buffers(v4l2_m2m_get_dst_vq(inst->m2m_ctx));
	if (!actual) {
		p->buf_full = false;
		return;
	}
	min_th = inst->buffers[BUF_OUTPUT].min_count;
	max_th = min(min_th + IRIS_DCVS_DEC_EXTRA_OUTPUT_BUFFERS, actual);

	v4l2_m2m_for_each_dst_buf_safe(inst->m2m_ctx, buffer, n) {
		buf = to_iris_buffer(&buffer->vb);
		if (buf->attr & BUF_ATTR_QUEUED)
			bufs_with_fw++;
	}
	bufs_with_fw += 1;
	p->buf_full = bufs_with_fw >= max_th;

	dev_dbg(inst->core->dev,
		"dcvs: bufs_with_fw %u th[%u %u] actual %u input_rate %u U %u%% in_flight %u lat_us %u unmatched %u state %u row %llu R_x10 %u L_p75_us %llu trials %u keeps %u reverts %u backoff %u\n",
		bufs_with_fw, min_th, max_th, actual, p->input_rate, p->util_pct,
		p->in_flight, p->last_lat_us, p->unmatched, p->state, p->row_freq,
		p->rate_x10, div_u64(p->l_p75_ns, NSEC_PER_USEC), p->trials, p->keeps,
		p->reverts, p->backoff_left);
}

static u32 iris_calc_bw(struct iris_inst *inst, struct icc_vote_data *data)
{
	const struct bw_info *bw_tbl = NULL;
	struct iris_core *core = inst->core;
	u32 num_rows, i, mbs, mbps;
	u32 icc_bw = 0;

	mbs = DIV_ROUND_UP(data->height, 16) * DIV_ROUND_UP(data->width, 16);
	mbps = mbs * data->fps;
	if (mbps == 0)
		goto exit;

	bw_tbl = core->iris_platform_data->bw_tbl_dec;
	num_rows = core->iris_platform_data->bw_tbl_dec_size;

	for (i = 0; i < num_rows; i++) {
		if (i != 0 && mbps > bw_tbl[i].mbs_per_sec)
			break;

		icc_bw = bw_tbl[i].bw_ddr;
	}

exit:
	return icc_bw;
}

static int iris_set_interconnects(struct iris_inst *inst)
{
	struct iris_core *core = inst->core;
	struct iris_inst *instance;
	u64 total_bw_ddr = 0;
	int ret;

	mutex_lock(&core->lock);
	list_for_each_entry(instance, &core->instances, list) {
		if (!instance->max_input_data_size)
			continue;

		total_bw_ddr += instance->power.icc_bw;
	}

	ret = iris_set_icc_bw(core, total_bw_ddr);

	mutex_unlock(&core->lock);

	return ret;
}

static int iris_vote_interconnects(struct iris_inst *inst)
{
	struct icc_vote_data *vote_data = &inst->icc_data;
	struct v4l2_format *inp_f = inst->fmt_src;

	vote_data->width = inp_f->fmt.pix_mp.width;
	vote_data->height = inp_f->fmt.pix_mp.height;
	vote_data->fps = READ_ONCE(test_bw_fps) ? READ_ONCE(test_bw_fps) : DEFAULT_FPS;

	inst->power.icc_bw = iris_calc_bw(inst, vote_data);

	return iris_set_interconnects(inst);
}

static int iris_set_clocks(struct iris_inst *inst)
{
	struct iris_core *core = inst->core;
	struct iris_inst *instance;
	unsigned int floor_hz = READ_ONCE(test_freq_floor_hz);
	bool use_rows = READ_ONCE(dcvs);
	u64 freq = 0, row = 0;
	int ret;

	mutex_lock(&core->lock);
	list_for_each_entry(instance, &core->instances, list) {
		if (!instance->max_input_data_size)
			continue;

		freq += instance->power.min_freq;
		/* the row each decoder instance's state machine voted: max over instances */
		/*
		 * never touch another instance's vb2 / m2m state here: iris_close()
		 * frees its m2m ctx before the instance leaves core->instances
		 */
		if (use_rows && instance->domain == DECODER &&
		    READ_ONCE(instance->power.started) &&
		    READ_ONCE(instance->power.src_streaming))
			row = max(row, READ_ONCE(instance->power.row_freq));
	}

	if (row > freq)
		freq = row;

	if (floor_hz && freq < floor_hz)
		freq = floor_hz;
	if (floor_hz != test_freq_floor_logged) {
		test_freq_floor_logged = floor_hz;
		dev_info(core->dev,
			 "TEST freq floor now %u Hz (requested %llu Hz)\n",
			 floor_hz, freq);
	}

	core->power.clk_freq = freq;
	ret = iris_opp_set_rate(core->dev, freq);
	mutex_unlock(&core->lock);

	return ret;
}

static int iris_scale_clocks(struct iris_inst *inst)
{
	const struct vpu_ops *vpu_ops = inst->core->iris_platform_data->vpu_ops;
	struct v4l2_m2m_ctx *m2m_ctx = inst->m2m_ctx;
	struct v4l2_m2m_buffer *buffer, *n;
	struct iris_inst_power *p = &inst->power;
	struct iris_buffer *buf;
	size_t data_size = 0;

	v4l2_m2m_for_each_src_buf_safe(m2m_ctx, buffer, n) {
		buf = to_iris_buffer(&buffer->vb);
		data_size = max(data_size, buf->data_size);
	}

	inst->max_input_data_size = data_size;
	if (!inst->max_input_data_size)
		return 0;

	/* the formula's own rate: f_base of U and the floor of every DOWN */
	p->base_freq = vpu_ops->calc_freq(inst, inst->max_input_data_size);
	p->min_freq = p->base_freq;
	p->dcvs_flags = 0;

	if (inst->domain == DECODER && READ_ONCE(dcvs)) {
		if (!p->started)
			iris_power_data_reset(inst);
		/* test_start_floor: row 0 -> the formula row */
		if (!p->row_freq)
			p->row_freq = iris_ceil_freq(inst->core, p->base_freq);
		/* DRC / drain: clear windows, hold the row, abort a trial */
		if (inst->sub_state & (IRIS_INST_SUB_DRC | IRIS_INST_SUB_DRAIN))
			iris_dcvs_soft_reset(p, ktime_get_ns());
		iris_apply_dcvs(inst);
	} else {
		p->row_freq = 0;
	}

	return iris_set_clocks(inst);
}

int iris_scale_power(struct iris_inst *inst)
{
	struct iris_core *core = inst->core;
	int ret;

	if (pm_runtime_suspended(core->dev)) {
		ret = pm_runtime_resume_and_get(core->dev);
		if (ret < 0)
			return ret;

		pm_runtime_put_autosuspend(core->dev);
	}

	ret = iris_scale_clocks(inst);
	if (ret)
		return ret;

	return iris_vote_interconnects(inst);
}
