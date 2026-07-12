// SPDX-License-Identifier: GPL-2.0
/*
 * Silicon Mitus SM5440 switched-capacitor 2:1 charge-pump - telemetry driver.
 *
 * The SM5440 is the cell-side 2:1 charge pump on the Samsung Galaxy Tab S9
 * Ultra (SM-X910 / gts9u), used for high-power (45 W) fast charging in
 * series with the SM5714 primary charger.  This driver is the register /
 * ADC telemetry layer: it probes the chip, gates on its DEVICEID, enables
 * the on-chip ADC, and exposes input voltage / input current / die
 * temperature as a power_supply.
 *
 * It deliberately does NOT control charging.  The pump's power FETs are
 * gated solely by the op-mode field (CNTL5, bits[3:2]); this driver never
 * writes that register, so op-mode stays at its power-on CHG_OFF default and
 * the pump never switches.  The fast-charge control loop lands separately.
 *
 * Register addresses, ADC scaling and the DEVICEID gate are transcribed
 * verbatim from the device's own downstream driver,
 * drivers/battery/charger/sm5440_charger/sm5440_charger.c (silicon rev "WE2").
 *
 * Copyright (C) 2026 omar fityan <me@omarfityan.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/usb/pd.h>		/* RDO_PROG_{VOLT_MV,CURR_MA}_STEP grids */
#include <linux/workqueue.h>

/*
 * Cross-driver hook for the buck<->pump handoff: the SM5714 VBUS/charger driver
 * (the buck + MUIC AFC) lives in drivers/usb/misc and exposes no-op fallbacks
 * when not built, so the telemetry-only path still builds without it.  Included
 * by relative path for this bring-up (a shared include/linux/ header is a
 * planned cleanup before upstreaming).
 *
 * PPS voltage/current stepping no longer uses a bespoke cross-driver hook: the
 * pump drives mainline tcpm's "tcpm-source-psy" power-supply directly via the
 * generic power_supply API (see the PPS bridge below), so the bespoke
 * sm5714-typec.h is gone.
 */
#include "../../usb/misc/sm5714-usb-vbus.h"

/* The ported sm_dc CC/CV direct-charging engine (sm5440_direct_charger.c). */
#include "sm5440_direct_charger.h"

/* Register map (device-own sm5440_charger.h). */
#define SM5440_REG_INT1		0x00
#define SM5440_REG_INT2		0x01
#define SM5440_REG_INT3		0x02
#define SM5440_REG_INT4		0x03
#define SM5440_REG_STATUS1	0x08
#define SM5440_REG_STATUS2	0x09
#define SM5440_REG_STATUS3	0x0a
#define SM5440_REG_STATUS4	0x0b
#define SM5440_REG_CNTL1	0x0c
#define SM5440_REG_CNTL2	0x0d
#define SM5440_REG_CNTL3	0x0e
#define SM5440_REG_CNTL4	0x0f
#define SM5440_REG_CNTL5	0x10
#define SM5440_REG_CNTL6	0x11
#define SM5440_REG_CNTL7	0x12
#define SM5440_REG_VBUSCNTL	0x13
#define SM5440_REG_VBATCNTL	0x14
#define SM5440_REG_VOUTCNTL	0x15
#define SM5440_REG_IBUSCNTL	0x16
#define SM5440_REG_PRTNCNTL	0x19
#define SM5440_REG_THEMCNTL1	0x1a
#define SM5440_REG_ADCCNTL1	0x1c
#define SM5440_REG_ADCCNTL2	0x1d
#define SM5440_REG_ADC_VBUS1	0x1e
#define SM5440_REG_ADC_VBUS2	0x1f
#define SM5440_REG_ADC_VOUT1	0x20
#define SM5440_REG_ADC_VOUT2	0x21
#define SM5440_REG_ADC_IBUS1	0x22
#define SM5440_REG_ADC_IBUS2	0x23
#define SM5440_REG_ADC_THEM1	0x24
#define SM5440_REG_ADC_THEM2	0x25
#define SM5440_REG_ADC_DIETEMP	0x26
#define SM5440_REG_ADC_VBAT1	0x27
#define SM5440_REG_ADC_VBAT2	0x28
#define SM5440_REG_DEVICEID	0x2b

/* STATUS3: VBUS power-OK (device-own psy_chg_get_online: bit5). */
#define SM5440_STATUS3_VBUS_POK	BIT(5)

/* DEVICEID: low nibble == 1 means the chip is present; high nibble is rev. */
#define SM5440_DEVICEID_MASK	GENMASK(3, 0)
#define SM5440_DEVICEID_PRESENT	0x1
#define SM5440_DEVICEID_REV(x)	((x) >> 4)

/*
 * ADC enable, telemetry-only (device-own sm5440_init_reg_param +
 * enable_adc / set_adc_rate):
 *   ADCCNTL2 = 0xdf  - per-channel enable (device-own value, verbatim)
 *   ADCCNTL1 = 0x0b  - bit0 ADC enable | bit1 continuous rate | bit3 32-avg
 */
#define SM5440_ADCCNTL2_CHANNELS	0xdf
#define SM5440_ADCCNTL1_ENABLE		0x0b

/*
 * op-mode field (CNTL5 bits[3:2], device-own enum sm5440_op_mode).  CHG_OFF is
 * also the power-on/idle state; CHG_ON switches the 2:1 pump for forward
 * charging.  REV_* are reverse/D2D boost modes and are not used here.
 */
#define SM5440_OPMODE_SHIFT	2
#define SM5440_OPMODE_MASK	0x3
#define SM5440_OPMODE_CHG_OFF	0x0
#define SM5440_OPMODE_CHG_ON	0x1

/* WDT timer field (CNTL1 bits[6:4]); 0x4 == 30 s (device-own WDT_TIMER_S_30). */
#define SM5440_WDT_TIMER_30S	0x4

/*
 * Fault / status bits.  The INT1-4 registers are clear-on-read; their STATUS1-4
 * level mirrors carry the SAME bit layout and are re-readable, so a POLLED
 * driver (this board has no SM5440 irq-gpio) MUST read STATUS, never INT.
 */
#define SM5440_STATUS1_VOUTOVP	BIT(4)
#define SM5440_STATUS1_VBATOVP	BIT(3)
#define SM5440_STATUS2_IBUSLIM	BIT(7)	/* CC input-current loop active */
#define SM5440_STATUS2_VBATREG	BIT(3)	/* CV loop active */
#define SM5440_STATUS2_THEM_REG	BIT(1)	/* thermal regulation active */
#define SM5440_STATUS3_VBUS_POK	BIT(5)
#define SM5440_STATUS3_VBUSOVP	BIT(7)
#define SM5440_STATUS3_VBUSUVLO	BIT(6)
#define SM5440_STATUS3_THEMSHDN_ALM	BIT(4)	/* thermal-shutdown alarm (SW-OCP) */
#define SM5440_STATUS3_STUP_FAIL	BIT(2)
#define SM5440_STATUS3_REVBLK	BIT(1)
#define SM5440_STATUS3_CFLY_SHORT	BIT(0)
#define SM5440_STATUS4_MIDVBUS2VOUT	BIT(5)
#define SM5440_STATUS4_SW_RDY	BIT(4)

/* The five forced-cutoff faults to poll for during a pump engage. */
#define SM5440_FAULT_S1		(SM5440_STATUS1_VOUTOVP)
#define SM5440_FAULT_S3		(SM5440_STATUS3_VBUSOVP | SM5440_STATUS3_STUP_FAIL | \
				 SM5440_STATUS3_REVBLK | SM5440_STATUS3_CFLY_SHORT)

/*
 * init_reg_param values (device-own sm5440_init_reg_param), transcribed
 * verbatim.  CNTL2 disables HW IBUS/IBAT-OCP + THEM (SW OCP used instead);
 * CNTL6 forces PWM and clears ENHIZ; VBUSCNTL[2:0]=7 is the 11 V VBUS-OVP;
 * CNTL3 disables the charge-timer and enables VBATREG.
 */
#define SM5440_CNTL2_INIT	0xf2
#define SM5440_CNTL4_INIT	0xff
#define SM5440_CNTL6_INIT	0x09
#define SM5440_VOUTCNTL_INIT	0x3f
#define SM5440_PRTNCNTL_INIT	0xfe
#define SM5440_THEMCNTL1_INIT	0x0c
#define SM5440_CNTL3_INIT	0xb8
#define SM5440_VBUSCNTL_OVP_11V	0x7	/* VBUSCNTL bits[2:0] */

/*
 * Per-register encoders (device-own).  VBATREG: (mV-3800)*10/125 into bits[5:0];
 * IBUSLIM: mA/50 (full byte); FREQ: (kHz-250)/50 into bits[4:0].
 */
#define SM5440_VBATREG_REG(mv)	((((mv) - 3800) * 10) / 125)
#define SM5440_VBATREG_MASK	0x3f
#define SM5440_IBUSLIM_REG(ma)	((ma) / 50)
#define SM5440_FREQ_REG(khz)	((((khz) - 250) / 50) & 0x1f)
#define SM5440_FREQ_MASK	0x1f

/*
 * gts9u board values (device-own sm5440_charger.dtsi): switching freq 450 kHz,
 * total path resistance 500 mOhm for the PPS-voltage IR offset, en_vbatreg=0
 * (so the chip VBATREG ceiling sits CV_OFFSET=50 mV above the cell-V target).
 */
#define SM5440_GTS9U_FREQ_KHZ	450
#define SM5440_GTS9U_R_TTL_UOHM	500000
#define SM5440_CV_OFFSET	50
#define SM5440_TA_MIN_VOLTAGE	8200	/* PPS request floor (device-own config) */
#define SM5440_DC_VBUS_OVP_TH	11000	/* PPS ceiling = this - 500 = 10500 mV */
/*
 * Bug-1a VBUS-sanity guard floor: refuse to flip op-mode CHG_ON when the pump
 * input VBUS has collapsed to < 2*Vcell - this (a dead ~5 V rail after a PD
 * contract collapse).  The healthy 2:1 operating point is VBUS ~= 2*Vcell +
 * cable-IR (~+270 mV, measured at the cool re-preset troughs) -- a full ~2000 mV
 * above this floor; a collapse (VBUS ~= 2*Vcell - 3800) sits well below it.
 */
#define SM5440_DC_ENABLE_VBUS_FLOOR_MV	2000

/*
 * Increment-1 guarded manual-engage parameters (NOT the production step table).
 * A conservative 2000 mA input limit -> ~4000 mA into the cell via the 2:1 pump,
 * a clear ~2x over the SM5714 buck's ~2100 mA baseline yet well under step-0's
 * 4500 mA; the cell-V ceiling is step-0's 4250 mV.  Auto-off backstop at 30 s.
 */
#define SM5440_ENGAGE_IBUS_MA	2000
#define SM5440_ENGAGE_VBAT_MV	4250
#define SM5440_ENGAGE_MAX_TICKS	30	/* monitor ticks (1 s each) before auto-off */

/*
 * Increment-2 sm_dc engine config + step-0 test targets (device-own values,
 * sm5440_charger.{c,h}).  IBUSLIM = ci_gl + CI_OFFSET (the chip targets the
 * input-current goal plus headroom); SIOP freq de-rating is unused in our test
 * range (ci_gl >= 2000 > SIOP_LEV2), so set_charging_config uses the single
 * 450 kHz.  TOPOFF/CHG_FLOAT drive the CV done-event only when target == float
 * (step-0's 4250 mV != 4440, so DONE does not fire -- disengage is by WDT/stop).
 */
#define SM5440_CI_OFFSET	300
#define SM5440_TA_MIN_CURRENT	1000
#define SM5440_SIOP_LEV2	1700
#define SM5440_DC_TOPOFF_MA	1000
#define SM5440_DC_CHG_FLOAT_MV	4440
#define SM5440_DC_STEP0_VBAT_MV	4250	/* step-0 (<=62% SoC) cell-V ceiling */
#define SM5440_VBAT_MIN		3300	/* device-own SM5440_VBAT_MIN / dc_min_vbat */

/*
 * Increment-3b-4 auto-engage supervisor gates + the device-own 3-step
 * direct-charge ladder.  The supervisor engages at SoC < the step-0 ceiling
 * (62 %), starts in step 0, and walks the LIVE engine up through steps 1 and 2
 * by re-targeting vbat/ibus at each boundary (sm_dc_set_target_vbat/ibus -- the
 * engine re-ramps inside the live PD contract, no re-negotiation); the buck owns
 * the final dchg_end_soc -> 100 % tail.  The ongoing N->N+1 transition mirrors the
 * device-own min(step_vol, step_input) with SoC start-only (gts9u DT
 * dc_step_chg_type 0xe9 = SOC_INIT_ONLY|INPUT_CURRENT|FLOAT_VOLTAGE|ONLINE|
 * VOLTAGE): advance when (FG cell-V + v_margin >= cond_vol[N]) AND
 * (IBUS <= cond_iin[N]), debounced iin_check_cnt ticks.  All values transcribed
 * from the device's own DT: cond_vol == val_vfloat 4250/4420/4440 mV; val_iout
 * (CELL current) 9000/8200/4000 -> engine IBUS = val_iout/2 for the 2:1 pump =
 * 4500/4100/2000 mA; cond_iin[N] = val_iout[N+1]/2 = 4100/2000 mA; v_margin 40 mV;
 * iin_check_cnt 3; dchg_end_soc default 95.  This validation ships the ladder from
 * a sub-step-0 plug; start-step-by-SoC + a high-SoC engage are a deferred
 * completeness edge (the buck owns a high-SoC plug for now).
 */
#define SM5440_AUTO_ENGAGE_SOC		62	/* engage only when SoC < this (device-own step-0 start) */
#define SM5440_AUTO_DISENGAGE_SOC	95	/* disengage at/above = device-own dchg_end_soc; buck finishes 95->100 */
#define SM5440_AUTO_ENGAGE_CI_GL	4000	/* engage/low-VBAT IBUS cap (mA).  The device-own step-0 is
						 * val_iout[0]/2 = 4500, but mainline tcpm sends a Hard Reset when a
						 * PPS Request cannot PS_RDY within tPSTransition (500 ms): the source
						 * current-limits at the ~10.5 V ceiling, so a request above the
						 * last-good PS_RDY boundary trips the timer (the bespoke PD stack
						 * tolerates the missing PS_RDY; tcpm does not).  At auto-engage
						 * ta.c_max == ci_gl, so _try_to_adjust_cc_up caps ta.c at ta.c_max
						 * and the PEAK request == ci_gl (NOT ci_gl+200 -- that band is
						 * unreachable once c_max binds; sess-234 measured peak 4000, delivered
						 * ~3760, ZERO HRST at loaded 3939-4009 mV).  This 4000 is the LOW-VBAT
						 * band only: the PS_RDY boundary FALLS as the cell climbs, so
						 * dc_monitor's Build-2a sm5440_ci_gl_cap() ratchets ci_gl DOWN per
						 * VBAT band to stay below the falling boundary through step 1. */
#define SM5440_AUTO_RETRY_MAX		3	/* consecutive engage failures before latching off until replug */
/*
 * dc_intent false-transition debounce depth (consecutive buck-worker false polls
 * required before committing dc_intent=false).  dc_intent is VBUS-POK keyed
 * (sm5714-usb-vbus.c: dc_intent = vbus && !charge_cut) and the buck worker is a
 * PURE 3 s poller (SM5714_CHG_POLL_MS, no IRQ wired).  A PD Hard Reset drives VBUS
 * to vSafe0V for ~825 ms (< one poll period), so at most ONE poll samples the
 * transient -> one false notify -> the NEXT poll (VBUS back) re-asserts true.
 * Committing false on that single transient misfires auto_engage_work's !intent
 * disengage (cancels the dc_reengage_work recovery -> the pump strands on a fixed
 * PDO).  Requiring 2 consecutive false polls holds dc_intent true across a
 * single-poll HRST transient yet still disengages a genuine unplug within one extra
 * poll.  Grid-synchronised to poll COUNT (not wall-clock), so immune to the poll
 * body's variable duration (i2c stalls / an AFC 9 V handshake in the corrective
 * poll).  2 is the LARGEST depth that still commits false inside the reengage
 * no-strike window on a real unplug.
 */
#define SM5440_DC_INTENT_FALSE_DEBOUNCE	2

/*
 * Build-2b: re-engage-on-HRST recovery.  A real Hard Reset drops the PPS contract
 * to a FIXED PDO (online 2->1) and stops the engine; the keepalive gives up on the
 * contract loss and nothing re-drives PROG_ONLINE, so the pump sticks on the buck
 * (sess-233 §9).  dc_monitor arms the recovery worker on a mid-charge (non-topoff)
 * engine stop; the worker re-activates PPS and re-engages, bounded by the SAME
 * 3-strike dc_retry_cnt latch.  The poll window MUST exceed the SM5714 shim's
 * worst-case CC-open->reattach recovery (sess-231: ~1.37 s/leg limit cycle,
 * cycle-break ~2.5-4 s) so a still-reattaching source counts as a WAIT, not a
 * strike -- else we latch off a source the shim would have brought back.
 */
#define SM5440_REENGAGE_DELAY_MS	1000	/* initial beat after the HRST teardown (let tcpm start its recovery) */
#define SM5440_REENGAGE_POLL_MS		500	/* PPS re-activation poll cadence */
#define SM5440_REENGAGE_POLL_TRIES	16	/* * 500 ms = 8 s window (> the shim's worst-case reattach) */
#define SM5440_REENGAGE_GAP_MS		2000	/* gap between full recovery attempts (each attempt = one strike) */
#define SM5440_REENGAGE_PPS_MV		10000	/* PPS re-activation target (the 10 V sustain baseline) */
#define SM5440_REENGAGE_PPS_MA		3000	/* PPS re-activation current ceiling */

/*
 * TEST-ONLY (Build-2b validation): the HRST-injection over-request current.  4500
 * mA = the device-own step-0 input (val_iout[0]/2) -- within the source's APDO (so
 * pps_request does not clamp it away) but ABOVE the PS_RDY boundary at any VBAT the
 * run reaches, so a held request at this value provokes a real tcpm-sent Hard Reset
 * (the sess-233 mechanism).  REMOVE this and its sysfs knob before any production
 * or upstream build.
 */
#define SM5440_HRST_INJECT_MA		4500

/*
 * The device-own ladder table (gts9u DT battery,dc_step_chg_*; see the comment
 * above for the encoding + provenance).  Index = step 0..SM5440_DC_STEP_MAX;
 * cond_iin has one fewer entry (the last step has no "next").
 */
#define SM5440_DC_STEP_MAX	2	/* last step index (3 steps: 0,1,2) */
#define SM5440_DC_STEP_V_MARGIN	40	/* device-own dc_step_chg_cond_v_margin (mV) */
#define SM5440_DC_STEP_IIN_CNT	3	/* device-own dc_step_chg_iin_check_cnt (debounce ticks) */
static const u32 sm5440_dc_step_vbat[SM5440_DC_STEP_MAX + 1]     = { 4250, 4420, 4440 }; /* val_vfloat (mV) */
static const u32 sm5440_dc_step_ibus[SM5440_DC_STEP_MAX + 1]     = { 4500, 4100, 2000 }; /* val_iout/2 IBUS (mA) */
static const u32 sm5440_dc_step_cond_vol[SM5440_DC_STEP_MAX + 1] = { 4250, 4420, 4440 }; /* cond_vol (mV) */
static const u32 sm5440_dc_step_cond_iin[SM5440_DC_STEP_MAX]     = { 4100, 2000 };       /* val_iout[N+1]/2 (mA) */

/*
 * Build-2a: VBAT-aware ci_gl (INPUT-current) ceiling.  The source's PS_RDY
 * boundary -- the maximum input current tcpm can Request before the source
 * withholds PS_RDY within tPSTransition (500 ms) and tcpm SENDS a Hard Reset --
 * FALLS as the cell voltage climbs (2*Vcell rises, so the source's headroom under
 * the ~10.5 V PPS ceiling shrinks).  Empirical model (LOADED FG voltage_now, the
 * dc_monitor's cell-V reading): boundary(mV) ~= 4450 - 3*(mV - 3900) mA -- fit to
 * the sess-233 Hard Reset (req 4000 mA @ loaded 4050 mV) and confirmed by sess-234
 * (req 4000 mA held, delivered ~3760 mA, ZERO HRST @ loaded 3939-4009 mV).  A
 * FIXED cap (Build 1's 4000) eventually MEETS the falling boundary -> one miss
 * fires the HRST -> pump drops to buck.  So cap ci_gl per VBAT band, a conservative
 * >=480 mA below the modeled boundary, ratcheting DOWN as the cell climbs
 * (dc_monitor applies it).  VBAT rises monotonically on charge, so the ratchet
 * only ever lowers -> no flap, no hysteresis needed.  This caps steps 0 AND 1 (the
 * pump runs continuously ENGAGE_SOC..DISENGAGE_SOC; step 1's 4100 mA is itself a
 * high-VBAT HRST site).  Sub-ceiling requests DELIVER their request (the source
 * meets them below the boundary), so the current tapers with VBAT -- the same
 * physics the device-own sec_step_charging supervisor imposes.
 */
static u32 sm5440_ci_gl_cap(int fg_mv)
{
	if (fg_mv < 4010)	return 4000;	/* boundary >4120; sess-234 measured-safe (delivered ~3760) */
	if (fg_mv < 4090)	return 3400;	/* boundary 4120->3880; margin >=480 mA */
	if (fg_mv < 4170)	return 3200;	/* deliver toward ceiling (10500-2Vcell)/0.66 ~3270-3510; boundary 3880->3640, margin >=440 mA (measured-safe: 3400 held @4090, 0 HRST) */
	if (fg_mv < 4280)	return 2900;	/* ceiling ~2940-3270; boundary ~3640->3310, margin >=410 mA */
	return 2500;				/* ceiling ~2460-2940; boundary ~3310->2830, margin >=330; CV float (4440) approaching */
}

/*
 * Increment-3a graceful-stop ramp-down (the buck handoff).  sess-206 run-1 showed
 * that restoring the sustained-PPS voltage in ONE step (the engine's ~10500 mV
 * ceiling -> 10000) while the buck simultaneously re-loaded the source Hard-Reset
 * the PPS contract -> the source fell back to AFC 9 V, forcing a re-arm + replug.
 * The device-own supervisor (sec_direct_charger.c:462-481) charges-off the pump
 * BEFORE moving the TA voltage ("to prevent reverse-current into TA") and drops to
 * a fixed 9 V for the buck.  We keep pump-off-first (the caller's sm_dc_stop_charging)
 * and -- our PD requester being PPS-only -- step the PPS target DOWN to 9 V in small
 * increments, letting the source settle UNLOADED (pump off, buck still inhibited)
 * between steps before the buck re-loads it.  The unloaded settle is the fix.
 */
#define SM5440_DC_STOP_TA_MV	9000	/* graceful-stop PPS target (device-own 9 V, buck-OK) */
#define SM5440_DC_STOP_TA_MA	3000	/* PPS current ceiling requested during the ramp-down */
#define SM5440_DC_STOP_STEP_MV	300	/* PPS down-step per settle */
#define SM5440_DC_STOP_SETTLE_MS 150	/* unloaded settle between steps */
/*
 * Increment-3a comp-1 reload-race fix: after the final 9 V commit, the source
 * takes ~1-2 s / several keepalive Requests to actually transition (sess-207 saw
 * ~2.15 s) -- so VERIFY the (still-live) VBUS ADC has settled near 9 V before the
 * caller re-loads the buck, rather than returning immediately onto a source still
 * mid-transition (the sess-207 clean reload was buck-poll-phase luck, not a
 * guaranteed settle).  Bounded poll, then proceed regardless (the buck restore
 * must not be blocked indefinitely on a flaky source).
 */
#define SM5440_DC_STOP_VERIFY_MS	100	/* VBUS re-read interval */
#define SM5440_DC_STOP_VERIFY_TRIES 30	/* * 100 ms = 3 s max wait for the 9 V settle */

/*
 * Increment-3b comp-2: the adaptive ta.v_max probe-up (the dc_probe path).  Keep
 * ci_gl HIGH (the engine perpetually wants more, so it pushes ta.v toward
 * ta.v_max every CC loop) and walk ta.v_max -- the request-voltage ceiling -- UP
 * in small steps, reading the IBUS response, until either the input current
 * reaches the goal (success) or the source's deliverable ceiling is found (flat
 * IBUS below the goal).  Two independent mechanisms, by design:
 *   - the FLAT-IBUS detector decides when to STOP CLIMBING (slow, post-settle);
 *   - the COLLAPSE-GUARD decides when to RETREAT (fast, every tick) -- if the
 *     device VBUS sags toward the 2*Vcell 2:1 floor, drop ta.v_max immediately,
 *     BEFORE the sag deepens into a VBUSUVLO cliff.  This is the safety mechanism
 *     (the flat-detector alone cannot prevent a single bump cliffing the source).
 * ta.v_max is the PPS REQUEST; the device VBUS = request - cable IR-drop, so the
 * guard reads VBUS (device side) while the lever moves the request.
 */
#define SM5440_PROBE_TICK_MS	500	/* probe + collapse-guard cadence */
#define SM5440_PROBE_SETTLE_TICKS 20	/* * 500 ms = 10 s for the engine's ~40 mV/iter CC ramp to walk ta.v up + IBUS to settle after a bump (shorter risks a premature flat-lock mid-climb) */
#define SM5440_PROBE_VMAX_STEP	200	/* ta.v_max (request) step per bump, mV */
#define SM5440_PROBE_VMAX_CEIL	(SM5440_DC_VBUS_OVP_TH - 500)	/* 10500 mV hard request ceiling */
#define SM5440_PROBE_START_MARGIN 600	/* initial ta.v_max = the engine's engage request + this */
#define SM5440_PROBE_FLAT_MA	100	/* delta-IBUS below this == flat (matches CC_ST_IBUS_OFFSET) */
/*
 * Retreat if device VBUS < 2*Vcell + this.  Must sit BELOW the engine's natural
 * engage headroom or the guard fires spuriously at startup: the engine engages at
 * a fixed ta.v = 2*Vcell + (target/2)*r_ttl + 200, so at the ~460 mOhm test cable
 * + the ~2250 mA engage current the device VBUS is ~2*Vcell + 290 mV.  200 leaves
 * a ~90 mV start margin; healthy operation's headroom only GROWS as the probe
 * climbs (the request rises faster than the IR-drop), so the guard then fires only
 * on a genuine source sag (the CC knee), with the engine's VBUSUVLO-RETRY +
 * dc_monitor buck-restore as the deeper backstops.
 */
#define SM5440_PROBE_COLLAPSE_MV 200

/*
 * Mainline-tcpm PPS bridge (Increment-3c, upstream-readiness).  The pump steps
 * the PPS contract through tcpm's source power-supply.  ONLINE-write states are
 * private to tcpm.c (no exported header) so they are hand-replicated here.  The
 * psy name is "tcpm-source-psy-" + dev_name(PDIC) = i2c bus 1 addr 0x33.  The
 * by-name binding is the string-coupled v1; the by-phandle
 * devm_power_supply_get_by_reference() form is the upstream-final shape.
 */
#define SM5440_TCPM_PSY_OFFLINE		0
#define SM5440_TCPM_PSY_FIXED_ONLINE	1
#define SM5440_TCPM_PSY_PROG_ONLINE	2
#define SM5440_TCPM_SOURCE_PSY		"tcpm-source-psy-1-0033"
#define SM5440_PPS_KEEPALIVE_MS		5000	/* re-ping < source tPPSTimeout (~12 s) */
#define SM5440_PPS_KEEPALIVE_SKIP_MS	3000	/* skip the ping if the engine stepped recently */

struct sm5440 {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply *fg;	/* sm5714-fuelgauge, for the engage cross-check */
	struct delayed_work pump_monitor;
	/*
	 * engage_lock serialises BOTH manual bring-up triggers (the Inc-1 pump_test
	 * one-shot engage AND the Inc-2 dc_test engine run) + their monitors.  They
	 * are mutually exclusive: each path checks the other's pump_engaged/dc_running
	 * flag under this one lock before touching the shared pump / FG handle (a
	 * single lock, not two disjoint ones, is what makes that check race-free).
	 */
	struct mutex engage_lock;
	bool pump_engaged;
	int monitor_ticks;
	u8 rev_id;

	/*
	 * Increment-2: the ported sm_dc CC/CV engine drives the closed-loop ramp.
	 * dc is the engine instance (its ops are this driver's sm5440_dc_ops);
	 * ocp_check_work is the device-faithful SW-OCP backstop; dc_monitor is a
	 * LOG-ONLY telemetry logger for the engine run (it does NOT auto-disengage --
	 * disengage is owned by the engine / chip WDT / dc_test=0).
	 */
	struct sm_dc_info *dc;
	struct delayed_work ocp_check_work;
	struct delayed_work dc_monitor;
	bool dc_running;
	int dc_ticks;

	/*
	 * Increment-3b comp-2: adaptive ta.v_max probe-up state (the dc_probe path).
	 * probe_work runs every SM5440_PROBE_TICK_MS while dc_adaptive; the
	 * collapse-guard runs every tick, the climb only while !probe_locked and the
	 * engine is in CC.  All accessed under engage_lock (like dc_running).
	 */
	struct delayed_work probe_work;
	bool dc_adaptive;	/* this run is the adaptive probe (vs fixed-ci_gl dc_test) */
	bool probe_locked;	/* climb settled (success or source ceiling) -- guard still runs */
	u32 probe_vmax;		/* current ta.v_max request setpoint (mV) */
	u32 probe_ci_gl;	/* the high ci_gl goal (mA) for the success test */
	u32 probe_floor_off;	/* ta.v_max never retreats below 2*Vcell + this (the engage point) */
	int probe_prev_ibus;	/* IBUS (mA) at the last bump, for the flat-detector */
	int probe_settle;	/* probe ticks remaining before judging the last bump */

	/*
	 * Increment-3b-3: the auto-engage supervisor (executor half).  The SM5714 buck
	 * worker (the arbiter) publishes dc_intent via its registered notify; the
	 * executor (auto_engage_work) ANDs that with the pump-side gates (a PPS contract
	 * is held, SoC < the step-0 ceiling, no fault latch) and engages/disengages.
	 * Push-triggered by the notify (no independent poll).  auto_engaged marks a run
	 * the supervisor started, so it never disturbs a manual dc_test/pump_test run.
	 * All accessed under engage_lock except dc_intent (WRITE_ONCE in the cb / READ_ONCE
	 * in the executor -- the cb must not take engage_lock; see sm5440_dc_intent_cb).
	 */
	struct delayed_work auto_engage_work;
	bool dc_intent;		/* DEBOUNCED buck-worker notify: charger present + cell temp OK */
	int dc_intent_false_cnt;	/* consecutive false polls; commit false only at SM5440_DC_INTENT_FALSE_DEBOUNCE (HRST VBUS-off debounce).  cb-only -- single serial writer, no lock needed */
	bool auto_engaged;	/* this dc run was started by the supervisor (gates auto-disengage) */
	bool dc_err_latched;	/* engage failed SM5440_AUTO_RETRY_MAX times -- stop until charger replug */
	int dc_retry_cnt;	/* consecutive auto-engage failures (device-own dc_retry_cnt analog) */

	/*
	 * Build-2b: re-engage-on-HRST recovery.  dc_done marks a benign engine
	 * topoff (done_cb) so dc_monitor does NOT mistake it for an HRST;
	 * dc_recover_armed gates the recovery worker; dc_inject_hrst is the
	 * TEST-ONLY over-request arming (see the sysfs knob).  All accessed under
	 * engage_lock except the two READ_ONCE/WRITE_ONCE bools crossed between the
	 * engine done_cb / the sysfs write and dc_monitor.
	 */
	struct delayed_work dc_reengage_work;
	bool dc_done;		/* engine self-DONEd (topoff) -- suppress recovery */
	bool dc_recover_armed;	/* an HRST teardown armed the recovery worker */
	bool dc_inject_hrst;	/* TEST-ONLY: dc_monitor holds an over-request to provoke an HRST */

	/*
	 * Increment-3b-4: the 3-step ladder state.  dc_step is the current step
	 * (0..SM5440_DC_STEP_MAX, the device-own step_chg_status analog); the
	 * advance machine lives in dc_monitor and re-targets the live engine at
	 * each boundary.  Accessed only under engage_lock (like dc_running).
	 */
	int dc_step;		/* current ladder step */
	int dc_step_iin_cnt;	/* consecutive ticks the advance predicate held (debounce -> SM5440_DC_STEP_IIN_CNT) */
	u32 dc_ci_applied;	/* Build-2a: currently-applied VBAT-banded ci_gl cap (mA); dc_monitor ratchets it DOWN */

	/*
	 * Increment-3c: the mainline-tcpm PPS bridge.  tcpm_psy is tcpm's
	 * "tcpm-source-psy-1-0033" power-supply (acquired by name in probe, ref held
	 * for our lifetime, put in remove); the PPS contract is stepped through it.
	 * pps_lock guards the {target,sent} stash that the engine's op-8 and the
	 * keepalive both touch -- held ONLY for the u32 copy, NEVER across the
	 * blocking set_property.  pps_active = PROG_ONLINE written + keepalive armed.
	 */
	struct power_supply *tcpm_psy;
	struct delayed_work pps_keepalive;
	spinlock_t pps_lock;
	bool pps_active;
	u32 pps_target_mv, pps_target_ma;	/* desired (the keepalive re-issues this) */
	u32 pps_sent_mv, pps_sent_ma;		/* last actually sent (skip-unchanged) */
	unsigned long pps_last_step;		/* jiffies of the last step (keepalive backoff) */
};

/*
 * The SM5440 ADC packs a 13-bit sample across two registers as
 * (hi << 5) | (lo >> 3) (device-own sm5440_convert_adc).
 */
static int sm5440_read_adc_raw(struct sm5440 *chip, u8 hi_reg, u8 lo_reg, u32 *raw)
{
	unsigned int hi, lo;
	int ret;

	ret = regmap_read(chip->regmap, hi_reg, &hi);
	if (ret)
		return ret;
	ret = regmap_read(chip->regmap, lo_reg, &lo);
	if (ret)
		return ret;

	*raw = (hi << 5) | (lo >> 3);
	return 0;
}

/* VBUS input voltage, microvolts.  Device-own: mV = 4096 + raw. */
static int sm5440_get_vbus_uv(struct sm5440 *chip, int *uv)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_VBUS1,
				  SM5440_REG_ADC_VBUS2, &raw);
	if (ret)
		return ret;

	*uv = (4096 + raw) * 1000;
	return 0;
}

/* IBUS input current, microamps.  Device-own: mA = raw * 625 / 1000. */
static int sm5440_get_ibus_ua(struct sm5440 *chip, int *ua)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_IBUS1,
				  SM5440_REG_ADC_IBUS2, &raw);
	if (ret)
		return ret;

	*ua = raw * 625;
	return 0;
}

/*
 * Die temperature, tenths of a degree C (power_supply TEMP unit).
 * Device-own: deci-C = 225 + reg * 5 (single register, "C x 10").
 */
static int sm5440_get_dietemp_dc(struct sm5440 *chip, int *dc)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(chip->regmap, SM5440_REG_ADC_DIETEMP, &reg);
	if (ret)
		return ret;

	*dc = 225 + reg * 5;
	return 0;
}

/* VBAT (pump output = cell side) and VOUT, millivolts.  Device-own: 2048 + raw*500/1000. */
static int sm5440_get_vbat_mv(struct sm5440 *chip, int *mv)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_VBAT1,
				  SM5440_REG_ADC_VBAT2, &raw);
	if (ret)
		return ret;

	*mv = 2048 + (raw * 500) / 1000;
	return 0;
}

static int sm5440_get_vout_mv(struct sm5440 *chip, int *mv)
{
	u32 raw;
	int ret;

	ret = sm5440_read_adc_raw(chip, SM5440_REG_ADC_VOUT1,
				  SM5440_REG_ADC_VOUT2, &raw);
	if (ret)
		return ret;

	*mv = 2048 + (raw * 500) / 1000;
	return 0;
}

/*
 * Charge-control register helpers.  update_field is a read-modify-write of a
 * bit field (device-own sm5440_update_reg(reg, val, mask, pos)).
 */
static int sm5440_update_field(struct sm5440 *chip, u8 reg, u8 val, u8 mask, u8 pos)
{
	return regmap_update_bits(chip->regmap, reg, mask << pos,
				  (val & mask) << pos);
}

static int sm5440_set_op_mode(struct sm5440 *chip, u8 op_mode)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL5, op_mode,
				   SM5440_OPMODE_MASK, SM5440_OPMODE_SHIFT);
}

static int sm5440_get_op_mode(struct sm5440 *chip)
{
	unsigned int reg;
	int ret;

	ret = regmap_read(chip->regmap, SM5440_REG_CNTL5, &reg);
	if (ret)
		return ret;
	return (reg >> SM5440_OPMODE_SHIFT) & SM5440_OPMODE_MASK;
}

static int sm5440_set_vbatreg(struct sm5440 *chip, u32 mv)
{
	u8 reg = (mv < 3800) ? 0 : SM5440_VBATREG_REG(mv);

	return sm5440_update_field(chip, SM5440_REG_VBATCNTL, reg,
				   SM5440_VBATREG_MASK, 0);
}

static int sm5440_set_ibuslim(struct sm5440 *chip, u32 ma)
{
	return regmap_write(chip->regmap, SM5440_REG_IBUSCNTL,
			    SM5440_IBUSLIM_REG(ma));
}

static int sm5440_set_freq(struct sm5440 *chip, u32 khz)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL7,
				   SM5440_FREQ_REG(khz), SM5440_FREQ_MASK, 0);
}

static int sm5440_set_wdt_timer(struct sm5440 *chip, u8 tmr)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL1, tmr, 0x7, 4);
}

static int sm5440_enable_wdt(struct sm5440 *chip, bool enable)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL1, enable, 0x1, 7);
}

static int sm5440_enable_chg_timer(struct sm5440 *chip, bool enable)
{
	return sm5440_update_field(chip, SM5440_REG_CNTL3, enable, 0x1, 2);
}

/* SW reset: write CNTL1 bit0, then poll it clear (device-own sm5440_sw_reset). */
static int sm5440_sw_reset(struct sm5440 *chip)
{
	unsigned int reg;
	int i, ret;

	ret = regmap_write(chip->regmap, SM5440_REG_CNTL1, 0x1);
	if (ret)
		return ret;

	for (i = 0; i < 0xff; i++) {
		usleep_range(1000, 2000);
		if (!regmap_read(chip->regmap, SM5440_REG_CNTL1, &reg) &&
		    !(reg & 0x1))
			return 0;
	}

	dev_err(chip->dev, "sw reset bit did not clear\n");
	return -EBUSY;
}

/*
 * One-time chip init before op-mode -> CHG_ON (device-own sm5440_init_reg_param),
 * transcribed verbatim in the same order.  Best-effort like the vendor (the
 * individual writes are not checked); the op-mode flip later is the gate.
 */
static void sm5440_init_reg_param(struct sm5440 *chip)
{
	sm5440_set_wdt_timer(chip, SM5440_WDT_TIMER_30S);
	sm5440_set_freq(chip, SM5440_GTS9U_FREQ_KHZ);
	regmap_write(chip->regmap, SM5440_REG_CNTL2, SM5440_CNTL2_INIT);
	regmap_write(chip->regmap, SM5440_REG_CNTL4, SM5440_CNTL4_INIT);
	regmap_write(chip->regmap, SM5440_REG_CNTL6, SM5440_CNTL6_INIT);
	sm5440_update_field(chip, SM5440_REG_VBUSCNTL, SM5440_VBUSCNTL_OVP_11V, 0x7, 0);
	regmap_write(chip->regmap, SM5440_REG_VOUTCNTL, SM5440_VOUTCNTL_INIT);
	regmap_write(chip->regmap, SM5440_REG_PRTNCNTL, SM5440_PRTNCNTL_INIT);
	sm5440_update_field(chip, SM5440_REG_ADCCNTL1, 1, 0x1, 3);  /* 32-sample avg */
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL2, SM5440_ADCCNTL2_CHANNELS);
	regmap_write(chip->regmap, SM5440_REG_THEMCNTL1, SM5440_THEMCNTL1_INIT);
	regmap_write(chip->regmap, SM5440_REG_CNTL3, SM5440_CNTL3_INIT);
}

/*
 * Poll the STATUS1-4 level mirrors for the forced-cutoff faults.  Returns the
 * raw STATUS bytes via st[] and a bool "a hard fault is asserted".  Polled (no
 * irq-gpio): read STATUS, never the clear-on-read INT registers.
 */
static bool sm5440_fault_present(struct sm5440 *chip, u8 st[4])
{
	int ret;

	ret = regmap_bulk_read(chip->regmap, SM5440_REG_STATUS1, st, 4);
	if (ret) {
		dev_warn(chip->dev, "STATUS read failed (%d)\n", ret);
		return false;
	}
	return (st[0] & SM5440_FAULT_S1) || (st[2] & SM5440_FAULT_S3);
}

/*
 * Lazy-cached fuelgauge handle.  The fuelgauge may probe AFTER this driver, so
 * acquire on first use (under engage_lock) rather than at probe; held for the
 * driver's lifetime and put once in remove().  A persistent handle (not the old
 * per-engage get/put) is required because the auto-engage supervisor reads SoC
 * while IDLE -- between runs there would otherwise be no handle to read.
 */
static struct power_supply *sm5440_fg(struct sm5440 *chip)
{
	if (!chip->fg)
		chip->fg = power_supply_get_by_name("sm5714-fuelgauge");
	return chip->fg;
}

/* Cell SoC percent via the fuelgauge, or -1 if unavailable.  Caller holds engage_lock. */
static int sm5440_read_soc(struct sm5440 *chip)
{
	union power_supply_propval v = { };

	if (!sm5440_fg(chip))
		return -1;
	if (power_supply_get_property(chip->fg, POWER_SUPPLY_PROP_CAPACITY, &v))
		return -1;
	return v.intval;
}

/*
 * Chip-side pump engage (device-faithful order: sw_reset -> init -> ADC on ->
 * IBUSLIM/VBATREG -> op-mode CHG_ON -> WDT).  The PPS input must already be
 * commanded to ~2x Vcell and the buck disarmed by the caller.  sw_reset wipes
 * the telemetry ADC config, so it is re-enabled here and restored on disengage.
 */
static int sm5440_pump_engage_chip(struct sm5440 *chip, u32 ibus_ma, u32 vbat_mv)
{
	int ret;

	ret = sm5440_sw_reset(chip);
	if (ret)
		return ret;

	sm5440_init_reg_param(chip);
	/* re-enable continuous telemetry ADC (sw_reset cleared ADCCNTL1). */
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);

	sm5440_set_ibuslim(chip, ibus_ma);
	/* en_vbatreg=0 on gts9u: the chip CV ceiling sits CV_OFFSET above target. */
	sm5440_set_vbatreg(chip, vbat_mv + SM5440_CV_OFFSET);
	sm5440_set_wdt_timer(chip, SM5440_WDT_TIMER_30S);
	sm5440_enable_chg_timer(chip, false);

	ret = sm5440_set_op_mode(chip, SM5440_OPMODE_CHG_ON);  /* the pump-on write */
	if (ret)
		return ret;
	sm5440_enable_wdt(chip, true);
	return 0;
}

/* Chip-side disengage: op-mode CHG_OFF, WDT off, restore telemetry ADC config. */
static void sm5440_pump_disengage_chip(struct sm5440 *chip)
{
	sm5440_set_op_mode(chip, SM5440_OPMODE_CHG_OFF);
	sm5440_enable_wdt(chip, false);
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL2, SM5440_ADCCNTL2_CHANNELS);
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);
}

/* ===================================================================== *
 * Increment-3c: the mainline-tcpm PPS bridge.
 *
 * The pump steps the PPS contract by writing tcpm's "tcpm-source-psy" through
 * the generic power_supply API -- no EXPORT_SYMBOL cross-driver coupling (the
 * upstream-shaped form validated in isolation at sess-219: a userspace echo in
 * Step 0 + a standalone in-kernel consumer in Step 1).  These helpers replace
 * the bespoke sm5714_pd_request_voltage() / sm5714_pd_contract_active() hooks.
 * ===================================================================== */

/*
 * True iff a PPS-capable PD contract is up.  usb_type == PD_PPS is derived from
 * the SOURCE's advertised caps (an APDO present), NOT from our contract being on
 * the PPS APDO right now -- so it reads true with a PPS-capable adapter even at
 * the 9 V FIXED pre-activation state.  That is exactly the non-circular gate the
 * supervisor needs: it must pass BEFORE the engine activates PPS.  A non-PPS PD
 * charger reads plain PD here, so the pump cleanly declines to engage on a
 * source it could not step.
 */
static bool sm5440_pps_contract_active(struct sm5440 *chip)
{
	union power_supply_propval pv;

	if (!chip->tcpm_psy)
		return false;
	if (power_supply_get_property(chip->tcpm_psy,
				      POWER_SUPPLY_PROP_USB_TYPE, &pv))
		return false;
	if (pv.intval != POWER_SUPPLY_USB_TYPE_PD_PPS)
		return false;
	if (power_supply_get_property(chip->tcpm_psy,
				      POWER_SUPPLY_PROP_ONLINE, &pv))
		return false;
	return pv.intval != SM5440_TCPM_PSY_OFFLINE;
}

/*
 * Step the PPS contract to {mv, ma}; activate PPS lazily on first use.
 *
 * RETURNS 0 for every contract-state outcome -- transient -EAGAIN (AMS in
 * flight), -EINVAL (momentary out-of-window) and -ETIMEDOUT (a slow PS_RDY on a
 * still-live contract) from a step are ABSORBED.  The engine treats any < 0 from
 * op-8 as a FATAL SM_DC_ERR_SEND_PD_MSG and stops the pump, but a transient AMS
 * hiccup must NOT do that: genuine contract loss is detected out-of-band by the
 * supervisor's contract-active gate + the dc_monitor, exactly as the bespoke
 * async hook behaved (it returned < 0 only on genuine loss).  Only -ENODEV (no
 * psy handle -- a structural condition) is surfaced.
 *
 * tcpm exposes V and I as two separate set-properties => two separate PPS AMS
 * (the bespoke packed both into one RDO).  The CC loop steps voltage and holds
 * current constant, so CURRENT_NOW is emitted only when ma changes (and at
 * activation); a steady CC step is then one AMS, matching the bespoke cadence
 * and halving the Hard-Reset collision surface.  force re-sends VOLTAGE_NOW even
 * when unchanged (the keepalive path -- a redundant Request is how the source's
 * tPPSTimeout is refreshed; tcpm has neither an internal keepalive nor an
 * unchanged-value short-circuit).
 *
 * Process / workqueue context only (set_property blocks up to ~10 s).  Never
 * holds pps_lock across a set_property (tcpm serialises concurrent steppers
 * internally on its own port lock).
 */
static int sm5440_pps_request(struct sm5440 *chip, u32 mv, u32 ma, bool force)
{
	union power_supply_propval pv;
	unsigned long flags;
	bool need_c, need_v;
	int ret;

	if (!chip->tcpm_psy)
		return -ENODEV;

	/*
	 * Activate PPS once.  -EAGAIN (state != SNK_READY mid-AMS): leave inactive
	 * and return 0 so the engine is not faulted; the next step (or dc_start's
	 * VBUS-verify abort + the supervisor's retry) re-attempts.  On success arm
	 * the keepalive and reset the sent-cache so the first step emits both V + I.
	 */
	if (!READ_ONCE(chip->pps_active)) {
		pv.intval = SM5440_TCPM_PSY_PROG_ONLINE;
		ret = power_supply_set_property(chip->tcpm_psy,
						POWER_SUPPLY_PROP_ONLINE, &pv);
		if (ret) {
			dev_dbg(chip->dev, "pps: activate deferred (%d)\n", ret);
			return 0;
		}
		spin_lock_irqsave(&chip->pps_lock, flags);
		chip->pps_active = true;
		chip->pps_sent_mv = 0;
		chip->pps_sent_ma = 0;
		spin_unlock_irqrestore(&chip->pps_lock, flags);
		schedule_delayed_work(&chip->pps_keepalive,
				      msecs_to_jiffies(SM5440_PPS_KEEPALIVE_MS));
		dev_info(chip->dev, "pps: activated (PROG_ONLINE)\n");
	}

	/*
	 * Clamp to the live APDO window -- the SOURCE max exposed by tcpm (the sink
	 * APDO does not throttle a request).  Guards against a current-limited
	 * adapter, which tcpm would otherwise -EINVAL.
	 */
	if (!power_supply_get_property(chip->tcpm_psy,
				       POWER_SUPPLY_PROP_VOLTAGE_MAX, &pv) &&
	    pv.intval > 0)
		mv = min(mv, (u32)pv.intval / 1000);
	if (!power_supply_get_property(chip->tcpm_psy,
				       POWER_SUPPLY_PROP_CURRENT_MAX, &pv) &&
	    pv.intval > 0)
		ma = min(ma, (u32)pv.intval / 1000);

	/* Align to the PD programmable grids or tcpm silently floors the request. */
	mv -= mv % RDO_PROG_VOLT_MV_STEP;
	ma -= ma % RDO_PROG_CURR_MA_STEP;

	/* Decide what to emit + stash the target (the keepalive re-issues it). */
	spin_lock_irqsave(&chip->pps_lock, flags);
	chip->pps_target_mv = mv;
	chip->pps_target_ma = ma;
	need_c = (ma != chip->pps_sent_ma);
	need_v = force || (mv != chip->pps_sent_mv);
	spin_unlock_irqrestore(&chip->pps_lock, flags);

	/*
	 * Current first (raise the ceiling before a voltage that may need it), then
	 * voltage.  Every step errno is logged + absorbed -> 0 (see the kerneldoc).
	 */
	if (need_c) {
		pv.intval = ma * 1000;
		ret = power_supply_set_property(chip->tcpm_psy,
						POWER_SUPPLY_PROP_CURRENT_NOW, &pv);
		if (ret) {
			dev_dbg(chip->dev, "pps: set I=%u mA (%d, absorbed)\n", ma, ret);
		} else {
			WRITE_ONCE(chip->pps_sent_ma, ma);
			/*
			 * Bump pps_last_step ONLY on a real wire send -- the source's
			 * tPPSTimeout is refreshed by the Request that just landed.  It
			 * must NOT be bumped on a C2-suppressed no-op: the engine's
			 * support_pd_remain re-calls op-8 every CC/CV loop (<3 s) with an
			 * unchanged target, which C2 swallows; if those bumped the step
			 * stamp they would keep it younger than KEEPALIVE_SKIP_MS forever,
			 * starving the keepalive of its only job -> no Request reaches the
			 * source -> tPPSTimeout -> Hard Reset.  Keyed to wire activity, the
			 * stamp goes stale during a suppressed hold and the keepalive fires.
			 */
			WRITE_ONCE(chip->pps_last_step, jiffies);
		}
	}
	if (need_v) {
		pv.intval = mv * 1000;
		ret = power_supply_set_property(chip->tcpm_psy,
						POWER_SUPPLY_PROP_VOLTAGE_NOW, &pv);
		if (ret) {
			dev_dbg(chip->dev, "pps: set V=%u mV (%d, absorbed)\n", mv, ret);
		} else {
			WRITE_ONCE(chip->pps_sent_mv, mv);
			WRITE_ONCE(chip->pps_last_step, jiffies);	/* real send -> refresh the skip window */
		}
	}

	return 0;
}

/*
 * PPS keepalive: periodically re-Request the held voltage so the source does not
 * drop the contract on tPPSTimeout (tcpm has no internal keepalive; sess-219
 * Step 1 proved a 5 s re-ping holds it 80 s+ with zero Hard-Reset).  Skips the
 * ping when the engine stepped within the last few seconds (an active CC/CV loop
 * already refreshes the timer) and self-terminates when the contract is gone
 * (charger unplug).  Self-reschedules while pps_active.  Never holds pps_lock
 * across the blocking set_property.
 */
static void sm5440_pps_keepalive_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, pps_keepalive.work);
	unsigned long flags;
	u32 mv, ma;

	if (!READ_ONCE(chip->pps_active))
		return;

	if (!sm5440_pps_contract_active(chip)) {
		dev_info(chip->dev, "pps keepalive: contract gone -- stopping\n");
		WRITE_ONCE(chip->pps_active, false);
		return;			/* do not reschedule */
	}

	/* The engine pinged recently: it owns the cadence this window; just rearm. */
	if (time_before(jiffies, READ_ONCE(chip->pps_last_step) +
				 msecs_to_jiffies(SM5440_PPS_KEEPALIVE_SKIP_MS)))
		goto rearm;

	spin_lock_irqsave(&chip->pps_lock, flags);
	mv = chip->pps_target_mv;
	ma = chip->pps_target_ma;
	spin_unlock_irqrestore(&chip->pps_lock, flags);

	sm5440_pps_request(chip, mv, ma, true);		/* force the re-Request */
	dev_dbg(chip->dev, "pps keepalive: re-Request %u mV\n", mv);

rearm:
	if (READ_ONCE(chip->pps_active))
		schedule_delayed_work(&chip->pps_keepalive,
				      msecs_to_jiffies(SM5440_PPS_KEEPALIVE_MS));
}

/*
 * Increment-1 guarded manual pump-engage orchestration.  This is NOT the
 * production CC/CV loop (that is the ported sm_dc engine) -- it is a one-shot,
 * conservative, instrumented engage to prove on hardware that the pump switches
 * (2:1, VBUS ~= 2*VOUT), draws input current, and charges the cell faster than
 * the SM5714 buck, before the ~850 LOC engine is ported.  It coordinates all
 * three chips: disarm the SM5714 buck, step the SM5714 PPS contract down to
 * ~2*Vcell, then flip the SM5440 op-mode to CHG_ON; a 1 s monitor logs the
 * conversion + currents + faults and auto-disengages on any fault or after a
 * short backstop window.
 */

/*
 * Tear down (device-own order: pump OFF first to block reverse current into the
 * TA, then restore the buck + PPS).  Caller holds engage_lock; the caller must
 * NOT be inside a monitor sync-cancel while holding it.
 */
static void sm5440_pump_disengage_locked(struct sm5440 *chip)
{
	sm5440_pump_disengage_chip(chip);		/* op-mode -> CHG_OFF first */
	chip->pump_engaged = false;
	sm5440_pps_request(chip, 10000, 3000, false);	/* restore the 10 V sustain baseline */
	sm5714_charger_inhibit_buck(false);		/* re-allow the buck to charge */
	sm5714_usb_vbus_inhibit_afc_pump(false);	/* pump no longer owns the contract */
	/* chip->fg is persistent (put once in remove); not released here */
	dev_info(chip->dev, "pump DISENGAGED (buck re-allowed, PPS restored to 10 V)\n");
}

static void sm5440_pump_monitor_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, pump_monitor.work);
	int vbus_uv = 0, ibus_ua = 0, dietemp = 0, vout_mv = 0, vbat_mv = 0;
	int fg_uv = 0, fg_ua = 0, opmode;
	union power_supply_propval pv;
	bool fault;
	u8 st[4] = { };

	mutex_lock(&chip->engage_lock);
	if (!chip->pump_engaged) {
		mutex_unlock(&chip->engage_lock);
		return;
	}

	sm5440_get_vbus_uv(chip, &vbus_uv);
	sm5440_get_ibus_ua(chip, &ibus_ua);
	sm5440_get_dietemp_dc(chip, &dietemp);
	sm5440_get_vout_mv(chip, &vout_mv);
	sm5440_get_vbat_mv(chip, &vbat_mv);
	opmode = sm5440_get_op_mode(chip);
	fault = sm5440_fault_present(chip, st);

	/*
	 * FG cross-check: the en_vbatreg=0 vnow source decision for Increment 2 is
	 * "does the SM5440 VBAT ADC agree with the fuel-gauge?" -- log both so the
	 * data, not an inference, picks the CC/CV regulation source.
	 */
	if (chip->fg) {
		if (!power_supply_get_property(chip->fg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pv))
			fg_uv = pv.intval;
		if (!power_supply_get_property(chip->fg,
				POWER_SUPPLY_PROP_CURRENT_NOW, &pv))
			fg_ua = pv.intval;
	}

	dev_info(chip->dev,
		 "pump[%2d]: opmode=%d VBUS=%dmV 2xVOUT=%dmV IBUS=%dmA die=%d.%dC | cell SM5440-VBAT=%dmV FG-V=%dmV FG-I=%dmA | STATUS=%02x:%02x:%02x:%02x%s\n",
		 chip->monitor_ticks, opmode, vbus_uv / 1000, vout_mv * 2,
		 ibus_ua / 1000, dietemp / 10, dietemp % 10, vbat_mv,
		 fg_uv / 1000, fg_ua / 1000, st[0], st[1], st[2], st[3],
		 fault ? "  *** FAULT ***" : "");

	if (fault || opmode != SM5440_OPMODE_CHG_ON ||
	    ++chip->monitor_ticks >= SM5440_ENGAGE_MAX_TICKS) {
		dev_warn(chip->dev, "pump auto-disengage: %s\n",
			 fault ? "hard fault" :
			 opmode != SM5440_OPMODE_CHG_ON ? "op-mode dropped (WDT/STUP?)" :
			 "max window reached");
		sm5440_pump_disengage_locked(chip);
		mutex_unlock(&chip->engage_lock);
		return;
	}

	schedule_delayed_work(&chip->pump_monitor, msecs_to_jiffies(1000));
	mutex_unlock(&chip->engage_lock);
}

/* Engage sequence.  Caller holds engage_lock and has checked !pump_engaged. */
static int sm5440_pump_engage(struct sm5440 *chip)
{
	int vbat_mv = 0, target_mv, off_mv, ret;

	if (!sm5440_pps_contract_active(chip)) {
		dev_warn(chip->dev,
			 "engage: no PPS contract active -- arm pd_request + plug first\n");
		return -ENOTCONN;
	}

	/* device-own handoff: buck OFF before pump ON. */
	ret = sm5714_charger_inhibit_buck(true);
	if (ret) {
		dev_warn(chip->dev, "engage: buck inhibit failed (%d)\n", ret);
		return ret;
	}
	sm5714_usb_vbus_inhibit_afc_pump(true);		/* pump owns the contract: stand AFC down */

	/*
	 * Command the PPS input to ~2*Vcell + IR offset (device-own
	 * _calc_pps_v_init_offset = Ibus*r_ttl/1e6 + 200 mV), clamped to
	 * [ta_min_voltage, dc_vbus_ovp_th - 500].  Engaging at the 10 V keepalive
	 * would be ~2.5:1 and trip the pump's startup precondition.  The SM5440 VBAT
	 * ADC reads live because the contract already supplies VBUS.
	 */
	ret = sm5440_get_vbat_mv(chip, &vbat_mv);
	if (ret || vbat_mv < 2500 || vbat_mv > 4500) {
		dev_warn(chip->dev, "engage: implausible VBAT %d mV (ret=%d) -- aborting\n",
			 vbat_mv, ret);
		ret = ret ? ret : -EIO;
		goto err_uninhibit;
	}
	off_mv = (SM5440_ENGAGE_IBUS_MA * (SM5440_GTS9U_R_TTL_UOHM / 1000)) / 1000 + 200;
	target_mv = 2 * vbat_mv + off_mv;
	target_mv = min(target_mv, SM5440_DC_VBUS_OVP_TH - 500);
	target_mv = max(target_mv, SM5440_TA_MIN_VOLTAGE);

	ret = sm5440_pps_request(chip, target_mv, 5000, false);
	if (ret) {
		dev_warn(chip->dev, "engage: PPS request %d mV failed (%d)\n",
			 target_mv, ret);
		goto err_uninhibit;
	}
	dev_info(chip->dev,
		 "engage: VBAT=%dmV -> PPS target=%dmV (offset %dmV); settling 300 ms\n",
		 vbat_mv, target_mv, off_mv);
	msleep(300);

	/*
	 * Confirm the PPS actually transitioned before flipping op-mode.  The
	 * keepalive re-Request passes through the SinkTxOk collision gate (unverified
	 * on this hardware); if it deferred, VBUS may still be ~10 V, which would
	 * engage at ~2.5:1 into the cell and trip STUP_FAIL.  Verifying VBUS here
	 * turns that into a clean "PPS did not transition" abort instead.
	 */
	{
		int vbus_uv = 0, vbus_mv, diff;

		sm5440_get_vbus_uv(chip, &vbus_uv);
		vbus_mv = vbus_uv / 1000;
		diff = vbus_mv - target_mv;
		if (diff < 0)
			diff = -diff;
		if (diff > target_mv / 10) {
			dev_warn(chip->dev,
				 "engage: VBUS=%dmV not within 10%% of target %dmV (PPS did not transition) -- aborting\n",
				 vbus_mv, target_mv);
			ret = -EIO;
			goto err_uninhibit;
		}
		dev_info(chip->dev,
			 "engage: VBUS settled to %dmV (target %dmV) -- flipping op-mode\n",
			 vbus_mv, target_mv);
	}

	ret = sm5440_pump_engage_chip(chip, SM5440_ENGAGE_IBUS_MA, SM5440_ENGAGE_VBAT_MV);
	if (ret) {
		dev_err(chip->dev, "engage: chip engage failed (%d)\n", ret);
		goto err_uninhibit;
	}

	sm5440_fg(chip);			/* lazy-cache the persistent FG handle */
	chip->pump_engaged = true;
	chip->monitor_ticks = 0;
	schedule_delayed_work(&chip->pump_monitor, msecs_to_jiffies(1000));
	dev_info(chip->dev,
		 "pump ENGAGED: IBUSLIM=%dmA VBATREG=%dmV -- monitoring (max %d s)\n",
		 SM5440_ENGAGE_IBUS_MA, SM5440_ENGAGE_VBAT_MV, SM5440_ENGAGE_MAX_TICKS);
	return 0;

err_uninhibit:
	sm5714_charger_inhibit_buck(false);
	sm5714_usb_vbus_inhibit_afc_pump(false);	/* engage aborted: release the AFC stand-down */
	return ret;
}

/*
 * Write-only sysfs bring-up trigger: "1" engages the pump (guarded), "0"
 * disengages.  Per-execution gated; never auto-engages.
 */
static ssize_t pump_test_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sm5440 *chip = dev_get_drvdata(dev);
	bool engage;
	int ret = 0;

	if (kstrtobool(buf, &engage))
		return -EINVAL;

	if (engage) {
		mutex_lock(&chip->engage_lock);
		/* mutually exclusive with the Inc-2 dc_test engine run (shared pump/FG). */
		ret = (chip->pump_engaged || chip->dc_running) ? -EBUSY :
		      sm5440_pump_engage(chip);
		mutex_unlock(&chip->engage_lock);
		return ret ? ret : count;
	}

	/*
	 * Disengage: sync-cancel the monitor WITHOUT engage_lock (the monitor takes
	 * it -- holding it here would deadlock the sync-cancel), then tear down.
	 */
	cancel_delayed_work_sync(&chip->pump_monitor);
	mutex_lock(&chip->engage_lock);
	if (chip->pump_engaged)
		sm5440_pump_disengage_locked(chip);
	mutex_unlock(&chip->engage_lock);
	return count;
}
static DEVICE_ATTR_WO(pump_test);

/* ===================================================================== *
 * Increment-2: the sm_dc CC/CV engine integration.
 *
 * The 9 sm_dc_ops are thin wrappers over the Inc-1 register helpers above.
 * The engine (sm5440_direct_charger.c) is hardware-agnostic and drives the
 * SM5440 only through this vtable + the single send_power_source_msg seam,
 * which routes the engine's stepped PPS target to the tcpm PPS contract via
 * sm5440_pps_request().
 * ===================================================================== */

/* op 1: ADC read.  Engine units are mV (voltages) / mA (currents). */
static int sm5440_dc_get_adc_value(struct i2c_client *i2c, u8 adc_ch)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	int val = 0;

	switch (adc_ch) {
	case SM_DC_ADC_VBAT:
		sm5440_get_vbat_mv(chip, &val);
		return val;			/* mV */
	case SM_DC_ADC_VBUS:
		sm5440_get_vbus_uv(chip, &val);
		return val / 1000;		/* uV -> mV */
	case SM_DC_ADC_IBUS:
		sm5440_get_ibus_ua(chip, &val);
		return val / 1000;		/* uA -> mA */
	case SM_DC_ADC_VOUT:
		sm5440_get_vout_mv(chip, &val);
		return val;			/* mV */
	case SM_DC_ADC_DIETEMP:
		sm5440_get_dietemp_dc(chip, &val);
		return val;			/* deci-C (logged only) */
	case SM_DC_ADC_THEM:
	case SM_DC_ADC_IBAT:		/* no IBAT ADC on this chip (device-own: 0) */
	default:
		return 0;
	}
}

/*
 * op 2: ADC mode.  The device-own ONESHOT needs a 200 ms re-poll for freshness
 * that we drop; alias ONESHOT->CONTINUOUS (every read fresh, zero cost) and keep
 * the ADC continuously enabled even on OFF so the telemetry psy + monitor keep
 * reading.  All modes -> ADCCNTL1 = enable|continuous|32-avg.
 */
static int sm5440_dc_set_adc_mode(struct i2c_client *i2c, u8 mode)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	return regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);
}

/* op 3: is the pump switching? */
static int sm5440_dc_get_charging_enable(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	return sm5440_get_op_mode(chip) == SM5440_OPMODE_CHG_ON ? 1 : 0;
}

/*
 * op 4: enable/disable the pump (op-mode + WDT).  ENHIZ stays cleared by init
 * (CNTL6=0x09).  The device-own set_charging_enable also high-Zs the VBUS input
 * (ENHIZ=1) on a disable-with-VBUS-present; we deliberately omit that here --
 * Inc-1's pump_test disengage proved op-mode CHG_OFF alone is a clean teardown
 * on this hardware, and the 11 V VBUS-OVP plus the buck-restore handoff protect
 * the path, so we do not add an untested input-isolation write on the disable.
 */
static int sm5440_dc_set_charging_enable(struct i2c_client *i2c, bool enable)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	/*
	 * Bug-1a VBUS-sanity guard (defense-in-depth): the 2:1 pump requires
	 * VBUS ~= 2*Vcell.  If a mid-charge ci_gl re-preset collapsed the PD
	 * contract to a fixed ~5 V PDO, VBUS sits FAR below 2*Vcell; flipping
	 * op-mode CHG_ON into that dead rail is what turns a recoverable contract
	 * loss into a VBUSOVP -- the blind re-enable at
	 * sm5440_direct_charger.c:pd_preset_dc_work (which ignores this return).
	 * Refuse the enable into a collapsed rail; mirrors the probe collapse-
	 * guard (VBUS vs 2*Vcell) and dc_start's settle-verify.  Leaving op-mode
	 * OFF makes the engine's next PRE_CC error poll (check_error_state ->
	 * get_dc_error_status sees CHG_OFF) fault to a clean recoverable ERR ->
	 * dc_monitor -> recovery, never an OVP-into-a-corpse.  A refuse only skips
	 * the CHG_ON flip, so it can only PREVENT an off->on (never turns a running
	 * pump off) and it fails OPEN on a bad sensor read (vbat_mv > 0 gate).
	 */
	if (enable) {
		int vbus_uv = 0, vbat_mv = 0;

		if (!sm5440_get_vbus_uv(chip, &vbus_uv) &&
		    !sm5440_get_vbat_mv(chip, &vbat_mv) && vbat_mv > 0 &&
		    vbus_uv / 1000 < 2 * vbat_mv - SM5440_DC_ENABLE_VBUS_FLOOR_MV) {
			dev_err(chip->dev,
				"enable REFUSED: VBUS=%dmV < 2*Vcell(%d)-%d mV (rail collapsed) -- fault to recovery, not OVP\n",
				vbus_uv / 1000, vbat_mv, SM5440_DC_ENABLE_VBUS_FLOOR_MV);
			return -EIO;
		}
	}

	sm5440_set_op_mode(chip, enable ? SM5440_OPMODE_CHG_ON : SM5440_OPMODE_CHG_OFF);
	sm5440_enable_wdt(chip, enable);
	return 0;
}

/*
 * op 5: program the regulation targets.  en_vbatreg=0 on gts9u -> the chip
 * VBATREG ceiling sits CV_OFFSET above the cell-V goal; IBUSLIM = ci_gl +
 * CI_OFFSET (the input-current goal plus headroom).  freq is the single 450 kHz
 * (SIOP de-rating unused in our ci_gl >= 2000 range).
 */
static int sm5440_dc_set_charging_config(struct i2c_client *i2c, u32 cv_gl, u32 ci_gl, u32 cc_gl)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	u32 vbatreg, ibuslim;

	vbatreg = cv_gl + SM5440_CV_OFFSET;
	if (ci_gl <= SM5440_TA_MIN_CURRENT)
		ibuslim = ci_gl + SM5440_CI_OFFSET * 2;
	else
		ibuslim = ci_gl + SM5440_CI_OFFSET;

	sm5440_set_ibuslim(chip, ibuslim);
	sm5440_set_vbatreg(chip, vbatreg);
	sm5440_set_freq(chip, SM5440_GTS9U_FREQ_KHZ);

	dev_info(chip->dev, "dc config: vbatreg=%umV ibuslim=%umA freq=%dkHz (cv_gl=%u ci_gl=%u)\n",
		 vbatreg, ibuslim, SM5440_GTS9U_FREQ_KHZ, cv_gl, ci_gl);
	return 0;
}

/*
 * op 6: error status (polled adaptation -- this board has no SM5440 irq-gpio,
 * so the 5 forced-cutoff faults the device-own checks in its IRQ are folded in
 * here, read from the STATUS level mirrors, not the clear-on-read INT regs).
 */
static u32 sm5440_dc_get_dc_error_status(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	u32 err = SM_DC_ERR_NONE;
	int op_mode, vbat = 0;
	u8 st[4] = { };

	if (regmap_bulk_read(chip->regmap, SM5440_REG_STATUS1, st, 4)) {
		dev_warn(chip->dev, "dc err: STATUS read failed -> retry\n");
		return SM_DC_ERR_RETRY;
	}
	op_mode = sm5440_get_op_mode(chip);

	if (op_mode == SM5440_OPMODE_CHG_OFF) {
		/* the chip force-cut charging off: identify which fault latched. */
		if (st[0] & SM5440_STATUS1_VOUTOVP)	err |= SM_DC_ERR_VOUTOVP;
		if (st[2] & SM5440_STATUS3_VBUSOVP)	err |= SM_DC_ERR_VBUSOVP;
		if (st[2] & SM5440_STATUS3_STUP_FAIL)	err |= SM_DC_ERR_STUP_FAIL;
		if (st[2] & SM5440_STATUS3_REVBLK)	err |= SM_DC_ERR_REVBLK;
		if (st[2] & SM5440_STATUS3_CFLY_SHORT)	err |= SM_DC_ERR_CFLY_SHORT;
		if (st[2] & SM5440_STATUS3_VBUSUVLO)	err |= SM_DC_ERR_VBUSUVLO;
		if (err == SM_DC_ERR_NONE)
			err = SM_DC_ERR_UNKNOWN;
		dev_err(chip->dev, "dc err: op-mode CHG_OFF, STATUS %02x:%02x:%02x:%02x -> 0x%x\n",
			st[0], st[1], st[2], st[3], err);
		return err;
	}

	/* charging active: VBUSUVLO is a transient worth a retry, not a stop. */
	if (st[2] & SM5440_STATUS3_VBUSUVLO)
		return SM_DC_ERR_RETRY;

	sm5440_get_vbat_mv(chip, &vbat);
	if (vbat < SM5440_VBAT_MIN) {
		dev_err(chip->dev, "dc err: abnormal vbat=%dmV\n", vbat);
		return SM_DC_ERR_INVAL_VBAT;
	}
	return SM_DC_ERR_NONE;
}

/*
 * op 7: which regulation loop is active.  The sm_dc_charging_loop enum values
 * equal the STATUS2 bit positions (IBUSLIM=0x80, VBATREG=0x08, THEMREG=0x02),
 * so the masked STATUS2 bits ARE the loop status.  The target_vbat<=vnow
 * override forces CV when the cell has reached the goal regardless of the bit.
 */
static int sm5440_dc_get_dc_loop_status(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	unsigned int reg = 0;
	int loop = LOOP_INACTIVE, vnow = 0;

	regmap_read(chip->regmap, SM5440_REG_STATUS2, &reg);
	sm5440_get_vbat_mv(chip, &vnow);

	if ((reg & SM5440_STATUS2_VBATREG) ||
	    (chip->dc && chip->dc->target_vbat && chip->dc->target_vbat <= vnow))
		loop = LOOP_VBATREG;
	else if (reg & SM5440_STATUS2_IBUSLIM)
		loop = (reg & SM5440_STATUS2_THEM_REG) ? LOOP_THEMREG : LOOP_IBUSLIM;

	return loop;
}

/*
 * op 8: THE SEAM.  Route the engine's stepped PPS target to the tcpm contract
 * via sm5440_pps_request().  ALWAYS returns 0: the engine treats any < 0 here as
 * a fatal SM_DC_ERR_SEND_PD_MSG and stops the pump, but under tcpm a step can
 * transiently fail (-EAGAIN mid-AMS, a slow PS_RDY -ETIMEDOUT) on an otherwise
 * live contract -- those must NOT fault the engine.  Genuine contract loss is
 * detected out-of-band by the supervisor's contract-active gate + the dc_monitor
 * (which restore the buck), exactly as the bespoke async hook behaved.
 */
static int sm5440_dc_send_power_source_msg(struct i2c_client *i2c,
					   struct sm_dc_power_source_info *ta)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);
	int ret;

	ret = sm5440_pps_request(chip, ta->v, ta->c, false);
	if (ret)
		dev_warn(chip->dev,
			 "dc send: PPS %umV/%umA -> %d (absorbed; not faulting engine)\n",
			 ta->v, ta->c, ret);
	return 0;
}

/* op 9: SW-OCP trigger (async, device-faithful -- must return 0, see below). */
static int sm5440_dc_check_sw_ocp(struct i2c_client *i2c)
{
	struct sm5440 *chip = i2c_get_clientdata(i2c);

	schedule_delayed_work(&chip->ocp_check_work, msecs_to_jiffies(1000));
	return 0;
}

/*
 * Device-faithful SW-OCP backstop (the SM5440 has HW OCP disabled in CNTL2).
 * A sustained IBUSLIM + thermal-shutdown alarm over 3 s == a real over-current;
 * report it out-of-band (NOT synchronously from check_sw_ocp, whose <0 return
 * the engine treats as "the op itself failed" and bails WITHOUT stopping the
 * pump).  sm_dc_report_error_status stops the engine; the dc_monitor then sees
 * the engine left its active states and restores the buck.
 */
static void sm5440_ocp_check_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, ocp_check_work.work);
	unsigned int s2, s3;
	int i;

	for (i = 0; i < 3; i++) {
		if (regmap_read(chip->regmap, SM5440_REG_STATUS2, &s2) ||
		    regmap_read(chip->regmap, SM5440_REG_STATUS3, &s3))
			return;
		if ((s2 & SM5440_STATUS2_IBUSLIM) && (s3 & SM5440_STATUS3_THEMSHDN_ALM)) {
			dev_err(chip->dev, "SW-OCP: IBUSLIM+THEMSHDN (i=%d)\n", i);
			msleep(1000);
		} else {
			return;
		}
	}
	dev_err(chip->dev, "SW-OCP confirmed -> stopping engine (IBUSOCP)\n");
	if (chip->dc)
		sm_dc_report_error_status(chip->dc, SM_DC_ERR_IBUSOCP);
}

static const struct sm_dc_ops sm5440_dc_ops = {
	.get_adc_value		= sm5440_dc_get_adc_value,
	.set_adc_mode		= sm5440_dc_set_adc_mode,
	.get_charging_enable	= sm5440_dc_get_charging_enable,
	.set_charging_enable	= sm5440_dc_set_charging_enable,
	.set_charging_config	= sm5440_dc_set_charging_config,
	.get_dc_error_status	= sm5440_dc_get_dc_error_status,
	.get_dc_loop_status	= sm5440_dc_get_dc_loop_status,
	.send_power_source_msg	= sm5440_dc_send_power_source_msg,
	.check_sw_ocp		= sm5440_dc_check_sw_ocp,
};

/*
 * Common teardown (idempotent): stop the engine, restore the 10 V sustain
 * baseline, re-allow the buck, clear dc_running + auto_engaged (the FG handle is
 * persistent, put in remove).  Called from dc_test=0, the done callback, the
 * monitor's auto-restore, and the supervisor's auto-disengage -- so a topoff/
 * fault/WDT stop can never leave the buck inhibited + pump off = cell not charging.
 */
/*
 * Graceful PPS ramp-down toward 9 V before the buck re-loads the source (Inc-3a,
 * comp 1).  Caller has already turned the pump op-mode OFF (sm_dc_stop_charging),
 * so the input draws ~0 mA and the buck is still inhibited -- the source is
 * UNLOADED.  Step the PPS request down from the present VBUS to SM5440_DC_STOP_TA_MV
 * in SM5440_DC_STOP_STEP_MV increments, settling between each so the source tracks
 * the change with no load fighting it (the sess-206 run-1 Hard-Reset came from the
 * one-shot down-step + immediate buck re-load).  Stops early if the contract is
 * already gone (request returns < 0).  Stays on the PPS APDO throughout, so the
 * keepalive remains consistent and the next engage just re-requests a higher PPS.
 */
static void sm5440_dc_ramp_ta_down(struct sm5440 *chip)
{
	int vbus_uv = 0, mv, i;

	if (sm5440_get_vbus_uv(chip, &vbus_uv) || vbus_uv / 1000 < SM5440_DC_STOP_TA_MV)
		mv = SM5440_DC_STOP_TA_MV;
	else
		mv = vbus_uv / 1000;

	while (mv > SM5440_DC_STOP_TA_MV) {
		mv -= SM5440_DC_STOP_STEP_MV;
		if (mv < SM5440_DC_STOP_TA_MV)
			mv = SM5440_DC_STOP_TA_MV;
		if (sm5440_pps_request(chip, mv, SM5440_DC_STOP_TA_MA, false))
			return;			/* no psy handle */
		msleep(SM5440_DC_STOP_SETTLE_MS);
	}
	/* commit the 9 V target even if we started at/below it (e.g. post-UVLO collapse),
	 * so the buck never re-enables against a stale/high PPS request. */
	if (sm5440_pps_request(chip, SM5440_DC_STOP_TA_MV, SM5440_DC_STOP_TA_MA, false))
		return;

	/*
	 * comp-1 reload-race fix: wait for the source to actually transition to ~9 V
	 * before returning (the caller un-inhibits the buck immediately after).  The
	 * VBUS ADC stays live after sm_dc_stop_charging, and with the pump off + the
	 * buck still inhibited the source is UNLOADED, so it settles quickly once the
	 * keepalive re-Requests 9 V.  Bounded poll; proceed regardless on timeout so a
	 * flaky source can never block the buck restore (cell-not-charging is worse).
	 */
	for (i = 0; i < SM5440_DC_STOP_VERIFY_TRIES; i++) {
		msleep(SM5440_DC_STOP_VERIFY_MS);
		if (!sm5440_get_vbus_uv(chip, &vbus_uv) &&
		    abs(vbus_uv / 1000 - SM5440_DC_STOP_TA_MV) <= SM5440_DC_STOP_TA_MV / 10)
			break;
	}
	dev_info(chip->dev,
		 "dc graceful-stop: PPS settled to %d mV (VBUS=%d mV after ~%d ms) before buck restore\n",
		 SM5440_DC_STOP_TA_MV, vbus_uv / 1000, (i + 1) * SM5440_DC_STOP_VERIFY_MS);
}

static void sm5440_dc_teardown(struct sm5440 *chip)
{
	if (chip->dc)
		sm_dc_stop_charging(chip->dc);	/* pump op-mode OFF first (reverse-current guard) */
	sm5440_dc_ramp_ta_down(chip);		/* gentle PPS ramp to 9 V, settle UNLOADED */
	sm5714_charger_inhibit_buck(false);	/* re-allow the buck (now on a settled 9 V PPS) */
	sm5714_usb_vbus_inhibit_afc_pump(false);	/* pump no longer owns the contract */
	/* chip->fg is persistent (put once in remove); not released here */
	chip->dc_running = false;
	chip->auto_engaged = false;		/* a fresh run (manual or auto) re-decides ownership */
}

/*
 * LOG-ONLY engine monitor.  Unlike Inc-1's pump_monitor it does NOT auto-
 * disengage on a tick count (that would rip control from the engine); it only
 * logs telemetry (incl. the headline FG cell-current) and, if the engine has
 * left its active states (topoff/fault/WDT-drop), runs the unconditional
 * restore.  Takes engage_lock; dc_test=0 sync-cancels it WITHOUT the lock.
 */
static void sm5440_dc_monitor_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, dc_monitor.work);
	int state, vbus = 0, ibus = 0, vout = 0, vbat = 0, die = 0, fg_ua = 0, opmode;
	union power_supply_propval pv;

	mutex_lock(&chip->engage_lock);
	if (!chip->dc_running) {
		mutex_unlock(&chip->engage_lock);
		return;
	}

	state = chip->dc ? sm_dc_get_current_state(chip->dc) : SM_DC_CHG_OFF;
	opmode = sm5440_get_op_mode(chip);
	sm5440_get_vbus_uv(chip, &vbus);
	sm5440_get_ibus_ua(chip, &ibus);
	sm5440_get_vout_mv(chip, &vout);
	sm5440_get_vbat_mv(chip, &vbat);
	sm5440_get_dietemp_dc(chip, &die);
	if (chip->fg && !power_supply_get_property(chip->fg,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pv))
		fg_ua = pv.intval;

	dev_info(chip->dev,
		 "dc[%3d] st=%d op=%d VBUS=%dmV 2xVOUT=%dmV IBUS=%dmA | cell-VBAT=%dmV FG-I=%dmA | die=%d.%dC\n",
		 chip->dc_ticks++, state, opmode, vbus / 1000, vout * 2, ibus / 1000,
		 vbat, fg_ua / 1000, die / 10, die % 10);

	/* engine left the active states (CHG_OFF/ERR/EOC) -> restore the buck. */
	if (state < SM_DC_CHECK_VBAT) {
		bool was_auto = chip->auto_engaged;	/* capture before teardown clears it */
		int rsoc = sm5440_read_soc(chip);
		/*
		 * Build-2b discriminator.  Only two things reach here: a benign engine
		 * self-topoff (done_cb -> CHG_OFF; the SoC-95 auto-disengage goes through
		 * dc_stop, which sync-cancels this monitor first, so it never reaches
		 * here) and a mid-charge fault/HRST (-> SM_DC_ERR).  dc_done separates
		 * them robustly (no dependence on the exact topoff SoC); soc>=disengage is
		 * belt-and-suspenders.
		 */
		bool benign = READ_ONCE(chip->dc_done) ||
			      (rsoc >= 0 && rsoc >= SM5440_AUTO_DISENGAGE_SOC);

		WRITE_ONCE(chip->dc_inject_hrst, false);	/* TEST: the injection (if any) has now fired */
		dev_warn(chip->dev, "dc engine stopped (state=%d, %s) -> restoring buck\n",
			 state, benign ? "benign topoff" : "HRST/fault");
		sm5440_dc_teardown(chip);		/* clears dc_running + auto_engaged */

		/*
		 * A supervisor-owned run that stopped mid-charge without a topoff = an
		 * HRST/fault teardown -> arm the recovery worker (re-activate PPS +
		 * re-engage), bounded by the 3-strike latch.  A benign topoff, a manual
		 * dc_test run (!was_auto), or an already-latched state is left alone.
		 */
		if (was_auto && !benign && !chip->dc_err_latched) {
			chip->dc_recover_armed = true;
			mod_delayed_work(system_long_wq, &chip->dc_reengage_work,
					 msecs_to_jiffies(SM5440_REENGAGE_DELAY_MS));
			dev_info(chip->dev,
				 "reengage: armed (was_auto=1 soc=%d%%) -- attempting PPS recovery\n",
				 rsoc);
		}
		mutex_unlock(&chip->engage_lock);
		return;
	}

	/*
	 * The input-current ceiling = min(Build-2a VBAT-aware cap, device-own
	 * step_ibus[dc_step]).  Reached only with the engine ACTIVE (past the teardown
	 * check), so a re-target never lands on a dead engine.  FG voltage_now is the
	 * LOADED cell reading (the boundary model's domain); it reads ~300 mV above the
	 * resting cell under a ~7.6 A charge, and is a DIFFERENT quantity from the ~2x
	 * battery-side cell current.
	 *
	 * Build-2a cap: sm5440_ci_gl_cap() falls per VBAT band as the source's PS_RDY
	 * boundary falls; we ratchet it DOWN (never up -- VBAT rises monotonically on
	 * charge) via set_ta_cmax (holds the CC peak AT ci_gl) + set_target_ibus (diverts
	 * the engine to a PRESET re-ramp at the lower target -- inherently safe on a
	 * REDUCTION, re-ramping from 50% of the new cap).  This caps steps 0 AND 1: the
	 * pump runs continuously ENGAGE_SOC(62)..DISENGAGE_SOC(95), so step 1's device-own
	 * 4100 mA would itself over-request at its high entry VBAT -- the min() caps it.
	 */
	{
		int fgv_uv = 0, cell_mv;

		if (chip->fg && !power_supply_get_property(chip->fg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pv))
			fgv_uv = pv.intval;
		cell_mv = fgv_uv / 1000;

		if (READ_ONCE(chip->dc_inject_hrst) && !chip->dc_adaptive) {
			/*
			 * TEST-ONLY (Build-2b validation): HOLD an over-request above the
			 * PS_RDY boundary EACH tick (bypassing the ci_gl ratchet, which would
			 * otherwise claw it straight back down) until the engine climbs past
			 * the boundary and tcpm SENDS a Hard Reset.  The teardown branch above
			 * clears the flag once the engine stops, so recovery re-engages at the
			 * normal capped ci_gl with no re-injection.
			 */
			sm_dc_set_ta_cmax(chip->dc, SM5440_HRST_INJECT_MA);
			sm_dc_set_target_ibus(chip->dc, SM5440_HRST_INJECT_MA);
			chip->dc_ci_applied = SM5440_HRST_INJECT_MA;
			dev_warn(chip->dev,
				 "TEST: HRST-inject holding over-request %u mA at FG-V=%dmV\n",
				 SM5440_HRST_INJECT_MA, cell_mv);
		} else {
			u32 cap = sm5440_ci_gl_cap(cell_mv);
			u32 eff = min_t(u32, cap, sm5440_dc_step_ibus[chip->dc_step]);
			/* Fixed (production auto-engage / dc_test) runs only: the adaptive dc_probe
			 * deliberately holds ci_gl HIGH to push ta.v_max, so the cap must not fight it. */
			if (!chip->dc_adaptive && cell_mv > 0 && eff < chip->dc_ci_applied) {
				sm_dc_set_ta_cmax(chip->dc, eff);	/* hold the CC peak at ci_gl (no +offset) */
				sm_dc_set_target_ibus(chip->dc, eff);	/* divert to the re-preset at the lower cap */
				dev_info(chip->dev,
					 "ci_gl cap: %u -> %u mA at FG-V=%dmV (step=%d) -- VBAT-aware, boundary-safe\n",
					 chip->dc_ci_applied, eff, cell_mv, chip->dc_step);
				chip->dc_ci_applied = eff;
			}
		}

		/*
		 * The device-own 3-step CV-float ladder (sec_step_charging.c
		 * min(step_vol, step_input)).  Advance N->N+1 when the cell reaches this
		 * step's plateau (FG cell-V + v_margin >= cond_vol[N]) AND the bus current
		 * has tapered (IBUS <= cond_iin[N]), debounced SM5440_DC_STEP_IIN_CNT ticks.
		 * The step now owns ONLY target_vbat (the CV ceiling); the INPUT current is
		 * owned by the min()-cap above, so a step entry never RAISES ibus into a
		 * high-VBAT over-request.  (Build-1's old "vbat-up + ibus-down ->
		 * need_to_preset=0, no Hard-Reset" claim held only for the UNCAPPED table:
		 * under the 4000 cap step 0->1 was ibus 4000->4100 = UP, which is exactly the
		 * over-request the cap now prevents.)  With the cap in force IBUS <=
		 * cond_iin[N] is trivially true, so advance is cell-V gated -- the correct
		 * CV-transition trigger.  Step 2 (4440 mV) then self-terminates via the
		 * engine's native topoff; SoC >= dchg_end_soc is the ceiling backstop.
		 */
		if (chip->dc_step < SM5440_DC_STEP_MAX) {
			int n = chip->dc_step;

			if (cell_mv + SM5440_DC_STEP_V_MARGIN >= sm5440_dc_step_cond_vol[n] &&
			    ibus / 1000 <= sm5440_dc_step_cond_iin[n]) {
				if (++chip->dc_step_iin_cnt >= SM5440_DC_STEP_IIN_CNT) {
					chip->dc_step = n + 1;
					chip->dc_step_iin_cnt = 0;
					sm_dc_set_target_vbat(chip->dc, sm5440_dc_step_vbat[n + 1]);
					dev_info(chip->dev,
						 "dc ladder: step %d -> %d (vbat %u->%u mV; ibus owned by ci_gl cap) at FG-V=%dmV IBUS=%dmA\n",
						 n, n + 1, sm5440_dc_step_vbat[n], sm5440_dc_step_vbat[n + 1],
						 cell_mv, ibus / 1000);
				}
			} else {
				chip->dc_step_iin_cnt = 0;
			}
		}
	}

	schedule_delayed_work(&chip->dc_monitor, msecs_to_jiffies(1000));
	mutex_unlock(&chip->engage_lock);
}

/*
 * Adaptive ta.v_max probe (Inc-3b comp-2).  Runs every SM5440_PROBE_TICK_MS while
 * an adaptive dc run is active.  The collapse-guard runs EVERY tick (the fast
 * safety retreat); the climb runs only while !probe_locked AND the engine is in
 * CC.  Once locked the guard keeps running -- only the climb stops.  Lock order
 * is engage_lock -> st_lock (via sm_dc_set_ta_vmax), the same order teardown
 * already uses.  dc_test/dc_probe=0 sync-cancels this WITHOUT engage_lock; teardown
 * clears dc_running so an in-flight tick self-terminates with no explicit cancel.
 */
/*
 * ta.v_max MUST be a multiple of the 20 mV PPS request granularity (the engine's
 * PPS_V_STEP).  The engine clamps ta.v = pps_v(MIN(.., ta.v_max)) and pps_v() rounds
 * to the NEAREST step -- so a non-step-aligned ta.v_max (the probe derives it from
 * 2*Vcell + the cable-IR offset, rarely a 20 mV multiple) lets the clamped request
 * round UP past ta.v_max and trip the engine's own out-of-bounds check
 * (send_power_source_msg "out of bounce" -> SM_DC_ERR_SEND_PD_MSG, faults in PRE_CC).
 * Flooring ta.v_max to the step guarantees pps_v(x) <= ta.v_max for any x <= ta.v_max,
 * keeping every engine clamp site in bounds.  The fixed dc_test path is unaffected
 * (its ta.v_max = OVP_TH - 500 = 10500 mV is already aligned).
 */
#define SM5440_PPS_V_STEP	20
static inline u32 sm5440_vmax_align(u32 mv)
{
	return (mv / SM5440_PPS_V_STEP) * SM5440_PPS_V_STEP;
}

static void sm5440_dc_probe_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440, probe_work.work);
	int vbus_uv = 0, vbat_mv = 0, ibus_ua = 0, vbus_mv, ibus_ma, floor, state;

	mutex_lock(&chip->engage_lock);
	if (!chip->dc_running || !chip->dc_adaptive)
		goto out;			/* torn down -> self-terminate (no resched) */

	sm5440_get_vbus_uv(chip, &vbus_uv);
	sm5440_get_vbat_mv(chip, &vbat_mv);
	sm5440_get_ibus_ua(chip, &ibus_ua);
	vbus_mv = vbus_uv / 1000;
	ibus_ma = ibus_ua / 1000;
	/* PPS-step-aligned so the engine's pps_v() clamp to it can't round above it */
	floor = sm5440_vmax_align(2 * vbat_mv + chip->probe_floor_off);	/* never retreat below the engage point */

	/*
	 * (1) collapse-guard, EVERY tick.  The device VBUS is the PPS request minus the
	 * cable IR-drop; if it sags toward the 2*Vcell 2:1 floor the source is being
	 * over-drawn -> drop ta.v_max one step immediately.  Easing the pump's pull lets
	 * the source recover BEFORE the sag deepens into a VBUSUVLO cliff (the
	 * flat-detector below is too slow to prevent that on its own).
	 */
	if (vbus_mv < 2 * vbat_mv + SM5440_PROBE_COLLAPSE_MV) {
		chip->probe_vmax = max_t(int, floor,
					 (int)chip->probe_vmax - SM5440_PROBE_VMAX_STEP);
		sm_dc_set_ta_vmax(chip->dc, chip->probe_vmax);
		if (!chip->probe_locked) {
			chip->probe_prev_ibus = ibus_ma;
			chip->probe_settle = SM5440_PROBE_SETTLE_TICKS;
		}
		dev_warn(chip->dev,
			 "probe: collapse-guard VBUS=%dmV < 2*Vcell(%d)+%d -> v_max down to %dmV\n",
			 vbus_mv, vbat_mv, SM5440_PROBE_COLLAPSE_MV, chip->probe_vmax);
		goto resched;
	}

	/* climb only while not locked and the engine is actually regulating in CC. */
	state = chip->dc ? sm_dc_get_current_state(chip->dc) : SM_DC_CHG_OFF;
	if (chip->probe_locked || state != SM_DC_CC)
		goto resched;

	/* (2) wait out the post-bump settle before judging the IBUS response. */
	if (chip->probe_settle > 0) {
		chip->probe_settle--;
		goto resched;
	}

	if (ibus_ma >= (int)chip->probe_ci_gl - SM5440_PROBE_FLAT_MA) {
		/* (3a) success: input current reached the goal -> lock, keep v_max. */
		chip->probe_locked = true;
		dev_info(chip->dev, "probe: IBUS=%dmA reached ci_gl(%d) -> LOCKED at v_max=%dmV\n",
			 ibus_ma, chip->probe_ci_gl, chip->probe_vmax);
	} else if (ibus_ma - chip->probe_prev_ibus < SM5440_PROBE_FLAT_MA) {
		/* (3b) flat below the goal: source ceiling -> revert one step + lock. */
		chip->probe_vmax = max_t(int, floor,
					 (int)chip->probe_vmax - SM5440_PROBE_VMAX_STEP);
		sm_dc_set_ta_vmax(chip->dc, chip->probe_vmax);
		chip->probe_locked = true;
		dev_info(chip->dev,
			 "probe: IBUS=%dmA flat (prev %d, ci_gl %d) -> source ceiling, LOCKED at v_max=%dmV\n",
			 ibus_ma, chip->probe_prev_ibus, chip->probe_ci_gl, chip->probe_vmax);
	} else {
		/* (3c) IBUS responded and is below the goal: bump v_max one step. */
		chip->probe_vmax = min_t(u32, chip->probe_vmax + SM5440_PROBE_VMAX_STEP,
					 SM5440_PROBE_VMAX_CEIL);
		sm_dc_set_ta_vmax(chip->dc, chip->probe_vmax);
		chip->probe_prev_ibus = ibus_ma;
		chip->probe_settle = SM5440_PROBE_SETTLE_TICKS;
		if (chip->probe_vmax >= SM5440_PROBE_VMAX_CEIL)
			chip->probe_locked = true;	/* hit the hard request ceiling */
		dev_info(chip->dev, "probe: IBUS=%dmA responding -> v_max up to %dmV%s\n",
			 ibus_ma, chip->probe_vmax,
			 chip->probe_locked ? " (request ceiling, LOCKED)" : "");
	}

resched:
	schedule_delayed_work(&chip->probe_work, msecs_to_jiffies(SM5440_PROBE_TICK_MS));
out:
	mutex_unlock(&chip->engage_lock);
}

/* The engine's topoff "done" event (CV, target==float).  Stop the engine; the
 * monitor's next tick sees CHG_OFF and restores the buck. */
static void sm5440_dc_done_cb(void *ctx)
{
	struct sm5440 *chip = ctx;

	/*
	 * Build-2b: mark the benign topoff BEFORE stopping the engine, so the next
	 * dc_monitor tick (which sees state<CHECK_VBAT) classifies this as a topoff,
	 * not an HRST, and does NOT arm the re-engage recovery.
	 */
	WRITE_ONCE(chip->dc_done, true);
	dev_info(chip->dev, "dc: charging done (topoff) -> stopping engine\n");
	if (chip->dc)
		sm_dc_stop_charging(chip->dc);
}

/*
 * Engine START.  Caller holds engage_lock and has checked !dc_running.  Reuses the
 * Inc-1 proven precondition (buck OFF -> chip sw_reset+init -> step PPS to
 * ~2*Vcell and VERIFY VBUS settled) so the engine's preset re-send is a near
 * no-op against an already-2:1-ready source, THEN hands control to the engine.
 */
static int sm5440_dc_start(struct sm5440 *chip, u32 target_ibus_ma, bool adaptive)
{
	struct sm_dc_power_source_info ta = { };
	int vbat_mv = 0, target_mv, off_mv, vbus_mv, diff, ret, engage_off, i;

	if (!sm5440_pps_contract_active(chip)) {
		dev_warn(chip->dev, "dc start: no PPS contract -- arm pd_request + plug first\n");
		return -ENOTCONN;
	}

	ret = sm5714_charger_inhibit_buck(true);	/* buck OFF before pump ON */
	if (ret) {
		dev_warn(chip->dev, "dc start: buck inhibit failed (%d)\n", ret);
		return ret;
	}
	sm5714_usb_vbus_inhibit_afc_pump(true);		/* pump owns the contract: stand AFC down */

	/* chip prep: sw_reset + device-faithful init + continuous ADC, NO op-mode
	 * (the engine flips op-mode itself in preset's set_charging_enable). */
	ret = sm5440_sw_reset(chip);
	if (ret)
		goto err_restore;
	sm5440_init_reg_param(chip);
	regmap_write(chip->regmap, SM5440_REG_ADCCNTL1, SM5440_ADCCNTL1_ENABLE);

	/* precondition the PPS input to ~2*Vcell + IR offset, clamped to
	 * [ta_min_voltage, dc_vbus_ovp_th - 500], and verify VBUS transitioned. */
	ret = sm5440_get_vbat_mv(chip, &vbat_mv);
	if (ret || vbat_mv < 2500 || vbat_mv > 4500) {
		dev_warn(chip->dev, "dc start: implausible VBAT %dmV -- abort\n", vbat_mv);
		ret = ret ? ret : -EIO;
		goto err_restore;
	}
	off_mv = (target_ibus_ma * (SM5440_GTS9U_R_TTL_UOHM / 1000)) / 1000 + 200;
	target_mv = 2 * vbat_mv + off_mv;
	target_mv = min(target_mv, SM5440_DC_VBUS_OVP_TH - 500);
	target_mv = max(target_mv, SM5440_TA_MIN_VOLTAGE);

	ret = sm5440_pps_request(chip, target_mv, target_ibus_ma, false);
	if (ret) {
		dev_warn(chip->dev, "dc start: PPS request %dmV failed (%d)\n", target_mv, ret);
		goto err_restore;
	}
	dev_info(chip->dev, "dc start: VBAT=%dmV -> PPS target=%dmV; settling\n",
		 vbat_mv, target_mv);

	/*
	 * Settle + verify VBUS reached the PPS target.  sm5440_get_vbus_uv reads
	 * the SM5440 ADC, which was sw_reset just above -- its first conversions
	 * lag the freshly-stepped rail, so a single 300 ms read caught a stale
	 * pre-step sample (~9 V) and aborted a healthy 10 V engage.  Re-read until
	 * the ADC tracks the rail (within 10 %) or a generous timeout; the rail
	 * itself steps fine (confirmed by a userspace PPS sweep).
	 */
	vbus_mv = 0;
	diff = target_mv;
	for (i = 0; i < 12; i++) {
		msleep(150);
		sm5440_get_vbus_uv(chip, &vbus_mv);
		vbus_mv /= 1000;
		diff = vbus_mv - target_mv;
		if (diff < 0)
			diff = -diff;
		if (diff <= target_mv / 10)
			break;
	}
	if (diff > target_mv / 10) {
		dev_warn(chip->dev, "dc start: VBUS=%dmV not within 10%% of %dmV after settle (PPS not transitioned) -- abort\n",
			 vbus_mv, target_mv);
		ret = -EIO;
		goto err_restore;
	}
	dev_info(chip->dev, "dc start: VBUS settled to %dmV (%d reads) -- handing to the engine\n",
		 vbus_mv, i + 1);

	/*
	 * TA descriptor: our 10 V APDO (pos 5); the engine ramps within it.  c_max =
	 * the step target (the PD layer clamps to the APDO's advertised max current).
	 * For the ADAPTIVE probe ta.v_max STARTS conservative -- the engine's own
	 * engage request (2*Vcell + the half-current IR offset) plus a small margin --
	 * so the CC ramp settles at a safe sub-ceiling current and probe_work then
	 * walks it up; the fixed dc_test path uses the full 10500 mV (ci_gl is the
	 * limiter there).  p_max always uses the full 10500 ceiling so the engine's own
	 * p_max guard never caps the probe's climb before ta.v_max does.
	 */
	engage_off = ((target_ibus_ma / 2) * (SM5440_GTS9U_R_TTL_UOHM / 1000)) / 1000 + 200;
	chip->probe_floor_off = engage_off;	/* ta.v_max never retreats below 2*Vcell + this */
	chip->probe_vmax = sm5440_vmax_align(min_t(u32,
				 2 * vbat_mv + engage_off + SM5440_PROBE_START_MARGIN,
				 SM5440_PROBE_VMAX_CEIL));

	ta.pdo_pos = 5;
	ta.v_max = adaptive ? chip->probe_vmax : (SM5440_DC_VBUS_OVP_TH - 500);
	ta.c_max = target_ibus_ma;
	ta.p_max = (SM5440_DC_VBUS_OVP_TH - 500) * ta.c_max;

	sm_dc_set_target_vbat(chip->dc, SM5440_DC_STEP0_VBAT_MV);
	sm_dc_set_target_ibus(chip->dc, target_ibus_ma);

	sm5440_fg(chip);			/* lazy-cache the persistent FG handle */
	chip->dc_running = true;
	chip->dc_ticks = 0;
	chip->dc_done = false;		/* Build-2b: a fresh run has not topoff'd */
	chip->dc_recover_armed = false;	/* Build-2b: a fresh run is not itself in recovery */
	WRITE_ONCE(chip->dc_inject_hrst, false);	/* TEST: clear any stale injection arming */
	chip->dc_step = 0;		/* the ladder starts in step 0 (engage gate keeps SoC < 62) */
	chip->dc_step_iin_cnt = 0;
	chip->dc_ci_applied = target_ibus_ma;	/* Build-2a: cap-ratchet baseline; dc_monitor lowers it as the loaded cell climbs.
						 * Safe even on a high-VBAT (re-)engage: the engine PRESETs at 50% of target and
						 * ramps ~PPS_C_STEP/tick, so the first 1 s dc_monitor tick reads the loaded cell-V
						 * and ratchets the cap down long before the request could reach the boundary. */
	chip->dc_adaptive = adaptive;
	chip->probe_locked = false;
	chip->probe_ci_gl = target_ibus_ma;
	chip->probe_prev_ibus = 0;
	chip->probe_settle = SM5440_PROBE_SETTLE_TICKS;

	ret = sm_dc_start_charging(chip->dc, &ta);
	if (ret) {
		dev_err(chip->dev, "dc start: sm_dc_start_charging failed (%d)\n", ret);
		sm5440_dc_teardown(chip);	/* clears dc_running, restores buck */
		return ret;
	}

	schedule_delayed_work(&chip->dc_monitor, msecs_to_jiffies(1000));
	if (adaptive) {
		schedule_delayed_work(&chip->probe_work,
				      msecs_to_jiffies(SM5440_PROBE_TICK_MS));
		dev_info(chip->dev,
			 "dc ENGINE STARTED (ADAPTIVE probe): ci_gl=%umA v_max start=%umV (engage~%dmV) ceil=%dmV\n",
			 target_ibus_ma, chip->probe_vmax, 2 * vbat_mv + engage_off,
			 SM5440_PROBE_VMAX_CEIL);
	} else {
		dev_info(chip->dev,
			 "dc ENGINE STARTED (fixed): target_vbat=%dmV target_ibus=%umA -- monitoring\n",
			 SM5440_DC_STEP0_VBAT_MV, target_ibus_ma);
	}
	return 0;

err_restore:
	sm5440_dc_ramp_ta_down(chip);		/* gentle PPS restore (same buck handoff as teardown) */
	sm5714_charger_inhibit_buck(false);
	sm5714_usb_vbus_inhibit_afc_pump(false);	/* dc start aborted: release the AFC stand-down */
	return ret;
}

/*
 * Stop + unconditional restore, shared by dc_test=0 and dc_probe=0.  Sync-cancel
 * the LOG monitor AND the adaptive probe WITHOUT engage_lock (both take it --
 * holding it here would deadlock the sync-cancel), then one locked teardown.
 */
static void sm5440_dc_stop(struct sm5440 *chip)
{
	cancel_delayed_work_sync(&chip->probe_work);
	cancel_delayed_work_sync(&chip->dc_monitor);		/* the recovery armer -- stop it first */
	cancel_delayed_work_sync(&chip->dc_reengage_work);	/* Build-2b: kill a pending recovery */
	mutex_lock(&chip->engage_lock);
	if (chip->dc_running)
		sm5440_dc_teardown(chip);
	chip->dc_recover_armed = false;		/* Build-2b: a manual/auto stop disarms recovery */
	mutex_unlock(&chip->engage_lock);
}

/*
 * Increment-3b-3 supervisor -- the buck worker's notify callback.  Runs in the buck
 * worker's context and MUST be non-blocking (the buck worker holds the vbus
 * instance_lock across this call): just publish the intent and kick the executor.
 * No engage_lock here -- taking it under instance_lock could invert against the
 * engage path's engage_lock -> instance_lock -> sv->lock ordering (inhibit_buck).
 * WRITE_ONCE pairs with the executor's READ_ONCE.  system_long_wq because the
 * executor may sleep for the full engage (msleep 300) / graceful-stop ramp (~3 s).
 */
static void sm5440_dc_intent_cb(void *ctx, bool intent)
{
	struct sm5440 *chip = ctx;
	bool commit = READ_ONCE(chip->dc_intent);	/* default: HOLD the last committed value */

	/*
	 * Debounce the FALSE transition.  dc_intent is VBUS-POK keyed
	 * (sm5714-usb-vbus.c: dc_intent = vbus && !charge_cut) and a PD Hard Reset
	 * drives VBUS to vSafe0V for ~825 ms.  The buck worker is a pure 3 s poller (no
	 * IRQ), so at most ONE poll samples that sub-period transient; committing
	 * dc_intent=false on it would misfire auto_engage_work's !intent disengage,
	 * cancel the dc_reengage_work recovery, and strand the pump on a fixed contract.
	 * Hold true across a single false poll; commit false only after
	 * SM5440_DC_INTENT_FALSE_DEBOUNCE consecutive false polls (a genuine unplug).  A
	 * true notify clears the counter at once (charger present overrides).  The
	 * counter is cb-only -- this cb runs synchronously inside the buck worker's
	 * single, serial charger_work (the sole writer of dc_intent) -- so it needs no
	 * locking; READ_ONCE/WRITE_ONCE stay on dc_intent for its cross-thread readers.
	 */
	if (intent) {
		chip->dc_intent_false_cnt = 0;
		commit = true;
	} else if (++chip->dc_intent_false_cnt >= SM5440_DC_INTENT_FALSE_DEBOUNCE) {
		commit = false;
	}

	if (READ_ONCE(chip->dc_intent) != commit)
		dev_info(chip->dev, "dc_intent -> %d (buck notify; false_cnt=%d)\n",
			 commit, chip->dc_intent_false_cnt);
	else if (!intent && chip->dc_intent_false_cnt == 1)
		dev_info(chip->dev,
			 "dc_intent HELD true across a transient VBUS-off (false_cnt=1) -- HRST debounce suppressed a disengage\n");
	WRITE_ONCE(chip->dc_intent, commit);
	/*
	 * Kick the executor UNCONDITIONALLY (not only on a commit change): the
	 * auto-engage retry path re-attempts a failed dc_start "on the next poll",
	 * which only happens because every notify kicks auto_engage_work.
	 */
	mod_delayed_work(system_long_wq, &chip->auto_engage_work, 0);
}

/*
 * The executor.  Engage when the arbiter signals intent AND the pump-side gates
 * pass (a PPS contract is held, SoC below the step-0 ceiling, no fault latch);
 * disengage a SUPERVISOR-OWNED run when intent drops, the contract is lost, or SoC
 * reaches the disengage ceiling.  Step-0's 4250 mV != the 4440 mV float, so the
 * engine never self-DONEs and this SoC ceiling is the SOLE terminator -- an
 * unreadable SoC therefore fails SAFE to disengage (mirrors the buck's temp-read
 * fail-to-cut).  engage_lock serializes against the manual triggers + the monitors,
 * so a concurrent invocation can never double-engage.  dc_start/dc_stop run WITHOUT
 * engage_lock held across their sync-cancels (dc_stop sync-cancels the monitors,
 * which themselves take engage_lock).
 */
static void sm5440_auto_engage_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440,
					   auto_engage_work.work);
	bool intent = READ_ONCE(chip->dc_intent);
	int soc, ret;

	mutex_lock(&chip->engage_lock);
	soc = sm5440_read_soc(chip);

	if (chip->dc_running) {
		/*
		 * Only manage a run the supervisor itself started; leave a manual
		 * dc_test/dc_probe run to the operator (dc_test=0).
		 *
		 * A mid-charge PPS contract loss is deliberately NOT a disengage
		 * trigger: a fault (HRST/collapse) drops the contract with intent
		 * still held, and disengaging here would sm5440_dc_stop() -> cancel
		 * dc_monitor + dc_reengage_work + clear dc_recover_armed BEFORE the
		 * dc_monitor teardown tick can arm recovery -- defeating the
		 * re-engage-on-HRST recovery.  Let a contract-loss-with-intent flow
		 * to the engine fault -> dc_monitor -> recovery path instead.  A
		 * genuine unplug still disengages via !intent; the 3-strike latch
		 * backstops a truly deaf source.
		 */
		if (chip->auto_engaged &&
		    (!intent ||
		     soc < 0 || soc >= SM5440_AUTO_DISENGAGE_SOC)) {
			dev_info(chip->dev,
				 "auto-disengage: intent=%d contract=%d soc=%d%%\n",
				 intent, sm5440_pps_contract_active(chip), soc);
			mutex_unlock(&chip->engage_lock);
			sm5440_dc_stop(chip);
			return;
		}
		mutex_unlock(&chip->engage_lock);
		return;
	}

	/* Idle.  The charger going away (intent low) clears the fault latch. */
	if (!intent) {
		chip->dc_err_latched = false;
		chip->dc_retry_cnt = 0;
		chip->dc_recover_armed = false;	/* Build-2b: unplug disarms any pending recovery */
		mutex_unlock(&chip->engage_lock);
		return;
	}

	/*
	 * Engage gates: not already on the pump, not latched-off after repeated
	 * failures, a PPS contract is held, and SoC is known and below the step-0
	 * ceiling (above it a step-0-only engine has nothing to do; the buck owns it).
	 */
	if (chip->pump_engaged || chip->dc_err_latched ||
	    !sm5440_pps_contract_active(chip) ||
	    soc < 0 || soc >= SM5440_AUTO_ENGAGE_SOC) {
		mutex_unlock(&chip->engage_lock);
		return;
	}

	/*
	 * Force a fresh PPS (re-)activation before engaging.  If a prior HRST collapse
	 * routed recovery through auto-disengage->auto-engage, the contract can be a
	 * FIXED PDO while pps_active stayed stale-true (sm5440_pps_contract_active reads
	 * true on a PPS-capable adapter even at fixed 9 V, so the keepalive never cleared
	 * it) -- then pps_request skips PROG_ONLINE and the pump strands on the fixed
	 * contract at half rate.  Mirror the reengage worker (which forces pps_active
	 * false) so the auto-engage fallback rebuilds real PPS regardless of which
	 * recovery path ran.  Kept here (not in dc_start) so reengage does not
	 * double-force and the manual dc_test/dc_probe paths are unchanged.
	 */
	WRITE_ONCE(chip->pps_active, false);
	ret = sm5440_dc_start(chip, SM5440_AUTO_ENGAGE_CI_GL, false);
	if (ret) {
		if (++chip->dc_retry_cnt >= SM5440_AUTO_RETRY_MAX) {
			chip->dc_err_latched = true;
			dev_warn(chip->dev,
				 "auto-engage: %d consecutive failures -- latching off until charger replug\n",
				 chip->dc_retry_cnt);
		} else {
			dev_warn(chip->dev,
				 "auto-engage: dc_start failed (%d) -- retry %d/%d on next poll\n",
				 ret, chip->dc_retry_cnt, SM5440_AUTO_RETRY_MAX);
		}
	} else {
		chip->dc_retry_cnt = 0;
		chip->auto_engaged = true;
		dev_info(chip->dev,
			 "auto-engage: pump engaged at SoC %d%% (ci_gl=%u mA) -- no sysfs trigger\n",
			 soc, SM5440_AUTO_ENGAGE_CI_GL);
	}
	mutex_unlock(&chip->engage_lock);
}

/*
 * Build-2b: the re-engage-on-HRST recovery worker.  Armed by dc_monitor when a
 * supervisor-owned run stops mid-charge without a topoff (an HRST dropped the PPS
 * contract to a FIXED PDO and the pump fell to the buck; sess-233 §9).  It
 * re-activates PPS (re-driving PROG_ONLINE, which the keepalive stopped doing on
 * the contract loss) and re-engages, bounded by the same 3-strike dc_retry_cnt
 * latch as the engage path.
 *
 * Lock discipline mirrors dc_start's contract: the blocking PPS re-activation poll
 * (set_property blocks up to ~10 s) runs WITHOUT engage_lock, like the keepalive;
 * only the pre-flight gate and the final dc_start take it.  A "PPS not back yet"
 * within the (generous, shim-reattach-exceeding) window is a WAIT, not a strike;
 * only a full-window-deaf source OR contract-back-but-dc_start-failed is a strike.
 * The recovery ceiling is the DISENGAGE SoC (95), not the ENGAGE gate (62): this
 * CONTINUES an already-active run that legitimately climbed above 62 (the pump runs
 * 62..95 once engaged), so it must not surrender the pump for a high-SoC HRST --
 * where HRSTs are in fact more likely (the PS_RDY boundary falls with VBAT).  The
 * ci_gl cap + this latch + the topoff self-limit bound the high-SoC re-engage.
 */
static void sm5440_dc_reengage_work(struct work_struct *work)
{
	struct sm5440 *chip = container_of(work, struct sm5440,
					   dc_reengage_work.work);
	bool contract = false;
	int soc, i, ret;

	/*
	 * Pre-flight under the lock: bail if superseded (a manual run started,
	 * disarmed, or already latched) or the charger/SoC left the window.
	 */
	mutex_lock(&chip->engage_lock);
	if (chip->dc_running || !chip->dc_recover_armed || chip->dc_err_latched) {
		mutex_unlock(&chip->engage_lock);
		return;
	}
	soc = sm5440_read_soc(chip);
	if (!READ_ONCE(chip->dc_intent) || soc < 0 || soc >= SM5440_AUTO_DISENGAGE_SOC) {
		/*
		 * Genuine unplug, unreadable SoC, or SoC reached the disengage ceiling
		 * (the charge is effectively done -- the buck owns 95..100): abandon
		 * WITHOUT a strike (unplug clears the latch via auto_engage_work's idle
		 * path; a fail-safe SoC read leaves the buck in charge).
		 */
		chip->dc_recover_armed = false;
		dev_info(chip->dev, "reengage: abandon (intent=%d soc=%d%%) -- not a strike\n",
			 READ_ONCE(chip->dc_intent), soc);
		mutex_unlock(&chip->engage_lock);
		return;
	}
	mutex_unlock(&chip->engage_lock);

	/*
	 * Force pps_active off so the FIRST pps_request definitely re-enters the
	 * PROG_ONLINE activation path (the keepalive should already have cleared it on
	 * the contract loss, but do not depend on that race).  Then poll: re-invoke
	 * pps_request EACH iteration -- a single post-HRST call hits the "activate
	 * deferred" bail because the port is still mid-transition after the reset.
	 */
	WRITE_ONCE(chip->pps_active, false);
	for (i = 0; i < SM5440_REENGAGE_POLL_TRIES; i++) {
		if (!READ_ONCE(chip->dc_intent))
			break;			/* unplugged mid-poll -> handled below (no strike) */
		sm5440_pps_request(chip, SM5440_REENGAGE_PPS_MV,
				   SM5440_REENGAGE_PPS_MA, false);
		if (sm5440_pps_contract_active(chip)) {
			contract = true;
			break;
		}
		msleep(SM5440_REENGAGE_POLL_MS);
	}

	/* Decide under the lock. */
	mutex_lock(&chip->engage_lock);
	if (chip->dc_running || !chip->dc_recover_armed) {
		mutex_unlock(&chip->engage_lock);	/* superseded during the poll */
		return;
	}
	if (!READ_ONCE(chip->dc_intent)) {		/* unplugged during the poll: no strike */
		chip->dc_recover_armed = false;
		mutex_unlock(&chip->engage_lock);
		return;
	}

	if (contract) {
		union power_supply_propval pv = { };
		int cell_mv = 0;
		u32 reengage_ci;

		/*
		 * Re-engage at the VBAT-band ci_gl cap (mirroring the dc_monitor
		 * ratchet's fuelgauge read), NOT the fixed SM5440_AUTO_ENGAGE_CI_GL:
		 * dc_start seeds dc_ci_applied = target, so starting AT the band cap
		 * makes the first dc_monitor tick eff == dc_ci_applied -- no immediate
		 * re-ratchet.  Re-engaging at 4000 above the boundary would re-preset
		 * -> re-collapse at once, and dc_retry_cnt resets on each success so it
		 * never latches -> livelock.  sm5440_fg() ensures chip->fg is cached; a
		 * failed read falls back to the fixed engage current.
		 */
		sm5440_fg(chip);
		if (chip->fg && !power_supply_get_property(chip->fg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pv))
			cell_mv = pv.intval / 1000;
		reengage_ci = cell_mv > 0 ? sm5440_ci_gl_cap(cell_mv)
					  : SM5440_AUTO_ENGAGE_CI_GL;

		ret = sm5440_dc_start(chip, reengage_ci, false);
		if (!ret) {
			chip->dc_retry_cnt = 0;
			chip->auto_engaged = true;
			chip->dc_recover_armed = false;
			dev_info(chip->dev,
				 "reengage: RECOVERED -- PPS re-established, pump re-engaged (ci_gl=%u mA)\n",
				 reengage_ci);
			mutex_unlock(&chip->engage_lock);
			return;
		}
		dev_warn(chip->dev, "reengage: contract back but dc_start failed (%d)\n", ret);
	} else {
		dev_warn(chip->dev, "reengage: PPS did not return within %d ms (source deaf)\n",
			 SM5440_REENGAGE_POLL_TRIES * SM5440_REENGAGE_POLL_MS);
	}

	/* Strike: deaf through the whole window, or contract back but dc_start failed. */
	if (++chip->dc_retry_cnt >= SM5440_AUTO_RETRY_MAX) {
		chip->dc_err_latched = true;
		chip->dc_recover_armed = false;
		dev_warn(chip->dev,
			 "reengage: %d consecutive failures -- latching off until charger replug\n",
			 chip->dc_retry_cnt);
	} else {
		dev_warn(chip->dev, "reengage: retry %d/%d -- re-arming in %d ms\n",
			 chip->dc_retry_cnt, SM5440_AUTO_RETRY_MAX, SM5440_REENGAGE_GAP_MS);
		mod_delayed_work(system_long_wq, &chip->dc_reengage_work,
				 msecs_to_jiffies(SM5440_REENGAGE_GAP_MS));
	}
	mutex_unlock(&chip->engage_lock);
}

/*
 * Engine test trigger: "<mA>" starts the CC/CV engine at that FIXED input-current
 * target (e.g. 3000 then 4500, with ta.v_max at the full 10500 mV ceiling so the
 * engine settles at ci_gl); "0" stops + restores.  This is the conservative path:
 * a ci_gl chosen below the source's deliverable ceiling settles cleanly with no
 * probe.  Per-execution gated; never auto-starts.
 */
static ssize_t dc_test_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sm5440 *chip = dev_get_drvdata(dev);
	u32 target_ibus;
	int ret;

	if (kstrtou32(buf, 0, &target_ibus))
		return -EINVAL;

	if (target_ibus == 0) {		/* stop + unconditional restore */
		sm5440_dc_stop(chip);
		return count;
	}

	/*
	 * Lower bound is SIOP_LEV2 (1700), not ta_min_current: below it the
	 * device-own set_charging_config de-rates the switching frequency
	 * (freq_siop[]), which this port does not model -- so keep test targets in
	 * the single-450 kHz range (the documented steps are 3000 / 4500 anyway).
	 */
	if (target_ibus <= SM5440_SIOP_LEV2 || target_ibus > 5000)
		return -EINVAL;

	mutex_lock(&chip->engage_lock);
	/* mutually exclusive with the Inc-1 pump_test manual engage (shared pump/FG). */
	ret = (chip->dc_running || chip->pump_engaged) ? -EBUSY :
	      sm5440_dc_start(chip, target_ibus, false);
	mutex_unlock(&chip->engage_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(dc_test);

/*
 * Adaptive-probe trigger (Inc-3b comp-2): "<ci_gl-mA>" starts the engine with a
 * HIGH input-current goal AND the adaptive ta.v_max probe-up -- the supervisor
 * (probe_work) walks the PPS request ceiling toward the source's deliverable
 * maximum while the collapse-guard prevents an over-draw VBUSUVLO cliff; "0" stops
 * + restores (shared with dc_test).  ci_gl must sit ABOVE what the source can
 * deliver so the engine stays unsatisfied and keeps pushing ta.v toward ta.v_max
 * (the probe's lever); below ~3300 it would simply regulate to ci_gl -- use
 * dc_test for a fixed target.  Capped at the step-0 input (4500 mA).  Per-execution
 * gated; never auto-starts.
 */
static ssize_t dc_probe_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sm5440 *chip = dev_get_drvdata(dev);
	u32 ci_gl;
	int ret;

	if (kstrtou32(buf, 0, &ci_gl))
		return -EINVAL;

	if (ci_gl == 0) {		/* stop + unconditional restore */
		sm5440_dc_stop(chip);
		return count;
	}

	if (ci_gl < 3300 || ci_gl > 4500)
		return -EINVAL;

	mutex_lock(&chip->engage_lock);
	/* mutually exclusive with pump_test + a fixed dc_test (shared pump/FG). */
	ret = (chip->dc_running || chip->pump_engaged) ? -EBUSY :
	      sm5440_dc_start(chip, ci_gl, true);
	mutex_unlock(&chip->engage_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_WO(dc_probe);

/*
 * TEST-ONLY (Build-2b validation): "1" during a live auto-engage run arms a forced
 * over-request.  dc_monitor then HOLDS the request at SM5440_HRST_INJECT_MA each
 * tick (bypassing the ci_gl ratchet) until the engine climbs past the source's
 * PS_RDY boundary and tcpm SENDS a real Hard Reset (the sess-233 mechanism) -- so
 * the Build-2b recovery path can be exercised on demand instead of waiting for a
 * stochastic field HRST.  Inert unless written; one-shot (the teardown clears it
 * once the HRST fires).  REMOVE this knob (and SM5440_HRST_INJECT_MA + the inject
 * consumption in dc_monitor) before any production or upstream build.
 */
static ssize_t dc_inject_hrst_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sm5440 *chip = dev_get_drvdata(dev);
	u32 v;

	if (kstrtou32(buf, 0, &v) || v != 1)
		return -EINVAL;
	if (!READ_ONCE(chip->dc_running))
		return -ENODEV;		/* only meaningful during a live engine run */
	WRITE_ONCE(chip->dc_inject_hrst, true);
	dev_warn(chip->dev,
		 "TEST: HRST injection armed -- next dc_monitor tick over-requests %u mA\n",
		 SM5440_HRST_INJECT_MA);
	return count;
}
static DEVICE_ATTR_WO(dc_inject_hrst);

static int sm5440_get_property(struct power_supply *psy,
			       enum power_supply_property psp,
			       union power_supply_propval *val)
{
	struct sm5440 *chip = power_supply_get_drvdata(psy);
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SM5440";
		return 0;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Silicon Mitus";
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chip->regmap, SM5440_REG_STATUS3, &reg);
		if (ret)
			return ret;
		val->intval = !!(reg & SM5440_STATUS3_VBUS_POK);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return sm5440_get_vbus_uv(chip, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return sm5440_get_ibus_ua(chip, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return sm5440_get_dietemp_dc(chip, &val->intval);
	default:
		return -EINVAL;
	}
}

static enum power_supply_property sm5440_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,	/* VBUS input voltage */
	POWER_SUPPLY_PROP_CURRENT_NOW,	/* IBUS input current */
	POWER_SUPPLY_PROP_TEMP,		/* die temperature */
};

static const struct power_supply_desc sm5440_psy_desc = {
	.name		= "sm5440-charge-pump",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= sm5440_props,
	.num_properties	= ARRAY_SIZE(sm5440_props),
	.get_property	= sm5440_get_property,
};

static const struct regmap_config sm5440_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SM5440_REG_DEVICEID,
	/*
	 * INT registers (0x00-0x03) are clear-on-read and the STATUS / ADC
	 * registers are live, so never cache: every read hits the hardware.
	 */
	.cache_type	= REGCACHE_NONE,
};

static int sm5440_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };
	struct sm5440 *chip;
	unsigned int devid = 0;
	int ret, i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	i2c_set_clientdata(client, chip);
	mutex_init(&chip->engage_lock);
	spin_lock_init(&chip->pps_lock);
	INIT_DELAYED_WORK(&chip->pump_monitor, sm5440_pump_monitor_work);
	INIT_DELAYED_WORK(&chip->ocp_check_work, sm5440_ocp_check_work);
	INIT_DELAYED_WORK(&chip->dc_monitor, sm5440_dc_monitor_work);
	INIT_DELAYED_WORK(&chip->dc_reengage_work, sm5440_dc_reengage_work);	/* Build-2b */
	INIT_DELAYED_WORK(&chip->probe_work, sm5440_dc_probe_work);
	INIT_DELAYED_WORK(&chip->auto_engage_work, sm5440_auto_engage_work);
	INIT_DELAYED_WORK(&chip->pps_keepalive, sm5440_pps_keepalive_work);

	chip->regmap = devm_regmap_init_i2c(client, &sm5440_regmap_config);
	if (IS_ERR(chip->regmap))
		return dev_err_probe(dev, PTR_ERR(chip->regmap),
				     "failed to init regmap\n");

	/*
	 * DEVICEID gate (device-own sm5440_hw_init_config): the low nibble
	 * must read 1, retried up to 3 times, else the chip is absent.
	 */
	for (i = 0; i < 3; i++) {
		ret = regmap_read(chip->regmap, SM5440_REG_DEVICEID, &devid);
		if (!ret && (devid & SM5440_DEVICEID_MASK) == SM5440_DEVICEID_PRESENT)
			break;
		usleep_range(1000, 2000);
	}
	if (i == 3)
		return dev_err_probe(dev, -ENODEV,
				     "SM5440 not found (DEVICEID=0x%02x)\n", devid);

	chip->rev_id = SM5440_DEVICEID_REV(devid);

	/*
	 * Acquire tcpm's source power-supply -- the PPS contract this pump steps.
	 * Defer until the SM5714 tcpm shim has registered it (the pump's role is to
	 * drive this contract; without it there is nothing to do).  The ref is held
	 * for our lifetime and put in remove; EPROBE_DEFER re-probes when the shim
	 * binds.  DEV-LOOP: unload the pump BEFORE the shim, or a step races a freed
	 * tcpm_port -- get_by_name pins use_cnt, so the shim's unregister WARN_ON's
	 * but the psy stays half-alive; the WARN in dmesg is the canary.
	 */
	chip->tcpm_psy = power_supply_get_by_name(SM5440_TCPM_SOURCE_PSY);
	if (!chip->tcpm_psy)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "tcpm source-psy '%s' not ready\n",
				     SM5440_TCPM_SOURCE_PSY);

	/*
	 * Enable the on-chip ADC for telemetry only.  We intentionally skip
	 * the vendor's full init (protection thresholds / switching freq /
	 * op-mode) - that belongs to the charge-control path.  op-mode (CNTL5)
	 * is left untouched at its power-on CHG_OFF default, so the pump never
	 * switches.
	 */
	ret = regmap_write(chip->regmap, SM5440_REG_ADCCNTL2,
			   SM5440_ADCCNTL2_CHANNELS);
	if (ret)
		return dev_err_probe(dev, ret, "failed to select ADC channels\n");

	ret = regmap_write(chip->regmap, SM5440_REG_ADCCNTL1,
			   SM5440_ADCCNTL1_ENABLE);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable ADC\n");

	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(dev);
	chip->psy = devm_power_supply_register(dev, &sm5440_psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy),
				     "failed to register power supply\n");

	/* Guarded manual-engage bring-up trigger (non-fatal if it fails). */
	if (device_create_file(dev, &dev_attr_pump_test))
		dev_warn(dev, "could not create pump_test sysfs attribute\n");

	/*
	 * Increment-2: create the sm_dc CC/CV engine instance, wire its ops to this
	 * driver, and populate the device-own config (PD path).  Non-fatal: if the
	 * engine fails to come up, telemetry + the Inc-1 pump_test still work; only
	 * the closed-loop dc_test is unavailable.
	 */
	chip->dc = sm_dc_create_pd_instance("SM5440-PD-DC", client);
	if (IS_ERR(chip->dc)) {
		dev_warn(dev, "could not create sm_dc engine (%ld) -- dc_test disabled\n",
			 PTR_ERR(chip->dc));
		chip->dc = NULL;
	} else {
		chip->dc->ops = &sm5440_dc_ops;
		chip->dc->config.ta_min_current = SM5440_TA_MIN_CURRENT;
		chip->dc->config.ta_min_voltage = SM5440_TA_MIN_VOLTAGE;
		chip->dc->config.dc_min_vbat = SM5440_VBAT_MIN;
		chip->dc->config.dc_vbus_ovp_th = SM5440_DC_VBUS_OVP_TH;
		chip->dc->config.r_ttl = SM5440_GTS9U_R_TTL_UOHM;
		chip->dc->config.topoff_current = SM5440_DC_TOPOFF_MA;
		chip->dc->config.need_to_sw_ocp = 1;	/* SM5440 has no HW OCP */
		chip->dc->config.support_pd_remain = 1;	/* engine sustains the contract */
		chip->dc->config.chg_float_voltage = SM5440_DC_CHG_FLOAT_MV;
		chip->dc->config.sec_dc_name = "sm5440-charge-pump";

		if (sm_dc_verify_configuration(chip->dc)) {
			dev_warn(dev, "sm_dc verify failed -- dc_test disabled\n");
			sm_dc_destroy_instance(chip->dc);
			chip->dc = NULL;
		} else {
			sm_dc_set_done_notify(chip->dc, sm5440_dc_done_cb, chip);
			if (device_create_file(dev, &dev_attr_dc_test))
				dev_warn(dev, "could not create dc_test sysfs attribute\n");
			if (device_create_file(dev, &dev_attr_dc_probe))
				dev_warn(dev, "could not create dc_probe sysfs attribute\n");
			if (device_create_file(dev, &dev_attr_dc_inject_hrst))	/* TEST-ONLY (Build-2b) */
				dev_warn(dev, "could not create dc_inject_hrst sysfs attribute\n");
			/*
			 * Auto-engage supervisor (Inc-3b-3): register with the SM5714
			 * buck worker (the arbiter) so the pump engages on a PD-charger
			 * attach with NO sysfs trigger.  Only with the engine up -- the
			 * supervisor drives the closed-loop dc_start path.
			 */
			sm5714_charger_set_dc_notify(sm5440_dc_intent_cb, chip);
			dev_info(dev, "auto-engage supervisor registered\n");
		}
	}

	dev_info(dev, "SM5440 charge-pump telemetry ready (rev 0x%x, DEVICEID 0x%02x)%s\n",
		 chip->rev_id, devid, chip->dc ? " + sm_dc engine" : "");
	return 0;
}

static void sm5440_remove(struct i2c_client *client)
{
	struct sm5440 *chip = i2c_get_clientdata(client);

	/*
	 * Unregister the auto-engage notify FIRST so the buck worker stops kicking the
	 * executor, THEN sync-cancel it below -- otherwise a notify landing after the
	 * cancel would re-queue work against a torn-down device.  Harmless if the cb
	 * was never registered (engine absent).
	 */
	sm5714_charger_set_dc_notify(NULL, NULL);

	device_remove_file(&client->dev, &dev_attr_pump_test);
	if (chip->dc) {
		device_remove_file(&client->dev, &dev_attr_dc_test);
		device_remove_file(&client->dev, &dev_attr_dc_probe);
		device_remove_file(&client->dev, &dev_attr_dc_inject_hrst);	/* TEST-ONLY (Build-2b) */
	}

	/*
	 * Sync-cancel every monitor/backstop WITHOUT engage_lock (the monitors take
	 * it; holding it here would deadlock the sync-cancel).  ocp_check_work,
	 * dc_monitor and probe_work are INIT'd unconditionally in probe, so cancelling
	 * them is safe even when the engine never came up.  Then one locked region
	 * tears down whichever path was live (mutually exclusive); the engine last.
	 */
	cancel_delayed_work_sync(&chip->auto_engage_work);
	cancel_delayed_work_sync(&chip->probe_work);
	cancel_delayed_work_sync(&chip->dc_monitor);
	cancel_delayed_work_sync(&chip->dc_reengage_work);	/* Build-2b: after dc_monitor (its armer) */
	cancel_delayed_work_sync(&chip->ocp_check_work);
	cancel_delayed_work_sync(&chip->pump_monitor);

	mutex_lock(&chip->engage_lock);
	if (chip->dc_running)
		sm5440_dc_teardown(chip);
	if (chip->pump_engaged)
		sm5440_pump_disengage_locked(chip);
	mutex_unlock(&chip->engage_lock);

	if (chip->dc) {
		sm_dc_destroy_instance(chip->dc);
		chip->dc = NULL;
	}

	if (chip->fg)
		power_supply_put(chip->fg);	/* the persistent FG handle */

	/*
	 * PPS bridge teardown: stop the keepalive (it may be mid-step, blocking up to
	 * ~10 s) then drop our tcpm source-psy ref.  pps_active is cleared FIRST so a
	 * self-requeue cannot survive the sync-cancel.  The locked teardown above ran
	 * with pps_active still set, so its ramp steps did not re-activate PPS.  (Per
	 * the get_by_name lifetime: this runs only when the pump is unloaded before
	 * the shim, so tcpm_psy is still backed by a live port here.)
	 */
	WRITE_ONCE(chip->pps_active, false);
	cancel_delayed_work_sync(&chip->pps_keepalive);
	if (chip->tcpm_psy)
		power_supply_put(chip->tcpm_psy);
}

static const struct i2c_device_id sm5440_i2c_id[] = {
	{ "sm5440" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5440_i2c_id);

static const struct of_device_id sm5440_of_match[] = {
	{ .compatible = "siliconmitus,sm5440" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5440_of_match);

static struct i2c_driver sm5440_driver = {
	.driver = {
		.name		= "sm5440-charge-pump",
		.of_match_table	= sm5440_of_match,
	},
	.probe		= sm5440_probe,
	.remove		= sm5440_remove,
	.id_table	= sm5440_i2c_id,
};
module_i2c_driver(sm5440_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5440 charge-pump telemetry driver");
MODULE_AUTHOR("omar fityan <me@omarfityan.com>");
MODULE_LICENSE("GPL");
