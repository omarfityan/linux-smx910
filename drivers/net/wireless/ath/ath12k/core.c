// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/export.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/remoteproc.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include "ahb.h"
#include "core.h"
#include "dp_tx.h"
#include "dp_rx.h"
#include "debug.h"
#include "debugfs.h"
#include "fw.h"
#include "hif.h"
#include "pci.h"
#include "wow.h"
#include "dp_cmn.h"
#include "peer.h"

unsigned int ath12k_debug_mask;
module_param_named(debug_mask, ath12k_debug_mask, uint, 0644);
MODULE_PARM_DESC(debug_mask, "Debugging mask");
EXPORT_SYMBOL(ath12k_debug_mask);

bool ath12k_ftm_mode;
module_param_named(ftm_mode, ath12k_ftm_mode, bool, 0444);
MODULE_PARM_DESC(ftm_mode, "Boots up in factory test mode");
EXPORT_SYMBOL(ath12k_ftm_mode);

/*
 * Regulatory country for platforms that supply none.
 *
 * ath12k is a self-managed regulatory wiphy: with no country it stays at WORLD,
 * where every 5 GHz channel is NO_IR and the firmware returns no 5 GHz scan
 * results at all, so the band is simply absent. It has three possible sources:
 *
 *   - an SMBIOS/DMI record, read just below. This is what x86 and UEFI/ACPI
 *     machines use, and why the problem never appears there;
 *   - 802.11d beacon learning, which only runs while NOT associated and is
 *     stopped on association -- on a device that associates a few seconds after
 *     boot it never gets a usable window;
 *   - a user hint via cfg80211, which ath12k_reg_notifier() discards unless
 *     CONFIG_ATH_REG_DYNAMIC_USER_REG_HINTS is enabled (that in turn needs
 *     CFG80211_CERTIFICATION_ONUS and EXPERT), and which also latches
 *     regdom_set_by_user, permanently disabling 802.11d for the boot even when
 *     the push fails.
 *
 * A device-tree platform such as an Android-derived tablet has no DMI at all --
 * dmi_walk() returns -ENXIO -- so the first source is structurally unavailable
 * and the country never gets set by any route.
 *
 * This parameter supplies it directly, feeding the same new_alpha2 the SMBIOS
 * reader feeds, so the country is applied by the driver's existing init-time
 * push in ath12k_mac_hw_register(), before anything associates. No Kconfig
 * option, no user hint, and regdom_set_by_user is untouched, so 802.11d remains
 * available afterwards.
 *
 * SMBIOS wins if present: this only fills a country nothing else provided.
 *
 * Set it to the country the device is actually IN. It selects permitted
 * frequencies, channel widths and transmit powers.
 *
 * It carries a default rather than being left NULL because ath12k is built into
 * this kernel, so the command line is the only source: a boot image that loses
 * the argument drops the radio to WORLD, which costs every 5 GHz channel while
 * the device still associates happily on 2.4 GHz -- so the symptom reads as "the
 * router moved" rather than as a misconfiguration, and nothing logs an error.
 * The default is a fallback against that silent failure, NOT a claim about where
 * this device is; it is deliberately overridable and a command-line value wins.
 * Anyone operating outside the default's jurisdiction should set their own.
 */
static char *ath12k_country = "US";
module_param_named(country, ath12k_country, charp, 0444);
MODULE_PARM_DESC(country, "ISO 3166-1 alpha-2 regulatory country (e.g. US), used when the platform supplies none");

/*
 * WoW flags sent in WMI_WOW_ENABLE_CMDID at suspend.
 *
 * The field exists in wmi_wow_enable_cmd and ath12k has never populated it, so
 * every suspend transmits zero. Zero is not "unspecified": the vendor firmware
 * interface defines the low bit of interest in the NEGATIVE --
 *
 *   WMI_WOW_FLAG_SYSTEM_SUSPEND_WOW = 0x20
 *     "To differentiate system suspend Vs RTPM.  BIT set - System Suspend WOW,
 *      BIT Reset - RTPM (DRV)"
 *
 * -- so sending zero tells the firmware the host is in runtime power management
 * and may return at any instant, rather than that the whole system is going to
 * sleep. A firmware held responsive for RTPM has no reason to enter its deepest
 * state, and the cost of that would be a continuous DC load rather than a
 * duty-cycled one.
 *
 * The downstream stack on this hardware sends 0xE0 at system suspend: 0x20
 * together with the two modulated/forced-DTIM bits, which are a separate
 * mechanism (wake cadence) and are deliberately NOT enabled here.
 *
 * Default 0 preserves the existing behaviour exactly, so carrying this patch
 * changes nothing until the parameter is written. It is 0644 so a control and a
 * treatment arm can be taken in ONE boot from ONE kernel image -- a measurement
 * at this scale cannot survive a cross-build comparison, and without a runtime
 * knob that is the only kind available.
 */
unsigned int ath12k_wow_flags;
module_param_named(wow_flags, ath12k_wow_flags, uint, 0644);
MODULE_PARM_DESC(wow_flags, "WMI_WOW_FLAG_* bitmask sent in WMI_WOW_ENABLE_CMDID (0x20 = SYSTEM_SUSPEND_WOW; default 0 = upstream behaviour)");

/*
 * Wake-cadence parameters sent to the firmware when station power save is enabled.
 *
 * MEASURED PROBLEM.  With power save on, this device is awake for 1.2 % of the AP's
 * beacons -- 0.12/s against 9.65/s measured with power save off on the same boot, the
 * off arm calibrating the counter at 96.5 % of the theoretical rate.  That is a wake
 * roughly every 8 seconds.  The consequences are not subtle: group-addressed traffic is
 * almost entirely missed (6 of 200 multicast frames delivered, and 0 of 30 in an earlier
 * window), so inbound ARP never resolves and NOTHING ON THE LAN CAN OPEN A CONNECTION TO
 * THE DEVICE until it happens to transmit first.  Unicast latency to an already-resolved
 * neighbour runs 406 ms mean against 12 ms with power save off.
 *
 * WHY IT HAPPENS.  ath12k turns station power save on and then never tells the firmware
 * anything about how to sleep.  WMI_VDEV_PARAM_LISTEN_INTERVAL, WMI_VDEV_PARAM_DTIM_POLICY
 * and WMI_STA_DTIM_PS_METHOD_CMDID are all enumerated in wmi.h and called from nowhere.
 * The firmware is left to choose its own cadence, and it chooses a deep one.
 *
 * (An earlier note here also blamed ath12k_mac_op_config() being a stub while the driver
 * advertises IEEE80211_HW_SUPPORTS_DYNAMIC_PS.  That is a red herring: ieee80211_recalc_ps()
 * returns early for any SUPPORTS_DYNAMIC_PS driver, BEFORE the only assignments to
 * hw.conf.dynamic_ps_timeout and hw.conf.ps_dtim_period, and cfg.c fires .config before
 * recalc runs -- so the callback always arrives with IEEE80211_CONF_PS clear and the timeout
 * zero.  There is nothing in it to discard.  rtw88 and rtw89 were fixed in 2023 by DELETING
 * their CONF_CHANGE_PS handlers, and iwlwifi stubs .config in both op_modes.)
 *
 * WHAT THE VENDOR DOES ON THIS EXACT SILICON.  Its WLAN configuration carries
 * gEnableModulatedDTIM=5 together with enable_mod_dtim_on_system_suspend=1, and the
 * vendor's own documentation of that second value reads:
 *
 *     "1 -> Enable modulated DTIM only for System suspend wow.
 *           For RTPM wow, the device will stay in DTIM 1 (non-modulated DTIM)"
 *
 * so the 5-beacon cadence is a SUSPEND-ONLY policy; while awake the vendor holds DTIM 1.
 * It sets the awake state explicitly, sending the listen interval and then
 * WMI_VDEV_PARAM_DTIM_POLICY = NORMAL_DTIM together (pmo_core_set_vdev_resume_dtim()).
 *
 * 🔴🔴 THE PARAGRAPH THAT STOOD HERE WAS WRONG, AND IT POINTED THE WORK AWAY FROM THE FIX.
 * It argued that advertising listen interval 5 (ATH12K_MAX_HW_LISTEN_INTERVAL) "cannot
 * account for the observed cadence: 5 beacons is 0.5 s and the station sleeps ~8 s".
 * Both halves fail.
 *
 * The ~8 s came from `beacon rx` in `iw station dump`, which is
 * sdata->deflink.u.mgd.count_beacon_signal -- a count of beacons MAC80211 PARSED IN THE
 * HOST, incremented in ieee80211_rx_mgmt_beacon().  It counts what the firmware chose to
 * forward, not what the radio woke for, and the power-save-off calibration cannot separate
 * the two because firmware-side beacon forwarding is exactly what changes between the arms.
 * Measured against it: a station completing 40 SSH round trips over 200 s still read 3.8 %.
 *
 * The retrieval cadence the pooled unicast RTT actually shows is ~512 ms = 5 x 102.4 ms --
 * every sample taken with power save on falls in [8, 473] ms, none above 512 -- which is
 * the listen interval, exactly the explanation the arithmetic was used to dismiss.
 *
 * ⇒ The listen interval is BACK ON THE TABLE, and ath10k has forced it to 1 for stations
 * since forever, in ath10k_peer_assoc_h_listen_intval(), whose comment describes this
 * defect: "unstable STA powersave when listen interval is set too high (e.g. 5) ... firmware
 * would seldom wake up to pull buffered frames.  Often pinging the device from AP would
 * simply fail."  ath11k and ath12k both dropped that workaround.
 *
 *   bit 0  send WMI_VDEV_PARAM_DTIM_POLICY = WMI_BEACON_DTIM_POLICY_NORMAL
 *   bit 1  send WMI_VDEV_PARAM_LISTEN_INTERVAL = 1
 *   bit 2  send WMI_STA_PS_PARAM_RX_WAKE_POLICY = WAKE
 *   bit 3  send WMI_STA_PS_PARAM_TX_WAKE_THRESHOLD = ALWAYS
 *   bit 4  send WMI_STA_PS_PARAM_PSPOLL_COUNT = NO_MAX
 *   bit 5  send WMI_STA_PS_PARAM_SPEC_WAKE_INTERVAL = 0
 *
 * 🔴 MEASURED: bits 0 and 1 are REFUTED. Singly and together they moved the wake rate
 * from a 3.5 % control to 3.2 / 2.7 / 4.0 %, against a 96.0 % power-save-off calibration
 * taken in the same run -- noise. Both commands were ACCEPTED by the firmware (each logged
 * its send and none warned), so that is a real negative and not an untaken test. Do not
 * re-test them.
 *
 * Bits 2-5 are the four parameters wma_set_force_sleep() sends on every power-save enable
 * and ath12k never has; three of their ids were already in this driver's enum and simply
 * never transmitted. RX_WAKE_POLICY is the one whose firmware documentation names the
 * measured defect -- see enum wmi_sta_ps_rx_wake_policy in wmi.h. This device's values are
 * WAKE / ALWAYS / NO_MAX, because gMaxPsPoll is absent from its vendor configuration and
 * defaults to 0, selecting that branch of the vendor's own code.
 *
 * Default 0 sends nothing and preserves current behaviour exactly, so carrying this patch
 * changes the driver only once the parameter is written.  It is 0644 and the parameters are
 * applied from ath12k_mac_vif_setup_ps(), which mac80211 re-runs on every BSS_CHANGED_PS --
 * so `iw dev <dev> set power_save off; iw dev <dev> set power_save on` re-applies them with
 * NO re-association.  Control and treatment therefore run in ONE boot from ONE image, which
 * is the only kind of comparison a measurement at this scale survives.
 */
unsigned int ath12k_sta_wake_cadence;
module_param_named(sta_wake_cadence, ath12k_sta_wake_cadence, uint, 0644);
MODULE_PARM_DESC(sta_wake_cadence, "wake-cadence params sent on STA power-save enable: bit0 DTIM_POLICY NORMAL, bit1 LISTEN_INTERVAL 1, bit2 RX_WAKE_POLICY WAKE, bit3 TX_WAKE_THRESHOLD ALWAYS, bit4 PSPOLL_COUNT NO_MAX, bit5 SPEC_WAKE_INTERVAL 0 (default 0 = send nothing, upstream behaviour)");

/* The DTIM policy sent at ASSOCIATION, which is a different lever from the bitmask above and
 * is the one upstream chose.
 *
 * WHAT IT FIXES.  WCN7850's firmware follows the listen interval and ignores the AP's DTIM
 * period, so the station wakes on its own schedule and collects almost no group-addressed
 * traffic.  Upstream states it plainly in af50baccaa5f ("wifi: ath12k: Set DTIM policy to
 * stick mode for station interface", tested on WCN7850 hw2.0 PCI):
 *
 *     "Currently, the station always follows the listen interval regardless of the DTIM
 *      value.  The DTIM function does not work as expected.  The default value of the listen
 *      interval is 5 so that the STA wakes up every 500ms when power save is on."
 *
 * Its fix is WMI_VDEV_PARAM_DTIM_POLICY = STICK, "per the firmware team's request", so the
 * station follows the AP's DTIM "rather than listen interval, which is set in the peer assoc
 * command".  ath11k has carried the identical fix since 55e18e5a76ab (2021).
 *
 * 🔴 WHY THE CALL SITE MATTERS, AND WHY bit 0 ABOVE IS NOT THIS.  bit 0 sends the same vdev
 * parameter with the value NORMAL, from ath12k_mac_vif_setup_ps() -- a different value, at a
 * different moment.  Upstream sends STICK from ath12k_bss_assoc(), inside the association
 * exchange, because the listen interval it has to override is latched from peer-assoc.  A
 * send from the power-save path is accepted by the firmware and may change nothing, which is
 * indistinguishable from the parameter having no effect.  bit 0's refutation therefore says
 * nothing about this knob.
 *
 * WHY IT CARRIES A VALUE AND NOT A BOOLEAN.  This firmware branch is c5-00284; upstream
 * validated STICK on c7-00108.  A null result has to be separable from "this build ignores
 * the policy", so all four values stay reachable from one image -- including NORMAL, which
 * re-tests bit 0's value from the correct call site and so separates "wrong value" from
 * "wrong call site" without a second build.
 *
 *   0  send nothing (default; v7.0.6 behaviour exactly)
 *   1  WMI_BEACON_DTIM_POLICY_IGNORE
 *   2  WMI_BEACON_DTIM_POLICY_NORMAL   <- the value bit 0 sends, from the other call site
 *   3  WMI_BEACON_DTIM_POLICY_STICK    <- upstream's fix
 *   4  WMI_BEACON_DTIM_POLICY_AUTO
 *
 * It is 0644, but unlike the bitmask above it is read at association, so re-applying it needs
 * a RE-ASSOCIATION rather than a power-save toggle.  Control and treatment still share one
 * boot and one image: write the value, then bounce the connection.
 */
unsigned int ath12k_sta_dtim_policy;
module_param_named(sta_dtim_policy, ath12k_sta_dtim_policy, uint, 0644);
MODULE_PARM_DESC(sta_dtim_policy, "WMI_VDEV_PARAM_DTIM_POLICY sent at STA association: 0 send nothing (default, upstream v7.0.6 behaviour), 1 IGNORE, 2 NORMAL, 3 STICK (upstream af50baccaa5f), 4 AUTO");

/* The LISTEN INTERVAL CARRIED IN WMI_PEER_ASSOC_CMDID -- the quantity every parameter tried
 * so far was a proxy for, and the one thing in this area that had never been touched.
 *
 * WHAT GOVERNS THE FIRMWARE'S WAKE CADENCE.  Upstream af50baccaa5f says the station "always
 * follows the listen interval regardless of the DTIM value", and names where that listen
 * interval comes from: "the peer assoc command".  The chain is entirely inside this tree:
 *
 *     net/mac80211/main.c:1520-1523   hw.max_listen_interval defaults to 5 only if the driver
 *                                     left it 0; conf.listen_interval = max_listen_interval
 *     ath12k/mac.c:14837              hw->max_listen_interval = ATH12K_MAX_HW_LISTEN_INTERVAL
 *     ath12k/wmi.h:85                 #define ATH12K_MAX_HW_LISTEN_INTERVAL 5
 *     ath12k/mac.c (h_basic)          arg->peer_listen_intval = hw->conf.listen_interval
 *     ath12k/wmi.c:2221               cmd->peer_listen_intval  -> WMI_PEER_ASSOC_CMDID
 *
 * so this device tells its own firmware to wake every 5 beacons -- 512 ms at a 102.4 ms beacon
 * interval -- and the pooled unicast RTT measured on it, every sample in [8, 473] ms with none
 * above 512, is what that looks like sampled by a 1 Hz ping.
 *
 * 🔴 THIS IS NOT WMI_VDEV_PARAM_LISTEN_INTERVAL, WHICH WAS REFUTED.  That vdev parameter was
 * sent with the value 1, was ACCEPTED by the firmware, and moved nothing.  It is a DIFFERENT
 * CARRIER: the peer-assoc value is latched for the peer at association and is what the
 * firmware's power-save scheduler actually follows, which is precisely why upstream's commit
 * message names the peer assoc command and not the vdev parameter.  A refutation of one says
 * nothing about the other.
 *
 * WHY THIS OVERRIDES THE PEER-ASSOC VALUE AND NOT ATH12K_MAX_HW_LISTEN_INTERVAL.  Lowering the
 * #define would move the OVER-THE-AIR ADVERTISED listen interval as well, because
 * conf.listen_interval is also what mac80211 puts in the association request
 * (net/mac80211/mlme.c:2318-2334).  That is the wrong half twice over: the advertised value
 * tells the AP how long to buffer UNICAST for us, whereas the symptom is loss of
 * GROUP-addressed traffic, which an AP emits after DTIM beacons whatever our listen interval
 * says; and changing it puts the AP's buffering behaviour inside a single-variable test.
 *
 * ath10k, which has carried this workaround for stations since forever, does exactly what this
 * does -- ath10k_peer_assoc_h_listen_intval() returns 1 for NL80211_IFTYPE_STATION while
 * ath10k/mac.c:10124 leaves hw->max_listen_interval at 5.  Its comment describes the defect
 * measured here in the vendor's own words:
 *
 *     "Some firmware revisions have unstable STA powersave when listen interval is set too
 *      high (e.g. 5).  The symptoms are firmware doesn't generate NullFunc frames properly
 *      even if buffered frames have been indicated in Beacon TIM.  Firmware would seldom wake
 *      up to pull buffered frames.  Often pinging the device from AP would simply fail."
 *
 * ath11k and ath12k both dropped that workaround; ath12k_peer_assoc_h_basic() still carries
 * the open question as a comment ("TODO: STA WAR in ath10k for listen interval required?").
 * The vendor's own configuration agrees with ath10k: qcacld-3.0's CFG_LISTEN_INTERVAL has
 * range 0-65535 and default 1, not 5.
 *
 * ⚠️ WHAT THIS CONTROL CANNOT WITNESS.  Peer assoc returns no per-field accept or reject, so
 * the log below proves the value was SENT, never that the firmware honoured it -- a weaker
 * control than the DTIM knob's, and it is recorded as weaker rather than read as stronger.
 * Note too that the firmware at listen interval 5 may MALFUNCTION rather than degrade
 * linearly (the ath10k comment says so), so a null result is not evidence that the effect is
 * merely small, and a positive need not be proportionate.
 *
 *   0        leave arg->peer_listen_intval at hw->conf.listen_interval -- upstream behaviour
 *            exactly, which on this driver is ATH12K_MAX_HW_LISTEN_INTERVAL = 5 (default)
 *   1        wake every beacon; ath10k's value and the vendor's CFG_LISTEN_INTERVAL default
 *   2-65535  any other value, so a null at 1 can be separated from an ignored field
 *
 * It is 0644 and is read when the peer assoc command is built, so re-applying it needs a
 * RE-ASSOCIATION -- as with sta_dtim_policy, write the value and then bounce the connection.
 * Control and treatment still share one boot and one image.
 */
unsigned int ath12k_sta_listen_interval;
module_param_named(sta_listen_interval, ath12k_sta_listen_interval, uint, 0644);
MODULE_PARM_DESC(sta_listen_interval, "override peer_listen_intval in WMI_PEER_ASSOC_CMDID for a station interface, in beacon intervals: 0 use mac80211's value (default, upstream behaviour = ATH12K_MAX_HW_LISTEN_INTERVAL 5), 1 wake every beacon (the ath10k workaround and the vendor default)");

/* protected with ath12k_hw_group_mutex */
static struct list_head ath12k_hw_group_list = LIST_HEAD_INIT(ath12k_hw_group_list);

static DEFINE_MUTEX(ath12k_hw_group_mutex);

static const struct
ath12k_mem_profile_based_param ath12k_mem_profile_based_param[] = {
[ATH12K_QMI_MEMORY_MODE_DEFAULT] = {
		.num_vdevs = 17,
		.max_client_single = 512,
		.max_client_dbs = 128,
		.max_client_dbs_sbs = 128,
		.dp_params = {
			.tx_comp_ring_size = 32768,
			.rxdma_monitor_buf_ring_size = 4096,
			.rxdma_monitor_dst_ring_size = 8092,
			.num_pool_tx_desc = 32768,
			.rx_desc_count = 12288,
		},
	},
[ATH12K_QMI_MEMORY_MODE_LOW_512_M] = {
		.num_vdevs = 9,
		.max_client_single = 128,
		.max_client_dbs = 64,
		.max_client_dbs_sbs = 64,
		.dp_params = {
			.tx_comp_ring_size = 16384,
			.rxdma_monitor_buf_ring_size = 256,
			.rxdma_monitor_dst_ring_size = 512,
			.num_pool_tx_desc = 16384,
			.rx_desc_count = 6144,
		},
	},
};

static int ath12k_core_rfkill_config(struct ath12k_base *ab)
{
	struct ath12k *ar;
	int ret = 0, i;

	if (!(ab->target_caps.sys_cap_info & WMI_SYS_CAP_INFO_RFKILL))
		return 0;

	if (ath12k_acpi_get_disable_rfkill(ab))
		return 0;

	for (i = 0; i < ab->num_radios; i++) {
		ar = ab->pdevs[i].ar;

		ret = ath12k_mac_rfkill_config(ar);
		if (ret && ret != -EOPNOTSUPP) {
			ath12k_warn(ab, "failed to configure rfkill: %d", ret);
			return ret;
		}
	}

	return ret;
}

/* Check if we need to continue with suspend/resume operation.
 * Return:
 *	a negative value: error happens and don't continue.
 *	0:  no error but don't continue.
 *	positive value: no error and do continue.
 */
static int ath12k_core_continue_suspend_resume(struct ath12k_base *ab)
{
	struct ath12k *ar;

	if (!ab->hw_params->supports_suspend)
		return -EOPNOTSUPP;

	/* so far single_pdev_only chips have supports_suspend as true
	 * so pass 0 as a dummy pdev_id here.
	 */
	ar = ab->pdevs[0].ar;
	if (!ar || !ar->ah || ar->ah->state != ATH12K_HW_STATE_OFF)
		return 0;

	return 1;
}

int ath12k_core_suspend(struct ath12k_base *ab)
{
	struct ath12k *ar;
	int ret, i;

	ret = ath12k_core_continue_suspend_resume(ab);
	if (ret <= 0)
		return ret;

	for (i = 0; i < ab->num_radios; i++) {
		ar = ab->pdevs[i].ar;
		if (!ar)
			continue;

		wiphy_lock(ath12k_ar_to_hw(ar)->wiphy);

		ret = ath12k_mac_wait_tx_complete(ar);
		if (ret) {
			wiphy_unlock(ath12k_ar_to_hw(ar)->wiphy);
			ath12k_warn(ab, "failed to wait tx complete: %d\n", ret);
			return ret;
		}

		wiphy_unlock(ath12k_ar_to_hw(ar)->wiphy);
	}

	/* PM framework skips suspend_late/resume_early callbacks
	 * if other devices report errors in their suspend callbacks.
	 * However ath12k_core_resume() would still be called because
	 * here we return success thus kernel put us on dpm_suspended_list.
	 * Since we won't go through a power down/up cycle, there is
	 * no chance to call complete(&ab->restart_completed) in
	 * ath12k_core_restart(), making ath12k_core_resume() timeout.
	 * So call it here to avoid this issue. This also works in case
	 * no error happens thus suspend_late/resume_early get called,
	 * because it will be reinitialized in ath12k_core_resume_early().
	 */
	complete(&ab->restart_completed);

	return 0;
}
EXPORT_SYMBOL(ath12k_core_suspend);

int ath12k_core_suspend_late(struct ath12k_base *ab)
{
	int ret;

	ret = ath12k_core_continue_suspend_resume(ab);
	if (ret <= 0)
		return ret;

	ath12k_acpi_stop(ab);

	ath12k_hif_irq_disable(ab);
	ath12k_hif_ce_irq_disable(ab);

	ath12k_hif_power_down(ab, true);

	return 0;
}
EXPORT_SYMBOL(ath12k_core_suspend_late);

int ath12k_core_resume_early(struct ath12k_base *ab)
{
	int ret;

	ret = ath12k_core_continue_suspend_resume(ab);
	if (ret <= 0)
		return ret;

	reinit_completion(&ab->restart_completed);
	ret = ath12k_hif_power_up(ab);
	if (ret)
		ath12k_warn(ab, "failed to power up hif during resume: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL(ath12k_core_resume_early);

/* The firmware is powered down across a system suspend and comes back at its
 * own default regulatory domain. The configured country is pushed to it in
 * exactly two places, and a resume reaches NEITHER:
 *
 *   - ath12k_mac_hw_register(), which runs once when the radio is registered
 *     with mac80211. A resume does not re-register it;
 *   - ath12k_mac_op_reconfig_complete(), which mac80211 calls only after an
 *     ieee80211_restart_hw(), and which ath12k_core_restart() only issues when
 *     ab->is_reset -- that is, on firmware-crash recovery. A resume is not one.
 *
 * So nothing re-sends it and the radio silently reverts to WORLD: every 5 GHz
 * band becomes passive-scan only and the band effectively disappears, while the
 * interface still associates and routes perfectly well on 2.4 GHz. That is why
 * it presents as a network problem rather than a driver one.
 *
 * Re-push the LATCHED configured country, not ar->alpha2 -- the latter is live
 * state that ath12k_update_11d() overwrites with whatever 802.11d last learned.
 */
static void ath12k_core_reapply_country(struct ath12k_base *ab)
{
	struct wmi_set_current_country_arg arg = {};
	const char *src;
	struct ath12k *ar;
	int i, ret;

	if (!ab->hw_params->current_cc_support)
		return;

	/* Prefer the country 802.11d actually LEARNED over the configured
	 * fallback, and prefer it for a reason that costs real time when it is
	 * ignored.
	 *
	 * cfg_alpha2 is the ath12k.country= default. Its own documentation says
	 * it is "NOT a claim about where this device is" -- it exists only so a
	 * boot that supplies no country does not sit at WORLD and lose 5 GHz.
	 * new_alpha2 is what the SMBIOS record or the 802.11d country event
	 * reported, i.e. where the radio actually is. Once 11d has spoken, the
	 * fallback is strictly worse information.
	 *
	 * Pushing the fallback here does not merely mislabel the regdomain, it
	 * makes the first scan after every resume far slower, because the
	 * regdomain sets the channel list and the scan pays for every channel in
	 * it. Measured on this device, in JO, with the US default:
	 *
	 *   scan 1, US list : 98 channels, 13.61 s  (139 ms/channel)
	 *   scan 2, JO list : 48 channels,  5.72 s  (119 ms/channel)
	 *
	 * US adds the 16 5 GHz DFS channels -- which must be scanned PASSIVELY,
	 * the slowest kind -- and 36 more 6 GHz channels than JO permits. 802.11d
	 * corrects the domain about six seconds in, but a scan already in flight
	 * keeps the channel list it started with, so the correction arrives too
	 * late to help and the user waits out the long sweep on every resume.
	 *
	 * ar->alpha2 is deliberately NOT used: that is per-radio live state.
	 * new_alpha2 is validated rather than trusted -- ath12k_wmi_11d_new_cc_event()
	 * copies whatever the firmware sends, and the SMBIOS worldwide branch
	 * stores a literal "00". Accepting either would push WORLD on resume and
	 * cause exactly the 5 GHz loss this function exists to prevent.
	 */
	spin_lock_bh(&ab->base_lock);
	if (ab->new_alpha2[0] >= 'A' && ab->new_alpha2[0] <= 'Z' &&
	    ab->new_alpha2[1] >= 'A' && ab->new_alpha2[1] <= 'Z') {
		memcpy(&arg.alpha2, ab->new_alpha2, 2);
		src = "learned";
	} else {
		memcpy(&arg.alpha2, ab->cfg_alpha2, 2);
		src = "configured";
	}
	spin_unlock_bh(&ab->base_lock);

	if (!arg.alpha2[0])
		return;

	for (i = 0; i < ab->num_radios; i++) {
		ar = ab->pdevs[i].ar;
		if (!ar)
			continue;

		memcpy(&ar->alpha2, arg.alpha2, 2);
		reinit_completion(&ar->regd_update_completed);

		ret = ath12k_wmi_send_set_current_country_cmd(ar, &arg);
		if (ret)
			ath12k_warn(ab,
				    "pdev %d failed to re-apply country %c%c after resume: %d\n",
				    i, arg.alpha2[0], arg.alpha2[1], ret);
		else
			ath12k_info(ab, "regulatory country %c%c re-applied after resume (%s)\n",
				    arg.alpha2[0], arg.alpha2[1], src);
	}
}

int ath12k_core_resume(struct ath12k_base *ab)
{
	long time_left;
	int ret;

	ret = ath12k_core_continue_suspend_resume(ab);
	if (ret <= 0)
		return ret;

	time_left = wait_for_completion_timeout(&ab->restart_completed,
						ATH12K_RESET_TIMEOUT_HZ);
	if (time_left == 0) {
		ath12k_warn(ab, "timeout while waiting for restart complete");
		return -ETIMEDOUT;
	}

	ath12k_core_reapply_country(ab);

	return 0;
}
EXPORT_SYMBOL(ath12k_core_resume);

static int __ath12k_core_create_board_name(struct ath12k_base *ab, char *name,
					   size_t name_len, bool with_variant,
					   bool bus_type_mode, bool with_default)
{
	/* strlen(',variant=') + strlen(ab->qmi.target.bdf_ext) */
	char variant[9 + ATH12K_QMI_BDF_EXT_STR_LENGTH] = {};

	if (with_variant && ab->qmi.target.bdf_ext[0] != '\0')
		scnprintf(variant, sizeof(variant), ",variant=%s",
			  ab->qmi.target.bdf_ext);

	switch (ab->id.bdf_search) {
	case ATH12K_BDF_SEARCH_BUS_AND_BOARD:
		if (bus_type_mode)
			scnprintf(name, name_len,
				  "bus=%s",
				  ath12k_bus_str(ab->hif.bus));
		else
			scnprintf(name, name_len,
				  "bus=%s,vendor=%04x,device=%04x,subsystem-vendor=%04x,subsystem-device=%04x,qmi-chip-id=%d,qmi-board-id=%d%s",
				  ath12k_bus_str(ab->hif.bus),
				  ab->id.vendor, ab->id.device,
				  ab->id.subsystem_vendor,
				  ab->id.subsystem_device,
				  ab->qmi.target.chip_id,
				  ab->qmi.target.board_id,
				  variant);
		break;
	default:
		scnprintf(name, name_len,
			  "bus=%s,qmi-chip-id=%d,qmi-board-id=%d%s",
			  ath12k_bus_str(ab->hif.bus),
			  ab->qmi.target.chip_id,
			  with_default ?
			  ATH12K_BOARD_ID_DEFAULT : ab->qmi.target.board_id,
			  variant);
		break;
	}

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "boot using board name '%s'\n", name);

	return 0;
}

static int ath12k_core_create_board_name(struct ath12k_base *ab, char *name,
					 size_t name_len)
{
	return __ath12k_core_create_board_name(ab, name, name_len, true, false, false);
}

static int ath12k_core_create_fallback_board_name(struct ath12k_base *ab, char *name,
						  size_t name_len)
{
	return __ath12k_core_create_board_name(ab, name, name_len, false, false, true);
}

static int ath12k_core_create_bus_type_board_name(struct ath12k_base *ab, char *name,
						  size_t name_len)
{
	return __ath12k_core_create_board_name(ab, name, name_len, false, true, true);
}

const struct firmware *ath12k_core_firmware_request(struct ath12k_base *ab,
						    const char *file)
{
	const struct firmware *fw;
	char path[100];
	int ret;

	if (!file)
		return ERR_PTR(-ENOENT);

	ath12k_core_create_firmware_path(ab, file, path, sizeof(path));

	ret = firmware_request_nowarn(&fw, path, ab->dev);
	if (ret)
		return ERR_PTR(ret);

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "boot firmware request %s size %zu\n",
		   path, fw->size);

	return fw;
}

void ath12k_core_free_bdf(struct ath12k_base *ab, struct ath12k_board_data *bd)
{
	if (!IS_ERR(bd->fw))
		release_firmware(bd->fw);

	memset(bd, 0, sizeof(*bd));
}

static int ath12k_core_parse_bd_ie_board(struct ath12k_base *ab,
					 struct ath12k_board_data *bd,
					 const void *buf, size_t buf_len,
					 const char *boardname,
					 int ie_id,
					 int name_id,
					 int data_id)
{
	const struct ath12k_fw_ie *hdr;
	bool name_match_found;
	int ret, board_ie_id;
	size_t board_ie_len;
	const void *board_ie_data;

	name_match_found = false;

	/* go through ATH12K_BD_IE_BOARD_/ATH12K_BD_IE_REGDB_ elements */
	while (buf_len > sizeof(struct ath12k_fw_ie)) {
		hdr = buf;
		board_ie_id = le32_to_cpu(hdr->id);
		board_ie_len = le32_to_cpu(hdr->len);
		board_ie_data = hdr->data;

		buf_len -= sizeof(*hdr);
		buf += sizeof(*hdr);

		if (buf_len < ALIGN(board_ie_len, 4)) {
			ath12k_err(ab, "invalid %s length: %zu < %zu\n",
				   ath12k_bd_ie_type_str(ie_id),
				   buf_len, ALIGN(board_ie_len, 4));
			ret = -EINVAL;
			goto out;
		}

		if (board_ie_id == name_id) {
			ath12k_dbg_dump(ab, ATH12K_DBG_BOOT, "board name", "",
					board_ie_data, board_ie_len);

			if (board_ie_len != strlen(boardname))
				goto next;

			ret = memcmp(board_ie_data, boardname, strlen(boardname));
			if (ret)
				goto next;

			name_match_found = true;
			ath12k_dbg(ab, ATH12K_DBG_BOOT,
				   "boot found match %s for name '%s'",
				   ath12k_bd_ie_type_str(ie_id),
				   boardname);
		} else if (board_ie_id == data_id) {
			if (!name_match_found)
				/* no match found */
				goto next;

			ath12k_dbg(ab, ATH12K_DBG_BOOT,
				   "boot found %s for '%s'",
				   ath12k_bd_ie_type_str(ie_id),
				   boardname);

			bd->data = board_ie_data;
			bd->len = board_ie_len;

			ret = 0;
			goto out;
		} else {
			ath12k_warn(ab, "unknown %s id found: %d\n",
				    ath12k_bd_ie_type_str(ie_id),
				    board_ie_id);
		}
next:
		/* jump over the padding */
		board_ie_len = ALIGN(board_ie_len, 4);

		buf_len -= board_ie_len;
		buf += board_ie_len;
	}

	/* no match found */
	ret = -ENOENT;

out:
	return ret;
}

static int ath12k_core_fetch_board_data_api_n(struct ath12k_base *ab,
					      struct ath12k_board_data *bd,
					      const char *boardname,
					      int ie_id_match,
					      int name_id,
					      int data_id)
{
	size_t len, magic_len;
	const u8 *data;
	char *filename, filepath[100];
	size_t ie_len;
	struct ath12k_fw_ie *hdr;
	int ret, ie_id;

	filename = ATH12K_BOARD_API2_FILE;

	if (!bd->fw)
		bd->fw = ath12k_core_firmware_request(ab, filename);

	if (IS_ERR(bd->fw))
		return PTR_ERR(bd->fw);

	data = bd->fw->data;
	len = bd->fw->size;

	ath12k_core_create_firmware_path(ab, filename,
					 filepath, sizeof(filepath));

	/* magic has extra null byte padded */
	magic_len = strlen(ATH12K_BOARD_MAGIC) + 1;
	if (len < magic_len) {
		ath12k_err(ab, "failed to find magic value in %s, file too short: %zu\n",
			   filepath, len);
		ret = -EINVAL;
		goto err;
	}

	if (memcmp(data, ATH12K_BOARD_MAGIC, magic_len)) {
		ath12k_err(ab, "found invalid board magic\n");
		ret = -EINVAL;
		goto err;
	}

	/* magic is padded to 4 bytes */
	magic_len = ALIGN(magic_len, 4);
	if (len < magic_len) {
		ath12k_err(ab, "failed: %s too small to contain board data, len: %zu\n",
			   filepath, len);
		ret = -EINVAL;
		goto err;
	}

	data += magic_len;
	len -= magic_len;

	while (len > sizeof(struct ath12k_fw_ie)) {
		hdr = (struct ath12k_fw_ie *)data;
		ie_id = le32_to_cpu(hdr->id);
		ie_len = le32_to_cpu(hdr->len);

		len -= sizeof(*hdr);
		data = hdr->data;

		if (len < ALIGN(ie_len, 4)) {
			ath12k_err(ab, "invalid length for board ie_id %d ie_len %zu len %zu\n",
				   ie_id, ie_len, len);
			ret = -EINVAL;
			goto err;
		}

		if (ie_id == ie_id_match) {
			ret = ath12k_core_parse_bd_ie_board(ab, bd, data,
							    ie_len,
							    boardname,
							    ie_id_match,
							    name_id,
							    data_id);
			if (ret == -ENOENT)
				/* no match found, continue */
				goto next;
			else if (ret)
				/* there was an error, bail out */
				goto err;
			/* either found or error, so stop searching */
			goto out;
		}
next:
		/* jump over the padding */
		ie_len = ALIGN(ie_len, 4);

		len -= ie_len;
		data += ie_len;
	}

out:
	if (!bd->data || !bd->len) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "failed to fetch %s for %s from %s\n",
			   ath12k_bd_ie_type_str(ie_id_match),
			   boardname, filepath);
		ret = -ENODATA;
		goto err;
	}

	return 0;

err:
	ath12k_core_free_bdf(ab, bd);
	return ret;
}

int ath12k_core_fetch_board_data_api_1(struct ath12k_base *ab,
				       struct ath12k_board_data *bd,
				       char *filename)
{
	bd->fw = ath12k_core_firmware_request(ab, filename);
	if (IS_ERR(bd->fw))
		return PTR_ERR(bd->fw);

	bd->data = bd->fw->data;
	bd->len = bd->fw->size;

	return 0;
}

#define BOARD_NAME_SIZE 200
int ath12k_core_fetch_bdf(struct ath12k_base *ab, struct ath12k_board_data *bd)
{
	char boardname[BOARD_NAME_SIZE], fallback_boardname[BOARD_NAME_SIZE];
	char *filename, filepath[100];
	int bd_api;
	int ret;

	filename = ATH12K_BOARD_API2_FILE;

	ret = ath12k_core_create_board_name(ab, boardname, sizeof(boardname));
	if (ret) {
		ath12k_err(ab, "failed to create board name: %d", ret);
		return ret;
	}

	bd_api = 2;
	ret = ath12k_core_fetch_board_data_api_n(ab, bd, boardname,
						 ATH12K_BD_IE_BOARD,
						 ATH12K_BD_IE_BOARD_NAME,
						 ATH12K_BD_IE_BOARD_DATA);
	if (!ret)
		goto success;

	ret = ath12k_core_create_fallback_board_name(ab, fallback_boardname,
						     sizeof(fallback_boardname));
	if (ret) {
		ath12k_err(ab, "failed to create fallback board name: %d", ret);
		return ret;
	}

	ret = ath12k_core_fetch_board_data_api_n(ab, bd, fallback_boardname,
						 ATH12K_BD_IE_BOARD,
						 ATH12K_BD_IE_BOARD_NAME,
						 ATH12K_BD_IE_BOARD_DATA);
	if (!ret)
		goto success;

	bd_api = 1;
	ret = ath12k_core_fetch_board_data_api_1(ab, bd, ATH12K_DEFAULT_BOARD_FILE);
	if (ret) {
		ath12k_core_create_firmware_path(ab, filename,
						 filepath, sizeof(filepath));
		ath12k_err(ab, "failed to fetch board data for %s from %s\n",
			   boardname, filepath);
		if (memcmp(boardname, fallback_boardname, strlen(boardname)))
			ath12k_err(ab, "failed to fetch board data for %s from %s\n",
				   fallback_boardname, filepath);

		ath12k_err(ab, "failed to fetch board.bin from %s\n",
			   ab->hw_params->fw.dir);
		return ret;
	}

success:
	ath12k_dbg(ab, ATH12K_DBG_BOOT, "using board api %d\n", bd_api);
	return 0;
}

int ath12k_core_fetch_regdb(struct ath12k_base *ab, struct ath12k_board_data *bd)
{
	char boardname[BOARD_NAME_SIZE], default_boardname[BOARD_NAME_SIZE];
	int ret;

	ret = ath12k_core_create_board_name(ab, boardname, BOARD_NAME_SIZE);
	if (ret) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "failed to create board name for regdb: %d", ret);
		goto exit;
	}

	ret = ath12k_core_fetch_board_data_api_n(ab, bd, boardname,
						 ATH12K_BD_IE_REGDB,
						 ATH12K_BD_IE_REGDB_NAME,
						 ATH12K_BD_IE_REGDB_DATA);
	if (!ret)
		goto exit;

	ret = ath12k_core_create_bus_type_board_name(ab, default_boardname,
						     BOARD_NAME_SIZE);
	if (ret) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "failed to create default board name for regdb: %d", ret);
		goto exit;
	}

	ret = ath12k_core_fetch_board_data_api_n(ab, bd, default_boardname,
						 ATH12K_BD_IE_REGDB,
						 ATH12K_BD_IE_REGDB_NAME,
						 ATH12K_BD_IE_REGDB_DATA);
	if (!ret)
		goto exit;

	ret = ath12k_core_fetch_board_data_api_1(ab, bd, ATH12K_REGDB_FILE_NAME);
	if (ret)
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "failed to fetch %s from %s\n",
			   ATH12K_REGDB_FILE_NAME, ab->hw_params->fw.dir);

exit:
	if (!ret)
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "fetched regdb\n");

	return ret;
}

u32 ath12k_core_get_max_station_per_radio(struct ath12k_base *ab)
{
	if (ab->num_radios == 2)
		return TARGET_NUM_STATIONS(ab, DBS);
	if (ab->num_radios == 3)
		return TARGET_NUM_STATIONS(ab, DBS_SBS);
	return TARGET_NUM_STATIONS(ab, SINGLE);
}

u32 ath12k_core_get_max_peers_per_radio(struct ath12k_base *ab)
{
	return ath12k_core_get_max_station_per_radio(ab) + TARGET_NUM_VDEVS(ab);
}
EXPORT_SYMBOL(ath12k_core_get_max_peers_per_radio);

struct reserved_mem *ath12k_core_get_reserved_mem(struct ath12k_base *ab,
						  int index)
{
	struct device *dev = ab->dev;
	struct reserved_mem *rmem;
	struct device_node *node;

	node = of_parse_phandle(dev->of_node, "memory-region", index);
	if (!node) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "failed to parse memory-region for index %d\n", index);
		return NULL;
	}

	rmem = of_reserved_mem_lookup(node);
	of_node_put(node);
	if (!rmem) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "unable to get memory-region for index %d\n", index);
		return NULL;
	}

	return rmem;
}

static inline
void ath12k_core_to_group_ref_get(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ab->ag;

	lockdep_assert_held(&ag->mutex);

	if (ab->hw_group_ref) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "core already attached to group %d\n",
			   ag->id);
		return;
	}

	ab->hw_group_ref = true;
	ag->num_started++;

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "core attached to group %d, num_started %d\n",
		   ag->id, ag->num_started);
}

static inline
void ath12k_core_to_group_ref_put(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ab->ag;

	lockdep_assert_held(&ag->mutex);

	if (!ab->hw_group_ref) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "core already de-attached from group %d\n",
			   ag->id);
		return;
	}

	ab->hw_group_ref = false;
	ag->num_started--;

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "core de-attached from group %d, num_started %d\n",
		   ag->id, ag->num_started);
}

static void ath12k_core_stop(struct ath12k_base *ab)
{
	ath12k_link_sta_rhash_tbl_destroy(ab);

	ath12k_core_to_group_ref_put(ab);

	if (!test_bit(ATH12K_FLAG_CRASH_FLUSH, &ab->dev_flags))
		ath12k_qmi_firmware_stop(ab);

	ath12k_acpi_stop(ab);

	ath12k_dp_rx_pdev_reo_cleanup(ab);
	ath12k_hif_stop(ab);
	ath12k_wmi_detach(ab);
	ath12k_dp_cmn_device_deinit(ath12k_ab_to_dp(ab));

	/* De-Init of components as needed */
}

static void ath12k_core_check_cc_code_bdfext(const struct dmi_header *hdr, void *data)
{
	struct ath12k_base *ab = data;
	const char *magic = ATH12K_SMBIOS_BDF_EXT_MAGIC;
	struct ath12k_smbios_bdf *smbios = (struct ath12k_smbios_bdf *)hdr;
	ssize_t copied;
	size_t len;
	int i;

	if (ab->qmi.target.bdf_ext[0] != '\0')
		return;

	if (hdr->type != ATH12K_SMBIOS_BDF_EXT_TYPE)
		return;

	if (hdr->length != ATH12K_SMBIOS_BDF_EXT_LENGTH) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "wrong smbios bdf ext type length (%d).\n",
			   hdr->length);
		return;
	}

	spin_lock_bh(&ab->base_lock);

	switch (smbios->country_code_flag) {
	case ATH12K_SMBIOS_CC_ISO:
		ab->new_alpha2[0] = u16_get_bits(smbios->cc_code >> 8, 0xff);
		ab->new_alpha2[1] = u16_get_bits(smbios->cc_code, 0xff);
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "boot smbios cc_code %c%c\n",
			   ab->new_alpha2[0], ab->new_alpha2[1]);
		break;
	case ATH12K_SMBIOS_CC_WW:
		ab->new_alpha2[0] = '0';
		ab->new_alpha2[1] = '0';
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "boot smbios worldwide regdomain\n");
		break;
	default:
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "boot ignore smbios country code setting %d\n",
			   smbios->country_code_flag);
		break;
	}

	spin_unlock_bh(&ab->base_lock);

	if (!smbios->bdf_enabled) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "bdf variant name not found.\n");
		return;
	}

	/* Only one string exists (per spec) */
	if (memcmp(smbios->bdf_ext, magic, strlen(magic)) != 0) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "bdf variant magic does not match.\n");
		return;
	}

	len = min_t(size_t,
		    strlen(smbios->bdf_ext), sizeof(ab->qmi.target.bdf_ext));
	for (i = 0; i < len; i++) {
		if (!isascii(smbios->bdf_ext[i]) || !isprint(smbios->bdf_ext[i])) {
			ath12k_dbg(ab, ATH12K_DBG_BOOT,
				   "bdf variant name contains non ascii chars.\n");
			return;
		}
	}

	/* Copy extension name without magic prefix */
	copied = strscpy(ab->qmi.target.bdf_ext, smbios->bdf_ext + strlen(magic),
			 sizeof(ab->qmi.target.bdf_ext));
	if (copied < 0) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "bdf variant string is longer than the buffer can accommodate\n");
		return;
	}

	ath12k_dbg(ab, ATH12K_DBG_BOOT,
		   "found and validated bdf variant smbios_type 0x%x bdf %s\n",
		   ATH12K_SMBIOS_BDF_EXT_TYPE, ab->qmi.target.bdf_ext);
}

int ath12k_core_check_smbios(struct ath12k_base *ab)
{
	ab->qmi.target.bdf_ext[0] = '\0';
	dmi_walk(ath12k_core_check_cc_code_bdfext, ab);

	/*
	 * Fall back to the module parameter when the platform supplied no
	 * country. dmi_walk() returns -ENXIO immediately on a device-tree
	 * system, so new_alpha2 is still zero here on any non-DMI platform.
	 * See the ath12k_country definition above for why this matters.
	 *
	 * Deliberately after dmi_walk(): a real SMBIOS record describes the
	 * hardware as shipped and outranks a value passed on the command line.
	 */
	if (!ab->new_alpha2[0] && ath12k_country &&
	    ath12k_country[0] >= 'A' && ath12k_country[0] <= 'Z' &&
	    ath12k_country[1] >= 'A' && ath12k_country[1] <= 'Z' &&
	    ath12k_country[2] == '\0') {
		ab->new_alpha2[0] = ath12k_country[0];
		ab->new_alpha2[1] = ath12k_country[1];
		ath12k_info(ab, "regulatory country %c%c from module parameter\n",
			    ab->new_alpha2[0], ab->new_alpha2[1]);
	} else if (!ab->new_alpha2[0] && ath12k_country) {
		ath12k_warn(ab, "ignoring malformed country parameter, expected two uppercase letters\n");
	}

	/* Latch the configured country the first time it is known. This runs on
	 * every QMI init, including after a resume, by which point new_alpha2 may
	 * already have been overwritten by 802.11d -- hence the one-shot guard.
	 */
	if (!ab->cfg_alpha2[0] && ab->new_alpha2[0]) {
		ab->cfg_alpha2[0] = ab->new_alpha2[0];
		ab->cfg_alpha2[1] = ab->new_alpha2[1];
	}

	if (ab->qmi.target.bdf_ext[0] == '\0')
		return -ENODATA;

	return 0;
}

static int ath12k_core_soc_create(struct ath12k_base *ab)
{
	int ret;

	if (ath12k_ftm_mode) {
		ab->fw_mode = ATH12K_FIRMWARE_MODE_FTM;
		ath12k_info(ab, "Booting in ftm mode\n");
	}

	ret = ath12k_qmi_init_service(ab);
	if (ret) {
		ath12k_err(ab, "failed to initialize qmi :%d\n", ret);
		return ret;
	}

	ath12k_debugfs_soc_create(ab);

	ret = ath12k_hif_power_up(ab);
	if (ret) {
		ath12k_err(ab, "failed to power up :%d\n", ret);
		goto err_qmi_deinit;
	}

	ath12k_debugfs_pdev_create(ab);

	return 0;

err_qmi_deinit:
	ath12k_debugfs_soc_destroy(ab);
	ath12k_qmi_deinit_service(ab);
	return ret;
}

static void ath12k_core_soc_destroy(struct ath12k_base *ab)
{
	ath12k_hif_power_down(ab, false);
	ath12k_reg_free(ab);
	ath12k_debugfs_soc_destroy(ab);
	ath12k_qmi_deinit_service(ab);
}

static int ath12k_core_pdev_create(struct ath12k_base *ab)
{
	int ret;

	ret = ath12k_dp_pdev_alloc(ab);
	if (ret) {
		ath12k_err(ab, "failed to attach DP pdev: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ath12k_core_pdev_destroy(struct ath12k_base *ab)
{
	ath12k_dp_pdev_free(ab);
}

static int ath12k_core_start(struct ath12k_base *ab)
{
	int ret;

	lockdep_assert_held(&ab->core_lock);

	ret = ath12k_wmi_attach(ab);
	if (ret) {
		ath12k_err(ab, "failed to attach wmi: %d\n", ret);
		return ret;
	}

	ret = ath12k_htc_init(ab);
	if (ret) {
		ath12k_err(ab, "failed to init htc: %d\n", ret);
		goto err_wmi_detach;
	}

	ret = ath12k_hif_start(ab);
	if (ret) {
		ath12k_err(ab, "failed to start HIF: %d\n", ret);
		goto err_wmi_detach;
	}

	ret = ath12k_htc_wait_target(&ab->htc);
	if (ret) {
		ath12k_err(ab, "failed to connect to HTC: %d\n", ret);
		goto err_hif_stop;
	}

	ret = ath12k_dp_htt_connect(ath12k_ab_to_dp(ab));
	if (ret) {
		ath12k_err(ab, "failed to connect to HTT: %d\n", ret);
		goto err_hif_stop;
	}

	ret = ath12k_wmi_connect(ab);
	if (ret) {
		ath12k_err(ab, "failed to connect wmi: %d\n", ret);
		goto err_hif_stop;
	}

	ret = ath12k_htc_start(&ab->htc);
	if (ret) {
		ath12k_err(ab, "failed to start HTC: %d\n", ret);
		goto err_hif_stop;
	}

	ret = ath12k_wmi_wait_for_service_ready(ab);
	if (ret) {
		ath12k_err(ab, "failed to receive wmi service ready event: %d\n",
			   ret);
		goto err_hif_stop;
	}

	ath12k_hal_cc_config(ab);

	ret = ath12k_dp_rx_pdev_reo_setup(ab);
	if (ret) {
		ath12k_err(ab, "failed to initialize reo destination rings: %d\n", ret);
		goto err_hif_stop;
	}

	ret = ath12k_wmi_cmd_init(ab);
	if (ret) {
		ath12k_err(ab, "failed to send wmi init cmd: %d\n", ret);
		goto err_reo_cleanup;
	}

	ret = ath12k_wmi_wait_for_unified_ready(ab);
	if (ret) {
		ath12k_err(ab, "failed to receive wmi unified ready event: %d\n",
			   ret);
		goto err_reo_cleanup;
	}

	/* put hardware to DBS mode */
	if (ab->hw_params->single_pdev_only) {
		ret = ath12k_wmi_set_hw_mode(ab, WMI_HOST_HW_MODE_DBS);
		if (ret) {
			ath12k_err(ab, "failed to send dbs mode: %d\n", ret);
			goto err_reo_cleanup;
		}
	}

	ret = ath12k_dp_tx_htt_h2t_ver_req_msg(ab);
	if (ret) {
		ath12k_err(ab, "failed to send htt version request message: %d\n",
			   ret);
		goto err_reo_cleanup;
	}

	ath12k_acpi_set_dsm_func(ab);

	/* Indicate the core start in the appropriate group */
	ath12k_core_to_group_ref_get(ab);

	ret = ath12k_link_sta_rhash_tbl_init(ab);
	if (ret) {
		ath12k_warn(ab, "failed to init peer addr rhash table %d\n", ret);
		goto err_reo_cleanup;
	}

	return 0;

err_reo_cleanup:
	ath12k_dp_rx_pdev_reo_cleanup(ab);
err_hif_stop:
	ath12k_hif_stop(ab);
err_wmi_detach:
	ath12k_wmi_detach(ab);
	return ret;
}

static void ath12k_core_device_cleanup(struct ath12k_base *ab)
{
	mutex_lock(&ab->core_lock);

	ath12k_hif_irq_disable(ab);
	ath12k_core_pdev_destroy(ab);

	mutex_unlock(&ab->core_lock);
}

static void ath12k_core_hw_group_stop(struct ath12k_hw_group *ag)
{
	struct ath12k_base *ab;
	int i;

	lockdep_assert_held(&ag->mutex);

	clear_bit(ATH12K_GROUP_FLAG_REGISTERED, &ag->flags);

	ath12k_mac_unregister(ag);

	for (i = ag->num_devices - 1; i >= 0; i--) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		clear_bit(ATH12K_FLAG_REGISTERED, &ab->dev_flags);

		ath12k_core_device_cleanup(ab);
	}

	ath12k_mac_destroy(ag);
}

u8 ath12k_get_num_partner_link(struct ath12k *ar)
{
	struct ath12k_base *partner_ab, *ab = ar->ab;
	struct ath12k_hw_group *ag = ab->ag;
	struct ath12k_pdev *pdev;
	u8 num_link = 0;
	int i, j;

	lockdep_assert_held(&ag->mutex);

	for (i = 0; i < ag->num_devices; i++) {
		partner_ab = ag->ab[i];

		for (j = 0; j < partner_ab->num_radios; j++) {
			pdev = &partner_ab->pdevs[j];

			/* Avoid the self link */
			if (ar == pdev->ar)
				continue;

			num_link++;
		}
	}

	return num_link;
}

static int __ath12k_mac_mlo_ready(struct ath12k *ar)
{
	u8 num_link = ath12k_get_num_partner_link(ar);
	int ret;

	if (num_link == 0)
		return 0;

	ret = ath12k_wmi_mlo_ready(ar);
	if (ret) {
		ath12k_err(ar->ab, "MLO ready failed for pdev %d: %d\n",
			   ar->pdev_idx, ret);
		return ret;
	}

	ath12k_dbg(ar->ab, ATH12K_DBG_MAC, "mlo ready done for pdev %d\n",
		   ar->pdev_idx);

	return 0;
}

int ath12k_mac_mlo_ready(struct ath12k_hw_group *ag)
{
	struct ath12k_hw *ah;
	struct ath12k *ar;
	int ret;
	int i, j;

	for (i = 0; i < ag->num_hw; i++) {
		ah = ag->ah[i];
		if (!ah)
			continue;

		for_each_ar(ah, ar, j) {
			ar = &ah->radio[j];
			ret = __ath12k_mac_mlo_ready(ar);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int ath12k_core_mlo_setup(struct ath12k_hw_group *ag)
{
	int ret, i;

	if (!ag->mlo_capable)
		return 0;

	ret = ath12k_mac_mlo_setup(ag);
	if (ret)
		return ret;

	for (i = 0; i < ag->num_devices; i++)
		ath12k_dp_partner_cc_init(ag->ab[i]);

	ret = ath12k_mac_mlo_ready(ag);
	if (ret)
		goto err_mlo_teardown;

	return 0;

err_mlo_teardown:
	ath12k_mac_mlo_teardown(ag);

	return ret;
}

static int ath12k_core_hw_group_start(struct ath12k_hw_group *ag)
{
	struct ath12k_base *ab;
	int ret, i;

	lockdep_assert_held(&ag->mutex);

	if (test_bit(ATH12K_GROUP_FLAG_REGISTERED, &ag->flags))
		goto core_pdev_create;

	ret = ath12k_mac_allocate(ag);
	if (WARN_ON(ret))
		return ret;

	ret = ath12k_core_mlo_setup(ag);
	if (WARN_ON(ret))
		goto err_mac_destroy;

	ret = ath12k_mac_register(ag);
	if (WARN_ON(ret))
		goto err_mlo_teardown;

	set_bit(ATH12K_GROUP_FLAG_REGISTERED, &ag->flags);

core_pdev_create:
	for (i = 0; i < ag->num_devices; i++) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		mutex_lock(&ab->core_lock);

		set_bit(ATH12K_FLAG_REGISTERED, &ab->dev_flags);

		ret = ath12k_core_pdev_create(ab);
		if (ret) {
			ath12k_err(ab, "failed to create pdev core %d\n", ret);
			mutex_unlock(&ab->core_lock);
			goto err;
		}

		ath12k_hif_irq_enable(ab);

		ret = ath12k_core_rfkill_config(ab);
		if (ret && ret != -EOPNOTSUPP) {
			mutex_unlock(&ab->core_lock);
			goto err;
		}

		mutex_unlock(&ab->core_lock);
	}

	return 0;

err:
	ath12k_core_hw_group_stop(ag);
	return ret;

err_mlo_teardown:
	ath12k_mac_mlo_teardown(ag);

err_mac_destroy:
	ath12k_mac_destroy(ag);

	return ret;
}

static int ath12k_core_start_firmware(struct ath12k_base *ab,
				      enum ath12k_firmware_mode mode)
{
	int ret;

	ath12k_ce_get_shadow_config(ab, &ab->qmi.ce_cfg.shadow_reg_v3,
				    &ab->qmi.ce_cfg.shadow_reg_v3_len);

	ret = ath12k_qmi_firmware_start(ab, mode);
	if (ret) {
		ath12k_err(ab, "failed to send firmware start: %d\n", ret);
		return ret;
	}

	return ret;
}

static inline
bool ath12k_core_hw_group_start_ready(struct ath12k_hw_group *ag)
{
	lockdep_assert_held(&ag->mutex);

	return (ag->num_started == ag->num_devices);
}

static void ath12k_fw_stats_pdevs_free(struct list_head *head)
{
	struct ath12k_fw_stats_pdev *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

void ath12k_fw_stats_bcn_free(struct list_head *head)
{
	struct ath12k_fw_stats_bcn *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

static void ath12k_fw_stats_vdevs_free(struct list_head *head)
{
	struct ath12k_fw_stats_vdev *i, *tmp;

	list_for_each_entry_safe(i, tmp, head, list) {
		list_del(&i->list);
		kfree(i);
	}
}

void ath12k_fw_stats_init(struct ath12k *ar)
{
	INIT_LIST_HEAD(&ar->fw_stats.vdevs);
	INIT_LIST_HEAD(&ar->fw_stats.pdevs);
	INIT_LIST_HEAD(&ar->fw_stats.bcn);
	init_completion(&ar->fw_stats_complete);
	init_completion(&ar->fw_stats_done);
}

void ath12k_fw_stats_free(struct ath12k_fw_stats *stats)
{
	ath12k_fw_stats_pdevs_free(&stats->pdevs);
	ath12k_fw_stats_vdevs_free(&stats->vdevs);
	ath12k_fw_stats_bcn_free(&stats->bcn);
}

void ath12k_fw_stats_reset(struct ath12k *ar)
{
	spin_lock_bh(&ar->data_lock);
	ath12k_fw_stats_free(&ar->fw_stats);
	ar->fw_stats.num_vdev_recvd = 0;
	spin_unlock_bh(&ar->data_lock);
}

static void ath12k_core_trigger_partner(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ab->ag;
	struct ath12k_base *partner_ab;
	bool found = false;
	int i;

	for (i = 0; i < ag->num_devices; i++) {
		partner_ab = ag->ab[i];
		if (!partner_ab)
			continue;

		if (found)
			ath12k_qmi_trigger_host_cap(partner_ab);

		found = (partner_ab == ab);
	}
}

int ath12k_core_qmi_firmware_ready(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ath12k_ab_to_ag(ab);
	int ret, i;

	ret = ath12k_core_start_firmware(ab, ab->fw_mode);
	if (ret) {
		ath12k_err(ab, "failed to start firmware: %d\n", ret);
		return ret;
	}

	ret = ath12k_ce_init_pipes(ab);
	if (ret) {
		ath12k_err(ab, "failed to initialize CE: %d\n", ret);
		goto err_firmware_stop;
	}

	ret = ath12k_dp_cmn_device_init(ath12k_ab_to_dp(ab));
	if (ret) {
		ath12k_err(ab, "failed to init DP: %d\n", ret);
		goto err_firmware_stop;
	}

	mutex_lock(&ag->mutex);
	mutex_lock(&ab->core_lock);

	ret = ath12k_core_start(ab);
	if (ret) {
		ath12k_err(ab, "failed to start core: %d\n", ret);
		goto err_deinit;
	}

	mutex_unlock(&ab->core_lock);

	if (ath12k_core_hw_group_start_ready(ag)) {
		ret = ath12k_core_hw_group_start(ag);
		if (ret) {
			ath12k_warn(ab, "unable to start hw group\n");
			goto err_core_stop;
		}
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "group %d started\n", ag->id);
	} else {
		ath12k_core_trigger_partner(ab);
	}

	mutex_unlock(&ag->mutex);

	return 0;

err_core_stop:
	for (i = ag->num_devices - 1; i >= 0; i--) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		mutex_lock(&ab->core_lock);
		ath12k_core_stop(ab);
		mutex_unlock(&ab->core_lock);
	}
	mutex_unlock(&ag->mutex);
	goto exit;

err_deinit:
	ath12k_dp_cmn_device_deinit(ath12k_ab_to_dp(ab));
	mutex_unlock(&ab->core_lock);
	mutex_unlock(&ag->mutex);

err_firmware_stop:
	ath12k_qmi_firmware_stop(ab);

exit:
	return ret;
}

static int ath12k_core_reconfigure_on_crash(struct ath12k_base *ab)
{
	int ret, total_vdev;

	mutex_lock(&ab->core_lock);
	ath12k_link_sta_rhash_tbl_destroy(ab);
	ath12k_dp_pdev_free(ab);
	ath12k_ce_cleanup_pipes(ab);
	ath12k_wmi_detach(ab);
	ath12k_dp_rx_pdev_reo_cleanup(ab);
	mutex_unlock(&ab->core_lock);

	ath12k_dp_cmn_device_deinit(ath12k_ab_to_dp(ab));
	ath12k_hal_srng_deinit(ab);
	total_vdev = ab->num_radios * TARGET_NUM_VDEVS(ab);
	ab->free_vdev_map = (1LL << total_vdev) - 1;

	ret = ath12k_hal_srng_init(ab);
	if (ret)
		return ret;

	clear_bit(ATH12K_FLAG_CRASH_FLUSH, &ab->dev_flags);

	ret = ath12k_core_qmi_firmware_ready(ab);
	if (ret)
		goto err_hal_srng_deinit;

	clear_bit(ATH12K_FLAG_RECOVERY, &ab->dev_flags);

	return 0;

err_hal_srng_deinit:
	ath12k_hal_srng_deinit(ab);
	return ret;
}

static void ath12k_rfkill_work(struct work_struct *work)
{
	struct ath12k_base *ab = container_of(work, struct ath12k_base, rfkill_work);
	struct ath12k_hw_group *ag = ab->ag;
	struct ath12k *ar;
	struct ath12k_hw *ah;
	struct ieee80211_hw *hw;
	bool rfkill_radio_on;
	int i, j;

	spin_lock_bh(&ab->base_lock);
	rfkill_radio_on = ab->rfkill_radio_on;
	spin_unlock_bh(&ab->base_lock);

	for (i = 0; i < ag->num_hw; i++) {
		ah = ath12k_ag_to_ah(ag, i);
		if (!ah)
			continue;

		for (j = 0; j < ah->num_radio; j++) {
			ar = &ah->radio[j];
			if (!ar)
				continue;

			ath12k_mac_rfkill_enable_radio(ar, rfkill_radio_on);
		}

		hw = ah->hw;
		wiphy_rfkill_set_hw_state(hw->wiphy, !rfkill_radio_on);
	}
}

void ath12k_core_halt(struct ath12k *ar)
{
	struct list_head *pos, *n;
	struct ath12k_base *ab = ar->ab;

	lockdep_assert_wiphy(ath12k_ar_to_hw(ar)->wiphy);

	ar->num_created_vdevs = 0;
	ar->allocated_vdev_map = 0;

	ath12k_mac_scan_finish(ar);
	ath12k_mac_peer_cleanup_all(ar);
	cancel_delayed_work_sync(&ar->scan.timeout);
	cancel_work_sync(&ar->regd_update_work);
	cancel_work_sync(&ar->regd_channel_update_work);
	cancel_work_sync(&ab->rfkill_work);
	cancel_work_sync(&ab->update_11d_work);

	rcu_assign_pointer(ab->pdevs_active[ar->pdev_idx], NULL);
	synchronize_rcu();

	spin_lock_bh(&ar->data_lock);
	list_for_each_safe(pos, n, &ar->arvifs)
		list_del_init(pos);
	spin_unlock_bh(&ar->data_lock);

	idr_init(&ar->txmgmt_idr);
}

static void ath12k_core_pre_reconfigure_recovery(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ab->ag;
	struct ath12k *ar;
	struct ath12k_hw *ah;
	int i, j;

	spin_lock_bh(&ab->base_lock);
	ab->stats.fw_crash_counter++;
	spin_unlock_bh(&ab->base_lock);

	if (ab->is_reset)
		set_bit(ATH12K_FLAG_CRASH_FLUSH, &ab->dev_flags);

	for (i = 0; i < ag->num_hw; i++) {
		ah = ath12k_ag_to_ah(ag, i);
		if (!ah || ah->state == ATH12K_HW_STATE_OFF ||
		    ah->state == ATH12K_HW_STATE_TM)
			continue;

		wiphy_lock(ah->hw->wiphy);

		/* If queue 0 is stopped, it is safe to assume that all
		 * other queues are stopped by driver via
		 * ieee80211_stop_queues() below. This means, there is
		 * no need to stop it again and hence continue
		 */
		if (ieee80211_queue_stopped(ah->hw, 0)) {
			wiphy_unlock(ah->hw->wiphy);
			continue;
		}

		ieee80211_stop_queues(ah->hw);

		for (j = 0; j < ah->num_radio; j++) {
			ar = &ah->radio[j];

			ath12k_mac_drain_tx(ar);
			ar->state_11d = ATH12K_11D_IDLE;
			complete(&ar->completed_11d_scan);
			complete(&ar->scan.started);
			complete_all(&ar->scan.completed);
			complete(&ar->scan.on_channel);
			complete(&ar->peer_assoc_done);
			complete(&ar->peer_delete_done);
			complete(&ar->install_key_done);
			complete(&ar->vdev_setup_done);
			complete(&ar->vdev_delete_done);
			complete(&ar->bss_survey_done);
			complete_all(&ar->regd_update_completed);

			wake_up(&ar->dp.tx_empty_waitq);
			idr_for_each(&ar->txmgmt_idr,
				     ath12k_mac_tx_mgmt_pending_free, ar);
			idr_destroy(&ar->txmgmt_idr);
			wake_up(&ar->txmgmt_empty_waitq);

			ar->monitor_vdev_id = -1;
			ar->monitor_vdev_created = false;
			ar->monitor_started = false;
		}

		wiphy_unlock(ah->hw->wiphy);
	}

	wake_up(&ab->wmi_ab.tx_credits_wq);
	wake_up(&ab->peer_mapping_wq);
}

static void ath12k_update_11d(struct work_struct *work)
{
	struct ath12k_base *ab = container_of(work, struct ath12k_base, update_11d_work);
	struct ath12k *ar;
	struct ath12k_pdev *pdev;
	struct wmi_set_current_country_arg arg = {};
	int ret, i;

	spin_lock_bh(&ab->base_lock);
	memcpy(&arg.alpha2, &ab->new_alpha2, 2);
	spin_unlock_bh(&ab->base_lock);

	ath12k_dbg(ab, ATH12K_DBG_WMI, "update 11d new cc %c%c\n",
		   arg.alpha2[0], arg.alpha2[1]);

	for (i = 0; i < ab->num_radios; i++) {
		pdev = &ab->pdevs[i];
		ar = pdev->ar;

		memcpy(&ar->alpha2, &arg.alpha2, 2);

		reinit_completion(&ar->regd_update_completed);

		ret = ath12k_wmi_send_set_current_country_cmd(ar, &arg);
		if (ret)
			ath12k_warn(ar->ab,
				    "pdev id %d failed set current country code: %d\n",
				    i, ret);
	}
}

static void ath12k_core_post_reconfigure_recovery(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ab->ag;
	struct ath12k_hw *ah;
	struct ath12k *ar;
	int i, j;

	for (i = 0; i < ag->num_hw; i++) {
		ah = ath12k_ag_to_ah(ag, i);
		if (!ah || ah->state == ATH12K_HW_STATE_OFF)
			continue;

		wiphy_lock(ah->hw->wiphy);
		mutex_lock(&ah->hw_mutex);

		switch (ah->state) {
		case ATH12K_HW_STATE_ON:
			ah->state = ATH12K_HW_STATE_RESTARTING;

			for (j = 0; j < ah->num_radio; j++) {
				ar = &ah->radio[j];
				ath12k_core_halt(ar);
			}

			ath12k_mac_dp_peer_cleanup(ah);
			break;
		case ATH12K_HW_STATE_OFF:
			ath12k_warn(ab,
				    "cannot restart hw %d that hasn't been started\n",
				    i);
			break;
		case ATH12K_HW_STATE_RESTARTING:
			break;
		case ATH12K_HW_STATE_RESTARTED:
			ah->state = ATH12K_HW_STATE_WEDGED;
			fallthrough;
		case ATH12K_HW_STATE_WEDGED:
			ath12k_warn(ab,
				    "device is wedged, will not restart hw %d\n", i);
			break;
		case ATH12K_HW_STATE_TM:
			ath12k_warn(ab, "fw mode reset done radio %d\n", i);
			break;
		}

		mutex_unlock(&ah->hw_mutex);
		wiphy_unlock(ah->hw->wiphy);
	}

	complete(&ab->driver_recovery);
}

static void ath12k_core_restart(struct work_struct *work)
{
	struct ath12k_base *ab = container_of(work, struct ath12k_base, restart_work);
	struct ath12k_hw_group *ag = ab->ag;
	struct ath12k_hw *ah;
	int ret, i;

	ret = ath12k_core_reconfigure_on_crash(ab);
	if (ret) {
		ath12k_err(ab, "failed to reconfigure driver on crash recovery\n");
		return;
	}

	if (ab->is_reset) {
		if (!test_bit(ATH12K_FLAG_REGISTERED, &ab->dev_flags)) {
			atomic_dec(&ab->reset_count);
			complete(&ab->reset_complete);
			ab->is_reset = false;
			atomic_set(&ab->fail_cont_count, 0);
			ath12k_dbg(ab, ATH12K_DBG_BOOT, "reset success\n");
		}

		mutex_lock(&ag->mutex);

		if (!ath12k_core_hw_group_start_ready(ag)) {
			mutex_unlock(&ag->mutex);
			goto exit_restart;
		}

		for (i = 0; i < ag->num_hw; i++) {
			ah = ath12k_ag_to_ah(ag, i);
			ieee80211_restart_hw(ah->hw);
		}

		mutex_unlock(&ag->mutex);
	}

exit_restart:
	complete(&ab->restart_completed);
}

static void ath12k_core_reset(struct work_struct *work)
{
	struct ath12k_base *ab = container_of(work, struct ath12k_base, reset_work);
	struct ath12k_hw_group *ag = ab->ag;
	int reset_count, fail_cont_count, i;
	long time_left;

	if (!(test_bit(ATH12K_FLAG_QMI_FW_READY_COMPLETE, &ab->dev_flags))) {
		ath12k_warn(ab, "ignore reset dev flags 0x%lx\n", ab->dev_flags);
		return;
	}

	/* Sometimes the recovery will fail and then the next all recovery fail,
	 * this is to avoid infinite recovery since it can not recovery success
	 */
	fail_cont_count = atomic_read(&ab->fail_cont_count);

	if (fail_cont_count >= ATH12K_RESET_MAX_FAIL_COUNT_FINAL)
		return;

	if (fail_cont_count >= ATH12K_RESET_MAX_FAIL_COUNT_FIRST &&
	    time_before(jiffies, ab->reset_fail_timeout))
		return;

	reset_count = atomic_inc_return(&ab->reset_count);

	if (reset_count > 1) {
		/* Sometimes it happened another reset worker before the previous one
		 * completed, then the second reset worker will destroy the previous one,
		 * thus below is to avoid that.
		 */
		ath12k_warn(ab, "already resetting count %d\n", reset_count);

		reinit_completion(&ab->reset_complete);
		time_left = wait_for_completion_timeout(&ab->reset_complete,
							ATH12K_RESET_TIMEOUT_HZ);
		if (time_left) {
			ath12k_dbg(ab, ATH12K_DBG_BOOT, "to skip reset\n");
			atomic_dec(&ab->reset_count);
			return;
		}

		ab->reset_fail_timeout = jiffies + ATH12K_RESET_FAIL_TIMEOUT_HZ;
		/* Record the continuous recovery fail count when recovery failed*/
		fail_cont_count = atomic_inc_return(&ab->fail_cont_count);
	}

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "reset starting\n");

	ab->is_reset = true;
	atomic_set(&ab->recovery_count, 0);

	ath12k_coredump_collect(ab);
	ath12k_core_pre_reconfigure_recovery(ab);

	ath12k_core_post_reconfigure_recovery(ab);

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "waiting recovery start...\n");

	ath12k_hif_irq_disable(ab);
	ath12k_hif_ce_irq_disable(ab);

	ath12k_hif_power_down(ab, false);

	/* prepare for power up */
	ab->qmi.num_radios = U8_MAX;

	mutex_lock(&ag->mutex);
	ath12k_core_to_group_ref_put(ab);

	if (ag->num_started > 0) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT,
			   "waiting for %d partner device(s) to reset\n",
			   ag->num_started);
		mutex_unlock(&ag->mutex);
		return;
	}

	/* Prepare MLO global memory region for power up */
	ath12k_qmi_reset_mlo_mem(ag);

	for (i = 0; i < ag->num_devices; i++) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		ath12k_hif_power_up(ab);
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "reset started\n");
	}

	mutex_unlock(&ag->mutex);
}

enum ath12k_qmi_mem_mode ath12k_core_get_memory_mode(struct ath12k_base *ab)
{
	unsigned long total_ram;
	struct sysinfo si;

	si_meminfo(&si);
	total_ram = si.totalram * si.mem_unit;

	if (total_ram < SZ_512M)
		return ATH12K_QMI_MEMORY_MODE_LOW_512_M;

	return ATH12K_QMI_MEMORY_MODE_DEFAULT;
}
EXPORT_SYMBOL(ath12k_core_get_memory_mode);

int ath12k_core_pre_init(struct ath12k_base *ab)
{
	const struct ath12k_mem_profile_based_param *param;

	param = &ath12k_mem_profile_based_param[ab->target_mem_mode];
	ab->profile_param = param;
	ath12k_fw_map(ab);

	return 0;
}

static int ath12k_core_panic_handler(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct ath12k_base *ab = container_of(nb, struct ath12k_base,
					      panic_nb);

	return ath12k_hif_panic_handler(ab);
}

static int ath12k_core_panic_notifier_register(struct ath12k_base *ab)
{
	ab->panic_nb.notifier_call = ath12k_core_panic_handler;

	return atomic_notifier_chain_register(&panic_notifier_list,
					      &ab->panic_nb);
}

static void ath12k_core_panic_notifier_unregister(struct ath12k_base *ab)
{
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &ab->panic_nb);
}

static inline
bool ath12k_core_hw_group_create_ready(struct ath12k_hw_group *ag)
{
	lockdep_assert_held(&ag->mutex);

	return (ag->num_probed == ag->num_devices);
}

static struct ath12k_hw_group *ath12k_core_hw_group_alloc(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag;
	int count = 0;

	lockdep_assert_held(&ath12k_hw_group_mutex);

	list_for_each_entry(ag, &ath12k_hw_group_list, list)
		count++;

	ag = kzalloc_obj(*ag);
	if (!ag)
		return NULL;

	ag->id = count;
	list_add(&ag->list, &ath12k_hw_group_list);
	mutex_init(&ag->mutex);
	ag->mlo_capable = false;

	return ag;
}

static void ath12k_core_hw_group_free(struct ath12k_hw_group *ag)
{
	mutex_lock(&ath12k_hw_group_mutex);

	list_del(&ag->list);
	kfree(ag);

	mutex_unlock(&ath12k_hw_group_mutex);
}

static struct ath12k_hw_group *ath12k_core_hw_group_find_by_dt(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag;
	int i;

	if (!ab->dev->of_node)
		return NULL;

	list_for_each_entry(ag, &ath12k_hw_group_list, list)
		for (i = 0; i < ag->num_devices; i++)
			if (ag->wsi_node[i] == ab->dev->of_node)
				return ag;

	return NULL;
}

static int ath12k_core_get_wsi_info(struct ath12k_hw_group *ag,
				    struct ath12k_base *ab)
{
	struct device_node *wsi_dev = ab->dev->of_node, *next_wsi_dev;
	struct device_node *tx_endpoint, *next_rx_endpoint;
	int device_count = 0;

	next_wsi_dev = wsi_dev;

	if (!next_wsi_dev)
		return -ENODEV;

	do {
		ag->wsi_node[device_count] = next_wsi_dev;

		tx_endpoint = of_graph_get_endpoint_by_regs(next_wsi_dev, 0, -1);
		if (!tx_endpoint) {
			of_node_put(next_wsi_dev);
			return -ENODEV;
		}

		next_rx_endpoint = of_graph_get_remote_endpoint(tx_endpoint);
		if (!next_rx_endpoint) {
			of_node_put(next_wsi_dev);
			of_node_put(tx_endpoint);
			return -ENODEV;
		}

		of_node_put(tx_endpoint);
		of_node_put(next_wsi_dev);

		next_wsi_dev = of_graph_get_port_parent(next_rx_endpoint);
		if (!next_wsi_dev) {
			of_node_put(next_rx_endpoint);
			return -ENODEV;
		}

		of_node_put(next_rx_endpoint);

		device_count++;
		if (device_count > ATH12K_MAX_DEVICES) {
			ath12k_warn(ab, "device count in DT %d is more than limit %d\n",
				    device_count, ATH12K_MAX_DEVICES);
			of_node_put(next_wsi_dev);
			return -EINVAL;
		}
	} while (wsi_dev != next_wsi_dev);

	of_node_put(next_wsi_dev);
	ag->num_devices = device_count;

	return 0;
}

static int ath12k_core_get_wsi_index(struct ath12k_hw_group *ag,
				     struct ath12k_base *ab)
{
	int i, wsi_controller_index = -1, node_index = -1;
	bool control;

	for (i = 0; i < ag->num_devices; i++) {
		control = of_property_read_bool(ag->wsi_node[i], "qcom,wsi-controller");
		if (control)
			wsi_controller_index = i;

		if (ag->wsi_node[i] == ab->dev->of_node)
			node_index = i;
	}

	if (wsi_controller_index == -1) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "wsi controller is not defined in dt");
		return -EINVAL;
	}

	if (node_index == -1) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "unable to get WSI node index");
		return -EINVAL;
	}

	ab->wsi_info.index = (ag->num_devices + node_index - wsi_controller_index) %
		ag->num_devices;

	return 0;
}

static struct ath12k_hw_group *ath12k_core_hw_group_assign(struct ath12k_base *ab)
{
	struct ath12k_wsi_info *wsi = &ab->wsi_info;
	struct ath12k_hw_group *ag;

	lockdep_assert_held(&ath12k_hw_group_mutex);

	if (ath12k_ftm_mode)
		goto invalid_group;

	/* The grouping of multiple devices will be done based on device tree file.
	 * The platforms that do not have any valid group information would have
	 * each device to be part of its own invalid group.
	 *
	 * We use group id ATH12K_INVALID_GROUP_ID for single device group
	 * which didn't have dt entry or wrong dt entry, there could be many
	 * groups with same group id, i.e ATH12K_INVALID_GROUP_ID. So
	 * default group id of ATH12K_INVALID_GROUP_ID combined with
	 * num devices in ath12k_hw_group determines if the group is
	 * multi device or single device group
	 */

	ag = ath12k_core_hw_group_find_by_dt(ab);
	if (!ag) {
		ag = ath12k_core_hw_group_alloc(ab);
		if (!ag) {
			ath12k_warn(ab, "unable to create new hw group\n");
			return NULL;
		}

		if (ath12k_core_get_wsi_info(ag, ab) ||
		    ath12k_core_get_wsi_index(ag, ab)) {
			ath12k_dbg(ab, ATH12K_DBG_BOOT,
				   "unable to get wsi info from dt, grouping single device");
			ag->id = ATH12K_INVALID_GROUP_ID;
			ag->num_devices = 1;
			memset(ag->wsi_node, 0, sizeof(ag->wsi_node));
			wsi->index = 0;
		}

		goto exit;
	} else if (test_bit(ATH12K_GROUP_FLAG_UNREGISTER, &ag->flags)) {
		ath12k_dbg(ab, ATH12K_DBG_BOOT, "group id %d in unregister state\n",
			   ag->id);
		goto invalid_group;
	} else {
		if (ath12k_core_get_wsi_index(ag, ab))
			goto invalid_group;
		goto exit;
	}

invalid_group:
	ag = ath12k_core_hw_group_alloc(ab);
	if (!ag) {
		ath12k_warn(ab, "unable to create new hw group\n");
		return NULL;
	}

	ag->id = ATH12K_INVALID_GROUP_ID;
	ag->num_devices = 1;
	wsi->index = 0;

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "single device added to hardware group\n");

exit:
	if (ag->num_probed >= ag->num_devices) {
		ath12k_warn(ab, "unable to add new device to group, max limit reached\n");
		goto invalid_group;
	}

	ab->device_id = ag->num_probed++;
	ag->ab[ab->device_id] = ab;
	ab->ag = ag;

	ath12k_dp_cmn_hw_group_assign(ath12k_ab_to_dp(ab), ag);

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "wsi group-id %d num-devices %d index %d",
		   ag->id, ag->num_devices, wsi->index);

	return ag;
}

void ath12k_core_hw_group_unassign(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag = ath12k_ab_to_ag(ab);
	u8 device_id = ab->device_id;
	int num_probed;

	if (!ag)
		return;

	mutex_lock(&ag->mutex);

	if (WARN_ON(device_id >= ag->num_devices)) {
		mutex_unlock(&ag->mutex);
		return;
	}

	if (WARN_ON(ag->ab[device_id] != ab)) {
		mutex_unlock(&ag->mutex);
		return;
	}

	ath12k_dp_cmn_hw_group_unassign(ath12k_ab_to_dp(ab), ag);

	ag->ab[device_id] = NULL;
	ab->ag = NULL;
	ab->device_id = ATH12K_INVALID_DEVICE_ID;

	if (ag->num_probed)
		ag->num_probed--;

	num_probed = ag->num_probed;

	mutex_unlock(&ag->mutex);

	if (!num_probed)
		ath12k_core_hw_group_free(ag);
}

static void ath12k_core_hw_group_destroy(struct ath12k_hw_group *ag)
{
	struct ath12k_base *ab;
	int i;

	if (WARN_ON(!ag))
		return;

	for (i = 0; i < ag->num_devices; i++) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		ath12k_core_soc_destroy(ab);
	}
}

void ath12k_core_hw_group_cleanup(struct ath12k_hw_group *ag)
{
	struct ath12k_base *ab;
	int i;

	if (!ag)
		return;

	mutex_lock(&ag->mutex);

	if (test_bit(ATH12K_GROUP_FLAG_UNREGISTER, &ag->flags)) {
		mutex_unlock(&ag->mutex);
		return;
	}

	set_bit(ATH12K_GROUP_FLAG_UNREGISTER, &ag->flags);

	ath12k_core_hw_group_stop(ag);

	for (i = 0; i < ag->num_devices; i++) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		mutex_lock(&ab->core_lock);
		ath12k_core_stop(ab);
		mutex_unlock(&ab->core_lock);
	}

	mutex_unlock(&ag->mutex);
}

static int ath12k_core_hw_group_create(struct ath12k_hw_group *ag)
{
	struct ath12k_base *ab;
	int i, ret;

	lockdep_assert_held(&ag->mutex);

	for (i = 0; i < ag->num_devices; i++) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		mutex_lock(&ab->core_lock);

		ret = ath12k_core_soc_create(ab);
		if (ret) {
			mutex_unlock(&ab->core_lock);
			ath12k_err(ab, "failed to create soc %d core: %d\n", i, ret);
			goto destroy;
		}

		mutex_unlock(&ab->core_lock);
	}

	return 0;

destroy:
	for (i--; i >= 0; i--) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		mutex_lock(&ab->core_lock);
		ath12k_core_soc_destroy(ab);
		mutex_unlock(&ab->core_lock);
	}

	return ret;
}

void ath12k_core_hw_group_set_mlo_capable(struct ath12k_hw_group *ag)
{
	struct ath12k_base *ab;
	int i;

	if (ath12k_ftm_mode)
		return;

	lockdep_assert_held(&ag->mutex);

	if (ag->num_devices == 1) {
		ab = ag->ab[0];
		/* QCN9274 firmware uses firmware IE for MLO advertisement */
		if (ab->fw.fw_features_valid) {
			ag->mlo_capable =
				ath12k_fw_feature_supported(ab, ATH12K_FW_FEATURE_MLO);
			return;
		}

		/* while WCN7850 firmware uses QMI single_chip_mlo_support bit */
		ag->mlo_capable = ab->single_chip_mlo_support;
		return;
	}

	ag->mlo_capable = true;

	for (i = 0; i < ag->num_devices; i++) {
		ab = ag->ab[i];
		if (!ab)
			continue;

		/* even if 1 device's firmware feature indicates MLO
		 * unsupported, make MLO unsupported for the whole group
		 */
		if (!ath12k_fw_feature_supported(ab, ATH12K_FW_FEATURE_MLO)) {
			ag->mlo_capable = false;
			return;
		}
	}
}

int ath12k_core_init(struct ath12k_base *ab)
{
	struct ath12k_hw_group *ag;
	int ret;

	ret = ath12k_core_panic_notifier_register(ab);
	if (ret)
		ath12k_warn(ab, "failed to register panic handler: %d\n", ret);

	mutex_lock(&ath12k_hw_group_mutex);

	ag = ath12k_core_hw_group_assign(ab);
	if (!ag) {
		mutex_unlock(&ath12k_hw_group_mutex);
		ath12k_warn(ab, "unable to get hw group\n");
		ret = -ENODEV;
		goto err_unregister_notifier;
	}

	mutex_unlock(&ath12k_hw_group_mutex);

	mutex_lock(&ag->mutex);

	ath12k_dbg(ab, ATH12K_DBG_BOOT, "num devices %d num probed %d\n",
		   ag->num_devices, ag->num_probed);

	if (ath12k_core_hw_group_create_ready(ag)) {
		ret = ath12k_core_hw_group_create(ag);
		if (ret) {
			mutex_unlock(&ag->mutex);
			ath12k_warn(ab, "unable to create hw group\n");
			goto err_unassign_hw_group;
		}
	}

	mutex_unlock(&ag->mutex);

	return 0;

err_unassign_hw_group:
	ath12k_core_hw_group_unassign(ab);
err_unregister_notifier:
	ath12k_core_panic_notifier_unregister(ab);

	return ret;
}

void ath12k_core_deinit(struct ath12k_base *ab)
{
	ath12k_core_hw_group_destroy(ab->ag);
	ath12k_core_hw_group_unassign(ab);
	ath12k_core_panic_notifier_unregister(ab);
}

void ath12k_core_free(struct ath12k_base *ab)
{
	timer_delete_sync(&ab->rx_replenish_retry);
	destroy_workqueue(ab->workqueue_aux);
	destroy_workqueue(ab->workqueue);
	kfree(ab);
}

struct ath12k_base *ath12k_core_alloc(struct device *dev, size_t priv_size,
				      enum ath12k_bus bus)
{
	struct ath12k_base *ab;

	ab = kzalloc(sizeof(*ab) + priv_size, GFP_KERNEL);
	if (!ab)
		return NULL;

	init_completion(&ab->driver_recovery);

	ab->workqueue = create_singlethread_workqueue("ath12k_wq");
	if (!ab->workqueue)
		goto err_sc_free;

	ab->workqueue_aux = create_singlethread_workqueue("ath12k_aux_wq");
	if (!ab->workqueue_aux)
		goto err_free_wq;

	mutex_init(&ab->core_lock);
	spin_lock_init(&ab->base_lock);
	init_completion(&ab->reset_complete);

	init_waitqueue_head(&ab->peer_mapping_wq);
	init_waitqueue_head(&ab->wmi_ab.tx_credits_wq);
	INIT_WORK(&ab->restart_work, ath12k_core_restart);
	INIT_WORK(&ab->reset_work, ath12k_core_reset);
	INIT_WORK(&ab->rfkill_work, ath12k_rfkill_work);
	INIT_WORK(&ab->dump_work, ath12k_coredump_upload);
	INIT_WORK(&ab->update_11d_work, ath12k_update_11d);

	timer_setup(&ab->rx_replenish_retry, ath12k_ce_rx_replenish_retry, 0);
	init_completion(&ab->htc_suspend);
	init_completion(&ab->restart_completed);
	init_completion(&ab->wow.wakeup_completed);

	ab->dev = dev;
	ab->hif.bus = bus;
	ab->qmi.num_radios = U8_MAX;
	ab->single_chip_mlo_support = false;

	/* Device index used to identify the devices in a group.
	 *
	 * In Intra-device MLO, only one device present in a group,
	 * so it is always zero.
	 *
	 * In Inter-device MLO, Multiple device present in a group,
	 * expect non-zero value.
	 */
	ab->device_id = 0;

	return ab;

err_free_wq:
	destroy_workqueue(ab->workqueue);
err_sc_free:
	kfree(ab);
	return NULL;
}

MODULE_DESCRIPTION("Driver support for Qualcomm Technologies WLAN devices");
MODULE_LICENSE("Dual BSD/GPL");
