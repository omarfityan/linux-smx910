// SPDX-License-Identifier: GPL-2.0
//
// cs35l45.c - CS35L45 ALSA SoC audio driver
//
// Copyright 2019-2022 Cirrus Logic, Inc.
//
// Author: James Schulman <james.schulman@cirrus.com>

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/firmware.h>
#include <linux/firmware/cirrus/wmfw.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#include "cs35l45.h"

static bool cs35l45_check_cspl_mbox_sts(const enum cs35l45_cspl_mboxcmd cmd,
					enum cs35l45_cspl_mboxstate sts)
{
	switch (cmd) {
	case CSPL_MBOX_CMD_NONE:
	case CSPL_MBOX_CMD_UNKNOWN_CMD:
		return true;
	case CSPL_MBOX_CMD_PAUSE:
	case CSPL_MBOX_CMD_OUT_OF_HIBERNATE:
		return (sts == CSPL_MBOX_STS_PAUSED);
	case CSPL_MBOX_CMD_RESUME:
		return (sts == CSPL_MBOX_STS_RUNNING);
	case CSPL_MBOX_CMD_REINIT:
		return (sts == CSPL_MBOX_STS_RUNNING);
	case CSPL_MBOX_CMD_STOP_PRE_REINIT:
		return (sts == CSPL_MBOX_STS_RDY_FOR_REINIT);
	case CSPL_MBOX_CMD_HIBERNATE:
		return (sts == CSPL_MBOX_STS_HIBERNATE);
	default:
		return false;
	}
}

static int cs35l45_set_cspl_mbox_cmd(struct cs35l45_private *cs35l45,
				      struct regmap *regmap,
				      const enum cs35l45_cspl_mboxcmd cmd)
{
	unsigned int sts = 0, i;
	int ret;

	if (!cs35l45->dsp.cs_dsp.running) {
		dev_err(cs35l45->dev, "DSP not running\n");
		return -EPERM;
	}

	// Set mailbox cmd
	ret = regmap_write(regmap, CS35L45_DSP_VIRT1_MBOX_1, cmd);
	if (ret < 0) {
		if (cmd != CSPL_MBOX_CMD_OUT_OF_HIBERNATE)
			dev_err(cs35l45->dev, "Failed to write MBOX: %d\n", ret);
		return ret;
	}

	// Read mailbox status and verify it is appropriate for the given cmd
	for (i = 0; i < 5; i++) {
		usleep_range(1000, 1100);

		ret = regmap_read(regmap, CS35L45_DSP_MBOX_2, &sts);
		if (ret < 0) {
			dev_err(cs35l45->dev, "Failed to read MBOX STS: %d\n", ret);
			continue;
		}

		if (!cs35l45_check_cspl_mbox_sts(cmd, sts))
			dev_dbg(cs35l45->dev, "[%u] cmd %u returned invalid sts %u", i, cmd, sts);
		else
			return 0;
	}

	if (cmd != CSPL_MBOX_CMD_OUT_OF_HIBERNATE)
		dev_err(cs35l45->dev, "Failed to set mailbox cmd %u (status %u)\n", cmd, sts);

	return -ENOMSG;
}

static int cs35l45_global_en_ev(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(component);

	dev_dbg(cs35l45->dev, "%s event : %x\n", __func__, event);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		regmap_write(cs35l45->regmap, CS35L45_GLOBAL_ENABLES,
			     CS35L45_GLOBAL_EN_MASK);

		usleep_range(CS35L45_POST_GLOBAL_EN_US, CS35L45_POST_GLOBAL_EN_US + 100);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		usleep_range(CS35L45_PRE_GLOBAL_DIS_US, CS35L45_PRE_GLOBAL_DIS_US + 100);

		regmap_write(cs35l45->regmap, CS35L45_GLOBAL_ENABLES, 0);
		break;
	default:
		break;
	}

	return 0;
}

static int cs35l45_write_cal_ctl(struct cs35l45_private *cs35l45,
				 const char *name, unsigned int alg,
				 unsigned int value)
{
	__be32 val = cpu_to_be32(value);
	int ret;

	ret = wm_adsp_write_ctl(&cs35l45->dsp, name, WMFW_ADSP2_XM, alg,
				&val, sizeof(val));
	if (ret)
		dev_err(cs35l45->dev,
			"Failed to write calibration control %s: %d\n", name, ret);

	return ret;
}

/*
 * Apply this unit's factory speaker calibration.
 *
 * Split in two, because the two halves must be written at DIFFERENT points in
 * the firmware boot, and the reason is the controls' own volatility:
 *
 *   CAL_STATUS / CAL_R / CAL_AMBIENT / CAL_CHECKSUM are NON-volatile, so
 *   cs_dsp_coeff_write_ctrl() stores them in the control cache, and
 *   cs_dsp_run() flushes the cache with cs_dsp_coeff_sync_controls() BEFORE it
 *   calls ops->start_core(). Writing them before wm_adsp_event() therefore
 *   places them in DSP memory before the core executes a single instruction --
 *   which is precisely what this device's own downstream driver does: its
 *   cs35l45_dsp_boot_ev() holds the core in reset, calls cirrus_cal_apply(),
 *   and only then calls halo_start_core().
 *
 *   VIMON_CAL_STATE / VSC / ISC are VOLATILE, so the same call returns -EPERM
 *   unless cs_dsp->running. They have to be written after the core is up.
 *
 * The ordering IS the fix. Written after the core has started, the values do
 * land in DSP memory and do read back correctly -- but the firmware has already
 * initialised without them, so CSPL reports an uncalibrated default in
 * CAL_R_SELECTED and clears the calibration block on its first processing pass.
 * Written before the core starts, CSPL adopts them, and CAL_R_SELECTED reads
 * back equal to CAL_R.
 *
 * There is deliberately NO mailbox handshake here. The downstream apply,
 * cirrus_cal_cspl_cal_apply(), issues none -- it writes these same controls and
 * returns. STOP_PRE_REINIT/REINIT belong to the calibration *procedure*, not to
 * applying a stored result, and a mailbox command cannot be acknowledged by a
 * core that has not been started.
 */
static void cs35l45_apply_cspl_calibration(struct cs35l45_private *cs35l45)
{
	unsigned int status = CS35L45_CAL_STATUS_APPLIED;
	int ret;

	if (!cs35l45->cal_valid)
		return;

	/*
	 * Failures are logged by cs35l45_write_cal_ctl() and not propagated:
	 * uncalibrated protection still produces sound, whereas failing this
	 * event would leave the device silent. Confirm success by reading CAL_R
	 * back, never by the absence of an error here.
	 */
	ret  = cs35l45_write_cal_ctl(cs35l45, "CAL_STATUS", CS35L45_ALG_ID_CSPL,
				     status);
	ret |= cs35l45_write_cal_ctl(cs35l45, "CAL_R", CS35L45_ALG_ID_CSPL,
				     cs35l45->cal_rdc);
	ret |= cs35l45_write_cal_ctl(cs35l45, "CAL_AMBIENT", CS35L45_ALG_ID_CSPL,
				     cs35l45->cal_ambient);
	ret |= cs35l45_write_cal_ctl(cs35l45, "CAL_CHECKSUM", CS35L45_ALG_ID_CSPL,
				     status + cs35l45->cal_rdc);

	/*
	 * Report what actually happened. An unconditional success message here
	 * previously printed "staged" for an amplifier whose four writes had all
	 * returned -ENOENT, which is precisely the kind of instrument that turns
	 * a failure into a passing log line.
	 */
	if (ret)
		dev_err(cs35l45->dev,
			"Calibration NOT staged (rdc=%u): the protection firmware will run uncalibrated\n",
			cs35l45->cal_rdc);
	else
		dev_info(cs35l45->dev,
			 "Calibration staged before DSP start: rdc=%u ambient=%u\n",
			 cs35l45->cal_rdc, cs35l45->cal_ambient);
}

static void cs35l45_apply_vimon_calibration(struct cs35l45_private *cs35l45)
{
	unsigned int vimon_status;

	if (!cs35l45->cal_valid)
		return;

	vimon_status = cs35l45->cal_vimon_valid ?
			CS35L45_VIMON_CAL_STATUS_SUCCESS :
			CS35L45_VIMON_CAL_STATUS_INVALID;

	cs35l45_write_cal_ctl(cs35l45, "VIMON_CAL_STATE", CS35L45_ALG_ID_VIMON,
			      vimon_status);

	if (!cs35l45->cal_vimon_valid)
		return;

	cs35l45_write_cal_ctl(cs35l45, "VSC", CS35L45_ALG_ID_VIMON,
			      cs35l45->cal_vsc);
	cs35l45_write_cal_ctl(cs35l45, "ISC", CS35L45_ALG_ID_VIMON,
			      cs35l45->cal_isc);

	dev_info(cs35l45->dev,
		 "VIMON trims applied: vsc=0x%06x isc=0x%06x state=%u\n",
		 cs35l45->cal_vsc, cs35l45->cal_isc, vimon_status);
}


static int cs35l45_dsp_preload_ev(struct snd_soc_dapm_widget *w,
				  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(component);
	int ret;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (cs35l45->dsp.cs_dsp.booted)
			return 0;

		return wm_adsp_early_event(w, kcontrol, event);
	case SND_SOC_DAPM_POST_PMU:
		if (cs35l45->dsp.cs_dsp.running)
			return 0;

		regmap_set_bits(cs35l45->regmap, CS35L45_PWRMGT_CTL,
				   CS35L45_MEM_RDY_MASK);

		/*
		 * wm_adsp_early_event() does not load the firmware; it only
		 * queues it (queue_work(&dsp->boot_work)). PRE_PMU therefore
		 * returns long before the wmfw and bin are parsed and before a
		 * single coefficient control exists, so staging the calibration
		 * here without waiting is a race against that work. Measured on
		 * this device: the first amplifier's four writes all returned
		 * -ENOENT while the other three happened to find their controls
		 * already created.
		 *
		 * wm_adsp_run() waits with exactly this flush before calling
		 * cs_dsp_run(), so performing it here places the staging
		 * deterministically between the firmware load and the core
		 * start. The flush inside wm_adsp_event() below then becomes a
		 * no-op.
		 */
		flush_work(&cs35l45->dsp.boot_work);

		/*
		 * Stage the CSPL calibration into the control cache BEFORE the
		 * core starts: cs_dsp_run() syncs cached controls into DSP
		 * memory ahead of ops->start_core(), so the firmware
		 * initialises with this unit's measured ReDC instead of a
		 * default. Unconditional -- the writes are four cached stores,
		 * and a flag that survived a firmware boot is what previously
		 * made the loss permanent.
		 */
		cs35l45_apply_cspl_calibration(cs35l45);

		ret = wm_adsp_event(w, kcontrol, event);
		if (ret)
			return ret;

		/* The VIMON trims are volatile and need a running core. */
		cs35l45_apply_vimon_calibration(cs35l45);

		return 0;
	case SND_SOC_DAPM_PRE_PMD:
		if (cs35l45->dsp.preloaded)
			return 0;

		if (cs35l45->dsp.cs_dsp.running) {
			ret = wm_adsp_event(w, kcontrol, event);
			if (ret)
				return ret;
		}

		return wm_adsp_early_event(w, kcontrol, event);
	default:
		return 0;
	}
}


static int cs35l45_dsp_audio_ev(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_component *component = snd_soc_dapm_to_component(w->dapm);
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(component);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		return cs35l45_set_cspl_mbox_cmd(cs35l45, cs35l45->regmap,
						 CSPL_MBOX_CMD_RESUME);
	case SND_SOC_DAPM_PRE_PMD:
		return cs35l45_set_cspl_mbox_cmd(cs35l45, cs35l45->regmap,
						 CSPL_MBOX_CMD_PAUSE);
	default:
		return 0;
	}
}

static int cs35l45_activate_ctl(struct snd_soc_component *component,
				const char *ctl_name, bool active)
{
	struct snd_card *card = component->card->snd_card;
	struct snd_kcontrol *kcontrol;
	struct snd_kcontrol_volatile *vd;
	unsigned int index_offset;

	kcontrol = snd_soc_component_get_kcontrol(component, ctl_name);
	if (!kcontrol) {
		dev_err(component->dev, "Can't find kcontrol %s\n", ctl_name);
		return -EINVAL;
	}

	index_offset = snd_ctl_get_ioff(kcontrol, &kcontrol->id);
	vd = &kcontrol->vd[index_offset];
	if (active)
		vd->access |= SNDRV_CTL_ELEM_ACCESS_WRITE;
	else
		vd->access &= ~SNDRV_CTL_ELEM_ACCESS_WRITE;

	snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_INFO, &kcontrol->id);

	return 0;
}

static int cs35l45_amplifier_mode_get(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs35l45_private *cs35l45 =
			snd_soc_component_get_drvdata(component);

	ucontrol->value.integer.value[0] = cs35l45->amplifier_mode;

	return 0;
}

static int cs35l45_amplifier_mode_put(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component = snd_kcontrol_chip(kcontrol);
	struct cs35l45_private *cs35l45 =
			snd_soc_component_get_drvdata(component);
	struct snd_soc_dapm_context *dapm =
			snd_soc_component_to_dapm(component);
	unsigned int amp_state;
	int ret;

	if ((ucontrol->value.integer.value[0] == cs35l45->amplifier_mode) ||
	    (ucontrol->value.integer.value[0] > AMP_MODE_RCV))
		return 0;

	snd_soc_dapm_mutex_lock(dapm);

	ret = regmap_read(cs35l45->regmap, CS35L45_BLOCK_ENABLES, &amp_state);
	if (ret < 0) {
		dev_err(cs35l45->dev, "Failed to read AMP state: %d\n", ret);
		snd_soc_dapm_mutex_unlock(dapm);
		return ret;
	}

	regmap_clear_bits(cs35l45->regmap, CS35L45_BLOCK_ENABLES,
				  CS35L45_AMP_EN_MASK);
	snd_soc_dapm_disable_pin_unlocked(dapm, "SPK");
	snd_soc_dapm_sync_unlocked(dapm);

	if (ucontrol->value.integer.value[0] == AMP_MODE_SPK) {
		regmap_clear_bits(cs35l45->regmap, CS35L45_BLOCK_ENABLES,
				  CS35L45_RCV_EN_MASK);

		regmap_update_bits(cs35l45->regmap, CS35L45_BLOCK_ENABLES,
				   CS35L45_BST_EN_MASK,
				   CS35L45_BST_ENABLE << CS35L45_BST_EN_SHIFT);

		regmap_update_bits(cs35l45->regmap, CS35L45_HVLV_CONFIG,
				   CS35L45_HVLV_MODE_MASK,
				   CS35L45_HVLV_OPERATION <<
				   CS35L45_HVLV_MODE_SHIFT);

		ret = cs35l45_activate_ctl(component, "Analog PCM Volume", true);
		if (ret < 0)
			dev_err(cs35l45->dev,
				"Unable to deactivate ctl (%d)\n", ret);

	} else  /* AMP_MODE_RCV */ {
		regmap_set_bits(cs35l45->regmap, CS35L45_BLOCK_ENABLES,
				CS35L45_RCV_EN_MASK);

		regmap_update_bits(cs35l45->regmap, CS35L45_BLOCK_ENABLES,
				   CS35L45_BST_EN_MASK,
				   CS35L45_BST_DISABLE_FET_OFF <<
				   CS35L45_BST_EN_SHIFT);

		regmap_update_bits(cs35l45->regmap, CS35L45_HVLV_CONFIG,
				   CS35L45_HVLV_MODE_MASK,
				   CS35L45_FORCE_LV_OPERATION <<
				   CS35L45_HVLV_MODE_SHIFT);

		regmap_clear_bits(cs35l45->regmap,
				  CS35L45_BLOCK_ENABLES2,
				  CS35L45_AMP_DRE_EN_MASK);

		regmap_update_bits(cs35l45->regmap, CS35L45_AMP_GAIN,
				   CS35L45_AMP_GAIN_PCM_MASK,
				   CS35L45_AMP_GAIN_PCM_13DBV <<
				   CS35L45_AMP_GAIN_PCM_SHIFT);

		ret = cs35l45_activate_ctl(component, "Analog PCM Volume", false);
		if (ret < 0)
			dev_err(cs35l45->dev,
				"Unable to deactivate ctl (%d)\n", ret);
	}

	if (amp_state & CS35L45_AMP_EN_MASK)
		regmap_set_bits(cs35l45->regmap, CS35L45_BLOCK_ENABLES,
				CS35L45_AMP_EN_MASK);

	snd_soc_dapm_enable_pin_unlocked(dapm, "SPK");
	snd_soc_dapm_sync_unlocked(dapm);
	snd_soc_dapm_mutex_unlock(dapm);

	cs35l45->amplifier_mode = ucontrol->value.integer.value[0];

	return 1;
}

static const char * const cs35l45_asp_tx_txt[] = {
	"Zero", "ASP_RX1", "ASP_RX2",
	"VMON", "IMON", "ERR_VOL",
	"VDD_BATTMON", "VDD_BSTMON",
	"DSP_TX1", "DSP_TX2",
	"Interpolator", "IL_TARGET",
};

static const unsigned int cs35l45_asp_tx_val[] = {
	CS35L45_PCM_SRC_ZERO, CS35L45_PCM_SRC_ASP_RX1, CS35L45_PCM_SRC_ASP_RX2,
	CS35L45_PCM_SRC_VMON, CS35L45_PCM_SRC_IMON, CS35L45_PCM_SRC_ERR_VOL,
	CS35L45_PCM_SRC_VDD_BATTMON, CS35L45_PCM_SRC_VDD_BSTMON,
	CS35L45_PCM_SRC_DSP_TX1, CS35L45_PCM_SRC_DSP_TX2,
	CS35L45_PCM_SRC_INTERPOLATOR, CS35L45_PCM_SRC_IL_TARGET,
};

static const struct soc_enum cs35l45_asp_tx_enums[] = {
	SOC_VALUE_ENUM_SINGLE(CS35L45_ASPTX1_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_asp_tx_txt), cs35l45_asp_tx_txt,
			      cs35l45_asp_tx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_ASPTX2_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_asp_tx_txt), cs35l45_asp_tx_txt,
			      cs35l45_asp_tx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_ASPTX3_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_asp_tx_txt), cs35l45_asp_tx_txt,
			      cs35l45_asp_tx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_ASPTX4_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_asp_tx_txt), cs35l45_asp_tx_txt,
			      cs35l45_asp_tx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_ASPTX5_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_asp_tx_txt), cs35l45_asp_tx_txt,
			      cs35l45_asp_tx_val),
};

static const char * const cs35l45_dsp_rx_txt[] = {
	"Zero", "ASP_RX1", "ASP_RX2",
	"VMON", "IMON", "ERR_VOL",
	"CLASSH_TGT", "VDD_BATTMON",
	"VDD_BSTMON", "TEMPMON",
};

static const unsigned int cs35l45_dsp_rx_val[] = {
	CS35L45_PCM_SRC_ZERO, CS35L45_PCM_SRC_ASP_RX1, CS35L45_PCM_SRC_ASP_RX2,
	CS35L45_PCM_SRC_VMON, CS35L45_PCM_SRC_IMON, CS35L45_PCM_SRC_ERR_VOL,
	CS35L45_PCM_SRC_CLASSH_TGT, CS35L45_PCM_SRC_VDD_BATTMON,
	CS35L45_PCM_SRC_VDD_BSTMON, CS35L45_PCM_SRC_TEMPMON,
};

static const struct soc_enum cs35l45_dsp_rx_enums[] = {
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX1_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX2_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX3_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX4_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX5_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX6_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX7_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
	SOC_VALUE_ENUM_SINGLE(CS35L45_DSP1RX8_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dsp_rx_txt), cs35l45_dsp_rx_txt,
			      cs35l45_dsp_rx_val),
};

static const char * const cs35l45_dac_txt[] = {
	"Zero", "ASP_RX1", "ASP_RX2", "DSP_TX1", "DSP_TX2"
};

static const unsigned int cs35l45_dac_val[] = {
	CS35L45_PCM_SRC_ZERO, CS35L45_PCM_SRC_ASP_RX1, CS35L45_PCM_SRC_ASP_RX2,
	CS35L45_PCM_SRC_DSP_TX1, CS35L45_PCM_SRC_DSP_TX2
};

static const struct soc_enum cs35l45_dacpcm_enums[] = {
	SOC_VALUE_ENUM_SINGLE(CS35L45_DACPCM1_INPUT, 0, CS35L45_PCM_SRC_MASK,
			      ARRAY_SIZE(cs35l45_dac_txt), cs35l45_dac_txt,
			      cs35l45_dac_val),
};

static const struct snd_kcontrol_new cs35l45_asp_muxes[] = {
	SOC_DAPM_ENUM("ASP_TX1 Source", cs35l45_asp_tx_enums[0]),
	SOC_DAPM_ENUM("ASP_TX2 Source", cs35l45_asp_tx_enums[1]),
	SOC_DAPM_ENUM("ASP_TX3 Source", cs35l45_asp_tx_enums[2]),
	SOC_DAPM_ENUM("ASP_TX4 Source", cs35l45_asp_tx_enums[3]),
	SOC_DAPM_ENUM("ASP_TX5 Source", cs35l45_asp_tx_enums[4]),
};

static const struct snd_kcontrol_new cs35l45_dsp_muxes[] = {
	SOC_DAPM_ENUM("DSP_RX1 Source", cs35l45_dsp_rx_enums[0]),
	SOC_DAPM_ENUM("DSP_RX2 Source", cs35l45_dsp_rx_enums[1]),
	SOC_DAPM_ENUM("DSP_RX3 Source", cs35l45_dsp_rx_enums[2]),
	SOC_DAPM_ENUM("DSP_RX4 Source", cs35l45_dsp_rx_enums[3]),
	SOC_DAPM_ENUM("DSP_RX5 Source", cs35l45_dsp_rx_enums[4]),
	SOC_DAPM_ENUM("DSP_RX6 Source", cs35l45_dsp_rx_enums[5]),
	SOC_DAPM_ENUM("DSP_RX7 Source", cs35l45_dsp_rx_enums[6]),
	SOC_DAPM_ENUM("DSP_RX8 Source", cs35l45_dsp_rx_enums[7]),
};

static const struct snd_kcontrol_new cs35l45_dac_muxes[] = {
	SOC_DAPM_ENUM("DACPCM Source", cs35l45_dacpcm_enums[0]),
};
static const struct snd_kcontrol_new amp_en_ctl =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

/*
 * The two Battery Protection Engines. Both controls are SND_SOC_NOPM: they
 * write no register themselves, they only connect or disconnect the path. The
 * register bit belongs to the WIDGET, and DAPM writes it when the branch
 * powers up -- which is why stock's BLOCK_ENABLES2 carries bits 12 and 13
 * during playback and clears them at idle.
 */
static const struct snd_kcontrol_new abpe_en_ctl =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_kcontrol_new bbpe_en_ctl =
	SOC_DAPM_SINGLE("Switch", SND_SOC_NOPM, 0, 1, 0);

static const struct snd_soc_dapm_widget cs35l45_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("DSP1 Preload", NULL),
	SND_SOC_DAPM_SUPPLY_S("DSP1 Preloader", 100, SND_SOC_NOPM, 0, 0,
				cs35l45_dsp_preload_ev,
				SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("DSP1", SND_SOC_NOPM, 0, 0, NULL, 0,
				cs35l45_dsp_audio_ev,
				SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("GLOBAL_EN", SND_SOC_NOPM, 0, 0,
			    cs35l45_global_en_ev,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("ASP_EN", CS35L45_BLOCK_ENABLES2, CS35L45_ASP_EN_SHIFT, 0, NULL, 0),

	SND_SOC_DAPM_SIGGEN("VMON_SRC"),
	SND_SOC_DAPM_SIGGEN("IMON_SRC"),
	SND_SOC_DAPM_SIGGEN("TEMPMON_SRC"),
	SND_SOC_DAPM_SIGGEN("VDD_BATTMON_SRC"),
	SND_SOC_DAPM_SIGGEN("VDD_BSTMON_SRC"),
	SND_SOC_DAPM_SIGGEN("ERR_VOL"),
	SND_SOC_DAPM_SIGGEN("AMP_INTP"),
	SND_SOC_DAPM_SIGGEN("IL_TARGET"),

	SND_SOC_DAPM_SUPPLY("VMON_EN", CS35L45_BLOCK_ENABLES, CS35L45_VMON_EN_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("IMON_EN", CS35L45_BLOCK_ENABLES, CS35L45_IMON_EN_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("TEMPMON_EN", CS35L45_BLOCK_ENABLES, CS35L45_TEMPMON_EN_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("VDD_BATTMON_EN", CS35L45_BLOCK_ENABLES, CS35L45_VDD_BATTMON_EN_SHIFT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("VDD_BSTMON_EN", CS35L45_BLOCK_ENABLES, CS35L45_VDD_BSTMON_EN_SHIFT, 0, NULL, 0),

	SND_SOC_DAPM_ADC("VMON", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("IMON", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("TEMPMON", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("VDD_BATTMON", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("VDD_BSTMON", NULL, SND_SOC_NOPM, 0, 0),


	SND_SOC_DAPM_AIF_IN("ASP_RX1", NULL, 0, CS35L45_ASP_ENABLES1, CS35L45_ASP_RX1_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_IN("ASP_RX2", NULL, 1, CS35L45_ASP_ENABLES1, CS35L45_ASP_RX2_EN_SHIFT, 0),

	SND_SOC_DAPM_AIF_OUT("ASP_TX1", NULL, 0, CS35L45_ASP_ENABLES1, CS35L45_ASP_TX1_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX2", NULL, 1, CS35L45_ASP_ENABLES1, CS35L45_ASP_TX2_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX3", NULL, 2, CS35L45_ASP_ENABLES1, CS35L45_ASP_TX3_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX4", NULL, 3, CS35L45_ASP_ENABLES1, CS35L45_ASP_TX4_EN_SHIFT, 0),
	SND_SOC_DAPM_AIF_OUT("ASP_TX5", NULL, 4, CS35L45_ASP_ENABLES1, CS35L45_ASP_TX5_EN_SHIFT, 0),

	SND_SOC_DAPM_MUX("ASP_TX1 Source", SND_SOC_NOPM, 0, 0, &cs35l45_asp_muxes[0]),
	SND_SOC_DAPM_MUX("ASP_TX2 Source", SND_SOC_NOPM, 0, 0, &cs35l45_asp_muxes[1]),
	SND_SOC_DAPM_MUX("ASP_TX3 Source", SND_SOC_NOPM, 0, 0, &cs35l45_asp_muxes[2]),
	SND_SOC_DAPM_MUX("ASP_TX4 Source", SND_SOC_NOPM, 0, 0, &cs35l45_asp_muxes[3]),
	SND_SOC_DAPM_MUX("ASP_TX5 Source", SND_SOC_NOPM, 0, 0, &cs35l45_asp_muxes[4]),

	SND_SOC_DAPM_MUX("DSP_RX1 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[0]),
	SND_SOC_DAPM_MUX("DSP_RX2 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[1]),
	SND_SOC_DAPM_MUX("DSP_RX3 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[2]),
	SND_SOC_DAPM_MUX("DSP_RX4 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[3]),
	SND_SOC_DAPM_MUX("DSP_RX5 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[4]),
	SND_SOC_DAPM_MUX("DSP_RX6 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[5]),
	SND_SOC_DAPM_MUX("DSP_RX7 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[6]),
	SND_SOC_DAPM_MUX("DSP_RX8 Source", SND_SOC_NOPM, 0, 0, &cs35l45_dsp_muxes[7]),

	SND_SOC_DAPM_MUX("DACPCM Source", SND_SOC_NOPM, 0, 0, &cs35l45_dac_muxes[0]),

	SND_SOC_DAPM_SWITCH("AMP Enable", SND_SOC_NOPM, 0, 0, &amp_en_ctl),

	SND_SOC_DAPM_SWITCH("ABPE Enable", CS35L45_BLOCK_ENABLES2,
			    CS35L45_ABPE_EN_SHIFT, 0, &abpe_en_ctl),
	SND_SOC_DAPM_SWITCH("BBPE Enable", CS35L45_BLOCK_ENABLES2,
			    CS35L45_BBPE_EN_SHIFT, 0, &bbpe_en_ctl),

	SND_SOC_DAPM_OUT_DRV("AMP", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("SPK"),
};

#define CS35L45_ASP_MUX_ROUTE(name) \
	{ name" Source", "ASP_RX1",	 "ASP_RX1" }, \
	{ name" Source", "ASP_RX2",	 "ASP_RX2" }, \
	{ name" Source", "DSP_TX1",	 "DSP1" }, \
	{ name" Source", "DSP_TX2",	 "DSP1" }, \
	{ name" Source", "VMON",	 "VMON" }, \
	{ name" Source", "IMON",	 "IMON" }, \
	{ name" Source", "ERR_VOL",	 "ERR_VOL" }, \
	{ name" Source", "VDD_BATTMON",	 "VDD_BATTMON" }, \
	{ name" Source", "VDD_BSTMON",	 "VDD_BSTMON" }, \
	{ name" Source", "Interpolator", "AMP_INTP" }, \
	{ name" Source", "IL_TARGET",	 "IL_TARGET" }

#define CS35L45_DSP_MUX_ROUTE(name) \
	{ name" Source", "ASP_RX1",	"ASP_RX1" }, \
	{ name" Source", "ASP_RX2",	"ASP_RX2" }

#define CS35L45_DAC_MUX_ROUTE(name) \
	{ name" Source", "ASP_RX1",	"ASP_RX1" }, \
	{ name" Source", "ASP_RX2",	"ASP_RX2" }, \
	{ name" Source", "DSP_TX1",	"DSP1" }, \
	{ name" Source", "DSP_TX2",	"DSP1" }

static const struct snd_soc_dapm_route cs35l45_dapm_routes[] = {
	/* Feedback */
	{ "VMON", NULL, "VMON_SRC" },
	{ "IMON", NULL, "IMON_SRC" },
	{ "TEMPMON", NULL, "TEMPMON_SRC" },
	{ "VDD_BATTMON", NULL, "VDD_BATTMON_SRC" },
	{ "VDD_BSTMON", NULL, "VDD_BSTMON_SRC" },

	{ "VMON", NULL, "VMON_EN" },
	{ "IMON", NULL, "IMON_EN" },
	{ "TEMPMON", NULL, "TEMPMON_EN" },
	{ "VDD_BATTMON", NULL, "VDD_BATTMON_EN" },
	{ "VDD_BSTMON", NULL, "VDD_BSTMON_EN" },

	{ "Capture", NULL, "ASP_TX1"},
	{ "Capture", NULL, "ASP_TX2"},
	{ "Capture", NULL, "ASP_TX3"},
	{ "Capture", NULL, "ASP_TX4"},
	{ "Capture", NULL, "ASP_TX5"},
	{ "ASP_TX1", NULL, "ASP_TX1 Source"},
	{ "ASP_TX2", NULL, "ASP_TX2 Source"},
	{ "ASP_TX3", NULL, "ASP_TX3 Source"},
	{ "ASP_TX4", NULL, "ASP_TX4 Source"},
	{ "ASP_TX5", NULL, "ASP_TX5 Source"},

	{ "ASP_TX1", NULL, "ASP_EN" },
	{ "ASP_TX2", NULL, "ASP_EN" },
	{ "ASP_TX3", NULL, "ASP_EN" },
	{ "ASP_TX4", NULL, "ASP_EN" },
	{ "ASP_TX1", NULL, "GLOBAL_EN" },
	{ "ASP_TX2", NULL, "GLOBAL_EN" },
	{ "ASP_TX3", NULL, "GLOBAL_EN" },
	{ "ASP_TX4", NULL, "GLOBAL_EN" },
	{ "ASP_TX5", NULL, "GLOBAL_EN" },

	CS35L45_ASP_MUX_ROUTE("ASP_TX1"),
	CS35L45_ASP_MUX_ROUTE("ASP_TX2"),
	CS35L45_ASP_MUX_ROUTE("ASP_TX3"),
	CS35L45_ASP_MUX_ROUTE("ASP_TX4"),
	CS35L45_ASP_MUX_ROUTE("ASP_TX5"),

	/* Playback */
	{ "ASP_RX1", NULL, "Playback" },
	{ "ASP_RX2", NULL, "Playback" },
	{ "ASP_RX1", NULL, "ASP_EN" },
	{ "ASP_RX2", NULL, "ASP_EN" },

	{ "AMP", NULL, "DACPCM Source"},
	{ "AMP", NULL, "GLOBAL_EN"},

	CS35L45_DSP_MUX_ROUTE("DSP_RX1"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX2"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX3"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX4"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX5"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX6"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX7"),
	CS35L45_DSP_MUX_ROUTE("DSP_RX8"),

	{"DSP1", NULL, "DSP_RX1 Source"},
	{"DSP1", NULL, "DSP_RX2 Source"},
	{"DSP1", NULL, "DSP_RX3 Source"},
	{"DSP1", NULL, "DSP_RX4 Source"},
	{"DSP1", NULL, "DSP_RX5 Source"},
	{"DSP1", NULL, "DSP_RX6 Source"},
	{"DSP1", NULL, "DSP_RX7 Source"},
	{"DSP1", NULL, "DSP_RX8 Source"},

	{"DSP1", NULL, "VMON_EN"},
	{"DSP1", NULL, "IMON_EN"},
	{"DSP1", NULL, "VDD_BATTMON_EN"},
	{"DSP1", NULL, "VDD_BSTMON_EN"},
	{"DSP1", NULL, "TEMPMON_EN"},

	{"DSP1 Preload", NULL, "DSP1 Preloader"},
	{"DSP1", NULL, "DSP1 Preloader"},

	CS35L45_DAC_MUX_ROUTE("DACPCM"),

	{ "AMP Enable", "Switch", "AMP" },
	{ "SPK", NULL, "AMP Enable"},

	/*
	 * The Battery Protection Engines hang off the playback path as parallel
	 * branches, which is the shape the vendor uses:
	 *
	 *	{"Entry", NULL, "AMP Enable"},
	 *	{"ABPE Enable", "Switch", "Entry"},
	 *	{"Exit",  NULL, "ABPE Enable"},
	 *	{"SPK",   NULL, "Exit"},
	 *
	 * The vendor's "Entry" sits immediately downstream of its "AMP Enable"
	 * switch and its "Exit" feeds SPK, so with no Entry/Exit mixers in this
	 * graph the faithful attachment point is our own "AMP Enable", and SPK
	 * is where the branch rejoins.
	 *
	 * The switches are Off at power-on -- dapm_kcontrol_data_alloc() takes a
	 * kzalloc and these are not autodisable -- so userspace has to close
	 * them, exactly as it does on stock, where they read On at 68 s uptime
	 * with the amplifier switch still Off. Ours are closed by the UCM
	 * profile's EnableSequence, alongside "AMP Enable".
	 */
	{ "ABPE Enable", "Switch", "AMP Enable" },
	{ "BBPE Enable", "Switch", "AMP Enable" },
	{ "SPK", NULL, "ABPE Enable" },
	{ "SPK", NULL, "BBPE Enable" },
};

static const char * const amplifier_mode_texts[] = {"SPK", "RCV"};
static SOC_ENUM_SINGLE_DECL(amplifier_mode_enum, SND_SOC_NOPM, 0,
			    amplifier_mode_texts);
static DECLARE_TLV_DB_SCALE(amp_gain_tlv, 1000, 300, 0);
static const DECLARE_TLV_DB_SCALE(cs35l45_dig_pcm_vol_tlv, -10225, 25, true);

static const struct snd_kcontrol_new cs35l45_controls[] = {
	SOC_ENUM_EXT("Amplifier Mode", amplifier_mode_enum,
		     cs35l45_amplifier_mode_get, cs35l45_amplifier_mode_put),
	SOC_SINGLE_TLV("Analog PCM Volume", CS35L45_AMP_GAIN,
			CS35L45_AMP_GAIN_PCM_SHIFT,
			CS35L45_AMP_GAIN_PCM_MASK >> CS35L45_AMP_GAIN_PCM_SHIFT,
			0, amp_gain_tlv),
	/* Ignore bit 0: it is beyond the resolution of TLV_DB_SCALE */
	SOC_SINGLE_S_TLV("Digital PCM Volume",
			 CS35L45_AMP_PCM_CONTROL,
			 CS35L45_AMP_VOL_PCM_SHIFT + 1,
			 -409, 48,
			 (CS35L45_AMP_VOL_PCM_WIDTH - 1) - 1,
			 0, cs35l45_dig_pcm_vol_tlv),
	WM_ADSP2_PRELOAD_SWITCH("DSP1", 1),
	WM_ADSP_FW_CONTROL("DSP1", 0),

	/*
	 * ASP slot positions.
	 *
	 * A CS35L45 does not learn its slot from snd_soc_dai_set_tdm_slot():
	 * cs35l45_asp_set_tdm_slot() keeps the slot width and the slot count and
	 * discards both masks. The part takes its position from
	 * ASP_FRAME_CONTROL1 and ASP_FRAME_CONTROL5 instead, and nothing in this
	 * driver has ever written either, so every part stayed at the reset
	 * default: RX1 on slot 0, RX2 on slot 1, TX1..TX4 on slots 0..3.
	 *
	 * That default only describes a lone amplifier on a stereo link. A board
	 * that hangs several parts off one TDM frame gives each of them a
	 * different slot, and that is a per-part decision the DAI op cannot
	 * carry: the parts share a DAI link, so one set_tdm_slot() call reaches
	 * all of them with the same mask. Cirrus's own driver therefore exposes
	 * the positions as controls and lets the card's UCM profile place each
	 * part; these are transcribed from it, name for name, so that an
	 * existing mixer configuration keeps working.
	 *
	 * The fields are six bits wide, hence 0..63. Pointing a receiver at a
	 * slot the frame does not contain is how an unused input is told to take
	 * nothing, so the full range is offered rather than the slot count.
	 */
	SOC_SINGLE_RANGE("ASPTX1 Slot Position", CS35L45_ASP_FRAME_CONTROL1,
			 CS35L45_ASP_TX1_SLOT_SHIFT, 0, 63, 0),
	SOC_SINGLE_RANGE("ASPTX2 Slot Position", CS35L45_ASP_FRAME_CONTROL1,
			 CS35L45_ASP_TX2_SLOT_SHIFT, 0, 63, 0),
	SOC_SINGLE_RANGE("ASPTX3 Slot Position", CS35L45_ASP_FRAME_CONTROL1,
			 CS35L45_ASP_TX3_SLOT_SHIFT, 0, 63, 0),
	SOC_SINGLE_RANGE("ASPTX4 Slot Position", CS35L45_ASP_FRAME_CONTROL1,
			 CS35L45_ASP_TX4_SLOT_SHIFT, 0, 63, 0),
	SOC_SINGLE_RANGE("ASPRX1 Slot Position", CS35L45_ASP_FRAME_CONTROL5,
			 CS35L45_ASP_RX1_SLOT_SHIFT, 0, 63, 0),
	SOC_SINGLE_RANGE("ASPRX2 Slot Position", CS35L45_ASP_FRAME_CONTROL5,
			 CS35L45_ASP_RX2_SLOT_SHIFT, 0, 63, 0),
};

static int cs35l45_set_pll(struct cs35l45_private *cs35l45, unsigned int freq)
{
	unsigned int val;
	int freq_id;

	freq_id = cs35l45_get_clk_freq_id(freq);
	if (freq_id < 0) {
		dev_err(cs35l45->dev, "Invalid freq: %u\n", freq);
		return -EINVAL;
	}

	regmap_read(cs35l45->regmap, CS35L45_REFCLK_INPUT, &val);
	val = (val & CS35L45_PLL_REFCLK_FREQ_MASK) >> CS35L45_PLL_REFCLK_FREQ_SHIFT;
	if (val == freq_id)
		return 0;

	regmap_set_bits(cs35l45->regmap, CS35L45_REFCLK_INPUT, CS35L45_PLL_OPEN_LOOP_MASK);
	regmap_update_bits(cs35l45->regmap, CS35L45_REFCLK_INPUT,
			   CS35L45_PLL_REFCLK_FREQ_MASK,
			   freq_id << CS35L45_PLL_REFCLK_FREQ_SHIFT);
	regmap_clear_bits(cs35l45->regmap, CS35L45_REFCLK_INPUT, CS35L45_PLL_REFCLK_EN_MASK);
	regmap_clear_bits(cs35l45->regmap, CS35L45_REFCLK_INPUT, CS35L45_PLL_OPEN_LOOP_MASK);
	regmap_set_bits(cs35l45->regmap, CS35L45_REFCLK_INPUT, CS35L45_PLL_REFCLK_EN_MASK);

	return 0;
}

static int cs35l45_asp_set_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(codec_dai->component);
	unsigned int asp_fmt, fsync_inv, bclk_inv;

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_CBC_CFC:
		break;
	default:
		dev_err(cs35l45->dev, "Invalid DAI clocking\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		asp_fmt = CS35l45_ASP_FMT_DSP_A;
		break;
	case SND_SOC_DAIFMT_I2S:
		asp_fmt = CS35L45_ASP_FMT_I2S;
		break;
	default:
		dev_err(cs35l45->dev, "Invalid DAI format\n");
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_IF:
		fsync_inv = 1;
		bclk_inv = 0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		fsync_inv = 0;
		bclk_inv = 1;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		fsync_inv = 1;
		bclk_inv = 1;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		fsync_inv = 0;
		bclk_inv = 0;
		break;
	default:
		dev_warn(cs35l45->dev, "Invalid DAI clock polarity\n");
		return -EINVAL;
	}

	regmap_update_bits(cs35l45->regmap, CS35L45_ASP_CONTROL2,
			   CS35L45_ASP_FMT_MASK |
			   CS35L45_ASP_FSYNC_INV_MASK |
			   CS35L45_ASP_BCLK_INV_MASK,
			   (asp_fmt << CS35L45_ASP_FMT_SHIFT) |
			   (fsync_inv << CS35L45_ASP_FSYNC_INV_SHIFT) |
			   (bclk_inv << CS35L45_ASP_BCLK_INV_SHIFT));

	return 0;
}

static int cs35l45_asp_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(dai->component);
	unsigned int asp_width, asp_wl, global_fs, slot_multiple, asp_fmt;
	int bclk;

	switch (params_rate(params)) {
	case 44100:
		global_fs = CS35L45_44P100_KHZ;
		break;
	case 48000:
		global_fs = CS35L45_48P0_KHZ;
		break;
	case 88200:
		global_fs = CS35L45_88P200_KHZ;
		break;
	case 96000:
		global_fs = CS35L45_96P0_KHZ;
		break;
	default:
		dev_warn(cs35l45->dev, "Unsupported sample rate (%d)\n",
			 params_rate(params));
		return -EINVAL;
	}

	regmap_update_bits(cs35l45->regmap, CS35L45_GLOBAL_SAMPLE_RATE,
			   CS35L45_GLOBAL_FS_MASK,
			   global_fs << CS35L45_GLOBAL_FS_SHIFT);

	asp_wl = params_width(params);

	if (cs35l45->slot_width)
		asp_width = cs35l45->slot_width;
	else
		asp_width = params_width(params);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(cs35l45->regmap, CS35L45_ASP_CONTROL2,
				   CS35L45_ASP_WIDTH_RX_MASK,
				   asp_width << CS35L45_ASP_WIDTH_RX_SHIFT);

		regmap_update_bits(cs35l45->regmap, CS35L45_ASP_DATA_CONTROL5,
				   CS35L45_ASP_WL_MASK,
				   asp_wl << CS35L45_ASP_WL_SHIFT);
	} else {
		regmap_update_bits(cs35l45->regmap, CS35L45_ASP_CONTROL2,
				   CS35L45_ASP_WIDTH_TX_MASK,
				   asp_width << CS35L45_ASP_WIDTH_TX_SHIFT);

		regmap_update_bits(cs35l45->regmap, CS35L45_ASP_DATA_CONTROL1,
				   CS35L45_ASP_WL_MASK,
				   asp_wl << CS35L45_ASP_WL_SHIFT);
	}

	if (cs35l45->sysclk_set)
		return 0;

	/* I2S always has an even number of channels */
	regmap_read(cs35l45->regmap, CS35L45_ASP_CONTROL2, &asp_fmt);
	asp_fmt = (asp_fmt & CS35L45_ASP_FMT_MASK) >> CS35L45_ASP_FMT_SHIFT;
	if (asp_fmt == CS35L45_ASP_FMT_I2S)
		slot_multiple = 2;
	else
		slot_multiple = 1;

	bclk = snd_soc_tdm_params_to_bclk(params, asp_width,
					  cs35l45->slot_count, slot_multiple);

	return cs35l45_set_pll(cs35l45, bclk);
}

static int cs35l45_asp_set_tdm_slot(struct snd_soc_dai *dai,
				    unsigned int tx_mask, unsigned int rx_mask,
				    int slots, int slot_width)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(dai->component);

	if (slot_width && ((slot_width < 16) || (slot_width > 128)))
		return -EINVAL;

	cs35l45->slot_width = slot_width;
	cs35l45->slot_count = slots;

	return 0;
}

static int cs35l45_asp_set_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(dai->component);
	int ret;

	if (clk_id != 0) {
		dev_err(cs35l45->dev, "Invalid clk_id %d\n", clk_id);
		return -EINVAL;
	}

	cs35l45->sysclk_set = false;
	if (freq == 0)
		return 0;

	ret = cs35l45_set_pll(cs35l45, freq);
	if (ret < 0)
		return -EINVAL;

	cs35l45->sysclk_set = true;

	return 0;
}

static int cs35l45_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(dai->component);
	unsigned int global_fs, val, hpf_tune;

	if (mute)
		return 0;

	regmap_read(cs35l45->regmap, CS35L45_GLOBAL_SAMPLE_RATE, &global_fs);
	global_fs = (global_fs & CS35L45_GLOBAL_FS_MASK) >> CS35L45_GLOBAL_FS_SHIFT;
	switch (global_fs) {
	case CS35L45_44P100_KHZ:
		hpf_tune = CS35L45_HPF_44P1;
		break;
	case CS35L45_88P200_KHZ:
		hpf_tune = CS35L45_HPF_88P2;
		break;
	default:
		hpf_tune = CS35l45_HPF_DEFAULT;
		break;
	}

	regmap_read(cs35l45->regmap, CS35L45_AMP_PCM_HPF_TST, &val);
	if (val != hpf_tune) {
		struct reg_sequence hpf_override_seq[] = {
			{ 0x00000040,			0x00000055 },
			{ 0x00000040,			0x000000AA },
			{ 0x00000044,			0x00000055 },
			{ 0x00000044,			0x000000AA },
			{ CS35L45_AMP_PCM_HPF_TST,	hpf_tune },
			{ 0x00000040,			0x00000000 },
			{ 0x00000044,			0x00000000 },
		};
		regmap_multi_reg_write(cs35l45->regmap, hpf_override_seq,
				       ARRAY_SIZE(hpf_override_seq));
	}

	return 0;
}

static const struct snd_soc_dai_ops cs35l45_asp_dai_ops = {
	.set_fmt = cs35l45_asp_set_fmt,
	.hw_params = cs35l45_asp_hw_params,
	.set_tdm_slot = cs35l45_asp_set_tdm_slot,
	.set_sysclk = cs35l45_asp_set_sysclk,
	.mute_stream = cs35l45_mute_stream,
};

static struct snd_soc_dai_driver cs35l45_dai[] = {
	{
		.name = "cs35l45",
		.playback = {
			.stream_name = "Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = CS35L45_RATES,
			.formats = CS35L45_FORMATS,
		},
		.capture = {
			.stream_name = "Capture",
			.channels_min = 1,
			.channels_max = 5,
			.rates = CS35L45_RATES,
			.formats = CS35L45_FORMATS,
		},
		.symmetric_rate = true,
		.symmetric_sample_bits = true,
		.ops = &cs35l45_asp_dai_ops,
	},
};

static int cs35l45_component_probe(struct snd_soc_component *component)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(component);

	return wm_adsp2_component_probe(&cs35l45->dsp, component);
}

static void cs35l45_component_remove(struct snd_soc_component *component)
{
	struct cs35l45_private *cs35l45 = snd_soc_component_get_drvdata(component);

	wm_adsp2_component_remove(&cs35l45->dsp, component);
}

static const struct snd_soc_component_driver cs35l45_component = {
	.probe = cs35l45_component_probe,
	.remove = cs35l45_component_remove,

	.dapm_widgets = cs35l45_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(cs35l45_dapm_widgets),

	.dapm_routes = cs35l45_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(cs35l45_dapm_routes),

	.controls = cs35l45_controls,
	.num_controls = ARRAY_SIZE(cs35l45_controls),

	.name = "cs35l45",

	.endianness = 1,
};

static void cs35l45_setup_hibernate(struct cs35l45_private *cs35l45)
{
	unsigned int wksrc;

	if (cs35l45->bus_type == CONTROL_BUS_I2C)
		wksrc = CS35L45_WKSRC_I2C;
	else
		wksrc = CS35L45_WKSRC_SPI;

	regmap_update_bits(cs35l45->regmap, CS35L45_WAKESRC_CTL,
			   CS35L45_WKSRC_EN_MASK,
			   wksrc << CS35L45_WKSRC_EN_SHIFT);

	regmap_set_bits(cs35l45->regmap, CS35L45_WAKESRC_CTL,
			   CS35L45_UPDT_WKCTL_MASK);

	regmap_update_bits(cs35l45->regmap, CS35L45_WKI2C_CTL,
			   CS35L45_WKI2C_ADDR_MASK, cs35l45->i2c_addr);

	regmap_set_bits(cs35l45->regmap, CS35L45_WKI2C_CTL,
			   CS35L45_UPDT_WKI2C_MASK);
}

static int cs35l45_enter_hibernate(struct cs35l45_private *cs35l45)
{
	dev_dbg(cs35l45->dev, "Enter hibernate\n");

	cs35l45_setup_hibernate(cs35l45);

	regmap_set_bits(cs35l45->regmap, CS35L45_IRQ1_MASK_2, CS35L45_DSP_VIRT2_MBOX_MASK);

	// Don't wait for ACK since bus activity would wake the device
	regmap_write(cs35l45->regmap, CS35L45_DSP_VIRT1_MBOX_1, CSPL_MBOX_CMD_HIBERNATE);

	return 0;
}

static int cs35l45_exit_hibernate(struct cs35l45_private *cs35l45)
{
	const int wake_retries = 20;
	const int sleep_retries = 5;
	int ret, i, j;

	for (i = 0; i < sleep_retries; i++) {
		dev_dbg(cs35l45->dev, "Exit hibernate\n");

		for (j = 0; j < wake_retries; j++) {
			ret = cs35l45_set_cspl_mbox_cmd(cs35l45, cs35l45->regmap,
					  CSPL_MBOX_CMD_OUT_OF_HIBERNATE);
			if (!ret) {
				dev_dbg(cs35l45->dev, "Wake success at cycle: %d\n", j);
				regmap_clear_bits(cs35l45->regmap, CS35L45_IRQ1_MASK_2,
						 CS35L45_DSP_VIRT2_MBOX_MASK);
				return 0;
			}
			usleep_range(100, 200);
		}

		dev_err(cs35l45->dev, "Wake failed, re-enter hibernate: %d\n", ret);

		cs35l45_setup_hibernate(cs35l45);
	}

	dev_err(cs35l45->dev, "Timed out waking device\n");

	return -ETIMEDOUT;
}

static void cs35l45_pulse_classh_ovb_latch(struct cs35l45_private *cs35l45);
static void cs35l45_pulse_bpe_misc_commit(struct cs35l45_private *cs35l45);

static int cs35l45_runtime_suspend(struct device *dev)
{
	struct cs35l45_private *cs35l45 = dev_get_drvdata(dev);

	if (!cs35l45->dsp.preloaded || !cs35l45->dsp.cs_dsp.running)
		return 0;

	cs35l45_enter_hibernate(cs35l45);

	regcache_cache_only(cs35l45->regmap, true);
	regcache_mark_dirty(cs35l45->regmap);

	dev_dbg(cs35l45->dev, "Runtime suspended\n");

	return 0;
}

static int cs35l45_runtime_resume(struct device *dev)
{
	struct cs35l45_private *cs35l45 = dev_get_drvdata(dev);
	int ret;

	if (!cs35l45->dsp.preloaded || !cs35l45->dsp.cs_dsp.running)
		return 0;

	dev_dbg(cs35l45->dev, "Runtime resume\n");

	regcache_cache_only(cs35l45->regmap, false);

	ret = cs35l45_exit_hibernate(cs35l45);
	if (ret)
		return ret;

	ret = regcache_sync(cs35l45->regmap);
	if (ret != 0)
		dev_warn(cs35l45->dev, "regcache_sync failed: %d\n", ret);

	/*
	 * regcache_sync() has just restored the Class-H configuration, but a
	 * cache can only restore values, not edges: CH_OVB_LATCH rests at zero
	 * and its pulse is never replayed. Re-issue it here, on the boards
	 * that configure Class-H at all. Writing it twice is harmless; leaving
	 * thresholds written but not latched would not be.
	 */
	if (cs35l45->classh_configured)
		cs35l45_pulse_classh_ovb_latch(cs35l45);

	/*
	 * Same shape, same reason: the BPE_MISC_CONFIG bit-15 pulse rests at
	 * zero, so regcache_sync() has restored the block's configuration
	 * without re-issuing it.
	 */
	if (cs35l45->bpe_misc_configured)
		cs35l45_pulse_bpe_misc_commit(cs35l45);

	/* Clear global error status */
	regmap_clear_bits(cs35l45->regmap, CS35L45_ERROR_RELEASE, CS35L45_GLOBAL_ERR_RLS_MASK);
	regmap_set_bits(cs35l45->regmap, CS35L45_ERROR_RELEASE, CS35L45_GLOBAL_ERR_RLS_MASK);
	regmap_clear_bits(cs35l45->regmap, CS35L45_ERROR_RELEASE, CS35L45_GLOBAL_ERR_RLS_MASK);
	return ret;
}

static int cs35l45_sys_suspend(struct device *dev)
{
	struct cs35l45_private *cs35l45 = dev_get_drvdata(dev);

	dev_dbg(cs35l45->dev, "System suspend, disabling IRQ\n");
	disable_irq(cs35l45->irq);

	return 0;
}

static int cs35l45_sys_suspend_noirq(struct device *dev)
{
	struct cs35l45_private *cs35l45 = dev_get_drvdata(dev);

	dev_dbg(cs35l45->dev, "Late system suspend, reenabling IRQ\n");
	enable_irq(cs35l45->irq);

	return 0;
}

static int cs35l45_sys_resume_noirq(struct device *dev)
{
	struct cs35l45_private *cs35l45 = dev_get_drvdata(dev);

	dev_dbg(cs35l45->dev, "Early system resume, disabling IRQ\n");
	disable_irq(cs35l45->irq);

	return 0;
}

static int cs35l45_sys_resume(struct device *dev)
{
	struct cs35l45_private *cs35l45 = dev_get_drvdata(dev);

	dev_dbg(cs35l45->dev, "System resume, reenabling IRQ\n");
	enable_irq(cs35l45->irq);

	return 0;
}

/*
 * Read this unit's factory speaker calibration out of the device tree.
 *
 * These are per-unit measurements, not per-board constants: the factory
 * records them in the sec_efs partition under /cirrus, keyed by an amplifier
 * suffix, and they are carried here per amplifier node. Absent properties
 * simply leave the calibration disabled, which is the pre-existing behaviour.
 *
 * cirrus,cal-isc and cirrus,cal-vsc are signed 24-bit trims stored zero-filled
 * in 32 bits, matching the vendor's own bounds (e.g. its ISC lower bound is
 * 0x00FFBE77). They must not be sign-extended.
 */
static void cs35l45_parse_calibration(struct cs35l45_private *cs35l45)
{
	struct device *dev = cs35l45->dev;

	if (device_property_read_u32(dev, "cirrus,cal-rdc", &cs35l45->cal_rdc) ||
	    device_property_read_u32(dev, "cirrus,cal-ambient",
				     &cs35l45->cal_ambient))
		return;

	cs35l45->cal_valid = true;

	/* The VIMON trims are optional; a unit may carry ReDC calibration only. */
	if (device_property_read_u32(dev, "cirrus,cal-vsc", &cs35l45->cal_vsc) ||
	    device_property_read_u32(dev, "cirrus,cal-isc", &cs35l45->cal_isc))
		return;

	cs35l45->cal_vimon_valid = true;
}


/*
 * ---------------------------------------------------------------------------
 * Battery Protection Engine (BPE) and boost-BPE.
 *
 * Two staged brown-out protections the amplifier runs autonomously. BPE watches
 * the battery rail VP against four thresholds and applies a per-level output
 * attenuation with its own attack, hold and release rates. Boost-BPE drives the
 * boost converter's current limits from the same ladder, and the IL_LIM block
 * adds an inductor-current limiter on top.
 *
 * This device's own stock firmware programs all of it, identically, on all four
 * amplifiers; mainline programmed none of it. The values live in the device
 * tree, so the port is a transcription of the vendor's MECHANISM -- the
 * register layout and the order things are applied -- while the numbers stay
 * where they belong, in the DT.
 *
 * A level whose entry has reg == 0 is one the hardware does not implement at
 * that level (there is no BST_BPE_INST_L4_THLD field, no L0_ILIM, and so on).
 * The vendor's tables carry the same holes; they are reproduced rather than
 * filled, because filling them would write a neighbouring field.
 * ---------------------------------------------------------------------------
 */
struct cs35l45_bpe_field {
	unsigned int reg;	/* 0: not implemented at this level -- skip */
	unsigned int mask;
	unsigned int shift;
};

/* A DT property holding one u32 per level, e.g. bpe-inst-thld = <a b c d>. */
struct cs35l45_bpe_level_prop {
	const char *name;
	unsigned int levels;
	const struct cs35l45_bpe_field *field;
};

/* A DT property holding a single u32. */
struct cs35l45_scalar_prop {
	const char *name;
	unsigned int reg;
	unsigned int mask;
	unsigned int shift;
};

static const struct cs35l45_bpe_field cs35l45_bpe_inst_thld_lvl[] = {
	{ CS35L45_BPE_INST_THLD, CS35L45_BPE_INST_L0_THLD_MASK, CS35L45_BPE_INST_L0_THLD_SHIFT },
	{ CS35L45_BPE_INST_THLD, CS35L45_BPE_INST_L1_THLD_MASK, CS35L45_BPE_INST_L1_THLD_SHIFT },
	{ CS35L45_BPE_INST_THLD, CS35L45_BPE_INST_L2_THLD_MASK, CS35L45_BPE_INST_L2_THLD_SHIFT },
	{ CS35L45_BPE_INST_THLD, CS35L45_BPE_INST_L3_THLD_MASK, CS35L45_BPE_INST_L3_THLD_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bpe_inst_attn_lvl[] = {
	{ CS35L45_BPE_INST_ATTN, CS35L45_BPE_INST_L0_ATTN_MASK, CS35L45_BPE_INST_L0_ATTN_SHIFT },
	{ CS35L45_BPE_INST_ATTN, CS35L45_BPE_INST_L1_ATTN_MASK, CS35L45_BPE_INST_L1_ATTN_SHIFT },
	{ CS35L45_BPE_INST_ATTN, CS35L45_BPE_INST_L2_ATTN_MASK, CS35L45_BPE_INST_L2_ATTN_SHIFT },
	{ CS35L45_BPE_INST_ATTN, CS35L45_BPE_INST_L3_ATTN_MASK, CS35L45_BPE_INST_L3_ATTN_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bpe_inst_atk_rate_lvl[] = {
	{ CS35L45_BPE_INST_ATK_RATE, CS35L45_BPE_INST_L0_ATK_RATE_MASK, CS35L45_BPE_INST_L0_ATK_RATE_SHIFT },
	{ CS35L45_BPE_INST_ATK_RATE, CS35L45_BPE_INST_L1_ATK_RATE_MASK, CS35L45_BPE_INST_L1_ATK_RATE_SHIFT },
	{ CS35L45_BPE_INST_ATK_RATE, CS35L45_BPE_INST_L2_ATK_RATE_MASK, CS35L45_BPE_INST_L2_ATK_RATE_SHIFT },
	{ CS35L45_BPE_INST_ATK_RATE, CS35L45_BPE_INST_L3_ATK_RATE_MASK, CS35L45_BPE_INST_L3_ATK_RATE_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bpe_inst_hold_time_lvl[] = {
	{ CS35L45_BPE_INST_HOLD_TIME, CS35L45_BPE_INST_L0_HOLD_TIME_MASK, CS35L45_BPE_INST_L0_HOLD_TIME_SHIFT },
	{ CS35L45_BPE_INST_HOLD_TIME, CS35L45_BPE_INST_L1_HOLD_TIME_MASK, CS35L45_BPE_INST_L1_HOLD_TIME_SHIFT },
	{ CS35L45_BPE_INST_HOLD_TIME, CS35L45_BPE_INST_L2_HOLD_TIME_MASK, CS35L45_BPE_INST_L2_HOLD_TIME_SHIFT },
	{ CS35L45_BPE_INST_HOLD_TIME, CS35L45_BPE_INST_L3_HOLD_TIME_MASK, CS35L45_BPE_INST_L3_HOLD_TIME_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bpe_inst_rls_rate_lvl[] = {
	{ CS35L45_BPE_INST_RLS_RATE, CS35L45_BPE_INST_L0_RLS_RATE_MASK, CS35L45_BPE_INST_L0_RLS_RATE_SHIFT },
	{ CS35L45_BPE_INST_RLS_RATE, CS35L45_BPE_INST_L1_RLS_RATE_MASK, CS35L45_BPE_INST_L1_RLS_RATE_SHIFT },
	{ CS35L45_BPE_INST_RLS_RATE, CS35L45_BPE_INST_L2_RLS_RATE_MASK, CS35L45_BPE_INST_L2_RLS_RATE_SHIFT },
	{ CS35L45_BPE_INST_RLS_RATE, CS35L45_BPE_INST_L3_RLS_RATE_MASK, CS35L45_BPE_INST_L3_RLS_RATE_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bst_bpe_inst_thld_lvl[] = {
	{ CS35L45_BST_BPE_INST_THLD, CS35L45_BST_BPE_INST_L0_THLD_MASK, CS35L45_BST_BPE_INST_L0_THLD_SHIFT },
	{ CS35L45_BST_BPE_INST_THLD, CS35L45_BST_BPE_INST_L1_THLD_MASK, CS35L45_BST_BPE_INST_L1_THLD_SHIFT },
	{ CS35L45_BST_BPE_INST_THLD, CS35L45_BST_BPE_INST_L2_THLD_MASK, CS35L45_BST_BPE_INST_L2_THLD_SHIFT },
	{ CS35L45_BST_BPE_INST_THLD, CS35L45_BST_BPE_INST_L3_THLD_MASK, CS35L45_BST_BPE_INST_L3_THLD_SHIFT },
	{ 0 },   /* hardware has no such field at this level */
};

static const struct cs35l45_bpe_field cs35l45_bst_bpe_inst_ilim_lvl[] = {
	{ 0 },   /* hardware has no such field at this level */
	{ CS35L45_BST_BPE_INST_ILIM, CS35L45_BST_BPE_INST_L1_ILIM_MASK, CS35L45_BST_BPE_INST_L1_ILIM_SHIFT },
	{ CS35L45_BST_BPE_INST_ILIM, CS35L45_BST_BPE_INST_L2_ILIM_MASK, CS35L45_BST_BPE_INST_L2_ILIM_SHIFT },
	{ CS35L45_BST_BPE_INST_ILIM, CS35L45_BST_BPE_INST_L3_ILIM_MASK, CS35L45_BST_BPE_INST_L3_ILIM_SHIFT },
	{ CS35L45_BST_BPE_INST_ILIM, CS35L45_BST_BPE_INST_L4_ILIM_MASK, CS35L45_BST_BPE_INST_L4_ILIM_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bst_bpe_inst_ss_ilim_lvl[] = {
	{ 0 },   /* hardware has no such field at this level */
	{ CS35L45_BST_BPE_INST_SS_ILIM, CS35L45_BST_BPE_INST_L1_SS_ILIM_MASK, CS35L45_BST_BPE_INST_L1_SS_ILIM_SHIFT },
	{ CS35L45_BST_BPE_INST_SS_ILIM, CS35L45_BST_BPE_INST_L2_SS_ILIM_MASK, CS35L45_BST_BPE_INST_L2_SS_ILIM_SHIFT },
	{ CS35L45_BST_BPE_INST_SS_ILIM, CS35L45_BST_BPE_INST_L3_SS_ILIM_MASK, CS35L45_BST_BPE_INST_L3_SS_ILIM_SHIFT },
	{ CS35L45_BST_BPE_INST_SS_ILIM, CS35L45_BST_BPE_INST_L4_SS_ILIM_MASK, CS35L45_BST_BPE_INST_L4_SS_ILIM_SHIFT },
};

static const struct cs35l45_bpe_field cs35l45_bst_bpe_inst_atk_rate_lvl[] = {
	{ 0 },   /* hardware has no such field at this level */
	{ CS35L45_BST_BPE_INST_ATK_RATE, CS35L45_BST_BPE_INST_L1_ATK_RATE_MASK, CS35L45_BST_BPE_INST_L1_ATK_RATE_SHIFT },
	{ CS35L45_BST_BPE_INST_ATK_RATE, CS35L45_BST_BPE_INST_L2_ATK_RATE_MASK, CS35L45_BST_BPE_INST_L2_ATK_RATE_SHIFT },
	{ CS35L45_BST_BPE_INST_ATK_RATE, CS35L45_BST_BPE_INST_L3_ATK_RATE_MASK, CS35L45_BST_BPE_INST_L3_ATK_RATE_SHIFT },
	{ 0 },   /* hardware has no such field at this level */
};

static const struct cs35l45_bpe_field cs35l45_bst_bpe_inst_hold_time_lvl[] = {
	{ CS35L45_BST_BPE_INST_HOLD_TIME, CS35L45_BST_BPE_INST_L0_HOLD_TIME_MASK, CS35L45_BST_BPE_INST_L0_HOLD_TIME_SHIFT },
	{ CS35L45_BST_BPE_INST_HOLD_TIME, CS35L45_BST_BPE_INST_L1_HOLD_TIME_MASK, CS35L45_BST_BPE_INST_L1_HOLD_TIME_SHIFT },
	{ CS35L45_BST_BPE_INST_HOLD_TIME, CS35L45_BST_BPE_INST_L2_HOLD_TIME_MASK, CS35L45_BST_BPE_INST_L2_HOLD_TIME_SHIFT },
	{ CS35L45_BST_BPE_INST_HOLD_TIME, CS35L45_BST_BPE_INST_L3_HOLD_TIME_MASK, CS35L45_BST_BPE_INST_L3_HOLD_TIME_SHIFT },
	{ 0 },   /* hardware has no such field at this level */
};

static const struct cs35l45_bpe_field cs35l45_bst_bpe_inst_rls_rate_lvl[] = {
	{ CS35L45_BST_BPE_INST_RLS_RATE, CS35L45_BST_BPE_INST_L0_RLS_RATE_MASK, CS35L45_BST_BPE_INST_L0_RLS_RATE_SHIFT },
	{ CS35L45_BST_BPE_INST_RLS_RATE, CS35L45_BST_BPE_INST_L1_RLS_RATE_MASK, CS35L45_BST_BPE_INST_L1_RLS_RATE_SHIFT },
	{ CS35L45_BST_BPE_INST_RLS_RATE, CS35L45_BST_BPE_INST_L2_RLS_RATE_MASK, CS35L45_BST_BPE_INST_L2_RLS_RATE_SHIFT },
	{ CS35L45_BST_BPE_INST_RLS_RATE, CS35L45_BST_BPE_INST_L3_RLS_RATE_MASK, CS35L45_BST_BPE_INST_L3_RLS_RATE_SHIFT },
	{ 0 },   /* hardware has no such field at this level */
};

static const struct cs35l45_bpe_level_prop cs35l45_bpe_inst_props[] = {
	{ "bpe-inst-thld",	4, cs35l45_bpe_inst_thld_lvl },
	{ "bpe-inst-attn",	4, cs35l45_bpe_inst_attn_lvl },
	{ "bpe-inst-atk-rate",	4, cs35l45_bpe_inst_atk_rate_lvl },
	{ "bpe-inst-hold-time",	4, cs35l45_bpe_inst_hold_time_lvl },
	{ "bpe-inst-rls-rate",	4, cs35l45_bpe_inst_rls_rate_lvl },
};

/*
 * 🔴 bpe-mode-sel and bpe-filt-sel are DELIBERATELY ABSENT from this table.
 *
 * This device's own device tree declares both of them inside
 * cirrus,bpe-misc-config -- but the vendor driver's bpe_misc_map looks them up
 * as "bst-bpe-mode-sel" and "bst-bpe-filt-sel", with a bst- prefix that belongs
 * to the SEPARATE cirrus,bst-bpe-misc-config node. Those names do not exist in
 * the node it searches, the lookups fail, and stock therefore NEVER WRITES
 * EITHER FIELD -- they sit at their reset value for the life of the device.
 *
 * Reading them here would be the obvious "correct" thing to do and would be a
 * silent DEVIATION FROM STOCK, which is the one thing this port must not be. Do
 * not "fix" this without first reading the two fields back from a live
 * amplifier and confirming what stock actually leaves them at.
 */
static const struct cs35l45_scalar_prop cs35l45_bpe_misc_props[] = {
	{ "bpe-inst-bpe-byp",	   CS35L45_BPE_MISC_CONFIG,
	  CS35L45_BPE_INST_BPE_BYP_MASK, CS35L45_BPE_INST_BPE_BYP_SHIFT },
	{ "bpe-inst-inf-hold-rls", CS35L45_BPE_MISC_CONFIG,
	  CS35L45_BPE_INST_INF_HOLD_RLS_MASK, CS35L45_BPE_INST_INF_HOLD_RLS_SHIFT },
	{ "bpe-inst-l1-byp",	   CS35L45_BPE_MISC_CONFIG,
	  CS35L45_BPE_INST_L1_BYP_MASK, CS35L45_BPE_INST_L1_BYP_SHIFT },
	{ "bpe-inst-l2-byp",	   CS35L45_BPE_MISC_CONFIG,
	  CS35L45_BPE_INST_L2_BYP_MASK, CS35L45_BPE_INST_L2_BYP_SHIFT },
	{ "bpe-inst-l3-byp",	   CS35L45_BPE_MISC_CONFIG,
	  CS35L45_BPE_INST_L3_BYP_MASK, CS35L45_BPE_INST_L3_BYP_SHIFT },
};

static const struct cs35l45_bpe_level_prop cs35l45_bst_bpe_inst_props[] = {
	{ "bst-bpe-inst-thld",	    5, cs35l45_bst_bpe_inst_thld_lvl },
	{ "bst-bpe-inst-ilim",	    5, cs35l45_bst_bpe_inst_ilim_lvl },
	{ "bst-bpe-inst-ss-ilim",   5, cs35l45_bst_bpe_inst_ss_ilim_lvl },
	{ "bst-bpe-inst-atk-rate",  5, cs35l45_bst_bpe_inst_atk_rate_lvl },
	{ "bst-bpe-inst-hold-time", 5, cs35l45_bst_bpe_inst_hold_time_lvl },
	{ "bst-bpe-inst-rls-rate",  5, cs35l45_bst_bpe_inst_rls_rate_lvl },
};

static const struct cs35l45_scalar_prop cs35l45_bst_bpe_misc_props[] = {
	{ "bst-bpe-inst-inf-hold-rls", CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_INST_INF_HOLD_RLS_MASK, CS35L45_BST_BPE_INST_INF_HOLD_RLS_SHIFT },
	{ "bst-bpe-il-lim-mode",       CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_IL_LIM_MODE_MASK, CS35L45_BST_BPE_IL_LIM_MODE_SHIFT },
	{ "bst-bpe-out-opmode-sel",    CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_OUT_OPMODE_SEL_MASK, CS35L45_BST_BPE_OUT_OPMODE_SEL_SHIFT },
	{ "bst-bpe-inst-l1-byp",       CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_INST_L1_BYP_MASK, CS35L45_BST_BPE_INST_L1_BYP_SHIFT },
	{ "bst-bpe-inst-l2-byp",       CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_INST_L2_BYP_MASK, CS35L45_BST_BPE_INST_L2_BYP_SHIFT },
	{ "bst-bpe-inst-l3-byp",       CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_INST_L3_BYP_MASK, CS35L45_BST_BPE_INST_L3_BYP_SHIFT },
	{ "bst-bpe-filt-sel",	       CS35L45_BST_BPE_MISC_CONFIG,
	  CS35L45_BST_BPE_FILT_SEL_MASK, CS35L45_BST_BPE_FILT_SEL_SHIFT },
};

static const struct cs35l45_scalar_prop cs35l45_bst_bpe_il_lim_props[] = {
	{ "bst-bpe-il-lim-thld-del1", CS35L45_BST_BPE_IL_LIM_THLD,
	  CS35L45_BST_BPE_IL_LIM_THLD_DEL1_MASK, CS35L45_BST_BPE_IL_LIM_THLD_DEL1_SHIFT },
	{ "bst-bpe-il-lim-thld-del2", CS35L45_BST_BPE_IL_LIM_THLD,
	  CS35L45_BST_BPE_IL_LIM_THLD_DEL2_MASK, CS35L45_BST_BPE_IL_LIM_THLD_DEL2_SHIFT },
	{ "bst-bpe-il-lim1-thld",     CS35L45_BST_BPE_IL_LIM_THLD,
	  CS35L45_BST_BPE_IL_LIM1_THLD_MASK, CS35L45_BST_BPE_IL_LIM1_THLD_SHIFT },
	{ "bst-bpe-il-lim-thld-hyst", CS35L45_BST_BPE_IL_LIM_THLD,
	  CS35L45_BST_BPE_IL_LIM_THLD_HYST_MASK, CS35L45_BST_BPE_IL_LIM_THLD_HYST_SHIFT },
	{ "bst-bpe-il-lim1-dly",      CS35L45_BST_BPE_IL_LIM_DLY,
	  CS35L45_BST_BPE_IL_LIM1_DLY_MASK, CS35L45_BST_BPE_IL_LIM1_DLY_SHIFT },
	{ "bst-bpe-il-lim2-dly",      CS35L45_BST_BPE_IL_LIM_DLY,
	  CS35L45_BST_BPE_IL_LIM2_DLY_MASK, CS35L45_BST_BPE_IL_LIM2_DLY_SHIFT },
	{ "bst-bpe-il-lim-dly-hyst",  CS35L45_BST_BPE_IL_LIM_DLY,
	  CS35L45_BST_BPE_IL_LIM_DLY_HYST_MASK, CS35L45_BST_BPE_IL_LIM_DLY_HYST_SHIFT },
};

#define CS35L45_BPE_MAX_LEVELS	5

static void cs35l45_apply_bpe_levels(struct cs35l45_private *cs35l45,
				     struct device_node *child,
				     const struct cs35l45_bpe_level_prop *props,
				     unsigned int nprops)
{
	u32 vals[CS35L45_BPE_MAX_LEVELS];
	unsigned int i, j;
	int ret;

	for (i = 0; i < nprops; i++) {
		/*
		 * A property that is absent, or whose array is the wrong
		 * length, is SKIPPED ENTIRELY rather than partially applied.
		 * of_property_read_u32_array() leaves vals[] untouched on
		 * error, so applying anything here would write stack garbage
		 * into a protection threshold.
		 */
		if (of_property_read_u32_array(child, props[i].name, vals,
					       props[i].levels))
			continue;

		for (j = 0; j < props[i].levels; j++) {
			if (!props[i].field[j].reg)
				continue;
			ret = regmap_update_bits(cs35l45->regmap,
						 props[i].field[j].reg,
						 props[i].field[j].mask,
						 vals[j] << props[i].field[j].shift);
			if (ret)
				dev_err(cs35l45->dev,
					"%pOFn/%s[%u]: write failed (%d)\n",
					child, props[i].name, j, ret);
		}
	}
}

static void cs35l45_apply_scalar_props(struct cs35l45_private *cs35l45,
				       struct device_node *child,
				       const struct cs35l45_scalar_prop *props,
				       unsigned int nprops)
{
	unsigned int i, val;
	int ret;

	for (i = 0; i < nprops; i++) {
		if (of_property_read_u32(child, props[i].name, &val))
			continue;
		ret = regmap_update_bits(cs35l45->regmap, props[i].reg,
					 props[i].mask, val << props[i].shift);
		if (ret)
			dev_err(cs35l45->dev, "%pOFn/%s: write failed (%d)\n",
				child, props[i].name, ret);
	}
}

/*
 * Each block is applied only if its node exists, so a device tree that
 * describes none of them behaves exactly as before this was added.
 *
 * The values are written once, here. They survive runtime suspend without any
 * further work: the registers are in the regmap cache (REGCACHE_MAPLE) and
 * cs35l45_runtime_resume() calls regcache_sync(). That is why they also had to
 * be added to cs35l45_readable_reg() -- they are not in cs35l45_defaults, so
 * regmap_update_bits() must read each one before it can modify it.
 */
static void cs35l45_apply_bpe_config(struct cs35l45_private *cs35l45,
				     struct device_node *node)
{
	struct device_node *child;

	child = of_get_child_by_name(node, "cirrus,bpe-inst-config");
	if (child) {
		cs35l45_apply_bpe_levels(cs35l45, child, cs35l45_bpe_inst_props,
					 ARRAY_SIZE(cs35l45_bpe_inst_props));
		of_node_put(child);
	}

	child = of_get_child_by_name(node, "cirrus,bpe-misc-config");
	if (child) {
		cs35l45_apply_scalar_props(cs35l45, child, cs35l45_bpe_misc_props,
					   ARRAY_SIZE(cs35l45_bpe_misc_props));
		cs35l45->bpe_misc_configured = true;
		cs35l45_pulse_bpe_misc_commit(cs35l45);
		of_node_put(child);
	}

	child = of_get_child_by_name(node, "cirrus,bst-bpe-inst-config");
	if (child) {
		cs35l45_apply_bpe_levels(cs35l45, child,
					 cs35l45_bst_bpe_inst_props,
					 ARRAY_SIZE(cs35l45_bst_bpe_inst_props));
		of_node_put(child);
	}

	child = of_get_child_by_name(node, "cirrus,bst-bpe-misc-config");
	if (child) {
		cs35l45_apply_scalar_props(cs35l45, child,
					   cs35l45_bst_bpe_misc_props,
					   ARRAY_SIZE(cs35l45_bst_bpe_misc_props));
		of_node_put(child);
	}

	child = of_get_child_by_name(node, "cirrus,bst-bpe-il-lim-config");
	if (child) {
		cs35l45_apply_scalar_props(cs35l45, child,
					   cs35l45_bst_bpe_il_lim_props,
					   ARRAY_SIZE(cs35l45_bst_bpe_il_lim_props));
		of_node_put(child);
	}
}

/*
 * HVLV -- the high-voltage/low-voltage supply mode.
 *
 * The amplifier can run from the boost rail or from the battery directly, and
 * HVLV decides where the crossover sits: a threshold, a hysteresis around it
 * and a transition delay. cs35l45_amplifier_mode_put() already writes the MODE
 * field of this register; the three DT-supplied fields around it were not read
 * at all, so the part crossed over at its power-on threshold rather than the
 * one this device is tuned for.
 *
 * Stock declares all three and the vendor driver reads all three under exactly
 * these names, so unlike the BPE misc block there is nothing to reproduce here
 * but the values themselves.
 */
static const struct cs35l45_scalar_prop cs35l45_hvlv_props[] = {
	{ "hvlv-thld-hys",	  CS35L45_HVLV_CONFIG,
	  CS35L45_HVLV_THLD_HYS_MASK, CS35L45_HVLV_THLD_HYS_SHIFT },
	{ "hvlv-thld",		  CS35L45_HVLV_CONFIG,
	  CS35L45_HVLV_THLD_MASK, CS35L45_HVLV_THLD_SHIFT },
	{ "hvlv-dly",		  CS35L45_HVLV_CONFIG,
	  CS35L45_HVLV_DLY_MASK, CS35L45_HVLV_DLY_SHIFT },
};

/*
 * ---------------------------------------------------------------------------
 * Low-power mode (LDPM) and Class-H envelope tracking.
 *
 * LDPM decides which blocks keep running once the PCM input goes quiet. There
 * are two independent groups, each with its own signal threshold and entry
 * delay: group 1 covers the amplifier and the boost converter, group 2 covers
 * the VMON and IMON sense paths.
 *
 * Class-H moves the boost target with the audio envelope instead of holding
 * the rail at its maximum, and the OVB thresholds decide how far ahead of the
 * signal the rail is allowed to sit.
 *
 * This device's stock firmware programs both, identically on all four
 * amplifiers; mainline programmed neither. Same shape as the BPE
 * transcription above -- the vendor's mechanism is ported, the numbers stay
 * in the device tree.
 *
 * The tables below list the properties in the same order as the vendor's
 * ldpm_map[] and classh_map[], so the two can be compared line for line.
 * ---------------------------------------------------------------------------
 */
static const struct cs35l45_scalar_prop cs35l45_ldpm_props[] = {
	{ "ldpm-gp1-boost-sel",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP1_BOOST_SEL_MASK, CS35L45_LDPM_GP1_BOOST_SEL_SHIFT },
	{ "ldpm-gp1-amp-sel",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP1_AMP_SEL_MASK, CS35L45_LDPM_GP1_AMP_SEL_SHIFT },
	{ "ldpm-gp1-delay",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP1_DELAY_MASK, CS35L45_LDPM_GP1_DELAY_SHIFT },
	{ "ldpm-gp1-pcm-thld",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP1_PCM_THLD_MASK, CS35L45_LDPM_GP1_PCM_THLD_SHIFT },
	{ "ldpm-gp2-imon-sel",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP2_IMON_SEL_MASK, CS35L45_LDPM_GP2_IMON_SEL_SHIFT },
	{ "ldpm-gp2-vmon-sel",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP2_VMON_SEL_MASK, CS35L45_LDPM_GP2_VMON_SEL_SHIFT },
	{ "ldpm-gp2-delay",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP2_DELAY_MASK, CS35L45_LDPM_GP2_DELAY_SHIFT },
	{ "ldpm-gp2-pcm-thld",	  CS35L45_LDPM_CONFIG,
	  CS35L45_LDPM_GP2_PCM_THLD_MASK, CS35L45_LDPM_GP2_PCM_THLD_SHIFT },
};

static const struct cs35l45_scalar_prop cs35l45_classh_props[] = {
	/*
	 * "ch-hdrm" is kept here so this table matches the vendor's
	 * classh_map[] entry for entry, but this device's device tree does
	 * not declare it -- deliberately, because stock's does not either.
	 * See the comment on the Class-H node in the device tree.
	 */
	{ "ch-hdrm",		  CS35L45_CLASSH_CONFIG1,
	  CS35L45_CH_HDRM_MASK, CS35L45_CH_HDRM_SHIFT },
	{ "ch-ratio",		  CS35L45_CLASSH_CONFIG1,
	  CS35L45_CH_RATIO_MASK, CS35L45_CH_RATIO_SHIFT },
	{ "ch-rel-rate",	  CS35L45_CLASSH_CONFIG1,
	  CS35L45_CH_REL_RATE_MASK, CS35L45_CH_REL_RATE_SHIFT },
	{ "ch-ovb-thld1",	  CS35L45_CLASSH_CONFIG2,
	  CS35L45_CH_OVB_THLD1_MASK, CS35L45_CH_OVB_THLD1_SHIFT },
	{ "ch-ovb-thlddelta",	  CS35L45_CLASSH_CONFIG2,
	  CS35L45_CH_OVB_THLDDELTA_MASK, CS35L45_CH_OVB_THLDDELTA_SHIFT },
	{ "ch-vdd-bst-max",	  CS35L45_CLASSH_CONFIG2,
	  CS35L45_CH_VDD_BST_MAX_MASK, CS35L45_CH_VDD_BST_MAX_SHIFT },
	{ "ch-ovb-ratio",	  CS35L45_CLASSH_CONFIG3,
	  CS35L45_CH_OVB_RATIO_MASK, CS35L45_CH_OVB_RATIO_SHIFT },
	{ "ch-thld1-offset",	  CS35L45_CLASSH_CONFIG3,
	  CS35L45_CH_THLD1_OFFSET_MASK, CS35L45_CH_THLD1_OFFSET_SHIFT },
};

/*
 * The vendor drives CH_OVB_LATCH high and then low immediately after writing
 * the Class-H configuration, and never at any other time. The bit is named for
 * latching the OVB thresholds; what it does inside the block has not been
 * measured here, so this reproduces the vendor's sequence rather than an
 * explanation of it.
 *
 * It cannot be left to the regmap cache. A pulse's resting state is the value
 * the cache keeps -- zero -- so regcache_sync() restores the configuration
 * without ever re-issuing the pulse. cs35l45_runtime_suspend() hibernates the
 * part, so that path is real and not theoretical; the ERROR_RELEASE pulse in
 * cs35l45_runtime_resume() is the same shape and is handled the same way.
 */
static void cs35l45_pulse_classh_ovb_latch(struct cs35l45_private *cs35l45)
{
	regmap_update_bits(cs35l45->regmap, CS35L45_CLASSH_CONFIG3,
			   CS35L45_CH_OVB_LATCH_MASK,
			   CS35L45_CH_OVB_LATCH_MASK);
	regmap_update_bits(cs35l45->regmap, CS35L45_CLASSH_CONFIG3,
			   CS35L45_CH_OVB_LATCH_MASK, 0);
}

/*
 * Reproduce the vendor's BPE_MISC_CONFIG write sequence, which puts bit 15 on
 * the bus along with the device-tree fields.
 *
 * WHY THIS IS A PULSE AND NOT A PLAIN SET. The two are indistinguishable in
 * the register and very different in the cache.
 *
 * Bit 15 is undocumented at this address. Two hypotheses survive every reading
 * that can be taken, and no register read can separate them, because a
 * self-clearing latch the part CONSUMES and an unimplemented bit the part DROPS
 * read back identically:
 *
 *   unimplemented  the part ignores it, the write is inert, and reproducing
 *                  stock's sequence costs one I2C transaction.
 *   a latch        the part consumes it and clears it -- exactly what BIT(15)
 *                  does at WAKESRC_CTL and WKI2C_CTL in this same part, where
 *                  it is documented as UPDT_WKCTL and UPDT_WKI2C. Then the
 *                  write is what COMMITS this block's configuration, and
 *                  omitting it leaves the fields written but never applied.
 *
 * The stakes are asymmetric: inert under the first, corrective under the
 * second, and undetectable either way. So issue it.
 *
 * Session 329 added this as a plain regmap_set_bits() and it was reverted,
 * correctly, for a reason that has nothing to do with the hardware: this
 * project reads the regmap debugfs "registers" file as primary evidence, and
 * a non-volatile register is served from the cache. A cache asserting 0x8600
 * over silicon holding 0x0600 makes every future capture of this register wrong
 * in a way that looks like data.
 *
 * The pulse satisfies both. The set writes 0x8600 to the part; the clear writes
 * 0x0600 and leaves the cache holding 0x0600, which is what the part holds --
 * measured under cache_bypass on all four amplifiers, on stock and here alike.
 * Dropping the bit afterwards cannot diverge from stock, because stock's part
 * does not hold it either.
 *
 * Marking the register volatile instead would ALSO keep the cache honest, and
 * it would be a regression: a volatile register is not cached, so
 * regcache_sync() would stop replaying the device-tree fields and the block
 * would be left unconfigured after the first hibernate.
 *
 * And like the Class-H latch above, a pulse cannot be replayed from a cache --
 * its resting state is zero -- so cs35l45_runtime_resume() re-issues it.
 */
static void cs35l45_pulse_bpe_misc_commit(struct cs35l45_private *cs35l45)
{
	regmap_update_bits(cs35l45->regmap, CS35L45_BPE_MISC_CONFIG,
			   CS35L45_BPE_MISC_CONFIG_UNNAMED_BIT15,
			   CS35L45_BPE_MISC_CONFIG_UNNAMED_BIT15);
	regmap_update_bits(cs35l45->regmap, CS35L45_BPE_MISC_CONFIG,
			   CS35L45_BPE_MISC_CONFIG_UNNAMED_BIT15, 0);
}

/*
 * ---------------------------------------------------------------------------
 * Noise gate.
 *
 * The gate mutes the mixer channel once the signal sits below a threshold for
 * a hold time. This device's stock device tree declares a threshold and a hold
 * for both channels, identically on all four amplifiers, and the vendor driver
 * reads all four under exactly these names -- checked in both directions, 4 for
 * 4, so unlike the BPE misc block there is no name mismatch to reproduce here.
 *
 * 🔴 THE GATE ITSELF IS LEFT OFF, AND THAT IS THE POINT. The enable is bit 16
 * of the same register, and neither driver writes it from the device tree: the
 * vendor drives it from a "NGATE1/2 Enable" DAPM switch whose kcontrol defaults
 * to off, and stock's mixer state has all eight of those switches Off, in idle
 * and during playback alike. So porting the DAPM widgets, the source enums, the
 * enable switches and the routes would add a control surface that stock does
 * not use, and would be the deviation -- not the omission. Do not "complete"
 * this block by adding them.
 *
 * What this does change: the register goes from its power-on 0x00000303 to
 * stock's 0x00000206, with the gate still disabled on both sides.
 *
 * These properties sit DIRECTLY on the amplifier node, not in a child node like
 * the BPE, HVLV, LDPM and Class-H blocks, so the names here carry the "cirrus,"
 * prefix and the node passed to cs35l45_apply_scalar_props() is the amplifier
 * itself.
 * ---------------------------------------------------------------------------
 */
static const struct cs35l45_scalar_prop cs35l45_ngate_props[] = {
	{ "cirrus,ngate-ch1-hold", CS35L45_MIXER_NGATE_CH1_CFG,
	  CS35L45_AUX_NGATE_CH_HOLD_MASK, CS35L45_AUX_NGATE_CH_HOLD_SHIFT },
	{ "cirrus,ngate-ch1-thr",  CS35L45_MIXER_NGATE_CH1_CFG,
	  CS35L45_AUX_NGATE_CH_THR_MASK, CS35L45_AUX_NGATE_CH_THR_SHIFT },
	{ "cirrus,ngate-ch2-hold", CS35L45_MIXER_NGATE_CH2_CFG,
	  CS35L45_AUX_NGATE_CH_HOLD_MASK, CS35L45_AUX_NGATE_CH_HOLD_SHIFT },
	{ "cirrus,ngate-ch2-thr",  CS35L45_MIXER_NGATE_CH2_CFG,
	  CS35L45_AUX_NGATE_CH_THR_MASK, CS35L45_AUX_NGATE_CH_THR_SHIFT },
};

static void cs35l45_apply_power_mode_config(struct cs35l45_private *cs35l45,
					   struct device_node *node)
{
	struct device_node *child;

	child = of_get_child_by_name(node, "cirrus,hvlv-config");
	if (child) {
		cs35l45_apply_scalar_props(cs35l45, child, cs35l45_hvlv_props,
					   ARRAY_SIZE(cs35l45_hvlv_props));
		of_node_put(child);
	}

	child = of_get_child_by_name(node, "cirrus,ldpm-config");
	if (child) {
		cs35l45_apply_scalar_props(cs35l45, child, cs35l45_ldpm_props,
					   ARRAY_SIZE(cs35l45_ldpm_props));
		of_node_put(child);
	}

	child = of_get_child_by_name(node, "cirrus,classh-config");
	if (child) {
		cs35l45_apply_scalar_props(cs35l45, child,
					   cs35l45_classh_props,
					   ARRAY_SIZE(cs35l45_classh_props));
		of_node_put(child);

		cs35l45->classh_configured = true;
		cs35l45_pulse_classh_ovb_latch(cs35l45);
	}

	/*
	 * The noise-gate properties live on the amplifier node itself, so the
	 * node is passed straight through. A device tree that declares none of
	 * them leaves both registers at their power-on value, exactly as before.
	 */
	cs35l45_apply_scalar_props(cs35l45, node, cs35l45_ngate_props,
				   ARRAY_SIZE(cs35l45_ngate_props));
}

static int cs35l45_apply_property_config(struct cs35l45_private *cs35l45)
{
	struct device_node *node = cs35l45->dev->of_node;
	unsigned int gpio_regs[] = {CS35L45_GPIO1_CTRL1, CS35L45_GPIO2_CTRL1,
				    CS35L45_GPIO3_CTRL1};
	unsigned int pad_regs[] = {CS35L45_SYNC_GPIO1,
				   CS35L45_INTB_GPIO2_MCLK_REF, CS35L45_GPIO3};
	struct device_node *child;
	unsigned int val;
	char of_name[32];
	int ret, i;

	if (!node)
		return 0;

	for (i = 0; i < CS35L45_NUM_GPIOS; i++) {
		sprintf(of_name, "cirrus,gpio-ctrl%d", i + 1);
		child = of_get_child_by_name(node, of_name);
		if (!child)
			continue;

		ret = of_property_read_u32(child, "gpio-dir", &val);
		if (!ret)
			regmap_update_bits(cs35l45->regmap, gpio_regs[i],
					   CS35L45_GPIO_DIR_MASK,
					   val << CS35L45_GPIO_DIR_SHIFT);

		ret = of_property_read_u32(child, "gpio-lvl", &val);
		if (!ret)
			regmap_update_bits(cs35l45->regmap, gpio_regs[i],
					   CS35L45_GPIO_LVL_MASK,
					   val << CS35L45_GPIO_LVL_SHIFT);

		ret = of_property_read_u32(child, "gpio-op-cfg", &val);
		if (!ret)
			regmap_update_bits(cs35l45->regmap, gpio_regs[i],
					   CS35L45_GPIO_OP_CFG_MASK,
					   val << CS35L45_GPIO_OP_CFG_SHIFT);

		ret = of_property_read_u32(child, "gpio-pol", &val);
		if (!ret)
			regmap_update_bits(cs35l45->regmap, gpio_regs[i],
					   CS35L45_GPIO_POL_MASK,
					   val << CS35L45_GPIO_POL_SHIFT);

		ret = of_property_read_u32(child, "gpio-ctrl", &val);
		if (!ret)
			regmap_update_bits(cs35l45->regmap, pad_regs[i],
					   CS35L45_GPIO_CTRL_MASK,
					   val << CS35L45_GPIO_CTRL_SHIFT);

		ret = of_property_read_u32(child, "gpio-invert", &val);
		if (!ret) {
			regmap_update_bits(cs35l45->regmap, pad_regs[i],
					   CS35L45_GPIO_INVERT_MASK,
					   val << CS35L45_GPIO_INVERT_SHIFT);
			if (i == 1)
				cs35l45->irq_invert = val;
		}

		of_node_put(child);
	}

	if (device_property_read_u32(cs35l45->dev,
				     "cirrus,asp-sdout-hiz-ctrl", &val) == 0) {
		regmap_update_bits(cs35l45->regmap, CS35L45_ASP_CONTROL3,
				   CS35L45_ASP_DOUT_HIZ_CTRL_MASK,
				   val << CS35L45_ASP_DOUT_HIZ_CTRL_SHIFT);
	}

	cs35l45_apply_bpe_config(cs35l45, node);
	cs35l45_apply_power_mode_config(cs35l45, node);

	return 0;
}

static int cs35l45_dsp_virt2_mbox3_irq_handle(struct cs35l45_private *cs35l45,
					      const unsigned int cmd,
					      unsigned int data)
{
	static char *speak_status = "Unknown";

	switch (cmd) {
	case EVENT_SPEAKER_STATUS:
		switch (data) {
		case 1:
			speak_status = "All Clear";
			break;
		case 2:
			speak_status = "Open Circuit";
			break;
		case 4:
			speak_status = "Short Circuit";
			break;
		}

		dev_info(cs35l45->dev, "MBOX event (SPEAKER_STATUS): %s\n",
			 speak_status);
		break;
	case EVENT_BOOT_DONE:
		dev_dbg(cs35l45->dev, "MBOX event (BOOT_DONE)\n");
		break;
	default:
		dev_err(cs35l45->dev, "MBOX event not supported %u\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t cs35l45_dsp_virt2_mbox_cb(int irq, void *data)
{
	struct cs35l45_private *cs35l45 = data;
	unsigned int mbox_val;
	int ret = 0;

	ret = regmap_read(cs35l45->regmap, CS35L45_DSP_VIRT2_MBOX_3, &mbox_val);
	if (!ret && mbox_val)
		cs35l45_dsp_virt2_mbox3_irq_handle(cs35l45, mbox_val & CS35L45_MBOX3_CMD_MASK,
				(mbox_val & CS35L45_MBOX3_DATA_MASK) >> CS35L45_MBOX3_DATA_SHIFT);

	/* Handle DSP trace log IRQ */
	ret = regmap_read(cs35l45->regmap, CS35L45_DSP_VIRT2_MBOX_4, &mbox_val);
	if (!ret && mbox_val != 0) {
		dev_err(cs35l45->dev, "Spurious DSP MBOX4 IRQ\n");
	}

	return IRQ_RETVAL(ret);
}

static irqreturn_t cs35l45_pll_unlock(int irq, void *data)
{
	struct cs35l45_private *cs35l45 = data;

	dev_dbg(cs35l45->dev, "PLL unlock detected!");

	return IRQ_HANDLED;
}

static irqreturn_t cs35l45_pll_lock(int irq, void *data)
{
	struct cs35l45_private *cs35l45 = data;

	dev_dbg(cs35l45->dev, "PLL lock detected!");

	return IRQ_HANDLED;
}

static irqreturn_t cs35l45_spk_safe_err(int irq, void *data);

static const struct cs35l45_irq cs35l45_irqs[] = {
	CS35L45_IRQ(AMP_SHORT_ERR, "Amplifier short error", cs35l45_spk_safe_err),
	CS35L45_IRQ(UVLO_VDDBATT_ERR, "VDDBATT undervoltage error", cs35l45_spk_safe_err),
	CS35L45_IRQ(BST_SHORT_ERR, "Boost inductor error", cs35l45_spk_safe_err),
	CS35L45_IRQ(BST_UVP_ERR, "Boost undervoltage error", cs35l45_spk_safe_err),
	CS35L45_IRQ(TEMP_ERR, "Overtemperature error", cs35l45_spk_safe_err),
	CS35L45_IRQ(AMP_CAL_ERR, "Amplifier calibration error", cs35l45_spk_safe_err),
	CS35L45_IRQ(UVLO_VDDLV_ERR, "LV threshold detector error", cs35l45_spk_safe_err),
	CS35L45_IRQ(GLOBAL_ERROR, "Global error", cs35l45_spk_safe_err),
	CS35L45_IRQ(DSP_WDT_EXPIRE, "DSP Watchdog Timer", cs35l45_spk_safe_err),
	CS35L45_IRQ(PLL_UNLOCK_FLAG_RISE, "PLL unlock", cs35l45_pll_unlock),
	CS35L45_IRQ(PLL_LOCK_FLAG, "PLL lock", cs35l45_pll_lock),
	CS35L45_IRQ(DSP_VIRT2_MBOX, "DSP virtual MBOX 2 write flag", cs35l45_dsp_virt2_mbox_cb),
};

static irqreturn_t cs35l45_spk_safe_err(int irq, void *data)
{
	struct cs35l45_private *cs35l45 = data;
	int i;

	i = irq - regmap_irq_get_virq(cs35l45->irq_data, 0);

	if (i < 0 || i >= ARRAY_SIZE(cs35l45_irqs))
		dev_err(cs35l45->dev, "Unspecified global error condition (%d) detected!\n", irq);
	else
		dev_err(cs35l45->dev, "%s condition detected!\n", cs35l45_irqs[i].name);

	return IRQ_HANDLED;
}

static const struct regmap_irq cs35l45_reg_irqs[] = {
	CS35L45_REG_IRQ(IRQ1_EINT_1, AMP_SHORT_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_1, UVLO_VDDBATT_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_1, BST_SHORT_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_1, BST_UVP_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_1, TEMP_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_3, AMP_CAL_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_18, UVLO_VDDLV_ERR),
	CS35L45_REG_IRQ(IRQ1_EINT_18, GLOBAL_ERROR),
	CS35L45_REG_IRQ(IRQ1_EINT_2, DSP_WDT_EXPIRE),
	CS35L45_REG_IRQ(IRQ1_EINT_3, PLL_UNLOCK_FLAG_RISE),
	CS35L45_REG_IRQ(IRQ1_EINT_3, PLL_LOCK_FLAG),
	CS35L45_REG_IRQ(IRQ1_EINT_2, DSP_VIRT2_MBOX),
};

static const struct regmap_irq_chip cs35l45_regmap_irq_chip = {
	.name = "cs35l45 IRQ1 Controller",
	.main_status = CS35L45_IRQ1_STATUS,
	.status_base = CS35L45_IRQ1_EINT_1,
	.mask_base = CS35L45_IRQ1_MASK_1,
	.ack_base = CS35L45_IRQ1_EINT_1,
	.num_regs = 18,
	.irqs = cs35l45_reg_irqs,
	.num_irqs = ARRAY_SIZE(cs35l45_reg_irqs),
	.runtime_pm = true,
};

static int cs35l45_initialize(struct cs35l45_private *cs35l45)
{
	struct device *dev = cs35l45->dev;
	unsigned int dev_id[5];
	unsigned int sts;
	int ret;

	ret = regmap_read_poll_timeout(cs35l45->regmap, CS35L45_IRQ1_EINT_4, sts,
				       (sts & CS35L45_OTP_BOOT_DONE_STS_MASK),
				       1000, 5000);
	if (ret < 0) {
		dev_err(cs35l45->dev, "Timeout waiting for OTP boot\n");
		return ret;
	}

	ret = regmap_bulk_read(cs35l45->regmap, CS35L45_DEVID, dev_id, ARRAY_SIZE(dev_id));
	if (ret) {
		dev_err(cs35l45->dev, "Get Device ID failed: %d\n", ret);
		return ret;
	}

	switch (dev_id[0]) {
	case 0x35A450:
	case 0x35A460:
		break;
	default:
		dev_err(cs35l45->dev, "Bad DEVID 0x%x\n", dev_id[0]);
		return -ENODEV;
	}

	dev_info(cs35l45->dev, "Cirrus Logic CS35L45: REVID %02X OTPID %02X\n",
		 dev_id[1], dev_id[4]);

	regmap_write(cs35l45->regmap, CS35L45_IRQ1_EINT_4,
		     CS35L45_OTP_BOOT_DONE_STS_MASK | CS35L45_OTP_BUSY_MASK);

	ret = cs35l45_apply_patch(cs35l45);
	if (ret < 0) {
		dev_err(dev, "Failed to apply init patch %d\n", ret);
		return ret;
	}

	ret = cs35l45_apply_property_config(cs35l45);
	if (ret < 0)
		return ret;

	cs35l45_parse_calibration(cs35l45);

	cs35l45->amplifier_mode = AMP_MODE_SPK;

	return 0;
}

static const struct reg_sequence cs35l45_fs_errata_patch[] = {
	{0x02B80080,			0x00000001},
	{0x02B80088,			0x00000001},
	{0x02B80090,			0x00000001},
	{0x02B80098,			0x00000001},
	{0x02B800A0,			0x00000001},
	{0x02B800A8,			0x00000001},
	{0x02B800B0,			0x00000001},
	{0x02B800B8,			0x00000001},
	{0x02B80280,			0x00000001},
	{0x02B80288,			0x00000001},
	{0x02B80290,			0x00000001},
	{0x02B80298,			0x00000001},
	{0x02B802A0,			0x00000001},
	{0x02B802A8,			0x00000001},
	{0x02B802B0,			0x00000001},
	{0x02B802B8,			0x00000001},
};

static const struct cs_dsp_region cs35l45_dsp1_regions[] = {
	{ .type = WMFW_HALO_PM_PACKED,	.base = CS35L45_DSP1_PMEM_0 },
	{ .type = WMFW_HALO_XM_PACKED,	.base = CS35L45_DSP1_XMEM_PACK_0 },
	{ .type = WMFW_HALO_YM_PACKED,	.base = CS35L45_DSP1_YMEM_PACK_0 },
	{. type = WMFW_ADSP2_XM,	.base = CS35L45_DSP1_XMEM_UNPACK24_0},
	{. type = WMFW_ADSP2_YM,	.base = CS35L45_DSP1_YMEM_UNPACK24_0},
};

static int cs35l45_dsp_init(struct cs35l45_private *cs35l45)
{
	struct wm_adsp *dsp = &cs35l45->dsp;
	int ret;

	dsp->part = "cs35l45";
	dsp->fw = 9; /* 9 is WM_ADSP_FW_SPK_PROT in wm_adsp.c */
	dsp->toggle_preload = true;
	dsp->cs_dsp.num = 1;
	dsp->cs_dsp.type = WMFW_HALO;
	dsp->cs_dsp.rev = 0;
	dsp->cs_dsp.dev = cs35l45->dev;
	dsp->cs_dsp.regmap = cs35l45->regmap;
	dsp->cs_dsp.base = CS35L45_DSP1_CLOCK_FREQ;
	dsp->cs_dsp.base_sysinfo = CS35L45_DSP1_SYS_ID;
	dsp->cs_dsp.mem = cs35l45_dsp1_regions;
	dsp->cs_dsp.num_mems = ARRAY_SIZE(cs35l45_dsp1_regions);
	dsp->cs_dsp.lock_regions = 0xFFFFFFFF;

	ret = wm_halo_init(dsp);

	regmap_multi_reg_write(cs35l45->regmap, cs35l45_fs_errata_patch,
						   ARRAY_SIZE(cs35l45_fs_errata_patch));

	return ret;
}

int cs35l45_probe(struct cs35l45_private *cs35l45)
{
	struct device *dev = cs35l45->dev;
	unsigned long irq_pol = IRQF_ONESHOT | IRQF_SHARED;
	int ret, i, irq;

	cs35l45->vdd_batt = devm_regulator_get(dev, "vdd-batt");
	if (IS_ERR(cs35l45->vdd_batt))
		return dev_err_probe(dev, PTR_ERR(cs35l45->vdd_batt),
				     "Failed to request vdd-batt\n");

	cs35l45->vdd_a = devm_regulator_get(dev, "vdd-a");
	if (IS_ERR(cs35l45->vdd_a))
		return dev_err_probe(dev, PTR_ERR(cs35l45->vdd_a),
				     "Failed to request vdd-a\n");

	/* VDD_BATT must always be enabled before other supplies */
	ret = regulator_enable(cs35l45->vdd_batt);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to enable vdd-batt\n");

	ret = regulator_enable(cs35l45->vdd_a);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to enable vdd-a\n");

	/* If reset is shared only one instance can claim it */
	cs35l45->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(cs35l45->reset_gpio)) {
		ret = PTR_ERR(cs35l45->reset_gpio);
		cs35l45->reset_gpio = NULL;
		if (ret == -EBUSY) {
			dev_dbg(dev, "Reset line busy, assuming shared reset\n");
		} else {
			dev_err_probe(dev, ret, "Failed to get reset GPIO\n");
			goto err;
		}
	}

	if (cs35l45->reset_gpio) {
		usleep_range(CS35L45_RESET_HOLD_US, CS35L45_RESET_HOLD_US + 100);
		gpiod_set_value_cansleep(cs35l45->reset_gpio, 1);
	}

	usleep_range(CS35L45_RESET_US, CS35L45_RESET_US + 100);

	ret = cs35l45_initialize(cs35l45);
	if (ret < 0)
		goto err_reset;

	ret = cs35l45_dsp_init(cs35l45);
	if (ret < 0)
		goto err_reset;

	pm_runtime_set_autosuspend_delay(cs35l45->dev, 3000);
	pm_runtime_use_autosuspend(cs35l45->dev);
	pm_runtime_set_active(cs35l45->dev);
	pm_runtime_get_noresume(cs35l45->dev);
	pm_runtime_enable(cs35l45->dev);

	if (cs35l45->irq) {
		if (cs35l45->irq_invert)
			irq_pol |= IRQF_TRIGGER_HIGH;
		else
			irq_pol |= IRQF_TRIGGER_LOW;

		ret = devm_regmap_add_irq_chip(dev, cs35l45->regmap, cs35l45->irq, irq_pol, 0,
					       &cs35l45_regmap_irq_chip, &cs35l45->irq_data);
		if (ret) {
			dev_err(dev, "Failed to register IRQ chip: %d\n", ret);
			goto err_dsp;
		}

		for (i = 0; i < ARRAY_SIZE(cs35l45_irqs); i++) {
			irq = regmap_irq_get_virq(cs35l45->irq_data, cs35l45_irqs[i].irq);
			if (irq < 0) {
				dev_err(dev, "Failed to get %s\n", cs35l45_irqs[i].name);
				ret = irq;
				goto err_dsp;
			}

			ret = devm_request_threaded_irq(dev, irq, NULL, cs35l45_irqs[i].handler,
							irq_pol, cs35l45_irqs[i].name, cs35l45);
			if (ret) {
				dev_err(dev, "Failed to request IRQ %s: %d\n",
					cs35l45_irqs[i].name, ret);
				goto err_dsp;
			}
		}
	}

	ret = devm_snd_soc_register_component(dev, &cs35l45_component,
					      cs35l45_dai,
					      ARRAY_SIZE(cs35l45_dai));
	if (ret < 0)
		goto err_dsp;

	pm_runtime_put_autosuspend(cs35l45->dev);

	return 0;

err_dsp:
	pm_runtime_disable(cs35l45->dev);
	pm_runtime_put_noidle(cs35l45->dev);
	wm_adsp2_remove(&cs35l45->dsp);

err_reset:
	gpiod_set_value_cansleep(cs35l45->reset_gpio, 0);
err:
	regulator_disable(cs35l45->vdd_a);
	regulator_disable(cs35l45->vdd_batt);

	return ret;
}
EXPORT_SYMBOL_NS_GPL(cs35l45_probe, "SND_SOC_CS35L45");

void cs35l45_remove(struct cs35l45_private *cs35l45)
{
	pm_runtime_get_sync(cs35l45->dev);
	pm_runtime_disable(cs35l45->dev);
	wm_adsp2_remove(&cs35l45->dsp);

	gpiod_set_value_cansleep(cs35l45->reset_gpio, 0);

	pm_runtime_put_noidle(cs35l45->dev);
	regulator_disable(cs35l45->vdd_a);
	/* VDD_BATT must be the last to power-off */
	regulator_disable(cs35l45->vdd_batt);
}
EXPORT_SYMBOL_NS_GPL(cs35l45_remove, "SND_SOC_CS35L45");

EXPORT_GPL_DEV_PM_OPS(cs35l45_pm_ops) = {
	RUNTIME_PM_OPS(cs35l45_runtime_suspend, cs35l45_runtime_resume, NULL)

	SYSTEM_SLEEP_PM_OPS(cs35l45_sys_suspend, cs35l45_sys_resume)
	NOIRQ_SYSTEM_SLEEP_PM_OPS(cs35l45_sys_suspend_noirq, cs35l45_sys_resume_noirq)
};

MODULE_DESCRIPTION("ASoC CS35L45 driver");
MODULE_AUTHOR("James Schulman, Cirrus Logic Inc, <james.schulman@cirrus.com>");
MODULE_AUTHOR("Richard Fitzgerald <rf@opensource.cirrus.com>");
MODULE_LICENSE("GPL");
