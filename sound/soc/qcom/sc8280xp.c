// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "sdw.h"

/*
 * TDM frame, overridable at boot.
 *
 * The defaults are what stock drives this hardware with: four 32-bit slots
 * carrying 24-bit samples.  They are parameters rather than constants because
 * the ADSP asserts on its own heap when handed some TDM configurations, and
 * finding which value it objects to means sweeping them -- one rebuild and
 * flash per combination is minutes each, while a module parameter is a reboot.
 *
 * Both arms of any comparison then share one binary, which is the point: a
 * sweep where each step is a different build cannot tell a value's effect from
 * a build's.
 */
static int tdm_slots = 4;
module_param(tdm_slots, int, 0444);
MODULE_PARM_DESC(tdm_slots, "TDM slots per frame (default 4, as stock)");

/*
 * CS35L45 "Digital PCM Volume", in ALSA units, at exactly 0.00 dB.
 *
 * The control is SOC_SINGLE_S_TLV(..., -409, 48, ...) over a -102.25 dB scale
 * with 0.25 dB steps, so ALSA value v is (-102.25 + v * 0.25) dB: 409 is unity
 * and 457 -- the unlimited maximum -- is +12.00 dB.
 */
#define CS35L45_DIG_PCM_VOL_0DB		409

static int tdm_slot_width = 32;
module_param(tdm_slot_width, int, 0444);
MODULE_PARM_DESC(tdm_slot_width, "TDM slot width in bits (default 32, as stock)");

static int tdm_channels = 4;
module_param(tdm_channels, int, 0444);
MODULE_PARM_DESC(tdm_channels, "channels on a TDM backend (default 4, as stock)");

static int tdm_sample_bits = 24;
module_param(tdm_sample_bits, int, 0444);
MODULE_PARM_DESC(tdm_sample_bits, "sample width on a TDM backend, 16 or 24 (default 24, as stock)");

/*
 * Digital microphones on a VA-macro backend.
 *
 * Stock records from THREE microphones.  Its vendor mixer config gives every
 * ordinary capture path -- recording, calls, VoIP -- the same shape: enable
 * three decimators and set three gains, DEC0, DEC1 and DEC2.  It also defines a
 * fourth microphone and never uses it: that path selects DMIC7, which lives on
 * a pin pair stock's own device tree leaves disabled.
 *
 * A parameter rather than a constant for the same reason as the TDM ones above:
 * a channel count is exactly the kind of value where a wrong guess produces a
 * capture that opens, runs and contains silence in the extra channel, so both
 * arms of a comparison should share one binary.
 */
static int dmic_channels = 3;
module_param(dmic_channels, int, 0444);
MODULE_PARM_DESC(dmic_channels, "max channels on a digital-microphone backend (default 3, as stock)");

struct sc8280xp_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack dp_jack[8];
	bool jack_setup;
};

static int sc8280xp_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_jack *dp_jack  = NULL;
	struct snd_soc_dai *codec_dai;
	int dp_pcm_id = 0;
	int ret, i;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX...QUATERNARY_MI2S_TX:
	case QUINARY_MI2S_RX...QUINARY_MI2S_TX:
		snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_BP_FP);
		break;
	case PRIMARY_TDM_RX_0...QUINARY_TDM_TX_7:
		/*
		 * DSP_A framing: a one-bit-clock-wide sync pulse with the first
		 * slot starting one bit clock after it.
		 *
		 * snd_soc_runtime_set_dai_fmt() rather than
		 * snd_soc_dai_set_fmt(cpu_dai, ...) as the MI2S case above uses,
		 * because on a TDM link the codecs need the format too and the
		 * cpu-only call never reaches them.  A codec left at its reset
		 * format samples the wrong framing and, where the link asks for
		 * an inverted bit clock as here, the wrong clock edge -- neither
		 * of which reports an error, they just corrupt the audio.
		 *
		 * SND_SOC_DAIFMT_IB_NF is meaningful only to the codecs: the
		 * host's TDM interface parameter can invert the frame but has no
		 * bit-clock inversion at all, so the amplifiers are what latch on
		 * the opposite edge.
		 */
		ret = snd_soc_runtime_set_dai_fmt(rtd, SND_SOC_DAIFMT_DSP_A |
						  SND_SOC_DAIFMT_CBC_CFC |
						  SND_SOC_DAIFMT_IB_NF);
		if (ret) {
			dev_err(card->dev, "Failed to set TDM format: %d\n", ret);
			return ret;
		}

		/*
		 * A four-slot frame, all four slots carried, in 32-bit slots.
		 *
		 * The slot width is 32 while the samples are 24-bit: a 24-bit
		 * sample rides in a 32-bit slot, and the bit clock follows the
		 * SLOT width, so this is what makes the link clock at
		 * rate x 32 x 4 rather than three quarters of that.
		 *
		 * The codecs are given the slot WIDTH but no mask and no slot
		 * count, because a CS35L45 does not take its slot position from
		 * here -- it has its own ASPRX Slot Position controls, set from
		 * userspace, which is how four amplifiers on one frame each end
		 * up on a different slot.
		 */
		ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0,
					       GENMASK(tdm_slots - 1, 0),
					       tdm_slots, tdm_slot_width);
		if (ret) {
			dev_err(card->dev, "Failed to set TDM slots: %d\n", ret);
			return ret;
		}

		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, 0, 0,
						       tdm_slot_width);
			if (ret && ret != -ENOTSUPP) {
				dev_err(card->dev, "%s: failed to set slot width: %d\n",
					codec_dai->name, ret);
				return ret;
			}
		}

		/*
		 * Cap the amplifiers at unity, which is where the vendor parks
		 * them.  The part offers +12 dB of digital gain above 0 dB and
		 * the vendor never uses it: its mixer configuration writes a
		 * constant 0.00 dB on every playback path and does stream
		 * volume further upstream.
		 *
		 * This matters because the digital volume is a HARDWARE VOLUME
		 * CONTROL for the sound server -- userspace gangs the four
		 * per-amplifier controls into one element and binds it as the
		 * sink's volume.  PulseAudio maps its 0..100 % onto the
		 * element's min_dB..max_dB, so an unlimited control puts +12 dB
		 * at the top of the user's slider: four times the amplitude the
		 * board is tuned for, and a level no vendor path produces.
		 *
		 * snd_soc_limit_volume() sets platform_max, which
		 * snd_soc_info_volsw() reports as the control's maximum and
		 * soc_put_volsw() enforces, so the cap reaches userspace as a
		 * smaller dB range rather than as writes that are silently
		 * clipped -- the slider ends at 0.00 dB instead of ending at
		 * +12 dB with a dead zone.
		 *
		 * The name is built from each codec's own name_prefix rather
		 * than hard-coded, so it follows a prefix rename.  A failure is
		 * warned about and not ignored: ASoC registers the ALREADY
		 * TRUNCATED control name, so a prefix long enough to push
		 * "Digital PCM Volume" past the 44-byte limit would stop
		 * matching, and a safety cap that quietly did not apply is
		 * worse than one that never existed.
		 */
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			const char *prefix = codec_dai->component->name_prefix;
			char ctl[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

			if (!prefix)
				continue;

			snprintf(ctl, sizeof(ctl), "%s Digital PCM Volume",
				 prefix);

			ret = snd_soc_limit_volume(card, ctl,
						   CS35L45_DIG_PCM_VOL_0DB);
			if (ret)
				dev_warn(card->dev,
					 "%s: 0 dB cap NOT applied: %d\n",
					 ctl, ret);
		}
		break;
	case WSA_CODEC_DMA_RX_0:
	case WSA_CODEC_DMA_RX_1:
		/*
		 * Set limit of -3 dB on Digital Volume and 0 dB on PA Volume
		 * to reduce the risk of speaker damage until we have active
		 * speaker protection in place.
		 */
		snd_soc_limit_volume(card, "WSA_RX0 Digital Volume", 81);
		snd_soc_limit_volume(card, "WSA_RX1 Digital Volume", 81);
		snd_soc_limit_volume(card, "SpkrLeft PA Volume", 17);
		snd_soc_limit_volume(card, "SpkrRight PA Volume", 17);
		break;
	case DISPLAY_PORT_RX_0:
		/* DISPLAY_PORT dai ids are not contiguous */
		dp_pcm_id = 0;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	case DISPLAY_PORT_RX_1 ... DISPLAY_PORT_RX_7:
		dp_pcm_id = cpu_dai->id - DISPLAY_PORT_RX_1 + 1;
		dp_jack = &data->dp_jack[dp_pcm_id];
		break;
	default:
		break;
	}

	if (dp_jack)
		return qcom_snd_dp_jack_setup(rtd, dp_jack, dp_pcm_id);

	return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
}

static int sc8280xp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	channels->min = 2;
	channels->max = 2;
	switch (cpu_dai->id) {
	case PRIMARY_TDM_RX_0...QUINARY_TDM_TX_7:
		/*
		 * A TDM frame is not a stereo pair.  The defaults above describe
		 * a two-channel 16-bit link, which is what every other backend
		 * here happens to be; forcing them onto a TDM link would silently
		 * collapse a four-slot frame to its first two slots and cut the
		 * bit clock accordingly, without anything reporting an error.
		 *
		 * S24_LE is a 24-bit sample in a 32-bit container, which is what
		 * the frame carries: the container is the slot.
		 */
		/*
		 * snd_mask_set_format() SETS a bit; it does not replace the
		 * mask.  The default above has already set S16_LE, so simply
		 * setting S24_LE here leaves BOTH permitted and the resolution
		 * takes the lower one.  The backend is then programmed for
		 * 16-bit samples while the DSP endpoint has been told 24, and
		 * nothing reports an error: the amplifiers' ASP_WL reads 16
		 * beside an ASP_WIDTH_RX of 32, because the slot width comes
		 * from set_tdm_slot() while the sample width comes from here.
		 *
		 * Clear the mask first so the backend carries exactly one
		 * format and both ends of the link agree on it.
		 */
		snd_mask_none(fmt);
		snd_mask_set_format(fmt, tdm_sample_bits == 16 ?
					 SNDRV_PCM_FORMAT_S16_LE :
					 SNDRV_PCM_FORMAT_S24_LE);
		channels->min = tdm_channels;
		channels->max = tdm_channels;
		break;
	case TX_CODEC_DMA_TX_0:
	case TX_CODEC_DMA_TX_1:
	case TX_CODEC_DMA_TX_2:
	case TX_CODEC_DMA_TX_3:
		channels->min = 1;
		break;
	case VA_CODEC_DMA_TX_0:
	case VA_CODEC_DMA_TX_1:
	case VA_CODEC_DMA_TX_2:
		/*
		 * The defaults above pin this backend to a stereo pair, which
		 * silently caps a three-microphone board at two microphones:
		 * the third is wired, clocked and populated, and simply has
		 * nowhere to go.  Widen the range instead of moving it, so a
		 * narrower capture still negotiates.
		 *
		 * The floor set here is not the floor that results.  DPCM merges
		 * the front end, the backend and the platform driver by keeping
		 * the HIGHEST minimum and the LOWEST maximum, and
		 * q6apm_dai_hardware_capture() declares channels_min 2 -- so the
		 * PCM offers 2..3 whatever this asks for below 2.  Measured on
		 * the device: 2 and 3 open, 1 and 4 are refused.
		 *
		 * THE CHANNEL COUNT IS DECIDED IN TWO PLACES AND THEY MUST
		 * AGREE.  This sets what the PCM may negotiate.  What the VA
		 * macro actually configures comes from somewhere else entirely:
		 * va_macro_hw_params() walks active_ch_mask, the bitmap the
		 * "VA_AIF1_CAP Mixer DECn" switches maintain, and programs one
		 * path per set bit.  It never compares that count against
		 * params_channels().  So a PCM opened for three channels with
		 * only two decimator switches enabled produces no error and a
		 * third channel of nothing.  Whatever selects this backend --
		 * the UCM verb here -- has to enable exactly as many
		 * decimators as it opens channels.
		 */
		channels->min = 1;
		channels->max = dmic_channels;
		break;
	default:
		break;
	}


	return 0;
}

static int sc8280xp_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int sc8280xp_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sc8280xp_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sc8280xp_be_ops = {
	.startup = qcom_snd_sdw_startup,
	.shutdown = qcom_snd_sdw_shutdown,
	.hw_free = sc8280xp_snd_hw_free,
	.prepare = sc8280xp_snd_prepare,
};

static void sc8280xp_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = sc8280xp_snd_init;
			link->be_hw_params_fixup = sc8280xp_be_hw_params_fixup;
			link->ops = &sc8280xp_be_ops;
		}
	}
}

static int sc8280xp_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sc8280xp_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;
	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	sc8280xp_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sc8280xp_dt_match[] = {
	{.compatible = "qcom,kaanapali-sndcard", "kaanapali"},
	{.compatible = "qcom,qcm6490-idp-sndcard", "qcm6490"},
	{.compatible = "qcom,qcs615-sndcard", "qcs615"},
	{.compatible = "qcom,qcs6490-rb3gen2-sndcard", "qcs6490"},
	{.compatible = "qcom,qcs8275-sndcard", "qcs8300"},
	{.compatible = "qcom,qcs9075-sndcard", "sa8775p"},
	{.compatible = "qcom,qcs9100-sndcard", "sa8775p"},
	{.compatible = "qcom,sc8280xp-sndcard", "sc8280xp"},
	{.compatible = "qcom,sm8450-sndcard", "sm8450"},
	{.compatible = "qcom,sm8550-sndcard", "sm8550"},
	{.compatible = "qcom,sm8650-sndcard", "sm8650"},
	{.compatible = "qcom,sm8750-sndcard", "sm8750"},
	{}
};

MODULE_DEVICE_TABLE(of, snd_sc8280xp_dt_match);

static struct platform_driver snd_sc8280xp_driver = {
	.probe  = sc8280xp_platform_probe,
	.driver = {
		.name = "snd-sc8280xp",
		.of_match_table = snd_sc8280xp_dt_match,
	},
};
module_platform_driver(snd_sc8280xp_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SC8280XP ASoC Machine Driver");
MODULE_LICENSE("GPL");
