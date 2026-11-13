// SPDX-License-Identifier: GPL-2.0
/*
 * dw-hdmi-i2s-audio.c
 *
 * Copyright (c) 2017 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>

#include <drm/bridge/dw_hdmi.h>
#include <drm/drm_crtc.h>

#include <sound/hdmi-codec.h>

#include "dw-hdmi.h"
#include "dw-hdmi-audio.h"

#define DRIVER_NAME "dw-hdmi-i2s-audio"

static inline void hdmi_write(struct dw_hdmi_i2s_audio_data *audio,
			      u8 val, int offset)
{
	struct dw_hdmi *hdmi = audio->hdmi;

	audio->write(hdmi, val, offset);
}

static inline u8 hdmi_read(struct dw_hdmi_i2s_audio_data *audio, int offset)
{
	struct dw_hdmi *hdmi = audio->hdmi;

	return audio->read(hdmi, offset);
}

static void dw_hdmi_audio_reset(struct dw_hdmi_i2s_audio_data *audio,
				u8 conf0)
{
	u8 reset = HDMI_MC_SWRSTZ_I2SSWRST_REQ;

	hdmi_write(audio, conf0 | HDMI_AUD_CONF0_SW_RESET, HDMI_AUD_CONF0);
	hdmi_write(audio, conf0, HDMI_AUD_CONF0);
	if (audio->spdif) {
		reset |= HDMI_MC_SWRSTZ_SPDIFSWRST_REQ;
		hdmi_write(audio, HDMI_AUD_SPDIF0_SW_RESET, HDMI_AUD_SPDIF0);
		hdmi_write(audio, 0, HDMI_AUD_SPDIF0);
	}
	hdmi_write(audio, (u8)~reset, HDMI_MC_SWRSTZ);
	if (audio->spdif) {
		/* The Amlogic sampler requires another FIFO reset after MC. */
		hdmi_write(audio, HDMI_AUD_SPDIF0_SW_RESET, HDMI_AUD_SPDIF0);
		hdmi_write(audio, 0, HDMI_AUD_SPDIF0);
		usleep_range(9, 11);
	}
}

static int dw_hdmi_i2s_prepare(struct device *dev, void *data,
				 struct hdmi_codec_daifmt *fmt,
				 struct hdmi_codec_params *hparms)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	struct dw_hdmi *hdmi = audio->hdmi;
	bool spdif = fmt->fmt == HDMI_SPDIF;
	bool non_pcm = hparms->iec.status[0] & IEC958_AES0_NONAUDIO;
	u8 conf0, conf1, conf2, inputclkfs, sample_freq, sample_layout;

	if (fmt->bit_clk_provider || fmt->frame_clk_provider) {
		dev_err(dev, "unsupported clock settings\n");
		return -EINVAL;
	}
	if (hparms->iec.status[0] & IEC958_AES0_PROFESSIONAL)
		return -EINVAL;
	if (fmt->bit_fmt == SNDRV_PCM_FORMAT_IEC958_SUBFRAME_LE ||
	    fmt->bit_fmt == SNDRV_PCM_FORMAT_IEC958_SUBFRAME_BE)
		return -EINVAL;
	if (hparms->sample_width != snd_pcm_format_width(fmt->bit_fmt))
		return -EINVAL;
	if (hparms->channels < 2 || hparms->channels > 8)
		return -EINVAL;
	switch (hparms->sample_rate) {
	case 32000:
		sample_freq = IEC958_AES3_CON_FS_32000;
		break;
	case 44100:
		sample_freq = IEC958_AES3_CON_FS_44100;
		break;
	case 48000:
		sample_freq = IEC958_AES3_CON_FS_48000;
		break;
	case 88200:
		sample_freq = IEC958_AES3_CON_FS_88200;
		break;
	case 96000:
		sample_freq = IEC958_AES3_CON_FS_96000;
		break;
	case 176400:
		sample_freq = IEC958_AES3_CON_FS_176400;
		break;
	case 192000:
		sample_freq = IEC958_AES3_CON_FS_192000;
		break;
	default:
		return -EINVAL;
	}
	if (non_pcm && !spdif) {
		dev_err(dev, "compressed audio requires the S/PDIF input\n");
		return -EOPNOTSUPP;
	}
	if (spdif && (!audio->spdif || hparms->channels != 2))
		return -EINVAL;
	/* Stereo NLPCM uses the carrier rate, not an HBR aggregate rate. */
	if (non_pcm &&
	    ((hparms->iec.status[3] & IEC958_AES3_CON_FS) != sample_freq ||
	     hparms->sample_width != 16))
		return -EINVAL;

	switch (hparms->sample_width) {
	case 16:
		conf1 = HDMI_AUD_CONF1_WIDTH_16;
		break;
	case 20:
		conf1 = HDMI_AUD_CONF1_WIDTH_20;
		break;
	case 24:
		conf1 = HDMI_AUD_CONF1_WIDTH_24;
		break;
	case 32:
		if (spdif)
			return -EINVAL;
		conf1 = HDMI_AUD_CONF1_WIDTH_24;
		break;
	default:
		return -EINVAL;
	}

	conf0 = HDMI_AUD_CONF0_I2S_SELECT |
		((1 << DIV_ROUND_UP(hparms->channels, 2)) - 1);
	inputclkfs = HDMI_AUD_INPUTCLKFS_64FS;
	switch (fmt->fmt) {
	case HDMI_I2S:
		conf1 |= HDMI_AUD_CONF1_MODE_I2S;
		break;
	case HDMI_RIGHT_J:
		conf1 |= HDMI_AUD_CONF1_MODE_RIGHT_J;
		break;
	case HDMI_LEFT_J:
		conf1 |= HDMI_AUD_CONF1_MODE_LEFT_J;
		break;
	case HDMI_DSP_A:
		conf1 |= HDMI_AUD_CONF1_MODE_BURST_1;
		break;
	case HDMI_DSP_B:
		conf1 |= HDMI_AUD_CONF1_MODE_BURST_2;
		break;
	case HDMI_SPDIF:
		conf0 = 0;
		conf1 = HDMI_AUD_CONF1_WIDTH_24;
		inputclkfs = HDMI_AUD_INPUTCLKFS_128FS;
		break;
	default:
		dev_err(dev, "unsupported format\n");
		return -EINVAL;
	}

	/* Stop I2S lanes while replacing all sampler and packet state. */
	hdmi_write(audio, conf0 & HDMI_AUD_CONF0_I2S_SELECT, HDMI_AUD_CONF0);
	hdmi_write(audio, HDMI_AUD_INT_FIFO_EMPTY_MSK |
		   HDMI_AUD_INT_FIFO_FULL_MSK, HDMI_AUD_INT);
	hdmi_write(audio, conf1, HDMI_AUD_CONF1);
	conf2 = hdmi_read(audio, HDMI_AUD_CONF2);
	conf2 &= ~(HDMI_AUD_CONF2_HBR | HDMI_AUD_CONF2_NLPCM);
	hdmi_write(audio, conf2, HDMI_AUD_CONF2);
	if (audio->spdif) {
		/* NLPCM status and validity arrive in the S/PDIF subframes. */
		hdmi_write(audio, HDMI_AUD_SPDIF1_WIDTH_24, HDMI_AUD_SPDIF1);
		hdmi_write(audio, HDMI_AUD_INT_FIFO_EMPTY_MSK |
			   HDMI_AUD_INT_FIFO_FULL_MSK, HDMI_AUD_SPDIFINT);
	}
	hdmi_write(audio, inputclkfs, HDMI_AUD_INPUTCLKFS);
	if (!spdif)
		dw_hdmi_set_channel_status(hdmi, hparms->iec.status);
	dw_hdmi_set_channel_count(hdmi, hparms->channels);
	sample_layout = hdmi_read(audio, HDMI_FC_AUDSCONF);
	sample_layout &= ~HDMI_FC_AUDSCONF_AUD_PACKET_SAMPFIT_MASK;
	hdmi_write(audio, sample_layout, HDMI_FC_AUDSCONF);
	dw_hdmi_set_channel_allocation(hdmi, hparms->cea.channel_allocation);
	/* Coding type, sample size and frequency refer to the stream. */
	hdmi_write(audio, (hparms->cea.channels ? hparms->cea.channels - 1 : 0) <<
		   HDMI_FC_AUDICONF0_CC_OFFSET, HDMI_FC_AUDICONF0);
	hdmi_write(audio, 0, HDMI_FC_AUDICONF1);
	hdmi_write(audio, 0, HDMI_FC_AUDICONF3);
	hdmi_write(audio, 0, HDMI_FC_AUDSV);
	hdmi_write(audio, 0, HDMI_FC_AUDSU);
	hdmi_write(audio, conf0, HDMI_AUD_CONF0);
	dw_hdmi_audio_reset(audio, conf0);
	/* Relatch N/CTS after reset, using the carrier frame rate. */
	dw_hdmi_set_sample_rate(hdmi, hparms->sample_rate);

	return 0;
}

static int dw_hdmi_i2s_audio_startup(struct device *dev, void *data)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	struct dw_hdmi *hdmi = audio->hdmi;

	dw_hdmi_audio_enable(hdmi);

	return 0;
}

static void dw_hdmi_i2s_audio_shutdown(struct device *dev, void *data)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	struct dw_hdmi *hdmi = audio->hdmi;

	dw_hdmi_audio_disable(hdmi);
}

static int dw_hdmi_i2s_get_eld(struct device *dev, void *data, uint8_t *buf,
			       size_t len)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	u8 *eld;

	eld = audio->get_eld(audio->hdmi);
	if (eld)
		memcpy(buf, eld, min_t(size_t, MAX_ELD_BYTES, len));
	else
		/* Pass en empty ELD if connector not available */
		memset(buf, 0, len);

	return 0;
}

static int dw_hdmi_i2s_get_dai_id(struct snd_soc_component *component,
				  struct device_node *endpoint,
				  void *data)
{
	struct of_endpoint of_ep;
	int ret;

	ret = of_graph_parse_endpoint(endpoint, &of_ep);
	if (ret < 0)
		return ret;

	/*
	 * HDMI sound is on port 2. Endpoint 0 is I2S and endpoint 1
	 * selects the optional S/PDIF DAI.
	 */
	if (of_ep.port == 2) {
		struct dw_hdmi_i2s_audio_data *audio = data;

		if (of_ep.id == 0 || (of_ep.id == 1 && audio->spdif))
			return of_ep.id;
	}

	return -EINVAL;
}

static int dw_hdmi_i2s_hook_plugged_cb(struct device *dev, void *data,
				       hdmi_codec_plugged_cb fn,
				       struct device *codec_dev)
{
	struct dw_hdmi_i2s_audio_data *audio = data;
	struct dw_hdmi *hdmi = audio->hdmi;

	return dw_hdmi_set_plugged_cb(hdmi, fn, codec_dev);
}

static const struct hdmi_codec_ops dw_hdmi_i2s_ops = {
	.prepare	= dw_hdmi_i2s_prepare,
	.audio_startup  = dw_hdmi_i2s_audio_startup,
	.audio_shutdown	= dw_hdmi_i2s_audio_shutdown,
	.get_eld	= dw_hdmi_i2s_get_eld,
	.get_dai_id	= dw_hdmi_i2s_get_dai_id,
	.hook_plugged_cb = dw_hdmi_i2s_hook_plugged_cb,
};

static int snd_dw_hdmi_probe(struct platform_device *pdev)
{
	struct dw_hdmi_i2s_audio_data *audio = pdev->dev.platform_data;
	struct platform_device_info pdevinfo;
	struct hdmi_codec_pdata pdata;
	struct platform_device *platform;

	memset(&pdata, 0, sizeof(pdata));
	pdata.ops		= &dw_hdmi_i2s_ops;
	pdata.i2s		= 1;
	pdata.spdif		= audio->spdif;
	pdata.no_i2s_capture	= 1;
	pdata.no_spdif_capture	= 1;
	pdata.i2s_formats	= SNDRV_PCM_FMTBIT_S16_LE |
		SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_3LE |
		SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE;
	pdata.max_i2s_channels	= 8;
	pdata.data		= audio;

	memset(&pdevinfo, 0, sizeof(pdevinfo));
	pdevinfo.parent		= pdev->dev.parent;
	pdevinfo.id		= PLATFORM_DEVID_AUTO;
	pdevinfo.name		= HDMI_CODEC_DRV_NAME;
	pdevinfo.data		= &pdata;
	pdevinfo.size_data	= sizeof(pdata);
	pdevinfo.dma_mask	= DMA_BIT_MASK(32);

	platform = platform_device_register_full(&pdevinfo);
	if (IS_ERR(platform))
		return PTR_ERR(platform);

	dev_set_drvdata(&pdev->dev, platform);

	return 0;
}

static void snd_dw_hdmi_remove(struct platform_device *pdev)
{
	struct platform_device *platform = dev_get_drvdata(&pdev->dev);

	platform_device_unregister(platform);
}

static struct platform_driver snd_dw_hdmi_driver = {
	.probe	= snd_dw_hdmi_probe,
	.remove = snd_dw_hdmi_remove,
	.driver	= {
		.name = DRIVER_NAME,
	},
};
module_platform_driver(snd_dw_hdmi_driver);

MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
MODULE_DESCRIPTION("Synopsis Designware HDMI I2S ALSA SoC interface");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
