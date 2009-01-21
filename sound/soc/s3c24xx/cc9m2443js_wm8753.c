/* -*- linux-c -*-
 *
 * cc9m2443js_wm8753.c  --  SoC audio for the Digi CC9M2443JS
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Graeme Gregory
 *         graeme.gregory@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Based on the driver of the Neo1973
 *
 *  IMPORTANT: In difference to the Neo1973, we are using the S3C2443 as master
 *  and not as slave. That means, the configuration of the codec is quite different
 *  (Luis Galdos)
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/math64.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <mach/hardware.h>
#include <mach/audio.h>
#include <linux/io.h>
#include <mach/spi-gpio.h>
#include <sound/pcm_params.h>

#include <asm/plat-s3c24xx/regs-iis.h>

#include "../codecs/wm8753.h"
#include "lm4857.h"
#include "s3c24xx-pcm.h"
#include "s3c2412-i2s.h"

/* define the scenarios */
#define NEO_AUDIO_OFF			0
#define NEO_GSM_CALL_AUDIO_HANDSET	1
#define NEO_GSM_CALL_AUDIO_HEADSET	2
#define NEO_GSM_CALL_AUDIO_BLUETOOTH	3
#define NEO_STEREO_TO_SPEAKERS		4
#define NEO_STEREO_TO_HEADPHONES	5
#define NEO_CAPTURE_HANDSET		6
#define NEO_CAPTURE_HEADSET		7
#define NEO_CAPTURE_BLUETOOTH		8


/* Additional info and error messages */
#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] cc9m2443-snd: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "cc9m2443-snd: " fmt, ## args)



#if 1
#define CC9M2443JS_SND_DEBUG
#endif

#ifdef CC9M2443JS_SND_DEBUG
#  define printk_debug(fmt, args...)            printk(KERN_DEBUG "cc9m2443-snd: %s() " fmt, __func__ , ## args)
#else
#  define printk_debug(fmt, args...)
#endif


static int cc9m2443js_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->dai->cpu_dai;
	unsigned int pll_out = 0, bclk = 0;
	int ret = 0;
	unsigned long bitfs, is16bit;
	struct s3c2412_rate_calc info;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
		pll_out = 12288000;
		break;
	case 48000:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* cpu_iis_mclk = iis_clkrate / params_rate(params); */
	s3c2412_iis_calc_rate(&info, NULL, params_rate(params), s3c2412_get_iisclk());
	
	printk_debug("Sample rate %i | Master Clk %lu Hz | FS %u | Clk divisor %u\n",
		     params_rate(params), clk_get_rate(s3c2412_get_iisclk()),
		     info.fs_div, info.clk_div);
	
	/* REQUIRED: Set codec DAI configuration (normal bclk and frm ) and as slave */
	ret = codec_dai->dai_ops.set_fmt(codec_dai,
					 SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					 SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* REQUIRED: Set cpu DAI configuration (normal bclk and frm) and CPU as master */
	ret = cpu_dai->dai_ops.set_fmt(cpu_dai,
				       SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				       SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* REQUIRED: Set the codec system clock for DAC and ADC */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, WM8753_MCLK, pll_out,
					    SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk_err("set_sysclk() failed (%i)\n", ret);
		return ret;
	}

	/* REQUIRED: Set MCLK division for sample rate (256, 384, 512, 768) */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C2412_DIV_RCLK, info.fs_div);
	if (ret < 0)
		return ret;
	
	/*
	 * Set the BCLK configuration for the CPU
	 * The required value depends on the configured root clock (RFS) according
	 * to the table 25-2 of the data sheet of the S3C2443
	 * @XXX: Define the corresponding macros for the Bit FS selection
	 */
	is16bit = (params_format(params) == SNDRV_PCM_FORMAT_S16_LE) ? 1 : 0;
	switch (info.fs_div) {
	case 256:
		bitfs = (is16bit) ? 0x0 : 0x2;
		break;
	case 512:
		bitfs = (is16bit) ? 0x0 : 0x2;
		break;
	case 384:
		bitfs = (is16bit) ? 0x0 : 0x2;
		break;
	case 768:
		bitfs = (is16bit) ? 0x0 : 0x2;
		break;
	default:
		printk_err("Unsupported root clock FS %i divisor found\n", info.fs_div);
		ret = -EINVAL;
		goto exit_all;
		break;
	}
	
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C2412_DIV_BCLK, bitfs);
	if (ret) {
		printk_err("Couldn't set the BLCK, %i\n", ret);
		return ret;
	}
	
	/* Set the prescaler divisor for the master clock */
	ret = cpu_dai->dai_ops.set_clkdiv(cpu_dai, S3C2412_DIV_PRESCALER,
					  info.clk_div);
	if (ret < 0) {
		printk_err("Prescaler setting failed, %i\n", ret);
		return ret;
	}

 exit_all:
	return ret;
}

static int cc9m2443js_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	/* disable the PLL */
	return codec_dai->dai_ops.set_pll(codec_dai, WM8753_PLL1, 0, 0);
}


static int cc9m2443js_hifi_startup(struct snd_pcm_substream *substream)
{
	printk_debug("Going to startup the codec\n");
	
	return 0;
}

/* We don't need this function, but we know that they are available */
static void cc9m2443js_hifi_shutdown(struct snd_pcm_substream *substream)
{

	printk_debug("Going to shutdown the codec\n");
}


/*
 * WM8753 HiFi DAI operations.
 */
static struct snd_soc_ops cc9m2443js_hifi_ops = {
	.hw_params = cc9m2443js_hifi_hw_params,
	.hw_free = cc9m2443js_hifi_hw_free,
	.startup = cc9m2443js_hifi_startup,
	.shutdown = cc9m2443js_hifi_shutdown,
};

static int cc9m2443js_voice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int pcmdiv = 0;
	int ret = 0;

	printk_err("@XXX: Remove this function (no more required).\n");
	
	if (params_rate(params) != 8000)
		return -EINVAL;
	if (params_channels(params) != 1)
		return -EINVAL;

	pcmdiv = WM8753_PCM_DIV_6; /* 2.048 MHz */

	/* todo: gg check mode (DSP_B) against CSR datasheet */
	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_B |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->dai_ops.set_sysclk(codec_dai, WM8753_PCMCLK, 12288000,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = codec_dai->dai_ops.set_clkdiv(codec_dai, WM8753_PCMDIV, pcmdiv);
	if (ret < 0)
		return ret;

	return 0;
}

static int cc9m2443js_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->dai->codec_dai;

	printk_err("@XXX: Remove this function (no more required).\n");
	
	/* disable the PLL */
	return codec_dai->dai_ops.set_pll(codec_dai, WM8753_PLL2, 0, 0);
}


static const struct snd_soc_dapm_widget wm8753_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Audio Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line Out", NULL),
	SND_SOC_DAPM_LINE("GSM Line In", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_MIC("Call Mic", NULL),
};



static const struct snd_soc_dapm_route dapm_routes[] = {

        /* Connections to the lm4857 amp */
        {"Audio Out", NULL, "LOUT1"},
        {"Audio Out", NULL, "ROUT1"},

        /* Connections to Headset */
        {"MIC1", NULL, "Mic Bias"},
        {"Mic Bias", NULL, "Headset Mic"},

        /* Call Mic */
        {"MIC2", NULL, "Mic Bias"},
        {"MIC2N", NULL, "Mic Bias"},
        {"Mic Bias", NULL, "Call Mic"},

        /* Connect the ALC pins */
        {"ACIN", NULL, "ACOP"},
};


static const char *neo_scenarios[] = {
	"Off",
	"Speakers",
	"Headphones",
/* 	"Capture Handset", */
/* 	"Capture Headset", */
};

static const struct soc_enum neo_scenario_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(neo_scenarios), neo_scenarios),
};

static const struct snd_kcontrol_new wm8753_cc9m2443js_controls[] = {
/* 	SOC_SINGLE_EXT("Amp Left Playback Volume", LM4857_LVOL, 0, 31, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
/* 	SOC_SINGLE_EXT("Amp Right Playback Volume", LM4857_RVOL, 0, 31, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
/* 	SOC_SINGLE_EXT("Amp Mono Playback Volume", LM4857_MVOL, 0, 31, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
/* 	SOC_ENUM_EXT("Amp Mode", lm4857_mode_enum[0], */
/* 		lm4857_get_mode, lm4857_set_mode), */
/* 	SOC_ENUM_EXT("Neo Mode", neo_scenario_enum[0], */
/* 		cc9m2443js_get_scenario, cc9m2443js_set_scenario), */
/* 	SOC_SINGLE_EXT("Amp Spk 3D Playback Switch", LM4857_LVOL, 5, 1, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
/* 	SOC_SINGLE_EXT("Amp HP 3d Playback Switch", LM4857_RVOL, 5, 1, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
/* 	SOC_SINGLE_EXT("Amp Fast Wakeup Playback Switch", LM4857_CTRL, 5, 1, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
/* 	SOC_SINGLE_EXT("Amp Earpiece 6dB Playback Switch", LM4857_CTRL, 4, 1, 0, */
/* 		lm4857_get_reg, lm4857_set_reg), */
};

/*
 * This is an example machine initialisation for a wm8753 connected to a
 * neo1973 II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int cc9m2443js_wm8753_snd_init(struct snd_soc_codec *codec)
{
	int i, err;

	/* set up NC codec pins */
	snd_soc_dapm_disable_pin(codec, "LOUT2");
	snd_soc_dapm_disable_pin(codec, "ROUT2");
	snd_soc_dapm_disable_pin(codec, "OUT3");
	snd_soc_dapm_disable_pin(codec, "OUT4");
	snd_soc_dapm_disable_pin(codec, "LINE1");
	snd_soc_dapm_disable_pin(codec, "LINE2");

	/* set endpoints to default mode */
	/* set_scenario_endpoints(codec, NEO_AUDIO_OFF); */

	/* Add neo1973 specific widgets */
	for (i = 0; i < ARRAY_SIZE(wm8753_dapm_widgets); i++)
		snd_soc_dapm_new_control(codec, &wm8753_dapm_widgets[i]);

	/* Add specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8753_cc9m2443js_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm8753_cc9m2443js_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}

	/* set up specific audio path audio_mapnects */
	err = snd_soc_dapm_add_routes(codec, dapm_routes, ARRAY_SIZE(dapm_routes));
	if (err)
		printk_err("Couldn't add the routes (%i)\n", err);
	else
		snd_soc_dapm_sync(codec);
	
	return err;
}

static struct snd_soc_ops cc9m2443js_voice_ops = {
	.hw_params = cc9m2443js_voice_hw_params,
	.hw_free = cc9m2443js_voice_hw_free,
};

/* Structure for the interface with the DAIs of the codec and CPU */
static struct snd_soc_dai_link cc9m2443js_dai[] = {
	{
		/*
		 * Hifi Playback
		 * Infos under: proc/asound/card0/pcm0p/info
		 */
		.name = "WM8753",
		.stream_name = "WM8753 HiFi",
		.cpu_dai = &s3c2443_i2s_dai,
		.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
		.init = cc9m2443js_wm8753_snd_init,
		.ops = &cc9m2443js_hifi_ops,
	},{
		/*
		 * Voice
		 * Infos under: proc/asound/card0/pcm1c/info
		 */
		.name = "Voice",
		.stream_name = "WM8753 Voice",
		.cpu_dai = &s3c2443_i2s_dai,
		.codec_dai = &wm8753_dai[WM8753_DAI_VOICE],
		.ops = &cc9m2443js_voice_ops,
	}
};

/* This is the machine-structure */
static struct snd_soc_machine cc9m2443js_snd = {
	.name = "cc9m2443js",
	.dai_link = cc9m2443js_dai,
	.num_links = ARRAY_SIZE(cc9m2443js_dai),
};

static struct wm8753_setup_data cc9m2443js_wm8753_setup = {
	.i2c_address	= 0x1a,
	.i2c_bus	= CONFIG_I2C_S3C2410_ADAPTER_NR,
};

static struct snd_soc_device cc9m2443js_snd_devdata = {
	.machine = &cc9m2443js_snd,
	.platform = &s3c24xx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8753,
	.codec_data = &cc9m2443js_wm8753_setup,
};


static struct platform_device *cc9m2443js_snd_device;

static int __init cc9m2443js_wm8753_init(void)
{
	int ret;

	cc9m2443js_snd_device = platform_device_alloc("soc-audio", -1);
	if (!cc9m2443js_snd_device)
		return -ENOMEM;

	platform_set_drvdata(cc9m2443js_snd_device, &cc9m2443js_snd_devdata);
	cc9m2443js_snd_devdata.dev = &cc9m2443js_snd_device->dev;
	ret = platform_device_add(cc9m2443js_snd_device);

	if (ret)
		platform_device_put(cc9m2443js_snd_device);

	return ret;
}

static void __exit cc9m2443js_wm8753_exit(void)
{
	platform_device_unregister(cc9m2443js_snd_device);
}

module_init(cc9m2443js_wm8753_init);
module_exit(cc9m2443js_wm8753_exit);

/* Module information */
MODULE_AUTHOR("Graeme Gregory, graeme@openmoko.org, www.openmoko.org");
MODULE_AUTHOR("Luis Galdos, luis.galdos@digi.com");
MODULE_DESCRIPTION("ALSA SoC WM8753 Digi CC9M2443JS");
MODULE_LICENSE("GPL");
