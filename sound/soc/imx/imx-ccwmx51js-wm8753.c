/*
 * imx-ccwmx51js-wm8753.c  --  i.MX Driver for Digi CCWMS51JS and the Wolfson WM8753 Codec
 *
 * Adapted by Timesys from sgtl5000 machine driver and other sources.
 *
 * Copyright 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2010 Timesys Corpotation. All Rights Reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Based on sound/soc/imx/imx-3stack-sgtl5000.c
 *
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <mach/dma.h>
#include <mach/spba.h>
#include <mach/clock.h>
#include <mach/mxc.h>
#include "../codecs/wm8753.h"
#include "imx-ssi.h"
#include "imx-pcm.h"


/* SSI BCLK and LRC master
 *
 * Set WM8753_SSI_MASTER to operate the WM8753 codec in master mode,
 * and set to zero to operate it in slave mode.
 */
#define WM8753_SSI_MASTER	1


struct ccwmx51_priv {
	int sysclk;
	struct platform_device *pdev;
};

static struct ccwmx51_priv machine_priv;

static struct snd_soc_dai ccwmx51_cpu_dai;


static int ccwmx51js_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct ccwmx51_priv *priv = &machine_priv;
	int ret = 0;

	unsigned int channels = params_channels(params);
	u32 dai_format;


#if WM8753_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_SYNC;
	if (channels == 2)
		dai_format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);

	if (ret < 0)
		return ret;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_SYNC;
	if (channels == 2)
		dai_format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);

	if (ret < 0)
		return ret;
#endif

	snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, priv->sysclk, 0);

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc, 2);

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);


	/*  The WM8753 PLL on the Digi CCWMX51JS is referenced to a fixed 13-MHz
	 *  oscillator.  We presently operate the PLL only at 11.289600
	 *  MHz output, which supports a 44.1-kHz sample rate and a few others.
	 *
	 */

		// set codec BCLK division for sample rate */
	ret = snd_soc_dai_set_clkdiv(codec_dai, WM8753_BCLKDIV, WM8753_BCLK_DIV_4);
	if (ret < 0)
		return ret;

		// Specify input PLL frequency and desired output frequency.
	ret = snd_soc_dai_set_pll(codec_dai, WM8753_PLL1, 13000000, 11289600);
	if (ret < 0)
		return ret;
	return 0;
}



	// Sets up the i.mx51 Digital Audio Multiplexer
static void ccwmx51_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* i.mx51 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if WM8753_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}



/*
 * Still to do: set the line out to become enabled within
 * this driver.  Currently the initial driver configuration
 * has all outputs disabled.  They can be enabled in user space
 * by doing:
 *
 *     amixer sset 'Left Mixer Left' unmute
 *     amixer sset 'Right Mixer Right Playback Swit' unmute
 *
 */
static int ccwmx51js_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void ccwmx51js_shutdown(struct snd_pcm_substream *substream)
{
}

static struct snd_soc_ops ccwmx51js_ops = {
	.startup = ccwmx51js_startup,
	.shutdown = ccwmx51js_shutdown,
	.hw_params = ccwmx51js_audio_hw_params,
};


    // Logic for wm8753 as connected on Digi CCWMX51JS board.
static int ccwmx51js_wm8753_init(struct snd_soc_codec *codec)
{
	return 0;
}


/* digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link ccwmx51js_dai = {
	.name = "CCWMX51JS WM8753",

	.stream_name = "WM8753 HiFi",
	.cpu_dai = &ccwmx51_cpu_dai,
	.codec_dai = &wm8753_dai[WM8753_DAI_HIFI],
	.init = ccwmx51js_wm8753_init,
	.ops = &ccwmx51js_ops,
};

static int ccwmx51js_machine_remove(struct platform_device *pdev)
{
	struct ccwmx51_priv *priv = &machine_priv;
	struct mxc_audio_platform_data *plat;

	if (priv->pdev) {
		plat = priv->pdev->dev.platform_data;
		if (plat->finit)
			plat->finit();
	}

	return 0;
}

/* audio machine driver */
static struct snd_soc_machine snd_soc_machine_ccwmx51js = {
	.name = "imx-ccwmx51js",
	.dai_link = &ccwmx51js_dai,
	.num_links = 1,
	.remove = ccwmx51js_machine_remove,
};

static struct wm8753_setup_data ccwmx51_wm8753_setup = {
	.i2c_address = 0x1a,
	.i2c_bus = 1,
};

static struct snd_soc_device ccwmx51js_snd_devdata = {
	.machine = &snd_soc_machine_ccwmx51js,
	.platform = &imx_soc_platform,
	.codec_dev = &soc_codec_dev_wm8753,
	.codec_data = &ccwmx51_wm8753_setup,
};

static int __devinit ccwmx51js_wm8753_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct ccwmx51_priv *priv = &machine_priv;
	int ret = 0;

	priv->sysclk = plat->sysclk;
	priv->pdev = pdev;

	imx_ssi_dai_init(&ccwmx51_cpu_dai);
	ccwmx51_cpu_dai.private_data = plat;

	ccwmx51_init_dam(plat->src_port, plat->ext_port);

	if (plat->src_port == 2)
		ccwmx51_cpu_dai.name = "imx-ssi-3";
	else
		ccwmx51_cpu_dai.name = "imx-ssi-1";

	ret = -EINVAL;
	if (plat->init && plat->init()){
		goto err_plat_init;
	}
	return 0;

err_plat_init:
	return ret;
}

static int ccwmx51js_wm8753_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver ccwmx51js_wm8753_audio_driver = {
	.probe = ccwmx51js_wm8753_probe,
	.remove = ccwmx51js_wm8753_remove,
	.driver = {
		.name = "imx-ccwmx51js-wm8753",
	},
};

static struct platform_device *ccwmx51js_snd_device;

static int __init ccwmx51js_init(void)
{
	int ret;

	ret = platform_driver_register(&ccwmx51js_wm8753_audio_driver);
	if (ret)
		return -ENOMEM;

	ccwmx51js_snd_device = platform_device_alloc("soc-audio", -1);
	if (!ccwmx51js_snd_device)
		return -ENOMEM;

	platform_set_drvdata(ccwmx51js_snd_device, &ccwmx51js_snd_devdata);
	ccwmx51js_snd_devdata.dev = &ccwmx51js_snd_device->dev;
	ret = platform_device_add(ccwmx51js_snd_device);

	if (ret)
		platform_device_put(ccwmx51js_snd_device);

	return ret;
}

static void __exit ccwmx51js_exit(void)
{
	platform_driver_unregister(&ccwmx51js_wm8753_audio_driver);
	platform_device_unregister(ccwmx51js_snd_device);
}

module_init(ccwmx51js_init);
module_exit(ccwmx51js_exit);

MODULE_AUTHOR("Timesys");
MODULE_DESCRIPTION("WM8753 Driver for Digi CCWMX51JS");
MODULE_LICENSE("GPL");
