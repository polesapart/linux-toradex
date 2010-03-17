/*
 * imx-ccwmx51-wm8753.c  --  i.MX CCWMX51 Driver for Freescale WM8753 Codec
 *
 * Copyright 2010 Digi International, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 * Adapted from sound/soc/imx/imx-3stack-sgtl5000.c.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
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

#include "../codecs/wm8753.h"
#include "imx-ssi.h"
#include "imx-pcm.h"

/* SSI BCLK and LRC master */
#define WM8753_SSI_MASTER	1

struct imx_ccwmx51_priv {
	int sysclk;
	struct platform_device *pdev;
};

static struct imx_ccwmx51_priv card_priv;

static int imx_ccwmx51_audio_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct imx_ccwmx51_priv *priv = &card_priv;
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	int ret = 0;

	unsigned int channels = params_channels(params);
	u32 dai_format;

	snd_soc_dai_set_sysclk(codec_dai, WM8753_MCLK, priv->sysclk, 0);

#if WM8753_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;
#endif

	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc, 2);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_ccwmx51_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void imx_ccwmx51_shutdown(struct snd_pcm_substream *substream)
{

}

/*
 * imx_ccwmx51 WM8753 audio DAI operations.
 */
static struct snd_soc_ops imx_ccwmx51_ops = {
	.startup = imx_ccwmx51_startup,
	.shutdown = imx_ccwmx51_shutdown,
	.hw_params = imx_ccwmx51_audio_hw_params,
};

static void imx_ccwmx51_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* iMX51 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

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

static int imx_ccwmx51_wm8753_init(struct snd_soc_codec *codec)
{
	return 0;
}

/* imx_ccwmx51 digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_ccwmx51_dai = {
	.name = "WM8753",
	.stream_name = "WM8753",
	.codec_dai = (struct snd_soc_dai *) &wm8753_dai,
	.init = imx_ccwmx51_wm8753_init,
	.ops = &imx_ccwmx51_ops,
};

static int imx_ccwmx51_card_remove(struct platform_device *pdev)
{
	struct imx_ccwmx51_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat;

	if (priv->pdev) {
		plat = priv->pdev->dev.platform_data;
		if (plat->finit)
			plat->finit();
	}

	return 0;
}

static struct snd_soc_card snd_soc_card_imx_ccwmx51 = {
	.name = "ccwmx51js",
	.platform = &imx_soc_platform,
	.dai_link = &imx_ccwmx51_dai,
	.num_links = 1,
	.remove = imx_ccwmx51_card_remove,
};

static struct snd_soc_device imx_ccwmx51_snd_devdata = {
	.card = &snd_soc_card_imx_ccwmx51,
	.codec_dev = &soc_codec_dev_wm8753,
};

static int __devinit imx_ccwmx51_wm8753_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_ccwmx51_priv *priv = &card_priv;
	struct snd_soc_dai *wm8753_cpu_dai;
	int ret = 0;

	priv->sysclk = plat->sysclk;
	priv->pdev = pdev;

	imx_ccwmx51_init_dam(plat->src_port, plat->ext_port);

	if (plat->src_port == 2)
		wm8753_cpu_dai = &imx_ssi_dai[2];
	else
		wm8753_cpu_dai = &imx_ssi_dai[0];

	imx_ccwmx51_dai.cpu_dai = wm8753_cpu_dai;

	ret = -EINVAL;
	if (plat->init && plat->init())
		goto err_plat_init;

	return 0;

err_plat_init:
	return ret;
}

static int imx_ccwmx51_wm8753_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_ccwmx51_wm8753_audio_driver = {
	.probe = imx_ccwmx51_wm8753_probe,
	.remove = imx_ccwmx51_wm8753_remove,
	.driver = {
		   .name = "ccwmx51js",
		   },
};

static struct platform_device *imx_ccwmx51_snd_device;

static int __init imx_ccwmx51_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_ccwmx51_wm8753_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_ccwmx51_snd_device = platform_device_alloc("soc-audio", 2);
	if (!imx_ccwmx51_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_ccwmx51_snd_device, &imx_ccwmx51_snd_devdata);
	imx_ccwmx51_snd_devdata.dev = &imx_ccwmx51_snd_device->dev;
	ret = platform_device_add(imx_ccwmx51_snd_device);

	if (ret)
		platform_device_put(imx_ccwmx51_snd_device);

	return ret;
}

static void __exit imx_ccwmx51_exit(void)
{
	platform_driver_unregister(&imx_ccwmx51_wm8753_audio_driver);
	platform_device_unregister(imx_ccwmx51_snd_device);
}

module_init(imx_ccwmx51_init);
module_exit(imx_ccwmx51_exit);

MODULE_AUTHOR("Digi International, Inc.");
MODULE_DESCRIPTION("WM8753 Driver for i.MX CCWMX51");
MODULE_LICENSE("GPL");
