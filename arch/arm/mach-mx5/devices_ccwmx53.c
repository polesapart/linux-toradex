/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/smsc911x.h>
#include <linux/sysfs.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#include <mach/iomux-mx53.h>
#include <video/ad9389.h>
#include <linux/smc911x.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#endif

#include "devices_ccwmx53.h"
#include "board-ccwmx51.h"
#include "crm_regs.h"
#include "devices.h"


#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	int ret = 0;

#ifdef ESDHC1_WP_GPIO
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(ESDHC1_WP_GPIO);
#endif
#ifdef ESDHC2_WP_GPIO
	if (to_platform_device(dev)->id == 1)
		ret = gpio_get_value(ESDHC2_WP_GPIO);
#endif
#ifdef ESDHC3_WP_GPIO
	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(ESDHC3_WP_GPIO);
#endif
#ifdef ESDHC4_WP_GPIO
	if (to_platform_device(dev)->id == 3)
		ret = gpio_get_value(ESDHC4_WP_GPIO);
#endif
	return ret;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

#ifdef ESDHC1_CD_GPIO
	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(ESDHC1_CD_GPIO);
#endif
#ifdef ESDHC2_CD_GPIO
	if (to_platform_device(dev)->id == 1)
		ret = gpio_get_value(ESDHC2_CD_GPIO);
#endif
#ifdef ESDHC3_CD_GPIO
	if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(ESDHC3_CD_GPIO);
#endif
#ifdef ESDHC4_CD_GPIO
	if (to_platform_device(dev)->id == 3)
		ret = gpio_get_value(ESDHC4_CD_GPIO);
#endif
	return ret;
}

#ifdef CONFIG_ESDHCI_MXC_SELECT1
static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
#endif

#ifdef CONFIG_ESDHCI_MXC_SELECT2
static struct mxc_mmc_platform_data mmc2_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
#endif

#ifdef CONFIG_ESDHCI_MXC_SELECT3
static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30
		| MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
//	.caps = MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA
//		| MMC_CAP_DATA_DDR,
	.min_clk = 400000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
};
#endif


void ccwmx53_register_sdio(int interface)
{
	gpio_sdhc_active(interface);

	switch (interface) {
#ifdef CONFIG_ESDHCI_MXC_SELECT1
	case 0:
#ifdef ESDHC1_CD_GPIO
		mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ_V3(ESDHC1_CD_GPIO);
		mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ_V3(ESDHC1_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc1_device, &mmc1_data);
		break;
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT2
	case 1:
#ifdef ESDHC2_CD_GPIO
		mxcsdhc2_device.resource[2].start = IOMUX_TO_IRQ_V3(ESDHC2_CD_GPIO);
		mxcsdhc2_device.resource[2].end = IOMUX_TO_IRQ_V3(ESDHC2_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc2_device, &mmc2_data);
		break;
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT3
	case 2:
#ifdef ESDHC3_CD_GPIO
		mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ_V3(ESDHC3_CD_GPIO);
		mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ_V3(ESDHC3_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc3_device, &mmc3_data);
		break;
#endif
#ifdef CONFIG_ESDHCI_MXC_SELECT4
	case 3:
#ifdef ESDHC4_CD_GPIO
		mxcsdhc4_device.resource[2].start = IOMUX_TO_IRQ_V3(ESDHC4_CD_GPIO);
		mxcsdhc4_device.resource[2].end = IOMUX_TO_IRQ_V3(ESDHC4_CD_GPIO);
#endif
		mxc_register_device(&mxcsdhc4_device, &mmc4_data);
		break;
#endif
	}
}
#else
void ccwmx53_register_sdio(int interface) {}
#endif

#if defined(CONFIG_MTD_NAND_MXC) \
	|| defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) \
	|| defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3) \
	|| defined(CONFIG_MTD_NAND_MXC_V3_MODULE)

extern void gpio_nand_active(void);
extern void gpio_nand_inactive(void);

static int nand_init(void)
{
      gpio_nand_active();
      return 0;
}

static void nand_exit(void)
{
      gpio_nand_inactive();
}

struct flash_platform_data mxc_nand_data = {
	.width = 1,
	.init = nand_init,
	.exit = nand_exit,
};

void ccwmx53_register_nand(void)
{
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
}
#else
void ccwmx53_register_nand(void) {}
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
struct smsc911x_platform_config ccwmx53_smsc9118 = {
	.flags          = SMSC911X_USE_32BIT,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,    /* push-pull irq */
};

static struct resource smsc911x_device_resources[] = {
        {
		.name	= "smsc911x-memory",
		.start	= MX53_CS1_BASE_ADDR,
		.end	= MX53_CS1_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IOMUX_TO_IRQ_V3(CCWMX53_EXT_IRQ_GPIO),
		.end	= IOMUX_TO_IRQ_V3(CCWMX53_EXT_IRQ_GPIO),
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device smsc911x_device = {
	.name             = "smsc911x",
	.id               = -1,
	.num_resources    = ARRAY_SIZE(smsc911x_device_resources),
	.resource         = smsc911x_device_resources,
};

/* WEIM registers */
#define CSGCR1	0x00
#define CSGCR2	0x04
#define CSRCR1	0x08
#define CSRCR2	0x0C
#define CSWCR1	0x10
#define CSWCR2	0x14

void ccwmx53_register_ext_eth(void)
{
	__iomem void *weim_vbaddr, *iomux_vbaddr;
	u32 reg;

	gpio_smsc911x_active();

	weim_vbaddr = ioremap(WEIM_BASE_ADDR, SZ_4K);
	if (weim_vbaddr == 0) {
		printk(KERN_ERR "Unable to ioremap 0x%08x in %s\n", WEIM_BASE_ADDR, __func__);
		return;
	}

	iomux_vbaddr = ioremap(IOMUXC_BASE_ADDR, SZ_4K);
	if (iomux_vbaddr == 0) {
		printk(KERN_ERR "Unable to ioremap 0x%08x in %s\n", IOMUXC_BASE_ADDR, __func__);
		goto unmap_weim;
	}

	/** Configure the CS timming, bus width, etc.
	 * 16 bit on DATA[31..16], not multiplexed, async
	 * RWSC=50, RADVA=2, RADVN=6, OEA=0, OEN=0, RCSA=0, RCSN=0, APR=0
	 * WAL=0, WBED=1, WWSC=50, WADVA=2, WADVN=6, WEA=0, WEN=0, WCSA=0
	 */
	__raw_writel(0x00420081, (u32)weim_vbaddr + 0x18 + CSGCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x18 + CSGCR2);
	__raw_writel(0x32260000, (u32)weim_vbaddr + 0x18 + CSRCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x18 + CSRCR2);
	__raw_writel(0x72080f00, (u32)weim_vbaddr + 0x18 + CSWCR1);
	__raw_writel(0, (u32)weim_vbaddr + 0x18 + CSWCR2);

	/* Configure CS1 iomem with 32MB length */
	reg = __raw_readl(iomux_vbaddr + 0x4);
	reg &= ~0x3f;
	reg |= 0x1b;
	__raw_writel(reg, (u32)iomux_vbaddr + 0x4);

	mxc_register_device(&smsc911x_device, &ccwmx53_smsc9118);

	iounmap(iomux_vbaddr);
unmap_weim:
	iounmap(weim_vbaddr);
}
#else
void ccwmx53_register_ext_eth(void)
#endif
