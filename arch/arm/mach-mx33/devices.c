/*
 * Author: MontaVista Software, Inc.
 *       <source@mvista.com>
 *
 * Based on the OMAP devices.c
 *
 * 2005 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 *
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/spi/spi.h>

#include <asm/hardware.h>
#include <asm/arch/clock.h>
#include <asm/arch/mmc.h>

#include <asm/arch/spba.h>
#include "iomux.h"
#include <asm/arch/sdma.h>
#include "sdma_script_code.h"

#if 0
int board_device_enable(u32 device_id);
int board_device_disable(u32 device_id);

int mxc_device_enable(u32 device_id)
{
	int ret = 0;

	switch (device_id) {
	default:
		ret = board_device_enable(device_id);
	}

	return ret;
}

int mxc_device_disable(u32 device_id)
{
	int ret = 0;

	switch (device_id) {
	default:
		ret = board_device_disable(device_id);
	}
	return ret;
}

EXPORT_SYMBOL(mxc_device_enable);
EXPORT_SYMBOL(mxc_device_disable);
#endif

void mxc_sdma_get_script_info(sdma_script_start_addrs * sdma_script_addr)
{
	sdma_script_addr->mxc_sdma_app_2_mcu_addr = app_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_ap_2_ap_addr = ap_2_ap_ADDR;
	sdma_script_addr->mxc_sdma_ap_2_bp_addr = -1;
	sdma_script_addr->mxc_sdma_bp_2_ap_addr = -1;
	sdma_script_addr->mxc_sdma_loopback_on_dsp_side_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_app_addr = mcu_2_app_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_shp_addr = mcu_2_shp_ADDR;
	sdma_script_addr->mxc_sdma_mcu_interrupt_only_addr = -1;
	sdma_script_addr->mxc_sdma_shp_2_mcu_addr = shp_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_start_addr = (unsigned short *)sdma_code;
	sdma_script_addr->mxc_sdma_uartsh_2_mcu_addr = uartsh_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_uart_2_mcu_addr = uart_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_ram_code_size = RAM_CODE_SIZE;
	sdma_script_addr->mxc_sdma_ram_code_start_addr = RAM_CODE_START_ADDR;
	sdma_script_addr->mxc_sdma_dptc_dvfs_addr = dptc_dvfs_ADDR;
	sdma_script_addr->mxc_sdma_firi_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_firi_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_mshc_2_mcu_addr = -1;
	sdma_script_addr->mxc_sdma_per_2_app_addr = -1;
	sdma_script_addr->mxc_sdma_per_2_firi_addr = -1;
	sdma_script_addr->mxc_sdma_per_2_shp_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_ata_addr = mcu_2_ata_ADDR;
	sdma_script_addr->mxc_sdma_mcu_2_firi_addr = -1;
	sdma_script_addr->mxc_sdma_mcu_2_mshc_addr = -1;
	sdma_script_addr->mxc_sdma_ata_2_mcu_addr = ata_2_mcu_ADDR;
	sdma_script_addr->mxc_sdma_uartsh_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_shp_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_uart_2_per_addr = -1;
	sdma_script_addr->mxc_sdma_app_2_per_addr = -1;
}

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

#if defined(CONFIG_W1_MXC) || defined(CONFIG_W1_MXC_MODULE)
static struct platform_device mxc_w1_devices = {
	.name = "mxc_w1",
	.dev = {
		.release = mxc_nop_release,
		},
	.id = 0
};

static void mxc_init_owire(void)
{
	platform_device_register(&mxc_w1_devices);
}
#else
static inline void mxc_init_owire(void)
{
}
#endif

#if defined(CONFIG_MXC_RTC)

static struct platform_device mxc_rtc_device = {
	.name = "mxc_rtc",
	.id = 0,
};
static void mxc_init_rtc(void)
{
	(void)platform_device_register(&mxc_rtc_device);
}
#else
static inline void mxc_init_rtc(void)
{
}
#endif

#if defined(CONFIG_MXC_WATCHDOG) || defined(CONFIG_MXC_WATCHDOG_MODULE)

static struct resource wdt_resources[] = {
	{
	 .start = WDOG1_BASE_ADDR,
	 .end = WDOG1_BASE_ADDR + 0x30,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device mxc_wdt_device = {
	.name = "mxc_wdt",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource = wdt_resources,
};

static void mxc_init_wdt(void)
{
	(void)platform_device_register(&mxc_wdt_device);
}
#else
static inline void mxc_init_wdt(void)
{
}
#endif

#if defined(CONFIG_MXC_IPU) || defined(CONFIG_MXC_IPU_MODULE)
static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 1,
};

static struct resource ipu_resources[] = {
	{
	 .start = IPU_CTRL_BASE_ADDR,
	 .end = IPU_CTRL_BASE_ADDR + SZ_4K,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = INT_IPU_SYN,
	 .flags = IORESOURCE_IRQ,
	 },
	{
	 .start = INT_IPU_ERR,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device mxc_ipu_device = {
	.name = "mxc_ipu",
	.id = -1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_ipu_data,
		},
	.num_resources = ARRAY_SIZE(ipu_resources),
	.resource = ipu_resources,
};

static void mxc_init_ipu(void)
{
	platform_device_register(&mxc_ipu_device);
}
#else
static inline void mxc_init_ipu(void)
{
}
#endif

/* MMC device data */

#if defined(CONFIG_MMC_MXC) || defined(CONFIG_MMC_MXC_MODULE)

extern unsigned int sdhc_get_card_det_status(struct device *dev);
extern int sdhc_init_card_det(int id);

static struct mxc_mmc_platform_data mmc_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30,
	.min_clk = 150000,
	.max_clk = 25000000,
	.card_inserted_state = 1,
};

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_16K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = INT_MMC_SDHC1,
	       .end = INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
	[3] = {
	       .start = MXC_SDIO1_CARD_IRQ,
	       .end = MXC_SDIO1_CARD_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*!
 * Resource definition for the SDHC2
 */
static struct resource mxcsdhc2_resources[] = {
	[0] = {
	       .start = MMC_SDHC2_BASE_ADDR,
	       .end = MMC_SDHC2_BASE_ADDR + SZ_16K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = INT_MMC_SDHC2,
	       .end = INT_MMC_SDHC2,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ,
	       },
	[3] = {
	       .start = MXC_SDIO2_CARD_IRQ,
	       .end = MXC_SDIO2_CARD_IRQ,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxc_esdhc",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC2 */
static struct platform_device mxcsdhc2_device = {
	.name = "mxc_esdhc",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc_data,
		},
	.num_resources = ARRAY_SIZE(mxcsdhc2_resources),
	.resource = mxcsdhc2_resources,
};

static inline void mxc_init_mmc(void)
{
	int cd_irq = 0;

	/*TODO:cd_irq = sdhc_init_card_det(0); */
	if (cd_irq) {
		mxcsdhc1_device.resource[2].start = cd_irq;
		mxcsdhc1_device.resource[2].end = cd_irq;
	}
	cd_irq = 0;
	/* set cd_irq = 0 here to disable sdhc2 */
	/* TODO:cd_irq = sdhc_init_card_det(1); */
	if (cd_irq) {
		mxcsdhc2_device.resource[2].start = cd_irq;
		mxcsdhc2_device.resource[2].end = cd_irq;
	}

	spba_take_ownership(SPBA_SDHC1, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc1_device);
	spba_take_ownership(SPBA_SDHC2, SPBA_MASTER_A | SPBA_MASTER_C);
	(void)platform_device_register(&mxcsdhc2_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

/* SPI controller and device data */
#if defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE)

#ifdef CONFIG_SPI_MXC_SELECT1
/*!
 * Resource definition for the CSPI1
 */
static struct resource mxcspi1_resources[] = {
	[0] = {
	       .start = CSPI1_BASE_ADDR,
	       .end = CSPI1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = INT_CSPI1,
	       .end = INT_CSPI1,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI1 */
static struct mxc_spi_master mxcspi1_data = {
	.bus_num = 1,
	.maxchipselect = 4,
	.spi_version = 4,
	.clock = CSPI1_CLK,
};

/*! Device Definition for MXC CSPI1 */
static struct platform_device mxcspi1_device = {
	.name = "mxc_spi",
	.id = 1,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi1_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi1_resources),
	.resource = mxcspi1_resources,
};

#endif				/* CONFIG_SPI_MXC_SELECT1 */

#ifdef CONFIG_SPI_MXC_SELECT2
/*!
 * Resource definition for the CSPI2
 */
static struct resource mxcspi2_resources[] = {
	[0] = {
	       .start = CSPI2_BASE_ADDR,
	       .end = CSPI2_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = INT_CSPI2,
	       .end = INT_CSPI2,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI2 */
static struct mxc_spi_master mxcspi2_data = {
	.bus_num = 2,
	.maxchipselect = 4,
	.spi_version = 4,
	.clock = CSPI2_CLK,
};

/*! Device Definition for MXC CSPI2 */
static struct platform_device mxcspi2_device = {
	.name = "mxc_spi",
	.id = 2,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi2_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi2_resources),
	.resource = mxcspi2_resources,
};
#endif				/* CONFIG_SPI_MXC_SELECT2 */

#ifdef CONFIG_SPI_MXC_SELECT3
/*!
 * Resource definition for the CSPI3
 */
static struct resource mxcspi3_resources[] = {
	[0] = {
	       .start = CSPI3_BASE_ADDR,
	       .end = CSPI3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = INT_CSPI3,
	       .end = INT_CSPI3,
	       .flags = IORESOURCE_IRQ,
	       },
};

/*! Platform Data for MXC CSPI3 */
static struct mxc_spi_master mxcspi3_data = {
	.bus_num = 3,
	.maxchipselect = 4,
	.spi_version = 4,
	.clock = CSPI3_CLK,
};

/*! Device Definition for MXC CSPI3 */
static struct platform_device mxcspi3_device = {
	.name = "mxc_spi",
	.id = 3,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxcspi3_data,
		},
	.num_resources = ARRAY_SIZE(mxcspi3_resources),
	.resource = mxcspi3_resources,
};
#endif				/* CONFIG_SPI_MXC_SELECT3 */

static inline void mxc_init_spi(void)
{
	/* SPBA configuration for CSPI2 - MCU is set */
	spba_take_ownership(SPBA_CSPI2, SPBA_MASTER_A);
#ifdef CONFIG_SPI_MXC_SELECT1
	if (platform_device_register(&mxcspi1_device) < 0)
		printk("Error: Registering the SPI Controller_1\n");
#endif				/* CONFIG_SPI_MXC_SELECT1 */
#ifdef CONFIG_SPI_MXC_SELECT2
	if (platform_device_register(&mxcspi2_device) < 0)
		printk("Error: Registering the SPI Controller_2\n");
#endif				/* CONFIG_SPI_MXC_SELECT2 */
#ifdef CONFIG_SPI_MXC_SELECT3
	if (platform_device_register(&mxcspi3_device) < 0)
		printk("Error: Registering the SPI Controller_3\n");
#endif				/* CONFIG_SPI_MXC_SELECT3 */
}
#else
static inline void mxc_init_spi(void)
{
}
#endif

struct mxc_gpio_port mxc_gpio_ports[GPIO_PORT_NUM] = {
	{
	 .num = 0,
	 .base = IO_ADDRESS(GPIO1_BASE_ADDR),
	 .irq_0_15 = INT_GPIO1_LOW,
	 .irq_16_31 = INT_GPIO1_HIGH,
	 .virtual_irq_start = MXC_GPIO_BASE,
	 },
	{
	 .num = 1,
	 .base = IO_ADDRESS(GPIO2_BASE_ADDR),
	 .irq_0_15 = INT_GPIO2_LOW,
	 .irq_16_31 = INT_GPIO2_HIGH,
	 .virtual_irq_start = MXC_GPIO_BASE + GPIO_NUM_PIN * 1,
	 },
	{
	 .num = 2,
	 .base = IO_ADDRESS(GPIO3_BASE_ADDR),
	 .irq_0_15 = INT_GPIO3_LOW,
	 .irq_16_31 = INT_GPIO3_HIGH,
	 .virtual_irq_start = MXC_GPIO_BASE + GPIO_NUM_PIN * 2,
	 },
	{
	 .num = 3,
	 .base = IO_ADDRESS(GPIO4_BASE_ADDR),
	 .irq_0_15 = INT_GPIO4_LOW,
	 .irq_16_31 = INT_GPIO4_HIGH,
	 .virtual_irq_start = MXC_GPIO_BASE + GPIO_NUM_PIN * 3,
	 },
};

#if	defined(CONFIG_PCMCIA_MX33ADS) || defined(CONFIG_PCMCIA_MX33ADS_MODULE)

static struct platform_device mx33ads_device = {
	.name = "Mx33ads pcmcia socket",
	.id = 0,
	.dev.release = mxc_nop_release,
};
static inline void mxc_init_pcmcia(void)
{
	platform_device_register(&mx33ads_device);
}
#else
static inline void mxc_init_pcmcia(void)
{
}
#endif

static struct platform_device mxc_dma_device = {
	.name = "mxc_dma",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
};

static inline void mxc_init_dma(void)
{
	(void)platform_device_register(&mxc_dma_device);
}

static int __init mxc_init_devices(void)
{
	mxc_init_wdt();
	mxc_init_ipu();
	mxc_init_mmc();
	mxc_init_spi();
	mxc_init_rtc();
	mxc_init_owire();
	mxc_init_pcmcia();
	mxc_init_dma();

	/* SPBA configuration for SSI2 - SDMA and MCU are set */
	spba_take_ownership(SPBA_SSI2, SPBA_MASTER_C | SPBA_MASTER_A);
	spba_take_ownership(SPBA_FEC, SPBA_MASTER_A);
	return 0;
}

arch_initcall(mxc_init_devices);
