/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Digi International, Inc. All Rights Reserved.
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
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/spba.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include "board.h"
#include "iomux.h"
#include "crm_regs.h"


#include <linux/smc911x.h>


#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>
#endif

extern void __init ccwmx51_io_init(void);
extern int __init ccwmx51_init_mc13892(void);
extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1175000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1100000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 166250000,
	 .pdf = 4,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 4,
	 .cpu_voltage = 850000,},
};

struct cpu_wp *mx51_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}
static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

/* MTD NAND flash */
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
	/* Configure the pins */
	gpio_nand_active();
	return 0;
}

static void nand_exit(void)
{
	/* Free the pins */
	gpio_nand_inactive();
}

static struct flash_platform_data mxc_nand_data = {
	.width = 1,
	.init = nand_init,
	.exit = nand_exit,
};

static struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
	},
};

static void ccwmx51_init_nand_mtd(void)
{
	(void)platform_device_register(&mxc_nandv2_mtd_device);
}
#else
static inline void ccwmx51_init_nand_mtd(void) { }
#endif

#if defined(CONFIG_SMSC9118)
static struct resource smsc911x_device_resources[] = {
	[0] = {
		.name	= "smsc911x-memory",
		.start	= CS5_BASE_ADDR,
		.end	= CS5_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IOMUX_TO_IRQ(MX51_PIN_GPIO1_9),
		.end	= IOMUX_TO_IRQ(MX51_PIN_GPIO1_9),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct smc911x_platdata ccwmx51_smsc9118 = {
	.flags          = SMC911X_USE_16BIT,
	.irq_flags      = IRQF_DISABLED | IRQF_TRIGGER_FALLING,
	.irq_polarity   = 0,
};

static struct platform_device smsc911x_device = {
        .name             = "smsc911x",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(smsc911x_device_resources),
        .resource         = smsc911x_device_resources,
        .dev            = {
                .platform_data = &ccwmx51_smsc9118,
        }
};

/* WEIM registers */
#define CSGCR1	0x00
#define CSGCR2	0x04
#define CSRCR1	0x08
#define CSRCR2	0x0C
#define CSWCR1	0x10

static void ccwmx51_init_ext_eth_mac(void)
{
	__iomem u32 *weim_vbaddr;

	weim_vbaddr = ioremap(WEIM_BASE_ADDR, SZ_4K);
	if (weim_vbaddr == 0) {
		printk(KERN_ERR "Unable to ioremap 0x%08x in %s\n", WEIM_BASE_ADDR, __func__);
		return;
	}

	/** Configure the CS timming, bus width, etc.
	 * 16 bit on DATA[31..16], not multiplexed, async
	 * RWSC=50, RADVA=2, RADVN=6, OEA=0, OEN=0, RCSA=0, RCSN=0, APR=0
	 * WAL=0, WBED=1, WWSC=50, WADVA=2, WADVN=6, WEA=0, WEN=0, WCSA=0
	 */
	__raw_writel(0x00420081, weim_vbaddr + 0x78 + CSGCR1);
	__raw_writel(0, weim_vbaddr + 0x78 + CSGCR2);
	__raw_writel(0x32260000, weim_vbaddr + 0x78 + CSRCR1);
	__raw_writel(0, weim_vbaddr + 0x78 + CSRCR2);
	__raw_writel(0x72080f00, weim_vbaddr + 0x78 + CSWCR1);

	iounmap(weim_vbaddr);

	/* Configure interrupt line as GPIO input, the iomux should be already setup */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_9), "LAN2-irq");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_9));

	(void)platform_device_register(&smsc911x_device);
}
#else
static void ccwmx51_init_ext_eth_mac(void) { }
#endif


#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
unsigned int expio_intr_fec;

EXPORT_SYMBOL(expio_intr_fec);
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = 1;	/* Not supported WP on JSK board */
	else if (to_platform_device(dev)->id == 2)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS1));

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0)
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
	else if (to_platform_device(dev)->id == 2)
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND));

	return ret;
}

static struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 52000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

static struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

/**
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
		.start = MMC_SDHC1_BASE_ADDR,
		.end   = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_MMC_SDHC1,
		.end   = MXC_INT_MMC_SDHC1,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
		.end   = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
		.flags = IORESOURCE_IRQ,
	},
};

/**
 * Resource definition for the SDHC3
 */
static struct resource mxcsdhc3_resources[] = {
	[0] = {
		.start = MMC_SDHC3_BASE_ADDR,
		.end   = MMC_SDHC3_BASE_ADDR + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = MXC_INT_MMC_SDHC3,
		.end   = MXC_INT_MMC_SDHC3,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND),
		.end   = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND),
		.flags = IORESOURCE_IRQ,
	},
};

/*! Device Definition for MXC SDHC1 */
static struct platform_device mxcsdhc1_device = {
	.name = "mxsdhci",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc1_data,
	},
	.num_resources = ARRAY_SIZE(mxcsdhc1_resources),
	.resource = mxcsdhc1_resources,
};

/*! Device Definition for MXC SDHC3 */
static struct platform_device mxcsdhc3_device = {
	.name = "mxsdhci",
	.id = 2,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mmc3_data,
	},
	.num_resources = ARRAY_SIZE(mxcsdhc3_resources),
	.resource = mxcsdhc3_resources,
};

static inline void ccwmx51_init_mmc(void)
{
	(void)platform_device_register(&mxcsdhc1_device);
	(void)platform_device_register(&mxcsdhc3_device);
}
#else
static inline void ccwmx51_init_mmc(void) {}
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)

static struct mxc_fb_platform_data fb_data_vga = {
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1024x768M-16@60",	/* Default */
};

static struct platform_device mxc_fb_device[] = {
	{
		.name = "mxc_sdc_fb",
		.id = 0,
		.dev = {
			.release = mxc_nop_release,
			.coherent_dma_mask = 0xFFFFFFFF,
			.platform_data = &fb_data_vga,
		},
	 }, {
		.name = "mxc_sdc_fb",
		.id = 1,
		.dev = {
			.release = mxc_nop_release,
			.coherent_dma_mask = 0xFFFFFFFF,
			.platform_data = NULL,
		},
	}, {
		.name = "mxc_sdc_fb",
		.id = 2,
		.dev = {
			.release = mxc_nop_release,
			.coherent_dma_mask = 0xFFFFFFFF,
		},
	},
};

static int __init ccwmx51_init_fb(void)
{
	char *options = NULL, *p;

	if (fb_get_options("displayfb", &options))
		pr_warning("no display information available in commnad line\n");

	if (!options)
		return -ENODEV;

	if (!strncasecmp(options, "VGA", 3)) {
		pr_info("VGA interface is primary\n");

		/* Get the desired configuration provided by the bootloader */
		if (options[3] != '@') {
			pr_info("Video resolution for VGA interface not provided, using default\n");
			/* TODO set default video here */
		} else {
			options = &options[4];
			if (((p = strsep (&options, "@")) != NULL) && *p) {
				if (!strcmp(p, "640x480x16")) {
					strcpy(fb_data_vga.mode_str, "640x480M-16@60");
				} else if (!strcmp(p, "800x600x16")) {
					strcpy(fb_data_vga.mode_str, "800x600M-16@60");
				} else if (!strcmp(p, "1024x768x16")) {
					strcpy(fb_data_vga.mode_str, "1024x768M-16@60");
				} else if (!strcmp(p, "1280x1024x16")) {
					strcpy(fb_data_vga.mode_str, "1280x1024M-16@60");
				} else if (!strcmp(p, "1280x1024x16")) {
					strcpy(fb_data_vga.mode_str, "1280x1024M-16@60");
				} else
					pr_warning("Unsuported video resolution: %s, using default\n", p);
			}
		}
		(void)platform_device_register(&mxc_fb_device[0]); /* VGA */
	}

	return 0;
}
device_initcall(ccwmx51_init_fb);
#else
static inline void ccwmx51_init_fb(void) { }
#endif



/*!
 * Board specific fixup function. It is called by \b setup_arch() in
 * setup.c file very early on during kernel starts. It allows the user to
 * statically fill in the proper values for the passed-in parameters. None of
 * the parameters is used currently.
 *
 * @param  desc         pointer to \b struct \b machine_desc
 * @param  tags         pointer to \b struct \b tag
 * @param  cmdline      pointer to the command line
 * @param  mi           pointer to \b struct \b meminfo
 */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	char *str;
	int size = SZ_512M - SZ_32M;
	struct tag *t;

	mxc_cpu_init();

	get_cpu_wp = mx51_get_cpu_wp;
	set_num_cpu_wp = mx51_set_num_cpu_wp;

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_CMDLINE)
			continue;
		str = t->u.cmdline.cmdline;
		str = strstr(str, "mem=");
		if (str != NULL) {
			str += 4;
			size = memparse(str, &str);
			if (size == 0 || size == SZ_512M)
				return;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag != ATAG_MEM)
			continue;

		t->u.mem.size = size;
	}
}

#define PWGT1SPIEN (1<<15)
#define PWGT2SPIEN (1<<16)
#define USEROFFSPI (1<<3)

static void mxc_power_off(void)
{
	/* We can do power down one of two ways:
	   Set the power gating
	   Set USEROFFSPI */

	/* Set the power gate bits to power down */
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
}

static struct i2c_board_info ccwmx51_i2c_devices[] __initdata = {
#if defined(CONFIG_INPUT_MMA7455L)
	{
        I2C_BOARD_INFO("mma7455l", 0x1d),
		.irq = IOMUX_TO_IRQ(MX51_PIN_GPIO1_7),
	},
#endif
};

int __init ccwmx51_init_mma7455l(void)
{

	return i2c_register_board_info(1, ccwmx51_i2c_devices , ARRAY_SIZE(ccwmx51_i2c_devices) );
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
	mxc_register_gpios();
	ccwmx51_io_init();
	early_console_setup(saved_command_line);

	mxc_init_devices();
	ccwmx51_init_mmc();
	ccwmx51_init_nand_mtd();
//	ccwmx51_init_ext_eth_mac();
	ccwmx51_init_mma7455l();
	ccwmx51_init_mc13892();

	pm_power_off = mxc_power_off;
}

static void __init ccwmx51_timer_init(void)
{
	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mxc_clocks_init(32768, 24000000, 22579200, 24576000);
	mxc_timer_init("gpt_clk.0");
}

static struct sys_timer mxc_timer = {
	.init	= ccwmx51_timer_init,
};

MACHINE_START(CCWMX51JS, "ConnectCore Wi-MX51 on a JSK board")
	/* Maintainer: Digi International, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mxc_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
