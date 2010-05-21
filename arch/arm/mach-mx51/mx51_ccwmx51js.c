/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Digi International, Inc. All Rights Reserved.
 * Copyright 2010 Timesys Corporation. All Rights Reserved.
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
#include <asm/mach/keypad.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <mach/mmc.h>
#include "board-mx51_ccwmx51js.h"
#include "iomux.h"
#include "crm_regs.h"
#include <mach/mxc_edid.h>
#include <linux/smc911x.h>

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif


/*!
 * @file mach-mx51/mx51_ccwmx51js.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX51
 */
extern void __init mx51_ccwmx51js_io_init(void);
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

struct cpu_wp *mx51_ccwmx51js_get_cpu_wp(int *wp)
{
	*wp = num_cpu_wp;
	return cpu_wp_auto;
}

void mx51_ccwmx51js_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}
static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}


#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)

static struct fb_videomode wvga_video_mode =
{
	.name = "Digi LCD",
	.xres = 800,
	.yres = 480,
	.refresh = 60,
	.pixclock = 30062,
	.left_margin    = 64,
	.right_margin   = 64,
	.lower_margin   = 10,
	.upper_margin   = 30,
	.hsync_len      = 128,
	.vsync_len      = 5,
	.vmode          = FB_VMODE_NONINTERLACED,
	.flag           = FB_MODE_IS_DETAILED,
// Digi says that inverting the clock is necessary
// to avoid problems with video.
//	.sync		= FB_SYNC_CLK_LAT_FALL,

};

static struct mxc_fb_platform_data fb_data_vga = {
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
};

static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
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
	 .num_resources = ARRAY_SIZE(mxcfb_resources),
	 .resource = mxcfb_resources,
	 },
	{
	 .name = "mxc_sdc_fb",
	 .id = 1,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 },
	 },
	{
	 .name = "mxc_sdc_fb",
	 .id = 2,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 },
	 },
};

extern void gpio_lcd_active(void);

// We let the user specify e.g. 800x600x24, but pay attention
// only to the 800x600 part, and use 24-bit color regardless.
static int video_matches(char *user, char *template)
{
	return !strncasecmp(user, template, strlen(template));
}

static int __init ccwmx51_init_fb(void)
{
	char *options = NULL, *p;

	if (fb_get_options("displayfb", &options))
		pr_warning("no display information available in command line\n");

	if (!options)
		return -ENODEV;

	if (!strncasecmp(options, "VGA", 3)) {
		pr_info("VGA interface is primary\n");

		fb_data_vga.mode = 0; // Do not use LCD timings.
		fb_data_vga.mode_str = "1024x768M-16@60";

		/* Get the desired configuration provided by the bootloader */
		if (options[3] != '@') {
			pr_info("Video resolution for VGA interface not provided, using default\n");
		} else {
			options = &options[4];
			if (((p = strsep (&options, "@")) != NULL) && *p) {
				if (video_matches(p, "640x480")) {
					strcpy(fb_data_vga.mode_str, "640x480M-16@60");
				} else if (video_matches(p, "800x600")) {
					strcpy(fb_data_vga.mode_str, "800x600M-16@60");
				} else if (video_matches(p, "1024x768")) {
					strcpy(fb_data_vga.mode_str, "1024x768M-16@60");
				} else if (video_matches(p, "1280x1024")) {
					strcpy(fb_data_vga.mode_str, "1280x1024M-16@60");
				} else if (video_matches(p, "1280x1024")) {
					strcpy(fb_data_vga.mode_str, "1280x1024M-16@60");
				} else
					pr_warning("Unsuported video resolution: %s, using default\n", p);
			}
		}
		(void)platform_device_register(&mxc_fb_device[0]); /* VGA */

	} else if (!strncasecmp(options, "LCD", 3)){
		gpio_lcd_active();
		fb_data_vga.mode = &wvga_video_mode;  // Use timings for Digi LCD.
		fb_data_vga.mode_str = "800x480-16@60", // 16-bit color more compatible with Factory apps
		pr_info("Using LDC wvga video timings and mode %s\n", fb_data_vga.mode_str);
		platform_device_register(&mxc_fb_device[0]); /* LCD */
	}

	return 0;
}
device_initcall(ccwmx51_init_fb);
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT2
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{I2C_BOARD_INFO("wm8753",0x1a)},
};
#endif

#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
static struct i2c_board_info mxc_i2c_hs_board_info[] __initdata = {
};
#endif

#endif

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)

extern void gpio_nand_active(void);
extern void gpio_nand_inactive(void);

#if defined(CONFIG_MTD_NAND_MXC_V3)
static struct mtd_partition __initdata ccwmx51js_nand_partition[] = {
	{
		.name	= "U-Boot",
		.offset	= 0,
		.size	= SZ_256K + SZ_512K,
	},
	{
		.name	= "U-Boot Environment",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= SZ_512K,
	},
	{
		.name	= "Kernel",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= 40 * SZ_1M,
	},
	{
		.name	= "RFS",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

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
#ifdef CONFIG_MTD_PARTITIONS
	.parts = ccwmx51js_nand_partition,
	.nr_parts = ARRAY_SIZE(ccwmx51js_nand_partition),
#endif
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

#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
	unsigned short rc = 0;

	if (to_platform_device(dev)->id == 0)
		rc = 0;	/* Not supported WP on JSK board */
	else if (to_platform_device(dev)->id == 2)
		rc = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_NANDF_CS1));

	return rc;
}

static unsigned int sdhc_get_card_det_status(struct device *dev)
{
	int ret = 0;

	if (to_platform_device(dev)->id == 0) {
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
	} else if (to_platform_device(dev)->id == 2) {
		ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND));
	}

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

/*!
 * Resource definition for the SDHC1
 */
static struct resource mxcsdhc1_resources[] = {
	[0] = {
	       .start = MMC_SDHC1_BASE_ADDR,
	       .end = MMC_SDHC1_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC1,
	       .end = MXC_INT_MMC_SDHC1,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
	       .end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0),
	       .flags = IORESOURCE_IRQ,
	       },
};

/*!
 * Resource definition for the SDHC3
 */
static struct resource mxcsdhc3_resources[] = {
	[0] = {
	       .start = MMC_SDHC3_BASE_ADDR,
	       .end = MMC_SDHC3_BASE_ADDR + SZ_4K - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .start = MXC_INT_MMC_SDHC3,
	       .end = MXC_INT_MMC_SDHC3,
	       .flags = IORESOURCE_IRQ,
	       },
	[2] = {
	       .start = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND),
	       .end = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND),
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

static inline void mxc_init_mmc(void)
{
	(void)platform_device_register(&mxcsdhc1_device);
	(void)platform_device_register(&mxcsdhc3_device);
}
#else
static inline void mxc_init_mmc(void)
{
}
#endif

#if defined CONFIG_FEC
static struct resource mxc_fec_resources[] = {
	{
		.start	= FEC_BASE_ADDR,
		.end	= FEC_BASE_ADDR + 0xfff,
		.flags	= IORESOURCE_MEM
	}, {
		.start	= MXC_INT_FEC,
		.end	= MXC_INT_FEC,
		.flags	= IORESOURCE_IRQ
	},
};

struct platform_device mxc_fec_device = {
	.name = "fec",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_fec_resources),
	.resource = mxc_fec_resources,
};

static __init int mxc_init_fec(void)
{
	return platform_device_register(&mxc_fec_device);
}
#else
static inline int mxc_init_fec(void)
{
	return 0;
}
#endif

#if defined(CONFIG_SMSC911X)
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

static struct smc911x_platdata ccwmx51_smsc911x = {
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
		.platform_data = &ccwmx51_smsc911x,
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

	get_cpu_wp = mx51_ccwmx51js_get_cpu_wp;
	set_num_cpu_wp = mx51_ccwmx51js_set_num_cpu_wp;

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
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		mxcfb_resources[0].start = t->u.mem.start + size;
		mxcfb_resources[0].end = t->u.mem.start + SZ_512M - 1;
#endif
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

static struct mxc_audio_platform_data wm8753_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.sysclk = 11289600,  // So we can do 44.1 kHz
};

static struct platform_device mxc_wm8753_device = {
	.name = "imx-ccwmx51js-wm8753",
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &wm8753_data,
	},
};

static void mxc_init_wm8753(void)
{
	platform_device_register(&mxc_wm8753_device);
}


/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ccwmx51js_buttons[] = {
	{
		.gpio		= IOMUX_TO_GPIO(MX51_PIN_GPIO1_8),
		.code		= BTN_1,
		.desc		= "Button 1",
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		.gpio		= IOMUX_TO_GPIO(MX51_PIN_GPIO1_1),
		.code		= BTN_2,
		.desc		= "Button 2",
		.active_low	= 1,
		.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data ccwmx51js_button_data = {
	.buttons	= ccwmx51js_buttons,
	.nbuttons	= ARRAY_SIZE(ccwmx51js_buttons),
};

static struct platform_device ccwmx51js_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ccwmx51js_button_data,
	}
};

static void __init ccwmx51js_add_device_buttons(void)
{
	platform_device_register(&ccwmx51js_button_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif


#if defined(CONFIG_NEW_LEDS)

/*
 * GPIO LEDs
 */
static struct gpio_led_platform_data led_data;

static struct gpio_led ccwmx51js_leds[] = {
	{
		.name			= "LED1",
		.gpio			= IOMUX_TO_GPIO(MX51_PIN_NANDF_RB2),
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{
		.name			= "LED2",
		.gpio			= IOMUX_TO_GPIO(MX51_PIN_NANDF_RB1),
		.active_low		= 1,
		.default_trigger	= "none",
	}
};

static struct platform_device ccwmx51js_gpio_leds_device = {
	.name			= "leds-gpio",
	.id			= -1,
	.dev.platform_data	= &led_data,
};

void __init ccwmx51js_gpio_leds(struct gpio_led *leds, int nr)
{
	if (!nr)
		return;

	led_data.leds = leds;
	led_data.num_leds = nr;
	platform_device_register(&ccwmx51js_gpio_leds_device);
}

#else
void __init at91_gpio_leds(struct gpio_led *leds, int nr) {}
#endif



/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
	mxc_register_gpios();
	mx51_ccwmx51js_io_init();
	early_console_setup(saved_command_line);

	mxc_init_devices();

	mxc_init_mmc();
	mx51_ccwmx51js_init_mc13892();
	mxc_init_wm8753();

#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
	ccwmx51_init_nand_mtd();
#endif

	ccwmx51_init_ext_eth_mac();

#if defined(CONFIG_I2C_MXC) || defined(CONFIG_I2C_MXC_MODULE)

#ifdef CONFIG_I2C_MXC_SELECT2
	i2c_register_board_info(1, mxc_i2c1_board_info,
				ARRAY_SIZE(mxc_i2c1_board_info));
#endif
#if defined(CONFIG_I2C_MXC_HS) || defined(CONFIG_I2C_MXC_HS_MODULE)
	i2c_register_board_info(3, mxc_i2c_hs_board_info,
				ARRAY_SIZE(mxc_i2c_hs_board_info));
#endif

#endif
	pm_power_off = mxc_power_off;
	mxc_init_fec();
	ccwmx51js_add_device_buttons();
	ccwmx51js_gpio_leds(ccwmx51js_leds, ARRAY_SIZE(ccwmx51js_leds));
}

static void __init mx51_ccwmx51js_timer_init(void)
{
	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);
}

static struct sys_timer mxc_timer = {
	.init	= mx51_ccwmx51js_timer_init,
};

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX51_CCWMX51JS data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX51_CCWMX51JS, "Digi ConnectCore Wi-MX51 on a JSK Board")
	/* Maintainer: Digi International, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx51_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
