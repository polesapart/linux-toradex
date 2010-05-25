/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 - 2010 Digi International, Inc. All Rights Reserved.
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
#include <linux/pwm_backlight.h>
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
#include "board-ccwmx51.h"
#include "iomux.h"
#include "crm_regs.h"
#include "devices.h"
#include "mx51_pins.h"
#include "displays/displays.h"
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
#endif

#if defined(CONFIG_SMSC9118) || defined(CONFIG_SMSC9118_MODULE)
static struct smc911x_platdata ccwmx51_smsc9118 = {
	.flags          = 0,
	.irq_flags      = IRQF_DISABLED | IRQF_TRIGGER_FALLING,
	.irq_polarity   = 0,
	.irq_type	= 1,	/* push-pull irq */
};
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static int sdhc_write_protect(struct device *dev)
{
     unsigned short rc = 0;

     if (to_platform_device(dev)->id == 0)
             rc = 0; /* Not supported WP on JSK board, therefore write is enabled */
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
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static struct resource mxcfb_resources[] = {
	[0] = {
	       .flags = IORESOURCE_MEM,
	       },
};
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
	struct tag *t;
	struct tag *mem_tag = 0;
	int total_mem = SZ_512M;
	int left_mem = 0;
	int gpu_mem = SZ_64M;
	int fb_mem = SZ_32M;

	mxc_set_cpu_type(MXC_CPU_MX51);

	get_cpu_wp = mx51_get_cpu_wp;
	set_num_cpu_wp = mx51_set_num_cpu_wp;

	for_each_tag(mem_tag, tags) {
		if (mem_tag->hdr.tag == ATAG_MEM) {
			total_mem = mem_tag->u.mem.size;
			left_mem = total_mem - gpu_mem - fb_mem;
			break;
		}
	}

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "mem=");
			if (str != NULL) {
				str += 4;
				left_mem = memparse(str, &str);
				if (left_mem == 0 || left_mem > total_mem)
					left_mem = total_mem - gpu_mem - fb_mem;
			}

			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpu_memory=");
			if (str != NULL) {
				str += 11;
				gpu_mem = memparse(str, &str);
			}

			break;
		}
	}

	if (mem_tag) {
		fb_mem = total_mem - left_mem - gpu_mem;
		if (fb_mem < 0) {
			gpu_mem = total_mem - left_mem;
			fb_mem = 0;
		}
		mem_tag->u.mem.size = left_mem;

		/*reserve memory for gpu*/
		gpu_device.resource[5].start =
				mem_tag->u.mem.start + left_mem;
		gpu_device.resource[5].end =
				gpu_device.resource[5].start + gpu_mem - 1;
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || \
	defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
		if (fb_mem) {
			mxcfb_resources[0].start =
				gpu_device.resource[5].end + 1;
			mxcfb_resources[0].end =
				mxcfb_resources[0].start + fb_mem - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
		}
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
#ifdef CONFIG_MXC_PMIC_MC13892
	pmic_write_reg(REG_POWER_MISC, (PWGT1SPIEN|PWGT2SPIEN),
		(PWGT1SPIEN|PWGT2SPIEN));
#endif
}

static struct i2c_board_info ccwmx51_i2c_devices[] __initdata = {
#if defined(CONFIG_INPUT_MMA7455L) || defined(CONFIG_INPUT_MMA7455L_MODULE)
	{
        I2C_BOARD_INFO("mma7455l", 0x1d),
		.irq = IOMUX_TO_IRQ(MX51_PIN_GPIO1_7),
	},
#endif
#if defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753_MODULE)
	{
        I2C_BOARD_INFO("wm8753", 0x1A),
	},
#endif
};

int __init ccwmx51_init_i2c2(void)
{
	return i2c_register_board_info(1, ccwmx51_i2c_devices , ARRAY_SIZE(ccwmx51_i2c_devices) );
}

static struct mxc_i2c_platform_data mxci2c_data = {
	.i2c_clk = 100000,
};

static struct mxc_i2c_platform_data mxci2c_hs_data = {
	.i2c_clk = 400000,
};

#if defined(CONFIG_SPI_MXC_SELECT1_SS1) && (defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE))
static struct spi_board_info spi_devices[] __initdata = {
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{	/* SPIDEV */
		.modalias       = "spidev",
		.max_speed_hz   = 6000000,
		.bus_num        = 1,
		.chip_select    = 1,
	},
	/* Add here other SPI devices, if any... */
#endif
};

static void ccwmx51_init_spidevices(void)
{
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}
#else
static void ccwmx51_init_spidevices(void) { }
#endif

extern void ccwmx51_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void ccwmx51_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);

static struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = ccwmx51_gpio_spi_chipselect_active,
	.chipselect_inactive = ccwmx51_gpio_spi_chipselect_inactive,
};

static struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = 0x83F98840,
};



static struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
};

#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
static struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};
#endif

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

static struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
	.dig_reg = "VDIG",
};

static struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.ccm_cdcr_reg_addr = MXC_CCM_CDCR,
	.ccm_cacrr_reg_addr = MXC_CCM_CACRR,
	.ccm_cdhipr_reg_addr = MXC_CCM_CDHIPR,
	.dvfs_thrs_reg_addr = MXC_DVFSTHRS,
	.dvfs_coun_reg_addr = MXC_DVFSCOUN,
	.dvfs_emac_reg_addr = MXC_DVFSEMAC,
	.dvfs_cntr_reg_addr = MXC_DVFSCNTR,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 30,
	.num_wp = 3,
};

static struct mxc_dvfsper_data dvfs_per_data = {
	.reg_id = "SW2",
	.clk_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.gpc_adu = 0x0,
	.vai_mask = MXC_DVFSPMCR0_FSVAI_MASK,
	.vai_offset = MXC_DVFSPMCR0_FSVAI_OFFSET,
	.dvfs_enable_bit = MXC_DVFSPMCR0_DVFEN,
	.irq_mask = MXC_DVFSPMCR0_FSVAIM,
	.div3_offset = 0,
	.div3_mask = 0x7,
	.div3_div = 2,
	.lp_high = 1200000,
	.lp_low = 1200000,
};

static struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

static struct mxc_audio_platform_data wm8753_data = {
    .ssi_num = 1,
    .src_port = 2,
    .ext_port = 3,
    .sysclk = 12000000,
};

struct mxc_fb_platform_data mx51_fb_data[] = {
	/*VGA*/
	{
			.interface_pix_fmt = IPU_PIX_FMT_RGB24,
			.mode_str = "1024x768M-16@60",	/* Default */
	}
};
#if defined(CONFIG_UIO_PDRV_GENIRQ) || defined(CONFIG_UIO_PDRV_GENIRQ_MODULE)
static struct uio_info gpu2d_platform_data = {
	.name = "imx_gpu2d",
	.version = "1",
	.irq = MXC_INT_GPU2_IRQ,
	.open = gpu2d_open,
	.release = gpu2d_release,
	.mmap = gpu2d_mmap,
};
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
struct ccwmx51_lcd_pdata * plcd_platform_data;

struct ccwmx51_lcd_pdata * ccwmx51_get_display(char *name)
{
#if defined(CONFIG_CCWMX51_LQ070Y3DG3B) || defined(CONFIG_CCWMX51_CUSTOM)
	int i;

	for (i = 0; i < ARRAY_SIZE(lcd_display_list); i++)
		if (!strncmp(lcd_display_list[i].fb_pdata.mode->name,
			     name, strlen(lcd_display_list[i].fb_pdata.mode->name)))
			return &lcd_display_list[i];
#endif
	return NULL;
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

		/* Get the desired configuration provided by the bootloader */
		if (options[3] != '@') {
			pr_info("Video resolution for VGA interface not provided, using default\n");
			/* TODO set default video here */
		} else {
			options = &options[4];
			if (((p = strsep (&options, "@")) != NULL) && *p) {
				if (!strcmp(p, "640x480x16")) {
					strcpy(mx51_fb_data[0].mode_str, "640x480M-16@60");
				} else if (!strcmp(p, "800x600x16")) {
					strcpy(mx51_fb_data[0].mode_str, "800x600M-16@60");
				} else if (!strcmp(p, "1024x768x16")) {
					strcpy(mx51_fb_data[0].mode_str, "1024x768M-16@60");
				} else if (!strcmp(p, "1280x1024x16")) {
					strcpy(mx51_fb_data[0].mode_str, "1280x1024M-16@60");
				} else
					pr_warning("Unsuported video resolution: %s, using default\n", p);
			}
		}
	} else {
		if ((plcd_platform_data = ccwmx51_get_display(options)) != NULL) {
			memcpy(&mx51_fb_data[0], &plcd_platform_data->fb_pdata, sizeof(struct mxc_fb_platform_data));
			plcd_platform_data->vif = 0;	/* Select video interface 0 */
		}
	}
	return 0;
}
device_initcall(ccwmx51_init_fb);
#endif

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{

	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");

	mxc_spdif_data.spdif_core_clk = clk_get(NULL, "spdif_xtal_clk");
	clk_put(mxc_spdif_data.spdif_core_clk);

	mxc_cpu_common_init();
	mxc_register_gpios();
	ccwmx51_io_init();

	mxc_register_device(&mxc_wdt_device, NULL);
	mxc_register_device(&mxcspi1_device, &mxcspi1_data);
	mxc_register_device(&mxci2c_devices[0], &mxci2c_data);
	mxc_register_device(&mxci2c_devices[1], &mxci2c_data);
	mxc_register_device(&mxci2c_hs_device, &mxci2c_hs_data);
	mxc_register_device(&mxc_rtc_device, &srtc_data);
	mxc_register_device(&mxc_ssi1_device, NULL);
	mxc_register_device(&mxc_ssi2_device, NULL);
	mxc_register_device(&mxc_dma_device, NULL);
#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
	mxc_register_device(&mxc_w1_master_device, &mxc_w1_data);
#endif
	mxc_ipu_data.di_clk[0] = clk_get(NULL, "ipu_di0_clk");
	mxc_ipu_data.di_clk[1] = clk_get(NULL, "ipu_di1_clk");
	mxc_register_device(&mxc_ipu_device, &mxc_ipu_data);
	mxc_register_device(&mxcvpu_device, NULL);
	mxc_register_device(&mxc_alsa_spdif_device, &mxc_spdif_data);
	mxc_register_device(&mxc_tve_device, &tve_data);
	mxc_register_device(&mx51_lpmode_device, NULL);
	mxc_register_device(&busfreq_device, NULL);
	mxc_register_device(&sdram_autogating_device, NULL);
	mxc_register_device(&mxc_dvfs_core_device, &dvfs_core_data);
	mxc_register_device(&mxc_dvfs_per_device, &dvfs_per_data);
	mxc_register_device(&mxc_iim_device, NULL);
	mxc_register_device(&gpu_device, NULL);
#if defined(CONFIG_UIO_PDRV_GENIRQ) || defined(CONFIG_UIO_PDRV_GENIRQ_MODULE)
	mxc_register_device(&mxc_gpu2d_device, &gpu2d_platform_data);
#endif
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm_backlight_device, &mxc_pwm_backlight_data);

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
	/* SD card detect irqs */
	mxcsdhc1_device.resource[2].start = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0);
	mxcsdhc1_device.resource[2].end = IOMUX_TO_IRQ(MX51_PIN_GPIO1_0);
	mxcsdhc3_device.resource[2].start = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND);
	mxcsdhc3_device.resource[2].end = IOMUX_TO_IRQ(MX51_PIN_GPIO_NAND);
	mxc_register_device(&mxcsdhc1_device, &mmc1_data);
	mxc_register_device(&mxcsdhc3_device, &mmc3_data);
#endif
#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
	mxc_register_device(&mxc_fec_device, NULL);
#endif
#if defined(CONFIG_MTD_NAND_MXC) \
	|| defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) \
	|| defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3) \
	|| defined(CONFIG_MTD_NAND_MXC_V3_MODULE)
	mxc_register_device(&mxc_nandv2_mtd_device, &mxc_nand_data);
#endif
#if defined(CONFIG_SMSC9118) || defined(CONFIG_SMSC9118_MODULE)
	mxc_register_device(&smsc911x_device, &ccwmx51_smsc9118);
#endif
#if defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753_MODULE)
	mxc_register_device(&mxc_wm8753_device, &wm8753_data);
#endif
	ccwmx51_init_spidevices();
	ccwmx51_init_i2c2();
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
	mxc_register_device(&lcd_pdev, plcd_platform_data);
	mxc_fb_devices[0].num_resources = ARRAY_SIZE(mxcfb_resources);
	mxc_fb_devices[0].resource = mxcfb_resources;
	mxc_register_device(&mxc_fb_devices[0], &mx51_fb_data[0]);
//	mxc_register_device(&mxc_fb_devices[1], &mx51_fb_data[1]);
//	mxc_register_device(&mxc_fb_devices[2], NULL);
#endif

#ifdef CONFIG_MXC_PMIC_MC13892
	ccwmx51_init_mc13892();
	/* Configure PMIC irq line */
	set_irq_type(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), IRQ_TYPE_EDGE_BOTH);
#endif

	pm_power_off = mxc_power_off;
}

static void __init ccwmx51_timer_init(void)
{
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto[0].cpu_voltage = 1175000;
		cpu_wp_auto[1].cpu_voltage = 1100000;
		cpu_wp_auto[2].cpu_voltage = 1000000;
	}

	mx51_clocks_init(32768, 24000000, 22579200, 24576000);

	uart_clk = clk_get(NULL, "uart_clk.1");
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mxc_timer = {
	.init	= ccwmx51_timer_init,
};

MACHINE_START(CCWMX51JS, "ConnectCore Wi-i.MX51 on a JSK board")
	/* Maintainer: Digi International, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx5_map_io,
	.init_irq = mx5_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
