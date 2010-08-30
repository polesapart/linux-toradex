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
#include <video/ad9389.h>
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

static u8 ccwmx51_mod_variant = 0;
static u8 ccwmx51_mod_rev = 0;
static u32 ccwmx51_mod_sn = 0;
static u8 ccwmx51_bb_rev = BASE_BOARD_REV;

void ccwmx51_set_mod_variant(u8 variant)
{
	ccwmx51_mod_variant = variant;
}
void ccwmx51_set_mod_revision(u8 revision)
{
	ccwmx51_mod_rev = revision;
}
void ccwmx51_set_mod_sn(u32 sn)
{
	ccwmx51_mod_sn = sn;
}

#ifdef CONFIG_SYSFS
static ssize_t ccwmx51_mod_variant_attr_show(struct kobject *kobj,
					     struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccwmx51_mod_variant);
}

static ssize_t ccwmx51_mod_rev_attr_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccwmx51_mod_rev);
}

static ssize_t ccwmx51_mod_sn_attr_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccwmx51_mod_sn);
}

static ssize_t cccwmx51_bb_rev_attr_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccwmx51_bb_rev);
}

static struct kobj_attribute ccwmx51_mod_variant_attr =
	__ATTR(mod_variant, S_IRUGO, ccwmx51_mod_variant_attr_show, NULL);
static struct kobj_attribute ccwmx51_mod_rev_attr =
	__ATTR(mod_rev, S_IRUGO, ccwmx51_mod_rev_attr_show, NULL);
static struct kobj_attribute ccwmx51_mod_sn_attr =
	__ATTR(mod_sn, S_IRUGO, ccwmx51_mod_sn_attr_show, NULL);
static struct kobj_attribute ccwmx51_bb_rev_attr =
	__ATTR(bb_rev, S_IRUGO, cccwmx51_bb_rev_attr_show, NULL);

static int ccwmx51_create_sysfs_entries(void)
{
	struct kobject *ccwmx51_kobj;
	int ret;

	ccwmx51_kobj = kobject_create_and_add("ccwmx51", kernel_kobj);
	if (!ccwmx51_kobj) {
		printk(KERN_WARNING "kobject_create_and_add ccwmx51 failed\n");
		return -EINVAL;
	}

	ret = sysfs_create_file(ccwmx51_kobj, &ccwmx51_mod_variant_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for ccwmx51 hardware variant\n");
		return ret;
	}
	ret = sysfs_create_file(ccwmx51_kobj, &ccwmx51_mod_rev_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for ccwmx51 hardware revision\n");
		return ret;
	}
	ret = sysfs_create_file(ccwmx51_kobj, &ccwmx51_mod_sn_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for ccwmx51 hardware SN\n");
		return ret;
	}
	ret = sysfs_create_file(ccwmx51_kobj, &ccwmx51_bb_rev_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for ccwmx51 base board hardware revision\n");
		return ret;
	}

	return 0;
}
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
       /* Configure the pins */
      gpio_nand_active();
      return 0;
}

static void nand_exit(void)
{
      /* Free the pins */
      gpio_nand_inactive();
}

struct flash_platform_data mxc_nand_data = {
	.width = 1,
	.init = nand_init,
	.exit = nand_exit,
};
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
struct smsc911x_platform_config ccwmx51_smsc9118 = {
	.flags          = SMSC911X_USE_32BIT,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,    /* push-pull irq */
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
     int ret = 1;

     if (to_platform_device(dev)->id == 0)
#ifdef CONFIG_JSCCWMX51_V1
	ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO1_0));
#else
	ret = 0;
#endif
     else if (to_platform_device(dev)->id == 2)
         ret = gpio_get_value(IOMUX_TO_GPIO(MX51_PIN_GPIO_NAND));
     return ret;
}

struct mxc_mmc_platform_data mmc1_data = {
	.ocr_mask = MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 400000,
	.max_clk = 52000000,
	.card_inserted_state = 0,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};

struct mxc_mmc_platform_data mmc3_data = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29 | MMC_VDD_29_30 |
	    MMC_VDD_31_32,
	.caps = MMC_CAP_4_BIT_DATA,
	.min_clk = 150000,
	.max_clk = 50000000,
	.card_inserted_state = 1,
	.status = sdhc_get_card_det_status,
	.wp_status = sdhc_write_protect,
	.clock_mmc = "esdhc_clk",
	.power_mmc = NULL,
};
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
struct resource mxcfb_resources[2] = {
	{
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = 0,
	},
	{
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end   = 0,
	},
};
#endif

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

#if defined (CONFIG_MXC_CAMERA_MICRON111_1) || defined(CONFIG_MXC_CAMERA_MICRON111_1_MODULE)
	{
		I2C_BOARD_INFO("mt9v111_1", 0xB8>>1),
	},
#endif
#if defined (CONFIG_MXC_CAMERA_MICRON111_2) || defined(CONFIG_MXC_CAMERA_MICRON111_2_MODULE)
	{
		I2C_BOARD_INFO("mt9v111_2", 0x90>>1),
	},
#endif
};

int __init ccwmx51_init_i2c2(void)
{
	return i2c_register_board_info(1, ccwmx51_i2c_devices , ARRAY_SIZE(ccwmx51_i2c_devices) );
}

struct mxc_i2c_platform_data mxci2c_data = {
	.i2c_clk = 100000,
};

struct mxc_i2c_platform_data mxci2c_hs_data = {
	.i2c_clk = 400000,
};

#if defined(CONFIG_SPI_MXC_SELECT1_SS1) && (defined(CONFIG_SPI_MXC) || defined(CONFIG_SPI_MXC_MODULE))
#if defined(CONFIG_CCWMX51_SECOND_TOUCH)
static int touch_pendown_state(void)
{
	return gpio_get_value(IOMUX_TO_GPIO(SECOND_TS_IRQ_PIN)) ? 0 : 1;
}

static struct ads7846_platform_data ccwmx51js_touch_data = {
	.model			= 7843,
	.x_min			= 100,
	.y_min			= 100,
	.x_max			= 4000,
	.y_max			= 4000,
	.vref_delay_usecs	= 100,
	.x_plate_ohms		= 450,
	.y_plate_ohms		= 250,
	.pressure_max		= 15000,
	.debounce_max		= 1,
	.debounce_rep		= 0,
	.debounce_tol		= (~0),
	.settle_delay_usecs	= 20,
	.get_pendown_state	= touch_pendown_state,
};

static struct spi_board_info ccwmx51_2nd_touch[] = {
	{
		.modalias	= "ads7846",
		.max_speed_hz	= 1000000,
		.irq		= IOMUX_TO_IRQ(SECOND_TS_IRQ_PIN),
		.bus_num        = 1,
		.chip_select    = 3,
		.platform_data	= &ccwmx51js_touch_data,
	},
};

void ccwmx51_init_2nd_touch(void)
{
	ccwmx51_2nd_touch_gpio_init();
	spi_register_board_info(ccwmx51_2nd_touch, ARRAY_SIZE(ccwmx51_2nd_touch));
}
#else
void ccwmx51_init_2nd_touch(void) {}
#endif

static struct spi_board_info spi_devices[] = {
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{       /* SPIDEV */
		.modalias	= "spidev",
		.max_speed_hz	= 6000000,
		.bus_num	= 1,
		.chip_select	= 1,
	},
#endif
        /* Add here other SPI devices, if any... */
};

void ccwmx51_init_spidevices(void)
{
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));
}
#else
void ccwmx51_init_spidevices(void) { }
#endif

extern void ccwmx51_gpio_spi_chipselect_active(int cspi_mode, int status,
						    int chipselect);
extern void ccwmx51_gpio_spi_chipselect_inactive(int cspi_mode, int status,
						      int chipselect);

struct mxc_spi_master mxcspi1_data = {
	.maxchipselect = 4,
	.spi_version = 23,
	.chipselect_active = ccwmx51_gpio_spi_chipselect_active,
	.chipselect_inactive = ccwmx51_gpio_spi_chipselect_inactive,
};

struct mxc_srtc_platform_data srtc_data = {
	.srtc_sec_mode_addr = 0x83F98840,
};



struct mxc_ipu_config mxc_ipu_data = {
	.rev = 2,
};

#if defined(CONFIG_W1_MASTER_MXC) || defined(CONFIG_W1_MASTER_MXC_MODULE)
struct mxc_w1_config mxc_w1_data = {
	.search_rom_accelerator = 1,
};
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)

static struct resource smsc911x_device_resources[] = {
        {
		.name	= "smsc911x-memory",
		.start	= CS5_BASE_ADDR,
		.end	= CS5_BASE_ADDR + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IOMUX_TO_IRQ(MX51_PIN_GPIO1_9),
		.end	= IOMUX_TO_IRQ(MX51_PIN_GPIO1_9),
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

static void __init ccwmx51_init_ext_eth_mac(void)
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
	writel(0x00420081, (unsigned int)(weim_vbaddr) + 0x78 + CSGCR1);
	writel(0, (unsigned int)(weim_vbaddr) + 0x78 + CSGCR2);
	writel(0x32260000, (unsigned int)(weim_vbaddr) + 0x78 + CSRCR1);
	writel(0, (unsigned int)(weim_vbaddr) + 0x78 + CSRCR2);
	writel(0x72080f00, (unsigned int)(weim_vbaddr) + 0x78 + CSWCR1);

	iounmap(weim_vbaddr);

	/* Configure interrupt line as GPIO input, the iomux should be already setup */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_9), "LAN2-irq");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_9));
}
#endif

#if defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753_MODULE)
struct platform_device mxc_wm8753_device = {
	.name = "ccwmx51js",
};
#endif

struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx = 1,
	.spdif_rx = 0,
	.spdif_clk_44100 = 0,	/* spdif_ext_clk source for 44.1KHz */
	.spdif_clk_48000 = 7,	/* audio osc source */
	.spdif_clkid = 0,
	.spdif_clk = NULL,	/* spdif bus clk */
};

struct tve_platform_data tve_data = {
	.dac_reg = "VVIDEO",
	.dig_reg = "VDIG",
};

struct mxc_dvfs_platform_data dvfs_core_data = {
	.reg_id = "SW1",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_reg_addr = MXC_GPC_CNTR,
	.gpc_vcr_reg_addr = MXC_GPC_VCR,
	.ccm_cdcr_reg_addr = MXC_CCM_CDCR,
	.ccm_cacrr_reg_addr = MXC_CCM_CACRR,
	.ccm_cdhipr_reg_addr = MXC_CCM_CDHIPR,
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

struct mxc_dvfsper_data dvfs_per_data = {
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

struct platform_pwm_backlight_data mxc_pwm_backlight_data = {
	.pwm_id = 0,
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 78770,
};

struct mxc_audio_platform_data wm8753_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 3,
	.sysclk = 12000000,
};

struct mxc_fb_platform_data mx51_fb_data[2] = {
	/* DISP0 */
	{
#ifdef CCWMX51_DISP0_RGB666
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
#else
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,
#endif
		.mode_str = "1024x768M-16@60",  /* Default */
	},
	/* DISP1 */
	{
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,
		.mode_str = "800x480-16@60",    /* Default */
	}
};
#if defined(CONFIG_UIO_PDRV_GENIRQ) || defined(CONFIG_UIO_PDRV_GENIRQ_MODULE)
struct uio_info gpu2d_platform_data = {
	.name = "imx_gpu2d",
	.version = "1",
	.irq = MXC_INT_GPU2_IRQ,
	.open = gpu2d_open,
	.release = gpu2d_release,
	.mmap = gpu2d_mmap,
};
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
struct ccwmx51_lcd_pdata plcd_platform_data[2];

#if defined(CONFIG_CCWMX51_DISP1)
static char *video2_options[FB_MAX] __read_mostly;
static int ofonly2 __read_mostly;

int fb2_get_options(char *name, char **option)
{
	char *opt, *options = NULL;
	int opt_len, retval = 0;
	int name_len = strlen(name), i;

	if (name_len && ofonly2 && strncmp(name, "offb", 4))
		retval = 1;

	if (name_len && !retval) {
		for (i = 0; i < FB_MAX; i++) {
			if (video2_options[i] == NULL)
				continue;
			opt_len = strlen(video2_options[i]);
			if (!opt_len)
				continue;
			opt = video2_options[i];
			if (!strncmp(name, opt, name_len) &&
			    opt[name_len] == ':')
				options = opt + name_len + 1;
		}
	}
	if (options && !strncmp(options, "off", 3))
		retval = 1;

	if (option)
		*option = options;

	return retval;
}

static int __init video2_setup(char *options)
{
	int i, global = 0;

	if (!options || !*options)
		global = 1;

	if (!global && !strncmp(options, "ofonly", 6)) {
		ofonly2 = 1;
		global = 1;
	}

	if (!global && !strstr(options, "fb:")) {
		fb_mode_option = options;
		global = 1;
	}

	if (!global) {
		for (i = 0; i < FB_MAX; i++) {
			if (video2_options[i] == NULL) {
				video2_options[i] = options;
				break;
			}
		}
	}

        return 1;
}
__setup("video2=", video2_setup);
#endif /* defined(CONFIG_CCWMX51_DISP1) */

#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)

int __initdata pixclk_limit = { KHZ2PICOS(133000) };
char *vmode = NULL;

#define res_matches_refresh(v, x, y, r) \
                        ((v).xres == (x) && (v).yres == (y) && (v).refresh == (r))

static int fb_dump_mode(const char *str, const struct fb_videomode *vm)
{
	if (vm == NULL)
		return -1;

	printk(KERN_INFO "%s geometry %u %u %u\n",
	       str, vm->xres, vm->yres, vm->pixclock);
	printk(KERN_INFO "%s timings %u %u %u %u %u %u %u\n", str, vm->pixclock, vm->left_margin,
	       vm->right_margin, vm->upper_margin, vm->lower_margin, vm->hsync_len, vm->vsync_len);
	printk(KERN_INFO "%s flag %u sync %u vmode %u %s\n", str, vm->flag, vm->sync, vm->vmode,
	       vm->flag & FB_MODE_IS_FIRST ? "preferred" : "");
	return 0;
}

static void fb_dump_var(const char *str, struct fb_var_screeninfo *var)
{
	if (var == NULL)
		return;

	printk(KERN_INFO "%s geometry %u %u %u %u\n",
	       str,  var->xres, var->yres, var->xres_virtual, var->yres_virtual);
	printk(KERN_INFO "%s offset %u %u %u %u %u\n",
	       str, var->xoffset, var->yoffset, var->height, var->width, var->bits_per_pixel);
	printk(KERN_INFO "%s timings %u %u %u %u %u %u %u\n",
	       str, var->pixclock, var->left_margin, var->right_margin,
	       var->upper_margin, var->lower_margin, var->hsync_len, var->vsync_len);
	printk(KERN_INFO "%s accel_flags %u sync %u vmode %u\n",
	       str, var->accel_flags, var->sync, var->vmode);
}

struct ccwmx51_lcd_pdata * ccwmx51_get_hdmi_cmdline_mode(char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hdmi_display_list); i++) {
		if (!strncmp(hdmi_display_list[i].fb_pdata.mode->name,
			     name, strlen(hdmi_display_list[i].fb_pdata.mode->name)))
			return &hdmi_display_list[i];
	}
	return NULL;
}

static int ccwmx51_get_hdmi_videomode(struct ad9389_dev *ad9389, struct fb_videomode **vm)
{
	struct ad9389_pdata *pdata = ad9389->client->dev.platform_data;
	char *options = NULL;
	struct ccwmx51_lcd_pdata *plcd;
	int ret = 1;

	if (pdata->dispif == 0) {
		ret = fb_get_options("displayfb", &options);
	}
#if defined(CONFIG_CCWMX51_DISP1)
	else if (pdata->dispif == 1) {
		ret = fb2_get_options("displayfb", &options);
	}
#endif
	if (ret || !options)
		return -ENODEV;
	if (!strncasecmp(options, "HDMI", 4)) {
		pr_info("HDMI interface in DISP%d\n", pdata->dispif);

		/* Get the desired configuration provided by the bootloader */
		if (options[4] != '@') {
			pr_info("Video resolution for HDMI interface not provided, using auto\n");
			return 0;
		} else {
			options = &options[5];
			if ((plcd = ccwmx51_get_hdmi_cmdline_mode(options)) != NULL) {
				pr_info("HDMI: %s config on DISP%d\n", options, pdata->dispif);
				printk("HDMI %p, %p\n", plcd, plcd->fb_pdata.mode);
				*vm = plcd->fb_pdata.mode;
				printk("vm %p\n", vm);
				memcpy(&mx51_fb_data[pdata->dispif], &plcd_platform_data[pdata->dispif].fb_pdata,
				       sizeof(struct mxc_fb_platform_data));
				return 1;
			}
		}
	}
	return -EINVAL;
}

static char *ccwmx51_get_hdmi_cmdline_mode2(struct ad9389_dev *ad9389)
{
        struct ad9389_pdata *pdata = ad9389->client->dev.platform_data;
        char *options = NULL;
        int ret = 1;

        if (pdata->dispif == 0) {
                ret = fb_get_options("displayfb", &options);
        }
#if defined(CONFIG_CCWMX51_DISP1)
        else if (pdata->dispif == 1) {
                ret = fb2_get_options("displayfb", &options);
        }
#endif
        if (ret || !options)
                return NULL;
        if (!strncasecmp(options, "HDMI", 4)) {
                pr_info("HDMI interface in DISP%d\n", pdata->dispif);

                /* Get the desired configuration provided by the bootloader */
                if (options[4] == '@')
			return &options[5];
	}

	return NULL;
}

static void mxc_videomode_to_var(struct ad9389_dev *ad9389, struct fb_var_screeninfo *var)
{
	struct fb_info *info = ad9389->fbi;
	struct fb_monspecs *specs = &info->monspecs;
	const struct fb_videomode *mode = NULL;
	char *vmode;
	int ret, modeidx;

	/* First, check if we have a predefined mode through the kernel command line */
        ret = ccwmx51_get_hdmi_videomode(ad9389, (struct fb_videomode **)&mode);
	if (ret == 1) {
		printk("Configuration found!!\n");
		fb_dump_mode("hdmi cmdline mode:", mode);
		fb_videomode_to_var(var, mode);
		fb_dump_var("hdmi var:", var);
	} else if ((vmode = ccwmx51_get_hdmi_cmdline_mode2(ad9389)) != NULL) {
		printk("HDMI mode %s\n", vmode);
		modeidx = fb_find_mode(var, info, vmode, specs->modedb, specs->modedb_len, NULL, 16);
                switch (modeidx) {
		case 0:		printk("NO mode found!\n\n");
				break;
		case 1:		printk("Mode found!\n\n");
				break;
		case 2:
		case 3:
		case 4:		printk("Mode ???\n\n");
				break;
		}
	} else {
		printk("HDMI AUTO!!\n\n");
		mode = fb_find_best_display(specs, &info->modelist);
		if (mode) {
			fb_videomode_to_var(var, mode);
			printk(KERN_INFO "best mode is %ux%u@%u pclk=%u\n",
			       mode->xres, mode->yres, mode->refresh, mode->pixclock);
			fb_dump_mode("best mode", mode);
		} else {
			printk("NO mode found!!\n\n");
			//mode = fb_find_nearest_mode
		}
	}
}

void mxcfb_videomode_to_modelist(struct ad9389_dev *ad9389, struct fb_videomode *modedb,
				 int num, struct list_head *head)
{
	static struct fb_var_screeninfo var;
	int i, del = 0;
	INIT_LIST_HEAD(head);

	memset(&var, 0, sizeof(var));

	for (i = 0; i < num; i++) {
		struct list_head *pos, *n;
		struct fb_modelist *modelist;

                del = 0;

                if ( modedb[i].flag & FB_MODE_IS_FIRST ) {
                        printk(KERN_INFO "found preferred video mode %ux%u%s%u pclk=%u\n",
                                modedb[i].xres, modedb[i].yres,
                                (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
                                modedb[i].refresh,
                                modedb[i].pixclock );

                        memcpy(&ad9389->preferred, &modedb[i], sizeof(struct fb_videomode));
                        // and we carry on because the next few lines may delete it
                }
                if ( modedb[i].pixclock < pixclk_limit ) {
                        printk(KERN_INFO "%ux%u%s%u pclk=%u removed (exceed %u limit)\n",
                                modedb[i].xres, modedb[i].yres,
                                (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
                                modedb[i].refresh,
                                modedb[i].pixclock,
                                pixclk_limit );
                        continue; // next i, practically deleted
                        //modedb[i].pixclock = pixclk_limit;
                }
                else if ( (modedb[i].vmode & FB_VMODE_INTERLACED) ) {
                        printk(KERN_INFO "%ux%u%s%u pclk=%u removed (interlaced)\n",
                                modedb[i].xres, modedb[i].yres,
                                (modedb[i].vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
                                modedb[i].refresh,
                                modedb[i].pixclock );
                        continue; // next i, practically deleted
                }

                /* if candidate is a detail timing, delete existing one in modelist !
                 * note: some TV has 1280x720@60 in standard timings but also has 1280x720 in detail timing block
                 *         in this case, use detail timing !
                 */
                list_for_each_safe(pos, n, head) {
                        modelist = list_entry(pos, struct fb_modelist, list);

                        if ( res_matches_refresh(modelist->mode,
                                modedb[i].xres, modedb[i].yres, modedb[i].refresh) ) {

                                if ( ( modedb[i].flag & FB_MODE_IS_DETAILED ) ) {

                                        printk(KERN_INFO "%ux%u%s%u pclk=%u removed (duplicate)\n",
                                                modelist->mode.xres, modelist->mode.yres,
                                                (modelist->mode.vmode & FB_VMODE_INTERLACED ) ? "i@" : "@",
                                                modelist->mode.refresh,
                                                modelist->mode.pixclock );
                                        list_del(pos);
                                        kfree(pos);
                                        del = 1;
                                }
                        }
                }

                if (del == 0)
                        fb_add_videomode(&modedb[i], head);
        }
}

static int ccwmx51_hdmi_hw_init(void)
{
	mxc_request_iomux(AD9389_GPIO_IRQ, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION);
	mxc_iomux_set_pad(AD9389_GPIO_IRQ, PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM |
			  PAD_CTL_100K_PU | PAD_CTL_HYS_ENABLE |
			  PAD_CTL_DRV_VOT_HIGH);

	gpio_request(IOMUX_TO_GPIO(AD9389_GPIO_IRQ), "ad9389_irq");
	gpio_direction_input(IOMUX_TO_GPIO(AD9389_GPIO_IRQ));

	set_irq_type(IOMUX_TO_GPIO(AD9389_GPIO_IRQ), IRQ_TYPE_EDGE_BOTH);

	return 0;
}

static void ccwmx51_hdmi_disp_connected(void)
{
	printk("%s: detecting display...\n", __func__);
}

static void ccwmx51_hdmi_disp_disconnected(void)
{
	printk("%s: display disconnected...\n", __func__);
}

void mxcfb_pre_set_var(struct fb_var_screeninfo *var)
{
	/* Nothing todo */
}

static struct ad9389_pdata hdmi_pdata = {
	.hw_init		= &ccwmx51_hdmi_hw_init,
	.disp_connected		= &ccwmx51_hdmi_disp_connected,
	.disp_disconnected	= &ccwmx51_hdmi_disp_disconnected,
	.videomode_to_modelist	= &mxcfb_videomode_to_modelist,
	.videomode_to_var	= &mxc_videomode_to_var,
	.pre_set_var		= &mxcfb_pre_set_var,
	.edid_addr		= (0x7e >> 1),
	.dispif			= 0,
};

struct i2c_board_info ccwmx51_hdmi[] __initdata = {
	{
		I2C_BOARD_INFO("ad9389", 0x39),
		.irq		= IOMUX_TO_IRQ(AD9389_GPIO_IRQ),
		.platform_data	= &hdmi_pdata,
	},
};
#endif

struct ccwmx51_lcd_pdata * ccwmx51_get_display(char *name)
{
        int i;

        for (i = 0; i < ARRAY_SIZE(lcd_display_list); i++)
                if (!strncmp(lcd_display_list[i].fb_pdata.mode->name,
                             name, strlen(lcd_display_list[i].fb_pdata.mode->name)))
                        return &lcd_display_list[i];

        return NULL;
}

struct ccwmx51_lcd_pdata * ccwmx51_get_ad9389_config(char *name)
{
        int i;

        for (i = 0; i < ARRAY_SIZE(hdmi_display_list); i++)
                if (!strncmp(hdmi_display_list[i].fb_pdata.mode->name,
                             name, strlen(hdmi_display_list[i].fb_pdata.mode->name)))
                        return &hdmi_display_list[i];

        return NULL;
}

int __init ccwmx51_init_fb(void)
{
        char *options = NULL, *options2 = NULL, *p;
	struct ccwmx51_lcd_pdata *plcd;
        int ret1 = 1, ret2 = 1;

	plcd_platform_data[0].vif = -1;
	plcd_platform_data[1].vif = -1;


#if defined(CONFIG_VIDEO_AD9389) || defined(CONFIG_VIDEO_AD9389_MODULE)
printk("registering ad9389\n\n");
       i2c_register_board_info(1, ccwmx51_hdmi, 1);
#endif


#if defined(CONFIG_CCWMX51_DISP0)
	ret1 = fb_get_options("displayfb", &options);
#endif
#if defined(CONFIG_CCWMX51_DISP1)
	ret2 = fb2_get_options("displayfb", &options2);
#endif
	if (ret1 != 0 && ret2 != 0) {
		pr_warning("no display information available in command line\n");
		return -ENODEV;
	}

	if (!options && !options2)
		return 0;
#if defined(CONFIG_CCWMX51_DISP0)
        if (options) {
                pr_info("options %s\n", options);

                if (!strncasecmp(options, "VGA", 3)) {
                        pr_info("VGA interface in DISP1\n");

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
                } else if (!strncasecmp(options, "HDMI", 4)) {
                        pr_info("HDMI interface in DISP1\n");

                        /* Get the desired configuration provided by the bootloader */
                        if (options[3] != '@') {
                                pr_info("Video resolution for VGA interface not provided, using default\n");
                                /* TODO set default video here */
                        } else {
                                options = &options[4];
                                if (((p = strsep (&options, "@")) != NULL) && *p) {
                                        if ((plcd = ccwmx51_get_ad9389_config(options)) != NULL) {
                                                pr_info("HDMI: %s config on DISP1\n", options);
                                                memcpy(&plcd_platform_data[0], plcd, sizeof(struct ccwmx51_lcd_pdata));
                                                memcpy(&mx51_fb_data[0], &plcd_platform_data[0].fb_pdata, sizeof(struct mxc_fb_platform_data));
                                                plcd_platform_data[0].vif = 0; /* Select video interface 0 */
                                        }


                                }
                        }
                } else {
                        if ((plcd = ccwmx51_get_display(options)) != NULL) {
                                pr_info("%s panel on DISP1\n", options);
				memcpy(&plcd_platform_data[0], plcd, sizeof(struct ccwmx51_lcd_pdata));
				memcpy(&mx51_fb_data[0], &plcd_platform_data[0].fb_pdata, sizeof(struct mxc_fb_platform_data));
				plcd_platform_data[0].vif = 0; /* Select video interface 0 */
			}
		}
	} else {
		pr_info("NO options NULL\n");
	}
#endif
#if defined(CONFIG_CCWMX51_DISP1)
        if (options2) {
                if (!strncasecmp(options2, "VGA", 3)) {
                        pr_info("VGA interface in DISP2\n");

                        /* Get the desired configuration provided by the bootloader */
                        if (options2[3] != '@') {
                                pr_info("Video resolution for VGA interface not provided, using default\n");
                                /* TODO set default video here */
                        } else {
                                options2 = &options2[4];
                                if (((p = strsep (&options2, "@")) != NULL) && *p) {
                                        if (!strcmp(p, "640x480x16")) {
                                                strcpy(mx51_fb_data[1].mode_str, "640x480M-16@60");
                                        } else if (!strcmp(p, "800x600x16")) {
                                                strcpy(mx51_fb_data[1].mode_str, "800x600M-16@60");
                                        } else if (!strcmp(p, "1024x768x16")) {
                                                strcpy(mx51_fb_data[1].mode_str, "1024x768M-16@60");
                                        } else if (!strcmp(p, "1280x1024x16")) {
                                                strcpy(mx51_fb_data[1].mode_str, "1280x1024M-16@60");
                                        } else
                                                pr_warning("Unsuported video resolution: %s, using default\n", p);
                                }
                        }
                } else {
                        if ((plcd = ccwmx51_get_display(options2)) != NULL) {
                                pr_info("%s panel on DISP2\n", options2);
				memcpy(&plcd_platform_data[1], plcd, sizeof(struct ccwmx51_lcd_pdata));
                                memcpy(&mx51_fb_data[1], &plcd_platform_data[1].fb_pdata, sizeof(struct mxc_fb_platform_data));
                                plcd_platform_data[1].vif = 1; /* Select video interface 1 */
                        }
                }
        }
#endif
        return 0;
}
#endif

#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
struct fsl_ata_platform_data ata_data = {
	.udma_mask  = ATA_UDMA3,
	.mwdma_mask = ATA_MWDMA2,
	.pio_mask   = ATA_PIO4,
	.fifo_alarm = MXC_IDE_DMA_WATERMARK / 2,
	.max_sg     = MXC_IDE_DMA_BD_NR,
	.init       = NULL,
	.exit       = NULL,
	.core_reg   = NULL,
	.io_reg     = NULL,
};
#endif

void __init ccwmx51_init_devices ( void )
{
#ifdef CONFIG_SYSFS
        ccwmx51_create_sysfs_entries();
#endif
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
        ccwmx51_init_ext_eth_mac();
#endif
}
