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
#include "devices_ccwmx51.h"
#include "usb.h"

extern struct cpu_wp *(*get_cpu_wp)(int *wp);
extern void (*set_num_cpu_wp)(int num);
static int num_cpu_wp = 3;

/* working point(wp): 0 - 800MHz; 1 - 166.25MHz; */
static struct cpu_wp cpu_wp_auto_800[] = {
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
	 .cpu_voltage = 900000,},
};

static struct cpu_wp cpu_wp_auto_600[] = {
	{
	 .pll_rate = 600000000,
	 .cpu_rate = 600000000,
	 .pdf = 0,
	 .mfi = 6,
	 .mfd = 3,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1000000,},
	{
	 .pll_rate = 600000000,
	 .cpu_rate = 150000000,
	 .pdf = 3,
	 .mfi = 6,
	 .mfd = 3,
	 .mfn = 1,
	 .cpu_podf = 3,
	 .cpu_voltage = 950000,},
};

static u32 ccwmx51_get_cpu_freq(void)
{
	u32 cpu_freq = 800000000;

	switch (system_serial_low & 0xff) {
	case 4:
	case 5:	cpu_freq = 600000000;
		num_cpu_wp = 2;
		break;
	}

	return cpu_freq;
}

struct cpu_wp *mx51_get_cpu_wp(int *wp)
{
	u32 cpu_clk_rate = ccwmx51_get_cpu_freq();

	*wp = num_cpu_wp;

	if (cpu_clk_rate == 800000000) {
		return cpu_wp_auto_800;
	} else if (cpu_clk_rate == 600000000) {
		return cpu_wp_auto_600;
	}
	return NULL;
}

void mx51_set_num_cpu_wp(int num)
{
	num_cpu_wp = num;
	return;
}

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
				mxcfb_resources[0].start + fb_mem / 2 - 1;
			mxcfb_resources[1].start =
				mxcfb_resources[0].end + 1;
			mxcfb_resources[1].end =
				mxcfb_resources[1].start + fb_mem / 2 - 1;
		} else {
			mxcfb_resources[0].start = 0;
			mxcfb_resources[0].end = 0;
			mxcfb_resources[1].start = 0;
			mxcfb_resources[1].end = 0;
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

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* Setup hwid information, passed through Serial ATAG */
	ccwmx51_set_mod_variant(system_serial_low & 0xff);
	ccwmx51_set_mod_revision((system_serial_low >> 8) & 0xff);
	ccwmx51_set_mod_sn(((system_serial_low << 8) & 0xff000000) |
			   ((system_serial_low >> 8) & 0x00ff0000) |
			   ((system_serial_high << 8) & 0x0000ff00) |
			   ((system_serial_high >> 8) & 0xff));

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
#if defined (CONFIG_MXC_SECURITY_SCC2)
	mxc_register_device(&mxcscc_device, NULL);
#endif
	mxc_register_device(&mxc_pwm1_device, NULL);
	mxc_register_device(&mxc_pwm_backlight_device, &mxc_pwm_backlight_data);

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
	/* SD card detect irqs */
	mxcsdhc1_device.resource[2].start = CCWMX51_SD1_CD_IRQ;
	mxcsdhc1_device.resource[2].end = CCWMX51_SD1_CD_IRQ;
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
#if defined(CONFIG_PATA_FSL) || defined(CONFIG_PATA_FSL_MODULE)
	mxc_register_device(&pata_fsl_device, &ata_data);
#endif
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
	mxc_register_device(&smsc911x_device, &ccwmx51_smsc9118);
#endif
#if defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753) || defined(CONFIG_SND_SOC_IMX_CCWMX51_WM8753_MODULE)
	mxc_register_device(&mxc_wm8753_device, &wm8753_data);
#endif
	ccwmx51_init_spidevices();
	ccwmx51_init_i2c2();
#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	mx5_usbh1_init();
#endif
	mx5_usb_dr_init();
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE) && \
   (defined(CONFIG_CCWMX51_DISP0) || defined(CONFIG_CCWMX51_DISP1))
	ccwmx51_init_fb();
#if defined(CONFIG_CCWMX51_DISP0)
	if (plcd_platform_data[0].vif != -1)
		mxc_register_device(&lcd_pdev[0], (void *)&plcd_platform_data[0]);

	mxc_fb_devices[0].num_resources = 1;
	mxc_fb_devices[0].resource = &mxcfb_resources[0];
	mxc_register_device(&mxc_fb_devices[0], &mx51_fb_data[0]);
#endif
#if defined(CONFIG_CCWMX51_DISP1)
	if (plcd_platform_data[1].vif != -1)
		mxc_register_device(&lcd_pdev[1], (void *)&plcd_platform_data[1]);

	mxc_fb_devices[1].num_resources = 1;
	mxc_fb_devices[1].resource = &mxcfb_resources[1];
	mxc_register_device(&mxc_fb_devices[1], &mx51_fb_data[1]);
#endif /* CONFIG_CCWMX51_DISP1 */

	/* DI0/1 DP-FG channel, used by the VPU */
	mxc_register_device(&mxc_fb_devices[2], NULL);
#endif /* defined(CONFIG_FB_MXC_SYNC_PANEL) || ... */

#ifdef CONFIG_MXC_PMIC_MC13892
	ccwmx51_init_mc13892();
	/* Configure PMIC irq line */
	set_irq_type(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), IRQ_TYPE_EDGE_BOTH);
#endif

#ifdef CONFIG_CCWMX51_SECOND_TOUCH
	ccwmx51_init_2nd_touch();
#endif

	pm_power_off = mxc_power_off;
}

static void __init ccwmx51_timer_init(void)
{
	struct clk *uart_clk;

	/* Change the CPU voltages for TO2*/
	if (cpu_is_mx51_rev(CHIP_REV_2_0) <= 1) {
		cpu_wp_auto_800[0].cpu_voltage = 1175000;
		cpu_wp_auto_800[1].cpu_voltage = 1100000;
		cpu_wp_auto_800[2].cpu_voltage = 1000000;
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
