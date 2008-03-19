/*
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#if defined(CONFIG_MTD) || defined(CONFIG_MTD_MODULE)
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/mach/flash.h>
#endif

#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/keypad.h>
#include <asm/arch/memory.h>
#include <asm/arch/gpio.h>
#include "board-mx37_3stack.h"
#include "iomux.h"
#include "crm_regs.h"

/*!
 * @file mach-mx37/mx37_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX37
 */
extern void mxc_map_io(void);
extern void mxc_init_irq(void);
extern void mxc_cpu_init(void) __init;
extern struct sys_timer mxc_timer;
extern void mxc_cpu_common_init(void);
extern int mxc_clocks_init(void);
extern void __init early_console_setup(char *);

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

/* Get reference input clocks */
void board_ref_clk_rate(unsigned long *ckil, unsigned long *osc,
			unsigned long *ckih)
{
	*ckil = 32768;
	*osc = 24000000;
	*ckih = 22579200;
}

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V3)

static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "nand.bootloader",
	 .offset = 0,
	 .size = 2 * 1024 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 4 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 128 * 1024 * 1024},
	{
	 .name = "nand.userfs1",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 256 * 1024 * 1024},
	{
	 .name = "nand.userfs2",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 512 * 1024 * 1024},
	{
	 .name = "nand.userfs3",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 1024 * 1024 * 1024},
};

static struct flash_platform_data mxc_nand_data = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
};

static struct platform_device mxc_nandv2_mtd_device = {
	.name = "mxc_nandv2_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void mxc_init_nand_mtd(void)
{
//      if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B) {
//              mxc_nand_data.width = 2;
//      }
	(void)platform_device_register(&mxc_nandv2_mtd_device);
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static void lcd_reset(void)
{
	static int first;

	/* ensure that LCDIO(1.8V) has been turn on */
	/* active reset line GPIO */
	if (!first) {
		mxc_request_iomux(MX37_PIN_GPIO1_5, IOMUX_CONFIG_GPIO);
		first = 1;
	}
	mxc_set_gpio_dataout(MX37_PIN_GPIO1_5, 0);
	mxc_set_gpio_direction(MX37_PIN_GPIO1_5, 0);
	/* do reset */
	msleep(10);		/* tRES >= 100us */
	mxc_set_gpio_dataout(MX37_PIN_GPIO1_5, 1);
	msleep(60);
}

static struct mxc_lcd_platform_data lcd_data = {
	.core_reg = "LDO1",
	.io_reg = "DCDC6",
	.reset = lcd_reset,
};

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
	 .driver_name = "TSC2007",
	 .addr = 0x48,
	 .irq = IOMUX_TO_IRQ(MX37_PIN_AUD5_RXFS),
	 },
};

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "cpld_spi",
	 .max_speed_hz = 27000000,
	 .bus_num = 2,
	 .chip_select = 0,
	 },
	{
	 .modalias = "lcd_spi",
	 .max_speed_hz = 5000000,
	 .bus_num = 2,
	 .platform_data = &lcd_data,
	 .chip_select = 1,},
};

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static struct platform_device mxc_fb_device[] = {
	{
	 .name = "mxc_sdc_fb",
	 .id = 0,
	 .dev = {
		 .release = mxc_nop_release,
		 .coherent_dma_mask = 0xFFFFFFFF,
		 },
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

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device[0]);
	(void)platform_device_register(&mxc_fb_device[1]);
	(void)platform_device_register(&mxc_fb_device[2]);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)

static int __init mxc_init_touchscreen(void)
{
	int pad_val;

	mxc_request_iomux(MX37_PIN_AUD5_RXFS, IOMUX_CONFIG_GPIO);
	pad_val = PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU;
	mxc_iomux_set_pad(MX37_PIN_AUD5_RXFS, pad_val);
	mxc_set_gpio_direction(MX37_PIN_AUD5_RXFS, 1);

	return 0;
}
#else
static int __init mxc_init_touchscreen(void)
{
	return 0;
}
#endif

/*lan9217 device*/
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource smsc911x_resources[] = {
	{
	 .start = LAN9217_BASE_ADDR,
	 .end = LAN9217_BASE_ADDR + 255,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MXC_EXP_IO_BASE,
	 .flags = IORESOURCE_IRQ,
	 },
};
static struct platform_device smsc_lan9217_device = {
	.name = "smsc911x",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		},
	.num_resources = ARRAY_SIZE(smsc911x_resources),
	.resource = smsc911x_resources,
};

static int __init mxc_init_enet(void)
{
	(void)platform_device_register(&smsc_lan9217_device);
	return 0;
}
#else
static int __init mxc_init_enet(void)
{
	return 0;
}
#endif

late_initcall(mxc_init_enet);

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
	mxc_cpu_init();

#ifdef CONFIG_DISCONTIGMEM
	do {
		int nid;
		mi->nr_banks = MXC_NUMNODES;
		for (nid = 0; nid < mi->nr_banks; nid++) {
			SET_NODE(mi, nid);
		}
	} while (0);
#endif
}

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	mxc_cpu_common_init();
	mxc_clocks_init();
	mxc_gpio_init();
	early_console_setup(saved_command_line);
	i2c_register_board_info(0, mxc_i2c0_board_info,
				ARRAY_SIZE(mxc_i2c0_board_info));

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));
	mxc_init_nand_mtd();

	mxc_init_fb();
	mxc_init_touchscreen();
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX37_3STACK data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX37_3DS, "Freescale MX37 3-Stack Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io = AIPS1_BASE_ADDR,
	.io_pg_offst = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mxc_map_io,
	.init_irq = mxc_init_irq,
	.init_machine = mxc_board_init,
	.timer = &mxc_timer,
MACHINE_END
