/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2006-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
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
#include <asm/mach/irq.h>
#include <asm/arch/memory.h>
#include <asm/arch/gpio.h>
#include "gpio_mux.h"
#include "board-mx27lite.h"

/*!
 * @file mach-mx27/mx27lite.c
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX27
 */

extern void mxc_map_io(void);
extern void mxc_init_irq(void);
extern void mxc_cpu_init(void) __init;
extern void mxc_clocks_init(void);
extern void mxc_cpu_common_init(void);
extern struct sys_timer mxc_timer;
extern void __init early_console_setup(char *);

static char command_line[COMMAND_LINE_SIZE];
int mxc_board_is_ads = 0;
EXPORT_SYMBOL(mxc_board_is_ads);

static void mxc_nop_release(struct device *dev)
{
	/* Nothing */
}

unsigned long board_get_ckih_rate(void)
{
	return 26000000;
}


/* --------------------------------------------------------------------
 * MTD NOR Flash
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MTD_MXC) || defined(CONFIG_MTD_MXC_MODULE)

static struct mtd_partition mxc_nor_partitions[] = {
	{
	 .name = "nor.bootloader",
	 .size = 256 * 1024,
	 .offset = 0x00000000,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
	{
	 .name = "nor.config",
	 .size = 64 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_WRITEABLE	/* force read-only */
	 },
	{
	 .name = "nor.extra",
	 .size = MTDPART_SIZ_FULL,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0
	 },
};

static struct flash_platform_data mxc_flash_data = {
	.map_name = "cfi_probe",
	.width = 2,
	.parts = mxc_nor_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nor_partitions),
};

static struct resource mxc_flash_resource = {
	.start = 0xc0000000,
	.end = 0xc0000000 + 0x02000000 - 1,
	.flags = IORESOURCE_MEM,

};

static struct platform_device mxc_nor_mtd_device = {
	.name = "mxc_nor_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_flash_data,
		},
	.num_resources = 1,
	.resource = &mxc_flash_resource,
};

static void __init mxc_init_nor_mtd(void)
{
	(void)platform_device_register(&mxc_nor_mtd_device);
}
#else
static void __init mxc_init_nor_mtd(void)
{
}
#endif


/* --------------------------------------------------------------------
 * MTD NAND Flash
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE)

static struct mtd_partition mxc_nand_partitions[4] = {
	{
	 .name = "IPL-SPL",
	 .offset = 0,
	 .size = 128 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 4 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 22 * 1024 * 1024},
	{
	 .name = "nand.userfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL},
};

static struct flash_platform_data mxc_nand_data = {
	.parts = mxc_nand_partitions,
	.nr_parts = ARRAY_SIZE(mxc_nand_partitions),
	.width = 1,
};

static struct platform_device mxc_nand_mtd_device = {
	.name = "mxc_nand_flash",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_nand_data,
		},
};

static void __init mxc_init_nand_mtd(void)
{
	(void)platform_device_register(&mxc_nand_mtd_device);
}
#else
static void __init mxc_init_nand_mtd(void)
{
}
#endif


/* --------------------------------------------------------------------
 * LCD
 * -------------------------------------------------------------------- */
#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)

static const char fb_default_mode[] = "Sharp-VGA";

/* mxc lcd driver */
static struct platform_device mxc_fb_device = {
	.name = "mxc_sdc_fb",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &fb_default_mode,
		.coherent_dma_mask = 0xFFFFFFFF,
		},
};

static void __init mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device);
}
#else
static void __init mxc_init_fb(void)
{
}
#endif


/* --------------------------------------------------------------------
 * SPI
 * -------------------------------------------------------------------- */
static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "pmic_spi",
	 .irq = MXC_PMIC_INT_LINE,
	 .max_speed_hz = 4000000,
	 .bus_num = 1,
	 .chip_select = 0,
	 },
};


/* --------------------------------------------------------------------
 * PMIC Audio
 * -------------------------------------------------------------------- */
#if defined(CONFIG_MXC_PMIC_MC13783) && defined(CONFIG_SND_MXC_PMIC)
extern void gpio_ssi_active(int ssi_num);

static void __init mxc_init_pmic_audio(void)
{
	struct clk *ssi_clk;
	struct clk *ckih_clk;
	struct clk *cko_clk;

	/* Enable 26 mhz clock on CKO1 for PMIC audio */
	ckih_clk = clk_get(NULL, "ckih");
	cko_clk = clk_get(NULL, "clko_clk");
	if (IS_ERR(ckih_clk) || IS_ERR(cko_clk)) {
		printk(KERN_ERR "Unable to set CLKO output to CKIH\n");
	} else {
		clk_set_parent(cko_clk, ckih_clk);
		clk_set_rate(cko_clk, clk_get_rate(ckih_clk));
		clk_enable(cko_clk);
	}
	clk_put(ckih_clk);
	clk_put(cko_clk);

	ssi_clk = clk_get(NULL, "ssi_clk.0");
	clk_enable(ssi_clk);
	clk_put(ssi_clk);

	gpio_ssi_active(0);
}
#else
static void __init mxc_init_pmic_audio(void)
{
}
#endif

static __init void mxc_board_init(void)
{
	pr_info("AIPI VA base: 0x%x\n", IO_ADDRESS(AIPI_BASE_ADDR));
	mxc_cpu_common_init();
	mxc_clocks_init();
	early_console_setup(saved_command_line);
	mxc_gpio_init();
	mxc_init_nor_mtd();
	mxc_init_nand_mtd();
	mxc_init_pmic_audio();

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));

	mxc_init_fb();
}

static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
	mxc_cpu_init();
	/* Store command line for use on mxc_board_init */
	strcpy(command_line, *cmdline);

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

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX27LITE data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX27LITE, "LogicPD i.MX27LITE")
	/* maintainer: TimeSys, Inc. */
	.phys_io        = AIPI_BASE_ADDR,
	.io_pg_offst    = ((AIPI_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.fixup          = fixup_mxc_board,
	.map_io         = mxc_map_io,
	.init_irq       = mxc_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &mxc_timer,
MACHINE_END
