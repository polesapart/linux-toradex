/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2005-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
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
#include <asm/mach/keypad.h>
#include <asm/arch/memory.h>
#include <asm/arch/gpio.h>

#include "board-mx3_3stack.h"
#include "crm_regs.h"
#include "iomux.h"
/*!
 * @file mach-mx3/mx3_3stack.c
 *
 * @brief This file contains the board specific initialization routines.
 *
 * @ingroup MSL_MX31
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

unsigned long board_get_ckih_rate(void)
{
	return 26000000;
}

#if defined(CONFIG_KEYBOARD_MXC) || defined(CONFIG_KEYBOARD_MXC_MODULE)
static u16 keymapping[12] = {
	KEY_UP, KEY_DOWN, 0, 0,
	KEY_RIGHT, KEY_LEFT, KEY_ENTER, 0,
	KEY_F6, KEY_F8, KEY_F9, KEY_F10,
};

static struct resource mxc_kpp_resources[] = {
	[0] = {
	       .start = MXC_INT_KPP,
	       .end = MXC_INT_KPP,
	       .flags = IORESOURCE_IRQ,
	       }
};

static struct keypad_data keypad_plat_data = {
	.rowmax = 3,
	.colmax = 4,
	.irq = MXC_INT_KPP,
	.learning = 0,
	.delay = 2,
	.matrix = keymapping,
};

/* mxc keypad driver */
static struct platform_device mxc_keypad_device = {
	.name = "mxc_keypad",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_kpp_resources),
	.resource = mxc_kpp_resources,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &keypad_plat_data,
		},
};

static void mxc_init_keypad(void)
{
	(void)platform_device_register(&mxc_keypad_device);
}
#else
static inline void mxc_init_keypad(void)
{
}
#endif

/* MTD NAND flash */
#if defined(CONFIG_MTD_NAND_MXC) || defined(CONFIG_MTD_NAND_MXC_MODULE) \
	|| defined(CONFIG_MTD_NAND_MXC_V2) || defined(CONFIG_MTD_NAND_MXC_V2_MODULE)

static struct mtd_partition mxc_nand_partitions[] = {
	{
	 .name = "IPL-SPL",
	 .offset = 0,
	 .size = 256 * 1024},
	{
	 .name = "nand.kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 4 * 1024 * 1024},
	{
	 .name = "nand.rootfs",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 96 * 1024 * 1024},
	{
	 .name = "nand.configure",
	 .offset = MTDPART_OFS_APPEND,
	 .size = 8 * 1024 * 1024},
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
	if (__raw_readl(MXC_CCM_RCSR) & MXC_CCM_RCSR_NF16B) {
		mxc_nand_data.width = 2;
	}
	if (cpu_is_mx31()) {
		(void)platform_device_register(&mxc_nand_mtd_device);
	}
	if (cpu_is_mx32()) {
		(void)platform_device_register(&mxc_nandv2_mtd_device);
	}
}
#else
static inline void mxc_init_nand_mtd(void)
{
}
#endif

static void lcd_reset(void)
{
	/* ensure that LCDIO(1.8V) has been turn on */
	/* active reset line GPIO */
	mxc_request_iomux(MX31_PIN_LCS1, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);
	mxc_set_gpio_dataout(MX31_PIN_LCS1, 0);
	mxc_set_gpio_direction(MX31_PIN_LCS1, 0);
	/* do reset */
	msleep(10);		/* tRES >= 100us */
	mxc_set_gpio_dataout(MX31_PIN_LCS1, 1);
	msleep(60);
}

static struct mxc_lcd_platform_data lcd_data = {
	.io_reg = "VGEN",
	.core_reg = "VMMC1",
	.reset = lcd_reset,
};

static struct spi_board_info mxc_spi_board_info[] __initdata = {
	{
	 .modalias = "pmic_spi",
	 .irq = IOMUX_TO_IRQ(MX31_PIN_GPIO1_3),
	 .max_speed_hz = 4000000,
	 .bus_num = 2,
	 .chip_select = 2,},
	{
	 .modalias = "lcd_spi",
	 .platform_data = (void *)&lcd_data,
	 .max_speed_hz = 5000000,
	 .bus_num = 1,
	 .chip_select = 2,},
};

/*lan9217 device*/
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource smsc911x_resources[] = {
	{
	 .start = LAN9217_BASE_ADDR,
	 .end = LAN9217_BASE_ADDR + 0x100,
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = LAN9217_IRQ,
	 .end = LAN9217_IRQ,
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
static void mxc_init_enet(void)
{
	(void)platform_device_register(&smsc_lan9217_device);
}
#else
static inline void mxc_init_enet(void)
{
}
#endif

#if defined(CONFIG_FB_MXC_SYNC_PANEL) || defined(CONFIG_FB_MXC_SYNC_PANEL_MODULE)
static const char fb_default_mode[] = "Epson-VGA";

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

static void mxc_init_fb(void)
{
	(void)platform_device_register(&mxc_fb_device);
}
#else
static inline void mxc_init_fb(void)
{
}
#endif

#if defined(CONFIG_BACKLIGHT_MXC)
static struct platform_device mxcbl_devices[] = {
#if defined(CONFIG_BACKLIGHT_MXC_IPU) || defined(CONFIG_BACKLIGHT_MXC_IPU_MODULE)
	{
	 .name = "mxc_ipu_bl",
	 .id = 0,
	 .dev = {
		 .platform_data = (void *)3,	/* DISP # for this backlight */
		 },
	 },
#endif
};
static inline void mxc_init_bl(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(mxcbl_devices); i++) {
		platform_device_register(&mxcbl_devices[i]);
	}
}
#else
static inline void mxc_init_bl(void)
{
}
#endif

#if defined(CONFIG_MXC_HP_DETECT) || defined(CONFIG_MXC_HP_DETECT_MODULE)

static struct resource mxc_hp_detect_resources[] = {
	[0] = {
	       .start = IOMUX_TO_IRQ(MX31_PIN_BATT_LINE),
	       .end = IOMUX_TO_IRQ(MX31_PIN_BATT_LINE),
	       .flags = IORESOURCE_IRQ,
	       }
};

/* mxc headphone detect driver */
static struct platform_device mxc_hp_detect_device = {
	.name = "hp_detect",
	.id = 0,
	.num_resources = ARRAY_SIZE(mxc_hp_detect_resources),
	.resource = mxc_hp_detect_resources,
	.dev = {
		.release = mxc_nop_release,
		},
};

static int mxc_init_hp_detect(void)
{
	/* request gpio for phone jack detect */
	mxc_request_iomux(MX31_PIN_BATT_LINE, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	mxc_iomux_set_pad(MX31_PIN_BATT_LINE, PAD_CTL_PKE_NONE);
	mxc_set_gpio_direction(MX31_PIN_BATT_LINE, 1);

	return platform_device_register(&mxc_hp_detect_device);
}
#else
static inline void mxc_init_hp_detect(void)
{
}
#endif

static u32 brd_io;

static void mxc_expio_irq_handler(u32 irq, struct irq_desc *desc)
{
	u32 imr_val;
	u32 int_valid;
	u32 expio_irq;

	desc->chip->mask(irq);	/* irq = gpio irq number */

	imr_val = __raw_readw(brd_io + INTR_MASK_REG);
	int_valid = __raw_readw(brd_io + INTR_STATUS_REG) & ~imr_val;

	if (unlikely(!int_valid)) {
		goto out;
	}

	expio_irq = MXC_EXP_IO_BASE;
	for (; int_valid != 0; int_valid >>= 1, expio_irq++) {
		struct irq_desc *d;
		if ((int_valid & 1) == 0)
			continue;
		d = irq_desc + expio_irq;
		if (unlikely(!(d->handle_irq))) {
			printk(KERN_ERR "\nEXPIO irq: %d unhandled\n",
			       expio_irq);
			BUG();	/* oops */
		}
		d->handle_irq(expio_irq, d);
	}

      out:
	desc->chip->ack(irq);
	desc->chip->unmask(irq);
}

/*
 * Disable an expio pin's interrupt by setting the bit in the imr.
 * @param irq		an expio virtual irq number
 */
static void expio_mask_irq(u32 irq)
{
	u16 reg;
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* mask the interrupt */
	reg = __raw_readw(brd_io + INTR_MASK_REG);
	reg |= (1 << expio);
	__raw_writew(reg, brd_io + INTR_MASK_REG);
}

/*
 * Acknowledge an expanded io pin's interrupt by clearing the bit in the isr.
 * @param irq		an expanded io virtual irq number
 */
static void expio_ack_irq(u32 irq)
{
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* clear the interrupt status */
	__raw_writew(1 << expio, brd_io + INTR_RESET_REG);
	__raw_writew(0, brd_io + INTR_RESET_REG);
	/* mask the interrupt */
	expio_mask_irq(irq);
}

/*
 * Enable a expio pin's interrupt by clearing the bit in the imr.
 * @param irq		a expio virtual irq number
 */
static void expio_unmask_irq(u32 irq)
{
	u16 reg;
	u32 expio = MXC_IRQ_TO_EXPIO(irq);
	/* unmask the interrupt */
	reg = __raw_readw(brd_io + INTR_MASK_REG);
	reg &= ~(1 << expio);
	__raw_writew(reg, brd_io + INTR_MASK_REG);
}

static struct irq_chip expio_irq_chip = {
	.ack = expio_ack_irq,
	.mask = expio_mask_irq,
	.unmask = expio_unmask_irq,
};

static int __init mxc_expio_init(void)
{
	int i;

	brd_io = (u32) ioremap(BOARD_IO_ADDR, SZ_4K);
	if (brd_io == 0)
		return -ENOMEM;

	if ((__raw_readw(brd_io + MAGIC_NUMBER1_REG) != 0xAAAA) ||
	    (__raw_readw(brd_io + MAGIC_NUMBER2_REG) != 0x5555) ||
	    (__raw_readw(brd_io + MAGIC_NUMBER3_REG) != 0xCAFE)) {
		iounmap((void *)brd_io);
		brd_io = 0;
		return -ENODEV;
	}

	pr_info("3-Stack Debug board detected, rev = 0x%04X\n",
		readw(brd_io + CPLD_CODE_VER_REG));

	/*
	 * Configure INT line as GPIO input
	 */
	mxc_request_iomux(MX31_PIN_GPIO1_1, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_GPIO);
	mxc_set_gpio_direction(MX31_PIN_GPIO1_1, 1);

	/* disable the interrupt and clear the status */
	__raw_writew(0, brd_io + INTR_MASK_REG);
	__raw_writew(0xFFFF, brd_io + INTR_RESET_REG);
	__raw_writew(0, brd_io + INTR_RESET_REG);
	__raw_writew(0x1F, brd_io + INTR_MASK_REG);
	for (i = MXC_EXP_IO_BASE; i < (MXC_EXP_IO_BASE + MXC_MAX_EXP_IO_LINES);
	     i++) {
		set_irq_chip(i, &expio_irq_chip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID);
	}
	set_irq_type(EXPIO_PARENT_INT, IRQT_LOW);
	set_irq_chained_handler(EXPIO_PARENT_INT, mxc_expio_irq_handler);

	return 0;
}

#if defined(CONFIG_MXC_PMIC_MC13783) && defined(CONFIG_SND_MXC_PMIC)
static void __init mxc_init_pmic_audio(void)
{
	struct clk *pll_clk;
	struct clk *ssi_clk;
	struct clk *ckih_clk;
	struct clk *cko_clk;

	/* Enable 26 mhz clock on CKO1 for PMIC audio */
	ckih_clk = clk_get(NULL, "ckih");
	cko_clk = clk_get(NULL, "cko1_clk");
	if (IS_ERR(ckih_clk) || IS_ERR(cko_clk)) {
		printk(KERN_ERR "Unable to set CKO1 output to CKIH\n");
	} else {
		clk_set_parent(cko_clk, ckih_clk);
		clk_set_rate(cko_clk, clk_get_rate(ckih_clk));
		clk_enable(cko_clk);
	}
	clk_put(ckih_clk);
	clk_put(cko_clk);

	/* Assign USBPLL to be used by SSI1/2 */
	pll_clk = clk_get(NULL, "usb_pll");
	ssi_clk = clk_get(NULL, "ssi_clk.0");
	clk_set_parent(ssi_clk, pll_clk);
	clk_put(ssi_clk);

	ssi_clk = clk_get(NULL, "ssi_clk.1");
	clk_set_parent(ssi_clk, pll_clk);
	clk_put(ssi_clk);
	clk_put(pll_clk);

	/* config Audio ports (4 & 5) */
	mxc_request_iomux(MX31_PIN_SCK4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SRXD4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_STXD4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SFS4, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SCK5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SRXD5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_STXD5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
	mxc_request_iomux(MX31_PIN_SFS5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);
}
#else
static void __inline mxc_init_pmic_audio(void)
{
}
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

/* IDE device data */
#if defined(CONFIG_BLK_DEV_IDE_MXC) || defined(CONFIG_BLK_DEV_IDE_MXC_MODULE)

/*! Platform Data for MXC IDE */
static struct mxc_ide_platform_data mxc_ide_data = {
	.power_drive = "GPO2",
	.power_io = "GPO3",
};

static struct platform_device mxc_ide_device = {
	.name = "mxc_ide",
	.id = 0,
	.dev = {
		.release = mxc_nop_release,
		.platform_data = &mxc_ide_data,
		},
};

static inline void mxc_init_ide(void)
{
	if (platform_device_register(&mxc_ide_device) < 0)
		printk(KERN_ERR "Error: Registering the ide.\n");
}
#else
static inline void mxc_init_ide(void)
{
}
#endif

/*!
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
	/* config CS5 for debug board */
	mxc_request_iomux(MX31_PIN_CS5, OUTPUTCONFIG_FUNC, INPUTCONFIG_FUNC);

	mxc_cpu_common_init();
	mxc_clocks_init();
	mxc_gpio_init();
	early_console_setup(saved_command_line);

	/*Pull down MX31_PIN_USB_BYP to reset USB3317 */
	mxc_request_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_set_gpio_direction(MX31_PIN_USB_BYP, 0);
	mxc_set_gpio_dataout(MX31_PIN_USB_BYP, 0);
	mxc_free_iomux(MX31_PIN_USB_BYP, OUTPUTCONFIG_GPIO, INPUTCONFIG_NONE);

	/* Reset BT/WiFi chip */
	mxc_request_iomux(MX31_PIN_DCD_DCE1, OUTPUTCONFIG_GPIO,
			  INPUTCONFIG_NONE);
	mxc_set_gpio_direction(MX31_PIN_DCD_DCE1, 0);
	mxc_set_gpio_dataout(MX31_PIN_DCD_DCE1, 0);

	mxc_init_pmic_audio();
	mxc_expio_init();
	mxc_init_keypad();
	mxc_init_enet();
	mxc_init_nand_mtd();
	mxc_init_hp_detect();

	spi_register_board_info(mxc_spi_board_info,
				ARRAY_SIZE(mxc_spi_board_info));

	mxc_init_fb();
	mxc_init_bl();
	mxc_init_ide();
}

#define PLL_PCTL_REG(pd, mfd, mfi, mfn)		\
	((((pd) - 1) << 26) + (((mfd) - 1) << 16) + ((mfi)  << 10) + mfn)

/* For 26MHz input clock */
#define PLL_532MHZ		PLL_PCTL_REG(1, 13, 10, 3)
#define PLL_399MHZ		PLL_PCTL_REG(1, 52, 7, 35)
#define PLL_133MHZ		PLL_PCTL_REG(2, 26, 5, 3)

#define PDR0_REG(mcu, max, hsp, ipg, nfc)	\
	(MXC_CCM_PDR0_MCU_DIV_##mcu | MXC_CCM_PDR0_MAX_DIV_##max | \
	 MXC_CCM_PDR0_HSP_DIV_##hsp | MXC_CCM_PDR0_IPG_DIV_##ipg | \
	 MXC_CCM_PDR0_NFC_DIV_##nfc)

/* working point(wp): 0 - 133MHz; 1 - 266MHz; 2 - 399MHz; 3 - 532MHz */
/* 26MHz input clock table */
static struct cpu_wp cpu_wp_26[] = {
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 133000000,
	 .pdr0_reg = PDR0_REG(4, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 266000000,
	 .pdr0_reg = PDR0_REG(2, 4, 4, 2, 6),},
	{
	 .pll_reg = PLL_399MHZ,
	 .pll_rate = 399000000,
	 .cpu_rate = 399000000,
	 .pdr0_reg = PDR0_REG(1, 3, 3, 2, 6),},
	{
	 .pll_reg = PLL_532MHZ,
	 .pll_rate = 532000000,
	 .cpu_rate = 532000000,
	 .pdr0_reg = PDR0_REG(1, 4, 4, 2, 6),},
};

struct cpu_wp *get_cpu_wp(int *wp)
{
	*wp = 4;
	return cpu_wp_26;
}

/*
 * The following uses standard kernel macros define in arch.h in order to
 * initialize __mach_desc_MX3_3STACK data structure.
 */
/* *INDENT-OFF* */
MACHINE_START(MX31_3DS, "Freescale MX31/MX32 3-Stack Board")
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
