/*
 * Copyright 2007 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 * Copyright 2009 Daniel Schaeffer (daniel.schaeffer@timesys.com)
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

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mtd/physmap.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/imx-uart.h>
#include <mach/iomux.h>
#include <mach/board-mx27lite.h>
#include <mach/mxc_nand.h>
#include <mach/imxfb.h>

#include "devices.h"

static unsigned int mx27lite_pins[] = {
	/* UART1 */
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	PE14_PF_UART1_CTS,
	PE15_PF_UART1_RTS,
	/* FEC */
	PD0_AIN_FEC_TXD0,
	PD1_AIN_FEC_TXD1,
	PD2_AIN_FEC_TXD2,
	PD3_AIN_FEC_TXD3,
	PD4_AOUT_FEC_RX_ER,
	PD5_AOUT_FEC_RXD1,
	PD6_AOUT_FEC_RXD2,
	PD7_AOUT_FEC_RXD3,
	PD8_AF_FEC_MDIO,
	PD9_AIN_FEC_MDC,
	PD10_AOUT_FEC_CRS,
	PD11_AOUT_FEC_TX_CLK,
	PD12_AOUT_FEC_RXD0,
	PD13_AOUT_FEC_RX_DV,
	PD14_AOUT_FEC_RX_CLK,
	PD15_AOUT_FEC_COL,
	PD16_AIN_FEC_TX_ER,
	PF23_AIN_FEC_TX_EN,
	/* display */
	PA5_PF_LSCLK,
	PA6_PF_LD0,
	PA7_PF_LD1,
	PA8_PF_LD2,
	PA9_PF_LD3,
	PA10_PF_LD4,
	PA11_PF_LD5,
	PA12_PF_LD6,
	PA13_PF_LD7,
	PA14_PF_LD8,
	PA15_PF_LD9,
	PA16_PF_LD10,
	PA17_PF_LD11,
	PA18_PF_LD12,
	PA19_PF_LD13,
	PA20_PF_LD14,
	PA21_PF_LD15,
	PA22_PF_LD16,
	PA23_PF_LD17,
	PA24_PF_REV,
	PA25_PF_CLS,
	PA26_PF_PS,
	PA27_PF_SPL_SPR,
	PA28_PF_HSYNC,
	PA29_PF_VSYNC,
	PA30_PF_CONTRAST,
	PA31_PF_OE_ACD,
};

static struct mxc_nand_platform_data mx27lite_nand_board_info = {
	.width = 1,
	.hw_ecc = 1,
};

static struct mtd_partition mx27lite_nor_partitions[] = {
	{
	 .name = "nor.bootloader",
	 .size = 256 * 1024,
	 .offset = 0x00000000,
	 .mask_flags = MTD_WRITEABLE
	 },
	{
	 .name = "nor.config",
	 .size = 64 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = MTD_WRITEABLE
	 },
	{
	 .name = "nor.extra",
	 .size = MTDPART_SIZ_FULL,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0
	 },
};

static struct physmap_flash_data mx27lite_flash_data = {
	.width = 2,
	.parts = mx27lite_nor_partitions,
	.nr_parts = ARRAY_SIZE(mx27lite_nor_partitions),
};

static struct resource mx27lite_flash_resource = {
	.start = 0xc0000000,
	.end   = 0xc0000000 + 0x02000000 - 1,
	.flags = IORESOURCE_MEM,
};

static struct platform_device mx27lite_nor_mtd_device = {
	.name = "physmap-flash",
	.id = 0,
	.dev = {
		.platform_data = &mx27lite_flash_data,
	},
	.num_resources = 1,
	.resource = &mx27lite_flash_resource,
};

static struct imxuart_platform_data uart_pdata = {
	.flags = IMXUART_HAVE_RTSCTS,
};

static struct imx_fb_videomode logic_mbimx27_modes[] = {
	{
		.mode = {
			.name		= "SHARP-LQ64D343",
			.refresh	= 60,
			.xres		= 640,
			.yres		= 480,
			.pixclock	= 35285,
			.left_margin	= 134,
			.right_margin	= 34,
			.upper_margin	= 35,
			.lower_margin	= 0,
			.hsync_len	= 21,
			.vsync_len	= 5,
		},
		.pcr		= 0xFC008B80,
		.bpp		= 16,
	},
};

static struct imx_fb_platform_data logic_mbimx27_fb_data = {
	.mode = logic_mbimx27_modes,
	.num_modes = ARRAY_SIZE(logic_mbimx27_modes),

        .pwmr           = 0x00A903FF,
        .lscr1          = 0x00120300,
	.dmacr          = 0x00020010,
	//.dmacr = 0x00040060,
};

static struct platform_device *platform_devices[] __initdata = {
	&mx27lite_nor_mtd_device,
	&mxc_fec_device,
};

static void __init mx27lite_init(void)
{
	mxc_gpio_setup_multiple_pins(mx27lite_pins, ARRAY_SIZE(mx27lite_pins),
		"imx27lite");
	mxc_register_device(&mxc_uart_device0, &uart_pdata);
	mxc_register_device(&mxc_nand_device, &mx27lite_nand_board_info);
	mxc_register_device(&mxc_fb_device, 	&logic_mbimx27_fb_data);
	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));
}

static void __init mx27lite_timer_init(void)
{
	mx27_clocks_init(26000000);
}

static struct sys_timer mx27lite_timer = {
	.init	= mx27lite_timer_init,
};

MACHINE_START(IMX27LITE, "LogicPD i.MX27LITE")
	.phys_io        = AIPI_BASE_ADDR,
	.io_pg_offst    = ((AIPI_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = mx27_map_io,
	.init_irq       = mx27_init_irq,
	.init_machine   = mx27lite_init,
	.timer          = &mx27lite_timer,
MACHINE_END
