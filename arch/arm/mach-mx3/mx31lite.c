/*
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/memory.h>
#include <linux/gpio.h>
#include <linux/smc911x.h>
#include <linux/interrupt.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>
#include <mach/common.h>
#include <asm/page.h>
#include <asm/setup.h>
#include <mach/board-mx31lite.h>
#include <mach/imx-uart.h>
#include <mach/iomux-mx3.h>

#include "devices.h"

/*
 * This file contains the board-specific initialization routines.
 */

#if defined(CONFIG_SERIAL_IMX) || defined(CONFIG_SERIAL_IMX_MODULE)
static struct imxuart_platform_data uart_pdata = {
       .flags = IMXUART_HAVE_RTSCTS,
};

static inline void mxc_init_imx_uart(void)
{
       mxc_iomux_mode(MX31_PIN_CTS1__CTS1);
       mxc_iomux_mode(MX31_PIN_RTS1__RTS1);
       mxc_iomux_mode(MX31_PIN_TXD1__TXD1);
       mxc_iomux_mode(MX31_PIN_RXD1__RXD1);

       mxc_register_device(&mxc_uart_device0, &uart_pdata);
}
#else /* !SERIAL_IMX */
static inline void mxc_init_imx_uart(void)
{
}
#endif /* !SERIAL_IMX */


#if defined(CONFIG_SPI_IMX) || defined(CONFIG_SPI_IMX_MODULE)
static inline void mxc_init_imx_spi(void)
{
       /* CSPI1 - AT93C66A EEPROM attached to CS2 */
       mxc_iomux_mode(MX31_PIN_CSPI1_MOSI__MOSI);
       mxc_iomux_mode(MX31_PIN_CSPI1_MISO__MISO);
       mxc_iomux_mode(MX31_PIN_CSPI1_SCLK__SCLK);
       mxc_iomux_mode(MX31_PIN_CSPI1_SPI_RDY__SPI_RDY);
       mxc_iomux_mode(MX31_PIN_CSPI1_SS0__SS0);
       mxc_iomux_mode(MX31_PIN_CSPI1_SS1__SS1);
       mxc_iomux_mode(MX31_PIN_CSPI1_SS2__SS2);
       /* CSPI2 - PMIC attached to CS0 */
       mxc_iomux_mode(MX31_PIN_CSPI2_MOSI__MOSI);
       mxc_iomux_mode(MX31_PIN_CSPI2_MISO__MISO);
       mxc_iomux_mode(MX31_PIN_CSPI2_SCLK__SCLK);
       mxc_iomux_mode(MX31_PIN_CSPI2_SPI_RDY__SPI_RDY);
       mxc_iomux_mode(MX31_PIN_CSPI2_SS0__SS0);
       mxc_iomux_mode(MX31_PIN_CSPI2_SS1__SS1);
       mxc_iomux_mode(MX31_PIN_CSPI2_SS2__SS2);

       /* register the spi masters */
       platform_device_register(&mxc_spi_device0);
       platform_device_register(&mxc_spi_device1);

}
#else /* !SPI_IMX */
static inline void mxc_init_imx_spi(void)
{
}
#endif /* !SPI_IMX */

static struct resource smc911x_resources[] = {
       [0] = {
	      .start          = CS4_BASE_ADDR,
	      .end            = (CS4_BASE_ADDR + 0xfffff),
	      .flags          = IORESOURCE_MEM,
       },
       [1] = {
	      .start          = IOMUX_TO_IRQ(MX31_PIN_SFS6),
	      .end            = IOMUX_TO_IRQ(MX31_PIN_SFS6),
	      .flags          = IORESOURCE_IRQ,
       },
};

static struct smc911x_platdata smc911x_info = {
       .flags          = SMC911X_USE_32BIT,
       .irq_flags      = IRQF_SHARED | IRQF_TRIGGER_LOW,
};

static struct platform_device mx31lite_eth = {
       .name           = "smc911x",
       .id             = -1,
       .num_resources  = ARRAY_SIZE(smc911x_resources),
       .resource       = smc911x_resources,
       .dev            = {
       .platform_data = &smc911x_info,
       },
};


/*
 * This structure defines the MX31 memory map.
 */
static struct map_desc mx31lite_io_desc[] __initdata = {
	{
		.virtual = AIPS1_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(AIPS1_BASE_ADDR),
		.length = AIPS1_SIZE,
		.type = MT_DEVICE_NONSHARED
	}, {
		.virtual = SPBA0_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(SPBA0_BASE_ADDR),
		.length = SPBA0_SIZE,
		.type = MT_DEVICE_NONSHARED
	}, {
		.virtual = AIPS2_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(AIPS2_BASE_ADDR),
		.length = AIPS2_SIZE,
		.type = MT_DEVICE_NONSHARED
	}, {
		.virtual = CS4_BASE_ADDR_VIRT,
		.pfn = __phys_to_pfn(CS4_BASE_ADDR),
		.length = CS4_SIZE,
		.type = MT_DEVICE
	}
};

/*
 * Set up static virtual mappings.
 */
void __init mx31lite_map_io(void)
{
	mxc_map_io();
	iotable_init(mx31lite_io_desc, ARRAY_SIZE(mx31lite_io_desc));
}

static struct platform_device *devices[] __initdata = {
       &mx31lite_eth,
};

/*
 * Board specific initialization.
 */
static void __init mxc_board_init(void)
{
       /* init eth */
       mxc_iomux_mode(IOMUX_MODE(MX31_PIN_SFS6, IOMUX_CONFIG_GPIO));
       if (!gpio_request(MX31_PIN_GPIO3_1, "mx31lite-eth"))
	 gpio_direction_input(MX31_PIN_SFS6);

       platform_add_devices(devices, ARRAY_SIZE(devices));

       mxc_init_imx_uart();
       mxc_init_imx_spi();
}

static void __init mx31lite_timer_init(void)
{
	mxc_clocks_init(26000000);
	mxc_timer_init("ipg_clk.0");
}

struct sys_timer mx31lite_timer = {
	.init	= mx31lite_timer_init,
};

/*
 * The following uses standard kernel macros defined in arch.h in order to
 * initialize __mach_desc_MX31LITE data structure.
 */

MACHINE_START(MX31LITE, "LogicPD MX31 LITEKIT")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.phys_io        = AIPS1_BASE_ADDR,
	.io_pg_offst    = ((AIPS1_BASE_ADDR_VIRT) >> 18) & 0xfffc,
	.boot_params    = PHYS_OFFSET + 0x100,
	.map_io         = mx31lite_map_io,
	.init_irq       = mxc_init_irq,
	.init_machine   = mxc_board_init,
	.timer          = &mx31lite_timer,
MACHINE_END
