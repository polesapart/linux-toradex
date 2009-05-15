/*
 * arch/arm/mach-ns9xxx/mach-cme9210js.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/spi/spi.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include "irq.h"
#include "processor-ns921x.h"
#include "ns921x_devices.h"
#include "cme9210_devices.h"

/* SPI devices */
static struct spi_board_info spi_devices[] __initdata = {
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
	{
		.modalias	= "spidev",
		.max_speed_hz	= 10000000,
		.bus_num	= 1,
		.chip_select	= 0,
	},
#endif
};

/* I2C devices */
/* Array to add I2C devices 
static struct i2c_board_info i2c_devices[] __initdata = {
	{
		I2C_BOARD_INFO("device_name", address),
	},
};
*/

static void __init mach_cme9210js_init_machine(void)
{
	/* register several system clocks */
	ns921x_init_machine();

	/* UART */
#if defined(CONFIG_CME9210JS_SERIAL_PORTA_RXTX)
	ns9xxx_add_device_cme9210_uarta_rxtx();
#elif defined(CONFIG_CME9210JS_SERIAL_PORTA_CTSRTSRXTX) || \
	defined(CONFIG_CME9210JS_SERIAL_PORTA_RXTX485)
	ns9xxx_add_device_cme9210_uarta_ctsrtsrxtx();
#elif defined(CONFIG_CME9210JS_SERIAL_PORTA_FULL)
	ns9xxx_add_device_cme9210_uarta_full();
#endif
#if defined(CONFIG_CME9210JS_SERIAL_PORTC_RXTX)
	ns9xxx_add_device_cme9210_uartc_rxtx();
#endif

	/* Ethernet */
	/* Adjust the netdev_max_backlog to a lower value. A big queue
	 * of incoming packets in the kernel can exhaust the kernel memory
	 * and trigger the OOM killer. This value can be tuned later in
	 * user space throught proc fs and with sysctl */
	netdev_max_backlog = 300;
	ns9xxx_add_device_cme9210_eth();

	/* Watchdog timer */
	ns9xxx_add_device_ns921x_wdt();

	/* NOR Flash */
	ns9xxx_add_device_cme9210_flash();

	/* SPI */
#ifdef CONFIG_CME9210JS_SPI
	ns9xxx_add_device_cme9210_spi();
#endif

	/* SPI devices */
	spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));

	/* I2C controller */
	ns9xxx_add_device_cme9210_i2c();

	/* I2C devices */
	/*
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));
	*/

	/* Init the FIM devices */
	ns9xxx_add_device_ns921x_fims();

	/* AES HW Encryption module */
#if defined(CONFIG_CRYPTO_DEV_NS921X_AES) || \
	defined(CONFIG_CRYPTO_DEV_NS921X_AES_MODULE)
	ns9xxx_add_device_ns921x_aes();
#endif
}

MACHINE_START(CME9210JS, "Digi Connect ME 9210 on Devboard")
	.map_io = ns921x_map_io,
	.init_irq = ns9xxx_init_irq,
	.init_machine = mach_cme9210js_init_machine,
	.timer = &ns921x_timer,
	.boot_params = 0x100,
MACHINE_END
