 /*
 * arch/arm/mach-ns9xxx/cme9210_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/mtd/physmap.h>
#include <mach/fim-ns921x.h>
#include <linux/gpio.h>

#include "ns921x_devices.h"
#include "cme9210_devices.h"

#if defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)
void __init ns9xxx_add_device_cme9210_eth(void)
{
	int gpio[] = {32, 33, 34, 35, 36, 37, 38, 39, 40,
		      41, 42, 43, 44, 45, 46, 47, 48, 49};

	ns9xxx_add_device_ns921x_eth(NULL, 0, gpio, 0);
}
#else
void __init ns9xxx_add_device_cme9210_eth(void) {}
#endif

#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static void cme9210_i2c_gpio_reconfigure(void)
{
	gpio_configure_ns921x(9, 0, 0, 1, 0);
	gpio_configure_ns921x(12, 0, 0, 1, 0);
}

static struct plat_ns9xxx_i2c ns9xxx_device_cme9210_i2c_data = {
	.gpio_scl = 9,
	.gpio_sda = 12,
	.speed = 100000,
	.gpio_configuration_func = cme9210_i2c_gpio_reconfigure,
};

void __init ns9xxx_add_device_cme9210_i2c(void)
{
	ns9xxx_add_device_ns921x_i2c(&ns9xxx_device_cme9210_i2c_data);
}
#else
void __init ns9xxx_add_device_cme9210_i2c(void) {}
#endif

#if defined(CONFIG_SERIAL_NS921X) || defined(CONFIG_SERIAL_NS921X_MODULE)
void __init ns9xxx_add_device_cme9210_uarta(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uarta(0, gpio_nr, 0);
}
void __init ns9xxx_add_device_cme9210_uartc(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartc(8, gpio_nr, 0);
}
#else
void __init ns9xxx_add_device_cme9210_uarta(int gpio_nr) {}
void __init ns9xxx_add_device_cme9210_uartc(int gpio_nr) {}
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP)
static struct physmap_flash_data ns9xxx_device_cme9210_flash_data = {
	.width	= 2,
};

void __init ns9xxx_add_device_cme9210_flash(void)
{
	ns9xxx_add_device_ns921x_flash(&ns9xxx_device_cme9210_flash_data);
}
#else
void __init ns9xxx_add_device_cme9210_flash(void) {}
#endif

#if defined(CONFIG_SPI_NS921X) || defined(CONFIG_SPI_NS921X_MODULE)
/* SPI ports and their related GPIOs */
static struct spi_ns9xxx_data ns9xxx_device_cme9210_spi_data = {
	.gpios = {7, 3, 5, 0},
	.gpio_funcs = { NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4 },
	.nr_gpios = 4,
};

void __init ns9xxx_add_device_cme9210_spi(void)
{
	ns9xxx_add_device_ns921x_spi(&ns9xxx_device_cme9210_spi_data);
}
#else
void __init ns9xxx_add_device_cme9210_spi(void) {}
#endif


#if defined(CONFIG_FIM_SERIAL)
static struct fim_serial_platform_data fim_serial_data0 = {
	.fim_nr        = 0,
	NS921X_FIM_SERIAL_GPIOS(0, 1, /* RX + TX */
				FIM_GPIO_DONT_USE, FIM_GPIO_DONT_USE, /* RTS + CTS */
				NS921X_GPIO_FUNC_2),
};
struct platform_device ns921x_fim_serial0 = {
	.name              = "fim-serial",
	.id                = 0,
	.dev.platform_data = &fim_serial_data0,
};
EXPORT_SYMBOL(ns921x_fim_serial0);

static struct fim_serial_platform_data fim_serial_data1 = {
	.fim_nr        = 1,
	NS921X_FIM_SERIAL_GPIOS(27, 26, /* RX + TX */
				FIM_GPIO_DONT_USE, FIM_GPIO_DONT_USE, /* RTS + CTS */
				NS921X_GPIO_FUNC_2),
};
struct platform_device ns921x_fim_serial1 = {
	.name              = "fim-serial",
	.id                = 1,
	.dev.platform_data = &fim_serial_data1,
};
EXPORT_SYMBOL(ns921x_fim_serial1);
#endif /* CONFIG_FIM_SERIAL */
