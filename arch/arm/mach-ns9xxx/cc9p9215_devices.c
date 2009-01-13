/*
 * arch/arm/mach-ns9xxx/cc9p9215_devices.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <mach/fim-ns921x.h>

#include "clock.h"
#include "ns921x_devices.h"
#include "cc9p9215_devices.h"

#if defined(CONFIG_NS9XXX_ETH) || defined(CONFIG_NS9XXX_ETH_MODULE)
static int cc9p9215_phy_endisable(struct clk *clk, int enable)
{
	int ret;

	if (enable) {
		ret = gpio_request(90, "ns9xxx-eth-phy");
		if (ret)
			return ret;

		gpio_direction_output(90, 1);
	} else {
		gpio_set_value(90, 0);
		gpio_free(90);
	}

	return 0;
}

static struct clk phyclk = {
	.name		= "ns9xxx-eth-phy",
	.id		= -1,
	.owner		= THIS_MODULE,
	.endisable	= cc9p9215_phy_endisable,
};

void __init ns9xxx_add_device_cc9p9215_eth(void)
{
	int gpio[] = {32, 33, 34, 35, 36, 37, 38, 39, 40,
		      41, 42, 43, 44, 45, 46, 47, 48, 49};

	if (clk_register(&phyclk))
		return;

	ns9xxx_add_device_ns921x_eth(&phyclk, 0, gpio, 0);
}
#else
void __init ns9xxx_add_device_cc9p9215_eth(void) {}
#endif

#if defined(CONFIG_I2C_NS9XXX) || defined(CONFIG_I2C_NS9XXX_MODULE)
static void cc9p9215_i2c_gpio_reconfigure(void)
{
	gpio_configure_ns921x(102, 0, 0, 2, 0);
	gpio_configure_ns921x(103, 0, 0, 2, 0);
}

static struct plat_ns9xxx_i2c ns9xxx_device_cc9p9215_i2c_data = {
	.gpio_scl = 102,
	.gpio_sda = 103,
	.speed = 100000,
	.gpio_configuration_func = cc9p9215_i2c_gpio_reconfigure,
};

void __init ns9xxx_add_device_cc9p9215_i2c(void)
{
	ns9xxx_add_device_ns921x_i2c(&ns9xxx_device_cc9p9215_i2c_data);
}
#else
void __init ns9xxx_add_device_cc9p9215_i2c(void) {}
#endif

#if defined(CONFIG_SERIAL_NS921X) || defined(CONFIG_SERIAL_NS921X_MODULE)
void __init ns9xxx_add_device_cc9p9215_uarta(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uarta(0, gpio_nr, 0);
}

void __init ns9xxx_add_device_cc9p9215_uartb(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartb(51, gpio_nr, 0);
}

void __init ns9xxx_add_device_cc9p9215_uartc(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartc(8, gpio_nr, 0);
}

void __init ns9xxx_add_device_cc9p9215_uartd(int gpio_nr)
{
	ns9xxx_add_device_ns921x_uartd(59, gpio_nr, 0);
}
#else
void __init ns9xxx_add_device_cc9p9215_uarta(int gpio_nr) {}
void __init ns9xxx_add_device_cc9p9215_uartb(int gpio_nr) {}
void __init ns9xxx_add_device_cc9p9215_uartc(int gpio_nr) {}
void __init ns9xxx_add_device_cc9p9215_uartd(int gpio_nr) {}
#endif

#if defined(CONFIG_MTD_PHYSMAP) || defined(CONFIG_MTD_PHYSMAP)
static struct physmap_flash_data ns9xxx_device_cc9p9215_flash_data = {
	.width	= 2,
};

void __init ns9xxx_add_device_cc9p9215_flash(void)
{
	ns9xxx_add_device_ns921x_flash(&ns9xxx_device_cc9p9215_flash_data);
}
#else
void __init ns9xxx_add_device_cc9p9215_flash(void) {}
#endif

#if defined(CONFIG_SPI_NS921X) || defined(CONFIG_SPI_NS921X_MODULE)
/* SPI ports and their related GPIOs */
static struct spi_ns9xxx_data ns9xxx_device_cc9p9215_spi_data = {
	.gpios = {7, 3, 5, 0},
	.gpio_funcs = { NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4,
			NS921X_GPIO_FUNC_4 },
	.nr_gpios = 4,
};

void __init ns9xxx_add_device_cc9p9215_spi(void) {
	ns9xxx_add_device_ns921x_spi(&ns9xxx_device_cc9p9215_spi_data);
}
#else
void __init ns9xxx_add_device_cc9p9215_spi(void) {}
#endif


#if defined(CONFIG_FIM_SERIAL)
static struct fim_serial_platform_data fim_serial_data0 = {
	.fim_nr        = 0,
	NS921X_FIM_SERIAL_GPIOS(69, 68, /* RX + TX */
				70, 71, /* RTS + CTS */
				NS921X_GPIO_FUNC_0),
};
struct platform_device ns921x_fim_serial0 = {
	.name              = "fim-serial",
	.id                = 0,
	.dev.platform_data = &fim_serial_data0,
};
EXPORT_SYMBOL(ns921x_fim_serial0);

static struct fim_serial_platform_data fim_serial_data1 = {
	.fim_nr        = 1,
	NS921X_FIM_SERIAL_GPIOS(73, 72, /* RX + TX */
				74, 75, /* RTS + CTS */
				NS921X_GPIO_FUNC_1),
};
struct platform_device ns921x_fim_serial1 = {
	.name              = "fim-serial",
	.id                = 1,
	.dev.platform_data = &fim_serial_data1,
};
EXPORT_SYMBOL(ns921x_fim_serial1);
#endif /* CONFIG_FIM_SERIAL */



#if defined(CONFIG_FIM_SDIO)
static struct fim_sdio_platform_data fim_sdio_data0 = {
	.fim_nr        = 0,
	NS921X_FIM_SDIO_GPIOS(68, 69, 70, 71, /* D0 to D3 */
			      72, 73,         /* WP + CD */
			      76, 77,         /* CLK + CMD */
			      NS921X_GPIO_FUNC_0),
};
struct platform_device ns921x_fim_sdio0 = {
	.name              = "fim-sdio",
	.id                = 0,
	.dev.platform_data = &fim_sdio_data0,
};
EXPORT_SYMBOL(ns921x_fim_sdio0);

static struct fim_sdio_platform_data fim_sdio_data1 = {
	.fim_nr        = 1,
	NS921X_FIM_SDIO_GPIOS(68, 69, 70, 71, /* D0 to D3 */
			      72, 73,         /* WP + CD */
			      76, 77,         /* CLK + CMD */
			      NS921X_GPIO_FUNC_1),
};
struct platform_device ns921x_fim_sdio1 = {
	.name              = "fim-sdio",
	.id                = 1,
	.dev.platform_data = &fim_sdio_data1,
};
EXPORT_SYMBOL(ns921x_fim_sdio1);

#endif /* CONFIG_FIM_SDIO */
