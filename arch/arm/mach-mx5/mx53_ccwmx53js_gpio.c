/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "iomux.h"
#include "board-ccwmx53.h"

#if defined(CONFIG_SERIAL_MXC) || defined(CONFIG_SERIAL_MXC_MODULE)

void gpio_uart_active(int port, int no_irda)
{
	/* Configure the IOMUX control registers for the UART signals */
	switch (port) {

	case 0:		/* UART 1 IOMUX Configs */
#ifdef CONFIG_UART1_ENABLED

		/* UART1 MUX CONFIG */

#if defined(CONFIG_UART1_CTS_RTS_ENABLED) || defined(CONFIG_UART1_FULL_UART_ENABLED)

		/* UART1 MUX CONFIG */

#endif /* CONFIG_UART1_CTS_RTS_ENABLED */
#ifdef CONFIG_UART1_FULL_UART_ENABLED

		/* UART1 MUX CONFIG */

#endif /* CONFIG_UART1_FULL_UART_ENABLED */
#endif /* CONFIG_UART1_ENABLED */
		break;

	case 1:		/* UART 2 IOMUX Configs */
#ifdef CONFIG_UART2_ENABLED

		/* UART2 MUX CONFIG */

#ifdef CONFIG_UART2_CTS_RTS_ENABLED

		/* UART2 MUX CONFIG */

#endif /* CONFIG_UART2_CTS_RTS_ENABLED */
#endif /* CONFIG_UART2_CTS_RTS_ENABLED */
		break;

	case 2:		/* UART 3 IOMUX Configs */
#ifdef CONFIG_UART3_ENABLED

		/* UART3 MUX CONFIG */

#ifdef CONFIG_UART3_CTS_RTS_ENABLED

		/* UART3 MUX CONFIG */

#endif /* CONFIG_UART3_CTS_RTS_ENABLED */
#endif /* CONFIG_UART3_ENABLED */
		break;

	case 3:		/* UART 4 IOMUX Configs */
#ifdef CONFIG_UART4_ENABLED

		/* UART4 MUX CONFIG */

#ifdef CONFIG_UART4_CTS_RTS_ENABLED

		/* UART4 MUX CONFIG */

#endif /* CONFIG_UART4_CTS_RTS_ENABLED */
#endif /* CONFIG_UART4_ENABLED */
		break;

	case 4:		/* UART 5 IOMUX Configs */
#ifdef CONFIG_UART5_ENABLED

		/* UART5 MUX CONFIG */

#ifdef CONFIG_UART5_CTS_RTS_ENABLED

		/* UART5 MUX CONFIG */

#endif /* CONFIG_UART5_CTS_RTS_ENABLED */
#endif /* CONFIG_UART5_ENABLED */
		break;

	default:
		break;
	}
}
#else
void gpio_uart_active(int port, int no_irda) {}
#endif
void gpio_uart_inactive(int port, int no_irda) {}
EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);

void gpio_sdhc_active(int interface){}
EXPORT_SYMBOL(gpio_sdhc_active);
void gpio_sdhc_inactive(int module) {}
EXPORT_SYMBOL(gpio_sdhc_inactive);
