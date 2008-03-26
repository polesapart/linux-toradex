/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/gpio.h>
#include "board-mx35_3stack.h"
#include "iomux.h"

/*!
 * @file mach-mx35/mx35_3stack_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO_MX35
 */

/*!
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO
 * initialization code inside this function. It is called by \b fixup_mx31ads()
 * during system startup. This function is board specific.
 */
void mx35_3stack_gpio_init(void)
{
	/* config CS5 */
	mxc_request_iomux(MX35_PIN_CS5, MUX_CONFIG_FUNC);

}

/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
	/*
	 * Configure the IOMUX control registers for the UART signals
	 */
	switch (port) {
		/* UART 1 IOMUX Configs */
	case 0:
		mxc_request_iomux(MX35_PIN_RXD1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_TXD1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RTS1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CTS1, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_RXD1,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_TXD1,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX35_PIN_RTS1,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_CTS1,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);

		break;
		/* UART 2 IOMUX Configs */
	case 1:
		mxc_request_iomux(MX35_PIN_TXD2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RXD2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_RTS2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CTS2, MUX_CONFIG_FUNC);
		mxc_iomux_set_pad(MX35_PIN_RXD2,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_TXD2,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		mxc_iomux_set_pad(MX35_PIN_RTS2,
				  PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE |
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PU);
		mxc_iomux_set_pad(MX35_PIN_CTS2,
				  PAD_CTL_PUE_PUD | PAD_CTL_100K_PD);
		break;
		/* UART 3 IOMUX Configs */
	case 2:
		mxc_request_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_ALT2);
		mxc_request_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_ALT2);
		mxc_request_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_ALT2);
		mxc_request_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_ALT2);

		mxc_iomux_set_input(MUX_IN_UART3_UART_RTS_B, INPUT_CTL_PATH2);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH3);
		break;
	default:
		break;
	}

}

EXPORT_SYMBOL(gpio_uart_active);

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
	switch (port) {
	case 0:
		mxc_request_gpio(MX35_PIN_RXD1);
		mxc_request_gpio(MX35_PIN_TXD1);
		mxc_request_gpio(MX35_PIN_RTS1);
		mxc_request_gpio(MX35_PIN_CTS1);

		mxc_free_iomux(MX35_PIN_RXD1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_TXD1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_RTS1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CTS1, MUX_CONFIG_GPIO);
		break;
	case 1:
		mxc_request_gpio(MX35_PIN_RXD2);
		mxc_request_gpio(MX35_PIN_TXD2);
		mxc_request_gpio(MX35_PIN_RTS2);
		mxc_request_gpio(MX35_PIN_CTS2);

		mxc_free_iomux(MX35_PIN_RXD2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_TXD2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_RTS2, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CTS2, MUX_CONFIG_GPIO);
		break;
	case 2:
		mxc_request_gpio(MX35_PIN_FEC_TX_CLK);
		mxc_request_gpio(MX35_PIN_FEC_RX_CLK);
		mxc_request_gpio(MX35_PIN_FEC_COL);
		mxc_request_gpio(MX35_PIN_FEC_RX_DV);

		mxc_free_iomux(MX35_PIN_FEC_TX_CLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_FEC_RX_CLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_FEC_COL, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_FEC_RX_DV, MUX_CONFIG_GPIO);

		mxc_iomux_set_input(MUX_IN_UART3_UART_RTS_B, INPUT_CTL_PATH0);
		mxc_iomux_set_input(MUX_IN_UART3_UART_RXD_MUX, INPUT_CTL_PATH0);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_uart_inactive);

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{
}

EXPORT_SYMBOL(config_uartdma_event);

void gpio_fec_active(void)
{

}

EXPORT_SYMBOL(gpio_fec_active);

void gpio_fec_inactive(void)
{

}

EXPORT_SYMBOL(gpio_fec_inactive);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		mxc_request_iomux(MX35_PIN_I2C1_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C1_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C1_CLK, 0x1e8);
		mxc_iomux_set_pad(MX35_PIN_I2C1_DAT, 0x1e8);
		break;
	case 1:
		mxc_request_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C2_CLK, 0x1e8);
		mxc_iomux_set_pad(MX35_PIN_I2C2_DAT, 0x1e8);

		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_i2c_active);

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	/*TODO:complete inactive function */
}

EXPORT_SYMBOL(gpio_i2c_inactive);
