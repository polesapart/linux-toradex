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

#define PAD_CONFIG (PAD_CTL_HYS_SCHMITZ | PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD | PAD_CTL_ODE_OpenDrain)

	switch (i2c_num) {
	case 0:
		mxc_request_iomux(MX35_PIN_I2C1_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C1_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C1_CLK, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_I2C1_DAT, PAD_CONFIG);
		break;
	case 1:
		mxc_request_iomux(MX35_PIN_I2C2_CLK, MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_I2C2_DAT, MUX_CONFIG_SION);

		mxc_iomux_set_pad(MX35_PIN_I2C2_CLK, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_I2C2_DAT, PAD_CONFIG);

		break;
	case 2:
		mxc_request_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_ALT1);
		mxc_request_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_ALT1);
		mxc_iomux_set_pad(MX35_PIN_TX3_RX2, PAD_CONFIG);
		mxc_iomux_set_pad(MX35_PIN_TX2_RX3, PAD_CONFIG);
		break;
	default:
		break;
	}

#undef PAD_CONFIG

}

EXPORT_SYMBOL(gpio_i2c_active);

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
	switch (i2c_num) {
	case 0:
		break;
	case 1:
		break;
	case 2:
		mxc_request_iomux(MX35_PIN_TX3_RX2, MUX_CONFIG_GPIO);
		mxc_request_iomux(MX35_PIN_TX2_RX3, MUX_CONFIG_GPIO);
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_i2c_inactive);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		mxc_request_iomux(MX35_PIN_CSPI1_MOSI, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_MISO, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SS0, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SS1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SCLK, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_CSPI1_SPI_RDY, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_CSPI1_MOSI,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_MISO,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SS0,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_CMOS |
				  PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SS1,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_ODE_CMOS |
				  PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SCLK,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PD | PAD_CTL_DRV_NORMAL);
		mxc_iomux_set_pad(MX35_PIN_CSPI1_SPI_RDY,
				  PAD_CTL_DRV_3_3V | PAD_CTL_HYS_SCHMITZ |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PUD |
				  PAD_CTL_100K_PU | PAD_CTL_DRV_NORMAL);
		break;
	case 1:
		/* SPI2 */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_spi_active);

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		/* SPI1 */
		mxc_request_gpio(MX35_PIN_CSPI1_MOSI);
		mxc_request_gpio(MX35_PIN_CSPI1_MISO);
		mxc_request_gpio(MX35_PIN_CSPI1_SS0);
		mxc_request_gpio(MX35_PIN_CSPI1_SS1);
		mxc_request_gpio(MX35_PIN_CSPI1_SCLK);
		mxc_request_gpio(MX35_PIN_CSPI1_SPI_RDY);

		mxc_free_iomux(MX35_PIN_CSPI1_MOSI, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_MISO, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SS0, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SS1, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SCLK, MUX_CONFIG_GPIO);
		mxc_free_iomux(MX35_PIN_CSPI1_SPI_RDY, MUX_CONFIG_GPIO);
		break;
	case 1:
		/* SPI2 */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_spi_inactive);

/*!
 * Setup GPIO for LCD to be active
 */
void gpio_lcd_active(void)
{
	mxc_request_iomux(MX35_PIN_LD0, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD1, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD2, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD3, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD4, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD5, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD6, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD7, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD8, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD9, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD10, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD11, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD12, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD13, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD14, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD15, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD16, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_LD17, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_VSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_HSYNC, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_FPSHIFT, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_D3_DRDY, MUX_CONFIG_FUNC);
	mxc_request_iomux(MX35_PIN_CONTRAST, MUX_CONFIG_FUNC);
}

EXPORT_SYMBOL(gpio_lcd_active);

/*!
 * Setup GPIO for LCD to be inactive
 */
void gpio_lcd_inactive(void)
{
}

EXPORT_SYMBOL(gpio_lcd_inactive);

/*!
 * Setup pin for touchscreen
 */
void gpio_tsc_active(void)
{
	unsigned int pad_val = PAD_CTL_PKE_ENABLE | PAD_CTL_100K_PU;
	mxc_request_iomux(MX35_PIN_CAPTURE, MUX_CONFIG_GPIO);
	mxc_iomux_set_pad(MX35_PIN_CAPTURE, pad_val);
	mxc_set_gpio_direction(MX35_PIN_CAPTURE, 1);
}

/*!
 * Release pin for touchscreen
 */
void gpio_tsc_inactive(void)
{
	mxc_free_iomux(MX35_PIN_CAPTURE, MUX_CONFIG_GPIO);
}

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
	unsigned int pad_val;

	switch (module) {
	case 0:
		mxc_request_iomux(MX35_PIN_SD1_CLK,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_CMD,
				  MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_request_iomux(MX35_PIN_SD1_DATA0, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_SD1_DATA1, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_SD1_DATA2, MUX_CONFIG_FUNC);
		mxc_request_iomux(MX35_PIN_SD1_DATA3, MUX_CONFIG_FUNC);

		pad_val = PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST;

		mxc_iomux_set_pad(MX35_PIN_SD1_CLK, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_CMD, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA0, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA1, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA2, pad_val);
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA3, pad_val);
		break;
	case 1:
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
	switch (module) {
	case 0:
		mxc_free_iomux(MX35_PIN_SD1_CLK,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_CMD,
			       MUX_CONFIG_FUNC | MUX_CONFIG_SION);
		mxc_free_iomux(MX35_PIN_SD1_DATA0, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX35_PIN_SD1_DATA1, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX35_PIN_SD1_DATA2, MUX_CONFIG_FUNC);
		mxc_free_iomux(MX35_PIN_SD1_DATA3, MUX_CONFIG_FUNC);

		mxc_iomux_set_pad(MX35_PIN_SD1_CLK,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_CMD,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA0,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA1,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA2,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		mxc_iomux_set_pad(MX35_PIN_SD1_DATA3,
				  (PAD_CTL_DRV_NORMAL | PAD_CTL_SRE_SLOW));
		break;
	case 1:
		/* TODO:what are the pins for SDHC2? */
		break;
	default:
		break;
	}
}

EXPORT_SYMBOL(gpio_sdhc_inactive);
