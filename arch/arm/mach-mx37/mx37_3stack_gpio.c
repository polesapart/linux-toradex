/*
 * Copyright 2007-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include "iomux.h"

/*!
 * @file mx31ads_gpio.c
 *
 * @brief This file contains all the GPIO setup functions for the board.
 *
 * @ingroup GPIO
 */

void gpio_activate_audio_ports(void);

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
	 * and enable the UART transceivers
	 */
	switch (port) {
		/* UART 1 IOMUX Configs */
	case 0:
		mxc_request_iomux(MX37_PIN_UART1_RXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_RXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_iomux_set_input(MUX_IN_UART1_UART_RXD_MUX, INPUT_CTL_PATH4);
		mxc_request_iomux(MX37_PIN_UART1_TXD, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_TXD, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST);
		mxc_request_iomux(MX37_PIN_UART1_RTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_RTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		mxc_iomux_set_input(MUX_IN_UART1_UART_RTS_B, INPUT_CTL_PATH4);
		mxc_request_iomux(MX37_PIN_UART1_CTS, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_UART1_CTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
				  PAD_CTL_DRV_HIGH);
		break;
	case 1:
		break;
	case 2:
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{

}

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{

}

EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);
EXPORT_SYMBOL(config_uartdma_event);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
	switch (cspi_mod) {
	case 0:
		break;
	case 1:
		/* SPI2 */
		mxc_request_iomux(MX37_PIN_CSPI2_MISO, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_MISO, PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_GRP_H9, PAD_CTL_HYS_ENABLE);

		mxc_request_iomux(MX37_PIN_CSPI2_MOSI, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_MOSI, PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX37_PIN_UART1_CTS, IOMUX_CONFIG_ALT3);
		mxc_iomux_set_pad(MX37_PIN_UART1_CTS, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_PKE_ENABLE);

		mxc_request_iomux(MX37_PIN_CSPI2_SCLK, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_SCLK, PAD_CTL_HYS_ENABLE |
				  PAD_CTL_SRE_FAST);

		mxc_request_iomux(MX37_PIN_CSPI2_SS0, IOMUX_CONFIG_ALT0);
		mxc_iomux_set_pad(MX37_PIN_CSPI2_SS0, PAD_CTL_SRE_FAST);
		mxc_iomux_set_pad(MX37_PIN_GRP_H10, PAD_CTL_HYS_ENABLE);
		break;
	case 2:
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
}

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_active(void)
{
 /*TODO*/}

/*!
 * Setup 1-Wire to be active
 */
void gpio_owire_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_owire_active);
EXPORT_SYMBOL(gpio_owire_inactive);

/*!
 * Setup GPIO for an I2C device to be active
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_active(int i2c_num)
{
	iomux_pad_config_t regval = 0;

	switch (i2c_num) {
	case 0:
		/* Touch */
		/* select I2C1_SCK as daisy chain input */
		mxc_iomux_set_input(MUX_IN_I2C1_SCL, INPUT_CTL_PATH1);
		/* OpenDrain enabled, 100k PU enabled */
		mxc_iomux_set_pad(MX37_PIN_I2C1_CLK,
				  PAD_CTL_100K_PU |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE);
		mxc_request_iomux(MX37_PIN_I2C1_CLK, IOMUX_CONFIG_ALT0);
		/*select I2C1_SDA as daisy chain input */
		mxc_iomux_set_input(MUX_IN_I2C1_SDA, INPUT_CTL_PATH1);
		/* OpenDrain enabled, 100k PU enabled */
		mxc_iomux_set_pad(MX37_PIN_I2C1_DAT,
				  PAD_CTL_100K_PU |
				  PAD_CTL_ODE_OPENDRAIN_ENABLE);
		mxc_request_iomux(MX37_PIN_I2C1_DAT, IOMUX_CONFIG_ALT0);
		break;
	case 1:
		/* PMIC */
		/*select I2C2_SCL as daisy chain input */
		mxc_iomux_set_input(MUX_IN_I2C2_SCL, INPUT_CTL_PATH1);
		regval = PAD_CTL_HYS_ENABLE | PAD_CTL_PKE_ENABLE |
		    PAD_CTL_PUE_PULL | PAD_CTL_100K_PU |
		    PAD_CTL_ODE_OPENDRAIN_ENABLE | PAD_CTL_DRV_HIGH;
		mxc_iomux_set_pad(MX37_PIN_GPIO1_0, regval);
		mxc_request_iomux(MX37_PIN_GPIO1_0,
				  (IOMUX_CONFIG_SION | IOMUX_CONFIG_ALT2));

		/*select I2C2_SDA as daisy chain input */
		mxc_iomux_set_input(MUX_IN_I2C2_SDA, INPUT_CTL_PATH1);
		mxc_iomux_set_pad(MX37_PIN_GPIO1_1, regval);
		mxc_request_iomux(MX37_PIN_GPIO1_1,
				  (IOMUX_CONFIG_SION | IOMUX_CONFIG_ALT2));
		break;
	case 2:
		break;
	default:
		break;
	}
}

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
 /*TODO*/}

/*!
 * This function activates DAM ports 4 & 5 to enable
 * audio I/O.
 */
void gpio_activate_audio_ports(void)
{
	unsigned int pad_val;

	/* AUD4_TXD */
	mxc_request_iomux(MX37_PIN_DISP1_DAT20, IOMUX_CONFIG_ALT5);
	/* AUD4_RXD */
	mxc_request_iomux(MX37_PIN_DISP1_DAT21, IOMUX_CONFIG_ALT5);
	/* AUD4_TXC */
	mxc_request_iomux(MX37_PIN_DISP1_DAT22, IOMUX_CONFIG_ALT5);
	/* AUD4_TXFS */
	mxc_request_iomux(MX37_PIN_DISP1_DAT23, IOMUX_CONFIG_ALT5);

	pad_val = PAD_CTL_PKE_ENABLE | PAD_CTL_PUE_PULL |
	    PAD_CTL_DRV_HIGH | PAD_CTL_SRE_FAST;
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_CK, PAD_CTL_100K_PU | pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_CK, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_RXD, pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_RXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_TXD, pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_TXD, IOMUX_CONFIG_ALT0);
	mxc_iomux_set_pad(MX37_PIN_AUD5_WB_FS, PAD_CTL_100K_PU | pad_val);
	mxc_request_iomux(MX37_PIN_AUD5_WB_FS, IOMUX_CONFIG_ALT0);

	/* Enable hysteresis for AUD5_WB_CK, AUD5_WB_RXD, AUD5_WB_TXD, AUD5_WB_FS */
	mxc_iomux_set_pad(MX37_PIN_GRP_H5, PAD_CTL_HYS_ENABLE);
}

EXPORT_SYMBOL(gpio_activate_audio_ports);

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{

}

EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{

}

EXPORT_SYMBOL(gpio_sdhc_inactive);

/*
 * Probe for the card. If present the GPIO data would be set.
 */
int sdhc_get_card_det_status(struct device *dev)
{
	return 0;
}

EXPORT_SYMBOL(sdhc_get_card_det_status);

/*
 * Return the card detect pin.
 */
int sdhc_init_card_det(int id)
{
	return 0;
}

EXPORT_SYMBOL(sdhc_init_card_det);

/*!
 * Setup GPIO for LCD to be active
 *
 */
void gpio_lcd_active(void)
{
 /*TODO*/}

/*!
 * Setup GPIO for LCD to be inactive
 *
 */
void gpio_lcd_inactive(void)
{
 /*TODO*/}

/*!
 * Setup pins for SLCD to be active
 *
 */
void slcd_gpio_config(void)
{
 /*TODO*/}

/*!
 * Switch to the specified sensor - MX33 ADS has two
 *
 */
void gpio_sensor_select(int sensor)
{
 /*TODO*/}

/*!
 * Setup GPIO for sensor to be active
 *
 */
void gpio_sensor_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_sensor_active);

/*!
 * Setup GPIO for sensor to be inactive
 *
 */
void gpio_sensor_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_sensor_inactive);

/*!
 * Setup GPIO for ATA interface
 *
 */
void gpio_ata_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_ata_active);

/*!
 * Restore ATA interface pins to reset values
 *
 */
void gpio_ata_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_ata_inactive);

/*
 * USB Host 1
 * pins conflict with SPI1, ATA, UART3
 */
int gpio_usbh1_active(void)
{
	 /*TODO*/ return 0;
}

EXPORT_SYMBOL(gpio_usbh1_active);

void gpio_usbh1_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_usbh1_inactive);

/*
 * USB Host 2
 * pins conflict with UART5, PCMCIA
 */
int gpio_usbh2_active(void)
{
	 /*TODO*/ return 0;
}

EXPORT_SYMBOL(gpio_usbh2_active);

void gpio_usbh2_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_usbh2_inactive);

/*
 * USB OTG HS port
 */
int gpio_usbotg_hs_active(void)
{
	 /*TODO*/ return 0;
}

EXPORT_SYMBOL(gpio_usbotg_hs_active);

void gpio_usbotg_hs_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_usbotg_hs_inactive);

/*
 * USB OTG FS port
 */
int gpio_usbotg_fs_active(void)
{
	 /*TODO*/ return 0;
}

EXPORT_SYMBOL(gpio_usbotg_fs_active);

void gpio_usbotg_fs_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_usbotg_fs_inactive);

/*!
 * Setup GPIO for PCMCIA interface
 *
 */
void gpio_pcmcia_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_pcmcia_active);

/*!
 * Setup GPIO for pcmcia to be inactive
 */
void gpio_pcmcia_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_pcmcia_inactive);

/*!
 * Setup GPIO for fec to be active
 */
void gpio_fec_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_fec_active);
/*!
 * Setup GPIO for fec to be inactive
 */
void gpio_fec_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_fec_inactive);

void gpio_spdif_active(void)
{
	iomux_pad_config_t regval = 0;
	regval =
	    PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PKE_ENABLE |
	    PAD_CTL_100K_PU;
	mxc_iomux_set_pad(MX37_PIN_AUD3_BB_RXD, regval);
	mxc_request_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT1);
}

EXPORT_SYMBOL(gpio_spdif_active);

void gpio_spdif_inactive(void)
{
	mxc_free_iomux(MX37_PIN_AUD3_BB_RXD, IOMUX_CONFIG_ALT1);
}

EXPORT_SYMBOL(gpio_spdif_inactive);
