/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * This system-wise GPIO function initializes the pins during system startup.
 * All the statically linked device drivers should put the proper GPIO initialization
 * code inside this function. It is called by \b fixup_mx31ads() during
 * system startup. This function is board specific.
 */
void mx33ads_gpio_init(void)
{
 /*TODO*/}

/*!
 * Setup GPIO for a UART port to be active
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_active(int port, int no_irda)
{
 /*TODO*/}

/*!
 * Setup GPIO for a UART port to be inactive
 *
 * @param  port         a UART port
 * @param  no_irda      indicates if the port is used for SIR
 */
void gpio_uart_inactive(int port, int no_irda)
{
 /*TODO*/}

/*!
 * Configure the IOMUX GPR register to receive shared SDMA UART events
 *
 * @param  port         a UART port
 */
void config_uartdma_event(int port)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);
EXPORT_SYMBOL(config_uartdma_event);

/*!
 * Setup GPIO for Keypad  to be active
 *
 */
void gpio_keypad_active(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_keypad_active);

/*!
 * Setup GPIO for Keypad to be inactive
 *
 */
void gpio_keypad_inactive(void)
{
 /*TODO*/}

EXPORT_SYMBOL(gpio_keypad_inactive);

/*!
 * Setup GPIO for a CSPI device to be active
 *
 * @param  cspi_mod         an CSPI device
 */
void gpio_spi_active(int cspi_mod)
{
 /*TODO*/}

/*!
 * Setup GPIO for a CSPI device to be inactive
 *
 * @param  cspi_mod         a CSPI device
 */
void gpio_spi_inactive(int cspi_mod)
{
 /*TODO*/}

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
 /*TODO*/}

/*!
 * Setup GPIO for an I2C device to be inactive
 *
 * @param  i2c_num         an I2C device
 */
void gpio_i2c_inactive(int i2c_num)
{
 /*TODO*/}

/*!
 * This function configures the IOMux block for MC13783 standard operations.
 *
 */
void gpio_mc13783_active(void)
{
 /*TODO*/}

/*!
 * This function clears the MC13783 intrrupt.
 *
 */
void gpio_mc13783_clear_int(void)
{
 /*TODO*/}

/*!
 * This function return the SPI connected to MC13783.
 *
 */
int gpio_mc13783_get_spi(void)
{
	 /*TODO*/ return 1;
}

/*!
 * This function return the SPI smave select for MC13783.
 *
 */
int gpio_mc13783_get_ss(void)
{
	 /*TODO*/ return 0;
}

/*!
 * This function activates DAM ports 4 & 5 to enable
 * audio I/O. Thsi function is called from mx31ads_gpio_init
 * function, which is board-specific.
 */
void gpio_activate_audio_ports(void)
{
}

/*!
 * Setup GPIO for SDHC to be active
 *
 * @param module SDHC module number
 */
void gpio_sdhc_active(int module)
{
#if 0
	switch (module) {
	case 0:
		mxc_request_iomux(MX33_PIN_SD1_CLK, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX33_PIN_SD1_CMD, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX33_PIN_SD1_DATA0, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX33_PIN_SD1_DATA1, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX33_PIN_SD1_DATA2, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		mxc_request_iomux(MX33_PIN_SD1_DATA3, OUTPUTCONFIG_FUNC,
				  INPUTCONFIG_FUNC);
		break;
	case 1:
		mxc_request_iomux(MX33_PIN_PC_CD2_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX33_PIN_PC_CD1_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX33_PIN_PC_WAIT_B, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX33_PIN_PC_READY, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX33_PIN_PC_VS1, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		mxc_request_iomux(MX33_PIN_PC_PWRON, OUTPUTCONFIG_ALT1,
				  INPUTCONFIG_ALT1);
		break;
	default:
		break;
	}
#endif
}

EXPORT_SYMBOL(gpio_sdhc_active);

/*!
 * Setup GPIO for SDHC1 to be inactive
 *
 * @param module SDHC module number
 */
void gpio_sdhc_inactive(int module)
{
#if 0
	switch (module) {
	case 0:
		mxc_request_iomux(MX33_PIN_SD1_CLK, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_SD1_CMD, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_SD1_DATA0, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_SD1_DATA1, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_SD1_DATA2, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_SD1_DATA3, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		break;
	case 1:
		/* TODO:what are the pins for SDHC2? */
		mxc_request_iomux(MX33_PIN_PC_CD2_B, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_PC_CD1_B, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_PC_WAIT_B, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_PC_READY, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_PC_VS1, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		mxc_request_iomux(MX33_PIN_PC_PWRON, OUTPUTCONFIG_GPIO,
				  INPUTCONFIG_NONE);
		break;
	default:
		break;
	}
#endif
}

EXPORT_SYMBOL(gpio_sdhc_inactive);

/*
 * Probe for the card. If present the GPIO data would be set.
 */
int sdhc_get_card_det_status(struct device *dev)
{
#if 0
	if (to_platform_device(dev)->id == 0) {
		return mxc_get_gpio_datain(MX33_PIN_GPIO1_1);
	} else {
		return mxc_get_gpio_datain(MX33_PIN_GPIO1_2);
	}
#endif
}

EXPORT_SYMBOL(sdhc_get_card_det_status);

/*
 * Return the card detect pin.
 */
int sdhc_init_card_det(int id)
{
#if 0
	if (id == 0) {
		iomux_config_mux(MX33_PIN_GPIO1_1, OUTPUTCONFIG_GPIO,
				 INPUTCONFIG_GPIO);
		return IOMUX_TO_IRQ(MX33_PIN_GPIO1_1);
	} else {
		iomux_config_mux(MX33_PIN_GPIO1_2, OUTPUTCONFIG_GPIO,
				 INPUTCONFIG_GPIO);
		return IOMUX_TO_IRQ(MX33_PIN_GPIO1_2);

	}
#endif
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

/*!
 * Setup EDIO/IOMUX for external UART.
 *
 * @param port UART port
 * @param irq Interrupt line to allocate
 * @param handler Function to be called when the IRQ occurs
 * @param irq_flags Interrupt type flags
 * @param devname An ascii name for the claiming device
 * @param dev_id A cookie passed back to the handler function
 * @return  Returns 0 if the interrupt was successfully requested,
 *          otherwise returns an error code.
 */
int extuart_intr_setup(unsigned int port, unsigned int irq,
		       irqreturn_t(*handler) (int, void *, struct pt_regs *),
		       unsigned long irq_flags, const char *devname,
		       void *dev_id)
{
	 /*TODO*/ return 0;
}

/*!
 * Get the EDIO interrupt, clear if set.
 *
 * @param port UART port
 */
void extuart_intr_clear(unsigned int port)
{
 /*TODO*/}

/*!
 * Do IOMUX configs required to put the
 * pin back in low power mode.
 *
 * @param port UART port
 * @param irq Interrupt line to free
 * @param dev_id Device identity to free
 * @return  Returns 0 if the interrupt was successfully freed,
 *          otherwise returns an error code.
 */
int extuart_intr_cleanup(unsigned int port, unsigned int irq, void *dev_id)
{
	 /*TODO*/ return 0;
}

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
