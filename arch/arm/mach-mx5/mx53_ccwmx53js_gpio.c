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
#include <mach/iomux-mx53.h>

#include "board-ccwmx53.h"
#include "devices_ccwmx53.h"

#if defined(CONFIG_SERIAL_MXC) || defined(CONFIG_SERIAL_MXC_MODULE)

#ifdef CONFIG_UART1_ENABLED
static struct pad_desc ccwmx53_uart1_pads[] = {
	MX53_PAD_ATA_DMACK__UART1_RXD,
	MX53_PAD_ATA_DIOW__UART1_TXD,
#if defined(CONFIG_UART1_CTS_RTS_ENABLED) || defined(CONFIG_UART1_FULL_UART_ENABLED)
#endif /* (CONFIG_UART1_CTS_RTS_ENABLED) || (CONFIG_UART1_FULL_UART_ENABLED) */
#ifdef CONFIG_UART1_FULL_UART_ENABLED
#endif /* CONFIG_UART1_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART1_ENABLED */

#ifdef CONFIG_UART2_ENABLED
static struct pad_desc ccwmx53_uart2_pads[] = {
#if defined(CONFIG_UART2_CTS_RTS_ENABLED) || defined(CONFIG_UART2_FULL_UART_ENABLED)
#endif /* (CONFIG_UART2_CTS_RTS_ENABLED) || (CONFIG_UART2_FULL_UART_ENABLED) */
#ifdef CONFIG_UART2_FULL_UART_ENABLED
#endif /* CONFIG_UART2_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART2_ENABLED */

#ifdef CONFIG_UART3_ENABLED
static struct pad_desc ccwmx53_uart3_pads[] = {
#if defined(CONFIG_UART3_CTS_RTS_ENABLED) || defined(CONFIG_UART3_FULL_UART_ENABLED)
#endif /* (CONFIG_UART3_CTS_RTS_ENABLED) || (CONFIG_UART3_FULL_UART_ENABLED) */
#ifdef CONFIG_UART3_FULL_UART_ENABLED
#endif /* CONFIG_UART3_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART3_ENABLED */

#ifdef CONFIG_UART4_ENABLED
static struct pad_desc ccwmx53_uart4_pads[] = {
#if defined(CONFIG_UART4_CTS_RTS_ENABLED) || defined(CONFIG_UART4_FULL_UART_ENABLED)
#endif /* (CONFIG_UART4_CTS_RTS_ENABLED) || (CONFIG_UART4_FULL_UART_ENABLED) */
#ifdef CONFIG_UART4_FULL_UART_ENABLED
#endif /* CONFIG_UART4_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART4_ENABLED */

#ifdef CONFIG_UART5_ENABLED
static struct pad_desc ccwmx53_uart5_pads[] = {
#if defined(CONFIG_UART5_CTS_RTS_ENABLED) || defined(CONFIG_UART5_FULL_UART_ENABLED)
#endif /* (CONFIG_UART5_CTS_RTS_ENABLED) || (CONFIG_UART5_FULL_UART_ENABLED) */
#ifdef CONFIG_UART5_FULL_UART_ENABLED
#endif /* CONFIG_UART5_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART5_ENABLED */


void gpio_uart_active(int port, int no_irda)
{
	/* Configure the IOMUX control registers for the UART signals */
	switch (port) {
#ifdef CONFIG_UART1_ENABLED
	case 0:		/* UART 1 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_uart1_pads,
						 ARRAY_SIZE(ccwmx53_uart1_pads));
		break;
#endif
#ifdef CONFIG_UART2_ENABLED
	case 1:		/* UART 2 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_uart2_pads,
						 ARRAY_SIZE(ccwmx53_uart2_pads));
		break;
#endif
#ifdef CONFIG_UART3_ENABLED
	case 2:		/* UART 3 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_uart3_pads,
						 ARRAY_SIZE(ccwmx53_uart3_pads));
		break;
#endif
#ifdef CONFIG_UART4_ENABLED
	case 3:		/* UART 4 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_uart4_pads,
						 ARRAY_SIZE(ccwmx53_uart4_pads));
		break;
#endif
#ifdef CONFIG_UART5_ENABLED
	case 4:		/* UART 5 IOMUX Configs */
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_uart5_pads,
						 ARRAY_SIZE(ccwmx53_uart5_pads));
		break;
#endif
	}
}
#else
void gpio_uart_active(int port, int no_irda) {}
#endif
void gpio_uart_inactive(int port, int no_irda) {}
EXPORT_SYMBOL(gpio_uart_active);
EXPORT_SYMBOL(gpio_uart_inactive);


#ifdef ESDHCI_MXC_SELECT1
static struct pad_desc ccwmx53_mmc1_pads[] = {
	MX53_PAD_SD1_CLK__SD1_CLK,
	MX53_PAD_SD1_CMD__SD1_CMD,
	MX53_PAD_SD1_DATA0__SD1_DATA0,
#if defined(ESDHCI_MXC_SELECT1_8BIT_PORT) || defined(ESDHCI_MXC_SELECT1_4BIT_PORT)
	MX53_PAD_SD1_DATA1__SD1_DATA1,
	MX53_PAD_SD1_DATA2__SD1_DATA2,
	MX53_PAD_SD1_DATA3__SD1_DATA3,
#endif /* (ESDHCI_MXC_SELECT1_8BIT_PORT) || (ESDHCI_MXC_SELECT1_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT1_8BIT_PORT
	MX53_PAD_ATA_DATA8__SD1_DATA4,
	MX53_PAD_ATA_DATA9__SD1_DATA5,
	MX53_PAD_ATA_DATA10__SD1_DATA6,
	MX53_PAD_ATA_DATA11__SD1_DATA7,
#endif /* ESDHCI_MXC_SELECT1_8BIT_PORT */
#ifdef ESDHC1_WP_PAD
	ESDHC1_WP_PAD,
#endif
#ifdef ESDHC1_CD_PAD
	ESDHC1_CD_PAD,
#endif
};
#endif /* ESDHCI_MXC_SELECT1 */

#ifdef ESDHCI_MXC_SELECT2
static struct pad_desc ccwmx53_mmc2_pads[] = {
	MX53_PAD_SD2_CLK__SD2_CLK,
	MX53_PAD_SD2_CMD__SD2_CMD,
	MX53_PAD_SD2_DATA0__SD2_DAT0,
#if defined(ESDHCI_MXC_SELECT2_8BIT_PORT) || defined(ESDHCI_MXC_SELECT2_4BIT_PORT)
	MX53_PAD_SD2_DATA1__SD2_DAT1,
	MX53_PAD_SD2_DATA2__SD2_DAT2,
	MX53_PAD_SD2_DATA3__SD2_DAT3,
#endif /* (ESDHCI_MXC_SELECT2_8BIT_PORT) || (ESDHCI_MXC_SELECT2_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT2_8BIT_PORT
	MX53_PAD_ATA_DATA12__SD2_DAT4,
	MX53_PAD_ATA_DATA13__SD2_DAT5,
	MX53_PAD_ATA_DATA14__SD2_DAT6,
	MX53_PAD_ATA_DATA15__SD2_DAT7,
#endif /* ESDHCI_MXC_SELECT2_8BIT_PORT */
#ifdef ESDHC2_WP_PAD
	ESDHC2_WP_PAD,
#endif
#ifdef ESDHC2_CD_PAD
	ESDHC2_CD_PAD,
#endif
};
#endif /* ESDHCI_MXC_SELECT2 */

#ifdef ESDHCI_MXC_SELECT3
static struct pad_desc ccwmx53_mmc3_pads[] = {
	MX53_PAD_ATA_IORDY__SD3_CLK,
	MX53_PAD_ATA_RESET_B__SD3_CMD,
	MX53_PAD_ATA_DATA8__SD3_DAT0,
#if defined(ESDHCI_MXC_SELECT3_8BIT_PORT) || defined(ESDHCI_MXC_SELECT3_4BIT_PORT)
	MX53_PAD_ATA_DATA9__SD3_DAT1,
	MX53_PAD_ATA_DATA10__SD3_DAT2,
	MX53_PAD_ATA_DATA11__SD3_DAT3,
#endif /* (ESDHCI_MXC_SELECT3_8BIT_PORT) || (ESDHCI_MXC_SELECT3_4BIT_PORT) */
#ifdef ESDHCI_MXC_SELECT3_8BIT_PORT
	MX53_PAD_ATA_DATA0__SD3_DAT4,
	MX53_PAD_ATA_DATA1__SD3_DAT5,
	MX53_PAD_ATA_DATA2__SD3_DAT6,
	MX53_PAD_ATA_DATA3__SD3_DAT7,
#endif /* ESDHCI_MXC_SELECT3_8BIT_PORT */
#ifdef ESDHC3_WP_PAD
	ESDHC3_WP_PAD,
#endif
#ifdef ESDHC3_CD_PAD
	ESDHC3_CD_PAD,
#endif
};
#endif /* ESDHCI_MXC_SELECT3 */

void gpio_sdhc_active(int interface)
{
	switch (interface) {
#ifdef ESDHCI_MXC_SELECT1
	case 0:
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_mmc1_pads,
						 ARRAY_SIZE(ccwmx51_mmc1_pads));
		break;
#endif /* ESDHCI_MXC_SELECT1 */
#ifdef ESDHCI_MXC_SELECT2
	case 1:
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_mmc2_pads,
						 ARRAY_SIZE(ccwmx51_mmc2_pads));
		break;
#endif /* ESDHCI_MXC_SELECT2 */
#ifdef ESDHCI_MXC_SELECT3
	case 2:
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_mmc3_pads,
						 ARRAY_SIZE(ccwmx53_mmc3_pads));
		break;
#endif /* ESDHCI_MXC_SELECT3 */
#ifdef ESDHCI_MXC_SELECT4
	case 3:
		mxc_iomux_v3_setup_multiple_pads(ccwmx53_mmc4_pads,
						 ARRAY_SIZE(ccwmx53_mmc4_pads));
		break;
#endif /* ESDHCI_MXC_SELECT4 */
	}
}
EXPORT_SYMBOL(gpio_sdhc_active);
void gpio_sdhc_inactive(int module) {}
EXPORT_SYMBOL(gpio_sdhc_inactive);



#if defined(CONFIG_CCXMX5X_DISP0)
static struct pad_desc ccwmx53_disp0_pads[] = {
	MX53_PAD_DISP0_DAT0__DISP0_DAT0,
	MX53_PAD_DISP0_DAT1__DISP0_DAT1,
	MX53_PAD_DISP0_DAT2__DISP0_DAT2,
	MX53_PAD_DISP0_DAT3__DISP0_DAT3,
	MX53_PAD_DISP0_DAT4__DISP0_DAT4,
	MX53_PAD_DISP0_DAT5__DISP0_DAT5,
	MX53_PAD_DISP0_DAT6__DISP0_DAT6,
	MX53_PAD_DISP0_DAT7__DISP0_DAT7,
	MX53_PAD_DISP0_DAT8__DISP0_DAT8,
	MX53_PAD_DISP0_DAT9__DISP0_DAT9,
	MX53_PAD_DISP0_DAT10__DISP0_DAT10,
	MX53_PAD_DISP0_DAT11__DISP0_DAT11,
	MX53_PAD_DISP0_DAT12__DISP0_DAT12,
	MX53_PAD_DISP0_DAT13__DISP0_DAT13,
	MX53_PAD_DISP0_DAT14__DISP0_DAT14,
	MX53_PAD_DISP0_DAT15__DISP0_DAT15,
	MX53_PAD_DISP0_DAT16__DISP0_DAT16,
	MX53_PAD_DISP0_DAT17__DISP0_DAT17,
#if defined(CCXMX5X_DISP0_RGB888)
	MX53_PAD_DISP0_DAT18__DISP0_DAT18,
	MX53_PAD_DISP0_DAT19__DISP0_DAT19,
	MX53_PAD_DISP0_DAT20__DISP0_DAT20,
	MX53_PAD_DISP0_DAT21__DISP0_DAT21,
	MX53_PAD_DISP0_DAT22__DISP0_DAT22,
	MX53_PAD_DISP0_DAT23__DISP0_DAT23,
#endif
	MX53_PAD_DI0_DISP_CLK__DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__DI0_PIN15,
	MX53_PAD_DI0_PIN2__DI0_PIN2,
	MX53_PAD_DI0_PIN3__DI0_PIN3,
#if defined(DISP1_ENABLE_PAD)
	DISP1_ENABLE_PAD,
#endif
};
#endif /* CONFIG_CCWMX53_DISP0 */

void gpio_video_active(int vif, u32 pad)
{
#if defined(CONFIG_CCXMX5X_DISP0)
	mxc_iomux_v3_setup_multiple_pads(ccwmx53_disp0_pads,
					 ARRAY_SIZE(ccwmx53_disp0_pads));
#if defined(DISP1_ENABLE_PAD) && defined(DISP1_ENABLE_GPIO)
	gpio_request(DISP1_ENABLE_GPIO, "lcd1_enable");
	gpio_direction_output(DISP1_ENABLE_GPIO, 0);
#endif /* DISP1_ENABLE_PAD && DISP1_ENABLE_GPIO */
#endif /* CONFIG_CCXMX5X_DISP0 */
}

void gpio_video_inactive(int vif, u32 pad)
{
}
EXPORT_SYMBOL(gpio_video_active);
EXPORT_SYMBOL(gpio_video_inactive);

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct pad_desc ccwmx53_smsc911x_pads[] = {
	MX53_PAD_EIM_EB3__GPIO_2_31,
	MX53_PAD_EIM_D16__EIM_D16,
	MX53_PAD_EIM_D17__EIM_D17,
	MX53_PAD_EIM_D18__EIM_D18,
	MX53_PAD_EIM_D19__EIM_D19,
	MX53_PAD_EIM_D20__EIM_D20,
	MX53_PAD_EIM_D21__EIM_D21,
	MX53_PAD_EIM_D22__EIM_D22,
	MX53_PAD_EIM_D23__EIM_D23,
	MX53_PAD_EIM_D24__EIM_D24,
	MX53_PAD_EIM_D25__EIM_D25,
	MX53_PAD_EIM_D26__EIM_D26,
	MX53_PAD_EIM_D27__EIM_D27,
	MX53_PAD_EIM_D28__EIM_D28,
	MX53_PAD_EIM_D29__EIM_D29,
	MX53_PAD_EIM_D30__EIM_D30,
	MX53_PAD_EIM_D31__EIM_D31,
	MX53_PAD_EIM_DA0__EIM_DA0,
	MX53_PAD_EIM_DA1__EIM_DA1,
	MX53_PAD_EIM_DA2__EIM_DA2,
	MX53_PAD_EIM_DA3__EIM_DA3,
	MX53_PAD_EIM_DA4__EIM_DA4,
	MX53_PAD_EIM_DA5__EIM_DA5,
	MX53_PAD_EIM_DA6__EIM_DA6,
	MX53_PAD_EIM_OE__EIM_OE,
	MX53_PAD_EIM_RW__EIM_RW,
	MX53_PAD_EIM_CS1__EIM_CS1,
	MX53_PAD_GPIO_12__GPIO_4_2,     /* IRQ */
};

void gpio_smsc911x_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccwmx53_smsc911x_pads,
					 ARRAY_SIZE(ccwmx53_smsc911x_pads));

	/* Configure interrupt line as GPIO input, the iomux should be already setup */
	gpio_request(CCWMX53_EXT_IRQ_GPIO, "ext-eth-irq");
	gpio_direction_input(CCWMX53_EXT_IRQ_GPIO);
}
#endif

#ifdef CONFIG_SPI_MXC_SELECT1

static struct pad_desc ccwmx53_spi_pads[] = {
		MX53_PAD_CSI0_D7__ECSPI1_SS0,
		MX53_PAD_CSI0_D4__ECSPI_SCLK,
		MX53_PAD_CSI0_D6__ECSPI1_MISO,
		MX53_PAD_CSI0_D5__ECSPI1_MOSI,
		MX53_PAD_GPIO_19__GPIO_4_5,
};

void gpio_spi_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccwmx53_spi_pads,
					 ARRAY_SIZE(ccwmx53_spi_pads));

#ifdef CONFIG_CCWMX5X_SECOND_TOUCH
	/* Configure external touch interrupt line */
	gpio_request(SECOND_TS_IRQ_PIN, "ts2_irq");
	gpio_direction_input(SECOND_TS_IRQ_PIN);

	/* Configure the Slave Select signal as gpio */
	gpio_request(SECOND_TS_SPI_SS_PIN, "ts2_spi_ss");
	gpio_direction_output(SECOND_TS_SPI_SS_PIN, 1);
	gpio_set_value(SECOND_TS_SPI_SS_PIN, 1);
#endif
}

void ccwmx53_gpio_spi_chipselect_active(int busnum, int ssb_pol, int chipselect)
{
	u8 mask = 0x1 << (chipselect - 1);
	/* Deassert/Assert the different CS lines for the different buses */
	switch (busnum) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			break;
#ifdef CONFIG_CCWMX5X_SECOND_TOUCH
		case 0x4:
			gpio_set_value(SECOND_TS_SPI_SS_PIN, 0);
			break;
#endif
		default:
			break;
		}
		break;
	case 2:
	case 3:
	default:
		break;
	}
}
EXPORT_SYMBOL(ccwmx53_gpio_spi_chipselect_active);

void ccwmx53_gpio_spi_chipselect_inactive(int busnum, int ssb_pol,
					  int chipselect)
{
	u8 mask = 0x1 << (chipselect - 1);
	switch (busnum) {
	case 1:
		switch (chipselect) {
		case 0x1:
			break;
		case 0x2:
			break;
#ifdef CONFIG_CCWMX5X_SECOND_TOUCH
		case 0x4:
			gpio_set_value(SECOND_TS_SPI_SS_PIN, 1);
			break;
#endif
		default:
			break;
		}
		break;
	case 2:
	case 3:
	default:
		break;
        }
}
EXPORT_SYMBOL(ccwmx53_gpio_spi_chipselect_inactive);
#endif
