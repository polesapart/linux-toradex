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
static iomux_v3_cfg_t ccwmx53_uart1_pads[] = {
	MX53_PAD_PATA_DMACK__UART1_RXD_MUX,
	MX53_PAD_PATA_DIOW__UART1_TXD_MUX,
#if defined(CONFIG_UART1_CTS_RTS_ENABLED) || defined(CONFIG_UART1_FULL_UART_ENABLED)
#endif /* (CONFIG_UART1_CTS_RTS_ENABLED) || (CONFIG_UART1_FULL_UART_ENABLED) */
#ifdef CONFIG_UART1_FULL_UART_ENABLED
#endif /* CONFIG_UART1_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART1_ENABLED */

#ifdef CONFIG_UART2_ENABLED
static iomux_v3_cfg_t ccwmx53_uart2_pads[] = {
#if defined(CONFIG_UART2_CTS_RTS_ENABLED) || defined(CONFIG_UART2_FULL_UART_ENABLED)
#endif /* (CONFIG_UART2_CTS_RTS_ENABLED) || (CONFIG_UART2_FULL_UART_ENABLED) */
#ifdef CONFIG_UART2_FULL_UART_ENABLED
#endif /* CONFIG_UART2_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART2_ENABLED */

#ifdef CONFIG_UART3_ENABLED
static iomux_v3_cfg_t ccwmx53_uart3_pads[] = {
#if defined(CONFIG_UART3_CTS_RTS_ENABLED) || defined(CONFIG_UART3_FULL_UART_ENABLED)
#endif /* (CONFIG_UART3_CTS_RTS_ENABLED) || (CONFIG_UART3_FULL_UART_ENABLED) */
#ifdef CONFIG_UART3_FULL_UART_ENABLED
#endif /* CONFIG_UART3_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART3_ENABLED */

#ifdef CONFIG_UART4_ENABLED
static iomux_v3_cfg_t ccwmx53_uart4_pads[] = {
#if defined(CONFIG_UART4_CTS_RTS_ENABLED) || defined(CONFIG_UART4_FULL_UART_ENABLED)
#endif /* (CONFIG_UART4_CTS_RTS_ENABLED) || (CONFIG_UART4_FULL_UART_ENABLED) */
#ifdef CONFIG_UART4_FULL_UART_ENABLED
#endif /* CONFIG_UART4_FULL_UART_ENABLED */
};
#endif /* CONFIG_UART4_ENABLED */

#ifdef CONFIG_UART5_ENABLED
static iomux_v3_cfg_t ccwmx53_uart5_pads[] = {
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


#if defined(ESDHCI_MXC_SELECT1)
static iomux_v3_cfg_t ccwmx53_mmc1_pads[] = {
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
#if defined(ESDHCI_MXC_SELECT1_8BIT_PORT) || defined(ESDHCI_MXC_SELECT1_4BIT_PORT)
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
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
static iomux_v3_cfg_t ccwmx53_mmc2_pads[] = {
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
static iomux_v3_cfg_t ccwmx53_mmc3_pads[] = {
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
#if defined(ESDHCI_MXC_SELECT1)
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

#if defined(CONFIG_MACH_CCWMX53JS) || defined(CONFIG_MACH_CCWMX53JS)
static iomux_v3_cfg_t ccwmx53_wireless_pads[] = {
	MX53_PAD_SD1_CLK__ESDHC1_CLK,
	MX53_PAD_SD1_CMD__ESDHC1_CMD,
	MX53_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX53_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX53_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX53_PAD_SD1_DATA3__ESDHC1_DAT3,
};

void gpio_wireless_active(void)
{
	/* Wireless module is connected to SD1 interface */
	mxc_iomux_v3_setup_multiple_pads(ccwmx53_wireless_pads,
					 ARRAY_SIZE(ccwmx53_wireless_pads));
}
#else
void gpio_wireless_active(int module) {}
#endif


#if defined(CONFIG_CCXMX5X_DISP0)
static iomux_v3_cfg_t ccwmx53_disp0_pads[] = {
	MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
	MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
	MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
	MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
	MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
	MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
	MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
	MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
	MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
	MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
	MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
	MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
	MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
	MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
	MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
	MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
	MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
	MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
#if defined(CCXMX5X_DISP0_RGB888)
	MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
	MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
	MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
	MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
	MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
	MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
#endif
	MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
	MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
	MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
	MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
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

#if defined(CONFIG_FEC) || defined(CONFIG_FEC_MODULE)
static iomux_v3_cfg_t ccwmx53_fec_pads[] = {
	MX53_PAD_FEC_MDC__FEC_MDC,
	MX53_PAD_FEC_MDIO__FEC_MDIO,
	MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
	MX53_PAD_FEC_RX_ER__FEC_RX_ER,
	MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
	MX53_PAD_FEC_RXD1__FEC_RDATA_1,
	MX53_PAD_FEC_RXD0__FEC_RDATA_0,
	MX53_PAD_FEC_TX_EN__FEC_TX_EN,
	MX53_PAD_FEC_TXD1__FEC_TDATA_1,
	MX53_PAD_FEC_TXD0__FEC_TDATA_0,
};

void gpio_fec_active(void)
{
	mxc_iomux_v3_setup_multiple_pads(ccwmx53_fec_pads,
					 ARRAY_SIZE(ccwmx53_fec_pads));
}
#endif

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static iomux_v3_cfg_t ccwmx53_smsc911x_pads[] = {
	MX53_PAD_EIM_EB3__GPIO2_31,
	MX53_PAD_EIM_D16__EMI_WEIM_D_16,
	MX53_PAD_EIM_D17__EMI_WEIM_D_17,
	MX53_PAD_EIM_D18__EMI_WEIM_D_18,
	MX53_PAD_EIM_D19__EMI_WEIM_D_19,
	MX53_PAD_EIM_D20__EMI_WEIM_D_20,
	MX53_PAD_EIM_D21__EMI_WEIM_D_21,
	MX53_PAD_EIM_D22__EMI_WEIM_D_22,
	MX53_PAD_EIM_D23__EMI_WEIM_D_23,
	MX53_PAD_EIM_D24__EMI_WEIM_D_24,
	MX53_PAD_EIM_D25__EMI_WEIM_D_25,
	MX53_PAD_EIM_D26__EMI_WEIM_D_26,
	MX53_PAD_EIM_D27__EMI_WEIM_D_27,
	MX53_PAD_EIM_D28__EMI_WEIM_D_28,
	MX53_PAD_EIM_D29__EMI_WEIM_D_29,
	MX53_PAD_EIM_D30__EMI_WEIM_D_30,
	MX53_PAD_EIM_D31__EMI_WEIM_D_31,
	MX53_PAD_EIM_DA0__EMI_NAND_WEIM_DA_0,
	MX53_PAD_EIM_DA1__EMI_NAND_WEIM_DA_1,
	MX53_PAD_EIM_DA2__EMI_NAND_WEIM_DA_2,
	MX53_PAD_EIM_DA3__EMI_NAND_WEIM_DA_3,
	MX53_PAD_EIM_DA4__EMI_NAND_WEIM_DA_4,
	MX53_PAD_EIM_DA5__EMI_NAND_WEIM_DA_5,
	MX53_PAD_EIM_DA6__EMI_NAND_WEIM_DA_6,
	_MX53_PAD_EIM_OE__EMI_WEIM_OE,
	_MX53_PAD_EIM_RW__EMI_WEIM_RW,
	_MX53_PAD_EIM_CS1__EMI_WEIM_CS_1,
	MX53_PAD_GPIO_12__GPIO4_2,     /* IRQ */
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

static iomux_v3_cfg_t ccwmx53_spi_pads[] = {
		MX53_PAD_CSI0_DAT7__ECSPI1_SS0,
		MX53_PAD_CSI0_DAT4__ECSPI1_SCLK,
		MX53_PAD_CSI0_DAT6__ECSPI1_MISO,
		MX53_PAD_CSI0_DAT5__ECSPI1_MOSI,
		MX53_PAD_GPIO_19__GPIO4_5,
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
