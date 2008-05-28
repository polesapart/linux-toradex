/*
 * arch/arm/mach-mx27/board-mx27lite.h
 *
 * MX27LITE Board Configuration Options
 *
 * This file contains all the board level configuration options.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __BOARD_MX27LITE_H__
#define __BOARD_MX27LITE_H__

/*
 * Include Files
 */
#include <asm/arch/mxc_uart.h>

/*
 * MXC UART EVB board level configurations
 *
 * UARTx_MODE:
 * This define specifies if the UART port is configured to be in DTE or
 * DCE mode. There exists a define like this for each UART port. Valid
 * values that can be used are MODE_DTE or MODE_DCE.
 *
 * UARTx_IR:
 * This define specifies if the UART is to be used for IRDA. There exists a
 * define like this for each UART port. Valid values that can be used are
 * IRDA or NO_IRDA.
 *
 * UARTx_ENABLED:
 * This define is used to enable or disable a particular UART port. If
 * disabled, the UART will not be registered in the file system and the user
 * will not be able to access it. There exists a define like this for each UART
 * port. Specify a value of 1 to enable the UART and 0 to disable it.
 *
 */
#define MXC_IRDA_TX_INV    0 /* Select if Irda tx path is inverting */
#define MXC_IRDA_RX_INV    0 /* Select if Irda rx path is inverting */

/* UART 1 configuration */
#define UART1_MODE         MODE_DCE
#define UART1_IR           NO_IRDA
#define UART1_ENABLED      1
/* UART 2 configuration */
#define UART2_MODE         MODE_DCE
#define UART2_IR           NO_IRDA
#define UART2_ENABLED      0
/* UART 3 configuration */
#define UART3_MODE         MODE_DCE
#define UART3_IR           NO_IRDA
#define UART3_ENABLED      0
/* UART 4 configuration */
#define UART4_MODE         MODE_DTE
#define UART4_IR           NO_IRDA
#define UART4_ENABLED      0	/* UART 4 shares pins with ATA */
/* UART 5 configuration */
#define UART5_MODE         MODE_DTE
#define UART5_IR           NO_IRDA
#define UART5_ENABLED      0
/* UART 6 configuration */
#define UART6_MODE         MODE_DTE
#define UART6_IR           NO_IRDA
#define UART6_ENABLED      0

#define MXC_LL_UART_PADDR       UART1_BASE_ADDR
#define MXC_LL_UART_VADDR       AIPI_IO_ADDRESS(UART1_BASE_ADDR)

/*
 * MXC MMC/SD Card board level configurations
 */
#ifndef __ASSEMBLY__
/*
 * Enumerations for SD cards and memory stick card. This corresponds to
 * the card EN bits in the IMR: SD1_EN | MS_EN | SD3_EN | SD2_EN.
 */
enum mxc_card_no {
	MXC_CARD_SD2 = 0,
	MXC_CARD_SD3,
	MXC_CARD_MS,
	MXC_CARD_SD1,
	MXC_CARD_MIN = MXC_CARD_SD2,
	MXC_CARD_MAX = MXC_CARD_SD1,
};
#endif

/*
 * MXC PMIC/Atlas Chip Configuration
 */
#define MXC_PMIC_INT_LINE       IOMUX_TO_IRQ(MX27_PIN_TOUT)

#endif				/* __BOARD_MX27LITE_H__ */
