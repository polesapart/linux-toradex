/*
 * Copyright 2010 Digi International, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_CCWMX51_H__
#define __ASM_ARCH_MXC_BOARD_CCWMX51_H__

#include <mach/mxc_uart.h>

/* UART 1 configuration */
#if defined CONFIG_UART1_ENABLED
#define UART1_ENABLED		1
#else
#define UART1_ENABLED		0
#endif
#define UART1_MODE		MODE_DCE
#define UART1_IR		NO_IRDA

/* UART 2 configuration */
#if defined CONFIG_UART2_ENABLED
#define UART2_ENABLED		1
#else
#define UART2_ENABLED		0
#endif
#define UART2_MODE		MODE_DCE
#define UART2_IR		NO_IRDA

/* UART 3 configuration */
#if defined CONFIG_UART3_ENABLED
#define UART3_ENABLED		1
#else
#define UART3_ENABLED		0
#endif
#define UART3_MODE		MODE_DCE
#define UART3_IR		NO_IRDA

/*!
 * Specifies if the Irda transmit path is inverting
 */
#define MXC_IRDA_TX_INV	0

#define MXC_LL_UART_PADDR	UART1_BASE_ADDR
#define MXC_LL_UART_VADDR	AIPS1_IO_ADDRESS(UART1_BASE_ADDR)

/* Second touch interface configuration */
#ifdef CONFIG_CCWMX51_SECOND_TOUCH
#ifdef CONFIG_JSCCWMX51_V1
/* Settings for the JSCCWMX51 Board RevA, for the DISP0 */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_DI1_D1_CS
#elif CONFIG_JSCCWMX51_V2
/* Settings for the JSCCWMX51 Board RevB, for the DISP0/DISP1 */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_CSPI1_RDY
#endif /* CONFIG_JSCCWMX51_VX */
#endif /* CONFIG_CCWMX51_SECOND_TOUCH */

/* Set Base board revision */
#ifdef CONFIG_JSCCWMX51_V1
#define BASE_BOARD_REV		1
#elif CONFIG_JSCCWMX51_V2
#define BASE_BOARD_REV		2
#else
#define BASE_BOARD_REV		0
#endif

void ccwmx51_2nd_touch_gpio_init(void);
void ccwmx51_init_2nd_touch(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX51_H__ */
