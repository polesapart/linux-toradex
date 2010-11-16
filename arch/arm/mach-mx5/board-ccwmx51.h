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
#if defined CONFIG_UART1_IRDA_ENABLED
#define UART1_IR		IRDA
#else
#define UART1_IR		NO_IRDA
#endif
#define UART1_MODE		MODE_DCE
#define UART1_DMA_ENABLED	0


/* UART 2 configuration */
#if defined CONFIG_UART2_ENABLED
#define UART2_ENABLED		1
#else
#define UART2_ENABLED		0
#endif
#if defined CONFIG_UART2_IRDA_ENABLED
#define UART2_IR		IRDA
#else
#define UART2_IR		NO_IRDA
#endif
#define UART2_MODE		MODE_DCE
#define UART2_DMA_ENABLED	0

/* UART 3 configuration */
#if defined CONFIG_UART3_ENABLED
#define UART3_ENABLED		1
#else
#define UART3_ENABLED		0
#endif
#if defined CONFIG_UART3_IRDA_ENABLED
#define UART3_IR		IRDA
#else
#define UART3_IR		NO_IRDA
#endif
#define UART3_MODE		MODE_DCE
#define UART3_DMA_ENABLED	0

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
#elif defined(CONFIG_JSCCWMX51_V2)
/* Settings for the JSCCWMX51 Board RevB, for the DISP0/DISP1 */
#endif /* CONFIG_JSCCWMX51_VX */
#endif /* CONFIG_CCWMX51_SECOND_TOUCH */

/* AD9389 interrupt */
#ifdef CONFIG_JSCCWMX51_V1
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_4
#elif defined(CONFIG_JSCCWMX51_V2)
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_0
#endif


/* Set Base board revision */
#ifdef CONFIG_JSCCWMX51_V1
/* Board revision and mach name postfix */
#define BASE_BOARD_REV		1
#define BOARD_NAME		" on a EAK board"
/* SD1 card detect irq */
#define CCWMX51_SD1_CD_IRQ	IOMUX_TO_IRQ(MX51_PIN_GPIO1_0)
/* Second touch settings */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_DI1_D1_CS
#elif defined(CONFIG_JSCCWMX51_V2)
/* Board revision */
#define BASE_BOARD_REV		2
#define BOARD_NAME		" on a JSK board"
/* SD1 card detect irq, not present CD line... */
#define CCWMX51_SD1_CD_IRQ	0
/* Second touch settings */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_CSPI1_RDY
#else
#define BASE_BOARD_REV		0
#define BOARD_NAME		" on an undefined board"
#endif

/* framebuffer settings */
#if defined(CONFIG_CCWMX51_DISP1) && defined(CONFIG_CCWMX51_DISP2)
#define FB_MEM_SIZE		SZ_16M
#else
#define FB_MEM_SIZE		SZ_8M
#endif

void ccwmx51_2nd_touch_gpio_init(void);
void ccwmx51_init_2nd_touch(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX51_H__ */
