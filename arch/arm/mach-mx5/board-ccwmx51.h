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

#include "board-ccwmx5x.h"

/* Second touch interface configuration */
#ifdef CONFIG_CCWMX5X_SECOND_TOUCH
#ifdef CONFIG_JSCCWMX51_V1
/* Settings for the JSCCWMX51 Board RevA, for the DISP0 */
#elif defined(CONFIG_JSCCWMX51_V2)
/* Settings for the JSCCWMX51 Board RevB, for the DISP0/DISP1 */
#endif /* CONFIG_JSCCWMX51_VX */
#endif /* CONFIG_CCWMX5X_SECOND_TOUCH */


/* Set Base board revision */
#ifdef CONFIG_JSCCWMX51_V1
/* Board revision and mach name postfix */
#define BASE_BOARD_REV		1
#define BOARD_NAME		" on a EAK board"
/* SD1 card detect irq */
#define CCWMX51_SD1_CD_IRQ	IOMUX_TO_IRQ(MX51_PIN_GPIO1_0)
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_4	/* AD9389 interrupt */

/* Second touch settings */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_DI1_D1_CS
#elif defined(CONFIG_JSCCWMX51_V2)
/* Board revision */
#define BASE_BOARD_REV		2
#define BOARD_NAME		" on a JSK board"
/* SD1 card detect irq, not present CD line... */
#define CCWMX51_SD1_CD_IRQ	0
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_0	/* AD9389 interrupt */
/* Second touch settings */
#define SECOND_TS_IRQ_PIN	MX51_PIN_DI1_D0_CS
#define SECOND_TS_SPI_SS_PIN	MX51_PIN_CSPI1_RDY
#else
#define BASE_BOARD_REV		0
#define BOARD_NAME		" on a custom board"
#define CCWMX51_SD1_CD_IRQ	0	/* Customize this value to support a CD irq on the SD1 */
/* #define AD9389_GPIO_IRQ */
#endif

/* framebuffer settings */
#if defined(CONFIG_CCXMX5X_DISP0) && defined(CONFIG_CCXMX5X_DISP1)
#define FB_MEM_SIZE		SZ_32M
#else
#define FB_MEM_SIZE		SZ_16M
#endif

#if defined(CONFIG_MXC_AMD_GPU) || defined(CONFIG_MXC_AMD_GPU_MODULE)
#define GPU_MEM_SIZE		SZ_64M
#else
#define GPU_MEM_SIZE		0
#endif


void ccwmx51_2nd_touch_gpio_init(void);
void ccwmx51_init_2nd_touch(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX51_H__ */
