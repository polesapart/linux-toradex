/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ASM_ARCH_MXC_BOARD_CCWMX53_H__
#define __ASM_ARCH_MXC_BOARD_CCWMX53_H__

#include "board-ccwmx5x.h"

/* AD9389 interrupt */
#ifdef CONFIG_JSCCWMX51_V1
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_4
#elif defined(CONFIG_JSCCWMX51_V2)
#define AD9389_GPIO_IRQ		MX51_PIN_GPIO1_0
#endif

/* Board revision */
#define BASE_BOARD_REV		1
#define BOARD_NAME		" on a JSK board"

#define ESDHC3_WP_PAD		MX53_PAD_NANDF_CS1__NANDF_CS1
//#define ESDHC3_CD_PAD

#define DISP1_ENABLE_PAD	MX53_PAD_DI0_PIN4__GPIO_4_20
#define DISP1_ENABLE_GPIO	(3*32 + 20)

void ccwmx51_2nd_touch_gpio_init(void);
void ccwmx51_init_2nd_touch(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX53_H__ */
