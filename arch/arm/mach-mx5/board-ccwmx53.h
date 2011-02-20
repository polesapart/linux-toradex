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

/* framebuffer settings */
#if defined(CONFIG_CCWMX51_DISP1) && defined(CONFIG_CCWMX51_DISP2)
#define FB_MEM_SIZE		SZ_32M
#else
#define FB_MEM_SIZE		SZ_16M
#endif

void ccwmx51_2nd_touch_gpio_init(void);
void ccwmx51_init_2nd_touch(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX53_H__ */
