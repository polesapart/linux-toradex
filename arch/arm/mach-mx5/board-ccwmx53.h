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
#include "devices_ccwmx53.h"	/* MX53_GPIO(port,pin) */

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

#define DISP1_ENABLE_PAD	MX53_PAD_DI0_PIN4__GPIO4_20
#define DISP1_ENABLE_GPIO	MX53_GPIO(4, 20)	/* GPIO_4_20 */

#define USER_LED1_PAD		MX53_PAD_KEY_COL2__GPIO4_10
#define USER_LED1_GPIO		MX53_GPIO(4, 10)	/* GPIO_4_10 */
#define USER_LED2_PAD		MX53_PAD_KEY_COL3__GPIO4_12
#define USER_LED2_GPIO		MX53_GPIO(4, 12)	/* GPIO_4_12 */
#define USER_KEY1_PAD		MX53_PAD_KEY_ROW2__GPIO4_11
#define USER_KEY1_GPIO		MX53_GPIO(4, 11)	/* GPIO_4_11 */
#define USER_KEY2_PAD		MX53_PAD_KEY_ROW3__GPIO4_13
#define USER_KEY2_GPIO		MX53_GPIO(4, 13)	/* GPIO_4_13 */

/* Second touch settings */
void ccwmx53_init_2nd_touch(void);
void gpio_wireless_active(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX53_H__ */
