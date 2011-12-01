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
#define AD9389_GPIO_IRQ		MX53_GPIO(5,2)		/* GPIO_5_2 */
#define AD9389_IRQ_PAD		MX53_PAD_EIM_A25__GPIO5_2

/* Set base board type and revision */
#ifdef CONFIG_CERTMX53_V1

#define BASE_BOARD_REV		1
#define BOARD_NAME		" on a CERTMX53 board"

/* base board custom GPIOs */
#define DISP1_ENABLE_PAD	MX53_PAD_NANDF_CS2__GPIO6_15
#define DISP1_ENABLE_GPIO	MX53_GPIO(6, 15)	/* GPIO_6_15 */

#elif defined(CONFIG_JSCCWMX53_V2)

#define BASE_BOARD_REV		2
#define BOARD_NAME		" on a JSK board"

/* base board custom GPIOs */
#define DISP1_ENABLE_PAD	MX53_PAD_DI0_PIN4__GPIO4_20
#define DISP1_ENABLE_GPIO	MX53_GPIO(4, 20)	/* GPIO_4_20 */
#define DISP1_ENABLE_ACT_HIGH	1

#elif defined(CONFIG_JSCCWMX53_CUSTOM)

#define BASE_BOARD_REV		0
#define BOARD_NAME		" on a custom board"

#endif /* CONFIG_base_board */

#define ESDHC3_WP_PAD		MX53_PAD_NANDF_CS1__NANDF_CS1
//#define ESDHC3_CD_PAD

#define USER_LED1_PAD		MX53_PAD_CSI0_DATA_EN__GPIO5_20
#define USER_LED1_GPIO		MX53_GPIO(5, 20)	/* GPIO_5_20 */
#define USER_LED2_PAD		MX53_PAD_GPIO_17__GPIO7_12
#define USER_LED2_GPIO		MX53_GPIO(7, 12)	/* GPIO_7_12 */
#define USER_KEY1_PAD		MX53_PAD_GPIO_10__GPIO4_0
#define USER_KEY1_GPIO		MX53_GPIO(4, 0)	/* GPIO_4_0 */
#define USER_KEY2_PAD		MX53_PAD_GPIO_11__GPIO4_1
#define USER_KEY2_GPIO		MX53_GPIO(4, 1)	/* GPIO_4_1 */

/* Second touch settings */
void ccwmx53_init_2nd_touch(void);
void gpio_wireless_active(void);

#endif		/* __ASM_ARCH_MXC_BOARD_CCWMX53_H__ */
