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

#ifndef __ASM_ARCH_MXC_BOARD_H__
#define __ASM_ARCH_MXC_BOARD_H__

#if defined(CONFIG_MACH_CCWMX51JS) || defined(CONFIG_MACH_CCMX51JS) || \
    defined(CONFIG_MACH_CCWMX51) || defined(CONFIG_MACH_CCMX51)
#include "board-ccwmx51.h"
#elif defined(MACH_MX51_3DS)
#include "board-3stack.h"
#elif defined(MACH_MX51_BABBAGE)
#include "board-mx51_babbage.h"
#endif

#endif	/* __ASM_ARCH_MXC_BOARD_H__ */
