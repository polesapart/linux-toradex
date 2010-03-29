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

#ifndef __ASM_ARCH_MX51_BOARD_H__
#define __ASM_ARCH_MX51_BOARD_H__

#if defined(CONFIG_MACH_MX51_CCWMX51JS)
#include "board-mx51_ccwmx51js.h"
#elif defined(CONFIG_MACH_MX51_3DS)
#include "board-mx51_3stack.h"
#elif defined(CONFIG_MACH_MX51_BABBAGE)
#include "board-mx51_babbage.h"
#endif

#endif	/* __ASM_ARCH_MXC_BOARD_H__ */
