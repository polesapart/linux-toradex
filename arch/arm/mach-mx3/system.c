/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#include "crm_regs.h"

/*!
 * @defgroup MSL Machine Specific Layer (MSL)
 */

/*!
 * @defgroup System System-wide Misc Files for MSL
 * @ingroup MSL
 */

/*!
 * @file system.c
 * @brief This file contains idle and reset functions.
 *
 * @ingroup System
 */

/*!
 * This function puts the CPU into idle mode. It is called by default_idle()
 * in process.c file.
 */
void arch_idle(void)
{
	unsigned int reg, v;

	/*
	 * This should do all the clock switching
	 * and wait for interrupt tricks.
	 */
	if ((__raw_readl(AVIC_VECTOR) & MXC_WFI_ENABLE) != 0) {
		/*
		 * workaround ARM11 Platform defect TLSbo64855.
		 * Also see TLSbo78761.
		 */
		reg = __raw_readl(MXC_CCM_PDR0);
		v = (reg & MXC_CCM_PDR0_MAX_PODF_MASK) >>
		    MXC_CCM_PDR0_MAX_PODF_OFFSET;

		/* make sure ARM:AHB clock ratio is 1:1 */
		__raw_writel((reg & ~MXC_CCM_PDR0_MCU_PODF_MASK) | v,
			     MXC_CCM_PDR0);
		__raw_readl(MXC_CCM_PDR0);
		__raw_readl(MXC_CCM_PDR0);

		cpu_do_idle();

		/* restore original clock dividers */
		__raw_writel(reg, MXC_CCM_PDR0);
	}
}

/*
 * This function resets the system. It is called by machine_restart().
 *
 * @param  mode         indicates different kinds of resets
 */
void arch_reset(char mode)
{
	if ((__raw_readw(IO_ADDRESS(WDOG_BASE_ADDR)) & 0x4) != 0) {
		/* If WDOG enabled, wait till it's timed out */
		asm("cpsid iaf");
		while (1) {
		}
	} else {
		__raw_writew(__raw_readw(IO_ADDRESS(WDOG_BASE_ADDR)) | 0x4,
			     IO_ADDRESS(WDOG_BASE_ADDR));
	}
}
