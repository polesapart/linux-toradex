/*
 * arch/arm/mach-ns9xxx/include/mach/regs-sys-common.h
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_REGSSYSCOMMON_H
#define __ASM_ARCH_REGSSYSCOMMON_H
#include <mach/hardware.h>

/* Interrupt Vector Address Register Level x */
#define SYS_IVA(x)	__REG2(0xa09000c4, (x))

/* Interrupt Configuration registers */
#define SYS_IC(x)	__REG2(0xa0900144, (x))
#define __SYS_IC_FIELDNUM(i)		(3 - ((i) & 3))
#define __SYS_IC_SHIFT(i)		(((i) & 3) << 3)
#define SYS_IC_IE(i)		__REGBIT(7 + __SYS_IC_SHIFT(i))
#define SYS_IC_IE_EN(i)		__REGVAL(SYS_IC_IE(i), 1)
#define SYS_IC_IE_DIS(i)	__REGVAL(SYS_IC_IE(i), 0)
#define SYS_IC_ISD(i)		__REGBITS_SHIFT(4, 0, __SYS_IC_SHIFT(i))

/* ISRADDR */
#define SYS_ISRADDR     __REG(0xa0900164)

/* Interrupt Status Active */
#define SYS_ISA		__REG(0xa0900168)

/* Interrupt Status Raw */
#define SYS_ISR		__REG(0xa090016c)

/* Active Interrupt Level ID Status register */
#define SYS_AILID	__REG(0xa090018c)

#define SYS_EIxCTRL(i)	__REG2(0xa0900214, (i))
#define SYS_EIxCTRL_CLEAR	__REGBIT(2)
#define SYS_EIxCTRL_PLTY	__REGBIT(1)
#define SYS_EIxCTRL_PLTY_HIGH		__REGVAL(SYS_EIxCTRL_PLTY, 0)
#define SYS_EIxCTRL_PLTY_LOW		__REGVAL(SYS_EIxCTRL_PLTY, 1)
#define SYS_EIxCTRL_TYPE	__REGBIT(0)
#define SYS_EIxCTRL_TYPE_LEVEL		__REGVAL(SYS_EIxCTRL_TYPE, 0)
#define SYS_EIxCTRL_TYPE_EDGE		__REGVAL(SYS_EIxCTRL_TYPE, 1)

#endif /* ifndef __ASM_ARCH_REGSSYSCOMMON_H */
