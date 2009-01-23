/*
 * arch/arm/mach-ns9xxx/include/mach/regs-sys-ns921x.h
 *
 * Copyright (C) 2007,2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef __ASM_ARCH_REGSSYSNS921X_H
#define __ASM_ARCH_REGSSYSNS921X_H

#include <mach/hardware.h>
#include <mach/regs-sys-common.h>

/* System Control Module */

/* Timer Master Control */
#define SYS_TMC		__REG(0xa0900024)

/* Timer x Reload Count and Compare register */
#define SYS_TRCC(x)	__REG2(0xa0900028, (x))

/* Timer x Read and Capture register */
#define SYS_TRC(x)	__REG2(0xa0900050, (x))

/* Interrupt Vector Address Register Level x */
#define SYS_IVA(x)	__REG2(0xa09000c4, (x))

/* Clock Configuration register */
#define SYS_CLOCK	__REG(0xa090017c)
#define SYS_CLOCK_CSC		__REGBITS(31, 29)
#define SYS_CLOCK_CSSEL		__REGBIT(25)
#define SYS_CLOCK_RTC		__REGBIT(12)
#define SYS_CLOCK_I2C		__REGBIT(11)
#define SYS_CLOCK_AES		__REGBIT(9)
#define SYS_CLOCK_ADC		__REGBIT(8)
#define SYS_CLOCK_SPI		__REGBIT(5)
#define SYS_CLOCK_UARTx(i)	__REGBIT(1 + (i))
#define SYS_CLOCK_UARTD		SYS_CLOCK_UARTx(3)
#define SYS_CLOCK_UARTC		SYS_CLOCK_UARTx(2)
#define SYS_CLOCK_UARTB		SYS_CLOCK_UARTx(1)
#define SYS_CLOCK_UARTA		SYS_CLOCK_UARTx(0)
#define SYS_CLOCK_ETH		__REGBIT(0)

#define SYS_RESET	__REG(0xa0900180)

/* PLL Configuration register */
#define SYS_PLL		__REG(0xa0900188)
#define SYS_PLL_NF		__REGBITS(16, 8)
#define SYS_PLL_BP		__REGBIT(7)
#define SYS_PLL_OD		__REGBITS(6, 5)
#define SYS_PLL_NR		__REGBITS(4, 0)

/* Timer x Control register */
#define SYS_TC(x)	__REG2(0xa0900190, (x))
#define SYS_TCx_TE		__REGBIT(15)
#define SYS_TCx_TE_DIS			__REGVAL(SYS_TCx_TE, 0)
#define SYS_TCx_TE_EN			__REGVAL(SYS_TCx_TE, 1)
#define SYS_TCx_DEBUG	       __REGBIT(11)
#define SYS_TCx_DEBUG_CONT		__REGVAL(SYS_TCx_DEBUG, 0)
#define SYS_TCx_DEBUG_STOP		__REGVAL(SYS_TCx_DEBUG, 1)
#define SYS_TCx_INTCLR		__REGBIT(10)
#define SYS_TCx_TCS		__REGBITS(9, 6)
#define SYS_TCx_TCS_2AHB		__REGVAL(SYS_TCx_TCS, 0)
#define SYS_TCx_MODE		__REGBITS(5, 4)
#define SYS_TCx_MODE_INTERNAL		__REGVAL(SYS_TCx_MODE, 0)
#define SYS_TCx_INTSEL		__REGBIT(3)
#define SYS_TCx_INTSEL_DIS		__REGVAL(SYS_TCx_INTSEL, 0)
#define SYS_TCx_INTSEL_EN		__REGVAL(SYS_TCx_INTSEL, 1)
#define SYS_TCx_UPDOWN		__REGBIT(2)
#define SYS_TCx_UPDOWN_UP		__REGVAL(SYS_TCx_UPDOWN, 0)
#define SYS_TCx_UPDOWN_DOWN		__REGVAL(SYS_TCx_UPDOWN, 1)
#define SYS_TCx_BITTIMER	__REGBIT(1)
#define SYS_TCx_BITTIMER_16		__REGVAL(SYS_TCx_BITTIMER, 0)
#define SYS_TCx_BITTIMER_32		__REGVAL(SYS_TCx_BITTIMER, 1)
#define SYS_TCx_RELENBL			__REGBIT(0)
#define SYS_TCx_RELENBL_DIS		__REGVAL(SYS_TCx_RELENBL, 0)
#define SYS_TCx_RELENBL_EN		__REGVAL(SYS_TCx_RELENBL, 1)

#define SYS_RTCMC	__REG(0xa0900224)
#define SYS_RTCMC_SS		__REGBIT(4)
#define SYS_RTCMC_RIS		__REGBIT(3)
#define SYS_RTCMC_MIS		__REGBIT(2)
#define SYS_RTCMC_MODE		__REGBIT(1)
#define SYS_RTCMC_MODE_STANDBY		__REGVAL(SYS_RTCMC_MODE, 0)
#define SYS_RTCMC_MODE_NORMAL		__REGVAL(SYS_RTCMC_MODE, 1)
#define SYS_RTCMC_RIC		__REGBIT(0)

#define SYS_POWER	__REG(0xa0900228)
#define SYS_POWER_SLFRFSH	__REGBIT(21)
#define SYS_POWER_INTCLR	__REGBIT(20)
#define SYS_POWER_EXTIRQx(i)	__REGBIT(16 + (i))
#define SYS_POWER_EXTIRQ3	SYS_POWER_EXTIRQx(3)
#define SYS_POWER_EXTIRQ2	SYS_POWER_EXTIRQx(2)
#define SYS_POWER_EXTIRQ1	SYS_POWER_EXTIRQx(1)
#define SYS_POWER_EXTIRQ0	SYS_POWER_EXTIRQx(0)
#define SYS_POWER_RTC		SYS_CLOCK_RTC
#define SYS_POWER_I2C		SYS_CLOCK_I2C
#define SYS_POWER_SPI		SYS_CLOCK_SPI
#define SYS_POWER_UARTx(i)	SYS_CLOCK_UARTx(i)
#define SYS_POWER_UARTD		SYS_POWER_UARTx(3)
#define SYS_POWER_UARTC		SYS_POWER_UARTx(2)
#define SYS_POWER_UARTB		SYS_POWER_UARTx(1)
#define SYS_POWER_UARTA		SYS_POWER_UARTx(0)
#define SYS_POWER_ETH		SYS_CLOCK_ETH

#endif /* ifndef __ASM_ARCH_REGSSYSNS921X_H */
