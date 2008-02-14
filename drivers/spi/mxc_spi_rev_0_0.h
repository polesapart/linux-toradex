/*
 * Copyright 2004-2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_spi_rev_0_0.h
 * @brief This header file contains SPI driver low level register definitions.
 *
 * @ingroup SPI
 */

#ifndef __MXC_SPI_REV_0_0_H__
#define __MXC_SPI_REV_0_0_H__

#include <asm/hardware.h>
#include <asm/mach-types.h>

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPICONFIG		0x08
#define MXC_CSPIINT		0x0C
#define MXC_CSPIDMA		0x18
#define MXC_CSPISTAT		0x0C
#define MXC_CSPIPERIOD		0x14
#define MXC_CSPITEST		0x10
#define MXC_CSPIRESET		0x1C

#define MXC_CSPICTRL_ENABLE	(1 << 10)
#define MXC_CSPICTRL_DISABLE	0x0
#define MXC_CSPICTRL_XCH	(1 << 9)
#define MXC_CSPICTRL_SLAVE	0x0
#define MXC_CSPICTRL_MODEMASK	0x1
#define MXC_CSPICTRL_MODESHIFT	11
#define MXC_CSPICTRL_CSMASK	0x3
#define MXC_CSPICTRL_MAXDATRATE	0x10
#define MXC_CSPICTRL_DATAMASK	0x1F
#define MXC_CSPICTRL_DATASHIFT 	14
/* This adjustment in the shift is valid only for even states only(i.e. divide
   ratio of 2). SDHC_SPIEN is not set by default. If SDHC_SPIEN bit is set in
   MXC_CSPICTRL, then divide ratio is 3, this shift adjustment is invalid. */
#define MXC_CSPICTRL_ADJUST_SHIFT(x) ((x) = ((x) - 1) * 2)

#define MXC_SCLKPOLMASK	0x1
#define MXC_LOWPOLSHIFT	5
#define MXC_HIGHPOL	0x0
#define MXC_PHAMASK	0x1
#define MXC_PHASHIFT	6
#define MXC_NOPHA	0x0
#define MXC_SSCTLMASK	0x1
#define MXC_SSCTLSHIFT	7
#define MXC_SSPOLMASK 	0x1
#define MXC_SSPOLSHIFT 	8
#define MXC_LOWSSPOL	0x0

#define MXC_CSPIINT_TEEN	(1 << 9)
#define MXC_CSPIINT_THEN	(1 << 10)
#define MXC_CSPIINT_TFEN	(1 << 11)
#define MXC_CSPIINT_RREN	(1 << 13)
#define MXC_CSPIINT_RHEN        (1 << 14)
#define MXC_CSPIINT_RFEN        (1 << 15)
#define MXC_CSPIINT_ROEN        (1 << 16)

#define MXC_CSPISTAT_TE		(1 << 0)
#define MXC_CSPISTAT_TH		(1 << 1)
#define MXC_CSPISTAT_TF		(1 << 2)
#define MXC_CSPISTAT_RR		(1 << 3)
#define MXC_CSPISTAT_RH         (1 << 4)
#define MXC_CSPISTAT_RF         (1 << 5)
#define MXC_CSPISTAT_RO         (1 << 6)

#define MXC_CSPIPERIOD_32KHZ	(1 << 15)

#define MXC_CSPITEST_LBC	(1 << 14)
#define MXC_CSPITEST_RXCNT_OFF	(4)
#define MXC_CSPITEST_RXCNT_MASK	(0xF << 4)
#define MXC_CSPIRESET_START	1

#endif				//__MXC_SPI_REV_0_0_H__
