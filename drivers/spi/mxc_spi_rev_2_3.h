/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mxc_spi_rev_2_3.h
 * @brief This header file contains SPI driver low level register definitions.
 *
 * @ingroup SPI
 */

#ifndef __MXC_SPI_REV_2_3_H__
#define __MXC_SPI_REV_2_3_H__

#include <asm/hardware.h>
#include <asm/mach-types.h>

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPICONFIG		0x0C
#define MXC_CSPIINT		0x10
#define MXC_CSPIDMA		0x14
#define MXC_CSPISTAT		0x18
#define MXC_CSPIPERIOD		0x1C
#define MXC_CSPITEST		0x20
#define MXC_CSPIRESET		0x00

#define MXC_CSPICTRL_ENABLE	0x1
#define MXC_CSPICTRL_DISABLE	0x0
#define MXC_CSPICTRL_HT_ENABLE	(1 << 1)
#define MXC_CSPICTRL_HT_DISABLE	0x0
#define MXC_CSPICTRL_XCH	(1 << 2)
#define MXC_CSPICTRL_SMC	(1 << 3)
#define MXC_CSPICTRL_SLAVE	0x0
#define MXC_CSPICTRL_MODEMASK	0xF
#define MXC_CSPICTRL_MODESHIFT	4
#define MXC_CSPICTRL_CSMASK	0x3
#define MXC_CSPICTRL_MAXDATRATE	0xF
#define MXC_CSPICTRL_DATAMASK	0xFF
#define MXC_CSPICTRL_DATASHIFT 	8
#define MXC_CSPICTRL_ADJUST_SHIFT(x)	(x)

#define MXC_SCLKPOLMASK		0xF
#define MXC_LOWPOLSHIFT		4
#define MXC_HIGHPOL		0x0
#define MXC_PHAMASK		0xF
#define MXC_PHASHIFT		0
#define MXC_NOPHA		0x0
#define MXC_SSCTLMASK		0xF
#define MXC_SSCTLSHIFT		8
#define MXC_SSPOLMASK		0xF
#define MXC_SSPOLSHIFT		12
#define MXC_LOWSSPOL		0x0

#define MXC_CSPIINT_TEEN	(1 << 0)
#define MXC_CSPIINT_THEN	(1 << 1)
#define MXC_CSPIINT_TFEN	(1 << 2)
#define MXC_CSPIINT_RREN	(1 << 3)
#define MXC_CSPIINT_RHEN        (1 << 4)
#define MXC_CSPIINT_RFEN        (1 << 5)
#define MXC_CSPIINT_ROEN        (1 << 6)

#define MXC_CSPISTAT_TE		(1 << 0)
#define MXC_CSPISTAT_TH		(1 << 1)
#define MXC_CSPISTAT_TF		(1 << 2)
#define MXC_CSPISTAT_RR		(1 << 3)
#define MXC_CSPISTAT_RH         (1 << 4)
#define MXC_CSPISTAT_RF         (1 << 5)
#define MXC_CSPISTAT_RO         (1 << 6)

#define MXC_CSPIPERIOD_32KHZ	(1 << 15)

#define MXC_CSPITEST_LBC	(1 << 31)
#define MXC_CSPITEST_RXCNT_OFF	(8)
#define MXC_CSPITEST_RXCNT_MASK	(0x7F << 8)
#define MXC_CSPIRESET_START	0

#endif				//__MXC_SPI_REV_2_3_H__
