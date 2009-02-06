/* linux/include/asm-arm/arch-s3c2410/regs-cfata.h
 *
 * Copyright (c) 2009 Digi Internationa Inc.
 * http://www.digi.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/


#ifndef ___ASM_ARCH_REGS_CFATA_H
#define ___ASM_ARCH_REGS_CFATA_H

/* CF -ATA controller */
#define S3C2443_SFR_BASE	(0x1800)
#define S3C2443_MUX_REG		(0x1800)
#define S3C2443_PCCARD_BASE	(0x1820)
#define S3C2443_PCCARD_CFG	(0x1820)

#define S3C2443_ATA_BASE	(0x1900)
#define S3C2443_ATA_CONTROL	(0x1900)
#define S3C2443_ATA_STATUS	(0x1904)
#define S3C2443_ATA_COMMAND	(0x1908)
#define S3C2443_ATA_SWRST	(0x190c)
#define S3C2443_ATA_IRQ		(0x1910)
#define S3C2443_ATA_IRQ_MASK	(0x1914)
#define S3C2443_ATA_CFG		(0x1918)

#define S3C2443_ATA_PIO_TIME	(0x192c)
#define S3C2443_ATA_UDMA_TIME	(0x1930)

#define S3C2443_ATA_PIO_DTR	(0x1954)
#define S3C2443_ATA_PIO_FED	(0x1958)
#define S3C2443_ATA_PIO_SCR	(0x195c)
#define S3C2443_ATA_PIO_LLR	(0x1960)
#define S3C2443_ATA_PIO_LMR	(0x1964)
#define S3C2443_ATA_PIO_LHR	(0x1968)
#define S3C2443_ATA_PIO_DVR	(0x196c)
#define S3C2443_ATA_PIO_CSD	(0x1970)
#define S3C2443_ATA_PIO_DAD	(0x1974)
#define S3C2443_ATA_PIO_RDATA	(0x197c)
#define S3C2443_BUS_FIFO_STATUS	(0x1990)
#define S3C2443_ATA_FIFO_STATUS	(0x1994)


#endif /* ___ASM_ARCH_REGS_CFATA_H */
