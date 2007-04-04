/*
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __ARCH_ARM_MACH_MX27_CRM_REGS_H__
#define __ARCH_ARM_MACH_MX27_CRM_REGS_H__

#include <asm/arch/hardware.h>

/* Register offsets */
#define CCM_CSCR                0x0
#define CCM_MPCTL0              0x4
#define CCM_MPCTL1              0x8
#define CCM_SPCTL0              0xC
#define CCM_SPCTL1              0x10
#define CCM_OSC26MCTL           0x14
#define CCM_PCDR0               0x18
#define CCM_PCDR1               0x1c
#define CCM_PCCR0               0x20
#define CCM_PCCR1               0x24
#define CCM_CCSR                0x28
#define CCM_PMCTL               0x2c
#define CCM_PMCOUNT             0x30
#define CCM_WKGDCTL             0x34

#define CCM_CSCR_USB_OFFSET     28
#define CCM_CSCR_USB_MASK       (0x7 << 28)
#define CCM_CSCR_SD_OFFSET      24
#define CCM_CSCR_SD_MASK        (0x3 << 24)
#define CCM_CSCR_SSI2           (1 << 23)
#define CCM_CSCR_SSI2_OFFSET    23
#define CCM_CSCR_SSI1           (1 << 22)
#define CCM_CSCR_SSI1_OFFSET    22
#define CCM_CSCR_VPU           (1 << 21)
#define CCM_CSCR_VPU_OFFSET    21
#define CCM_CSCR_MSHC           (1 << 20)
#define CCM_CSCR_SPLLRES        (1 << 19)
#define CCM_CSCR_MPLLRES        (1 << 18)
#define CCM_CSCR_SP             (1 << 17)
#define CCM_CSCR_MCU            (1 << 16)
#define CCM_CSCR_PRESC_OFFSET   13
#define CCM_CSCR_PRESC_MASK     (0x7 << 13)
#define CCM_CSCR_BCLK_OFFSET    9
#define CCM_CSCR_BCLK_MASK      (0xf << 9)
#define CCM_CSCR_IPDIV_OFFSET   8
#define CCM_CSCR_IPDIV          (1 << 8)
#define CCM_CSCR_OSC26MDIV      (1 << 4)
#define CCM_CSCR_OSC26M         (1 << 3)
#define CCM_CSCR_FPM            (1 << 2)
#define CCM_CSCR_SPEN           (1 << 1)
#define CCM_CSCR_MPEN           1

#define CCM_MPCTL0_CPLM         (1 << 31)
#define CCM_MPCTL0_PD_OFFSET    26
#define CCM_MPCTL0_PD_MASK      (0xf << 26)
#define CCM_MPCTL0_MFD_OFFSET   16
#define CCM_MPCTL0_MFD_MASK     (0x3ff << 16)
#define CCM_MPCTL0_MFI_OFFSET   10
#define CCM_MPCTL0_MFI_MASK     (0xf << 10)
#define CCM_MPCTL0_MFN_OFFSET   0
#define CCM_MPCTL0_MFN_MASK     0x3ff

#define CCM_MPCTL1_LF           (1 << 15)
#define CCM_MPCTL1_BRMO         (1 << 6)

#define CCM_SPCTL0_CPLM         (1 << 31)
#define CCM_SPCTL0_PD_OFFSET    26
#define CCM_SPCTL0_PD_MASK      (0xf << 26)
#define CCM_SPCTL0_MFD_OFFSET   16
#define CCM_SPCTL0_MFD_MASK     (0x3ff << 16)
#define CCM_SPCTL0_MFI_OFFSET   10
#define CCM_SPCTL0_MFI_MASK     (0xf << 10)
#define CCM_SPCTL0_MFN_OFFSET   0
#define CCM_SPCTL0_MFN_MASK     0x3ff

#define CCM_SPCTL1_LF           (1 << 15)
#define CCM_SPCTL1_BRMO         (1 << 6)

#define CCM_OSC26MCTL_PEAK_OFFSET       16
#define CCM_OSC26MCTL_PEAK_MASK         (0x3 << 16)
#define CCM_OSC26MCTL_AGC_OFFSET        8
#define CCM_OSC26MCTL_AGC_MASK          (0x3f << 8)
#define CCM_OSC26MCTL_ANATEST_OFFSET    0
#define CCM_OSC26MCTL_ANATEST_MASK      0x3f

#define CCM_PCDR0_SSI2BAUDDIV_OFFSET    26
#define CCM_PCDR0_SSI2BAUDDIV_MASK      (0x3f << 26)
#define CCM_PCDR0_CLKO_EN               25
#define CCM_PCDR0_CLKODIV_OFFSET        22
#define CCM_PCDR0_CLKODIV_MASK          (0x7 << 22)
#define CCM_PCDR0_SSI1BAUDDIV_OFFSET    16
#define CCM_PCDR0_SSI1BAUDDIV_MASK      (0x3f << 16)
#define CCM_PCDR0_NFCDIV_OFFSET         12
#define CCM_PCDR0_NFCDIV_MASK           (0xf << 12)
#define CCM_PCDR0_VPUDIV_OFFSET        8
#define CCM_PCDR0_VPUDIV_MASK          (0xf << 8)
#define CCM_PCDR0_MSHCDIV_OFFSET        0
#define CCM_PCDR0_MSHCDIV_MASK          0x1f

#define CCM_PCDR1_PERDIV4_OFFSET        24
#define CCM_PCDR1_PERDIV4_MASK          (0x3f << 24)
#define CCM_PCDR1_PERDIV3_OFFSET        16
#define CCM_PCDR1_PERDIV3_MASK          (0x3f << 16)
#define CCM_PCDR1_PERDIV2_OFFSET        8
#define CCM_PCDR1_PERDIV2_MASK          (0x3f << 8)
#define CCM_PCDR1_PERDIV1_OFFSET        0
#define CCM_PCDR1_PERDIV1_MASK          0x3f

#define CCM_PCCR0_CSPI1         (1 << 31)
#define CCM_PCCR0_CSPI2         (1 << 30)
#define CCM_PCCR0_CSPI3         (1 << 29)
#define CCM_PCCR0_DMA           (1 << 28)
#define CCM_PCCR0_EMMA          (1 << 27)
#define CCM_PCCR0_FEC           (1 << 26)
#define CCM_PCCR0_GPIO          (1 << 25)
#define CCM_PCCR0_GPT1          (1 << 24)
#define CCM_PCCR0_GPT2          (1 << 23)
#define CCM_PCCR0_GPT3          (1 << 22)
#define CCM_PCCR0_GPT4          (1 << 21)
#define CCM_PCCR0_GPT5          (1 << 20)
#define CCM_PCCR0_GPT6          (1 << 19)
#define CCM_PCCR0_I2C1          (1 << 18)
#define CCM_PCCR0_I2C2          (1 << 17)
#define CCM_PCCR0_IIM           (1 << 16)
#define CCM_PCCR0_KPP           (1 << 15)
#define CCM_PCCR0_LCDC          (1 << 14)
#define CCM_PCCR0_MSHC          (1 << 13)
#define CCM_PCCR0_OWIRE         (1 << 12)
#define CCM_PCCR0_PWM           (1 << 11)
#define CCM_PCCR0_RTC           (1 << 9)
#define CCM_PCCR0_RTIC          (1 << 8)
#define CCM_PCCR0_SAHARA        (1 << 7)
#define CCM_PCCR0_SCC           (1 << 6)
#define CCM_PCCR0_SDHC1         (1 << 5)
#define CCM_PCCR0_SDHC2         (1 << 4)
#define CCM_PCCR0_SDHC3         (1 << 3)
#define CCM_PCCR0_SLCDC         (1 << 2)
#define CCM_PCCR0_SSI1_IPG      (1 << 1)
#define CCM_PCCR0_SSI2_IPG      (1 << 0)

#define CCM_PCCR1_UART1         (1 << 31)
#define CCM_PCCR1_UART2         (1 << 30)
#define CCM_PCCR1_UART3         (1 << 29)
#define CCM_PCCR1_UART4         (1 << 28)
#define CCM_PCCR1_UART5         (1 << 27)
#define CCM_PCCR1_UART6         (1 << 26)
#define CCM_PCCR1_USBOTG        (1 << 25)
#define CCM_PCCR1_WDT           (1 << 24)
#define CCM_PCCR1_HCLK_ATA      (1 << 23)
#define CCM_PCCR1_HCLK_BROM     (1 << 22)
#define CCM_PCCR1_HCLK_CSI      (1 << 21)
#define CCM_PCCR1_HCLK_DMA      (1 << 20)
#define CCM_PCCR1_HCLK_EMI      (1 << 19)
#define CCM_PCCR1_HCLK_EMMA     (1 << 18)
#define CCM_PCCR1_HCLK_FEC      (1 << 17)
#define CCM_PCCR1_HCLK_VPU     (1 << 16)
#define CCM_PCCR1_HCLK_LCDC     (1 << 15)
#define CCM_PCCR1_HCLK_RTIC     (1 << 14)
#define CCM_PCCR1_HCLK_SAHARA   (1 << 13)
#define CCM_PCCR1_HCLK_SLCDC    (1 << 12)
#define CCM_PCCR1_HCLK_USBOTG   (1 << 11)
#define CCM_PCCR1_PERCLK1       (1 << 10)
#define CCM_PCCR1_PERCLK2       (1 << 9)
#define CCM_PCCR1_PERCLK3       (1 << 8)
#define CCM_PCCR1_PERCLK4       (1 << 7)
#define CCM_PCCR1_VPU_BAUD     (1 << 6)
#define CCM_PCCR1_SSI1_BAUD     (1 << 5)
#define CCM_PCCR1_SSI2_BAUD     (1 << 4)
#define CCM_PCCR1_NFC_BAUD      (1 << 3)
#define CCM_PCCR1_MSHC_BAUD     (1 << 2)

#define CCM_CCSR_32KSR          (1 << 15)
#define CCM_CCSR_CLKMODE1       (1 << 9)
#define CCM_CCSR_CLKMODE0       (1 << 8)
#define CCM_CCSR_CLKOSEL_OFFSET 0
#define CCM_CCSR_CLKOSEL_MASK   0x1f

#define SYS_FMCR                0x14	/*  Functional Muxing Control Reg */

#endif				/* __ARCH_ARM_MACH_MX27_CRM_REGS_H__ */
