/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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


/* --------------------------------------------------------------
 * System Control
 * -------------------------------------------------------------- */

#define SYSCTRL_BASE	IO_ADDRESS(SYSCTRL_BASE_ADDR)

#define SYS_CID     (SYSCTRL_BASE + 0x00)  /* The offset of CHIP ID register */
#define SYS_FMCR    (SYSCTRL_BASE + 0x14)  /* Functional Muxing Control Reg */
#define SYS_GPCR    (SYSCTRL_BASE + 0x18)  /* Global Peripheral Control Reg */
#define SYS_WBCR    (SYSCTRL_BASE + 0x1C)  /* Well Bias Control Reg */
#define SYS_DSCR1   (SYSCTRL_BASE + 0x20)  /* Drive Strength Control Reg 1 */
#define SYS_DSCR2   (SYSCTRL_BASE + 0x24)  /* Drive Strength Control Reg 2 */
#define SYS_DSCR3   (SYSCTRL_BASE + 0x28)  /* Drive Strength Control Reg 3 */
#define SYS_DSCR4   (SYSCTRL_BASE + 0x2C)  /* Drive Strength Control Reg 4 */
#define SYS_DSCR5   (SYSCTRL_BASE + 0x30)  /* Drive Strength Control Reg 5 */
#define SYS_DSCR6   (SYSCTRL_BASE + 0x34)  /* Drive Strength Control Reg 6 */
#define SYS_DSCR7   (SYSCTRL_BASE + 0x38)  /* Drive Strength Control Reg 7 */
#define SYS_DSCR8   (SYSCTRL_BASE + 0x3C)  /* Drive Strength Control Reg 8 */
#define SYS_DSCR9   (SYSCTRL_BASE + 0x40)  /* Drive Strength Control Reg 9 */
#define SYS_DSCR10  (SYSCTRL_BASE + 0x44)  /* Drive Strength Control Reg 10 */
#define SYS_DSCR11  (SYSCTRL_BASE + 0x48)  /* Drive Strength Control Reg 11 */
#define SYS_DSCR12  (SYSCTRL_BASE + 0x4C)  /* Drive Strength Control Reg 12 */
#define SYS_DSCR13  (SYSCTRL_BASE + 0x50)  /* Drive Strength Control Reg 13 */
#define SYS_PSCR    (SYSCTRL_BASE + 0x54)  /* Pull Strength Control Reg */
#define SYS_PCSR    (SYSCTRL_BASE + 0x58)  /* Priority Control and Select Reg */
#define SYS_PMCR    (SYSCTRL_BASE + 0x60)  /* Power Management Control Reg */
#define SYS_DCVR0   (SYSCTRL_BASE + 0x64)  /* DPTC Comparator Value Reg 0 */
#define SYS_DCVR1   (SYSCTRL_BASE + 0x68)  /* DPTC Comparator Value Reg 1 */
#define SYS_DCVR2   (SYSCTRL_BASE + 0x6C)  /* DPTC Comparator Value Reg 2 */
#define SYS_DCVR3   (SYSCTRL_BASE + 0x70)  /* DPTC Comparator Value Reg 3 */


/* --------------------------------------------------------------
 * Clock Controller Module
 * -------------------------------------------------------------- */

#define CCM_BASE	IO_ADDRESS(CCM_BASE_ADDR)

/*
 * Register offsets
 */

#define CCM_CSCR     (CCM_BASE + 0x0)   /* Clock Source Control Reg */
#define CCM_MPCTL0   (CCM_BASE + 0x4)   /* MPLL Control Reg 0 */
#define CCM_MPCTL1   (CCM_BASE + 0x8)   /* MPLL Control Reg 1 */
#define CCM_SPCTL0   (CCM_BASE + 0xC)   /* SPLL Control Reg 0 */
#define CCM_SPCTL1   (CCM_BASE + 0x10)  /* SPLL Control Reg 1 */
#define CCM_OSC26MCT (CCM_BASE + 0x14)  /* Oscillator 26M Reg */
#define CCM_PCDR0    (CCM_BASE + 0x18)  /* Peripheral Clock Divider Reg 0 */
#define CCM_PCDR1    (CCM_BASE + 0x1c)  /* Peripheral Clock Divider Reg 1 */
#define CCM_PCCR0    (CCM_BASE + 0x20)  /* Peripheral Clock Control Reg 0 */
#define CCM_PCCR1    (CCM_BASE + 0x24)  /* Peripheral Clock Control Reg 1 */
#define CCM_CCSR     (CCM_BASE + 0x28)  /* Clock Control Status Reg */
#define CCM_WKGDCTL  (CCM_BASE + 0x34)  /* Wakeup Guard Mode Control Reg */

/*
 * Register Bit Offsets
 */

/* CSCR */
#define CCM_CSCR_MPEN_OFFSET            0
#define CCM_CSCR_SPEN_OFFSET            1
#define CCM_CSCR_FPM_OFFSET             2
#define CCM_CSCR_OSC26M_OFFSET          3
#define CCM_CSCR_OSC26MDIV_OFFSET       4
#define CCM_CSCR_IPDIV_OFFSET           8
#define CCM_CSCR_AHB_OFFSET             8  /* Rev 2 Only */
#define CCM_CSCR_BCLK_OFFSET            9
#define CCM_CSCR_ARM_OFFSET             12 /* Rev 2 Only */
#define CCM_CSCR_PRESC_OFFSET           13
#define CCM_CSCR_ARM_SRC_OFFSET         15 /* Rev 2 Only */
#define CCM_CSCR_MCU_OFFSET             16
#define CCM_CSCR_SP_OFFSET              17
#define CCM_CSCR_MPLLRES_OFFSET         18
#define CCM_CSCR_SPLLRES_OFFSET         19
#define CCM_CSCR_MSHC_OFFSET            20
#define CCM_CSCR_VPU_OFFSET             21
#define CCM_CSCR_SSI1_OFFSET            22
#define CCM_CSCR_SSI2_OFFSET            23
#define CCM_CSCR_SD_OFFSET              24
#define CCM_CSCR_USB_OFFSET             28

/* MPCTL0 */
#define CCM_MPCTL0_MFN_OFFSET           0
#define CCM_MPCTL0_MFI_OFFSET           10
#define CCM_MPCTL0_MFD_OFFSET           16
#define CCM_MPCTL0_PD_OFFSET            26
#define CCM_MPCTL0_CPLM_OFFSET          31

/* MPCTL1 */
#define CCM_MPCTL1_BRMO_OFFSET          6
#define CCM_MPCTL1_LF_OFFSET            15

/* SPCTL0 */
#define CCM_SPCTL0_MFN_OFFSET           0
#define CCM_SPCTL0_MFI_OFFSET           10
#define CCM_SPCTL0_MFD_OFFSET           16
#define CCM_SPCTL0_PD_OFFSET            26
#define CCM_SPCTL0_CPLM_OFFSET          31

/* SPCTL1 */
#define CCM_SPCTL1_BRMO_OFFSET          6
#define CCM_SPCTL1_LF_OFFSET            15

/* OSC26MCTL */
#define CCM_OSC26MCTL_ANATEST_OFFSET    0
#define CCM_OSC26MCTL_AGC_OFFSET        8
#define CCM_OSC26MCTL_PEAK_OFFSET       16

/* PCDR0 */
#define CCM_PCDR0_MSHCDIV_OFFSET        0
#define CCM_PCDR0_NFCDIV2_OFFSET        6   /* Rev 2 Only */
#define CCM_PCDR0_VPUDIV_OFFSET         8   /* Rev 1 Only */
#define CCM_PCDR0_VPUDIV2_OFFSET        10  /* Rev 2 Only */
#define CCM_PCDR0_NFCDIV_OFFSET         12  /* Rev 1 Only */
#define CCM_PCDR0_SSI1BAUDDIV_OFFSET    16
#define CCM_PCDR0_CLKODIV_OFFSET        22
#define CCM_PCDR0_CLKO_EN               25
#define CCM_PCDR0_SSI2BAUDDIV_OFFSET    26

/* PCDR1 */
#define CCM_PCDR1_PERDIV1_OFFSET        0
#define CCM_PCDR1_PERDIV2_OFFSET        8
#define CCM_PCDR1_PERDIV3_OFFSET        16
#define CCM_PCDR1_PERDIV4_OFFSET        24

/* PCCR0 */
#define CCM_PCCR0_SSI2_IPG_OFFSET       0
#define CCM_PCCR0_SSI1_IPG_OFFSET       1
#define CCM_PCCR0_SLCDC_OFFSET          2
#define CCM_PCCR0_SDHC3_OFFSET          3
#define CCM_PCCR0_SDHC2_OFFSET          4
#define CCM_PCCR0_SDHC1_OFFSET          5
#define CCM_PCCR0_SCC_OFFSET            6
#define CCM_PCCR0_SAHARA_OFFSET         7
#define CCM_PCCR0_RTIC_OFFSET           8
#define CCM_PCCR0_RTC_OFFSET            9
#define CCM_PCCR0_PWM_OFFSET            11
#define CCM_PCCR0_OWIRE_OFFSET          12
#define CCM_PCCR0_MSHC_OFFSET           13
#define CCM_PCCR0_LCDC_OFFSET           14
#define CCM_PCCR0_KPP_OFFSET            15
#define CCM_PCCR0_IIM_OFFSET            16
#define CCM_PCCR0_I2C2_OFFSET           17
#define CCM_PCCR0_I2C1_OFFSET           18
#define CCM_PCCR0_GPT6_OFFSET           19
#define CCM_PCCR0_GPT5_OFFSET           20
#define CCM_PCCR0_GPT4_OFFSET           21
#define CCM_PCCR0_GPT3_OFFSET           22
#define CCM_PCCR0_GPT2_OFFSET           23
#define CCM_PCCR0_GPT1_OFFSET           24
#define CCM_PCCR0_GPIO_OFFSET           25
#define CCM_PCCR0_FEC_OFFSET            26
#define CCM_PCCR0_EMMA_OFFSET           27
#define CCM_PCCR0_DMA_OFFSET            28
#define CCM_PCCR0_CSPI3_OFFSET          29
#define CCM_PCCR0_CSPI2_OFFSET          30
#define CCM_PCCR0_CSPI1_OFFSET          31

/* PCCR1 */
#define CCM_PCCR1_MSHC_BAUD_OFFSET      2
#define CCM_PCCR1_NFC_BAUD_OFFSET       3
#define CCM_PCCR1_SSI2_BAUD_OFFSET      4
#define CCM_PCCR1_SSI1_BAUD_OFFSET      5
#define CCM_PCCR1_VPU_BAUD_OFFSET       6
#define CCM_PCCR1_PERCLK4_OFFSET        7
#define CCM_PCCR1_PERCLK3_OFFSET        8
#define CCM_PCCR1_PERCLK2_OFFSET        9
#define CCM_PCCR1_PERCLK1_OFFSET        10
#define CCM_PCCR1_HCLK_USBOTG_OFFSET    11
#define CCM_PCCR1_HCLK_SLCDC_OFFSET     12
#define CCM_PCCR1_HCLK_SAHARA_OFFSET    13
#define CCM_PCCR1_HCLK_RTIC_OFFSET      14
#define CCM_PCCR1_HCLK_LCDC_OFFSET      15
#define CCM_PCCR1_HCLK_VPU_OFFSET       16
#define CCM_PCCR1_HCLK_FEC_OFFSET       17
#define CCM_PCCR1_HCLK_EMMA_OFFSET      18
#define CCM_PCCR1_HCLK_EMI_OFFSET       19
#define CCM_PCCR1_HCLK_DMA_OFFSET       20
#define CCM_PCCR1_HCLK_CSI_OFFSET       21
#define CCM_PCCR1_HCLK_BROM_OFFSET      22
#define CCM_PCCR1_HCLK_ATA_OFFSET       23
#define CCM_PCCR1_WDT_OFFSET            24
#define CCM_PCCR1_USBOTG_OFFSET         25
#define CCM_PCCR1_UART6_OFFSET          26
#define CCM_PCCR1_UART5_OFFSET          27
#define CCM_PCCR1_UART4_OFFSET          28
#define CCM_PCCR1_UART3_OFFSET          29
#define CCM_PCCR1_UART2_OFFSET          30
#define CCM_PCCR1_UART1_OFFSET          31

/* CCSR */
#define CCM_CCSR_CLKOSEL_OFFSET         0
#define CCM_CCSR_CLKMODE0_OFFSET        8
#define CCM_CCSR_CLKMODE1_OFFSET        9
#define CCM_CCSR_32KSR_OFFSET           15


/*
 * Register Bit Field Masks
 */

/* CSCR */
#define CCM_CSCR_MPEN              (0x1 << CCM_CSCR_MPEN_OFFSET)
#define CCM_CSCR_SPEN              (0x1 << CCM_CSCR_SPEN_OFFSET)
#define CCM_CSCR_FPM               (0x1 << CCM_SCSR_FPM_OFFSET)
#define CCM_CSCR_OSC26M            (0x1 << CCM_CSCR_OSC26M_OFFSET)
#define CCM_CSCR_OSC26MDIV         (0x1 << CCM_CSCR_OSC26MDIV_OFFSET)
#define CCM_CSCR_IPDIV             (0x1 << CCM_CSCR_IPDIV_OFFSET)
#define CCM_CSCR_AHB_MASK          (0x3 << CCM_CSCR_AHB_OFFSET)     /* Rev 2 */
#define CCM_CSCR_BCLK_MASK         (0xf << CCM_CSCR_BCLK_OFFSET)
#define CCM_CSCR_ARM_MASK          (0x3 << CCM_CSCR_ARM_OFFSET)     /* Rev 2 */
#define CCM_CSCR_PRESC_MASK        (0x7 << CCM_CSCR_PRESC_OFFSET)
#define CCM_CSCR_ARM_SRC           (0x1 << CCM_CSCR_ARM_SRC_OFFSET) /* Rev 2 */
#define CCM_CSCR_MCU               (0x1 << CCM_CSCR_MCU_OFFSET)
#define CCM_CSCR_SP                (0x1 << CCM_CSCR_SP_OFFSET)
#define CCM_CSCR_MPLLRES           (0x1 << CCM_CSCR_MPLLRES_OFFSET)
#define CCM_CSCR_SPLLRES           (0x1 << CCM_CSCR_SPLLRES_OFFSET)
#define CCM_CSCR_MSHC              (0x1 << CCM_CSCR_MSHC_OFFSET)
#define CCM_CSCR_VPU               (0x1 << CCM_CSCR_VPU_OFFSET)
#define CCM_CSCR_SSI1              (0x1 << CCM_CSCR_SSI1_OFFSET)
#define CCM_CSCR_SSI2              (0x1 << CCM_CSCR_SSI2_OFFSET)
#define CCM_CSCR_SD_MASK           (0x3 << CCM_CSCR_SD_OFFSET)
#define CCM_CSCR_USB_MASK          (0x7 << CCM_CSCR_USB_OFFSET)

/* MPCTL0 */
#define CCM_MPCTL0_MFN_MASK        (0x3ff << CCM_MPCTL0_MFN_OFFSET)
#define CCM_MPCTL0_MFI_MASK        (0x00f << CCM_MPCTL0_MFI_OFFSET)
#define CCM_MPCTL0_MFD_MASK        (0x3ff << CCM_MPCTL0_MFD_OFFSET)
#define CCM_MPCTL0_PD_MASK         (0x00f << CCM_MPCTL0_PD_OFFSET)
#define CCM_MPCTL0_CPLM            (0x001 << CCM_MPCTL0_CPLM_OFFSET)

/* MPCTL1 */
#define CCM_MPCTL1_BRMO            (0x1 << CCM_MPCTL1_BRMO_OFFSET)
#define CCM_MPCTL1_LF              (0x1 << CCM_MPCTL1_LF_OFFSET)

/* SPCTL0 */
#define CCM_SPCTL0_MFN_MASK        (0x3ff << CCM_SPCTL0_MFN_OFFSET)
#define CCM_SPCTL0_MFI_MASK        (0x00f << CCM_SPCTL0_MFI_OFFSET)
#define CCM_SPCTL0_MFD_MASK        (0x3ff << CCM_SPCTL0_MFD_OFFSET)
#define CCM_SPCTL0_PD_MASK         (0x00f << CCM_SPCTL0_PD_OFFSET)
#define CCM_SPCTL0_CPLM            (0x001 << CCM_SPCTL0_CPLM_OFFSET)

/* SPCTL1 */
#define CCM_SPCTL1_BRMO            (0x1 << CCM_SPCTL0_BRMO_OFFSET)
#define CCM_SPCTL1_LF              (0x1 << CCM_SPCTL1_LF_OFFSET)

/* OSC26MCTL */
#define CCM_OSC26MCTL_ANATEST_MASK (0x3f << CCM_OSC26MCTL_ANATEST_OFFSET)
#define CCM_OSC26MCTL_AGC_MASK     (0x3f << CCM_OSC26MCTL_AGC_OFFSET)
#define CCM_OSC26MCTL_PEAK_MASK    (0x03 << CCM_OSC26MCTL_PEAK_OFFSET)

/* PCDR0 */
#define CCM_PCDR0_MSHCDIV_MASK     (0x1f << CCM_PCDR0_MSHCDIV_OFFSET)
#define CCM_PCDR0_MSHCDIV2_MASK    (0x3f << CCM_PCDR0_MSHCDIV_OFFSET) /* Rev2 */
#define CCM_PCDR0_NFCDIV2_MASK     (0x0f << CCM_PCDR0_NFCDIV2_OFFSET) /* Rev2 */
#define CCM_PCDR0_VPUDIV_MASK      (0x0f << CCM_PCDR0_VPUDIV_OFFSET)
#define CCM_PCDR0_VPUDIV2_MASK     (0x3f << CCM_PCDR0_VPUDIV2_OFFSET) /* Rev2 */
#define CCM_PCDR0_NFCDIV_MASK      (0x0f << CCM_PCDR0_NFCDIV_OFFSET)
#define CCM_PCDR0_SSI1BAUDDIV_MASK (0x3f << CCM_PCDR0_SSI1BAUDDIV_OFFSET)
#define CCM_PCDR0_CLKODIV_MASK     (0x07 << CCM_PCDR0_CLKODIV_OFFSET)
#define CCM_PCDR0_CLKO_EN_MASK     (0x01 << CCM_PCDR0_CLKO_EN)
#define CCM_PCDR0_SSI2BAUDDIV_MASK (0x3f << CCM_PCDR0_SSI2BAUDDIV_OFFSET)

/* PCDR1 */
#define CCM_PCDR1_PERDIV1_MASK     (0x3f << CCM_PCDR1_PERDIV1_OFFSET)
#define CCM_PCDR1_PERDIV2_MASK     (0x3f << CCM_PCDR1_PERDIV2_OFFSET)
#define CCM_PCDR1_PERDIV3_MASK     (0x3f << CCM_PCDR1_PERDIV3_OFFSET)
#define CCM_PCDR1_PERDIV4_MASK     (0x3f << CCM_PCDR1_PERDIV4_OFFSET)

/* PCCR0 */
#define CCM_PCCR0_SSI2_IPG_MASK    (0x1 << CCM_PCCR0_SSI2_IPG_OFFSET)
#define CCM_PCCR0_SSI1_IPG_MASK    (0x1 << CCM_PCCR0_SSI1_IPG_OFFSET)
#define CCM_PCCR0_SLCDC_MASK       (0x1 << CCM_PCCR0_SLCDC_OFFSET)
#define CCM_PCCR0_SDHC3_MASK       (0x1 << CCM_PCCR0_SDHC3_OFFSET)
#define CCM_PCCR0_SDHC2_MASK       (0x1 << CCM_PCCR0_SDHC2_OFFSET)
#define CCM_PCCR0_SDHC1_MASK       (0x1 << CCM_PCCR0_SDHC1_OFFSET)
#define CCM_PCCR0_SCC_MASK         (0x1 << CCM_PCCR0_SCC_OFFSET)
#define CCM_PCCR0_SAHARA_MASK      (0x1 << CCM_PCCR0_SAHARA_OFFSET)
#define CCM_PCCR0_RTIC_MASK        (0x1 << CCM_PCCR0_RTIC_OFFSET)
#define CCM_PCCR0_RTC_MASK         (0x1 << CCM_PCCR0_RTC_OFFSET)
#define CCM_PCCR0_PWM_MASK         (0x1 << CCM_PCCR0_PWM_OFFSET)
#define CCM_PCCR0_OWIRE_MASK       (0x1 << CCM_PCCR0_OWIRE_OFFSET)
#define CCM_PCCR0_MSHC_MASK        (0x1 << CCM_PCCR0_MSHC_OFFSET)
#define CCM_PCCR0_LCDC_MASK        (0x1 << CCM_PCCR0_LCDC_OFFSET)
#define CCM_PCCR0_KPP_MASK         (0x1 << CCM_PCCR0_KPP_OFFSET)
#define CCM_PCCR0_IIM_MASK         (0x1 << CCM_PCCR0_IIM_OFFSET)
#define CCM_PCCR0_I2C2_MASK        (0x1 << CCM_PCCR0_I2C2_OFFSET)
#define CCM_PCCR0_I2C1_MASK        (0x1 << CCM_PCCR0_I2C1_OFFSET)
#define CCM_PCCR0_GPT6_MASK        (0x1 << CCM_PCCR0_GPT6_OFFSET)
#define CCM_PCCR0_GPT5_MASK        (0x1 << CCM_PCCR0_GPT5_OFFSET)
#define CCM_PCCR0_GPT4_MASK        (0x1 << CCM_PCCR0_GPT4_OFFSET)
#define CCM_PCCR0_GPT3_MASK        (0x1 << CCM_PCCR0_GPT3_OFFSET)
#define CCM_PCCR0_GPT2_MASK        (0x1 << CCM_PCCR0_GPT2_OFFSET)
#define CCM_PCCR0_GPT1_MASK        (0x1 << CCM_PCCR0_GPT1_OFFSET)
#define CCM_PCCR0_GPIO_MASK        (0x1 << CCM_PCCR0_GPIO_OFFSET)
#define CCM_PCCR0_FEC_MASK         (0x1 << CCM_PCCR0_FEC_OFFSET)
#define CCM_PCCR0_EMMA_MASK        (0x1 << CCM_PCCR0_EMMA_OFFSET)
#define CCM_PCCR0_DMA_MASK         (0x1 << CCM_PCCR0_DMA_OFFSET)
#define CCM_PCCR0_CSPI3_MASK       (0x1 << CCM_PCCR0_CSPI3_OFFSET)
#define CCM_PCCR0_CSPI2_MASK       (0x1 << CCM_PCCR0_CSPI2_OFFSET)
#define CCM_PCCR0_CSPI1_MASK       (0x1 << CCM_PCCR0_CSPI1_OFFSET)

/* PCCR1 */
#define CCM_PCCR1_MSHC_BAUD_MASK   (0x1 << CCM_PCCR1_MSHC_BAUD_OFFSET)
#define CCM_PCCR1_NFC_BAUD_MASK    (0x1 << CCM_PCCR1_NFC_BAUD_OFFSET)
#define CCM_PCCR1_SSI2_BAUD_MASK   (0x1 << CCM_PCCR1_SSI2_BAUD_OFFSET)
#define CCM_PCCR1_SSI1_BAUD_MASK   (0x1 << CCM_PCCR1_SSI1_BAUD_OFFSET)
#define CCM_PCCR1_VPU_BAUD_MASK    (0x1 << CCM_PCCR1_VPU_BAUD_OFFSET)
#define CCM_PCCR1_PERCLK4_MASK     (0x1 << CCM_PCCR1_PERCLK4_OFFSET)
#define CCM_PCCR1_PERCLK3_MASK     (0x1 << CCM_PCCR1_PERCLK3_OFFSET)
#define CCM_PCCR1_PERCLK2_MASK     (0x1 << CCM_PCCR1_PERCLK2_OFFSET)
#define CCM_PCCR1_PERCLK1_MASK     (0x1 << CCM_PCCR1_PERCLK1_OFFSET)
#define CCM_PCCR1_HCLK_USBOTG_MASK (0x1 << CCM_PCCR1_HCLK_USBOTG_OFFSET)
#define CCM_PCCR1_HCLK_SLCDC_MASK  (0x1 << CCM_PCCR1_HCLK_SLCDC_OFFSET)
#define CCM_PCCR1_HCLK_SAHARA_MASK (0x1 << CCM_PCCR1_HCLK_SAHARA_OFFSET)
#define CCM_PCCR1_HCLK_RTIC_MASK   (0x1 << CCM_PCCR1_HCLK_RTIC_OFFSET)
#define CCM_PCCR1_HCLK_LCDC_MASK   (0x1 << CCM_PCCR1_HCLK_LCDC_OFFSET)
#define CCM_PCCR1_HCLK_VPU_MASK    (0x1 << CCM_PCCR1_HCLK_VPU_OFFSET)
#define CCM_PCCR1_HCLK_FEC_MASK    (0x1 << CCM_PCCR1_HCLK_FEC_OFFSET)
#define CCM_PCCR1_HCLK_EMMA_MASK   (0x1 << CCM_PCCR1_HCLK_EMMA_OFFSET)
#define CCM_PCCR1_HCLK_EMI_MASK    (0x1 << CCM_PCCR1_HCLK_EMI_OFFSET)
#define CCM_PCCR1_HCLK_DMA_MASK    (0x1 << CCM_PCCR1_HCLK_DMA_OFFSET)
#define CCM_PCCR1_HCLK_CSI_MASK    (0x1 << CCM_PCCR1_HCLK_CSI_OFFSET)
#define CCM_PCCR1_HCLK_BROM_MASK   (0x1 << CCM_PCCR1_HCLK_BROM_OFFSET)
#define CCM_PCCR1_HCLK_ATA_MASK    (0x1 << CCM_PCCR1_HCLK_ATA_OFFSET)
#define CCM_PCCR1_WDT_MASK         (0x1 << CCM_PCCR1_WDT_OFFSET)
#define CCM_PCCR1_USBOTG_MASK      (0x1 << CCM_PCCR1_USBOTG_OFFSET)
#define CCM_PCCR1_UART6_MASK       (0x1 << CCM_PCCR1_UART6_OFFSET)
#define CCM_PCCR1_UART5_MASK       (0x1 << CCM_PCCR1_UART5_OFFSET)
#define CCM_PCCR1_UART4_MASK       (0x1 << CCM_PCCR1_UART4_OFFSET)
#define CCM_PCCR1_UART3_MASK       (0x1 << CCM_PCCR1_UART3_OFFSET)
#define CCM_PCCR1_UART2_MASK       (0x1 << CCM_PCCR1_UART2_OFFSET)
#define CCM_PCCR1_UART1_MASK       (0x1 << CCM_PCCR1_UART1_OFFSET)

/* CCSR */
#define CCM_CCSR_CLKOSEL_MASK      (0x1f << CCM_CCSR_CLKOSEL_OFFSET)
#define CCM_CCSR_CLKMODE0          (0x01 << CCM_CCSR_CLKMODE0_OFFSET)
#define CCM_CCSR_CLKMODE1          (0x01 << CCM_CCSR_CLKMODE1_OFFSET)
#define CCM_CCSR_32KSR             (0x01 << CCM_CCSR_32KSR_OFFSET)

#endif				/* __ARCH_ARM_MACH_MX27_CRM_REGS_H__ */
