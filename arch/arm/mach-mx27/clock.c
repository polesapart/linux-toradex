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

/*!
 * @file clock.c
 * @brief API for setting up and retrieving clocks.
 *
 * This file contains API for setting up and retrieving clocks.
 *
 * @ingroup CLOCKS
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/arch/clock.h>
#include "crm_regs.h"
/*!
 * Spinlock to protect CRM register accesses
 */
static DEFINE_SPINLOCK(mxc_crm_lock);

/*!
 * define a variable & macro to control the perclk2.
 */
#define MXC_CLK_PCLK2_CSPI1	0
#define MXC_CLK_PCLK2_CSPI2	1
#define MXC_CLK_PCLK2_CSPI3	2
#define MXC_CLK_PCLK2_SDHC1	3
#define MXC_CLK_PCLK2_SDHC2	4
static int g_perclk2_map = 0;

/*!
 * g_emma_clock_map is defined to control the emma_clock .
 * emma_clock will be disabled until emma_prp_clk and emma_pp_clk are closed.
 */
#define MXC_CLK_EMMA_PRP	0
#define MXC_CLK_EMMA_PP		1
static int g_emma_clock_map = 0;

/*
 * Bit fields
 */
#define CLKSRC_NO_CLK	    	0
#define CLKSRC_PERCLK	    	1
#define CLKSRC_PERCLK_4   	2
#define CLKSRC_EXT_PAD	    	3

/*
 * Get Clock errors
 */
#define ERR_CLK_NO_SUPPORT	1

/*!
 * This function enables the perclk2.
 * @param	source each bit of source indicate the clock status of mdoule
 *		which is using emma clk
 * @return      none.
 */
static void inline __enable_perclk2(unsigned long source)
{
	unsigned long reg;

	BUG_ON(source > 31);
	if (g_perclk2_map == 0) {
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_PERCLK2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
	}
	g_perclk2_map |= (1 << source);
}

/*!
 * This function disables the perclk2.
 * @param	source each bit of source indicate the clock status of mdoule
 *		which is using emma clk
 * @return      none.
 */
static void inline __disable_perclk2(unsigned long source)
{
	unsigned long reg;

	BUG_ON(source > 31);
	g_perclk2_map &= ~(1 << source);
	if (g_perclk2_map == 0) {
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_PERCLK2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
	}
}

/*!
 * This function enables the emma clock.
 * @param	source each bit of source indicate the clock status of mdoule
 *		which is using emma clk
 * @return 	none
 */
static void inline __enable_emma_clk(unsigned long source)
{
	unsigned long reg;

	if (g_emma_clock_map == 0) {
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_EMMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_EMMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
	}
	g_emma_clock_map |= (1 << source);
}

/*!
 * This function disables the emma clock.
 * @param	source each bit of source indicate the clock status of mdoule
 *		which is using emma clk
 * @return 	none
 */
static void inline __disable_emma_clk(unsigned long source)
{
	unsigned long reg;

	g_emma_clock_map &= ~(1 << source);
	if (g_emma_clock_map == 0) {
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_EMMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_EMMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
	}
}

/*
 * This function returns the clock CKIH clock input value.
 */
static unsigned long mxc_clock_ckih(void)
{
	static unsigned long g_clk_ckih = 0;

	if (g_clk_ckih == 0) {
		if ((__raw_readw(PBC_VERSION_REG) & CKIH_27MHZ_BIT_SET) == 0) {
			g_clk_ckih = CKIH_CLK_FREQ_27MHZ;
		} else {
			g_clk_ckih = CKIH_CLK_FREQ;
		}
		pr_info("Clock input source is %ld\n", g_clk_ckih);
	}

	return g_clk_ckih;
}

/*!
 * This function returns the PLL output value in Hz based on pll.
 * @param       pll     PLL as defined in enum plls
 * @return      PLL value in Hz.
 */
unsigned long mxc_pll_clock(enum plls pll)
{
	unsigned long mfi = 0, mfn = 0, mfd = 0, pdf = 0;
	unsigned long ref_clk = 0, prcs = 0;
	unsigned long reg, cscr;
	unsigned long long temp;

	cscr = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
	if (pll == MCUPLL) {
		prcs = (cscr & CCM_CSCR_MCU);
		if (prcs == CCM_CSCR_MCU) {
			ref_clk = mxc_clock_ckih();
		} else {
			ref_clk = CKIL_CLK_FREQ;
		}
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_MPCTL0);
		pdf = (reg & CCM_MPCTL0_PD_MASK) >> CCM_MPCTL0_PD_OFFSET;
		mfd = (reg & CCM_MPCTL0_MFD_MASK) >> CCM_MPCTL0_MFD_OFFSET;
		mfi = (reg & CCM_MPCTL0_MFI_MASK) >> CCM_MPCTL0_MFI_OFFSET;
		mfn = (reg & CCM_MPCTL0_MFN_MASK) >> CCM_MPCTL0_MFN_OFFSET;
	} else if (pll == SERIALPLL) {
		prcs = (cscr & CCM_CSCR_SP);
		if (prcs == CCM_CSCR_SP) {
			ref_clk = mxc_clock_ckih();
		} else {
			ref_clk = CKIL_CLK_FREQ;
		}
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_SPCTL0);
		pdf = (reg & CCM_SPCTL0_PD_MASK) >> CCM_SPCTL0_PD_OFFSET;
		mfd = (reg & CCM_SPCTL0_MFD_MASK) >> CCM_SPCTL0_MFD_OFFSET;
		mfi = (reg & CCM_SPCTL0_MFI_MASK) >> CCM_SPCTL0_MFI_OFFSET;
		mfn = (reg & CCM_SPCTL0_MFN_MASK) >> CCM_SPCTL0_MFN_OFFSET;
	} else {
		printk(KERN_ERR "\nUnknown pll: %d\n\n", pll);
		BUG();		/* oops */
	}

	mfi = (mfi <= 5) ? 5 : mfi;
	temp = (unsigned long long)2 *ref_clk * mfn;
	do_div(temp, mfd + 1);
	temp = (unsigned long long)2 *ref_clk * mfi + temp;
	do_div(temp, pdf + 1);

	return (unsigned long)temp;
}

/*!
 * This function returns the mcu main clock frequency
 *
 * @return      mcu main clock value in Hz.
 */
static unsigned long mxc_mcu_main_clock(void)
{
	return mxc_pll_clock(MCUPLL);
}

/*!
 * This function returns the GPT module clock frequency
 *
 * @param       gpt_ctrl     GPT control register value
 * @param       perclk       PERCLK1 frequency in Hz
 *
 * @return      GPT clock frequency value in Hz.
 */
static unsigned long mxc_get_gpt_clk(unsigned long gpt_ctrl,
				     unsigned long perclk)
{
	unsigned long ret_val = 0;
	int clksrc;

	/*
	 * Three possibilities
	 */
	clksrc = (gpt_ctrl & (~GPT_CTRL_MASK)) >> (GPT_CTRL_OFFSET);
	switch (clksrc) {
	case CLKSRC_PERCLK:
		ret_val = perclk;
		break;
	case CLKSRC_PERCLK_4:
		ret_val = (perclk / 4);
		break;
	case CLKSRC_NO_CLK:
	case CLKSRC_EXT_PAD:
		printk(KERN_ERR
		       "Warning: GPT clock source not supported !!!\n");
		return -ERR_CLK_NO_SUPPORT;
	default:
		ret_val = CKIL_CLK_FREQ;
		break;
	}

	return ret_val;
}

/*!
 * This function returns the main clock values in Hz.
 *
 * @param       clk     as defined in enum mxc_clocks
 *
 * @return      clock value in Hz
 */
unsigned long mxc_get_clocks(enum mxc_clocks clk)
{
	unsigned long pll, spll, ret_val = 0, hclk, gpt_ctrl;
	unsigned long presc_pdf, ipg_pdf, nfc_pdf, usb_pdf;
	unsigned long cscr = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
	unsigned long pcdr0 =
	    __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
	unsigned long pcdr1 =
	    __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);

	unsigned long fclk, ipgclk, perclk1, perclk2, perclk3, perclk4;
	unsigned long bclk_pdf;
	unsigned long perclk1_pdf, perclk2_pdf, perclk3_pdf, perclk4_pdf;
	unsigned long clk_src;
	unsigned long ssi1_pdf = 0;
	unsigned long ssi2_pdf = 0;
	unsigned long vpu_pdf = 0;
	presc_pdf = (cscr & CCM_CSCR_PRESC_MASK) >> CCM_CSCR_PRESC_OFFSET;
	bclk_pdf = (cscr & CCM_CSCR_BCLK_MASK) >> CCM_CSCR_BCLK_OFFSET;
	ipg_pdf = (cscr & CCM_CSCR_IPDIV) >> CCM_CSCR_IPDIV_OFFSET;
	perclk1_pdf =
	    (pcdr1 & CCM_PCDR1_PERDIV1_MASK) >> CCM_PCDR1_PERDIV1_OFFSET;
	perclk2_pdf =
	    (pcdr1 & CCM_PCDR1_PERDIV2_MASK) >> CCM_PCDR1_PERDIV2_OFFSET;
	perclk3_pdf =
	    (pcdr1 & CCM_PCDR1_PERDIV3_MASK) >> CCM_PCDR1_PERDIV3_OFFSET;
	perclk4_pdf =
	    (pcdr1 & CCM_PCDR1_PERDIV4_MASK) >> CCM_PCDR1_PERDIV4_OFFSET;

	pll = mxc_mcu_main_clock();
	spll = mxc_pll_clock(SERIALPLL);
	fclk = pll / (presc_pdf + 1);
	hclk = fclk / (bclk_pdf + 1);
	ipgclk = hclk / (ipg_pdf + 1);
	perclk1 = pll / (perclk1_pdf + 1);
	perclk2 = pll / (perclk2_pdf + 1);
	perclk3 = pll / (perclk3_pdf + 1);
	perclk4 = pll / (perclk4_pdf + 1);
	switch (clk) {
	case CKIL_CLK:
		ret_val = CKIL_CLK_FREQ;
		break;
	case CKIH_CLK:
		ret_val = mxc_clock_ckih();
		break;
	case CPU_CLK:
		ret_val = fclk;
		break;
	case AHB_CLK:
		ret_val = hclk;
		break;
	case PERCLK1:
	case UART6_BAUD:
	case UART5_BAUD:
	case UART4_BAUD:
	case UART3_BAUD:
	case UART2_BAUD:
	case UART1_BAUD:
		ret_val = perclk1;
		break;
	case GPT6_CLK:
		gpt_ctrl = __raw_readl(MXC_GPT_TCTL(6));
		ret_val = mxc_get_gpt_clk(gpt_ctrl, perclk1);
		break;
	case GPT5_CLK:
		gpt_ctrl = __raw_readl(MXC_GPT_TCTL(5));
		ret_val = mxc_get_gpt_clk(gpt_ctrl, perclk1);
		break;
	case GPT4_CLK:
		gpt_ctrl = __raw_readl(MXC_GPT_TCTL(4));
		ret_val = mxc_get_gpt_clk(gpt_ctrl, perclk1);
		break;
	case GPT3_CLK:
		gpt_ctrl = __raw_readl(MXC_GPT_TCTL(3));
		ret_val = mxc_get_gpt_clk(gpt_ctrl, perclk1);
		break;
	case GPT2_CLK:
		gpt_ctrl = __raw_readl(MXC_GPT_TCTL(2));
		ret_val = mxc_get_gpt_clk(gpt_ctrl, perclk1);
		break;
	case GPT1_CLK:
		gpt_ctrl = __raw_readl(MXC_GPT_TCTL(1));
		ret_val = mxc_get_gpt_clk(gpt_ctrl, perclk1);
		break;
	case PWM_CLK:
		ret_val = perclk1;
		break;
	case PERCLK2:
	case SDHC2_CLK:
	case SDHC1_CLK:
	case CSPI3_CLK:
	case CSPI2_CLK:
	case CSPI1_CLK:
		ret_val = perclk2;
		break;
	case PERCLK3:
	case LCDC_CLK:
		ret_val = perclk3;
		break;
	case PERCLK4:
	case CSI_BAUD:
		ret_val = perclk4;
		break;
	case USB_CLK:
		usb_pdf = (cscr & CCM_CSCR_USB_MASK) >> CCM_CSCR_USB_OFFSET;
		ret_val = spll / (usb_pdf + 1);
		break;
	case SSI1_BAUD:
		ssi1_pdf = (pcdr0 & CCM_PCDR0_SSI1BAUDDIV_MASK) >>
		    CCM_PCDR0_SSI1BAUDDIV_OFFSET;
		clk_src = (cscr & CCM_CSCR_SSI1) >> CCM_CSCR_SSI1_OFFSET;
		if (clk_src)
			ret_val = pll / (ssi1_pdf + 1);
		else
			ret_val = spll / (ssi1_pdf + 1);
		break;
	case SSI2_BAUD:
		ssi1_pdf = (pcdr0 & CCM_PCDR0_SSI2BAUDDIV_MASK) >>
		    CCM_PCDR0_SSI2BAUDDIV_OFFSET;
		clk_src = (cscr & CCM_CSCR_SSI2) >> CCM_CSCR_SSI2_OFFSET;
		if (clk_src)
			ret_val = pll / (ssi2_pdf + 1);
		else
			ret_val = spll / (ssi2_pdf + 1);
		break;
	case NFC_CLK:
		nfc_pdf = (pcdr0 & CCM_PCDR0_NFCDIV_MASK) >>
		    CCM_PCDR0_NFCDIV_OFFSET;
		ret_val = hclk / (nfc_pdf + 1);
		break;
	case VPU_BAUD:
		vpu_pdf = (pcdr0 & CCM_PCDR0_VPUDIV_MASK) >>
		    CCM_PCDR0_VPUDIV_OFFSET;
		clk_src = (cscr & CCM_CSCR_VPU) >> CCM_CSCR_VPU_OFFSET;
		if (clk_src)
			ret_val = pll / (vpu_pdf + 1);
		else
			ret_val = spll / (vpu_pdf + 1);
		break;
	case OWIRE_CLK:
	default:
		ret_val = ipgclk;
		break;
	}
	return ret_val;
}

/*!
 * This function returns the parent clock values in Hz.
 *
 * @param       clk     as defined in enum mxc_clocks
 *
 * @return      clock value in Hz
 */
unsigned long mxc_get_clocks_parent(enum mxc_clocks clk)
{
	unsigned long ret_val = 0;

	switch (clk) {
	case CSI_BAUD:
		ret_val = mxc_mcu_main_clock();
		break;
	default:
		break;
	}
	return ret_val;
}

/*!
 * This function sets the PLL source for a clock.
 *
 * @param clk     as defined in enum mxc_clocks
 * @param pll_num the PLL that you wish to use as source for this clock
 */
void mxc_set_clocks_pll(enum mxc_clocks clk, enum plls pll_num)
{
	unsigned long cscr;
	unsigned long flags;

	spin_lock_irqsave(&mxc_crm_lock, flags);
	cscr = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);

	switch (clk) {
	case SSI1_BAUD:
		cscr = (cscr & (~CCM_CSCR_SSI1)) |
		    (pll_num << CCM_CSCR_SSI1_OFFSET);
		break;
	case SSI2_BAUD:
		cscr = (cscr & (~CCM_CSCR_SSI2)) |
		    (pll_num << CCM_CSCR_SSI2_OFFSET);
		break;
	case VPU_BAUD:
		cscr = (cscr & (~CCM_CSCR_VPU)) |
		    (pll_num << CCM_CSCR_VPU_OFFSET);
		break;
	default:
		printk(KERN_ERR
		       "This clock does not have ability to choose its clock source\n");
		break;
	}
	__raw_writel(cscr, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
	return;
}

/*!
 * This function sets the divider value for a clock.
 *
 * @param clk as defined in enum mxc_clocks
 * @param div the division factor to be used for the clock (For SSI & CSI, pass
 *            in 2 times the expected division value to account for FP vals on certain
 *            platforms)
 */
void mxc_set_clocks_div(enum mxc_clocks clk, unsigned int div)
{
	unsigned long reg;
	unsigned long flags;

	spin_lock_irqsave(&mxc_crm_lock, flags);

	switch (clk) {
	case SSI2_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		reg = (reg & (~CCM_PCDR0_SSI2BAUDDIV_MASK)) |
		    ((div - 1) << CCM_PCDR0_SSI2BAUDDIV_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		break;
	case SSI1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		reg = (reg & (~CCM_PCDR0_SSI1BAUDDIV_MASK)) |
		    ((div - 1) << CCM_PCDR0_SSI1BAUDDIV_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		break;
	case NFC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		reg = (reg & (~CCM_PCDR0_NFCDIV_MASK)) |
		    ((div - 1) << CCM_PCDR0_NFCDIV_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		break;
	case VPU_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		reg = (reg & (~CCM_PCDR0_VPUDIV_MASK)) |
		    ((div - 1) << CCM_PCDR0_VPUDIV_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		break;
	case PERCLK1:
	case UART6_BAUD:
	case UART5_BAUD:
	case UART4_BAUD:
	case UART3_BAUD:
	case UART2_BAUD:
	case UART1_BAUD:
	case GPT6_CLK:
	case GPT5_CLK:
	case GPT4_CLK:
	case GPT3_CLK:
	case GPT2_CLK:
	case GPT1_CLK:
	case PWM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		reg = (reg & (~CCM_PCDR1_PERDIV1_MASK)) |
		    ((div - 1) << CCM_PCDR1_PERDIV1_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		break;
	case PERCLK2:
	case SDHC2_CLK:
	case SDHC1_CLK:
	case CSPI3_CLK:
	case CSPI2_CLK:
	case CSPI1_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		reg = (reg & (~CCM_PCDR1_PERDIV2_MASK)) |
		    ((div - 1) << CCM_PCDR1_PERDIV2_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		break;
	case PERCLK3:
	case LCDC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		reg = (reg & (~CCM_PCDR1_PERDIV3_MASK)) |
		    ((div - 1) << CCM_PCDR1_PERDIV3_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		break;
	case PERCLK4:
	case CSI_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		reg = (reg & (~CCM_PCDR1_PERDIV4_MASK)) |
		    ((div - 1) << CCM_PCDR1_PERDIV4_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR1);
		break;
	case USB_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		reg = (reg & (~CCM_CSCR_USB_MASK)) |
		    ((div - 1) << CCM_CSCR_USB_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		break;
	case IPG_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		reg = (reg & (~CCM_CSCR_IPDIV)) |
		    ((div - 1) << CCM_CSCR_IPDIV_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		break;
	case CPU_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		reg = (reg & (~CCM_CSCR_PRESC_MASK)) |
		    ((div - 1) << CCM_CSCR_PRESC_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		break;
	case AHB_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		reg = (reg & (~CCM_CSCR_BCLK_MASK)) |
		    ((div - 1) << CCM_CSCR_BCLK_OFFSET);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		break;
	default:
		break;
	}

	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * This function is called to enable the individual module clocks
 *
 * @param       clk     as defined in enum mxc_clocks
 */
void mxc_clks_enable(enum mxc_clocks clk)
{
	unsigned long flags;
	unsigned long reg;

	spin_lock_irqsave(&mxc_crm_lock, flags);
	switch (clk) {
	case CSI_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_PERCLK4;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_CSI;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case DMA_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_DMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_DMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case RTIC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_RTIC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_RTIC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case BROM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_BROM;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case EMMA_PRP_CLK:
		__enable_emma_clk(MXC_CLK_EMMA_PRP);
		break;
	case EMMA_PP_CLK:
		__enable_emma_clk(MXC_CLK_EMMA_PP);
		break;
	case LCDC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_PERCLK3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_LCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_LCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case SLCDC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SLCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_SLCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case USB_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_USBOTG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_USBOTG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case FEC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_FEC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_FEC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case EMI_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_EMI;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case ATA_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_ATA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case VPU_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_VPU_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_VPU;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case SAHARA2_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SAHARA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_HCLK_SAHARA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case SSI1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_SSI1_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SSI1_IPG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SSI2_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_SSI2_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SSI2_IPG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case NFC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_NFC_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case MSTICK1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_MSHC_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_MSHC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case UART1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_UART1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART2_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_UART2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART3_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_UART3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART4_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_UART4;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART5_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_UART5;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART6_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_UART6;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case WDOG_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg |= CCM_PCCR1_WDT;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;

	case CSPI3_CLK:
		__enable_perclk2(MXC_CLK_PCLK2_CSPI3);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_CSPI3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case CSPI2_CLK:
		__enable_perclk2(MXC_CLK_PCLK2_CSPI2);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_CSPI2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case CSPI1_CLK:
		__enable_perclk2(MXC_CLK_PCLK2_CSPI1);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_CSPI1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPIO_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPIO;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT3_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPT3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT2_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPT2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT1_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPT1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT4_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPT4;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT5_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPT5;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT6_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_GPT6;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case I2C1_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_I2C1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case I2C2_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_I2C2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case IIM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_IIM;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case KPP_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_KPP;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case OWIRE_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_OWIRE;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case PWM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_PWM;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case RTC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_RTC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SCC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SCC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SDHC2_CLK:
		__enable_perclk2(MXC_CLK_PCLK2_SDHC2);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SDHC2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SDHC1_CLK:
		__enable_perclk2(MXC_CLK_PCLK2_SDHC1);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SDHC1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SDHC3_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg |= CCM_PCCR0_SDHC3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;

	default:
		break;
	}
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * This function is called to disable the individual module clocks
 *
 * @param       clk     as defined in enum mxc_clocks
 */
void mxc_clks_disable(enum mxc_clocks clk)
{
	unsigned long flags;
	unsigned long reg;

	spin_lock_irqsave(&mxc_crm_lock, flags);
	switch (clk) {
	case CSI_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_CSI;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_PERCLK4;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case DMA_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_DMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_DMA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case RTIC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_RTIC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_RTIC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case BROM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_BROM;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case EMMA_PRP_CLK:
		__disable_emma_clk(MXC_CLK_EMMA_PRP);
		break;
	case EMMA_PP_CLK:
		__disable_emma_clk(MXC_CLK_EMMA_PP);
		break;
		break;
	case LCDC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_LCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_LCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_PERCLK3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case SLCDC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SLCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_SLCDC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case USB_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_USBOTG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_USBOTG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case FEC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_FEC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_FEC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case EMI_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_EMI;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case ATA_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_ATA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case VPU_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_VPU_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_VPU;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case SAHARA2_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SAHARA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_HCLK_SAHARA;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case SSI1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_SSI1_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SSI1_IPG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SSI2_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_SSI2_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SSI2_IPG;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case NFC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_NFC_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case MSTICK1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_MSHC_BAUD;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_MSHC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case UART1_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_UART1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART2_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_UART2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART3_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_UART3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART4_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_UART4;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART5_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_UART5;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case UART6_BAUD:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_UART6;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case WDOG_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		reg &= ~CCM_PCCR1_WDT;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR1);
		break;
	case CSPI3_CLK:
		__disable_perclk2(MXC_CLK_PCLK2_CSPI3);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_CSPI3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case CSPI2_CLK:
		__disable_perclk2(MXC_CLK_PCLK2_CSPI2);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_CSPI2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case CSPI1_CLK:
		__disable_perclk2(MXC_CLK_PCLK2_CSPI1);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_CSPI1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPIO_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPIO;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT3_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPT3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT2_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPT2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT1_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPT1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT4_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPT4;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT5_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPT5;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case GPT6_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_GPT6;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case I2C1_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_I2C1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case I2C2_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_I2C2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case IIM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_IIM;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case KPP_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_KPP;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case OWIRE_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_OWIRE;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case PWM_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_PWM;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case RTC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_RTC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SCC_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SCC;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SDHC2_CLK:
		__disable_perclk2(MXC_CLK_PCLK2_SDHC2);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SDHC2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SDHC1_CLK:
		__disable_perclk2(MXC_CLK_PCLK2_SDHC1);
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SDHC1;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	case SDHC3_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		reg &= ~CCM_PCCR0_SDHC3;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCCR0);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * This function is used to modify PLL registers to generate the required
 * frequency.
 *
 * @param  pll_num  the PLL that you wish to modify
 * @param  mfi      multiplication factor integer part
 * @param  pdf      pre-division factor
 * @param  mfd      multiplication factor denominator
 * @param  mfn      multiplication factor numerator
 */
void mxc_pll_set(enum plls pll_num, unsigned int mfi, unsigned int pdf,
		 unsigned int mfd, unsigned int mfn)
{
	unsigned long cscr;
	unsigned long flags;
	unsigned long new_pll = 0;

	spin_lock_irqsave(&mxc_crm_lock, flags);

	if (pll_num == MCUPLL) {
		/* Change the Pll value      */
		new_pll = (mfi << CCM_MPCTL0_MFI_OFFSET) |
		    (mfn << CCM_MPCTL0_MFN_OFFSET) |
		    (mfd << CCM_MPCTL0_MFD_OFFSET) |
		    (pdf << CCM_MPCTL0_PD_OFFSET);

		__raw_writel(new_pll, IO_ADDRESS(CCM_BASE_ADDR) + CCM_MPCTL0);
		/* Swap to reference clock and disable PLL */
		cscr = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		cscr |= CCM_CSCR_MPLLRES;
		__raw_writel(cscr, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
	} else {
		if (pll_num == SERIALPLL) {
			/* Change the Pll value      */
			new_pll = (mfi << CCM_SPCTL0_MFI_OFFSET) |
			    (mfn << CCM_SPCTL0_MFN_OFFSET) |
			    (mfd << CCM_SPCTL0_MFD_OFFSET) |
			    (pdf << CCM_SPCTL0_PD_OFFSET);

			__raw_writel(new_pll,
				     IO_ADDRESS(CCM_BASE_ADDR) + CCM_SPCTL0);
			/* Swap to reference clock and disable PLL */
			cscr =
			    __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
			cscr |= CCM_CSCR_SPLLRES;
			__raw_writel(cscr,
				     IO_ADDRESS(CCM_BASE_ADDR) + CCM_CSCR);
		}
	}

	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * Configure clock output on CKO pins
 *
 * @param   output  clock output pin
 * @param   clk     clock source to output
 * @param   div     CLKO divider
 *
 */
void mxc_set_clock_output(enum mxc_clk_out output, enum mxc_clocks clk, int div)
{
	unsigned long flags;
	unsigned long reg;

	if (output != CKO) {
		return;
	}

	spin_lock_irqsave(&mxc_crm_lock, flags);

	switch (clk) {
	case CKIH_CLK:
		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_CCSR);
		reg = (reg & (~0x1f)) | 0x2;
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_CCSR);

		reg = __raw_readl(IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);
		reg = (reg & (~0x03c00000)) | 0x02000000 | ((div - 1) << 22);
		__raw_writel(reg, IO_ADDRESS(CCM_BASE_ADDR) + CCM_PCDR0);

		break;
	default:
		break;
	};

	spin_unlock_irqrestore(&mxc_crm_lock, flags);
	return;
}

/*!
 * This function is called to read the contents of a CCM_MCU register
 *
 * @param reg_offset the CCM_MCU register that will read
 *
 * @return the register contents
 */
unsigned long mxc_ccm_get_reg(unsigned int reg_offset)
{
	/* TODO */
	return 0xFFFFFFFF;
}

/*!
 * This function is called to modify the contents of a CCM_MCU register
 *
 * @param reg_offset the CCM_MCU register that will read
 * @param mask       the mask to be used to clear the bits that are to be modified
 * @param data       the data that should be written to the register
 */
void mxc_ccm_modify_reg(unsigned int reg_offset, unsigned int mask,
			unsigned int data)
{
	/* TODO */
}

EXPORT_SYMBOL(mxc_pll_set);
EXPORT_SYMBOL(mxc_pll_clock);
EXPORT_SYMBOL(mxc_get_clocks);
EXPORT_SYMBOL(mxc_set_clocks_pll);
EXPORT_SYMBOL(mxc_set_clocks_div);
EXPORT_SYMBOL(mxc_clks_disable);
EXPORT_SYMBOL(mxc_clks_enable);
EXPORT_SYMBOL(mxc_ccm_get_reg);
EXPORT_SYMBOL(mxc_ccm_modify_reg);
EXPORT_SYMBOL(mxc_set_clock_output);
EXPORT_SYMBOL(mxc_get_clocks_parent);
