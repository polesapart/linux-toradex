/*
 * Copyright 2005-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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

int __mxc_get_clocks_div(enum mxc_clocks clk, unsigned int *div);

/*
 * This function returns the clock CKIH clock input value.
 */
static unsigned long mxc_clock_ckih(void)
{
	static unsigned long g_clk_ckih = 0;

	if (g_clk_ckih == 0) {
		g_clk_ckih = CKIH_CLK_FREQ;
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
	unsigned long mfi, mfn, mfd, pdf, ref_clk;
	volatile unsigned long base, reg;
	unsigned long long temp;

	reg = __raw_readl(MXC_CCM_CCSR);
	ref_clk = mxc_clock_ckih();

	switch (pll) {
	case MCUPLL:
		/*TODO::assume just configure step_clk to reference clock */
		if ((reg & MXC_CCM_CCSR_PLL1_BYPASS) == 1) {
			return ref_clk;
		}
		base = MXC_DPLL1_BASE;
		break;
	case BUSPLL:
		base = MXC_DPLL2_BASE;
		break;
	case SERIALPLL:
		base = MXC_DPLL3_BASE;
		break;
	default:
		printk(KERN_ERR "\nUnknown pll: %d\n\n", pll);
		BUG();		/* oops */
		return 0;
	}

	mfn = __raw_readl(base + MXC_DPLL_DP_HFS_MFN);
	reg = __raw_readl(base + MXC_DPLL_DP_HFS_OP);
	pdf = ((reg & MXC_DPLL_PDF_MASK) >> MXC_DPLL_PDF_OFFSET);
	mfi = ((reg & MXC_DPLL_MFI_MASK) >> MXC_DPLL_MFI_OFFSET);
	reg = __raw_readl(base + MXC_DPLL_DP_HFS_MFD);
	mfd = ((reg & MXC_DPLL_MFD_MASK) >> MXC_DPLL_MFD_OFFSET);
	reg = __raw_readl(base + MXC_DPLL_DP_HFS_MFN);
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
static unsigned long mxc_main_clock(unsigned long reg, unsigned int offset)
{
	unsigned long val = (__raw_readl(reg) >> offset) & 3;

	if (val == 2) {
		return mxc_pll_clock(SERIALPLL);
	} else if (val == 1) {
		return mxc_pll_clock(BUSPLL);
	}
	return mxc_pll_clock(MCUPLL);
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
	unsigned long pll, ret_val = 0;
	switch (clk) {
		/*TODO:: Need support it. */
		/*case CKIL_CLK:
		   ret_val = CKIL_CLK_FREQ;
		   break; */
	case CKIH_CLK:
		ret_val = mxc_clock_ckih();
		break;
	case CPU_CLK:
		pll = mxc_pll_clock(MCUPLL);
		ret_val = pll / mxc_get_clocks_div(CPU_CLK);
		break;
	case AHB_CLK:
		pll =
		    mxc_main_clock(MXC_CCM_CAMR,
				   MXC_CCM_CAMR_PERIPH_CLK_SEL_OFFSET);
		ret_val = pll / mxc_get_clocks_div(AHB_CLK);
		break;
	case NFC_CLK:
		pll =
		    mxc_main_clock(MXC_CCM_CAMR,
				   MXC_CCM_CAMR_PERIPH_CLK_SEL_OFFSET);
		ret_val = pll / mxc_get_clocks_div(NFC_CLK);
		break;
	case UART1_BAUD:
	case UART2_BAUD:
	case UART3_BAUD:
	case UART4_BAUD:
	case UART5_BAUD:
	case I2C_CLK:
	case I2C2_CLK:
	case I2C3_CLK:
	case OWIRE_CLK:
	case GPT_CLK:
	case PERCLK:
		pll = __raw_readl(MXC_CCM_CSCMR1);
		pll =
		    mxc_main_clock(MXC_CCM_CAMR,
				   MXC_CCM_CAMR_PERIPH_CLK_SEL_OFFSET);
		ret_val = pll / mxc_get_clocks_div(clk);
		break;
	case SDMA_CLK:
	case IPG_CLK:
		pll =
		    mxc_main_clock(MXC_CCM_CAMR,
				   MXC_CCM_CAMR_PERIPH_CLK_SEL_OFFSET);
		ret_val =
		    pll / (mxc_get_clocks_div(AHB_CLK) +
			   mxc_get_clocks_div(IPG_CLK));
		break;
	case SDHC1_CLK:
	case SDHC2_CLK:
	case SDHC3_CLK:
	case MSTICK1_BAUD:
	case MSTICK2_BAUD:
		pll = mxc_get_clocks_parent(clk);
		ret_val = pll / mxc_get_clocks_div(clk);
		break;
	default:
		ret_val = 66516666;
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
	unsigned long reg = __raw_readl(MXC_CCM_CSCMR1);

	switch (clk) {
	case CSI1_BAUD:
		reg = MXC_CCM_GET_FIELD(reg,
					MXC_CCM_CSCMR1_SEL_BITS,
					MXC_CCM_CSCMR1_CSI_MCLK1_CLK_SEL_OFFSET);
		break;
	case CSI2_BAUD:
		reg = MXC_CCM_GET_FIELD(reg,
					MXC_CCM_CSCMR1_SEL_BITS,
					MXC_CCM_CSCMR1_CSI_MCLK2_CLK_SEL_OFFSET);
		break;
	case SDHC3_CLK:
		if (reg & (1 << MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_OFFSET)) {
			clk = SDHC2_CLK;
		} else {
			clk = SDHC1_CLK;
		}
	case SDHC1_CLK:
	case SDHC2_CLK:
	case MSTICK1_BAUD:
	case MSTICK2_BAUD:
		if (clk == SDHC1_CLK || clk == MSTICK1_BAUD) {
			reg = MXC_CCM_GET_FIELD(reg,
						MXC_CCM_CSCMR1_SEL_BITS,
						MXC_CCM_CSCMR1_ESDHC1_MSHC1_CLK_SEL_OFFSET);
		} else {
			reg = MXC_CCM_GET_FIELD(reg,
						MXC_CCM_CSCMR1_SEL_BITS,
						MXC_CCM_CSCMR1_ESDHC2_MSHC2_CLK_SEL_OFFSET);
		}
		break;
	default:
		return ret_val;
	}
	switch (reg) {
	case 0:		/*PLL1_sw_clock */
		ret_val = mxc_pll_clock(MCUPLL);
		break;
	case 1:		/*PLL2_sw_clock */
		ret_val = mxc_pll_clock(BUSPLL);
		break;
	case 2:		/*PLL3_sw_clock */
		ret_val = mxc_pll_clock(SERIALPLL);
	}
	return ret_val;
}

/*!
 * This function calculates the pre and post divider values for a clock
 *
 * @param div  divider value passed in
 * @param div1 returns the pre-divider value
 * @param div2 returns the post-divider value
 * @param lim1 limit of divider 1
 * @param lim2 limit of divider 2
 */
void mxc_clk_getdivs(unsigned int div, unsigned int *div1,
		     unsigned int *div2, int lim1, int lim2)
{
	int i;

	if (div <= lim1) {
		*div1 = div;
		*div2 = 1;
		return;
	}
	if (div <= lim2) {
		*div1 = 1;
		*div2 = div;
		return;
	}
	for (i = 2; i < (lim1 + 1); i++) {
		if ((div % i) == 0) {
			*div1 = i;
			*div2 = div / i;
			return;
		}
	}

	*div1 = 1;
	*div2 = lim2;
	return;
}

/*!
 * This function sets the PLL source for a clock.
 *
 * @param clk     as defined in enum mxc_clocks
 * @param pll_num the PLL that you wish to use as source for this clock
 */
void mxc_set_clocks_pll(enum mxc_clocks clk, enum plls pll_num)
{
	unsigned long reg, sel;
	unsigned long flags;

	switch (pll_num) {
	case MCUPLL:
		sel = 0;
		break;
	case BUSPLL:
		sel = 1;
		break;
	case SERIALPLL:
		sel = 2;
		break;
	default:
		return;
	}

	spin_lock_irqsave(&mxc_crm_lock, flags);

	switch (clk) {
	case CSI1_BAUD:
		reg = __raw_readl(MXC_CCM_CSCMR1);
		reg &= ~(MXC_CCM_MASK(MXC_CCM_CSCMR1_SEL_BITS,
				      MXC_CCM_CSCMR1_CSI_MCLK1_CLK_SEL_OFFSET));
		reg |= sel << MXC_CCM_CSCMR1_CSI_MCLK1_CLK_SEL_OFFSET;
		__raw_writel(reg, MXC_CCM_CSCMR1);
		break;
	case CSI2_BAUD:
		reg = __raw_readl(MXC_CCM_CSCMR1);
		reg &= ~(MXC_CCM_MASK(MXC_CCM_CSCMR1_SEL_BITS,
				      MXC_CCM_CSCMR1_CSI_MCLK2_CLK_SEL_OFFSET));
		reg |= sel << MXC_CCM_CSCMR1_CSI_MCLK2_CLK_SEL_OFFSET;
		__raw_writel(reg, MXC_CCM_CSCMR1);
		break;
	default:
		printk(KERN_ERR
		       "This clock does not have ability to choose its clock source\n");
		break;
	}
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
	return;
}

/*!
 * This function gets the divider value for a clock .
 *
 * @param clk as defined in enum mxc_clocks
 * @return the division factor
 */
unsigned long mxc_get_clocks_div(enum mxc_clocks clk)
{
	unsigned long reg, flags, ret = 1;
	unsigned int div1 = 0, div2 = 0, div3 = 0;

	spin_lock_irqsave(&mxc_crm_lock, flags);
	switch (clk) {
	case AHB_CLK:
		reg = __raw_readl(MXC_CCM_CBCDR2);
		ret = 1 + MXC_CCM_GET_FIELD(reg,
					    MXC_CCM_CBCDR2_AHB_CORE_PODF_BITS,
					    MXC_CCM_CBCDR2_AHB_CORE_PODF_OFFSET);
		break;
	case CPU_CLK:
		reg = __raw_readl(MXC_CCM_CACRR);
		ret = 1 + (MXC_CCM_GET_FIELD(reg,
					     MXC_CCM_CACRR_ARM_PODF_BITS,
					     MXC_CCM_CACRR_ARM_PODF_OFFSET));
		break;
	case SDMA_CLK:
	case IPG_CLK:
		reg = __raw_readl(MXC_CCM_CBCDR2);
		ret = 1 + MXC_CCM_GET_FIELD(reg,
					    MXC_CCM_CBCDR2_IPG_CORE_PODF_BITS,
					    MXC_CCM_CBCDR2_IPG_CORE_PODF_OFFSET);
		break;
	case NFC_CLK:
		reg = __raw_readl(MXC_CCM_CBCDR7);
		ret = 1 + MXC_CCM_GET_FIELD(reg,
					    MXC_CCM_CBCDR7_NFC_PODF_BITS,
					    MXC_CCM_CBCDR7_NFC_PODF_OFFSET);
		break;
	case CSI1_BAUD:
	case CSI2_BAUD:
		reg = __raw_readl(MXC_CCM_CSCDR2);
		if (clk == CSI1_BAUD) {
			div1 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PRED_BITS,
						     MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PRED_OFFSET);
			div2 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_BITS,
						     MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_OFFSET);
		} else {
			div1 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PRED_BITS,
						     MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PRED_OFFSET);
			div2 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_BITS,
						     MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_OFFSET);
		}
		ret = div1 * div2;
		break;
	case UART1_BAUD:
	case UART2_BAUD:
	case UART3_BAUD:
	case UART4_BAUD:
	case UART5_BAUD:
	case I2C_CLK:
	case I2C2_CLK:
	case I2C3_CLK:
	case OWIRE_CLK:
	case GPT_CLK:
	case PERCLK:
		reg = __raw_readl(MXC_CCM_CBCDR2);
		div1 = 1 + MXC_CCM_GET_FIELD(reg,
					     MXC_CCM_CBCDR2_PERCLK_PRED1_BITS,
					     MXC_CCM_CBCDR2_PERCLK_PRED1_OFFSET);
		div2 = 1 + MXC_CCM_GET_FIELD(reg,
					     MXC_CCM_CBCDR2_PERCLK_PRED2_BITS,
					     MXC_CCM_CBCDR2_PERCLK_PRED2_OFFSET);
		div3 = 1 + MXC_CCM_GET_FIELD(reg,
					     MXC_CCM_CBCDR2_PERCLK_PODF_BITS,
					     MXC_CCM_CBCDR2_PERCLK_PODF_OFFSET);
		ret = div1 * div2 * div3;
		break;
	case SDHC3_CLK:
		reg = __raw_readl(MXC_CCM_CSCMR1);
		if (reg & (1 << MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_OFFSET)) {
			clk = SDHC2_CLK;
		} else {
			clk = SDHC1_CLK;
		}
	case SDHC1_CLK:
	case SDHC2_CLK:
	case MSTICK1_BAUD:
	case MSTICK2_BAUD:
		reg = __raw_readl(MXC_CCM_CSCDR1);
		if (clk == SDHC1_CLK || clk == MSTICK1_BAUD) {
			div1 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PRED_BITS,
						     MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PRED_OFFSET);
			div2 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PODF_BITS,
						     MXC_CCM_CSCDR1_ESDHC1_MSHC1_CLK_PODF_OFFSET);
		} else {
			div1 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_BITS,
						     MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PRED_OFFSET);
			div2 = 1 + MXC_CCM_GET_FIELD(reg,
						     MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_BITS,
						     MXC_CCM_CSCDR1_ESDHC2_MSHC2_CLK_PODF_OFFSET);
		}
		ret = div1 * div2;
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
	return ret;
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
	volatile unsigned long reg;
	unsigned long flags;
	unsigned int d = 0, div1 = 0, div2 = 0, div3;

	spin_lock_irqsave(&mxc_crm_lock, flags);

	switch (clk) {
	case AHB_CLK:
		reg = __raw_readl(MXC_CCM_CBCDR2);
		reg &= (~MXC_CCM_MASK(MXC_CCM_CBCDR2_AHB_CORE_PODF_BITS,
				      MXC_CCM_CBCDR2_AHB_CORE_PODF_OFFSET));
		reg |= (((div - 1) << MXC_CCM_CBCDR2_AHB_CORE_PODF_OFFSET) &
			MXC_CCM_MASK(MXC_CCM_CBCDR2_AHB_CORE_PODF_BITS,
				     MXC_CCM_CBCDR2_AHB_CORE_PODF_OFFSET));
		__raw_writel(reg, MXC_CCM_CBCDR2);
		break;
	case CPU_CLK:
		reg = __raw_readl(MXC_CCM_CACRR);
		reg &= (~MXC_CCM_MASK(MXC_CCM_CACRR_ARM_PODF_BITS,
				      MXC_CCM_CACRR_ARM_PODF_OFFSET));
		reg |= (((div - 1) << MXC_CCM_CACRR_ARM_PODF_OFFSET) &
			MXC_CCM_MASK(MXC_CCM_CACRR_ARM_PODF_BITS,
				     MXC_CCM_CACRR_ARM_PODF_OFFSET));
		__raw_writel(reg, MXC_CCM_CACRR);
		break;
	case IPG_CLK:
		reg = __raw_readl(MXC_CCM_CBCDR2);
		reg &= (~MXC_CCM_MASK(MXC_CCM_CBCDR2_AHB_CORE_PODF_BITS,
				      MXC_CCM_CBCDR2_IPG_CORE_PODF_OFFSET));
		reg |= (((div - 1) << MXC_CCM_CBCDR2_IPG_CORE_PODF_OFFSET) &
			MXC_CCM_MASK(MXC_CCM_CBCDR2_AHB_CORE_PODF_BITS,
				     MXC_CCM_CBCDR2_IPG_CORE_PODF_OFFSET));
		__raw_writel(reg, MXC_CCM_CBCDR2);
		break;
	case NFC_CLK:
		reg = __raw_readl(MXC_CCM_CBCDR7);
		reg &= (~MXC_CCM_MASK(MXC_CCM_CBCDR7_NFC_PODF_BITS,
				      MXC_CCM_CBCDR7_NFC_PODF_OFFSET));
		reg |= (((div - 1) << MXC_CCM_CBCDR7_NFC_PODF_OFFSET) &
			MXC_CCM_MASK(MXC_CCM_CBCDR7_NFC_PODF_BITS,
				     MXC_CCM_CBCDR7_NFC_PODF_OFFSET));
		__raw_writel(reg, MXC_CCM_CBCDR7);
		break;
	case CSI1_BAUD:
	case CSI2_BAUD:
		d = div / 2;
		mxc_clk_getdivs(d, &div1, &div2, 8, 64);
		reg = __raw_readl(MXC_CCM_CSCDR2);
		if (clk == CSI1_BAUD) {
			reg &=
			    ~MXC_CCM_MASK(MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PRED_BITS
					  +
					  MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_BITS,
					  MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_OFFSET);
			reg |=
			    ((((div1 -
				1) << MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PRED_OFFSET)
			      | ((div2 - 1) <<
				 MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_OFFSET)) &
			     MXC_CCM_MASK(MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PRED_BITS
					  +
					  MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_BITS,
					  MXC_CCM_CSCDR2_CSI_MCLK1_CLK_PODF_OFFSET));
		} else {
			reg &=
			    ~MXC_CCM_MASK(MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PRED_BITS
					  +
					  MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_BITS,
					  MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_OFFSET);
			reg |=
			    ((((div1 -
				1) << MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PRED_OFFSET)
			      | ((div2 - 1) <<
				 MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_OFFSET)) &
			     MXC_CCM_MASK(MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PRED_BITS
					  +
					  MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_BITS,
					  MXC_CCM_CSCDR2_CSI_MCLK2_CLK_PODF_OFFSET));
		}
		__raw_writel(reg, MXC_CCM_CSCDR2);
		break;
	case UART1_BAUD:
	case UART2_BAUD:
	case UART3_BAUD:
	case UART4_BAUD:
	case UART5_BAUD:
	case I2C_CLK:
	case I2C2_CLK:
	case I2C3_CLK:
	case OWIRE_CLK:
	case GPT_CLK:
	case PERCLK:
		d = div / 2;
		mxc_clk_getdivs(d, &div2, &div3, 32, 8);
		d = div2 / 2;
		mxc_clk_getdivs(d, &div1, &div2, 4, 8);
		reg = __raw_readl(MXC_CCM_CBCDR2);
		reg &= (~MXC_CCM_MASK(MXC_CCM_CBCDR2_PERCLK_PRED1_BITS +
				      MXC_CCM_CBCDR2_PERCLK_PRED2_BITS +
				      MXC_CCM_CBCDR2_PERCLK_PODF_BITS,
				      MXC_CCM_CBCDR2_PERCLK_PODF_OFFSET));
		reg |= ((((div1 - 1) << MXC_CCM_CBCDR2_PERCLK_PRED1_OFFSET) |
			 ((div2 - 1) << MXC_CCM_CBCDR2_PERCLK_PRED2_OFFSET) |
			 ((div3 - 1) << MXC_CCM_CBCDR2_PERCLK_PODF_OFFSET)) &
			MXC_CCM_MASK(MXC_CCM_CBCDR2_PERCLK_PRED1_BITS +
				     MXC_CCM_CBCDR2_PERCLK_PRED2_BITS +
				     MXC_CCM_CBCDR2_PERCLK_PODF_BITS,
				     MXC_CCM_CBCDR2_PERCLK_PODF_OFFSET));
		__raw_writel(reg, MXC_CCM_CBCDR2);
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
	case UART1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART1_IPG_CLK_GATE,
				    MXC_CCM_CCGR_UART1_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART1_PERCLK_GATE,
				    MXC_CCM_CCGR_UART1_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART2_IPG_CLK_GATE,
				    MXC_CCM_CCGR_UART2_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART2_PERCLK_GATE,
				    MXC_CCM_CCGR_UART2_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART3_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART3_IPG_CLK_GATE,
				    MXC_CCM_CCGR_UART3_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART3_PERCLK_GATE,
				    MXC_CCM_CCGR_UART3_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART4_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART4_IPG_CLK_GATE,
				    MXC_CCM_CCGR_UART4_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART4_PERCLK_GATE,
				    MXC_CCM_CCGR_UART4_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART5_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART5_IPG_CLK_GATE,
				    MXC_CCM_CCGR_UART5_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_UART5_PERCLK_GATE,
				    MXC_CCM_CCGR_UART5_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case I2C1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_I2C1_CLK_GATE,
				    MXC_CCM_CCGR_I2C1_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case I2C2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_I2C2_CLK_GATE,
				    MXC_CCM_CCGR_I2C2_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case I2C3_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_I2C3_CLK_GATE,
				    MXC_CCM_CCGR_I2C3_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case OWIRE_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_OWIRE_CLK_GATE,
				    MXC_CCM_CCGR_OWIRE_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case FEC_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_FEC_CLK_GATE,
				    MXC_CCM_CCGR_FEC_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case USB_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_USBOH2_IPG_HCLK_GATE,
				    MXC_CCM_CCGR_USBOH2_IPG_HCLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_USBOH2_60M_CLK_GATE,
				    MXC_CCM_CCGR_USBOH2_60M_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case SSI1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SSI1_IPG_CLK_GATE,
				    MXC_CCM_CCGR_SSI1_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SSI1_SSI_CLK_GATE,
				    MXC_CCM_CCGR_SSI1_SSI_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SSI2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SSI2_IPG_CLK_GATE,
				    MXC_CCM_CCGR_SSI2_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SSI2_SSI_CLK_GATE,
				    MXC_CCM_CCGR_SSI2_SSI_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SSI_EXT1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SSI_EXT1_CLK_GATE,
				    MXC_CCM_CCGR_SSI_EXT1_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SSI_EXT2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SSI_EXT2_CLK_GATE,
				    MXC_CCM_CCGR_SSI_EXT2_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case MARC_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_MARC_CLK_GATE,
				    MXC_CCM_CCGR_MARC_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case CSI1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR5);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSI_MCLK1_CLK_GATE,
				    MXC_CCM_CCGR_CSI_MCLK1_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR5);
		break;
	case CSI2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR5);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSI_MCLK2_CLK_GATE,
				    MXC_CCM_CCGR_CSI_MCLK2_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR5);
		break;
	case IPU_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_DI_CLK_GATE,
				    MXC_CCM_CCGR_DI_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_IPU_CLK_GATE,
				    MXC_CCM_CCGR_IPU_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case SDHC1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_ESDHC1_IPG_HCLK_GATE,
				    MXC_CCM_CCGR_ESDHC1_IPG_HCLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_ESDHC1_PERCLK_GATE,
				    MXC_CCM_CCGR_ESDHC1_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case SDHC2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_ESDHC2_IPG_HCLK_GATE,
				    MXC_CCM_CCGR_ESDHC2_IPG_HCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR2);
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_ESDHC2_PERCLK_GATE,
				    MXC_CCM_CCGR_ESDHC2_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SDHC3_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_ESDHC3_IPG_HCLK_GATE,
				    MXC_CCM_CCGR_ESDHC3_IPG_HCLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_ESDHC3_PERCLK_GATE,
				    MXC_CCM_CCGR_ESDHC3_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case CSPI1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSPI1_IPG_CLK_GATE,
				    MXC_CCM_CCGR_CSPI1_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSPI1_PERCLK_GATE,
				    MXC_CCM_CCGR_CSPI1_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case CSPI2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSPI2_IPG_CLK_GATE,
				    MXC_CCM_CCGR_CSPI2_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSPI2_PERCLK_GATE,
				    MXC_CCM_CCGR_CSPI2_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case CSPI3_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSPI3_IPG_CLK_GATE,
				    MXC_CCM_CCGR_CSPI3_IPG_CLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_CSPI3_PERCLK_GATE,
				    MXC_CCM_CCGR_CSPI3_PERCLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case ATA_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_PATA_CLK_GATE,
				    MXC_CCM_CCGR_PATA_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SIM_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SIM_CLK_GATE,
				    MXC_CCM_CCGR_SIM_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case RTIC_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_RTIC_CLK_GATE,
				    MXC_CCM_CCGR_RTIC_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case SDMA_CLK:
		reg = __raw_readl(MXC_CCM_CCGR5);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_SDMA_CLK_GATE,
				    MXC_CCM_CCGR_SDMA_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR5);
		break;
	case MSTICK1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_MSHC1_IPG_HCLK_GATE,
				    MXC_CCM_CCGR_MSHC1_IPG_HCLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_MSHC1_DI_CLK_GATE,
				    MXC_CCM_CCGR_MSHC1_DI_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case MSTICK2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_MSHC2_IPG_HCLK_GATE,
				    MXC_CCM_CCGR_MSHC2_IPG_HCLK_EN);
		reg =
		    MXC_CCM_CCGR_ON(reg, MXC_CCM_CCGR_MSHC2_DI_CLK_GATE,
				    MXC_CCM_CCGR_MSHC2_DI_CLK_EN);
		__raw_writel(reg, MXC_CCM_CCGR3);
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
	case UART1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART1_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART1_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART2_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART2_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART3_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART3_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART3_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART4_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART4_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART4_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case UART5_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART5_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_UART5_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case I2C1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_I2C1_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case I2C2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR1);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_I2C2_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR1);
		break;
	case I2C3_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_I2C3_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case OWIRE_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_OWIRE_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case FEC_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_FEC_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case USB_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_USBOH2_IPG_HCLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_USBOH2_60M_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case SSI1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SSI1_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SSI1_SSI_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SSI2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SSI2_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SSI2_SSI_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SSI_EXT1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SSI_EXT1_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SSI_EXT2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SSI_EXT2_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case MARC_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_MARC_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case CSI1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR5);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSI_MCLK1_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR5);
		break;
	case CSI2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR5);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSI_MCLK2_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR5);
		break;
	case IPU_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_DI_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_IPU_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case SDHC1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_ESDHC1_IPG_HCLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_ESDHC1_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR2);
		break;
	case SDHC2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR2);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_ESDHC2_IPG_HCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR2);
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_ESDHC2_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SDHC3_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_ESDHC3_IPG_HCLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_ESDHC3_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case CSPI1_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSPI1_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSPI1_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case CSPI2_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSPI2_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSPI2_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case CSPI3_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSPI3_IPG_CLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_CSPI3_PERCLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case ATA_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_PATA_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case SIM_CLK:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SIM_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case RTIC_CLK:
		reg = __raw_readl(MXC_CCM_CCGR4);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_RTIC_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR4);
		break;
	case SDMA_CLK:
		reg = __raw_readl(MXC_CCM_CCGR5);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_SDMA_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR5);
		break;
	case MSTICK1_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_MSHC1_IPG_HCLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_MSHC1_DI_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	case MSTICK2_BAUD:
		reg = __raw_readl(MXC_CCM_CCGR3);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_MSHC2_IPG_HCLK_GATE);
		reg = MXC_CCM_CCGR_OFF(reg, MXC_CCM_CCGR_MSHC2_DI_CLK_GATE);
		__raw_writel(reg, MXC_CCM_CCGR3);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * This function is called to read the contents of a CCM register
 *
 * @param reg_offset the CCM register that will read
 *
 * @return the register contents
 */
unsigned long mxc_ccm_get_reg(unsigned int reg_offset)
{
	volatile unsigned long reg;

	reg = __raw_readl(reg_offset);
	return reg;
}

/*!
 * This function is called to modify the contents of a CCM register
 *
 * @param reg_offset the CCM register that will read
 * @param mask       the mask to be used to clear the bits that are to be modified
 * @param data       the data that should be written to the register
 */
void mxc_ccm_modify_reg(unsigned int reg_offset, unsigned int mask,
			unsigned int data)
{
	unsigned long flags;
	volatile unsigned long reg;

	spin_lock_irqsave(&mxc_crm_lock, flags);
	reg = __raw_readl(reg_offset);
	reg = (reg & (~mask)) | data;
	__raw_writel(reg, reg_offset);
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
	volatile unsigned long ccmr;
	unsigned long flags;
	unsigned long base, val;

	spin_lock_irqsave(&mxc_crm_lock, flags);

	if (pll_num == MCUPLL) {
		/* Swap to reference clock and disable PLL */
		/* The PLL1 step clock should be connect with reference clock */
		ccmr = __raw_readl(MXC_CCM_CCSR);
		ccmr |= MXC_CCM_CCSR_PLL1_BYPASS;
		__raw_writel(ccmr, MXC_CCM_CCSR);
	}
	/* Change the Pll value */
	switch (pll_num) {
	case MCUPLL:
		base = MXC_DPLL1_BASE;
		break;
	case BUSPLL:
		base = MXC_DPLL2_BASE;
		break;
	case SERIALPLL:
		base = MXC_DPLL3_BASE;
		break;
	default:
		goto exit;
	}

	__raw_writel(mfn, base + MXC_DPLL_DP_HFS_MFN);

	val = (mfd << MXC_DPLL_MFD_OFFSET) & MXC_DPLL_MFD_MASK;
	__raw_writel(val, base + MXC_DPLL_DP_HFS_MFD);

	val = (mfi << MXC_DPLL_MFI_OFFSET) & MXC_DPLL_MFI_MASK;
	val |= (pdf << MXC_DPLL_PDF_OFFSET) & MXC_DPLL_PDF_MASK;
	__raw_writel(val, base + MXC_DPLL_DP_HFS_OP);

	val = __raw_readl(base + MXC_DPLL_DP_CONFIG);
	val |= MXC_DPLL_DP_CONFIG_LDREQ;
	__raw_writel(val, base + MXC_DPLL_DP_CONFIG);

#if 0
	//TODO: enable this section when it run in hardware.
	while (1) {
		if (!(__raw_readl(base + MXC_DPLL_DP_CONFIG) &
		      MXC_DPLL_DP_CONFIG_LDREQ)) {
			break;
		}
	}
#endif

	if (pll_num == MCUPLL) {
		/* Swap to the new value */
		ccmr = __raw_readl(MXC_CCM_CCSR);
		ccmr &= ~MXC_CCM_CCSR_PLL1_BYPASS;
		__raw_writel(ccmr, MXC_CCM_CCSR);
	}
      exit:
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * This function is used to get PLL registers value used to generate the clock
 * frequency.
 *
 * @param  pll_num  the PLL that you wish to modify
 * @param  mfi      pointer that holds multiplication factor integer part
 * @param  pdf      pointer that holds pre-division factor
 * @param  mfd      pointer that holds multiplication factor denominator
 * @param  mfn      pointer that holds multiplication factor numerator
 */
void mxc_pll_get(enum plls pll_num, unsigned int *mfi, unsigned int *pdf,
		 unsigned int *mfd, unsigned int *mfn)
{
	unsigned long flags;
	unsigned long base, val;

	spin_lock_irqsave(&mxc_crm_lock, flags);

	switch (pll_num) {
	case MCUPLL:
		base = MXC_DPLL1_BASE;
		break;
	case BUSPLL:
		base = MXC_DPLL2_BASE;
		break;
	case SERIALPLL:
		base = MXC_DPLL3_BASE;
		break;
	default:
		goto exit;
	}

	*mfn = __raw_readl(base + MXC_DPLL_DP_HFS_MFN);

	val = __raw_readl(base + MXC_DPLL_DP_HFS_MFD);
	*mfd = (val & MXC_DPLL_MFD_MASK) >> MXC_DPLL_MFD_OFFSET;

	val = __raw_readl(base + MXC_DPLL_DP_HFS_OP);
	*mfi = (val & MXC_DPLL_MFI_MASK) >> MXC_DPLL_MFI_OFFSET;
	*pdf = (val & MXC_DPLL_PDF_MASK) >> MXC_DPLL_PDF_OFFSET;
      exit:
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

/*!
 * Configure clock output on CKO pin
 *
 * @param       output     as defined in enum mxc_clk_out
 * @param       clk        as defined in enum mxc_clocks
 * @param       div        clock output divider value. The possible values
 *                               for this are 1, 2, 4, 8 and 16
 *
 */
void mxc_set_clock_output(enum mxc_clk_out output, enum mxc_clocks clk, int div)
{
	unsigned long flags;
	u32 reg;

	if ((--div) & 0xFFFFFFF8) {
		/*invalid dividor */
		return;
	}

	spin_lock_irqsave(&mxc_crm_lock, flags);

	reg = __raw_readl(MXC_CCM_CCOSR);

	switch (output) {
	case CKO2:
		goto ret;
	case CKO1:
		reg &=
		    ~(((1 << MXC_CCM_CCOSR_CKO1_DIV_BITS) -
		       1) << MXC_CCM_CCOSR_CKO1_DIV_OFFSET);
		reg &=
		    ~(((1 << MXC_CCM_CCOSR_CKO1_SEL_BITS) -
		       1) << MXC_CCM_CCOSR_CKO1_SEL_OFFSET);
		switch (clk) {
		case CPU_CLK:
			/* To select AP clock : */
			/*CPU_CLK = (0<<MXC_CCM_CCOSR_CKO1_SEL_OFFSET) */
			break;
		case AHB_CLK:
			/* To select AHB clock */
			reg |= 0x0000000B << MXC_CCM_CCOSR_CKO1_SEL_OFFSET;
			break;
		case IPG_CLK:
			/* To select IP clock */
			reg |= 0x0000000C << MXC_CCM_CCOSR_CKO1_SEL_OFFSET;
			break;
		default:
			goto ret;
		}
		reg |= (1 << MXC_CCM_CCOSR_CKO1_EN_OFFSET);
		break;
	default:
		goto ret;
	}

	__raw_writel(reg, MXC_CCM_CCOSR);

      ret:
	spin_unlock_irqrestore(&mxc_crm_lock, flags);
}

EXPORT_SYMBOL(mxc_pll_get);
EXPORT_SYMBOL(mxc_pll_set);
EXPORT_SYMBOL(mxc_pll_clock);
EXPORT_SYMBOL(mxc_get_clocks);
EXPORT_SYMBOL(mxc_get_clocks_parent);
EXPORT_SYMBOL(mxc_set_clocks_pll);
EXPORT_SYMBOL(mxc_set_clocks_div);
EXPORT_SYMBOL(mxc_get_clocks_div);
EXPORT_SYMBOL(mxc_clks_disable);
EXPORT_SYMBOL(mxc_clks_enable);
EXPORT_SYMBOL(mxc_ccm_get_reg);
EXPORT_SYMBOL(mxc_ccm_modify_reg);
EXPORT_SYMBOL(mxc_set_clock_output);
