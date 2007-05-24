/*
 * Copyright 2005-2006 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * @file mx27_csi.c
 *
 * @brief CMOS Sensor interface functions
 *
 * @ingroup CSI
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <asm/arch/hardware.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <asm/arch/clock.h>
#include <linux/module.h>

#include "mx27_csi.h"

#undef MX27_CSI_INT

static csi_config_t g_csi_cfg;	/* csi hardware configuration */
static csi_status_t g_csi_status;	/* cache copy of csi status register */

static bool gcsi_mclk_on = false;
static int gcsi_mclk_source = 0;

#ifdef MX27_CSI_INT
static void csihw_read_status(csi_status_t * cs)
{
	cs->sff_or_int = (__raw_readl(CSI_CSISR) & BIT_SFF_OR_INT) ? 1 : 0;
	cs->rff_or_int = (__raw_readl(CSI_CSISR) & BIT_RFF_OR_INT) ? 1 : 0;
	cs->statff_int = (__raw_readl(CSI_CSISR) & BIT_STATFF_INT) ? 1 : 0;
	cs->rxff_int = (__raw_readl(CSI_CSISR) & BIT_RXFF_INT) ? 1 : 0;
	cs->eof_int = (__raw_readl(CSI_CSISR) & BIT_EOF_INT) ? 1 : 0;
	cs->sof_int = (__raw_readl(CSI_CSISR) & BIT_SOF_INT) ? 1 : 0;
	cs->f2_int = (__raw_readl(CSI_CSISR) & BIT_F2_INT) ? 1 : 0;
	cs->f1_int = (__raw_readl(CSI_CSISR) & BIT_F1_INT) ? 1 : 0;
	cs->cof_int = (__raw_readl(CSI_CSISR) & BIT_COF_INT) ? 1 : 0;
	cs->ecc_int = (__raw_readl(CSI_CSISR) & BIT_ECC_INT) ? 1 : 0;
	cs->drdy = (__raw_readl(CSI_CSISR) & BIT_DRDY) ? 1 : 0;
}

static irqreturn_t csi_irq_handler(int irq, void *data)
{
	csihw_read_status(&g_csi_status);

	/* rx fifo overflow */
	if (g_csi_cfg.rf_or_inten && g_csi_status.rff_or_int) {
		pr_debug("csi_irq_handler rx fifo overflow. \n");
		__raw_writel(BIT_RFF_OR_INT, CSI_CSISR);
	}

	/* start of frame */
	if (g_csi_cfg.sof_inten && g_csi_status.sof_int) {
		pr_debug("csi_irq_handler SOF.\n");
		__raw_writel(BIT_SOF_INT, CSI_CSISR);
	}

	return IRQ_HANDLED;
}
#endif

static void csihw_set_config(csi_config_t * cfg)
{
	unsigned val = 0;

	/* control reg 1 */
	val |= cfg->swap16_en ? BIT_SWAP16_EN : 0;
	val |= cfg->ext_vsync ? BIT_EXT_VSYNC : 0;
	val |= cfg->eof_int_en ? BIT_EOF_INT_EN : 0;
	val |= cfg->prp_if_en ? BIT_PRP_IF_EN : 0;
	val |= cfg->ccir_mode ? BIT_CCIR_MODE : 0;
	val |= cfg->cof_int_en ? BIT_COF_INT_EN : 0;
	val |= cfg->sf_or_inten ? BIT_SF_OR_INTEN : 0;
	val |= cfg->rf_or_inten ? BIT_RF_OR_INTEN : 0;
	val |= cfg->statff_level << SHIFT_STATFF_LEVEL;
	val |= cfg->staff_inten ? BIT_STATFF_INTEN : 0;
	val |= cfg->rxff_level << SHIFT_RXFF_LEVEL;
	val |= cfg->rxff_inten ? BIT_RXFF_INTEN : 0;
	val |= cfg->sof_pol ? BIT_SOF_POL : 0;
	val |= cfg->sof_inten ? BIT_SOF_INTEN : 0;
	val |= cfg->mclkdiv << SHIFT_MCLKDIV;
	val |= cfg->hsync_pol ? BIT_HSYNC_POL : 0;
	val |= cfg->ccir_en ? BIT_CCIR_EN : 0;
	val |= cfg->mclken ? BIT_MCLKEN : 0;
	val |= cfg->fcc ? BIT_FCC : 0;
	val |= cfg->pack_dir ? BIT_PACK_DIR : 0;
	val |= cfg->gclk_mode ? BIT_GCLK_MODE : 0;
	val |= cfg->inv_data ? BIT_INV_DATA : 0;
	val |= cfg->inv_pclk ? BIT_INV_PCLK : 0;
	val |= cfg->redge ? BIT_REDGE : 0;

	__raw_writel(val, CSI_CSICR1);

	/* control reg 3 */
	val = 0x0;
	val |= cfg->csi_sup ? BIT_CSI_SUP : 0;
	val |= cfg->zero_pack_en ? BIT_ZERO_PACK_EN : 0;
	val |= cfg->ecc_int_en ? BIT_ECC_INT_EN : 0;
	val |= cfg->ecc_auto_en ? BIT_ECC_AUTO_EN : 0;

	__raw_writel(val, CSI_CSICR3);

	/* rxfifo counter */
	__raw_writel(cfg->rxcnt, CSI_CSIRXCNT);

	/* update global config */
	memcpy(&g_csi_cfg, cfg, sizeof(csi_config_t));

	/* status flags */
	memset(&g_csi_status, 0, sizeof(csi_status_t));
}

static void csihw_enable_mclk(bool flag)
{
	if (flag)
		__raw_writel(__raw_readl(CSI_CSICR1) | BIT_MCLKEN, CSI_CSICR1);
	else
		__raw_writel(__raw_readl(CSI_CSICR1) & ~BIT_MCLKEN, CSI_CSICR1);
}

static void csihw_reset_frame_count(void)
{
	__raw_writel(__raw_readl(CSI_CSICR3) | BIT_FRMCNT_RST, CSI_CSICR3);
}

static void csihw_reset(void)
{
	csihw_reset_frame_count();
	__raw_writel(CSICR1_RESET_VAL, CSI_CSICR1);
	__raw_writel(CSICR2_RESET_VAL, CSI_CSICR2);
	__raw_writel(CSICR3_RESET_VAL, CSI_CSICR3);
}

/*!
 * csi_init_interface
 *    Sets initial values for the CSI registers.
 *    The width and height of the sensor and the actual frame size will be
 *    set to the same values.
 * @param       width        Sensor width
 * @param       height       Sensor height
 * @param       pixel_fmt    pixel format
 * @param       sig          csi_signal_cfg_t
 *
 * @return      0 for success, -EINVAL for error
 */
int32_t csi_init_interface(uint16_t width, uint16_t height,
			   uint32_t pixel_fmt, csi_signal_cfg_t sig)
{
	csi_config_t cfg;

	/* Set the CSI_SENS_CONF register remaining fields */
	cfg.swap16_en = 1;
	cfg.ext_vsync = sig.ext_vsync;
	cfg.eof_int_en = 0;
	cfg.prp_if_en = 1;
	cfg.ccir_mode = 0;
	cfg.cof_int_en = 0;
	cfg.sf_or_inten = 0;
	cfg.rf_or_inten = 0;
	cfg.statff_level = 0;
	cfg.staff_inten = 0;
	cfg.rxff_level = 2;
	cfg.rxff_inten = 1;
	cfg.sof_pol = 1;
	cfg.sof_inten = 0;
	cfg.mclkdiv = 0;
	cfg.hsync_pol = 1;
	cfg.ccir_en = 0;
	cfg.mclken = gcsi_mclk_on ? 1 : 0;
	cfg.fcc = 1;
	cfg.pack_dir = 0;
	cfg.gclk_mode = 1;
	cfg.inv_data = sig.data_pol;
	cfg.inv_pclk = sig.pixclk_pol;
	cfg.redge = 1;
	cfg.csicnt1_rsv = 0;

	/* control reg 3 */
	cfg.frmcnt = 0;
	cfg.frame_reset = 0;
	cfg.csi_sup = 0;
	cfg.zero_pack_en = 0;
	cfg.ecc_int_en = 0;
	cfg.ecc_auto_en = 0;

	csihw_set_config(&cfg);

	return 0;
}

/*!
 * csi_enable_mclk
 *
 * @param       src         enum define which source to control the clk
 *                          CSI_MCLK_VF CSI_MCLK_ENC CSI_MCLK_RAW CSI_MCLK_I2C
 * @param       flag        true to enable mclk, false to disable mclk
 * @param       wait        true to wait 100ms make clock stable, false not wait
 *
 * @return      0 for success
 */
int32_t csi_enable_mclk(int src, bool flag, bool wait)
{
	if (flag == true)
		gcsi_mclk_source |= src;
	else
		gcsi_mclk_source &= ~src;

	if (gcsi_mclk_on == flag)
		return 0;

	if (flag == true) {
		csihw_enable_mclk(true);
		if (wait == true)
			msleep(10);
		pr_debug("Enable csi clock from source %d\n", src);
		gcsi_mclk_on = true;
	} else if (gcsi_mclk_source == 0) {
		csihw_enable_mclk(false);
		pr_debug("Disable csi clock from source %d\n", src);
		gcsi_mclk_on = flag;
	}

	return 0;
}

/*!
 * csi_read_mclk_flag
 *
 * @return  gcsi_mclk_source
 */
int csi_read_mclk_flag(void)
{
	return gcsi_mclk_source;
}

int32_t __init csi_init_module(void)
{
	int ret = 0;

	mxc_clks_enable(CSI_BAUD);
	csihw_reset();

#ifdef MX27_CSI_INT
	/* interrupt enable */
	ret = request_irq(INT_CSI, csi_irq_handler, 0, "csi", 0);
	if (ret)
		pr_debug("CSI error: irq request fail\n");
#endif

	return ret;
}

void __exit csi_cleanup_module(void)
{
#ifdef MX27_CSI_INT
	/* free irq */
	free_irq(INT_CSI, 0);
#endif

	mxc_clks_disable(CSI_BAUD);
}

module_init(csi_init_module);
module_exit(csi_cleanup_module);

EXPORT_SYMBOL(csi_init_interface);
EXPORT_SYMBOL(csi_enable_mclk);
EXPORT_SYMBOL(csi_read_mclk_flag);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MX27 CSI driver");
MODULE_LICENSE("GPL");
