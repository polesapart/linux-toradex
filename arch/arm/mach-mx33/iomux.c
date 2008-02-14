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

#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/hardware.h>
#include <asm/arch/gpio.h>
#include "iomux.h"

#define IOMUX_BASE_ADDRESS IO_ADDRESS(IOMUXC_BASE_ADDR)

static DEFINE_SPINLOCK(gpio_mux_lock);

static u32 iomux_pin_res_table[MUX_PIN_NUM_MAX];

static int inline iomux_warning_ignored(iomux_pin_name_t pin)
{
	if (pin != MX33_PIN_UART1_CTS &&
	    pin != MX33_PIN_UART1_RTS &&
	    pin != MX33_PIN_UART1_DCD &&
	    pin != MX33_PIN_UART1_DSR &&
	    pin != MX33_PIN_UART1_DTR &&
	    pin != MX33_PIN_UART1_RI &&
	    pin != MX33_PIN_UART1_RXD && pin != MX33_PIN_UART1_TXD) {
		return 1;
	}
	return 0;
}

/*!
 * This function is used to configure a pin through the IOMUX module.
 * FIXED ME: for backward compatible. Will be static function!
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 *
 * @return 		0 if successful; Non-zero otherwise
 */
int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		     iomux_pin_icfg_t in)
{
	u32 ret = 0;
	u32 mux_index = PIN_TO_IOMUX_INDEX(pin);
	u32 mux_reg = IOMUX_BASE_ADDRESS + PIN_TO_IOMUX_OFFSET(pin);
	u32 *rp;

	BUG_ON((mux_index > (MUX_PIN_NUM_MAX - 1)) || (out != in));

	spin_lock(&gpio_mux_lock);
	/*
	 * Log a warning if a pin changes ownership
	 */
	rp = iomux_pin_res_table + (mux_index >> 2);
	if ((out & *rp) && (*rp != out)) {
		/*
		 * Don't call printk if we're tweaking the console uart or
		 * we'll deadlock.
		 */
		if (!iomux_warning_ignored(pin)) {
			printk(KERN_ERR "iomux_config_mux: Warning: iomux pin"
			       " config changed, reg=%x"
			       " prev=0x%x new=0x%x\n", mux_reg, *rp, out);
		}
		ret = -EINVAL;
	} else {
		__raw_writel(out, mux_reg);

		*rp = out;
	}
	spin_unlock(&gpio_mux_lock);

	return ret;
}

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		      iomux_pin_icfg_t in)
{
	int ret = iomux_config_mux(pin, out, in);
	int gpio_port = GPIO_TO_PORT(IOMUX_TO_GPIO(pin));
	if (!ret && (gpio_port != 7) && (out == PIN_TO_IOGPIO_INDEX(pin))) {
		ret |= mxc_request_gpio(pin);
	}
	return ret;
}

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		    iomux_pin_icfg_t in)
{
	u32 mux_index = PIN_TO_IOMUX_INDEX(pin);
	u32 *rp = iomux_pin_res_table + (mux_index >> 2);
	int gpio_port = GPIO_TO_PORT(IOMUX_TO_GPIO(pin));

	BUG_ON(mux_index > (MUX_PIN_NUM_MAX - 1));

	*rp = 0;
	if ((gpio_port != 7) && (out == PIN_TO_IOGPIO_INDEX(pin))) {
		mxc_free_gpio(pin);
	}
}

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config)
{
	u32 pad_reg = IOMUX_BASE_ADDRESS + PIN_TO_IOPAD_OFFSET(pin);

	BUG_ON(pad_reg > (PAD_CTL_GRP_BASE - 1));

	spin_lock(&gpio_mux_lock);
	__raw_writel(config, pad_reg);
	spin_unlock(&gpio_mux_lock);
}

/*!
 * This function configures the pad value for a IOMUX pin group.
 *
 * @param  grp         a pin number as defined in \b #iomux_pad_grp_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad_grp(iomux_pad_grp_t grp, u32 config)
{
	u32 grp_reg = IOMUX_BASE_ADDRESS + PAD_CTL_GRP_INDEX(grp);

	BUG_ON(grp > (PAD_CTL_GRP_MAX - 1));

	spin_lock(&gpio_mux_lock);
	__raw_writel(config, grp_reg);
	spin_unlock(&gpio_mux_lock);

}

/*!
 * This function select a input for a IOMUX signal.
 *
 * @param  signal       a signal number as defined in \b #iomux_signal_name_t
 * @param  config       value of input configuration
 */
void iomux_select_input(iomux_signal_name_t signal, __u32 config)
{
	u32 input_reg = IOMUX_BASE_ADDRESS + IOMUXC_SELECT_INPUT(signal);

	BUG_ON(signal > (IOMUX_SIGNAL_MAX - 1));

	spin_lock(&gpio_mux_lock);
	__raw_writel(config, input_reg);
	spin_unlock(&gpio_mux_lock);
}

/*
 * FIXED ME: for backward compatible. to be removed!
 */
void iomux_config_pad(iomux_pin_name_t pin, u32 config)
{
	mxc_iomux_set_pad(pin, config);
}

/*
 * FIXED ME: for backward compatible. to be removed!
 */
void iomux_config_pad_grp(iomux_pad_grp_t grp, __u32 config)
{
	mxc_iomux_set_pad_grp(grp, config);
}

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en)
{
//      u32 reg, l;

	spin_lock(&gpio_mux_lock);
/*TODO: after the spec is finished
	reg = base + gp>>5;
	BUG_ON(reg > base+num)
	l = __raw_readl(reg);
	if (en) {
		l |= 1<<(gp&31);
	} else {
		l &= ~(1<<(gp&31));
	}
	__raw_writel(l, reg);
*/
	spin_unlock(&gpio_mux_lock);
}

/*!
 * FIXED ME: for backward compatible. to be removed!
 */
void iomux_config_gpr(iomux_gp_func_t gp, bool en)
{
	mxc_iomux_set_gpr(gp, en);
}

EXPORT_SYMBOL(mxc_request_iomux);
EXPORT_SYMBOL(mxc_free_iomux);
EXPORT_SYMBOL(mxc_iomux_set_pad);
EXPORT_SYMBOL(mxc_iomux_set_gpr);
EXPORT_SYMBOL(iomux_config_pad);
EXPORT_SYMBOL(iomux_config_gpr);
EXPORT_SYMBOL(iomux_config_mux);
