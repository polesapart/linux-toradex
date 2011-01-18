/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2009 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/gpio.h>

#include "iomux.h"

/**
 * iomux settings for the external ethernet mac
 */
#if defined(CONFIG_SMSC9118)
static struct mxc_iomux_pin_cfg __initdata ccwmx51_iomux_ext_eth_pins[] = {
	{MX51_PIN_EIM_CS5, IOMUX_CONFIG_ALT0,
			(PAD_CTL_HYS_ENABLE | PAD_CTL_PUE_KEEPER | PAD_CTL_DRV_MEDIUM), },
	{MX51_PIN_EIM_OE, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA0, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA1, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA2, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA3, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA4, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA5, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA6, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_DA7, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D16, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D17, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D18, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D19, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D20, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D21, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D22, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D23, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D24, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D25, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D26, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D27, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D28, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D29, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D30, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_EIM_D31, IOMUX_CONFIG_ALT0,},
	{MX51_PIN_GPIO1_9, IOMUX_CONFIG_ALT0, (PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU), },
};
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccwmx51_iomux_mmc_pins[] = {
	/* SDHC1*/
	{
		MX51_PIN_SD1_CMD, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_CLK, IOMUX_CONFIG_ALT0 | IOMUX_CONFIG_SION,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA0, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA1, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA2, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_SD1_DATA3, IOMUX_CONFIG_ALT0,
		(PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_DRV_HIGH |
		PAD_CTL_47K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_GPIO1_0, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},

	/* SDHC3*/
	{
		MX51_PIN_NANDF_RDY_INT, IOMUX_CONFIG_ALT5 | IOMUX_CONFIG_SION,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	},
	{
		MX51_PIN_NANDF_CS7, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
	},
	{	/* SD3 DATA0 */
		MX51_PIN_NANDF_D8, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT0_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 DATA1 */
		MX51_PIN_NANDF_D9, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT1_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 DATA2 */
		MX51_PIN_NANDF_D10, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT2_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 DATA3 */
		MX51_PIN_NANDF_D11, IOMUX_CONFIG_ALT5,
		(PAD_CTL_DRV_MAX | PAD_CTL_22K_PU | PAD_CTL_SRE_FAST),
		MUX_IN_ESDHC3_IPP_DAT3_IN_SELECT_INPUT, INPUT_CTL_PATH1
	},
	{	/* SD3 Card detect */
		MX51_PIN_GPIO_NAND, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},
	{	/* SD3 Write protect */
		MX51_PIN_NANDF_CS1, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_HYS_ENABLE | PAD_CTL_100K_PU),
	},
};
#endif

#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
static struct mxc_iomux_pin_cfg __initdata ccwmx51_iomux_usbh1_pins[] = {
	{	/* USBH1_STP */
		MX51_PIN_USBH1_STP, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_CLK */
		MX51_PIN_USBH1_CLK, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	},
	{	/* USBH1_DIR */
		MX51_PIN_USBH1_DIR, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	},
	{	/* USBH1_NXT */
		MX51_PIN_USBH1_NXT, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_PUE_KEEPER |
		PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE | PAD_CTL_DDR_INPUT_CMOS),
	},
	{	/* USBH1_DATA0 */
		MX51_PIN_USBH1_DATA0, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA1 */
		MX51_PIN_USBH1_DATA1, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA2 */
		MX51_PIN_USBH1_DATA2, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA3 */
		MX51_PIN_USBH1_DATA3, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA4 */
		MX51_PIN_USBH1_DATA4, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA5 */
		MX51_PIN_USBH1_DATA5, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA6 */
		MX51_PIN_USBH1_DATA6, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH1_DATA7 */
		MX51_PIN_USBH1_DATA7, IOMUX_CONFIG_ALT0,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_100K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
	{	/* USBH PHY RESET */
		MX51_PIN_DISPB2_SER_RS, IOMUX_CONFIG_GPIO,
		(PAD_CTL_SRE_FAST | PAD_CTL_DRV_HIGH | PAD_CTL_47K_PU |
		PAD_CTL_PUE_KEEPER | PAD_CTL_PKE_ENABLE | PAD_CTL_HYS_ENABLE),
	},
};
#endif

static struct mxc_iomux_pin_cfg __initdata ccwmx51_iomux_devices_pins[] = {
	{	/* PMIC interrupt line */
		MX51_PIN_GPIO1_5, IOMUX_CONFIG_GPIO | IOMUX_CONFIG_SION,
		(PAD_CTL_SRE_SLOW | PAD_CTL_DRV_MEDIUM | PAD_CTL_100K_PU |
		PAD_CTL_HYS_ENABLE | PAD_CTL_DRV_VOT_HIGH),
	},
};


void __init ccwmx51_io_init(void)
{
	int i;

#if defined(CONFIG_SMSC9118)
	for (i = 0; i < ARRAY_SIZE(ccwmx51_iomux_ext_eth_pins); i++) {
		mxc_request_iomux(ccwmx51_iomux_ext_eth_pins[i].pin,
				ccwmx51_iomux_ext_eth_pins[i].mux_mode);
		if (ccwmx51_iomux_ext_eth_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccwmx51_iomux_ext_eth_pins[i].pin,
					ccwmx51_iomux_ext_eth_pins[i].pad_cfg);
		if (ccwmx51_iomux_ext_eth_pins[i].in_select)
			mxc_iomux_set_input(ccwmx51_iomux_ext_eth_pins[i].in_select,
					ccwmx51_iomux_ext_eth_pins[i].in_mode);
	}
#endif

	for (i = 0; i < ARRAY_SIZE(ccwmx51_iomux_devices_pins); i++) {
		mxc_request_iomux(ccwmx51_iomux_devices_pins[i].pin,
				  ccwmx51_iomux_devices_pins[i].mux_mode);
		if (ccwmx51_iomux_devices_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccwmx51_iomux_devices_pins[i].pin,
					  ccwmx51_iomux_devices_pins[i].pad_cfg);
		if (ccwmx51_iomux_devices_pins[i].in_select)
			mxc_iomux_set_input(ccwmx51_iomux_devices_pins[i].in_select,
					    ccwmx51_iomux_devices_pins[i].in_mode);
	}

	/* PMIC interrupt line */
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5), "gpio1_5");
	gpio_direction_input(IOMUX_TO_GPIO(MX51_PIN_GPIO1_5));

#if defined(CONFIG_USB_EHCI_ARC_H1) || defined(CONFIG_USB_EHCI_ARC_H1_MODULE)
	for (i = 0; i < ARRAY_SIZE(ccwmx51_iomux_usbh1_pins); i++) {
		mxc_request_iomux(ccwmx51_iomux_usbh1_pins[i].pin,
				  ccwmx51_iomux_usbh1_pins[i].mux_mode);
		if (ccwmx51_iomux_usbh1_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccwmx51_iomux_usbh1_pins[i].pin,
					  ccwmx51_iomux_usbh1_pins[i].pad_cfg);
		if (ccwmx51_iomux_usbh1_pins[i].in_select)
			mxc_iomux_set_input(ccwmx51_iomux_usbh1_pins[i].in_select,
					    ccwmx51_iomux_usbh1_pins[i].in_mode);
	}

	/* USB PHY/HUB reset*/
	gpio_request(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), "gpio3_8");
	gpio_direction_output(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 0);
	msleep(1);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DISPB2_SER_RS), 1);
#endif

#if defined(CONFIG_MMC_IMX_ESDHCI) || defined(CONFIG_MMC_IMX_ESDHCI_MODULE)
	for (i = 0; i < ARRAY_SIZE(ccwmx51_iomux_mmc_pins); i++) {
		mxc_request_iomux(ccwmx51_iomux_mmc_pins[i].pin,
				  ccwmx51_iomux_mmc_pins[i].mux_mode);
		if (ccwmx51_iomux_mmc_pins[i].pad_cfg)
			mxc_iomux_set_pad(ccwmx51_iomux_mmc_pins[i].pin,
					  ccwmx51_iomux_mmc_pins[i].pad_cfg);
		if (ccwmx51_iomux_mmc_pins[i].in_select)
			mxc_iomux_set_input(ccwmx51_iomux_mmc_pins[i].in_select,
					    ccwmx51_iomux_mmc_pins[i].in_mode);
	}
#endif
}
