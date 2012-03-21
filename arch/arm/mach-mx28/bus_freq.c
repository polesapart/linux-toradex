/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file bus_freq.c
 *
 * @brief A common API for the Freescale Semiconductor i.MXC CPUfreq module.
 *
 * @ingroup PM
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/notifier.h>
#include <linux/cpufreq.h>

#include <mach/hardware.h>
#include <linux/io.h>
#include <asm/system.h>
#include <mach/clock.h>
#include <mach/bus_freq.h>
#include <mach/arc_otg.h>

#include "regs-clkctrl.h"
#include "regs-digctl.h"

#define CLKCTRL_BASE_ADDR IO_ADDRESS(CLKCTRL_PHYS_ADDR)
#define DIGCTRL_BASE_ADDR IO_ADDRESS(DIGCTL_PHYS_ADDR)
#define USBCTRL0_BASE_ADDR IO_ADDRESS(USBCTRL0_PHYS_ADDR)
#define USBCTRL1_BASE_ADDR IO_ADDRESS(USBCTRL1_PHYS_ADDR)

#define BF(value, field) (((value) << BP_##field) & BM_##field)
struct profile profiles[] = {
	{ 454736, 151570, 205710, 0, 1550000,
	1450000, 355000, 3300000, 1750000, 24000, 0 },
	{ 360000, 120000, 130910, 0, 1350000,
	1250000, 200000, 3300000, 1750000, 24000, 0 },
	{ 261818, 130910, 130910, 0, 1350000,
	1250000, 173000, 3300000, 1750000, 24000, 0 },
	{  64000,  64000, 130910, 3, 1350000,
	1250000, 150000, 3300000, 1750000, 24000, 0 },
	{  0,  0,  0, 0, 0,
	0, 0, 0, 0, 0, 0 },
};

static struct device *busfreq_dev;
static struct clk *lcdif_clk;
u32 clkseq_setting;

/*
 * How to judge the usb port is in use
 *
 * Device port (usb0)
 * ID=1 and B Session Valid = 1
 *
 * Host port (usb0 and usb1)
 * ID=0(usb1 ID is always 1) and Current Connect Status = 1
 */
static int usb_is_in_use(void)
{
	void __iomem *addr_usb0_portsc1 = USBCTRL0_BASE_ADDR + UOG_PORTSC1;
	void __iomem *addr_usb0_otgsc = USBCTRL0_BASE_ADDR + UOG_OTGSC;
	void __iomem *addr_usb1_portsc1 = USBCTRL1_BASE_ADDR + UOG_PORTSC1;

	if ((__raw_readl(addr_usb0_otgsc) & OTGSC_STS_USB_ID) &&
		(__raw_readl(addr_usb0_otgsc) & OTGSC_STS_B_SESSION_VALID)) {
			pr_debug("usb device is in use \n");
			return 1;
	}

	if (!(__raw_readl(addr_usb0_otgsc) & OTGSC_STS_USB_ID) &&
		((__raw_readl(addr_usb0_portsc1) & PORTSC_CURRENT_CONNECT_STATUS) ||
		(__raw_readl(addr_usb1_portsc1) & PORTSC_CURRENT_CONNECT_STATUS))) {
			pr_debug("usb host is in use \n");
			return 1;
	} else {
		pr_debug("usb is not in use \n");
		return 0;
	}
}

int low_freq_used(void)
{
	if (((clk_get_usecount(lcdif_clk) == 0))
	    && (!usb_is_in_use())) {
		pr_debug("low freq\n");
		return 1;
	}
	else
		return 0;
}

int is_hclk_autoslow_ok(void)
{
	if (!usb_is_in_use()) {
		pr_debug("autoslow\n");
		return 1;
	}
	else
		return 0;
}

int timing_ctrl_rams(int ss)
{
	__raw_writel(BF(ss, DIGCTL_ARMCACHE_VALID_SS) |
				      BF(ss, DIGCTL_ARMCACHE_DRTY_SS) |
				      BF(ss, DIGCTL_ARMCACHE_CACHE_SS) |
				      BF(ss, DIGCTL_ARMCACHE_DTAG_SS) |
				      BF(ss, DIGCTL_ARMCACHE_ITAG_SS),
				      DIGCTRL_BASE_ADDR + HW_DIGCTL_ARMCACHE);
	return 0;
}

/*!
 * This is the probe routine for the bus frequency driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */
static int __devinit busfreq_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;

	busfreq_dev = &pdev->dev;

	lcdif_clk = clk_get(NULL, "dis_lcdif");
	if (IS_ERR(lcdif_clk)) {
		ret = PTR_ERR(lcdif_clk);
		return ret;
	}
	return 0;
}

static struct platform_driver busfreq_driver = {
	.driver = {
		   .name = "busfreq",
		},
	.probe = busfreq_probe,
};

/*!
 * Initialise the busfreq_driver.
 *
 * @return  The function always returns 0.
 */

static int __init busfreq_init(void)
{
	if (platform_driver_register(&busfreq_driver) != 0) {
		printk(KERN_ERR "busfreq_driver register failed\n");
		return -ENODEV;
	}

	printk(KERN_INFO "Bus freq driver module loaded\n");
	return 0;
}

static void __exit busfreq_cleanup(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
}

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("BusFreq driver");
MODULE_LICENSE("GPL");
