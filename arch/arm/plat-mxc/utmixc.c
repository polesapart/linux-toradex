/*
 * Copyright 2005-2008 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/usb/fsl_xcvr.h>

#include <asm/hardware.h>
#include <asm/arch/arc_otg.h>

static void usb_utmi_init(struct fsl_xcvr_ops *this)
{
}

static void usb_utmi_uninit(struct fsl_xcvr_ops *this)
{
}

static struct fsl_xcvr_ops utmi_ops = {
	.name = "utmi",
	.xcvr_type = PORTSC_PTS_UTMI,
	.init = usb_utmi_init,
	.uninit = usb_utmi_uninit,
};

extern void fsl_usb_xcvr_register(struct fsl_xcvr_ops *xcvr_ops);

static int __init utmixc_init(void)
{
	fsl_usb_xcvr_register(&utmi_ops);
	return 0;
}

extern void fsl_usb_xcvr_unregister(struct fsl_xcvr_ops *xcvr_ops);

static void __exit utmixc_exit(void)
{
	fsl_usb_xcvr_unregister(&utmi_ops);
}

module_init(utmixc_init);
module_exit(utmixc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("utmi xcvr driver");
MODULE_LICENSE("GPL");
