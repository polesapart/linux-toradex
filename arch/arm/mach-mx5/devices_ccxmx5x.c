/*
 * Copyright 2011 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/smsc911x.h>
#include <linux/sysfs.h>
#include <mach/common.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <mach/memory.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/mxc_dvfs.h>
#if defined(CONFIG_MODULE_CCXMX53)
#include <mach/iomux-mx53.h>
#endif

#include "devices_ccxmx5x.h"

static u8 ccxmx5x_mod_variant = 0;
static u8 ccxmx5x_mod_rev = 0;
static u32 ccxmx5x_mod_sn = 0;
static u8 ccxmx5x_bb_rev = BASE_BOARD_REV;

void ccxmx5x_set_mod_variant(u8 variant)
{
	ccxmx5x_mod_variant = variant;
}
void ccxmx5x_set_mod_revision(u8 revision)
{
	ccxmx5x_mod_rev = revision;
}
u8 ccxmx5x_get_mod_revision(void)
{
	return ccxmx5x_mod_rev;
}
void ccxmx5x_set_mod_sn(u32 sn)
{
	ccxmx5x_mod_sn = sn;
}

#ifdef CONFIG_SYSFS
static ssize_t ccxmx5x_mod_variant_attr_show(struct kobject *kobj,
					     struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccxmx5x_mod_variant);
}

static ssize_t ccxmx5x_mod_rev_attr_show(struct kobject *kobj,
					 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccxmx5x_mod_rev);
}

static ssize_t ccxmx5x_mod_sn_attr_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccxmx5x_mod_sn);
}

static ssize_t ccxmx5x_bb_rev_attr_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", ccxmx5x_bb_rev);
}

static struct kobj_attribute ccxmx5x_mod_variant_attr =
	__ATTR(mod_variant, S_IRUGO, ccxmx5x_mod_variant_attr_show, NULL);
static struct kobj_attribute ccxmx5x_mod_rev_attr =
	__ATTR(mod_rev, S_IRUGO, ccxmx5x_mod_rev_attr_show, NULL);
static struct kobj_attribute ccxmx5x_mod_sn_attr =
	__ATTR(mod_sn, S_IRUGO, ccxmx5x_mod_sn_attr_show, NULL);
static struct kobj_attribute ccxmx5x_bb_rev_attr =
	__ATTR(bb_rev, S_IRUGO, ccxmx5x_bb_rev_attr_show, NULL);

int ccxmx5x_create_sysfs_entries(void)
{
	struct kobject *ccxmx5x_kobj;
	int ret;

	ccxmx5x_kobj = kobject_create_and_add(CCXMX5X_SYSFS_FNAME, kernel_kobj);
	if (!ccxmx5x_kobj) {
		printk(KERN_WARNING "kobject_create_and_add %s failed\n",
		       CCXMX5X_SYSFS_FNAME);
		return -EINVAL;
	}

	ret = sysfs_create_file(ccxmx5x_kobj, &ccxmx5x_mod_variant_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware variant\n",
		       CCXMX5X_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccxmx5x_kobj, &ccxmx5x_mod_rev_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware revision\n",
		       CCXMX5X_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccxmx5x_kobj, &ccxmx5x_mod_sn_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s hardware SN\n",
		       CCXMX5X_SYSFS_FNAME);
		return ret;
	}
	ret = sysfs_create_file(ccxmx5x_kobj, &ccxmx5x_bb_rev_attr.attr);
	if (ret) {
		printk(KERN_ERR
		       "Unable to register sysdev entry for %s base board hardware revision\n",
		       CCXMX5X_SYSFS_FNAME);
		return ret;
	}

	return 0;
}
#else
int ccxmx5x_create_sysfs_entries(void) { return 0; }
#endif /* CONFIG_SYSFS */

