/*
 * Real Time Clock driver for Freescale MC13783 PMIC
 *
 * (C) 2009 Sascha Hauer, Pengutronix
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/mfd/mc13783-private.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/rtc.h>

struct mc13783_rtc {
	struct rtc_device *rtc;
	struct mc13783 *mc13783;
};

static int mc13783_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	struct mc13783_rtc *priv = dev_get_drvdata(dev);
	unsigned int seconds, days;

	seconds = secs % 86400;
	days = secs / 86400;

	mc13783_reg_write(priv->mc13783, MC13783_REG_RTC_TIME, seconds);
	mc13783_reg_write(priv->mc13783, MC13783_REG_RTC_DAY, days);

	return 0;
}

static int mc13783_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct mc13783_rtc *priv = dev_get_drvdata(dev);
	unsigned int seconds, days1, days2;
	unsigned long s1970;

	rtc_tm_to_time(tm, &s1970);

	do {
		mc13783_reg_read(priv->mc13783, MC13783_REG_RTC_TIME, &seconds);
		mc13783_reg_read(priv->mc13783, MC13783_REG_RTC_DAY, &days1);
		mc13783_reg_read(priv->mc13783, MC13783_REG_RTC_DAY, &days2);
	} while (days1 != days2);

	s1970 = days1 * 86400 + seconds;

	rtc_time_to_tm(s1970, tm);

	return rtc_valid_tm(tm);
}

static const struct rtc_class_ops mc13783_rtc_ops = {
	.read_time	= mc13783_rtc_read_time,
	.set_mmss	= mc13783_rtc_set_mmss,
};

static int __devinit mc13783_rtc_probe(struct platform_device *pdev)
{
	int err;
	struct mc13783_rtc *priv;

	priv = kzalloc(sizeof *priv, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->mc13783 = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, priv);

	priv->rtc = rtc_device_register(pdev->name,
			&pdev->dev, &mc13783_rtc_ops, THIS_MODULE);

	if (IS_ERR(priv->rtc)) {
		err = PTR_ERR(priv->rtc);
		goto exit_kfree;
	}

	return 0;

exit_kfree:
	kfree(priv);
	return err;
}

static int __devexit mc13783_rtc_remove(struct platform_device *pdev)
{
	struct mc13783_rtc *priv = platform_get_drvdata(pdev);

	rtc_device_unregister(priv->rtc);
	platform_set_drvdata(pdev, NULL);

	kfree(priv);

	return 0;
}

static struct platform_driver mc13783_rtc_driver = {
	.remove		= __devexit_p(mc13783_rtc_remove),
	.driver = {
		.name	= "mc13783-rtc",
		.owner	= THIS_MODULE,
	},
};

static int __init mc13783_rtc_init(void)
{
	return platform_driver_probe(&mc13783_rtc_driver, &mc13783_rtc_probe);
}
module_init(mc13783_rtc_init);

static void __exit mc13783_rtc_exit(void)
{
	platform_driver_unregister(&mc13783_rtc_driver);
}
module_exit(mc13783_rtc_exit);


MODULE_AUTHOR("Sascha Hauer <s.hauer@pengutronix.de>");
MODULE_DESCRIPTION("RTC driver for Freescale MC13783 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mc13783-rtc");
