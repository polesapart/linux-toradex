/* -*- linux-c -*-
 * 
 * linux/arch/arm/mach-s3c2443/cc9m2443js-pm.c
 *
 * Copyright (c) 2009 Digi International Spain
 *
 * http://www.digi.com/products/embeddedsolutions/connectcore9m.jsp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Author : Luis Galdos <luis.galdos@digi.com>
 *
 */

#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/memory.h>
#include <asm/memory.h>
#include <mach/map.h>
#include <mach/regs-s3c2443-mem.h>
#include <mach/hardware.h>

#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>
#include <mach/regs-power.h>

#include <linux/interrupt.h>
#include <asm/mach/irq.h>
#include <mach/gpio.h>

#include "cc9m2443js-pm.h"


#define CC9M2443_PM_ITEM_OFFSET(off)           { .offset = off }
#define CC9M2443_PM_ITEM_BASE(bas)             { .base = bas }

struct cc9m2443js_pm_gpio {
	int gpio;
	unsigned long config;
	int irq;
	int state;
	unsigned long flags;
	irqreturn_t (* irq_handler)(int irq, void *gpio);
};

struct cc9m2443_pm_reg {
	void __iomem *base;
	unsigned long offset;
	unsigned long val;
};

/*
 * The SSMC registers are not part of the minimal IO mapping of the CPU
 */
static void __iomem *ssmc_vptr = NULL;

/* Converts the virtual to physical address of the stack pointer */
unsigned long sleep_phys_sp(void *sp)
{
	return virt_to_phys(sp);
}

/* These are the register for the external Ethernet controller (CS5) */
static struct cc9m2443_pm_reg s3c2443_regs_ssmc[] = {
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBIDCYR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTRDR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTWRR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTOENR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTWENR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBCR5),
	CC9M2443_PM_ITEM_OFFSET(S3C2443_SSMC_SMBWSTBRDR5),
};

/* We MUST save the reset configuration register */
static struct cc9m2443_pm_reg s3c2443_regs_syscon[] = {
	CC9M2443_PM_ITEM_BASE(S3C2443_RSTCON),
	CC9M2443_PM_ITEM_BASE(S3C2443_PWRCFG),
	CC9M2443_PM_ITEM_BASE(S3C2443_OSCSET),
};

/* If the passed register doesn't have a base register, the offset will be used */
static void cc9m2443_pm_restore_regs(struct cc9m2443_pm_reg *regs, int count,
				     void __iomem *offptr)
{
	int cnt;
	void __iomem *reg_ptr;
	
	if (!regs) {
		printk(KERN_ERR "[ RESUME ] NULL pointer passed.\n");
		return;
	}

	for (cnt = 0; cnt < count; cnt++, regs++) {
		reg_ptr = (regs->base) ? (regs->base) : (offptr + regs->offset);
		__raw_writel(regs->val, reg_ptr);
	}
}

/* If the passed register doesn't have a base register, the offset will be used */
static void cc9m2443_pm_save_regs(struct cc9m2443_pm_reg *regs, int count,
				  void __iomem *offptr)
{
	int cnt;
	void __iomem *reg_ptr;
	
	if (!regs) {
		printk(KERN_ERR "[ SUSPEND ] NULL pointer passed!\n");
		return;
	}
	
	for (cnt = 0; cnt < count; cnt++, regs++) {
		reg_ptr = (regs->base) ? (regs->base) : (offptr + regs->offset);
		regs->val = __raw_readl(reg_ptr);
	}
}

/* Save all the registers that are part of the CC9M2443 */
static int cc9m2443_pm_save(void)
{
	/* First map to our virtual memory if required */
	if (!ssmc_vptr)
		ssmc_vptr = ioremap_nocache(S3C2443_PA_SSMC, S3C2443_SZ_SSMC);

	if (!ssmc_vptr)
		printk(KERN_ERR "Couldn't save the regs of the SSMC\n");
	else
		cc9m2443_pm_save_regs(s3c2443_regs_ssmc, ARRAY_SIZE(s3c2443_regs_ssmc),
				      ssmc_vptr);

	cc9m2443_pm_save_regs(s3c2443_regs_syscon, ARRAY_SIZE(s3c2443_regs_syscon),
			      NULL);
	
	return 0;
}

static int cc9m2443_pm_restore(void)
{
	if (ssmc_vptr)
		cc9m2443_pm_restore_regs(s3c2443_regs_ssmc,
					 ARRAY_SIZE(s3c2443_regs_ssmc), ssmc_vptr);

	cc9m2443_pm_restore_regs(s3c2443_regs_syscon,
				 ARRAY_SIZE(s3c2443_regs_syscon), NULL);
	
	return 0;
}

/*
 * This is the CPU-specific prepare function (Code coming from the WinCE world)
 * (Luis Galdos)
 */
static void s3c2443_pm_prepare(void)
{
	__raw_writel(0xff80, S3C2443_RSTCON);
	__raw_writel(0x8000, S3C2443_OSCSET);
	__raw_writel(0x8201, S3C2443_PWRCFG);
}

static int cc9m2443js_pm_suspend(struct sys_device *sd, pm_message_t state)
{
        /* Write the magic value u-boot uses to check for resume into
         * the INFORM0 register, and ensure INFORM1 is set to the
         * correct address to resume from. */

	printk(KERN_DEBUG "[ SUSPEND ] Setting resume address to %p [0x%lx]\n",
	       s3c2443_cpu_resume,
	       virt_to_phys(s3c2443_cpu_resume));
        __raw_writel(0x2BED, S3C2412_INFORM0);
        __raw_writel(virt_to_phys(s3c2443_cpu_resume), S3C2412_INFORM1);

	/*
	 * This two function pointers are used by the platform driver (see [1])
	 * for preparing and entering in the sleep mode.
	 * [1] arch/arm/plat-s3c24xx/pm.c
	 */
	pm_cpu_prep = s3c2443_pm_prepare;
	pm_cpu_sleep = s3c2443_cpu_suspend;

	/* Save the registers */
	cc9m2443_pm_save();
	
        return 0;
}

static int cc9m2443js_pm_resume(struct sys_device *sd)
{
        __raw_writel(0x0, S3C2412_INFORM0);
	__raw_writel(0x0, S3C2412_INFORM1);
	__raw_writel(0x0, S3C2412_INFORM2);
	
	cc9m2443_pm_restore();
	
        return 0;
}

/* Internal system class/device for accessing to the PM-functions */
static struct sysdev_class cc9m2443js_pm_sysclass = {
        .name           = "cc9m2443js-pm",
        .suspend        = cc9m2443js_pm_suspend,
        .resume         = cc9m2443js_pm_resume,
};

static struct sys_device cc9m2443js_pm_sysdev = {
        .cls            = &cc9m2443js_pm_sysclass,
};

/* IRQ-handler for the GPIOs configured as wakeup-source */
static irqreturn_t cc9m2443js_wakeup_irq(int irq, void *_gpio)
{
	irqreturn_t retval;
	struct cc9m2443js_pm_gpio *gpio;

	gpio = (struct cc9m2443js_pm_gpio *)_gpio;
	retval = IRQ_HANDLED;
	if (gpio->irq_handler)
		retval = gpio->irq_handler(irq, gpio);
	
	return retval;
}

static int cc9m2443js_pm_gpio_probe(struct platform_device *pdev)
{
	struct cc9m2443js_pm_gpio *gpio;
	int retval;

	gpio = pdev->dev.platform_data;	
	gpio->irq = s3c2443_gpio_getirq(gpio->gpio);
	if (gpio->irq < 0) {
		printk(KERN_ERR "[ ERROR ] Invalid IRQ GPIO %i\n", gpio->gpio);
		retval = -EINVAL;
		goto exit_probe;
	}

	retval = request_irq(gpio->irq, cc9m2443js_wakeup_irq,
			     IRQF_DISABLED | gpio->flags, "cc9m2443-wio", gpio);
	if (retval) {
		printk(KERN_ERR "[ ERROR ] Requesting IRQ %i\n", gpio->irq);
		gpio->irq = -EINVAL;
		goto exit_probe;
	}

	/*
	 * Disable the IRQ first, we will activate it when we are going into
	 * the sleep-mode
	 */
	disable_irq(gpio->irq);
	s3c2443_gpio_cfgpin(gpio->gpio, gpio->config);
	
	/* Enable the wakeup support */
	device_init_wakeup(&pdev->dev, 1);
        device_set_wakeup_enable(&pdev->dev, 0);
	
	return 0;
	
exit_probe:
	return retval;
}

/* Free the requested resources (IRQ, etc.) */
static int cc9m2443js_pm_gpio_remove(struct platform_device *pdev)
{
	struct cc9m2443js_pm_gpio *gpio;

	gpio = pdev->dev.platform_data;

	/* Free the requested GPIO and configure it as input */
	free_irq(gpio->irq, gpio);

	s3c2443_gpio_cfgpin(gpio->gpio, S3C2410_GPIO_INPUT);
	
	return 0;
}

/*
 * Enable the IRQ when we are going into the sleep-mode and the GPIO is enabled
 * as a wakeup-source
 */
static int cc9m2443js_pm_gpio_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct cc9m2443js_pm_gpio *gpio;
	int retval;

	gpio = pdev->dev.platform_data;

	if (device_may_wakeup(&pdev->dev)) {

		if (gpio->irq <= 0) {
			retval = -EINVAL;
			goto exit_suspend;
		}

		enable_irq(gpio->irq);
		
		retval = enable_irq_wake(gpio->irq);
		if (retval)
			goto exit_suspend;
	}

	retval = 0;
	
exit_suspend:
	return retval;
}

/*
 * Disable the IRQ when resuming and the GPIO was enabled as a wakeup-source
 */
static int cc9m2443js_pm_gpio_resume(struct platform_device *pdev)
{
	struct cc9m2443js_pm_gpio *gpio;
	int retval;

	gpio = pdev->dev.platform_data;

	retval = 0;
	if (device_may_wakeup(&pdev->dev)) {

		if (gpio->irq <= 0) {
			retval = -ENODEV;
			goto exit_resume;
		}
		
		disable_irq_wake(gpio->irq);

		disable_irq(gpio->irq);
	}

 exit_resume:
	return retval;
}

/* Internal driver for the handling with wakeup GPIOs */
static struct platform_driver cc9m2443js_pm_drv = {
	.probe		= cc9m2443js_pm_gpio_probe,
	.remove		= cc9m2443js_pm_gpio_remove,
	.suspend	= cc9m2443js_pm_gpio_suspend,
	.resume		= cc9m2443js_pm_gpio_resume,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "cc9m2443-wio"
	}
};

/* Macro used for skipping the registering of the GPIO wakeup device */
#define CC9M2443JS_PM_GPIO_DISABLED             (-1)

/* Platform device for the wakeup GPIO 226 */
static struct cc9m2443js_pm_gpio cc9m2443js_gpio226 = {
#if defined(CONFIG_MACH_CC9M2443JS_PM_EXTIRQ_226)
	.gpio           = 226,
#else
	.gpio           = CC9M2443JS_PM_GPIO_DISABLED,
#endif
	.config         = S3C2410_GPF0_EINT0,
	.flags          = IRQF_TRIGGER_FALLING,
};

/* Platform device for the wakeup GPIO 232 */
static struct cc9m2443js_pm_gpio cc9m2443js_gpio232 = {
#if defined(CONFIG_MACH_CC9M2443JS_PM_EXTIRQ_232)
	.gpio           = 232,
#else
	.gpio           = CC9M2443JS_PM_GPIO_DISABLED,
#endif
	.config         = S3C2410_GPF6_EINT6,
	.flags          = IRQF_TRIGGER_FALLING,
};

/* Table of GPIOs that should be used as external wakeup */
static struct platform_device cc9m2443js_pm_gpios[] = {
	{
		.name = "cc9m2443-wio",
		.dev  = {
			.platform_data = &cc9m2443js_gpio226,
		}
	},{
		.name = "cc9m2443-wio",
		.dev  = {
			.platform_data = &cc9m2443js_gpio232,
		}
	}
};

/* This function generates the required sysclass for our platform */
void __init cc9m2443js_pm_init(void)
{
	int cnt;
	
	sysdev_class_register(&cc9m2443js_pm_sysclass);
	sysdev_register(&cc9m2443js_pm_sysdev);

	/* Register our small driver for the wakeup IRQs */
	platform_driver_register(&cc9m2443js_pm_drv);

	/* Add the corresponding GPIOs */
	for (cnt = 0; cnt < ARRAY_SIZE(cc9m2443js_pm_gpios); cnt++) {
		struct cc9m2443js_pm_gpio *gpio;
		struct platform_device *pdev;

		pdev = &cc9m2443js_pm_gpios[cnt];
		gpio = pdev->dev.platform_data;
		if (gpio->gpio == CC9M2443JS_PM_GPIO_DISABLED)
			continue;

		pdev->id = gpio->gpio;
		platform_device_register(pdev);
	}

	/* This is the main function for the PM of the Samsung-platforms */
	s3c2410_pm_init();
}
