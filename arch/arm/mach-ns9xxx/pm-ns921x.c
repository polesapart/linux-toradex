/*
 * arch/arm/mach-ns9xxx/pm-ns921x.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/gpio.h>

#include <asm/io.h>
#include <asm/leds.h>

#include <mach/processor.h>
#include <mach/regs-sys-ns921x.h>

#include "processor-ns921x.h"


#if defined(DEBUG)
#define REGMAP(reg) { .name = #reg, .addr = reg, }
static const struct {
	const char *name;
	void __iomem *addr;
} regmap[] = {
	REGMAP(SYS_ISA),
	REGMAP(SYS_ISR),
	REGMAP(SYS_CLOCK),
	REGMAP(SYS_POWER),
};

static void dump_regs(void)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(regmap); ++i)
		pr_debug("%s = 0x%08x\n", regmap[i].name,
				__raw_readl(regmap[i].addr));
}
#else
#define dump_regs() ((void)0)
#endif

static irqreturn_t ns921x_cpuwake_irq(int irq, void *dev_id)
{
	u32 power = __raw_readl(SYS_POWER);

	/* ack wake irq */
	__raw_writel(power | SYS_POWER_INTCLR, SYS_POWER);
	__raw_writel(power, SYS_POWER);

	return IRQ_HANDLED;
}

static int ns921x_pm_prepare(void)
{
	int ret;

	ret = request_irq(IRQ_NS921X_CPUWAKE, ns921x_cpuwake_irq, 0, "cpuwake",
			NULL);
	if (ret) {
		pr_debug("%s: err_%s -> %d\n",
				__func__, "request_irq_cpuwake", ret);
	}

	return ret;
}

static int ns921x_pm_enter(suspend_state_t state)
{
	u32 power = __raw_readl(SYS_POWER) | SYS_POWER_SLFRFSH;

	/* ack any possible wake up irq before going to sleep... */
	__raw_writel(power | SYS_POWER_INTCLR, SYS_POWER);
	__raw_writel(power, SYS_POWER);

	if (power & 0xf183f) {
		leds_event(led_idle_start);
		asm volatile("mcr p15, 0, %0, c7, c0, 4" : : "r" (0));
		leds_event(led_idle_end);
	} else {
		pr_warning("No wakeup source, not going to sleep (0x%08x)\n", 
			   __raw_readl(SYS_POWER));
		return -EDEADLK;
	}

	return 0;
}

static void ns921x_pm_finish(void)
{
	free_irq(IRQ_NS921X_CPUWAKE, NULL);
}

static struct platform_suspend_ops ns921x_pm_ops = {
	.valid = suspend_valid_only_mem,
	.prepare = ns921x_pm_prepare, 
	.enter = ns921x_pm_enter,
	.finish = ns921x_pm_finish, 
};

static int __init ns921x_pm_init(void)
{
	if (!processor_is_ns921x())
		return -ENODEV;

	suspend_set_ops(&ns921x_pm_ops);
	return 0;
}
arch_initcall(ns921x_pm_init);
