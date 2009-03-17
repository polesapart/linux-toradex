/* -*- linux-c -*-
 * 
 * linux/arch/arm/mach-s3c2443/gpio.c
 *
 * Copyright (c) 2009 Digi International Spain
 *	Luis Galdos <luis.galdos@digi.com>
 *
 * http://www.digi.com/products/embeddedsolutions/connectcore9m.jsp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm-generic/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/gpio.h>

/* Available capabilities for the GPIOs */
enum gpio_caps {
	GPIO_CAPS_WAKEUP = (1 << 0),
	GPIO_CAPS_IRQ    = (1 << 1),	
};

/* For the internal handling */
struct s3c_gpio_t {
        unsigned int pin;
        unsigned int port;
	enum gpio_caps caps; /* Capabilities: wakeup, IRQ, etc. */
	unsigned long wakeup_trigger;
	unsigned long wakeup_ready;
	unsigned long wakeup_config;
	wait_queue_head_t wait_q;
	unsigned long triggered;
};

/* Create a standard GPIO */
#define S3C2443_GPIO(pi, pt)		   \
{ \
	.pin = pi, \
	.port = pt, \
}

/* Create a GPIO with IRQ capabilities */
#define S3C2443_GPIO_WAKEUP(pi, pt, iconf)	\
{ \
	.pin = pi,				   \
	.port = pt,			   \
	.caps =  GPIO_CAPS_IRQ  | GPIO_CAPS_WAKEUP,	   \
	.wakeup_trigger = IRQF_TRIGGER_FALLING, \
	.wakeup_config = iconf,		   \
}

static struct s3c_gpio_t s3c2443_gpios_table[] = {

	/* Port A */
	S3C2443_GPIO(393, S3C2410_GPA0),
	S3C2443_GPIO(373, S3C2410_GPA1),

        /* Port F */
	S3C2443_GPIO_WAKEUP(226, S3C2410_GPF0, S3C2410_GPF0_EINT0),
        S3C2443_GPIO(227, S3C2410_GPF1),
        S3C2443_GPIO(228, S3C2410_GPF2),
	S3C2443_GPIO_WAKEUP(232, S3C2410_GPF6, S3C2410_GPF6_EINT6),
	S3C2443_GPIO(233, S3C2410_GPF7),

        /* Port G */
	S3C2443_GPIO(242, S3C2410_GPG0),
	S3C2443_GPIO(243, S3C2410_GPG1),
	S3C2443_GPIO(244, S3C2410_GPG2),
	S3C2443_GPIO(245, S3C2410_GPG3),
	S3C2443_GPIO(246, S3C2410_GPG4),
	S3C2443_GPIO(247, S3C2410_GPG5),
        S3C2443_GPIO(248, S3C2410_GPG6),
	S3C2443_GPIO(249, S3C2410_GPG7),

	/* Port L */
	S3C2443_GPIO(166, S3C2443_GPL10),
	S3C2443_GPIO(165, S3C2443_GPL11),
	S3C2443_GPIO(164, S3C2443_GPL12),
	S3C2443_GPIO(163, S3C2443_GPL13),
	S3C2443_GPIO(162, S3C2443_GPL14),
};

static struct s3c_gpio_t *__get_gpio(int pin)
{
	int cnt;
	
        for (cnt = 0; cnt < ARRAY_SIZE(s3c2443_gpios_table); cnt++) {
                if (s3c2443_gpios_table[cnt].pin == pin)
                        return &s3c2443_gpios_table[cnt];
        }

	return NULL;
}

unsigned int s3c2443_gpio_read_porta(unsigned int pin)
{
	unsigned long gpacdh, gpacdl;
	unsigned long flags;
	unsigned long res, mask;
	int i;

	if (pin >= S3C2410_GPIO_BANKB)
		return s3c2410_gpio_getcfg(pin);

	/* Port A requieres special handling... */
	local_irq_save(flags);
	gpacdl = __raw_readl(S3C2410_GPACON);
	gpacdh = __raw_readl(S3C2410_GPACON + 0x4);
	local_irq_restore(flags);

	if (pin > S3C2410_GPA7) {
		gpacdl >>= 8;
		gpacdh >>= 8;
	}

	for (i=0, res = 0, mask = 0x1; i < 8; i++) {
		res |= (((gpacdh & mask) | ((gpacdl & mask) << 1)) << i);
		mask = mask << 1;
	}
	
	if (pin > S3C2410_GPA7)
		res |= (gpacdh & mask) << i;
	
	return res;
}

void s3c2443_gpio_cfgpin(unsigned int pin, unsigned int function)
{
	void __iomem *base;
	unsigned long mask;
	unsigned long con;
	unsigned long offs;
	unsigned long flags;

	if (pin >= S3C2410_GPIO_BANKB) {
		s3c2410_gpio_cfgpin(pin, function);
		return;
	}
  
	/* Port A requieres special handling... */
	con  = s3c2443_gpio_read_porta(pin);

	offs = S3C2410_GPIO_OFFSET(pin);
	if (pin > S3C2410_GPA7) 
		offs = offs - S3C2410_GPA7 - 1;

	mask = 1 << ((offs * 2) + 1);
	con &= ~mask;
	con |= (function << ((offs * 2) + 1));

	base = S3C24XX_GPIO_BASE(pin);
	if (pin > S3C2410_GPA7 )
		base += 0x4;

	local_irq_save(flags);
	__raw_writel(con, base);
	local_irq_restore(flags);
}

static unsigned int s3c2443_gpio2port(unsigned int pin)
{
        int cnt;

        for (cnt = 0; cnt < ARRAY_SIZE(s3c2443_gpios_table); cnt++) {
                if (s3c2443_gpios_table[cnt].pin == pin)
                        return s3c2443_gpios_table[cnt].port;
        }

        return 0;
}

/* By errors return a negative value */
int s3c2443_gpio_getirq(unsigned int gpio)
{
	unsigned int port;
	int retval;
	
	retval = -EINVAL;
	port = s3c2443_gpio2port(gpio);
	if (port)
		retval = s3c2410_gpio_getirq(port);

	return retval;
}

int s3c2443_gpio_dir_input(struct gpio_chip *chip, unsigned gpio)
{
	unsigned int port;

	port = s3c2443_gpio2port(gpio);
	s3c2410_gpio_cfgpin(port, S3C2410_GPIO_INPUT);
	return 0;
}

int s3c2443_gpio_dir_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned int port;

	port = s3c2443_gpio2port(gpio);
	s3c2410_gpio_cfgpin(port, S3C2410_GPIO_OUTPUT);
	s3c2410_gpio_setpin(port, value);
	return 0;
}

int s3c2443_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	unsigned int port;
	int retval;

	retval = -EINVAL;
	port = s3c2443_gpio2port(gpio);
	if (port)
		retval = s3c2410_gpio_getpin(port) ? 1 : 0;

	return retval;
}

void s3c2443_gpio_set(struct gpio_chip *chip, unsigned gpio, int value)
{
	unsigned int port;

	port = s3c2443_gpio2port(gpio);
	s3c2410_gpio_setpin(port, value);
}

/* Enable the pull-down of an external interrupt GPIO */
int s3c2443_gpio_extpull(unsigned int pin, int pullup)
{
	void __iomem *base;
	unsigned long regval;
	unsigned int offs;
	
	if (pin < S3C2410_GPF0 || pin > S3C2410_GPG15)
		return -ENODEV;

	if (pin >= S3C2410_GPG0)
		base = S3C24XX_EXTINT1;
	else
		base = S3C24XX_EXTINT0;

	offs = (S3C2410_GPIO_OFFSET(pin) * 4) + 3;

	/*
	 * First clear the control bit with the corresponding offset and then
	 * use the passed configuration value for setting the control bit
	 */
	pullup &= 0x1;

	/* There is a HW bug in the EXTINT0 register (code coming from the SMDK) */
	regval = __raw_readl(base);
	if (base == S3C24XX_EXTINT0) {
		unsigned int t;		
		t  =  (regval  &  0x0000000f)  <<  28;
		t  |=  (regval  &  0x000000f0)  <<  20;
		t  |=  (regval  &  0x00000f00)  <<  12;
		t  |=  (regval  &  0x0000f000)  <<  4;
		t  |=  (regval  &  0x000f0000)  >>  4;
		t  |=  (regval  &  0x00f00000)  >>  12;
		t  |=  (regval  &  0x0f000000)  >>  20;
		t  |=  (regval  &  0xf0000000)  >>  28;
		regval  =  t;
	}
	
	regval &= (~0x1UL << offs);
	regval |= (pullup << offs);
	__raw_writel(regval, base);
	return 0;
}

static irqreturn_t s3c2443_wakeup_irq(int irq, void *_gpio)
{
	struct s3c_gpio_t *gpio;

	gpio = (struct s3c_gpio_t *)_gpio;

	if (!gpio)
		goto exit_wakeup;

	/*
	 * Wakeup the waiting processes
	 * 
	 */
/* 	gpio->triggered = 1; */
/*         wake_up_interruptible(&gpio->wait_q); */
	
exit_wakeup:
	return IRQ_HANDLED;
}

int s3c2443_gpio_wakeup_configure(struct gpio_chip *chip, unsigned gpio, int enable)
{
	struct s3c_gpio_t *gp;
	int irq, retval;
		
	gp = __get_gpio(chip->base + gpio);
	if (!gp)
		return -ENODEV;
	
	/* Check if the GPIO is wakeup capable */
	if (!(gp->caps & GPIO_CAPS_WAKEUP)) {
		pr_err("[ ERROR ] GPIO %i not wakeup capable\n", gp->pin);
		return -ENODEV;
	}

	/* Sanity checks */
	if ((gp->wakeup_ready && enable) || (!gp->wakeup_ready && !enable)) {
		pr_err("[ ERROR ] GPIO %i already requested?\n", gp->pin);
		return -EINVAL;
	}

	irq = s3c2443_gpio_getirq(gpio);
	if (irq < 0) {
		pr_err("[ ERROR ] Couldn't get an IRQ for pin %i\n", gp->pin);
		return -ENODEV;
	}

	if (enable) {
		retval = request_irq(irq, s3c2443_wakeup_irq,
				     IRQF_DISABLED | gp->wakeup_trigger,
				     "s3c2443-wio", gp);
		if (retval) {
			pr_err("[ ERROR ] Couldn't request IRQ %i\n", irq);
			goto exit_wakeup;
		}

		enable_irq_wake(irq);
		s3c2410_gpio_cfgpin(gp->port, gp->wakeup_config);
		gp->wakeup_ready = 1;
	} else {
		disable_irq_wake(irq);
		free_irq(irq, gp);
		s3c2410_gpio_cfgpin(gp->port, S3C2410_GPIO_INPUT);
		gp->wakeup_ready = 0;
	}

	retval = 0;
	
exit_wakeup:
	return retval;
}

static struct gpio_chip s3c2443_gpios = {
	.label            = "s3c2443-gpios",
	.direction_input  = s3c2443_gpio_dir_input,
	.direction_output = s3c2443_gpio_dir_output,
	.get              = s3c2443_gpio_get,
	.set              = s3c2443_gpio_set,
	.wakeup_configure = s3c2443_gpio_wakeup_configure,
	.base             = 0,
	.ngpio            = 256,
};

void s3c2443_gpio_init(void)
{
	/* Now register the different chips */
	if (gpiochip_add(&s3c2443_gpios))
		pr_err("[ ERROR ] Registering the GPIOs for the CC9M2443JS\n");
}


EXPORT_SYMBOL(s3c2443_gpio_init);
EXPORT_SYMBOL(s3c2443_gpio_get);
EXPORT_SYMBOL(s3c2443_gpio_set);
EXPORT_SYMBOL(s3c2443_gpio_getirq);
EXPORT_SYMBOL(s3c2443_gpio_extpull);
EXPORT_SYMBOL(s3c2443_gpio_cfgpin);
EXPORT_SYMBOL(s3c2443_gpio_read_porta);
