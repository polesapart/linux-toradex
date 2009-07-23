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
#include <linux/gpio.h>
#include <linux/platform_device.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <mach/gpio.h>

struct s3c2443_gpio_chip {
	struct gpio_chip	chip;
	u32			oldbase;
	u32			in_mask;
	u32			out_mask;
	u32			wakeup_mask;
};

#define to_s3c2443_gpio_chip(c)	container_of(c, struct s3c2443_gpio_chip, chip)

unsigned int s3c2443_gpio_read_porta(unsigned int pin)
{
	unsigned long gpacdh, gpacdl;
	unsigned long flags;
	unsigned long res, mask;
	int i;

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

/* Code coming from the U-Boot 1.1.6 */
void s3c2443_gpio_setpin(unsigned int pin, unsigned int to)
{
	unsigned long dat;
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin);

	if (pin < S3C2410_GPIO_BANKB) {
		dat = s3c2443_gpio_read_porta(pin);
		if (pin > S3C2410_GPA7) {
			dat &= ~(1 << ((offs - S3C2410_GPA7 - 1)  * 2));
			dat |= to << ((offs - S3C2410_GPA7 - 1) * 2);
			writel(dat, base + 0x04);
		} else {
			dat &= ~(1 << (offs * 2));
			dat |= to << (offs * 2);
			writel(dat, base);
		}
	} else
		s3c2410_gpio_setpin(pin, to);
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

/* Setup the UDP register of the S3C2443 ports */
void s3c2443_gpio_set_udp(unsigned int pin, int val)
{
	void __iomem *base = S3C24XX_GPIO_BASE(pin);
	unsigned long offs = S3C2410_GPIO_OFFSET(pin) * 2;
	unsigned long flags;
	unsigned long up;

	if (pin < S3C2410_GPIO_BANKB)
		return;

	local_irq_save(flags);

	up = __raw_readl(base + 0x08);
	up &= ~(3L << offs);
	up |= val << offs;
	__raw_writel(up, base + 0x08);

	local_irq_restore(flags);
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
		t  =   (regval  &  0x0000000f)  <<  28;
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

/* Gets the s3c2410 GPIO number from a s3c2443 GPIO number */
static inline unsigned int s3c2410_gpio_num(struct gpio_chip *chip, unsigned gpio)
{
	struct s3c2443_gpio_chip *port = to_s3c2443_gpio_chip(chip);

	return (gpio + port->oldbase);
}

static int s3c2443_gpio_to_irq(struct gpio_chip *chip, unsigned gpio)
{
	return s3c2410_gpio_getirq(s3c2410_gpio_num(chip, gpio));
}

int s3c2443_gpio_getirq(unsigned gpio)
{
	return s3c2410_gpio_getirq(gpio);
}

static int s3c2443_gpio_dir_input(struct gpio_chip *chip, unsigned gpio)
{
	struct s3c2443_gpio_chip *port = to_s3c2443_gpio_chip(chip);

	if ((1 << gpio) & port->in_mask) {
		s3c2410_gpio_cfgpin(s3c2410_gpio_num(chip, gpio),
				    S3C2410_GPIO_INPUT);
		return 0;
	}

	return -EINVAL;
}

static int s3c2443_gpio_dir_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	struct s3c2443_gpio_chip *port = to_s3c2443_gpio_chip(chip);

	if ((1 << gpio) & port->out_mask) {
		s3c2410_gpio_cfgpin(s3c2410_gpio_num(chip, gpio),
				    S3C2410_GPIO_OUTPUT);
		s3c2410_gpio_setpin(s3c2410_gpio_num(chip, gpio), value);
		return 0;
	}

	return -EINVAL;
}

static int s3c2443_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	return s3c2410_gpio_getpin(s3c2410_gpio_num(chip, gpio)) ? 1 : 0;
}

static void s3c2443_gpio_set(struct gpio_chip *chip, unsigned gpio, int value)
{
	s3c2410_gpio_setpin(s3c2410_gpio_num(chip, gpio), value);
}

static irqreturn_t s3c2443_wakeup_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int s3c2443_gpio_wakeup_conf(struct gpio_chip *chip, unsigned gpio, int enable)
{
	struct s3c2443_gpio_chip *port = to_s3c2443_gpio_chip(chip);
	int irq, ret = 0;

	/* Configure as wakeup interrupt if capable */
	if (!((1 << gpio) & port->wakeup_mask)) {
		pr_err("[ ERROR ] GPIO %i not wakeup capable\n", gpio);
		return -ENODEV;
	}

	irq = s3c2443_gpio_to_irq(chip, gpio);
	if (irq < 0) {
		pr_err("[ ERROR ] Couldn't get an IRQ for gpio %i\n", gpio);
		return -ENODEV;
	}

	if (enable) {
		ret = request_irq(irq, s3c2443_wakeup_irq,
				  IRQF_DISABLED | IRQF_TRIGGER_FALLING,
				  "s3c2443-wio", NULL);
		if (ret) {
			pr_err("[ ERROR ] Couldn't request IRQ %i\n", irq);
			goto exit_wakeup;
		}
		enable_irq_wake(irq);
	} else {
		disable_irq_wake(irq);
		free_irq(irq, NULL);
		s3c2443_gpio_dir_input(chip, gpio);
	}

 exit_wakeup:
	return ret;
}

static void s3c2443_set_pullupdown(struct gpio_chip *chip, unsigned gpio, int value)
{
	s3c2443_gpio_set_udp( s3c2410_gpio_num(chip, gpio), value);
}

#define S3C2443_GPIO_PORT(name,num,start,obase,				\
			  inmsk,outmsk,wakemsk)				\
	{								\
		.chip = {						\
			.label		  = "Port" #name,		\
			.direction_input  = s3c2443_gpio_dir_input, 	\
			.direction_output = s3c2443_gpio_dir_output,	\
			.get		  = s3c2443_gpio_get,		\
			.set		  = s3c2443_gpio_set,		\
			.to_irq           = s3c2443_gpio_to_irq,	\
			.wakeup_configure = s3c2443_gpio_wakeup_conf,	\
			.pullupdown	  = s3c2443_set_pullupdown,	\
			.base		  = start,			\
			.ngpio		  = num,			\
		},							\
		.oldbase                  = obase,			\
		.in_mask                  = inmsk,			\
		.out_mask                 = outmsk,			\
		.wakeup_mask              = wakemsk,			\
	}


static struct s3c2443_gpio_chip s3c2443_gpio_ports[] = {
	S3C2443_GPIO_PORT(A, 16,   0, S3C2410_GPIO_BANKA, 0x00000000, 0x0000ffff, 0),
	S3C2443_GPIO_PORT(B, 11,  16, S3C2410_GPIO_BANKB, 0x000003ff, 0x000003ff, 0),
	S3C2443_GPIO_PORT(C, 16,  27, S3C2410_GPIO_BANKC, 0x0000ffff, 0x0000ffff, 0),
	S3C2443_GPIO_PORT(D, 16,  43, S3C2410_GPIO_BANKD, 0x0000ffff, 0x0000ffff, 0),
	S3C2443_GPIO_PORT(E, 16,  59, S3C2410_GPIO_BANKE, 0x0000ffff, 0x0000ffff, 0),
	S3C2443_GPIO_PORT(F,  8,  75, S3C2410_GPIO_BANKF, 0x000000ff, 0x000000ff, 0xff),
	S3C2443_GPIO_PORT(G, 16,  83, S3C2410_GPIO_BANKG, 0x0000ffff, 0x0000ffff, 0xff),
	S3C2443_GPIO_PORT(H, 15,  99, S3C2410_GPIO_BANKH, 0x00007fff, 0x00007fff, 0),
	S3C2443_GPIO_PORT(J, 16, 114, S3C2440_GPIO_BANKJ, 0x0000ffff, 0x0000ffff, 0),
	S3C2443_GPIO_PORT(L, 15, 130, S3C2410_GPIO_BANKL, 0x00007fff, 0x00007fff, 0),
	S3C2443_GPIO_PORT(M,  2, 145, S3C2410_GPIO_BANKM, 0x00000003, 0x00000000, 0),
};

/* Gets the s3c2443 GPIO number from a s3c2410 GPIO number */
unsigned int s3c2443_gpio_num(unsigned gpio)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s3c2443_gpio_ports); i++) {
		if (S3C2410_BANKBASE(gpio) == s3c2443_gpio_ports[i].oldbase) {
			return (s3c2443_gpio_ports[i].chip.base + S3C2410_GPIO_OFFSET(gpio));
		}
	}

	return -1;
}

static int __init s3c2443_gpio_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(s3c2443_gpio_ports); i++)
		gpiochip_add(&s3c2443_gpio_ports[i].chip);

	return 0;
}
pure_initcall(s3c2443_gpio_init);

EXPORT_SYMBOL(s3c2443_gpio_getirq);
EXPORT_SYMBOL(s3c2443_gpio_extpull);
EXPORT_SYMBOL(s3c2443_gpio_cfgpin);
EXPORT_SYMBOL(s3c2443_gpio_set_udp);
EXPORT_SYMBOL(s3c2443_gpio_setpin);
EXPORT_SYMBOL(s3c2443_gpio_num);
