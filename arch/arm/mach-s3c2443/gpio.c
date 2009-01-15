/* -*- linux-c -*-
 * 
 * linux/arch/arm/mach-s3c2443/gpio.c
 *
 * Copyright (c) 2007 Simtec Electronics
 * Copyright (c) 2008 Digi International Spain
 *	Ben Dooks <ben@simtec.co.uk>
 *	Luis Galdos <luis.galdos@digi.com>
 *
 * http://www.fluff.org/ben/smdk2443/
 * http://www.digi.com/products/embeddedsolutions/connectcore9m.jsp
 *
 * Thanks to Samsung for the loan of an SMDK2443
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

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <asm-generic/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/gpio.h>



struct s3c_gpio_t {
        unsigned int pin;
        unsigned int port;
};


static struct s3c_gpio_t s3c2443_gpios_table[] = {

	/* Port A */
        { 393, S3C2410_GPA0 },
        { 373, S3C2410_GPA1 },

        /* Port F */
        { 226, S3C2410_GPF0 },
        { 227, S3C2410_GPF1 },
        { 228, S3C2410_GPF2 },

        /* Port G */
	{ 242, S3C2410_GPG0 },
	{ 243, S3C2410_GPG1 },
	{ 244, S3C2410_GPG2 },
	{ 245, S3C2410_GPG3 },
	{ 246, S3C2410_GPG4 },
	{ 247, S3C2410_GPG5 },
        { 248, S3C2410_GPG6 },
	{ 249, S3C2410_GPG7 },

	/* Port L */
	{ 166, S3C2443_GPL10 },
	{ 165, S3C2443_GPL11 },
	{ 164, S3C2443_GPL12 },
	{ 163, S3C2443_GPL13 },
	{ 162, S3C2443_GPL14 },
};


static unsigned int s3c2443_gpio2port(unsigned int pin)
{
        int cnt;

        for (cnt=0; cnt < sizeof(s3c2443_gpios_table); cnt++) {
                if (s3c2443_gpios_table[cnt].pin == pin)
                        return s3c2443_gpios_table[cnt].port;
        }

        return 0;
}


int s3c2443_gpio_getirq(unsigned int gpio)
{
	unsigned int port;

	port = s3c2443_gpio2port(gpio);

	return s3c2410_gpio_getirq(port);
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


EXPORT_SYMBOL(s3c2443_gpio_get);
EXPORT_SYMBOL(s3c2443_gpio_set);
EXPORT_SYMBOL(s3c2443_gpio_getirq);
EXPORT_SYMBOL(s3c2443_gpio_extpull);
