/*
 * linux/include/asm-arm/arch-s3c2410/gpio.h
 *
 * S3C2410 GPIO wrappers for arch-neutral GPIO calls
 *
 * Written by Philipp Zabel <philipp.zabel@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __ASM_ARCH_S3C2410_GPIO_H
#define __ASM_ARCH_S3C2410_GPIO_H

#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/regs-gpio.h>
#include <asm-generic/gpio.h>


/* Functions for the S3C2443 (Luis Galdos) */
extern unsigned int s3c2443_gpio_read_porta(unsigned int pin);
extern void s3c2443_gpio_cfgpin(unsigned int pin, unsigned int function);
extern int s3c2443_gpio_get(struct gpio_chip *chip, unsigned gpio);
extern void s3c2443_gpio_set(struct gpio_chip *chip, unsigned gpio, int value);
extern int s3c2443_gpio_dir_input(struct gpio_chip *chip, unsigned gpio);
extern int s3c2443_gpio_dir_output(struct gpio_chip *chip, unsigned gpio, int value);
extern int s3c2443_gpio_getirq(unsigned int gpio);
extern int s3c2443_gpio_extpull(unsigned int pin, int pullup);


static inline int s3c_gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void s3c_gpio_free(unsigned gpio)
{
	return;
}

static inline int s3c_gpio_direction_input(struct gpio_chip *chip, unsigned gpio)
{
	s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_INPUT);
	return 0;
}

static inline int s3c_gpio_direction_output(struct gpio_chip *chip, unsigned gpio, int value)
{
	s3c2410_gpio_cfgpin(gpio, S3C2410_GPIO_OUTPUT);
	/* REVISIT can we write the value first, to avoid glitching? */
	s3c2410_gpio_setpin(gpio, value);
	return 0;
}


static inline void s3c_gpio_set(struct gpio_chip *chip, unsigned gpio, int value)
{
       s3c2410_gpio_setpin(gpio, value);
}


static inline int s3c_gpio_get(struct gpio_chip *chip, unsigned gpio)
{
	return s3c2410_gpio_getpin(gpio);
}


/* Call the functions from gpiolib.c (Luis G.) */
#define gpio_get_value(gpio)		__gpio_get_value(gpio)
#define gpio_set_value(gpio,value)	__gpio_set_value(gpio, value)

#ifdef CONFIG_CPU_S3C2400
#define gpio_to_irq(gpio)		s3c2400_gpio_getirq(gpio)
#elif CONFIG_CPU_S3C2443
#define gpio_to_irq(gpio)		s3c2443_gpio_getirq(gpio)
#else
#define gpio_to_irq(gpio)		s3c2410_gpio_getirq(gpio)
#endif

/* FIXME implement irq_to_gpio() */

#endif
