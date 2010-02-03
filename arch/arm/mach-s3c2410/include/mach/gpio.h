/* arch/arm/mach-s3c2410/include/mach/gpio.h
 *
 * Copyright (c) 2008 Simtec Electronics
 *      http://armlinux.simtec.co.uk/
 *      Ben Dooks <ben@simtec.co.uk>
 *
 * S3C2410 - GPIO lib support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#define gpio_get_value  __gpio_get_value
#define gpio_set_value  __gpio_set_value
#define gpio_cansleep   __gpio_cansleep
#define gpio_to_irq     __gpio_to_irq
#define gpio_set_pullupdown __gpio_set_pullupdown

#include <asm-generic/gpio.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>

void s3c2443_gpio_cfgpin(unsigned int pin, unsigned int function);
int s3c2443_gpio_extpull(unsigned int pin, int pullup);
int s3c2443_gpio_getirq(unsigned int gpio);
unsigned int s3c2443_gpio_read_porta(unsigned int pin);
void s3c2443_gpio_set_udp(unsigned int pin, int val);
void s3c2443_gpio_setpin(unsigned int pin, unsigned int to);
unsigned int s3c2443_gpio_num(unsigned gpio);
