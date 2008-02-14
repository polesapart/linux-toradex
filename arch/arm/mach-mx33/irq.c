/*
 *  Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>

/*!
 * Disable interrupt number "irq" in the TZIC
 *
 * @param  irq          interrupt source number
 */
static void mxc_mask_irq(unsigned int irq)
{
	int index, off;
	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, TZIC_ENCLEAR0 + (index << 2));
}

/*!
 * Enable interrupt number "irq" in the TZIC
 *
 * @param  irq          interrupt source number
 */
static void mxc_unmask_irq(unsigned int irq)
{
	int index, off;
	index = irq >> 5;
	off = irq & 0x1F;
	__raw_writel(1 << off, TZIC_ENSET0 + (index << 2));
}

static struct irqchip mxc_avic_chip = {
	.ack = mxc_mask_irq,
	.mask = mxc_mask_irq,
	.unmask = mxc_unmask_irq,
};

/*!
 * This function initializes the AVIC hardware and disables all the
 * interrupts. It registers the interrupt enable and disable functions
 * to the kernel for each interrupt source.
 */
void __init mxc_init_irq(void)
{
	int i;
	u32 reg;

	/* put the AVIC into the reset value with
	 * all interrupts disabled
	 */
	i = __raw_readl(TZIC_INTCNTL);

	__raw_writel(0x80010001, TZIC_INTCNTL);
	i = __raw_readl(TZIC_INTCNTL);
	__raw_writel(0x1f, TZIC_PRIOMASK);
	i = __raw_readl(TZIC_PRIOMASK);
	__raw_writel(0x02, TZIC_SYNCCTRL);
	i = __raw_readl(TZIC_SYNCCTRL);
	for (i = 0; i < 32; i++) {
		__raw_writel(0xFFFFFFFF, TZIC_INTSEC0 + i * 4);
	}
	/* disable all interrupts */
	for (i = 0; i < 32; i++) {
		__raw_writel(0xFFFFFFFF, TZIC_ENCLEAR0 + i * 4);
	}

	/* all IRQ no FIQ Warning :: No selection */

	for (i = 0; i < MXC_MAX_INT_LINES; i++) {
		set_irq_chip(i, &mxc_avic_chip);
		set_irq_handler(i, do_level_IRQ);
		set_irq_flags(i, IRQF_VALID);
	}

	printk(KERN_INFO "MXC IRQ initialized\n");
}
