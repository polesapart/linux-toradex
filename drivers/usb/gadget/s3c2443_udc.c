/* -*- linux-c -*-
 *
 * drivers/usb/gadget/s3c24xx_udc.c
 *
 * Samsung S3C on-chip full/high speed USB device controllers
 *
 * $Id: s3c-udc-hs.c,v 1.26 2007/02/22 09:45:04 ihlee215 Exp $*
 *
 * Copyright (C) 2006 for Samsung Electronics
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "s3c2443_udc.h"
#include <linux/platform_device.h>
#include <linux/moduleparam.h>

/* @TODO: USB Device DMA support */
#define RX_DMA_MODE 0
#define TX_DMA_MODE 0

#if 0
#define DEBUG_S3C2443_UDC
#endif

#define printk_err(fmt, args...)                printk(KERN_ERR "[ ERROR ] s3c2443-udc: " fmt, ## args)
#define printk_info(fmt, args...)               printk(KERN_INFO "s3c2443-udc: " fmt, ## args)

#ifdef DEBUG_S3C2443_UDC
#define printk_debug(fmt,args...)		printk(KERN_DEBUG "s3c2443-udc: %s() " fmt, __func__ , ## args)
#else
#define printk_debug(fmt,args...)		do { } while(0)
#endif

/*
 * This macro enables the debug messages when the driver is going to access to the
 * internal queue of the IN-endpoints
 */
#if 0
#define DEBUG_S3C2443_UDC_QUEUE
#endif

/* Some driver infos */
#define	DRIVER_DESC		                "S3C2443 Dual-speed USB Device"
#define DRIVER_NAME                             "s3c2443_udc"
#define	DRIVER_BUILD_TIME	                 __TIME__
#define	DRIVER_BUILD_DATE	                 __DATE__

#define IOMEMSIZE(s)		                (s->end - s->start + 1)

/* Internal variables */
struct s3c24xx_udc *the_controller;
static const char driver_desc[] = DRIVER_DESC;
static const char ep0name[] = "ep0-control";

/* Max packet sizes */
static u32 ep0_fifo_size = 64;
static u32 ep_fifo_size =  512;
static u32 ep_fifo_size2 = 1024;

/* Internal functions */
static int s3c24xx_udc_ep_enable(struct usb_ep *ep,
				 const struct usb_endpoint_descriptor *);
static int s3c24xx_udc_ep_disable(struct usb_ep *ep);
static struct usb_request *s3c24xx_udc_alloc_request(struct usb_ep *ep, gfp_t gfp_flags);
static void s3c24xx_udc_free_request(struct usb_ep *ep, struct usb_request *);
static int s3c24xx_udc_queue(struct usb_ep *ep, struct usb_request *, gfp_t gfp_flags);
static int s3c24xx_udc_dequeue(struct usb_ep *ep, struct usb_request *);
static int s3c24xx_udc_set_halt(struct usb_ep *ep, int);
static int s3c24xx_udc_fifo_status(struct usb_ep *ep);
static void s3c24xx_udc_fifo_flush(struct usb_ep *ep);
static void s3c24xx_udc_ep0_kick(struct s3c24xx_udc *udc, struct s3c_ep *ep);
static void s3c24xx_handle_ep0(struct s3c24xx_udc *udc);
static void done(struct s3c_ep *ep, struct s3c_request *req, int status);
static void stop_activity(struct s3c24xx_udc *dev, struct usb_gadget_driver *driver);
static int s3c24xx_udc_enable(struct s3c24xx_udc *udc);
static void s3c24xx_udc_set_address(struct s3c24xx_udc *dev, unsigned char address);
static void reconfig_usbd(struct s3c24xx_udc *udc);

static int s3c24xx_udc_write_fifo(struct s3c_ep *ep, struct s3c_request *req);


static inline struct s3c24xx_udc *gadget_to_udc(struct usb_gadget *gadget)
{
        return container_of(gadget, struct s3c24xx_udc, gadget);
}


static spinlock_t regs_lock = SPIN_LOCK_UNLOCKED;

/* Read access to one of the indexed registers */
static inline ulong usb_read(struct s3c24xx_udc *udc, ulong port, u8 ind)
{
	ulong retval;
	
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	retval = readl(udc->base + port);
	spin_unlock(&regs_lock);
	return retval;
}

/* Write access to one of the indexed registers */
static inline void usb_write(struct s3c24xx_udc *udc, ulong val, ulong port, u8 ind)
{
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	writel(val, udc->base + port);
	spin_unlock(&regs_lock);
}

static inline void usb_set(struct s3c24xx_udc *udc, ulong val, ulong port, u8 ind)
{
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	writel(readl(udc->base + port) | val, udc->base + port);
	spin_unlock(&regs_lock);
}

static inline void usb_clear(struct s3c24xx_udc *udc, ulong val, ulong port, u8 ind)
{
	spin_lock(&regs_lock);
	writel(ind, udc->base + S3C24XX_UDC_IR_REG);
	writel(readl(udc->base + port) & ~val, udc->base + port);
	spin_unlock(&regs_lock);
}

/* Return a value different than zero if the EP is enabled */
static inline int s3c24xx_ep_enabled(struct s3c24xx_udc *udc, int epnr)
{
	ulong regval;

	regval = readl(udc->base + S3C24XX_UDC_EIER_REG);
	return (regval & (1 << epnr));
}

/* Enable/disable the interrupt of the passed EP-number */
static inline void s3c24xx_ep_irq_enable(struct s3c24xx_udc *udc, int epnr, int enable)
{
	ulong eier;

	eier = readl(udc->base + S3C24XX_UDC_EIER_REG);
	if (enable)
		eier |= (1 << epnr);
	else
		eier &= ~(1 << epnr);
	writel(eier, udc->base + S3C24XX_UDC_EIER_REG);
}

static struct usb_ep_ops s3c24xx_ep_ops = {
	.enable = s3c24xx_udc_ep_enable,
	.disable = s3c24xx_udc_ep_disable,

	.alloc_request = s3c24xx_udc_alloc_request,
	.free_request = s3c24xx_udc_free_request,

	.queue = s3c24xx_udc_queue,
	.dequeue = s3c24xx_udc_dequeue,

	.set_halt = s3c24xx_udc_set_halt,
	.fifo_status = s3c24xx_udc_fifo_status,
	.fifo_flush = s3c24xx_udc_fifo_flush,
};

/*
 * Function for writing from the request buffer into the EP-FIFO
 * The function updates the internal actual length of the USB-request for a possible
 * next transfer of the same request.
 * The return value is the number of remaining bytes in the request. If the return
 * value is equal zero, then there is no more data to process in the request
 * (Luis Galdos)
 */
static inline int s3c24xx_udc_write_packet(struct s3c_ep *ep, struct s3c_request *req)
{
	u16 *buf;
	int length, count;
	u32 fifo = ep->fifo;
	struct s3c24xx_udc *udc;
	int max, remaining, epnr;
	u8 *ptr;
	
	/* @XXX: Need some sanity checks (Luis Galdos) */
	udc = ep->dev;
	max = ep->ep.maxpacket;
	epnr = ep_index(ep);

	/* Get the number of remaining bytes */
	remaining = req->req.length - req->req.actual;
	if (!remaining) {
		printk(KERN_DEBUG "EP%i: Zero bytes remaining. Skipping.", epnr);
		length = remaining;
		goto exit_write_packet;
	}

	/* Use first a u8 pointer for obtaining the correct buffer address */
	ptr = req->req.buf + req->req.actual;
	buf = (u16 *)ptr;
	prefetch(buf);

	/* Only send the maximal allowed number of bytes */
	length = min(remaining, max);
	req->req.actual += length;

	/* First write the number of bytes to transfer, and then fill the FIFO */
	usb_write(udc, length, S3C24XX_UDC_BWCR_REG, epnr);
	for (count = 0; count < length; count += 2)
		writel(*buf++, udc->base + fifo);
	
	/* Return the number of remaining bytes of the passed request */
exit_write_packet:
	return (remaining - length);
}

/*
 * Disable the controller by resetting the PHY for informing the USB-host
 * that the device was disconnected
 * (Luis Galdos)
 */
static void s3c24xx_udc_disable(struct s3c24xx_udc *udc)
{
	ulong regval;
	
	printk_debug("UDC disable called\n");

	/* Disable the EP interrupts */
	writel(0, udc->base + S3C24XX_UDC_EIER_REG);
	writel(0xff, udc->base + S3C24XX_UDC_EIR_REG);

	/* Clear all the status bits of the EP0 and flush it */
	writel(S3C24XX_UDC_EP0SR_RSR | S3C24XX_UDC_EP0SR_TST |
	       S3C24XX_UDC_EP0SR_SHT | S3C24XX_UDC_EP0SR_LWO,
	       udc->base + S3C24XX_UDC_EP0SR_REG);
	writel(0, udc->base + S3C24XX_UDC_EP0CR_REG);
	usb_write(udc, S3C24XX_UDC_ECR_FLUSH, S3C24XX_UDC_ECR_REG, 0);

	/* Unset the function address */
	s3c24xx_udc_set_address(udc, 0);

	udc->ep0state = WAIT_FOR_SETUP;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->usb_address = 0;

	/* Clear all the status bits from the system status register */
	regval = S3C24XX_UDC_INT_RESET | S3C24XX_UDC_INT_SUSPEND |
		S3C24XX_UDC_INT_RESUME | S3C24XX_UDC_INT_SDE |
		S3C24XX_UDC_SSR_TBM | S3C24XX_UDC_INT_VBUSON |
		S3C24XX_UDC_SSR_VBUSOFF;
	writel(regval, udc->base + S3C24XX_UDC_SSR_REG);
	
	/* Reset the USB-function */
	regval = readl(S3C2443_URSTCON);
	writel(regval | S3C2443_URSTCON_PHY, S3C2443_URSTCON);

	/* PHY power disable */
#if defined(DISABLED_UDC_CODE)
	regval = readl(S3C2443_PWRCFG);
	writel(regval | S3C2443_PWRCFG_USBPHY_ON, S3C2443_PWRCFG);
#endif
	regval = 0;
	writel(regval, S3C2443_PHYPWR);

	regval = readl(S3C2443_PWRCFG);
	regval &= ~S3C2443_PWRCFG_USBPHY_ON;
	writel(regval, S3C2443_PWRCFG);
}

/*
 * Function for sending request data to the FIFO
 * This function uses the EP-lock for avoiding the wrong queue order of the packets
 * that are incoming from the Gadget-driver
 * (Luis Galdos)
 */
static void s3c24xx_udc_epin_tasklet_func(unsigned long data)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	int retval;
	ulong esr;
	struct s3c24xx_udc *udc;

	ep = (struct s3c_ep *)data;
	if (!ep) {
		printk_err("Invalid EP pointer. Aborting %s\n", __func__);
		return;
	}
	
	spin_lock(&ep->lock);

	udc = ep->dev;
	esr = usb_read(udc, S3C24XX_UDC_ESR_REG, ep_index(ep));

	/*
	 * Paranoic sanity check: If the FIFO has still a packet, then abort this
	 * tasklet and wait for the call from the interrupt handler (TPS)
	 */
	if (S3C24XX_UDC_ESR_PSIFNR(esr) == 2) {
		printk_debug("The FIFO seems to have still a packet\n");
		goto exit_unlock;
	}
	
	/* Check if there is a pending request for us */
	if (list_empty(&ep->queue))
		goto exit_unlock;

	/* Get the next request from the queue of the endpoint */
	req = list_entry(ep->queue.next, struct s3c_request, queue);
	if (!req) {
		printk_err("EP%i: NULL request pointer.\n", ep_index(ep));
		goto exit_unlock;
	}

#if defined(DEBUG_S3C2443_UDC_QUEUE)
	{
		u8 ch1, ch2;
		int len;
		u8 *ptr = (u8 *)req->req.buf;
		len = req->req.length;
		ch1 = *ptr;
		ch2 = *(ptr + len - 1);
		printk(KERN_DEBUG "%p: len=%i, 0x%02x ... 0x%02x\n", req, len, ch1, ch2);
	}
#endif	
	retval = s3c24xx_udc_write_fifo(ep, req);
	
 exit_unlock:
	spin_unlock(&ep->lock);
}

/*
 * Restart the UDC and the corresponding resources (tasklet, queues, etc.)
 * (Luis Galdos)
 */
static void s3c24xx_udc_reinit(struct s3c24xx_udc *udc)
{
	u32 i;

	/* device/ep0 records init */
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);
	udc->ep0state = WAIT_FOR_SETUP;

	/* basic endpoint records init */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &udc->ep[i];

		if (i != 0)
			list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);

		ep->desc = 0;
		ep->stopped = 0;
		INIT_LIST_HEAD(&ep->queue);
		ep->pio_irqs = 0;
	}

	/* the rest was statically initialized, and is read-only */
}

#define BYTES2MAXP(x)	(x / 8)
#define MAXP2BYTES(x)	(x * 8)


/*
 * Until it's enabled, this UDC should be completely invisible
 * to any USB host.
 */
static int s3c24xx_udc_enable(struct s3c24xx_udc *udc)
{
	int regval;

	printk_debug("UDC enable called\n");

	/* if reset by sleep wakeup, control the retention I/O cell */
/* 	if (__raw_readl(S3C_RSTSTAT) & 0x8) */
/* 		__raw_writel(__raw_readl(S3C_RSTCON)|(1<<16), S3C_RSTCON); */

	/* USB Port is Normal mode */
	regval = __raw_readl(S3C24XX_MISCCR);
	regval &= ~S3C2410_MISCCR_USBSUSPND0;
	__raw_writel(regval, S3C24XX_MISCCR);

	/* PHY power enable */
	regval = __raw_readl(S3C2443_PWRCFG);
	regval |= S3C2443_PWRCFG_USBPHY_ON;
	__raw_writel(regval, S3C2443_PWRCFG);

	/*
	 * USB device 2.0 must reset like bellow,
	 * 1st phy reset and after at least 10us, func_reset & host reset
	 * phy reset can reset bellow registers.
	 */
	/* PHY 2.0 S/W reset */
	regval = S3C2443_URSTCON_PHY;
	__raw_writel(regval, S3C2443_URSTCON);
	udelay(20); /* phy reset must be asserted for at 10us */
	
	/* Function 2.0, Host 1.1 S/W reset */
	regval = S3C2443_URSTCON_FUNC;
	__raw_writel(regval, S3C2443_URSTCON);
	udelay(1);
	__raw_writel(0x00, S3C2443_URSTCON);

	/* 48Mhz,Oscillator,External X-tal,device */
#if defined(DISABLED_UDC_CODE)
	regval = __raw_readl(S3C2443_PHYCTRL);
	regval |= (S3C2443_PHYCTRL_EXTCLK_OSCI | S3C2443_PHYCTRL_INTPLL_USB);
	regval = 0;
	__raw_writel(regval, S3C2443_PHYCTRL);
#endif
	
	/* 48Mhz clock on ,PHY2.0 analog block power on
	 * XO block power on,XO block power in suspend mode,
	 * PHY 2.0 Pll power on ,suspend signal for save mode disable
	 */
	regval = S3C2443_PHYPWR_COMMON_ON;
	__raw_writel(regval, S3C2443_PHYPWR);

	/* D+ pull up disable(VBUS detect), USB2.0 Function clock Enable,
	 * USB1.1 HOST disable, USB2.0 PHY test enable
	 */
	regval = __raw_readl(S3C2443_UCLKCON);
	regval |= S3C2443_UCLKCON_FUNC_ENABLE | S3C2443_UCLKCON_TFUNC_ENABLE;
	__raw_writel(regval, S3C2443_UCLKCON);

	reconfig_usbd(udc);

	udc->gadget.speed = USB_SPEED_UNKNOWN;
	
	/*
	 * So, now enable the pull up, USB2.0 Function clock Enable,
	 * USB1.1 HOST disable,USB2.0 PHY test enable
	 */
	regval = __raw_readl(S3C2443_UCLKCON);
#if defined(DISABLED_UDC_CODE)
	regval |= S3C2443_UCLKCON_VBUS_PULLUP | S3C2443_UCLKCON_FUNC_ENABLE |
		S3C2443_UCLKCON_TFUNC_ENABLE;
#else
	regval |= S3C2443_UCLKCON_VBUS_PULLUP | S3C2443_UCLKCON_FUNC_ENABLE;
#endif
	__raw_writel(regval, S3C2443_UCLKCON);

	return 0;
}

/*
 * Function called from the Gadget-drivers for registering a new profile.
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	struct s3c24xx_udc *udc = the_controller;
	int retval;

	if (!driver)
		return -EINVAL;
	
	printk_debug("%s\n", driver->driver.name);

	if (!driver ||
	    (driver->speed != USB_SPEED_FULL && driver->speed != USB_SPEED_HIGH) ||
	    !driver->bind || !driver->unbind || !driver->disconnect || !driver->setup)
		return -EINVAL;
	
	if (!udc) {
		printk_err("No UDC-controller probed? Aborting.\n");
		return -ENODEV;
	}

	if (udc->driver) {
		printk_err("UDC already in use by '%s'\n", udc->driver->driver.name);
		return -EBUSY;
	}

	/* first hook up the driver ... */
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;

	retval = device_add(&udc->gadget.dev);
	if (retval) {
		printk_err("Couldn't add the new Gadget device (%i)\n", retval);
		goto err_exit;
	}

	retval = driver->bind(&udc->gadget);
	if (retval) {
		printk("%s: bind to driver %s --> error %d\n", udc->gadget.name,
		       driver->driver.name, retval);
		goto err_del_device;
	}

	enable_irq(IRQ_USBD);

	/*
	 * If a host was already detected, then only call the UDC enable function,
	 * otherwise check over the configured GPIO if a host is connected.
	 */
	if (udc->vbus)
		s3c24xx_udc_enable(udc);
	else {
		struct s3c2410_udc_mach_info *info;
		unsigned long state, iocfg;

		info = udc->mach_info;

		iocfg = s3c2410_gpio_getcfg(info->vbus_pin);
		s3c2410_gpio_cfgpin(info->vbus_pin, S3C2410_GPIO_INPUT);
		state = s3c2410_gpio_getpin(info->vbus_pin);
		s3c2410_gpio_cfgpin(info->vbus_pin, iocfg);

		if (info->vbus_pin_inverted)
			state = !state;

		if (state)
			s3c24xx_udc_enable(udc);
	}
	
	return 0;

 err_del_device:
	device_del(&udc->gadget.dev);
	
 err_exit:
	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;
	return retval;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

/*
 * Unregister entry point for the peripheral controller driver.
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct s3c24xx_udc *udc = the_controller;
	unsigned long flags;

	if (!udc)
		return -ENODEV;
	if (!driver || driver != udc->driver)
		return -EINVAL;

	spin_lock_irqsave(&udc->lock, flags);
	udc->driver = NULL;
	stop_activity(udc, driver);
	spin_unlock_irqrestore(&udc->lock, flags);

	driver->unbind(&udc->gadget);
	device_del(&udc->gadget.dev);

	disable_irq(IRQ_USBD);

	printk_info("Unregistered gadget driver '%s'\n", driver->driver.name);

	/* Disable the pull-up for informing the host about the removed driver */
	s3c24xx_udc_disable(udc);

	/* Reset the delay of the IN-handling */
	/* enable_epin_udelay = 0; */
	
	return 0;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);

/*
 * Write request to FIFO (max write == maxp size)
 * Return:  0 = still running, 1 = completed, negative = errno
 */
static int s3c24xx_udc_write_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	unsigned count;
	int is_last;

	count = s3c24xx_udc_write_packet(ep, req);

	/* Last packet is usually short (or a zlp) */
	is_last = (count == 0) ? (1) : (0);
	
	printk_debug("Wrote %s %d bytes%s%s req %p %d/%d\n", ep->ep.name, count,
		     is_last ? "/L" : "", is_short ? "/S" : "",
		     req, req->req.actual, req->req.length);
	
	/* Requests complete when all IN data is in the FIFO */
	if (is_last) {
		if (!ep_index(ep)) {
			printk_err("Error EP0 must not come here!\n");
			BUG();
		}
		done(ep, req, 0);
		return 1;
	}

	return 0;
}

/*
 * Read to request from FIFO (max read == bytes in fifo)
 * Return: 0 = still running, 1 = completed, negative = errno
 */
static int s3c24xx_udc_read_fifo(struct s3c_ep *ep, struct s3c_request *req)
{
	u32 csr;
	u16 *buf;
	unsigned bufferspace, count, count_bytes, is_short = 0;
	u32 fifo = ep->fifo;
	struct s3c24xx_udc *udc;

	udc = ep->dev;
	csr = usb_read(udc, S3C24XX_UDC_ESR_REG, ep_index(ep));

	/*
	 * If the FIFO is empty then return zero, so that a caller, like the queue-
	 * function, doesn't fail. Returning zero means that the request is not done
	 * and it can be added to the internal EP-request queue
	 * (Luis Galdos)
	 */
	if (!(csr & S3C24XX_UDC_ESR_RPS)) {
		printk_debug("EP%i: No packet to read.\n", ep_index(ep));
		return 0;
	}

	buf = req->req.buf + req->req.actual;
	prefetchw(buf);

	/* Calculate the current buffer space */
	bufferspace = req->req.length - req->req.actual;

	/* Read all bytes from this packet */
	count = usb_read(udc, S3C24XX_UDC_BRCR_REG, ep_index(ep));
	if (csr & S3C24XX_UDC_ESR_LWO)
		count_bytes = count * 2 - 1;
	else
		count_bytes = count * 2;

	/* Update the actual variable of the request */
	req->req.actual += min(count_bytes, bufferspace);

	is_short = (count_bytes < ep->ep.maxpacket);
	printk_debug("Read %s, %d bytes%s req %p %d/%d\n",
		     ep->ep.name, count_bytes,
		     is_short ? "/S" : "", req, req->req.actual, req->req.length);

	while (likely(count-- != 0)) {
		u16 byte = (u16)readl(udc->base + fifo);

		/*
		 * If there is no more space in the request-buffer, then continue
		 * reading from the FIFO and return with the done value
		 * (Luis Galdos)
		 */
		if (unlikely(bufferspace == 0)) {
			req->req.status = -EOVERFLOW;
			is_short = 1;
		} else {
			*buf++ = byte;
			bufferspace--;
		}
	}
	
	/*
	 * If the complete FIFO-data passed into the request-buffer, then
	 * return one, otherwise skip the return
	 * (Luis Galdos)
	 */
	if (is_short || req->req.actual == req->req.length) {
		done(ep, req, 0);
		return 1;
	}

	/* finished that packet.  the next one may be waiting... */
	return 0;
}

/*
 * Retire a request from the internal EP-queue and call the complete
 * function of the Gadget-request
 * (Luis Galdos)
 */
static void done(struct s3c_ep *ep, struct s3c_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	list_del_init(&req->queue);

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN) {
		printk_debug("EP%i: done req %p | stat %d | actual %u | length %u\n",
			     ep_index(ep),
			     &req->req, status, req->req.actual, req->req.length);
	}

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;

	/*
	 * We must unlock the queue of the EP at this place, then the Gadget-driver
	 * probably will try to enqueue a new request by calling our queue-function.
	 * (Luis Galdos)
	 */
	spin_unlock(&ep->lock);
	spin_unlock(&ep->dev->lock);
	req->req.complete(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);
	spin_lock(&ep->lock);
	
	ep->stopped = stopped;
}

/* Nuke/dequeue all the requested transfers */
void nuke(struct s3c_ep *ep, int status)
{
	struct s3c_request *req;

	printk_debug("EP%i: Nuke function called\n", ep_index(ep));
	
	/* called with irqs blocked */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct s3c_request, queue);
		done(ep, req, status);
	}
}

/*
 * This function handles the IN-operations of the endpoints different than zero
 */
static void s3c24xx_udc_in_epn(struct s3c24xx_udc *udc, u32 epnr)
{
	ulong esr, handled;
	struct s3c_ep *ep = &udc->ep[epnr];

	handled = 0;
	esr = usb_read(udc, S3C24XX_UDC_ESR_REG, epnr);

	/* ACK the function stall condition */
	if (esr & S3C24XX_UDC_ESR_FSC) {
		usb_set(udc, S3C24XX_UDC_ESR_FSC, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* The flush operation generates an interrupt too */
	if (esr & S3C24XX_UDC_ESR_FFS) {
		usb_set(udc, S3C24XX_UDC_ESR_FFS, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	/* Underflow check */
	if (esr & S3C24XX_UDC_ESR_FUDR) {
		printk_err("EP%i: Underflow detected\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FUDR, S3C24XX_UDC_ESR_REG, epnr);
	}

	/* Overflow check */
	if (esr & S3C24XX_UDC_ESR_FOVF) {
		printk_err("EP%i: Overflow detected\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FOVF, S3C24XX_UDC_ESR_REG, epnr);
	}
	
	/* By successed transfer of a IN-packet then only schedule the tasklet */
	if (esr & S3C24XX_UDC_ESR_TPS) {
		spin_lock(&ep->lock);
		usb_set(udc, S3C24XX_UDC_ESR_TPS, S3C24XX_UDC_ESR_REG, epnr);		
		spin_unlock(&ep->lock);
		tasklet_hi_schedule(&ep->in_tasklet);
		handled = 1;
	}

	if (!handled)
		printk_err("EP%i: Unhandled IRQ (ESR 0x%04lx)\n", epnr, esr);
}

/*
 * The below function is called when data was received with an OUT-transaction
 */
static void s3c24xx_udc_out_epn(struct s3c24xx_udc *udc, u32 ep_idx)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	ulong esr, epnr, handled;

	ep = &udc->ep[ep_idx];
	epnr = ep_index(ep);
	if (epnr != ep_idx) {
		printk_err("Invalid EP structure (%lu) or index (%u) passed\n",
			   epnr, ep_idx);
		return;
	}

	/* Read the status register of the EP */
	handled = 0;	
	esr = usb_read(udc, S3C24XX_UDC_ESR_REG, epnr);
	printk_debug("EP%lu: Status reg 0x%08lx\n", epnr, esr);

	if (unlikely(!(ep->desc))) {
		printk_err("No descriptor for EP%lu\n", epnr);
		return;
	}

	if (esr & S3C24XX_UDC_ESR_FSC) {
		printk_debug("EP%lu stall sent\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FSC, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}

	if (esr & S3C24XX_UDC_ESR_FFS) {
		printk_debug("EP%lu FIFO flush\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FFS, S3C24XX_UDC_ESR_REG, epnr);
		handled = 1;
	}
	
	/* RX means we have received some data over an OUT-transaction */
	if (esr & S3C24XX_UDC_ESR_RPS) {
		int retval;
		ulong packets;

		/*
		 * The read-fifo function returns zero if there is
		 * additional data in the FIFO. In that case read once
		 * again from the FIFO
		 */
		packets = (esr & S3C24XX_UDC_ESR_PSIF_TWO) ? 2 : 1;
		while (packets--) {

			req = (list_empty(&ep->queue)) ? NULL :
				list_entry(ep->queue.next, struct s3c_request, queue);

			/*
			 * If we dont have a request for the received data, then only
			 * break the loop and return without flushing the FIFO.
			 */
			if (unlikely(!req)) {
				printk_debug("EP%lu: Null USB-request found\n", epnr);
				break;
			}
						
			retval = s3c24xx_udc_read_fifo(ep, req);
			if (retval < 0) {
				printk_err("EP%lu: FIFO read (%i)\n", epnr, retval);
				break;
			}
		}

		handled = 1;
	}

	/* Handlings for the overflow and underrun */
	if (esr & S3C24XX_UDC_ESR_FOVF) {
		printk_err("EP%lu FIFO overflow\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FOVF, S3C24XX_UDC_ESR_REG, epnr);
	}

	if (esr & S3C24XX_UDC_ESR_FUDR) {
		printk_err("EP%lu FIFO underrun\n", epnr);
		usb_set(udc, S3C24XX_UDC_ESR_FUDR, S3C24XX_UDC_ESR_REG, epnr);
	}
	
	/*
	 * Check if the function was handled, otherwise only uses a debug message
	 * for informing about the error
	 */
	if (!handled) {
		printk_debug("EP%lu: Unhandled OUT | ESR 0x%08lx.\n", epnr, esr);
	}
}

static void stop_activity(struct s3c24xx_udc *udc,
			  struct usb_gadget_driver *driver)
{
	int i;

	/* don't disconnect drivers more than once */
	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;

	/* prevent new request submissions, kill any outstanding requests  */
	for (i = 0; i < S3C_MAX_ENDPOINTS; i++) {
		struct s3c_ep *ep = &udc->ep[i];
		ep->stopped = 1;
		nuke(ep, -ESHUTDOWN);
	}

	/* report disconnect; the driver is already quiesced */
	if (driver) {
		spin_unlock(&udc->lock);
		driver->disconnect(&udc->gadget);
		spin_lock(&udc->lock);
	}

	/* re-init driver-visible data structures */
	s3c24xx_udc_reinit(udc);
}

static void reconfig_usbd(struct s3c24xx_udc *udc)
{
	struct s3c_ep *ep;
	int cnt;
	unsigned long edr;

	/*
	 * Configure the endpoints depending on the defined structure which
	 * will be used by the gadget-drivers (see below)
	 */
	edr = 0;
	for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &the_controller->ep[cnt];
		if (ep->bEndpointAddress & USB_DIR_IN)
			edr |= (1 << cnt);
	}
	writel(edr, udc->base + S3C24XX_UDC_EDR_REG);

	/* Only enable the EP0 */
	writel(0x1, udc->base + S3C24XX_UDC_EIER_REG);
	
 	writel(0x0, udc->base + S3C24XX_UDC_TR_REG);

	/* error interrupt enable, 16bit bus, Little format,
	 * suspend&reset enable
	 */
	writel(S3C24XX_UDC_EIE_EN |
	       S3C24XX_UDC_RRD_EN |
	       S3C24XX_UDC_SUS_EN |
	       S3C24XX_UDC_RST_EN,		
	       udc->base + S3C24XX_UDC_SCR_REG);
	
	writel(0x0000, udc->base + S3C24XX_UDC_EP0CR_REG);

	writel(0, udc->base + S3C24XX_UDC_IR_REG);
}


static void s3c24xx_set_max_pktsize(struct s3c24xx_udc *udc, enum usb_device_speed speed)
{
	if (speed == USB_SPEED_HIGH) {
		ep0_fifo_size = 64;
		ep_fifo_size = 512;
		ep_fifo_size2 = 1024;
		udc->gadget.speed = USB_SPEED_HIGH;
	} else {
		ep0_fifo_size = 64;
		ep_fifo_size = 64;
		ep_fifo_size2 = 64;
		udc->gadget.speed = USB_SPEED_FULL;
	}

	udc->ep[0].ep.maxpacket = ep0_fifo_size;
	udc->ep[1].ep.maxpacket = ep_fifo_size;
	udc->ep[2].ep.maxpacket = ep_fifo_size;
	udc->ep[3].ep.maxpacket = ep_fifo_size;
	udc->ep[4].ep.maxpacket = ep_fifo_size;
	udc->ep[5].ep.maxpacket = ep_fifo_size2;
	udc->ep[6].ep.maxpacket = ep_fifo_size2;
	udc->ep[7].ep.maxpacket = ep_fifo_size2;
	udc->ep[8].ep.maxpacket = ep_fifo_size2;
	
	usb_write(udc, ep0_fifo_size, (u32) S3C24XX_UDC_MAXP_REG, 0);
	usb_write(udc, ep_fifo_size, (u32) S3C24XX_UDC_MAXP_REG, 1);
	usb_write(udc, ep_fifo_size, (u32) S3C24XX_UDC_MAXP_REG, 2);
	usb_write(udc, ep_fifo_size, (u32) S3C24XX_UDC_MAXP_REG, 3);
	usb_write(udc, ep_fifo_size, (u32) S3C24XX_UDC_MAXP_REG, 4);
	usb_write(udc, ep_fifo_size2, (u32) S3C24XX_UDC_MAXP_REG, 5);
	usb_write(udc, ep_fifo_size2, (u32) S3C24XX_UDC_MAXP_REG, 6);
	usb_write(udc, ep_fifo_size2, (u32) S3C24XX_UDC_MAXP_REG, 7);
	usb_write(udc, ep_fifo_size2, (u32) S3C24XX_UDC_MAXP_REG, 8);
}


static int s3c24xx_udc_set_pullup(struct s3c24xx_udc *udc, int is_on)
{
	struct s3c2410_udc_mach_info *info;
	enum s3c2410_udc_cmd_e cmd;
	
	info = udc->mach_info;

	if (is_on)
		cmd = S3C2410_UDC_P_ENABLE;
	else
		cmd = S3C2410_UDC_P_DISABLE;
	
	/* Call the platform dependet function if it's available */
	if (info && info->udc_command) {
		info->udc_command(cmd);
	} else {
                if (is_on) {
			/*
			 * Only enable the UDC if a Gadget-driver was already registered,
			 * otherwise by the registration the UDC-enable function should
			 * be called.
			 * (Luis Galdos)
			 */
			if (udc->driver)
				s3c24xx_udc_enable(udc);
		} else {
                        if (udc->gadget.speed != USB_SPEED_UNKNOWN) {
                                if (udc->driver && udc->driver->disconnect)
                                        udc->driver->disconnect(&udc->gadget);

                        }
                        s3c24xx_udc_disable(udc);
                }
        }

        return 0;
}


static int s3c24xx_udc_vbus_session(struct usb_gadget *gadget, int is_active)
{
        struct s3c24xx_udc *udc = gadget_to_udc(gadget);

        udc->vbus = (is_active != 0);
        s3c24xx_udc_set_pullup(udc, is_active);
        return 0;
}

/*
 * This function is called by detection of the bus-power over the requested IRQ
 */
static irqreturn_t s3c24xx_udc_vbus_irq(int irq, void *_udc)
{
	struct s3c24xx_udc	*udc = _udc;
	unsigned int		value;
	struct s3c2410_udc_mach_info *info;
	ulong cfg;

	info = udc->mach_info;

	/* Some cpus cannot read from an line configured to IRQ! */
	cfg = s3c2410_gpio_getcfg(info->vbus_pin);
	s3c2410_gpio_cfgpin(info->vbus_pin, S3C2410_GPIO_INPUT);
	value = s3c2410_gpio_getpin(info->vbus_pin);
	s3c2410_gpio_cfgpin(info->vbus_pin, cfg);

	if (info->vbus_pin_inverted)
		value = !value;

	printk_debug("Bus detect %s: vbus %i | value %i\n",
		     info->vbus_pin_inverted ? "inverted" : "", udc->vbus, value);
	
	if (value != udc->vbus)
		s3c24xx_udc_vbus_session(&udc->gadget, value);
	
	return IRQ_HANDLED;
}


/*
 * Interrupt handler of the USB-function. The interrupts to detect are coming
 * from the SMDK, but it doesn't consider the speed detection, which can lead
 * to some failures.
 *
 * (Luis Galdos)
 */
#define S3C2443_UDC_INT_CHECK			(0xff8f | S3C24XX_UDC_INT_HSP)
static irqreturn_t s3c24xx_udc_irq(int irq, void *_udc)
{
	struct s3c24xx_udc *udc = _udc;
	u32 intr_out, intr_in, intr_all;
	u32 sys_stat, sys_stat_chk;
	u32 stat, cnt;
	unsigned long flags;
	struct s3c_ep *ep;

	spin_lock_irqsave(&udc->lock, flags);

	sys_stat = readl(udc->base + S3C24XX_UDC_SSR_REG);
	stat = sys_stat;
	intr_all = readl(udc->base + S3C24XX_UDC_EIR_REG);
	
	/* We have only 3 usable eps now */
	sys_stat_chk = sys_stat & S3C2443_UDC_INT_CHECK;

	/* Only check for the correct endpoints (Luis Galdos) */
	for (cnt = 0, intr_in = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &udc->ep[cnt];

		/* Skip the OUT-endpoints different than zero */
		if (!(ep->bEndpointAddress & USB_DIR_IN) && cnt != 0)
			continue;
		
		if (s3c24xx_ep_enabled(udc, cnt))
			intr_in |= (1 << cnt);
	}
	intr_in &= intr_all;

	/* Check for the OUT-EPs that have generated an interrupt (Luis Galdos) */
	for (cnt = 0, intr_out = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &udc->ep[cnt];

		/* Skip the IN-endpoints */
		if (ep->bEndpointAddress & USB_DIR_IN)
			continue;
		
		if (s3c24xx_ep_enabled(udc, cnt))
			intr_out |= (1 << cnt);
	}
	intr_out &= intr_all;

	printk_debug("UDC IRQ: stat 0x%08x (0x%08x) | in 0x%08x | out 0x%08x\n",
		     stat, sys_stat_chk, intr_in, intr_out);

	if (!intr_out && !intr_in && !sys_stat_chk)
		goto exit_ack;
	
	if (sys_stat) {
		if (sys_stat & S3C24XX_UDC_INT_VBUSON) {
			printk_debug("Vbus ON interrupt\n");
			writel(S3C24XX_UDC_INT_VBUSON, udc->base + S3C24XX_UDC_SSR_REG);
			udc->vbus = 1;
			/* s3c24xx_udc_enable(udc); */
		}

		if (sys_stat & S3C24XX_UDC_INT_ERR) {
			printk_debug("ERROR interrupt\n");
			writel(S3C24XX_UDC_INT_ERR,
			       udc->base + S3C24XX_UDC_SSR_REG);
		}

		if (sys_stat & S3C24XX_UDC_INT_SDE) {

			writel(S3C24XX_UDC_INT_SDE,
			       udc->base + S3C24XX_UDC_SSR_REG);

			if (sys_stat & S3C24XX_UDC_INT_HSP) {
				printk_debug("HIGH SPEED detection\n");
				s3c24xx_set_max_pktsize(udc, USB_SPEED_HIGH);
			} else {
				printk_debug("FULL SPEED detection\n");
				s3c24xx_set_max_pktsize(udc, USB_SPEED_FULL);
			}
		}

		if (sys_stat & S3C24XX_UDC_INT_HSP) {

			writel(S3C24XX_UDC_INT_HSP, udc->base + S3C24XX_UDC_SSR_REG);
			printk_debug("High Speed interrupt\n");
		}
		
		if (sys_stat & S3C24XX_UDC_INT_SUSPEND) {
			printk_debug("SUSPEND interrupt\n");
			writel(S3C24XX_UDC_INT_SUSPEND,
			       udc->base + S3C24XX_UDC_SSR_REG);
			if (udc->gadget.speed != USB_SPEED_UNKNOWN
			    && udc->driver
			    && udc->driver->suspend) {
				udc->driver->suspend(&udc->gadget);
			}
		}

		if (sys_stat & S3C24XX_UDC_INT_RESUME) {
			printk_debug("RESUME interrupt\n");
			writel(S3C24XX_UDC_INT_RESUME,
			       udc->base + S3C24XX_UDC_SSR_REG);
			if (udc->gadget.speed != USB_SPEED_UNKNOWN
			    && udc->driver
			    && udc->driver->resume) {
				udc->driver->resume(&udc->gadget);
			}
		}

		if (sys_stat & S3C24XX_UDC_INT_RESET) {
			printk_debug("RESET interrupt\n");
			writel(S3C24XX_UDC_INT_RESET,
			       udc->base + S3C24XX_UDC_SSR_REG);
			reconfig_usbd(udc);
			udc->ep0state = WAIT_FOR_SETUP;
		}
	}

	if (intr_in) {
		if (intr_in & S3C24XX_UDC_INT_EP0){
			ulong ep0sr;
			
			/* First handle the arrived data, and then clear the IRQ */
			s3c24xx_handle_ep0(udc);
			writel(S3C24XX_UDC_INT_EP0, udc->base + S3C24XX_UDC_EIR_REG);

			/*
			 * By long setup-handlings it's possible to have a TST at
			 * this point.
			 */
			ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
			if (ep0sr & S3C24XX_UDC_EP0SR_TST)
				writel(S3C24XX_UDC_EP0SR_TST,
				       udc->base + S3C24XX_UDC_EP0SR_REG);
		} else {
			unsigned long cnt, epm;
			
			/* First get the EP-number that generated the interrupt */
			for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
				epm = (1 << cnt);
				if (intr_in & epm) {
					s3c24xx_udc_in_epn(udc, cnt);
					writel(epm, udc->base + S3C24XX_UDC_EIR_REG);
				}
			}
		}
	}

	/*
	 * We don't havea handling for the EP0-OUT cause the RSR is set by the
	 * TST too.
	 */
	if (intr_out) {
		unsigned long cnt, epm;

		for (cnt = 1; cnt < S3C_MAX_ENDPOINTS; cnt++) {
			epm = (1 << cnt);
			if (intr_out & epm) {
				writel(epm, udc->base + S3C24XX_UDC_EIR_REG);
				s3c24xx_udc_out_epn(udc, cnt);
			}
		}
	}

 exit_ack:	
	/* writel(stat, udc->base + S3C24XX_UDC_SSR_REG); */
	spin_unlock_irqrestore(&udc->lock, flags);
	return IRQ_HANDLED;
}


/*
 * Enable one EP, but only if it's different than the EP zero
 * This function is called from the gadget-drivers over the UDC-operation functions
 */
static int s3c24xx_udc_ep_enable(struct usb_ep *_ep,
				 const struct usb_endpoint_descriptor *desc)
{
	struct s3c_ep *ep;
	struct s3c24xx_udc *udc;
	unsigned long flags, epnr, regval;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || !desc || ep->desc) {
		printk_err("NULL pointer or bad EP or descriptor found.\n");
		return -EINVAL;
	} else if (_ep->name == ep0name) {
		printk_err("Invalid EP name (%s) to enable.\n", _ep->name);
		return -EINVAL;
	} else if (desc->bDescriptorType != USB_DT_ENDPOINT) {
		printk_err("Invalid descriptor type (USB_DT_ENDPOINT)\n");
		return -EINVAL;
	} else if (ep->bEndpointAddress != desc->bEndpointAddress) {
		printk_err("Invalid EP address found (valid %x | invalid %x)\n",
			   ep->bEndpointAddress, desc->bEndpointAddress);
		return -EINVAL;
	} else if (ep_maxpacket(ep) < le16_to_cpu(desc->wMaxPacketSize)) {
		printk_err("Invalid EP size %u (max. %u)\n",
			   le16_to_cpu(desc->wMaxPacketSize), ep_maxpacket(ep));
		return -EINVAL;
	}
	
	/* xfer types must match, except that interrupt ~= bulk */
	if (ep->bmAttributes != desc->bmAttributes
	    && ep->bmAttributes != USB_ENDPOINT_XFER_BULK
	    && desc->bmAttributes != USB_ENDPOINT_XFER_INT) {
		printk_err("Type mismatch by EP %s\n", _ep->name);
		return -EINVAL;
	}

	/* hardware _could_ do smaller, but driver doesn't */
	if ((desc->bmAttributes == USB_ENDPOINT_XFER_BULK
	     && le16_to_cpu(desc->wMaxPacketSize) > ep_maxpacket(ep))
	    || !desc->wMaxPacketSize) {
		printk_err("Bad %s maxpacket (desc %u | max %u)\n", _ep->name,
			   le16_to_cpu(desc->wMaxPacketSize), ep_maxpacket(ep));
		return -ERANGE;
	}

	udc = ep->dev;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN) {
		printk_err("Bogus device state\n");
		return -ESHUTDOWN;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);

	ep->stopped = 0;
	ep->desc = desc;
	ep->pio_irqs = 0;
	ep->ep.maxpacket = le16_to_cpu(desc->wMaxPacketSize);

	/* Enable the interrupt for this EP */
	epnr = ep_index(ep);
	regval = readl(udc->base + S3C24XX_UDC_EIER_REG);
	regval |= (1 << epnr);
	writel(regval, udc->base + S3C24XX_UDC_EIER_REG);

	/* Enable the dual FIFO mode */
	regval = usb_read(udc, S3C24XX_UDC_ECR_REG, epnr);
	regval |= S3C24XX_UDC_ECR_DUEN;
        usb_write(udc, regval, S3C24XX_UDC_ECR_REG, epnr);
	
	/* Reset halt state */
	s3c24xx_udc_set_halt(_ep, 0);

	spin_unlock_irqrestore(&ep->dev->lock, flags);

	printk_debug("Enabled %s | Addr. 0x%02x\n", _ep->name, ep->bEndpointAddress);
	return 0;
}

static int s3c24xx_udc_ep_disable(struct usb_ep *_ep)
{
	struct s3c_ep *ep;
	unsigned long flags;
	int epnr;
	struct s3c24xx_udc *udc;
	ulong regval;

	if (!_ep) {
		printk_err("Null pointer passed! Aborting.\n");
		return -EINVAL;
	}
	
	ep = container_of(_ep, struct s3c_ep, ep);
	if (!ep->desc) {
		printk_debug("%s has an empty descriptor\n", ep->ep.name);
		return -EINVAL;
	}

	spin_lock_irqsave(&ep->dev->lock, flags);
	
	/* Disable the corresponding IRQ */
	udc = ep->dev;
	epnr = ep_index(ep);
	regval = readl(udc->base + S3C24XX_UDC_EIER_REG);
	regval &= ~(1 << epnr);
	writel(regval, udc->base + S3C24XX_UDC_EIER_REG);
	
	/* Nuke all pending requests */
	nuke(ep, -ESHUTDOWN);

	ep->desc = 0;
	ep->stopped = 1;

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

static struct usb_request *s3c24xx_udc_alloc_request(struct usb_ep *ep, gfp_t gfp_flags)
{
	struct s3c_request *req;

	req = kmalloc(sizeof *req, gfp_flags);
	if (!req)
		return NULL;

	memset(req, 0, sizeof *req);
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}


static void s3c24xx_udc_free_request(struct usb_ep *ep, struct usb_request *_req)
{
	struct s3c_request *req;

	req = container_of(_req, struct s3c_request, req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/*
 * This function is called by the Gadget-drivers when they have a request for 
 * the UDC (for us).
 */
static int s3c24xx_udc_queue(struct usb_ep *_ep, struct usb_request *_req,
			     gfp_t gfp_flags)
{
	struct s3c_request *req;
	struct s3c_ep *ep;
	struct s3c24xx_udc *udc;
	unsigned long flags;
	int retval;
	
	spin_lock_irqsave(&udc->lock, flags);
	
	req = container_of(_req, struct s3c_request, req);
	if (unlikely(!_req || !_req->complete || !_req->buf || 
		     !list_empty(&req->queue))) {
		printk_err("Bad params for a new EP queue\n");
		retval = -EINVAL;
		goto exit_queue_unlock;
	}

	ep = container_of(_ep, struct s3c_ep, ep);
	if (unlikely(!_ep || (!ep->desc && ep->ep.name != ep0name))) {
		printk_err("Bad EP or invalid descriptor\n");
		retval = -EINVAL;
		goto exit_queue_unlock;
	}

	udc = ep->dev;
	if (unlikely(!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)) {
		printk_err("Bogus device state %p\n", udc->driver);
		retval = -ESHUTDOWN;
		goto exit_queue_unlock;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;
	
	/*
	 * By the IN-endpoints only add the new request to the
	 * internal EP-queue and schedule the tasklet. The tasklet
	 * function will check if the request data can be sent or
	 * not.
	 * (Luis Galdos)
	 */
	if (ep_is_in(ep) && ep_index(ep) != 0) {
		spin_lock(&ep->lock);
#if defined(DEBUG_S3C2443_UDC_QUEUE)
		{
			u8 ch1, ch2;
			int len;
			u8 *ptr;

			ptr = (u8 *)_req->buf;
			len = _req->length;
			ch1 = *ptr;
			ch2 = *(ptr + len - 1);
			printk(KERN_DEBUG "%p: len=%i, 0x%02x ... 0x%02x [QUEUE]\n",
			       req, len, ch1, ch2);
		}
#endif
		list_add_tail(&req->queue, &ep->queue);
		spin_unlock(&ep->lock);
		tasklet_hi_schedule(&ep->in_tasklet);
	} else {
		int handled;

		/*
		 * If the request couldn't be handled, then tail it into the
		 * queue of the endpoint
		 */
		handled = 0;
		if (list_empty(&ep->queue) && likely(!ep->stopped)) {
		
			if (unlikely(ep_index(ep) == 0)) {
				list_add_tail(&req->queue, &ep->queue);
				s3c24xx_udc_ep0_kick(udc, ep);
				handled = 1;
				
			} else {
				/*
				 * The read-function returns zero if the request is not
				 * done (there is free available buffer space). In this
				 * case we must add the request to the internal queue.
				 * (Luis Galdos)
				 */
				retval = s3c24xx_udc_read_fifo(ep, req);
				handled = (retval == 1) ?  (1) : (0);

				/* Error handling */
				if (retval < 0) {
					printk_err("EP%i: Read FIFO error\n",
						   ep_index(ep));
					goto exit_queue_unlock;
				}
			}
		}

		/* Advances the queue with the non handled request */
		if (!handled)
			list_add_tail(&req->queue, &ep->queue);
	}
	
	retval = 0;
	
 exit_queue_unlock:
	spin_unlock_irqrestore(&udc->lock, flags);
	return retval;
}

/* Dequeue one USB-request */
static int s3c24xx_udc_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct s3c_ep *ep;
	struct s3c_request *req;
	unsigned long flags;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep || ep->ep.name == ep0name)
		return -EINVAL;

	printk_debug("EP%i: Dequeue called\n", ep_index(ep));
	
	spin_lock_irqsave(&ep->dev->lock, flags);

	/* Make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		spin_unlock_irqrestore(&ep->dev->lock, flags);
		return -EINVAL;
	}

	done(ep, req, -ECONNRESET);

	spin_unlock_irqrestore(&ep->dev->lock, flags);
	return 0;
}

/*
 * Halt specific EP (Return 0 if success)
 */
static int s3c24xx_udc_set_halt(struct usb_ep *_ep, int value)
{
	return 0;
}

/*
 * Return the available data bytes of the EP-FIFO
 */
static int s3c24xx_udc_fifo_status(struct usb_ep *_ep)
{
	u32 csr;
	int count = 0;
	struct s3c_ep *ep;

	ep = container_of(_ep, struct s3c_ep, ep);
	if (!_ep) {
		printk_debug("%s: bad ep\n", __FUNCTION__);
		return -ENODEV;
	}

	/* LPD can't report unclaimed bytes from IN fifos */
	if (ep_is_in(ep))
		return -EOPNOTSUPP;

	csr = usb_read(ep->dev, S3C24XX_UDC_EP_STATUS_REG, ep_index(ep));
	if (ep->dev->gadget.speed != USB_SPEED_UNKNOWN ||
	    csr & S3C24XX_UDC_EP_RX_SUCCESS) {
	    
		count = usb_read(ep->dev, S3C24XX_UDC_BYTE_READ_CNT_REG, ep_index(ep));
		
		if (usb_read(ep->dev, S3C24XX_UDC_EP_STATUS_REG, ep_index(ep))
		    & S3C24XX_UDC_EP_LWO)
			count = count * 2 -1;
		else
			count = count * 2;
	}

	return count;
}

/*
 * Flush the FIFO of the endpoint
 */
static void s3c24xx_udc_fifo_flush(struct usb_ep *_ep)
{
	struct s3c_ep *ep;
	struct s3c24xx_udc *udc;
	int epnr;

	if (!_ep) {
		printk_err("Can't flush an EP, NULL pointer passed\n");
		return;
	}
	
	ep = container_of(_ep, struct s3c_ep, ep);
	epnr = ep_index(ep);
	udc = ep->dev;
	
	if (unlikely(epnr == 0)) {
		printk_err("EP0 can't be flushed. Aborting.\n");
		return;
	}

	/* Flush the EP by using the control register */
	printk_debug("EP%i: Flushing now\n", epnr);
	usb_write(udc, S3C24XX_UDC_ECR_FLUSH, S3C24XX_UDC_ECR_REG, epnr);
}


/* Return:  0 = still running, 1 = completed, negative = errno */
static int s3c24xx_udc_write_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req)
{
	unsigned count;
	int is_last;

	count = s3c24xx_udc_write_packet(ep, req);

	/* Last packet is usually short (or a zlp) */
	is_last = (count == 0) ? (1) : (0);
	
	printk_debug("%d bytes | Last %i | Left %d | Request %p\n",
		     count, is_last, req->req.length - req->req.actual, req);

	/* Requests complete when all IN data is in the FIFO */
	if (is_last) {
		return 1;
	}

	return 0;
}

/*
 * Function used for reading the data from the FIFO to the passed buffer
 * This function is only used for the setup-handling
 */
static inline int s3c24xx_udc_ep0_setup_read(struct s3c_ep *ep, u16 *cp, int max)
{
        int bytes;
        int count;
        struct s3c24xx_udc *udc;
        ulong ep0sr;

        udc = ep->dev;

        ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
        if (!(ep0sr & S3C24XX_UDC_EP0SR_RSR)) {
                printk_debug("RSR-bit unset. Aborting setup read.\n");
                return 0;
        }

        /* Now get the number of bytes to read from the FIFO */
        count = usb_read(udc, S3C24XX_UDC_BRCR_REG, ep_index(ep));
        if (ep0sr & S3C24XX_UDC_EP0SR_LWO)
                bytes = count * 2 - 1;
        else
                bytes = count * 2;

        if (bytes > max) {
                printk_err("Not enough space for the SETUP-frame!\n");
                count = max / 2;
                bytes = max;
        }

        while (count--)
                *cp++ = (u16)readl(udc->base + S3C24XX_UDC_EP0BR_REG);

        /*
         * IMPORTANT: Dont delete the below line, then otherwise the controller will
         * not work (but why not?)
         * (Luis Galdos)
         */
	writel(S3C24XX_UDC_EP0_RX_SUCCESS, udc->base + S3C24XX_UDC_EP0SR_REG);

	return bytes;
}

/* static int s3c24xx_udc_read_fifo_ep0(struct s3c_ep *ep, struct s3c_request *req) */
/* { */
/* 	u32 csr; */
/* 	u16 *buf; */
/* 	unsigned bufferspace, count, is_short, bytes; */
/* 	u32 fifo = ep->fifo; */
/* 	struct s3c24xx_udc *udc; */

/* 	udc = ep->dev; */

/* 	csr = readl(udc->base + S3C24XX_UDC_EP0SR_REG); */
/* 	if (!(csr & S3C24XX_UDC_EP0_RX_SUCCESS)) */
/* 		return 0; */

/* 	buf = req->req.buf + req->req.actual; */
/* 	prefetchw(buf); */
/* 	bufferspace = req->req.length - req->req.actual; */

/* 	/\* read all bytes from this packet *\/ */
/* 	if (likely(csr & S3C24XX_UDC_EP0_RX_SUCCESS)) { */
/* 		count = usb_read(udc, S3C24XX_UDC_BYTE_READ_CNT_REG, ep_index(ep)); */
/* 		if (csr & S3C24XX_UDC_EP0_LWO) */
/* 			bytes = count * 2 - 1; */
/* 		else */
/* 			bytes = count * 2; */
			
/* 		req->req.actual += min(bytes, bufferspace); */
/* 	} else	{ */
/* 		count = 0; */
/* 		bytes = 0; */
/* 	} */

/* 	is_short = (bytes < ep->ep.maxpacket); */
/* 	while (likely(count-- != 0)) { */
/* 		u16 byte = (u16)readl(udc->base + fifo); */

/* 		if (unlikely(bufferspace == 0)) { */
/* 			/\* this happens when the driver's buffer */
/* 			 * is smaller than what the host sent. */
/* 			 * discard the extra data. */
/* 			 *\/ */
/* 			if (req->req.status != -EOVERFLOW) */
/* 				printk_err("Overflow %s, %d\n", ep->ep.name, count); */
/* 			req->req.status = -EOVERFLOW; */
/* 		} else { */
/* 			*buf++ = byte; */
/* 			bufferspace = bufferspace - 2; */
/* 		} */
/* 	} */
	
/* 	/\* By completions returns one *\/ */
/* 	if (is_short || req->req.actual == req->req.length) */
/* 		return 1; */

/* 	return 0; */
/* } */


/*
 * udc_set_address - set the USB address for this device
 * @address:
 *
 * Called from control endpoint function
 * after it decodes a set address setup packet.
 */
static void s3c24xx_udc_set_address(struct s3c24xx_udc *udc, unsigned char address)
{
	udc->usb_address = address;
}


/*
 * DATA_STATE_RECV (OUT_PKT_RDY)
 */
/* static int first_time = 1; */

/* static void s3c24xx_udc_ep0_read(struct s3c24xx_udc *udc) */
/* { */
/* 	struct s3c_request *req; */
/* 	struct s3c_ep *ep = &udc->ep[0]; */
/* 	int ret; */

/* 	if (!list_empty(&ep->queue)) */
/* 		req = list_entry(ep->queue.next, struct s3c_request, queue); */
/* 	else { */
/* 		BUG();	//logic ensures		-jassi */
/* 		return; */
/* 	} */
	
/* 	if(req->req.length == 0) { */
/* 		writel(S3C24XX_UDC_EP0_RX_SUCCESS, udc->base + S3C24XX_UDC_EP0SR_REG); */
/* 		udc->ep0state = WAIT_FOR_SETUP; */
/* 		first_time = 1; */
/* 		done(ep, req, 0); */
/* 		return; */
/* 	} */

/* 	if(!req->req.actual && first_time) { */
/* 		writel(S3C24XX_UDC_EP0_RX_SUCCESS, udc->base + S3C24XX_UDC_EP0SR_REG); */
/* 		first_time = 0; */
/* 		return; */
/* 	} */

/* 	ret = s3c24xx_udc_read_fifo_ep0(ep, req); */
/* 	if (ret) { */
/* 		udc->ep0state = WAIT_FOR_SETUP; */
/* 		first_time = 1; */
/* 		done(ep, req, 0); */
/* 		return; */
/* 	} */
/* } */


/*
 * DATA_STATE_XMIT
 */
static int s3c24xx_udc_ep0_write(struct s3c24xx_udc *udc)
{
	struct s3c_request *req;
	struct s3c_ep *ep = &udc->ep[0];
	int ret, need_zlp = 0;

	/* printk_info("EP0: Write request\n"); */

	if (list_empty(&ep->queue))
		req = 0;
	else
		req = list_entry(ep->queue.next, struct s3c_request, queue);

	if (!req) {
		printk_debug("EP0: NULL write request?\n");
		return 0;
	}

	if (req->req.length == 0) {
		udc->ep0state = WAIT_FOR_SETUP;
	   	done(ep, req, 0);
		return 1;
	}

	if (req->req.length - req->req.actual == ep0_fifo_size) {
		/* Next write will end with the packet size, */
		/* so we need Zero-length-packet */
		need_zlp = 1;
	}

	ret = s3c24xx_udc_write_fifo_ep0(ep, req);

	if ((ret == 1) && !need_zlp) {
		/* Last packet */
		printk_debug("EP0: Finished, waiting for status\n");
		udc->ep0state = WAIT_FOR_SETUP;
	} else {
		/* We need to send more data to the host in the next transfer */
		printk_debug("EP0: not finished | %p\n", req);
	}

	if (need_zlp) {
		printk_debug("EP0: Need ZLP!\n");
		udc->ep0state = DATA_STATE_NEED_ZLP;
	}

	if(ret)
		done(ep, req, 0);

	return 1;
}

/* Return zero if NO additional request is available */
static inline int s3c2443_ep0_fix_set_setup(struct s3c24xx_udc *udc,
					    struct usb_ctrlrequest *ctrl)
{
	int timeout_us, cnt;
	ulong ep0sr;
	int retval;
	struct s3c_ep *ep;

	ep = &udc->ep[0];
	timeout_us = 10000;
	do {
		udelay(1);
		ep0sr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);
		timeout_us--;
	} while (timeout_us && !(ep0sr & S3C24XX_UDC_EP0SR_RSR));

	/* If a timeout happens then returns zero */
	retval = 0;
	if (timeout_us) {
		cnt = s3c24xx_udc_ep0_setup_read(ep, (u16 *)ctrl,
						 sizeof(struct usb_ctrlrequest));
		if (cnt > 0)
			retval = 1;
	}

	return retval;
}

/*
 * Wait for a setup packet and read if from the FIFO before passint it to the
 * gadget driver
 */
#define S3C2443_PACKET_IS_SET_REQ(ctrl) \
(ctrl.bRequest == USB_REQ_SET_CONFIGURATION || \
 ctrl.bRequest == USB_REQ_SET_INTERFACE || \
	    ctrl.bRequest == USB_REQ_SET_DESCRIPTOR || \
	    ctrl.bRequest == USB_REQ_SET_FEATURE || \
 ctrl.bRequest == USB_REQ_SET_ADDRESS)
static void s3c24xx_ep0_setup(struct s3c24xx_udc *udc, u32 csr)
{
	struct s3c_ep *ep;
	int retval, bytes, is_in;
	int second_request, third_request;
	struct usb_ctrlrequest ctrl1, ctrl2, ctrl3;

	/* Nuke all previous transfers of this control EP */
	ep = &udc->ep[0];
	nuke(ep, -EPROTO);

	/* Read control req from fifo (8 bytes) */
	bytes = s3c24xx_udc_ep0_setup_read(ep, (u16 *)&ctrl1,
					   sizeof(struct usb_ctrlrequest));

	/*
	 * @FIXME: This is a really bad code, but at this moment there is no better
	 * proposal for a workaround, since I dont know why the HELL the controller
	 * stops working by some SETUP-frames which require a status stage.
	 * Probably the problem is that the controller sends automatically the
	 * IN-Data of the status stage (with length zero) and it doesn't NAK the
	 * proceeded IN-request. Since, we have still data from the first IN-frame in
	 * the FIFO and the second arrived very fast, the FIFO gets stuck.
	 * Unfortunately we can't reset the FIFO (the control bit for flushing the
	 * FIFO seems to be only for the EPs different than zero).
	 *
	 * For avoiding the described problem probably a delayed-work function can help,
	 * otherwise the below code can't be removed.
	 * (Luis Galdos)
	 */
	second_request = 0;
	third_request = 0;
	if (S3C2443_PACKET_IS_SET_REQ(ctrl1)) {

		/* Check for a second setup frame */
		second_request = s3c2443_ep0_fix_set_setup(udc, &ctrl2);
		if (second_request && S3C2443_PACKET_IS_SET_REQ(ctrl2))
			third_request = s3c2443_ep0_fix_set_setup(udc, &ctrl3);

		/* Check for a third setup frame */
		if (third_request) {
			printk(KERN_DEBUG "3. SETUP: bRequestType 0x%02x | "
			       "bRequest 0x%02x | wValue 0x%04x\n",
			       ctrl3.bRequestType, ctrl3.bRequest, ctrl3.wValue);
		}
	}

	/* Set direction of EP0 */
	if (likely(ctrl1.bRequestType & USB_DIR_IN)) {
		printk_debug("EP0: Preparing new IN frame (%i bytes)\n", bytes);
		ep->bEndpointAddress |= USB_DIR_IN;
		is_in = 1;
	} else {
		printk_debug("EP0: Preparing new OUT frame (%i bytes)\n", bytes);
		ep->bEndpointAddress &= ~USB_DIR_IN;
		is_in = 0;
	}

	/* Handle some SETUP packets ourselves */
	retval = 0;
	switch (ctrl1.bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (ctrl1.bRequestType != (USB_TYPE_STANDARD | USB_RECIP_DEVICE))
			break;
		
		printk_debug("Set address request (%d)\n", ctrl1.wValue);
		s3c24xx_udc_set_address(udc, ctrl1.wValue);
		goto exit_ep0_setup;

	default:
		printk_debug("bRequestType 0x%02x | bRequest 0x%02x | "
			     "wValue 0x%04x | wIndex 0x%04x | wLength %u\n",
		       ctrl1.bRequestType, ctrl1.bRequest, ctrl1.wValue, ctrl1.wIndex,
		       ctrl1.wLength);
		break;
	}

	/* Check if need to call the Gadget-setup handler */
	if (likely(udc->driver)) {
		spin_unlock(&udc->lock);
		retval = udc->driver->setup(&udc->gadget, &ctrl1);
		spin_lock(&udc->lock);
	}

exit_ep0_setup:
	if (udc->driver && second_request && !retval) {
		spin_unlock(&udc->lock);
		retval = udc->driver->setup(&udc->gadget, &ctrl2);
		spin_lock(&udc->lock);
	}

	if (udc->driver && third_request && !retval) {
		spin_unlock(&udc->lock);
		retval = udc->driver->setup(&udc->gadget, &ctrl3);
		spin_lock(&udc->lock);
	}
	
	/* By error free setups return at this place */
	if (!retval)
		return;

	/* @XXX: Test if the STALL is really send to the host */
	printk_err("Gadget setup FAILED (%d). Stalling.\n", retval);
	udc->ep0state = WAIT_FOR_SETUP;
	writel(S3C24XX_UDC_EP0CR_ESS, udc->base + S3C24XX_UDC_EP0CR_REG);
}

/*
 * handle ep0 interrupt
 */
static void s3c24xx_handle_ep0(struct s3c24xx_udc *udc)
{
	struct s3c_ep *ep = &udc->ep[0];
	u32 csr;
	unsigned long handled;

	handled = 0;
 	csr = readl(udc->base + S3C24XX_UDC_EP0SR_REG);

	/* Clear the STALL bit */
	if (csr & S3C24XX_UDC_EP0_STALL) {
		printk_debug("EP0: Stall success\n");
		writel(S3C24XX_UDC_EP0_STALL, udc->base + S3C24XX_UDC_EP0SR_REG);
		nuke(ep, -ECONNABORTED);
		udc->ep0state = WAIT_FOR_SETUP;
		handled = 1;
	}

	/* We must check here if there is additional data to send */
	if (csr & S3C24XX_UDC_EP0_TX_SUCCESS) {
		struct s3c_ep *ep0 = &udc->ep[0];
		struct s3c_request *req;
		int left;

		req = list_entry(ep0->queue.next, struct s3c_request, queue);
		if (req) {
			left = req->req.length - req->req.actual;
			printk_debug("EP0: TX success | %p: left %i\n", req, left);
			if (left)
				s3c24xx_udc_ep0_write(udc);
		}				

		/* Always clear the status bit */
		writel(S3C24XX_UDC_EP0_TX_SUCCESS, udc->base + S3C24XX_UDC_EP0SR_REG);
		handled = 1;
	}

	/* Check if we have received data from the host (SETUP-frames) */
	if (csr & S3C24XX_UDC_EP0_RX_SUCCESS) {
		if (udc->ep0state == WAIT_FOR_SETUP) {
			printk_debug("EP0: RX success | Wait for setup\n");
			s3c24xx_ep0_setup(udc, csr);
		} else {
			udc->ep0state = WAIT_FOR_SETUP;
			printk_err("EP0: RX success | Strange state\n");
			writel(S3C24XX_UDC_EP0SR_RSR, udc->base + S3C24XX_UDC_EP0SR_REG);
		}
		
		handled = 1;
	}

	/*
	 * Under unknown conditions the EP0 is generating some interrupts which we can't
	 * identify. Since these IRQs seem to have none side effects, we only print
	 * a debug message and return inmediately. Please note that the IRQs are
	 * being generated only by the enumeration of the device, just when the EP0 is
	 * in use.
	 * @XXX: Add some additional register outprints (EP0SR, EP0CR, SSR, etc.)
	 * (Luis Galdos)
	 */
	if (!handled) {
		printk_debug("[ ERROR ] s3c24xx-udc: Unhandled EP0 IRQ.\n");
	}
}

/*
 * @FIXME: The below function is coming from the SMDK, but it can't be correct! The
 * EP0 is neither IN- nor OUT-EP, it is a control EP!
 * (Luis Galdos)
 */
static void s3c24xx_udc_ep0_kick(struct s3c24xx_udc *udc, struct s3c_ep *ep)
{
	/* if (ep_is_in(ep)) { */
		udc->ep0state = DATA_STATE_XMIT;
		s3c24xx_udc_ep0_write(udc);
	/* } else { */
/* 		udc->ep0state = DATA_STATE_RECV; */
/* 		s3c24xx_udc_ep0_read(udc); */
/* 	} */
}

static int s3c_udc_get_frame(struct usb_gadget *_gadget)
{
	u32 frame;
	struct s3c24xx_udc *udc;

	udc = the_controller;
	if (!udc)
		return -ENODEV;
	
	frame = readl(udc->base + S3C24XX_UDC_FNR_REG);
	return (frame & 0x7ff);
}

static int s3c_udc_wakeup(struct usb_gadget *_gadget)
{
	return -ENOTSUPP;
}

static const struct usb_gadget_ops s3c_udc_ops = {
	.get_frame = s3c_udc_get_frame,
	.wakeup = s3c_udc_wakeup,
	/* current versions must always be self-powered */
};

static void nop_release(struct device *dev)
{
	printk_debug("%s %s\n", __FUNCTION__, dev->bus_id);
}

static struct s3c24xx_udc memory = {
	.usb_address = 0,

	.gadget = {
		.ops = &s3c_udc_ops,
		.ep0 = &memory.ep[0].ep,
		.name = DRIVER_NAME,
		.dev = {
			.bus_id = "gadget",
			.release = nop_release,
		},
	},

	.ep[0] = {
		.ep = {
			.name = ep0name,
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP0_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 0,
		.bmAttributes = 0,
		.ep_type = ep_control,
		.fifo = S3C24XX_UDC_EP0BR_REG,
	},

	.ep[1] = {
		.ep = {
			.name = "ep1-bulk",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 1,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.ep_type = ep_bulk_out,
		.fifo = S3C24XX_UDC_EP1BR_REG,
	},

	.ep[2] = {
		.ep = {
			.name = "ep2-bulk",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 2,
		.bmAttributes = USB_ENDPOINT_XFER_BULK,
		.ep_type = ep_bulk_in,
		.fifo = S3C24XX_UDC_EP2BR_REG,
	},

	.ep[3] = {
		.ep = {
			.name = "ep3-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 3,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP3BR_REG,
	},

	.ep[4] = {				// Though NOT USED XXX
		.ep = {
			.name = "ep4-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE,
		},
		.dev = &memory,
		.bEndpointAddress = 4,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP4BR_REG,
	},
	.ep[5] = {				// Though NOT USED XXX
		.ep = {
			.name = "ep5-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 5,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP5BR_REG,
	},
	.ep[6] = {				// Though NOT USED XXX
		.ep = {
			.name = "ep6-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,
		.bEndpointAddress = 6,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP6BR_REG,
	},
	.ep[7] = {				// Though NOT USED XXX
		.ep = {
			.name = "ep7-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,
		.bEndpointAddress = USB_DIR_IN | 7,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP7BR_REG,
	},
	.ep[8] = {				// Though NOT USED XXX
		.ep = {
			.name = "ep8-int",
			.ops = &s3c24xx_ep_ops,
			.maxpacket = EP_FIFO_SIZE2,
		},
		.dev = &memory,
		.bEndpointAddress = 8,
		.bmAttributes = USB_ENDPOINT_XFER_INT,
		.ep_type = ep_interrupt,
		.fifo = S3C24XX_UDC_EP8BR_REG,
	},
};


/*
 * probe - binds to the platform device
 */
static int s3c24xx_udc_probe(struct platform_device *pdev)
{
	struct s3c24xx_udc *udc = &memory;
	int retval;
	struct s3c2410_udc_mach_info *mach_info;
	int cnt;
	
	printk_debug("Probing a new device ID %i\n", pdev->id);

	/* Get the platform data */
	mach_info = pdev->dev.platform_data;
	if (!mach_info) {
		printk_err("No platform data? Aborting.\n");
		retval = -EINVAL;
		goto err_exit;
	}

	udc->mach_info = mach_info;
	udc->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!udc->mem) {
		printk_err("Couldn't get the IO memory resource\n");
		retval = -EINVAL;
		goto err_exit;
	}

        udc->mem = request_mem_region(udc->mem->start, IOMEMSIZE(udc->mem), pdev->name);
        if (!udc->mem) {
                printk_err("Failed to request IO memory region.\n");
                retval = -ENOENT;
                goto err_exit;
        }
	
	udc->base = ioremap(udc->mem->start, IOMEMSIZE(udc->mem));
	if (!udc->base) {
		printk_err("Couldn't ioremap the IO memory region\n");
		retval = -EINVAL;
		goto err_free_mem;
	}

	/* Init the internal gadget device */
	device_initialize(&udc->gadget.dev);
	udc->gadget.dev.parent = &pdev->dev;
	udc->gadget.dev.dma_mask = pdev->dev.dma_mask;

	/* @XXX: We can use only one device, right? */
	the_controller = udc;
	platform_set_drvdata(pdev, udc);

	/* Init the EPs */
	s3c24xx_udc_reinit(udc);
	
	/* Disable the platform dependent UDC-hardware */
	s3c24xx_udc_disable(udc);
	
	spin_lock_init(&udc->lock);
	udc->dev = pdev;

	udc->gadget.is_dualspeed = 1;
	udc->gadget.is_otg = 0;
	udc->gadget.is_a_peripheral = 0;
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

/* 	local_irq_disable(); */

	/* Get the IRQ for the internal handling of the EPs */
	retval = request_irq(IRQ_USBD, s3c24xx_udc_irq, 0, pdev->name, udc);
	if (retval) {
		printk_err("Cannot get irq %i, err %d\n", IRQ_USBD, retval);
		retval = -EBUSY;
		goto err_iounmap;
	}

	/* Activate the driver first when is going to be used */
	disable_irq(IRQ_USBD);
	printk_debug("IRQ %i for the UDC\n", IRQ_USBD);
	
	if (mach_info && mach_info->vbus_pin > 0) {
		udc->irq_vbus = s3c2410_gpio_getirq(mach_info->vbus_pin);
		retval = request_irq(udc->irq_vbus,
				     s3c24xx_udc_vbus_irq,
				     IRQF_DISABLED | IRQF_SHARED |
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				     pdev->name, udc);
		if (retval) {
			printk_err("Can't get vbus IRQ (%i) for IO %i (err %d)\n",
				   udc->irq_vbus, mach_info->vbus_pin, retval);
			goto err_free_udc_irq;
		}
		printk_debug("IRQ %i for vbus detection\n", udc->irq_vbus);
	} else
		udc->vbus = 1;
	
	/*
	 * Init the tasklet for the IN-endpoints at this place, so that we can kill
	 * the tasklets when the module is going to be removed
	 * (Luis Galdos)
	 */
	for (cnt = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		struct s3c_ep *ep = &udc->ep[cnt];
		
		if (ep_is_in(ep)) {
			tasklet_init(&ep->in_tasklet,
				     s3c24xx_udc_epin_tasklet_func,
				     (unsigned long)ep);
		}
		
		spin_lock_init(&ep->lock);
	}
	
/* 	local_irq_enable(); */
/* 	create_proc_files(); */
	return 0;

 err_free_udc_irq:
	free_irq(IRQ_USBD, udc);

 err_iounmap:
	iounmap(udc->base);
	
 err_free_mem:
	release_mem_region(udc->mem->start, IOMEMSIZE(udc->mem));
		
 err_exit:
	platform_set_drvdata(pdev, NULL);

	/*
	 * Unset the controller pointer otherwise the function for registering
	 * a new gadget can crash the system (it uses this pointer)
	 * (Luis Galdos)
	 */
	the_controller = NULL;
	return retval;
}


static int s3c24xx_udc_remove(struct platform_device *pdev)
{
	struct s3c2410_udc_mach_info *imach;
	struct s3c24xx_udc *udc;
	int cnt;
	struct s3c_ep *ep;

	printk_debug("Removing the UDC driver (ID %i)\n", pdev->id);

	udc = platform_get_drvdata(pdev);
	imach = pdev->dev.platform_data;

	/* Kill the tasklet of all the IN-endpoints */
	for (cnt = 0; cnt < S3C_MAX_ENDPOINTS; cnt++) {
		ep = &udc->ep[cnt];
		if (ep_is_in(ep))
			tasklet_kill(&ep->in_tasklet);
	}
	
	s3c24xx_udc_disable(udc);
/* 	remove_proc_files(); */
	usb_gadget_unregister_driver(udc->driver);

	free_irq(IRQ_USBD, udc);

	/*
	 * If an IRQ for the vbus was passed, then disable it too
	 * (Luis Galdos)
	 */
	if (imach && udc->irq_vbus) {
		printk_debug("Freeing the vbus IRQ %i\n", udc->irq_vbus);
		free_irq(udc->irq_vbus, udc);
		udc->irq_vbus = 0;
	}
	
	release_mem_region(udc->mem->start, IOMEMSIZE(udc->mem));
	
	platform_set_drvdata(pdev, NULL);
	
	the_controller = NULL;

	return 0;
}

static struct platform_driver s3c24xx_udc_driver = {
	.probe		= s3c24xx_udc_probe,
	.remove		= s3c24xx_udc_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
	},
};

static int __init udc_init(void)
{
	int ret;

	printk_info("Loading (%s - %s)\n", DRIVER_BUILD_DATE, DRIVER_BUILD_TIME);
	ret = platform_driver_register(&s3c24xx_udc_driver);
	return ret;
}

static void __exit udc_exit(void)
{
	printk_info("Unloading (%s - %s)\n", DRIVER_BUILD_DATE, DRIVER_BUILD_TIME);
	platform_driver_unregister(&s3c24xx_udc_driver);
}

module_init(udc_init);
module_exit(udc_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Luis Galdos, luis.galdos[at]digi.com");
MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
