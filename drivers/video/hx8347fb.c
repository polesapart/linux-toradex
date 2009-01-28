/*
 * linux/drivers/video/hx8347fb.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/irq.h>

#include <video/hx8347fb.h>

#include <asm/unaligned.h>

#define DRV_NAME	"hx8347"

static void hx8347_dpy_update(struct hx8347fb_par *par)
{
	u16 *buf = (u16 __force *)par->info->screen_base;

	par->pdata->write_cmd(par, SRAM_WR_CTRL);
	par->pdata->write_data(par, buf, par->info->fix.smem_len);
}

/* this is called back from the deferred io workqueue */
static void hx8347_dpy_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	hx8347_dpy_update(info->par);
}

static void hx8347_fillrect(struct fb_info *info,
				   const struct fb_fillrect *rect)
{
	struct hx8347fb_par *par = info->par;

	sys_fillrect(info, rect);
	hx8347_dpy_update(par);
}

static void hx8347_copyarea(struct fb_info *info,
				   const struct fb_copyarea *area)
{
	struct hx8347fb_par *par = info->par;

	sys_copyarea(info, area);
	hx8347_dpy_update(par);
}

static void hx8347_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct hx8347fb_par *par = info->par;

	sys_imageblit(info, image);
	hx8347_dpy_update(par);
}

/*
 * this is the slow path from userspace. they can seek and write to
 * the fb. it is based on fb_sys_write
 */
static ssize_t hx8347_write(struct fb_info *info, const char __user *buf,
				size_t count, loff_t *ppos)
{
	struct hx8347fb_par *par = info->par;
	unsigned long p = *ppos;
	void *dst;
	int err = 0;
	unsigned long total_size;

	if (info->state != FBINFO_STATE_RUNNING)
		return -EPERM;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EFBIG;

	if (count > total_size) {
		err = -EFBIG;
		count = total_size;
	}

	if (count + p > total_size) {
		if (!err)
			err = -ENOSPC;

		count = total_size - p;
	}

	dst = (void __force *)(info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		err = -EFAULT;

	if  (!err)
		*ppos += count;

	hx8347_dpy_update(par);

	return (err) ? err : count;
}

static int hx8347_set_par(struct fb_info *fb)
{
	fb->fix.visual = FB_VISUAL_TRUECOLOR;
	fb->fix.line_length = fb->var.width * fb->var.bits_per_pixel / 8;

	return 0;
}


static struct fb_ops hx8347fb_ops = {
	.owner		= THIS_MODULE,
	.fb_set_par 	= hx8347_set_par,
	.fb_read        = fb_sys_read,
	.fb_write	= hx8347_write,
	.fb_fillrect	= hx8347_fillrect,
	.fb_copyarea	= hx8347_copyarea,
	.fb_imageblit	= hx8347_imageblit,
};

static struct fb_deferred_io hx8347fb_defio = {
	.delay		= HZ,
	.deferred_io	= hx8347_dpy_deferred_io,
};

static int __devinit hx8347_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct hx8347fb_pdata *pdata;
	struct hx8347fb_par *par;
	struct resource *cmdregs = NULL;
	struct resource *dataregs = NULL;
	unsigned char *videomemory;
	int ret;

	/* pick up board specific routines */
	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	/* try to count device specific driver, if can't, platform recalls */
	if (!try_module_get(pdata->owner))
		return -ENODEV;

	cmdregs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!cmdregs) {
		dev_err(&pdev->dev, "%s: unable to get iomem resources\n", __func__);
		ret = -ENXIO;
		goto err_resources;
	}
	dataregs = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!dataregs) {
		dev_err(&pdev->dev, "%s: unable to get iomem resources\n", __func__);
		ret = -ENXIO;
		goto err_resources;
	}

	pdata->irq = platform_get_irq(pdev, 0);
	if (pdata->irq < 0) {
		dev_err(&pdev->dev, "%s: unable to get irq\n", __func__);
		ret = pdata->irq;
		goto err_resources;
	}

	if (!request_mem_region(cmdregs[0].start, cmdregs[0].end - cmdregs[0].start + 1, "hx8347fb cmd")) {
		dev_dbg(&pdev->dev, "%s: request_mem_region failed\n", __func__);
		ret = -EBUSY;
		goto err_resources;
	}

	if (!request_mem_region(dataregs[0].start, dataregs[0].end - dataregs[0].start + 1, "hx8347fb data")) {
		dev_dbg(&pdev->dev, "%s:request_mem_region failed\n", __func__);
		ret = -EBUSY;
		goto err_request_mem;
	}
	info = framebuffer_alloc(sizeof(struct hx8347fb_par), &pdev->dev);
	if (!info) {
		dev_dbg(&pdev->dev, "%s: unable to allocate frambebuffer device\n", __func__);
		ret = -ENOMEM;
		goto err_fballoc;
	}
	par = info->par;
	par->mmio_cmd = (void __iomem *)ioremap_nocache(cmdregs[0].start, cmdregs[0].end - cmdregs[0].start + 1);
	if (!par->mmio_cmd) {
		dev_err(&pdev->dev, "%s: error ioremap cmd\n", __func__);
		ret = -EBUSY;
		goto err_map_cmd;
	}
	par->mmio_data = (void __iomem *)ioremap_nocache(dataregs[0].start, dataregs[0].end - dataregs[0].start + 1);
	if (!par->mmio_data) {
		dev_dbg(&pdev->dev, "%s: error ioremap data\n", __func__);
		ret = -EBUSY;
		goto err_map_data;
	}

	info->fix.smem_len = pdata->xres * pdata->yres * pdata->bits_per_pixel / 8;
	if (!pdata->usedma) {
		if ((videomemory = vmalloc(info->fix.smem_len)) == NULL) {
			dev_err(&pdev->dev, "%s: error vmalloc\n", __func__);
			ret = -ENOMEM;
	       		goto err_vmalloc;
		}
	} else {
		pdev->dev.coherent_dma_mask = (u32)-1;
		videomemory = dma_alloc_writecombine(&pdev->dev, info->fix.smem_len,
			&par->fb_p, GFP_KERNEL);
		if (!videomemory) {
			dev_dbg(&pdev->dev, "%s: err_alloc_fb\n", __func__);
			ret = -ENOMEM;
			goto err_dma_alloc;
		}
	}

	memset(videomemory, 0, info->fix.smem_len);
	info->screen_base = (char __force __iomem *)videomemory;

	info->fbops = &hx8347fb_ops;
	info->pseudo_palette = &pdata->pseudo_pal[0];
	strcpy(info->fix.id, DRV_NAME);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.smem_start = dataregs[0].start;
	info->fix.line_length = pdata->xres * pdata->bits_per_pixel / 8;

	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = pdata->yres;
	info->var.yres = pdata->yres;
	info->var.yres_virtual = pdata->yres;
	info->var.width = pdata->xres;
	info->var.xres = pdata->xres;
	info->var.xres_virtual = pdata->xres;
	info->var.vmode = FB_VMODE_NONINTERLACED;
	info->var.bits_per_pixel = pdata->bits_per_pixel;

	info->var.red.offset = 11;
	info->var.green.offset = 5;
	info->var.blue.offset = 0;
	info->var.red.length = 5;
	info->var.green.length = 6;
	info->var.blue.length = 5;

	par->info = info;
	par->pdata = pdata;

	info->flags = FBINFO_FLAG_DEFAULT;
	info->fbdefio = &hx8347fb_defio;
	fb_deferred_io_init(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: unable to register fb device\n", DRV_NAME);
		goto err_fbreg;
	}

	/* Call the custom init function */
	ret = par->pdata->init(par);
	if (ret < 0)
		goto err_init;

	platform_set_drvdata(pdev, info);
	printk(KERN_INFO
	       "fb%d: HX8347 frame buffer device, using %dK of video memory\n",
	       info->node, info->fix.smem_len >> 10);

	return 0;

err_init:
	framebuffer_release(info);
err_fbreg:
	if (!pdata->usedma)
		vfree(videomemory);
err_vmalloc:
err_dma_alloc:
	iounmap(par->mmio_data);
err_map_data:
	iounmap(par->mmio_cmd);
err_map_cmd:
	framebuffer_release(info);
err_fballoc:
	release_mem_region(dataregs[0].start, dataregs[0].end - dataregs[0].start + 1);
err_request_mem:
	release_mem_region(cmdregs[0].start, cmdregs[0].end - cmdregs[0].start + 1);
err_resources:
	module_put(pdata->owner);

	return ret;
}

static int __devexit hx8347_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct hx8347fb_par *par = info->par;

	if (info) {
		unregister_framebuffer(info);
		fb_deferred_io_cleanup(info);
		fb_dealloc_cmap(&info->cmap);
		if (!par->pdata->usedma)
			vfree((void __force *)info->screen_base);
		iounmap(par->mmio_data);
		iounmap(par->mmio_cmd);

		if (par->pdata->cleanup)
			par->pdata->cleanup(par);

		module_put(par->pdata->owner);
		dev_dbg(&dev->dev, "calling release\n");
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver hx8347_driver = {
	.probe	= hx8347_probe,
	.remove = hx8347_remove,
	.driver	= {
		.owner	= THIS_MODULE,
		.name	= "hx8347",
	},
};

static int __init hx8347_init(void)
{
	return platform_driver_register(&hx8347_driver);
}

static void __exit hx8347_exit(void)
{
	platform_driver_unregister(&hx8347_driver);
}

module_init(hx8347_init);
module_exit(hx8347_exit);

MODULE_DESCRIPTION("Framebuffer driver for the HX8347 controller");
MODULE_AUTHOR("Pedro Perez de Heredia");
MODULE_LICENSE("GPL");
