/*
 * arch/arm/mach-mx5/displays/lcd.h
 *
 * Copyright (C) 2010 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCWMX51_DISPLAYS_LCD_H__
#define __ASM_ARCH_MXC_CCWMX51_DISPLAYS_LCD_H__

#include "../iomux.h"

#if defined (CONFIG_JSCCWMX51_V1)
#include "../drivers/mxc/ipu3/ipu_regs.h"

/**
 * This code is only valide to enable/disable the backlight of the second
 * display, on the first version of the JumpStart Board (JSCCWMX51 RevA).
 * Newer versions use a GPIO to enable the BL of the second display.
 */
void ipu_ccwmx51_disp1_enable(int enable)
{
	uint32_t tmp;

	tmp = __raw_readl(DI_GENERAL(1));
	tmp &= ~DI_GEN_POLARITY_4;
	if (enable)
		tmp |= DI_GEN_POLARITY_4;
	__raw_writel(tmp, DI_GENERAL(1));
}
#endif

static void lcd_bl_enable_lq70(int enable, int vif)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), !enable);
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), !enable);
	if (vif == 0)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), !enable);
	else if (vif == 1)
#ifdef CONFIG_JSCCWMX51_V1
		ipu_ccwmx51_disp1_enable(enable);
#elif defined(CONFIG_JSCCWMX51_V2)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), !enable);
#else
#error "A function to enable/disalbe the display have to be specified"
#endif
}

static void lcd_init(int vif)
{
	/* Initialize lcd enable gpio and video interface lines */
	gpio_video_active(vif, PAD_CTL_DRV_MAX | PAD_CTL_SRE_FAST);
}

static struct fb_videomode lq70y3dg3b = {
	.name		= "LQ070Y3DG3B",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= 38000,
	.left_margin	= 0,
	.right_margin	= 50,
	.upper_margin	= 25,
	.lower_margin	= 10,
	.hsync_len	= 128,
	.vsync_len	= 10,
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
	.flag		= 0,
};

static struct fb_videomode lcd_custom_1 = {
	.name		= "custom1",
	.refresh	= 0,
	.xres		= 0,
	.yres		= 0,
	.pixclock	= 0,
	.left_margin	= 0,
	.right_margin	= 0,
	.upper_margin	= 0,
	.lower_margin	= 0,
	.hsync_len	= 0,
	.vsync_len	= 0,
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

static struct fb_videomode lcd_custom_2 = {
	.name		= "custom2",
	.refresh	= 0,
	.xres		= 0,
	.yres		= 0,
	.pixclock	= 0,
	.left_margin	= 0,
	.right_margin	= 0,
	.upper_margin	= 0,
	.lower_margin	= 0,
	.hsync_len	= 0,
	.vsync_len	= 0,
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

struct ccwmx51_lcd_pdata lcd_panel_list[] = {
	{
		.fb_pdata = {
			.interface_pix_fmt = VIDEO_PIX_FMT,
			.mode_str = "LQ070Y3DG3B",
			.mode = &lq70y3dg3b,
		},
		.bl_enable = lcd_bl_enable_lq70,
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.interface_pix_fmt = VIDEO_PIX_FMT,
			.mode_str = "custom1",
			.mode = &lcd_custom_1,
		},
		.bl_enable = NULL,
		.init = &lcd_init,
	}, {
		.fb_pdata = {
			.interface_pix_fmt = VIDEO_PIX_FMT,
			.mode_str = "custom2",
			.mode = &lcd_custom_2,
		},
		.bl_enable = NULL,
		.init = &lcd_init,
	},
};
#endif /* __ASM_ARCH_MXC_CCWMX51_DISPLAYS_LCD_H__ */
