
static void ccwmx51js_ad9389_enable(int enable, int vif)
{
}

#ifdef CONFIG_CCWMX51_DISP0_RGB888

static struct fb_videomode ad9389_1024x768x24_65MHz = {
	.name		= "1024x768x24@60",
	.refresh	= 60,
	.xres		= 1024,
	.yres		= 768,
	.pixclock	= 15384, /* pico seconds of 65.0MHz */
	.left_margin	= 160,
	.right_margin	= 24,
	.upper_margin	= 29,
	.lower_margin	= 3,
	.hsync_len	= 136,
	.vsync_len	= 6,
	.sync		= FB_SYNC_EXT,
	.vmode		= FB_VMODE_NONINTERLACED,
	.flag		= FB_MODE_IS_VESA,
};

#define AD9389_1024x768x24_60				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "1024x768x24@60",		\
		.mode = &ad9389_1024x768x24_65MHz,	\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}

static struct fb_videomode ad9389_1920x1080x24 = {
	.name		= "1920x1080x24@60",
	.refresh	= 60,
	.xres 		= 1920,
	.yres 		= 1080,
	.pixclock 	= 7518,
	.left_margin 	= 30,
	.right_margin 	= 304,
	.upper_margin	= 3,
	.lower_margin 	= 46,
	.hsync_len 	= 64,
	.vsync_len 	= 3,
	.sync 		= FB_SYNC_HOR_HIGH_ACT,
	.vmode		= FB_VMODE_NONINTERLACED,
};

#define AD9389_1920x1080x24_60				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "1920x1080x24@60",		\
		.mode = &ad9389_1920x1080x24,	\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}

#endif /* CONFIG_CCWMX51_DISP0_RGB888 */

#if defined(CONFIG_CCWMX51_DISP0_RGB666) || defined(CONFIG_CCWMX51_DISP1)
#endif /* CONFIG_CCWMX51_DISP0_RGB666 */
