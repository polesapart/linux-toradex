
static void ccwmx51js_ad9389_enable(int enable, int vif)
{
}

#ifdef CONFIG_CCWMX51_DISP0_RGB888

static struct fb_videomode ad9389_1280x720x24_60 = {
	.name		= "1280x720x24@60",
	.refresh	= 60,
	.xres		= 1280,
	.yres		= 720,
	.pixclock	= 20100,
	.left_margin	= 32,
	.right_margin	= 48,
	.upper_margin	= 7,
	.lower_margin	= 3,
	.hsync_len	= 32,
	.vsync_len	= 6,
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

#define AD9389_1280x720x24_60				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "1280x720x24@60",		\
		.mode = &ad9389_1280x720x24_60,		\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}

static struct fb_videomode ad9389_1360x768x24_60 = {
	.name		= "1360x768x24@60",
	.refresh	= 60,
	.xres		= 1360,
	.yres		= 768,
	.pixclock	= 20300,
	.left_margin	= 139,
	.right_margin	= 256,
	.upper_margin	= 3,
	.lower_margin	= 18,
	.hsync_len	= 76,
	.vsync_len	= 6,
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

#define AD9389_1360x768x24_60				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "1360x768x24@60",		\
		.mode = &ad9389_1360x768x24_60,	\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}

static struct fb_videomode ad9389_1366x768x24_60 = {
	.name		= "1366x768x24@60",
	.refresh	= 60,
	.xres		= 1366,
	.yres		= 768,
	.pixclock	= 20100,
	.left_margin	= 139,
	.right_margin	= 256,
	.upper_margin	= 3,
	.lower_margin	= 18,
	.hsync_len	= 76,
	.vsync_len	= 6,
	.vmode		= FB_VMODE_NONINTERLACED,
	.sync		= FB_SYNC_EXT,
};

#define AD9389_1366x768x24_60				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "1366x768x24@60",		\
		.mode = &ad9389_1366x768x24_60,		\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}

static struct fb_videomode ad9389_1920x1080x24_60 = {
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
	.vmode		= FB_VMODE_NONINTERLACED,
	.flag		= FB_MODE_IS_VESA,
	.sync		= FB_SYNC_EXT,
};

#define AD9389_1920x1080x24_60				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "1920x1080x24@60",		\
		.mode = &ad9389_1920x1080x24_60,	\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}


static struct fb_videomode ad9389_1024x768x24_60 = {
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
		.mode = &ad9389_1024x768x24_60,		\
	},						\
	.bl_enable = &ccwmx51js_ad9389_enable,		\
}
#endif /* CONFIG_CCWMX51_DISP0_RGB888 */

#if defined(CONFIG_CCWMX51_DISP0_RGB666) || defined(CONFIG_CCWMX51_DISP1)
#endif /* CONFIG_CCWMX51_DISP0_RGB666 */
