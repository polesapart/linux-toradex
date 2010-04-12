
static void lcd_bl_enable_lq70(int enable)
{
	gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), enable);
}

static struct fb_videomode lq70y3dg3b = {
	.name = "LQ070Y3DG3B",
	.refresh = 60,
	.xres = 800,
	.yres = 480,
	.pixclock = 33000,
	.left_margin = 0,
	.right_margin = 50,
	.upper_margin = 25,
	.lower_margin = 10,
	.hsync_len = 128,
	.vsync_len = 10,
	.vmode = FB_VMODE_NONINTERLACED,
	.flag = 0,
};

#define LQ070Y3DG3B_DISPLAY				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB24,	\
		.mode_str = "LQ070Y3DG3B",		\
		.mode = &lq70y3dg3b,			\
	},						\
	.bl_enable = &lcd_bl_enable_lq70,		\
}
