
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
	if (vif == 0)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN11), !enable);
#ifdef CONFIG_JSCCWMX51_V1
	else if (vif == 1)
		ipu_ccwmx51_disp1_enable(enable);
#elif defined(CONFIG_JSCCWMX51_V2)
		gpio_set_value(IOMUX_TO_GPIO(MX51_PIN_DI1_PIN12), !enable);
#else
#error "A function to enable/disalbe the display have to be specified"
#endif
}

#ifdef CONFIG_CCWMX51_DISP0_RGB888
static struct fb_videomode lq70y3dg3b = {
	.name		= "LQ070Y3DG3B",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= 33000,
	.left_margin	= 0,
	.right_margin	= 50,
	.upper_margin	= 25,
	.lower_margin	= 10,
	.hsync_len	= 128,
	.vsync_len	= 10,
	.vmode		= FB_VMODE_NONINTERLACED,
	.flag		= 0,
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
#endif /* CONFIG_CCWMX51_DISP0_RGB888 */

#if defined(CONFIG_CCWMX51_DISP0_RGB666) || defined(CONFIG_CCWMX51_DISP1)
static struct fb_videomode lq70y3dg3b = {
	.name		= "LQ070Y3DG3B",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= 37000,
	.left_margin	= 0,
	.right_margin	= 50,
	.upper_margin	= 25,
	.lower_margin	= 10,
	.hsync_len	= 128,
	.vsync_len	= 10,
	.vmode		= FB_VMODE_NONINTERLACED,
	.flag		= 0,
};

#define LQ070Y3DG3B_DISPLAY				\
{							\
	.fb_pdata = {					\
		.interface_pix_fmt = IPU_PIX_FMT_RGB666,\
		.mode_str = "LQ070Y3DG3B",		\
		.mode = &lq70y3dg3b,			\
	},						\
	.bl_enable = &lcd_bl_enable_lq70,		\
}
#endif /* CONFIG_CCWMX51_DISP0_RGB666 */
