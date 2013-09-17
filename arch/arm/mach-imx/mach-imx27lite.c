/*
 * Copyright 2007 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2008 Juergen Beisert (kernel@pengutronix.de)
 * Copyright 2009 Daniel Schaeffer (daniel.schaeffer@timesys.com)
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
 */

#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/mfd/mc13783.h>
#include <linux/regulator/machine.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/iomux-mx27.h>
#include <mach/spi.h>
#include <mach/ulpi.h>

#include "devices-imx27.h"
#include "cpu_op-mx27.h"

#define SD1_EN_GPIO (GPIO_PORTB + 22)
#define SPI1_SS0_GPIO (GPIO_PORTD + 28)
#define DISPLAY_POWER_GPIO (GPIO_PORTB + 31)
#define BACKLIGHT_POWER_GPIO (GPIO_PORTC + 27)

static const int mx27lite_pins[] __initconst = {
	/* UART1 */
	PE12_PF_UART1_TXD,
	PE13_PF_UART1_RXD,
	PE14_PF_UART1_CTS,
	PE15_PF_UART1_RTS,
	/* Display */
	PA5_PF_LSCLK,
	PA6_PF_LD0,
	PA7_PF_LD1,
	PA8_PF_LD2,
	PA9_PF_LD3,
	PA10_PF_LD4,
	PA11_PF_LD5,
	PA12_PF_LD6,
	PA13_PF_LD7,
	PA14_PF_LD8,
	PA15_PF_LD9,
	PA16_PF_LD10,
	PA17_PF_LD11,
	PA18_PF_LD12,
	PA19_PF_LD13,
	PA20_PF_LD14,
	PA21_PF_LD15,
	PA22_PF_LD16,
	PA23_PF_LD17,
	PA24_PF_REV,
	PA25_PF_CLS,
	PA26_PF_PS,
	PA27_PF_SPL_SPR,
	PA28_PF_HSYNC,
	PA29_PF_VSYNC,
	PA30_PF_CONTRAST,
	PA31_PF_OE_ACD,
	/* USB OTG */
	PC9_PF_USBOTG_DATA0,
	PC11_PF_USBOTG_DATA1,
	PC10_PF_USBOTG_DATA2,
	PC13_PF_USBOTG_DATA3,
	PC12_PF_USBOTG_DATA4,
	PC7_PF_USBOTG_DATA5,
	PC8_PF_USBOTG_DATA6,
	PE25_PF_USBOTG_DATA7,
	PE24_PF_USBOTG_CLK,
	PE2_PF_USBOTG_DIR,
	PE0_PF_USBOTG_NXT,
	PE1_PF_USBOTG_STP,
	/* USB H2 */
	PD22_AF_USBH2_DATA0,
	PD24_AF_USBH2_DATA1,
	PD23_AF_USBH2_DATA2,
	PD20_AF_USBH2_DATA3,
	PD19_AF_USBH2_DATA4,
	PD26_AF_USBH2_DATA5,
	PD21_AF_USBH2_DATA6,
	PA2_PF_USBH2_DATA7,
	PA0_PF_USBH2_CLK,
	PA1_PF_USBH2_DIR,
	PA3_PF_USBH2_NXT,
	PA4_PF_USBH2_STP,
	/* FEC */
	PD0_AIN_FEC_TXD0,
	PD1_AIN_FEC_TXD1,
	PD2_AIN_FEC_TXD2,
	PD3_AIN_FEC_TXD3,
	PD4_AOUT_FEC_RX_ER,
	PD5_AOUT_FEC_RXD1,
	PD6_AOUT_FEC_RXD2,
	PD7_AOUT_FEC_RXD3,
	PD8_AF_FEC_MDIO,
	PD9_AIN_FEC_MDC,
	PD10_AOUT_FEC_CRS,
	PD11_AOUT_FEC_TX_CLK,
	PD12_AOUT_FEC_RXD0,
	PD13_AOUT_FEC_RX_DV,
	PD14_AOUT_FEC_RX_CLK,
	PD15_AOUT_FEC_COL,
	PD16_AIN_FEC_TX_ER,
	PF23_AIN_FEC_TX_EN,
	/* SSI1 */
	PC20_PF_SSI1_FS,
	PC21_PF_SSI1_RXD,
	PC22_PF_SSI1_TXD,
	PC23_PF_SSI1_CLK,
	/* SDHC2 */
	PB4_PF_SD2_D0,
	PB5_PF_SD2_D1,
	PB6_PF_SD2_D2,
	PB7_PF_SD2_D3,
	PB8_PF_SD2_CMD,
	PB9_PF_SD2_CLK,
	/* SPI1 */
	PD25_PF_CSPI1_RDY,
	PD29_PF_CSPI1_SCLK,
	PD30_PF_CSPI1_MISO,
	PD31_PF_CSPI1_MOSI,
	/* I2C1 */
	PD17_PF_I2C_DATA,
	PD18_PF_I2C_CLK,
	/* I2C2 */
	PC5_PF_I2C2_SDA,
	PC6_PF_I2C2_SCL,
};

/* NAND Flash */
static const struct mxc_nand_platform_data
mx27lite_nand_board_info __initconst = {
	.width = 1,
	.hw_ecc = 1,
};

/* NOR flash */
static struct mtd_partition mx27lite_nor_partitions[] = {
	{
	 .name = "nor.bootloader",
	 .size = 256 * 1024,
	 .offset = 0x00000000,
	 .mask_flags = MTD_WRITEABLE
	},
	{
	 .name = "nor.user",
	 .size = 1664 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0
	},
	{
	 .name = "nor.env1",
	 .size = 64 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0
	},
	{
	 .name = "nor.env2",
	 .size = 64 * 1024,
	 .offset = MTDPART_OFS_APPEND,
	 .mask_flags = 0
	},
};

static struct physmap_flash_data mx27lite_flash_data = {
	.width = 2,
	.parts = mx27lite_nor_partitions,
	.nr_parts = ARRAY_SIZE(mx27lite_nor_partitions),
};

static struct resource mx27lite_flash_resource = {
	.start = 0xc0000000,
	.end = 0xc0000000 + 0x0200000 - 1,
	.flags = IORESOURCE_MEM,

};

static struct platform_device mx27lite_nor_mtd_device = {
	.name = "physmap-flash",
	.id = 0,
	.dev = {
		.platform_data = &mx27lite_flash_data,
	},
	.num_resources = 1,
	.resource = &mx27lite_flash_resource,
};

/* RTC */
static struct resource rtc_resources[] = {
	{
		.start = MX27_RTC_BASE_ADDR,
		.end = MX27_RTC_BASE_ADDR + 0x30,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = MX27_INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device mxc_rtc_device = {
	.name = "mxc_rtc",
	.id = 0,
	.num_resources = ARRAY_SIZE(rtc_resources),
	.resource = rtc_resources,
};

/* LEDs */
static struct gpio_led gpio_leds[] = {
	{
		.name			= "gpio0",
		.default_trigger	= "none",
		.active_low		= 1,
		.gpio			= GPIO_PORTC | 25,
	},
	{
		.name			= "gpio1",
		.default_trigger	= "none",
		.active_low		= 1,
		.gpio			= GPIO_PORTC | 26,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};


static struct platform_device *platform_devices[] __initdata = {
	&mx27lite_nor_mtd_device,
	&mxc_rtc_device,
	&leds_gpio,
};

/* I2C */
static struct imxi2c_platform_data mx27lite_i2c0_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mx27lite_i2c0_devices[] = {
};

static struct imxi2c_platform_data mx27lite_i2c1_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mx27lite_i2c1_devices[] = {
};


/* USB OTG */
static const struct fsl_usb2_platform_data otg_device_pdata __initconst = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_ULPI,
};

/* USB Host */
static struct mxc_usbh_platform_data usbh2_pdata = {
	.portsc	= MXC_EHCI_MODE_ULPI | MXC_EHCI_UTMI_8BIT,
	.flags	= MXC_EHCI_POWER_PINS_ENABLED,
};

static const struct imxuart_platform_data uart_pdata __initconst = {
	.flags = IMXUART_HAVE_RTSCTS,
};

/* Display */
int mx27lite_lcd_init(struct platform_device *pdev)
{
	mxc_gpio_mode(DISPLAY_POWER_GPIO | GPIO_GPIO | GPIO_OUT);
	mxc_gpio_mode(BACKLIGHT_POWER_GPIO | GPIO_GPIO | GPIO_OUT);

	gpio_request(DISPLAY_POWER_GPIO, "lcd_power");
	gpio_request(BACKLIGHT_POWER_GPIO, "backlight_power");

	return 0;
}

void mx27lite_lcd_exit(struct platform_device *pdev)
{
	gpio_free(DISPLAY_POWER_GPIO);
	gpio_free(BACKLIGHT_POWER_GPIO);
}

void mx27lite_lcd_power(int on)
{
	gpio_set_value(DISPLAY_POWER_GPIO, on);
}

void mx27lite_backlight_power(int on)
{
	gpio_set_value(BACKLIGHT_POWER_GPIO, on);
}

static struct imx_fb_videomode mx27lite_modes[] = {
	{
		.mode = {
			.name		= "SHARP-LQ64D343",
			.refresh	= 60,
			.xres		= 640,
			.yres		= 480,
			.pixclock	= 35285,
			.left_margin	= 134,
			.right_margin	= 34,
			.upper_margin	= 35,
			.lower_margin	= 0,
			.hsync_len	= 21,
			.vsync_len	= 5,
		},
		.pcr		= 0xFC008B80,
		.bpp		= 16,
	},
};

static struct imx_fb_platform_data mx27lite_fb_data = {
	.mode		= mx27lite_modes,
	.num_modes	= ARRAY_SIZE(mx27lite_modes),
	.init		= &mx27lite_lcd_init,
	.exit		= &mx27lite_lcd_exit,
	.lcd_power	= &mx27lite_lcd_power,
	.backlight_power = &mx27lite_backlight_power,
	.pwmr		= 0x00A903FF,
	.lscr1		= 0x00120300,
	.dmacr		= 0x00020010,
};

/* SD Card */
static int mx27lite_sdhc2_init(struct device *dev, irq_handler_t detect_irq,
				void *data)
{
	return request_irq(IRQ_GPIOB(22), detect_irq, IRQF_TRIGGER_FALLING |
			IRQF_TRIGGER_RISING, "sdhc1-card-detect", data);
}

static void mx27lite_sdhc2_exit(struct device *dev, void *data)
{
	free_irq(IRQ_GPIOB(22), data);
}

static const struct imxmmc_platform_data sdhc2_pdata __initconst = {
	.init = mx27lite_sdhc2_init,
	.exit = mx27lite_sdhc2_exit,
};

/* MC13783 */
static struct mc13783_platform_data mc13783_pdata __initdata = {
	.regulators = NULL,
	.num_regulators = 0,
	.flags = MC13XXX_USE_TOUCHSCREEN,
};

/* SPI */
static int spi1_internal_chipselect[] = {SPI1_SS0_GPIO};

static const struct spi_imx_master spi1_pdata __initconst = {
	.chipselect	= spi1_internal_chipselect,
	.num_chipselect	= ARRAY_SIZE(spi1_internal_chipselect),
};

static struct spi_board_info mx27lite_spi_devs[] __initdata = {
	{
		.modalias	= "mc13783",
		.max_speed_hz	= 300000,
		.bus_num	= 0,
		.chip_select	= 0, /* SS0 */
		.platform_data	= &mc13783_pdata,
		.irq = IRQ_GPIOB(29),
	},
};

static void __init mx27lite_init(void)
{
	mxc_gpio_setup_multiple_pins(mx27lite_pins, ARRAY_SIZE(mx27lite_pins),
		"imx27lite");
	mxc_gpio_mode(PD28_PF_CSPI1_SS0); /* MC13783 SPI CS */
	mxc_gpio_mode(GPIO_PORTB | 29 | GPIO_GPIO | GPIO_IN); /* MC13783 IRQ */

	/* LEDs */
	mxc_gpio_mode(GPIO_PORTC | 25 | GPIO_GPIO | GPIO_OUT);
	mxc_gpio_mode(GPIO_PORTC | 26 | GPIO_GPIO | GPIO_OUT);

	imx27_add_imx_fb(&mx27lite_fb_data);
	imx27_add_imx_uart0(&uart_pdata);
	imx27_add_mxc_ehci_hs(2, &usbh2_pdata);
	imx27_add_fsl_usb2_udc(&otg_device_pdata);
	imx27_add_fec(NULL);
	imx27_add_mxc_nand(&mx27lite_nand_board_info);

	imx27_add_mxc_mmc(1, &sdhc2_pdata);
	imx27_add_imx2_wdt(NULL);

	i2c_register_board_info(0, mx27lite_i2c0_devices,
				ARRAY_SIZE(mx27lite_i2c0_devices));
	imx27_add_imx_i2c(0, &mx27lite_i2c0_data);
	i2c_register_board_info(1, mx27lite_i2c1_devices,
				ARRAY_SIZE(mx27lite_i2c1_devices));
	imx27_add_imx_i2c(1, &mx27lite_i2c1_data);


	imx27_add_spi_imx0(&spi1_pdata);
	spi_register_board_info(mx27lite_spi_devs,
			ARRAY_SIZE(mx27lite_spi_devs));

	platform_add_devices(platform_devices, ARRAY_SIZE(platform_devices));

#if defined(CONFIG_CPU_FREQ_IMX)
	get_cpu_op = mx27_get_cpu_op;
#endif
}

static void __init mx27lite_timer_init(void)
{
	mx27_clocks_init(26000000);
}

static struct sys_timer mx27lite_timer = {
	.init	= mx27lite_timer_init,
};

MACHINE_START(IMX27LITE, "LogicPD i.MX27LITE")
	.boot_params    = MX27_PHYS_OFFSET + 0x100,
	.map_io         = mx27_map_io,
	.init_irq       = mx27_init_irq,
	.init_machine   = mx27lite_init,
	.timer          = &mx27lite_timer,
MACHINE_END
