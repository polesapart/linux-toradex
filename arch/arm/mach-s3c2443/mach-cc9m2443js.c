/* -*- linux-c -*-
 * 
 * linux/arch/arm/mach-s3c2443/mach-cc9m2443js.c
 *
 * Copyright (c) 2007 Simtec Electronics
 * Copyright (c) 2008 Digi International Spain
 *	Ben Dooks <ben@simtec.co.uk>
 *	Miriam Ruiz <Miriam.Ruiz@digi.com>
 *	Luis Galdos <luis.galdos@digi.com>
 *
 * http://www.fluff.org/ben/smdk2443/
 * http://www.digi.com/products/embeddedsolutions/connectcore9m.jsp
 *
 * Thanks to Samsung for the loan of an SMDK2443
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/smc911x.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/delay.h>

#include <plat/regs-serial.h>
#include <mach/regs-gpio.h>
#include <mach/regs-gpioj.h>
#include <mach/regs-lcd.h>
#include <mach/regs-s3c2443-clock.h>

#include <mach/idle.h>
#include <mach/fb.h>
#include <mach/spi.h>
#include <mach/gpio.h>
#include <linux/spi/spi.h>

#include <plat/s3c2410.h>
#include <plat/s3c2440.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/hsmmc.h>
#include <plat/ts.h>
#include <plat/udc.h>
#include <plat/spi.h>
#include <mach/irqs.h>
#include <mach/regs-irq.h>
#include <plat/irq.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <plat/common-smdk.h>
#include <asm/plat-s3c24xx/mci.h>
#include <asm/plat-s3c/nand.h>



/*
 * IMPORTANT: We are using the Ethernet-driver of the Blackfin-project because:
 * - The driver from the Vanilla-kernel is not working out of the box
 * - After some modifications (for accessing to the FIFO correctly) the driver
 *   has some performance-problems (lost some packets)
 * - The driver doesn't use the new NAPI for reading from the controller
 * (Luis Galdos)
 */
static struct resource smsc911x_device_resources[] = {
        [0] = {
                .name = "smsc911x-memory",
                .start = S3C2410_CS5,
                .end   = S3C2410_CS5 + SZ_1M - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EINT9,
                .end = IRQ_EINT9,
                .flags = IORESOURCE_IRQ,
        },
};

static struct smc911x_platdata cc9m2443_smsc9118 = {
        .flags          = SMC911X_USE_16BIT,
        .irq_flags      = IRQF_DISABLED | IRQF_TRIGGER_FALLING,
        .irq_polarity   = 0,
};

static struct platform_device smsc911x_device = {
        .name             = "smsc911x",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(smsc911x_device_resources),
        .resource         = smsc911x_device_resources,
        .dev            = {
                .platform_data = &cc9m2443_smsc9118,
        }
};

/* @XXX: The macro will be used for external board-devices under cc9m2443-custom.h */
#if !defined(CC9M2443_I2C_BASE_BOARD_DEVICES)
#define CC9M2443_I2C_BASE_BOARD_DEVICES
/*#define CC9M2443_I2C_BASE_BOARD_DEVICES		\
	{					\
		.driver_name = "rtc-ds1307",	\
		.addr = 0x68,			\
		.type = "ds1337",		\
	}					\
#endif */
#endif


static struct i2c_board_info cc9m2443_i2c_devices[] = {
	{
                I2C_BOARD_INFO("ds1337", 0x68),
	},
	CC9M2443_I2C_BASE_BOARD_DEVICES
};



static struct map_desc cc9m2443_iodesc[] __initdata = {
	/* ISA IO Space map (memory space selected by A24) */
	{
		.virtual	= (u32)S3C24XX_VA_ISA_WORD,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_WORD + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE,
		.pfn		= __phys_to_pfn(S3C2410_CS2),
		.length		= 0x10000,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)S3C24XX_VA_ISA_BYTE + 0x10000,
		.pfn		= __phys_to_pfn(S3C2410_CS2 + (1<<24)),
		.length		= SZ_4M,
		.type		= MT_DEVICE,
	}, {
		.virtual	= (u32)CC9M2443_VA_LAN,
		.pfn		= __phys_to_pfn(CC9M2443_PA_LAN),
		.length		= CC9M2443_SZ_LAN,
		.type		= MT_DEVICE,
	}
};


/* Macros for the default port configuration on the CC9M2443 */
#define CC9M2443_PORT_UCON			(S3C2412_UCON_PCLK | \
						S3C2410_UCON_RXIRQMODE | \
						S3C2410_UCON_TXIRQMODE | \
						S3C2410_UCON_RXFIFO_TOI | \
						S3C2443_UCON_RXERR_IRQEN | \
						S3C2410_UCON_TXILEVEL | \
						S3C2410_UCON_RXILEVEL)
#define CC9M2443_PORT_UFCON			(S3C2410_UFCON_FIFOMODE | \
						S3C2440_UFCON_RXTRIG8 | \
						S3C2440_UFCON_TXTRIG16)
#define CC9M2443_PORT_LCON			(S3C2410_LCON_CS8)


static struct s3c2410_uartcfg cc9m2443_uartcfgs[] __initdata = {
#ifdef CONFIG_SERIAL_S3C2410_ENABLE_PORTA
	{
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = CC9M2443_PORT_UCON,
		.ulcon	     = S3C2410_LCON_CS8,
		.ufcon	     = CC9M2443_PORT_UFCON,
		.rx_gpio     = S3C2410_GPH1,
		.rx_cfg      = S3C2443_GPH1_RXD0,
		.tx_gpio     = S3C2410_GPH0,
		.tx_cfg      = S3C2443_GPH0_TXD0,
#if defined(CONFIG_SERIAL_S3C2410_PORTA_HWCTRL)
		.rts_gpio    = S3C2410_GPH9,
		.rts_cfg     = S3C2443_GPH9_nRTS0,
		.cts_gpio    = S3C2410_GPH8,
		.cts_cfg     = S3C2443_GPH8_nCTS0,
#endif /* CONFIG_SERIAL_S3C2410_PORTA_HWCTRL */
	},
#endif /* CONFIG_SERIAL_S3C2410_ENABLE_PORTA */
#ifdef CONFIG_SERIAL_S3C2410_ENABLE_PORTB
	{
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = CC9M2443_PORT_UCON,
		.ulcon	     = CC9M2443_PORT_LCON,
		.ufcon	     = CC9M2443_PORT_UFCON,
		.rx_gpio     = S3C2410_GPH3,
		.rx_cfg      = S3C2443_GPH3_RXD1,
		.tx_gpio     = S3C2410_GPH2,
		.tx_cfg      = S3C2443_GPH2_TXD1,
#if defined(CONFIG_SERIAL_S3C2410_PORTB_HWCTRL)
		.rts_gpio    = S3C2443_GPH11,
		.rts_cfg     = S3C2443_GPH11_nRTS1,
		.cts_gpio    = S3C2410_GPH10,
		.cts_cfg     = S3C2443_GPH10_nCTS1,
#endif /* CONFIG_SERIAL_S3C2410_PORTB_HWCTRL */
	},
#endif /* CONFIG_SERIAL_S3C2410_ENABLE_PORTB */
#ifdef CONFIG_SERIAL_S3C2410_ENABLE_PORTC
	{
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = CC9M2443_PORT_UCON,
		.ulcon	     = CC9M2443_PORT_LCON,
		.ufcon	     = CC9M2443_PORT_UFCON,
		.rx_gpio     = S3C2410_GPH5     ,
		.rx_cfg      = S3C2443_GPH5_RXD2,
		.tx_gpio     = S3C2410_GPH4     ,
		.tx_cfg      = S3C2443_GPH4_TXD2,
#if defined(CONFIG_SERIAL_S3C2410_PORTC_HWCTRL)
		.rts_gpio    = S3C2410_GPH6,
		.rts_cfg     = S3C2443_GPH6_nRTS2,
		.cts_gpio    = S3C2410_GPH7,
		.cts_cfg     = S3C2443_GPH7_nCTS2,
#endif /* CONFIG_SERIAL_S3C2410_PORTC_HWCTRL */

	},
#endif /* CONFIG_SERIAL_S3C2410_ENABLE_PORTC */
#ifdef CONFIG_SERIAL_S3C2410_ENABLE_PORTD
	{
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = CC9M2443_PORT_UCON,
		.ulcon	     = CC9M2443_PORT_LCON,
		.ufcon	     = CC9M2443_PORT_UFCON,
		.rx_gpio     = S3C2410_GPH7,
		.rx_cfg      = S3C2443_GPH7_RXD3,
		.tx_gpio     = S3C2410_GPH6,
		.tx_cfg      = S3C2443_GPH6_TXD3,
	}
#endif /* CONFIG_SERIAL_S3C2410_ENABLE_PORTD */
};




/* The below structure belongs to the Sharp LQ064 */
static struct s3c2410fb_display cc9m2443fd_display = {

	/* @FIXME: Need to define another way for setting the colors per pixel */
	.lcdcon5	= S3C2410_LCDCON5_INVPWREN | S3C2410_LCDCON5_PWREN,

	.vidcon1	= S3C24XX_LCD_VIDCON1_IHSYNC |
	S3C24XX_LCD_VIDCON1_IVSYNC,
	
	.type		= S3C2410_LCDCON1_TFT,
	.width		= 640,
	.height		= 480,

	.xres           = 640,  /* horizontal pixels */
	.yres           = 480,  /* vertical pixels */
	.bpp            = 16,
	
	.left_margin    = 5,   /* horizontal front porch */
	.upper_margin   = 3,   /* vertical front porch */
	.right_margin   = 5,   /* horizontal back porch */
	.lower_margin   = 30,  /* vertical back porch */

	.hsync_len      = 25,
	.vsync_len      = 4,

	/*
	 * Set the configuration registers
	 * IMPORTANT: In difference to WinCE we use the LCD clock instead of the HCLK
	 */
	.vidcon0	= S3C24XX_LCD_VIDCON0_VIDOUT_RGB_IF |
	S3C24XX_LCD_VIDCON0_RGB_PAR |
	S3C24XX_LCD_VIDCON0_VCLK_OFF |
	S3C24XX_LCD_VIDCON0_CLKDIR_DIVIDED,

	/* Select the colors per pixel (565, 1555, etc.) */
/* 	.wincon0	= S3C24XX_LCD_WINCON_16BPP_1555, */
	.wincon0	= S3C24XX_LCD_WINCON_16BPP_565,
	
	.frame_rate	= 60,

};


/* This is required for the LCD-display */
static struct s3c2410fb_mach_info cc9m2443fb_mach = {
	.displays	= &cc9m2443fd_display,
	.num_displays	= 1,

	/* Set the GPIO-configuration */
        .gpccon		= 0xaaa002a8,
        .gpccon_mask	= 0xfff003fc,

	.gpcup		= 0xfff003fc,
        .gpcup_mask	= 0xfff003fc,

	.gpdcon		= 0xaaa0aaa0,
        .gpdcon_mask	= 0xfff0fff0,

	.gpdup		= 0xfff0fff0,
        .gpdup_mask	= 0xfff0fff0,

};


/* This is for the LCD platform device */
static struct resource s3c_lcd_resource[] = {
        [0] = {
                .start = S3C24XX_PA_TFTLCD,
                .end   = S3C24XX_PA_TFTLCD + S3C24XX_SZ_LCD - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_S3C2443_LCD3,
                .end   = IRQ_S3C2443_LCD3,
                .flags = IORESOURCE_IRQ,
        }
};

static u64 s3c_device_lcd_dmamask = 0xffffffffUL;

struct platform_device s3c_device_tft_lcd = {
        .name             = "s3c2410fb-tft",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(s3c_lcd_resource),
        .resource         = s3c_lcd_resource,
        .dev              = {
                .dma_mask               = &s3c_device_lcd_dmamask,
                .coherent_dma_mask      = 0xffffffffUL,
		.platform_data		= &cc9m2443fb_mach,
        }
};



/*
 * SDIO device for the CC9M2443JS
 * The correct driver name is "s3c2440-sdi" and NOT "s3c2410-sdi"
 */
static struct resource cc9m2443_sdi_resource[] = {
	[0] = {
		.start = S3C2410_PA_SDI,
		.end   = S3C2410_PA_SDI + S3C24XX_SZ_SDI - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_SDI,
		.end   = IRQ_SDI,
		.flags = IORESOURCE_IRQ,
	}

};

static struct s3c24xx_mci_pdata cc9m2443_sdi_pdata = {
	.gpio_detect	= S3C2410_GPF1,
	.gpio_wprotect	= S3C2410_GPG9,
	.dma_enable     = 1,
};

struct platform_device cc9m2443_device_sdi = {
	.name		= "s3c2440-sdi",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cc9m2443_sdi_resource),
	.resource	= cc9m2443_sdi_resource,
	.dev		= {
		.platform_data = &cc9m2443_sdi_pdata,
	}
};


/* High-speed MMC/SD */
static struct resource cc9m2443_hsmmc_resource[] = {
	[0] = {
		.start = S3C2443_PA_HSMMC,
		.end   = S3C2443_PA_HSMMC + S3C2443_SZ_HSMMC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_S3C2443_HSMMC,
		.end   = IRQ_S3C2443_HSMMC,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 cc9m2443_device_hsmmc_dmamask = 0xffffffffUL;

#define HOST_CAPS		(MMC_CAP_4_BIT_DATA | \
				MMC_CAP_MMC_HIGHSPEED | \
				MMC_CAP_SD_HIGHSPEED)

struct s3c_hsmmc_cfg cc9m2443_hsmmc_platform = {
        .hwport        = 0,
        .host_caps     = HOST_CAPS,
        .ctrl3[0]      = 0x00800080,
        .ctrl3[1]      = 0x00800080,
        .clk_name[0]   = "hclk",          /* 1st clock HCLK */
        .clk_name[1]   = "hsmmc-div",     /* 2nd clock hsmmc-epll */
        .clk_name[2]   = "hsmmc-ext",     /* 3rd clock external */
	.gpio_wprotect = S3C2443_GPJ15,
	.gpio_detect   = S3C2443_GPJ14,
	.gpio_led      = S3C2443_GPJ13,
};

struct platform_device cc9m2443_device_hsmmc = {
	.name		  = "s3c-hsmmc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(cc9m2443_hsmmc_resource),
	.resource	  = cc9m2443_hsmmc_resource,
	.dev              = {
		.dma_mask = &cc9m2443_device_hsmmc_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data = &cc9m2443_hsmmc_platform,
	}
};





/* Touch device */
static struct resource cc9m2443_ts_resource[] = {
	[0] = {
		.start = S3C2410_PA_ADC,
		.end   = S3C2410_PA_ADC + S3C24XX_SZ_ADC - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_ADC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.start = IRQ_TC,
		.flags = IORESOURCE_IRQ,
	}
};

/* @XXX: Set the correct values */
static struct s3c_ts_mach_info cc9m2443_ts_info = {
	.delay			= 1,
	.presc			= 255,
	.oversampling_shift	= 1,
	.xmin			= 0x3ff,
	.ymin			= 0x3ff,
	.trigger_ms		= 1,
	.probes			= 9,
};

struct platform_device cc9m2443_device_ts = {
	.name		= "s3c24xx-ts",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(cc9m2443_ts_resource),
	.resource	= cc9m2443_ts_resource,
	.dev		= {
			.platform_data = &cc9m2443_ts_info,
	}
};



/* This is the High Speed SPI-master of the S3C2443 */
static struct s3c2443_spi_info cc9m2443_hsspi_info = {
        .bus_num = 0,
	.num_chipselect = 1,
	.miso = {
		.nr = S3C2410_GPE11,
		.cfg = S3C2410_GPE11_SPIMISO0,
	},
	.mosi = {
		.nr = S3C2410_GPE12,
		.cfg = S3C2410_GPE12_SPIMOSI0,
	},
	.clk = {
		.nr = S3C2410_GPE13,
		.cfg = S3C2410_GPE13_SPICLK0,
	},
	.cs[0] = {
		.nr = S3C2443_GPL13,
		.cfg = S3C2443_GPL13_nSS0,
	}
};

static struct resource cc9m2443_hsspi_resources[] = {
        {
                .start  = S3C2443_PA_HSSPI,
                .end    = S3C2443_PA_HSSPI + S3C2443_SZ_HSSPI - 1,
                .flags  = IORESOURCE_MEM,
        }, {
                .start  = IRQ_SPI0,
                .flags  = IORESOURCE_IRQ,
        }
};

static struct platform_device cc9m2443_device_hsspi = {
        .name           = "spi-s3c2443",
        .id             = 0,
        .resource       = cc9m2443_hsspi_resources,
        .num_resources  = ARRAY_SIZE(cc9m2443_hsspi_resources),
        .dev            = {
                .platform_data = &cc9m2443_hsspi_info,
		.coherent_dma_mask = DMA_BIT_MASK(32),
        }
};



/* This is the USB-device of the S3C2443 */
static struct resource cc9m2443_udc_resources[] = {
        [0] = {
                .start = S3C2443_PA_USBDEV,
                .end   = S3C2443_PA_USBDEV+ S3C2443_SZ_USBDEV - 1,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_USBD,
                .end   = IRQ_USBD,
                .flags = IORESOURCE_IRQ,
        }
};

static struct s3c2410_udc_mach_info cc9m2443_udc_pdata = {
        .vbus_pin               = S3C2410_GPF5,
        .vbus_pin_inverted      = 0,
        .udc_command            = NULL,
};

static struct platform_device cc9m2443_device_udc = {
        .name             = "s3c24xx_udc",
        .id               = -1,
        .num_resources    = ARRAY_SIZE(cc9m2443_udc_resources),
        .resource         = cc9m2443_udc_resources,
	.dev		= {
		.platform_data = &cc9m2443_udc_pdata,
	}
};
	




/* Platform devices for the CC9M2443 */
static struct platform_device *cc9m2443_devices[] __initdata = {
	&s3c_device_usb,
	&s3c_device_tft_lcd,
	&s3c_device_wdt,
	&s3c_device_i2c,
	&s3c_device_nand,
	&smsc911x_device,
        &cc9m2443_device_sdi,
	&cc9m2443_device_hsmmc,
	&cc9m2443_device_ts,
	&cc9m2443_device_hsspi,
	&cc9m2443_device_udc,
};

static void __init cc9m2443_map_io(void)
{
	struct s3c2410_uartcfg *cfg;
	int cnt;

	s3c24xx_init_io(cc9m2443_iodesc, ARRAY_SIZE(cc9m2443_iodesc));
	s3c24xx_init_clocks(12000000);


	/*
	 * Init the GPIOs for the UART-ports
	 * @XXX: We can probably move this part to the UART-driver, but for this
	 * moment let's initialize the GPIOs at this place
	 * Luis Galdos
	 */
	for (cnt = 0; cnt < ARRAY_SIZE(cc9m2443_uartcfgs); cnt++) {

		cfg = &cc9m2443_uartcfgs[cnt];

		s3c2410_gpio_cfgpin(cfg->rx_gpio, cfg->rx_cfg);
		s3c2410_gpio_cfgpin(cfg->tx_gpio, cfg->tx_cfg);
		
		if (cfg->cts_gpio && cfg->rts_gpio) {
			s3c2410_gpio_cfgpin(cfg->cts_gpio, cfg->cts_cfg);
			s3c2410_gpio_cfgpin(cfg->rts_gpio, cfg->rts_cfg);
		}
	}
	
	s3c24xx_init_uarts(cc9m2443_uartcfgs, ARRAY_SIZE(cc9m2443_uartcfgs));
}


/*
 * These are the NAND-timing parameters
 * @TODO: Set the correct timing parameters for the NAND-device
 */
static struct s3c2410_platform_nand cc9m2443_nand_info = {
	.tacls          = 20,
	.twrph0         = 50,
	.twrph1         = 20,
	.nr_sets	= 1,
	.sets           = NULL,
};



/*
 * @FIXME: That's really bad, we need a better way for compiling the GPIO-support
 * into the kernel
 */
#if defined(CONFIG_HAVE_GPIO_LIB)
#include <asm/gpio.h>
static struct gpio_chip cc9m2443_gpios = {
	.label            = "cc9m2443-gpios",
	.direction_input  = s3c2443_gpio_dir_input,
	.direction_output = s3c2443_gpio_dir_output,
	.get              = s3c2443_gpio_get,
	.set              = s3c2443_gpio_set,
	.base             = 0,
	.ngpio            = 256,
};
#endif /* CONFIG_HAVE_GPIO_LIB */



/* SPI devices */
static struct spi_board_info spi_devices[] __initdata = {
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
        {
                .modalias       = "spidev",
                .max_speed_hz   = 500000,
                .bus_num        = 0,
                .chip_select    = 0,
        },
#endif
};



static void __init cc9m2443_machine_init(void)
{
	printk("Starting Linux on ARM ConnectCore 9M 2443\n");

	s3c_device_nand.dev.platform_data = &cc9m2443_nand_info;

	/* Enable the pullup for the write protect GPIO of the SD-port */
	s3c2410_gpio_pullup(S3C2410_GPG9, 0x0);

        /* For the bus detection we MUST activate the pull-down */
        s3c2443_gpio_extpull(S3C2410_GPF5, 1);
		
	/* Though this call the probes of the different drivers will be called */
	platform_add_devices(cc9m2443_devices, ARRAY_SIZE(cc9m2443_devices));

	/* Add the GPIOs of this board */
#if defined(CONFIG_HAVE_GPIO_LIB)
	gpiochip_add(&cc9m2443_gpios);
#endif /* CONFIG_HAVE_GPIO_LIB */
	
	/* The first argument is the bus or adapter number to attach the devices */
	i2c_register_board_info(CONFIG_I2C_S3C2410_ADAPTER_NR,
				cc9m2443_i2c_devices,
				ARRAY_SIZE(cc9m2443_i2c_devices));

        /* SPI devices */
        spi_register_board_info(spi_devices, ARRAY_SIZE(spi_devices));


	
#if 0
	/* @XXX: Add the support for the power managment */
	s3c2410_pm_init();
#endif
}



MACHINE_START(CC9M2443, "ConnectCore 9M 2443 on a JSCC9M2443 Devboard")
        .phys_io	= S3C2410_PA_UART,
	.io_pg_offst	= (((u32)S3C24XX_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S3C2410_SDRAM_PA + 0x100,

	.init_irq	= s3c24xx_init_irq,
	.map_io		= cc9m2443_map_io,
	.init_machine	= cc9m2443_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
