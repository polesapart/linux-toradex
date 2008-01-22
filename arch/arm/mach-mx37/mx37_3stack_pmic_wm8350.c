/*
 * mx37-3stack-pmic-wm8350.c  --  i.MX37 3STACK Driver for Wolfson WM8350 PMIC
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Copyright 2008 Freescale Semiconductor Inc.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/pmic/wm8350.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/fb.h>
#include <linux/pmic.h>

#include <asm/arch/dma.h>
#include <asm/arch/spba.h>
#include <asm/arch/clock.h>
#include "board-mx37_3stack.h"

/*
 * Set to 1 when testing battery that is connected otherwise spuriuos debug
 */
#define BATTERY 0

extern const char imx37_3stack_audio[32];
extern struct led_trigger *imx32ads_led_trigger;

/* program WM8350 so board will boot from WM8350 supplies */
static int program = 0;
module_param(program, int, 0);
MODULE_PARM_DESC(program, "program initial DCDC & LDO values");

/*
 * Program WM8350 with initial DCDC & LDO values.
 */
static int program_wm8350(struct wm8350 *wm8350)
{
	struct wm8350_pmic *pmic = &wm8350->pmic;
	u16 val;

	wm8350_reg_unlock(wm8350);

	/* DCDC1 @ 1.6 V  CPU */
	wm8350_dcdc_set_voltage(pmic, WM8350_DCDC_1, 1600);
	wm8350_dcdc_set_slot(pmic, WM8350_DCDC_1, 1, 0,
			     WM8350_DC1_ERRACT_SHUTDOWN_SYS);
	wm8350_dcdc_enable(pmic, WM8350_DCDC_1, 1);

	/* DCDC3 @ 2.8 V  Peripherals & IO */
	wm8350_dcdc_set_voltage(pmic, WM8350_DCDC_3, 2800);
	wm8350_dcdc_set_slot(pmic, WM8350_DCDC_3, 1, 0, WM8350_DC1_ERRACT_NONE);
	wm8350_dcdc_enable(pmic, WM8350_DCDC_3, 1);

	/* DCDC4 @ 1.8 V Peripherals & IO */
	wm8350_dcdc_set_voltage(pmic, WM8350_DCDC_4, 1800);
	wm8350_dcdc_set_slot(pmic, WM8350_DCDC_4, 1, 0, WM8350_DC1_ERRACT_NONE);
	wm8350_dcdc_enable(pmic, WM8350_DCDC_4, 1);

	/* DCDC6 @ 1.8 V DDR Memory */
	wm8350_dcdc_set_voltage(pmic, WM8350_DCDC_6, 1800);
	wm8350_dcdc_set_slot(pmic, WM8350_DCDC_6, 1, 0, WM8350_DC1_ERRACT_NONE);
	wm8350_dcdc_enable(pmic, WM8350_DCDC_6, 1);

	/* DCDC2 @  OFF - 5.5V SW3 output */
	wm8350_dcdc_set_slot(pmic, WM8350_DCDC_2, 1, 0,
			     WM8350_DC1_ERRACT_SHUTDOWN_CONV);
	wm8350_dcdc_enable(pmic, WM8350_DCDC_2, 0);

	/* DCDC5 @  OFF  - White LEDS */
	wm8350_dcdc_set_slot(pmic, WM8350_DCDC_5, 1, 1,
			     WM8350_DC1_ERRACT_SHUTDOWN_CONV);
	wm8350_dcdc_enable(pmic, WM8350_DCDC_5, 0);

	/* LDO1 @ 2.8 V Peripherals MMC & Camera */
	wm8350_ldo_set_voltage(pmic, WM8350_LDO_1, 2800);
	wm8350_ldo_set_slot(pmic, WM8350_LDO_1, 1, 0);
	wm8350_ldo_enable(pmic, WM8350_LDO_1, 1);

	/* LDO2 @ 3.3 V */
	wm8350_ldo_set_voltage(pmic, WM8350_LDO_2, 3300);
	wm8350_ldo_set_slot(pmic, WM8350_LDO_2, 1, 0);
	wm8350_ldo_enable(pmic, WM8350_LDO_2, 1);

	/* LDO3 @ 1.5 V VDIG, VGEN */
	wm8350_ldo_set_voltage(pmic, WM8350_LDO_3, 1500);
	wm8350_ldo_set_slot(pmic, WM8350_LDO_3, 1, 0);
	wm8350_ldo_enable(pmic, WM8350_LDO_3, 1);

	/* LDO4 @ 2.6 V Transceivers */
	wm8350_ldo_set_voltage(pmic, WM8350_LDO_4, 2600);
	wm8350_ldo_set_slot(pmic, WM8350_LDO_4, 1, 0);
	wm8350_ldo_enable(pmic, WM8350_LDO_4, 1);

	val = wm8350_reg_read(wm8350, WM8350_INTERFACE_CONTROL) &
	    ~(WM8350_RECONFIG_AT_ON | WM8350_USE_DEV_PINS |
	      WM8350_SPI_3WIRE | WM8350_SPI_4WIRE);
	wm8350_reg_write(wm8350, WM8350_INTERFACE_CONTROL,
			 val | WM8350_CONFIG_DONE | WM8350_AUTOINC);

	/* dummy read required when changing Sec -> Pri I2C */
	printk("dummy read %x\n", wm8350_reg_read(wm8350, 176));
	printk("now press button and then swap jumpers if leds work\n");
	return 0;
}

#ifdef NOT_PORTED_TO_IMX37_3STACK_YET

static void imx37_3stack_switch_handler(struct wm8350 *wm8350, int irq)
{
	printk("switch pressed %d\n", irq);
}
#endif
static struct platform_device *imx_snd_device;

/* i.MX CPU voltage scaling */
struct pm_circuit imx_cpu_circuit = {
	.name = "imx_cpu",
	.regulator_id = WM8350_DCDC_1,
	.load = PM_LOAD_CPU,
	.mV_range = {
		     .dmin = 1400,
		     .dmax = 1600,
		     },
};

/* i.MX 3stack System IO */
struct pm_circuit imx_io_hi_circuit = {
	.name = "imx_io_hi",
	.regulator_id = WM8350_DCDC_3,
	.load = PM_LOAD_SYS_IO,
	.mV_range = {
		     .sval = 2700,
		     },
};

/* i.MX 3stack System IO */
struct pm_circuit imx_io_lo_circuit = {
	.name = "imx_io_lo",
	.regulator_id = WM8350_DCDC_4,
	.load = PM_LOAD_SYS_IO,
	.mV_range = {
		     .sval = 1800,
		     },
};

/* i.MX 3stack System IO */
struct pm_circuit imx_io_gen_circuit = {
	.name = "imx_io_gen",
	.regulator_id = WM8350_LDO_3,
	.load = PM_LOAD_SYS_IO,
	.mV_range = {
		     .sval = 1500,
		     },
};

/* i.MX 3stack DDR RAM */
struct pm_circuit imx_mem_circuit = {
	.name = "imx_memory",
	.regulator_id = WM8350_DCDC_6,
	.load = PM_LOAD_MEM,
	.mV_range = {
		     .sval = 1800,
		     },
};

/* i.MX 3stack & MMC & MStick */
struct pm_circuit imx_vcam_circuit = {
	.name = "imx_vcam",
	.regulator_id = WM8350_LDO_1,
	.load = PM_LOAD_MMC | PM_LOAD_MSTICK | PM_LOAD_CAMERA,
	.mV_range = {
		     .sval = 2800,
		     },
};

/* i.MX 3stack SIM Card */
struct pm_circuit imx_vsim_circuit = {
	.name = "imx_vsim",
	.regulator_id = WM8350_LDO_2,
	.load = PM_LOAD_SIM | PM_LOAD_AUDIO,
	.mV_range = {
		     .sval = 3300,
		     },
};

/* i.MX 3stack Tranceivers and other peripherals */
struct pm_circuit imx_transceiver_circuit = {
	.name = "imx_transceiver",
	.regulator_id = WM8350_LDO_4,
	.load = PM_LOAD_SERIAL | PM_LOAD_USB | PM_LOAD_IRDA | PM_LOAD_LCD |
	    PM_LOAD_TV | PM_LOAD_DISK | PM_LOAD_MMC,
	.mV_range = {
		     .sval = 2800,
		     },
};

void wm8350_free(struct wm8350 *wm8350)
{
#if BATTERY
	struct wm8350_power *power = &wm8350->power;
#endif

	pm_unregister_circuit(&imx_cpu_circuit);
	pm_unregister_circuit(&imx_io_hi_circuit);
	pm_unregister_circuit(&imx_io_lo_circuit);
	pm_unregister_circuit(&imx_io_gen_circuit);
	pm_unregister_circuit(&imx_mem_circuit);
	pm_unregister_circuit(&imx_vcam_circuit);
	pm_unregister_circuit(&imx_vsim_circuit);
	pm_unregister_circuit(&imx_transceiver_circuit);
#ifdef NOT_PORTED_TO_IMX37
	/* make sure DCDC1 is back at 1.6 volts so we can safely reboot */
	/* DCDC1 @ 1.6 V */
	wm8350_dcdc_set_voltage(&wm8350->pmic, WM8350_DCDC_1, 1600);
	wm8350_dcdc_set_slot(&wm8350->pmic, WM8350_DCDC_1, 1, 0,
			     WM8350_DC1_ERRACT_SHUTDOWN_SYS);
	wm8350_dcdc_enable(&wm8350->pmic, WM8350_DCDC_1, 1);
#endif
	wm8350_mask_irq(wm8350, WM8350_IRQ_GPIO(7));
	wm8350_free_irq(wm8350, WM8350_IRQ_GPIO(7));
	wm8350_mask_irq(wm8350, WM8350_IRQ_WKUP_ONKEY);
	wm8350_free_irq(wm8350, WM8350_IRQ_WKUP_ONKEY);
#if BATTERY
	wm8350_charger_enable(power, 0);
	wm8350_fast_charger_enable(power, 0);
#endif
	if (wm8350->nirq)
		free_irq(wm8350->nirq, wm8350);

	flush_scheduled_work();
#ifdef MXC_DPTC
	mxc_pmic_unregister(&wm8350->pmic);
#endif
	if (wm8350->pmic.dev.is_registered)
		device_unregister(&wm8350->pmic.dev);
	if (wm8350->rtc.dev.is_registered)
		device_unregister(&wm8350->rtc.dev);
	if (wm8350->led.dev.is_registered)
		device_unregister(&wm8350->led.dev);
	if (wm8350->wdg.dev.is_registered)
		device_unregister(&wm8350->wdg.dev);
	if (wm8350->power.dev.is_registered)
		device_unregister(&wm8350->power.dev);
	if (wm8350->backlight.dev.is_registered)
		device_unregister(&wm8350->backlight.dev);
	platform_device_unregister(imx_snd_device);
}

#if BATTERY
static int wm8350_init_battery(struct wm8350 *wm8350)
{
	struct wm8350_power *power = &wm8350->power;
	struct wm8350_charger_policy *policy = &power->policy;

	policy->eoc_mA = WM8350_CHG_EOC_mA(10);
	policy->charge_mV = WM8350_CHG_4_05V;
	policy->fast_limit_mA = WM8350_CHG_FAST_LIMIT_mA(400);
	policy->charge_timeout = WM8350_CHG_TIME_MIN(60);
	policy->trickle_start_mV = WM8350_CHG_TRICKLE_3_1V;
	policy->trickle_charge_mA = WM8350_CHG_TRICKLE_50mA;

	wm8350_charger_enable(power, 1);
	wm8350_fast_charger_enable(power, 1);
	return 0;
}
#endif

#ifdef NOT_PORTED_TO_IMX37_3STACK_YET
static int config_gpios(struct wm8350 *wm8350)
{
	/* power on */
	wm8350_gpio_config(wm8350, 0, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO0_PWR_ON_IN, WM8350_GPIO_ACTIVE_LOW,
			   WM8350_GPIO_PULL_UP, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_ON);

	/* Sw3 --> PWR_OFF_GPIO3 */
	/* lg - TODO: GPIO1_0 to be pulled down */
	wm8350_gpio_config(wm8350, 3, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO3_PWR_OFF_IN, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_DOWN, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_ON);

	/* MR or MEMRST ????? */
	wm8350_gpio_config(wm8350, 4, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO4_MR_IN, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_DOWN, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* Hibernate -- needs level but GPIO5 is edge so we
	 * will only wake with PWR_ON switch atm */
	wm8350_gpio_config(wm8350, 5, WM8350_GPIO_DIR_IN,
			   WM8350_GPIO5_HIBERNATE_IN, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_DOWN, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* SDOUT */
	wm8350_gpio_config(wm8350, 6, WM8350_GPIO_DIR_OUT,
			   WM8350_GPIO6_SDOUT_OUT, WM8350_GPIO_ACTIVE_HIGH,
			   WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* GPIO switch SW2 */
	wm8350_gpio_config(wm8350, 7, WM8350_GPIO_DIR_IN, WM8350_GPIO7_GPIO_IN,
			   WM8350_GPIO_ACTIVE_HIGH, WM8350_GPIO_PULL_DOWN,
			   WM8350_GPIO_INVERT_OFF, WM8350_GPIO_DEBOUNCE_ON);
	wm8350_register_irq(wm8350, WM8350_IRQ_GPIO(7),
			    imx37_3stack_switch_handler);
	wm8350_unmask_irq(wm8350, WM8350_IRQ_GPIO(7));

	/* PWR_FAIL */
	wm8350_gpio_config(wm8350, 8, WM8350_GPIO_DIR_OUT,
			   WM8350_GPIO8_VCC_FAULT_OUT, WM8350_GPIO_ACTIVE_LOW,
			   WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	/* BATT Fault */
	wm8350_gpio_config(wm8350, 9, WM8350_GPIO_DIR_OUT,
			   WM8350_GPIO9_BATT_FAULT_OUT, WM8350_GPIO_ACTIVE_LOW,
			   WM8350_GPIO_PULL_NONE, WM8350_GPIO_INVERT_OFF,
			   WM8350_GPIO_DEBOUNCE_OFF);

	return 0;
}

static int config_hibernate(struct wm8350 *wm8350)
{
	struct wm8350_pmic *pmic = &wm8350->pmic;

	/* dont assert RTS when hibernating */
	wm8350_set_bits(wm8350, WM8350_SYSTEM_HIBERNATE, WM8350_RST_HIB_MODE);

	/* set up hibernate voltages -- needs refining */
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_1, 1400,	/*1200, *//* 1.0v for mx32 */
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_3, 2800,
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_4, 1800,
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_dcdc_set_image_voltage(pmic, WM8350_DCDC_6, 1800,
				      WM8350_DCDC_HIB_MODE_IMAGE,
				      WM8350_DCDC_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_1, 2800,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_2, 3300,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_3, 1500,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_SIG_REG);
	wm8350_ldo_set_image_voltage(pmic, WM8350_LDO_4, 2500,
				     WM8350_LDO_HIB_MODE_IMAGE,
				     WM8350_LDO_HIB_MODE_DIS);
	return 0;
}
#endif				//NOT_PORTED_TO_IMX37_3STACK_YET

int wm8350_init(struct wm8350 *wm8350)
{
	int ret;

	if (program) {
		/* once only...
		 * or if VRTC batt discharges and there is no PIC */
		program_wm8350(wm8350);
		return -EBUSY;
	}

	/* configure system power circuits */
	pm_register_circuit(&imx_cpu_circuit);
	pm_register_circuit(&imx_io_hi_circuit);
	pm_register_circuit(&imx_io_lo_circuit);
	pm_register_circuit(&imx_io_gen_circuit);
	pm_register_circuit(&imx_mem_circuit);
	pm_register_circuit(&imx_vcam_circuit);
	pm_register_circuit(&imx_vsim_circuit);
	pm_register_circuit(&imx_transceiver_circuit);

	/* backlight properties */
	wm8350->backlight.props.max_brightness = 10;
	wm8350->backlight.props.power = FB_BLANK_UNBLANK;
	wm8350->backlight.props.brightness = 5;
	wm8350->backlight.dcdc = WM8350_DCDC_2;
	wm8350->backlight.isink = WM8350_ISINK_B;
	wm8350->backlight.retries = 5;

	/* LED properties */
	wm8350->led.dcdc = WM8350_DCDC_5;
	wm8350->led.isink = WM8350_ISINK_A;
	wm8350->led.retries = 5;
	wm8350->led.half_value = 986376;
	wm8350->led.full_value = 1972752;

	/* RTC periodic irq :-
	 * If we had a dedicated RTC PER IRQ pin we could add it here. */
	wm8350->rtc.per_irq = 0;

	/* now register all i.MX37 3STACK WM8350 client devices */
	wm8350_device_register_pmic(wm8350);
#ifdef NOT_PORTED_TO_IMX37
	wm8350_device_register_rtc(wm8350);
	wm8350_device_register_wdg(wm8350);
	wm8350_device_register_led(wm8350);
	wm8350_device_register_power(wm8350);
	wm8350_device_register_backlight(wm8350);
	/* make sure DCDC1 is back at 1.6 volts so we can safely work */
	/* DCDC1 @ 1.6 V */
	wm8350_dcdc_set_voltage(&wm8350->pmic, WM8350_DCDC_1, 1600);
	wm8350_dcdc_set_slot(&wm8350->pmic, WM8350_DCDC_1, 1, 0,
			     WM8350_DC1_ERRACT_SHUTDOWN_SYS);
	wm8350_dcdc_enable(&wm8350->pmic, WM8350_DCDC_1, 1);
#endif
	/* register sound */
	printk("Registering imx37_snd_device");
	imx_snd_device = platform_device_alloc(imx37_3stack_audio, -1);
	if (!imx_snd_device) {
		ret = -ENOMEM;
		goto err;
	}
	platform_set_drvdata(imx_snd_device, &wm8350->audio);
	ret = platform_device_add(imx_snd_device);
	if (ret)
		goto snd_err;

	/* set up PMIC IRQ (active high) to i.MX32ADS */
#ifdef NOT_PORTED_TO_IMX37
	printk("Registering PMIC INT");
	INIT_WORK(&wm8350->work, wm8350_irq_work);
	wm8350_reg_unlock(wm8350);
	wm8350_set_bits(wm8350, WM8350_SYSTEM_CONTROL_1, WM8350_IRQ_POL);
	wm8350_reg_lock(wm8350);
	set_irq_type(MXC_PMIC_INT_LINE, IRQT_RISING);
	ret = request_irq(MXC_PMIC_INT_LINE, wm8350_irq_handler,
			  IRQF_DISABLED, "wm8350-pmic", wm8350);
	if (ret != 0) {
		printk(KERN_ERR "wm8350: cant request irq %d\n",
		       MXC_PMIC_INT_LINE);
		goto err;
	}
	wm8350->nirq = MXC_PMIC_INT_LINE;
	printk("Configuring WM8350 GPIOS");
	config_gpios(wm8350);
	config_hibernate(wm8350);

	/* Sw1 --> PWR_ON */
	printk("Registering and unmasking the WM8350 wakeup key");
	wm8350_register_irq(wm8350, WM8350_IRQ_WKUP_ONKEY,
			    imx37_3stack_switch_handler);
	wm8350_unmask_irq(wm8350, WM8350_IRQ_WKUP_ONKEY);

	/* unmask all & clear sticky */
	printk("Unmasking WM8350 local interrupts");
	wm8350_reg_write(wm8350, WM8350_SYSTEM_INTERRUPTS_MASK, 0x0);
	schedule_work(&wm8350->work);
#endif

#ifdef MXC_DPTC
	/* DPTC reg */
	mxc_pmic_register(&wm8350->pmic, wm8350_dcdc_set_voltage);
#endif

#if BATTERY
	/* not much use without a battery atm */
	wm8350_init_battery(wm8350);
#endif
	printk("Exiting normally from wm8350_init()");
	return ret;
      snd_err:
	platform_device_put(imx_snd_device);
      err:
	printk("wm8350_init() FAILED");
	kfree(wm8350->reg_cache);
	return ret;
}
