/*
 * wm8350_pmu.c  --  Power Managment Driver for Wolfson WM8350 PMIC
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    23rd Jan 2007   Initial version.
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
#include <linux/pmic.h>

#define WM8350_PMIC_VERSION	"0.3"

/* debug */
#define WM8350_DEBUG 0
#if WM8350_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

static ssize_t pmic_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct wm8350_pmic *pmic = to_wm8350_pmic_device(dev);
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int i = 0, count = 0;

	for (;i < WM8350_MAX_REGISTER + 1; i++)
	//	count += sprintf(buf, "R%d = 0x%2.2x%2.2x\n", i >> 1, reg[i], reg[i+1]);
		printk("R%d = 0x%4.4x\n", i,  wm8350_reg_read(wm8350,i)); // hack to fix

	return count;
}
static DEVICE_ATTR(pmic_reg, 0444, pmic_reg_show, NULL);

/* hundredths of uA, 405 = 4.05 uA */
static const int isink_cur[] = {
	405, 482, 573, 681, 810, 963, 1146, 1362, 1620, 1927, 2291, 2725,
	3240, 3853, 4582, 5449, 6480, 7706, 9164, 10898, 12960, 15412, 18328,
	21796, 25920, 30824, 36656, 43592, 51840, 61648, 73313, 87184,
	103680, 123297, 146626, 174368, 207360, 246594, 293251, 348737, 
	414720, 493188, 586503, 697473, 829440, 986376, 1173005, 1394946,
	1658880, 1972752, 2346011, 2789892, 3317760, 3945504, 4692021,
	5579785, 6635520, 7891008, 9384042, 11159570, 13271040, 15782015,
	18768085, 22319140,
};

static int get_isink_val(int huA)
{
	int i;
	
	for (i = ARRAY_SIZE(isink_cur); i >= 0 ; i--) {
		if (huA > isink_cur[i])
			return i;
	}
	return 0;
}

int wm8350_isink_set_current(struct wm8350_pmic *pmic, int isink, 
	int huA)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	u16 val;
	
	switch (isink) {
	case WM8350_ISINK_A:
		val = wm8350_reg_read(wm8350, WM8350_CURRENT_SINK_DRIVER_A) & 
			~WM8350_CS1_ISEL_MASK;
		wm8350_reg_write(wm8350, WM8350_CURRENT_SINK_DRIVER_A, val | 
			get_isink_val(huA));
		//printk("val %x\n", wm8350_reg_read(wm8350, WM8350_CURRENT_SINK_DRIVER_A));
		break;
	case WM8350_ISINK_B:
		val = wm8350_reg_read(wm8350, WM8350_CURRENT_SINK_DRIVER_B) & 
			~WM8350_CS1_ISEL_MASK;
		wm8350_reg_write(wm8350, WM8350_CURRENT_SINK_DRIVER_B, val | 
			get_isink_val(huA));
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_isink_set_current);

int wm8350_isink_enable(struct wm8350_pmic *pmic, int isink, int enable, 
	int hibernate_enable)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	
	switch (isink) {
	case WM8350_ISINK_A:
		if (enable)
			wm8350_set_bits(wm8350, WM8350_POWER_MGMT_7, 
				WM8350_CS1_ENA);
		else {
			wm8350_clear_bits(wm8350, WM8350_POWER_MGMT_7, 
				WM8350_CS1_ENA);
		}	
		if (hibernate_enable)
			wm8350_set_bits(wm8350, WM8350_CURRENT_SINK_DRIVER_A, 
				WM8350_CS1_HIB_MODE);
		else
			wm8350_clear_bits(wm8350, WM8350_CURRENT_SINK_DRIVER_A, 
				WM8350_CS1_HIB_MODE);	
		break;
	case WM8350_ISINK_B:
		if (enable)
			wm8350_set_bits(wm8350, WM8350_POWER_MGMT_7, 
				WM8350_CS2_ENA);
		else {
			wm8350_clear_bits(wm8350, WM8350_POWER_MGMT_7, 
				WM8350_CS2_ENA);
		}	
		if (hibernate_enable)
			wm8350_set_bits(wm8350, WM8350_CURRENT_SINK_DRIVER_B, 
				WM8350_CS2_HIB_MODE);
		else
			wm8350_clear_bits(wm8350, WM8350_CURRENT_SINK_DRIVER_B, 
				WM8350_CS2_HIB_MODE);	
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_isink_enable);

int wm8350_isink_set_flash(struct wm8350_pmic *pmic, int isink, u16 mode, 
	u16 trigger, u16 duration, u16 on_ramp, u16 off_ramp, u16 drive)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	
	switch (isink) {
	case WM8350_ISINK_A:
		wm8350_reg_write(wm8350, WM8350_CSA_FLASH_CONTROL, 
			(mode ? WM8350_CS1_FLASH_MODE : 0) | 
			(trigger ? WM8350_CS1_TRIGSRC : 0) |
			duration | on_ramp | off_ramp | drive);
		break;
	case WM8350_ISINK_B:
		wm8350_reg_write(wm8350, WM8350_CSB_FLASH_CONTROL, 
			(mode ? WM8350_CS2_FLASH_MODE : 0) | 
			(trigger ? WM8350_CS2_TRIGSRC : 0) |
			duration | on_ramp | off_ramp | drive);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_isink_set_flash);

int wm8350_isink_trigger_flash(struct wm8350_pmic *pmic, int isink)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	u16 val;
	
	switch (isink) {
	case WM8350_ISINK_A:
		val = wm8350_reg_read(wm8350, WM8350_CSA_FLASH_CONTROL) & 
			~WM8350_CS1_DRIVE;
		wm8350_reg_write(wm8350, WM8350_CSA_FLASH_CONTROL, val | 
			WM8350_CS1_DRIVE);
		break;
	case WM8350_ISINK_B:
		val = wm8350_reg_read(wm8350, WM8350_CSA_FLASH_CONTROL) & 
			~WM8350_CS2_DRIVE;
		wm8350_reg_write(wm8350, WM8350_CSA_FLASH_CONTROL, val | 
			WM8350_CS2_DRIVE);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int wm8350_dcdc_set_voltage(struct wm8350_pmic *pmic, int dcdc, int mV)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int volt_reg;
	u16 val;
	
	dbg("%s %d mV %d\n", __FUNCTION__, dcdc, mV);
	
	if (mV < 850 || mV > 4025) {
		printk(KERN_ERR "wm8350: dcdc %d voltage %d mV out of range\n", 
			dcdc, mV);
		return -EINVAL;
	}
	
	switch (dcdc) {
	case WM8350_DCDC_1:
		volt_reg = WM8350_DCDC1_CONTROL;
		break;
	case WM8350_DCDC_3:
		volt_reg = WM8350_DCDC3_CONTROL;
		break;
	case WM8350_DCDC_4:
		volt_reg = WM8350_DCDC4_CONTROL;
		break;
	case WM8350_DCDC_6:
		volt_reg = WM8350_DCDC6_CONTROL;
		break;
	case WM8350_DCDC_2:
	case WM8350_DCDC_5:
	default:
		return -EINVAL;	
	}
	
	/* all DCDC's have same mV bits */
	val = wm8350_reg_read(wm8350, volt_reg) & ~WM8350_DC1_VSEL_MASK;
	wm8350_reg_write(wm8350, volt_reg, val | WM8350_DC1_VSEL(mV));
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_dcdc_set_voltage);

int wm8350_dcdc_set_image_voltage(struct wm8350_pmic *pmic, int dcdc, int mV,
	int mode, int signal)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int volt_reg;
	
	dbg("%s %d mV %d\n", __FUNCTION__, dcdc, mV);
	
	if (mV && (mV < 850 || mV > 4025)) {
		printk(KERN_ERR "wm8350: dcdc %d image voltage %d mV out of range\n", 
			dcdc, mV);
		return -EINVAL;
	}
	if (mV == 0)
		mV = 850;
	
	switch (dcdc) {
	case WM8350_DCDC_1:
		volt_reg = WM8350_DCDC1_LOW_POWER;
		break;
	case WM8350_DCDC_3:
		volt_reg = WM8350_DCDC3_LOW_POWER;
		break;
	case WM8350_DCDC_4:
		volt_reg = WM8350_DCDC4_LOW_POWER;
		break;
	case WM8350_DCDC_6:
		volt_reg = WM8350_DCDC6_LOW_POWER;
		break;
	case WM8350_DCDC_2:
	case WM8350_DCDC_5:
	default:
		return -EINVAL;	
	}
	
	/* all DCDC's have same mV bits */
	wm8350_reg_write(wm8350, volt_reg, WM8350_DC1_VSEL(mV) | mode | signal);
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_dcdc_set_image_voltage);

int wm8350_ldo_set_voltage(struct wm8350_pmic *pmic, int ldo, int mV)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int volt_reg;
	u16 val;
	
	dbg("%s %d mV %d\n", __FUNCTION__, ldo, mV);
	
	if (mV < 900 || mV > 3300) {
		printk(KERN_ERR "wm8350: ldo %d voltage %d mV out of range\n", 
			ldo, mV);
		return -EINVAL;
	}
	
	switch (ldo) {
	case WM8350_LDO_1:
		volt_reg = WM8350_LDO1_CONTROL;
		break;
	case WM8350_LDO_2:
		volt_reg = WM8350_LDO2_CONTROL;
		break;
	case WM8350_LDO_3:
		volt_reg = WM8350_LDO3_CONTROL;
		break;
	case WM8350_LDO_4:
		volt_reg = WM8350_LDO4_CONTROL;
		break;
	default:
		return -EINVAL;	
	}
	
	/* all LDO's have same mV bits */
	val = wm8350_reg_read(wm8350, volt_reg) & ~WM8350_LDO1_VSEL_MASK;
	wm8350_reg_write(wm8350, volt_reg, val | WM8350_LDO1_VSEL(mV));
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_ldo_set_voltage);

int wm8350_ldo_set_image_voltage(struct wm8350_pmic *pmic, int ldo, int mV,
	int mode, int signal)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int volt_reg;

	
	dbg("%s %d mV %d\n", __FUNCTION__, ldo, mV);
	
	if (mV < 900 || mV > 3300) {
		printk(KERN_ERR "wm8350: ldo %d voltage %d mV out of range\n", 
			ldo, mV);
		return -EINVAL;
	}
	
	switch (ldo) {
	case WM8350_LDO_1:
		volt_reg = WM8350_LDO1_LOW_POWER;
		break;
	case WM8350_LDO_2:
		volt_reg = WM8350_LDO2_LOW_POWER;
		break;
	case WM8350_LDO_3:
		volt_reg = WM8350_LDO3_LOW_POWER;
		break;
	case WM8350_LDO_4:
		volt_reg = WM8350_LDO4_LOW_POWER;
		break;
	default:
		return -EINVAL;	
	}
	
	/* all LDO's have same mV bits */
	wm8350_reg_write(wm8350, volt_reg, WM8350_LDO1_VSEL(mV) | mode | signal);
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_ldo_set_image_voltage);

int wm8350_dcdc_set_slot(struct wm8350_pmic *pmic, int dcdc, u16 start, 
	u16 stop, u16 fault)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int slot_reg;
	u16 val;
	
	dbg("%s %d start %d stop %d\n", __FUNCTION__, dcdc, start, stop);
	
	/* slot valid ? */
	if (start > 15 || stop > 15)
		return -EINVAL; 
	
	switch (dcdc) {
	case WM8350_DCDC_1:
		slot_reg = WM8350_DCDC1_TIMEOUTS;
		break;
	case WM8350_DCDC_2:
		slot_reg = WM8350_DCDC2_TIMEOUTS;
		break;
	case WM8350_DCDC_3:
		slot_reg = WM8350_DCDC3_TIMEOUTS;
		break;
	case WM8350_DCDC_4:
		slot_reg = WM8350_DCDC4_TIMEOUTS;
		break;
	case WM8350_DCDC_5:
		slot_reg = WM8350_DCDC5_TIMEOUTS;
		break;
	case WM8350_DCDC_6:
		slot_reg = WM8350_DCDC6_TIMEOUTS;
		break;
	default:
		return -EINVAL;	
	}
	
	val = wm8350_reg_read(wm8350, slot_reg) & 
		~(WM8350_DC1_ENSLOT_MASK | WM8350_DC1_SDSLOT_MASK | 
		WM8350_DC1_ERRACT_MASK);
	wm8350_reg_write(wm8350, slot_reg, 
		val | (start << WM8350_DC1_ENSLOT_SHIFT) | 
		(stop << WM8350_DC1_SDSLOT_SHIFT) | 
		(fault << WM8350_DC1_ERRACT_SHIFT));
	
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_dcdc_set_slot);

int wm8350_ldo_set_slot(struct wm8350_pmic *pmic, int ldo, u16 start, 
	u16 stop)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	int slot_reg;
	u16 val;
	
	dbg("%s %d start %d stop %d\n", __FUNCTION__, ldo, start, stop);
	
	/* slot valid ? */
	if (start > 15 || stop > 15)
		return -EINVAL; 
	
	switch (ldo) {
	case WM8350_LDO_1:
		slot_reg = WM8350_LDO1_TIMEOUTS;
		break;
	case WM8350_LDO_2:
		slot_reg = WM8350_LDO2_TIMEOUTS;
		break;
	case WM8350_LDO_3:
		slot_reg = WM8350_LDO3_TIMEOUTS;
		break;
	case WM8350_LDO_4:
		slot_reg = WM8350_LDO4_TIMEOUTS;
		break;
	default:
		return -EINVAL;	
	}
	
	val = wm8350_reg_read(wm8350, slot_reg) & ~WM8350_LDO1_SDSLOT_MASK;
	wm8350_reg_write(wm8350, slot_reg, val | ((start << 10) | (stop << 6)));
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_ldo_set_slot);

int wm8350_dcdc_set_mode(struct wm8350_pmic *pmic, int dcdc, u16 active, 
	u16 sleep)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	u16 shift;
	
	if (dcdc == WM8350_DCDC_2 || dcdc == WM8350_DCDC_5)
		return -EINVAL;
	
	dbg("%s %d mode: %s %s\n", __FUNCTION__, dcdc, active ? "on" : "off",
		sleep ? "on" : "off");
	
	shift = dcdc - WM8350_DCDC_1;
	
	if (active)
		wm8350_set_bits(wm8350, WM8350_DCDC_ACTIVE_OPTIONS, 1 << shift);
	else
		wm8350_clear_bits(wm8350, WM8350_DCDC_ACTIVE_OPTIONS, 1 << shift);
	
	if (sleep)
		wm8350_set_bits(wm8350, WM8350_DCDC_SLEEP_OPTIONS, 1 << shift);
	else
		wm8350_clear_bits(wm8350, WM8350_DCDC_SLEEP_OPTIONS, 1 << shift);
		
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_dcdc_set_mode);

int wm8350_dcdc25_set_mode(struct wm8350_pmic *pmic, int dcdc, u16 mode, 
	u16 ilim, u16 ramp, u16 feedback)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	u16 val;
	
	dbg("%s %d mode: %s %s\n", __FUNCTION__, dcdc, mode ? "normal" : "boost",
		ilim ? "low" : "normal");
	
	switch (dcdc) {
	case WM8350_DCDC_2:
		val = wm8350_reg_read(wm8350, WM8350_DCDC2_CONTROL)
			 & ~(WM8350_DC2_MODE_MASK | WM8350_DC2_ILIM_MASK |
			 WM8350_DC2_RMP_MASK | WM8350_DC2_FBSRC_MASK);
		wm8350_reg_write(wm8350, WM8350_DCDC2_CONTROL, val | 
			(mode << WM8350_DC2_MODE_SHIFT) | 
			(ilim << WM8350_DC2_ILIM_SHIFT) |
			(ramp << WM8350_DC2_RMP_SHIFT) |
			(feedback << WM8350_DC2_FBSRC_SHIFT));
		break;
	case WM8350_DCDC_5:
		val = wm8350_reg_read(wm8350, WM8350_DCDC5_CONTROL)
			 & ~(WM8350_DC5_MODE_MASK | WM8350_DC5_ILIM_MASK |
			 WM8350_DC5_RMP_MASK | WM8350_DC5_FBSRC_MASK);
		wm8350_reg_write(wm8350, WM8350_DCDC5_CONTROL, val | 
			(mode << WM8350_DC5_MODE_SHIFT) | 
			(ilim << WM8350_DC5_ILIM_SHIFT) |
			(ramp << WM8350_DC5_RMP_SHIFT) |
			(feedback << WM8350_DC5_FBSRC_SHIFT));
		break;
	default:
		return -EINVAL;
	}
		
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_dcdc25_set_mode);

int wm8350_dcdc_enable(struct wm8350_pmic *pmic, int dcdc, int enable)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	u16 shift;
	
	dbg("%s %d --> %s\n", __FUNCTION__, dcdc, enable ? "on" : "off");
	
	if (dcdc < WM8350_DCDC_1 || dcdc > WM8350_DCDC_6)
		return -EINVAL;
	
	shift = dcdc - WM8350_DCDC_1;
	
	if (enable)
		wm8350_set_bits(wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << shift);
	else
		wm8350_clear_bits(wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << shift);
	
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_dcdc_enable);

int wm8350_ldo_enable(struct wm8350_pmic *pmic, int ldo, int enable)
{
	struct wm8350 *wm8350 = to_wm8350_from_pmic(pmic);
	u16 shift;
	
	dbg("%s %d --> %s\n", __FUNCTION__, ldo, enable ? "on" : "off");
	
	if (ldo < WM8350_LDO_1 || ldo > WM8350_LDO_4)
		return -EINVAL;
	
	shift = (ldo - WM8350_LDO_1) + 8;

	if (enable)
		wm8350_set_bits(wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << shift);
	else
		wm8350_clear_bits(wm8350, WM8350_DCDC_LDO_REQUESTED, 1 << shift);
	
	return 0;
}
EXPORT_SYMBOL_GPL(wm8350_ldo_enable);

static int set_voltage (struct pm_regulator *reg, int mV, int Vid)
{
	struct wm8350_pmic *pmic = (struct wm8350_pmic*)reg->private;
	int ret = 0;
	
	switch (reg->id) {
	case WM8350_DCDC_1:
	case WM8350_DCDC_2:
	case WM8350_DCDC_3:
	case WM8350_DCDC_4:
	case WM8350_DCDC_5:
	case WM8350_DCDC_6:
		ret = wm8350_dcdc_set_voltage(pmic, reg->id, mV);
		break;
	case WM8350_LDO_1:
	case WM8350_LDO_2:
	case WM8350_LDO_3:
	case WM8350_LDO_4:
		ret = wm8350_ldo_set_voltage(pmic, reg->id, mV);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int enable (struct pm_regulator *reg, int enable)
{
	struct wm8350_pmic *pmic = (struct wm8350_pmic*)reg->private;
	int ret = 0;
	
	switch (reg->id) {
	case WM8350_DCDC_1:
	case WM8350_DCDC_2:
	case WM8350_DCDC_3:
	case WM8350_DCDC_4:
	case WM8350_DCDC_5:
	case WM8350_DCDC_6:
		ret = wm8350_dcdc_enable(pmic, reg->id, enable);
		break;
	case WM8350_LDO_1:
	case WM8350_LDO_2:
	case WM8350_LDO_3:
	case WM8350_LDO_4:
		ret = wm8350_ldo_enable(pmic, reg->id, enable);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

struct pm_regulator_ops wm8350_pmic_ops = {
	.set_voltage	= set_voltage,
	.enable		= enable,
};

struct pm_regulator wm8350_dcdc1 = {
	.name	= "DCDC1",
	.id	= WM8350_DCDC_1,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 850,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_dcdc6 = {
	.name	= "DCDC6",
	.id	= WM8350_DCDC_6,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 850,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_dcdc3 = {
	.name	= "DCDC3",
	.id	= WM8350_DCDC_3,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 850,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_dcdc4 = {
	.name	= "DCDC4",
	.id	= WM8350_DCDC_4,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 850,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_dcdc2 = {
	.name	= "DCDC2",
	.id	= WM8350_DCDC_2,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 5000,
		.dmax	= 20000,
	},
	.mV_control	= PM_CONTROL_NONE,
	.mA_range	= {
		.dmin	= 0,
		.dmax	= 224,
	},
	.mA_control	= PM_CONTROL_DYNAMIC,
};

struct pm_regulator wm8350_dcdc5 = {
	.name	= "DCDC5",
	.id	= WM8350_DCDC_5,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 5000,
		.dmax	= 20000,
	},
	.mV_control	= PM_CONTROL_NONE,
	.mA_range	= {
		.dmin	= 0,
		.dmax	= 224,
	},
	.mA_control	= PM_CONTROL_DYNAMIC,
};

struct pm_regulator wm8350_ldo1 = {
	.name	= "LDO1",
	.id	= WM8350_LDO_1,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 900,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_ldo2 = {
	.name	= "LDO2",
	.id	= WM8350_LDO_2,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 900,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_ldo3 = {
	.name	= "LDO3",
	.id	= WM8350_LDO_3,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 900,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};

struct pm_regulator wm8350_ldo4 = {
	.name	= "LDO4",
	.id	= WM8350_LDO_4,
	.ops	= &wm8350_pmic_ops,
	.mV_range	= {
		.dmin	= 900,
		.dmax	= 3400,
	},
	.mV_control	= PM_CONTROL_DYNAMIC,
	.mA_control	= PM_CONTROL_NONE,
};


static int wm8350_probe(struct device *dev)
{
	int err;
	
	printk(KERN_INFO "WM8350 PMIC driver version %s\n", WM8350_PMIC_VERSION);
	err = device_create_file(dev, &dev_attr_pmic_reg);
	if (err < 0)
		printk(KERN_WARNING "asoc: failed to add pmic sysfs entries\n");
	pm_register_regulator(&wm8350_dcdc1, dev);
	pm_register_regulator(&wm8350_dcdc2, dev);
	pm_register_regulator(&wm8350_dcdc3, dev);
	pm_register_regulator(&wm8350_dcdc4, dev);
	pm_register_regulator(&wm8350_dcdc5, dev);
	pm_register_regulator(&wm8350_dcdc6, dev);
	pm_register_regulator(&wm8350_ldo1, dev);
	pm_register_regulator(&wm8350_ldo2, dev);
	pm_register_regulator(&wm8350_ldo3, dev);
	pm_register_regulator(&wm8350_ldo4, dev);
	return 0;
}

static int wm8350_remove (struct device *dev)
{
	pm_unregister_regulator(&wm8350_ldo4);
	pm_unregister_regulator(&wm8350_ldo3);
	pm_unregister_regulator(&wm8350_ldo2);
	pm_unregister_regulator(&wm8350_ldo1);
	pm_unregister_regulator(&wm8350_dcdc6);
	pm_unregister_regulator(&wm8350_dcdc5);
	pm_unregister_regulator(&wm8350_dcdc4);
	pm_unregister_regulator(&wm8350_dcdc3);
	pm_unregister_regulator(&wm8350_dcdc2);
	pm_unregister_regulator(&wm8350_dcdc1);
	device_remove_file(dev, &dev_attr_pmic_reg);
	return 0;
}

static void wm8350_shutdown (struct device *dev)
{
}

static int wm8350_suspend (struct device *dev, pm_message_t state)
{
	return 0;
}

static int wm8350_resume (struct device *dev)
{
	return 0;
}

struct device_driver wm8350_driver = {
	.name = "wm8350-pmic",
	.bus = &wm8350_bus_type,
	.owner = THIS_MODULE,
	.probe = wm8350_probe,
	.remove = wm8350_remove,
	.shutdown = wm8350_shutdown,
	.suspend = wm8350_suspend,
	.resume = wm8350_resume,
};

static int __devinit wm8350_pmu_init(void)
{
	return driver_register(&wm8350_driver);
}

static void wm8350_pmu_exit(void)
{
 	driver_unregister(&wm8350_driver);
}

module_init(wm8350_pmu_init);
module_exit(wm8350_pmu_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("WM8350 PMIC driver");
MODULE_LICENSE("GPL");
