/*
 * da9052-battery.c  --  Batttery Driver for Dialog DA9052 on a CCXMX53 JSK.
 *
 * Copyright(c) 2009 Dialog Semiconductor Ltd.
 * Copyright(c) 2011 Digi International Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/freezer.h>

#include <linux/mfd/da9052/da9052.h>
#include <linux/mfd/da9052/reg.h>
#include <linux/mfd/da9052/bat_ccxmx53js.h>
#include <linux/mfd/da9052/adc.h>

#define DA9052_BAT_DEVICE_NAME			"da9052-bat"

static const char  __initdata banner[] = KERN_INFO "DA9052 battery driver on "\
		"a CCXMX53 JSK.";

static struct da9052_bat_device bat_info;
static struct da9052_bat_status bat_status;
static struct monitoring_state monitoring_status;
struct power_supply_info battery_info;

struct da9052_charger_device charger;

static u16 bat_target_voltage;
static u16 filter_size = FILTER_SIZE;

static int da9052_bat_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val);

s32 monitoring_thread_pid;
u8 monitoring_thread_state = ACTIVE;
struct completion monitoring_thread_notifier;

static enum power_supply_property da902_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static int da9052_read(struct da9052 *da9052, u8 reg_address, u8 *reg_data)
{
	struct da9052_ssc_msg msg;
	int ret;

	msg.addr = reg_address;
	msg.data = 0;

	da9052_lock(da9052);
	ret = da9052->read(da9052, &msg);
	if (ret)
		goto ssc_comm_err;
	da9052_unlock(da9052);

	*reg_data = msg.data;
	return 0;
ssc_comm_err:
	da9052_unlock(da9052);
	return ret;
}

static int da9052_write(struct da9052 *da9052, u8 reg_address, u8 reg_data)
{
	struct da9052_ssc_msg msg;
	int ret;

	msg.addr = reg_address;
	msg.data = reg_data;

	da9052_lock(da9052);
	ret = da9052->write(da9052, &msg);
	if (ret)
		goto ssc_comm_err;
	da9052_unlock(da9052);

	return 0;
ssc_comm_err:
	da9052_unlock(da9052);
	return ret;
}

static int pmic_write( struct da9052 *da9052, u8 reg_address, u8 reg_mask , u8 reg_data )
{
	u8 value;
	int ret = 0;

	if( (ret = da9052_read( da9052 , reg_address , &value )) == 0 ){
		value = (value & ~reg_mask) | reg_data;
		if( (ret = da9052_write( da9052 , reg_address , value )) )
			pr_err("Error writing %08x into %d\n",value,reg_address);
	}
	else
		pr_err("Error reading from %d\n",reg_address);

	return ret;;
}

static s32 da9052_adc_read_ich(struct da9052 *da9052, u16 *data)
{
	struct da9052_ssc_msg msg;
	da9052_lock(da9052);
	/* Read charging conversion register */
	msg.addr = DA9052_ICHGAV_REG;
	msg.data = 0;
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}
	da9052_unlock(da9052);

	*data = (u16)msg.data;
	DA9052_DEBUG("Read 0x%08x\n", msg.data);
	return SUCCESS;
}

#if 0
static s32 da9052_adc_read_vddout(struct da9052 *da9052, u16 *data)
{
	u8 reg_data;
	s32 ret;

	ret = da9052_read(da9052, DA9052_ADCCONT_REG, &reg_data);
	if (ret)
		return ret;

	if (!(reg_data & DA9052_ADCCONT_AUTOVDDEN)) {
		reg_data = (reg_data | DA9052_ADCCONT_AUTOVDDEN);

		ret = da9052_write(da9052, DA9052_INPUTCONT_REG, reg_data);
		if (ret)
			return ret;
		reg_data = 0x0;

		ret = da9052_read(da9052, DA9052_ADCCONT_REG, &reg_data);
		if (ret)
			return ret;

		if (reg_data & DA9052_ADCCONT_ADCMODE)
			msleep(1);
		else
			msleep(10);

		ret = da9052_read(da9052, DA9052_VDDRES_REG, &reg_data);
		if (ret)
			return ret;

		*data = (u16)reg_data;

		ret = da9052_read(da9052, DA9052_ADCCONT_REG, &reg_data);
		if (ret)
			return ret;

		reg_data = reg_data & ~(DA9052_ADCCONT_AUTOVDDEN);
		ret = da9052_write(da9052, DA9052_ADCCONT_REG, reg_data);
		if (ret)
			return ret;
	} else {
		ret = da9052_read(da9052, DA9052_VDDRES_REG, &reg_data);
		if (ret)
			return ret;

		*data = (u16)reg_data;
	}
	return 0;
}

s32 da9052_adc_read_tjunc(struct da9052 *da9052, u16 *data)
{
	struct da9052_ssc_msg msg;
	u16 temp;

	da9052_lock(da9052);

	/* Read TJunction conversion result */
	msg.addr = DA9052_TJUNCRES_REG;
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}
	temp = msg.data;
	/* Read calibration for junction temperature */
	msg.addr = DA9052_TOFFSET_REG;
	if (da9052->read(da9052, &msg)) {
		da9052_unlock(da9052);
		return DA9052_SSC_FAIL;
	}

	da9052_unlock(da9052);
	/* Calculate Junction temperature */
	*data = temp - (u16)msg.data;
	DA9052_DEBUG("Read 0x%08x\n", *data);
	return 0;
}
#endif

s32 da9052_adc_read_vbat(struct da9052 *da9052, u16 *data)
{
	s32 ret;

	ret = da9052_manual_read(da9052, DA9052_ADC_VBAT);
	if (ret == -EIO) {
		*data = 0;
		return ret;
	} else {
		*data = ret;
	}
	DA9052_DEBUG("Read 0x%08x\n", *data);
	return 0;
}


static u16 filter_sample(u16 *buffer)
{
	u8 count;
	u16 tempvalue = 0;
	u16 ret;

	if (buffer == NULL)
		return -EINVAL;

	for (count = 0; count < filter_size; count++)
		tempvalue = tempvalue + *(buffer + count);

	ret = tempvalue/filter_size;
	return ret;
}


#if 0
static s32 da9052_bat_get_charger_vddout(struct da9052 *da9052, u16 *buffer)
{

	u8 count;
	u16 filterqueue[filter_size];

	if (bat_status.status != DA9052_CHARGING_CC ||
			bat_status.status != DA9052_CHARGING_CV)
		return -EIO;

	/* Measure the charger voltage using ADC function. Number
					of read equal to average filter size*/
	for (count = 0; count < filter_size; count++)
		if (da9052_adc_read_vddout(da9052, &filterqueue[count]))
			return -EIO;

	/*Apply average filter */
	filterqueue[0] = filter_sample(filterqueue);
	/* Convert the charger voltage in terms of mV */
	bat_info.vddout = vddout_reg_to_mV(filterqueue[0]);
	*buffer = bat_info.vddout;

	return SUCCESS;
}
#endif

static s32  da9052_bat_get_chg_current(struct da9052 *da9052, u16 *buffer)
{

	if (bat_status.status == DA9052_DISCHARGING_WITHOUT_CHARGER)
		return -EIO;

	/* Measure the Charger current using ADC function */
	if (da9052_adc_read_ich(da9052, buffer)){
		*buffer=0;
		return -EIO;
	}

	/* Convert the raw value in terms of mA */
	bat_info.chg_current = ichg_reg_to_mA(*buffer);
	*buffer = bat_info.chg_current;

	DA9052_DEBUG("bat_info.chg_current %08x\n",bat_info.chg_current);
	return 0;
}

s32  da9052_bat_get_battery_voltage(struct da9052 *da9052, u16 *buffer)
{

	u8 count;
	u16 filterqueue[filter_size];

	/* Measure the battery voltage using ADC function.
		Number of read equal to average filter size*/
	for (count = 0; count < filter_size; count++)
		if (da9052_adc_read_vbat(da9052, &filterqueue[count]))
			return -EIO;

	/* Apply average filter */
	filterqueue[0] = filter_sample(filterqueue);

	/* Convert battery voltage raw value in terms of mV */
	bat_info.bat_voltage = volt_reg_to_mV(filterqueue[0]);
	*buffer = bat_info.bat_voltage;
	DA9052_DEBUG("bat_info.bat_voltage %08x\n",bat_info.bat_voltage);
	return 0;
}

#if DA9052_BAT_DEBUG
void get_bat_mode(u8 mode_num, char *temp_name)
{
	if (mode_num == DA9052_NONE)
		sprintf(temp_name, "NONE");
	else if (mode_num == DA9052_CHARGING_CC || mode_num == DA9052_CHARGING_CV)
		sprintf(temp_name, "CHARGING");
	else if (mode_num == DA9052_DISCHARGING_WITH_CHARGER)
		sprintf(temp_name, "DISCHARGING_WITH_CHARGER");
	else if (mode_num == DA9052_DISCHARGING_WITHOUT_CHARGER)
		sprintf(temp_name, "DISCHARGING_WITHOUT_CHARGER");
}
#endif

#if 0
void get_charging_mode(u8 mode, char *temp_name)
{
	if (mode == DA9052_NONE)
		sprintf(temp_name, "NONE");
	else if (mode == DA9052_PRECHARGING)
		sprintf(temp_name, "PRECHARGING");
	else if (mode == DA9052_LINEARCHARGING)
		sprintf(temp_name, "LINEARCHARGING");
	else if (mode == DA9052_CHARGEEND)
		sprintf(temp_name, "CHARGEEND");
}

void get_charger_type(u8 type, char *temp_name)
{
	if (type == DA9052_USB_HUB)
		sprintf(temp_name, "USB_HUB");
	else if (type == DA9052_NOCHARGER)
		sprintf(temp_name, "NOCHARGER");
	else if (type == DA9052_USB_CHARGER)
		sprintf(temp_name, "USB_CHARGER");
	else if (type == DA9052_WALL_CHARGER)
		sprintf(temp_name, "WALL_CHARGER");
}
#endif

static void da9052_charger_status_update(struct da9052_charger_device
	*chg_device)
{
	u8 regvalue = 0;
	u16 ichgm = 0;
	u16 vbat = 0;
	static int bat_status_old;

	// Will return error and ichgm=0 if no charger present
	da9052_bat_get_chg_current(chg_device->da9052,&ichgm);
	if( da9052_bat_get_battery_voltage(chg_device->da9052, &vbat) )
		goto read_err;

	if( da9052_read(chg_device->da9052, DA9052_STATUSA_REG , &regvalue ))
		goto read_err;

	if( (regvalue & DA9052_STATUSA_DCINSEL) == 0 ){
		bat_status.charger_type = DA9052_NOCHARGER;
		bat_status.status = DA9052_DISCHARGING_WITHOUT_CHARGER;
	}
	else if ( vbat > VCHARGE_MAX && ichgm < ICHARGE_END ){
		bat_status.charger_type = DA9052_WALL_CHARGER;
		bat_status.status = DA9052_CHARGEEND;
	}
	else if ( vbat > VCHARGE_MAX && ichgm > ICHARGE_END ){
		bat_status.charger_type = DA9052_WALL_CHARGER;
		bat_status.status = DA9052_CHARGING_CV;
	}
	else if ( vbat > VDISCHG_MIN && ichgm > ICHARGE_CC ){
		bat_status.charger_type = DA9052_WALL_CHARGER;
		bat_status.status = DA9052_CHARGING_CC;
	}
	else if ( vbat > VDISCHG_MIN ){
		bat_status.charger_type = DA9052_WALL_CHARGER;
		bat_status.status = DA9052_CHARGING_CC;
	}
	else if ( vbat <= VDISCHG_MIN ){
		pmic_write( chg_device->da9052 , DA9052_BATCHG_REG , DA9052_BATCHG_ICHGBAT , ICHG_BAT_0);
		pmic_write( chg_device->da9052 , DA9052_BATCHG_REG , DA9052_BATCHG_ICHGPRE , ICHG_PRE_60);

		// Will return error and ichgm=0 if no charger present
		da9052_bat_get_chg_current(chg_device->da9052,&ichgm);

		if (ichgm > ICHARGE_PRECHG) {
			bat_status.charger_type = DA9052_WALL_CHARGER;
			bat_status.status = DA9052_PRECHARGING;
		}
		else{
			pmic_write( chg_device->da9052 , DA9052_BATCHG_REG , DA9052_BATCHG_ICHGBAT , ICHG_BAT_0);
			pmic_write( chg_device->da9052 , DA9052_BATCHG_REG , DA9052_BATCHG_ICHGPRE , ICHG_PRE_0);
			pmic_write( chg_device->da9052 , DA9052_BATCHG_REG , DA9052_BATCHG_ICHGBAT , ICHG_BAT_1890);
			bat_status.charger_type = DA9052_WALL_CHARGER;
			bat_status.status = DA9052_NONE;
		}
	}
	DA9052_DEBUG("Status %s\n",get_bat_mode(bat_status.status));
	return;

	if( bat_status_old != bat_status.status )
		power_supply_changed(&chg_device->psy);
	bat_status_old = bat_status.status;

read_err:
	pr_err("da9052: Error on read\n");
return;

}

s32 da9052_get_bat_level(struct da9052_charger_device *chg_device)
{
	u16 vbat = 0;
	u16 ichgm = 0;
	static int bat_capacity_old;

	da9052_charger_status_update(chg_device);

	if( da9052_bat_get_battery_voltage(chg_device->da9052, &vbat) )
		goto read_err;

	// Will return error and ichgm=0 if no charger present
	da9052_bat_get_chg_current(chg_device->da9052,&ichgm);

	switch(bat_status.status){
		case DA9052_DISCHARGING_WITHOUT_CHARGER:
		case DA9052_DISCHARGING_WITH_CHARGER:
			if( vbat > VDISCHARGE_100 )
				bat_status.cal_capacity = 100;
			else if(vbat > VDISCHARGE_95)
				bat_status.cal_capacity = 95;
			else if(vbat > VDISCHARGE_90)
				bat_status.cal_capacity = 90;
			else if(vbat > VDISCHARGE_85)
				bat_status.cal_capacity = 85;
			else if(vbat > VDISCHARGE_80)
				bat_status.cal_capacity = 80;
			else if(vbat > VDISCHARGE_75)
				bat_status.cal_capacity = 75;
			else if(vbat > VDISCHARGE_65)
				bat_status.cal_capacity = 65;
			else if(vbat > VDISCHARGE_55)
				bat_status.cal_capacity = 55;
			else if(vbat > VDISCHARGE_50)
				bat_status.cal_capacity = 50;
			else if(vbat > VDISCHARGE_45)
				bat_status.cal_capacity = 45;
			else if(vbat > VDISCHARGE_40)
				bat_status.cal_capacity = 40;
			else if(vbat > VDISCHARGE_35)
				bat_status.cal_capacity = 35;
			else if(vbat > VDISCHARGE_25)
				bat_status.cal_capacity = 25;
			else if(vbat > VDISCHARGE_20)
				bat_status.cal_capacity = 20;
			else if(vbat > VDISCHARGE_15)
				bat_status.cal_capacity = 15;
			else if(vbat > VDISCHARGE_10)
				bat_status.cal_capacity = 10;
			else if(vbat > VDISCHARGE_5)
				bat_status.cal_capacity = 5;
			else
				bat_status.cal_capacity = 2;
			break;
		case DA9052_CHARGING_CC:
			if(vbat<VCHARGE_5)
				bat_status.cal_capacity = 5;
			else if(vbat<VCHARGE_10)
				bat_status.cal_capacity = 10;
			else if(vbat<VCHARGE_15)
				bat_status.cal_capacity = 15;
			else if(vbat<VCHARGE_20)
				bat_status.cal_capacity = 20;
			else if(vbat<VCHARGE_25)
				bat_status.cal_capacity = 25;
			else if(vbat<VCHARGE_30)
				bat_status.cal_capacity = 30;
			else if(vbat<VCHARGE_35)
				bat_status.cal_capacity = 35;
			else if(vbat<VCHARGE_40)
				bat_status.cal_capacity = 40;
			else if(vbat<VCHARGE_45)
				bat_status.cal_capacity = 45;
			else if(vbat<VCHARGE_50)
				bat_status.cal_capacity = 50;
			else if(vbat<VCHARGE_55)
				bat_status.cal_capacity = 55;
			else if(vbat<VCHARGE_60)
				bat_status.cal_capacity = 60;
			else if(vbat<VCHARGE_65)
				bat_status.cal_capacity = 65;
			else if(vbat<VCHARGE_70)
				bat_status.cal_capacity = 70;
			else if(vbat<VCHARGE_75)
				bat_status.cal_capacity = 75;
			else if(vbat<VCHARGE_80)
				bat_status.cal_capacity = 80;
			else
				bat_status.cal_capacity = 90;
			break;
		case DA9052_CHARGING_CV:
			if( ichgm > ICHARGE_85)
				bat_status.cal_capacity = 85;
			else if( ichgm > ICHARGE_90)
				bat_status.cal_capacity = 90;
			else if( ichgm > ICHARGE_95)
				bat_status.cal_capacity = 95;
			else
				bat_status.cal_capacity = 100;
			break;
		case DA9052_CHARGEEND:
			bat_status.cal_capacity = 100;
			break;
		case DA9052_PRECHARGING:
			bat_status.cal_capacity = 0;
			break;
		default:
			break;
	}

	DA9052_DEBUG("bat_status.cal_capacity %d\n",bat_status.cal_capacity);
	if( bat_capacity_old != bat_status.cal_capacity )
		power_supply_changed(&chg_device->psy);
	bat_capacity_old = bat_status.cal_capacity;
	return 0;

read_err:
	pr_err("da9052: Error on read\n");
	return 0;
}

static s32 monitoring_thread(void *data)
{
	s32 ret = 0;

	struct da9052_charger_device *chg_device =
		(struct da9052_charger_device *)data;

	set_freezable();

	while (monitoring_thread_state == ACTIVE) {

		/* Make this thread friendly to system suspend and resume */
		try_to_freeze();

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(chg_device->monitoring_interval);

		da9052_charger_status_update(chg_device);

		ret = da9052_get_bat_level(chg_device);
		if (!ret) {
			/* BAT Capacity is low then
			update the monitoring status*/
			if (bat_status.cal_capacity <
			chg_device->bat_pdata->bat_capacity_limit_low) {
				monitoring_status.bat_level_status =
					TRUE;
				monitoring_status.bat_level =
					bat_status.cal_capacity;
			} else {
				monitoring_status.bat_level_status = 0;
				monitoring_status.bat_level =
					bat_status.cal_capacity;
			}
		} else
			DA9052_DEBUG("Battery Measurement Fails = %d\n", ret);
	}

	complete_and_exit(&monitoring_thread_notifier, 0);

	return 0;
}

static int da9052_bat_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	s32 ret = 0;

	struct da9052_charger_device *chg_device =
	container_of(psy, struct da9052_charger_device, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bat_status.status == DA9052_CHARGING_CC ||
						bat_status.status == DA9052_CHARGING_CV)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;

		else if (bat_status.status ==
			DA9052_DISCHARGING_WITH_CHARGER)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

		else if (bat_status.status ==
				DA9052_DISCHARGING_WITHOUT_CHARGER)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

		else if (bat_status.status == DA9052_CHARGEEND)
			val->intval = POWER_SUPPLY_STATUS_FULL;

	break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (bat_status.charger_type == DA9052_NOCHARGER)
			val->intval = 0;
		else
			val->intval = 1;
	break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (bat_status.status == DA9052_NONE )
			val->intval = 0;
		else
			val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (bat_status.health != POWER_SUPPLY_HEALTH_OVERHEAT) {
			if (bat_status.cal_capacity <
				chg_device->bat_pdata->bat_capacity_limit_low)
				bat_status.health = POWER_SUPPLY_HEALTH_DEAD;
			else
				bat_status.health = POWER_SUPPLY_HEALTH_GOOD;
		}
		val->intval = bat_status.health;
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = (bat_target_voltage * 1000);
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = (chg_device->bat_pdata->bat_volt_cutoff * 1000);
	break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = (bat_info.bat_voltage * 1000);
	break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = (bat_info.chg_current * 1000);
	break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bat_status.cal_capacity;
	break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (bat_status.cal_capacity <
			chg_device->bat_pdata->bat_capacity_limit_low)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;

		else if (bat_status.cal_capacity <
			chg_device->bat_pdata->bat_capacity_limit_high)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

		else if (bat_status.cal_capacity ==
			chg_device->bat_pdata->bat_capacity_full)
				val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;

		else if (bat_status.cal_capacity >
			chg_device->bat_pdata->bat_capacity_limit_high)
				val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;

		else
			val->intval = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
	break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = BAT_TYPE;
	break;
	default:
		ret = -EINVAL;
	break;
	}
	return 0;
}

static void da9052_battery_setup_psy(struct da9052_charger_device *chg_device)
{
	battery_info.name = DA9052_BAT_DEVICE_NAME;
	battery_info.technology = BAT_TYPE;
	battery_info.voltage_max_design =
				(chg_device->bat_target_voltage*1000);
	battery_info.voltage_min_design =
		(chg_device->bat_pdata->bat_volt_cutoff*1000);
	battery_info.energy_full_design =
		chg_device->bat_pdata->bat_capacity_full;
	battery_info.energy_empty_design =
		chg_device->bat_pdata->bat_capacity_limit_low;
	battery_info.use_for_apm = 1;

	chg_device->psy.name = DA9052_BAT_DEVICE_NAME;
	chg_device->psy.use_for_apm = 1;
	chg_device->psy.type = POWER_SUPPLY_TYPE_BATTERY;
	chg_device->psy.get_property = da9052_bat_get_property;

	chg_device->psy.properties = da902_bat_props;
	chg_device->psy.num_properties = ARRAY_SIZE(da902_bat_props);

};

static void da9052_ccxmx53_init( struct da9052 *da9052 )
{
	// ISET_BUCK to maximum
	pmic_write(da9052,DA9052_CHGBUCK_REG,DA9052_CHGBUCK_ISETBUCK,ISET_BUCK_1800);
	// ISET_USB and ISET_DCIN to maximum
	pmic_write(da9052,DA9052_ISET_REG,DA9052_ISET_ISETDCIN,ISET_DCIN_1800);
	pmic_write(da9052,DA9052_ISET_REG,DA9052_ISET_ISETVBUS,ISET_VBUS_1800);
	// ICHG_BAT to maximum
	pmic_write(da9052,DA9052_BATCHG_REG,DA9052_BATCHG_ICHGBAT,ICHG_BAT_1890);
	// ICHG_PRE to disable
	pmic_write(da9052,DA9052_BATCHG_REG,DA9052_BATCHG_ICHGPRE,ICHG_PRE_0);
	// VCHG_THR to 3.7V
	pmic_write(da9052,DA9052_CHGCONT_REG,DA9052_CHGCONT_VCHTHR,VCH_THR_38);
	// VCHG_BAT to IonLi voltage (4.2V)
	pmic_write(da9052,DA9052_CHGCONT_REG,DA9052_CHGCONT_VCHGBAT,VCHG_BAT_LIION);
	// Charge Time out disabled
	pmic_write(da9052,DA9052_INPUTCONT_REG,DA9053_INPUTCONT_TCTR,TCTR_DIS);
}

static s32 __devinit da9052_bat_probe(struct platform_device *pdev)
{
	struct da9052_bat_platform_data *pdata = pdev->dev.platform_data;
	struct da9052_charger_device *chg_device;
	u8 reg_data;
	int ret;

	chg_device = kzalloc(sizeof(*chg_device), GFP_KERNEL);
	if (!chg_device)
		return -ENOMEM;

	chg_device->da9052 = dev_get_drvdata(pdev->dev.parent);

	chg_device->bat_pdata = pdata;

	platform_set_drvdata(pdev, chg_device);

	chg_device->monitoring_interval =
		msecs_to_jiffies(chg_device->bat_pdata->monitoring_interval);

	if(chg_device->bat_pdata->filter_size)
		filter_size = chg_device->bat_pdata->filter_size;

	ret = da9052_read(chg_device->da9052, DA9052_CHGCONT_REG, &reg_data);
	if (ret)
		goto err_charger_init;
	chg_device->charger_voltage_drop = bat_drop_reg_to_mV(reg_data &&
							DA9052_CHGCONT_TCTR);
	chg_device->bat_target_voltage =
			bat_reg_to_mV(reg_data && DA9052_CHGCONT_VCHGBAT);
	bat_target_voltage = chg_device->bat_target_voltage;

	reg_data = 0;
	ret = da9052_read(chg_device->da9052, DA9052_ICHGEND_REG, &reg_data);
	if (ret)
		goto err_charger_init;

	chg_device->chg_end_current = ichg_reg_to_mA(reg_data);

	bat_status.charger_type = DA9052_NOCHARGER;
	bat_status.status = DA9052_CHARGING_CC;
	bat_status.charging_mode = DA9052_NONE;

	da9052_ccxmx53_init(chg_device->da9052);

	da9052_battery_setup_psy(chg_device);

	ret = power_supply_register(&pdev->dev, &chg_device->psy);
	 if (ret)
		goto err_charger_init;

	monitoring_thread_state = ACTIVE;
	init_completion(&monitoring_thread_notifier);
	monitoring_thread_pid = kernel_thread(monitoring_thread, chg_device,
						CLONE_KERNEL | SIGCHLD);
	if (monitoring_thread_pid > 0) {
		printk(KERN_ERR "Monitoring thread is successfully started,\
		pid = %d\n", monitoring_thread_pid);
	}

	printk(KERN_INFO "Exiting DA9052 battery probe \n");
	return 0;

err_charger_init:
	platform_set_drvdata(pdev, NULL);
	kfree(chg_device);
	return ret;
}
static int __devexit da9052_bat_remove(struct platform_device *dev)
{
	monitoring_thread_state = INACTIVE;
	wait_for_completion(&monitoring_thread_notifier);
	return 0;
}

static struct platform_driver da9052_bat_driver = {
	.probe		= da9052_bat_probe,
	.remove		= __devexit_p(da9052_bat_remove),
	.driver		= {
		.name	= DA9052_BAT_DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static s32 __init da9052_bat_init(void)
{
	printk(banner);
	return platform_driver_register(&da9052_bat_driver);
}
module_init(da9052_bat_init);

static void __exit da9052_bat_exit(void)
{
	printk("DA9052: Unregistering BAT device.\n");
	platform_driver_unregister(&da9052_bat_driver);
}
module_exit(da9052_bat_exit);

MODULE_AUTHOR("Digi International.");
MODULE_DESCRIPTION("DA9052 BAT Device Driver on a CCXMX53 JSK.");
MODULE_LICENSE("GPL");
