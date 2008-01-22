/*
 * pmic.c -- Power Management IC for SoC support.
 *
 * Copyright (C) 2007 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood <lg@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pmic.h>
#include <linux/string.h>

static DEFINE_MUTEX(list_mutex);
static DEFINE_MUTEX(regulator_mutex);
static LIST_HEAD(regulator_list);
static LIST_HEAD(circuit_list);
static LIST_HEAD(device_list);
static LIST_HEAD(cpu_list);

struct _pm_device {
	struct device *dev;
	enum pm_load load;
	char *name;
	struct list_head list;
};

struct _pm_cpu {
	int cpu;
	struct list_head list;
}; 

struct _pm_load_str {
	enum pm_load load;
	const char* str;
};

static const struct _pm_load_str sysfs_loads[] = {
	{PM_LOAD_CPU, "cpu"},
	{PM_LOAD_MEM, "memory"},
	{PM_LOAD_SYS_IO, "io"},
	{PM_LOAD_NAND, "nand"},
	{PM_LOAD_NOR, "nor"},
	{PM_LOAD_EEPROM, "eeprom"},
	{PM_LOAD_LCD, "lcd"},
	{PM_LOAD_GPU, "gpu"},
	{PM_LOAD_TV, "tv"},
	{PM_LOAD_BACKLIGHT, "backlight"},
	{PM_LOAD_AUDIO, "audio"},
	{PM_LOAD_DISK, "disk"},
	{PM_LOAD_MMC, "mmc"},
	{PM_LOAD_MSTICK, "memory_stick"},
	{PM_LOAD_PCMCIA, "pcmcia"},
	{PM_LOAD_SIM, "sim"},
	{PM_LOAD_GSM, "gsm"},
	{PM_LOAD_VIBRATOR, "vibrator"},
	{PM_LOAD_ETH, "ethernet"},
	{PM_LOAD_WIFI, "wifi"},
	{PM_LOAD_BT, "bluetooth"},
	{PM_LOAD_USB, "usb"},
	{PM_LOAD_SERIAL, "serial"},
	{PM_LOAD_IRDA, "irda"},
	{PM_LOAD_BATTERY, "battery"},
	{PM_LOAD_GPS, "gps"},
	{PM_LOAD_CAMERA, "camera"},
	{PM_LOAD_LED, "led"},
	{PM_LOAD_TOUCH, "touchscreen"},
};

/* is the voltage valid for regulator & circuit */
static inline int mV_valid(struct pm_regulator *r, int mV)
{
	struct pm_circuit *c = r->circuit;
	
	if (mV < c->mV_range.dmin || mV > c->mV_range.dmax)
		return 0;
	if (mV < r->mV_range.dmin || mV > r->mV_range.dmax)
		return 0;
	return 1;
}

/* is the current limit valid for regulator & circuit */
static inline int mA_valid(struct pm_regulator *r, int mA)
{
	struct pm_circuit *c = r->circuit;
	
	if (mA < c->mA_range.dmin || mA > c->mA_range.dmax)
		return 0;
	if (mA < r->mA_range.dmin || mA > r->mA_range.dmax)
		return 0;
	return 1;
}

/* get a pm device from device */
static struct _pm_device *get_pmd(struct device *dev)
{
	struct _pm_device *pmd;
	
	mutex_lock(&list_mutex);
	list_for_each_entry(pmd, &device_list, list) {
		if (dev == pmd->dev) {
			mutex_unlock(&list_mutex);
			return pmd;
		}
	}
	mutex_unlock(&list_mutex);
	return NULL;
}

/* find the circuit that this regulator supplies */
static struct pm_circuit *find_regulator_circuit(struct pm_regulator *r)
{
	struct pm_circuit *c;
	
	list_for_each_entry(c, &circuit_list, list) {
		if (c->regulator_id == r->id)
			return c;
	}
	return NULL;
}

static struct pm_circuit *get_cpu_circuit(int cpu)
{
	struct _pm_cpu *pmc;
	struct pm_circuit *c;
	
	/* is cpu valid ? */
	list_for_each_entry(pmc, &cpu_list, list) {
		if (cpu == pmc->cpu) {
			list_for_each_entry(c, &circuit_list, list) {
				if (c->load & PM_LOAD_CPU)
					return c;
			}
		}
	}
	return NULL;
}

static inline int regulator_enable(struct pm_regulator *r, int enable)
{
	if (!r->ops->enable)
		return -EINVAL;
	return r->ops->enable(r, enable);
}

static inline int regulator_voltage(struct pm_regulator *r, int mV, int Vid)
{
	if (!mV_valid(r, mV))
		return -EINVAL;
	if (!r->ops->set_voltage)
		return -EINVAL;
	return r->ops->set_voltage(r, mV, Vid);
}

static inline int regulator_current(struct pm_regulator *r, int mA, int Aid)
{
	if (!mA_valid(r, mA))
		return -EINVAL;
	if (!r->ops->set_current)
		return -EINVAL;
	return r->ops->set_current(r, mA, Aid);
}


static int request_power(struct _pm_device *pmd)
{
	struct pm_regulator *r;
	int ret = 0;
	
	list_for_each_entry(r, &regulator_list, list) {
		if (r->circuit->load & pmd->load) {
			mutex_lock(&regulator_mutex);
			if (r->use_count == 0) 
				ret = regulator_enable(r, 1);
			if (ret < 0) {
				mutex_unlock(&regulator_mutex);
				break;
			} else
				r->use_count++;
			mutex_unlock(&regulator_mutex);
		}
	}
	return ret;
}

static int release_power(struct _pm_device *pmd)
{
	struct pm_regulator *r;
	int ret = 0;
	
	list_for_each_entry(r, &regulator_list, list) {
		if (r->circuit->load & pmd->load) {
			mutex_lock(&regulator_mutex);
			r->use_count--;
			if (r->use_count == 0) 
				ret = regulator_enable(r, 0);
			if (ret < 0)
				printk("%s : can't disable regulator %s\n",
					__func__, r->name);
			mutex_unlock(&regulator_mutex);
		}
	}
	return ret;
}

static int request_voltage(struct _pm_device *pmd, int mV)
{
	struct pm_regulator *r;
	int ret = 0;
	
	list_for_each_entry(r, &regulator_list, list) {
		if (r->circuit->load & pmd->load) {
			mutex_lock(&regulator_mutex);
			ret = regulator_voltage(r, mV, 0);
			mutex_unlock(&regulator_mutex);
			if (ret < 0)
				printk(KERN_ERR "%s : failed to set regulator"
				 	"%d voltage to %d mV\n", __func__,
				 	r->id, mV);
		}
	}
	return ret;
}

static int request_current(struct _pm_device *pmd, int mA)
{
	struct pm_regulator *r;
	int ret = 0;
	
	list_for_each_entry(r, &regulator_list, list) {
		if (r->circuit->load & pmd->load) {
			mutex_lock(&regulator_mutex);
			ret = regulator_current(r, mA, 0);
			mutex_unlock(&regulator_mutex);
			if (ret < 0)
				printk(KERN_ERR "%s : failed to set regulator"
				 	"%d current limit to %d mA\n",
				 	 __func__, r->id, mA);
		}
	}
	return ret;
}

int pm_regulator_set_voltage(int id, int mV, int Vid)
{
	struct pm_regulator *r;
	int ret = -EINVAL;
	
	mutex_lock(&list_mutex);
	list_for_each_entry(r, &regulator_list, list) {
		if (id == r->id) {
			mutex_lock(&regulator_mutex);
			ret = regulator_voltage(r, mV, Vid);
			mutex_unlock(&regulator_mutex);
			goto out;;
		}
	}
out:	
	mutex_unlock(&list_mutex);
	return ret;	
}
EXPORT_SYMBOL_GPL(pm_regulator_set_voltage);

/*
 * PMIC driver API
 */
int pm_register_regulator(struct pm_regulator *reg, void *data)
{
	struct pm_circuit *c;
	int ret = 0;

	/* some ops are mandatory */
	if (reg->ops == NULL)
		return -EINVAL;
	
	mutex_lock(&list_mutex);

	/* find the circuit we supply power to */
	c = find_regulator_circuit(reg);
	if (c != NULL) {
		reg->circuit = c;
		reg->private = data;
		c->regulator = reg;
			
		/* are we already in use */
		if (c->boot_time == PM_BOOT_POWER_ON)
			reg->use_count = 1;
			
		printk(KERN_INFO "%s: added %s \n", __func__, reg->name);
	} else
		/* no circuit */
		printk(KERN_INFO "%s: no circuit for %s\n", __func__, 
			reg->name);
	
	/* add circuit to list */		
	INIT_LIST_HEAD(&reg->list);
	list_add(&reg->list, &regulator_list);
	mutex_unlock(&list_mutex);
	return ret;	
}
EXPORT_SYMBOL_GPL(pm_register_regulator);

void pm_unregister_regulator(struct pm_regulator *reg)
{
	mutex_lock(&list_mutex);
	list_del(&reg->list);
	mutex_unlock(&list_mutex);
}
EXPORT_SYMBOL_GPL(pm_unregister_regulator);

/*
 * SoC Machine power API
 */
int pm_register_circuit(struct pm_circuit *circuit)
{
	if (circuit->load == 0)
		return -EINVAL;
		
	mutex_lock(&list_mutex);
	INIT_LIST_HEAD(&circuit->list);
	list_add(&circuit->list, &circuit_list);
	mutex_unlock(&list_mutex);
	return 0;	
}
EXPORT_SYMBOL_GPL(pm_register_circuit);

void pm_unregister_circuit(struct pm_circuit *circuit)
{
	mutex_lock(&list_mutex);
	list_del(&circuit->list);
	mutex_unlock(&list_mutex);	
}
EXPORT_SYMBOL_GPL(pm_unregister_circuit);

/* 
 * General device driver SoC pm API
 */
/* load registration */
int pm_register_load(struct device *dev, enum pm_load load, char *id)
{
	struct _pm_device *pmd;

	pmd = kzalloc(sizeof(struct _pm_device), GFP_KERNEL);
	if (pmd == NULL)
		return -ENOMEM;
		
	pmd->name = kstrdup(id, GFP_KERNEL);
	if (pmd->name == NULL) {
		kfree(pmd);
		return -ENOMEM;
	}
	
	pmd->load = load;
	INIT_LIST_HEAD(&pmd->list);
	mutex_lock(&list_mutex);
	list_add(&pmd->list, &device_list);
	mutex_unlock(&list_mutex);
	return 0;	
}
EXPORT_SYMBOL_GPL(pm_register_load);

void pm_unregister_load(struct device *dev)
{
	struct _pm_device *pmd;

	pmd = get_pmd(dev);
	if (pmd) {
		mutex_lock(&list_mutex);
		list_del(&pmd->list);
		kfree(pmd->name);
		kfree(pmd);
		mutex_unlock(&list_mutex);
	}	
}
EXPORT_SYMBOL_GPL(pm_unregister_load);


/* power request/release for fixed voltage loads 
 * (mV set by machine specific code in arch/cpu/platform/machine.c ) */
int pm_request_power(struct device *dev)
{
	struct _pm_device *pmd;

	pmd = get_pmd(dev);
	if (pmd == NULL) {
		printk(KERN_ERR "%s : device not registered\n", __func__);
		return -EINVAL;
	} else
		return request_power(pmd);
}
EXPORT_SYMBOL_GPL(pm_request_power);

int pm_release_power(struct device *dev)
{
	struct _pm_device *pmd;

	pmd = get_pmd(dev);
	if (pmd == NULL) {
		printk(KERN_ERR "%s : device not registered\n", __func__);
		return -EINVAL;
	} else
		return release_power(pmd);
}
EXPORT_SYMBOL_GPL(pm_release_power);

/* power request for variable voltage loads */
int pm_request_voltage(struct device *dev, int mV)
{
	struct _pm_device *pmd;

	pmd = get_pmd(dev);
	if (pmd == NULL) {
		printk(KERN_ERR "%s : device not registered\n", __func__);
		return -EINVAL;
	} else
		return request_voltage(pmd, mV);
}
EXPORT_SYMBOL_GPL(pm_request_voltage);

/* power request for variable voltage loads */
int pm_request_current(struct device *dev, int mA)
{
	struct _pm_device *pmd;

	pmd = get_pmd(dev);
	if (pmd == NULL) {
		printk(KERN_ERR "%s : device not registered\n", __func__);
		return -EINVAL;
	} else
		return request_current(pmd, mA);
}
EXPORT_SYMBOL_GPL(pm_request_current);

/*
 * CPUFREQ Client
 * SoC PMIC registration and voltage scaling for CPUFREQ based drivers.
 */ 
int pm_register_cpu_load(int cpu)
{
	struct _pm_cpu *pmc;

	pmc = kzalloc(sizeof(struct _pm_cpu), GFP_KERNEL);
	if (pmc == NULL)
		return -ENOMEM;
	
	INIT_LIST_HEAD(&pmc->list);
	mutex_lock(&list_mutex);
	list_add(&pmc->list, &cpu_list);
	mutex_unlock(&list_mutex);
	return 0;	
}
EXPORT_SYMBOL_GPL(pm_register_cpu_load);

void pm_unregister_cpu_load(int cpu)
{
	struct _pm_cpu *pmc;

	mutex_lock(&list_mutex);
	list_for_each_entry(pmc, &cpu_list, list) {
		if (cpu == pmc->cpu) {
			kfree(pmc);
			goto out;
		}
	}
	
out:
	mutex_unlock(&list_mutex);	
}
EXPORT_SYMBOL_GPL(pm_unregister_cpu_load);

int pm_request_cpu_voltage(int cpu, int mV)
{
	struct pm_circuit *c;
	int ret = 0;
	
	mutex_lock(&list_mutex);
	
	c = get_cpu_circuit(cpu);
	if (c == NULL) {
		ret = -EINVAL;
		goto out;
	}
		
	if (c->regulator) {
		struct pm_regulator *r = c->regulator;
		
		if (!mV_valid(r, mV)) {
			ret = -EINVAL;
			goto out;
		}
	
		if (r->ops->set_voltage) {
			ret = r->ops->set_voltage(r, mV, 0);
			goto out;
		}
	}
		 
out:
	mutex_unlock(&list_mutex);
	return ret;	
}
EXPORT_SYMBOL_GPL(pm_request_cpu_voltage);
