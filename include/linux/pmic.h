/*
 * pm.c -- Power Management IC for SoC support.
 *
 * Copyright (C) 2007 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood <lg@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __LINUX_PM_H_
#define __LINUX_PM_H_

#include <linux/list.h>
#include <asm/atomic.h>

/* 
 * pm power loads. i.e. subsystems we supply power to.
 * TODO: Need more subsystem types.
 */
enum pm_load {
	/* system (inc RAM) */
	PM_LOAD_CPU	= (1 << 0),
	PM_LOAD_MEM	= (1 << 1),
	PM_LOAD_SYS_IO	= (1 << 2),
	
	/* nv memory */
	PM_LOAD_NAND	= (1 << 3),
	PM_LOAD_NOR	= (1 << 4),
	PM_LOAD_EEPROM	= (1 << 5),
	
	/* display */
	PM_LOAD_LCD	= (1 << 6),
	PM_LOAD_GPU	= (1 << 7),
	PM_LOAD_TV	= (1 << 8),
	PM_LOAD_BACKLIGHT	= (1 << 9),
	
	/* audio */
	PM_LOAD_AUDIO	= (1 << 10),
	
	/* storage */
	PM_LOAD_DISK	= (1 << 11),
	PM_LOAD_MMC	= (1 << 12),
	PM_LOAD_MSTICK	= (1 << 13),
	PM_LOAD_PCMCIA	= (1 << 14),
	
	/* phone */
	PM_LOAD_SIM	= (1 << 15),
	PM_LOAD_GSM	= (1 << 16),
	PM_LOAD_VIBRATOR	= (1 << 17),
	
	/* communications */
	PM_LOAD_ETH	= (1 << 18),
	PM_LOAD_WIFI	= (1 << 19),
	PM_LOAD_BT	= (1 << 20),
	PM_LOAD_USB	= (1 << 21),
	PM_LOAD_SERIAL	= (1 << 22),
	PM_LOAD_IRDA	= (1 << 23),
	
	/* power */
	PM_LOAD_BATTERY	= (1 << 24),
	
	/* misc */
	PM_LOAD_GPS	= (1 << 25),
	PM_LOAD_CAMERA	= (1 << 26),
	PM_LOAD_LED	= (1 << 27),
	PM_LOAD_TOUCH	= (1 << 28),
};

/*
 * Regulator control types.
 * Regulators are used to supply either a constant or variable mV (or mA)
 */ 
enum pm_control {
	PM_CONTROL_STATIC = 0,	/* on/off */
	PM_CONTROL_DYNAMIC	= 1, /* on/off + voltage/current scale */
	PM_CONTROL_NONE	= 2, /* no software control */
};

/*
 * Regulator boot time status.
 * Set by bootloader, PIC, PMIC or other hardware.
 */
enum pm_boot {
	PM_BOOT_POWER_ON = 0, /* supplies power for boot */
	PM_BOOT_POWER_OFF = 1, /* not required for boot */
}; 

struct pm_regulator;

/*
 * Regulator voltage/current limits
 * TODO: may add table support for log/linear range
 */
struct pm_range {
	int sval; /* static value */ 
	int dmin; /* dynamic min & max */
	int dmax;
};

/*
 * A regulator circuit that supplies power to many subsystems (or loads)
 */
struct pm_circuit {
	char *name;
	int regulator_id;
	enum pm_load load;
	
	struct pm_range mV_range;
	struct pm_range mA_range;
	
	enum pm_boot boot_time;
	
	/* private */
	struct list_head list;
	struct pm_regulator *regulator;
};

/*
 * Voltage regulator operations.
 * enable is madatory, others optional.
 */
struct pm_regulator_ops {
	int (*enable)(struct pm_regulator *, int enable);
	
	/* get/set voltage for regulator mode */
	int (*set_voltage)(struct pm_regulator *, int mV, int Vmode);
	int (*get_voltage)(struct pm_regulator *, int Vmode);
	
	/* get/set regulator mode (e.g. normal, hibernate, etc) */
	int (*get_mode)(struct pm_regulator *);
	int (*set_mode)(struct pm_regulator *, int Vmode);
	
	/* get/set regulator current (set limit) */
	int (*set_current)(struct pm_regulator *, int mA, int Amode);
	int (*get_current)(struct pm_regulator *);
	
	/* regulator ioctl (for vendor specific ops) */
	int (*ioctl)(struct pm_regulator *, int cmd, int arg);
};

/*
 * PMIC voltage regulator.
 * PMIC may have many DCDC or LDO regulators.
 */
struct pm_regulator {
	char *name;
	int id;
	struct pm_regulator_ops *ops;
	struct pm_range mV_range;
	struct pm_range mA_range;
	enum pm_control mV_control;
	enum pm_control mA_control;
	
	int use_count;
	struct list_head list;
	struct pm_circuit *circuit;
	void *private;
};
	

/*
 * PMIC driver API
 */
/* registration */
int pm_register_regulator(struct pm_regulator *reg, void *data);
void pm_unregister_regulator(struct pm_regulator *reg);

/*
 * SoC Machine power API
 */
/* registration of machine circuits */ 
int pm_register_circuit(struct pm_circuit *circuit);
void pm_unregister_circuit(struct pm_circuit *circuit);


/* 
 * General device driver SoC pm API
 * This API is used by device drivers for access to system power. The machines
 * power architecture is transparent to all drivers, i.e. they don't need to
 * know or care about it. 
 */
/* load registration */
int pm_register_load(struct device *dev, enum pm_load load, char *id);
void pm_unregister_load(struct device *dev);

/* power request/release for static voltage loads 
 * (mV set by machine specific code in arch/cpu/platform/machine.c ) */
int pm_request_power(struct device *dev);
int pm_release_power(struct device *dev);

/* power request for variable voltage loads (i.e. backlight, high power LED's) */
int pm_set_voltage(struct device *dev, int mV);
int pm_get_voltage(struct device *dev);
int pm_set_current(struct device *dev, int mA);
int pm_get_current(struct device *dev);

/*
 * General CPUFREQ SoC pm API
 */
/* load registration */
int pm_register_cpu_load(int cpu);
void pm_unregister_cpu_load(int cpu);

/* power request for variable voltage loads */
int pm_request_cpu_voltage(int cpu, int mV);

#endif /*__LINUX_PM_H_*/
