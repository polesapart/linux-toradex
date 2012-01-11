/*
  * Copyright 2012 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __DEVICES_CCXMX5X_H_
#define __DEVICES_CCXMX5X_H_

extern void ccxmx5x_set_mod_variant(u8 variant);
extern void ccxmx5x_set_mod_revision(u8 revision);
u8 ccxmx5x_get_mod_revision(void);
extern void ccxmx5x_set_mod_sn(u32 sn);
extern int ccxmx5x_create_sysfs_entries(void);

#if defined(CONFIG_MODULE_CCXMX51)
#define CCXMX5X_SYSFS_FNAME	"ccwmx51"
#include "board-ccwmx51.h"
#elif defined(CONFIG_MODULE_CCXMX53)
#define CCXMX5X_SYSFS_FNAME	"ccwmx53"
#include "board-ccwmx53.h"
#endif

#endif /* __DEVICES_CCXMX5X_H_ */
