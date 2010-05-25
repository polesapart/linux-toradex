/*
 * arch/arm/mach-s3c2443/displays/displays.h
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCWMX51_DISPLAYS_H__
#define __ASM_ARCH_MXC_CCWMX51_DISPLAYS_H__

#if defined(CONFIG_CCWMX51_LQ070Y3DG3B)
#include "LQ070Y3DG3B.h"
#endif
#if defined(CONFIG_CCWMX51_CUSTOM)
#include "CUSTOM.h"
#endif

struct ccwmx51_lcd_pdata lcd_display_list[] = {
#if defined(CONFIG_CCWMX51_LQ070Y3DG3B)
	LQ070Y3DG3B_DISPLAY,
#endif
#if defined(CONFIG_CCWMX51_CUSTOM)
	CUSTOM_DISPLAY,
#endif
};

#endif /* __ASM_ARCH_MXC_CCWMX51_DISPLAYS_H__ */
