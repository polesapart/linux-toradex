/*
 * arch/arm/mach-mx5/displays/displays.h
 *
 * Copyright (C) 2009-2010 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef __ASM_ARCH_MXC_CCWMX51_DISPLAYS_H__
#define __ASM_ARCH_MXC_CCWMX51_DISPLAYS_H__

extern void gpio_video_active(int vif, u32 pad);
extern void gpio_video_inactive(int vif);

#ifdef CONFIG_CCWMX51_DISP0_RGB888
#define VIDEO_PIX_FMT	IPU_PIX_FMT_RGB24
#else
#define VIDEO_PIX_FMT	IPU_PIX_FMT_RGB666
#endif

#include "hdmi_ad9389.h"
#include "vga.h"
#include "lcd.h"

#endif /* __ASM_ARCH_MXC_CCWMX51_DISPLAYS_H__ */
