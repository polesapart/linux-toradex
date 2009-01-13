/*
 * arch/arm/mach-ns9xxx/include/mach/display/fb.h
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#if defined(CONFIG_NS9XXX_FB_VGA)
#include "VGA.h"
#elif defined(CONFIG_NS9XXX_FB_LQ057Q3DC12I)
#include "LQ057Q3DC12I.h"
#elif defined(CONFIG_NS9XXX_FB_LQ064V3DG01)
#include "LQ064V3DG01.h"
#elif defined(CONFIG_NS9XXX_FB_CUSTOM)
#include "CUSTOM.h"
#else
#define NS9XXX_NODISPLAY
#endif
