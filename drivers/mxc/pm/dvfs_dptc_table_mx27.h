/*
 * Copyright 2004-2006 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file dptc.h
 *
 * @brief i.MX27 dptc table file. 
 * 
 * @ingroup PM
 */
#ifndef __DVFS_DPTC_TABLE_MX27_H__
#define __DVFS_DPTC_TABLE_MX27_H__

/*!
 * Default DPTC table definition
 */
#define NUM_OF_FREQS 1
#define NUM_OF_WP    17

static char *default_table_str = "WORKING POINT 17\n\
\n\
WP 0x1c\n\
WP 0x1b\n\
WP 0x1a\n\
WP 0x19\n\
WP 0x18\n\
WP 0x17\n\
WP 0x16\n\
WP 0x15\n\
WP 0x14\n\
WP 0x13\n\
WP 0x12\n\
WP 0x11\n\
WP 0x10\n\
WP 0xf\n\
WP 0xe\n\
WP 0xd\n\
WP 0xc\n\
\n\
DCVR 0xffc00000 0x9162a838 0xffc00000 0xdcb41c5c \n\
DCVR 0xffc00000 0x8ee2081c 0xffc00000 0xd8f34c2c \n\
DCVR 0xffc00000 0x8c617800 0xffc00000 0xd5728bfc \n\
DCVR 0xffc00000 0x89e0e7d0 0xffc00000 0xd171bbb4 \n\
DCVR 0xffc00000 0x87a057a4 0xffc00000 0xcd70fb6c \n\
DCVR 0xffc00000 0x841fb78c 0xffc00000 0xc8300b48 \n\
DCVR 0xffc00000 0x80df2774 0xffc00000 0xc2ef1b24 \n\
DCVR 0xffc00000 0x7ede174c 0xffc00000 0xbfed9ae8 \n\
DCVR 0xffc00000 0x7cdd0728 0xffc00000 0xbcec2ab0 \n\
DCVR 0xffc00000 0x79dc9700 0xffc00000 0xb82b3a74 \n\
DCVR 0xffc00000 0x771c26d8 0xffc00000 0xb3aa4a38 \n\
DCVR 0xffc00000 0x739b76b0 0xffc00000 0xade949f8 \n\
DCVR 0xffc00000 0x701ad688 0xffc00000 0xa86849b8 \n\
DCVR 0xffc00000 0x6d9a365c 0xffc00000 0xa4673970 \n\
DCVR 0xffc00000 0x6b19a62c 0xffc00000 0xa0663928 \n\
DCVR 0xffc00000 0x67d8c600 0xffc00000 0x9b24f8e8 \n\
DCVR 0xffc00000 0x6497f5d8 0xffc00000 0x9623c8a8 \n\
";

#endif
