/*
 * This file contains the code which performs automatic recalibration of the
 * Airoha transceiver.
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef airohaCalibration_h
#define airohaCalibration_h

#include "linux/types.h"

#define WCD_MAGIC		        "WCALDATA"
#define WCD_MAX_CAL_POINTS	    (8)
#define WCD_CHANNELS_BG		    (14)
#define WCD_CHANNELS_A		    (35)
#define WCD_B_CURVE_INDEX       (0)
#define WCD_G_CURVE_INDEX       (1)


/*! \brief The wireless calibration information
 */
typedef struct nv_wcd_header {
	char	magic_string[8];	/* WCALDATA */
	char	ver_major;		/* Major version in ascii */
	char	ver_minor;		/* Minor version in ascii */
	uint16_t hw_platform;		/* Hardware Platform used for calibration */
	uint8_t	numcalpoints;		/* Number of points per curve */
	uint8_t	padding[107];		/* Reserved for future use */
	uint32_t wcd_len;		/* Total length of the data section */
	uint32_t wcd_crc;		/* Data section crc32 */
} nv_wcd_header_t;


typedef struct wcd_point {
	int16_t	 out_power;		/* Output Power */
	uint16_t adc_val;		/* Measured ADC val */
	uint8_t  power_index;		/* Airoha Power Index */
	uint8_t  reserved[3];		/* For future use */
} wcd_point_t;

typedef struct wcd_curve {
	uint8_t  max_power_index;	/* Airoha Max Power Index */
	uint8_t  reserved[3];		/* Resered for future use */
	wcd_point_t points[WCD_MAX_CAL_POINTS];	/* Calibration curve points */
} wcd_curve_t;

typedef struct wcd_data {
	nv_wcd_header_t  header;
	wcd_curve_t cal_curves_bg[WCD_CHANNELS_BG][2];
    wcd_curve_t cal_curves_a[WCD_CHANNELS_A];
} wcd_data_t;

void digiWifiInitCalibration(struct piper_priv *digi);


#endif
