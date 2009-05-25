#ifndef __NET_PIPER_PLATFORM_H__
#define __NET_PIPER_PLATFORM_H__

#define WCD_MAGIC		"WCALDATA"
#define WCD_MAX_CAL_POINTS	(8)
#define WCD_CHANNELS_BG		(14)
#define WCD_CHANNELS_A		(35)
#define WCD_B_CURVE_INDEX       (0)
#define WCD_G_CURVE_INDEX       (1)

#define PIPER_DRIVER_NAME	"piper"


typedef struct nv_wcd_header {
	char magic_string[8];	/* WCALDATA */
	char ver_major;		/* Major version in ascii */
	char ver_minor;		/* Minor version in ascii */
	u16 hw_platform;	/* Hardware Platform used for calibration */
	u8 numcalpoints;	/* Number of points per curve */
	u8 padding[107];	/* Reserved for future use */
	u32 wcd_len;		/* Total length of the data section */
	u32 wcd_crc;		/* Data section crc32 */
} nv_wcd_header_t;

typedef struct wcd_point {
	s16 out_power;		/* Output Power */
	u16 adc_val;		/* Measured ADC val */
	u8 power_index;		/* Airoha Power Index */
	u8 reserved[3];		/* For future use */
} wcd_point_t;

typedef struct wcd_curve {
	u8 max_power_index;	/* Airoha Max Power Index */
	u8 reserved[3];		/* Resered for future use */
	/* Calibration curve points */
	wcd_point_t points[WCD_MAX_CAL_POINTS];
} wcd_curve_t;

typedef struct wcd_data {
	nv_wcd_header_t header;
	wcd_curve_t cal_curves_bg[WCD_CHANNELS_BG][2];
	wcd_curve_t cal_curves_a[WCD_CHANNELS_A];
} wcd_data_t;


struct piper_pdata {
	u8 macaddr[6];
	int rst_gpio;
	int irq_gpio;
	wcd_data_t wcd;
};

#endif /* __NET_PIPER_PLATFORM_H__ */
