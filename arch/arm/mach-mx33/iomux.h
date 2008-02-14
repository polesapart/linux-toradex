/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MACH_MX33_IOMUX_H__
#define __MACH_MX33_IOMUX_H__

#include <linux/types.h>

/*!
 * various IOMUX output functions
 */
typedef enum iomux_output_config {
	OUTPUTCONFIG_ALT0 = 0,	/*!< used as alternate function 0 */
	OUTPUTCONFIG_ALT1,	/*!< used as alternate function 1 */
	OUTPUTCONFIG_ALT2,	/*!< used as GPIO : same as ALT2  */
	OUTPUTCONFIG_ALT3,	/*!< used as alternate function 3 */
	OUTPUTCONFIG_ALT4,	/*!< used as alternate function 4 */
	OUTPUTCONFIG_ALT5,	/*!< used as alternate function 5 */
	OUTPUTCONFIG_ALT6,	/*!< used as alternate function 6 */
	OUTPUTCONFIG_ALT7,	/*!< used as alternate function 2 */
	OUTPUTCONFIG_FUNC = 0x10,	/*!< used as function */
} iomux_pin_ocfg_t;

/*!
 * various IOMUX input functions
 */
typedef enum iomux_input_config {
	INPUTCONFIG_ALT0 = 0,	/*!< used as alternate function 0 */
	INPUTCONFIG_ALT1,	/*!< used as alternate function 1 */
	INPUTCONFIG_ALT2,	/*!< used as GPIO : same as ALT2  */
	INPUTCONFIG_ALT3,	/*!< used as alternate function 3 */
	INPUTCONFIG_ALT4,	/*!< used as alternate function 4 */
	INPUTCONFIG_ALT5,	/*!< used as alternate function 5 */
	INPUTCONFIG_ALT6,	/*!< used as alternate function 6 */
	INPUTCONFIG_ALT7,	/*!< used as alternate function 2 */
	INPUTCONFIG_FUNC = 0x10,	/*!< used as function */
} iomux_pin_icfg_t;

/*!
 * various IOMUX pad functions
 */
typedef enum iomux_pad_config {
	PAD_CTL_CMOS_INPUT = 0x0 << 9,
	PAD_CTL_DDR_INPUT = 0x1 << 9,
	PAD_CTL_HYS_NONE = 0x0 << 8,
	PAD_CTL_HYS_ENABLE = 0x1 << 8,
	PAD_CTL_PKE_NONE = 0x0 << 7,
	PAD_CTL_PKE_ENABLE = 0x1 << 7,
	PAD_CTL_PUE_KEEPER = 0x0 << 6,
	PAD_CTL_PUE_PUD = 0x1 << 6,
	PAD_CTL_100K_PD = 0x0 << 4,
	PAD_CTL_100K_PU = 0x1 << 4,
	PAD_CTL_47K_PU = 0x2 << 4,
	PAD_CTL_22K_PU = 0x3 << 4,
	PAD_CTL_ODE_CMOS = 0x0 << 3,
	PAD_CTL_ODE_OpenDrain = 0x1 << 3,
	PAD_CTL_DRV_LOW = 0x0 << 1,
	PAD_CTL_DRV_MEDIUM = 0x1 << 1,
	PAD_CTL_DRV_HIGH = 0x2 << 1,
	PAD_CTL_DRV_MAX = 0x3 << 1,
	PAD_CTL_SRE_SLOW = 0x0 << 0,
	PAD_CTL_SRE_FAST = 0x1 << 0
} iomux_pad_config_t;

/*!
 * various IOMUX pad functions for pin group
 */
#define PAD_CTL_GRP_BASE	0x05F8
#define PAD_CTL_GRP_INDEX(x) (PAD_CTL_GRP_BASE+((x)<<2))

typedef enum iomux_pad_group {
	PAD_CTL_GRP_H323 = 0,
	PAD_CTL_GRP_H26,
	PAD_CTL_GRP_H8,
	PAD_CTL_GRP_DS1,
	PAD_CTL_GRP_H27,
	PAD_CTL_GRP_H243,
	PAD_CTL_GRP_H10,
	PAD_CTL_GRP_H9,
	PAD_CTL_GRP_DS272,
	PAD_CTL_GRP_H11,
	PAD_CTL_GRP_H12,
	PAD_CTL_GRP_H31,
	PAD_CTL_GRP_H13,
	PAD_CTL_GRP_H32,
	PAD_CTL_GRP_SD2_PU,
	PAD_CTL_GRP_DS275,
	PAD_CTL_GRP_DS68,
	PAD_CTL_GRP_H33,
	PAD_CTL_GRP_DS69,
	PAD_CTL_GRP_H34,
	PAD_CTL_GRP_H16,
	PAD_CTL_GRP_H17,
	PAD_CTL_GRP_H18,
	PAD_CTL_GRP_DS55,
	PAD_CTL_GRP_H1,
	PAD_CTL_GRP_H20,
	PAD_CTL_GRP_H19,
	PAD_CTL_GRP_H2,
	PAD_CTL_GRP_DS310,
	PAD_CTL_GRP_DS299,
	PAD_CTL_GRP_H290,
	PAD_CTL_GRP_H21,
	PAD_CTL_GRP_H3,
	PAD_CTL_GRP_SD1_PU,
	PAD_CTL_GRP_H22,
	PAD_CTL_GRP_H4,
	PAD_CTL_GRP_H319,
	PAD_CTL_GRP_H5,
	PAD_CTL_GRP_SD3_PU,
	PAD_CTL_GRP_H24,
	PAD_CTL_GRP_H239,
	PAD_CTL_GRP_H6,
	PAD_CTL_GRP_IMDQS,
	PAD_CTL_GRP_H294,
	PAD_CTL_GRP_H25,
	PAD_CTL_GRP_H7,
	PAD_CTL_GRP_IM1,
	PAD_CTL_GRP_MAX
} iomux_pad_grp_t;

/*!
 * various IOMUX the index of pad's select input register
 */
#define IOMUXC_SELECT_INPUT(x) (0x06B4+((x)<<2))

typedef enum iomux_signal_name {
	AUDMUX_P7_INPUT_DB_AMX = 0,
	AUDMUX_P7_INPUT_TXCLK_AMX,
	AUDMUX_P7_INPUT_TXFS_AMX,
	CCM_IPP_DI_CLK,
	CCM_PLL1_BYPASS_CLK,
	CCM_PLL2_BYPASS_CLK,
	CCM_PLL3_BYPASS_CLK,
	CSPI1_IPP_CSPI_CLK_IN,
	CSPI1_IPP_IND_MISO,
	CSPI1_IPP_IND_MOSI,
	CSPI1_IPP_IND_SS_B_0,
	CSPI1_IPP_IND_SS_B_1,
	CSPI2_IPP_CSPI_CLK_IN,
	CSPI2_IPP_IND_DATAREADY_B,
	CSPI2_IPP_IND_MISO,
	CSPI2_IPP_IND_MOSI,
	CSPI2_IPP_IND_SS_B_0,
	CSPI2_IPP_IND_SS_B_2,
	CSPI2_IPP_IND_SS_B_3,
	CSPI3_IPP_CSPI_CLK_IN,
	CSPI3_IPP_IND_MISO,
	CSPI3_IPP_IND_MOSI,
	CSPI3_IPP_IND_SS_B_3,
	EMI_IPP_IND_RDY_INT,
	FEC_FEC_COL,
	FEC_FEC_CRS,
	FEC_FEC_MDI,
	FEC_FEC_RDATA_0,
	FEC_FEC_RDATA_1,
	FEC_FEC_RDATA_2,
	FEC_FEC_RDATA_3,
	FEC_FEC_RX_CLK,
	FEC_FEC_RX_DV,
	FEC_FEC_RX_ER,
	FEC_FEC_TX_CLK,
	HSC_IPP_IND_SENS1_DATA_EN,
	HSC_IPP_IND_SENS2_DATA_EN,
	HSC_PAR_SISG_TRIG,
	I2C3_IPP_SCL_IN,
	I2C3_IPP_SDA_IN,
	IPU_IPP_DI_1_EXT_CLK,
	IPU_IPP_DI_1_IND_DISPB_DATA_16,
	IPU_IPP_DI_1_IND_DISPB_DATA_17,
	IPU_IPP_DI_1_IND_DISPB_SD_D,
	PATA_IPP_IND_ATA_DATA_0,
	PATA_IPP_IND_ATA_DATA_10,
	PATA_IPP_IND_ATA_DATA_11,
	PATA_IPP_IND_ATA_DATA_12,
	PATA_IPP_IND_ATA_DATA_13,
	PATA_IPP_IND_ATA_DATA_14,
	PATA_IPP_IND_ATA_DATA_15,
	PATA_IPP_IND_ATA_DATA_1,
	PATA_IPP_IND_ATA_DATA_2,
	PATA_IPP_IND_ATA_DATA_3,
	PATA_IPP_IND_ATA_DATA_4,
	PATA_IPP_IND_ATA_DATA_5,
	PATA_IPP_IND_ATA_DATA_6,
	PATA_IPP_IND_ATA_DATA_7,
	PATA_IPP_IND_ATA_DATA_8,
	PATA_IPP_IND_ATA_DATA_9,
	PATA_IPP_IND_ATA_DMARQ,
	PATA_IPP_IND_ATA_INTRQ,
	PATA_IPP_IND_ATA_IORDY,
	UART1_IPP_UART_DCD_DTE_I_B,
	UART1_IPP_UART_DSR_DTE_I_B,
	UART1_IPP_UART_DTR_DCE_I_B,
	UART1_IPP_UART_RI_DTE_I_B,
	UART1_IPP_UART_RTS_B,
	UART1_IPP_UART_RXD_MUX,
	UART2_IPP_UART_RTS_B,
	UART2_IPP_UART_RXD_MUX,
	UART3_IPP_UART_RTS_B,
	UART3_IPP_UART_RXD_MUX,
	UART4_IPP_UART_RTS_B,
	UART4_IPP_UART_RXD_MUX,
	UART5_IPP_UART_RTS_B,
	UART5_IPP_UART_RXD_MUX,
	IOMUX_SIGNAL_MAX
} iomux_signal_name_t;

/*!
 * various IOMUX general purpose functions
 * TODO:// Still be not define in zappa
 */
typedef enum iomux_gp_func {
	MUX_PGP_FIRI = 0x1 << 0,
	MUX_DDR_MODE = 0x1 << 1,
	MUX_PGP_CSPI_BB = 0x1 << 2,
	MUX_PGP_ATA_1 = 0x1 << 3,
	MUX_PGP_ATA_2 = 0x1 << 4,
	MUX_PGP_ATA_3 = 0x1 << 5,
	MUX_PGP_ATA_4 = 0x1 << 6,
	MUX_PGP_ATA_5 = 0x1 << 7,
	MUX_PGP_ATA_6 = 0x1 << 8,
	MUX_PGP_ATA_7 = 0x1 << 9,
	MUX_PGP_ATA_8 = 0x1 << 10,
	MUX_PGP_UH2 = 0x1 << 11,
	MUX_SDCTL_CSD0_SEL = 0x1 << 12,
	MUX_SDCTL_CSD1_SEL = 0x1 << 13,
	MUX_CSPI1_UART3 = 0x1 << 14,
	MUX_EXTDMAREQ2_MBX_SEL = 0x1 << 15,
	MUX_TAMPER_DETECT_EN = 0x1 << 16,
	MUX_PGP_USB_4WIRE = 0x1 << 17,
	MUX_PGB_USB_COMMON = 0x1 << 18,
	MUX_SDHC_MEMSTICK1 = 0x1 << 19,
	MUX_SDHC_MEMSTICK2 = 0x1 << 20,
	MUX_PGP_SPLL_BYP = 0x1 << 21,
	MUX_PGP_UPLL_BYP = 0x1 << 22,
	MUX_PGP_MSHC1_CLK_SEL = 0x1 << 23,
	MUX_PGP_MSHC2_CLK_SEL = 0x1 << 24,
	MUX_CSPI3_UART5_SEL = 0x1 << 25,
	MUX_PGP_ATA_9 = 0x1 << 26,
	MUX_PGP_USB_SUSPEND = 0x1 << 27,
	MUX_PGP_USB_OTG_LOOPBACK = 0x1 << 28,
	MUX_PGP_USB_HS1_LOOPBACK = 0x1 << 29,
	MUX_PGP_USB_HS2_LOOPBACK = 0x1 << 30,
	MUX_CLKO_DDR_MODE = 0x1 << 31,
} iomux_gp_func_t;

/*!
 * This function is used to configure a pin through the IOMUX module.
 *
 * @param  pin		a pin number as defined in \b #iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 * @return 		0 if successful; Non-zero otherwise
 */
int iomux_config_mux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		     iomux_pin_icfg_t in);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pins
 * @param  config       ORed value of elements defined in \b #iomux_pad_config_t
 */
void iomux_config_pad(iomux_pin_name_t pin, __u32 config);

/*!
 * This function select a input for a IOMUX signal.
 *
 * @param  signal       a signal number as defined in \b #iomux_signal_name_t
 * @param  config       value of input configuration
 */
void iomux_select_input(iomux_signal_name_t signal, __u32 config);

/*!
 * This function configures the pad value for a IOMUX pin group.
 *
 * @param  grp          a pin number as defined in \b #iomux_pad_grp_t
 * @param  config       ORed value of elements defined in \b #iomux_pad_config_t
 */
void iomux_config_pad_grp(iomux_pad_grp_t grp, __u32 config);

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void iomux_config_gpr(iomux_gp_func_t gp, bool en);

/*!
 * Request ownership for an IO pin. This function has to be the first one
 * being called before that pin is used. The caller has to check the
 * return value to make sure it returns 0.
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 *
 * @return		0 if successful; Non-zero otherwise
 */
int mxc_request_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		      iomux_pin_icfg_t in);

/*!
 * Release ownership for an IO pin
 *
 * @param  pin		a name defined by \b iomux_pin_name_t
 * @param  out		an output function as defined in \b #iomux_pin_ocfg_t
 * @param  in		an input function as defined in \b #iomux_pin_icfg_t
 */
void mxc_free_iomux(iomux_pin_name_t pin, iomux_pin_ocfg_t out,
		    iomux_pin_icfg_t in);

/*!
 * This function enables/disables the general purpose function for a particular
 * signal.
 *
 * @param  gp   one signal as defined in \b #iomux_gp_func_t
 * @param  en   \b #true to enable; \b #false to disable
 */
void mxc_iomux_set_gpr(iomux_gp_func_t gp, bool en);

/*!
 * This function configures the pad value for a IOMUX pin.
 *
 * @param  pin          a pin number as defined in \b #iomux_pin_name_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad(iomux_pin_name_t pin, u32 config);

/*!
 * This function configures the pad value for a IOMUX pin group.
 *
 * @param  grp         a pin number as defined in \b #iomux_pad_grp_t
 * @param  config       the ORed value of elements defined in \b #iomux_pad_config_t
 */
void mxc_iomux_set_pad_grp(iomux_pad_grp_t grp, u32 config);

#endif
