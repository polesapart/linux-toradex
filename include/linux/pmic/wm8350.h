/*
 * wm8350.h  --  Power Managment Driver for Wolfson WM8350 PMIC
 *
 * Copyright 2007 Wolfson Microelectronics PLC
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    8th Feb 2007   Initial version.
 *
 */

#ifndef __LINUX_PMIC_WM8350_H
#define __LINUX_PMIC_WM8350_H

#include <linux/device.h>
#include <linux/power_supply.h>
#include <linux/leds.h>
#include <linux/backlight.h>
#include <sound/soc.h>

/*
 * Register values.
 */
#define WM8350_RESET_ID                         0x00
#define WM8350_ID                               0x01
#define WM8350_SYSTEM_CONTROL_1                 0x03
#define WM8350_SYSTEM_CONTROL_2                 0x04
#define WM8350_SYSTEM_HIBERNATE                 0x05
#define WM8350_INTERFACE_CONTROL                0x06
#define WM8350_POWER_MGMT_1                     0x08
#define WM8350_POWER_MGMT_2                     0x09
#define WM8350_POWER_MGMT_3                     0x0A
#define WM8350_POWER_MGMT_4                     0x0B
#define WM8350_POWER_MGMT_5                     0x0C
#define WM8350_POWER_MGMT_6                     0x0D
#define WM8350_POWER_MGMT_7                     0x0E
#define WM8350_RTC_SECONDS_MINUTES              0x10
#define WM8350_RTC_HOURS_DAY                    0x11
#define WM8350_RTC_DATE_MONTH                   0x12
#define WM8350_RTC_YEAR                         0x13
#define WM8350_ALARM_SECONDS_MINUTES            0x14
#define WM8350_ALARM_HOURS_DAY                  0x15
#define WM8350_ALARM_DATE_MONTH                 0x16
#define WM8350_RTC_TIME_CONTROL                 0x17
#define WM8350_SYSTEM_INTERRUPTS                0x18
#define WM8350_INTERRUPT_STATUS_1               0x19
#define WM8350_INTERRUPT_STATUS_2               0x1A
#define WM8350_POWER_UP_INTERRUPT_STATUS        0x1B
#define WM8350_UNDER_VOLTAGE_INTERRUPT_STATUS   0x1C
#define WM8350_OVER_CURRENT_INTERRUPT_STATUS    0x1D
#define WM8350_GPIO_INTERRUPT_STATUS            0x1E
#define WM8350_COMPARATOR_INTERRUPT_STATUS      0x1F
#define WM8350_SYSTEM_INTERRUPTS_MASK           0x20
#define WM8350_INTERRUPT_STATUS_1_MASK          0x21
#define WM8350_INTERRUPT_STATUS_2_MASK          0x22
#define WM8350_POWER_UP_INTERRUPT_STATUS_MASK   0x23
#define WM8350_UNDER_VOLTAGE_INTERRUPT_STATUS_MASK 0x24
#define WM8350_OVER_CURRENT_INTERRUPT_STATUS_MASK 0x25
#define WM8350_GPIO_INTERRUPT_STATUS_MASK       0x26
#define WM8350_COMPARATOR_INTERRUPT_STATUS_MASK 0x27
#define WM8350_CLOCK_CONTROL_1                  0x28
#define WM8350_CLOCK_CONTROL_2                  0x29
#define WM8350_FLL_CONTROL_1                    0x2A
#define WM8350_FLL_CONTROL_2                    0x2B
#define WM8350_FLL_CONTROL_3                    0x2C
#define WM8350_FLL_CONTROL_4                    0x2D
#define WM8350_DAC_CONTROL                      0x30
#define WM8350_DAC_DIGITAL_VOLUME_L             0x32
#define WM8350_DAC_DIGITAL_VOLUME_R             0x33
#define WM8350_DAC_LR_RATE                      0x35
#define WM8350_DAC_CLOCK_CONTROL                0x36
#define WM8350_DAC_MUTE                         0x3A
#define WM8350_DAC_MUTE_VOLUME                  0x3B
#define WM8350_DAC_SIDE                         0x3C
#define WM8350_ADC_CONTROL                      0x40
#define WM8350_ADC_DIGITAL_VOLUME_L             0x42
#define WM8350_ADC_DIGITAL_VOLUME_R             0x43
#define WM8350_ADC_DIVIDER                      0x44
#define WM8350_ADC_LR_RATE                      0x46
#define WM8350_INPUT_CONTROL                    0x48
#define WM8350_IN3_INPUT_CONTROL                0x49
#define WM8350_MIC_BIAS_CONTROL                 0x4A
#define WM8350_OUTPUT_CONTROL                   0x4C
#define WM8350_JACK_DETECT                      0x4D
#define WM8350_ANTI_POP_CONTROL                 0x4E
#define WM8350_LEFT_INPUT_VOLUME                0x50
#define WM8350_RIGHT_INPUT_VOLUME               0x51
#define WM8350_LEFT_MIXER_CONTROL               0x58
#define WM8350_RIGHT_MIXER_CONTROL              0x59
#define WM8350_OUT3_MIXER_CONTROL               0x5C
#define WM8350_OUT4_MIXER_CONTROL               0x5D
#define WM8350_OUTPUT_LEFT_MIXER_VOLUME         0x60
#define WM8350_OUTPUT_RIGHT_MIXER_VOLUME        0x61
#define WM8350_INPUT_MIXER_VOLUME_L             0x62
#define WM8350_INPUT_MIXER_VOLUME_R             0x63
#define WM8350_INPUT_MIXER_VOLUME               0x64
#define WM8350_LOUT1_VOLUME                     0x68
#define WM8350_ROUT1_VOLUME                     0x69
#define WM8350_LOUT2_VOLUME                     0x6A
#define WM8350_ROUT2_VOLUME                     0x6B
#define WM8350_BEEP_VOLUME                      0x6F
#define WM8350_AI_FORMATING                     0x70
#define WM8350_ADC_DAC_COMP                     0x71
#define WM8350_AI_ADC_CONTROL                   0x72
#define WM8350_AI_DAC_CONTROL                   0x73
#define WM8350_AIF_TEST                         0x74
#define WM8350_GPIO_DEBOUNCE                    0x80
#define WM8350_GPIO_PIN_PULL_UP_CONTROL         0x81
#define WM8350_GPIO_PULL_DOWN_CONTROL           0x82
#define WM8350_GPIO_INTERRUPT_MODE              0x83
#define WM8350_GPIO_CONTROL                     0x85
#define WM8350_GPIO_CONFIGURATION_I_O           0x86
#define WM8350_GPIO_PIN_POLARITY_TYPE           0x87
#define WM8350_GPIO_FUNCTION_SELECT_1           0x8C
#define WM8350_GPIO_FUNCTION_SELECT_2           0x8D
#define WM8350_GPIO_FUNCTION_SELECT_3           0x8E
#define WM8350_GPIO_FUNCTION_SELECT_4           0x8F
#define WM8350_DIGITISER_CONTROL_1              0x90
#define WM8350_DIGITISER_CONTROL_2              0x91
#define WM8350_AUX1_READBACK                    0x98
#define WM8350_AUX2_READBACK                    0x99
#define WM8350_AUX3_READBACK                    0x9A
#define WM8350_AUX4_READBACK                    0x9B
#define WM8350_USB_VOLTAGE_READBACK             0x9C
#define WM8350_LINE_VOLTAGE_READBACK            0x9D
#define WM8350_BATT_VOLTAGE_READBACK            0x9E
#define WM8350_CHIP_TEMP_READBACK               0x9F
#define WM8350_GENERIC_COMPARATOR_CONTROL       0xA3
#define WM8350_GENERIC_COMPARATOR_1             0xA4
#define WM8350_GENERIC_COMPARATOR_2             0xA5
#define WM8350_GENERIC_COMPARATOR_3             0xA6
#define WM8350_GENERIC_COMPARATOR_4             0xA7
#define WM8350_BATTERY_CHARGER_CONTROL_1        0xA8
#define WM8350_BATTERY_CHARGER_CONTROL_2        0xA9
#define WM8350_BATTERY_CHARGER_CONTROL_3        0xAA
#define WM8350_CURRENT_SINK_DRIVER_A            0xAC
#define WM8350_CSA_FLASH_CONTROL                0xAD
#define WM8350_CURRENT_SINK_DRIVER_B            0xAE
#define WM8350_CSB_FLASH_CONTROL                0xAF
#define WM8350_DCDC_LDO_REQUESTED               0xB0
#define WM8350_DCDC_ACTIVE_OPTIONS              0xB1
#define WM8350_DCDC_SLEEP_OPTIONS               0xB2
#define WM8350_POWER_CHECK_COMPARATOR           0xB3
#define WM8350_DCDC1_CONTROL                    0xB4
#define WM8350_DCDC1_TIMEOUTS                   0xB5
#define WM8350_DCDC1_LOW_POWER                  0xB6
#define WM8350_DCDC2_CONTROL                    0xB7
#define WM8350_DCDC2_TIMEOUTS                   0xB8
#define WM8350_DCDC3_CONTROL                    0xBA
#define WM8350_DCDC3_TIMEOUTS                   0xBB
#define WM8350_DCDC3_LOW_POWER                  0xBC
#define WM8350_DCDC4_CONTROL                    0xBD
#define WM8350_DCDC4_TIMEOUTS                   0xBE
#define WM8350_DCDC4_LOW_POWER                  0xBF
#define WM8350_DCDC5_CONTROL                    0xC0
#define WM8350_DCDC5_TIMEOUTS                   0xC1
#define WM8350_DCDC6_CONTROL                    0xC3
#define WM8350_DCDC6_TIMEOUTS                   0xC4
#define WM8350_DCDC6_LOW_POWER                  0xC5
#define WM8350_LIMIT_SWITCH_CONTROL             0xC7
#define WM8350_LDO1_CONTROL                     0xC8
#define WM8350_LDO1_TIMEOUTS                    0xC9
#define WM8350_LDO1_LOW_POWER                   0xCA
#define WM8350_LDO2_CONTROL                     0xCB
#define WM8350_LDO2_TIMEOUTS                    0xCC
#define WM8350_LDO2_LOW_POWER                   0xCD
#define WM8350_LDO3_CONTROL                     0xCE
#define WM8350_LDO3_TIMEOUTS                    0xCF
#define WM8350_LDO3_LOW_POWER                   0xD0
#define WM8350_LDO4_CONTROL                     0xD1
#define WM8350_LDO4_TIMEOUTS                    0xD2
#define WM8350_LDO4_LOW_POWER                   0xD3
#define WM8350_VCC_FAULT_MASKS                  0xD7
#define WM8350_MAIN_BANDGAP_CONTROL             0xD8
#define WM8350_OSC_CONTROL                      0xD9
#define WM8350_RTC_TICK_CONTROL                 0xDA
#define WM8350_SECURITY                         0xDB
#define WM8350_RAM_BIST_1                       0xDC
#define WM8350_DCDC_LDO_STATUS                  0xE1
#define WM8350_GPIO_PIN_STATUS                  0xE6
#define WM8350_JACK_PIN_STATUS                  0xE7

#define WM8350_REGISTER_COUNT                   156
#define WM8350_MAX_REGISTER                     0xE9

/*
 * Field Definitions.
 */

/*
 * R0 (0x00) - Reset/ID
 */
#define WM8350_SW_RESET_CHIP_ID_MASK            0xFFFF  /* SW_RESET/CHIP_ID - [15:0] */

/*
 * R1 (0x01) - ID
 */
#define WM8350_CHIP_REV_MASK                    0x7000  /* CHIP_REV - [14:12] */
#define WM8350_CONF_STS_MASK                    0x0C00  /* CONF_STS - [11:10] */
#define WM8350_CUST_ID_MASK                     0x00FF  /* CUST_ID - [7:0] */

/*
 * R3 (0x03) - System Control 1
 */
#define WM8350_CHIP_ON                          0x8000  /* CHIP_ON */
#define WM8350_POWERCYCLE                       0x2000  /* POWERCYCLE */
#define WM8350_VCC_FAULT_OV                     0x1000  /* VCC_FAULT_OV */
#define WM8350_REG_RSTB_TIME_MASK               0x0C00  /* REG_RSTB_TIME - [11:10] */
#define WM8350_BG_SLEEP                         0x0200  /* BG_SLEEP */
#define WM8350_MEM_VALID                        0x0020  /* MEM_VALID */
#define WM8350_CHIP_SET_UP                      0x0010  /* CHIP_SET_UP */
#define WM8350_ON_DEB_T                         0x0008  /* ON_DEB_T */
#define WM8350_ON_POL                           0x0002  /* ON_POL */
#define WM8350_IRQ_POL                          0x0001  /* IRQ_POL */

/*
 * R4 (0x04) - System Control 2
 */
#define WM8350_USB_SUSPEND_8MA                  0x8000  /* USB_SUSPEND_8MA */
#define WM8350_USB_SUSPEND                      0x4000  /* USB_SUSPEND */
#define WM8350_USB_MSTR                         0x2000  /* USB_MSTR */
#define WM8350_USB_MSTR_SRC                     0x1000  /* USB_MSTR_SRC */
#define WM8350_USB_500MA                        0x0800  /* USB_500MA */
#define WM8350_USB_NOLIM                        0x0400  /* USB_NOLIM */
#define WM8350_WDOG_HIB_MODE                    0x0080  /* WDOG_HIB_MODE */
#define WM8350_WDOG_DEBUG                       0x0040  /* WDOG_DEBUG */
#define WM8350_WDOG_MODE_MASK                   0x0030  /* WDOG_MODE - [5:4] */
#define WM8350_WDOG_TO_MASK                     0x0007  /* WDOG_TO - [2:0] */

/*
 * R5 (0x05) - System Hibernate
 */
#define WM8350_HIBERNATE                        0x8000  /* HIBERNATE */
#define WM8350_WDOG_HIB_MODE                    0x0080  /* WDOG_HIB_MODE */
#define WM8350_REG_HIB_STARTUP_SEQ              0x0040  /* REG_HIB_STARTUP_SEQ */
#define WM8350_REG_RESET_HIB_MODE               0x0020  /* REG_RESET_HIB_MODE */
#define WM8350_RST_HIB_MODE                     0x0010  /* RST_HIB_MODE */
#define WM8350_IRQ_HIB_MODE                     0x0008  /* IRQ_HIB_MODE */
#define WM8350_MEMRST_HIB_MODE                  0x0004  /* MEMRST_HIB_MODE */
#define WM8350_PCCOMP_HIB_MODE                  0x0002  /* PCCOMP_HIB_MODE */
#define WM8350_TEMPMON_HIB_MODE                 0x0001  /* TEMPMON_HIB_MODE */

/*
 * R6 (0x06) - Interface Control
 */
#define WM8350_USE_DEV_PINS                     0x8000  /* USE_DEV_PINS */
#define WM8350_USE_DEV_PINS_MASK                0x8000  /* USE_DEV_PINS */
#define WM8350_USE_DEV_PINS_SHIFT                   15  /* USE_DEV_PINS */
#define WM8350_DEV_ADDR_MASK                    0x6000  /* DEV_ADDR - [14:13] */
#define WM8350_DEV_ADDR_SHIFT                       13  /* DEV_ADDR - [14:13] */
#define WM8350_CONFIG_DONE                      0x1000  /* CONFIG_DONE */
#define WM8350_CONFIG_DONE_MASK                 0x1000  /* CONFIG_DONE */
#define WM8350_CONFIG_DONE_SHIFT                    12  /* CONFIG_DONE */
#define WM8350_RECONFIG_AT_ON                   0x0800  /* RECONFIG_AT_ON */
#define WM8350_RECONFIG_AT_ON_MASK              0x0800  /* RECONFIG_AT_ON */
#define WM8350_RECONFIG_AT_ON_SHIFT                 11  /* RECONFIG_AT_ON */
#define WM8350_AUTOINC                          0x0200  /* AUTOINC */
#define WM8350_AUTOINC_MASK                     0x0200  /* AUTOINC */
#define WM8350_AUTOINC_SHIFT                         9  /* AUTOINC */
#define WM8350_ARA                              0x0100  /* ARA */
#define WM8350_ARA_MASK                         0x0100  /* ARA */
#define WM8350_ARA_SHIFT                             8  /* ARA */
#define WM8350_SPI_CFG                          0x0008  /* SPI_CFG */
#define WM8350_SPI_CFG_MASK                     0x0008  /* SPI_CFG */
#define WM8350_SPI_CFG_SHIFT                         3  /* SPI_CFG */
#define WM8350_SPI_4WIRE                        0x0004  /* SPI_4WIRE */
#define WM8350_SPI_4WIRE_MASK                   0x0004  /* SPI_4WIRE */
#define WM8350_SPI_4WIRE_SHIFT                       2  /* SPI_4WIRE */
#define WM8350_SPI_3WIRE                        0x0002  /* SPI_3WIRE */
#define WM8350_SPI_3WIRE_MASK                   0x0002  /* SPI_3WIRE */
#define WM8350_SPI_3WIRE_SHIFT                       1  /* SPI_3WIRE */

/* Bit values for R06 (0x06) */
#define WM8350_USE_DEV_PINS_PRIMARY                  0  /* Primary control interface */
#define WM8350_USE_DEV_PINS_DEV                      1  /* Secondary control interface */

#define WM8350_DEV_ADDR_34                           0  /* 2-wire address 0x1A/0x34 */
#define WM8350_DEV_ADDR_36                           1  /* 2-wire address 0x1B/0x36 */
#define WM8350_DEV_ADDR_3C                           2  /* 2-wire address 0x1E/0x3C */
#define WM8350_DEV_ADDR_3E                           3  /* 2-wire address 0x1F/0x3E */

#define WM8350_CONFIG_DONE_OFF                       0  /* Not programmed */
#define WM8350_CONFIG_DONE_DONE                      1  /* Programming complete */

#define WM8350_RECONFIG_AT_ON_OFF                    0  /* Don't reset registers on ON-event */
#define WM8350_RECONFIG_AT_ON_ON                     1  /* Reset registers on ON-event */

#define WM8350_AUTOINC_OFF                           0  /* No register auto-increment */
#define WM8350_AUTOINC_ON                            1  /* Enable register auto-increment */

#define WM8350_ARA_OFF                               0  /* Alert response address disabled */
#define WM8350_ARA_ON                                1  /* Alert response address enabled */

#define WM8350_SPI_CFG_CMOS                          0  /* SDOUT is CMOS */
#define WM8350_SPI_CFG_OD                            1  /* SDOUT is open-drain */

#define WM8350_SPI_4WIRE_3WIRE                       0  /* read data output on SDA */
#define WM8350_SPI_4WIRE_4WIRE                       1  /* read data output on SDOUT */

#define WM8350_SPI_3WIRE_I2C                         0  /* Select 2-wire interface */
#define WM8350_SPI_3WIRE_SPI                         1  /* Select 3-wire interface */

/*
 * R8 (0x08) - Power mgmt (1)
 */
#define WM8350_CODEC_ISEL_MASK                  0xC000  /* CODEC_ISEL - [15:14] */
#define WM8350_VBUFEN                           0x2000  /* VBUFEN */
#define WM8350_OUTPUT_DRAIN_EN                  0x0400  /* OUTPUT_DRAIN_EN */
#define WM8350_MIC_DET_ENA                      0x0100  /* MIC_DET_ENA */
#define WM8350_BIASEN                           0x0020  /* BIASEN */
#define WM8350_MICBEN                           0x0010  /* MICBEN */
#define WM8350_VMIDEN                           0x0004  /* VMIDEN */
#define WM8350_VMID_MASK                        0x0003  /* VMID - [1:0] */
#define WM8350_VMID_SHIFT                            0  /* VMID - [1:0] */

/* Bit values for R08 (0x08) */
#define WM8350_CODEC_ISEL_1_5                        0  /* 0 = x1.5 */
#define WM8350_CODEC_ISEL_1_0                        1  /* 0 = x1.0 */
#define WM8350_CODEC_ISEL_0_75                       2  /* 0 = x0.75 */
#define WM8350_CODEC_ISEL_0_5                        3  /* 0 = x0.5 */

#define WM8350_VMID_OFF                              0  /* 0 = 500k resistor string */
#define WM8350_VMID_500K                             1  /* 1 = 160k resistor string */
#define WM8350_VMID_100K                             2  /* 2 = 80k resistor string */
#define WM8350_VMID_10K                              3  /* 3 = 40k resistor string */

/*
 * R9 (0x09) - Power mgmt (2)
 */
#define WM8350_IN3R_ENA                         0x0800  /* IN3R_ENA */
#define WM8350_IN3L_ENA                         0x0400  /* IN3L_ENA */
#define WM8350_INR_ENA                          0x0200  /* INR_ENA */
#define WM8350_INL_ENA                          0x0100  /* INL_ENA */
#define WM8350_MIXINR_ENA                       0x0080  /* MIXINR_ENA */
#define WM8350_MIXINL_ENA                       0x0040  /* MIXINL_ENA */
#define WM8350_OUT4_ENA                         0x0020  /* OUT4_ENA */
#define WM8350_OUT3_ENA                         0x0010  /* OUT3_ENA */
#define WM8350_MIXOUTR_ENA                      0x0002  /* MIXOUTR_ENA */
#define WM8350_MIXOUTL_ENA                      0x0001  /* MIXOUTL_ENA */

/*
 * R10 (0x0A) - Power mgmt (3)
 */
#define WM8350_IN3R_TO_OUT2R                    0x0080  /* IN3R_TO_OUT2R */
#define WM8350_OUT2R_ENA                        0x0008  /* OUT2R_ENA */
#define WM8350_OUT2L_ENA                        0x0004  /* OUT2L_ENA */
#define WM8350_OUT1R_ENA                        0x0002  /* OUT1R_ENA */
#define WM8350_OUT1L_ENA                        0x0001  /* OUT1L_ENA */

/*
 * R11 (0x0B) - Power mgmt (4)
 */
#define WM8350_SYSCLK_ENA                       0x4000  /* SYSCLK_ENA */
#define WM8350_ADC_HPF_ENA                      0x2000  /* ADC_HPF_ENA */
#define WM8350_FLL_ENA                          0x0800  /* FLL_ENA */
#define WM8350_FLL_OSC_ENA                      0x0400  /* FLL_OSC_ENA */
#define WM8350_TOCLK_ENA                        0x0100  /* TOCLK_ENA */
#define WM8350_DACR_ENA                         0x0020  /* DACR_ENA */
#define WM8350_DACL_ENA                         0x0010  /* DACL_ENA */
#define WM8350_ADCR_ENA                         0x0008  /* ADCR_ENA */
#define WM8350_ADCL_ENA                         0x0004  /* ADCL_ENA */

/*
 * R12 (0x0C) - Power mgmt (5)
 */
#define WM8350_CODEC_ENA                        0x1000  /* CODEC_ENA */
#define WM8350_RTC_TICK_ENA                     0x0800  /* RTC_TICK_ENA */
#define WM8350_OSC32K_ENA                       0x0400  /* OSC32K_ENA */
#define WM8350_CHG_ENA                          0x0200  /* CHG_ENA */
#define WM8350_ACC_DET_ENA                      0x0100  /* ACC_DET_ENA */
#define WM8350_AUXADC_ENA                       0x0080  /* AUXADC_ENA */
#define WM8350_DCMP4_ENA                        0x0008  /* DCMP4_ENA */
#define WM8350_DCMP3_ENA                        0x0004  /* DCMP3_ENA */
#define WM8350_DCMP2_ENA                        0x0002  /* DCMP2_ENA */
#define WM8350_DCMP1_ENA                        0x0001  /* DCMP1_ENA */

/*
 * R13 (0x0D) - Power mgmt (6)
 */
#define WM8350_LS_ENA                           0x8000  /* LS_ENA */
#define WM8350_LDO4_ENA                         0x0800  /* LDO4_ENA */
#define WM8350_LDO3_ENA                         0x0400  /* LDO3_ENA */
#define WM8350_LDO2_ENA                         0x0200  /* LDO2_ENA */
#define WM8350_LDO1_ENA                         0x0100  /* LDO1_ENA */
#define WM8350_DC6_ENA                          0x0020  /* DC6_ENA */
#define WM8350_DC5_ENA                          0x0010  /* DC5_ENA */
#define WM8350_DC4_ENA                          0x0008  /* DC4_ENA */
#define WM8350_DC3_ENA                          0x0004  /* DC3_ENA */
#define WM8350_DC2_ENA                          0x0002  /* DC2_ENA */
#define WM8350_DC1_ENA                          0x0001  /* DC1_ENA */

/*
 * R14 (0x0E) - Power mgmt (7)
 */
#define WM8350_CS2_ENA                          0x0002  /* CS2_ENA */
#define WM8350_CS1_ENA                          0x0001  /* CS1_ENA */

/*
 * R16 (0x10) - RTC Seconds/Minutes
 */
#define WM8350_RTC_MINS_MASK                    0x7F00  /* RTC_MINS - [14:8] */
#define WM8350_RTC_MINS_SHIFT                        8  /* RTC_MINS - [14:8] */
#define WM8350_RTC_SECS_MASK                    0x007F  /* RTC_SECS - [6:0] */
#define WM8350_RTC_SECS_SHIFT                        0  /* RTC_SECS - [6:0] */

/*
 * R17 (0x11) - RTC Hours/Day
 */
#define WM8350_RTC_DAY_MASK                     0x0700  /* RTC_DAY - [10:8] */
#define WM8350_RTC_DAY_SHIFT                         8  /* RTC_DAY - [10:8] */
#define WM8350_RTC_HPM_MASK                     0x0020  /* RTC_HPM */
#define WM8350_RTC_HPM_SHIFT                         5  /* RTC_HPM */
#define WM8350_RTC_HRS_MASK                     0x001F  /* RTC_HRS - [4:0] */
#define WM8350_RTC_HRS_SHIFT                         0  /* RTC_HRS - [4:0] */

/* Bit values for R21 (0x15) */
#define WM8350_RTC_DAY_SUN                           1  /* 1 = Sunday */
#define WM8350_RTC_DAY_MON                           2  /* 2 = Monday */
#define WM8350_RTC_DAY_TUE                           3  /* 3 = Tuesday */
#define WM8350_RTC_DAY_WED                           4  /* 4 = Wednesday */
#define WM8350_RTC_DAY_THU                           5  /* 5 = Thursday */
#define WM8350_RTC_DAY_FRI                           6  /* 6 = Friday */
#define WM8350_RTC_DAY_SAT                           7  /* 7 = Saturday */

#define WM8350_RTC_HPM_AM                            0  /* 0 = AM */
#define WM8350_RTC_HPM_PM                            1  /* 1 = PM */


/*
 * R18 (0x12) - RTC Date/Month
 */
#define WM8350_RTC_MTH_MASK                     0x1F00  /* RTC_MTH - [12:8] */
#define WM8350_RTC_MTH_SHIFT                         8  /* RTC_MTH - [12:8] */
#define WM8350_RTC_DATE_MASK                    0x003F  /* RTC_DATE - [5:0] */
#define WM8350_RTC_DATE_SHIFT                        0  /* RTC_DATE - [5:0] */

/* Bit values for R22 (0x16) */
#define WM8350_RTC_MTH_JAN                           1  /* 1 = January */
#define WM8350_RTC_MTH_FEB                           2  /* 2 = February */
#define WM8350_RTC_MTH_MAR                           3  /* 3 = March */
#define WM8350_RTC_MTH_APR                           4  /* 4 = April */
#define WM8350_RTC_MTH_MAY                           5  /* 5 = May */
#define WM8350_RTC_MTH_JUN                           6  /* 6 = June */
#define WM8350_RTC_MTH_JUL                           7  /* 7 = July */
#define WM8350_RTC_MTH_AUG                           8  /* 8 = August */
#define WM8350_RTC_MTH_SEP                           9  /* 9 = September */
#define WM8350_RTC_MTH_OCT                          10  /* 10 = October */
#define WM8350_RTC_MTH_NOV                          11  /* 11 = November */
#define WM8350_RTC_MTH_DEC                          12  /* 12 = December */
#define WM8350_RTC_MTH_JAN_BCD                    0x01  /* 1 = January */
#define WM8350_RTC_MTH_FEB_BCD                    0x02  /* 2 = February */
#define WM8350_RTC_MTH_MAR_BCD                    0x03  /* 3 = March */
#define WM8350_RTC_MTH_APR_BCD                    0x04  /* 4 = April */
#define WM8350_RTC_MTH_MAY_BCD                    0x05  /* 5 = May */
#define WM8350_RTC_MTH_JUN_BCD                    0x06  /* 6 = June */
#define WM8350_RTC_MTH_JUL_BCD                    0x07  /* 7 = July */
#define WM8350_RTC_MTH_AUG_BCD                    0x08  /* 8 = August */
#define WM8350_RTC_MTH_SEP_BCD                    0x09  /* 9 = September */
#define WM8350_RTC_MTH_OCT_BCD                    0x10  /* 10 = October */
#define WM8350_RTC_MTH_NOV_BCD                    0x11  /* 11 = November */
#define WM8350_RTC_MTH_DEC_BCD                    0x12  /* 12 = December */

/*
 * R19 (0x13) - RTC Year
 */
#define WM8350_RTC_YHUNDREDS_MASK               0x3F00  /* RTC_YHUNDREDS - [13:8] */
#define WM8350_RTC_YHUNDREDS_SHIFT                   8  /* RTC_YHUNDREDS - [13:8] */
#define WM8350_RTC_YUNITS_MASK                  0x00FF  /* RTC_YUNITS - [7:0] */
#define WM8350_RTC_YUNITS_SHIFT                      0  /* RTC_YUNITS - [7:0] */

/*
 * R20 (0x14) - Alarm Seconds/Minutes
 */
#define WM8350_RTC_ALMMINS_MASK                 0x7F00  /* RTC_ALMMINS - [14:8] */
#define WM8350_RTC_ALMMINS_SHIFT                     8  /* RTC_ALMMINS - [14:8] */
#define WM8350_RTC_ALMSECS_MASK                 0x007F  /* RTC_ALMSECS - [6:0] */
#define WM8350_RTC_ALMSECS_SHIFT                     0  /* RTC_ALMSECS - [6:0] */

/* Bit values for R20 (0x14) */
#define WM8350_RTC_ALMMINS_DONT_CARE                -1  /* -1 = don't care */

#define WM8350_RTC_ALMSECS_DONT_CARE                -1  /* -1 = don't care */

/*
 * R21 (0x15) - Alarm Hours/Day
 */
#define WM8350_RTC_ALMDAY_MASK                  0x0F00  /* RTC_ALMDAY - [11:8] */
#define WM8350_RTC_ALMDAY_SHIFT                      8  /* RTC_ALMDAY - [11:8] */
#define WM8350_RTC_ALMHPM_MASK                  0x0020  /* RTC_ALMHPM */
#define WM8350_RTC_ALMHPM_SHIFT                      5  /* RTC_ALMHPM */
#define WM8350_RTC_ALMHRS_MASK                  0x001F  /* RTC_ALMHRS - [4:0] */
#define WM8350_RTC_ALMHRS_SHIFT                      0  /* RTC_ALMHRS - [4:0] */

/* Bit values for R21 (0x15) */
#define WM8350_RTC_ALMDAY_DONT_CARE                 -1  /* -1 = don't care */
#define WM8350_RTC_ALMDAY_SUN                        1  /* 1 = Sunday */
#define WM8350_RTC_ALMDAY_MON                        2  /* 2 = Monday */
#define WM8350_RTC_ALMDAY_TUE                        3  /* 3 = Tuesday */
#define WM8350_RTC_ALMDAY_WED                        4  /* 4 = Wednesday */
#define WM8350_RTC_ALMDAY_THU                        5  /* 5 = Thursday */
#define WM8350_RTC_ALMDAY_FRI                        6  /* 6 = Friday */
#define WM8350_RTC_ALMDAY_SAT                        7  /* 7 = Saturday */

#define WM8350_RTC_ALMHPM_AM                         0  /* 0 = AM */
#define WM8350_RTC_ALMHPM_PM                         1  /* 1 = PM */

#define WM8350_RTC_ALMHRS_DONT_CARE                 -1  /* -1 = don't care */

/*
 * R22 (0x16) - Alarm Date/Month
 */
#define WM8350_RTC_ALMMTH_MASK                  0x1F00  /* RTC_ALMMTH - [12:8] */
#define WM8350_RTC_ALMMTH_SHIFT                      8  /* RTC_ALMMTH - [12:8] */
#define WM8350_RTC_ALMDATE_MASK                 0x003F  /* RTC_ALMDATE - [5:0] */
#define WM8350_RTC_ALMDATE_SHIFT                     0  /* RTC_ALMDATE - [5:0] */

/* Bit values for R22 (0x16) */
#define WM8350_RTC_ALMDATE_DONT_CARE                -1  /* -1 = don't care */

#define WM8350_RTC_ALMMTH_DONT_CARE                 -1  /* -1 = don't care */
#define WM8350_RTC_ALMMTH_JAN                        1  /* 1 = January */
#define WM8350_RTC_ALMMTH_FEB                        2  /* 2 = February */
#define WM8350_RTC_ALMMTH_MAR                        3  /* 3 = March */
#define WM8350_RTC_ALMMTH_APR                        4  /* 4 = April */
#define WM8350_RTC_ALMMTH_MAY                        5  /* 5 = May */
#define WM8350_RTC_ALMMTH_JUN                        6  /* 6 = June */
#define WM8350_RTC_ALMMTH_JUL                        7  /* 7 = July */
#define WM8350_RTC_ALMMTH_AUG                        8  /* 8 = August */
#define WM8350_RTC_ALMMTH_SEP                        9  /* 9 = September */
#define WM8350_RTC_ALMMTH_OCT                       10  /* 10 = October */
#define WM8350_RTC_ALMMTH_NOV                       11  /* 11 = November */
#define WM8350_RTC_ALMMTH_DEC                       12  /* 12 = December */
#define WM8350_RTC_ALMMTH_JAN_BCD                 0x01  /* 1 = January */
#define WM8350_RTC_ALMMTH_FEB_BCD                 0x02  /* 2 = February */
#define WM8350_RTC_ALMMTH_MAR_BCD                 0x03  /* 3 = March */
#define WM8350_RTC_ALMMTH_APR_BCD                 0x04  /* 4 = April */
#define WM8350_RTC_ALMMTH_MAY_BCD                 0x05  /* 5 = May */
#define WM8350_RTC_ALMMTH_JUN_BCD                 0x06  /* 6 = June */
#define WM8350_RTC_ALMMTH_JUL_BCD                 0x07  /* 7 = July */
#define WM8350_RTC_ALMMTH_AUG_BCD                 0x08  /* 8 = August */
#define WM8350_RTC_ALMMTH_SEP_BCD                 0x09  /* 9 = September */
#define WM8350_RTC_ALMMTH_OCT_BCD                 0x10  /* 10 = October */
#define WM8350_RTC_ALMMTH_NOV_BCD                 0x11  /* 11 = November */
#define WM8350_RTC_ALMMTH_DEC_BCD                 0x12  /* 12 = December */

/*
 * R23 (0x17) - RTC Time Control
 */
#define WM8350_RTC_BCD                          0x8000  /* RTC_BCD */
#define WM8350_RTC_BCD_MASK                     0x8000  /* RTC_BCD */
#define WM8350_RTC_BCD_SHIFT                        15  /* RTC_BCD */
#define WM8350_RTC_12HR                         0x4000  /* RTC_12HR */
#define WM8350_RTC_12HR_MASK                    0x4000  /* RTC_12HR */
#define WM8350_RTC_12HR_SHIFT                       14  /* RTC_12HR */
#define WM8350_RTC_DST                          0x2000  /* RTC_DST */
#define WM8350_RTC_DST_MASK                     0x2000  /* RTC_DST */
#define WM8350_RTC_DST_SHIFT                        13  /* RTC_DST */
#define WM8350_RTC_SET                          0x0800  /* RTC_SET */
#define WM8350_RTC_SET_MASK                     0x0800  /* RTC_SET */
#define WM8350_RTC_SET_SHIFT                        11  /* RTC_SET */
#define WM8350_RTC_STS                          0x0400  /* RTC_STS */
#define WM8350_RTC_STS_MASK                     0x0400  /* RTC_STS */
#define WM8350_RTC_STS_SHIFT                        10  /* RTC_STS */
#define WM8350_RTC_ALMSET                       0x0200  /* RTC_ALMSET */
#define WM8350_RTC_ALMSET_MASK                  0x0200  /* RTC_ALMSET */
#define WM8350_RTC_ALMSET_SHIFT                      9  /* RTC_ALMSET */
#define WM8350_RTC_ALMSTS                       0x0100  /* RTC_ALMSTS */
#define WM8350_RTC_ALMSTS_MASK                  0x0100  /* RTC_ALMSTS */
#define WM8350_RTC_ALMSTS_SHIFT                      8  /* RTC_ALMSTS */
#define WM8350_RTC_PINT                         0x0070  /* RTC_PINT - [6:4] */
#define WM8350_RTC_PINT_MASK                    0x0070  /* RTC_PINT - [6:4] */
#define WM8350_RTC_PINT_SHIFT                        4  /* RTC_PINT - [6:4] */
#define WM8350_RTC_DSW                          0x000F  /* RTC_DSW - [3:0] */
#define WM8350_RTC_DSW_MASK                     0x000F  /* RTC_DSW - [3:0] */
#define WM8350_RTC_DSW_SHIFT                         0  /* RTC_DSW - [3:0] */

/* Bit values for R23 (0x17) */
#define WM8350_RTC_BCD_BINARY                        0  /* 0 = binary */
#define WM8350_RTC_BCD_BCD                           1  /* 1 = BCD */

#define WM8350_RTC_12HR_24HR                         0  /* 0 = 24-hour */
#define WM8350_RTC_12HR_12HR                         1  /* 1 = 12-hour */

#define WM8350_RTC_DST_DISABLED                      0  /* 0 = disabled */
#define WM8350_RTC_DST_ENABLED                       1  /* 1 = enabled */

#define WM8350_RTC_SET_RUN                           0  /* 0 = let RTC run */
#define WM8350_RTC_SET_SET                           1  /* 1 = request RTC set */

#define WM8350_RTC_STS_RUNNING                       0  /* 0 = RTC running */
#define WM8350_RTC_STS_STOPPED                       1  /* 1 = RTC stopped */

#define WM8350_RTC_ALMSET_RUN                        0  /* 0 = let RTC alarm run */
#define WM8350_RTC_ALMSET_SET                        1  /* 1 = request RTC alarm set */

#define WM8350_RTC_ALMSTS_RUNNING                    0  /* 0 = RTC alarm running */
#define WM8350_RTC_ALMSTS_STOPPED                    1  /* 1 = RTC alarm stopped */

#define WM8350_RTC_PINT_DISABLED                     0  /* 0 = disabled */
#define WM8350_RTC_PINT_SECS                         1  /* 1 = seconds rollover */
#define WM8350_RTC_PINT_MINS                         2  /* 2 = minutes rollover */
#define WM8350_RTC_PINT_HRS                          3  /* 3 = hours rollover */
#define WM8350_RTC_PINT_DAYS                         4  /* 4 = days rollover */
#define WM8350_RTC_PINT_MTHS                         5  /* 5 = months rollover */

#define WM8350_RTC_DSW_DISABLED                      0  /* 0 = disabled */
#define WM8350_RTC_DSW_1HZ                           1  /* 1 = 1Hz */
#define WM8350_RTC_DSW_2HZ                           2  /* 2 = 2Hz */
#define WM8350_RTC_DSW_4HZ                           3  /* 3 = 4Hz */
#define WM8350_RTC_DSW_8HZ                           4  /* 4 = 8Hz */
#define WM8350_RTC_DSW_16HZ                          5  /* 5 = 16Hz */
#define WM8350_RTC_DSW_32HZ                          6  /* 6 = 32Hz */
#define WM8350_RTC_DSW_64HZ                          7  /* 7 = 64Hz */
#define WM8350_RTC_DSW_128HZ                         8  /* 8 = 128Hz */
#define WM8350_RTC_DSW_256HZ                         9  /* 9 = 256Hz */
#define WM8350_RTC_DSW_512HZ                        10  /* 10 = 512Hz */
#define WM8350_RTC_DSW_1024HZ                       11  /* 11 = 1024Hz */

/*
 * R24 (0x18) - System Interrupts
 */
#define WM8350_OC_INT                           0x2000  /* OC_INT */
#define WM8350_UV_INT                           0x1000  /* UV_INT */
#define WM8350_PUTO_INT                         0x0800  /* PUTO_INT */
#define WM8350_CS_INT                           0x0200  /* CS_INT */
#define WM8350_EXT_INT                          0x0100  /* EXT_INT */
#define WM8350_CODEC_INT                        0x0080  /* CODEC_INT */
#define WM8350_GP_INT                           0x0040  /* GP_INT */
#define WM8350_AUXADC_INT                       0x0020  /* AUXADC_INT */
#define WM8350_RTC_INT                          0x0010  /* RTC_INT */
#define WM8350_SYS_INT                          0x0008  /* SYS_INT */
#define WM8350_CHG_INT                          0x0004  /* CHG_INT */
#define WM8350_USB_INT                          0x0002  /* USB_INT */
#define WM8350_WKUP_INT                         0x0001  /* WKUP_INT */

/*
 * R25 (0x19) - Interrupt Status 1
 */
#define WM8350_CHG_BAT_HOT_EINT                 0x8000  /* CHG_BAT_HOT_EINT */
#define WM8350_CHG_BAT_COLD_EINT                0x4000  /* CHG_BAT_COLD_EINT */
#define WM8350_CHG_BAT_FAIL_EINT                0x2000  /* CHG_BAT_FAIL_EINT */
#define WM8350_CHG_TO_EINT                      0x1000  /* CHG_TO_EINT */
#define WM8350_CHG_END_EINT                     0x0800  /* CHG_END_EINT */
#define WM8350_CHG_START_EINT                   0x0400  /* CHG_START_EINT */
#define WM8350_CHG_FAST_RDY_EINT                0x0200  /* CHG_FAST_RDY_EINT */
#define WM8350_RTC_PER_EINT                     0x0080  /* RTC_PER_EINT */
#define WM8350_RTC_SEC_EINT                     0x0040  /* RTC_SEC_EINT */
#define WM8350_RTC_ALM_EINT                     0x0020  /* RTC_ALM_EINT */
#define WM8350_CHG_VBATT_LT_3P9_EINT            0x0004  /* CHG_VBATT_LT_3P9_EINT */
#define WM8350_CHG_VBATT_LT_3P1_EINT            0x0002  /* CHG_VBATT_LT_3P1_EINT */
#define WM8350_CHG_VBATT_LT_2P85_EINT           0x0001  /* CHG_VBATT_LT_2P85_EINT */

/*
 * R26 (0x1A) - Interrupt Status 2
 */
#define WM8350_CS1_EINT                         0x2000  /* CS1_EINT */
#define WM8350_CS2_EINT                         0x1000  /* CS2_EINT */
#define WM8350_USB_LIMIT_EINT                   0x0400  /* USB_LIMIT_EINT */
#define WM8350_AUXADC_DATARDY_EINT              0x0100  /* AUXADC_DATARDY_EINT */
#define WM8350_AUXADC_DCOMP4_EINT               0x0080  /* AUXADC_DCOMP4_EINT */
#define WM8350_AUXADC_DCOMP3_EINT               0x0040  /* AUXADC_DCOMP3_EINT */
#define WM8350_AUXADC_DCOMP2_EINT               0x0020  /* AUXADC_DCOMP2_EINT */
#define WM8350_AUXADC_DCOMP1_EINT               0x0010  /* AUXADC_DCOMP1_EINT */
#define WM8350_SYS_HYST_COMP_FAIL_EINT          0x0008  /* SYS_HYST_COMP_FAIL_EINT */
#define WM8350_SYS_CHIP_GT115_EINT              0x0004  /* SYS_CHIP_GT115_EINT */
#define WM8350_SYS_CHIP_GT140_EINT              0x0002  /* SYS_CHIP_GT140_EINT */
#define WM8350_SYS_WDOG_TO_EINT                 0x0001  /* SYS_WDOG_TO_EINT */

/*
 * R27 (0x1B) - Power Up Interrupt Status
 */
#define WM8350_PUTO_LDO4_EINT                   0x0800  /* PUTO_LDO4_EINT */
#define WM8350_PUTO_LDO3_EINT                   0x0400  /* PUTO_LDO3_EINT */
#define WM8350_PUTO_LDO2_EINT                   0x0200  /* PUTO_LDO2_EINT */
#define WM8350_PUTO_LDO1_EINT                   0x0100  /* PUTO_LDO1_EINT */
#define WM8350_PUTO_DC6_EINT                    0x0020  /* PUTO_DC6_EINT */
#define WM8350_PUTO_DC5_EINT                    0x0010  /* PUTO_DC5_EINT */
#define WM8350_PUTO_DC4_EINT                    0x0008  /* PUTO_DC4_EINT */
#define WM8350_PUTO_DC3_EINT                    0x0004  /* PUTO_DC3_EINT */
#define WM8350_PUTO_DC2_EINT                    0x0002  /* PUTO_DC2_EINT */
#define WM8350_PUTO_DC1_EINT                    0x0001  /* PUTO_DC1_EINT */

/*
 * R28 (0x1C) - Under Voltage Interrupt status
 */
#define WM8350_UV_LDO4_EINT                     0x0800  /* UV_LDO4_EINT */
#define WM8350_UV_LDO3_EINT                     0x0400  /* UV_LDO3_EINT */
#define WM8350_UV_LDO2_EINT                     0x0200  /* UV_LDO2_EINT */
#define WM8350_UV_LDO1_EINT                     0x0100  /* UV_LDO1_EINT */
#define WM8350_UV_DC6_EINT                      0x0020  /* UV_DC6_EINT */
#define WM8350_UV_DC5_EINT                      0x0010  /* UV_DC5_EINT */
#define WM8350_UV_DC4_EINT                      0x0008  /* UV_DC4_EINT */
#define WM8350_UV_DC3_EINT                      0x0004  /* UV_DC3_EINT */
#define WM8350_UV_DC2_EINT                      0x0002  /* UV_DC2_EINT */
#define WM8350_UV_DC1_EINT                      0x0001  /* UV_DC1_EINT */

/*
 * R29 (0x1D) - Over Current Interrupt status
 */
#define WM8350_OC_LS_EINT                       0x8000  /* OC_LS_EINT */

/*
 * R30 (0x1E) - GPIO Interrupt Status
 */
#define WM8350_GP12_EINT                        0x1000  /* GP12_EINT */
#define WM8350_GP11_EINT                        0x0800  /* GP11_EINT */
#define WM8350_GP10_EINT                        0x0400  /* GP10_EINT */
#define WM8350_GP9_EINT                         0x0200  /* GP9_EINT */
#define WM8350_GP8_EINT                         0x0100  /* GP8_EINT */
#define WM8350_GP7_EINT                         0x0080  /* GP7_EINT */
#define WM8350_GP6_EINT                         0x0040  /* GP6_EINT */
#define WM8350_GP5_EINT                         0x0020  /* GP5_EINT */
#define WM8350_GP4_EINT                         0x0010  /* GP4_EINT */
#define WM8350_GP3_EINT                         0x0008  /* GP3_EINT */
#define WM8350_GP2_EINT                         0x0004  /* GP2_EINT */
#define WM8350_GP1_EINT                         0x0002  /* GP1_EINT */
#define WM8350_GP0_EINT                         0x0001  /* GP0_EINT */

/*
 * R31 (0x1F) - Comparator Interrupt Status
 */
#define WM8350_EXT_USB_FB_EINT                  0x8000  /* EXT_USB_FB_EINT */
#define WM8350_EXT_WALL_FB_EINT                 0x4000  /* EXT_WALL_FB_EINT */
#define WM8350_EXT_BAT_FB_EINT                  0x2000  /* EXT_BAT_FB_EINT */
#define WM8350_CODEC_JCK_DET_L_EINT             0x0800  /* CODEC_JCK_DET_L_EINT */
#define WM8350_CODEC_JCK_DET_R_EINT             0x0400  /* CODEC_JCK_DET_R_EINT */
#define WM8350_CODEC_MICSCD_EINT                0x0200  /* CODEC_MICSCD_EINT */
#define WM8350_CODEC_MICD_EINT                  0x0100  /* CODEC_MICD_EINT */
#define WM8350_WKUP_OFF_STATE_EINT              0x0040  /* WKUP_OFF_STATE_EINT */
#define WM8350_WKUP_HIB_STATE_EINT              0x0020  /* WKUP_HIB_STATE_EINT */
#define WM8350_WKUP_CONV_FAULT_EINT             0x0010  /* WKUP_CONV_FAULT_EINT */
#define WM8350_WKUP_WDOG_RST_EINT               0x0008  /* WKUP_WDOG_RST_EINT */
#define WM8350_WKUP_GP_PWR_ON_EINT              0x0004  /* WKUP_GP_PWR_ON_EINT */
#define WM8350_WKUP_ONKEY_EINT                  0x0002  /* WKUP_ONKEY_EINT */
#define WM8350_WKUP_GP_WAKEUP_EINT              0x0001  /* WKUP_GP_WAKEUP_EINT */

/*
 * R32 (0x20) - System Interrupts Mask
 */
#define WM8350_IM_OC_INT                        0x2000  /* IM_OC_INT */
#define WM8350_IM_UV_INT                        0x1000  /* IM_UV_INT */
#define WM8350_IM_PUTO_INT                      0x0800  /* IM_PUTO_INT */
#define WM8350_IM_SPARE_INT                     0x0400  /* IM_SPARE_INT */
#define WM8350_IM_CS_INT                        0x0200  /* IM_CS_INT */
#define WM8350_IM_EXT_INT                       0x0100  /* IM_EXT_INT */
#define WM8350_IM_CODEC_INT                     0x0080  /* IM_CODEC_INT */
#define WM8350_IM_GP_INT                        0x0040  /* IM_GP_INT */
#define WM8350_IM_AUXADC_INT                    0x0020  /* IM_AUXADC_INT */
#define WM8350_IM_RTC_INT                       0x0010  /* IM_RTC_INT */
#define WM8350_IM_SYS_INT                       0x0008  /* IM_SYS_INT */
#define WM8350_IM_CHG_INT                       0x0004  /* IM_CHG_INT */
#define WM8350_IM_USB_INT                       0x0002  /* IM_USB_INT */
#define WM8350_IM_WKUP_INT                      0x0001  /* IM_WKUP_INT */

/*
 * R33 (0x21) - Interrupt Status 1 Mask
 */
#define WM8350_IM_CHG_BAT_HOT_EINT              0x8000  /* IM_CHG_BAT_HOT_EINT */
#define WM8350_IM_CHG_BAT_COLD_EINT             0x4000  /* IM_CHG_BAT_COLD_EINT */
#define WM8350_IM_CHG_BAT_FAIL_EINT             0x2000  /* IM_CHG_BAT_FAIL_EINT */
#define WM8350_IM_CHG_TO_EINT                   0x1000  /* IM_CHG_TO_EINT */
#define WM8350_IM_CHG_END_EINT                  0x0800  /* IM_CHG_END_EINT */
#define WM8350_IM_CHG_START_EINT                0x0400  /* IM_CHG_START_EINT */
#define WM8350_IM_CHG_FAST_RDY_EINT             0x0200  /* IM_CHG_FAST_RDY_EINT */
#define WM8350_IM_RTC_PER_EINT                  0x0080  /* IM_RTC_PER_EINT */
#define WM8350_IM_RTC_SEC_EINT                  0x0040  /* IM_RTC_SEC_EINT */
#define WM8350_IM_RTC_ALM_EINT                  0x0020  /* IM_RTC_ALM_EINT */
#define WM8350_IM_CHG_VBATT_LT_3P9_EINT         0x0004  /* IM_CHG_VBATT_LT_3P9_EINT */
#define WM8350_IM_CHG_VBATT_LT_3P1_EINT         0x0002  /* IM_CHG_VBATT_LT_3P1_EINT */
#define WM8350_IM_CHG_VBATT_LT_2P85_EINT        0x0001  /* IM_CHG_VBATT_LT_2P85_EINT */

/*
 * R34 (0x22) - Interrupt Status 2 Mask
 */
#define WM8350_IM_SPARE2_EINT                   0x8000  /* IM_SPARE2_EINT */
#define WM8350_IM_SPARE1_EINT                   0x4000  /* IM_SPARE1_EINT */
#define WM8350_IM_CS1_EINT                      0x2000  /* IM_CS1_EINT */
#define WM8350_IM_CS2_EINT                      0x1000  /* IM_CS2_EINT */
#define WM8350_IM_USB_LIMIT_EINT                0x0400  /* IM_USB_LIMIT_EINT */
#define WM8350_IM_AUXADC_DATARDY_EINT           0x0100  /* IM_AUXADC_DATARDY_EINT */
#define WM8350_IM_AUXADC_DCOMP4_EINT            0x0080  /* IM_AUXADC_DCOMP4_EINT */
#define WM8350_IM_AUXADC_DCOMP3_EINT            0x0040  /* IM_AUXADC_DCOMP3_EINT */
#define WM8350_IM_AUXADC_DCOMP2_EINT            0x0020  /* IM_AUXADC_DCOMP2_EINT */
#define WM8350_IM_AUXADC_DCOMP1_EINT            0x0010  /* IM_AUXADC_DCOMP1_EINT */
#define WM8350_IM_SYS_HYST_COMP_FAIL_EINT       0x0008  /* IM_SYS_HYST_COMP_FAIL_EINT */
#define WM8350_IM_SYS_CHIP_GT115_EINT           0x0004  /* IM_SYS_CHIP_GT115_EINT */
#define WM8350_IM_SYS_CHIP_GT140_EINT           0x0002  /* IM_SYS_CHIP_GT140_EINT */
#define WM8350_IM_SYS_WDOG_TO_EINT              0x0001  /* IM_SYS_WDOG_TO_EINT */

/*
 * R35 (0x23) - Power Up Interrupt Status Mask
 */
#define WM8350_IM_PUTO_LDO4_EINT                0x0800  /* IM_PUTO_LDO4_EINT */
#define WM8350_IM_PUTO_LDO3_EINT                0x0400  /* IM_PUTO_LDO3_EINT */
#define WM8350_IM_PUTO_LDO2_EINT                0x0200  /* IM_PUTO_LDO2_EINT */
#define WM8350_IM_PUTO_LDO1_EINT                0x0100  /* IM_PUTO_LDO1_EINT */
#define WM8350_IM_PUTO_DC6_EINT                 0x0020  /* IM_PUTO_DC6_EINT */
#define WM8350_IM_PUTO_DC5_EINT                 0x0010  /* IM_PUTO_DC5_EINT */
#define WM8350_IM_PUTO_DC4_EINT                 0x0008  /* IM_PUTO_DC4_EINT */
#define WM8350_IM_PUTO_DC3_EINT                 0x0004  /* IM_PUTO_DC3_EINT */
#define WM8350_IM_PUTO_DC2_EINT                 0x0002  /* IM_PUTO_DC2_EINT */
#define WM8350_IM_PUTO_DC1_EINT                 0x0001  /* IM_PUTO_DC1_EINT */

/*
 * R36 (0x24) - Under Voltage Interrupt status Mask
 */
#define WM8350_IM_UV_LDO4_EINT                  0x0800  /* IM_UV_LDO4_EINT */
#define WM8350_IM_UV_LDO3_EINT                  0x0400  /* IM_UV_LDO3_EINT */
#define WM8350_IM_UV_LDO2_EINT                  0x0200  /* IM_UV_LDO2_EINT */
#define WM8350_IM_UV_LDO1_EINT                  0x0100  /* IM_UV_LDO1_EINT */
#define WM8350_IM_UV_DC6_EINT                   0x0020  /* IM_UV_DC6_EINT */
#define WM8350_IM_UV_DC5_EINT                   0x0010  /* IM_UV_DC5_EINT */
#define WM8350_IM_UV_DC4_EINT                   0x0008  /* IM_UV_DC4_EINT */
#define WM8350_IM_UV_DC3_EINT                   0x0004  /* IM_UV_DC3_EINT */
#define WM8350_IM_UV_DC2_EINT                   0x0002  /* IM_UV_DC2_EINT */
#define WM8350_IM_UV_DC1_EINT                   0x0001  /* IM_UV_DC1_EINT */

/*
 * R37 (0x25) - Over Current Interrupt status Mask
 */
#define WM8350_IM_OC_LS_EINT                    0x8000  /* IM_OC_LS_EINT */

/*
 * R38 (0x26) - GPIO Interrupt Status Mask
 */
#define WM8350_IM_GP12_EINT                     0x1000  /* IM_GP12_EINT */
#define WM8350_IM_GP11_EINT                     0x0800  /* IM_GP11_EINT */
#define WM8350_IM_GP10_EINT                     0x0400  /* IM_GP10_EINT */
#define WM8350_IM_GP9_EINT                      0x0200  /* IM_GP9_EINT */
#define WM8350_IM_GP8_EINT                      0x0100  /* IM_GP8_EINT */
#define WM8350_IM_GP7_EINT                      0x0080  /* IM_GP7_EINT */
#define WM8350_IM_GP6_EINT                      0x0040  /* IM_GP6_EINT */
#define WM8350_IM_GP5_EINT                      0x0020  /* IM_GP5_EINT */
#define WM8350_IM_GP4_EINT                      0x0010  /* IM_GP4_EINT */
#define WM8350_IM_GP3_EINT                      0x0008  /* IM_GP3_EINT */
#define WM8350_IM_GP2_EINT                      0x0004  /* IM_GP2_EINT */
#define WM8350_IM_GP1_EINT                      0x0002  /* IM_GP1_EINT */
#define WM8350_IM_GP0_EINT                      0x0001  /* IM_GP0_EINT */

/*
 * R39 (0x27) - Comparator Interrupt Status Mask
 */
#define WM8350_IM_EXT_USB_FB_EINT               0x8000  /* IM_EXT_USB_FB_EINT */
#define WM8350_IM_EXT_WALL_FB_EINT              0x4000  /* IM_EXT_WALL_FB_EINT */
#define WM8350_IM_EXT_BAT_FB_EINT               0x2000  /* IM_EXT_BAT_FB_EINT */
#define WM8350_IM_CODEC_JCK_DET_L_EINT          0x0800  /* IM_CODEC_JCK_DET_L_EINT */
#define WM8350_IM_CODEC_JCK_DET_R_EINT          0x0400  /* IM_CODEC_JCK_DET_R_EINT */
#define WM8350_IM_CODEC_MICSCD_EINT             0x0200  /* IM_CODEC_MICSCD_EINT */
#define WM8350_IM_CODEC_MICD_EINT               0x0100  /* IM_CODEC_MICD_EINT */
#define WM8350_IM_WKUP_OFF_STATE_EINT           0x0040  /* IM_WKUP_OFF_STATE_EINT */
#define WM8350_IM_WKUP_HIB_STATE_EINT           0x0020  /* IM_WKUP_HIB_STATE_EINT */
#define WM8350_IM_WKUP_CONV_FAULT_EINT          0x0010  /* IM_WKUP_CONV_FAULT_EINT */
#define WM8350_IM_WKUP_WDOG_RST_EINT            0x0008  /* IM_WKUP_WDOG_RST_EINT */
#define WM8350_IM_WKUP_GP_PWR_ON_EINT           0x0004  /* IM_WKUP_GP_PWR_ON_EINT */
#define WM8350_IM_WKUP_ONKEY_EINT               0x0002  /* IM_WKUP_ONKEY_EINT */
#define WM8350_IM_WKUP_GP_WAKEUP_EINT           0x0001  /* IM_WKUP_GP_WAKEUP_EINT */

/*
 * R40 (0x28) - Clock Control 1
 */
// lg #define WM8350_TOCLK_ENA                        0x8000  /* TOCLK_ENA */
#define WM8350_TOCLK_RATE                       0x4000  /* TOCLK_RATE */
#define WM8350_MCLK_SEL                         0x0800  /* MCLK_SEL */
#define WM8350_MCLK_DIV_MASK                    0x0100  /* MCLK_DIV - [10:8] */
#define WM8350_BCLK_DIV_MASK                    0x00F0  /* BCLK_DIV - [7:4] */
#define WM8350_OPCLK_DIV_MASK                   0x0007  /* OPCLK_DIV - [2:0] */

/*
 * R41 (0x29) - Clock Control 2
 */
#define WM8350_LRC_ADC_SEL                      0x8000  /* LRC_ADC_SEL */
#define WM8350_MCLK_DIR                         0x0001  /* MCLK_DIR */

/*
 * R42 (0x2A) - FLL Control 1
 */
// lg #define WM8350_FLL_ENA                          0x8000  /* FLL_ENA */
#define WM8350_FLL_DITHER_WIDTH_MASK            0x3000  /* FLL_DITHER_WIDTH - [13:12] */
#define WM8350_FLL_DITHER_HP                    0x0800  /* FLL_DITHER_HP */
#define WM8350_FLL_OUTDIV_MASK                  0x0700  /* FLL_OUTDIV - [10:8] */
#define WM8350_FLL_RSP_RATE_MASK                0x00F0  /* FLL_RSP_RATE - [7:4] */
#define WM8350_FLL_RATE_MASK                    0x0007  /* FLL_RATE - [2:0] */

/*
 * R43 (0x2B) - FLL Control 2
 */
#define WM8350_FLL_RATIO_MASK                   0xF800  /* FLL_RATIO - [15:11] */
#define WM8350_FLL_N_MASK                       0x03FF  /* FLL_N - [9:0] */

/*
 * R44 (0x2C) - FLL Control 3
 */
#define WM8350_FLL_K_MASK                       0xFFFF  /* FLL_K - [15:0] */

/*
 * R45 (0x2D) - FLL Control 4
 */
#define WM8350_FLL_FRAC                         0x0020  /* FLL_FRAC */
#define WM8350_FLL_SLOW_LOCK_REF                0x0010  /* FLL_SLOW_LOCK_REF */
#define WM8350_FLL_CLK_SRC_MASK                 0x0003  /* FLL_CLK_SRC - [1:0] */

/*
 * R48 (0x30) - DAC Control
 */
#define WM8350_DAC_MONO                         0x2000  /* DAC_MONO */
#define WM8350_AIF_LRCLKRATE                    0x1000  /* AIF_LRCLKRATE */
#define WM8350_DEEMP_MASK                       0x0030  /* DEEMP - [5:4] */
#define WM8350_DACL_DATINV                      0x0002  /* DACL_DATINV */
#define WM8350_DACR_DATINV                      0x0001  /* DACR_DATINV */

/*
 * R50 (0x32) - DAC Digital Volume L
 */
// lg #define WM8350_DACL_ENA                         0x8000  /* DACL_ENA */
#define WM8350_DAC_VU                           0x0100  /* DAC_VU */
#define WM8350_DACL_VOL_MASK                    0x00FF  /* DACL_VOL - [7:0] */

/*
 * R51 (0x33) - DAC Digital Volume R
 */
// lg #define WM8350_DACR_ENA                         0x8000  /* DACR_ENA */
#define WM8350_DAC_VU                           0x0100  /* DAC_VU */
#define WM8350_DACR_VOL_MASK                    0x00FF  /* DACR_VOL - [7:0] */

/*
 * R53 (0x35) - DAC LR Rate
 */
#define WM8350_DACLRC_ENA                       0x0800  /* DACLRC_ENA */
#define WM8350_DACLRC_RATE_MASK                 0x07FF  /* DACLRC_RATE - [10:0] */

/*
 * R54 (0x36) - DAC Clock Control
 */
#define WM8350_DACCLK_POL                       0x0010  /* DACCLK_POL */
#define WM8350_DAC_CLKDIV_MASK                  0x0007  /* DAC_CLKDIV - [2:0] */

/*
 * R58 (0x3A) - DAC Mute
 */
#define WM8350_DAC_MUTE_ENA                         0x4000  /* DAC_MUTE */

/*
 * R59 (0x3B) - DAC Mute Volume
 */
#define WM8350_DAC_MUTEMODE                     0x4000  /* DAC_MUTEMODE */
#define WM8350_DAC_MUTERATE                     0x2000  /* DAC_MUTERATE */
#define WM8350_DAC_SB_FILT                      0x1000  /* DAC_SB_FILT */

/*
 * R60 (0x3C) - DAC Side
 */
#define WM8350_ADC_TO_DACL_MASK                 0x3000  /* ADC_TO_DACL - [13:12] */
#define WM8350_ADC_TO_DACR_MASK                 0x0C00  /* ADC_TO_DACR - [11:10] */

/*
 * R64 (0x40) - ADC Control
 */
// lg #define WM8350_ADC_HPF_ENA                      0x8000  /* ADC_HPF_ENA */
#define WM8350_ADC_HPF_CUT_MASK                 0x0300  /* ADC_HPF_CUT - [9:8] */
#define WM8350_ADCL_DATINV                      0x0002  /* ADCL_DATINV */
#define WM8350_ADCR_DATINV                      0x0001  /* ADCR_DATINV */

/*
 * R66 (0x42) - ADC Digital Volume L
 */
// lg #define WM8350_ADCL_ENA                         0x8000  /* ADCL_ENA */
#define WM8350_ADC_VU                           0x0100  /* ADC_VU */
#define WM8350_ADCL_VOL_MASK                    0x00FF  /* ADCL_VOL - [7:0] */

/*
 * R67 (0x43) - ADC Digital Volume R
 */
// lg #define WM8350_ADCR_ENA                         0x8000  /* ADCR_ENA */
#define WM8350_ADC_VU                           0x0100  /* ADC_VU */
#define WM8350_ADCR_VOL_MASK                    0x00FF  /* ADCR_VOL - [7:0] */

/*
 * R68 (0x44) - ADC Divider
 */
#define WM8350_ADCL_DAC_SVOL_MASK               0x0F00  /* ADCL_DAC_SVOL - [11:8] */
#define WM8350_ADCR_DAC_SVOL_MASK               0x00F0  /* ADCR_DAC_SVOL - [7:4] */
#define WM8350_ADCCLK_POL                       0x0008  /* ADCCLK_POL */
#define WM8350_ADC_CLKDIV_MASK                  0x0007  /* ADC_CLKDIV - [2:0] */

/*
 * R70 (0x46) - ADC LR Rate
 */
#define WM8350_ADCLRC_ENA                       0x0800  /* ADCLRC_ENA */
#define WM8350_ADCLRC_RATE_MASK                 0x07FF  /* ADCLRC_RATE - [10:0] */

/*
 * R72 (0x48) - Input Control
 */
#define WM8350_IN2R_ENA                         0x0400  /* IN2R_ENA */
#define WM8350_IN1RN_ENA                        0x0200  /* IN1RN_ENA */
#define WM8350_IN1RP_ENA                        0x0100  /* IN1RP_ENA */
#define WM8350_IN2L_ENA                         0x0004  /* IN2L_ENA */
#define WM8350_IN1LN_ENA                        0x0002  /* IN1LN_ENA */
#define WM8350_IN1LP_ENA                        0x0001  /* IN1LP_ENA */

/*
 * R73 (0x49) - IN3 Input Control
 */
// lg #define WM8350_IN3R_ENA                         0x8000  /* IN3R_ENA */
#define WM8350_IN3R_SHORT                       0x4000  /* IN3R_SHORT */
// lg #define WM8350_IN3L_ENA                         0x0080  /* IN3L_ENA */
#define WM8350_IN3L_SHORT                       0x0040  /* IN3L_SHORT */

/*
 * R74 (0x4A) - Mic Bias Control
 */
// lg #define WM8350_MICBEN                           0x8000  /* MICBEN */
#define WM8350_MICBSEL                          0x4000  /* MICBSEL */
// lg #define WM8350_MIC_DET_ENA                      0x0080  /* MIC_DET_ENA */
#define WM8350_MCDTHR_MASK                      0x001C  /* MCDTHR - [4:2] */
#define WM8350_MCDSCTHR_MASK                    0x0003  /* MCDSCTHR - [1:0] */

/*
 * R76 (0x4C) - Output Control
 */
#define WM8350_OUT4_VROI                        0x0800  /* OUT4_VROI */
#define WM8350_OUT3_VROI                        0x0400  /* OUT3_VROI */
#define WM8350_OUT2_VROI                        0x0200  /* OUT2_VROI */
#define WM8350_OUT1_VROI                        0x0100  /* OUT1_VROI */
// lg #define WM8350_OUTPUT_DRAIN_EN                  0x0010  /* OUTPUT_DRAIN_EN */
#define WM8350_OUT2_FB                          0x0004  /* OUT2_FB */
#define WM8350_OUT1_FB                          0x0001  /* OUT1_FB */

/*
 * R77 (0x4D) - Jack Detect
 */
#define WM8350_JDL_ENA                          0x8000  /* JDL_ENA */
#define WM8350_JDR_ENA                          0x4000  /* JDR_ENA */

/*
 * R78 (0x4E) - Anti Pop Control
 */
#define WM8350_ANTI_POP_MASK                    0x0300  /* ANTI_POP - [9:8] */
#define WM8350_DIS_OP_LN4_MASK                  0x00C0  /* DIS_OP_LN4 - [7:6] */
#define WM8350_DIS_OP_LN3_MASK                  0x0030  /* DIS_OP_LN3 - [5:4] */
#define WM8350_DIS_OP_OUT2_MASK                 0x000C  /* DIS_OP_OUT2 - [3:2] */
#define WM8350_DIS_OP_OUT1_MASK                 0x0003  /* DIS_OP_OUT1 - [1:0] */

/*
 * R80 (0x50) - Left Input Volume
 */
// lg #define WM8350_INL_ENA                          0x8000  /* INL_ENA */
#define WM8350_INL_MUTE                         0x4000  /* INL_MUTE */
#define WM8350_INL_ZC                           0x2000  /* INL_ZC */
#define WM8350_IN_VU                            0x0100  /* IN_VU */
#define WM8350_INL_VOL_MASK                     0x00FC  /* INL_VOL - [7:2] */

/*
 * R81 (0x51) - Right Input Volume
 */
// lg #define WM8350_INR_ENA                          0x8000  /* INR_ENA */
#define WM8350_INR_MUTE                         0x4000  /* INR_MUTE */
#define WM8350_INR_ZC                           0x2000  /* INR_ZC */
#define WM8350_IN_VU                            0x0100  /* IN_VU */
#define WM8350_INR_VOL_MASK                     0x00FC  /* INR_VOL - [7:2] */

/*
 * R88 (0x58) - Left Mixer Control
 */
// lg #define WM8350_MIXOUTL_ENA                      0x8000  /* MIXOUTL_ENA */
#define WM8350_DACR_TO_MIXOUTL                  0x1000  /* DACR_TO_MIXOUTL */
#define WM8350_DACL_TO_MIXOUTL                  0x0800  /* DACL_TO_MIXOUTL */
#define WM8350_IN3L_TO_MIXOUTL                  0x0004  /* IN3L_TO_MIXOUTL */
#define WM8350_INR_TO_MIXOUTL                   0x0002  /* INR_TO_MIXOUTL */
#define WM8350_INL_TO_MIXOUTL                   0x0001  /* INL_TO_MIXOUTL */

/*
 * R89 (0x59) - Right Mixer Control
 */
// lg #define WM8350_MIXOUTR_ENA                      0x8000  /* MIXOUTR_ENA */
#define WM8350_DACR_TO_MIXOUTR                  0x1000  /* DACR_TO_MIXOUTR */
#define WM8350_DACL_TO_MIXOUTR                  0x0800  /* DACL_TO_MIXOUTR */
#define WM8350_IN3R_TO_MIXOUTR                  0x0008  /* IN3R_TO_MIXOUTR */
#define WM8350_INR_TO_MIXOUTR                   0x0002  /* INR_TO_MIXOUTR */
#define WM8350_INL_TO_MIXOUTR                   0x0001  /* INL_TO_MIXOUTR */

/*
 * R92 (0x5C) - OUT3 Mixer Control
 */
// lg #define WM8350_OUT3_ENA                         0x8000  /* OUT3_ENA */
#define WM8350_DACL_TO_OUT3                     0x0800  /* DACL_TO_OUT3 */
#define WM8350_MIXINL_TO_OUT3                   0x0100  /* MIXINL_TO_OUT3 */
#define WM8350_OUT4_TO_OUT3                     0x0008  /* OUT4_TO_OUT3 */
#define WM8350_MIXOUTL_TO_OUT3                  0x0001  /* MIXOUTL_TO_OUT3 */

/*
 * R93 (0x5D) - OUT4 Mixer Control
 */
// lg #define WM8350_OUT4_ENA                         0x8000  /* OUT4_ENA */
#define WM8350_DACR_TO_OUT4                     0x1000  /* DACR_TO_OUT4 */
#define WM8350_DACL_TO_OUT4                     0x0800  /* DACL_TO_OUT4 */
#define WM8350_OUT4_ATTN                        0x0400  /* OUT4_ATTN */
#define WM8350_MIXINR_TO_OUT4                   0x0200  /* MIXINR_TO_OUT4 */
#define WM8350_OUT3_TO_OUT4                     0x0004  /* OUT3_TO_OUT4 */
#define WM8350_MIXOUTR_TO_OUT4                  0x0002  /* MIXOUTR_TO_OUT4 */
#define WM8350_MIXOUTL_TO_OUT4                  0x0001  /* MIXOUTL_TO_OUT4 */

/*
 * R96 (0x60) - Output Left Mixer Volume
 */
#define WM8350_IN3L_MIXOUTL_VOL_MASK            0x0E00  /* IN3L_MIXOUTL_VOL - [11:9] */
#define WM8350_IN3L_MIXOUTL_VOL_SHIFT                9  /* IN3L_MIXOUTL_VOL - [11:9] */
#define WM8350_INR_MIXOUTL_VOL_MASK             0x00E0  /* INR_MIXOUTL_VOL - [7:5] */
#define WM8350_INR_MIXOUTL_VOL_SHIFT                 5  /* INR_MIXOUTL_VOL - [7:5] */
#define WM8350_INL_MIXOUTL_VOL_MASK             0x000E  /* INL_MIXOUTL_VOL - [3:1] */
#define WM8350_INL_MIXOUTL_VOL_SHIFT                 1  /* INL_MIXOUTL_VOL - [3:1] */

/* Bit values for R96 (0x60) */
#define WM8350_IN3L_MIXOUTL_VOL_OFF                  0  /* 0 = disabled */
#define WM8350_IN3L_MIXOUTL_VOL_M12DB                1  /* 1 = -12dBFS */
#define WM8350_IN3L_MIXOUTL_VOL_M9DB                 2  /* 2 = -9dBFS */
#define WM8350_IN3L_MIXOUTL_VOL_M6DB                 3  /* 3 = -6dBFS */
#define WM8350_IN3L_MIXOUTL_VOL_M3DB                 4  /* 4 = -3dBFS */
#define WM8350_IN3L_MIXOUTL_VOL_0DB                  5  /* 5 = 0dBFS */
#define WM8350_IN3L_MIXOUTL_VOL_3DB                  6  /* 6 = 3dBFS */
#define WM8350_IN3L_MIXOUTL_VOL_6DB                  7  /* 7 = 6dBFS */

#define WM8350_INR_MIXOUTL_VOL_OFF                   0  /* 0 = disabled */
#define WM8350_INR_MIXOUTL_VOL_M12DB                  1  /* 1 = -12dBFS */
#define WM8350_INR_MIXOUTL_VOL_M9DB                  2  /* 2 = -9dBFS */
#define WM8350_INR_MIXOUTL_VOL_M6DB                  3  /* 3 = -6dBFS */
#define WM8350_INR_MIXOUTL_VOL_M3DB                  4  /* 4 = -3dBFS */
#define WM8350_INR_MIXOUTL_VOL_0DB                   5  /* 5 = 0dBFS */
#define WM8350_INR_MIXOUTL_VOL_3DB                   6  /* 6 = 3dBFS */
#define WM8350_INR_MIXOUTL_VOL_6DB                   7  /* 7 = 6dBFS */

#define WM8350_INL_MIXOUTL_VOL_OFF                   0  /* 0 = disabled */
#define WM8350_INL_MIXOUTL_VOL_M12DB                 1  /* 1 = -12dBFS */
#define WM8350_INL_MIXOUTL_VOL_M9DB                  2  /* 2 = -9dBFS */
#define WM8350_INL_MIXOUTL_VOL_M6DB                  3  /* 3 = -6dBFS */
#define WM8350_INL_MIXOUTL_VOL_M3DB                  4  /* 4 = -3dBFS */
#define WM8350_INL_MIXOUTL_VOL_0DB                   5  /* 5 = 0dBFS */
#define WM8350_INL_MIXOUTL_VOL_3DB                   6  /* 6 = 3dBFS */
#define WM8350_INL_MIXOUTL_VOL_6DB                   7  /* 7 = 6dBFS */

/*
 * R97 (0x61) - Output Right Mixer Volume
 */
#define WM8350_IN3R_MIXOUTR_VOL_MASK            0xE000  /* IN3R_MIXOUTR_VOL - [15:13] */
#define WM8350_IN3R_MIXOUTR_VOL_SHIFT               13  /* IN3R_MIXOUTR_VOL - [11:9] */
#define WM8350_INR_MIXOUTR_VOL_MASK             0x00E0  /* INR_MIXOUTR_VOL - [7:5] */
#define WM8350_INR_MIXOUTR_VOL_SHIFT                 5  /* INR_MIXOUTR_VOL - [7:5] */
#define WM8350_INL_MIXOUTR_VOL_MASK             0x000E  /* INL_MIXOUTR_VOL - [3:1] */
#define WM8350_INL_MIXOUTR_VOL_SHIFT                 1  /* INL_MIXOUTR_VOL - [3:1] */

/* Bit values for R96 (0x60) */
#define WM8350_IN3R_MIXOUTR_VOL_OFF                  0  /* 0 = disabled */
#define WM8350_IN3R_MIXOUTR_VOL_M12DB                1  /* 1 = -12dBFS */
#define WM8350_IN3R_MIXOUTR_VOL_M9DB                 2  /* 2 = -9dBFS */
#define WM8350_IN3R_MIXOUTR_VOL_M6DB                 3  /* 3 = -6dBFS */
#define WM8350_IN3R_MIXOUTR_VOL_M3DB                 4  /* 4 = -3dBFS */
#define WM8350_IN3R_MIXOUTR_VOL_0DB                  5  /* 5 = 0dBFS */
#define WM8350_IN3R_MIXOUTR_VOL_3DB                  6  /* 6 = 3dBFS */
#define WM8350_IN3R_MIXOUTR_VOL_6DB                  7  /* 7 = 6dBFS */

#define WM8350_INR_MIXOUTR_VOL_OFF                   0  /* 0 = disabled */
#define WM8350_INR_MIXOUTR_VOL_M12DB                  1  /* 1 = -12dBFS */
#define WM8350_INR_MIXOUTR_VOL_M9DB                  2  /* 2 = -9dBFS */
#define WM8350_INR_MIXOUTR_VOL_M6DB                  3  /* 3 = -6dBFS */
#define WM8350_INR_MIXOUTR_VOL_M3DB                  4  /* 4 = -3dBFS */
#define WM8350_INR_MIXOUTR_VOL_0DB                   5  /* 5 = 0dBFS */
#define WM8350_INR_MIXOUTR_VOL_3DB                   6  /* 6 = 3dBFS */
#define WM8350_INR_MIXOUTR_VOL_6DB                   7  /* 7 = 6dBFS */

#define WM8350_INL_MIXOUTR_VOL_OFF                   0  /* 0 = disabled */
#define WM8350_INL_MIXOUTR_VOL_M12DB                 1  /* 1 = -12dBFS */
#define WM8350_INL_MIXOUTR_VOL_M9DB                  2  /* 2 = -9dBFS */
#define WM8350_INL_MIXOUTR_VOL_M6DB                  3  /* 3 = -6dBFS */
#define WM8350_INL_MIXOUTR_VOL_M3DB                  4  /* 4 = -3dBFS */
#define WM8350_INL_MIXOUTR_VOL_0DB                   5  /* 5 = 0dBFS */
#define WM8350_INL_MIXOUTR_VOL_3DB                   6  /* 6 = 3dBFS */
#define WM8350_INL_MIXOUTR_VOL_6DB                   7  /* 7 = 6dBFS */

/*
 * R98 (0x62) - Input Mixer Volume L
 */
#define WM8350_IN3L_MIXINL_VOL_MASK             0x0E00  /* IN3L_MIXINL_VOL - [11:9] */
#define WM8350_IN2L_MIXINL_VOL_MASK             0x000E  /* IN2L_MIXINL_VOL - [3:1] */
#define WM8350_INL_MIXINL_VOL                   0x0001  /* INL_MIXINL_VOL */

/*
 * R99 (0x63) - Input Mixer Volume R
 */
#define WM8350_IN3R_MIXINR_VOL_MASK             0xE000  /* IN3R_MIXINR_VOL - [15:13] */
#define WM8350_IN2R_MIXINR_VOL_MASK             0x00E0  /* IN2R_MIXINR_VOL - [7:5] */
#define WM8350_INR_MIXINR_VOL                   0x0001  /* INR_MIXINR_VOL */

/*
 * R100 (0x64) - Input Mixer Volume
 */
#define WM8350_OUT4_MIXIN_DST                   0x8000  /* OUT4_MIXIN_DST */
#define WM8350_OUT4_MIXIN_VOL_MASK              0x000E  /* OUT4_MIXIN_VOL - [3:1] */

/*
 * R104 (0x68) - LOUT1 Volume
 */
// lg // lg #define WM8350_OUT1L_ENA                        0x8000  /* OUT1L_ENA */
#define WM8350_OUT1L_MUTE                       0x4000  /* OUT1L_MUTE */
#define WM8350_OUT1L_ZC                         0x2000  /* OUT1L_ZC */
#define WM8350_OUT1_VU                          0x0100  /* OUT1_VU */
#define WM8350_OUT1L_VOL_MASK                   0x00FC  /* OUT1L_VOL - [7:2] */
#define WM8350_OUT1L_VOL_SHIFT                       2  /* OUT1L_VOL - [7:2] */

/* Bit values for R104 (0x68) */
#define WM8350_OUT1L_VOL( _dB )                 ((_dB) + 57)    /* 0 = -57dB, 1dB steps */
#define WM8350_OUT1L_DB( _vol )                 ((_vol) - 57)   /* 0 = -57dB, 1dB steps */

/*
 * R105 (0x69) - ROUT1 Volume
 */
// lg #define WM8350_OUT1R_ENA                        0x8000  /* OUT1R_ENA */
#define WM8350_OUT1R_MUTE                       0x4000  /* OUT1R_MUTE */
#define WM8350_OUT1R_ZC                         0x2000  /* OUT1R_ZC */
#define WM8350_OUT1_VU                          0x0100  /* OUT1_VU */
#define WM8350_OUT1R_VOL_MASK                   0x00FC  /* OUT1R_VOL - [7:2] */
#define WM8350_OUT1R_VOL_SHIFT                       2  /* OUT1R_VOL - [7:2] */

/* Bit values for R105 (0x69) */
#define WM8350_OUT1R_VOL( _dB )                 ((_dB) + 57)    /* 0 = -57dB, 1dB steps */
#define WM8350_OUT1R_DB( _vol )                 ((_vol) - 57)   /* 0 = -57dB, 1dB steps */

/*
 * R106 (0x6A) - LOUT2 Volume
 */
// lg #define WM8350_OUT2L_ENA                        0x8000  /* OUT2L_ENA */
#define WM8350_OUT2L_MUTE                       0x4000  /* OUT2L_MUTE */
#define WM8350_OUT2L_ZC                         0x2000  /* OUT2L_ZC */
#define WM8350_OUT2_VU                          0x0100  /* OUT2_VU */
#define WM8350_OUT2L_VOL_MASK                   0x00FC  /* OUT2L_VOL - [7:2] */

/*
 * R107 (0x6B) - ROUT2 Volume
 */
// lg #define WM8350_OUT2R_ENA                        0x8000  /* OUT2R_ENA */
#define WM8350_OUT2R_MUTE                       0x4000  /* OUT2R_MUTE */
#define WM8350_OUT2R_ZC                         0x2000  /* OUT2R_ZC */
#define WM8350_OUT2R_INV                        0x0400  /* OUT2R_INV */
#define WM8350_OUT2R_INV_MUTE                   0x0200  /* OUT2R_INV_MUTE */
#define WM8350_OUT2_VU                          0x0100  /* OUT2_VU */
#define WM8350_OUT2R_VOL_MASK                   0x00FC  /* OUT2R_VOL - [7:2] */

/*
 * R111 (0x6F) - BEEP Volume
 */
// lg #define WM8350_IN3R_TO_OUT2R                    0x8000  /* IN3R_TO_OUT2R */
#define WM8350_IN3R_OUT2R_VOL_MASK              0x00E0  /* IN3R_OUT2R_VOL - [7:5] */

/*
 * R112 (0x70) - AI Formating
 */
#define WM8350_AIF_BCLK_INV                     0x8000  /* AIF_BCLK_INV */
#define WM8350_AIF_TRI                          0x2000  /* AIF_TRI */
#define WM8350_AIF_LRCLK_INV                    0x1000  /* AIF_LRCLK_INV */
#define WM8350_AIF_WL_MASK                      0x0C00  /* AIF_WL - [11:10] */
#define WM8350_AIF_FMT_MASK                     0x0300  /* AIF_FMT - [9:8] */

/*
 * R113 (0x71) - ADC DAC COMP
 */
#define WM8350_DAC_COMP                         0x0080  /* DAC_COMP */
#define WM8350_DAC_COMPMODE                     0x0040  /* DAC_COMPMODE */
#define WM8350_ADC_COMP                         0x0020  /* ADC_COMP */
#define WM8350_ADC_COMPMODE                     0x0010  /* ADC_COMPMODE */
#define WM8350_LOOPBACK                         0x0001  /* LOOPBACK */

/*
 * R114 (0x72) - AI ADC Control
 */
#define WM8350_AIFADC_PD                        0x0080  /* AIFADC_PD */
#define WM8350_AIFADCL_SRC                      0x0040  /* AIFADCL_SRC */
#define WM8350_AIFADCR_SRC                      0x0020  /* AIFADCR_SRC */
#define WM8350_AIFADC_TDM_CHAN                  0x0010  /* AIFADC_TDM_CHAN */
#define WM8350_AIFADC_TDM                       0x0008  /* AIFADC_TDM */

/*
 * R115 (0x73) - AI DAC Control
 */
#define WM8350_BCLK_MSTR                        0x4000  /* BCLK_MSTR */
#define WM8350_AIFDAC_PD                        0x0080  /* AIFDAC_PD */
#define WM8350_DACL_SRC                         0x0040  /* DACL_SRC */
#define WM8350_DACR_SRC                         0x0020  /* DACR_SRC */
#define WM8350_AIFDAC_TDM_CHAN                  0x0010  /* AIFDAC_TDM_CHAN */
#define WM8350_AIFDAC_TDM                       0x0008  /* AIFDAC_TDM */
#define WM8350_DAC_BOOST_MASK                   0x0003  /* DAC_BOOST - [1:0] */

/*
 * R116 (0x74) - AIF Test
 */
#define WM8350_CODEC_BYP                        0x4000  /* CODEC_BYP */
#define WM8350_AIFADC_WR_TST                    0x2000  /* AIFADC_WR_TST */
#define WM8350_AIFADC_RD_TST                    0x1000  /* AIFADC_RD_TST */
#define WM8350_AIFDAC_WR_TST                    0x0800  /* AIFDAC_WR_TST */
#define WM8350_AIFDAC_RD_TST                    0x0400  /* AIFDAC_RD_TST */
#define WM8350_AIFADC_ASYN                      0x0020  /* AIFADC_ASYN */
#define WM8350_AIFDAC_ASYN                      0x0010  /* AIFDAC_ASYN */

/*
 * R128 (0x80) - GPIO Debounce
 */
#define WM8350_GP12_DB                          0x1000  /* GP12_DB */
#define WM8350_GP11_DB                          0x0800  /* GP11_DB */
#define WM8350_GP10_DB                          0x0400  /* GP10_DB */
#define WM8350_GP9_DB                           0x0200  /* GP9_DB */
#define WM8350_GP8_DB                           0x0100  /* GP8_DB */
#define WM8350_GP7_DB                           0x0080  /* GP7_DB */
#define WM8350_GP6_DB                           0x0040  /* GP6_DB */
#define WM8350_GP5_DB                           0x0020  /* GP5_DB */
#define WM8350_GP4_DB                           0x0010  /* GP4_DB */
#define WM8350_GP3_DB                           0x0008  /* GP3_DB */
#define WM8350_GP2_DB                           0x0004  /* GP2_DB */
#define WM8350_GP1_DB                           0x0002  /* GP1_DB */
#define WM8350_GP0_DB                           0x0001  /* GP0_DB */

/*
 * R129 (0x81) - GPIO Pin pull up Control
 */
#define WM8350_GP12_PU                          0x1000  /* GP12_PU */
#define WM8350_GP11_PU                          0x0800  /* GP11_PU */
#define WM8350_GP10_PU                          0x0400  /* GP10_PU */
#define WM8350_GP9_PU                           0x0200  /* GP9_PU */
#define WM8350_GP8_PU                           0x0100  /* GP8_PU */
#define WM8350_GP7_PU                           0x0080  /* GP7_PU */
#define WM8350_GP6_PU                           0x0040  /* GP6_PU */
#define WM8350_GP5_PU                           0x0020  /* GP5_PU */
#define WM8350_GP4_PU                           0x0010  /* GP4_PU */
#define WM8350_GP3_PU                           0x0008  /* GP3_PU */
#define WM8350_GP2_PU                           0x0004  /* GP2_PU */
#define WM8350_GP1_PU                           0x0002  /* GP1_PU */
#define WM8350_GP0_PU                           0x0001  /* GP0_PU */

/*
 * R130 (0x82) - GPIO Pull down Control
 */
#define WM8350_GP12_PD                          0x1000  /* GP12_PD */
#define WM8350_GP11_PD                          0x0800  /* GP11_PD */
#define WM8350_GP10_PD                          0x0400  /* GP10_PD */
#define WM8350_GP9_PD                           0x0200  /* GP9_PD */
#define WM8350_GP8_PD                           0x0100  /* GP8_PD */
#define WM8350_GP7_PD                           0x0080  /* GP7_PD */
#define WM8350_GP6_PD                           0x0040  /* GP6_PD */
#define WM8350_GP5_PD                           0x0020  /* GP5_PD */
#define WM8350_GP4_PD                           0x0010  /* GP4_PD */
#define WM8350_GP3_PD                           0x0008  /* GP3_PD */
#define WM8350_GP2_PD                           0x0004  /* GP2_PD */
#define WM8350_GP1_PD                           0x0002  /* GP1_PD */
#define WM8350_GP0_PD                           0x0001  /* GP0_PD */

/*
 * R131 (0x83) - GPIO Interrupt Mode
 */
#define WM8350_GP12_INTMODE                     0x1000  /* GP12_INTMODE */
#define WM8350_GP11_INTMODE                     0x0800  /* GP11_INTMODE */
#define WM8350_GP10_INTMODE                     0x0400  /* GP10_INTMODE */
#define WM8350_GP9_INTMODE                      0x0200  /* GP9_INTMODE */
#define WM8350_GP8_INTMODE                      0x0100  /* GP8_INTMODE */
#define WM8350_GP7_INTMODE                      0x0080  /* GP7_INTMODE */
#define WM8350_GP6_INTMODE                      0x0040  /* GP6_INTMODE */
#define WM8350_GP5_INTMODE                      0x0020  /* GP5_INTMODE */
#define WM8350_GP4_INTMODE                      0x0010  /* GP4_INTMODE */
#define WM8350_GP3_INTMODE                      0x0008  /* GP3_INTMODE */
#define WM8350_GP2_INTMODE                      0x0004  /* GP2_INTMODE */
#define WM8350_GP1_INTMODE                      0x0002  /* GP1_INTMODE */
#define WM8350_GP0_INTMODE                      0x0001  /* GP0_INTMODE */

/*
 * R133 (0x85) - GPIO Control
 */
#define WM8350_GP_DBTIME_MASK                   0x00C0  /* GP_DBTIME - [7:6] */

/*
 * R134 (0x86) - GPIO Configuration (i/o)
 */
#define WM8350_GP12_DIR                         0x1000  /* GP12_DIR */
#define WM8350_GP11_DIR                         0x0800  /* GP11_DIR */
#define WM8350_GP10_DIR                         0x0400  /* GP10_DIR */
#define WM8350_GP9_DIR                          0x0200  /* GP9_DIR */
#define WM8350_GP8_DIR                          0x0100  /* GP8_DIR */
#define WM8350_GP7_DIR                          0x0080  /* GP7_DIR */
#define WM8350_GP6_DIR                          0x0040  /* GP6_DIR */
#define WM8350_GP5_DIR                          0x0020  /* GP5_DIR */
#define WM8350_GP4_DIR                          0x0010  /* GP4_DIR */
#define WM8350_GP3_DIR                          0x0008  /* GP3_DIR */
#define WM8350_GP2_DIR                          0x0004  /* GP2_DIR */
#define WM8350_GP1_DIR                          0x0002  /* GP1_DIR */
#define WM8350_GP0_DIR                          0x0001  /* GP0_DIR */

/*
 * R135 (0x87) - GPIO Pin Polarity / Type
 */
#define WM8350_GP12_CFG                         0x1000  /* GP12_CFG */
#define WM8350_GP11_CFG                         0x0800  /* GP11_CFG */
#define WM8350_GP10_CFG                         0x0400  /* GP10_CFG */
#define WM8350_GP9_CFG                          0x0200  /* GP9_CFG */
#define WM8350_GP8_CFG                          0x0100  /* GP8_CFG */
#define WM8350_GP7_CFG                          0x0080  /* GP7_CFG */
#define WM8350_GP6_CFG                          0x0040  /* GP6_CFG */
#define WM8350_GP5_CFG                          0x0020  /* GP5_CFG */
#define WM8350_GP4_CFG                          0x0010  /* GP4_CFG */
#define WM8350_GP3_CFG                          0x0008  /* GP3_CFG */
#define WM8350_GP2_CFG                          0x0004  /* GP2_CFG */
#define WM8350_GP1_CFG                          0x0002  /* GP1_CFG */
#define WM8350_GP0_CFG                          0x0001  /* GP0_CFG */

/*
 * R140 (0x8C) - GPIO Function Select 1
 */
#define WM8350_GP3_FN_MASK                      0xF000  /* GP3_FN - [15:12] */
#define WM8350_GP2_FN_MASK                      0x0F00  /* GP2_FN - [11:8] */
#define WM8350_GP1_FN_MASK                      0x00F0  /* GP1_FN - [7:4] */
#define WM8350_GP0_FN_MASK                      0x000F  /* GP0_FN - [3:0] */

/*
 * R141 (0x8D) - GPIO Function Select 2
 */
#define WM8350_GP7_FN_MASK                      0xF000  /* GP7_FN - [15:12] */
#define WM8350_GP6_FN_MASK                      0x0F00  /* GP6_FN - [11:8] */
#define WM8350_GP5_FN_MASK                      0x00F0  /* GP5_FN - [7:4] */
#define WM8350_GP4_FN_MASK                      0x000F  /* GP4_FN - [3:0] */

/*
 * R142 (0x8E) - GPIO Function Select 3
 */
#define WM8350_GP11_FN_MASK                     0xF000  /* GP11_FN - [15:12] */
#define WM8350_GP10_FN_MASK                     0x0F00  /* GP10_FN - [11:8] */
#define WM8350_GP9_FN_MASK                      0x00F0  /* GP9_FN - [7:4] */
#define WM8350_GP8_FN_MASK                      0x000F  /* GP8_FN - [3:0] */

/*
 * R143 (0x8F) - GPIO Function Select 4
 */
#define WM8350_GP12_FN_MASK                     0x000F  /* GP12_FN - [3:0] */

/*
 * R144 (0x90) - Digitiser Control (1)
 */
// lg #define WM8350_AUXADC_ENA                       0x8000  /* AUXADC_ENA */
#define WM8350_AUXADC_CTC                       0x4000  /* AUXADC_CTC */
#define WM8350_AUXADC_POLL                      0x2000  /* AUXADC_POLL */
#define WM8350_AUXADC_HIB_MODE                  0x1000  /* AUXADC_HIB_MODE */
#define WM8350_AUXADC_SEL8                      0x0080  /* AUXADC_SEL8 */
#define WM8350_AUXADC_SEL7                      0x0040  /* AUXADC_SEL7 */
#define WM8350_AUXADC_SEL6                      0x0020  /* AUXADC_SEL6 */
#define WM8350_AUXADC_SEL5                      0x0010  /* AUXADC_SEL5 */
#define WM8350_AUXADC_SEL4                      0x0008  /* AUXADC_SEL4 */
#define WM8350_AUXADC_SEL3                      0x0004  /* AUXADC_SEL3 */
#define WM8350_AUXADC_SEL2                      0x0002  /* AUXADC_SEL2 */
#define WM8350_AUXADC_SEL1                      0x0001  /* AUXADC_SEL1 */

/*
 * R145 (0x91) - Digitiser Control (2)
 */
#define WM8350_AUXADC_MASKMODE_MASK             0x3000  /* AUXADC_MASKMODE - [13:12] */
#define WM8350_AUXADC_CRATE_MASK                0x0700  /* AUXADC_CRATE - [10:8] */
#define WM8350_AUXADC_CAL                       0x0004  /* AUXADC_CAL */
#define WM8350_AUX_RBMODE                       0x0002  /* AUX_RBMODE */
#define WM8350_AUXADC_WAIT                      0x0001  /* AUXADC_WAIT */

/*
 * R152 (0x98) - AUX1 Readback
 */
#define WM8350_AUXADC_SCALE1_MASK               0x6000  /* AUXADC_SCALE1 - [14:13] */
#define WM8350_AUXADC_REF1                      0x1000  /* AUXADC_REF1 */
#define WM8350_AUXADC_DATA1_MASK                0x0FFF  /* AUXADC_DATA1 - [11:0] */

/*
 * R153 (0x99) - AUX2 Readback
 */
#define WM8350_AUXADC_SCALE2_MASK               0x6000  /* AUXADC_SCALE2 - [14:13] */
#define WM8350_AUXADC_REF2                      0x1000  /* AUXADC_REF2 */
#define WM8350_AUXADC_DATA2_MASK                0x0FFF  /* AUXADC_DATA2 - [11:0] */

/*
 * R154 (0x9A) - AUX3 Readback
 */
#define WM8350_AUXADC_SCALE3_MASK               0x6000  /* AUXADC_SCALE3 - [14:13] */
#define WM8350_AUXADC_REF3                      0x1000  /* AUXADC_REF3 */
#define WM8350_AUXADC_DATA3_MASK                0x0FFF  /* AUXADC_DATA3 - [11:0] */

/*
 * R155 (0x9B) - AUX4 Readback
 */
#define WM8350_AUXADC_SCALE4_MASK               0x6000  /* AUXADC_SCALE4 - [14:13] */
#define WM8350_AUXADC_REF4                      0x1000  /* AUXADC_REF4 */
#define WM8350_AUXADC_DATA4_MASK                0x0FFF  /* AUXADC_DATA4 - [11:0] */

/*
 * R156 (0x9C) - USB Voltage Readback
 */
#define WM8350_AUXADC_DATA_USB_MASK             0x0FFF  /* AUXADC_DATA_USB - [11:0] */

/*
 * R157 (0x9D) - LINE Voltage Readback
 */
#define WM8350_AUXADC_DATA_LINE_MASK            0x0FFF  /* AUXADC_DATA_LINE - [11:0] */

/*
 * R158 (0x9E) - BATT Voltage Readback
 */
#define WM8350_AUXADC_DATA_BATT_MASK            0x0FFF  /* AUXADC_DATA_BATT - [11:0] */

/*
 * R159 (0x9F) - Chip Temp Readback
 */
#define WM8350_AUXADC_DATA_CHIPTEMP_MASK        0x0FFF  /* AUXADC_DATA_CHIPTEMP - [11:0] */

/*
 * R163 (0xA3) - Generic Comparator Control
 */
#define WM8350_DCMP4_ENA                        0x0008  /* DCMP4_ENA */
#define WM8350_DCMP3_ENA                        0x0004  /* DCMP3_ENA */
#define WM8350_DCMP2_ENA                        0x0002  /* DCMP2_ENA */
#define WM8350_DCMP1_ENA                        0x0001  /* DCMP1_ENA */

/*
 * R164 (0xA4) - Generic comparator 1
 */
#define WM8350_DCMP1_SRCSEL_MASK                0xE000  /* DCMP1_SRCSEL - [15:13] */
#define WM8350_DCMP1_GT                         0x1000  /* DCMP1_GT */
#define WM8350_DCMP1_THR_MASK                   0x0FFF  /* DCMP1_THR - [11:0] */

/*
 * R165 (0xA5) - Generic comparator 2
 */
#define WM8350_DCMP2_SRCSEL_MASK                0xE000  /* DCMP2_SRCSEL - [15:13] */
#define WM8350_DCMP2_GT                         0x1000  /* DCMP2_GT */
#define WM8350_DCMP2_THR_MASK                   0x0FFF  /* DCMP2_THR - [11:0] */

/*
 * R166 (0xA6) - Generic comparator 3
 */
#define WM8350_DCMP3_SRCSEL_MASK                0xE000  /* DCMP3_SRCSEL - [15:13] */
#define WM8350_DCMP3_GT                         0x1000  /* DCMP3_GT */
#define WM8350_DCMP3_THR_MASK                   0x0FFF  /* DCMP3_THR - [11:0] */

/*
 * R167 (0xA7) - Generic comparator 4
 */
#define WM8350_DCMP4_SRCSEL_MASK                0xE000  /* DCMP4_SRCSEL - [15:13] */
#define WM8350_DCMP4_GT                         0x1000  /* DCMP4_GT */
#define WM8350_DCMP4_THR_MASK                   0x0FFF  /* DCMP4_THR - [11:0] */

/*
 * R168 (0xA8) - Battery Charger Control 1
 */
#define WM8350_CHG_ENA_R168                     0x8000  /* CHG_ENA */
#define WM8350_CHG_THR                          0x2000  /* CHG_THR */
#define WM8350_CHG_EOC_SEL_MASK                 0x1C00  /* CHG_EOC_SEL - [12:10] */
#define WM8350_CHG_TRICKLE_TEMP_CHOKE           0x0200  /* CHG_TRICKLE_TEMP_CHOKE */
#define WM8350_CHG_TRICKLE_USB_CHOKE            0x0100  /* CHG_TRICKLE_USB_CHOKE */
#define WM8350_CHG_RECOVER_T                    0x0080  /* CHG_RECOVER_T */
#define WM8350_CHG_END_ACT                      0x0040  /* CHG_END_ACT */
#define WM8350_CHG_FAST                         0x0020  /* CHG_FAST */
#define WM8350_CHG_FAST_USB_THROTTLE            0x0010  /* CHG_FAST_USB_THROTTLE */
#define WM8350_CHG_NTC_MON                      0x0008  /* CHG_NTC_MON */
#define WM8350_CHG_BATT_HOT_MON                 0x0004  /* CHG_BATT_HOT_MON */
#define WM8350_CHG_BATT_COLD_MON                0x0002  /* CHG_BATT_COLD_MON */
#define WM8350_CHG_CHIP_TEMP_MON                0x0001  /* CHG_CHIP_TEMP_MON */

/*
 * R169 (0xA9) - Battery Charger Control 2
 */
#define WM8350_CHG_ACTIVE                       0x8000  /* CHG_ACTIVE */
#define WM8350_CHG_PAUSE                        0x4000  /* CHG_PAUSE */
#define WM8350_CHG_STS_MASK                     0x3000  /* CHG_STS - [13:12] */
#define WM8350_CHG_TIME_MASK                    0x0F00  /* CHG_TIME - [11:8] */
#define WM8350_CHG_MASK_WALL_FB                 0x0080  /* CHG_MASK_WALL_FB */
#define WM8350_CHG_TRICKLE_SEL                  0x0040  /* CHG_TRICKLE_SEL */
#define WM8350_CHG_VSEL_MASK                    0x0030  /* CHG_VSEL - [5:4] */
#define WM8350_CHG_ISEL_MASK                    0x000F  /* CHG_ISEL - [3:0] */
#define WM8350_CHG_STS_OFF                      0x0000
#define WM8350_CHG_STS_TRICKLE                  0x1000
#define WM8350_CHG_STS_FAST                     0x2000

/*
 * R170 (0xAA) - Battery Charger Control 3
 */
#define WM8350_CHG_THROTTLE_T_MASK              0x0060  /* CHG_THROTTLE_T - [6:5] */
#define WM8350_CHG_SMART                        0x0010  /* CHG_SMART */
#define WM8350_CHG_TIMER_ADJT_MASK              0x000F  /* CHG_TIMER_ADJT - [3:0] */

/*
 * R172 (0xAC) - Current Sink Driver A
 */
// lg #define WM8350_CS1_ENA                          0x8000  /* CS1_ENA */
#define WM8350_CS1_HIB_MODE                     0x1000  /* CS1_HIB_MODE */
#define WM8350_CS1_HIB_MODE_MASK                0x1000  /* CS1_HIB_MODE */
#define WM8350_CS1_HIB_MODE_SHIFT                   12  /* CS1_HIB_MODE */
#define WM8350_CS1_ISEL_MASK                    0x003F  /* CS1_ISEL - [5:0] */
#define WM8350_CS1_ISEL_SHIFT                        0  /* CS1_ISEL - [5:0] */

/* Bit values for R172 (0xAC) */
#define WM8350_CS1_HIB_MODE_DISABLE                  0  /* Disable current sink in hibernate */
#define WM8350_CS1_HIB_MODE_LEAVE                    1  /* Leave current sink as-is in hibernate */

#define WM8350_CS1_ISEL_220M                      0x3F  /* 220mA */

/*
 * R173 (0xAD) - CSA Flash control
 */
#define WM8350_CS1_FLASH_MODE                   0x8000  /* CS1_FLASH_MODE */
#define WM8350_CS1_TRIGSRC                      0x4000  /* CS1_TRIGSRC */
#define WM8350_CS1_DRIVE                        0x2000  /* CS1_DRIVE */
#define WM8350_CS1_FLASH_DUR_MASK               0x0300  /* CS1_FLASH_DUR - [9:8] */
#define WM8350_CS1_OFF_RAMP_MASK                0x0030  /* CS1_OFF_RAMP - [5:4] */
#define WM8350_CS1_ON_RAMP_MASK                 0x0003  /* CS1_ON_RAMP - [1:0] */

/*
 * R174 (0xAE) - Current Sink Driver B
 */
// lg #define WM8350_CS2_ENA                          0x8000  /* CS2_ENA */
#define WM8350_CS2_HIB_MODE                     0x1000  /* CS2_HIB_MODE */
#define WM8350_CS2_ISEL_MASK                    0x003F  /* CS2_ISEL - [5:0] */

/*
 * R175 (0xAF) - CSB Flash control
 */
#define WM8350_CS2_FLASH_MODE                   0x8000  /* CS2_FLASH_MODE */
#define WM8350_CS2_TRIGSRC                      0x4000  /* CS2_TRIGSRC */
#define WM8350_CS2_DRIVE                        0x2000  /* CS2_DRIVE */
#define WM8350_CS2_FLASH_DUR_MASK               0x0300  /* CS2_FLASH_DUR - [9:8] */
#define WM8350_CS2_OFF_RAMP_MASK                0x0030  /* CS2_OFF_RAMP - [5:4] */
#define WM8350_CS2_ON_RAMP_MASK                 0x0003  /* CS2_ON_RAMP - [1:0] */

/*
 * R176 (0xB0) - DCDC/LDO requested
 */
#define WM8350_LS_ENA                           0x8000  /* LS_ENA */
#define WM8350_LDO4_ENA                         0x0800  /* LDO4_ENA */
#define WM8350_LDO3_ENA                         0x0400  /* LDO3_ENA */
#define WM8350_LDO2_ENA                         0x0200  /* LDO2_ENA */
#define WM8350_LDO1_ENA                         0x0100  /* LDO1_ENA */
#define WM8350_DC6_ENA                          0x0020  /* DC6_ENA */
#define WM8350_DC5_ENA                          0x0010  /* DC5_ENA */
#define WM8350_DC4_ENA                          0x0008  /* DC4_ENA */
#define WM8350_DC3_ENA                          0x0004  /* DC3_ENA */
#define WM8350_DC2_ENA                          0x0002  /* DC2_ENA */
#define WM8350_DC1_ENA                          0x0001  /* DC1_ENA */

/*
 * R177 (0xB1) - DCDC Active options
 */
#define WM8350_PUTO_MASK                        0x3000  /* PUTO - [13:12] */
#define WM8350_PWRUP_DELAY_MASK                 0x0300  /* PWRUP_DELAY - [9:8] */
#define WM8350_DC6_ACTIVE                       0x0020  /* DC6_ACTIVE */
#define WM8350_DC4_ACTIVE                       0x0008  /* DC4_ACTIVE */
#define WM8350_DC3_ACTIVE                       0x0004  /* DC3_ACTIVE */
#define WM8350_DC1_ACTIVE                       0x0001  /* DC1_ACTIVE */

/*
 * R178 (0xB2) - DCDC Sleep options
 */
#define WM8350_DC6_SLEEP                        0x0020  /* DC6_SLEEP */
#define WM8350_DC4_SLEEP                        0x0008  /* DC4_SLEEP */
#define WM8350_DC3_SLEEP                        0x0004  /* DC3_SLEEP */
#define WM8350_DC1_SLEEP                        0x0001  /* DC1_SLEEP */

/*
 * R179 (0xB3) - Power-check comparator
 */
#define WM8350_PCCMP_ERRACT                     0x4000  /* PCCMP_ERRACT */
// lg #define WM8350_PCCOMP_HIB_MODE                  0x1000  /* PCCOMP_HIB_MODE */
#define WM8350_PCCMP_RAIL                       0x0100  /* PCCMP_RAIL */
#define WM8350_PCCMP_OFF_THR_MASK               0x0070  /* PCCMP_OFF_THR - [6:4] */
#define WM8350_PCCMP_ON_THR_MASK                0x0007  /* PCCMP_ON_THR - [2:0] */

/*
 * R180 (0xB4) - DCDC1 Control
 */
#define WM8350_DC1_OPFLT                        0x0400  /* DC1_OPFLT */
#define WM8350_DC1_VSEL_MASK                    0x007F  /* DC1_VSEL - [6:0] */
#define WM8350_DC1_VSEL_SHIFT                        0  /* DC1_VSEL - [6:0] */

/* Bit values for R180 (0xB4) */
#define WM8350_DC1_VSEL( _mV )                  (((_mV)-850)/25)
#define WM8350_DC1_VSEL_V( _val )               (((_val)*25)+850)

/*
 * R181 (0xB5) - DCDC1 Timeouts
 */
#define WM8350_DC1_ERRACT_MASK                  0xC000  /* DC1_ERRACT - [15:14] */
#define WM8350_DC1_ERRACT_SHIFT                     14  /* DC1_ERRACT - [15:14] */
#define WM8350_DC1_ENSLOT_MASK                  0x3C00  /* DC1_ENSLOT - [13:10] */
#define WM8350_DC1_ENSLOT_SHIFT                     10  /* DC1_ENSLOT - [13:10] */
#define WM8350_DC1_SDSLOT_MASK                  0x03C0  /* DC1_SDSLOT - [9:6] */
#define WM8350_DC1_UVTO_MASK                    0x0030  /* DC1_UVTO - [5:4] */
#define WM8350_DC1_SDSLOT_SHIFT                      6

/* Bit values for R181 (0xB5) */
#define WM8350_DC1_ERRACT_NONE                       0
#define WM8350_DC1_ERRACT_SHUTDOWN_CONV              1
#define WM8350_DC1_ERRACT_SHUTDOWN_SYS               2


/*
 * R182 (0xB6) - DCDC1 Low Power
 */
#define WM8350_DC1_HIB_MODE_MASK                0x7000  /* DC1_HIB_MODE - [14:12] */
#define WM8350_DC1_HIB_TRIG_MASK                0x0300  /* DC1_HIB_TRIG - [9:8] */
#define WM8350_DC1_VIMG_MASK                    0x007F  /* DC1_VIMG - [6:0] */

/*
 * R183 (0xB7) - DCDC2 Control
 */
#define WM8350_DC2_MODE                         0x4000  /* DC2_MODE */
#define WM8350_DC2_MODE_MASK                    0x4000  /* DC2_MODE */
#define WM8350_DC2_MODE_SHIFT                       14  /* DC2_MODE */
#define WM8350_DC2_HIB_MODE                     0x1000  /* DC2_HIB_MODE */
#define WM8350_DC2_HIB_MODE_MASK                0x1000  /* DC2_HIB_MODE */
#define WM8350_DC2_HIB_MODE_SHIFT                   12  /* DC2_HIB_MODE */
#define WM8350_DC2_HIB_TRIG_MASK                0x0300  /* DC2_HIB_TRIG - [9:8] */
#define WM8350_DC2_HIB_TRIG_SHIFT                    8  /* DC2_HIB_TRIG - [9:8] */
#define WM8350_DC2_ILIM                         0x0040  /* DC2_ILIM */
#define WM8350_DC2_ILIM_MASK                    0x0040  /* DC2_ILIM */
#define WM8350_DC2_ILIM_SHIFT                        6  /* DC2_ILIM */
#define WM8350_DC2_RMP_MASK                     0x0018  /* DC2_RMP - [4:3] */
#define WM8350_DC2_RMP_SHIFT                         3  /* DC2_RMP */
#define WM8350_DC2_FBSRC_MASK                   0x0003  /* DC2_FBSRC - [1:0] */
#define WM8350_DC2_FBSRC_SHIFT                       0  /* DC2_FBSRC - [1:0] */

/* Bit values for R183 (0xB7) */
#define WM8350_DC2_MODE_BOOST                        0
#define WM8350_DC2_MODE_SWITCH                       1

#define WM8350_DC2_HIB_MODE_ACTIVE                   0
#define WM8350_DC2_HIB_MODE_DISABLE                  1

#define WM8350_DC2_HIB_TRIG_NONE                     0
#define WM8350_DC2_HIB_TRIG_LPWR1                    1
#define WM8350_DC2_HIB_TRIG_LPWR2                    2
#define WM8350_DC2_HIB_TRIG_LPWR3                    3

#define WM8350_DC2_ILIM_HIGH                         0
#define WM8350_DC2_ILIM_LOW                          1

#define WM8350_DC2_RMP_30V                           0
#define WM8350_DC2_RMP_20V                           1
#define WM8350_DC2_RMP_10V                           2
#define WM8350_DC2_RMP_5V                            3

#define WM8350_DC2_FBSRC_FB2                         0
#define WM8350_DC2_FBSRC_ISINKA                      1
#define WM8350_DC2_FBSRC_ISINKB                      2
#define WM8350_DC2_FBSRC_USB                         3

/*
 * R184 (0xB8) - DCDC2 Timeouts
 */
#define WM8350_DC2_ERRACT_MASK                  0xC000  /* DC2_ERRACT - [15:14] */
#define WM8350_DC2_ERRACT_SHIFT                     14  /* DC2_ERRACT - [15:14] */
#define WM8350_DC2_ENSLOT_MASK                  0x3C00  /* DC2_ENSLOT - [13:10] */
#define WM8350_DC2_ENSLOT_SHIFT                     10  /* DC2_ENSLOT - [13:10] */
#define WM8350_DC2_SDSLOT_MASK                  0x03C0  /* DC2_SDSLOT - [9:6] */
#define WM8350_DC2_UVTO_MASK                    0x0030  /* DC2_UVTO - [5:4] */

/* Bit values for R184 (0xB8) */
#define WM8350_DC2_ERRACT_NONE                       0
#define WM8350_DC2_ERRACT_SHUTDOWN_CONV              1
#define WM8350_DC2_ERRACT_SHUTDOWN_SYS               2

/*
 * R186 (0xBA) - DCDC3 Control
 */
#define WM8350_DC3_OPFLT                        0x0400  /* DC3_OPFLT */
#define WM8350_DC3_VSEL_MASK                    0x007F  /* DC3_VSEL - [6:0] */
#define WM8350_DC3_VSEL_SHIFT                        0  /* DC3_VSEL - [6:0] */

/* Bit values for R186 (0xBA) */
#define WM8350_DC3_VSEL( _mV )                  (((_mV)-850)/25)
#define WM8350_DC3_VSEL_V( _val )               (((_val)*25)+850)

/*
 * R187 (0xBB) - DCDC3 Timeouts
 */
#define WM8350_DC3_ERRACT_MASK                  0xC000  /* DC3_ERRACT - [15:14] */
#define WM8350_DC3_ERRACT_SHIFT                     14  /* DC3_ERRACT - [15:14] */
#define WM8350_DC3_ENSLOT_MASK                  0x3C00  /* DC3_ENSLOT - [13:10] */
#define WM8350_DC3_ENSLOT_SHIFT                     10  /* DC3_ENSLOT - [13:10] */
#define WM8350_DC3_SDSLOT_MASK                  0x03C0  /* DC3_SDSLOT - [9:6] */
#define WM8350_DC3_UVTO_MASK                    0x0030  /* DC3_UVTO - [5:4] */
#define WM8350_DC3_SDSLOT_SHIFT                      6  /* DC3_SDSLOT - [9:6] */

/* Bit values for R187 (0xBB) */
#define WM8350_DC3_ERRACT_NONE                       0
#define WM8350_DC3_ERRACT_SHUTDOWN_CONV              1
#define WM8350_DC3_ERRACT_SHUTDOWN_SYS               2
/*
 * R188 (0xBC) - DCDC3 Low Power
 */
#define WM8350_DC3_HIB_MODE_MASK                0x7000  /* DC3_HIB_MODE - [14:12] */
#define WM8350_DC3_HIB_TRIG_MASK                0x0300  /* DC3_HIB_TRIG - [9:8] */
#define WM8350_DC3_VIMG_MASK                    0x007F  /* DC3_VIMG - [6:0] */

/*
 * R189 (0xBD) - DCDC4 Control
 */
#define WM8350_DC4_OPFLT                        0x0400  /* DC4_OPFLT */
#define WM8350_DC4_VSEL_MASK                    0x007F  /* DC4_VSEL - [6:0] */
#define WM8350_DC4_VSEL_SHIFT                        0  /* DC4_VSEL - [6:0] */

/* Bit values for R189 (0xBD) */
#define WM8350_DC4_VSEL( _mV )                  (((_mV)-850)/25)
#define WM8350_DC4_VSEL_V( _val )               (((_val)*25)+850)

/*
 * R190 (0xBE) - DCDC4 Timeouts
 */
#define WM8350_DC4_ERRACT_MASK                  0xC000  /* DC4_ERRACT - [15:14] */
#define WM8350_DC4_ERRACT_SHIFT                     14  /* DC4_ERRACT - [15:14] */
#define WM8350_DC4_ENSLOT_MASK                  0x3C00  /* DC4_ENSLOT - [13:10] */
#define WM8350_DC4_ENSLOT_SHIFT                     10  /* DC4_ENSLOT - [13:10] */
#define WM8350_DC4_SDSLOT_MASK                  0x03C0  /* DC4_SDSLOT - [9:6] */
#define WM8350_DC4_UVTO_MASK                    0x0030  /* DC4_UVTO - [5:4] */
#define WM8350_DC4_SDSLOT_SHIFT                      6  /* DC4_SDSLOT - [9:6] */

/* Bit values for R190 (0xBE) */
#define WM8350_DC4_ERRACT_NONE                       0
#define WM8350_DC4_ERRACT_SHUTDOWN_CONV              1
#define WM8350_DC4_ERRACT_SHUTDOWN_SYS               2

/*
 * R191 (0xBF) - DCDC4 Low Power
 */
#define WM8350_DC4_HIB_MODE_MASK                0x7000  /* DC4_HIB_MODE - [14:12] */
#define WM8350_DC4_HIB_TRIG_MASK                0x0300  /* DC4_HIB_TRIG - [9:8] */
#define WM8350_DC4_VIMG_MASK                    0x007F  /* DC4_VIMG - [6:0] */

/*
 * R192 (0xC0) - DCDC5 Control
 */
#define WM8350_DC5_MODE                         0x4000  /* DC5_MODE */
#define WM8350_DC5_MODE_MASK                    0x4000  /* DC5_MODE */
#define WM8350_DC5_MODE_SHIFT                       14  /* DC5_MODE */
#define WM8350_DC5_HIB_MODE                     0x1000  /* DC5_HIB_MODE */
#define WM8350_DC5_HIB_MODE_MASK                0x1000  /* DC5_HIB_MODE */
#define WM8350_DC5_HIB_MODE_SHIFT                   12  /* DC5_HIB_MODE */
#define WM8350_DC5_HIB_TRIG_MASK                0x0300  /* DC5_HIB_TRIG - [9:8] */
#define WM8350_DC5_HIB_TRIG_SHIFT                    8  /* DC5_HIB_TRIG - [9:8] */
#define WM8350_DC5_ILIM                         0x0040  /* DC5_ILIM */
#define WM8350_DC5_ILIM_MASK                    0x0040  /* DC5_ILIM */
#define WM8350_DC5_ILIM_SHIFT                        6  /* DC5_ILIM */
#define WM8350_DC5_RMP_MASK                     0x0018  /* DC5_RMP - [4:3] */
#define WM8350_DC5_RMP_SHIFT                         3  /* DC5_RMP */
#define WM8350_DC5_FBSRC_MASK                   0x0003  /* DC5_FBSRC - [1:0] */
#define WM8350_DC5_FBSRC_SHIFT                       0  /* DC5_FBSRC - [1:0] */

/* Bit values for R192 (0xC0) */
#define WM8350_DC5_MODE_BOOST                        0
#define WM8350_DC5_MODE_SWITCH                       1

#define WM8350_DC5_HIB_MODE_ACTIVE                   0
#define WM8350_DC5_HIB_MODE_DISABLE                  1

#define WM8350_DC5_HIB_TRIG_NONE                     0
#define WM8350_DC5_HIB_TRIG_LPWR1                    1
#define WM8350_DC5_HIB_TRIG_LPWR2                    2
#define WM8350_DC5_HIB_TRIG_LPWR3                    3

#define WM8350_DC5_ILIM_HIGH                         0
#define WM8350_DC5_ILIM_LOW                          1

#define WM8350_DC5_RMP_30V                           0
#define WM8350_DC5_RMP_20V                           1
#define WM8350_DC5_RMP_10V                           2
#define WM8350_DC5_RMP_5V                            3

#define WM8350_DC5_FBSRC_FB2                         0
#define WM8350_DC5_FBSRC_ISINKA                      1
#define WM8350_DC5_FBSRC_ISINKB                      2
#define WM8350_DC5_FBSRC_USB                         3


/*
 * R193 (0xC1) - DCDC5 Timeouts
 */
#define WM8350_DC5_ERRACT_MASK                  0xC000  /* DC5_ERRACT - [15:14] */
#define WM8350_DC5_ERRACT_SHIFT                     14  /* DC5_ERRACT - [15:14] */
#define WM8350_DC5_ENSLOT_MASK                  0x3C00  /* DC5_ENSLOT - [13:10] */
#define WM8350_DC5_ENSLOT_SHIFT                     10  /* DC5_ENSLOT - [13:10] */
#define WM8350_DC5_SDSLOT_MASK                  0x03C0  /* DC5_SDSLOT - [9:6] */
#define WM8350_DC5_UVTO_MASK                    0x0030  /* DC5_UVTO - [5:4] */
#define WM8350_DC5_SDSLOT_SHIFT                      6  /* DC5_SDSLOT - [9:6] */

/* Bit values for R193 (0xC1) */
#define WM8350_DC5_ERRACT_NONE                       0
#define WM8350_DC5_ERRACT_SHUTDOWN_CONV              1
#define WM8350_DC5_ERRACT_SHUTDOWN_SYS               2

/*
 * R195 (0xC3) - DCDC6 Control
 */
#define WM8350_DC6_OPFLT                        0x0400  /* DC6_OPFLT */
#define WM8350_DC6_VSEL_MASK                    0x007F  /* DC6_VSEL - [6:0] */
#define WM8350_DC6_VSEL_SHIFT                        0  /* DC6_VSEL - [6:0] */

/* Bit values for R195 (0xC3) */
#define WM8350_DC6_VSEL( _mV )                  (((_mV)-850)/25)
#define WM8350_DC6_VSEL_V( _val )               (((_val)*25)+850)

/*
 * R196 (0xC4) - DCDC6 Timeouts
 */
#define WM8350_DC6_ERRACT_MASK                  0xC000  /* DC6_ERRACT - [15:14] */
#define WM8350_DC6_ERRACT_SHIFT                     14  /* DC6_ERRACT - [15:14] */
#define WM8350_DC6_ENSLOT_MASK                  0x3C00  /* DC6_ENSLOT - [13:10] */
#define WM8350_DC6_ENSLOT_SHIFT                     10  /* DC6_ENSLOT - [13:10] */
#define WM8350_DC6_SDSLOT_MASK                  0x03C0  /* DC6_SDSLOT - [9:6] */
#define WM8350_DC6_UVTO_MASK                    0x0030  /* DC6_UVTO - [5:4] */
#define WM8350_DC6_SDSLOT_SHIFT                      6  /* DC6_SDSLOT - [9:6] */

/* Bit values for R196 (0xC4) */
#define WM8350_DC6_ERRACT_NONE                       0
#define WM8350_DC6_ERRACT_SHUTDOWN_CONV              1
#define WM8350_DC6_ERRACT_SHUTDOWN_SYS               2

/*
 * R197 (0xC5) - DCDC6 Low Power
 */
#define WM8350_DC6_HIB_MODE_MASK                0x7000  /* DC6_HIB_MODE - [14:12] */
#define WM8350_DC6_HIB_TRIG_MASK                0x0300  /* DC6_HIB_TRIG - [9:8] */
#define WM8350_DC6_VIMG_MASK                    0x007F  /* DC6_VIMG - [6:0] */

/*
 * R199 (0xC7) - Limit Switch Control
 */
#define WM8350_LS_ERRACT_MASK                   0xC000  /* LS_ERRACT - [15:14] */
#define WM8350_LS_ERRACT_SHIFT                      14  /* LS_ERRACT - [15:14] */
#define WM8350_LS_ENSLOT_MASK                   0x3C00  /* LS_ENSLOT - [13:10] */
#define WM8350_LS_ENSLOT_SHIFT                      10  /* LS_ENSLOT - [13:10] */
#define WM8350_LS_SDSLOT_MASK                   0x03C0  /* LS_SDSLOT - [9:6] */
#define WM8350_LS_SDSLOT_SHIFT                       6  /* LS_SDSLOT - [9:6] */
#define WM8350_LS_HIB_MODE                      0x0010  /* LS_HIB_MODE */
#define WM8350_LS_HIB_MODE_MASK                 0x0010  /* LS_HIB_MODE */
#define WM8350_LS_HIB_MODE_SHIFT                     4  /* LS_HIB_MODE */
#define WM8350_LS_HIB_PROT                      0x0002  /* LS_HIB_PROT */
#define WM8350_LS_HIB_PROT_MASK                 0x0002  /* LS_HIB_PROT */
#define WM8350_LS_HIB_PROT_SHIFT                     1  /* LS_HIB_PROT */
#define WM8350_LS_PROT                          0x0001  /* LS_PROT */
#define WM8350_LS_PROT_MASK                     0x0001  /* LS_PROT */
#define WM8350_LS_PROT_SHIFT                         0  /* LS_PROT */

/* Bit values for R199 (0xC7) */
#define WM8350_LS_ERRACT_NONE                       0
#define WM8350_LS_ERRACT_SHUTDOWN_CONV              1
#define WM8350_LS_ERRACT_SHUTDOWN_SYS               2

/*
 * R200 (0xC8) - LDO1 Control
 */
#define WM8350_LDO1_SWI                         0x4000  /* LDO1_SWI */
#define WM8350_LDO1_OPFLT                       0x0400  /* LDO1_OPFLT */
#define WM8350_LDO1_VSEL_MASK                   0x001F  /* LDO1_VSEL - [4:0] */
#define WM8350_LDO1_VSEL_SHIFT                       0  /* LDO1_VSEL - [4:0] */

/* Bit values for R200 (0xC8) */
#define WM8350_LDO1_VSEL( _mV )                 ((u16)( ((_mV)<1800) ? (((_mV)-900)/50)  : (((_mV)-1800)/100 + 16) ))
#define WM8350_LDO1_VSEL_V( _val )              ( ((_val)<16)  ? (((_val)*50)+900) : ((((_val)-16)*100)+1800) )

/*
 * R201 (0xC9) - LDO1 Timeouts
 */
#define WM8350_LDO1_ERRACT_MASK                 0xC000  /* LDO1_ERRACT - [15:14] */
#define WM8350_LDO1_ERRACT_SHIFT                    14  /* LDO1_ERRACT - [15:14] */
#define WM8350_LDO1_ENSLOT_MASK                 0x3C00  /* LDO1_ENSLOT - [13:10] */
#define WM8350_LDO1_ENSLOT_SHIFT                    10  /* LDO1_ENSLOT - [13:10] */
#define WM8350_LDO1_SDSLOT_MASK                 0x03C0  /* LDO1_SDSLOT - [9:6] */
#define WM8350_LDO1_UVTO_MASK                   0x0030  /* LDO1_UVTO - [5:4] */
#define WM8350_LDO1_SDSLOT_SHIFT                     6  /* LDO1_SDSLOT - [9:6] */

/* Bit values for R201 (0xC9) */
#define WM8350_LDO1_ERRACT_NONE                       0
#define WM8350_LDO1_ERRACT_SHUTDOWN_CONV              1
#define WM8350_LDO1_ERRACT_SHUTDOWN_SYS               2

/*
 * R202 (0xCA) - LDO1 Low Power
 */
#define WM8350_LDO1_HIB_MODE_MASK               0x3000  /* LDO1_HIB_MODE - [13:12] */
#define WM8350_LDO1_HIB_TRIG_MASK               0x0300  /* LDO1_HIB_TRIG - [9:8] */
#define WM8350_LDO1_VIMG_MASK                   0x001F  /* LDO1_VIMG - [4:0] */

/*
 * R203 (0xCB) - LDO2 Control
 */
#define WM8350_LDO2_SWI                         0x4000  /* LDO2_SWI */
#define WM8350_LDO2_OPFLT                       0x0400  /* LDO2_OPFLT */
#define WM8350_LDO2_VSEL_MASK                   0x001F  /* LDO2_VSEL - [4:0] */
#define WM8350_LDO2_VSEL_SHIFT                       0  /* LDO2_VSEL - [4:0] */

/* Bit values for R203 (0xCB) */
#define WM8350_LDO2_VSEL( _mV )                 ((u16)( ((_mV)<1800) ? (((_mV)-900)/50)  : (((_mV)-1800)/100 + 16) ))
#define WM8350_LDO2_VSEL_V( _val )              ( ((_val)<16)  ? (((_val)*50)+900) : ((((_val)-16)*100)+1800) )

/*
 * R204 (0xCC) - LDO2 Timeouts
 */
#define WM8350_LDO2_ERRACT_MASK                 0xC000  /* LDO2_ERRACT - [15:14] */
#define WM8350_LDO2_ERRACT_SHIFT                    14  /* LDO2_ERRACT - [15:14] */
#define WM8350_LDO2_ENSLOT_MASK                 0x3C00  /* LDO2_ENSLOT - [13:10] */
#define WM8350_LDO2_ENSLOT_SHIFT                    10  /* LDO2_ENSLOT - [13:10] */
#define WM8350_LDO2_SDSLOT_MASK                 0x03C0  /* LDO2_SDSLOT - [9:6] */
#define WM8350_LDO2_SDSLOT_SHIFT                     6  /* LDO2_SDSLOT - [9:6] */

/* Bit values for R204 (0xCC) */
#define WM8350_LDO2_ERRACT_NONE                       0
#define WM8350_LDO2_ERRACT_SHUTDOWN_CONV              1
#define WM8350_LDO2_ERRACT_SHUTDOWN_SYS               2

/*
 * R205 (0xCD) - LDO2 Low Power
 */
#define WM8350_LDO2_HIB_MODE_MASK               0x3000  /* LDO2_HIB_MODE - [13:12] */
#define WM8350_LDO2_HIB_TRIG_MASK               0x0300  /* LDO2_HIB_TRIG - [9:8] */
#define WM8350_LDO2_VIMG_MASK                   0x001F  /* LDO2_VIMG - [4:0] */

/*
 * R206 (0xCE) - LDO3 Control
 */
#define WM8350_LDO3_SWI                         0x4000  /* LDO3_SWI */
#define WM8350_LDO3_OPFLT                       0x0400  /* LDO3_OPFLT */
#define WM8350_LDO3_VSEL_MASK                   0x001F  /* LDO3_VSEL - [4:0] */
#define WM8350_LDO3_VSEL_SHIFT                       0  /* LDO3_VSEL - [4:0] */

/* Bit values for R206 (0xCE) */
#define WM8350_LDO3_VSEL( _mV )                 ((u16)( ((_mV)<1800) ? (((_mV)-900)/50)  : (((_mV)-1800)/100 + 16) ))
#define WM8350_LDO3_VSEL_V( _val )              ( ((_val)<16)  ? (((_val)*50)+900) : ((((_val)-16)*100)+1800) )

/*
 * R207 (0xCF) - LDO3 Timeouts
 */
#define WM8350_LDO3_ERRACT_MASK                 0xC000  /* LDO3_ERRACT - [15:14] */
#define WM8350_LDO3_ERRACT_SHIFT                    14  /* LDO3_ERRACT - [15:14] */
#define WM8350_LDO3_ENSLOT_MASK                 0x3C00  /* LDO3_ENSLOT - [13:10] */
#define WM8350_LDO3_ENSLOT_SHIFT                    10  /* LDO3_ENSLOT - [13:10] */
#define WM8350_LDO3_SDSLOT_MASK                 0x03C0  /* LDO3_SDSLOT - [9:6] */
#define WM8350_LDO3_UVTO_MASK                   0x0030  /* LDO3_UVTO - [5:4] */
#define WM8350_LDO3_SDSLOT_SHIFT                     6  /* LDO3_SDSLOT - [9:6] */

/* Bit values for R207 (0xCF) */
#define WM8350_LDO3_ERRACT_NONE                       0
#define WM8350_LDO3_ERRACT_SHUTDOWN_CONV              1
#define WM8350_LDO3_ERRACT_SHUTDOWN_SYS               2


/*
 * R208 (0xD0) - LDO3 Low Power
 */
#define WM8350_LDO3_HIB_MODE_MASK               0x3000  /* LDO3_HIB_MODE - [13:12] */
#define WM8350_LDO3_HIB_TRIG_MASK               0x0300  /* LDO3_HIB_TRIG - [9:8] */
#define WM8350_LDO3_VIMG_MASK                   0x001F  /* LDO3_VIMG - [4:0] */

/*
 * R209 (0xD1) - LDO4 Control
 */
#define WM8350_LDO4_SWI                         0x4000  /* LDO4_SWI */
#define WM8350_LDO4_OPFLT                       0x0400  /* LDO4_OPFLT */
#define WM8350_LDO4_VSEL_MASK                   0x001F  /* LDO4_VSEL - [4:0] */
#define WM8350_LDO4_VSEL_SHIFT                       0  /* LDO4_VSEL - [4:0] */

/* Bit values for R209 (0xD1) */
#define WM8350_LDO4_VSEL( _mV )                 ((u16)( ((_mV)<1800) ? (((_mV)-900)/50)  : (((_mV)-1800)/100 + 16) ))
#define WM8350_LDO4_VSEL_V( _val )              ( ((_val)<16)  ? (((_val)*50)+900) : ((((_val)-16)*100)+1800) )

/*
 * R210 (0xD2) - LDO4 Timeouts
 */
#define WM8350_LDO4_ERRACT_MASK                 0xC000  /* LDO4_ERRACT - [15:14] */
#define WM8350_LDO4_ERRACT_SHIFT                    14  /* LDO4_ERRACT - [15:14] */
#define WM8350_LDO4_ENSLOT_MASK                 0x3C00  /* LDO4_ENSLOT - [13:10] */
#define WM8350_LDO4_ENSLOT_SHIFT                    10  /* LDO4_ENSLOT - [13:10] */
#define WM8350_LDO4_SDSLOT_MASK                 0x03C0  /* LDO4_SDSLOT - [9:6] */
#define WM8350_LDO4_UVTO_MASK                   0x0030  /* LDO4_UVTO - [5:4] */
#define WM8350_LDO4_SDSLOT_SHIFT                     6  /* LDO4_SDSLOT - [9:6] */

/* Bit values for R210 (0xD2) */
#define WM8350_LDO4_ERRACT_NONE                       0
#define WM8350_LDO4_ERRACT_SHUTDOWN_CONV              1
#define WM8350_LDO4_ERRACT_SHUTDOWN_SYS               2

/*
 * R211 (0xD3) - LDO4 Low Power
 */
#define WM8350_LDO4_HIB_MODE_MASK               0x3000  /* LDO4_HIB_MODE - [13:12] */
#define WM8350_LDO4_HIB_TRIG_MASK               0x0300  /* LDO4_HIB_TRIG - [9:8] */
#define WM8350_LDO4_VIMG_MASK                   0x001F  /* LDO4_VIMG - [4:0] */

/*
 * R215 (0xD7) - VCC_FAULT Masks
 */
#define WM8350_LS_FAULT                         0x8000  /* LS_FAULT */
#define WM8350_LDO4_FAULT                       0x0800  /* LDO4_FAULT */
#define WM8350_LDO3_FAULT                       0x0400  /* LDO3_FAULT */
#define WM8350_LDO2_FAULT                       0x0200  /* LDO2_FAULT */
#define WM8350_LDO1_FAULT                       0x0100  /* LDO1_FAULT */
#define WM8350_DC6_FAULT                        0x0020  /* DC6_FAULT */
#define WM8350_DC5_FAULT                        0x0010  /* DC5_FAULT */
#define WM8350_DC4_FAULT                        0x0008  /* DC4_FAULT */
#define WM8350_DC3_FAULT                        0x0004  /* DC3_FAULT */
#define WM8350_DC2_FAULT                        0x0002  /* DC2_FAULT */
#define WM8350_DC1_FAULT                        0x0001  /* DC1_FAULT */

/*
 * R216 (0xD8) - Main Bandgap Control
 */
#define WM8350_MBG_LOAD_FUSES                   0x8000  /* MBG_LOAD_FUSES */
#define WM8350_MBG_FUSE_WPREP                   0x4000  /* MBG_FUSE_WPREP */
#define WM8350_MBG_FUSE_WRITE                   0x2000  /* MBG_FUSE_WRITE */
#define WM8350_MBG_FUSE_TRIM_MASK               0x1F00  /* MBG_FUSE_TRIM - [12:8] */
#define WM8350_MBG_TRIM_SRC                     0x0020  /* MBG_TRIM_SRC */
#define WM8350_MBG_USER_TRIM_MASK               0x001F  /* MBG_USER_TRIM - [4:0] */

/*
 * R217 (0xD9) - OSC Control
 */
#define WM8350_OSC_LOAD_FUSES                   0x8000  /* OSC_LOAD_FUSES */
#define WM8350_OSC_FUSE_WPREP                   0x4000  /* OSC_FUSE_WPREP */
#define WM8350_OSC_FUSE_WRITE                   0x2000  /* OSC_FUSE_WRITE */
#define WM8350_OSC_FUSE_TRIM_MASK               0x0F00  /* OSC_FUSE_TRIM - [11:8] */
#define WM8350_OSC_TRIM_SRC                     0x0020  /* OSC_TRIM_SRC */
#define WM8350_OSC_USER_TRIM_MASK               0x000F  /* OSC_USER_TRIM - [3:0] */

/*
 * R218 (0xDA) - RTC Tick Control
 */
// lg #define WM8350_RTC_TICK_ENA                     0x8000  /* RTC_TICK_ENA */
#define WM8350_RTC_TICKSTS                      0x4000  /* RTC_TICKSTS */
#define WM8350_RTC_CLKSRC                       0x2000  /* RTC_CLKSRC */
// lg #define WM8350_OSC32K_ENA                       0x1000  /* OSC32K_ENA */
#define WM8350_RTC_TRIM_MASK                    0x03FF  /* RTC_TRIM - [9:0] */

/*
 * R220 (0xDC) - RAM BIST 1
 */
#define WM8350_READ_STATUS                      0x0800  /* READ_STATUS */
#define WM8350_TSTRAM_CLK                       0x0100  /* TSTRAM_CLK */
#define WM8350_TSTRAM_CLK_ENA                   0x0080  /* TSTRAM_CLK_ENA */
#define WM8350_STARTSEQ                         0x0040  /* STARTSEQ */
#define WM8350_READ_SRC                         0x0020  /* READ_SRC */
#define WM8350_COUNT_DIR                        0x0010  /* COUNT_DIR */
#define WM8350_TSTRAM_MODE_MASK                 0x000E  /* TSTRAM_MODE - [3:1] */
#define WM8350_TSTRAM_ENA                       0x0001  /* TSTRAM_ENA */

/*
 * R225 (0xE1) - DCDC/LDO status
 */
#define WM8350_LS_STS                           0x8000  /* LS_STS */
#define WM8350_LDO4_STS                         0x0800  /* LDO4_STS */
#define WM8350_LDO3_STS                         0x0400  /* LDO3_STS */
#define WM8350_LDO2_STS                         0x0200  /* LDO2_STS */
#define WM8350_LDO1_STS                         0x0100  /* LDO1_STS */
#define WM8350_DC6_STS                          0x0020  /* DC6_STS */
#define WM8350_DC5_STS                          0x0010  /* DC5_STS */
#define WM8350_DC4_STS                          0x0008  /* DC4_STS */
#define WM8350_DC3_STS                          0x0004  /* DC3_STS */
#define WM8350_DC2_STS                          0x0002  /* DC2_STS */
#define WM8350_DC1_STS                          0x0001  /* DC1_STS */

/*
 * R230 (0xE6) - GPIO Pin Status
 */
#define WM8350_GP12_LVL                         0x1000  /* GP12_LVL */
#define WM8350_GP11_LVL                         0x0800  /* GP11_LVL */
#define WM8350_GP10_LVL                         0x0400  /* GP10_LVL */
#define WM8350_GP9_LVL                          0x0200  /* GP9_LVL */
#define WM8350_GP8_LVL                          0x0100  /* GP8_LVL */
#define WM8350_GP7_LVL                          0x0080  /* GP7_LVL */
#define WM8350_GP6_LVL                          0x0040  /* GP6_LVL */
#define WM8350_GP5_LVL                          0x0020  /* GP5_LVL */
#define WM8350_GP4_LVL                          0x0010  /* GP4_LVL */
#define WM8350_GP3_LVL                          0x0008  /* GP3_LVL */
#define WM8350_GP2_LVL                          0x0004  /* GP2_LVL */
#define WM8350_GP1_LVL                          0x0002  /* GP1_LVL */
#define WM8350_GP0_LVL                          0x0001  /* GP0_LVL */

/*
 * R231 (0xE7) - Jack Status
 */
#define WM8350_JACK_R_LVL                       0x0400  /* Right Jack */

/*
 * Default values.
 */
#define WM8350_CONFIG_BANKS                     4

/* Bank 0 */
#define WM8350_REGISTER_DEFAULTS_0 \
{ \
    0x17FF,     /* R0   - Reset/ID */ \
    0x1000,     /* R1   - ID */ \
    0x0000,     /* R2 */ \
    0x1002,     /* R3   - System Control 1 */ \
    0x0004,     /* R4   - System Control 2 */ \
    0x0000,     /* R5   - System Hibernate */ \
    0x8A00,     /* R6   - Interface Control */ \
    0x0000,     /* R7 */ \
    0x8000,     /* R8   - Power mgmt (1) */ \
    0x0000,     /* R9   - Power mgmt (2) */ \
    0x0000,     /* R10  - Power mgmt (3) */ \
    0x2000,     /* R11  - Power mgmt (4) */ \
    0x0E00,     /* R12  - Power mgmt (5) */ \
    0x0000,     /* R13  - Power mgmt (6) */ \
    0x0000,     /* R14  - Power mgmt (7) */ \
    0x0000,     /* R15 */ \
    0x0000,     /* R16  - RTC Seconds/Minutes */ \
    0x0100,     /* R17  - RTC Hours/Day */ \
    0x0101,     /* R18  - RTC Date/Month */ \
    0x1400,     /* R19  - RTC Year */ \
    0x0000,     /* R20  - Alarm Seconds/Minutes */ \
    0x0000,     /* R21  - Alarm Hours/Day */ \
    0x0000,     /* R22  - Alarm Date/Month */ \
    0x0320,     /* R23  - RTC Time Control */ \
    0x0000,     /* R24  - System Interrupts */ \
    0x0000,     /* R25  - Interrupt Status 1 */ \
    0x0000,     /* R26  - Interrupt Status 2 */ \
    0x0000,     /* R27  - Power Up Interrupt Status */ \
    0x0000,     /* R28  - Under Voltage Interrupt status */ \
    0x0000,     /* R29  - Over Current Interrupt status */ \
    0x0000,     /* R30  - GPIO Interrupt Status */ \
    0x0000,     /* R31  - Comparator Interrupt Status */ \
    0x3FFF,     /* R32  - System Interrupts Mask */ \
    0x0000,     /* R33  - Interrupt Status 1 Mask */ \
    0x0000,     /* R34  - Interrupt Status 2 Mask */ \
    0x0000,     /* R35  - Power Up Interrupt Status Mask */ \
    0x0000,     /* R36  - Under Voltage Interrupt status Mask */ \
    0x0000,     /* R37  - Over Current Interrupt status Mask */ \
    0x0000,     /* R38  - GPIO Interrupt Status Mask */ \
    0x0000,     /* R39  - Comparator Interrupt Status Mask */ \
    0x0040,     /* R40  - Clock Control 1 */ \
    0x0000,     /* R41  - Clock Control 2 */ \
    0x3B00,     /* R42  - FLL Control 1 */ \
    0x7086,     /* R43  - FLL Control 2 */ \
    0xC226,     /* R44  - FLL Control 3 */ \
    0x0000,     /* R45  - FLL Control 4 */ \
    0x0000,     /* R46 */ \
    0x0000,     /* R47 */ \
    0x0000,     /* R48  - DAC Control */ \
    0x0000,     /* R49 */ \
    0x00C0,     /* R50  - DAC Digital Volume L */ \
    0x00C0,     /* R51  - DAC Digital Volume R */ \
    0x0000,     /* R52 */ \
    0x0040,     /* R53  - DAC LR Rate */ \
    0x0000,     /* R54  - DAC Clock Control */ \
    0x0000,     /* R55 */ \
    0x0000,     /* R56 */ \
    0x0000,     /* R57 */ \
    0x4000,     /* R58  - DAC Mute */ \
    0x0000,     /* R59  - DAC Mute Volume */ \
    0x0000,     /* R60  - DAC Side */ \
    0x0000,     /* R61 */ \
    0x0000,     /* R62 */ \
    0x0000,     /* R63 */ \
    0x8000,     /* R64  - ADC Control */ \
    0x0000,     /* R65 */ \
    0x00C0,     /* R66  - ADC Digital Volume L */ \
    0x00C0,     /* R67  - ADC Digital Volume R */ \
    0x0000,     /* R68  - ADC Divider */ \
    0x0000,     /* R69 */ \
    0x0040,     /* R70  - ADC LR Rate */ \
    0x0000,     /* R71 */ \
    0x0303,     /* R72  - Input Control */ \
    0x0000,     /* R73  - IN3 Input Control */ \
    0x0000,     /* R74  - Mic Bias Control */ \
    0x0000,     /* R75 */ \
    0x0000,     /* R76  - Output Control */ \
    0x0000,     /* R77  - Jack Detect */ \
    0x0000,     /* R78  - Anti Pop Control */ \
    0x0000,     /* R79 */ \
    0x0040,     /* R80  - Left Input Volume */ \
    0x0040,     /* R81  - Right Input Volume */ \
    0x0000,     /* R82 */ \
    0x0000,     /* R83 */ \
    0x0000,     /* R84 */ \
    0x0000,     /* R85 */ \
    0x0000,     /* R86 */ \
    0x0000,     /* R87 */ \
    0x0800,     /* R88  - Left Mixer Control */ \
    0x1000,     /* R89  - Right Mixer Control */ \
    0x0000,     /* R90 */ \
    0x0000,     /* R91 */ \
    0x0000,     /* R92  - OUT3 Mixer Control */ \
    0x0000,     /* R93  - OUT4 Mixer Control */ \
    0x0000,     /* R94 */ \
    0x0000,     /* R95 */ \
    0x0000,     /* R96  - Output Left Mixer Volume */ \
    0x0000,     /* R97  - Output Right Mixer Volume */ \
    0x0000,     /* R98  - Input Mixer Volume L */ \
    0x0000,     /* R99  - Input Mixer Volume R */ \
    0x0000,     /* R100 - Input Mixer Volume */ \
    0x0000,     /* R101 */ \
    0x0000,     /* R102 */ \
    0x0000,     /* R103 */ \
    0x00E4,     /* R104 - LOUT1 Volume */ \
    0x00E4,     /* R105 - ROUT1 Volume */ \
    0x00E4,     /* R106 - LOUT2 Volume */ \
    0x02E4,     /* R107 - ROUT2 Volume */ \
    0x0000,     /* R108 */ \
    0x0000,     /* R109 */ \
    0x0000,     /* R110 */ \
    0x0000,     /* R111 - BEEP Volume */ \
    0x0A00,     /* R112 - AI Formating */ \
    0x0000,     /* R113 - ADC DAC COMP */ \
    0x0020,     /* R114 - AI ADC Control */ \
    0x0020,     /* R115 - AI DAC Control */ \
    0x0000,     /* R116 - AIF Test */ \
    0x0000,     /* R117 */ \
    0x0000,     /* R118 */ \
    0x0000,     /* R119 */ \
    0x0000,     /* R120 */ \
    0x0000,     /* R121 */ \
    0x0000,     /* R122 */ \
    0x0000,     /* R123 */ \
    0x0000,     /* R124 */ \
    0x0000,     /* R125 */ \
    0x0000,     /* R126 */ \
    0x0000,     /* R127 */ \
    0x1FFF,     /* R128 - GPIO Debounce */ \
    0x0000,     /* R129 - GPIO Pin pull up Control */ \
    0x03FC,     /* R130 - GPIO Pull down Control */ \
    0x0000,     /* R131 - GPIO Interrupt Mode */ \
    0x0000,     /* R132 */ \
    0x0000,     /* R133 - GPIO Control */ \
    0x0FFC,     /* R134 - GPIO Configuration (i/o) */ \
    0x0FFC,     /* R135 - GPIO Pin Polarity / Type */ \
    0x0000,     /* R136 */ \
    0x0000,     /* R137 */ \
    0x0000,     /* R138 */ \
    0x0000,     /* R139 */ \
    0x0013,     /* R140 - GPIO Function Select 1 */ \
    0x0000,     /* R141 - GPIO Function Select 2 */ \
    0x0000,     /* R142 - GPIO Function Select 3 */ \
    0x0003,     /* R143 - GPIO Function Select 4 */ \
    0x0000,     /* R144 - Digitiser Control (1) */ \
    0x0002,     /* R145 - Digitiser Control (2) */ \
    0x0000,     /* R146 */ \
    0x0000,     /* R147 */ \
    0x0000,     /* R148 */ \
    0x0000,     /* R149 */ \
    0x0000,     /* R150 */ \
    0x0000,     /* R151 */ \
    0x7000,     /* R152 - AUX1 Readback */ \
    0x7000,     /* R153 - AUX2 Readback */ \
    0x7000,     /* R154 - AUX3 Readback */ \
    0x7000,     /* R155 - AUX4 Readback */ \
    0x0000,     /* R156 - USB Voltage Readback */ \
    0x0000,     /* R157 - LINE Voltage Readback */ \
    0x0000,     /* R158 - BATT Voltage Readback */ \
    0x0000,     /* R159 - Chip Temp Readback */ \
    0x0000,     /* R160 */ \
    0x0000,     /* R161 */ \
    0x0000,     /* R162 */ \
    0x0000,     /* R163 - Generic Comparator Control */ \
    0x0000,     /* R164 - Generic comparator 1 */ \
    0x0000,     /* R165 - Generic comparator 2 */ \
    0x0000,     /* R166 - Generic comparator 3 */ \
    0x0000,     /* R167 - Generic comparator 4 */ \
    0xA00F,     /* R168 - Battery Charger Control 1 */ \
    0x0B06,     /* R169 - Battery Charger Control 2 */ \
    0x0000,     /* R170 - Battery Charger Control 3 */ \
    0x0000,     /* R171 */ \
    0x0000,     /* R172 - Current Sink Driver A */ \
    0x0000,     /* R173 - CSA Flash control */ \
    0x0000,     /* R174 - Current Sink Driver B */ \
    0x0000,     /* R175 - CSB Flash control */ \
    0x0000,     /* R176 - DCDC/LDO requested */ \
    0x002D,     /* R177 - DCDC Active options */ \
    0x0000,     /* R178 - DCDC Sleep options */ \
    0x0025,     /* R179 - Power-check comparator */ \
    0x000E,     /* R180 - DCDC1 Control */ \
    0x0000,     /* R181 - DCDC1 Timeouts */ \
    0x1006,     /* R182 - DCDC1 Low Power */ \
    0x0018,     /* R183 - DCDC2 Control */ \
    0x0000,     /* R184 - DCDC2 Timeouts */ \
    0x0000,     /* R185 */ \
    0x0000,     /* R186 - DCDC3 Control */ \
    0x0000,     /* R187 - DCDC3 Timeouts */ \
    0x0006,     /* R188 - DCDC3 Low Power */ \
    0x0000,     /* R189 - DCDC4 Control */ \
    0x0000,     /* R190 - DCDC4 Timeouts */ \
    0x0006,     /* R191 - DCDC4 Low Power */ \
    0x0008,     /* R192 - DCDC5 Control */ \
    0x0000,     /* R193 - DCDC5 Timeouts */ \
    0x0000,     /* R194 */ \
    0x0000,     /* R195 - DCDC6 Control */ \
    0x0000,     /* R196 - DCDC6 Timeouts */ \
    0x0006,     /* R197 - DCDC6 Low Power */ \
    0x0000,     /* R198 */ \
    0x0003,     /* R199 - Limit Switch Control */ \
    0x001C,     /* R200 - LDO1 Control */ \
    0x0000,     /* R201 - LDO1 Timeouts */ \
    0x001C,     /* R202 - LDO1 Low Power */ \
    0x001B,     /* R203 - LDO2 Control */ \
    0x0000,     /* R204 - LDO2 Timeouts */ \
    0x001C,     /* R205 - LDO2 Low Power */ \
    0x001B,     /* R206 - LDO3 Control */ \
    0x0000,     /* R207 - LDO3 Timeouts */ \
    0x001C,     /* R208 - LDO3 Low Power */ \
    0x001B,     /* R209 - LDO4 Control */ \
    0x0000,     /* R210 - LDO4 Timeouts */ \
    0x001C,     /* R211 - LDO4 Low Power */ \
    0x0000,     /* R212 */ \
    0x0000,     /* R213 */ \
    0x0000,     /* R214 */ \
    0x0000,     /* R215 - VCC_FAULT Masks */ \
    0x001F,     /* R216 - Main Bandgap Control */ \
    0x0000,     /* R217 - OSC Control */ \
    0x9000,     /* R218 - RTC Tick Control */ \
    0x0000,     /* R219 */ \
    0x4000,     /* R220 - RAM BIST 1 */ \
    0x0000,     /* R221 */ \
    0x0000,     /* R222 */ \
    0x0000,     /* R223 */ \
    0x0000,     /* R224 */ \
    0x0000,     /* R225 - DCDC/LDO status */ \
    0x0000,     /* R226 */ \
    0x0000,     /* R227 */ \
    0x0000,     /* R228 */ \
    0x0000,     /* R229 */ \
    0xE000,     /* R230 - GPIO Pin Status */ \
    0x0000,     /* R231 */ \
    0x0000,     /* R232 */ \
    0x0000,     /* R233 */ \
}

/* Bank 1 */
#define WM8350_REGISTER_DEFAULTS_1 \
{ \
    0x17FF,     /* R0   - Reset/ID */ \
    0x1000,     /* R1   - ID */ \
    0x0000,     /* R2 */ \
    0x1002,     /* R3   - System Control 1 */ \
    0x0014,     /* R4   - System Control 2 */ \
    0x0000,     /* R5   - System Hibernate */ \
    0x8A00,     /* R6   - Interface Control */ \
    0x0000,     /* R7 */ \
    0x8000,     /* R8   - Power mgmt (1) */ \
    0x0000,     /* R9   - Power mgmt (2) */ \
    0x0000,     /* R10  - Power mgmt (3) */ \
    0x2000,     /* R11  - Power mgmt (4) */ \
    0x0E00,     /* R12  - Power mgmt (5) */ \
    0x0000,     /* R13  - Power mgmt (6) */ \
    0x0000,     /* R14  - Power mgmt (7) */ \
    0x0000,     /* R15 */ \
    0x0000,     /* R16  - RTC Seconds/Minutes */ \
    0x0100,     /* R17  - RTC Hours/Day */ \
    0x0101,     /* R18  - RTC Date/Month */ \
    0x1400,     /* R19  - RTC Year */ \
    0x0000,     /* R20  - Alarm Seconds/Minutes */ \
    0x0000,     /* R21  - Alarm Hours/Day */ \
    0x0000,     /* R22  - Alarm Date/Month */ \
    0x0320,     /* R23  - RTC Time Control */ \
    0x0000,     /* R24  - System Interrupts */ \
    0x0000,     /* R25  - Interrupt Status 1 */ \
    0x0000,     /* R26  - Interrupt Status 2 */ \
    0x0000,     /* R27  - Power Up Interrupt Status */ \
    0x0000,     /* R28  - Under Voltage Interrupt status */ \
    0x0000,     /* R29  - Over Current Interrupt status */ \
    0x0000,     /* R30  - GPIO Interrupt Status */ \
    0x0000,     /* R31  - Comparator Interrupt Status */ \
    0x3FFF,     /* R32  - System Interrupts Mask */ \
    0x0000,     /* R33  - Interrupt Status 1 Mask */ \
    0x0000,     /* R34  - Interrupt Status 2 Mask */ \
    0x0000,     /* R35  - Power Up Interrupt Status Mask */ \
    0x0000,     /* R36  - Under Voltage Interrupt status Mask */ \
    0x0000,     /* R37  - Over Current Interrupt status Mask */ \
    0x0000,     /* R38  - GPIO Interrupt Status Mask */ \
    0x0000,     /* R39  - Comparator Interrupt Status Mask */ \
    0x0040,     /* R40  - Clock Control 1 */ \
    0x0000,     /* R41  - Clock Control 2 */ \
    0x3B00,     /* R42  - FLL Control 1 */ \
    0x7086,     /* R43  - FLL Control 2 */ \
    0xC226,     /* R44  - FLL Control 3 */ \
    0x0000,     /* R45  - FLL Control 4 */ \
    0x0000,     /* R46 */ \
    0x0000,     /* R47 */ \
    0x0000,     /* R48  - DAC Control */ \
    0x0000,     /* R49 */ \
    0x00C0,     /* R50  - DAC Digital Volume L */ \
    0x00C0,     /* R51  - DAC Digital Volume R */ \
    0x0000,     /* R52 */ \
    0x0040,     /* R53  - DAC LR Rate */ \
    0x0000,     /* R54  - DAC Clock Control */ \
    0x0000,     /* R55 */ \
    0x0000,     /* R56 */ \
    0x0000,     /* R57 */ \
    0x4000,     /* R58  - DAC Mute */ \
    0x0000,     /* R59  - DAC Mute Volume */ \
    0x0000,     /* R60  - DAC Side */ \
    0x0000,     /* R61 */ \
    0x0000,     /* R62 */ \
    0x0000,     /* R63 */ \
    0x8000,     /* R64  - ADC Control */ \
    0x0000,     /* R65 */ \
    0x00C0,     /* R66  - ADC Digital Volume L */ \
    0x00C0,     /* R67  - ADC Digital Volume R */ \
    0x0000,     /* R68  - ADC Divider */ \
    0x0000,     /* R69 */ \
    0x0040,     /* R70  - ADC LR Rate */ \
    0x0000,     /* R71 */ \
    0x0303,     /* R72  - Input Control */ \
    0x0000,     /* R73  - IN3 Input Control */ \
    0x0000,     /* R74  - Mic Bias Control */ \
    0x0000,     /* R75 */ \
    0x0000,     /* R76  - Output Control */ \
    0x0000,     /* R77  - Jack Detect */ \
    0x0000,     /* R78  - Anti Pop Control */ \
    0x0000,     /* R79 */ \
    0x0040,     /* R80  - Left Input Volume */ \
    0x0040,     /* R81  - Right Input Volume */ \
    0x0000,     /* R82 */ \
    0x0000,     /* R83 */ \
    0x0000,     /* R84 */ \
    0x0000,     /* R85 */ \
    0x0000,     /* R86 */ \
    0x0000,     /* R87 */ \
    0x0800,     /* R88  - Left Mixer Control */ \
    0x1000,     /* R89  - Right Mixer Control */ \
    0x0000,     /* R90 */ \
    0x0000,     /* R91 */ \
    0x0000,     /* R92  - OUT3 Mixer Control */ \
    0x0000,     /* R93  - OUT4 Mixer Control */ \
    0x0000,     /* R94 */ \
    0x0000,     /* R95 */ \
    0x0000,     /* R96  - Output Left Mixer Volume */ \
    0x0000,     /* R97  - Output Right Mixer Volume */ \
    0x0000,     /* R98  - Input Mixer Volume L */ \
    0x0000,     /* R99  - Input Mixer Volume R */ \
    0x0000,     /* R100 - Input Mixer Volume */ \
    0x0000,     /* R101 */ \
    0x0000,     /* R102 */ \
    0x0000,     /* R103 */ \
    0x00E4,     /* R104 - LOUT1 Volume */ \
    0x00E4,     /* R105 - ROUT1 Volume */ \
    0x00E4,     /* R106 - LOUT2 Volume */ \
    0x02E4,     /* R107 - ROUT2 Volume */ \
    0x0000,     /* R108 */ \
    0x0000,     /* R109 */ \
    0x0000,     /* R110 */ \
    0x0000,     /* R111 - BEEP Volume */ \
    0x0A00,     /* R112 - AI Formating */ \
    0x0000,     /* R113 - ADC DAC COMP */ \
    0x0020,     /* R114 - AI ADC Control */ \
    0x0020,     /* R115 - AI DAC Control */ \
    0x0000,     /* R116 - AIF Test */ \
    0x0000,     /* R117 */ \
    0x0000,     /* R118 */ \
    0x0000,     /* R119 */ \
    0x0000,     /* R120 */ \
    0x0000,     /* R121 */ \
    0x0000,     /* R122 */ \
    0x0000,     /* R123 */ \
    0x0000,     /* R124 */ \
    0x0000,     /* R125 */ \
    0x0000,     /* R126 */ \
    0x0000,     /* R127 */ \
    0x1FFF,     /* R128 - GPIO Debounce */ \
    0x0000,     /* R129 - GPIO Pin pull up Control */ \
    0x03FC,     /* R130 - GPIO Pull down Control */ \
    0x0000,     /* R131 - GPIO Interrupt Mode */ \
    0x0000,     /* R132 */ \
    0x0000,     /* R133 - GPIO Control */ \
    0x00FB,     /* R134 - GPIO Configuration (i/o) */ \
    0x04FE,     /* R135 - GPIO Pin Polarity / Type */ \
    0x0000,     /* R136 */ \
    0x0000,     /* R137 */ \
    0x0000,     /* R138 */ \
    0x0000,     /* R139 */ \
    0x0312,     /* R140 - GPIO Function Select 1 */ \
    0x1003,     /* R141 - GPIO Function Select 2 */ \
    0x1331,     /* R142 - GPIO Function Select 3 */ \
    0x0003,     /* R143 - GPIO Function Select 4 */ \
    0x0000,     /* R144 - Digitiser Control (1) */ \
    0x0002,     /* R145 - Digitiser Control (2) */ \
    0x0000,     /* R146 */ \
    0x0000,     /* R147 */ \
    0x0000,     /* R148 */ \
    0x0000,     /* R149 */ \
    0x0000,     /* R150 */ \
    0x0000,     /* R151 */ \
    0x7000,     /* R152 - AUX1 Readback */ \
    0x7000,     /* R153 - AUX2 Readback */ \
    0x7000,     /* R154 - AUX3 Readback */ \
    0x7000,     /* R155 - AUX4 Readback */ \
    0x0000,     /* R156 - USB Voltage Readback */ \
    0x0000,     /* R157 - LINE Voltage Readback */ \
    0x0000,     /* R158 - BATT Voltage Readback */ \
    0x0000,     /* R159 - Chip Temp Readback */ \
    0x0000,     /* R160 */ \
    0x0000,     /* R161 */ \
    0x0000,     /* R162 */ \
    0x0000,     /* R163 - Generic Comparator Control */ \
    0x0000,     /* R164 - Generic comparator 1 */ \
    0x0000,     /* R165 - Generic comparator 2 */ \
    0x0000,     /* R166 - Generic comparator 3 */ \
    0x0000,     /* R167 - Generic comparator 4 */ \
    0xA00F,     /* R168 - Battery Charger Control 1 */ \
    0x0B06,     /* R169 - Battery Charger Control 2 */ \
    0x0000,     /* R170 - Battery Charger Control 3 */ \
    0x0000,     /* R171 */ \
    0x0000,     /* R172 - Current Sink Driver A */ \
    0x0000,     /* R173 - CSA Flash control */ \
    0x0000,     /* R174 - Current Sink Driver B */ \
    0x0000,     /* R175 - CSB Flash control */ \
    0x0000,     /* R176 - DCDC/LDO requested */ \
    0x002D,     /* R177 - DCDC Active options */ \
    0x0000,     /* R178 - DCDC Sleep options */ \
    0x0025,     /* R179 - Power-check comparator */ \
    0x0062,     /* R180 - DCDC1 Control */ \
    0x0400,     /* R181 - DCDC1 Timeouts */ \
    0x1006,     /* R182 - DCDC1 Low Power */ \
    0x0018,     /* R183 - DCDC2 Control */ \
    0x0000,     /* R184 - DCDC2 Timeouts */ \
    0x0000,     /* R185 */ \
    0x0026,     /* R186 - DCDC3 Control */ \
    0x0400,     /* R187 - DCDC3 Timeouts */ \
    0x0006,     /* R188 - DCDC3 Low Power */ \
    0x0062,     /* R189 - DCDC4 Control */ \
    0x0400,     /* R190 - DCDC4 Timeouts */ \
    0x0006,     /* R191 - DCDC4 Low Power */ \
    0x0008,     /* R192 - DCDC5 Control */ \
    0x0000,     /* R193 - DCDC5 Timeouts */ \
    0x0000,     /* R194 */ \
    0x0026,     /* R195 - DCDC6 Control */ \
    0x0800,     /* R196 - DCDC6 Timeouts */ \
    0x0006,     /* R197 - DCDC6 Low Power */ \
    0x0000,     /* R198 */ \
    0x0003,     /* R199 - Limit Switch Control */ \
    0x0006,     /* R200 - LDO1 Control */ \
    0x0400,     /* R201 - LDO1 Timeouts */ \
    0x001C,     /* R202 - LDO1 Low Power */ \
    0x0006,     /* R203 - LDO2 Control */ \
    0x0400,     /* R204 - LDO2 Timeouts */ \
    0x001C,     /* R205 - LDO2 Low Power */ \
    0x001B,     /* R206 - LDO3 Control */ \
    0x0000,     /* R207 - LDO3 Timeouts */ \
    0x001C,     /* R208 - LDO3 Low Power */ \
    0x001B,     /* R209 - LDO4 Control */ \
    0x0000,     /* R210 - LDO4 Timeouts */ \
    0x001C,     /* R211 - LDO4 Low Power */ \
    0x0000,     /* R212 */ \
    0x0000,     /* R213 */ \
    0x0000,     /* R214 */ \
    0x0000,     /* R215 - VCC_FAULT Masks */ \
    0x001F,     /* R216 - Main Bandgap Control */ \
    0x0000,     /* R217 - OSC Control */ \
    0x9000,     /* R218 - RTC Tick Control */ \
    0x0000,     /* R219 */ \
    0x4000,     /* R220 - RAM BIST 1 */ \
    0x0000,     /* R221 */ \
    0x0000,     /* R222 */ \
    0x0000,     /* R223 */ \
    0x0000,     /* R224 */ \
    0x0000,     /* R225 - DCDC/LDO status */ \
    0x0000,     /* R226 */ \
    0x0000,     /* R227 */ \
    0x0000,     /* R228 */ \
    0x0000,     /* R229 */ \
    0xE000,     /* R230 - GPIO Pin Status */ \
    0x0000,     /* R231 */ \
    0x0000,     /* R232 */ \
    0x0000,     /* R233 */ \
}

/* Bank 2 */
#define WM8350_REGISTER_DEFAULTS_2 \
{ \
    0x17FF,     /* R0   - Reset/ID */ \
    0x1000,     /* R1   - ID */ \
    0x0000,     /* R2 */ \
    0x1002,     /* R3   - System Control 1 */ \
    0x0014,     /* R4   - System Control 2 */ \
    0x0000,     /* R5   - System Hibernate */ \
    0x8A00,     /* R6   - Interface Control */ \
    0x0000,     /* R7 */ \
    0x8000,     /* R8   - Power mgmt (1) */ \
    0x0000,     /* R9   - Power mgmt (2) */ \
    0x0000,     /* R10  - Power mgmt (3) */ \
    0x2000,     /* R11  - Power mgmt (4) */ \
    0x0E00,     /* R12  - Power mgmt (5) */ \
    0x0000,     /* R13  - Power mgmt (6) */ \
    0x0000,     /* R14  - Power mgmt (7) */ \
    0x0000,     /* R15 */ \
    0x0000,     /* R16  - RTC Seconds/Minutes */ \
    0x0100,     /* R17  - RTC Hours/Day */ \
    0x0101,     /* R18  - RTC Date/Month */ \
    0x1400,     /* R19  - RTC Year */ \
    0x0000,     /* R20  - Alarm Seconds/Minutes */ \
    0x0000,     /* R21  - Alarm Hours/Day */ \
    0x0000,     /* R22  - Alarm Date/Month */ \
    0x0320,     /* R23  - RTC Time Control */ \
    0x0000,     /* R24  - System Interrupts */ \
    0x0000,     /* R25  - Interrupt Status 1 */ \
    0x0000,     /* R26  - Interrupt Status 2 */ \
    0x0000,     /* R27  - Power Up Interrupt Status */ \
    0x0000,     /* R28  - Under Voltage Interrupt status */ \
    0x0000,     /* R29  - Over Current Interrupt status */ \
    0x0000,     /* R30  - GPIO Interrupt Status */ \
    0x0000,     /* R31  - Comparator Interrupt Status */ \
    0x3FFF,     /* R32  - System Interrupts Mask */ \
    0x0000,     /* R33  - Interrupt Status 1 Mask */ \
    0x0000,     /* R34  - Interrupt Status 2 Mask */ \
    0x0000,     /* R35  - Power Up Interrupt Status Mask */ \
    0x0000,     /* R36  - Under Voltage Interrupt status Mask */ \
    0x0000,     /* R37  - Over Current Interrupt status Mask */ \
    0x0000,     /* R38  - GPIO Interrupt Status Mask */ \
    0x0000,     /* R39  - Comparator Interrupt Status Mask */ \
    0x0040,     /* R40  - Clock Control 1 */ \
    0x0000,     /* R41  - Clock Control 2 */ \
    0x3B00,     /* R42  - FLL Control 1 */ \
    0x7086,     /* R43  - FLL Control 2 */ \
    0xC226,     /* R44  - FLL Control 3 */ \
    0x0000,     /* R45  - FLL Control 4 */ \
    0x0000,     /* R46 */ \
    0x0000,     /* R47 */ \
    0x0000,     /* R48  - DAC Control */ \
    0x0000,     /* R49 */ \
    0x00C0,     /* R50  - DAC Digital Volume L */ \
    0x00C0,     /* R51  - DAC Digital Volume R */ \
    0x0000,     /* R52 */ \
    0x0040,     /* R53  - DAC LR Rate */ \
    0x0000,     /* R54  - DAC Clock Control */ \
    0x0000,     /* R55 */ \
    0x0000,     /* R56 */ \
    0x0000,     /* R57 */ \
    0x4000,     /* R58  - DAC Mute */ \
    0x0000,     /* R59  - DAC Mute Volume */ \
    0x0000,     /* R60  - DAC Side */ \
    0x0000,     /* R61 */ \
    0x0000,     /* R62 */ \
    0x0000,     /* R63 */ \
    0x8000,     /* R64  - ADC Control */ \
    0x0000,     /* R65 */ \
    0x00C0,     /* R66  - ADC Digital Volume L */ \
    0x00C0,     /* R67  - ADC Digital Volume R */ \
    0x0000,     /* R68  - ADC Divider */ \
    0x0000,     /* R69 */ \
    0x0040,     /* R70  - ADC LR Rate */ \
    0x0000,     /* R71 */ \
    0x0303,     /* R72  - Input Control */ \
    0x0000,     /* R73  - IN3 Input Control */ \
    0x0000,     /* R74  - Mic Bias Control */ \
    0x0000,     /* R75 */ \
    0x0000,     /* R76  - Output Control */ \
    0x0000,     /* R77  - Jack Detect */ \
    0x0000,     /* R78  - Anti Pop Control */ \
    0x0000,     /* R79 */ \
    0x0040,     /* R80  - Left Input Volume */ \
    0x0040,     /* R81  - Right Input Volume */ \
    0x0000,     /* R82 */ \
    0x0000,     /* R83 */ \
    0x0000,     /* R84 */ \
    0x0000,     /* R85 */ \
    0x0000,     /* R86 */ \
    0x0000,     /* R87 */ \
    0x0800,     /* R88  - Left Mixer Control */ \
    0x1000,     /* R89  - Right Mixer Control */ \
    0x0000,     /* R90 */ \
    0x0000,     /* R91 */ \
    0x0000,     /* R92  - OUT3 Mixer Control */ \
    0x0000,     /* R93  - OUT4 Mixer Control */ \
    0x0000,     /* R94 */ \
    0x0000,     /* R95 */ \
    0x0000,     /* R96  - Output Left Mixer Volume */ \
    0x0000,     /* R97  - Output Right Mixer Volume */ \
    0x0000,     /* R98  - Input Mixer Volume L */ \
    0x0000,     /* R99  - Input Mixer Volume R */ \
    0x0000,     /* R100 - Input Mixer Volume */ \
    0x0000,     /* R101 */ \
    0x0000,     /* R102 */ \
    0x0000,     /* R103 */ \
    0x00E4,     /* R104 - LOUT1 Volume */ \
    0x00E4,     /* R105 - ROUT1 Volume */ \
    0x00E4,     /* R106 - LOUT2 Volume */ \
    0x02E4,     /* R107 - ROUT2 Volume */ \
    0x0000,     /* R108 */ \
    0x0000,     /* R109 */ \
    0x0000,     /* R110 */ \
    0x0000,     /* R111 - BEEP Volume */ \
    0x0A00,     /* R112 - AI Formating */ \
    0x0000,     /* R113 - ADC DAC COMP */ \
    0x0020,     /* R114 - AI ADC Control */ \
    0x0020,     /* R115 - AI DAC Control */ \
    0x0000,     /* R116 - AIF Test */ \
    0x0000,     /* R117 */ \
    0x0000,     /* R118 */ \
    0x0000,     /* R119 */ \
    0x0000,     /* R120 */ \
    0x0000,     /* R121 */ \
    0x0000,     /* R122 */ \
    0x0000,     /* R123 */ \
    0x0000,     /* R124 */ \
    0x0000,     /* R125 */ \
    0x0000,     /* R126 */ \
    0x0000,     /* R127 */ \
    0x1FFF,     /* R128 - GPIO Debounce */ \
    0x0000,     /* R129 - GPIO Pin pull up Control */ \
    0x03FC,     /* R130 - GPIO Pull down Control */ \
    0x0000,     /* R131 - GPIO Interrupt Mode */ \
    0x0000,     /* R132 */ \
    0x0000,     /* R133 - GPIO Control */ \
    0x08FB,     /* R134 - GPIO Configuration (i/o) */ \
    0x0CFE,     /* R135 - GPIO Pin Polarity / Type */ \
    0x0000,     /* R136 */ \
    0x0000,     /* R137 */ \
    0x0000,     /* R138 */ \
    0x0000,     /* R139 */ \
    0x0312,     /* R140 - GPIO Function Select 1 */ \
    0x0003,     /* R141 - GPIO Function Select 2 */ \
    0x2331,     /* R142 - GPIO Function Select 3 */ \
    0x0003,     /* R143 - GPIO Function Select 4 */ \
    0x0000,     /* R144 - Digitiser Control (1) */ \
    0x0002,     /* R145 - Digitiser Control (2) */ \
    0x0000,     /* R146 */ \
    0x0000,     /* R147 */ \
    0x0000,     /* R148 */ \
    0x0000,     /* R149 */ \
    0x0000,     /* R150 */ \
    0x0000,     /* R151 */ \
    0x7000,     /* R152 - AUX1 Readback */ \
    0x7000,     /* R153 - AUX2 Readback */ \
    0x7000,     /* R154 - AUX3 Readback */ \
    0x7000,     /* R155 - AUX4 Readback */ \
    0x0000,     /* R156 - USB Voltage Readback */ \
    0x0000,     /* R157 - LINE Voltage Readback */ \
    0x0000,     /* R158 - BATT Voltage Readback */ \
    0x0000,     /* R159 - Chip Temp Readback */ \
    0x0000,     /* R160 */ \
    0x0000,     /* R161 */ \
    0x0000,     /* R162 */ \
    0x0000,     /* R163 - Generic Comparator Control */ \
    0x0000,     /* R164 - Generic comparator 1 */ \
    0x0000,     /* R165 - Generic comparator 2 */ \
    0x0000,     /* R166 - Generic comparator 3 */ \
    0x0000,     /* R167 - Generic comparator 4 */ \
    0xA00F,     /* R168 - Battery Charger Control 1 */ \
    0x0B06,     /* R169 - Battery Charger Control 2 */ \
    0x0000,     /* R170 - Battery Charger Control 3 */ \
    0x0000,     /* R171 */ \
    0x0000,     /* R172 - Current Sink Driver A */ \
    0x0000,     /* R173 - CSA Flash control */ \
    0x0000,     /* R174 - Current Sink Driver B */ \
    0x0000,     /* R175 - CSB Flash control */ \
    0x0000,     /* R176 - DCDC/LDO requested */ \
    0x002D,     /* R177 - DCDC Active options */ \
    0x0000,     /* R178 - DCDC Sleep options */ \
    0x0025,     /* R179 - Power-check comparator */ \
    0x000E,     /* R180 - DCDC1 Control */ \
    0x0400,     /* R181 - DCDC1 Timeouts */ \
    0x1006,     /* R182 - DCDC1 Low Power */ \
    0x0018,     /* R183 - DCDC2 Control */ \
    0x0000,     /* R184 - DCDC2 Timeouts */ \
    0x0000,     /* R185 */ \
    0x002E,     /* R186 - DCDC3 Control */ \
    0x0800,     /* R187 - DCDC3 Timeouts */ \
    0x0006,     /* R188 - DCDC3 Low Power */ \
    0x000E,     /* R189 - DCDC4 Control */ \
    0x0800,     /* R190 - DCDC4 Timeouts */ \
    0x0006,     /* R191 - DCDC4 Low Power */ \
    0x0008,     /* R192 - DCDC5 Control */ \
    0x0000,     /* R193 - DCDC5 Timeouts */ \
    0x0000,     /* R194 */ \
    0x0026,     /* R195 - DCDC6 Control */ \
    0x0C00,     /* R196 - DCDC6 Timeouts */ \
    0x0006,     /* R197 - DCDC6 Low Power */ \
    0x0000,     /* R198 */ \
    0x0003,     /* R199 - Limit Switch Control */ \
    0x001A,     /* R200 - LDO1 Control */ \
    0x0800,     /* R201 - LDO1 Timeouts */ \
    0x001C,     /* R202 - LDO1 Low Power */ \
    0x0010,     /* R203 - LDO2 Control */ \
    0x0800,     /* R204 - LDO2 Timeouts */ \
    0x001C,     /* R205 - LDO2 Low Power */ \
    0x000A,     /* R206 - LDO3 Control */ \
    0x0C00,     /* R207 - LDO3 Timeouts */ \
    0x001C,     /* R208 - LDO3 Low Power */ \
    0x001A,     /* R209 - LDO4 Control */ \
    0x0800,     /* R210 - LDO4 Timeouts */ \
    0x001C,     /* R211 - LDO4 Low Power */ \
    0x0000,     /* R212 */ \
    0x0000,     /* R213 */ \
    0x0000,     /* R214 */ \
    0x0000,     /* R215 - VCC_FAULT Masks */ \
    0x001F,     /* R216 - Main Bandgap Control */ \
    0x0000,     /* R217 - OSC Control */ \
    0x9000,     /* R218 - RTC Tick Control */ \
    0x0000,     /* R219 */ \
    0x4000,     /* R220 - RAM BIST 1 */ \
    0x0000,     /* R221 */ \
    0x0000,     /* R222 */ \
    0x0000,     /* R223 */ \
    0x0000,     /* R224 */ \
    0x0000,     /* R225 - DCDC/LDO status */ \
    0x0000,     /* R226 */ \
    0x0000,     /* R227 */ \
    0x0000,     /* R228 */ \
    0x0000,     /* R229 */ \
    0xE000,     /* R230 - GPIO Pin Status */ \
    0x0000,     /* R231 */ \
    0x0000,     /* R232 */ \
    0x0000,     /* R233 */ \
}

/* Bank 3 */
#define WM8350_REGISTER_DEFAULTS_3 \
{ \
    0x17FF,     /* R0   - Reset/ID */ \
    0x1000,     /* R1   - ID */ \
    0x0000,     /* R2 */ \
    0x1000,     /* R3   - System Control 1 */ \
    0x0004,     /* R4   - System Control 2 */ \
    0x0000,     /* R5   - System Hibernate */ \
    0x8A00,     /* R6   - Interface Control */ \
    0x0000,     /* R7 */ \
    0x8000,     /* R8   - Power mgmt (1) */ \
    0x0000,     /* R9   - Power mgmt (2) */ \
    0x0000,     /* R10  - Power mgmt (3) */ \
    0x2000,     /* R11  - Power mgmt (4) */ \
    0x0E00,     /* R12  - Power mgmt (5) */ \
    0x0000,     /* R13  - Power mgmt (6) */ \
    0x0000,     /* R14  - Power mgmt (7) */ \
    0x0000,     /* R15 */ \
    0x0000,     /* R16  - RTC Seconds/Minutes */ \
    0x0100,     /* R17  - RTC Hours/Day */ \
    0x0101,     /* R18  - RTC Date/Month */ \
    0x1400,     /* R19  - RTC Year */ \
    0x0000,     /* R20  - Alarm Seconds/Minutes */ \
    0x0000,     /* R21  - Alarm Hours/Day */ \
    0x0000,     /* R22  - Alarm Date/Month */ \
    0x0320,     /* R23  - RTC Time Control */ \
    0x0000,     /* R24  - System Interrupts */ \
    0x0000,     /* R25  - Interrupt Status 1 */ \
    0x0000,     /* R26  - Interrupt Status 2 */ \
    0x0000,     /* R27  - Power Up Interrupt Status */ \
    0x0000,     /* R28  - Under Voltage Interrupt status */ \
    0x0000,     /* R29  - Over Current Interrupt status */ \
    0x0000,     /* R30  - GPIO Interrupt Status */ \
    0x0000,     /* R31  - Comparator Interrupt Status */ \
    0x3FFF,     /* R32  - System Interrupts Mask */ \
    0x0000,     /* R33  - Interrupt Status 1 Mask */ \
    0x0000,     /* R34  - Interrupt Status 2 Mask */ \
    0x0000,     /* R35  - Power Up Interrupt Status Mask */ \
    0x0000,     /* R36  - Under Voltage Interrupt status Mask */ \
    0x0000,     /* R37  - Over Current Interrupt status Mask */ \
    0x0000,     /* R38  - GPIO Interrupt Status Mask */ \
    0x0000,     /* R39  - Comparator Interrupt Status Mask */ \
    0x0040,     /* R40  - Clock Control 1 */ \
    0x0000,     /* R41  - Clock Control 2 */ \
    0x3B00,     /* R42  - FLL Control 1 */ \
    0x7086,     /* R43  - FLL Control 2 */ \
    0xC226,     /* R44  - FLL Control 3 */ \
    0x0000,     /* R45  - FLL Control 4 */ \
    0x0000,     /* R46 */ \
    0x0000,     /* R47 */ \
    0x0000,     /* R48  - DAC Control */ \
    0x0000,     /* R49 */ \
    0x00C0,     /* R50  - DAC Digital Volume L */ \
    0x00C0,     /* R51  - DAC Digital Volume R */ \
    0x0000,     /* R52 */ \
    0x0040,     /* R53  - DAC LR Rate */ \
    0x0000,     /* R54  - DAC Clock Control */ \
    0x0000,     /* R55 */ \
    0x0000,     /* R56 */ \
    0x0000,     /* R57 */ \
    0x4000,     /* R58  - DAC Mute */ \
    0x0000,     /* R59  - DAC Mute Volume */ \
    0x0000,     /* R60  - DAC Side */ \
    0x0000,     /* R61 */ \
    0x0000,     /* R62 */ \
    0x0000,     /* R63 */ \
    0x8000,     /* R64  - ADC Control */ \
    0x0000,     /* R65 */ \
    0x00C0,     /* R66  - ADC Digital Volume L */ \
    0x00C0,     /* R67  - ADC Digital Volume R */ \
    0x0000,     /* R68  - ADC Divider */ \
    0x0000,     /* R69 */ \
    0x0040,     /* R70  - ADC LR Rate */ \
    0x0000,     /* R71 */ \
    0x0303,     /* R72  - Input Control */ \
    0x0000,     /* R73  - IN3 Input Control */ \
    0x0000,     /* R74  - Mic Bias Control */ \
    0x0000,     /* R75 */ \
    0x0000,     /* R76  - Output Control */ \
    0x0000,     /* R77  - Jack Detect */ \
    0x0000,     /* R78  - Anti Pop Control */ \
    0x0000,     /* R79 */ \
    0x0040,     /* R80  - Left Input Volume */ \
    0x0040,     /* R81  - Right Input Volume */ \
    0x0000,     /* R82 */ \
    0x0000,     /* R83 */ \
    0x0000,     /* R84 */ \
    0x0000,     /* R85 */ \
    0x0000,     /* R86 */ \
    0x0000,     /* R87 */ \
    0x0800,     /* R88  - Left Mixer Control */ \
    0x1000,     /* R89  - Right Mixer Control */ \
    0x0000,     /* R90 */ \
    0x0000,     /* R91 */ \
    0x0000,     /* R92  - OUT3 Mixer Control */ \
    0x0000,     /* R93  - OUT4 Mixer Control */ \
    0x0000,     /* R94 */ \
    0x0000,     /* R95 */ \
    0x0000,     /* R96  - Output Left Mixer Volume */ \
    0x0000,     /* R97  - Output Right Mixer Volume */ \
    0x0000,     /* R98  - Input Mixer Volume L */ \
    0x0000,     /* R99  - Input Mixer Volume R */ \
    0x0000,     /* R100 - Input Mixer Volume */ \
    0x0000,     /* R101 */ \
    0x0000,     /* R102 */ \
    0x0000,     /* R103 */ \
    0x00E4,     /* R104 - LOUT1 Volume */ \
    0x00E4,     /* R105 - ROUT1 Volume */ \
    0x00E4,     /* R106 - LOUT2 Volume */ \
    0x02E4,     /* R107 - ROUT2 Volume */ \
    0x0000,     /* R108 */ \
    0x0000,     /* R109 */ \
    0x0000,     /* R110 */ \
    0x0000,     /* R111 - BEEP Volume */ \
    0x0A00,     /* R112 - AI Formating */ \
    0x0000,     /* R113 - ADC DAC COMP */ \
    0x0020,     /* R114 - AI ADC Control */ \
    0x0020,     /* R115 - AI DAC Control */ \
    0x0000,     /* R116 - AIF Test */ \
    0x0000,     /* R117 */ \
    0x0000,     /* R118 */ \
    0x0000,     /* R119 */ \
    0x0000,     /* R120 */ \
    0x0000,     /* R121 */ \
    0x0000,     /* R122 */ \
    0x0000,     /* R123 */ \
    0x0000,     /* R124 */ \
    0x0000,     /* R125 */ \
    0x0000,     /* R126 */ \
    0x0000,     /* R127 */ \
    0x1FFF,     /* R128 - GPIO Debounce */ \
    0x0000,     /* R129 - GPIO Pin pull up Control */ \
    0x03FC,     /* R130 - GPIO Pull down Control */ \
    0x0000,     /* R131 - GPIO Interrupt Mode */ \
    0x0000,     /* R132 */ \
    0x0000,     /* R133 - GPIO Control */ \
    0x0A7B,     /* R134 - GPIO Configuration (i/o) */ \
    0x06FE,     /* R135 - GPIO Pin Polarity / Type */ \
    0x0000,     /* R136 */ \
    0x0000,     /* R137 */ \
    0x0000,     /* R138 */ \
    0x0000,     /* R139 */ \
    0x1312,     /* R140 - GPIO Function Select 1 */ \
    0x1030,     /* R141 - GPIO Function Select 2 */ \
    0x2231,     /* R142 - GPIO Function Select 3 */ \
    0x0003,     /* R143 - GPIO Function Select 4 */ \
    0x0000,     /* R144 - Digitiser Control (1) */ \
    0x0002,     /* R145 - Digitiser Control (2) */ \
    0x0000,     /* R146 */ \
    0x0000,     /* R147 */ \
    0x0000,     /* R148 */ \
    0x0000,     /* R149 */ \
    0x0000,     /* R150 */ \
    0x0000,     /* R151 */ \
    0x7000,     /* R152 - AUX1 Readback */ \
    0x7000,     /* R153 - AUX2 Readback */ \
    0x7000,     /* R154 - AUX3 Readback */ \
    0x7000,     /* R155 - AUX4 Readback */ \
    0x0000,     /* R156 - USB Voltage Readback */ \
    0x0000,     /* R157 - LINE Voltage Readback */ \
    0x0000,     /* R158 - BATT Voltage Readback */ \
    0x0000,     /* R159 - Chip Temp Readback */ \
    0x0000,     /* R160 */ \
    0x0000,     /* R161 */ \
    0x0000,     /* R162 */ \
    0x0000,     /* R163 - Generic Comparator Control */ \
    0x0000,     /* R164 - Generic comparator 1 */ \
    0x0000,     /* R165 - Generic comparator 2 */ \
    0x0000,     /* R166 - Generic comparator 3 */ \
    0x0000,     /* R167 - Generic comparator 4 */ \
    0xA00F,     /* R168 - Battery Charger Control 1 */ \
    0x0B06,     /* R169 - Battery Charger Control 2 */ \
    0x0000,     /* R170 - Battery Charger Control 3 */ \
    0x0000,     /* R171 */ \
    0x0000,     /* R172 - Current Sink Driver A */ \
    0x0000,     /* R173 - CSA Flash control */ \
    0x0000,     /* R174 - Current Sink Driver B */ \
    0x0000,     /* R175 - CSB Flash control */ \
    0x0000,     /* R176 - DCDC/LDO requested */ \
    0x002D,     /* R177 - DCDC Active options */ \
    0x0000,     /* R178 - DCDC Sleep options */ \
    0x0025,     /* R179 - Power-check comparator */ \
    0x000E,     /* R180 - DCDC1 Control */ \
    0x0400,     /* R181 - DCDC1 Timeouts */ \
    0x1006,     /* R182 - DCDC1 Low Power */ \
    0x0018,     /* R183 - DCDC2 Control */ \
    0x0000,     /* R184 - DCDC2 Timeouts */ \
    0x0000,     /* R185 */ \
    0x000E,     /* R186 - DCDC3 Control */ \
    0x0400,     /* R187 - DCDC3 Timeouts */ \
    0x0006,     /* R188 - DCDC3 Low Power */ \
    0x0026,     /* R189 - DCDC4 Control */ \
    0x0400,     /* R190 - DCDC4 Timeouts */ \
    0x0006,     /* R191 - DCDC4 Low Power */ \
    0x0008,     /* R192 - DCDC5 Control */ \
    0x0000,     /* R193 - DCDC5 Timeouts */ \
    0x0000,     /* R194 */ \
    0x0026,     /* R195 - DCDC6 Control */ \
    0x0400,     /* R196 - DCDC6 Timeouts */ \
    0x0006,     /* R197 - DCDC6 Low Power */ \
    0x0000,     /* R198 */ \
    0x0003,     /* R199 - Limit Switch Control */ \
    0x001C,     /* R200 - LDO1 Control */ \
    0x0000,     /* R201 - LDO1 Timeouts */ \
    0x001C,     /* R202 - LDO1 Low Power */ \
    0x001C,     /* R203 - LDO2 Control */ \
    0x0400,     /* R204 - LDO2 Timeouts */ \
    0x001C,     /* R205 - LDO2 Low Power */ \
    0x001C,     /* R206 - LDO3 Control */ \
    0x0400,     /* R207 - LDO3 Timeouts */ \
    0x001C,     /* R208 - LDO3 Low Power */ \
    0x001F,     /* R209 - LDO4 Control */ \
    0x0400,     /* R210 - LDO4 Timeouts */ \
    0x001C,     /* R211 - LDO4 Low Power */ \
    0x0000,     /* R212 */ \
    0x0000,     /* R213 */ \
    0x0000,     /* R214 */ \
    0x0000,     /* R215 - VCC_FAULT Masks */ \
    0x001F,     /* R216 - Main Bandgap Control */ \
    0x0000,     /* R217 - OSC Control */ \
    0x9000,     /* R218 - RTC Tick Control */ \
    0x0000,     /* R219 */ \
    0x4000,     /* R220 - RAM BIST 1 */ \
    0x0000,     /* R221 */ \
    0x0000,     /* R222 */ \
    0x0000,     /* R223 */ \
    0x0000,     /* R224 */ \
    0x0000,     /* R225 - DCDC/LDO status */ \
    0x0000,     /* R226 */ \
    0x0000,     /* R227 */ \
    0x0000,     /* R228 */ \
    0x0000,     /* R229 */ \
    0xE000,     /* R230 - GPIO Pin Status */ \
    0x0000,     /* R231 */ \
    0x0000,     /* R232 */ \
    0x0000,     /* R233 */ \
}

/*
 * Access masks.
 */

struct wm8350_reg_access {
    u16  readable;   /* Mask of readable bits */
    u16  writable;   /* Mask of writable bits */
    u16  vol;        /* Mask of volatile bits */
};

#define WM8350_ACCESS \
{  /*  read    write volatile */ \
    { 0xFFFF, 0xFFFF, 0xFFFF }, /* R0   - Reset/ID */ \
    { 0x7CFF, 0x0C00, 0x7FFF }, /* R1   - ID */ \
    { 0x0000, 0x0000, 0x0000 }, /* R2 */ \
    { 0xBE3B, 0xBE3B, 0x8000 }, /* R3   - System Control 1 */ \
    { 0xFCF7, 0xFCF7, 0xF800 }, /* R4   - System Control 2 */ \
    { 0x80FF, 0x80FF, 0x8000 }, /* R5   - System Hibernate */ \
    { 0xFB0E, 0xFB0E, 0x0000 }, /* R6   - Interface Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R7 */ \
    { 0xE537, 0xE537, 0x0000 }, /* R8   - Power mgmt (1) */ \
    { 0x0FF3, 0x0FF3, 0x0000 }, /* R9   - Power mgmt (2) */ \
    { 0x008F, 0x008F, 0x0000 }, /* R10  - Power mgmt (3) */ \
    { 0x6D3C, 0x6D3C, 0x0000 }, /* R11  - Power mgmt (4) */ \
    { 0x1F8F, 0x1F8F, 0x0000 }, /* R12  - Power mgmt (5) */ \
    { 0x8F3F, 0x8F3F, 0x8F3F }, /* R13  - Power mgmt (6) */ \
    { 0x0003, 0x0003, 0x0000 }, /* R14  - Power mgmt (7) */ \
    { 0x0000, 0x0000, 0x0000 }, /* R15 */ \
    { 0x7F7F, 0x7F7F, 0xFFFF }, /* R16  - RTC Seconds/Minutes */ \
    { 0x073F, 0x073F, 0xFFFF }, /* R17  - RTC Hours/Day */ \
    { 0x1F3F, 0x1F3F, 0xFFFF }, /* R18  - RTC Date/Month */ \
    { 0x3FFF, 0x00FF, 0xFFFF }, /* R19  - RTC Year */ \
    { 0x7F7F, 0x7F7F, 0x0000 }, /* R20  - Alarm Seconds/Minutes */ \
    { 0x0F3F, 0x0F3F, 0x0000 }, /* R21  - Alarm Hours/Day */ \
    { 0x1F3F, 0x1F3F, 0x0000 }, /* R22  - Alarm Date/Month */ \
    { 0xEF7F, 0xEA7F, 0xFFFF }, /* R23  - RTC Time Control */ \
    { 0x3BFF, 0x0000, 0xFFFF }, /* R24  - System Interrupts */ \
    { 0xFEE7, 0x0000, 0xFFFF }, /* R25  - Interrupt Status 1 */ \
    { 0x35FF, 0x0000, 0xFFFF }, /* R26  - Interrupt Status 2 */ \
    { 0x0F3F, 0x0000, 0xFFFF }, /* R27  - Power Up Interrupt Status */ \
    { 0x0F3F, 0x0000, 0xFFFF }, /* R28  - Under Voltage Interrupt status */ \
    { 0x8000, 0x0000, 0xFFFF }, /* R29  - Over Current Interrupt status */ \
    { 0x1FFF, 0x0000, 0xFFFF }, /* R30  - GPIO Interrupt Status */ \
    { 0xEF7F, 0x0000, 0xFFFF }, /* R31  - Comparator Interrupt Status */ \
    { 0x3FFF, 0x3FFF, 0x0000 }, /* R32  - System Interrupts Mask */ \
    { 0xFEE7, 0xFEE7, 0x0000 }, /* R33  - Interrupt Status 1 Mask */ \
    { 0xF5FF, 0xF5FF, 0x0000 }, /* R34  - Interrupt Status 2 Mask */ \
    { 0x0F3F, 0x0F3F, 0x0000 }, /* R35  - Power Up Interrupt Status Mask */ \
    { 0x0F3F, 0x0F3F, 0x0000 }, /* R36  - Under Voltage Interrupt status Mask */ \
    { 0x8000, 0x8000, 0x0000 }, /* R37  - Over Current Interrupt status Mask */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R38  - GPIO Interrupt Status Mask */ \
    { 0xEF7F, 0xEF7F, 0x0000 }, /* R39  - Comparator Interrupt Status Mask */ \
    { 0xC9F7, 0xC9F7, 0xFFFF }, /* R40  - Clock Control 1 */ \
    { 0x8001, 0x8001, 0x0000 }, /* R41  - Clock Control 2 */ \
    { 0xFFF7, 0xFFF7, 0xFFFF }, /* R42  - FLL Control 1 */ \
    { 0xFBFF, 0xFBFF, 0x0000 }, /* R43  - FLL Control 2 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R44  - FLL Control 3 */ \
    { 0x0033, 0x0033, 0x0000 }, /* R45  - FLL Control 4 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R46 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R47 */ \
    { 0x3033, 0x3033, 0x0000 }, /* R48  - DAC Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R49 */ \
    { 0x81FF, 0x81FF, 0xFF00 }, /* R50  - DAC Digital Volume L */ \
    { 0x81FF, 0x81FF, 0xFF00 }, /* R51  - DAC Digital Volume R */ \
    { 0x0000, 0x0000, 0x0000 }, /* R52 */ \
    { 0x0FFF, 0x0FFF, 0x0000 }, /* R53  - DAC LR Rate */ \
    { 0x0017, 0x0017, 0x0000 }, /* R54  - DAC Clock Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R55 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R56 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R57 */ \
    { 0x4000, 0x4000, 0x0000 }, /* R58  - DAC Mute */ \
    { 0x7000, 0x7000, 0x0000 }, /* R59  - DAC Mute Volume */ \
    { 0x3C00, 0x3C00, 0x0000 }, /* R60  - DAC Side */ \
    { 0x0000, 0x0000, 0x0000 }, /* R61 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R62 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R63 */ \
    { 0x8303, 0x8303, 0xFFFF }, /* R64  - ADC Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R65 */ \
    { 0x81FF, 0x81FF, 0xFF00 }, /* R66  - ADC Digital Volume L */ \
    { 0x81FF, 0x81FF, 0xFF00 }, /* R67  - ADC Digital Volume R */ \
    { 0x0FFF, 0x0FFF, 0x0000 }, /* R68  - ADC Divider */ \
    { 0x0000, 0x0000, 0x0000 }, /* R69 */ \
    { 0x0FFF, 0x0FFF, 0x0000 }, /* R70  - ADC LR Rate */ \
    { 0x0000, 0x0000, 0x0000 }, /* R71 */ \
    { 0x0707, 0x0707, 0x0000 }, /* R72  - Input Control */ \
    { 0xC0C0, 0xC0C0, 0x8000 }, /* R73  - IN3 Input Control */ \
    { 0xC09F, 0xC09F, 0x8000 }, /* R74  - Mic Bias Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R75 */ \
    { 0x0F15, 0x0F15, 0xFFFF }, /* R76  - Output Control */ \
    { 0xC000, 0xC000, 0x0000 }, /* R77  - Jack Detect */ \
    { 0x03FF, 0x03FF, 0x0000 }, /* R78  - Anti Pop Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R79 */ \
    { 0xE1FC, 0xE1FC, 0x8000 }, /* R80  - Left Input Volume */ \
    { 0xE1FC, 0xE1FC, 0x8000 }, /* R81  - Right Input Volume */ \
    { 0x0000, 0x0000, 0x0000 }, /* R82 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R83 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R84 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R85 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R86 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R87 */ \
    { 0x9807, 0x9807, 0x8000 }, /* R88  - Left Mixer Control */ \
    { 0x980B, 0x980B, 0x8000 }, /* R89  - Right Mixer Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R90 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R91 */ \
    { 0x8909, 0x8909, 0x8000 }, /* R92  - OUT3 Mixer Control */ \
    { 0x9E07, 0x9E07, 0x8000 }, /* R93  - OUT4 Mixer Control */ \
    { 0x0000, 0x0000, 0x0000 }, /* R94 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R95 */ \
    { 0x0EEE, 0x0EEE, 0x0000 }, /* R96  - Output Left Mixer Volume */ \
    { 0xE0EE, 0xE0EE, 0x0000 }, /* R97  - Output Right Mixer Volume */ \
    { 0x0E0F, 0x0E0F, 0x0000 }, /* R98  - Input Mixer Volume L */ \
    { 0xE0E1, 0xE0E1, 0x0000 }, /* R99  - Input Mixer Volume R */ \
    { 0x800E, 0x800E, 0x0000 }, /* R100 - Input Mixer Volume */ \
    { 0x0000, 0x0000, 0x0000 }, /* R101 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R102 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R103 */ \
    { 0xE1FC, 0xE1FC, 0x9f03 }, /* R104 - LOUT1 Volume */ \
    { 0xE1FC, 0xE1FC, 0x9f03 }, /* R105 - ROUT1 Volume */ \
    { 0xE1FC, 0xE1FC, 0x9f03 }, /* R106 - LOUT2 Volume */ \
    { 0xE7FC, 0xE7FC, 0x9703 }, /* R107 - ROUT2 Volume */ \
    { 0x0000, 0x0000, 0x0000 }, /* R108 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R109 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R110 */ \
    { 0x80E0, 0x80E0, 0xFFFF }, /* R111 - BEEP Volume */ \
    { 0xBF00, 0xBF00, 0x0000 }, /* R112 - AI Formating */ \
    { 0x00F1, 0x00F1, 0x0000 }, /* R113 - ADC DAC COMP */ \
    { 0x00F8, 0x00F8, 0x0000 }, /* R114 - AI ADC Control */ \
    { 0x40FB, 0x40FB, 0x0000 }, /* R115 - AI DAC Control */ \
    { 0x7C30, 0x7C30, 0x0000 }, /* R116 - AIF Test */ \
    { 0x0000, 0x0000, 0x0000 }, /* R117 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R118 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R119 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R120 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R121 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R122 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R123 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R124 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R125 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R126 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R127 */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R128 - GPIO Debounce */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R129 - GPIO Pin pull up Control */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R130 - GPIO Pull down Control */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R131 - GPIO Interrupt Mode */ \
    { 0x0000, 0x0000, 0x0000 }, /* R132 */ \
    { 0x00C0, 0x00C0, 0x0000 }, /* R133 - GPIO Control */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R134 - GPIO Configuration (i/o) */ \
    { 0x1FFF, 0x1FFF, 0x0000 }, /* R135 - GPIO Pin Polarity / Type */ \
    { 0x0000, 0x0000, 0x0000 }, /* R136 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R137 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R138 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R139 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R140 - GPIO Function Select 1 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R141 - GPIO Function Select 2 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R142 - GPIO Function Select 3 */ \
    { 0x000F, 0x000F, 0x0000 }, /* R143 - GPIO Function Select 4 */ \
    { 0xF0FF, 0xF0FF, 0xA000 }, /* R144 - Digitiser Control (1) */ \
    { 0x3707, 0x3707, 0x0000 }, /* R145 - Digitiser Control (2) */ \
    { 0x0000, 0x0000, 0x0000 }, /* R146 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R147 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R148 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R149 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R150 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R151 */ \
    { 0x7FFF, 0x7000, 0xFFFF }, /* R152 - AUX1 Readback */ \
    { 0x7FFF, 0x7000, 0xFFFF }, /* R153 - AUX2 Readback */ \
    { 0x7FFF, 0x7000, 0xFFFF }, /* R154 - AUX3 Readback */ \
    { 0x7FFF, 0x7000, 0xFFFF }, /* R155 - AUX4 Readback */ \
    { 0x0FFF, 0x0000, 0xFFFF }, /* R156 - USB Voltage Readback */ \
    { 0x0FFF, 0x0000, 0xFFFF }, /* R157 - LINE Voltage Readback */ \
    { 0x0FFF, 0x0000, 0xFFFF }, /* R158 - BATT Voltage Readback */ \
    { 0x0FFF, 0x0000, 0xFFFF }, /* R159 - Chip Temp Readback */ \
    { 0x0000, 0x0000, 0x0000 }, /* R160 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R161 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R162 */ \
    { 0x000F, 0x000F, 0x0000 }, /* R163 - Generic Comparator Control */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R164 - Generic comparator 1 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R165 - Generic comparator 2 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R166 - Generic comparator 3 */ \
    { 0xFFFF, 0xFFFF, 0x0000 }, /* R167 - Generic comparator 4 */ \
    { 0xBFFF, 0xBFFF, 0x8000 }, /* R168 - Battery Charger Control 1 */ \
    { 0xFFFF, 0x4FFF, 0xB000 }, /* R169 - Battery Charger Control 2 */ \
    { 0x007F, 0x007F, 0x0000 }, /* R170 - Battery Charger Control 3 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R171 */ \
    { 0x903F, 0x903F, 0x8000 }, /* R172 - Current Sink Driver A */ \
    { 0xE333, 0xE333, 0xFFFF }, /* R173 - CSA Flash control */ \
    { 0x903F, 0x903F, 0x8000 }, /* R174 - Current Sink Driver B */ \
    { 0xE333, 0xE333, 0xFFFF }, /* R175 - CSB Flash control */ \
    { 0x8F3F, 0x8F3F, 0x8F3F }, /* R176 - DCDC/LDO requested */ \
    { 0x332D, 0x332D, 0x0000 }, /* R177 - DCDC Active options */ \
    { 0x002D, 0x002D, 0x0000 }, /* R178 - DCDC Sleep options */ \
    { 0x5177, 0x5177, 0x8000 }, /* R179 - Power-check comparator */ \
    { 0x047F, 0x047F, 0x0000 }, /* R180 - DCDC1 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R181 - DCDC1 Timeouts */ \
    { 0x737F, 0x737F, 0x0000 }, /* R182 - DCDC1 Low Power */ \
    { 0x535B, 0x535B, 0x0000 }, /* R183 - DCDC2 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R184 - DCDC2 Timeouts */ \
    { 0x0000, 0x0000, 0x0000 }, /* R185 */ \
    { 0x047F, 0x047F, 0x0000 }, /* R186 - DCDC3 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R187 - DCDC3 Timeouts */ \
    { 0x737F, 0x737F, 0x0000 }, /* R188 - DCDC3 Low Power */ \
    { 0x047F, 0x047F, 0x0000 }, /* R189 - DCDC4 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R190 - DCDC4 Timeouts */ \
    { 0x737F, 0x737F, 0x0000 }, /* R191 - DCDC4 Low Power */ \
    { 0x535B, 0x535B, 0x0000 }, /* R192 - DCDC5 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R193 - DCDC5 Timeouts */ \
    { 0x0000, 0x0000, 0x0000 }, /* R194 */ \
    { 0x047F, 0x047F, 0x0000 }, /* R195 - DCDC6 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R196 - DCDC6 Timeouts */ \
    { 0x737F, 0x737F, 0x0000 }, /* R197 - DCDC6 Low Power */ \
    { 0x0000, 0x0000, 0x0000 }, /* R198 */ \
    { 0xFFD3, 0xFFD3, 0x0000 }, /* R199 - Limit Switch Control */ \
    { 0x441F, 0x441F, 0x0000 }, /* R200 - LDO1 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R201 - LDO1 Timeouts */ \
    { 0x331F, 0x331F, 0x0000 }, /* R202 - LDO1 Low Power */ \
    { 0x441F, 0x441F, 0x0000 }, /* R203 - LDO2 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R204 - LDO2 Timeouts */ \
    { 0x331F, 0x331F, 0x0000 }, /* R205 - LDO2 Low Power */ \
    { 0x441F, 0x441F, 0x0000 }, /* R206 - LDO3 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R207 - LDO3 Timeouts */ \
    { 0x331F, 0x331F, 0x0000 }, /* R208 - LDO3 Low Power */ \
    { 0x441F, 0x441F, 0x0000 }, /* R209 - LDO4 Control */ \
    { 0xFFC0, 0xFFC0, 0x0000 }, /* R210 - LDO4 Timeouts */ \
    { 0x331F, 0x331F, 0x0000 }, /* R211 - LDO4 Low Power */ \
    { 0x0000, 0x0000, 0x0000 }, /* R212 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R213 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R214 */ \
    { 0x8F3F, 0x8F3F, 0x0000 }, /* R215 - VCC_FAULT Masks */ \
    { 0xFF3F, 0xE03F, 0x0000 }, /* R216 - Main Bandgap Control */ \
    { 0xEF2F, 0xE02F, 0x0000 }, /* R217 - OSC Control */ \
    { 0xF3FF, 0xB3FF, 0xc000 }, /* R218 - RTC Tick Control */ \
    { 0xFFFF, 0xFFFF, 0xFFFF }, /* R219 */ \
    { 0x09FF, 0x01FF, 0x0000 }, /* R220 - RAM BIST 1 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R221 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R222 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R223 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R224 */ \
    { 0x8F3F, 0x0000, 0x0000 }, /* R225 - DCDC/LDO status */ \
    { 0x0000, 0x0000, 0x0000 }, /* R226 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R227 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R228 */ \
    { 0x0000, 0x0000, 0x0000 }, /* R229 */ \
    { 0xFFFF, 0x1FFF, 0xFFFF }, /* R230 - GPIO Pin Status */ \
    { 0xFFFF, 0x1FFF, 0xFFFF }, /* R231 */ \
    { 0xFFFF, 0x1FFF, 0xFFFF }, /* R232 */ \
    { 0xFFFF, 0x1FFF, 0xFFFF }, /* R233 */ \
}

/*
 * Need to split this up per driver.
 */

#define WM8350_IO_I2C       0
#define WM8350_IO_SPI       1
#define WM8350_IO_CUSTOM    2

/*
 * IRQ & sources
 */

/* first level IRQ's - used by bus driver internally only */


/* number of 2nd level or real IRQ's */
#define WM8350_NUM_IRQ	79

#define WM8350_IRQ_CHG_BAT_HOT		0
#define WM8350_IRQ_CHG_BAT_COLD		1
#define WM8350_IRQ_CHG_BAT_FAIL		2
#define WM8350_IRQ_CHG_TO		3
#define WM8350_IRQ_CHG_END		4
#define WM8350_IRQ_CHG_START		5
#define WM8350_IRQ_CHG_FAST_RDY		6
#define WM8350_IRQ_RTC_PER		7
#define WM8350_IRQ_RTC_SEC		8
#define WM8350_IRQ_RTC_ALM		9
#define WM8350_IRQ_CHG_VBATT_LT_3P9		10
#define WM8350_IRQ_CHG_VBATT_LT_3P1		11
#define WM8350_IRQ_CHG_VBATT_LT_2P85		12
#define WM8350_IRQ_CS1		13
#define WM8350_IRQ_CS2		14
#define WM8350_IRQ_USB_LIMIT		15
#define WM8350_IRQ_AUXADC_DATARDY		16
#define WM8350_IRQ_AUXADC_DCOMP4		17
#define WM8350_IRQ_AUXADC_DCOMP3		18
#define WM8350_IRQ_AUXADC_DCOMP2		19
#define WM8350_IRQ_AUXADC_DCOMP1		20
#define WM8350_IRQ_SYS_HYST_COMP_FAIL		21
#define WM8350_IRQ_SYS_CHIP_GT115		22
#define WM8350_IRQ_SYS_CHIP_GT140		23
#define WM8350_IRQ_SYS_WDOG_TO		24
#define WM8350_IRQ_UV_LDO4		25
#define WM8350_IRQ_UV_LDO3		26
#define WM8350_IRQ_UV_LDO2		27
#define WM8350_IRQ_UV_LDO1		28
#define WM8350_IRQ_UV_DC6		29
#define WM8350_IRQ_UV_DC5		30
#define WM8350_IRQ_UV_DC4		31
#define WM8350_IRQ_UV_DC3		32
#define WM8350_IRQ_UV_DC2		33
#define WM8350_IRQ_UV_DC1		34
#define WM8350_IRQ_OC_LS		35
#define WM8350_IRQ_EXT_USB_FB		36
#define WM8350_IRQ_EXT_WALL_FB		37
#define WM8350_IRQ_EXT_BAT_FB		38
#define WM8350_IRQ_CODEC_JCK_DET_L		39
#define WM8350_IRQ_CODEC_JCK_DET_R		40
#define WM8350_IRQ_CODEC_MICSCD		41
#define WM8350_IRQ_CODEC_MICD		42
#define WM8350_IRQ_WKUP_OFF_STATE		43
#define WM8350_IRQ_WKUP_HIB_STATE		44
#define WM8350_IRQ_WKUP_CONV_FAULT		45
#define WM8350_IRQ_WKUP_WDOG_RST		46
#define WM8350_IRQ_WKUP_GP_PWR_ON		47
#define WM8350_IRQ_WKUP_ONKEY		48
#define WM8350_IRQ_WKUP_GP_WAKEUP		49
#define WM8350_IRQ_GPIO(x)		(WM8350_IRQ_WKUP_GP_WAKEUP + x + 1)

/*
 * DCDC's
 */
#define WM8350_DCDC_1	1
#define WM8350_DCDC_2	2
#define WM8350_DCDC_3	3
#define WM8350_DCDC_4	4
#define WM8350_DCDC_5	5
#define WM8350_DCDC_6	6

/* DCDC modes */
#define WM8350_DCDC_ACTIVE_STANDBY	0
#define WM8350_DCDC_ACTIVE_PULSE	1
#define WM8350_DCDC_SLEEP_NORMAL	0
#define WM8350_DCDC_SLEEP_LOW	1

/* DCDC Low power (Hibernate) mode */
#define WM8350_DCDC_HIB_MODE_CUR	(0 << 12)
#define WM8350_DCDC_HIB_MODE_IMAGE	(1 << 12)
#define WM8350_DCDC_HIB_MODE_STANDBY	(2 << 12)
#define WM8350_DCDC_HIB_MODE_LDO	(4 << 12)
#define WM8350_DCDC_HIB_MODE_LDO_IM	(5 << 12)
#define WM8350_DCDC_HIB_MODE_DIS	(7 << 12)

/* DCDC Low Power (Hibernate) signal */
#define WM8350_DCDC_HIB_SIG_REG	(0 << 8)
#define WM8350_DCDC_HIB_SIG_LPWR1	(1 << 8)
#define WM8350_DCDC_HIB_SIG_LPWR2	(2 << 8)
#define WM8350_DCDC_HIB_SIG_LPWR3	(3 << 8)

/* LDO Low power (Hibernate) mode */
#define WM8350_LDO_HIB_MODE_IMAGE	(0 << 0)
#define WM8350_LDO_HIB_MODE_DIS	(1 << 0)

/* LDO Low Power (Hibernate) signal */
#define WM8350_LDO_HIB_SIG_REG	(0 << 8)
#define WM8350_LDO_HIB_SIG_LPWR1	(1 << 8)
#define WM8350_LDO_HIB_SIG_LPWR2	(2 << 8)
#define WM8350_LDO_HIB_SIG_LPWR3	(3 << 8)

/*
 * LDO's
 */
#define WM8350_LDO_1	7
#define WM8350_LDO_2	8
#define WM8350_LDO_3	9
#define WM8350_LDO_4	10

/*
 * ISINK's
 */
#define WM8350_ISINK_A	0
#define WM8350_ISINK_B	1

#define WM8350_ISINK_MODE_BOOST		0
#define WM8350_ISINK_MODE_SWITCH	1
#define WM8350_ISINK_ILIM_NORMAL	0
#define WM8350_ISINK_ILIM_LOW		1

#define WM8350_ISINK_FLASH_DISABLE	0
#define WM8350_ISINK_FLASH_ENABLE	1
#define WM8350_ISINK_FLASH_TRIG_BIT	0
#define WM8350_ISINK_FLASH_TRIG_GPIO	1
#define WM8350_ISINK_FLASH_MODE_EN	(1 << 13)
#define WM8350_ISINK_FLASH_MODE_DIS	(0 << 13)
#define WM8350_ISINK_FLASH_DUR_32MS	(0 << 8)
#define WM8350_ISINK_FLASH_DUR_64MS	(1 << 8)
#define WM8350_ISINK_FLASH_DUR_96MS	(2 << 8)
#define WM8350_ISINK_FLASH_DUR_1024MS	(3 << 8)
#define WM8350_ISINK_FLASH_ON_INSTANT	(0 << 4)
#define WM8350_ISINK_FLASH_ON_0_25S	(1 << 4)
#define WM8350_ISINK_FLASH_ON_0_50S	(2 << 4)
#define WM8350_ISINK_FLASH_ON_1_00S	(3 << 4)
#define WM8350_ISINK_FLASH_ON_1_95S	(1 << 4)
#define WM8350_ISINK_FLASH_ON_3_91S	(2 << 4)
#define WM8350_ISINK_FLASH_ON_7_80S	(3 << 4)
#define WM8350_ISINK_FLASH_OFF_INSTANT	(0 << 0)
#define WM8350_ISINK_FLASH_OFF_0_25S	(1 << 0)
#define WM8350_ISINK_FLASH_OFF_0_50S	(2 << 0)
#define WM8350_ISINK_FLASH_OFF_1_00S	(3 << 0)
#define WM8350_ISINK_FLASH_OFF_1_95S	(1 << 0)
#define WM8350_ISINK_FLASH_OFF_3_91S	(2 << 0)
#define WM8350_ISINK_FLASH_OFF_7_80S	(3 << 0)

/*
 * Clock sources & directions
 */
#define WM3850_MCLK_SEL_PLL_MCLK 0
#define WM3850_MCLK_SEL_PLL_DAC 1
#define WM3850_MCLK_SEL_PLL_ADC 2
#define WM3850_MCLK_SEL_PLL_32K 3
#define WM3850_MCLK_SEL_MCLK	5

#define WM3850_MCLK_DIR_OUT 	0
#define WM3850_MCLK_DIR_IN 	1

/* clock divider id's */
#define WM8350_ADC_CLKDIV	0
#define WM8350_DAC_CLKDIV	1
#define WM8350_BCLK_CLKDIV	2
#define WM8350_OPCLK_CLKDIV	3
#define WM8350_TO_CLKDIV	4
#define WM8350_SYS_CLKDIV	5
#define WM8350_DACLR_CLKDIV	6
#define WM8350_ADCLR_CLKDIV	7

/* ADC clock dividers */
#define WM8350_ADCDIV_1	0x0
#define WM8350_ADCDIV_1_5	0x1
#define WM8350_ADCDIV_2	0x2
#define WM8350_ADCDIV_3	0x3
#define WM8350_ADCDIV_4	0x4
#define WM8350_ADCDIV_5_5	0x5
#define WM8350_ADCDIV_6	0x6

/* ADC clock dividers */
#define WM8350_DACDIV_1	0x0
#define WM8350_DACDIV_1_5	0x1
#define WM8350_DACDIV_2	0x2
#define WM8350_DACDIV_3	0x3
#define WM8350_DACDIV_4	0x4
#define WM8350_DACDIV_5_5	0x5
#define WM8350_DACDIV_6	0x6

/* BCLK clock dividers */
#define WM8350_BCLK_DIV_1	(0x0 << 4)
#define WM8350_BCLK_DIV_1_5	(0x1 << 4)
#define WM8350_BCLK_DIV_2	(0x2 << 4)
#define WM8350_BCLK_DIV_3	(0x3 << 4)
#define WM8350_BCLK_DIV_4	(0x4 << 4)
#define WM8350_BCLK_DIV_5_5	(0x5 << 4)
#define WM8350_BCLK_DIV_6	(0x6 << 4)
#define WM8350_BCLK_DIV_8	(0x7 << 4)
#define WM8350_BCLK_DIV_11	(0x8 << 4)
#define WM8350_BCLK_DIV_12	(0x9 << 4)
#define WM8350_BCLK_DIV_16	(0xa << 4)
#define WM8350_BCLK_DIV_22	(0xb << 4)
#define WM8350_BCLK_DIV_24	(0xc << 4)
#define WM8350_BCLK_DIV_32	(0xd << 4)
#define WM8350_BCLK_DIV_44	(0xe << 4)
#define WM8350_BCLK_DIV_48	(0xf << 4)

/* Sys (M) clock dividers */
#define WM8350_MCLK_DIV_1	(0x0 << 8)
#define WM8350_MCLK_DIV_2	(0x1 << 8)

/* OP clock dividers */
#define WM8350_OPCLK_DIV_1	0x0
#define WM8350_OPCLK_DIV_2	0x1
#define WM8350_OPCLK_DIV_3	0x2
#define WM8350_OPCLK_DIV_4	0x3
#define WM8350_OPCLK_DIV_5_5	0x4
#define WM8350_OPCLK_DIV_6	0x5

/*
 * GPIO Functions
 */
#define WM8350_GPIO0_GPIO_IN		0x0
#define WM8350_GPIO0_GPIO_OUT		0x0
#define WM8350_GPIO0_PWR_ON_IN		0x1
#define WM8350_GPIO0_PWR_ON_OUT		0x1
#define WM8350_GPIO0_LDO_EN_IN		0x2
#define WM8350_GPIO0_VRTC_OUT		0x2
#define WM8350_GPIO0_LPWR1_IN		0x3
#define WM8350_GPIO0_POR_B_OUT		0x3

#define WM8350_GPIO1_GPIO_IN		0x0
#define WM8350_GPIO1_GPIO_OUT		0x0
#define WM8350_GPIO1_PWR_ON_IN		0x1
#define WM8350_GPIO1_DO_CONF_OUT	0x1
#define WM8350_GPIO1_LDO_EN_IN		0x2
#define WM8350_GPIO1_RESET_OUT		0x2
#define WM8350_GPIO1_LPWR2_IN		0x3
#define WM8350_GPIO1_MEMRST_OUT		0x3

#define WM8350_GPIO2_GPIO_IN		0x0
#define WM8350_GPIO2_GPIO_OUT		0x0
#define WM8350_GPIO2_PWR_ON_IN		0x1
#define WM8350_GPIO2_PWR_ON_OUT		0x1
#define WM8350_GPIO2_WAKE_UP_IN		0x2
#define WM8350_GPIO2_VRTC_OUT		0x2
#define WM8350_GPIO2_32KHZ_IN		0x3
#define WM8350_GPIO2_32KHZ_OUT		0x3

#define WM8350_GPIO3_GPIO_IN		0x0
#define WM8350_GPIO3_GPIO_OUT		0x0
#define WM8350_GPIO3_PWR_ON_IN		0x1
#define WM8350_GPIO3_P_CLK_OUT		0x1
#define WM8350_GPIO3_LDO_EN_IN		0x2
#define WM8350_GPIO3_VRTC_OUT		0x2
#define WM8350_GPIO3_PWR_OFF_IN		0x3
#define WM8350_GPIO3_32KHZ_OUT		0x3

#define WM8350_GPIO4_GPIO_IN		0x0
#define WM8350_GPIO4_GPIO_OUT		0x0
#define WM8350_GPIO4_MR_IN		0x1
#define WM8350_GPIO4_MEM_RST_OUT	0x1
#define WM8350_GPIO4_FLASH_IN		0x2
#define WM8350_GPIO4_ADA_OUT		0x2
#define WM8350_GPIO4_HIBERNATE_IN	0x3
#define WM8350_GPIO4_FLASH_OUT		0x3
#define WM8350_GPIO4_MICDET_OUT		0x4
#define WM8350_GPIO4_MICSHT_OUT		0x5

#define WM8350_GPIO5_GPIO_IN		0x0
#define WM8350_GPIO5_GPIO_OUT		0x0
#define WM8350_GPIO5_LPWR1_IN		0x1
#define WM8350_GPIO5_P_CLK_OUT		0x1
#define WM8350_GPIO5_ADCLRCLK_IN	0x2
#define WM8350_GPIO5_ADCLRCLK_OUT	0x2
#define WM8350_GPIO5_HIBERNATE_IN	0x3
#define WM8350_GPIO5_32KHZ_OUT		0x3
#define WM8350_GPIO5_MICDET_OUT		0x4
#define WM8350_GPIO5_MICSHT_OUT		0x5
#define WM8350_GPIO5_ADA_OUT		0x6
#define WM8350_GPIO5_OPCLK_OUT		0x7

#define WM8350_GPIO6_GPIO_IN		0x0
#define WM8350_GPIO6_GPIO_OUT		0x0
#define WM8350_GPIO6_LPWR2_IN		0x1
#define WM8350_GPIO6_MEMRST_OUT		0x1
#define WM8350_GPIO6_FLASH_IN		0x2
#define WM8350_GPIO6_ADA_OUT		0x2
#define WM8350_GPIO6_HIBERNATE_IN	0x3
#define WM8350_GPIO6_RTC_OUT		0x3
#define WM8350_GPIO6_MICDET_OUT		0x4
#define WM8350_GPIO6_MICSHT_OUT		0x5
#define WM8350_GPIO6_ADCLRCLKB_OUT	0x6
#define WM8350_GPIO6_SDOUT_OUT		0x7

#define WM8350_GPIO7_GPIO_IN		0x0
#define WM8350_GPIO7_GPIO_OUT		0x0
#define WM8350_GPIO7_LPWR3_IN		0x1
#define WM8350_GPIO7_P_CLK_OUT		0x1
#define WM8350_GPIO7_MASK_IN		0x2
#define WM8350_GPIO7_VCC_FAULT_OUT	0x2
#define WM8350_GPIO7_HIBERNATE_IN	0x3
#define WM8350_GPIO7_BATT_FAULT_OUT	0x3
#define WM8350_GPIO7_MICDET_OUT		0x4
#define WM8350_GPIO7_MICSHT_OUT		0x5
#define WM8350_GPIO7_ADA_OUT		0x6
#define WM8350_GPIO7_CSB_IN		0x7

#define WM8350_GPIO8_GPIO_IN		0x0
#define WM8350_GPIO8_GPIO_OUT		0x0
#define WM8350_GPIO8_MR_IN		0x1
#define WM8350_GPIO8_VCC_FAULT_OUT	0x1
#define WM8350_GPIO8_ADCBCLK_IN		0x2
#define WM8350_GPIO8_ADCBCLK_OUT	0x2
#define WM8350_GPIO8_PWR_OFF_IN		0x3
#define WM8350_GPIO8_BATT_FAULT_OUT	0x3
#define WM8350_GPIO8_ALTSCL_IN		0xf

#define WM8350_GPIO9_GPIO_IN		0x0
#define WM8350_GPIO9_GPIO_OUT		0x0
#define WM8350_GPIO9_HEARTBEAT_IN	0x1
#define WM8350_GPIO9_VCC_FAULT_OUT	0x1
#define WM8350_GPIO9_MASK_IN		0x2
#define WM8350_GPIO9_LINE_GT_BATT_OUT	0x2
#define WM8350_GPIO9_PWR_OFF_IN		0x3
#define WM8350_GPIO9_BATT_FAULT_OUT	0x3
#define WM8350_GPIO9_ALTSDA_OUT		0xf

#define WM8350_GPIO10_GPIO_IN		0x0
#define WM8350_GPIO10_GPIO_OUT		0x0
#define WM8350_GPIO10_ISINKC_OUT	0x1
#define WM8350_GPIO10_PWR_OFF_IN	0x2
#define WM8350_GPIO10_LINE_GT_BATT_OUT	0x2
#define WM8350_GPIO10_CHD_IND_IN	0x3

#define WM8350_GPIO11_GPIO_IN		0x0
#define WM8350_GPIO11_GPIO_OUT		0x0
#define WM8350_GPIO11_ISINKD_OUT	0x1
#define WM8350_GPIO11_WAKEUP_IN		0x2
#define WM8350_GPIO11_LINE_GT_BATT_OUT	0x2
#define WM8350_GPIO11_CHD_IND_IN	0x3

#define WM8350_GPIO12_GPIO_IN		0x0
#define WM8350_GPIO12_GPIO_OUT		0x0
#define WM8350_GPIO12_ISINKE_OUT	0x1
#define WM8350_GPIO12_LINE_GT_BATT_OUT	0x2
#define WM8350_GPIO12_LINE_EN_OUT	0x3
#define WM8350_GPIO12_32KHZ_OUT		0x4

#define WM8350_GPIO_DIR_IN	0
#define WM8350_GPIO_DIR_OUT	1
#define WM8350_GPIO_ACTIVE_LOW	0
#define WM8350_GPIO_ACTIVE_HIGH	1
#define WM8350_GPIO_PULL_NONE	0
#define WM8350_GPIO_PULL_UP	1
#define WM8350_GPIO_PULL_DOWN	2
#define WM8350_GPIO_INVERT_OFF	0
#define WM8350_GPIO_INVERT_ON	1
#define WM8350_GPIO_DEBOUNCE_OFF	0
#define WM8350_GPIO_DEBOUNCE_ON	1

/*
 * USB/2, LINE & BATT = ((VRTC * 2) / 4095)) * 10e6 uV
 * Where VRTC = 2.7 V
 */
#define WM8350_AUX_COEFF 1319

#define WM8350_AUXADC_AUX1	0
#define WM8350_AUXADC_AUX2	1
#define WM8350_AUXADC_AUX3	2
#define WM8350_AUXADC_AUX4	3
#define WM8350_AUXADC_USB	4
#define WM8350_AUXADC_LINE	5
#define WM8350_AUXADC_BATT	6
#define WM8350_AUXADC_TEMP	7

/*
 * Charger Policy
 */
#define WM8350_CHG_TIME_MIN(x)	((((x - 30) / 15) & 0xf) << 8)
#define WM8350_CHG_TRICKLE_50mA		(0 << 6)
#define WM8350_CHG_TRICKLE_100mA		(1 << 6)
#define WM8350_CHG_4_05V		(0 << 4)
#define WM8350_CHG_4_10V		(1 << 4)
#define WM8350_CHG_4_15V		(2 << 4)
#define WM8350_CHG_4_20V		(3 << 4)
#define WM8350_CHG_FAST_LIMIT_mA(x)	((x / 50) & 0xf)
#define WM8350_CHG_EOC_mA(x)		(((x - 10) & 0x7) << 10)
#define WM8350_CHG_TRICKLE_3_1V		(0 << 13)
#define WM8350_CHG_TRICKLE_3_9V		(1 << 13)


extern struct bus_type wm8350_bus_type;

struct wm8350_pmic {
	struct device dev;
};
#define to_wm8350_pmic_device(d) container_of(d, struct wm8350_pmic, dev)

struct wm8350_backlight {
	struct device dev;
	struct backlight_properties props;
	struct work_struct work;
	struct mutex mutex;
	int intensity;
	int suspend;
	int dcdc; /* DCDC2 or DCDC5 */
	int isink; /* ISINK A or ISINK B */
	int retries;
};
#define to_wm8350_backlight_device(d) \
	container_of(d, struct wm8350_backlight, dev)

struct wm8350_wdg {
	struct device dev;
};
#define to_wm8350_wdg_device(d) container_of(d, struct wm8350_wdg, dev)

struct wm8350_led {
	struct device dev;
	struct work_struct work;
	struct mutex mutex;
	enum led_brightness value;
	int dcdc; /* DCDC2 or DCDC5 */
	int isink; /* ISINK A or ISINK B */
	int retries;
	int half_value;
	int full_value;
	int current_value;
};
#define to_wm8350_led_device(d) container_of(d, struct wm8350_led, dev)

struct wm8350_rtc {
	struct device dev;
	struct rtc_device *rtc;
	int pie_freq;
	int pie_enabled;
	int update_enabled;
	int alarm_enabled;
	int per_irq; /* platform specific period irq */
};
#define to_wm8350_rtc_device(d) container_of(d, struct wm8350_rtc, dev)

struct wm8350;
typedef int (*wm8350_hw_read_t)(struct wm8350 *wm8350, char reg, int size, char *dest);
typedef int (*wm8350_hw_write_t)(struct wm8350 *wm8350, char reg, int size, char *src);

struct wm8350_charger_policy {
	struct mutex mutex;
	int enable;
	int fast_enable;
	
	/* charger state machine policy  - set in machine driver or by sysfs */
	int eoc_mA; /* end of charge current (mA)  */
	int charge_mV; /* charge voltage */
	int fast_limit_mA; /* fast charge current limit */
	int charge_timeout; /* charge timeout (mins) */
	int trickle_start_mV; /* trickle charge starts at mV */
	int trickle_charge_mA; /* trickle charge current */
};

struct wm8350_power {
	struct device dev;
	struct power_supply battery;
	struct power_supply usb;
	struct power_supply ac;
	struct wm8350_charger_policy policy;
};
#define to_wm8350_power_device(d) container_of(d, struct wm8350_power, dev)
#define to_wm8350_battery_supply(d) container_of(d, struct wm8350_power, battery)
#define to_wm8350_usb_supply(d) container_of(d, struct wm8350_power, usb)
#define to_wm8350_ac_supply(d) container_of(d, struct wm8350_power, ac)

struct wm8350 {
	/* device IO */
	unsigned short i2c_address;
	struct i2c_client *i2c_client;
	struct spi_device *spi_device;
	wm8350_hw_read_t read_dev;
	wm8350_hw_write_t write_dev;
	u16 *reg_cache;

	/* clients */
	struct wm8350_pmic pmic;
	struct wm8350_wdg wdg;
	struct wm8350_led led;
	struct wm8350_rtc rtc;
	struct snd_soc_machine audio;
	struct wm8350_power power;
	struct wm8350_backlight backlight;
	
	/* irq handlers and workq */
	int nirq;
	struct work_struct work;
	struct mutex work_mutex;
	void (*handler[WM8350_NUM_IRQ])(struct wm8350 *, int);
};
#define to_wm8350_from_pmic(d) container_of(d, struct wm8350, pmic)
#define to_wm8350_from_led(d) container_of(d, struct wm8350, led)
#define to_wm8350_from_wdg(d) container_of(d, struct wm8350, wdg)
#define to_wm8350_from_rtc(d) container_of(d, struct wm8350, rtc)
#define to_wm8350_from_audio(d) container_of(d, struct wm8350, audio)
#define to_wm8350_from_power(d) container_of(d, struct wm8350, power)
#define to_wm8350_from_backlight(d) container_of(d, struct wm8350, backlight)

/*
 * WM8350 device IO
 */
int wm8350_clear_bits(struct wm8350 *wm8350, u16 reg, u16 mask);
int wm8350_set_bits(struct wm8350 *wm8350, u16 reg, u16 mask);
u16 wm8350_reg_read(struct wm8350 *wm8350, int reg);
int wm8350_reg_write(struct wm8350 *wm8350, int reg, u16 val);
int wm8350_set_io(struct wm8350 *wm8350, int io, wm8350_hw_read_t read_dev,
	wm8350_hw_write_t write_dev);
int wm8350_reg_lock(struct wm8350 *wm8350);
int wm8350_reg_unlock(struct wm8350 *wm8350);
int wm8350_block_read(struct wm8350 *wm8350, int reg, int size,
	u16 *dest);
int wm8350_block_write(struct wm8350 *wm8350, int reg, int size,
	u16 *src);
int wm8350_create_cache(struct wm8350 *wm8350);

/*
 * Client device registration.
 */
int wm8350_device_register_wdg(struct wm8350 *wm8350);
int wm8350_device_register_pmic(struct wm8350 *wm8350);
int wm8350_device_register_rtc(struct wm8350 *wm8350);
int wm8350_device_register_led(struct wm8350 *wm8350);
int wm8350_device_register_power(struct wm8350 *wm8350);
int wm8350_device_register_backlight(struct wm8350 *wm8350);

/*
 * IRQ
 */
int wm8350_register_irq(struct wm8350 *wm8350, int irq,
	void (*handler)(struct wm8350 *, int));
int wm8350_free_irq(struct wm8350 *wm8350, int irq);
int wm8350_mask_irq(struct wm8350 *wm8350, int irq);
int wm8350_unmask_irq(struct wm8350 *wm8350, int irq);
void wm8350_irq_worker(struct work_struct *work);

/*
 * GPIO
 */
int wm8350_gpio_set_status(struct wm8350 *wm8350, int gpio, int status);
int wm8350_gpio_get_status(struct wm8350 *wm8350, int gpio);
int wm8350_gpio_config(struct wm8350 *wm8350, int gpio, int dir, int func,
	int pol, int pull, int invert, int debounce);

/*
 * DCDC control
 */
int wm8350_dcdc_enable(struct wm8350_pmic *pmic, int dcdc, int enable);
int wm8350_dcdc_set_voltage(struct wm8350_pmic *pmic, int dcdc, int mV);
int wm8350_dcdc_set_mode(struct wm8350_pmic *pmic, int dcdc, u16 active, 
	u16 sleep);
int wm8350_dcdc_set_slot(struct wm8350_pmic *pmic, int dcdc, u16 start, 
	u16 stop, u16 fault);
int wm8350_dcdc25_set_mode(struct wm8350_pmic *pmic, int dcdc, u16 mode, 
	u16 ilim, u16 ramp, u16 feedback);
int wm8350_dcdc_set_image_voltage(struct wm8350_pmic *pmic, int dcdc, int mV,
	int mode, int signal);

/*
 * LDO control
 */
int wm8350_ldo_enable(struct wm8350_pmic *pmic, int ldo, int enable);
int wm8350_ldo_set_slot(struct wm8350_pmic *pmic, int ldo, u16 start, 
	u16 stop);
int wm8350_ldo_set_voltage(struct wm8350_pmic *pmic, int ldo, int mV);
int wm8350_ldo_set_image_voltage(struct wm8350_pmic *pmic, int ldo, int mV,
	int mode, int signal);

/*
 * ISINK control
 */
int wm8350_isink_trigger_flash(struct wm8350_pmic *pmic, int isink);
int wm8350_isink_set_flash(struct wm8350_pmic *pmic, int isink, u16 mode, 
	u16 trigger, u16 duration, u16 on_ramp, u16 off_ramp, u16 drive);
int wm8350_isink_enable(struct wm8350_pmic *pmic, int isink, int enable, 
	int hibernate_enable);
int wm8350_isink_set_current(struct wm8350_pmic *pmic, int isink, 
	int huA);
/*
 * AUX ADC Readback
 */
int wm8350_read_auxadc(struct wm8350 *wm8350, int channel, int scale, int vref);

#define wm8350_read_battery_uvolts(wm8350) \
	(wm8350_read_auxadc(wm8350, WM8350_AUXADC_BATT, 0, 0) * WM8350_AUX_COEFF)
#define wm8350_read_battery_mvolts(wm8350) \
	(wm8350_read_battery_uvolts(wm8350 / 1000)
	
#define wm8350_read_line_uvolts(wm8350) \
	(wm8350_read_auxadc(wm8350, WM8350_AUXADC_LINE, 0, 0) * WM8350_AUX_COEFF)
#define wm8350_read_line_mvolts(wm8350) \
	(wm8350_read_line_uvolts(wm8350 / 1000)
	
#define wm8350_read_usb_uvolts(wm8350) \
	(wm8350_read_auxadc(wm8350, WM8350_AUXADC_USB, 0, 0) * WM8350_AUX_COEFF)
#define wm8350_read_usb_mvolts(wm8350) \
	(wm8350_read_usb_uvolts(wm8350 / 1000)

#define wm8350_read_temp(wm8350) \
	wm8350_read_auxadc(wm8350, WM8350_AUXADC_TEMP, 0, 0)
	
/*
 * Charger
 */
int wm8350_charger_enable(struct wm8350_power *power, int enable);
int wm8350_fast_charger_enable(struct wm8350_power *power, int enable);

#endif	/* __LINUX_PMIC_WM8350_H */
