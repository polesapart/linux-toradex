/*
 * Copyright 2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __ASM_ARCH_MXC_MX33_H__
#define __ASM_ARCH_MXC_MX33_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

#include <asm/arch/mx33_pins.h>

/*!
 * defines the OS clock tick rate
 */
#define CLOCK_TICK_RATE         16625000

#define ATA_BASE_CLK            (IPG_CLK)
/*!
 * Register an interrupt handler for the SMN as well as the SCC.  In some
 * implementations, the SMN is not connected at all, and in others, it is
 * on the same interrupt line as the SCM. Comment this line out accordingly
 */
#define USE_SMN_INTERRUPT

/*
 * UART Chip level Configuration that a user may not have to edit. These
 * configuration vary depending on how the UART module is integrated with
 * the ARM core
 */
#define MXC_UART_NR 5
/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive Irda data.
 */
#define MXC_UART_IR_RXDMUX      0x0004
/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive UART data.
 */
#define MXC_UART_RXDMUX         0x0004

/*
 * IRAM
 */
#define IRAM_BASE_ADDR		0x10000000	/* internal ram */
#define IRAM_BASE_ADDR_VIRT	0xE0000000
#define IRAM_SIZE		SZ_128K

/*
 * L2CC
 */
#define L2CC_BASE_ADDR		0xB0000000
#define L2CC_BASE_ADDR_VIRT	0xE1000000
#define L2CC_SIZE		SZ_1M

#define PLATFORM_BASE_ADDR	0xB0400000
#define PLATFORM_BASE_ADDR_VIRT 0xE1400000
#define PLATFORM_SIZE		SZ_1M
#define EVTMON_BASE_ADDR	(PLATFORM_BASE_ADDR + 0x00000000)
#define ARM1176_BASE_ADDR	(PLATFORM_BASE_ADDR + 0x00002000)

#define TZIC_BASE_ADDR		0xB0800000
#define TZIC_BASE_ADDR_VIRT	0xE1800000
#define TZIC_SIZE		SZ_1M

#define DEBUG_BASE_ADDR		0xB0C00000
#define DEBUG_BASE_ADDR_VIRT	0xE1C00000
#define DEBUG_SIZE		SZ_1M
#define ETB_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00001000)
#define ETM_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00002000)
#define TPIU_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00003000)
#define CTI0_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00004000)
#define CTI1_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00005000)
#define CTI2_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00006000)
#define CTI3_BASE_ADDR		(DEBUG_BASE_ADDR + 0x00007000)

/*
 * SPBA global module enabled #0
 */
#define SPBA0_BASE_ADDR 	0xC0000000
#define SPBA0_BASE_ADDR_VIRT	0xE4000000
#define SPBA0_SIZE		SZ_1M

#define MMC_SDHC1_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00004000)
#define MMC_SDHC2_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00008000)
#define UART3_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x0000C000)
#define CSPI2_BASE_ADDR 	(SPBA0_BASE_ADDR + 0x00010000)
#define SSI2_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00014000)
#define MMC_SDHC3_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00020000)
#define MARC_BASE_ADDR		(SPBA0_BASE_ADDR + 0x0002C000)
#define ATA_DMA_BASE_ADDR	(SPBA0_BASE_ADDR + 0x00034000)
#define FEC_BASE_ADDR		(SPBA0_BASE_ADDR + 0x00038000)
#define SPBA_CTRL_BASE_ADDR	(SPBA0_BASE_ADDR + 0x0003C000)
/*
 * AIPS 1
 */
#define AIPS1_BASE_ADDR 	0xC3F00000
#define AIPS1_BASE_ADDR_VIRT	0xE4100000
#define AIPS1_SIZE		SZ_1M

#define MAX_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00080000)
#define GPIO1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00084000)
#define GPIO2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00088000)
#define GPIO3_BASE_ADDR		(AIPS1_BASE_ADDR + 0x0008C000)
#define GPIO4_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00090000)
#define KPP_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00094000)
#define WDOG1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00098000)
#define WDOG2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x0009C000)
#define GPT1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A0000)
#define SRTC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000A4000)
#define IOMUXC_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000A8000)
#define IIM_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000AC000)
#define CSU_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000B0000)
#define SDMA_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000B4000)
#define SAHARA_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000B8000)
#define SCC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000BC000)
#define ROMCP_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000C0000)
#define RTIC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000C4000)
#define MU_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000C8000)
#define SDDC_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000CC000)
#define VPU_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000D0000)
#define USBOH2_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000D4000)
#define ATA_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000D8000)
#define SIM1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000DC000)
#define MSHC1_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000E0000)
#define MSHC2_BASE_ADDR		(AIPS1_BASE_ADDR + 0x000E4000)

/*!
 * defines for SPBA modules
 */

#define SPBA_SDHC1	0x04
#define SPBA_SDHC2	0x08
#define SPBA_UART3	0x0C
#define SPBA_CSPI2	0x10
#define SPBA_SSI2	0x14
#define SPBA_SDHC3	0x20
#define SPBA_MARC	0x2C
#define SPBA_ATA	0x34
#define SPBA_FEC	0x38

/*!
 * Defines for modules using static and dynamic DMA channels
 */
#define MXC_DMA_CHANNEL_UART1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART3_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART4_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART4_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART5_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_UART5_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC1  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MMC2  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_SSI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_FIR_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_FIR_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI1_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_CSPI2_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_RX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_ATA_TX  MXC_DMA_DYNAMIC_CHANNEL
#define MXC_DMA_CHANNEL_MEMORY  MXC_DMA_DYNAMIC_CHANNEL

/*
 * AIPS 2
 */
#define AIPS2_BASE_ADDR		0xE3F00000
#define AIPS2_BASE_ADDR_VIRT	0xE4800000
#define AIPS2_SIZE		SZ_1M

#define DPLLIP1_BASE_ADDR	(AIPS2_BASE_ADDR + 0x00080000)
#define DPLLIP2_BASE_ADDR	(AIPS2_BASE_ADDR + 0x00084000)
#define DPLLIP3_BASE_ADDR	(AIPS2_BASE_ADDR + 0x00088000)
#define CCM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x0008C000)
#define GPC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00090000)
#define SRC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00094000)
#define EPIT1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x00098000)
#define EPIT2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x0009C000)
#define PWM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000A0000)
#define OWIRE_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000A4000)
#define CSPI3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000A8000)
#define CSPI1_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000AC000)
#define UART1_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B0000)
#define UART4_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B4000)
#define UART5_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000B8000)
#define UART2_BASE_ADDR 	(AIPS2_BASE_ADDR + 0x000BC000)
#define I2C3_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000C0000)
#define I2C2_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000C4000)
#define I2C_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000C8000)
#define SSI1_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000CC000)
#define AUDMUX_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000D0000)
#define EMI_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D8000)
#define HSC_CTL_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000DC000)

#if 0
#define PCMCIA_CTL_BASE_ADDR	EMI_CTL_BASE_ADDR
#define FIRI_BASE_ADDR		(AIPS2_BASE_ADDR + 0x0008C000)
#define SCM_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000AE000)
#define SMN_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000AF000)
#define RNGA_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000B0000)
#define IPU_CTRL_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C0000)
#define MPEG4_ENC_BASE_ADDR	(AIPS2_BASE_ADDR + 0x000C8000)
#define RTC_BASE_ADDR		(AIPS2_BASE_ADDR + 0x000D8000)

#define ETB_SLOT4_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00010000)
#define ETB_SLOT5_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00014000)
#define ECT_CTIO_BASE_ADDR	(AIPS1_BASE_ADDR + 0x00018000)
#define OTG_BASE_ADDR		(AIPS1_BASE_ADDR + 0x00088000)
#define ECT_IP1_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000B8000)
#define ECT_IP2_BASE_ADDR	(AIPS1_BASE_ADDR + 0x000BC000)

#endif

/*
 * NAND, SDRAM, WEIM, M3IF, EMI controllers
 */
#define M4IF_BASE_ADDR		(EMI_BASE_ADDR + 0x0000)
#define ESDCTL_BASE_ADDR	(EMI_BASE_ADDR + 0x1000)
#define WEIM_BASE_ADDR		(EMI_BASE_ADDR + 0x2000)
#define NFC_BASE_ADDR		(EMI_BASE_ADDR + 0x3000)
#define EMI_CTL_BASE_ADDR	(EMI_BASE_ADDR + 0x3F00)

/*
 * Memory regions and CS
 */
/* MX33 ADS SDRAM is from CSD1, 64M */
#define CSD0_BASE_ADDR          0x40000000
#define CSD1_BASE_ADDR          0x50000000

#define CS0_BASE_ADDR           0x60000000
#define CS1_BASE_ADDR           0x68000000
#define CS2_BASE_ADDR           0x70000000
#define CS3_BASE_ADDR           0x72000000
#define CS4_BASE_ADDR           0x74000000
#define CS4_BASE_ADDR_VIRT      0xEB000000
#define CS4_SIZE                SZ_16M
#define CS5_BASE_ADDR           0x76000000
#define SDRAM_BASE_ADDR         0x50000000

#define IPU_MEM_BASE_ADDR       0x80000000
#define GPU_MEM_BASE_ADDR       0xA0000000

#define PCMCIA_MEM_BASE_ADDR    0xBC000000

/*!
 * This macro defines the physical to virtual address mapping for all the
 * peripheral modules. It is used by passing in the physical address as x
 * and returning the virtual address. If the physical address is not mapped,
 * it returns 0xEEADBEEF
 */
#define IO_ADDRESS(x)   \
        (((x >= IRAM_BASE_ADDR) && (x < (IRAM_BASE_ADDR + IRAM_SIZE))) ? IRAM_IO_ADDRESS(x):\
        ((x >= CS4_BASE_ADDR) && (x < (CS4_BASE_ADDR + CS4_SIZE))) ? CS4_IO_ADDRESS(x):\
        ((x >= L2CC_BASE_ADDR) && (x < (L2CC_BASE_ADDR + L2CC_SIZE))) ? L2CC_IO_ADDRESS(x):\
        ((x >= PLATFORM_BASE_ADDR) && (x < (PLATFORM_BASE_ADDR + PLATFORM_SIZE))) ? PLATFORM_IO_ADDRESS(x):\
        ((x >= TZIC_BASE_ADDR) && (x < (TZIC_BASE_ADDR + TZIC_SIZE))) ? TZIC_IO_ADDRESS(x):\
        ((x >= DEBUG_BASE_ADDR) && (x < (DEBUG_BASE_ADDR + DEBUG_SIZE))) ? DEBUG_IO_ADDRESS(x):\
        ((x >= SPBA0_BASE_ADDR) && (x < (SPBA0_BASE_ADDR + SPBA0_SIZE))) ? SPBA0_IO_ADDRESS(x):\
        ((x >= AIPS1_BASE_ADDR) && (x < (AIPS1_BASE_ADDR + AIPS1_SIZE))) ? AIPS1_IO_ADDRESS(x):\
        ((x >= AIPS2_BASE_ADDR) && (x < (AIPS2_BASE_ADDR + AIPS2_SIZE))) ? AIPS2_IO_ADDRESS(x):\
        0xEEADBEEF)

/*
 * define the address mapping macros: in physical address order
 */

#define IRAM_IO_ADDRESS(x)  \
        (((x) - IRAM_BASE_ADDR) + IRAM_BASE_ADDR_VIRT)

#define CS4_IO_ADDRESS(x)  \
        (((x) - CS4_BASE_ADDR) + CS4_BASE_ADDR_VIRT)

#define L2CC_IO_ADDRESS(x)  \
        (((x) - L2CC_BASE_ADDR) + L2CC_BASE_ADDR_VIRT)

#define PLATFORM_IO_ADDRESS(x)  \
        (((x) - PLATFORM_BASE_ADDR) + PLATFORM_BASE_ADDR_VIRT)

#define TZIC_IO_ADDRESS(x)  \
        (((x) - TZIC_BASE_ADDR) + TZIC_BASE_ADDR_VIRT)

#define DEBUG_IO_ADDRESS(x)  \
        (((x) - DEBUG_BASE_ADDR) + DEBUG_BASE_ADDR_VIRT)

#define SPBA0_IO_ADDRESS(x)  \
        (((x) - SPBA0_BASE_ADDR) + SPBA0_BASE_ADDR_VIRT)

#define AIPS1_IO_ADDRESS(x)  \
        (((x) - AIPS1_BASE_ADDR) + AIPS1_BASE_ADDR_VIRT)

#define AIPS2_IO_ADDRESS(x)  \
        (((x) - AIPS2_BASE_ADDR) + AIPS2_BASE_ADDR_VIRT)

/*
 * DMA request assignments
 */
#define DMA_REQ_47	   47
#define DMA_REQ_46	   46
#define DMA_REQ_45	   45
#define DMA_REQ_UART3_TX   44
#define DMA_REQ_UART3_RX   43
#define DMA_REQ_MU_RX      42
#define DMA_REQ_MU_TX      41
#define DMA_REQ_SDHC3      40
#define DMA_REQ_CSPI3_TX   39
#define DMA_REQ_CSPI3_RX   38
#define DMA_REQ_37         37
#define DMA_REQ_IPU	   36
#define DMA_REQ_GPU	   35
#define DMA_REQ_34         34
#define DMA_REQ_MARC_TX    33
#define DMA_REQ_MARC_RX    32
#define DMA_REQ_ECT        31
#define DMA_REQ_NFC        30
#define DMA_REQ_SSI1_TX1   29
#define DMA_REQ_SSI1_RX1   28
#define DMA_REQ_SSI1_TX2   27
#define DMA_REQ_SSI1_RX2   26
#define DMA_REQ_SSI2_TX1   25
#define DMA_REQ_SSI2_RX1   24
#define DMA_REQ_SSI2_TX2   23
#define DMA_REQ_SSI2_RX2   22
#define DMA_REQ_SDHC2      21
#define DMA_REQ_SDHC1      20
#define DMA_REQ_UART1_TX   19
#define DMA_REQ_UART1_RX   18
#define DMA_REQ_UART2_TX   17
#define DMA_REQ_UART2_RX   16
#define DMA_REQ_GPIO1_0    15
#define DMA_REQ_GPIO1_1    14
#define DMA_REQ_UART4_TX   13
#define DMA_REQ_UART4_RX   12
#define DMA_REQ_UART5_TX   11
#define DMA_REQ_UART5_RX   10
#define DMA_REQ_CSPI1_TX   9
#define DMA_REQ_CSPI1_RX   8
#define DMA_REQ_CSPI2_TX   7
#define DMA_REQ_CSPI2_RX   6
#define DMA_REQ_SIM        5
#define DMA_REQ_ATA_TX_END 4
#define DMA_REQ_ATA_TX     3
#define DMA_REQ_ATA_RX     2
#define DMA_REQ_GPC        1
#define DMA_REQ_0   	   0

/*
 * Interrupt numbers
 */
#define MXC_INT_BASE		0

#define INT_RESV0               0
#define INT_MMC_SDHC1           1
#define INT_MMC_SDHC2           2
#define INT_MMC_SDHC3           3
#define INT_RESV4               4
#define INT_SDDC		5
#define INT_SDMA                6
#define INT_IOMUX		7
#define INT_RESV8		8
#define INT_VPU			9
#define INT_IPU_ERR             10
#define INT_IPU_SYN             11
#define INT_GPU    	        12
#define INT_MARC    	        13
#define INT_RESV14		14
#define INT_EMI			15
#define INT_USB_HOST1           16
#define INT_USB_HOST2           17
#define INT_USB_OTG             18
#define INT_SAHARA_HOST1	19
#define INT_SAHARA_HOST2	20
#define INT_SCC_SCM             21
#define INT_SCC_STZ             22
#define INT_SCC_SNTZ            23
#define INT_SRTC_NTZ		24
#define INT_SRTC_TZ		25
#define INT_RTIC		26
#define INT_CSU			27
#define INT_RESV28		28
#define INT_SSI1                29
#define INT_SSI2                30
#define INT_UART1               31
#define INT_UART2               32
#define INT_UART3               33
#define INT_UART4               34
#define INT_UART5               35
#define INT_CSPI1               36
#define INT_CSPI2               37
#define INT_CSPI3               38
#define INT_GPT                 39
#define INT_EPIT1               40
#define INT_EPIT2               41
#define INT_GPIO1_INT7          42
#define INT_GPIO1_INT6          43
#define INT_GPIO1_INT5          44
#define INT_GPIO1_INT4          45
#define INT_GPIO1_INT3          46
#define INT_GPIO1_INT2          47
#define INT_GPIO1_INT1          48
#define INT_GPIO1_INT0          49
#define INT_GPIO1_LOW           50
#define INT_GPIO1_HIGH          51
#define INT_GPIO2_LOW           52
#define INT_GPIO2_HIGH          53
#define INT_GPIO3_LOW           54
#define INT_GPIO3_HIGH          55
#define INT_GPIO4_LOW           56
#define INT_GPIO4_HIGH          57
#define INT_WDOG1               58
#define INT_WDOG2               59
#define INT_KPP                 60
#define INT_PWM                 61
#define INT_I2C                 62
#define INT_I2C2                63
#define INT_I2C3                64
#define INT_MSHC1               65
#define INT_MSHC2               66
#define INT_SIM1                67
#define INT_SIM2                68
#define INT_IIM                 69
#define INT_ATA                 70
#define INT_CCM1                71
#define INT_CCM2                72
#define INT_GPC1		73
#define INT_GPC2		74
#define INT_SRC			75
#define INT_EVTMON              76
#define INT_PER_MEASURE         77
#define INT_DECODE_ERR		78
#define INT_EVT_COUNT		79
#define INT_SLAVE_ERR		80
#define INT_HSC_ERR		81
#define INT_HSC_FUN		82
#define INT_HSC_TIMER		83
#define INT_MU_APU		84
#define INT_MU_RX1		85
#define INT_MU_TX1		86
#define INT_FEC			87
#define INT_OWIRE		88
#define INT_CTI0		89
#define INT_CTM0		90
#define INT_CTM1		91

#define MXC_MAX_INT_LINES       128

/*!
 * Interrupt Number for ARM11 PMU
 */
#define ARM11_PMU_IRQ		INT_EVTMON

#define	MXC_GPIO_BASE		(MXC_MAX_INT_LINES)

/*!
 * Number of GPIO port as defined in the IC Spec
 */
#define GPIO_PORT_NUM           4
/*!
 * Number of GPIO pins per port
 */
#define GPIO_NUM_PIN            32

#define PROD_SIGNATURE        0x1	/* For MX33 */
#define MXC_GPIO_SPLIT_IRQ_2

#define SYSTEM_REV_MIN          CHIP_REV_1_0
#define SYSTEM_REV_NUM          3

/*
 * Used for 1-Wire
 */
#define owire_read(a) (__raw_readw(a))
#define owire_write(v,a) (__raw_writew(v,a))

#endif				/*  __ASM_ARCH_MXC_MX33_H__ */
