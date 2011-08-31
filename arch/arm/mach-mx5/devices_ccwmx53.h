/*
  * Copyright 2011 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef __DEVICES_CCWMX53_H_
#define __DEVICES_CCWMX53_H_

extern struct resource mxcfb_resources[];
extern struct mxc_spi_master mxcspi1_data ;

void ccwmx53_register_sdio(int interface);
void ccwmx53_register_nand(void);
void ccwmx53_register_ext_eth(void);
void ccwmx53_register_fec(void);
int __init ccwmx5x_init_fb(void);
int __init ccwmx53_init_i2c_devices(void);

void gpio_smsc911x_active(void);
void gpio_sdhc_active(int interface);
void gpio_fec_active(void);


/* framebuffer settings */
#if defined(CONFIG_CCWMX5X_DISP0) && defined(CONFIG_CCWMX5X_DISP1)
#define FB_MEM_SIZE		SZ_32M
#else
#define FB_MEM_SIZE		SZ_16M
#endif

#define MX53_GPIO(port,pin)		((port - 1) * 32 + pin)
#define CCWMX53_EXT_IRQ_GPIO		MX53_GPIO(4,2)		/* GPIO_4_2 */
#define SECOND_TS_IRQ_PIN		MX53_GPIO(1,0)		/* GPIO_1_0 */
#define SECOND_TS_SPI_SS_PIN		MX53_GPIO(4,5)		/* GPIO_4_5 / ECSPI1_RDY */
#define CCWMX53_MMA7455_IRQ_GPIO	MX53_GPIO(4,4)		/* GPIO_4_4 */
#define CCWMX53_DA9052_IRQ_GPIO		MX53_GPIO(7,11)		/* GPIO7_11 */

#endif /* __DEVICES_CCWMX53_H_ */
