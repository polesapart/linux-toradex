/*
  * Copyright 2010 Digi International, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#ifndef DEVICES_CCWMX51_H_
#define DEVICES_CCWMX51_H_

extern struct flash_platform_data mxc_nand_data;
extern struct smc911x_platdata ccwmx51_smsc9118;
extern struct mxc_mmc_platform_data mmc1_data ;
extern struct mxc_mmc_platform_data mmc3_data;
extern struct resource mxcfb_resources[1];
#ifdef CONFIG_I2C_MXC
extern struct mxc_i2c_platform_data mxci2c_data;
extern struct mxc_i2c_platform_data mxci2c_hs_data;
#endif
#ifdef CONFIG_I2C_IMX
extern struct imxi2c_platform_data mxci2c_data;
#endif
extern struct mxc_spi_master mxcspi1_data ;
extern struct mxc_spi_master mxcspi2_data ;
extern struct mxc_spi_master mxcspi3_data ;
extern struct mxc_ipu_config mxc_ipu_data;
extern struct mxc_vpu_platform_data mxc_vpu_data;
extern struct mxc_w1_config mxc_w1_data;
extern struct mxc_spdif_platform_data mxc_spdif_data;
extern struct tve_platform_data tve_data;
extern struct mxc_bus_freq_platform_data bus_freq_data;
extern struct platform_pwm_backlight_data mxc_pwm_backlight_data;
extern struct mxc_audio_platform_data wm8753_data;
extern struct mxc_fb_platform_data mx51_fb_data[];
extern struct uio_info gpu2d_platform_data;
extern struct ccwmx5x_lcd_pdata plcd_platform_data[2];
extern struct fsl_ata_platform_data ata_data;
extern int __init ccwmx51_init_i2c2(void);
extern void ccwmx51_init_spidevices(void);
extern int __init ccwmx51_init_fb(void);
extern void __init ccwmx51_io_init(void);
extern int __init ccwmx51_init_mc13892(void);
extern struct platform_device smsc911x_device;
extern void ccwmx51_set_mod_variant(u8 variant);
extern void ccwmx51_set_mod_revision(u8 revision);
extern void ccwmx51_set_mod_sn(u32 sn);
extern void ccwmx51_register_sdio(int interface);
extern void ccwmx51_init_devices(void);
extern int ccwmx51_create_sysfs_entries(void);
extern struct mxc_iim_data iim_data;

#endif /* DEVICES_CCWMX51_H_ */
