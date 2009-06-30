/*
 * arch/arm/mach-ns9xxx/cc9p9215_devices.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */


#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>

#include <mach/regs-s3c2443-mem.h>

#include "pipermain.h"
#include "mac.h"
#include "piperDsp.h"
#include "piperMacAssist.h"
#include "airoha.h"

#if defined(CONFIG_DIGI_PIPER_WIFI)
/* Low level functions to access piper chip */
static u32 read_reg(struct piper_priv *piperp, u8 reg)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);
	val = ioread32(piperp->vbase + reg);
	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return val;
}

static int write_reg(struct piper_priv *piperp, u8 reg, u32 val, reg_op_t op)
{
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);
	switch (op) {
	case op_write:
		iowrite16((u16)(val >> 16), piperp->vbase + reg + 2);
		iowrite16((u16)val, piperp->vbase + reg);
		break;
	case op_or:
		tmp = ioread32(piperp->vbase + reg) | val;
		iowrite16((u16)(tmp >> 16), piperp->vbase + reg + 2);
		iowrite16((u16)tmp, piperp->vbase + reg);
		break;
	case op_and:
		tmp = ioread32(piperp->vbase + reg) & val;
		iowrite16((u16)(tmp >> 16), piperp->vbase + reg + 2);
		iowrite16((u16)tmp, piperp->vbase + reg);
		break;
	default:
		printk(KERN_WARNING PIPER_DRIVER_NAME
		       ": Invalid write register operation (%d)\n", op);
		WARN_ON(1);
		break;
	}
	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return 0;
}

/*
 * This macro waits for the AES busy bit to clear if we are writing to the
 * AES FIFO.
 */
#define wait_for_aes_ready()	while ((addr == BB_AES_FIFO)			\
				      && ((ioread32(piperp->vbase + BB_RSSI)	\
					    & BB_RSSI_EAS_FIFO_FULL) != 0)) {	\
					udelay(1);				\
				}

static int write_fifo(struct piper_priv *piperp, u8 addr, u8 *buf, int len)
{
	int wordIndex, wordLength = len / sizeof(unsigned int);
	bool loadingBeacon = (addr == BEACON_FIFO);
	unsigned long flags;
	u32 tmp;

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);

	if (loadingBeacon) {
		/*
		 * If we are loading a new beacon, then adjust the address to point
		 * to the data FIFO, and set the beacon enable bit which tells piper
		 * to put this data into the beacon buffer.
		 */
		addr = BB_DATA_FIFO;
		tmp = ioread32(piperp->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_BEACON_EN;
		iowrite16((u16)(tmp >> 16), piperp->vbase + BB_GENERAL_CTL + 2);
		iowrite16((u16)tmp, piperp->vbase + BB_GENERAL_CTL);
	}

	if (((unsigned)(buf) & 0x3) == 0) {
		/*
		 * We come here if the data is 32-bit aligned.  We can dispense
		 * with memcpys
		 */
		if (wordLength == 1) {
			/*
			 * Only 1 word of data, so just one write.
			 */
			unsigned int *word = (unsigned int *)buf;
			tmp = cpu_to_be32(*word);

			wait_for_aes_ready();

			//iowrite32(cpu_to_be32(*word), piperp->vbase + addr);
			iowrite16((u16)(tmp >> 16), piperp->vbase + addr + 2);
			iowrite16((u16)tmp, piperp->vbase + addr);
			len -= 4;
		} else {
			/*
			 * More than one word of data, so set up a for loop.
			 */
			for (wordIndex = 0; wordIndex < wordLength; wordIndex++) {
				unsigned int *word = (unsigned int *)buf;

				tmp = cpu_to_be32(word[wordIndex]);

				wait_for_aes_ready();
				iowrite16((u16)(tmp >> 16), piperp->vbase + addr + 2);
				iowrite16((u16)tmp, piperp->vbase + addr);
				len -= 4;
			}
		}
	} else {
		/*
		 * Ugh!  Data is not 32-bit aligned.  We have to memcpy it!
		 */
		for (wordIndex = 0; wordIndex < wordLength; wordIndex++) {
			
			memcpy(&tmp, &buf[wordIndex * sizeof(u32)], sizeof(u32));
		  
			tmp = cpu_to_be32(tmp);

			wait_for_aes_ready();
			iowrite16((u16)(tmp >> 16), piperp->vbase + addr + 2);
			iowrite16((u16)tmp, piperp->vbase + addr);
			len -= 4;
		}
	}

	if (len) {
		/*
		 * Double Ugh!  There was left over data at the end.  We have to write
		 * the leftover data into the upper bytes of the last word, making
		 * sure the unused bytes are set to zero.
		 */
		memcpy(&tmp, &buf[wordLength * sizeof(u32)], sizeof(u32));
		tmp = cpu_to_be32(tmp);
		switch (len) {
		case 1:
			tmp &= 0xff000000;
			break;
		case 2:
			tmp &= 0xffff0000;
			break;
		case 3:
			tmp &= 0xffffff00;
			break;
		default:
			printk(KERN_WARNING PIPER_DRIVER_NAME
			       ": len = %d at end of piper_write\n", len);
			break;
		}
		wait_for_aes_ready();
		iowrite16((u16)(tmp >> 16), piperp->vbase + addr + 2);
		iowrite16((u16)tmp, piperp->vbase + addr);
	}

	if (loadingBeacon) {
		/*
		 * If we just loaded a beacon, then don't forget to turn off the
		 * load beacon bit.
		 */
		tmp = ioread32(piperp->vbase + BB_GENERAL_CTL) & ~BB_GENERAL_CTL_BEACON_EN;
		iowrite16((u16)(tmp >> 16), piperp->vbase + BB_GENERAL_CTL + 2);
		iowrite16((u16)tmp, piperp->vbase + BB_GENERAL_CTL);
		
		piperp->beacon.loaded = true;
	}

	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return 0;
}

/*
 * This routine waits for the empty flag to clear when we are reading from
 * the AES FIFO.
 */
#define wait_for_aes_not_empty()							\
	while ((addr == BB_AES_FIFO)							\
	      && ((ioread32(piperp->vbase + BB_RSSI) & BB_RSSI_EAS_FIFO_EMPTY) != 0)) {	\
		udelay(1);								\
		if (--timeout == 0) {							\
			timeout = 10000;						\
		}									\
	}

//TODO		piperpWifiDumpRegisters(piperp, MAIN_REGS);


static int read_fifo(struct piper_priv *piperp, u8 addr, u8 *buf, int len)
{
	int wordIndex;
	unsigned long flags;
	int timeout = 10000;

	spin_lock_irqsave(&piperp->ac->reg_lock, flags);

	/*
	 * We can only read 32-bit words, so round the length up to an even multiple of
	 * 4 if necessary.
	 */
	len += 3;
	len &= 0xfffffc;

	if ((len == 4) && ((((unsigned)buf) & 0x3) == 0)) {
		unsigned *word = (unsigned *)buf;

		wait_for_aes_not_empty();
		*word = be32_to_cpu(ioread32(piperp->vbase + addr));
	} else if ((((unsigned)buf) & 0x3) == 0) {
		unsigned *word = (unsigned *)buf;

		for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++) {
			wait_for_aes_not_empty();
			word[wordIndex] = be32_to_cpu(ioread32(piperp->vbase + addr));
		}
	} else {
		/*
		 * If we come here, then the buffer is not aligned and we have to
		 * memcpy the data.
		 */
		for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++) {
			unsigned word;

			wait_for_aes_not_empty();
			word = be32_to_cpu(ioread32(piperp->vbase + addr));
			memcpy(&buf[wordIndex * sizeof(unsigned)], &word, sizeof(word));
		}
	}
	spin_unlock_irqrestore(&piperp->ac->reg_lock, flags);

	return 0;
}

/*
 * Load the MAC Assist firmware into the chip. This is done by setting a bit
 * in the control register to enable MAC Assist firmware download, and then
 * writing the firmware into the data FIFO.
 */
static void piper_load_mac_firmware(struct piper_priv *piperp)
{
	unsigned int i;

	printk(KERN_DEBUG PIPER_DRIVER_NAME ": loading MAC Assist firmware\n");

	/* Zero out MAC assist SRAM (put into known state before enabling MAC assist) */
	for (i = 0; i < 0x100; i += 4)
		piperp->ac->wr_reg(piperp, i, 0, op_write);

	/* Enable download the MAC Assist program RAM */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_FW_LOAD_ENABLE, op_or);

	/* load MAC Assist data */
	for (i = 0; i < piper_macassist_data_len; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, piper_wifi_macassist_ucode[i],
				   op_write);

	/* disable MAC Assist download */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_FW_LOAD_ENABLE, op_and);
}

/*
 * Load the DSP firmware into the chip.  This is done by setting a bit
 * in the control register to enable DSP firmware download, and then
 * writing the firmware into the data FIFO.
 */
static void piper_load_dsp_firmware(struct piper_priv *piperp)
{
	unsigned int i;

	printk(KERN_DEBUG PIPER_DRIVER_NAME ": loading DSP firmware\n");

	/* Enable load of DSP firmware */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_DSP_LOAD_ENABLE, op_or);

	/* load DSP data */
	for (i = 0; i < piper_dsp_data_len; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, piper_wifi_dsp_ucode[i],
				   op_write);

	/* Disable load of DSP firmware */
	udelay(10);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_DSP_LOAD_ENABLE, op_and);

	/* Let her rip */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_MAC_ASSIST_ENABLE, op_or);
}


/*
 * This routine corrects a bug in the Piper chip where internal clocks would
 * be out of sync with each other and cause the chip to generate noise spikes.
 * This problem should be fixed in the next chip (Chopper).
 *
 * I'm not sure exactly what this code is doing.  It comes straight from the
 * guy who designed the chip.
 */
static int piper_spike_suppression(struct piper_priv *piperp)
{
	int timeout1 = 300, timeout2 = 300;
	int ret = 0;

	/*
	 * Initial timing measurement to avoid spike
	 * The new "magic" value is 0x63 at address 0xA62.  Bit-0 indicates the
	 * timing measurement is complete.  Bit-1 indicates that a second timing
	 * measurment was performed.  The upper nibble is the timing measurement
	 * value. This code should eliminate the possibility of spikes at the
	 * beginning of all PSK/CCK frames and eliminate the spikes at the end of
	 * all PSK (1M, 2M) frames.
	 */

	/* reset the timing value */
	piperp->ac->wr_reg(piperp, MAC_STATUS, 0xffff00ff, op_and);

	while ((piperp->ac->rd_reg(piperp, MAC_STATUS) & 0x0000ff00) != 0x00006300) {

		/* reset the timing value */
		piperp->ac->wr_reg(piperp, MAC_STATUS, 0xffff00ff, op_and);
		
		/* issue WiFi soft reset */
		piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x40000000, op_write);

		/* Set TX_ON Low */
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0xffffff3f, op_and);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x00000080, op_or);

		/* Set PA_2G Low */
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0xfffff0ff, op_and);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x00000a00, op_or);

		/* Set RX_ON low  */
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0xcfffffff, op_and);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x20000000, op_or);

		/* start the WiFi mac & dsp */
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720820, op_write);
		timeout1 = 500;

		/* Wait for timing measurement to finish */
		while ((piperp->ac->rd_reg(piperp, MAC_STATUS) & 0x0000ff00) != 0x00000100) {
			udelay(2);
			timeout1--;
			if (!timeout1)
				break;
		}

		timeout2--;
		if (!timeout2)
			ret = -EIO;
	}

	/* Set TX_ON/RXHP_ON and RX to normal wifi, restore the reset value to HW_OUT_CTRL */
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x1, op_write);

	return ret;
}

static void piper_reset_mac(struct piper_priv *piperp)
{
	int i;

	/* set the TX-hold bit */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720080, op_write);

	/* clear the TX-FIFO memory */
	for (i = 0; i < 448; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);

	/* reset the TX-FIFO */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x377200C0, op_write);

	/* release the TX-hold and reset */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720000, op_write);

/*	iowrite32(ioread32(piperp->vbase + MAC_STATUS) & ~0x40000000,
		  piperp->vbase + BB_GENERAL_STAT);*/
	mdelay(1);
}

/*
 * Load the MAC address into the chip. Use the value stored in the
 * environment, if there is one, otherwise use the default value.
 */
static void piper_set_macaddr(struct piper_priv *piperp)
{
	/* Default MAC Addr used if the nvram parameters are corrupted */
	u8 mac[6] = {0x00, 0x04, 0xf3, 0x00, 0x43, 0x35};
	u8 *pmac = piperp->pdata->macaddr;
	int i;

	for (i = 0; i < 6; i++) {
		if (*(pmac + i) != 0)
			break;
		if (i == 5) {
			/* There is a problem with the parameters, use default */
			printk(KERN_INFO PIPER_DRIVER_NAME
				": invalid mac address, using default\n");
			memcpy(piperp->pdata->macaddr, mac, sizeof(piperp->pdata->macaddr));
		}
	}

	memcpy(piperp->hw->wiphy->perm_addr, piperp->pdata->macaddr,
	       sizeof(piperp->hw->wiphy->perm_addr));

	/* configure ethernet address */
	piperp->ac->wr_reg(piperp, MAC_STA_ID0, *(pmac + 3) | *(pmac + 2) << 8 |
			   *(pmac + 1) << 16 | *(pmac + 0) << 24, op_write);
	piperp->ac->wr_reg(piperp, MAC_STA_ID1, *(pmac + 5) << 16 | *(pmac + 4) << 24,
			   op_write);
}

/* Initialize piper hardware, mac and dsp firmwares and mac address */
static int piper_init_chip_hw(struct piper_priv *piperp)
{
	int ret;

	piper_load_mac_firmware(piperp);
	piper_load_dsp_firmware(piperp);

	ret = piper_spike_suppression(piperp);
	if (ret) {
		printk(KERN_WARNING PIPER_DRIVER_NAME
		       ": spike suppresion error\n");
		return ret;
	}
	piper_reset_mac(piperp);
	piper_set_macaddr(piperp);

	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xffffffff, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);
	
	/* Enable host interface output control, to ensure a correct 
	 * level on the interrupt line */
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x04000000, op_or);

	return ret;
}

static void ccw9m2443_piper_reset(struct piper_priv *piperp, int reset)
{
	gpio_set_value(piperp->pdata->rst_gpio, !reset);
}

#ifdef LINIX_CONFIG_WIFI_CS
static int ccw9m2443_wifi_cs_config(void)
{
	void __iomem *ssmc;
	u32 reg;

	/* Configure the memory controller (CS3) with the appropriate settings */
 	if ((ssmc = ioremap(S3C2443_PA_SSMC, 0x100)) == NULL)
		return -EBUSY;  

	writel(10, ssmc + S3C2443_SSMC_SMBIDCYR4);	/* Idle cycle ctrl */
	writel(14, ssmc + S3C2443_SSMC_SMBWSTWRR4);	/* Write Wait State ctrl */
	writel(3, ssmc + S3C2443_SSMC_SMBWSTOENR4);	/* Output Enable Assertion Delay */
	writel(3, ssmc + S3C2443_SSMC_SMBWSTWENR4);	/* Write Enable Assertion Delay */
	writel(14 , ssmc + S3C2443_SSMC_SMBWSTRDR4);	/* Read Wait State control */
	/* Read Byte Lane Enable and configure memory width to 16 bit */
 	reg = (readl(ssmc + S3C2443_SSMC_SMBCR4) & ~(3 << 4)) | (1 << 4) | 0x1;
	writel(reg, ssmc + S3C2443_SSMC_SMBCR4);

	iounmap(ssmc);

	return 0;
}
#else
static int ccw9m2443_wifi_cs_config(void)	{return 0;}
#endif

static int ccw9m2443_piper_init(struct piper_priv *piperp)
{
	ccw9m2443_piper_reset(piperp, 1);
	mdelay(1);
	ccw9m2443_piper_reset(piperp, 0);
	mdelay(1);

	ccw9m2443_wifi_cs_config();

	/* Initialize functions to access register */
	piperp->ac->wr_reg = write_reg;
	piperp->ac->rd_reg = read_reg;
	piperp->ac->wr_fifo = write_fifo;
	piperp->ac->rd_fifo = read_fifo;

	mdelay(1);

	return piper_init_chip_hw(piperp);
}

static int ccw9m2443_piper_late_init(struct piper_priv *piperp)
{
	return 0;
}


static struct resource piper_resources[] = {
	{
		.start = S3C2410_CS4,
		.end   = S3C2410_CS4 + 0x100,
		.flags = IORESOURCE_MEM,
	}, {
		.start	= IRQ_EINT15,
		.flags	= IORESOURCE_IRQ,
	}
};

/* describes the device */
static struct platform_device piper_device = {
	.id		= 0,
	.name		= PIPER_DRIVER_NAME,
	.num_resources	= ARRAY_SIZE(piper_resources),
	.resource	= piper_resources,
};

void __init add_device_ccw9m2443_piper(struct piper_pdata *pdata)
{
	if (!pdata)
		return;

	if (pdata->rst_gpio > 0)
		gpio_request(pdata->rst_gpio, PIPER_DRIVER_NAME);
	if (pdata->irq_gpio > 0)
		gpio_request(pdata->irq_gpio, PIPER_DRIVER_NAME);

	/* Configure reset line and hold the chip in reset */
	gpio_direction_output(pdata->rst_gpio, 0);

	pdata->rf_transceiver = RF_AIROHA_7230;
	pdata->init = ccw9m2443_piper_init;
	pdata->late_init = ccw9m2443_piper_late_init;
	pdata->reset = ccw9m2443_piper_reset;
	piper_device.dev.platform_data = pdata;
	platform_device_register(&piper_device);
}

#else
void __init add_device_ccw9m2443_piper(struct piper_pdata *pdata) {}
#endif

