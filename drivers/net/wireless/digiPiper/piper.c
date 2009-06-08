/*
 * piper.c
 *
 * Copyright (C) 2008 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <net/mac80211.h>
#include <linux/usb.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <linux/timer.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"
#include "airoha.h"
#include "airohaCalibration.h"

#define WANT_AIROHA_CALIBRATION     (1)

static void piper_clear_irq_mask(struct piper_priv *piperp, unsigned int bits)
{
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, ~bits, op_and);
}

static void piper_set_irq_mask(struct piper_priv *piperp, unsigned int bits)
{
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, bits, op_or);
}

/* Generate a random number */
static int local_rand(void) 
{
	static unsigned long next = 1;

	/* RAND_MAX assumed to be 32767 */
	next = next * 1103515245 + 12345;
	return((unsigned)(next/65536) % 32768);
}

/*
 * Compute a beacon backoff time as described in section 11.1.2.2 of 802.11 spec.
 *
 */
static u16 get_next_beacon_backoff(void)
{
#define MAX_BEACON_BACKOFF      (2 * ASLOT_TIME * DEFAULT_CW_MIN)

	/*
	* We shift the result of local_rand() by 4 bits because the notes
	* for the algorithm say that we shouldn't rely on the last few
	* bits being random.  Other than that, we just take the random
	* value and make sure it is less than MAX_BEACON_BACKOFF.
	*/
	return (local_rand() >> 4) % MAX_BEACON_BACKOFF;
}

static int load_beacon(struct piper_priv *digi, unsigned char *buffer,
                        unsigned int length)
{
    return digi->ac->wr_fifo(digi, BEACON_FIFO, buffer, length);
}

static int piper_init_rx_tx(struct piper_priv *piperp)
{
	tasklet_init(&piperp->rx_tasklet, piper_rx_tasklet, (unsigned long)piperp);
	tasklet_disable(&piperp->rx_tasklet);

	piperp->txPacket = NULL;
	tasklet_init(&piperp->tx_tasklet, piper_tx_tasklet, (unsigned long)piperp);
	tasklet_disable(&piperp->tx_tasklet);

	return 0;
}

static void piper_free_rx_tx(struct piper_priv *piperp)
{
	tasklet_disable(&piperp->rx_tasklet);
	tasklet_kill(&piperp->rx_tasklet);
	tasklet_disable(&piperp->tx_tasklet);
	tasklet_kill(&piperp->tx_tasklet);
}

static int piper_init_hw(struct piper_priv *piperp, enum ieee80211_band band)
{
	int ret = 0;

	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_INIT, op_write);

	/* Initialize baseband general control register for the specific transceiver */
	if (piperp->pdata->rf_transceiver == RF_AIROHA_7230) {
		if (band == IEEE80211_BAND_2GHZ) {
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ, op_write);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xff00ffff, op_and);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);
			digi_dbg("piper_init_hw Initialized for band B / BG\n");
		} else {
			piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, GEN_INIT_AIROHA_50GHZ, op_write);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xff00ffff, op_and);
			piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_5150_5350_A_BAND, op_or);
			digi_dbg("piper_init_hw Initialized for band A\n");
		}
		piperp->ac->wr_reg(piperp, BB_CONF_2, 0x08329AD4, op_write);
		/* Initialize the SPI word length */
		piperp->ac->wr_reg(piperp, BB_SPI_CTRL, SPI_INIT_AIROHA, op_write);
	} else if (piperp->pdata->rf_transceiver == RF_AIROHA_2236) {
		piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ, op_write);
		piperp->ac->wr_reg(piperp, BB_CONF_2, 0x08329AD4, op_write);
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xff00ffff, op_and);
		piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, TRACK_BG_BAND, op_or);
		/* Initialize the SPI word length */
		piperp->ac->wr_reg(piperp, BB_SPI_CTRL, SPI_INIT_AIROHA2236, op_write);
	} else {
		printk(KERN_WARNING PIPER_DRIVER_NAME ": undefined rf transceiver!\n");
		return -EINVAL;
	}
	/*
	 *Clear the Intretupt Mask Register before enabling external intretupts.
	 * Also clear out any status bits in the Intretupt Status Register.
	 */
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xff, op_write);

	/*
	 * If this firmware supports additional MAC addresses.
	 */
	if (((piperp->ac->rd_reg(piperp, MAC_STATUS) >> 16) & 0xff) >= 8) {
		/* Disable additional addresses to start with */
		piperp->ac->wr_reg(piperp, MAC_CTL, ~MAC_CTL_MAC_FLTR, op_and);
		piperp->ac->wr_reg(piperp, MAC_STA2_ID0, 0, op_write);
		piperp->ac->wr_reg(piperp, MAC_STA2_ID1, 0, op_write);
		piperp->ac->wr_reg(piperp, MAC_STA3_ID0, 0, op_write);
		piperp->ac->wr_reg(piperp, MAC_STA3_ID1, 0, op_write);
	}
	/* TODO:  Set this register programatically */
	piperp->ac->wr_reg(piperp, MAC_DTIM_PERIOD, 0x0, op_write);

	/*
	 * Note that antenna diversity will be set by hw_start, which is the
	 * caller of this function.
	 */

	/* reset RX and TX FIFOs */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_RXFIFORST
			 | BB_GENERAL_CTL_TXFIFORST, op_or);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~(BB_GENERAL_CTL_RXFIFORST
						 | BB_GENERAL_CTL_TXFIFORST), op_and);

	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, 0xC043002C, op_write);

	/* Initialize RF transceiver */
	piperp->rf->init(piperp->hw, band);
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, 0x04000001, op_or);
	piperp->ac->wr_reg(piperp, MAC_CFP_ATIM, 0x0, op_write);
	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, ~(BB_GENERAL_STAT_DC_DIS
						  | BB_GENERAL_STAT_SPRD_DIS), op_and);
	piperp->ac->wr_reg(piperp, MAC_SSID_LEN, (MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK),
			   op_write);

	/*
	 * Set BSSID to the broadcast address so that we receive all packets.  The stack
	 * will set a real BSSID when it's ready.
	 */
	piperp->ac->wr_reg(piperp, MAC_BSS_ID0, 0xffffffff, op_write);
	piperp->ac->wr_reg(piperp, MAC_BSS_ID1, 0xffffffff, op_write);

#if WANT_AIROHA_CALIBRATION
	digi_dbg("Calling digiWifiInitCalibration()\n");
	digiWifiInitCalibration(piperp);
#endif

	return ret;
}

static int piper_deinit_hw(struct piper_priv *piperp)
{
	int ret = 0;
#if WANT_AIROHA_CALIBRATION
	digi_dbg("Calling digiWifiDeInitCalibration()\n");
	digiWifiDeInitCalibration(piperp);
#endif

	return ret;
}

/* Make sure all keys are disabled when we start */
static void piper_init_keys(struct piper_priv *piperp)
{
	unsigned int i;

	for (i = 0; i < PIPER_MAX_KEYS; i++)
		piperp->key[i].valid = false;

	piperp->aes_key_count = 0;
}

static void tx_timer_timeout(unsigned long arg)
{
	struct piper_priv *piperp = (struct piper_priv *) arg;

	packet_tx_done(piperp, RECEIVED_ACK, 0);
}


#ifdef CONFIG_PM

u32 saved_mac_regs[][2] = {
	/* Register, value */
	{BB_GENERAL_CTL,	0},
	{BB_GENERAL_STAT,	0},	
	{BB_RSSI,		0},
	{BB_IRQ_MASK,		0},
	{BB_SPI_CTRL,		0},
	{BB_TRACK_CONTROL,	0},
	{BB_CONF_2,		0},
	{BB_OUTPUT_CONTROL,	0},
	{MAC_CTL,		0}
};

static int piper_suspend(struct platform_device *dev, pm_message_t state)
{
	struct piper_priv *piperp = platform_get_drvdata(dev);
	int i;

	/* wait until aes done  */
	while (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY)
			;
	/* wait for tx fifo empty */
	while ((piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) &
		BB_GENERAL_CTL_TX_FIFO_EMPTY) == 0)
		;

	for (i = 0; i < (sizeof(saved_mac_regs)/(2*sizeof(u32))); i++)
		saved_mac_regs[i][1] = piperp->ac->rd_reg(piperp, saved_mac_regs[i][0]);

	/* disable receiver */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);
	piperp->ac->wr_reg(piperp, MAC_CTL, 0, op_write);
	piperp->clear_irq_mask_bit(piperp, 0xffffffff);

	disable_irq(piperp->irq);

	/* held the transceiver in reset mode, if reset cb was defined */
	if (piperp->pdata->reset)
		piperp->pdata->reset(piperp, 1);

	return 0;
}

static int piper_resume(struct platform_device *dev)
{
	struct piper_priv *piperp = platform_get_drvdata(dev);
	int i;

 	if (piperp->pdata->reset)
 		piperp->pdata->reset(piperp, 0);
	mdelay(20);

	for (i = 0; i < (sizeof(saved_mac_regs)/(2*sizeof(u32))); i++)
		piperp->ac->wr_reg(piperp, saved_mac_regs[i][0],
				  saved_mac_regs[i][0], op_write);
		
	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x30000000, op_write);
	piperp->ac->wr_reg(piperp, BB_AES_CTL, 0x0, op_write);

	/* Sanity checks... interrupt mask is not null... */

	/* Restart the processors */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   BB_GENERAL_CTL_MAC_ASSIST_ENABLE, op_or);
	/* set the TX-hold bit */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   0x37720080, op_write);

	/* clear the TX-FIFO memory */
	for (i = 0; i < 448; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);
	
	/* reset the TX-FIFO */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   0x377200C0, op_write);

	/* release the TX-hold and reset */
 	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   0x37720000, op_write);

	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, saved_mac_regs[0][1] |
			   0x37000000 | BB_GENERAL_CTL_RX_EN, op_write);

	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xff, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0x0, op_write);

	enable_irq(piperp->irq);

	return 0;
}
#else
#define piper_suspend	NULL
#define piper_resume	NULL
#endif

static int __init piper_probe(struct platform_device* pdev)
{
	struct piper_pdata *pdata = pdev->dev.platform_data;
	struct piper_priv *piperp;
	int ret = 0;

	if (!pdata)
		return -EINVAL;

	ret = piper_alloc_hw(&piperp, sizeof(*piperp));
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to alloc piper_priv\n");
		return ret;
	}

	piperp->ac = kzalloc(sizeof(struct access_ops), GFP_KERNEL);
	if (!piperp->ac){
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to alloc memory for ac struct\n");
		ret = -ENOMEM;
		goto error_alloc;
	}

	piperp->drv_name = PIPER_DRIVER_NAME;
	dev_set_drvdata(&pdev->dev, piperp);
	piperp->pdata = pdata;
	pdata->piperp = piperp;
	spin_lock_init(&piperp->ac->reg_lock);
	spin_lock_init(&piperp->aesLock);

	piperp->vbase = ioremap(pdev->resource[0].start,
				pdev[0].resource->end - pdev->resource[0].start);

	if (!piperp->vbase) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": ioremap base %x, len %x error\n",
		       pdev->resource[0].start,	pdev[0].resource->end - pdev->resource[0].start);
		ret = -ENOMEM;
		goto error_remap;
	}

	piperp->pstats.tx_start_count = 0;
	piperp->pstats.tx_complete_count = 0;

	/*
	 * Platform initialization. This will initialize the hardware, including the load
	 * of the mac and dsp firmware into the piper chip
	 */
	if (pdata->init)
		pdata->init(piperp);

	init_timer(&piperp->tx_timer);
	piperp->tx_timer.function = tx_timer_timeout;
	piperp->tx_timer.data = (unsigned long) piperp;

	piper_init_rx_tx(piperp);
	piper_init_keys(piperp);

	piperp->init_hw = piper_init_hw;
	piperp->deinit_hw = piper_deinit_hw;
	piperp->set_irq_mask_bit = piper_set_irq_mask;
	piperp->clear_irq_mask_bit = piper_clear_irq_mask;
	piperp->load_beacon = load_beacon;
	piperp->rand = local_rand;
	piperp->get_next_beacon_backoff = get_next_beacon_backoff;
	piperp->version = piperp->ac->rd_reg(piperp, BB_VERSION);

	piperp->irq = pdev->resource[1].start;
	piperp->tx_cts = false;
	piperp->beacon.loaded = false;
	piperp->beacon.enabled = false;
	piperp->beacon.weSentLastOne = false;

	ret = request_irq(piperp->irq, piper_irq_handler,
			  IRQF_TRIGGER_HIGH, PIPER_DRIVER_NAME,	piperp);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": unable to request irq %d (%d)",
			piperp->irq, ret);
		goto retor_irq;
	}
	disable_irq(piperp->irq);

	ret = piper_register_hw(piperp, &pdev->dev, &al7230_rf_ops);
	if (ret) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to register priv\n");
		goto do_free_rx;
	}

	if (pdata->late_init)
		pdata->late_init(piperp);

	printk(KERN_INFO PIPER_DRIVER_NAME ": driver loaded (fw ver = 0x%08x)\n",
		piperp->version);

	return 0;

do_free_rx:
	piper_free_rx_tx(piperp);
retor_irq:
	free_irq(piperp->irq, piperp);
	iounmap(piperp->vbase);
	piperp->vbase = NULL;
error_remap:
	release_resource(pdev->resource);
error_alloc:
	piper_free_hw(piperp);
	return ret;
}

static int piper_remove(struct platform_device *pdev)
{
	struct piper_priv *piperp = dev_get_drvdata(&pdev->dev);

	printk(KERN_DEBUG PIPER_DRIVER_NAME " %s\n", __func__);

	piper_unregister_hw(piperp);
	disable_irq(piperp->irq);
	piper_clear_irq_mask(piperp, 0xffffffff);
	free_irq(piperp->irq, piperp);
	piper_free_rx_tx(piperp);
	release_resource(pdev->resource);
	piper_free_hw(piperp);

	return 0;
}

/* describes the driver */
static struct platform_driver piper_driver = {
	.probe		= piper_probe,
	.remove		= piper_remove,
	.suspend 	= piper_suspend,
	.resume		= piper_resume,
	.driver 	= {
		.name  = PIPER_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init piper_init_module(void)
{
	return platform_driver_register(&piper_driver);
}

static void __exit piper_exit_module(void)
{
	platform_driver_unregister(&piper_driver);
}

module_init(piper_init_module);
module_exit(piper_exit_module);

MODULE_DESCRIPTION("Digi Piper WLAN Driver");
MODULE_AUTHOR("Contact support@digi.com for questions on this code");
MODULE_VERSION(DRV_VERS);
MODULE_LICENSE("GPL");
