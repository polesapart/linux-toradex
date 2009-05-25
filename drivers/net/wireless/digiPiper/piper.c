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
#include <net/piper_pdata.h>

#include "pipermain.h"
#include "mac.h"
#include "piperhwinit.h"
#include "phy.h"
#include "airoha.h"
#include "adc121c027.h"
#include "airohaCalibration.h"




#define	DRIVER_VERSION	"0.1"

#define MAC_CTRL_BASE   (volatile unsigned int *) (MAC_REG_BASE + (0x40))



#define WANT_DEBUG_THREAD           (0)
#define WANT_AIROHA_CALIBRATION     (1)



static void clearIrqMaskBit(struct piper_priv *digi, unsigned int bits)
{
    digi->write_reg(digi, BB_IRQ_MASK, ~bits, op_and);
}

static void setIrqMaskBit(struct piper_priv *digi, unsigned int bits)
{
    digi->write_reg(digi, BB_IRQ_MASK, bits, op_or);
}



/*
 * Generate a random number.
 */
static int myrand(void) {
/* RAND_MAX assumed to be 32767 */
    static unsigned long next = 1;
    next = next * 1103515245 + 12345;
    return((unsigned)(next/65536) % 32768);
}

/*
 * Compute a beacon backoff time as described in section 11.1.2.2 of 802.11 spec.
 *
 */
static u16 getNextBeaconBackoff(void)
{
#define MAX_BEACON_BACKOFF      (2 * ASLOT_TIME * DEFAULT_CW_MIN)

    /*
     * We shift the result of myrand() by 4 bits because the notes
     * for the algorithm say that we shouldn't rely on the last few
     * bits being random.  Other than that, we just take the random
     * value and make sure it is less than MAX_BEACON_BACKOFF.
     */
    return (myrand() >> 4) % MAX_BEACON_BACKOFF;
}



static int load_beacon(struct piper_priv *digi, unsigned char *buffer,
                        unsigned int length)
{
    return digi->write(digi, BEACON_FIFO, buffer, length);
}

#if WANT_DEBUG_THREAD


static int debugThreadEntry(void *data)
{
	struct piper_priv *digi = data;

    while (1)
    {
        ssleep(10);
        if (digiWifiInitAdc(digi) == 0)
        {
            break;
        }
    }
    while (1)
    {
        ssleep(10);
        digi_dbg("Peak = 0x%4.4X, last conversion = 0x%4.4X\n", digi->adcReadPeak(digi), digi->adcReadLastValue(digi));
        digi->adcClearPeak(digi);
    }
    return 0;
}
#endif



static int init_rx_tx(struct piper_priv *digi)
{
#if WANT_DEBUG_THREAD
    struct task_struct *debugThread;
#endif

    tasklet_init(&digi->rxTasklet, digiWifiRxTaskletEntry, (unsigned long) digi);
    tasklet_disable(&digi->rxTasklet);

    digi->txPacket = NULL;
    tasklet_init(&digi->txRetryTasklet, digiWifiTxRetryTaskletEntry, (unsigned long) digi);
    tasklet_disable(&digi->txRetryTasklet);

#if WANT_DEBUG_THREAD
	debugThread = kthread_run(debugThreadEntry, digi, PIPER_DRIVER_NAME " - debug");
#endif
    return 0;
}

static void free_rx_tx(struct piper_priv *digi)
{
    tasklet_disable(&digi->rxTasklet);
    tasklet_kill(&digi->rxTasklet);
    tasklet_disable(&digi->txRetryTasklet);
    tasklet_kill(&digi->txRetryTasklet);
}





static int initHw(struct piper_priv *digi, enum ieee80211_band band)
{
    int result = 0;

	digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_INIT, op_write);

#if (TRANSCEIVER == RF_AIROHA_7230)
        /* Initialize baseband general control register */
        if (band == IEEE80211_BAND_2GHZ)
        {
            digi->write_reg(digi, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ, op_write);
            digi->write_reg(digi, BB_TRACK_CONTROL,
                            0xff00ffff, op_and);
            digi->write_reg(digi, BB_TRACK_CONTROL,
                            TRACK_BG_BAND, op_or);
            digi_dbg("initHw Initialized for band B / BG\n");
        }
        else
        {
            digi->write_reg(digi, BB_GENERAL_CTL, GEN_INIT_AIROHA_50GHZ, op_write);
            digi->write_reg(digi, BB_TRACK_CONTROL,
                            0xff00ffff, op_and);
            digi->write_reg(digi, BB_TRACK_CONTROL,
                            TRACK_5150_5350_A_BAND, op_or);
            digi_dbg("initHw Initialized for band A\n");
        }

        digi->write_reg(digi, BB_CONF_2, 0x08329AD4, op_write);

        /* Initialize the SPI word length */
        digi->write_reg(digi, BB_SPI_CTRL, SPI_INIT_AIROHA, op_write);
#elif (TRANSCEIVER == RF_AIROHA_2236)

        /* Initialize baseband general control register */
        digi->write_reg(digi, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ, op_write);

        digi->write_reg(digi, BB_CONF_2, 0x08329AD4, op_write);

        digi->write_reg(digi, BB_TRACK_CONTROL,
                        0xff00ffff, op_and);
        digi->write_reg(digi, BB_TRACK_CONTROL,
                        TRACK_BG_BAND, op_or);

        /* Initialize the SPI word length */
        digi->write_reg(digi, BB_SPI_CTRL, SPI_INIT_AIROHA2236, op_write);
#endif
    // Clear the Interrupt Mask Register before enabling external interrupts.
    // Also clear out any status bits in the Interrupt Status Register.
    digi->write_reg(digi, BB_IRQ_MASK, 0, op_write);
    digi->write_reg(digi, BB_IRQ_STAT, 0xff, op_write);

    /*
     * If this firmware supports additional MAC addresses.
     */
    if (((digi->read_reg(digi, MAC_STATUS) >> 16) & 0xff) >= 8)
    {
        /* Disable additional addresses to start with */
        digi->write_reg(digi, MAC_CTL, ~MAC_CTL_MAC_FLTR, op_and);
        /*
         * Clear registers that hold extra addresses.
         */

        digi->write_reg(digi, MAC_STA2_ID0, 0, op_write);
        digi->write_reg(digi, MAC_STA2_ID1, 0, op_write);
        digi->write_reg(digi, MAC_STA3_ID0, 0, op_write);
        digi->write_reg(digi, MAC_STA3_ID1, 0, op_write);
    }
/*
 * TODO:  Set this register programatically.
 */
/****/ digi->write_reg(digi, MAC_DTIM_PERIOD, 0x0, op_write);

    /*
     * Note that antenna diversity will be set by hw_start, which is the
     * caller of this function.
     */

    // reset RX and TX FIFOs
    digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_RXFIFORST
                                    | BB_GENERAL_CTL_TXFIFORST, op_or);
    digi->write_reg(digi, BB_GENERAL_CTL, ~(BB_GENERAL_CTL_RXFIFORST
                                            | BB_GENERAL_CTL_TXFIFORST), op_and);

    digi->write_reg(digi, BB_TRACK_CONTROL, 0xC043002C, op_write);

    /* Initialize RF transceiver */
    digi->rf->init(digi->hw, band);
    digi->write_reg(digi, BB_OUTPUT_CONTROL, 0x04000001, op_or);
/****/ digi->write_reg(digi, MAC_CFP_ATIM, 0x0, op_write);
    digi->write_reg(digi, BB_GENERAL_STAT, ~(BB_GENERAL_STAT_DC_DIS
                                    | BB_GENERAL_STAT_SPRD_DIS), op_and);
    digi->write_reg(digi, MAC_SSID_LEN, (MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK), op_write);

    /*
     * Set BSSID to the broadcast address so that we receive all packets.  The stack
     * will set a real BSSID when it's ready.
     */
    digi->write_reg(digi, MAC_BSS_ID0, 0xffffffff, op_write);
    digi->write_reg(digi, MAC_BSS_ID1, 0xffffffff, op_write);

#if WANT_AIROHA_CALIBRATION
    digi_dbg("Calling digiWifiInitCalibration()\n");
    digiWifiInitCalibration(digi);
#endif



#ifdef MAC_PS_ENABLED
    PIO_OUTPUT(WLN_PS_CNTRL_PIN);
#endif

    return result;
}


/*
 * Make sure all keys are disabled when we start.
 */
static void initializeKeys(struct piper_priv *digi)
{
    unsigned int i;

    for (i = 0; i < PIPER_MAX_KEYS; i++)
    {
        digi->key[i].valid = false;
    }
    digi->AESKeyCount = 0;
}

static void kickTransmitterTask(unsigned long arg)
{
    struct piper_priv *digi = (struct piper_priv *) arg;

    digi_dbg("Forced ACK\n");
    digiWifiTxDone(digi, RECEIVED_ACK, 0);
}


#ifdef CONFIG_PM
static int piper_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int piper_resume(struct platform_device *dev)
{
	return 0;
}
#else
#define piper_suspend	NULL
#define piper_resume	NULL
#endif

static int __init piper_probe(struct platform_device* pdev)
{
	int err = 0;
	struct piper_priv *digi;
	unsigned int version, status;
	struct piper_pdata *pdata = pdev->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	err = digiWifiAllocateHw(&digi, sizeof(*digi));
	if (err)
	{
		printk(KERN_ERR PIPER_DRIVER_NAME "failed to alloc priv\n");
		return err;
	}

	dev_set_drvdata(&pdev->dev, digi);
	digi->pdata = pdata;
	digi->drv_name = PIPER_DRIVER_NAME;
	digi->txStartCount = 0;
	digi->txCompleteCount = 0;
	spin_lock_init(&digi->registerAccessLock);
	spin_lock_init(&digi->aesLock);

	{
	    /*
	     * Create a temporary timer structure named temp so that we can use
	     * the standard timer initialization macro DEFINE_TIMER to initialize
	     * it and then copy the initialized values to digi->tx_timer.
	     */
	    DEFINE_TIMER(temp, kickTransmitterTask, 0, (unsigned long) digi);
	    digi->txTimer = temp;
	}

    /*
     * Reserve access to the Piper registers mapped to memory.
     *
     * iomem_resource:  some global data structure that has all
     *                  the special memory address spaces
     * pdev->resource   set to piper_mem
     */
#ifdef BUILD_AS_LOADABLE_MODULE
	err = request_resource(&iomem_resource, &piper_mem);
	if (err)
	{
		printk(KERN_INFO "request_resource returned %d", err);
		printk(KERN_INFO "Memory already in used: 0x%08x...0x%08x",
				pdev->resource[0].start, pdev->resource[0].end);
		goto piper_probe_exit;
	}
#endif
	digi->vbase = ioremap(pdev->resource[0].start,
			pdev[0].resource->end - pdev->resource[0].start);
	printk(KERN_INFO "digi->vbase = 0x%p\n", digi->vbase);

	if (NULL == digi->vbase)
	{
        ERROR("ioremap failed");
        err = -ENOMEM;
        goto error_remap;
    }

    /*
     * Besides loading the Piper firmware, this function also sets the
     * MAC address and loads it into digi->hw->wiphy->perm_addr.
     */
    piper_load_firmware(digi);

    init_rx_tx(digi);

    initializeKeys(digi);
	/* provide callbacks to generic mac code */
    digiWifiSetRegisterAccessRoutines(digi);
	digi->initHw = initHw;
	digi->setIrqMaskBit = setIrqMaskBit;
	digi->clearIrqMaskBit = clearIrqMaskBit;
	digi->load_beacon = load_beacon;
    digi->myrand = myrand;
    digi->getNextBeaconBackoff = getNextBeaconBackoff;

    version =digi->read_reg(digi, BB_VERSION);
    status = digi->read_reg(digi, BB_GENERAL_STAT);

    digi_dbg("Piper firmware version = 0x%8.8X, status = 0x%8.8X\n", version, status);

    digi->irq = pdev->resource[1].start;
    digi->bssWantCtsProtection = false;
    digi->beacon.loaded = false;
    digi->beacon.enabled = false;
    digi->beacon.weSentLastOne = false;

	err = request_irq(digi->irq, digiWifiIsr, IRQF_TRIGGER_HIGH, PIPER_DRIVER_NAME,
			digi);
	if (err)
	{
		printk(KERN_ERR PIPER_DRIVER_NAME "register interrupt %d failed, err %d", digi->irq, err);
		goto error_irq;
	}
	else
	{
	    printk(KERN_INFO PIPER_DRIVER_NAME " IRQ %d installed.\n", digi->irq);
	}

	disable_irq(digi->irq);

	/* TODO move this to platform code*/
	gpio_configure_ns921x(pdata->irq_gpio, NS921X_GPIO_INPUT,
			      NS921X_GPIO_DONT_INVERT, NS921X_GPIO_FUNC_2,
			      NS921X_GPIO_ENABLE_PULLUP);

	err = digiWifiRegisterHw(digi, &pdev->dev, &al7230_rf_ops);
	if (err) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to register priv\n");
		goto do_free_rx;
	}


	goto piper_probe_exit;


do_free_rx:
    free_rx_tx(digi);
error_irq:
    free_irq(pdev->resource[1].start, digi);
    iounmap(digi->vbase);
    digi->vbase = NULL;
error_remap:
    release_resource(pdev->resource);
piper_probe_exit:
	return err;
}


static int piper_remove(struct platform_device *pdev)
{
	struct piper_priv *digi;

	digi = dev_get_drvdata(&pdev->dev);

    printk(KERN_INFO "piper_remove called\n");
    printk(KERN_INFO "digi = 0x%p\n", digi);

    /* extern void enable_irq(unsigned int irq); */
    digiWifiUnregisterHw(digi);
	disable_irq(pdev->resource[1].start);
    clearIrqMaskBit(digi, 0xffffffff);
    free_irq(pdev->resource[1].start, digi);
    free_rx_tx(digi);
    release_resource(pdev->resource);
    printk(KERN_INFO "piper_remove release_resource() has returned\n");
	digiWifiFreeHw(digi);

    printk(KERN_INFO "piper_remove returning\n");
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
