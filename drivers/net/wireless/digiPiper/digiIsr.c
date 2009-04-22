/*
 * digiIsr.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains the routines that are related to processing interrupts
 * from the MAC.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"






/*
 * This routine handles interrupts from the MAC.  
 */
irqreturn_t digiWifiIsr(int irq, void *dev_id)
{
	struct piper_priv *digi = dev_id;
	unsigned int status;

	/* 
	 * Read the interrupt status bits and acknowledge
	 * the interrupt.
	 */
	status = digi->read_reg(digi, BB_IRQ_STAT);
    status &= digi->read_reg(digi, BB_IRQ_MASK);
	digi->write_reg(digi, BB_IRQ_STAT, status, op_write);
    
	if (status & BB_IRQ_MASK_RX_FIFO) 
	{
	    /* 
	     * This interrupt indicates we have a frame in the FIFO.
	     * Set up to receive the packet.  Disable further interrupts
	     * until the receive is complete.
	     */
	    digi->clearIrqMaskBit(digi, BB_IRQ_MASK_RX_FIFO); 
#if WANT_TO_RECEIVE_FRAMES_IN_ISR
        /*
         * Call the receive routine directly we are built to
         * do so.
         */
        digiWifiRxTaskletEntry ((unsigned long) digi);
#else
        /*
         * Or schedule the RX tasklet to process the frame.
         */
        tasklet_hi_schedule(&digi->rxTasklet);
#endif
	} 

    if (status & BB_IRQ_MASK_TX_FIFO_EMPTY) 
    {
        /*
         * Transmit complete interrupt.  This IRQ is only unmasked if we are
         * not expecting the packet to be ACKed.  This will be the case for
         * broadcasts.  In this case, tell mac80211 the transmit occurred and
         * restart the tx queue.
         */
	    if (digi->txPacket != NULL)
	    {
	        digiWifiTxDone(digi, TX_COMPLETE, 0);
        } else digi_dbg("(status & BB_IRQ_..._EMPTY) && (digi->txPacket != NULL)\n");
		digi->clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
    }

	if (status & BB_IRQ_MASK_TIMEOUT)
	{
	    /*
	     * AP did not ACK our TX packet.
	     */
	    if (digi->txPacket != NULL)
	    {
	        tasklet_hi_schedule(&digi->txRetryTasklet);
        } else digi_dbg("(status & TIMEOUT) && (digi->txPacket == NULL)\n");
		digi->clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

	if (unlikely(status & BB_IRQ_MASK_TX_ABORT)) 
	{
        digi_dbg("Got tx abort\n");
	    /*
	     * Could not transmit a packet because the media was busy.
	     */
	    if (digi->txPacket != NULL)
	    {
	        tasklet_hi_schedule(&digi->txRetryTasklet);
        } else digi_dbg("(status & TX_ABORT) && (digi->txPacket != NULL)\n");
		digi->clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

    if (status & BB_IRQ_MASK_TBTT)
    {
        /*
         * This interrupt occurs at the start of a beacon period.  The only thing
         * we need to do is to write a new beacon backoff value.
         */
        u32 reg = digi->read_reg(digi, MAC_BEACON_FILT) & ~MAC_BEACON_BACKOFF_MASK;
        digi->write_reg(digi, MAC_BEACON_FILT, reg | digi->getNextBeaconBackoff(), op_write);
        /*
         * TODO:  Improve the way we keep track of whether or not we sent the last
         *        beacon.  What we are doing now is to assume that we did until and
         *        unless we receive a beacon.  What we should do is look for either
         *        a beacon or a TX end interrupt.  However, since mac80211 doesn't
         *        tell us what the ATIM window is, we have to assume it is zero, 
         *        which means we could be transmitting a frame at the same
         *        time we are sending the beacon, so there isn't really any easy
         *        way for us to do this.  In fact, even if there was an ATIM 
         *        window, we could have started a transmit just before we get this
         *        interrupt, so I'm not sure how we are really suppose to keep
         *        track of this.
         */
        digi->beacon.weSentLastOne = true;         /* assume we sent last beacon unless we receive one*/
    }

	if (status & BB_IRQ_MASK_ATIM)
	{
	    /*
	     * This interrupt should not occur since we are not using it.  When in
	     * IBSS mode, the beacon period starts at the TBTT interrupt and ends
	     * at this interrupt.  We are not suppose to send packets between the
	     * two interrupts.  However, mac80211 does not seem to provide a way
	     * for us to find out how long the ATIM period is, so we have to assume
	     * that there isn't one.
	     *
	     * If we were supporting this interrupt we would have to synchronize 
	     * with the transmit routine so that transmit is paused during this 
	     * time.
	     */
        digi_dbg("Got BB_IRQ_MASK_ATIM\n");
		digi->clearIrqMaskBit(digi, BB_IRQ_MASK_ATIM);
	}

	if (unlikely(status & BB_IRQ_MASK_RX_OVERRUN)) 
	{
	    /* digi_dbg("Got a receive overrun\n"); */
#if 0
	    if (digi->read_reg(digi, BB_IRQ_MASK) & BB_IRQ_MASK_TIMEOUT)
	    {
            digi->txTimer.expires = jiffies + (HZ >> 2);
            add_timer(&digi->txTimer);
        }
#endif
		digi->rxOverruns++;
	}

	return IRQ_HANDLED;
}

EXPORT_SYMBOL_GPL(digiWifiIsr);
        


