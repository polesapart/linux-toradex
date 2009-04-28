/*
 * digiTx.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains the routines that are related to transmitting
 * frames.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>

#include "pipermain.h"
#include "mac.h"
#include "phy.h"


/*
 * This routine writes a frame using H/W AES encryption.  
 *
 * Arguments
 *      digi            context
 *      buffer          pointer to start of frame
 *      length          number of bytes in frame
 *
 * Return Values
 *      0           success
 *      !0          transmit failed
 */
static int piper_write_aes(struct piper_priv *digi, unsigned char *buffer,
		unsigned int length)
{
#define AES_TIMEOUT     (10000)
    int result;
    int timeout = AES_TIMEOUT;
    unsigned long spinLockFlags;
    
    /*
     * Step 1: Wait for AES to become ready.
     */
    spin_lock_irqsave(&digi->aesLock, spinLockFlags);
    while (digi->read_reg(digi, BB_RSSI) & BB_RSSI_EAS_BUSY)
    {
        timeout--;
        if (timeout == 0)
        {
            digiWifiDumpRegisters(digi, MAIN_REGS | MAC_REGS);
        }
        udelay(1);
    }
    
    /*
     * Step 2: Write the unencrypted part of the frame into the normal
     *         data FIFO.
     */
    digi->write(digi, BB_DATA_FIFO, buffer, 
                    _80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE);

    /*
     * Step 3: Write to the AES control register.  Writing to it puts 
     *         AES H/W engine into transmit mode.  We also make sure 
     *         the AES mode is set correctly.
     */
    digi->write_reg(digi, BB_AES_CTL, 0, op_write);
    
    /*
     * Step 4: Write the expanded AES key into the AES FIFO.
     */
    digi->write(digi, BB_AES_FIFO, 
                        (unsigned char *) digi->key[digi->txAesKey].expandedKey, 
                        EXPANDED_KEY_LENGTH);
    
    /*
     * Step 5: Write the AES IV and headers into the AES FIFO.
     */
    digi->write(digi, BB_AES_FIFO, (unsigned char *) digi->txAesBlob,
                         AES_BLOB_LENGTH);
    
    /*
     * Step 6: Now, finally, write the part of the frame that needs to
     *         be encrypted into the AES FIFO.
     */
    result = digi->write(digi, BB_AES_FIFO, &buffer[_80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE], 
                        length - (_80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE));

    spin_unlock_irqrestore(&digi->aesLock, spinLockFlags);
    
    return result;
}



/*
 * Determine what bit rate the next retry should be sent at.
 *
 * The mac80211 library passes us an array of tx bit rates.  Each entry 
 * has a rate index and a limit (max number of retries at that rate).  
 * We use the rate index to build the H/W transmit header.  The limit 
 * is decremented each time we retry.  When it reaches zero, we try the 
 * next rate in the array.
 */
static struct ieee80211_rate *getTxRate(struct piper_priv *digi, struct ieee80211_tx_info *txInfo)
{
    #define FIRST_RETRY_INDEX       (1)
    struct ieee80211_rate *result = NULL;
    
    if (   (txInfo->status.retries[FIRST_RETRY_INDEX].limit == 0) 
        || (txInfo->status.retries[FIRST_RETRY_INDEX].limit == -1)
        || (txInfo->status.retries[FIRST_RETRY_INDEX].rate_idx == -1))
    {
        /*
         * At the time this driver was written, the mac80211 library was passing
         * the retry array uninitialized.  So this piece of code will default to
         * retrying the transmit at a lower speed each time it is retried until
         * we reach 1 Mbps.  The last retry is always done at 1 Mbps.
         */
        txInfo->tx_rate_idx = txInfo->status.retries[0].rate_idx - (txInfo->status.retry_count - 1);
        if (txInfo->tx_rate_idx < 0)
        {
            txInfo->tx_rate_idx = 0;
        }
    }
    else
    {
        /*
         * We will come here if it looks like mac80211 actually gave us some
         * different rates to use for retries.
         *
         * Note:  The ieee80211_get_tx_rate and ieee80211_get_alt_retry_rate 
         *        functions are essentually worthless because they attempt to
         *        access fields in the control part of the tx info structure
         *        that have may already have been overwritten by information
         *        written in the status portion.  This is a UNION!
         */
        if (txInfo->status.retries[digi->txRetryIndex].limit == 0)
        {
            /*
             * If we get here, then it's time to try a new rate.
             */
            if (   (IEEE80211_TX_MAX_ALTRATE != digi->txRetryIndex)
                && (txInfo->status.retries[digi->txRetryIndex+1].rate_idx != -1)
                && (txInfo->status.retries[digi->txRetryIndex+1].limit > 0))
            {
                digi_dbg("Trying next rate, rate_idx = %d, limit = %d\n", 
                            txInfo->status.retries[digi->txRetryIndex+1].rate_idx,
                            txInfo->status.retries[digi->txRetryIndex+1].limit);
                /*
                 * Looks like we still have more entries in the array.  Update
                 * our array index and load the rate index with the new rate.
                 */
                digi->txRetryIndex++;
                txInfo->tx_rate_idx = txInfo->status.retries[digi->txRetryIndex].rate_idx;
                digi_dbg("Try new rate index %d\n", txInfo->tx_rate_idx);
            }
            else
            {
                /*
                 * Oops.  Out of rate entries.  Default to 1 Mbps.
                 */
                txInfo->tx_rate_idx = 0;
            }
        }
    }
    if (txInfo->tx_rate_idx == -1)
    {
        /*
         * A rate index of -1 indicates an unitialized entry.  Default it
         * to 1 Mbps.
         */
        txInfo->tx_rate_idx = 0;
    }
    if (txInfo->status.retry_count == digi->txMaxRetries)
    {
        /*
         * Send the last retry out at 1 Mbps reguardless.
         */
        txInfo->tx_rate_idx = 0;
    }
    if (txInfo->status.retries[digi->txRetryIndex].limit > 0)
    {
        /*
         * One less retry to go at this rate.
         */
        txInfo->status.retries[digi->txRetryIndex].limit--;
    }        
    result = ieee80211_get_tx_rate(digi->hw, txInfo);
    
    return result;
}





/*
 * This function returns a value for the contention window in microseconds.  We
 * start with the contention window at CW_MIN and double it everytime we have to
 * retry.  
 */
static u16 getCw(struct piper_priv *digi, bool isFirstTime)
{
    static u16 cw = DEFAULT_CW_MIN;
    
    if (isFirstTime)
    {
        cw = DEFAULT_CW_MIN;
    }
    else
    {
        cw <<= 1;
        if (cw > DEFAULT_CW_MAX)
        {
            cw = DEFAULT_CW_MAX;
        }
    }
    return (cw + (10*(digi->myrand() & (cw - 1)))) & 0xffff;
}



/*
 * This function will prepend an RTS or CTS to self frame ahead of the current
 * TX frame.  This is done if the wantRts or wantCts flag is set.  The mac80211
 * library determines if either of these flags is set.
 *
 * The RTS or CTS message is written into the transmit FIFO ahead of the
 * data frame.  Note that RTS and CTS messages are always sent in the clear
 * so we do not have to worry about encryption.
 *
 * Our caller, the master transmit routine, is responsible for setting the
 * transmit hold bit before calling us and clearing it after the data frame
 * has been written into the FIFO.  This ensures that the RTS/CTS frame is
 * not transmitted until after the data frame is ready to go.
 *
 * Also note that if we are unable to send the RTS/CTS frame, then the H/W
 * is smart enough to also about the data frame.  So we will not send
 * the data frame without the RTS/CTS frame.
 */
void handleRtsCts(struct piper_priv *digi, struct ieee80211_tx_info *txInfo, 
                    unsigned int frameType)
{
    unsigned int header[2];

    if (frameType == TYPE_DATA)
    {    
        if (digi->wantRts)
        {
            /*
             * If we come here, then we need to send an RTS frame ahead of the
             * current data frame.
             */
            if (digi->rtsCtsRate)
            {
                phy_set_plcp((unsigned char *) header, sizeof(struct ieee80211_rts), 
                                digi->rtsCtsRate, 0);
        		digi->write(digi, BB_DATA_FIFO, (unsigned char *) header, TX_HEADER_LENGTH);
        		digi->write(digi, BB_DATA_FIFO, (unsigned char *) &digi->rtsFrame, 
        		                        sizeof(digi->rtsFrame));
            }
            else 
            {
                digi_dbg("No rate for RTS frame.\n");
            }
        }
        else if (digi->wantCts)
        {
            /*
             * If we come here, then we need to send a CTS to self frame ahead of the 
             * current data frame.
             */
            if (digi->rtsCtsRate)
            {
                phy_set_plcp((unsigned char *) header, sizeof(struct ieee80211_cts), 
                                digi->rtsCtsRate, 0);
        		digi->write(digi, BB_DATA_FIFO, (unsigned char *) header, TX_HEADER_LENGTH);
        		digi->write(digi, BB_DATA_FIFO, (unsigned char *) &digi->ctsFrame, 
        		                        sizeof(digi->ctsFrame));
            }
            else 
            {
                digi_dbg("No rate for CTS frame.\n");
            }
        }
    }
}        
    

/*
 * This function is the entry point for the transmit tasklet.  It 
 * is called to transmit frames.  It will first be called to transmit
 * the frame and then to retry if the original transmit fails.  So
 * it does both the first transmit and the subsequent retries.
 *
 * Arguments
 *      context     context information
 */
void digiWifiTxRetryTaskletEntry (unsigned long context)
{
    struct piper_priv *digi = (struct piper_priv *) context;
    int err;

    /*
     * Clear flags here to cover ACK case.  We do not clear the flags in the ACK 
     * routine since it is possible to receive an ACK after we have started the
     * next packet.  The appropriate interrupts will be reenabled if we decide
     * to retransmit.
     */
	digi->clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	if (digi->txPacket != NULL)
	{
        struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(digi->txPacket);

        if (txInfo->status.retry_count != digi->txMaxRetries)
        {
            #define FRAME_CONTROL_FIELD_OFFSET      (sizeof(struct tx_frame_hdr) + sizeof(struct psk_cck_hdr))
            frameControlFieldType_t *fc = (frameControlFieldType_t *) &digi->txPacket->data[FRAME_CONTROL_FIELD_OFFSET];

            if (txInfo->status.retry_count != 0)
            {
                fc->retry = 1;              /* set retry bit */
            }
    
            digi->write_reg(digi, MAC_BACKOFF, getCw(digi, txInfo->status.retry_count == 0), op_write);
            txInfo->status.retry_count++; 
	        /*
	         * Build the H/W transmit header.  The transmit header is rebuilt on each
	         * retry because it has the TX rate information which may change for 
	         * retries.
	         */
	        phy_set_plcp(digi->txPacket->data, 
	                     digi->txPacket->len - TX_HEADER_LENGTH, 
	                     getTxRate(digi, txInfo), digi->useAesHwEncryption ? 8 : 0);
	        
        	/* 
        	 * Pause the transmitter so that we don't start transmitting before we
        	 * are ready.
        	 */
        	digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_TX_HOLD, op_or);
        	                
            /* Clear any pending TX interrupts */
            digi->write_reg(digi, BB_IRQ_STAT, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT, op_write);
	        handleRtsCts(digi, txInfo, fc->type);
	        
        	if (digi->useAesHwEncryption)
        	{
        		err = piper_write_aes(digi, digi->txPacket->data, digi->txPacket->len);
            }
        	else
        	{
        		err = digi->write(digi, BB_DATA_FIFO, digi->txPacket->data, digi->txPacket->len);
        	}

        	/* 
        	 * Now start the transmitter.
        	 */
        	err = digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_TX_HOLD, 
        	                      op_and);
        	                
	        /*
	         * Set interrupt flags.  Use the timeout interrupt if we expect
	         * an ACK.  Use the FIFO empty interrupt if we do not expect an ACK.
	         */
            if (txInfo->flags & IEEE80211_TX_CTL_NO_ACK)
            {
                digi->setIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TX_ABORT);
            }
            else
            {
                digi->setIrqMaskBit(digi, BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
            }
        }
        else
        {
            txInfo->status.excessive_retries = true;
            digi_dbg("All %d retries used up\n", digi->txMaxRetries);
            ieee80211_tx_status_irqsafe(digi->hw, digi->txPacket);
            				           
            digi->txPacket = NULL;
            ieee80211_wake_queues(digi->hw);
        }
    }
    else digi_dbg("digi->txPacket == NULL\n");
}

EXPORT_SYMBOL_GPL(digiWifiTxRetryTaskletEntry);
 

