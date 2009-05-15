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
#include <linux/jiffies.h>
#include <linux/timer.h>

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
    struct ieee80211_rate *result = NULL;
    
    if (digi->calibrationTxRate)
    {
        result = digi->calibrationTxRate;
        goto getTxRateDone;
    }
    
#if 0
struct ieee80211_tx_rate {
	s8 idx;
	u8 count;
	u8 flags;
} __attribute__((packed));
struct ieee80211_tx_info {
	/* common information */
	u32 flags;
	u8 band;

	u8 antenna_sel_tx;

	/* 2 byte hole */
	u8 pad[2];

	union {
		struct {
			union {
				/* rate control */
				struct {
					struct ieee80211_tx_rate rates[
						IEEE80211_TX_MAX_RATES];
					s8 rts_cts_rate_idx;
				};
				/* only needed before rate control */
				unsigned long jiffies;
			};
			/* NB: vif can be NULL for injected frames */
			struct ieee80211_vif *vif;
			struct ieee80211_key_conf *hw_key;
			struct ieee80211_sta *sta;
		} control;
		struct {
			struct ieee80211_tx_rate rates[IEEE80211_TX_MAX_RATES];
			u8 ampdu_ack_len;
			u64 ampdu_ack_map;
			int ack_signal;
			/* 8 bytes free */
		} status;
		struct {
			struct ieee80211_tx_rate driver_rates[
				IEEE80211_TX_MAX_RATES];
			void *rate_driver_data[
				IEEE80211_TX_INFO_RATE_DRIVER_DATA_SIZE / sizeof(void *)];
		};
		void *driver_data[
			IEEE80211_TX_INFO_DRIVER_DATA_SIZE / sizeof(void *)];
	};
};

#endif    
#if 0
    digi_dbg("0 = %d :: %d, 1 = %d :: %d, 2 = %d :: %d, 3 = %d :: %d\n",
                txInfo->control.rates[0].count, txInfo->control.rates[0].idx,
                txInfo->control.rates[1].count, txInfo->control.rates[1].idx,
                txInfo->control.rates[2].count, txInfo->control.rates[2].idx,
                txInfo->control.rates[3].count, txInfo->control.rates[3].idx);
#endif
#if 1
/* TODO:  Remove this code when mac80211 implements retry counts */
if (   (txInfo->control.rates[0].count == 1) 
    && (txInfo->control.rates[1].count == 1)
    && (txInfo->control.rates[1].idx == -1))
{
    s8 idx = txInfo->control.rates[0].idx - 1;

    txInfo->control.rates[0].count = 2;
    txInfo->control.rates[1].count = 1;
    txInfo->control.rates[1].idx = (idx > -1)? idx : 0;
    idx--;
    txInfo->control.rates[2].count = 1;
    txInfo->control.rates[2].idx = (idx > -1)? idx : 0;
    txInfo->control.rates[3].count = 1;
    txInfo->control.rates[3].idx = 0;
    txInfo->control.rates[4].count = 1;
    txInfo->control.rates[4].idx = -1;
}
#endif
    if (txInfo->control.rates[digi->txRetryIndex].count > digi->txRetryCount[digi->txRetryIndex])
    {
        if (digi->txRetryIndex == 0)
        {
            result = ieee80211_get_tx_rate(digi->hw, txInfo);
        }
        else
        {
            result = ieee80211_get_alt_retry_rate(digi->hw, txInfo, 
                                digi->txRetryIndex - 1);
        }
    }
    else if (txInfo->control.rates[digi->txRetryIndex + 1].idx != -1)
    {
        result = ieee80211_get_alt_retry_rate(digi->hw, txInfo, 
                            digi->txRetryIndex);
        digi->txRetryIndex++;
    }
    if (result != NULL)
    {
        digi->txRetryCount[digi->txRetryIndex]++;
    }
#if 0        
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
#if 0
                digi_dbg("Trying next rate, rate_idx = %d, limit = %d\n", 
                            txInfo->status.retries[digi->txRetryIndex+1].rate_idx,
                            txInfo->status.retries[digi->txRetryIndex+1].limit);
#endif
                /*
                 * Looks like we still have more entries in the array.  Update
                 * our array index and load the rate index with the new rate.
                 */
                digi->txRetryIndex++;
                txInfo->tx_rate_idx = txInfo->status.retries[digi->txRetryIndex].rate_idx;
#if 0
                digi_dbg("Try new rate index %d\n", txInfo->tx_rate_idx);
#endif
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
#endif

getTxRateDone:
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
static void handleRtsCts(struct piper_priv *digi, struct ieee80211_tx_info *txInfo, 
                    unsigned int frameType)
{

    digi->txRts = false;
    
    if (frameType == TYPE_DATA)
    {    
        unsigned int header[2];
        struct ieee80211_rate *rate = NULL;
        bool wantCts = (!!(txInfo->control.rates[digi->txRetryIndex].flags
                             & IEEE80211_TX_RC_USE_CTS_PROTECT)
                         | digi->bssWantCtsProtection);
        bool wantRts = !!(txInfo->control.rates[digi->txRetryIndex].flags
                            & IEEE80211_TX_RC_USE_RTS_CTS);
    
        if ((wantRts) || (wantCts))
        {
            /*
             * If we are sending an RTS or a CTS, then get the rate information.
             */
            if (digi->calibrationTxRate)
            {
                rate = digi->calibrationTxRate;
            }
            else
            {
                rate = ieee80211_get_rts_cts_rate(digi->hw, txInfo);
            }
            if (rate == NULL)
            {
                digi_dbg("ieee80211_get_rts_cts_rate(digi->hw, txInfo) returned NULL!\n");
            }
        }
        if ((wantRts) && (rate))
        {
            /*
             * We're sending an RTS, so load it into the FIFO.
             */
            struct ieee80211_rts rtsFrame;
            
            ieee80211_rts_get(digi->hw, txInfo->control.vif, digi->txPacket->data + TX_HEADER_LENGTH, 
                              digi->txPacket->len - TX_HEADER_LENGTH, txInfo, &rtsFrame);
            /*
             * If we come here, then we need to send an RTS frame ahead of the
             * current data frame.
             */
            phy_set_plcp((unsigned char *) header, sizeof(struct ieee80211_rts), 
                            rate, 0);
    		digi->write(digi, BB_DATA_FIFO, (unsigned char *) header, TX_HEADER_LENGTH);
    		digi->write(digi, BB_DATA_FIFO, (unsigned char *) &rtsFrame, 
    		                        sizeof(rtsFrame));
    		if (digi->txTotalRetries != 0)
    		{
    		    digi->lowLevelStats.dot11RTSFailureCount++;
    		}
    		digi->txRts = true;
        }
        else if ((wantCts) && (rate))
        {
            /*
             * We're sending a CTS, so load it into the FIFO.
             */
            struct ieee80211_cts ctsFrame;
            
            ieee80211_ctstoself_get(digi->hw, txInfo->control.vif, 
                                    digi->txPacket->data + TX_HEADER_LENGTH, 
                                    digi->txPacket->len - TX_HEADER_LENGTH,
                                    txInfo, &ctsFrame);
            /*
             * At the time this code was written, the mac80211 library had 
             * a bug in the ieee80211_ctstoself_get which caused it to copy 
             * the wrong MAC address into the cts frame.  So we copy the
             * right one (ours) in now.
             */
            memcpy(digi->ctsFrame.ra, digi->hw->wiphy->perm_addr, ETH_ALEN);

            /*
             * If we come here, then we need to send a CTS to self frame ahead of the 
             * current data frame.
             */
            phy_set_plcp((unsigned char *) header, sizeof(struct ieee80211_cts), 
                            rate, 0);
    		digi->write(digi, BB_DATA_FIFO, (unsigned char *) header, TX_HEADER_LENGTH);
    		digi->write(digi, BB_DATA_FIFO, (unsigned char *) &ctsFrame, 
    		                        sizeof(ctsFrame));
        }
    }
}        


/*
 * This routine is called to report the result of a transmit operation to
 * mac80211.  It is used for both successful transmissions and failures.
 * It sends the result to the stack, zeros digi->txPacket, and then wakes 
 * up the transmit queue.
 */
void digiWifiTxDone(struct piper_priv *digi, enum digiWifiTxResult_t result,
                    int signalStrength)
{
	if (digi->txPacket != NULL)
	{
        struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(digi->txPacket);
        unsigned int i;
        
        del_timer_sync(&digi->txTimer);
        
    	if (digi->txTransmitFinished)
    	{
    	    digi->txTransmitFinished(digi);
    	}
        ieee80211_tx_info_clear_status(txInfo);
        
        for (i = 0; i < IEEE80211_TX_MAX_RATES; i++)
        {
            txInfo->status.rates[i].count = digi->txRetryCount[i];
        }

        txInfo->status.ampdu_ack_len = 0;
		txInfo->status.ampdu_ack_map = 0;
		txInfo->status.ack_signal = signalStrength;
        
        if (result == RECEIVED_ACK)
        {
            txInfo->flags |= IEEE80211_TX_STAT_ACK;
        }

        ieee80211_tx_status_irqsafe(digi->hw, digi->txPacket);
        digi->txCompleteCount++;
        
        digi->txPacket = NULL;
        ieee80211_wake_queues(digi->hw);
	    digi->txQueueStats.len--;
	    if (digi->txRts)
	    {
	        digi->lowLevelStats.dot11RTSSuccessCount++;
	    }
    } else digi_dbg("(digi->txPacket == NULL)\n");
}
EXPORT_SYMBOL_GPL(digiWifiTxDone);

    

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
    
    del_timer_sync(&digi->txTimer);

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
        struct ieee80211_rate *txRate = getTxRate(digi, txInfo);
        
        if (txRate != NULL)
        {
            #define FRAME_CONTROL_FIELD_OFFSET      (sizeof(struct tx_frame_hdr) + sizeof(struct psk_cck_hdr))
            frameControlFieldType_t *fc = (frameControlFieldType_t *) &digi->txPacket->data[FRAME_CONTROL_FIELD_OFFSET];

            if (digi->txRetryCount[0] != 0) /* is this the first try? */
            {                          
                fc->retry = 1;              /* set retry bit if not*/
            }
    
            digi->write_reg(digi, MAC_BACKOFF, getCw(digi, (digi->txRetryCount[0] == 0)), op_write);

	        /*
	         * Build the H/W transmit header.  The transmit header is rebuilt on each
	         * retry because it has the TX rate information which may change for 
	         * retries.
	         */
	        phy_set_plcp(digi->txPacket->data, 
	                     digi->txPacket->len - TX_HEADER_LENGTH, 
	                     txRate, digi->useAesHwEncryption ? 8 : 0);
	        
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
                /*
                 * We set up a timer to fire in 1/4 second.  We should not need it, but somehow
                 * we seem to miss a timeout interrupt occasionally.  Perhaps we encounter a receive
                 * overrun which causes the H/W to discard the ACK packet without generating
                 * a timeout.
                 */
                digi->txTimer.expires = jiffies + (HZ >> 2);
                add_timer(&digi->txTimer);

                /*
                 * Also set the IRQ mask to listen for timeouts and TX aborts.  We will receive
                 * an ACK (which is handled by the RX routine) if the TX is successful.
                 */
                digi->setIrqMaskBit(digi, BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
            }
        	                
            if (   (digi->txTotalRetries != 0) 
                && ((txInfo->flags & IEEE80211_TX_CTL_NO_ACK) == 0))
            {
                digi->lowLevelStats.dot11ACKFailureCount++;
            }
            digi->txTotalRetries++;
        }
        else
        {
            digiWifiTxDone(digi, OUT_OF_RETRIES, 0);
            /* digi_dbg("All %d retries used up\n", digi->txTotalRetries); */
        }
    }
    else digi_dbg("digi->txPacket == NULL\n");
}

EXPORT_SYMBOL_GPL(digiWifiTxRetryTaskletEntry);
 


