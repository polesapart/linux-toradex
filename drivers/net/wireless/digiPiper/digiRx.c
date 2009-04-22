/*
 * digiRx.c
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
 * This routine is called to receive a frame.  The hardware header has
 * already been read from the FIFO.  We need to read out the frame.  If
 * the frame was encrypted with AES and we have the correct key, then
 * we use the AES H/W encryption engine to decrypt the frame.  We also
 * set up a ieee80211_rx_status structure with the appropriate info.
 *
 * Arguments
 *      digi                context information
 *      skb                 empty buffer to receive packet into
 *      length              number of bytes in FIFO
 *      frameControlField   buffer to copy frame control header into
 *      status              status structure we must write status into
 *      
 * Returns
 *      true            frame was received
 *      false           an encryption error was detected
 */
static bool receivePacket(struct piper_priv *digi, struct sk_buff *skb, int length, 
                          frameControlFieldType_t *frameControlField, 
                          struct ieee80211_rx_status *status)
{
    int headerLength = _80211_HEADER_LENGTH;
#define WANT_RECEIVE_COUNT_SCROLL   (0)
#if WANT_RECEIVE_COUNT_SCROLL
    static int packetCount = 0;
#endif
    _80211HeaderType *header;
    bool result = true;
    int originalLength = length;
    
    if (headerLength > length)
    {
        /*
         * If we branch here, then there is not enough data to make a
         * complete header.  This is possible if this is a control frame.
         * Adjust our length so that we do not read too much data from
         * the FIFO.
         */
        headerLength = length;
    }
    
    /*
     * Read the frame header.  This includes the frame control fields
     * as well as the 802.11 header.
     */
    header = (_80211HeaderType *) skb_put(skb, headerLength);
    length -= headerLength;
    digi->read(digi, BB_DATA_FIFO, (uint8_t *) header, headerLength);
    memcpy(frameControlField, &header->fc, sizeof(frameControlField));

    if (header->fc.protected)
    {
        /*
         * If we branch here, then the frame is encrypted.  We need 
         * to figure out if we should try to decrypt it.
         */
        unsigned char *rsnHeader;
        unsigned int aesDataBlob[AES_BLOB_LENGTH / sizeof(unsigned int)];
        unsigned int keyIndex;
        rsnHeader = skb_put(skb, PIPER_EXTIV_SIZE);

        /*
         * Step 1: Read the rest of the unencrypted data, which should
         *         consist of the extiv fields.
         */
        digi->read(digi, BB_DATA_FIFO, rsnHeader, PIPER_EXTIV_SIZE);
        length -= PIPER_EXTIV_SIZE;
        keyIndex = rsnHeader[3] >> 6;
        
        if (digiWifiPrepareAESDataBlob(digi, keyIndex, (u8 *) aesDataBlob, 
                                    (unsigned char *) header, originalLength - 12, false))
        {
            /*
             * If we come here, then we have the correct encryption key for 
             * the frame and will now try to decrypt it.
             */
#define AES_TIMEOUT         (10000)
            unsigned int timeout = AES_TIMEOUT;
            unsigned long flags;
            
            spin_lock_irqsave(&digi->aesLock, flags);

            /*
             * Step 2: Wait for AES to become ready.
             */
            while (digi->read_reg(digi, BB_RSSI) & BB_RSSI_EAS_BUSY)
            {
                timeout--;
                if (timeout == 0)
                {
                    digi_dbg("1st AES busy never became ready\n");
                    digiWifiDumpRegisters(digi, MAIN_REGS | MAC_REGS);
                }
                udelay(1);
            }
    
            /*
             * Step 3: Set the AES mode, and then read from the AES control
             *         register to put the AES engine into receive mode.
             */
            digi->read_reg(digi, BB_AES_CTL);
            
            /*
             * Step 4: Write the expanded AES key into the AES FIFO.
             */
            digi->write(digi, BB_AES_FIFO, 
                                (unsigned char *) digi->key[keyIndex].expandedKey, 
                                EXPANDED_KEY_LENGTH);
            
            /*
             * Step 5: Write the AES IV and headers into the AES FIFO.
             */
            digi->write(digi, BB_AES_FIFO, (unsigned char *) aesDataBlob,
                                 AES_BLOB_LENGTH);
            
            /*
             * Step 6: Now, finally, read the unencrypted frame from the
             *         AES FIFO.  Adjust the length so that we don't try
             *         to read the MIC or the ICV which the AES engine will
             *         process for us.
             */
            /***/ length -= MIC_SIZE + ICV_SIZE;
            result = digi->read(digi, BB_AES_FIFO, skb_put(skb, length), 
                                length);
            /*
             * mac80211 seems to expect there to be a MIC even if the packet
             * has already been decrypted.  It will drop off what it thinks
             * are the extra MIC bytes, so add some extra bytes that it
             * can drop off without losing any data.
             */
            skb_put(skb, MIC_SIZE);   /* add fake MIC */

            /*
             * Step 7: Wait for AES to become ready.
             */
            while (digi->read_reg(digi, BB_RSSI) & BB_RSSI_EAS_BUSY)
            {
                timeout--;
                if (timeout == 0)
                {
                    digi_dbg("2nd AES busy never became ready\n");
                    digiWifiDumpRegisters(digi, MAIN_REGS | MAC_REGS);
                }
                udelay(1);
            }
            result = ((digi->read_reg(digi, BB_RSSI) & BB_RSSI_EAS_MIC) != 0);
            spin_unlock_irqrestore(&digi->aesLock, flags);
            
            skb_put(skb, 8);          /*  pad an extra 8 bytes for the MIC which the H/W strips*/
            if (result)
            {
                status->flag |= RX_FLAG_DECRYPTED;
            }
            else 
            {
                digi_dbg("Error decrypting packet.\n");
            }
        }
        else 
        {
            /*
             * If we branch here, then we are not able to decrypt the
             * packet possibly because we don't have the key, or because
             * the packet was encrypted using TKIP.  Read the rest of the
             * encrypted data.  mac80211 will have to decrypt it in software.
             */
            digi->read(digi, BB_DATA_FIFO, skb_put(skb, length), length);
         }
     }
    else
    {
        /*
         * Frame is not encrypted, so just read it.
         */
        digi->read (digi, BB_DATA_FIFO, skb_put(skb, length), length);
    }

#if WANT_RECEIVE_COUNT_SCROLL
    if (((++packetCount) & 1023) == 0)
    {
        digi_dbg("%d recd, txStart = %d, txComplete = %d.\n", packetCount, digi->txStartCount, digi->txCompleteCount);
        if (digi->txPacket)
        {
            struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(digi->txPacket);
            
            digi_dbg("TxPacket yes!, totalRetries = %d, flags = 0x%8.8X\n", digi->txTotalRetries, txInfo->flags);
        }
        else
        {
            digi_dbg("TxPacket == NULL, totalRetries = %d\n", digi->txTotalRetries);
        }
    }
#endif

    return result;
}


/*
 * This routine is called when we receive an ACK.  This should be after
 * we have transmitted a packet.  We need to tell the upper layer we
 * have a packet by calling ieee80211_tx_status_irqsafe with status
 * information.  The transmit routine also disables queuing whenever we
 * transmit since we can only transmit one packet at a time, so we need
 * to reenable to transmit queue too.
 */
static void handleAck(struct piper_priv *digi, int signalStrength)
{
    if (digi->txPacket) 
    {
        struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(digi->txPacket);
        if ((txInfo->flags & IEEE80211_TX_CTL_NO_ACK) == 0)
        {
    	    digi->clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	        digiWifiTxDone(digi, RECEIVED_ACK, signalStrength);
        }
    }
}




/*
 * This is the entry point for the receive tasklet.  It is executed
 * to process receive packets.  It allocates an SKB and receives
 * the packet into it.
 *
 * We may be called from the receive ISR if WANT_TO_RECEIVE_FRAMES_IN_ISR
 * is set.
 */
void digiWifiRxTaskletEntry (unsigned long context)
{
    struct piper_priv *digi = (struct piper_priv *) context;
    
    /*
     * This while loop will keep executing as long as the H/W indicates there
     * are more frames in the FIFO to be received.
     */
    while ((digi->read_reg(digi, BB_GENERAL_STAT) & BB_GENERAL_STAT_RX_FIFO_EMPTY) == 0)
    {
	    struct sk_buff *skb = NULL;
		struct ieee80211_rx_status status = {0};
	    struct rx_frame_hdr header;
	    unsigned int length = 0;
        frameControlFieldType_t frameControlField;
	    
		skb = __dev_alloc_skb(RX_FIFO_SIZE, GFP_ATOMIC);
		if (skb == NULL)
		{
		    /*
		     * Oops.  Out of memory.  Exit the tasklet.
		     */
		    digi_dbg("__dev_alloc_skb failed\n");
		    break;
		}
        /*
         * Read and process the H/W header.  This header is created by
         * the hardware is is not part of the frame.
         */
        digi->read(digi, BB_DATA_FIFO, (uint8_t *) &header, sizeof(header));
        phy_process_plcp(digi, &header, &status, &length);

		if (length != 0)
		{
    		if (receivePacket(digi, skb, length, &frameControlField, &status))
    	    {
        		if (frameControlField.type == TYPE_ACK)
        		{
        		    handleAck(digi, status.signal);
        		}
        		if (   (frameControlField.type == TYPE_ACK)
        		    || (frameControlField.type == TYPE_RTS)
        		    || (frameControlField.type == TYPE_CTS))
        		{
        		    /*
        		     * Don't pass up RTS, CTS, or ACK frames.  They just confuse
        		     * the stack.
        		     */
        		    dev_kfree_skb(skb);
        		}
        		else
        		{
        		    if (frameControlField.type == TYPE_BEACON)
        		    {
        		        digi->beacon.weSentLastOne = false;
        		    }
#if WANT_TO_RECEIVE_FRAMES_IN_ISR
                    ieee80211_rx_irqsafe(digi->hw, skb, &status);  
#else
        		    ieee80211_rx(digi->hw, skb, &status);
#endif
        		}
        	}
        	else
        	{
        	    /*
        	     * Frame failed MIC, so discard it.
        	     */
        	    digi_dbg("Dropping bad frame\n");
    		    dev_kfree_skb(skb);
    		}
         }
        else
        {
            dev_kfree_skb(skb);
            digi_dbg("Ignoring 0 length packet\n");
        }
    }

    digi->setIrqMaskBit(digi, BB_IRQ_MASK_RX_FIFO);
}

EXPORT_SYMBOL_GPL(digiWifiRxTaskletEntry);
        


