/*
 * minimal.c
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

#include "pipermain.h"
#include "mac.h"
#include "piperhwinit.h"
#include "phy.h"
#include "airoha.h"


#define	DRIVER_VERSION	"0.1"

#define MAC_REG_BASE	(0x70000000)		/* Register base address*/
#define MAC_REG_SIZE    (0x100)             /* range of MAC registers*/
#define MAC_CTRL_BASE   (volatile unsigned int *) (MAC_REG_BASE + (0x40))

#define WANT_DEBUG_THREAD   (0)

static struct piper_priv *localCopyDigi = NULL;

#define DUMP_WORDS_MAX      (40)
static unsigned int dumpWordsWord[DUMP_WORDS_MAX];
static unsigned int dumpWordsCount = 0;
static void dumpWordsAdd(unsigned int word)
{
    if (dumpWordsCount < DUMP_WORDS_MAX)
    {
        dumpWordsWord[dumpWordsCount++] = word;
    }
}
static void dumpWordsDump(void)
{
    unsigned int *p = dumpWordsWord;
    unsigned int wordsToGo = dumpWordsCount;
    
    dumpWordsCount = 0;
    
    while (wordsToGo > 4)
    {
        digi_dbg("%8.8X %8.8X - %8.8X %8.8X\n", p[0], p[1], p[2], p[3]);
        p += 4;
        wordsToGo -= 4;
    }
    if (wordsToGo == 3)
    {
        digi_dbg("%8.8X %8.8X - %8.8X\n", p[0], p[1], p[2]);
    }
    else if (wordsToGo == 2)
    {
        digi_dbg("%8.8X %8.8X \n", p[0], p[1]);
    }
    else if (wordsToGo == 1)
    {
        digi_dbg("%8.8X \n", p[0]);
    }
}

void dumpSkb(struct sk_buff *skb)
{
    unsigned int bytesLeft = skb->len;
    unsigned char *p = skb->data;
        
    digi_dbg("skb has %d bytes\n", skb->len);
    while (bytesLeft >= 16)
    {
        digi_dbg("%2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X - %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X\n",
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
        p += 16;
        bytesLeft -= 16;
    }
    if (bytesLeft >= 8)
    {
        digi_dbg("%2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X\n",
                 p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        p += 8;
        bytesLeft -= 8;
    }
    if (bytesLeft >= 4)
    {
        digi_dbg("%2.2X %2.2X %2.2X %2.2X \n",
                 p[0], p[1], p[2], p[3]);
        p += 4;
        bytesLeft -= 4;
    }
    if (bytesLeft >= 2)
    {
        digi_dbg("%2.2X %2.2X \n",
                 p[0], p[1]);
        p += 2;
        bytesLeft -= 2;
    }
    if (bytesLeft >= 1)
    {
        digi_dbg("%2.2X \n",
                 p[0]);
        p += 1;
        bytesLeft -= 1;
    }
}

EXPORT_SYMBOL_GPL(dumpSkb);
 

void dumpRegisters(struct piper_priv *digi, unsigned int regs)
{
    unsigned int i;
    
    if (regs & CTRL_STATUS_REGS)
    {
                digi_dbg("  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", "Gen Ctrl", digi->read_reg(digi, BB_GENERAL_CTL), 
                         "Gen Status", digi->read_reg(digi, BB_GENERAL_STAT));
    }
    else if (regs & IRQ_REGS)
    {
                digi_dbg("  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", "IRQ Mask", digi->read_reg(digi, BB_IRQ_MASK), 
                         "IRQ Status", digi->read_reg(digi, BB_IRQ_STAT));
    }
    else if (regs & MAIN_REGS)
    {
        const char *regNames[] = {"Version", "Gen Ctrl", "Gen Status", "RSSI/AES",
                                  "Int Mask", "Int Status", "SPI Data", "SPI Ctrl",
                                  "Data FIFO", "not used", "conf-1", "conf-2", "AES FIFO",
                                "not used", "AES Ctrl", "IO Ctrl"};
        digi_dbg("Main Registers:\n");
        for (i = BB_VERSION; i <= BB_OUTPUT_CONTROL; i = i+8)
        {
            if ((i != BB_DATA_FIFO) && (i != BB_AES_FIFO))
            {
                digi_dbg("  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", regNames[i>>2], digi->read_reg(digi, i), regNames[(i>>2) + 1], digi->read_reg(digi, i+4));
            }
        }
    }
    if (regs & MAC_REGS)
    {
        const char *regNames[] = {"STA ID0", "STA ID1", "BSS ID0", "BSS ID1",
                                  "OFDM/PSK", "Backoff", "DTIM/List", "B Int",
                                "Rev/M Stat", "C C/M CTL", "Measure", "Beac Fltr"};
                               
        digi_dbg("Secondary Registers:\n");
        for (i = MAC_STA_ID0; i <= MAC_BEACON_FILT; i = i+8)
        {
            digi_dbg("  %10.10s = 0x%8.8X  %10.10s = 0x%8.8X\n", regNames[((i - MAC_STA_ID0) >>2)], digi->read_reg(digi, i), regNames[((i - MAC_STA_ID0)>>2) + 1], digi->read_reg(digi, i+4));
        }
    }
    if (regs & FRAME_BUFFER_REGS)
    {
        unsigned int word[4];
        digi_dbg("Real time frame buffer\n");
            
        word[0] = be32_to_cpu(digi->read_reg(digi, 0xc0));
        word[1] = be32_to_cpu(digi->read_reg(digi, 0xc4));
        word[2] = be32_to_cpu(digi->read_reg(digi, 0xc8));
        word[3] = be32_to_cpu(digi->read_reg(digi, 0xcc));
        digi_dbg(" %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
        word[0] = be32_to_cpu(digi->read_reg(digi, 0xd0));
        word[1] = be32_to_cpu(digi->read_reg(digi, 0xd4));
        word[2] = be32_to_cpu(digi->read_reg(digi, 0xd8));
        word[3] = be32_to_cpu(digi->read_reg(digi, 0xdc));
        digi_dbg(" %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
    }
    if (regs & FIFO_REGS)
    {
        unsigned int word[4];
        digi_dbg("FIFO contents\n");
            
        word[0] = digi->read_reg(digi, BB_DATA_FIFO);
        word[1] = digi->read_reg(digi, BB_DATA_FIFO);
        word[2] = digi->read_reg(digi, BB_DATA_FIFO);
        word[3] = digi->read_reg(digi, BB_DATA_FIFO);
        digi_dbg(" %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
        word[0] = digi->read_reg(digi, BB_DATA_FIFO);
        word[1] = digi->read_reg(digi, BB_DATA_FIFO);
        word[2] = digi->read_reg(digi, BB_DATA_FIFO);
        word[3] = digi->read_reg(digi, BB_DATA_FIFO);
        digi_dbg(" %8.8X %8.8X - %8.8X %8.8X\n", word[0], word[1], word[2], word[3]);
    }
}       


/*
 * TODO: make a fast version of this which works for single register writes
 */
static int piper_write(struct piper_priv *digi, uint8_t addr, uint8_t *buf,
		int len)
{
    int wordIndex;
    int wordLength = (len + 3) / sizeof(unsigned int);
                        
    if (((unsigned) (buf) & 0x3) == 0)
    {
        if (wordLength == 1)
        {
            unsigned int *word = (unsigned int *) buf;
            
            iowrite32(cpu_to_be32(*word), digi->vbase + addr);
        }
        else
        {
            for (wordIndex = 0; wordIndex < wordLength; wordIndex++)
            {
                unsigned int *word = (unsigned int *) buf;
                
                iowrite32(cpu_to_be32(word[wordIndex]), digi->vbase + addr);
            }
        }
    }
    else
    {
        for (wordIndex = 0; wordIndex < wordLength; wordIndex++)
        {
            unsigned int word;
            
            memcpy(&word, &buf[wordIndex*sizeof(unsigned int)], sizeof(unsigned int));
            
            iowrite32(cpu_to_be32(word), digi->vbase + addr);
        }
    }
    return 0;
}


static int piper_write_reg(struct piper_priv *digi, uint8_t reg, uint32_t val)
{    
    /* TODO: Eliminate this */
    iowrite32(val, digi->vbase + reg);
	return 0;
}


struct ccmp_mic {
	uint8_t mic_iv[16];
	uint8_t mic_hdr[32];
};

static int prepare_aes_data(struct piper_priv *digi, struct sk_buff *skb,
		int keyidx)
{
	struct ccmp_mic mic;
	uint8_t pn[6], *ptr;
	int hdrlen, err;

	/* select mode 1 and key */
	err = digi->write_reg(digi, BB_AES_CTL, BB_AES_CTL_AES_MODE | keyidx);
	if (err) {
		digi_dbg("unable to set aes mode/key\n");
		goto done;
	}

	/* write beginning of data frame to TX FIFO; 10 words */
	err = digi->write(digi, BB_DATA_FIFO, skb->data, 40);
	if (err)
		goto done;

	/* pop off any headers that we've added */
	skb_pull(skb, sizeof(struct psk_cck_hdr) + sizeof(struct tx_frame_hdr));

	/* at this point, the skb points back to the mac header.  right
	 * after the MAC header there should be a CCMP header (created
	 * by the mac80211 stack via GENERATE_IV) */
	hdrlen = ieee80211_get_hdrlen_from_skb(skb) ;
	ptr = skb->data + hdrlen;
	pn[0] = ptr[7];
	pn[1] = ptr[6];
	pn[2] = ptr[5];
	pn[3] = ptr[4];
	pn[4] = ptr[1];
	pn[5] = ptr[0];

#if 1
    printk(KERN_DEBUG "H/W AES not yet implemented.\n");
#else
	/* generate mic_iv and mic_hdr */
	ieee80211_crypto_ccmp_mic_blocks(skb, pn, (uint8_t *) mic.mic_iv,
			(uint8_t *) mic.mic_hdr, 0);
#endif
	/* drop the MAC header and CCMP header */
	skb_pull(skb, hdrlen + 8);

	/* replace with MIC IV and MIC header */
	skb_push(skb, sizeof(mic));
	memcpy(skb->data, (uint8_t *) &mic, sizeof(mic));

done:
	return err;
}


static int __piper_write_fifo(struct piper_priv *digi, struct sk_buff *skb,
		int hw_aes)
{
    uint8_t addr;
    
    if (hw_aes)
    {
        addr = BB_AES_FIFO;
    }
    else
    {
        addr = BB_DATA_FIFO;
    }

    return piper_write(digi, addr, skb->data, skb->len);
}


static void clearIrqMaskBit(struct piper_priv *piper, unsigned bits)
{
    unsigned int newMask;
    unsigned long flags;
    
    spin_lock_irqsave(&piper->irqMaskLock, flags);
    newMask = piper->read_reg(piper, BB_IRQ_MASK) & (~bits);
    piper->write_reg(piper, BB_IRQ_MASK, newMask);
    spin_unlock_irqrestore(&piper->irqMaskLock, flags);
}


static void setIrqMaskBit(struct piper_priv *piper, unsigned bits)
{
    unsigned int newMask;
    unsigned long flags;
    
    spin_lock_irqsave(&piper->irqMaskLock, flags);
    newMask = piper->read_reg(piper, BB_IRQ_MASK) | bits;
    piper->write_reg(piper, BB_IRQ_MASK, newMask);
    spin_unlock_irqrestore(&piper->irqMaskLock, flags);
}





static int piper_write_fifo(struct piper_priv *digi, struct sk_buff *skb,
            struct ieee80211_tx_control *ctl)
{
	int err;

	/* Hardware workaround; sometimes it starts pulling from the
	 * TX FIFO before we've finished sending it the entire packet.
	 * To deal with that we set the TX_HOLD bit, which will force
	 * the hardware to wait for us to finish writing to the FIFO */
	digi->write_reg(digi, BB_GENERAL_CTL, 
	                BB_GENERAL_CTL_TX_HOLD | digi->read_reg(digi, BB_GENERAL_CTL));

    if (ctl->flags & IEEE80211_TXCTL_NO_ACK)
    {
        setIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TX_ABORT);
    }
    else
    {
        setIrqMaskBit(digi, BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
    }
	err = __piper_write_fifo(digi, skb, 0);
	if (err)
		goto done;

	/* write_reg will start the transmit */
	err = digi->write_reg(digi, BB_GENERAL_CTL, 
	                ~BB_GENERAL_CTL_TX_HOLD & digi->read_reg(digi, BB_GENERAL_CTL));
done:
	return err;
}

static int piper_write_aes(struct piper_priv *digi, struct sk_buff *skb,
		int keyidx, struct ieee80211_tx_control *ctl)
{
	int err;

	/* Hardware workaround; sometimes it starts pulling from the
	 * TX FIFO before we've finished sending it the entire packet.
	 * To deal with that we set the TX_HOLD bit, which will force
	 * the hardware to wait for us to finish writing to the FIFO */
	digi->write_reg(digi, BB_GENERAL_CTL, 
	                BB_GENERAL_CTL_TX_HOLD | digi->read_reg(digi, BB_GENERAL_CTL));
    if (ctl->flags & IEEE80211_TXCTL_NO_ACK)
    {
        setIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TX_ABORT);
    }
    else
    {
        setIrqMaskBit(digi, BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
    }
	err = prepare_aes_data(digi, skb, keyidx);
	if (err)
		goto done;
	err = __piper_write_fifo(digi, skb, 1);
	if (err)
		goto done;

	/* write_reg will start the transmit*/
	err = digi->write_reg(digi, BB_GENERAL_CTL, 
	                ~BB_GENERAL_CTL_TX_HOLD & digi->read_reg(digi, BB_GENERAL_CTL));

done:
	return err;
}




static void txRetryTaskletEntry (unsigned long context)
{
    struct piper_priv *digi = (struct piper_priv *) context;
    int err;
    static struct sk_buff *debugFrame = NULL;
    
    if (debugFrame == NULL)
    {
        debugFrame = __dev_alloc_skb(RX_FIFO_SIZE, GFP_ATOMIC);
        if (debugFrame)
        {
            memcpy(skb_put(debugFrame, digi->txPacket->len), digi->txPacket->data, digi->txPacket->len);
        }
    }
    /*
     * Clear flags here to cover ACK case.  We do not clear the flags in the ACK 
     * routine since it is possible to receive an ACK after we have started the
     * next packet.  The appropriate interrupts will be reenabled if we decide
     * to retransmit.
     */
	clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	if (digi->txPacket != NULL)
	{
        if (digi->txRetries != 0)
        {
#if 0
    	    int aes_fifo = digi->txStatus.control.flags & IEEE80211_TXCTL_DO_NOT_ENCRYPT ? 0 : 1;
#else
    	    int aes_fifo = 0;
#endif
            if (digi->txRetries != digi->txStatus.control.retry_limit)
            {
                #define FRAME_CONTROL_FIELD_OFFSET      (sizeof(struct tx_frame_hdr) + sizeof(struct psk_cck_hdr))
                frameControlFieldType_t *fc = (frameControlFieldType_t *) &digi->txPacket->data[FRAME_CONTROL_FIELD_OFFSET];
                digi->txStatus.retry_count++;
                fc->retry = 1;              /* set retry bit */
            }
    
            digi->txRetries--; 
/*
 * TODO:  Implement tx_conf function to receive contention window size.
 */
    /****/ digi->write_reg(digi, MAC_BACKOFF, 0x118);
        	if (aes_fifo)
        		err = digi->write_aes(digi, digi->txPacket, digi->txStatus.control.key_idx, &digi->txStatus.control);
        	else
        		err = digi->write_fifo(digi, digi->txPacket, &digi->txStatus.control);
       /* TODO: What to do if err != 0 */
        }
        else
        {
            digi->txStatus.excessive_retries = true;
            digi->txStatus.flags = 0;
            digi_dbg("All %d retries used up, ieee80211_tx_status_irqsafe\n", digi->txStatus.control.retry_limit);
            ieee80211_tx_status_irqsafe(digi->hw,
            				           digi->txPacket,
            				           &digi->txStatus);
            digi->txPacket = NULL;
        }
    }
}

static int piper_write_aes_key(struct piper_priv *digi, struct sk_buff *skb)
{
	int err;

	/* Hardware workaround; sometimes it starts pulling from the
	 * TX FIFO before we've finished sending it the entire packet.
	 * To deal with that we set the TX_HOLD bit, which will force
	 * the hardware to wait for us to finish writing to the FIFO */
	/* TODO:  Does this make sense?  Is this a bug caused by copying code
	          from piper_write? */
	err = digi->write_reg(digi, BB_GENERAL_CTL, 
	                BB_GENERAL_CTL_TX_HOLD | digi->read_reg(digi, BB_GENERAL_CTL));
	if (err)
		goto done;

	err = __piper_write_fifo(digi, skb, 1);
	if (err)
		goto done;

	/* write_reg will kick the tx_thread to start processing packets if
	 * it's not already active */
	err = digi->write_reg(digi, BB_GENERAL_CTL, 
	                ~BB_GENERAL_CTL_TX_HOLD & digi->read_reg(digi, BB_GENERAL_CTL));
	
done:
	return err;
}




static int piper_read(struct piper_priv *digi, uint8_t addr, uint8_t *buf,
		int len)
{

    int wordIndex;
    
/*
 * We can only read 32-bit words, so round the length up to an even multiple of
 * 4 if necessary.
 */
    len += 3;
    len &= 0xfffffc;
    
    if ((len == 4) && ((((unsigned)buf) & 0x3) == 0))
    {
        unsigned *word = (unsigned *)buf;
        
        *word = be32_to_cpu(ioread32(digi->vbase + addr));
    }
    else if ((((unsigned)buf) & 0x3) == 0)
    {
        unsigned *word = (unsigned *)buf;
        
        for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++)
        {
            word[wordIndex] = be32_to_cpu(ioread32(digi->vbase + addr));
        }
    }
    else
    {
        for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++)
        {
            unsigned word = be32_to_cpu(ioread32(digi->vbase + addr));
            memcpy(&buf[wordIndex * sizeof(unsigned)], &word, sizeof(word));
        }
    }
    
    return 0;
}



static unsigned int piper_read_reg(struct piper_priv *digi, uint8_t reg)
{
    return ioread32(digi->vbase + reg);

	return 0;
}





/**
 * Interrupt handler for Piper IRQs.
 */
static irqreturn_t piperIsr(int irq, void *dev_id)
{
	struct piper_priv *piper = dev_id;
	unsigned int status;
    
	/* acknowledge interrupt */
	status = piper->read_reg(piper, BB_IRQ_STAT);
    status &= piper->read_reg(piper, BB_IRQ_MASK);
	piper->write_reg(piper, BB_IRQ_STAT, status);
    
	if (status & BB_IRQ_MASK_RX_FIFO) 
	{
	    /* 
	     * Wake up the tasklet and mask off further interrupts until
	     * the tasklet reenables them.
	     */
	    clearIrqMaskBit(piper, BB_IRQ_MASK_RX_FIFO); 
        tasklet_hi_schedule(&piper->rxTasklet);
	} 

    if (status & BB_IRQ_MASK_TX_FIFO_EMPTY) 
    {
        digi_dbg("Got tx FIFO empty\n");
        /*
         * Transmit complete interrupt.  This IRQ is only unmasked if we are
         * not expecting the packet to be ACKed.
         */
	    if (piper->txPacket != NULL)
	    {
            piper->txStatus.excessive_retries = false;
            piper->txStatus.flags = 0;
            digi_dbg("Calling ieee80211_tx_status_irqsafe\n");
            ieee80211_tx_status_irqsafe(piper->hw,
            				           piper->txPacket,
            				           &piper->txStatus);
            piper->txPacket = NULL;
            ieee80211_start_queues(piper->hw);
        }
		clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
    }

	if (status & BB_IRQ_MASK_TIMEOUT)
	{
        digi_dbg("Got tx timeout\n");
	    /*
	     * AP did not ACK our TX packet.
	     */
	    if (piper->txPacket != NULL)
	    {
	        tasklet_schedule(&piper->txRetryTasklet);
        }
		clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

	if (unlikely(status & BB_IRQ_MASK_TX_ABORT)) 
	{
        digi_dbg("Got tx abort\n");
	    /*
	     * Could not transmit a packet because the media was busy.
	     */
	    if (piper->txPacket != NULL)
	    {
	        tasklet_schedule(&piper->txRetryTasklet);
        }
		clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

	if (status & (BB_IRQ_MASK_TBTT | BB_IRQ_MASK_ATIM)) 
	{
        digi_dbg("Got BB_IRQ_MASK_TBTT | BB_IRQ_MASK_ATIM\n");
		clearIrqMaskBit(piper, (BB_IRQ_MASK_TBTT | BB_IRQ_MASK_ATIM));
	}

	if (unlikely(status & BB_IRQ_MASK_RX_OVERRUN)) 
	{
        digi_dbg("Got rx overrun\n");
		piper->rxOverruns++;
	}

	return IRQ_HANDLED;
}


static void receivePacket(struct piper_priv *piper, struct sk_buff *skb, int length, 
                          frameControlFieldType_t *frameControlField)
{
    // Use hardware AES if encrypted with AES
    int headerLength = _80211_HEADER_LENGTH;
#define WANT_RECEIVE_COUNT_SCROLL   (0)
#if WANT_RECEIVE_COUNT_SCROLL
    static int packetCount = 0;
#endif
    _80211HeaderType *header;
    if (headerLength > length)
    {
        headerLength = length;
    }
    
    header = (_80211HeaderType *) skb_put(skb, headerLength);
    length -= headerLength;
    piper_read(piper, BB_DATA_FIFO, (uint8_t *) header, headerLength);
    memcpy(frameControlField, &header->fc, sizeof(frameControlField));
    
    if (header->fc.protected)
    {
#ifdef WANT_HW_AES
        Can check for presence of key to determine if AES or not
/* TODO: Implement support for H/W AES */
        // Get AES key, init block, headers
        if (CcmpGetData (header->addr1, buf, &data))
        {
            while (HW_RSSI_AES & AES_BUSY)
                ;
            // Set receive mode
            tmp = HW_AES_MODE;

            // Write key, init vector, headers to AES FIFO
            HWWriteAES (data.key->rd_key, 4*44);
            HWWriteAES (data.init, 4*12);

            // Get plaintext data from AES FIFO
            HWReadAES (buf->body+EXTIV_SIZE, buf->length-DATA_SIZE-CCMP_SIZE);

            while (HW_RSSI_AES & AES_BUSY)
                ;

            // Check if MIC is correct
            if (!(HW_RSSI_AES & AES_MIC))
            {
                macStats.rxDropDecrypt++;
                macStats.rxDropMICFail++;
                return FALSE;
            }
        }

        // Not AES, get remaining data from receive FIFO
        else
#endif
        {
            piper_read (piper, BB_DATA_FIFO, skb_put(skb, length), length);
        }
    }

    // Get remaining data from receive FIFO
    else
    {
        piper_read (piper, BB_DATA_FIFO, skb_put(skb, length), length);
    }

#if WANT_RECEIVE_COUNT_SCROLL
    if (((++packetCount) & 31) == 0)
    {
        digi_dbg("%d packets received so far.\n", packetCount);
    }
#endif
}


/*
 * This routine is called when we receive an ACK.  This should be after
 * we have transmitted a packet.  We need to tell the upper layer we
 * have a packet by calling ieee80211_tx_status_irqsafe with status
 * information.  The transmit routine also disables queuing whenever we
 * transmit since we can only transmit one packet at a time, so we need
 * to reenable to transmit queue too.
 */
static void handleAck(struct piper_priv *piper, int signalStrength)
{
    struct ieee80211_tx_status status;
    
    memset(&status, 0, sizeof(status));
    
    if ((piper->txPacket) && ((piper->txStatus.control.flags & IEEE80211_TXCTL_NO_ACK) == 0))
    {
	    clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
     	piper->txStatus.flags = IEEE80211_TX_STATUS_ACK;
    	piper->txStatus.retry_count = 0;
    	piper->txStatus.excessive_retries = false;
    	piper->txStatus.ampdu_ack_len = 0;
    	piper->txStatus.ampdu_ack_map = 0;
    	piper->txStatus.ack_signal = signalStrength;
    	
        ieee80211_tx_status_irqsafe(piper->hw, piper->txPacket, &piper->txStatus);
        piper->txPacket = NULL;
        /* TODO:  Is it possible to get an ACK when we are not expecting one and so
                  screw up our system?  Should txPacket and friends be protected with
                  a spinlock?
         */
        ieee80211_start_queues(piper->hw);
    }
}


#if 0
        TODO: Deal with this problem
    	if (ALIGN(length, 4) - skb->len == 8) 
    	{
    		/* we can assume this is an AES packet; the 8 bytes is the
    		 * MIC which was stripped in hardware.  The mac80211 stack
    		 * doesn't attempt to verify the MIC if we set
    		 * RX_FLAG_DECRYPTED, but it does strip the 8 byte MIC from
    		 * the end of the packet.  For now, we append a garbage 8 bytes
    		 * that mac80211 later trims.  It smells an awful lot like a
    		 * bug.. */
    		status.flag |= RX_FLAG_DECRYPTED;
    		skb_put(skb, 8);
    	} 
    	else if (ALIGN(length, 4) != (skb->len)) 
    	{
    		printk(KERN_ERR DRV_NAME
    				": rx plcp length incorrect (%d vs %d)!\n",
    				ALIGN(length, 4), skb->len);
    		status.flag |= RX_FLAG_FAILED_PLCP_CRC;
    	}

#endif


static void rxTaskletEntry (unsigned long context)
{
    struct piper_priv *piper = (struct piper_priv *) context;
    
    while ((piper->read_reg(piper, BB_GENERAL_STAT) & BB_GENERAL_STAT_RX_FIFO_EMPTY) == 0)
    {
	    struct sk_buff *skb = NULL;
		struct ieee80211_rx_status status = {0};
	    struct rx_frame_hdr header;
	    unsigned int length = 0;
        frameControlFieldType_t frameControlField;
	    
		skb = __dev_alloc_skb(RX_FIFO_SIZE, GFP_ATOMIC);
		if (skb == NULL)
		{
		    digi_dbg("__dev_alloc_skb failed\n");
		    break;
		}
        piper_read(piper, BB_DATA_FIFO, (uint8_t *) &header, sizeof(header));
        phy_process_plcp(piper, &header, &status, &length);

		if (length != 0)
		{
    		receivePacket(piper, skb, length, &frameControlField);
    		if (frameControlField.type == TYPE_ACK)
    		{
    		    digi_dbg("Received an ACK\n");
    		    handleAck(piper, status.signal);
    		}
    		if ((skb->data[4] & 1) == 0)
    		{
    		    digi_dbg("Received unicast packet\n");
    		}
    		ieee80211_rx(piper->hw, skb, &status);
         }
        else
        {
            dev_kfree_skb(skb);
            digi_dbg("Ignoring 0 length packet\n");
        }
    }

    setIrqMaskBit(piper, BB_IRQ_MASK_RX_FIFO);
}

        



#if WANT_DEBUG_THREAD

static int debugThreadEntry(void *data)
{
	struct piper_priv *piper = data;

    ssleep(20);
    
    while (debugFrame == NULL)
    {
        ssleep(1);
    }
    
    while (1)
    {
        

    return 0;
}
#endif



static int init_rx_tx(struct piper_priv *piper)
{
#if WANT_DEBUG_THREAD
    struct task_struct *debugThread;
#endif

    tasklet_init(&piper->rxTasklet, rxTaskletEntry, (unsigned long) piper);
    tasklet_disable(&piper->rxTasklet);
    
    piper->txPacket = NULL;
    tasklet_init(&piper->txRetryTasklet, txRetryTaskletEntry, (unsigned long) piper);
    tasklet_disable(&piper->txRetryTasklet);

#if WANT_DEBUG_THREAD
	debugThread = kthread_run(debugThreadEntry, piper, PIPER_DRIVER_NAME " - debug");
#endif
    return 0;
}

static void free_rx_tx(struct piper_priv *piper)
{
    tasklet_disable(&piper->rxTasklet);
    tasklet_kill(&piper->rxTasklet);
    tasklet_disable(&piper->txRetryTasklet);
    tasklet_kill(&piper->txRetryTasklet);
}





static int initHw(struct piper_priv *digi)
{
#define WLN_BAND_B      (0)
#define WLN_BAND_BG     (1)
#define WLN_BAND_A      (2)
    int band = WLN_BAND_BG;
    int result = 0;
    
#if (TRANSCEIVER == RF_AIROHA_7230)
        /* Initialize baseband general control register */
        if ((band == WLN_BAND_B) || (band == WLN_BAND_BG))
        {
            digi->write_reg(digi, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ);
            digi->write_reg(digi, BB_TRACK_CONTROL, 
                            0xff00ffff & digi->read_reg(digi, BB_TRACK_CONTROL));
            digi->write_reg(digi, BB_TRACK_CONTROL, 
                            TRACK_BG_BAND | digi->read_reg(digi, BB_TRACK_CONTROL));
            digi_dbg("initHw Initialized for band B / BG\n");
        }
        else
        {
            digi->write_reg(digi, BB_GENERAL_CTL, GEN_INIT_AIROHA_50GHZ);
            digi->write_reg(digi, BB_TRACK_CONTROL, 
                            0xff00ffff & digi->read_reg(digi, BB_TRACK_CONTROL));
            digi->write_reg(digi, BB_TRACK_CONTROL, 
                            TRACK_5150_5350_A_BAND | digi->read_reg(digi, BB_TRACK_CONTROL));
            digi_dbg("initHw Initialized for band A\n");
        }
        
        digi->write_reg(digi, BB_CONF_2, 0x08329AD4);
 
        /* Initialize the SPI word length */	
        digi->write_reg(digi, BB_SPI_CTRL, SPI_INIT_AIROHA);
#elif (TRANSCEIVER == RF_AIROHA_2236)

        /* Initialize baseband general control register */
        digi->write_reg(digi, BB_GENERAL_CTL, GEN_INIT_AIROHA_24GHZ);
      
        digi->write_reg(digi, BB_CONF_2, 0x08329AD4);
        
        digi->write_reg(digi, BB_TRACK_CONTROL, 
                        0xff00ffff & digi->read_reg(digi, BB_TRACK_CONTROL));
        digi->write_reg(digi, BB_TRACK_CONTROL, 
                        TRACK_BG_BAND | digi->read_reg(digi, BB_TRACK_CONTROL));
      
        /* Initialize the SPI word length */	   
        digi->write_reg(digi, BB_SPI_CTRL, SPI_INIT_AIROHA2236);
#endif
    // Clear the Interrupt Mask Register before enabling external interrupts.
    // Also clear out any status bits in the Interrupt Status Register.
    digi->write_reg(digi, BB_IRQ_MASK, 0);
    digi->write_reg(digi, BB_IRQ_STAT, 0xff);
    
    /*
     * If this firmware supports additional MAC addresses.
     */
    if (((digi->read_reg(digi, MAC_STATUS) >> 16) & 0xff) >= 8)
    {
        /* Disable additional addresses to start with */
        digi->write_reg(digi, MAC_CTL, ~MAC_CTL_MAC_FLTR & digi->read_reg(digi, MAC_CTL));
        /*
         * Clear registers that hold extra addresses.
         */
        
        digi->write_reg(digi, MAC_STA2_ID0, 0);
        digi->write_reg(digi, MAC_STA2_ID1, 0);
        digi->write_reg(digi, MAC_STA3_ID0, 0);
        digi->write_reg(digi, MAC_STA3_ID1, 0);
    }
/*
 * TODO:  Set this register programatically.
 */
/****/ digi->write_reg(digi, MAC_DTIM_PERIOD, 0x0);
    
    digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_ANT_DIV | digi->read_reg(digi, BB_GENERAL_CTL));
    digi->write_reg(digi, BB_RSSI, 0x96000000 | digi->read_reg(digi, BB_RSSI));
#if 1
    digi->write_reg(digi, BB_GENERAL_CTL, 0x96000000 | digi->read_reg(digi, BB_GENERAL_CTL));
#else
/* TODO:  Deal with this when antenna div changed */
//on connectcorewi9p9215_a platform
        if (HW_GEN_CONTROL & GEN_ANTDIV)
            HW_RSSI_AES |= 0x96000000;  
        else
        {
            HW_RSSI_AES |= 0x1E000000; 
            HW_GEN_CONTROL &= 0xfffffffb;
        }
#endif

    // reset RX and TX FIFOs
    digi->write_reg(digi, BB_GENERAL_CTL, digi->read_reg(digi, BB_GENERAL_CTL) 
                | BB_GENERAL_CTL_RXFIFORST | BB_GENERAL_CTL_TXFIFORST);
    digi->write_reg(digi, BB_GENERAL_CTL, digi->read_reg(digi, BB_GENERAL_CTL) 
                & ~(BB_GENERAL_CTL_RXFIFORST | BB_GENERAL_CTL_TXFIFORST));

/* TODO:  Is this a bug? */
    digi->write_reg(digi, BB_TRACK_CONTROL, 0xC043002C);  
        
    /* Initialize RF transceiver */
    if (band == WLN_BAND_A)
    {
        digi->rf->init(digi->hw, IEEE80211_BAND_5GHZ);
    }
    else
    {
        digi->rf->init(digi->hw, IEEE80211_BAND_2GHZ);
    }
    digi_dbg("forcing value of BB_OUTPUT_CONTROL to be 0x04000301\n");
/****/ digi->write_reg(digi, BB_OUTPUT_CONTROL, 0x04000301);
/****/ digi->write_reg(digi, MAC_CFP_ATIM, 0x0);
    digi->write_reg(digi, BB_GENERAL_STAT, 
        digi->read_reg(digi, BB_GENERAL_STAT) &
        ~(BB_GENERAL_STAT_DC_DIS | BB_GENERAL_STAT_SPRD_DIS));

#ifdef AIROHA_PWR_CALIBRATION
    initPwrCal();
#endif

        
    
#ifdef MAC_PS_ENABLED
    PIO_OUTPUT(WLN_PS_CNTRL_PIN);
#endif

    return result;
}


static int __init piper_probe(struct platform_device* pdev)
{
	int err = 0;
	struct piper_priv *digi;
    unsigned int version, status;
    
	pr_info("piper_probe called\n");
	err = piper_alloc_hw(&digi, sizeof(*digi));
	localCopyDigi = digi;
    printk(KERN_INFO "digi = 0x%p\n", digi);
	if (err) 
	{
		printk(KERN_ERR PIPER_DRIVER_NAME "failed to alloc priv\n");
		return err;
	}
	digi->drv_name = PIPER_DRIVER_NAME;
	spin_lock_init(&digi->irqMaskLock);
	
    /*
     * Reserve access to the Piper registers mapped to memory.
     *
     * iomem_resource:  some global data structure that has all
     *                  the special memory address spaces
     * pdev->resource   set to piper_mem
     */
#if 0
    printk(KERN_INFO "start address = 0x%8.8X, end address = 0x%8.8X\n", 
            pdev->resource[0].start, pdev->resource[0].end);
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
    
	if (gpio_request(PIPER_RESET_GPIO, PIPER_DRIVER_NAME) != 0) 
	{
		ERROR("PIPER_RESET_GPIO already in use");
		goto resetGpioInuse;
	}
	
    /*
     * Besides loading the Piper firmware, this function also sets the
     * MAC address and loads it into piper->hw->wiphy->perm_addr.
     */
    piper_load_firmware(digi);
    
    init_rx_tx(digi);

    version =piper_read_reg(digi, BB_VERSION);
    status = piper_read_reg(digi, BB_GENERAL_STAT);
    
    printk(KERN_INFO "version = 0x%8.8X, status = 0x%8.8X\n", version, status);


	/* provide callbacks to generic mac code */
	digi->write_reg = piper_write_reg;
	digi->read_reg = piper_read_reg;
	digi->write = piper_write;
	digi->write_fifo = piper_write_fifo;
	digi->write_aes = piper_write_aes;
	digi->write_aes_key = piper_write_aes_key;
	digi->initHw = initHw;
	digi->setIrqMaskBit = setIrqMaskBit;
	digi->clearIrqMaskBit = clearIrqMaskBit;

    digi->irq = pdev->resource[1].start;
    
	err = request_irq(digi->irq, piperIsr, 0, PIPER_DRIVER_NAME,
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

	if (gpio_request(PIPER_IRQ_GPIO, PIPER_DRIVER_NAME) != 0) 
	{
		ERROR("PIPER_IRQ_GPIO already in use");
		goto irqGpioInuse;
	}
	
    gpio_configure_ns921x(PIPER_IRQ_GPIO, NS921X_GPIO_INPUT, NS921X_GPIO_DONT_INVERT, 
		                NS921X_GPIO_FUNC_2, NS921X_GPIO_ENABLE_PULLUP);
		                
    printk(KERN_INFO "calling piper_register_hw\n");
	err = piper_register_hw(digi, &pdev->dev, &al7230_rf_ops);
	if (err) {
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to register priv\n");
		goto do_free_rx;
	}

	goto piper_probe_exit;


do_free_rx:    
    free_rx_tx(digi);
irqGpioInuse:
    gpio_free(PIPER_IRQ_GPIO);
error_irq:
    free_irq(pdev->resource[1].start, digi);    
resetGpioInuse:
    gpio_free(PIPER_RESET_GPIO);
    iounmap(digi->vbase);
    digi->vbase = NULL;
error_remap:
    release_resource(pdev->resource);
piper_probe_exit:
	return err;
}


static int piper_remove(struct platform_device *dev)
{
#if 0
    /* TODO:  Figure out why this doesn't work and get rid of localCopyDigi */
    struct piper_priv *digi = platform_get_drvdata(dev);
#else
    struct piper_priv *digi;
    
    digi = localCopyDigi;
#endif
    printk(KERN_INFO "piper_remove called\n");
    printk(KERN_INFO "digi = 0x%p\n", digi);

    /* extern void enable_irq(unsigned int irq); */
	disable_irq(dev->resource[1].start);
    clearIrqMaskBit(digi, 0xffffffff);
    free_irq(dev->resource[1].start, digi);
    free_rx_tx(digi);
    gpio_free(PIPER_IRQ_GPIO);
    gpio_free(PIPER_RESET_GPIO);
    release_resource(dev->resource);
    printk(KERN_INFO "piper_remove release_resource() has returned\n");
	piper_free_hw(digi); 
    
    printk(KERN_INFO "piper_remove returning\n");
    return 0;
}


static void piper_release_device(struct device* dev)
{
    printk(KERN_INFO "piper_release_device called\n");
	/* nothing to do. But we need a !NULL function */
}




/* define the resources the driver will use */
/* This structure must match piper_resources in ns921x_devices.c*/
static struct resource piper_resources[] = 
{
    {
    	.name  = PIPER_DRIVER_NAME,
    	.start = MAC_REG_BASE,
    	.end   = MAC_REG_BASE + MAC_REG_SIZE,
    	.flags = IORESOURCE_MEM,
    }, 
    {
		.start	= IRQ_NS9XXX_EXT0,
		.flags	= IORESOURCE_IRQ,
    }
};

/* describes the device */
static struct platform_device piper_device = {
	.id     = -1,
	.name   = PIPER_DRIVER_NAME,/* must be equal to platform-driver.driver.name*/
    .num_resources = ARRAY_SIZE(piper_resources),
	.resource = piper_resources,
	.dev = {
		.release = piper_release_device,
	},
};

/* describes the driver */
static struct platform_driver piper_driver = {
	.probe  = piper_probe,
	.remove = piper_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name  = PIPER_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init piper_init_module(void)
{
	int err;

	pr_info("piper_init_module called\n");
    printk(KERN_INFO "piper_init_module, version %s\n", DRV_VERS);

    err = platform_device_register(&piper_device);
    if (err) {
            printk(KERN_ALERT "Device Register Failed, error = %d\n", err);
            goto error;
    }

    err = platform_driver_register(&piper_driver);
    if (err) {
            printk(KERN_ALERT "Driver Register Failed, error = %d\n", err);
            goto error;
    }

    return 0;

error:
    return err;
}

static void __exit piper_exit_module(void)
{
    printk(KERN_ALERT "piper_exit_module called\n");
    platform_driver_unregister(&piper_driver);
    printk(KERN_ALERT "piper_exit_module platform_driver_unregister returned\n");
    platform_device_unregister(&piper_device);
    printk(KERN_ALERT "piper_exit_module platform_device_unregister returned\n");
	pr_info("Piper driver unloaded\n");
}



module_init(piper_init_module);
module_exit(piper_exit_module);

MODULE_DESCRIPTION("Digi Piper WLAN Driver");
MODULE_AUTHOR("Contact support@digi.com for questions on this code");
MODULE_VERSION(DRV_VERS);
MODULE_LICENSE("GPL");
