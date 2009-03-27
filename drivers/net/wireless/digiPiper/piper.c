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

#include "pipermain.h"
#include "mac.h"
#include "piperhwinit.h"
#include "phy.h"
#include "airoha.h"


#define	DRIVER_VERSION	"0.1"

#define MAC_REG_BASE	(0x70000000)		/* Register base address*/
#define MAC_REG_SIZE    (0x100)             /* range of MAC registers*/
#define MAC_CTRL_BASE   (volatile unsigned int *) (MAC_REG_BASE + (0x40))

#define WANT_TO_RECEIVE_FRAMES_IN_ISR   (1)


#define WANT_DEBUG_THREAD   (0)


#define WANT_TOGGLE         (0)

static struct piper_priv *localCopyDigi = NULL;

#define DUMP_WORDS_MAX      (700)
static unsigned int dumpWordsWord[DUMP_WORDS_MAX];
static unsigned int dumpWordsCount = 0;
void dumpWordsAdd(unsigned int word)
{
    if (dumpWordsCount < DUMP_WORDS_MAX)
    {
        dumpWordsWord[dumpWordsCount++] = word;
    }
}
void dumpWordsDump(void)
{
    unsigned int *p = dumpWordsWord;
    unsigned int wordsToGo = dumpWordsCount;
    
    dumpWordsCount = 0;
    
    while (wordsToGo >= 4)
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
    digi_dbg("--------------\n");
}

void dumpWordsReset(void)
{
    dumpWordsCount = 0;
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

static void clearIrqMaskBit(struct piper_priv *piper, unsigned int bits) 
{
    piper->write_reg(piper, BB_IRQ_MASK, ~bits, op_and);
}

static void setIrqMaskBit(struct piper_priv *piper, unsigned int bits) 
{
    piper->write_reg(piper, BB_IRQ_MASK, bits, op_or);
}



/*
 * This routine writes data into a FIFO.  For best performance, there are
 * 3 different implementations of this routine below.  The if statements
 * select the implementation depending upon if the data is aligned and if
 * we are writing multiple words of data.
 */
static int piper_write(struct piper_priv *digi, uint8_t addr, uint8_t *buf,
		int len)
{
#define WAIT_FOR_AES        while (    (addr == BB_AES_FIFO)        \
                                    && ((ioread32(digi->vbase + BB_RSSI) & BB_RSSI_EAS_FIFO_FULL) != 0)) {udelay(1);}
                                    
    int wordIndex;
    int wordLength = len / sizeof(unsigned int);
    unsigned long flags;
    bool loadingBeacon = (addr == BEACON_FIFO);
    
    spin_lock_irqsave(&digi->rxRegisterAccessLock, flags);

    if (loadingBeacon)
    {
        /*
         * If we are loading a new beacon, then adjust the address to point
         * to the data FIFO, and set the beacon enable bit which tells piper
         * to put this data into the beacon buffer.
         */
        addr = BB_DATA_FIFO;
        iowrite32(ioread32(digi->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_BEACON_EN,
                  digi->vbase + BB_GENERAL_CTL);
    }
    
    if (((unsigned) (buf) & 0x3) == 0)
    {
        /*
         * We come here if the data is 32-bit aligned.  We can dispense 
         * with memcpys
         */
        if (wordLength == 1)
        {
            /*
             * Only 1 word of data, so just one write.
             */
            unsigned int *word = (unsigned int *) buf;
            
            WAIT_FOR_AES;
            iowrite32(cpu_to_be32(*word), digi->vbase + addr);
/* TODO: Remove this */ dumpWordsAdd(cpu_to_be32(*word));
            len -= 4;
        }
        else
        {
            /*
             * More than one word of data, so set up a for loop.
             */
            for (wordIndex = 0; wordIndex < wordLength; wordIndex++)
            {
                unsigned int *word = (unsigned int *) buf;
                
                WAIT_FOR_AES;
                iowrite32(cpu_to_be32(word[wordIndex]), digi->vbase + addr);
/* TODO: Remove this */ dumpWordsAdd(cpu_to_be32(word[wordIndex]));
                len -= 4;
            }
        }
    }
    else
    {
        /*
         * Ugh!  Data is not 32-bit aligned.  We have to memcpy it!
         */
        for (wordIndex = 0; wordIndex < wordLength; wordIndex++)
        {
            unsigned int word;
            
            memcpy(&word, &buf[wordIndex*sizeof(unsigned int)], sizeof(unsigned int));
            
            WAIT_FOR_AES;
            iowrite32(cpu_to_be32(word), digi->vbase + addr);
/* TODO: Remove this */ dumpWordsAdd(cpu_to_be32(word));
            len -= 4;
        }
    }
  
    if (len)
    {
        /*
         * Double Ugh!  There was left over data at the end.  We have to write
         * the leftover data into the upper bytes of the last word, making 
         * sure the unused bytes are set to zero.
         */
        unsigned int word;
        
        memcpy(&word, &buf[wordLength*sizeof(unsigned int)], sizeof(unsigned int));
        word = cpu_to_be32(word);
        switch (len)
        {
            case 1:
                word &= 0xff000000;
                break;
            case 2:
                word &= 0xffff0000;
                break;
            case 3:
                word &= 0xffffff00;
                break;
            default:
                digi_dbg("len = %d at end of piper_write\n", len);
                break;
        }
        WAIT_FOR_AES;
        iowrite32(word, digi->vbase + addr);
/* TODO: Remove this */ dumpWordsAdd(word);
    }

    if (loadingBeacon)
    {
        /*
         * If we just loaded a beacon, then don't forget to turn off the 
         * load beacon bit.
         */
        iowrite32(ioread32(digi->vbase + BB_GENERAL_CTL) & ~BB_GENERAL_CTL_BEACON_EN,
                  digi->vbase + BB_GENERAL_CTL);
        digi->beacon.loaded = true;
    }

    spin_unlock_irqrestore(&digi->rxRegisterAccessLock, flags);
    
    return 0;
}


/*
 * Write a value to a Piper register.  This function takes an operations
 * parameter that allows you to specify a simple write, a logical-and operation, or
 * a logical-or operation.  There are two reasons for this:
 *
 * 1.  We want to maintain some compatibility with the USB driven 
 *     WiWave which actually uses these operations.
 * 2.  If we do the logical operations seperately, for example read a register,
 *     or the value with the result, and then write, we run the risk of
 *     some other thread interrupting us between the read and the write.
 *     By doing both in this function we can protect the operation with
 *     a spinlock.
 */
static int piper_write_reg(struct piper_priv *digi, uint8_t reg, uint32_t val, 
                            piperRegisterWriteOperation_t op)
{    
    unsigned long flags;
    
    spin_lock_irqsave(&digi->rxRegisterAccessLock, flags);

    switch (op)
    {
        case op_write:
            iowrite32(val, digi->vbase + reg);
            break;
        case op_or:
            iowrite32(val | ioread32(digi->vbase + reg), digi->vbase + reg);
            break;
        case op_and:
            iowrite32(val & ioread32(digi->vbase + reg), digi->vbase + reg);
            break;
        default:
            digi_dbg("Invalid operation %d passed to piper_write_reg\n", op);
            WARN_ON(1);
            break;
    }

    spin_unlock_irqrestore(&digi->rxRegisterAccessLock, flags);

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
	err = digi->write_reg(digi, BB_AES_CTL, BB_AES_CTL_AES_MODE | keyidx, op_write);
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


static int __piper_write_fifo(struct piper_priv *digi, unsigned char *buffer,
		unsigned int length, int fifo)
{
    uint8_t addr;
    
    if (fifo == BB_AES_FIFO)
    {
        addr = BB_AES_FIFO;
    }
    else
    {
        addr = BB_DATA_FIFO;
    }

    return piper_write(digi, addr, buffer, length);
}


static int load_beacon(struct piper_priv *digi, unsigned char *buffer,
                        unsigned int length)
{
    return piper_write(digi, BEACON_FIFO, buffer, length);
}



static int piper_write_fifo(struct piper_priv *digi, unsigned char *buffer,
            unsigned int length, unsigned int flags)
{
	int err;

	err = __piper_write_fifo(digi, buffer, length, BB_DATA_FIFO);
	if (err)
		goto done;

done:
	return err;
}

/*
 * This routine writes a frame using H/W AES encryption.  
 */
static int piper_write_aes(struct piper_priv *digi, unsigned char *buffer,
		unsigned int length, unsigned int flags)
{
    int result;
    int timeout = 10000;
    
    /*
     * Step 1: Wait for AES to become ready.
     */
    while (digi->read_reg(digi, BB_RSSI) & BB_RSSI_EAS_BUSY)
    {
        timeout--;
        if (timeout == 0)
        {
            dumpRegisters(digi, MAIN_REGS | MAC_REGS);
        }
        udelay(1);
    }
    
    /*
     * Step 2: Write the unencrypted part of the frame into the normal
     *         data FIFO.
     */
    __piper_write_fifo(digi, buffer, _80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE, 
                        BB_DATA_FIFO);

    /*
     * Step 3: Write to the AES control register.  Writing to it puts 
     *         AES H/W engine into transmit mode.  We also make sure 
     *         the AES mode is set correctly.
     */
    digi->write_reg(digi, BB_AES_CTL, 0, op_write);
    
    /*
     * Step 4: Write the expanded AES key into the AES FIFO.
     */
    __piper_write_fifo(digi, 
                        (unsigned char *) digi->key[digi->txAesKey].expandedKey, 
                        EXPANDED_KEY_LENGTH, BB_AES_FIFO);
    
    /*
     * Step 5: Write the AES IV and headers into the AES FIFO.
     */
    __piper_write_fifo(digi, (unsigned char *) digi->txAesBlob,
                         AES_BLOB_LENGTH, BB_AES_FIFO);
    
    /*
     * Step 6: Now, finally, write the part of the frame that needs to
     *         be encrypted into the AES FIFO.
     */
    result = __piper_write_fifo(digi, &buffer[_80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE], 
                        length - (_80211_HEADER_LENGTH + TX_HEADER_LENGTH + PIPER_EXTIV_SIZE),
                        BB_AES_FIFO);
    
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
        txInfo->tx_rate_idx = txInfo->status.retries[0].rate_idx - (txInfo->status.retry_count - 1);
        if (txInfo->tx_rate_idx < 0)
        {
            txInfo->tx_rate_idx = 0;
        }
#if 0
        /*
         * At the time this driver was written, the mac80211 library was passing
         * the retry array uninitialized.  So this piece of code will default to
         * retrying at the initially set rate for the first half of the retry
         * period, and then retrying at a low speed for the 2nd half.
         */
        if (digi->txRetryIndex < ((digi->txMaxRetries + 1) >> 1))
        {
            txInfo->tx_rate_idx = txInfo->status.retries[0].rate_idx;
        }
        else
        {
            txInfo->tx_rate_idx = 0;
        }
#endif
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
    
    result = ieee80211_get_tx_rate(digi->hw, txInfo);
    
    return result;
}

static int myrand(void) {
/* RAND_MAX assumed to be 32767 */
    static unsigned long next = 1;
    next = next * 1103515245 + 12345;
    return((unsigned)(next/65536) % 32768);
}

/*
 * Compute a beacon backoff time as described in section 11.1.2.2 of 802.11 spec.
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



/*
 * This function returns a value for the contention window in microseconds.  We
 * start with the contention window at CW_MIN and double it everytime we have to
 * retry.  
 */
static u16 getCw(bool isFirstTime)
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
    return (cw + (10*(myrand() & (cw - 1)))) & 0xffff;
}



/*
 * This function will prepend an RTS or CTS to self frame ahead of the current
 * TX frame.  This is done if the wantRts or wantCts flag is set.
 */
void handleRtsCts(struct piper_priv *digi, struct ieee80211_tx_info *txInfo)
{
    unsigned int header[2];
    
    if (digi->wantRts)
    {
        digi_dbg("Want RTS\n");
        /*
         * If we come here, then we need to send an RTS frame ahead of the
         * current data frame.
         */
        if (digi->rtsCtsRate)
        {
            phy_set_plcp((unsigned char *) header, sizeof(struct ieee80211_rts), 
                            digi->rtsCtsRate, 0);
    		digi->write_fifo(digi, (unsigned char *) header, TX_HEADER_LENGTH,
    		                        txInfo->flags);
    		digi->write_fifo(digi, (unsigned char *) &digi->rtsFrame, 
    		                        sizeof(digi->rtsFrame), txInfo->flags);
        }
        else 
        {
            digi_dbg("No rate for RTS frame.\n");
        }
    }
    else if (digi->wantCts)
    {
        digi_dbg("Want CTS\n");
        /*
         * If we come here, then we need to send a CTS to self frame ahead of the 
         * current data frame.
         */
        if (digi->rtsCtsRate)
        {
            phy_set_plcp((unsigned char *) header, sizeof(struct ieee80211_cts), 
                            digi->rtsCtsRate, 0);
    		digi->write_fifo(digi, (unsigned char *) header, TX_HEADER_LENGTH,
    		                        txInfo->flags);
    		digi->write_fifo(digi, (unsigned char *) &digi->ctsFrame, 
    		                        sizeof(digi->ctsFrame), txInfo->flags);
        }
        else 
        {
            digi_dbg("No rate for CTS frame.\n");
        }
    }
}        
    

void piperTxRetryTaskletEntry (unsigned long context)
{
    struct piper_priv *digi = (struct piper_priv *) context;
    int err;

    /*
     * Clear flags here to cover ACK case.  We do not clear the flags in the ACK 
     * routine since it is possible to receive an ACK after we have started the
     * next packet.  The appropriate interrupts will be reenabled if we decide
     * to retransmit.
     */
	clearIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	if (digi->txPacket != NULL)
	{
        struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(digi->txPacket);

        if (txInfo->status.retry_count != digi->txMaxRetries)
        {
            if (txInfo->status.retry_count != 0)
            {
                #define FRAME_CONTROL_FIELD_OFFSET      (sizeof(struct tx_frame_hdr) + sizeof(struct psk_cck_hdr))
                frameControlFieldType_t *fc = (frameControlFieldType_t *) &digi->txPacket->data[FRAME_CONTROL_FIELD_OFFSET];
                fc->retry = 1;              /* set retry bit */
            }
    
            digi->write_reg(digi, MAC_BACKOFF, getCw(txInfo->status.retry_count == 0), op_write);
            txInfo->status.retry_count++; 
	        /*
	         * Build the H/W transmit header.
	         */
	        phy_set_plcp(digi->txPacket->data, 
	                     digi->txPacket->len - TX_HEADER_LENGTH, 
	                     getTxRate(digi, txInfo), digi->useAesHwEncryption ? 8 : 0);
            /*
             * One less retry to go at this rate.
             */
	        txInfo->status.retries[digi->txRetryIndex].limit--;
	        
        	/* 
        	 * Pause the transmitter so that we don't start transmitting before we
        	 * are ready.
        	 */
        	digi->write_reg(digi, BB_GENERAL_CTL, 
        	                BB_GENERAL_CTL_TX_HOLD, op_or);
            /* Clear any pending TX interrupts */
            digi->write_reg(digi, BB_IRQ_STAT, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT, op_write);
	        handleRtsCts(digi, txInfo);
        	if (digi->useAesHwEncryption)
        	{
        		err = digi->write_aes(digi, digi->txPacket->data, digi->txPacket->len, 
        		                txInfo->flags);
            }
        	else
        	{
        		err = digi->write_fifo(digi, digi->txPacket->data, digi->txPacket->len, 
        		                    txInfo->flags);
        	}
       /* TODO: What to do if err != 0 */
        	/* write_reg will start the transmit*/
        	err = digi->write_reg(digi, BB_GENERAL_CTL, 
        	                ~BB_GENERAL_CTL_TX_HOLD, op_and);
	        /*
	         * Set interrupt flags.  Use the timeout interrupt if we expect
	         * an ACK.  Use the FIFO empty interrupt if we do not expect an ACK.
	         */
            if (txInfo->flags & IEEE80211_TX_CTL_NO_ACK)
            {
                setIrqMaskBit(digi, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TX_ABORT);
            }
            else
            {
                setIrqMaskBit(digi, BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
            }
        }
        else
        {
            txInfo->status.excessive_retries = true;
            digi_dbg("All %d retries used up, ieee80211_tx_status_irqsafe\n", digi->txMaxRetries);
            ieee80211_tx_status_irqsafe(digi->hw,
            				           digi->txPacket);
            digi->txPacket = NULL;
            ieee80211_wake_queues(digi->hw);
        }
    }
    /* TODO: Remove this debug code */ else digi_dbg("digi->txPacket == NULL\n");
}

static int piper_write_aes_key(struct piper_priv *digi, struct sk_buff *skb)
{
	int err;

	/* Hardware workaround; sometimes it starts pulling from the
	 * TX FIFO before we've finished sending it the entire packet.
	 * To deal with that we set the TX_HOLD bit, which will force
	 * the hardware to wait for us to finish writing to the FIFO */
	/* TODO:  Does this make sense?  Is this a bug caused by copying code
	          from piper_write? 
       Answer from Mike:
       If you are using the old mode (mode 0) where the keys are written 
       with each frame, then you do not want to clear the TX Hold bit 
       until the key and all the plaintext data has been written to the 
       AES FIFO (unless you can guarantee the writes are not interrupted). 

        With the new mode 1 there is no need to set the TX Hold bit when 
        updating the keys.
	*/
	err = digi->write_reg(digi, BB_GENERAL_CTL, 
	                BB_GENERAL_CTL_TX_HOLD, op_or);
	if (err)
		goto done;

	err = __piper_write_fifo(digi, skb->data, skb->len, BB_AES_FIFO);
	if (err)
		goto done;

	/* write_reg will kick the tx_thread to start processing packets if
	 * it's not already active */
	err = digi->write_reg(digi, BB_GENERAL_CTL, 
	                ~BB_GENERAL_CTL_TX_HOLD, op_and);
	
done:
	return err;
}




static int piper_read(struct piper_priv *digi, uint8_t addr, uint8_t *buf,
		int len)
{

    int wordIndex;
    unsigned long flags;
    
    spin_lock_irqsave(&digi->rxRegisterAccessLock, flags);
    
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
    spin_unlock_irqrestore(&digi->rxRegisterAccessLock, flags);
    
    return 0;
}



static unsigned int piper_read_reg(struct piper_priv *digi, uint8_t reg)
{
    unsigned long flags;
    unsigned int result;
    
    spin_lock_irqsave(&digi->rxRegisterAccessLock, flags);

    result = ioread32(digi->vbase + reg);

    spin_unlock_irqrestore(&digi->rxRegisterAccessLock, flags);

	return result;
}





/**
 * Interrupt handler for Piper IRQs.
 */
/* TODO: Remove this */ static void rxTaskletEntry (unsigned long context);    
static irqreturn_t piperIsr(int irq, void *dev_id)
{
	struct piper_priv *piper = dev_id;
	unsigned int status;
	/* acknowledge interrupt */
	status = piper->read_reg(piper, BB_IRQ_STAT);
    status &= piper->read_reg(piper, BB_IRQ_MASK);
	piper->write_reg(piper, BB_IRQ_STAT, status, op_write);
    
	if (status & BB_IRQ_MASK_RX_FIFO) 
	{
	    /* 
	     * Wake up the tasklet and mask off further interrupts until
	     * the tasklet reenables them.
	     */
	    clearIrqMaskBit(piper, BB_IRQ_MASK_RX_FIFO); 
#if WANT_TO_RECEIVE_FRAMES_IN_ISR
        rxTaskletEntry ((unsigned long) piper);
#else
        tasklet_hi_schedule(&piper->rxTasklet);
#endif
	} 

    if (status & BB_IRQ_MASK_TX_FIFO_EMPTY) 
    {
        /*
         * Transmit complete interrupt.  This IRQ is only unmasked if we are
         * not expecting the packet to be ACKed.
         */
	    if (piper->txPacket != NULL)
	    {
            ieee80211_tx_status_irqsafe(piper->hw,
            				           piper->txPacket);
            piper->txPacket = NULL;
            ieee80211_wake_queues(piper->hw);
        }
		clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
    }

	if (status & BB_IRQ_MASK_TIMEOUT)
	{
	    /*
	     * AP did not ACK our TX packet.
	     */
	    if (piper->txPacket != NULL)
	    {
	        tasklet_hi_schedule(&piper->txRetryTasklet);
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
	        tasklet_hi_schedule(&piper->txRetryTasklet);
        }
		clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
	}

    if (status & BB_IRQ_MASK_TBTT)
    {
        /*
         * This interrupt occurs at the start of a beacon period.  The only thing
         * we need to do is to write a new beacon backoff value.
         */
        u32 reg = piper->read_reg(piper, MAC_BEACON_FILT) & ~MAC_BEACON_BACKOFF_MASK;
        piper->write_reg(piper, MAC_BEACON_FILT, reg | getNextBeaconBackoff(), op_write);
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
        piper->beacon.weSentLastOne = true;         /* assume we sent last beacon unless we receive one*/
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
		clearIrqMaskBit(piper, BB_IRQ_MASK_ATIM);
	}

	if (unlikely(status & BB_IRQ_MASK_RX_OVERRUN)) 
	{
		piper->rxOverruns++;
	}

	return IRQ_HANDLED;
}


/*
 * This routine is called to receive a frame.  The AES engine is used
 * to decrypt the frame if it was encrypted with AES and we have the
 * key.
 *
 * returns true if we received the frame, or false if it failed the MIC
 */
static bool receivePacket(struct piper_priv *piper, struct sk_buff *skb, int length, 
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
    
    if (headerLength > length)
    {
        headerLength = length;
    }
    
    header = (_80211HeaderType *) skb_put(skb, headerLength);
    length -= headerLength;
    piper_read(piper, BB_DATA_FIFO, (uint8_t *) header, headerLength);
    memcpy(frameControlField, &header->fc, sizeof(frameControlField));

#if 0
    if (header->fc.protected)
    {
        unsigned char *rsnHeader = skb_put(skb, _80211_HEADER_LENGTH);
        unsigned int aesDataBlob[AES_BLOB_LENGTH / sizeof(unsigned int)];
        unsigned int keyIndex;

        /*
         * Step 1: Read the rest of the unencrypted data.
         */
        piper_read(piper, BB_DATA_FIFO, rsnHeader, _80211_HEADER_LENGTH);
        length -= _80211_HEADER_LENGTH;
        
        keyIndex = rsnHeader[3] >> 6;
        
        if (piperPrepareAESDataBlob(piper, keyIndex, aesDataBlob, 
                                    (unsigned char *) header, length, false))
        {
            
            /*
             * Step 2: Wait for AES to become ready.
             */
            while (piper->read_reg(piper, BB_RSSI) & BB_RSSI_EAS_BUSY)
            {
            }
    
            /*
             * Step 3: Set the AES mode, and then read from the AES control
             *         register to put the AES engine into receive mode.
             */
            piper->write_reg(piper, BB_AES_CTL, ~BB_AES_CTL_AES_MODE, op_and);

            piper->read_reg(piper, BB_AES_CTL);
            
            /*
             * Step 4: Write the expanded AES key into the AES FIFO.
             */
            __piper_write_fifo(piper, 
                                (unsigned char *) piper->key[keyIndex].expandedKey, 
                                EXPANDED_KEY_LENGTH, BB_AES_FIFO);
            
            /*
             * Step 5: Write the AES IV and headers into the AES FIFO.
             */
            __piper_write_fifo(piper, (unsigned char *) aesDataBlob,
                                 AES_BLOB_LENGTH, BB_AES_FIFO);
            
            /*
             * Step 6: Now, finally, read the unencrypted frame from the
             *         AES FIFO.
             */
            length -= MIC_SIZE + DATA_SIZE;
            result = __piper_read_fifo(piper, skb_put(skb, length), 
                                length,
                                BB_AES_FIFO);
            /*
             * Step 7: Wait for AES to become ready.
             */
            while (piperi->read_reg(piper, BB_RSSI) & BB_RSSI_EAS_BUSY)
            {
            }
    
            result = ((piper->read_reg(piper, BB_RSSI) && BB_RSSI_EAS_MIC) != 0);
            
            skb_put(skb, 8);            /* pad an extra 8 bytes for the MIC which the H/W strips*/
            if (result)
            {
                status->flag |= RX_FLAG_DECRYPTED;
                /* TODO:  should I also set RX_FLAG_MMIC_STRIPPED */
            }
        }
        else 
        {
            /*
             * If we branch here, then we are not able to decrypt the
             * packet possibly because we don't have the key.  Read
             * the rest of the packet and let mac80211 decrypt it.
             */
            __piper_read_fifo(piper, skb_put(skb, length), 
                                length,
                                BB_DATA_FIFO);
         }
     }
    else
#endif
    {
        /*
         * Frame is not encrypted, so just read it.
         */
        piper_read (piper, BB_DATA_FIFO, skb_put(skb, length), length);
    }

#if WANT_RECEIVE_COUNT_SCROLL
    if (((++packetCount) & 31) == 0)
    {
        digi_dbg("%d packets received so far.\n", packetCount);
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
static void handleAck(struct piper_priv *piper, int signalStrength)
{
    if (piper->txPacket) 
    {
        struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(piper->txPacket);
        if ((txInfo->flags & IEEE80211_TX_CTL_NO_ACK) == 0)
        {
    	    clearIrqMaskBit(piper, BB_IRQ_MASK_TX_FIFO_EMPTY | BB_IRQ_MASK_TIMEOUT | BB_IRQ_MASK_TX_ABORT);
         	txInfo->flags = IEEE80211_TX_STAT_ACK;
        	txInfo->status.ack_signal = signalStrength;
        	
            ieee80211_tx_status_irqsafe(piper->hw, piper->txPacket);
            piper->txPacket = NULL;
            /* TODO:  Is it possible to get an ACK when we are not expecting one and so
                      screw up our system?  Should txPacket and friends be protected with
                      a spinlock?
             */
            ieee80211_wake_queues(piper->hw);
        }
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
    		if (receivePacket(piper, skb, length, &frameControlField, &status))
    	    {
        		if (frameControlField.type == TYPE_ACK)
        		{
        		    handleAck(piper, status.signal);
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
        		        piper->beacon.weSentLastOne = false;
        		    }
#if WANT_TO_RECEIVE_FRAMES_IN_ISR
                    ieee80211_rx_irqsafe(piper->hw, skb, &status);  
#else
        		    ieee80211_rx(piper->hw, skb, &status);
#endif
        		}
        	}
        	else
        	{
        	    /*
        	     * Frame failed MIC, so discard it.
        	     */
    		    dev_kfree_skb(skb);
    		}
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

    while (1)
    {
        ssleep(60);
        dumpRegisters(piper, ALL_REGS);
    }
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
    tasklet_init(&piper->txRetryTasklet, piperTxRetryTaskletEntry, (unsigned long) piper);
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
    if (band == WLN_BAND_A)
    {
        digi->rf->init(digi->hw, IEEE80211_BAND_5GHZ);
    }
    else
    {
        digi->rf->init(digi->hw, IEEE80211_BAND_2GHZ);
    }
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

#ifdef AIROHA_PWR_CALIBRATION
    initPwrCal();
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
	spin_lock_init(&digi->rxRegisterAccessLock);
	
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

    initializeKeys(digi);
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
	digi->load_beacon = load_beacon;
    digi->getNextBeaconBackoff = getNextBeaconBackoff;
    
    digi->irq = pdev->resource[1].start;
    digi->bssWantCtsProtection = false;
    digi->beacon.loaded = false;
    digi->beacon.enabled = false;
    digi->beacon.weSentLastOne = false;
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
#if 0
    err = platform_device_register(&piper_device);
    if (err) {
            printk(KERN_ALERT "Device Register Failed, error = %d\n", err);
            goto error;
    }
#endif
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
