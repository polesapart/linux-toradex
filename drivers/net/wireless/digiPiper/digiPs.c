/*
 * digiPs.c
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
#include "airoha.h"
#include "digiPs.h"


// Powersave register index
#define	INDX_GEN_CONTROL    0		// General control
#define	INDX_GEN_STATUS	    1		// General status
#define	INDX_RSSI_AES		2		// RSSI and AES status
#define	INDX_INTR_MASK	    3		// Interrupt mask
#define INDX_SPI_CTRL       4		// RF SPI control
#define INDX_CONF1          5		// Configuration 1
#define INDX_CONF2          6		// Configuration 2
#define	INDX_AES_MODE		7		// ARS mode
#define INDX_OUT_CTRL       8       // Output control
#define INDX_MAC_CONTROL    9       // MAC control
#define INDX_TOTAL          10

static u32 savedRegs[INDX_TOTAL]; // Used to save registers for sleep mode


/*
 * TODO: Delete this.
 */
struct ps_stats {
	unsigned int modeStart;
	unsigned int cycleStart;
	unsigned int receivedBeacons;
	unsigned int expectedBeacons;
	unsigned int jiffiesOn;
	unsigned int jiffiesOff;
} stats;

int piper_hw_tx(struct ieee80211_hw *hw, struct sk_buff *skb);


static int ps_free_frame(struct piper_priv *piperp, struct sk_buff *skb)
{
	if (skb)
	{
		dev_kfree_skb(skb);
		piperp->ps.tx_complete_fn = NULL;
	}

	return PS_DONT_RETURN_SKB_TO_MAC80211;
}


static int ps_inject_frame(struct piper_priv *piperp, struct sk_buff *skb)
{
	if (piper_hw_tx(piperp->hw, piperp->ps.psFrame) == 0)
	{
		piperp->ps.tx_complete_fn = ps_free_frame;
	}
	else
	{
		printk(KERN_ERR "ps_inject_frame() failed unexpectedly when sending null data frame.\n");
		ps_free_frame(piperp, piperp->ps.psFrame);
	}
	piperp->ps.psFrame = NULL;

	return PS_RETURN_SKB_TO_MAC80211;
}



#define PS_ON			(true)
#define PS_OFF			(false)


#define ACK_SIZE		(14)	/* length of ACK in bytes */

// Length (in usecs) of a MAC frame of bytes at rate (in 500kbps units)
// not including SIFS and PLCP preamble/header
#define	CCK_DURATION(bytes, rate)		((16*(bytes)+(rate)-1)/(rate))

#define USE_SHORTPREAMBLE(is_1_Mbit, use_short_preamble)	((!is_1_Mbit) & use_short_preamble)

// Length (in usecs) of SIFS and PLCP preamble/header.
#define	PRE_LEN(is_1_Mbit, use_short_preamble)			(USE_SHORTPREAMBLE(is_1_Mbit, use_short_preamble) ? 106 : 202)

// Duration (in usecs) of an OFDM frame at rate (in 500kbps units)
// including SIFS and PLCP preamble/header
#define	OFDM_DURATION(bytes, rate)	(36 + 4*((4*(bytes)+(rate)+10)/(rate)))

static unsigned int getRateIndex(struct piper_priv *piperp)
{
    unsigned int rates = piperp->ac->rd_reg(piperp, MAC_SSID_LEN);
    unsigned int result = AIROHA_LOWEST_OFDM_RATE_INDEX;

    if (piperp->rf->getBand(piperp->channel) == IEEE80211_BAND_2GHZ)
    {
        if ((rates & MAC_PSK_BRS_MASK) == 0)
        {
        	result = AIROHA_LOWEST_PSK_RATE_INDEX;
        }
    }

    return result;
}

static int getAckDuration (struct piper_priv *piperp)
{
	bool is_1_Mbit = (getRateIndex(piperp) == AIROHA_LOWEST_PSK_RATE_INDEX);
	int duration = 0;

	if (is_1_Mbit)
	{
		duration = CCK_DURATION(ACK_SIZE, 1);
	}
	else
	{
		duration = OFDM_DURATION(ACK_SIZE, 6);
	}

	duration += PRE_LEN(is_1_Mbit, piperp->use_short_preamble);

	return duration;
}


/*
 * This function is used to notify the AP about the current state of
 * power save.  One of the bits in the 802.11 header field is set to
 * indicate the status of power save.  This bit is actually set appropriately
 * for all frames sent, we just send a null data frame just to make
 * sure something is sent to the AP in a reasonable amount of time.
 */
static void sendNullDataFrame(struct piper_priv *piperp, bool isPowerSaveOn)
{
	struct sk_buff *skb = NULL;
	_80211HeaderType *header;
	struct ieee80211_tx_info *tx_info;

	if ((piperp->ps.tx_complete_fn != NULL) || (piperp->ps.psFrame != NULL))
	{
		printk(KERN_DEBUG "sendNullDataFrame called when there was a frame already on the queue.\n");
		goto sendNullDataFrame_Exit;
	}

	skb = __dev_alloc_skb(sizeof(_80211HeaderType) + piperp->hw->extra_tx_headroom,
							GFP_ATOMIC);
	if (skb == NULL)
		goto sendNullDataFrame_Exit;

	tx_info = (struct ieee80211_tx_info *) skb->cb;

	skb_reserve(skb, piperp->hw->extra_tx_headroom);
	header = (_80211HeaderType *) skb_put(skb, sizeof(_80211HeaderType));
	memset(header, 0, sizeof(*header));
	header->fc.type = TYPE_NULL_DATA;
	header->fc.pwrMgt = isPowerSaveOn;
	header->duration = getAckDuration(piperp);
	memcpy(header->addr1, piperp->bssid, sizeof(header->addr1));
	memcpy(header->addr2, piperp->pdata->macaddr, sizeof(header->addr2));
	memcpy(header->addr3, piperp->bssid, sizeof(header->addr3));

	tx_info->flags = IEEE80211_TX_CTL_ASSIGN_SEQ | IEEE80211_TX_CTL_FIRST_FRAGMENT;
	tx_info->band = piperp->rf->getBand(piperp->channel);
	tx_info->antenna_sel_tx = 1;	/* actually this is ignored for now*/
	tx_info->control.rates[0].idx = 0;
	tx_info->control.rates[0].count = 5;		/* 5 retries, value is completely arbitrary*/
	tx_info->control.rates[0].flags = 0;
	tx_info->control.rates[1].idx = -1;
	tx_info->control.rts_cts_rate_idx = -1;

	if ((piperp->txPacket == NULL) && (piperp->is_radio_on))
	{
		/*
		 * Note that we are called with interrupts off, so there should
		 * be no race condition with the fn call below.
		 */
		if (piper_hw_tx(piperp->hw, skb) != 0)
		{
			printk(KERN_ERR "piper_hw_tx() failed unexpectedly when sending null data frame.\n");
			ps_free_frame(piperp, skb);
		}
		else
		{
			/*
			 * Use our special tx complete function which will free
			 * the SKB.
			 */
			piperp->ps.tx_complete_fn = ps_free_frame;
		}
	}
	else
	{
		piperp->ps.tx_complete_fn = ps_inject_frame;
		piperp->ps.psFrame = skb;
	}

sendNullDataFrame_Exit:
	return;
}

/*
 * This routine sends a PS-Poll frame, which is used to request a buffered
 * frame from the AP.
 *
 * TODO: Share code with sendNullDataFrame.
 */
static void sendPSPollFrame(struct piper_priv *piperp)
{
	struct sk_buff *skb = NULL;
	_80211PSPollType *header;
	struct ieee80211_tx_info *tx_info;

	if ((piperp->ps.tx_complete_fn != NULL) || (piperp->ps.psFrame != NULL))
	{
		printk(KERN_DEBUG "sendPSPollFrame called when there was a frame already on the queue.\n");
		goto sendPSPollFrame_Exit;
	}

	skb = __dev_alloc_skb(sizeof(_80211PSPollType) + piperp->hw->extra_tx_headroom,
							GFP_ATOMIC);
	if (skb == NULL)
		goto sendPSPollFrame_Exit;

	tx_info = (struct ieee80211_tx_info *) skb->cb;

	skb_reserve(skb, piperp->hw->extra_tx_headroom);
	header = (_80211PSPollType *) skb_put(skb, sizeof(_80211PSPollType));
	memset(header, 0, sizeof(*header));
	header->fc.type = TYPE_PSPOLL;
	header->fc.pwrMgt = (piperp->ps.mode == PS_MODE_LOW_POWER);
	header->aid = piperp->ps.aid;
	memcpy(header->addr1, piperp->bssid, sizeof(header->addr1));
	memcpy(header->addr2, piperp->pdata->macaddr, sizeof(header->addr2));

	tx_info->flags = IEEE80211_TX_CTL_FIRST_FRAGMENT;
	tx_info->band = piperp->rf->getBand(piperp->channel);
	tx_info->antenna_sel_tx = 1;	/* actually this is ignored for now*/
	tx_info->control.rates[0].idx = 0;
	tx_info->control.rates[0].count = 3;		/* 3 retries, value is completely arbitrary*/
	tx_info->control.rates[0].flags = 0;
	tx_info->control.rates[1].idx = -1;
	tx_info->control.rts_cts_rate_idx = -1;

	if ((piperp->txPacket == NULL) && (piperp->is_radio_on))
	{
		/*
		 * Note that we are called with interrupts off, so there should
		 * be no race condition with the fn call below.
		 */
		if (piper_hw_tx(piperp->hw, skb) != 0)
		{
			printk(KERN_ERR "piper_hw_tx() failed unexpectedly when sending PS-Poll frame.\n");
			ps_free_frame(piperp, skb);
		}
		else
		{
			/*
			 * Use our special tx complete function which will free
			 * the SKB.
			 */
			piperp->ps.tx_complete_fn = ps_free_frame;
		}
	}
	else
	{
		piperp->ps.tx_complete_fn = ps_inject_frame;
		piperp->ps.psFrame = skb;
	}

sendPSPollFrame_Exit:
	return;
}


static u32 saved_mac_regs[][2] = {
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



/*
 * This API backs up the required transceiver register value and holds the
 * transceiver reset line. This API can also be called in power save mode
 * before entering deep sleep mode.
 */
static void MacEnterSleepMode(struct piper_priv *piperp)
{
	int i;

	/* wait until aes done  */
	i = 10000;
	while (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY && i-- > 0)
		udelay(1);

	/* wait for tx fifo empty */
	i = 10000;
	while (((piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) &
		BB_GENERAL_CTL_TX_FIFO_EMPTY) == 0) && i-- > 0)
		udelay(1);

	for (i = 0; i < (sizeof(saved_mac_regs)/(2*sizeof(u32))); i++)
		saved_mac_regs[i][1] = piperp->ac->rd_reg(piperp, saved_mac_regs[i][0]);

	/* disable receiver */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_RXFIFORST | BB_GENERAL_CTL_TXFIFORST, op_or);
	piperp->ac->wr_reg(piperp, MAC_CTL, 0, op_write);
	piperp->clear_irq_mask_bit(piperp, 0xffffffff);

	disable_irq(piperp->irq);

	/* held the transceiver in reset mode, if reset cb was defined */
//	if (piperp->pdata->reset)
//		piperp->pdata->reset(piperp, 1);

    stats.jiffiesOn += jiffies - stats.cycleStart;
    stats.cycleStart = jiffies;
}


/*
 * This API releases the transceiver reset line and writes back the stored
 * transceiver register value. This API can also be called in power save mode
 * after coming out of the sleep mode.
 */
static void MacEnterActiveMode(struct piper_priv *piperp)
{
	int i;

// 	if (piperp->pdata->reset)
// 		piperp->pdata->reset(piperp, 0);
//	mdelay(2);

	for (i = 0; i < (sizeof(saved_mac_regs)/(2*sizeof(u32))); i++)
		piperp->ac->wr_reg(piperp, saved_mac_regs[i][0],
				  saved_mac_regs[i][1], op_write);

	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x30000000, op_write);
	piperp->ac->wr_reg(piperp, BB_AES_CTL, 0x0, op_write);

	/* Restart the processors */
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~(BB_GENERAL_CTL_RXFIFORST | BB_GENERAL_CTL_TXFIFORST), op_and);
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

	/*
	 * Reset the interrupt mask.  We could have been receiving a frame when we
	 * powered down.  This could cause us to store the wrong mask, so we want
	 * to make sure we enabe the RX interrupts.  However, we should not have the
	 * TX interrupts enabled when we come out of power save mode.
	 */
	piperp->clear_irq_mask_bit(piperp, 0xffffffff);
	piperp->set_irq_mask_bit(piperp, BB_IRQ_MASK_RX_OVERRUN | BB_IRQ_MASK_RX_FIFO);

    stats.jiffiesOff += jiffies - stats.cycleStart;
    stats.cycleStart = jiffies;
//	mdelay(5);
	enable_irq(piperp->irq);
}



/*
 * Macro converts milliseconds to HZ.
 *
 * TODO:  Look for standard Linux version of this.
 */
#define MILLS_TO_JIFFIES(x)		(((x*HZ) + 500) / 1000)

/*
 * Amount of time we have to be idle before we will go to sleep.
 * This value has to be large enough to allow us to send a full size
 * frame at 1 Mbps and receive an ACK.
 */
#define IDLE_TIMEOUT			(20)

/*
 * Number of milliseconds to wake up before beacon.  Linux might wake us
 * up later than what we schedule, so this value has to be large enough
 * to account for errors in the kernel's scheduler.  It is very important
 * for us to be awake when the beacon arrives.
 */
#define WAKEUP_TIME_BEFORE_BEACON	(15)

/*
 * Minimum amount of time we will sleep.  If we will end up sleeping
 * for less than this, then don't go to sleep.
 */
#define MINIMUM_SLEEP_PERIOD	(20)

/*
 * Program timer for next event which will either be an idle timeout
 * or beacon wake up.
 */
static void piper_ps_set_next_timer_event(struct piper_priv *piperp)
{
	/*
	 * piperp->ps.lock has already been grabbed by the time we get here.
	 */

	del_timer_sync(&piperp->ps.timer);

	if (piperp->ps.mode == PS_MODE_LOW_POWER)
	{
		if (piperp->ps.state == PS_STATE_WANT_TO_SLEEP)
		{
			u32 idleStart = jiffies + MILLS_TO_JIFFIES(IDLE_TIMEOUT);

			if ((idleStart + MILLS_TO_JIFFIES(MINIMUM_SLEEP_PERIOD)) < piperp->ps.next_wakeup)
			{
				/* TODO:  What about roll over */
				piperp->ps.timer.expires = idleStart;
				add_timer(&piperp->ps.timer);
			}
		}
		else if (piperp->ps.state == PS_STATE_SLEEPING)
		{
			piperp->ps.timer.expires = piperp->ps.next_wakeup;
			add_timer(&piperp->ps.timer);
		}
	}
}

/*
 * This function is called when there is activity, such as a command
 * from mac80211 to transmit a packet.  If we are powered down, then
 * power up.  Reset the idle timer.
 */
void piper_ps_active(struct piper_priv *piperp)
{
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (piperp->ps.mode == PS_MODE_LOW_POWER) {
		if (piperp->ps.state == PS_STATE_WANT_TO_SLEEP) {
			/*
			 * This will reset the idle timer.
			 */
			piper_ps_set_next_timer_event(piperp);
		} else if (piperp->ps.state == PS_STATE_SLEEPING) {
			/*
			 * If we are powered down, then power up and
			 * reset the idle timer.
			 */
			MacEnterActiveMode(piperp);
			piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
			piper_ps_set_next_timer_event(piperp);
		}
	}
	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}
EXPORT_SYMBOL_GPL(piper_ps_active);


/*
 * Called when timer expires.  Shut down the transceiver if we
 * are in power save mode and the idle timer expired.  Otherwise,
 * power up the transceive if it's time to receive a beacon.
 */
static void ps_timer(unsigned long context)
{
	struct piper_priv *piperp = (struct piper_priv *) context;
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (piperp->ps.mode == PS_MODE_LOW_POWER)
	{
		if (piperp->ps.state == PS_STATE_WANT_TO_SLEEP)
		{
			if ((jiffies + MILLS_TO_JIFFIES(MINIMUM_SLEEP_PERIOD)) < piperp->ps.next_wakeup)
			{
				/* TODO:  What about roll over */
				piperp->ps.state = PS_STATE_SLEEPING;
				MacEnterSleepMode(piperp);
				piper_ps_set_next_timer_event(piperp);
			}
		}
		else if (piperp->ps.state == PS_STATE_SLEEPING)
		{
			stats.expectedBeacons++;
			MacEnterActiveMode(piperp);
			piperp->ps.state = PS_STATE_WAITING_FOR_BEACON;
		}
	}
	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}

/*
 * Called on every beacon.  Reset timer for next beacon, and process the
 * beacon's TIM information.
 */
#define FCS_SIZE		(4)
#define TIM_ELEMENT		(5)
#define MIN_TIM_SIZE	(5)
static void piper_ps_handle_beacon(struct piper_priv *piperp, struct sk_buff *skb)
{
	unsigned long flags;
	unsigned char *bp = skb->data + sizeof(_80211HeaderType) + 12;
	unsigned char *end = &skb->data[skb->len - FCS_SIZE - MIN_TIM_SIZE];

	spin_lock_irqsave(&piperp->ps.lock, flags);

	stats.receivedBeacons++;
	piperp->ps.next_beacon = jiffies + piperp->ps.beacon_int;
	piperp->ps.next_wakeup = piperp->ps.next_beacon - MILLS_TO_JIFFIES(WAKEUP_TIME_BEFORE_BEACON);

	if (piperp->ps.mode == PS_MODE_LOW_POWER)
	{
		for ( ; bp < end; bp = bp+bp[1]+2) {
			if (*bp == TIM_ELEMENT) {
				u8 length = bp[1];
				bool is_aid_0_set = bp[4] & 1;
				u8 offset = bp[4] & 0xfe;

				piperp->ps.expectingMulticastFrames = is_aid_0_set;

				if (length > 3)	{ /* if partial virtual bitmap is not empty */
					int firstBit = offset * 16;
					int lastBit = (((length - 3) * 8) + firstBit) - 1;

					if ((piperp->ps.aid >= firstBit) && (piperp->ps.aid <= lastBit)) {
						int idx = (piperp->ps.aid - firstBit) / 8;
						u8 mask = 1 << ((piperp->ps.aid - firstBit) % 8);

						piperp->ps.apHasBufferedFrame = (bp[5 + idx] & mask) != 0;
					}
					break;
				}
				if (piperp->ps.expectingMulticastFrames || piperp->ps.apHasBufferedFrame)
					printk (KERN_ERR "multicast %d, to us %d\n", piperp->ps.expectingMulticastFrames,
							piperp->ps.apHasBufferedFrame);
			}
		}
		if (piperp->ps.expectingMulticastFrames)
		{
			piperp->ps.state = PS_STATE_WAITING_FOR_BUFFERED_DATA;
			/*
			 * AP will be sending us multicast frames.  The receive processor
			 * will take care of scheduling further stuff after we have received
			 * them.
			 */
		}
		else
		{
			/*
			 * Not expecting any multicast frames to follow.
			 */
			if (piperp->ps.apHasBufferedFrame)
			{
				/*
				 * Ask for our buffered data if there is any.
				 */
				piperp->ps.state = PS_STATE_WAITING_FOR_BUFFERED_DATA;
				sendPSPollFrame(piperp);
			}
			else
			{
				/*
				 * Go back to sleep.  We let the idle timer time us out
				 * because we could be in the middle of a transmit.
				 */
				piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
				piper_ps_set_next_timer_event(piperp);
			}
		}

	}

	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}


/*
 * This routine is called so we can process incoming frames.  We do the
 * handshaking to receive buffered frames in PS mode here.
 */
void piper_ps_process_receive_frame(struct piper_priv *piperp, struct sk_buff *skb)
{
	_80211HeaderType *header = (_80211HeaderType *) skb->data;

	if (header->fc.type == TYPE_BEACON)
	{
		piper_ps_handle_beacon(piperp, skb);
	}
	else
	{
		unsigned long flags;

		/*
		 * The lock is needed because we may call sendPSPollFrame which
		 * expected to be called locked.
		 */
		spin_lock_irqsave(&piperp->ps.lock, flags);

		if (piperp->ps.expectingMulticastFrames)
		{
			if (((header->addr1[0] & 1) == 0) || (header->fc.moreData == 0))
			{
				piperp->ps.expectingMulticastFrames = false;
				if (piperp->ps.apHasBufferedFrame)
				{
					sendPSPollFrame(piperp);
				}
				else
				{
					piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
					piper_ps_set_next_timer_event(piperp);
				}
			}
		}
		else if (piperp->ps.apHasBufferedFrame)
		{
			if (header->fc.moreData)
			{
				sendPSPollFrame(piperp);
			}
			else
			{
				piperp->ps.apHasBufferedFrame = false;
				piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
				piper_ps_set_next_timer_event(piperp);
			}
		}
		spin_unlock_irqrestore(&piperp->ps.lock, flags);
	}
}
EXPORT_SYMBOL_GPL(piper_ps_process_receive_frame);


/*
 * This function turns power save mode on or off.
 */
void piper_ps_set(struct piper_priv *piperp, bool powerSaveOn)
{
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (powerSaveOn) {
		printk(KERN_ERR "** Power save on\n");
		if (piperp->ps.mode != PS_MODE_LOW_POWER) {
printk(KERN_ERR "AID = %d, gpio = %d.\n", piperp->ps.aid, piperp->pdata->rst_gpio);
			piperp->ps.apHasBufferedFrame = false;
			piperp->ps.expectingMulticastFrames = false;
			piperp->ps.mode = PS_MODE_LOW_POWER;
			piperp->ps.state = PS_STATE_WAITING_FOR_BEACON;
			memset(&stats, 0, sizeof(stats));
			stats.modeStart = jiffies;
			stats.cycleStart = jiffies;
			sendNullDataFrame(piperp, PS_ON);
			/*
			 * Will start it the next time we receive a beacon.
			 */
		}
	} else {
		printk(KERN_ERR "** Power save off\n");
		del_timer_sync(&piperp->ps.timer);
		if (   (piperp->ps.mode == PS_MODE_LOW_POWER)
			&& (piperp->ps.state == PS_STATE_SLEEPING))
		{
			/*
			 * If we were powered down, then power up.
			 */
			MacEnterActiveMode(piperp);
		} else stats.jiffiesOn += jiffies - stats.cycleStart;
		if (   (piperp->ps.beacon_int != 0)
		    && ((jiffies - stats.modeStart) != 0))
		{
			printk(KERN_ERR "jiffiesOff = %u, jiffiesOn = %u, total time = %lu\n", stats.jiffiesOff, stats.jiffiesOn, (jiffies - stats.modeStart));
			printk(KERN_ERR "Powered down %ld percent of the time.\n", (stats.jiffiesOff * 100) / (jiffies - stats.modeStart));
			printk(KERN_ERR "Received %u of %lu beacons while in powersave mode.\n", stats.receivedBeacons, (jiffies - stats.modeStart) / piperp->ps.beacon_int);
		}
		piperp->ps.mode = PS_MODE_FULL_POWER;
		piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
		sendNullDataFrame(piperp, PS_OFF);
	}

	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}
EXPORT_SYMBOL_GPL(piper_ps_set);



/*
 * Called when driver is loaded.  Initialize our context.
 */
void piper_ps_init(struct piper_priv *piperp)
{
	memset(&piperp->ps, 0, sizeof(piperp->ps));
	spin_lock_init(&piperp->ps.lock);
	piperp->ps.mode = PS_MODE_FULL_POWER;

	init_timer(&piperp->ps.timer);
	piperp->ps.timer.function = ps_timer;
	piperp->ps.timer.data = (unsigned long) piperp;
}
EXPORT_SYMBOL_GPL(piper_ps_init);


/*
 * Called when driver is unloaded.  Make sure the PS
 * timer is shut down.
 */
void piper_ps_deinit(struct piper_priv *piperp)
{
	piperp->ps.mode = PS_MODE_FULL_POWER;
	del_timer_sync(&piperp->ps.timer);
}
EXPORT_SYMBOL_GPL(piper_ps_deinit);



