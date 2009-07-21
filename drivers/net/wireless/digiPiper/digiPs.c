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
#define	INDX_GEN_CONTROL    0	// General control
#define	INDX_GEN_STATUS	    1	// General status
#define	INDX_RSSI_AES		2	// RSSI and AES status
#define	INDX_INTR_MASK	    3	// Interrupt mask
#define INDX_SPI_CTRL       4	// RF SPI control
#define INDX_CONF1          5	// Configuration 1
#define INDX_CONF2          6	// Configuration 2
#define	INDX_AES_MODE		7	// ARS mode
#define INDX_OUT_CTRL       8	// Output control
#define INDX_MAC_CONTROL    9	// MAC control
#define INDX_TOTAL          10

static u32 savedRegs[INDX_TOTAL];	// Used to save registers for sleep mode


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
	if (skb) {
		dev_kfree_skb(skb);
		piperp->ps.tx_complete_fn = NULL;
	}

	return PS_DONT_RETURN_SKB_TO_MAC80211;
}


static int ps_inject_frame(struct piper_priv *piperp, struct sk_buff *skb)
{
	if (piper_hw_tx(piperp->hw, piperp->ps.psFrame) == 0) {
		piperp->ps.tx_complete_fn = ps_free_frame;
	} else {
		printk(KERN_ERR
		       "ps_inject_frame() failed unexpectedly when sending null data frame.\n");
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

	if (piperp->rf->getBand(piperp->channel) == IEEE80211_BAND_2GHZ) {
		if ((rates & MAC_PSK_BRS_MASK) == 0) {
			result = AIROHA_LOWEST_PSK_RATE_INDEX;
		}
	}

	return result;
}

static int getAckDuration(struct piper_priv *piperp)
{
	bool is_1_Mbit = (getRateIndex(piperp) == AIROHA_LOWEST_PSK_RATE_INDEX);
	int duration = 0;

	if (is_1_Mbit) {
		duration = CCK_DURATION(ACK_SIZE, 1);
	} else {
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

	if ((piperp->ps.tx_complete_fn != NULL)
	    || (piperp->ps.psFrame != NULL)) {
		printk(KERN_DEBUG
		       "sendNullDataFrame called when there was a frame already on the queue.\n");
		goto sendNullDataFrame_Exit;
	}

	skb =
	    __dev_alloc_skb(sizeof(_80211HeaderType) +
			    piperp->hw->extra_tx_headroom, GFP_ATOMIC);
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
	tx_info->antenna_sel_tx = 1;	/* actually this is ignored for now */
	tx_info->control.rates[0].idx = 0;
	tx_info->control.rates[0].count = 5;	/* 5 retries, value is completely arbitrary */
	tx_info->control.rates[0].flags = 0;
	tx_info->control.rates[1].idx = -1;
	tx_info->control.rts_cts_rate_idx = -1;

	if ((piperp->txPacket == NULL) && (piperp->is_radio_on)) {
		/*
		 * Note that we are called with interrupts off, so there should
		 * be no race condition with the fn call below.
		 */
		if (piper_hw_tx(piperp->hw, skb) != 0) {
			printk(KERN_ERR
			       "piper_hw_tx() failed unexpectedly when sending null data frame.\n");
			ps_free_frame(piperp, skb);
		} else {
			/*
			 * Use our special tx complete function which will free
			 * the SKB.
			 */
			piperp->ps.tx_complete_fn = ps_free_frame;
		}
	} else {
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

	if ((piperp->ps.tx_complete_fn != NULL)
	    || (piperp->ps.psFrame != NULL)) {
		printk(KERN_DEBUG
		       "sendPSPollFrame called when there was a frame already on the queue.\n");
		goto sendPSPollFrame_Exit;
	}

	skb =
	    __dev_alloc_skb(sizeof(_80211PSPollType) +
			    piperp->hw->extra_tx_headroom, GFP_ATOMIC);
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
	tx_info->antenna_sel_tx = 1;	/* actually this is ignored for now */
	tx_info->control.rates[0].idx = 0;
	tx_info->control.rates[0].count = 3;	/* 3 retries, value is completely arbitrary */
	tx_info->control.rates[0].flags = 0;
	tx_info->control.rates[1].idx = -1;
	tx_info->control.rts_cts_rate_idx = -1;

	if ((piperp->txPacket == NULL) && (piperp->is_radio_on)) {
		/*
		 * Note that we are called with interrupts off, so there should
		 * be no race condition with the fn call below.
		 */
		if (piper_hw_tx(piperp->hw, skb) != 0) {
			printk(KERN_ERR
			       "piper_hw_tx() failed unexpectedly when sending PS-Poll frame.\n");
			ps_free_frame(piperp, skb);
		} else {
			/*
			 * Use our special tx complete function which will free
			 * the SKB.
			 */
			piperp->ps.tx_complete_fn = ps_free_frame;
		}
	} else {
		piperp->ps.tx_complete_fn = ps_inject_frame;
		piperp->ps.psFrame = skb;
	}

      sendPSPollFrame_Exit:
	return;
}




#define RESET_PIPER		(1)

static int MacEnterSleepMode(struct piper_priv *piperp)
{
	/*
	 * Interrupts are already disabled when we get here.
	 */
	if (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY)
		return -1;

	if ((piperp->ac->
	     rd_reg(piperp, BB_GENERAL_CTL) & BB_GENERAL_CTL_TX_FIFO_EMPTY) == 0)
		return -1;

	if ((!piperp->ps.stoppedTransmit) && (piperp->txPacket))
		return -1;

	savedRegs[INDX_GEN_CONTROL] = piperp->ac->rd_reg(piperp, BB_GENERAL_CTL);
	savedRegs[INDX_GEN_STATUS] = piperp->ac->rd_reg(piperp, BB_GENERAL_STAT);
	savedRegs[INDX_RSSI_AES] =
	    piperp->ac->rd_reg(piperp, BB_RSSI) & ~BB_RSSI_EAS_BUSY;
	savedRegs[INDX_INTR_MASK] = piperp->ac->rd_reg(piperp, BB_IRQ_MASK);
	savedRegs[INDX_SPI_CTRL] = piperp->ac->rd_reg(piperp, BB_SPI_CTRL);
	savedRegs[INDX_CONF1] = piperp->ac->rd_reg(piperp, BB_TRACK_CONTROL);
	savedRegs[INDX_CONF2] = piperp->ac->rd_reg(piperp, BB_CONF_2);
	savedRegs[INDX_OUT_CTRL] = piperp->ac->rd_reg(piperp, BB_OUTPUT_CONTROL);
	savedRegs[INDX_MAC_CONTROL] = piperp->ac->rd_reg(piperp, MAC_CTL);

	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);	//disable receiving
	piperp->ac->wr_reg(piperp, MAC_CTL, 0, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);

#if RESET_PIPER
	// held the transceiver in reset mode
	if (piperp->pdata->reset)
		piperp->pdata->reset(piperp, 1);
#endif

	stats.jiffiesOn += jiffies - stats.cycleStart;
	stats.cycleStart = jiffies;
	disable_irq(piperp->irq);

	return 0;
}



/*
 * This API releases the transceiver reset line and writes back the stored
 * transceiver register value. This API can also be called in power save mode
 * after coming out of the sleep mode.
 */
static void MacEnterActiveMode(struct piper_priv *piperp, bool want_spike_suppression)
{
	int i;

#if RESET_PIPER
	if (piperp->pdata->reset) {
		piperp->pdata->reset(piperp, 0);
		piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL,
				   savedRegs[INDX_OUT_CTRL], op_write);
		udelay(150);
		if (want_spike_suppression)
			piper_spike_suppression(piperp);
	}
#endif
/*
 * TODO:  Fix these magic numbers.  They came from the NET+OS driver.
 */

	// store the registers back

	piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x30000000, op_write);
	piperp->ac->wr_reg(piperp, BB_RSSI, savedRegs[INDX_RSSI_AES], op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_MASK, savedRegs[INDX_INTR_MASK], op_write);
	piperp->ac->wr_reg(piperp, BB_SPI_CTRL, savedRegs[INDX_SPI_CTRL], op_write);
	piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, savedRegs[INDX_CONF1], op_write);
	piperp->ac->wr_reg(piperp, BB_CONF_2, savedRegs[INDX_CONF2], op_write);
	piperp->ac->wr_reg(piperp, BB_AES_CTL, 0, op_write);
	piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, savedRegs[INDX_OUT_CTRL], op_write);
	piperp->ac->wr_reg(piperp, MAC_CTL, savedRegs[INDX_MAC_CONTROL], op_write);

	// set bit-11 in the general control register to a 1 to start the processors
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   BB_GENERAL_CTL_MAC_ASSIST_ENABLE, op_or);

	// set the TX-hold bit
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720080, op_write);

	// clear the TX-FIFO memory
	for (i = 0; i < 448; i++)
		piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);

	// reset the TX-FIFO
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x377200C0, op_write);

	// release the TX-hold and reset
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720000, op_write);


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

#if RESET_PIPER
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   ((savedRegs[INDX_GEN_CONTROL] | 0x37000000) &
			    ~BB_GENERAL_CTL_RX_EN), op_write);
	piperp->rf->set_chan(piperp->hw, piperp->channel);
#else
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL,
			   (savedRegs[INDX_GEN_CONTROL] | 0x37000000 |
			    BB_GENERAL_CTL_RX_EN), op_write);
#endif

	stats.jiffiesOff += jiffies - stats.cycleStart;
	stats.cycleStart = jiffies;
	enable_irq(piperp->irq);
}



/*
 * Macro converts milliseconds to HZ.
 *
 * TODO:  Look for standard Linux version of this.
 */
#define MILLS_TO_JIFFIES(x)		((((x)*HZ) + 500) / 1000)


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
#define MINIMUM_SLEEP_PERIOD	(30)


/*
 * This function is called when there is activity, such as a command
 * from mac80211 to transmit a packet.  If we are powered down, then
 * power up.  Reset the idle timer.
 */
int piper_ps_active(struct piper_priv *piperp)
{
	unsigned long flags;
	int result = 0;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (piperp->ps.allowTransmits) {
		result = PS_CONTINUE_TRANSMIT;
	} else {
		piperp->ps.stoppedTransmit = true;
		result = PS_STOP_TRANSMIT;
	}

	spin_unlock_irqrestore(&piperp->ps.lock, flags);

	return result;
}

EXPORT_SYMBOL_GPL(piper_ps_active);


/*
 * So what crazy thing are we doing here?  Well, Piper has a bug where it
 * can send noise spikes at 1 Mbps and 2 Mbps if it is powered up without
 * running a special spike suppression routine.  The spike suppression code
 * takes an average of 30 ms, and I have timed it taking as long as 300 ms.
 * This is not something you want to use for duty cycling.  The solution is
 * to avoid sending at those two rates.  After the transmit routine determines
 * the rate mac80211 specified, it will call us and we will decide whether
 * we like that rate.  If it is one of our bad rates, then we will bump it
 * up to a good rate.
 */
struct ieee80211_rate *piper_ps_check_rate(struct piper_priv *piperp,
					   struct ieee80211_rate *rate)
{
#define BAD_RATE_1MBPS		(10)
#define BAD_RATE_2MBPS		(20)
	if ((piperp->ps.mode == PS_MODE_LOW_POWER) && (rate != NULL)) {
		if ((rate->bitrate == BAD_RATE_1MBPS)
		    || (rate->bitrate == BAD_RATE_2MBPS)) {
			rate =
			    (struct ieee80211_rate *) piperp->rf->
			    getRate(AIROHA_55_MBPS_RATE_INDEX);
		}
	}

	return rate;
}

EXPORT_SYMBOL_GPL(piper_ps_check_rate);



/*
 * This routine is called to restart the transmit task if
 * we had to stop it while duty cycling off.
 */
static void resume_transmits(struct piper_priv *piperp)
{
	piperp->ps.allowTransmits = true;
	if (piperp->ps.stoppedTransmit) {
		piperp->ps.stoppedTransmit = false;
		tasklet_hi_schedule(&piperp->tx_tasklet);
	}
}



/*
 * Programs the next timer event.
 */
static void set_timer_event(struct piper_priv *piperp,
			    enum piper_ps_events next_event, u32 delay)
{
	del_timer_sync(&piperp->ps.timer);
	piperp->ps.this_event = next_event;
	piperp->ps.timer.expires = delay;
	add_timer(&piperp->ps.timer);
}

/*
 * Called by either the duty cycle timer or the startDutyCycleTimer routine
 * to duty cycle us off.
 */
static void dutyCycleOff(struct piper_priv *piperp)
{
	if ((jiffies + MILLS_TO_JIFFIES(MINIMUM_SLEEP_PERIOD)) < piperp->ps.next_wakeup) {
		piperp->ps.allowTransmits = false;
		if (MacEnterSleepMode(piperp) == 0) {
			set_timer_event(piperp, PS_EVENT_WAKEUP_FOR_BEACON,
					piperp->ps.next_wakeup);
			piperp->ps.state = PS_STATE_SLEEPING;
		} else {
			/*
			 * If we couldn't go to sleep this time, then try again
			 * in 1 tick.
			 */
			piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
			set_timer_event(piperp, PS_EVENT_DUTY_CYCLE_EXPIRED, 1);
		}
	} else if (piperp->ps.state == PS_STATE_WANT_TO_SLEEP) {
		piperp->ps.state = PS_STATE_WAITING_FOR_BEACON;
		resume_transmits(piperp);
	}
}

/*
 * Called when we receive a beacon to start the duty cycle timer.
 */
static void startDutyCyleTimer(struct piper_priv *piperp)
{
	if ((piperp->ps.wantToSleepThisDutyCycle)
		&& (piperp->ps.reallyDoDutyCycling)) {
		if ((piperp->ps.next_duty_cycle & 0x80000000)
		    && ((jiffies & 0x80000000) == 0)) {
			/*
			 * Don't do duty cycle if jiffies has rolled over.
			 */
		} else {
			if (jiffies == piperp->ps.next_duty_cycle) {
				dutyCycleOff(piperp);
			} else if (jiffies < piperp->ps.next_duty_cycle) {
				set_timer_event(piperp,
						PS_EVENT_DUTY_CYCLE_EXPIRED,
						piperp->ps.next_duty_cycle);
			} else {
				/*
				 * If late on duty cycle, skip it.
				 */
			}
		}
	}
}



/*
 * Called when timer expires.  Called when our duty cycle expires and
 * it is time to shut down, and when it is time to wake up to receive
 * the next beacon.
 */
static void ps_timer(unsigned long context)
{
	struct piper_priv *piperp = (struct piper_priv *) context;
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (piperp->ps.mode == PS_MODE_LOW_POWER) {
		if (piperp->ps.this_event == PS_EVENT_DUTY_CYCLE_EXPIRED) {
			dutyCycleOff(piperp);
		} else if (piperp->ps.this_event == PS_EVENT_WAKEUP_FOR_BEACON) {
			stats.expectedBeacons++;
			MacEnterActiveMode(piperp, false);
			resume_transmits(piperp);
			piperp->ps.state = PS_STATE_WAITING_FOR_BEACON;
		}
	}
	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}


static unsigned int beaconHistoryIndex = 0, beaconHistoryCount = 0;

/*
 * This routine predicts when the next beacon will occur.  We have to use
 * this convoluted routine because the process of power down and then
 * powering up Piper and the transceiver screws up the timing of when we
 * receive beacons.  This routine averages together the timing for the
 * last 16 beacons to predict when the next one will occur.
 */
static u32 next_beacon_time(struct piper_priv *piperp)
{
#define MAX_BEACON_ENTRIES		(16)
#define BEACON_SHIFT_FOR_DIVIDE	(4)
#define NEXT_BEACON_INDEX(x)	((x + 1) % MAX_BEACON_ENTRIES)
/*
 * The fudge factor is needed because the beacon interval is not really an
 * even multiple of ticks long.  This fudge factor will need to be adjusted
 * if you change the macros above since duty cycling the H/W induces delays
 * which have to be accounted for in this fudge factor.
 */
#define FUDGE_FACTOR			MILLS_TO_JIFFIES((4*piperp->ps.beacon_int)/10)
	u32 result = 0;
	static u32 beacon[MAX_BEACON_ENTRIES];

	if (piperp->areWeAssociated) {
		result = MILLS_TO_JIFFIES(piperp->ps.beacon_int - 10) + jiffies;

		if ((jiffies >
		     (piperp->ps.next_beacon +
		      (MILLS_TO_JIFFIES(piperp->ps.beacon_int) / 2)))
		    && (piperp->ps.next_beacon != 0)) {
			beacon[beaconHistoryIndex] = piperp->ps.next_beacon;
			beaconHistoryIndex = NEXT_BEACON_INDEX(beaconHistoryIndex);
		}


		if (jiffies < beacon[beaconHistoryIndex]) {
			/*
			 * Reset history if system clock rolls over.
			 */
			beaconHistoryIndex = 0;
			beaconHistoryCount = 0;
		}
		beacon[beaconHistoryIndex] = jiffies;
		beaconHistoryIndex = NEXT_BEACON_INDEX(beaconHistoryIndex);

		if (beaconHistoryCount == MAX_BEACON_ENTRIES) {
			u64 sum = 0;
			u32 average, offset;
			unsigned int i;

			for (i = 0; i < MAX_BEACON_ENTRIES; i++) {
				sum += beacon[i];
			}
			/*
			 * This strange looking code divides the sum of the beacon times by 16.
			 * We have to do it this way because we needed to use a u64 to sum up
			 * the beacon times, which are u32s.  The Linux kernel does not have
			 * a u64 divide operation.
			 */
			average =
			    ((sum +
			      (MAX_BEACON_ENTRIES /
			       2)) >> BEACON_SHIFT_FOR_DIVIDE) & 0xffffffff;
			offset =
			    MILLS_TO_JIFFIES((((MAX_BEACON_ENTRIES / 2) *
					       piperp->ps.beacon_int) +
					      (piperp->ps.beacon_int)));
			result = average + offset - FUDGE_FACTOR;
			if (piperp->ps.beacon_int >= 200) {
				/* Yes, another fudge factor */
				result +=
				    MILLS_TO_JIFFIES((piperp->ps.beacon_int + 100) / 30);
			}
		} else {
			beaconHistoryCount++;
		}
	} else {
		result = MILLS_TO_JIFFIES(10) + jiffies;
	}

	return result;
}


/*
 * Called on every beacon.  Reset timer for next beacon, and process the
 * beacon's TIM information.
 */
#define FCS_SIZE						(4)
#define TIM_ELEMENT						(5)
#define MIN_TIM_SIZE					(5)
#define TAGGED_BEACON_FIELDS_START		(12)
#define BEACON_INT_LSB					(8)
#define BEACON_INT_MSB					(9)
#define MAX_DUTY_CYCLE					(90)
static void piper_ps_handle_beacon(struct piper_priv *piperp, struct sk_buff *skb)
{
	unsigned long flags;
	unsigned char *bp =
	    skb->data + sizeof(_80211HeaderType) + TAGGED_BEACON_FIELDS_START;
	unsigned char *end = &skb->data[skb->len - FCS_SIZE - MIN_TIM_SIZE];
	int time_to_next_beacon, duty_cycle_off, time_remaining;
	u32 beacon_int;

	/*
	 * mac80211 does not inform us when the beacon interval changes, so we have
	 * to read this information from the beacon ourselves.  Reset the beacon
	 * history if the beacon interval has changed.
	 */
	beacon_int = skb->data[sizeof(_80211HeaderType) + BEACON_INT_LSB];
	beacon_int |= (skb->data[sizeof(_80211HeaderType) + BEACON_INT_MSB] << 8);
	if (beacon_int != piperp->ps.beacon_int) {
		piperp->ps.beacon_int = beacon_int;
		beaconHistoryIndex = 0;
		beaconHistoryCount = 0;
	}

	if (piperp->ps.reallyDoDutyCycling != (piperp->power_duty <= MAX_DUTY_CYCLE)) {
		piperp->ps.reallyDoDutyCycling = (piperp->power_duty <= MAX_DUTY_CYCLE);
		printk(KERN_ERR "piperp->ps.reallyDoDutyCycling = %d.\n", piperp->ps.reallyDoDutyCycling);
		sendNullDataFrame(piperp, piperp->ps.reallyDoDutyCycling);
	}
	spin_lock_irqsave(&piperp->ps.lock, flags);
	stats.receivedBeacons++;
	piperp->ps.next_beacon = next_beacon_time(piperp);
	piperp->ps.next_wakeup =
	    piperp->ps.next_beacon - MILLS_TO_JIFFIES(WAKEUP_TIME_BEFORE_BEACON);

	time_to_next_beacon = piperp->ps.next_beacon - jiffies;
	duty_cycle_off = ((time_to_next_beacon * (100 - piperp->power_duty)) + 50) / 100;
	time_remaining =
	    time_to_next_beacon - (duty_cycle_off +
				   MILLS_TO_JIFFIES(WAKEUP_TIME_BEFORE_BEACON));

	if (piperp->ps.reallyDoDutyCycling) {
		piperp->ps.wantToSleepThisDutyCycle = false;
		if ((duty_cycle_off >= MILLS_TO_JIFFIES(MINIMUM_SLEEP_PERIOD))
		    && ((jiffies & 0x80000000) == ((piperp->ps.next_beacon + 1) & 0x80000000))) {
			if ((time_remaining >= 0)
			    && (piperp->power_duty != 100)) {
				piperp->ps.wantToSleepThisDutyCycle = true;
				piperp->ps.next_duty_cycle = jiffies + time_remaining;
			} else
			    if ((MILLS_TO_JIFFIES(WAKEUP_TIME_BEFORE_BEACON) +
				 MILLS_TO_JIFFIES(MINIMUM_SLEEP_PERIOD))
				< time_to_next_beacon) {
				piperp->ps.wantToSleepThisDutyCycle = true;
				piperp->ps.next_duty_cycle = jiffies;
			}
			time_remaining = piperp->ps.next_duty_cycle - jiffies;
		} else {
			time_remaining = time_to_next_beacon;
		}
	} else {
		piperp->ps.wantToSleepThisDutyCycle = false;
		time_remaining = time_to_next_beacon;
	}

	if (piperp->ps.mode == PS_MODE_LOW_POWER) {
		for (; bp < end; bp = bp + bp[1] + 2) {
			if (*bp == TIM_ELEMENT) {
				u8 length = bp[1];
				bool is_aid_0_set = bp[4] & 1;
				u8 offset = bp[4] & 0xfe;

				piperp->ps.expectingMulticastFrames = is_aid_0_set;

				if (length > 3) {	/* if partial virtual bitmap is not empty */
					int firstBit = offset * 16;
					int lastBit = (((length - 3) * 8) + firstBit) - 1;

					if ((piperp->ps.aid >= firstBit)
					    && (piperp->ps.aid <= lastBit)) {
						int idx = (piperp->ps.aid - firstBit) / 8;
						u8 mask =
						    1 <<
						    ((piperp->ps.aid - firstBit) % 8);

						piperp->ps.
						    apHasBufferedFrame =
						    (bp[5 + idx] & mask) != 0;
					}
					break;
				}
			}
		}
		if ((!piperp->ps.wantToSleepThisDutyCycle)
		    || (time_remaining > 0)) {
			if (piperp->ps.expectingMulticastFrames) {
				piperp->ps.state = PS_STATE_WAITING_FOR_BUFFERED_DATA;
				/*
				 * AP will be sending us multicast frames.  The receive processor
				 * will take care of scheduling further stuff after we have received
				 * them.
				 */
			} else {
				/*
				 * Not expecting any multicast frames to follow.
				 */
				if (piperp->ps.apHasBufferedFrame) {
					/*
					 * Ask for our buffered data if there is any.
					 */
					piperp->ps.state =
					    PS_STATE_WAITING_FOR_BUFFERED_DATA;
					sendPSPollFrame(piperp);
				}
			}
		}
		startDutyCyleTimer(piperp);
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

	if (header->fc.type == TYPE_BEACON) {
		piper_ps_handle_beacon(piperp, skb);
	} else {
		unsigned long flags;

		/*
		 * The lock is needed because we may call sendPSPollFrame which
		 * expected to be called locked.
		 */
		spin_lock_irqsave(&piperp->ps.lock, flags);

		if (piperp->ps.expectingMulticastFrames) {
			if (((header->addr1[0] & 1) == 0)
			    || (header->fc.moreData == 0)) {
				piperp->ps.expectingMulticastFrames = false;
				if (piperp->ps.apHasBufferedFrame) {
					sendPSPollFrame(piperp);
				}
			}
		} else if (piperp->ps.apHasBufferedFrame) {
			if (header->fc.moreData) {
				sendPSPollFrame(piperp);
			} else {
				piperp->ps.apHasBufferedFrame = false;
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
#define MINIMUM_PS_BEACON_INT		(100)
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	if (powerSaveOn) {
		if (piperp->ps.beacon_int >= MINIMUM_PS_BEACON_INT) {
			if (piperp->ps.mode != PS_MODE_LOW_POWER) {
				piperp->ps.allowTransmits = true;
				piperp->ps.stoppedTransmit = false;
				piperp->ps.wantToSleepThisDutyCycle = false;
				beaconHistoryIndex = 0;
				beaconHistoryCount = 0;
				piperp->ps.next_beacon = 0;
				piperp->ps.apHasBufferedFrame = false;
				piperp->ps.expectingMulticastFrames = false;
				piperp->ps.mode = PS_MODE_LOW_POWER;
				piperp->ps.state = PS_STATE_WAITING_FOR_BEACON;
				memset(&stats, 0, sizeof(stats));
				stats.modeStart = jiffies;
				stats.cycleStart = jiffies;
				piperp->ps.reallyDoDutyCycling = (piperp->power_duty <= MAX_DUTY_CYCLE);
				sendNullDataFrame(piperp, piperp->ps.reallyDoDutyCycling);
				/*
				 * Will start it the next time we receive a beacon.
				 */
			}
		} else {
			printk(KERN_ERR
			       "\nUnable to set power save mode because the beacon \ninterval set on this access point less than 100ms.\n");
		}
	} else {
		del_timer_sync(&piperp->ps.timer);
		if ((piperp->ps.mode == PS_MODE_LOW_POWER)
		    && (piperp->ps.state == PS_STATE_SLEEPING)) {
			/*
			 * If we were powered down, then power up and do the spike suppression.
			 */
			MacEnterActiveMode(piperp, true);
		} else if (piperp->ps.mode == PS_MODE_LOW_POWER) {
			/*
			 * If we were in power save mode, but powered up, then we must power
			 * down and then back up so that we can do spike suppression.
			 */
			piperp->ps.mode = PS_MODE_FULL_POWER;	/* stop duty cycle timer */
			while (MacEnterSleepMode(piperp) != 0) {
				spin_unlock_irqrestore(&piperp->ps.lock, flags);
				mdelay(10);
				spin_lock_irqsave(&piperp->ps.lock, flags);
			}
			MacEnterActiveMode(piperp, true);
			stats.jiffiesOn += jiffies - stats.cycleStart;
		}
		if ((piperp->ps.beacon_int != 0)
		    && ((jiffies - stats.modeStart) != 0)) {
#define WANT_STATS		(0)
#if WANT_STATS
			printk(KERN_ERR
			       "jiffiesOff = %u, jiffiesOn = %u, total time = %lu\n",
			       stats.jiffiesOff, stats.jiffiesOn,
			       (jiffies - stats.modeStart));
			printk(KERN_ERR
			       "Powered down %ld percent of the time.\n",
			       (stats.jiffiesOff * 100) / (jiffies - stats.modeStart));
			printk(KERN_ERR
			       "Received %u of %lu beacons while in powersave mode.\n",
			       stats.receivedBeacons,
			       (jiffies -
				stats.modeStart) /
			       MILLS_TO_JIFFIES(piperp->ps.beacon_int));
#endif
		}
		piperp->ps.mode = PS_MODE_FULL_POWER;
		piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
		resume_transmits(piperp);
		piperp->ps.reallyDoDutyCycling = false;
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
	piperp->ps.allowTransmits = true;
	piperp->ps.stoppedTransmit = false;
	init_timer(&piperp->ps.timer);
	piperp->ps.timer.function = ps_timer;
	piperp->ps.timer.data = (unsigned long) piperp;
	piperp->ps.wantToSleepThisDutyCycle = false;
	piperp->ps.reallyDoDutyCycling = false;
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
