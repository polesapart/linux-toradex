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

/*
 * This API backs up the required transceiver register value and holds the
 * transceiver reset line. This API can also be called in power save mode
 * before entering deep sleep mode.
 */
static void MacEnterSleepMode(struct piper_priv *piperp)
{
	/*
	 * Interrupts are already disabled when we get here.
	 */
    while (piperp->ac->rd_reg(piperp, BB_RSSI) & BB_RSSI_EAS_BUSY)
        ;

    while ((piperp->ac->rd_reg(piperp, BB_GENERAL_CTL) & BB_GENERAL_CTL_TX_FIFO_EMPTY) == 0)
                ;//wait for tx fifo empty

    savedRegs[INDX_GEN_CONTROL] = piperp->ac->rd_reg(piperp, BB_GENERAL_CTL);
    savedRegs[INDX_GEN_STATUS]  = piperp->ac->rd_reg(piperp, BB_GENERAL_STAT);
    savedRegs[INDX_RSSI_AES]    = piperp->ac->rd_reg(piperp, BB_RSSI) & ~BB_RSSI_EAS_BUSY;
    savedRegs[INDX_INTR_MASK]   = piperp->ac->rd_reg(piperp, BB_IRQ_MASK);
    savedRegs[INDX_SPI_CTRL]    = piperp->ac->rd_reg(piperp, BB_SPI_CTRL);
    savedRegs[INDX_CONF1]       = piperp->ac->rd_reg(piperp, BB_TRACK_CONTROL);
    savedRegs[INDX_CONF2]       = piperp->ac->rd_reg(piperp, BB_CONF_2);
    savedRegs[INDX_OUT_CTRL]    = piperp->ac->rd_reg(piperp, BB_OUTPUT_CONTROL);
    savedRegs[INDX_MAC_CONTROL] = piperp->ac->rd_reg(piperp, MAC_CTL);

    piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and); //disable receiving
    piperp->ac->wr_reg(piperp, MAC_CTL, 0, op_write);
    piperp->ac->wr_reg(piperp, BB_IRQ_MASK, 0, op_write);

#if 0
    // held the transceiver in reset mode
	if (piperp->pdata->reset)
		piperp->pdata->reset(piperp, 1);
#endif
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

#if 0
	if (piperp->pdata->reset) {
		piperp->pdata->reset(piperp, 0);
		udelay(500);
	}
#endif
/*
 * TODO:  Fix these magic numbers.  They came from the NET+OS driver.
 */

    // store the registers back
    piperp->ac->wr_reg(piperp, BB_GENERAL_STAT, 0x30000000, op_write);
    piperp->ac->wr_reg(piperp, BB_RSSI, savedRegs[INDX_RSSI_AES], op_write);
    piperp->ac->wr_reg(piperp, BB_SPI_CTRL, savedRegs[INDX_SPI_CTRL], op_write);
    piperp->ac->wr_reg(piperp, BB_TRACK_CONTROL, savedRegs[INDX_CONF1], op_write);
    piperp->ac->wr_reg(piperp, BB_CONF_2, savedRegs[INDX_CONF2], op_write);
    piperp->ac->wr_reg(piperp, BB_AES_CTL, 0, op_write);
    piperp->ac->wr_reg(piperp, BB_OUTPUT_CONTROL, savedRegs[INDX_OUT_CTRL], op_write);
    piperp->ac->wr_reg(piperp, MAC_CTL, savedRegs[INDX_MAC_CONTROL], op_write);

    // set bit-11 in the general control register to a 1 to start the processors
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, BB_GENERAL_CTL_MAC_ASSIST_ENABLE, op_or);

    // set the TX-hold bit
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720080, op_write);

    // clear the TX-FIFO memory
    for (i=0; i<448; i++)
    	piperp->ac->wr_reg(piperp, BB_DATA_FIFO, 0, op_write);

    // reset the TX-FIFO
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x377200C0, op_write);

    // release the TX-hold and reset
	piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, 0x37720000, op_write);

    piperp->ac->wr_reg(piperp, BB_GENERAL_CTL, (savedRegs[INDX_GEN_CONTROL] |
    					0x37000000 | BB_GENERAL_CTL_RX_EN), op_write);

	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0xff, op_write);
	piperp->ac->wr_reg(piperp, BB_IRQ_STAT, 0, op_write);

    piperp->ac->wr_reg(piperp, BB_IRQ_MASK, savedRegs[INDX_INTR_MASK], op_write);

    stats.jiffiesOff += jiffies - stats.cycleStart;
    stats.cycleStart = jiffies;
}



/*
 * Macro converts milliseconds to HZ.
 *
 * TODO:  Look for standard Linux version of this.
 */
#define MILLS_TO_JIFFIES(x)		(((x*HZ) + 500) / 1000)

/*
 * Amount of time we have to be idle before we will go to sleep.
 * This value is completely arbitrary.
 */
#define IDLE_TIMEOUT			(10)

/*
 * Number of milliseconds to wake up before beacon.  Linux might wake us
 * up later than what we schedule, so this value has to be large enough
 * to account for errors in the kernel's scheduler.  It is very important
 * for us to be awake when the beacon arrives.
 */
#define WAKEUP_TIME_BEFORE_BEACON	(10)

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
		else if (piperp->ps.state == PS_STATE_WAITING_FOR_BEACON)
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
		} else {
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
				piperp->ps.state = PS_STATE_WAITING_FOR_BEACON;
				MacEnterSleepMode(piperp);
				piper_ps_set_next_timer_event(piperp);
			}
		}
		else if (piperp->ps.state == PS_STATE_WAITING_FOR_BEACON)
		{
			stats.expectedBeacons++;
			MacEnterActiveMode(piperp);
			piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
			piper_ps_set_next_timer_event(piperp);
		}
	}
	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}

/*
 * Called on every beacon.  Reset timer for next beacon, and process the
 * beacon's TIM information.
 */
void piper_ps_handle_beacon(struct piper_priv *piperp, struct sk_buff *skb)
{
	unsigned long flags;

	spin_lock_irqsave(&piperp->ps.lock, flags);

	stats.receivedBeacons++;
	piperp->ps.next_beacon = jiffies + piperp->ps.beacon_int;
	piperp->ps.next_wakeup = piperp->ps.next_beacon - MILLS_TO_JIFFIES(WAKEUP_TIME_BEFORE_BEACON);

	if (piperp->ps.mode == PS_MODE_LOW_POWER)
	{
		piperp->ps.state = PS_STATE_WANT_TO_SLEEP;
		piper_ps_set_next_timer_event(piperp);
	}
	spin_unlock_irqrestore(&piperp->ps.lock, flags);
}
EXPORT_SYMBOL_GPL(piper_ps_handle_beacon);


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
			piperp->ps.mode = PS_MODE_LOW_POWER;
			memset(&stats, 0, sizeof(stats));
			stats.modeStart = jiffies;
			stats.cycleStart = jiffies;
			/*
			 * Will start it the next time we receive a beacon.
			 */
		}
	} else {
		printk(KERN_ERR "** Power save off\n");
		del_timer_sync(&piperp->ps.timer);
		if (   (piperp->ps.mode == PS_MODE_LOW_POWER)
			&& (piperp->ps.state == PS_STATE_WAITING_FOR_BEACON))
		{
			/*
			 * If we were powered down, then power up.
			 */
			MacEnterActiveMode(piperp);
		} else stats.jiffiesOn += jiffies - stats.cycleStart;
		if (   (piperp->ps.beacon_int != 0)
		    && ((jiffies - stats.modeStart) != 0))
		{
			printk(KERN_ERR "jiffiesOff = %u, jiffiesOn = %u, total time = %u\n", stats.jiffiesOff, stats.jiffiesOn, (jiffies - stats.modeStart));
			printk(KERN_ERR "Powered down %ld percent of the time.\n", (stats.jiffiesOff * 100) / (jiffies - stats.modeStart));
			printk(KERN_ERR "Received %u of %u beacons while in powersave mode.\n", stats.receivedBeacons, (jiffies - stats.modeStart) / piperp->ps.beacon_int);
		}
		piperp->ps.mode = PS_MODE_FULL_POWER;
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



