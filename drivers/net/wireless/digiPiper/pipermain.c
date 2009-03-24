/*
 * Linux device driver for Digi's Piper based 802.11 MAC
 *
 * Copyright © 2008  Digi International, Inc
 *
 * Author: Andres Salomon <dilinger@debian.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <net/mac80211.h>
#include <crypto/aes.h>

#define DEBUG

#include "pipermain.h"
#include "mac.h"
#include "phy.h"



enum antenna_select
{
	ANTENNA_BOTH = 0,
	ANTENNA_1,
	ANTENNA_2,
};


static int enableIbssSupport(struct piper_priv *digi, enum nl80211_iftype nodeType)
{
    int result = 0;
    
    if (   ((nodeType == NL80211_IFTYPE_ADHOC) || (nodeType == NL80211_IFTYPE_MESH_POINT))
        && (digi->beacon.loaded)
        && (digi->beacon.enabled)
        && ((digi->read_reg(digi, MAC_CFP_ATIM) & MAC_BEACON_INTERVAL_MASK) != 0))
    {
        /*
         * If we come here, then we are running in IBSS mode, beacons are enabled,
         * and we have the information we need, so start sending beacons.
         */
        /* TODO: Handle non-zero ATIM period.  mac80211 currently has no way to
                 tell us what the ATIM period is, but someday they might fix that.*/
                                                    /* set a beacon backoff value */
         u32 reg = digi->read_reg(digi, MAC_BEACON_FILT) & ~MAC_BEACON_BACKOFF_MASK;
         digi->write_reg(digi, MAC_BEACON_FILT, reg | digi->getNextBeaconBackoff(), op_write);
         digi->setIrqMaskBit(digi, BB_IRQ_MASK_TBTT); /* enable beacon interrupts*/
                                                    /* and let her rip */
         result = digi->write_reg(digi, MAC_CTL, MAC_CTL_BEACON_TX | MAC_CTL_IBSS, op_or);
         digi_dbg("IBSS supported turned on!\n");
    }
    else
    {
        /*
         * If we come here, then either we are not suppose to transmit beacons,
         * or we do not yet have all the information we need to transmit 
         * beacons.  Make sure the automatic beacon function is disabled.
         */
                                                    /* shut down beacons */
         result = digi->write_reg(digi, MAC_CTL, ~(MAC_CTL_BEACON_TX | MAC_CTL_IBSS), op_and);
         digi->setIrqMaskBit(digi, BB_IRQ_MASK_TBTT);     /* disable beacon interrupts*/
         digi_dbg("IBSS supported turned OFF\n");
    }
    
    return result;
}


static int set_antenna_div(struct ieee80211_hw *hw, enum antenna_select sel)
{
	int err = 0;
    const char *antennaText[] = {
        "ANTENNA_BOTH",
    	"ANTENNA_1",
    	"ANTENNA_2"
    };
	struct piper_priv *digi = hw->priv;

	digi_dbg("set_antenna_div called, sel = %s\n", antennaText[sel]);
	
	/* set antenna diversity */
	if (sel == ANTENNA_BOTH)
	{
		err = digi->write_reg(digi, BB_GENERAL_CTL,
				BB_GENERAL_CTL_ANT_DIV, op_or);
		err = digi->write_reg(digi, BB_GENERAL_CTL,
				~BB_GENERAL_CTL_ANT_SEL, op_and);
	}
	else {
		err = digi->write_reg(digi, BB_GENERAL_CTL,
				~BB_GENERAL_CTL_ANT_DIV, op_and);
		if (err)
			goto done;

		/* selected the antenna if !diversity */
		if (sel == ANTENNA_1)
			err = digi->write_reg(digi, BB_GENERAL_CTL,
					~BB_GENERAL_CTL_ANT_SEL, op_and);
		else
			err = digi->write_reg(digi, BB_GENERAL_CTL,
					BB_GENERAL_CTL_ANT_SEL, op_or);
	}
	if (err)
		goto done;
	
	/* select which antenna to transmit on */
	err = digi->write_reg(digi, BB_RSSI, ~BB_RSSI_ANT_MASK, op_and);
	if (err)
		goto done;
	if (sel == ANTENNA_BOTH)
		err = digi->write_reg(digi, BB_RSSI, BB_RSSI_ANT_DIV_MAP, op_or);
    else
		err = digi->write_reg(digi, BB_RSSI, BB_RSSI_ANT_NO_DIV_MAP, op_or);
done:

	if (err)
		printk(KERN_ERR PIPER_DRIVER_NAME ": %s failed - %d\n", __func__, err);
	return err;
}

static int set_status_led(struct ieee80211_hw *hw, int on)
{
#if 0
	struct piper_priv *digi = hw->priv;
	int err;

	if (on)
		err = digi->write_reg(digi, BB_RSSI, BB_RSSI_LED, op_or);
	else
		err = digi->write_reg(digi, BB_RSSI, ~BB_RSSI_LED, op_and);
	
	return err;
#else
    return 0;
#endif
}

static int set_tx_power(struct ieee80211_hw *hw, int power)
{
	struct piper_priv *digi = hw->priv;
	int err;

	if (power == digi->tx_power)
		return 0;

	err = digi->rf->set_pwr(hw, power);
	if (!err)
		digi->tx_power = power;
    
	return err;
}


static void assignSequenceNumber(struct sk_buff *skb, bool shouldIncrement)
{
    #define SEQUENCE_NUMBER_MASK        (0xfff0)
    
	static u16 sequenceNumber = 0;
    _80211HeaderType *header = (_80211HeaderType *) skb->data;
    
    if (skb->len >= sizeof(*header))
    {
        u16 sequenceField;
        
        /*
         * TODO:  memcpy's are here because I am concerned we may get
         *        an unaligned frame.  Is this a real possibility?  Or
         *        am I just wasting CPU time?
         */
        memcpy(&sequenceField, &header->squ.sq16, sizeof(header->squ.sq16));
        sequenceField &= ~SEQUENCE_NUMBER_MASK;
        sequenceField |= (SEQUENCE_NUMBER_MASK & (sequenceNumber << 4));
        memcpy(&header->squ.sq16, &sequenceField, sizeof(header->squ.sq16));
        if (shouldIncrement)
        {
            sequenceNumber++;
        }
    }
}



static void setupRtsCtsFrame(struct piper_priv *digi, struct sk_buff *skb,
                             struct ieee80211_tx_info *txInfo)
{
    digi->wantCts = (!!(txInfo->flags & IEEE80211_TX_CTL_USE_CTS_PROTECT)
                     | digi->bssWantCtsProtection);
    digi->wantRts = !!(txInfo->flags & IEEE80211_TX_CTL_USE_RTS_CTS);
    
#if 0
    {
        _80211HeaderType *header = (_80211HeaderType *) skb->data;
        
        if ((header->fc.type == TYPE_DATA) && (header->addr1[0] == 0))
        {
            digi_dbg("Forcing CTS to self.\n");
            digi->wantCts = true;
        }
        if (header->fc.type == TYPE_DATA) 
        {
            digi_dbg("Forcing RTS.\n");
            digi->wantRts = true;
        }
    }
	{ /* TODO:  Remove this debug code. */
	    const char *yesOrNo[2] = {"no", "yes"};
	    digi_dbg("flags = 0x%8.8X, RTS/CTS = %s, CTS protect = %s\n", 
	    txInfo->flags,
	    yesOrNo[!!(txInfo->flags & IEEE80211_TX_CTL_USE_RTS_CTS)],
	    yesOrNo[!!(txInfo->flags & IEEE80211_TX_CTL_USE_CTS_PROTECT)]);
	}
#endif

    if ((digi->wantRts) || (digi->wantCts))
    {
        /*
         * Need to get rate info now because the control part of the union
         * will be overwritten by status information later.
         */
        digi->rtsCtsRate = ieee80211_get_rts_cts_rate(digi->hw, txInfo);
        if (digi->rtsCtsRate == NULL)
        {
            digi_dbg("ieee80211_get_rts_cts_rate(digi->hw, txInfo) returned NULL!\n");
        }

        if (digi->wantRts)
        {
            ieee80211_rts_get(digi->hw, txInfo->control.vif, skb->data, skb->len,
                              txInfo, &digi->rtsFrame);
        } 
        else if (digi->wantCts)
        {
            ieee80211_ctstoself_get(digi->hw, txInfo->control.vif, skb->data, skb->len,
                                    txInfo, &digi->ctsFrame);
            /*
             * At the time this code was written, the mac80211 library had 
             * a bug in the ieee80211_ctstoself_get which caused it to copy 
             * the wrong MAC address into the cts frame.  So we copy the
             * right one (ours) in now.
             */
            memcpy(digi->ctsFrame.ra, digi->hw->wiphy->perm_addr, ETH_ALEN);
        }
    }
    else
    {
        digi->rtsCtsRate = NULL;
    }
}


#define GETU32(pt) (((u32)(pt)[0] << 24) ^ ((u32)(pt)[1] << 16) ^ ((u32)(pt)[2] <<  8) ^ ((u32)(pt)[3]))
/* Get 16 bits at byte pointer */
#define	GET16(bp)		((bp)[0] | ((bp)[1] << 8))
/* Get 32 bits at byte pointer */
#define	GET32(bp)		((bp)[0] | ((bp)[1] << 8) | ((bp)[2] << 16) | ((bp)[3] << 24))
/* Store 16 bits at byte pointer */
#define	SET16(bp, data)		{ (bp)[0] = (data); \
				(bp)[1] = (data) >> 8; }
/* Store 32 bits at byte pointer */
#define	SET32(bp, data)		{ (bp)[0] = (data); \
				(bp)[1] = (data) >> 8;  \
				(bp)[2] = (data) >> 16; \
				(bp)[3] = (data) >> 24; }

static inline void dw_inc_48(u48* n)
{
	(*n)++;
	*n &= ((u64) 1 << 48) - 1;
}

#define EXT_IV_IS_PRESENT       (0x20)
void piperGenerateAesExtIV(struct piper_priv *digi, unsigned int keyIndex, u8 *extiv)
{
	/* Increment packet number */
	dw_inc_48(&digi->key[keyIndex].txPn);
    memset(extiv, 0, PIPER_EXTIV_SIZE);

	SET16(extiv, digi->key[keyIndex].txPn & 0xffff);
	extiv[3] = (keyIndex << 6) | EXT_IV_IS_PRESENT;

	SET32(&extiv[4], digi->key[keyIndex].txPn >> 16);
}

/*
 * This function prepares a blob of data we will have to send to the AES 
 * H/W encryption engine.  The data consists of the AES initialization 
 * vector and 2 16 byte headers.
 *
 * Returns true if successful, or false if something goes wrong
 */
void dumpWordsAdd(unsigned int word);
void dumpWordsDump(void);
bool piperPrepareAESDataBlob(struct piper_priv *digi, unsigned int keyIndex, 
                             u8 *aesBlob, u8 *frame, u32 length, bool isTransmit)
{
//
// 802.11 MAC frame formats
//
    bool result = false;
    _80211HeaderType *header = (_80211HeaderType *) frame;
    u8 *body = &frame[sizeof(*header)];
	int dlen = length - _80211_HEADER_LENGTH;
    
    if (keyIndex >= PIPER_MAX_KEYS) 
    {
        digi_dbg("encryption key index %d is out of range.\n", keyIndex);
        goto prepareAESBlobDone;
    }
    
    if (digi->key[keyIndex].valid == false)
    {
        digi_dbg("encryption key %d is not valid.\n", keyIndex);
        goto prepareAESBlobDone;
    }
    digi_dbg("Preparing AES blob for key %d.\n", keyIndex);
    digi_dbg("header->squ.sq.frag = %d\n", header->squ.sq.frag);
    
	// Set up CCM initial block for MIC IV
	memset(aesBlob, 0, AES_BLOB_LENGTH);
	aesBlob[0] = 0x59;
	aesBlob[1] = 0;
	memcpy (&aesBlob[2], header->addr2, ETH_ALEN);
	
	if (isTransmit)
	{
	    

	    piperGenerateAesExtIV(digi, keyIndex, digi->txExtIV);
	    
    	aesBlob[8]  = digi->txExtIV[7];
    	aesBlob[9]  = digi->txExtIV[6];
    	aesBlob[10] = digi->txExtIV[5];
    	aesBlob[11] = digi->txExtIV[4];
    	aesBlob[12] = digi->txExtIV[1];
    	aesBlob[13] = digi->txExtIV[0];
	}
	else
	{
    	aesBlob[8]  = body[7];
    	aesBlob[9]  = body[6];
    	aesBlob[10] = body[5];
    	aesBlob[11] = body[4];
    	aesBlob[12] = body[1];
    	aesBlob[13] = body[0];
    }
	aesBlob[14] = dlen >> 8;
	aesBlob[15] = dlen;
	
	// Set up MIC header blocks
	/* TODO:  Do I need this: squ.sq16 = NTOH16 (buf->macHdr.squ.sq16); */

#define AES_HEADER_0_OFFSET (16)
#define AES_HEADER_1_OFFSET (32)
	aesBlob[AES_HEADER_0_OFFSET+0] = 0;
	aesBlob[AES_HEADER_0_OFFSET+1] = 22;
	aesBlob[AES_HEADER_0_OFFSET+2] = frame[0] & 0xcf;
	aesBlob[AES_HEADER_0_OFFSET+3] = frame[1] & 0xd7;
	/*
	 * This memcpy writes data into the last 12 bytes of the first header
	 * and the first 6 bytes of the 2nd header.  I did it as one memcpy
	 * call for efficiency.
	 */
	memcpy(&aesBlob[AES_HEADER_0_OFFSET+4], header->addr1, 3*ETH_ALEN);
	aesBlob[AES_HEADER_1_OFFSET+6] = header->squ.sq.frag;
	aesBlob[AES_HEADER_1_OFFSET+7] = 0;
	memset (&aesBlob[AES_HEADER_1_OFFSET+8], 0, 8);	/* clear vector location in data */
	
	result = true;
	
prepareAESBlobDone:
    return result;
}


static int hw_tx(struct ieee80211_hw *hw, struct sk_buff *skb)
{
	struct piper_priv *digi = hw->priv;
	struct ieee80211_tx_info *txInfo = IEEE80211_SKB_CB(skb);

    if (digi->isRadioOn == false)
    {
        /*
         * If we come here, then someone told us to turn our radio off.  This
         * function is never suppose to fail, but they told us to turn our 
         * radio off!
         */
        return -EBUSY;
    }
    ieee80211_stop_queues(hw);      /* only transmit one packet at a time */
	if (txInfo->flags & IEEE80211_TX_CTL_ASSIGN_SEQ)
	{
	    assignSequenceNumber(skb, !!(txInfo->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT));
	}
	setupRtsCtsFrame(digi, skb, txInfo);
    if (txInfo->control.hw_key != NULL)
    {
        digi->txAesKey = txInfo->control.hw_key->hw_key_idx;
        digi_dbg("digi->txAesKey set to %d.\n", digi->txAesKey);
        digi->useAesHwEncryption = piperPrepareAESDataBlob(digi, 
                                        txInfo->control.hw_key->hw_key_idx, 
                                        (u8 *) digi->txAesBlob, skb->data, skb->len,
                                        true);
    }
    else
    {
        digi->useAesHwEncryption = false;
    }
	/* set the plcp/frame header (adding space for AES MIC tail) */
	skb_push(skb, TX_HEADER_LENGTH);
	digi->txRetryIndex = 0;
    digi->txMaxRetries = txInfo->control.retry_limit;
	memcpy(&txInfo->status.retries[1], &txInfo->control.retries[0], sizeof(txInfo->control.retries));
	txInfo->status.retries[0].rate_idx = txInfo->tx_rate_idx;
	txInfo->status.retries[0].limit = 1;
	txInfo->status.ampdu_ack_map = 0;
	txInfo->status.ack_signal = 0;
	txInfo->status.retry_count = 0;
	txInfo->status.excessive_retries = false;
	txInfo->status.ampdu_ack_len = 0;
    txInfo->flags &= ~(IEEE80211_TX_STAT_TX_FILTERED
                                | IEEE80211_TX_STAT_ACK
                                | IEEE80211_TX_STAT_AMPDU
                                | IEEE80211_TX_STAT_AMPDU_NO_BACK);
	digi->txPacket = skb;
	tasklet_hi_schedule(&digi->txRetryTasklet);
	
	return 0;
}

static int hw_start(struct ieee80211_hw *hw)
{
	int err = 0;
	struct piper_priv *digi = hw->priv;

	digi->if_type = __NL80211_IFTYPE_AFTER_LAST;

	/* initialize */
	err = digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_INIT, op_write);
	if (err)
		goto done;

    err = digi->initHw(digi);
    digi->isRadioOn = true;
	
	err = set_antenna_div(hw, ANTENNA_BOTH);
	if (err)
	{
	    digi_dbg("set_antenna_div failed, err = %d\n", err);
		goto done;
    }

	/* set status led to link off */
	err = set_status_led(hw, 0);

    tasklet_enable(&digi->rxTasklet);
    tasklet_enable(&digi->txRetryTasklet);
    digi->clearIrqMaskBit(digi, 0xffffffff);
    digi->setIrqMaskBit(digi, BB_IRQ_MASK_RX_OVERRUN | BB_IRQ_MASK_RX_FIFO);
	enable_irq(digi->irq);
	if (err)
		goto done;

	memset(digi->bssid, 0, ETH_ALEN);
done:

	return err;
}

static void hw_stop(struct ieee80211_hw *hw)
{
	struct piper_priv *digi = hw->priv;

	digi_dbg("hw_stop called\n");
	/* set status led to link off */
	if (set_status_led(hw, 0))
		return;		/* hardware's probably gone, give up */

	/* turn off phy */
	digi->rf->stop(hw);

	/* turn off MAX_GAIN, ADC clocks, and so on */
	digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RESET, op_and);

	/* turn off MAC control/mac filt/aes key */
	digi->write_reg(digi, MAC_CTL, 0, op_write);

	/* turn off interrupts */
    tasklet_disable(&digi->rxTasklet);
    tasklet_disable(&digi->txRetryTasklet);
	digi->irq_mask = 0;
	digi->clearIrqMaskBit(digi, 0xffffffff);
	disable_irq(digi->irq);
}

static int hw_add_intf(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	int err = 0;
	struct piper_priv *digi = hw->priv;

	digi_dbg("hw_add_intf called\n");
	digi_dbg("if_type: %x\n", conf->type);

	/* __NL80211_IFTYPE_AFTER_LAST means no mode selected */
	if (digi->if_type != __NL80211_IFTYPE_AFTER_LAST)
	{
	    digi_dbg("hw_add_intf (digi->if_type != __NL80211_IFTYPE_AFTER_LAST)\n");
		err = -EOPNOTSUPP;
		goto done;
    }
    
	switch (conf->type) 
	{
    	case NL80211_IFTYPE_ADHOC:
    	case NL80211_IFTYPE_STATION:
    	case NL80211_IFTYPE_MESH_POINT:
    		digi->if_type = conf->type;
    		break;
    	default:
	        digi_dbg("hw_add_intf conf->type is unsupported\n");
    		return -EOPNOTSUPP;
	}

done:

	return err;
}

static void hw_rm_intf(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct piper_priv *digi = hw->priv;

	digi_dbg("hw_rm_intf called\n");

	digi->if_type = __NL80211_IFTYPE_AFTER_LAST;

}

static int hw_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	struct piper_priv *digi = hw->priv;
	int chan;
	int err = 0;
    unsigned int word;
    
	digi_dbg("hw_config called\n");
	digi->isRadioOn = (conf->radio_enabled != 0);
	if (digi->isRadioOn)
	{
	    digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_RX_EN, op_or);
	}
	else
	{
	    digi_dbg("Turning radio off\n");
	    err = digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RX_EN, op_and);
	    /*
	     * Besides turning off the receiver, the transmit function will fail
	     * if digi->isRadioOn = false.
	     */
	    goto done;
	}
	word = digi->read_reg(digi, MAC_DTIM_PERIOD) & ~MAC_BEACON_INTERVAL_MASK;
	word |= conf->beacon_int << MAC_BEACON_INTERVAL_SHIFT;
	word &= ~MAC_LISTEN_INTERVAL_MASK;
	word |= conf->listen_interval;
	digi->write_reg(digi, MAC_DTIM_PERIOD, word, op_write);
	
	err = set_tx_power(hw, conf->power_level);
	if (err)
	{
	    goto done;
	}
	err = set_antenna_div(hw, conf->antenna_sel_tx);
	if (err)
	{
	    goto done;
	}
	
	if (conf->flags & IEEE80211_CONF_SHORT_SLOT_TIME)
		digi_dbg("flag IEEE80211_CONF_SHORT_SLOT_TIME set\n");
	if (conf->flags & IEEE80211_CONF_RADIOTAP)
		digi_dbg("flag IEEE80211_CONF_RADIOTAP set\n");
	if (conf->flags & IEEE80211_CONF_SUPPORT_HT_MODE)
		digi_dbg("flag IEEE80211_CONF_SUPPORT_HT_MODE set\n");

	chan = ieee80211_frequency_to_channel(conf->channel->center_freq);
	if (chan != digi->channel) {
		err = digi->rf->set_chan(hw, chan);
		if (err) {
			printk(KERN_ERR PIPER_DRIVER_NAME
					": unable to set channel (%d)\n", chan);
			goto done;
		} else
			digi->channel = chan;
	}

done:
	return err;
}

static int hw_config_intf(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_if_conf *conf)
{
	int err = -EOPNOTSUPP;
	struct piper_priv *digi = hw->priv;
	__be32 tmp;
    unsigned int word, bssid[2];

	digi_dbg("conf->changed = %d\n", conf->changed);
	if (conf->changed & IEEE80211_IFCC_BSSID)
	{
    	digi_dbg("bssid: %08x, conf->ssid_len = %d\n", *((uint32_t *) conf->bssid), conf->ssid_len);
        bssid[0] = conf->bssid[ 3 ] | conf->bssid[ 2 ] << 8 | conf->bssid[ 1 ] << 16 | conf->bssid[ 0 ] << 24;
        bssid[1] = conf->bssid[ 5 ] << 16 | conf->bssid[ 4 ] << 24;
        if ((bssid[0] == 0) && (bssid[1] == 0))
        {
            /*
             * If we come here, then the MAC layer is telling us to set a 0
             * SSID.  In this case, we really want to set the SSID to the broadcast
             * address so that we receive broadcasts.
             */
            bssid[0] = 0xffffffff;
            bssid[1] = 0xffffffff;
        }

        digi->write_reg(digi, MAC_BSS_ID0, bssid[0], op_write);
        digi->write_reg(digi, MAC_BSS_ID1, bssid[1], op_write);
    	memcpy(digi->bssid, conf->bssid, ETH_ALEN);
    }
    if (conf->changed & IEEE80211_IFCC_SSID)
    {
    	if (conf->ssid_len) 
    	{
    	    int i;
    	    
    		digi_dbg("setting ssid=%s\n", conf->ssid);
            for (i = 0; i < conf->ssid_len; i += 4)
            {
                memcpy(&word, &conf->ssid[i], sizeof(word));
                word = cpu_to_be32(word);
                digi->write_reg(digi, MAC_SSID+i, word, op_write);
            }
    	}
		tmp = digi->read_reg(digi, MAC_SSID_LEN) & ~MAC_SSID_LEN_MASK;
        tmp |= conf->ssid_len;
		err = digi->write_reg(digi, MAC_SSID_LEN, tmp, op_write);
    }
    if (conf->changed & IEEE80211_IFCC_BEACON)
    {
        struct sk_buff *beacon = ieee80211_beacon_get(hw, vif);
        struct ieee80211_rate rate;
        
        rate.bitrate = 10;              /* beacons always sent at 1 Megabit*/
        skb_push(beacon, TX_HEADER_LENGTH);
        phy_set_plcp(beacon->data, beacon->len - TX_HEADER_LENGTH, &rate, 0);
        digi->write_reg(digi, MAC_CTL, ~MAC_CTL_BEACON_TX, op_and);
        digi->load_beacon(digi, beacon->data, beacon->len);
        digi_dbg("Beacon has been loaded\n");
        /* TODO: digi->beacon.enabled should be set by IEEE80211_IFCC_BEACON_ENABLED
                 when we update to latest mac80211 */ digi->beacon.enabled = true;
        err = enableIbssSupport(digi, vif->type);
        dev_kfree_skb(beacon);          /* we are responsible for freeing this buffer*/
    }
    
	return err;
}

static void hw_configure_filter(struct ieee80211_hw *hw,
		unsigned int changed_flags, unsigned int *total_flags,
		int mc_count, struct dev_addr_list *mclist)
{
	/* we don't support filtering so clear all flags; however, we also
	 * can't pass failed FCS/PLCP frames, so don't clear those. */
	*total_flags &= (FIF_FCSFAIL | FIF_PLCPFAIL);
}

/*
 * Update the LED depending upon whether we've associated.
 */
static void hw_bss_changed(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		struct ieee80211_bss_conf *conf, u32 changed)
{
	struct piper_priv *digi = hw->priv;
	unsigned int reg;
	
	digi_dbg("hw_bss_changed called\n");
	if (changed & BSS_CHANGED_ASSOC)
	{
		set_status_led(hw, conf->assoc ? 1 : 0);
    }
    if (changed & BSS_CHANGED_ERP_CTS_PROT)
    {
        digi->bssWantCtsProtection = conf->use_cts_prot;
    }
    if (changed & BSS_CHANGED_ERP_PREAMBLE)
    {
#define WANT_SHORT_PREAMBLE_SUPPORT     (0)
/* TODO: Determine if short preambles really hurt us, or if I'm just seeing things. */
#if WANT_SHORT_PREAMBLE_SUPPORT
        if (conf->use_short_preamble)
        {
            digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_SH_PRE, op_or);
        }
        else
        {
            digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_SH_PRE, op_and);
        }
#else
            digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_SH_PRE, op_or);
#endif
    }
    if (changed & BSS_CHANGED_BASIC_RATES)
    {
        unsigned int word = digi->read_reg(digi, MAC_SSID_LEN) 
                        & ~(MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK);
        unsigned int ofdm = 0, psk = 0;
        
        digi->rf->getOfdmBrs(conf->basic_rates, &ofdm, &psk);
        
        word |= ofdm << MAC_OFDM_BRS_SHIFT;
        word |= psk << MAC_PSK_BRS_SHIFT;
        digi->write_reg(digi, MAC_SSID_LEN, word, op_write);
        digi_dbg("BRS mask set to 0x%8.8X\n", word);
        if (ofdm == 0)
        {
            /*
             * Disable ofdm receiver if no ofdm rates supported.
             */
            digi->write_reg(digi, BB_GENERAL_STAT, ~BB_GENERAL_STAT_A_EN, op_and);
        }
        else
        {
            /*
             * Enable ofdm receiver if any ofdm rates supported.
             */
            digi->write_reg(digi, BB_GENERAL_STAT, BB_GENERAL_STAT_A_EN, op_or);
        }
            
    }
    reg = digi->read_reg(digi, MAC_DTIM_PERIOD) & ~MAC_DTIM_PERIOD_MASK;
    reg |= conf->dtim_period << MAC_DTIM_PERIOD_SHIFT;
    digi->write_reg(digi, MAC_DTIM_PERIOD, reg, op_write);
    reg = digi->read_reg(digi, MAC_CFP_ATIM) & ~MAC_DTIM_CFP_MASK;
    reg |= conf->beacon_int << 16;
    digi->write_reg(digi, MAC_CFP_ATIM, reg, op_write);

}


#if 0
static int expand_aes_key(struct ieee80211_key_conf *key, u32 *expandedKey)
{
	struct crypto_aes_ctx aes;
	int err, i;

	if (key->keylen != AES_KEYSIZE_128)
		return -EOPNOTSUPP;
	
	err = crypto_aes_expand_key(&aes, key->key, key->keylen);
	if (err)
		return -EOPNOTSUPP;

    memcpy(expandedKey, aes.key_enc, EXPANDED_KEY_LENGTH);
    for (i = 0; i < (EXPANDED_KEY_LENGTH / sizeof(u32)); i++)
    {
        expandedKey[i] = cpu_to_be32(expandedKey[i]);
    }
	return 0;
}
#endif
/* CCMP encryption data */
static const u32 te4[256] = {
	0x63636363U, 0x7c7c7c7cU, 0x77777777U, 0x7b7b7b7bU,
	0xf2f2f2f2U, 0x6b6b6b6bU, 0x6f6f6f6fU, 0xc5c5c5c5U,
	0x30303030U, 0x01010101U, 0x67676767U, 0x2b2b2b2bU,
	0xfefefefeU, 0xd7d7d7d7U, 0xababababU, 0x76767676U,
	0xcacacacaU, 0x82828282U, 0xc9c9c9c9U, 0x7d7d7d7dU,
	0xfafafafaU, 0x59595959U, 0x47474747U, 0xf0f0f0f0U,
	0xadadadadU, 0xd4d4d4d4U, 0xa2a2a2a2U, 0xafafafafU,
	0x9c9c9c9cU, 0xa4a4a4a4U, 0x72727272U, 0xc0c0c0c0U,
	0xb7b7b7b7U, 0xfdfdfdfdU, 0x93939393U, 0x26262626U,
	0x36363636U, 0x3f3f3f3fU, 0xf7f7f7f7U, 0xccccccccU,
	0x34343434U, 0xa5a5a5a5U, 0xe5e5e5e5U, 0xf1f1f1f1U,
	0x71717171U, 0xd8d8d8d8U, 0x31313131U, 0x15151515U,
	0x04040404U, 0xc7c7c7c7U, 0x23232323U, 0xc3c3c3c3U,
	0x18181818U, 0x96969696U, 0x05050505U, 0x9a9a9a9aU,
	0x07070707U, 0x12121212U, 0x80808080U, 0xe2e2e2e2U,
	0xebebebebU, 0x27272727U, 0xb2b2b2b2U, 0x75757575U,
	0x09090909U, 0x83838383U, 0x2c2c2c2cU, 0x1a1a1a1aU,
	0x1b1b1b1bU, 0x6e6e6e6eU, 0x5a5a5a5aU, 0xa0a0a0a0U,
	0x52525252U, 0x3b3b3b3bU, 0xd6d6d6d6U, 0xb3b3b3b3U,
	0x29292929U, 0xe3e3e3e3U, 0x2f2f2f2fU, 0x84848484U,
	0x53535353U, 0xd1d1d1d1U, 0x00000000U, 0xededededU,
	0x20202020U, 0xfcfcfcfcU, 0xb1b1b1b1U, 0x5b5b5b5bU,
	0x6a6a6a6aU, 0xcbcbcbcbU, 0xbebebebeU, 0x39393939U,
	0x4a4a4a4aU, 0x4c4c4c4cU, 0x58585858U, 0xcfcfcfcfU,
	0xd0d0d0d0U, 0xefefefefU, 0xaaaaaaaaU, 0xfbfbfbfbU,
	0x43434343U, 0x4d4d4d4dU, 0x33333333U, 0x85858585U,
	0x45454545U, 0xf9f9f9f9U, 0x02020202U, 0x7f7f7f7fU,
	0x50505050U, 0x3c3c3c3cU, 0x9f9f9f9fU, 0xa8a8a8a8U,
	0x51515151U, 0xa3a3a3a3U, 0x40404040U, 0x8f8f8f8fU,
	0x92929292U, 0x9d9d9d9dU, 0x38383838U, 0xf5f5f5f5U,
	0xbcbcbcbcU, 0xb6b6b6b6U, 0xdadadadaU, 0x21212121U,
	0x10101010U, 0xffffffffU, 0xf3f3f3f3U, 0xd2d2d2d2U,
	0xcdcdcdcdU, 0x0c0c0c0cU, 0x13131313U, 0xececececU,
	0x5f5f5f5fU, 0x97979797U, 0x44444444U, 0x17171717U,
	0xc4c4c4c4U, 0xa7a7a7a7U, 0x7e7e7e7eU, 0x3d3d3d3dU,
	0x64646464U, 0x5d5d5d5dU, 0x19191919U, 0x73737373U,
	0x60606060U, 0x81818181U, 0x4f4f4f4fU, 0xdcdcdcdcU,
	0x22222222U, 0x2a2a2a2aU, 0x90909090U, 0x88888888U,
	0x46464646U, 0xeeeeeeeeU, 0xb8b8b8b8U, 0x14141414U,
	0xdedededeU, 0x5e5e5e5eU, 0x0b0b0b0bU, 0xdbdbdbdbU,
	0xe0e0e0e0U, 0x32323232U, 0x3a3a3a3aU, 0x0a0a0a0aU,
	0x49494949U, 0x06060606U, 0x24242424U, 0x5c5c5c5cU,
	0xc2c2c2c2U, 0xd3d3d3d3U, 0xacacacacU, 0x62626262U,
	0x91919191U, 0x95959595U, 0xe4e4e4e4U, 0x79797979U,
	0xe7e7e7e7U, 0xc8c8c8c8U, 0x37373737U, 0x6d6d6d6dU,
	0x8d8d8d8dU, 0xd5d5d5d5U, 0x4e4e4e4eU, 0xa9a9a9a9U,
	0x6c6c6c6cU, 0x56565656U, 0xf4f4f4f4U, 0xeaeaeaeaU,
	0x65656565U, 0x7a7a7a7aU, 0xaeaeaeaeU, 0x08080808U,
	0xbabababaU, 0x78787878U, 0x25252525U, 0x2e2e2e2eU,
	0x1c1c1c1cU, 0xa6a6a6a6U, 0xb4b4b4b4U, 0xc6c6c6c6U,
	0xe8e8e8e8U, 0xddddddddU, 0x74747474U, 0x1f1f1f1fU,
	0x4b4b4b4bU, 0xbdbdbdbdU, 0x8b8b8b8bU, 0x8a8a8a8aU,
	0x70707070U, 0x3e3e3e3eU, 0xb5b5b5b5U, 0x66666666U,
	0x48484848U, 0x03030303U, 0xf6f6f6f6U, 0x0e0e0e0eU,
	0x61616161U, 0x35353535U, 0x57575757U, 0xb9b9b9b9U,
	0x86868686U, 0xc1c1c1c1U, 0x1d1d1d1dU, 0x9e9e9e9eU,
	0xe1e1e1e1U, 0xf8f8f8f8U, 0x98989898U, 0x11111111U,
	0x69696969U, 0xd9d9d9d9U, 0x8e8e8e8eU, 0x94949494U,
	0x9b9b9b9bU, 0x1e1e1e1eU, 0x87878787U, 0xe9e9e9e9U,
	0xcecececeU, 0x55555555U, 0x28282828U, 0xdfdfdfdfU,
	0x8c8c8c8cU, 0xa1a1a1a1U, 0x89898989U, 0x0d0d0d0dU,
	0xbfbfbfbfU, 0xe6e6e6e6U, 0x42424242U, 0x68686868U,
	0x41414141U, 0x99999999U, 0x2d2d2d2dU, 0x0f0f0f0fU,
	0xb0b0b0b0U, 0x54545454U, 0xbbbbbbbbU, 0x16161616U,
};

static const u32 rcon[] = {
	/* for 128-bit blocks, Rijndael never uses more than 10 rcon values */
	0x01000000, 0x02000000,
	0x04000000, 0x08000000,
	0x10000000, 0x20000000,
	0x40000000, 0x80000000,
	0x1B000000, 0x36000000,
};

/***********************************************************************
 * @Function: dw_aes_set_encrypt_key
 * @Return:
 * @Descr: Expands the cipher key into the encryption key schedule
 ***********************************************************************/
static int expand_aes_key(struct ieee80211_key_conf *key,
		u32 *rk)
{
	int i = 0;
	u32 temp;
	unsigned char *user_key = key->key;
	int bits = key->keylen * 8;

	if (!user_key || !key)
		return -1;
	if (bits != 128 && bits != 192 && bits != 256)
		return -2;

    memset(rk, 0, EXPANDED_KEY_LENGTH);
	rk[0] = GETU32(user_key    );
	rk[1] = GETU32(user_key +  4);
	rk[2] = GETU32(user_key +  8);
	rk[3] = GETU32(user_key + 12);
	if (bits == 128) {
		while (1) {
			temp  = rk[3];
			rk[4] = rk[0] ^
				(te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(te4[(temp     ) & 0xff] & 0x0000ff00) ^
				(te4[(temp >> 24)       ] & 0x000000ff) ^
				rcon[i];
			rk[5] = rk[1] ^ rk[4];
			rk[6] = rk[2] ^ rk[5];
			rk[7] = rk[3] ^ rk[6];
			if (++i == 10) {
				return 0;
			}
			rk += 4;
		}
	}
	rk[4] = GETU32(user_key + 16);
	rk[5] = GETU32(user_key + 20);
	if (bits == 192) {
		while (1) {
			temp = rk[ 5];
			rk[ 6] = rk[ 0] ^
				(te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(te4[(temp     ) & 0xff] & 0x0000ff00) ^
				(te4[(temp >> 24)       ] & 0x000000ff) ^
				rcon[i];
			rk[ 7] = rk[ 1] ^ rk[ 6];
			rk[ 8] = rk[ 2] ^ rk[ 7];
			rk[ 9] = rk[ 3] ^ rk[ 8];
			if (++i == 8) {
				return 0;
			}
			rk[10] = rk[ 4] ^ rk[ 9];
			rk[11] = rk[ 5] ^ rk[10];
			rk += 6;
		}
	}
	rk[6] = GETU32(user_key + 24);
	rk[7] = GETU32(user_key + 28);
	if (bits == 256) {
		while (1) {
			temp = rk[ 7];
			rk[ 8] = rk[ 0] ^
				(te4[(temp >> 16) & 0xff] & 0xff000000) ^
				(te4[(temp >>  8) & 0xff] & 0x00ff0000) ^
				(te4[(temp     ) & 0xff] & 0x0000ff00) ^
				(te4[(temp >> 24)       ] & 0x000000ff) ^
				rcon[i];
			rk[ 9] = rk[ 1] ^ rk[ 8];
			rk[10] = rk[ 2] ^ rk[ 9];
			rk[11] = rk[ 3] ^ rk[10];
			if (++i == 7) {
				return 0;
			}
			temp = rk[11];
			rk[12] = rk[ 4] ^
				(te4[(temp >> 24)       ] & 0xff000000) ^
				(te4[(temp >> 16) & 0xff] & 0x00ff0000) ^
				(te4[(temp >>  8) & 0xff] & 0x0000ff00) ^
				(te4[(temp     ) & 0xff] & 0x000000ff);
			rk[13] = rk[ 5] ^ rk[12];
			rk[14] = rk[ 6] ^ rk[13];
			rk[15] = rk[ 7] ^ rk[14];

			rk += 8;
		}
	}
	return 0;
}


static int hw_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		const u8 *local_address, const u8 *address,
		struct ieee80211_key_conf *key)
{
	int err = -EOPNOTSUPP;
	struct piper_priv *digi = hw->priv;

/** TODO: Remove this */ goto hw_set_key_done;
	if (cmd == SET_KEY)
	{
	    unsigned char *k = key->key;
	    
	    digi_dbg("index = %d, %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n", key->keyidx, address[0], address[1], address[2], address[3], address[4], address[5]);
	    digi_dbg("key = %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X  %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X %2.2X \n", 
	    k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8], k[9], k[10], k[11], k[12], k[13], k[14], k[15]);
	    
	}
	else
	{
	    digi_dbg("disable key index = %d\n", key->keyidx);
	}

    if ((key->alg != ALG_CCMP) || (key->keyidx >= PIPER_MAX_KEYS))
    {
        /*
         * If we come here, then mac80211 was trying to set a key for some
         * algorithm other than AES, or trying to set a key index greater
         * than 3.  We only support AES, and only 4 keys.
         */
        err = -EOPNOTSUPP;
        goto hw_set_key_done;
    }

	key->hw_key_idx = key->keyidx;
	if (cmd == SET_KEY) 
	{
		err = expand_aes_key(key, digi->key[key->keyidx].expandedKey);
		if ((!digi->key[key->keyidx].valid) && (err == 0))
		{
		    digi->AESKeyCount++;
		}
		digi->key[key->keyidx].txPn = 0;
		digi->key[key->keyidx].rxPn = 0;
		digi->key[key->keyidx].valid = (err == 0);
	} 
	else 
	{
		/* disable key */
		if (digi->key[key->keyidx].valid)
		{
		    digi->AESKeyCount--;
		}
		digi->key[key->keyidx].valid = false;
		err = 0;
	}
	
    if (digi->AESKeyCount > 0)
    {
        digi->write_reg(digi, MAC_CTL, ~MAC_CTL_AES_DISABLE, op_and);
    }
    else
    {
        digi->write_reg(digi, MAC_CTL, MAC_CTL_AES_DISABLE, op_or);
    }

hw_set_key_done:
	if (err)
		printk(KERN_ERR PIPER_DRIVER_NAME ": unable to set AES key\n");

    return err;
}


/*
 * Return nonzero if we sent the last beacon on an IBSS.
 * Note the function result is not 100% reliable.
 */
int hw_tx_last_beacon(struct ieee80211_hw *hw)
{
    struct piper_priv *digi = hw->priv;

    digi_dbg("returning %d.\n", digi->beacon.weSentLastOne ? 1 : 0);
    
    return digi->beacon.weSentLastOne ? 1 : 0;
}


const struct ieee80211_ops hw_ops = {
	.tx = hw_tx,
	.start = hw_start,
	.stop = hw_stop,
	.add_interface = hw_add_intf,
	.remove_interface = hw_rm_intf,
	.config = hw_config,
	.config_interface = hw_config_intf,
	.configure_filter = hw_configure_filter,
	.bss_info_changed = hw_bss_changed,
	.tx_last_beacon = hw_tx_last_beacon,
	.set_key = hw_set_key,
};

int piper_alloc_hw(struct piper_priv **priv, size_t priv_sz)
{
	struct piper_priv *digi;
	struct ieee80211_hw *hw;

	hw = ieee80211_alloc_hw(priv_sz, &hw_ops);
	if (!hw)
		return -ENOMEM;

	hw->flags |= IEEE80211_HW_RX_INCLUDES_FCS
	             | IEEE80211_HW_SIGNAL_DBM
	             | IEEE80211_HW_NOISE_DBM
#if !WANT_SHORT_PREAMBLE_SUPPORT
	             /* TODO: Remove this */ | IEEE80211_HW_2GHZ_SHORT_SLOT_INCAPABLE
#endif
	             /* | IEEE80211_HW_SPECTRUM_MGMT TODO:  Turn this on when we are ready*/;
	hw->queues = 1;
	hw->extra_tx_headroom = 4 +
			sizeof(struct ofdm_hdr);
	*priv = digi = hw->priv;
	digi->hw = hw;

	return 0;
}

void piper_free_hw(struct piper_priv *digi)
{
    printk(KERN_INFO "piper_free_hw called\n");
    ieee80211_free_hw(digi->hw);
    printk(KERN_INFO "piper_free_hw returning\n");
}


EXPORT_SYMBOL_GPL(piper_alloc_hw);
EXPORT_SYMBOL_GPL(piper_free_hw);

int piper_register_hw(struct piper_priv *priv, struct device *dev,
		struct digi_rf_ops *rf)
{
	struct ieee80211_hw *hw = priv->hw;
	int i, err;
	unsigned char macAddress[2*sizeof(unsigned int)];
    unsigned int word;
    
	digi_dbg("piper_register_hw called\n");
	priv->rf = rf;
	for (i = 0; i < rf->n_bands; i++) {
		enum ieee80211_band b = rf->bands[i].band;
		hw->wiphy->bands[b] = &rf->bands[i];
	}
	hw->channel_change_time = rf->channelChangeTime;
	hw->vif_data_size = 0;
	hw->sta_data_size = 0;
	hw->max_altrates = IEEE80211_TX_MAX_ALTRATE;
#define OUR_MAX_RETRIES     (10)        /* completely arbitrary.  We don't care how many times we retry */
	hw->max_altrate_tries = OUR_MAX_RETRIES;



	SET_IEEE80211_DEV(hw, dev);
	
    word = cpu_to_be32(priv->read_reg(priv, MAC_STA_ID0));
    memcpy(macAddress, &word, sizeof(word));
    word = cpu_to_be32(priv->read_reg(priv, MAC_STA_ID1));
    memcpy(&macAddress[4], &word, sizeof(word));
    SET_IEEE80211_PERM_ADDR(hw, macAddress);
    
	err = ieee80211_register_hw(hw);
	if (err)
		printk(KERN_ERR PIPER_DRIVER_NAME ": failed to register ieee80211 hw\n");
	else
		printk(KERN_INFO PIPER_DRIVER_NAME ": registered as %s\n",
				wiphy_name(hw->wiphy));
	return err;
}
EXPORT_SYMBOL_GPL(piper_register_hw);

void piper_unregister_hw(struct piper_priv *digi)
{
	digi_dbg("piper_unregister_hw called\n");
	ieee80211_unregister_hw(digi->hw);
}
EXPORT_SYMBOL_GPL(piper_unregister_hw);


MODULE_DESCRIPTION("Digi Piper WLAN core");
MODULE_AUTHOR("contact support@digi.com for information about this code");
MODULE_LICENSE("GPL");
