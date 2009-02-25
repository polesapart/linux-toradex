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
	digi_dbg("forcing sel to be ANTENNA_1\n");
	sel = ANTENNA_1;
	
	/* set antenna diversity */
	if (sel == ANTENNA_BOTH)
	{
		err = digi->write_reg(digi, BB_GENERAL_CTL,
				BB_GENERAL_CTL_ANT_DIV | digi->read_reg(digi, BB_GENERAL_CTL));
		err = digi->write_reg(digi, BB_GENERAL_CTL,
				~BB_GENERAL_CTL_ANT_SEL & digi->read_reg(digi, BB_GENERAL_CTL));
	}
	else {
		err = digi->write_reg(digi, BB_GENERAL_CTL,
				~BB_GENERAL_CTL_ANT_DIV & digi->read_reg(digi, BB_GENERAL_CTL));
		if (err)
			goto done;

		/* selected the antenna if !diversity */
		if (sel == ANTENNA_1)
			err = digi->write_reg(digi, BB_GENERAL_CTL,
					~BB_GENERAL_CTL_ANT_SEL & digi->read_reg(digi, BB_GENERAL_CTL));
		else
			err = digi->write_reg(digi, BB_GENERAL_CTL,
					BB_GENERAL_CTL_ANT_SEL | digi->read_reg(digi, BB_GENERAL_CTL));
	}
	if (err)
		goto done;
	
	/* select which antenna to transmit on */
	err = digi->write_reg(digi, BB_RSSI, ~BB_RSSI_ANT_MASK & digi->read_reg(digi, BB_RSSI));
	if (err)
		goto done;
	if (sel == ANTENNA_BOTH)
		err = digi->write_reg(digi, BB_RSSI, BB_RSSI_ANT_DIV_MAP | digi->read_reg(digi, BB_RSSI));
    else
		err = digi->write_reg(digi, BB_RSSI, BB_RSSI_ANT_NO_DIV_MAP | digi->read_reg(digi, BB_RSSI));
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
		err = digi->write_reg(digi, BB_RSSI, BB_RSSI_LED | digi->read_reg(digi, BB_RSSI));
	else
		err = digi->write_reg(digi, BB_RSSI, ~BB_RSSI_LED & digi->read_reg(digi, BB_RSSI));
	
	return err;
#else
    return 0;
#endif
}

static int set_tx_power(struct ieee80211_hw *hw, int power)
{
	struct piper_priv *digi = hw->priv;
	int err;

	digi_dbg("set_tx_power called\n");
	if (power == digi->tx_power)
		return 0;

	err = digi->rf->set_pwr(hw, power);
	if (!err)
		digi->tx_power = power;
    
    digi_dbg("tx_power set to %d.\n", digi->tx_power);
	return err;
}

static int hw_tx(struct ieee80211_hw *hw, struct sk_buff *skb,
		struct ieee80211_tx_control *ctl)
{
	struct piper_priv *digi = hw->priv;
#if 0
	int aes_fifo = ctl->flags & IEEE80211_TXCTL_DO_NOT_ENCRYPT ? 0 : 1;
#else
	int aes_fifo = 0;
#endif

    ieee80211_stop_queues(hw);      /* only transmit one packet at a time */
	/* set the plcp/frame header (adding space for AES MIC tail) */
	phy_set_plcp(skb, ctl->tx_rate, aes_fifo ? 8 : 0);

    digi->txStatus.control = *ctl;
	digi->txStatus.ampdu_ack_len = 0;
	digi->txStatus.ampdu_ack_map = 0;
	digi->txStatus.ack_signal = 0;
	digi->txStatus.retry_count = 0;
	digi->txPacket = skb;
	digi->txRetries = ctl->retry_limit;
	tasklet_schedule(&digi->txRetryTasklet);
	
	return 0;
}

static int hw_start(struct ieee80211_hw *hw)
{
	int err = 0;
	struct piper_priv *digi = hw->priv;

	digi_dbg("hw_start called\n");
	digi->if_type = IEEE80211_IF_TYPE_INVALID;

	/* initialize */
	err = digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_INIT);
	udelay(10); /* TODO:  remove this debug code */
	if (err)
		goto done;

    digi_dbg("calling digi->initHw\n");
    err = digi->initHw(digi);
    digi_dbg("digi->initHw returned %d\n", err);
	
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
	digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_RESET & digi->read_reg(digi, BB_GENERAL_CTL));

	/* turn off MAC control/mac filt/aes key */
	digi->write_reg(digi, MAC_CTL, 0);

	/* turn off interrupts */
    tasklet_disable(&digi->rxTasklet);
	digi->irq_mask = 0;
	digi->write_reg(digi, BB_IRQ_MASK, 0);
}

static int hw_add_intf(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	int err = 0;
	struct piper_priv *digi = hw->priv;

	digi_dbg("hw_add_intf called\n");
	digi_dbg("if_type: %x\n", conf->type);

	/* INVALID means no mode selected */
	if (digi->if_type != IEEE80211_IF_TYPE_INVALID)
	{
	    digi_dbg("hw_add_intf (digi->if_type != IEEE80211_IF_TYPE_INVALID)\n");
		return -EOPNOTSUPP;
    }
    
	switch (conf->type) 
	{
    	case IEEE80211_IF_TYPE_IBSS:
    	case IEEE80211_IF_TYPE_STA:
    		digi->if_type = conf->type;
    		break;
    	default:
	        digi_dbg("hw_add_intf conf->type is unsupported\n");
    		return -EOPNOTSUPP;
	}

#if 0
	/* set the MAC address */
	err = digi->write(digi, MAC_STA_ID0, (uint8_t *) hw->wiphy->perm_addr,
			sizeof(hw->wiphy->perm_addr));
	if (err)
	{
	    digi_dbg("hw_add_intf 1st digi->write failed\n");
		goto done;
    }
    
	err = digi->write_reg(digi, MAC_CTL, MAC_CTL_AES_DISABLE | digi->read_reg(digi, MAC_CTL));
	if (err)
	{
	    digi_dbg("hw_add_intf 2nd digi->write failed\n");
		goto done;
    }
#endif

done:

	return err;
}

static void hw_rm_intf(struct ieee80211_hw *hw,
		struct ieee80211_if_init_conf *conf)
{
	struct piper_priv *digi = hw->priv;

	digi_dbg("hw_rm_intf called\n");

	digi->if_type = IEEE80211_IF_TYPE_INVALID;

}

static int hw_config(struct ieee80211_hw *hw, struct ieee80211_conf *conf)
{
	struct piper_priv *digi = hw->priv;
	int chan;
	int err = 0;

	digi_dbg("hw_config called\n");
	err = set_tx_power(hw, conf->power_level);
	if (err)
		goto done;
	
/** TODO:  Support some of this stuff 
	int radio_enabled;

	int beacon_int;
	u32 flags;
	int power_level;
	int max_antenna_gain;
	u8 antenna_sel_tx;
	u8 antenna_sel_rx;

	struct ieee80211_channel *channel;

	struct ieee80211_ht_info ht_conf;
	struct ieee80211_ht_bss_info ht_bss_conf;
***/
	
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

	digi_dbg("hw_config_intf called\n");
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
    digi->write_reg(digi, MAC_BSS_ID0, bssid[0]);
    digi->write_reg(digi, MAC_BSS_ID1, bssid[1]);

	memcpy(digi->bssid, conf->bssid, ETH_ALEN);

	if (conf->ssid_len) 
	{
	    int i;
	    
		digi_dbg("setting ssid=%s\n", conf->ssid);
        for (i = 0; i < conf->ssid_len; i += 4)
        {
            memcpy(&word, &conf->ssid[i], sizeof(word));
            word = cpu_to_be32(word);
            digi->write_reg(digi, MAC_SSID+i, word);
        }
		/* along with the SSID len, set the OFDM and CCK basic rates */
		tmp = conf->ssid_len |
/*** TODO: Put correct values here 
				((RATE_MASK_BASIC & RATE_MASK_OFDM) << 20) |
				((RATE_MASK_BASIC & RATE_MASK_PSK_CCK) << 16);
****/
				0xff0f0000;

		err = digi->write_reg(digi, MAC_SSID_LEN, tmp);
		if (err)
			goto done;
	}
#if 0
	/* handle STA now; IBSS stuff is dealt with later in hw_beacon_update */
	if (vif->type !=  IEEE80211_IF_TYPE_IBSS) {
		/* disable TBTT/ATIM interrupts */
		digi->irq_mask &= ~(BB_IRQ_MASK_TBTT | BB_IRQ_MASK_ATIM);
		err = digi->write_reg(digi, BB_IRQ_MASK, digi->irq_mask);
		if (err)
			goto done;

		/* disable IBSS mode */
		err = digi->write_reg(digi, MAC_CTL,
				~(MAC_CTL_BEACON_TX|MAC_CTL_IBSS) & digi->read_reg(digi, MAC_CTL));
		if (err)
			goto done;

		/* disable HW AES (until set_key is called) */
		err = digi->write_reg(digi, MAC_CTL, MAC_CTL_AES_DISABLE | digi->read_reg(digi, MAC_CTL));
		if (err)
			goto done;
	}
#endif
done:
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
	
	digi_dbg("hw_bss_changed called\n");
	if (changed & BSS_CHANGED_ASSOC)
		set_status_led(hw, conf->assoc ? 1 : 0);
    if (changed & BSS_CHANGED_ERP_PREAMBLE)
    {
        if (conf->use_short_preamble)
        {
            digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_SH_PRE | digi->read_reg(digi, BB_GENERAL_CTL));
        }
        else
        {
            digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_SH_PRE & digi->read_reg(digi, BB_GENERAL_CTL));
        }
    }
}

/*
 * When in IBSS mode, if we update the beacon we should tell the hardware
 * about it (which is generating beacons for us)
 */
static int hw_beacon_update(struct ieee80211_hw *hw, struct sk_buff *skb,
		struct ieee80211_tx_control *ctl)
{
	digi_dbg("hw_beacon_update called\n");
#if 0
	struct piper_priv *digi = hw->priv;
	struct ieee80211_mgmt *mgmt;

	mgmt = (struct ieee80211_mgmt *) skb->data;

	/* set beacon and ATIM intervals */
	digi->write_reg(digi, MAC_CFP_ATIM, /*atim*/0 |
			mgmt->u.beacon.beacon_int << 16);

	/* writing beacon frame to beacon buffer */
	digi->write_reg(digi, BB_GENERAL_CTL, BB_GENERAL_CTL_BEACON_EN | digi->read_reg(digi, BB_GENERAL_CTL));

	/* prepare the beacon frame, and send it off to the tx thread */
	phy_set_plcp(skb, ctl->tx_rate, 0);
	digi->write_fifo(digi, skb, 0);

	digi->write_reg(digi, BB_GENERAL_CTL, ~BB_GENERAL_CTL_BEACON_EN & digi->read_reg(digi, BB_GENERAL_CTL));

	/* enable TBTT and ATIM interrupts */
	digi->irq_mask |= BB_IRQ_MASK_TBTT | BB_IRQ_MASK_ATIM;
	digi->write_reg(digi, BB_IRQ_MASK, digi->irq_mask);

	/* enable IBSS mode, turn off HW AES decryption. */
	digi->write_reg(digi, MAC_CTL, MAC_CTL_IBSS | MAC_CTL_BEACON_TX
			| MAC_CTL_AES_DISABLE | digi->read_reg(digi, MAC_CTL));
	
	/* last register write is synchronous, so skb should've been
	 * processed by now */
	kfree_skb(skb);
#endif
	return 0;
}

static int expand_aes_key(struct ieee80211_key_conf *key, struct sk_buff *skb)
{
#if 0
	struct crypto_aes_ctx aes;
	uint32_t *ptr, *ctx_ptr;
	int err, i;

	if (key->keylen != AES_KEYSIZE_128)
		return -EOPNOTSUPP;
	
	err = crypto_aes_expand_key(&aes, key->key, key->keylen);
	if (err)
		return -EOPNOTSUPP;

	/* expanded key gives us 176 bytes (10 rounds * 16  + 16) */
	ptr = (uint32_t *) skb_put(skb, 176);
	ctx_ptr = (uint32_t *) aes.key_enc;

	/* copy the key int the skb */
	for (i = 0; i < skb->len / sizeof(uint32_t); i++) {
		*ptr = cpu_to_le32(*ctx_ptr);
		ptr++;
		ctx_ptr++;
	}
#endif
	return 0;
}

static int hw_set_key(struct ieee80211_hw *hw, enum set_key_cmd cmd,
		const u8 *local_address, const u8 *address,
		struct ieee80211_key_conf *key)
{
	int err = -EOPNOTSUPP;
	digi_dbg("hw_set_key called\n");
#if 0
	struct piper_priv *digi = hw->priv;
	struct sk_buff *skb;

	switch (key->alg) {
	case ALG_CCMP:
		break;
	default:
		digi_dbg("ignoring ALG type %d\n", key->alg);
		/* only hardware accel support for AES/CCMP, no WEP/TKIP */
		return -EOPNOTSUPP;
	}

	key->hw_key_idx = key->keyidx;
	if (cmd == SET_KEY) {
		skb = dev_alloc_skb(TX_FIFO_SIZE);
		if (!skb)
			return -ENOMEM;

		err = expand_aes_key(key, skb);
		if (err)
			goto done;

		/* turn off AES DISABLE and the key-specific disable */
		err = digi->write_reg(digi, MAC_CTL, ~(MAC_CTL_AES_DISABLE |
				(MAC_CTL_KEY0_DISABLE << key->keyidx)) & digi->read_reg(digi, MAC_CTL));
		if (err)
			goto done;

		/* select mode 1 and load AES key */
		err = digi->write_reg(digi, BB_AES_CTL, BB_AES_CTL_AES_MODE |
				BB_AES_CTL_KEY_LOAD | key->keyidx);

		/* write plaintext key, IV, and data to AES FIFO */
		digi->write_aes_key(digi, skb);
		key->flags |= IEEE80211_KEY_FLAG_GENERATE_IV;

		kfree_skb(skb);
	} else {
		/* disable key */
		digi_dbg("disabling keyidx=%d\n", key->keyidx);
		digi->write_reg(digi, MAC_CTL,
		MAC_CTL_KEY0_DISABLE << key->keyidx | digi->read_reg(digi, MAC_CTL));
		err = 0;
	}

done:
	if (err)
		printk(KERN_ERR PIPER_DRIVER_NAME ": unable to set AES key\n");
#endif
	return err;
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
#if 0
	.beacon_update = hw_beacon_update,
	.set_key = hw_set_key,
#endif
};

int piper_alloc_hw(struct piper_priv **priv, size_t priv_sz)
{
	struct piper_priv *digi;
	struct ieee80211_hw *hw;

	hw = ieee80211_alloc_hw(priv_sz, &hw_ops);
	if (!hw)
		return -ENOMEM;

	hw->flags |= IEEE80211_HW_RX_INCLUDES_FCS |
			IEEE80211_HW_HOST_GEN_BEACON_TEMPLATE;
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

	digi_dbg("piper_register_hw called\n");
	priv->rf = rf;
	for (i = 0; i < rf->n_bands; i++) {
		enum ieee80211_band b = rf->bands[i].band;
		hw->wiphy->bands[b] = &rf->bands[i];
	}

	SET_IEEE80211_DEV(hw, dev);
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
