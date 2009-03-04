#ifndef DIGI_MAIN_H_
#define DIGI_MAIN_H_

#include <linux/completion.h>
#include <linux/if_ether.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>

// TODO:  Move this define into a header file shared with ns921x_devices.c
#define PIPER_DRIVER_NAME   "ns9xxx-piper"
#define DRV_VERS "0.1"

#define DEBUG
#ifdef DEBUG
#define digi_dbg(fmt, arg...) \
    printk(KERN_ERR PIPER_DRIVER_NAME ": %s - " fmt, __func__, ##arg)
#else
#define digi_dbg(fmt, arg...) \
	do { } while (0)
#endif


#define PIPER_RESET_GPIO    (92)
#define PIPER_IRQ_GPIO      (104)


struct piper_write_pkt {
    struct sk_buff *skb;
    int waitingForAck;
};

#define MAX_TX_QUEUE 128	/* XXX: tune later */

#define ERROR(x)    printk(KERN_ALERT x)

#define WORD32_LENGTH       (sizeof(unsigned int))

/* rf */
struct digi_rf_ops {
	const char *name;
	void (*init)(struct ieee80211_hw *, int);
	int (*stop)(struct ieee80211_hw *);
	int (*set_chan)(struct ieee80211_hw *, int chan);
	int (*set_pwr)(struct ieee80211_hw *, uint8_t val);
	void (*getOfdmBrs)(u64 brsBitMask, unsigned int *ofdm, unsigned int *psk);
	int channelChangeTime;

	struct ieee80211_supported_band *bands;
	uint8_t n_bands;
};



/**
 * piper_priv - 
 */
struct piper_priv {
    wait_queue_head_t rx_wait;
    struct task_struct *rx_thread;
    struct tasklet_struct rxTasklet;
    struct tasklet_struct txRetryTasklet;
    struct ieee80211_key_conf txKeyInfo;
    struct ieee80211_cts ctsFrame;
    struct ieee80211_rts rtsFrame;
    bool wantRts;                   /* set if we should send RTS on current TX frame*/
    bool wantCts;                   /* set if we should send CTS to self on current TX frame*/
    struct ieee80211_rate *rtsCtsRate;
    unsigned int rxOverruns;
    unsigned int irq;
    bool isRadioOn;
        
    void* __iomem vbase;
	struct sk_buff *read_skb;
	struct sk_buff *txPacket;
	unsigned int txMaxRetries;
	unsigned int txRetryIndex;
    spinlock_t irqMaskLock;
    spinlock_t rxReadyCountLock;
    
	struct ieee80211_hw *hw;
	const char *drv_name;

	int (*write_reg)(struct piper_priv *, uint8_t reg, uint32_t val);
	unsigned int (*read_reg)(struct piper_priv *, uint8_t reg);
	int (*write)(struct piper_priv *, uint8_t addr, uint8_t *buf, int len);
	int (*write_fifo)(struct piper_priv *, unsigned char *buffer, unsigned int length, unsigned int flags);
	int (*write_aes)(struct piper_priv *, unsigned char *buffer, unsigned int length, int keyidx, unsigned int flags);
	int (*write_aes_key)(struct piper_priv *, struct sk_buff *skb);
	int (*initHw)(struct piper_priv *);
	void (*setIrqMaskBit)(struct piper_priv *, unsigned int maskBits);
	void (*clearIrqMaskBit)(struct piper_priv *, unsigned int maskBits);

	uint16_t irq_mask;
	uint8_t bssid[ETH_ALEN];
	uint8_t ourMacAddress[ETH_ALEN];

	int channel;
	uint8_t tx_power;
	enum nl80211_iftype if_type;

	struct digi_rf_ops *rf;
};

/* main */
extern int piper_alloc_hw(struct piper_priv **priv, size_t priv_sz);
extern int piper_register_hw(struct piper_priv *priv, struct device *dev,
		struct digi_rf_ops *rf);
extern void piper_unregister_hw(struct piper_priv *priv);
extern void piper_free_hw(struct piper_priv *priv);

extern void piperTxRetryTaskletEntry (unsigned long context);

#endif
