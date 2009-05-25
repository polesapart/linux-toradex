#ifndef DIGI_MAIN_H_
#define DIGI_MAIN_H_

#include <linux/completion.h>
#include <linux/if_ether.h>
#include <linux/spinlock.h>
#include <net/mac80211.h>
#include <linux/i2c.h>
#include <net/piper_pdata.h>

#define DRV_VERS "0.1"


/* #define DEBUG */
#ifdef DEBUG
#define digi_dbg(fmt, arg...) \
    printk(KERN_ERR PIPER_DRIVER_NAME ": %s - " fmt, __func__, ##arg)
#else
#define digi_dbg(fmt, arg...) \
	do { } while (0)
#endif

typedef u64 u48;


/*
 * Set this #define to receive frames in the ISR.  This may improve 
 * performance under heavy load at the expense of interrupt latency.
 */
#define WANT_TO_RECEIVE_FRAMES_IN_ISR   (0)

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
	void (*set_pwr_index)(struct ieee80211_hw *, unsigned int val);
	void (*getOfdmBrs)(u64 brsBitMask, unsigned int *ofdm, unsigned int *psk);
	enum ieee80211_band (*getBand)(int);
	int (*getFrequency)(int);
	const struct ieee80211_rate * (*getRate)(unsigned int);
	int channelChangeTime;
	s8 maxSignal;

	struct ieee80211_supported_band *bands;
	uint8_t n_bands;
};

/*
 * This structure holds the information we need to support
 * H/W AES encryption.
 */
#define EXPANDED_KEY_LENGTH     (176)           /* length of expanded AES key */
#define PIPER_MAX_KEYS      (4)
#define AES_BLOB_LENGTH     (48)                /* length of AES IV and headers */
struct piperKeyInfo
{
    bool valid;                 /* indicates if this record is valid */
    u8 addr[ETH_ALEN];          /* MAC address associated with key */
    u32 expandedKey[EXPANDED_KEY_LENGTH / sizeof(u32)];
    u48 txPn;                   /* packet number for transmit*/
    u48 rxPn;                   /* expected receive packet number */
};

/*
 * Useful defines for AES.
 */
#define	PIPER_EXTIV_SIZE	8		// IV and extended IV size
#define	MIC_SIZE			8		// Message integrity check size
#define ICV_SIZE            4       
#define	DATA_SIZE			28		// Data frame header+FCS size
#define	CCMP_SIZE			(PIPER_EXTIV_SIZE+MIC_SIZE)	// Total CCMP size

typedef enum 
{
    op_write,
    op_or,
    op_and
} piperRegisterWriteOperation_t;

typedef struct
{
    bool loaded;
    bool enabled;
    bool weSentLastOne;
} piperBeaconInfo_t;


enum digiWifiTxResult_t
{
    RECEIVED_ACK,
    TX_COMPLETE,
    OUT_OF_RETRIES
} ;



/**
 * piper_priv - 
 */
struct piper_priv {
    struct tasklet_struct rxTasklet;
    struct tasklet_struct txRetryTasklet;
    struct ieee80211_key_conf txKeyInfo;
    struct ieee80211_cts ctsFrame;
    struct ieee80211_rts rtsFrame;
    bool bssWantCtsProtection;      /* set if configured for CTS protection */
    unsigned int rxOverruns;
    unsigned int irq;
    bool isRadioOn;
    struct piperKeyInfo key[PIPER_MAX_KEYS];
    unsigned int AESKeyCount;
    piperBeaconInfo_t beacon;
    bool txRts;
    
    void* __iomem vbase;
	struct sk_buff *txPacket;
	unsigned int txTotalRetries;
	unsigned int txRetryCount[IEEE80211_TX_MAX_RATES];
	unsigned int txRetryIndex;
	bool useAesHwEncryption;
	unsigned int txAesKey;
	u32 txAesBlob[AES_BLOB_LENGTH / sizeof(u32)];   /* used to store AES IV and headers*/
    spinlock_t registerAccessLock;
    spinlock_t aesLock;
    
	struct ieee80211_hw *hw;
	struct piper_pdata *pdata;
	const char *drv_name;

	int (*write_reg)(struct piper_priv *, uint8_t reg, uint32_t val, piperRegisterWriteOperation_t op);
	unsigned int (*read_reg)(struct piper_priv *, uint8_t reg);
	int (*write)(struct piper_priv *, uint8_t addr, uint8_t *buf, int len);
	int (*read)(struct piper_priv *, uint8_t, uint8_t *, int);
	int (*initHw)(struct piper_priv *, enum ieee80211_band band);
	void (*setIrqMaskBit)(struct piper_priv *, unsigned int maskBits);
	void (*clearIrqMaskBit)(struct piper_priv *, unsigned int maskBits);
	u16 (*getNextBeaconBackoff)(void);
	int (*load_beacon)(struct piper_priv *, unsigned char *, unsigned int);
	int (*myrand)(void);
	void (*kickTransmitterTask)(void);

	uint8_t bssid[ETH_ALEN];
	int channel;
	int tx_power;
	enum nl80211_iftype if_type;
    struct ieee80211_tx_queue_stats txQueueStats;
    struct ieee80211_low_level_stats lowLevelStats;
	struct digi_rf_ops *rf;
	int txStartCount;
	int txCompleteCount;
	struct timer_list txTimer;
	struct i2c_client *adcI2cClient;
	u16 (*adcReadPeak)(struct piper_priv *digi);
	void (*adcClearPeak)(struct piper_priv *digi);
	u16 (*adcReadLastValue)(struct piper_priv *digi);
	void (*adcShutdown)(struct piper_priv *digi);
    void (*txTransmitFinished)(struct piper_priv *digi);
    struct ieee80211_rate *calibrationTxRate;
};

/* main */
extern int digiWifiAllocateHw(struct piper_priv **priv, size_t priv_sz);
extern int digiWifiRegisterHw(struct piper_priv *priv, struct device *dev,
		struct digi_rf_ops *rf);
extern void digiWifiUnregisterHw(struct piper_priv *priv);
extern void digiWifiFreeHw(struct piper_priv *priv);

extern void digiWifiTxRetryTaskletEntry (unsigned long context);
extern bool digiWifiPrepareAESDataBlob(struct piper_priv *digi, unsigned int keyIndex, 
                                    u8 *aesBlob, u8 *frame, u32 length, bool isTransmit);
extern void digiWifiSetRegisterAccessRoutines(struct piper_priv *digi);
extern void digiWifiDumpWordsAdd(unsigned int word);
extern void digiWifiDumpWordsDump(void);
extern void digiWifiDumpWordsReset(void);
extern void digiWifiTxRetryTaskletEntry (unsigned long context);
extern void digiWifiRxTaskletEntry (unsigned long context);
extern irqreturn_t digiWifiIsr(int irq, void *dev_id);
extern void digiWifiDumpRegisters(struct piper_priv *digi, unsigned int regs);
extern void digiWifiTxDone(struct piper_priv *digi, enum digiWifiTxResult_t result,
                            int signalStrength);


#endif
