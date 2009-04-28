/*
 * Linux device driver for Digi's WiWave WLAN card
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
#include <linux/string.h>
#include <linux/module.h>
#include <net/mac80211.h>

#define DEBUG
#include "pipermain.h"
#include "mac.h"
#include "phy.h"

#define WANT_DEBUG  (0)

#if WANT_DEBUG
#define debugPrintf(fmt, arg...) \
    printk(KERN_ERR PIPER_DRIVER_NAME ": %s - " fmt, __func__, ##arg)
#else
#define debugPrintf(fmt, arg...) \
	do { } while (0)
#endif

#define NUMBER_OF_WORD32(x)     ((x + 3) >> 2)

static int is_ofdm_rate(int rate)
{
	return (rate % 3) == 0;
}

void phy_set_plcp(unsigned char *frame, unsigned length, struct ieee80211_rate *rate, int aes_len)
{
	int ofdm = is_ofdm_rate(rate->bitrate);
	int plcp_len = length + FCS_LEN + aes_len;
	struct tx_frame_hdr *hdr;
    
	if (ofdm) {
		/* OFDM header */
		struct ofdm_hdr *ofdm;

		ofdm = (struct ofdm_hdr *) &frame[sizeof(struct tx_frame_hdr)];
		memset(ofdm, 0, sizeof(*ofdm));
		ofdm->rate = rate->hw_value;
		ofdm->length = cpu_to_le16(plcp_len);
        debugPrintf("ofdm->rate=%x (@%dM), ofdm->length=%d\n", ofdm->rate, rate->bitrate/10, ofdm->length);

	} else {
		/* PSK/CCK header */
		struct psk_cck_hdr *pskcck;
		int us_len;

		pskcck = (struct psk_cck_hdr *) &frame[sizeof(struct tx_frame_hdr)];
		pskcck->signal = rate->bitrate;
		pskcck->service = PLCP_SERVICE_LOCKED;

		/* convert length from bytes to usecs */
		switch (rate->bitrate) {
		case 10:
			us_len = plcp_len * 8;
			break;
		case 20:
			us_len = plcp_len * 4;
			break;
		case 55:
			us_len = (16 * plcp_len + 10) / 11;
			break;
		case 110:
			us_len = (8 * plcp_len + 10) / 11;
			
			/* set length extension bit if needed */
			debugPrintf("us_len = %d, plcp_len = %d, (11 * us_len) = %d, (11 * us_len) / 8 = %d\n", us_len, plcp_len, (11 * us_len), (11 * us_len) / 8);
			
			if ((11 * us_len) / 8 > plcp_len)
			{
				pskcck->service |= PLCP_SERVICE_LENEXT;
			    debugPrintf("Set PLCP_SERVICE_LENEXT, pskcck->service = 0x%4.4X\n", pskcck->service);
			} else  debugPrintf("Did not set PLCP_SERVICE_LENEXT, pskcck->service = 0x%4.4X\n", pskcck->service);

			break;
		default:
			WARN_ON(1);
			us_len = 0;
		}
		
		pskcck->length = cpu_to_le16(us_len);
    debugPrintf("pskcck .length = %d, signal = %d, service = %d\n", pskcck->length, pskcck->signal, pskcck->service);
debugPrintf("rate->bitrate=%x (@%dM), pckcck->length=%d\n", rate->bitrate, rate->bitrate/10, pskcck->length);
	}
		
	hdr = (struct tx_frame_hdr *) frame;
	hdr->pad = 0;
	hdr->length = NUMBER_OF_WORD32((length + aes_len + TX_HEADER_LENGTH));
	hdr->modulation_type = ofdm ? MOD_TYPE_OFDM : MOD_TYPE_PSKCCK;
    debugPrintf("tx_frame_hdr .length = %d, modulation_type = %d\n", hdr->length, hdr->modulation_type);
	debugPrintf("TX: %d byte %s packet @ %dmbit\n", length,
			ofdm ? "OFDM" : "PSK/CCK", rate->bitrate/10);
}
EXPORT_SYMBOL_GPL(phy_set_plcp);

static int get_signal(struct rx_frame_hdr *hdr)
{
	int gain;
	int signal;
	
	/* map LNA value to gain value */
	const unsigned char lna_table[] =
	{
		0, 0, 23, 39
	};
	/* map high gain values to dBm */
	const char gain_table[] =
	{
		-82, -84, -85, -86, -87, -89, -90, -92, -94, -98
	};

	gain = lna_table[hdr->rssi_low_noise_amp] +
			2 * hdr->rssi_variable_gain_attenuator;
	if (gain > 96)
		signal = -98;
	else if (gain > 86)
		signal = gain_table[gain - 87];
	else
		signal = 5 - gain;

	return signal;
}

void phy_process_plcp(struct piper_priv *piper, struct rx_frame_hdr *hdr,
		struct ieee80211_rx_status *status, unsigned int *length)
{
	unsigned rate, i;
    struct digi_rf_ops *rf = piper->rf;
    memset(status, 0, sizeof(*status));
	status->signal = get_signal(hdr);
	status->antenna = hdr->antenna;
	status->band = IEEE80211_BAND_2GHZ;	/* hw only supports 2G */
    status->freq = piper->rf->bands[IEEE80211_BAND_2GHZ].channels[piper->channel - 1].center_freq;

	if (hdr->modulation_type == MOD_TYPE_OFDM) 
	{
		/* OFDM */
		struct ofdm_hdr *ofdm = &hdr->mod.ofdm;
		const int ofdm_rate[] = 
		{
			480, 240, 120, 60, 540, 360, 180, 90
		};


		rate = ofdm_rate[ofdm->rate & 0x7];
		*length = le16_to_cpu(ofdm->length);
		debugPrintf("%d byte OFDM packet @ %dmbit\n", *length, rate/10);
	} 
	else 
	{
		/* PSK/CCK */
		struct psk_cck_hdr *pskcck = &hdr->mod.psk;

		rate = pskcck->signal;

		*length = le16_to_cpu(pskcck->length);
		switch (rate) {
		case 10:
			*length /= 8;
			break;
		case 20:
			*length /= 4;
			break;
		case 55:
			*length = (11 * (*length)) / 16;
			break;
		case 110:
			*length = (11 * (*length)) / 8;
			if (pskcck->service & PLCP_SERVICE_LENEXT)
				(*length)--;
			break;
		default:
			WARN_ON(1);
			*length = 0;
		}
#if WANT_DEBUG
		debugPrintf("%d byte PSK/CCK packet @ %dmbit\n", *length, rate/10);
#endif
	}

	/* match rate with the list of bitrates that we supplied the stack */
	for (i = 0; i < rf->bands[status->band].n_bitrates; i++) 
	{
		if (rf->bands[status->band].bitrates[i].bitrate == rate)
			break;
	}

	if (i != rf->bands[status->band].n_bitrates)
		status->rate_idx = i;
	else 
	{
		printk(KERN_ERR PIPER_DRIVER_NAME
				": couldn't find bitrate index for %d?\n",
				rate);
		status->flag |= RX_FLAG_FAILED_PLCP_CRC;
		return;
	}


}
EXPORT_SYMBOL_GPL(phy_process_plcp);
