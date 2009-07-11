/*
 * Header file for digiPs.c.  Declares functions used for power save mode.
 */

#ifndef digiPs_h

#define digiPS_h

#include "pipermain.h"

enum piper_ps_idle_command {
	PS_STOP_IDLE_TIMER,
	PS_START_IDLE_TIMER,
	PS_RESET_IDLE_TIMER
};

/*
 * Current version of mac80211 doesn't set power management bit in frame headers,
 * so I guess we have to for now.
 *
 * TODO:  See if we still have to do this in the next drop.
 */
#define piper_ps_set_header_flag(piperp, header) 	header->fc.pwrMgt = (piperp->ps.mode == PS_MODE_LOW_POWER)


void piper_ps_active(struct piper_priv *piperp);
void piper_ps_handle_beacon(struct piper_priv *piperp, struct sk_buff *skb);
void piper_ps_init(struct piper_priv *piperp);
void piper_ps_deinit(struct piper_priv *piperp);
void piper_ps_set(struct piper_priv *piperp, bool powerSaveOn);

#endif

