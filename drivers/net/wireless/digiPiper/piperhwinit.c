/***********************************************************************
 *
 * Copyright (C) 2006 by FS Forth-Systeme GmbH.
 * All rights reserved.
 *
 * $Id: fpga.c,v 1.17 2007-05-25 08:26:23 mpietrek Exp $
 * @Author: Bernd Westermann
 * @Descr: Defines helper functions for loading fpga firmware
 * @Usage: 
 * @References: [1]
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *  
 ***********************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <net/mac80211.h>
#include <linux/usb.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>

#include "pipermain.h"
#include "mac.h"
#include "piperDsp.h"
#include "piperMacAssist.h"

#include <asm/gpio.h>
#include <asm/io.h>
#include <mach/regs-mem.h>

#ifndef FALSE
    #define FALSE   (0)
#endif
#ifndef TRUE
    #define TRUE    (1)
#endif

#define MAC_BASE		0x70000000		/* Register base address */
#define MAC_CTRL_BASE   (volatile uint32 *) (MAC_BASE + (0x40))
#define MAC_MASK		0xffffc001		/* Size mask and enable bit */

#define LOAD_FPGA_OK    0
#define LOAD_FPGA_FAIL  -1

/*
 * Program CS3 to map Piper registers to address 0x70000000 in memory.
 */
static void setupChipSelect(void)
{
#define SYS_MEM_CS3_BASE    __REG(0xA0900208)
#define SYS_MEM_CS3_MASK    __REG(0xA090020C)

    /*
     * Configure the memory controller with the appropriate settings for the Piper
     * chip.  
     */
#define OFFSET_OF_MREG(x)    (x - MEM_SMC(3))
    iowrite32(0x82, MEM_SMC(3));
    iowrite32(0x0, MEM_SMWED(3));
    iowrite32(0x2, MEM_SMOED(3));
    iowrite32(0x8, MEM_SMRD(3));
    iowrite32(0x0, MEM_SMPMRD(3));
    iowrite32(0x4, MEM_SMWD(3));
    iowrite32(0x2, MEM_SWT(3));

/*
 * Configure CS base and mask.
 */
    iowrite32(MAC_BASE, SYS_MEM_CS3_BASE);
    iowrite32(MAC_MASK, SYS_MEM_CS3_MASK);

}


/*
 * Load the MAC Assist firmware into the chip.  This is done by setting a bit
 * in the control register to enable MAC Assist firmware download, and then
 * writing the firmware into the data FIFO.
 */
static void loadMacAssistFirmware(struct piper_priv *piper)
{
    unsigned int word;
    unsigned int offset;
    
    digi_dbg("piper_macassist_data_len = %d.\n", piper_macassist_data_len);
    
    // Zero out MAC assist SRAM (put into known state before enabling MAC assist)
    for (offset = 0; offset < 0x100; offset += 4)
    {
        iowrite32(0, piper->vbase + offset);
    }
    // Enable download the MAC Assist program RAM
    iowrite32(ioread32(piper->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_FW_LOAD_ENABLE,
                piper->vbase + BB_GENERAL_CTL);

    //load MAC Assist data
    for (word = 0; word < piper_macassist_data_len; word++)
    {
        iowrite32(piper_wifi_macassist_ucode[word], piper->vbase + BB_DATA_FIFO);
    }
    
    // disable MAC Assist download
    iowrite32(ioread32(piper->vbase + BB_GENERAL_CTL) & ~BB_GENERAL_CTL_FW_LOAD_ENABLE,
                piper->vbase + BB_GENERAL_CTL);
}


/*
 * Load the DSP firmware into the chip.  This is done by setting a bit
 * in the control register to enable DSP firmware download, and then
 * writing the firmware into the data FIFO.
 */
static void loadDspFirmware(struct piper_priv *piper)
{
    int i;
    
    /*
     * Enable load of DSP firmware.
     */
    iowrite32(ioread32(piper->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_DSP_LOAD_ENABLE,
                piper->vbase + BB_GENERAL_CTL);
    
    //load DSP data
    for (i = 0; i < piper_dsp_data_len; i++)
    {
        iowrite32(piper_wifi_dsp_ucode[i], piper->vbase + BB_DATA_FIFO);
    }
    
    /*
     * Disable load of DSP firmware.
     */
    udelay(10); 
    iowrite32(ioread32(piper->vbase + BB_GENERAL_CTL) & ~BB_GENERAL_CTL_DSP_LOAD_ENABLE,
                piper->vbase + BB_GENERAL_CTL);
    
    /*
     * Let her rip.
     */
    iowrite32(ioread32(piper->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_MAC_ASSIST_ENABLE,
                piper->vbase + BB_GENERAL_CTL);
}


/*
 * This routine corrects a bug in the Piper chip where internal clocks would
 * be out of sync with each other and cause the chip to generate noise spikes.
 * This problem should be fixed in the next chip (Chopper).
 *
 * I'm not sure exactly what this code is doing.  It comes straight from the
 * guy who designed the chip.
 */
static int spikeSuppression(struct piper_priv *piper)
{
    int timeout1 = 300, timeout2 = 300;
    int result = LOAD_FPGA_OK;

    // Initial timing measurement to avoid spike
     /* The new "magic" value is 0x63 at address 0xA62.  Bit-0 indicates the
      timing measurement is complete.  Bit-1 indicates that a second timing 
      measurment was performed.  The upper nibble is the timing measurement 
      value. This code should eliminate the possibility of spikes at the 
      beginning of all PSK/CCK frames and eliminate the spikes at the end of 
      all PSK (1M, 2M) frames.
      */
                
    // reset the timing value WrPortI(0xA62, NULL, 0x00); 
    iowrite32(ioread32(piper->vbase + MAC_STATUS) & 0xffff00ff,
                piper->vbase + MAC_STATUS);
                    
    while ( (ioread32(piper->vbase + MAC_STATUS) & 0x0000ff00) != 0x00006300) //while (RdPortI(0xA62) != 0x63)
    {          
        // reset the timing value 
        //WrPortI(0xA62, NULL, 0x00);    
        iowrite32(ioread32(piper->vbase + MAC_STATUS) & 0xffff00ff,
                    piper->vbase + MAC_STATUS);

        // issue WiFi soft reset  
        iowrite32(0x40000000, piper->vbase + BB_GENERAL_STAT); //_wc_write32(0xA08, 0x40000000); 

                // Set TX_ON Low WrPortI(0xA3C, NULL, ((RdPortI(0xA3C) & ~0xC0) | 0x80));
        iowrite32(ioread32(piper->vbase + BB_OUTPUT_CONTROL) & 0xffffff3f,
                    piper->vbase + BB_OUTPUT_CONTROL);
        iowrite32(ioread32(piper->vbase + BB_OUTPUT_CONTROL) | 0x00000080,
                    piper->vbase + BB_OUTPUT_CONTROL);
    
        // Set PA_2G Low WrPortI(0xA3D, NULL, ((RdPortI(0xA3D) & ~0x0C) | 0x08)); 
        iowrite32(ioread32(piper->vbase + BB_OUTPUT_CONTROL) & 0xfffff0ff,
                    piper->vbase + BB_OUTPUT_CONTROL);
        iowrite32(ioread32(piper->vbase + BB_OUTPUT_CONTROL) | 0x00000a00,
                    piper->vbase + BB_OUTPUT_CONTROL);
           
        // Set RX_ON low  WrPortI(0xA3F, NULL, ((RdPortI(0xA3F) & ~0x30) | 0x20)); 
        iowrite32(ioread32(piper->vbase + BB_OUTPUT_CONTROL) & 0xcfffffff,
                    piper->vbase + BB_OUTPUT_CONTROL);
        iowrite32(ioread32(piper->vbase + BB_OUTPUT_CONTROL) | 0x20000000,
                    piper->vbase + BB_OUTPUT_CONTROL);
           
        // start the WiFi mac & dsp
        iowrite32(0x37720820, piper->vbase + BB_GENERAL_CTL);

        timeout1= 500;

        // Wait for timing measurement to finish
        while ( (ioread32(piper->vbase + MAC_STATUS) & 0x0000ff00) != 0x00000100) //((RdPortI(0xA62) & 0x01) != 0x01)
        {
           udelay(2);
           timeout1--;
           if (!timeout1)
               break;
        }

        timeout2--;
        if(!timeout2)
            result = LOAD_FPGA_FAIL;
    }   

    // Set TX_ON/RXHP_ON and RX to normal wifi, restore the reset value to HW_OUT_CTRL
    iowrite32(0x1, piper->vbase + BB_OUTPUT_CONTROL);
    // ---- end of spike detection
    
    return result;
}
    

static void resetMac(struct piper_priv *piper)
{
    int i;
    
    // set the TX-hold bit
    iowrite32(0x37720080, piper->vbase + BB_GENERAL_CTL);
    
    // clear the TX-FIFO memory
    for (i=0; i<448; i++)
        iowrite32(0, piper->vbase + BB_DATA_FIFO);

    // reset the TX-FIFO
    iowrite32(0x377200C0, piper->vbase + BB_GENERAL_CTL);
    
    // release the TX-hold and reset 	
    iowrite32(0x37720000, piper->vbase + BB_GENERAL_CTL);

    iowrite32(ioread32(piper->vbase + MAC_STATUS) & ~0x40000000, piper->vbase + BB_GENERAL_STAT);
    mdelay(1);
}


/*
 * Load the MAC address into the chip.  Use the value stored in the
 * environment, if there is one, otherwise use the default value.
 */
static void setMacAddress(struct piper_priv *piper)
{
    unsigned char macAddress[6] = {0x00, 0x04, 0xf3, 0x00, 0x43, 0x35};
    
    if (((piper->pdata->macaddr[0] & 1) == 0) 
        && ((piper->pdata->macaddr[0] | piper->pdata->macaddr[1] | piper->pdata->macaddr[2]
             | piper->pdata->macaddr[3] | piper->pdata->macaddr[4] | piper->pdata->macaddr[5])
             != 0))
    {
        memcpy (macAddress, piper->pdata->macaddr, sizeof(macAddress));
    }
    else
    {
        digi_dbg("Invalid MAC address set in NVRAM.  Using default MAC address.\n");
    }
    memcpy(piper->hw->wiphy->perm_addr, macAddress, sizeof(piper->hw->wiphy->perm_addr));

        /* configure ethernet address */
    iowrite32(macAddress[ 3 ] | macAddress[ 2 ] << 8 | macAddress[ 1 ] << 16 | macAddress[ 0 ] << 24,
                piper->vbase + MAC_STA_ID0);
    iowrite32(macAddress[ 5 ] << 16 | macAddress[ 4 ] << 24,
                piper->vbase + MAC_STA_ID1);
#if 0
    digi_dbg("MAC address set to %2.2X:%2.2X:%2.2X:%2.2X:%2.2X:%2.2X\n", macAddress[0],
             macAddress[1], macAddress[2], macAddress[3], 
             macAddress[4], macAddress[5]);
#endif
}

/*
 * Set GPIO pin 92 high to take Piper out of reset on the ccw9p9215.
 */
static void clearReset(void)
{
/* TODO use the platform information */      
#define PIPER_RESET_GPIO	92

    /*
     * Set the GPIO pin high to take Piper out of reset.
     */
    gpio_direction_output(PIPER_RESET_GPIO, 1);
    mdelay(1);
}






/*
 * Program the CS to make Piper registers visible and then download
 * the DSP and MAC Assist firmware to the chip.
 */
int piper_load_firmware(struct piper_priv *piper) 
{
    int result = 0;
    
    static int loaded = FALSE;

    // Load only once after reset
    if (loaded)
        return LOAD_FPGA_OK;

    printk(KERN_INFO "Loading Piper MAC Assist firmware\n");
    
    setupChipSelect();
    clearReset();
    
    loadMacAssistFirmware(piper);
    loadDspFirmware(piper); 
    result = spikeSuppression(piper);
    
    resetMac(piper);
    setMacAddress(piper);
    
    return result;
}
    

