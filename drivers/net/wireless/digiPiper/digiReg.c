/*
 * digiReg.c
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This file contains the routines that read and write registers and FIFOs
 * on piper, chopper, and the FPGA.  Register access goes through these 
 * routines so that we have some hope of sharing code with WiWave.  The
 * WiWave registers and FIFOs have to be accessed across a USB interface,
 * and so their use is quite complicated.  The intent is the create a
 * standard API for accessing registers and FIFOs that can be shared with
 * all the different H/W implementations and so hide the hardware from the
 * rest of the software.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>

#include "pipermain.h"
#include "mac.h"


/*
 * This routine writes data into a FIFO.  For best performance, there are
 * 3 different implementations of this routine below.  The if statements
 * select the implementation depending upon if the data is aligned and if
 * we are writing multiple words of data.
 *
 * Arguments:
 *      digi        pointer to context information
 *      address     set to BB_AES_FIFO, BB_FIFO, or BEACON_FIFO.  Setting to
 *                  BEACON_FIFO indicates that we are loading a beacon.
 *      buf         pointer to buffer to write into the FIFO.  The buffer
 *                  is not necessarily aligned.
 *      len         number of bytes in the buffer.
 *
 * Return Values
 *      0           success
 *                  (nonzero return value would be possible with USB version)
 */
static int writeFifo(struct piper_priv *digi, uint8_t addr, uint8_t *buf,
		int len)
{
/*
 * This macro waits for the AES busy bit to clear if we are writing to the
 * AES FIFO.
 */
#define WAIT_FOR_AES        while (    (addr == BB_AES_FIFO)        \
                                    && ((ioread32(digi->vbase + BB_RSSI) & BB_RSSI_EAS_FIFO_FULL) != 0)) {udelay(1);}
                                    
    int wordIndex;
    int wordLength = len / sizeof(unsigned int);
    unsigned long flags;
    bool loadingBeacon = (addr == BEACON_FIFO);
    
    spin_lock_irqsave(&digi->registerAccessLock, flags);

    if (loadingBeacon)
    {
        /*
         * If we are loading a new beacon, then adjust the address to point
         * to the data FIFO, and set the beacon enable bit which tells piper
         * to put this data into the beacon buffer.
         */
        addr = BB_DATA_FIFO;
        iowrite32(ioread32(digi->vbase + BB_GENERAL_CTL) | BB_GENERAL_CTL_BEACON_EN,
                  digi->vbase + BB_GENERAL_CTL);
    }
    
    if (((unsigned) (buf) & 0x3) == 0)
    {
        /*
         * We come here if the data is 32-bit aligned.  We can dispense 
         * with memcpys
         */
        if (wordLength == 1)
        {
            /*
             * Only 1 word of data, so just one write.
             */
            unsigned int *word = (unsigned int *) buf;
            
            WAIT_FOR_AES;
            iowrite32(cpu_to_be32(*word), digi->vbase + addr);
            len -= 4;
        }
        else
        {
            /*
             * More than one word of data, so set up a for loop.
             */
            for (wordIndex = 0; wordIndex < wordLength; wordIndex++)
            {
                unsigned int *word = (unsigned int *) buf;
                
                WAIT_FOR_AES;
                iowrite32(cpu_to_be32(word[wordIndex]), digi->vbase + addr);
                len -= 4;
            }
        }
    }
    else
    {
        /*
         * Ugh!  Data is not 32-bit aligned.  We have to memcpy it!
         */
        for (wordIndex = 0; wordIndex < wordLength; wordIndex++)
        {
            unsigned int word;
            
            memcpy(&word, &buf[wordIndex*sizeof(unsigned int)], sizeof(unsigned int));
            
            WAIT_FOR_AES;
            iowrite32(cpu_to_be32(word), digi->vbase + addr);
            len -= 4;
        }
    }
  
    if (len)
    {
        /*
         * Double Ugh!  There was left over data at the end.  We have to write
         * the leftover data into the upper bytes of the last word, making 
         * sure the unused bytes are set to zero.
         */
        unsigned int word;
        
        memcpy(&word, &buf[wordLength*sizeof(unsigned int)], sizeof(unsigned int));
        word = cpu_to_be32(word);
        switch (len)
        {
            case 1:
                word &= 0xff000000;
                break;
            case 2:
                word &= 0xffff0000;
                break;
            case 3:
                word &= 0xffffff00;
                break;
            default:
                digi_dbg("len = %d at end of piper_write\n", len);
                break;
        }
        WAIT_FOR_AES;
        iowrite32(word, digi->vbase + addr);
    }

    if (loadingBeacon)
    {
        /*
         * If we just loaded a beacon, then don't forget to turn off the 
         * load beacon bit.
         */
        iowrite32(ioread32(digi->vbase + BB_GENERAL_CTL) & ~BB_GENERAL_CTL_BEACON_EN,
                  digi->vbase + BB_GENERAL_CTL);
        digi->beacon.loaded = true;
    }

    spin_unlock_irqrestore(&digi->registerAccessLock, flags);
    
    return 0;
}


/*
 * Write a value to a Piper register.  This function takes an operations
 * parameter that allows you to specify a simple write, a logical-and operation, or
 * a logical-or operation.  There are two reasons for this:
 *
 * 1.  We want to maintain some compatibility with the USB driven 
 *     WiWave which actually uses these operations.
 * 2.  If we do the logical operations seperately, for example read a register,
 *     or the value with the result, and then write, we run the risk of
 *     some other thread interrupting us between the read and the write.
 *     By doing both in this function we can protect the operation with
 *     a spinlock.
 *
 *  Arguments
 *      digi            context information
 *      reg             address of register to write
 *      val             value to write
 *      op              logical operation
 *
 *  Return Values
 *      0               success
 */
static int writeReg(struct piper_priv *digi, uint8_t reg, uint32_t val, 
                            piperRegisterWriteOperation_t op)
{    
    unsigned long flags;
    
    spin_lock_irqsave(&digi->registerAccessLock, flags);

    switch (op)
    {
        case op_write:
            iowrite32(val, digi->vbase + reg);
            break;
        case op_or:
            iowrite32(val | ioread32(digi->vbase + reg), digi->vbase + reg);
            break;
        case op_and:
            iowrite32(val & ioread32(digi->vbase + reg), digi->vbase + reg);
            break;
        default:
            digi_dbg("Invalid operation %d passed to piper_write_reg\n", op);
            WARN_ON(1);
            break;
    }

    spin_unlock_irqrestore(&digi->registerAccessLock, flags);

	return 0;
}




/*
 * This routine reads data from a FIFO.  For best performance, there are
 * 3 different implementations of this routine below.  The if statements
 * select the implementation depending upon if the data is aligned and if
 * we are writing multiple words of data.
 *
 * Arguments:
 *      digi        pointer to context information
 *      address     set to BB_AES_FIFO, or BB_FIFO.  
 *      buf         pointer to buffer to write data from the FIFO into.
 *                  We do not assume the buffer is aligned
 *      len         number of bytes to read.
 *
 * Return Values
 *      0           success
 *                  (nonzero return value would be possible with USB version)
 */
static int readFifo(struct piper_priv *digi, uint8_t addr, uint8_t *buf,
		int len)
{
/*
 * This routine waits for the empty flag to clear when we are reading from
 * the AES FIFO.
 */
#define WAIT_FOR_AES_NOT_EMPTY        while (    (addr == BB_AES_FIFO)        \
                                    && ((ioread32(digi->vbase + BB_RSSI) & BB_RSSI_EAS_FIFO_EMPTY) != 0)) \
                                    { \
                                        udelay(1); \
                                        if (--timeout == 0) \
                                        { \
                                            digiWifiDumpRegisters(digi, MAIN_REGS); \
                                            timeout = 10000;\
                                        }\
                                    }
    int wordIndex;
    unsigned long flags;
    int timeout = 10000;
        
    spin_lock_irqsave(&digi->registerAccessLock, flags);
    
/*
 * We can only read 32-bit words, so round the length up to an even multiple of
 * 4 if necessary.
 */
    len += 3;
    len &= 0xfffffc;
    
    if ((len == 4) && ((((unsigned)buf) & 0x3) == 0))
    {
        unsigned *word = (unsigned *)buf;
        
        WAIT_FOR_AES_NOT_EMPTY;
        *word = be32_to_cpu(ioread32(digi->vbase + addr));
    }
    else if ((((unsigned)buf) & 0x3) == 0)
    {
        unsigned *word = (unsigned *)buf;
        
        for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++)
        {
            WAIT_FOR_AES_NOT_EMPTY;
            word[wordIndex] = be32_to_cpu(ioread32(digi->vbase + addr));
        }
    }
    else
    {
        /*
         * If we come here, then the buffer is not aligned and we have to
         * memcpy the data.
         */
        for (wordIndex = 0; wordIndex < (len / sizeof(unsigned)); wordIndex++)
        {
            unsigned word;
            
            WAIT_FOR_AES_NOT_EMPTY;
            word = be32_to_cpu(ioread32(digi->vbase + addr));
            memcpy(&buf[wordIndex * sizeof(unsigned)], &word, sizeof(word));
        }
    }
    spin_unlock_irqrestore(&digi->registerAccessLock, flags);
    
    return 0;
}



/*
 * Read a value from a Piper register.  
 *
 *  Arguments
 *      digi            context information
 *      reg             address of register to read
 *
 *  Return Values
 *      register value
 */
static unsigned int readReg(struct piper_priv *digi, uint8_t reg)
{
    unsigned long flags;
    unsigned int result;
    
    spin_lock_irqsave(&digi->registerAccessLock, flags);

    result = ioread32(digi->vbase + reg);

    spin_unlock_irqrestore(&digi->registerAccessLock, flags);

	return result;
}


/*
 * This routine is called during initialization to set the
 * register and fifo access routines.
 */
void digiWifiSetRegisterAccessRoutines(struct piper_priv *digi)
{
	digi->write_reg = writeReg;
	digi->read_reg = readReg;
	digi->write = writeFifo;
	digi->read = readFifo;
}

EXPORT_SYMBOL_GPL(digiWifiSetRegisterAccessRoutines);
