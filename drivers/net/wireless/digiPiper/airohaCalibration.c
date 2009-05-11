/*
 * This file contains the code which performs automatic recalibration of the
 * Airoha transceiver.
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define MAX_SAMPLES                     (3)

/*
 * Events we will wait for, also return values for waitForEvent().
 */
#define TIMED_OUT_EVENT                 (1 << 0)
#define SAMPLES_DONE_EVENT              (1 << 1)
#define SHUTDOWN_AUTOCALIBRATION_EVENT  (1 << 2)
#define RESTART_AUTOCALIBRATION_EVENT   (1 << 3)

/*
 * States for the auto calibration thread.
 */
enum
{
    COLLECT_SAMPLES_STATE,
    RECALIBRATE_STATE
};


static DECLARE_WAIT_QUEUE_HEAD(waitQueue)

typedef struct
{
    unsigned rate;              /* rate packet transmitted at */
    unsigned int sample;        /* measured sample */
} sampleInfo_t;

typedef struct
{
    struct task_struct *threadCB;
    spinlock_t lock;
    volatile unsigned int events;
    unsigned int sampleCount;
    sampleInfo_t sample[MAX_SAMPLES];
    wcd_data_t nvram;
} airohaCalibrationData_t;

static airohaCalibrationData_t calibration;



static void transmitIsStarting(struct piper_priv *digi)
{
    adcClearPeak(digi);
}

static void transmitHasCompleted(struct piper_priv *digi)
{
    if (calibration.sampleCount < MAX_SAMPLES)
    {
        calibration.sample[calibration.sampleCount].rate = adcReadPeak(digi);
        calibration.sampleCount++;
        if (calibration.sampleCount == MAX_SAMPLES)
        {
            calibration.event |= SAMPLES_DONE_EVENT;
            wake_up_interruptible(&waitQueue);
        }
    }
}


static void startSampleCollection(struct piper_priv *digi)
{
    calibration.sampleCount = 0;
    /* set a rate mask and set a flag to tell transmit routine to collect samples */
}

static void recalibrate(struct piper_priv *digi)
{
    /* run the magic recalibration routine */
}



static waitForEvent(unsigned int timeout, unsigned int eventToWaitFor)
{
#define ALL_EVENTS_TO_WAIT_FOR(x)   (eventToWaitFor \
                                     | SHUTDOWN_AUTOCALIBRATION_EVENT \
	                                 | RESTART_AUTOCALIBRATION_EVENT)
    unsigned long spinlockFlags;
	int ccode;
	unsigned int event;
	int result = TIMED_OUT_EVENT;
	unsigned int fullEventToWaitFor = 
	                                    
    ccode = wait_event_interruptible_timeout(&waitQueue, 
                                             ALL_EVENTS_TO_WAIT_FOR(eventToWaitFor),
                                             RECALIBRATION_PERIOD);
                                             
    spin_lock_irqsave(&calibration.lock, spinlockFlags);
    event = calibration.events;
    calibration.events = 0;
    spin_unlock_irqrestore(&calibration.lock, spinlockFlags);

    digi_dbg("ccode = %d, event = 0x%8.8X\n", ccode, event);
    
    if ((ccode < 0) || (event & SHUTDOWN_AUTOCALIBRATION_EVENT))
    {
        result = SHUTDOWN_AUTOCALIBRATION;
    }
    else if (event & RESTART_AUTOCALIBRATION_EVENT)
    {
        result = RESTART_AUTOCALIBRATION_EVENT;
    }
    else if (event & eventToWaitFor)
    {
        result = (event & eventToWaitFor);
    }
        
    return result;
}


static int digiWifiCalibrationThreadEntry(void *data)
{
	struct piper_priv *digi = data;
    
    while (1)
    {
        /*
         * We, the wireless driver, may be loaded before the I2C core has
         * loaded.  Therefore we may not be able to load our ADC driver,
         * which is an I2C client driver, when we load.  This loop tries
         * and retries to load the ADC driver until it succeeds.
         */
        ssleep(10);
        if (digiWifiInitAdc(digi) == 0)
        {
            break;
        }
    }

    calibrate.state = COLLECT_SAMPLES_STATE;
    
    while (1)
    {
        switch (calibration.state)
        {
            case COLLECT_SAMPLES_STATE:
                startSampleCollection(digi);
                calibration.state = RECALIBRATE_STATE;
                ccode = waitForEvent(SAMPLE_TIMEOUT, SAMPLES_DONE_EVENT);
                break;
            case RECALIBRATE_STATE:
                recalibrate(digi);
                calibration.state = COLLECT_SAMPLES_STATE;
                ccode = waitForEvent(IDLE_TIMEOUT, NO_EVENT);
                break;
            default:
                digi_dbg("Unknown state %d\n", calibration.state);
                calibration.state = COLLECT_SAMPLES_STATE;
                ccode = waitForEvent(IDLE_TIMEOUT, NO_EVENT);
                break;
        }

            
        if (ccode == SHUTDOWN_AUTOCALIBRATION_EVENT)
        {
            break;
        }
        else if (ccode == RESTART_AUTOCALIBRATION_EVENT)
        {
            calibration.state = COLLECT_SAMPLE_STATE;
            break;
        }
    }
    
    return 0;
}

/*
 * This routine is called at initialization to set up the Airoha calibration routines.
 */
void digiWifiInitCalibration(struct piper_priv *digi)
{
	
    calibration.events = 0;
    calibration.sampleCount = 0;
    
    spin_lock_init(&calibration.lock);
    
    calibration.threadCB = kthread_run(digiWifiCalibrationThreadEntry, digi, PIPER_DRIVER_NAME " - calibration");

    return (calibration.threadCB != NULL);
}

EXPORT_SYMBOL_GPL(digiWifiInitCalibration);

