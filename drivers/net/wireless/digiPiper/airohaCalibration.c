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
 * Field values used for computing ABS values.
 */
enum 
{
    OUT_POWER,
    ADC_VAL,
    POWER_INDEX
};


/*
 * States for the auto calibration thread.
 */
enum
{
    RESTART_STATE,
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


/*
 * This routine is called to shut down the transmit ADC sampler.
 */
static void stopSampler(struct piper_priv *digi)
{
    digi->txTransmitStarted = NULL;
    digi->txTransmitFinished = NULL;
}

/*
 * This routine is called to update the state of the calibration state machine
 * and wake up its thread.
 */
static void kickCalibrationThread(struct piper_priv *digi, unsigned int event)
{
    spin_lock_irqsave(&calibration.lock, spinlockFlags);
    calibration.event |= event;
    spin_unlock_irqrestore(&calibration.lock, spinlockFlags);
    wake_up_interruptible(&waitQueue);
}

    
/*
 * This routine is called each time we complete a transmit while we are in
 * the sampling state.  We record the peak ADC reading.  We kick the state
 * machine if we now have all the samples we need.
 */
static void transmitHasCompleted(struct piper_priv *digi)
{
    if (calibration.sampleCount < MAX_SAMPLES)
    {
        calibration.sample[calibration.sampleCount].rate = adcReadPeak(digi);
        calibration.sampleCount++;
        if (calibration.sampleCount == MAX_SAMPLES)
        {
            stopSampler(digi);
            kickCalibrationThread(digi, SAMPLES_DONE_EVENT);
        }
    }
}


static void startSampleCollection(struct piper_priv *digi)
{
    calibration.sampleCount = 0;
    digi->txTransmitStarted = transmitIsStarting;
    digi->txTransmitStop = transmitHasCompleted;
    /* TODO: set a rate mask and set a flag to tell transmit routine to collect samples */
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


/*
 * Determine the ABS of a field in a point and a value.
 */
static unsigned fieldAbs(wcd_point_t *p1, int value, int field)
{
    int difference;
    
    switch (field)
    {
        case OUT_POWER:
            difference = value - p1->out_power;
            break;
        case ADC_VAL:
            difference = value - p1->adc_val;
            break;
        case POWER_INDEX:
            difference = value - p1->power_index;
            break;
        default:
            digi_dbg("Unknown field type %d.\n", field);
            difference = 0;
    }
    
    result = abs(difference);
}


/*
 * This routine finds the closest pair of points in a calibration curve.
 *
 *      curve       calibration curve 
 *      value       look for the pr of points closest to this value
 *      p1          storage for one point
 *      p2          storage for another point
 *      field       tells us which field in the point struct to compare
 */
static findClosestPoints(wcd_curve_t *curve, int value, wcd_point_t *p1, wcd_point_t *p2, int field)
{
    unsigned int idx;
    
    p1 = &curve->points[0];
    
    for (idx = 1; idx < calibration.nvram.header.numcalpoints; idx++)
    {
        if (fieldAbs(p1, value, field) > fieldAbs(&curve->points[idx], value, field))
        {
            p1 = fieldAbs(&curve->points[idx], value, field);
        }
    }
    
    if (p1 == &curve->points[0])
    {
        p2 = &curve->points[1];
    }
    else
    {
        p2 = &curve->points[0];
    }
    
    for (idx = 0; idx < calibration.nvram.header.numcalpoints; idx++)
    {
        if (p1 != &curve->points[idx])
        {
            if (fieldAbs(p2, value, field) > fieldAbs(&curve->points[idx], value, field))
            {
                p2 = fieldAbs(&curve->points[idx], value, field);
            }
        }
    }
}


/*
 * This routine performs open loop calibration for Airoha.  It takes a value in mdbm
 * and uses the factory calibration routines to determine the appropriate register 
 * value to write to airoha.
 */
static int setInitialPowerLevel(struct piper_priv *digi, int mdbm)
{
    struct piper_priv *digi = hw->priv;
    wcd_curve_t *curve = determineCurve(digi->channel);
    wcd_point_t p1, p2;
    int slope;
    
    findClosestPoints(curve, value, &p1, &p2, OUT_POWER);
    slope = computeSlope(&p1, &p2, POWER_INDEX_OVER_OUT_POWER);
    airohaReg = computeY(p1, slope, value, POWER_INDEX_OVER_OUT_POWER);
    
    digi->rf->setPowerIndex(airohaReg);
}

    
    
typedef struct wcd_point {
	int16_t	 out_power;		/* Output Power */
	uint16_t adc_val;		/* Measured ADC val */
	uint8_t  power_index;		/* Airoha Power Index */
	uint8_t  reserved[3];		/* For future use */
} wcd_point_t;
    

Open loop calibration
    Set power level p on channel c
        Find power levels X1 and X2 closest to desired level Xd
        Compute slope m = (Y2 - Y1)/(X2 - X1)
        Compute Yd where
            Yd - Y1 = m(Xd - X1)
            Yd = mXd - mX1 + Y1

/*
 * This routine is called by the 80211mac library to set a new power level.  We
 * update the value in context memory and then kick the autocalibration thread.
 */
static int setNewPowerLevel(struct ieee80211_hw *hw, int value)
{
    struct piper_priv *digi = hw->priv;

    digi->tx_power = value;             /* save new power level */
    /*
     * Kick the calibration thread.
     */
    stopSampler(digi);
    kickCalibrationThread(RESTART_AUTOCALIBRATION_EVENT);
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
    
    while (getCalibrationData() == -1)
    {
        ssleep(60);
    }
    digi->rf->set_pwr = setNewPowerLevel;

    calibrate.state = RESTART_STATE;
    
    while (1)
    {
        switch (calibration.state)
        {
            case RESTART_STATE:
                setInitialPowerLevel(digi);
                /* Fall through is intended operation */
            case COLLECT_SAMPLES_STATE:
                startSampleCollection(digi);
                calibration.state = RECALIBRATE_STATE;
                ccode = waitForEvent(SAMPLE_TIMEOUT, SAMPLES_DONE_EVENT);
                break;
            case RECALIBRATE_STATE:
                stopSampler(digi);
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
            calibration.state = RESTART_STATE;
            break;
        }
    }
    
    return 0;
}


#if 1 //126

static int getPowerCalibrationData(void)
{
	int ret=-1;
	int i;

	strncpy(calibration.nvram.header.magic_string,"WCALDATA",strlen("WCALDATA"));
	calibration.nvram.header.numcalpoints = 5;

	for (i=0;i<WCD_CHANNELS_BG;i++) //Fill BG-CCK channels 1-14
	{
		calibration.nvram.cal_curves_bg[i][0].max_power_index = 63;

		calibration.nvram.cal_curves_bg[i][0].points[0].out_power= 	292;		/* Output Power */
		calibration.nvram.cal_curves_bg[i][0].points[0].adc_val=		29;			/* Measured ADC val */
		calibration.nvram.cal_curves_bg[i][0].points[0].power_index=	0;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][0].points[1].out_power= 	954;		/* Output Power */
		calibration.nvram.cal_curves_bg[i][0].points[1].adc_val=		69;			/* Measured ADC val */
		calibration.nvram.cal_curves_bg[i][0].points[1].power_index=	19;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][0].points[2].out_power= 	1340;		/* Output Power */
		calibration.nvram.cal_curves_bg[i][0].points[2].adc_val=	114;			/* Measured ADC val */
		calibration.nvram.cal_curves_bg[i][0].points[2].power_index=	33;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][0].points[3].out_power= 	1676;		/* Output Power */
		calibration.nvram.cal_curves_bg[i][0].points[3].adc_val=	166;			/* Measured ADC val */
		calibration.nvram.cal_curves_bg[i][0].points[3].power_index=	47;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][0].points[4].out_power=	1997;		/* Output Power */
		calibration.nvram.cal_curves_bg[i][0].points[4].adc_val=	187; 			/* Measured ADC val */
		calibration.nvram.cal_curves_bg[i][0].points[4].power_index=	63;				/* Airoha Power Index */;

		//calibration.nvram.cal_curves_bg[i][0].points[5].out_power=	1796;		/* Output Power */
		//calibration.nvram.cal_curves_bg[i][0].points[5].adc_val=	192;			/* Measured ADC val */
		//calibration.nvram.cal_curves_bg[i][0].points[5].power_index=	63;				/* Airoha Power Index */;
	}

	for (i=0;i<WCD_CHANNELS_BG;i++) //Fill BG-OFDM channels 1-14
	{
		//calibration.nvram.cal_curves_bg[i][1].max_power_index = 63;

		calibration.nvram.cal_curves_bg[i][1].points[0].adc_val=		12;			/* OFDM-CCK ADC offset */
		calibration.nvram.cal_curves_bg[i][1].points[0].power_index=	0;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][1].points[1].adc_val=		29;			/* OFDM-CCK ADC offset */
		calibration.nvram.cal_curves_bg[i][1].points[1].power_index=	19;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][1].points[2].adc_val=		46;			/* OFDM-CCK ADC offset */
		calibration.nvram.cal_curves_bg[i][1].points[2].power_index=	33;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][1].points[3].adc_val=		58;			/* OFDM-CCK ADC offset */
		calibration.nvram.cal_curves_bg[i][1].points[3].power_index=	47;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_bg[i][1].points[4].adc_val=		16; 			/* OFDM-CCK ADC offset */
		calibration.nvram.cal_curves_bg[i][1].points[4].power_index=	63;				/* Airoha Power Index */;

		//calibration.nvram.cal_curves_bg[i][1].points[5].adc_val=	15;			/* OFDM-CCK ADC offset */
		//calibration.nvram.cal_curves_bg[i][1].points[5].power_index=	63;				/* Airoha Power Index */;	
	}

	for (i=0;i<WCD_CHANNELS_A;i++) //Fill A channels 17-51
	{
		calibration.nvram.cal_curves_a[i].max_power_index = 50;

		calibration.nvram.cal_curves_a[i].points[0].out_power= 	-3000;		/* Output Power */
		calibration.nvram.cal_curves_a[i].points[0].adc_val=		0x08;			/* Measured ADC val */
		calibration.nvram.cal_curves_a[i].points[0].power_index=	0;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_a[i].points[1].out_power= 	1200;		/* Output Power */
		calibration.nvram.cal_curves_a[i].points[1].adc_val=		0x27;			/* Measured ADC val */
		calibration.nvram.cal_curves_a[i].points[1].power_index=	10;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_a[i].points[2].out_power=		1400;		/* Output Power */
		calibration.nvram.cal_curves_a[i].points[2].adc_val=		0x34;			/* Measured ADC val */
		calibration.nvram.cal_curves_a[i].points[2].power_index=	15;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_a[i].points[3].out_power=		1500;		/* Output Power */
		calibration.nvram.cal_curves_a[i].points[3].adc_val=		0x41;			/* Measured ADC val */
		calibration.nvram.cal_curves_a[i].points[3].power_index=	22;				/* Airoha Power Index */;

		calibration.nvram.cal_curves_a[i].points[4].out_power=		1600;		/* Output Power */
		calibration.nvram.cal_curves_a[i].points[4].adc_val=		183; //0x9C			/* Measured ADC val */
		calibration.nvram.cal_curves_a[i].points[4].power_index=	47;				/* Airoha Power Index */;

		//calibration.nvram.cal_curves_a[i].points[5].out_power=		1700;		/* Output Power */
		//calibration.nvram.cal_curves_a[i].points[5].adc_val=		192;//0xBD			/* Measured ADC val */
		//calibration.nvram.cal_curves_a[i].points[5].power_index=	63;				/* Airoha Power Index */;
	}

	ret = 0;

	return ret;
}
#endif



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

