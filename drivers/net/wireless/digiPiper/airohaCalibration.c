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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <net/piper_pdata.h>

#include "pipermain.h"
#include "mac.h"
#include "airoha.h"
#include "adc121c027.h"
#include "airohaCalibration.h"

#define MAX_SAMPLES                     (3)
#define MAX_ALLOWED_ADC_ERROR           (200)           /* TODO: come up with a rational value for this */
#define POWER_INDEX_STEP                (10)            /* TODO: come up with a rational value for this */

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




/*
 * Slope types accepted by computeSlope().
 */
enum
{
    POWER_INDEX_OVER_OUT_POWER,
    ADC_OVER_OUT_POWER,
};



static DECLARE_WAIT_QUEUE_HEAD(waitQueue);

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
    wcd_curve_t *curve;
    int state;
    int slope;
    int adcSlope;
    int expectedAdc;
    unsigned int powerIndex;
} airohaCalibrationData_t;

static airohaCalibrationData_t calibration;



/*
 * This routine is called when the transmit frame has been
 * loaded into the FIFO and we just cleared the TX_HOLD bit.
 * The frame should be going out now.  We reset the peak
 * ADC measurement register and the receive frame count.
 */
static void transmitIsStarting(struct piper_priv *digi)
{
    digi->adcClearPeak(digi);
}

/*
 * This routine is called to shut down the transmit ADC sampler.
 */
static void stopSampler(struct piper_priv *digi)
{
    digi->txTransmitStarted = NULL;
    digi->txTransmitFinished = NULL;
    digi->calibrationTxRate = NULL;
}

/*
 * This routine is called to update the state of the calibration state machine
 * and wake up its thread.
 */
static void kickCalibrationThread(struct piper_priv *digi, unsigned int event)
{
    unsigned long spinlockFlags;
    
    spin_lock_irqsave(&calibration.lock, spinlockFlags);
    calibration.events |= event;
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
#define MINIMUM_ADC_VALUE       (400)       /* if ADC is below this value, it's probably bad*/
    if (calibration.sampleCount < MAX_SAMPLES)
    {
        calibration.sample[calibration.sampleCount].sample = digi->adcReadPeak(digi);
        if (calibration.sample[calibration.sampleCount].sample > MINIMUM_ADC_VALUE)
        {
            calibration.sampleCount++;
            if (calibration.sampleCount == MAX_SAMPLES)
            {
                stopSampler(digi);
                kickCalibrationThread(digi, SAMPLES_DONE_EVENT);
            }
        }
    }
}



/*
 * Scan the BRS mask and determine the lowest rate.  Look up the ieee80211_rate struct
 * for that rate and return it.
 */
static struct ieee80211_rate *findLowestRate(struct piper_priv *digi, unsigned int rates)
{
    unsigned int idx, rateIndex = 0;
    
    for (idx = 16; idx < sizeof(rates); idx++)
    {
        if (rates & (MAC_OFDM_BRS_MASK | MAC_PSK_BRS_MASK))
        {
            if (rates & (1 << idx))
            {
                break;
            }
            rateIndex++;
        }
    }
    
    return (struct ieee80211_rate *) digi->rf->getRate(rateIndex);
}



/*
 * Determine the appropriate transmit rate to use during calibration.
 */
static struct ieee80211_rate *determineCalibrationTxRate(struct piper_priv *digi)
{
    unsigned int rates = digi->read_reg(digi, MAC_SSID_LEN);
    struct ieee80211_rate *calibrationTxRate;
    
    if (   (digi->rf->getBand(digi->channel) == IEEE80211_BAND_2GHZ)
        && (rates & MAC_PSK_BRS_MASK))
    {
        calibrationTxRate = findLowestRate(digi, rates & MAC_PSK_BRS_MASK);
    }
    else
    {
        calibrationTxRate = findLowestRate(digi, rates & MAC_OFDM_BRS_MASK);
    }
    
    return calibrationTxRate;
}


                

/*
 * Start collecting sample ADC peak measurements for calibration.  Start
 * the process by installing the callbacks which the transmit code will
 * use to notify us when transmit frames go out.
 */
static void startSampleCollection(struct piper_priv *digi)
{    
    digi->calibrationTxRate = determineCalibrationTxRate(digi);
    calibration.sampleCount = 0;
    digi->txTransmitStarted = transmitIsStarting;
    digi->txTransmitFinished = transmitHasCompleted;
}


static unsigned int waitForEvent(unsigned int timeout, unsigned int eventToWaitFor)
{
#define ALL_EVENTS_TO_WAIT_FOR(x)   (eventToWaitFor \
                                     | SHUTDOWN_AUTOCALIBRATION_EVENT \
	                                 | RESTART_AUTOCALIBRATION_EVENT)

    unsigned long spinlockFlags;
	int ccode;
	unsigned int event;
	int result = TIMED_OUT_EVENT;
	                                    
    ccode = wait_event_interruptible_timeout(waitQueue, 
                                             ALL_EVENTS_TO_WAIT_FOR(eventToWaitFor),
                                             timeout);
                                             
    spin_lock_irqsave(&calibration.lock, spinlockFlags);
    event = calibration.events;
    calibration.events = 0;
    spin_unlock_irqrestore(&calibration.lock, spinlockFlags);

    digi_dbg("ccode = %d, event = 0x%8.8X\n", ccode, event);
    
    if ((ccode < 0) || (event & SHUTDOWN_AUTOCALIBRATION_EVENT))
    {
        result = SHUTDOWN_AUTOCALIBRATION_EVENT;
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
    
    return abs(difference);
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
static void findClosestPoints(wcd_curve_t *curve, int value, wcd_point_t *p1, wcd_point_t *p2, int field)
{
    unsigned int idx;
    wcd_point_t *temp;
    
    p1 = &curve->points[0];
    
    for (idx = 1; idx < calibration.nvram.header.numcalpoints; idx++)
    {
        if (fieldAbs(p1, value, field) > fieldAbs(&curve->points[idx], value, field))
        {
            p1 = &curve->points[idx];
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
        if ((p1 != &curve->points[idx]) && (p2 != &curve->points[idx]))
        {
            if (fieldAbs(p2, value, field) > fieldAbs(&curve->points[idx], value, field))
            {
                p2 = &curve->points[idx];
            }
        }
    }

    /*
     * Make sure p1 is before p2.  Swap them if necessary.
     *
     * TODO:  Do I always want to look at out_power, or should I use the field
     *        argument to determine which field in the struct to compare?
     */
    switch (field)
    {
        case OUT_POWER:
            if (p1->out_power > p2->out_power)
            {
                temp = p2;
                p2 = p1;
                p1 = temp;
            }
            break;
        case ADC_VAL:
            if (p1->adc_val > p2->adc_val)
            {
                temp = p2;
                p2 = p1;
                p1 = temp;
            }
            break;
        case POWER_INDEX:
            if (p1->power_index > p2->power_index)
            {
                temp = p2;
                p2 = p1;
                p1 = temp;
            }
            break;
        default:
            digi_dbg("Unknown field type %d.\n", field);
    }
}


/*
 * Compute the slope of a curve between 2 points.  The slope is the rise over the run,
 * or (Y2 - Y1)/(X2 - X1).  This function handles more than one type of slope.
 */
static int computeSlope(wcd_point_t *p1, wcd_point_t *p2, int slopeType)
{
    int slope = 0;
    int divisor;
    
    switch (slopeType)
    {
        default:
            digi_dbg("Unexpected slope type %d.\n", slopeType);
            break;
        case POWER_INDEX_OVER_OUT_POWER:
            divisor = (p2->out_power - p1->out_power);
            if (divisor != 0)
            {
                slope = ((p2->power_index - p1->power_index) + (divisor / 2))/(p2->out_power - p1->out_power);
            }
            else
            {
                digi_dbg("divisor is zero\n");
            }
            break;
        case ADC_OVER_OUT_POWER:
            divisor = (p2->out_power - p1->out_power);
            if (divisor != 0)
            {
                slope = ((p2->adc_val - p1->adc_val) + (divisor / 2))/(p2->out_power - p1->out_power);
            }
            else
            {
                digi_dbg("divisor is zero\n");
            }
            break;
    }
    
    return slope;
}


/*
 * If (x,y) is a point on a curve, then compute y given x, the slope of the curve,
 * and a known point on the curve.
 *
 * If (Xd, Yd) is the desired point, p1 is the known point, and m the slope, then
 *
 *      Yd - p1->y = m(Xd - p1->x)
 *      Yd         = m(Xd - p1->x) - p1->y
 *      Yd         = m(Xd) - m(p1->x) - p1->y
 */
static int computeY(wcd_point_t *p1, int slope, int x, int slopeType)
{
    int y = 0;
    
    switch (slopeType)
    {
        default:
            digi_dbg("Unexpected slope type %d.\n", slopeType);
            break;
        case POWER_INDEX_OVER_OUT_POWER:
            y = (slope*x) - (slope*p1->out_power) + p1->power_index;
            break;
        case ADC_OVER_OUT_POWER:
            y = (slope*x) - (slope*p1->out_power) + p1->adc_val;
            break;
    }
    
    return y;
}
    


static wcd_curve_t *determineCurve(struct piper_priv *digi)
{
    unsigned int rates = digi->read_reg(digi, MAC_SSID_LEN);
    wcd_curve_t *curve = NULL;
    
    if (digi->rf->getBand(digi->channel) == IEEE80211_BAND_2GHZ)
    {
        if (rates & MAC_PSK_BRS_MASK)
        {
            curve = &calibration.nvram.cal_curves_bg[digi->channel - 1][WCD_B_CURVE_INDEX];
        }
        else /* if associated with AP that only supports G rates */
        {
            curve = &calibration.nvram.cal_curves_bg[digi->channel - 1][WCD_G_CURVE_INDEX];
        }
    }
    else
    {
        curve = &calibration.nvram.cal_curves_a[digi->channel - BAND_A_OFFSET];
    }
    
    return curve;
}

                
/*
 * This routine performs open loop calibration for Airoha.  It takes a value in mdbm
 * and uses the factory calibration routines to determine the appropriate register 
 * value to write to airoha.
 */
static void setInitialPowerLevel(struct piper_priv *digi, int mdBm)
{
    wcd_point_t p1, p2;
    
    calibration.curve = determineCurve(digi);
    findClosestPoints(calibration.curve, mdBm, &p1, &p2, OUT_POWER);
    calibration.slope = computeSlope(&p1, &p2, POWER_INDEX_OVER_OUT_POWER);
    calibration.powerIndex = computeY(&p1, calibration.slope, mdBm, POWER_INDEX_OVER_OUT_POWER);
    
    digi->rf->set_pwr_index(digi->hw, calibration.powerIndex);
    
    /*
     * Let's compute and save the expected ADC value while we have all the necessary 
     * information handy.
     */
    calibration.adcSlope = computeSlope(&p1, &p2, ADC_OVER_OUT_POWER);
    calibration.expectedAdc = computeY(&p1, calibration.adcSlope, mdBm, ADC_OVER_OUT_POWER);
}



/*
 * This routine performs closed loop recalibration.  It is called periodically
 * to adjust the transmit power level.  It will be called after the ADC levels
 * for several transmit frames have been sampled.  It does the following:
 *
 *      1.  Average the samples together.
 *      2.  If the measured ADC level is too low, then increase the power 
 *          level one step.
 *      3.  If the measured ADC level is too high, then decrease the power
 *          level one step.
 */
static void recalibrate(struct piper_priv *digi)
{
    unsigned int idx;
    int actualAdc = 0;
    
    for (idx = 0; idx < calibration.sampleCount; idx++)
    {
        actualAdc = calibration.sample[idx].sample;
    }
    actualAdc = actualAdc / calibration.sampleCount;
    
    digi_dbg("actualAdc = %d, expectedAdc = %d\n", actualAdc, calibration.expectedAdc);
    if (abs(actualAdc - calibration.expectedAdc) > MAX_ALLOWED_ADC_ERROR)
    {
        if (actualAdc < calibration.expectedAdc)
        {
            calibration.powerIndex += POWER_INDEX_STEP;
            if (calibration.powerIndex > calibration.curve->max_power_index)
            {
                calibration.powerIndex = calibration.curve->max_power_index;
            }
            digi->rf->set_pwr_index(digi->hw, calibration.powerIndex);
            digi_dbg("increasing power index to %d\n", calibration.powerIndex);
        }
        else if (actualAdc > calibration.expectedAdc)
        {
            calibration.powerIndex -= POWER_INDEX_STEP;
            digi->rf->set_pwr_index(digi->hw, calibration.powerIndex);
            digi_dbg("reducing power index to %d\n", calibration.powerIndex);
        }
    }
    else
    {
        digi_dbg("Leaving power level unchanged.\n");
    }
}
    

/*
 * This routine is called by the 80211mac library to set a new power level.  We
 * update the value in context memory and then kick the autocalibration thread.
 */
static int setNewPowerLevel(struct ieee80211_hw *hw, uint8_t value)
{
    struct piper_priv *digi = hw->priv;

    digi->tx_power = value;             /* save new power level */

    /*
     * Kick the calibration thread.
     */
    stopSampler(digi);
    kickCalibrationThread(digi, RESTART_AUTOCALIBRATION_EVENT);
    
    return 0;
}


#if 1 //126

static int getCalibrationData(void)
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
 * This routine:
 *
 *      1. Loads the ADC driver.  
 *      2. Loads the calibration data.
 *      3. Implements the automatic calibration state machine.
 *
 */
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

    calibration.state = RESTART_STATE;
    
    while (1)
    {
        int ccode;
        
        switch (calibration.state)
        {
            case RESTART_STATE:
                setInitialPowerLevel(digi, digi->tx_power);
                /* Fall through is intended operation */
            case COLLECT_SAMPLES_STATE:
#define SAMPLE_TIMEOUT      (HZ * 5)            /* TODO: What is a good sample timeout?  Do we need one? */
                startSampleCollection(digi);
                calibration.state = RECALIBRATE_STATE;
                ccode = waitForEvent(SAMPLE_TIMEOUT, SAMPLES_DONE_EVENT);
                break;
            case RECALIBRATE_STATE:
#define RECALIBRATION_PERIOD        (HZ * 15)           /* amount of time to wait between recalibrations*/
                stopSampler(digi);
                recalibrate(digi);
                calibration.state = COLLECT_SAMPLES_STATE;
                ccode = waitForEvent(RECALIBRATION_PERIOD, TIMED_OUT_EVENT);
                break;
            default:
                digi_dbg("Unknown state %d\n", calibration.state);
                calibration.state = COLLECT_SAMPLES_STATE;
                ccode = waitForEvent(RECALIBRATION_PERIOD, TIMED_OUT_EVENT);
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



/*
 * This routine is called at initialization to set up the Airoha calibration routines.
 */
void digiWifiInitCalibration(struct piper_priv *digi)
{
	
    calibration.events = 0;
    calibration.sampleCount = 0;
    
    spin_lock_init(&calibration.lock);
    
    calibration.threadCB = kthread_run(digiWifiCalibrationThreadEntry, digi, PIPER_DRIVER_NAME " - calibration");
}

EXPORT_SYMBOL_GPL(digiWifiInitCalibration);

