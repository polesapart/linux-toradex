/*
 * This file contains the code which performs automatic recalibration of the
 * Airoha transceiver.
 *
 * Copyright (C) 2009 by Digi International Inc.
 * All rights reserved.
 *
 * This program is free softbware; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/timer.h>
#include <linux/crc32.h>

#include "pipermain.h"
#include "mac.h"
#include "airoha.h"
#include "airohaCalibration.h"
#include "adc121c027.h"

#define MAX_ALLOWED_ADC_ERROR           (2)             /* TODO: come up with a rational value for this */
#define POWER_INDEX_STEP                (1)             /* TODO: come up with a rational value for this */

#define SAMPLE_TIMEOUT              (HZ * 10)            /* TODO: What is a good sample timeout?  Do we need one? */
#define RECALIBRATION_PERIOD        (HZ * 1)           /* amount of time to wait between recalibrations*/

#define MAX_TOLERATED_ERROR_IN_MDBM     (1000)          /* amount of error we will ignore */

#define CONVERT_TO_MDBM(x)              (1000 * x)     /* power levels are dBm externally, but dBm/1000 internally */

#define NVRAM_WCAL_SIGNATURE            "WCALDATA"

#define MINIMUM_POWER_INDEX             (10)


/*
 * Events we will wait for, also return values for waitForEvent().
 */
#define TIMED_OUT_EVENT                 (1 << 0)
#define TRANSMIT_DONE_EVENT             (1 << 1)
#define SHUTDOWN_AUTOCALIBRATION_EVENT  (1 << 2)
#define RESTART_AUTOCALIBRATION_EVENT   (1 << 3)




static struct airohaCalibrationData calibration;

static DECLARE_WAIT_QUEUE_HEAD(waitQueue);


/*
 * This routine is called when the transmit frame has been
 * loaded into the FIFO and we just cleared the TX_HOLD bit.
 * The frame should be going out now.  We reset the peak
 * ADC measurement register and the receive frame count.
 */
#if 0
static void transmitIsStarting(struct piper_priv *digi)
{
    calibration.cops->adc_clear_peak(&calibration);
}
#endif

/*
 * This routine is called to shut down the transmit ADC sampler.
 */
static void stopSampler(struct piper_priv *digi)
{
    digi->tx_calib_cb = NULL;
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
static void processSample(struct piper_priv *digi)
{
#define MINIMUM_ADC_VALUE       (10)       /* if ADC is below this value, it's probably bad*/
    if (calibration.sampleCount < MAX_SAMPLES)
    {
        /*
         * Read the ADC value.  It is a 12-bit value.  We shift it 4 bits to
         * create an 8-bit value.
         */
        calibration.sample[calibration.sampleCount].sample = (calibration.cops->adc_read_peak(&calibration) >> 4);
        if (calibration.sample[calibration.sampleCount].sample > MINIMUM_ADC_VALUE)
        {
            calibration.sampleCount++;
        }
    }
}

static void transmitHasCompleted(struct piper_priv *digi)
{
    stopSampler(digi);
    kickCalibrationThread(digi, TRANSMIT_DONE_EVENT);
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
    unsigned int rates = digi->ac->rd_reg(digi, MAC_SSID_LEN);
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
    calibration.cops->adc_clear_peak(&calibration);
    digi->tx_calib_cb = transmitHasCompleted;
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

    if (timeout != 0)
    {
        ccode = wait_event_interruptible_timeout(waitQueue,
                                                 ((calibration.events & ALL_EVENTS_TO_WAIT_FOR(eventToWaitFor)) != 0),
                                                 timeout);
	__set_current_state(TASK_RUNNING);

    }
    else
    {
        ccode = 0;
    }
    spin_lock_irqsave(&calibration.lock, spinlockFlags);
    event = calibration.events;
    calibration.events = 0;
    spin_unlock_irqrestore(&calibration.lock, spinlockFlags);

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
    else
    {
        result = TIMED_OUT_EVENT;
    }

    return result;
}


#ifdef DEBUG
static void printPoint(wcd_point_t *p)
{
    printk("(%d, %d, %d)", p->out_power, p->adc_val, p->power_index);
}
#endif


/*
 * This routine finds the closest pair of points in a calibration curve.
 *
 *      curve       calibration curve
 *      value       look for the pr of points closest to this value
 *      p1          storage for one point
 *      p2          storage for another point
 *      field       tells us which field in the point struct to compare
 */
static void findClosestPoints(wcd_curve_t *curve, int value, wcd_point_t **p1, wcd_point_t **p2, int field)
{
    if (value <= curve->points[0].out_power)
    {
        *p1 = &curve->points[0];
        *p2 = &curve->points[1];
    }
    else if (value >= curve->points[calibration.nvram->header.numcalpoints - 1].out_power)
    {
        *p1 = &curve->points[calibration.nvram->header.numcalpoints - 2];
        *p2 = &curve->points[calibration.nvram->header.numcalpoints - 1];
    }
    else
    {
        unsigned int idx;

        for (idx = 1; idx < calibration.nvram->header.numcalpoints; idx++)
        {
            if (value < curve->points[idx].out_power)
            {
                *p1 = &curve->points[idx - 1];
                *p2 = &curve->points[idx];
                break;
            }
            else if (value == curve->points[idx].out_power)
            {
                /*
                 * Note that the if statement befpre the for loop already tested for the
                 * value being equal to the first or last point in the curve, so we don't
                 * have to worry about that condition in the code below.
                 */
                if ((value - curve->points[idx - 1].out_power) >= (curve->points[idx + 1].out_power - value))
                {
                    /*
                     * If the two points are equal distant, then favor the larger pair
                     * because I think the values on the low end are screwy.
                     */
                    *p1 = &curve->points[idx];
                    *p2 = &curve->points[idx + 1];
                }
                else
                {
                    *p1 = &curve->points[idx - 1];
                    *p2 = &curve->points[idx];
                }
                break;
            }
        }
    }
}


/*
 * Compute the slope of a curve between 2 points.  The slope is the rise over the run,
 * or (Y2 - Y1)/(X2 - X1).  This function handles more than one type of slope.
 */
static int computeSlopeTimes1000(wcd_point_t *p1, wcd_point_t *p2, int slopeType)
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
                slope = (((p2->power_index - p1->power_index) * 1000) + (divisor / 2))/divisor;
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
                slope = (((p2->adc_val - p1->adc_val) * 1000) + (divisor / 2))/divisor;
            }
            else
            {
                digi_dbg("divisor is zero\n");
            }
            break;
        case OUT_POWER_OVER_ADC:
            divisor = (p2->adc_val - p1->adc_val);
            if (divisor != 0)
            {
                slope = (((p2->out_power - p1->out_power) * 1000) + (divisor / 2))/divisor;
            }
            else
            {
                digi_dbg("divisor is zero\n");
            }
            break;
        case POWER_INDEX_OVER_ADC:
            divisor = (p2->adc_val - p1->adc_val);
            if (divisor != 0)
            {
                slope = (((p2->power_index - p1->power_index) * 1000) + (divisor / 2))/divisor;
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
 *      Yd         = m(Xd - p1->x) + p1->y
 *      Yd         = m(Xd) - m(p1->x) + p1->y
 */
static int computeY(wcd_point_t *p1, int slopeTimes1000, int x, int slopeType)
{
    int y = 0;

    switch (slopeType)
    {
        default:
            digi_dbg("Unexpected slope type %d.\n", slopeType);
            break;
        case POWER_INDEX_OVER_OUT_POWER:
            y = (((slopeTimes1000*x) - (slopeTimes1000*p1->out_power) + 500)/ 1000) + p1->power_index;
            break;
        case ADC_OVER_OUT_POWER:
            y = (((slopeTimes1000*x) - (slopeTimes1000*p1->out_power) + 500)/ 1000) + p1->adc_val;
            break;
        case OUT_POWER_OVER_ADC:
            y = (((slopeTimes1000*x) - (slopeTimes1000*p1->adc_val) + 500)/ 1000) + p1->out_power;
            break;
        case POWER_INDEX_OVER_ADC:
            y = (((slopeTimes1000*x) - (slopeTimes1000*p1->adc_val) + 500)/ 1000) + p1->power_index;
            break;
    }

    return y;
}



static wcd_curve_t *determineCurve(struct piper_priv *digi)
{
    unsigned int rates = digi->ac->rd_reg(digi, MAC_SSID_LEN);
    wcd_curve_t *curve = NULL;

    if (digi->rf->getBand(digi->channel) == IEEE80211_BAND_2GHZ)
    {
        if (rates & MAC_PSK_BRS_MASK)
        {
            digi_dbg("Using bg curve [%d][%d]\n", digi->channel, WCD_B_CURVE_INDEX);
            curve = &calibration.nvram->cal_curves_bg[digi->channel][WCD_B_CURVE_INDEX];
        }
        else /* if associated with AP that only supports G rates */
        {
            digi_dbg("Using bg curve [%d][%d]\n", digi->channel, WCD_G_CURVE_INDEX);
            curve = &calibration.nvram->cal_curves_bg[digi->channel][WCD_G_CURVE_INDEX];
        }
    }
    else
    {
        curve = &calibration.nvram->cal_curves_a[digi->channel - BAND_A_OFFSET];
        digi_dbg("Using A curve [%d]\n", digi->channel - BAND_A_OFFSET);
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
    wcd_point_t *p1, *p2;

    digi_dbg("Setting initial powerlevel %d milli dBm.\n", mdBm);
    calibration.curve = determineCurve(digi);
    findClosestPoints(calibration.curve, mdBm, &p1, &p2, OUT_POWER);
    calibration.slopeTimes1000 = computeSlopeTimes1000(p1, p2, POWER_INDEX_OVER_OUT_POWER);
    if (abs(p1->out_power - mdBm) < abs(p2->out_power - mdBm))
    {
        calibration.p1 = p1;
    }
    else
    {
        calibration.p1 = p2;
    }
    calibration.powerIndex = computeY(calibration.p1, calibration.slopeTimes1000, mdBm, POWER_INDEX_OVER_OUT_POWER);
    calibration.correctedPowerIndex = calibration.powerIndex;

    digi_dbg("Computed power index = %d.\n", calibration.powerIndex);
    digi->rf->set_pwr_index(digi->hw, calibration.powerIndex);

    /*
     * Let's compute and save the expected ADC value while we have all the necessary
     * information handy.
     */
#ifdef DEBUG
    digi_dbg("Using points "); printPoint(p1); printPoint(p2); printk("\n");
#endif
    calibration.adcSlopeTimes1000 = computeSlopeTimes1000(p1, p2, ADC_OVER_OUT_POWER);
    calibration.expectedAdc = computeY(calibration.p1, calibration.adcSlopeTimes1000, mdBm, ADC_OVER_OUT_POWER);
    digi_dbg("adcSlopeTimes1000 = %d, expectedAdc = %d\n", calibration.adcSlopeTimes1000, calibration.expectedAdc);
    calibration.outPowerSlopeTimes1000 = computeSlopeTimes1000(p1, p2, OUT_POWER_OVER_ADC);
    calibration.powerIndexSlopeTimes1000 = computeSlopeTimes1000(p1, p2, POWER_INDEX_OVER_ADC);
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
    int errorInAdc, errorInMdbm;

    digi_dbg("Samples: ");
    for (idx = 0; idx < calibration.sampleCount; idx++)
    {
#ifdef DEBUG
        printk("%d, ", calibration.sample[idx].sample);
#endif
        actualAdc += calibration.sample[idx].sample;
    }
#ifdef DEBUG
    printk("\n");
#endif
    actualAdc = actualAdc / calibration.sampleCount;

#if 1
    errorInAdc = actualAdc - calibration.expectedAdc;
    {
        wcd_point_t p =
        {
            .out_power = 0,
            .adc_val = 0
        };

        errorInMdbm = computeY(&p, calibration.outPowerSlopeTimes1000, abs(errorInAdc), OUT_POWER_OVER_ADC);
    }
    digi_dbg("actualAdc = %d, expectedAdc = %d, error mdbm = %d\n", actualAdc, calibration.expectedAdc, errorInMdbm);
    if (errorInMdbm > MAX_TOLERATED_ERROR_IN_MDBM)
    {
        int correction = computeY(calibration.p1, calibration.powerIndexSlopeTimes1000,
                                       actualAdc/*(calibration.expectedAdc - errorInAdc)*/, POWER_INDEX_OVER_ADC);
/*        correction -= calibration.powerIndex; */
        correction = (calibration.powerIndex - correction) / 2;
        calibration.correctedPowerIndex += correction;
        if (calibration.correctedPowerIndex < MINIMUM_POWER_INDEX)
        {
            calibration.correctedPowerIndex = MINIMUM_POWER_INDEX;
        }
        else if (calibration.correctedPowerIndex > calibration.curve->max_power_index)
        {
            calibration.correctedPowerIndex = calibration.curve->max_power_index;
        }
        digi->rf->set_pwr_index(digi->hw, calibration.correctedPowerIndex);
        digi_dbg("Power index corrected by %d, set to %d.\n", correction, calibration.correctedPowerIndex);
    }
#else
    digi_dbg("actualAdc = %d, expectedAdc = %d, ", actualAdc, calibration.expectedAdc);
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
#ifdef DEBUG
            printk(" ++ index to %d\n", calibration.powerIndex);
#endif
        }
        else if (actualAdc > calibration.expectedAdc)
        {
            calibration.powerIndex -= POWER_INDEX_STEP;
            digi->rf->set_pwr_index(digi->hw, calibration.powerIndex);
#ifdef DEBUG
            printk(" -- index to %d\n", calibration.powerIndex);
#endif
        }
    }
    else
    {
#ifdef DEBUG
        printk("Leaving power level unchanged.\n");
#endif
    }
#endif
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


#if 0 //126
/*
 * The calibration data is passed to the kernel from U-Boot.  The kernel
 * start up routines will have copied the data into digi->pdata->wcd.
 * We do a few sanity checks on the data and set up our own pointers to
 * it.
 */
static int getCalibrationData(struct piper_priv *digi)
{
    int result = -1;

    calibration.nvram = &digi->pdata->wcd;

    if (strncmp(calibration.nvram->header.magic_string,
                NVRAM_WCAL_SIGNATURE, strlen(NVRAM_WCAL_SIGNATURE)) == 0)
    {
        unsigned int crc = ~crc32_le(~0, (unsigned char const *)calibration.nvram->cal_curves_bg,
                                         calibration.nvram->header.wcd_len);

        if (crc == calibration.nvram->header.wcd_crc)
        {
            digi_dbg("CRC and signature for calibration data is okay\n");
            result = 0;
        }
        else
        {
            digi_dbg("Calibration data has invalid CRC.\n");
        }
    }
    else
    {
        digi_dbg("Calibration data has invalid signature.\n");
    }

    return result;
}

#else

/*
 * This routine generates dummy calibration data for testing.
 */
static int getCalibrationData(struct piper_priv *digi)
{
	int ret=-1;
	int i, j;
    static wcd_data_t nvram;

    (void) digi;

    calibration.nvram = &nvram;

	strncpy(calibration.nvram->header.magic_string,"WCALDATA",strlen("WCALDATA"));
	calibration.nvram->header.numcalpoints = 5;

	for (i=0;i<WCD_CHANNELS_BG;i++) //Fill BG-CCK channels 1-14
	{
	    for (j = 0; j < 2; j++)
	    {
    		calibration.nvram->cal_curves_bg[i][j].max_power_index = 63;

    		calibration.nvram->cal_curves_bg[i][j].points[0].out_power= 	-2905;		/* Output Power */
    		calibration.nvram->cal_curves_bg[i][j].points[0].adc_val=	 5;			    /* Measured ADC val */
    		calibration.nvram->cal_curves_bg[i][j].points[0].power_index= 0;				/* Airoha Power Index */;

    		calibration.nvram->cal_curves_bg[i][j].points[1].out_power= 	6457;		/* Output Power */
    		calibration.nvram->cal_curves_bg[i][j].points[1].adc_val=	 32;			/* Measured ADC val */
    		calibration.nvram->cal_curves_bg[i][j].points[1].power_index= 24;				/* Airoha Power Index */;

    		calibration.nvram->cal_curves_bg[i][j].points[2].out_power= 	 11287;		/* Output Power */
    		calibration.nvram->cal_curves_bg[i][j].points[2].adc_val=	 68;			/* Measured ADC val */
    		calibration.nvram->cal_curves_bg[i][j].points[2].power_index= 37;				/* Airoha Power Index */;

    		calibration.nvram->cal_curves_bg[i][j].points[3].out_power= 	16307;		/* Output Power */
    		calibration.nvram->cal_curves_bg[i][j].points[3].adc_val=	108;			/* Measured ADC val */
    		calibration.nvram->cal_curves_bg[i][j].points[3].power_index= 54;				/* Airoha Power Index */;

    		calibration.nvram->cal_curves_bg[i][j].points[4].out_power=	18198;		/* Output Power */
    		calibration.nvram->cal_curves_bg[i][j].points[4].adc_val=	130; 			/* Measured ADC val */
    		calibration.nvram->cal_curves_bg[i][j].points[4].power_index= 63;				/* Airoha Power Index */;
        }
	}


	for (i=0;i<WCD_CHANNELS_A;i++) //Fill A channels 17-51
	{
		calibration.nvram->cal_curves_a[i].max_power_index = 63;

		calibration.nvram->cal_curves_a[i].points[0].out_power= 	-2905;		/* Output Power */
		calibration.nvram->cal_curves_a[i].points[0].adc_val=	 19;			/* Measured ADC val */
		calibration.nvram->cal_curves_a[i].points[0].power_index= 0;				/* Airoha Power Index */;

		calibration.nvram->cal_curves_a[i].points[1].out_power= 	6457;		/* Output Power */
		calibration.nvram->cal_curves_a[i].points[1].adc_val=	 62;			/* Measured ADC val */
		calibration.nvram->cal_curves_a[i].points[1].power_index= 24;				/* Airoha Power Index */;

		calibration.nvram->cal_curves_a[i].points[2].out_power= 	 11287;		/* Output Power */
		calibration.nvram->cal_curves_a[i].points[2].adc_val=	 116;			/* Measured ADC val */
		calibration.nvram->cal_curves_a[i].points[2].power_index= 37;				/* Airoha Power Index */;

		calibration.nvram->cal_curves_a[i].points[3].out_power= 	16307;		/* Output Power */
		calibration.nvram->cal_curves_a[i].points[3].adc_val=	187;			/* Measured ADC val */
		calibration.nvram->cal_curves_a[i].points[3].power_index= 54;				/* Airoha Power Index */;

		calibration.nvram->cal_curves_a[i].points[4].out_power=	18198;		/* Output Power */
		calibration.nvram->cal_curves_a[i].points[4].adc_val=	190; 			/* Measured ADC val */
		calibration.nvram->cal_curves_a[i].points[4].power_index= 63;				/* Airoha Power Index */;
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
	int state;

	__set_current_state(TASK_RUNNING);

    while (1)
    {
        /*
         * We, the wireless driver, may be loaded before the I2C core has
         * loaded.  Therefore we may not be able to load our ADC driver,
         * which is an I2C client driver, when we load.  This loop tries
         * and retries to load the ADC driver until it succeeds.
         */

	/* TODO, FIXME make following code dependent on platform information
	 * allowign to initialize different adc */
        if (adc121C027_init(&calibration) == 0)
        {
            digi_dbg("ADC driver loaded...\n");
            break;
        }
        digi_dbg("Will try to load ADC driver again...\n");
        ssleep(10);
    }

    while (getCalibrationData(digi) == -1)
    {
        digi_dbg("getCalibrationData() failed.  Will try again in 60 seconds.\n");
        ssleep(60);
    }
    digi->rf->set_pwr = setNewPowerLevel;

    state = RESTART_STATE;

    digi_dbg("Starting autocalibration state machine.\n");
    do
    {
        int event;
        int timeout = 0;
        int expectedEvent = TIMED_OUT_EVENT;
        int nextState = RESTART_STATE;

        switch (state)
        {
            case RESTART_STATE:
                setInitialPowerLevel(digi, CONVERT_TO_MDBM(digi->tx_power));
                digi->calibrationTxRate = determineCalibrationTxRate(digi);
                calibration.sampleCount = 0;
                /* Fall through is intended operation */
            case COLLECT_SAMPLES_STATE:
                startSampleCollection(digi);
                nextState = GOT_SAMPLE_STATE;
                timeout = SAMPLE_TIMEOUT;
                expectedEvent = TRANSMIT_DONE_EVENT;
                break;
            case GOT_SAMPLE_STATE:
                processSample(digi);
                if (calibration.sampleCount < MAX_SAMPLES)
                {
                    nextState = COLLECT_SAMPLES_STATE;
                    timeout = 0;
                    break;
                }
                /* fall through is intended operation */
            case RECALIBRATE_STATE:
                recalibrate(digi);
                calibration.sampleCount = 0;
                nextState = COLLECT_SAMPLES_STATE;
                timeout = RECALIBRATION_PERIOD;
                expectedEvent = TIMED_OUT_EVENT;
                break;
            default:
                digi_dbg("Unknown state %d\n", state);
                nextState = COLLECT_SAMPLES_STATE;
                timeout = RECALIBRATION_PERIOD;
                expectedEvent = TIMED_OUT_EVENT;
                break;
        }

        state = nextState;
        event = waitForEvent(timeout, expectedEvent);

        switch (event)
        {
            case SHUTDOWN_AUTOCALIBRATION_EVENT:
                digi_dbg("Received SHUTDOWN_AUTOCALIBRATION_EVENT\n");
                break;
            case RESTART_AUTOCALIBRATION_EVENT:
                digi_dbg("Received RESTART_AUTOCALIBRATION_EVENT\n");
                state = RESTART_STATE;
                break;
            case TRANSMIT_DONE_EVENT:
                break;
            case TIMED_OUT_EVENT:
                if (state == GOT_SAMPLE_STATE)
                {
                    state = COLLECT_SAMPLES_STATE;
                }
        }
    } while (!kthread_should_stop());

    calibration.cops->adc_shutdown(&calibration);

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

void digiWifiDeInitCalibration(struct piper_priv *digi)
{
    calibration.events = SHUTDOWN_AUTOCALIBRATION_EVENT;

    wake_up_interruptible(&waitQueue);
    kthread_stop(calibration.threadCB);
}
EXPORT_SYMBOL_GPL(digiWifiDeInitCalibration);

