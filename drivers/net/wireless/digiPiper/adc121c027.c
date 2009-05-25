/*
    adc121C027.c - Analog to Digital converter integrated into Piper.

    Copyright (C) 2009 Digi International <sales2@digi.com>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; version 2 of the License.
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon-sysfs.h>
#include "pipermain.h"

/* Addresses to scan: none, device is not autodetected */
/* static const unsigned short normal_i2c[] = { I2C_CLIENT_END }; */

/* Insmod parameters */
#define ADC_I2C_ADDR        (0x51)

static const unsigned short normal_i2c[] =
    { ADC_I2C_ADDR, I2C_CLIENT_END };
static const unsigned short dummy_i2c_addrlist[] = { I2C_CLIENT_END };

static struct i2c_client_address_data addr = {
	.normal_i2c = normal_i2c,
	.probe = dummy_i2c_addrlist,
	.ignore = dummy_i2c_addrlist,
};
/*I2C_CLIENT_INSMOD_1(adc121C027);*/

enum adc121C027_cmd
{
        ADC_RESULT              = 0,
        ADC_ALERT_STATUS        = 1,
        ADC_CONFIGURATION       = 2,
        ADC_LOW_LIMIT           = 3,
        ADC_HIGH_LIMIT          = 4,
        ADC_HYSTERESIS          = 5,
        ADC_LOWEST_VALUE        = 6,
        ADC_HIGHEST_VALUE       = 7,
};



static u16 adcReadPeak(struct piper_priv *digi)
{
    return be16_to_cpu(i2c_smbus_read_word_data(digi->adcI2cClient, ADC_HIGHEST_VALUE));
}

static void adcClearPeak(struct piper_priv *digi)
{
    i2c_smbus_write_word_data(digi->adcI2cClient, ADC_HIGHEST_VALUE, 0);
}

static u16 adcReadLastValue(struct piper_priv *digi)
{
    return be16_to_cpu(i2c_smbus_read_word_data(digi->adcI2cClient, ADC_RESULT));
}



static int adc121C027_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int result = -EINVAL;
    
    if (client->addr == ADC_I2C_ADDR)
    {
        result = 0;
    }
    
    return result;
}

static int adc121C027_remove(struct i2c_client *client)
{
    /*
     * Real shut down will be done by adcShutdown().
     */
    return 0;
}

static const struct i2c_device_id adc121C027_id[] = {
        { "adc121C027", 0 },
        { }
};

static struct i2c_driver adc121C027_driver = {
        .driver = {
                .name   = "adc121C027",
        },
        .probe          = adc121C027_probe,
        .remove         = adc121C027_remove,
        .id_table       = adc121C027_id,

        .address_data   = &addr,
};


/*
 * Turn on automatic A/D process by setting a non zero cycle time.
 */
static void adcInitializeHw(struct piper_priv *digi)
{
    #define ADC_CYCLE_TIME      (0x20)
    
    i2c_smbus_write_word_data(digi->adcI2cClient, ADC_CONFIGURATION, ADC_CYCLE_TIME);
}

void adcShutdown(struct piper_priv *digi)
{
    if (digi->adcI2cClient)
    {
        i2c_unregister_device(digi->adcI2cClient);
        digi->adcI2cClient = NULL;
    }
    
    i2c_del_driver(&adc121C027_driver);
}
    
    
int digiWifiInitAdc(struct piper_priv *digi)
{
    int result = -1;
    static struct i2c_adapter *adapter;    
    struct i2c_board_info board_info = 
    {
        .type = "adc121C027",
        .addr = ADC_I2C_ADDR,
    };
    static int needToAddDriver = 1;
    static int needToGetAdapter = 1;
    
    if (needToAddDriver)
    {
        needToAddDriver = (i2c_add_driver(&adc121C027_driver) != 0);
        if (needToAddDriver)
        {
            goto done;
        }
    }

    if (needToGetAdapter)
    {
        adapter = i2c_get_adapter(0);
        needToGetAdapter = (adapter == NULL);
    }
    
    if (adapter)
    {
        digi->adcI2cClient = i2c_new_device(adapter, 
                            &board_info);
        if (digi->adcI2cClient != NULL)
        {
            adcInitializeHw(digi);
            result = 0;
        }
    }

done:
    digi->adcReadPeak = adcReadPeak;
    digi->adcClearPeak = adcClearPeak;
    digi->adcReadLastValue = adcReadLastValue;
    digi->adcShutdown = adcShutdown;
    
    return result;
}

    
EXPORT_SYMBOL_GPL(digiWifiInitAdc);


