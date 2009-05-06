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
I2C_CLIENT_INSMOD_1(adc121C027);

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

/* following are the sysfs callback functions */
static ssize_t adc121C027_show(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
        struct i2c_client *client = to_i2c_client(dev);

    printk(KERN_ALERT "****************adc121C027_show\n");
        return sprintf(buf, "%d\n", i2c_smbus_read_word_data(client,
                                                             psa->index));
}

static ssize_t adc121C027_store(struct device *dev, struct device_attribute *attr,
                             const char *buf, size_t count)
{
        struct sensor_device_attribute *psa = to_sensor_dev_attr(attr);
        struct i2c_client *client = to_i2c_client(dev);
        unsigned long val = simple_strtoul(buf, NULL, 0);

    printk(KERN_ALERT "****************adc121C027_store\n");

        if (val > 0xff)
                return -EINVAL;
        i2c_smbus_write_word_data(client, psa->index, val);
        return count;
}

/* Define the device attributes */

#define ADC121C027_ENTRY_RO(name, cmd_idx) \
        static SENSOR_DEVICE_ATTR(name, S_IRUGO, adc121C027_show, NULL, cmd_idx)

#define ADC121C027_ENTRY_RW(name, cmd_idx) \
        static SENSOR_DEVICE_ATTR(name, S_IRUGO | S_IWUSR, adc121C027_show, \
                                  adc121C027_store, cmd_idx)


ADC121C027_ENTRY_RO(result, ADC_RESULT);
ADC121C027_ENTRY_RW(alertStatus, ADC_ALERT_STATUS);
ADC121C027_ENTRY_RW(configuration, ADC_CONFIGURATION);
ADC121C027_ENTRY_RW(lowLimit, ADC_LOW_LIMIT);
ADC121C027_ENTRY_RW(highLimit, ADC_HIGH_LIMIT);
ADC121C027_ENTRY_RW(hysteresis, ADC_HYSTERESIS);
ADC121C027_ENTRY_RW(lowestValue, ADC_LOWEST_VALUE);
ADC121C027_ENTRY_RW(highestValue, ADC_HIGHEST_VALUE);

static struct attribute *adc121C027_attributes[] = {
        &sensor_dev_attr_result.dev_attr.attr,
        &sensor_dev_attr_alertStatus.dev_attr.attr,
        &sensor_dev_attr_configuration.dev_attr.attr,
        &sensor_dev_attr_lowLimit.dev_attr.attr,
        &sensor_dev_attr_highLimit.dev_attr.attr,
        &sensor_dev_attr_hysteresis.dev_attr.attr,
        &sensor_dev_attr_lowestValue.dev_attr.attr,
        &sensor_dev_attr_highestValue.dev_attr.attr,
        NULL
};

static struct attribute_group adc121C027_defattr_group = {
        .attrs = adc121C027_attributes,
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
        /* Register sysfs hooks */
    printk(KERN_ALERT "****************adc121C027_probe\n");
    printk(KERN_ALERT "****************client->addr = 0x%X\n", client->addr);
    
    return 0;
}

static int adc121C027_remove(struct i2c_client *client)
{
    printk(KERN_ALERT "****************adc121C027_remove\n");
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

        .address_data   = &addr_data,
};


/*
 * Turn on automatic A/D process by setting a non zero cycle time.
 */
static void adcInitializeHw(struct piper_priv *digi)
{
    #define ADC_CYCLE_TIME      (0x20)
    
    i2c_smbus_write_word_data(digi->adcI2cClient, ADC_CONFIGURATION, ADC_CYCLE_TIME);
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
        printk(KERN_ALERT "****************i2c_get_adapter(0) returned 0x%p\n", adapter);
        needToGetAdapter = (adapter == NULL);
    }
    
    if (adapter)
    {
        digi->adcI2cClient = i2c_new_device(adapter, 
                            &board_info);
        printk("i2c_new_device returned 0x%p\n", digi->adcI2cClient);
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
    
    return result;
}

    
EXPORT_SYMBOL_GPL(digiWifiInitAdc);


