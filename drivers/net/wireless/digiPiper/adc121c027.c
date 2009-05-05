/*
    adc121C027.c - 16-bit I/O port with interrupt and reset

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

/* Addresses to scan: none, device is not autodetected */
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* Insmod parameters */
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

/* Return 0 if detection is successful, -ENODEV otherwise */
static int adc121C027_detect(struct i2c_client *client, int kind,
                          struct i2c_board_info *info)
{
        struct i2c_adapter *adapter = client->adapter;

    printk(KERN_ALERT "****************adc121C027_detect\n");
        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
                return -ENODEV;

        strlcpy(info->type, "adc121C027", I2C_NAME_SIZE);

        return 0;
}

static int adc121C027_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
        /* Register sysfs hooks */
    printk(KERN_ALERT "****************adc121C027_probe\n");
        return sysfs_create_group(&client->dev.kobj,
                                  &adc121C027_defattr_group);
}

static int adc121C027_remove(struct i2c_client *client)
{
    printk(KERN_ALERT "****************adc121C027_remove\n");
        sysfs_remove_group(&client->dev.kobj, &adc121C027_defattr_group);
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

        .detect         = adc121C027_detect,
        .address_data   = &addr_data,
};

static int __init adc121C027_init(void)
{
    printk(KERN_ALERT "****************adc121C027_init\n");
        return i2c_add_driver(&adc121C027_driver);
}

static void __exit adc121C027_exit(void)
{
    printk(KERN_ALERT "****************adc121C027_exit\n");
        i2c_del_driver(&adc121C027_driver);
}

MODULE_AUTHOR("Digi International <sales2@digi.com>");
MODULE_DESCRIPTION("ADC121C027 driver");
MODULE_LICENSE("GPL");

module_init(adc121C027_init);
module_exit(adc121C027_exit);



