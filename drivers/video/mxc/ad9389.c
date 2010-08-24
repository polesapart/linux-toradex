/*
 * ad9389.c
 *
 * Copyright 2010 - Digi International, Inc. All Rights Reserved.
 *
 * Based on ad9889.c driver from Armadeus:
 * Copyright (C) 2009 Armadeus Systems <nicolas.colombain@armadeus.com>
 * And also on mxcfb_sii9022.c from Pegatron:
 * Copyright 2009 Pegatron Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/console.h>
#include <video/ad9389.h>

#define HPD_INT			0x80
#define MSEN_INT		0x40
#define	VS_INT			0x20
#define	AUDIO_FIFO_FULL_INT	0x10
#define	ITU656_ERR_INT		0x08
#define EDID_RDY_INT		0x04
#define EDID_LENGTH		128
#define DRV_NAME		"ad9389"

struct ad9389_dev *pad9389;

static inline int ad9389_read_reg(struct i2c_client *client, u8 reg)
{
	return i2c_smbus_read_byte_data(client, reg);
}

static inline int ad9389_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static void ad9389_set_av_mute(struct i2c_client *client, int mute)
{
	u8 reg = mute ? 0x40 : 0x80;

	ad9389_write_reg(client, 0x45, reg);
}

static void ad9389_set_power_down(struct i2c_client *client, int powerd)
{
	u8 reg = ad9389_read_reg(client, 0x41) & ~0x40;

	if (powerd)
		reg |= 0x40;
	ad9389_write_reg(client, 0x41, reg);
}

static int ad9389_disp_connected(struct i2c_client *client)
{
	return (ad9389_read_reg(client, 0x42) & 0x40) != 0;
}

#ifdef DEBUG
static void ad9389_dump_edid(u8 *edid)
{
	int i;

	printk("EDID data:\n");
	for (i = 0; i < EDID_LENGTH; i++) {
		if (i % 8 == 0)
			printk("\n");
		printk("%02x ", edid[i]);
	}
	printk("\n");
}

static void fb_dump_modeline( struct fb_videomode *modedb, int num)
{
	struct fb_videomode *mode;
	int i;

	printk("Monitor/TV supported modelines:\n");

	for (i = 0; i < num; i++) {
		mode = &modedb[i];

		printk("   \"%dx%d%s%d\" %lu.%02lu ",
			mode->xres, mode->yres, (mode->vmode & FB_VMODE_INTERLACED) ? "i@" : "@", mode->refresh,
			(PICOS2KHZ(mode->pixclock) * 1000UL)/1000000,
			(PICOS2KHZ(mode->pixclock) ) % 1000);
		printk("%d %d %d %d ",
			mode->xres,
			mode->xres + mode->right_margin,
			mode->xres + mode->right_margin + mode->hsync_len,
			mode->xres + mode->right_margin + mode->hsync_len + mode->left_margin );
		printk("%d %d %d %d ",
			mode->yres,
			mode->yres + mode->lower_margin,
			mode->yres + mode->lower_margin + mode->vsync_len,
			mode->yres + mode->lower_margin + mode->vsync_len + mode->upper_margin );
		printk("%shsync %svsync\n", (mode->sync & FB_SYNC_HOR_HIGH_ACT) ? "+" : "-",
			   (mode->sync & FB_SYNC_VERT_HIGH_ACT) ? "+" : "-" );
	}
}
#else
static void ad9389_dump_edid(u8 *edid) {}
static void fb_dump_modeline( struct fb_videomode *modedb, int num) {}
#endif

static int ad9389_read_edid(struct i2c_client *client, u8 *edid)
{
	union i2c_smbus_data data;
	struct ad9389_pdata *config = client->dev.platform_data;
	struct ad9389_dev *ad9389 = i2c_get_clientdata(client);
	u8 *pd;
	int status, i;

	for (i = 0, pd = edid; i < EDID_LENGTH/I2C_SMBUS_BLOCK_MAX; i++, pd += I2C_SMBUS_BLOCK_MAX) {
		data.block[0] = I2C_SMBUS_BLOCK_MAX;
		status = i2c_smbus_xfer(ad9389->edid_ram->adapter, config->edid_addr,
					ad9389->edid_ram->flags,
					I2C_SMBUS_READ, i*I2C_SMBUS_BLOCK_MAX,
					I2C_SMBUS_I2C_BLOCK_DATA, &data);
		if (status < 0)
			return status;
		memcpy(pd, &data.block[1], data.block[0]);
	}

	return 0;
}

static int ad9389_parse_edid(struct fb_var_screeninfo *einfo, u8 *edid, int *dvi)
{
	int ret;

	if (einfo == NULL || edid == NULL || dvi == NULL)
		return -EINVAL;

	if (edid[1] == 0x00)
		return -ENOENT;

	*dvi = 0;
	if ((edid[20] == 0x80) || (edid[20] == 0x88) || (edid[20] == 0))
		*dvi = 1;

	ret = fb_parse_edid(edid, einfo);
	if (ret)
		return -ret;

	/* This is valid for version 1.3 of the EDID */
	if ((edid[18] == 1) && (edid[19] == 3)) {
		einfo->height = edid[21] * 10;
		einfo->width = edid[22] * 10;
	}

	return 0;
}

static void ad9389_fb_init(struct fb_info *info)
{
	struct ad9389_dev *ad9389 = pad9389;
	struct i2c_client *client = ad9389->client;
	struct ad9389_pdata *pdata = client->dev.platform_data;
	static struct fb_var_screeninfo var;
	int ret = 0;

	dev_info(info->dev, "%s\n", __func__);

	if (!ad9389_disp_connected(client)) {
		ad9389_set_power_down(client, 1);
		if(pdata->disp_disconnected)
			pdata->disp_disconnected();
		return;
	}

	if(pdata->disp_connected)
		pdata->disp_connected();

	dev_info(info->dev, "%s, display connected\n", __func__);
	memset(&var, 0, sizeof(var));

	/* Disable Power down and set mute to on */
	ad9389_set_power_down(client, 0);
	ad9389_set_av_mute(client, 1);

	/* disable I2S0 and set I2S standard format*/
	ad9389_write_reg(client, 0x0c, 0x00);

	/* set static reserved registers*/
	ad9389_write_reg(client,0x0a, 0);
	ad9389_write_reg(client, 0x98, 0x07);
	ad9389_write_reg(client, 0x9C, 0x38);
	ad9389_write_reg(client, 0x9d, 0x61);
	ad9389_write_reg(client, 0x9f, 0x70);
	/* set low speed pixel clock */
	ad9389_write_reg(client, 0xA2, 0x87);
	ad9389_write_reg(client, 0xA3, 0x87);

	/* set capture edge */
	ad9389_write_reg(client, 0xba, 0x60);
	ad9389_write_reg(client, 0x47, 0x80);

	mdelay(200);

	ret = ad9389_read_edid(ad9389->client, ad9389->edid_data);
	if (!ret) {
		ad9389_dump_edid(ad9389->edid_data);
		ret = ad9389_parse_edid(&var, ad9389->edid_data, &ad9389->dvi);
		if (!ret) {
			fb_edid_to_monspecs(ad9389->edid_data, &info->monspecs);
			if (info->monspecs.modedb_len) {
				fb_dump_modeline(info->monspecs.modedb, info->monspecs.modedb_len);
				pdata->videomode_to_modelist(ad9389, info->monspecs.modedb,
							     info->monspecs.modedb_len, &info->modelist);
			}
		}
	} else {
		/* TODO */
		printk("NO EDID information found, using default mode?\n");
	}

	pdata->videomode_to_var(ad9389, &var);
	pdata->pre_set_var(&var);

	var.activate = FB_ACTIVATE_ALL;
	acquire_console_sem();
	info->flags |= FBINFO_MISC_USEREVENT;
	fb_set_var(info, &var);
	info->flags &= ~FBINFO_MISC_USEREVENT;
	fb_blank(info, FB_BLANK_UNBLANK);
	release_console_sem();
}

int ad9389_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	return 0;
}

static void ad9389_work(struct work_struct *work)
{
	struct ad9389_dev *ad9389 = container_of(work, struct ad9389_dev, work);
	struct i2c_client *client =  ad9389->client;
	unsigned char irq_reg1;
	unsigned char irq_reg2;

	dev_dbg(&client->dev, "%s\n", __func__);

	mutex_lock(&ad9389->irq_lock);

	/* Interrupts are disabled here... */
	irq_reg1 = ad9389_read_reg(client, 0x96);
	irq_reg2 = ad9389_read_reg(client, 0x97);

	while ((irq_reg1 & 0xc4) | (irq_reg2 & 0xc0)) {

		dev_dbg(&client->dev, "IRQ register %02x/%02x\n",
			irq_reg1, irq_reg2);

		/* hot plug detections interrupt? */
		if (irq_reg1 & HPD_INT) {
			dev_dbg(&client->dev, "HPD irq\n");
			ad9389_fb_init(ad9389->fbi);
		}

		/* check for EDID ready flag, then call EDID Handler */
		if (irq_reg1 & EDID_RDY_INT) {
			dev_dbg(&client->dev, "EDID_RDY_INT\n");
		}

		/* ack and check again */
		ad9389_write_reg(client, 0x96, irq_reg1);
		ad9389_write_reg(client, 0x97, irq_reg2);

		irq_reg1 = ad9389_read_reg(client, 0x96);
		irq_reg2 = ad9389_read_reg(client, 0x97);
	}

	mutex_unlock(&ad9389->irq_lock);
}


static irqreturn_t ad9389_handler(int irq, void *dev_id)
{
	struct ad9389_dev *dev = (struct ad9389_dev *) dev_id;

	dev_dbg(&dev->client->dev, "%s\n", __func__);
	schedule_work(&dev->work);

	return IRQ_HANDLED;
}

static ssize_t ad9389_show_mute(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int read, ret = 0;

	read = ad9389_read_reg(client, 0x45);
	if (read < 0)
		ret = snprintf(buf, PAGE_SIZE, "error");
	else if (read & 0x40)
		ret = snprintf(buf, PAGE_SIZE, "on");
	else if (read & 0x80)
	  	ret = snprintf(buf, PAGE_SIZE, "off");

	return ret;
}

static ssize_t ad9389_store_mute(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int mute;

	if (!strcmp(buf, "on"))
	      mute = 1;
	else if (!strcmp(buf, "off"))
	      mute = 0;
	else
	      return 0;

	ad9389_set_av_mute(client, mute);

	return count;
}

static DEVICE_ATTR(mute, S_IWUSR | S_IRUGO, ad9389_show_mute, ad9389_store_mute);

static struct attribute *ad9389_attributes[] = {
	&dev_attr_mute.attr,
	NULL
};

static const struct attribute_group ad9389_attr_group = {
	.attrs = ad9389_attributes,
};

static struct notifier_block nb = {
	.notifier_call = ad9389_fb_event,
};

static int ad9389_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
	struct ad9389_pdata *pdata = client->dev.platform_data;
	struct ad9389_dev *ad9389;
	int ret = -EINVAL;

	/* Sanity checks */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	if (!pdata) {
		printk(KERN_ERR DRV_NAME ": Platform data not supplied\n");
		return -ENOENT;
	}

	if (!client->irq) {
		printk(KERN_ERR DRV_NAME ": Invalid irq value\n");
		return -ENOENT;
	}

	ad9389 = kzalloc(sizeof(struct ad9389_dev), GFP_KERNEL);
	if (ad9389 == NULL)
		return -ENOMEM;

	ad9389->edid_data = kmalloc(EDID_LENGTH, GFP_KERNEL);
	if (ad9389->edid_data == NULL) {
		ret = -ENOMEM;
		goto err_edid_alloc;
	}

	pad9389 = ad9389;
	ad9389->client = client;
	i2c_set_clientdata(client, ad9389);

	INIT_WORK(&ad9389->work, ad9389_work);
	mutex_init(&ad9389->irq_lock);
	mutex_lock(&ad9389->irq_lock);

	/* platform specific initialization (gpio, irq...) */
	if (pdata->hw_init)
		pdata->hw_init();

	ret = request_irq(client->irq, ad9389_handler,
			  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, DRV_NAME, ad9389);
	if (ret < 0) {
		printk(KERN_ERR DRV_NAME ": Could not allocate IRQ (n %d)\n", client->irq);
		goto err_irq;
	}

	/**
	 * There is no good way to detect if the chip is present. We assume that its present
	 * because somebody answered (ack) on the device address...
	 */
	ret = ad9389_read_reg(client, 0x00);
	if (ret < 0) {
		printk(KERN_WARNING DRV_NAME ": i2c transfer error, (device present?)\n");
		ret = -ENODEV;
		goto err_presence;
	}

	ad9389->chiprev = (u8)ret;
	ad9389->edid_ram = i2c_new_dummy(client->adapter, pdata->edid_addr);
	if (!ad9389->edid_ram) {
		printk(KERN_WARNING DRV_NAME ": can't add i2c device at 0x%x\n", pdata->edid_addr);
		goto err_presence;
	}

	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &ad9389_attr_group);
	if (ret)
		goto err_sysfs_file;

	ad9389->fbi = registered_fb[pdata->dispif];
	fb_show_logo(registered_fb[pdata->dispif], 0);
	fb_register_client(&nb);

	/* Ack any active interrupt and enable irqs */
	ad9389_write_reg(client, 0x94, 0x84);
	ad9389_write_reg(client, 0x95, 0xc3);
	ad9389_write_reg(client, 0x96, 0x84);
	ad9389_write_reg(client, 0x97, 0xc3);

	mutex_unlock(&ad9389->irq_lock);

	ad9389_fb_init(registered_fb[pdata->dispif]);

	printk(KERN_INFO DRV_NAME ": device detected at address 0x%x, chip revision 0x%02x\n",
	       client->addr << 1, ad9389->chiprev);

	return 0;

err_sysfs_file:
	i2c_unregister_device(ad9389->edid_ram);
err_presence:
	free_irq(client->irq, ad9389);
err_irq:
	flush_scheduled_work();
	if (pdata->hw_deinit)
		pdata->hw_deinit();
	kfree(ad9389->edid_data);
err_edid_alloc:
	kfree(ad9389);
	pad9389 = NULL;
	return ret;
}


static int ad9389_remove(struct i2c_client *client)
{
	struct ad9389_pdata *pdata = client->dev.platform_data;
	struct ad9389_dev *ad9389 = i2c_get_clientdata(client);

	free_irq(client->irq, ad9389);
	flush_scheduled_work();
	sysfs_remove_group(&client->dev.kobj, &ad9389_attr_group);
	i2c_unregister_device(ad9389->edid_ram);
	fb_unregister_client(&nb);
	kfree(ad9389->edid_data);
	kfree(ad9389);
	pad9389 = NULL;

	if (pdata->hw_deinit)
		pdata->hw_deinit();

	return 0;
}

#ifdef CONFIG_PM
static int ad9389_suspend(struct i2c_client *client, pm_message_t state)
{
	dev_dbg(&client->dev, "PM suspend\n");
	ad9389_set_power_down(client, 1);

	return 0;
}

static int ad9389_resume(struct i2c_client *client)
{
	dev_dbg(&client->dev, "PM resume\n");
	ad9389_set_power_down(client, 0);
	return 0;
}
#else
#define ad9389_suspend	NULL
#define ad9389_resume	NULL
#endif


static struct i2c_device_id ad9389_id[] = {
	{ "ad9389", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ad9389_id);

static struct i2c_driver ad9389_driver = {
	.driver = {
		.name = "ad9389",
	},
	.probe		= ad9389_probe,
	.remove		= ad9389_remove,
	.suspend	= ad9389_suspend,
	.resume		= ad9389_resume,
	.id_table	= ad9389_id,

};

static int __init ad9389_init(void)
{
	return i2c_add_driver(&ad9389_driver);
}

static void __exit ad9389_exit(void)
{
	i2c_del_driver(&ad9389_driver);
}

module_init(ad9389_init);
module_exit(ad9389_exit);

MODULE_DESCRIPTION("AD9389 hdmi/dvi driver");
MODULE_AUTHOR("Digi International Inc.");
MODULE_LICENSE("GPL");
