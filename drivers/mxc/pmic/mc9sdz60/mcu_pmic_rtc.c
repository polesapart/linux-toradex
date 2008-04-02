/*
 * Copyright 2008 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mc9sdz60/mcu_pmic_rtc.c
 * @brief This is the main file of mc9sdz60 RTC driver.
 *
 * @ingroup PMIC_POWER
 */

/*
 * Includes
 */
#include <linux/platform_device.h>
#include <asm/ioctl.h>
#include <linux/rtc.h>
#include <linux/bcd.h>

#include <asm/arch/pmic_status.h>
#include <asm/arch/pmic_rtc.h>
#include <asm/arch/pmic_external.h>
#include <asm/arch/pmic_power.h>

#define MCU_PMIC_RTC_NAME "pmic_rtc"
/*
 * Global variables
 */
static int pmic_rtc_major;
static void callback_alarm_asynchronous(void *);
static void callback_alarm_synchronous(void *);
static unsigned int pmic_rtc_poll(struct file *file, poll_table *wait);
static DECLARE_WAIT_QUEUE_HEAD(queue_alarm);
static DECLARE_WAIT_QUEUE_HEAD(pmic_rtc_wait);
static pmic_event_callback_t alarm_callback;
static pmic_event_callback_t rtc_callback;
static int pmic_rtc_detected;
static bool pmic_rtc_done;
static struct class *pmic_rtc_class;
struct rtc_time alarm_greg_time;
static DECLARE_MUTEX(mutex);

#define DEBUG_PMIC_RTC 1
#if DEBUG_PMIC_RTC
#define DPRINTK(format, args...) printk(KERN_ERR "pmic_rtc"format"\n", ##args)
#else
#define DPRINTK(format, args...)
#endif

/*
 * Real Time Clock Pmic API
 */

/*!
 * This is the callback function called on TSI Pmic event, used in asynchronous
 * call.
 */
static void callback_alarm_asynchronous(void *unused)
{
	pmic_rtc_done = true;
}

/*!
 * This is the callback function is used in test code for (un)sub.
 */
static void callback_test_sub(void)
{
	printk(KERN_INFO "*****************************************\n");
	printk(KERN_INFO "***** PMIC RTC 'Alarm IT CallBack' ******\n");
	printk(KERN_INFO "*****************************************\n");
}

/*!
 * This is the callback function called on TSI Pmic event, used in synchronous
 * call.
 */
static void callback_alarm_synchronous(void *unused)
{
	printk(KERN_INFO "*** Alarm IT Pmic ***\n");
	wake_up(&queue_alarm);
}

/*!
 * This function wait the Alarm event
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_wait_alarm(void)
{
	DEFINE_WAIT(wait);
	alarm_callback.func = callback_alarm_synchronous;
	alarm_callback.param = NULL;
	CHECK_ERROR(pmic_event_subscribe(EVENT_RTC, alarm_callback));
	prepare_to_wait(&queue_alarm, &wait, TASK_UNINTERRUPTIBLE);
	schedule();
	finish_wait(&queue_alarm, &wait);
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_RTC, alarm_callback));
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_wait_alarm);

/*!
 * This function set the real time clock of PMIC
 *
 * @param        pmic_time  	value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_set_time(struct timeval *pmic_time)
{
	u8 reg_val;
	struct rtc_time greg_time;

	rtc_time_to_tm(pmic_time->tv_sec, &greg_time);

	reg_val = greg_time.tm_sec % 10;
	reg_val |= ((greg_time.tm_sec / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_SECS, reg_val, 0x7f));

	reg_val = greg_time.tm_min % 10;
	reg_val |= ((greg_time.tm_min / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_MINS, reg_val, 0x7f));

	reg_val = greg_time.tm_hour % 10;
	reg_val |= ((greg_time.tm_hour / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_HRS, reg_val, 0x3f));

	reg_val = greg_time.tm_wday;
	CHECK_ERROR(pmic_write_reg(REG_MCU_DAY, reg_val, 0x3));

	reg_val = greg_time.tm_mday % 10;
	reg_val |= ((greg_time.tm_mday / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_DATE, reg_val, 0x3f));

	reg_val = greg_time.tm_mon % 10;
	reg_val |= ((greg_time.tm_mon / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_MONTH, reg_val, 0x1f));

	reg_val = greg_time.tm_year % 10;
	reg_val |= ((greg_time.tm_year / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_YEAR, reg_val, 0xff));

	DPRINTK("set time = %d y, %d m, %d d, %d h, %d m, %d s",
		greg_time.tm_year, greg_time.tm_mon, greg_time.tm_mday,
		greg_time.tm_hour, greg_time.tm_min, greg_time.tm_sec);
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_set_time);

/*!
 * This function get the real time clock of PMIC
 *
 * @param        pmic_time  	return value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_get_time(struct timeval *pmic_time)
{
	unsigned int reg_val;
	struct rtc_time greg_time;
	unsigned long time = 0;

	CHECK_ERROR(pmic_read_reg(REG_MCU_SECS, &reg_val, 0x7f));
	greg_time.tm_sec = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_MINS, &reg_val, 0x7f));
	greg_time.tm_min = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_HRS, &reg_val, 0x3f));
	greg_time.tm_hour = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_DAY, &reg_val, 0x3));
	greg_time.tm_wday = reg_val;

	CHECK_ERROR(pmic_read_reg(REG_MCU_DATE, &reg_val, 0x3f));
	greg_time.tm_mday = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_MONTH, &reg_val, 0x1f));
	greg_time.tm_mon = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_YEAR, &reg_val, 0xff));
	greg_time.tm_year = BCD2BIN(reg_val);

	DPRINTK("get time = %d y, %d m, %d d, %d h, %d m, %d s",
		greg_time.tm_year, greg_time.tm_mon, greg_time.tm_mday,
		greg_time.tm_hour, greg_time.tm_min, greg_time.tm_sec);

	rtc_tm_to_time(&greg_time, &time);
	pmic_time->tv_sec = time;

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_get_time);

/*!
 * This function set the real time clock alarm of PMIC
 *
 * @param        pmic_time  	value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_set_time_alarm(struct timeval *pmic_time)
{
	u8 reg_val;
	struct rtc_time greg_time;

	down_interruptible(&mutex);

	rtc_time_to_tm(pmic_time->tv_sec, &greg_time);
	rtc_time_to_tm(pmic_time->tv_sec, &alarm_greg_time);

	reg_val = greg_time.tm_sec % 10;
	reg_val |= ((greg_time.tm_sec / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_ALARM_SECS, reg_val, 0x7f));

	reg_val = greg_time.tm_min % 10;
	reg_val |= ((greg_time.tm_min / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_ALARM_MINS, reg_val, 0x7f));

	reg_val = greg_time.tm_hour % 10;
	reg_val |= ((greg_time.tm_hour / 10) << 4);
	CHECK_ERROR(pmic_write_reg(REG_MCU_ALARM_HRS, reg_val, 0x3f));

	/* enable alarm */
	CHECK_ERROR(pmic_write_reg(REG_MCU_ALARM_SECS, 0x80, 0x80));
	DPRINTK("set alarm time = %dh, %dm, %ds\n",
		greg_time.tm_hour, greg_time.tm_min, greg_time.tm_sec);

	up(&mutex);

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_set_time_alarm);

/*!
 * This function get the real time clock alarm of PMIC
 *
 * @param        pmic_time  	return value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_get_time_alarm(struct timeval *pmic_time)
{
	unsigned int reg_val;
	struct rtc_time greg_time;
	unsigned long time = 0;

	memzero(&greg_time, sizeof(struct rtc_time));
	greg_time = alarm_greg_time;

	CHECK_ERROR(pmic_read_reg(REG_MCU_ALARM_SECS, &reg_val, 0x7f));
	greg_time.tm_sec = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_ALARM_MINS, &reg_val, 0x7f));
	greg_time.tm_min = BCD2BIN(reg_val);

	CHECK_ERROR(pmic_read_reg(REG_MCU_ALARM_HRS, &reg_val, 0x3f));
	greg_time.tm_hour = BCD2BIN(reg_val);

	rtc_tm_to_time(&greg_time, &time);
	pmic_time->tv_sec = time;
	DPRINTK("get alarm time = %dh, %dm, %ds\n",
		greg_time.tm_hour, greg_time.tm_min, greg_time.tm_sec);

	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_get_time_alarm);

/*!
 * This function is used to un/subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 * @param        sub      	define if Un/subscribe event.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_event(t_rtc_int event, void *callback, bool sub)
{
	type_event rtc_event;
	if (callback == NULL) {
		return PMIC_ERROR;
	} else {
		rtc_callback.func = callback;
		rtc_callback.param = NULL;
	}
	switch (event) {
	case RTC_IT_ALARM:
		rtc_event = EVENT_RTC;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	if (sub)
		CHECK_ERROR(pmic_event_subscribe(rtc_event, rtc_callback));
	else
		CHECK_ERROR(pmic_event_unsubscribe(rtc_event, rtc_callback));

	return PMIC_SUCCESS;
}

/*!
 * This function is used to subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_event_sub(t_rtc_int event, void *callback)
{
	CHECK_ERROR(pmic_rtc_event(event, callback, true));
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_event_sub);

/*!
 * This function is used to un subscribe on RTC event IT.
 *
 * @param        event  	type of event.
 * @param        callback  	event callback function.
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_event_unsub(t_rtc_int event, void *callback)
{
	CHECK_ERROR(pmic_rtc_event(event, callback, false));
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(pmic_rtc_event_unsub);

/* Called without the kernel lock - fine */
static unsigned int pmic_rtc_poll(struct file *file, poll_table *wait)
{
	if (pmic_rtc_done)
		return POLLIN | POLLRDNORM;
	return 0;
}

/*!
 * This function implements IOCTL controls on a PMIC RTC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @param        cmd         the command
 * @param        arg         the parameter
 * @return       This function returns 0 if successful.
 */
static int pmic_rtc_ioctl(struct inode *inode, struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	struct timeval *pmic_time = NULL;

	if (_IOC_TYPE(cmd) != 'p')
		return -ENOTTY;

	if (arg) {
		pmic_time = kmalloc(sizeof(struct timeval), GFP_KERNEL);
		if (pmic_time == NULL)
			return -ENOMEM;

		/*      if (copy_from_user(pmic_time, (struct timeval *)arg,
		   sizeof(struct timeval))) {
		   return -EFAULT;
		   } */
	}

	switch (cmd) {
	case PMIC_RTC_SET_TIME:
		if (copy_from_user(pmic_time, (struct timeval *)arg,
				   sizeof(struct timeval)))
			return -EFAULT;

		pr_debug("SET RTC\n");
		CHECK_ERROR(pmic_rtc_set_time(pmic_time));
		break;
	case PMIC_RTC_GET_TIME:
		if (copy_to_user((struct timeval *)arg, pmic_time,
				 sizeof(struct timeval)))
			return -EFAULT;

		pr_debug("GET RTC\n");
		CHECK_ERROR(pmic_rtc_get_time(pmic_time));
		break;
	case PMIC_RTC_SET_ALARM:
		if (copy_from_user(pmic_time, (struct timeval *)arg,
				   sizeof(struct timeval)))
			return -EFAULT;

		pr_debug("SET RTC ALARM\n");
		CHECK_ERROR(pmic_rtc_set_time_alarm(pmic_time));
		break;
	case PMIC_RTC_GET_ALARM:
		if (copy_to_user((struct timeval *)arg, pmic_time,
				 sizeof(struct timeval)))
			return -EFAULT;

		pr_debug("GET RTC ALARM\n");
		CHECK_ERROR(pmic_rtc_get_time_alarm(pmic_time));
		break;
	case PMIC_RTC_WAIT_ALARM:
		printk(KERN_INFO "WAIT ALARM...\n");
		CHECK_ERROR(pmic_rtc_event_sub(RTC_IT_ALARM,
					       callback_test_sub));
		CHECK_ERROR(pmic_rtc_wait_alarm());
		printk(KERN_INFO "ALARM DONE\n");
		CHECK_ERROR(pmic_rtc_event_unsub(RTC_IT_ALARM,
						 callback_test_sub));
		break;
	case PMIC_RTC_ALARM_REGISTER:
		printk(KERN_INFO "PMIC RTC ALARM REGISTER\n");
		alarm_callback.func = callback_alarm_asynchronous;
		alarm_callback.param = NULL;
		CHECK_ERROR(pmic_event_subscribe(EVENT_RTC, alarm_callback));
		break;
	case PMIC_RTC_ALARM_UNREGISTER:
		printk(KERN_INFO "PMIC RTC ALARM UNREGISTER\n");
		alarm_callback.func = callback_alarm_asynchronous;
		alarm_callback.param = NULL;
		CHECK_ERROR(pmic_event_unsubscribe(EVENT_RTC, alarm_callback));
		pmic_rtc_done = false;
		break;
	default:
		pr_debug("%d unsupported ioctl command\n", (int)cmd);
		return -EINVAL;
	}

	if (arg) {
		if (copy_to_user((struct timeval *)arg, pmic_time,
				 sizeof(struct timeval)))
			return -EFAULT;

		kfree(pmic_time);
	}

	return 0;
}

/*!
 * This function implements the open method on a PMIC RTC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_rtc_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * This function implements the release method on a PMIC RTC device.
 *
 * @param        inode       pointer on the node
 * @param        file        pointer on the file
 * @return       This function returns 0.
 */
static int pmic_rtc_release(struct inode *inode, struct file *file)
{
	return 0;
}

/*!
 * This function is called to put the RTC in a low power state.
 * There is no need for power handlers for the RTC device.
 * The RTC cannot be suspended.
 *
 * @param   pdev  the device structure used to give information on which RTC
 *                device (0 through 3 channels) to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int pmic_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

/*!
 * This function is called to resume the RTC from a low power state.
 *
 * @param   pdev  the device structure used to give information on which RTC
 *                device (0 through 3 channels) to suspend
 *
 * @return  The function always returns 0.
 */
static int pmic_rtc_resume(struct platform_device *pdev)
{
	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */

static struct file_operations pmic_rtc_fops = {
	.owner = THIS_MODULE,
	.ioctl = pmic_rtc_ioctl,
	.poll = pmic_rtc_poll,
	.open = pmic_rtc_open,
	.release = pmic_rtc_release,
};

int pmic_rtc_loaded(void)
{
	return pmic_rtc_detected;
}
EXPORT_SYMBOL(pmic_rtc_loaded);


static int pmic_rtc_remove(struct platform_device *pdev)
{
	class_device_destroy(pmic_rtc_class, MKDEV(pmic_rtc_major, 0));
	class_destroy(pmic_rtc_class);
	unregister_chrdev(pmic_rtc_major, MCU_PMIC_RTC_NAME);
	return 0;
}

static int pmic_rtc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct class_device *temp_class;

	pmic_rtc_major = register_chrdev(0, MCU_PMIC_RTC_NAME, &pmic_rtc_fops);
	if (pmic_rtc_major < 0) {
		printk(KERN_ERR "Unable to get a major for pmic_rtc\n");
		return pmic_rtc_major;
	}

	pmic_rtc_class = class_create(THIS_MODULE, MCU_PMIC_RTC_NAME);
	if (IS_ERR(pmic_rtc_class)) {
		printk(KERN_ERR "Error creating pmic rtc class.\n");
		ret = PTR_ERR(pmic_rtc_class);
		goto err_out1;
	}

	temp_class = class_device_create(pmic_rtc_class, NULL,
					 MKDEV(pmic_rtc_major, 0),
					 NULL, MCU_PMIC_RTC_NAME);
	if (IS_ERR(temp_class)) {
		printk(KERN_ERR "Error creating pmic rtc class device.\n");
		ret = PTR_ERR(temp_class);
		goto err_out2;
	}

	pmic_rtc_detected = 1;

	/* start RTC */
	pmic_write_reg(REG_MCU_SECS, 0x80, 0x80);

	return ret;

err_out2:
	class_destroy(pmic_rtc_class);
err_out1:
	unregister_chrdev(pmic_rtc_major, MCU_PMIC_RTC_NAME);
	return ret;
}

static struct platform_driver pmic_rtc_driver_ldm = {
	.driver = {
		   .name = MCU_PMIC_RTC_NAME,
		   .owner = THIS_MODULE,
		   },
	.suspend = pmic_rtc_suspend,
	.resume = pmic_rtc_resume,
	.probe = pmic_rtc_probe,
	.remove = pmic_rtc_remove,
};

static int __init pmic_rtc_init(void)
{
	return platform_driver_register(&pmic_rtc_driver_ldm);
}
static void __exit pmic_rtc_exit(void)
{
	platform_driver_unregister(&pmic_rtc_driver_ldm);
	pr_debug("PMIC RTC driver successfully unloaded\n");
}

/*
 * Module entry points
 */

subsys_initcall(pmic_rtc_init);
module_exit(pmic_rtc_exit);

MODULE_DESCRIPTION("Pmic_rtc driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
