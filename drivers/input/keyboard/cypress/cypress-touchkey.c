/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#ifdef CONFIG_CPU_FREQ
/* #include <mach/cpu-freq-v210.h> */
#warning "TODO"
#endif
#include "c1-cypress-gpio.h"

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
/*
Melfas touchkey register
*/
#define KEYCODE_REG 0x00
#define FIRMWARE_VERSION 0x01
#define TOUCHKEY_MODULE_VERSION 0x02
#define TOUCHKEY_ADDRESS	0x20

#define UPDOWN_EVENT_BIT 0x08
#define KEYCODE_BIT 0x07
#define ESD_STATE_BIT 0x10

#define I2C_M_WR 0		/* for i2c */

#define DEVICE_NAME "melfas-touchkey"

static int touchkey_keycode[3] = { 0, KEY_MENU, KEY_BACK };

static int touchkey_enable = 0;

struct i2c_touchkey_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
};
struct i2c_touchkey_driver *touchkey_driver = NULL;
struct work_struct touchkey_work;
struct workqueue_struct *touchkey_wq;

struct work_struct touch_update_work;
struct delayed_work touch_resume_work;

#ifdef WHY_DO_WE_NEED_THIS
static void __iomem *gpio_pend_mask_mem;
#define 	INT_PEND_BASE	0xE0200A54
#endif

static const struct i2c_device_id melfas_touchkey_id[] = {
	{"melfas_touchkey", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, melfas_touchkey_id);

static void init_hw(void);
static int i2c_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id);

extern int get_touchkey_firmware(char *version);
static int touchkey_led_status = 0;

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		   .name = "melfas_touchkey_driver",
		   },
	.id_table = melfas_touchkey_id,
	.probe = i2c_touchkey_probe,
};

static int touchkey_debug_count = 0;
static char touchkey_debug[104];
static int touch_version = 0;

static int touch_is_pressed(void)
{
	return 0;
}

static void touch_forced_release(void)
{
}

static void set_touchkey_debug(char value)
{
	if (touchkey_debug_count == 100)
		touchkey_debug_count = 0;

	touchkey_debug[touchkey_debug_count] = value;
	touchkey_debug_count++;
}

static int i2c_touchkey_read(u8 reg, u8 * val, unsigned int len)
{
	int err = 0;
	int retry = 10;
	struct i2c_msg msg[1];

	if ((touchkey_driver == NULL)) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled.R\n");
		return -ENODEV;
	}

	while (retry--) {
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = val;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);

		if (err >= 0) {
			return 0;
		}
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
		mdelay(10);
	}
	return err;

}

static int i2c_touchkey_write(u8 * val, unsigned int len)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 2;

	if ((touchkey_driver == NULL) || !(touchkey_enable == 1)) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled.W\n");
		return -ENODEV;
	}

	while (retry--) {
		data[0] = *val;
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = data;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);

		if (err >= 0) {
			return 0;
		}
		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		mdelay(10);
	}
	return err;
}

extern void TSP_forced_release(void);
void touchkey_work_func(struct work_struct *p)
{
	u8 data[3];
	int ret;
	int retry = 10;

	printk(KERN_DEBUG "[TouchKey] %s\n", __func__);

	set_touchkey_debug('a');
	if (!gpio_get_value(_3_GPIO_TOUCH_INT)) {
#ifdef CONFIG_CPU_FREQ
		/* set_dvfs_target_level(LEV_800MHZ); */
#warning "TODO"
#endif
		ret = i2c_touchkey_read(KEYCODE_REG, data, 1);
		set_touchkey_debug(data[0]);
		if ((data[0] & ESD_STATE_BIT) || (ret != 0)) {
			printk(KERN_DEBUG
			       "[TouchKey] ESD_STATE_BIT set or I2C fail: data: %d, retry: %d\n",
			       data[0], retry);

			/* releae key */
			input_report_key(touchkey_driver->input_dev,
					 touchkey_keycode[1], 0);
			input_report_key(touchkey_driver->input_dev,
					 touchkey_keycode[2], 0);
			retry = 10;
			while (retry--) {
				gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
				mdelay(300);
				init_hw();

				if (i2c_touchkey_read(KEYCODE_REG, data, 3) >= 0) {
					printk(KERN_DEBUG
					       "[TouchKey] %s touchkey init success\n",
					       __func__);
					set_touchkey_debug('O');
					enable_irq(IRQ_TOUCH_INT);
					return;
				}
				printk(KERN_ERR
				       "[TouchKey] %s %d i2c transfer error retry = %d\n",
				       __func__, __LINE__, retry);
			}

			/* touchkey die , do not enable touchkey
			   enable_irq(IRQ_TOUCH_INT); */
			touchkey_enable = -1;
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
			gpio_direction_output(_3_TOUCH_SDA_28V, 0);
			gpio_direction_output(_3_TOUCH_SCL_28V, 0);
			printk(KERN_DEBUG "[TouchKey] %s touchkey died\n",
			       __func__);
			set_touchkey_debug('D');
			return;
		}

		if (data[0] & UPDOWN_EVENT_BIT) {
			input_report_key(touchkey_driver->input_dev,
					 touchkey_keycode[data[0] &
							  KEYCODE_BIT], 0);
			input_sync(touchkey_driver->input_dev);
			printk(KERN_DEBUG "[TouchKey] release keycode:%d \n",
			       touchkey_keycode[data[0] & KEYCODE_BIT]);
		} else {
			if (touch_is_pressed()) {
				printk(KERN_DEBUG
				       "[TouchKey] touchkey pressed but don't send event because touch is pressed. \n");
				set_touchkey_debug('P');
			} else {
				if ((data[0] & KEYCODE_BIT) == 2) {
					/* if back key is pressed, release multitouch */
					printk(KERN_DEBUG "[TouchKey] touchkey release tsp input. \n");
					touch_forced_release();
				}

				input_report_key(touchkey_driver->input_dev,
						 touchkey_keycode[data[0] &
								  KEYCODE_BIT],
						 1);
				input_sync(touchkey_driver->input_dev);
				printk(KERN_DEBUG
				       "[TouchKey] press keycode:%d \n",
				       touchkey_keycode[data[0] & KEYCODE_BIT]);
			}
		}
	}

#ifdef WHY_DO_WE_NEED_THIS
	/* clear interrupt */
	if (readl(gpio_pend_mask_mem) & (0x1 << 1)) {
		writel(readl(gpio_pend_mask_mem) | (0x1 << 1),
		       gpio_pend_mask_mem);
	}
#endif
	set_touchkey_debug('A');
	enable_irq(IRQ_TOUCH_INT);
}

static irqreturn_t touchkey_interrupt(int irq, void *dummy)
{
	set_touchkey_debug('I');
	disable_irq_nosync(IRQ_TOUCH_INT);
	queue_work(touchkey_wq, &touchkey_work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int melfas_touchkey_early_suspend(struct early_suspend *h)
{
	struct regulator *regulator;

	touchkey_enable = 0;
	set_touchkey_debug('S');
	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_early_suspend\n");
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		return;
	}

	disable_irq(IRQ_TOUCH_INT);
	gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
	gpio_direction_output(_3_TOUCH_SDA_28V, 0);
	gpio_direction_output(_3_TOUCH_SCL_28V, 0);

	/* disable ldo18 */
	regulator = regulator_get(NULL, "touch_led");
	if (IS_ERR(regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "touch_led");
		return PTR_ERR(regulator);
	}
	regulator_disable(regulator);
	regulator_put(regulator);

	/* disable ldo11 */
	regulator = regulator_get(NULL, "touch");
	if (IS_ERR(regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "touch");
		return PTR_ERR(regulator);
	}
	regulator_disable(regulator);
	regulator_put(regulator);

	return 0;
}

static int melfas_touchkey_early_resume(struct early_suspend *h)
{
	unsigned char data = 1;
	struct regulator *regulator;

	set_touchkey_debug('R');
	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_early_resume\n");

	/* enable ldo18 */
	regulator = regulator_get(NULL, "touch_led");
	if (IS_ERR(regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "touch_led");
		return PTR_ERR(regulator);
	}
	regulator_enable(regulator);
	regulator_put(regulator);

	/* enable ldo11 */
	regulator = regulator_get(NULL, "touch");
	if (IS_ERR(regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "touch");
		return PTR_ERR(regulator);
	}
	regulator_enable(regulator);
	regulator_put(regulator);


	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		return;
	}
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	gpio_direction_output(_3_TOUCH_SDA_28V, 1);
	gpio_direction_output(_3_TOUCH_SCL_28V, 1);

	msleep(50);

#ifdef WHY_DO_WE_NEED_THIS
	/* clear interrupt */
	if (readl(gpio_pend_mask_mem) & (0x1 << 1)) {
		writel(readl(gpio_pend_mask_mem) | (0x1 << 1),
		       gpio_pend_mask_mem);
	}
#endif

	enable_irq(IRQ_TOUCH_INT);
	touchkey_enable = 1;
	i2c_touchkey_write(&data, 1);

	return 0;
}
#endif

extern int mcsdl_download_binary_data(void);
static int i2c_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char data;
	struct regulator *regulator;

	printk(KERN_DEBUG "[TouchKey] melfas i2c_touchkey_probe\n");

	touchkey_driver =
	    kzalloc(sizeof(struct i2c_touchkey_driver), GFP_KERNEL);
	if (touchkey_driver == NULL) {
		dev_err(dev, "failed to create our state\n");
		return -ENOMEM;
	}

	touchkey_driver->client = client;
	touchkey_driver->client->irq = IRQ_TOUCH_INT;
	strlcpy(touchkey_driver->client->name, "melfas-touchkey",
		I2C_NAME_SIZE);

	input_dev = input_allocate_device();

	if (!input_dev) {
		return -ENOMEM;
	}

	touchkey_driver->input_dev = input_dev;

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "melfas-touchkey/input0";
	input_dev->id.bustype = BUS_HOST;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(touchkey_keycode[1], input_dev->keybit);
	set_bit(touchkey_keycode[2], input_dev->keybit);

	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		return err;
	}

#ifdef WHY_DO_WE_NEED_THIS
	gpio_pend_mask_mem = ioremap(INT_PEND_BASE, 0x10);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	touchkey_driver->early_suspend.suspend = melfas_touchkey_early_suspend;
	touchkey_driver->early_suspend.resume = melfas_touchkey_early_resume;
	register_early_suspend(&touchkey_driver->early_suspend);
#endif				/* CONFIG_HAS_EARLYSUSPEND */


	/* enable ldo18 */
	regulator = regulator_get(NULL, "touch_led");
	if (IS_ERR(regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "touch_led");
		return PTR_ERR(regulator);
	}
	regulator_enable(regulator);
	regulator_put(regulator);

	/* enable ldo11 */
	regulator = regulator_get(NULL, "touch");
	if (IS_ERR(regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "touch");
		return PTR_ERR(regulator);
	}
	regulator_enable(regulator);
	regulator_put(regulator);

	touchkey_enable = 1;
	data = 1;
	i2c_touchkey_write(&data, 1);

	if (request_irq
	    (IRQ_TOUCH_INT, touchkey_interrupt, IRQF_DISABLED, DEVICE_NAME,
	     NULL)) {
		printk(KERN_ERR "[TouchKey] %s Can't allocate irq ..\n",
		       __func__);
		return -EBUSY;
	}

	set_touchkey_debug('K');
	return 0;
}

static void init_hw(void)
{
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	msleep(200);
	s3c_gpio_setpull(_3_GPIO_TOUCH_INT, S3C_GPIO_PULL_NONE);
	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_LEVEL_LOW);
	s3c_gpio_cfgpin(_3_GPIO_TOUCH_INT, _3_GPIO_TOUCH_INT_AF);
}

int touchkey_update_open(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t touchkey_update_read(struct file * filp, char *buf, size_t count,
			     loff_t * f_pos)
{
	char data[3] = { 0, };

	get_touchkey_firmware(data);
	put_user(data[1], buf);

	return 1;
}

int touchkey_update_release(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations touchkey_update_fops = {
	.owner = THIS_MODULE,
	.read = touchkey_update_read,
	.open = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_touchkey",
	.fops = &touchkey_update_fops,
};

static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;

	init_hw();
	if (get_touchkey_firmware(data) != 0) {
		i2c_touchkey_read(KEYCODE_REG, data, 3);
	}
	count = sprintf(buf, "0x%x\n", data[1]);

	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%x\n", data[1]);
	return count;
}

static ssize_t touch_version_write(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] input data --> %s\n", buf);

	return size;
}

extern int ISSP_main(void);
static int touchkey_update_status = 0;

void touchkey_update_func(struct work_struct *p)
{
	int retry = 10;
	touchkey_update_status = 1;
	printk(KERN_DEBUG "[TouchKey] %s start\n", __func__);
	while (retry--) {
		if (ISSP_main() == 0) {
			touchkey_update_status = 0;
			printk(KERN_DEBUG
			       "[TouchKey] touchkey_update succeeded\n");
			enable_irq(IRQ_TOUCH_INT);
			return;
		}
	}

	touchkey_update_status = -1;
	printk(KERN_DEBUG "[TouchKey] touchkey_update failed\n");
	return;
}

static ssize_t touch_update_write(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] touchkey firmware update \n");

	if (*buf == 'S') {
		disable_irq(IRQ_TOUCH_INT);
		INIT_WORK(&touch_update_work, touchkey_update_func);
		queue_work(touchkey_wq, &touch_update_work);
	}
	return size;
}

static ssize_t touch_update_read(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int count = 0;

	printk(KERN_DEBUG
	       "[TouchKey] touch_update_read: touchkey_update_status %d\n",
	       touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "Downloading\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "Fail\n");
	}

	return count;
}

static ssize_t touch_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	int data;
	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_DEBUG "[TouchKey] touch_led_control: %d \n", data);
		i2c_touchkey_write((u8 *)&data, 1);
		touchkey_led_status = data;
	} else {
		printk(KERN_DEBUG "[TouchKey] touch_led_control Error\n");
	}

	return size;
}

static ssize_t touchkey_enable_disable(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	return size;
}

static DEVICE_ATTR(touch_version, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH,
		   touch_version_read, touch_version_write);
static DEVICE_ATTR(touch_update, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH,
		   touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, NULL,
		   touch_led_control);
static DEVICE_ATTR(enable_disable, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, NULL,
		   touchkey_enable_disable);

static int __init touchkey_init(void)
{
	int ret = 0;
	int retry = 10;
	char data[3] = { 0, };

	gpio_request(_3_TOUCH_SDA_28V, "_3_TOUCH_SDA_28V");
	gpio_request(_3_TOUCH_SCL_28V, "_3_TOUCH_SCL_28V");
	gpio_request(_3_GPIO_TOUCH_EN, "_3_GPIO_TOUCH_EN");
	gpio_request(_3_GPIO_TOUCH_INT, "_3_GPIO_TOUCH_INT");

	ret = misc_register(&touchkey_update_device);
	if (ret) {
		printk(KERN_ERR "[TouchKey] %s misc_register fail\n", __func__);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_touch_version) < 0) {
		printk(KERN_ERR
		       "[TouchKey] %s device_create_file fail dev_attr_touch_version\n",
		       __func__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_touch_version.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_touch_update) < 0) {
		printk(KERN_ERR
		       "[TouchKey] %s device_create_file fail dev_attr_touch_update\n",
		       __func__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_touch_update.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device, &dev_attr_brightness) < 0) {
		printk(KERN_ERR
		       "[TouchKey] %s device_create_file fail dev_attr_touch_update\n",
		       __func__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_brightness.attr.name);
	}

	if (device_create_file
	    (touchkey_update_device.this_device,
	     &dev_attr_enable_disable) < 0) {
		printk(KERN_ERR
		       "[TouchKey] %s device_create_file fail dev_attr_touch_update\n",
		       __func__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_enable_disable.attr.name);
	}

	touchkey_wq = create_singlethread_workqueue("melfas_touchkey_wq");
	if (!touchkey_wq) {
		return -ENOMEM;
	}

	INIT_WORK(&touchkey_work, touchkey_work_func);

	init_hw();

	while (retry--) {
		if (get_touchkey_firmware(data) == 0)	/* melfas need delay for multiple read */
			break;
	}

	printk(KERN_DEBUG
	       "[TouchKey] %s F/W version: 0x%x, Module version:0x%x\n",
	       __func__, data[1], data[2]);
	touch_version = data[1];
	retry = 3;

	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk(KERN_ERR
		       "[TouchKey] melfas touch keypad registration failed, module not inserted.ret= %d\n",
		       ret);
	}

	return ret;

}

static void __exit touchkey_exit(void)
{
	printk(KERN_DEBUG "[TouchKey] %s \n", __func__);
	i2c_del_driver(&touchkey_i2c_driver);
	misc_deregister(&touchkey_update_device);

	if (touchkey_wq) {
		destroy_workqueue(touchkey_wq);
	}

	gpio_free(_3_TOUCH_SDA_28V);
	gpio_free(_3_TOUCH_SCL_28V);
	gpio_free(_3_GPIO_TOUCH_EN);
	gpio_free(_3_GPIO_TOUCH_INT);
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("melfas touch keypad");
