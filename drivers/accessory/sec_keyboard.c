#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/30pin_con.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "sec_keyboard.h"

static struct sec_keyboard_drvdata *g_data;
extern struct class *sec_class;

static void timer_work(struct work_struct *work)
{
	struct sec_keyboard_drvdata *data = container_of(work,
			struct sec_keyboard_drvdata, work_timer);

	int error;
	if (UNKOWN_KEYLAYOUT == data->kl) {
		data->acc_power(1, false);
		data->pre_connected = false;

		/* Set baud rate for the keyboard uart */
		error = change_console_baud_rate(115200);
		if (error < 0)
			printk(KERN_ERR "[Keyboard] Couldn't modify the baud rate.\n");

		if (data->check_uart_path)
			data->check_uart_path(false);
	}
}

static void keyboard_timer(unsigned long _data)
{
	/* this part will be run in the disable() func */
	struct sec_keyboard_drvdata *data =
		(struct sec_keyboard_drvdata *)_data;

	if (!work_pending(&data->work_timer))
		schedule_work(&data->work_timer);
}

void remapkey_timer(unsigned long _data)
{
	struct sec_keyboard_drvdata *data =
		(struct sec_keyboard_drvdata *)_data;
	unsigned int keycode = 0;
	if (data->pressed[0x45] || data->pressed[0x48]) {
		keycode = data->keycode[data->remap_key];
		input_report_key(data->input_dev, keycode, 1);
		input_sync(data->input_dev);
	}
	data->remap_key = 0;
}

void release_all_keys(struct sec_keyboard_drvdata *data)
{
	int i;
	pr_info("[Keyboard] Release the pressed keys.\n");
	for (i = 0; i < KEYBOARD_MAX; i++) {
		if (data->pressed[i]) {
			input_report_key(data->input_dev, data->keycode[i], 0);
			data->pressed[i] = false;
		}
		input_sync(data->input_dev);
	}
}

static void key_event_work(struct work_struct *work)
{
	struct sec_keyboard_drvdata *data = container_of(work,
			struct sec_keyboard_drvdata, work_msg);
	bool press;
	unsigned int keycode;
	unsigned char scan_code;

	while (data->buf_front != data->buf_rear) {
		mutex_lock(&data->mutex);
		scan_code = data->key_buf[data->buf_front];
		data->buf_front++;
		if (data->buf_front > MAX_BUF)
			data->buf_front = 0;
		mutex_unlock(&data->mutex);

		/* keyboard driver need the contry code*/
		if (data->kl == UNKOWN_KEYLAYOUT) {
			switch (scan_code) {
			case US_KEYBOARD:
				data->kl = US_KEYLAYOUT;
				data->keycode[49] = KEY_BACKSLASH;
				/* for the wakeup state*/
				data->pre_kl = data->kl;
				pr_info("[Keyboard] US keyboard is attacted.\n");
				break;

			case UK_KEYBOARD:
				data->kl = UK_KEYLAYOUT;
				data->keycode[49] = KEY_NUMERIC_POUND;
				/* for the wakeup state*/
				data->pre_kl = data->kl;
				pr_info("[Keyboard] UK keyboard is attacted.\n");
				break;

			default:
				pr_info("[Keyboard] Unkown layout : %x\n",
					scan_code);
				break;
			}
		} else {
			switch (scan_code) {
			case 0x0:
				release_all_keys(data);
				break;

			case 0xca: /* Caps lock on */
			case 0xcb: /* Caps lock off */
			case 0xeb: /* US keyboard */
			case 0xec: /* UK keyboard */
				break; /* Ignore */

			case 0x45:
			case 0x48:
				data->remap_key = scan_code;
				data->pressed[scan_code] = true;
				mod_timer(&data->key_timer, jiffies + HZ/3);
				break;

			case 0xc5:
			case 0xc8:
				keycode = (scan_code & 0x7f);
				data->pressed[keycode] = false;
				if (0 == data->remap_key) {
					input_report_key(data->input_dev,
						data->keycode[keycode], 0);
					input_sync(data->input_dev);
				} else {
					del_timer(&data->key_timer);
					if (0x48 == keycode)
						keycode = KEY_NEXTSONG;
					else
						keycode = KEY_PREVIOUSSONG;

					input_report_key(data->input_dev,
						keycode, 1);
					input_report_key(data->input_dev,
						keycode, 0);
					input_sync(data->input_dev);
				}
				break;

			default:
				keycode = (scan_code & 0x7f);
				press = ((scan_code & 0x80) != 0x80);

				if (keycode >= KEYBOARD_MIN
					|| keycode <= KEYBOARD_MAX) {
					data->pressed[keycode] = press;
					input_report_key(data->input_dev,
						data->keycode[keycode], press);
					input_sync(data->input_dev);
				}
				break;
			}
		}
	}
}

void send_keyevent(unsigned int key_code)
{
	struct sec_keyboard_drvdata *data = g_data;

	data->key_buf[data->buf_rear]  = key_code;
	data->buf_rear++;
	if (data->buf_rear > MAX_BUF)
		data->buf_rear = 0;

	if (!work_pending(&data->work_msg))
		schedule_work(&data->work_msg);
}

static int check_keyboard_dock(struct sec_keyboard_callbacks *cb, bool val)
{
	struct sec_keyboard_drvdata *data =
		container_of(cb, struct sec_keyboard_drvdata, callbacks);
	int try_cnt = 0;
	int error = 0;
	int max_cnt = 14;

	if (!val)
		data->dockconnected = false;
	else {
		del_timer(&data->timer);
		/* wakeup by keyboard dock */
		if (data->pre_connected) {
			if (UNKOWN_KEYLAYOUT != data->pre_kl) {
				data->kl = data->pre_kl;
				data->acc_power(1, true);
				pr_info("[Keyboard] kl : %d\n", data->pre_kl);
				return 1;
			}
		} else
			data->pre_kl = UNKOWN_KEYLAYOUT;

		data->pre_connected = true;

		/* to prevent the over current issue */
		data->acc_power(0, false);

		if (data->check_uart_path)
			data->check_uart_path(true);

		/* Set baud rate for the keyboard uart */
		error = change_console_baud_rate(9600);
		if (error < 0)
			pr_err("[Keyboard] Couldn't modify the baud rate.\n");

		msleep(200);
		data->acc_power(1, true);

		/* try to get handshake data */
		for (try_cnt = 0; try_cnt < max_cnt; try_cnt++) {
			msleep(50);
			if (data->kl != UNKOWN_KEYLAYOUT) {
				data->dockconnected = true;
				break;
			}
			if (gpio_get_value(data->acc_int_gpio)) {
				pr_info("[Keyboard] acc is disconnected.\n");
				break;
			}
		}
	}

	if (data->dockconnected)
		return 1;
	else	{
		if (data->pre_connected) {
			/* stop the thread and clear the buffer*/
			data->buf_front = data->buf_rear = 0;

			data->dockconnected = false;
			mod_timer(&data->timer, jiffies + HZ/2);

			data->kl = UNKOWN_KEYLAYOUT;
			release_all_keys(data);
		}
		return 0;
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void keyboard_early_suspend(struct early_suspend *early_sus)
{
	struct sec_keyboard_drvdata *data = container_of(early_sus,
		struct sec_keyboard_drvdata, early_suspend);

	if (data->kl != UNKOWN_KEYLAYOUT) {
		/*
		if the command of the caps lock off is needed,
		this command should be sent.
		sec_keyboard_tx(0xcb);
		msleep(20);
		*/
		sec_keyboard_tx(0x10);	/* the idle mode */
	}
}

static void keyboard_late_resume(struct early_suspend *early_sus)
{
	struct sec_keyboard_drvdata *data = container_of(early_sus,
		struct sec_keyboard_drvdata, early_suspend);

	if (data->kl != UNKOWN_KEYLAYOUT)
		pr_info("[Keyboard] %s", __func__);

}
#endif

static int sec_keyboard_event(struct input_dev *dev,
			unsigned int type, unsigned int code, int value)
{
	/*
	struct sec_keyboard_drvdata *data = input_get_drvdata(dev);
	*/

	switch (type) {
	case EV_LED:
		if (value)
			sec_keyboard_tx(0xca);
		else
			sec_keyboard_tx(0xcb);
		return 0;
	}
	return -1;
}

static ssize_t check_keyboard_connection(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct sec_keyboard_drvdata *ddata = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", ddata->dockconnected);
}

static DEVICE_ATTR(attached, S_IRUGO, check_keyboard_connection, NULL);

static struct attribute *sec_keyboard_attributes[] = {
	&dev_attr_attached.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = sec_keyboard_attributes,
};

static int __devinit sec_keyboard_probe(struct platform_device *pdev)
{
	struct sec_keyboard_platform_data *pdata = pdev->dev.platform_data;
	struct sec_keyboard_drvdata *ddata;
	struct input_dev *input;
	int i, error;

	ddata = kzalloc(sizeof(struct sec_keyboard_drvdata), GFP_KERNEL);
	if (NULL == ddata) {
		error = -ENOMEM;
		goto err_free_mem;
	}

	input = input_allocate_device();
	if (NULL == input) {
		printk(KERN_ERR "[Keyboard] Fail to allocate input device.\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	ddata->input_dev = input;
	ddata->acc_power = pdata->acc_power;
	ddata->check_uart_path = pdata->check_uart_path;
	ddata->acc_int_gpio = pdata->accessory_irq_gpio;
	ddata->buf_front = ddata->buf_rear = 0;
	ddata->led_on = false;
	ddata->dockconnected = false;
	ddata->pre_connected = false;
	ddata->remap_key = 0;
	ddata->kl = UNKOWN_KEYLAYOUT;
	ddata->callbacks.check_keyboard_dock = check_keyboard_dock;
	if (pdata->register_cb)
		pdata->register_cb(&ddata->callbacks);

	memcpy(ddata->keycode, sec_keycodes, sizeof(sec_keycodes));

	mutex_init(&ddata->mutex);
	INIT_WORK(&ddata->work_msg, key_event_work);
	INIT_WORK(&ddata->work_timer, timer_work);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;
	input->id.bustype = BUS_RS232;
	input->event = sec_keyboard_event;

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_LED, input->evbit);
	set_bit(LED_CAPSL, input->ledbit);
	/* framework doesn't use repeat event */
	/* set_bit(EV_REP, input->evbit); */

	for (i = 0; i < KEYBOARD_SIZE; i++) {
		if (KEY_RESERVED != ddata->keycode[i])
			input_set_capability(input, EV_KEY, ddata->keycode[i]);
	}

	/* for the UK keyboard */
	input_set_capability(input, EV_KEY, KEY_NUMERIC_POUND);

	/* for the remaped keys */
	input_set_capability(input, EV_KEY, KEY_NEXTSONG);
	input_set_capability(input, EV_KEY, KEY_PREVIOUSSONG);

	/* for the wakeup key */
	input_set_capability(input, EV_KEY, KEY_WAKEUP);

	error = input_register_device(input);
	if (error < 0) {
		printk(KERN_ERR "[Keyboard] Fail to register input device.\n");
		error = -ENOMEM;
		goto err_input_allocate_device;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ddata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ddata->early_suspend.suspend = keyboard_early_suspend;
	ddata->early_suspend.resume = keyboard_late_resume;
	register_early_suspend(&ddata->early_suspend);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	init_timer(&ddata->timer);
	ddata->timer.expires = jiffies + HZ;
	ddata->timer.function = keyboard_timer;
	ddata->timer.data = (unsigned long)ddata;

	init_timer(&ddata->key_timer);
	ddata->key_timer.expires = jiffies + HZ/3;
	ddata->key_timer.function = remapkey_timer;
	ddata->key_timer.data = (unsigned long)ddata;

	ddata->keyboard_dev = device_create(sec_class, NULL, 0,
		ddata, "sec_keyboard");
	if (IS_ERR(ddata->keyboard_dev)) {
		pr_err("Failed to create device for the sysfs\n");
		error = -ENODEV;
		goto err_sysfs_create_group;
	}

	error = sysfs_create_group(&ddata->keyboard_dev->kobj, &attr_group);
	if (error) {
		pr_err("Failed to create sysfs group\n");
		goto err_sysfs_create_group;
	}

	g_data = ddata;

	return 0;

err_sysfs_create_group:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ddata->early_suspend);
#endif
	del_timer_sync(&ddata->timer);
	del_timer_sync(&ddata->key_timer);
err_input_allocate_device:
	input_free_device(input);
err_free_mem:
	kfree(ddata);
	return error;

}

static int __devexit sec_keyboard_remove(struct platform_device *pdev)
{
	struct sec_keyboard_drvdata *data = platform_get_drvdata(pdev);
	input_unregister_device(data->input_dev);
	return 0;
}

#ifndef CONFIG_HAS_EARLYSUSPEND
static int sec_keyboard_suspend(struct platform_device *pdev,
			pm_message_t state)
{
	struct sec_keyboard_drvdata *data = platform_get_drvdata(pdev);

	if (data->kl != UNKOWN_KEYLAYOUT)
		sec_keyboard_tx(0x10);

	return 0;
}

static int sec_keyboard_resume(struct platform_device *pdev)
{
	struct sec_keyboard_platform_data *pdata = pdev->dev.platform_data;
	struct sec_keyboard_drvdata *data = platform_get_drvdata(pdev);
	int keycode = 0;
	if (pdata->wakeup_key)
		keycode = pdata->wakeup_key();

	if (KEY_WAKEUP == keycode) {
		input_report_key(data->input_dev, keycode, 1);
		input_report_key(data->input_dev, keycode, 0);
		input_sync(data->input_dev);
	}
	return 0;
}
#endif

static struct platform_driver sec_keyboard_driver = {
	.probe = sec_keyboard_probe,
	.remove = __devexit_p(sec_keyboard_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = sec_keyboard_suspend,
	.resume	= sec_keyboard_resume,
#endif
	.driver = {
		.name	= "sec_keyboard",
		.owner	= THIS_MODULE,
	}
};

static int __init sec_keyboard_init(void)
{
	return platform_driver_register(&sec_keyboard_driver);
}

static void __exit sec_keyboard_exit(void)
{
	platform_driver_unregister(&sec_keyboard_driver);
}

module_init(sec_keyboard_init);
module_exit(sec_keyboard_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SEC P series Dock Keyboard driver");
