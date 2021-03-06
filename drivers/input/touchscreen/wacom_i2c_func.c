#include <linux/wacom_i2c.h>
#include "wacom_i2c_flash.h"

/*
#define CONFIG_SAMSUNG_KERNEL_DEBUG_USER
*/

int wacom_i2c_test(struct wacom_i2c *wac_i2c)
{
	int ret, i;
	char buf, test[10];
	buf = COM_QUERY;

	ret = i2c_master_send(wac_i2c->client, &buf, sizeof(buf));
	if (ret > 0)
		pr_info("[E-PEN]: buf:%d, sent:%d\n", buf, ret);
	else {
		pr_err("[E-PEN]: Digitizer is not active\n");
		return -1;
	}

	ret = i2c_master_recv(wac_i2c->client, test, sizeof(test));
	if (ret >= 0) {
		for (i = 0; i < 8; i++)
			pr_info("[E-PEN]: %d\n", test[i]);
	} else {
		pr_err("[E-PEN]: Digitizer does not reply\n");
		return -1;
	}

	return 0;
}

int wacom_i2c_master_send(struct i2c_client *client,
			const char *buf, int count,
			unsigned short addr)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;

	msg.addr = addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

int wacom_i2c_master_recv(struct i2c_client *client, char *buf,
			int count, unsigned short addr)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;

	ret = i2c_transfer(adap, &msg, 1);

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 1) ? count : ret;
}

int wacom_i2c_query(struct wacom_i2c *wac_i2c)
{
	struct wacom_features *wac_feature = wac_i2c->wac_feature;
	struct i2c_msg msg[2];
	int ret = 0, i = 0;
	int query_limit = 10;
	u8 *data;
	u16 buf;

	buf = COM_QUERY;
	data = wac_feature->data;

	msg[0].addr  = wac_i2c->client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &buf;

	msg[1].addr  = wac_i2c->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = COM_QUERY_NUM;
	msg[1].buf   = (u8 *) data;

	do {
		ret = i2c_transfer(wac_i2c->client->adapter, msg, 2);
		pr_info("[E-PEN]: %dth ret of wacom query=%d\n", i, ret);

		i++;
		if (ret < 0)
			continue;

		wac_feature->fw_version = ((u16)data[7] << 8) + (u16)data[8];
		if ((wac_feature->fw_version != 0xFF) &&
			(wac_feature->fw_version != 0xFFFF)) {
			wac_feature->x_max = ((u16)data[1] << 8) + (u16)data[2];
			wac_feature->y_max = ((u16)data[3] << 8) + (u16)data[4];
			wac_feature->pressure_max = ((u16)data[5] << 8) + (u16)data[6];
			break;
		} else {
			pr_info("[E-PEN]: %X, %X, %X, %X, %X, %X, %X, %X, %X\n",
				data[0], data[1], data[2], data[3], data[4],
				data[5], data[6], data[7], data[8]);
		}

	} while (i < query_limit);

	pr_info("[E-PEN]: x_max=0x%X\n", wac_feature->x_max);
	pr_info("[E-PEN]: y_max=0x%X\n", wac_feature->y_max);
	pr_info("[E-PEN]: pressure_max=0x%X\n", wac_feature->pressure_max);
	pr_info("[E-PEN]: fw_version=0x%X (d7:0x%X,d8:0x%X)\n",
		wac_feature->fw_version, data[7], data[8]);

	return 0;
}

int wacom_i2c_coord(struct wacom_i2c *wac_i2c)
{
	u8 data[COM_COORD_NUM];
	bool prox = false, rubber = false, stylus = false;
	u16 x, y, pressure;
	u16 tmp;

	if (COM_COORD_NUM != i2c_master_recv(wac_i2c->client,
							data, COM_COORD_NUM)) {
		pr_err("[E-PEN] failed to get coordinate.\n");
		return -1;
	}

	/* add the workaround code for the fw bug */
	if (0xff == data[0]) {
		pr_err("[E-PEN] %x, %x, %x, %x, %x, %x, %x\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6]);
		return -1;
	}

	/* check the valid data */
	if (data[0] & 0x80) {
		/* enable emr device */
		if (!wac_i2c->pen_prox) {
			wac_i2c->pen_prox = 1;
			if (data[0] & 0x40)
				wac_i2c->tool = EPEN_TOOL_RUBBER;
			else
				wac_i2c->tool = EPEN_TOOL_PEN;
			printk(KERN_DEBUG "[E-PEN] %s is in\n",
				wac_i2c->tool & 0x1 ? "Rubber" : "Pen");
		}

		prox = !!(data[0] & 0x10);
		stylus = !!(data[0] & 0x20);
		rubber = !!(data[0] & 0x40);

		x = (data[1] << 8) + data[2];
		y = (data[3] << 8) + data[4];
		pressure = ((data[5] & 0x3) << 8) + data[6];

		if (wac_i2c->wac_pdata->x_invert)
			x = wac_i2c->wac_feature->x_max - x;
		if (wac_i2c->wac_pdata->y_invert)
			y = wac_i2c->wac_feature->y_max - y;

		if (wac_i2c->wac_pdata->xy_switch) {
			tmp = x;
			x = y;
			y = tmp;
		}

		if (prox && !wac_i2c->pen_pressed)
			pr_info("[E-PEN] %s is pressed (%d,%d)\n",
				wac_i2c->tool & 0x1 ? "Rubber" : "Pen", x, y);
		else if (!prox && wac_i2c->pen_pressed)
			pr_info("[E-PEN] %s is released(%d,%d)\n",
				wac_i2c->tool & 0x1 ? "Rubber" : "Pen", x, y);

		if (stylus && !wac_i2c->side_pressed)
			pr_info("[E-PEN] side on(%d,%d)\n", x, y);
		else if (!stylus && wac_i2c->side_pressed)
			pr_info("[E-PEN] side off(%d,%d)\n", x, y);

		wac_i2c->pen_pressed = prox;
		wac_i2c->side_pressed = stylus;

		input_report_abs(wac_i2c->input_dev, ABS_X, x);
		input_report_abs(wac_i2c->input_dev, ABS_Y, y);
		input_report_abs(wac_i2c->input_dev, ABS_PRESSURE, pressure);
		input_report_key(wac_i2c->input_dev, EPEN_STYLUS, stylus);
		input_report_key(wac_i2c->input_dev, BTN_TOUCH, prox);
		input_report_key(wac_i2c->input_dev, wac_i2c->tool, 1);
		input_sync(wac_i2c->input_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
		printk(KERN_DEBUG "[E-PEN] (%d, %d) 0x%x\n", x, y, data[0]);
#endif
	} else {
		/* enable emr device */

		if (wac_i2c->pen_prox) {
			/* input_report_abs(wac->input_dev, ABS_X, x); */
			/* input_report_abs(wac->input_dev, ABS_Y, y); */
			input_report_abs(wac_i2c->input_dev, ABS_PRESSURE, 0);
			input_report_key(wac_i2c->input_dev, EPEN_STYLUS, 0);
			input_report_key(wac_i2c->input_dev, BTN_TOUCH, 0);
			input_report_key(wac_i2c->input_dev, wac_i2c->tool, 0);
			input_sync(wac_i2c->input_dev);

			if (wac_i2c->pen_pressed || wac_i2c->side_pressed)
				printk(KERN_DEBUG "[E-PEN] %s is out\n",
					wac_i2c->tool & 0x1 ? "Rubber" : "Pen");

			/* check_emr_device(false); */
			/* unset dvfs level */
		}
		wac_i2c->pen_prox = 0;
		wac_i2c->pen_pressed = 0;
		wac_i2c->side_pressed = 0;
	}
	return 0;
}
