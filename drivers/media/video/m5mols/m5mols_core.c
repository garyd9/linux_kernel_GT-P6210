/*
 * Driver for M-5MOLS 8M Pixel camera sensor with ISP
 *
 * Copyright (C) 2011 Samsung Electronics Co., Ltd
 * Author: HeungJun Kim, riverful.kim@samsung.com
 *
 * Copyright (C) 2009 Samsung Electronics Co., Ltd
 * Author: Dongsoo Nathaniel Kim, dongsoo45.kim@samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/m5mols.h>

#include "m5mols.h"
#include "m5mols_reg.h"

int m5mols_debug;

module_param(m5mols_debug, int, 0644);

#define MOD_NAME		"M5MOLS"
#define M5MOLS_I2C_CHECK_RETRY	50

/* M-5MOLS mode */
static u8 m5mols_reg_mode[] = {
	[MODE_SYSINIT]		= 0x00,
	[MODE_PARMSET]		= 0x01,
	[MODE_MONITOR]		= 0x02,
	[MODE_UNKNOWN]		= 0xff,
};

/* M-5MOLS regulator consumer names */
/* The DEFAULT names of power are referenced with M-5MOLS datasheet. */
static struct regulator_bulk_data supplies[] = {
	{
		/* core power - 1.2v, generally at the M-5MOLS */
		.supply		= "core",
	}, {
		.supply		= "dig_18",	/* digital power 1 - 1.8v */
	}, {
		.supply		= "d_sensor",	/* sensor power 1 - 1.8v */
	}, {
		.supply		= "dig_28",	/* digital power 2 - 2.8v */
	}, {
		.supply		= "a_sensor",	/* analog power */
	}, {
		.supply		= "dig_12",	/* digital power 3 - 1.2v */
	},
};

/* M-5MOLS default format (codes, sizes, preset values) */
static const struct v4l2_mbus_framefmt default_fmt = {
	.width		= 1920,
	.height		= 1080,
	.code		= V4L2_MBUS_FMT_VYUY8_2X8,
	.field		= V4L2_FIELD_NONE,
	.colorspace	= V4L2_COLORSPACE_JPEG,
};

static const struct m5mols_format m5mols_formats[] = {
	{
		.code		= V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
	},
};

static const struct m5mols_resolution m5mols_resolutions[] = {
	/* monitor size */
	{ 0x01, M5MOLS_RES_MON, 128, 96 },	/* SUB-QCIF */
	{ 0x03, M5MOLS_RES_MON, 160, 120 },	/* QQVGA */
	{ 0x05, M5MOLS_RES_MON, 176, 144 },	/* QCIF */
	{ 0x06, M5MOLS_RES_MON, 176, 176 },	/* 176*176 */
	{ 0x08, M5MOLS_RES_MON, 240, 320 },	/* 1 QVGA */
	{ 0x09, M5MOLS_RES_MON, 320, 240 },	/* QVGA */
	{ 0x0c, M5MOLS_RES_MON, 240, 400 },	/* l WQVGA */
	{ 0x0d, M5MOLS_RES_MON, 400, 240 },	/* WQVGA */
	{ 0x0e, M5MOLS_RES_MON, 352, 288 },	/* CIF */
	{ 0x13, M5MOLS_RES_MON, 480, 360 },	/* 480*360 */
	{ 0x15, M5MOLS_RES_MON, 640, 360 },	/* qHD */
	{ 0x17, M5MOLS_RES_MON, 640, 480 },	/* VGA */
	{ 0x18, M5MOLS_RES_MON, 720, 480 },	/* 720x480 */
	{ 0x1a, M5MOLS_RES_MON, 800, 480 },	/* WVGA */
	{ 0x1f, M5MOLS_RES_MON, 800, 600 },	/* SVGA */
	{ 0x21, M5MOLS_RES_MON, 1280, 720 },	/* HD */
	{ 0x25, M5MOLS_RES_MON, 1920, 1080 },	/* 1080p */
	{ 0x29, M5MOLS_RES_MON, 3264, 2448 },	/* 8M (2.63fps@3264*2448) */
	{ 0x30, M5MOLS_RES_MON, 320, 240 },	/* 60fps for slow motion */
	{ 0x31, M5MOLS_RES_MON, 320, 240 },	/* 120fps for slow motion */
	{ 0x39, M5MOLS_RES_MON, 800, 602 },	/* AHS_MON debug */
};

/* M-5MOLS default FPS */
static const struct v4l2_fract default_fps = {
	.numerator		= 1,
	.denominator		= M5MOLS_FPS_AUTO,
};

static u8 m5mols_reg_fps[] = {
	[M5MOLS_FPS_AUTO]	= 0x01,
	[M5MOLS_FPS_10]		= 0x05,
	[M5MOLS_FPS_12]		= 0x04,
	[M5MOLS_FPS_15]		= 0x03,
	[M5MOLS_FPS_20]		= 0x08,
	[M5MOLS_FPS_21]		= 0x09,
	[M5MOLS_FPS_22]		= 0x0a,
	[M5MOLS_FPS_23]		= 0x0b,
	[M5MOLS_FPS_24]		= 0x07,
	[M5MOLS_FPS_30]		= 0x02,
};

static u32 m5mols_swap_byte(u8 *data, enum m5mols_i2c_size size) {
	if (size == I2C_8BIT)
		return *data;
	if (size == I2C_16BIT)
		return be16_to_cpu(*((u16 *)data));
	return be32_to_cpu(*((u32 *)data));
}

/*
 * m5mols_read_reg/m5mols_write_reg - handle sensor's I2C communications.
 *
 * The I2C command packet of M-5MOLS is made up 3 kinds of I2C bytes(category,
 * command, bytes). Reference m5mols.h.
 *
 * The packet is needed 2, when M-5MOLS is read through I2C.
 * The packet is needed 1, when M-5MOLS is written through I2C.
 *
 * I2C packet common order(including both reading/writing)
 *   1st : size (data size + 4)
 *   2nd : READ/WRITE (R - 0x01, W - 0x02)
 *   3rd : Category
 *   4th : Command
 *
 * I2C packet order for READING operation
 *   5th : data real size for reading
 *   And, read another I2C packet again, until data size.
 *
 * I2C packet order for WRITING operation
 *   5th to 8th: an actual data to write
 */

#define M5MOLS_BYTE_READ	0x01
#define M5MOLS_BYTE_WRITE	0x02

int m5mols_read_reg(struct v4l2_subdev *sd, enum m5mols_i2c_size size,
		u8 category, u8 cmd, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg[2];
	u8 wbuf[5], rbuf[I2C_MAX + 1];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (size != I2C_8BIT && size != I2C_16BIT && size != I2C_32BIT)
		return -EINVAL;

	/* 1st I2C operation for writing category & command. */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;		/* 1(cmd size per bytes) + 4 */
	msg[0].buf = wbuf;
	wbuf[0] = 5;		/* same right above this */
	wbuf[1] = M5MOLS_BYTE_READ;
	wbuf[2] = category;
	wbuf[3] = cmd;
	wbuf[4] = size;

	/* 2nd I2C operation for reading data. */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size + 1;
	msg[1].buf = rbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "failed READ[%d] at "
				"cat[%02x] cmd[%02x]\n",
				size, category, cmd);
		return ret;
	}

	*val = m5mols_swap_byte(&rbuf[1], size);
	usleep_range(15000, 20000);	/* must be for stabilization */

	return 0;
}

int m5mols_write_reg(struct v4l2_subdev *sd, enum m5mols_i2c_size size,
		u8 category, u8 cmd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *cdev = &client->dev;
	struct i2c_msg msg[1];
	u8 wbuf[I2C_MAX + 4];
	u32 *buf = (u32 *)&wbuf[4];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (size != I2C_8BIT && size != I2C_16BIT && size != I2C_32BIT) {
		dev_err(cdev, "Wrong data size\n");
		return -EINVAL;
	}

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = size + 4;
	msg->buf = wbuf;
	wbuf[0] = size + 4;
	wbuf[1] = M5MOLS_BYTE_WRITE;
	wbuf[2] = category;
	wbuf[3] = cmd;

	*buf = m5mols_swap_byte((u8 *)&val, size);

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed WRITE[%d] at "
				"cat[%02x] cmd[%02x], ret %d\n",
				size, msg->buf[2], msg->buf[3], ret);
		return ret;
	}

	usleep_range(15000, 20000);	/* must be for stabilization */

	return 0;
}

int m5mols_check_busy(struct v4l2_subdev *sd, u8 category, u8 cmd, u32 value)
{
	u32 busy, i;
	int ret;

	for (i = 0; i < M5MOLS_I2C_CHECK_RETRY; i++) {
		ret = m5mols_read_reg(sd, I2C_8BIT, category, cmd, &busy);
		if (ret < 0)
			return ret;

		if (busy == value)	/* bingo */
			return 0;

		/* must be for stabilization */
		usleep_range(10000, 10000);
	}

	return -EBUSY;
}

/*
 * m5mols_set_mode/backup/restore - change and set mode of M-5MOLS.
 *
 * This driver supports now only 3 modes(System, Monitor, Parameter).
 */
int m5mols_set_mode(struct v4l2_subdev *sd, enum m5mols_mode mode)
{
	struct m5mols_info *info = to_m5mols(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *cdev = &client->dev;
	const char *m5mols_str_mode[] = {
		"System initialization",
		"Parameter setting",
		"Monitor setting",
		"Unknown",
	};
	int ret = 0;

	if (mode < MODE_SYSINIT || mode > MODE_UNKNOWN)
		return -EINVAL;

	ret = i2c_w8_system(sd, CAT0_SYSMODE, m5mols_reg_mode[mode]);
	if (!ret)
		ret = m5mols_check_busy(sd, CAT_SYSTEM, CAT0_SYSMODE,
				m5mols_reg_mode[mode]);
	if (ret < 0)
		return ret;

	info->mode = m5mols_reg_mode[mode];
	dev_dbg(cdev, " mode: %s\n", m5mols_str_mode[mode]);

	return ret;
}

static int m5mols_set_mode_backup(struct v4l2_subdev *sd, enum m5mols_mode mode)
{
	struct m5mols_info *info = to_m5mols(sd);

	info->mode_backup = info->mode;

	return m5mols_set_mode(sd, mode);
}

static int m5mols_set_mode_restore(struct v4l2_subdev *sd)
{
	struct m5mols_info *info = to_m5mols(sd);
	int ret;

	ret = m5mols_set_mode(sd, info->mode_backup);
	if (!ret)
		info->mode = info->mode_backup;

	return ret;
}

/*
 * get_version - get M-5MOLS sensor versions.
 */
static int get_version(struct v4l2_subdev *sd)
{
	struct m5mols_info *info = to_m5mols(sd);
	union {
		struct m5mols_version	ver;
		u8			bytes[10];
	} value;
	int ret, i;

	for (i = CAT0_CUSTOMER_CODE; i <= CAT0_VERSION_AWB_L; i++) {
		ret = i2c_r8_system(sd, i, (u32 *)&value.bytes[i]);
		if (ret)
			return ret;
	}

	info->ver = value.ver;

	info->ver.fw = be16_to_cpu(info->ver.fw);
	info->ver.hw = be16_to_cpu(info->ver.hw);
	info->ver.parm = be16_to_cpu(info->ver.parm);
	info->ver.awb = be16_to_cpu(info->ver.awb);

	return ret;
}

static void m5mols_show_version(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	struct m5mols_info *info = to_m5mols(sd);

	dev_info(dev, "customer code\t0x%02x\n", info->ver.ctm_code);
	dev_info(dev, "project code\t0x%02x\n", info->ver.pj_code);
	dev_info(dev, "firmware version\t0x%04x\n", info->ver.fw);
	dev_info(dev, "hardware version\t0x%04x\n", info->ver.hw);
	dev_info(dev, "parameter version\t0x%04x\n", info->ver.parm);
	dev_info(dev, "AWB version\t0x%04x\n", info->ver.awb); }

/*
 * get_res_preset - find out M-5MOLS register value from requested resolution.
 *
 * @width: requested width
 * @height: requested height
 * @type: requested type of each modes. It supports only monitor mode now.
 */
static int get_res_preset(struct v4l2_subdev *sd, u16 width, u16 height,
		enum m5mols_res_type type)
{
	struct m5mols_info *info = to_m5mols(sd);
	int i;

	for (i = 0; i < ARRAY_SIZE(m5mols_resolutions); i++) {
		if ((m5mols_resolutions[i].type == type) &&
			(m5mols_resolutions[i].width == width) &&
			(m5mols_resolutions[i].height == height))
			break;
	}

	if (i >= ARRAY_SIZE(m5mols_resolutions)) {
		v4l2msg("no matching resolution\n");
		return -EINVAL;
	}

	return m5mols_resolutions[i].value;
}

/*
 * get_fps - calc & check FPS from v4l2_captureparm, if FPS is adequate, set.
 *
 * In M-5MOLS case, the denominator means FPS. The each value of numerator and
 * denominator should not be minus. If numerator is 0, it sets AUTO FPS. If
 * numerator is not 1, it recalculates denominator. After it checks, the
 * denominator is set to timeperframe.denominator, and used by FPS.
 */
static int get_fps(struct v4l2_subdev *sd, struct v4l2_captureparm *parm)
{
	int numerator = parm->timeperframe.numerator;
	int denominator = parm->timeperframe.denominator;

	/* The denominator should be +, except 0. The numerator shoud be +. */
	if (numerator < 0 || denominator <= 0)
		return -EINVAL;

	/* The numerator is 0, return auto fps. */
	if (numerator == 0) {
		parm->timeperframe.denominator = M5MOLS_FPS_AUTO;
		return 0;
	}

	/* calc FPS(not time per frame) per 1 numerator */
	denominator = denominator / numerator;

	if (denominator < M5MOLS_FPS_AUTO || denominator > M5MOLS_FPS_MAX)
		return -EINVAL;

	if (!m5mols_reg_fps[denominator])
		return -EINVAL;

	return 0;
}

static int m5mols_g_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *ffmt)
{
	struct m5mols_info *info = to_m5mols(sd);

	*ffmt = info->fmt;

	return 0;
}

static int m5mols_s_mbus_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *ffmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mols_info *info = to_m5mols(sd);
	int size;
	int ret = -EINVAL;

	dev_dbg(&client->dev, "%s : width : %d, height : %d\n", __func__, ffmt->width,
			ffmt->height);

	/* If user set portrait for preview, it is substitued width width height
	 * unless get_res_preset will fail that M5MOLS did not support
	 * reverse WVGA */
	if (ffmt->width < ffmt->height) {
		int temp;
		temp  = ffmt->width;
		ffmt->width = ffmt->height;
		ffmt->height = temp;
	}
	size = get_res_preset(sd, ffmt->width, ffmt->height, M5MOLS_RES_MON);
	if (size < 0)
		return -EINVAL;
	dev_dbg(&client->dev, "%s : size : %d\n", __func__, size);

	ret = m5mols_set_mode(sd, MODE_PARMSET);
	if (!ret)
		ret = i2c_w8_param(sd, CAT1_MONITOR_SIZE, (u8)size);
	if (!ret) {
		info->fmt = default_fmt;
		info->fmt.width = ffmt->width;
		info->fmt.height = ffmt->height;

		*ffmt = info->fmt;
	}

	return ret;
}

static int m5mols_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
			      enum v4l2_mbus_pixelcode *code)
{
	if (!code || index >= ARRAY_SIZE(m5mols_formats))
		return -EINVAL;

	*code = m5mols_formats[index].code;

	return 0;
}

static int m5mols_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct m5mols_info *info = to_m5mols(sd);
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
			parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe = info->tpf;

	return 0;
}

static int m5mols_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct m5mols_info *info = to_m5mols(sd);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	int ret = -EINVAL;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
			parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	ret = m5mols_set_mode_backup(sd, MODE_PARMSET);
	if (!ret)
		ret = get_fps(sd, cp);	/* set right FPS to denominator. */
	if (!ret)
		ret = i2c_w8_param(sd, CAT1_MONITOR_FPS,
				m5mols_reg_fps[cp->timeperframe.denominator]);
	if (!ret)
		ret = m5mols_set_mode_restore(sd);
	if (!ret) {
		cp->capability = V4L2_CAP_TIMEPERFRAME;
		info->tpf = cp->timeperframe;
	}

	v4l2msg("denominator: %d / numerator: %d.\n",
		cp->timeperframe.denominator, cp->timeperframe.numerator);

	return ret;
}

static int m5mols_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable) {
		if (!is_streaming(sd))
			return m5mols_set_mode(sd, MODE_MONITOR);

		return -EINVAL;
	}

	if (is_streaming(sd))
		return m5mols_set_mode(sd, MODE_PARMSET);

	return -EINVAL;
}

static const struct v4l2_subdev_video_ops m5mols_video_ops = {
	.g_mbus_fmt		= m5mols_g_mbus_fmt,
	.s_mbus_fmt		= m5mols_s_mbus_fmt,
	.enum_mbus_fmt		= m5mols_enum_mbus_fmt,
	.g_parm			= m5mols_g_parm,
	.s_parm			= m5mols_s_parm,
	.s_stream		= m5mols_s_stream,
};

static int m5mols_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	int ret;

	ret = m5mols_set_mode_backup(sd, MODE_PARMSET);
	if (!ret)
		ret = m5mols_set_ctrl(ctrl);
	if (!ret)
		ret = m5mols_set_mode_restore(sd);

	return ret;
}

static const struct v4l2_ctrl_ops m5mols_ctrl_ops = {
	.s_ctrl = m5mols_s_ctrl,
};

/*
 * m5mols_sensor_power - handle sensor power up/down.
 *
 * @enable: If it is true, power up. If is not, power down.
 */
static int m5mols_sensor_power(struct m5mols_info *info, bool enable)
{
	struct v4l2_subdev *sd = &info->sd;
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	int ret;

	if (enable) {
		if (is_powerup(sd))
			return 0;

		/* power-on additional power */
		if (info->pdata->set_power) {
			ret = info->pdata->set_power(&c->dev, 1);
			if (ret)
				return ret;
		}

		ret = regulator_bulk_enable(ARRAY_SIZE(supplies), supplies);
		if (ret)
			return ret;
		gpio_set_value(info->pdata->gpio_rst, info->pdata->enable_rst);
		usleep_range(1000, 1000);
		mdelay(40);
		info->power = true;

	} else {
		if (!is_powerup(sd))
			return 0;

		ret = regulator_bulk_disable(ARRAY_SIZE(supplies), supplies);
		if (ret)
			return ret;

		/* power-off additional power */
		if (info->pdata->set_power) {
			ret = info->pdata->set_power(&c->dev, 0);
			if (ret)
				return ret;
		}

		info->power = false;

		gpio_set_value(info->pdata->gpio_rst, !info->pdata->enable_rst);
		usleep_range(1000, 1000);
	}

	return ret;
}

/*
 * m5mols_sensor_armboot - booting M-5MOLS internal ARM core-controller.
 *
 * It makes to ready M-5MOLS for I2C & MIPI interface. After M-5MOLS powers up,
 * it needed specific I2C command for booting internal ARM core. And then,
 * it must wait about least 500ms referenced by M-5MOLS datasheet.
 */
static int m5mols_sensor_armboot(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mols_info *info = to_m5mols(sd);
	int ret;

	ret = i2c_w8_flash(sd, CATF_CAM_START, true);
	if (!ret)
		return ret;

	msleep(500);
	dev_dbg(&client->dev, "Success M-5MOLS ARM-Booting\n");

	/* after ARM booting, the M-5MOLS state changed Parameter mode. */
	info->mode = MODE_PARMSET;

	ret = get_version(sd);
	if (!ret)
		/* This value 0x02 changed the M-5MOSLS interface using MIPI.
		 * The M-5MOLS camera core supports Parallel interface, but
		 * it still not be testified in this driver. */
		ret = i2c_w8_param(sd, CAT1_DATA_INTERFACE, 0x02);

	m5mols_show_version(sd);

	return ret;
}

/*
 * m5mols_init_controls - initialization using v4l2_ctrl.
 */
static int m5mols_init_controls(struct m5mols_info *info)
{
	struct v4l2_subdev *sd = &info->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 max_ex_mon;
	int ret;

	/* check minimum & maximum of M-5MOLS controls. This value varies with
	 * the M-5MOLS firmware. So, V4L2 control setting should be done after
	 * the state power-up and I2C communication enable. */
	ret = i2c_r16_ae(sd, CAT3_MAX_GAIN_MON, (u32 *)&max_ex_mon);
	if (!ret)
		return ret;

	/* set the controls using v4l2 control frameworks */
	v4l2_ctrl_handler_init(&info->handle, 5);

	info->colorfx = v4l2_ctrl_new_std_menu(&info->handle,
			&m5mols_ctrl_ops, V4L2_CID_COLORFX,
			9, 1, V4L2_COLORFX_NONE);
	info->autoexposure = v4l2_ctrl_new_std_menu(&info->handle,
			&m5mols_ctrl_ops, V4L2_CID_EXPOSURE_AUTO,
			1, 0, V4L2_EXPOSURE_AUTO);
	info->exposure = v4l2_ctrl_new_std(&info->handle,
			&m5mols_ctrl_ops, V4L2_CID_EXPOSURE,
			0, max_ex_mon, 1, (int)max_ex_mon/2);
	info->autowb = v4l2_ctrl_new_std(&info->handle,
			&m5mols_ctrl_ops, V4L2_CID_AUTO_WHITE_BALANCE,
			0, 1, 1, 1);
	info->saturation = v4l2_ctrl_new_std(&info->handle,
			&m5mols_ctrl_ops, V4L2_CID_SATURATION,
			0, 6, 1, 3);

	sd->ctrl_handler = &info->handle;
	if (info->handle.error) {
		dev_err(&client->dev, "Failed to init controls, %d\n", ret);
		v4l2_ctrl_handler_free(&info->handle);
		return info->handle.error;
	}

	v4l2_ctrl_cluster(2, &info->autoexposure);
	v4l2_ctrl_handler_setup(&info->handle);

	return 0;
}

/*
 * m5mols_setup_default - set default size & fps in the monitor mode.
 */
static int m5mols_setup_default(struct v4l2_subdev *sd)
{
	struct m5mols_info *info = to_m5mols(sd);
	int value;
	int ret = -EINVAL;

	value = get_res_preset(sd, default_fmt.width, default_fmt.height,
			M5MOLS_RES_MON);
	if (value >= 0)
		ret = i2c_w8_param(sd, CAT1_MONITOR_SIZE, (u8)value);
	if (!ret)
		ret = i2c_w8_param(sd, CAT1_MONITOR_FPS,
			m5mols_reg_fps[default_fps.denominator]);
	if (!ret)
		ret = m5mols_init_controls(info);
	if (!ret) {
		info->fmt = default_fmt;
		info->tpf = default_fps;

		ret = 0;
	}

	return ret;
}

static int m5mols_s_power(struct v4l2_subdev *sd, int on)
{
	struct m5mols_info *info = to_m5mols(sd);
	int ret;

	if (on) {
		ret = m5mols_sensor_power(info, true);
		if (!ret)
			ret = m5mols_sensor_armboot(sd);
		if (!ret)
			ret = m5mols_setup_default(sd);
	} else {
		ret = m5mols_sensor_power(info, false);
	}

	return ret;
}

static int m5mols_log_status(struct v4l2_subdev *sd)
{
	struct m5mols_info *info = to_m5mols(sd);

	v4l2_ctrl_handler_log_status(&info->handle, sd->name);

	return 0;
}

static const struct v4l2_subdev_core_ops m5mols_core_ops = {
	.s_power		= m5mols_s_power,
	.g_ctrl			= v4l2_subdev_g_ctrl,
	.s_ctrl			= v4l2_subdev_s_ctrl,
	.queryctrl		= v4l2_subdev_queryctrl,
	.querymenu		= v4l2_subdev_querymenu,
	.g_ext_ctrls		= v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls		= v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls		= v4l2_subdev_s_ext_ctrls,
	.log_status		= m5mols_log_status,
};

static const struct v4l2_subdev_ops m5mols_ops = {
	.core	= &m5mols_core_ops,
	.video	= &m5mols_video_ops,
};

static int m5mols_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	const struct m5mols_platform_data *pdata =
		client->dev.platform_data;
	struct m5mols_info *info;
	struct v4l2_subdev *sd;
	int ret = 0;
	if (!pdata) {
		dev_err(&client->dev, "No platform data\n");
		return -EIO;
	}

	if (!gpio_is_valid(pdata->gpio_rst)) {
		dev_err(&client->dev, "No valid RST gpio pin.\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "Failed to allocate info\n");
		return -ENOMEM;
	}

	/* The used things from platform data is gpio_rst, enable_rst, and
	 * set_power(). */
	info->pdata = pdata;

	ret = gpio_request(info->pdata->gpio_rst, "M-5MOLS nRST");
	if (ret) {
		dev_err(&client->dev, "Failed to set gpio, %d\n", ret);
		goto out_gpio;
	}

	gpio_direction_output(info->pdata->gpio_rst, !info->pdata->enable_rst);

	ret = regulator_bulk_get(&client->dev, ARRAY_SIZE(supplies), supplies);
	if (ret) {
		dev_err(&client->dev, "Failed to get regulators, %d\n", ret);
		goto out_reg;
	}

	sd = &info->sd;
	strlcpy(sd->name, MOD_NAME, sizeof(sd->name));
	v4l2_i2c_subdev_init(sd, client, &m5mols_ops);

	return 0;

out_reg:
	regulator_bulk_free(ARRAY_SIZE(supplies), supplies);
out_gpio:
	gpio_free(info->pdata->gpio_rst);
	kfree(info);

	return ret;
}

static int m5mols_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct m5mols_info *info = to_m5mols(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&info->handle);

	regulator_bulk_free(ARRAY_SIZE(supplies), supplies);
	gpio_free(info->pdata->gpio_rst);
	kfree(info);

	return 0;
}

static const struct i2c_device_id m5mols_id[] = {
	{ MOD_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, m5mols_id);

static struct i2c_driver m5mols_i2c_driver = {
	.driver = {
		.name	= MOD_NAME,
	},
	.probe		= m5mols_probe,
	.remove		= m5mols_remove,
	.id_table	= m5mols_id,
};

static int __init m5mols_mod_init(void)
{
	return i2c_add_driver(&m5mols_i2c_driver);
}

static void __exit m5mols_mod_exit(void)
{
	i2c_del_driver(&m5mols_i2c_driver);
}

module_init(m5mols_mod_init);
module_exit(m5mols_mod_exit);

MODULE_AUTHOR("HeungJun Kim <riverful.kim@samsung.com>");
MODULE_AUTHOR("Dongsoo Kim <dongsoo45.kim@samsung.com>");
MODULE_DESCRIPTION("Fujitsu M-5MOLS 8M Pixel camera sensor with ISP driver");
MODULE_LICENSE("GPL");
