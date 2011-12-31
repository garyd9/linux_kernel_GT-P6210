/*
 * Driver for M5MO (5MP Camera) from NEC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <media/v4l2-i2c-drv.h>
#include <media/v4l2-device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/vmalloc.h>
#include <linux/firmware.h>

#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif

//#include <mach/mach-aries.h>
//#include <mach/max8966_flash.h>

#include <media/m5mo_platform.h>
#include "m5mo.h"


#define M5MO_DRIVER_NAME		"M5MO"
//#define SDCARD_FW
#ifdef SDCARD_FW
#define M5MO_FW_PATH			"/sdcard/external_sd/RS_M5LS.bin"
#else /* SDCARD_FW */
#define M5MO_FW_PATH			"m5mo/RS_M5LS.bin"
#endif /* SDCARD_FW */
#define M5MO_FLASH_BASE_ADDR	0x10000000
#define M5MO_INT_RAM_BASE_ADDR	0x68000000

#define M5MO_I2C_RETRY			5
#define M5MO_I2C_VERIFY			100
#define M5MO_ISP_INT_TIMEOUT	1000

#define M5MO_JPEG_MAXSIZE		0x3A0000
#define M5MO_THUMB_MAXSIZE		0xFC00
#define M5MO_POST_MAXSIZE		0xBB800

#define m5mo_readb(c, g, b, v)		m5mo_read(c, 1, g, b, v)
#define m5mo_readw(c, g, b, v)		m5mo_read(c, 2, g, b, v)
#define m5mo_readl(c, g, b, v)		m5mo_read(c, 4, g, b, v)

#define m5mo_writeb(c, g, b, v)		m5mo_write(c, 1, g, b, v)
#define m5mo_writew(c, g, b, v)		m5mo_write(c, 2, g, b, v)
#define m5mo_writel(c, g, b, v)		m5mo_write(c, 4, g, b, v)

#define CHECK_ERR(x)   if ((x) < 0) { \
							cam_err("i2c falied, err %d\n", x); \
							return x; \
						}

static const struct m5mo_frmsizeenum m5mo_pre_frmsizes[] = {
	{ M5MO_PREVIEW_QCIF,	176,		144,		0x05 },
	{ M5MO_PREVIEW_QVGA,	320,		240,		0x09 },
	{ M5MO_PREVIEW_VGA,		640,		480,		0x17 },
	{ M5MO_PREVIEW_D1,		720,		480,		0x18 },
	{ M5MO_PREVIEW_WVGA,	800,		480,		0x1A },
	{ M5MO_PREVIEW_720P,	1280,	720,		0x21 },
	{ M5MO_PREVIEW_1080P,	1920,	1080,	0x25 },
};

static const struct m5mo_frmsizeenum m5mo_cap_frmsizes[] = {
	{ M5MO_CAPTURE_VGA,		640,		480,		0x09 },
	{ M5MO_CAPTURE_WVGA,	800,		480,		0x0A },
	{ M5MO_CAPTURE_W1MP,	1600,	960,		0x17 },
	{ M5MO_CAPTURE_2MP,		1600,	1200,	0x17 },
	{ M5MO_CAPTURE_W2MP,	2048,	1232,	0x1A },
	{ M5MO_CAPTURE_3MP,		2048,	1536,	0x1B },
	{ M5MO_CAPTURE_W4MP,	2560,	1536,	0x1C },
	{ M5MO_CAPTURE_5MP,		2560,	1920,	0x1F },
	{ M5MO_CAPTURE_W7MP,	3264,	1968,	0x22 },
	{ M5MO_CAPTURE_8MP,		3264,	2448,	0x25 },
};

static inline struct m5mo_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct m5mo_state, sd);
}

static int m5mo_read(struct i2c_client *c,
	u8 len, u8 category, u8 byte, int *val)
{
	struct i2c_msg msg;
	unsigned char data[5];
	unsigned char recv_data[len + 1];
	int i, err = 0;

	if (!c->adapter)
		return -ENODEV;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = msg.len;
	data[1] = 0x01;			/* Read category parameters */
	data[2] = category;
	data[3] = byte;
	data[4] = len;

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		cam_err("category %#x, byte %#x\n", category, byte);
		return err;
	}

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for(i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		cam_err("category %#x, byte %#x\n", category, byte);
		return err;
	}

	if (recv_data[0] != sizeof(recv_data))
		cam_warn("expected length %d, but return length %d\n",
				 sizeof(recv_data), recv_data[0]);

	if (len == 0x01)
		*val = recv_data[1];
	else if (len == 0x02)
		*val = recv_data[1] << 8 | recv_data[2];
	else
		*val = recv_data[1] << 24 | recv_data[2] << 16 |
				recv_data[3] << 8 | recv_data[4];

	cam_dbg("category %#02x, byte %#x, value %#x\n", category, byte, *val);
	return err;
}

static int m5mo_write(struct i2c_client *c,
	u8 len, u8 category, u8 byte, int val)
{
	struct i2c_msg msg;
	unsigned char data[len + 4];
	int i, err;

	if (!c->adapter)
		return -ENODEV;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = 0x02;			/* Write category parameters */
	data[2] = category;
	data[3] = byte;
	if (len == 0x01) {
		data[4] = val & 0xFF;
	} else if (len == 0x02) {
		data[4] = (val >> 8) & 0xFF;
		data[5] = val & 0xFF;
	} else {
		data[4] = (val >> 24) & 0xFF;
		data[5] = (val >> 16) & 0xFF;
		data[6] = (val >> 8) & 0xFF;
		data[7] = val & 0xFF;
	}

	cam_dbg("category %#x, byte %#x, value %#x\n", category, byte, val);

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}

static int m5mo_mem_read(struct i2c_client *c, u16 len, u32 addr, u8 *val)
{
	struct i2c_msg msg;
	unsigned char data[8];
	unsigned char recv_data[len + 3];
	int i, err = 0;
	u16 recv_len;

	if (!c->adapter)
		return -ENODEV;

	if (len <= 0)
		return -EINVAL;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x03;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for(i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	if (recv_len != (recv_data[0] << 8 | recv_data[1]))
		cam_warn("expected length %d, but return length %d\n",
				 len, recv_len);

	memcpy(val, recv_data + 1 + sizeof(recv_len), len);

	return err;
}

static int m5mo_mem_write(struct i2c_client *c, u16 len, u32 addr, u8 *val)
{
	struct i2c_msg msg;
	unsigned char data[len + 8];
	int i, err = 0;

	if (!c->adapter)
		return -ENODEV;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x04;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;
	memcpy(data + 2 + sizeof(addr) + sizeof(len), val, len);

	for(i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}

static inline void m5mo_clear_interrupt(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int int_factor;

	state->isp.issued = 0;
	m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_INT_FACTOR, &int_factor);
}

static irqreturn_t m5mo_isp_isr(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_id;
	struct m5mo_state *state = to_state(sd);

	cam_dbg("**************** interrupt ****************\n");
	if (!state->isp.issued) {
		wake_up_interruptible(&state->isp.wait);
		state->isp.issued = 1;
	}

	return IRQ_HANDLED;
}

static u32 m5mo_wait_interrupt(struct v4l2_subdev *sd,
	unsigned int timeout)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	u32 int_factor = 0;

	if (wait_event_interruptible_timeout(state->isp.wait,
		state->isp.issued == 1,
		msecs_to_jiffies(timeout)) == 0) {
		//msecs_to_jiffies(const unsigned int m)(timeout)) == 0) {
		cam_err("timeout\n");
		return 0;
	}

	m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_INT_FACTOR, &int_factor);

	return int_factor;
}

static int m5mo_set_mode(struct i2c_client *client, u32 mode, u32 *old_mode)
{
	int i, err;
	u32 val;

	err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, &val);
	if (err < 0)
		return err;

	if (old_mode)
		*old_mode = val;

	if (val == mode)
		return 0;

	switch (val) {
	case M5MO_SYSINIT_MODE:
		cam_warn("sensor is initializing\n");
		err = -EBUSY;
		break;

	case M5MO_PARMSET_MODE:
		if (mode == M5MO_STILLCAP_MODE) {
			err = m5mo_writeb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, M5MO_MONITOR_MODE);
			if (err < 0)
				return err;
			for (i = M5MO_I2C_VERIFY; i; i--) {
				err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, &val);
				if (val == M5MO_MONITOR_MODE)
					break;
				msleep(10);
			}
		}
	case M5MO_MONITOR_MODE:
	case M5MO_STILLCAP_MODE:
		err = m5mo_writeb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, mode);
		break;

	default:
		cam_warn("current mode is unknown\n");
		err = -EINVAL;
	}

	if (err < 0)
		return err;

	for (i = M5MO_I2C_VERIFY; i; i--) {
		err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, &val);
		if (val == mode)
			break;
		msleep(10);
	}
	return err;
}

/*
 * v4l2_subdev_video_ops
 */
static const struct m5mo_frmsizeenum *m5mo_get_frmsize(const struct m5mo_frmsizeenum *frmsizes,
	int num_entries, int index)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (frmsizes[i].index == index)
			return &frmsizes[i];
	}

	return NULL;
}

static int m5mo_set_frmsize(struct v4l2_subdev *sd)
{
	struct m5mo_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;

	if (state->colorspace != V4L2_COLORSPACE_JPEG) {
		err = m5mo_writeb(client, M5MO_CATEGORY_PARM, M5MO_PARM_MON_SIZE,
							state->frmsize->reg_val);
		CHECK_ERR(err);
	} else {
		err = m5mo_writeb(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_MAIN_IMG_SIZE,
							state->frmsize->reg_val);
		CHECK_ERR(err);
	}
	cam_info("frame size %dx%d\n", state->frmsize->width, state->frmsize->height);
	return 0;
}

static int m5mo_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *f)
{
	struct m5mo_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, num_entries, err;
	u32 old_index;

	state->colorspace = f->fmt.pix.colorspace;

	old_index = state->frmsize ? state->frmsize->index : -1;
	state->frmsize = NULL;

	if (state->colorspace != V4L2_COLORSPACE_JPEG) {
		num_entries = sizeof(m5mo_pre_frmsizes)/sizeof(struct m5mo_frmsizeenum);
		if (f->fmt.pix.width == 800 && f->fmt.pix.height == 450) {
			state->frmsize = m5mo_get_frmsize(m5mo_pre_frmsizes, num_entries,
												M5MO_PREVIEW_1080P);
		} else if (f->fmt.pix.width == 800 && f->fmt.pix.height == 448) {
			state->frmsize = m5mo_get_frmsize(m5mo_pre_frmsizes, num_entries,
												M5MO_PREVIEW_720P);
		} else {
			for (i = 0; i < num_entries; i++) {
				if (f->fmt.pix.width == m5mo_pre_frmsizes[i].width &&
					f->fmt.pix.height == m5mo_pre_frmsizes[i].height) {
					state->frmsize = &m5mo_pre_frmsizes[i];
				}
			}
		}
	} else {
		num_entries = sizeof(m5mo_cap_frmsizes)/sizeof(struct m5mo_frmsizeenum);
		for (i = 0; i < num_entries; i++) {
			if (f->fmt.pix.width == m5mo_cap_frmsizes[i].width &&
				f->fmt.pix.height == m5mo_cap_frmsizes[i].height) {
				state->frmsize = &m5mo_cap_frmsizes[i];
			}
		}
	}

	if (state->frmsize == NULL) {
		cam_warn("invalid frame size %dx%d\n", f->fmt.pix.width, f->fmt.pix.height);
		state->frmsize = state->colorspace != V4L2_COLORSPACE_JPEG ?
			m5mo_get_frmsize(m5mo_pre_frmsizes, num_entries, M5MO_PREVIEW_VGA) :
			m5mo_get_frmsize(m5mo_cap_frmsizes, num_entries, M5MO_CAPTURE_3MP);
	}

	if (state->initialized) {
		if (old_index != state->frmsize->index) {
			if (state->colorspace != V4L2_COLORSPACE_JPEG) {
				err = m5mo_set_mode(client, M5MO_PARMSET_MODE, NULL);
				CHECK_ERR(err);
			}
			m5mo_set_frmsize(sd);
		}
	}

	return 0;
}

static int m5mo_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct m5mo_state *state = to_state(sd);

	a->parm.capture.timeperframe.numerator = 1;
	a->parm.capture.timeperframe.denominator = state->fps;

	return 0;
}

static int m5mo_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct m5mo_state *state = to_state(sd);

	u32 new_fps = a->parm.capture.timeperframe.denominator /
					a->parm.capture.timeperframe.numerator;

	if (new_fps != state->fps) {
		if(new_fps <= 0 || new_fps > 30) {
			cam_err("invalid frame rate %d\n", new_fps);
			new_fps = 30;
		}
		state->fps = new_fps;
	}

	return 0;
}

static int m5mo_enum_framesizes(struct v4l2_subdev *sd,
	struct v4l2_frmsizeenum *fsize)
{
	struct m5mo_state *state = to_state(sd);

	if (state->frmsize == NULL || state->frmsize->index < 0)
		return -EINVAL;

	/* The camera interface should read this value, this is the resolution
 	 * at which the sensor would provide framedata to the camera i/f
 	 *
 	 * In case of image capture, this returns the default camera resolution (VGA)
 	 */
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = state->frmsize->width;
	fsize->discrete.height = state->frmsize->height;

	return 0;
}

static int m5mo_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct m5mo_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 int_factor;

	cam_info("%s", enable ? "on" : "off");

	if (enable) {
		m5mo_clear_interrupt(sd);

		if (state->colorspace != V4L2_COLORSPACE_JPEG) {
			err = m5mo_writeb(client, M5MO_CATEGORY_SYS, M5MO_SYS_INT_EN, M5MO_INT_MODE);
			CHECK_ERR(err);

			err = m5mo_set_mode(client, M5MO_MONITOR_MODE, NULL);
			if (err <= 0) {
				cam_err("m5mo_set_mode failed\n");
				return err;
			}

			int_factor = m5mo_wait_interrupt(sd, M5MO_ISP_INT_TIMEOUT);
			if (!(int_factor & M5MO_INT_MODE)) {
				cam_err("M5MO_INT_MODE isn't issued, %#x\n", int_factor);
				return -ETIMEDOUT;
			}
		} else {
			if (state->flash_mode == 3) {
//				if (HWREV >= SEINE_HWREV_UNIV03)
//					MAX8966_flash_led_en(1);
				err = m5mo_writeb(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_FLASH_CTRL, 0x01);
				CHECK_ERR(err);
				err = m5mo_writew(client, M5MO_CATEGORY_AE, M5MO_AE_ONESHOT_MAX_EXP, 0x320);
				CHECK_ERR(err);
			}

			err = m5mo_writeb(client, M5MO_CATEGORY_SYS, M5MO_SYS_INT_EN, M5MO_INT_CAPTURE);
			CHECK_ERR(err);

			err = m5mo_set_mode(client, M5MO_STILLCAP_MODE, NULL);
			if (err <= 0) {
				cam_err("m5mo_set_mode failed\n");
				return err;
			}

			int_factor = m5mo_wait_interrupt(sd, M5MO_ISP_INT_TIMEOUT);
			if (!(int_factor & M5MO_INT_CAPTURE)) {
				cam_err("M5MO_INT_CAPTURE isn't issued, %#x\n", int_factor);
				return -ETIMEDOUT;
			}

			err = m5mo_writeb(client, M5MO_CATEGORY_CAPCTRL,
								M5MO_CAPCTRL_FRM_SEL, 0x01);
			CHECK_ERR(err);
		}
	}

	return 0;
}

/*
 * v4l2_subdev_core_ops
 */
static int m5mo_get_af_status(struct v4l2_subdev *sd,
	struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 val = 0;
	int err;

	err = m5mo_readb(client, M5MO_CATEGORY_LENS, M5MO_LENS_AF_STATUS, &val);
	CHECK_ERR(err);

	ctrl->value = val;
	return 0;
}

static int m5mo_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m5mo_state *state = to_state(sd);
	int err = 0;

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		err = m5mo_get_af_status(sd, ctrl);
		break;

	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ctrl->value = M5MO_JPEG_MAXSIZE + M5MO_THUMB_MAXSIZE + M5MO_POST_MAXSIZE;
		break;

	case V4L2_CID_CAM_JPEG_MAIN_SIZE:
		ctrl->value = state->jpeg.main_size;
		break;

	case V4L2_CID_CAM_JPEG_MAIN_OFFSET:
		ctrl->value = state->jpeg.main_offset;
		break;

	case V4L2_CID_CAM_JPEG_THUMB_SIZE:
		ctrl->value = state->jpeg.thumb_size;
		break;

	case V4L2_CID_CAM_JPEG_THUMB_OFFSET:
		ctrl->value = state->jpeg.thumb_offset;
		break;

	case V4L2_CID_CAM_JPEG_POSTVIEW_OFFSET:
		ctrl->value = state->jpeg.postview_offset;
		break;

	case V4L2_CID_CAMERA_EXIF_EXPTIME:
		ctrl->value = state->exif.exptime;
		break;

	case V4L2_CID_CAMERA_EXIF_FLASH:
		ctrl->value = state->exif.flash;
		break;

	case V4L2_CID_CAMERA_EXIF_ISO:
		ctrl->value = state->exif.iso;
		break;

	case V4L2_CID_CAMERA_EXIF_TV:
		ctrl->value = state->exif.tv;
		break;

	case V4L2_CID_CAMERA_EXIF_BV:
		ctrl->value = state->exif.bv;
		break;

	case V4L2_CID_CAMERA_EXIF_EBV:
		ctrl->value = state->exif.ebv;
		break;

	default:
		cam_err("no such control id %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE);
		//err = -ENOIOCTLCMD
		err = 0;
		break;
	}

	if (err < 0 && err != -ENOIOCTLCMD)
		cam_err("failed, id %d\n", ctrl->id - V4L2_CID_PRIVATE_BASE);

	return err;
}

static int m5mo_set_sensor_mode(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = 0;
	u32 hdmovie, fps;

	if (val == 0) {	/* Camera */
		hdmovie = 0x00;
		fps = 0x01;
	} else {		/* Camcorder */
		hdmovie = 0x01;
		fps = 0x02;
	}

	err = m5mo_set_mode(client, M5MO_PARMSET_MODE, NULL);
	CHECK_ERR(err);

	err = m5mo_writeb(client, M5MO_CATEGORY_PARM, M5MO_PARM_HDMOVIE, hdmovie);
	CHECK_ERR(err);
	err = m5mo_writeb(client, M5MO_CATEGORY_PARM, M5MO_PARM_MON_FPS, fps);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_exposure(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int offset, err;
	u32 reg_vals[] = {0x00, 0x07, 0x0F, 0x16, 0x1E, 0x25, 0x2D, 0x34, 0x3C};

	if (val < M5MO_EXPOSURE_MIN || val > M5MO_EXPOSURE_MAX) {
		cam_warn("invalied value, %d\n", val);
		val = M5MO_EXPOSURE_DEF;
	}

	offset = -M5MO_EXPOSURE_MIN;
	val += offset;

	err = m5mo_writeb(client, M5MO_CATEGORY_AE, M5MO_AE_EV_BIAS, reg_vals[val]);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_contrast(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int offset, err;
	u32 reg_vals[] = {0x06, 0x05, 0x02, 0x04, 0x03};

	if (val < M5MO_CONTRAST_MIN || val > M5MO_CONTRAST_MAX) {
		cam_warn("invalied value, %d\n", val);
		val = M5MO_CONTRAST_DEF;
	}

	offset = -M5MO_CONTRAST_MIN;
	val += offset;

	err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_TONE_CTRL, reg_vals[val]);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_saturation(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int offset, err;
	u32 reg_vals[] = {0x01, 0x02, 0x03, 0x04, 0x05};

	if (val < M5MO_SATURATION_MIN || val > M5MO_SATURATION_MAX) {
		cam_warn("invalied value, %d\n", val);
		val = M5MO_SATURATION_DEF;
	}

	offset = -M5MO_SATURATION_MIN;
	val += offset;

	err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_CHROMA_LVL, reg_vals[val]);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_sharpness(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int offset, err;
	u32 sharpness[] = {0x06, 0x05, 0x02, 0x04, 0x03};

	if (val < M5MO_SHARPNESS_MIN || val > M5MO_SHARPNESS_MAX) {
		cam_warn("invalied value, %d\n", val);
		val = M5MO_SHARPNESS_DEF;
	}

	offset = -M5MO_SHARPNESS_MIN;
	val += offset;

	err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_EDGE_LVL, sharpness[val]);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_metering(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;

	switch (val) {
	case METERING_CENTER:
		err = m5mo_writeb(client, M5MO_CATEGORY_AE, M5MO_AE_MODE, 0x03);
		CHECK_ERR(err);
		break;
	case METERING_SPOT:
		err = m5mo_writeb(client, M5MO_CATEGORY_AE, M5MO_AE_MODE, 0x06);
		CHECK_ERR(err);
		break;
	case METERING_MATRIX:
		err = m5mo_writeb(client, M5MO_CATEGORY_AE, M5MO_AE_MODE, 0x02);
		CHECK_ERR(err);
		break;
	default:
		cam_warn("invalid value, %d\n", val);
		err = m5mo_writeb(client, M5MO_CATEGORY_AE, M5MO_AE_MODE, 0x03);
		CHECK_ERR(err);
	}
	return 0;
}

static int m5mo_set_whitebalance(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;

	switch (val) {
	case WHITE_BALANCE_AUTO:
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MODE, 0x01);
		CHECK_ERR(err);
		break;

	case WHITE_BALANCE_SUNNY:
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MODE, 0x02);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MANUAL, 0x04);
		CHECK_ERR(err);
		break;

	case WHITE_BALANCE_CLOUDY:
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MODE, 0x02);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MANUAL, 0x05);
		CHECK_ERR(err);
		break;

	case WHITE_BALANCE_TUNGSTEN:
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MODE, 0x02);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MANUAL, 0x01);
		CHECK_ERR(err);
		break;

	case WHITE_BALANCE_FLUORESCENT:
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MODE, 0x02);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MANUAL, 0x02);
		CHECK_ERR(err);
		break;

	default:
		cam_warn("invalid value, %d\n", val);
		err = m5mo_writeb(client, M5MO_CATEGORY_WB, M5MO_WB_AWB_MODE, 0x01);
		CHECK_ERR(err);
		break;
	}

	return 0;
}

static int m5mo_set_effect_color(struct i2c_client *c, int val)
{
	int on, err;

	err = m5mo_readb(c, M5MO_CATEGORY_PARM, M5MO_PARM_EFFECT, &on);
	CHECK_ERR(err);
	if (on)	{
		err = m5mo_writeb(c, M5MO_CATEGORY_PARM, M5MO_PARM_EFFECT, 0);
		CHECK_ERR(err);
	}

	switch (val) {
	case IMAGE_EFFECT_NONE:
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_COLOR_EFFECT, 0x00);
		CHECK_ERR(err);
		break;

	case IMAGE_EFFECT_SEPIA:
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_COLOR_EFFECT, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_CFIXB, 0xD8);
		CHECK_ERR(err);
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_CFIXR, 0x18);
		CHECK_ERR(err);
		break;

	case IMAGE_EFFECT_BNW:
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_COLOR_EFFECT, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_CFIXB, 0x00);
		CHECK_ERR(err);
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_CFIXR, 0x00);
		CHECK_ERR(err);
		break;

	case IMAGE_EFFECT_ANTIQUE:
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_COLOR_EFFECT, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_CFIXB, 0xD0);
		CHECK_ERR(err);
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_CFIXR, 0x30);
		CHECK_ERR(err);
		break;
	}

	return 0;
}

static int m5mo_set_effect_gamma(struct i2c_client *c, s32 val)
{
	int on, err;

	err = m5mo_readb(c, M5MO_CATEGORY_MON, M5MO_MON_COLOR_EFFECT, &on);
	CHECK_ERR(err);
	if (on) {
		err = m5mo_writeb(c, M5MO_CATEGORY_MON, M5MO_MON_COLOR_EFFECT, 0);
		CHECK_ERR(err);
	}

	switch (val) {
	case IMAGE_EFFECT_NEGATIVE:
		err = m5mo_writeb(c, M5MO_CATEGORY_PARM, M5MO_PARM_EFFECT, 0x01);
		CHECK_ERR(err);
		break;

	case IMAGE_EFFECT_AQUA:
		err = m5mo_writeb(c, M5MO_CATEGORY_PARM, M5MO_PARM_EFFECT, 0x08);
		CHECK_ERR(err);
		break;
	}

	return err;
}

static int m5mo_set_effect(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int old_mode, err;

	err = m5mo_set_mode(client, M5MO_PARMSET_MODE, &old_mode);
	CHECK_ERR(err);

	switch (val) {
	case IMAGE_EFFECT_NONE:
	case IMAGE_EFFECT_BNW:
	case IMAGE_EFFECT_SEPIA:
	case IMAGE_EFFECT_ANTIQUE:
	case IMAGE_EFFECT_SHARPEN:
		err = m5mo_set_effect_color(client, val);
		break;

	case IMAGE_EFFECT_AQUA:
	case IMAGE_EFFECT_NEGATIVE:
		err = m5mo_set_effect_gamma(client, val);
		break;

	default:
		cam_warn("invalid value, %d\n", val);
		err = m5mo_set_effect_color(client, IMAGE_EFFECT_NONE);
		break;
	}
	CHECK_ERR(err);

	err = m5mo_set_mode(client, old_mode, NULL);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_iso(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u32 iso[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};

	if (val < ISO_AUTO || val > ISO_1600) {
		cam_warn("invalied value, %d\n", val);
		val = ISO_AUTO;
	}

	err = m5mo_writeb(client, M5MO_CATEGORY_AE, M5MO_AE_ISOSEL, iso[val]);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_af_mode(struct v4l2_subdev *sd, int val)
{
	struct m5mo_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 mode, status = 0;
	int i, err;

	switch (val) {
	case FOCUS_MODE_MACRO:
	case FOCUS_MODE_MACRO_DEFAULT:
		mode = 0x01;
		break;

	case FOCUS_MODE_CONTINOUS:
		mode = 0x02;
		break;

	case FOCUS_MODE_FD:
	case FOCUS_MODE_FD_DEFAULT:
		mode = 0x03;
		break;

	case FOCUS_MODE_TOUCH:
		mode = 0x04;
		break;

	case FOCUS_MODE_AUTO:
	case FOCUS_MODE_AUTO_DEFAULT:
	default:
		mode = 0x00;
		break;
	}

	if (state->focus.mode == mode)
		return 0;

	if (mode == 0x03) {	/* enable face detection */
		err = m5mo_writeb(client, M5MO_CATEGORY_FD, M5MO_FD_SIZE, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_FD, M5MO_FD_MAX, 0x0B);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_FD, M5MO_FD_CTL, 0x11);
		CHECK_ERR(err);
		msleep(10);
	} else if (state->focus.mode == 0x03) {	/* disable face detection */
		err = m5mo_writeb(client, M5MO_CATEGORY_FD, M5MO_FD_CTL, 0x00);
		CHECK_ERR(err);
	}

	state->focus.mode = mode;

	err = m5mo_writeb(client, M5MO_CATEGORY_LENS, M5MO_LENS_AF_MODE, mode);
	CHECK_ERR(err);

	for (i = M5MO_I2C_VERIFY; i; i--) {
		msleep(10);
		err = m5mo_readb(client, M5MO_CATEGORY_LENS, M5MO_LENS_AF_STATUS, &status);
		CHECK_ERR(err);

		if ((status & 0x01) == 0x00)
			break;
	}

	if ((status & 0x01) != 0x00) {
		cam_err("failed\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int m5mo_set_af(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int err;

	cam_info("%s, mode %#x\n", val ? "start" : "stop", state->focus.mode);

	if (val) {
		if (state->focus.mode != FOCUS_MODE_CONTINOUS)
			err = m5mo_writeb(client, M5MO_CATEGORY_LENS, M5MO_LENS_AF_START, 0x01);
		else
			err = m5mo_writeb(client, M5MO_CATEGORY_LENS, M5MO_LENS_AF_START, 0x02);
	} else {
		err = m5mo_writeb(client, M5MO_CATEGORY_LENS, M5MO_LENS_AF_START, 0x00);
	}

	CHECK_ERR(err);

	return err;
}

static int m5mo_set_touch_auto_focus(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int err;

	if (val) {
		err = m5mo_set_af_mode(sd, FOCUS_MODE_TOUCH);
		if (err < 0) {
			cam_err("m5mo_set_af_mode failed\n");
			return err;
		}
		err = m5mo_writew(client, M5MO_CATEGORY_LENS,
				M5MO_LENS_AF_TOUCH_POSX, state->focus.pos_x);
		CHECK_ERR(err);
		err = m5mo_writew(client, M5MO_CATEGORY_LENS,
				M5MO_LENS_AF_TOUCH_POSY, state->focus.pos_y);
		CHECK_ERR(err);
	}
	err = m5mo_set_af(sd, val);
	if (err < 0)
		cam_err("m5mo_set_af failed\n");

	return err;
}

static int m5mo_set_continous_af(struct v4l2_subdev *sd, int val)
{
	int err;

	if (val) {
		err = m5mo_set_af_mode(sd, FOCUS_MODE_CONTINOUS);
		if (err < 0) {
			cam_err("m5mo_set_af_mode failed\n");
			return err;
		}
	}
	err = m5mo_set_af(sd, val);
	if (err < 0)
		cam_err("m5mo_set_af failed\n");

	return err;
}

static int m5mo_set_zoom(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int zoom[31] = { 1, 4, 6, 8, 10, 13, 15, 17, 19, 22, 24, 26, 28, 31, 33, 35,
					 37, 40, 42, 44, 46, 49, 51, 53, 55, 58, 60, 62, 64, 67, 69};
	int err;

	if (val < 0 || val > sizeof(zoom)/sizeof(int)) {
		cam_warn("invalid value, %d\n", val);
		val = 0;
	}

	err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_ZOOM, zoom[val]);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_set_jpeg_quality(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;

	if (val < 1 || val > 100) {
		cam_warn("invalid value, %d\n", val);
		val = 100;
	}

	err = m5mo_writeb(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_JPEG_RATIO, val);
	CHECK_ERR(err);

	return 0;
}

static int m5mo_start_capture(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int err, int_factor;
	int num, den;

	m5mo_clear_interrupt(sd);

	err = m5mo_writeb(client, M5MO_CATEGORY_CAPCTRL, M5MO_CAPCTRL_TRANSFER, 0x01);
	int_factor = m5mo_wait_interrupt(sd, M5MO_ISP_INT_TIMEOUT);
	if (!(int_factor & M5MO_INT_CAPTURE)) {
		cam_warn("M5MO_INT_CAPTURE isn't issued\n");
		return -ETIMEDOUT;
	}

	err = m5mo_readl(client, M5MO_CATEGORY_CAPCTRL, M5MO_CAPCTRL_IMG_SIZE,
				&state->jpeg.main_size);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_CAPCTRL, M5MO_CAPCTRL_THUMB_SIZE,
				&state->jpeg.thumb_size);
	CHECK_ERR(err);

	state->jpeg.main_offset = 0;
	state->jpeg.thumb_offset = M5MO_JPEG_MAXSIZE;
	state->jpeg.postview_offset = M5MO_JPEG_MAXSIZE + M5MO_THUMB_MAXSIZE;

	/* EXIF */
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EXPTIME_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EXPTIME_DEN, &den);
	CHECK_ERR(err);
	state->exif.exptime = (u32)num*1000000/den;

	err = m5mo_readw(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_FLASH, &num);
	CHECK_ERR(err);
	state->exif.flash = (u16)num;

	err = m5mo_readw(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_ISO, &num);
	CHECK_ERR(err);
	state->exif.iso = (u16)num;

	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_TV_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_TV_DEN, &den);
	CHECK_ERR(err);
	state->exif.tv = num/den;

	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_BV_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_BV_DEN, &den);
	CHECK_ERR(err);
	state->exif.bv = num/den;

	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EBV_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EBV_DEN, &den);
	CHECK_ERR(err);
	state->exif.ebv = num/den;

//	if (HWREV >= SEINE_HWREV_UNIV03)
//		MAX8966_flash_led_en(0);

	return err;
}

static int m5mo_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct m5mo_state *state = to_state(sd);
	int err = 0;

	cam_dbg("id %d, value %d\n", ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_SENSOR_MODE:
		if (state->initialized)
			err = m5mo_set_sensor_mode(sd, ctrl->value);
		state->sensor_mode = ctrl->value;
		break;

	case V4L2_CID_CAMERA_FLASH_MODE:
		state->flash_mode = ctrl->value;
		break;

	case V4L2_CID_CAMERA_BRIGHTNESS:
		if (state->initialized)
			err = m5mo_set_exposure(sd, ctrl->value);
		state->exposure = ctrl->value;
		break;

	case V4L2_CID_CAMERA_CONTRAST:
		if (state->initialized)
			err = m5mo_set_contrast(sd, ctrl->value);
		state->contrast = ctrl->value;
		break;

	case V4L2_CID_CAMERA_SATURATION:
		if (state->initialized)
			err = m5mo_set_saturation(sd, ctrl->value);
		state->saturation = ctrl->value;
		break;

	case V4L2_CID_CAMERA_SHARPNESS:
		if (state->initialized)
			err = m5mo_set_sharpness(sd, ctrl->value);
		state->sharpness = ctrl->value;
		break;

	case V4L2_CID_CAMERA_METERING:
		if (state->initialized)
			err = m5mo_set_metering(sd, ctrl->value);
		state->metering = ctrl->value;
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:
		if (state->initialized)
			err = m5mo_set_whitebalance(sd, ctrl->value);
		state->white_balance = ctrl->value;
		break;

	case V4L2_CID_CAMERA_EFFECT:
		if (state->initialized)
			err = m5mo_set_effect(sd, ctrl->value);
		state->effect = ctrl->value;
		break;

	case V4L2_CID_CAMERA_ISO:
		if (state->initialized)
			err = m5mo_set_iso(sd, ctrl->value);
		state->iso = ctrl->value;
		break;

	case V4L2_CID_CAMERA_FOCUS_MODE:
		err = m5mo_set_af_mode(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		err = m5mo_set_af(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_X:
		state->focus.pos_x = ctrl->value;
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
		state->focus.pos_y = ctrl->value;
		break;

	case V4L2_CID_CAMERA_TOUCH_AF_START_STOP:
		err = m5mo_set_touch_auto_focus(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_CAF_START_STOP:
		err = m5mo_set_continous_af(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_ZOOM:
		err = m5mo_set_zoom(sd, ctrl->value);
		break;

	case V4L2_CID_CAM_JPEG_QUALITY:
		if (state->initialized)
			err = m5mo_set_jpeg_quality(sd, ctrl->value);
		state->jpeg.quality = ctrl->value;
		break;

	case V4L2_CID_CAMERA_CAPTURE:
		err = m5mo_start_capture(sd, ctrl->value);
		break;

	default:
		cam_err("no such control id %d, value %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);
		// err = -ENOIOCTLCMD;
		err = 0;
		break;
	}

	if (err < 0 && err != -ENOIOCTLCMD)
		cam_err("failed, id %d, value %d\n",
				ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);
	return err;
}

static int
m5mo_program_fw(struct i2c_client *c, u8 *buf, u32 addr, u32 unit, u32 count)
{
	u32 val;
	u32 intram_unit = 0x1000;
	int i, j, retries, err = 0;

	for (i = 0; i < count; i++) {
		/* Set Flash ROM memory address */
		err = m5mo_writel(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ADDR, addr);
		CHECK_ERR(err);

		/* Erase FLASH ROM entire memory */
		err = m5mo_writeb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ERASE, 0x01);
		CHECK_ERR(err);
		/* Response while sector-erase is operating */
		retries = 0;
		do {
			mdelay(10);
			err = m5mo_readb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ERASE, &val);
			CHECK_ERR(err);
		} while (val && retries++ < M5MO_I2C_VERIFY);

		/* Set FLASH ROM programming size */
		err = m5mo_writew(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_BYTE,
									unit == SZ_64K ? 0 : unit);
		CHECK_ERR(err);

		/* Clear M-5MoLS internal RAM */
		err = m5mo_writeb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_RAM_CLEAR, 0x01);
		CHECK_ERR(err);

		/* Set Flash ROM programming address */
		err = m5mo_writel(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ADDR, addr);
		CHECK_ERR(err);

		/* Send programmed firmware */
		for (j = 0; j < unit; j += intram_unit) {
			err = m5mo_mem_write(c, intram_unit, M5MO_INT_RAM_BASE_ADDR + j,
													buf + (i * unit) + j);
			CHECK_ERR(err);
			mdelay(10);
		}

		/* Start Programming */
		err = m5mo_writeb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_WR, 0x01);
		CHECK_ERR(err);

		/* Confirm programming has been completed */
		retries = 0;
		do {
			mdelay(10);
			err = m5mo_readb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_WR, &val);
			CHECK_ERR(err);
		} while (val && retries++ < M5MO_I2C_VERIFY);

		/* Increase Flash ROM memory address */
		addr += unit;
	}
	return 0;
}

static int m5mo_load_fw(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 *buf, val;
	int err;

#ifdef SDCARD_FW
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;
	loff_t fpos = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M5MO_FW_PATH, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		cam_err("failed to open %s\n", M5MO_FW_PATH);
		return -ENOENT;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;
	cam_info("start, file path %s, size %ld Bytes\n", M5MO_FW_PATH, fsize);

	buf = vmalloc(fsize);
	if (!buf) {
		cam_err("failed to allocate memory\n");
		return -ENOMEM;
	}

	nread = vfs_read(fp, (char __user *)buf, fsize, &fpos);
	if (nread != fsize) {
		cam_err("failed to read firmware file, nread %ld Bytes\n", nread);
		return -EIO;
	}
#else /* SDCARD_FW */
	struct device *dev = &client->adapter->dev;
	const struct firmware *fentry;

	err = request_firmware(&fentry, M5MO_FW_PATH, dev);
	if (err != 0) {
		cam_err("request_firmware falied\n");
		return -EINVAL;
	}

	buf = (u8 *)fentry->data;
#endif /* SDCARD_FW */

	/* set pin */
	val = 0x7E;
	err = m5mo_mem_write(client, sizeof(val), 0x50000308, &val);
	CHECK_ERR(err);
	/* select flash memory */
	err = m5mo_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_SEL, 0x01);
	CHECK_ERR(err);
	/* program FLSH ROM */
	err = m5mo_program_fw(client, buf, M5MO_FLASH_BASE_ADDR, SZ_64K, 31);
	CHECK_ERR(err);
	err = m5mo_program_fw(client, buf, M5MO_FLASH_BASE_ADDR + SZ_64K * 31, SZ_8K, 4);
	CHECK_ERR(err);

	cam_info("end\n");

#ifdef SDCARD_FW
	vfree(buf);
	filp_close(fp, current->files);
	set_fs(old_fs);
#endif  /* SDCARD_FW */

	return 0;
}

static int m5mo_check_version(struct i2c_client *client)
{
	u32 ver_chip, ver_fw, ver_hw, ver_param, ver_awb;
	m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_PJT_CODE, &ver_chip);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_FW, &ver_fw);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_HW, &ver_hw);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_PARAM, &ver_param);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_AWB, &ver_awb);

	cam_info("****************************************\n");
	cam_info("Chip\tF/W\tH/W\tParam\tAWB\n");
	cam_info("----------------------------------------\n");
	cam_info("%#02x\t%#04x\t%#04x\t%#04x\t%#04x\n",
			 ver_chip, ver_fw, ver_hw, ver_param, ver_awb);
	cam_info("****************************************\n");

	return 0;
}

static int m5mo_init_param(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int err;

	err = m5mo_writeb(client, M5MO_CATEGORY_PARM, M5MO_PARM_OUT_SEL, 0x02);
	CHECK_ERR(err);

	err = m5mo_set_frmsize(sd);
	CHECK_ERR(err);

	err = m5mo_set_sensor_mode(sd, state->sensor_mode);
	CHECK_ERR(err);

	err = m5mo_writeb(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_YUVOUT_MAIN, 0x20);
	CHECK_ERR(err);

	err = m5mo_writel(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_THUMB_JPEG_MAX, M5MO_THUMB_MAXSIZE);
	CHECK_ERR(err);

	err = m5mo_set_jpeg_quality(sd, state->jpeg.quality);
	CHECK_ERR(err);

//	if (HWREV == SEINE_HWREV_UNIV02 || HWREV == SEINE_HWREV_UNIV03) {
		err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_MON_REVERSE, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_MON_MIRROR, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_SHOT_REVERSE, 0x01);
		CHECK_ERR(err);
		err = m5mo_writeb(client, M5MO_CATEGORY_MON, M5MO_MON_SHOT_MIRROR, 0x01);
		CHECK_ERR(err);
//	}

	return 0;
}

static int m5mo_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	u32 int_factor;
	int err;

	m5mo_clear_interrupt(sd);

	/* start camera program(parallel FLASH ROM) */
	err = m5mo_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_CAM_START, 0x01);
	CHECK_ERR(err);

	int_factor = m5mo_wait_interrupt(sd, M5MO_ISP_INT_TIMEOUT);
	if (!(int_factor & M5MO_INT_MODE)) {
		cam_err("firmware was erased?\n");
		return -ETIMEDOUT;
	}

	/* check up F/W version */
	err = m5mo_check_version(client);
	CHECK_ERR(err);

	m5mo_init_param(sd);

	state->initialized = 1;

	return 0;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize every single opening time therefor,
 * it is not necessary to be initialized on probe time. except for version checking
 * NOTE: version checking is optional
 */
static int m5mo_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct m5mo_state *state = to_state(sd);
	struct m5mo_platform_data *pdata = (struct m5mo_platform_data *)platform_data;
	int err = 0;

	/* Default state values */
	state->initialized = 0;
	state->sensor_mode = 0;	/* camera */
	state->isp.issued = 1;

	state->colorspace = 0;
	state->frmsize = NULL;
	state->fps = 0;			/* auto */
	state->focus.mode = 0;

	state->jpeg.main_size = 0;
	state->jpeg.main_offset = 0;
	state->jpeg.postview_offset = 0;

	/* Register ISP irq */
	if (!pdata) {
		cam_err("no platform data\n");
		return -ENODEV;
	}

	if (irq) {
		if (pdata->config_isp_irq)
			pdata->config_isp_irq();

 		err = request_irq(irq, m5mo_isp_isr, IRQF_TRIGGER_RISING, "m5mo isp", sd);
		if (err) {
			cam_err("failed to request irq\n");
			return err;
		}
		state->isp.irq = irq;

		/* wait queue initialize */
		init_waitqueue_head(&state->isp.wait);
	}

	return 0;
}


static const struct v4l2_subdev_core_ops m5mo_core_ops = {
	.s_config = m5mo_s_config,	/* Fetch platform data */
	.init = m5mo_init,			/* initializing API */
	.load_fw = m5mo_load_fw,
	.g_ctrl = m5mo_g_ctrl,
	.s_ctrl = m5mo_s_ctrl,
};

static const struct v4l2_subdev_video_ops m5mo_video_ops = {
	.s_fmt = m5mo_s_fmt,
	.g_parm = m5mo_g_parm,
	.s_parm = m5mo_s_parm,
	.enum_framesizes = m5mo_enum_framesizes,
	.s_stream = m5mo_s_stream,
};

static const struct v4l2_subdev_ops m5mo_ops = {
	.core = &m5mo_core_ops,
	.video = &m5mo_video_ops,
};

/*
 * m5mo_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int m5mo_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct m5mo_state *state;
	struct v4l2_subdev *sd;

	state = kzalloc(sizeof(struct m5mo_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, M5MO_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &m5mo_ops);

	return 0;
}

static int m5mo_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct m5mo_state *state = to_state(sd);

	state->initialized = 0;

	if (state->isp.irq > 0)
	free_irq(state->isp.irq, sd);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

	return 0;
}

static const struct i2c_device_id m5mo_id[] = {
	{ M5MO_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, m5mo_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = M5MO_DRIVER_NAME,
	.probe = m5mo_probe,
	.remove = m5mo_remove,
	.id_table = m5mo_id,
};

MODULE_DESCRIPTION("Fujitsu M5MO LS 8MP ISP driver");
MODULE_AUTHOR("BestIQ <@samsung.com>");
MODULE_LICENSE("GPL");
