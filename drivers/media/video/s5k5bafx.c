/*
 * Driver for S5K5BAFX from Samsung Electronics
 *
 * 1/6" 2Mp CMOS Image Sensor SoC with an Embedded Image Processor
 *
 * Copyright (c) 2011, Samsung Electronics. All rights reserved
 * Author: dongseong.lim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#ifdef CONFIG_VIDEO_SAMSUNG_V4L2
#include <linux/videodev2_samsung.h>
#endif
#include <media/s5k5bafx_platform.h>
#include "s5k5bafx.h"
#ifdef CONFIG_CPU_FREQ
#include <mach/cpufreq.h>
#endif
#ifdef S5K5BAFX_USLEEP
#include <linux/hrtimer.h>
#endif

static const struct s5k5bafx_fps s5k5bafx_framerates[] = {
	{ I_FPS_0,	FRAME_RATE_AUTO },
	{ I_FPS_7,	FRAME_RATE_7 },
	{ I_FPS_10,	10 },
	{ I_FPS_12,	12 },
	{ I_FPS_15,	FRAME_RATE_15 },
	{ I_FPS_25,	FRAME_RATE_25 },
};

static const struct s5k5bafx_regs reg_datas = {
	.ev = {
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_MINUS_4), s5k5bafx_bright_m4),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_MINUS_3), s5k5bafx_bright_m3),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_MINUS_2), s5k5bafx_bright_m2),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_MINUS_1), s5k5bafx_bright_m1),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_DEFAULT),
						s5k5bafx_bright_default),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_PLUS_1), s5k5bafx_bright_p1),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_PLUS_2), s5k5bafx_bright_p2),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_PLUS_3), s5k5bafx_bright_p3),
		S5K5BAFX_REGSET(GET_EV_INDEX(EV_PLUS_4), s5k5bafx_bright_p4),
	},
	.blur = {
		S5K5BAFX_REGSET(BLUR_LEVEL_0, s5k5bafx_vt_pretty_default),
		S5K5BAFX_REGSET(BLUR_LEVEL_1, s5k5bafx_vt_pretty_1),
		S5K5BAFX_REGSET(BLUR_LEVEL_2, s5k5bafx_vt_pretty_2),
		S5K5BAFX_REGSET(BLUR_LEVEL_3, s5k5bafx_vt_pretty_3),
	},
	.fps = {
		S5K5BAFX_REGSET(I_FPS_0, s5k5bafx_fps_auto),
		S5K5BAFX_REGSET(I_FPS_7, s5k5bafx_fps_7fix),
		S5K5BAFX_REGSET(I_FPS_10, s5k5bafx_fps_10fix),
		S5K5BAFX_REGSET(I_FPS_12, s5k5bafx_fps_12fix),
		S5K5BAFX_REGSET(I_FPS_15, s5k5bafx_fps_15fix),
		S5K5BAFX_REGSET(I_FPS_25, s5k5bafx_fps_25fix),
	},
	.preview_start = S5K5BAFX_REGSET_TABLE(s5k5bafx_preview),
	.capture_start = S5K5BAFX_REGSET_TABLE(s5k5bafx_capture),
	.init = S5K5BAFX_REGSET_TABLE(s5k5bafx_common),
	.init_vt = S5K5BAFX_REGSET_TABLE(s5k5bafx_vt_common),
	.init_vt_wifi = S5K5BAFX_REGSET_TABLE(s5k5bafx_vt_wifi_common),
#if defined(CONFIG_TARGET_LOCALE_KOR) || defined(CONFIG_TARGET_LOCALE_NAATT)
	.init_recording = S5K5BAFX_REGSET_TABLE(s5k5bafx_recording_60Hz_common),
#else
	.init_recording = S5K5BAFX_REGSET_TABLE(s5k5bafx_recording_50Hz_common),
#endif
	.stream_stop = S5K5BAFX_REGSET_TABLE(s5k5bafx_stream_stop),
	.dtp_on = S5K5BAFX_REGSET_TABLE(s5k5bafx_pattern_on),
	.dtp_off = S5K5BAFX_REGSET_TABLE(s5k5bafx_pattern_off),
};

/**
 * Use msleep() if the sleep time is over 1000 us.
 */
static void __used s5k5bafx_usleep(u32 usecs)
{
	ktime_t expires;
	u64 add_time = (u64)usecs * 1000;

	if (unlikely(!usecs))
		return;

	expires = ktime_add_ns(ktime_get(), add_time);
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_hrtimeout(&expires, HRTIMER_MODE_ABS);
}

static inline int s5k5bafx_read(struct i2c_client *client,
	u16 subaddr, u16 *data)
{
	u8 buf[2] = {0,};
	int err = -EIO;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = 2,
		.buf = buf,
	};

	*(u16 *)buf = cpu_to_be16(subaddr);

	err = i2c_transfer(client->adapter, &msg, 1);
	CHECK_ERR_N_MSG(err, "%d register wite fail\n", __LINE__);

	msg.flags = I2C_M_RD;

	err = i2c_transfer(client->adapter, &msg, 1);
	CHECK_ERR_N_MSG(err, "%d register read fail\n", __LINE__);

	*data = ((buf[0] << 8) | buf[1]);

	return 0;
}

/*
 * s5k6aafx sensor i2c write routine
 * <start>--<Device address><2Byte Subaddr><2Byte Value>--<stop>
 */
#ifdef CONFIG_LOAD_FILE
static int loadFile(void)
{
	struct file *fp = NULL;
	struct test *nextBuf = NULL;

	u8 *nBuf = NULL;
	size_t file_size = 0, max_size = 0, testBuf_size = 0;
	ssize_t nread = 0;
	s32 check = 0, starCheck = 0;
	s32 tmp_large_file = 0;
	s32 i = 0;
	int ret = 0;
	loff_t pos;

	mm_segment_t fs = get_fs();
	set_fs(get_ds());

	cam_info("%s: E\n", __func__);

	BUG_ON(testBuf);

	fp = filp_open(TUNING_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		cam_err("%s: ERROR, file open error\n", __func__);
		return PTR_ERR(fp);
	}

	file_size = (size_t) fp->f_path.dentry->d_inode->i_size;
	max_size = file_size;

	cam_dbg("file_size = %d\n", file_size);

	nBuf = kmalloc(file_size, GFP_ATOMIC);
	if (nBuf == NULL) {
		cam_dbg("Fail to 1st get memory\n");
		nBuf = vmalloc(file_size);
		if (nBuf == NULL) {
			cam_err("%s: ERROR, nBuf Out of Memory\n", __func__);
			ret = -ENOMEM;
			goto error_out;
		}
		tmp_large_file = 1;
	}

	testBuf_size = sizeof(struct test) * file_size;
	if (tmp_large_file) {
		testBuf = (struct test *)vmalloc(testBuf_size);
		large_file = 1;
	} else {
		testBuf = kmalloc(testBuf_size, GFP_ATOMIC);
		if (testBuf == NULL) {
			cam_dbg("Fail to get mem(%d bytes)\n", testBuf_size);
			testBuf = (struct test *)vmalloc(testBuf_size);
			large_file = 1;
		}
	}
	if (testBuf == NULL) {
		cam_err("%s: ERROR, Out of Memory\n", __func__);
		ret = -ENOMEM;
		goto error_out;
	}

	pos = 0;
	memset(nBuf, 0, file_size);
	memset(testBuf, 0, file_size * sizeof(struct test));

	nread = vfs_read(fp, (char __user *)nBuf, file_size, &pos);
	if (nread != file_size) {
		cam_err("%s: ERROR, failed to read file ret = %d\n",
						__func__, nread);
		ret = -1;
		goto error_out;
	}

	set_fs(fs);

	i = max_size;

	printk("i = %d\n", i);

	while (i) {
		testBuf[max_size - i].data = *nBuf;
		if (i != 1) {
			testBuf[max_size - i].nextBuf = &testBuf[max_size - i + 1];
		} else {
			testBuf[max_size - i].nextBuf = NULL;
			break;
		}
		i--;
		nBuf++;
	}

	i = max_size;
	nextBuf = &testBuf[0];

#if 1
	while (i - 1) {
		if (!check && !starCheck) {
			if (testBuf[max_size - i].data == '/') {
				if (testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data
								== '/') {
						check = 1;/* when find '//' */
						i--;
					} else if (testBuf[max_size-i].nextBuf->data == '*') {
						starCheck = 1;/* when find '/ *' */
						i--;
					}
				} else
					break;
			}
			if (!check && !starCheck) {
				/* ignore '\t' */
				if (testBuf[max_size - i].data != '\t') {
					nextBuf->nextBuf = &testBuf[max_size-i];
					nextBuf = &testBuf[max_size - i];
				}
			}
		} else if (check && !starCheck) {
			if (testBuf[max_size - i].data == '/') {
				if(testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data == '*') {
						starCheck = 1; /* when find '/ *' */
						check = 0;
						i--;
					}
				} else
					break;
			}

			 /* when find '\n' */
			if (testBuf[max_size - i].data == '\n' && check) {
				check = 0;
				nextBuf->nextBuf = &testBuf[max_size - i];
				nextBuf = &testBuf[max_size - i];
			}

		} else if (!check && starCheck) {
			if (testBuf[max_size - i].data == '*') {
				if (testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data == '/') {
						starCheck = 0; /* when find '* /' */
						i--;
					}
				} else
					break;
			}
		}

		i--;

		if (i < 2) {
			nextBuf = NULL;
			break;
		}

		if (testBuf[max_size - i].nextBuf == NULL) {
			nextBuf = NULL;
			break;
		}
	}
#endif

#if 0 // for print
	printk("i = %d\n", i);
	nextBuf = &testBuf[0];
	while (1) {
		//printk("sdfdsf\n");
		if (nextBuf->nextBuf == NULL)
			break;
		printk("%c", nextBuf->data);
		nextBuf = nextBuf->nextBuf;
	}
#endif

error_out:

	if (nBuf)
		tmp_large_file ? vfree(nBuf) : kfree(nBuf);
	if (fp)
		filp_close(fp, current->files);
	return ret;
}
#endif

static inline int s5k5bafx_write(struct i2c_client *client,
		u32 packet)
{
	u8 buf[4];
	int err = 0, retry_count = 5;

	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.buf	= buf,
		.len	= 4,
	};

	if (!client->adapter) {
		cam_err("%s: ERROR, can't search i2c client adapter\n",
							__func__);
		return -EIO;
	}

	while (retry_count--) {
		*(u32 *)buf = cpu_to_be32(packet);
		err = i2c_transfer(client->adapter, &msg, 1);
		if (likely(err == 1))
			break;
		mdelay(10);
	}
	CHECK_ERR_N_MSG(err, "0x%08x write failed err=%d\n",
					(u32)packet, err)
	return 0;
}

#ifdef CONFIG_LOAD_FILE
/* #define DEBUG_LOAD_FILE */
static int s5k5bafx_write_regs_from_sd(struct v4l2_subdev *sd, u8 s_name[])
{

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct test *tempData = NULL;

	int ret = -EAGAIN;
	u32 temp;
	u32 delay = 0;
	u8 data[11];
	s32 searched = 0;
	size_t size = strlen(s_name);
	s32 i;
#ifdef DEBUG_LOAD_FILE
	u8 regs_name[128] = {0,};

	BUG_ON(size > sizeof(regs_name));
#endif

	cam_dbg("E size = %d, string = %s\n", size, s_name);
	tempData = &testBuf[0];
	while (!searched) {
		searched = 1;
		for (i = 0; i < size; i++) {
			if (tempData->data != s_name[i]) {
				searched = 0;
				break;
			}
#ifdef DEBUG_LOAD_FILE
			regs_name[i] = tempData->data;
#endif

			tempData = tempData->nextBuf;
		}
#ifdef DEBUG_LOAD_FILE
		if (i > 9) {
			regs_name[i] = '\0';
			cam_dbg("Searching: regs_name = %s\n", regs_name);
		}
#endif

		tempData = tempData->nextBuf;
	}
	/* structure is get..*/
#ifdef DEBUG_LOAD_FILE
	regs_name[i] = '\0';
	cam_dbg("Searched regs_name = %s\n\n", regs_name);
#endif

	while (1) {
		if (tempData->data == '{')
			break;
		else
			tempData = tempData->nextBuf;
	}

	while (1) {
		searched = 0;
		while (1) {
			if (tempData->data == 'x') {
				/* get 10 strings.*/
				data[0] = '0';
				for (i = 1; i < 11; i++) {
					data[i] = tempData->data;
					tempData = tempData->nextBuf;
				}
				/*cam_dbg("%s\n", data);*/
				temp = simple_strtoul(data, NULL, 16);
				break;
			} else if (tempData->data == '}') {
				searched = 1;
				break;
			} else
				tempData = tempData->nextBuf;

			if (tempData->nextBuf == NULL)
				return -1;
		}

		if (searched)
			break;

		if ((temp & S5K5BAFX_DELAY) == S5K5BAFX_DELAY) {
			delay = temp & 0xFFFF;
			debug_msleep(delay);
			continue;
		}

		ret = s5k5bafx_write(client, temp);

		/* In error circumstances */
		/* Give second shot */
		if (unlikely(ret)) {
			dev_info(&client->dev,
					"s5k5bafx i2c retry one more time\n");
			ret = s5k5bafx_write(client, temp);

			/* Give it one more shot */
			if (unlikely(ret)) {
				dev_info(&client->dev,
						"s5k5bafx i2c retry twice\n");
				ret = s5k5bafx_write(client, temp);
			}
		}
	}

	return ret;
}
#endif

/*
* Read a register.
*/
static int s5k5bafx_read_reg(struct v4l2_subdev *sd,
		u16 page, u16 addr, u16 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 page_cmd = (0x002C << 16) | page;
	u32 addr_cmd = (0x002E << 16) | addr;
	int err = -EIO;

	/* cam_trace("page_cmd=0x%X, addr_cmd=0x%X\n", page_cmd, addr_cmd); */

	err = s5k5bafx_write(client, page_cmd);
	CHECK_ERR(err);
	err = s5k5bafx_write(client, addr_cmd);
	CHECK_ERR(err);
	err = s5k5bafx_read(client, 0x0F12, val);
	CHECK_ERR(err);

	return 0;
}

/* program multiple registers */
static int s5k5bafx_write_regs(struct v4l2_subdev *sd,
		const u32 *packet, u32 num)
{
#ifdef S5K5BAFX_USLEEP
	struct s5k5bafx_state *state = to_state(sd);
#endif
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -EAGAIN;
	u32 temp = 0;
	u16 delay = 0;
	int retry_count = 5;

#ifdef S5K5BAFX_BURST_MODE
	u16 addr, value;

	int len = 0;
	u8 buf[SZ_2K] = {0,};
#else
	u8 buf[4] = {0,};
#endif
	struct i2c_msg msg = {
		msg.addr = client->addr,
		msg.flags = 0,
		msg.len = 4,
		msg.buf = buf,
	};

	while (num--) {
		temp = *packet++;

		if ((temp & S5K5BAFX_DELAY) == S5K5BAFX_DELAY) {
			delay = temp & 0xFFFF;
			debug_msleep(delay);
			continue;
		}

#ifdef S5K5BAFX_BURST_MODE
		addr = temp >> 16;
		value = temp & 0xFFFF;

		switch (addr) {
		case 0x0F12:
			if (len == 0) {
				buf[len++] = addr >> 8;
				buf[len++] = addr & 0xFF;
			}
			buf[len++] = value >> 8;
			buf[len++] = value & 0xFF;

			if ((*packet >> 16) != addr) {
				msg.len = len;
				goto s5k5bafx_burst_write;
			}
			break;

		case 0xFFFF:
			break;

		default:
			msg.len = 4;
			*(u32 *)buf = cpu_to_be32(temp);
			goto s5k5bafx_burst_write;
		}

		continue;
#else
		*(u32 *)buf = cpu_to_be32(temp);
#endif

#ifdef S5K5BAFX_BURST_MODE
s5k5bafx_burst_write:
		len = 0;
#endif
		retry_count = 5;

		while (retry_count--) {
			ret = i2c_transfer(client->adapter, &msg, 1);
			if (likely(ret == 1))
				break;
			mdelay(10);
		}

		if (unlikely(ret < 0)) {
			cam_err("%s: ERROR, 0x%08x write failed err=%d\n",
						__func__, (u32)packet, ret);
			break;
		}

#ifdef S5K5BAFX_USLEEP
		if (unlikely(state->vt_mode))
			if (!(num%200))
				s5k5bafx_usleep(3);
#endif		
	}

	if (unlikely(ret < 0)) {
		cam_err("%s: ERROR, fail to write registers. err=%d!!\n",
						__func__, ret);
		return -EIO;
	}

	return 0;
}

static int s5k5bafx_set_from_table(struct v4l2_subdev *sd,
				const char *setting_name,
				const struct s5k5bafx_regset_table *table,
				int table_size, int index)
{
	int err = 0;

	cam_dbg("%s: set %s index %d\n",
		__func__, setting_name, index);
	if ((index < 0) || (index >= table_size)) {
		cam_err("%s: ERROR, index(%d) out of range[0:%d]"
			"for table for %s\n", __func__, index,
			table_size, setting_name);
		return -EINVAL;
	}

	table += index;
	if (unlikely(!table->reg)) {
		cam_err("%s: ERROR, reg = NULL\n", __func__);
		return -EFAULT;
	}

#ifdef CONFIG_LOAD_FILE
	cam_dbg("%s: \"%s\", reg_name=%s\n", __func__, setting_name,
						table->name);
	return s5k5bafx_write_regs_from_sd(sd, table->name);
#else
	err = s5k5bafx_write_regs(sd, table->reg, table->array_size);
	if (unlikely(err < 0)) {
		cam_err("%s: ERROR, fail to write regs(%s), err=%d\n",
				__func__, setting_name, err);
		return -EIO;
	}

	return 0;
#endif
}

static inline int s5k5bafx_get_iso(struct v4l2_subdev *sd, u16 *iso)
{
	u16 iso_gain_table[] = {10, 18, 23, 28};
	u16 iso_table[] = {0, 50, 100, 200, 400};
	u16 val = 0, gain = 0;
	int i = 0;

	s5k5bafx_read_reg(sd, REG_PAGE_ISO, REG_ADDR_ISO, &val);
	cam_dbg("val = %d\n", val);
	gain = val * 10 / 256;
	for (i = 0; i < ARRAY_SIZE(iso_gain_table); i++) {
		if (gain < iso_gain_table[i])
			break;
	}

	*iso = iso_table[i];

	cam_dbg("gain=%d, ISO=%d\n", gain, *iso);

	return 0;
}

static inline int  s5k5bafx_get_expousretime(struct v4l2_subdev *sd,
						u32 *exp_time)
{
	u16 val = 0;

	s5k5bafx_read_reg(sd, REG_PAGE_SHUTTER, REG_ADDR_SHUTTER, &val);
	*exp_time = val * 1000 / 500;

	return 0;
}

static int s5k5bafx_get_exif(struct v4l2_subdev *sd)
{
	struct s5k5bafx_state *state = to_state(sd);
	u32 exposure_time = 0;

	state->exif.exp_time_den = 0;
	state->exif.iso = 0;

	/* Get exposure-time */
	s5k5bafx_get_expousretime(sd, &exposure_time);
	state->exif.exp_time_den = 1000 * 1000 / exposure_time;
	cam_dbg("real exposure time=%dms\n", exposure_time / 1000);

	/* Get ISO */
	s5k5bafx_get_iso(sd, &state->exif.iso);

	cam_dbg("%s: exp_time_den=%d, ISO=%d\n",
		__func__, state->exif.exp_time_den, state->exif.iso);

	return 0;
}

#ifdef SUPPORT_FACTORY_TEST
static int s5k5bafx_check_dataline(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EIO;

	cam_info("DTP %s\n", val ? "ON" : "OFF");

	if (val)
		err = s5k5bafx_set_from_table(sd, "dtp_on",
				&state->regs->dtp_on, 1, 0);
	else
		err = s5k5bafx_set_from_table(sd, "dtp_off",
				&state->regs->dtp_off, 1, 0);

	CHECK_ERR_N_MSG(err, "fail to DTP setting\n");
	return 0;
}
#endif

static int s5k5bafx_debug_sensor_status(struct v4l2_subdev *sd)
{
	u16 val = 0;
	int err = -EINVAL;

	/* Read Mon_DBG_Counters_2 */
	/*err = s5k5bafx_read_reg(sd, 0x7000, 0x0402, &val);
	CHECK_ERR(err);
	cam_info("counter = %d\n", val); */

	/* Read REG_TC_GP_EnableCaptureChanged. */
	err = s5k5bafx_read_reg(sd, 0x7000, 0x01F6, &val);
	CHECK_ERR(err);
	
	switch(val) {
	case 0:
		cam_info("In normal mode(0)\n");
		break;
	case 1:
		cam_info("In swiching to capture mode(1).....\n");
		break;
	default:
		cam_err("%s: ERROR, In Unknown mode(?)\n", __func__);
		break;
	}

	return 0;
}

static int s5k5bafx_check_sensor_status(struct v4l2_subdev *sd)
{
	/*struct i2c_client *client = v4l2_get_subdevdata(sd);*/
	u16 val_1 = 0, val_2 = 0;
	int err = -EINVAL;

	err = s5k5bafx_read_reg(sd, 0x7000, 0x0132, &val_1);
	CHECK_ERR(err);
	err = s5k5bafx_read_reg(sd, 0xD000, 0x1002, &val_2);
	CHECK_ERR(err);

	cam_dbg("read val1=0x%x, val2=0x%x\n", val_1, val_2);

	if ((val_1 != 0xAAAA) || (val_2 != 0))
		goto error_occur;

	cam_info("Sensor ESD Check: not detected\n");
	return 0;

error_occur:
	cam_err("%s: ERROR, ESD Shock detected!\n\n", __func__);
	return -ERESTART;
}

static inline int s5k5bafx_check_esd(struct v4l2_subdev *sd)
{
	int err = -EINVAL;

	err = s5k5bafx_check_sensor_status(sd);
	CHECK_ERR(err);

	return 0;	
}

static int s5k5bafx_set_preview_start(struct v4l2_subdev *sd)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_info("set_preview_start\n");

	err = s5k5bafx_set_from_table(sd, "preview_start",
			&state->regs->preview_start, 1, 0);
#ifdef SUPPORT_FACTORY_TEST
	if (state->check_dataline)
		err = s5k5bafx_check_dataline(sd, 1);
#endif
	CHECK_ERR_N_MSG(err, "fail to make preview\n")

	return 0;
}

static int s5k5bafx_set_capture_start(struct v4l2_subdev *sd)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_info("set_capture_start\n");

	err = s5k5bafx_set_from_table(sd, "capture_start",
				&state->regs->capture_start, 1, 0);
	CHECK_ERR_N_MSG(err, "failed to make capture\n");

	s5k5bafx_get_exif(sd);

	return err;
}

static int s5k5bafx_set_sensor_mode(struct v4l2_subdev *sd,
					s32 val)
{
	struct s5k5bafx_state *state = to_state(sd);

	switch (val) {
	case SENSOR_MOVIE:
		if (state->vt_mode) {
			state->sensor_mode = SENSOR_CAMERA;
			cam_warn("%s: WARNING, Not support movie in vt mode\n",
							__func__);
			break;
		}
		/* We do not break. */
	case SENSOR_CAMERA:
		state->sensor_mode = val;
		break;
	default:
		cam_err("%s: ERROR: Not support mode.(%d)\n",
						__func__, val);
		return -EINVAL;
	}

	return 0;
}

static int s5k5bafx_g_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	cam_trace("E\n");
	return 0;
}

static int s5k5bafx_enum_framesizes(struct v4l2_subdev *sd, \
					struct v4l2_frmsizeenum *fsize)
{
	struct s5k5bafx_state *state = to_state(sd);

	cam_trace("E\n");

	/*
	 * Return the actual output settings programmed to the camera
	 */
	if (state->req_fmt.priv == V4L2_PIX_FMT_MODE_CAPTURE) {
		fsize->discrete.width = state->capture_frmsizes.width;
		fsize->discrete.height = state->capture_frmsizes.height;
	} else {
		fsize->discrete.width = state->preview_frmsizes.width;
		fsize->discrete.height = state->preview_frmsizes.height;
	}

	cam_info("enum_framesizes: width - %d , height - %d\n",
		fsize->discrete.width, fsize->discrete.height);

	return 0;
}

#if (0) /* not used */
static int s5k5bafx_enum_fmt(struct v4l2_subdev *sd, struct v4l2_fmtdesc *fmtdesc)
{
	int err = 0;

	FUNC_ENTR();
	return err;
}

static int s5k5bafx_enum_frameintervals(struct v4l2_subdev *sd,
					struct v4l2_frmivalenum *fival)
{
	int err = 0;

	FUNC_ENTR();
	return err;
}
#endif

static int s5k5bafx_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int err = 0;

	cam_trace("E\n");

	return err;
}

static int s5k5bafx_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	struct s5k5bafx_state *state = to_state(sd);
	u32 *width = NULL, *height = NULL;

	cam_trace("E\n");
	/*
	 * Just copying the requested format as of now.
	 * We need to check here what are the formats the camera support, and
	 * set the most appropriate one according to the request from FIMC
	 */
	memcpy(&state->req_fmt, &fmt->fmt.pix, sizeof(fmt->fmt.pix));

	switch (state->req_fmt.priv) {
	case V4L2_PIX_FMT_MODE_PREVIEW:
		width = &state->preview_frmsizes.width;
		height = &state->preview_frmsizes.height;
		break;

	case V4L2_PIX_FMT_MODE_CAPTURE:
		width = &state->capture_frmsizes.width;
		height = &state->capture_frmsizes.height;
		break;

	default:
		cam_err("%s: ERROR, inavlid FMT Mode(%d)\n",
						__func__, state->req_fmt.priv);
		return -EINVAL;
	}

	if ((*width != state->req_fmt.width) ||
		(*height != state->req_fmt.height)) {
		cam_err("%s: ERROR, Invalid size. width= %d, height= %d\n",
			__func__, state->req_fmt.width, state->req_fmt.height);
	}

	return 0;
}

static int s5k5bafx_set_frame_rate(struct v4l2_subdev *sd, u32 fps)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EIO;
	int i = 0, fps_index = -1;

	cam_info("set frame rate %d\n\n", fps);

	if (state->sensor_mode == SENSOR_MOVIE) {
		cam_warn("%s: WARNING, recording mode supports fixed 25 fps.\n",
						__func__);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(s5k5bafx_framerates); i++) {
		if (fps == s5k5bafx_framerates[i].fps) {
			fps_index = s5k5bafx_framerates[i].index;;
			break;
		}
	}

	if (unlikely(fps_index < 0)) {
		cam_err("%s: WARNING, Not supported FPS(%d)\n", __func__, fps);
		return 0;
	}

	err = s5k5bafx_set_from_table(sd, "fps",
				state->regs->fps,
				ARRAY_SIZE(state->regs->fps), fps_index);

	CHECK_ERR_N_MSG(err, "fail to set framerate\n")
	return 0;
}

static int s5k5bafx_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	int err = 0;

	cam_trace("E\n");

	return err;
}

static int s5k5bafx_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	int err = 0;
	u32 fps = 0;
	struct s5k5bafx_state *state = to_state(sd);

	if (!state->vt_mode)
		return 0;

	cam_trace("E\n");

	fps = parms->parm.capture.timeperframe.denominator /
			parms->parm.capture.timeperframe.numerator;

	if (fps != state->set_fps) {
		if (fps < 0 && fps > 15) {
			cam_err("%s: ERROR, invalid frame rate %d\n",
						__func__, fps);
			fps = 15;
		}
		state->req_fps = fps;

		if (state->initialized) {
			err = s5k5bafx_set_frame_rate(sd, state->req_fps);
			if (err >= 0)
				state->set_fps = state->req_fps;
		}

	}

	return err;
}

#if (0) /* not used */
static int s5k5bafx_set_60hz_antibanding(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;

	FUNC_ENTR();

	u32 s5k5bafx_antibanding60hz[] = {
	0xFCFCD000,
	0x00287000,
	// Anti-Flicker //
	// End user init script
	0x002A0400,
	0x0F12005F,  //REG_TC_DBG_AutoAlgEnBits //Auto Anti-Flicker is enabled bit[5] = 1.
	0x002A03DC,
	0x0F120002,  //02 REG_SF_USER_FlickerQuant //Set flicker quantization(0: no AFC, 1: 50Hz, 2: 60 Hz)
	0x0F120001,
	};

	err = s5k5bafx_write_regs(sd, s5k5bafx_antibanding60hz,
				       	sizeof(s5k5bafx_antibanding60hz) / sizeof(s5k5bafx_antibanding60hz[0]));
	printk("%s:  setting 60hz antibanding \n", __func__);
	if (unlikely(err))
	{
		printk("%s: failed to set 60hz antibanding \n", __func__);
		return err;
	}

	return 0;
}
#endif

static int s5k5bafx_control_stream(struct v4l2_subdev *sd, u32 cmd)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;

	if (unlikely(cmd != STREAM_STOP))
		return 0;

	cam_info("STREAM STOP!!\n");
	err = s5k5bafx_set_from_table(sd, "stream_stop",
				&state->regs->stream_stop, 1, 0);

	CHECK_ERR_N_MSG(err, "failed to stop stream\n");
#ifndef CONFIG_VIDEO_IMPROVE_STREAMOFF
	debug_msleep(state->pdata->streamoff_delay);
#endif
	return 0;
}

static int s5k5bafx_init(struct v4l2_subdev *sd, u32 val)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;
	static int lock_level = -1;

	cam_trace("E\n");

#ifdef CONFIG_CPU_FREQ
	if (lock_level < 0)
		lock_level = s5pv310_cpufreq_round_idx(CPUFREQ_1200MHZ);
	if (s5pv310_cpufreq_lock(DVFS_LOCK_ID_CAM, lock_level))
		cam_err("%s: ERROR, failed lock DVFS\n", __func__);
#endif
	/* set initial regster value */
	if (state->sensor_mode == SENSOR_CAMERA) {
		if (!state->vt_mode) {
			cam_info("load camera common setting\n");
			err = s5k5bafx_set_from_table(sd, "init",
				&state->regs->init, 1, 0);
		} else {
			if (state->vt_mode == 1) {
				cam_info("load camera VT call setting\n");
				err = s5k5bafx_set_from_table(sd, "init_vt",
					&state->regs->init_vt, 1, 0);
			} else {
				cam_info("load camera WIFI VT call setting\n");
				err = s5k5bafx_set_from_table(sd,
					"init_vt_wifi",
					&state->regs->init_vt_wifi, 1, 0);
			}
		}
	} else {
		cam_info("load recording setting\n");
		err = s5k5bafx_set_from_table(sd, "init_recording",
					&state->regs->init_recording, 1, 0);
	}
#ifdef CONFIG_CPU_FREQ
	s5pv310_cpufreq_lock_free(DVFS_LOCK_ID_CAM);
#endif
	CHECK_ERR_N_MSG(err, "failed to init. err=%d\n", err);

	/* We stop stream-output from sensor when starting camera. */
	if (likely(state->pdata->is_mipi)) {
		err = s5k5bafx_control_stream(sd, STREAM_STOP);
		CHECK_ERR(err);
	}

	if (state->vt_mode && (state->req_fps != state->set_fps)) {
		err = s5k5bafx_set_frame_rate(sd, state->req_fps);
		CHECK_ERR(err);
		state->set_fps = state->req_fps;
	}

	state->initialized = 1;

	return 0;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize
 * every single opening time therefor,
 * it is not necessary to be initialized on probe time.
 * except for version checking
 * NOTE: version checking is optional
 */
static int s5k5bafx_s_config(struct v4l2_subdev *sd,
		int irq, void *platform_data)
{
	struct s5k5bafx_state *state = to_state(sd);
#ifdef CONFIG_LOAD_FILE
	int err = 0;
#endif

	cam_trace("E\n");

	state->initialized = 0;
	state->req_fps = state->set_fps = 8;
	state->sensor_mode = SENSOR_CAMERA;
	state->regs = &reg_datas;

	if (!platform_data) {
		cam_err("%s: ERROR, no platform data\n", __func__);
		return -ENODEV;
	}
	state->pdata = platform_data;

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */
	if (!(state->pdata->default_width && state->pdata->default_height)) {
		state->default_frmsizes.width = DEFAULT_PREVIEW_WIDTH;
		state->default_frmsizes.height = DEFAULT_PREVIEW_HEIGHT;
	} else {
		state->default_frmsizes.width = state->pdata->default_width;
		state->default_frmsizes.height = state->pdata->default_height;
	}
	
	state->preview_frmsizes.width = state->default_frmsizes.width;
	state->preview_frmsizes.height = state->default_frmsizes.height;
	state->capture_frmsizes.width = DEFAULT_CAPTURE_WIDTH;
	state->capture_frmsizes.height = DEFAULT_CAPTURE_HEIGHT;

	cam_dbg("Default preview_width: %d , preview_height: %d, "
		"capture_width: %d, capture_height: %d",
		state->preview_frmsizes.width, state->preview_frmsizes.height,
		state->capture_frmsizes.width, state->capture_frmsizes.height);

	state->req_fmt.width = state->preview_frmsizes.width;
	state->req_fmt.height = state->preview_frmsizes.height;
	if (!state->pdata->pixelformat)
		state->req_fmt.pixelformat = DEFAULT_FMT;
	else
		state->req_fmt.pixelformat = state->pdata->pixelformat;

#ifdef CONFIG_LOAD_FILE
	err = loadFile();
	CHECK_ERR_N_MSG(err, "failed to load file ERR=%d\n", err)
#endif

	return 0;
}

#if 0
static int s5k5bafx_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	FUNC_ENTR();
	return 0;
}

static int s5k5bafx_querymenu(struct v4l2_subdev *sd, struct v4l2_querymenu *qm)
{
	FUNC_ENTR();
	return 0;
}
#endif

static int s5k5bafx_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k5bafx_state *state = to_state(sd);
	/* struct i2c_client *client = v4l2_get_subdevdata(sd); */
	int err = 0;

	cam_info("s_stream: mode = %d\n", enable);

	switch (enable) {
	case STREAM_MODE_CAM_OFF:
		if (state->sensor_mode == SENSOR_CAMERA) {
#ifdef SUPPORT_FACTORY_TEST
			if (state->check_dataline)
				err = s5k5bafx_check_dataline(sd, 0);
			else
#endif
				if (likely(state->pdata->is_mipi))
					err = s5k5bafx_control_stream(sd,
						STREAM_STOP);
		}
		break;

	case STREAM_MODE_CAM_ON:
		if ((state->sensor_mode == SENSOR_CAMERA)
		    && (state->req_fmt.priv == V4L2_PIX_FMT_MODE_CAPTURE))
			err = s5k5bafx_set_capture_start(sd);
		else
			err = s5k5bafx_set_preview_start(sd);
		break;

	case STREAM_MODE_MOVIE_ON:
		cam_dbg("%s: do nothing(movie on)!!\n", __func__);
		break;

	case STREAM_MODE_MOVIE_OFF:
		cam_dbg("%s: do nothing(movie off)!!\n", __func__);
		break;

	default:
		cam_err("%s: ERROR, Invalid stream mode %d\n",
						__func__, enable);
		err = -EINVAL;
		break;
	}

	CHECK_ERR_N_MSG(err, "stream on(off) fail")

	return 0;
}

static int s5k5bafx_set_exposure(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_info("set_exposure: val=%d\n", val);

#ifdef SUPPORT_FACTORY_TEST
	if (state->check_dataline)
		return 0;
#endif
	if ((val < EV_MINUS_4) || (val >= EV_MAX_V4L2)) {
		cam_err("%s: ERROR, invalid value(%d)\n", __func__, val);
		return -EINVAL;
	}

	err = s5k5bafx_set_from_table(sd, "ev", state->regs->ev,
		ARRAY_SIZE(state->regs->ev), GET_EV_INDEX(val));
	CHECK_ERR_N_MSG(err, "i2c_write for set brightness\n")

	return 0;
}

static int s5k5bafx_set_blur(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_info("set_blur: val=%d\n", val);

#ifdef SUPPORT_FACTORY_TEST
	if (state->check_dataline)
		return 0;
#endif
	if (unlikely(val < BLUR_LEVEL_0 || val >= BLUR_LEVEL_MAX)) {
		cam_err("%s: ERROR, Invalid blur(%d)\n", __func__, val);
		return -EINVAL;
	}

	err = s5k5bafx_set_from_table(sd, "blur", state->regs->blur,
		ARRAY_SIZE(state->regs->blur), val);
	CHECK_ERR_N_MSG(err, "i2c_write for set blur\n")

	return 0;
}

#if (0)
static int s5k5bafx_check_dataline_stop(struct v4l2_subdev *sd)
{
	return 0;
}
#endif

static int s5k5bafx_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct s5k5bafx_state *state = to_state(sd);
	int err = 0;

	cam_dbg("g_ctrl: id = %d\n", ctrl->id - V4L2_CID_PRIVATE_BASE);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_EXIF_EXPTIME:
		ctrl->value = state->exif.exp_time_den;
		break;
	case V4L2_CID_CAMERA_EXIF_ISO:
		ctrl->value = state->exif.iso;
		break;
	default:
		cam_err("%s: ERROR, no such control id %d\n",
			__func__, ctrl->id - V4L2_CID_PRIVATE_BASE);
		break;
	}

	return err;
}

static int s5k5bafx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	/* struct i2c_client *client = v4l2_get_subdevdata(sd); */
	struct s5k5bafx_state *state = to_state(sd);
	int err = 0;

	cam_info("s_ctrl: id = %d, value=%d\n",
		ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);

	if ((ctrl->id != V4L2_CID_CAMERA_CHECK_DATALINE)
	    && (ctrl->id != V4L2_CID_CAMERA_SENSOR_MODE)
	    && ((ctrl->id != V4L2_CID_CAMERA_VT_MODE))
	    && (!state->initialized)) {
		cam_warn("%s: WARNING, camera not initialized\n", __func__);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_BRIGHTNESS:
		err = s5k5bafx_set_exposure(sd, ctrl->value);
		cam_dbg("V4L2_CID_CAMERA_BRIGHTNESS [%d]\n", ctrl->value);
		break;

	case V4L2_CID_CAMERA_VGA_BLUR:
		err = s5k5bafx_set_blur(sd, ctrl->value);
		cam_dbg("V4L2_CID_CAMERA_VGA_BLUR [%d]\n", ctrl->value);
		break;

	case V4L2_CID_CAMERA_VT_MODE:
		state->vt_mode = ctrl->value;
		break;

	case V4L2_CID_CAMERA_SENSOR_MODE:
		err = s5k5bafx_set_sensor_mode(sd, ctrl->value);
		cam_dbg("sensor_mode = %d\n", ctrl->value);
		break;

	case V4L2_CID_CAMERA_CHECK_ESD:
		err = s5k5bafx_check_esd(sd);
		break;

	case V4L2_CID_CAMERA_CHECK_SENSOR_STATUS:
		s5k5bafx_debug_sensor_status(sd);
		err = s5k5bafx_check_sensor_status(sd);
		break;

#ifdef SUPPORT_FACTORY_TEST
	case V4L2_CID_CAMERA_CHECK_DATALINE:
		state->check_dataline = ctrl->value;
		cam_dbg("check_dataline = %d\n", state->check_dataline);
		err = 0;
		break;
#endif

	default:
		cam_err("%s: ERROR, Not supported ctrl-ID(%d)\n",
			__func__, ctrl->id - V4L2_CID_PRIVATE_BASE);
		/* no errors return.*/
		break;
	}

	cam_trace("X\n");
	return 0;
}

static const struct v4l2_subdev_core_ops s5k5bafx_core_ops = {
	.init = s5k5bafx_init,		/* initializing API */
	.s_config = s5k5bafx_s_config,	/* Fetch platform data */
#if 0
	.queryctrl = s5k5bafx_queryctrl,
	.querymenu = s5k5bafx_querymenu,
#endif
	.g_ctrl = s5k5bafx_g_ctrl,
	.s_ctrl = s5k5bafx_s_ctrl,
};

static const struct v4l2_subdev_video_ops s5k5bafx_video_ops = {
	/*.s_crystal_freq = s5k5bafx_s_crystal_freq,*/
	.g_fmt	= s5k5bafx_g_fmt,
	.s_fmt	= s5k5bafx_s_fmt,
	.s_stream = s5k5bafx_s_stream,
	.enum_framesizes = s5k5bafx_enum_framesizes,
	/*.enum_frameintervals = s5k5bafx_enum_frameintervals,*/
	/*.enum_fmt = s5k5bafx_enum_fmt,*/
	.try_fmt = s5k5bafx_try_fmt,
	.g_parm	= s5k5bafx_g_parm,
	.s_parm	= s5k5bafx_s_parm,
};

static const struct v4l2_subdev_ops s5k5bafx_ops = {
	.core = &s5k5bafx_core_ops,
	.video = &s5k5bafx_video_ops,
};

ssize_t s5k5bafx_camera_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *cam_type = "SLSI_S5K5BAFX";
	cam_info("%s\n", __func__);

	return sprintf(buf, "%s\n", cam_type);
}

static DEVICE_ATTR(camera_type, S_IRUGO, s5k5bafx_camera_type_show, NULL);

/*
 * s5k5bafx_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int s5k5bafx_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct s5k5bafx_state *state = NULL;
	struct v4l2_subdev *sd = NULL;

	cam_trace("E\n");

	state = kzalloc(sizeof(struct s5k5bafx_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	sd = &state->sd;
	strcpy(sd->name, S5K5BAFX_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &s5k5bafx_ops);
	if (device_create_file(&client->dev, &dev_attr_camera_type) < 0) {
		cam_warn("%s: WARNING, failed to create device file, %s\n",
				__func__, dev_attr_camera_type.attr.name);
	}

	cam_dbg("%s %s: driver probed!!\n", dev_driver_string(&client->dev),
		dev_name(&client->dev));
	return 0;
}

static int s5k5bafx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5bafx_state *state = to_state(sd);

	cam_trace("E\n");

	state->initialized = 0;

	device_remove_file(&client->dev, &dev_attr_camera_type);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

#ifdef CONFIG_LOAD_FILE
	if (testBuf) {
		large_file ? vfree(testBuf) : kfree(testBuf);
		large_file = 0;
		testBuf = NULL;
	}
#endif
	cam_dbg("%s %s: driver removed!!\n", dev_driver_string(&client->dev),
			dev_name(&client->dev));
	return 0;
}

static const struct i2c_device_id s5k5bafx_id[] = {
	{ S5K5BAFX_DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, s5k5bafx_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = S5K5BAFX_DRIVER_NAME,
	.probe = s5k5bafx_probe,
	.remove = s5k5bafx_remove,
	.id_table = s5k5bafx_id,
};

MODULE_DESCRIPTION("S5K5BAFX ISP driver");
MODULE_AUTHOR("DongSeong Lim<dongseong.lim@samsung.com>");
MODULE_LICENSE("GPL");
