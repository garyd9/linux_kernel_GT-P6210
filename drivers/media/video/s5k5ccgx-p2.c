/* drivers/media/video/s5k5ccgx.c
 *
 * Copyright (c) 2010, Samsung Electronics. All rights reserved
 * Author: dongseong.lim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/vmalloc.h>
#include <linux/completion.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-i2c-drv.h>
#include <media/s5k5ccgx_platform.h>
#include <linux/videodev2_samsung.h>
#if defined(CONFIG_VIDEO_S5K5CCGX_P8)
#include "s5k5ccgx-p8.h"
#elif defined(CONFIG_VIDEO_S5K5CCGX_P4W)
#include "s5k5ccgx-p4w.h"
#elif defined(CONFIG_VIDEO_S5K5CCGX_P2)
#include "s5k5ccgx-p2.h"
#else
#include "s5k5ccgx.h"
#endif

static int s5k5ccgx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl);

static const struct s5k5ccgx_fps s5k5ccgx_framerates[] = {
	{ I_FPS_0,	FRAME_RATE_AUTO },
	{ I_FPS_15,	FRAME_RATE_15 },
	{ I_FPS_25,	FRAME_RATE_25 },
	{ I_FPS_30,	FRAME_RATE_30 },
};

static const struct s5k5ccgx_framesize s5k5ccgx_preview_frmsizes[] = {
	{ S5K5CCGX_PREVIEW_QCIF,	176,  144 },
	{ S5K5CCGX_PREVIEW_320x240,	320,  240 },
	{ S5K5CCGX_PREVIEW_CIF,		352,  288 },
	{ S5K5CCGX_PREVIEW_528x432,	528,  432 },
	{ S5K5CCGX_PREVIEW_VGA,		640,  480 },
	{ S5K5CCGX_PREVIEW_D1,		720,  480 },
	{ S5K5CCGX_PREVIEW_736x552,	736,  552 },
	{ S5K5CCGX_PREVIEW_SVGA,	800,  600 },
#ifdef CONFIG_VIDEO_S5K5CCGX_P2
	{ S5K5CCGX_PREVIEW_1024x552,	1024, 552 },
#else
	{ S5K5CCGX_PREVIEW_1024x576,	1024, 576 },
#endif
	/*{ S5K5CCGX_PREVIEW_1024x616,	1024, 616 },*/
	{ S5K5CCGX_PREVIEW_XGA,		1024, 768 },
	{ S5K5CCGX_PREVIEW_PVGA,	1280, 720 },
};

static const struct s5k5ccgx_framesize s5k5ccgx_capture_frmsizes[] = {
	{ S5K5CCGX_CAPTURE_VGA,		640,  480 },
#ifdef CONFIG_VIDEO_S5K5CCGX_P2
	{ S5K5CCGX_CAPTURE_W2MP,	2048, 1104 },
#else
	{ S5K5CCGX_CAPTURE_W2MP,	2048, 1152 },
#endif
	{ S5K5CCGX_CAPTURE_3MP,		2048, 1536 },
};

static struct s5k5ccgx_control s5k5ccgx_ctrls[] = {
	S5K5CCGX_INIT_CONTROL(V4L2_CID_CAMERA_FLASH_MODE, \
					FLASH_MODE_OFF),

	S5K5CCGX_INIT_CONTROL(V4L2_CID_CAMERA_BRIGHTNESS, \
					EV_DEFAULT),

	S5K5CCGX_INIT_CONTROL(V4L2_CID_CAMERA_METERING, \
					METERING_MATRIX),

	S5K5CCGX_INIT_CONTROL(V4L2_CID_CAMERA_WHITE_BALANCE, \
					WHITE_BALANCE_AUTO),

	S5K5CCGX_INIT_CONTROL(V4L2_CID_CAMERA_EFFECT, \
					IMAGE_EFFECT_NONE),
};

static const struct s5k5ccgx_regs reg_datas = {
	.ev = {
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_MINUS_4),
					s5k5ccgx_brightness_m_4),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_MINUS_3),
					s5k5ccgx_brightness_m_3),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_MINUS_2),
					s5k5ccgx_brightness_m_2),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_MINUS_1),
					s5k5ccgx_brightness_m_1),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_DEFAULT),
					s5k5ccgx_brightness_0),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_PLUS_1),
					s5k5ccgx_brightness_p_1),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_PLUS_2),
					s5k5ccgx_brightness_p_2),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_PLUS_3),
					s5k5ccgx_brightness_p_3),
		S5K5CCGX_REGSET(GET_EV_INDEX(EV_PLUS_4),
					s5k5ccgx_brightness_p_4),
	},
	.metering = {
		S5K5CCGX_REGSET(METERING_MATRIX, s5k5ccgx_metering_normal),
		S5K5CCGX_REGSET(METERING_CENTER, s5k5ccgx_metering_center),
		S5K5CCGX_REGSET(METERING_SPOT, s5k5ccgx_metering_spot),
	},
	.iso = {
		S5K5CCGX_REGSET(ISO_AUTO, s5k5ccgx_iso_auto),
		S5K5CCGX_REGSET(ISO_100, s5k5ccgx_iso_100),
		S5K5CCGX_REGSET(ISO_200, s5k5ccgx_iso_200),
		S5K5CCGX_REGSET(ISO_400, s5k5ccgx_iso_400),
	},
	.effect = {
		S5K5CCGX_REGSET(IMAGE_EFFECT_NONE, s5k5ccgx_effect_off),
		S5K5CCGX_REGSET(IMAGE_EFFECT_BNW, s5k5ccgx_effect_mono),
		S5K5CCGX_REGSET(IMAGE_EFFECT_SEPIA, s5k5ccgx_effect_sepia),
		S5K5CCGX_REGSET(IMAGE_EFFECT_NEGATIVE,
				s5k5ccgx_effect_negative),
	},
	.white_balance = {
		S5K5CCGX_REGSET(WHITE_BALANCE_AUTO, s5k5ccgx_wb_auto),
		S5K5CCGX_REGSET(WHITE_BALANCE_SUNNY, s5k5ccgx_wb_daylight),
		S5K5CCGX_REGSET(WHITE_BALANCE_CLOUDY, s5k5ccgx_wb_cloudy),
		S5K5CCGX_REGSET(WHITE_BALANCE_TUNGSTEN,
				s5k5ccgx_wb_incandescent),
		S5K5CCGX_REGSET(WHITE_BALANCE_FLUORESCENT,
				s5k5ccgx_wb_fluorescent),
	},
	.preview_size = {
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_QCIF,
				s5k5ccgx_176_144_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_320x240,
				s5k5ccgx_320_240_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_CIF, s5k5ccgx_352_288_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_528x432,
				s5k5ccgx_528_432_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_VGA, s5k5ccgx_640_480_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_736x552, s5k5ccgx_736_552_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_D1, s5k5ccgx_720_480_Preview),
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_SVGA,
				s5k5ccgx_800_600_Preview),
#ifdef CONFIG_VIDEO_S5K5CCGX_P2
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_1024x552, \
					s5k5ccgx_1024_552_Preview),
#else
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_1024x576, \
					s5k5ccgx_1024_576_Preview),
#endif
		S5K5CCGX_REGSET(S5K5CCGX_PREVIEW_XGA, s5k5ccgx_1024_768_Preview),
	},
	.scene_mode = {
		S5K5CCGX_REGSET(SCENE_MODE_NONE, s5k5ccgx_scene_off),
		S5K5CCGX_REGSET(SCENE_MODE_PORTRAIT, s5k5ccgx_scene_portrait),
		S5K5CCGX_REGSET(SCENE_MODE_NIGHTSHOT, s5k5ccgx_scene_nightshot),
		S5K5CCGX_REGSET(SCENE_MODE_LANDSCAPE, s5k5ccgx_scene_landscape),
		S5K5CCGX_REGSET(SCENE_MODE_SPORTS, s5k5ccgx_scene_sports),
		S5K5CCGX_REGSET(SCENE_MODE_PARTY_INDOOR, s5k5ccgx_scene_party),
		S5K5CCGX_REGSET(SCENE_MODE_BEACH_SNOW, s5k5ccgx_scene_beach),
		S5K5CCGX_REGSET(SCENE_MODE_SUNSET, s5k5ccgx_scene_sunset),
		S5K5CCGX_REGSET(SCENE_MODE_DUSK_DAWN, s5k5ccgx_scene_dawn),
		S5K5CCGX_REGSET(SCENE_MODE_TEXT, s5k5ccgx_scene_text),
		S5K5CCGX_REGSET(SCENE_MODE_CANDLE_LIGHT, s5k5ccgx_scene_candle),
	},
	.saturation = {
		S5K5CCGX_REGSET(SATURATION_MINUS_2, s5k5ccgx_saturation_m_2),
		S5K5CCGX_REGSET(SATURATION_MINUS_1, s5k5ccgx_saturation_m_1),
		S5K5CCGX_REGSET(SATURATION_DEFAULT, s5k5ccgx_saturation_0),
		S5K5CCGX_REGSET(SATURATION_PLUS_1, s5k5ccgx_saturation_p_1),
		S5K5CCGX_REGSET(SATURATION_PLUS_2, s5k5ccgx_saturation_p_2),
	},
	.contrast = {
		S5K5CCGX_REGSET(CONTRAST_MINUS_2, s5k5ccgx_contrast_m_2),
		S5K5CCGX_REGSET(CONTRAST_MINUS_1, s5k5ccgx_contrast_m_1),
		S5K5CCGX_REGSET(CONTRAST_DEFAULT, s5k5ccgx_contrast_0),
		S5K5CCGX_REGSET(CONTRAST_PLUS_1, s5k5ccgx_contrast_p_1),
		S5K5CCGX_REGSET(CONTRAST_PLUS_2, s5k5ccgx_contrast_p_2),

	},
	.sharpness = {
		S5K5CCGX_REGSET(SHARPNESS_MINUS_2, s5k5ccgx_sharpness_m_2),
		S5K5CCGX_REGSET(SHARPNESS_MINUS_1, s5k5ccgx_sharpness_m_1),
		S5K5CCGX_REGSET(SHARPNESS_DEFAULT, s5k5ccgx_sharpness_0),
		S5K5CCGX_REGSET(SHARPNESS_PLUS_1, s5k5ccgx_sharpness_p_1),
		S5K5CCGX_REGSET(SHARPNESS_PLUS_2, s5k5ccgx_sharpness_p_2),
	},
	.fps = {
		S5K5CCGX_REGSET(I_FPS_0, s5k5ccgx_fps_auto),
		S5K5CCGX_REGSET(I_FPS_15, s5k5ccgx_fps_15fix),
		S5K5CCGX_REGSET(I_FPS_25, s5k5ccgx_fps_25fix),
		S5K5CCGX_REGSET(I_FPS_30, s5k5ccgx_fps_30fix),
	},
	.preview_return = S5K5CCGX_REGSET_TABLE(s5k5ccgx_preview_return),

	.flash_start = S5K5CCGX_REGSET_TABLE(s5k5ccgx_mainflash_start),
	.flash_end = S5K5CCGX_REGSET_TABLE(s5k5ccgx_mainflash_end),
	.af_pre_flash_start =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_preflash_start),
	.af_pre_flash_end =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_preflash_end),
	.flash_ae_set =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_flash_ae_set),
	.flash_ae_clear =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_flash_ae_clear),
	.ae_lock_on =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_ae_lock),
	.ae_lock_off =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_ae_unlock),
	.awb_lock_on =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_awb_lock),
	.awb_lock_off =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_awb_unlock),

	.restore_cap = S5K5CCGX_REGSET_TABLE(s5k5ccgx_restore_capture_reg),
#ifdef CONFIG_VIDEO_S5K5CCGX_P2
	.change_wide_cap = S5K5CCGX_REGSET_TABLE(s5k5ccgx_change_wide_cap_p2),
#else
	.change_wide_cap = S5K5CCGX_REGSET_TABLE(s5k5ccgx_change_wide_cap),
#endif

	.af_macro_mode = S5K5CCGX_REGSET_TABLE(s5k5ccgx_af_macro_on),
	.af_normal_mode = S5K5CCGX_REGSET_TABLE(s5k5ccgx_af_normal_on),
#if !defined(CONFIG_VIDEO_S5K5CCGX_P2)
	.af_night_normal_mode =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_af_night_normal_on),
#endif
	.hd_af_start = S5K5CCGX_REGSET_TABLE(s5k5ccgx_720P_af_do),
	.hd_first_af_start = S5K5CCGX_REGSET_TABLE(s5k5ccgx_1st_720P_af_do),
	.single_af_start = S5K5CCGX_REGSET_TABLE(s5k5ccgx_af_do),
	.capture_start = {
		S5K5CCGX_REGSET(S5K5CCGX_CAPTURE_VGA, s5k5ccgx_snapshot_pvga),
		S5K5CCGX_REGSET(S5K5CCGX_CAPTURE_W2MP, s5k5ccgx_snapshot),
		S5K5CCGX_REGSET(S5K5CCGX_CAPTURE_3MP, s5k5ccgx_snapshot),
	},
	.init_reg = S5K5CCGX_REGSET_TABLE(s5k5ccgx_init_reg),
	.get_esd_status = S5K5CCGX_REGSET_TABLE(s5k5ccgx_get_esd_reg),
	.stream_stop = S5K5CCGX_REGSET_TABLE(s5k5ccgx_stream_stop_reg),
	.get_light_level = S5K5CCGX_REGSET_TABLE(s5k5ccgx_get_light_status),
	.get_iso = S5K5CCGX_REGSET_TABLE(s5k5ccgx_get_iso_reg),
	.get_ae_stable = S5K5CCGX_REGSET_TABLE(s5k5ccgx_get_ae_stable_reg),
	.get_shutterspeed =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_get_shutterspeed_reg),
	.update_preview_setting =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_update_preview_setting),
	.antibanding_50hz =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_antibanding_50hz_reg),
	.antibanding_60hz =
		S5K5CCGX_REGSET_TABLE(s5k5ccgx_antibanding_60hz_reg),
#ifdef DEBUG_FILTER_DATA
	/* for debugging AF fail in HD lowlight. */
	.get_filter_data = S5K5CCGX_REGSET_TABLE(s5k5ccgx_get_filter_data_reg),
#endif
};

static const struct v4l2_fmtdesc capture_fmts[] = {
	{
		.index		= 0,
		.type		= V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.flags		= FORMAT_FLAGS_COMPRESSED,
		.description	= "JPEG + Postview",
		.pixelformat	= V4L2_PIX_FMT_JPEG,
	},
};

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

struct test {
	u8 data;
	struct test *nextBuf;
};
static struct test *testBuf;
static s32 large_file;

#define TEST_INIT	\
{			\
	.data = 0;	\
	.nextBuf = NULL;	\
}

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
		cam_err("file open error\n");
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
			cam_err("ERR: nBuf Out of Memory\n");
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
		cam_err("ERR: Out of Memory\n");
		ret = -ENOMEM;
		goto error_out;
	}

	pos = 0;
	memset(nBuf, 0, file_size);
	memset(testBuf, 0, file_size * sizeof(struct test));

	nread = vfs_read(fp, (char __user *)nBuf, file_size, &pos);
	if (nread != file_size) {
		cam_err("failed to read file ret = %d\n", nread);
		ret = -1;
		goto error_out;
	}

	set_fs(fs);

	i = max_size;

	printk(KERN_DEBUG "i = %d\n", i);

	while (i) {
		testBuf[max_size - i].data = *nBuf;
		if (i != 1) {
			testBuf[max_size - i].nextBuf =
				&testBuf[max_size - i + 1];
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
					} else if (
					    testBuf[max_size-i].nextBuf->data
					    == '*') {
						/* when find '/ *' */
						starCheck = 1;
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
				if (testBuf[max_size-i].nextBuf != NULL) {
					if (testBuf[max_size-i].nextBuf->data
					    == '*') {
						/* when find '/ *' */
						starCheck = 1;
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
					if (testBuf[max_size-i].nextBuf->data
					    == '/') {
						/* when find '* /' */
						starCheck = 0;
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

#if 0 /* for print */
	cam_dbg("i = %d\n", i);
	nextBuf = &testBuf[0];
	while (1) {
		if (nextBuf->nextBuf == NULL)
			break;
		cam_dbg("%c", nextBuf->data);
		nextBuf = nextBuf->nextBuf;
	}
#endif

error_out:
	tmp_large_file ? vfree(nBuf) : kfree(nBuf);

	if (fp)
		filp_close(fp, current->files);
	return ret;
}


#endif

/**
 * s5k5ccgx_i2c_read_twobyte: Read 2 bytes from sensor
 */
static int s5k5ccgx_i2c_read_twobyte(struct i2c_client *client,
				  u16 subaddr, u16 *data)
{
	int err;
	u8 buf[2];
	struct i2c_msg msg[2];

	cpu_to_be16s(&subaddr);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8 *)&subaddr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = buf;

	err = i2c_transfer(client->adapter, msg, 2);
	if (unlikely(err != 2)) {
		dev_err(&client->dev,
			"%s: register read fail\n", __func__);
		return -EIO;
	}

	*data = ((buf[0] << 8) | buf[1]);

	return 0;
}

/**
 * s5k5ccgx_i2c_write_twobyte: Write (I2C) multiple bytes to the camera sensor
 * @client: pointer to i2c_client
 * @cmd: command register
 * @w_data: data to be written
 * @w_len: length of data to be written
 *
 * Returns 0 on success, <0 on error
 */
static int s5k5ccgx_i2c_write_twobyte(struct i2c_client *client,
					 u16 addr, u16 w_data)
{
	int retry_count = 5;
	int ret = 0;
	u8 buf[4] = {0,};
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 4,
		.buf	= buf,
	};

	buf[0] = addr >> 8;
	buf[1] = addr;
	buf[2] = w_data >> 8;
	buf[3] = w_data & 0xff;

#if (1)
	s5k5ccgx_debug(S5K5CCGX_DEBUG_I2C, "%s : W(0x%02X%02X%02X%02X)\n",
		__func__, buf[0], buf[1], buf[2], buf[3]);
#else
	printk(KERN_INFO "%s : W(0x%02X%02X%02X%02X)\n",
		__func__, buf[0], buf[1], buf[2], buf[3]);
#endif

	do {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;
		msleep(POLL_TIME_MS);
		cam_err("%s: ERROR(%d), write (%04X, %04X), retry %d.\n",
				__func__, ret, addr, w_data, retry_count);
	} while (retry_count-- > 0);

	if (unlikely(ret != 1)) {
		cam_err("%s: ERROR, I2C does not working.\n\n", __func__);
		return -EIO;
	}

	return 0;
}

/* PX: */
#ifdef CONFIG_LOAD_FILE
static int s5k5ccgx_write_regs_from_sd(struct v4l2_subdev *sd,
						const u8 s_name[])
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
#ifdef DEBUG_WRITE_REGS
	u8 regs_name[128] = {0,};

	BUG_ON(size > sizeof(regs_name));
#endif

	cam_dbg("%s: E size = %d, string = %s\n", __func__, size, s_name);
	tempData = &testBuf[0];
	while (!searched) {
		searched = 1;
		for (i = 0; i < size; i++) {
			if (tempData->data != s_name[i]) {
				searched = 0;
				break;
			}
#ifdef DEBUG_WRITE_REGS
			regs_name[i] = tempData->data;
#endif
			tempData = tempData->nextBuf;
		}
#ifdef DEBUG_WRITE_REGS
		if (i > 9) {
			regs_name[i] = '\0';
			cam_dbg("Searching: regs_name = %s\n", regs_name);
		}
#endif
		tempData = tempData->nextBuf;
	}
	/* structure is get..*/
#ifdef DEBUG_WRITE_REGS
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

		if ((temp & S5K5CCGX_DELAY) == S5K5CCGX_DELAY) {
			delay = temp & 0x0FFFF;
			cam_dbg("line(%d):delay(%#x)(%d)\n",
						__LINE__, delay, delay);
			msleep(delay);
			continue;
		}

		/* cam_dbg("I2C writing: 0x%08X,\n",temp);*/
		ret = s5k5ccgx_i2c_write_twobyte(client,
			(temp >> 16), (u16)temp);

		/* In error circumstances */
		/* Give second shot */
		if (unlikely(ret)) {
			dev_info(&client->dev,
					"s5k5ccgx i2c retry one more time\n");
			ret = s5k5ccgx_i2c_write_twobyte(client,
				(temp >> 16), (u16)temp);

			/* Give it one more shot */
			if (unlikely(ret)) {
				dev_info(&client->dev,
						"s5k5ccgx i2c retry twice\n");
				ret = s5k5ccgx_i2c_write_twobyte(client,
					(temp >> 16), (u16)temp);
			}
		}
	}

	return ret;
}
#endif

/* Write register
 * If success, return value: 0
 * If fail, return value: -EIO
 */
static int s5k5ccgx_write_regs(struct v4l2_subdev *sd, const u32 regs[],
			     int size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 delay = 0;
	int i, err = 0;

	for (i = 0; i < size; i++) {
		if ((regs[i] & S5K5CCGX_DELAY) == S5K5CCGX_DELAY) {
			delay = regs[i] & 0xFFFF;
			cam_dbg("line(%d):delay(%#x)(%d)\n",
						__LINE__, delay, delay);
			msleep(delay);
			continue;
		}
		/* cam_dbg("I2C writing: 0x%08X,\n", regs[i]); */

		err = s5k5ccgx_i2c_write_twobyte(client,
			(regs[i] >> 16), regs[i]);
		CHECK_ERR_N_MSG(err, "write registers\n")
	}

	return 0;
}

#if 0
static int s5k5ccgx_i2c_write_block(struct v4l2_subdev *sd, u8 *buf, u32 size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int retry_count = 5;
	int ret = 0;
	struct i2c_msg msg = {client->addr, 0, size, buf};

#ifdef CONFIG_VIDEO_S5K5CCGX_DEBUG
	if (s5k5ccgx_debug_mask & S5K5CCGX_DEBUG_I2C_BURSTS) {
		if ((buf[0] == 0x0F) && (buf[1] == 0x12))
			pr_info("%s : data[0,1] = 0x%02X%02X,"
				" total data size = %d\n",
				__func__, buf[2], buf[3], size-2);
		else
			pr_info("%s : 0x%02X%02X%02X%02X\n",
				__func__, buf[0], buf[1], buf[2], buf[3]);
	}
#endif

	do {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (likely(ret == 1))
			break;
		msleep(POLL_TIME_MS);
	} while (retry_count-- > 0);
	if (ret != 1) {
		dev_err(&client->dev, "%s: I2C is not working.\n", __func__);
		return -EIO;
	}

	return 0;
}
#endif

#define BURST_MODE_BUFFER_MAX_SIZE 2700
u8 s5k5ccgx_burstmode_buf[BURST_MODE_BUFFER_MAX_SIZE];

/* PX: */
static int s5k5ccgx_burst_write_regs(struct v4l2_subdev *sd,
			const u32 list[], u32 size, char *name)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;
	int i = 0, idx = 0;
	u16 subaddr = 0, next_subaddr = 0, value = 0;
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 0,
		.buf	= s5k5ccgx_burstmode_buf,
	};

	cam_trace("E\n");

	for (i = 0; i < size; i++) {
		if (idx > (BURST_MODE_BUFFER_MAX_SIZE - 10)) {
			cam_err("%s: ERROR, BURST MOD" \
				"buffer overflow!\n", __func__);
			return err;
		}

		subaddr = (list[i] & 0xFFFF0000) >> 16;
		if (subaddr == 0x0F12)
			next_subaddr = (list[i+1] & 0xFFFF0000) >> 16;

		value = list[i] & 0x0000FFFF;

		switch (subaddr) {
		case 0x0F12:
			/* make and fill buffer for burst mode write. */
			if (idx == 0) {
				s5k5ccgx_burstmode_buf[idx++] = 0x0F;
				s5k5ccgx_burstmode_buf[idx++] = 0x12;
			}
			s5k5ccgx_burstmode_buf[idx++] = value >> 8;
			s5k5ccgx_burstmode_buf[idx++] = value & 0xFF;

			/* write in burstmode*/
			if (next_subaddr != 0x0F12) {
				msg.len = idx;
				err = i2c_transfer(client->adapter,
					&msg, 1) == 1 ? 0 : -EIO;
				CHECK_ERR_N_MSG(err, "i2c_transfer\n");
				/* cam_dbg("s5k5ccgx_sensor_burst_write,
						idx = %d\n", idx); */
				idx = 0;
			}
			break;

		case 0xFFFF:
			cam_dbg("Butst mode: line(%d), delay(%#x)(%d)\n",
					__LINE__, value, value);
			msleep(value);
			break;

		default:
			idx = 0;
			err = s5k5ccgx_i2c_write_twobyte(client,
						subaddr, value);
			CHECK_ERR_N_MSG(err, "i2c_write_twobytes\n");
			break;
		}
	}

	return 0;
}

/* PX: */
static int s5k5ccgx_set_from_table(struct v4l2_subdev *sd,
				const char *setting_name,
				const struct s5k5ccgx_regset_table *table,
				u32 table_size, s32 index)
{
	int err = 0;

	/* cam_dbg("%s: set %s index %d\n",
		__func__, setting_name, index); */
	if ((index < 0) || (index >= table_size)) {
		cam_err("%s: ERROR, index(%d) out of range[0:%d]"
			"for table for %s\n", __func__, index,
			table_size, setting_name);
		return -EINVAL;
	}

	table += index;

#ifdef CONFIG_LOAD_FILE
	cam_dbg("%s: \"%s\", reg_name=%s\n", __func__,
			setting_name, table->name);
	return s5k5ccgx_write_regs_from_sd(sd, table->name);

#else /* CONFIG_LOAD_FILE */

#ifdef DEBUG_WRITE_REGS
	cam_dbg("%s: \"%s\", reg_name=%s\n", __func__,
			setting_name, table->name);
#endif /* DEBUG_WRITE_REGS */
	if (unlikely(!table->reg)) {
		cam_err("%s: ERROR, reg = NULL\n", __func__);
		return -EFAULT;
	}

	err = s5k5ccgx_write_regs(sd, table->reg, table->array_size);
	CHECK_ERR_N_MSG(err, "write regs(%s), err=%d\n",
				setting_name, err);
	return 0;
#endif /* CONFIG_LOAD_FILE */
}

/* PX: */
static inline int s5k5ccgx_save_ctrl(struct v4l2_control *ctrl)
{
	int i;

	/* cam_trace("E, Ctrl-ID = 0x%X", ctrl->id);*/

	for (i = 0; i < ARRAY_SIZE(s5k5ccgx_ctrls); i++) {
		if (ctrl->id == s5k5ccgx_ctrls[i].id) {
			s5k5ccgx_ctrls[i].value = ctrl->value;
			return 0;
		}
	}

	return -ENOIOCTLCMD;
}

/* PX: Contro Flash LED */
static inline int s5k5ccgx_flash_en(struct v4l2_subdev *sd, s32 mode, s32 onoff)
{
	struct s5k5ccgx_state *state = to_state(sd);

	if (unlikely(state->ignore_flash)) {
		cam_warn("WARNING, we ignore flash command.\n");
		return 0;
	}

	return state->pdata->flash_en(mode, onoff);
}

/* PX: Set scene mode */
static int s5k5ccgx_set_scene_mode(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -ENODEV;

	cam_trace("E, value %d\n", val);

	if (state->scene_mode == val)
		return 0;

	/* when scene mode is switched,
	 * we frist have to write scene_off.
	 */
	if (state->scene_mode != SCENE_MODE_NONE)
		err = s5k5ccgx_set_from_table(sd, "scene_mode",
			state->regs->scene_mode,
			ARRAY_SIZE(state->regs->scene_mode), SCENE_MODE_NONE);

	if (val != SCENE_MODE_NONE)
		err = s5k5ccgx_set_from_table(sd, "scene_mode",
			state->regs->scene_mode,
			ARRAY_SIZE(state->regs->scene_mode), val);

	state->scene_mode = val;

	cam_trace("X\n");
	return 0;
}

/* PX: Set brightness */
static int s5k5ccgx_set_exposure(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

	if ((val < EV_MINUS_4) || (val > EV_PLUS_4)) {
		cam_err("%s: ERROR, invalid value(%d)\n", __func__, val);
		return -EINVAL;
	}

	err = s5k5ccgx_set_from_table(sd, "brightness", state->regs->ev,
		ARRAY_SIZE(state->regs->ev), GET_EV_INDEX(val));

	return err;
}

/* PX: Check light level */
static u32 s5k5ccgx_get_light_level(struct v4l2_subdev *sd, u32 *light_level)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_state *state = to_state(sd);
	u16 val_lsb = 0;
	u16 val_msb = 0;
	int err = -ENODEV;

	err = s5k5ccgx_set_from_table(sd, "get_light_level",
			&state->regs->get_light_level, 1, 0);
	CHECK_ERR_N_MSG(err, "fail to get light level\n");

	err = s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &val_lsb);
	err = s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &val_msb);
	CHECK_ERR_N_MSG(err, "fail to read light level\n");

	*light_level = val_lsb | (val_msb<<16);

	/* cam_trace("X, light level = 0x%X", *light_level); */

	return 0;
}

static int s5k5ccgx_set_capture_size(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);

#ifdef CONFIG_VIDEO_S5K5CCGX_P2
	int err = 0;
	int capture_width = state->capture->width;
	int capture_height = state->capture->height;

	int ratio = capture_width*10/capture_height;

	cam_warn("set_capture_size = %d\n", ratio);

	if (ratio != 13) {
		cam_info("%s: Wide Capture setting\n\n\n", __func__);
		err = s5k5ccgx_set_from_table(sd, "change_wide_cap",
			&state->regs->change_wide_cap, 1, 0);
	} else {
		cam_info("%s: Wide Restore setting\n\n\n", __func__);
		err = s5k5ccgx_set_from_table(sd, "restore_capture",
				&state->regs->restore_cap, 1, 0);
	}
#endif
	return 0;
}

/* PX: Set sensor mode */
static int s5k5ccgx_set_sensor_mode(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);

	state->hd_videomode = false;

	switch (val) {
	case SENSOR_MOVIE:
		/* We does not support movie mode when in VT. */
		if (state->vt_mode) {
			state->sensor_mode = SENSOR_CAMERA;
			cam_err("%s: ERROR, Not support movie\n", __func__);
			break;
		}
		/* We do not break. */

	case SENSOR_CAMERA:
		state->sensor_mode = val;
		break;

	case 2:	/* 720p HD video mode */
		state->sensor_mode = SENSOR_MOVIE;
		state->hd_videomode = true;
		break;

	default:
		cam_err("%s: ERROR, Not support.(%d)\n", __func__, val);
		state->sensor_mode = SENSOR_CAMERA;
		WARN_ON(1);
		break;
	}

	return 0;
}

/* PX: Set framerate */
static int s5k5ccgx_set_frame_rate(struct v4l2_subdev *sd, u32 fps)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EIO;
	int i = 0, fps_index = -1;

	if (state->hd_videomode)
		return 0;

	cam_info("set frame rate %d\n\n", fps);

	for (i = 0; i < ARRAY_SIZE(s5k5ccgx_framerates); i++) {
		if (fps == s5k5ccgx_framerates[i].fps) {
			fps_index = s5k5ccgx_framerates[i].index;
			state->fps = fps;
			break;
		}
	}

	if (unlikely(fps_index < 0)) {
		cam_err("%s: WARNING, Not supported FPS(%d)\n", __func__, fps);
		return 0;
	}

	err = s5k5ccgx_set_from_table(sd, "fps",
				state->regs->fps,
				ARRAY_SIZE(state->regs->fps), fps_index);

	CHECK_ERR_N_MSG(err, "fail to set framerate\n")
	return 0;
}

static int s5k5ccgx_set_ae_lock(struct v4l2_subdev *sd, s32 val, bool force)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 0;

	switch (val) {
	case AE_LOCK:
		if (state->focus.touch)
			return 0;

		err = s5k5ccgx_set_from_table(sd, "ae_lock_on",
				&state->regs->ae_lock_on, 1, 0);
		WARN_ON(state->focus.ae_lock);
		state->focus.ae_lock = 1;
		break;

	case AE_UNLOCK:
		if (unlikely(!force && !state->focus.ae_lock))
			return 0;

		err = s5k5ccgx_set_from_table(sd, "ae_lock_off",
				&state->regs->ae_lock_off, 1, 0);
		state->focus.ae_lock = 0;
		break;

	default:
		cam_err("%s: WARNING, invalid argument(%d)\n", __func__, val);
	}

	CHECK_ERR_N_MSG(err, "fail to lock AE(%d), err=%d\n", val, err);

	return 0;
}

static int s5k5ccgx_set_awb_lock(struct v4l2_subdev *sd, s32 val, bool force)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 0;

	switch (val) {
	case AWB_LOCK:
		if (state->flash_on ||
		    (state->wb_mode != WHITE_BALANCE_AUTO))
			return 0;

		err = s5k5ccgx_set_from_table(sd, "awb_lock_on",
				&state->regs->awb_lock_on, 1, 0);
		WARN_ON(state->focus.awb_lock);
		state->focus.awb_lock = 1;
		break;

	case AWB_UNLOCK:
		if (unlikely(!force && !state->focus.awb_lock))
			return 0;

		err = s5k5ccgx_set_from_table(sd, "awb_lock_off",
			&state->regs->awb_lock_off, 1, 0);
		state->focus.awb_lock = 0;
		break;

	default:
		cam_err("%s: WARNING, invalid argument(%d)\n", __func__, val);
	}

	CHECK_ERR_N_MSG(err, "fail to lock AWB(%d), err=%d\n", val, err);

	return 0;
}

/* PX: Set AE, AWB Lock */
static int s5k5ccgx_set_lock(struct v4l2_subdev *sd, s32 lock, bool force)
{
	int err = -EIO;

	cam_trace("%s\n", lock ? "on" : "off");
	if (unlikely((u32)lock >= AEAWB_LOCK_MAX)) {
		cam_err("%s: ERROR, invalid argument\n", __func__);
		return -EINVAL;
	}

	err = s5k5ccgx_set_ae_lock(sd, (lock == AEAWB_LOCK) ?
				AE_LOCK : AE_UNLOCK, force);
	if (unlikely(err))
		goto out_err;

	err = s5k5ccgx_set_awb_lock(sd, (lock == AEAWB_LOCK) ?
				AWB_LOCK : AWB_UNLOCK, force);
	if (unlikely(err))
		goto out_err;

	cam_trace("X\n");
	return 0;

out_err:
	cam_err("%s: ERROR, failed to set lock\n", __func__);
	return err;
}

/* PX: */
static int s5k5ccgx_return_focus(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_trace("E\n");

	switch (state->focus.mode) {
	case FOCUS_MODE_MACRO:
		err = s5k5ccgx_set_from_table(sd, "af_macro_mode",
				&state->regs->af_macro_mode, 1, 0);
		break;

	default:
#if !defined(CONFIG_VIDEO_S5K5CCGX_P2)
		if (state->scene_mode == SCENE_MODE_NIGHTSHOT)
			err = s5k5ccgx_set_from_table(sd,
				"af_night_normal_mode",
				&state->regs->af_night_normal_mode, 1, 0);
		else
#endif
			err = s5k5ccgx_set_from_table(sd,
				"af_norma_mode",
				&state->regs->af_normal_mode, 1, 0);
		break;
	}

	CHECK_ERR(err);
	return 0;
}

#ifdef DEBUG_FILTER_DATA
static void __used s5k5ccgx_display_AF_win_info(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_rect first_win = {0, 0, 0, 0};
	struct s5k5ccgx_rect second_win = {0, 0, 0, 0};

	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x022C);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&first_win.x);
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x022E);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&first_win.y);
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0230);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&first_win.width);
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0232);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&first_win.height);

	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0234);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&second_win.x);
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0236);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&second_win.y);
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0238);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&second_win.width);
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x023A);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, (u16 *)&second_win.height);

	cam_info("------- AF Window info -------\n");
	cam_info("Firtst Window: (%4d %4d %4d %4d)\n",
		first_win.x, first_win.y, first_win.width, first_win.height);
	cam_info("Second Window: (%4d %4d %4d %4d)\n",
		second_win.x, second_win.y,
		second_win.width, second_win.height);
	cam_info("------- AF Window info -------\n\n");
}

#define DISPLAY_ROW_CNT		(16 + 10)
#define DISPLAY_COL_CNT		8
static void __used s5k5ccgx_display_filter_data(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EIO;
	u16 filter_buf[DISPLAY_ROW_CNT][DISPLAY_COL_CNT] = {{0,},};
	u8 line_buf[128] = {0,};
	u32 write_cnt = 0;
	int i, j;

	err = s5k5ccgx_set_from_table(sd, "get_filter_data",
			&state->regs->get_filter_data, 1, 0);

	s5k5ccgx_i2c_write_twobyte(client, 0x0028, 0x7000);

	/* Lens reposition */
	s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x0224);
	s5k5ccgx_i2c_write_twobyte(client, 0x0F12, 0x0003);
	msleep(100);

	/* Init: log count */
	s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x1542);
	s5k5ccgx_i2c_write_twobyte(client, 0x0F12, 0x0000);

	/* Init: log clear */
	s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x0224);
	s5k5ccgx_i2c_write_twobyte(client, 0x0F12, 0x0003);
	msleep(100);

	/* AF start */
	s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x0224);
	s5k5ccgx_i2c_write_twobyte(client, 0x0F12, 0x0005);
	msleep(2000); /* Sleep 2 second */

	/* Start reading */
	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x3000);
	for (i = 0; i < DISPLAY_ROW_CNT; i++) {
		for (j = 0; j < DISPLAY_COL_CNT; j++)
			s5k5ccgx_i2c_read_twobyte(client, 0x0F12,
					&filter_buf[i][j]);
	}

	cam_info("------- Display filter data -------\n");
	for (i = 0; i < DISPLAY_ROW_CNT; i++) {
		for (j = 0, write_cnt = 0; j < DISPLAY_COL_CNT; j++)
			write_cnt += sprintf(line_buf+write_cnt, " %04X",
					filter_buf[i][j]);

		pr_info("%04X: %s\n", DISPLAY_ROW_CNT * i, line_buf);
	}
	cam_info("------- Display filter data -------\n\n");

	/* Restore */
	s5k5ccgx_i2c_write_twobyte(client, 0x0028, 0x7000);
}
#endif

/* PX: Prepare AF Flash */
static int s5k5ccgx_af_start_preflash(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_state *state = to_state(sd);
	u16 read_value = 0;
	u32 light_level = 0xFFFFFFFF, count = 0;
	int err = 0;

	cam_trace("E\n");

	if (state->sensor_mode == SENSOR_MOVIE)
		return 0;

	cam_dbg("Start SINGLE AF, flash mode %d\n", state->flash_mode);

	/* in case user calls auto_focus repeatedly without a cancel
	 * or a capture, we need to cancel here to allow ae_awb
	 * to work again, or else we could be locked forever while
	 * that app is running, which is not the expected behavior.
	 */
	err = s5k5ccgx_set_lock(sd, AEAWB_UNLOCK, false);
	CHECK_ERR_N_MSG(err, "fail to set lock\n");

	state->focus.preflash = PREFLASH_OFF;

	s5k5ccgx_get_light_level(sd, &light_level);

	switch(state->flash_mode) {
	case FLASH_MODE_AUTO:
		if (light_level >= LOW_LIGHT_LEVEL) {
			/* flash not needed */
			break;
		}

	case FLASH_MODE_ON:
		s5k5ccgx_set_from_table(sd, "af_pre_flash_start",
			&state->regs->af_pre_flash_start, 1, 0);
		s5k5ccgx_set_from_table(sd, "flash_ae_set",
			&state->regs->flash_ae_set, 1, 0);
		s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
			S5K5CCGX_FLASH_ON);
		state->flash_on = true;
		state->focus.preflash = PREFLASH_ON;
		break;

	case FLASH_MODE_OFF:
		if (light_level < LOW_LIGHT_LEVEL)
			state->one_frame_delay_ms = ONE_FRAME_DELAY_MS_LOW;
		break;

	default:
		break;
	}

	/* We wait for 200ms after pre flash on.
	 * check whether AE is stable.*/
	msleep(200);

	/* Check AE-stable */
	if (state->flash_on) {
		/* Do checking AE-stable */
		for (count = 0; count < AE_STABLE_SEARCH_COUNT; count++) {
			if (state->focus.start == AUTO_FOCUS_OFF) {
				cam_info("af_start_preflash: \
						AF is cancelled!\n");
				state->focus.status = AF_RESULT_CANCELLED;
				break;
			}

			s5k5ccgx_set_from_table(sd, "get_ae_stable",
					&state->regs->get_ae_stable, 1, 0);
			s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value);
			if (read_value == 0x1) {
				af_dbg("AE-stable=0x%X, count=%d",
						read_value, count);
				break;
			}

			msleep(state->one_frame_delay_ms);
		}

		/* restore write mode */
		s5k5ccgx_i2c_write_twobyte(client, 0x0028, 0x7000);

		if (unlikely(count >= AE_STABLE_SEARCH_COUNT)) {
			cam_err("%s: ERROR, AE unstable\n\n", __func__);
			/* return -ENODEV; */
		}
	} else if (state->focus.start == AUTO_FOCUS_OFF) {
		cam_info("af_start_preflash: AF is cancelled!\n");
		state->focus.status = AF_RESULT_CANCELLED;
	}

	/* If AF cancel, finish pre-flash process. */
	if (state->focus.status == AF_RESULT_CANCELLED) {
		if (state->flash_on) {
			s5k5ccgx_set_from_table(sd, "af_pre_flash_end",
				&state->regs->af_pre_flash_end, 1, 0);
			s5k5ccgx_set_from_table(sd, "flash_ae_clear",
				&state->regs->flash_ae_clear, 1, 0);
			s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
				S5K5CCGX_FLASH_OFF);
			state->flash_on = false;
			state->focus.preflash = PREFLASH_NONE;
		}

		if (state->focus.touch)
			state->focus.touch = 0;
	}

	cam_trace("X\n");

	return 0;
}

/* PX: Do AF */
static int s5k5ccgx_do_af(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_state *state = to_state(sd);
	u16 read_value = 0;
	u32 count = 0;

	cam_trace("E\n");

	/* AE, AWB Lock */
	s5k5ccgx_set_lock(sd, AEAWB_LOCK, false);

	if (state->sensor_mode == SENSOR_MOVIE) {
		s5k5ccgx_set_from_table(sd, "hd_af_start",
			&state->regs->hd_af_start, 1, 0);

		cam_info("%s : 720P Auto Focus Operation\n\n", __func__);
	} else
		s5k5ccgx_set_from_table(sd, "single_af_start",
			&state->regs->single_af_start, 1, 0);

	/* Sleep while 2frame */
	if (state->hd_videomode)
		msleep(100); /* 100ms */
	else if (state->scene_mode == SCENE_MODE_NIGHTSHOT)
		msleep(TWO_FRAME_DELAY_MS_NIGHTMODE);
	else
		msleep(200); /* 200ms */

	/* AF Searching */
	cam_dbg("AF 1st search\n");

	/*1st search*/
	for (count = 0; count < FIRST_AF_SEARCH_COUNT; count++) {
		if (state->focus.start == AUTO_FOCUS_OFF) {
			cam_dbg("do_af: AF is cancelled while doing(1st)\n");
			state->focus.status = AF_RESULT_CANCELLED;
			goto check_done;
		}

		read_value = 0x0;
		s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
		s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x2D12);
		s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value);
		af_dbg("1st AF status(%02d) = 0x%04X\n",
					count, read_value);
		if (read_value != 0x01)
			break;

		msleep(state->one_frame_delay_ms);
	}

	if (read_value != 0x02) {
		cam_err("%s: ERROR, 1st AF failed. count=%d, read_val=0x%X\n\n",
					__func__, count, read_value);
		state->focus.status = AF_RESULT_FAILED;
		goto check_done;
	}

	/*2nd search*/
	cam_dbg("AF 2nd search\n");
	for (count = 0; count < SECOND_AF_SEARCH_COUNT; count++) {
		msleep(state->one_frame_delay_ms);

		if (state->focus.start == AUTO_FOCUS_OFF) {
			cam_dbg("do_af: AF is cancelled while doing(2nd)\n");
			state->focus.status = AF_RESULT_CANCELLED;
			goto check_done;
		}

		read_value = 0x0FFFF;
		s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
		s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x1F2F);
		s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value);
		af_dbg("2nd AF status(%02d) = 0x%04X\n",
						count, read_value);
		if ((read_value & 0x0ff00) == 0x0)
			break;
	}

	if (count >= SECOND_AF_SEARCH_COUNT) {
		/* 0x01XX means "Not Finish". */
		cam_err("%s: ERROR, 2nd AF failed. read_val=0x%X\n\n",
			__func__, read_value & 0x0ff00);
		state->focus.status = AF_RESULT_FAILED;
		goto check_done;
	}

	cam_info("AF Success!\n");
	state->focus.status = AF_RESULT_SUCCESS;

check_done:
	/* restore write mode */

	/* We only unlocked AE,AWB in case of being cancelled.
	 * But we now unlock it unconditionally if AF is started,
	 */
	if (state->focus.status == AF_RESULT_CANCELLED) {
		cam_dbg("%s: Single AF cancelled.\n", __func__);
		s5k5ccgx_set_lock(sd, AEAWB_UNLOCK, false);
	} else {
		state->focus.start = AUTO_FOCUS_OFF;
		cam_dbg("%s: Single AF finished\n", __func__);
	}

	if (state->flash_on && !state->hd_videomode) {
		s5k5ccgx_set_from_table(sd, "af_pre_flash_end",
				&state->regs->af_pre_flash_end, 1, 0);
		s5k5ccgx_set_from_table(sd, "flash_ae_clear",
			&state->regs->flash_ae_clear, 1, 0);
		s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
			S5K5CCGX_FLASH_OFF);
		state->flash_on = false;
		if (state->focus.status == AF_RESULT_CANCELLED) {
			state->focus.preflash = PREFLASH_NONE;
		}
	}

	/* Notice: we here turn off touch flag set when doing  Touch AF. */
	if (state->focus.touch)
		state->focus.touch = 0;

	/* complete(&state->af_complete); */

	return 0;
}

/* PX: Set AF */
static int s5k5ccgx_set_af(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 0;

	cam_info("%s: %s, focus mode %d\n\n\n", __func__,
			val ? "start" : "stop", state->focus.mode);

	if (state->focus.start == val)
		return 0;

	if (unlikely((u32)val >= AUTO_FOCUS_MAX)) {
		cam_err("%s: ERROR, invalid value(%d)\n", __func__, val);
		return -EINVAL;
	}
	state->focus.start = val;

	if (val == AUTO_FOCUS_ON) {
		mutex_lock(&state->af_lock);
		/* state->focus.af_cancel = 0; */
		state->focus.status = AF_RESULT_DOING;

		if (!state->hd_videomode) {
			state->one_frame_delay_ms = ONE_FRAME_DELAY_MS_NORMAL;
			err = s5k5ccgx_af_start_preflash(sd);
			if (unlikely(err))
				goto out;

			if (state->focus.status == AF_RESULT_CANCELLED)
				goto out;
		} else
			state->one_frame_delay_ms = 50;

		s5k5ccgx_do_af(sd);
		mutex_unlock(&state->af_lock);
	} else {
		/* Cancel AF */
		cam_info("set_af: AF cancel requested!\n");
	}

	cam_trace("X\n");
	return 0;

out:
	mutex_unlock(&state->af_lock);
	return err;
}

/* PX: Stop AF */
static int s5k5ccgx_stop_af(struct v4l2_subdev *sd, s32 touch)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 0;

	cam_trace("E\n");
	mutex_lock(&state->af_lock);

	switch (state->focus.status) {
	case AF_RESULT_FAILED:
	case AF_RESULT_SUCCESS:
		cam_dbg("Stop AF, focus mode %d, AF result %d\n",
			state->focus.mode, state->focus.status);

		err = s5k5ccgx_set_lock(sd, AEAWB_UNLOCK, false);
		if (unlikely(err)) {
			cam_err("%s: ERROR, fail to set lock\n", __func__);
			goto err_out;
		}
		state->focus.status = AF_RESULT_CANCELLED;
		state->focus.preflash = PREFLASH_NONE;
		break;

	case AF_RESULT_CANCELLED:
		break;

	default:
		cam_warn("%s: WARNING, unnecessary calling. AF status=%d\n",
			__func__, state->focus.status);
		/* Return 0. */
		goto err_out;
		break;
	}

	if (!touch) {
		err = s5k5ccgx_return_focus(sd);
		if (unlikely(err)) {
			cam_err("%s: ERROR, fail to af_norma_mode (%d)\n",
				__func__, err);
			goto err_out;
		}
	}

	mutex_unlock(&state->af_lock);
	cam_trace("X\n");
	return 0;

err_out:
	mutex_unlock(&state->af_lock);
	return err;
}

/* PX: Set focus mode */
static int s5k5ccgx_set_focus_mode(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	u32 af_cancel = 0;
	int err = -EINVAL;

	/* cam_trace("E\n");*/
	cam_dbg("%s val =%d(0x%X)\n", __func__, val, val);

	if (state->focus.mode == val)
		return 0;

	af_cancel = (u32)val & FOCUS_MODE_DEFAULT;
	mutex_lock(&state->af_lock);

	switch (val) {
	case FOCUS_MODE_MACRO:
		err = s5k5ccgx_set_from_table(sd, "af_macro_mode",
				&state->regs->af_macro_mode, 1, 0);
		if (unlikely(err)) {
			cam_err("%s: ERROR, fail to af_macro_mode (%d)\n",
							__func__, err);
			goto err_out;
		}

		state->focus.mode = FOCUS_MODE_MACRO;
		break;

	case FOCUS_MODE_INFINITY:
	case FOCUS_MODE_AUTO:
	case FOCUS_MODE_FIXED:
		err = s5k5ccgx_set_from_table(sd, "af_norma_mode",
				&state->regs->af_normal_mode, 1, 0);
		if (unlikely(err)) {
			cam_err("%s: ERROR, fail to af_norma_mode (%d)\n",
							__func__, err);
			goto err_out;
		}

		state->focus.mode = val;
		break;

	case FOCUS_MODE_FACEDETECT:
	case FOCUS_MODE_CONTINOUS:
	case FOCUS_MODE_TOUCH:
		break;

	default:
		if (!af_cancel) {
			cam_err("%s: ERROR, invalid val(0x%X)\n:",
						__func__, val);
			goto err_out;
		}
		break;
	}
	mutex_unlock(&state->af_lock);

	if (af_cancel)
		s5k5ccgx_stop_af(sd, 0);

	return 0;

err_out:
	mutex_unlock(&state->af_lock);
	return err;
}

/* PX: */
static int s5k5ccgx_set_af_window(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EIO;
	struct s5k5ccgx_rect inner_window = {0, 0, 0, 0};
	struct s5k5ccgx_rect outter_window = {0, 0, 0, 0};
	struct s5k5ccgx_rect first_window = {0, 0, 0, 0};
	struct s5k5ccgx_rect second_window = {0, 0, 0, 0};
	const s32 mapped_x = state->focus.pos_x;
	const s32 mapped_y = state->focus.pos_y;
	const u32 preview_width = state->preview->width;
	const u32 preview_height = state->preview->height;
	u32 inner_half_width = 0, inner_half_height = 0;
	u32 outter_half_width = 0, outter_half_height = 0;

	cam_trace("E\n");

	inner_window.width = SCND_WINSIZE_X * preview_width / 1024;
	inner_window.height = SCND_WINSIZE_Y * preview_height / 1024;
	outter_window.width = FIRST_WINSIZE_X * preview_width / 1024;
	outter_window.height = FIRST_WINSIZE_Y * preview_height / 1024;

	inner_half_width = inner_window.width / 2;
	inner_half_height = inner_window.height / 2;
	outter_half_width = outter_window.width / 2;
	outter_half_height = outter_window.height / 2;

	af_dbg("Preview width=%d, height=%d\n", preview_width, preview_height);
	af_dbg("inner_window_width=%d, inner_window_height=%d, " \
		"outter_window_width=%d, outter_window_height=%d\n ",
		inner_window.width, inner_window.height,
		outter_window.width, outter_window.height);

	/* Get X */
	if (mapped_x <= inner_half_width) {
		inner_window.x = outter_window.x = 0;
		af_dbg("inner & outter window over sensor left."
			"in_x=%d, out_x=%d\n", inner_window.x, outter_window.x);
	} else if (mapped_x <= outter_half_width) {
		inner_window.x = mapped_x - inner_half_width;
		outter_window.x = 0;
		af_dbg("outter window over sensor left. in_x=%d, out_x=%d\n",
					inner_window.x, outter_window.x);
	} else if (mapped_x >= ((preview_width - 1) - inner_half_width)) {
		inner_window.x = (preview_width - 1) - inner_window.width;
		outter_window.x = (preview_width - 1) - outter_window.width;
		af_dbg("inner & outter window over sensor right." \
			"in_x=%d, out_x=%d\n", inner_window.x, outter_window.x);
	} else if (mapped_x >= ((preview_width - 1) - outter_half_width)) {
		inner_window.x = mapped_x - inner_half_width;
		outter_window.x = (preview_width - 1) - outter_window.width;
		af_dbg("outter window over sensor right. in_x=%d, out_x=%d\n",
					inner_window.x, outter_window.x);
	} else {
		inner_window.x = mapped_x - inner_half_width;
		outter_window.x = mapped_x - outter_half_width;
		af_dbg("inner & outter window within sensor area." \
			"in_x=%d, out_x=%d\n", inner_window.x, outter_window.x);
	}

	/* Get Y */
	if (mapped_y <= inner_half_height) {
		inner_window.y = outter_window.y = 0;
		af_dbg("inner & outter window over sensor top." \
			"in_y=%d, out_y=%d\n", inner_window.y, outter_window.y);
	} else if (mapped_y <= outter_half_height) {
		inner_window.y = mapped_y - inner_half_height;
		outter_window.y = 0;
		af_dbg("outter window over sensor top. in_y=%d, out_y=%d\n",
					inner_window.y, outter_window.y);
	} else if (mapped_y >= ((preview_height - 1) - inner_half_height)) {
		inner_window.y = (preview_height - 1) - inner_window.height;
		outter_window.y = (preview_height - 1) - outter_window.height;
		af_dbg("inner & outter window over sensor bottom." \
			"in_y=%d, out_y=%d\n", inner_window.y, outter_window.y);
	} else if (mapped_y >= ((preview_height - 1) - outter_half_height)) {
		inner_window.y = mapped_y - inner_half_height;
		outter_window.y = (preview_height - 1) - outter_window.height;
		af_dbg("outter window over sensor bottom. in_y=%d, out_y=%d\n",
					inner_window.y, outter_window.y);
	} else {
		inner_window.y = mapped_y - inner_half_height;
		outter_window.y = mapped_y - outter_half_height;
		af_dbg("inner & outter window within sensor area." \
			"in_y=%d, out_y=%d\n", inner_window.y, outter_window.y);
	}

	af_dbg("==> inner_window top=(%d,%d), bottom=(%d, %d)\n",
		inner_window.x, inner_window.y,
		inner_window.x + inner_window.width,
		inner_window.y + inner_window.height);
	af_dbg("==> outter_window top=(%d,%d), bottom=(%d, %d)\n",
		outter_window.x, outter_window.y,
		outter_window.x + outter_window.width ,
		outter_window.y + outter_window.height);

	second_window.x = inner_window.x * 1024 / preview_width;
	second_window.y = inner_window.y * 1024 / preview_height;
	first_window.x = outter_window.x * 1024 / preview_width;
	first_window.y = outter_window.y * 1024 / preview_height;

	af_dbg("=> second_window top=(%d, %d)\n",
		second_window.x, second_window.y);
	af_dbg("=> first_window top=(%d, %d)\n",
		first_window.x, first_window.y);

	if (!mutex_is_locked(&state->af_lock)) {
		mutex_unlock(&state->af_lock);
		/* restore write mode */
		err = s5k5ccgx_i2c_write_twobyte(client, 0x0028, 0x7000);

		/* Set first window x, y */
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x022C);
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x0F12,
						(u16)(first_window.x));
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x022E);
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x0F12,
						(u16)(first_window.y));

		/* Set second widnow x, y */
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x0234);
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x0F12,
						(u16)(second_window.x));
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x0236);
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x0F12,
						(u16)(second_window.y));

		/* Update AF window */
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x002A, 0x023C);
		err |= s5k5ccgx_i2c_write_twobyte(client, 0x0F12, 0x0001);
		mutex_unlock(&state->af_lock);
		CHECK_ERR(err);
		cam_dbg("%s: AF window position completed.\n", __func__);
	}

	cam_trace("X\n");
	return 0;
}

static int s5k5ccgx_set_touch_af(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EIO;

	cam_trace("%s, x=%d y=%d\n", val ? "start" : "stop",
			state->focus.pos_x, state->focus.pos_y);

	state->focus.touch = val;

	if (val) {
		if (mutex_is_locked(&state->af_lock))
			goto out;

		err = s5k5ccgx_set_af_window(sd);
		CHECK_ERR_N_MSG(err, "val=%d\n", 1);

		/* We do not give delays */
		msleep(60);
	} else {
		err = s5k5ccgx_stop_af(sd, 1);
		CHECK_ERR_N_MSG(err, "val=%d\n", 0)
	}

out:
	cam_trace("X\n");
	return 0;
}

static int s5k5ccgx_init_param(struct v4l2_subdev *sd)
{
	struct v4l2_control ctrl;
	int i;

	for (i = 0; i < ARRAY_SIZE(s5k5ccgx_ctrls); i++) {
		if (s5k5ccgx_ctrls[i].value !=
				s5k5ccgx_ctrls[i].default_value) {
			ctrl.id = s5k5ccgx_ctrls[i].id;
			ctrl.value = s5k5ccgx_ctrls[i].value;
			s5k5ccgx_s_ctrl(sd, &ctrl);
		}
	}

	return 0;
}

static int s5k5ccgx_init_regs(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct s5k5ccgx_state *state = to_state(sd);
	u16 read_value = 0;
	int err = -ENODEV;

	/* we'd prefer to do this in probe, but the framework hasn't
	 * turned on the camera yet so our i2c operations would fail
	 * if we tried to do it in probe, so we have to do it here
	 * and keep track if we succeeded or not.
	 */

	/* enter read mode */
	err = s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	if (unlikely(err < 0))
		return -ENODEV;

	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0150);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value);
	pr_info("%s : FW ChipID  revision : %04X\n", __func__, read_value);

	s5k5ccgx_i2c_write_twobyte(client, 0x002C, 0x7000);
	s5k5ccgx_i2c_write_twobyte(client, 0x002E, 0x0152);
	s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value);
	pr_info("%s :FW EVT revision : %04X\n", __func__, read_value);

	/* restore write mode */
	s5k5ccgx_i2c_write_twobyte(client, 0x0028, 0x7000);

	state->regs = &reg_datas;

	return 0;
}

static const struct s5k5ccgx_framesize *s5k5ccgx_get_framesize
	(const struct s5k5ccgx_framesize *frmsizes,
	u32 frmsize_count, u32 index)
{
	int i = 0;

	for (i = 0; i < frmsize_count; i++) {
		if (frmsizes[i].index == index)
			return &frmsizes[i];
	}

	return NULL;
}

/* This function is called from the g_ctrl api
 *
 * This function should be called only after the s_fmt call,
 * which sets the required width/height value.
 *
 * It checks a list of available frame sizes and sets the
 * most appropriate frame size.
 *
 * The list is stored in an increasing order (as far as possible).
 * Hence the first entry (searching from the beginning) where both the
 * width and height is more than the required value is returned.
 * In case of no perfect match, we set the last entry (which is supposed
 * to be the largest resolution supported.)
 */
static void s5k5ccgx_set_framesize(struct v4l2_subdev *sd,
				const struct s5k5ccgx_framesize *frmsizes,
				u32 num_frmsize, bool preview)
{
	struct s5k5ccgx_state *state = to_state(sd);
	const struct s5k5ccgx_framesize **found_frmsize = NULL;
	u32 width = state->req_fmt.width;
	u32 height = state->req_fmt.height;
	int i = 0;

	cam_dbg("%s: Requested Res %dx%d\n", __func__,
			width, height);

	found_frmsize = (const struct s5k5ccgx_framesize **)
			(preview ? &state->preview : &state->capture);

	for (i = 0; i < num_frmsize; i++) {
		if ((frmsizes[i].width == width) &&
			(frmsizes[i].height == height)) {
			*found_frmsize = &frmsizes[i];
			break;
		}
	}

	if (*found_frmsize == NULL) {
		cam_err("%s: ERROR, invalid frame size %dx%d\n", __func__,
						width, height);
		*found_frmsize = preview ?
			s5k5ccgx_get_framesize(frmsizes, num_frmsize,
					S5K5CCGX_PREVIEW_XGA) :
			s5k5ccgx_get_framesize(frmsizes, num_frmsize,
					S5K5CCGX_CAPTURE_3MP);
		BUG_ON(!(*found_frmsize));
	}

	if (preview)
		cam_info("Preview Res Set: %dx%d, index %d\n",
			(*found_frmsize)->width, (*found_frmsize)->height,
			(*found_frmsize)->index);
	else
		cam_info("Capture Res Set: %dx%d, index %d\n",
			(*found_frmsize)->width, (*found_frmsize)->height,
			(*found_frmsize)->index);
}

static int s5k5ccgx_control_stream(struct v4l2_subdev *sd, u32 cmd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

	if (unlikely(cmd != STREAM_STOP))
		return 0;

		cam_info("STREAM STOP!!\n");
		err = s5k5ccgx_set_from_table(sd, "stream_stop",
				&state->regs->stream_stop, 1, 0);

		if (state->runmode == S5K5CCGX_RUNMODE_CAPTURING) {
			if (state->flash_on) {
				s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_NORMAL,
					S5K5CCGX_FLASH_OFF);
				state->flash_on = false;
			}

			state->runmode = S5K5CCGX_RUNMODE_CAPTURE_STOP;
			cam_dbg("Capture Stop!\n");
		}

	CHECK_ERR_N_MSG(err, "failed to stop stream\n");
	return 0;
}

/* PX: Set flash mode */
static int s5k5ccgx_set_flash_mode(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);

	/* movie flash mode should be set when recording is started */
/*	if (state->sensor_mode == SENSOR_MOVIE && !state->recording)
		return 0;*/

	if (state->flash_mode == val) {
		cam_dbg("the same flash mode=%d\n", val);
		return 0;
	}

	if (val == FLASH_MODE_TORCH) {
		s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
				S5K5CCGX_FLASH_ON);
	}

	if (state->flash_mode == FLASH_MODE_TORCH) {
		s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
				S5K5CCGX_FLASH_OFF);
	}

	state->flash_mode = val;
	cam_dbg("Flash mode = %d\n", val);
	return 0;
}

static int s5k5ccgx_check_esd(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err = -EINVAL;
	u16 read_value = 0;

	err = s5k5ccgx_set_from_table(sd, "get_esd_status",
		&state->regs->get_esd_status, 1, 0);
	CHECK_ERR(err);
	err = s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value);
	CHECK_ERR(err);

	if (read_value != 0xAAAA)
		goto esd_out;

	cam_trace("X, No ESD(val=0x%X)\n", read_value);
	return 0;

esd_out:
	cam_err("%s: ESD Shock detected! (val=0x%X)\n\n\n",
				__func__, read_value);
	return -ERESTART;
}

/* returns the real iso currently used by sensor due to lighting
 * conditions, not the requested iso we sent using s_ctrl.
 */
/* PX: */
static inline int s5k5ccgx_get_iso(struct v4l2_subdev *sd, u16 *iso)
{
	struct s5k5ccgx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 iso_gain_table[] = {10, 15, 25, 35};
	u16 iso_table[] = {0, 50, 100, 200, 400};
	int err = -EIO;
	u16 val = 0, gain = 0;
	int i = 0;

	err = s5k5ccgx_set_from_table(sd, "get_iso",
				&state->regs->get_iso, 1, 0);
	err |= s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &val);
	CHECK_ERR(err);

	gain = val * 10 / 256;
	for (i = 0; i < ARRAY_SIZE(iso_gain_table); i++) {
		if (gain < iso_gain_table[i])
			break;
	}

	*iso = iso_table[i];

	cam_dbg("gain=%d, ISO=%d\n", gain, *iso);

	/* We do not restore write mode */

	return 0;
}

/* PX: Set ISO */
static int __used s5k5ccgx_set_iso(struct v4l2_subdev *sd, s32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

retry:
	switch (val) {
	case ISO_AUTO:
	case ISO_50:
	case ISO_100:
	case ISO_200:
	case ISO_400:
		err = s5k5ccgx_set_from_table(sd, "iso",
			state->regs->iso, ARRAY_SIZE(state->regs->iso),
			val);
		break;

	default:
		cam_warn("%s: invalid value, %d\n\n", __func__, val);
		val = ISO_AUTO;
		goto retry;
		break;
	}

	cam_trace("X\n");
	return 0;
}

/* PX: Return exposure time (ms) */
static inline int s5k5ccgx_get_expousretime(struct v4l2_subdev *sd,
						u32 *exp_time)
{
	struct s5k5ccgx_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;
	u16 read_value_lsb = 0;
	u16 read_value_msb = 0;

	err = s5k5ccgx_set_from_table(sd, "get_shutterspeed",
				&state->regs->get_shutterspeed, 1, 0);
	CHECK_ERR(err);

	err = s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value_lsb);
	err |= s5k5ccgx_i2c_read_twobyte(client, 0x0F12, &read_value_msb);
	CHECK_ERR(err);

	*exp_time = (((read_value_msb << 16) | (read_value_lsb & 0xFFFF))
			* 1000) / 400;

	/* We do not restore write mode */

	return 0;

}

/* PX: */
static int s5k5ccgx_get_exif(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	u32 exposure_time = 0;

	/* exposure time */
	state->exif.exp_time_den = 0;
	s5k5ccgx_get_expousretime(sd, &exposure_time);
	/*WARN(!exposure_time, "WARNING: exposure time is 0\n");*/
	state->exif.exp_time_den = 1000 * 1000 / exposure_time;

	/* iso */
	state->exif.iso = 0;
	s5k5ccgx_get_iso(sd, &state->exif.iso);

	/* flash */
	state->exif.flash = 0;
	if (state->flash_mode == FLASH_MODE_AUTO)
		state->exif.flash |= EXIF_FLASH_MODE_AUTO;

	if (state->flash_on)
		state->exif.flash |= EXIF_FLASH_FIRED;

	cam_dbg("EXIF: ex_time_den=%d, iso=%d, flash=0x%02X\n",
		state->exif.exp_time_den, state->exif.iso, state->exif.flash);

	return 0;
}

static int s5k5ccgx_set_preview_size(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

	switch (state->wide_mode) {
	case WIDE_REQ_CHANGE:
		cam_info("%s: Wide Capture setting\n", __func__);
		err = s5k5ccgx_set_from_table(sd, "change_wide_cap",
			&state->regs->change_wide_cap, 1, 0);
		break;

	case WIDE_REQ_RESTORE:
		cam_info("%s:Restore capture setting\n", __func__);
		err = s5k5ccgx_set_from_table(sd, "restore_capture",
				&state->regs->restore_cap, 1, 0);
		/* We do not break */

	default:
		cam_dbg("%s: Set preview size\n", __func__);
		err = s5k5ccgx_set_from_table(sd, "preview_size",
				state->regs->preview_size,
				ARRAY_SIZE(state->regs->preview_size),
				state->preview->index);
		BUG_ON(state->preview->index == S5K5CCGX_PREVIEW_PVGA);
		break;
	}
	CHECK_ERR(err);

	return 0;
}

static int s5k5ccgx_set_preview_start(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;
	/* bool set_size = true; */

	cam_dbg("Camera Preview start, runmode = %d\n", state->runmode);

	if ((state->runmode == S5K5CCGX_RUNMODE_NOTREADY) ||
	    (state->runmode == S5K5CCGX_RUNMODE_CAPTURING)) {
		cam_err("%s: ERROR - Invalid runmode\n", __func__);
		return -EPERM;
	}

	state->focus.status = AF_RESULT_NONE;
	state->focus.preflash = PREFLASH_NONE;

	err = s5k5ccgx_set_preview_size(sd);
	CHECK_ERR_N_MSG(err, "failed to set preview size(%d)\n", err);

	if (state->runmode == S5K5CCGX_RUNMODE_CAPTURE_STOP) {
		err = s5k5ccgx_set_lock(sd, AEAWB_UNLOCK, true);
		CHECK_ERR_N_MSG(err, "fail to set lock\n")

		cam_info("Sending Preview_Return cmd\n");
		err = s5k5ccgx_set_from_table(sd, "preview_return",
					&state->regs->preview_return, 1, 0);
		CHECK_ERR_N_MSG(err, "fail to set Preview_Return (%d)\n", err)
	} else {
		err = s5k5ccgx_set_from_table(sd, "update_preview_setting",
			&state->regs->update_preview_setting, 1, 0);
		CHECK_ERR_N_MSG(err, "failed to update preview(%d)\n", err);
	}

	state->runmode = S5K5CCGX_RUNMODE_RUNNING;

	return 0;
}

static int s5k5ccgx_set_video_preview(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_dbg("Video Preview start, runmode = %d\n", state->runmode);

	if ((state->runmode == S5K5CCGX_RUNMODE_NOTREADY) ||
	    (state->runmode == S5K5CCGX_RUNMODE_CAPTURING)) {
		cam_err("%s: ERROR - Invalid runmode\n", __func__);
		return -EPERM;
	}

	state->focus.status = AF_RESULT_NONE;
	state->focus.preflash = PREFLASH_NONE;

	if (state->hd_videomode) {
		err = S5K5CCGX_BURST_WRITE_REGS(sd, s5k5ccgx_1280_720_Preview);
		CHECK_ERR_N_MSG(err, "failed to write HD regs\n");
#if defined(CONFIG_VIDEO_ANTIBANDING_60Hz)
		err = s5k5ccgx_set_from_table(sd, "antibanding_60hz",
					&state->regs->antibanding_60hz, 1, 0);
		CHECK_ERR(err);

#else
		err = s5k5ccgx_set_from_table(sd, "antibanding_50hz",
					&state->regs->antibanding_50hz, 1, 0);
		CHECK_ERR(err);
#endif
		s5k5ccgx_init_param(sd);
		s5k5ccgx_set_from_table(sd, "hd_first_af_start",
				&state->regs->hd_first_af_start, 1, 0);
	} else {
		err = s5k5ccgx_set_from_table(sd, "preview_size",
				state->regs->preview_size,
				ARRAY_SIZE(state->regs->preview_size),
				state->preview->index);
		CHECK_ERR_N_MSG(err, "failed to set preview size\n");

		err = s5k5ccgx_set_from_table(sd, "update_preview_setting",
			&state->regs->update_preview_setting, 1, 0);
		CHECK_ERR_N_MSG(err, "failed to update preview\n");
	}

	cam_dbg("runmode now RUNNING\n");
	state->runmode = S5K5CCGX_RUNMODE_RUNNING;

	return 0;
}

/* PX: Start capture */
static int s5k5ccgx_set_capture_start(struct v4l2_subdev *sd)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -ENODEV;
	u32 light_level = 0xFFFFFFFF;

	/* Set capture size */
	err = s5k5ccgx_set_capture_size(sd);
	CHECK_ERR_N_MSG(err, "fail to set capture size (%d)\n", err);

	/* Set flash */
	switch (state->flash_mode) {
	case FLASH_MODE_AUTO:
		if (state->focus.preflash == PREFLASH_NONE) {
			s5k5ccgx_get_light_level(sd, &light_level);
			if (light_level >= LOW_LIGHT_LEVEL)
				break;
		} else if (state->focus.preflash == PREFLASH_OFF)
			break;
		/* We do not break. */

	case FLASH_MODE_ON:
		s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_NORMAL,
			S5K5CCGX_FLASH_ON);
		state->flash_on = true;
		/* Full flash start */
		err = s5k5ccgx_set_from_table(sd, "flash_start",
			&state->regs->flash_start, 1, 0);
		break;

	case FLASH_MODE_OFF:
	default:
		break;
	}

	state->focus.preflash = PREFLASH_NONE;
	state->runmode = S5K5CCGX_RUNMODE_CAPTURING;

	/* Send capture start command. */
	cam_dbg("Send Capture_Start cmd\n");
	err = s5k5ccgx_set_from_table(sd, "capture_start",
			state->regs->capture_start,
			ARRAY_SIZE(state->regs->capture_start),
			state->capture->index);
	if (state->scene_mode == SCENE_MODE_NIGHTSHOT)
		msleep(140);

	CHECK_ERR_N_MSG(err, "fail to capture_start (%d)\n", err);

	s5k5ccgx_get_exif(sd);

	return 0;
}

static int s5k5ccgx_s_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	struct s5k5ccgx_state *state = to_state(sd);
	bool is_pre_widesize = false;

	cam_dbg("%s: pixelformat = 0x%x (%c%c%c%c),"
		" colorspace = 0x%x, width = %d, height = %d\n",
		__func__, fmt->fmt.pix.pixelformat,
		fmt->fmt.pix.pixelformat,
		fmt->fmt.pix.pixelformat >> 8,
		fmt->fmt.pix.pixelformat >> 16,
		fmt->fmt.pix.pixelformat >> 24,
		fmt->fmt.pix.colorspace,
		fmt->fmt.pix.width, fmt->fmt.pix.height);

	state->req_fmt = fmt->fmt.pix;
	state->format_mode = fmt->fmt.pix.priv;
	state->wide_mode = WIDE_REQ_NONE;

	if (state->format_mode != V4L2_PIX_FMT_MODE_CAPTURE) {
		if (state->preview &&
		    (state->preview->index == PREVIEW_WIDE_SIZE))
			is_pre_widesize = true;
		else
			is_pre_widesize = false;

		s5k5ccgx_set_framesize(sd, s5k5ccgx_preview_frmsizes,
				ARRAY_SIZE(s5k5ccgx_preview_frmsizes),
				true);

		if (unlikely((state->sensor_mode == SENSOR_CAMERA) &&
		    (state->preview->index == S5K5CCGX_PREVIEW_PVGA))) {
			cam_err("%s: ERROR, invalid preview size\n", __func__);
			return -EINVAL;
		}

		if (is_pre_widesize &&
		    (state->preview->index != PREVIEW_WIDE_SIZE))
			state->wide_mode = WIDE_REQ_RESTORE;
		else if (!is_pre_widesize &&
		    (state->preview->index == PREVIEW_WIDE_SIZE))
			state->wide_mode = WIDE_REQ_CHANGE;
	} else {
		/*
		 * In case of image capture mode,
		 * if the given image resolution is not supported,
		 * use the next higher image resolution. */
		s5k5ccgx_set_framesize(sd, s5k5ccgx_capture_frmsizes,
				ARRAY_SIZE(s5k5ccgx_capture_frmsizes),
				false);
	}

	return 0;
}

static int s5k5ccgx_enum_fmt(struct v4l2_subdev *sd,
			struct v4l2_fmtdesc *fmtdesc)
{
	pr_debug("%s: index = %d\n", __func__, fmtdesc->index);

	if (fmtdesc->index >= ARRAY_SIZE(capture_fmts))
		return -EINVAL;

	memcpy(fmtdesc, &capture_fmts[fmtdesc->index], sizeof(*fmtdesc));

	return 0;
}

static int s5k5ccgx_try_fmt(struct v4l2_subdev *sd, struct v4l2_format *fmt)
{
	int num_entries;
	int i;

	num_entries = ARRAY_SIZE(capture_fmts);

	cam_dbg("%s: pixelformat = 0x%x (%c%c%c%c), num_entries = %d\n",
		__func__, fmt->fmt.pix.pixelformat,
		fmt->fmt.pix.pixelformat,
		fmt->fmt.pix.pixelformat >> 8,
		fmt->fmt.pix.pixelformat >> 16,
		fmt->fmt.pix.pixelformat >> 24,
		num_entries);

	for (i = 0; i < num_entries; i++) {
		if (capture_fmts[i].pixelformat == fmt->fmt.pix.pixelformat) {
			pr_debug("%s: match found, returning 0\n", __func__);
			return 0;
		}
	}

	cam_err("%s: no match found, returning -EINVAL\n", __func__);
	return -EINVAL;
}


static int s5k5ccgx_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	struct s5k5ccgx_state *state = to_state(sd);

	/*
	* The camera interface should read this value, this is the resolution
	* at which the sensor would provide framedata to the camera i/f
	* In case of image capture,
	* this returns the default camera resolution (VGA)
	*/
	if (state->format_mode != V4L2_PIX_FMT_MODE_CAPTURE) {
		if (unlikely(state->preview == NULL)) {
			cam_err("%s: ERROR\n", __func__);
			return -EFAULT;
		}

		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = state->preview->width;
		fsize->discrete.height = state->preview->height;
	} else {
		if (unlikely(state->capture == NULL)) {
			cam_err("%s: ERROR\n", __func__);
			return -EFAULT;
		}

		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = state->capture->width;
		fsize->discrete.height = state->capture->height;
	}

	return 0;
}

static int s5k5ccgx_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct s5k5ccgx_state *state = to_state(sd);

	memcpy(param, &state->strm, sizeof(param));
	return 0;
}

static int s5k5ccgx_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	int err = 0;
	struct s5k5ccgx_state *state = to_state(sd);

	u32 fps = param->parm.capture.timeperframe.denominator /
			param->parm.capture.timeperframe.numerator;

	cam_trace("E fps=%d\n", fps);

	if (fps != state->fps) {
		if (fps < 0 || fps > 30) {
			cam_err("%s: ERROR, invalid frame rate %d\n",
						__func__, fps);
			fps = 30;
		}
		state->req_fps = fps;
	}

	if (state->initialized && (state->scene_mode == SCENE_MODE_NONE)) {
		err = s5k5ccgx_set_frame_rate(sd, state->req_fps);
		CHECK_ERR(err);
		state->fps = state->req_fps;
	}

	return 0;
}

static int s5k5ccgx_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 0;

	if (!state->initialized) {
		cam_err("%s: WARNING, camera not initialized\n", __func__);
		return 0;
	}

	mutex_lock(&state->ctrl_lock);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_EXIF_EXPTIME:
		ctrl->value = state->exif.exp_time_den;
		break;

	case V4L2_CID_CAMERA_EXIF_ISO:
		ctrl->value = state->exif.iso;
		break;

	case V4L2_CID_CAMERA_EXIF_FLASH:
		ctrl->value = state->exif.flash;
		break;

#if !defined(FEATURE_YUV_CAPTURE)
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

	case V4L2_CID_CAM_JPEG_QUALITY:
		ctrl->value = state->jpeg.quality;
		break;

	case V4L2_CID_CAM_JPEG_MEMSIZE:
		ctrl->value = SENSOR_JPEG_SNAPSHOT_MEMSIZE;
		break;
#endif

	case V4L2_CID_CAMERA_AUTO_FOCUS_RESULT:
		ctrl->value = state->focus.status;
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:
	case V4L2_CID_CAMERA_EFFECT:
	case V4L2_CID_CAMERA_CONTRAST:
	case V4L2_CID_CAMERA_SATURATION:
	case V4L2_CID_CAMERA_SHARPNESS:
	case V4L2_CID_CAMERA_OBJ_TRACKING_STATUS:
	case V4L2_CID_CAMERA_SMART_AUTO_STATUS:
	default:
		cam_err("%s: WARNING, unknown Ctrl-ID 0x%x\n",
					__func__, ctrl->id);
		err = 0; /* we return no error. */
		break;
	}

	mutex_unlock(&state->ctrl_lock);

	return err;
}

static int s5k5ccgx_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 1; /* Don't fix err is 1. */

	if (unlikely(state->sensor_mode == SENSOR_MOVIE))
		err = s5k5ccgx_save_ctrl(ctrl);

	if (!state->initialized && ctrl->id != V4L2_CID_CAMERA_SENSOR_MODE) {
		if (!err)
			return 0;

		cam_err("%s: WARNING, camera not initialized. ID = %d(0x%X)\n",
				__func__, ctrl->id - V4L2_CID_PRIVATE_BASE,
				ctrl->id - V4L2_CID_PRIVATE_BASE);
		return 0;
	}

	cam_dbg("%s: ID =%d, val = %d\n",
		__func__, ctrl->id - V4L2_CID_PRIVATE_BASE, ctrl->value);

	if (ctrl->id != V4L2_CID_CAMERA_SET_AUTO_FOCUS)
		mutex_lock(&state->ctrl_lock);

	switch (ctrl->id) {
	case V4L2_CID_CAMERA_SENSOR_MODE:
		err = s5k5ccgx_set_sensor_mode(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_X:
		state->focus.pos_x = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_OBJECT_POSITION_Y:
		state->focus.pos_y = ctrl->value;
		err = 0;
		break;

	case V4L2_CID_CAMERA_TOUCH_AF_START_STOP:
		err = s5k5ccgx_set_touch_af(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_FOCUS_MODE:
		err = s5k5ccgx_set_focus_mode(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_SET_AUTO_FOCUS:
		err = s5k5ccgx_set_af(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_FLASH_MODE:
		err = s5k5ccgx_set_flash_mode(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_BRIGHTNESS:
		err = s5k5ccgx_set_exposure(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_WHITE_BALANCE:
		err = s5k5ccgx_set_from_table(sd, "white balance",
			state->regs->white_balance,
			ARRAY_SIZE(state->regs->white_balance), ctrl->value);
		state->wb_mode = ctrl->value;
		break;

	case V4L2_CID_CAMERA_EFFECT:
		err = s5k5ccgx_set_from_table(sd, "effects",
			state->regs->effect,
			ARRAY_SIZE(state->regs->effect), ctrl->value);
		break;

	case V4L2_CID_CAMERA_METERING:
		err = s5k5ccgx_set_from_table(sd, "metering",
			state->regs->metering,
			ARRAY_SIZE(state->regs->metering), ctrl->value);
		break;

	case V4L2_CID_CAMERA_CONTRAST:
		err = s5k5ccgx_set_from_table(sd, "contrast",
			state->regs->contrast,
			ARRAY_SIZE(state->regs->contrast), ctrl->value);
		break;

	case V4L2_CID_CAMERA_SATURATION:
		err = s5k5ccgx_set_from_table(sd, "saturation",
			state->regs->saturation,
			ARRAY_SIZE(state->regs->saturation), ctrl->value);
		break;

	case V4L2_CID_CAMERA_SHARPNESS:
		err = s5k5ccgx_set_from_table(sd, "sharpness",
			state->regs->sharpness,
			ARRAY_SIZE(state->regs->sharpness), ctrl->value);
		break;

	case V4L2_CID_CAMERA_SCENE_MODE:
		err = s5k5ccgx_set_scene_mode(sd, ctrl->value);
		break;

	case V4L2_CID_CAMERA_AE_LOCK_UNLOCK:
		err = s5k5ccgx_set_ae_lock(sd, ctrl->value, false);
		break;

	case V4L2_CID_CAMERA_AWB_LOCK_UNLOCK:
		err = s5k5ccgx_set_awb_lock(sd, ctrl->value, false);
		break;

	case V4L2_CID_CAMERA_CHECK_ESD:
		err = s5k5ccgx_check_esd(sd);
		break;

	case V4L2_CID_CAMERA_ISO:
		/* we do not break. */
	case V4L2_CID_CAMERA_FRAME_RATE:
	default:
		cam_err("%s: WARNING, unknown Ctrl-ID 0x%x\n",
			__func__, ctrl->id);
		err = 0; /* we return no error. */
		break;
	}

	if (ctrl->id != V4L2_CID_CAMERA_SET_AUTO_FOCUS)
		mutex_unlock(&state->ctrl_lock);

	CHECK_ERR_N_MSG(err, "s_ctrl failed %d\n", err)

	return 0;
}

static int s5k5ccgx_s_ext_ctrl(struct v4l2_subdev *sd,
			      struct v4l2_ext_control *ctrl)
{
	return 0;
}

static int s5k5ccgx_s_ext_ctrls(struct v4l2_subdev *sd,
				struct v4l2_ext_controls *ctrls)
{
	struct v4l2_ext_control *ctrl = ctrls->controls;
	int ret;
	int i;

	for (i = 0; i < ctrls->count; i++, ctrl++) {
		ret = s5k5ccgx_s_ext_ctrl(sd, ctrl);

		if (ret) {
			ctrls->error_idx = i;
			break;
		}
	}

	return ret;
}

#ifdef CONFIG_VIDEO_S5K5CCGX_DEBUG
static void s5k5ccgx_dump_regset(struct s5k5ccgx_regset *regset)
{
	if ((regset->data[0] == 0x00) && (regset->data[1] == 0x2A)) {
		if (regset->size <= 6)
			pr_err("odd regset size %d\n", regset->size);
		pr_info("regset: addr = 0x%02X%02X, data[0,1] = 0x%02X%02X,"
			" total data size = %d\n",
			regset->data[2], regset->data[3],
			regset->data[6], regset->data[7],
			regset->size-6);
	} else {
		pr_info("regset: 0x%02X%02X%02X%02X\n",
			regset->data[0], regset->data[1],
			regset->data[2], regset->data[3]);
		if (regset->size != 4)
			pr_err("odd regset size %d\n", regset->size);
	}
}
#endif

static int s5k5ccgx_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = 0;

	cam_info("stream mode = %d\n", enable);

	switch (enable) {
	case STREAM_MODE_CAM_OFF:
		if (likely(state->pdata->is_mipi))
			err = s5k5ccgx_control_stream(sd, STREAM_STOP);
		break;

	case STREAM_MODE_CAM_ON:
		switch (state->sensor_mode) {
		case SENSOR_CAMERA:
			if (state->format_mode == V4L2_PIX_FMT_MODE_CAPTURE)
				err = s5k5ccgx_set_capture_start(sd);
			else
				err = s5k5ccgx_set_preview_start(sd);
			break;

		case SENSOR_MOVIE:
			err = s5k5ccgx_set_video_preview(sd);
			break;

		default:
			break;
		}
		break;

	case STREAM_MODE_MOVIE_ON:
		state->recording = true;
		if (state->flash_mode != FLASH_MODE_OFF)
			s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
			S5K5CCGX_FLASH_ON);
		break;

	case STREAM_MODE_MOVIE_OFF:
		state->recording = false;
		s5k5ccgx_flash_en(sd, S5K5CCGX_FLASH_MODE_MOVIE,
			S5K5CCGX_FLASH_OFF);
		break;

	default:
		cam_err("%s: ERROR - Invalid stream mode\n", __func__);
		break;
	}

	CHECK_ERR_N_MSG(err, "failed\n");

	return 0;
}

static int s5k5ccgx_reset(struct v4l2_subdev *sd, u32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);

	cam_trace("EX\n");

	s5k5ccgx_return_focus(sd);
	state->initialized = 0;

	if (state->sensor_mode == SENSOR_MOVIE)
		state->hd_videomode = state->hd_videomode ?
					false : true;
	return 0;
}

static int s5k5ccgx_init(struct v4l2_subdev *sd, u32 val)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int err = -EINVAL;

	cam_dbg("%s: start\n", __func__);

	err = s5k5ccgx_init_regs(sd);
	CHECK_ERR_N_MSG(err, "fail to initialize camera device\n");

	if (state->hd_videomode)
		cam_info("HD(720p) mode\n");
	else {
		err = S5K5CCGX_BURST_WRITE_REGS(sd, s5k5ccgx_init_reg);
		CHECK_ERR_N_MSG(err, "fail to init\n");

#if defined(CONFIG_VIDEO_ANTIBANDING_60Hz)
		err = s5k5ccgx_set_from_table(sd, "antibanding_60hz",
					&state->regs->antibanding_60hz, 1, 0);
		CHECK_ERR(err);

#else
		err = s5k5ccgx_set_from_table(sd, "antibanding_50hz",
					&state->regs->antibanding_50hz, 1, 0);
		CHECK_ERR(err);
#endif
	}

	state->runmode = S5K5CCGX_RUNMODE_INIT;

	/* Default state values */
	state->flash_mode = FLASH_MODE_OFF;
	state->scene_mode = SCENE_MODE_NONE;
	state->flash_on = false;
	memset(&state->focus, 0, sizeof(state->focus));

	state->initialized = true;

	if ((state->sensor_mode == SENSOR_MOVIE) && !state->hd_videomode) {
		s5k5ccgx_init_param(sd);
		if (state->fps != state->req_fps)
			err = s5k5ccgx_set_frame_rate(sd, state->req_fps);
	}

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
static int s5k5ccgx_s_config(struct v4l2_subdev *sd,
			int irq, void *platform_data)
{
	struct s5k5ccgx_state *state = to_state(sd);
	int i;
#ifdef CONFIG_LOAD_FILE
	int err = 0;
#endif

	/*
	 * Assign default format and resolution
	 * Use configured default information in platform data
	 * or without them, use default information in driver
	 */

	if (!platform_data) {
		cam_err("%s: ERROR, no platform data\n", __func__);
		return -ENODEV;
	}
	state->pdata = platform_data;

	state->req_fmt.width = state->pdata->default_width;
	state->req_fmt.height = state->pdata->default_height;

	if (!state->pdata->pixelformat)
		state->req_fmt.pixelformat = DEFAULT_PIX_FMT;
	else
		state->req_fmt.pixelformat = state->pdata->pixelformat;

	if (!state->pdata->freq)
		state->freq = DEFAULT_MCLK;	/* 24MHz default */
	else
		state->freq = state->pdata->freq;

	state->preview = NULL;
	state->capture = NULL;
	state->sensor_mode = SENSOR_CAMERA;
	state->hd_videomode = false;
	state->format_mode = V4L2_PIX_FMT_MODE_PREVIEW;
	state->fps = state->req_fps = 0;

	for (i = 0; i < ARRAY_SIZE(s5k5ccgx_ctrls); i++)
		s5k5ccgx_ctrls[i].value = s5k5ccgx_ctrls[i].default_value;

	if (state->pdata->is_flash_on())
		state->ignore_flash = true;

	state->jpeg.enable = 0;
	state->jpeg.quality = 100;
	state->jpeg.main_offset = 1280; /* 0x500 */

	/* Maximum size 2048 * 1536 * 2 = 6291456 */
	state->jpeg.main_size = SENSOR_JPEG_SNAPSHOT_MEMSIZE;

	state->jpeg.thumb_offset = 636; /* 0x27C */
	state->jpeg.thumb_size = 320 * 240 * 2; /* 320 * 240 * 2 = 153600 */

#ifdef CONFIG_LOAD_FILE
	err = loadFile();
	if (unlikely(err < 0)) {
		cam_err("failed to load file ERR=%d\n", err);
		return err;
	}
#endif

	return 0;
}

static const struct v4l2_subdev_core_ops s5k5ccgx_core_ops = {
	.init = s5k5ccgx_init,	/* initializing API */
	.s_config = s5k5ccgx_s_config,	/* Fetch platform data */
	.g_ctrl = s5k5ccgx_g_ctrl,
	.s_ctrl = s5k5ccgx_s_ctrl,
	.s_ext_ctrls = s5k5ccgx_s_ext_ctrls,
	.reset = s5k5ccgx_reset,
};

static const struct v4l2_subdev_video_ops s5k5ccgx_video_ops = {
	.s_fmt = s5k5ccgx_s_fmt,
	.enum_framesizes = s5k5ccgx_enum_framesizes,
	.enum_fmt = s5k5ccgx_enum_fmt,
	.try_fmt = s5k5ccgx_try_fmt,
	.g_parm = s5k5ccgx_g_parm,
	.s_parm = s5k5ccgx_s_parm,
	.s_stream = s5k5ccgx_s_stream,
};

static const struct v4l2_subdev_ops s5k5ccgx_ops = {
	.core = &s5k5ccgx_core_ops,
	.video = &s5k5ccgx_video_ops,
};

/* PX: */
ssize_t s5k5ccgx_camera_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char *cam_type = "SLSI_S5K5CCGX";
	cam_info("%s\n", __func__);

	return sprintf(buf, "%s\n", cam_type);
}

static DEVICE_ATTR(camera_type, S_IRUGO, s5k5ccgx_camera_type_show, NULL);

/*
 * s5k5ccgx_probe
 * Fetching platform data is being done with s_config subdev call.
 * In probe routine, we just register subdev device
 */
static int s5k5ccgx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct s5k5ccgx_state *state;

	state = kzalloc(sizeof(struct s5k5ccgx_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;

	mutex_init(&state->ctrl_lock);
	mutex_init(&state->af_lock);
	init_completion(&state->af_complete);

	state->runmode = S5K5CCGX_RUNMODE_NOTREADY;
	sd = &state->sd;
	strcpy(sd->name, S5K5CCGX_DRIVER_NAME);

	/* Registering subdev */
	v4l2_i2c_subdev_init(sd, client, &s5k5ccgx_ops);

	if (device_create_file(&client->dev, &dev_attr_camera_type) < 0) {
		cam_warn("%s: WARNING, failed to create device file, %s\n",
			__func__, dev_attr_camera_type.attr.name);
	}

	cam_dbg("%s %s: driver probed!!\n", dev_driver_string(&client->dev),
		dev_name(&client->dev));

	return 0;
}

static int s5k5ccgx_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5ccgx_state *state = to_state(sd);

	if (state->initialized)
		s5k5ccgx_return_focus(sd);

	device_remove_file(&client->dev, &dev_attr_camera_type);

	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&state->ctrl_lock);
	mutex_destroy(&state->af_lock);
	kfree(state);

#ifdef CONFIG_LOAD_FILE
	large_file ? vfree(testBuf) : kfree(testBuf);
	large_file = 0;
	testBuf = NULL;
#endif

	cam_dbg("%s %s: driver removed!!\n", dev_driver_string(&client->dev),
			dev_name(&client->dev));
	return 0;
}

static const struct i2c_device_id s5k5ccgx_id[] = {
	{ S5K5CCGX_DRIVER_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, s5k5ccgx_id);

static struct v4l2_i2c_driver_data v4l2_i2c_data = {
	.name = S5K5CCGX_DRIVER_NAME,
	.probe = s5k5ccgx_probe,
	.remove = s5k5ccgx_remove,
	.id_table = s5k5ccgx_id,
};

MODULE_DESCRIPTION("LSI S5K5CCGX 3MP SOC camera driver");
MODULE_AUTHOR("Dong-Seong Lim <dongseong.lim@samsung.com>");
MODULE_LICENSE("GPL");
