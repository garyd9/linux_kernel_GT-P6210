/*
 * Driver for SR200PC20 2M ISP from Samsung
 *
 * Copyright (c) 2011, Samsung Electronics. All rights reserved
 * Author: dongseong.lim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __SR200PC20_H
#define __SR200PC20_H

#include <linux/types.h>

#define SR200PC20_DRIVER_NAME	"SR200PC20"

/************************************
 * FEATURE DEFINITIONS
 ************************************/
/* #define SR200PC20_USLEEP */
/* #define CONFIG_LOAD_FILE */
#define SUPPORT_FACTORY_TEST

/** Debuging Feature **/
#define CONFIG_CAM_DEBUG
#define CONFIG_CAM_TRACE /* Enable it with CONFIG_CAM_DEBUG */
/***********************************/

#define TAG_NAME	"["SR200PC20_DRIVER_NAME"]"" "
#define cam_err(fmt, ...)	\
	printk(KERN_ERR TAG_NAME fmt, ##__VA_ARGS__)
#define cam_warn(fmt, ...)	\
	printk(KERN_WARNING TAG_NAME fmt, ##__VA_ARGS__)
#define cam_info(fmt, ...)	\
	printk(KERN_INFO TAG_NAME fmt, ##__VA_ARGS__)

#if defined(CONFIG_CAM_DEBUG)
#define cam_dbg(fmt, ...)	\
	printk(KERN_DEBUG TAG_NAME fmt, ##__VA_ARGS__)
#else
#define cam_dbg(fmt, ...)
#endif /* CONFIG_CAM_DEBUG */

#if defined(CONFIG_CAM_DEBUG) && defined(CONFIG_CAM_TRACE)
#define cam_trace(fmt, ...)	cam_dbg("%s: " fmt, __func__, ##__VA_ARGS__);
#else
#define cam_trace(fmt, ...)
#endif

#define CHECK_ERR(x)	do { if (unlikely((x) < 0)) return x; } while (0)
#define CHECK_ERR_N_MSG(x, fmt, ...) \
	if (unlikely((x) < 0)) { \
		cam_err("%s: ERROR, " fmt, __func__, ##__VA_ARGS__); \
		return x; \
	}


enum stream_cmd {
	STREAM_STOP,
	STREAM_START,
};

enum sr200pc20_fps_index {
	I_FPS_0,
	I_FPS_7,
	I_FPS_10,
	I_FPS_12,
	I_FPS_15,
	I_FPS_25,
	I_FPS_30,
	I_FPS_MAX,
};

struct sr200pc20_framesize {
	u32 width;
	u32 height;
};

struct sr200pc20_fps {
	u32 index;
	u32 fps;
};

struct sr200pc20_exif {
	u16 exp_time_den;
	u16 iso;
	u32 shutter_speed;
};

typedef struct regs_array_type {
	u16 subaddr;
	u16 value;
} regs_short_t;

#ifdef CONFIG_LOAD_FILE
struct sr200pc20_regset_table {
	const regs_short_t	*reg;
	int			array_size;
	char			*name;
};

#define SR200PC20_REGSET(x, y)		\
	[(x)] = {					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
		.name		= #y,			\
}

#define SR200PC20_REGSET_TABLE(y)		\
	{					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
		.name		= #y,			\
}
#else
struct sr200pc20_regset_table {
	const regs_short_t	*reg;
	int			array_size;
};

#define SR200PC20_REGSET(x, y)		\
	[(x)] = {					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
}

#define SR200PC20_REGSET_TABLE(y)		\
	{					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
}
#endif

#define EV_MIN_VLAUE	EV_MINUS_4
#define GET_EV_INDEX(EV)	((EV) - (EV_MIN_VLAUE))

struct sr200pc20_regs {
	struct sr200pc20_regset_table ev[GET_EV_INDEX(EV_MAX_V4L2)];
	struct sr200pc20_regset_table blur[BLUR_LEVEL_MAX];
	/* struct sr200pc20_regset_table capture_size[SR200PC20_CAPTURE_MAX];*/
	struct sr200pc20_regset_table preview_start;
	struct sr200pc20_regset_table capture_start;
	struct sr200pc20_regset_table fps[I_FPS_MAX];
	struct sr200pc20_regset_table init;
	struct sr200pc20_regset_table init_vt;
	struct sr200pc20_regset_table init_vt_wifi;
	struct sr200pc20_regset_table init_recording;
	struct sr200pc20_regset_table get_light_level;
	struct sr200pc20_regset_table stream_stop;
	struct sr200pc20_regset_table dtp_on;
	struct sr200pc20_regset_table dtp_off;
};

/*
 * Driver information
 */
struct sr200pc20_state {
	struct v4l2_subdev sd;
	struct sr200pc20_platform_data *pdata;
	/*
	 * req_fmt is the requested format from the application.
	 * set_fmt is the output format of the camera. Finally FIMC
	 * converts the camera output(set_fmt) to the requested format
	 * with hardware scaler.
	 */
	struct v4l2_pix_format req_fmt;
	struct sr200pc20_framesize default_frmsizes;
	struct sr200pc20_framesize preview_frmsizes;
	struct sr200pc20_framesize capture_frmsizes;
	struct sr200pc20_exif exif;

	enum v4l2_sensor_mode sensor_mode;
	s32 vt_mode;
	s32 check_dataline;
	u32 req_fps;
	u32 set_fps;
	u32 initialized;
	const struct sr200pc20_regs *regs;
};

static inline struct sr200pc20_state *to_state(struct v4l2_subdev *sd) {
	return container_of(sd, struct sr200pc20_state, sd);
}

static inline void debug_msleep(u32 msecs)
{
	cam_dbg("delay for %dms\n", msecs);
	msleep(msecs);
}

#define DELAY_SEQ               0xFF

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

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
#if 0
#define dbg_setfile(fmt, ...)	\
	printk(KERN_ERR TAG_NAME fmt, ##__VA_ARGS__)
#else
#define dbg_setfile(fmt, ...)
#endif /* 0 */

#ifdef CONFIG_VIDEO_SR200PC20_P2
#define TUNING_FILE_PATH "/mnt/sdcard/sr200pc20_regs-p2.h"
#elif defined(CONFIG_VIDEO_SR200PC20_P4W)
#define TUNING_FILE_PATH "/mnt/sdcard/sr200pc20_regs-p4w.h"
#else
#define TUNING_FILE_PATH NULL
#endif /* CONFIG_VIDEO_SR200PC20_P2 */

#endif /* CONFIG_LOAD_FILE */

#ifdef CONFIG_VIDEO_SR200PC20_P2
#include  "sr200pc20_regs-p2.h"
#elif defined(CONFIG_VIDEO_SR200PC20_P4W)
#include  "sr200pc20_regs-p4w.h"
#else
#include  "sr200pc20_regs.h"
#endif

#endif /* __SR200PC20_H */

