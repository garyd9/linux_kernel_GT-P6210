/*
 * Driver for S5K5BAFX 2M ISP from Samsung
 *
 * Copyright (c) 2011, Samsung Electronics. All rights reserved
 * Author: dongseong.lim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __S5K5BAFX_H
#define __S5K5BAFX_H

#include <linux/types.h>

#define S5K5BAFX_DRIVER_NAME	"S5K5BAFX"

/************************************
 * FEATURE DEFINITIONS
 ************************************/
/* #define S5K5BAFX_USLEEP */
#define S5K5BAFX_BURST_MODE
/* #define CONFIG_LOAD_FILE */
/* #define SUPPORT_FACTORY_TEST */

/** Debuging Feature **/
/* #define CONFIG_CAM_DEBUG */
/* #define CONFIG_CAM_TRACE *//* Enable it with CONFIG_CAM_DEBUG */
/***********************************/

#define TAG_NAME	"["S5K5BAFX_DRIVER_NAME"]"" "
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

enum s5k5bafx_fps_index {
	I_FPS_0,
	I_FPS_7,
	I_FPS_10,
	I_FPS_12,
	I_FPS_15,
	I_FPS_25,
	I_FPS_30,
	I_FPS_MAX,
};
	
struct s5k5bafx_framesize {
	u32 width;
	u32 height;
};

struct s5k5bafx_fps {
	u32 index;
	u32 fps;
};

struct s5k5bafx_exif {
	u32 exp_time_den;
	u32 shutter_speed;
	u16 iso;
};

#ifdef CONFIG_LOAD_FILE
struct s5k5bafx_regset_table {
	const u32	*reg;
	int		array_size;
	char		*name;
};

#define S5K5BAFX_REGSET(x, y)		\
	[(x)] = {					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
		.name		= #y,			\
}

#define S5K5BAFX_REGSET_TABLE(y)		\
	{					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
		.name		= #y,			\
}
#else
struct s5k5bafx_regset_table {
	const u32	*reg;
	int		array_size;
};

#define S5K5BAFX_REGSET(x, y)		\
	[(x)] = {					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
}

#define S5K5BAFX_REGSET_TABLE(y)		\
	{					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
}
#endif

#define EV_MIN_VLAUE	EV_MINUS_4
#define GET_EV_INDEX(EV)	((EV) - (EV_MIN_VLAUE))

struct s5k5bafx_regs {
	struct s5k5bafx_regset_table ev[GET_EV_INDEX(EV_MAX_V4L2)];
	struct s5k5bafx_regset_table blur[BLUR_LEVEL_MAX];
	/* struct s5k5bafx_regset_table capture_size[S5K5BAFX_CAPTURE_MAX];*/
	struct s5k5bafx_regset_table preview_start;
	struct s5k5bafx_regset_table capture_start;
	struct s5k5bafx_regset_table fps[I_FPS_MAX];
	struct s5k5bafx_regset_table init;	/* Used */
	struct s5k5bafx_regset_table init_vt;	/* Used */
	struct s5k5bafx_regset_table init_vt_wifi;	/* Used */
	struct s5k5bafx_regset_table init_recording;	/* Used */
	struct s5k5bafx_regset_table get_light_level;
	struct s5k5bafx_regset_table get_iso;
	struct s5k5bafx_regset_table get_shutterspeed;
	struct s5k5bafx_regset_table stream_stop;	/* Used */
	struct s5k5bafx_regset_table dtp_on;
	struct s5k5bafx_regset_table dtp_off;
};

/*
 * Driver information
 */
struct s5k5bafx_state {
	struct v4l2_subdev sd;
	struct s5k5bafx_platform_data *pdata;
	/*
	 * req_fmt is the requested format from the application.
	 * set_fmt is the output format of the camera. Finally FIMC
	 * converts the camera output(set_fmt) to the requested format
	 * with hardware scaler.
	 */
	struct v4l2_pix_format req_fmt;
	struct s5k5bafx_framesize default_frmsizes;
	struct s5k5bafx_framesize preview_frmsizes;
	struct s5k5bafx_framesize capture_frmsizes;
	struct s5k5bafx_exif exif;

	enum v4l2_sensor_mode sensor_mode;
	s32 vt_mode;
	s32 check_dataline;
	u32 req_fps;
	u32 set_fps;
	u32 initialized;
	const struct s5k5bafx_regs *regs;
};

static inline struct s5k5bafx_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5bafx_state, sd);
}

static inline void debug_msleep(u32 msecs)
{
	cam_dbg("delay for %dms\n", msecs);
	msleep(msecs);
}

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

#define TUNING_FILE_PATH "/mnt/sdcard/s5k5bafx_regs.h"
#endif

/*********** Sensor specific ************/
/* #define S5K5BAFX_100MS_DELAY	0xAA55AA5F */
/* #define S5K5BAFX_10MS_DELAY	0xAA55AA5E */
#define S5K5BAFX_DELAY		0xFFFF0000
#define S5K5BAFX_DEF_APEX_DEN	100
	
/* Register address */
#define REG_PAGE_SHUTTER    0x7000
#define REG_ADDR_SHUTTER    0x14D0
#define REG_PAGE_ISO        0x7000
#define REG_ADDR_ISO        0x14C8
	
#include  "s5k5bafx_regs.h"

#endif /* __S5K5BAFX_H */
