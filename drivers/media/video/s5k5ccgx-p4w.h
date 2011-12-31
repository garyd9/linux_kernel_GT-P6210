/* drivers/media/video/s5k5ccgx.h
 *
 * Driver for s5k5ccgx (3MP Camera) from SEC(LSI), firmware EVT1.1
 *
 * Copyright (C) 2010, SAMSUNG ELECTRONICS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __S5K5CCGX_P4W_H__
#define __S5K5CCGX_P4W_H__

#define S5K5CCGX_DRIVER_NAME	"S5K5CCGX"

#define S5K5CCGX_VERSION_1_1	0x11

#define S5K5CCGX_DELAY		0xFFFF0000

/************************************
 * FEATURE DEFINITIONS
 ************************************/
#define FEATURE_YUV_CAPTURE
/* #define CONFIG_LOAD_FILE */ /* for tuning */

/** Debuging Feature **/
#define CAM_TRACE /* It should be activated with DEBUG */
/* #define CAM_AF_DEBUG */ /* It should be activated with DEBUG */
/* #define DEBUG_WRITE_REGS */
/***********************************/

#ifdef CONFIG_VIDEO_S5K5CCGX_DEBUG
enum {
	S5K5CCGX_DEBUG_I2C		= 1U << 0,
	S5K5CCGX_DEBUG_I2C_BURSTS	= 1U << 1,
};
static uint32_t s5k5ccgx_debug_mask = S5K5CCGX_DEBUG_I2C_BURSTS;
module_param_named(debug_mask, s5k5ccgx_debug_mask, uint, S_IWUSR | S_IRUGO);

#define s5k5ccgx_debug(mask, x...) \
	do { \
		if (s5k5ccgx_debug_mask & mask) \
			pr_info(x);	\
	} while (0)
#else
#define s5k5ccgx_debug(mask, x...)
#endif

#define VOID_SYMBOL(symbol)	\
	do {			\
		(void)(symbol);	\
	} while (0)

#define TAG_NAME	"["S5K5CCGX_DRIVER_NAME"]"" "
#define cam_err(fmt, ...)	\
	printk(KERN_ERR TAG_NAME fmt, ##__VA_ARGS__)
#define cam_warn(fmt, ...)	\
	printk(KERN_WARNING TAG_NAME fmt, ##__VA_ARGS__)
#define cam_info(fmt, ...)	\
	printk(KERN_INFO TAG_NAME fmt, ##__VA_ARGS__)

#if defined(DEBUG)
#define cam_dbg(fmt, ...)	\
	printk(KERN_DEBUG TAG_NAME fmt, ##__VA_ARGS__)
#else
#define cam_dbg(fmt, ...)
#endif

#if defined(DEBUG) && defined(CAM_TRACE)
#define cam_trace(fmt, ...)	cam_dbg("%s: " fmt, __func__, ##__VA_ARGS__);
#else
#define cam_trace(fmt, ...)
#endif

#if defined(DEBUG) && defined(CAM_AF_DEBUG)
#define af_dbg(fmt, ...)	cam_dbg(fmt, ##__VA_ARGS__);
#else
#define af_dbg(fmt, ...)
#endif

#define CHECK_ERR(x)	do { if (unlikely((x) < 0)) return x; } while (0)
#define CHECK_ERR_N_MSG(x, fmt, ...) \
	if (unlikely((x) < 0)) { \
		cam_err("%s: ERROR, " fmt, __func__, ##__VA_ARGS__); \
		return x; \
	}

#ifdef CONFIG_LOAD_FILE
#define S5K5CCGX_BURST_WRITE_REGS(sd, A) ({ \
	int ret; \
		cam_info("BURST_WRITE_REGS: reg_name=%s from setfile\n", #A); \
		ret = s5k5ccgx_write_regs_from_sd(sd, #A); \
		ret; \
	})
#else
#define S5K5CCGX_BURST_WRITE_REGS(sd, A) \
	s5k5ccgx_burst_write_regs(sd, A, (sizeof(A) / sizeof(A[0])), #A)
#endif

/* result values returned to HAL */
enum af_result_status {
	AF_RESULT_NONE = 0x00,
	AF_RESULT_FAILED = 0x01,
	AF_RESULT_SUCCESS = 0x02,
	AF_RESULT_CANCELLED = 0x04,
	AF_RESULT_DOING = 0x08
};

enum af_operation_status {
	AF_NONE = 0,
	AF_START,
	AF_CANCEL,
};

enum preflash_status {
	PREFLASH_NONE = 0,
	PREFLASH_OFF,
	PREFLASH_ON,
};

enum s5k5ccgx_oprmode {
	S5K5CCGX_OPRMODE_VIDEO = 0,
	S5K5CCGX_OPRMODE_IMAGE = 1,
};
/*
enum s5k5ccgx_cammode {
	CAM_CAMEREA = 0,
	CAM_CAMCORDER = 1,
};
*/

enum stream_cmd {
	STREAM_STOP,
	STREAM_START,
};

enum wide_req_cmd {
	WIDE_REQ_NONE,
	WIDE_REQ_CHANGE,
	WIDE_REQ_RESTORE,
};

enum s5k5ccgx_preview_frame_size {
	S5K5CCGX_PREVIEW_QCIF = 0,	/* 176x144 */
	S5K5CCGX_PREVIEW_320x240,	/* 320x240 */
	S5K5CCGX_PREVIEW_CIF,		/* 352x288 */
	S5K5CCGX_PREVIEW_528x432,	/* 528x432 */
	S5K5CCGX_PREVIEW_VGA,		/* 640x480 */
	S5K5CCGX_PREVIEW_D1,		/* 720x480 */
	S5K5CCGX_PREVIEW_SVGA,		/* 800x600 */
#ifdef CONFIG_VIDEO_S5K5CCGX_P2
	S5K5CCGX_PREVIEW_1024x552,	/* 1024x552, ? */
#else
	S5K5CCGX_PREVIEW_1024x576,	/* 1024x576, 16:9 */
#endif
	S5K5CCGX_PREVIEW_1024x616,	/* 1024x616, ? */
	S5K5CCGX_PREVIEW_XGA,		/* 1024x768*/
	S5K5CCGX_PREVIEW_PVGA,		/* 1280*720*/
	S5K5CCGX_PREVIEW_SXGA,		/* 1280x1024*/
	S5K5CCGX_PREVIEW_MAX,
};

/* Capture Size List: Capture size is defined as below.
 *
 *	S5K5CCGX_CAPTURE_VGA:		640x480
 *	S5K5CCGX_CAPTURE_WVGA:		800x480
 *	S5K5CCGX_CAPTURE_SVGA:		800x600
 *	S5K5CCGX_CAPTURE_WSVGA:		1024x600
 *	S5K5CCGX_CAPTURE_1MP:		1280x960
 *	S5K5CCGX_CAPTURE_W1MP:		1600x960
 *	S5K5CCGX_CAPTURE_2MP:		UXGA - 1600x1200
 *	S5K5CCGX_CAPTURE_W2MP:		35mm Academy Offset Standard 1.66
 *					2048x1232, 2.4MP
 *	S5K5CCGX_CAPTURE_3MP:		QXGA  - 2048x1536
 *	S5K5CCGX_CAPTURE_W4MP:		WQXGA - 2560x1536
 *	S5K5CCGX_CAPTURE_5MP:		2560x1920
 */

enum s5k5ccgx_capture_frame_size {
	S5K5CCGX_CAPTURE_VGA = 0,	/* 640x480 */
	S5K5CCGX_CAPTURE_W2MP,		/* 35mm Academy Offset Standard 1.66 */
					/* 2048x1232, 2.4MP */
	S5K5CCGX_CAPTURE_3MP,		/* QXGA  - 2048x1536 */
	S5K5CCGX_CAPTURE_MAX,
};

#ifdef CONFIG_VIDEO_S5K5CCGX_P2
#define PREVIEW_WIDE_SIZE	S5K5CCGX_PREVIEW_1024x552
#else
#define PREVIEW_WIDE_SIZE	S5K5CCGX_PREVIEW_1024x576
#define CAPTURE_WIDE_SIZE	S5K5CCGX_CAPTURE_W2MP
#endif

enum s5k5ccgx_fps_index {
	I_FPS_0,
	I_FPS_7,
	I_FPS_10,
	I_FPS_12,
	I_FPS_15,
	I_FPS_25,
	I_FPS_30,
	I_FPS_MAX,
};

struct s5k5ccgx_control {
	u32 id;
	s32 value;
	s32 default_value;
};

#define S5K5CCGX_INIT_CONTROL(ctrl_id, default_val) \
	{					\
		.id = ctrl_id,			\
		.value = default_val,		\
		.default_value = default_val,	\
	}

struct s5k5ccgx_framesize {
	u32 index;
	u32 width;
	u32 height;
};

#define FRM_RATIO(framesize) \
	(((framesize)->width) * 10 / ((framesize)->height))

struct s5k5ccgx_fps {
	u32 index;
	u32 fps;
};

struct s5k5ccgx_version {
	u32 major;
	u32 minor;
};

struct s5k5ccgx_date_info {
	u32 year;
	u32 month;
	u32 date;
};

enum s5k5ccgx_runmode {
	S5K5CCGX_RUNMODE_NOTREADY,
	S5K5CCGX_RUNMODE_INIT,
	/*S5K5CCGX_RUNMODE_IDLE,*/
	S5K5CCGX_RUNMODE_RUNNING, /* previewing */
	S5K5CCGX_RUNMODE_RUNNING_STOP,
	S5K5CCGX_RUNMODE_CAPTURING,
	S5K5CCGX_RUNMODE_CAPTURE_STOP,
};

struct s5k5ccgx_firmware {
	u32 addr;
	u32 size;
};

struct s5k5ccgx_jpeg_param {
	u32 enable;
	u32 quality;
	u32 main_size;		/* Main JPEG file size */
	u32 thumb_size;		/* Thumbnail file size */
	u32 main_offset;
	u32 thumb_offset;
	/* u32 postview_offset; */
};

struct s5k5ccgx_position {
	s32 x;
	s32 y;
};

struct s5k5ccgx_rect {
	s32 x;
	s32 y;
	u32 width;
	u32 height;
};

struct gps_info_common {
	u32 direction;
	u32 dgree;
	u32 minute;
	u32 second;
};

struct s5k5ccgx_gps_info {
	u8 gps_buf[8];
	u8 altitude_buf[4];
	s32 gps_timeStamp;
};

struct s5k5ccgx_focus {
	u32 start:1;	/* enum v4l2_auto_focus*/
	u32 ae_lock:1;
	u32 awb_lock:1;
	u32 touch:1;
	u32 af_cancel:1;

	enum v4l2_focusmode mode;
	enum af_result_status status;
	enum preflash_status preflash;

	u32 pos_x;
	u32 pos_y;
};

struct s5k5ccgx_exif {
	u16 exp_time_den;
	u16 iso;
	u16 flash;

	/*int bv;*/		/* brightness */
	/*int ebv;*/		/* exposure bias */
};

struct s5k5ccgx_regset {
	u32 size;
	u8 *data;
};

#ifdef CONFIG_LOAD_FILE
#define DEBUG_WRITE_REGS
struct s5k5ccgx_regset_table {
	const char	*const name;
};

#define S5K5CCGX_REGSET(x, y)		\
	[(x)] = {			\
		.name		= #y,	\
}

#define S5K5CCGX_REGSET_TABLE(y)	\
	{				\
		.name		= #y,	\
}
#else
struct s5k5ccgx_regset_table {
	const u32	*const reg;
	const u32	array_size;
#ifdef DEBUG_WRITE_REGS
	const char	*const name;
#endif
};

#ifdef DEBUG_WRITE_REGS
#define S5K5CCGX_REGSET(x, y)		\
	[(x)] = {					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
		.name		= #y,			\
}

#define S5K5CCGX_REGSET_TABLE(y)		\
	{					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
		.name		= #y,			\
}
#else
#define S5K5CCGX_REGSET(x, y)		\
	[(x)] = {					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
}

#define S5K5CCGX_REGSET_TABLE(y)		\
	{					\
		.reg		= (y),			\
		.array_size	= ARRAY_SIZE((y)),	\
}
#endif /* DEBUG_WRITE_REGS */
#endif /* CONFIG_LOAD_FILE */

#define EV_MIN_VLAUE		EV_MINUS_4
#define GET_EV_INDEX(EV)	((EV) - (EV_MIN_VLAUE))

struct s5k5ccgx_regs {
	struct s5k5ccgx_regset_table ev[GET_EV_INDEX(EV_MAX_V4L2)];
	struct s5k5ccgx_regset_table metering[METERING_MAX];
	struct s5k5ccgx_regset_table iso[ISO_MAX];
	struct s5k5ccgx_regset_table effect[IMAGE_EFFECT_MAX];
	struct s5k5ccgx_regset_table white_balance[WHITE_BALANCE_MAX];
	struct s5k5ccgx_regset_table preview_size[S5K5CCGX_PREVIEW_MAX];
	struct s5k5ccgx_regset_table capture_start[S5K5CCGX_CAPTURE_MAX];
	struct s5k5ccgx_regset_table scene_mode[SCENE_MODE_MAX];
	struct s5k5ccgx_regset_table saturation[SATURATION_MAX];
	struct s5k5ccgx_regset_table contrast[CONTRAST_MAX];
	struct s5k5ccgx_regset_table sharpness[SHARPNESS_MAX];
	struct s5k5ccgx_regset_table fps[I_FPS_MAX];
	struct s5k5ccgx_regset_table preview_return;
	struct s5k5ccgx_regset_table flash_start;
	struct s5k5ccgx_regset_table flash_end;
	struct s5k5ccgx_regset_table af_pre_flash_start;
	struct s5k5ccgx_regset_table af_pre_flash_end;
	struct s5k5ccgx_regset_table flash_ae_set;
	struct s5k5ccgx_regset_table flash_ae_clear;
	struct s5k5ccgx_regset_table ae_lock_on;
	struct s5k5ccgx_regset_table ae_lock_off;
	struct s5k5ccgx_regset_table awb_lock_on;
	struct s5k5ccgx_regset_table awb_lock_off;
	struct s5k5ccgx_regset_table restore_cap;
	struct s5k5ccgx_regset_table change_wide_cap;
#ifdef CONFIG_VIDEO_S5K5CCGX_P8
	struct s5k5ccgx_regset_table set_lowlight_cap;
#endif
	struct s5k5ccgx_regset_table af_macro_mode;
	struct s5k5ccgx_regset_table af_normal_mode;
#if !defined(CONFIG_VIDEO_S5K5CCGX_P2)
	struct s5k5ccgx_regset_table af_night_normal_mode;
#endif
	struct s5k5ccgx_regset_table hd_af_start;
	struct s5k5ccgx_regset_table hd_first_af_start;
	struct s5k5ccgx_regset_table single_af_start;
	struct s5k5ccgx_regset_table init_reg;
	struct s5k5ccgx_regset_table get_light_level;
	struct s5k5ccgx_regset_table get_esd_status;
	struct s5k5ccgx_regset_table get_iso;
	struct s5k5ccgx_regset_table get_ae_stable;
	struct s5k5ccgx_regset_table get_shutterspeed;
	struct s5k5ccgx_regset_table update_preview_setting;
	struct s5k5ccgx_regset_table stream_stop;
#ifdef CONFIG_VIDEO_S5K5CCGX_P8
	struct s5k5ccgx_regset_table antibanding;
#endif
};

struct s5k5ccgx_state {
	struct s5k5ccgx_platform_data *pdata;
	struct v4l2_subdev sd;
	struct v4l2_pix_format req_fmt;		/* used */
	struct v4l2_fract timeperframe;
	struct s5k5ccgx_jpeg_param jpeg;	/* ? */
	struct s5k5ccgx_version fw;
	struct s5k5ccgx_version prm;
	struct s5k5ccgx_date_info dateinfo;
	struct s5k5ccgx_position position;
	struct v4l2_streamparm strm;		/* ? */
	struct s5k5ccgx_gps_info gps_info;
	struct mutex ctrl_lock;
	struct mutex af_lock; /* Used */
	struct completion af_complete;
	enum s5k5ccgx_runmode runmode;		/* Used */
	enum s5k5ccgx_oprmode oprmode;		/* Not Used */
	enum v4l2_sensor_mode sensor_mode;	/* Used */
	enum v4l2_pix_format_mode format_mode;	/* Used */
	enum v4l2_flash_mode flash_mode;        /* Used ? */
	enum v4l2_scene_mode scene_mode;        /* Used */
	enum v4l2_wb_mode wb_mode;		/* Used */
	enum wide_req_cmd wide_mode;		/* Used */
	bool flash_on;				/* Used */
	s32 vt_mode;			/* Used */
	bool recording;			/* Used */
	bool hd_videomode;		/* Used */
	enum af_operation_status af_status;
	struct s5k5ccgx_framesize *preview;     /* Used */
	struct s5k5ccgx_framesize *capture;     /* Used */
	struct s5k5ccgx_focus focus;		/* Used */
	struct s5k5ccgx_exif exif;		/* Used */
	u32 req_fps;				/* Used */
	u32 fps;				/* Used */
/*	s32 preview_index;
	s32 capture_index; */
	int sensor_version;
	int freq;		/* MCLK in Hz */
	int check_dataline;		/* ? */
	int check_previewdata;		/* ? */
	bool torch_on;
	bool sensor_af_in_low_light_mode;
	bool flash_state_on_previous_capture;	/* ? */
	bool initialized;
	bool restore_preview_size_needed;	/*Not used */
	int one_frame_delay_ms;			/* ? */
#if !defined(CONFIG_VIDEO_S5K5CCGX_P2)
	u32 light_level;	/* light level */
#endif
	const struct s5k5ccgx_regs *regs;
};

static inline struct  s5k5ccgx_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5ccgx_state, sd);
}

#define FORMAT_FLAGS_COMPRESSED		0x3

/* JPEG MEMORY SIZE */
//#define SENSOR_JPEG_SNAPSHOT_MEMSIZE	0x410580
#define SENSOR_JPEG_OUTPUT_MAXSIZE	0x29999A /*2726298bytes, 2.6M */
#define EXTRA_MEMSIZE			(0 * SZ_1K)
#define SENSOR_JPEG_SNAPSHOT_MEMSIZE \
	(((SENSOR_JPEG_OUTPUT_MAXSIZE + EXTRA_MEMSIZE  + SZ_16K-1) / SZ_16K) * SZ_16K)

#define POLL_TIME_MS		10
#define CAPTURE_POLL_TIME_MS    1000

/* maximum time for one frame in norma light */
#define ONE_FRAME_DELAY_MS_NORMAL		66
/* maximum time for one frame in low light */
#define ONE_FRAME_DELAY_MS_LOW			100

/* level at or below which we need to enable flash when in auto mode */
#ifdef CONFIG_VIDEO_S5K5CCGX_P2
#define LOW_LIGHT_LEVEL		0x3A
#else
#define FLASH_LOW_LIGHT_LEVEL		0x50
#ifdef CONFIG_VIDEO_S5K5CCGX_P8
#define CAPTURE_LOW_LIGHT_LEVEL		0x20
#endif
#endif /* CONFIG_VIDEO_S5K5CCGX_P2 */

#define FIRST_AF_SEARCH_COUNT   80
#define SECOND_AF_SEARCH_COUNT  80
#define AE_STABLE_SEARCH_COUNT	7 /* 4->7. but ae-unstable still occurs. */

/* Sensor AF first,second window size.
 * we use constant values intead of reading sensor register */
#define FIRST_WINSIZE_X			512
#define FIRST_WINSIZE_Y			568
#define SCND_WINSIZE_X			230
#define SCND_WINSIZE_Y			306

/* The Path of Setfile */
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

#if defined(CONFIG_VIDEO_S5K5CCGX_P4W)
#define TUNING_FILE_PATH "/mnt/sdcard/s5k5ccgx_regs-p4w.h"
#elif defined(CONFIG_VIDEO_S5K5CCGX_P8)
#define TUNING_FILE_PATH "/mnt/sdcard/s5k5ccgx_regs-p8.h"
#elif defined(CONFIG_VIDEO_S5K5CCGX_P2)
#define TUNING_FILE_PATH "/mnt/sdcard/s5k5ccgx_regs-p2.h"
#else
#define TUNING_FILE_PATH NULL
#endif
#endif /* CONFIG_LOAD_FILE*/

#if defined(CONFIG_VIDEO_S5K5CCGX_P4W)
#include "s5k5ccgx_regs-p4w.h"
#elif defined(CONFIG_VIDEO_S5K5CCGX_P8)
#include "s5k5ccgx_regs-p8.h"
#elif defined(CONFIG_VIDEO_S5K5CCGX_P2)
#include "s5k5ccgx_regs-p2.h"
#else
#include "s5k5ccgx_reg.h"
#endif /* CONFIG_VIDEO_S5K5CCGX_P4W*/

#endif /* __S5K5CCGX_P4W_H__ */
