/*
 * Driver internal Header for M-5MOLS 8M Pixel camera sensor with ISP
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

#ifndef M5MOLS_H
#define M5MOLS_H

#include <media/v4l2-subdev.h>
#include "m5mols_reg.h"

#define v4l2msg(fmt, arg...)	do {				\
	v4l2_dbg(1, m5mols_debug, &info->sd, fmt, ## arg);	\
} while (0)

extern int m5mols_debug;

enum m5mols_mode {
	MODE_SYSINIT,
	MODE_PARMSET,
	MODE_MONITOR,
	MODE_UNKNOWN,
};

enum m5mols_i2c_size {
	I2C_8BIT	= 1,
	I2C_16BIT	= 2,
	I2C_32BIT	= 4,
	I2C_MAX		= 4,
};

enum m5mols_fps {
	M5MOLS_FPS_AUTO	= 0,
	M5MOLS_FPS_10	= 10,
	M5MOLS_FPS_12	= 12,
	M5MOLS_FPS_15	= 15,
	M5MOLS_FPS_20	= 20,
	M5MOLS_FPS_21	= 21,
	M5MOLS_FPS_22	= 22,
	M5MOLS_FPS_23	= 23,
	M5MOLS_FPS_24	= 24,
	M5MOLS_FPS_30	= 30,
	M5MOLS_FPS_MAX	= M5MOLS_FPS_30,
};

enum m5mols_res_type {
	M5MOLS_RES_MON,
	/* It's not supported below yet. */
	M5MOLS_RES_PREVIEW,
	M5MOLS_RES_THUMB,
	M5MOLS_RES_CAPTURE,
	M5MOLS_RES_UNKNOWN,
};

struct m5mols_resolution {
	u8			value;
	enum m5mols_res_type	type;
	u16			width;
	u16			height;
};

struct m5mols_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

struct m5mols_version {
	u8	ctm_code;	/* customer code */
	u8	pj_code;	/* project code */
	u16	fw;		/* firmware version */
	u16	hw;		/* hardware version */
	u16	parm;		/* parameter version */
	u16	awb;		/* AWB version */
};

struct m5mols_info {
	const struct m5mols_platform_data	*pdata;
	struct v4l2_subdev		sd;
	struct v4l2_mbus_framefmt	fmt;
	struct v4l2_fract		tpf;

	struct v4l2_ctrl_handler	handle;
	struct {
		/* support only AE of the Monitor Mode in this version */
		struct v4l2_ctrl	*autoexposure;
		struct v4l2_ctrl	*exposure;
	};
	struct v4l2_ctrl		*autowb;
	struct v4l2_ctrl		*colorfx;
	struct v4l2_ctrl		*saturation;

	enum m5mols_mode		mode;
	enum m5mols_mode		mode_backup;

	struct m5mols_version		ver;
	bool				power;
};

/* control functions */
int m5mols_set_ctrl(struct v4l2_ctrl *ctrl);

/* I2C functions - referenced by below I2C helper functions */
int m5mols_read_reg(struct v4l2_subdev *sd, enum m5mols_i2c_size size,
		u8 category, u8 cmd, u32 *val);
int m5mols_write_reg(struct v4l2_subdev *sd, enum m5mols_i2c_size size,
		u8 category, u8 cmd, u32 val);
int m5mols_check_busy(struct v4l2_subdev *sd,
		u8 category, u8 cmd, u32 value);
int m5mols_set_mode(struct v4l2_subdev *sd, enum m5mols_mode mode);

/*
 * helper functions
 */
static inline struct m5mols_info *to_m5mols(struct v4l2_subdev *sd)
{
	return container_of(sd, struct m5mols_info, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct m5mols_info, handle)->sd;
}

static inline bool is_streaming(struct v4l2_subdev *sd)
{
	struct m5mols_info *info = to_m5mols(sd);
	return info->mode == MODE_MONITOR;
}

static inline bool is_powerup(struct v4l2_subdev *sd)
{
	struct m5mols_info *info = to_m5mols(sd);
	return info->power;
}

static inline int __must_check i2c_w8_system(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_8BIT, CAT_SYSTEM, cmd, val);
}

static inline int __must_check i2c_w8_param(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_8BIT, CAT_PARAM, cmd, val);
}

static inline int __must_check i2c_w8_mon(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_8BIT, CAT_MONITOR, cmd, val);
}

static inline int __must_check i2c_w8_ae(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_8BIT, CAT_AE, cmd, val);
}

static inline int __must_check i2c_w16_ae(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_16BIT, CAT_AE, cmd, val);
}

static inline int __must_check i2c_w8_wb(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_8BIT, CAT_WB, cmd, val);
}

static inline int __must_check i2c_w8_flash(struct v4l2_subdev *sd,
		u8 cmd, u32 val)
{
	return m5mols_write_reg(sd, I2C_8BIT, CAT_FLASH, cmd, val);
}

static inline int __must_check i2c_r8_system(struct v4l2_subdev *sd,
		u8 cmd, u32 *val)
{
	return m5mols_read_reg(sd, I2C_8BIT, CAT_SYSTEM, cmd, val);
}

static inline int __must_check i2c_r16_ae(struct v4l2_subdev *sd,
		u8 cmd, u32 *val)
{
	return m5mols_read_reg(sd, I2C_16BIT, CAT_AE, cmd, val);
}

#endif	/* M5MOLS_H */
