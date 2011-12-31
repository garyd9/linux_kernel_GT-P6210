/*
 * Controls for M-5MOLS 8M Pixel camera sensor with ISP
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
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>

#include "m5mols.h"
#include "m5mols_reg.h"

static int m5mols_set_ae_lock(struct m5mols_info *info, bool lock)
{
	struct v4l2_subdev *sd = &info->sd;

	return i2c_w8_ae(sd, CAT3_AE_LOCK, lock ? 0x0 : 0x1);
}

static int m5mols_set_awb_lock(struct m5mols_info *info, bool lock)
{
	struct v4l2_subdev *sd = &info->sd;

	return i2c_w8_wb(sd, CAT6_AWB_LOCK, lock ? 0x0 : 0x1);
}

static int m5mols_wb_mode(struct m5mols_info *info, struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &info->sd;
	static u8 m5mols_wb_auto[] = { 0x1, 0x2, };
	int ret;

	ret = m5mols_set_awb_lock(info, false);
	if (!ret)
		ret = i2c_w8_wb(sd, CAT6_AWB_MODE, m5mols_wb_auto[ctrl->val]);

	return ret;
}

static int m5mols_exposure_mode(struct m5mols_info *info, struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &info->sd;
	static u8 m5mols_ae_mode[] = {
		[V4L2_EXPOSURE_MANUAL]	= 0x00,	/* AE off */
		[V4L2_EXPOSURE_AUTO]	= 0x01,	/* Exposure all blocks */
	};
	int ret;

	ret = m5mols_set_ae_lock(info, false);
	if (!ret)
		ret = i2c_w8_ae(sd, CAT3_AE_MODE, m5mols_ae_mode[ctrl->val]);

	return ret;
}

static int m5mols_exposure(struct m5mols_info *info)
{
	struct v4l2_subdev *sd = &info->sd;

	return i2c_w16_ae(sd, CAT3_MANUAL_GAIN_MON, info->exposure->val);
}

static int m5mols_set_saturation(struct m5mols_info *info, struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &info->sd;
	static u8 m5mols_chroma_lvl[] = {
		0x1c, 0x3e, 0x5f, 0x80, 0xa1, 0xc2, 0xe4,
	};
	int ret;

	ret = i2c_w8_mon(sd, CAT2_CHROMA_LVL, m5mols_chroma_lvl[ctrl->val]);
	if (!ret)
		ret = i2c_w8_mon(sd, CAT2_CHROMA_EN, true);

	return ret;
}

static int m5mols_set_colorfx(struct m5mols_info *info, struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = &info->sd;
	static u8 m5mols_effects_gamma[] = {	/* cat 1: Effects */
		[V4L2_COLORFX_NEGATIVE]		= 0x01,
		[V4L2_COLORFX_EMBOSS]		= 0x06,
		[V4L2_COLORFX_SKETCH]		= 0x07,
	};
	static u8 m5mols_cfixb_chroma[] = {	/* cat 2: Cr for effect */
		[V4L2_COLORFX_BW]		= 0x0,
		[V4L2_COLORFX_SEPIA]		= 0xd8,
		[V4L2_COLORFX_SKY_BLUE]		= 0x40,
		[V4L2_COLORFX_GRASS_GREEN]	= 0xe0,
	};
	static u8 m5mols_cfixr_chroma[] = {	/* cat 2: Cb for effect */
		[V4L2_COLORFX_BW]		= 0x0,
		[V4L2_COLORFX_SEPIA]		= 0x18,
		[V4L2_COLORFX_SKY_BLUE]		= 0x00,
		[V4L2_COLORFX_GRASS_GREEN]	= 0xe0,
	};
	int ret = -EINVAL;

	switch (ctrl->val) {
	case V4L2_COLORFX_NONE:
		return i2c_w8_mon(sd, CAT2_COLOR_EFFECT, false);
	case V4L2_COLORFX_BW:		/* chroma: Gray */
	case V4L2_COLORFX_SEPIA:	/* chroma: Sepia */
	case V4L2_COLORFX_SKY_BLUE:	/* chroma: Blue */
	case V4L2_COLORFX_GRASS_GREEN:	/* chroma: Green */
		ret = i2c_w8_mon(sd, CAT2_CFIXB,
				m5mols_cfixb_chroma[ctrl->val]);
		if (!ret)
			ret = i2c_w8_mon(sd, CAT2_CFIXR,
					m5mols_cfixr_chroma[ctrl->val]);
		if (!ret)
			ret = i2c_w8_mon(sd, CAT2_COLOR_EFFECT, true);
		return ret;
	case V4L2_COLORFX_NEGATIVE:	/* gamma: Negative */
	case V4L2_COLORFX_EMBOSS:	/* gamma: Emboss */
	case V4L2_COLORFX_SKETCH:	/* gamma: Outline */
		ret = i2c_w8_param(sd, CAT1_EFFECT,
				m5mols_effects_gamma[ctrl->val]);
		if (!ret)
			ret = i2c_w8_mon(sd, CAT2_COLOR_EFFECT, true);
		return ret;
	}

	return ret;
}

int m5mols_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct m5mols_info *info = to_m5mols(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE_AUTO:
/* Check this variable
 * ctrl->is_new in 2.6.38 */
		if (!ctrl->has_new)
			ctrl->val = V4L2_EXPOSURE_MANUAL;
		ret = m5mols_exposure_mode(info, ctrl);
		if (!ret && ctrl->val == V4L2_EXPOSURE_MANUAL)
			ret = m5mols_exposure(info);
		return ret;
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return m5mols_wb_mode(info, ctrl);
	case V4L2_CID_SATURATION:
		return m5mols_set_saturation(info, ctrl);
	case V4L2_CID_COLORFX:
		return m5mols_set_colorfx(info, ctrl);
	}

	return -EINVAL;
}
