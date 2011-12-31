/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_dec.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __S5P_MFC_DEC_H_
#define __S5P_MFC_DEC_H_ __FILE__

#define MAX_FRAME_SIZE		(2*1024*1024)

struct s5p_mfc_codec_ops *get_dec_codec_ops(void);
struct vb2_ops *get_dec_queue_ops(void);
const struct v4l2_ioctl_ops *get_dec_v4l2_ioctl_ops(void);
struct s5p_mfc_fmt *get_dec_def_fmt(bool src);

#endif /* __S5P_MFC_DEC_H_ */
