/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_cmd.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __S5P_MFC_CMD_H
#define __S5P_MFC_CMD_H __FILE__

#if 0
#include <linux/interrupt.h>

#include "mfc_dev.h"

#define MAX_H2R_ARG		4
#define H2R_CMD_TIMEOUT		100	/* ms */
#define H2R_INT_TIMEOUT		5000	/* ms */
#define CODEC_INT_TIMEOUT	1000	/* ms */

enum mfc_h2r_cmd {
	H2R_NOP		= 0,
	OPEN_CH		= 1,
	CLOSE_CH	= 2,
	SYS_INIT	= 3,
	FLUSH		= 4,
	SLEEP		= 5,
	WAKEUP		= 6,
	CONTINUE_ENC	= 7,
	ABORT_ENC	= 8,
};

enum mfc_codec_cmd {
	SEQ_HEADER		= 1,
	FRAME_START		= 2,
	LAST_SEQ		= 3,
	INIT_BUFFERS		= 4,
	FRAME_START_REALLOC	= 5,
	FRAME_BATCH_START	= 6,
};

enum mfc_r2h_ret {
	R2H_NOP			= 0x00,
	OPEN_CH_RET		= 0x01,
	CLOSE_CH_RET		= 0x02,

	SEQ_DONE_RET		= 0x04,
	FRAME_DONE_RET		= 0x05,
	SLICE_DONE_RET		= 0x06,
	ENC_COMPLETE_RET	= 0x07,
	SYS_INIT_RET		= 0x08,
	FW_STATUS_RET		= 0x09,
	SLEEP_RET		= 0x0A,
	WAKEUP_RET		= 0x0B,
	FLUSH_CMD_RET		= 0x0C,
	ABORT_RET		= 0x0D,
	BATCH_ENC_RET		= 0x0E,
	INIT_BUFFERS_RET	= 0x0F,
	EDFU_INIT_RET		= 0x10,

	ERR_RET			= 0x20,
};

struct mfc_cmd_args {
	unsigned int	arg[MAX_H2R_ARG];
};

irqreturn_t mfc_irq(int irq, void *dev_id);

int mfc_cmd_fw_start(struct mfc_dev *dev);

int mfc_cmd_sys_init(struct mfc_dev *dev);
int mfc_cmd_sys_sleep(struct mfc_dev *dev);
int mfc_cmd_sys_wakeup(struct mfc_dev *dev);

int mfc_cmd_inst_open(struct mfc_inst_ctx *ctx);
int mfc_cmd_inst_close(struct mfc_inst_ctx *ctx);
int mfc_cmd_seq_start(struct mfc_inst_ctx *ctx);
int mfc_cmd_init_buffers(struct mfc_inst_ctx *ctx);
int mfc_cmd_frame_start(struct mfc_inst_ctx *ctx);
#endif

#define MAX_H2R_ARG		4

struct s5p_mfc_cmd_args {
	unsigned int	arg[MAX_H2R_ARG];
};

int s5p_mfc_sys_init_cmd(struct s5p_mfc_dev *dev);
int s5p_mfc_sleep_cmd(struct s5p_mfc_dev *dev);
int s5p_mfc_wakeup_cmd(struct s5p_mfc_dev *dev);
int s5p_mfc_open_inst_cmd(struct s5p_mfc_ctx *ctx);
int s5p_mfc_close_inst_cmd(struct s5p_mfc_ctx *ctx);

#endif /* __S5P_MFC_CMD_H */
