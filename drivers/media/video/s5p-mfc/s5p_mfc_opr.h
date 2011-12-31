/*
 * drivers/media/video/samsung/mfc5/s5p_mfc_opr.h
 *
 * Header file for Samsung MFC (Multi Function Codec - FIMV) driver
 * Contains declarations of hw related functions.
 *
 * Kamil Debski, Copyright (c) 2010 Samsung Electronics
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef S5P_MFC_OPR_H_
#define S5P_MFC_OPR_H_

#include "s5p_mfc_common.h"
#include "s5p_mfc_mem.h"

/*
int s5p_mfc_release_firmware(struct s5p_mfc_dev *dev);
int s5p_mfc_alloc_firmware(struct s5p_mfc_dev *dev);
int s5p_mfc_load_firmware(struct s5p_mfc_dev *dev);
int s5p_mfc_init_hw(struct s5p_mfc_dev *dev);
*/

int s5p_mfc_init_decode(struct s5p_mfc_ctx *ctx);
int s5p_mfc_init_encode(struct s5p_mfc_ctx *mfc_ctx);
/*
void s5p_mfc_deinit_hw(struct s5p_mfc_dev *dev);
int s5p_mfc_set_sleep(struct s5p_mfc_ctx *ctx);
int s5p_mfc_set_wakeup(struct s5p_mfc_ctx *ctx);
*/

int s5p_mfc_set_dec_frame_buffer(struct s5p_mfc_ctx *ctx);
int s5p_mfc_set_dec_stream_buffer(struct s5p_mfc_ctx *ctx, int buf_addr,
						  unsigned int start_num_byte,
						  unsigned int buf_size);

void s5p_mfc_set_enc_frame_buffer(struct s5p_mfc_ctx *ctx,
		unsigned long y_addr, unsigned long c_addr);
int s5p_mfc_set_enc_stream_buffer(struct s5p_mfc_ctx *ctx,
		unsigned long addr, unsigned int size);
void s5p_mfc_get_enc_frame_buffer(struct s5p_mfc_ctx *ctx,
		unsigned long *y_addr, unsigned long *c_addr);
int s5p_mfc_set_enc_ref_buffer(struct s5p_mfc_ctx *mfc_ctx);

int s5p_mfc_decode_one_frame(struct s5p_mfc_ctx *ctx, int last_frame);
int s5p_mfc_encode_one_frame(struct s5p_mfc_ctx *mfc_ctx);

/* Instance handling */
/*
int s5p_mfc_open_inst(struct s5p_mfc_ctx *ctx);
int s5p_mfc_return_inst_no(struct s5p_mfc_ctx *ctx);
*/

/* Memory allocation */
int s5p_mfc_alloc_dec_temp_buffers(struct s5p_mfc_ctx *ctx);
void s5p_mfc_set_dec_desc_buffer(struct s5p_mfc_ctx *ctx);
void s5p_mfc_release_dec_desc_buffer(struct s5p_mfc_ctx *ctx);

int s5p_mfc_alloc_codec_buffers(struct s5p_mfc_ctx *ctx);
void s5p_mfc_release_codec_buffers(struct s5p_mfc_ctx *ctx);

int s5p_mfc_alloc_instance_buffer(struct s5p_mfc_ctx *ctx);
void s5p_mfc_release_instance_buffer(struct s5p_mfc_ctx *ctx);

#define s5p_mfc_get_dspl_y_adr()	(readl(dev->regs_base + \
					S5P_FIMV_SI_DISPLAY_Y_ADR) << 11)
#define s5p_mfc_get_dspl_status()	readl(dev->regs_base + \
						S5P_FIMV_SI_DISPLAY_STATUS)
#define s5p_mfc_get_frame_type()	(readl(dev->regs_base + \
						S5P_FIMV_DECODE_FRAME_TYPE) \
					& S5P_FIMV_DECODE_FRAME_MASK)
#define s5p_mfc_get_consumed_stream()	readl(dev->regs_base + \
						S5P_FIMV_SI_CONSUMED_BYTES)
#define s5p_mfc_get_int_reason()	(readl(dev->regs_base + \
					S5P_FIMV_RISC2HOST_CMD) & 0x1FFFF)
#define s5p_mfc_get_int_err()		readl(dev->regs_base + \
						S5P_FIMV_RISC2HOST_ARG2)
#define s5p_mfc_get_img_width()		readl(dev->regs_base + \
						S5P_FIMV_SI_HRESOL)
#define s5p_mfc_get_img_height()	readl(dev->regs_base + \
						S5P_FIMV_SI_VRESOL)
#define s5p_mfc_get_dpb_count()		readl(dev->regs_base + \
						S5P_FIMV_SI_BUF_NUMBER)
#define s5p_mfc_get_inst_no()		readl(dev->regs_base + \
						S5P_FIMV_RISC2HOST_ARG1)
#define s5p_mfc_get_enc_strm_size()	readl(dev->regs_base + \
						S5P_FIMV_ENC_SI_STRM_SIZE)
#define s5p_mfc_get_enc_slice_type()	readl(dev->regs_base + \
						S5P_FIMV_ENC_SI_SLICE_TYPE)

/* Interrupt handling routines */
/*
#define s5p_mfc_clear_int_flags()					\
	do {								\
		writel(0, dev->regs_base + S5P_FIMV_RISC_HOST_INT);	\
		writel(0, dev->regs_base + S5P_FIMV_RISC2HOST_CMD);	\
		writel(0xffff, dev->regs_base + S5P_FIMV_SI_RTN_CHID);	\
	} while (0)
*/


void s5p_mfc_try_run(struct s5p_mfc_dev *dev);

void s5p_mfc_cleanup_queue(struct list_head *lh, struct vb2_queue *vq);

#endif /* S5P_MFC_OPR_H_ */
