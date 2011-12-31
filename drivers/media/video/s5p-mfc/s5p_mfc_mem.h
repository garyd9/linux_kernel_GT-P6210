/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_mem.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __S5P_MFC_MEM_H_
#define __S5P_MFC_MEM_H_ __FILE__

#include <linux/platform_device.h>
#if defined(CONFIG_S5P_MFC_VB2_CMA)
#include <media/videobuf2-cma.h>
#elif defined(CONFIG_S5P_MFC_VB2_DMA_POOL)
#include <media/videobuf2-dma-pool.h>
#elif defined(CONFIG_S5P_MFC_VB2_SDVMM)
#include <media/videobuf2-sdvmm.h>
#endif

/* Offset base used to differentiate between CAPTURE and OUTPUT
*  while mmaping */
#define DST_QUEUE_OFF_BASE      (TASK_SIZE / 2)

#define FIRMWARE_CODE_SIZE	0x60000		/* 384KB */
#define MFC_H264_CTX_BUF_SIZE	0x96000		/* 600KB per H264 instance */
#define MFC_CTX_BUF_SIZE	0x2800		/* 10KB per instance */
#define DESC_BUF_SIZE		0x20000		/* 128KB for DESC buffer */
#define SHARED_BUF_SIZE		0x01000		/* 4KB for shared buffer */
#define CPB_BUF_SIZE		0x400000	/* 4MB fr decoder */

#if defined(CONFIG_S5P_MFC_VB2_CMA)
/* Define names for CMA memory kinds used by MFC */
#define MFC_CMA_ALLOC_CTX_NUM	3

#define MFC_CMA_BANK1		"a"
#define MFC_CMA_BANK2		"b"
#define MFC_CMA_FW		"f"

#define MFC_CMA_BANK1_ALLOC_CTX 1
#define MFC_CMA_BANK2_ALLOC_CTX 0
#define MFC_CMA_FW_ALLOC_CTX	2

#define MFC_CMA_BANK1_ALIGN	0x2000	/* 8KB */
#define MFC_CMA_BANK2_ALIGN	0x2000	/* 8KB */
#define MFC_CMA_FW_ALIGN	0x20000	/* 128KB */

#define mfc_plane_cookie(v, n)	vb2_cma_plane_paddr(v, n)

static inline void *s5p_mfc_mem_alloc(void *a, unsigned int s)
{
	return vb2_cma_memops.alloc(a, s);
}

static inline size_t s5p_mfc_mem_cookie(void *a, void *b)
{
	return (size_t)vb2_cma_memops.cookie(b);
}

static inline void s5p_mfc_mem_put(void *a, void *b)
{
	vb2_cma_memops.put(b);
}

static inline void *s5p_mfc_mem_vaddr(void *a, void *b)
{
	return vb2_cma_memops.vaddr(b);
}
#elif defined(CONFIG_S5P_MFC_VB2_DMA_POOL)
#define MFC_ALLOC_CTX_NUM	2

#define MFC_BANK_A_ALLOC_CTX	0
#define MFC_BANK_B_ALLOC_CTX	1

#define MFC_CMA_BANK1_ALLOC_CTX MFC_BANK_A_ALLOC_CTX
#define MFC_CMA_BANK2_ALLOC_CTX MFC_BANK_B_ALLOC_CTX
#define MFC_CMA_FW_ALLOC_CTX	MFC_BANK_A_ALLOC_CTX

#define mfc_plane_cookie(v, n)	vb2_dma_pool_plane_paddr(v, n)

static inline void *s5p_mfc_mem_alloc(void *a, unsigned int s)
{
	return vb2_dma_pool_memops.alloc(a, s);
}

static inline size_t s5p_mfc_mem_cookie(void *a, void *b)
{
	return (size_t)vb2_dma_pool_memops.cookie(b);
}

static inline void s5p_mfc_mem_put(void *a, void *b)
{
	vb2_dma_pool_memops.put(b);
}

static inline void *s5p_mfc_mem_vaddr(void *a, void *b)
{
	return vb2_dma_pool_memops.vaddr(b);
}
#elif defined(CONFIG_S5P_MFC_VB2_SDVMM)
#define MFC_ALLOC_CTX_NUM	2

#define MFC_BANK_A_ALLOC_CTX	0
#define MFC_BANK_B_ALLOC_CTX	1

#define MFC_BANK_A_ALIGN_ORDER	11
#define MFC_BANK_B_ALIGN_ORDER	11

#define MFC_CMA_BANK1_ALLOC_CTX MFC_BANK_A_ALLOC_CTX
#define MFC_CMA_BANK2_ALLOC_CTX MFC_BANK_B_ALLOC_CTX
#define MFC_CMA_FW_ALLOC_CTX	MFC_BANK_A_ALLOC_CTX


#define mfc_plane_cookie(v, n)	vb2_sdvmm_plane_dvaddr(v, n)

static inline void *s5p_mfc_mem_alloc(void *a, unsigned int s)
{
	return vb2_sdvmm_memops.alloc(a, s);
}

static inline size_t s5p_mfc_mem_cookie(void *a, void *b)
{
	return (size_t)vb2_sdvmm_memops.cookie(b);
}

static inline void s5p_mfc_mem_put(void *a, void *b)
{
	vb2_sdvmm_memops.put(b);
}

static inline void *s5p_mfc_mem_vaddr(void *a, void *b)
{
	return vb2_sdvmm_memops.vaddr(b);
}
#endif

struct vb2_mem_ops *s5p_mfc_mem_ops(void);
void **s5p_mfc_mem_init_multi(struct device *dev);
void s5p_mfc_mem_cleanup_multi(void **alloc_ctxes);

void s5p_mfc_cache_clean(const void *start_addr, unsigned long size);
void s5p_mfc_cache_inv(const void *start_addr, unsigned long size);

void s5p_mfc_mem_suspend(void *alloc_ctx);
void s5p_mfc_mem_resume(void *alloc_ctx);

#endif /* __S5P_MFC_MEM_H_ */
