/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_shm.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/io.h>

#include "s5p_mfc_mem.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_common.h"

int s5p_mfc_init_shm(struct s5p_mfc_ctx *ctx)
{
	struct s5p_mfc_dev *dev = ctx->dev;
	void *shm_alloc_ctx = dev->alloc_ctx[MFC_CMA_BANK1_ALLOC_CTX];

	ctx->shm_alloc = s5p_mfc_mem_alloc(shm_alloc_ctx, SHARED_BUF_SIZE);
	if (IS_ERR(ctx->shm_alloc)) {
		mfc_err("failed to allocate shared memory\n");
		return PTR_ERR(ctx->shm_alloc);
	}

	/* shm_ofs only keeps the offset from base (port a) */
	ctx->shm_ofs = s5p_mfc_mem_cookie(shm_alloc_ctx, ctx->shm_alloc) - dev->port_a;
	ctx->shm = s5p_mfc_mem_vaddr(shm_alloc_ctx, ctx->shm_alloc);
	if (!ctx->shm) {
		s5p_mfc_mem_put(shm_alloc_ctx, ctx->shm_alloc);
		ctx->shm_ofs = 0;
		ctx->shm_alloc = NULL;

		mfc_err("failed to virt addr of shared memory\n");
		return -ENOMEM;
	}

	memset((void *)ctx->shm, 0, SHARED_BUF_SIZE);
	s5p_mfc_cache_clean(ctx->shm, SHARED_BUF_SIZE);

	mfc_debug(2, "shm info addr: 0x%08x, phys: 0x%08x\n",
		 (unsigned int)ctx->shm, ctx->shm_ofs);

	return 0;
}

