/*
 * videobuf2-dma-pool.h - DMA pool memory allocator for videobuf2
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _MEDIA_VIDEOBUF2_DMA_POOL_H
#define _MEDIA_VIDEOBUF2_DMA_POOL_H

#include <media/videobuf2-core.h>

static inline unsigned long vb2_dma_pool_plane_paddr(struct vb2_buffer *vb,
						     unsigned int plane_no)
{
	return (unsigned long)vb2_plane_cookie(vb, plane_no);
}

void *vb2_dma_pool_init(struct device *dev, unsigned long base_order,
			unsigned long alloc_order, unsigned long size);
void vb2_dma_pool_cleanup(struct vb2_alloc_ctx *alloc_ctx);

void **vb2_dma_pool_init_multi(struct device *dev, unsigned int num_planes,
			       unsigned long base_orders[],
			       unsigned long alloc_orders[],
			       unsigned long sizes[]);
void vb2_dma_pool_cleanup_multi(void **alloc_ctxes,
				unsigned int num_planes);

extern const struct vb2_mem_ops vb2_dma_pool_memops;

#endif /* _MEDIA_VIDEOBUF2_DMA_POOL_H */
