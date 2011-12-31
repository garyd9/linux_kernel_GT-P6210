/*
 * videobuf2-cma.h - CMA memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <p.osciak@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#ifndef _MEDIA_VIDEOBUF2_CMA_H
#define _MEDIA_VIDEOBUF2_CMA_H

#include <media/videobuf2-core.h>

static inline unsigned long vb2_cma_plane_paddr(struct vb2_buffer *vb,
						unsigned int plane_no)
{
	return (unsigned long)vb2_plane_cookie(vb, plane_no);
}

struct vb2_alloc_ctx *vb2_cma_init(struct device *dev, const char *type,
					unsigned long alignment);
void vb2_cma_cleanup(struct vb2_alloc_ctx *alloc_ctx);

struct vb2_alloc_ctx **vb2_cma_init_multi(struct device *dev,
				  unsigned int num_planes, const char *types[],
				  unsigned long alignments[]);
void vb2_cma_cleanup_multi(struct vb2_alloc_ctx **alloc_ctxes);

struct vb2_alloc_ctx *vb2_cma_init(struct device *dev, const char *type,
					unsigned long alignment);
void vb2_cma_cleanup(struct vb2_alloc_ctx *alloc_ctx);

extern const struct vb2_mem_ops vb2_cma_memops;

#endif
