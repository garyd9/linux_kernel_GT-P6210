/* include/media/videobuf2-sdvmm.h
 *
 * Copyright 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Definition of SDVMM memory allocator for videobuf2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _MEDIA_VIDEOBUF2_SDVMM_H
#define _MEDIA_VIDEOBUF2_SDVMM_H

#include <linux/vcm.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-memops.h>
#include <plat/s5p-vcm.h>

struct vb2_vcm {
	enum vcm_dev_id			vcm_id;
	resource_size_t			size;
};

struct vb2_cma {
	struct device	*dev;
	const char	*type;
	unsigned long	alignment;
};

struct vb2_drv {
	bool cacheable;
	bool remap_dva;
};

static inline unsigned long vb2_sdvmm_plane_dvaddr(struct vb2_buffer *vb, unsigned int plane_no)
{
	return (unsigned long)vb2_plane_cookie(vb, plane_no);
}

static inline unsigned long vb2_sdvmm_plane_kvaddr(struct vb2_buffer *vb, unsigned int plane_no)
{
	return (unsigned long)vb2_plane_vaddr(vb, plane_no);
}

void *vb2_sdvmm_init(struct vb2_vcm *vcm, struct vb2_cma *cma, struct vb2_drv *drv);
void vb2_sdvmm_cleanup(void *alloc_ctx);

void **vb2_sdvmm_init_multi(unsigned int num_planes, struct vb2_vcm *vcm, struct vb2_cma *cma[], struct vb2_drv *drv);
void vb2_sdvmm_cleanup_multi(void **alloc_ctxes);

void vb2_sdvmm_set_cacheable(void *alloc_ctx, bool cacheable);
bool vb2_sdvmm_get_cacheable(void *alloc_ctx);
int vb2_sdvmm_cache_flush(void *alloc_ctx, struct vb2_buffer *vb, u32 plane_no);

void vb2_sdvmm_suspend(void *alloc_ctx);
void vb2_sdvmm_resume(void *alloc_ctx);

extern const struct vb2_mem_ops vb2_sdvmm_memops;

#endif
