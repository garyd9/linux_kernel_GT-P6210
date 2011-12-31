/*
 * videobuf2-cma.c - CMA memory allocator for videobuf2
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Pawel Osciak <p.osciak@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cma.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/file.h>

#include <media/videobuf2-core.h>
#include <media/videobuf2-memops.h>

#include <asm/io.h>

struct vb2_cma_conf {
	struct device		*dev;
	const char		*type;
	unsigned long		alignment;
};

struct vb2_cma_buf {
	struct vb2_cma_conf		*conf;
	dma_addr_t			paddr;
	unsigned long			size;
	struct vm_area_struct		*vma;
	atomic_t			refcount;
	struct vb2_vmarea_handler	handler;
};

static void vb2_cma_put(void *buf_priv);

static void *vb2_cma_alloc(void *alloc_ctx, unsigned long size)
{
	struct vb2_cma_conf *conf = alloc_ctx;
	struct vb2_cma_buf *buf;

	buf = kzalloc(sizeof *buf, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->paddr = cma_alloc(conf->dev, conf->type, size, conf->alignment);
	if (IS_ERR((void *)buf->paddr)) {
		printk(KERN_ERR "cma_alloc of size %ld failed\n", size);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	buf->conf = conf;
	buf->size = size;

	buf->handler.refcount = &buf->refcount;
	buf->handler.put = vb2_cma_put;
	buf->handler.arg = buf;

	atomic_inc(&buf->refcount);

	return buf;
}

static void vb2_cma_put(void *buf_priv)
{
	struct vb2_cma_buf *buf = buf_priv;

	if (atomic_dec_and_test(&buf->refcount)) {
		cma_free(buf->paddr);
		kfree(buf);
	}
}

static void *vb2_cma_cookie(void *buf_priv)
{
	struct vb2_cma_buf *buf = buf_priv;

	return (void *)buf->paddr;
}

static unsigned int vb2_cma_num_users(void *buf_priv)
{
	struct vb2_cma_buf *buf = buf_priv;

	return atomic_read(&buf->refcount);
}

static int vb2_cma_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct vb2_cma_buf *buf = buf_priv;

	if (!buf) {
		printk(KERN_ERR "No buffer to map\n");
		return -EINVAL;
	}

	return vb2_mmap_pfn_range(vma, buf->paddr, buf->size,
				  &vb2_common_vm_ops, &buf->handler);
}

static void *vb2_cma_get_userptr(void *alloc_ctx, unsigned long vaddr,
				 unsigned long size, int write)
{
	struct vb2_cma_buf *buf;
	struct vm_area_struct *vma;
	dma_addr_t paddr = 0;
	int ret;

	buf = kzalloc(sizeof *buf, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	ret = vb2_get_contig_userptr(vaddr, size, &vma, &paddr);
	if (ret) {
		printk(KERN_ERR "Failed acquiring VMA for vaddr 0x%08lx\n",
				vaddr);
		kfree(buf);
		return ERR_PTR(ret);
	}

	buf->size = size;
	buf->paddr = paddr;
	buf->vma = vma;

	return buf;
}

static void vb2_cma_put_userptr(void *mem_priv)
{
	struct vb2_cma_buf *buf = mem_priv;

	if (!buf)
		return;

	vb2_put_vma(buf->vma);
	kfree(buf);
}

static void *vb2_cma_vaddr(void *mem_priv)
{
	struct vb2_cma_buf *buf = mem_priv;
	if (!buf)
		return 0;

	return phys_to_virt(buf->paddr);
}

const struct vb2_mem_ops vb2_cma_memops = {
	.alloc		= vb2_cma_alloc,
	.put		= vb2_cma_put,
	.cookie		= vb2_cma_cookie,
	.mmap		= vb2_cma_mmap,
	.get_userptr	= vb2_cma_get_userptr,
	.put_userptr	= vb2_cma_put_userptr,
	.num_users	= vb2_cma_num_users,
	.vaddr		= vb2_cma_vaddr,
};
EXPORT_SYMBOL_GPL(vb2_cma_memops);

void *vb2_cma_init(struct device *dev, const char *type,
					unsigned long alignment)
{
	struct vb2_cma_conf *conf;

	conf = kzalloc(sizeof *conf, GFP_KERNEL);
	if (!conf)
		return ERR_PTR(-ENOMEM);

	conf->dev = dev;
	conf->type = type;
	conf->alignment = alignment;

	return conf;
}
EXPORT_SYMBOL_GPL(vb2_cma_init);

void vb2_cma_cleanup(void *conf)
{
	kfree(conf);
}
EXPORT_SYMBOL_GPL(vb2_cma_cleanup);

void **vb2_cma_init_multi(struct device *dev,
					  unsigned int num_planes,
					  const char *types[],
					  unsigned long alignments[])
{
	struct vb2_cma_conf *cma_conf;
	void **alloc_ctxes;
	unsigned int i;

	alloc_ctxes = kzalloc((sizeof *alloc_ctxes + sizeof *cma_conf)
				* num_planes, GFP_KERNEL);
	if (!alloc_ctxes)
		return ERR_PTR(-ENOMEM);

	cma_conf = (void *)(alloc_ctxes + num_planes);

	for (i = 0; i < num_planes; ++i, ++cma_conf) {
		alloc_ctxes[i] = cma_conf;
		cma_conf->dev = dev;
		cma_conf->type = types[i];
		cma_conf->alignment = alignments[i];
	}

	return alloc_ctxes;
}
EXPORT_SYMBOL_GPL(vb2_cma_init_multi);

void vb2_cma_cleanup_multi(void **alloc_ctxes)
{
	kfree(alloc_ctxes);
}
EXPORT_SYMBOL_GPL(vb2_cma_cleanup_multi);

MODULE_DESCRIPTION("CMA allocator handling routines for videobuf2");
MODULE_AUTHOR("Pawel Osciak");
MODULE_LICENSE("GPL");
