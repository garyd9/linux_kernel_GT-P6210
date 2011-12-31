/*
 * videobuf2-dma-pool.c - DMA pool memory allocator for videobuf2
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>

#include <media/videobuf2-core.h>
#include <media/videobuf2-memops.h>

#define DMA_POOL_MAGIC	0x706F6F6C
#define MAGIC_CHECK(is, should)							\
	if (unlikely((is) != (should))) {					\
		pr_err("magic mismatch: %x expected %x\n", (is), (should));	\
		BUG();								\
	}

static int debug;
module_param(debug, int, 0644);

#define dprintk(level, fmt, arg...)						\
	do {									\
		if (debug >= level)						\
			printk(KERN_DEBUG "vb2_dma_pool: " fmt, ## arg);	\
	} while (0)

struct vb2_dma_pool_conf {
	struct device		*dev;
	struct gen_pool		*pool;
	dma_addr_t		paddr;
	void			*vaddr;
	unsigned long		alignorder;
	unsigned long		size;
	u32			magic;
};

struct vb2_dma_pool_buf {
	struct vb2_dma_pool_conf	*conf;
	void				*vaddr;
	dma_addr_t			paddr;
	unsigned long			size;
	struct vm_area_struct		*vma;
	atomic_t			refcount;
	struct vb2_vmarea_handler	handler;
};

static void vb2_dma_pool_put(void *buf_priv);

static void *vb2_dma_pool_alloc(void *alloc_ctx, unsigned long size)
{
	struct vb2_dma_pool_conf *conf = alloc_ctx;
	struct vb2_dma_pool_buf *buf;

	BUG_ON(!conf);
	MAGIC_CHECK(conf->magic, DMA_POOL_MAGIC);

	buf = kzalloc(sizeof *buf, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->paddr = gen_pool_alloc_aligned(conf->pool, size,
					    conf->alignorder);
	if (!buf->paddr) {
		dev_err(conf->dev, "failed to get buffer from pool: %ld\n",
			size);
		kfree(buf);
		return ERR_PTR(-ENOMEM);
	}

	buf->conf = conf;
	buf->vaddr = conf->vaddr + (buf->paddr - conf->paddr);
	buf->size = size;

	buf->handler.refcount = &buf->refcount;
	buf->handler.put = vb2_dma_pool_put;
	buf->handler.arg = buf;

	atomic_inc(&buf->refcount);

	dprintk(2, "alloc-> vaddr: %p, paddr: 0x%08x, size: %ld\n",
		buf->vaddr, buf->paddr, size);

	return buf;
}

static void vb2_dma_pool_put(void *buf_priv)
{
	struct vb2_dma_pool_buf *buf = buf_priv;

	dprintk(3, "put-> refcount: %d\n", atomic_read(&buf->refcount));

	if (atomic_dec_and_test(&buf->refcount)) {
		dprintk(2, "put-> paddr: 0x%08x, size: %ld\n",
			buf->paddr, buf->size);

		gen_pool_free(buf->conf->pool, buf->paddr, buf->size);
		kfree(buf);
	}
}

static void *vb2_dma_pool_cookie(void *buf_priv)
{
	struct vb2_dma_pool_buf *buf = buf_priv;

	if (!buf) {
		pr_err("failed to get buffer\n");
		return NULL;
	}

	return (void *)buf->paddr;
}

static void *vb2_dma_pool_vaddr(void *buf_priv)
{
	struct vb2_dma_pool_buf *buf = buf_priv;

	if (!buf) {
		pr_err("failed to get buffer\n");
		return NULL;
	}

	return buf->vaddr;
}

static unsigned int vb2_dma_pool_num_users(void *buf_priv)
{
	struct vb2_dma_pool_buf *buf = buf_priv;

	if (!buf) {
		pr_err("failed to get buffer\n");
		return 0;
	}

	return atomic_read(&buf->refcount);
}

static int vb2_dma_pool_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct vb2_dma_pool_buf *buf = buf_priv;

	if (!buf) {
		pr_err("no buffer to map\n");
		return -EINVAL;
	}

	return vb2_mmap_pfn_range(vma, buf->paddr, buf->size,
				  &vb2_common_vm_ops, &buf->handler);
}

const struct vb2_mem_ops vb2_dma_pool_memops = {
	.alloc		= vb2_dma_pool_alloc,
	.put		= vb2_dma_pool_put,
	.cookie		= vb2_dma_pool_cookie,
	.vaddr		= vb2_dma_pool_vaddr,
	.mmap		= vb2_dma_pool_mmap,
	.num_users	= vb2_dma_pool_num_users,
};
EXPORT_SYMBOL_GPL(vb2_dma_pool_memops);

void *vb2_dma_pool_init(struct device *dev, unsigned long base_order,
			unsigned long alloc_order, unsigned long size)
{
	struct vb2_dma_pool_conf *conf;
	int ret;
	unsigned long margin, margin_order;

	if (!(size >> alloc_order))
		return ERR_PTR(-EINVAL);

	conf = kzalloc(sizeof *conf, GFP_KERNEL);
	if (!conf)
		return ERR_PTR(-ENOMEM);

	conf->magic = DMA_POOL_MAGIC;
	conf->dev = dev;
	conf->alignorder = alloc_order;
	conf->size = size;

	conf->vaddr = dma_alloc_coherent(conf->dev, conf->size,
					 &conf->paddr, GFP_KERNEL);
	if (!conf->vaddr) {
		dev_err(dev, "dma_alloc_coherent of size %ld failed\n",
			size);
		ret = -ENOMEM;
		goto fail_dma_alloc;
	}

	dprintk(1, "init-> vaddr: %p, paddr: 0x%08x, size: %ld\n",
		conf->vaddr, conf->paddr, conf->size);

	margin_order = (base_order > alloc_order) ? base_order : alloc_order;
	margin = ALIGN(conf->paddr, (1 << margin_order)) - conf->paddr;

	dprintk(1, "init-> margin_order: %ld, margin: %ld\n",
		margin_order, margin);

	if (margin >= conf->size) {
		ret = -ENOMEM;
		goto fail_base_align;
	}

	if (!((conf->size - margin) >> conf->alignorder)) {
		ret = -ENOMEM;
		goto fail_base_align;
	}

	conf->pool = gen_pool_create(conf->alignorder, -1);
	if (!conf->pool) {
		dev_err(conf->dev, "failed to create pool\n");
		ret = -ENOMEM;
		goto fail_pool_create;
	}

	ret = gen_pool_add(conf->pool, (conf->paddr + margin),
			   (conf->size - margin), -1);
	if (ret) {
		dev_err(conf->dev, "could not add buffer to pool");
		goto fail_pool_add;
	}

	return conf;

fail_pool_add:
	gen_pool_destroy(conf->pool);
fail_base_align:
fail_pool_create:
	dma_free_coherent(conf->dev, conf->size, conf->vaddr, conf->paddr);
fail_dma_alloc:
	kfree(conf);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(vb2_dma_pool_init);

void vb2_dma_pool_cleanup(void *conf)
{
	struct vb2_dma_pool_conf *_conf;

	_conf = (struct vb2_dma_pool_conf *)conf;

	BUG_ON(!_conf);
	MAGIC_CHECK(_conf->magic, DMA_POOL_MAGIC);

	gen_pool_destroy(_conf->pool);
	dma_free_coherent(_conf->dev, _conf->size, _conf->vaddr,
			  _conf->paddr);

	kfree(conf);
}
EXPORT_SYMBOL_GPL(vb2_dma_pool_cleanup);

void **vb2_dma_pool_init_multi(struct device *dev, unsigned int num_planes,
			       unsigned long base_orders[],
			       unsigned long alloc_orders[],
			       unsigned long sizes[])
{
	struct vb2_dma_pool_conf *conf;
	void **alloc_ctxes;
	int i, j;

	alloc_ctxes = kzalloc(sizeof *alloc_ctxes * num_planes, GFP_KERNEL);
	if (!alloc_ctxes)
		return ERR_PTR(-ENOMEM);

	conf = (void *)(alloc_ctxes + num_planes);

	for (i = 0; i < num_planes; ++i, ++conf) {
		dprintk(1, "init_multi-> index: %d, orders: %ld, %ld\n",
			i, base_orders[i], alloc_orders[i]);

		conf = vb2_dma_pool_init(dev, base_orders[i],
					 alloc_orders[i], sizes[i]);
		if (IS_ERR(conf)) {
			for (j = i - 1; j >= 0; j--)
				vb2_dma_pool_cleanup(alloc_ctxes[j]);

			kfree(alloc_ctxes);
			return ERR_PTR(PTR_ERR(conf));
		}

		alloc_ctxes[i] = conf;
	}

	return alloc_ctxes;
}
EXPORT_SYMBOL_GPL(vb2_dma_pool_init_multi);

void vb2_dma_pool_cleanup_multi(void **alloc_ctxes, unsigned int num_planes)
{
	int i;

	for (i = 0; i < num_planes; i++)
		vb2_dma_pool_cleanup(alloc_ctxes[i]);

	kfree(alloc_ctxes);
}
EXPORT_SYMBOL_GPL(vb2_dma_pool_cleanup_multi);

MODULE_DESCRIPTION("DMA-pool handling routines for videobuf2");
MODULE_AUTHOR("Jeongtae Park");
MODULE_LICENSE("GPL");

