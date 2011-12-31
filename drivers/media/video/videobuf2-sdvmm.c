/* linux/drivers/media/video/videobuf2-sdvmm.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Implementation of SDVMM memory allocator for videobuf2
 * SDVMM : Shared Device Virtual Memory Management
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/cma.h>
#include <linux/vcm-drv.h>

#include <asm/cacheflush.h>

#include <plat/s5p-vcm.h>
#include <media/videobuf2-sdvmm.h>

#include "ump_kernel_interface.h"
#include "ump_kernel_interface_ref_drv.h"
#include "ump_kernel_interface_vcm.h"

static int sdvmm_debug;
module_param(sdvmm_debug, int, 0644);
#define dbg(level, fmt, arg...)						\
	do {								\
		if (sdvmm_debug >= level)				\
			printk(KERN_DEBUG "vb2_sdvmm: " fmt, ## arg);	\
	} while (0)

#define SIZE_THRESHOLD (1280 * 720 * 1.5)

struct vb2_sdvmm_conf {
	spinlock_t              slock;

	/* For CMA */
	struct device		*dev;
	const char		*type;
	unsigned long		alignment;
	bool			use_cma;

	/* For VCMM */
	struct vcm		*vcm_ctx;
	enum vcm_dev_id		vcm_id;

	/* SYS.MMU */
	bool			mmu_clk;

	bool			cacheable;
	bool			remap_dva;
};

struct vb2_sdvmm_buf {
	struct vm_area_struct		*vma;
	struct vb2_sdvmm_conf		*conf;
	struct vb2_vmarea_handler	handler;

	atomic_t			ref;
	unsigned long			size;

	struct vcm_res			*vcm_res;
	struct vcm_res			*vcm_res_kern;
	ump_dd_handle			ump_dd_handle;
	size_t				dva_offset;

	bool				cacheable;
	bool				remap_dva;
};

static void vb2_sdvmm_put(void *buf_priv);
static int _vb2_sdvmm_mmap_pfn_range(struct vm_area_struct *vma,
				     struct vcm_phys *vcm_phys,
				     unsigned long size,
				     const struct vm_operations_struct *vm_ops,
				     void *priv);

static void *_vb2_sdvmm_ump_register(struct vb2_sdvmm_buf *buf)
{
	struct vcm_phys_part	*part = buf->vcm_res->phys->parts;
	ump_dd_physical_block	*blocks;
	ump_dd_handle		*handle;
	struct ump_vcm		ump_vcm;
	int num_blocks = buf->vcm_res->phys->count;
	int block_size, i;

	block_size = sizeof(ump_dd_physical_block) * num_blocks;
	blocks = (ump_dd_physical_block *)vmalloc(block_size);
	for (i = 0; i < num_blocks; i++) {
		blocks[i].addr = part->start;
		blocks[i].size = part->size;
		++part;

		dbg(6, "block addr(0x%08x), size(0x%08x)\n",
			(u32)blocks[i].addr, (u32)blocks[i].size);
	}

	handle = ump_dd_handle_create_from_phys_blocks(blocks, num_blocks);
	vfree(blocks);
	if (handle == UMP_DD_HANDLE_INVALID) {
		pr_err("ump_dd_handle_create_from_phys_blocks failed\n");
		return ERR_PTR(-ENOMEM);
	}

	ump_vcm.vcm = buf->conf->vcm_ctx;
	ump_vcm.vcm_res = buf->vcm_res;
	ump_vcm.dev_id = buf->conf->vcm_id;

	if (ump_dd_meminfo_set(handle, (void *)&ump_vcm)) {
		ump_dd_reference_release(handle);
		return ERR_PTR(-EINVAL);
	}

	return (void *)handle;
}

static void _vb2_sdvmm_cma_free(struct vcm_phys *vcm_phys)
{
	cma_free(vcm_phys->parts[0].start);
	kfree(vcm_phys);
}

static void *vb2_sdvmm_alloc(void *alloc_ctx, unsigned long size)
{
	struct vb2_sdvmm_conf	*conf = alloc_ctx;
	struct vb2_sdvmm_buf	*buf;
	struct vcm_phys		*vcm_phys = NULL;
	dma_addr_t		paddr;
	unsigned long		aligned_size = ALIGN(size, SZ_4K);
	int ret;

	buf = kzalloc(sizeof *buf, GFP_KERNEL);
	if (!buf) {
		pr_err("no memory for vb2_sdvmm_conf\n");
		return ERR_PTR(-ENOMEM);
	}

	/* Set vb2_sdvmm_buf.conf and size */
	buf->conf = conf;
	buf->size = size;
	buf->cacheable = conf->cacheable;

	/* Allocate: physical memory */
	if (conf->use_cma) {	/* physically contiguous memory allocation */
		paddr = cma_alloc(conf->dev, conf->type, size, conf->alignment);
		if (IS_ERR((void *)paddr)) {
			pr_err("cma_alloc of size %ld failed\n", size);
			ret = -ENOMEM;
			goto err_alloc;
		}

		vcm_phys = kzalloc(sizeof(*vcm_phys) + sizeof(*vcm_phys->parts),
				   GFP_KERNEL);
		vcm_phys->count = 1;
		vcm_phys->size = aligned_size;
		vcm_phys->free = _vb2_sdvmm_cma_free;
		vcm_phys->parts[0].start = paddr;
		vcm_phys->parts[0].size = aligned_size;
	} else {
		vcm_phys = vcm_alloc(conf->vcm_ctx, aligned_size, 0);
		if (IS_ERR((struct vcm_phys *)vcm_phys)) {
			pr_err("vcm_alloc of size %ld failed\n", size);
			ret = -ENOMEM;
			goto err_alloc;
		}
	}
	dbg(6, "PA(0x%x)\n", vcm_phys->parts[0].start);

	/* Reserve & Bind: device virtual address */
	buf->vcm_res = vcm_map(conf->vcm_ctx, vcm_phys, 0);
	if (IS_ERR((struct vcm_res *)buf->vcm_res)) {
		pr_err("vcm_map of size %ld failed\n", size);
		ret = -ENOMEM;
		goto err_map;
	}
	dbg(6, "DVA(0x%x)\n", buf->vcm_res->start);

	/* Register: UMP */
	buf->ump_dd_handle = _vb2_sdvmm_ump_register(buf);
	if (IS_ERR(buf->ump_dd_handle)) {
		pr_err("ump_register failed\n");
		ret = -ENOMEM;
		goto err_ump;
	}

	/* Set struct vb2_vmarea_handler */
	buf->handler.refcount = &buf->ref;
	buf->handler.put = vb2_sdvmm_put;
	buf->handler.arg = buf;

	atomic_inc(&buf->ref);

	return buf;

err_ump:
	vcm_unmap(buf->vcm_res);

err_map:
	vcm_free(vcm_phys);

err_alloc:
	kfree(buf);

	return ERR_PTR(ret);
}

static void vb2_sdvmm_put(void *buf_priv)
{
	struct vb2_sdvmm_buf *buf = buf_priv;

	if (atomic_dec_and_test(&buf->ref)) {
		if (buf->vcm_res_kern)
			vcm_unmap(buf->vcm_res_kern);

		ump_dd_reference_release(buf->ump_dd_handle);

		kfree(buf);
	}

	dbg(6, "released: buf_refcnt(%d)\n", atomic_read(&buf->ref));
}

/**
 * _vb2_get_sdvmm_userptr() - lock userspace mapped memory
 * @vaddr:	starting virtual address of the area to be verified
 * @size:	size of the area
 * @res_vma:	will return locked copy of struct vm_area for the given area
 *
 * This function will go through memory area of size @size mapped at @vaddr
 * If they are contiguous the virtual memory area is locked and a @res_vma is
 * filled with the copy and @res_pa set to the physical address of the buffer.
 *
 * Returns 0 on success.
 */
static int _vb2_get_sdvmm_userptr(unsigned long vaddr, unsigned long size,
				  struct vm_area_struct **res_vma)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	unsigned long offset, start, end;
	int ret = -EFAULT;

	start = vaddr;
	offset = start & ~PAGE_MASK;
	end = start + size;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, start);

	if (vma == NULL || vma->vm_end < end)
		goto done;

	/* Lock vma and return to the caller */
	*res_vma = vb2_get_vma(vma);
	if (*res_vma == NULL) {
		ret = -ENOMEM;
		goto done;
	}
	ret = 0;

done:
	up_read(&mm->mmap_sem);
	return ret;
}

static void *vb2_sdvmm_get_userptr(void *alloc_ctx, unsigned long vaddr,
				   unsigned long size, int write)
{
	struct vb2_sdvmm_conf *conf = alloc_ctx;
	struct vb2_sdvmm_buf *buf = NULL;
	struct vm_area_struct *vma = NULL;
	struct vcm *vcm = NULL;
	struct vcm_res *vcm_res = NULL;
	ump_dd_handle ump_dd_handle = NULL;
	ump_secure_id secure_id = 0;
	size_t offset = 0;
	int ret;

	/* buffer should be registered in UMP before QBUF */
	ret = ump_dd_secure_id_get_from_vaddr(vaddr, &secure_id, &offset);
	if (ret) {
		pr_err("fail: get SecureID from vaddr(0x%08x)\n", (u32)vaddr);
		return ERR_PTR(-EINVAL);
	}

	ump_dd_handle = ump_dd_handle_create_from_secure_id(secure_id);
	if (ump_dd_handle == NULL) {
		pr_err("ump_dd_handle_get_from_vaddr failed\n");
		return ERR_PTR(-EINVAL);
	}

	buf = kzalloc(sizeof *buf, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	buf->vcm_res = (struct vcm_res *)ump_dd_meminfo_get(secure_id,
							(void *)conf->vcm_id);
	if (buf->vcm_res == NULL) {
		pr_err("ump_dd_meminfo_get failed\n");
		kfree(buf);
		return ERR_PTR(-EINVAL);
	}

	buf->dva_offset = offset;
	dbg(6, "dva(0x%x), size(0x%x), offset(0x%x)\n",
			(u32)buf->vcm_res->start, (u32)size, (u32)offset);

	vcm = vcm_find_vcm(conf->vcm_id);
	switch (vcm_reservation_in_vcm(vcm, buf->vcm_res)) {
	case S5PVCM_RES_IN_VCM:	/* No need to remap */
		break;

	case S5PVCM_RES_IN_ADDRSPACE:
		if (conf->remap_dva) {	/* Need to remap */
			vcm_res = buf->vcm_res;
			buf->vcm_res = vcm_map(vcm, vcm_res->phys, 0);
			buf->remap_dva = true;
			dbg(6, "remap: dva(0x%x)\n", (u32)buf->vcm_res->start);
		}

		break;

	case S5PVCM_RES_NOT_IN_VCM:
		pr_err("fail: vcm_reservation_in_vcm\n");
		ump_dd_reference_release(ump_dd_handle);
		kfree(buf);
		return ERR_PTR(-EINVAL);
	}

	ret = _vb2_get_sdvmm_userptr(vaddr, size, &vma);
	if (ret) {
		pr_err("Failed acquiring VMA 0x%08lx\n", vaddr);
		ump_dd_reference_release(ump_dd_handle);
		kfree(buf);
		return ERR_PTR(ret);
	}

	buf->conf = conf;
	buf->size = size;
	buf->vma = vma;
	buf->ump_dd_handle = ump_dd_handle;

	return buf;
}

static void vb2_sdvmm_put_userptr(void *mem_priv)
{
	struct vb2_sdvmm_buf *buf = mem_priv;

	if (!buf) {
		pr_err("No buffer to put\n");
		return;
	}

	if (buf->remap_dva)	/* Need to unmap */
		vcm_unmap(buf->vcm_res);

	ump_dd_reference_release(buf->ump_dd_handle);

	vb2_put_vma(buf->vma);

	kfree(buf);
}

static void *vb2_sdvmm_cookie(void *buf_priv)
{
	struct vb2_sdvmm_buf *buf = buf_priv;

	return (void *)(buf->vcm_res->start + buf->dva_offset);
}

static void *vb2_sdvmm_vaddr(void *buf_priv)
{
	struct vb2_sdvmm_buf *buf = buf_priv;

	if (!buf) {
		pr_err("failed to get buffer\n");
		return NULL;
	}

	if (!buf->vcm_res_kern) {
		buf->vcm_res_kern = vcm_map(vcm_vmm, buf->vcm_res->phys, 0);
		if (IS_ERR(buf->vcm_res_kern)) {
			pr_err("failed to get kernel virtual\n");
			return NULL;
		}
	}

	return (void *)buf->vcm_res_kern->start;
}

static unsigned int vb2_sdvmm_num_users(void *buf_priv)
{
	struct vb2_sdvmm_buf *buf = buf_priv;

	return atomic_read(&buf->ref);
}

static int vb2_sdvmm_mmap(void *buf_priv, struct vm_area_struct *vma)
{
	struct vb2_sdvmm_buf *buf = buf_priv;

	if (!buf) {
		pr_err("No buffer to map\n");
		return -EINVAL;
	}

	if (!buf->cacheable)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	return _vb2_sdvmm_mmap_pfn_range(vma, buf->vcm_res->phys, buf->size,
				&vb2_common_vm_ops, &buf->handler);
}

const struct vb2_mem_ops vb2_sdvmm_memops = {
	.alloc		= vb2_sdvmm_alloc,
	.put		= vb2_sdvmm_put,
	.cookie		= vb2_sdvmm_cookie,
	.vaddr		= vb2_sdvmm_vaddr,
	.mmap		= vb2_sdvmm_mmap,
	.get_userptr	= vb2_sdvmm_get_userptr,
	.put_userptr	= vb2_sdvmm_put_userptr,
	.num_users	= vb2_sdvmm_num_users,
};
EXPORT_SYMBOL_GPL(vb2_sdvmm_memops);

void vb2_sdvmm_set_cacheable(void *alloc_ctx, bool cacheable)
{
	((struct vb2_sdvmm_conf *)alloc_ctx)->cacheable = cacheable;
}

bool vb2_sdvmm_get_cacheable(void *alloc_ctx)
{
	return ((struct vb2_sdvmm_conf *)alloc_ctx)->cacheable;
}

static void _vb2_sdvmm_cache_flush_all(void)
{
	flush_cache_all();	/* L1 */
	outer_flush_all();	/* L2 */
}

static void _vb2_sdvmm_cache_flush_range(struct vb2_sdvmm_buf *buf)
{
	struct vcm_phys *vcm_phys = buf->vcm_res->phys;
	phys_addr_t start, end;
	int count = vcm_phys->count;
	int i;

	/* sequentially traversal phys */
	for (i = 0; i < count; i++) {
		start = vcm_phys->parts[i].start;
		end = start + vcm_phys->parts[i].size - 1;

		dmac_flush_range(phys_to_virt(start), phys_to_virt(end));
		outer_flush_range(start, end);	/* L2 */
	}
}

int vb2_sdvmm_cache_flush(void *alloc_ctx, struct vb2_buffer *vb, u32 plane_no)
{
	struct vb2_sdvmm_buf *buf = vb->planes[plane_no].mem_priv;

	if (!buf->cacheable) {
		pr_warning("This is non-cacheable buffer allocator\n");
		return -EINVAL;
	}

	if (buf->size > (unsigned long)SIZE_THRESHOLD)
		_vb2_sdvmm_cache_flush_all();
	else
		_vb2_sdvmm_cache_flush_range(buf);

	return 0;
}

void vb2_sdvmm_suspend(void *alloc_ctx)
{
	struct vb2_sdvmm_conf *conf = alloc_ctx;
	unsigned long flags;

	spin_lock_irqsave(&conf->slock, flags);
	if (!conf->mmu_clk) {
		pr_warning("Already suspend: vcm_id(%d)\n", conf->vcm_id);
		spin_unlock_irqrestore(&conf->slock, flags);
		return;
	}

	conf->mmu_clk = false;
	s5p_vcm_turn_off(conf->vcm_ctx);

	spin_unlock_irqrestore(&conf->slock, flags);
}

void vb2_sdvmm_resume(void *alloc_ctx)
{
	struct vb2_sdvmm_conf *conf = alloc_ctx;
	unsigned long flags;

	spin_lock_irqsave(&conf->slock, flags);

	if (conf->mmu_clk) {
		pr_warning("Already resume: vcm_id(%d)\n", conf->vcm_id);
		spin_unlock_irqrestore(&conf->slock, flags);
		return;
	}

	conf->mmu_clk = true;
	s5p_vcm_turn_on(conf->vcm_ctx);

	spin_unlock_irqrestore(&conf->slock, flags);
}

void *vb2_sdvmm_init(struct vb2_vcm *vcm,
		     struct vb2_cma *cma,
		     struct vb2_drv *drv)
{
	struct vb2_sdvmm_conf *conf;
	int ret;

	conf = kzalloc(sizeof *conf, GFP_KERNEL);
	if (!conf)
		return ERR_PTR(-ENOMEM);

	if (cma != NULL) {
		conf->dev	= cma->dev;
		conf->type	= cma->type;
		conf->alignment = cma->alignment;
		conf->use_cma	= true;
	}

	conf->vcm_id = vcm->vcm_id;
	conf->vcm_ctx = vcm_create_unified(vcm->size, vcm->vcm_id, NULL);
	if (IS_ERR(conf->vcm_ctx)) {
		pr_err("vcm_create failed: vcm_id(%d), size(%ld)\n",
				conf->vcm_id, (long int)vcm->size);
		goto err_vcm_create;
	}

	s5p_vcm_turn_off(conf->vcm_ctx);
	ret = vcm_activate(conf->vcm_ctx);
	if (ret < 0) {
		pr_err("vcm_activate failed\n");
		goto err_vcm_activate;
	}

	conf->mmu_clk	= false;
	conf->cacheable = drv->cacheable;
	conf->remap_dva = drv->remap_dva;

	spin_lock_init(&conf->slock);

	return conf;

err_vcm_activate:
	s5p_vcm_turn_off(conf->vcm_ctx);
	vcm_destroy(conf->vcm_ctx);

err_vcm_create:
	kfree(conf);

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(vb2_sdvmm_init);

void vb2_sdvmm_cleanup(void *alloc_ctx)
{
	struct vb2_sdvmm_conf *local_conf = alloc_ctx;

	vcm_deactivate(local_conf->vcm_ctx);
	vcm_destroy(local_conf->vcm_ctx);
	kfree(alloc_ctx);
}
EXPORT_SYMBOL_GPL(vb2_sdvmm_cleanup);

void **vb2_sdvmm_init_multi(unsigned int num_planes,
			    struct vb2_vcm *vcm,
			    struct vb2_cma *cma[],
			    struct vb2_drv *drv)
{
	struct vb2_sdvmm_conf *conf;
	struct vcm *vcm_ctx;
	void **alloc_ctxes;
	u32 i, ret;

	/* allocate structure of alloc_ctxes */
	alloc_ctxes = kzalloc((sizeof *alloc_ctxes + sizeof *conf) * num_planes,
			      GFP_KERNEL);

	if (!alloc_ctxes)
		return ERR_PTR(-ENOMEM);

	vcm_ctx = vcm_create_unified(vcm->size, vcm->vcm_id, NULL);
	if (IS_ERR(vcm_ctx)) {
		pr_err("vcm_create of size %ld failed\n", (long int)vcm->size);
		goto err_vcm_create;
	}

	s5p_vcm_turn_off(vcm_ctx);
	ret = vcm_activate(vcm_ctx);
	if (ret < 0) {
		pr_err("vcm_activate failed\n");
		goto err_vcm_activate;
	}

	conf = (void *)(alloc_ctxes + num_planes);

	for (i = 0; i < num_planes; ++i, ++conf) {
		alloc_ctxes[i] = conf;
		if ((cma != NULL) && (cma[i] != NULL)) {
			conf->dev	= cma[i]->dev;
			conf->type	= cma[i]->type;
			conf->alignment = cma[i]->alignment;
			conf->use_cma	= true;
		}
		conf->vcm_ctx	= vcm_ctx;
		conf->vcm_id	= vcm->vcm_id;
		conf->mmu_clk	= false;
		conf->cacheable = drv->cacheable;
		conf->remap_dva = drv->remap_dva;
		spin_lock_init(&conf->slock);
	}

	return alloc_ctxes;

err_vcm_activate:
	s5p_vcm_turn_off(vcm_ctx);
	vcm_destroy(vcm_ctx);

err_vcm_create:
	kfree(alloc_ctxes);

	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(vb2_sdvmm_init_multi);

void vb2_sdvmm_cleanup_multi(void **alloc_ctxes)
{
	struct vb2_sdvmm_conf *local_conf = alloc_ctxes[0];

	vcm_deactivate(local_conf->vcm_ctx);
	vcm_destroy(local_conf->vcm_ctx);

	kfree(alloc_ctxes);
}
EXPORT_SYMBOL_GPL(vb2_sdvmm_cleanup_multi);

/**
 * _vb2_sdvmm_mmap_pfn_range() - map physical pages(vcm) to userspace
 * @vma:	virtual memory region for the mapping
 * @vcm_phys:	vcm physical group information to be mapped
 * @size:	size of the memory to be mapped
 * @vm_ops:	vm operations to be assigned to the created area
 * @priv:	private data to be associated with the area
 *
 * Returns 0 on success.
 */
static int _vb2_sdvmm_mmap_pfn_range(struct vm_area_struct *vma,
				      struct vcm_phys *vcm_phys,
				      unsigned long size,
				      const struct vm_operations_struct *vm_ops,
				      void *priv)
{
	unsigned long org_vm_start = vma->vm_start;
	int ret, i;
	int count = vcm_phys->count;
	int mapped_size = 0;
	int vma_size = vma->vm_end - vma->vm_start;
	int remap_break = 0;
	resource_size_t remap_size;

	/* sequentially physical-virtual mapping */
	for (i = 0; (i < count && !remap_break); i++) {
		if ((mapped_size + vcm_phys->parts[i].size) > vma_size) {
			remap_size = vma_size - mapped_size;
			remap_break = 1;
		} else {
			remap_size = vcm_phys->parts[i].size;
		}

		ret = remap_pfn_range(vma, vma->vm_start,
				vcm_phys->parts[i].start >> PAGE_SHIFT,
				remap_size, vma->vm_page_prot);
		if (ret) {
			pr_err("Remapping failed, error: %d\n", ret);
			return ret;
		}

		dbg(6, "%dth page vaddr(0x%08x), paddr(0x%08x),	size(0x%08x)\n",
			i, (u32)vma->vm_start, vcm_phys->parts[i].start,
			vcm_phys->parts[i].size);

		mapped_size += remap_size;
		vma->vm_start += vcm_phys->parts[i].size;
	}

	WARN_ON(size > mapped_size);

	/* re-assign initial start address */
	vma->vm_start		= org_vm_start;
	vma->vm_flags		|= VM_DONTEXPAND | VM_RESERVED;
	vma->vm_private_data	= priv;
	vma->vm_ops		= vm_ops;

	vma->vm_ops->open(vma);

	return 0;
}

MODULE_AUTHOR("Sewoon Park <senui.park@samsung.com>");
MODULE_AUTHOR("Jonghun,	Han <jonghun.han@samsung.com>");
MODULE_DESCRIPTION("SDVMM allocator handling routines for videobuf2");
MODULE_LICENSE("GPL");
