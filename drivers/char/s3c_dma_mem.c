/* drivers/char/s3c_dma_mem.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * s3c dma mem driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/errno.h>	/* error codes */
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/mman.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#include <linux/unistd.h>
#include <linux/version.h>
#include <mach/map.h>
#include <mach/hardware.h>
#include <linux/semaphore.h>
#include <asm/cacheflush.h>
#include <plat/sysmmu.h>

#include "s3c_dma_mem.h"

#define S3C_DMA_MEM_MAX_CH (DMACH_MTOM_7 - DMACH_MTOM_0 + 1)
#define S3C_DMA_MEM_WIDTH (8)

static DEFINE_SPINLOCK(res_lock);
static LIST_HEAD(chan_list);
#define DMA_MEM_TIMEOUT_MS (5*1000)
#define CH_ALLOC_TRY_COUNT (5)
enum ch_status {
	CH_RUN,
	CH_RESERVED,
	CH_LOADED,
	CH_IDLE,
};

struct s3c_dma_mem_ch {
	struct completion xfer_cmplt;
	struct list_head node;
	enum ch_status status;
	int id;
	struct mm_struct *mm;
};
struct s3c_dma_mem_client {
	struct s3c2410_dma_client cl;
	struct s3c_dma_mem_ch ch[S3C_DMA_MEM_MAX_CH];
	struct completion xfer_cmplt;
	struct semaphore mutex;
};

static struct s3c_dma_mem_client s3c_dma_mem_client = {
	.cl = {
	       .name = "s3c-dma-mem",
	       },
};

#ifdef DEBUG_S3C_DMA_MEM
static bool dmatest_buf_same(u32 src[], u32 dst[], unsigned int bytes)
{
	unsigned int i;

	for (i = 0; i < bytes / 4; i++) {
		if (src[i] != dst[i])
			return false;
	}
	for (i = 0; i < bytes / 4; i++) {
		if (dst[i])
			return false;
	}
	return true;
}
#endif

__init void s3c_dma_mem_init(void)
{
	int i;
	struct s3c_dma_mem_ch *ch;

	for (i = 0; i < S3C_DMA_MEM_MAX_CH; i++) {
		init_completion(&s3c_dma_mem_client.ch[i].xfer_cmplt);
		s3c_dma_mem_client.ch[i].status = CH_IDLE;
		s3c_dma_mem_client.ch[i].id = DMACH_MTOM_0 + i;
		list_add_tail(&s3c_dma_mem_client.ch[i].node, &chan_list);
		printk(KERN_INFO "%s: create channels = &ch[%d] = 0x%x, id = %d\n",
			__func__, i, (unsigned int)&s3c_dma_mem_client.ch[i],
			s3c_dma_mem_client.ch[i].id);
	}

	list_for_each_entry(ch, &chan_list, node) {
		DEBUG("%s: create channels = &ch = 0x%x, id = %d\n", __func__,
			ch, ch->id);
	}
	init_MUTEX(&s3c_dma_mem_client.mutex);
	init_completion(&s3c_dma_mem_client.xfer_cmplt);
}
EXPORT_SYMBOL_GPL(s3c_dma_mem_init);

#if defined(CONFIG_OUTER_CACHE) && defined(CONFIG_ARM)
static void s3c_dma_mem_flush_outercache_pagetable(struct mm_struct *mm,
						   unsigned long addr,
						   unsigned long size)
{
	unsigned long end;
	pgd_t *pgd, *pgd_end;
	pmd_t *pmd;
	pte_t *pte, *pte_end;
	unsigned long next;

	end = addr + size;
	addr &= PAGE_MASK;
	end = PAGE_ALIGN(end + PAGE_SIZE);
	pgd = pgd_offset(mm, addr);
	pgd_end = pgd_offset(mm, (addr + size + PGDIR_SIZE - 1) & PGDIR_MASK);

	/* Clean L1 page table entries */
	outer_flush_range(virt_to_phys(pgd), virt_to_phys(pgd_end));

	/* clean L2 page table entries */
	/* this regards pgd == pmd and no pud */
	do {
		next = pgd_addr_end(addr, end);
		pgd = pgd_offset(mm, addr);
		pmd = pmd_offset(pgd, addr);
		pte = pte_offset_map(pmd, addr) - PTRS_PER_PTE;
		pte_end = pte_offset_map(pmd, next - 4) - PTRS_PER_PTE + 1;
		outer_flush_range(virt_to_phys(pte), virt_to_phys(pte_end));
		addr = next;
	} while (addr != end);
}
#else
#define s3c_dma_mem_flush_outercache_pagetable(mm, addr, size) do { } while (0)
#endif /* CONFIG_OUTER_CACHE && CONFIG_ARM */

static void s3c_dma_mem_flush_cache(struct mm_struct *mm, unsigned int res_addr,
				    int size)
{
	struct page *page = NULL;
	unsigned int addr = res_addr;
	pgd_t *pgd;
	unsigned int num_pages;
	unsigned int i;

	num_pages = size >> PAGE_SHIFT;
	num_pages += 1;

	for (i = 0; i < num_pages; i++) {
		page = NULL;
		pgd = pgd_offset(mm, addr);
		if (!pgd_none(*pgd)) {
			pud_t *pud = pud_offset(pgd, addr);
			if (!pud_none(*pud)) {
				pmd_t *pmd = pmd_offset(pud, addr);
				if (!pmd_none(*pmd)) {
					pte_t *ptep, pte;

					ptep = pte_offset_map(pmd, addr);
					pte = *ptep;
					if (pte_present(pte))
						page = pte_page(pte);
					else
						goto err;
					pte_unmap(ptep);
				} else {
					goto err;
				}
			} else {
				goto err;
			}
		} else {
			goto err;
		}

		if (page) {
			dmac_flush_range(page_address(page),
					 page_address(page) + PAGE_SIZE);
			outer_flush_range(page_to_phys(page),
					  page_to_phys(page) + PAGE_SIZE);
		} else {
			goto err;
		}
		addr += PAGE_SIZE;
	}

	return;
err:
	printk(KERN_WARNING "%s: l2 cache flush error.\n", __func__);
}

static void s3c_dma_mem_flush(struct mm_struct *mm,
			      struct s3c_mem_dma_param *dma_param,
			      enum s3c2410_dmasrc source)
{
	if (source == S3C_DMA_MEM2MEM) {
		s3c_dma_mem_flush_cache(mm, dma_param->src_addr,
					dma_param->size);
		s3c_dma_mem_flush_cache(mm, dma_param->dst_addr,
					dma_param->size);
	} else if (source == S3C_DMA_MEM2MEM_SET) {
		s3c_dma_mem_flush_cache(mm, dma_param->dst_addr,
					dma_param->size);
	}
}

static void s3c_dma_mem_finish(struct s3c2410_dma_chan *chan, void *donech,
			       int size, enum s3c2410_dma_buffresult res)
{
	struct s3c_dma_mem_ch *ch = (struct s3c_dma_mem_ch *)donech;

	DEBUG("%s done : ch: %d\n", __func__, ch->id);
	complete(&ch->xfer_cmplt);
}

static struct s3c_dma_mem_ch *s3c_dma_mem_alloc_ch(struct mm_struct *mm)
{
	struct s3c_dma_mem_ch *ch;
	unsigned trycount = 0;
	unsigned find = 0;

alloc_ch:
	ch = NULL;

	spin_lock(&res_lock);
	list_for_each_entry(ch, &chan_list, node) {
		if (ch->status == CH_IDLE) {
			ch->status = CH_RESERVED;
			find = 1;
			break;
		}
	}
	spin_unlock(&res_lock);

	/* doesn't exist useful channel.
	 * wait to release first loaded channel
	 */
	while ((!find) && (trycount < CH_ALLOC_TRY_COUNT)) {
		DEBUG("%s:  Doesn't exist useful channel\n", __func__);
		if (trycount > CH_ALLOC_TRY_COUNT) {
			ch = NULL;
			goto alloc_end;
		}
		wait_for_completion_timeout(&s3c_dma_mem_client.xfer_cmplt,
			msecs_to_jiffies(DMA_MEM_TIMEOUT_MS *
						     S3C_DMA_MEM_MAX_CH));
		trycount++;
		DEBUG("%s:  Allocation tries again.	\
			doesn't exist useful channel\n", __func__);
		goto alloc_ch;
	}

	ch->mm = mm;
alloc_end:
	if (ch) {
		if (trycount == 0)
			DEBUG("%s: 1st Alloc:		\
				ch->id: %d, ch: 0x%x, ch->pgd: 0x%x\n",
				__func__, ch->id, ch, ch->mm->pgd);
		else if (trycount == 1)
			DEBUG("%s: 2nd Alloc:		\
				ch->id: %d, ch: 0x%x, ch->pgd: 0x%x\n",
				__func__, ch->id, ch, ch->mm->pgd);
		else
			DEBUG
			("%s: Error: ch->id: %d, ch: 0x%x, ch->pgd: 0x%x\n",
				__func__, ch->id, ch, ch->mm->pgd);
	}
	return ch;
}

static int s3c_dma_mem_load(struct s3c_dma_mem_ch *ch, dma_addr_t src_addr,
			    dma_addr_t dst_addr, int size,
			    enum s3c2410_dmasrc source)
{
	int res = 0;

	if (down_interruptible(&s3c_dma_mem_client.mutex)) {
		printk(KERN_WARNING "%s: ch %d, fail to get sema.\n",
			__func__, ch->id);
		return -EAGAIN;
	}

	/* request dma for transfer */
	if (s3c2410_dma_request(ch->id, &s3c_dma_mem_client.cl, NULL)) {
		printk(KERN_WARNING "%s: ch %d, Unable to get DMA channel.\n",
		       __func__, ch->id);
		return -EAGAIN;
	}
	if (s3c2410_dma_set_buffdone_fn(ch->id, s3c_dma_mem_finish)) {
		printk(KERN_WARNING
		       "%s: ch %d, Fail to set callback function.\n", __func__,
		       ch->id);
		return -EAGAIN;
	}
	if (s3c2410_dma_config(ch->id, S3C_DMA_MEM_WIDTH)) {
		printk(KERN_WARNING "%s: ch %d, Fail to config dma.\n",
		       __func__, ch->id);
		return -EAGAIN;
	}
	if (s3c2410_dma_devconfig(ch->id, source, src_addr)) {
		printk(KERN_WARNING "%s: ch %d, Fail to config dev_dma.\n",
		       __func__, ch->id);
		return -EAGAIN;
	}
	if (s3c2410_dma_enqueue(ch->id, (void *)ch, dst_addr, size)) {
		printk(KERN_WARNING "%s: ch %d, Fail to queue.\n", __func__,
		       ch->id);
		return -EAGAIN;
	}

	DEBUG("ch: %d, loaded:	\
		src: 0x%x, dst:0x%x, size: 0x%x, config: %d\n",
		ch->id, src_addr, dst_addr, size, source);


	ch->status = CH_LOADED;

	return res;
}

static int s3c_dma_mem_trigger(struct s3c_dma_mem_ch *load_ch,
			       dma_addr_t src_addr, dma_addr_t dst_addr,
			       int size)
{
	s3c_dma_mem_flush_outercache_pagetable(load_ch->mm, src_addr, size);
	s3c_dma_mem_flush_outercache_pagetable(load_ch->mm, dst_addr, size);
	sysmmu_on(SYSMMU_MDMA2, __pa(load_ch->mm->pgd));

	load_ch->status = CH_RUN;
	if (s3c2410_dma_ctrl(load_ch->id, S3C2410_DMAOP_START)) {
		printk(KERN_WARNING "%s: ch %d, Fail to start dma.\n", __func__,
		       load_ch->id);
		return -EAGAIN;
	}
	DEBUG("ch %d,start dma\n", load_ch->id);

	return 0;
}

static int s3c_dma_mem_end(struct s3c_dma_mem_ch *ch)
{
	/* wait for dma operation. */
	wait_for_completion_timeout(&ch->xfer_cmplt,
				    msecs_to_jiffies(DMA_MEM_TIMEOUT_MS));
	DEBUG("ch: %d, dma done.\n", ch->id);

	sysmmu_off(SYSMMU_MDMA2);

	if (s3c2410_dma_free(ch->id, &s3c_dma_mem_client.cl)) {
		printk(KERN_WARNING "%s: ch %d, Fail to free dma.\n", __func__,
		       ch->id);
		ch->status = CH_IDLE;
		return -EAGAIN;
	}
	ch->status = CH_IDLE;

	up(&s3c_dma_mem_client.mutex);

	return 0;
}

int s3c_dma_mem_start(struct mm_struct *mm, struct s3c_mem_dma_param *dma_param,
		      enum s3c2410_dmasrc source)
{
	struct s3c_dma_mem_ch *ch;
	int res = 0;

#ifdef DEBUG_S3C_DMA_MEM
	unsigned int tstart, tend;
	tstart = jiffies;
#endif

	if (dma_param->size % S3C_DMA_MEM_WIDTH) {
		printk(KERN_WARNING
		       "%s: Size should be alinged with 16bust * Dword.\n",
		       __func__);
		return -EINVAL;
	}

	if ((source != S3C_DMA_MEM2MEM) && (source != S3C_DMA_MEM2MEM_SET)) {
		printk(KERN_WARNING "%s: invalid operation.\n", __func__);
		return -EINVAL;
	}

	ch = s3c_dma_mem_alloc_ch(mm);
	if (ch == NULL) {
		printk(KERN_WARNING "%s: ch %d, Fail to allocate channel.\n",
		       __func__, ch->id);
		ch->status = CH_IDLE;
		return -EAGAIN;
	}

	/* load a dma operation. */
	if (s3c_dma_mem_load
	    (ch, dma_param->src_addr, dma_param->dst_addr, dma_param->size,
	     source)) {
		printk(KERN_WARNING "%s: ch %d, Fail to loading error.\n",
		       __func__, ch->id);
		res = -EAGAIN;
		goto end_func;
	}

	/* if cacheable, flush cache */
	if (dma_param->cfg)
		s3c_dma_mem_flush(mm, dma_param, source);

	/* trigger the own dma operation. */
	if (s3c_dma_mem_trigger
	    (ch, dma_param->dst_addr, dma_param->src_addr, dma_param->size)) {
		printk(KERN_WARNING "%s: ch %d, Fail to trigger 1st.\n",
			__func__, ch->id);
		res = -EAGAIN;
		goto end_func;
	}

	/* dma end operation. */
	s3c_dma_mem_end(ch);

#ifdef DEBUG_S3C_DMA_MEM
	tend = jiffies;
	if (!dmatest_buf_same
	    (dma_param->src_addr, dma_param->dst_addr, dma_param->size)) {
		printk(KERN_ERR
			"[s3c-dma-perf] failed: ch: %d, src: 0x%x,\
			dst: 0x%x, size: 0x%x\n", ch->id,
			dma_param->src_addr, dma_param->dst_addr,
			dma_param->size);
	} else {
		if (jiffies_to_usecs(tend - tstart))
			DEBUG("[s3c-dma-perf] success: ch: %d, MBps : %u\n",
			ch->id,	dma_param->size /
			jiffies_to_usecs(tend - tstart));
	}
#endif
	return res;

end_func:
	printk(KERN_WARNING "%s: dma mem is failed\n", __func__);
	s3c2410_dma_free(ch->id, &s3c_dma_mem_client.cl);
	ch->status = CH_IDLE;
	return res;
}
EXPORT_SYMBOL_GPL(s3c_dma_mem_start);
