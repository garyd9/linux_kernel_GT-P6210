/* drivers/char/s3c_dma_mem.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Header file for s3c dma mem
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#ifndef _S3C_DMA_MEM_H
#define _S3C_DMA_MEM_H

#include <mach/dma.h>

#define DEBUG_S3C_DMA_MEM
#undef  DEBUG_S3C_DMA_MEM

#ifdef DEBUG_S3C_DMA_MEM
#define DEBUG(fmt, args...)	printk(fmt, ##args)
#else
#define DEBUG(fmt, args...)	do {} while (0)
#endif

struct s3c_mem_dma_param {
	int size;
	unsigned int src_addr;
	unsigned int dst_addr;
	int cfg;
};

void s3c_dma_mem_init(void);
int s3c_dma_mem_start(struct mm_struct *mm, struct s3c_mem_dma_param *dma_param,
		      enum s3c2410_dmasrc source);

#define s3c_dma_mem_cpy(dma_param) \
	s3c_dma_mem_start(&init_mm, dma_param, S3C_DMA_MEM2MEM);
#endif
