/* linux/arch/arm/mach-s5pv310/dmc.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV310 - DMC support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/dmc.h>

void s5pv310_dmc_ppmu_reset(struct s5pv310_dmc_ppmu_hw *ppmu)
{
	void __iomem *dmc_base = ppmu->dmc_hw_base;

	__raw_writel(0x8000000f, dmc_base + 0xf010);
	__raw_writel(0x8000000f, dmc_base + 0xf050);
	__raw_writel(0x6, dmc_base + 0xf000);
	__raw_writel(0x0, dmc_base + 0xf100);

	ppmu->ccnt = 0;
	ppmu->event = 0;
	ppmu->count = 0;
}

void s5pv310_dmc_ppmu_setevent(struct s5pv310_dmc_ppmu_hw *ppmu,
				  unsigned int evt)
{
	void __iomem *dmc_base = ppmu->dmc_hw_base;

	ppmu->event = evt;

	__raw_writel(((evt << 12) | 0x1), dmc_base + 0xfc);
}

void s5pv310_dmc_ppmu_start(struct s5pv310_dmc_ppmu_hw *ppmu)
{
	void __iomem *dmc_base = ppmu->dmc_hw_base;

	__raw_writel(0x1, dmc_base + 0xf000);
}

void s5pv310_dmc_ppmu_stop(struct s5pv310_dmc_ppmu_hw *ppmu)
{
	void __iomem *dmc_base = ppmu->dmc_hw_base;

	__raw_writel(0x0, dmc_base + 0xf000);
}

void s5pv310_dmc_ppmu_update(struct s5pv310_dmc_ppmu_hw *ppmu)
{
	void __iomem *dmc_base = ppmu->dmc_hw_base;

	ppmu->ccnt = __raw_readl(dmc_base + 0xf100);
	ppmu->count = __raw_readl(dmc_base + 0xf110);
}

void set_refresh_rate(unsigned int auto_refresh)
{
	/*
	 * uRlk = FIN / 100000;
	 * refresh_usec =  (unsigned int)(fMicrosec * 10);
	 * uRegVal = ((unsigned int)(uRlk * uMicroSec / 100)) - 1;
	*/
	/* change auto refresh period of dmc0 */
	__raw_writel(auto_refresh, S5P_VA_DMC0 + TIMMING_AREF);

	/* change auto refresh period of dmc1 */
	__raw_writel(auto_refresh, S5P_VA_DMC1 + TIMMING_AREF);
}
