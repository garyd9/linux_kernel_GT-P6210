/* linux/arch/arm/mach-s5pv310/include/mach/dmc.h
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

#ifndef __ASM_ARCH_DMC_H
#define __ASM_ARCH_DMC_H __FILE__

#define NUMBER_OF_COUNTER	2

#define DEVT0_SEL       0x1000
#define DEVT0_ID	0x1010
#define DEVT0_IDMSK     0x1014
#define DEVT1_ID	0x1110
#define DEVT1_IDMSK     0x1114
#define DEVT2_ID	0x1210
#define DEVT2_IDMSK     0x1214
#define DEVT3_ID	0x1310
#define DEVT3_IDMSK     0x1314


#define DMC_NUM		2

#define DMC_READWRITE_DATA_EVT  0x6
#define CPU_READ_DATA_EVT	0x5
#define CPU_WRITE_DATA_EVT	0x6

/* memory throttling */
#define TIMMING_AREF 0x30
#define FIN	(24*1000*1000)
#define AUTO_REFRESH_PERIOD_TQ0		0x2E /* auto refresh preiod 1.95us */
#define AUTO_REFRESH_PERIOD_NORMAL	0x5D /* auto refresh period 3.9us */

enum ppmu_counter {
	PPMU_PMNCNT0,
	PPMU_PMCCNT1,
	PPMU_PMNCNT2,
	PPMU_PMNCNT3,
	PPMU_PMNCNT_MAX,
};

enum dmc_ch {
	DMC0,
	DMC1,
};

struct s5pv310_dmc_ppmu_hw {
	void __iomem *dmc_hw_base;
	unsigned int ccnt;
	unsigned int event;
	unsigned int count;
};

struct s5pv310_cpu_ppmu_hw {
	void __iomem *cpu_hw_base;
	unsigned int id;
	unsigned int ccnt;
	unsigned int event;
	unsigned int count[NUMBER_OF_COUNTER];
};
extern void s5pv310_dmc_ppmu_reset(struct s5pv310_dmc_ppmu_hw *ppmu);
extern void s5pv310_dmc_ppmu_start(struct s5pv310_dmc_ppmu_hw *ppmu);
extern void s5pv310_dmc_ppmu_stop(struct s5pv310_dmc_ppmu_hw *ppmu);
extern void s5pv310_dmc_ppmu_setevent(struct s5pv310_dmc_ppmu_hw *ppmu,
				  unsigned int evt);
extern void s5pv310_dmc_ppmu_update(struct s5pv310_dmc_ppmu_hw *ppmu);

extern void s5pv310_cpu_ppmu_reset(struct s5pv310_cpu_ppmu_hw *ppmu);
extern void s5pv310_cpu_ppmu_start(struct s5pv310_cpu_ppmu_hw *ppmu);
extern void s5pv310_cpu_ppmu_stop(struct s5pv310_cpu_ppmu_hw *ppmu);
extern void s5pv310_cpu_ppmu_setevent(struct s5pv310_cpu_ppmu_hw *ppmu,
				  unsigned int evt, unsigned int evt_num);
extern void s5pv310_cpu_ppmu_update(struct s5pv310_cpu_ppmu_hw *ppmu);

extern void set_refresh_rate(unsigned int auto_refresh);
#endif /* __ASM_ARCH_DMC_H */
