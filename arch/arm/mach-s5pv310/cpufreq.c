/* linux/arch/arm/mach-s5pv310/cpufreq.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * S5PV310 - CPU frequency scaling support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/cpufreq.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#ifdef CONFIG_S5PV310_ASV
#include <mach/regs-iem.h>
#endif

#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>

#include <plat/pm.h>
#include <plat/s5pv310.h>

#include <mach/cpufreq.h>
#include <mach/dmc.h>
#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/pm-core.h>

#define CPUFREQ_SAMPLING_RATE      40000

static struct clk *arm_clk;
static struct clk *moutcore;
static struct clk *mout_mpll;
static struct clk *mout_apll;
static struct clk *sclk_dmc;

#ifdef CONFIG_REGULATOR
static struct regulator *arm_regulator;
static struct regulator *int_regulator;
/* If the PMIC uses I2C interface,
 * disable cpufreq from i2c driver suspend to resume */
static bool disable_further_cpufreq;
#endif

/* CPU & BUS freq lock information */
static struct freq_lock_info cpufreq_lock = {
	.level		= (CPUFREQ_LEVEL_END - 1),
	.min_level	= (CPUFREQ_LEVEL_END - 1),
	.disable_lock	= false,
};

static struct freq_lock_info cpufreq_upper_lock = {
	.level		= CPU_L0,
	.max_level	= CPU_L0,
	.disable_lock	= false,
};

static struct freq_lock_info busfreq_lock = {
	.level		= (BUSFREQ_LEVEL_END - 1),
	.min_level	= (BUSFREQ_LEVEL_END - 1),
};

/*********************************************************************
 *                     CPUFREQ DEFINITIONS                           *
 *********************************************************************/

static DEFINE_MUTEX(set_cpu_freq_change);
static DEFINE_MUTEX(set_cpu_freq_lock);

static struct s5pv310_cpufreq_info cpufreq_info;
static struct s5pv310_cpufreq_table cpufreq_table[CPUFREQ_LEVEL_END];
static struct cpufreq_frequency_table cpufreq_freq_table[CPUFREQ_LEVEL_END + 1];
static struct cpufreq_freqs freqs;

/*
 * available_cpufreq_table[] is composed of cpufreq level index
 * Select one of the tables by reading chip ID and make cpufreq_table
 */
static unsigned int available_cpufreq_table[][CPUFREQ_LEVEL_END + 1] = {
	{ CPU_L2, CPU_L3, CPU_L4, CPU_L5, CPUFREQ_LEVEL_END },
	{ CPU_L1, CPU_L2, CPU_L3, CPU_L4, CPU_L5, CPUFREQ_LEVEL_END },
	{ CPU_L0, CPU_L1, CPU_L2, CPU_L3, CPU_L4, CPU_L5, CPUFREQ_LEVEL_END },
};

/*
 * available_cpufreq_level[] shows
 * support for cpu frequency at each cpufreq level
 */
static struct cpufreq_frequency_table available_cpufreq_level[] = {
	{CPU_L0, 1400*1000},
	{CPU_L1, 1200*1000},
	{CPU_L2, 1000*1000},
	{CPU_L3, 800*1000},
	{CPU_L4, 500*1000},
	{CPU_L5, 200*1000},
	{0, CPUFREQ_TABLE_END},
};

static unsigned int clkdiv_cpu0[][7] = {
	/*
	 * Clock divider value for following
	 * { DIVCORE, DIVCOREM0, DIVCOREM1, DIVPERIPH,
	 *		DIVATB, DIVPCLK_DBG, DIVAPLL }
	 */
	/* ARM L0: 1400MHz */
	{ 0, 3, 7, 3, 4, 1, 7 },

	/* ARM L1: 1200MHz */
	{ 0, 3, 7, 3, 4, 1, 7 },

	/* ARM L2: 1000MHz */
	{ 0, 3, 7, 3, 4, 1, 7 },

	/* ARM L3: 800MHz */
	{ 0, 3, 7, 3, 3, 1, 7 },

	/* ARM L4: 500MHz */
	{ 0, 3, 7, 3, 3, 1, 7 },

	/* ARM L5: 200MHz */
	{ 0, 1, 3, 1, 3, 1, 7 },
};

static unsigned int clkdiv_cpu1[][2] = {
	/* Clock divider value for following
	 * { DIVCOPY, DIVHPM }
	 */
	/* ARM L0: 1400MHz */
	{ 5, 0 },

	/* ARM L1: 1200MHz */
	{ 5, 0 },

	/* ARM L2: 1000MHz */
	{ 4, 0 },

	/* ARM L3: 800MHz */
	{ 3, 0 },

	/* ARM L4: 500MHz */
	{ 3, 0 },

	/* ARM L5: 200MHz */
	{ 3, 0 },
};

static unsigned int apll_pms_value[] = {
	/* APLL FOUT L0: 1400MHz */
	((350<<16)|(6<<8)|(0x1)),

	/* APLL FOUT L1: 1200MHz */
	((150<<16)|(3<<8)|(0x1)),

	/* APLL FOUT L2: 1000MHz */
	((250<<16)|(6<<8)|(0x1)),

	/* APLL FOUT L3: 800MHz */
	((200<<16)|(6<<8)|(0x1)),

	/* APLL FOUT L4: 500MHz */
	((250<<16)|(6<<8)|(0x2)),

	/* APLL FOUT L5: 200MHz */
	((200<<16)|(6<<8)|(0x3)),
};

/* This armvolt_table support until max armclk 1.2GHz */
static unsigned int asv_armvolt_table[][CPUFREQ_LEVEL_END] = {
	/* CPU_L0,  CPU_L1,  CPU_L2,  CPU_L3,  CPU_L4, CPU_L5  */
	{ 0,	1350000, 1300000, 1200000, 1100000, 1050000 }, /* SS */
	{ 0,	1350000, 1250000, 1150000, 1050000, 1000000 }, /* A1 */
	{ 0,	1300000, 1200000, 1100000, 1000000, 975000 }, /* A2 */
	{ 0,	1275000, 1175000, 1075000, 975000, 950000 }, /* B1 */
	{ 0,	1250000, 1150000, 1050000, 975000, 950000 }, /* B2 */
	{ 0,	1225000, 1125000, 1025000, 950000, 925000 }, /* C1 */
	{ 0,	1200000, 1100000, 1000000, 925000, 925000 }, /* C2 */
	{ 0,	1175000, 1075000, 975000, 925000, 925000 }, /* D1 */
};

/* This armvolt_table_1400 support only max armclk 1.4GHz */
static unsigned int asv_armvolt_table_1400[][CPUFREQ_LEVEL_END] = {
	/* CPU_L0,  CPU_L1,  CPU_L2,  CPU_L3,  CPU_L4,	CPU_L5  */
	{ 1350000, 1325000, 1225000, 1150000, 1050000, 1025000 }, /* SS */
	{ 1350000, 1275000, 1175000, 1100000, 1000000, 975000 }, /* A */
	{ 1300000, 1225000, 1125000, 1050000, 950000, 950000 }, /* B */
	{ 1250000, 1175000, 1075000, 1000000, 950000, 950000 }, /* C */
	{ 1225000, 1150000, 1050000, 975000, 950000, 950000 }, /* D */
};

#ifdef CONFIG_ARM_OVERCLK_TO_1400
/* Temporarily support over clock to 1.4GHz only for development */
static unsigned int asv_armvolt_table_1400_overclk[][CPUFREQ_LEVEL_END] = {
	/* CPU_L0,  CPU_L1,  CPU_L2,  CPU_L3,  CPU_L4,	CPU_L5  */
	{ 1475000, 1400000, 1300000, 1200000, 10100000, 1050000 }, /* SS */
	{ 1425000, 1300000, 1200000, 1125000, 1025000, 975000 }, /* A */
	{ 1375000, 1250000, 1150000, 1075000, 975000, 950000 }, /* B */
	{ 1350000, 1200000, 1100000, 1025000, 975000, 950000 }, /* C */
	{ 1300000, 1175000, 1075000, 1000000, 975000, 950000 }, /* D */
};
#endif

#define NORMAL			(0)
#define APLL_ONLY_S		(1)
#define VOLT_UP_ARM800		(2)
static unsigned int freq_trans_table[][CPUFREQ_LEVEL_END] = {
	/* This indicates what to do when cpufreq is changed.
	 * i.e. s-value change in apll changing.
	 *      arm voltage up in freq changing btn 500MHz and 200MHz.
	 * The line & column of below array means new & old frequency.
	 * the conents of array means types to do when frequency is changed.
	 *  @type 0 (NORMAL): normal frequency & voltage change.
	 *  @type 1 (APLL_ONLY_S):  change only s-value in apll for cpufreq
	 *  @type 2 (VOLT_UP_ARM800): change frequecy btn 500MMhz & 200MHz,
	 *    and temporaily set to arm voltage @ 800MHz
	 *
	 * (for example)
	 * from\to  (1400/1200/1000/800/500/200 (new_index)
	 * 1400
	 * 1200
	 * 1000
	 *  800
	 *  500
	 *  200
	 * (old_index)
	*/
	{ 0, 0, 0, 0, 0, 0 },
	{ 0, 0, 0, 0, 0, 0 },
	{ 0, 0, 0, 0, 1, 0 },
	{ 0, 0, 0, 0, 0, 1 },
	{ 0, 0, 1, 0, 0, 2 },
	{ 0, 0, 0, 1, 2, 0 },
};

/*********************************************************************
 *                     BUSFREQ DEFINITIONS                           *
 *********************************************************************/

#define SYSFS_DEBUG_BUSFREQ

#define MAX_LOAD	100
#define LIMIT_BUS_LOAD	(MAX_LOAD / 2)

#define UP_THRESHOLD_DEFAULT	27
#define CPU_PMU_L0_THRESHOLD	10
#define CPU_PMU_L1_THRESHOLD	5

static unsigned int up_threshold;
static struct s5pv310_dmc_ppmu_hw dmc[2];
static struct s5pv310_cpu_ppmu_hw cpu;
static unsigned int bus_utilization[2];

static unsigned int busfreq_fix;
static unsigned int fix_busfreq_level;

static unsigned int calc_bus_utilization(struct s5pv310_dmc_ppmu_hw *ppmu);
static void busfreq_target(void);

static DEFINE_MUTEX(set_bus_freq_change);
static DEFINE_MUTEX(set_bus_freq_lock);

#ifdef SYSFS_DEBUG_BUSFREQ
static unsigned int time_in_state[BUSFREQ_LEVEL_END];
static unsigned long pre_jiffies;
static unsigned long cur_jiffies;
#endif

static unsigned int cur_busfreq_index;

struct busfreq_table {
	unsigned int index;
	unsigned int mem_clk;
	unsigned int volt;
};

static struct busfreq_table s5pv310_busfreq_table[] = {
	{BUS_L0, 400000, 1100000},
	{BUS_L1, 267000, 1000000},
	{BUS_L2, 160000, 1000000},
	{0, 0, 0},
};

static unsigned int clkdiv_dmc0[BUSFREQ_LEVEL_END][8] = {
	/*
	 * Clock divider value for following
	 * { DIVACP, DIVACP_PCLK, DIVDPHY, DIVDMC, DIVDMCD
	 *		DIVDMCP, DIVCOPY2, DIVCORE_TIMERS }
	 */

	/* DMC L0: 400MHz */
	{ 3, 2, 1, 1, 1, 1, 3, 1 },

	/* DMC L1: 266.7MHz */
	{ 4, 2, 1, 2, 1, 1, 3, 1 },

	/* DMC L2: 160MHz */
	{ 5, 2, 1, 4, 1, 1, 3, 1 },
};


static unsigned int clkdiv_top[BUSFREQ_LEVEL_END][5] = {
	/*
	 * Clock divider value for following
	 * { DIVACLK200, DIVACLK100, DIVACLK160, DIVACLK133, DIVONENAND }
	 */

	/* ACLK200 L1: 200MHz */
	{ 3, 7, 4, 5, 1 },

	/* ACLK200 L2: 160MHz */
	{ 4, 7, 5, 6, 1 },

	/* ACLK200 L3: 133MHz */
	{ 5, 7, 7, 7, 1 },
};

static unsigned int clkdiv_lr_bus[BUSFREQ_LEVEL_END][2] = {
	/*
	 * Clock divider value for following
	 * { DIVGDL/R, DIVGPL/R }
	 */

	/* ACLK_GDL/R L1: 200MHz */
	{ 3, 1 },

	/* ACLK_GDL/R L2: 160MHz */
	{ 4, 1 },

	/* ACLK_GDL/R L3: 133MHz */
	{ 5, 1 },
};

static unsigned int clkdiv_ip_bus[BUSFREQ_LEVEL_END][3] = {
	/*
	 * Clock divider value for following
	 * { DIV_MFC, DIV_G2D, DIV_FIMC }
	 */

	/* L0: MFC 200MHz G2D 266MHz FIMC 160MHz */
	{ 3, 2, 4 },

	/* L1: MFC 200MHz G2D 160MHz FIMC 133MHz */
	{ 3, 4, 5 },

	/* L2: MFC 200MHz G2D 133MHz FIMC 100MHz */
	{ 3, 5, 7 },
};

#ifdef CONFIG_S5PV310_ASV

enum asv_group_8_index {
	GR_SS, GR_A1, GR_A2, GR_B1, GR_B2, GR_C1, GR_C2, GR_D1, ASV_8_GROUP_END,
};

enum asv_group_5_index {
	GR_S, GR_A, GR_B, GR_C, GR_D, ASV_5_GROUP_END,
};

/* level 1 and 2 of vdd_int uses the same voltage value in U1 project */
static unsigned int asv_int_volt_8_table[ASV_8_GROUP_END][BUSFREQ_LEVEL_END] = {
	{1150000, 1050000, 1050000},	/* SS */
	{1125000, 1025000, 1025000},	/* A1 */
	{1125000, 1025000, 1025000},	/* A2 */
	{1100000, 1000000, 1000000},	/* B1 */
	{1100000, 1000000, 1000000},	/* B2 */
	{1075000,  975000,  975000},	/* C1 */
	{1075000,  975000,  975000},	/* C1 */
	{1050000,  950000,  950000},	/* D1 */
};

static unsigned int asv_int_volt_5_table[ASV_5_GROUP_END][BUSFREQ_LEVEL_END] = {
	{1150000, 1050000, 1050000},	/* SS */
	{1125000, 1025000, 1025000},	/* A */
	{1100000, 1000000, 1000000},	/* B */
	{1075000,  975000,  975000},	/* C */
	{1050000,  950000,  950000},	/* D */
};

static unsigned int asv_8_threshold[][ASV_8_GROUP_END - 1] = {
	/* SS, A1, A2, B1, B2, C1, C2, D1 */
	{4,  8, 12, 17, 27, 45, 55},	/* IDS */
	{8, 11, 14, 18, 21, 23, 25},	/* HPM */
};

static unsigned int asv_5_threshold[][ASV_5_GROUP_END - 1] = {
	/* S, A, B, C, D */
	{ 8, 15, 37, 52},	/* IDS */
	{13, 17, 22, 26},	/* HPM */
};
#endif

int s5pv310_verify_policy(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, cpufreq_freq_table);
}

unsigned int s5pv310_getspeed(unsigned int cpu)
{
	unsigned long rate;

	rate = clk_get_rate(arm_clk) / 1000;

	return rate;
}

void s5pv310_set_busfreq(unsigned int index)
{
	unsigned int tmp, val, volt;

	if (cur_busfreq_index == index)
		return;

#if defined(CONFIG_REGULATOR)
	volt = s5pv310_busfreq_table[index].volt;

	if (cur_busfreq_index > index)
		regulator_set_voltage(int_regulator, volt, volt);
#endif

	/* Change Divider - DMC0 */
	tmp = __raw_readl(S5P_CLKDIV_DMC0);

	tmp &= ~(S5P_CLKDIV_DMC0_ACP_MASK | S5P_CLKDIV_DMC0_ACPPCLK_MASK |
		S5P_CLKDIV_DMC0_DPHY_MASK | S5P_CLKDIV_DMC0_DMC_MASK |
		S5P_CLKDIV_DMC0_DMCD_MASK | S5P_CLKDIV_DMC0_DMCP_MASK |
		S5P_CLKDIV_DMC0_COPY2_MASK | S5P_CLKDIV_DMC0_CORETI_MASK);

	tmp |= ((clkdiv_dmc0[index][0] << S5P_CLKDIV_DMC0_ACP_SHIFT) |
		(clkdiv_dmc0[index][1] << S5P_CLKDIV_DMC0_ACPPCLK_SHIFT) |
		(clkdiv_dmc0[index][2] << S5P_CLKDIV_DMC0_DPHY_SHIFT) |
		(clkdiv_dmc0[index][3] << S5P_CLKDIV_DMC0_DMC_SHIFT) |
		(clkdiv_dmc0[index][4] << S5P_CLKDIV_DMC0_DMCD_SHIFT) |
		(clkdiv_dmc0[index][5] << S5P_CLKDIV_DMC0_DMCP_SHIFT) |
		(clkdiv_dmc0[index][6] << S5P_CLKDIV_DMC0_COPY2_SHIFT) |
		(clkdiv_dmc0[index][7] << S5P_CLKDIV_DMC0_CORETI_SHIFT));

	__raw_writel(tmp, S5P_CLKDIV_DMC0);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_DMC0);
	} while (tmp & 0x11111111);


	/* Change Divider - TOP */
	tmp = __raw_readl(S5P_CLKDIV_TOP);

	tmp &= ~(S5P_CLKDIV_TOP_ACLK200_MASK | S5P_CLKDIV_TOP_ACLK100_MASK |
		S5P_CLKDIV_TOP_ACLK160_MASK | S5P_CLKDIV_TOP_ACLK133_MASK |
		S5P_CLKDIV_TOP_ONENAND_MASK);

	tmp |= ((clkdiv_top[index][0] << S5P_CLKDIV_TOP_ACLK200_SHIFT) |
		(clkdiv_top[index][1] << S5P_CLKDIV_TOP_ACLK100_SHIFT) |
		(clkdiv_top[index][2] << S5P_CLKDIV_TOP_ACLK160_SHIFT) |
		(clkdiv_top[index][3] << S5P_CLKDIV_TOP_ACLK133_SHIFT) |
		(clkdiv_top[index][4] << S5P_CLKDIV_TOP_ONENAND_SHIFT));

	__raw_writel(tmp, S5P_CLKDIV_TOP);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_TOP);
	} while (tmp & 0x11111);

	/* Change Divider - LEFTBUS */
	tmp = __raw_readl(S5P_CLKDIV_LEFTBUS);

	tmp &= ~(S5P_CLKDIV_BUS_GDLR_MASK | S5P_CLKDIV_BUS_GPLR_MASK);

	tmp |= ((clkdiv_lr_bus[index][0] << S5P_CLKDIV_BUS_GDLR_SHIFT) |
		(clkdiv_lr_bus[index][1] << S5P_CLKDIV_BUS_GPLR_SHIFT));

	__raw_writel(tmp, S5P_CLKDIV_LEFTBUS);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_LEFTBUS);
	} while (tmp & 0x11);

	/* Change Divider - RIGHTBUS */
	tmp = __raw_readl(S5P_CLKDIV_RIGHTBUS);

	tmp &= ~(S5P_CLKDIV_BUS_GDLR_MASK | S5P_CLKDIV_BUS_GPLR_MASK);

	tmp |= ((clkdiv_lr_bus[index][0] << S5P_CLKDIV_BUS_GDLR_SHIFT) |
		(clkdiv_lr_bus[index][1] << S5P_CLKDIV_BUS_GPLR_SHIFT));

	__raw_writel(tmp, S5P_CLKDIV_RIGHTBUS);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_RIGHTBUS);
	} while (tmp & 0x11);

	/* Change Divider - SCLK_MFC */
	tmp = __raw_readl(S5P_CLKDIV_MFC);

	tmp &= ~S5P_CLKDIV_MFC_MASK;

	tmp |= (clkdiv_ip_bus[index][0] << S5P_CLKDIV_MFC_SHIFT);

	__raw_writel(tmp, S5P_CLKDIV_MFC);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_MFC);
	} while (tmp & 0x1);

	/* Change Divider - SCLK_G2D */
	tmp = __raw_readl(S5P_CLKDIV_IMAGE);

	tmp &= ~S5P_CLKDIV_IMAGE_MASK;

	tmp |= (clkdiv_ip_bus[index][1] << S5P_CLKDIV_IMAGE_SHIFT);

	__raw_writel(tmp, S5P_CLKDIV_IMAGE);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_IMAGE);
	} while (tmp & 0x1);

	/* Change Divider - SCLK_FIMC */
	tmp = __raw_readl(S5P_CLKDIV_CAM);

	tmp &= ~S5P_CLKDIV_CAM_MASK;

	val = clkdiv_ip_bus[index][2];
	tmp |= ((val << 0) | (val << 4) | (val << 8) | (val << 12));

	__raw_writel(tmp, S5P_CLKDIV_CAM);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STAT_CAM);
	} while (tmp & 0x1111);

#if defined(CONFIG_REGULATOR)
	if (cur_busfreq_index < index)
		regulator_set_voltage(int_regulator, volt, volt);
#endif

	cur_busfreq_index = index;
}

void s5pv310_set_clkdiv(unsigned int div_index)
{
	unsigned int tmp;

	/* Change Divider - CPU0 */
	tmp = cpufreq_table[div_index].clkdiv_cpu0;

	__raw_writel(tmp, S5P_CLKDIV_CPU);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STATCPU);
	} while (tmp & 0x1111111);

	/* Change Divider - CPU1 */
	tmp = cpufreq_table[div_index].clkdiv_cpu1;

	__raw_writel(tmp, S5P_CLKDIV_CPU1);

	do {
		tmp = __raw_readl(S5P_CLKDIV_STATCPU1);
	} while (tmp & 0x11);

}

void s5pv310_set_apll(unsigned int index)
{
	unsigned int tmp;
	unsigned int save_val;

	/* 1. MUX_CORE_SEL = MPLL,
	 * Reduce the CLKDIVCPU value for using MPLL */
	save_val = __raw_readl(S5P_CLKDIV_CPU);
	tmp = save_val;
	tmp &= ~S5P_CLKDIV_CPU0_CORE_MASK;
	tmp |= (((save_val & 0xf) + 1) << S5P_CLKDIV_CPU0_CORE_SHIFT);
	__raw_writel(tmp, S5P_CLKDIV_CPU);
	do {
		tmp = __raw_readl(S5P_CLKDIV_STATCPU);
	} while (tmp & 0x1);

	/* ARMCLK uses MPLL for lock time */
	clk_set_parent(moutcore, mout_mpll);

	do {
		tmp = (__raw_readl(S5P_CLKMUX_STATCPU)
			>> S5P_CLKSRC_CPU_MUXCORE_SHIFT);
		tmp &= 0x7;
	} while (tmp != 0x2);

	/* 2. Set APLL Lock time */
	__raw_writel(S5P_APLL_LOCKTIME, S5P_APLL_LOCK);

	/* 3. Change PLL PMS values */
	tmp = __raw_readl(S5P_APLL_CON0);
	tmp &= ~((0x3ff << 16) | (0x3f << 8) | (0x7 << 0));
	tmp |= cpufreq_table[index].apll_pms;
	__raw_writel(tmp, S5P_APLL_CON0);

	/* 4. wait_lock_time */
	do {
		tmp = __raw_readl(S5P_APLL_CON0);
	} while (!(tmp & (0x1 << S5P_APLLCON0_LOCKED_SHIFT)));

	/* 5. MUX_CORE_SEL = APLL */
	clk_set_parent(moutcore, mout_apll);

	do {
		tmp = __raw_readl(S5P_CLKMUX_STATCPU);
		tmp &= S5P_CLKMUX_STATCPU_MUXCORE_MASK;
	} while (tmp != (0x1 << S5P_CLKSRC_CPU_MUXCORE_SHIFT));

	/* Restore the CLKDIVCPU value for APLL */
	__raw_writel(save_val, S5P_CLKDIV_CPU);
	do {
		tmp = __raw_readl(S5P_CLKDIV_STATCPU);
	} while (tmp & 0x1);

}

void s5pv310_set_frequency(unsigned int old_index, unsigned int new_index)
{
	unsigned int tmp;
	unsigned int is_curfreq_table = 0;

	if (freqs.old == cpufreq_freq_table[old_index].frequency)
		is_curfreq_table = 1;

	if (freqs.old < freqs.new) {
		/* 500->1000 & 200->800 change require to only change s value */
		if (is_curfreq_table &&
			(cpufreq_table[old_index].freq_trans[new_index] &
			APLL_ONLY_S)) {
			/* 1. Change the system clock divider values */
			s5pv310_set_clkdiv(new_index);

			/* 2. Change just s value in apll m,p,s value */
			tmp = __raw_readl(S5P_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (cpufreq_table[new_index].apll_pms & 0x7);
			__raw_writel(tmp, S5P_APLL_CON0);

		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the system clock divider values */
			s5pv310_set_clkdiv(new_index);

			/* 2. Change the apll m,p,s value */
			s5pv310_set_apll(new_index);
		}
	} else if (freqs.old > freqs.new) {
		/* 1000->500 & 800->200 change require to only change s value */
		if (is_curfreq_table &&
			(cpufreq_table[old_index].freq_trans[new_index] &
			APLL_ONLY_S)) {
			/* 1. Change just s value in apll m,p,s value */
			tmp = __raw_readl(S5P_APLL_CON0);
			tmp &= ~(0x7 << 0);
			tmp |= (cpufreq_table[new_index].apll_pms & 0x7);
			__raw_writel(tmp, S5P_APLL_CON0);

			/* 2. Change the system clock divider values */
			s5pv310_set_clkdiv(new_index);
		} else {
			/* Clock Configuration Procedure */
			/* 1. Change the apll m,p,s value */
			s5pv310_set_apll(new_index);

			/* 2. Change the system clock divider values */
			s5pv310_set_clkdiv(new_index);
		}
	}
}

#define VOLT_PRECHANGE		(1)
#define VOLT_POSTCHANGE		(2)
void s5pv310_set_cpufreq_armvolt(unsigned int old_index, unsigned int index)
{
	unsigned int pre_volt = 0, post_volt = 0;
	unsigned int freq_trans_val, volt_change = 0;

	freq_trans_val = cpufreq_table[old_index].freq_trans[index];

	if (freqs.new > freqs.old) {
		volt_change = VOLT_PRECHANGE;
		pre_volt = cpufreq_table[index].arm_volt;
		if (freq_trans_val & VOLT_UP_ARM800) {
			volt_change |= VOLT_POSTCHANGE;
			pre_volt = cpufreq_table[index - 1].arm_volt;
			post_volt = cpufreq_table[index].arm_volt;
		}
	} else {
		volt_change = VOLT_POSTCHANGE;
		post_volt = cpufreq_table[index].arm_volt;
		if (freq_trans_val & VOLT_UP_ARM800) {
			volt_change |= VOLT_PRECHANGE;
			pre_volt = cpufreq_table[index - 2].arm_volt;
			post_volt = cpufreq_table[index].arm_volt;
		}
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	/* When the new frequency is higher than current frequency
	 * and freqency is changed btn 500MHz to 200MHz
	 */
#if defined(CONFIG_REGULATOR)
	if (volt_change & VOLT_PRECHANGE)
		regulator_set_voltage(arm_regulator, pre_volt, pre_volt);
#endif
	s5pv310_set_frequency(old_index, index);

	/* When the new frequency is lower than current frequency
	 * and freqency is increased from 200MHz to 500MHz
	*/
#if defined(CONFIG_REGULATOR)
	if (volt_change & VOLT_POSTCHANGE)
		regulator_set_voltage(arm_regulator, post_volt, post_volt);
#endif
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
}

static int s5pv310_target(struct cpufreq_policy *policy,
		unsigned int target_freq,
		unsigned int relation)
{
	int ret = 0;
	unsigned int index, old_index;

	mutex_lock(&set_cpu_freq_change);
#if defined(CONFIG_REGULATOR)
	/* Do not set voltage during disable_further_cpufreq */
	if (disable_further_cpufreq)
		goto cpufreq_out;
#endif
	freqs.old = s5pv310_getspeed(policy->cpu);
	if (cpufreq_frequency_table_target(policy, cpufreq_freq_table,
				freqs.old, relation, &old_index)) {
		ret = -EINVAL;
		goto cpufreq_out;
	}

	if (cpufreq_frequency_table_target(policy, cpufreq_freq_table,
				target_freq, relation, &index)) {
		ret = -EINVAL;
		goto cpufreq_out;
	}

	if (!cpufreq_lock.disable_lock && (index > cpufreq_lock.level))
		index = cpufreq_lock.level;

	if (!cpufreq_upper_lock.disable_lock &&
				(index < cpufreq_upper_lock.level))
		index = cpufreq_upper_lock.level;

	/* WORKAROUND
	 * Do NOT step up max arm clock directly to reduce power consumption */
	if (cpufreq_info.max_arm_clk == CPUFREQ_1400MHZ) {
		if ((index == 0) && (old_index > 3))
			index = 3;
	} else {
		if ((index == 0) && (old_index > 2))
			index = 2;
	}

set_cpufreq:
	freqs.new = cpufreq_freq_table[index].frequency;
	freqs.cpu = policy->cpu;

	/* If the new frequency is same with previous frequency, skip */
	if (freqs.new == freqs.old)
		goto bus_freq;

	s5pv310_set_cpufreq_armvolt(old_index, index);

bus_freq:
	mutex_unlock(&set_cpu_freq_change);
	busfreq_target();
	return ret;

cpufreq_out:
	mutex_unlock(&set_cpu_freq_change);
	return ret;
}

static void busfreq_ppmu_init(void)
{
	unsigned int i;

	for (i = 0; i < DMC_NUM; i++) {
		s5pv310_dmc_ppmu_reset(&dmc[i]);
		s5pv310_dmc_ppmu_setevent(&dmc[i], DMC_READWRITE_DATA_EVT);
		s5pv310_dmc_ppmu_start(&dmc[i]);
	}
}

static void cpu_ppmu_init(void)
{
	s5pv310_cpu_ppmu_reset(&cpu);
	s5pv310_cpu_ppmu_setevent(&cpu, CPU_READ_DATA_EVT, 0);
	s5pv310_cpu_ppmu_setevent(&cpu, CPU_WRITE_DATA_EVT, 1);
	s5pv310_cpu_ppmu_start(&cpu);
}

static unsigned int calc_bus_utilization(struct s5pv310_dmc_ppmu_hw *ppmu)
{
	unsigned int bus_usage;

	if (ppmu->ccnt == 0) {
		printk(KERN_DEBUG "%s: 0 value is not permitted\n", __func__);
		return MAX_LOAD;
	}

	if (!(ppmu->ccnt >> 7))
		bus_usage = (ppmu->count * 100) / ppmu->ccnt;
	else
		bus_usage = ((ppmu->count >> 7) * 100) / (ppmu->ccnt >> 7);

	return bus_usage;
}

static unsigned int calc_cpu_bus_utilization(struct s5pv310_cpu_ppmu_hw *cpu_ppmu)
{
	unsigned int cpu_bus_usage;

	if (cpu.ccnt == 0)
		cpu.ccnt = MAX_LOAD;

	cpu_bus_usage = (cpu.count[0] + cpu.count[1])*100 / cpu.ccnt;

	return cpu_bus_usage;
}

static unsigned int get_cpu_ppmu_load(void)
{
	unsigned int cpu_bus_load;

	s5pv310_cpu_ppmu_stop(&cpu);
	s5pv310_cpu_ppmu_update(&cpu);

	cpu_bus_load = calc_cpu_bus_utilization(&cpu);

	return cpu_bus_load;
}

static unsigned int get_ppc_load(void)
{
	int i;
	unsigned int bus_load;

	for (i = 0; i < DMC_NUM; i++) {
		s5pv310_dmc_ppmu_stop(&dmc[i]);
		s5pv310_dmc_ppmu_update(&dmc[i]);
		bus_utilization[i] = calc_bus_utilization(&dmc[i]);
	}

	bus_load = max(bus_utilization[0], bus_utilization[1]);

	return bus_load;
}

#ifdef SYSFS_DEBUG_BUSFREQ
static void update_busfreq_stat(void)
{
	unsigned long level_state_jiffies;

	cur_jiffies = jiffies;
	if (pre_jiffies != 0)
		level_state_jiffies = cur_jiffies - pre_jiffies;
	else
		level_state_jiffies = 0;

	pre_jiffies = jiffies;

	switch (cur_busfreq_index) {
	case BUS_L0:
		time_in_state[BUS_L0] += level_state_jiffies;
		break;
	case BUS_L1:
		time_in_state[BUS_L1] += level_state_jiffies;
		break;
	case BUS_L2:
		time_in_state[BUS_L2] += level_state_jiffies;
		break;
	default:
		break;
	}
}
#endif


static void select_busfreq(unsigned int bus_load,
			unsigned int cpu_bus_load,
			enum busfreq_level_index *target_index)
{
	unsigned int i, target_freq, index = 0;

	/*
	* Maximum bus_load of S5PV310 is about 50%.
	* Default Up Threshold is 27%.
	*/
	if (bus_load > LIMIT_BUS_LOAD) {
		printk(KERN_DEBUG "BUSLOAD is larger than 50(%d)\n", bus_load);
		bus_load = LIMIT_BUS_LOAD;
	}

	target_freq = (s5pv310_busfreq_table[cur_busfreq_index].mem_clk
			* bus_load) / up_threshold;

	for (i = 1; i <= BUSFREQ_LEVEL_END; i++) {
		if (target_freq >= s5pv310_busfreq_table[i].mem_clk) {
			index = i - 1;
			break;
		}
	}

	*target_index = s5pv310_busfreq_table[index].index;

	if ((*target_index > BUS_L1) && (cpu_bus_load > CPU_PMU_L1_THRESHOLD))
		*target_index = BUS_L1;

	if (*target_index > busfreq_lock.level)
		*target_index = busfreq_lock.level;

	return;
}

static void busfreq_target(void)
{
	unsigned int bus_load, cpu_bus_load;
	unsigned int target_index = 0;

	mutex_lock(&set_bus_freq_change);

	if (busfreq_fix)
		goto fix_out;

#ifdef SYSFS_DEBUG_BUSFREQ
	update_busfreq_stat();
#endif

	/*
	 * Get the CPU PPMU load value which compare for
	 *  whether need to change
	 */
	cpu_bus_load = get_cpu_ppmu_load();

	/*
	 * For Fast response time, if cpu bus load is heavy,
	 * Bus frequency is up right now
	 */
	if (cpu_bus_load > CPU_PMU_L0_THRESHOLD) {
		s5pv310_set_busfreq(BUS_L0);
		goto out;
	}

	/*
	 * Get the PPC load value which compare for
	 * whether need to change
	 */
	bus_load = get_ppc_load();

	/* Change bus frequency */
	select_busfreq(bus_load, cpu_bus_load, &target_index);

	s5pv310_set_busfreq(target_index);

out:
	busfreq_ppmu_init();
	cpu_ppmu_init();
fix_out:
	mutex_unlock(&set_bus_freq_change);
}


int s5pv310_busfreq_lock(unsigned int nId,
			enum busfreq_level_index req_lock_level)
{
	if (!cpufreq_info.init_done || req_lock_level < 0)
		return 0;

	if (busfreq_lock.dev_id & (1 << nId)) {
		printk(KERN_ERR
		"[BUSFREQ] This device [%d] already locked busfreq\n", nId);
		return 0;
	}
	mutex_lock(&set_bus_freq_lock);
	busfreq_lock.dev_id |= (1 << nId);
	busfreq_lock.value[nId] = req_lock_level;

	/* If the requested cpufreq is higher than current min frequency */
	if (req_lock_level < busfreq_lock.level) {
		busfreq_lock.level = req_lock_level;
		mutex_unlock(&set_bus_freq_lock);
		busfreq_target();
	} else
		mutex_unlock(&set_bus_freq_lock);

	return 0;
}

void s5pv310_busfreq_lock_free(unsigned int nId)
{
	unsigned int i;

	mutex_lock(&set_bus_freq_lock);

	busfreq_lock.dev_id &= ~(1 << nId);
	busfreq_lock.value[nId] = busfreq_lock.min_level;
	busfreq_lock.level = busfreq_lock.min_level;

	if (busfreq_lock.dev_id) {
		for (i = 0; i < DVFS_LOCK_ID_END; i++) {
			if (busfreq_lock.value[i] < busfreq_lock.level)
				busfreq_lock.level = busfreq_lock.value[i];
		}
	}

	mutex_unlock(&set_bus_freq_lock);
}

int s5pv310_cpufreq_round_idx(unsigned int cpu_freq)
{
	unsigned int i;
	unsigned int cpu_idx = 0;

	if (!cpufreq_info.init_done)
		return -EINVAL;

	/* verify with cpufreq limits */
	if ((cpu_freq > available_cpufreq_level[0].frequency) ||
	(cpu_freq < available_cpufreq_level[CPUFREQ_LEVEL_END - 1].frequency))
		printk(KERN_ERR "This freq is out of limit (%d ~ %d KHz)\n",
		available_cpufreq_level[0].frequency,
		available_cpufreq_level[CPUFREQ_LEVEL_END - 1].frequency);

	for (i = 0; cpufreq_freq_table[i].frequency !=
				CPUFREQ_TABLE_END; i++) {
		if (cpufreq_freq_table[i].frequency >= cpu_freq)
			cpu_idx = i;
		else
			break;
	}

	return cpufreq_freq_table[cpu_idx].index;
}

int s5pv310_cpufreq_lock(unsigned int nId,
			enum cpufreq_level_index req_lock_level)
{
	unsigned int i, cur_idx = 0;
	unsigned int cur_freq, req_freq;

	if (!cpufreq_info.init_done || req_lock_level < 0)
		return 0;

	if (cpufreq_lock.dev_id & (1 << nId)) {
		printk(KERN_ERR
		"[CPUFREQ]This device [%d] already locked cpufreq\n", nId);
		return 0;
	}

	if (req_lock_level > cpufreq_lock.min_level) {
		printk(KERN_ERR
		"[CPUFREQ] This is wrong cpufreq_level %d (min level is %d)\n",
			req_lock_level, cpufreq_lock.min_level);
		req_lock_level = cpufreq_lock.min_level;
	}

	mutex_lock(&set_cpu_freq_lock);
	cpufreq_lock.dev_id |= (1 << nId);
	cpufreq_lock.value[nId] = req_lock_level;

	/* If the requested cpufreq is higher than current min frequency */
	if (req_lock_level < cpufreq_lock.level)
		cpufreq_lock.level = req_lock_level;

	mutex_unlock(&set_cpu_freq_lock);

	/* If the lock level is higher than upper lock
	 * do not setting cpufreq lock frequency
	 * because the priority of upper lock is the highest except ID_PM */
	if ((cpufreq_lock.level < cpufreq_upper_lock.level)
					&& (nId != DVFS_LOCK_ID_PM))
		return 0;

	/* Do not setting cpufreq lock frequency
	 * because current governor doesn't support dvfs level lock
	 * except DVFS_LOCK_ID_PM */
	if (cpufreq_lock.disable_lock && (nId != DVFS_LOCK_ID_PM))
		return 0;

	/* If current frequency is lower than requested freq,
	 * it needs to update
	 */
	mutex_lock(&set_cpu_freq_change);
	cur_freq = s5pv310_getspeed(0);
	req_freq = cpufreq_freq_table[req_lock_level].frequency;
	if (cur_freq < req_freq) {
		/* Find out current level index */
		for (i = 0; cpufreq_freq_table[i].frequency
				!= CPUFREQ_TABLE_END; i++) {
			if (cur_freq == cpufreq_freq_table[i].frequency) {
				cur_idx = cpufreq_freq_table[i].index;
				break;
			} else if (i == (CPUFREQ_LEVEL_END - 1)) {
				printk(KERN_ERR "%s: Level not found\n",
					__func__);
				mutex_unlock(&set_cpu_freq_change);
				return -EINVAL;
			} else {
				continue;
			}
		}
		freqs.old = cur_freq;
		freqs.new = req_freq;
		s5pv310_set_cpufreq_armvolt(cur_idx, req_lock_level);

	}
	mutex_unlock(&set_cpu_freq_change);

	return 0;
}

void s5pv310_cpufreq_lock_free(unsigned int nId)
{
	unsigned int i;

	if (!cpufreq_info.init_done)
		return;

	mutex_lock(&set_cpu_freq_lock);

	cpufreq_lock.dev_id &= ~(1 << nId);
	cpufreq_lock.value[nId] = cpufreq_lock.min_level;
	cpufreq_lock.level = cpufreq_lock.min_level;
	if (cpufreq_lock.dev_id) {
		for (i = 0; i < DVFS_LOCK_ID_END; i++) {
			if (cpufreq_lock.value[i] < cpufreq_lock.level)
				cpufreq_lock.level = cpufreq_lock.value[i];
		}
	}

	mutex_unlock(&set_cpu_freq_lock);
}

int s5pv310_cpufreq_upper_limit(unsigned int nId,
			enum cpufreq_level_index req_lock_level)
{
	unsigned int i, cur_idx = 0;
	unsigned int cur_freq, req_freq;

	if (!cpufreq_info.init_done || req_lock_level < 0)
		return 0;

	if (cpufreq_upper_lock.dev_id & (1 << nId)) {
		printk(KERN_ERR
		"[CPUFREQ]This device [%d] already upper locked cpufreq\n", nId);
		return 0;
	}

	if (req_lock_level < cpufreq_upper_lock.max_level) {
		printk(KERN_ERR
		"[CPUFREQ] This is wrong cpufreq_level %d (max level is %d)\n",
			req_lock_level, cpufreq_upper_lock.max_level);
		req_lock_level = cpufreq_upper_lock.max_level;
	}

	mutex_lock(&set_cpu_freq_lock);
	cpufreq_upper_lock.dev_id |= (1 << nId);
	cpufreq_upper_lock.value[nId] = req_lock_level;

	/* If the requested limit level is lower than current value */
	if (req_lock_level > cpufreq_upper_lock.level)
		cpufreq_upper_lock.level = req_lock_level;

	mutex_unlock(&set_cpu_freq_lock);

	/* Do not setting cpufreq upper lock frequency
	 * because current governor doesn't support dvfs upper level lock
	 */
	if (cpufreq_upper_lock.disable_lock)
		return 0;

	/* If cur frequency is higher than limit freq, it needs to update */
	mutex_lock(&set_cpu_freq_change);
	cur_freq = s5pv310_getspeed(0);
	req_freq = cpufreq_freq_table[req_lock_level].frequency;
	if (cur_freq > req_freq) {
		/* Find out current level index */
		for (i = 0; cpufreq_freq_table[i].frequency
				!= CPUFREQ_TABLE_END; i++) {
			if (cur_freq == cpufreq_freq_table[i].frequency) {
				cur_idx = cpufreq_freq_table[i].index;
				break;
			} else if (i == (CPUFREQ_LEVEL_END - 1)) {
				printk(KERN_ERR "%s: Level not found\n",
					__func__);
				mutex_unlock(&set_cpu_freq_change);
				return -EINVAL;
			} else {
				continue;
			}
		}
		freqs.old = cur_freq;
		freqs.new = req_freq;
		s5pv310_set_cpufreq_armvolt(cur_idx, req_lock_level);

	}
	mutex_unlock(&set_cpu_freq_change);
	return 0;
}

void s5pv310_cpufreq_upper_limit_free(unsigned int nId)
{
	unsigned int i;

	if (!cpufreq_info.init_done)
		return;

	mutex_lock(&set_cpu_freq_lock);

	cpufreq_upper_lock.dev_id &= ~(1 << nId);
	cpufreq_upper_lock.value[nId] = cpufreq_upper_lock.max_level;
	cpufreq_upper_lock.level = cpufreq_upper_lock.max_level;
	if (cpufreq_upper_lock.dev_id) {
		for (i = 0; i < DVFS_LOCK_ID_END; i++) {
			if (cpufreq_upper_lock.value[i] > cpufreq_upper_lock.level)
				cpufreq_upper_lock.level = cpufreq_upper_lock.value[i];
		}
	}

	mutex_unlock(&set_cpu_freq_lock);
}


#ifdef CONFIG_PM
static int s5pv310_cpufreq_suspend(struct cpufreq_policy *policy,
			pm_message_t pmsg)
{
	return 0;
}

static int s5pv310_cpufreq_resume(struct cpufreq_policy *policy)
{
	return 0;
}
#endif

static int s5pv310_cpufreq_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	int ret = 0;
	static int cpu_lv = -1;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		if (cpu_lv == -1)
			cpu_lv = s5pv310_cpufreq_round_idx(CPUFREQ_800MHZ);
		ret = s5pv310_cpufreq_lock(DVFS_LOCK_ID_PM, cpu_lv);
		if (ret < 0)
			return NOTIFY_BAD;
		ret = s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_PM, cpu_lv);
		if (ret < 0)
			return NOTIFY_BAD;
		s5pv310_busfreq_lock(DVFS_LOCK_ID_PM, BUS_L0);
#if defined(CONFIG_REGULATOR)
		disable_further_cpufreq = true;
#endif
		printk(KERN_DEBUG "PM_SUSPEND_PREPARE for CPUFREQ\n");
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		printk(KERN_DEBUG "PM_POST_SUSPEND for CPUFREQ\n");
		s5pv310_cpufreq_lock_free(DVFS_LOCK_ID_PM);
		s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_PM);
		s5pv310_busfreq_lock_free(DVFS_LOCK_ID_PM);
#if defined(CONFIG_REGULATOR)
		disable_further_cpufreq = false;
#endif
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block s5pv310_cpufreq_notifier = {
	.notifier_call = s5pv310_cpufreq_notifier_event,
};

static int s5pv310_cpufreq_reboot_notifier_call(struct notifier_block *this,
				   unsigned long code, void *_cmd)
{
	int ret = 0;

	ret = s5pv310_cpufreq_lock(DVFS_LOCK_ID_PM, CPU_L0);
	if (ret < 0)
		return NOTIFY_BAD;
	s5pv310_busfreq_lock(DVFS_LOCK_ID_PM, BUS_L0);

	return NOTIFY_DONE;
}

static struct notifier_block s5pv310_cpufreq_reboot_notifier = {
	.notifier_call = s5pv310_cpufreq_reboot_notifier_call,
};

static int s5pv310_cpufreq_policy_notifier_call(struct notifier_block *this,
				unsigned long code, void *data)
{
	struct cpufreq_policy *policy = data;

	switch (code) {
	case CPUFREQ_ADJUST:
		if ((!strnicmp(policy->governor->name, "powersave", CPUFREQ_NAME_LEN))
		|| (!strnicmp(policy->governor->name, "performance", CPUFREQ_NAME_LEN))
		|| (!strnicmp(policy->governor->name, "userspace", CPUFREQ_NAME_LEN))) {
			printk(KERN_DEBUG "cpufreq governor is changed to %s\n",
							policy->governor->name);
			cpufreq_lock.disable_lock = true;
		} else
			cpufreq_lock.disable_lock = false;
	case CPUFREQ_INCOMPATIBLE:
	case CPUFREQ_NOTIFY:
	default:
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block s5pv310_cpufreq_policy_notifier = {
	.notifier_call = s5pv310_cpufreq_policy_notifier_call,
};

static int s5pv310_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	policy->cur = policy->min = policy->max = s5pv310_getspeed(policy->cpu);

	cpufreq_frequency_table_get_attr(cpufreq_freq_table, policy->cpu);

	/* set the transition latency value
	 */
	policy->cpuinfo.transition_latency = CPUFREQ_SAMPLING_RATE;

	/* S5PV310 multi-core processors has 2 cores
	 * that the frequency cannot be set independently.
	 * Each cpu is bound to the same speed.
	 * So the affected cpu is all of the cpus.
	 */
	if (!cpu_online(1)) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}

	return cpufreq_frequency_table_cpuinfo(policy, cpufreq_freq_table);
}

static struct cpufreq_driver s5pv310_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = s5pv310_verify_policy,
	.target = s5pv310_target,
	.get = s5pv310_getspeed,
	.init = s5pv310_cpufreq_cpu_init,
	.name = "s5pv310_cpufreq",
#ifdef CONFIG_PM
	.suspend = s5pv310_cpufreq_suspend,
	.resume = s5pv310_cpufreq_resume,
#endif
};

#ifdef CONFIG_S5PV310_ASV

#define IDS_INDEX	0
#define HPM_INDEX	1

#define IDS_OFFSET	24
#define IDS_MASK	0xFF

struct s5pv310_asv_info asv_info = {
	.group = 0,
	.loop_cnt = 50,
	.level = ASV_8_GROUP_END,
	.asv_init_done = false,
};
EXPORT_SYMBOL(asv_info);

static int init_iem_clock(void)
{
	struct clk *clk_hpm = NULL;
	struct clk *clk_pwi = NULL;
	struct clk *clk_pwi_parent = NULL;
	struct clk *clk_copy = NULL;
	struct clk *clk_copy_parent = NULL;

	/* PWI clock setting */
	clk_pwi = clk_get(NULL, "sclk_pwi");
	if (IS_ERR(clk_pwi)) {
		printk(KERN_ERR"ASV : SCLK_PWI clock get error\n");
		goto out;
	}
	clk_pwi_parent = clk_get(NULL, "xusbxti");
	if (IS_ERR(clk_pwi_parent)) {
		printk(KERN_ERR"ASV : MOUT_APLL clock get error\n");
		goto out;
	}

	clk_set_parent(clk_pwi, clk_pwi_parent);
	clk_put(clk_pwi_parent);

	clk_set_rate(clk_pwi, 4800000);
	clk_put(clk_pwi);

	/* HPM clock setting */
	clk_copy = clk_get(NULL, "dout_copy");
	if (IS_ERR(clk_copy)) {
		printk(KERN_ERR"ASV : DOUT_COPY clock get error\n");
		goto out;
	}
	clk_copy_parent = clk_get(NULL, "mout_mpll");
	if (IS_ERR(clk_copy_parent)) {
		printk(KERN_ERR"ASV : MOUT_MPLL clock get error\n");
		goto out;
	}

	clk_set_parent(clk_copy, clk_copy_parent);
	clk_put(clk_copy_parent);

	clk_set_rate(clk_copy, (400 * 1000 * 1000));
	clk_put(clk_copy);

	clk_hpm = clk_get(NULL, "sclk_hpm");
	if (IS_ERR(clk_hpm))
		return -EINVAL;

	clk_set_rate(clk_hpm, (200 * 1000 * 1000));
	clk_put(clk_hpm);

	return 0;
out:
	if (!IS_ERR(clk_hpm))
		clk_put(clk_hpm);

	if (!IS_ERR(clk_copy_parent))
		clk_put(clk_copy_parent);

	if (!IS_ERR(clk_copy))
		clk_put(clk_copy);

	if (!IS_ERR(clk_pwi_parent))
		clk_put(clk_pwi_parent);

	if (!IS_ERR(clk_pwi))
		clk_put(clk_pwi);

	return -EINVAL;
}

void set_iem_clock(void)
{
	/* APLL_CON0 level register */
	__raw_writel(0x80FA0601, S5P_APLL_CON0L8);
	__raw_writel(0x80C80601, S5P_APLL_CON0L7);
	__raw_writel(0x80C80602, S5P_APLL_CON0L6);
	__raw_writel(0x80C80604, S5P_APLL_CON0L5);
	__raw_writel(0x80C80601, S5P_APLL_CON0L4);
	__raw_writel(0x80C80601, S5P_APLL_CON0L3);
	__raw_writel(0x80C80601, S5P_APLL_CON0L2);
	__raw_writel(0x80C80601, S5P_APLL_CON0L1);

	/* IEM Divider register */
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L8);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L7);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L6);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L5);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L4);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L3);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L2);
	__raw_writel(0x00500000, S5P_CLKDIV_IEM_L1);
}
static int s5pv310_get_hpm_code(void)
{
	unsigned int i;
	unsigned long hpm_code = 0;
	unsigned int tmp;
	unsigned int asv_loop_cnt = asv_info.loop_cnt;
	unsigned int hpm[asv_loop_cnt];
	static void __iomem *iem_base;
	struct clk *clk_iec = NULL;
	struct clk *clk_apc = NULL;
	struct clk *clk_hpm = NULL;

	iem_base = ioremap(S5PV310_PA_IEM, (128 * 1024));
	if (iem_base == NULL) {
		printk(KERN_ERR "faile to ioremap\n");
		goto out;
	}
	/* IEC clock gate enable */
	clk_iec = clk_get(NULL, "iem-iec");
	if (IS_ERR(clk_iec)) {
		printk(KERN_ERR"ASV : IEM IEC clock get error\n");
		goto out;
	}
	clk_enable(clk_iec);

	/* APC clock gate enable */
	clk_apc = clk_get(NULL, "iem-apc");
	if (IS_ERR(clk_apc)) {
		printk(KERN_ERR"ASV : IEM APC clock get error\n");
		goto out;
	}
	clk_enable(clk_apc);

	/* hpm clock gate enalbe */
	clk_hpm = clk_get(NULL, "hpm");
	if (IS_ERR(clk_hpm)) {
		printk(KERN_ERR"ASV : HPM clock get error\n");
		goto out;
	}
	clk_enable(clk_hpm);

	if (init_iem_clock()) {
		printk(KERN_ERR "ASV driver clock_init fail\n");
		goto out;
	} else {
		/* HPM enable  */
		tmp = __raw_readl(iem_base + S5PV310_APC_CONTROL);
		tmp |= APC_HPM_EN;
		__raw_writel(tmp, (iem_base + S5PV310_APC_CONTROL));

		set_iem_clock();

		/* IEM enable */
		tmp = __raw_readl(iem_base + S5PV310_IECDPCCR);
		tmp |= IEC_EN;
		__raw_writel(tmp, (iem_base + S5PV310_IECDPCCR));
	}

	for (i = 0; i < asv_loop_cnt; i++) {
		tmp = __raw_readb(iem_base + S5PV310_APC_DBG_DLYCODE);
		hpm_code += tmp;
		hpm[i] = tmp;
	}

	for (i = 0; i < asv_loop_cnt; i++)
		printk(KERN_INFO "ASV : hpm[%d] = %d value\n", i, hpm[i]);

	hpm_code /= asv_loop_cnt;
	printk(KERN_INFO "ASV hpm_code average value : %lu\n", hpm_code);
	hpm_code -= 1;

	/* HPM clock gate disable */
	clk_disable(clk_hpm);
	clk_put(clk_hpm);

	/* APC clock gate disable */
	clk_disable(clk_apc);
	clk_put(clk_apc);

	/* IEC clock gate disable */
	clk_disable(clk_iec);
	clk_put(clk_iec);

	iounmap(iem_base);

	return hpm_code;

out:
	if (!IS_ERR(clk_hpm)) {
		clk_disable(clk_hpm);
		clk_put(clk_hpm);
	}

	if (!IS_ERR(clk_apc)) {
		clk_disable(clk_apc);
		clk_put(clk_apc);
	}

	if (!IS_ERR(clk_iec)) {
		clk_disable(clk_iec);
		clk_put(clk_iec);
	}

	iounmap(iem_base);

	return -EINVAL;
}
static int s5pv310_get_hpm_group(void)
{
	unsigned int hpm_code, i;
	unsigned int hpm_group = 0xff;

	hpm_code = s5pv310_get_hpm_code();
	printk(KERN_INFO "ASV hpm_code = %d\n", hpm_code);

	/* hpm grouping */
	if (asv_info.level == ASV_8_GROUP_END) {
		for (i = 0; i < (ASV_8_GROUP_END - 1); i++) {
			if (hpm_code <= asv_8_threshold[HPM_INDEX][i])
				break;
		}
	} else if (asv_info.level == ASV_5_GROUP_END) {
		for (i = 0; i < (ASV_5_GROUP_END - 1); i++) {
			if (hpm_code <= asv_5_threshold[HPM_INDEX][i])
				break;
		}
	}
	hpm_group = i;

	return hpm_group;
}

static int s5pv310_get_ids_arm_group(void)
{
	unsigned int ids_arm, tmp, i;
	unsigned int ids_arm_group = 0xff;
	struct clk *clk_chipid = NULL;

	/* chip id clock gate enable*/
	clk_chipid = clk_get(NULL, "chipid");
	if (IS_ERR(clk_chipid)) {
		printk(KERN_ERR "ASV : chipid clock get error\n");
		return -EINVAL;
	}
	clk_enable(clk_chipid);

	tmp = __raw_readl(S5P_VA_CHIPID + 0x4);

	/* get the ids_arm */
	ids_arm = ((tmp >> IDS_OFFSET) & IDS_MASK);
	if (!ids_arm) {
		printk(KERN_ERR "S5PV310 : Cannot read IDS\n");
		goto out;
	}
	printk(KERN_INFO "ASV ids_arm = %d\n", ids_arm);

	clk_disable(clk_chipid);
	clk_put(clk_chipid);

	/* ids grouping */
	if (asv_info.level == ASV_8_GROUP_END) {
		for (i = 0; i < (ASV_8_GROUP_END - 1); i++) {
			if (ids_arm <= asv_8_threshold[IDS_INDEX][i])
				break;
		}
	} else if (asv_info.level == ASV_5_GROUP_END) {
		for (i = 0; i < (ASV_5_GROUP_END - 1); i++) {
			if (ids_arm <= asv_5_threshold[IDS_INDEX][i])
				break;
		}
	}
	ids_arm_group = i;

	return ids_arm_group;

out:
	if (!IS_ERR(clk_chipid)) {
		clk_disable(clk_chipid);
		clk_put(clk_chipid);
	}

	return -EINVAL;
}
static int s5pv310_get_asv_level(void)
{
	unsigned int asv_level;
	unsigned int max_arm_clk;

	max_arm_clk = s5pv310_get_max_speed();

	/* asv level uses 5 level when max_arm_clk is 1400MHz */
	if (max_arm_clk == available_cpufreq_level[0].frequency)
		asv_level = ASV_5_GROUP_END;
	else {
		asv_level = ASV_8_GROUP_END;
#ifdef CONFIG_ARM_OVERCLK_TO_1400
		asv_level = ASV_5_GROUP_END;
#endif
	}

	return asv_level;
}
static int s5pv310_get_asv_info(void)
{
	unsigned int hpm_group = 0xff, ids_arm_group = 0xff;
	unsigned int asv_group, asv_level;

	/* get the asv level */
	asv_level = s5pv310_get_asv_level();
	asv_info.level = asv_level;

	/* get the ids_arm and hpm group */
	ids_arm_group = s5pv310_get_ids_arm_group();
	hpm_group = s5pv310_get_hpm_group();

	/* decide asv group */
	if (ids_arm_group > hpm_group) {
		if (ids_arm_group - hpm_group >= 3)
			asv_group = ids_arm_group - 3;
		else
			asv_group = hpm_group;
	} else {
		if (hpm_group - ids_arm_group >= 3)
			asv_group = hpm_group - 3;
		else
			asv_group = ids_arm_group;
	}

	/* set asv infomation for 3D */
	asv_info.group = asv_group;
	asv_info.asv_init_done = true;

	printk(KERN_INFO "******************************ASV *********************\n");
	printk(KERN_INFO "ids_group %d hpm_group %d asv_group = %d\n",
		ids_arm_group, hpm_group, asv_info.group);

	return 0;
}

static unsigned int s5pv310_getspeed_dmc(void)
{
	unsigned long rate;

	rate = (clk_get_rate(sclk_dmc) / (1000*1000)) * 1000;

	return rate;
}

static void s5pv310_set_asv_voltage(void)
{
	unsigned int arm_index = 0, int_index = 0;
	unsigned int arm_volt, int_volt;
	unsigned int rate, i;

	/* get current ARM level */
	mutex_lock(&set_cpu_freq_change);

	freqs.old = s5pv310_getspeed(0);

	for (i = 0; cpufreq_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		if (freqs.old == cpufreq_freq_table[i].frequency) {
			arm_index = cpufreq_freq_table[i].index;
			arm_volt = cpufreq_table[i].arm_volt;
#if defined(CONFIG_REGULATOR)
			regulator_set_voltage(arm_regulator, arm_volt, arm_volt);
#endif
			break;
		}
	}

	mutex_unlock(&set_cpu_freq_change);

	/* get current INT level */
	mutex_lock(&set_bus_freq_change);

	rate = s5pv310_getspeed_dmc();

	for (i = 0; i < BUSFREQ_LEVEL_END; i++) {
		/* round up to the defined BUS_L1 */
		if (i == BUS_L1)
			rate += 1000;
		if (s5pv310_busfreq_table[i].mem_clk == rate) {
			int_index = s5pv310_busfreq_table[i].index;
			int_volt = s5pv310_busfreq_table[i].volt;
#if defined(CONFIG_REGULATOR)
			regulator_set_voltage(int_regulator, int_volt, int_volt);
#endif
			break;
		}
	}

	mutex_unlock(&set_bus_freq_change);

	printk(KERN_INFO "******************************ASV *********************\n");
	printk(KERN_INFO "ASV**** current arm_index %d, arm_volt %d\n",
			arm_index, arm_volt);
	printk(KERN_INFO "ASV**** current int_index %d, int_volt %d\n",
			int_index, int_volt);
}

#endif

static void print_cpufreq_table(void)
{
	unsigned int i, j;

	printk(KERN_INFO "@@@@@@@@@@@@ cpufreq table values @@@@@@@@@@@@@@@@\n");
	for (i = 0; cpufreq_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		printk(KERN_INFO "index = %d, frequency = %d\n",
					cpufreq_freq_table[i].index,
					cpufreq_freq_table[i].frequency);
	}

	printk(KERN_INFO "index, apll_pms, clkdiv_cpu0, clkdiv_cpu1, arm_volt\n");
	for (i = 0; cpufreq_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		printk(KERN_INFO "%4d, 0x%x, 0x%x, 0x%x, %duV\n",
					cpufreq_table[i].index,
					cpufreq_table[i].apll_pms,
					cpufreq_table[i].clkdiv_cpu0,
					cpufreq_table[i].clkdiv_cpu1,
					cpufreq_table[i].arm_volt);
	}

	printk(KERN_INFO "freq_trans_table\n");
	for (i = 0; cpufreq_freq_table[i].frequency != CPUFREQ_TABLE_END; i++) {
		for (j = 0; cpufreq_freq_table[j].frequency != CPUFREQ_TABLE_END; j++)
			printk("%d,", cpufreq_table[i].freq_trans[j]);
		printk(KERN_INFO "\n");
	}
}

static int s5pv310_cpufreq_table_update(void)
{
	unsigned int i, j, tmp;
	unsigned int start, cur, col;
	unsigned int check_max_1400 = 0;
	unsigned int asv_group;

	/*
	 * By getting chip information from pkg_id & pro_id register,
	 * get the maximum arm clock and set cpufreq information.
	*/
	cpufreq_info.max_arm_clk = s5pv310_get_max_speed();
	if (cpufreq_info.max_arm_clk < 0) {
		printk(KERN_ERR "Fail to get max armclk infomation. Use 1.2GHz\n");
		cpufreq_info.max_arm_clk = CPUFREQ_1200MHZ;
	}
#ifdef CONFIG_ARM_OVERCLK_TO_1400
	if (cpufreq_info.max_arm_clk == CPUFREQ_1200MHZ) {
		cpufreq_info.max_arm_clk = CPUFREQ_1400MHZ;
		printk(KERN_INFO "===================================\n");
		printk(KERN_INFO "===========   CAUTION !!!==========\n");
		printk(KERN_INFO "Temporarily use over clock 1.4GHz!!\n");
		printk(KERN_INFO "in spite of max 1.2GHz chip\n");
		printk(KERN_INFO "It's just for development\n");
		printk(KERN_INFO "===================================\n");
	}
#endif
	printk(KERN_INFO "armclk set max %d\n", cpufreq_info.max_arm_clk);

	if (cpufreq_info.max_arm_clk == available_cpufreq_level[0].frequency)
		check_max_1400 = 1;

	for (i = 0; i < ARRAY_SIZE(available_cpufreq_table); i++) {
		tmp = available_cpufreq_table[i][0];
		if (cpufreq_info.max_arm_clk ==
				available_cpufreq_level[tmp].frequency) {
			cpufreq_info.table_entry = i;
			printk(KERN_INFO "Use cpufreq table %d entry\n",
						cpufreq_info.table_entry);
			break;
		} else if (i == (ARRAY_SIZE(available_cpufreq_table) - 1)) {
			printk(KERN_ERR "***********************************");
			printk(KERN_ERR "CPUFREQ table cannot support max arm clock\n");
			printk(KERN_ERR "Plz update cpufreq table\n");
			printk(KERN_ERR "system halt !!!\n");
			printk(KERN_ERR "***********************************");
			panic("fault cpufreq");
		}
	}

#ifdef CONFIG_S5PV310_ASV
	asv_group = asv_info.group;
#else
	asv_group = ASV_DEFAULT_GR_NUM;
#endif

	/* 1st cpufreq level from available_cpufreq_table */
	start = available_cpufreq_table[cpufreq_info.table_entry][0];

	/* Make cpufreq table according to table_entry */
	for (i = 0, cur = start; cur != CPUFREQ_LEVEL_END;
		i++, cur = available_cpufreq_table[cpufreq_info.table_entry][i]) {

		cpufreq_freq_table[i].index = i;
		cpufreq_freq_table[i].frequency = available_cpufreq_level[cur].frequency;

		cpufreq_table[i].index = i;
		cpufreq_table[i].apll_pms = apll_pms_value[cur];

		/* update clkdiv_cpu0 & clkdiv_cpu1 */
		tmp = __raw_readl(S5P_CLKDIV_CPU);
		tmp &= ~(S5P_CLKDIV_CPU0_CORE_MASK | S5P_CLKDIV_CPU0_COREM0_MASK |
			S5P_CLKDIV_CPU0_COREM1_MASK | S5P_CLKDIV_CPU0_PERIPH_MASK |
			S5P_CLKDIV_CPU0_ATB_MASK | S5P_CLKDIV_CPU0_PCLKDBG_MASK |
			S5P_CLKDIV_CPU0_APLL_MASK);
		tmp |= ((clkdiv_cpu0[cur][0] << S5P_CLKDIV_CPU0_CORE_SHIFT) |
			(clkdiv_cpu0[cur][1] << S5P_CLKDIV_CPU0_COREM0_SHIFT) |
			(clkdiv_cpu0[cur][2] << S5P_CLKDIV_CPU0_COREM1_SHIFT) |
			(clkdiv_cpu0[cur][3] << S5P_CLKDIV_CPU0_PERIPH_SHIFT) |
			(clkdiv_cpu0[cur][4] << S5P_CLKDIV_CPU0_ATB_SHIFT) |
			(clkdiv_cpu0[cur][5] << S5P_CLKDIV_CPU0_PCLKDBG_SHIFT) |
			(clkdiv_cpu0[cur][6] << S5P_CLKDIV_CPU0_APLL_SHIFT));
		cpufreq_table[i].clkdiv_cpu0 = tmp;

		tmp = __raw_readl(S5P_CLKDIV_CPU1);
		tmp &= ~(S5P_CLKDIV_CPU1_COPY_MASK | S5P_CLKDIV_CPU1_HPM_MASK);
		tmp |= ((clkdiv_cpu1[cur][0] << S5P_CLKDIV_CPU1_COPY_SHIFT) |
			(clkdiv_cpu1[cur][1] << S5P_CLKDIV_CPU1_HPM_SHIFT));
		cpufreq_table[i].clkdiv_cpu1 = tmp;

		for (j = 0, col = start; col != CPUFREQ_LEVEL_END;
			j++, col = available_cpufreq_table[cpufreq_info.table_entry][j])
			cpufreq_table[i].freq_trans[j] = freq_trans_table[cur][col];

		if (check_max_1400) {
#ifdef CONFIG_ARM_OVERCLK_TO_1400
			if (s5pv310_get_max_speed() == CPUFREQ_1200MHZ)
				cpufreq_table[i].arm_volt =
				asv_armvolt_table_1400_overclk[asv_group][cur];
			else
#endif
			cpufreq_table[i].arm_volt =
				asv_armvolt_table_1400[asv_group][cur];
		} else {
			cpufreq_table[i].arm_volt =
				asv_armvolt_table[asv_group][cur];
		}
	}
	/* cpufreq min level is lowest level in table */
	cpufreq_lock.min_level = (i - 1);

	/* Update end of cpufreq_freq_table */
	cpufreq_freq_table[i].index = i;
	cpufreq_freq_table[i].frequency = CPUFREQ_TABLE_END;

#ifdef CONFIG_S5PV310_ASV
	/* Update VDD_INT table */
	for (i = 0; i < BUSFREQ_LEVEL_END; i++) {
		if (asv_info.level == ASV_5_GROUP_END)
			s5pv310_busfreq_table[i].volt = asv_int_volt_5_table[asv_info.group][i];
		else
			s5pv310_busfreq_table[i].volt = asv_int_volt_8_table[asv_info.group][i];

		printk(KERN_INFO "ASV busfreq_table[%d].volt = %d\n",
			i, s5pv310_busfreq_table[i].volt);
	}
#endif

	/* logout the selected cpufreq table value for debugging */
	print_cpufreq_table();

	return 0;
}

#ifdef CONFIG_MACH_C1
extern void max8997_set_arm_voltage_table(int *voltage_table, int arr_size);

static void s5pv310_cpufreq_set_pmic_vol_table(void)
{
	int vol_table[CPUFREQ_LEVEL_END];
	int i;

	for (i = 0; i < CPUFREQ_LEVEL_END; i++)
		vol_table[i] = cpufreq_table[i].arm_volt;

	max8997_set_arm_voltage_table(vol_table, CPUFREQ_LEVEL_END);
}
#endif /* CONFIG_MACH_C1 */

static int __init s5pv310_cpufreq_init(void)
{
	int i, ret;

	printk(KERN_INFO "++ %s\n", __func__);

	arm_clk = clk_get(NULL, "armclk");
	if (IS_ERR(arm_clk))
		return PTR_ERR(arm_clk);

	moutcore = clk_get(NULL, "moutcore");
	if (IS_ERR(moutcore))
		goto out;

	mout_mpll = clk_get(NULL, "mout_mpll");
	if (IS_ERR(mout_mpll))
		goto out;

	mout_apll = clk_get(NULL, "mout_apll");
	if (IS_ERR(mout_apll))
		goto out;

	sclk_dmc = clk_get(NULL, "sclk_dmc");
	if (IS_ERR(sclk_dmc))
		goto out;

#if defined(CONFIG_REGULATOR)
	arm_regulator = regulator_get(NULL, "vdd_arm");
	if (IS_ERR(arm_regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "vdd_arm");
		goto out;
	}
	int_regulator = regulator_get(NULL, "vdd_int");
	if (IS_ERR(int_regulator)) {
		printk(KERN_ERR "failed to get resource %s\n", "vdd_int");
		goto out;
	}
	disable_further_cpufreq = false;
#endif

#ifdef CONFIG_S5PV310_ASV
#if defined(CONFIG_REGULATOR)
	pr_info("%s: set vdd_arm (1.2V)\n", __func__);
	ret = regulator_set_voltage(arm_regulator, 1200000, 1200000);
	if (ret < 0)
		pr_err("%s: fail to set vdd_arm(%d)\n", __func__, ret);
#endif
	if (s5pv310_get_asv_info())
		return -EINVAL;
#endif

	/*
	 * update cpufreq table according to max_arm_clk
	 * based on available_cpufreq_table.
	*/
	if (s5pv310_cpufreq_table_update() < 0)
		printk(KERN_INFO "arm clock limited to max 1000MHz.\n");

#ifdef CONFIG_MACH_C1
	s5pv310_cpufreq_set_pmic_vol_table();
#endif

#ifdef CONFIG_S5PV310_ASV
	/* Update current VDD_ARM/VDD_INT voltage through ASV info */
	s5pv310_set_asv_voltage();
#endif

	up_threshold = UP_THRESHOLD_DEFAULT;
	cpu.cpu_hw_base = S5PV310_VA_PPMU_CPU;
	dmc[DMC0].dmc_hw_base = S5P_VA_DMC0;
	dmc[DMC1].dmc_hw_base = S5P_VA_DMC1;

	busfreq_ppmu_init();
	cpu_ppmu_init();

	for (i = 0; i < DVFS_LOCK_ID_END; i++)
		busfreq_lock.value[i] = busfreq_lock.min_level;

	/* g_cpufreq_lock_val & g_cpufreq_lock level
	 * initialize to minimum cpufreq level
	*/
	for (i = 0; i < DVFS_LOCK_ID_END; i++)
		cpufreq_lock.value[i] = cpufreq_lock.min_level;
	cpufreq_lock.level = cpufreq_lock.min_level;

	register_pm_notifier(&s5pv310_cpufreq_notifier);
	register_reboot_notifier(&s5pv310_cpufreq_reboot_notifier);
	cpufreq_register_notifier(&s5pv310_cpufreq_policy_notifier,
						CPUFREQ_POLICY_NOTIFIER);

	cpufreq_info.init_done = true;

	printk(KERN_INFO "-- %s\n", __func__);
	return cpufreq_register_driver(&s5pv310_driver);

out:
	if (!IS_ERR(arm_clk))
		clk_put(arm_clk);

	if (!IS_ERR(moutcore))
		clk_put(moutcore);

	if (!IS_ERR(mout_mpll))
		clk_put(mout_mpll);

	if (!IS_ERR(mout_apll))
		clk_put(mout_apll);

#ifdef CONFIG_REGULATOR
	if (!IS_ERR(arm_regulator))
		regulator_put(arm_regulator);

	if (!IS_ERR(int_regulator))
		regulator_put(int_regulator);
#endif
	printk(KERN_ERR "%s: failed initialization\n", __func__);
	return -EINVAL;
}

late_initcall(s5pv310_cpufreq_init);

static ssize_t show_busfreq_fix(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	if (!busfreq_fix)
		return sprintf(buf, "Busfreq is NOT fixed\n");
	else
		return sprintf(buf, "Busfreq is Fixed\n");
}

static ssize_t store_busfreq_fix(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int i, ret;

	ret = sscanf(buf, "%u", &busfreq_fix);

	if (ret != 1)
		return -EINVAL;

	mutex_lock(&set_bus_freq_change);
	if (busfreq_fix) {
		for (i = 0; i < DMC_NUM; i++)
			s5pv310_dmc_ppmu_stop(&dmc[i]);
		s5pv310_cpu_ppmu_stop(&cpu);
	} else {
		busfreq_ppmu_init();
		cpu_ppmu_init();
	}
	mutex_unlock(&set_bus_freq_change);

	return count;
}

static DEVICE_ATTR(busfreq_fix, 0644, show_busfreq_fix, store_busfreq_fix);

static ssize_t show_fix_busfreq_level(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	if (!busfreq_fix)
		return sprintf(buf,
		"busfreq level fix is only available in busfreq_fix state\n");
	else
		return sprintf(buf, "BusFreq Level L%u\n", fix_busfreq_level);
}

static ssize_t store_fix_busfreq_level(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int ret;
	if (!busfreq_fix) {
		printk(KERN_ERR
		"busfreq level fix is only avaliable in Busfreq Fix state\n");
		return count;
	}

	ret = sscanf(buf, "%u", &fix_busfreq_level);
	if (ret != 1)
		return -EINVAL;

	if (fix_busfreq_level >= BUSFREQ_LEVEL_END) {
		printk(KERN_INFO "Fixing Busfreq level is invalid\n");
		return count;
	}

	mutex_lock(&set_bus_freq_change);
	s5pv310_set_busfreq(fix_busfreq_level);
	mutex_unlock(&set_bus_freq_change);
	return count;
}

static DEVICE_ATTR(fix_busfreq_level, 0644, show_fix_busfreq_level,
				store_fix_busfreq_level);

static ssize_t show_cur_busfreq(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "bus speed=%u\n", s5pv310_busfreq_table[cur_busfreq_index].mem_clk);
}

static DEVICE_ATTR(cur_busfreq, 0444, show_cur_busfreq, NULL);

#ifdef SYSFS_DEBUG_BUSFREQ
static ssize_t show_time_in_state(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t len = 0;
	int i;

	for (i = 0; i < BUSFREQ_LEVEL_END; i++)
		len += sprintf(buf + len, "%u: %u\n",
			s5pv310_busfreq_table[i].mem_clk, time_in_state[i]);

	return len;
}

static DEVICE_ATTR(time_in_state, 0444, show_time_in_state, NULL);

static ssize_t show_up_threshold(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return sprintf(buf, "%u\n", up_threshold);
}

static ssize_t store_up_threshold(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	int ret;

	ret = sscanf(buf, "%u", &up_threshold);
	if (ret != 1)
		return -EINVAL;

	printk(KERN_INFO "** Up_Threshold is changed to %u **\n", up_threshold);

	return count;
}

static DEVICE_ATTR(up_threshold, 0644, show_up_threshold, store_up_threshold);
#endif

static int sysfs_busfreq_create(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_busfreq_fix);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_fix_busfreq_level);
	if (ret)
		goto busfreq_level_err;

	ret = device_create_file(dev, &dev_attr_cur_busfreq);
	if (ret)
		goto cur_busfreq_err;

#ifdef SYSFS_DEBUG_BUSFREQ
	ret = device_create_file(dev, &dev_attr_up_threshold);
	if (ret)
		goto up_threshold_err;

	ret = device_create_file(dev, &dev_attr_time_in_state);
	if (ret)
		goto time_in_state_err;

	return ret;

time_in_state_err:
	device_remove_file(dev, &dev_attr_up_threshold);
up_threshold_err:
	device_remove_file(dev, &dev_attr_cur_busfreq);
#else
	return ret;
#endif
cur_busfreq_err:
	device_remove_file(dev, &dev_attr_fix_busfreq_level);
busfreq_level_err:
	device_remove_file(dev, &dev_attr_busfreq_fix);

	return ret;
}

static struct platform_device s5pv310_busfreq_device = {
	.name	= "s5pv310-busfreq",
	.id	= -1,
};

static int __init s5pv310_busfreq_device_init(void)
{
	int ret;

	ret = platform_device_register(&s5pv310_busfreq_device);
	if (ret) {
		printk(KERN_ERR "failed at(%d)\n", __LINE__);
		return ret;
	}

	ret = sysfs_busfreq_create(&s5pv310_busfreq_device.dev);
	if (ret) {
		printk(KERN_ERR "failed at(%d)\n", __LINE__);
		goto sysfs_err;
	}

	printk(KERN_INFO "s5pv310_busfreq_device_init: %d\n", ret);

	return ret;
sysfs_err:

	platform_device_unregister(&s5pv310_busfreq_device);
	return ret;
}

late_initcall(s5pv310_busfreq_device_init);
