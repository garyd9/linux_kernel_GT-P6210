/* linux/arch/arm/mach-s5pv310/include/mach/cpufreq.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* CPUFreq for S5PV310/S5PC210 */

/* CPU frequency level index */
enum cpufreq_level_index{
	CPU_L0,		/* 1.4GHz */
	CPU_L1,		/* 1.2GHz */
	CPU_L2,		/* 1GHz */
	CPU_L3,		/* 800MHz */
	CPU_L4,		/* 500MHz */
	CPU_L5,		/* 200MHz */
	CPUFREQ_LEVEL_END,
};

/* BUS frequency level index */
enum busfreq_level_index{
	BUS_L0,		/* MEM 400MHz BUS 200MHz */
	BUS_L1,		/* MEM 267MHz BUS 160MHz */
	BUS_L2,		/* MEM 133MHz BUS 133MHz */
	BUSFREQ_LEVEL_END,
};

enum cpufreq_lock_ID{
	DVFS_LOCK_ID_TV,	/* TV */
	DVFS_LOCK_ID_MFC,	/* MFC */
	DVFS_LOCK_ID_CAM,	/* CAM */
	DVFS_LOCK_ID_APP,	/* APP */
	DVFS_LOCK_ID_PM,	/* PM */
	DVFS_LOCK_ID_WLAN,	/* WLAN */
	DVFS_LOCK_ID_TSP,	/* TSP */
	DVFS_LOCK_ID_TMU,	/* TMU */
	DVFS_LOCK_ID_IR_LED,	/* IR_LED */
	DVFS_LOCK_ID_END,
};

#define CPUFREQ_1400MHZ	1400000
#define CPUFREQ_1200MHZ	1200000
#define CPUFREQ_1000MHZ	1000000
#define CPUFREQ_800MHZ   800000
#define CPUFREQ_500MHZ   500000
#define CPUFREQ_200MHZ   200000

/*
 * struct s5pv310_cpufreq table - has information as follows.
 *
 * @index
 *	cpu frequency level index
 * @apll_pms
 *	When cpufreq change, need to update p,m,s value in APLL
 *	this value indicates p,m,s value at each level
 * @clkdiv_cpu0
 * @clkdiv_cpu1
 *      CLK_DIV_CPU0/1 register value for cpufreq
 * @freq_trans[]
 *	requirements for cpufreq level change
 * @arm_volt
 *	arm voltage value for cpufreq
*/
struct s5pv310_cpufreq_table {
	unsigned int index;
	unsigned int apll_pms;
	unsigned int clkdiv_cpu0;
	unsigned int clkdiv_cpu1;
	unsigned int freq_trans[CPUFREQ_LEVEL_END];
	unsigned int arm_volt;
};

/*
 * struct s5pv310_cpufreq info - has information as follows.
 *
 * @max_arm_clk
 *	indicates the maximum arm clock in KHz to be opearable
 *	according chip ID.
 * @table_entry
 *	has lists to be selected from available_cpufreq_table.
 * @init_done
 *      has info whether finish cpufreq driver init.
*/
struct s5pv310_cpufreq_info {
	int max_arm_clk;  /* in order of KHz */
	unsigned int table_entry;
	bool init_done;
};

/* Structure for keeping frequency locking information */
struct freq_lock_info {
	unsigned int dev_id;
	unsigned int level;
	unsigned int value[DVFS_LOCK_ID_END];
	unsigned int min_level;
	unsigned int max_level;
	bool disable_lock;
};

#define ASV_DEFAULT_GR_NUM 0

/*
 * struct s5pv310_asv_info - has information as follows.
 *
 * @ group
 *	has which group is included in case of supporting ASV
 * @ loop_cnt
 *	has loop count value to read average value of hpm
 * @ level
 *	has info what number of asv group
 * @ asv_init_done
 *	has info whether finish asv group init.
*/
struct s5pv310_asv_info {
	unsigned int group;
	unsigned int loop_cnt;
	unsigned int level;
	bool asv_init_done;
};

int s5pv310_cpufreq_round_idx(unsigned int cpu_freq);
int s5pv310_cpufreq_lock(unsigned int nId, enum cpufreq_level_index cpufreq_level);
void s5pv310_cpufreq_lock_free(unsigned int nId);

int s5pv310_cpufreq_upper_limit(unsigned int nId, enum cpufreq_level_index cpufreq_level);
void s5pv310_cpufreq_upper_limit_free(unsigned int nId);

int s5pv310_busfreq_lock(unsigned int nId, enum busfreq_level_index busfreq_level);
void s5pv310_busfreq_lock_free(unsigned int nId);
