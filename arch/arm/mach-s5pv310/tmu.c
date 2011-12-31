/* linux/arch/arm/mach-s5pv310/tmu.c
*
* Copyright (c) 2010 Samsung Electronics Co., Ltd.
*      http://www.samsung.com/
*
* S5PV310 - TMU driver
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/slab.h>

#include <asm/irq.h>

#include <mach/regs-tmu.h>
#include <mach/cpufreq.h>
#include <mach/map.h>
#include <mach/dmc.h>
#include <plat/s5p-tmu.h>
#include <plat/gpio-cfg.h>

/*
 *  For example
 *  test value for room temperature
 *  base operation temp : OPERATION_TEMP_BASE_78
*/
/* Selectable one room temperature among 3 kinds */
#undef  OPERATION_TEMP_BASE_78
#define OPERATION_TEMP_BASE_61

#ifdef OPERATION_TEMP_BASE_78
/* TMU register setting value */
#define THD_TEMP 0x80	  /* 78 degree  : threshold temp */
#define TRIGGER_LEV0 0x9  /* 87 degree  : throttling temperature */
#define	TRIGGER_LEV1 0x19 /* 103 degree  : Waring temperature */
#define TRIGGER_LEV2 0x20 /* 110 degree : Tripping temperature  */
#define	TRIGGER_LEV3 0xFF /* Reserved */
#define TEMP_1ST_TROTTLED_CELCIUS	 87
#endif

#ifdef OPERATION_TEMP_BASE_61
/* test on 35 celsius base */
#define THD_TEMP     0x6F /* 61 degree: thershold temp */
#define TRIGGER_LEV0 0x3  /* 64	degree: Throttling temperature */
#define TRIGGER_LEV1 0x2A /* 103 degree: Waring temperature */
#define TRIGGER_LEV2 0x31 /* 110 degree: Tripping temperature */
#define TRIGGER_LEV3 0xFF /* Reserved */
#define TEMP_1ST_TROTTLED_CELCIUS	64
#endif

/* interrupt level by celcius degree */
#define TEMP_MIN_CELCIUS	25
#define TEMP_TQ0_CELCIUS	85
#define TEMP_2ND_TROTTLED_CELCIUS	103
#define TEMP_TRIPPED_CELCIUS	110
#define TEMP_MAX_CELCIUS	125

#define TMU_SAVE_NUM   10
#define VREF_SLOPE     0x07000F02
#define TMU_EN         0x1
#define TMU_DC_VALUE   25
#define TMU_CODE_25_DEGREE 0x4B
#define EFUSE_MIN_VALUE 60
#define EFUSE_AVG_VALUE 80
#define EFUSE_MAX_VALUE 100

/* flags that throttling or trippint is treated */
#define THROTTLE_FLAG (0x1 << 0)
#define WARNING_FLAG (0x1 << 1)
#define TRIPPING_FLAG	(0x1 << 2)
#define MEM_THROTTLE_FLAG (0x1 << 4)
#define RETRY_FLAG (0x1 << 5)

static struct workqueue_struct  *tmu_monitor_wq;
unsigned int tmu_save[TMU_SAVE_NUM];

enum tmu_status_t {
	TMU_STATUS_NORMAL = 0,
	TMU_STATUS_THROTTLED,
	TMU_STATUS_WARNING,
	TMU_STATUS_TRIPPED,
	TMU_STATUS_INIT,
};

/*
 * struct temperature_set has values to manange throttling, tripping
 * and other software safety control
 */
struct temperature_set {
	unsigned int stop_1st_throttle;
	unsigned int start_1st_throttle;
	unsigned int stop_2nd_throttle;
	unsigned int start_2nd_throttle;
	unsigned int start_tripping; /* temp to do tripping */
	unsigned int retry_tripping; /* If trip is delayed, retry tripping */
	unsigned int start_emergency; /* To protect chip,forcely kernel panic */
	unsigned int stop_mem_throttle;
	unsigned int start_mem_throttle;
};

static DEFINE_MUTEX(tmu_lock);

struct s5p_tmu_info {
	struct device *dev;

	char *s5p_name;
	struct s5p_tmu *ctz;
	struct temperature_set ts;

	struct delayed_work monitor_work;
	struct delayed_work polling_work;

	unsigned int monitor_period;
	unsigned int sampling_rate;

	struct resource *ioarea;
	int irq;
	unsigned int reg_save[TMU_SAVE_NUM];
	int tmu_status;
	unsigned int last_temperature;
	unsigned int cpufreq_level_1st;
	unsigned int cpufreq_level_2nd;
};
struct s5p_tmu_info *tmu_info;

#ifdef CONFIG_TMU_DEBUG

static int tmu_test_on;
static struct temperature_set in = {
#ifdef OPERATION_TEMP_BASE_61
	.stop_1st_throttle = (TEMP_1ST_TROTTLED_CELCIUS - 3),
#else
	.stop_1st_throttle = (TEMP_1ST_TROTTLED_CELCIUS - 4),
#endif
	.start_1st_throttle = TEMP_1ST_TROTTLED_CELCIUS,
	.stop_2nd_throttle = (TEMP_2ND_TROTTLED_CELCIUS - 6),
	.start_2nd_throttle = TEMP_2ND_TROTTLED_CELCIUS,
	.start_tripping = TEMP_TRIPPED_CELCIUS,
	.start_mem_throttle = TEMP_TQ0_CELCIUS,
};

static int set_sampling_rate;

static int tmu_limit_on;
static int upper_limit_1st = 2; /* default 800MHz */
static int upper_limit_2nd = 4; /* default 200MHz */

static int __init tmu_set_temperatures_param(char *str)
{
	unsigned int tmu_temp[7] = { (int)NULL, (int)NULL, (int)NULL,
		 (int)NULL, (int)NULL, (int)NULL, (int)NULL,};

	get_options(str, 7, tmu_temp);
	tmu_test_on = tmu_temp[0];
	printk(KERN_INFO "@@@tmu_test enable = %d\n", tmu_test_on);

	if (tmu_temp[1] > 0)
		in.stop_1st_throttle = tmu_temp[1];

	if (tmu_temp[2] > 0)
		in.start_1st_throttle = tmu_temp[2];

	if (tmu_temp[3] > 0)
		in.stop_2nd_throttle = tmu_temp[3];

	if (tmu_temp[4] > 0)
		in.start_2nd_throttle = tmu_temp[4];

	if (tmu_temp[5] > 0)
		in.start_tripping = tmu_temp[5];

	if (tmu_temp[6] > 0)
		in.start_mem_throttle = tmu_temp[6];

	/*  output the input value */
	pr_info("@@@ 1st throttling temp: start = %d, stop = %d,\n"
		"@@@ 2nd throttling temp: start = %d, stop = %d,\n"
		"@@@ trpping temp = %d, start_tq0 temp = %d\n",
		in.start_1st_throttle, in.stop_1st_throttle,
		in.start_2nd_throttle, in.stop_2nd_throttle,
		in.start_tripping, in.start_mem_throttle);

	return 0;
}
early_param("tmu_test", tmu_set_temperatures_param);

static int __init tmu_set_cpufreq_limit_param(char *str)
{
	int tmu_temp[3] = { (int)NULL, (int)NULL, (int)NULL};

	get_options(str, 3, tmu_temp);

	tmu_limit_on = tmu_temp[0];
	printk(KERN_INFO "@@@tmu_limit_on = %d\n", tmu_limit_on);

	if (tmu_temp[1] > 0)
		upper_limit_1st = tmu_temp[1];

	if (tmu_temp[2] > 0)
		upper_limit_2nd = tmu_temp[2];

	pr_info("@@@ 1st throttling : cpu_level = %d, 2nd cpu_level = %d\n",
		upper_limit_1st, upper_limit_2nd);

	return 0;
}
early_param("cpu_level", tmu_set_cpufreq_limit_param);

static int __init sampling_rate_param(char *str)
{
	get_option(&str, &set_sampling_rate);
	if (set_sampling_rate < 0)
		set_sampling_rate = 0;

	return 0;
}
early_param("tmu_sampling_rate", sampling_rate_param);

static void tmu_mon_timer(struct work_struct *work)
{
	unsigned char cur_temp_adc;
	unsigned int cur_temp;

	/* Compensation temperature */
	cur_temp_adc =
		__raw_readl(tmu_info->ctz->tmu_base + CURRENT_TEMP) & 0xff;
	cur_temp = cur_temp_adc - tmu_info->ctz->data.te1 + TMU_DC_VALUE;
	if (cur_temp < 25) {
		/* temperature code range is from 25 to 125 */
		pr_info("current temp is under 25 celsius degree!\n");
		cur_temp = 0;
	}

	pr_info("Current: %d c, 1st Throttling: stop = %d c, start = %d c\n,"
		"Warning(2nd Throttling): stop = %d c, start = %d c\n,"
		"Tripping = %d c, Mem throttle: stop = %d c, start = %d c\n",
		cur_temp,
		tmu_info->ts.stop_1st_throttle, tmu_info->ts.start_1st_throttle,
		tmu_info->ts.stop_2nd_throttle, tmu_info->ts.start_2nd_throttle,
		tmu_info->ts.start_tripping,
		tmu_info->ts.stop_mem_throttle, tmu_info->ts.start_mem_throttle);

	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->monitor_work,
			tmu_info->monitor_period);
}
#endif

static void print_temperature_set(void)
{
	pr_info("1st throttling stop_temp  = %d, start_temp = %d\n,"
		"2nd throttling stop_temp = %d, start_tmep = %d\n,"
		"tripping temp = %d, tripping retry_temp = %d\n"
		"memory throttling stop_temp = %d, start_temp = %d\n",
		tmu_info->ts.stop_1st_throttle,
		tmu_info->ts.start_1st_throttle,
		tmu_info->ts.stop_2nd_throttle,
		tmu_info->ts.start_2nd_throttle,
		tmu_info->ts.start_tripping,
		tmu_info->ts.retry_tripping,
		tmu_info->ts.stop_mem_throttle,
		tmu_info->ts.start_mem_throttle);
}

static void tmu_temperatures_init(void)
{
	/* Assign temperate_set to pre-defined value */
#ifdef OPERATION_TEMP_BASE_61
	tmu_info->ts.stop_1st_throttle = TEMP_1ST_TROTTLED_CELCIUS - 3;
#else
	tmu_info->ts.stop_1st_throttle = TEMP_1ST_TROTTLED_CELCIUS - 4;
#endif
	tmu_info->ts.start_1st_throttle = TEMP_1ST_TROTTLED_CELCIUS;
	tmu_info->ts.stop_2nd_throttle = TEMP_2ND_TROTTLED_CELCIUS - 6;
	tmu_info->ts.start_2nd_throttle = TEMP_2ND_TROTTLED_CELCIUS;
	tmu_info->ts.start_tripping =  TEMP_TRIPPED_CELCIUS;
	tmu_info->ts.retry_tripping = TEMP_TRIPPED_CELCIUS + 3;
	tmu_info->ts.start_emergency = TEMP_MAX_CELCIUS - 5;
	tmu_info->ts.stop_mem_throttle = TEMP_TQ0_CELCIUS - 5;
	tmu_info->ts.start_mem_throttle = TEMP_TQ0_CELCIUS;

#ifdef CONFIG_TMU_DEBUG
	if (tmu_test_on) {
		/* In the tmu_test mode, change temperature_set value
		 * input data.
		*/
		tmu_info->ts.stop_1st_throttle = in.stop_1st_throttle;
		tmu_info->ts.start_1st_throttle = in.start_1st_throttle;
		tmu_info->ts.stop_2nd_throttle = in.stop_2nd_throttle;
		tmu_info->ts.start_2nd_throttle = in.start_2nd_throttle;
		tmu_info->ts.start_tripping = in.start_tripping;
		tmu_info->ts.retry_tripping = in.start_tripping + 3;
		tmu_info->ts.stop_mem_throttle = in.start_mem_throttle - 5;
		tmu_info->ts.start_mem_throttle = in.start_mem_throttle;
	}
#endif
	print_temperature_set();

}

static int notify_change_of_tmu_state(struct s5p_tmu_info *info)
{
	char temp_buf[20];
	char *envp[2];
	int env_offset = 0;

	snprintf(temp_buf, sizeof(temp_buf), "TMUSTATE=%d",
			info->ctz->data.tmu_flag);
	envp[env_offset++] = temp_buf;
	envp[env_offset] = NULL;

	pr_info("%s: uevent: %d, name = %s\n",
			__func__, info->ctz->data.tmu_flag, temp_buf);

	return kobject_uevent_env(&info->dev->kobj, KOBJ_CHANGE, envp);
}

static void tmu_poll_timer(struct work_struct *work)
{
	unsigned int cur_temp;
	static int auto_refresh_changed;
	static int check_handle;
	int trend = 0;
	struct s5p_tmu_info *info =
		container_of(work, struct s5p_tmu_info, polling_work.work);

	mutex_lock(&tmu_lock);

	/* Compensation temperature */
	cur_temp = (__raw_readl(info->ctz->tmu_base + CURRENT_TEMP) & 0xff)
			- info->ctz->data.te1 + TMU_DC_VALUE;
	if (cur_temp < 25) {
		/* temperature code range is between min 25 and 125 */
		pr_info("current temp is under 25 celsius degree!\n");
		cur_temp = 0;
	}
	trend = cur_temp - info->last_temperature;
	pr_info("cur_temp = %d, tmu_state = %d, temp_diff = %d\n",
			cur_temp, info->ctz->data.tmu_flag, trend);

	switch (info->ctz->data.tmu_flag) {
	case TMU_STATUS_NORMAL:
		/* 1. change state: 1st-throttling */
		if (cur_temp >= info->ts.start_1st_throttle) {
			info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
			pr_info("change state: normal->throttle.\n");
		/* 2. polling end and uevent */
		} else if ((cur_temp <= info->ts.stop_1st_throttle)
			&& (trend < 0)) {
			if (check_handle & THROTTLE_FLAG) {
				s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
				check_handle &= ~(THROTTLE_FLAG);
			}
			pr_info("check_handle = %d\n", check_handle);
			notify_change_of_tmu_state(info);
			pr_info("normal: free cpufreq_limit & interrupt enable.\n");

			/* clear to prevent from interfupt by peindig bit */
			__raw_writel(INTCLEARALL,
				info->ctz->tmu_base + INTCLEAR);
			enable_irq(info->irq);
			mutex_unlock(&tmu_lock);
			return;
		}
		break;

	case TMU_STATUS_THROTTLED:
		/* 1. change state: 2nd-throttling or warning */
		if (cur_temp >= info->ts.start_2nd_throttle) {
			info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
			pr_info("change state: 1st throttle->2nd throttle.\n");
		/* 2. cpufreq limitation and uevent */
		} else if ((cur_temp >= info->ts.start_1st_throttle) &&
			(trend > 0) && !(check_handle & THROTTLE_FLAG)) {
			if (check_handle & WARNING_FLAG) {
				s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
				check_handle &= ~(WARNING_FLAG);
			}
			pr_info("before check_handle = %d\n", check_handle);
#ifdef CONFIG_TMU_DEBUG
			if (tmu_limit_on)
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
					upper_limit_1st);
			else
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
					info->cpufreq_level_1st);
#else
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
					info->cpufreq_level_1st);
#endif
			check_handle |= THROTTLE_FLAG;
			pr_info("after check_handle = %d\n", check_handle);
			notify_change_of_tmu_state(info);
			pr_info("throttling: set cpufreq upper limit.\n");
		/* 3. change state: normal */
		} else if ((cur_temp <= info->ts.stop_1st_throttle)
			&& (trend < 0)) {
			info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;
			pr_info("change state: 1st throttle->normal.\n");
		}
		break;

	case TMU_STATUS_WARNING:
		/* 1. change state: tripping */
		if (cur_temp >= info->ts.start_tripping) {
			info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
			pr_info("change state: 2nd throttle->trip\n");
		/* 2. cpufreq limitation and uevent */
		} else if ((cur_temp >= info->ts.start_2nd_throttle)
			&& (trend > 0) && !(check_handle & WARNING_FLAG)) {
			if (check_handle & THROTTLE_FLAG) {
				s5pv310_cpufreq_upper_limit_free(DVFS_LOCK_ID_TMU);
				check_handle &= ~(THROTTLE_FLAG);
			}
			pr_info("before check_handle = %d\n", check_handle);
#ifdef CONFIG_TMU_DEBUG
			if (tmu_limit_on)
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
					upper_limit_2nd);
			else
				s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
					info->cpufreq_level_2nd);
#else
			s5pv310_cpufreq_upper_limit(DVFS_LOCK_ID_TMU,
					info->cpufreq_level_2nd);
#endif
			check_handle |= WARNING_FLAG;
			pr_info("after check_handle = %d\n", check_handle);
			notify_change_of_tmu_state(info);
			pr_info("2nd throttle: cpufreq is limited.\n");
		/* 3. change state: 1st-throttling */
		} else if ((cur_temp <= info->ts.stop_2nd_throttle)
			&& (trend < 0)) {
			info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
			pr_info("change state: 2nd throttle->1st throttle, "
				"and release cpufreq upper limit.\n");
		}
		break;

	case TMU_STATUS_TRIPPED:
		/* 1. call uevent to shut-down */
		if ((cur_temp >= info->ts.start_tripping) &&
			(trend > 0) && !(check_handle & TRIPPING_FLAG)) {
			notify_change_of_tmu_state(info);
			pr_info("tripping: on waiting shutdown.\n");
			check_handle |= TRIPPING_FLAG;
			pr_info("check_handle = %d\n", check_handle);
		/* 2. change state: 2nd-throttling or warning */
		} else if ((cur_temp <= info->ts.stop_2nd_throttle)
				&& (trend < 0)) {
			info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
			pr_info("change state: trip->2nd throttle, "
				"Check! occured only test mode.\n");
		}

		/* 3. chip protection: kernel panic as SW workaround */
		if ((cur_temp >= info->ts.start_emergency) && (trend > 0)) {
			panic("Emergency!!!!"
				"tmu tripping event is not treated!\n");
			/* clear to prevent from interfupt by peindig bit */
			__raw_writel(INTCLEARALL,
				info->ctz->tmu_base + INTCLEAR);
			enable_irq(info->irq);
			mutex_unlock(&tmu_lock);
			return;
		/* 4. chip protection: retry as SW workaround */
		} else if ((cur_temp >= info->ts.retry_tripping) && (trend > 0)
			&& (check_handle & TRIPPING_FLAG)) {
			if (!(check_handle & RETRY_FLAG)) {
				pr_warn("WARNING: try to send uevent"
					"to platform again.\n");
				check_handle &= ~(TRIPPING_FLAG);
				check_handle |= RETRY_FLAG;
			}
		}
		break;

	case TMU_STATUS_INIT:
		/* sned tmu initial status to platform */
		disable_irq(info->irq);
		if (cur_temp >= info->ts.start_tripping)
			info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
		else if (cur_temp >= info->ts.start_2nd_throttle)
			info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
		else if (cur_temp >= info->ts.start_1st_throttle)
			info->ctz->data.tmu_flag = TMU_STATUS_THROTTLED;
		else if (cur_temp <= info->ts.stop_1st_throttle)
			info->ctz->data.tmu_flag = TMU_STATUS_NORMAL;

		notify_change_of_tmu_state(info);
		pr_info("%s: inform to init state to platform.\n", __func__);
		break;

	default:
		pr_warn("Bug: checked tmu_state.\n");
		if (cur_temp >= info->ts.start_tripping)
			info->ctz->data.tmu_flag = TMU_STATUS_TRIPPED;
		else
			info->ctz->data.tmu_flag = TMU_STATUS_WARNING;
		break;
	} /* end */

	if (cur_temp >= info->ts.start_mem_throttle) {  /* 85 */
		if (!(auto_refresh_changed) && (trend > 0)) {
			pr_info("set auto_refresh 1.95us\n");
			set_refresh_rate(AUTO_REFRESH_PERIOD_TQ0);
			auto_refresh_changed = 1;
		}
	} else if (cur_temp <= (info->ts.stop_mem_throttle)) {/* 80 */
		if ((auto_refresh_changed) && (trend < 0)) {
			pr_info("set auto_refresh 3.9us\n");
			set_refresh_rate(AUTO_REFRESH_PERIOD_NORMAL);
			auto_refresh_changed = 0;
		}
	}

	info->last_temperature = cur_temp;
	/* rescheduling next work */
	queue_delayed_work_on(0, tmu_monitor_wq, &info->polling_work,
			info->sampling_rate);

	mutex_unlock(&tmu_lock);

	return;
}

static int tmu_initialize(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	unsigned int en;
	unsigned int te_temp;

	__raw_writel(INTCLEAR2, tz->tmu_base + INTCLEAR);

	en = (__raw_readl(tz->tmu_base + TMU_STATUS) & 0x1);

	if (!en) {
		dev_err(&pdev->dev, "failed to start tmu drvier\n");
		return -ENOENT;
	}

	/* get the compensation parameter */
	te_temp = __raw_readl(tz->tmu_base + TRIMINFO);
	tz->data.te1 = te_temp & TRIM_TEMP_MASK;
	tz->data.te2 = ((te_temp >> 8) & TRIM_TEMP_MASK);

	pr_info("%s: te_temp = 0x%08x, low 8bit = %d, high 24 bit = %d\n",
			__func__, te_temp, tz->data.te1, tz->data.te2);

	if ((EFUSE_MIN_VALUE > tz->data.te1) || (tz->data.te1 > EFUSE_MAX_VALUE)
		||  (tz->data.te2 != 0))
		tz->data.te1 = EFUSE_AVG_VALUE;

	/* Need to initial regsiter setting after getting parameter info */
	/* [28:23] vref [11:8] slope - Tunning parameter */
	__raw_writel(VREF_SLOPE, tz->tmu_base + TMU_CON0);

	return 0;
}

static void tmu_start(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	unsigned int con;
	unsigned int thresh_temp_adc;
	unsigned int thr_temp_adc, warn_temp_adc, trip_temp_adc;

	__raw_writel(INTCLEARALL, tz->tmu_base + INTCLEAR);

	/* Compensation temperature THD_TEMP */
	thresh_temp_adc	= tmu_info->ts.stop_1st_throttle
			+ tz->data.te1 - TMU_DC_VALUE;
	thr_temp_adc = tmu_info->ts.start_1st_throttle
			- tmu_info->ts.stop_1st_throttle;
	warn_temp_adc = tmu_info->ts.start_2nd_throttle
			- tmu_info->ts.stop_1st_throttle;
	trip_temp_adc = tmu_info->ts.start_tripping
			- tmu_info->ts.stop_1st_throttle;

	/* Set interrupt trigger level */
	__raw_writel(thresh_temp_adc, tz->tmu_base + THRESHOLD_TEMP);
	__raw_writel(thr_temp_adc, tz->tmu_base + TRG_LEV0);
	__raw_writel(warn_temp_adc, tz->tmu_base + TRG_LEV1);
	__raw_writel(trip_temp_adc, tz->tmu_base + TRG_LEV2);
	__raw_writel(TRIGGER_LEV3, tz->tmu_base + TRG_LEV3);

	pr_info("Cooling: %dc  THD_TEMP:0x%02x:  TRIG_LEV0: 0x%02x"
		"TRIG_LEV1: 0x%02x TRIG_LEV2: 0x%02x, TRIG_LEV3: 0x%02x\n",
		tmu_info->ts.stop_1st_throttle,
		__raw_readl(tz->tmu_base + THRESHOLD_TEMP),
		__raw_readl(tz->tmu_base + TRG_LEV0),
		__raw_readl(tz->tmu_base + TRG_LEV1),
		__raw_readl(tz->tmu_base + TRG_LEV2),
		__raw_readl(tz->tmu_base + TRG_LEV3));

	mdelay(50);

	/* TMU core enable */
	con = __raw_readl(tz->tmu_base + TMU_CON0);
	con |= TMU_EN;
	__raw_writel(con, tz->tmu_base + TMU_CON0);

	/* LEV0 LEV1 LEV2 interrupt enable */
	__raw_writel(INTEN0 | INTEN1 | INTEN2, tz->tmu_base + INTEN);

	return;
}

static irqreturn_t s5p_tmu_irq(int irq, void *id)
{
	struct s5p_tmu *tz = id;
	unsigned int status;

	disable_irq_nosync(irq);

	status = __raw_readl(tz->tmu_base + INTSTAT);

	pr_info("TMU interrupt occured : status = 0x%08x\n", status);

	if (status & INTSTAT2) {
		tz->data.tmu_flag = TMU_STATUS_TRIPPED;
		__raw_writel(INTCLEAR2, tz->tmu_base + INTCLEAR);

	} else if (status & INTSTAT1) {
		tz->data.tmu_flag = TMU_STATUS_WARNING;
		__raw_writel(INTCLEAR1, tz->tmu_base + INTCLEAR);

	} else if (status & INTSTAT0) {
		tz->data.tmu_flag = TMU_STATUS_THROTTLED;
		__raw_writel(INTCLEAR0, tz->tmu_base + INTCLEAR);

	} else {
		pr_err("%s: TMU interrupt error\n", __func__);
		__raw_writel(INTCLEARALL, tz->tmu_base + INTCLEAR);
		queue_delayed_work_on(0, tmu_monitor_wq,
			&tmu_info->polling_work, tmu_info->sampling_rate / 2);
		return -ENODEV;
	}

	/* read current temperature & save */
	tmu_info->last_temperature =  (__raw_readl(tmu_info->ctz->tmu_base
		+ CURRENT_TEMP) & 0xff) - tmu_info->ctz->data.te1
		+ TMU_DC_VALUE;

	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->polling_work,
		tmu_info->sampling_rate);

	return IRQ_HANDLED;
}

static int __devinit s5p_tmu_probe(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);
	struct resource *res;
	int ret = 0;

	pr_debug("%s: probe=%p\n", __func__, pdev);

	tmu_info = kzalloc(sizeof(struct s5p_tmu_info), GFP_KERNEL);
	if (!tmu_info) {
		dev_err(&pdev->dev, "failed to alloc memory!\n");
		ret = -ENOMEM;
		goto err_nomem;
	}
	tmu_info->dev = &pdev->dev;
	tmu_info->ctz = tz;
	tmu_info->ctz->data.tmu_flag = TMU_STATUS_INIT;
	tmu_info->cpufreq_level_1st = s5pv310_cpufreq_round_idx(CPUFREQ_800MHZ);
	tmu_info->cpufreq_level_2nd = s5pv310_cpufreq_round_idx(CPUFREQ_200MHZ);
	tmu_temperatures_init();

	/* To poll current temp, set sampling rate to ONE second sampling */
	tmu_info->sampling_rate  = usecs_to_jiffies(1000 * 1000);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get memory region resource\n");
		ret = -ENODEV;
		goto err_nores;
	}

	tmu_info->ioarea = request_mem_region(res->start,
			res->end-res->start + 1, pdev->name);
	if (!(tmu_info->ioarea)) {
		dev_err(&pdev->dev, "failed to reserve memory region\n");
		ret = -EBUSY;
		goto err_nores;
	}

	tz->tmu_base = ioremap(res->start, (res->end - res->start) + 1);
	if (!(tz->tmu_base)) {
		dev_err(&pdev->dev, "failed ioremap()\n");
		ret = -EINVAL;
		goto err_nomap;
	}

	tmu_monitor_wq = create_freezeable_workqueue(dev_name(&pdev->dev));
	if (!tmu_monitor_wq) {
		pr_info("Creation of tmu_monitor_wq failed\n");
		return -EFAULT;
	}

#ifdef CONFIG_TMU_DEBUG
	if (set_sampling_rate) {
		tmu_info->sampling_rate =
			usecs_to_jiffies(set_sampling_rate * 1000);
		tmu_info->monitor_period =
			usecs_to_jiffies(set_sampling_rate * 10 * 1000);
	} else {
		/* 50sec monitroing */
		tmu_info->monitor_period = usecs_to_jiffies(10000 * 1000);
	}

	INIT_DELAYED_WORK_DEFERRABLE(&tmu_info->monitor_work, tmu_mon_timer);
	queue_delayed_work_on(0, tmu_monitor_wq, &tmu_info->monitor_work,
			tmu_info->monitor_period);
#endif

	INIT_DELAYED_WORK_DEFERRABLE(&tmu_info->polling_work, tmu_poll_timer);

	tmu_info->irq = platform_get_irq(pdev, 0);
	if (tmu_info->irq < 0) {
		dev_err(&pdev->dev, "no irq for thermal\n");
		ret = tmu_info->irq;
		goto err_irq;
	}

	ret = request_irq(tmu_info->irq, s5p_tmu_irq,
			IRQF_DISABLED,  "s5p-tmu interrupt", tz);
	if (ret) {
		dev_err(&pdev->dev, "IRQ%d error %d\n", tmu_info->irq, ret);
		goto err_irq;
	}

	ret = tmu_initialize(pdev);
	if (ret)
		goto err_init;

	tmu_start(pdev);

	return ret;

err_init:
	if (tmu_info->irq >= 0)
		free_irq(tmu_info->irq, tz);

err_irq:
	iounmap(tz->tmu_base);

err_nomap:
	release_resource(tmu_info->ioarea);
	kfree(tmu_info->ioarea);

err_nores:
	kfree(tmu_info);
	tmu_info = NULL;

err_nomem:
	dev_err(&pdev->dev, "initialization failed.\n");

	return ret;
}

static int __devinit s5p_tmu_remove(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);

	cancel_delayed_work(&tmu_info->polling_work);

	if (tmu_info->irq >= 0)
		free_irq(tmu_info->irq, tz);

	iounmap(tz->tmu_base);

	release_resource(tmu_info->ioarea);
	kfree(tmu_info->ioarea);

	kfree(tmu_info);
	tmu_info = NULL;

	pr_info("%s is removed\n", dev_name(&pdev->dev));
	return 0;
}

#ifdef CONFIG_PM
static int s5p_tmu_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);

	/* save tmu register value */
	tmu_info->reg_save[0] = __raw_readl(tz->tmu_base + TMU_CON0);
	tmu_info->reg_save[1] = __raw_readl(tz->tmu_base + SAMPLING_INTERNAL);
	tmu_info->reg_save[2] = __raw_readl(tz->tmu_base + CNT_VALUE0);
	tmu_info->reg_save[3] = __raw_readl(tz->tmu_base + CNT_VALUE1);
	tmu_info->reg_save[4] = __raw_readl(tz->tmu_base + THRESHOLD_TEMP);
	tmu_info->reg_save[5] = __raw_readl(tz->tmu_base + INTEN);
	tmu_info->reg_save[6] = __raw_readl(tz->tmu_base + TRG_LEV0);
	tmu_info->reg_save[7] = __raw_readl(tz->tmu_base + TRG_LEV1);
	tmu_info->reg_save[8] = __raw_readl(tz->tmu_base + TRG_LEV2);
	tmu_info->reg_save[9] = __raw_readl(tz->tmu_base + TRG_LEV3);

	disable_irq(tmu_info->irq);

	return 0;
}

static int s5p_tmu_resume(struct platform_device *pdev)
{
	struct s5p_tmu *tz = platform_get_drvdata(pdev);

	/* save tmu register value */
	__raw_writel(tmu_info->reg_save[0], tz->tmu_base + TMU_CON0);
	__raw_writel(tmu_info->reg_save[1], tz->tmu_base + SAMPLING_INTERNAL);
	__raw_writel(tmu_info->reg_save[2], tz->tmu_base + CNT_VALUE0);
	__raw_writel(tmu_info->reg_save[3], tz->tmu_base + CNT_VALUE1);
	__raw_writel(tmu_info->reg_save[4], tz->tmu_base + THRESHOLD_TEMP);
	__raw_writel(tmu_info->reg_save[5], tz->tmu_base + INTEN);
	__raw_writel(tmu_info->reg_save[6], tz->tmu_base + TRG_LEV0);
	__raw_writel(tmu_info->reg_save[7], tz->tmu_base + TRG_LEV1);
	__raw_writel(tmu_info->reg_save[8], tz->tmu_base + TRG_LEV2);
	__raw_writel(tmu_info->reg_save[9], tz->tmu_base + TRG_LEV3);

	enable_irq(tmu_info->irq);

	return 0;
}
#else
#define s5p_tmu_suspend	NULL
#define s5p_tmu_resume	NULL
#endif

static struct platform_driver s5p_tmu_driver = {
	.probe		= s5p_tmu_probe,
	.remove		= s5p_tmu_remove,
	.suspend	= s5p_tmu_suspend,
	.resume		= s5p_tmu_resume,
	.driver		= {
		.name   = "s5p-tmu",
		.owner  = THIS_MODULE,
	},
};

static int __init s5p_tmu_driver_init(void)
{
	return platform_driver_register(&s5p_tmu_driver);
}

late_initcall(s5p_tmu_driver_init);
