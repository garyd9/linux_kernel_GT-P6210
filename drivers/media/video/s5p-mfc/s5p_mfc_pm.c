/*
 * linux/drivers/media/video/s5p-mfc/s5p_mfc_pm.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/clk.h>

#include "s5p_mfc_common.h"
#include "s5p_mfc_debug.h"
#include "s5p_mfc_pm.h"

#if defined(CONFIG_ARCH_S5PV210)
int s5p_mfc_init_pm(struct mfc_dev *mfcdev)
{
	return -1;
}

void s5p_mfc_final_pm(struct mfc_dev *mfcdev)
{
	/* NOP */
}

int s5p_mfc_clock_on(void)
{
	return -1;
}

void s5p_mfc_clock_off(void)
{
	/* NOP */
}

int s5p_mfc_power_on(void)
{
	return -1;
}

int s5p_mfc_power_off(void)
{
	return -1;
}
#elif defined(CONFIG_ARCH_S5PV310)
#include <linux/platform_device.h>
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#define MFC_PARENT_CLK_NAME	"mout_mfc0"
#define MFC_CLKNAME		"sclk_mfc"
#define MFC_GATE_CLK_NAME	"mfc"

#define CLK_DEBUG

static struct s5p_mfc_pm *pm;

#ifdef CLK_DEBUG
atomic_t clk_ref;
#endif

int s5p_mfc_init_pm(struct s5p_mfc_dev *dev)
{
	struct clk *parent, *sclk;
	int ret = 0;

	pm = &dev->pm;

	/* FIXME : move to platform resource NAME */
	parent = clk_get(&dev->plat_dev->dev, MFC_PARENT_CLK_NAME);
	if (IS_ERR(parent)) {
		printk(KERN_ERR "failed to get parent clock\n");
		ret = -ENOENT;
		goto err_p_clk;
	}

	/* FIXME : move to platform resource NAME */
	sclk = clk_get(&dev->plat_dev->dev, MFC_CLKNAME);
	if (IS_ERR(sclk)) {
		printk(KERN_ERR "failed to get source clock\n");
		ret = -ENOENT;
		goto err_s_clk;
	}

	clk_set_parent(sclk, parent);
	/* FIXME : move to platform resource RATE */
	clk_set_rate(sclk, 200 * 1000000);

	/* FIXME : move to platform resource NAME */
	/* clock for gating */
	pm->clock = clk_get(&dev->plat_dev->dev, MFC_GATE_CLK_NAME);
	if (IS_ERR(pm->clock)) {
		printk(KERN_ERR "failed to get clock-gating control\n");
		ret = -ENOENT;
		goto err_g_clk;
	}

	atomic_set(&pm->power, 0);

#ifdef CONFIG_PM_RUNTIME
	pm->device = &dev->plat_dev->dev;

	pm_runtime_enable(pm->device);
#endif

#ifdef CLK_DEBUG
	atomic_set(&clk_ref, 0);
#endif

	return 0;

err_g_clk:
	clk_put(sclk);
err_s_clk:
	clk_put(parent);
err_p_clk:
	return ret;
}

void s5p_mfc_final_pm(struct s5p_mfc_dev *dev)
{
	clk_put(pm->clock);

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(pm->device);
#endif
}

int s5p_mfc_clock_on(void)
{
#ifdef CLK_DEBUG
	atomic_inc(&clk_ref);
	mfc_debug(3, "+ %d", atomic_read(&clk_ref));
#endif

#ifdef CONFIG_PM_RUNTIME
	return clk_enable(pm->clock);
#else
	return clk_enable(pm->clock);
#endif
}

void s5p_mfc_clock_off(void)
{
#ifdef CLK_DEBUG
	atomic_dec(&clk_ref);
	mfc_debug(3, "- %d", atomic_read(&clk_ref));
#endif

#ifdef CONFIG_PM_RUNTIME
	clk_disable(pm->clock);
#else
	clk_disable(pm->clock);
#endif
}

int s5p_mfc_power_on(void)
{
#ifdef CONFIG_PM_RUNTIME
	return pm_runtime_get_sync(pm->device);
#else
	atomic_set(&pm->power, 1);

	return 0;
#endif
}

int s5p_mfc_power_off(void)
{
#ifdef CONFIG_PM_RUNTIME
	return pm_runtime_put_sync(pm->device);
#else
	atomic_set(&pm->power, 0);

	return 0;
#endif
}

bool s5p_mfc_power_chk(void)
{
	mfc_debug(2, "%s", atomic_read(&pm->power) ? "on" : "off");

	return atomic_read(&pm->power) ? true : false;
}
#else /* CONFIG_ARCH_NOT_SUPPORT */
int s5p_mfc_init_pm(struct mfc_dev *mfcdev)
{
	return -1;
}

void s5p_mfc_final_pm(struct mfc_dev *mfcdev)
{
	/* NOP */
}

int s5p_mfc_clock_on(void)
{
	return -1;
}

void s5p_mfc_clock_off(void)
{
	/* NOP */
}

int s5p_mfc_power_on(void)
{
	return -1;
}

int s5p_mfc_power_off(void)
{
	return -1;
}
#endif
