/* linux/arch/arm/mach-s5pv210/setup-tvout.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Base TVOUT gpio configuration
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <plat/clock.h>
#include <plat/gpio-cfg.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <linux/io.h>
#include <mach/map.h>
#include <mach/gpio.h>
#include <mach/gpio-bank.h>
//#include <mach/regs-gpio.h>

struct platform_device; /* don't need the contents */

void s5p_int_src_hdmi_hpd(struct platform_device *pdev)
{
	s3c_gpio_cfgpin(S5PV210_GPH1(5), S3C_GPIO_SFN(0x4));
	s3c_gpio_setpull(S5PV210_GPH1(5), S3C_GPIO_PULL_DOWN);
	writel(readl(S5PV210_GPH1DRV)|0x3<<10, S5PV210_GPH1DRV);
}

void s5p_int_src_ext_hpd(struct platform_device *pdev)
{
	s3c_gpio_cfgpin(S5PV210_GPH1(5), S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(S5PV210_GPH1(5), S3C_GPIO_PULL_DOWN);
	writel(readl(S5PV210_GPH1DRV)|0x3<<10, S5PV210_GPH1DRV);
}

int s5p_hpd_read_gpio(struct platform_device *pdev)
{
	return gpio_get_value(S5PV210_GPH1(5));
}

void s5p_cec_cfg_gpio(struct platform_device *pdev)
{
	s3c_gpio_cfgpin(S5PV210_GPH1(4), S3C_GPIO_SFN(0x4));
	s3c_gpio_setpull(S5PV210_GPH1(4), S3C_GPIO_PULL_NONE);
}
