/*
 * linux/arch/arm/mach-s5pv210/dev-pmu.c
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * S5PV210 - PMU IRQ registration
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/platform_device.h>
#include <asm/pmu.h>
#include <mach/irqs.h>

static struct resource pmu_resource = {
	.start	= IRQ_PMU,
	.end	= IRQ_PMU,
	.flags	= IORESOURCE_IRQ,
};

struct platform_device pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource	= &pmu_resource,
	.num_resources	= 1,
};

static int __init s5pv210_pmu_init(void)
{
	platform_device_register(&pmu_device);
	return 0;
}
arch_initcall(s5pv210_pmu_init);
