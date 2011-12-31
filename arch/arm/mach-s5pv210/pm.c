/* linux/arch/arm/mach-s5pv210/pm.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * S5PV210 - Power Management support
 *
 * Based on arch/arm/mach-s3c2410/pm.c
 * Copyright (c) 2006 Simtec Electronics
 *	Ben Dooks <ben@simtec.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/regs-timer.h>
#include <plat/gpio-cfg.h>

#include <mach/regs-irq.h>
#include <mach/regs-clock.h>

static struct sleep_save s5pv210_core_save[] = {
	/* Clock source */
	SAVE_ITEM(S5P_CLK_SRC0),
	SAVE_ITEM(S5P_CLK_SRC1),
	SAVE_ITEM(S5P_CLK_SRC2),
	SAVE_ITEM(S5P_CLK_SRC3),
	SAVE_ITEM(S5P_CLK_SRC4),
	SAVE_ITEM(S5P_CLK_SRC5),
	SAVE_ITEM(S5P_CLK_SRC6),

	/* Clock source Mask */
	SAVE_ITEM(S5P_CLK_SRC_MASK0),
	SAVE_ITEM(S5P_CLK_SRC_MASK1),

	/* Clock Divider */
	SAVE_ITEM(S5P_CLK_DIV0),
	SAVE_ITEM(S5P_CLK_DIV1),
	SAVE_ITEM(S5P_CLK_DIV2),
	SAVE_ITEM(S5P_CLK_DIV3),
	SAVE_ITEM(S5P_CLK_DIV4),
	SAVE_ITEM(S5P_CLK_DIV5),
	SAVE_ITEM(S5P_CLK_DIV6),
	SAVE_ITEM(S5P_CLK_DIV7),

	/* Clock Main Gate */
	SAVE_ITEM(S5P_CLKGATE_MAIN0),
	SAVE_ITEM(S5P_CLKGATE_MAIN1),
	SAVE_ITEM(S5P_CLKGATE_MAIN2),

	/* Clock source Peri Gate */
	SAVE_ITEM(S5P_CLKGATE_PERI0),
	SAVE_ITEM(S5P_CLKGATE_PERI1),

	/* Clock source SCLK Gate */
	SAVE_ITEM(S5P_CLKGATE_SCLK0),
	SAVE_ITEM(S5P_CLKGATE_SCLK1),

	/* Clock IP Clock gate */
	SAVE_ITEM(S5P_CLKGATE_IP0),
	SAVE_ITEM(S5P_CLKGATE_IP1),
	SAVE_ITEM(S5P_CLKGATE_IP2),
	SAVE_ITEM(S5P_CLKGATE_IP3),
	SAVE_ITEM(S5P_CLKGATE_IP4),

	/* Clock Blcok and Bus gate */
	SAVE_ITEM(S5P_CLKGATE_BLOCK),
	SAVE_ITEM(S5P_CLKGATE_BUS0),

	/* Clock ETC */
	SAVE_ITEM(S5P_CLK_OUT),
	SAVE_ITEM(S5P_MDNIE_SEL),

	/* PWM Register */
	SAVE_ITEM(S3C2410_TCFG0),
	SAVE_ITEM(S3C2410_TCFG1),
	SAVE_ITEM(S3C64XX_TINT_CSTAT),
	SAVE_ITEM(S3C2410_TCON),
	SAVE_ITEM(S3C2410_TCNTB(0)),
	SAVE_ITEM(S3C2410_TCMPB(0)),
	SAVE_ITEM(S3C2410_TCNTO(0)),
};

void s5pv210_cpu_suspend(void)
{
	unsigned long tmp;

	/* issue the standby signal into the pm unit. Note, we
	 * issue a write-buffer drain just in case */

	tmp = 0;

	volatile unsigned int gpio;
        volatile unsigned int cfg;
        volatile unsigned int sfr;

	/* Read OM pin */
	cfg = __raw_readl(S5P_VA_CHIPID + 0x4);
	cfg &= ~0xFFFFFFC1;

	/* If eMMC booting */
	if ((cfg == 0xE) || (cfg == 0x16)) {

		/* Card power off */
		sfr = __raw_readl(S5P_VA_GPIO + 0x2E4);
		sfr &= ~(0x1 << 3);
		__raw_writel(sfr, (S5P_VA_GPIO + 0x2E4));

		sfr = __raw_readl(S5P_VA_GPIO + 0x2E0);
		sfr &= ~(0xF << 12);
		sfr |= (0x1 << 12);
		__raw_writel(sfr, (S5P_VA_GPIO + 0x2E0));
		
		/* Set power down mode */
		sfr = __raw_readl(S5P_VA_GPIO + 0x2F0);
		sfr &= ~(0x3 << 6);
		__raw_writel(sfr, (S5P_VA_GPIO + 0x2F0));


		/* MMC CH0 all GPIO set output0 */
		for (gpio = S5PV210_GPG0(0); gpio <= S5PV210_GPG0(6); gpio++) {
			if (!gpio_request(gpio, "SD_0")) {
				gpio_direction_output(gpio, 0);
				s3c_gpio_cfgpin(gpio, S3C_GPIO_OUTPUT);
				
				/* MMC CH0 all GPIO set output0 at powerdown mode */
				s5p_gpio_set_conpdn(gpio, S5P_GPIO_OUTPUT0);
				s5p_gpio_set_pudpdn(gpio, S5P_GPIO_PULL_UP_DOWN_DISABLE);
				gpio_free(gpio);
			}
		}

		/* early wakeup defensive delay */
		mdelay(10);
	}

	asm("b 1f\n\t"
	    ".align 5\n\t"
	    "1:\n\t"
	    "mcr p15, 0, %0, c7, c10, 5\n\t"
	    "mcr p15, 0, %0, c7, c10, 4\n\t"
	    "wfi" : : "r" (tmp));

	/* we should never get past here */
	panic("sleep resumed to originator?");
}

static void s5pv210_pm_prepare(void)
{
	unsigned int tmp;

	/* ensure at least INFORM0 has the resume address */
	__raw_writel(virt_to_phys(s3c_cpu_resume), S5P_INFORM0);

	tmp = __raw_readl(S5P_SLEEP_CFG);
	tmp &= ~(S5P_SLEEP_CFG_OSC_EN | S5P_SLEEP_CFG_USBOSC_EN);
	__raw_writel(tmp, S5P_SLEEP_CFG);

	/* WFI for SLEEP mode configuration by SYSCON */
	tmp = __raw_readl(S5P_PWR_CFG);
	tmp &= S5P_CFG_WFI_CLEAN;
	tmp |= S5P_CFG_WFI_SLEEP;
	__raw_writel(tmp, S5P_PWR_CFG);

	/* SYSCON interrupt handling disable */
	tmp = __raw_readl(S5P_OTHERS);
	tmp |= S5P_OTHER_SYSC_INTOFF;
	__raw_writel(tmp, S5P_OTHERS);

	s3c_pm_do_save(s5pv210_core_save, ARRAY_SIZE(s5pv210_core_save));
}

static int s5pv210_pm_begin(suspend_state_t state)
{
	int ret = 0;
#if defined(CONFIG_REGULATOR)
	ret = regulator_suspend_prepare(PM_SUSPEND_MEM);

#endif
	return ret;
}

static void s5pv210_pm_end(void)
{
	/* To be implemented */
}

static int s5pv210_pm_add(struct sys_device *sysdev)
{
	pm_cpu_prep = s5pv210_pm_prepare;
	pm_cpu_sleep = s5pv210_cpu_suspend;
	pm_begin = s5pv210_pm_begin;
	pm_end = s5pv210_pm_end;

	return 0;
}

static int s5pv210_pm_resume(struct sys_device *dev)
{
	u32 tmp;

	tmp = __raw_readl(S5P_OTHERS);
	tmp |= (S5P_OTHERS_RET_IO | S5P_OTHERS_RET_CF |\
		S5P_OTHERS_RET_MMC | S5P_OTHERS_RET_UART);
	__raw_writel(tmp , S5P_OTHERS);

	s3c_pm_do_restore_core(s5pv210_core_save, ARRAY_SIZE(s5pv210_core_save));

	/*
	 * Enable clock for UART ch-2 to which was enabled by IROM bootup code
	 * handle irq and clear pending bit after resume from sleep.
	 */
	tmp = __raw_readl(S5P_CLKGATE_IP3);
	tmp |= (0x1 << 19);
	__raw_writel(tmp, S5P_CLKGATE_IP3);

	return 0;
}

static struct sysdev_driver s5pv210_pm_driver = {
	.add		= s5pv210_pm_add,
	.resume		= s5pv210_pm_resume,
};

static __init int s5pv210_pm_drvinit(void)
{
	return sysdev_driver_register(&s5pv210_sysclass, &s5pv210_pm_driver);
}
arch_initcall(s5pv210_pm_drvinit);
