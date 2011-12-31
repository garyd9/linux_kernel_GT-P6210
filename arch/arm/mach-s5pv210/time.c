/* arch/arm/mach-s5pv210/time.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/clockchips.h>

#include <asm/mach/time.h>

#include <plat/map-s5p.h>
#include <plat/regs-rtc.h>

#include <mach/regs-systimer.h>
#include <mach/map.h>

#define USE_SYSTIMER_IRQ

/* sched_timer_running
 * 0 : sched timer stopped or not initialized
 * 1 : sched timer started
 */
static unsigned int sched_timer_running;

void __iomem *rtc_base = S5P_VA_RTC;
static struct clk *clk_event;
static struct clk *clk_sched;
static int tick_timer_mode;	/* 0: oneshot, 1: autoreload */

/*
 * Helper functions
 * s5pv210_systimer_read() : Read from System timer register
 * s5pv210_systimer_write(): Write to System timer register
 *
 */
static unsigned int s5pv210_systimer_read(unsigned int *reg_offset)
{
	return __raw_readl(reg_offset);
}

static unsigned int s5pv210_systimer_write(unsigned int *reg_offset,
					unsigned int value)
{
	unsigned int temp_regs;

	__raw_writel(value, reg_offset);

	if (reg_offset == S5PV210_TCON) {
		while (!(__raw_readl(S5PV210_INT_CSTAT) &
				S5PV210_INT_TCON));
		temp_regs = __raw_readl(S5PV210_INT_CSTAT);
		temp_regs |= S5PV210_INT_TCON;
		__raw_writel(temp_regs, S5PV210_INT_CSTAT);

	} else if (reg_offset == S5PV210_ICNTB) {
		while (!(__raw_readl(S5PV210_INT_CSTAT) &
				S5PV210_INT_ICNTB));
		temp_regs = __raw_readl(S5PV210_INT_CSTAT);
		temp_regs |= S5PV210_INT_ICNTB;
		__raw_writel(temp_regs, S5PV210_INT_CSTAT);

	} else if (reg_offset == S5PV210_TCNTB) {
		while (!(__raw_readl(S5PV210_INT_CSTAT) &
				S5PV210_INT_TCNTB));
		temp_regs = __raw_readl(S5PV210_INT_CSTAT);
		temp_regs |= S5PV210_INT_TCNTB;
		__raw_writel(temp_regs, S5PV210_INT_CSTAT);
	}

	return 0;
}

static void s5pv210_rtc_set_tick(int enabled)
{
	unsigned int tmp;

	tmp = __raw_readl(rtc_base + S3C2410_RTCCON) & ~S3C64XX_RTCCON_TICEN;
	if (enabled)
		tmp |= S3C64XX_RTCCON_TICEN;
	__raw_writel(tmp, rtc_base + S3C2410_RTCCON);
}

static void s5pv210_tick_timer_setup(void);

static void s5pv210_tick_timer_start(unsigned long load_val,
					int autoreset)
{
	unsigned int tmp;

	tmp = __raw_readl(rtc_base + S3C2410_RTCCON) &
		~(S3C64XX_RTCCON_TICEN | S3C2410_RTCCON_RTCEN);
	__raw_writel(tmp, rtc_base + S3C2410_RTCCON);

	__raw_writel(load_val, rtc_base + S3C2410_TICNT);

	tmp |= S3C64XX_RTCCON_TICEN;

	__raw_writel(tmp, rtc_base + S3C2410_RTCCON);
}

static inline void s5pv210_tick_timer_stop(void)
{
	unsigned int tmp;

	tmp = __raw_readl(rtc_base + S3C2410_RTCCON) &
		~(S3C64XX_RTCCON_TICEN | S3C2410_RTCCON_RTCEN);

	__raw_writel(tmp, rtc_base + S3C2410_RTCCON);

}

static void s5pv210_sched_timer_start(unsigned long load_val,
					int autoreset)
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg;

	/* clock configuration setting and enable */
	struct clk *clk;

	clk = clk_get(NULL, "systimer");
	if (IS_ERR(clk))
		panic("failed to get clock[%s] for system timer", "systimer");

	clk_enable(clk);

	clk_put(clk);

	tcnt = 0xffffffff;  /* default value for tcnt */

	/* initialize system timer clock */
	tcfg = s5pv210_systimer_read(S5PV210_TCFG);

	tcfg &= ~S5PV210_TCFG_TCLK_MASK;
	tcfg |= S5PV210_TCFG_TCLK_USB;

	s5pv210_systimer_write(S5PV210_TCFG, tcfg);

	/* TCFG must not be changed at run-time.
	 * If you want to change TCFG, stop timer(TCON[0] = 0)
	 */
	s5pv210_systimer_write(S5PV210_TCON, 0);

	/* read the current timer configuration bits */
	tcon = s5pv210_systimer_read(S5PV210_TCON);
	tcfg = s5pv210_systimer_read(S5PV210_TCFG);

	tcfg &= ~S5PV210_TCFG_TCLK_MASK;
	tcfg |= S5PV210_TCFG_TCLK_USB;
	tcfg &= ~S5PV210_PRESCALER_MASK;

	s5pv210_systimer_write(S5PV210_TCFG, tcfg);

	s5pv210_systimer_write(S5PV210_TCNTB, tcnt);

#if !defined(USE_SYSTIMER_IRQ)
	/* set timer con */
	tcon =  (S5PV210_TCON_START | S5PV210_TCON_AUTO_RELOAD);
	s5pv210_systimer_write(S5PV210_TCON, tcon);
#else
	/* set timer con */
	tcon =  S5PV210_TCON_INT_AUTO | S5PV210_TCON_START |
			S5PV210_TCON_AUTO_RELOAD;
	s5pv210_systimer_write(S5PV210_TCON, tcon);

	tcon |= S5PV210_TCON_INT_START;
	s5pv210_systimer_write(S5PV210_TCON, tcon);

	/* Interrupt Start and Enable */
	s5pv210_systimer_write(S5PV210_INT_CSTAT,
				(S5PV210_INT_ICNTEIE |
					S5PV210_INT_EN));
#endif
	sched_timer_running = 1;
}

/*
 * RTC tick : count down to zero, interrupt, reload
 */
static int s5pv210_tick_set_next_event(unsigned long cycles,
				   struct clock_event_device *evt)
{
	/* printk(KERN_INFO "%d\n", cycles); */
	if  (cycles == 0)	/* Should be larger than 0 */
		cycles = 1;
	s5pv210_tick_timer_start(cycles, 0);
	return 0;
}

static void s5pv210_tick_set_mode(enum clock_event_mode mode,
			      struct clock_event_device *evt)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		tick_timer_mode = 1;
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		s5pv210_tick_timer_stop();
		tick_timer_mode = 0;
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		break;
	case CLOCK_EVT_MODE_RESUME:
		s5pv210_tick_timer_setup();
		s5pv210_sched_timer_start(~0, 1);
		break;
	}
}

static struct clock_event_device clockevent_tick_timer = {
	.name		= "S5PV210 event timer",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_next_event	= s5pv210_tick_set_next_event,
	.set_mode	= s5pv210_tick_set_mode,
};

irqreturn_t s5pv210_tick_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_tick_timer;

	__raw_writel(S3C2410_INTP_TIC, rtc_base + S3C2410_INTP);
	/* In case of oneshot mode */
	if (tick_timer_mode == 0)
		s5pv210_tick_timer_stop();

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction s5pv210_tick_timer_irq = {
	.name		= "rtc-tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= s5pv210_tick_timer_interrupt,
};

static void  s5pv210_init_dynamic_tick_timer(unsigned long rate)
{
	tick_timer_mode = 1;

	s5pv210_tick_timer_stop();

	s5pv210_tick_timer_start((rate / HZ) - 1, 1);

	clockevent_tick_timer.mult = div_sc(rate, NSEC_PER_SEC,
					    clockevent_tick_timer.shift);
	clockevent_tick_timer.max_delta_ns =
		clockevent_delta2ns(-1, &clockevent_tick_timer);
	clockevent_tick_timer.min_delta_ns =
		clockevent_delta2ns(1, &clockevent_tick_timer);

	clockevent_tick_timer.cpumask = cpumask_of(0);
	clockevents_register_device(&clockevent_tick_timer);

	printk(KERN_INFO "mult[%u]\n", clockevent_tick_timer.mult);
	printk(KERN_INFO "max_delta_ns[%llu]\n", clockevent_tick_timer.max_delta_ns);
	printk(KERN_INFO "min_delta_ns[%llu]\n", clockevent_tick_timer.min_delta_ns);
	printk(KERN_INFO "rate[%lu]\n", rate);
	printk(KERN_INFO "HZ[%d]\n", HZ);
}


/*
 * ---------------------------------------------------------------------------
 * SYSTEM TIMER ... free running 32-bit clock source and scheduler clock
 * ---------------------------------------------------------------------------
 */
irqreturn_t s5pv210_sched_timer_interrupt(int irq, void *dev_id)
{
	volatile unsigned int temp_cstat;

	temp_cstat = s5pv210_systimer_read(S5PV210_INT_CSTAT);
	temp_cstat |= S5PV210_INT_STATS;

	s5pv210_systimer_write(S5PV210_INT_CSTAT, temp_cstat);

	return IRQ_HANDLED;
}

struct irqaction s5pv210_systimer_irq = {
	.name		= "System timer",
	.flags		= IRQF_DISABLED ,
	.handler	= s5pv210_sched_timer_interrupt,
};


static cycle_t s5pv210_sched_timer_read(void)
{

	return (cycle_t)~__raw_readl(S5PV210_TCNTO);
}

struct clocksource clocksource_s5pv210 = {
	.name		= "clock_source_systimer",
	.rating		= 300,
	.read		= s5pv210_sched_timer_read,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static void s5pv210_init_clocksource(unsigned long rate)
{
	static char err[] __initdata = KERN_ERR
			"%s: can't register clocksource!\n";

	clocksource_s5pv210.mult
		= clocksource_khz2mult(rate/1000, clocksource_s5pv210.shift);

	s5pv210_sched_timer_start(~0, 1);

	if (clocksource_register(&clocksource_s5pv210))
		printk(err, clocksource_s5pv210.name);
}

/*
 *  Event/Sched Timer initialization
 */
static void s5pv210_timer_setup(void)
{
	unsigned long rate;
	struct clk *clk_rtc;

	clk_rtc = clk_get(NULL, "rtc");
	if (IS_ERR(clk_rtc))
		panic("failed to get clock for RTC");
	clk_enable(clk_rtc);
	clk_put(clk_rtc);

	/* Setup event timer using XrtcXTI */
	if (clk_event == NULL)
		clk_event = clk_get(NULL, "xrtcxti");

	if (IS_ERR(clk_event))
		panic("failed to get clock for event timer");

	rate = clk_get_rate(clk_event);
	s5pv210_init_dynamic_tick_timer(rate);

	/* Setup sched-timer using XusbXTI */
	if (clk_sched == NULL)
		clk_sched = clk_get(NULL, "xusbxti");
	if (IS_ERR(clk_sched))
		panic("failed to get clock for sched-timer");
	rate = clk_get_rate(clk_sched);
	s5pv210_init_clocksource(rate);
}


static void s5pv210_tick_timer_setup(void)
{
	unsigned long rate;

	rate = clk_get_rate(clk_event);
	s5pv210_tick_timer_start((rate / HZ) - 1, 1);
}


static void __init s5pv210_timer_init(void)
{
	s5pv210_timer_setup();
	setup_irq(IRQ_RTC_TIC, &s5pv210_tick_timer_irq);
#if defined(USE_SYSTIMER_IRQ)
	setup_irq(IRQ_SYSTIMER, &s5pv210_systimer_irq);
#endif
}

struct sys_timer s5pv210_timer = {
	.init		= s5pv210_timer_init,
};

