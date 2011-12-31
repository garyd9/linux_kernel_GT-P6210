/* arch/arm/mach-s5pv210/include/mach/regs-systimer.h
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * S5PV210 System Time configutation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __ASM_ARCH_REGS_SYSTIMER_H
#define __ASM_ARCH_REGS_SYSTIMER_H

#include <mach/map.h>

#define S5PV210_TCFG			(S5P_VA_SYSTIMER + (0x00))
#define S5PV210_TCON			(S5P_VA_SYSTIMER + (0x04))
#define S5PV210_TCNTB			(S5P_VA_SYSTIMER + (0x08))
#define S5PV210_TCNTO			(S5P_VA_SYSTIMER + (0x0C))
#define S5PV210_ICNTB			(S5P_VA_SYSTIMER + (0x18))
#define S5PV210_ICNTO			(S5P_VA_SYSTIMER + (0x1C))
#define S5PV210_INT_CSTAT		(S5P_VA_SYSTIMER + (0x20))

/* Value for TCFG */
#define S5PV210_TCFG_TCLK_MASK          (3<<12)
#define S5PV210_TCFG_TCLK_XXTI          (0<<12)
#define S5PV210_TCFG_TCLK_RTC           (1<<12)
#define S5PV210_TCFG_TCLK_USB           (2<<12)
#define S5PV210_TCFG_TCLK_PCLK          (3<<12)

#define S5PV210_TCFG_DIV_MASK           (7<<8)
#define S5PV210_TCFG_DIV_1              (0<<8)
#define S5PV210_TCFG_DIV_2              (1<<8)
#define S5PV210_TCFG_DIV_4              (2<<8)
#define S5PV210_TCFG_DIV_8              (3<<8)
#define S5PV210_TCFG_DIV_16             (4<<8)

#define S5PV210_TARGET_HZ          200
#define S5PV210_PRESCALER          5
#define S5PV210_PRESCALER_MASK     (0x3f<<0)

/* value for TCON */
#define S5PV210_TCON_INT_AUTO           (1<<5)
#define S5PV210_TCON_INT_IMM            (1<<4)
#define S5PV210_TCON_INT_START          (1<<3)
#define S5PV210_TCON_AUTO_RELOAD        (1<<2)
#define S5PV210_TCON_IMM_UPDATE         (1<<1)
#define S5PV210_TCON_START              (1<<0)

/* Value for INT_CSTAT */
#define S5PV210_INT_IWIE           (1<<9)
#define S5PV210_INT_TWIE           (1<<10)
#define S5PV210_INT_ICNTEIE        (1<<6)
#define S5PV210_INT_TCON           (1<<5)
#define S5PV210_INT_ICNTB          (1<<4)
#define S5PV210_INT_TCNTB          (1<<2)
#define S5PV210_INT_STATS          (1<<1)
#define S5PV210_INT_EN             (1<<0)

#endif /*  __ASM_ARCH_REGS_SYSTIMER_H */

