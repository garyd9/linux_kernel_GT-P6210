#ifndef __MACH_GPIO_C1_H
#define __MACH_GPIO_C1_H __FILE__

#if defined(CONFIG_MACH_C1_REV02)
#include "gpio-c1-rev02.h"
#elif defined(CONFIG_MACH_Q1_REV02)
#include "gpio-q1-rev02.h"
#elif defined(CONFIG_MACH_TALBOT_REV02)
#include "gpio-talbot-rev02.h"
#elif defined(CONFIG_MACH_C1_REV01)
#include "gpio-c1-rev01.h"
#elif defined(CONFIG_MACH_P8_REV00)
#include "gpio-p8-rev00.h"
#elif defined(CONFIG_MACH_P8_REV01)
#include "gpio-p8-rev01.h"
#elif defined(CONFIG_MACH_P4W_REV00)
#include "gpio-p4w-rev00.h"
#elif defined(CONFIG_MACH_P4W_REV01)
#include "gpio-p4w-rev01.h"
#elif defined(CONFIG_MACH_P2_REV00)
#include "gpio-p2-rev00.h"
#elif defined(CONFIG_MACH_P2_REV01)
#include "gpio-p2-rev01.h"
#elif defined(CONFIG_MACH_P2_REV02)
#include "gpio-p2-rev02.h"
#elif defined(CONFIG_MACH_P8LTE_REV00)
#include "gpio-p8lte-rev00.h"
#else
#include "gpio-c1-rev00.h"
#endif

#endif /* __MACH_GPIO_C1_H */
