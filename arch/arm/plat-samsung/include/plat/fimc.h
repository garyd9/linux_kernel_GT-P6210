/*
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * Common FIMC devices definitions and helper functions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __PLAT_SAMSUNG_FIMC_H
#define __PLAT_SAMSUNG_FIMC_H __FILE__

#include <media/s5p_fimc.h>
#if 0
struct s3c_fimc_subdev_key {
	char *name;
	int id;
	struct s3c_fimc_cam_info *cam_info;
};

/* The maximum number of ISPs that can be multiplexed into single FIMC */
#define FIMC_MAX_CAM_SOURCES	2

struct s3c_platform_fimc {
	/* table of links to camera sensors */
	struct s3c_fimc_subdev_key *source_subdev_key[FIMC_MAX_CAM_SOURCES];
};

extern struct s3c_platform_fimc s3c_fimc0_default_data;
extern struct s3c_platform_fimc s3c_fimc1_default_data;
extern struct s3c_platform_fimc s3c_fimc2_default_data;
extern struct s3c_platform_fimc s3c_fimc3_default_data;
/**
 * s5pv210_fimc_setup_clks() - S5PV210/S5PC110 fimc clocks setup function
 *
 * Set correct parent clocks on machines which bootloaded did not configured
 * fimc clocks correctly. FIMC devices works properly only if sourced from
 * certain clock sources. "mout_epll" clock is the recommended one.
 */
extern int s5pv210_fimc_setup_clks(void);


extern int s5pv310_fimc_setup_clks(void);
extern int s5pc210_fimc_setup_clks(void);
#endif
#endif /* __PLAT_SAMSUNG_FIMC_H */
