/*
 * Driver for S5K5CCGX (3MP camera) from LSI
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define DEFAULT_PIX_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */
#define DEFAULT_MCLK		24000000
#define S5K5CCGX_STREAMOFF_DELAY	120


enum {
	S5K5CCGX_FLASH_MODE_NORMAL,
	S5K5CCGX_FLASH_MODE_MOVIE,
	S5K5CCGX_FLASH_MODE_MAX,
};

enum {
	S5K5CCGX_FLASH_OFF = 0,
	S5K5CCGX_FLASH_ON = 1,
};

struct s5k5ccgx_platform_data {
	u32 default_width;
	u32 default_height;
	u32 pixelformat;
	u32 freq;	/* MCLK in Hz */

	/* This SoC supports Parallel & CSI-2 */
	u32 is_mipi;	/* set to 1 if mipi */
	u32 streamoff_delay;	/* ms */

	/* ISP interrupt */
	/* int (*config_isp_irq)(void);*/
	int (*flash_en)(u32 mode, u32 onoff);
	int (*is_flash_on)(void);
};

