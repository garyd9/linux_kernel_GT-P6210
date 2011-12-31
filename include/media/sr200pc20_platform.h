/*
 * Driver for SR200PC20 (2M camera) from SAMSUNG ELECTRONICS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#define DEFAULT_FMT		V4L2_PIX_FMT_UYVY	/* YUV422 */
#define DEFAULT_PREVIEW_WIDTH		800
#define DEFAULT_PREVIEW_HEIGHT		600
#define DEFAULT_CAPTURE_WIDTH		1600
#define DEFAULT_CAPTURE_HEIGHT		1200

struct sr200pc20_platform_data {
	u32 default_width;
	u32 default_height;
	u32 pixelformat;
	u32 freq;	/* MCLK in KHz */

	/* This SoC supports Parallel & CSI-2 */
	u32 is_mipi;
	u32 streamoff_delay;	/* ms */
};
