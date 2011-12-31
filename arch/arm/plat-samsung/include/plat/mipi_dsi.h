/* linux/arm/arch/mach-s5pc110/include/mach/mipi_dsi.h
 *
 * definitions for DDI based MIPI-DSI.
 *
 * Copyright (c) 2009 Samsung Electronics
 * InKi Dae <inki.dae <at> samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _MIPI_DDI_H
#define _MIPI_DDI_H

enum mipi_ddi_interface {
	RGB_IF = 0x4000,
	I80_IF = 0x8000,
	YUV_601 = 0x10000,
	YUV_656 = 0x20000,
	MIPI_VIDEO = 0x1000,
	MIPI_COMMAND = 0x2000,
};

enum mipi_ddi_panel_select {
	DDI_MAIN_LCD = 0,
	DDI_SUB_LCD = 1,
};

enum mipi_ddi_model {
	S6DR117 = 0,
};

enum mipi_ddi_parameter {
	/* DSIM video interface parameter */
	DSI_VIRTUAL_CH_ID = 0,
	DSI_FORMAT = 1,
	DSI_VIDEO_MODE_SEL = 2,
};

struct lcd_device;
struct fb_info;

struct mipi_ddi_platform_data {
	void *dsim_data;
	/*
	 * it is used for command mode lcd panel and
	 * when all contents of framebuffer in panel module are transfered
	 * to lcd panel it occurs te signal.
	 *
	 * note:
	 * - in case of command mode(cpu mode), it should be triggered only
	 *   when TE signal of lcd panel and frame done interrupt of display
	 *   controller or mipi controller occurs.
	 */
	unsigned int te_irq;

	/*
	 * it is used for PM stable time at te interrupt handler and
	 * could be used according to lcd panel characteristic or not.
	 */
	unsigned int resume_complete;

	int (*lcd_reset) (struct lcd_device *ld);
	int (*lcd_power_on) (struct lcd_device *ld, int enable);
	int (*backlight_on) (int enable);

	/* transfer command to lcd panel at LP mode. */
	int (*cmd_write) (void *dsim_data, unsigned int data_id,
		unsigned int data0, unsigned int data1);
	int (*cmd_read) (void *dsim_data, unsigned int data_id,
		unsigned int data0, unsigned int data1);
	/*
	 * get the status that all screen data have been transferred
	 * to mipi-dsi.
	 */
	int (*get_dsim_frame_done) (void *dsim_data);
	int (*clear_dsim_frame_done) (void *dsim_data);

	/*
	 * changes mipi transfer mode to LP or HS mode.
	 *
	 * LP mode needs when some commands like gamma values transfers
	 * to lcd panel.
	 */
	int (*change_dsim_transfer_mode) (unsigned int mode);

	/* get frame done status of display controller. */
	int (*get_fb_frame_done) (struct fb_info *info);
	/* trigger display controller in case of cpu mode. */
	void (*trigger) (struct fb_info *info);

	unsigned int reset_delay;
	unsigned int power_on_delay;
	unsigned int power_off_delay;
};
#endif
