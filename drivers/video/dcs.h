/* linux/drivers/video/dcs.h
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * Header file for MIPI-DSI Command set.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _DCS_H
#define _DCS_H

enum dsim_data_id {
	/* short packet types of packet types for command */
	GEN_SHORT_WR_NO_PARA	= 0x03,
	GEN_SHORT_WR_1_PARA	= 0x13,
	GEN_SHORT_WR_2_PARA	= 0x23,
	GEN_RD_NO_PARA		= 0x04,
	GEN_RD_1_PARA		= 0x14,
	GEN_RD_2_PARA		= 0x24,
	DCS_WR_NO_PARA		= 0x05,
	DCS_WR_1_PARA		= 0x15,
	DCS_RD_NO_PARA		= 0x06,
	SET_MAX_RTN_PKT_SIZE	= 0x37,

	/* long packet types of packet types for command */
	NULL_PKT		= 0x09,
	BLANKING_PKT		= 0x19,
	GEN_LONG_WR		= 0x29,
	DCS_LONG_WR		= 0x39,

	/* short packet types of generic command */
	CMD_OFF			= 0x02,
	CMD_ON			= 0x12,
	SHUT_DOWN		= 0x22,
	TURN_ON			= 0x32,

	/* short packet types for video data */
	VSYNC_START		= 0x01,
	VSYNC_END		= 0x11,
	HSYNC_START		= 0x21,
	HSYNC_END		= 0x31,
	EOT_PKT			= 0x08,

	/* long packet types for video data */
	RGB565_PACKED		= 0x0e,
	RGB666_PACKED		= 0x1e,
	RGB666_LOOSLY		= 0x2e,
	RGB888_PACKED		= 0x3e,
};

void dcs_enter_idle(struct mipi_dsim_device *dsim);
void dcs_enter_invert(struct mipi_dsim_device *dsim);
void dcs_enter_normal(struct mipi_dsim_device *dsim);
void dcs_enter_partial(struct mipi_dsim_device *dsim);
void dcs_enter_sleep(struct mipi_dsim_device *dsim);
void dcs_exit_idle(struct mipi_dsim_device *dsim);
void dcs_exit_invert(struct mipi_dsim_device *dsim);
void dcs_exit_sleep(struct mipi_dsim_device *dsim);
void dcs_nop(struct mipi_dsim_device *dsim);
void dcs_softreset(struct mipi_dsim_device *dsim);
void dcs_set_pixcelformat(struct mipi_dsim_device *dsim);
void dcs_set_display_on(struct mipi_dsim_device *dsim);
void dcs_set_display_off(struct mipi_dsim_device *dsim);
void dcs_set_panel_condition(struct mipi_dsim_device *dsim);
void dcs_set_gamma(struct mipi_dsim_device *dsim);
void dcs_password(struct mipi_dsim_device *dsim);
void dcs_b0(struct mipi_dsim_device *dsim);
void dcs_display_off(struct mipi_dsim_device *dsim);
void dcs_chopping(struct mipi_dsim_device *dsim);
void dcs_panel_dir(struct mipi_dsim_device *dsim);
void dcs_stand_by_on(struct mipi_dsim_device *dsim);
void dcs_stand_by_off(struct mipi_dsim_device *dsim);
void dcs_display_on(struct mipi_dsim_device *dsim);
void dcs_set_tear_on(struct mipi_dsim_device *dsim);

int dcs_init(void);
void dcs_exit(void);

#endif /* _DCS_H */
