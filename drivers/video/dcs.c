/* linux/drivers/video/dcs.c
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * MIPI-DSI Command set.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <plat/regs-dsim.h>

#include <plat/dsim.h>
#include <plat/mipi_dsi.h>

#include "s5p_mipi_dsi_lowlevel.h"
#include "dcs.h"
#include "s5p_mipi_dsi_common.h"

/* DCS commands */

/*
Function Name : DCS_EnterIdle
Function Description :

6.1 enter_idle_mode
This command causes the display module to enter Idle Mode.
In Idle Mode, color expression is reduced.
Colors are shown on the display device using the MSB of each
of the R, G and B color components in the frame memory.
*/
void dcs_enter_idle(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x39, 0);
}

/*
6.2 enter_invert_mode
This command causes the display module to invert the image data
only on the display device.
The frame memory contents remain unchanged.
No status bits are changed.
*/
void dcs_enter_invert(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x21, 0);
}

/*
6.3 enter_normal_mode
This command causes the display module to enter the Normal mode.
Normal Mode is defined as Partial Display mode and Scroll mode are off.
The host processor sends PCLK, HS and VS information
to Type 2 display modules two frames before this
command is sent when the display module is in Partial Display Mode.
*/

void dcs_enter_normal(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x13, 0);
}

/*
6.4 enter_partial_mode
This command causes the display module to enter the Partial Display Mode.
The Partial Display Mode window is described by the set_partial_area command.
See section 6.30 for details.
To leave Partial Display Mode, the enter_normal_mode command should be written.
The host processor continues to send PCLK,
HS and VS information to Type 2 display modules for two
frames after this command is sent
when the display module is in Normal Display Mode.
*/

void dcs_enter_partial(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x12, 0);
}

/*
6.5 enter_sleep_mode
This command causes the display module
to enter the Sleep mode.
In this mode, all unnecessary blocks inside
the display module are disabled except interface communication.
This is the lowest power mode the display module supports.
DBI or DSI Command Mode remains operational
and the frame memory maintains its contents. The host
processor continues to send PCLK, HS and VS information
to Type 2 and Type 3 display modules for two
frames after this command is sent
when the display module is in Normal mode.
*/

void dcs_enter_sleep(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x10, 0);
	mdelay(60);
}
/*
6.6 exit_idle_mode
This command causes the display module to exit Idle mode.
*/
void dcs_exit_idle(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x38, 0);
}

/*
6.7 exit_invert_mode
This command causes the display module to stop
inverting the image data on the display device. The frame
memory contents remain unchanged. No status bits are changed.
*/
void dcs_exit_invert(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x20, 0);
}

/*
6.8 exit_sleep_mode
This command causes the display module to exit Sleep mode.
All blocks inside the display module are enabled.
The host processor sends PCLK, HS and VS information
to Type 2 and Type 3 display modules two frames
before this command is sent when the display module is in Normal Mode.
*/

void dcs_exit_sleep(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x11, 0);
	mdelay(600);
}

/*
6.19 nop
This command does not have any effect on the display module.
It can be used to terminate a Frame  Memory Write or Read
as described in the descriptions for
write_memory_continue and read_memory_continue.
*/
void dcs_nop(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x0, 0);
}

/* softreset */
void dcs_softreset(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x1, 0);
}

/* set pixelformat */
void dcs_set_pixcelformat(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x3a, 0);
}

/* set display on */
void dcs_set_display_on(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x29, 0);
}

/* set display off */
void dcs_set_display_off(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA,
		0x28, 0);
}

void dcs_set_panel_condition(struct mipi_dsim_device *dsim)
{
	unsigned char data_to_send[14] = {
		0xf8, 0x28, 0x28, 0x08, 0x08, 0x40, 0xb0,
		0x50, 0x90, 0x10, 0x30, 0x10, 0x00, 0x00
	};

	s5p_mipi_dsi_wr_data(dsim, DCS_LONG_WR,
		(unsigned int) data_to_send, sizeof(data_to_send));
}

void dcs_set_gamma(struct mipi_dsim_device *dsim)
{
	unsigned char data_to_send[26] = {
		0xfa, 0x02, 0x18, 0x08, 0x24, 0xe0, 0xd9, 0xd5, 0xc8, 0xcf,
		0xc0, 0xd3, 0xd9, 0xcc, 0xa9, 0xb2, 0x9d, 0xbb, 0xc1, 0xb3,
		0x00, 0x9e, 0x00, 0x9a, 0x00, 0xd3
	};

	s5p_mipi_dsi_wr_data(dsim, DCS_LONG_WR,
		(unsigned int) data_to_send, sizeof(data_to_send));

	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xfa, 0x03);
}

void dcs_password(struct mipi_dsim_device *dsim)
{
	unsigned char rf[3] = {0x00, 0x5a, 0x5a};

	rf[0] = 0xf0;
	s5p_mipi_dsi_wr_data(dsim, DCS_LONG_WR,
		(unsigned int) rf, 3);

	rf[0] = 0xf1;
	s5p_mipi_dsi_wr_data(dsim, DCS_LONG_WR,
		(unsigned int) rf, 3);

	rf[0] = 0xfc;
	s5p_mipi_dsi_wr_data(dsim, DCS_LONG_WR,
		(unsigned int) rf, 3);

	/* HZ (0x16 = 60Hz) */
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xfd, 0x14);
}

void dcs_b0(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xb0, 0x02);
}

void dcs_display_off(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xfe, 0x00);
}

void dcs_chopping(struct mipi_dsim_device *dsim)
{
	unsigned char data_to_send[4] = {0xf6, 0x01, 0x8c, 0x0b};

	s5p_mipi_dsi_wr_data(dsim, DCS_LONG_WR,
		(unsigned int) data_to_send, sizeof(data_to_send));
}

void dcs_panel_dir(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xf7, 0x03);
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xb0, 0x02);
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xb3, 0xc3);
	/* added */
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_1_PARA,
		0xc0, 0x00);
}

void dcs_stand_by_on(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA, 0x10, 0);
}

void dcs_stand_by_off(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA, 0x11, 0);
}

void dcs_display_on(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA, 0x29, 0);
}

void dcs_set_tear_on(struct mipi_dsim_device *dsim)
{
	s5p_mipi_dsi_wr_data(dsim, DCS_WR_NO_PARA, 0x35, 0);
}


MODULE_DESCRIPTION("MIPI-DSI based dcs AMOLED LCD Panel Driver");
MODULE_LICENSE("GPL");
