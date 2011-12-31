/* linux/drivers/video/s5p_mipi_dsi_lcd.h
 *
 * Header file for Samsung MIPI-DSI lcd driver.
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef _S5P_MIPI_DSI_LCD_H
#define _S5P_MIPI_DSI_LCD_H

void lcd0_panel_init(struct mipi_dsim_device *dsim);
void lcd1_panel_init(struct mipi_dsim_device *dsim);
void lcd_off(struct mipi_dsim_device *dsim);

#endif
