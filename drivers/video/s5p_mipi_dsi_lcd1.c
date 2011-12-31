/* linux/drivers/video/s5p_mipi_dsi_lcd1.c
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
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
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/backlight.h>
#include <linux/lcd.h>

#include <plat/gpio-cfg.h>
#include <plat/regs-dsim.h>

#include <plat/dsim.h>
#include <plat/mipi_dsi.h>

#include "s5p_mipi_dsi_lowlevel.h"
#include "dcs.h"
#include "s5p_mipi_dsi_lcd.h"
#include "s5p_mipi_dsi_common.h"

static struct mipi_dsim_lcd_device *ddi1_dev;

struct s5p_mipi_lcd{
	struct backlight_device *bd;
	struct lcd_device *lcd_dev;
};

static struct s5p_mipi_lcd lcd1;

void lcd1_off(struct mipi_dsim_device *dsim)
{
	/* Enter sleep */
	dcs_enter_sleep(dsim);

	/* Set Display off */
	dcs_set_display_off(dsim);
}

static int lcd1_bl_update_status(struct backlight_device *bd)
{
	return 0;
}

static const struct backlight_ops lcd1_bl_ops = {
	.update_status = lcd1_bl_update_status,
};

static int lcd1_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct backlight_device *bd = NULL;
	struct backlight_properties props;
	ddi1_dev = (struct mipi_dsim_lcd_device *) dsim_dev;

	props.max_brightness = 255;
	bd = backlight_device_register("pwm-backlight1",
		&dsim_dev->dev, &lcd1, &lcd1_bl_ops, &props);
	return 0;
}

static int lcd1_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	lcd1_off(ddi1_dev->master);
	return 0;
}

static int lcd1_displayon(struct mipi_dsim_lcd_device *dsim_dev)
{
	unsigned char buf1[4] = {0xf2, 0x02, 0x03, 0x1b};
	unsigned char buf2[15] = {0xf8, 0x01, 0x27, 0x27, 0x07, 0x07, 0x54,
							0x9f, 0x63, 0x86, 0x1a,
							0x33, 0x0d, 0x00, 0x00};
	unsigned char buf3[4] = {0xf6, 0x00, 0x8c, 0x07};

	/* password */
	dcs_password(ddi1_dev->master);
	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_WR_1_PARA, 0xb0, 0x09);
	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_WR_1_PARA, 0xb0, 0x09);
	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_WR_1_PARA, 0xd5, 0x64);
	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_WR_1_PARA, 0xb0, 0x09);
	/* nonBurstSyncPulse */
	if (ddi1_dev->master->pd->dsim_lcd_config->parameter[DSI_VIDEO_MODE_SEL] == DSIM_NON_BURST_SYNC_PULSE)
		s5p_mipi_dsi_wr_data(ddi1_dev->master,
			DCS_WR_1_PARA, 0xd5, 0x84);
	else
		s5p_mipi_dsi_wr_data(ddi1_dev->master,
			DCS_WR_1_PARA, 0xd5, 0xc4);

	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_LONG_WR, (unsigned int) buf1, sizeof(buf1));
	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_LONG_WR, (unsigned int) buf2, sizeof(buf2));
	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_LONG_WR, (unsigned int) buf3, sizeof(buf3));

	s5p_mipi_dsi_wr_data(ddi1_dev->master,
		DCS_WR_1_PARA, 0xfa, 0x01);
	/* Exit sleep */
	dcs_exit_sleep(ddi1_dev->master);
	/* Set Display ON */
	dcs_set_display_on(ddi1_dev->master);

	return 0;
}

static int lcd1_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	return 0;
}

struct mipi_dsim_lcd_driver lcd1_mipi_driver = {
	.name = "lcd1",
	.probe = lcd1_probe,
	.suspend =  lcd1_suspend,
	.displayon = lcd1_displayon,
	.resume = lcd1_resume,
};

int lcd1_init(void)
{
	s5p_mipi_dsi_register_lcd_driver(&lcd1_mipi_driver);

	return 0;
}


static void lcd1_exit(void)
{
	return;
}

module_init(lcd1_init);
module_exit(lcd1_exit);

MODULE_DESCRIPTION("MIPI-DSI Panel Driver");
MODULE_LICENSE("GPL");
