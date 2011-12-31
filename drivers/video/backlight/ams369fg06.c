/* linux/drivers/video/backlight/ams369fg06.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lcd.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <plat/gpio-cfg.h>

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00
#define COMMAND_ONLY		0xFE
#define DATA_ONLY		0xFF

#define dbg(fmt...)

struct s5p_lcd{
	struct spi_device *g_spi;
	struct lcd_device *lcd_dev;
	struct backlight_device *bl_dev;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};
static struct s5p_lcd lcd;

#ifdef CONFIG_BACKLIGHT_AMS369FG06_AMOLED
static int locked;
#endif

const unsigned short SEQ_DISPLAY_ON[] = {
	0x14, 0x03,

	ENDDEF, 0x0000
};

const unsigned short SEQ_DISPLAY_OFF[] = {
	0x14, 0x00,
	ENDDEF, 0x0000
};

const unsigned short SEQ_STANDBY_ON[] = {
	0x1D, 0xA1,
	SLEEPMSEC, 200,

	ENDDEF, 0x0000
};

const unsigned short SEQ_STANDBY_OFF[] = {
	0x1D, 0xA0,
	SLEEPMSEC, 250,

	ENDDEF, 0x0000
};

const unsigned short SEQ_SETTING[] = {
	0x31, 0x08, /* panel setting */
	0x32, 0x14,
	0x30, 0x02,
	0x27, 0x01, /* 0x27, 0x01 */
	0x12, 0x08,
	0x13, 0x08,
	0x15, 0x00, /* 0x15, 0x00 */
	0x16, 0x00, /* 24bit line and 16M color */

	0xef, 0xd0, /* pentile key setting */
	DATA_ONLY, 0xe8,

	0x39, 0x44,
	0x40, 0x00,
	0x41, 0x3f,
	0x42, 0x2a,
	0x43, 0x27,
	0x44, 0x22,
	0x45, 0x1f,
	0x46, 0x44,
	0x50, 0x00,
	0x51, 0x00,
	0x52, 0x17,
	0x53, 0x24,
	0x54, 0x26,
	0x55, 0x1f,
	0x56, 0x43,
	0x60, 0x00,
	0x61, 0x3f,
	0x62, 0x2a,
	0x63, 0x25,
	0x64, 0x24,
	0x65, 0x1b,
	0x66, 0x5c,

	0x17, 0x22, /* Boosting Freq */
	0x18, 0x33, /* Power AMP Medium */
	0x19, 0x03, /* Gamma Amp Medium	*/
	0x1a, 0x01, /* Power Boosting */
	0x22, 0xa4, /* Vinternal = 0.65*VCI */
	0x23, 0x00, /* VLOUT1 Setting = 0.98*VCI */
	0x26, 0xa0, /* Display Condition LTPS signal generation: DOTCLK */

	0x1d, 0xa0,
	SLEEPMSEC, 300,

	0x14, 0x03,

	ENDDEF, 0x0000
};

static int ams369fg06_spi_write_driver(int addr, int data)
{
	u16 buf[1];
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len	= 2,
		.tx_buf	= buf,
	};

	buf[0] = (addr << 8) | data;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

#ifdef CONFIG_BACKLIGHT_AMS369FG06_AMOLED
	int ret;
	locked  = 1;
	ret = spi_sync(lcd.g_spi, &msg);
	locked = 0;

	return ret ;
#else
	return spi_sync(lcd.g_spi, &msg);
#endif
}

static void ams369fg06_spi_write(unsigned char address, unsigned char command)
{
	if (address != DATA_ONLY)
		ams369fg06_spi_write_driver(0x70, address);

	ams369fg06_spi_write_driver(0x72, command);
}

static void ams369fg06_panel_send_sequence(const unsigned short *wbuf)
{
	int i = 0;

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC)
			ams369fg06_spi_write(wbuf[i], wbuf[i+1]);
		else
			/* msleep(wbuf[i+1]); */
			mdelay(wbuf[i+1]);
		i += 2;
	}
}

void ams369fg06_ldi_init(void)
{
	ams369fg06_panel_send_sequence(SEQ_SETTING);
	ams369fg06_panel_send_sequence(SEQ_STANDBY_OFF);
}

void ams369fg06_ldi_enable(void)
{
	ams369fg06_panel_send_sequence(SEQ_DISPLAY_ON);
	ams369fg06_panel_send_sequence(SEQ_STANDBY_OFF);
}

void ams369fg06_ldi_disable(void)
{
	ams369fg06_panel_send_sequence(SEQ_DISPLAY_OFF);
	/* For fixing LCD suspend problem */
	ams369fg06_panel_send_sequence(SEQ_STANDBY_ON);
}


/* backlight operations and functions */
#ifdef CONFIG_BACKLIGHT_AMS369FG06_AMOLED
static int s5p_bl_update_status(struct backlight_device *bd)
{
	int bl = bd->props.brightness;
	dbg("\nUpdate brightness=%d\n", bd->props.brightness);
	int level = 0;

	if (!locked) {
		if ((bl >= 0) && (bl <= 80))
			level = 1;
		else if ((bl > 80) && (bl <= 100))
			level = 2;
		else if ((bl > 100) && (bl <= 150))
			level = 3;
		else if ((bl > 150) && (bl <= 180))
			level = 4;
		else if ((bl > 180) && (bl <= 200))
			level = 5;
		else if ((bl > 200) && (bl <= 255))
			level = 6;

		if (level) {
			ams369fg06_spi_write(0x39, ((bl/64 + 1)<<4)&(bl/64+1));

			switch (level) {
			/* If bl is not halved, variation in brightness is
			 * observed as a curve with the middle region being
			 * brightest and the sides being darker. It is
			 * required that brightness increases gradually
			 * from left to right.*/
			case 1:
				ams369fg06_spi_write(0x46, (bl/2)+6);
				ams369fg06_spi_write(0x56, (bl/2)+4);
				ams369fg06_spi_write(0x66, (bl/2)+30);
				break;
			case 2:
				ams369fg06_spi_write(0x46, (bl/2)+4);
				ams369fg06_spi_write(0x56, (bl/2)+2);
				ams369fg06_spi_write(0x66, (bl/2)+28);
				break;
			case 3:
				ams369fg06_spi_write(0x46, (bl/2)+6);
				ams369fg06_spi_write(0x56, (bl/2)+1);
				ams369fg06_spi_write(0x66, (bl/2)+32);
				break;
			case 4:
				ams369fg06_spi_write(0x46, (bl/2)+6);
				ams369fg06_spi_write(0x56, (bl/2)+1);
				ams369fg06_spi_write(0x66, (bl/2)+38);
				break;
			case 5:
				ams369fg06_spi_write(0x46, (bl/2)+7);
				ams369fg06_spi_write(0x56, (bl/2));
				ams369fg06_spi_write(0x66, (bl/2)+40);
				break;
			case 6:
				ams369fg06_spi_write(0x46, (bl/2)+10);
				ams369fg06_spi_write(0x56, (bl/2));
				ams369fg06_spi_write(0x66, (bl/2)+48);
				break;
			default:
				break;
			}
		} /* level check over */
	} else {
		dbg("\nLOCKED!!!Brightness cannot be changed now!locked=%d",
			locked);
	}
	return 0;
}

static int s5p_bl_get_brightness(struct backlilght_device *bd)
{
	dbg("\n reading brightness\n");
	return 0;
}

static const struct backlight_ops s5p_bl_ops = {
	.update_status = s5p_bl_update_status,
	.get_brightness = s5p_bl_get_brightness,
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ams369fg06_early_suspend(struct early_suspend *handler)
{
	ams369fg06_ldi_disable();
}

static void ams369fg06_late_resume(struct early_suspend *handler)
{
	ams369fg06_ldi_init();
	ams369fg06_ldi_enable();
}
#endif

static int __init ams369fg06_probe(struct spi_device *spi)
{
	struct lcd_platform_data *lcd_pd;
	int ret;

	lcd_pd = (struct lcd_platform_data *)spi->dev.platform_data;
	lcd_pd->reset(NULL);
	mdelay(lcd_pd->reset_delay);

	spi->bits_per_word = 16;
	ret = spi_setup(spi);
#ifdef CONFIG_BACKLIGHT_AMS369FG06_AMOLED
	lcd.g_spi = spi;

	/* The node is named as pwm-backlight even though PWM
	 * control is not being done since Eclair interface is
	 * looking for "pwm-backlight" for backlight brightness
	 * control*/
	lcd.bl_dev = backlight_device_register("pwm-backlight",
					&spi->dev, &lcd, &s5p_bl_ops);
	lcd.bl_dev->props.max_brightness = 255;

	dev_set_drvdata(&spi->dev, &lcd);
#else
	lcd.g_spi = spi;
#endif

	ams369fg06_ldi_init();
	ams369fg06_ldi_enable();

#ifdef CONFIG_HAS_EARLYSUSPEND
	lcd.early_suspend.suspend = ams369fg06_early_suspend;
	lcd.early_suspend.resume = ams369fg06_late_resume;
	lcd.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	register_early_suspend(&lcd.early_suspend);
#endif

	if (ret < 0)
		return 0;

	return ret;
}

#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
int ams369fg06_suspend(struct spi_device *spi, pm_message_t state)
{
	ams369fg06_ldi_disable();
	return 0;
}

int ams369fg06_resume(struct spi_device *spi)
{
	ams369fg06_ldi_init();
	ams369fg06_ldi_enable();

	return 0;
}
#endif
#else
#define ams369fg06_suspend	NULL
#define ams369fg06_resume	NULL
#endif

static struct spi_driver ams369fg06_driver = {
	.driver = {
		.name	= "ams369fg06",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ams369fg06_probe,
	.remove		= __exit_p(ams369fg06_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ams369fg06_suspend,
	.resume		= ams369fg06_resume,
#endif
};

static int __init ams369fg06_init(void)
{
	return spi_register_driver(&ams369fg06_driver);
}

static void __exit ams369fg06_exit(void)
{
	spi_unregister_driver(&ams369fg06_driver);
}

module_init(ams369fg06_init);
module_exit(ams369fg06_exit);

MODULE_AUTHOR("Jonghun, Han <jonghun.han@samsung.com>");
MODULE_DESCRIPTION("AMS369FG06 AMOLED Driver");
MODULE_LICENSE("GPL");

