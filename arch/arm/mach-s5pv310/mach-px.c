/* linux/arch/arm/mach-s5pv310/mach-p4w.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/delay.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ch9.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/spi/spi.h>
#include <linux/pwm_backlight.h>
#include <linux/mmc/host.h>
#include <linux/smsc911x.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spi/spi_gpio.h>
#include <linux/mfd/max8998.h>
#include <linux/mfd/max8997.h>
#include <linux/power/max17042_battery.h>
#include <linux/power_supply.h>
#include <linux/fsa9480.h>
#include <linux/lcd.h>
#include <linux/ld9040.h>
#if defined(CONFIG_S5P_MEM_CMA)
#include <linux/cma.h>
#endif
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#endif
#include <linux/bootmem.h>
#include <linux/pm_runtime.h>
#ifdef CONFIG_FB_S3C_MDNIE
#include <linux/mdnie.h>
#endif

#include <asm/pmu.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <plat/regs-serial.h>
#include <plat/s5pv310.h>
#include <plat/cpu.h>
#include <plat/fb.h>
#include <plat/iic.h>
#include <plat/devs.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/fimg2d.h>
#include <plat/sdhci.h>
#include <plat/mshci.h>
#include <plat/regs-otg.h>
#include <plat/pm.h>
#include <plat/tvout.h>

#include <plat/csis.h>
#include <plat/fimc.h>
#include <plat/gpio-cfg.h>
#include <plat/pd.h>
#include <plat/sysmmu.h>
#include <plat/media.h>

#include <plat/s3c64xx-spi.h>

#include <media/s5k3ba_platform.h>
#include <media/s5k4ba_platform.h>
#include <media/s5k4ea_platform.h>
#include <media/s5k6aa_platform.h>
#ifdef CONFIG_VIDEO_M5MO
#include <media/m5mo_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K6AAFX
#include <media/s5k6aafx_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K5CCGX_COMMON
#include <media/s5k5ccgx_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K5BAFX
#include <media/s5k5bafx_platform.h>
#endif
#ifdef CONFIG_VIDEO_S5K5BBGX
#include <media/s5k5bbgx_platform.h>
#endif
#ifdef CONFIG_VIDEO_SR200PC20
#include <media/sr200pc20_platform.h>
#endif

#if defined(CONFIG_DEV_THERMAL)
#include <plat/s5p-tmu.h>
#include <mach/regs-tmu.h>
#endif

#include <mach/regs-gpio.h>

#include <mach/map.h>
#include <mach/regs-mem.h>
#include <mach/regs-clock.h>
#include <mach/media.h>
#include <mach/gpio.h>
#include <mach/system.h>

#ifdef CONFIG_FB_S3C_MIPI_LCD
#include <mach/mipi_ddi.h>
#include <mach/dsim.h>
#endif

#include <mach/spi-clocks.h>
#include <mach/pm-core.h>

#include <mach/sec_debug.h>

#include <linux/sec_jack.h>

#ifdef CONFIG_BATTERY_SEC
#include <linux/power/sec_battery.h>
#endif
#ifdef CONFIG_SEC_THERMISTOR
#include <mach/sec_thermistor.h>
#include "px_thermistor.h"
#endif
#ifdef CONFIG_SMB136_CHARGER
#include <linux/power/smb136_charger.h>
#endif
#ifdef CONFIG_SMB347_CHARGER
#include <linux/power/smb347_charger.h>
#endif

#include <linux/sec_jack.h>

#include <linux/i2c/ak8975.h>

#ifdef CONFIG_MPU_SENSORS_MPU3050
#include <linux/mpu.h>
#endif

#ifdef CONFIG_OPTICAL_GP2A
#include <linux/gp2a.h>
#endif

#ifdef CONFIG_GYRO_K3G
#include <linux/input/k3g.h>
#include <linux/k3dh.h>
#endif

#ifdef CONFIG_SENSORS_BH1721FVC
#include <linux/bh1721fvc.h>
#endif

#include <linux/mfd/mc1n2_pdata.h>

#include <../../../drivers/video/samsung/s3cfb.h>

#ifdef CONFIG_TOUCHSCREEN_MXT1386
#include <linux/atmel_mxt1386.h>
#endif

#if defined(CONFIG_MHL_SII9234)
#include <linux/sii9234.h>
#endif
#ifdef CONFIG_30PIN_CONN
#include <linux/30pin_con.h>
#include <mach/usb_switch.h>
#endif
#include "c1.h"

#ifndef CONFIG_S3C_DEV_I2C11_EMUL
#define CONFIG_S3C_DEV_I2C11_EMUL
#endif

#ifdef CONFIG_EPEN_WACOM_G5SP
#include <linux/wacom_i2c.h>
static struct wacom_g5_callbacks *wacom_callbacks;
#endif /* CONFIG_EPEN_WACOM_G5SP */

#ifdef CONFIG_TOUCHSCREEN_MELFAS
#include <linux/melfas_ts.h>
#endif /* CONFIG_TOUCHSCREEN_MELFAS */

#ifdef CONFIG_TOUCHSCREEN_MXT768E
#include <linux/i2c/mxt768e.h>
#endif

#ifdef CONFIG_SAMSUNG_LTE
#include "../../../drivers/samsung/lte_bootloader/lte_modem_bootloader.h"
#endif

#ifdef CONFIG_USBHUB_USB3503
#include <linux/usb3503.h>
#endif

static struct charging_status_callbacks {
        void    (*tsp_set_charging_cable) (int type);
} charging_cbs;

static bool is_cable_attached;

extern struct sys_timer s5pv310_timer;

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct device *switch_dev;
EXPORT_SYMBOL(switch_dev);

static struct device *factory_dev;

void (*sec_set_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_set_param_value);

void (*sec_get_param_value)(int idx, void *value);
EXPORT_SYMBOL(sec_get_param_value);


unsigned int lcdtype;
static int __init lcdtype_setup(char *str)
{
	get_option(&str, &lcdtype);
	return 1;
}
__setup("lcdtype=", lcdtype_setup);

static void c1_sec_switch_init(void)
{
	sec_class = class_create(THIS_MODULE, "sec");

	if (IS_ERR(sec_class))
		pr_err("Failed to create class(sec)!\n");
};

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKC210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKC210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKC210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdkc210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
	[4] = {
		.hwport		= 4,
		.flags		= 0,
		.ucon		= SMDKC210_UCON_DEFAULT,
		.ulcon		= SMDKC210_ULCON_DEFAULT,
		.ufcon		= SMDKC210_UFCON_DEFAULT,
	},
};

#undef WRITEBACK_ENABLED

#if defined(CONFIG_S3C64XX_DEV_SPI0)
#if defined(CONFIG_SAMSUNG_LTE)

#define LTE_MODEM_SPI_BUS_NUM   0
#define LTE_MODEM_SPI_CS        0
#define LTE_MODEM_SPI_MAX_CLK   500*1000

struct lte_modem_bootloader_platform_data lte_modem_bootloader_pdata = {
	.name = LTE_MODEM_BOOTLOADER_DRIVER_NAME,
	.gpio_lte2ap_status = GPIO_LTE2AP_STATUS,
};

static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line = S5PV310_GPB(1),
		.set_level = gpio_set_value,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "lte_loader",
		.platform_data = &lte_modem_bootloader_pdata,
		.max_speed_hz = LTE_MODEM_SPI_MAX_CLK,
		.bus_num = LTE_MODEM_SPI_BUS_NUM,
		.chip_select = LTE_MODEM_SPI_CS,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
	}
};
#else
static struct s3c64xx_spi_csinfo spi0_csi[] = {
	[0] = {
		.line = S5PV310_GPB(1),
		.set_level = gpio_set_value,
	},
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
	}
};
#endif
#endif

#if defined(CONFIG_S3C64XX_DEV_SPI1)
static struct s3c64xx_spi_csinfo spi1_csi[] = {
	[0] = {
		.line = S5PV310_GPB(5),
		.set_level = gpio_set_value,
	},
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 1200000,
		.bus_num = 1,
		.chip_select = 0,
		.mode = SPI_MODE_3,
		.controller_data = &spi1_csi[0],
	}
};
#endif

#if defined(CONFIG_S3C64XX_DEV_SPI2)
static struct s3c64xx_spi_csinfo spi2_csi[] = {
	[0] = {
		.line = S5PV310_GPC1(2),
		.set_level = gpio_set_value,
	},
};

static struct spi_board_info spi2_board_info[] __initdata = {
	{
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 10*1000*1000,
		.bus_num = 2,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi2_csi[0],
	}
};
#endif

#ifdef CONFIG_VIDEO_FIMC
/*
 * External camera reset
 * Because the most of cameras take i2c bus signal, so that
 * you have to reset at the boot time for other i2c slave devices.
 * This function also called at fimc_init_camera()
 * Do optimization for cameras on your platform.
*/

#define CAM_CHECK_ERR_RET(x, msg)	\
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
		return x; \
	}
#define CAM_CHECK_ERR(x, msg)	\
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x); \
	}

#define CAM_CHECK_ERR_GOTO(x, out, fmt, ...) \
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR fmt, ##__VA_ARGS__); \
		goto out; \
	}

#ifdef CONFIG_VIDEO_FIMC_MIPI
int s3c_csis_power(int enable)
{
	struct regulator *regulator;
	int ret = -ENODEV;

	/* mipi_1.1v ,mipi_1.8v are always powered-on.
	* If they are off, we then power them on.
	*/
	if (enable) {
		/* VMIPI_1.1V */
		regulator = regulator_get(NULL, "vmipi_1.1v");
		if (IS_ERR(regulator))
			goto error_out;
		if (!regulator_is_enabled(regulator)) {
			printk(KERN_WARNING "%s: vmipi_1.1v is off. so ON\n",
				__func__);
			ret = regulator_enable(regulator);
			if (unlikely(ret < 0)) {
				pr_err("%s: error, vmipi_1.1v\n", __func__);
				return ret;
			}
		}
		regulator_put(regulator);

		/* VMIPI_1.8V */
		regulator = regulator_get(NULL, "vmipi_1.8v");
		if (IS_ERR(regulator))
			goto error_out;
		if (!regulator_is_enabled(regulator)) {
			printk(KERN_WARNING "%s: vmipi_1.8v is off. so ON\n",
				__func__);
			ret = regulator_enable(regulator);
			if (unlikely(ret < 0)) {
				pr_err("%s: error, vmipi_1.8v\n", __func__);
				return ret;
			}
		}
		regulator_put(regulator);
	}

	return 0;

error_out:
	printk(KERN_ERR "%s: ERROR: failed to check mipi-power\n", __func__);
	return ret;
}

#endif


#ifdef CONFIG_VIDEO_M5MO
static int m5mo_power_on(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in\n", __func__);

	/* CAM_SENSOR_CORE_1.2V */
	ret = gpio_request(GPIO_CAM_SENSOR_CORE, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_SENSOR_CORE)\n");
		return ret;
	}
	gpio_direction_output(GPIO_CAM_SENSOR_CORE, 1);
	gpio_free(GPIO_CAM_SENSOR_CORE);

	/* CAM_ISP_CORE_1.2V */
	// enable LX4(Buck4)
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	regulator_enable(regulator);
	regulator_put(regulator);
	mdelay(2);

	/* CAM_SENSOR_A2.8V */
	// enabel external LDO , GPE2[1], CAM_IO_EN
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_IO_EN)\n");
		return ret;
	}
	gpio_direction_output(GPIO_CAM_IO_EN, 1);
	gpio_free(GPIO_CAM_IO_EN);
	mdelay(5);

	/* CAM_SENSOR_IO_1.8V */
	// enable ldo16
	regulator = regulator_get(NULL, "cam_sensor_io");
	if (IS_ERR(regulator))
		return -ENODEV;
	regulator_enable(regulator);
	regulator_put(regulator);

	/* CAM_ISP_1.8V */
	// enable ldo7
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	regulator_enable(regulator);
	regulator_put(regulator);

	/* CAM_AF_2.8V */
	// enable ldo17
	regulator = regulator_get(NULL, "cam_af");
	if (IS_ERR(regulator))
		return -ENODEV;
	regulator_enable(regulator);
	regulator_put(regulator);
	mdelay(1);

	/* MCLK */
	// GPJ1[3] / CAM_A_CLKOUT
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	mdelay(2);

	/* ISP_RESET */
	// GPY3[7]
	ret = gpio_request(GPIO_ISP_RESET, "GPY3");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
	gpio_direction_output(GPIO_ISP_RESET, 1);
	gpio_free(GPIO_ISP_RESET);
	mdelay(2);

	return ret;
}

static int m5mo_power_down(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in\n", __func__);

	/*	ISP_RESET */
	ret = gpio_request(GPIO_ISP_RESET, "GPY3");
	if (ret) {
		printk(KERN_ERR "faile to request gpio(ISP_RESET)\n");
		return ret;
	}
	gpio_direction_output(GPIO_ISP_RESET, 0);
	gpio_free(GPIO_ISP_RESET);
	mdelay(2);

	/* MCLK */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(0));
	mdelay(1);

	/* CAM_AF_2.8V */
	regulator = regulator_get(NULL, "cam_af");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		regulator_force_disable(regulator);
	regulator_put(regulator);

	/* CAM_ISP_1.8V */
	regulator = regulator_get(NULL, "cam_isp");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		regulator_force_disable(regulator);
	regulator_put(regulator);

	/* CAM_SENSOR_IO_1.8V */
	regulator = regulator_get(NULL, "cam_sensor_io");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		regulator_force_disable(regulator);
	regulator_put(regulator);
	mdelay(50);

	/* CAM_SENSOR_A2.8V */
	ret = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(GPIO_CAM_IO_EN)\n");
		return ret;
	}
	gpio_direction_output(GPIO_CAM_IO_EN, 0);
	gpio_free(GPIO_CAM_IO_EN);
	mdelay(2);

	/* CAM_ISP_CORE_1.2V */
	regulator = regulator_get(NULL, "cam_isp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		regulator_force_disable(regulator);
	regulator_put(regulator);

	/* CAM_SENSOR_CORE_1.2V */
	ret = gpio_request(GPIO_CAM_SENSOR_CORE, "GPE2");
	if (ret) {
		printk(KERN_ERR "fail to request gpio(CAM_SENSOR_COR)\n");
		return ret;
	}
	gpio_direction_output(GPIO_CAM_SENSOR_CORE, 0);
	gpio_free(GPIO_CAM_SENSOR_CORE);

	return ret;
}

static int m5mo_power(int enable)
{
	int ret;

	printk("%s %s\n", __func__, enable ? "on" : "down");
	if(enable)
		ret = m5mo_power_on();
	else
		ret = m5mo_power_down();

	s3c_csis_power(enable);

	return ret;
}

static int m5mo_config_isp_irq(void)
{
	s3c_gpio_cfgpin(GPIO_ISP_INT, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_ISP_INT, S3C_GPIO_PULL_NONE);
	return 0;
}

static struct m5mo_platform_data m5mo_plat = {
	.default_width = 640, /* 1920 */
	.default_height = 480, /* 1080 */
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.config_isp_irq = m5mo_config_isp_irq,
};

static struct i2c_board_info  m5mo_i2c_info = {
	I2C_BOARD_INFO("M5MO", 0x1F),
	.platform_data = &m5mo_plat,
	.irq = IRQ_EINT(13),
};

static struct s3c_platform_camera m5mo = {
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.i2c_busnum	= 0,
	.cam_power	= m5mo_power, /*smdkv310_mipi_cam0_reset,*/
	.type		= CAM_TYPE_MIPI,
	.fmt		= ITU_601_YCBCR422_8BIT, /*MIPI_CSI_YCBCR422_8BIT*/
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &m5mo_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti", /* "mout_mpll" */
	.clk_rate	= 24000000, /* 48000000 */
	.line_length	= 1920,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 2,
	.mipi_settle	= 12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
};
#endif /* #ifdef CONFIG_VIDEO_M5MO */

#ifdef CONFIG_VIDEO_S5K6AAFX

static int s5k6aafx_power_on(void)
{
	struct regulator *regulator;
	int err = 0;

#if defined (CONFIG_VIDEO_M5MO)
	err = m5mo_power(1);
	if (err) {
		printk(KERN_ERR "%s ERROR: fail to power on\n", __func__);
		return err;
	}
#endif
	err = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_IO_EN)\n");
		return err;
	}

	err = gpio_request(GPIO_VT_CAM_15V, "GPE4");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return err;
	}

	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPL2");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nSTBY)\n");
		return err;
	}

	err = gpio_request(GPIO_CAM_VGA_nRST, "GPL2");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nRST)\n");
		return err;
	}

	/* CAM_SENSOR_A2.8V, CAM_IO_EN */
	gpio_direction_output(GPIO_CAM_IO_EN, 1);
	udelay(50); /*DSLIM. these delay-values need to be optimized later */

	/* VT_CAM_1.5V */
	gpio_direction_output(GPIO_VT_CAM_15V, 1);
	udelay(50);

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	regulator_enable(regulator);
	regulator_put(regulator);
	udelay(100);

	/* CAM_VGA_nSTBY */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 1);
	udelay(500);

	/* Mclk */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	udelay(200);

	/* CAM_VGA_nRST  */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 1);
	udelay(500);

	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_VT_CAM_15V);
	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);

	return 0;
}

static int s5k6aafx_power_off(void)
{
	struct regulator *regulator;
	int err = 0;

#if defined (CONFIG_VIDEO_M5MO)
	err = m5mo_power(0);
	if (err) {
		printk(KERN_ERR "%s ERROR: fail to power down\n", __func__);
		return err;
	}
#endif

	err = gpio_request(GPIO_CAM_IO_EN, "GPE2");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_IO_EN)\n");
		return err;
	}

	err = gpio_request(GPIO_VT_CAM_15V, "GPE4");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_VT_CAM_15V)\n");
		return err;
	}

	err = gpio_request(GPIO_CAM_VGA_nSTBY, "GPL2");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nSTBY)\n");
		return err;
	}

	err = gpio_request(GPIO_CAM_VGA_nRST, "GPL2");
	if (err) {
		printk(KERN_ERR "faile to request gpio(GPIO_CAM_VGA_nRST)\n");
		return err;
	}

	/* CAM_VGA_nRST  */
	gpio_direction_output(GPIO_CAM_VGA_nRST, 0);
	udelay(200);

	/* Mclk */
	s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(0));
	udelay(50);

	/* CAM_VGA_nSTBY */
	gpio_direction_output(GPIO_CAM_VGA_nSTBY, 0);
	udelay(200);

	/* VT_CAM_1.8V */
	regulator = regulator_get(NULL, "vt_cam_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		regulator_force_disable(regulator);
	regulator_put(regulator);
	udelay(50);

	/* VT_CAM_1.5V */
	gpio_direction_output(GPIO_VT_CAM_15V, 0);
	udelay(50);

	/* CAM_SENSOR_A2.8V, CAM_IO_EN */
	gpio_direction_output(GPIO_CAM_IO_EN, 0);

	gpio_free(GPIO_CAM_IO_EN);
	gpio_free(GPIO_VT_CAM_15V);
	gpio_free(GPIO_CAM_VGA_nSTBY);
	gpio_free(GPIO_CAM_VGA_nRST);

	return 0;
}

static int s5k6aafx_power(int onoff)
{
	if (onoff)
		s5k6aafx_power_on();
	else
	{
		s5k6aafx_power_off();
		/* s3c_i2c0_force_stop();*/ /* DSLIM. Should be implemented */
	}

	return 0;
}

static struct s5k6aafx_platform_data s5k6aafx_plat = {
	.default_width = 640,
	.default_height = 480,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
};

static struct i2c_board_info  s5k6aafx_i2c_info = {
	I2C_BOARD_INFO("S5K6AAFX", 0x78 >> 1),
	.platform_data = &s5k6aafx_plat,
};

static struct s3c_platform_camera s5k6aafx = {
	.id		= CAMERA_CSI_D,
	.type		= CAM_TYPE_MIPI,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,

	.mipi_lanes	= 1,
	.mipi_settle	= 6,
	.mipi_align	= 32,
	.i2c_busnum	= 13, /* CAM GPIO I2C just defined as 13 */
	.info		= &s5k6aafx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 24000000,
	.line_length	= 640,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
	.cam_power	= s5k6aafx_power,
};
#endif

#if defined(CONFIG_VIDEO_S5K5CCGX_COMMON) || defined(CONFIG_VIDEO_SR200PC20)
/*#define USE_CAM_GPIO_CFG*/
static int cfg_gpio_err;
void cam_cfg_gpio(struct platform_device *pdev)
{
	static int enable = 0;

	if (enable) {
#ifdef USE_CAM_GPIO_CFG
		gpio_free(GPIO_2M_nSTBY);
		gpio_free(GPIO_2M_nRST);
		gpio_free(GPIO_3M_nSTBY);
		gpio_free(GPIO_3M_nRST);
		printk(KERN_INFO "Camera GPIOs released!\n");
#endif
		enable = 0;
	} else {
		cfg_gpio_err = 0;
#ifdef USE_CAM_GPIO_CFG
		cfg_gpio_err = gpio_request(GPIO_2M_nSTBY, "GPL2");
		if (cfg_gpio_err) {
			printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
			return;
		}
		cfg_gpio_err = gpio_request(GPIO_2M_nRST, "GPL2");
		if (cfg_gpio_err) {
			printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
			return;
		}
		cfg_gpio_err = gpio_request(GPIO_3M_nSTBY, "GPL2");
		if (cfg_gpio_err) {
			printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
			return;
		}
		cfg_gpio_err = gpio_request(GPIO_3M_nRST, "GPL2");
		if (cfg_gpio_err) {
			printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
			return;
		}

		printk(KERN_INFO "Camera GPIOs configured!\n");
#endif
		enable = 1;
	}
}
#endif

#ifdef CONFIG_VIDEO_S5K5CCGX_COMMON
static int s5k5ccgx_get_i2c_busnum(void)
{
	return 0;
}

#ifdef CONFIG_VIDEO_S5K5CCGX_P8
static int s5k5ccgx_power_on(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in P8\n", __func__);

#ifndef USE_CAM_GPIO_CFG
#if !defined(CONFIG_MACH_P8LTE_REV00)
	ret = gpio_request(GPIO_CAM_AVDD_EN, "GPJ1");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(CAM_AVDD)\n");
		return ret;
	}
#endif
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
#endif

	/* 2M_nSTBY low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");

	/* 2M_nRST low */
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");

	/* CAM_A2.8V */
#if defined(CONFIG_MACH_P8LTE_REV00)
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "CAM_A2.8V");
#else
	ret = gpio_direction_output(GPIO_CAM_AVDD_EN, 1);
	CAM_CHECK_ERR_RET(ret, "CAM_AVDD");
	udelay(1);
#endif

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3mp_core");
	udelay(1);

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "vt_core_1.8v");
	udelay(1);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_io_1.8v");
	udelay(70);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	udelay(10);

	/* 3M_nSTBY */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "3M_nSTBY");
	udelay(16);

	/* 3M_nRST */
	ret = gpio_direction_output(GPIO_3M_nRST, 1);
	CAM_CHECK_ERR_RET(ret, "3M_nRST");

	/* 3MP_AF_2.8V */
	regulator = regulator_get(NULL, "3m_af_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3m_af_2.8v");
	msleep(10);

#ifndef USE_CAM_GPIO_CFG
#if !defined(CONFIG_MACH_P8LTE_REV00)
	gpio_free(GPIO_CAM_AVDD_EN);
#endif
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);
#endif
	return ret;
}

static int s5k5ccgx_power_down(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in P8\n", __func__);

#ifndef USE_CAM_GPIO_CFG
#if !defined(CONFIG_MACH_P8LTE_REV00)
	ret = gpio_request(GPIO_CAM_AVDD_EN, "GPJ1");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(CAM_AVDD)\n");
		return ret;
	}
#endif
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
#endif
	/* 3MP_AF_2.8V */
	regulator = regulator_get(NULL, "3m_af_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "3m_af_2.8v");

	/* 3M_nRST Low*/
	ret = gpio_direction_output(GPIO_3M_nRST, 0);
	CAM_CHECK_ERR(ret, "3M_nSTBY");
	udelay(50);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(5);

	/* 3M_nSTBY */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 0);
	CAM_CHECK_ERR(ret, "3M_nSTBY");
	udelay(1);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_io_1.8v");
	udelay(1);

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "vt_core_1.8v");
	udelay(1);

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "3mp_core");
	udelay(1);

	/* CAM_A2.8V */
#if defined(CONFIG_MACH_P8LTE_REV00)
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "CAM_A2.8V");
#else
	ret = gpio_direction_output(GPIO_CAM_AVDD_EN, 0);
	CAM_CHECK_ERR_RET(ret, "CAM_AVDD");
#endif

#ifndef USE_CAM_GPIO_CFG
#if !defined(CONFIG_MACH_P8LTE_REV00)
	gpio_free(GPIO_CAM_AVDD_EN);
#endif
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);
#endif
	return ret;
}
#else /* CONFIG_VIDEO_S5K5CCGX_P8 */
/* Power up/down func for P4C, P2. */
static int s5k5ccgx_power_on(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in P4C,P2\n", __func__);

#ifndef USE_CAM_GPIO_CFG
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
#endif

	/* 2M_nSTBY low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");

	/* 2M_nRST low */
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3mp_core");

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_io_1.8v");

	/* CAM_A2.8V, LDO13 */
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_analog_2.8v");

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "vt_core_1.8v");
	udelay(20);

	/* 2M_nSTBY High */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(3);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	msleep(5); /* >=5ms */

	/* 2M_nSTBY Low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	msleep(10); /* >=10ms */

	/* 2M_nRST High */
	ret = gpio_direction_output(GPIO_2M_nRST, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	msleep(5);

	/* 2M_nSTBY High */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(2);

	/* 3M_nSTBY */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "3M_nSTBY");
	udelay(16);

	/* 3M_nRST */
	ret = gpio_direction_output(GPIO_3M_nRST, 1);
	CAM_CHECK_ERR_RET(ret, "3M_nRST");
	/* udelay(10); */

	/* 3MP_AF_2.8V */
	regulator = regulator_get(NULL, "3m_af_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3m_af_2.8v");
	msleep(10);

#ifndef USE_CAM_GPIO_CFG
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);
#endif

	return ret;
}

static int s5k5ccgx_power_down(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in P4C,P2\n", __func__);

#ifndef USE_CAM_GPIO_CFG
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
#endif
	/* 3MP_AF_2.8V */
	regulator = regulator_get(NULL, "3m_af_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "3m_af_2.8v");

	/* 3M_nRST Low*/
	ret = gpio_direction_output(GPIO_3M_nRST, 0);
	CAM_CHECK_ERR(ret, "3M_nSTBY");
	udelay(50);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(5);

	/* 3M_nSTBY */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 0);
	CAM_CHECK_ERR(ret, "3M_nSTBY");
	udelay(1);

	/* 2M_nRST Low */
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");

	/* 2M_nSTBY Low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "vt_core_1.8v");

	/* CAM_A2.8V */
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
	    return -ENODEV;
	if (regulator_is_enabled(regulator))
	    ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_analog_2.8v");
	/* udelay(50); */

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_io_1.8v");
	/*udelay(50); */

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "3mp_core");

#ifndef USE_CAM_GPIO_CFG
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);
#endif
	return ret;
}
#endif /* CONFIG_VIDEO_S5K5CCGX_P8 */

static int s5k5ccgx_power(int enable)
{
	int ret = 0;

	printk("%s %s\n", __func__, enable ? "on" : "down");
	if(enable) {
#ifdef USE_CAM_GPIO_CFG
		if (cfg_gpio_err) {
			printk(KERN_ERR "%s: ERROR: gpio configuration", __func__);
			return cfg_gpio_err;
		}
#endif
		ret = s5k5ccgx_power_on();
	} else
		ret = s5k5ccgx_power_down();

	s3c_csis_power(enable);

	return ret;
}

static atomic_t flash_status = ATOMIC_INIT(S5K5CCGX_FLASH_OFF);
static int s5k5ccgx_flash_en(u32 mode, u32 onoff)
{
	static int flash_mode = S5K5CCGX_FLASH_MODE_NORMAL;
	static DEFINE_MUTEX(flash_lock);
	int ret = -ENODEV;

	printk(KERN_DEBUG "flash_en: mode=%d, onoff=%d\n", mode, onoff);

	if (unlikely((u32)mode >= S5K5CCGX_FLASH_MODE_MAX)) {
		pr_err("flash_en: ERROR, invalid flash mode(%d)\n", mode);
		return -EINVAL;
	}

	/* We could not use spin lock because of gpio kernel API.*/
	mutex_lock(&flash_lock);
	if (atomic_read(&flash_status) == onoff) {
		mutex_unlock(&flash_lock);
		pr_warn("flash_en: WARNING, already flash %s\n",
					onoff ? "On" : "Off");
		return 0;
	}

	switch (onoff) {
	case S5K5CCGX_FLASH_ON:
		if (mode == S5K5CCGX_FLASH_MODE_MOVIE)
			ret = gpio_direction_output(GPIO_CAM_MOVIE_EN, 1);
		else
			ret = gpio_direction_output(GPIO_CAM_FLASH_EN, 1);
		CAM_CHECK_ERR_GOTO(ret, out,
			"flash_en: ERROR, fail to turn flash on (mode:%d)\n",
			mode);
		flash_mode = mode;
		break;

	case S5K5CCGX_FLASH_OFF:
		if (unlikely(flash_mode != mode)) {
			pr_err("flash_en: ERROR, unmatched flash mode(%d, %d)\n",
						flash_mode, mode);
			WARN_ON(1);
			goto out;
		}

		if (mode == S5K5CCGX_FLASH_MODE_MOVIE)
			ret = gpio_direction_output(GPIO_CAM_MOVIE_EN, 0);
		else
			ret = gpio_direction_output(GPIO_CAM_FLASH_EN, 0);
		CAM_CHECK_ERR_GOTO(ret, out,
			"flash_en: ERROR, flash off (mode:%d)\n", mode);
		break;

	default:
		CAM_CHECK_ERR_GOTO(ret, out,
			"flash_en: ERROR, invalid flash cmd(%d)\n", onoff);
		break;
	}

	atomic_set(&flash_status, onoff);

out:
	mutex_unlock(&flash_lock);
	return 0;
}

static int s5k5ccgx_is_flash_on(void)
{
	return atomic_read(&flash_status);
}

static int px_cam_cfg_init(void)
{
	int ret = -ENODEV;

	/* pr_info("%s\n", __func__); */

	ret = gpio_request(GPIO_CAM_MOVIE_EN, "GPL0");
	if (unlikely(ret)) {
		pr_err("cam_cfg_init: fail to get gpio(MOVIE_EN), "
			"err=%d\n", ret);
		goto out;
	}

	ret = gpio_request(GPIO_CAM_FLASH_EN, "GPL0");
	if (unlikely(ret)) {
		pr_err("cam_cfg_init: fail to get gpio(FLASH_EN), "
			"err=%d\n", ret);
		goto out_free;
	}

	return 0;

out_free:
	gpio_free(GPIO_CAM_MOVIE_EN);
out:
	return ret;
}

static struct s5k5ccgx_platform_data s5k5ccgx_plat = {
	.default_width = 1024,
	.default_height = 768,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.streamoff_delay = S5K5CCGX_STREAMOFF_DELAY,
	.flash_en = s5k5ccgx_flash_en,
	.is_flash_on = s5k5ccgx_is_flash_on,
};

static struct i2c_board_info  s5k5ccgx_i2c_info = {
	I2C_BOARD_INFO("S5K5CCGX", 0x78>>1),
	.platform_data = &s5k5ccgx_plat,
};

static struct s3c_platform_camera s5k5ccgx = {
	.id		= CAMERA_CSI_C,
	.clk_name	= "sclk_cam0",
	.get_i2c_busnum	= s5k5ccgx_get_i2c_busnum,
	.cam_power	= s5k5ccgx_power, /*smdkv310_mipi_cam0_reset,*/
	.type		= CAM_TYPE_MIPI,
	.fmt		= ITU_601_YCBCR422_8BIT, /*MIPI_CSI_YCBCR422_8BIT*/
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.info		= &s5k5ccgx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti", /* "mout_mpll" */
	.clk_rate	= 24000000, /* 48000000 */
	.line_length	= 640,
	.width		= 640,
	.height		= 480,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 640,
		.height	= 480,
	},

	.mipi_lanes	= 1, //2,
	.mipi_settle	= 6, //12,
	.mipi_align	= 32,

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
	.streamoff_delay = S5K5CCGX_STREAMOFF_DELAY,
};
#endif /* #ifdef CONFIG_VIDEO_S5K5CCGX_COMMON */

#ifdef CONFIG_VIDEO_S5K5BAFX
static int s5k5bafx_get_i2c_busnum(void)
{
	return 0;
}

static int s5k5bafx_power_on(void)
{
	struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);
#if !defined(CONFIG_MACH_P8LTE_REV00)
	ret = gpio_request(GPIO_CAM_AVDD_EN, "GPJ1");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(CAM_AVDD)\n");
		return ret;
	}
#endif
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}

	/* 3M_nSTBY low */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "3M_nSTBY");

	/* 3M_nRST low */
	ret = gpio_direction_output(GPIO_3M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "3M_nRST");

	/* CAM_A2.8V */
#if defined(CONFIG_MACH_P8LTE_REV00)
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "CAM_A2.8V");
#else

	ret = gpio_direction_output(GPIO_CAM_AVDD_EN, 1);
	CAM_CHECK_ERR_RET(ret, "CAM_AVDD");
	//udelay(1);
#endif

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3mp_core");
	//udelay(1);

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "vt_core_1.8v");
	//udelay(1);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_io_1.8v");
	udelay(70);

	/* 2M_nSTBY High */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(10);

	/* Mclk */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	udelay(50);

	/* 2M_nRST High */
	ret = gpio_direction_output(GPIO_2M_nRST, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	udelay(50);

#if !defined(CONFIG_MACH_P8LTE_REV00)
	gpio_free(GPIO_CAM_AVDD_EN);
#endif
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);

	return 0;
}

static int s5k5bafx_power_off(void)
{
	struct regulator *regulator;
	int ret = 0;

	printk(KERN_DEBUG "%s: in\n", __func__);

#if !defined(CONFIG_MACH_P8LTE_REV00)
	ret = gpio_request(GPIO_CAM_AVDD_EN, "GPJ1");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(CAM_AVDD)\n");
		return ret;
	}
#endif
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}

	/* 2M_nRST Low*/
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR(ret, "3M_nSTBY");
	udelay(55);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(10);

	/* 2M_nSTBY */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR(ret, "3M_nSTBY");
	udelay(1);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
	    return -ENODEV;
	if (regulator_is_enabled(regulator))
	    ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_io_1.8v");
	//udelay(1);

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "vt_core_1.8v");
	//udelay(1);

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "3mp_core");

	/* CAM_A2.8V */
#if defined(CONFIG_MACH_P8LTE_REV00)
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "CAM_A2.8V");
#else
	ret = gpio_direction_output(GPIO_CAM_AVDD_EN, 0);
	CAM_CHECK_ERR_RET(ret, "CAM_AVDD");
#endif

#if !defined(CONFIG_MACH_P8LTE_REV00)
	gpio_free(GPIO_CAM_AVDD_EN);
#endif
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_2M_nSTBY);

	return 0;
}

static int s5k5bafx_power(int onoff)
{
	int ret = 0;

	printk(KERN_INFO "%s(): %s\n", __func__, onoff ? "on" : "down");
	if (onoff) {
		ret = s5k5bafx_power_on();
		if (unlikely(ret))
			goto error_out;
	} else {
		ret = s5k5bafx_power_off();
		/* s3c_i2c0_force_stop();*/ /* DSLIM. Should be implemented */
	}

	ret = s3c_csis_power(onoff);

error_out:
	return ret;
}

static struct s5k5bafx_platform_data s5k5bafx_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.freq = 24000000,
	.is_mipi = 1,
	.streamoff_delay = S5K5BAFX_STREAMOFF_DELAY,
};

static struct i2c_board_info  s5k5bafx_i2c_info = {
	I2C_BOARD_INFO("S5K5BAFX", 0x5A >> 1),
	.platform_data = &s5k5bafx_plat,
};

static struct s3c_platform_camera s5k5bafx = {
	.id		= CAMERA_CSI_D,
	.type		= CAM_TYPE_MIPI,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.mipi_lanes	= 1,
	.mipi_settle	= 6,
	.mipi_align	= 32,

	.get_i2c_busnum	= s5k5bafx_get_i2c_busnum,
	.info		= &s5k5bafx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 24000000,
	.line_length	= 800,
	.width		= 800,
	.height		= 600,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 800,
		.height	= 600,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
	.cam_power	= s5k5bafx_power,
	.streamoff_delay = S5K5BAFX_STREAMOFF_DELAY,
};
#endif


#ifdef CONFIG_VIDEO_S5K5BBGX
static int s5k5bbgx_power_on(void)
{
	struct regulator *regulator;
	int ret = 0;

#ifndef USE_CAM_GPIO_CFG
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
#endif

	/* 3M_nSTBY low */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "3M_nSTBY");

	/* 3M_nRST low */
	ret = gpio_direction_output(GPIO_3M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "3M_nRST");

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3mp_core");
	udelay(100);

	/* CAM_A2.8V */
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_analog_2.8v");
	udelay(100);

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "vt_core_1.8v");
	udelay(100);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_io_1.8v");
	udelay(400); /* 10us */

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	udelay(100);

	/* 2M_nSTBY */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(100);

	/* 2M_nRST */
	ret = gpio_direction_output(GPIO_2M_nRST, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	udelay(300);

#ifndef USE_CAM_GPIO_CFG
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);
#endif
	return 0;
}

static int s5k5bbgx_power_off(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in\n", __func__);

#ifndef USE_CAM_GPIO_CFG
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
#endif

	/* 2M_nRST */
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	udelay(100);

	/* 2M_nSTBY */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(100);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(200);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_io_1.8v");
	udelay(100);

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "vt_core_1.8v");
	udelay(100);

	/* CAM_A2.8V */
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_analog_2.8v");
	udelay(100);

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);

#ifndef USE_CAM_GPIO_CFG
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
#endif
	return 0;
}

static int s5k5bbgx_power(int onoff)
{
	int ret = 0;

	printk("%s(): %s \n", __func__, onoff ? "on" : "down");

	if (onoff) {
#ifdef USE_CAM_GPIO_CFG
		if (cfg_gpio_err) {
			printk(KERN_ERR "%s: ERROR: gpio configuration", __func__);
			return cfg_gpio_err;
		}
#endif
		ret = s5k5bbgx_power_on();
	} else {
		ret = s5k5bbgx_power_off();
		/* s3c_i2c0_force_stop();*/ /* DSLIM. Should be implemented */
	}

	return ret;
}

static struct s5k5bbgx_platform_data s5k5bbgx_plat = {
	.default_width = 640, /* DSLIM. Need to change to 800 later */
	.default_height = 480, /* DSLIM. Need to change to  600 later */
	.pixelformat = V4L2_PIX_FMT_UYVY,
};

static struct i2c_board_info  s5k5bbgx_i2c_info = {
	I2C_BOARD_INFO("S5K5BBGX", 0x5a>>1),
	.platform_data = &s5k5bbgx_plat,
};

static struct s3c_platform_camera s5k5bbgx = {
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR, /* DSLIM. Need to adjust YUV order */
	.info		= &s5k5bbgx_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 24000000,
	.line_length	= 800,
	.width		= 800,
	.height		= 600,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 800,
		.height	= 600,
	},

	/* Polarity */
	.inv_pclk	= 0,
	.inv_vsync	= 1,
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
	.cam_power	= s5k5bbgx_power,
};
#endif /* CONFIG_VIDEO_S5K5BBGX */

#ifdef CONFIG_VIDEO_SR200PC20
static int sr200pc20_get_i2c_busnum(void)
{
#ifdef CONFIG_MACH_P4W_REV01
	if (system_rev >= 2)
		return 0;
	else
#endif
		return 13;
}

static int sr200pc20_power_on(void)
{
	struct regulator *regulator;
	int ret = 0;

#ifndef USE_CAM_GPIO_CFG
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_3M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(3M_nRST)\n");
		return ret;
	}
#endif

	/* 3M_nSTBY low */
	ret = gpio_direction_output(GPIO_3M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "3M_nSTBY");

	/* 3M_nRST low */
	ret = gpio_direction_output(GPIO_3M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "3M_nRST");

	/* 2M_nSTBY low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");

	/* 2M_nRST low */
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "3mp_core");
	/* udelay(5); */

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_io_1.8v");
	/*udelay(5); */

	/* CAM_A2.8V */
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "cam_analog_2.8v");
	/* udelay(5); */

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	ret = regulator_enable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR_RET(ret, "vt_core_1.8v");
	udelay(20);

	/* ENB High */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(3); /* 30 -> 3 */

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_SFN(2));
	CAM_CHECK_ERR_RET(ret, "cfg mclk");
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_NONE);
	msleep(5); /* >= 5ms */

	/* ENB Low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	msleep(10); /* >= 10ms */

	/* 2M_nRST High*/
	ret = gpio_direction_output(GPIO_2M_nRST,1);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	/*msleep(7);*/ /* >= 7ms */

#if 0
	/* ENB High */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	msleep(12); /* >= 10ms */

	/* ENB Low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	msleep(12); /* >= 10ms */

	/* 2M_nRST Low*/
	ret = gpio_direction_output(GPIO_2M_nRST,0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	udelay(10); /* >= 16 cycle */

	/* 2M_nRST High */
	ret = gpio_direction_output(GPIO_2M_nRST,1);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
#endif
	udelay(10); /* >= 16 cycle */

#ifndef USE_CAM_GPIO_CFG
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
	gpio_free(GPIO_3M_nSTBY);
	gpio_free(GPIO_3M_nRST);
#endif
	return 0;
}

static int sr200pc20_power_off(void)
{
	struct regulator *regulator;
	int ret=0;

	printk("%s in\n", __func__);

#ifndef USE_CAM_GPIO_CFG
	ret = gpio_request(GPIO_2M_nSTBY, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nSTBY)\n");
		return ret;
	}
	ret = gpio_request(GPIO_2M_nRST, "GPL2");
	if (ret) {
		printk(KERN_ERR "Error: fail to request gpio(2M_nRST)\n");
		return ret;
	}
#endif

#if 0
	/* 2M_nRST */
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	udelay(100);

	/* 2M_nSTBY */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	udelay(100);
#endif
	/* Sleep command */
	//Sleep command ???? */
	mdelay(1);

	/* 2M_nRST Low*/
	ret = gpio_direction_output(GPIO_2M_nRST, 0);
	CAM_CHECK_ERR_RET(ret, "2M_nRST");
	udelay(3);

	/* CAM_MCLK */
	ret = s3c_gpio_cfgpin(GPIO_CAM_MCLK, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_CAM_MCLK, S3C_GPIO_PULL_DOWN);
	CAM_CHECK_ERR(ret, "cfg mclk");
	udelay(10);

	/* ENB High*/
	ret = gpio_direction_output(GPIO_2M_nSTBY, 1);
	CAM_CHECK_ERR_RET(ret, "2M_nSTBY");
	mdelay(5);

	/* ENB Low */
	ret = gpio_direction_output(GPIO_2M_nSTBY, 0);
	CAM_CHECK_ERR_RET(ret, "2M_spnSTBY");
	/* udelay(1); */

	/* VT_CORE_1.8V */
	regulator = regulator_get(NULL, "vt_core_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "vt_core_1.8v");
	/* udelay(10); */

	/* CAM_A2.8V */
	regulator = regulator_get(NULL, "cam_analog_2.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_analog_2.8v");
	//udelay(10);

	/* CAM_IO_1.8V */
	regulator = regulator_get(NULL, "cam_io_1.8v");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);
	CAM_CHECK_ERR(ret, "cam_io_1.8v");
	//udelay(10);

	/* 3MP_CORE_1.2V */
	regulator = regulator_get(NULL, "3mp_core");
	if (IS_ERR(regulator))
		return -ENODEV;
	if (regulator_is_enabled(regulator))
		ret = regulator_force_disable(regulator);
	regulator_put(regulator);

#ifndef USE_CAM_GPIO_CFG
	gpio_free(GPIO_2M_nSTBY);
	gpio_free(GPIO_2M_nRST);
#endif
	return 0;
}

static int sr200pc20_power(int onoff)
{
	int ret = 0;

	printk("%s(): %s \n", __func__, onoff ? "on" : "down");

	if (onoff) {
#ifdef USE_CAM_GPIO_CFG
		if (cfg_gpio_err) {
			printk(KERN_ERR "%s: ERROR: gpio configuration", __func__);
			return cfg_gpio_err;
		}
#endif
		ret = sr200pc20_power_on();
	} else {
		ret = sr200pc20_power_off();
		/* s3c_i2c0_force_stop();*/ /* DSLIM. Should be implemented */
	}

	return ret;
}

static struct sr200pc20_platform_data sr200pc20_plat = {
	.default_width = 800,
	.default_height = 600,
	.pixelformat = V4L2_PIX_FMT_UYVY,
	.is_mipi = 0,
	.streamoff_delay = 0
};

static struct i2c_board_info  sr200pc20_i2c_info = {
	I2C_BOARD_INFO("SR200PC20", 0x40 >> 1),
	.platform_data = &sr200pc20_plat,
};

static struct s3c_platform_camera sr200pc20 = {
	.id		= CAMERA_PAR_A,
	.type		= CAM_TYPE_ITU,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_YCBYCR, /* DSLIM. Need to adjust YUV order */
	.get_i2c_busnum	= sr200pc20_get_i2c_busnum,
	.info		= &sr200pc20_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_UYVY,
	.srclk_name	= "xusbxti",
	.clk_name	= "sclk_cam0",
	.clk_rate	= 24000000,
	.line_length	= 800,
	.width		= 800,
	.height		= 600,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 800,
		.height	= 600,
	},

	/* Polarity */
#ifdef CONFIG_VIDEO_SR200PC20_P4W
	.inv_pclk	= 0,
	.inv_vsync	= 1,
#else
	.inv_pclk	= 1,
	.inv_vsync	= 0,
#endif
	.inv_href	= 0,
	.inv_hsync	= 0,
	.reset_camera	= 0,
	.initialized	= 0,
	.cam_power	= sr200pc20_power,
	.streamoff_delay	= 0,
};
#endif /* CONFIG_VIDEO_SR200PC20 */

#ifdef WRITEBACK_ENABLED
static struct i2c_board_info  writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
	},

	.initialized	= 0,
};
#endif

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
#ifdef CONFIG_ITU_A
	.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
	.default_cam	= CAMERA_PAR_B,
#endif
#ifdef CONFIG_CSI_C
	.default_cam	= CAMERA_CSI_C,
#endif
#ifdef CONFIG_CSI_D
	.default_cam	= CAMERA_CSI_D,
#endif
#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#endif
	.camera		= {
#ifdef CONFIG_VIDEO_S5K5CCGX_COMMON
		&s5k5ccgx,
#endif
#ifdef CONFIG_VIDEO_S5K5BAFX
		&s5k5bafx,
#endif
#ifdef CONFIG_VIDEO_S5K5BBGX
		&s5k5bbgx,
#endif
#ifdef CONFIG_VIDEO_SR200PC20
		&sr200pc20,
#endif
#ifdef WRITEBACK_ENABLED
		&writeback,
#endif
	},
	.hw_ver		= 0x51,
};

ssize_t back_camera_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* Change camera type properly */
	char cam_type[] = "SLSI_S5K5CCGX";

	pr_info("%s\n", __func__);
	return sprintf(buf, "%s\n", cam_type);
}

ssize_t front_camera_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* Change camera type properly */
#if CONFIG_MACH_P8_REV01
	char cam_type[] = "SLSI_S5K5BAFX";
#else
	char cam_type[] = "SILICONFILE_SR200PC20";
#endif

	pr_info("%s\n", __func__);
	return sprintf(buf, "%s\n", cam_type);
}

ssize_t flash_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%s\n", s5k5ccgx_is_flash_on() ? "on" : "off");
}

ssize_t flash_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	switch (*buf) {
	case '0':
		s5k5ccgx_flash_en(S5K5CCGX_FLASH_MODE_MOVIE,
				S5K5CCGX_FLASH_OFF);
		break;
	case '1':
		s5k5ccgx_flash_en(S5K5CCGX_FLASH_MODE_MOVIE,
				S5K5CCGX_FLASH_ON);
		break;
	default:
		pr_err("flash: invalid data=%c(0x%X)\n", *buf, *buf);
		break;
	}

	return count;
}

static struct device_attribute dev_attr_camtype_back =
	__ATTR(type, S_IRUGO, back_camera_type_show, NULL);
static struct device_attribute dev_attr_camtype_front =
	__ATTR(type, S_IRUGO, front_camera_type_show, NULL);
static DEVICE_ATTR(flash, 0664, flash_show, flash_store);

static inline int cam_cfg_init(void)
{
#ifdef CONFIG_VIDEO_S5K5CCGX_COMMON
	return px_cam_cfg_init();
#else
	return 0;
#endif
}

static int cam_create_file(struct class *cls)
{
	struct device *cam_dev_back = NULL;
	struct device *cam_dev_front = NULL;
	int ret = -ENODEV;

	cam_dev_back = device_create(cls, NULL, 0, NULL, "backcam");
	if (IS_ERR(cam_dev_back)) {
		pr_err("cam_init: failed to create device(backcam_dev)\n");
		cam_dev_back = NULL;
		goto front;
	}

	ret = device_create_file(cam_dev_back, &dev_attr_camtype_back);
	if (unlikely(ret < 0)) {
		pr_err("cam_init: failed to create device file, %s\n",
			dev_attr_camtype_back.attr.name);
		device_destroy(cls, 0);
		goto front;
	}

	ret = device_create_file(cam_dev_back, &dev_attr_flash);
	if (unlikely(ret < 0)) {
		pr_err("cam_init: failed to create device file, %s\n",
			dev_attr_flash.attr.name);
	}

front:
	cam_dev_front = device_create(cls, NULL, 0, NULL, "frontcam");
	if (IS_ERR(cam_dev_front)) {
		pr_err("cam_init: failed to create device(frontcam_dev)\n");
		goto out_unreg_class;
	}

	ret = device_create_file(cam_dev_front, &dev_attr_camtype_front);
	if (unlikely(ret < 0)) {
		pr_err("cam_init: failed to create device file, %s\n",
			dev_attr_camtype_front.attr.name);
		goto out_unreg_dev_front;
	}

	return 0;

out_unreg_dev_front:
	device_destroy(cls, 0);
out_unreg_class:
	if (!cam_dev_back)
		class_destroy(cls);

	return -ENODEV;
}

static struct class *camera_class;

/**
 * cam_init - Intialize something concerning camera device if needed.
 *
 * And excute codes about camera needed on boot-up time
 */
static void cam_init(void)
{
	/* pr_info("%s: E\n", __func__); */

	cam_cfg_init();

	camera_class = class_create(THIS_MODULE, "camera");
	if (IS_ERR(camera_class)) {
		pr_err("cam_init: failed to create class\n");
		return;
	}

	/* create device and device file for supporting camera sysfs.*/
	cam_create_file(camera_class);

	pr_info("%s: X\n", __func__);
}
#endif /* CONFIG_VIDEO_FIMC */

static struct resource smdkc210_smsc911x_resources[] = {
	[0] = {
		.start = S5PV310_PA_SROM1,
		.end   = S5PV310_PA_SROM1 + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_EINT(5),
		.end   = IRQ_EINT(5),
		.flags = IORESOURCE_IRQ | IRQF_TRIGGER_HIGH,
	},
};

static struct smsc911x_platform_config smsc9215 = {
	.irq_polarity = SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type = SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags = SMSC911X_USE_16BIT | SMSC911X_FORCE_INTERNAL_PHY,
	.phy_interface = PHY_INTERFACE_MODE_MII,
	.mac = {0x00, 0x80, 0x00, 0x23, 0x45, 0x67},
};

static struct platform_device smdkc210_smsc911x = {
	.name          = "smsc911x",
	.id            = -1,
	.num_resources = ARRAY_SIZE(smdkc210_smsc911x_resources),
	.resource      = smdkc210_smsc911x_resources,
	.dev = {
		.platform_data = &smsc9215,
	},
};

#ifndef CONFIG_MACH_P4W_REV00 /* QCA */
static struct platform_device p4w_wlan_ar6000_pm_device = {
	.name				= "wlan_ar6000_pm_dev",
	.id					= 1,
	.num_resources		= 0,
	.resource			= NULL,
};


static void
(*wlan_status_notify_cb)(struct platform_device *dev_id, int card_present);
struct platform_device *wlan_devid;

static int register_wlan_status_notify
(void (*callback)(struct platform_device *dev_id, int card_present))
{
	wlan_status_notify_cb = callback;
	return 0;
}

static int register_wlan_pdev(struct platform_device *pdev)
{
	wlan_devid = pdev;
	printk(KERN_ERR "ATHR register_wlan_pdev pdev->id = %d\n", pdev->id);
	return 0;
}


#define WLAN_HOST_WAKE
#ifdef WLAN_HOST_WAKE
struct wlansleep_info {
	unsigned host_wake;
	unsigned host_wake_irq;
	struct wake_lock wake_lock;
};

static struct wlansleep_info *wsi;
static struct tasklet_struct hostwake_task;

static void wlan_hostwake_task(unsigned long data)
{
	printk(KERN_INFO "WLAN: wake lock timeout 0.5 sec...\n");

	wake_lock_timeout(&wsi->wake_lock, HZ / 2);
}

static irqreturn_t wlan_hostwake_isr(int irq, void *dev_id)
{
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

static int wlan_host_wake_init(void)
{
	int ret;

	wsi = kzalloc(sizeof(struct wlansleep_info), GFP_KERNEL);
	if (!wsi)
		return -ENOMEM;

	wake_lock_init(&wsi->wake_lock, WAKE_LOCK_SUSPEND, "bluesleep");
	tasklet_init(&hostwake_task, wlan_hostwake_task, 0);

	wsi->host_wake = GPIO_WLAN_HOST_WAKE;
	wsi->host_wake_irq = gpio_to_irq(GPIO_WLAN_HOST_WAKE);

	ret = request_irq(wsi->host_wake_irq, wlan_hostwake_isr,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"wlan hostwake", NULL);
	if (ret	< 0) {
		printk(KERN_ERR "WLAN: Couldn't acquire WLAN_HOST_WAKE IRQ");
		return -1;
	}

	ret = enable_irq_wake(wsi->host_wake_irq);
	if (ret < 0) {
		printk(KERN_ERR "WLAN: Couldn't enable WLAN_HOST_WAKE as wakeup interrupt");
		free_irq(wsi->host_wake_irq, NULL);
		return -1;
	}

	return 0;
}

static void wlan_host_wake_exit(void)
{
	if (disable_irq_wake(wsi->host_wake_irq))
		printk(KERN_ERR "WLAN: Couldn't disable hostwake IRQ wakeup mode\n");
	free_irq(wsi->host_wake_irq, NULL);
	tasklet_kill(&hostwake_task);
	wake_lock_destroy(&wsi->wake_lock);
	kfree(wsi);
}
#endif /* WLAN_HOST_WAKE */

#define GPK3DRV	(S5PV310_VA_GPIO2 + 0xAC)

static void config_wlan_gpio(void)
{
	int ret = 0;
	unsigned int gpio;

	printk(KERN_ERR "ATHR - %s\n", __func__);
	ret = gpio_request(GPIO_WLAN_HOST_WAKE, "wifi_irq");
	if (ret < 0) {
		printk(KERN_ERR "cannot reserve GPIO_WLAN_HOST_WAKE: %s - %d\n"\
				, __func__, GPIO_WLAN_HOST_WAKE);
		gpio_free(GPIO_WLAN_HOST_WAKE);
		return;
	}

	ret = gpio_request(GPIO_WLAN_EN, "wifi_pwr_33");

	if (ret < 0) {
		printk(KERN_ERR "cannot reserve GPIO_WLAN_EN: %s - %d\n"\
				, __func__, GPIO_WLAN_EN);
		gpio_free(GPIO_WLAN_EN);
		return;
	}

#if defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV01)
	if (system_rev >= 4) {
		ret = gpio_request(GPIO_WLAN_EN2, "wifi_pwr_18");

		if (ret < 0) {
			printk(KERN_ERR "cannot reserve GPIO_WLAN_EN2: "\
					"%s - %d\n", __func__, GPIO_WLAN_EN2);
			gpio_free(GPIO_WLAN_EN2);
			return;
		}
	}
#elif defined(CONFIG_MACH_P8_REV01)
	if (system_rev >= 7) {
		ret = gpio_request(GPIO_WLAN_EN2, "wifi_pwr_18");

		if (ret < 0) {
			printk(KERN_ERR "cannot reserve GPIO_WLAN_EN2: "\
					"%s - %d\n", __func__, GPIO_WLAN_EN2);
			gpio_free(GPIO_WLAN_EN2);
			return;
		}
	}
#elif defined(CONFIG_MACH_P8LTE_REV00)
	if (system_rev >= 6) {
		ret = gpio_request(GPIO_WLAN_EN2, "wifi_pwr_18");

		if (ret < 0) {
			printk(KERN_ERR "cannot reserve GPIO_WLAN_EN2: "\
					"%s - %d\n", __func__, GPIO_WLAN_EN2);
			gpio_free(GPIO_WLAN_EN2);
			return;
		}
	}
#endif

	ret = gpio_request(GPIO_WLAN_nRST, "wifi_rst");

	if (ret < 0) {
		printk(KERN_ERR "cannot reserve GPIO_WLAN_nRST: %s - %d\n"\
				, __func__, GPIO_WLAN_nRST);
		gpio_free(GPIO_WLAN_nRST);
		return;
	}

	/* SDIO GPIO pad configuraiton */
	s3c_gpio_setpull(S5PV310_GPK3(1), S3C_GPIO_PULL_UP);
	/* Data pin GPK3[3:6] to special-function 2 */
	for (gpio = S5PV310_GPK3(3); gpio <= S5PV310_GPK3(6); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(2));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	}
	__raw_writel(0x3FFF, GPK3DRV);

	gpio_direction_output(GPIO_WLAN_nRST, 0);
	gpio_direction_output(GPIO_WLAN_EN, 1);
#if defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV01)
	if (system_rev >= 4)
		gpio_direction_output(GPIO_WLAN_EN2, 0);
#elif defined(CONFIG_MACH_P8_REV01)
	if (system_rev >= 7)
		gpio_direction_output(GPIO_WLAN_EN2, 0);
#elif defined(CONFIG_MACH_P8LTE_REV00)
	if (system_rev >= 6)
		gpio_direction_output(GPIO_WLAN_EN2, 0);
#endif
}

void
wlan_setup_power(int on, int detect)
{
	printk(KERN_ERR "ATHR - %s %s --enter\n", __func__, on ? "on" : "off");

	if (on) {
		/* WAR for nRST is high */

#if defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV01)
		if (system_rev >= 4) {
			gpio_direction_output(GPIO_WLAN_EN2, 1);
			udelay(10);
		}
#elif defined(CONFIG_MACH_P8_REV01)
		if (system_rev >= 7) {
			gpio_direction_output(GPIO_WLAN_EN2, 1);
			udelay(10);
		}
#elif defined(CONFIG_MACH_P8LTE_REV00)
		if (system_rev >= 6) {
			gpio_direction_output(GPIO_WLAN_EN2, 1);
			udelay(10);
		}
#endif
		gpio_direction_output(GPIO_WLAN_nRST, 0);
		mdelay(30);
		gpio_direction_output(GPIO_WLAN_nRST, 1);

#ifdef WLAN_HOST_WAKE
		wlan_host_wake_init();
#endif /* WLAN_HOST_WAKE */

	} else {
#ifdef WLAN_HOST_WAKE
		wlan_host_wake_exit();
#endif /* WLAN_HOST_WAKE */

		gpio_direction_output(GPIO_WLAN_nRST, 0);
#if defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV01)
		if (system_rev >= 4)
			gpio_direction_output(GPIO_WLAN_EN2, 0);
#elif defined(CONFIG_MACH_P8_REV01)
		if (system_rev >= 7)
			gpio_direction_output(GPIO_WLAN_EN2, 0);
#elif defined(CONFIG_MACH_P8LTE_REV00)
		if (system_rev >= 6)
			gpio_direction_output(GPIO_WLAN_EN2, 0);
#endif
	}

	mdelay(100);

	printk(KERN_ERR "ATHR - rev : %02d\n", system_rev);

#if defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV01)
	if (system_rev >= 4) {
		printk(KERN_ERR "ATHR - GPIO_WLAN_EN1(%d: %d), "\
			"GPIO_WLAN_EN2(%d: %d), GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_EN2, gpio_get_value(GPIO_WLAN_EN2)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
	} else {
		printk(KERN_ERR "ATHR - GPIO_WLAN_EN(%d: %d), "\
			" GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
	}
#elif defined(CONFIG_MACH_P8_REV01)
	if (system_rev >= 7) {
		printk(KERN_ERR "ATHR - GPIO_WLAN_EN1(%d: %d), "\
			"GPIO_WLAN_EN2(%d: %d), GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_EN2, gpio_get_value(GPIO_WLAN_EN2)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
	} else {
		printk(KERN_ERR "ATHR - GPIO_WLAN_EN(%d: %d), "\
			" GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
	}
#elif defined(CONFIG_MACH_P8LTE_REV00)
	if (system_rev >= 6) {
		printk(KERN_ERR "ATHR - GPIO_WLAN_EN1(%d: %d), "\
			"GPIO_WLAN_EN2(%d: %d), GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_EN2, gpio_get_value(GPIO_WLAN_EN2)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
	} else {
		printk(KERN_ERR "ATHR - GPIO_WLAN_EN(%d: %d), "\
			" GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
	}
#else
	printk(KERN_ERR "ATHR - GPIO_WLAN_EN(%d: %d), GPIO_WALN_nRST(%d: %d)\n"\
			, GPIO_WLAN_EN, gpio_get_value(GPIO_WLAN_EN)
			, GPIO_WLAN_nRST, gpio_get_value(GPIO_WLAN_nRST));
#endif

	if (detect) {
		if (wlan_status_notify_cb)
			wlan_status_notify_cb(wlan_devid, on);
		else
			printk(KERN_ERR "ATHR - WLAN: No notify available\n");
	}
}
EXPORT_SYMBOL(wlan_setup_power);

#endif

#ifdef CONFIG_REGULATOR_MAX8998
static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("vusb_1.1v", NULL),
	REGULATOR_SUPPLY("ldo3", NULL),
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("ldo7", NULL),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("vusb_3.3v", NULL),
	REGULATOR_SUPPLY("ldo8", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("ldo17", NULL),
};

static struct regulator_consumer_supply buck1_supply[] = {
	REGULATOR_SUPPLY("buck1", NULL),
};

static struct regulator_consumer_supply buck2_supply[] = {
	REGULATOR_SUPPLY("buck2", NULL),
};

static struct regulator_init_data ldo3_init_data = {
	.constraints	= {
		.name		= "ldo3 range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo3_supply[0],
};

static struct regulator_init_data ldo7_init_data = {
	.constraints	= {
		.name		= "ldo7 range",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1200000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo7_supply[0],
};

static struct regulator_init_data ldo8_init_data = {
	.constraints	= {
		.name		= "ldo8 range",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo8_supply[0],
};

static struct regulator_init_data ldo17_init_data = {
	.constraints	= {
		.name		= "ldo17 range",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &ldo17_supply[0],
};

static struct regulator_init_data buck1_init_data = {
	.constraints	= {
		.name		= "buck1 range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck1_supply[0],
};

static struct regulator_init_data buck2_init_data = {
	.constraints	= {
		.name		= "buck2 range",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.always_on	= 1,
		.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck2_supply[0],
};

static struct max8998_regulator_data max8998_regulators[] = {
	{ MAX8998_LDO3, &ldo3_init_data, },
	{ MAX8998_LDO7, &ldo7_init_data, },
	{ MAX8998_LDO8, &ldo8_init_data, },
	{ MAX8998_LDO17, &ldo17_init_data, },
	{ MAX8998_BUCK1, &buck1_init_data, },
	{ MAX8998_BUCK2, &buck2_init_data, },
};

static struct max8998_power_data max8998_power = {
	.batt_detect = 1,
};

static struct max8998_platform_data s5pv310_max8998_info = {
	.num_regulators = ARRAY_SIZE(max8998_regulators),
	.regulators	= max8998_regulators,
	.irq_base	= IRQ_BOARD_START,
	.buck1_max_voltage1 = 1100000,
	.buck1_max_voltage2 = 1100000,
	.buck2_max_voltage = 1100000,
	.buck1_set1 = GPIO_BUCK1_EN_A,
	.buck1_set2 = GPIO_BUCK1_EN_B,
	.buck2_set3 = GPIO_BUCK2_EN,
	.power = &max8998_power,
};
#endif /* CONFIG_REGULATOR_MAX8998 */

#ifdef CONFIG_REGULATOR_MAX8997
static struct regulator_consumer_supply ldo1_supply[] = {
	REGULATOR_SUPPLY("vadc_3.3v", NULL),
};

static struct regulator_consumer_supply ldo3_supply[] = {
	REGULATOR_SUPPLY("vusb_1.1v", "s5p-ehci"),
	REGULATOR_SUPPLY("vusb_1.1v", "usb_otg"),
	REGULATOR_SUPPLY("vmipi_1.1v", "m5mo"),
	REGULATOR_SUPPLY("vmipi_1.1v", NULL),
};

static struct regulator_consumer_supply ldo4_supply[] = {
	REGULATOR_SUPPLY("vmipi_1.8v", NULL),
};

static struct regulator_consumer_supply ldo5_supply[] = {
	REGULATOR_SUPPLY("vhsic", NULL),
};

static struct regulator_consumer_supply ldo7_supply[] = {
	REGULATOR_SUPPLY("vt_core_1.5v", NULL),
	REGULATOR_SUPPLY("vt_core_1.8v", NULL),
};

static struct regulator_consumer_supply ldo8_supply[] = {
	REGULATOR_SUPPLY("vusb_3.3v", NULL),
};

static struct regulator_consumer_supply ldo10_supply[] = {
	REGULATOR_SUPPLY("vpll_1.2v", NULL),
};

static struct regulator_consumer_supply ldo11_supply[] = {
	REGULATOR_SUPPLY("hdp_2.8v", NULL),
};

static struct regulator_consumer_supply ldo12_supply[] = {
	REGULATOR_SUPPLY("cam_io_1.8v", NULL),
};

static struct regulator_consumer_supply ldo13_supply[] = {
	REGULATOR_SUPPLY("cam_analog_2.8v", NULL),
};

static struct regulator_consumer_supply ldo14_supply[] = {
	REGULATOR_SUPPLY("vmotor", NULL),
};

static struct regulator_consumer_supply ldo15_supply[] = {
	REGULATOR_SUPPLY("vled_3.3v", NULL),
};

static struct regulator_consumer_supply ldo16_supply[] = {
	REGULATOR_SUPPLY("irda_2.8v", NULL),
};

static struct regulator_consumer_supply ldo17_supply[] = {
	REGULATOR_SUPPLY("3m_af_2.8v", NULL),
};

static struct regulator_consumer_supply ldo18_supply[] = {
	REGULATOR_SUPPLY("vtf_2.8v", NULL),
};

static struct regulator_consumer_supply ldo21_supply[] = {
	REGULATOR_SUPPLY("vddq_m1m2", NULL),
};

static struct regulator_consumer_supply buck1_supply[] = {
	REGULATOR_SUPPLY("vdd_arm", NULL),
};

static struct regulator_consumer_supply buck2_supply[] = {
	REGULATOR_SUPPLY("vdd_int", NULL),
};

static struct regulator_consumer_supply buck3_supply[] = {
	REGULATOR_SUPPLY("vdd_g3d", NULL),
};

static struct regulator_consumer_supply buck4_supply[] = {
	REGULATOR_SUPPLY("3mp_core", NULL),
};

static struct regulator_consumer_supply buck7_supply[] = {
	REGULATOR_SUPPLY("vcc_sub", NULL),
};

static struct regulator_consumer_supply safeout1_supply[] = {
	REGULATOR_SUPPLY("safeout1", NULL),
};

static struct regulator_consumer_supply safeout2_supply[] = {
	REGULATOR_SUPPLY("safeout2", NULL),
};

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask,\
		_disabled) \
	static struct regulator_init_data _ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled	= _disabled,			\
				.enabled	= !(_disabled),		\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(_ldo##_supply),	\
		.consumer_supplies = &_ldo##_supply[0],			\
	};
REGULATOR_INIT(ldo1, "VADC_3.3V_C210", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo3, "VUSB_1.1V", 1100000, 1100000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo4, "VMIPI_1.8V", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo5, "VHSIC_1.2V", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);

#if defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01) \
	|| defined(CONFIG_MACH_P8LTE_REV00)
REGULATOR_INIT(ldo7, "VT_CORE_1.5V", 1500000, 1500000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#else
REGULATOR_INIT(ldo7, "VT_CORE_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#endif

REGULATOR_INIT(ldo8, "VUSB_3.3V", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo10, "VPLL_1.2V", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo11, "VCC_2.8V_HPD", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo12, "CAM_IO_1.8V", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo13, "CAM_ANALOG_2.8V", 2800000, 2800000, 0,
                REGULATOR_CHANGE_STATUS, 1);

#if defined(CONFIG_MACH_P2_REV02)
REGULATOR_INIT(ldo14, "VCC_3.0V_MOTOR", 2400000, 2400000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#elif defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01)
REGULATOR_INIT(ldo14, "VCC_3.0V_MOTOR", 3100000, 3100000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#else
REGULATOR_INIT(ldo14, "VCC_3.0V_MOTOR", 3000000, 3000000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#endif

#if defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01)
REGULATOR_INIT(ldo15, "VLED_3.3V", 3200000, 3200000, 1,
		REGULATOR_CHANGE_STATUS, 0);
#else
REGULATOR_INIT(ldo15, "VLED_3.3V", 3300000, 3300000, 0,
		REGULATOR_CHANGE_STATUS, 1);
#endif

REGULATOR_INIT(ldo16, "IRDA_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo17, "3M_AF_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo18, "VTF_2.8V", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo21, "VDDQ_M1M2_1.2V", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);

static struct regulator_init_data buck1_init_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.always_on	= 1,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1250000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck1_supply[0],
};

static struct regulator_init_data buck2_init_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		= 650000,
		.max_uV		= 2225000,
		.always_on	= 1,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck2_supply[0],
};

static struct regulator_init_data buck3_init_data = {
	.constraints	= {
		.name		= "G3D_1.1V",
		.min_uV		= 900000,
		.max_uV		= 1200000,
		.always_on	= 0,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck3_supply[0],
};

static struct regulator_init_data buck4_init_data = {
	.constraints	= {
		.name		= "3MP_CORE_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck4_supply[0],
};

static struct regulator_init_data buck5_init_data = {
	.constraints	= {
		.name		= "VMEM_1.2V",
		.min_uV		= 1200000,
		.max_uV		= 1200000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.uV		= 1200000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled = 1,
		},
	},
};

static struct regulator_init_data buck7_init_data = {
	.constraints	= {
		.name		= "VCC_SUB_2.0V",
		.min_uV		= 2000000,
		.max_uV		= 2000000,
		.apply_uV	= 1,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &buck7_supply[0],
};

static struct regulator_init_data safeout1_init_data = {
        .constraints    = {
                .name           = "safeout1 range",
                .valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on	= 1,
		.boot_on	= 1,
		.state_mem      = {
			.enabled = 1,
			},
        },
        .num_consumer_supplies  = ARRAY_SIZE(safeout1_supply),
        .consumer_supplies      = safeout1_supply,
};

static struct regulator_init_data safeout2_init_data = {
	.constraints	= {
		.name		= "safeout2 range",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on	= 0,
		.boot_on	= 0,
		.state_mem	= {
			.enabled = 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(safeout2_supply),
	.consumer_supplies	= safeout2_supply,
};

static struct max8997_regulator_data max8997_regulators[] = {
	{ MAX8997_LDO1,	 &ldo1_init_data, NULL, },
	{ MAX8997_LDO3,  &ldo3_init_data, NULL, },
	{ MAX8997_LDO4,  &ldo4_init_data, NULL, },
	{ MAX8997_LDO5,  &ldo5_init_data, NULL, },
	{ MAX8997_LDO7,  &ldo7_init_data, NULL, },
	{ MAX8997_LDO8,  &ldo8_init_data, NULL, },
	{ MAX8997_LDO10, &ldo10_init_data, NULL, },
	{ MAX8997_LDO11, &ldo11_init_data, NULL, },
	{ MAX8997_LDO12, &ldo12_init_data, NULL, },
	{ MAX8997_LDO13, &ldo13_init_data, NULL, },
	{ MAX8997_LDO14, &ldo14_init_data, NULL, },
	{ MAX8997_LDO15, &ldo15_init_data, NULL, },
	{ MAX8997_LDO16, &ldo16_init_data, NULL, },
	{ MAX8997_LDO17, &ldo17_init_data, NULL, },
	{ MAX8997_LDO18, &ldo18_init_data, NULL, },
	{ MAX8997_LDO21, &ldo21_init_data, NULL, },
	{ MAX8997_BUCK1, &buck1_init_data, NULL, },
	{ MAX8997_BUCK2, &buck2_init_data, NULL, },
	{ MAX8997_BUCK3, &buck3_init_data, NULL, },
	{ MAX8997_BUCK4, &buck4_init_data, NULL, },
	{ MAX8997_BUCK5, &buck5_init_data, NULL, },
	{ MAX8997_BUCK7, &buck7_init_data, NULL, },
	{ MAX8997_ESAFEOUT1, &safeout1_init_data, NULL, },
	{ MAX8997_ESAFEOUT2, &safeout2_init_data, NULL, },
};

void max8997_disable_ldo8_on_sleep(bool disabled)
{
	ldo8_init_data.constraints.state_mem.disabled = disabled;
	ldo8_init_data.constraints.state_mem.enabled = !disabled;
}

static int max8997_power_set_charger(int insert)
{
	struct power_supply *psy = power_supply_get_by_name("battery");
	union power_supply_propval value;

	if (!psy) {
		pr_err("%s: fail to get battery ps\n", __func__);
		return -ENODEV;
	}

	if (insert)
		value.intval = POWER_SUPPLY_TYPE_MAINS;
	else
		value.intval = POWER_SUPPLY_TYPE_BATTERY;

	if (psy->set_property)
		return psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);
	else
		return -EPERM;
}

static struct max8997_power_data max8997_power = {
	.set_charger = max8997_power_set_charger,
	.batt_detect = 1,
};

#ifdef CONFIG_VIBETONZ
static struct max8997_motor_data max8997_motor = {
	.max_timeout = 10000,
#if defined(CONFIG_MACH_P2_REV02)
	.duty = 44707,
	.period = 45159,
#elif defined(CONFIG_MACH_P8_REV01) || defined(CONFIG_MACH_P8LTE_REV00)
	.duty = 38288,
	.period = 38676,
#elif defined(CONFIG_MACH_P4W_REV01)
	.duty = 37000,
	.period = 38675,
#else
	.duty = 38250,
	.period = 38296,
#endif
	.init_hw = NULL,
	.motor_en = NULL,
	.pwm_id = 1,
};
#endif

static struct max8997_buck1_dvs_funcs *buck1_dvs_funcs;

void max8997_set_arm_voltage_table(int *voltage_table, int arr_size)
{
	pr_info("%s\n", __func__);
	if (buck1_dvs_funcs && buck1_dvs_funcs->set_buck1_dvs_table)
		buck1_dvs_funcs->set_buck1_dvs_table(buck1_dvs_funcs,
				voltage_table, arr_size);
}

static void max8997_register_buck1dvs_funcs(struct max8997_buck1_dvs_funcs *ptr)
{
	buck1_dvs_funcs = ptr;
}

static struct max8997_platform_data s5pv310_max8997_info = {
	.num_regulators = ARRAY_SIZE(max8997_regulators),
	.regulators	= &max8997_regulators[0],
	.irq_base	= IRQ_BOARD_START,
	.wakeup		= 1,
	.buck1_gpiodvs	= false,
	.buck1_max_vol	= 1350000,
	.buck2_max_vol	= 1150000,
	.buck5_max_vol	= 1200000,
	.buck_set1 = GPIO_BUCK1_EN_A,
	.buck_set2 = GPIO_BUCK1_EN_B,
	.buck_set3 = GPIO_BUCK2_EN,
	.buck_ramp_en = true,
	.buck_ramp_delay = 10,		/* 10.00mV /us (default) */
	.flash_cntl_val = 0x5F,		/* Flash safety timer duration: 800msec,
					   Maximum timer mode */
	.power = &max8997_power,
#ifdef CONFIG_VIBETONZ
	.motor = &max8997_motor,
#endif
	.register_buck1_dvs_funcs = max8997_register_buck1dvs_funcs,
};
#endif /* CONFIG_REGULATOR_MAX8997 */

#ifdef CONFIG_MPU_SENSORS_MPU3050

extern struct class *sec_class;

/* we use a skeleton to provide some information needed by MPL
 * but we don't use the suspend/resume/read functions so we
 * don't initialize them so that mldl_cfg.c doesn't try to
 * control it directly.  we have a separate mag driver instead.
 */
static struct mpu3050_platform_data mpu3050_pdata = {
	.int_config  = 0x12,
	/* Orientation for MPU.  Part is mounted rotated
	 * 90 degrees counter-clockwise from natural orientation.
	 * So X & Y are swapped and Y is negated.
	 */
#if defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01)
	.orientation = {0, 1, 0,
			1, 0, 0,
			0, 0, -1},
#elif defined(CONFIG_MACH_P8LTE_REV00)
	.orientation = {0, -1, 0,
			1, 0, 0,
			0, 0, 1},
#elif defined(CONFIG_MACH_P2_REV02)
	.orientation = {0, 1, 0,
			1, 0, 0,
			0, 0, -1},
#elif defined(CONFIG_MACH_P4W_REV00) || defined(CONFIG_MACH_P4W_REV01) ||defined(CONFIG_MACH_P4W_REV02)
	.orientation = {1 , 0, 0,
			0, -1, 0,
			0, 0, -1},
#else
	.orientation = {0, -1, 0,
			-1, 0, 0,
			0, 0, -1},
#endif
	.level_shifter = 0,
	.accel = {
		.get_slave_descr = kxtf9_get_slave_descr,
		.irq		= 0,	//not used
		.adapt_num	= 1,
		.bus		= EXT_SLAVE_BUS_SECONDARY,
		.address	= 0x0F,
		/* Orientation for the Acc.  Part is mounted rotated
		 * 180 degrees from natural orientation.
		 * So X & Y are both negated.
		 */
#if defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01)
		.orientation = {0, 1, 0,
				1, 0, 0,
				0, 0, -1},
#elif defined(CONFIG_MACH_P8LTE_REV00)
		.orientation = {0, 1, 0,
				-1, 0, 0,
				0, 0, 1},
#elif defined(CONFIG_MACH_P2_REV02)
		.orientation = {0, 1, 0,
				1, 0, 0,
				0, 0, -1},
#elif defined(CONFIG_MACH_P4W_REV00) || defined(CONFIG_MACH_P4W_REV01) ||defined(CONFIG_MACH_P4W_REV02)
		.orientation = {0, -1, 0,
				-1, 0, 0,
				0, 0, -1},
#else
		/* Rotate Accel Orientation for CL339008 */
		.orientation = {0, -1, 0,
				-1, 0, 0,
				0, 0, -1},
#endif
	},

	.compass = {
		.get_slave_descr = NULL,
		.adapt_num	= 7,	/*bus number 7*/
		.bus		= EXT_SLAVE_BUS_PRIMARY,
		.address	= 0x0C,
		/* Orientation for the Mag.  Part is mounted rotated
		 * 90 degrees clockwise from natural orientation.
		 * So X & Y are swapped and Y & Z are negated.
		 */
		.orientation = {0, -1, 0,
				1, 0, 0,
				0, 0, 1},
	},

};


static void ak8975_init(void)
{
	gpio_request(GPIO_MSENSE_INT, "ak8975_int");
	gpio_direction_input(GPIO_MSENSE_INT);
}

static void mpu3050_init(void)
{
	gpio_request(GPIO_GYRO_INT, "mpu3050_int");
	gpio_direction_input(GPIO_GYRO_INT);
	//mpu3050_pdata.sec_class = sec_class;
}

static const struct i2c_board_info i2c_mpu_sensor_board_info[] = {
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq = IRQ_EINT(0),
		.platform_data = &mpu3050_pdata,
	},
#if 0
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
	},
#endif
};
#endif   //CONFIG_MPU_SENSORS_MPU3050

#if defined(CONFIG_SMB136_CHARGER) || defined(CONFIG_SMB347_CHARGER)
struct smb_charger_callbacks *smb_callbacks;

static void smb_charger_register_callbacks(
		struct smb_charger_callbacks *ptr)
{
	smb_callbacks = ptr;
}

static void smb_charger_unregister_callbacks(void)
{
	smb_callbacks = NULL;
}

static struct smb_charger_data smb_charger_pdata = {
	.register_callbacks = smb_charger_register_callbacks,
	.unregister_callbacks = smb_charger_unregister_callbacks,
	.enable = GPIO_TA_EN,
	.stat = GPIO_TA_nCHG,
#if defined(CONFIG_MACH_P4W_REV00) || defined(CONFIG_MACH_P4W_REV01) || defined(CONFIG_MACH_P4W_REV02)
	.ta_nconnected = GPIO_TA_nCONNECTED,
#else
	.ta_nconnected = NULL,
#endif
};

static struct i2c_board_info i2c_devs12_emul[] __initdata = {
	{
#if defined(CONFIG_SMB347_CHARGER)
		I2C_BOARD_INFO("smb347-charger",  0x0C >> 1),
#else
		I2C_BOARD_INFO("smb136-charger",  0x9A >> 1),
#endif
		.platform_data = &smb_charger_pdata,
	},
};
static void __init smb_gpio_init(void)
{
	s3c_gpio_cfgpin(GPIO_TA_nCHG, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_TA_nCHG, S3C_GPIO_PULL_NONE); // external pull up
	i2c_devs12_emul[0].irq = gpio_to_irq(GPIO_TA_nCHG);
}

static struct i2c_gpio_platform_data gpio_i2c_data12 = {
	.sda_pin		= GPIO_CHG_SDA,
	.scl_pin		= GPIO_CHG_SCL,
};

static struct platform_device s3c_device_i2c12 = {
	.name			= "i2c-gpio",
	.id			= 12,
	.dev.platform_data	= &gpio_i2c_data12,
};

static void sec_bat_set_charging_state(int enable, int cable_status)
{
	if (smb_callbacks && smb_callbacks->set_charging_state)
		smb_callbacks->set_charging_state(enable, cable_status);
}

static int sec_bat_get_charging_state(void)
{
	if (smb_callbacks && smb_callbacks->get_charging_state)
		return smb_callbacks->get_charging_state();
	else
		return 0;
}

static void sec_bat_set_charging_current(int set_current)
{
	if (smb_callbacks && smb_callbacks->set_charging_current)
		smb_callbacks->set_charging_current(set_current);
}

static int sec_bat_get_charging_current(void)
{
	if (smb_callbacks && smb_callbacks->get_charging_current)
		return smb_callbacks->get_charging_current();
	else
		return 0;
}
#endif

static int check_bootmode(void)
{
	return __raw_readl(S5P_INFORM2);
}

static int check_jig_on(void)
{
	return !gpio_get_value(GPIO_IF_CON_SENSE);
}

static DEFINE_SPINLOCK(mic_bias_lock);
static bool mc1n2_mainmic_bias;
static bool mc1n2_submic_bias;

static void set_shared_mic_bias(void)
{
	gpio_set_value(GPIO_MIC_BIAS_EN, mc1n2_mainmic_bias || mc1n2_submic_bias);
}

void sec_set_sub_mic_bias(bool on)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
	unsigned long flags;
	spin_lock_irqsave(&mic_bias_lock, flags);
	mc1n2_submic_bias = on;

	#ifdef CONFIG_MACH_P8_REV01
		gpio_set_value(GPIO_MIC_BIAS_EN, mc1n2_submic_bias);
	#else
	set_shared_mic_bias();
	#endif

	spin_unlock_irqrestore(&mic_bias_lock, flags);
#endif
}


void sec_set_main_mic_bias(bool on)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
	unsigned long flags;
	spin_lock_irqsave(&mic_bias_lock, flags);
	mc1n2_mainmic_bias = on;
	#ifdef CONFIG_MACH_P8_REV01
		gpio_set_value(GPIO_MAIN_MIC_BIAS_EN, mc1n2_mainmic_bias);
	#else
	set_shared_mic_bias();
	#endif
	spin_unlock_irqrestore(&mic_bias_lock, flags);
#endif
}

void sec_set_ldo1_constraints(int disabled)
{
#if 0 /* VADC_3.3V_C210 is always on */
	/* VDD33_ADC */
	ldo1_init_data.constraints.state_mem.disabled = disabled;
	ldo1_init_data.constraints.state_mem.enabled = !disabled;
#endif
}

static struct mc1n2_platform_data mc1n2_pdata = {
	.set_main_mic_bias = sec_set_main_mic_bias,
	.set_sub_mic_bias = sec_set_sub_mic_bias,
	.set_adc_power_contraints = sec_set_ldo1_constraints,
};

static void c1_sound_init(void)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
	int err;

	#ifdef CONFIG_MACH_P8_REV01
	err = gpio_request(GPIO_MAIN_MIC_BIAS_EN, "GPC0");
	if (err) {
		pr_err(KERN_ERR "MAIN_MIC_BIAS_EN GPIO set error!\n");
		return;
	}

	gpio_direction_output(GPIO_MAIN_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_MAIN_MIC_BIAS_EN, 0);
	gpio_free(GPIO_MAIN_MIC_BIAS_EN);
	#endif

	err = gpio_request(GPIO_MIC_BIAS_EN, "GPE1");
	if (err) {
		pr_err(KERN_ERR "MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_MIC_BIAS_EN, 0);
	gpio_free(GPIO_MIC_BIAS_EN);

	err = gpio_request(GPIO_EAR_MIC_BIAS_EN, "GPE2");
	if (err) {
		pr_err(KERN_ERR "EAR_MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_EAR_MIC_BIAS_EN, 1);
	gpio_set_value(GPIO_EAR_MIC_BIAS_EN, 0);
	gpio_free(GPIO_EAR_MIC_BIAS_EN);

#if 0
	err = gpio_request(GPIO_SUB_MIC_BIAS_EN, "submic_bias");
	if (err) {
		pr_err(KERN_ERR "SUB_MIC_BIAS_EN GPIO set error!\n");
		return;
	}
	gpio_direction_output(GPIO_SUB_MIC_BIAS_EN, 0);
	gpio_free(GPIO_SUB_MIC_BIAS_EN);
#endif

#endif /* #ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS */
}


/* Ir-LED */
#ifdef CONFIG_IRDA

static struct platform_device ir_remote_device = {
	.name = "ir_rc",
	.id = 0,
	.dev = {
	},
};

#if defined(CONFIG_MACH_P2_REV00) || defined(CONFIG_MACH_P2_REV01) || defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV01)
static void ir_rc_init_hw(void)
{
	s3c_gpio_cfgpin(GPIO_IRDA_CONTROL,S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_IRDA_CONTROL,S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_IRDA_CONTROL,0);
}
#endif

#if defined(CONFIG_MACH_P8LTE_REV00) || defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01)
static void ir_rc_init_hw(void)
{
	s3c_gpio_cfgpin(GPIO_IRDA_nINT,S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_IRDA_nINT,S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_IRDA_nINT,0);

	s3c_gpio_cfgpin(GPIO_IRDA_EN,S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_IRDA_EN,S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_IRDA_EN,0);
}
#endif

#endif
/*Ir-LED*/

#if defined(CONFIG_TOUCHSCREEN_MXT768E) || defined(CONFIG_TOUCHSCREEN_MXT1386) || defined(CONFIG_TOUCHSCREEN_MELFAS)

#ifdef CONFIG_TOUCHSCREEN_MXT768E

static void ts_power_on(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, GPIO_LEVEL_HIGH);
	mdelay(70);
	s3c_gpio_setpull(GPIO_TSP_INT_18V, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_TSP_INT_18V, S3C_GPIO_SFN(0xf));
	mdelay(40);
	printk("mxt_power_on is finished\n");

}

static void ts_power_off(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_INT_18V, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TSP_INT_18V, S3C_GPIO_PULL_DOWN);

	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, GPIO_LEVEL_LOW);
	printk("mxt_power_off is finished\n");
}

static void ts_register_callback(void *function)
{
	printk("mxt_register_callback\n");
	charging_cbs.tsp_set_charging_cable = function;
}

static void ts_read_ta_status(bool * ta_status)
{
	*ta_status = is_cable_attached;
}


// Configuration for MXT

#define MXT224_MAX_MT_FINGERS		10
#define MXT224_ATCHCALST		9
#define MXT224_ATCHCALTHR		30

/*
	Configuration for MXT768-E REV_G
*/

#define MXT224_THRESHOLD_BATT		50
#define MXT224_THRESHOLD_CHRG		55
#define MXT224_CALCFG_BATT		80
#define MXT224_CALCFG_CHRG		112
#define MXT224_ATCHFRCCALTHR_WAKEUP		0
#define MXT224_ATCHFRCCALRATIO_WAKEUP		0
#define MXT224_ATCHFRCCALTHR_NORMAL		40
#define MXT224_ATCHFRCCALRATIO_NORMAL		55

static u8 t7_config[] = { GEN_POWERCONFIG_T7,
	32, 255, 50
};

static u8 t8_config[] = { GEN_ACQUISITIONCONFIG_T8,
	64, 0, 20, 20, 0, 0, 20, 0, MXT224_ATCHFRCCALTHR_WAKEUP,
	MXT224_ATCHFRCCALRATIO_WAKEUP
};

static u8 t9_config[] = { TOUCH_MULTITOUCHSCREEN_T9,
	139, 0, 0, 24, 32, 0, 192, MXT224_THRESHOLD_BATT, 2, 1, 0, 10, 1,
	46, MXT224_MAX_MT_FINGERS, 5, 40, 10, /*32, 3, */ 0, 4,
	/*0, 5, */ 0, 4, 10, 10, 245, 245, 143, 40, 143, 80, 18, 15, 0, 0, 2
};

static u8 t15_config[] = { TOUCH_KEYARRAY_T15,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static u8 t18_config[] = { SPT_COMCONFIG_T18,
	0, 0
};

static u8 t19_config[] = { SPT_GPIOPWM_T19,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
/*
static u8 t23_config[] = { TOUCH_PROXIMITY_T23,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
*/
static u8 t25_config[] = { SPT_SELFTEST_T25,
	0, 0, 188, 52, 124, 21, 188, 52, 124, 21, 0, 0, 0, 0
};

static u8 t40_config[] = { PROCI_GRIPSUPPRESSION_T40,
	0, 0, 0, 0, 0
};

static u8 t42_config[] = { PROCI_TOUCHSUPPRESSION_T42,
	0, 32, 120, 100, 0, 0, 0, 0
};

static u8 t46_config[] = { SPT_CTECONFIG_T46,
	0, 0, 8, 28, 0, 0, 1, 0
};

static u8 t47_config[] = { PROCI_STYLUS_T47,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*
static u8 t48_config[] = { PROCG_NOISESUPPRESSION_T48,
	0, 0, MXT224_CALCFG_BATT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0
};
*/
static u8 t48_config[] = {PROCG_NOISESUPPRESSION_T48,
				0, 0, MXT224_CALCFG_BATT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				6,	6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 5,
				0, 0, 0, 0, 0, 0, 32, MXT224_THRESHOLD_BATT, 2, 3, 1, 46, MXT224_MAX_MT_FINGERS, 5, 40, 10, 10,
				10, 10, 143, 40, 143, 80, 18, 15, 2, 32, MXT224_THRESHOLD_BATT, 2, 3, 1, 46, MXT224_MAX_MT_FINGERS, 5, 40, 10, 10,
				10, 10, 143, 40, 143, 80, 18, 15, 2 };


static u8 end_config[] = { RESERVED_T255 };

static const u8 *mxt224_config[] = {
	t7_config,
	t8_config,
	t9_config,
	t15_config,
	t18_config,
	t19_config,
//	t23_config,
	t25_config,
	t40_config,
	t42_config,
	t46_config,
	t47_config,
	t48_config,
	end_config,
};


/*
	Configuration for MXT768-E
*/
#define MXT768E_MAX_MT_FINGERS		10
#define MXT768E_CHRGTIME_BATT		45
#define MXT768E_CHRGTIME_CHRG		45
#define MXT768E_THRESHOLD_BATT		55
#define MXT768E_THRESHOLD_CHRG		50
#define MXT768E_CALCFG_BATT		210
#define MXT768E_CALCFG_CHRG		242
#define MXT768E_ATCHFRCCALTHR_WAKEUP		8
#define MXT768E_ATCHFRCCALRATIO_WAKEUP		180
#define MXT768E_ATCHFRCCALTHR_NORMAL		40
#define MXT768E_ATCHFRCCALRATIO_NORMAL		55
#define MXT768E_IDLESYNCSPERX_BATT		48
#define MXT768E_IDLESYNCSPERX_CHRG		36
#define MXT768E_ACTVSYNCSPERX_BATT		48
#define MXT768E_ACTVSYNCSPERX_CHRG		36
#define MXT768E_IDLEACQINT_BATT			45
#define MXT768E_IDLEACQINT_CHRG			255

static u8 t7_config_e[] = { GEN_POWERCONFIG_T7,
	MXT768E_IDLEACQINT_BATT, 255, 7
};

static u8 t8_config_e[] = { GEN_ACQUISITIONCONFIG_T8,
	MXT768E_CHRGTIME_BATT, 0, 5, 1, 0, 0, 4, 30, MXT768E_ATCHFRCCALTHR_WAKEUP,
	MXT768E_ATCHFRCCALRATIO_WAKEUP
};

static u8 t9_config_e[] = { TOUCH_MULTITOUCHSCREEN_T9,
	139, 0, 0, 24, 32, 0, 176, MXT768E_THRESHOLD_BATT, 2, 1,
	10, 16, 1, 47, MXT768E_MAX_MT_FINGERS, 20, 40, 20, 31, 3,
	255, 4, 12, 12, 5, 5, 128, 0, 136, 30,
	12, 25, 0, 0, 0
};

static u8 t15_config_e[] = { TOUCH_KEYARRAY_T15,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static u8 t18_config_e[] = { SPT_COMCONFIG_T18,
	0, 0
};

static u8 t19_config_e[] = { SPT_GPIOPWM_T19,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static u8 t25_config_e[] = { SPT_SELFTEST_T25,
	0, 0, 0, 0, 0, 0,0,0, 0, 0, 0, 0, 0, 0
};

static u8 t40_config_e[] = { PROCI_GRIPSUPPRESSION_T40,
	0, 0, 0, 0, 0
};

static u8 t42_config_e[] = { PROCI_TOUCHSUPPRESSION_T42,
	3, 35, 80, 64, 224, 0, 0, 0, 0
};

static u8 t43_config_e[] = { SPT_DIGITIZER_T43,
	0, 0, 0, 0, 0, 0, 0
};

static u8 t46_config_e[] = { SPT_CTECONFIG_T46,
	0, 0, MXT768E_IDLESYNCSPERX_BATT, MXT768E_ACTVSYNCSPERX_BATT, 0, 0, 2, 0
};

static u8 t47_config_e[] = { PROCI_STYLUS_T47,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static u8 t48_config_e[] = {PROCG_NOISESUPPRESSION_T48,
	3, 128, MXT768E_CALCFG_BATT, 0, 0, 0, 0, 0, 0, 0,
	176, 37, 0, 6, 6, 0, 0, 64, 4, 64,
	0, 0, 20, 0, 0, 0, 0, 15, 0, 0,
	0, 0, 0, 0, 112, 48, 3, 16, 2, 47,
	MXT768E_MAX_MT_FINGERS, 20, 40, 250, 250, 5, 5, 143, 50, 136,
	30, 12, 25, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0
};

static u8 t48_config_chrg_e[] = {PROCG_NOISESUPPRESSION_T48,
	3, 128, MXT768E_CALCFG_CHRG, 0, 0, 0, 0, 0, 0, 0,
	112, 11, 0, 6, 6, 0, 0, 64, 4, 64,
	0, 0, 20, 0, 0, 0, 0, 32, 0, 0,
	0, 0, 0, 0, 112, 48, 3, 16, 2, 47,
	MXT768E_MAX_MT_FINGERS, 20, 40, 250, 250, 5, 5, 143, 50, 136,
	30, 12, 25, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0
};

static u8 t52_config_e[] = { TOUCH_PROXIMITY_KEY_T52,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static u8 t55_config_e[] = {ADAPTIVE_T55,
	0, 0, 0, 0, 0
};

/* T56 used from 2.0 firmware */
static u8 t56_config_e[] = {PROCI_SHIELDLESS_T56,
	1,  0,  1,  47,  14,  15,  15,  16,  15,  17,
	16,  16,  16,  16,  17,  16,  16,  16,  16,  16,
	16,  16,  15,  15,  15,  15,  15,  14,  0,  48,
	1,  1, 27, 4
};

static u8 t57_config_e[] = {SPT_GENERICDATA_T57,
	0, 0, 0
};


static u8 end_config_e[] = { RESERVED_T255 };

static const u8 *mxt768e_config[] = {
	t7_config_e,
	t8_config_e,
	t9_config_e,
	t15_config_e,
	t18_config_e,
	t19_config_e,
	t25_config_e,
	t40_config_e,
	t42_config_e,
	t43_config_e,
	t46_config_e,
	t47_config_e,
	t48_config_e,
	t52_config_e,
	t55_config_e,
	t57_config_e,
	end_config_e,
};

static struct mxt_platform_data mxt_data = {
	.max_finger_touches = MXT768E_MAX_MT_FINGERS,
	.gpio_read_done = GPIO_TSP_INT_18V,
	.config = mxt224_config,
	.config_e = mxt768e_config,
	.min_x = 0,
	.max_x = 1279,
	.min_y = 0,
	.max_y = 799,
	.min_z = 0,
	.max_z = 255,
	.min_w = 0,
	.max_w = 30,
	.chrgtime_batt = MXT768E_CHRGTIME_BATT,
	.chrgtime_charging = MXT768E_CHRGTIME_CHRG,
	.atchcalst = MXT224_ATCHCALST,
	.atchcalsthr = MXT224_ATCHCALTHR,
	.tchthr_batt = MXT224_THRESHOLD_BATT,
	.tchthr_charging = MXT224_THRESHOLD_CHRG,
	.tchthr_batt_e = MXT768E_THRESHOLD_BATT,
	.tchthr_charging_e = MXT768E_THRESHOLD_CHRG,
	.calcfg_batt_e = MXT768E_CALCFG_BATT,
	.calcfg_charging_e = MXT768E_CALCFG_CHRG,
	.atchfrccalthr_e = MXT768E_ATCHFRCCALTHR_NORMAL,
	.atchfrccalratio_e = MXT768E_ATCHFRCCALRATIO_NORMAL,
	.idlesyncsperx_batt = MXT768E_IDLESYNCSPERX_BATT,
	.idlesyncsperx_charging = MXT768E_IDLESYNCSPERX_CHRG,
	.actvsyncsperx_batt = MXT768E_ACTVSYNCSPERX_BATT,
	.actvsyncsperx_charging = MXT768E_ACTVSYNCSPERX_CHRG,
	.idleacqint_batt = MXT768E_IDLEACQINT_BATT,
	.idleacqint_charging = MXT768E_IDLEACQINT_CHRG,
	.t48_config_batt_e = t48_config_e,
	.t48_config_chrg_e = t48_config_chrg_e,
	.t56_config_e = t56_config_e,
	.power_on = ts_power_on,
	.power_off = ts_power_off,
	.register_cb = ts_register_callback,
	.read_ta_status = ts_read_ta_status,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_MXT1386)
static struct mxt_callbacks *charger_callbacks;
static void  sec_mxt1386_charger_infom(bool en)
{
	if (charger_callbacks && charger_callbacks->inform_charger)
		charger_callbacks->inform_charger(charger_callbacks, en);
	is_cable_attached = en;
	printk(KERN_DEBUG "[TSP] %s - %s\n", __func__,
		en ? "on" : "off");
}

static void p3_register_touch_callbacks(struct mxt_callbacks *cb)
{
	charger_callbacks = cb;
}

static void mxt1386_power_on(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, 1);
	s3c_gpio_cfgpin(GPIO_TSP_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_RST, 1);
	mdelay(70);
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_SFN(0xf));
	mdelay(40);
	printk(KERN_ERR"[TSP]mxt1386_power_on is finished\n");
}

static void mxt1386_power_off(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_TSP_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_RST, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_RST, 0);

	s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_LDO_ON, 0);
	/* printk("mxt224_power_off is finished\n"); */
}

static struct mxt_platform_data p4w_touch_platform_data = {
	.numtouch = 10,
	.max_x  = 1280,
	.max_y  = 800,
	.init_platform_hw  = mxt1386_power_on,
	.exit_platform_hw  = mxt1386_power_off,
	.suspend_platform_hw = mxt1386_power_off,
	.resume_platform_hw = mxt1386_power_on,
	.register_cb = p3_register_touch_callbacks,
	/*mxt_power_config*/
	/* Set Idle Acquisition Interval to 32 ms. */
	.power_config.idleacqint = 32,
	.power_config.actvacqint = 255,
	/* Set Active to Idle Timeout to 4 s (one unit = 200ms). */
	.power_config.actv2idleto = 50,
	/*acquisition_config*/
	/* Atmel: 8 -> 10*/
	.acquisition_config.chrgtime = 10,
	.acquisition_config.reserved = 0,
	.acquisition_config.tchdrift = 5,
	/* Atmel: 0 -> 10*/
	.acquisition_config.driftst = 10,
	/* infinite*/
	.acquisition_config.tchautocal = 0,
	/* disabled*/
	.acquisition_config.sync = 0,
#ifdef MXT_CALIBRATE_WORKAROUND
	/*autocal config at wakeup status*/
	.acquisition_config.atchcalst = 9,
	.acquisition_config.atchcalsthr = 48,
	/* Atmel: 50 => 10 : avoid wakeup lockup : 2 or 3 finger*/
	.acquisition_config.atchcalfrcthr = 10,
	.acquisition_config.atchcalfrcratio = 215,
#else
	/* Atmel: 5 -> 0 -> 9  (to avoid ghost touch problem)*/
	.acquisition_config.atchcalst = 9,
	/* Atmel: 50 -> 55 -> 48 ->10 (to avoid ghost touch problem)*/
	.acquisition_config.atchcalsthr = 10,
	/* 50-> 20 (To avoid  wakeup touch lockup)  */
	.acquisition_config.atchcalfrcthr = 20,
	/* 25-> 0  (To avoid  wakeup touch lockup */
	.acquisition_config.atchcalfrcratio = 0,
#endif
	/*multitouch_config*/
	/* enable + message-enable*/
	.touchscreen_config.ctrl = 0x8b,
	.touchscreen_config.xorigin = 0,
	.touchscreen_config.yorigin = 0,
	.touchscreen_config.xsize = 27,
	.touchscreen_config.ysize = 42,
	.touchscreen_config.akscfg = 0,
	/* Atmel: 0x11 -> 0x21 -> 0x11*/
	.touchscreen_config.blen = 0x11,
	/* Atmel: 50 -> 55 -> 48,*/
	.touchscreen_config.tchthr = 48,
	.touchscreen_config.tchdi = 2,
	/* orient : Horizontal flip */
	.touchscreen_config.orient = 1,
	.touchscreen_config.mrgtimeout = 0,
	.touchscreen_config.movhysti = 10,
	.touchscreen_config.movhystn = 1,
	 /* Atmel  0x20 ->0x21 -> 0x2e(-2)*/
	.touchscreen_config.movfilter = 0x2f,
	.touchscreen_config.numtouch = MXT_MAX_NUM_TOUCHES,
	.touchscreen_config.mrghyst = 5, /*Atmel 10 -> 5*/
	 /* Atmel 20 -> 5 -> 50 (To avoid One finger Pinch Zoom) */
	.touchscreen_config.mrgthr = 50,
	.touchscreen_config.amphyst = 10,
	.touchscreen_config.xrange = 799,
	.touchscreen_config.yrange = 1279,
	.touchscreen_config.xloclip = 0,
	.touchscreen_config.xhiclip = 0,
	.touchscreen_config.yloclip = 0,
	.touchscreen_config.yhiclip = 0,
	.touchscreen_config.xedgectrl = 0,
	.touchscreen_config.xedgedist = 0,
	.touchscreen_config.yedgectrl = 0,
	.touchscreen_config.yedgedist = 0,
	.touchscreen_config.jumplimit = 18,
	.touchscreen_config.tchhyst = 10,
	.touchscreen_config.xpitch = 1,
	.touchscreen_config.ypitch = 3,
	/*noise_suppression_config*/
	.noise_suppression_config.ctrl = 0x87,
	.noise_suppression_config.reserved = 0,
	.noise_suppression_config.reserved1 = 0,
	.noise_suppression_config.reserved2 = 0,
	.noise_suppression_config.reserved3 = 0,
	.noise_suppression_config.reserved4 = 0,
	.noise_suppression_config.reserved5 = 0,
	.noise_suppression_config.reserved6 = 0,
	.noise_suppression_config.noisethr = 40,
	.noise_suppression_config.reserved7 = 0,/*1;*/
	.noise_suppression_config.freq[0] = 10,
	.noise_suppression_config.freq[1] = 18,
	.noise_suppression_config.freq[2] = 23,
	.noise_suppression_config.freq[3] = 30,
	.noise_suppression_config.freq[4] = 36,
	.noise_suppression_config.reserved8 = 0, /* 3 -> 0*/
	/*cte_config*/
	.cte_config.ctrl = 0,
	.cte_config.cmd = 0,
	.cte_config.mode = 0,
	/*16 -> 4 -> 8*/
	.cte_config.idlegcafdepth = 8,
	/*63 -> 16 -> 54(16ms sampling)*/
	.cte_config.actvgcafdepth = 54,
	.cte_config.voltage = 0x3c,
	/* (enable + non-locking mode)*/
	.gripsupression_config.ctrl = 0,
	.gripsupression_config.xlogrip = 0, /*10 -> 0*/
	.gripsupression_config.xhigrip = 0, /*10 -> 0*/
	.gripsupression_config.ylogrip = 0, /*10 -> 15*/
	.gripsupression_config.yhigrip = 0,/*10 -> 15*/
	.palmsupression_config.ctrl = 1,
	.palmsupression_config.reserved1 = 0,
	.palmsupression_config.reserved2 = 0,
	/* 40 -> 20(For PalmSuppression detect) */
	.palmsupression_config.largeobjthr = 20,
	/* 5 -> 50(For PalmSuppression detect) */
	.palmsupression_config.distancethr = 50,
	.palmsupression_config.supextto = 5,
	/*config change for ta connected*/
	.tchthr_for_ta_connect = 80,
	.noisethr_for_ta_connect = 55,
	.idlegcafdepth_ta_connect = 32,
	.freq_for_ta_connect[0] = 45,
	.freq_for_ta_connect[1] = 49,
	.freq_for_ta_connect[2] = 55,
	.freq_for_ta_connect[3] = 59,
	.freq_for_ta_connect[4] = 63,
	.fherr_cnt = 0,
	.tch_blen_for_fherr = 0,
	.tchthr_for_fherr = 35,
	.noisethr_for_fherr = 30,
	.freq_for_fherr1[0] = 45,
	.freq_for_fherr1[1] = 49,
	.freq_for_fherr1[2] = 55,
	.freq_for_fherr1[3] = 59,
	.freq_for_fherr1[4] = 63,
	.freq_for_fherr2[0] = 10,
	.freq_for_fherr2[1] = 12,
	.freq_for_fherr2[2] = 18,
	.freq_for_fherr2[3] = 40,
	.freq_for_fherr2[4] = 72,
	.freq_for_fherr3[0] = 7,
	.freq_for_fherr3[1] = 33,
	.freq_for_fherr3[2] = 39,
	.freq_for_fherr3[3] = 52,
	.freq_for_fherr3[4] = 64,
#ifdef MXT_CALIBRATE_WORKAROUND
	/*autocal config at idle status*/
	.atchcalst_idle = 9,
	.atchcalsthr_idle = 10,
	.atchcalfrcthr_idle = 50,
	/* Atmel: 25 => 55 : avoid idle palm on lockup*/
	.atchcalfrcratio_idle = 55,
#endif
};
#endif

#ifdef CONFIG_TOUCHSCREEN_MELFAS
static struct tsp_callbacks *charger_cbs;
static void sec_charger_melfas_cb(bool en)
{
	if (charger_cbs && charger_cbs->inform_charger)
		charger_cbs->inform_charger(charger_cbs, en);
	is_cable_attached = en;
	printk(KERN_DEBUG "[TSP] %s - %s\n", __func__,
		en ? "on" : "off");
}
static void register_tsp_callbacks(struct tsp_callbacks *cb)
{
	charger_cbs = cb;
}

static void ts_power_on(void)
{
//	s3c_gpio_cfgpin(GPIO_TSP_RST, S3C_GPIO_OUTPUT);
//	s3c_gpio_setpull(GPIO_TSP_RST, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_RST, GPIO_LEVEL_HIGH);
	mdelay(70);
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_SFN(0xf));
	pr_info("[TSP] TSP POWER ON\n");
}

static void ts_power_off(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);

//	s3c_gpio_cfgpin(GPIO_TSP_RST, S3C_GPIO_OUTPUT);
//	s3c_gpio_setpull(GPIO_TSP_RST, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TSP_RST, GPIO_LEVEL_LOW);
	pr_info("[TSP] TSP POWER OFF");
}


static void ts_read_ta_status(bool *ta_status)
{
	*ta_status = is_cable_attached;
}

static void ts_set_touch_i2c(void)
{
	s3c_gpio_cfgpin(GPIO_TSP_SDA, S3C_GPIO_SFN(3));
	s3c_gpio_setpull(GPIO_TSP_SDA, S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(GPIO_TSP_SCL, S3C_GPIO_SFN(3));
	s3c_gpio_setpull(GPIO_TSP_SCL, S3C_GPIO_PULL_UP);
	gpio_free(GPIO_TSP_SDA);
	gpio_free(GPIO_TSP_SCL);
}

static void ts_set_touch_i2c_to_gpio(void)
{

	s3c_gpio_cfgpin(GPIO_TSP_SDA, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_SDA, S3C_GPIO_PULL_UP);
	s3c_gpio_cfgpin(GPIO_TSP_SCL, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_SCL, S3C_GPIO_PULL_UP);
	gpio_request(GPIO_TSP_SDA, "GPIO_TSP_SDA");
	gpio_request(GPIO_TSP_SCL, "GPIO_TSP_SCL");

}

static struct ts_platform_data ts_data = {
	.gpio_read_done = GPIO_TSP_INT,
	.gpio_int = GPIO_TSP_INT,
	.power_on = ts_power_on,
	.power_off = ts_power_off,
	.register_cb = register_tsp_callbacks,
	.read_ta_status = ts_read_ta_status,
	.set_touch_i2c = ts_set_touch_i2c,
	.set_touch_i2c_to_gpio = ts_set_touch_i2c_to_gpio,
};

#endif //ifdef CONFIG_TOUCHSCREEN_MELFAS

#endif

#ifdef CONFIG_I2C_S3C2410
/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
	{ I2C_BOARD_INFO("24c128", 0x50), },	 /* Samsung S524AD0XD1 */
	{ I2C_BOARD_INFO("24c128", 0x52), },	 /* Samsung S524AD0XD1 */
};
#ifdef CONFIG_S3C_DEV_I2C1

#ifndef CONFIG_MPU_SENSORS_MPU3050

/* I2C1 */
static struct k3dh_platform_data k3dh_data = {
	.gpio_acc_int = GPIO_ACC_INT,
};

static struct i2c_board_info i2c_devs1[] __initdata = {
	{
		I2C_BOARD_INFO("k3g", 0x69),
		.irq = IRQ_EINT(1),
	},
	{
		I2C_BOARD_INFO("k3dh", 0x19),
		.platform_data	= &k3dh_data,
	},
};

#endif //!CONFIG_MPU_SENSORS_MPU3050

#endif //CONFIG_S3C_DEV_I2C1

#ifdef CONFIG_S3C_DEV_I2C2
/* I2C2 */
static struct i2c_board_info i2c_devs2[] __initdata = {
};
#endif

#ifdef CONFIG_S3C_DEV_I2C3
/* I2C3 */
static struct i2c_board_info i2c_devs3[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_MXT1386) || defined(CONFIG_TOUCHSCREEN_MXT768E)
	{
#ifdef CONFIG_TOUCHSCREEN_MXT1386
		I2C_BOARD_INFO("sec_touchscreen", 0x4c),
		.platform_data = &p4w_touch_platform_data,
#else
		I2C_BOARD_INFO(MXT_DEV_NAME, 0x4c),
		.platform_data = &mxt_data
#endif
	},
#elif defined (CONFIG_TOUCHSCREEN_MELFAS)
    {
	    I2C_BOARD_INFO(TS_DEV_NAME, TS_DEV_ADDR),
	    .platform_data	= &ts_data,

    },
#endif
};
#endif

#ifdef CONFIG_EPEN_WACOM_G5SP
static int p4w_wacom_init_hw(void);
static int p4w_wacom_exit_hw(void);
static int p4w_wacom_suspend_hw(void);
static int p4w_wacom_resume_hw(void);
static int p4w_wacom_early_suspend_hw(void);
static int p4w_wacom_late_resume_hw(void);
static int p4w_wacom_reset_hw(void);
static void p4w_wacom_register_callbacks(struct wacom_g5_callbacks *cb);

static struct wacom_g5_platform_data p4w_wacom_platform_data = {
	.x_invert = 0,
	.y_invert = 0,
	.xy_switch = 0,
	.init_platform_hw = p4w_wacom_init_hw,
/*	.exit_platform_hw =,	*/
	.suspend_platform_hw = p4w_wacom_suspend_hw,
	.resume_platform_hw = p4w_wacom_resume_hw,
	.early_suspend_platform_hw = p4w_wacom_early_suspend_hw,
	.late_resume_platform_hw = p4w_wacom_late_resume_hw,
	.reset_platform_hw = p4w_wacom_reset_hw,
	.register_cb = p4w_wacom_register_callbacks,
};
#endif /* CONFIG_EPEN_WACOM_G5SP */

#ifdef CONFIG_S3C_DEV_I2C4
/* I2C4 */
static struct i2c_board_info i2c_devs4[] __initdata = {
#ifdef CONFIG_EPEN_WACOM_G5SP
	{
		I2C_BOARD_INFO("wacom_g5sp_i2c", 0x56),
		.platform_data = &p4w_wacom_platform_data,
	},
#endif /* CONFIG_EPEN_WACOM_G5SP */
};
#endif

#ifdef CONFIG_EPEN_WACOM_G5SP
static void p4w_wacom_register_callbacks(struct wacom_g5_callbacks *cb)
{
	wacom_callbacks = cb;
};

static int __init p4w_wacom_init(void)
{
	p4w_wacom_init_hw();
	gpio_set_value(GPIO_PEN_LDO_EN, 1);
	printk(KERN_INFO "[E-PEN]: %s.\n", __func__);
	return 0;
}

static int p4w_wacom_init_hw(void)
{
	int ret;
	ret = gpio_request(GPIO_PEN_LDO_EN, "PEN_LDO_EN");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(GPIO_PEN_LDO_EN)\n");
		return ret;
	}
	s3c_gpio_cfgpin(GPIO_PEN_LDO_EN, S3C_GPIO_SFN(0x1));
	s3c_gpio_setpull(GPIO_PEN_LDO_EN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_PEN_LDO_EN, 0);

	ret = gpio_request(GPIO_PEN_PDCT_18V, "PEN_PDCT");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(GPIO_PEN_PDCT_18V)\n");
		return ret;
	}
	s3c_gpio_cfgpin(GPIO_PEN_PDCT_18V, S3C_GPIO_SFN(0x0));
	s3c_gpio_setpull(GPIO_PEN_PDCT_18V, S3C_GPIO_PULL_NONE);
	gpio_direction_input(GPIO_PEN_PDCT_18V);

	ret = gpio_request(GPIO_PEN_SLP_18V, "PEN_SLP");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(GPIO_PEN_SLP_18V)\n");
		return ret;
	}
	s3c_gpio_cfgpin(GPIO_PEN_SLP_18V, S3C_GPIO_SFN(0x1));
	s3c_gpio_setpull(GPIO_PEN_SLP_18V, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_PEN_SLP_18V, 0);

	ret = gpio_request(GPIO_PEN_IRQ_18V, "PEN_IRQ");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(GPIO_PEN_IRQ_18V)\n");
		return ret;
	}
	s3c_gpio_cfgpin(GPIO_PEN_IRQ_18V, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_PEN_IRQ_18V, S3C_GPIO_PULL_DOWN);
	gpio_direction_input(GPIO_PEN_IRQ_18V);
	i2c_devs4[0].irq = gpio_to_irq(GPIO_PEN_IRQ_18V);
	set_irq_type(i2c_devs4[0].irq, IRQ_TYPE_LEVEL_HIGH);

	return 0;
}


static int p4w_wacom_suspend_hw(void)
{
	return(p4w_wacom_early_suspend_hw());
}

static int p4w_wacom_resume_hw(void)
{
	return(p4w_wacom_late_resume_hw());
}

static int p4w_wacom_early_suspend_hw(void)
{
#if defined(WACOM_SLEEP_WITH_PEN_SLP)
	gpio_set_value(GPIO_PEN_SLP_18V, 1);
#elif defined(WACOM_SLEEP_WITH_PEN_LDO_EN)
	gpio_set_value(GPIO_PEN_LDO_EN, 0);
#endif
	return 0;
}

static int p4w_wacom_late_resume_hw(void)
{
#if defined(WACOM_SLEEP_WITH_PEN_SLP)
	gpio_set_value(GPIO_PEN_SLP_18V, 0);
#elif defined(WACOM_SLEEP_WITH_PEN_LDO_EN)
	gpio_set_value(GPIO_PEN_LDO_EN, 1);
#endif

#if (WACOM_HAVE_RESET_CONTROL == 1)
	msleep(WACOM_DELAY_FOR_RST_RISING);
	gpio_set_value(GPIO_PEN_SLP_18V, 1);
#endif
	return 0;
}
static int p4w_wacom_reset_hw(void)
{

#if (WACOM_HAVE_RESET_CONTROL == 1)
	gpio_set_value(OMAP_GPIO_PEN_RST, 0);
	msleep(200);
	gpio_set_value(OMAP_GPIO_PEN_RST, 1);
#endif
	printk(KERN_INFO "[E-PEN] : wacom warm reset(%d).\n", WACOM_HAVE_RESET_CONTROL);
	return 0;
}

#endif /* CONFIG_EPEN_WACOM_G5SP */

#ifdef CONFIG_S3C_DEV_I2C5
/* I2C5 */
static struct i2c_board_info i2c_devs5[] __initdata = {
#ifdef CONFIG_MFD_MAX8998
	{
		I2C_BOARD_INFO("lp3974", 0x66),
		.platform_data	= &s5pv310_max8998_info,
	},
#endif
#ifdef CONFIG_MFD_MAX8997
	{
		I2C_BOARD_INFO("max8997", (0xcc >> 1)),
		.platform_data	= &s5pv310_max8997_info,
	},
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_I2C6
/* I2C6 */
static struct i2c_board_info i2c_devs6[] __initdata = {
#ifdef CONFIG_SND_SOC_MC1N2
	{
		I2C_BOARD_INFO("mc1n2", 0x3a),		/* MC1N2 */
		.platform_data = &mc1n2_pdata,
	},
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_I2C7

static struct akm8975_platform_data akm8975_pdata = {
	.gpio_data_ready_int = GPIO_MSENSE_INT,
};
/* I2C7 */
static struct i2c_board_info i2c_ak8975_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("ak8975", 0x0C),
		.irq = IRQ_EINT(18),
		.platform_data = &akm8975_pdata,
	},
#ifdef CONFIG_VIDEO_TVOUT
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
#endif
};
#endif

#ifdef CONFIG_S3C_DEV_I2C8_EMUL
static struct i2c_gpio_platform_data gpio_i2c_data8 = {
	.sda_pin = GPIO_3_TOUCH_SDA,
	.scl_pin = GPIO_3_TOUCH_SCL,
};

struct platform_device s3c_device_i2c8 = {
	.name = "i2c-gpio",
	.id = 8,
	.dev.platform_data = &gpio_i2c_data8,
};

/* I2C8 */
static struct i2c_board_info i2c_devs8_emul[] __initdata = {
	{
		I2C_BOARD_INFO("melfas_touchkey", 0x20),
	},
};
#endif

#ifdef CONFIG_S3C_DEV_I2C9_EMUL
static struct i2c_gpio_platform_data gpio_i2c_data9 = {
	.sda_pin = GPIO_FUEL_SDA,
	.scl_pin = GPIO_FUEL_SCL,
};

struct platform_device s3c_device_i2c9 = {
	.name = "i2c-gpio",
	.id = 9,
	.dev.platform_data = &gpio_i2c_data9,
};

/* I2C9 */
#ifdef CONFIG_BATTERY_MAX17042
static struct max17042_platform_data max17042_pdata = {
#if defined(CONFIG_MACH_P2_REV00) || defined(CONFIG_MACH_P2_REV01) || defined(CONFIG_MACH_P2_REV02)
	.sdi_capacity = 0x1EC8,
	.sdi_vfcapacity = 0x290A,
	.atl_capacity = 0x1FBE,
	.atl_vfcapacity = 0x2A54,
	.sdi_low_bat_comp_start_vol = 3550,
	.atl_low_bat_comp_start_vol = 3450,
	.fuel_alert_line = GPIO_FUEL_ALERT,
#elif defined(CONFIG_MACH_P4W_REV00) || defined(CONFIG_MACH_P4W_REV01)	 || defined(CONFIG_MACH_P4W_REV02)	/* P4W battery parameter */
	.sdi_capacity = 0x3730,
	.sdi_vfcapacity = 0x4996,
	.atl_capacity = 0x3022,
	.atl_vfcapacity = 0x4024,
	.sdi_low_bat_comp_start_vol = 3600,
	.atl_low_bat_comp_start_vol = 3450,
	.fuel_alert_line = GPIO_FUEL_ALERT,
#elif defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01) || defined(CONFIG_MACH_P8LTE_REV00)		/* P8 battery parameter */
	.sdi_capacity = 0x2B06,
	.sdi_vfcapacity = 0x395E,
	.atl_capacity = 0x2B06,
	.atl_vfcapacity = 0x395E,
	.sdi_low_bat_comp_start_vol = 3600,
	.atl_low_bat_comp_start_vol = 3450,
	.fuel_alert_line = GPIO_FUEL_ALERT,
#else	/* default value */
	.sdi_capacity = 0x1F40,
	.sdi_vfcapacity = 0x29AC,
	.atl_capacity = 0x1FBE,
	.atl_vfcapacity = 0x2A54,
	.sdi_low_bat_comp_start_vol = 3600,
	.atl_low_bat_comp_start_vol = 3450,
	.fuel_alert_line = GPIO_FUEL_ALERT,
#endif
	.check_jig_status = check_jig_on
};

static struct i2c_board_info i2c_devs9_emul[] __initdata = {
	{
		I2C_BOARD_INFO("fuelgauge", 0x36),
		.platform_data = &max17042_pdata,
	},
};
#else
static struct i2c_board_info i2c_devs9_emul[] __initdata = {
	{
		I2C_BOARD_INFO("max17040", 0x36),
	},
};
#endif
#endif

#ifdef CONFIG_S3C_DEV_I2C10_EMUL
static struct i2c_gpio_platform_data gpio_i2c_data10 = {
	.sda_pin = GPIO_USB_SDA,
	.scl_pin = GPIO_USB_SCL,
};

struct platform_device s3c_device_i2c10 = {
	.name = "i2c-gpio",
	.id = 10,
	.dev.platform_data = &gpio_i2c_data10,
};

/* I2C10 */
static struct fsa9480_platform_data fsa9480_info = {
};

static struct i2c_board_info i2c_devs10_emul[] __initdata = {
	{
		I2C_BOARD_INFO("fsa9480", 0x25),
		.platform_data	= &fsa9480_info,
	},
};
#endif
#endif

#ifdef CONFIG_S3C_DEV_I2C11_EMUL

/* I2C11 */
static struct i2c_gpio_platform_data gpio_i2c_data11 = {
	.sda_pin = GPIO_PS_ALS_SDA,
	.scl_pin = GPIO_PS_ALS_SCL,
};

struct platform_device s3c_device_i2c11 = {
	.name = "i2c-gpio",
	.id = 11,
	.dev.platform_data = &gpio_i2c_data11,
};

#ifdef CONFIG_SENSORS_BH1721FVC
static int light_sensor_init(void)
{
	int err;
	int gpio_vout = GPIO_PS_VOUT;

	#if defined(CONFIG_OPTICAL_WAKE_ENABLE)
	if(system_rev >= 0x03) {
		printk(KERN_INFO" BH1721 Reset GPIO = GPX0(1) (rev%02d)\n", system_rev);
		gpio_vout = GPIO_PS_VOUT_WAKE;
	} else
		printk(KERN_INFO" BH1721 Reset GPIO = GPL0(6) (rev%02d)\n", system_rev);
	#endif

	printk(KERN_INFO"============================\n");
	printk(KERN_INFO"==    BH1721 Light Sensor Init         ==\n");
	printk(KERN_INFO"============================\n");
	printk("%d %d\n", GPIO_PS_ALS_SDA, GPIO_PS_ALS_SCL);
	err = gpio_request(gpio_vout, "LIGHT_SENSOR_RESET");
	if (err) {
		printk(KERN_INFO" bh1721fvc Failed to request the light "
			" sensor gpio (%d)\n", err);
		return err;
	}

	s3c_gpio_cfgpin(gpio_vout, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(gpio_vout, S3C_GPIO_PULL_NONE);

	err = gpio_direction_output(gpio_vout, 0);
	udelay(2);
	err = gpio_direction_output(gpio_vout, 1);
	if (err) {
		printk(KERN_INFO" bh1721fvc Failed to make the light sensor gpio(reset)"
			" high (%d)\n", err);
		return err;
	}

	return 0;
}

static int  bh1721fvc_light_sensor_reset(void)
{
	int err;
	int gpio_vout = GPIO_PS_VOUT;

	#if defined(CONFIG_OPTICAL_WAKE_ENABLE)
	if(system_rev >= 0x03)
		gpio_vout = GPIO_PS_VOUT_WAKE;
	#endif

	printk(KERN_INFO" bh1721fvc_light_sensor_reset\n");
	err = gpio_direction_output(gpio_vout, 0);
	if (err) {
		printk(KERN_INFO" bh1721fvc Failed to make the light sensor gpio(reset)"
			" low (%d)\n", err);
		return err;
	}

	udelay(2);

	err = gpio_direction_output(gpio_vout, 1);
	if (err) {
		printk(KERN_INFO" bh1721fvc Failed to make the light sensor gpio(reset)"
			" high (%d)\n", err);
		return err;
	}
	return 0;
}

static int  bh1721fvc_light_sensor_output(int value)
{
	int err;
	int gpio_vout = GPIO_PS_VOUT;

	#if defined(CONFIG_OPTICAL_WAKE_ENABLE)
	if(system_rev >= 0x03)
		gpio_vout = GPIO_PS_VOUT_WAKE;
	#endif

	err = gpio_direction_output(gpio_vout, value);
	if (err) {
		printk(KERN_INFO" bh1721fvc Failed to make the light sensor gpio(dvi)"
			" low (%d)\n", err);
		return err;
	}
	return 0;
}

static struct bh1721fvc_platform_data bh1721fvc_pdata = {
	.reset = bh1721fvc_light_sensor_reset,
	//.output = bh1721fvc_light_sensor_output,
};

static struct i2c_board_info i2c_bh1721_emul[] __initdata = {
	{
		I2C_BOARD_INFO("bh1721fvc", 0x23),
		.platform_data = &bh1721fvc_pdata,
	},
};
#endif

#ifdef CONFIG_OPTICAL_GP2A
static int gp2a_power(bool on)
{
	printk("%s : %d\n", __func__, on);
	return 0;
}


#if defined(CONFIG_OPTICAL_WAKE_ENABLE)

static struct gp2a_platform_data gp2a_wake_pdata = {
	.power = gp2a_power,
	.p_out = GPIO_PS_VOUT_WAKE,
};

static struct i2c_board_info i2c_wake_devs11[] __initdata = {
	{
		I2C_BOARD_INFO("gp2a", (0x88 >> 1)),
		.platform_data = &gp2a_wake_pdata,
	},
};
#endif

static struct gp2a_platform_data gp2a_pdata = {
	.power = gp2a_power,
	.p_out = GPIO_PS_VOUT,
};

static struct i2c_board_info i2c_devs11[] __initdata = {
	{
		I2C_BOARD_INFO("gp2a", (0x88 >> 1)),
		.platform_data = &gp2a_pdata,
	},
};

#endif

#endif

#if defined(CONFIG_MHL_SII9234)
static void sii9234_init(void)
{
	int ret = gpio_request(GPIO_HDMI_EN1, "hdmi_en1");
	if (ret) {
		pr_err("%s: gpio_request() for HDMI_EN1 failed\n", __func__);
		return;
	}
	gpio_direction_output(GPIO_HDMI_EN1, 0);
	if (ret) {
		pr_err("%s: gpio_direction_output() for HDMI_EN1 failed\n",
			__func__);
		return;
	}

	ret = gpio_request(GPIO_MHL_RST, "mhl_rst");
	if (ret) {
		pr_err("%s: gpio_request() for MHL_RST failed\n", __func__);
		return;
	}
	ret = gpio_direction_output(GPIO_MHL_RST, 0);
	if (ret) {
		pr_err("%s: gpio_direction_output() for MHL_RST failed\n",
			__func__);
		return;
	}
}

static void sii9234_hw_reset(void)
{
#if defined(CONFIG_HPD_PULL)
	struct regulator *reg;
	reg = regulator_get(NULL, "hdp_2.8v");
	if (IS_ERR_OR_NULL(reg)) {
		pr_err("%s: failed to get LDO11 regulator\n", __func__);
		return;
	}
#endif
	gpio_set_value(GPIO_MHL_RST, 0);
	gpio_set_value(GPIO_HDMI_EN1, 1);

	usleep_range(5000, 10000);
	gpio_set_value(GPIO_MHL_RST, 1);
#if defined(CONFIG_HPD_PULL)
	regulator_enable(reg);
	regulator_put(reg);
#endif
	printk(KERN_ERR "[MHL]sii9234_hw_reset.\n");
	msleep(30);
}

static void sii9234_hw_off(void)
{
#if defined(CONFIG_HPD_PULL)
	struct regulator *reg;
	reg = regulator_get(NULL, "hdp_2.8v");
	if (IS_ERR_OR_NULL(reg)) {
		pr_err("%s: failed to get LDO11 regulator\n", __func__);
		return;
	}
	regulator_disable(reg);
	regulator_put(reg);
#endif
	gpio_set_value(GPIO_HDMI_EN1, 0);
	gpio_set_value(GPIO_MHL_RST, 0);
	printk(KERN_ERR "[MHL]sii9234_hw_off.\n");
}

struct sii9234_platform_data sii9234_pdata = {
	.hw_reset = sii9234_hw_reset,
	.hw_off = sii9234_hw_off
};
static struct i2c_board_info i2c_devs15[] __initdata = {
	{
		I2C_BOARD_INFO("SII9234", 0x72>>1),
		.platform_data = &sii9234_pdata,
	},
	{
		I2C_BOARD_INFO("SII9234A", 0x7A>>1),
	},
	{
		I2C_BOARD_INFO("SII9234B", 0x92>>1),
	},
	{
		I2C_BOARD_INFO("SII9234C", 0xC8>>1),
	},
};
/* i2c-gpio emulation platform_data */
static struct i2c_gpio_platform_data i2c15_platdata = {
	.sda_pin		= GPIO_AP_SDA_18V,
	.scl_pin		= GPIO_AP_SCL_18V,
	.udelay			= 2,	/* 250 kHz*/
	.sda_is_open_drain	= 0,
	.scl_is_open_drain	= 0,
	.scl_is_output_only	= 0,
};

static struct platform_device s3c_device_i2c15 = {
	.name			= "i2c-gpio",
	.id			= 15,
	.dev.platform_data	= &i2c15_platdata,
};

#endif

#ifdef CONFIG_USBHUB_USB3503
int usb3503_hw_config(void)
{
	s3c_gpio_cfgpin(GPIO_USB_HUB_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_USB_HUB_RST, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_USB_HUB_RST, S3C_GPIO_SETPIN_ZERO);
	s5p_gpio_set_drvstr(GPIO_USB_HUB_RST, S5P_GPIO_DRVSTR_LV1); /* need to check drvstr 1 or 2 */

	return 0;
}

int usb3503_reset_n(int val)
{
	gpio_set_value(GPIO_USB_HUB_RST, !!val);

	pr_info("Board : %s = %d\n", __func__, gpio_get_value(GPIO_USB_HUB_RST));
}

/* I2C17_EMUL */
static struct i2c_gpio_platform_data i2c17_platdata = {
	.sda_pin = GPIO_USB_HUB_I2C_SDA,
	.scl_pin = GPIO_USB_HUB_I2C_SCL,
};

struct platform_device s3c_device_i2c17 = {
	.name = "i2c-gpio",
	.id = 17,
	.dev.platform_data = &i2c17_platdata,
};

struct usb3503_platform_data usb3503_pdata = {
	.init_needed    =  1,
	.es_ver         = 1,
	.inital_mode    = USB_3503_MODE_STANDBY,
	.hw_config      = usb3503_hw_config,
	.reset_n        = usb3503_reset_n,
};

static struct i2c_board_info i2c_devs17_emul[] __initdata = {
	{
		I2C_BOARD_INFO(USB3503_I2C_NAME, 0x08),
		.platform_data  = &usb3503_pdata,
	},
};
#endif /* CONFIG_USBHUB_USB3503 */

#ifdef CONFIG_30PIN_CONN
static void smdk_accessory_gpio_init(void)
{
	gpio_request(GPIO_ACCESSORY_INT, "accessory");
	s3c_gpio_cfgpin(GPIO_ACCESSORY_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_ACCESSORY_INT, S3C_GPIO_PULL_NONE);
	gpio_direction_input(GPIO_ACCESSORY_INT);

	gpio_request(GPIO_DOCK_INT, "dock");
	s3c_gpio_cfgpin(GPIO_DOCK_INT, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(GPIO_DOCK_INT, S3C_GPIO_PULL_NONE);
	gpio_direction_input(GPIO_DOCK_INT);

	gpio_request(GPIO_USB_OTG_EN, "GPIO_USB_OTG_EN");
	s3c_gpio_cfgpin(GPIO_USB_OTG_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_USB_OTG_EN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_USB_OTG_EN, false);
	gpio_free(GPIO_USB_OTG_EN);

	gpio_request(GPIO_ACCESSORY_EN, "GPIO_ACCESSORY_EN");
	s3c_gpio_cfgpin(GPIO_ACCESSORY_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_ACCESSORY_EN, S3C_GPIO_PULL_NONE);
	gpio_direction_output(GPIO_ACCESSORY_EN, false);
	gpio_free(GPIO_ACCESSORY_EN);
}

#ifdef CONFIG_USB_HOST_NOTIFY
void smdk_accessory_power(u8 token, bool active);
#define HOST_NOTIFIER_BOOSTER	smdk_accessory_power
#define HOST_NOTIFIER_GPIO		(GPIO_ACCESSORY_OUT_5V)
#include "dev-host-notifier.c"
#endif

#define RETRY_CNT_LIMIT 100

#ifdef CONFIG_MACH_P8LTE_REV00
extern void s5pv210_hsic_port1_power(int enable);

static void smdk_usb_otg_en(int active)
{
	int ret;

#ifdef CONFIG_USB_HOST_NOTIFY
	struct usb_hcd *ehci_hcd = platform_get_drvdata(&s3c_device_usb_ehci);
	int retry_cnt = 1;
	usb_switch_lock();
	if (active) {
		pm_runtime_get_sync(&s3c_device_usb_ehci.dev);
		usb_switch_set_path(USB_PATH_HOST);
		smdk_accessory_power(2, 1);

		host_notifier_pdata.ndev.mode = NOTIFY_HOST_MODE;
		if (host_notifier_pdata.usbhostd_start)
			host_notifier_pdata.usbhostd_start();
	} else {
		pm_runtime_put_sync(&s3c_device_usb_ehci.dev);
		/* waiting for ehci root hub suspend is done */
		while (ehci_hcd->state != HC_STATE_SUSPENDED) {
			msleep(50);
			if (retry_cnt++ > RETRY_CNT_LIMIT) {
				printk(KERN_ERR "ehci suspend not completed\n");
				break;
			}
		}
		usb_switch_clr_path(USB_PATH_HOST);
		if (host_notifier_pdata.usbhostd_stop)
			host_notifier_pdata.usbhostd_stop();

		smdk_accessory_power(2, 0);
	}
	usb_switch_unlock();
#endif

	if (!active) {
		s5pv210_hsic_port1_power(0);
		usb3503_set_mode(USB_3503_MODE_STANDBY);
	}

	ret = gpio_request(GPIO_USB_OTG_EN, "GPIO_USB_OTG_EN");
	if (ret) {
		printk(KERN_ERR "[ACC]: fail to request gpio(GPIO_USB_OTG_EN)\n");
		return;
	}

	gpio_direction_output(GPIO_USB_OTG_EN, active);
	gpio_free(GPIO_USB_OTG_EN);

	if (active) {
		usb3503_set_mode(USB_3503_MODE_HUB);
		s5pv210_hsic_port1_power(1);
	}

	pr_info("Board : %s = %d\n", __func__, active);
}
#else
static void smdk_usb_otg_en(int active)
{
	int ret;
	ret = gpio_request(GPIO_USB_OTG_EN, "GPIO_USB_OTG_EN");
	if (ret) {
		printk(KERN_ERR "[ACC]: fail to request gpio(GPIO_USB_OTG_EN)\n");
		return;
	}

	gpio_direction_output(GPIO_USB_OTG_EN, active);
	gpio_free(GPIO_USB_OTG_EN);

#ifdef CONFIG_USB_HOST_NOTIFY
	struct usb_hcd *ohci_hcd = platform_get_drvdata(&s3c_device_usb_ohci);
	struct usb_hcd *ehci_hcd = platform_get_drvdata(&s3c_device_usb_ehci);
	int retry_cnt = 1;
	usb_switch_lock();
	if (active) {
		pm_runtime_get_sync(&s3c_device_usb_ehci.dev);
		pm_runtime_get_sync(&s3c_device_usb_ohci.dev);
		usb_switch_set_path(USB_PATH_HOST);
		smdk_accessory_power(2,1);

		host_notifier_pdata.ndev.mode = NOTIFY_HOST_MODE;
		if (host_notifier_pdata.usbhostd_start)
			host_notifier_pdata.usbhostd_start();

	} else {
		pm_runtime_put_sync(&s3c_device_usb_ohci.dev);
		/* waiting for ohci root hub suspend is done */
		while (ohci_hcd->state != HC_STATE_SUSPENDED) {
			msleep(50);
			if (retry_cnt++ > RETRY_CNT_LIMIT) {
				printk(KERN_ERR "ohci suspend is not completed\n");
				break;
			}
		}
		pm_runtime_put_sync(&s3c_device_usb_ehci.dev);
		/* waiting for ehci root hub suspend is done */
		while (ehci_hcd->state != HC_STATE_SUSPENDED) {
			msleep(50);
			if (retry_cnt++ > RETRY_CNT_LIMIT) {
				printk(KERN_ERR "ehci suspend not completed\n");
				break;
			}
		}
		usb_switch_clr_path(USB_PATH_HOST);
		if (host_notifier_pdata.usbhostd_stop)
			host_notifier_pdata.usbhostd_stop();
		smdk_accessory_power(2, 0);
	}
	usb_switch_unlock();
#endif

	pr_info("Board : %s = %d\n", __func__, active);
}
#endif

void smdk_accessory_power(u8 token, bool active)
{
	int gpio_acc_en;
	int try_cnt = 0;
	int gpio_acc_5v = 0;
	static bool enable;
	static u8 acc_en_token;

	/*
		token info
		0 : power off,
		1 : Keyboard dock
		2 : USB
	*/
	gpio_acc_en = GPIO_ACCESSORY_EN;
#ifdef CONFIG_MACH_P4W_REV01
	if (system_rev >= 2)
		gpio_acc_5v = GPIO_ACCESSORY_OUT_5V;
#elif defined(CONFIG_MACH_P2_REV02)	/* for checking p2 3g and wifi */
	gpio_acc_5v = GPIO_ACCESSORY_OUT_5V;
#elif defined(CONFIG_MACH_P8LTE_REV00)
	if (system_rev >= 2)
		gpio_acc_5v = GPIO_ACCESSORY_OUT_5V;
#elif defined(CONFIG_MACH_P8_REV01)	/* for checking p8 3g and wifi */
	if (system_rev >= 4)
		gpio_acc_5v = GPIO_ACCESSORY_OUT_5V;
#endif

	gpio_request(gpio_acc_en, "GPIO_ACCESSORY_EN");
	s3c_gpio_cfgpin(gpio_acc_en, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(gpio_acc_en, S3C_GPIO_PULL_NONE);

	if (active) {
		if (acc_en_token) {
			pr_info("Board : Keyboard dock is connected.\n");
			gpio_direction_output(gpio_acc_en, 0);
			msleep(100);
		}

		acc_en_token |= (1 << token);
		enable = true;
		gpio_direction_output(gpio_acc_en, 1);

		if (0 != gpio_acc_5v) {
			gpio_request(gpio_acc_5v, "gpio_acc_5v");
			s3c_gpio_cfgpin(gpio_acc_5v, S3C_GPIO_INPUT);
			s3c_gpio_setpull(gpio_acc_5v, S3C_GPIO_PULL_NONE);
			msleep(20);

			/* prevent the overcurrent */
			while (!gpio_get_value(gpio_acc_5v)) {
				gpio_direction_output(gpio_acc_en, 0);
				msleep(20);
				gpio_direction_output(gpio_acc_en, 1);
				if (try_cnt > 10) {
					pr_err("[acc] failed to enable the accessory_en");
					break;
				} else
					try_cnt++;
			}
			gpio_free(gpio_acc_5v);

		} else
			pr_info("[ACC] gpio_acc_5v is not set\n");

	} else {
		if (0 == token) {
			gpio_direction_output(gpio_acc_en, 0);
			enable = false;
		} else {
			acc_en_token &= ~(1 << token);
			if (0 == acc_en_token) {
				gpio_direction_output(gpio_acc_en, 0);
				enable = false;
			}
		}
	}
	gpio_free(gpio_acc_en);
	pr_info("Board : %s (%d,%d) %s\n", __func__,
		token, active, enable ? "on" : "off");
}

static int smdk_get_acc_state(void)
{
	return gpio_get_value(GPIO_ACCESSORY_INT);
}

static int smdk_get_dock_state(void)
{
	return gpio_get_value(GPIO_DOCK_INT);
}

#ifdef CONFIG_SEC_KEYBOARD_DOCK
static struct sec_keyboard_callbacks *keyboard_callbacks;
static int check_sec_keyboard_dock(bool attached)
{
	if (keyboard_callbacks && keyboard_callbacks->check_keyboard_dock)
		return keyboard_callbacks->check_keyboard_dock(keyboard_callbacks, attached);
	return 0;
}

static void check_uart_path(bool en)
{
	int gpio_uart_sel;
#ifdef CONFIG_MACH_P8LTE_REV00
	int gpio_uart_sel2;

	gpio_uart_sel = GPIO_UART_SEL1;
	gpio_uart_sel2 = GPIO_UART_SEL2;
	if (en)
		gpio_direction_output(gpio_uart_sel2, 1);
	else
		gpio_direction_output(gpio_uart_sel2, 0);
	printk(KERN_DEBUG "[Keyboard] uart_sel2 : %d\n",
		gpio_get_value(gpio_uart_sel2));
#else
	gpio_uart_sel = GPIO_UART_SEL;
#endif

	if (en)
		gpio_direction_output(gpio_uart_sel, 1);
	else
		gpio_direction_output(gpio_uart_sel, 0);

	printk(KERN_DEBUG "[Keyboard] uart_sel : %d\n",
		gpio_get_value(gpio_uart_sel));
}

static void sec_keyboard_register_cb(struct sec_keyboard_callbacks *cb)
{
	keyboard_callbacks = cb;
}

static struct sec_keyboard_platform_data kbd_pdata = {
	.accessory_irq_gpio = GPIO_ACCESSORY_INT,
	.acc_power = smdk_accessory_power,
	.check_uart_path = check_uart_path,
	.register_cb = sec_keyboard_register_cb,
	.wakeup_key = NULL,
};

static struct platform_device sec_keyboard = {
	.name	= "sec_keyboard",
	.id	= -1,
	.dev = {
		.platform_data = &kbd_pdata,
	}
};
#endif

struct acc_con_platform_data acc_con_pdata = {
	.otg_en = smdk_usb_otg_en,
	.acc_power = smdk_accessory_power,
	.usb_ldo_en = NULL,
	.get_acc_state = smdk_get_acc_state,
	.get_dock_state = smdk_get_dock_state,
#ifdef CONFIG_SEC_KEYBOARD_DOCK
	.check_keyboard = check_sec_keyboard_dock,
#endif
	.accessory_irq_gpio = GPIO_ACCESSORY_INT,
	.dock_irq_gpio = GPIO_DOCK_INT,
#ifdef CONFIG_MHL_SII9234
	.mhl_irq_gpio = GPIO_MHL_INT,
	.hdmi_hpd_gpio = GPIO_HDMI_HPD,
#endif
};
struct platform_device sec_device_connector = {
	.name = "acc_con",
	.id = -1,
	.dev.platform_data = &acc_con_pdata,
};
#endif

#ifdef CONFIG_VIDEO_SR200PC20
static struct i2c_gpio_platform_data  i2c13_platdata = {
	.sda_pin                = VT_CAM_SDA_18V,
	.scl_pin                = VT_CAM_SCL_18V,
        .udelay                 = 2,    /* 250KHz */
        .sda_is_open_drain      = 0,
        .scl_is_open_drain      = 0,
        .scl_is_output_only     = 0,
};

/* IIC13 */
static struct platform_device s3c_device_i2c13 = {
        .name			= "i2c-gpio",
        .id			= 13,
        .dev.platform_data      = &i2c13_platdata,
};
#endif

static struct platform_device c1_regulator_consumer = {
	.name = "c1-regulator-consumer",
	.id = -1,
};

#ifdef CONFIG_S3C_DEV_HSMMC
static struct s3c_sdhci_platdata smdkc210_hsmmc0_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
#if defined(CONFIG_S5PV310_SD_CH0_8BIT)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
static struct s3c_sdhci_platdata smdkc210_hsmmc1_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
};
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata smdkc210_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_GPIO,
	.ext_cd_gpio		= S5PV310_GPX3(4),
	.ext_cd_gpio_invert	= true,
	.vmmc_name              = "vtf_2.8v",
#if defined(CONFIG_S5PV310_SD_CH2_8BIT)
	.max_width		= 4,
#endif
};
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
static struct s3c_sdhci_platdata smdkc210_hsmmc3_pdata __initdata = {
#ifdef CONFIG_MACH_P4W_REV00 /* original */
	.cd_type		= S3C_SDHCI_CD_PERMANENT,
#else /* QCA */
	.host_caps			= MMC_CAP_4_BIT_DATA,
	.cd_type			= S3C_SDHCI_CD_EXTERNAL,
	.ext_cd_init		= register_wlan_status_notify,
	.ext_pdev           = register_wlan_pdev
#endif
};
#endif
#ifdef CONFIG_S5P_DEV_MSHC
static struct s3c_mshci_platdata smdkc210_mshc_pdata __initdata = {
	.cd_type		= S3C_MSHCI_CD_PERMANENT,
#if defined(CONFIG_S5PV310_MSHC_CH0_8BIT) && \
	defined(CONFIG_S5PV310_MSHC_CH0_DDR)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA | MMC_CAP_DDR,
#elif defined(CONFIG_S5PV310_MSHC_CH0_8BIT)
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#elif defined(CONFIG_S5PV310_MSHC_CH0_DDR)
	.host_caps				= MMC_CAP_DDR,
#endif
	.int_power_gpio		= GPIO_XMMC0_CDn,
};
#endif


#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 30,
	.parent_clkname = "mout_mpll",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 250 * 1000000,
};
#endif


#if defined(CONFIG_FB_S3C_S6F1202A)
static struct s3cfb_lcd s6f1202a = {
	.width = 1024,
	.height = 600,
	.p_width = 161,
	.p_height = 98,
	.bpp = 24,

	.freq = 60,
	.timing = {
		.h_fp = 142,
		.h_bp = 210,
		.h_sw = 50,
		.v_fp = 10,
		.v_fpe = 1,
		.v_bp = 11,
		.v_bpe = 1,
		.v_sw = 10,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

static int lcd_power_on(struct lcd_device *ld, int enable)
{
	if (enable) {
		gpio_set_value(GPIO_LCD_EN, GPIO_LEVEL_HIGH);
		msleep(50);

		gpio_set_value(GPIO_LCD_LDO_EN, GPIO_LEVEL_HIGH);
		msleep(200);

		/* LVDS_N_SHDN to high*/
		gpio_set_value(GPIO_LVDS_NSHDN, GPIO_LEVEL_HIGH);
		mdelay(1);
	} else {
		/* LVDS_nSHDN low*/
		gpio_set_value(GPIO_LVDS_NSHDN, GPIO_LEVEL_LOW);
		msleep(50);

		gpio_set_value(GPIO_LCD_LDO_EN, GPIO_LEVEL_LOW);
		msleep(10);

		/* Disable LVDS Panel Power, 1.2, 1.8, display 3.3V */
		gpio_set_value(GPIO_LCD_EN, GPIO_LEVEL_LOW);
		msleep(300);
	}

	return 0;
}

static struct lcd_platform_data p2_lcd_platform_data = {
	.power_on		= lcd_power_on,
};

#endif

#if defined(CONFIG_FB_S3C_S6C1372)
static struct s3cfb_lcd s6c1372 = {
	.width = 1280,
	.height = 800,
	.p_width = 217,
	.p_height = 135,
	.bpp = 24,

	.freq = 60,
	.timing = {
		.h_fp = 48,
		.h_bp = 90,
		.h_sw = 16,
		.v_fp = 4,
		.v_fpe = 1,
		.v_bp = 12,
		.v_bpe = 1,
		.v_sw = 3,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};
static int lcd_power_on(struct lcd_device *ld, int enable)
{
	if (enable) {
		gpio_set_value(GPIO_LCD_EN, GPIO_LEVEL_HIGH);
		msleep(40);

		/* LVDS_N_SHDN to high*/
		gpio_set_value(GPIO_LVDS_NSHDN, GPIO_LEVEL_HIGH);
		msleep(300);

		gpio_set_value(GPIO_LED_BACKLIGHT_RESET, GPIO_LEVEL_HIGH);
		msleep(2);

	} else {
		gpio_set_value(GPIO_LED_BACKLIGHT_RESET, GPIO_LEVEL_LOW);
		msleep(200);

		/* LVDS_nSHDN low*/
		gpio_set_value(GPIO_LVDS_NSHDN, GPIO_LEVEL_LOW);
		msleep(40);

		/* Disable LVDS Panel Power, 1.2, 1.8, display 3.3V */
		gpio_set_value(GPIO_LCD_EN, GPIO_LEVEL_LOW);
		msleep(400);
	}

	return 0;
}

static struct lcd_platform_data p4_lcd_platform_data = {
	.power_on		= lcd_power_on,
};
#endif

#ifdef CONFIG_FB_S3C_S6E8AB0
/* for Geminus based on MIPI-DSI interface */
static struct s3cfb_lcd s6e8ab0 = {
       .name = "s6e8ab0",
       .width = 1280,
       .height = 800,
       .p_width = 165,
       .p_height = 103,
       .bpp = 24,

       .freq = 60,

       /* minumun value is 0 except for wr_act time. */
       .cpu_timing = {
               .cs_setup = 0,
               .wr_setup = 0,
               .wr_act = 1,
               .wr_hold = 0,
       },

       .timing = {
               .h_fp = 128,
               .h_bp = 128,
               .h_sw = 94,
               .v_fp = 13,
               .v_fpe = 1,
               .v_bp = 3,
               .v_bpe = 1,
               .v_sw = 2,
	       .cmd_allow_len = 11,    /*v_fp=stable_vfp + cmd_allow_len */
	       .stable_vfp = 2,
       },

       .polarity = {
               .rise_vclk = 1,
               .inv_hsync = 0,
               .inv_vsync = 0,
               .inv_vden = 0,
       },
};

static void dsim_power(int enable)
{
	struct regulator *regulator;

	if (enable) {
		regulator = regulator_get(NULL, "vmipi_1.1v");
		if (IS_ERR(regulator))
			return;
		regulator_enable(regulator);
		regulator_put(regulator);

		regulator = regulator_get(NULL, "vmipi_1.8v");
		if (IS_ERR(regulator))
			return;
		regulator_enable(regulator);
		regulator_put(regulator);

	} else {
		regulator = regulator_get(NULL, "vmipi_1.1v");
		if (IS_ERR(regulator))
			return;
		if (regulator_is_enabled(regulator)) {
			regulator_force_disable(regulator);
		}
		regulator_put(regulator);

		regulator = regulator_get(NULL, "vmipi_1.8v");
		if (IS_ERR(regulator))
			return;
		if (regulator_is_enabled(regulator)) {
			regulator_force_disable(regulator);
		}
		regulator_put(regulator);
	}

	return;
}

static int reset_lcd(void)
{
	int err;

	printk(KERN_INFO "%s\n", __func__);

	err = gpio_request(GPIO_LCD_RST, "MLCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request GPF0[1] for "
				"MLCD_RST control\n");
		return -EPERM;
	}
	gpio_direction_output(GPIO_LCD_RST, 0);

	/* Power Reset */
	gpio_set_value(GPIO_LCD_RST, GPIO_LEVEL_HIGH);
	msleep(5);
	gpio_set_value(GPIO_LCD_RST, GPIO_LEVEL_LOW);
	msleep(5);
	gpio_set_value(GPIO_LCD_RST, GPIO_LEVEL_HIGH);


	/* Release GPIO */
	gpio_free(GPIO_LCD_RST);

	return 0;
}

static void lcd_cfg_gpio(void)
{
	/* MLCD_RST */
	s3c_gpio_cfgpin(GPIO_LCD_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_RST, S3C_GPIO_PULL_NONE);

	/* MLCD_ON */
	s3c_gpio_cfgpin(GPIO_LCD_LDO_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_LDO_EN, S3C_GPIO_PULL_NONE);

	/* LCD_EN */
	s3c_gpio_cfgpin(GPIO_LCD_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_LCD_EN, S3C_GPIO_PULL_NONE);

	return;
}

static int lcd_power_on(void *pdev, int enable)
{
	int err;

	printk(KERN_INFO "%s : enable=%d\n", __func__, enable);

	/* Request GPIO */
	err = gpio_request(GPIO_LCD_LDO_EN, "MLCD_ON");
	if (err) {
		printk(KERN_ERR "failed to request GPK1[1] for "
				"MLCD_ON control\n");
		return -EPERM;
	}

	err = gpio_request(GPIO_LCD_EN, "LCD_EN");
	if (err) {
		printk(KERN_ERR "failed to request GPL0[7] for "
				"LCD_EN control\n");
		return -EPERM;
	}

	err = gpio_request(GPIO_LCD_RST, "LCD_RST");
	if (err) {
		printk(KERN_ERR "failed to request GPL0[7] for "
				"LCD_EN control\n");
		return -EPERM;
	}

	if (enable) {
		gpio_set_value(GPIO_LCD_LDO_EN, GPIO_LEVEL_HIGH);
		gpio_set_value(GPIO_LCD_EN, GPIO_LEVEL_HIGH);
	} else {
		gpio_set_value(GPIO_LCD_RST, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_LCD_LDO_EN, GPIO_LEVEL_LOW);
		gpio_set_value(GPIO_LCD_EN, GPIO_LEVEL_LOW);
	}

	gpio_free(GPIO_LCD_LDO_EN);
	gpio_free(GPIO_LCD_EN);
	gpio_free(GPIO_LCD_RST);

	return 0;
}
#endif

static struct s3c_platform_fb fb_platform_data __initdata = {
	.hw_ver		= 0x70,
	.clk_name	= "fimd",
	.nr_wins	= 5,
#ifdef CONFIG_FB_S3C_DEFAULT_WINDOW
	.default_win	= CONFIG_FB_S3C_DEFAULT_WINDOW,
#else
	.default_win	= 0,
#endif
	.swap		= FB_SWAP_HWORD | FB_SWAP_WORD,
#if defined(CONFIG_FB_S3C_S6F1202A)
	.lcd		= &s6f1202a
#endif
#if defined(CONFIG_FB_S3C_S6C1372)
	.lcd		= &s6c1372
#endif
#ifdef CONFIG_FB_S3C_S6E8AB0
	.lcd		= &s6e8ab0
#endif
};

#if defined(CONFIG_FB_S3C_S6F1202A) || defined(CONFIG_FB_S3C_S6C1372)
static void __init lcd_fb_init(void)
{
	s3cfb_set_platdata(&fb_platform_data);
}
#endif

#ifdef CONFIG_FB_S3C_MIPI_LCD
extern struct platform_device s5p_device_dsim;

static void __init mipi_fb_init(void)
{
	struct s5p_platform_dsim *dsim_pd = NULL;
	struct mipi_ddi_platform_data *mipi_ddi_pd = NULL;
	struct dsim_lcd_config *dsim_lcd_info = NULL;

	/* set platform data */

	/* gpio pad configuration for rgb and spi interface. */
	lcd_cfg_gpio();

	/*
	 * register lcd panel data.
	 */
	printk(KERN_INFO "%s :: fb_platform_data.hw_ver = 0x%x\n", __func__, fb_platform_data.hw_ver);

	fb_platform_data.mipi_is_enabled = 1;
	fb_platform_data.interface_mode = FIMD_CPU_INTERFACE;

	dsim_pd = (struct s5p_platform_dsim *)
		s5p_device_dsim.dev.platform_data;

#ifdef CONFIG_FB_S3C_S6E8AB0
	strcpy(dsim_pd->lcd_panel_name, "s6e8ab0");
#endif

	dsim_pd->platform_rev = 1;
	/* dsim_pd->mipi_power = dsim_power; */
	dsim_lcd_info = dsim_pd->dsim_lcd_info;

#ifdef CONFIG_FB_S3C_S6E8AB0
	dsim_lcd_info->lcd_panel_info = (void *)&s6e8ab0;
	dsim_lcd_info->lcd_id = lcdtype;
#endif

	mipi_ddi_pd = (struct mipi_ddi_platform_data *)
		dsim_lcd_info->mipi_ddi_pd;
	mipi_ddi_pd->lcd_reset = reset_lcd;
	mipi_ddi_pd->lcd_power_on = lcd_power_on;

	platform_device_register(&s5p_device_dsim);

	s3cfb_set_platdata(&fb_platform_data);

	printk(KERN_INFO "platform data of %s lcd panel has been registered.\n",
//			((struct s3cfb_lcd *) fb_platform_data.lcd_data)->name);
			dsim_pd->lcd_panel_name);
}
#endif

#if defined(CONFIG_BACKLIGHT_PWM)
static struct platform_pwm_backlight_data smdk_backlight_data = {
	.pwm_id  = 1,
	.max_brightness = 255,
	.dft_brightness = 30,
	.pwm_period_ns  = 25000,
};

static struct platform_device smdk_backlight_device = {
	.name      = "backlight",
	.id        = -1,
	.dev        = {
		.parent = &s3c_device_timer[0].dev,
		.platform_data = &smdk_backlight_data,
	},
};

static void __init smdk_backlight_register(void)
{
	int ret;

	if (system_rev < 3)
		smdk_backlight_data.pwm_id = 0;

	ret = platform_device_register(&smdk_backlight_device);
	if (ret)
		printk(KERN_ERR "failed to register backlight device: %d\n",
				ret);
}
#endif

#ifdef CONFIG_FB_S3C_MDNIE
static struct platform_mdnie_data mdnie_data = {
	.display_type	= -1,
#if defined(CONFIG_FB_S3C_S6F1202A)
	.lcd_pd		= &p2_lcd_platform_data,
#elif defined(CONFIG_FB_S3C_S6C1372)
	.lcd_pd		= &p4_lcd_platform_data,
#endif
};

static struct platform_device mdnie_device = {
	.name		= "mdnie",
	.id		= -1,
	.dev	    = {
		.parent = &s5pv310_device_pd[PD_LCD0].dev,
		.platform_data = &mdnie_data,
	},
};

static void __init mdnie_device_register(void)
{
	int ret;

	mdnie_data.display_type = lcdtype;

	ret = platform_device_register(&mdnie_device);
	if (ret)
		printk(KERN_ERR "failed to register mdnie device: %d\n",
				ret);
}

#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resource[] = {
	{
		.flags = IORESOURCE_MEM,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources = ARRAY_SIZE(ram_console_resource),
	.resource = ram_console_resource,
};

#define RAM_CONSOLE_CMDLINE ("0x100000@0x5e900000")

static void __init setup_ram_console_mem(char *str)
{
	str =  RAM_CONSOLE_CMDLINE;
	unsigned size = memparse(str, &str);
	unsigned long flags;

	if (size && (*str == '@')) {
		unsigned long long base = 0;

		base = simple_strtoul(++str, &str, 0);
		if (reserve_bootmem(base, size, BOOTMEM_EXCLUSIVE)) {
			pr_err("%s: failed reserving size %d "
			       "at base 0x%llx\n", __func__, size, base);
			return;
		}

		ram_console_resource[0].start = base;
		ram_console_resource[0].end = base + size - 1;
		pr_err("%s: %x at %x\n", __func__, size, base);
	}
}
/* without modifying the bootloader or harcoding cmdlines (which can mess up reboots), no way to pass 
   a ram_console command line.  Just work around that little issue by triggering on a different parameter
   and hardcoding the parameters to ram_console in the function */
__setup("loglevel=", setup_ram_console_mem);
#endif

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data pmem_pdata = {
	.name = "pmem",
	.no_allocator = 1,
	.cached = 0,
	.start = 0, // will be set during proving pmem driver.
	.size = 0 // will be set during proving pmem driver.
};

static struct android_pmem_platform_data pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.no_allocator = 1,
	.cached = 0,
	/* .buffered = 1, */
	.start = 0,
	.size = 0,
};

static struct platform_device pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &pmem_gpu1_pdata },
};

static void __init android_pmem_set_platdata(void)
{
#if defined(CONFIG_S5P_MEM_CMA)
	pmem_pdata.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K;
	pmem_gpu1_pdata.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K;
#elif defined(CONFIG_S5P_MEM_BOOTMEM)
	pmem_pdata.start = (u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM, 0);
	pmem_pdata.size = (u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM, 0);
	pmem_gpu1_pdata.start = (u32)s5p_get_media_memory_bank(S5P_MDEV_PMEM_GPU1, 0);
	pmem_gpu1_pdata.size = (u32)s5p_get_media_memsize_bank(S5P_MDEV_PMEM_GPU1, 0);
#endif
}
#endif

#ifdef CONFIG_BATTERY_SAMSUNG
struct platform_device samsung_device_battery = {
	.name	= "samsung-fake-battery",
	.id	= -1,
};
#endif

//extern usb_path_type usb_sel_status;
#ifdef CONFIG_BATTERY_SEC

void sec_bat_gpio_init(void)
{

	s3c_gpio_cfgpin(GPIO_TA_nCONNECTED, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TA_nCONNECTED, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_TA_nCHG, S3C_GPIO_INPUT);
	s3c_gpio_setpull(GPIO_TA_nCHG, S3C_GPIO_PULL_NONE);

	s3c_gpio_cfgpin(GPIO_TA_EN, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TA_EN, S3C_GPIO_PULL_NONE);
	gpio_set_value(GPIO_TA_EN, 0);

	s3c_gpio_cfgpin(GPIO_CURR_ADJ, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_CURR_ADJ, S3C_GPIO_PULL_NONE);

	pr_info("BAT : Battery GPIO initialized.\n");
}

static void  sec_charger_cb(bool bAttached)
{
	if (charging_cbs.tsp_set_charging_cable)
		charging_cbs.tsp_set_charging_cable(bAttached);
	is_cable_attached = bAttached;
	printk(KERN_ERR "sec_charger_cb: is_cable_attated=%d, bAttached=%d\n",is_cable_attached,bAttached);
}

static struct sec_battery_platform_data sec_battery_platform = {
	.charger = {
		.enable_line = GPIO_TA_EN,
		.connect_line = GPIO_TA_nCONNECTED,
		.fullcharge_line = GPIO_TA_nCHG,
		.currentset_line = GPIO_CURR_ADJ,
	},
#if defined(CONFIG_SMB136_CHARGER) || defined(CONFIG_SMB347_CHARGER)
	.set_charging_state = sec_bat_set_charging_state,
	.get_charging_state = sec_bat_get_charging_state,
	.set_charging_current = sec_bat_set_charging_current,
	.get_charging_current = sec_bat_get_charging_current,
#endif
	.init_charger_gpio = sec_bat_gpio_init,
#if defined(CONFIG_TOUCHSCREEN_MELFAS)
	.inform_charger_connection = sec_charger_melfas_cb,
#elif defined(CONFIG_TOUCHSCREEN_MXT1386)
	.inform_charger_connection = sec_mxt1386_charger_infom,
#else
	.inform_charger_connection = sec_charger_cb,
#endif

#if defined(CONFIG_MACH_P8LTE_REV00)
    .temp_high_threshold = 51000,	/* 51c   */
	.temp_high_recovery = 43400,	/* 43.4c */
	.temp_low_recovery = 700,		/* 0.7c  */
	.temp_low_threshold = -4200,	/* -4.2c */
    .recharge_voltage = 4000,	    /*4.00V */
#else
	.temp_high_threshold = 50000,	/* 50c */
	.temp_high_recovery = 42000,	/* 42c */
	.temp_low_recovery = 2000,		/* 2c */
	.temp_low_threshold = 0,		/* 0c */
    .recharge_voltage = 4150,	    /*4.15V */
#endif

	.charge_duration = 10*60*60,	/* 10 hour */
	.recharge_duration = 1.5*60*60,	/* 1.5 hour */
	.check_lp_charging_boot = check_bootmode,
	.check_jig_status = check_jig_on
};

static struct platform_device sec_battery_device = {
	.name = "sec-battery",
	.id = -1,
	.dev = {
		.platform_data = &sec_battery_platform,
	},
};
#endif /* CONFIG_BATTERY_SEC */

#ifdef CONFIG_SEC_THERMISTOR
static struct sec_therm_platform_data sec_therm_pdata = {
	.adc_channel	= 7,
	.adc_arr_size	= ARRAY_SIZE(adc_temp_table),
	.adc_table	= adc_temp_table,
	.polling_interval = 60 * 1000, /* msecs */
};

static struct platform_device sec_device_thermistor = {
	.name = "sec-thermistor",
	.id = -1,
	.dev.platform_data = &sec_therm_pdata,
};
#endif /* CONFIG_SEC_THERMISTOR */

static struct platform_device sec_device_rfkill = {
	.name = "bt_rfkill",
	.id	  = -1,
};

#ifdef CONFIG_KEYBOARD_GPIO
#define GPIO_KEYS(_code, _gpio, _active_low, _iswake, _hook)		\
{					\
	.code = _code,			\
	.gpio = _gpio,	\
	.active_low = _active_low,		\
	.type = EV_KEY,			\
	.wakeup = _iswake,		\
	.debounce_interval = 10,	\
	.isr_hook = _hook			\
}

struct gpio_keys_button px_buttons[] = {
	GPIO_KEYS(KEY_VOLUMEUP, GPIO_VOL_UP,
		1, 0, sec_debug_check_crash_key),
	GPIO_KEYS(KEY_VOLUMEDOWN, GPIO_VOL_DOWN,
		1, 0, sec_debug_check_crash_key),
	GPIO_KEYS(KEY_POWER, GPIO_nPOWER,
		1, 1, sec_debug_check_crash_key),
};

struct gpio_keys_platform_data px_keys_platform_data = {
	.buttons	= px_buttons,
	.nbuttons	 = ARRAY_SIZE(px_buttons),
};

struct platform_device px_gpio_keys = {
	.name	= "sec_key",
	.dev.platform_data = &px_keys_platform_data,
};
#endif

#ifdef CONFIG_SEC_DEV_JACK
static void sec_set_jack_micbias(bool on)
{
#ifdef CONFIG_SND_SOC_USE_EXTERNAL_MIC_BIAS
		gpio_set_value(GPIO_EAR_MIC_BIAS_EN, on);
		printk(KERN_DEBUG"[SND] %s %d status : %d\n", __func__, __LINE__, on);
#endif

	return;
}

static struct sec_jack_zone sec_jack_zones[] = {
	{
		/* adc == 0, unstable zone, default to 3pole if it stays
		 * in this range for 300ms (15ms delays, 20 samples)
		 */
		.adc_high = 0,
		.delay_ms = 15,
		.check_count = 20,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 0 < adc <= 900, unstable zone, default to 3pole if it stays
		 * in this range for 800ms (10ms delays, 80 samples)
		 */
		.adc_high = 1250,
		.delay_ms = 10,
		.check_count = 80,
		.jack_type = SEC_HEADSET_3POLE,
	},
	{
		/* 900 < adc <= 2000, unstable zone, default to 4pole if it
		 * stays in this range for 800ms (10ms delays, 80 samples)
		 */
		.adc_high = 2000,
		.delay_ms = 10,
		.check_count = 80,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* 2000 < adc <= 3550, 4 pole zone, default to 4pole if it
		 * stays in this range for 100ms (10ms delays, 10 samples)
		 */
		.adc_high = 3550,
		.delay_ms = 10,
		.check_count = 10,
		.jack_type = SEC_HEADSET_4POLE,
	},
	{
		/* adc > 3550, unstable zone, default to 3pole if it stays
		 * in this range for two seconds (10ms delays, 200 samples)
		 */
		.adc_high = 0x7fffffff,
		.delay_ms = 10,
		.check_count = 200,
		.jack_type = SEC_HEADSET_3POLE,
	},
};

/* To support 3-buttons earjack */
static struct sec_jack_buttons_zone sec_jack_buttons_zones[] = {
	{
		/* 0 <= adc <=170, stable zone */
		.code		= KEY_MEDIA,
		.adc_low	= 0,
		.adc_high	= 170,
	},
	{
		/* 171 <= adc <= 370, stable zone */
		.code		= KEY_VOLUMEUP,
		.adc_low	= 171,
		.adc_high	= 370,
	},
	{
		/* 371 <= adc <= 850, stable zone */
		.code		= KEY_VOLUMEDOWN,
		.adc_low	= 371,
		.adc_high	= 850,
	},
};

static struct sec_jack_platform_data sec_jack_data = {
	.set_micbias_state	= sec_set_jack_micbias,
	.zones			= sec_jack_zones,
	.num_zones		= ARRAY_SIZE(sec_jack_zones),
	.buttons_zones		= sec_jack_buttons_zones,
	.num_buttons_zones	= ARRAY_SIZE(sec_jack_buttons_zones),
	.det_gpio		= GPIO_DET_35,
	.send_end_gpio		= GPIO_EAR_SEND_END,
};

static struct platform_device sec_device_jack = {
	.name			= "sec_jack",
	.id			= 1, /* will be used also for gpio_event id */
	.dev.platform_data	= &sec_jack_data,
};
#endif	/* #ifdef CONFIG_SEC_DEV_JACK */

static struct resource pmu_resource[] = {
	[0] = {
		.start = IRQ_PMU_0,
		.end   = IRQ_PMU_0,
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.start = IRQ_PMU_1,
		.end   = IRQ_PMU_1,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device pmu_device = {
	.name		= "arm-pmu",
	.id		= ARM_PMU_DEVICE_CPU,
	.resource	= pmu_resource,
	.num_resources	= 2,
};

#ifdef CONFIG_S5PV310_WATCHDOG_RESET
static struct platform_device watchdog_reset_device = {
	.name = "watchdog-reset",
	.id = -1,
};
#endif

#if defined (CONFIG_SAMSUNG_PHONE_DPRAM_INTERNAL) ||defined (CONFIG_SAMSUNG_PHONE_DPRAM_INTERNAL_MODULE)
struct platform_device sec_device_dpram = {
	.name	= "dpram-device",
	.id	= -1,
};
#endif

#if defined(CONFIG_FB_S3C_S6C1372) || defined(CONFIG_FB_S3C_S6F1202A)
static struct platform_device lcd_s6c1372 = {
	.name   = "s6c1372",
	.id	= -1,
};
#endif

static struct platform_device *smdkc210_devices[] __initdata = {
#ifdef CONFIG_S5PV310_WATCHDOG_RESET
	&watchdog_reset_device,
#endif
#ifdef CONFIG_S5PV310_DEV_PD
	&s5pv310_device_pd[PD_MFC],
	&s5pv310_device_pd[PD_G3D],
	&s5pv310_device_pd[PD_LCD0],
	&s5pv310_device_pd[PD_LCD1],
	&s5pv310_device_pd[PD_CAM],
	&s5pv310_device_pd[PD_GPS],
	&s5pv310_device_pd[PD_TV],
	/* &s5pv310_device_pd[PD_MAUDIO], */
#endif

	&smdkc210_smsc911x,

#ifdef CONFIG_S3C_ADC
	&s3c_device_adc,
#endif

#ifndef CONFIG_MACH_P4W_REV00 /* QCA */
	&p4w_wlan_ar6000_pm_device,
#endif
#ifdef CONFIG_FB_S3C
	&s3c_device_fb,
#endif
#ifdef CONFIG_I2C_S3C2410
	&s3c_device_i2c0,
#if defined(CONFIG_S3C_DEV_I2C1)
	&s3c_device_i2c1,
#endif
#if defined(CONFIG_S3C_DEV_I2C2)
	&s3c_device_i2c2,
#endif
#if defined(CONFIG_S3C_DEV_I2C3)
	&s3c_device_i2c3,
#endif
#if defined(CONFIG_S3C_DEV_I2C4)
	&s3c_device_i2c4,
#endif
#if defined(CONFIG_S3C_DEV_I2C5)
	&s3c_device_i2c5,
#endif
#if defined(CONFIG_S3C_DEV_I2C6)
	&s3c_device_i2c6,
#endif
#if defined(CONFIG_S3C_DEV_I2C7)
	&s3c_device_i2c7,
#endif
#if defined(CONFIG_S3C_DEV_I2C8_EMUL)
	&s3c_device_i2c8,
#endif
#if defined(CONFIG_S3C_DEV_I2C9_EMUL)
	&s3c_device_i2c9,
#endif
#if defined(CONFIG_S3C_DEV_I2C10_EMUL)
	&s3c_device_i2c10,
#endif
#if defined(CONFIG_S3C_DEV_I2C11_EMUL)
	&s3c_device_i2c11,
#endif
#if defined(CONFIG_SMB136_CHARGER)
	&s3c_device_i2c12,
#endif
#if defined(CONFIG_VIDEO_SR200PC20) && !defined(CONFIG_MACH_P4W_REV01)
	&s3c_device_i2c13,
#endif
#if defined(CONFIG_MHL_SII9234)
	&s3c_device_i2c15,	/* MHL */
#endif
#endif
#ifdef CONFIG_USBHUB_USB3503
	&s3c_device_i2c17,	/* USB HUB */
#endif
#ifdef CONFIG_BATTERY_SAMSUNG
	&samsung_device_battery,
#endif
#ifdef CONFIG_BATTERY_SEC
	&sec_battery_device,
#endif
#ifdef CONFIG_SEC_THERMISTOR
	&sec_device_thermistor,
#endif
	/* consumer driver should resume after resuming i2c drivers */
	&c1_regulator_consumer,
#ifdef CONFIG_SND_S3C64XX_SOC_I2S_V4
	&s5pv310_device_iis0,
#endif
#ifdef CONFIG_SND_S3C_SOC_PCM
	&s5pv310_device_pcm1,
#endif
#ifdef CONFIG_SND_SOC_SMDK_WM9713
	&s5pv310_device_ac97,
#endif
#ifdef CONFIG_SND_SAMSUNG_SOC_SPDIF
	&s5pv310_device_spdif,
#endif
#ifdef CONFIG_SND_S5P_RP
	&s5pv310_device_rp,
#endif

#ifdef CONFIG_MTD_NAND
	&s3c_device_nand,
#endif
#ifdef CONFIG_MTD_ONENAND
	&s5p_device_onenand,
#endif

#ifdef CONFIG_S3C_DEV_HSMMC
	&s3c_device_hsmmc0,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	&s3c_device_hsmmc1,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	&s3c_device_hsmmc3,
#endif

#ifdef CONFIG_S5P_DEV_MSHC
	&s3c_device_mshci,
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
	&s3c_device_ts,
#endif
#ifdef CONFIG_S3C_DEV_ADC1
	&s3c_device_ts1,
#endif
#endif

#ifdef CONFIG_TOUCHSCREEN_S5PV310
	&s3c_device_ts,
#endif

#ifdef CONFIG_VIDEO_TVOUT
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif

#ifdef CONFIG_S3C2410_WATCHDOG
	&s3c_device_wdt,
#endif

#ifdef CONFIG_ANDROID_PMEM
	&pmem_device,
	&pmem_gpu1_device,
#endif

#ifdef CONFIG_VIDEO_FIMC
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
#ifdef CONFIG_VIDEO_FIMC_MIPI
	&s3c_device_csis0,
	&s3c_device_csis1,
#endif
#endif
#ifdef CONFIG_VIDEO_JPEG
	&s5p_device_jpeg,
#endif
#if defined(CONFIG_VIDEO_MFC50) || defined(CONFIG_VIDEO_MFC5X)
	&s5p_device_mfc,
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	&s5p_device_fimg2d,
#endif

#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	&s3c_device_rndis,
#endif
#ifdef CONFIG_USB_ANDROID
	&s3c_device_android_usb,
	&s3c_device_usb_mass_storage,
#endif
#if defined(CONFIG_S3C64XX_DEV_SPI0)
	&s5pv310_device_spi0,
#endif

#ifdef CONFIG_S5P_SYSMMU
	&s5p_device_sysmmu,
#endif

#ifdef CONFIG_S3C_DEV_GIB
	&s3c_device_gib,
#endif

#ifdef CONFIG_S3C_DEV_RTC
	&s3c_device_rtc,
#endif

	&s5p_device_ace,
#ifdef CONFIG_SATA_AHCI_PLATFORM
	&s5pv310_device_sata,
#endif
#ifdef CONFIG_KEYBOARD_GPIO
	&px_gpio_keys,
#endif
#ifdef CONFIG_SEC_DEV_JACK
	&sec_device_jack,
#endif
	&sec_device_rfkill,

	&pmu_device,

#ifdef CONFIG_DEV_THERMAL
	&s5p_device_tmu,
#endif

#ifdef CONFIG_HAVE_PWM
	&s3c_device_timer[0],
	&s3c_device_timer[1],
	&s3c_device_timer[2],
	&s3c_device_timer[3],
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif

#if defined(CONFIG_IRDA)
/*Ir-LED*/
	&ir_remote_device,
/*Ir-LED*/
#endif
#ifdef CONFIG_30PIN_CONN
	&sec_device_connector,
#ifdef CONFIG_SEC_KEYBOARD_DOCK
	&sec_keyboard,
#endif
#endif
#if defined (CONFIG_SAMSUNG_PHONE_DPRAM_INTERNAL) ||defined (CONFIG_SAMSUNG_PHONE_DPRAM_INTERNAL_MODULE)
        &sec_device_dpram,
#endif
#if defined(CONFIG_FB_S3C_S6C1372) || defined(CONFIG_FB_S3C_S6F1202A)
	&lcd_s6c1372,
#endif

#ifdef CONFIG_USB_HOST_NOTIFY
	&host_notifier_device,
#endif
};

#ifdef CONFIG_VIDEO_TVOUT
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

#if 0
static void __init sromc_setup(void)
{
	u32 tmp;

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(0xffff);
	tmp |= (0x9999);
	__raw_writel(tmp, S5P_SROM_BW);

	__raw_writel(0xff1ffff1, S5P_SROM_BC1);

	tmp = __raw_readl(S5P_VA_GPIO + 0x120);
	tmp &= ~(0xffffff);
	tmp |= (0x221121);
	__raw_writel(tmp, (S5P_VA_GPIO + 0x120));

	__raw_writel(0x22222222, (S5P_VA_GPIO + 0x180));
	__raw_writel(0x22222222, (S5P_VA_GPIO + 0x1a0));
	__raw_writel(0x22222222, (S5P_VA_GPIO + 0x1c0));
	__raw_writel(0x22222222, (S5P_VA_GPIO + 0x1e0));
}
#endif

#if defined(CONFIG_S5P_MEM_CMA)
static void __init s5pv310_reserve(void);
#endif
static void __init smdkc210_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkc210_uartcfgs, ARRAY_SIZE(smdkc210_uartcfgs));

#ifdef CONFIG_MTD_NAND
	s3c_device_nand.name = "s5pv310-nand";
#endif

#if defined(CONFIG_S5P_MEM_CMA)
	s5pv310_reserve();
#elif defined(CONFIG_S5P_MEM_BOOTMEM)
	s5p_reserve_bootmem();
#endif
	sec_getlog_supply_meminfo(meminfo.bank[0].size, meminfo.bank[0].start,
				  meminfo.bank[1].size, meminfo.bank[1].start);

	/* as soon as INFORM6 is visible, sec_debug is ready to run */
	sec_debug_init();
}

#ifdef CONFIG_TOUCHSCREEN_S3C2410
static struct s3c2410_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
};
#endif

#ifdef  CONFIG_TOUCHSCREEN_S5PV310
static struct s3c_ts_mach_info s3c_ts_platform __initdata = {
	.delay                  = 10000,
	.presc                  = 49,
	.oversampling_shift     = 2,
	.resol_bit              = 12,
	.s3c_adc_con            = ADC_TYPE_2,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_S5PV310) || defined(CONFIG_S3C_ADC)
static struct s3c_adc_mach_info s3c_adc_platform __initdata = {
	/*    s5pc100 supports 12-bit resolution */
	.delay          = 10000,
	.presc          = 49,
	.resolution     = 12,
};
#endif

static void smdkc210_power_off(void)
{
	int poweroff_try = 0;

	local_irq_disable();

	pr_emerg("%s : cable state=%d \n", __func__, is_cable_attached);

	while (1) {
		/* Check reboot charging */
		if (is_cable_attached || (poweroff_try >= 5)) {
			pr_emerg("%s: charger connected(%d) or power off failed(%d), reboot!\n",
				 __func__, is_cable_attached, poweroff_try);
			writel(0x0, S5P_INFORM2); /* To enter LP charging */

			flush_cache_all();
			outer_flush_all();
			arch_reset(0, 0);

			pr_emerg("%s: waiting for reboot\n", __func__);
			while (1);
		}

		/* wait for power button release */
		if (gpio_get_value(GPIO_nPOWER)) {
			pr_emerg("%s: set PS_HOLD low\n", __func__);

			/* power off code
			 * PS_HOLD Out/High -->
			 * Low PS_HOLD_CONTROL, R/W, 0x1002_330C
			 */
			writel(readl(S5P_PS_HOLD_CONTROL) & 0xFFFFFEFF,
					S5P_PS_HOLD_CONTROL);

			++poweroff_try;
			pr_emerg("%s: Should not reach here! (poweroff_try:%d)\n",
				 __func__, poweroff_try);
		} else {
			/* if power button is not released, wait and check TA again */
			pr_info("%s: PowerButton is not released.\n", __func__);
		}
		mdelay(1000);
	}
}

#define REBOOT_PREFIX		0x12345670
#define REBOOT_MODE_NONE	0
#define REBOOT_MODE_DOWNLOAD	1
#define REBOOT_MODE_UPLOAD	2
#define REBOOT_MODE_CHARGING	3
#define REBOOT_MODE_RECOVERY	4
#define REBOOT_MODE_ARM11_FOTA	5
#if defined(CONFIG_TARGET_LOCALE_NA)
#define REBOOT_MODE_ARM9_FOTA	6
#endif

static void px_reboot(char str, const char *cmd)
{
	local_irq_disable();

	pr_emerg("%s (%d, %s)\n", __func__, str, cmd ? cmd : "(null)");

	writel(0x12345678, S5P_INFORM2);	/* Don't enter lpm mode */

	if (!cmd) {
		writel(REBOOT_PREFIX | REBOOT_MODE_NONE, S5P_INFORM3);
	} else {
		if (!strcmp(cmd, "fota"))
			writel(REBOOT_PREFIX | REBOOT_MODE_ARM11_FOTA,
			       S5P_INFORM3);
#if defined(CONFIG_TARGET_LOCALE_NA)
		else if (!strcmp(cmd, "arm9_fota"))
			writel(REBOOT_PREFIX | REBOOT_MODE_ARM9_FOTA,
			       S5P_INFORM3);
#endif
		else if (!strcmp(cmd, "recovery"))
			writel(REBOOT_PREFIX | REBOOT_MODE_RECOVERY,
			       S5P_INFORM3);
		else if (!strcmp(cmd, "bootloader") || !strcmp(cmd, "download"))
			writel(REBOOT_PREFIX | REBOOT_MODE_DOWNLOAD,
			       S5P_INFORM3);
		else if (!strcmp(cmd, "upload"))
			writel(REBOOT_PREFIX | REBOOT_MODE_UPLOAD,
			       S5P_INFORM3);
		else
			writel(REBOOT_PREFIX | REBOOT_MODE_NONE,
			       S5P_INFORM3);
	}

	flush_cache_all();
	outer_flush_all();
	arch_reset(0, 0);

	pr_emerg("%s: waiting for reboot\n", __func__);
	while (1);
}

static void __init universal_tsp_init(void)
{
	int gpio;
	int gpio_touch_id;

#if defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01) || defined(CONFIG_MACH_P8LTE_REV00)
	/* TSP_LDO_ON: XMDMADDR_11 */
	gpio = GPIO_TSP_LDO_ON;
	gpio_request(gpio, "TSP_LDO_ON");
	gpio_direction_output(gpio, 1);
	gpio_export(gpio, 0);
#else
	gpio = GPIO_TSP_RST;
	gpio_request(gpio, "TSP_RST");
	gpio_direction_output(gpio, 1);
	gpio_export(gpio, 0);
#endif

#if defined(CONFIG_MACH_P8_REV00) || defined(CONFIG_MACH_P8_REV01) || defined(CONFIG_MACH_P8LTE_REV00)
	/* TSP_INT: XMDMADDR_7 */
	gpio = GPIO_TSP_INT_18V;
	gpio_request(gpio, "TSP_INT_18V");

#else
	/* TSP_INT: XMDMADDR_7 */
	gpio = GPIO_TSP_INT;
	gpio_request(gpio, "TSP_INT");
#endif
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	i2c_devs3[0].irq = gpio_to_irq(gpio);
	printk("%s touch : %d\n", __func__, i2c_devs3[0].irq);

#if defined(CONFIG_MACH_P2_REV00) || defined(CONFIG_MACH_P2_REV01) || defined(CONFIG_MACH_P2_REV02)

	gpio = GPIO_TSP_VENDOR1;
	gpio_request(gpio, "GPIO_TSP_VENDOR1");
	s3c_gpio_cfgpin(gpio, S3C_GPIO_INPUT);
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);

	gpio = GPIO_TSP_VENDOR2;
	gpio_request(gpio, "GPIO_TSP_VENDOR2");
	s3c_gpio_cfgpin(gpio, S3C_GPIO_INPUT);
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);


	s3c_gpio_cfgpin(GPIO_TSP_RST, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(GPIO_TSP_RST, S3C_GPIO_PULL_NONE);

	if(system_rev < 3){
		gpio_touch_id = gpio_get_value(GPIO_TSP_VENDOR1);
	}else{
		gpio_touch_id = gpio_get_value(GPIO_TSP_VENDOR1) + gpio_get_value(GPIO_TSP_VENDOR2)*2;
	}
	printk("[TSP] %s : gpio_touch_id = %d, system_rev = %d\n",__func__,gpio_touch_id,system_rev);
	ts_data.gpio_touch_id = gpio_touch_id;
#endif

}

#if defined(CONFIG_FB_S3C_S6C1372) || defined(CONFIG_FB_S3C_S6F1202A)
static int lcd_cfg_gpio(void)
{
	return 0;
}

int s6c1372_panel_gpio_init(void)
{
	int ret;

	lcd_cfg_gpio();

	/* GPIO Initialize  for S6C1372 LVDS panel */
	ret = gpio_request(GPIO_LCD_EN, "GPIO_LCD_EN");
	if (ret) {
		pr_err("failed to request LCD_EN GPIO%d\n",
				GPIO_LCD_EN);
		return ret;
	}
	ret = gpio_request(GPIO_LVDS_NSHDN, "GPIO_LVDS_NSHDN");
	if (ret) {
		pr_err("failed to request LVDS GPIO%d\n",
				GPIO_LVDS_NSHDN);
		return ret;
	}

	gpio_direction_output(GPIO_LCD_EN, 1);
	gpio_direction_output(GPIO_LVDS_NSHDN, 1);

	gpio_free(GPIO_LCD_EN);
	gpio_free(GPIO_LVDS_NSHDN);

#ifdef GPIO_LED_BACKLIGHT_RESET
	ret = gpio_request(GPIO_LED_BACKLIGHT_RESET, "GPIO_LED_BACKLIGHT_RESET");
	if (ret) {
		pr_err("failed to request LVDS GPIO%d\n",
				GPIO_LED_BACKLIGHT_RESET);
		return ret;
	}
	gpio_direction_output(GPIO_LED_BACKLIGHT_RESET, 1);
	gpio_free(GPIO_LED_BACKLIGHT_RESET);
#endif
	return 0;
}
#endif

static void __init px_factory_init(void)
{
	int ret;

	ret = gpio_request(GPIO_IF_CON_SENSE, "IF_CON_SENSE");
	if (ret < 0) {
		pr_err("%s: Failed to request IF_CON_SENSE\n", __func__);
		return;
	}

	s3c_gpio_setpull(GPIO_IF_CON_SENSE, S3C_GPIO_PULL_NONE);
	gpio_direction_input(GPIO_IF_CON_SENSE);
	gpio_export(GPIO_IF_CON_SENSE, 0);

	pr_info("%s: IF_CON_SENSE:%d\n", __func__,
			gpio_get_value(GPIO_IF_CON_SENSE));

	BUG_ON(!sec_class);
	factory_dev =
		device_create(sec_class, NULL, 0, NULL, "factory");
	if (IS_ERR(factory_dev))
		pr_err("Failed to create device(if_con_sense)!\n");

	gpio_export_link(factory_dev, "IF_CON_SENSE", GPIO_IF_CON_SENSE);
}

static void __init smdkc210_machine_init(void)
{
	struct clk *sclk = NULL;
	struct clk *prnt = NULL;
#if defined(CONFIG_S3C64XX_DEV_SPI0)
	struct device *spi0_dev = &s5pv310_device_spi0.dev;
#endif
#ifdef CONFIG_FB_S3C_AMS369FG06
	struct device *spi_dev = &s5pv310_device_spi1.dev;
#elif defined(CONFIG_S3C64XX_DEV_SPI1)
	struct device *spi1_dev = &s5pv310_device_spi1.dev;
#endif
#if defined(CONFIG_S3C64XX_DEV_SPI2)
	struct device *spi2_dev = &s5pv310_device_spi2.dev;
#endif
	/* to support system shut down */
	pm_power_off = smdkc210_power_off;
	arm_pm_restart = px_reboot;

	/* initialise the gpios */
	c1_config_gpio_table();
	s3c_config_sleep_gpio_table = c1_config_sleep_gpio_table;

#ifdef CONFIG_ANDROID_PMEM
	android_pmem_set_platdata();
#endif

	s3c_pm_init();

#ifndef CONFIG_MACH_P4W_REV00 /* QCA gpio configuration */
	config_wlan_gpio();
#endif


#if defined(CONFIG_S5PV310_DEV_PD) && !defined(CONFIG_PM_RUNTIME)
	/*
	 * These power domains should be always on
	 * without runtime pm support.
	 */
	s5pv310_pd_enable(&s5pv310_device_pd[PD_MFC].dev);
	s5pv310_pd_enable(&s5pv310_device_pd[PD_G3D].dev);
	s5pv310_pd_enable(&s5pv310_device_pd[PD_LCD0].dev);
	s5pv310_pd_enable(&s5pv310_device_pd[PD_LCD1].dev);
	s5pv310_pd_enable(&s5pv310_device_pd[PD_CAM].dev);
	s5pv310_pd_enable(&s5pv310_device_pd[PD_TV].dev);
	s5pv310_pd_enable(&s5pv310_device_pd[PD_GPS].dev);
#endif

	/* 400 kHz for initialization of MMC Card  */
	__raw_writel((__raw_readl(S5P_CLKDIV_FSYS3) & 0xfffffff0)
		     | 0x9, S5P_CLKDIV_FSYS3);
	__raw_writel((__raw_readl(S5P_CLKDIV_FSYS2) & 0xfff0fff0)
		     | 0x80008, S5P_CLKDIV_FSYS2);
	__raw_writel((__raw_readl(S5P_CLKDIV_FSYS1) & 0xfff0fff0)
		     | 0x80009, S5P_CLKDIV_FSYS1);

#ifdef CONFIG_I2C_S3C2410
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

#ifdef CONFIG_S3C_DEV_I2C1

#ifdef CONFIG_MPU_SENSORS_MPU3050
	ak8975_init();
	mpu3050_init();
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_mpu_sensor_board_info
			, ARRAY_SIZE(i2c_mpu_sensor_board_info));
#else
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
#endif
#endif

#ifdef CONFIG_S3C_DEV_I2C2
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, i2c_devs2, ARRAY_SIZE(i2c_devs2));
#endif
#ifdef CONFIG_S3C_DEV_I2C3
	universal_tsp_init();
	s3c_i2c3_set_platdata(NULL);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));
#endif
#ifdef CONFIG_S3C_DEV_I2C4
#ifdef CONFIG_EPEN_WACOM_G5SP
	p4w_wacom_init();
#endif /* CONFIG_EPEN_WACOM_G5SP */
	s3c_i2c4_set_platdata(NULL);
	i2c_register_board_info(4, i2c_devs4, ARRAY_SIZE(i2c_devs4));
#endif
#ifdef CONFIG_S3C_DEV_I2C5
	s3c_i2c5_set_platdata(NULL);
	s3c_gpio_cfgpin(GPIO_PMIC_IRQ, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_PMIC_IRQ, S3C_GPIO_PULL_NONE);
	i2c_devs5[0].irq = gpio_to_irq(GPIO_PMIC_IRQ);

#if defined(CONFIG_MACH_P2_REV02) || defined(CONFIG_MACH_P4W_REV00) || defined(CONFIG_MACH_P4W_REV01)
#ifdef CONFIG_VIBETONZ
	if (system_rev >= 3)
		max8997_motor.pwm_id = 0;
#endif
#endif
	i2c_register_board_info(5, i2c_devs5, ARRAY_SIZE(i2c_devs5));
#endif
#ifdef CONFIG_S3C_DEV_I2C6
	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
#endif
#ifdef CONFIG_S3C_DEV_I2C7
	s3c_i2c7_set_platdata(NULL);
	i2c_register_board_info(7, i2c_ak8975_board_info, ARRAY_SIZE(i2c_ak8975_board_info));
#endif
#ifdef CONFIG_S3C_DEV_I2C8_EMUL
	i2c_register_board_info(8, i2c_devs8_emul, ARRAY_SIZE(i2c_devs8_emul));
#endif
#ifdef CONFIG_S3C_DEV_I2C9_EMUL
	i2c_register_board_info(9, i2c_devs9_emul, ARRAY_SIZE(i2c_devs9_emul));
#endif
#ifdef CONFIG_S3C_DEV_I2C10_EMUL
	i2c_register_board_info(10, i2c_devs10_emul, ARRAY_SIZE(i2c_devs10_emul));
#endif
#ifdef CONFIG_S3C_DEV_I2C11_EMUL

#ifdef CONFIG_OPTICAL_GP2A
	#if defined(CONFIG_OPTICAL_WAKE_ENABLE)
	if(system_rev >= 0x03)
		i2c_register_board_info(11, i2c_wake_devs11, ARRAY_SIZE(i2c_wake_devs11));
	else
		i2c_register_board_info(11, i2c_devs11, ARRAY_SIZE(i2c_devs11));
	#else
	/* optical sensor */
	i2c_register_board_info(11, i2c_devs11, ARRAY_SIZE(i2c_devs11));
	#endif
#else
	light_sensor_init();
	i2c_register_board_info(11, i2c_bh1721_emul, ARRAY_SIZE(i2c_bh1721_emul));
#endif

#endif
#if defined(CONFIG_SMB136_CHARGER)
	/* smb charger */
	smb_gpio_init();
	i2c_register_board_info(12, i2c_devs12_emul, ARRAY_SIZE(i2c_devs12_emul));
#endif
#if defined(CONFIG_SMB347_CHARGER)
	if (system_rev >= 02) {
		printk(KERN_INFO "%s : Add smb347 charger.\n", __func__);
		/* smb charger */
		smb_gpio_init();
		i2c_register_board_info(12, i2c_devs12_emul, ARRAY_SIZE(i2c_devs12_emul));
		platform_device_register(&s3c_device_i2c12);
	}
#endif
	/* I2C13 EMUL */
#if defined(CONFIG_VIDEO_SR200PC20) && defined(CONFIG_MACH_P4W_REV01)
	if (system_rev < 2)
		platform_device_register(&s3c_device_i2c13);
#endif

#if defined(CONFIG_MHL_SII9234)
	sii9234_init();
	i2c_register_board_info(15, i2c_devs15, ARRAY_SIZE(i2c_devs15));
#endif
#endif

#ifdef CONFIG_USBHUB_USB3503
	i2c_register_board_info(17, i2c_devs17_emul, ARRAY_SIZE(i2c_devs17_emul));
#endif

#ifdef CONFIG_FB_S3C
#ifdef CONFIG_FB_S3C_AMS369FG06
#else
	s3cfb_set_platdata(NULL);
#endif
#endif

#ifdef CONFIG_VIDEO_FIMC
	/* fimc */
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(&fimc_plat);
	s3c_fimc3_set_platdata(&fimc_plat);
#ifdef CONFIG_ITU_A
#endif
#ifdef CONFIG_ITU_B
	smdkv310_cam1_reset(1);
#endif
#ifdef CONFIG_VIDEO_FIMC_MIPI
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);
#endif
#endif

#ifdef CONFIG_VIDEO_MFC5X
#ifdef CONFIG_S5PV310_DEV_PD
	s5p_device_mfc.dev.parent = &s5pv310_device_pd[PD_MFC].dev;
#endif
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#ifdef CONFIG_S5PV310_DEV_PD
	s5p_device_fimg2d.dev.parent = &s5pv310_device_pd[PD_LCD0].dev;
#endif
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&smdkc210_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&smdkc210_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&smdkc210_hsmmc2_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC3
	s3c_sdhci3_set_platdata(&smdkc210_hsmmc3_pdata);
#endif
#ifdef CONFIG_S5P_DEV_MSHC
	s3c_mshci_set_platdata(&smdkc210_mshc_pdata);
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
	s3c24xx_ts_set_platdata(&s3c_ts_platform);
#endif
#ifdef CONFIG_S3C_DEV_ADC1
        s3c24xx_ts1_set_platdata(&s3c_ts_platform);
#endif
#endif

#ifdef CONFIG_DEV_THERMAL
	s5p_tmu_set_platdata(NULL);
#endif

#if defined(CONFIG_TOUCHSCREEN_S5PV310) || defined(CONFIG_S3C_ADC)
	s3c_adc_set_platdata(&s3c_adc_platform);
#ifdef CONFIG_TOUCHSCREEN_S5PV310
	s3c_ts_set_platdata(&s3c_ts_platform);
#endif
#endif
#ifdef CONFIG_SND_SOC_MC1N2
	s3c_gpio_cfgpin(S5PV310_GPC1(3), S3C_GPIO_SFN(4));		/* I2C_6_SDA */
	s3c_gpio_cfgpin(S5PV310_GPC1(4), S3C_GPIO_SFN(4));		/* I2C_6_SCL */
#endif

#ifdef CONFIG_VIDEO_TVOUT
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);

#ifdef CONFIG_S5PV310_DEV_PD
	s5p_device_tvout.dev.parent = &s5pv310_device_pd[PD_TV].dev;
#endif
#endif

#ifdef CONFIG_S5PV310_DEV_PD
#ifdef CONFIG_FB_S3C
	s3c_device_fb.dev.parent = &s5pv310_device_pd[PD_LCD0].dev;
#endif
#endif

#ifdef CONFIG_S5PV310_DEV_PD
#ifdef CONFIG_VIDEO_FIMC
	s3c_device_fimc0.dev.parent = &s5pv310_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &s5pv310_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &s5pv310_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &s5pv310_device_pd[PD_CAM].dev;
#endif
#endif
#ifdef CONFIG_S5PV310_DEV_PD
#ifdef CONFIG_VIDEO_JPEG
	s5p_device_jpeg.dev.parent = &s5pv310_device_pd[PD_CAM].dev;
#endif
#endif
#ifdef CONFIG_S5PV310_DEV_PD
#ifdef CONFIG_SND_S3C64XX_SOC_I2S_V4
	/* s5pv310_device_iis0.dev.parent = &s5pv310_device_pd[PD_MAUDIO].dev; */
#endif
#ifdef CONFIG_SND_S3C_SOC_PCM
	/* s5pv310_device_pcm1.dev.parent = &s5pv310_device_pd[PD_MAUDIO].dev; */
#endif
#ifdef CONFIG_SND_SOC_SMDK_WM9713
	/* s5pv310_device_ac97.dev.parent = &s5pv310_device_pd[PD_MAUDIO].dev; */
#endif
#ifdef CONFIG_SND_SAMSUNG_SOC_SPDIF
	/* s5pv310_device_spdif.dev.parent = &s5pv310_device_pd[PD_MAUDIO].dev; */
#endif
#endif

	platform_add_devices(smdkc210_devices, ARRAY_SIZE(smdkc210_devices));

#if defined(CONFIG_S3C64XX_DEV_SPI0)
	sclk = clk_get(spi0_dev, "sclk_spi");
	if (IS_ERR(sclk))
		dev_err(spi0_dev, "failed to get sclk for SPI-0\n");
	prnt = clk_get(spi0_dev, "mout_mpll");
	if (IS_ERR(prnt))
		dev_err(spi0_dev, "failed to get prnt\n");
	clk_set_parent(sclk, prnt);
	clk_put(prnt);

	if (!gpio_request(S5PV310_GPB(1), "SPI_CS0")) {
		gpio_direction_output(S5PV310_GPB(1), 1);
		s3c_gpio_cfgpin(S5PV310_GPB(1), S3C_GPIO_SFN(2));
		s3c_gpio_setpull(S5PV310_GPB(1), S3C_GPIO_PULL_UP);
		s5pv310_spi_set_info(0, S5PV310_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi0_csi));
	}
	spi_register_board_info(spi0_board_info, ARRAY_SIZE(spi0_board_info));
#endif

#ifdef CONFIG_FB_S3C_AMS369FG06
	sclk = clk_get(spi_dev, "sclk_spi");
	if (IS_ERR(sclk))
		dev_err(spi_dev, "failed to get sclk for SPI-1\n");
	prnt = clk_get(spi_dev, "xusbxti");
	if (IS_ERR(prnt))
		dev_err(spi_dev, "failed to get prnt\n");
	clk_set_parent(sclk, prnt);
	clk_put(prnt);

	if (!gpio_request(S5PV310_GPB(5), "LCD_CS")) {
		gpio_direction_output(S5PV310_GPB(5), 1);
		s3c_gpio_cfgpin(S5PV310_GPB(5), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(S5PV310_GPB(5), S3C_GPIO_PULL_UP);
		s5pv310_spi_set_info(LCD_BUS_NUM, S5PV310_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi1_csi));
	}
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
	s3cfb_set_platdata(&ams369fg06_data);
#elif defined(CONFIG_FB_S3C_S6C1372) || defined(CONFIG_FB_S3C_S6F1202A)
	s6c1372_panel_gpio_init();
#elif defined(CONFIG_S3C64XX_DEV_SPI1)
	sclk = clk_get(spi1_dev, "sclk_spi");
	if (IS_ERR(sclk))
		dev_err(spi1_dev, "failed to get sclk for SPI-1\n");
	prnt = clk_get(spi1_dev, "mout_mpll");
	if (IS_ERR(prnt))
		dev_err(spi1_dev, "failed to get prnt\n");
	clk_set_parent(sclk, prnt);
	clk_put(prnt);

	if (!gpio_request(S5PV310_GPB(5), "SPI_CS1")) {
		gpio_direction_output(S5PV310_GPB(5), 1);
		s3c_gpio_cfgpin(S5PV310_GPB(5), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(S5PV310_GPB(5), S3C_GPIO_PULL_UP);
		s5pv310_spi_set_info(1, S5PV310_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi1_csi));
	}
	spi_register_board_info(spi1_board_info, ARRAY_SIZE(spi1_board_info));
#endif

#if defined(CONFIG_S3C64XX_DEV_SPI2)
	sclk = clk_get(spi2_dev, "sclk_spi");
	if (IS_ERR(sclk))
		dev_err(spi2_dev, "failed to get sclk for SPI-2\n");
	prnt = clk_get(spi2_dev, "mout_mpll");
	if (IS_ERR(prnt))
		dev_err(spi2_dev, "failed to get prnt\n");
	clk_set_parent(sclk, prnt);
	clk_put(prnt);

	if (!gpio_request(S5PV310_GPC1(2), "SPI_CS2")) {
		gpio_direction_output(S5PV310_GPC1(2), 1);
		s3c_gpio_cfgpin(S5PV310_GPC1(2), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(S5PV310_GPC1(2), S3C_GPIO_PULL_UP);
		s5pv310_spi_set_info(2, S5PV310_SPI_SRCCLK_SCLK,
			ARRAY_SIZE(spi2_csi));
	}
	spi_register_board_info(spi2_board_info, ARRAY_SIZE(spi2_board_info));
#endif

#ifdef CONFIG_BACKLIGHT_PWM
	smdk_backlight_register();
#endif

#ifdef CONFIG_FB_S3C_MDNIE
	mdnie_device_register();
#endif

#if defined(CONFIG_FB_S3C_S6F1202A) || defined(CONFIG_FB_S3C_S6C1372)
	lcd_fb_init();
#endif

#ifdef CONFIG_FB_S3C_MIPI_LCD
	mipi_fb_init();
#endif

#ifdef CONFIG_30PIN_CONN
	smdk_accessory_gpio_init();
#endif

	c1_sec_switch_init();
	c1_sound_init();
	cam_init();

	s3c_usb_set_serial();
#if defined(CONFIG_IRDA)
/*Ir-LED*/
	ir_rc_init_hw();
/*Ir-LED*/
#endif
	px_factory_init();
}

#if defined(CONFIG_S5P_MEM_CMA)
static void __init s5pv310_reserve(void)
{
	static struct cma_region regions[] = {
#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM
		{
			.name = "pmem",
			.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1
		{
			.name = "pmem_gpu1",
			.size = CONFIG_ANDROID_PMEM_MEMSIZE_PMEM_GPU1 * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC
		{
			.name = "mfc",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{
				.alignment = 1 << 17,
			},
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1
		{
			.name = "mfc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
			{
				.alignment = 1 << 17,
			},
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
	{
		.name = "fimc2",
		.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
		.start = 0
	},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TVOUT
		{
			.name = "tvout",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TVOUT * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0
		},
#endif

		{}
	};

	static const char map[] __initconst =
		"android_pmem.0=pmem;android_pmem.1=pmem_gpu1;s3cfb.0=fimd;"
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;"
		"s5p-tvout=tvout;s3c-mfc=mfc,mfc0,mfc1;s5p-rp=srp;"
		"s5p-jpeg=jpeg;";
	int i;
	unsigned int bank_end[2] = {
		(meminfo.bank[0].start + meminfo.bank[0].size),
		(meminfo.bank[1].start + meminfo.bank[1].size),
	};

	printk(KERN_ERR "mem info: bank0 start-> 0x%x, bank0 size-> 0x%x\
			\nbank1 start-> 0x%x, bank1 size-> 0x%x\n",
			(int)meminfo.bank[0].start, (int)meminfo.bank[0].size,
			(int)meminfo.bank[1].start, (int)meminfo.bank[1].size);

	for (i = ARRAY_SIZE(regions) - 1; i >= 0; i--) {
		struct cma_region *reg = &regions[i];
		int nr;

		if (!reg->name)
			continue;

		nr = reg->start;
		reg->start = bank_end[nr] - reg->size;

		if (reg->alignment)
			reg->start &= ~(reg->alignment - 1);

		bank_end[nr] = reg->start;

		printk(KERN_ERR "CMA reserve: %s, addr is 0x%x, size is 0x%x\n",
			reg->name, reg->start, reg->size);
	}
	printk(KERN_ERR "mem infor: bank0 start-> 0x%x, bank0 size-> 0x%x\
			\nbank1 start-> 0x%x, bank1 size-> 0x%x\n",
			(int)meminfo.bank[0].start, (int)meminfo.bank[0].size,
			(int)meminfo.bank[1].start, (int)meminfo.bank[1].size);

	cma_set_defaults(regions, map);
	cma_early_regions_reserve(NULL);
}
#endif

MACHINE_START(SMDKC210, "SMDKC210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= s5pv310_init_irq,
	.map_io		= smdkc210_map_io,
	.init_machine	= smdkc210_machine_init,
	.timer		= &s5pv310_timer,
MACHINE_END
