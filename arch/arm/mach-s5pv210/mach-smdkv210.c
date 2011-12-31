/* linux/arch/arm/mach-s5pv210/mach-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/pwm_backlight.h>
#include <linux/regulator/max8698.h>
#include <linux/dm9000.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/spi-clocks.h>
#include <mach/regs-mem.h>

#include <plat/s3c64xx-spi.h>
#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>
#include <plat/hwmon.h>
#include <plat/s5pv210.h>
#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/ata.h>
#include <plat/iic.h>
#include <plat/keypad.h>
#include <plat/fimg2d.h>
#include <plat/s5p-clock.h>
#include <plat/pm.h>
#include <plat/tvout.h>

#if CONFIG_DM9000
/* DM9000 registrations */
static struct resource s5p_dm9000_resources[] = {
	[0] = {
		.start = S5P_PA_DM9000,
		.end   = S5P_PA_DM9000,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
#if defined(CONFIG_DM9000_16BIT)
		.start = S5P_PA_DM9000 + 2,
		.end   = S5P_PA_DM9000 + 2,
		.flags = IORESOURCE_MEM,
#else
		.start = S5P_PA_DM9000 + 1,
		.end   = S5P_PA_DM9000 + 1,
		.flags = IORESOURCE_MEM,
#endif
	},
	[2] = {
		.start = IRQ_EINT9,
		.end   = IRQ_EINT9,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};

static struct dm9000_plat_data s5p_dm9000_platdata = {
#if defined(CONFIG_DM9000_16BIT)
	.flags = DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,
#else
	.flags = DM9000_PLATF_8BITONLY | DM9000_PLATF_NO_EEPROM,
#endif
};

struct platform_device s5p_device_dm9000 = {
	.name		= "dm9000",
	.id		=  0,
	.num_resources	= ARRAY_SIZE(s5p_dm9000_resources),
	.resource	= s5p_dm9000_resources,
	.dev		= {
		.platform_data = &s5p_dm9000_platdata,
	}
};
#endif

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

extern void s5p_reserve_bootmem(void);

static struct s3c2410_uartcfg smdkv210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
};

static struct s3c_ide_platdata smdkv210_ide_pdata __initdata = {
	.setup_gpio	= s5pv210_ide_setup_gpio,
};

static uint32_t smdkv210_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
};

static struct matrix_keymap_data smdkv210_keymap_data __initdata = {
	.keymap		= smdkv210_keymap,
	.keymap_size	= ARRAY_SIZE(smdkv210_keymap),
};

static struct samsung_keypad_platdata smdkv210_keypad_data __initdata = {
	.keymap_data	= &smdkv210_keymap_data,
	.rows		= 8,
	.cols		= 8,
};

#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
    .hw_ver = 30,
    .parent_clkname = "mout_mpll",
    .clkname = "sclk_fimg2d",
    .gate_clkname = "fimg2d",
    .clkrate = 250 * 1000000,
};
#endif

static int __init smdkc110_bl_init(struct device *dev)
{
    s3c_gpio_cfgpin(S5PV210_GPD0(3), S3C_GPIO_SFN(2));
    return 0;
}

static struct platform_pwm_backlight_data smdkc110_backlight_data = {
        .pwm_id         = 3,
        .max_brightness = 1000,
        .dft_brightness = 600,
        .pwm_period_ns  = 1000000000 / (1000 * 20),
        .init           = smdkc110_bl_init,
};

static struct platform_device smdkc110_backlight_device = {
        .name           = "pwm-backlight",
        .dev            = {
                .parent = &s3c_device_timer[3].dev,
                .platform_data = &smdkc110_backlight_data,
        },
};


#ifdef CONFIG_S3C_DEV_HWMON
static struct s3c_hwmon_pdata smdkc110_hwmon_pdata __initdata = {
	/* Reference voltage (1.2V) */
	.in[0] = &(struct s3c_hwmon_chcfg) {
		.name		= "smdk:reference-voltage",
		.mult		= 3300,
		.div		= 4096,
	},
};
#endif

static struct platform_device *smdkv210_devices[] __initdata = {
	&s5pv210_device_iis0,
	&s5pv210_device_ac97,
#ifdef CONFIG_S3C_ADC
	&s3c_device_adc,
#endif
	&s3c_device_cfcon,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,
	&samsung_device_keypad,
	&s3c_device_rtc,
#ifdef CONFIG_DM9000
	&s5p_device_dm9000,
#endif
#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
	&s3c_device_ts,
#endif
#ifdef CONFIG_S3C_DEV_ADC1
	&s3c_device_ts1,
#endif
#endif
#ifdef CONFIG_S3C_DEV_HWMON
	&s3c_device_hwmon,
#endif
	&s3c_device_wdt,
#ifdef CONFIG_HAVE_PWM
	&s3c_device_timer[3],
	&smdkc110_backlight_device,
#endif
#ifdef CONFIG_VIDEO_FIMG2D
    &s5p_device_fimg2d,
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI
	&s5pv210_device_spi0,
	&s5pv210_device_spi1,
#endif
#ifdef CONFIG_USB
	&s3c_device_usb_ehci,
	&s3c_device_usb_ohci,
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif

#if defined(CONFIG_VIDEO_TV20) || defined(CONFIG_VIDEO_TVOUT)
	&s5p_device_tvout,
	&s5p_device_cec,
	&s5p_device_hpd,
#endif

};

static struct regulator_consumer_supply buck1_consumers[] = {
	{
		.supply		= "vddarm",
	},
};

static struct regulator_init_data max8698_buck1_data = {
	.constraints	= {
		.name		= "VCC_ARM",
		.min_uV		=  750000,
		.max_uV		= 1500000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1250000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck1_consumers),
	.consumer_supplies	= buck1_consumers,
};

static struct regulator_consumer_supply buck2_consumers[] = {
	{
		.supply		= "vddint",
	},
};

static struct regulator_init_data max8698_buck2_data = {
	.constraints	= {
		.name		= "VCC_INTERNAL",
		.min_uV		= 950000,
		.max_uV		= 1200000,
		.always_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.uV		= 1100000,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= ARRAY_SIZE(buck2_consumers),
	.consumer_supplies	= buck2_consumers,
};

static struct regulator_init_data max8698_buck3_data = {
	.constraints	= {
		.name		= "VCC_MEM",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.state_mem	= {
			.uV		= 1800000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,
		},
	},
};

static struct regulator_init_data max8698_ldo2_data = {
	.constraints	= {
		.name		= "VALIVE_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.always_on	= 1,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_STANDBY,
			.enabled	= 1,
		},
	},
};

static struct regulator_init_data max8698_ldo3_data = {
	.constraints	= {
		.name		= "VUOTG_D_1.1V/VUHOST_D_1.1V",
		.min_uV		= 1100000,
		.max_uV		= 1100000,
		.apply_uV	= 1,
		.state_mem	={
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,	/* LDO3 should be OFF in sleep mode */
		},
	},
};

static struct regulator_init_data max8698_ldo4_data = {
	.constraints	= {
		.name		= "V_MIPI_1.8V",
		.min_uV		= 1800000,
		.max_uV		= 1800000,
		.apply_uV	= 1,
		.boot_on	= 1,
		.state_mem	={
			.uV		= 1800000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 0,	/* LDO4 should be OFF in sleep mode */

		},
	},
};

static struct regulator_init_data max8698_ldo5_data = {
	.constraints	= {
		.name		= "VMMC_2.8V/VEXT_2.8V",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.state_mem	={
			.uV		= 2800000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,
		},
	},
};

static struct regulator_init_data max8698_ldo6_data = {
	.constraints	= {
		.name		= "VCC_2.6V",
		.min_uV		= 2600000,
		.max_uV		= 2600000,
		.apply_uV	= 1,
		.state_mem	={
			.uV		= 2600000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,
		},
	},
};

static struct regulator_init_data max8698_ldo7_data = {
	.constraints	= {
		.name		= "VDAC_2.8V",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= 1,
		.state_mem	={
			.uV		= 2800000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,
		},
	},
};


static struct regulator_init_data max8698_ldo8_data = {
	.constraints	= {
		.name		= "VUOTG_A_3.3V/VUHOST_A_3.3V",
		.min_uV		= 3300000,
		.max_uV		= 3300000,
		.apply_uV	= 1,
		.state_mem	={
			.uV		= 0,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,	/* LDO8 should be OFF in sleep mode */
		},
	},
};

static struct regulator_init_data max8698_ldo9_data = {
	.constraints	= {
		.name		= "{VADC/VSYS/VKEY}_2.8V",
		.min_uV		= 3000000,
		.max_uV		= 3000000,
		.apply_uV	= 1,
		.state_mem	={
			.uV		= 3000000,
			.mode		= REGULATOR_MODE_NORMAL,
			.enabled	= 1,
		},
	},
};

static struct max8698_subdev_data smdkc110_regulators[] = {
	{ MAX8698_LDO2, &max8698_ldo2_data },
	{ MAX8698_LDO3, &max8698_ldo3_data },
	{ MAX8698_LDO4, &max8698_ldo4_data },
	{ MAX8698_LDO5, &max8698_ldo5_data },
	{ MAX8698_LDO6, &max8698_ldo6_data },
	{ MAX8698_LDO7, &max8698_ldo7_data },
	{ MAX8698_LDO8, &max8698_ldo8_data },
	{ MAX8698_LDO9, &max8698_ldo9_data },
	{ MAX8698_BUCK1, &max8698_buck1_data },
	{ MAX8698_BUCK2, &max8698_buck2_data },
	{ MAX8698_BUCK3, &max8698_buck3_data },
};

/* 800Mhz default voltage */
static struct max8698_platform_data max8698_platform_data_0 = {
	.num_regulators	= ARRAY_SIZE(smdkc110_regulators),
	.regulators	= smdkc110_regulators,

	.set1		= S5PV210_GPH1(6),
	.set2		= S5PV210_GPH1(7),
	.set3		= S5PV210_GPH0(4),

	.dvsarm1	= 0x8,	// 1.15v
	.dvsarm2	= 0x6,	// 1.05V
	.dvsarm3	= 0x5,	// 1.00V
	.dvsarm4	= 0x5,	// 1.00V

	.dvsint1	= 0x7,	// 1.10v
	.dvsint2	= 0x5,	// 1.00V
};

/* 1Ghz default voltage */
static struct max8698_platform_data max8698_platform_data_1 = {
	.num_regulators	= ARRAY_SIZE(smdkc110_regulators),
	.regulators	= smdkc110_regulators,

	.set1		= S5PV210_GPH1(6),
	.set2		= S5PV210_GPH1(7),
	.set3		= S5PV210_GPH0(4),

	.dvsarm1	= 0xa,	// 1.25v
	.dvsarm2	= 0x9,	// 1.20V
	.dvsarm3	= 0x6,	// 1.05V
	.dvsarm4	= 0x4,	// 0.95V

	.dvsint1	= 0x7,	// 1.10v
	.dvsint2	= 0x5,	// 1.00V
};

struct max8698_platform_data max8698_platform_default_data;

static struct i2c_board_info smdkv210_i2c_devs0[] __initdata = {
	{ I2C_BOARD_INFO("24c08", 0x50), },     /* Samsung S524AD0XD1 */
};

static struct i2c_board_info smdkv210_i2c_devs1[] __initdata = {
	/* To Be Updated */
#if defined(CONFIG_VIDEO_TV20) || defined(CONFIG_VIDEO_TVOUT)
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74>>1)),
	},
#endif
};

static struct i2c_board_info smdkv210_i2c_devs2[] __initdata = {
	/* To Be Updated */
	{
		/* The address is 0xCC used since SRAD = 0 */
		I2C_BOARD_INFO("max8698", (0xCC >> 1)),
		.platform_data = &max8698_platform_default_data,
	},
};

#ifdef CONFIG_S3C64XX_DEV_SPI

#define SMDK_MMCSPI_CS 0

static struct s3c64xx_spi_csinfo smdk_spi0_csi[] = {
	[SMDK_MMCSPI_CS] = {
		.line = S5PV210_GPB(1),
		.set_level = gpio_set_value,
		.fb_delay = 0x0,
	},
};

static struct s3c64xx_spi_csinfo smdk_spi1_csi[] = {
	[SMDK_MMCSPI_CS] = {
		.line = S5PV210_GPB(5),
		.set_level = gpio_set_value,
		.fb_delay = 0x0,
	},
};

static struct spi_board_info s3c_spi_devs[] __initdata = {
	{
		.modalias		= "spidev", /* spidev, mmc_spi */
		.mode			= SPI_MODE_0, /* CPOL=0, CPHA=0 */
		.max_speed_hz		= 10*1000*1000,
		.bus_num		= 0, /* Connected to SPI-0 as 1st Slave */
		.chip_select		= 0,
		.controller_data 	= &smdk_spi0_csi[SMDK_MMCSPI_CS],
	},
	{
		.modalias		= "spidev", /* spidev, mmc_spi */
		.mode			= SPI_MODE_0, /* CPOL=0, CPHA=0 */
		.max_speed_hz		= 10*1000*1000,
		.bus_num		= 1, /* Connected to SPI-0 as 1st Slave */
		.chip_select		= 0,
		.controller_data 	= &smdk_spi1_csi[SMDK_MMCSPI_CS],
	},
};
#endif

#ifdef CONFIG_DM9000
static void __init smdkv210_dm9000_set(void)
{
	unsigned int tmp;

	tmp = ((0<<28)|(0<<24)|(5<<16)|(0<<12)|(0<<8)|(0<<4)|(0<<0));
	__raw_writel(tmp, (S5P_SROM_BW+0x18));

	tmp = __raw_readl(S5P_SROM_BW);
	tmp &= ~(0xf << 20);

#ifdef CONFIG_DM9000_16BIT
	tmp |= (0x1 << 20);
#else
	tmp |= (0x2 << 20);
#endif
	__raw_writel(tmp, S5P_SROM_BW);

	/* Set SROM_CSn[5] */	
	tmp = ( __raw_readl(S5P_VA_GPIO+0x2E0) & ~(0xF<<20))|(0x2<<20); 
	/* Write MP0_1CON */	
	s3c_gpio_cfgpin(S5PV210_MP01(0),S3C_GPIO_SPECIAL(tmp));		
}
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C2410
static struct s3c2410_ts_mach_info s3c_ts_platform __initdata = {
	.delay			= 10000,
	.presc			= 49,
	.oversampling_shift	= 2,
};
#endif

#if defined(CONFIG_VIDEO_TV20) || defined(CONFIG_VIDEO_TVOUT)
static struct s5p_platform_hpd hdmi_hpd_data __initdata = {

};
static struct s5p_platform_cec hdmi_cec_data __initdata = {

};
#endif

static void __init smdkv210_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkv210_uartcfgs, ARRAY_SIZE(smdkv210_uartcfgs));

	clk_xusbxti.rate = 24000000;

}

static void __init smdkv210_machine_init(void)
{
#ifdef CONFIG_DM9000
	smdkv210_dm9000_set();
#endif

	s3c_pm_init();

	samsung_keypad_set_platdata(&smdkv210_keypad_data);

	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(0, smdkv210_i2c_devs0,
			ARRAY_SIZE(smdkv210_i2c_devs0));
	i2c_register_board_info(1, smdkv210_i2c_devs1,
			ARRAY_SIZE(smdkv210_i2c_devs1));
	i2c_register_board_info(2, smdkv210_i2c_devs2,
			ARRAY_SIZE(smdkv210_i2c_devs2));

	s3c_ide_set_platdata(&smdkv210_ide_pdata);
#ifdef CONFIG_VIDEO_FIMG2D
    s5p_fimg2d_set_platdata(&fimg2d_data);
#endif
#ifdef CONFIG_S3C64XX_DEV_SPI
	if (!gpio_request(S5PV210_GPB(1), "SPI_CS0")) {
		gpio_direction_output(S5PV210_GPB(1), 1);
		s3c_gpio_cfgpin(S5PV210_GPB(1), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(S5PV210_GPB(1), S3C_GPIO_PULL_UP);
		s5pv210_spi_set_info(0, S5PV210_SPI_SRCCLK_PCLK,
			ARRAY_SIZE(smdk_spi0_csi));
	}
	if (!gpio_request(S5PV210_GPB(5), "SPI_CS1")) {
		gpio_direction_output(S5PV210_GPB(5), 1);
		s3c_gpio_cfgpin(S5PV210_GPB(5), S3C_GPIO_SFN(1));
		s3c_gpio_setpull(S5PV210_GPB(5), S3C_GPIO_PULL_UP);
		s5pv210_spi_set_info(1, S5PV210_SPI_SRCCLK_PCLK,
			ARRAY_SIZE(smdk_spi1_csi));
	}
	spi_register_board_info(s3c_spi_devs, ARRAY_SIZE(s3c_spi_devs));
#endif
#ifdef CONFIG_TOUCHSCREEN_S3C2410
#ifdef CONFIG_S3C_DEV_ADC
        s3c24xx_ts_set_platdata(&s3c_ts_platform);
#endif
#ifdef CONFIG_S3C_DEV_ADC1
        s3c24xx_ts1_set_platdata(&s3c_ts_platform);
#endif
#endif
#ifdef CONFIG_S3C_DEV_HWMON
	s3c_hwmon_set_platdata(&smdkc110_hwmon_pdata);
#endif

#if defined(CONFIG_VIDEO_TV20) || defined(CONFIG_VIDEO_TVOUT)
	s5p_hdmi_hpd_set_platdata(&hdmi_hpd_data);
	s5p_hdmi_cec_set_platdata(&hdmi_cec_data);
#endif

	platform_add_devices(smdkv210_devices, ARRAY_SIZE(smdkv210_devices));
}

MACHINE_START(SMDKV210, "SMDKV210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= s5pv210_init_irq,
	.map_io		= smdkv210_map_io,
	.init_machine	= smdkv210_machine_init,
	.timer		= &s5pv210_timer,
	.reserve	= &s5p_reserve_bootmem,
MACHINE_END
