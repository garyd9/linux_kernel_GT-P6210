/* linux/arch/arm/mach-xxxx/board-tuna-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>

/* inlcude platform specific file */
#include <linux/platform_data/modem.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

#define DPRAM_SIZE 		0x4000
#define DPRAM_PHY_START 	0x13A00000
#define DPRAM_PHY_END (DPRAM_PHY_START + DPRAM_SIZE)

#define DP_MAGIC_CODE_SIZE 	2
#define DP_ACCESS_ENABLE 	2
#define DP_FMT_OUT_SIZE		2044
#define DP_RAW_OUT_SIZE		6128
#define DP_FMT_IN_SIZE		2044
#define DP_RAW_IN_SIZE		6128

#define DP_FMT_OUT_OFFSET 	(DP_MAGIC_CODE_SIZE + DP_ACCESS_ENABLE)
#define DP_RAW_OUT_OFFSET 	(DP_FMT_OUT_OFFSET + DP_FMT_OUT_SIZE)
#define DP_FMT_IN_OFFSET 	(DP_RAW_OUT_OFFSET + DP_RAW_OUT_SIZE)
#define DP_RAW_IN_OFFSET 	(DP_FMT_IN_OFFSET + DP_FMT_IN_SIZE)

#define DP_MAILBOX_OFFSET 	0x3FFC
#define MBXCP2AP_SIZE			0x2
#define MBXAP2CP_SIZE			0x2
#define MBXAP2CP_OFFSET		(DP_MAILBOX_OFFSET)
#define MBXCP2AP_OFFSET		(MBXAP2CP_OFFSET + MBXAP2CP_SIZE)

#define MAGIC_DMDL			0x4445444C

/* Link device extendions for DPRAM */
static struct modemlink_shared_channel dpram_map[] = {
	[0] = {
		.name = "fmt",
		.out_offset = DP_FMT_OUT_OFFSET,
		.out_size = DP_FMT_OUT_SIZE,
		.in_offset = DP_FMT_IN_OFFSET,
		.in_size = DP_FMT_OUT_SIZE,
	},
	[1] = {
		.name = "raw",
		.out_offset = DP_RAW_OUT_OFFSET,
		.out_size = DP_FMT_OUT_SIZE,
		.in_offset = DP_RAW_IN_OFFSET,
		.in_size = DP_FMT_OUT_SIZE,
	},
	[2] = {
		.name = "modem_update",
		.out_offset = DP_FMT_OUT_OFFSET,
		.out_size = (DP_MAILBOX_OFFSET - DP_FMT_OUT_OFFSET),
		.in_offset = DP_FMT_OUT_OFFSET,
		.in_size = 0,
	},
	/* we can add the multiple channels,
	   ex) rfs, or seperated raw channels*/
};

void s5pv310_dpram_int_clear(void);
static int p8_lte_ota_reset(void);
static struct modemlink_memmap dpram_memmap = {
	.magic_offset = 0,
	.magic_size = DP_MAGIC_CODE_SIZE,
	.access_offset = DP_MAGIC_CODE_SIZE,
	.access_size = DP_ACCESS_ENABLE,
	.in_box_offset = MBXCP2AP_OFFSET,
	.in_box_size = MBXCP2AP_SIZE,
	.out_box_offset = MBXAP2CP_OFFSET,
	.out_box_size = MBXAP2CP_SIZE,

	.num_shared_map = ARRAY_SIZE(dpram_map),
	.shared_map = dpram_map,
	.vendor_clear_irq = s5pv310_dpram_int_clear,
	.board_ota_reset = p8_lte_ota_reset,
};

/* umts target platform data */
static struct modem_io_t umts_io_devices[] = {
	[0] = {
		.name = "cdma_update",
		.id = 0x0,
		.format = IPC_UPDATE,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
};

static struct modem_data umts_modem_data = {
	.name = "cbp7.1",

	.gpio_cp_on = GPIO_PHONE_ON,
	.gpio_reset_req_n = 0,
	.gpio_cp_reset = GPIO_CP_RST,
	.gpio_pda_active = GPIO_PDA_ACTIVE,
	.gpio_phone_active = GPIO_PHONE_ACTIVE,
	.gpio_cp_dump_int = 0,
	.gpio_flm_uart_sel = 0,
	.gpio_cp_warm_reset = 0,
	.gpio_ap_wakeup = GPIO_CP_AP_DPRAM_INT,

	.modem_type = VIA_CBP71,
	.link_types = (1 << LINKDEV_DPRAM),
	.modem_net = CDMA_NETWORK,

	.num_iodevs = ARRAY_SIZE(umts_io_devices),
	.iodevs = umts_io_devices,

	/* linkdevice extension */
	.modemlink_extension = (void *)&dpram_memmap,
};

/* To get modem state, register phone active irq using resource */
static struct resource umts_modem_res[] = {
	[0] = {
		.name = "umts_phone_active",
		.start = IRQ_EINT14,	/* GPIO_PHONE_ACTIVE */
		.end = IRQ_EINT14,	/* GPIO_PHONE_ACTIVE */
		.flags = IORESOURCE_IRQ,
	},
	[1] = {
		.name = "host_wakeup",
		.start = IRQ_EINT(29),	/* GPIO_IPC_HOST_WAKEUP */
		.end = IRQ_EINT(29),		/* GPIO_IPC_HOST_WAKEUP */
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.name = "dpram_irq",
		.start = IRQ_MODEM_IF,
		.end = IRQ_MODEM_IF,
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.name = "dpram_map",
		.start = DPRAM_PHY_START,
		.end = DPRAM_PHY_END,
		.flags = IORESOURCE_MEM,
	},
};

/* if use more than one modem device, then set id num */
static struct platform_device umts_modem = {
	.name = "modem_if",
	.id = -1,
	.num_resources = ARRAY_SIZE(umts_modem_res),
	.resource = umts_modem_res,
	.dev = {
		.platform_data = &umts_modem_data,
	},
};

static void umts_modem_cfg_gpio(void)
{
	int err = 0;

	unsigned gpio_reset_req_n = umts_modem_data.gpio_reset_req_n;
	unsigned gpio_cp_on = umts_modem_data.gpio_cp_on;
	unsigned gpio_cp_rst = umts_modem_data.gpio_cp_reset;
	unsigned gpio_pda_active = umts_modem_data.gpio_pda_active;
	unsigned gpio_phone_active = umts_modem_data.gpio_phone_active;
	unsigned gpio_cp_dump_int = umts_modem_data.gpio_cp_dump_int;
	unsigned gpio_flm_uart_sel = umts_modem_data.gpio_flm_uart_sel;
	unsigned gpio_ap_wakeup = umts_modem_data.gpio_ap_wakeup;
	unsigned irq_phone_active = umts_modem_res[0].start;

	if (gpio_reset_req_n) {
		err = gpio_request(gpio_reset_req_n, "RESET_REQ_N");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "RESET_REQ_N", err);
		}
		gpio_direction_output(gpio_reset_req_n, 0);
	}

	if (gpio_cp_on) {
		err = gpio_request(gpio_cp_on, "CP_ON");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_ON", err);
		}
		gpio_direction_output(gpio_cp_on, 0);
	}

	if (gpio_cp_rst) {
		err = gpio_request(gpio_cp_rst, "CP_RST");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_RST", err);
		}
		gpio_direction_output(gpio_cp_rst, 0);
	}

	if (gpio_pda_active) {
		err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "PDA_ACTIVE", err);
		}
		gpio_direction_output(gpio_pda_active, 0);
	}

	if (gpio_phone_active) {
		err = gpio_request(gpio_phone_active, "PHONE_ACTIVE");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "PHONE_ACTIVE", err);
		}
		gpio_direction_input(gpio_phone_active);
		s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
		pr_err("check phone active = %d\n", irq_phone_active);
	}

	if (gpio_cp_dump_int) {
		err = gpio_request(gpio_cp_dump_int, "CP_DUMP_INT");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "CP_DUMP_INT", err);
		}
		gpio_direction_input(gpio_cp_dump_int);
	}

	if (gpio_ap_wakeup) {
		err = gpio_request(gpio_ap_wakeup, "AP_WAKE_UP");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "AP_WKAEUP_INT", err);
		}
		gpio_direction_input(gpio_ap_wakeup);
	}

	if (gpio_flm_uart_sel) {
		err = gpio_request(gpio_flm_uart_sel, "GPS_UART_SEL");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s : %d\n",
			       "GPS_UART_SEL", err);
		}
		gpio_direction_output(gpio_reset_req_n, 1);
	}

	if (gpio_phone_active) {
		//set_irq_type(irq_phone_active, IRQ_TYPE_LEVEL_HIGH);
		set_irq_type(gpio_to_irq(gpio_phone_active), IRQ_TYPE_LEVEL_HIGH);
	}

	printk(KERN_INFO "umts_modem_cfg_gpio done\n");
}

static int p8_lte_ota_reset(void)
{
	unsigned gpio_cp_rst = umts_modem_data.gpio_cp_reset;
	unsigned gpio_cp_on = umts_modem_data.gpio_cp_on;
	unsigned int *magickey_va;
	int i;

	pr_err("[MODEM_IF] %s Modem OTA reset\n", __func__);
	magickey_va = ioremap_nocache(DPRAM_PHY_START, sizeof(unsigned int));
	if (!magickey_va) {
		pr_err("%s: ioremap fail\n", __func__);
		return -ENOMEM;
	}

	gpio_set_value(gpio_cp_on, 1);
	msleep(100);
	gpio_set_value(gpio_cp_rst, 0);

	for (i=0; i < 3; i++) {
		*magickey_va = MAGIC_DMDL;
		if (*magickey_va == MAGIC_DMDL) {
			pr_err("magic key is ok!");
			break;
		}
	}

	msleep(500);
	gpio_set_value(gpio_cp_rst, 1);
	for (i=0; i < 3; i++) {
		*magickey_va = MAGIC_DMDL;
		if (*magickey_va == MAGIC_DMDL) {
			pr_err("magic key is ok!");
			break;
		}
	}

	iounmap(magickey_va);

	return 0;
}


extern void idpram_gpio_init(void);
extern int idpram_sfr_init(void);
static int __init init_modem(void)
{
	int ret;

	printk(KERN_INFO "[MODEM_IF] init_modem\n");

	/* interanl dpram gpio configure */
	idpram_gpio_init();
	idpram_sfr_init();

	/* umts gpios configuration */
	umts_modem_cfg_gpio();
	ret = platform_device_register(&umts_modem);
	if (ret < 0)
		return ret;

	return ret;
}
late_initcall(init_modem);
