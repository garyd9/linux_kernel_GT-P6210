#include <linux/gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <mach/regs-gpio.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include <linux/modemctl.h>

static struct modemctl_platform_data mdmctl_data;
void modemctl_cfg_gpio(void)
{
	int err = 0;

	unsigned gpio_phone_on = mdmctl_data.gpio_phone_on;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_cp_req_reset = mdmctl_data.gpio_cp_req_reset;
	unsigned gpio_ipc_slave_wakeup = mdmctl_data.gpio_ipc_slave_wakeup;
	unsigned gpio_ipc_host_wakeup = mdmctl_data.gpio_ipc_host_wakeup;
	unsigned gpio_suspend_request = mdmctl_data.gpio_suspend_request;
	unsigned gpio_active_state = mdmctl_data.gpio_active_state;
	unsigned gpio_cp_dump_int = mdmctl_data.gpio_cp_dump_int;
	unsigned gpio_sim_detect = mdmctl_data.gpio_sim_detect;

	/*TODO: check uart init func AP FLM BOOT RX -- */
	s3c_gpio_setpull(S5PV310_GPA1(4), S3C_GPIO_PULL_UP);

	err = gpio_request(gpio_phone_on, "PHONE_ON");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PHONE_ON");
	} else {
		gpio_direction_output(gpio_phone_on, 0);
		s3c_gpio_setpull(gpio_phone_on, S3C_GPIO_PULL_NONE);
	}
	err = gpio_request(gpio_cp_rst, "CP_RST");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_RST");
	} else {
		gpio_direction_output(gpio_cp_rst, 0);
		s3c_gpio_setpull(gpio_cp_rst, S3C_GPIO_PULL_NONE);
	}
	err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PDA_ACTIVE");
	} else {
		gpio_direction_output(gpio_pda_active, 0);
		s3c_gpio_setpull(gpio_pda_active, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_req_reset, "CP_REQ_RESET");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_REQ_RESET");
	} else {
		gpio_direction_output(gpio_cp_req_reset, 0);
		s3c_gpio_setpull(gpio_cp_req_reset, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_ipc_slave_wakeup, "IPC_SLAVE_WAKEUP");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n",
			"IPC_SLAVE_WAKEUP");
	} else {
		gpio_direction_output(gpio_ipc_slave_wakeup, 0);
		s3c_gpio_setpull(gpio_ipc_slave_wakeup, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_ipc_host_wakeup, "IPC_HOST_WAKEUP");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "IPC_HOST_WAKEUP");
	} else {
		gpio_direction_output(gpio_ipc_host_wakeup, 0);
		s3c_gpio_cfgpin(gpio_ipc_host_wakeup, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_ipc_host_wakeup, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_suspend_request, "SUSPEND_REQUEST");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "SUSPEND_REQUEST");
	} else {
		gpio_direction_input(gpio_suspend_request);
		s3c_gpio_setpull(gpio_suspend_request, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_active_state, "ACTIVE_STATE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "ACTIVE_STATE");
	} else {
		gpio_direction_output(gpio_active_state, 0);
		s3c_gpio_setpull(gpio_active_state, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_phone_active, "PHONE_ACTIVE");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "PHONE_ACTIVE");
	} else {
		gpio_direction_output(gpio_phone_active, 0);
		s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_phone_active, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_dump_int, "CP_DUMP_INT");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "CP_DUMP_INT");
	} else {
		gpio_direction_output(gpio_cp_dump_int, 0);
		s3c_gpio_cfgpin(gpio_cp_dump_int, S3C_GPIO_SFN(0xF));
		s3c_gpio_setpull(gpio_cp_dump_int, S3C_GPIO_PULL_DOWN);
	}
#ifdef CONFIG_SUPPORT_SIMDETECT
	if (gpio_sim_detect) {
		err = gpio_request(gpio_sim_detect, "SIM_DETECT");
		if (err) {
			printk(KERN_ERR "fail to request gpio %s\n",
				"SIM_DETECT");
		} else {
			gpio_direction_output(gpio_sim_detect, 0);
			s3c_gpio_cfgpin(gpio_sim_detect, S3C_GPIO_SFN(0xF));
			s3c_gpio_setpull(gpio_sim_detect, S3C_GPIO_PULL_NONE);
		}
	}
#endif
	err = gpio_request(GPIO_IPC_TXD, "IPC_TXD");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "IPC_TXD");
	} else {
		gpio_direction_output(GPIO_IPC_TXD, 0);
		s3c_gpio_setpull(GPIO_IPC_TXD, S3C_GPIO_PULL_NONE);
	}
	err = gpio_request(GPIO_IPC_RXD, "IPC_RXD");
	if (err) {
		printk(KERN_ERR "fail to request gpio %s\n", "IPC_RXD");
	} else {
		gpio_direction_output(GPIO_IPC_RXD, 0);
		s3c_gpio_setpull(GPIO_IPC_RXD, S3C_GPIO_PULL_NONE);
	}
}

static void xmm6260_vcc_init(struct modemctl *mc)
{
	int err;

	if (!mc->vcc) {
		mc->vcc = regulator_get(NULL, "vhsic");
		if (IS_ERR(mc->vcc)) {
			err = PTR_ERR(mc->vcc);
			dev_dbg(mc->dev, "No VHSIC_1.2V regualtor: %d\n", err);
			mc->vcc = NULL;
		}
	}

	if (mc->vcc)
		regulator_enable(mc->vcc);
}

static void xmm6260_vcc_off(struct modemctl *mc)
{
	if (mc->vcc)
		regulator_disable(mc->vcc);
}

static void xmm6260_reverse_bias_gpio_clear(struct modemctl *mc)
{
	gpio_direction_output(mc->gpio_cp_req_reset, 0);
	gpio_direction_output(mc->gpio_ipc_slave_wakeup, 0);
	gpio_direction_output(mc->gpio_ipc_host_wakeup, 0);
	gpio_direction_output(mc->gpio_suspend_request, 0);
	gpio_direction_output(mc->gpio_cp_dump_int, 0);

	msleep(20);
}

static void xmm6260_reverse_bias_gpio_restore(struct modemctl *mc)
{
	gpio_direction_output(mc->gpio_ipc_host_wakeup, 0);
	s3c_gpio_cfgpin(mc->gpio_ipc_host_wakeup, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(mc->gpio_ipc_host_wakeup, S3C_GPIO_PULL_NONE);
	set_irq_type(gpio_to_irq(mc->gpio_ipc_host_wakeup),
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	enable_irq_wake(gpio_to_irq(mc->gpio_ipc_host_wakeup));

	gpio_direction_output(mc->gpio_cp_dump_int, 0);
	s3c_gpio_cfgpin(mc->gpio_cp_dump_int, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(mc->gpio_cp_dump_int, S3C_GPIO_PULL_DOWN);
	set_irq_type(gpio_to_irq(mc->gpio_cp_dump_int),
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	enable_irq_wake(gpio_to_irq(mc->gpio_cp_dump_int));

	gpio_direction_input(mc->gpio_suspend_request);
}

static void xmm6260_on(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	printk(KERN_ERR "%s\n", __func__);
	if (!mc->gpio_cp_reset || !mc->gpio_phone_on || !mc->gpio_cp_req_reset)
		return;

	xmm6260_reverse_bias_gpio_clear(mc);
	xmm6260_vcc_init(mc);

	gpio_set_value(mc->gpio_phone_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	udelay(160);

	gpio_set_value(mc->gpio_pda_active, 0);
	gpio_set_value(mc->gpio_active_state, 0);
	msleep(500); /* must be >500ms for CP can boot up under -20 degrees */

	gpio_set_value(mc->gpio_cp_reset, 1);
	udelay(160);
	gpio_set_value(mc->gpio_cp_req_reset, 1);
	udelay(160);

	gpio_set_value(mc->gpio_phone_on, 1);

	msleep(20);
	xmm6260_reverse_bias_gpio_restore(mc);
	gpio_set_value(mc->gpio_active_state, 1);
	gpio_set_value(mc->gpio_pda_active, 1);
}

static void xmm6260_off(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if (!mc->gpio_cp_reset || !mc->gpio_phone_on)
		return;

	gpio_set_value(mc->gpio_phone_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);

	xmm6260_vcc_off(mc);
}

static void xmm6260_reset(struct modemctl *mc)
{
	dev_dbg(mc->dev, "%s\n", __func__);
	if (!mc->gpio_cp_reset || !mc->gpio_cp_req_reset)
		return;

/*	gpio_set_value(mc->gpio_pda_active, 0);
	gpio_set_value(mc->gpio_active_state, 0);
	gpio_set_value(mc->gpio_cp_reset, 0); */
	gpio_set_value(mc->gpio_cp_req_reset, 0);

	msleep(100);

	gpio_set_value(mc->gpio_cp_reset, 1);
	udelay(160);
	gpio_set_value(mc->gpio_cp_req_reset, 1);
}

/* move the PDA_ACTIVE Pin control to sleep_gpio_table */
static void xmm6260_suspend(struct modemctl *mc)
{
	xmm6260_vcc_off(mc);
}

static void xmm6260_resume(struct modemctl *mc)
{
	xmm6260_vcc_init(mc);
}

#ifndef CONFIG_SUPPORT_SIMDETECT
#define GPIO_SIM_DETECT 0
#endif
static struct modemctl_platform_data mdmctl_data = {
	.name = "xmm6260",
	.gpio_phone_on = GPIO_PHONE_ON,
	.gpio_phone_active = GPIO_PHONE_ACTIVE,
	.gpio_pda_active = GPIO_PDA_ACTIVE,
	.gpio_cp_reset = GPIO_CP_RST,
	.gpio_cp_req_reset = GPIO_CP_REQ_RESET,
	.gpio_ipc_slave_wakeup = GPIO_IPC_SLAVE_WAKEUP,
	.gpio_ipc_host_wakeup = GPIO_IPC_HOST_WAKEUP,
	.gpio_suspend_request = GPIO_SUSPEND_REQUEST,
	.gpio_active_state = GPIO_ACTIVE_STATE,
	.gpio_cp_dump_int = GPIO_CP_DUMP_INT,
	.gpio_sim_detect = GPIO_SIM_DETECT,
	.ops = {
		.modem_on = xmm6260_on,
		.modem_off = xmm6260_off,
		.modem_reset = xmm6260_reset,
		.modem_suspend = xmm6260_suspend,
		.modem_resume = xmm6260_resume,
		.modem_cfg_gpio = modemctl_cfg_gpio,
	}
};

/* TODO: check the IRQs..... */
static struct resource mdmctl_res[] = {
	[0] = {
		.start = IRQ_PHONE_ACTIVE,
		.end = IRQ_PHONE_ACTIVE,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device modemctl = {
	.name = "modemctl",
	.id = -1,
	.num_resources = ARRAY_SIZE(mdmctl_res),
	.resource = mdmctl_res,

	.dev = {
		.platform_data = &mdmctl_data,
	},
};
