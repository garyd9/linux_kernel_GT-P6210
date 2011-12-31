#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <mach/irqs.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/regulator/consumer.h>
#include <linux/lte_modemctl.h>
#include <plat/gpio-cfg.h>
#include <linux/irq.h>

static struct modemctl_platform_data mdmctl_data;

static void lte_on(struct modemctl *mc)
{
	gpio_set_value(mc->gpio_phone_on, 1);
	msleep(300);

	gpio_set_value(mc->gpio_cp_reset, 1);
	gpio_set_value(mc->gpio_phone_off, 1);
	msleep(300);

	return;
}

static void lte_off(struct modemctl *mc)
{
	gpio_set_value(mc->gpio_phone_on, 0);
	gpio_set_value(mc->gpio_cp_reset, 0);
	msleep(300);

	gpio_set_value(mc->gpio_phone_off, 0);
	msleep(300);

	return;
}

static void lte_reset(struct modemctl *mc)
{
	gpio_set_value(mc->gpio_cp_reset, 0);
	msleep(100);
	gpio_set_value(mc->gpio_cp_reset, 1);
	msleep(100);

	return;
}

static void lte_suspend(struct modemctl *mc)
{
	return;
}

static void lte_resume(struct modemctl *mc)
{
	return;
}

static void lte_boot(struct modemctl *mc)
{
	return;
}

static void modemctl_cfg_gpio(void)
{
	int err;
	int phone_active_irq;
	int host_wakeup_irq;

	unsigned gpio_phone_on = mdmctl_data.gpio_phone_on;
	unsigned gpio_phone_off = mdmctl_data.gpio_phone_off;
	unsigned gpio_cp_reset = mdmctl_data.gpio_cp_reset;

	unsigned gpio_slave_wakeup = mdmctl_data.gpio_slave_wakeup;
	unsigned gpio_host_wakeup = mdmctl_data.gpio_host_wakeup;
	unsigned gpio_host_active = mdmctl_data.gpio_host_active;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;

	/* unsigned gpio_pda_active = mdmctl_data.gpio_pda_active; */

	err = gpio_request(gpio_phone_on, "220_PMIC_PWRON");
	if (err)
		pr_err("fail to request gpio %s\n", "220_PMIC_PWRON");
	else {
		gpio_direction_output(gpio_phone_on, 0);
		s3c_gpio_setpull(gpio_phone_on, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_phone_off, "LTE_PS_HOLD_OFF");
	if (err)
		pr_err("fail to request gpio %s\n", "LTE_PS_HOLD_OFF");
	else {
		gpio_direction_output(gpio_phone_off, 0);
		s3c_gpio_setpull(gpio_phone_off, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_cp_reset, "CMC_RST");
	if (err)
		pr_err("fail to request gpio %s\n", "CMC_RST");
	else {
		gpio_direction_output(gpio_cp_reset, 0);
		s3c_gpio_setpull(gpio_cp_reset, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_slave_wakeup, "AP2LTE_WAKEUP");
	if (err)
		pr_err("fail to request gpio %s\n", "AP2LTE_WALEUP");
	else {
		gpio_direction_output(gpio_slave_wakeup, 0);
		s3c_gpio_setpull(gpio_slave_wakeup, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_host_wakeup, "LTE2AP_WAKEUP");
	if (err)
		pr_err("fail to request gpio %s\n", "LTE2AP_WAKEUP");
	else {
		host_wakeup_irq = gpio_to_irq(gpio_host_wakeup);
		gpio_direction_input(gpio_host_wakeup);
		s3c_gpio_setpull(gpio_host_wakeup, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(gpio_host_wakeup, S3C_GPIO_SFN(0xF));
		set_irq_type(host_wakeup_irq, IRQ_TYPE_EDGE_BOTH);
	}

	err = gpio_request(gpio_host_active, "AP2LTE_STATUS");
	if (err)
		pr_err("fail to request gpio %s\n", "AP2LTE_STATUS");
	else {
		gpio_direction_output(gpio_host_active, 0);
		s3c_gpio_setpull(gpio_host_active, S3C_GPIO_PULL_NONE);
	}

	err = gpio_request(gpio_phone_active, "LTE_ACTIVE");
	if (err)
		pr_err("fail to request gpio %s\n", "LTE_ACTIVE");
	else {
		phone_active_irq = gpio_to_irq(gpio_phone_active);
		gpio_direction_input(gpio_phone_active);
		s3c_gpio_setpull(gpio_phone_active, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(gpio_phone_active, S3C_GPIO_SFN(0xF));
		set_irq_type(phone_active_irq, IRQ_TYPE_EDGE_BOTH);
	}
}

static struct modemctl_platform_data mdmctl_data = {
	.name = "lte",
	.gpio_phone_on = GPIO_220_PMIC_PWRON,
	.gpio_phone_off = GPIO_LTE_PS_HOLD_OFF,
	.gpio_cp_reset = GPIO_CMC_RST,

	.gpio_slave_wakeup = GPIO_AP2LTE_WAKEUP,
	.gpio_host_wakeup = GPIO_LTE2AP_WAKEUP,
	.gpio_host_active = GPIO_AP2LTE_STATUS,
	.gpio_phone_active = GPIO_LTE_ACTIVE,
	/* .gpio_pda_active = GPIO_PDA_ACTIVE, */

	.ops = {
		.modem_on = lte_on,
		.modem_off = lte_off,
		.modem_reset = lte_reset,
		.modem_boot = lte_boot,
		.modem_suspend = lte_suspend,
		.modem_resume = lte_resume,
		.modem_cfg_gpio = modemctl_cfg_gpio,
	}
};

static struct resource mdmctl_res[] = {
	[0] = {
		.start = IRQ_LTE2AP_WAKEUP,
		.end = IRQ_LTE2AP_WAKEUP,
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
