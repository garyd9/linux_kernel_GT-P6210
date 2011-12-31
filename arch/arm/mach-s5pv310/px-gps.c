#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <plat/gpio-cfg.h>

extern struct class *sec_class;
static struct device *gps_dev = NULL;

static int __init gps_gpio_init(void)
{
#if defined (CONFIG_MACH_P2_REV02)
	int gpio_gps_nrst = system_rev >= 5 ?
		GPIO_GPS_nRST_28V : GPIO_GPS_nRST;
#else
	int gpio_gps_nrst = GPIO_GPS_nRST;
#endif
	gpio_request(gpio_gps_nrst, "GPIO_GPS_nRST");
	gpio_request(GPIO_GPS_PWR_EN, "GPIO_GPS_PWR_EN");

	gpio_direction_output(gpio_gps_nrst, 1);
	gpio_direction_output(GPIO_GPS_PWR_EN, 0);

	gpio_export(gpio_gps_nrst, 1);
	gpio_export(GPIO_GPS_PWR_EN, 1);

	BUG_ON(!sec_class);
	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");

	BUG_ON(!gps_dev);
	gpio_export_link(gps_dev, "GPS_nRST", gpio_gps_nrst);
	gpio_export_link(gps_dev, "GPS_PWR_EN", GPIO_GPS_PWR_EN);

	return 0;
}

arch_initcall(gps_gpio_init);
