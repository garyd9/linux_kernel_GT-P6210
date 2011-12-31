/*
 * Copyright (C) 2011 Samsung Electronics Co. Ltd.
 *  Hyuk Kang <hyuk78.kang@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/host_notify.h>


struct host_notifier_platform_data host_notifier_pdata = {
	.ndev.name	= "usb_otg",
	.gpio		= HOST_NOTIFIER_GPIO,
	.booster	= HOST_NOTIFIER_BOOSTER,
};

struct platform_device host_notifier_device = {
	.name = "host_notifier",
	.dev.platform_data = &host_notifier_pdata,
};
