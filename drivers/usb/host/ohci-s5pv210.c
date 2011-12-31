/* ohci-s5pv210.c - Driver for USB HOST on Samsung S5PV210 processor
 *
 * Bus Glue for SAMSUNG S5PV210 USB HOST OHCI Controller
 *
 * (C) Copyright 1999 Roman Weissgaerber <weissg@vienna.at>
 * (C) Copyright 2000-2002 David Brownell <dbrownell@users.sourceforge.net>
 * (C) Copyright 2002 Hewlett-Packard Company
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * Based on "ohci-au1xxx.c" by Matt Porter <mporter@kernel.crashing.org>
 * Modified for SAMSUNG s5pv210 OHCI by Jingoo Han <jg1.han@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

extern struct platform_device s3c_device_usb_ohci;
extern int usb_disabled(void);

extern void usb_host_phy_init(void);
extern void usb_host_phy_off(void);

static void s5pv210_start_ohc(void);
static void s5pv210_stop_ohc(void);
static int ohci_hcd_s5pv210_drv_probe(struct platform_device *pdev);
static int ohci_hcd_s5pv210_drv_remove(struct platform_device *pdev);

#ifdef CONFIG_PM
static int ohci_hcd_s5pv210_drv_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	unsigned long flags;
	int rc = 0;

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ohci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	s5pv210_stop_ohc();
bail:
	spin_unlock_irqrestore(&ohci->lock, flags);

	return rc;
}
static int ohci_hcd_s5pv210_drv_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	int rc = 0;

	s5pv210_start_ohc();

	pm_runtime_resume(&pdev->dev);

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	ohci_finish_controller_resume(hcd);

	return rc;
}
#else
#define ohci_hcd_s5pv210_drv_suspend NULL
#define ohci_hcd_s5pv210_drv_resume NULL
#endif


static void s5pv210_start_ohc(void)
{
#if defined(CONFIG_SAMSUNG_PHONE_SVNET) || defined(CONFIG_SAMSUNG_LTE) || defined(CONFIG_UMTS_LINK_HSIC)
	/* only ehci controls phy on/off */
#else
	usb_host_phy_init();
#endif
}

static void s5pv210_stop_ohc(void)
{
#if defined(CONFIG_SAMSUNG_PHONE_SVNET) || defined(CONFIG_SAMSUNG_LTE) || defined(CONFIG_UMTS_LINK_HSIC)
	/* only ehci controls phy on/off */
#else
	usb_host_phy_off();
#endif
}

static int __devinit ohci_s5pv210_start(struct usb_hcd *hcd)
{
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	int ret;

	ohci_dbg(ohci, "ohci_s5pv210_start, ohci:%p", ohci);

	ret = ohci_init(ohci);
	if (ret < 0)
		return ret;

	ret = ohci_run(ohci);
	if (ret < 0) {
		err("can't start %s", hcd->self.bus_name);
		ohci_stop(hcd);
		return ret;
	}

	return 0;
}

static const struct hc_driver ohci_s5pv210_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "s5pv210 OHCI",
	.hcd_priv_size		= sizeof(struct ohci_hcd),

	.irq			= ohci_irq,
	.flags			= HCD_MEMORY|HCD_USB11,

	.start			= ohci_s5pv210_start,
	.stop			= ohci_stop,
	.shutdown		= ohci_shutdown,

	.get_frame_number	= ohci_get_frame,

	.urb_enqueue		= ohci_urb_enqueue,
	.urb_dequeue		= ohci_urb_dequeue,
	.endpoint_disable	= ohci_endpoint_disable,

	.hub_status_data	= ohci_hub_status_data,
	.hub_control		= ohci_hub_control,
#ifdef	CONFIG_PM
	.bus_suspend		= ohci_bus_suspend,
	.bus_resume		= ohci_bus_resume,
#endif
	.start_port_reset	= ohci_start_port_reset,
};

static int ohci_hcd_s5pv210_drv_probe(struct platform_device *pdev)
{
	struct usb_hcd  *hcd = NULL;
	int retval = 0;

	if (usb_disabled())
		return -ENODEV;

	if (pdev->resource[1].flags != IORESOURCE_IRQ) {
		dev_err(&pdev->dev, "resource[1] is not IORESOURCE_IRQ.\n");
		return -ENODEV;
	}

	hcd = usb_create_hcd(&ohci_s5pv210_hc_driver, &pdev->dev, "s5pv210");
	if (!hcd) {
		dev_err(&pdev->dev, "usb_create_hcd failed!\n");
		return -ENODEV;
	}

	hcd->rsrc_start = pdev->resource[0].start;
	hcd->rsrc_len = pdev->resource[0].end - pdev->resource[0].start + 1;

	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len, hcd_name)) {
		dev_err(&pdev->dev, "request_mem_region failed!\n");
		retval = -EBUSY;
		goto err1;
	}

	s5pv210_start_ohc();
	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "ioremap failed!\n");
		retval = -ENOMEM;
		goto err2;
	}

	ohci_hcd_init(hcd_to_ohci(hcd));

	retval = usb_add_hcd(hcd, pdev->resource[1].start,
				IRQF_DISABLED | IRQF_SHARED);

	if (retval == 0) {
		platform_set_drvdata(pdev, hcd);
#ifdef CONFIG_USB_SUSPEND
		pm_runtime_set_active(&pdev->dev);
		pm_runtime_enable(&pdev->dev);
#endif
		return retval;
	}

	s5pv210_stop_ohc();
	iounmap(hcd->regs);
err2:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
	usb_put_hcd(hcd);
	return retval;
}

static int ohci_hcd_s5pv210_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);
	s5pv210_stop_ohc();
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_USB_SUSPEND
static int ohci_hcd_s5pv210_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ohci_hcd	*ohci = hcd_to_ohci(hcd);
	unsigned long flags;
	int rc = 0;

	/* Root hub was already suspended. Disable irq emission and
	 * mark HW unaccessible, bail out if RH has been resumed. Use
	 * the spinlock to properly synchronize with possible pending
	 * RH suspend or resume activity.
	 *
	 * This is still racy as hcd->state is manipulated outside of
	 * any locks =P But that will be a different fix.
	 */
	spin_lock_irqsave(&ohci->lock, flags);
	if (hcd->state != HC_STATE_SUSPENDED) {
		rc = -EINVAL;
		goto bail;
	}

	ohci_writel(ohci, OHCI_INTR_MIE, &ohci->regs->intrdisable);
	(void)ohci_readl(ohci, &ohci->regs->intrdisable);

	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

#if defined(CONFIG_SAMSUNG_PHONE_SVNET) || defined(CONFIG_SAMSUNG_LTE) || defined(CONFIG_UMTS_LINK_HSIC)
#else
	printk(KERN_DEBUG "USB_PM ohci rt susp\n");
	usb_host_phy_off();
#endif


bail:
	spin_unlock_irqrestore(&ohci->lock, flags);

	return rc;
}

static int ohci_hcd_s5pv210_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	int rc = 0;

	if (dev->power.status == DPM_RESUMING)
		return 0;

#if defined(CONFIG_SAMSUNG_PHONE_SVNET) || defined(CONFIG_SAMSUNG_LTE) || defined(CONFIG_UMTS_LINK_HSIC)
#else
	printk("USB_PM ohci rt resume\n");
	usb_host_phy_init();
#endif

	/* Mark hardware accessible again as we are out of D3 state by now */
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	ohci_finish_controller_resume(hcd);

	return rc;
}
#endif

static const struct dev_pm_ops ohci_s5pv210_pm_ops = {
	.suspend	= ohci_hcd_s5pv210_drv_suspend,
	.resume		= ohci_hcd_s5pv210_drv_resume,
#ifdef CONFIG_USB_SUSPEND
	.runtime_suspend = ohci_hcd_s5pv210_runtime_suspend,
	.runtime_resume = ohci_hcd_s5pv210_runtime_resume,
#endif
};

static struct platform_driver  ohci_hcd_s5pv210_driver = {
	.probe		= ohci_hcd_s5pv210_drv_probe,
	.remove		= ohci_hcd_s5pv210_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver = {
		.name = "s5p-ohci",
		.owner = THIS_MODULE,
		.pm	= &ohci_s5pv210_pm_ops,
	}
};

MODULE_ALIAS("platform:s5p-ohci");
