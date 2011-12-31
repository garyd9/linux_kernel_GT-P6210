/* linux/drivers/video/s5p_mipi_dsi.c
 *
 * Samsung SoC MIPI-DSIM driver.
 *
 * Copyright (c) 2011 Samsung Electronics
 *
 * InKi Dae, <inki.dae@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/memory.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/pm_runtime.h>

#include <plat/fb.h>
#include <plat/regs-dsim.h>
#include <plat/dsim.h>

#include <mach/map.h>

#include "s5p_mipi_dsi_common.h"
#include "s5p_mipi_dsi_lowlevel.h"
#include "s5p_mipi_dsi_lcd.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define master_to_driver(a)	(a->dsim_ddi->dsim_lcd_drv)
#define master_to_device(a)	(a->dsim_ddi->dsim_lcd_dev)
#define set_master_to_device(a)	(a->dsim_ddi->dsim_lcd_dev->master = a)

struct mipi_dsim_ddi {
	struct list_head		list;
	struct mipi_dsim_lcd_driver	*dsim_lcd_drv;
	struct mipi_dsim_lcd_device	*dsim_lcd_dev;
};

struct mipi_dsim_device *dsim[MIPI_DSIM_NUM];

static LIST_HEAD(dsim_ddi_list);
static DEFINE_MUTEX(mipi_lock);

static irqreturn_t s5p_mipi_dsi_interrupt_handler(int irq, void *dev_id)
{
	unsigned int int_src;
	struct mipi_dsim_device *mipi_dev = dev_id;

	s5p_mipi_dsi_set_interrupt_mask(dsim[mipi_dev->id], 0xffffffff, 1);

	int_src = readl(mipi_dev->reg_base + S5P_DSIM_INTSRC);
	s5p_mipi_dsi_clear_interrupt(dsim[mipi_dev->id], int_src);

	if (!(int_src && INTSRC_PLL_STABLE))
		printk(KERN_ERR "mipi dsi interrupt source (%x).\n", int_src);

	s5p_mipi_dsi_set_interrupt_mask(dsim[mipi_dev->id], 0xffffffff, 0);
	return IRQ_HANDLED;
}

int s5p_mipi_dsi_register_lcd_driver(struct mipi_dsim_lcd_driver *lcd_drv)
{
	struct mipi_dsim_ddi *dsim_ddi;
	struct mipi_dsim_lcd_device *dsim_lcd_dev;
	static unsigned int id;
	int ret;

	dsim_ddi = kzalloc(sizeof(struct mipi_dsim_ddi), GFP_KERNEL);
	if (!dsim_ddi) {
		printk(KERN_ERR "failed to allocate dsim_ddi object.\n");
		return -EFAULT;
	}

	dsim_ddi->dsim_lcd_drv = lcd_drv;

	dsim_lcd_dev = kzalloc(sizeof(struct mipi_dsim_lcd_device), GFP_KERNEL);
	if (!dsim_lcd_dev) {
		printk(KERN_ERR "failed to allocate dsim_lcd_dev object.\n");
		ret = -EFAULT;
		goto err_dsim;
	}

	mutex_lock(&mipi_lock);

	dsim_lcd_dev->id = id++;
	dsim_ddi->dsim_lcd_dev = dsim_lcd_dev;

	device_initialize(&dsim_lcd_dev->dev);

	strcpy(dsim_lcd_dev->modalias, lcd_drv->name);
	dev_set_name(&dsim_lcd_dev->dev, "mipi-dsim.%d\n", dsim_lcd_dev->id);

	ret = device_add(&dsim_lcd_dev->dev);
	if (ret < 0) {
		printk(KERN_ERR "can't %s %s, status %d\n",
				"add", dev_name(&dsim_lcd_dev->dev), ret);
		id--;
		goto err_device_add;
	}

	list_add_tail(&dsim_ddi->list, &dsim_ddi_list);

	mutex_unlock(&mipi_lock);

	printk(KERN_DEBUG "registered panel driver(%s) to mipi-dsi driver.\n",
		lcd_drv->name);

	return ret;

err_device_add:
	kfree(dsim_lcd_dev);

err_dsim:
	kfree(dsim_ddi);

	return ret;
}

static struct mipi_dsim_ddi *find_mipi_client_registered
		(struct mipi_dsim_device *dsim, const char *name)
{
	struct mipi_dsim_ddi *dsim_ddi;
	struct mipi_dsim_lcd_driver *dsim_lcd_drv = NULL;

	mutex_lock(&mipi_lock);

	dev_dbg(dsim->dev, "find lcd panel driver(%s).\n",
		name);
	list_for_each_entry(dsim_ddi, &dsim_ddi_list, list) {
		dsim_lcd_drv = dsim_ddi->dsim_lcd_drv;
		if ((strcmp(dsim_lcd_drv->name, name)) == 0) {
			mutex_unlock(&mipi_lock);
			dev_dbg(dsim->dev, "found!!!(%s).\n",
				dsim_lcd_drv->name);
			return dsim_ddi;
		}
	}

	dev_warn(dsim->dev, "failed to find lcd panel driver(%s).\n",
		name);

	mutex_unlock(&mipi_lock);

	return NULL;
}

#ifdef CONFIG_PM
#ifdef CONFIG_HAS_EARLYSUSPEND
static void s5p_mipi_dsi_early_suspend(struct early_suspend *handler)
{
	dsim[0]->resume_complete = 0;

	if (master_to_driver(dsim[0]) && (master_to_driver(dsim[0]))->suspend)
		(master_to_driver(dsim[0]))->suspend(master_to_device(dsim[0]));

	clk_disable(dsim[0]->clock);
	pm_runtime_put_sync(dsim[0]->dev);
}

static void s5p_mipi_dsi_late_resume(struct early_suspend *handler)
{
	pm_runtime_get_sync(dsim[0]->dev);
	clk_enable(dsim[0]->clock);

	s5p_mipi_dsi_init_dsim(dsim[0]);
	s5p_mipi_dsi_init_link(dsim[0]);

	s5p_mipi_dsi_set_hs_enable(dsim[0]);
	/* set cpu command transfer mode to hs. */
	s5p_mipi_dsi_set_data_transfer_mode(dsim[0], 0);

	/* it needs delay for stabilization */
	mdelay(dsim[0]->pd->delay_for_stabilization);

	if (master_to_driver(dsim[0]) && (master_to_driver(dsim[0]))->resume)
		(master_to_driver(dsim[0]))->resume(master_to_device(dsim[0]));

	s5p_mipi_dsi_set_display_mode(dsim[0],
		dsim[0]->dsim_config);

	/* set lcdc data transfer mode to hs. */
	s5p_mipi_dsi_set_data_transfer_mode(dsim[0], 1);

	dsim[0]->resume_complete = 1;
}
#else
static int s5p_mipi_dsi_suspend(struct platform_device *pdev,
		pm_message_t state)
{

	dsim[pdev->id]->resume_complete = 0;

	if (master_to_driver(dsim[pdev->id]) && (master_to_driver(dsim[pdev->id]))->suspend)
		(master_to_driver(dsim[pdev->id]))->suspend(master_to_device(dsim[pdev->id]));

	clk_disable(dsim[pdev->id]->clock);
	pm_runtime_put_sync(dsim[pdev->id]->dev);
	return 0;
}

static int s5p_mipi_dsi_resume(struct platform_device *pdev)
{
	pm_runtime_get_sync(dsim[pdev->id]->dev);
	clk_enable(dsim[pdev->id]->clock);

	s5p_mipi_dsi_init_dsim(dsim[pdev->id]);
	s5p_mipi_dsi_init_link(dsim[pdev->id]);

	s5p_mipi_dsi_set_hs_enable(dsim[pdev->id]);
	/* set cpu command transfer mode to hs. */
	s5p_mipi_dsi_set_data_transfer_mode(dsim[pdev->id], 0);

	/* it needs delay for stabilization */
	mdelay(dsim[pdev->id]->pd->delay_for_stabilization);

	if (master_to_driver(dsim[pdev->id]) && (master_to_driver(dsim[pdev->id]))->resume)
		(master_to_driver(dsim[pdev->id]))->resume(master_to_device(dsim[pdev->id]));

	s5p_mipi_dsi_set_display_mode(dsim[pdev->id],
		dsim[pdev->id]->dsim_config);

	/* set lcdc data transfer mode to hs. */
	s5p_mipi_dsi_set_data_transfer_mode(dsim[pdev->id], 1);

	dsim[pdev->id]->resume_complete = 1;

	return 0;
}
#endif
#else
#define s5p_mipi_dsi_suspend NULL
#define s5p_mipi_dsi_resume NULL
#endif


static int s5p_mipi_dsi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mipi_dsim_config *dsim_config;
	struct s5p_platform_mipi_dsim *dsim_pd;
	int ret = -1;

	if (!dsim[pdev->id])
		dsim[pdev->id] = kzalloc(sizeof(struct mipi_dsim_device),
			GFP_KERNEL);
	if (!dsim[pdev->id]) {
		dev_err(&pdev->dev, "failed to allocate dsim object.\n");
		return -EFAULT;
	}

	dsim[pdev->id]->pd = to_dsim_plat(&pdev->dev);
	dsim[pdev->id]->dev = &pdev->dev;
	dsim[pdev->id]->resume_complete = 0;
	dsim[pdev->id]->id = pdev->id;

	/* get s5p_platform_mipi_dsim. */
	dsim_pd = (struct s5p_platform_mipi_dsim *)dsim[pdev->id]->pd;
	/* get mipi_dsim_config. */
	dsim_config = dsim_pd->dsim_config;
	dsim[pdev->id]->dsim_config = dsim_config;

	if (pdev->id == 0)
		dsim[pdev->id]->clock = clk_get(&pdev->dev, "sclk_dsim0");
	else
		dsim[pdev->id]->clock = clk_get(&pdev->dev, "sclk_dsim1");

	if (IS_ERR(dsim[pdev->id]->clock)) {
		dev_err(&pdev->dev, "failed to get dsim clock source\n");
		goto err_clock_get;
	}

	clk_enable(dsim[pdev->id]->clock);

	pm_runtime_enable(dsim[pdev->id]->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get io memory region\n");
		ret = -EINVAL;
		goto err_platform_get;
	}
	res = request_mem_region(res->start, resource_size(res),
					dev_name(&pdev->dev));
	if (!res) {
		dev_err(&pdev->dev, "failed to request io memory region\n");
		ret = -EINVAL;
		goto err_mem_region;
	}

	dsim[pdev->id]->res = res;
	dsim[pdev->id]->reg_base = ioremap(res->start, resource_size(res));
	if (!dsim[pdev->id]->reg_base) {
		dev_err(&pdev->dev, "failed to remap io region\n");
		ret = -EINVAL;
		goto err_mem_region;
	}

	pm_runtime_get_sync(dsim[pdev->id]->dev);
	/*
	 * it uses frame done interrupt handler
	 * only in case of MIPI Video mode.
	 */
	if (dsim[pdev->id]->pd->dsim_lcd_config->e_interface == DSIM_VIDEO) {
		dsim[pdev->id]->irq = platform_get_irq(pdev, 0);
		if (request_irq(dsim[pdev->id]->irq, s5p_mipi_dsi_interrupt_handler,
				IRQF_DISABLED, "mipi-dsi", dsim[pdev->id])) {
			dev_err(&pdev->dev, "request_irq failed.\n");
			goto err_irq;
		}
	}
	/* find lcd panel driver registered to mipi-dsi driver. */
	if (pdev->id == 0)
		dsim[pdev->id]->dsim_ddi = find_mipi_client_registered(dsim[pdev->id],
				"lcd0");
	else
		dsim[pdev->id]->dsim_ddi = find_mipi_client_registered(dsim[pdev->id],
				"lcd1");

	if (dsim[pdev->id]->dsim_config == NULL) {
		dev_err(&pdev->dev, "dsim_config is NULL.\n");
		goto err_dsim_config;
	}
	/* set dsim to master of mipi_dsim_lcd_device. */
	set_master_to_device(dsim[pdev->id]);
	s5p_mipi_dsi_init_dsim(dsim[pdev->id]);
	s5p_mipi_dsi_init_link(dsim[pdev->id]);
	s5p_mipi_dsi_set_hs_enable(dsim[pdev->id]);
	/* set cpu command transfer mode to hs. */
	s5p_mipi_dsi_set_data_transfer_mode(dsim[pdev->id], 0);

	/* initialize mipi-dsi client(lcd panel). */
	if (master_to_driver(dsim[pdev->id]) && (master_to_driver(dsim[pdev->id]))->probe)
		(master_to_driver(dsim[pdev->id]))->probe(master_to_device(dsim[pdev->id]));

	/* it needs delay for stabilization */
	mdelay(dsim[pdev->id]->pd->delay_for_stabilization);

	/* lcd init */
	if (master_to_driver(dsim[pdev->id]) && (master_to_driver(dsim[pdev->id]))->displayon)
		(master_to_driver(dsim[pdev->id]))->displayon(master_to_device(dsim[pdev->id]));

	s5p_mipi_dsi_set_display_mode(dsim[pdev->id], dsim[pdev->id]->dsim_config);
	/* set lcdc data transfer mode to hs. */
	s5p_mipi_dsi_set_data_transfer_mode(dsim[pdev->id], 1);
	dev_info(&pdev->dev, "mipi-dsi driver(%s mode) has been probed.\n",
		(dsim_config->e_interface == DSIM_COMMAND) ?
			"CPU" : "RGB");

#ifdef CONFIG_HAS_EARLYSUSPEND
	dsim[pdev->id]->early_suspend.suspend = s5p_mipi_dsi_early_suspend;
	dsim[pdev->id]->early_suspend.resume = s5p_mipi_dsi_late_resume;
	dsim[pdev->id]->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&(dsim[pdev->id]->early_suspend));
#endif

	pm_runtime_put_sync(dsim[pdev->id]->dev);
	return 0;

err_dsim_config:
err_irq:
	release_resource(dsim[pdev->id]->res);
	kfree(dsim[pdev->id]->res);

	iounmap((void __iomem *) dsim[pdev->id]->reg_base);

err_mem_region:
err_platform_get:
	clk_disable(dsim[pdev->id]->clock);
	clk_put(dsim[pdev->id]->clock);

err_clock_get:
	kfree(dsim[pdev->id]);
	pm_runtime_put_sync(dsim[pdev->id]->dev);
	return ret;

}

static int __devexit s5p_mipi_dsi_remove(struct platform_device *pdev)
{

	struct mipi_dsim_ddi *dsim_ddi = NULL;

	if (dsim[pdev->id]->dsim_config->e_interface == DSIM_VIDEO)
		free_irq(dsim[pdev->id]->irq, dsim[pdev->id]);

	iounmap(dsim[pdev->id]->reg_base);

	clk_disable(dsim[pdev->id]->clock);
	clk_put(dsim[pdev->id]->clock);

	release_resource(dsim[pdev->id]->res);
	kfree(dsim[pdev->id]->res);

	list_for_each_entry(dsim_ddi, &dsim_ddi_list, list);
	kfree(dsim_ddi);

	kfree(dsim[pdev->id]);

	return 0;
}

static struct platform_driver s5p_mipi_dsi_driver = {
	.probe = s5p_mipi_dsi_probe,
	.remove = __devexit_p(s5p_mipi_dsi_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = s5p_mipi_dsi_suspend,
	.resume = s5p_mipi_dsi_resume,
#endif
	.driver = {
		   .name = "s5p-mipi-dsim",
		   .owner = THIS_MODULE,
	},
};

static int s5p_mipi_dsi_register(void)
{
	platform_driver_register(&s5p_mipi_dsi_driver);

	return 0;
}

static void s5p_mipi_dsi_unregister(void)
{
	platform_driver_unregister(&s5p_mipi_dsi_driver);
}
module_init(s5p_mipi_dsi_register);
module_exit(s5p_mipi_dsi_unregister);

MODULE_AUTHOR("InKi Dae <inki.dae@samsung.com>");
MODULE_DESCRIPTION("Samusung MIPI-DSI driver");
MODULE_LICENSE("GPL");
