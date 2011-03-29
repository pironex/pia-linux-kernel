/**
 * dwc3-omap.c - OMAP Specific Glue layer
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>

#include "platform_data.h"

struct dwc3_omap {
	/* device lock */
	spinlock_t		lock;
	struct device		*dev;
	struct platform_device	*dwc3;

	/*
	 * hold memory base pointers here
	 * so we can iounmap on exit
	 */
	void __iomem		*global;
	void __iomem		*device;
};

#ifdef CONFIG_PM
static int dwc3_omap_suspend(struct device *dev)
{
	struct dwc3_omap	*omap = dev_get_drvdata(dev);

	pm_runtime_suspend(omap->dev);

	return 0;
}

static int dwc3_omap_resume(struct device *dev)
{
	struct dwc3_omap	*omap = dev_get_drvdata(dev);

	pm_runtime_resume(omap->dev);

	return 0;
}

static const struct dev_pm_ops dwc3_omap_pm_ops = {
	.suspend	= dwc3_omap_suspend,
	.resume		= dwc3_omap_resume,
};

#define DEV_PM_OPS	(&dwc3_omap_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static int __devinit dwc3_omap_probe(struct platform_device *pdev)
{
	struct dwc3_core_platform_data	pdata;
	struct platform_device		*dwc3;
	struct dwc3_omap		*omap;
	struct resource			*res;

	unsigned int			irq;
	int				ret = -ENOMEM;

	void __iomem			*base;

	memset(&pdata, 0x00, sizeof(pdata));

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	omap = kzalloc(sizeof(*omap), GFP_KERNEL);
	if (!omap) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto err0;
	}

	dwc3 = platform_device_alloc("dwc3", -1);
	if (!dwc3) {
		dev_err(&pdev->dev, "couldn't allocate dwc3 device\n");
		goto err1;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "global");
	if (!res) {
		dev_err(&pdev->dev, "missing 'global' resource\n");
		goto err2;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err2;
	}

	pdata.global = base;
	omap->global = base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "device");
	if (!res) {
		dev_err(&pdev->dev, "missing 'device' resource\n");
		goto err3;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err3;
	}

	pdata.device = base;
	omap->device = base;

	irq = platform_get_irq(pdev, 0);
	if (irq == 0) {
		dev_err(&pdev->dev, "missing IRQ\n");
		goto err4;
	}

	pdata.irq = irq;

	ret = platform_device_add_data(dwc3, &pdata, sizeof(pdata));
	if (ret) {
		dev_err(&pdev->dev, "couldn't add platform_data to dwc3 device\n");
		goto err4;
	}

	spin_lock_init(&omap->lock);
	platform_set_drvdata(pdev, omap);

	dwc3->dev.parent = &pdev->dev;
	omap->dev	= &pdev->dev;
	omap->dwc3	= dwc3;

	ret = platform_device_register(dwc3);
	if (ret) {
		dev_err(&pdev->dev, "failed to register dwc3 device\n");
		goto err2;
	}

	pm_runtime_allow(&pdev->dev);

	return 0;

err4:
	iounmap(omap->device);

err3:
	iounmap(omap->global);

err2:
	platform_device_put(dwc3);

err1:
	kfree(omap);

err0:
	return ret;
}

static int __devexit dwc3_omap_remove(struct platform_device *pdev)
{
	struct dwc3_omap	*omap = platform_get_drvdata(pdev);

	platform_device_unregister(omap->dwc3);

	iounmap(omap->device);
	iounmap(omap->global);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	kfree(omap);

	return 0;
}

static struct platform_driver dwc3_omap_driver = {
	.probe		= dwc3_omap_probe,
	.remove		= __devexit_p(dwc3_omap_remove),
	.driver		= {
		.name	= "dwc3-omap",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("DesignWare USB3 OMAP Glue Layer");

static int __init dwc3_omap_init(void)
{
	return platform_driver_register(&dwc3_omap_driver);
}
module_init(dwc3_omap_init);

static void __exit dwc3_omap_exit(void)
{
	platform_driver_unregister(&dwc3_omap_driver);
}
module_exit(dwc3_omap_exit);
