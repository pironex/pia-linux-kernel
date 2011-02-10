/**
 * dwc3-omap.c - OMAP Specific Glue layer
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

struct dwc3_omap {
	/* device lock */
	spinlock_t		lock;
	struct device		*dev;
	struct platform_device	*dwc3;
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
	struct platform_device	*dwc3;
	struct dwc3_omap	*omap;
	int			ret = -ENOMEM;

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

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	kfree(omap);

	return 0;
}

static struct platform_driver dwc3_omap_driver = {
	.probe		= dwc3_omap_probe,
	.remove		= __exit_p(dwc3_omap_remove),
	.driver		= {
		.name	= "dwc3-omap",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
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
