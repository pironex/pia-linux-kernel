/**
 * dwc3-haps.c - HAPS Specific glue layer
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
#include <linux/pci.h>
#include <linux/platform_device.h>

/* FIXME define these in <linux/pci_ids.h> */
#define PCI_VENDOR_ID_SYNOPSYS		0x16c3
#define PCI_DEVICE_ID_SYNOPSYS_HAPSUSB3	0xabcd

struct dwc3_haps {
	/* device lock */
	spinlock_t		lock;
	struct device		*dev;
	struct platform_device	*dwc3;
};

static int __devinit dwc3_haps_probe(struct pci_dev *pci,
		const struct pci_device_id *id)
{
	struct platform_device	*dwc3;
	struct dwc3_haps	*haps;
	int			ret = -ENOMEM;

	haps = kzalloc(sizeof(*haps), GFP_KERNEL);
	if (!haps) {
		dev_err(&pci->dev, "not enough memory\n");
		goto err0;
	}

	ret = pci_enable_device(pci);
	if (ret) {
		dev_err(&pci->dev, "failed to enable pci device\n");
		goto err1;
	}

	pci_set_power_state(pci, PCI_D0);
	pci_set_master(pci);

	dwc3 = platform_device_alloc("dwc3", -1);
	if (!dwc3) {
		dev_err(&pci->dev, "couldn't allocate dwc3 device\n");
		goto err2;
	}

	ret = platform_device_add_resources(dwc3, pci->resource,
			PCI_NUM_RESOURCES);
	if (ret) {
		dev_err(&pci->dev, "couldn't add resources to dwc3 device\n");
		goto err3;
	}

	spin_lock_init(&haps->lock);
	pci_set_drvdata(pci, haps);

	dwc3->dev.parent = &pci->dev;
	haps->dev	= &pci->dev;
	haps->dwc3	= dwc3;

	ret = platform_device_register(dwc3);
	if (ret) {
		dev_err(&pci->dev, "failed to register dwc3 device\n");
		goto err3;
	}

	return 0;

err3:
	pci_set_drvdata(pci, NULL);
	platform_device_put(dwc3);

err2:
	pci_disable_device(pci);

err1:
	kfree(haps);

err0:
	return ret;
}

static void __devexit dwc3_haps_remove(struct pci_dev *pci)
{
	struct dwc3_haps	*haps = pci_get_drvdata(pci);

	platform_device_unregister(haps->dwc3);
	pci_set_drvdata(pci, NULL);
	pci_disable_device(pci);
	kfree(haps);
}

static DEFINE_PCI_DEVICE_TABLE(dwc3_haps_id_table) = {
	{
		PCI_DEVICE(PCI_VENDOR_ID_SYNOPSYS,
				PCI_DEVICE_ID_SYNOPSYS_HAPSUSB3),
	},
	{  }	/* Terminating Entry */
};
MODULE_DEVICE_TABLE(pci, dwc3_haps_id_table);

static struct pci_driver dwc3_haps_driver = {
	.name		= "dwc3-haps",
	.id_table	= dwc3_haps_id_table,
	.probe		= dwc3_haps_probe,
	.remove		= __devexit_p(dwc3_haps_remove),
	.driver		= {
		.name	= "dwc3-haps",
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 HAPS Glue Layer");

static int __init dwc3_haps_init(void)
{
	return pci_register_driver(&dwc3_haps_driver);
}
module_init(dwc3_haps_init);

static void __exit dwc3_haps_exit(void)
{
	pci_unregister_driver(&dwc3_haps_driver);
}
module_exit(dwc3_haps_exit);
