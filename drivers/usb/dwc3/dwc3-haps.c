/**
 * dwc3-haps.c - HAPS Specific glue layer
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Author: Felipe Balbi <balbi@ti.com>
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
MODULE_LICENSE("Dual BSD/GPL");
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
