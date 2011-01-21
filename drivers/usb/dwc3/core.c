/**
 * core.c - DesignWare USB3 DRD Controller Core file
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com
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
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#ifdef CONFIG_PM
static int dwc3_suspend(struct device *dev)
{
	pm_runtime_put_sync(dev);

	return 0;
}

static int dwc3_resume(struct device *dev)
{
	pm_runtime_get_sync(dev);

	return 0;
}

static const struct dev_pm_ops dwc3_pm_ops = {
	.suspend	= dwc3_suspend,
	.resume		= dwc3_resume,
};

#define DEV_PM_OPS	(&dwc3_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif

static irqreturn_t dwc3_gadget_interrupt(struct dwc3 *dwc, u32 event)
{
	switch (DWC3_DEVICE_EVENT(event)) {
	case DWC3_DEVICE_EVENT_DISCONNECT:
		/* handle disconnect IRQ here */
		break;
	case DWC3_DEVICE_EVENT_RESET:
		/* handle reset IRQ here */
		break;
	case DWC3_DEVICE_EVENT_CONNECT_DONE:
		/* handle connect done IRQ here */
		break;
	case DWC3_DEVICE_EVENT_WAKEUP:
		/* handle wakeup IRQ here */
		break;
	case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
		/* handle link status change IRQ here */
		break;
	case DWC3_DEVICE_EVENT_EOPF:
		/* handle end of periodic frame IRQ here */
		break;
	case DWC3_DEVICE_EVENT_SOF:
		/* handle start of frame IRQ here */
		break;
	case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
		/* handle erratic error IRQ here */
		break;
	case DWC3_DEVICE_EVENT_CMD_CMPL:
		/* handle command complete IRQ here */
		break;
	case DWC3_DEVICE_EVENT_OVERFLOW:
		/* handle device overflow IRQ here */
		break;
	default:
		dev_dbg(dwc->dev, "UNKNOWN IRQ %d\n", DWC3_DEVICE_EVENT(event));
	}

	return IRQ_NONE;
}

static irqreturn_t dwc3_otg_interrupt(struct dwc3 *dwc, u32 event)
{
	return IRQ_NONE;
}

static irqreturn_t dwc3_carkit_interrupt(struct dwc3 *dwc, u32 event)
{
	return IRQ_NONE;
}

static irqreturn_t dwc3_i2c_interrupt(struct dwc3 *dwc, u32 event)
{
	return IRQ_NONE;
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc)
{
	struct dwc3_event_buffer	*evt;
	struct dwc3			*dwc = _dwc;

	unsigned long			flags;

	int				count;

	irqreturn_t			ret = IRQ_NONE;

	spin_lock_irqsave(&dwc->lock, flags);

	count = dwc3_readl(dwc->dev, DWC3_GEVNTCOUNT);
	count &= DWC3_GEVNTCOUNT_MASK;
	if (!count)
		goto out;

	list_for_each_entry(evt, &dwc->event_buffer_list, list) {
		int			i;

		/*
		 * It's unclear if there's a possibility first of event
		 * buffer being NULL but still have valid event buffers
		 * after that.
		 */
		if (!evt->buf)
			break;

		for (i = 0; i < evt->length; i += 4) {
			u32		event;

			memcpy(&event, (evt->buf + i), sizeof(event));

			/*
			 * It's unclear if there's a possibility first of event
			 * being 0 and still have valid events after that.
			 */
			if (!event)
				break;

			switch (event & DWC3_EVENT_TYPE_MASK) {
			case DWC3_EVENT_TYPE_DEV:
				ret |= dwc3_gadget_interrupt(dwc, event);
				break;
			case DWC3_EVENT_TYPE_OTG:
				ret |= dwc3_otg_interrupt(dwc, event);
				break;
			case DWC3_EVENT_TYPE_CARKIT:
				ret |= dwc3_carkit_interrupt(dwc, event);
			case DWC3_EVENT_TYPE_I2C:
				ret |= dwc3_i2c_interrupt(dwc, event);
				break;
			default:
				dev_err(dwc->dev, "UNKNOWN IRQ type %d\n", event);
			}
		}
	}

out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->dev, evt->length, evt->buf, evt->dma);
	kfree(evt);
}

/**
 * dwc3_alloc_one_event_buffer - Allocated one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on succes
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *__devinit
dwc3_alloc_one_event_buffer(struct dwc3 *dwc, unsigned length)
{
	struct dwc3_event_buffer	*evt;

	evt = kzalloc(sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&evt->list);
	evt->dwc	= dwc;
	evt->length	= length;
	evt->buf	= dma_alloc_coherent(dwc->dev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf) {
		kfree(evt);
		return ERR_PTR(-ENOMEM);
	}

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	while (!list_empty(&dwc->event_buffer_list)) {
		evt = list_first_entry(&dwc->event_buffer_list,
				struct dwc3_event_buffer, list);
		list_del(&evt->list);
		dwc3_free_one_event_buffer(dwc, evt);
	}
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: Pointer to out controller context structure
 * @num: number of event buffers to allocate
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno.
 */
static int __devinit dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned num,
		unsigned length)
{
	int			i;

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		evt = dwc3_alloc_one_event_buffer(dwc, length);
		if (IS_ERR(evt)) {
			dev_err(dwc->dev, "can't allocate event buffer\n");
			return PTR_ERR(evt);
		}

		list_add_tail(&evt->list, &dwc->event_buffer_list);
	}

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: Pointer to out controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int __devinit dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n = 0;

	list_for_each_entry(evt, &dwc->event_buffer_list, list) {
		dev_dbg(dwc->dev, "Event buf %p dma %u length %d\n",
				evt->buf, evt->dma, evt->length);

		dwc3_writel(dwc->global, DWC3_GEVNTADR(n), evt->dma);
		dwc3_writel(dwc->global, DWC3_GEVNTADR(n + 1), 0);
		dwc3_writel(dwc->global, DWC3_GEVNTSIZ(n),
				evt->length & 0xffff);
		n += 2;
	}

	dwc3_writel(dwc->global, DWC3_GEVNTCOUNT, 0);

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n = 0;

	list_for_each_entry(evt, &dwc->event_buffer_list, list) {
		dwc3_writel(dwc->global, DWC3_GEVNTADR(n), 0);
		dwc3_writel(dwc->global, DWC3_GEVNTADR(n + 1), 0);
		dwc3_writel(dwc->global, DWC3_GEVNTSIZ(n), 0);

		n += 2;
	}

	dwc3_writel(dwc->global, DWC3_GEVNTCOUNT, 0);
}

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int __devinit dwc3_core_init(struct dwc3 *dwc)
{
	unsigned long		timeout = jiffies_to_msecs(jiffies + 500);
	u32			reg;

	int			ret;

	reg = dwc3_readl(dwc->global, DWC3_GSNPSID);
	if ((reg & 0xff00) != DWC3_GSNPSID_MASK) {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}

	dwc->revision = reg;

	dwc3_writel(dwc->device, DWC3_DCTL, DWC3_DCTL_CSFTRST);

	do {
		reg = dwc3_readl(dwc->device, DWC3_DCTL);

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto err0;
		}

		cpu_relax();
	} while (reg & DWC3_DCTL_CSFTRST);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_NUM,
			DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err1;
	}

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err1;
	}

	return 0;

err1:
	dwc3_free_event_buffers(dwc);

err0:
	return ret;
}

static int __devinit dwc3_probe(struct platform_device *pdev)
{
	struct resource		*res;
	struct dwc3		*dwc;
	void __iomem		*base;
	int			ret = -ENOMEM;
	int			irq;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get(&pdev->dev);
	pm_runtime_forbid(&pdev->dev);

	dwc = kzalloc(sizeof(*dwc), GFP_KERNEL);
	if (!dwc) {
		dev_err(&pdev->dev, "not enough memory\n");
		goto err0;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "xhci");
	if (!res) {
		dev_err(&pdev->dev, "missing 'xhci' resource\n");
		goto err1;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err1;
	}

	dwc->xhci	= base;

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

	dwc->global	= base;

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

	dwc->device	= base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "otg");
	if (!res) {
		dev_err(&pdev->dev, "missing 'otg' resource\n");
		goto err4;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err4;
	}

	dwc->otg	= base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ram0");
	if (!res) {
		dev_err(&pdev->dev, "missing 'ram0' resource\n");
		goto err5;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err5;
	}

	dwc->ram0	= base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ram1");
	if (!res) {
		dev_err(&pdev->dev, "missing 'ram1' resource\n");
		goto err6;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err6;
	}

	dwc->ram1	= base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ram2");
	if (!base) {
		dev_err(&pdev->dev, "missing 'ram2' resource\n");
		goto err7;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err7;
	}

	dwc->ram2	= base;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "missing IRQ\n");
		goto err8;
	}

	INIT_LIST_HEAD(&dwc->event_buffer_list);
	spin_lock_init(&dwc->lock);
	platform_set_drvdata(pdev, dwc);

	dwc->dev	= &pdev->dev;
	dwc->irq	= irq;

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize core\n");
		goto err8;
	}

	ret = dwc3_gadget_init(dwc);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialized gadget\n");
		goto err8;
	}

	ret = request_irq(irq, dwc3_interrupt, 0, "dwc3", dwc);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		goto err8;
	}

	pm_runtime_allow(&pdev->dev);

	return 0;

err8:
	iounmap(dwc->ram2);

err7:
	iounmap(dwc->ram1);

err6:
	iounmap(dwc->ram0);

err5:
	iounmap(dwc->otg);

err4:
	iounmap(dwc->device);

err3:
	iounmap(dwc->global);

err2:
	iounmap(dwc->xhci);

err1:
	kfree(dwc);

err0:
	return ret;
}

static int __devexit dwc3_remove(struct platform_device *pdev)
{
	struct dwc3	*dwc = platform_get_drvdata(pdev);

	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	free_irq(dwc->irq, dwc);
	dwc3_event_buffers_cleanup(dwc);
	dwc3_free_event_buffers(dwc);

	iounmap(dwc->ram2);
	iounmap(dwc->ram1);
	iounmap(dwc->ram0);
	iounmap(dwc->otg);
	iounmap(dwc->device);
	iounmap(dwc->global);
	iounmap(dwc->xhci);

	kfree(dwc);

	return 0;
}

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= __devexit_p(dwc3_remove),
	.driver		= {
		.name	= "dwc3",
		.pm	= DEV_PM_OPS,
	},
};

MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");

static int __init dwc3_init(void)
{
	return platform_driver_register(&dwc3_driver);
}
module_init(dwc3_init);

static void __exit dwc3_exit(void)
{
	platform_driver_unregister(&dwc3_driver);
}
module_exit(dwc3_exit);
