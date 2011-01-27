/**
 * gadget.c - DesignWare USB3 DRD Controller Gadget Framework Link
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

#define to_dwc3_ep(ep)		(container_of(ep, struct dwc3_ep, endpoint))
#define gadget_to_dwc(g)	(container_of(g, struct dwc, gadget))

static void dwc_map_buffer_to_dma(struct dwc3_request *req)
{
}

static void dwc_unmap_buffer_from_dma(struct dwc3_request *req)
{
}

static void dwc_gadget_giveback(struct dwc3_ep *dep, struct dwc3_request *req,
		int status)
{
	struct dwc3			*dwc = dep->dwc;

	list_del(&req->list);
	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	dwc_unmap_buffer_from_dma(req);

	spin_unlock(&dwc->lock);
	req->request.complete(&req->dep->endpoint, &req->request);
	spin_lock(&dwc->lock);

	dev_dbg(dwc->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);
}

static int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params)
{
	unsigned long		timeout = jiffies + msecs_to_jiffies(500);
	u32			reg;

	params->param1.depcfg.ep_number = ep;

	dwc3_writel(dwc->device, DWC3_DEPCMDPAR0(ep), params->param0.raw);
	dwc3_writel(dwc->device, DWC3_DEPCMDPAR1(ep), params->param1.raw);
	dwc3_writel(dwc->device, DWC3_DEPCMDPAR2(ep), params->param2.raw);

	dwc3_writel(dwc->device, DWC3_DEPCMD(ep), cmd | DWC3_DEPCMD_CMDACT);
	do {
		reg = dwc3_readl(dwc->device, DWC3_DEPCMD(0));
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (!(reg & DWC3_DEPCMD_CMDACT));

	return 0;
}

static int dwc_init_endpoint(struct dwc3_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	/*
	 * REVISIT here I should be sending the correct commands
	 * to initialize the HW endpoint.
	 */

	return 0;
}

static int dwc_disable_endpoint(struct dwc3_ep *ep)
{
	/*
	 * REVISIT here I should be sending correct commands
	 * to disable the HW endpoint.
	 */

	return 0;
}

/* -------------------------------------------------------------------------- */

static int dwc_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int dwc_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}

static struct usb_request *dwc_gadget_ep0_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	return NULL;
}

static void dwc_gadget_ep0_free_request(struct usb_ep *ep,
		struct usb_request *req)
{
}

static int dwc_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *req,
	gfp_t gfp_flags)
{
	return 0;
}

static int dwc_gadget_ep0_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	return 0;
}

static int dwc_gadget_ep0_set_halt(struct usb_ep *ep, int value)
{
	return 0;
}

static int dwc_gadget_ep0_set_wedge(struct usb_ep *ep)
{
	return 0;
}

static int dwc_gadget_ep0_fifo_status(struct usb_ep *ep)
{
	return 0;
}

static void dwc_gadget_ep0_fifo_flush(struct usb_ep *ep)
{
}

static const struct usb_ep_ops dwc_gadget_ep0_ops = {
	.enable		= dwc_gadget_ep0_enable,
	.disable	= dwc_gadget_ep0_disable,
	.alloc_request	= dwc_gadget_ep0_alloc_request,
	.free_request	= dwc_gadget_ep0_free_request,
	.queue		= dwc_gadget_ep0_queue,
	.dequeue	= dwc_gadget_ep0_dequeue,
	.set_halt	= dwc_gadget_ep0_set_halt,
	.set_wedge	= dwc_gadget_ep0_set_wedge,
	.fifo_status	= dwc_gadget_ep0_fifo_status,
	.fifo_flush	= dwc_gadget_ep0_fifo_flush,
};
/* -------------------------------------------------------------------------- */

static int dwc_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_ep		*d_ep;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("dwc3: missing bMaxPacketSize\n");
		return -EINVAL;
	}

	return dwc_init_endpoint(d_ep, desc);
}

static int dwc_gadget_ep_disable(struct usb_ep *ep)
{
	struct dwc3_ep		*d_ep;

	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	return dwc_disable_endpoint(d_ep);
}

static struct usb_request *dwc_gadget_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req;
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req) {
		dev_err(dwc->dev, "not enough memory\n");
		return NULL;
	}

	/*
	 * first list_head is for gadget driver usage, second one is
	 * for our own housekeeping.
	 */
	INIT_LIST_HEAD(&req->request.list);
	INIT_LIST_HEAD(&req->list);
	req->epnum	= dep->number;
	req->dep	= dep;

	return &req->request;
}

static void dwc_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);

	list_del(&req->list);
	kfree(req);
}

static int dwc_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	if (!dep->desc) {
		dev_dbg(dwc->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	dev_vdbg(dwc->dev, "queing request %p to %s\n", request, ep->name);

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	dwc_map_buffer_to_dma(req);

	spin_lock_irqsave(&dwc->lock, flags);

	/* this request can be added to endpoint list of requests */
	list_add_tail(&req->list, &dep->request_list);

	/*
	 * Now we need a way to start consuming the list. I'm interested
	 * in something which doesn't look as ugly as MUSB's way of consuming
	 * the list (if this is the first entry, consume) because that'll
	 * prevent us from queueing several transfer requests to the device
	 * and let DMA play its role.
	 *
	 * This is a new driver, so we have a chance to do it right. Let's
	 * do so
	 */

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static int dwc_gadget_ep_dequeue(struct usb_ep *ep, struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_request		*r = NULL;

	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	int				ret = 0;

	spin_lock_irqsave(&dwc->lock, flags);

	list_for_each_entry(r, &dep->request_list, list) {
		if (r == req)
			break;
	}

	if (r != req) {
		dev_err(dwc->dev, "requeust %p was not queued to %s\n",
				request, ep->name);
		ret = -EINVAL;
		goto out0;
	}

	/* giveback the request */
	dwc_gadget_giveback(dep, req, -ECONNRESET);

out0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	return 0;
}

static int dwc_gadget_ep_set_wedge(struct usb_ep *ep)
{
	return 0;
}

static int dwc_gadget_ep_fifo_status(struct usb_ep *ep)
{
	return 0;
}

static void dwc_gadget_ep_fifo_flush(struct usb_ep *ep)
{
}

static const struct usb_ep_ops dwc_gadget_ep_ops = {
	.enable		= dwc_gadget_ep_enable,
	.disable	= dwc_gadget_ep_disable,
	.alloc_request	= dwc_gadget_ep_alloc_request,
	.free_request	= dwc_gadget_ep_free_request,
	.queue		= dwc_gadget_ep_queue,
	.dequeue	= dwc_gadget_ep_dequeue,
	.set_halt	= dwc_gadget_ep_set_halt,
	.set_wedge	= dwc_gadget_ep_set_wedge,
	.fifo_status	= dwc_gadget_ep_fifo_status,
	.fifo_flush	= dwc_gadget_ep_fifo_flush,
};

/* -------------------------------------------------------------------------- */

static void __init dwc3_gadget_init_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*ep;
	u8				epnum;

	INIT_LIST_HEAD(&dwc->gadget.ep_list);

	/* we know we have 32 EPs */
	for (epnum = 0; epnum < 32; epnum++) {
		ep = kzalloc(sizeof(*ep), GFP_KERNEL);
		if (!ep) {
			dev_err(dwc->dev, "can't allocate endpoint %d\n", epnum);
			return;
		}

		INIT_LIST_HEAD(&ep->endpoint.ep_list);

		ep->dwc = dwc;

		snprintf(ep->name, 20, "ep%d%s", epnum, !epnum ? "shared" :
				(epnum % 2) ? "in" : "false");
		ep->endpoint.name = ep->name;

		if (epnum == 0) {
			ep->endpoint.maxpacket = 64;
			ep->endpoint.ops = &dwc_gadget_ep0_ops;
			dwc->gadget.ep0 = &ep->endpoint;
		} else {
			ep->endpoint.maxpacket = 512;
			ep->endpoint.ops = &dwc_gadget_ep_ops;
			ep->direction = (epnum % 2) ? true : false;
			list_add_tail(&ep->endpoint.ep_list,
					&dwc->gadget.ep_list);
		}
	}
}

/**
 * dwc3_gadget_init - Initializes gadget related registers
 * @dwc: Pointer to out controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int __devinit dwc3_gadget_init(struct dwc3 *dwc)
{
	struct dwc3_gadget_ep_cmd_params	params;
	u32					reg;

	int					ret;

	memset(&params, 0x00, sizeof(params));

	/*
	 * REVISIT: Here we should flush all FIFOs and
	 * clear all pending IRQs to be sure we're starting
	 * from a well known location.
	 */

	reg = dwc3_readl(dwc->global, DWC3_GCTL);

	/*
	 * REVISIT: power down scale might be different
	 * depending on PHY used, need to pass that via platform_data
	 */
	reg |= DWC3_GCTL_PWRDNSCALE(0x61a) | DWC3_GCTL_DISSCRAMBLE;
	dwc3_writel(dwc->global, DWC3_GCTL, reg);

	dwc3_writel(dwc->device, DWC3_DCFG, DWC3_DCFG_SUPERSPEED);

	ret = dwc3_send_gadget_ep_cmd(dwc, 0, DWC3_DEPCMD_DEPSTARTCFG, &params);
	if (ret)
		return ret;

	params.param0.depcfg.ep_type = DWC3_DEPCMD_TYPE_CONTROL;
	params.param0.depcfg.burst_size = 0;
	params.param0.depcfg.max_packet_size = 512;
	params.param0.depcfg.fifo_number = 0;

	params.param1.depcfg.xfer_not_ready_enable = true;
	params.param1.depcfg.xfer_complete_enable = true;

	ret = dwc3_send_gadget_ep_cmd(dwc, 0, DWC3_DEPCMD_SETEPCONFIG, &params);
	if (ret)
		return ret;

	ret = dwc3_send_gadget_ep_cmd(dwc, 1, DWC3_DEPCMD_SETEPCONFIG, &params);
	if (ret)
		return ret;

	params.param0.depxfercfg.number_xfer_resources = 1;
	params.param1.raw = 0;	/* be sure parameter1 is set to zero */

	ret = dwc3_send_gadget_ep_cmd(dwc, 0,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
	if (ret)
		return ret;

	ret = dwc3_send_gadget_ep_cmd(dwc, 1,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
	if (ret)
		return ret;

	/**
	 * TODO: Prepare a buffer for a setup packet, initialize
	 * a setup TRB, and issue a DEPSTRTXFER command for physical
	 * endpoint 0, pointing to the setup TRB. Poll CmdAct for completion.
	 *
	 * Note: The core will attempt to fetch the setup TRB via the master
	 * interface after this command completes.
	 */

	reg = dwc3_readl(dwc->device, DWC3_DEVTEN);

	/* Disable Start and End of Frame IRQs */
	reg &= ~(DWC3_DEVTEN_SOFEN | DWC3_DEVTEN_EOPFEN);
	dwc3_writel(dwc->device, DWC3_DEVTEN, reg);

	/* Enable physical EPs 0 & 1 */
	dwc3_writel(dwc->device, DWC3_DALEPENA, DWC3_DALEPENA_EPOUT(0)
			| DWC3_DALEPENA_EPIN(0));

	dwc3_gadget_init_endpoints(dwc);

	/* Set RUN/STOP bit */
	reg = dwc3_readl(dwc->device, DWC3_DCTL);
	reg |= DWC3_DCTL_RUN_STOP;
	dwc3_writel(dwc->device, DWC3_DCTL, reg);

	return 0;
}

