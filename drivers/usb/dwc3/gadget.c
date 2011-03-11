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
#include <linux/delay.h>
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

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

void dwc3_map_buffer_to_dma(struct dwc3_request *req)
{
	struct dwc3			*dwc = req->dep->dwc;

	if (req->request.dma == DMA_ADDR_INVALID) {
		req->request.dma = dma_map_single(dwc->dev, req->request.buf,
				req->request.length, req->direction
				? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = true;
	} else {
		dma_sync_single_for_device(dwc->dev, req->request.dma,
				req->request.length, req->direction
				? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = false;
	}
}

void dwc3_unmap_buffer_from_dma(struct dwc3_request *req)
{
	struct dwc3			*dwc = req->dep->dwc;

	if (req->mapped) {
		dma_unmap_single(dwc->dev, req->request.dma,
				req->request.length, req->direction
				? DMA_TO_DEVICE : DMA_FROM_DEVICE);
		req->mapped = 0;
	} else {
		dma_sync_single_for_cpu(dwc->dev, req->request.dma,
				req->request.length, req->direction
				? DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}

	dma_unmap_single(dwc->dev, req->trb_dma, sizeof(struct dwc3_trb),
			DMA_BIDIRECTIONAL);
}

static void dwc3_gadget_giveback(struct dwc3_ep *dep, struct dwc3_request *req,
		int status)
{
	struct dwc3			*dwc = dep->dwc;

	dwc3_gadget_del_request(req);

	if (req->request.status == -EINPROGRESS)
		req->request.status = status;

	dwc3_unmap_buffer_from_dma(req);

	spin_unlock(&dwc->lock);
	req->request.complete(&req->dep->endpoint, &req->request);
	spin_lock(&dwc->lock);

	dev_dbg(dwc->dev, "request %p from %s completed %d/%d ===> %d\n",
			req, dep->name, req->request.actual,
			req->request.length, status);
}

int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params)
{
	unsigned long		timeout = jiffies + msecs_to_jiffies(500);
	u32			reg;

	dwc3_writel(dwc->device, DWC3_DEPCMDPAR0(ep), params->param0.raw);
	dwc3_writel(dwc->device, DWC3_DEPCMDPAR1(ep), params->param1.raw);
	dwc3_writel(dwc->device, DWC3_DEPCMDPAR2(ep), params->param2.raw);

	dwc3_writel(dwc->device, DWC3_DEPCMD(ep), cmd | DWC3_DEPCMD_CMDACT);
	do {
		reg = dwc3_readl(dwc->device, DWC3_DEPCMD(ep));
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (!(reg & DWC3_DEPCMD_CMDACT));

	return 0;
}

static int dwc_alloc_trb_pool(struct dwc3_ep *dep)
{
	if (dep->trb_pool)
		return 0;
	if (dep->number == 0 || dep->number == 1)
		return 0;

	dep->trb_pool = kzalloc(sizeof(struct dwc3_trb) * DWC3_TRB_NUM,
			GFP_KERNEL);
	if (!dep->trb_pool) {
		dev_err(dep->dwc->dev, "failed to allocate trb pool for %s\n",
				dep->name);
		return -ENOMEM;
	}
	return 0;
}

static void dwc_free_trb_pool(struct dwc3_ep *dep)
{
	kfree(dep->trb_pool);
	dep->trb_pool = NULL;
}

/**
 * dwc3_init_endpoint - Initializes a HW endpoint
 * @dep: endpoint to be initialized
 * @desc: USB Endpoint Descriptor
 *
 * Caller should take care of locking
 */
static int dwc3_init_endpoint(struct dwc3_ep *dep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_gadget_ep_cmd_params params;

	struct dwc3		*dwc = dep->dwc;

	u32			reg;

	int			ret = -ENOMEM;

	if (dep->flags & DWC3_EP_ENABLED) {
		WARN_ONCE(true, "%s is already enabled\n", dep->name);
		return 0;
	}

	if (dwc_alloc_trb_pool(dep))
		goto err0;

	memset(&params, 0x00, sizeof(params));

	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_DEPSTARTCFG, &params);
	if (ret) {
		dev_err(dwc->dev, "failed to start new configuration for %s\n",
				dep->name);
		goto err1;
	}

	params.param0.depcfg.ep_type = desc->bmAttributes &
		USB_ENDPOINT_XFERTYPE_MASK;

	params.param0.depcfg.ignore_sequence_number = true;

	switch (dwc->speed) {
	case DWC3_DSTS_SUPERSPEED:
		params.param0.depcfg.max_packet_size = 512;
		break;

	case DWC3_DSTS_HIGHSPEED:
	case DWC3_DSTS_FULLSPEED2:
	case DWC3_DSTS_FULLSPEED1:
		params.param0.depcfg.max_packet_size = 64;
		break;

	case DWC3_DSTS_LOWSPEED:
		params.param0.depcfg.max_packet_size = 8;
		break;
	}

	params.param1.depcfg.xfer_complete_enable = true;
	params.param1.depcfg.xfer_in_progress_enable = true;
	params.param1.depcfg.xfer_not_ready_enable = true;
	params.param1.depcfg.ep_number = dep->number;

	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETEPCONFIG, &params);
	if (ret) {
		dev_err(dwc->dev, "failed to configure %s\n", dep->name);
		goto err1;
	}

	memset(&params, 0x00, sizeof(params));
	params.param0.depxfercfg.number_xfer_resources = 1;
	ret = dwc3_send_gadget_ep_cmd(dwc, 0,
			DWC3_DEPCMD_SETTRANSFRESOURCE, &params);
	if (ret)
		goto err1;

	dep->desc = desc;
	dep->type = usb_endpoint_type(desc);
	dep->flags |= DWC3_EP_ENABLED;

	reg = dwc3_readl(dwc->device, DWC3_DALEPENA);

	if (usb_endpoint_dir_in(dep->desc))
		reg |= DWC3_DALEPENA_EPIN(dep->number);
	else
		reg |= DWC3_DALEPENA_EPOUT(dep->number);

	dwc3_writel(dwc->device, DWC3_DALEPENA, reg);

	return 0;

err1:
	dwc_free_trb_pool(dep);

err0:
	return ret;
}

/**
 * dwc3_disable_endpoint - Disables a HW endpoint
 * @dep: the endpoint to disable
 *
 * Caller should take care of locking
 */
static int dwc3_disable_endpoint(struct dwc3_ep *dep)
{
	struct dwc3_gadget_ep_cmd_params params;

	struct dwc3		*dwc = dep->dwc;

	u32			reg;

	int			ret = -ENOMEM;

	if (!(dep->flags & DWC3_EP_ENABLED)) {
		WARN_ONCE(true, "%s is already disabled\n", dep->name);
		return 0;
	}

	memset(&params, 0x00, sizeof(params));

	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_DEPSTARTCFG, &params);
	if (ret) {
		dev_err(dwc->dev, "failed to start new configuration for %s\n",
				dep->name);
		return ret;
	}

	params.param0.depcfg.ep_type = 0;
	params.param0.depcfg.ignore_sequence_number = false;
	params.param0.depcfg.max_packet_size = 0;
	params.param1.depcfg.xfer_complete_enable = false;
	params.param1.depcfg.xfer_in_progress_enable = false;
	params.param1.depcfg.xfer_not_ready_enable = false;
	params.param1.depcfg.ep_number = dep->number;

	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETEPCONFIG, &params);
	if (ret) {
		dev_err(dwc->dev, "failed to configure %s\n", dep->name);
		return ret;
	}

	reg = dwc3_readl(dwc->device, DWC3_DALEPENA);

	if (usb_endpoint_dir_in(dep->desc))
		reg &= ~DWC3_DALEPENA_EPIN(dep->number);
	else
		reg &= ~DWC3_DALEPENA_EPOUT(dep->number);

	dwc3_writel(dwc->device, DWC3_DALEPENA, reg);

	dep->desc = NULL;
	dep->type = 0;
	dep->flags &= ~DWC3_EP_ENABLED;

	dwc_free_trb_pool(dep);

	return 0;
}

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_ep0_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	return -EINVAL;
}

static int dwc3_gadget_ep0_disable(struct usb_ep *ep)
{
	return -EINVAL;
}

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_ep_enable(struct usb_ep *ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct dwc3_ep			*dep;
	struct dwc3			*dwc;
	unsigned long			flags;
	int				ret;

	if (!ep || !desc || desc->bDescriptorType != USB_DT_ENDPOINT) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	if (!desc->wMaxPacketSize) {
		pr_debug("dwc3: missing bMaxPacketSize\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);
	dwc = dep->dwc;

	spin_lock_irqsave(&dwc->lock, flags);
	ret = dwc3_init_endpoint(dep, desc);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_disable(struct usb_ep *ep)
{
	struct dwc3_ep		*dep;

	if (!ep) {
		pr_debug("dwc3: invalid parameters\n");
		return -EINVAL;
	}

	dep = to_dwc3_ep(ep);

	return dwc3_disable_endpoint(dep);
}

static struct usb_request *dwc3_gadget_ep_alloc_request(struct usb_ep *ep,
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

	req->epnum	= dep->number;
	req->dep	= dep;
	req->request.dma = DMA_ADDR_INVALID;

	return &req->request;
}

static void dwc3_gadget_ep_free_request(struct usb_ep *ep,
		struct usb_request *request)
{
	struct dwc3_request		*req = to_dwc3_request(request);

	kfree(req);
}

static struct dwc3_trb *dwc3_alloc_trb(struct dwc3_ep *dep,
		unsigned type, unsigned length)
{
	struct dwc3_trb			*trb;

	trb = &dep->trb_pool[dep->current_trb];
	if (!trb)
		return NULL;

	trb->trbctl	= type;
	trb->length	= length;

	dep->current_trb += 1;

	return trb;
}

static void dwc3_free_trb(struct dwc3_ep *dep, struct dwc3_trb *trb)
{
	/* TODO */
}

static int __dwc3_gadget_kick_transfer(struct dwc3_ep *dep,
		struct dwc3_request *req, unsigned is_chained)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb		*trb = req->trb;
	int			ret;

	trb->bpl	= req->request.dma;
	trb->hwo	= true;
	trb->lst	= !is_chained;
	trb->chn	= !!is_chained;
	trb->ioc	= !is_chained;

	if (usb_endpoint_xfer_isoc(dep->desc))
		trb->isp_imi = true;

	req->trb_dma = dma_map_single(dwc->dev, trb, sizeof(*trb),
			DMA_BIDIRECTIONAL);

	if (!is_chained) {
		struct dwc3_gadget_ep_cmd_params params;

		/*
		 * We change the pointer here. This is needed in case we
		 * are handling chained TRBs.
		 *
		 * On that scenario, we need to prepare all TRBs (map
		 * the TRB buffer to DMA) but kick the transfer with
		 * the first one on the list.
		 */
		req = next_request(dep);

		memset(&params, 0, sizeof(params));
		params.param0.depstrtxfer.transfer_desc_addr_high = 0;
		params.param1.depstrtxfer.transfer_desc_addr_low = req->trb_dma;

		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_STARTTRANSFER, &params);
		if (ret < 0) {
			dev_dbg(dwc->dev, "failed to send STARTTRANSFER command\n");
			dwc3_unmap_buffer_from_dma(req);
			dwc3_free_trb(dep, trb);
			list_del(&req->list);

			/*
			 * FIXME we need to iterate over the list of requests
			 * here and stop, unmap, free and del each of the linked
			 * requests.
			 */
			return ret;
		}

		dep->res_trans_idx = dwc3_gadget_ep_get_transfer_index(dwc,
				dep->number);
	}

	return 0;
}

static int __dwc3_gadget_kick_transfers(struct dwc3_ep *dep)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_request	*req;

	unsigned		count = dep->request_count;
	unsigned		i = 0;

	int			ret = 0;

	list_for_each_entry(req, &dep->request_list, list) {
		ret = __dwc3_gadget_kick_transfer(dep, req,
				(i == count - 1) ? false : true);
		if (ret) {
			dev_err(dwc->dev, "%s failed to start request %p\n",
					dep->name, req);
			break;
		}
		i++;
	}

	return ret;
}

static int __dwc3_gadget_ep_queue(struct dwc3_ep *dep, struct dwc3_request *req,
		unsigned is_chained)
{
	struct dwc3		*dwc = dep->dwc;
	struct dwc3_trb		*trb;

	unsigned		trb_type;

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	switch (usb_endpoint_type(dep->desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		trb_type = DWC3_TRBCTL_CONTROL_SETUP;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		trb_type = DWC3_TRBCTL_ISOCHRONOUS;
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		trb_type = DWC3_TRBCTL_NORMAL;
		break;
	default:
		return -EINVAL;
	}

	trb = dwc3_alloc_trb(dep, trb_type, req->request.length);
	if (!trb) {
		dev_err(dwc->dev, "can't allocate TRB\n");
		return -ENOMEM;
	}

	req->trb = trb;
	dwc3_map_buffer_to_dma(req);

	dwc3_gadget_add_request(dep, req);

	if (!(dep->request_count == 1)) {
		dev_vdbg(dwc->dev, "%s's request_list isn't singular\n",
				dep->name);
		return 0;
	}

	return __dwc3_gadget_kick_transfer(dep, req, is_chained);
}

static int dwc3_gadget_ep_queue(struct usb_ep *ep, struct usb_request *request,
	gfp_t gfp_flags)
{
	struct dwc3_request		*req = to_dwc3_request(request);
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret;

	if (!dep->desc) {
		dev_dbg(dwc->dev, "trying to queue request %p to disabled %s\n",
				request, ep->name);
		return -ESHUTDOWN;
	}

	dev_vdbg(dwc->dev, "queing request %p to %s\n", request, ep->name);

	spin_lock_irqsave(&dwc->lock, flags);
	ret = __dwc3_gadget_ep_queue(dep, req, 0);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_dequeue(struct usb_ep *ep,
		struct usb_request *request)
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
	dwc3_gadget_giveback(dep, req, -ECONNRESET);

out0:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_set_halt(struct usb_ep *ep, int value)
{
	struct dwc3_gadget_ep_cmd_params params;

	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;

	int				ret = 0;

	memset(&params, 0x00, sizeof(params));

	spin_lock_irqsave(&dwc->lock, flags);

	if (usb_endpoint_xfer_isoc(dep->desc)) {
		dev_err(dwc->dev, "%s is of Isochronous type\n", dep->name);
		ret = -EINVAL;
		goto out;
	}

	if (value) {
		if (dep->number == 0 || dep->number == 1)
			dwc->ep0state = EP0_STALL;

		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_SETSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags |= DWC3_EP_STALL;
	} else {
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_CLEARSTALL, &params);
		if (ret)
			dev_err(dwc->dev, "failed to %s STALL on %s\n",
					value ? "set" : "clear",
					dep->name);
		else
			dep->flags &= ~DWC3_EP_STALL;
	}

out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_ep_set_wedge(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);

	dep->flags |= DWC3_EP_WEDGE;

	return usb_ep_set_halt(ep);
}

static void dwc3_gadget_ep_fifo_flush(struct usb_ep *ep)
{
	struct dwc3_ep			*dep = to_dwc3_ep(ep);
	struct dwc3			*dwc = dep->dwc;

	unsigned long			flags;
	unsigned			reg;

	spin_lock_irqsave(&dwc->lock, flags);

	reg = dep->number;
	reg |= ((dep->number & 1) << 5);

	dwc3_writel(dwc->global, DWC3_DGCMDPAR, reg);

	reg = DWC3_DGCMD_SELECTED_FIFO_FLUSH;

	dwc3_writel(dwc->global, DWC3_DGCMD, reg);

	spin_unlock_irqrestore(&dwc->lock, flags);
}

/* -------------------------------------------------------------------------- */

static const struct usb_endpoint_descriptor dwc3_gadget_ep0_desc = {
	.bLength	= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0,
	.bmAttributes	= USB_ENDPOINT_XFER_CONTROL,
	.bInterval	= 1, /* What value to put here ? */
};

static const struct usb_ep_ops dwc3_gadget_ep0_ops = {
	.enable		= dwc3_gadget_ep0_enable,
	.disable	= dwc3_gadget_ep0_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep0_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
	.fifo_flush	= dwc3_gadget_ep_fifo_flush,
};

static const struct usb_ep_ops dwc3_gadget_ep_ops = {
	.enable		= dwc3_gadget_ep_enable,
	.disable	= dwc3_gadget_ep_disable,
	.alloc_request	= dwc3_gadget_ep_alloc_request,
	.free_request	= dwc3_gadget_ep_free_request,
	.queue		= dwc3_gadget_ep_queue,
	.dequeue	= dwc3_gadget_ep_dequeue,
	.set_halt	= dwc3_gadget_ep_set_halt,
	.set_wedge	= dwc3_gadget_ep_set_wedge,
	.fifo_flush	= dwc3_gadget_ep_fifo_flush,
};

/* -------------------------------------------------------------------------- */

static int dwc3_gadget_get_frame(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;
	u32			reg;

	spin_lock_irqsave(&dwc->lock, flags);
	reg = dwc3_readl(dwc->device, DWC3_DSTS);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return DWC3_DSTS_SOFFN(reg);
}

static int dwc3_gadget_wakeup(struct usb_gadget *g)
{
	struct dwc3		*dwc = gadget_to_dwc(g);

	unsigned long		timeout;
	unsigned long		flags;

	u32			reg;

	int			ret = 0;

	u8			link_state;
	u8			speed;

	spin_lock_irqsave(&dwc->lock, flags);

	/*
	 * According to the Databook Remote wakeup request should
	 * be issued only when the device is in early suspend state.
	 *
	 * We can check that via USB Link State bits in DSTS register.
	 */
	reg = dwc3_readl(dwc->device, DWC3_DSTS);

	speed = reg & DWC3_DSTS_CONNECTSPD;
	if (speed == DWC3_DSTS_SUPERSPEED) {
		dev_dbg(dwc->dev, "no wakeup on SuperSpeed\n");
		ret = -EINVAL;
		goto out;
	}

	link_state = DWC3_DSTS_USBLNKST(reg);

	switch (link_state) {
	case DWC3_LINK_STATE_RX_DET:	/* in HS, means Early Suspend */
	case DWC3_LINK_STATE_U3:	/* in HS, means SUSPEND */
		break;
	default:
		dev_dbg(dwc->dev, "can't wakeup from link state %d\n",
				link_state);
		ret = -EINVAL;
		goto out;
	}

	reg = dwc3_readl(dwc->device, DWC3_DCTL);

	/*
	 * Switch link state to Recovery. In HS/FS/LS this means
	 * RemoteWakeup Request
	 * */
	reg |= DCW3_DCTL_ULSTCHNG_RECOVERY;
	dwc3_writel(dwc->device, DWC3_DCTL, reg);

	/* wait for at least 2000us */
	usleep_range(2000, 2500);

	/* write zeroes to Link Change Request */
	reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;

	/* pool until Link State change to ON */
	timeout = jiffies + msecs_to_jiffies(100);

	while (!(time_after(jiffies, timeout))) {
		reg = dwc3_readl(dwc->device, DWC3_DSTS);

		/* in HS, means ON */
		if (DWC3_DSTS_USBLNKST(reg) == DWC3_LINK_STATE_U0)
			break;
	}

	if (DWC3_DSTS_USBLNKST(reg) != DWC3_LINK_STATE_U0) {
		dev_err(dwc->dev, "failed to send remote wakeup\n");
		ret = -EINVAL;
	}

out:
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static int dwc3_gadget_set_selfpowered(struct usb_gadget *g,
		int is_selfpowered)
{
	struct dwc3		*dwc = gadget_to_dwc(g);

	dwc->is_selfpowered = !!is_selfpowered;

	return 0;
}

static int dwc3_gadget_vbus_session(struct usb_gadget *g, int is_active)
{
	return 0;
}

static int dwc3_gadget_vbus_draw(struct usb_gadget *g, unsigned mA)
{
	return 0;
}

static void dwc3_gadget_run_stop(struct dwc3 *dwc, int is_on)
{
	u32			reg;
	unsigned long timeout;

	reg = dwc3_readl(dwc->device, DWC3_DCTL);
	if (is_on)
		reg |= DWC3_DCTL_RUN_STOP;
	else
		reg &= ~DWC3_DCTL_RUN_STOP;

	dwc3_writel(dwc->device, DWC3_DCTL, reg);

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		reg = dwc3_readl(dwc->device, DWC3_DSTS);
		if (is_on) {
			if (!(reg & DWC3_DSTS_DEVCTRLHLT))
				break;
		} else {
			if (reg & DWC3_DSTS_DEVCTRLHLT)
				break;
		}
		cpu_relax();
		if (time_after(jiffies, timeout))
			break;
	} while (1);

	dev_vdbg(dwc->dev, "gadget %s data soft-%s\n",
			dwc->gadget_driver->function,
			is_on ? "connect" : "disconnect");
}

static int dwc3_gadget_pullup(struct usb_gadget *g, int is_on)
{
	struct dwc3		*dwc = gadget_to_dwc(g);
	unsigned long		flags;

	is_on = !!is_on;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_gadget_run_stop(dwc, is_on);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static const struct usb_gadget_ops dwc3_gadget_ops = {
	.get_frame		= dwc3_gadget_get_frame,
	.wakeup			= dwc3_gadget_wakeup,
	.set_selfpowered	= dwc3_gadget_set_selfpowered,
	.vbus_session		= dwc3_gadget_vbus_session,
	.vbus_draw		= dwc3_gadget_vbus_draw,
	.pullup			= dwc3_gadget_pullup,
};

/* -------------------------------------------------------------------------- */

static int __init dwc3_gadget_init_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	INIT_LIST_HEAD(&dwc->gadget.ep_list);

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = kzalloc(sizeof(*dep), GFP_KERNEL);
		if (!dep) {
			dev_err(dwc->dev, "can't allocate endpoint %d\n",
					epnum);
			return -ENOMEM;
		}

		dep->dwc = dwc;
		dep->number = epnum;
		dwc->eps[epnum] = dep;

		snprintf(dep->name, sizeof(dep->name), "ep%d%s", epnum,
				(epnum & 1) ? "in" : "out");
		dep->endpoint.name = dep->name;

		if (epnum == 0 || epnum == 1) {
			dep->endpoint.maxpacket = 64;
			dep->endpoint.ops = &dwc3_gadget_ep0_ops;
			if (!epnum)
				dwc->gadget.ep0 = &dep->endpoint;
		} else {
			dep->endpoint.maxpacket = 512;
			dep->endpoint.ops = &dwc3_gadget_ep_ops;
			dep->direction = (epnum % 2) ? true : false;
			list_add_tail(&dep->endpoint.ep_list,
					&dwc->gadget.ep_list);
		}
	}

	return 0;
}

static void __devexit dwc3_gadget_free_endpoints(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	u8				epnum;

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		dep = dwc->eps[epnum];
		if (epnum != 0)
			list_del(&dep->endpoint.ep_list);
		kfree(dep);
	}
}

static struct dwc3	*the_dwc;

static void dwc3_gadget_release(struct device *dev)
{
	dev_dbg(dev, "%s\n", __func__);
}

/* -------------------------------------------------------------------------- */

static irqreturn_t dwc3_endpoint_transfer_complete(struct dwc3 *dwc,
		struct dwc3_ep *dep, const unsigned count)
{
	struct dwc3_request	*req;
	unsigned		status = 0;
	int			ret;

	req = next_request(dep);

	if (!req) {
		dev_err(dwc->dev, "no transfer to complete on %s ?\n",
				dep->name);
		return IRQ_NONE;
	}

	if (req->trb->hwo) {
		dev_err(dwc->dev, "%s's TRB (%p) still owned by HW\n",
				dep->name, req->trb);
		return IRQ_NONE;
	}

	if (dep->direction) {
		if (count) {
			dev_err(dwc->dev, "incomplete TX/IN transfer on %s\n",
					dep->name);
			status = -ECONNRESET;
		}
	}

	/*
	 * We assume here we will always receive the entire data block
	 * which we should receive. Meaning, if we program RX to receive
	 * 4K but we receive only 2K, we assume that's all we should receive
	 * and we simply bounce the request back to the gadget driver for
	 * further processing.
	 */
	req->request.actual += count;

	/*
	 * Giveback the request. If we couldn't transfer everything,
	 * giveback anyway but change status to -ECONNRESET
	 */
	dwc3_gadget_giveback(dep, req, status);

	if (!dep->request_count)
		goto out;

	ret = __dwc3_gadget_kick_transfers(dep);
	if (ret) {
		dev_err(dwc->dev, "%s failed to start next request\n",
				dep->name);
		/*
		 * FIXME on error, we should be giving back all broken
		 * requests so gadget driver can re-start them or take
		 * any other action
		 */
	}

out:
	return IRQ_HANDLED;
}

static irqreturn_t dwc3_endpoint_interrupt(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	struct dwc3_ep		*dep;
	irqreturn_t		ret = IRQ_HANDLED;
	u8			epnum = event->endpoint_number;

	dep = dwc->eps[epnum];

	if (epnum == 0 || epnum == 1)
		return dwc3_ep0_interrupt(dwc, event);

	switch (event->endpoint_event) {
	case DWC3_DEPEVT_XFERCOMPLETE:
		if (usb_endpoint_xfer_isoc(dep->desc)) {
			dev_err(dwc->dev, "%s is an Isochronous endpoint\n",
					dep->name);
			return ret;
		}

		ret = dwc3_endpoint_transfer_complete(dwc, dep,
				(event->parameters & 0x00ffffff));
		break;
	case DWC3_DEPEVT_XFERINPROGRESS:
		if (!usb_endpoint_xfer_isoc(dep->desc)) {
			dev_err(dwc->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
			return ret;
		}

		break;
	case DWC3_DEPEVT_XFERNOTREADY:
		if (!usb_endpoint_xfer_isoc(dep->desc)) {
			dev_err(dwc->dev, "%s is not an Isochronous endpoint\n",
					dep->name);
			return ret;
		}

		break;
	case DWC3_DEPEVT_RXTXFIFOEVT:
		dev_dbg(dwc->dev, "%s FIFO Overrun\n", dep->name);
		break;
	case DWC3_DEPEVT_STREAMEVT:
		dev_dbg(dwc->dev, "%s Stream Event\n", dep->name);
		break;
	case DWC3_DEPEVT_EPCMDCMPLT:
		dev_dbg(dwc->dev, "%s Command Complete\n", dep->name);
		break;
	}

	return ret;
}

static void dwc3_disconnect_gadget(struct dwc3 *dwc)
{
	if (dwc->gadget_driver && dwc->gadget_driver->disconnect) {
		spin_unlock(&dwc->lock);
		dwc->gadget_driver->disconnect(&dwc->gadget);
		spin_lock(&dwc->lock);
	}
}

static void dwc3_stop_active_transfers(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 0; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;
		struct dwc3_gadget_ep_cmd_params params;
		u32 cmd;
		int ret;

		if (epnum == 0) {
			/*
			 * XXX
			 * get the core into the "Setup a Control-Setup TRB /
			 * Start Transfer" state in case it is busy here.
			 */
			continue;
		}
		dep = dwc->eps[epnum];

		if (!(dep->flags & DWC3_EP_ENABLED))
			continue;

		cmd = DWC3_DEPCMD_ENDTRANSFER;

		/*
		 * This one issues an interrupt. I wonder if we have to wait
		 * for it or can simply ignore/remove the inrerupt
		 */
		cmd |= DWC3_DEPCMD_CMDIOC;
		cmd |= DWC3_DEPCMD_HIPRI_FORCERM;
		cmd |= DWC3_DEPCMD_PARAM(dep->res_trans_idx);
		memset(&params, 0, sizeof(params));
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number, cmd, &params);
		WARN_ON_ONCE(ret);
	}
}

static void dwc3_clear_stall_all_ep(struct dwc3 *dwc)
{
	u32 epnum;

	for (epnum = 1; epnum < DWC3_ENDPOINTS_NUM; epnum++) {
		struct dwc3_ep *dep;
		struct dwc3_gadget_ep_cmd_params params;
		int ret;

		dep = dwc->eps[epnum];

		if (!(dep->flags & DWC3_EP_STALL))
			continue;

		dep->flags &= ~DWC3_EP_STALL;

		memset(&params, 0, sizeof(params));
		ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
				DWC3_DEPCMD_CLEARSTALL, &params);
		WARN_ON_ONCE(ret);
	}
}

static irqreturn_t dwc3_gadget_disconnect_interrupt(struct dwc3 *dwc)
{
	dev_vdbg(dwc->dev, "%s\n", __func__);
#if 0
	XXX
	U1/U2 is powersave optimization. Skip it for now. Anyway we need to
	enable it before we can disable it.

	reg = dwc3_readl(dwc->device, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_writel(dwc->device, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_writel(dwc->device, DWC3_DCTL, reg);
#endif
	dwc3_disconnect_gadget(dwc);
	dwc3_stop_active_transfers(dwc);

	dwc3_gadget_run_stop(dwc, 0);

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_gadget_reset_interrupt(struct dwc3 *dwc)
{
	u32			reg;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	dwc3_disconnect_gadget(dwc);
	dwc3_stop_active_transfers(dwc);
	dwc3_clear_stall_all_ep(dwc);

	/* Reset device address to zero */
	reg = dwc3_readl(dwc->device, DWC3_DCTL);
	reg &= ~(DWC3_DCFG_DEVADDR(0));
	dwc3_writel(dwc->device, DWC3_DCTL, reg);

	/* The following could be part of dwc3_stop_active_transfers() on EP0 */
	/* Enable ep0 in DALEPENA register */
	reg = dwc3_readl(dwc->device, DWC3_DALEPENA);
	reg |= DWC3_DALEPENA_EPOUT(0) | DWC3_DALEPENA_EPIN(0);
	dwc3_writel(dwc->device, DWC3_DALEPENA, reg);

	/*
	 * Wait for RxFifo to drain
	 *
	 * REVISIT probably shouldn't wait forever.
	 * In case Hardware ends up in a screwed up
	 * case, we error out, notify the user and,
	 * maybe, WARN() or BUG() but leave the rest
	 * of the kernel working fine.
	 *
	 * REVISIT the below is rather CPU intensive,
	 * maybe we should read and if it doesn't work
	 * sleep (not busy wait) for a few useconds.
	 *
	 * REVISIT why wait until the RXFIFO is empty anyway?
	 */
	while (!(dwc3_readl(dwc->device, DWC3_DSTS)
				& DWC3_DSTS_RXFIFOEMPTY))
		cpu_relax();

	dwc->ep0state = EP0_IDLE;

	return IRQ_HANDLED;
}

static void dwc3_update_ram_clk_sel(struct dwc3 *dwc, u32 speed)
{
	u32 reg;
	u32 usb30_clock = DWC3_GCTL_CLK_BUS;

	/*
	 * We change the clock only at SS but I dunno why I would want to do
	 * this. Maybe it becomes part of the power saving plan.
	 */

	if (speed != DWC3_DSTS_SUPERSPEED)
		return;

	/*
	 * RAMClkSel is reset to 0 after USB reset, so it must be reprogrammed
	 * each time on Connect Done.
	 */
	if (!usb30_clock)
		return;

	reg = dwc3_readl(dwc->global, DWC3_GCTL);
	reg |= DWC3_GCTL_RAMCLKSEL(usb30_clock);
	dwc3_writel(dwc->global, DWC3_GCTL, reg);
}

static irqreturn_t dwc3_gadget_conndone_interrupt(struct dwc3 *dwc)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_ep		*dep;

	u32			reg;
	int			ret;

	u8			speed;

	dev_vdbg(dwc->dev, "%s\n", __func__);

	memset(&params, 0x00, sizeof(params));

	dwc->ep0state = EP0_IDLE;
	reg = dwc3_readl(dwc->device, DWC3_DSTS);
	speed = reg & DWC3_DSTS_CONNECTSPD;
	dwc->speed = speed;

	dwc3_update_ram_clk_sel(dwc, speed);

	dep = dwc->eps[0];
	ret = dwc3_init_endpoint(dep, &dwc3_gadget_ep0_desc);
	if (ret) {
		dev_err(dwc->dev, "failed to enabled %s\n", dep->name);
		return IRQ_NONE;
	}

	dep = dwc->eps[1];
	ret = dwc3_init_endpoint(dep, &dwc3_gadget_ep0_desc);
	if (ret) {
		dev_err(dwc->dev, "failed to enabled %s\n", dep->name);
		return IRQ_NONE;
	}

	/*
	 * Configure PHY via GUSB3PIPECTLn if required.
	 *
	 * Update GTXFIFOSIZn
	 *
	 * In both cases reset values should be sufficient.
	 */

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_gadget_wakeup_interrupt(struct dwc3 *dwc)
{
	dev_vdbg(dwc->dev, "%s\n", __func__);

	/*
	 * TODO take core out of low power mode when that's
	 * implemented.
	 */

	dwc->gadget_driver->resume(&dwc->gadget);

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_gadget_linksts_change_interrupt(struct dwc3 *dwc,
		unsigned int evtinfo)
{
	dev_vdbg(dwc->dev, "%s\n", __func__);

	/*  The fith bit says SuperSpeed yes or no. */
	dwc->link_state = evtinfo & DWC3_LINK_STATE_MASK;

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_gadget_interrupt(struct dwc3 *dwc,
		struct dwc3_event_devt *event)
{
	irqreturn_t		ret = IRQ_NONE;

	switch (event->type) {
	case DWC3_DEVICE_EVENT_DISCONNECT:
		ret = dwc3_gadget_disconnect_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_RESET:
		ret = dwc3_gadget_reset_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_CONNECT_DONE:
		ret = dwc3_gadget_conndone_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_WAKEUP:
		ret = dwc3_gadget_wakeup_interrupt(dwc);
		break;
	case DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE:
		ret = dwc3_gadget_linksts_change_interrupt(dwc,
				event->event_info);
		break;
	case DWC3_DEVICE_EVENT_EOPF:
		dev_vdbg(dwc->dev, "End of Periodic Frame\n");
		break;
	case DWC3_DEVICE_EVENT_SOF:
		dev_vdbg(dwc->dev, "Start of Periodic Frame\n");
		break;
	case DWC3_DEVICE_EVENT_ERRATIC_ERROR:
		dev_vdbg(dwc->dev, "Erratic Error\n");
		break;
	case DWC3_DEVICE_EVENT_CMD_CMPL:
		dev_vdbg(dwc->dev, "Command Complete\n");
		break;
	case DWC3_DEVICE_EVENT_OVERFLOW:
		dev_vdbg(dwc->dev, "Overflow\n");
		break;
	default:
		dev_dbg(dwc->dev, "UNKNOWN IRQ %d\n", event->type);
	}

	return ret;
}

static irqreturn_t dwc3_process_event_entry(struct dwc3 *dwc,
		union dwc3_event *event)
{
	irqreturn_t ret;

	/* Endpoint IRQ, handle it and return early */
	if (event->type.is_devspec == 0) {
		/* depevt */
		return dwc3_endpoint_interrupt(dwc, &event->depevt);
	}

	switch (event->type.type) {
	case DWC3_EVENT_TYPE_DEV:
		ret = dwc3_gadget_interrupt(dwc, &event->devt);
		break;
	/* REVISIT what to do with Carkit and I2C events ? */
	default:
		ret = -EINVAL;
		dev_err(dwc->dev, "UNKNOWN IRQ type %d\n", event->raw);
	}

	return ret;
}

static irqreturn_t dwc3_process_event_buf(struct dwc3 *dwc, u32 buf)
{
	struct dwc3_event_buffer *evt;
	int left;
	u32 count;

	count = dwc3_readl(dwc->device, DWC3_GEVNTCOUNT(buf));
	count &= DWC3_GEVNTCOUNT_MASK;
	if (!count)
		return IRQ_NONE;

	evt = dwc->ev_buffs[buf];
	left = count;

	while (left > 0) {
		union dwc3_event event;

		memcpy(&event.raw, (evt->buf + evt->lpos), sizeof(event.raw));
		dwc3_process_event_entry(dwc, &event);
		/* what with the ret? */
		/*
		 * XXX we wrap around correctly to the next entry as almost all
		 * entries are 4 bytes in size. There is one entry which has 12
		 * bytes which is a regular entry followed by 8 bytes data. ATM
		 * I don't know how things are organized if were get next to the
		 * a boundary so I worry about that once we try to handle that.
		 */
		evt->lpos = (evt->lpos + 4) % DWC3_EVENT_BUFFERS_SIZE;
		left -= 4;
	}

	dwc3_writel(dwc->device, DWC3_GEVNTCOUNT(buf), count);

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_interrupt(int irq, void *_dwc)
{
	struct dwc3			*dwc = _dwc;
	unsigned long			flags;
	int				i;
	irqreturn_t			ret = IRQ_NONE;

	spin_lock_irqsave(&dwc->lock, flags);

	for (i = 0; i < DWC3_EVENT_BUFFERS_NUM; i++) {
		irqreturn_t status;

		status = dwc3_process_event_buf(dwc, i);
		if (status == IRQ_HANDLED)
			ret = status;
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

/**
 * dwc3_gadget_init - Initializes gadget related registers
 * @dwc: Pointer to out controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int __devinit dwc3_gadget_init(struct dwc3 *dwc)
{
	u32					reg;
	int					ret;
	int					irq;

	dev_set_name(&dwc->gadget.dev, "gadget");

	dwc->gadget.ops			= &dwc3_gadget_ops;
	dwc->gadget.is_dualspeed	= true;
	dwc->gadget.speed		= USB_SPEED_UNKNOWN;
	dwc->gadget.dev.parent		= dwc->dev;
	dwc->gadget.dev.dma_mask	= dwc->dev->dma_mask;
	dwc->gadget.dev.release		= dwc3_gadget_release;
	dwc->gadget.name		= "dwc3-gadget";

	the_dwc				= dwc;

	/* flush all fifos */
	reg = DWC3_DGCMD_ALL_FIFO_FLUSH;
	dwc3_writel(dwc->global, DWC3_DGCMD, reg);

	/*
	 * REVISIT: Here we should clear all pending IRQs to be
	 * sure we're starting from a well known location.
	 */

	reg = dwc3_readl(dwc->global, DWC3_GCTL);

	/*
	 * REVISIT: power down scale might be different
	 * depending on PHY used, need to pass that via platform_data
	 */
	reg |= DWC3_GCTL_PWRDNSCALE(0x61a) | DWC3_GCTL_DISSCRAMBLE;
	dwc3_writel(dwc->global, DWC3_GCTL, reg);

	dwc3_writel(dwc->device, DWC3_DCFG, DWC3_DCFG_SUPERSPEED);

	/* Disable Start and End of Frame IRQs */
	reg = dwc3_readl(dwc->device, DWC3_DEVTEN);
	reg &= ~(DWC3_DEVTEN_SOFEN | DWC3_DEVTEN_EOPFEN);
	dwc3_writel(dwc->device, DWC3_DEVTEN, reg);

	ret = dwc3_gadget_init_endpoints(dwc);
	if (ret)
		goto err1;

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);

	ret = request_irq(irq, dwc3_interrupt, 0, "dwc3", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
				irq, ret);
		goto err2;
	}
	/* begin to receive SETUP packets */
	dwc3_ep0_out_start(dwc, 0);

	return 0;

err2:
	dwc3_gadget_free_endpoints(dwc);

err1:
	the_dwc = NULL;

	return ret;
}

void __devexit dwc3_gadget_exit(struct dwc3 *dwc)
{
	int			irq;
	int			i;

	irq = platform_get_irq(to_platform_device(dwc->dev), 0);

	free_irq(irq, dwc);

	for (i = 0; i < ARRAY_SIZE(dwc->eps); i++)
		dwc3_disable_endpoint(dwc->eps[i]);

	dwc3_gadget_free_endpoints(dwc);

	the_dwc = NULL;
}

/* -------------------------------------------------------------------------- */

/**
 * usb_gadget_probe_driver - registers and probes the gadget driver.
 * @driver: the gadget driver to register and probe
 * @bind: the bind function
 */
int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	struct dwc3		*dwc = the_dwc;
	unsigned long		flags;
	int			ret;

	if (!driver || !bind || !driver->setup) {
		ret = -EINVAL;
		goto err0;
	}

	if (!dwc) {
		ret = -ENODEV;
		goto err0;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	if (dwc->gadget_driver) {
		dev_err(dwc->dev, "%s is already bound to %s\n",
				dwc->gadget.name,
				dwc->gadget_driver->driver.name);
		ret = -EBUSY;
		goto err1;
	}

	dwc->gadget_driver	= driver;
	dwc->gadget.dev.driver	= &driver->driver;
	driver->driver.bus	= NULL;

	spin_unlock_irqrestore(&dwc->lock, flags);

	ret = bind(&dwc->gadget);
	if (ret) {
		dev_err(dwc->dev, "bind failed\n");
		goto err2;
	}

	return 0;

err2:
	spin_lock_irqsave(&dwc->lock, flags);

	dwc->gadget_driver	= NULL;
	dwc->gadget.dev.driver	= NULL;

err1:
	spin_unlock_irqrestore(&dwc->lock, flags);

err0:
	return ret;
}
EXPORT_SYMBOL_GPL(usb_gadget_probe_driver);

/**
 * usb_gadget_unregister_driver - unregisters a gadget driver.
 * @driver: the gadget driver to unregister
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct dwc3		*dwc = the_dwc;
	unsigned long		flags;

	if (!driver || !driver->unbind)
		return -EINVAL;

	if (!dwc)
		return -ENODEV;

	if (dwc->gadget_driver != driver)
		return -EINVAL;

	driver->unbind(&dwc->gadget);

	spin_lock_irqsave(&dwc->lock, flags);

	dwc->gadget_driver	= NULL;
	dwc->gadget.dev.driver	= NULL;

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(usb_gadget_unregister_driver);
