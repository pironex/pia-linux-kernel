/**
 * ep0.c - DesignWare USB3 DRD Controller Endpoint 0 Handling
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
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

static int dwc3_ep0_start_trans(struct dwc3 *dwc, u8 epnum, dma_addr_t buf_dma,
		u32 len)
{
	struct dwc3_gadget_ep_cmd_params params;
	struct dwc3_trb			*trb;
	struct dwc3_ep			*dep;

	int				ret;

	dep = dwc->eps[epnum];

	trb = &dwc->ep0_trb;
	memset(trb, 0, sizeof(*trb));

	switch (dwc->ep0state) {
	case EP0_IDLE:
		trb->trbctl = DWC3_TRBCTL_CONTROL_SETUP;
		break;

	case EP0_IN_WAIT_NRDY:
	case EP0_OUT_WAIT_NRDY:
	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		if (dwc->three_stage_setup)
			trb->trbctl = DWC3_TRBCTL_CONTROL_STATUS3;
		else
			trb->trbctl = DWC3_TRBCTL_CONTROL_STATUS2;

		if (dwc->ep0state == EP0_IN_WAIT_NRDY)
			dwc->ep0state = EP0_IN_STATUS_PHASE;
		else if (dwc->ep0state == EP0_OUT_WAIT_NRDY)
			dwc->ep0state = EP0_OUT_STATUS_PHASE;

	case EP0_IN_WAIT_GADGET:
		dwc->ep0state = EP0_IN_WAIT_NRDY;
		/*
		 * Not sure what this is about. The reference code does nothing
		 * (except the sate change) and returns with 0
		 */
		WARN_ON(1);
		return 0;
		break;

	case EP0_OUT_WAIT_GADGET:
		dwc->ep0state = EP0_OUT_WAIT_NRDY;
		WARN_ON(1);
		return 0;

		break;

	case EP0_IN_DATA_PHASE:
	case EP0_OUT_DATA_PHASE:
		trb->trbctl = DWC3_TRBCTL_CONTROL_DATA;
		break;

	default:
		dev_err(dwc->dev, "%s() can't in state %d\n", __func__,
				dwc->ep0state);
		return -EINVAL;
	}

	trb->bpl = buf_dma;
	trb->length = len;

	trb->hwo	= 1;
	trb->lst	= 1;
	trb->chn	= 0;
	trb->ioc	= 1;

	dwc->ep0_trb_addr = dma_map_single(dwc->dev, trb, sizeof(*trb),
			DMA_BIDIRECTIONAL);

	memset(&params, 0, sizeof(params));
	params.param1.depstrtxfer.transfer_desc_addr_low = dwc->ep0_trb_addr;

	ret = dwc3_send_gadget_ep_cmd(dwc, dep->number,
			DWC3_DEPCMD_STARTTRANSFER, &params);
	if (ret < 0) {
		dev_dbg(dwc->dev, "failed to send STARTTRANSFER command\n");
		dma_unmap_single(dwc->dev, dwc->ep0_trb_addr, sizeof(*trb),
				DMA_BIDIRECTIONAL);
		return ret;
	}

	dep->res_trans_idx = dwc3_gadget_ep_get_transfer_index(dwc,
			dep->number);

	return 0;
}

static int __dwc3_gadget_ep0_queue(struct dwc3_ep *dep,
		struct dwc3_request *req)
{
	struct dwc3		*dwc = dep->dwc;
	int			ret;

	req->request.actual	= 0;
	req->request.status	= -EINPROGRESS;
	req->direction		= dep->direction;
	req->epnum		= dep->number;

	if (!(dep->number & 1)) {
		/* IS OUT */
		u32 len = req->request.length;

		WARN_ON(len % dep->endpoint.maxpacket);
	}

	dwc3_gadget_add_request(dep, req);
	dwc3_map_buffer_to_dma(req);

	ret = dwc3_ep0_start_trans(dwc, dep->number, req->request.dma,
			req->request.length);
	if (ret < 0) {
		list_del(&req->list);
		dwc3_unmap_buffer_from_dma(req);
	}

	return ret;
}

int dwc3_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *request,
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
	ret = __dwc3_gadget_ep0_queue(dep, req);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static void dwc3_ep0_stall_and_restart(struct dwc3 *dwc)
{
	/* stall is always issued on EP0 */
	__dwc3_gadget_ep_set_halt(dwc->eps[0], 1);
	dwc->eps[0]->flags &= ~DWC3_EP_STALL;
	dwc->ep0state = EP0_IDLE;
	dwc3_ep0_out_start(dwc);
}

void dwc3_ep0_out_start(struct dwc3 *dwc)
{
	struct dwc3_ep			*dep;
	int				ret;

	dep = dwc->eps[0];

	dwc->ctrl_req_addr = dma_map_single(dwc->dev, &dwc->ctrl_req,
			sizeof(dwc->ctrl_req), DMA_FROM_DEVICE);

	ret = dwc3_ep0_start_trans(dwc, 0, dwc->ctrl_req_addr,
			dep->endpoint.maxpacket);
	if (ret < 0) {
		dma_unmap_single(dwc->dev, dwc->ctrl_req_addr,
				sizeof(dwc->ctrl_req), DMA_FROM_DEVICE);
		WARN_ON(1);
	}
}

/*
 * Send a zero length packet for the status phase of the control transfer
 */
static void dwc3_ep0_do_setup_status(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	struct dwc3_ep			*dep;
	int				ret;
	u32				epnum;

	epnum = event->endpoint_number;
	dep = dwc->eps[epnum];

	if (epnum)
		dwc->ep0state = EP0_IN_STATUS_PHASE;
	else
		dwc->ep0state = EP0_OUT_STATUS_PHASE;

	/*
	 * Not sure Why I need a buffer for a zero transfer. Maybe the
	 * HW reacts strange on a NULL pointer
	 */
	dwc->ctrl_req_addr = dma_map_single(dwc->dev, &dwc->ctrl_req,
			sizeof(dwc->ctrl_req), DMA_FROM_DEVICE);

	/* no dma mapping because it should not write at all */
	ret = dwc3_ep0_start_trans(dwc, epnum, virt_to_phys(&dwc->ctrl_req), 0);
	if (ret)
		dwc3_ep0_stall_and_restart(dwc);
}

static void dwc3_ep0_xfernotready(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	switch (dwc->ep0state) {
	case EP0_UNCONNECTED:
		break;
	case EP0_IDLE:
		break;
	case EP0_IN_DATA_PHASE:
		break;
	case EP0_OUT_DATA_PHASE:
		break;
	case EP0_IN_WAIT_GADGET:
		dwc->ep0state = EP0_IN_WAIT_NRDY;
		break;
	case EP0_OUT_WAIT_GADGET:
		dwc->ep0state = EP0_OUT_WAIT_NRDY;
		break;
	case EP0_IN_WAIT_NRDY:
	case EP0_OUT_WAIT_NRDY:
		dwc3_ep0_do_setup_status(dwc, event);
		break;
	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		break;
	case EP0_STALL:
		break;
	}
}

static struct dwc3_ep *dwc3_wIndex_to_dep(struct dwc3 *dwc, __le16 wIndex_le)
{
	struct dwc3_ep		*dep;
	u32			windex = le16_to_cpu(wIndex_le);
	u32			epnum;

	epnum = (windex & USB_ENDPOINT_NUMBER_MASK) << 1;
	if ((windex & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN)
		epnum |= 1;

	dep = dwc->eps[epnum];
	if (dep->flags & DWC3_EP_ENABLED)
		return dep;
	return NULL;
}

static void dwc3_ep0_send_status_response(struct dwc3 *dwc)
{
	u32 epnum;

	if (dwc->ep0state == EP0_IN_DATA_PHASE)
		epnum = 1;
	else
		epnum = 0;

	dwc->ctrl_req_addr = dma_map_single(dwc->dev, &dwc->setup_buf,
			dwc->ep0_usb_req.length, DMA_BIDIRECTIONAL);
	dwc3_ep0_start_trans(dwc, epnum, dwc->ctrl_req_addr,
			dwc->ep0_usb_req.length);
	dwc->ep0_status_pending = 1;
}

/*
 * ch 9.4.5
 */
static int dwc3_ep0_handle_status(struct dwc3 *dwc, struct usb_ctrlrequest *ctrl)
{
	struct dwc3_ep		*dep;
	u32			recip;
	u16			usb_status = 0;
	__le16			*response_pkt;

	recip = ctrl->bRequestType & USB_RECIP_MASK;
	switch (recip) {
	case USB_RECIP_DEVICE:
		/*
		 * We are self-powered. U1/U2/LTM will be set later
		 * once we handle this states. RemoteWakeup is 0 on SS
		 */
		usb_status |= dwc->is_selfpowered << USB_DEVICE_SELF_POWERED;
		break;

	case USB_RECIP_INTERFACE:
		/*
		 * Function Remote Wake Capable	D0
		 * Function Remote Wakeup	D1
		 */
		break;

	case USB_RECIP_ENDPOINT:
		dep = dwc3_wIndex_to_dep(dwc, ctrl->wIndex);
		if (!dep)
		       return -EINVAL;

		if (dep->flags & DWC3_EP_STALL)
			usb_status = 1 << USB_ENDPOINT_HALT;
		break;
	default:
		return -EINVAL;
	};
	response_pkt = (__le16*)&dwc->setup_buf;
	*response_pkt = cpu_to_le16(usb_status);
	dwc->ep0_usb_req.length = sizeof(*response_pkt);
	dwc3_ep0_send_status_response(dwc);
	return 0;
}

static int dwc3_ep0_std_request(struct dwc3 *dwc, struct usb_ctrlrequest *ctrl)
{
	int ret;

	switch (ctrl->bRequest) {
	case USB_REQ_GET_STATUS:
		ret = dwc3_ep0_handle_status(dwc, ctrl);
		break;
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
	case USB_REQ_SET_ADDRESS:
	case USB_REQ_GET_DESCRIPTOR:
	case USB_REQ_SET_DESCRIPTOR:
	case USB_REQ_GET_CONFIGURATION:
	case USB_REQ_SET_CONFIGURATION:
	case USB_REQ_GET_INTERFACE:
	case USB_REQ_SET_INTERFACE:
	case USB_REQ_SYNCH_FRAME:
	default:
		ret = -EINVAL;
		break;
	};

	return ret;
}

static void dwc3_ep0_inspect_setup(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	struct usb_ctrlrequest *ctrl = &dwc->ctrl_req;
	int ret;
	u32 len;

	dma_unmap_single(dwc->dev, dwc->ctrl_req_addr, sizeof(dwc->ctrl_req),
			DMA_FROM_DEVICE);
	dma_unmap_single(dwc->dev, dwc->ep0_trb_addr, sizeof(struct dwc3_trb),
			DMA_BIDIRECTIONAL);

	if (!dwc->gadget_driver)
		goto err;

	len = le16_to_cpu(ctrl->wLength);
	if (!len) {
		dwc->ep0state = EP0_IN_WAIT_GADGET;
		dwc->three_stage_setup = 0;
	} else {
		dwc->three_stage_setup = 1;
		if (ctrl->bRequestType & USB_DIR_IN)
			dwc->ep0state = EP0_IN_DATA_PHASE;
		else
			dwc->ep0state = EP0_OUT_DATA_PHASE;
	}

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		ret = dwc3_ep0_std_request(dwc, ctrl);

	} else {

		spin_unlock(&dwc->lock);
		ret = dwc->gadget_driver->setup(&dwc->gadget, ctrl);
		spin_lock(&dwc->lock);
	}
	if (ret >= 0)
		return;

err:
	dwc3_ep0_stall_and_restart(dwc);
	return;
}

static void dwc3_ep0_complete_data(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	struct dwc3_request	*r;
	struct usb_request	*ur;
	struct dwc3_trb		*trb;
	struct dwc3_ep		*dep;
	u32			transfered;
	u8			epnum;

	epnum = event->endpoint_number;
	dep = dwc->eps[epnum];

	if (!dwc->ep0_status_pending) {
		r = list_first_entry(&dep->request_list, struct dwc3_request, list);
		ur = &r->request;
	} else {
		ur = &dwc->ep0_usb_req;
		dwc->ep0_status_pending = 0;
	}

	trb = &dwc->ep0_trb;
	dma_unmap_single(dwc->dev, dwc->ctrl_req_addr,
			sizeof(dwc->ctrl_req), DMA_BIDIRECTIONAL);
	dma_unmap_single(dwc->dev, dwc->ep0_trb_addr, sizeof(*trb),
			DMA_BIDIRECTIONAL);

	transfered = ur->length - trb->length;
	ur->actual += transfered;

	if ((epnum & 1) && ur->actual < ur->length) {
		/* for some reason we did not get everything out */

		dwc3_ep0_stall_and_restart(dwc);

	} else {
		/*
		 * handle the case where we have to send a zero packet. This
		 * seems to be case when req.length > maxpacket. Could it be?
		 */
		/* The transfer is complete, wait for HOST */
		if (epnum & 1)
			dwc->ep0state = EP0_IN_WAIT_NRDY;
		else
			dwc->ep0state = EP0_OUT_WAIT_NRDY;
	}
}

static void dwc3_ep0_complete_req(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	struct dwc3_request	*r;
	struct dwc3_trb		*trb;
	struct dwc3_ep		*dep;
	u8			epnum;

	epnum = event->endpoint_number;
	dep = dwc->eps[epnum];
	r = list_first_entry(&dep->request_list, struct dwc3_request, list);
	trb = &dwc->ep0_trb;

	list_del(&r->list);
	r->request.status = 0;
	/*
	 * not dropping locks because an enqueue in this callback would
	 * confuse the state engine
	 */
	r->request.complete(&dep->endpoint, &r->request);

	dwc->ep0state = EP0_IDLE;
	dwc3_ep0_out_start(dwc);
}

static void dwc3_ep0_xfer_complete(struct dwc3 *dwc,
			struct dwc3_event_depevt *event)
{
	switch (dwc->ep0state) {
	case EP0_UNCONNECTED:
		break;
	case EP0_IDLE:
		dwc3_ep0_inspect_setup(dwc, event);
		break;

	case EP0_IN_DATA_PHASE:
	case EP0_OUT_DATA_PHASE:
		dwc3_ep0_complete_data(dwc, event);
		break;

	case EP0_IN_WAIT_GADGET:
		break;
	case EP0_OUT_WAIT_GADGET:
		break;

	case EP0_IN_WAIT_NRDY:
		break;
	case EP0_OUT_WAIT_NRDY:
		break;

	case EP0_IN_STATUS_PHASE:
	case EP0_OUT_STATUS_PHASE:
		dwc3_ep0_complete_req(dwc, event);
		break;
	case EP0_STALL:
		break;
	}
}

void dwc3_ep0_interrupt(struct dwc3 *dwc,
		struct dwc3_event_depevt *event)
{
	u8			epnum = event->endpoint_number;

	switch (event->endpoint_event) {
	case DWC3_DEPEVT_XFERCOMPLETE:
		dev_vdbg(dwc->dev, "ep%din Transfer Complete\n", epnum);
		dwc3_ep0_xfer_complete(dwc, event);
		break;

	case DWC3_DEPEVT_XFERINPROGRESS:
		dev_dbg(dwc->dev, "ep%din Transfer In Progress\n", epnum);
		break;

	case DWC3_DEPEVT_XFERNOTREADY:
		dev_dbg(dwc->dev, "ep%din Transfer Not Ready\n", epnum);
		dwc3_ep0_xfernotready(dwc, event);
		break;

	case DWC3_DEPEVT_RXTXFIFOEVT:
		dev_dbg(dwc->dev, "ep%din FIFO Error\n", epnum);
		break;

	case DWC3_DEPEVT_STREAMEVT:
		dev_dbg(dwc->dev, "ep%din Stream Event\n", epnum);
		break;

	case DWC3_DEPEVT_EPCMDCMPLT:
		dev_dbg(dwc->dev, "ep%din Command Complete\n", epnum);
		break;
	}
}
