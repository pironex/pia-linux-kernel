/**
 * gadget.h - DesignWare USB3 DRD Gadget Header
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

#ifndef __DRIVERS_USB_DWC3_GADGET_H
#define __DRIVERS_USB_DWC3_GADGET_H

#include <linux/list.h>
#include <linux/usb/gadget.h>
#include "io.h"

struct dwc3;
#define to_dwc3_ep(ep)		(container_of(ep, struct dwc3_ep, endpoint))
#define gadget_to_dwc(g)	(container_of(g, struct dwc3, gadget))

/**
 * struct dwc3_gadget_ep_depcfg_param1 - DEPCMDPAR0 for DEPCFG command
 * @interrupt_number: self-explanatory
 * @reserved7_5: set to zero
 * @xfer_complete_enable: event generated when transfer completed
 * @xfer_in_progress_enable: event generated when transfer in progress
 * @xfer_not_ready_enable: event generated when transfer not read
 * @fifo_error_enable: generates events when FIFO Underrun (IN eps)
 *	or FIFO Overrun (OUT) eps
 * @reserved_12: set to zero
 * @stream_event_enable: event generated on stream
 * @reserved14_15: set to zero
 * @binterval_m1: bInterval minus 1
 * @stream_capable: this EP is capable of handling streams
 * @ep_number: self-explanatory
 * @bulk_based: Set to ‘1’ if this isochronous endpoint represents a bulk
 *	data stream that ignores the relationship of bus time to the
 *	intervals programmed in TRBs.
 * @fifo_based: Set to ‘1’ if this isochronous endpoint represents a
 *	FIFO-based data stream where TRBs have fixed values and are never
 *	written back by the core.
 */
struct dwc3_gadget_ep_depcfg_param1 {
	unsigned	interrupt_number:5;
	unsigned	reserved7_5:3;		/* set to zero */
	unsigned	xfer_complete_enable:1;
	unsigned	xfer_in_progress_enable:1;
	unsigned	xfer_not_ready_enable:1;
	unsigned	fifo_error_enable:1;	/* IN-underrun, OUT-overrun */
	unsigned	reserved12:1;		/* set to zero */
	unsigned	stream_event_enable:1;
	unsigned	reserved14_15:2;
	u8		binterval_m1;		/* bInterval minus 1 */
	unsigned	stream_capable:1;
	unsigned	ep_number:5;
	unsigned	bulk_based:1;
	unsigned	fifo_based:1;
} __packed;

/**
 * struct dwc3_gadget_ep_depcfg_param0 - Parameter 0 for DEPCFG
 * @reserved0: set to zero
 * @ep_type: Endpoint Type (control, bulk, iso, interrupt)
 * @max_packet_size: max packet size in bytes
 * @reserved16_14: set to zero
 * @fifo_number: self-explanatory
 * @burst_size: burst size minus 1
 * @data_sequence_number: Must be 0 when an endpoint is initially configured
 *	May be non-zero when an endpoint is configured after a power transition
 *	that requires a save/restore.
 * @ignore_sequence_number: Set to ‘1’ to avoid resetting the sequence
 *	number. This setting is used by software to modify the DEPEVTEN
 *	event enable bits without modifying other endpoint settings.
 */
struct dwc3_gadget_ep_depcfg_param0 {
	unsigned	reserved0:1;
	unsigned	ep_type:2;
	unsigned	max_packet_size:11;
	unsigned	reserved16_14:3;
	unsigned	fifo_number:5;
	unsigned	burst_size:4;
	unsigned	data_sequence_number:5;
	unsigned	ignore_sequence_number:1;
} __packed;

/**
 * struct dwc3_gadget_ep_depxfercfg_param0 - Parameter 0 of DEPXFERCFG
 * @number_xfer_resources: Defines the number of Transfer Resources allocated
 *	to this endpoint.  This field must be set to 1.
 * @reserved16_31: set to zero;
 */
struct dwc3_gadget_ep_depxfercfg_param0 {
	u16		number_xfer_resources;
	u16		reserved16_31;
} __packed;

/**
 * struct dwc3_gadget_ep_depstrtxfer_param1 - Parameter 1 of DEPSTRTXFER
 * @transfer_desc_addr_low: Indicates the lower 32 bits of the external
 *	memory's start address for the transfer descriptor. Because TRBs
 *	must be aligned to a 16-byte boundary, the lower 4 bits of this
 *	address must be 0.
 */
struct dwc3_gadget_ep_depstrtxfer_param1 {
	u32		transfer_desc_addr_low;
} __packed;

/**
 * struct dwc3_gadget_ep_depstrtxfer_param1 - Parameter 1 of DEPSTRTXFER
 * @transfer_desc_addr_high: Indicates the higher 32 bits of the external
 *	memory’s start address for the transfer descriptor.
 */
struct dwc3_gadget_ep_depstrtxfer_param0 {
	u32		transfer_desc_addr_high;
} __packed;

struct dwc3_gadget_ep_cmd_params {
	union {
		u32	raw;
	} param2;

	union {
		u32	raw;
		struct dwc3_gadget_ep_depcfg_param1 depcfg;
		struct dwc3_gadget_ep_depstrtxfer_param1 depstrtxfer;
	} param1;

	union {
		u32	raw;
		struct dwc3_gadget_ep_depcfg_param0 depcfg;
		struct dwc3_gadget_ep_depxfercfg_param0 depxfercfg;
		struct dwc3_gadget_ep_depstrtxfer_param0 depstrtxfer;
	} param0;
} __packed;

/* -------------------------------------------------------------------------- */

struct dwc3_request {
	struct usb_request	request;
	struct list_head	list;
	struct dwc3_ep		*dep;

	u8			epnum;
	struct dwc3_trb		*trb;
	dma_addr_t		trb_dma;

	unsigned		direction:1;
	unsigned		mapped:1;
	unsigned		queued:1;
};
#define to_dwc3_request(r)	(container_of(r, struct dwc3_request, request))

static inline struct dwc3_request *next_request(struct dwc3_ep *dep)
{
	struct list_head	*list = &dep->request_list;

	if (list_empty(list))
		return NULL;

	return list_first_entry(list, struct dwc3_request, list);
}

static inline struct dwc3_request *next_queued_request(struct dwc3_ep *dep)
{
	struct list_head	*list = &dep->req_queued;

	if (list_empty(list))
		return NULL;

	return list_first_entry(list, struct dwc3_request, list);
}

static inline void dwc3_gadget_add_request(struct dwc3_ep *dep,
		struct dwc3_request *req)
{
	list_add_tail(&req->list, &dep->request_list);
	dep->request_count++;
}

static inline void dwc3_gadget_del_request(struct dwc3_request *req)
{
	struct dwc3_ep		*dep = req->dep;

	dep->request_count--;
	list_del(&req->list);
}

static inline void dwc3_gadget_move_request_queued(struct dwc3_request *req)
{
	struct dwc3_ep		*dep = req->dep;

	dep->request_count--;
	list_move_tail(&req->list, &dep->req_queued);
}

#ifdef CONFIG_USB_GADGET_DWC3
int dwc3_gadget_init(struct dwc3 *dwc);
void dwc3_gadget_exit(struct dwc3 *dwc);
#else
static inline int dwc3_gadget_init(struct dwc3 *dwc) { return 0; }
static inline void dwc3_gadget_exit(struct dwc3 *dwc) { }
#endif

void dwc3_ep0_interrupt(struct dwc3 *dwc, struct dwc3_event_depevt *event);
void dwc3_ep0_out_start(struct dwc3 *dwc);
int dwc3_gadget_ep0_queue(struct usb_ep *ep, struct usb_request *request,
		gfp_t gfp_flags);
int __dwc3_gadget_ep_set_halt(struct dwc3_ep *dep, int value);
int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, struct dwc3_gadget_ep_cmd_params *params);
void dwc3_map_buffer_to_dma(struct dwc3_request *req);
void dwc3_unmap_buffer_from_dma(struct dwc3_request *req);

/**
 * dwc3_gadget_ep_get_transfer_index - Gets transfer index from HW
 * @dwc: DesignWare USB3 Pointer
 * @number: DWC endpoint number
 *
 * Caller should take care of locking
 */
static inline u32 dwc3_gadget_ep_get_transfer_index(struct dwc3 *dwc, u8 number)
{
	u32			res_id;

	res_id = dwc3_readl(dwc->device, DWC3_DEPCMD(number));

	return DWC3_DEPCMD_GET_RSC_IDX(res_id);
}

#endif /* __DRIVERS_USB_DWC3_GADGET_H */
