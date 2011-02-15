/**
 * gadget.h - DesignWare USB3 DRD Gadget Header
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

#ifndef __DRIVERS_USB_DWC3_GADGET_H
#define __DRIVERS_USB_DWC3_GADGET_H

#include <linux/list.h>

#include <linux/usb/gadget.h>

struct dwc3;

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

	unsigned		direction:1;
	unsigned		mapped:1;
};
#define to_dwc3_request(r)	(container_of(r, struct dwc3_request, request))

int dwc3_gadget_init(struct dwc3 *dwc);
void dwc3_gadget_exit(struct dwc3 *dwc);

#endif /* __DRIVERS_USB_DWC3_GADGET_H */
