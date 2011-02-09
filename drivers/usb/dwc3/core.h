/**
 * core.h - DesignWare USB3 DRD Core Header
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

#ifndef __DRIVERS_USB_DWC3_CORE_H
#define __DRIVERS_USB_DWC3_CORE_H

#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

/* Global constants */
#define DWC3_ENDPOINTS_NUM	32

#define DWC3_EVENT_BUFFERS_NUM	2
#define DWC3_EVENT_BUFFERS_SIZE	PAGE_SIZE
#define DWC3_EVENT_TYPE_MASK	0xfe

#define DWC3_EVENT_TYPE_DEV	0
#define DWC3_EVENT_TYPE_CARKIT	3
#define DWC3_EVENT_TYPE_I2C	4

#define DWC3_DEVICE_EVENT_DISCONNECT		0
#define DWC3_DEVICE_EVENT_RESET			1
#define DWC3_DEVICE_EVENT_CONNECT_DONE		2
#define DWC3_DEVICE_EVENT_LINK_STATUS_CHANGE	3
#define DWC3_DEVICE_EVENT_WAKEUP		4
#define DWC3_DEVICE_EVENT_EOPF			6
#define DWC3_DEVICE_EVENT_SOF			7
#define DWC3_DEVICE_EVENT_ERRATIC_ERROR		9
#define DWC3_DEVICE_EVENT_CMD_CMPL		10
#define DWC3_DEVICE_EVENT_OVERFLOW		11

#define DWC3_GEVNTCOUNT_MASK	0xfffc
#define DWC3_GSNPSID_MASK	0xffff0000
#define DWC3_GSNPSREV_MASK	0xffff

/* Global Registers */
#define DWC3_GSBUSCFG0		0xc100
#define DWC3_GSBUSCFG1		0xc104
#define DWC3_GTXTHRCFG		0xc108
#define DWC3_GRXTHRCFG		0xc10c
#define DWC3_GCTL		0xc110
#define DWC3_GEVTEN		0xc114
#define DWC3_GSTS		0xc118
#define DWC3_GSNPSID		0xc120
#define DWC3_GGPIO		0xc124
#define DWC3_GUID		0xc128
#define DWC3_GUCTL		0xc12c
#define DWC3_GBUSERRADDR0	0xc130
#define DWC3_GBUSERRADDR1	0xc134
#define DWC3_GPRTBIMAP0		0xc138
#define DWC3_GPRTBIMAP1		0xc13c
#define DWC3_GHWPARAMS0		0xc140
#define DWC3_GHWPARAMS1		0xc144
#define DWC3_GHWPARAMS2		0xc148
#define DWC3_GHWPARAMS3		0xc14c
#define DWC3_GHWPARAMS4		0xc150
#define DWC3_GHWPARAMS5		0xc154
#define DWC3_GHWPARAMS6		0xc158
#define DWC3_GHWPARAMS7		0xc15c
#define DWC3_GDBGFIFOSPACE	0xc160
#define DWC3_GDBGLTSSM		0xc164
#define DWC3_GPRTBIMAP_HS0	0xc180
#define DWC3_GPRTBIMAP_HS1	0xc184
#define DWC3_GPRTBIMAP_FS0	0xc188
#define DWC3_GPRTBIMAP_FS1	0xc18c

#define DWC3_GUSB2PHYCFG(n)	(0xc200 + (n * 0x04))
#define DWC3_GUSB2I2(n)		(0xc240 + (n * 0x04))

#define DWC3_GUSB2PHYACC(n)	(0xc280 + (n * 0x04))

#define DWC3_GUSB3PIPECTL(n)	(0xc2c0 + (n * 0x04))

#define DWC3_GTXFIFOSIZ(n)	(0xc300 + (n * 0x04))
#define DWC3_GRXFIFOSIZ(n)	(0xc380 + (n * 0x04))

#define DWC3_GEVNTADRLO(n)	(0xc400 + (n * 0x10))
#define DWC3_GEVNTADRHI(n)	(0xc404 + (n * 0x10))
#define DWC3_GEVNTSIZ(n)	(0xc408 + (n * 0x10))
#define DWC3_GEVNTCOUNT(n)	(0xc40c + (n * 0x10))

#define DWC3_GHWPARAMS8		0xc600

/* Device Registers */
#define DWC3_DCFG		0xc700
#define DWC3_DCTL		0xc704
#define DWC3_DEVTEN		0xc708
#define DWC3_DSTS		0xc70c
#define DWC3_DGCMDPAR		0xc710
#define DWC3_DGCMD		0xc714
#define DWC3_DALEPENA		0xc720
#define DWC3_DEPCMDPAR2(n)	(0xc800 + (n * 0x04))
#define DWC3_DEPCMDPAR1(n)	(0xc804 + (n * 0x04))
#define DWC3_DEPCMDPAR0(n)	(0xc808 + (n * 0x04))
#define DWC3_DEPCMD(n)		(0xc80c + (n * 0x04))

/* OTG Registers */
#define DWC3_OCFG		0xcc00
#define DWC3_OCTL		0xcc04
#define DWC3_OEVTEN		0xcc08
#define DWC3_OSTS		0xcc0C

/* Bit fields */

/* Global Configuration Register */
#define DWC3_GCTL_PWRDNSCALE(n)	(n << 19)
#define DWC3_GCTL_DISSCRAMBLE	(1 << 3)

/* Device Configuration Register */
#define DWC3_DCFG_DEVADDR(addr)	((addr) << 3)

#define DWC3_DCFG_SUPERSPEED	(4 << 0)
#define DWC3_DCFG_HIGHSPEED	(0 << 0)
#define DWC3_DCFG_FULLSPEED2	(1 << 0)
#define DWC3_DCFG_LOWSPEED	(2 << 0)
#define DWC3_DCFG_FULLSPEED1	(3 << 0)

/* Device Control Register */
#define DWC3_DCTL_RUN_STOP	(1 << 31)
#define DWC3_DCTL_CSFTRST	(1 << 30)
#define DWC3_DCTL_LSFTRST	(1 << 29)

#define DWC3_DCTL_HIRD_THRES_MASK	(0x1f << 24)
#define DWC3_DCTL_HIRD_THRES(n)	(((n) & DWC3_DCTL_HIRD_THRES_MASK) >> 24)

#define DWC3_DCTL_APPL1RES	(1 << 23)

#define DWC3_DCTL_INITU2ENA	(1 << 12)
#define DWC3_DCTL_ACCEPTU2ENA	(1 << 11)
#define DWC3_DCTL_INITU1ENA	(1 << 10)
#define DWC3_DCTL_ACCEPTU1ENA	(1 << 9)

/* Device Event Enable Register */
#define DWC3_DEVTEN_INACTTIMEOUTRCVEDEN	(1 << 13)
#define DWC3_DEVTEN_VNDRDEVTSTRCVEDEN	(1 << 12)
#define DWC3_DEVTEN_EVNTOVERFLOWEN	(1 << 11)
#define DWC3_DEVTEN_CMDCMPLTEN		(1 << 10)
#define DWC3_DEVTEN_ERRTICERREN		(1 << 9)
#define DWC3_DEVTEN_SOFEN		(1 << 7)
#define DWC3_DEVTEN_EOPFEN		(1 << 6)
#define DWC3_DEVTEN_WKUPEVTEN		(1 << 4)
#define DWC3_DEVTEN_ULSTCNGEN		(1 << 3)
#define DWC3_DEVTEN_CONNECTDONEEN	(1 << 2)
#define DWC3_DEVTEN_USBRSTEN		(1 << 1)
#define DWC3_DEVTEN_DISCONNEVTEN	(1 << 0)

/* Device Status Register */
#define DWC3_DSTS_PWRUPREQ		(1 << 24)
#define DWC3_DSTS_COREIDLE		(1 << 23)
#define DWC3_DSTS_DEVCTRLHLT		(1 << 22)

#define DWC3_DSTS_USBLNKST_MASK		(0x0f << 18)
#define DWC3_DSTS_USBLNKST(n)		(((n) & DWC3_DSTS_USBLNKST_MASK) >> 18)

#define DWC3_DSTS_RXFIFOEMPTY		(1 << 17)

#define DWC3_DSTS_SOFFN_MASK		(0x3ff << 3)
#define DWC3_DSTS_SOFFN(n)		(((n) & DWC3_DSTS_SOFFN_MASK) >> 3)

#define DWC3_DSTS_CONNECTSPD		(7 << 0)

#define DWC3_DSTS_SUPERSPEED		(4 << 0)
#define DWC3_DSTS_HIGHSPEED		(0 << 0)
#define DWC3_DSTS_FULLSPEED2		(1 << 0)
#define DWC3_DSTS_LOWSPEED		(2 << 0)
#define DWC3_DSTS_FULLSPEED1		(3 << 0)

/* Device Generic Command Register */
#define DWC3_DGCMD_SET_LMP		0x01
#define DWC3_DGCMD_SET_PERIODIC_PAR	0x02
#define DWC3_DGCMD_XMIT_FUNCTION	0x03
#define DWC3_DGCMD_SELECTED_FIFO_FLUSH	0x09
#define DWC3_DGCMD_ALL_FIFO_FLUSH	0x0a
#define DWC3_DGCMD_SET_ENDPOINT_NRDY	0x0c
#define DWC3_DGCMD_RUN_SOC_BUS_LOOPBACK	0x10

/* Device Endpoint Command Register */
#define DWC3_DEPCMD_PARAM(x)		(x << 16)
#define DWC3_DEPCMD_HIPRI_FORCERM	(1 << 11)
#define DWC3_DEPCMD_CMDACT		(1 << 10)
#define DWC3_DEPCMD_CMDIOC		(1 << 8)

#define DWC3_DEPCMD_DEPSTARTCFG		(0x09 << 0)
#define DWC3_DEPCMD_ENDTRANSFER		(0x08 << 0)
#define DWC3_DEPCMD_UPDATETRANSFER	(0x07 << 0)
#define DWC3_DEPCMD_STARTTRANSFER	(0x06 << 0)
#define DWC3_DEPCMD_CLEARSTALL		(0x05 << 0)
#define DWC3_DEPCMD_SETSTALL		(0x04 << 0)
#define DWC3_DEPCMD_GETSEQNUMBER	(0x03 << 0)
#define DWC3_DEPCMD_SETTRANSFRESOURCE	(0x02 << 0)
#define DWC3_DEPCMD_SETEPCONFIG		(0x01 << 0)

#define DWC3_DALEPENA_EPOUT(n)		(1 << n)
#define DWC3_DALEPENA_EPIN(n)		(1 << (n + 1))

#define DWC3_DEPCMD_TYPE_CONTROL	0
#define DWC3_DEPCMD_TYPE_ISOC		1
#define DWC3_DEPCMD_TYPE_BULK		2
#define DWC3_DEPCMD_TYPE_INTR		3

/* Structures */

/**
 * struct dwc3_event_buffer - Software event buffer representation
 * @list: a list of event buffers
 * @buf: _THE_ buffer
 * @length: size of this buffer
 * @dma: dma_addr_t
 * @dwc: pointer to DWC controller
 */
struct dwc3_event_buffer {
	void			*buf;
	unsigned		length;
	unsigned int		lpos;

	dma_addr_t		dma;

	struct dwc3		*dwc;
};

#define DWC3_EP_FLAG_STALLED	(1 << 0)
#define DWC3_EP_FLAG_WEDGED	(1 << 1)

#define DWC3_EP_DIRECTION_TX	true
#define DWC3_EP_DIRECTION_RX	false

/**
 * struct dwc3_ep - device side endpoint representation
 * @endpoint: usb endpoint
 * @request_list: list of requests for this endpoint
 * @desc: usb_endpoint_descriptor pointer
 * @dwc: pointer to DWC controller
 * @flags: endpoint flags (wedged, stalled, ...)
 * @number: endpoint number (1 - 15)
 * @type: set to bmAttributes & USB_ENDPOINT_XFERTYPE_MASK
 * @name: a human readable name e.g. ep1out-bulk
 * @direction: true for TX, false for RX
 */
struct dwc3_ep {
	struct usb_ep		endpoint;
	struct list_head	request_list;

	struct usb_endpoint_descriptor *desc;
	struct dwc3		*dwc;

	unsigned		flags;
#define DWC3_EP_ENABLED		(1 << 0)
#define DWC3_EP_STALL		(1 << 1)

	u8			number;
	u8			type;

	char			name[20];

	unsigned		direction:1;
};

enum dwc3_ep0_state {
	EP0_UNCONNECTED,
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_IN_WAIT_GADGET,
	EP0_OUT_WAIT_GADGET,
	EP0_IN_WAIT_NRDY,
	EP0_OUT_WAIT_NRDY,
	EP0_IN_STATUS_PHASE,
	EP0_OUT_STATUS_PHASE,
	EP0_STALL,
};

enum dwc3_link_state {
	/* In SuperSpeed */
	DWC3_LINK_STATE_U0		= 0x00, /* in HS, means ON */
	DWC3_LINK_STATE_U1		= 0x01,
	DWC3_LINK_STATE_U2		= 0x02, /* in HS, means SLEEP */
	DWC3_LINK_STATE_U3		= 0x03, /* in HS, means SUSPEND */
	DWC3_LINK_STATE_SS_DIS		= 0x04,
	DWC3_LINK_STATE_RX_DET		= 0x05, /* in HS, means Early Suspend */
	DWC3_LINK_STATE_SS_INACT	= 0x06,
	DWC3_LINK_STATE_POLL		= 0x07,
	DWC3_LINK_STATE_RECOV		= 0x08,
	DWC3_LINK_STATE_HRESET		= 0x09,
	DWC3_LINK_STATE_CMPLY		= 0x0a,
	DWC3_LINK_STATE_LPBK		= 0x0b,
};

/**
 * struct dwc3 - representation of our controller
 * @lock: for synchronizing
 * @dev: pointer to our struct device
 * @event_buffer_list: a list of event buffers
 * @gadget: device side representation of the peripheral controller
 * @gadget_driver: pointer to the gadget driver
 * @xhci: xHCI memory base
 * @global: global registers
 * @device: device registers
 * @otg: OTG registers
 * @ram0: for debug purposes, access to internal RAM
 * @ram1: for debug purposes, access to internal RAM
 * @ram2: for debug purposes, access to internal RAM
 * @irq: IRQ number
 * @revision: revision register contents
 * @is_selfpowered: true when we are selfpowered
 * @ep0state: state of endpoint zero
 * @link_state: link state
 * @speed: device speed (super, high, full, low)
 */
struct dwc3 {
	/* device lock */
	spinlock_t		lock;
	struct device		*dev;

	struct dwc3_event_buffer *ev_buffs[DWC3_EVENT_BUFFERS_NUM];
	struct dwc3_ep		*eps[DWC3_ENDPOINTS_NUM];

	struct usb_gadget	gadget;
	struct usb_gadget_driver *gadget_driver;

	void __iomem		*xhci;
	void __iomem		*global;
	void __iomem		*device;
	void __iomem		*otg;
	void __iomem		*ram0;
	void __iomem		*ram1;
	void __iomem		*ram2;

	int			irq;

	u32			revision;

	unsigned		is_selfpowered:1;

	enum dwc3_ep0_state	ep0state;
	enum dwc3_link_state	link_state;

	u8			speed;
};

/* -------------------------------------------------------------------------- */

#define DWC3_TRBSTS_OK			0
#define DWC3_TRBSTS_MISSED_ISOC		1
#define DWC3_TRBSTS_SETUP_PENDING	2

#define DWC3_TRBCTL_NORMAL		1
#define DWC3_TRBCTL_CONTROL_SETUP	2
#define DWC3_TRBCTL_CONTROL_STATUS2	3
#define DWC3_TRBCTL_CONTROL_STATUS3	4
#define DWC3_TRBCTL_CONTROL_DATA	5
#define DWC3_TRBCTL_ISOCHRONOUS_FIRST	6
#define DWC3_TRBCTL_ISOCHRONOUS		7
#define DWC3_TRBCTL_LINK_TRB		8

/**
 * struct dwc3_trb - transfer request block
 * @dma: up to 64 bit addressing
 * @length: buffer size (up to 16mb - 1)
 * @pcm1: packet count m1
 * @trbsts: trb status
 *	0 = ok
 *	1 = missed isoc
 *	2 = setup pending
 * @hwo: hardware owner of descriptor
 * @lst: last trm
 * @chn: chain buffers
 * @csp: continue on short packets (only supported on isoc eps)
 * @trbctl: trb control
 *	1 = normal
 *	2 = control-setup
 *	3 = control-status-2
 *	4 = control-status-3
 *	5 = control-data (first trb of data stage)
 *	6 = isochronous-first (first trb of service interval)
 *	7 = isochronous
 *	8 = link trb
 *	others = reserved
 * @isp_imi: interrupt on short packet / interrupt on missed isoc
 * @ioc: interrupt on complete
 * @sid_sofn: Stream ID / SOF Number
 */
struct dwc3_trb {
	dma_addr_t		dma;
	unsigned		length:24;
	unsigned		pcm1:2;
	unsigned		reserved27_26:2;
	unsigned		trbsts:4;

	unsigned		hwo:1;
	unsigned		lst:1;
	unsigned		chn:1;
	unsigned		csp:1;
	unsigned		trbctl:6;
	unsigned		isp_imi:1;
	unsigned		ioc:1;
	unsigned		reserved13_12:2;
	u16			sid_sofn;
	unsigned		reserved31_30:2;

} __attribute__ ((packed));

/* -------------------------------------------------------------------------- */

struct dwc3_event_type {
	unsigned	is_devspec:1;
	unsigned	type:6;
	unsigned	reserved8_31:25;
} __attribute__ ((packed));

#define DWC3_DEPEVT_XFERCOMPLETE	0x01
#define DWC3_DEPEVT_XFERINPROGRESS	0x02
#define DWC3_DEPEVT_XFERNOTREADY	0x03
#define DWC3_DEPEVT_RXTXFIFOEVT		0x04
#define DWC3_DEPEVT_STREAMEVT		0x06
#define DWC3_DEPEVT_EPCMDCMPLT		0x07

/**
 * struct dwc3_event_depvt - Device Endpoint Events
 * @one_bit: indicates this is an endpoint event (not used)
 * @endpoint_number: number of the endpoint
 * @endpoint_event: The event we have:
 *	0x00	- Reserved
 *	0x01	- XferComplete
 *	0x02	- XferInProgress
 *	0x03	- XferNotReady
 *	0x04	- RxTxFifoEvt (IN->Underrun, OUT->Overrun)
 *	0x05	- Reserved
 *	0x06	- StreamEvt
 *	0x07	- EPCmdCmplt
 * @reserved11_10: Reserved, don't use.
 * @event_status: Indicates the status of the event. Refer to databook for
 *	more information.
 * @event_parameters: Parameters of the current event. Refer to databook for
 *	more information.
 */
struct dwc3_event_depevt {
	unsigned	one_bit:1;
	unsigned	endpoint_number:5;
	unsigned	endpoint_event:4;
	unsigned	reserved11_10:2;
	unsigned	event_status:4;
	u16		event_parameters;
} __attribute__ ((packed));

/**
 * struct dwc3_event_devt - Device Events
 * @one_bit: indicates this is a non-endpoint event (not used)
 * @device_event: indicates it's a device event. Should read as 0x00
 * @type: indicates the type of device event.
 *	0	- DisconnEvt
 *	1	- USBRst
 *	2	- ConnectDone
 *	3	- ULStChng
 *	4	- WkUpEvt
 *	5	- Reserved
 *	6	- EOPF
 *	7	- SOF
 *	8	- Reserved
 *	9	- ErrticErr
 *	10	- CmdCmplt
 *	11	- EvntOverflow
 *	12	- VndrDevTstRcved
 * @reserved15_12: Reserved, not used
 * @event_info: Information about this event
 * @reserved31_24: Reserved, not used
 */
struct dwc3_event_devt {
	unsigned	one_bit:1;
	unsigned	device_event:7;
	unsigned	type:4;
	unsigned	reserved15_12:4;
	u8		event_info;
	u8		reserved31_24;
} __attribute__ ((packed));

/**
 * struct dwc3_event_gevt - Other Core Events
 * @one_bit: indicates this is a non-endpoint event (not used)
 * @device_event: indicates it's (0x03) Carkit or (0x04) I2C event.
 * @phy_port_number: self-explanatory
 * @reserved31_12: Reserved, not used.
 */
struct dwc3_event_gevt {
	unsigned	one_bit:1;
	unsigned	device_event:7;
	unsigned	phy_port_number:4;
	unsigned	reserved31_12:20;
} __attribute__ ((packed));

/**
 * union dwc3_event - representation of Event Buffer contents
 * @raw: raw 32-bit event
 * @type: the type of the event
 * @depevt: Device Endpoint Event
 * @devt: Device Event
 * @gevt: Global Event
 */
union dwc3_event {
	u32				raw;
	struct dwc3_event_type		type;
	struct dwc3_event_depevt	depevt;
	struct dwc3_event_devt		devt;
	struct dwc3_event_gevt		gevt;
};

#endif /* __DRIVERS_USB_DWC3_CORE_H */
