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

#define to_dwc_ep(ep)		(container_of(ep, struct dwc_ep, endpoint))

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

const struct usb_ep_ops dwc_gadget_ep0_ops = {
	.enable		= dwc_gadget_ep0_enable,
	.disable	= dwc_gadget_ep0_disable,
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

const struct usb_ep_ops dwc_gadget_ep_ops = {
	.enable		= dwc_gadget_ep_enable,
	.disable	= dwc_gadget_ep_disable,
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

