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
#include "io.h"

static int dwc3_send_gadget_ep_cmd(struct dwc3 *dwc, unsigned ep,
		unsigned cmd, unsigned params)
{
	unsigned long		timeout = jiffies + msecs_to_jiffies(500);
	u32			reg;

	reg = (params << 16) | (cmd << 0);

	dwc3_writel(dwc->device, DWC3_DEPCMD(ep), reg);
	do {
		reg = dwc3_readl(dwc->device, DWC3_DEPCMD(0));
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (!(reg & DWC3_DEPCMD_CMDACT));

	return 0;
}

/**
 * dwc3_gadget_init - Initializes gadget related registers
 * @dwc: Pointer to out controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
int __devinit dwc3_gadget_init(struct dwc3 *dwc)
{
	u32			params;
	u32			reg;

	int			ret;

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

	ret = dwc3_send_gadget_ep_cmd(dwc, 0, DWC3_DEPCMD_DEPSTARTCFG, 0);
	if (ret)
		return ret;

	/*
	 * FIXME params should contain the following:
	 *
	 * USB Endpoint Number = 0 or 1 (for physical endpoint 0 or 1)
	 * FIFONum= 0
	 * XferNRdyEn and XferCmplEn = 1
	 * Maximum Packet Size = 512
	 * Burst Size = 0
	 * EPType = 2’b00 (Control)
	 *
	 * Now, how to set those ?? Documentation doesn't fully explain.
	 */
	params = 0x00;

	ret = dwc3_send_gadget_ep_cmd(dwc, 0, DWC3_DEPCMD_SETEPCONFIG, params);
	if (ret)
		return ret;

	ret = dwc3_send_gadget_ep_cmd(dwc, 1, DWC3_DEPCMD_SETEPCONFIG, params);
	if (ret)
		return ret;

	/* set DEPCMD_PAR0(0) to 1 */
	ret = dwc3_send_gadget_ep_cmd(dwc, 0,
			DWC3_DEPCMD_SETTRANSFRESOURCE, 0x00);
	if (ret)
		return ret;

	/* set DEPCMD_PAR0(1) to 1 */
	ret = dwc3_send_gadget_ep_cmd(dwc, 1,
			DWC3_DEPCMD_SETTRANSFRESOURCE, 0x00);
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
	dwc3_writel(dwc->device, DWC3_DALEPENA, 0x03);

	/* Set RUN/STOP bit */
	reg = dwc3_readl(dwc->device, DWC3_DCTL);
	reg |= DWC3_DCTL_RUN_STOP;
	dwc3_writel(dwc->device, DWC3_DCTL, reg);

	return 0;
}

