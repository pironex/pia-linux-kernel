/*
 * LCD panel driver for DEM 480272D TMH-PW-N
 * based on Sharp LQ043T1DG01 driver
 *
 * Copyright (C) 2012 pironex GmbH
 *               2009 Texas Instruments Inc
 * Author: Bjoern Krombhol <b.krombholz@pironex.de>
 *         Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>

#include <plat/display.h>

static struct omap_video_timings dem_timings = {
	.x_res = 480,
	.y_res = 272,

	.pixel_clock	= 9000,
#if 0 // closes to datasheet, works but slight flickering
	.hsw		= 2,
	.hfp		= 8,
	.hbp		= 43,

	.vsw		= 10,
	.vfp		= 5,
	.vbp		= 12,
#else
	.hsw		= 3, // reduces flickering
	.hfp		= 8,
	.hbp		= 40,

	.vsw		= 11, // reduce flickering
	.vfp		= 4,
	.vbp		= 1,
#endif
};

static int dem_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void dem_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */
	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int dem_panel_probe(struct omap_dss_device *dssdev)
{

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IEO |
		OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF; /* HS/VS on rising edge */
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = dem_timings;

	return 0;
}

static void dem_panel_remove(struct omap_dss_device *dssdev)
{
}

static int dem_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = dem_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void dem_panel_disable(struct omap_dss_device *dssdev)
{
	dem_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int dem_panel_suspend(struct omap_dss_device *dssdev)
{
	dem_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int dem_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = dem_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static struct omap_dss_driver dem_driver = {
	.probe		= dem_panel_probe,
	.remove		= dem_panel_remove,

	.enable		= dem_panel_enable,
	.disable	= dem_panel_disable,
	.suspend	= dem_panel_suspend,
	.resume		= dem_panel_resume,

	.driver         = {
		.name   = "dem_480272d_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init dem_panel_drv_init(void)
{
	return omap_dss_register_driver(&dem_driver);
}

static void __exit dem_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&dem_driver);
}

module_init(dem_panel_drv_init);
module_exit(dem_panel_drv_exit);
MODULE_LICENSE("GPL");
