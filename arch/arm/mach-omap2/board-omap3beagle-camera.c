/*
 * Driver for Leopard Module Board used in Beagleboard (xM)
 *
 * Copyright (C) 2010 Texas Instruments Inc
 * Author: Sergio Aguirre <saaguirre@ti.com>
 *
 * Based on work done by:
 *     Vaibhav Hiremath <hvaibhav@ti.com>
 *     Anuj Aggarwal <anuj.aggarwal@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/videodev2.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#include <plat/mux.h>
#include <plat/board.h>

#include <media/v4l2-int-device.h>
#include <media/mt9t111.h>

/* Include V4L2 ISP-Camera driver related header file */
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#include "mux.h"
#include "board-omap3beagle-camera.h"

#define MODULE_NAME			"omap3beaglelmb"

#define MT9T111_I2C_BUSNUM	(2)

#define CAM_USE_XCLKA       1

static struct regulator *beagle_mt9t111_reg1;
static struct regulator *beagle_mt9t111_reg2;

static struct device *beaglecam_dev;

#if defined(CONFIG_VIDEO_MT9T111) || defined(CONFIG_VIDEO_MT9T111_MODULE)
static struct isp_interface_config mt9t111_if_config = {
	.ccdc_par_ser		= ISP_PARLL, 
	.dataline_shift		= 0x0,
	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe			= 0x0,
	.prestrobe		= 0x0,
	.shutter		= 0x0,
	.u.par.par_bridge	= 0x1,
	.u.par.par_clk_pol	= 0x0,
};

static struct v4l2_ifparm mt9t111_ifparm_s = {
#if 1
	.if_type = V4L2_IF_TYPE_RAW, 
	.u 	 = {
		.raw = {  
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct	= 0,
			.swap			= 0,
			.latch_clk_inv		= 0,
			.nobt_hs_inv		= 0,	/* active high */
			.nobt_vs_inv		= 0,	/* active high */
			.clock_min		= MT9T111_CLK_MIN,
			.clock_max		= MT9T111_CLK_MAX,
		},
	},
#else		
	.if_type = V4L2_IF_TYPE_YCbCr, 
	.u 	 = {
		.ycbcr = {  
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct	= 0,
			.swap			= 0,
			.latch_clk_inv		= 0,
			.nobt_hs_inv		= 0,	/* active high */
			.nobt_vs_inv		= 0,	/* active high */
			.clock_min		= MT9T111_CLK_MIN,
			.clock_max		= MT9T111_CLK_MAX,
		},
	},
#endif
};

/**
 * @brief mt9t111_ifparm - Returns the mt9t111 interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int mt9t111_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = mt9t111_ifparm_s;
	return 0;
}

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config mt9t111_hwc = {
	.dev_index		= 0,
	.dev_minor		= 0,
	.dev_type		= OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.sensor_isp	= 1,
};
#endif

/**
 * @brief mt9t111_set_prv_data - Returns mt9t111 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int mt9t111_set_prv_data(void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	hwc->u.sensor = mt9t111_hwc.u.sensor;
	hwc->dev_index = mt9t111_hwc.dev_index;
	hwc->dev_minor = mt9t111_hwc.dev_minor;
	hwc->dev_type = mt9t111_hwc.dev_type;
	return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief mt9t111_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int mt9t111_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	switch (power) {
	case V4L2_POWER_OFF:
	case V4L2_POWER_STANDBY:
		isp_set_xclk(vdev->cam->isp, 0, CAM_USE_XCLKA);

		if (regulator_is_enabled(beagle_mt9t111_reg1))
			regulator_disable(beagle_mt9t111_reg1);
		if (regulator_is_enabled(beagle_mt9t111_reg2))
			regulator_disable(beagle_mt9t111_reg2);
		break;

	case V4L2_POWER_ON:
		isp_set_xclk(vdev->cam->isp, MT9T111_CLK_MIN, CAM_USE_XCLKA);

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
		isp_configure_interface(vdev->cam->isp, &mt9t111_if_config);
#endif

		/* turn on analog power */
		regulator_enable(beagle_mt9t111_reg1);
		regulator_enable(beagle_mt9t111_reg2);
		udelay(100);

		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

static struct mt9t111_platform_data mt9t111_pdata = {
	.master		= "omap34xxcam",
	.power_set	= mt9t111_power_set,
	.priv_data_set	= mt9t111_set_prv_data,
	.ifparm		= mt9t111_ifparm,
	/* Some interface dependent params */
	.clk_polarity	= 0, /* data clocked out on falling edge */
	.hs_polarity	= 1, /* 0 - Active low, 1- Active high */
	.vs_polarity	= 1, /* 0 - Active low, 1- Active high */
};

static struct i2c_board_info __initdata mt9t111_i2c_board_info = {
	I2C_BOARD_INFO("mt9t111", MT9T111_I2C_ADDR),
	.platform_data	= &mt9t111_pdata,
};

#endif				/* #ifdef CONFIG_VIDEO_MT9T111 */


static int beagle_cam_probe(struct platform_device *pdev)
{
	int err;

	beagle_mt9t111_reg1 = regulator_get(beaglecam_dev, "vaux3_1");
	if (IS_ERR(beagle_mt9t111_reg1)) {
		dev_err(beaglecam_dev, "vaux3_1 regulator missing\n");
		return PTR_ERR(beagle_mt9t111_reg1);
	}
	beagle_mt9t111_reg2 = regulator_get(beaglecam_dev, "vaux4_1");
	if (IS_ERR(beagle_mt9t111_reg2)) {
		dev_err(beaglecam_dev, "vaux4_1 regulator missing\n");
		regulator_put(beagle_mt9t111_reg1);
		return PTR_ERR(beagle_mt9t111_reg2);
	}
	/*
	 * Register the I2C devices present in the board to the I2C
	 * framework.
	 * If more I2C devices are added, then each device information should
	 * be registered with I2C using i2c_register_board_info().
	 */
#if defined(CONFIG_VIDEO_MT9T111) || defined(CONFIG_VIDEO_MT9T111_MODULE)
	err = i2c_register_board_info(MT9T111_I2C_BUSNUM,
					&mt9t111_i2c_board_info, 1);
	if (err) {
		printk(KERN_ERR MODULE_NAME \
				": MT9T111 I2C Board Registration failed \n");
		return err;
	}
#endif

	beaglecam_dev = &pdev->dev;

	printk(KERN_INFO MODULE_NAME ": Driver registration complete \n");

	return 0;
}

static int beagle_cam_remove(struct platform_device *pdev)
{
	if (regulator_is_enabled(beagle_mt9t111_reg1))
		regulator_disable(beagle_mt9t111_reg1);
	regulator_put(beagle_mt9t111_reg1);
	if (regulator_is_enabled(beagle_mt9t111_reg2))
		regulator_disable(beagle_mt9t111_reg2);
	regulator_put(beagle_mt9t111_reg2);

	return 0;
}

static int beagle_cam_suspend(struct device *dev)
{
	return 0;
}

static int beagle_cam_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops beagle_cam_pm_ops = {
	.suspend = beagle_cam_suspend,
	.resume  = beagle_cam_resume,
};

static struct platform_driver beagle_cam_driver = {
	.probe		= beagle_cam_probe,
	.remove		= beagle_cam_remove,
	.driver		= {
		.name	= "beagle_cam",
		.pm	= &beagle_cam_pm_ops,
	},
};

/**
 * @brief omap3beaglelmb_init - module init function. Should be called before any
 *                          client driver init call
 *
 * @return result of operation - 0 is success
 */
int __init omap3beaglelmb_init(void)
{
	platform_driver_register(&beagle_cam_driver);
	return 0;
}
arch_initcall(omap3beaglelmb_init);
