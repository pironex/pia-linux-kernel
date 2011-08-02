/*
 * Support for AM3517/05 Craneboard
 * http://www.mistralsolutions.com/products/craneboard.php
 *
 * Copyright (C) 2010 Mistral Solutions Pvt Ltd. <www.mistralsolutions.com>
 * Author: R.Srinath <srinath@mistralsolutions.com>
 *
 * Based on mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/mfd/tps6507x.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6507x.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"

#define GPIO_USB_POWER		35
#define GPIO_USB_NRESET		38


/* Board initialization */
static struct omap_board_config_kernel am3517_crane_config[] __initdata = {
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = GPIO_USB_NRESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

/*
 * MMC
 */
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6507x.h>

/* MMC1 has fixed power supply */
static struct regulator_consumer_supply pia35x_vmmc1_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0");

static struct regulator_init_data pia35x_vmmc1_data = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &pia35x_vmmc1_supply,
};

static struct fixed_voltage_config pia35x_vmmc1_config = {
	.supply_name     = "vmmc",
	.microvolts      = 3300000,  /* 3.3V */
	//.gpio          = OMAP_BEAGLE_WLAN_EN_GPIO,
	.gpio            = -EINVAL,
	.startup_delay   = 70000, /* 70ms */
	.enable_high     = 1,
	.enabled_at_boot = 1,
	.init_data       = &pia35x_vmmc1_data,
};

static struct platform_device pia35x_vmmc1_device = {
	.name           = "reg-fixed-voltage",
	.id             = 0,
	.dev = {
		.platform_data = &pia35x_vmmc1_config,
	},
};


/*
 * Voltage Regulator
 */
static struct tps6507x_reg_platform_data pia35x_tps_vdd2_platform_data = {
		.defdcdc_default = true,
};

static struct tps6507x_reg_platform_data pia35x_tps_vdd3_platform_data = {
		.defdcdc_default = false,
};

static struct regulator_consumer_supply pia35x_vdd1_consumers[] = {
	{
		.supply = "vdds",
	},
};

static struct regulator_consumer_supply pia35x_vdd2_consumers[] = {
	{
		.supply = "vddshv",
	},
//	{
//		.supply         = "vmmc",
//		.dev_name       = "mmci-omap-hs.0", /* bind to our MMC1 device */
//	},
};

static struct regulator_consumer_supply pia35x_vdd3_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

static struct regulator_consumer_supply pia35x_ldo1_consumers[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
};

static struct regulator_consumer_supply pia35x_vpll_consumers[] = {
	{
		.supply = "vdds_dpll_mpu_usbhost",
	},
	{
		.supply = "vdds_dpll_per_core",
	},
	{
		.supply = "vdd_sram_mpu",
	},
	{
		.supply = "vdd_sram_core_bg0",
	},
	{
		.supply = "vddsosc",
	},
};

/* regulator_init_data for the outputs are organized in an array
 * [0]: VDCDC1  1.8V VDDS_1V8
 * [1]: VDCDC2  3.3V VDDSHV_3V3
 * [2]: VDCDC3  1.2V VDDCORE_1V2     (cpu core)
 * [3]: VLDO1   1.8V VDDA1P8V_USBPHY (usb)
 * [4]: VLDO2   1.8V VDDS_DPLL_1V8   (pll)
 */
static struct regulator_init_data pia35x_tps_regulator_data[] = {
		/* dcdc: VDDS_1V8*/
		{
				.constraints = {
						.min_uV = 1800000,
						.max_uV = 1800000,
						.valid_modes_mask = REGULATOR_MODE_NORMAL,
						.valid_ops_mask = REGULATOR_CHANGE_STATUS,
						.always_on = true,
						.apply_uV = true,
				},
				.num_consumer_supplies = ARRAY_SIZE(pia35x_vdd1_consumers),
				.consumer_supplies     = &pia35x_vdd1_consumers[0],
		},
		/* dcdc2: VDDSHV_3V3 */
		{
				.constraints = {
						.min_uV = 3300000,
						.max_uV = 3300000,
						.valid_modes_mask = REGULATOR_MODE_NORMAL,
						.valid_ops_mask = REGULATOR_CHANGE_STATUS,
						.always_on = true,
						.apply_uV = true
				},
				.num_consumer_supplies = ARRAY_SIZE(pia35x_vdd2_consumers),
				.consumer_supplies = &pia35x_vdd2_consumers[0],
				/* select high = 3.3V (low is 1.8) */
				.driver_data = &pia35x_tps_vdd2_platform_data,
		},
		/* dcdc3: VDDCORE_1V2 */
		{
				.constraints = {
						.min_uV = 1200000,
						.max_uV = 1200000,
						.valid_modes_mask = REGULATOR_MODE_NORMAL,
						.valid_ops_mask = REGULATOR_CHANGE_STATUS,
						.always_on = true,
						.apply_uV = true
				},
				.num_consumer_supplies = ARRAY_SIZE(pia35x_vdd3_consumers),
				.consumer_supplies = &pia35x_vdd3_consumers[0],
				/* select low = 1.2V (high is 1.35) */
				.driver_data = &pia35x_tps_vdd3_platform_data,
		},
		/* ldo1: VDDA1P8V_USBPHY */
		{
				.constraints = {
						.min_uV           = 1800000,
						.max_uV           = 1800000,
						.valid_modes_mask = REGULATOR_MODE_NORMAL,
						.valid_ops_mask   = REGULATOR_CHANGE_STATUS,
						//.boot_on = 1,
						.always_on = true,
						.apply_uV = true,
				},
				.num_consumer_supplies = ARRAY_SIZE(pia35x_ldo1_consumers),
				.consumer_supplies = &pia35x_ldo1_consumers[0],
		},
		/* ldo2: VDDS_DPLL_1V8 */
		{
				.constraints = {
						.min_uV           = 1800000,
						.max_uV           = 1800000,
						.valid_modes_mask = REGULATOR_MODE_NORMAL,
						.valid_ops_mask   = REGULATOR_CHANGE_STATUS,
						//.boot_on = 1,
						.always_on = true,
						.apply_uV = true,
				},
				.num_consumer_supplies = ARRAY_SIZE(pia35x_vpll_consumers),
				.consumer_supplies = &pia35x_vpll_consumers[0],
		},
};

static struct tps6507x_board pia35x_tps_board = {
		/* regulator */
		.tps6507x_pmic_init_data = &pia35x_tps_regulator_data[0],
		.tps6507x_ts_init_data   = 0,   /* no touchscreen */
};

static struct i2c_board_info __initdata pia35x_tps_info[] = {
		{
				I2C_BOARD_INFO("tps6507x", 0x48),
				.platform_data = &pia35x_tps_board,
		},
};

/* register our voltage regulator TPS650732 using I2C1 */
static int __init pia35x_pmic_tps65070_init(void)
{
	return i2c_register_board_info(1, pia35x_tps_info,
									ARRAY_SIZE(pia35x_tps_info));
}

/*
 * MMC1
 */
static struct omap2_hsmmc_info mmc[] = {
	/* first MMC port used for system MMC modules */
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = 41,
		.gpio_wp        = -EINVAL, /* doesn't work, GPIO is 40,*/
		.ocr_mask       = MMC_VDD_33_34,
	},
#if defined(CONFIG_WL1271) || defined (CONFIG_WL1271_MODULEx)
	{
		.name		= "wl1271",
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
		.nonremovable	= true,
	},
#endif /* CONFIG_WL12XX_PLATFORM_DATA */
	{}	/* Terminator */
};

static void pia35x_mmc_init(void)
{
	pr_info("piA-am35x: registering VMMC1 platform device\n");
	/* handling of different MMC2 expansions here */
	omap2_hsmmc_init(mmc);
	pia35x_vmmc1_supply.dev = mmc[0].dev;
	platform_device_register(&pia35x_vmmc1_device);
	/* link regulator to on-board MMC adapter */
	//pia35x_vdd2_consumers[1].dev = mmc[0].dev;
}


/*
 * base initialisation function
 */
static void __init am3517_crane_init(void)
{
	int ret;

	ret = omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	if (ret)
		pr_warning("pia35x_init: MUX init failed: %d", ret);

	omap_serial_init();
	pr_info("pia35x_init: serial init done");
	omap_sdrc_init(NULL, NULL);

	omap_board_config = am3517_crane_config;
	omap_board_config_size = ARRAY_SIZE(am3517_crane_config);

	pia35x_mmc_init();

	pr_info("pia35x_init: initializing TPS650732");
	ret = pia35x_pmic_tps65070_init();
	if (ret)
		pr_warning("pia35x_init: TPS650732 PMIC init failed: %d", ret);

#ifdef NOT_USED
	/* Configure GPIO for EHCI port */
	if (omap_mux_init_gpio(GPIO_USB_NRESET, OMAP_PIN_OUTPUT)) {
		pr_err("Can not configure mux for GPIO_USB_NRESET %d\n",
			GPIO_USB_NRESET);
		return;
	}

	if (omap_mux_init_gpio(GPIO_USB_POWER, OMAP_PIN_OUTPUT)) {
		pr_err("Can not configure mux for GPIO_USB_POWER %d\n",
			GPIO_USB_POWER);
		return;
	}

	ret = gpio_request_one(GPIO_USB_POWER, GPIOF_OUT_INIT_HIGH,
			       "usb_ehci_enable");
	if (ret < 0) {
		pr_err("Can not request GPIO %d\n", GPIO_USB_POWER);
		return;
	}

	usbhs_init(&usbhs_bdata);
#endif

}

MACHINE_START(CRANEBOARD, "AM3517/05 CRANEBOARD")
	.atag_offset	= 0x100,
	.reserve      = omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.init_machine = am3517_crane_init,
	.timer		= &omap3_timer,
MACHINE_END
