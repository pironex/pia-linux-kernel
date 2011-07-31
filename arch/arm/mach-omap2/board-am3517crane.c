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
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>

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
#else
#define board_mux	NULL
#endif

static void __init am3517_crane_init_irq(void)
{
	omap_board_config = am3517_crane_config;
	omap_board_config_size = ARRAY_SIZE(am3517_crane_config);

	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = GPIO_USB_NRESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

/*
 * MMC
 */
#include <linux/regulator/fixed.h>

static struct regulator_consumer_supply pia3517_vmmc1_supply = {
	.supply         = "vmmc",
	.dev_name       = "mmci-omap-hs.0", /* bind to our MMC1 device */
};

static struct regulator_init_data pia3517_vmmc1_data = {
	.constraints = {
			.min_uV           = 1850000,
			.max_uV           = 3150000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
			.valid_ops_mask   = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &pia3517_vmmc1_supply,
};

static struct fixed_voltage_config pia3517_vmmc1_config = {
	.supply_name = "vmmc1",
	.microvolts = 1800000,  /* 1.8V */
	//.gpio = OMAP_BEAGLE_WLAN_EN_GPIO,
	.startup_delay = 70000, /* 70ms */
	.enable_high = 1,
	.enabled_at_boot = 1,
	.init_data = &pia3517_vmmc1_data,
};

static struct platform_device pia3517_vmmc1_device = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data = &pia3517_vmmc1_config,
	},
};

static struct omap2_hsmmc_info mmc[] = {
	/* first MMC port used for system MMC modules */
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = 41,
		.gpio_wp        = 40,
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

static void pia3517_mmc_init(void)
{
	printk(KERN_INFO "piA-am35x: registering VMMC1 platform device\n");
	platform_device_register(&pia3517_vmmc1_device);
	/* handling of different MMC2 expansions here */
	omap2_hsmmc_init(mmc);
	/* link regulators to MMC adapters */
	//xx_vmmc1_supply.dev = mmc[0].dev;
	//xx_vsim_supply.dev = mmc[0].dev;

}

static void __init am3517_crane_init(void)
{
	/* additional pin muxing */
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	/* initialize uarts */
	omap_serial_init();

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

	ret = gpio_request(GPIO_USB_POWER, "usb_ehci_enable");
	if (ret < 0) {
		pr_err("Can not request GPIO %d\n", GPIO_USB_POWER);
		return;
	}

	ret = gpio_direction_output(GPIO_USB_POWER, 1);
	if (ret < 0) {
		gpio_free(GPIO_USB_POWER);
		pr_err("Unable to initialize EHCI power\n");
		return;
	}

	usb_ehci_init(&ehci_pdata);
#endif

	pia3517_mmc_init();
}

MACHINE_START(CRANEBOARD, "AM3517/05 CRANEBOARD")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= am3517_crane_init_irq,
	.init_machine	= am3517_crane_init,
	.timer		= &omap_timer,
MACHINE_END
