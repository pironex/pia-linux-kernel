/*
 * Code for piA-AM335X board.
 *
 * Copyright (C) 2013 pironex GmbH. - http://www.pironex.de/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/tps65910.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/mmc.h>

#include "common.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "hsmmc.h"

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/*
* EVM Config held in On-Board eeprom device.
*
* Header Format
*
*  Name			Size	Contents
*			(Bytes)
*-------------------------------------------------------------
*  Header		4	0xAA, 0x55, 0x33, 0xEE
*
*  Board Name		8	Name for board in ASCII.
*				example "A33515BB" = "AM335X
				Low Cost EVM board"
*
*  Version		4	Hardware version code for board in
*				in ASCII. "1.0A" = rev.01.0A
*
*  Serial Number	12	Serial number of the board. This is a 12
*				character string which is WWYY4P16nnnn, where
*				WW = 2 digit week of the year of production
*				YY = 2 digit year of production
*				nnnn = incrementing board number
*
*  Configuration option	32	Codes(TBD) to show the configuration
*				setup on this board.
*
*  Available		32720	Available space for other non-volatile
*				data.
*/
struct pia335x_eeprom_config {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
};
static struct pia335x_eeprom_config config;

static void pia335x_setup(struct memory_accessor *mem_acc, void *context)
{
	int ret;
	/* get board specific data */
	ret = mem_acc->read(mem_acc, (char *)&config, 0, sizeof(config));
	/* FIXME */
}

/**
 * I2C devices
 */
#define PIA335X_EEPROM_I2C_ADDR 0x50
static struct at24_platform_data pia335x_eeprom_info = {
	.byte_len       = 128,
	.page_size      = 8,
	.flags          = AT24_FLAG_TAKE8ADDR,
	.setup          = pia335x_setup,
	.context        = (void *)NULL,
};

static struct i2c_board_info __initdata pia335x_i2c1_boardinfo[] = {
	{
		/* Daughter Board EEPROM */
		I2C_BOARD_INFO("24c00", PIA335X_EEPROM_I2C_ADDR),
		.platform_data  = &pia335x_eeprom_info,
	},
};

static void __init pia335x_i2c_init(void)
{
	/* Initially assume Low Cost EVM Config */
	//am335x_evm_id = LOW_COST_EVM;

	omap_register_i2c_bus(1, 100, pia335x_i2c1_boardinfo,
				ARRAY_SIZE(pia335x_i2c1_boardinfo));
}

void __iomem *pia335x_emif_base;

void __iomem * __init pia335x_get_mem_ctlr(void)
{

	pia335x_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!pia335x_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return pia335x_emif_base;
}

#ifndef CONFIG_MACH_AM335XEVM
void __iomem *am33xx_get_ram_base(void)
{
	return pia335x_emif_base;
}
#endif

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config pia335x_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device pia335x_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &pia335x_cpuidle_pdata,
	},
};

static void __init pia335x_cpuidle_init(void)
{
	int ret;

	pia335x_cpuidle_pdata.emif_base = pia335x_get_mem_ctlr();

	ret = platform_device_register(&pia335x_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

#include <mach/board-am335xevm.h>
static void __init pia335x_init(void)
{
	pia335x_cpuidle_init();
	am33xx_mux_init(board_mux);
	pia335x_i2c_init();
}

static void __init pia335x_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}


MACHINE_START(PIA_AM335X, "am335xpia")
	/* Maintainer: pironex */
	.atag_offset	= 0x100,
	.map_io		= pia335x_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= pia335x_init,
MACHINE_END
