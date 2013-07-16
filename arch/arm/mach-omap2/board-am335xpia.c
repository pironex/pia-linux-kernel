/*
 * Code for piA-AM335X board.
 *
 * Copyright (C) 2013 pironex GmbH. - http://www.pironex.de/
 * (based on board-am335xevm.c)
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
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/if_ether.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/tps65910.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/pwm/pwm.h>
#include <linux/reboot.h>
#include <linux/platform_data/leds-pca9633.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/clock.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/mcspi.h>
#include <plat/nand.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <video/omap-panel-dvi.h>

#include "board-flash.h"
#include "common.h"
#include "cpuidle33xx.h"
#include "devices.h"
#include "mux.h"
#include "hsmmc.h"

/** BOARD CONFIG storage */
static struct omap_board_config_kernel pia335x_config[] __initdata = {
};

/* fallback mac addresses */
static char am335x_mac_addr[2][ETH_ALEN];

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define PIA335_KM_E2		20
#define PIA335_KM_MMI		21

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


/** PINMUX **/
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

/* pinmux for led device */
static struct pinmux_config gpio_led_mux[] = {
	{"gpmc_wait0.gpio0_30", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"gpmc_wpn.gpio0_31", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
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
	/* I2C0 */
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	/* I2C1*/
	AM33XX_MUX(UART0_CTSN, OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(UART0_RTSN, OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	/* RS485 / UART3 */
	AM33XX_MUX(MII1_RXD2, OMAP_MUX_MODE1 | AM33XX_PULL_ENBL),
	AM33XX_MUX(MII1_RXD3, OMAP_MUX_MODE1 | AM33XX_INPUT_EN),
	/* PMIC INT */
	AM33XX_MUX(MII1_TXD0, OMAP_MUX_MODE7 | AM33XX_INPUT_EN |
			AM33XX_PULL_UP | AM33XX_PULL_ENBL),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};


/* Module pin mux for LCDC on board KM MMI*/
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
								   | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
							   | AM33XX_PULL_DISA},
	{"gpmc_ad8.lcd_data16",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad9.lcd_data17",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad10.lcd_data18",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad11.lcd_data19",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad12.lcd_data20",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad13.lcd_data21",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad14.lcd_data22",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad15.lcd_data23",	OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"gpmc_ben1.gpio1_28", 		OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"mcasp0_ahclkr.gpio3_17", 	OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* piA335x_MMI: LCD GPIOs */
#define GPIO_LCD_DISP		GPIO_TO_PIN(1,28)
#define GPIO_LCD_BACKLIGHT	GPIO_TO_PIN(3,17)

static int pia335x_lcd_enable(struct omap_dss_device *dssdev)
{
	gpio_set_value(GPIO_LCD_DISP, 1);
	//msleep(1000);
	pr_info("pia335x: enabling LCD\n");
	gpio_set_value(GPIO_LCD_BACKLIGHT, 1);

	return 0;
}

static void pia335x_lcd_disable(struct omap_dss_device *dssdev)
{
	gpio_set_value(GPIO_LCD_BACKLIGHT, 0);
	pr_info("pia335x: disabling LCD\n");
	gpio_set_value(GPIO_LCD_DISP, 0);
}

/* LCD: J043WQCN0101 */
static struct panel_generic_dpi_data pia335x_lcd_panel = {
	.name               = "J043WQCN0101",
	.platform_enable    = pia335x_lcd_enable,
	.platform_disable   = pia335x_lcd_disable,
};

static struct omap_dss_device pia335x_lcd_device = {
	.type               = OMAP_DISPLAY_TYPE_DPI,
	.name               = "lcd",
	.driver_name        = "generic_dpi_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio         = -EINVAL,
	.data               = &pia335x_lcd_panel,
};

static struct omap_dss_device *pia335x_dss_devices[] = {
	&pia335x_lcd_device,
};

static struct omap_dss_board_info pia335x_dss_data = {
	.num_devices     = ARRAY_SIZE(pia335x_dss_devices),
	.devices         = pia335x_dss_devices,
	.default_device  = &pia335x_lcd_device,
};


/* Touch interface */
/*#if defined(CONFIG_INPUT_TOUCHSCREEN) && \
    defined(CONFIG_TOUCHSCREEN_TSC2007)*/
#if 0
//TODO: add touch driver for J043WQCN0101 Display

/* Pen Down IRQ, low active */
#define GPIO_LCD_PENDOWN GPIO_TO_PIN(2,0);
static int pia335x_j043wqcn_pendown(void)
{
	return !gpio_get_value(GPIO_LCD_PENDOWN);
}

static int pia335x_j043wqcn_init_hw(void)
{
	int gpio = GPIO_LCD_PENDOWN;
	int ret = 0;
	pr_info("pia335x_init: init J043WQCN0101\n");
	ret = gpio_request_one(gpio, GPIOF_DIR_IN, "j043wqcn_pen_down");
	if (ret < 0) {
		pr_err("Failed to request GPIO_LCD_PENDOWN: %d\n", ret);
		return ret;
	}
	gpio_set_debounce(gpio, 0xa);
	omap_mux_init_gpio(GPIO_LCD_PENDOWN, OMAP_PIN_INPUT_PULLUP);
	irq_set_irq_type(OMAP_GPIO_IRQ(GPIO_LCD_PENDOWN), IRQ_TYPE_EDGE_FALLING);

	return ret;
}

static struct j043wqcn_platform_data j043wqcn_info = {
	.model = 2007,
	.x_plate_ohms = 180,
	.get_pendown_state = pia335x_j043wqcn_pendown,
	.init_platform_hw = pia335x_j043wqcn_init_hw,
};

/* FIXME: i2c bus */
static struct i2c_board_info __initdata pia335x_i2c1_j043wqcn[] = {
	{
		I2C_BOARD_INFO("j043wqcn", 0x4B),	/* TODO: which i2c-address? */
		.irq = OMAP_GPIO_IRQ(GPIO_LCD_PENDOWN),
		.platform_data = &j043wqcn_info,
	},
};

static void __init pia335x_touch_init(void)
{
	pr_info("pia335x_init: init touch controller J043WQCN0101\n");
	i2c_register_board_info(1, pia335x_i2c1_j043wqcn,
			ARRAY_SIZE(pia335x_i2c1_j043wqcn));
}
#else
static void __init pia335x_touch_init(void)
{}
#endif

static void pia335x_lcd_init(void)
{
	int use_lcd = 1;
	int gpio;

	setup_pin_mux(lcdc_pin_mux);

	pia335x_dss_data.default_device = &pia335x_lcd_device;

	/* backlight GPIO */
	gpio = GPIO_LCD_BACKLIGHT;
	if (gpio_request(gpio, "lcd-backlight") < 0) {
		pr_err("Failed to request gpio for lcd-backlight");
		return;
	}

	gpio_direction_output(gpio, 0);
	gpio_export(gpio, 0);

	/* DISPLAY_EN GPIO */
	gpio = GPIO_LCD_DISP;
	if (gpio_request(gpio, "lcd-disp") < 0) {
		pr_err("Failed to request gpio for lcd-disp");
		return;
	}

	gpio_direction_output(gpio, 0);
	gpio_export(gpio, 0);

	pr_info("pia335x_init: init LCD\n");

	/* initialize touch interface only for LCD display */
	if (use_lcd)
		pia335x_touch_init();

	return;
}

/* Module pin mux for mmc0 on board am335x_E2 */
static struct pinmux_config mmc0_e2_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	/* write protect */
	{"mii1_txclk.gpio3_9", AM33XX_PIN_INPUT_PULLUP},
	/* card detect */
	{"mii1_txd2.gpio0_17",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for mii2 */
static struct pinmux_config mii2_pin_mux[] = {
	/*
	{"gpmc_wpn.mii2_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	*/
	{"gpmc_a0.mii2_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a1.mii2_rxdv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a2.mii2_txd3", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a3.mii2_txd2", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a4.mii2_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.mii2_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_a6.mii2_txclk", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a7.mii2_rxclk", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a8.mii2_rxd3", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a9.mii2_rxd2", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a10.mii2_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"gpmc_a11.mii2_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for nand */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};
/* pinmux for usb0 */
static struct pinmux_config usb0_pin_mux[] = {
	/*{"usb0_drvvbus.usb0_drvvbus",    OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},*/
	{NULL, 0},
};

/* pinmux for usb1 */
static struct pinmux_config usb1_pin_mux[] = {
	/* other usb pins are not muxable */
	{"usb1_drvvbus.usb1_drvvbus", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* pinmux for special gpios */
static struct pinmux_config km_e2_gpios_pin_mux[] = {
	/* Ext. RESET */
	{"gpmc_clk.gpio2_1",      OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP },
	/* USB OC */
	{"mii1_rxd1.gpio2_20",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP },
	/* WD_SET1 */
	{"lcd_vsync.gpio2_22",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP },
	/* WD_SET2 */
	{"lcd_hsync.gpio2_23",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* WDI */
	{"lcd_pclk.gpio2_24",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP },
	/* 24V FAIL */
	{"lcd_ac_bias_en.gpio2_25",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP },
	/* FRAM WP */
	{"mcasp0_ahclkx.gpio3_21",OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* CLEAR_RESET */
	{"lcd_data3.gpio2_9",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP },
	/* WD_RESET */
	{"gpmc_ad14.gpio1_14",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* PB_RESET */
	{"mii1_col.gpio3_0",      OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* *S_ASAUS */
	{"mii1_crs.gpio3_1",      OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* 230V_A */
	{"mii1_rxerr.gpio3_2",    OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* 230V_B */
	{"mii1_rxdv.gpio3_4",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* Wartung */
	{"mii1_txd3.gpio0_16",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	/* KSB_TERM2 */
	{"gpmc_csn1.gpio1_30",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	{NULL, 0},
};

/* E2 CAN 0+1 */
static struct pinmux_config km_e2_can_pin_mux[] = {
	{"uart1_ctsn.d_can0_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart1_rtsn.d_can0_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_rxd.d_can1_tx", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL},
	{"uart1_txd.d_can1_rx", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	/* KSB_TERM */
	{"mii1_rxd0.gpio2_21",     OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN },
	{NULL, 0},
};


/* pinmux for led drivers */
static struct pinmux_config km_e2_leds_pin_mux[] = {
	/* enable input to allow readback of status */
	{"mcasp0_ahclkr.gpio3_17", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config km_e2_rs485_pin_mux[] = {
	/* signal not implemented in mux33xx.c
	{"mii1_rxd2.uart3_txd", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	{"mii1_rxd3.uart3_rxd", OMAP_MUX_MODE1 | AM33XX_PULL_ENBL},*/
	{"lcd_data11.gpio2_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* SPI1 */
static struct pinmux_config km_e2_spi01_pin_mux[] = {
	/* SPI0 */
	{"spi0_sclk.spi0_sclk", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL
			| AM33XX_INPUT_EN},
	{"spi0_d0.spi0_d0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
			| AM33XX_INPUT_EN},
	{"spi0_d1.spi0_d1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
			| AM33XX_INPUT_EN},
	{"spi0_cs0.spi0_cs0", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
			| AM33XX_INPUT_EN},
	{"spi0_cs1.spi0_cs1", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL | AM33XX_PULL_UP
			| AM33XX_INPUT_EN},
	/* SPI1 */
	{"mcasp0_aclkx.spi1_sclk", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
			| AM33XX_INPUT_EN},
	{"mcasp0_fsx.spi1_d0", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
			| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"mcasp0_axr0.spi1_d1", OMAP_MUX_MODE3 | AM33XX_PULL_ENBL
			| AM33XX_INPUT_EN},
	{"rmii1_refclk.spi1_cs0", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL
			| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	{"ecap0_in_pwm0_out.spi1_cs1", OMAP_MUX_MODE2 | AM33XX_PULL_ENBL
			| AM33XX_PULL_UP | AM33XX_INPUT_EN},
	// can2 interrupt line MCP2515
	{"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PULL_ENBL
			| AM33XX_INPUT_EN},
	{NULL, 0},
};

/** CLKOUT2 */
/* divisor 8 not supported according to TI (error in TRM)
#define SYS_CLKOUT2_PARENT	"per_192mhz_clk"
*/
#define SYS_CLKOUT2_PARENT	"lcd_gclk" /* 24MHz */
static void km_e2_clkout2_enable(void)
{
#if 0
	/* code to enable default 32k clock output*/
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);
#endif
	/* change clkout2 to a 12 MHz clock */
	struct clk *sys_clkout2;
	struct clk *parent_clk;
	struct clk *sys_clkout2_src;

	pr_info("piA335x: Initializing SYS_CLKOUT2");
	sys_clkout2_src = clk_get(NULL, "sysclkout_pre_ck");

	if (IS_ERR(sys_clkout2_src)) {
		pr_err("pia35x: Could not get clkout2_src_ck");
		return;
	}

	sys_clkout2 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(sys_clkout2)) {
		pr_err("pia35x: Could not get sys_clkout2");
		clk_put(sys_clkout2_src);
		return;
	}

	parent_clk = clk_get(NULL, SYS_CLKOUT2_PARENT);
	if (IS_ERR(parent_clk)) {
		pr_err("pia35x: Could not get " SYS_CLKOUT2_PARENT);
		clk_put(sys_clkout2);
		clk_put(sys_clkout2_src);
		return;
	}

	clk_set_parent(sys_clkout2_src, parent_clk);
	//clk_set_rate(sys_clkout2, 13500000);
	clk_set_rate(sys_clkout2, 12000000);

	pr_info("pia35x: parent of SYS_CLKOUT2 %s ", parent_clk->name);
	pr_info("pia35x: CLK - enabling SYS_CLKOUT2 with %lu MHz",
			clk_get_rate(sys_clkout2));
	clk_enable(sys_clkout2);

	setup_pin_mux(clkout2_pin_mux);
}

/* NAND partition information */
static struct mtd_partition pia335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 15 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 40 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x780000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

/* taken from ti evm */
static struct gpmc_timings pia335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void nand_init(void)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		/*{ NULL, 0 },*/
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(pia335x_nand_partitions,
		ARRAY_SIZE(pia335x_nand_partitions), 0, 0,
		&pia335x_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true; /* Error Locator Module */
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* USB0 device */
static void usb0_init(void)
{
	setup_pin_mux(usb0_pin_mux);
}

/* USB1 host */
static void usb1_init(void)
{
	setup_pin_mux(usb1_pin_mux);
}

/* MII2 */
static void mii2_init(void)
{
	pr_info("piA335x: %s\n", __func__);
	setup_pin_mux(mii2_pin_mux);
}

/** I2C1 */
static struct led_info km_e2_leds1_config[] = {
	{
		.name = "led9",
		.default_trigger = "none",
	},
	{
		.name = "pbled1",
		.default_trigger = "none",
	},
	{
		.name = "pbled3",
		.default_trigger = "none",
	},
	{
		.name = "null",
		.default_trigger = "none",
	},
	{
		.name = "pbled2",
		.default_trigger = "default-on",
	},
};
static struct pca9633_platform_data km_e2_leds1_data = {
	.leds = {
		.num_leds = 5,
		.leds = km_e2_leds1_config,
	},
	.outdrv = PCA9633_OPEN_DRAIN,
};

static struct led_info km_e2_leds2_config[] = {
	{
		.name = "led1",
		.default_trigger = "heartbeat",
	},
	{
		.name = "led2",
		.default_trigger = "none",
	},
	{
		.name = "led3",
		.default_trigger = "none",
	},
	{
		.name = "led4",
		.default_trigger = "none",
	},
	{
		.name = "led5",
		.default_trigger = "none",
	},
	{
		.name = "led6",
		.default_trigger = "none",
	},
	{
		.name = "led7",
		.default_trigger = "none",
	},
	{
		.name = "led8",
		.default_trigger = "none",
	},
};
static struct pca9633_platform_data km_e2_leds2_data = {
	.leds = {
		.num_leds = 8,
		.leds = km_e2_leds2_config,
	},
	.outdrv = PCA9633_OPEN_DRAIN,
};

/* Module pin mux for mmc0 on board am335x_MMI*/
static struct pinmux_config mmc0_mmi_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	/* write protect */
	//{"mii1_txclk.gpio3_9", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	/* card detect */
	//{"mii1_txd2.gpio0_17",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct omap2_hsmmc_info pia335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(0, 17),
		 /* WP is GPIO_TO_PIN(3, 9) but we don't need it */
		.gpio_wp        = -1,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};

static void mmc0_init(int pia_id)
{
	switch(pia_id) {
	case PIA335_KM_E2:
		setup_pin_mux(mmc0_e2_pin_mux);
		break;
	case PIA335_KM_MMI:
		setup_pin_mux(mmc0_mmi_pin_mux);
		break;
	}

	omap2_hsmmc_init(pia335x_mmc);
	return;
}

/**
 * AM33xx LEDs
 */
static struct gpio_led gpio_leds[] = {
	{
		.name			= "am335x:KM_MMI:usr1",
		.gpio			= GPIO_TO_PIN(0, 30),	/* LED1 */
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "am335x:KM_MMI:usr2",
		.gpio			= GPIO_TO_PIN(0, 31),	/* LED2 */
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void gpio_led_init(void)
{
	int err;

	setup_pin_mux(gpio_led_mux);
	err = platform_device_register(&leds_gpio);
	if (err)
		pr_err("failed to register gpio led device\n");
}

static void km_e2_leds_init(void)
{
	int gpio = GPIO_TO_PIN(3, 17);
	setup_pin_mux(nand_pin_mux);
	if (gpio_request(gpio, "led_oe") < 0) {
		pr_err("Failed to request gpio for led_oe");
		return;
	}

	pr_info("Configure LEDs...\n");
	gpio_direction_output(gpio, 0);
	gpio_export(gpio, 0);
}

/* FRAM is similar to at24 eeproms without write delay and page limits */
static struct at24_platform_data e2_km_fram_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = (256*1024) / 8, /* no sequencial rw limit */
	.flags          = AT24_FLAG_ADDR16,
	.context        = (void *)NULL,
};

static struct i2c_board_info km_e2_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("pca9634", 0x22),
		.platform_data = &km_e2_leds1_data,
	},
	{
		I2C_BOARD_INFO("pca9634", 0x23),
		.platform_data = &km_e2_leds2_data,
	},
	{
		I2C_BOARD_INFO("tmp422", 0x4C),
	},
	{	I2C_BOARD_INFO("24c256", 0x52),
		.platform_data = &e2_km_fram_info,
	}
};

static void km_e2_i2c2_init(void)
{
	setup_pin_mux(km_e2_leds_pin_mux);
	km_e2_leds_init();
	omap_register_i2c_bus(2, 400, km_e2_i2c1_boardinfo,
			ARRAY_SIZE(km_e2_i2c1_boardinfo));
}

struct pia_gpios {
	int gpio; /* gpio number from GPIO_TO_PIN() */
	char *name;
	int input  :1;
	int export :1;
	int value  :1;
};
#define E2_GPIO_EXT_RESET	GPIO_TO_PIN(2, 1)
#define E2_GPIO_USB_OC		GPIO_TO_PIN(2, 20)
#define E2_GPIO_WDI		GPIO_TO_PIN(2, 24)
#define E2_GPIO_24V_FAIL	GPIO_TO_PIN(2, 25)
#define E2_GPIO_CLEAR_RESET	GPIO_TO_PIN(2, 9)
#define E2_GPIO_WD_RESET	GPIO_TO_PIN(1, 14)
#define E2_GPIO_PB_RESET	GPIO_TO_PIN(3, 0)
#define E2_GPIO_S_ASAUS		GPIO_TO_PIN(3, 1)
#define E2_GPIO_230V_A		GPIO_TO_PIN(3, 2)
#define E2_GPIO_230V_B		GPIO_TO_PIN(3, 4)
#define E2_GPIO_WARTUNG		GPIO_TO_PIN(0, 16)
#define E2_GPIO_KSB_TERM1	GPIO_TO_PIN(2, 21)
#define E2_GPIO_KSB_TERM2	GPIO_TO_PIN(1, 30)
static struct gpio km_e2_gpios[] = {
	{ E2_GPIO_EXT_RESET,	GPIOF_OUT_INIT_HIGH, "ext_reset" },
	{ E2_GPIO_USB_OC,	GPIOF_IN, "usb_oc" },
	{ GPIO_TO_PIN(2, 22),	GPIOF_OUT_INIT_HIGH, "wd_set1" },
	{ GPIO_TO_PIN(2, 23),	GPIOF_OUT_INIT_LOW,  "wd_set2" },
	{ E2_GPIO_WDI, 		GPIOF_IN, "wdi" },
	{ E2_GPIO_24V_FAIL,	GPIOF_IN, "24v_fail" },
	{ GPIO_TO_PIN(3, 21),	GPIOF_OUT_INIT_LOW,  "fram_wp" },
	{ E2_GPIO_CLEAR_RESET,	GPIOF_OUT_INIT_HIGH, "clear_reset" },
	{ E2_GPIO_WD_RESET,	GPIOF_IN, "wd_reset" },
	{ E2_GPIO_PB_RESET,	GPIOF_IN, "pb_reset" },
	{ E2_GPIO_S_ASAUS,	GPIOF_IN, "s_asaus" },
	{ E2_GPIO_230V_A,	GPIOF_IN, "230v_a" },
	{ E2_GPIO_230V_B,	GPIOF_IN, "230v_b" },
	{ E2_GPIO_WARTUNG,	GPIOF_IN, "wartung" },
	{ E2_GPIO_KSB_TERM1,	GPIOF_OUT_INIT_LOW, "ksb_term1" },
	{ E2_GPIO_KSB_TERM2,	GPIOF_OUT_INIT_LOW, "ksb_term2" },
};

static void pia_print_gpio_state(const char *msg, int gpio, int on)
{
	int val = gpio_get_value(gpio);
	if (val >= 0)
		pr_info("  %s: %s\n", msg, (val == on ? "ON!" : "OFF."));
	else
		pr_warn("  %s: Unable to read GPIO!\n", msg);
}
static void km_e2_gpios_init(void)
{
	int i;
	setup_pin_mux(km_e2_gpios_pin_mux);
	for (i = 0; i < ARRAY_SIZE(km_e2_gpios); ++i) {
		if (gpio_request_one(km_e2_gpios[i].gpio,
				km_e2_gpios[i].flags,
				km_e2_gpios[i].label) < 0) {
			pr_err("Failed to request gpio: %s\n",
					km_e2_gpios[i].label);
			return;
		}
		pr_info("piA335x: GPIO init %s\n", km_e2_gpios[i].label);
		gpio_export(km_e2_gpios[i].gpio, 0);
	}
	pr_info("E2 GPIO Status:\n");
	pia_print_gpio_state("WD_RESET: ", E2_GPIO_WD_RESET, 0);
	pia_print_gpio_state("WD_RESET: ", E2_GPIO_PB_RESET, 0);
	pr_info("  external Watchdog: OFF.\n");
	gpio_set_value(E2_GPIO_CLEAR_RESET, 0); /* high to low */
	pr_info("    RESET flags cleared.\n");

	pia_print_gpio_state("USB_OC:   ", E2_GPIO_USB_OC, 0);
	/* TODO check active low/high */
	pia_print_gpio_state("24V Fail: ", E2_GPIO_24V_FAIL, 1);
	pia_print_gpio_state("*S_ASAUS: ", E2_GPIO_S_ASAUS, 1);
	pia_print_gpio_state("230V_A:   ", E2_GPIO_230V_A, 1);
	pia_print_gpio_state("230V_B:   ", E2_GPIO_230V_B, 1);
	pia_print_gpio_state("WARTUNG:  ", E2_GPIO_WARTUNG, 1);
}

extern void am33xx_d_can_init(unsigned int instance);
static void km_e2_can_init(void)
{
	setup_pin_mux(km_e2_can_pin_mux);
	am33xx_d_can_init(0);
	am33xx_d_can_init(1);

}

/* SPI1 -> CAN2 */
#define KM_E2_CAN2_INT_GPIO GPIO_TO_PIN(3, 20)
#include <linux/can/platform/mcp251x.h>
static struct mcp251x_platform_data km_e2_mcp2515_data = {
	.oscillator_frequency = 25E6 ,
};
static struct omap2_mcspi_device_config km_e2_spi_def_cfg = {
	.turbo_mode	= 0,
	.d0_mosi	= 1, /* we use MOSI on D0 for all SPI devices */
};
static struct spi_board_info km_e1_spi0_info[] = {
	{	/* LS7366, max 8 MHz */
		.modalias      = "spidev",
		.bus_num         = 1,
		.chip_select     = 0,
		.controller_data = &km_e2_spi_def_cfg,
		.max_speed_hz    = 5E6, /* 5MHz */
	},
	{
		/* APS only MISO used */
		.modalias      = "spidev",
		.bus_num         = 1,
		.chip_select     = 1,
		.controller_data = &km_e2_spi_def_cfg,
		.max_speed_hz    = 1E6, /* 1MHz */
	},
};

static struct spi_board_info km_e1_spi1_info[] = {
	{
		/* 3rd CAN device */
		.modalias      = "mcp2515",
		.bus_num       = 2,
		.chip_select   = 0,
		.max_speed_hz  = 8E6, /* 5 MHz */
		.mode          = SPI_MODE_0,
		.irq           = OMAP_GPIO_IRQ(KM_E2_CAN2_INT_GPIO),
		.controller_data = &km_e2_spi_def_cfg,
		.platform_data = &km_e2_mcp2515_data,
	},
	{
		/* external Header */
		.modalias      = "spidev",
		.bus_num         = 2,
		.chip_select     = 1,
		.max_speed_hz    = 1E6, /* 1MHz */
	},
};
static void km_e2_spi1_init(void)
{
	setup_pin_mux(km_e2_spi01_pin_mux);
	spi_register_board_info(km_e1_spi0_info,
			ARRAY_SIZE(km_e1_spi0_info));
	spi_register_board_info(km_e1_spi1_info,
			ARRAY_SIZE(km_e1_spi1_info));
}

#define KM_E2_RS485_DE_GPIO	GPIO_TO_PIN(2, 17)
static void km_e2_rs485_init(void)
{
	setup_pin_mux(km_e2_rs485_pin_mux);
	/* use GPIO for RS485 Driver Enable signal
	 * Auto-RTS functionality (MUX MODE 6) cannot be used, because it
	 * doesn't de-assert RTS if a transmission is active, which would
	 * be required to provide a driver disable functionality.
	 *
	 * Instead it disables RTS only if RX FIFO is full, which means
	 * during normal operation RTS will always be active and so would the
	 * driver, while the receiver would be deactivated most of the time.
	 *
	 * For Half-Duplex RS485 we need the receiver to be enabled whenever
	 * no transmission is active, as Tranceiver and Receiver must never be
	 * active at the same time.
	 */
	if (gpio_request(KM_E2_RS485_DE_GPIO, "te_reg") < 0) {
		pr_err("Failed to request gpio for led_oe");
		return;
	}

	pr_info("Configure RS485 TE GPIO\n");
	/* enable receiver by default */
	gpio_direction_output(KM_E2_RS485_DE_GPIO, 0);
	gpio_export(KM_E2_RS485_DE_GPIO, 0);
}

static void km_e2_ls7366_init(void)
{
	//pia335x_clkout2_enable();
}

/**
 * AM33xx internal RTC
 */
#include <linux/rtc/rtc-omap.h>
static struct omap_rtc_pdata pia335x_rtc_info = {
	.pm_off		= false,
	.wakeup_capable	= 0,
};

static int pia335x_rtc_init(void)
{
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return -1;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return -1;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return -1;

	/* Unlock the rtc's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 *
	 * pia: we don't really need the 32k external OSC
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	// TODO check pia335x_rtc_info.pm_off = true;

	clk_disable(clk);
	clk_put(clk);

	if (omap_rev() == AM335X_REV_ES2_0)
		pia335x_rtc_info.wakeup_capable = 1;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return -1;
	}

	pdev = omap_device_build(dev_name, -1, oh, &pia335x_rtc_info,
			sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n",
			dev_name, oh->name);

	return 0;
}

/* Accelerometer LIS331DLH */
#include <linux/lis3lv02d.h>

static struct lis3lv02d_platform_data lis331dlh_pdata = {
	.click_flags = LIS3_CLICK_SINGLE_X |
			LIS3_CLICK_SINGLE_Y |
			LIS3_CLICK_SINGLE_Z,
	.wakeup_flags = LIS3_WAKEUP_X_LO | LIS3_WAKEUP_X_HI |
			LIS3_WAKEUP_Y_LO | LIS3_WAKEUP_Y_HI |
			LIS3_WAKEUP_Z_LO | LIS3_WAKEUP_Z_HI,
	.irq_cfg = LIS3_IRQ1_CLICK | LIS3_IRQ2_CLICK,
	.wakeup_thresh	= 10,
	.click_thresh_x = 10,
	.click_thresh_y = 10,
	.click_thresh_z = 10,
	.g_range	= 2,
	.st_min_limits[0] = 120,
	.st_min_limits[1] = 120,
	.st_min_limits[2] = 140,
	.st_max_limits[0] = 550,
	.st_max_limits[1] = 550,
	.st_max_limits[2] = 750,
};

static struct i2c_board_info lis331dlh_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("lis331dlh", 0x18),
		.platform_data = &lis331dlh_pdata,
	},
};

static void lis331dlh_init(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	unsigned int i2c_instance;

	i2c_instance = 1;

	/* I2C adapter request */
	adapter = i2c_get_adapter(i2c_instance);
	if (!adapter) {
		pr_err("failed to get adapter i2c%u\n", i2c_instance);
		return;
	}

	client = i2c_new_device(adapter, lis331dlh_i2c_boardinfo);
	if (!client)
		pr_err("failed to register lis331dlh to i2c%u\n", i2c_instance);

	i2c_put_adapter(adapter);
}

/*
 * Audio
 */
/* Module pin mux for mcasp0 */
static struct pinmux_config mcasp0_pin_mux[] = {
	/* Audio.BCLK */
	{"mcasp0.aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio.FSX */
	{"mcasp0.fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio.DIN */
	{"mcasp0.aclkr.mcasp0_axr2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio.DOUT */
	{"mcasp0.ahclkx.mcasp0_axr3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

static u8 am335x_iis_serializer_direction1[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	TX_MODE,	RX_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data am335x_km_mmi_snd_data1 = {
	.tx_dma_offset	= 0x46000000,	/* McASP1 */
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

/* Setup McASP 0 */
static void mcasp0_init(int pia_id)
{
	/* Configure McASP */
	setup_pin_mux(mcasp0_pin_mux);
	switch (pia_id) {
	case PIA335_KM_MMI:
		am335x_register_mcasp(&am335x_km_mmi_snd_data1, 0);
		break;
	default:
		break;
	}

	return;
}

static struct i2c_board_info tlv320aic3x_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
};

static void tlv320aic3x_i2c_init(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	unsigned int i2c_instance = 1;

	/* I2C adapter request */
	adapter = i2c_get_adapter(i2c_instance);
	if (!adapter) {
		pr_err("failed to get adapter i2c%u\n", i2c_instance);
		return;
	}

	client = i2c_new_device(adapter, tlv320aic3x_i2c_boardinfo);
	if (!client)
		pr_err("failed to register tlv320aic3x to i2c%u\n", i2c_instance);

	i2c_put_adapter(adapter);
}

static void setup_e2(void)
{
	pr_info("piA335x: Setup KM E2.\n");
	/* EVM - Starter Kit */
/*	static struct evm_dev_cfg evm_sk_dev_cfg[] = {
		{mmc1_wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{enable_ecap2,     DEV_ON_BASEBOARD, PROFILE_ALL},
		{mfd_tscadc_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{gpio_keys_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
		{lis331dlh_init, DEV_ON_BASEBOARD, PROFILE_ALL},
		{mcasp1_init,   DEV_ON_BASEBOARD, PROFILE_ALL},
		{uart1_wl12xx_init, DEV_ON_BASEBOARD, PROFILE_ALL},
		{wl12xx_init,       DEV_ON_BASEBOARD, PROFILE_ALL},
		{gpio_ddr_vtt_enb_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{NULL, 0, 0},
	};*/
	pia335x_rtc_init();
	km_e2_i2c2_init(); /* second i2c bus */
	mmc0_init(PIA335_KM_E2);
	mii2_init();
	usb0_init();
	usb1_init();
	nand_init();

	km_e2_clkout2_enable();

	km_e2_gpios_init();
	km_e2_can_init();
	km_e2_spi1_init();
	km_e2_rs485_init();
	km_e2_ls7366_init();

	pr_info("piA335x: cpsw_init\n");
	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:1e", "0:00");
}

static void setup_mmi(void)
{
	pr_info("piA335x: Setup KM MMI.\n");

	pia335x_rtc_init();

	mmc0_init(PIA335_KM_MMI);

	/* KM MMI has Micro-SD slot which doesn't have Write Protect pin */
	pia335x_mmc[0].gpio_wp = -EINVAL;

	/* KM MMI has Micro-SD slot which doesn't have Card Detect pin */
	pia335x_mmc[0].gpio_cd = -EINVAL,
	pia335x_mmc[0].nonremovable	= true,

	lis331dlh_init();
	tlv320aic3x_i2c_init();
	mcasp0_init(PIA335_KM_MMI);

	pr_info("piA335x: cpsw_init\n");
	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);

	gpio_led_init();
	pia335x_lcd_init();
}

void am33xx_cpsw_macidfillup(char *eeprommacid0, char *eeprommacid1);
static void pia335x_setup(struct memory_accessor *mem_acc, void *context)
{
	/* generic board detection triggered by eeprom init */
	int ret;
	char tmp[10];

	pr_info("piA335x: setup\n");
	/* from evm code
	 * 1st get the MAC address from EEPROM */
	ret = mem_acc->read(mem_acc, (char *)&am335x_mac_addr,
		0x60, sizeof(am335x_mac_addr));

	if (ret != sizeof(am335x_mac_addr)) {
		pr_warning("AM335X: EVM Config read fail: %d\n", ret);
		return;
	}

	/* Fillup global mac id */
	am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
				&am335x_mac_addr[1][0]);

	/* get board specific data */
	ret = mem_acc->read(mem_acc, (char *)&config, 0, sizeof(config));
	if (ret != sizeof(config)) {
		pr_err("piA335x config read fail, read %d bytes\n", ret);
		goto out;
	}

	if (config.header != 0xEE3355AA) { // header magic number
		pr_err("piA335x: wrong header 0x%x, expected 0x%x\n",
			config.header, 0xEE3355AA);
		goto out;
	}

	if (strncmp("PIA335", config.name, 6)) {
		pr_err("Board %s\ndoesn't look like a PIA335 board\n",
			config.name);
		goto out;
	}

	snprintf(tmp, sizeof(config.name) + 1, "%s", config.name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(config.version) + 1, "%s", config.version);
	pr_info("Board version: %s\n", tmp);

	if (!strncmp("PIA335E2", config.name, 8)) {
		//daughter_brd_detected = false;
		if(!strncmp("0.01", config.version, 4)) {
			setup_e2();
		} else {
			pr_info("PIA335E2: Unknown board revision %.4s\n",
					config.version);
		}
	} else if(!strncmp("PIA335MI", config.name, 8)) {
		if(!strncmp("0.01", config.version, 4)) {
			setup_mmi();
		} else {
			pr_info("PIA335MI: Unknown board revision %.4s\n",
					config.version);
		}
	}

	return;

out:
	/*
	 * If the EEPROM hasn't been programed or an incorrect header
	 * or board name are read then the hardware details are unknown.
	 * Notify the user and call machine_halt to stop the boot process.
	 */
	pr_err("PIA335x: Board identification failed... Halting...\n");
	machine_halt();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 */
	.mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

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

static struct regulator_init_data pia335x_tps_dummy = {
	.constraints.always_on	= true,
};

static struct tps65910_board pia335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &pia335x_tps_dummy,
	.gpio_base = (4 * 32),
	.irq = GPIO_TO_PIN(0, 28),
};

static struct i2c_board_info __initdata pia335x_i2c0_boardinfo[] = {
	{
		/* Daughter Board EEPROM */
		I2C_BOARD_INFO("24c00", PIA335X_EEPROM_I2C_ADDR),
		.platform_data  = &pia335x_eeprom_info,
	},
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &pia335x_tps65910_info,
	},
};

static void __init pia335x_i2c_init(void)
{
	/* I2C1 must be muxed in u-boot */
	pr_info("piA335x: %s", __func__);
	omap_register_i2c_bus(1, 100, pia335x_i2c0_boardinfo,
				ARRAY_SIZE(pia335x_i2c0_boardinfo));
}

#ifdef CONFIG_MACH_AM335XEVM
/* FIXME for some reason board specific stuff is called from mach code
 * e.g.
 * pm33xx.c depends on definitions from board-am33xevm.c
 * or
 * multiple AM335x board definitions must conflict with each other
 * either way, this is a hack to prevent multiple definitions for now
 */
extern void __iomem * am33xx_emif_base;
extern void __iomem * __init am33xx_get_mem_ctlr(void);
#else
void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{
	am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!am33xx_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return am335xx_emif_base;
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

	pr_info("piA335x: %s\n", __func__);

	pia335x_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&pia335x_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");

}

#include <mach/board-am335xevm.h>
static void __init pia335x_init(void)
{
	pia335x_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	pr_info("piA335x: i2c_init\n");
	pia335x_i2c_init();
	pr_info("piA335x: sdrc_init\n");
	omap_sdrc_init(NULL, NULL);
	pr_info("piA335x: musb_init\n");
	usb_musb_init(&musb_board_data);
	/* XXX what for? */
	omap_board_config = pia335x_config;
	omap_board_config_size = ARRAY_SIZE(pia335x_config);
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
