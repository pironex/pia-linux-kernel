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
#include <linux/spi/flash.h>
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
#include <linux/opp.h>

#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xpia.h>

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
#include <plat/lcdc.h>

#include "board-flash.h"
#include "common.h"
#include "cpuidle33xx.h"
#include "devices.h"
#include "mux.h"
#include "hsmmc.h"

// #define CONFIG_PIAAM335X_PROTOTYPE

/** BOARD CONFIG storage */
static struct omap_board_config_kernel pia335x_config[] __initdata = {
};

/* fallback mac addresses */
static char am335x_mac_addr[2][ETH_ALEN];

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

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
*
*  Version		4	Hardware version code for board in
*				in ASCII. e.g. "0.01"
*
*  Serial Number	12	currently not used on piA boards
*
* Configuration option	32	0: 'B' base version, 'X' extended
*				   only relevant for MMI
*				1: 'N' board has NAND
*				2: 'R' resitive Touch, 'C' capacitive touch
*				3: RAM: 'K' Kingston, 'H' Hynix, empty Micron
*				4: EMMC: 'K' Kingston, empty default
*
*  Available		60	Available space for other non-volatile data.
*/
struct pia335x_eeprom_config {
	u32	header;
	u8	name[8];
	char	version[4];
	u8	serial[12];
	u8	opt[32];
};
#define PIA335X_EEPROM_I2C_ADDR 0x50
static struct pia335x_eeprom_config config;
static struct pia335x_eeprom_config exp_config;
static struct pia335x_eeprom_config lcd_exp_config;

struct pia335x_board_id {
	const char *name; /* name as saved in EEPROM name field */
	int id;   /* internal ID */
	int rev;  /* 0: "a.bc" or continuous, 1: reserved */
	const int type; /* 0: main board/PM, 1: base board/expansion, 2 LCD */
	struct pia335x_eeprom_config *config;
	void (*setup)(void); /* TODO implement generic setup + extra setup */
};

static struct pia335x_board_id pia335x_boards[] = {
	{ "PIA335E2", PIA335_KM_E2,	0, 0},
	{ "PIA335MI", PIA335_KM_MMI,	0, 0},
	{ "PIA335PM", PIA335_PM,	0, 0},
	{ "PIA335EM", PIA335_LOKISA_EM, 0, 0},
	{ "P335BEBT", PIA335_BB_EBTFT,	0, 1},
	{ "P335BSK",  PIA335_BB_SK,	0, 1},
	{ "P335BAPC", PIA335_BB_APC,	0, 1},
	{ "LCDKMMMI", PIA335_LCD_KM_MMI,0, 2},
};

static struct pia335x_board_id pia335x_main_id = {
	.id	= -EINVAL,
	.rev	= -EINVAL,
	.type	= 0,
	.config = &config,
};
static struct pia335x_board_id pia335x_exp_id = {
	.id	= -EINVAL,
	.rev	= -EINVAL,
	.type	= 1,
	.config = &exp_config,
};

static struct pia335x_board_id pia335x_lcd_exp_id = {
	.id	= -EINVAL,
	.rev	= -EINVAL,
	.type	= 2,
	.config = &lcd_exp_config,
};

static int pm_setup_done = 0;
void am33xx_cpsw_macidfillup(char *eeprommacid0, char *eeprommacid1);
static int pia335x_read_eeprom(struct memory_accessor *mem_acc,
		struct pia335x_board_id* id)
{
	struct pia335x_eeprom_config *conf;
	int ret;

	if (!id || !id->config)
		return -ENODEV;

	conf = id->config;
	if (!pm_setup_done) {
		/* from evm code
		 * 1st get the MAC address from EEPROM */
		ret = mem_acc->read(mem_acc, (char *)&am335x_mac_addr,
				0x60, sizeof(am335x_mac_addr));

		if (ret != sizeof(am335x_mac_addr)) {
			pr_warning("AM335X: MAC Config read fail: %d\n", ret);
		}
		/* Fillup global mac id */
		am33xx_cpsw_macidfillup(&am335x_mac_addr[0][0],
				&am335x_mac_addr[1][0]);
	}

	/* get board specific data */
	ret = mem_acc->read(mem_acc, (char *)conf, 0, sizeof(*conf));
	if (ret == -ETIMEDOUT) {
		pr_warn("piA335x config read timed out\n");
		return ret;
	}
	if (ret != sizeof(*conf)) {
		pr_err("piA335x config read fail, read %d bytes\n", ret);
		return ret;
	}

	if (conf->header != 0xEE3355AA) { // header magic number
		pr_err("piA335x: wrong header 0x%x, expected 0x%x\n",
			conf->header, 0xEE3355AA);
		return -1;
	}

	return 0;
}

static int pia335x_parse_rev(const char *in, int revtype)
{
	int i = 0;
	int rev = 0;
	for (; i < 4; ++i) {
		if (in[i] >= '0' && in[i] <= '9') {
			rev = (rev * 10) + (in[i] - '0');
		}
		/* ignore everything else, like decimal point
		 * if there is no valid number in the field, result is 0,
		 * which is an invalid revision number */
	}
	return (rev ? rev : -EINVAL);
}

static void pia335x_parse_eeprom(struct pia335x_board_id *id)
{
	char tmp[10];
	int i = 0;
	struct pia335x_board_id *it;
	struct pia335x_eeprom_config *eeprom = id->config;

	if (!id || !id->config)
		return;

	for (; i < ARRAY_SIZE(pia335x_boards); ++i) {
		it = &pia335x_boards[i];
		if (0 != strncmp(it->name, eeprom->name, 8))
			continue;
		id->id = it->id;
		id->rev = pia335x_parse_rev(eeprom->version, it->rev);
	}
	snprintf(tmp, sizeof(eeprom->name) + 1, "%s", eeprom->name);
	pr_info("Board name: %s\n", tmp);
	snprintf(tmp, sizeof(eeprom->version) + 1, "%s", eeprom->version);
	pr_info("Board version: %s (%d)\n", tmp, id->rev);
}

/*
 * am335x_pia_get_id - returns Board Type (PIA335_KM_E2/PIA335_KM_MMI ...)
 * In case of a processor module + baseboard configuration,
 * return the baseboard id.
 */
int am335x_pia_get_id(void)
{
	return (pia335x_exp_id.id > 0 ?
			pia335x_exp_id.id : pia335x_main_id.id);
}
EXPORT_SYMBOL(am335x_pia_get_id);

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

/* add additional device to an already registered and initialized adapter. */
static int pia335x_add_i2c_device(int busnum, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;

	/* I2C adapter request */
	adapter = i2c_get_adapter(busnum);
	if (!adapter) {
		pr_err("failed to get adapter i2c%u\n", busnum);
		return -1;
	}

	client = i2c_new_device(adapter, info);
	if (!client) {
		pr_err("failed to register 0x%x to i2c%u\n",
				info->addr, busnum);
		return -1;
	}
	pr_info("I2C new device: %s 0x%x@%d", client->name, client->addr,
			client->adapter->nr);
	i2c_put_adapter(adapter);

	return 0;
}
static void pia335x_register_i2c_devices(int busnum,
		struct i2c_board_info *infos, int cnt)
{
	int i = 0;
	for (; i < cnt; ++i)
		pia335x_add_i2c_device(busnum, &infos[i]);
}

/** PINMUX tables */
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* I2C0 */
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

static struct pinmux_config km_e2_board_pin_mux[] = {
	/* I2C1*/
	{ "uart0_ctsn.i2c1_sda",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "uart0_rtsn.i2c1_scl",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	/* disable JTAG EMU 2+3 */
	{"xdma_event_intr0.gpio0_19", AM33XX_PIN_INPUT_PULLUP},
	{"xdma_event_intr1.gpio0_20", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config km_mmi_board_pin_mux[] = {
	/* I2C1*/
	{ "uart1_rxd.i2c1_sda",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "uart1_txd.i2c1_scl",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	/* CLKOUT1 */
	{"xdma_event_intr0.clkout1", AM33XX_PIN_INPUT_PULLUP},
	/* disable EMU3 by default */
	{"xdma_event_intr1.gpio0_20", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config pm_board_pin_mux[] = {
	/* I2C1*/
	{ "mii1_crs.i2c1_sda",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "mii1_rxerr.i2c1_scl",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "uart0_rxd.uart0_rxd",
		AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "uart0_txd.uart0_txd",
		AM33XX_PIN_OUTPUT | AM33XX_SLEWCTRL_SLOW },
	{NULL, 0},
};

static struct pinmux_config em_board_pin_mux[] = {
	{ "uart1_rxd.uart1_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "uart1_txd.uart1_txd", AM33XX_PIN_OUTPUT },
	{ "mii1_txclk.uart2_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxclk.uart2_txd", AM33XX_PIN_OUTPUT },
	{ "lcd_data8.uart2_ctsn", AM33XX_PIN_INPUT_PULLUP },
	{ "lcd_data9.uart2_rtsn", AM33XX_PIN_OUTPUT },
	{ "mii1_rxd3.uart3_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxd2.uart3_txd", AM33XX_PIN_OUTPUT },
	{ "mii1_txd3.uart4_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_txd2.uart4_txd", AM33XX_PIN_OUTPUT },
	{ "mii1_col.uart5_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "rmii1_refclk.uart5_txd", AM33XX_PIN_OUTPUT },
	{NULL, 0},
};

static struct pinmux_config apc_board_pin_mux[] = {
	{ "mii1_crs.i2c1_sda",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "mii1_rxerr.i2c1_scl",
			AM33XX_PIN_INPUT_PULLUP | AM33XX_SLEWCTRL_SLOW },
	{ "uart0_rxd.uart0_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "uart0_txd.uart0_txd", AM33XX_PIN_OUTPUT },
	{ "uart1_rxd.uart1_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "uart1_txd.uart1_txd", AM33XX_PIN_OUTPUT },
	{ "mii1_txclk.uart2_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxclk.uart2_txd", AM33XX_PIN_OUTPUT },
	{ "lcd_data8.uart2_ctsn", AM33XX_PIN_INPUT_PULLUP },
	{ "lcd_data9.uart2_rtsn", AM33XX_PIN_OUTPUT },
	{ "mii1_rxd3.uart3_rxd",  AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxd2.uart3_txd", AM33XX_PIN_OUTPUT },
	{ "mii1_txd3.uart4_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_txd2.uart4_txd", AM33XX_PIN_OUTPUT },
	{ "mii1_col.uart5_rxd", AM33XX_PIN_INPUT_PULLUP },
	{ "rmii1_refclk.uart5_txd", AM33XX_PIN_OUTPUT },
	{ "lcd_data14.uart5_ctsn", AM33XX_PIN_INPUT_PULLUP },
	{ "lcd_data15.uart5_rtsn", AM33XX_PIN_OUTPUT },
	{NULL, 0},
};

static struct pinmux_config clkout1_pin_mux[] = {
	{"xdma_event_intr0.clkout1", AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

#ifdef CONFIG_PIAAM335X_PROTOTYPE
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
#endif

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

/* E2 RS485 */
static struct pinmux_config km_e2_rs485_pin_mux[] = {
	{"mii1_rxd2.uart3_txd", AM33XX_PIN_OUTPUT_PULLUP},
	{"mii1_rxd3.uart3_rxd", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* E2 UART4 */
static struct pinmux_config km_e2_uart4_pin_mux[] = {
	{"mii1_txd2.uart4_txd", AM33XX_PIN_OUTPUT_PULLUP},
	{"mii1_txd3.uart4_rxd", AM33XX_PIN_INPUT_PULLUP},
	/* Boot0_E1 */
	{"mii1_rxdv.gpio3_4",      AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

struct pia_gpios {
	int gpio; /* gpio number from GPIO_TO_PIN() */
	char *name;
	int input  :1;
	int export :1;
	int value  :1;
};
#define E2_GPIO_EXT_RESET	GPIO_TO_PIN(2, 1)
#define E2_GPIO_USB_OC		GPIO_TO_PIN(2, 20)
#define E2_GPIO_WD_SET1		GPIO_TO_PIN(2, 22)
#define E2_GPIO_WD_SET2		GPIO_TO_PIN(2, 23)
#define E2_GPIO_WDI		GPIO_TO_PIN(2, 24)
#define E2_GPIO_24V_FAIL	GPIO_TO_PIN(2, 25)
#define E2_GPIO_CLEAR_RESET	GPIO_TO_PIN(2, 9)
#define E2_GPIO_FF_CLK		GPIO_TO_PIN(3, 10) /* flipflop reset */
#define E2_GPIO_WD_RESET	GPIO_TO_PIN(1, 14)
#define E2_GPIO_PB_RESET	GPIO_TO_PIN(3, 0)
#define E2_GPIO_S_ASAUS		GPIO_TO_PIN(3, 1)
#define E2_GPIO_230V_A		GPIO_TO_PIN(3, 2)
#define E2_GPIO_230V_B		GPIO_TO_PIN(3, 4)
#define E2_GPIO_FRAM_WP		GPIO_TO_PIN(3, 21)
#define E2_GPIO_WARTUNG		GPIO_TO_PIN(3, 3)
#define E2_GPIO_KSB_TERM1	GPIO_TO_PIN(2, 21)
#define E2_GPIO_APS_TERM	GPIO_TO_PIN(1, 31)
#define E2_GPIO_RESERVE2	GPIO_TO_PIN(3, 9)
#define E2_GPIO_RESERVE3	GPIO_TO_PIN(0, 27)
#define E2_GPIO_RESERVE4	GPIO_TO_PIN(3, 4)
#define E2_GPIO_POE_PS_SHUTDOWN GPIO_TO_PIN(3, 9)
#define E2_GPIO_LED_OE		GPIO_TO_PIN(3,17)
#define E2_GPIO_PSE_SHUTDOWN	GPIO_TO_PIN(3, 20)
/* special GPIOs, handled elsewhere */
#define E2_GPIO_BOOT0_E1	GPIO_TO_PIN(3, 4)
#define E2_GPIO_PMIC_INT	GPIO_TO_PIN(0, 28)
#define E2_GPIO_RS485_DE	GPIO_TO_PIN(2, 17)

#ifdef CONFIG_PIAAM335X_PROTOTYPE
#define E2_GPIO_WARTUNG_R1	GPIO_TO_PIN(0, 16)
#define E2_GPIO_KSB_TERM2	GPIO_TO_PIN(1, 30)
#define E2_GPIO_RESERVE1	GPIO_TO_PIN(0, 17)
#define E2_GPIO_CAN2_INT	GPIO_TO_PIN(3, 20)
/* pinmux for special gpios */
static struct pinmux_config km_e2_rev1_gpios_pin_mux[] = {
	/* PMIC INT */
	{ "mii1_txd0.gpio0_28",    AM33XX_PIN_INPUT_PULLUP },
	/* Ext. RESET */
	{"gpmc_clk.gpio2_1",       AM33XX_PIN_INPUT_PULLUP },
	/* USB OC */
	{"mii1_rxd1.gpio2_20",     AM33XX_PIN_INPUT_PULLUP },
	/* WD_SET1 */
	{"lcd_vsync.gpio2_22",     AM33XX_PIN_INPUT },
	/* WD_SET2 */
	{"lcd_hsync.gpio2_23",     AM33XX_PIN_INPUT_PULLDOWN },
	/* WDI */
	{"lcd_pclk.gpio2_24",      AM33XX_PIN_INPUT },
	/* 24V FAIL */
	{"lcd_ac_bias_en.gpio2_25",AM33XX_PIN_INPUT_PULLUP },
	/* LED OE */
	{"mcasp0_ahclkr.gpio3_17", AM33XX_PIN_INPUT_PULLDOWN},
	/* FRAM WP */
	{"mcasp0_ahclkx.gpio3_21", AM33XX_PIN_INPUT_PULLDOWN },
	/* CLEAR_RESET */
	{"lcd_data3.gpio2_9",      AM33XX_PIN_INPUT_PULLUP },
	/* WD_RESET */
	{"gpmc_ad14.gpio1_14",     AM33XX_PIN_INPUT_PULLDOWN },
	/* PB_RESET */
	{"mii1_col.gpio3_0",       AM33XX_PIN_INPUT_PULLDOWN },
	/* *S_ASAUS */
	{"mii1_crs.gpio3_1",       AM33XX_PIN_INPUT_PULLDOWN },
	/* 230V_A */
	{"mii1_rxerr.gpio3_2",     AM33XX_PIN_INPUT_PULLDOWN },
	/* 230V_B */
	{"mii1_rxdv.gpio3_4",      AM33XX_PIN_INPUT_PULLDOWN },
	/* Wartung */
	{"mii1_txd3.gpio0_16",     AM33XX_PIN_INPUT_PULLDOWN },
	/* KSB_TERM */
	{"mii1_rxd0.gpio2_21",     AM33XX_PIN_INPUT_PULLDOWN },
	/* KSB_TERM2 */
	{"gpmc_csn1.gpio1_30",     AM33XX_PIN_INPUT_PULLDOWN },
	/* CAN2 interrupt line MCP2515 */
	{"mcasp0_axr1.gpio3_20",   AM33XX_PIN_INPUT_PULLUP },
	/* RS485 DE */
	{"lcd_data11.gpio2_17",    AM33XX_PIN_INPUT_PULLUP},
	/* MMC0 write protect */
	{"mii1_txclk.gpio3_9",     AM33XX_PIN_INPUT_PULLUP},
	/* MMC0 card detect */
	{"mii1_txd2.gpio0_17",     AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
static struct gpio km_e2_rev1_gpios[] = {
	{ E2_GPIO_EXT_RESET,	GPIOF_OUT_INIT_HIGH, "ext_reset" },
	{ E2_GPIO_USB_OC,	GPIOF_IN, "usb_oc" },
	{ E2_GPIO_WD_SET1,	GPIOF_OUT_INIT_HIGH, "wd_set1" },
	{ E2_GPIO_WD_SET2,	GPIOF_OUT_INIT_LOW,  "wd_set2" },
	{ E2_GPIO_WDI, 		GPIOF_OUT_INIT_LOW, "wdi" },
	{ E2_GPIO_24V_FAIL,	GPIOF_IN, "24v_fail" },
	{ E2_GPIO_LED_OE,	GPIO_OUT_INIT_LOW, "led_oe" },
	{ E2_GPIO_FRAM_WP,	GPIOF_OUT_INIT_LOW,  "fram_wp" },
	{ E2_GPIO_CLEAR_RESET,	GPIOF_OUT_INIT_HIGH, "clear_reset" },
	{ E2_GPIO_WD_RESET,	GPIOF_IN, "wd_reset" },
	{ E2_GPIO_PB_RESET,	GPIOF_IN, "pb_reset" },
	{ E2_GPIO_S_ASAUS,	GPIOF_IN, "s_asaus" },
	{ E2_GPIO_230V_A,	GPIOF_IN, "230v_a" },
	{ E2_GPIO_230V_B,	GPIOF_IN, "230v_b" },
	{ E2_GPIO_WARTUNG_R1,	GPIOF_IN, "wartung" },
	{ E2_GPIO_KSB_TERM1,	GPIOF_OUT_INIT_LOW, "ksb_term1" },
	{ E2_GPIO_KSB_TERM2,	GPIOF_OUT_INIT_LOW, "ksb_term2" },
};

/* E2 rev0.02 */
static struct pinmux_config km_e2_rev2_gpios_pin_mux[] = {
	/* PMIC INT */
	{ "mii1_txd0.gpio0_28",    AM33XX_PIN_INPUT_PULLUP },
	/* Ext. RESET */
	{"gpmc_clk.gpio2_1",       AM33XX_PIN_INPUT_PULLUP },
	/* USB OC */
	{"mii1_rxd1.gpio2_20",     AM33XX_PIN_INPUT_PULLUP },
	/* WD_SET1 */
	{"lcd_vsync.gpio2_22",     AM33XX_PIN_INPUT },
	/* WD_SET2 */
	{"lcd_hsync.gpio2_23",     AM33XX_PIN_INPUT_PULLDOWN },
	/* WDI */
	{"lcd_pclk.gpio2_24",      AM33XX_PIN_INPUT },
	/* 24V FAIL */
	{"lcd_ac_bias_en.gpio2_25",AM33XX_PIN_INPUT_PULLUP },
	/* LED OE */
	{"mcasp0_ahclkr.gpio3_17", AM33XX_PIN_INPUT_PULLDOWN},
	/* FRAM WP */
	{"mcasp0_ahclkx.gpio3_21", AM33XX_PIN_INPUT_PULLDOWN },
	/* FF_CLK */
	{"mii1_rxclk.gpio3_10",    AM33XX_PIN_INPUT_PULLUP },
	/* WD_RESET */
	{"gpmc_ad14.gpio1_14",     AM33XX_PIN_INPUT_PULLDOWN },
	/* PB_RESET */
	{"mii1_col.gpio3_0",       AM33XX_PIN_INPUT_PULLDOWN },
	/* *S_ASAUS */
	{"mii1_crs.gpio3_1",       AM33XX_PIN_INPUT_PULLDOWN },
	/* 230V_A */
	{"mii1_rxerr.gpio3_2",     AM33XX_PIN_INPUT_PULLDOWN },
	/* Wartung */
	{"mii1_txd3.gpio0_16",     AM33XX_PIN_INPUT_PULLDOWN },
	/* KSB_TERM */
	{"mii1_rxd0.gpio2_21",     AM33XX_PIN_INPUT_PULLDOWN },
	/* APS_TERM */
	{"gpmc_csn2.gpio1_31",     AM33XX_PIN_INPUT_PULLDOWN },
	/* RS485 DE */
	{"lcd_data11.gpio2_17",    AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
static struct gpio km_e2_rev2_gpios[] = {
	{ E2_GPIO_EXT_RESET,	GPIOF_OUT_INIT_HIGH, "ext_reset" },
	{ E2_GPIO_USB_OC,	GPIOF_IN, "usb_oc" },
	{ E2_GPIO_WD_SET1,	GPIOF_OUT_INIT_HIGH, "wd_set1" },
	{ E2_GPIO_WD_SET2,	GPIOF_OUT_INIT_LOW,  "wd_set2" },
	{ E2_GPIO_WDI, 		GPIOF_OUT_INIT_LOW, "wdi" },
	{ E2_GPIO_24V_FAIL,	GPIOF_IN, "24v_fail" },
	{ E2_GPIO_LED_OE,	GPIO_OUT_INIT_LOW, "led_oe" },
	{ E2_GPIO_FRAM_WP,	GPIOF_OUT_INIT_LOW,  "fram_wp" },
	{ E2_GPIO_FF_CLK,	GPIOF_OUT_INIT_HIGH, "ff_clk" },
	{ E2_GPIO_WD_RESET,	GPIOF_IN, "wd_reset" },
	{ E2_GPIO_PB_RESET,	GPIOF_IN, "pb_reset" },
	{ E2_GPIO_S_ASAUS,	GPIOF_IN, "s_asaus" },
	{ E2_GPIO_230V_A,	GPIOF_IN, "230v_test" },
	{ E2_GPIO_WARTUNG_R1,	GPIOF_IN, "wartung" },
	{ E2_GPIO_KSB_TERM1,	GPIOF_OUT_INIT_LOW, "ksb_term" },
	{ E2_GPIO_APS_TERM,	GPIOF_OUT_INIT_LOW, "aps_term" },
};
#endif /* CONFIG_PIAAM335X_PROTOTYPE */

/* E2 rev0.03 */
static struct pinmux_config km_e2_gpios_pin_mux[] = {
	/* PMIC INT */
	{ "mii1_txd0.gpio0_28",    AM33XX_PIN_INPUT_PULLUP },
	/* Ext. RESET */
	{"gpmc_clk.gpio2_1",       AM33XX_PIN_INPUT_PULLUP },
	/* USB OC */
	{"mii1_rxd1.gpio2_20",     AM33XX_PIN_INPUT_PULLUP },
	/* WD_SET1 */
	{"lcd_vsync.gpio2_22",     AM33XX_PIN_INPUT },
	/* WD_SET2 */
	{"lcd_hsync.gpio2_23",     AM33XX_PIN_INPUT_PULLDOWN },
	/* WDI */
	{"lcd_pclk.gpio2_24",      AM33XX_PIN_INPUT },
	/* 24V FAIL */
	{"lcd_ac_bias_en.gpio2_25",AM33XX_PIN_INPUT_PULLUP },
	/* LED OE */
	{"mcasp0_ahclkr.gpio3_17", AM33XX_PIN_INPUT_PULLDOWN},
	/* FRAM WP */
	{"mcasp0_ahclkx.gpio3_21", AM33XX_PIN_INPUT_PULLDOWN },
	/* FF_CLK */
	{"mii1_rxclk.gpio3_10",    AM33XX_PIN_INPUT_PULLUP },
	/* WD_RESET */
	{"gpmc_ad14.gpio1_14",     AM33XX_PIN_INPUT_PULLDOWN },
	/* PB_RESET */
	{"mii1_col.gpio3_0",       AM33XX_PIN_INPUT_PULLDOWN },
	/* *S_ASAUS */
	{"mii1_crs.gpio3_1",       AM33XX_PIN_INPUT_PULLDOWN },
	/* 230V_A */
	{"mii1_rxerr.gpio3_2",     AM33XX_PIN_INPUT_PULLDOWN },
	/* Wartung */
	{"mii1_txen.gpio3_3",      AM33XX_PIN_INPUT_PULLDOWN },
	/* KSB_TERM */
	{"mii1_rxd0.gpio2_21",     AM33XX_PIN_INPUT_PULLDOWN },
	/* APS_TERM */
	{"gpmc_csn2.gpio1_31",     AM33XX_PIN_INPUT_PULLDOWN },
	/* RS485 DE */
	{"lcd_data11.gpio2_17",    AM33XX_PIN_INPUT_PULLUP},
	/* PoE_PS_Shutdown */
	{"mii1_txclk.gpio3_9",     AM33XX_PIN_INPUT_PULLUP},
	/* PSE_Shutdown */
	{"mcasp0_axr.gpio3_20",    AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
static struct gpio km_e2_gpios[] = {
	{ E2_GPIO_EXT_RESET,	GPIOF_OUT_INIT_HIGH, "ext_reset" },
	{ E2_GPIO_USB_OC,	GPIOF_IN, "usb_oc" },
	{ E2_GPIO_WD_SET1,	GPIOF_OUT_INIT_HIGH, "wd_set1" },
	{ E2_GPIO_WD_SET2,	GPIOF_OUT_INIT_LOW,  "wd_set2" },
	{ E2_GPIO_WDI, 		GPIOF_OUT_INIT_LOW, "wdi" },
	{ E2_GPIO_24V_FAIL,	GPIOF_IN, "24v_fail" },
	{ E2_GPIO_LED_OE,	GPIOF_OUT_INIT_LOW, "led_oe" },
	{ E2_GPIO_FRAM_WP,	GPIOF_OUT_INIT_LOW,  "fram_wp" },
	{ E2_GPIO_FF_CLK,	GPIOF_OUT_INIT_HIGH, "ff_clk" },
	{ E2_GPIO_WD_RESET,	GPIOF_IN, "wd_reset" },
	{ E2_GPIO_PB_RESET,	GPIOF_IN, "pb_reset" },
	{ E2_GPIO_S_ASAUS,	GPIOF_IN, "s_asaus" },
	{ E2_GPIO_230V_A,	GPIOF_IN, "230v_test" },
	{ E2_GPIO_WARTUNG,	GPIOF_IN, "wartung" },
	{ E2_GPIO_KSB_TERM1,	GPIOF_OUT_INIT_LOW, "ksb_term" },
	{ E2_GPIO_APS_TERM,	GPIOF_OUT_INIT_LOW, "aps_term" },
	{ E2_GPIO_POE_PS_SHUTDOWN, GPIOF_OUT_INIT_HIGH, "poe_ps_shutdown" },
	{ E2_GPIO_PSE_SHUTDOWN,	GPIOF_OUT_INIT_LOW, "pse_shutdown" },
};

/* MMI rev 0.x */
#define MMI_GPIO_PMIC_INT	GPIO_TO_PIN(2, 1)
#define MMI_GPIO_PMIC_SLEEP	GPIO_TO_PIN(3,16)
/* MMI: Watchdog */
#define MMI_GPIO_WDI		GPIO_TO_PIN(1, 0)
#define MMI_GPIO_WD_SET1	GPIO_TO_PIN(1, 1)
#define MMI_GPIO_WD_SET2	GPIO_TO_PIN(1, 2)
/* MMC */
#define MMI_GPIO_MMC_CD		GPIO_TO_PIN(0, 3)
/* 3G ACC */
#define MMI_GPIO_ACC_INT1	GPIO_TO_PIN(3,19)
#define MMI_GPIO_ACC_INT2	GPIO_TO_PIN(0, 7)
/* Monitoring */
#define MMI_GPIO_3V3_FAIL	GPIO_TO_PIN(3,20)
#define MMI_GPIO_USB_OC		GPIO_TO_PIN(0, 5)
#define MMI_GPIO_LED1		GPIO_TO_PIN(0, 30)
#define MMI_GPIO_LED2		GPIO_TO_PIN(0, 31)
/* MMI: LCD GPIOs */
#define MMI_GPIO_LCD_DISP	GPIO_TO_PIN(1,28)
#define MMI_GPIO_LCD_BACKLIGHT	GPIO_TO_PIN(3,17)
#define MMI_GPIO_LCD_PENDOWN	GPIO_TO_PIN(2, 0)
/* MMI: 24V IO */
#define MMI_GPIO_OUT1		GPIO_TO_PIN(1,24)
#define MMI_GPIO_OUT2		GPIO_TO_PIN(1,17)
#define MMI_GPIO_OUT3		GPIO_TO_PIN(1,18)
#define MMI_GPIO_OUT4		GPIO_TO_PIN(1,19)
#define MMI_GPIO_IN1		GPIO_TO_PIN(1,20)
#define MMI_GPIO_IN2		GPIO_TO_PIN(1,21)
#define MMI_GPIO_IN3		GPIO_TO_PIN(1,22)
#define MMI_GPIO_IN4		GPIO_TO_PIN(1,23)

static struct pinmux_config km_mmi_gpios_pin_mux[] = {
	/* PMIC INT */
	{ "gpmc_clk.gpio2_1",		AM33XX_PIN_INPUT_PULLUP },
	/* PMIC SLEEP 3_16 - not currently used */
	{ "mcasp0_axr0.gpio3_16",	AM33XX_PIN_INPUT_PULLUP },
	/* WDI        1_0 */
	{ "gpmc_ad0.gpio1_0",		AM33XX_PIN_INPUT},
	/* WD_SET1  1_1 - external pullup */
	{ "gpmc_ad1.gpio1_1",		AM33XX_PIN_INPUT},
	/* WD_SET2	1_2 */
	{ "gpmc_ad2.gpio1_2",		AM33XX_PIN_INPUT_PULLDOWN},
	/* 3.3V_Fail 3_20 */
	{ "mcasp0_axr1.gpio3_20",	AM33XX_PIN_INPUT_PULLUP},
	/* USB OC */
	{ "spi0_cs0.gpio0_5",		AM33XX_PIN_INPUT_PULLUP },
	/* LED1 */
	{ "gpmc_wait0.gpio0_30",	AM33XX_PIN_INPUT_PULLUP},
	/* LED2 */
	{ "gpmc_wpn.gpio0_31",		AM33XX_PIN_INPUT},
	/* ACC INT1 */
	{"mcasp0_fsr.gpio3_19",		AM33XX_PIN_INPUT_PULLUP},
	/* ACC INT2 */
	{"ecap0_in_pwm0_out.gpio0_7",	AM33XX_PIN_INPUT_PULLUP},
	/* touch INT */
	{ "gpmc_csn3.gpio2_0",		AM33XX_PIN_INPUT_PULLUP},
	/* MMC CD */
	{ "spi0_d0.gpio0_3",		AM33XX_PIN_INPUT_PULLUP },
	/* 24V IOs - external pulls for safe default mode */
	{ "gpmc_a1.gpio1_17",		AM33XX_PIN_INPUT },
	{ "gpmc_a2.gpio1_18",		AM33XX_PIN_INPUT },
	{ "gpmc_a3.gpio1_19",		AM33XX_PIN_INPUT },
	{ "gpmc_a4.gpio1_20",		AM33XX_PIN_INPUT },
	{ "gpmc_a5.gpio1_21",		AM33XX_PIN_INPUT },
	{ "gpmc_a6.gpio1_22",		AM33XX_PIN_INPUT },
	{ "gpmc_a7.gpio1_23",		AM33XX_PIN_INPUT },
	{ "gpmc_a8.gpio1_24",		AM33XX_PIN_INPUT },
	/* display enable GPIO */
	{ "gpmc_ben1.gpio1_28",		AM33XX_PIN_INPUT_PULLDOWN },
	/* backlight GPIO */
	{ "mcasp0_ahclkr.gpio3_17",	AM33XX_PIN_INPUT_PULLDOWN },
	{NULL, 0},
};

static struct gpio km_mmi_gpios[] = {
	{ MMI_GPIO_3V3_FAIL,	GPIOF_IN, "3v3_fail" },
	{ MMI_GPIO_USB_OC,	GPIOF_IN, "usb_oc" },
	{ MMI_GPIO_WD_SET1,	GPIOF_OUT_INIT_HIGH, "wd_set1" },
	/* REVISIT keeps the watchdog disabled, change when implemented */
	{ MMI_GPIO_WD_SET2,	GPIOF_OUT_INIT_LOW, "wd_set2" },
	{ MMI_GPIO_WDI,		GPIOF_OUT_INIT_LOW, "wdi" },
	{ MMI_GPIO_LCD_DISP,	GPIOF_OUT_INIT_LOW, "lcd:den" },
	{ MMI_GPIO_LCD_BACKLIGHT, GPIOF_OUT_INIT_LOW, "lcd:blen" },
};

static struct gpio km_mmi_24v_gpios[] = {
	{ MMI_GPIO_OUT1,	GPIOF_OUT_INIT_LOW, "out1" },
	{ MMI_GPIO_OUT2,	GPIOF_OUT_INIT_LOW, "out2" },
	{ MMI_GPIO_OUT3,	GPIOF_OUT_INIT_LOW, "out3" },
	{ MMI_GPIO_OUT4,	GPIOF_OUT_INIT_LOW, "out4" },
	{ MMI_GPIO_IN1,		GPIOF_IN, "in1" },
	{ MMI_GPIO_IN2,		GPIOF_IN, "in2" },
	{ MMI_GPIO_IN3,		GPIOF_IN, "in3" },
	{ MMI_GPIO_IN4,		GPIOF_IN, "in4" },
};

/* PM */
#define PM_GPIO_PMIC_INT	GPIO_TO_PIN(0, 21)
#define PM_GPIO_LED1		GPIO_TO_PIN(1, 29)
#define PM_GPIO_EMMC_RESET	GPIO_TO_PIN(2,  5)
#define PM_GPIO_NOR_WPN		GPIO_TO_PIN(2, 20)
#define PM_GPIO_NOR_RESET	GPIO_TO_PIN(2, 21)
static struct pinmux_config pm_gpios_pin_mux[] = {
	/* PMIC INT */
	{ "mii1_txd1.gpio0_21",		AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_csn0.gpio1_29",		AM33XX_PIN_OUTPUT },
	{ "gpmc_ben0_cle.gpio2_5",	AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxd1.gpio2_20",		AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxd0.gpio2_21",		AM33XX_PIN_INPUT_PULLUP },
	{NULL, 0},
};
static struct gpio pm_gpios[] = {
	{ PM_GPIO_EMMC_RESET,	GPIOF_OUT_INIT_HIGH, "emmc_reset" },
	{ PM_GPIO_NOR_WPN,	GPIOF_OUT_INIT_HIGH, "nor_wpn" },
	{ PM_GPIO_NOR_RESET,	GPIOF_OUT_INIT_HIGH, "nor_reset" },
};

/* EB_TFT */
#define EBTFT_GPIO_IN2		GPIO_TO_PIN(0, 28)
#define EBTFT_GPIO_IN1		GPIO_TO_PIN(2,  2)
#define EBTFT_GPIO_CAN0_TERM	GPIO_TO_PIN(2, 18)
#define EBTFT_GPIO_IN3		GPIO_TO_PIN(3, 10)
#define EBTFT_GPIO_LED		GPIO_TO_PIN(3, 17)
#define EBTFT_GPIO_MMC_CD	GPIO_TO_PIN(3, 21)
#define EBTFT_GPIO_RFID_IRQ	GPIO_TO_PIN(3,  4)
#define EBTFT_GPIO_RFID_POWEN	GPIO_TO_PIN(3,  0)
#define EBTFT_GPIO_LCD_BACKLIGHT GPIO_TO_PIN(2, 0)
#define EBTFT_GPIO_LCD_DISP	GPIO_TO_PIN(2,  4)
static struct pinmux_config ebtft_gpios_pin_mux[] = {
	{ "mii1_txd0.gpio0_28",		AM33XX_PIN_INPUT },
	{ "gpmc_advn_ale.gpio2_2",	AM33XX_PIN_INPUT },
	{ "mii1_rxclk.gpio3_10",	AM33XX_PIN_INPUT },
	{ "mcasp0_ahclkr.gpio3_17",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_rxd3.gpio2_18",		AM33XX_PIN_INPUT_PULLDOWN },
	{ "mcasp0_ahclkx.gpio3_21",	AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_rxdv.gpio3_4",		AM33XX_PIN_INPUT_PULLUP },
	{ "mii1_col.gpio3_0",		AM33XX_PIN_INPUT_PULLDOWN },
	/* LCD Backlight Enable */
	{ "gpmc_csn3.gpio2_0",		AM33XX_PIN_INPUT_PULLDOWN },
	/* LCD Display Enable */
	{ "gpmc_wen.gpio2_4",		AM33XX_PIN_INPUT_PULLDOWN },

	{ NULL, 0 },
};
static struct gpio ebtft_gpios[] = {
	{ EBTFT_GPIO_CAN0_TERM,	GPIOF_OUT_INIT_LOW, "can_term" },
	{ EBTFT_GPIO_IN1,	GPIOF_IN, "in1" },
	{ EBTFT_GPIO_IN2,	GPIOF_IN, "in2" },
	{ EBTFT_GPIO_IN3,	GPIOF_IN, "in3" },
	{ EBTFT_GPIO_RFID_IRQ,	GPIOF_IN, "rfid_int" },
	{ EBTFT_GPIO_RFID_POWEN,GPIOF_OUT_INIT_LOW, "rfid_powen" },
	{ EBTFT_GPIO_LCD_DISP,	GPIOF_OUT_INIT_LOW, "lcd:den" },
	{ EBTFT_GPIO_LCD_BACKLIGHT, GPIOF_OUT_INIT_LOW, "lcd:blen" },
};

/* SK */
#define SK_GPIO_LED2		GPIO_TO_PIN(3, 17)
#define SK_GPIO_LED3		GPIO_TO_PIN(3, 18)
#define SK_GPIO_MMC_CD		GPIO_TO_PIN(3, 20)
static struct pinmux_config sk_gpios_pin_mux[] = {
	{ "mcasp0_ahclkr.gpio3_17",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mcasp0_aclkr.gpio3_18",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mcasp0_axr1.gpio3_20",	AM33XX_PIN_INPUT_PULLUP },
	{ NULL, 0 },
};
static struct gpio sk_gpios[] = {

};

/* APC */
#define APC_GPIO_BT_EN		GPIO_TO_PIN(0, 7)
#define APC_GPIO_WLAN_IRQ	GPIO_TO_PIN(0, 22)
#define APC_GPIO_IN_FAULT	GPIO_TO_PIN(0, 23)
#define APC_GPIO_WLAN_EN	GPIO_TO_PIN(0, 27)
#define APC_GPIO_BAT_PWR	GPIO_TO_PIN(2,  3)
#define APC_GPIO_GSM_STATUS	GPIO_TO_PIN(3,  3)
#define APC_GPIO_LED2		GPIO_TO_PIN(3,  4)
#define APC_GPIO_CAN_TERM1	GPIO_TO_PIN(3, 14)
#define APC_GPIO_CAN_TERM0	GPIO_TO_PIN(3, 15)
#define APC_GPIO_RS485_DE1	GPIO_TO_PIN(3, 16)
#define APC_GPIO_GSM_RI		GPIO_TO_PIN(3, 17)
#define APC_GPIO_GSM_PWRKEY	GPIO_TO_PIN(3, 18)
#define APC_GPIO_GSM_DTR	GPIO_TO_PIN(3, 19)
#define APC_GPIO_GSM_FLIGHT	GPIO_TO_PIN(3, 20)
/* Rev 00.02 */
#define APC_GPIO_CHRG_ACT	GPIO_TO_PIN(0, 28)
#define APC_GPIO_CHRG_FIN	GPIO_TO_PIN(0, 31)
#define APC_GPIO_CHRG_EN	GPIO_TO_PIN(2, 24)
#define APC_GPIO_GSM_PWR_EN	GPIO_TO_PIN(2,  2)
#define APC_GPIO_GSM_PWR_OK	GPIO_TO_PIN(2,  4)
#define APC_GPIO_GSM_RESET	GPIO_TO_PIN(2, 25)
#define APC_GPIO_ODOMETER	GPIO_TO_PIN(2, 12)
static struct pinmux_config apc_gpios_pin_mux[] = {
	/* IO */
	{ "gpmc_ad9.gpio0_23",		AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_oen_ren.gpio2_3",	AM33XX_PIN_OUTPUT },
	/* LED */
	{ "mii1_rxdv.gpio3_4",		AM33XX_PIN_INPUT_PULLUP },
	/* CAN */
	{ "mcasp0_aclkx.gpio3_14",	AM33XX_PIN_INPUT_PULLUP },
	{ "mcasp0_fsx.gpio3_15",	AM33XX_PIN_INPUT_PULLUP },
	/* RS485 */
	{ "mcasp0_axr0.gpio3_16",	AM33XX_PIN_INPUT_PULLDOWN },
	/* GSM */
	{ "mcasp0_ahclkr.gpio3_17",	AM33XX_PIN_INPUT_PULLUP },
	{ "mcasp0_aclkr.gpio3_18",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mcasp0_fsr.gpio3_19",	AM33XX_PIN_INPUT_PULLUP },
	{ "mcasp0_axr1.gpio3_20",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_txen.gpio3_3",		AM33XX_PIN_INPUT },
	/* Rev 00.02 */
	/* Charger */
	{ "mii1_txd0.gpio0_28",		AM33XX_PIN_INPUT },
	{ "gpmc_wpn.gpio0_31",		AM33XX_PIN_INPUT },
	{ "lcd_pclk.gpio2_24",		AM33XX_PIN_INPUT }, // special OS
	/* Odometer */
	{ "lcd_data6.gpio2_12",		AM33XX_PIN_INPUT },
	/* GSM additional power control */
	{ "gpmc_advn_ale.gpio2_2",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_wen.gpio2_4",		AM33XX_PIN_INPUT },
	{ "lcd_ac_bias_en.gpio2_25",	AM33XX_PIN_INPUT },
	{ NULL, 0 },
};
static struct gpio apc_gpios[] = {
	{ APC_GPIO_IN_FAULT,	GPIOF_IN,		"in_fault" },
	/* turn battery power on here, it is turned off in bootloader
	 * to make sure the board doesn't boot, with battery only */
	{ APC_GPIO_BAT_PWR,	GPIOF_OUT_INIT_HIGH,	"bat_pwr" },
	{ APC_GPIO_RS485_DE1,	GPIOF_OUT_INIT_LOW,	"rs485:de1" },
	{ APC_GPIO_GSM_PWRKEY,	GPIOF_OUT_INIT_LOW,	"gsm:pwrkey" },
	{ APC_GPIO_GSM_DTR,	GPIOF_OUT_INIT_LOW,	"gsm:dtr" },
	{ APC_GPIO_GSM_RI,	GPIOF_IN,		"gsm:ri" },
	{ APC_GPIO_GSM_STATUS,	GPIOF_IN,		"gsm:status" },
	/* on SIM5360 disable flight mode, when HIGH */
	{ APC_GPIO_GSM_FLIGHT,	GPIOF_OUT_INIT_HIGH,	"gsm:flight_disable" },
	{ APC_GPIO_CAN_TERM0,	GPIOF_OUT_INIT_HIGH,	"can:term0" },
	{ APC_GPIO_CAN_TERM1,	GPIOF_OUT_INIT_HIGH,	"can:term1" },
	/* Rev 00.02 */
	{ APC_GPIO_CHRG_ACT,	GPIOF_IN,		"chrg:active" },
	{ APC_GPIO_CHRG_FIN,	GPIOF_IN,		"chrg:finish" },
	{ APC_GPIO_CHRG_EN,	GPIOF_OUT_INIT_LOW,	"chrg:en" },
	{ APC_GPIO_ODOMETER,	GPIOF_IN,		"odometer" },
	/* RESET is 1.8V in rev 00.02, don't output HIGH */
	{ APC_GPIO_GSM_RESET,	GPIOF_IN,		"gsm:reset" },
	{ APC_GPIO_GSM_PWR_EN,	GPIOF_OUT_INIT_HIGH,	"gsm:power_en" },
	{ APC_GPIO_GSM_PWR_OK,	GPIOF_IN,		"gsm:power_ok" },
};

#define EM_GPIO_GSM_STATUS	GPIO_TO_PIN(0,  5)
#define EM_GPIO_PMIC_INT	GPIO_TO_PIN(0, 21)
#define EM_GPIO_WLAN_EN		GPIO_TO_PIN(0, 22)
#define EM_GPIO_RS485_DE4	GPIO_TO_PIN(0, 26)
#define EM_GPIO_DISP_RSTB	GPIO_TO_PIN(0, 28)
#define EM_GPIO_BT_UART_OE	GPIO_TO_PIN(0, 31)
#define EM_GPIO_RS485_DE3	GPIO_TO_PIN(2,  2)
#define EM_GPIO_RS485_DE2	GPIO_TO_PIN(2,  3)
#define EM_GPIO_RS485_DE1	GPIO_TO_PIN(2,  4)
#define EM_GPIO_EMMC_RESET	GPIO_TO_PIN(2,  5)
#define EM_GPIO_GSM_RI		GPIO_TO_PIN(2,  6)
#define EM_GPIO_WLAN_IRQ	GPIO_TO_PIN(2,  7)
#define EM_GPIO_BT_EN		GPIO_TO_PIN(2,  8)
#define EM_GPIO_MMC_CD		GPIO_TO_PIN(2, 12)
#define EM_GPIO_SC_RESET	GPIO_TO_PIN(2, 22) /* LPC11 reset */
#define EM_GPIO_GSM_PWRKEY	GPIO_TO_PIN(2, 23)
#define EM_GPIO_SC_BOOTLDR	GPIO_TO_PIN(2, 24) /* LPC11 boot mode */
#define EM_GPIO_GSM_EMERG_OFF	GPIO_TO_PIN(2, 25)
#define EM_GPIO_FOIL_IRQ	GPIO_TO_PIN(3, 17)
#define EM_GPIO_S0		GPIO_TO_PIN(1, 29)
#define EM_GPIO_USB_OSC1	GPIO_TO_PIN(2, 16)
#define EM_GPIO_USB_OSC2	GPIO_TO_PIN(2, 17)
static struct pinmux_config em_gpios_pin_mux[] = {
	/* MMC CD */
	{ "lcd_data6.gpio2_12",		AM33XX_PIN_INPUT_PULLUP },
	/* Smartcard Controller */
	{ "lcd_vsync.gpio2_22",		AM33XX_PIN_OUTPUT },
	{ "lcd_pclk.gpio2_24",		AM33XX_PIN_OUTPUT },
	/* PMIC */
	{ "mii1_txd1.gpio0_21",		AM33XX_PIN_INPUT_PULLUP },
	/* IO Expander */
	{ "mcasp0_ahclkr.gpio3_17",	AM33XX_PIN_INPUT_PULLUP },
	/* Display (low active) */
	{ "mii1_txd0.gpio0_28",		AM33XX_PIN_OUTPUT },
	/* RS485 */
	{ "gpmc_ad10.gpio0_26",		AM33XX_PIN_OUTPUT },
	{ "gpmc_advn_ale.gpio2_2",	AM33XX_PIN_OUTPUT },
	{ "gpmc_oen_ren.gpio2_3",	AM33XX_PIN_OUTPUT },
	{ "gpmc_wen.gpio2_4",		AM33XX_PIN_OUTPUT },
	/* S0 */
	{ "gpmc_csn0.gpio1_29",	AM33XX_PIN_INPUT },
	/* USB OSC */
	{ "lcd_data10.gpio2_16",	AM33XX_PIN_INPUT_PULLUP },
	{ "lcd_data11.gpio2_17",	AM33XX_PIN_INPUT_PULLUP },
	/* GSM */
	{ "lcd_data0.gpio2_6",		AM33XX_PIN_INPUT_PULLUP },
	{ "lcd_hsync.gpio2_23",		AM33XX_PIN_INPUT_PULLDOWN },
	{ "lcd_ac_bias_en.gpio2_25",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "spi0_cs0.gpio0_5",		AM33XX_PIN_INPUT },
	/* EMMC */
	{ "gpmc_ben0_cle.gpio2_5",	AM33XX_PIN_INPUT_PULLUP },
	/* WLAN/BT */
	{ "gpmc_wpn.gpio0_31",		AM33XX_PIN_OUTPUT },
	{ NULL, 0 },
};
static struct gpio em_gpios[] = {
	{ EM_GPIO_SC_RESET,	GPIOF_OUT_INIT_LOW,	"sc:reset" },
	{ EM_GPIO_SC_BOOTLDR,	GPIOF_OUT_INIT_HIGH,	"sc:bootldr" },
	{ EM_GPIO_DISP_RSTB,	GPIOF_OUT_INIT_HIGH,	"disp:rstb" },
	{ EM_GPIO_RS485_DE1,	GPIOF_OUT_INIT_LOW,	"rs485:de1" },
	{ EM_GPIO_RS485_DE2,	GPIOF_OUT_INIT_LOW,	"rs485:de2" },
	{ EM_GPIO_RS485_DE3,	GPIOF_OUT_INIT_LOW,	"rs485:de3" },
	{ EM_GPIO_RS485_DE4,	GPIOF_OUT_INIT_HIGH,	"rs485:de4" },
	{ EM_GPIO_S0,		GPIOF_IN,		"s0" },
	{ EM_GPIO_USB_OSC1,	GPIOF_IN,		"usb:osc1" },
	{ EM_GPIO_USB_OSC2,	GPIOF_IN,		"usb:osc2" },
	{ EM_GPIO_GSM_EMERG_OFF, GPIOF_OUT_INIT_LOW,	"gsm:emerg_off" },
	{ EM_GPIO_GSM_PWRKEY,	GPIOF_OUT_INIT_LOW,	"gsm:pwrkey" },
	{ EM_GPIO_GSM_RI,	GPIOF_IN,		"gsm:ri" },
	/* since rev 0.02, nc in 0.01 */
	{ EM_GPIO_GSM_STATUS,	GPIOF_IN,		"gsm:status" },
	/* TODO this is ignored by the EMMC; on Rev 0.1 this is connected to
	 * OE of the level shifter for the BT module */
	{ EM_GPIO_EMMC_RESET,	GPIOF_OUT_INIT_LOW,	"emmc:reset" },
	/*{ EM_GPIO_BT_EN,	GPIOF_OUT_INIT_HIGH,	"bt:en" },*/
	{ EM_GPIO_BT_UART_OE,	GPIOF_OUT_INIT_HIGH,	"bt:uart_oe" },
};

#ifdef CONFIG_PIAAM335X_PROTOTYPE
static void pia_print_gpio_state(const char *msg, int gpio, int on)
{
	int val = gpio_get_value(gpio);
	if (val >= 0)
		pr_info("  %s: %s\n", msg, (val == on ? "ON!" : "OFF."));
	else
		pr_warn("  %s: Unable to read GPIO!\n", msg);
}
#endif
static void pia335x_gpios_export(struct gpio *gpiocfg, int count)
{
	int i;

	if (gpiocfg == 0)
		return;

	for (i = 0; i < count; ++i) {
		if (gpio_request_one(gpiocfg[i].gpio,
				gpiocfg[i].flags,
				gpiocfg[i].label) < 0) {
			pr_err("Failed to request gpio: %s\n",
					gpiocfg[i].label);
			return;
		}
		pr_info("piA335x: GPIO init %s\n", gpiocfg[i].label);
		gpio_export(gpiocfg[i].gpio, 1);
#ifdef CONFIG_PIAAM335X_PROTOTYPE
		pia_print_gpio_state(gpiocfg[i].label, gpiocfg[i].gpio, 1);
#endif
	}
}

static void pia335x_gpios_init(int boardid)
{
	int sz;
	struct pinmux_config *muxcfg;
	struct gpio *gpiocfg;

	pr_info("pia335x_init: GPIOs: %d\n", boardid);

	switch (boardid) {
	case PIA335_KM_E2:
#ifdef CONFIG_PIAAM335X_PROTOTYPE
		if (am33xx_piarev == 1) {
			muxcfg = km_e2_rev1_gpios_pin_mux;
			gpiocfg = km_e2_rev1_gpios;
			sz = ARRAY_SIZE(km_e2_rev1_gpios);
		} else if (am33xx_piarev == 2) {
			muxcfg = km_e2_rev2_gpios_pin_mux;
			gpiocfg = km_e2_rev2_gpios;
			sz = ARRAY_SIZE(km_e2_rev2_gpios);
		} else {
#endif
			muxcfg = km_e2_gpios_pin_mux;
			gpiocfg = km_e2_gpios;
			sz = ARRAY_SIZE(km_e2_gpios);
#ifdef CONFIG_PIAAM335X_PROTOTYPE
		}
#endif
		break;
	case PIA335_KM_MMI:
		muxcfg = km_mmi_gpios_pin_mux;
		gpiocfg = km_mmi_gpios;
		sz = ARRAY_SIZE(km_mmi_gpios);
		break;
	case PIA335_PM:
		muxcfg = pm_gpios_pin_mux;
		gpiocfg = pm_gpios;
		sz = ARRAY_SIZE(pm_gpios);
		break;
	case PIA335_BB_EBTFT:
		muxcfg = ebtft_gpios_pin_mux;
		gpiocfg = ebtft_gpios;
		sz = ARRAY_SIZE(ebtft_gpios);
		break;
	case PIA335_BB_SK:
		muxcfg = sk_gpios_pin_mux;
		gpiocfg = sk_gpios;
		sz = ARRAY_SIZE(sk_gpios);
		break;
	case PIA335_BB_APC:
		muxcfg = apc_gpios_pin_mux;
		gpiocfg = apc_gpios;
		sz = ARRAY_SIZE(apc_gpios);
		break;
	case PIA335_LOKISA_EM:
		muxcfg = em_gpios_pin_mux;
		gpiocfg = em_gpios;
		sz = ARRAY_SIZE(em_gpios);
		break;
	default:
		return;
	}

	setup_pin_mux(muxcfg);
	pia335x_gpios_export(gpiocfg, sz);

	/* board specific initializations */
	if (boardid == PIA335_KM_E2) {
#ifdef CONFIG_PIAAM335X_PROTOTYPE
		/* clear reset status */
		if (am33xx_piarev == 1)
			gpio_set_value(E2_GPIO_CLEAR_RESET, 0);
		else
#endif
			gpio_set_value(E2_GPIO_FF_CLK, 1);
	}
}

#ifdef CONFIG_PIAAM335X_PROTOTYPE
/** CLKOUT2 */
#define SYS_CLKOUT2_PARENT	"lcd_gclk" /* 24MHz */
static void clkout2_12m_enable(void)
{
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

static void clkout2_32k_enable(void)
{
	/* code to enable default 32k clock output*/
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);

	setup_pin_mux(clkout2_pin_mux);
}
#endif

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
	pr_info("piA335x: %s\n", __func__);

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

/* USB */
static struct pinmux_config usb0_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus", AM33XX_PIN_OUTPUT},
	{NULL, 0},
};
static struct pinmux_config usb1_pin_mux[] = {
	{"usb1_drvvbus.usb1_drvvbus", AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void usb_init(int boardid)
{
	pr_info("piA335x: %s\n", __func__);
	switch (boardid) {
	case PIA335_BB_EBTFT:
	case PIA335_BB_SK:
		setup_pin_mux(usb1_pin_mux);
		/* fall-trough for USB0 */
	case PIA335_KM_E2:
		if (pia335x_main_id.rev == 1) {
#ifdef CONFIG_PIAAM335X_PROTOTYPE
			clkout2_12m_enable();
#endif
			setup_pin_mux(usb1_pin_mux);
		}
	case PIA335_BB_APC:
	case PIA335_KM_MMI:
		setup_pin_mux(usb0_pin_mux);
		break;
	}
}

/* Ethernet */
/* MMI Ethernet MII1 + MDIO */
static struct pinmux_config mii1_pin_mux[] = {
	{ "mii1_txen.mii1_txen",	AM33XX_PIN_OUTPUT },
	{ "mii1_rxdv.mii1_rxdv",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_txd3.mii1_txd3",	AM33XX_PIN_OUTPUT },
	{ "mii1_txd2.mii1_txd2",	AM33XX_PIN_OUTPUT },
	{ "mii1_txd1.mii1_txd1",	AM33XX_PIN_OUTPUT },
	{ "mii1_txd0.mii1_txd0",	AM33XX_PIN_OUTPUT },
	{ "mii1_txclk.mii1_txclk",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_rxclk.mii1_rxclk",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_rxd3.mii1_rxd3",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_rxd2.mii1_rxd2",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_rxd1.mii1_rxd1",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mii1_rxd0.mii1_rxd0",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mdio_data.mdio_data",	AM33XX_PIN_INPUT_PULLUP },
	{ "mdio_clk.mdio_clk",		AM33XX_PIN_INPUT_PULLUP },
	{ NULL, 0 },
};
/* optional signals used on EBTFT */
static struct pinmux_config ebtft_mii2_opt_pin_mux[] = {
	{ "gpmc_wait0.mii2_crs", AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_wpn.mii2_rxerr", AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_ben1.mii2_col", AM33XX_PIN_INPUT_PULLDOWN },
	{ NULL, 0 },
};
static struct pinmux_config em_mii2_opt_pin_mux[] = {
	{ "gpmc_wait0.mii2_crs", AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_ben1.mii2_col", AM33XX_PIN_INPUT_PULLDOWN },
	{ NULL, 0 },
};
/* E2 Ethernet MII2 + MDIO */
static struct pinmux_config mii2_base_pin_mux[] = {
	{ "gpmc_a0.mii2_txen",	AM33XX_PIN_OUTPUT },
	{ "gpmc_a1.mii2_rxdv",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_a2.mii2_txd3",	AM33XX_PIN_OUTPUT },
	{ "gpmc_a3.mii2_txd2",	AM33XX_PIN_OUTPUT },
	{ "gpmc_a4.mii2_txd1",	AM33XX_PIN_OUTPUT },
	{ "gpmc_a5.mii2_txd0",	AM33XX_PIN_OUTPUT },
	{ "gpmc_a6.mii2_txclk",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_a7.mii2_rxclk",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_a8.mii2_rxd3",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_a9.mii2_rxd2",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_a10.mii2_rxd1",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_a11.mii2_rxd0",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "mdio_data.mdio_data",AM33XX_PIN_INPUT_PULLUP },
	{ "mdio_clk.mdio_clk",	AM33XX_PIN_OUTPUT_PULLUP },
	{ NULL, 0 },
};

static void ethernet_init(int boardid)
{
	pr_info("piA335x: %s\n", __func__);
	switch (boardid) {
	case PIA335_BB_EBTFT:
	case PIA335_BB_SK:
		setup_pin_mux(mii2_base_pin_mux);
		setup_pin_mux(ebtft_mii2_opt_pin_mux);
		break;
	case PIA335_KM_E2:
		setup_pin_mux(mii2_base_pin_mux);
		break;
	case PIA335_LOKISA_EM:
	case PIA335_BB_APC:
		setup_pin_mux(mii2_base_pin_mux);
		setup_pin_mux(em_mii2_opt_pin_mux);
		break;
	case PIA335_KM_MMI:
		setup_pin_mux(mii1_pin_mux);
		break;
	}
}

/* MMC */
static struct pinmux_config pia335x_mmc0_pin_mux[] = {
	{ "mmc0_dat3.mmc0_dat3",	AM33XX_PIN_INPUT_PULLUP },
	{ "mmc0_dat2.mmc0_dat2",	AM33XX_PIN_INPUT_PULLUP },
	{ "mmc0_dat1.mmc0_dat1",	AM33XX_PIN_INPUT_PULLUP },
	{ "mmc0_dat0.mmc0_dat0",	AM33XX_PIN_INPUT_PULLUP },
	{ "mmc0_clk.mmc0_clk",		AM33XX_PIN_INPUT_PULLUP },
	{ "mmc0_cmd.mmc0_cmd",		AM33XX_PIN_INPUT_PULLUP },
	{ NULL, 0 },
};

static struct pinmux_config pm_mmc1_pin_mux[] = {
	{ "gpmc_ad0.mmc1_dat0",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad1.mmc1_dat1",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad2.mmc1_dat2",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad3.mmc1_dat3",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad4.mmc1_dat4",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad5.mmc1_dat5",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad6.mmc1_dat6",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad7.mmc1_dat7",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_csn1.mmc1_clk",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_csn2.mmc1_cmd",	AM33XX_PIN_INPUT_PULLUP },
	{ NULL, 0 },
};

static struct pinmux_config mmc2_base_pin_mux[] = {
	{ "gpmc_ad12.mmc2_dat0",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad13.mmc2_dat1",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad14.mmc2_dat2",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_ad15.mmc2_dat3",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_csn3.mmc2_cmd",	AM33XX_PIN_INPUT_PULLUP },
	{ "gpmc_clk.mmc2_clk",	AM33XX_PIN_INPUT_PULLUP },
	{ NULL, 0 },
};
static struct pinmux_config em_mmc2_extra_pin_mux[] = {
	/* WLAN/BT */
	{ "gpmc_ad8.gpio0_22",		AM33XX_PIN_OUTPUT },
	{ "lcd_data1.gpio2_7",		AM33XX_PIN_INPUT_PULLUP },
	{ "lcd_data2.gpio2_8",		AM33XX_PIN_OUTPUT },
	/* { "xdma_event_intr1.clkout2",	AM33XX_PIN_OUTPUT },*/
	{ NULL, 0 },
};
static struct pinmux_config apc_mmc2_extra_pin_mux[] = {
	{ "gpmc_ad11.gpio0_27",		AM33XX_PIN_INPUT_PULLDOWN },
	{ "ecap0_in_pwm0_out.gpio0_7",	AM33XX_PIN_INPUT_PULLDOWN },
	{ "gpmc_ad8.gpio0_22",		AM33XX_PIN_INPUT_PULLUP },
	{ NULL, 0 },
};
static struct omap2_hsmmc_info pia335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = -1,
		.gpio_wp        = -1,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
		.nonremovable = true,
	},
	{
		.mmc            = 0,	/* will be set if needed */
	},
	{
		.mmc            = 0,	/* will be set if needed */
	},
	{}      /* Terminator */
};

/* WL12xx */
#include <linux/wl12xx.h>
static struct wl12xx_platform_data wl12xx_data = {
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4Mhz */
};

static void wl12xx_prepare(int boardid)
{
	int idx = 2; /* pia335x_mmc array index */
	pia335x_mmc[idx].mmc = 3;
	pia335x_mmc[idx].name = "wl1271";

	switch (boardid) {
	case PIA335_LOKISA_EM:
		setup_pin_mux(mmc2_base_pin_mux);
		setup_pin_mux(em_mmc2_extra_pin_mux);
		if (pia335x_main_id.rev > 1) {
			pia335x_mmc[idx].name = "wl18xx";
			wl12xx_data.board_ref_clock = WL12XX_REFCLOCK_26;
		}
		wl12xx_data.irq = OMAP_GPIO_IRQ(EM_GPIO_WLAN_IRQ);
		wl12xx_data.bt_enable_gpio = EM_GPIO_BT_EN;
		wl12xx_data.wlan_enable_gpio = EM_GPIO_WLAN_EN;

		break;
	case PIA335_BB_APC:
		setup_pin_mux(mmc2_base_pin_mux);
		setup_pin_mux(apc_mmc2_extra_pin_mux);
		if (pia335x_main_id.rev > 1) {
			pia335x_mmc[idx].name = "wl18xx";
			wl12xx_data.board_ref_clock = WL12XX_REFCLOCK_26;
		}
		wl12xx_data.irq = OMAP_GPIO_IRQ(APC_GPIO_WLAN_IRQ);
		wl12xx_data.bt_enable_gpio = APC_GPIO_BT_EN;
		wl12xx_data.wlan_enable_gpio = APC_GPIO_WLAN_EN;

		break;
	default:
		return;
	}
	/* WL12xx */
	pia335x_mmc[idx].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD;
	pia335x_mmc[idx].gpio_cd = -EINVAL;
	pia335x_mmc[idx].gpio_wp = -EINVAL;
	pia335x_mmc[idx].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34;
	pia335x_mmc[idx].nonremovable = true;
}

static int ebtft_mmc0_cd(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* high active CD */
	int cd = gpio_get_value_cansleep(mmc->slots[0].switch_pin);
	pr_info("EBTFT MMCCD: %d", cd);
	return (cd);
}
static int ebtft_mmccd_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
						struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 (external) Card detect */
	if (pdev->id == 0)
		pdata->slots[0].card_detect = ebtft_mmc0_cd;

	return ret;
}

static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	if (on) {
		gpio_direction_output(wl12xx_data.wlan_enable_gpio, 1);
		mdelay(70);
	} else {
		gpio_direction_output(wl12xx_data.wlan_enable_gpio, 0);
	}

	return 0;
}

static void wl12xx_init(int devid)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;

	int status = gpio_request(wl12xx_data.bt_enable_gpio, "bt_en");
	pr_info("piA335x: %s\n", __func__);

	if (status < 0)
		pr_err("Failed to request gpio for bt_enable");
	gpio_export(wl12xx_data.bt_enable_gpio, false);

	gpio_direction_output(wl12xx_data.bt_enable_gpio, 1);


	if (wl12xx_set_platform_data(&wl12xx_data))
		pr_err("error setting wl12xx data\n");

	dev = pia335x_mmc[devid].dev;
	if (!dev) {
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}

	ret = gpio_request_one(wl12xx_data.wlan_enable_gpio,
		GPIOF_OUT_INIT_LOW, "wlan_en");
	if (ret) {
		pr_err("Error requesting wlan enable gpio: %d\n", ret);
		goto out;
	}


	pdata->slots[0].set_power = wl12xx_set_power;
out:
	return;
}

static __init void mmc_extra_init(struct device *dev, int id)
{
	/* extra init for MMC slot run after omap2_hsmmc_init */
	struct omap_mmc_platform_data *pd;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	if (!dev || !pdev)
		return;

	switch (id) {
	case PIA335_BB_EBTFT:
		if (pia335x_exp_id.rev > 2) {
			pd = dev->platform_data;
			pd->slots[0].switch_pin = -EINVAL;
			pr_info("piA335x: Disable MMC CD\n");
			break;
		} else {
			pd = dev->platform_data;
			pd->init = ebtft_mmccd_init;
		}
		break;
	case PIA335_BB_SK:
		if (pia335x_exp_id.rev == 1) {
			pd = dev->platform_data;
			pd->init = ebtft_mmccd_init;
		}
		break;
	case PIA335_LOKISA_EM:
		/* setup WL12xx module*/
		if (pdev->id == 2) /* 3rd slot, mmc2 */
			wl12xx_init(pdev->id);
		break;
	case PIA335_BB_APC:
		/* setup WL12xx module*/
		// TODO update for WL18 module on rev 2
		if (pdev->id == 2) /* 3rd slot, mmc2 */
			wl12xx_init(pdev->id);
		break;
	}
}
static void __init mmc_init(int boardid)
{
	struct pinmux_config *mux = pia335x_mmc0_pin_mux;
	struct omap2_hsmmc_info *slot;
	pr_info("piA335x: %s\n", __func__);

	switch (boardid) {
	case PIA335_KM_E2:
#ifdef CONFIG_PIAAM335X_PROTOTYPE
		if (am33xx_piarev == 1) {
			pia335x_mmc[0].gpio_cd = GPIO_TO_PIN(0, 17);
			 /* WP is GPIO_TO_PIN(3, 9) but we don't need it */
			pia335x_mmc[0].nonremovable = false;
		}
#endif
		break;
	case PIA335_KM_MMI:
		pia335x_mmc[0].gpio_cd = MMI_GPIO_MMC_CD;
		break;
	case PIA335_BB_EBTFT:
		pia335x_mmc[0].gpio_cd = EBTFT_GPIO_MMC_CD;
		break;
	case PIA335_PM:
		// enable eMMC at second MMC bus
		pia335x_mmc[1].mmc            = 2;
		pia335x_mmc[1].caps           = MMC_CAP_8_BIT_DATA;
		pia335x_mmc[1].gpio_cd        = -EINVAL;
		pia335x_mmc[1].gpio_wp        = -EINVAL;
		pia335x_mmc[1].ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34;
		pia335x_mmc[1].nonremovable = true;
		setup_pin_mux(pm_mmc1_pin_mux);
		/* don't do anything here, wait for the expansion board setup */
		/* fall trough to return */
		return;
	case PIA335_BB_SK:
		pia335x_mmc[0].gpio_cd = SK_GPIO_MMC_CD;
		break;
	case PIA335_BB_APC:
		/* uSD and eMMC from PM */
		wl12xx_prepare(boardid);
		break;
	case PIA335_LOKISA_EM:
		pia335x_mmc[0].gpio_cd = EM_GPIO_MMC_CD;
		// identical to PM
		pia335x_mmc[1].mmc            = 2;
		pia335x_mmc[1].caps           = MMC_CAP_8_BIT_DATA;
		pia335x_mmc[1].gpio_cd        = -EINVAL;
		pia335x_mmc[1].gpio_wp        = -EINVAL;
		pia335x_mmc[1].ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34;
		pia335x_mmc[1].nonremovable = true;
		setup_pin_mux(pm_mmc1_pin_mux);
		wl12xx_prepare(boardid);
		break;
	default:
		return;
	}

	setup_pin_mux(mux);
	omap2_hsmmc_init(pia335x_mmc);
	for (slot = pia335x_mmc; slot->mmc; slot++)
		mmc_extra_init(slot->dev, boardid);

	return;
}

/** LEDs */
/* E2 LEDs */
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
		.name = "reserved1",
		.default_trigger = "none",
	},
	{
		.name = "pbled2",
		.default_trigger = "default-on",
	},
	/* 10 and 11 only used in Rev 0.3 */
	{
		.name = "reserved2",
		.default_trigger = "none",
	},
	{
		.name = "led10",
		.default_trigger = "none",
	},
	{
		.name = "led11",
		.default_trigger = "none",
	},
};
static struct pca9633_platform_data km_e2_leds1_data = {
	.leds = {
		.num_leds = 8,
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

/* MMI LEDs */
static struct gpio_led km_mmi_gpio_leds[] = {
	{
		.name			= "am335x:KM_MMI:usr1",
		.gpio			= MMI_GPIO_LED1,	/* LED1 */
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "am335x:KM_MMI:usr2",
		.gpio			= MMI_GPIO_LED2,	/* LED2 */
		.default_trigger	= "mmc0",
	},
};

static struct gpio_led_platform_data km_mmi_led_info = {
	.leds		= km_mmi_gpio_leds,
	.num_leds	= ARRAY_SIZE(km_mmi_gpio_leds),
};

static struct platform_device km_mmi_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &km_mmi_led_info,
	},
};

/* EB-TFT RGB Buttons */
static struct led_info ebtft_rgbleds_config[] = {
	{
		.name = "led:b1",
		.default_trigger = "none",
	},
	{
		.name = "led:g1",
		.default_trigger = "none",
	},
	{
		.name = "led:r1",
		.default_trigger = "none",
	},
	{
		.name = "led:b2",
		.default_trigger = "none",
	},
	{
		.name = "led:g2",
		.default_trigger = "none",
	},
	{
		.name = "led:r2",
		.default_trigger = "none",
	},
};
static struct pca9633_platform_data ebtft_rgbleds_data = {
	.leds = {
		.num_leds = 6,
		.leds = ebtft_rgbleds_config,
	},
	.outdrv = PCA9633_OPEN_DRAIN,
};

static struct gpio_led ebtft_gpio_leds[] = {
	{
		.name			= "led:PM:usr1",
		.gpio			= PM_GPIO_LED1,	/* LED1 */
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "led:EBTFT:usr1",
		.gpio			= EBTFT_GPIO_LED,
		.default_trigger	= "default-on",
	},
};
static struct gpio_led_platform_data ebtft_led_info = {
	.leds		= ebtft_gpio_leds,
	.num_leds	= ARRAY_SIZE(ebtft_gpio_leds),
};

static struct gpio_led apc_gpio_leds[] = {
	{
		.name			= "led:PM:usr1",
		.gpio			= PM_GPIO_LED1,	/* LED1 */
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "led:APC:usr1",
		.gpio			= APC_GPIO_LED2, /* LED2 */
		.default_trigger	= "mmc0",
	},
};
static struct gpio_led_platform_data apc_led_info = {
	.leds		= apc_gpio_leds,
	.num_leds	= ARRAY_SIZE(apc_gpio_leds),
};

static struct gpio_led sk_gpio_leds[] = {
	{
		.name			= "led:PM:usr1",
		.gpio			= PM_GPIO_LED1,	/* LED1 */
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "led:SK:usr1",
		.gpio			= SK_GPIO_LED2,
		.default_trigger	= "mmc0",
	},
	{
		.name			= "led:SK:usr2",
		.gpio			= SK_GPIO_LED3,
		.default_trigger	= "default-on",
	},
};

static struct gpio_led_platform_data sk_led_info = {
	.leds		= sk_gpio_leds,
	.num_leds	= ARRAY_SIZE(ebtft_gpio_leds),
};

static struct platform_device pia335x_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &ebtft_led_info,
	},
};

static void leds_init(int boardid)
{
	int err = 0;
	pr_info("piA335x: %s\n", __func__);

	switch (boardid) {
		case PIA335_KM_MMI:
			err = platform_device_register(&km_mmi_leds);
			break;
		case PIA335_PM:
		case PIA335_KM_E2:
			break;
		case PIA335_BB_EBTFT:
			err = platform_device_register(&pia335x_leds);
			break;
		case PIA335_BB_SK:
			pia335x_leds.dev.platform_data = &sk_led_info;
			err = platform_device_register(&pia335x_leds);
			break;
		case PIA335_BB_APC:
			pia335x_leds.dev.platform_data = &apc_led_info;
			err = platform_device_register(&pia335x_leds);
		default:
			break;
	}
	if (err)
		pr_err("failed to register gpio led device\n");
}

/* FRAM is similar to at24 eeproms without write delay and page limits */
static struct at24_platform_data km_e2_fram_info = {
	.byte_len       = (256*1024) / 8,
	.page_size      = (256*1024) / 8, /* no sequencial rw limit */
	.flags          = AT24_FLAG_ADDR16,
	.context        = (void *)NULL,
};
static struct tps65910_board pia335x_tps65910_info;
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
		/* rev 0.01 is using tmp422 */
		I2C_BOARD_INFO("tmp421", 0x4C),
	},
	{	I2C_BOARD_INFO("24c256", 0x52),
		.platform_data = &km_e2_fram_info,
	}
};

static void lcd_expansion_setup(struct memory_accessor *mem_acc, void *context);
/* AT24CS01 device with UID */
static struct at24_platform_data km_mmi_lcd_eeprom_info = {
	.byte_len       = 128,
	.page_size      = 8,
	.flags          = 0,
	.setup          = lcd_expansion_setup,
	.context        = (void *)NULL,
};
static struct at24_platform_data km_mmi_lcd_eepromuid_info = {
	.byte_len       = 256,
	.page_size      = 8,
	.flags          = AT24_FLAG_READONLY,
};

/* 24CS01 read-only unique ID area
 * these devices allow read on addresses 0x80..0xff only, so we need to
 * flag it as a 256 Byte Device */
static struct at24_platform_data pia335x_eepromuid_info = {
	.byte_len       = 256,
	.page_size      = 8,
	.flags          = AT24_FLAG_READONLY,
};

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
	.irq2 = OMAP_GPIO_IRQ(MMI_GPIO_ACC_INT2)
};

static struct i2c_board_info km_mmi_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &pia335x_tps65910_info,
	},
	{
		I2C_BOARD_INFO("lis331dlh", 0x18),
		.platform_data = &lis331dlh_pdata,
		.irq = OMAP_GPIO_IRQ(MMI_GPIO_ACC_INT1),
	},
	{
		I2C_BOARD_INFO("24c01", PIA335X_EEPROM_I2C_ADDR + 0x08),
		.platform_data  = &pia335x_eepromuid_info,
	},
};
static struct i2c_board_info km_mmi_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("24c01", 0x51),
		.platform_data = &km_mmi_lcd_eeprom_info,
	},
	{
		I2C_BOARD_INFO("24c01", 0x51 + 0x08),
		.platform_data = &km_mmi_lcd_eepromuid_info,
	},
};

static struct i2c_board_info pm_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &pia335x_tps65910_info,
	},
};

static struct i2c_board_info ebtft_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("pca9634", 0x2A),
		.platform_data = &ebtft_rgbleds_data,
	},
};

/* Lokisa EM DACs */
#include <linux/../../drivers/staging/iio/dac/max536.h>
static struct max536_platform_data em_dac_data = {
	/* Vref for MAX DAC is 0.9 * VDD (3.3V)
	 * amplifier is 2 x
	 * max V output DAC is 5 V */
	.vref_mv	= 3300 / 10 * 9 * 2,
};
/* Lokisa EM I2C */
#include <linux/i2c/pca953x.h>
static const char *em_xra1200_names[] = {
	"disp:green",
	"disp:red",
	"disp:up",
	"disp:down",
	"disp:enter",
	"disp:left",
	"disp:right",
	"disp:led",
};

static struct gpio_led em_disp_leds[] = {
	[0] = {
		.gpio = 0,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	},
	[1] = {
		.gpio = 1,
		.default_state = LEDS_GPIO_DEFSTATE_OFF,
	},
	[2] = {
		.gpio = 7,
		.default_state = LEDS_GPIO_DEFSTATE_ON,
	}
};

static struct gpio_led_platform_data em_disp_leds_pdata = {
	.leds = em_disp_leds,
	.num_leds = ARRAY_SIZE(em_disp_leds),
};

static struct platform_device em_disp_leds_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev = {
		.platform_data = &em_disp_leds_pdata
	}
};

static struct gpio em_disp_gpios[] = {
	{ 2, GPIOF_IN },
	{ 3, GPIOF_IN },
	{ 4, GPIOF_IN },
	{ 5, GPIOF_IN },
	{ 6, GPIOF_IN },
};

static void em_disp_leds_init(unsigned gpio)
{
	struct gpio_led* it;
	for (it = em_disp_leds; it < em_disp_leds + ARRAY_SIZE(em_disp_leds); ++it) {
		it->name = em_xra1200_names[it->gpio];
		it->gpio += gpio;
	}
}

static void em_disp_gpios_init(unsigned gpio)
{
	struct gpio* it;
	for (it = em_disp_gpios; it < em_disp_gpios + ARRAY_SIZE(em_disp_gpios); ++it) {
		it->label = em_xra1200_names[it->gpio];
		it->gpio += gpio;
	}
}

static int em_xra1200_setup(struct i2c_client *client,
		unsigned gpio, unsigned ngpio,
		void *c)
{
	int ret = 0;
	pr_info("piA335x: %s\n", __func__);

	/* prepare LED data */
	em_disp_leds_init(gpio);
	ret = platform_device_register(&em_disp_leds_device);
	if (ret) {
		pr_warning("Error during setup of IO-Expander");
	}

	/* prepare GPIO data */
	em_disp_gpios_init(gpio);
	pia335x_gpios_export(em_disp_gpios, ARRAY_SIZE(em_disp_gpios));

	return ret;
}
static int em_xra1200_teardown(struct i2c_client *client,
		unsigned gpio, unsigned ngpio, void *c)
{
	platform_device_unregister(&em_disp_leds_device);
	/* TODO implement keys */

	return 0;
}
static struct pca953x_platform_data em_xra1200_data = {
	/* FUTURE gpio and irq bases are not well defined on AM33xx */
	.gpio_base	= OMAP_MAX_GPIO_LINES + TPS65910_NUM_GPIO, /* some room for TPS */
	.irq_base	= TWL4030_IRQ_BASE + TPS65910_NUM_IRQ,
	.names		= em_xra1200_names,
	.setup		= em_xra1200_setup,
	.teardown	= em_xra1200_teardown,
};

static struct i2c_board_info em_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("pca9554", 0x27),
		.irq = OMAP_GPIO_IRQ(EM_GPIO_FOIL_IRQ),
		.platform_data = &em_xra1200_data,
	},
	{
		I2C_BOARD_INFO("max5362", 0x30),
		.platform_data = &em_dac_data,
	},
	{
		I2C_BOARD_INFO("max5362", 0x31),
		.platform_data = &em_dac_data,
	},
};

static void i2c1_init(int boardid)
{
	pr_info("piA335x: %s: %d\n", __func__, boardid);

	switch (boardid) {
	case PIA335_KM_E2:
		pia335x_register_i2c_devices(2, km_e2_i2c1_boardinfo,
				ARRAY_SIZE(km_e2_i2c1_boardinfo));
		break;
	case PIA335_KM_MMI:
		pia335x_register_i2c_devices(1, km_mmi_i2c1_boardinfo,
				ARRAY_SIZE(km_mmi_i2c1_boardinfo));
		pia335x_register_i2c_devices(2, km_mmi_i2c2_boardinfo,
				ARRAY_SIZE(km_mmi_i2c2_boardinfo));
		break;
	case PIA335_BB_EBTFT:
		pia335x_register_i2c_devices(2, ebtft_i2c2_boardinfo,
				ARRAY_SIZE(ebtft_i2c2_boardinfo));
		break;
	case PIA335_PM:
		pia335x_register_i2c_devices(1, pm_i2c1_boardinfo,
				ARRAY_SIZE(pm_i2c1_boardinfo));
		break;
	case PIA335_LOKISA_EM:
		pia335x_register_i2c_devices(1, pm_i2c1_boardinfo,
				ARRAY_SIZE(pm_i2c1_boardinfo));
		/* only PMIC and EEPROM on first bus */
		pia335x_register_i2c_devices(2, em_i2c2_boardinfo,
				ARRAY_SIZE(em_i2c2_boardinfo));
		break;
	default:
		break;
	}
}

/* LCD + TSC*/
/* display related info */
struct pia335x_lcd {
	int gpio_blen;
	int gpio_den;
	int gpio_tsc;
};
static struct pia335x_lcd exp_lcd = {
	.gpio_blen = -EINVAL,
	.gpio_den  = -EINVAL,
	.gpio_tsc  = -EINVAL
};

static struct pinmux_config lcdc_pin_mux[] = {
	{ "lcd_data0.lcd_data0", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data1.lcd_data1", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data2.lcd_data2", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data3.lcd_data3", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data4.lcd_data4", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data5.lcd_data5", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data6.lcd_data6", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data7.lcd_data7", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data8.lcd_data8", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data9.lcd_data9", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data10.lcd_data10", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data11.lcd_data11", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data12.lcd_data12", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data13.lcd_data13", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data14.lcd_data14", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "lcd_data15.lcd_data15", AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA },
	{ "gpmc_ad8.lcd_data16", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad9.lcd_data17", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad10.lcd_data18", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad11.lcd_data19", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad12.lcd_data20", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad13.lcd_data21", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad14.lcd_data22", AM33XX_PIN_OUTPUT },
	{ "gpmc_ad15.lcd_data23", AM33XX_PIN_OUTPUT },
	{ "lcd_vsync.lcd_vsync", AM33XX_PIN_OUTPUT },
	{ "lcd_hsync.lcd_hsync", AM33XX_PIN_OUTPUT },
	{ "lcd_pclk.lcd_pclk", AM33XX_PIN_OUTPUT },
	{ "lcd_ac_bias_en.lcd_ac_bias_en", AM33XX_PIN_OUTPUT },
	{NULL, 0},
};

/* Touch resitive integrated */
#include <linux/input/ti_tsc.h>
static struct tsc_data pia335x_res_touch_data  = {
	.wires  = 4,
	.x_plate_resistance = 200,
	.steps_to_configure = 5,
};

static struct mfd_tscadc_board pia335x_tscadc = {
	.tsc_init = &pia335x_res_touch_data,
	.adc_init = 0,
};

/* Touch interface FT5406 */
#if defined(CONFIG_TOUCHSCREEN_FT5X06) || \
	defined(CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE)
#include <linux/input/ft5x06_ts.h>

static struct ft5x06_ts_platform_data km_mmi_touch_data = {
	.x_max    = 480,
	.y_max    = 272,
	.irq_gpio = MMI_GPIO_LCD_PENDOWN,
	.irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.touch_threshold = 0x2B * 4,
};
static struct i2c_board_info km_mmi_i2c1_touch = {
	/* j043wqcn */
	I2C_BOARD_INFO("ft5x06_ts", 0x38),
	.irq = OMAP_GPIO_IRQ(MMI_GPIO_LCD_PENDOWN),
	.platform_data = &km_mmi_touch_data,
};
#endif

static void pia335x_touch_init(int boardid)
{
	int err = 0;

	pr_info("piA335x: %s\n", __func__);

	switch (boardid) {
	case PIA335_LCD_KM_MMI:
#if defined(CONFIG_TOUCHSCREEN_FT5X06) || \
		defined(CONFIG_TOUCHSCREEN_EDT_FT5X06_MODULE)
		pr_info("pia335x_init: init touch controller FT5x06\n");
		/* I2C adapter request */
		pia335x_add_i2c_device(2, &km_mmi_i2c1_touch);
#endif
		/* fall-through to resitive variant */
	case PIA335_LCD_EBTFT:
		if (exp_config.opt[2] == 'R')
			err = am33xx_register_mfd_tscadc(&pia335x_tscadc);
		if (exp_config.opt[2] == 'D') {
			pr_info("pia335x: init touch controller FT5x06\n");
			km_mmi_touch_data.irq_gpio = GPIO_TO_PIN(2, 1),
			km_mmi_i2c1_touch.irq =
					OMAP_GPIO_IRQ(GPIO_TO_PIN(2, 1)),
			pia335x_add_i2c_device(2, &km_mmi_i2c1_touch);
		}

		break;
	default:
		pr_warn("pia335x_init: TSC not detected/defined, id:%d\n",
				boardid);
		break;
	}

	if (err)
		pr_err("failed to register touch device\n");
}

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

struct da8xx_lcdc_platform_data  km_mmi_lcd_pdata = {
	/* display is a J043WQCN0101, works as NHD-4.3-ATXI */
	.manu_name              = "NHD",
	.controller_data        = &lcd_cfg,
	.type                   = "NHD-4.3-ATXI#-T-1",
};

/* REVISIT check if config identical to NewHaven */
struct da8xx_lcdc_platform_data  ebtft_lcd_pdata = {
	/* display is a ADKOM DLC0430LZG */
	.manu_name              = "DLC",
	.controller_data        = &lcd_cfg,
	.type                   = "DLC_DLC0430LZG",
};

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static void pia335x_lcd_power_ctrl(int val) {
	if (!gpio_is_valid(exp_lcd.gpio_den) ||
			!gpio_is_valid(exp_lcd.gpio_blen)) {
		pr_warn("LCD power control: invalid GPIO: BLEN or DEN\n");
		return;
	}

	if (val == 0) {
		pr_info("Turning off LCD\n");
		gpio_set_value(exp_lcd.gpio_blen, 0);
		usleep_range(10000, 11000); /* min 10 ms display hold time */
		gpio_set_value(exp_lcd.gpio_den, 0);
	} else {
		pr_info("Turning on LCD\n");
		gpio_set_value(exp_lcd.gpio_den, 1);
		usleep_range(10, 100); /* min 10 µs display setup time */
		gpio_set_value(exp_lcd.gpio_blen, 1);
	}
}

static void pia335x_lcd_init(int boardid)
{
	struct da8xx_lcdc_platform_data *lcdc_pdata;

	pr_info("piA335x: %s\n", __func__);
	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}
	switch (pia335x_lcd_exp_id.id) {
	case PIA335_LCD_KM_MMI:
		lcdc_pdata = &km_mmi_lcd_pdata;
		/* Backlight and Display enable GPIOs will be set in GPIO init */
		exp_lcd.gpio_blen = MMI_GPIO_LCD_BACKLIGHT;
		exp_lcd.gpio_den = MMI_GPIO_LCD_DISP;
		km_mmi_lcd_pdata.panel_power_ctrl = pia335x_lcd_power_ctrl;

		break;
	case PIA335_LCD_EBTFT:
		if (exp_config.opt[2] == 'D') {
			pr_info("piA335x: EBTFT with KM LCD\n");
			/* special demo version with KM display */
			lcdc_pdata = &km_mmi_lcd_pdata;
			km_mmi_lcd_pdata.panel_power_ctrl =
					pia335x_lcd_power_ctrl;
		} else {
			pr_info("piA335x: EBTFT with normal LCD\n");
			lcdc_pdata = &ebtft_lcd_pdata;
			ebtft_lcd_pdata.panel_power_ctrl =
					pia335x_lcd_power_ctrl;
		}
		exp_lcd.gpio_blen = EBTFT_GPIO_LCD_BACKLIGHT;
		exp_lcd.gpio_den = EBTFT_GPIO_LCD_DISP;

		break;
	default:
		pr_err("LCD not connected/supported, id:%d\n", boardid);
		return;
	}

	if (am33xx_register_lcdc(lcdc_pdata))
		pr_info("Failed to register LCDC device\n");

	/* initialize touch interface only for LCD display */
	pia335x_touch_init(boardid);

	return;
}

static void lcd_expansion_setup(struct memory_accessor *mem_acc,
		void *context)
{
	int res = 0;
	/* generic board detection triggered by eeprom init */
	pr_info("piA335x: lcd expansion setup\n");
	res = pia335x_read_eeprom(mem_acc, &pia335x_lcd_exp_id);

	if (res != 0) {
		pr_info("piA335x: no lcd expansion board detected\n");
		return;
	}

	/* EEPROM found, branch into specific board setups */
	pia335x_parse_eeprom(&pia335x_lcd_exp_id);
	pia335x_lcd_init(pia335x_lcd_exp_id.id);
}

/* CAN */
static struct pinmux_config can0_pin_mux[] = {
	{"uart1_ctsn.d_can0_tx", AM33XX_PULL_ENBL},
	{"uart1_rtsn.d_can0_rx", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
static struct pinmux_config can1_pin_mux[] = {
	{"uart0_ctsn.d_can1_tx", AM33XX_PULL_ENBL},
	{"uart0_rtsn.d_can1_rx", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
/* E2 CAN 1 */
static struct pinmux_config km_e2_can1_pin_mux[] = {
	{"uart1_rxd.d_can1_tx", AM33XX_PULL_ENBL},
	{"uart1_txd.d_can1_rx", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};
/* EB_TFT CAN0 */
static struct pinmux_config ebtft_can0_pin_mux[] = {
	{"mii1_txd3.d_can0_tx", AM33XX_PULL_ENBL},
	{"mii1_txd2.d_can0_rx", AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

extern void am33xx_d_can_init(unsigned int instance);
static void can_init(int boardid )
{
	pr_info("piA335x: %s\n", __func__);

	switch (boardid) {
	case PIA335_KM_E2:
		setup_pin_mux(km_e2_can1_pin_mux);
		am33xx_d_can_init(1);
		/* falltrough, CAN0 identical */
	case PIA335_KM_MMI:
		setup_pin_mux(can0_pin_mux);
		am33xx_d_can_init(0);
		break;
	case PIA335_BB_EBTFT:
		setup_pin_mux(ebtft_can0_pin_mux);
		am33xx_d_can_init(0);
		break;
	case PIA335_BB_SK:
		setup_pin_mux(can1_pin_mux);
		am33xx_d_can_init(1);
		break;
	case PIA335_LOKISA_EM:
	case PIA335_BB_APC:
		setup_pin_mux(can0_pin_mux);
		am33xx_d_can_init(0);
		setup_pin_mux(can1_pin_mux);
		am33xx_d_can_init(1);
		break;
	default:
		break;
	}

}

/* SPI */
static struct pinmux_config km_e2_spi_pin_mux[] = {
	/* SPI0 */
	{"spi0_sclk.spi0_sclk",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_d0.spi0_d0",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_d1.spi0_d1",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_cs0.spi0_cs0",	AM33XX_PIN_INPUT_PULLUP }, /* only rev 0.01 */
	{"spi0_cs1.spi0_cs1",	AM33XX_PIN_INPUT_PULLUP }, /* APS */
	/* SPI1 - only on exp. header since rev 0.02 */
	{"mcasp0_aclkx.spi1_sclk",	AM33XX_PIN_INPUT_PULLUP },
	{"mcasp0_fsx.spi1_d0", 		AM33XX_PIN_INPUT_PULLUP },
	{"mcasp0_axr0.spi1_d1",		AM33XX_PIN_INPUT_PULLUP },
	{"rmii1_refclk.spi1_cs0",	AM33XX_PIN_INPUT_PULLUP },
	{"ecap0_in_pwm0_out.spi1_cs1",	AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct omap2_mcspi_device_config spi_d1_mosi_cfg = {
	.turbo_mode	= 1,
	.d0_mosi	= 0,
};
static struct omap2_mcspi_device_config spi_d0_mosi_cfg = {
	.turbo_mode	= 1,
	.d0_mosi	= 1, /* we use MOSI on D0 for all SPI devices */
};

static struct spi_board_info km_e2_spi_aps_info[] = {
	{
		/* APS only MISO used */
		.modalias      = "spidev",
		.bus_num         = 1,
		.chip_select     = 1,
		.controller_data = &spi_d1_mosi_cfg,
		.max_speed_hz    = 1E6, /* 1MHz */
	},
};

#ifdef CONFIG_PIAAM335X_PROTOTYPE
static struct spi_board_info km_e2_spi_qenc_info[] = {
	{	/* LS7366, max 8 MHz */
		.modalias      = "spidev",
		.bus_num         = 1,
		.chip_select     = 0,
		.controller_data = &spi_d0_mosi_cfg,
		.max_speed_hz    = 5E6, /* 5MHz */
	},
};

#define KM_E2_CAN2_INT_GPIO GPIO_TO_PIN(3, 20)
#include <linux/can/platform/mcp251x.h>
static struct mcp251x_platform_data km_e2_mcp2515_data = {
	.oscillator_frequency = 25E6 ,
};
static struct spi_board_info km_e2_spi_mcp2515_info[] = {
	{
		/* 3rd CAN device */
		.modalias      = "mcp2515",
		.bus_num       = 2,
		.chip_select   = 0,
		.max_speed_hz  = 8E6, /* 5 MHz */
		.mode          = SPI_MODE_0,
		.irq           = OMAP_GPIO_IRQ(KM_E2_CAN2_INT_GPIO),
		.controller_data = &spi_d0_mosi_cfg,
		.platform_data = &km_e2_mcp2515_data,
	},
};
#endif /* CONFIG_PIAAM335X_PROTOTYPE */

static struct spi_board_info km_e2_spi1_0_info[] = {
	{
		/* external Header */
		.modalias      = "spidev",
		.bus_num         = 2,
		.chip_select     = 0,
		.max_speed_hz    = 1E6, /* 1MHz */
	},
};

static struct spi_board_info km_e2_spi1_1_info[] = {
	{
		/* external Header */
		.modalias      = "spidev",
		.bus_num         = 2,
		.chip_select     = 1,
		.max_speed_hz    = 1E6, /* 1MHz */
	},
};

/* PM SPI */
static struct pinmux_config pm_spi_pin_mux[] = {
	/* SPI0 */
	{"spi0_sclk.spi0_sclk",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_d0.spi0_d0",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_d1.spi0_d1",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_cs0.spi0_cs0",	AM33XX_PIN_INPUT_PULLUP }, /* NOR Flash */
	{NULL, 0},
};

/* SPI NOR flash */
static struct mtd_partition pm_spi_partitions[] = {
	/* All the partition sizes are listed in terms of erase size */
	{
		.name       = "SPL",
		.offset     = 0,			/* Offset = 0x0 */
		.size       = SZ_128K,
	},
	{
		.name       = "U-Boot",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size       = 2 * SZ_128K,
	},
	{
		.name       = "U-Boot Env",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size       = 2 * SZ_4K,
	},
	{
		.name       = "Kernel",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x62000 */
		.size       = 30 * SZ_128K,
	},
	{
		.name       = "File System",
		.offset     = MTDPART_OFS_APPEND,	/* Offset = 0x3E2000 */
		.size       = MTDPART_SIZ_FULL,		/* size ~= 4.1 MiB */
	}
};

static const struct flash_platform_data pm_spi_nor_flash = {
	.type      = "w25q64",
	.name      = "spi_flash",
	.parts     = pm_spi_partitions,
	.nr_parts  = ARRAY_SIZE(pm_spi_partitions),
};

/*
 * SPI Flash works at 80Mhz however SPI Controller works at 48MHz.
 * So setup Max speed to be less than that of Controller speed
 */
static struct spi_board_info pm_spi0_info[] = {
	{
		.modalias      = "m25p80",
		.platform_data = &pm_spi_nor_flash,
		.controller_data = &spi_d0_mosi_cfg,
		.irq           = -1,
		.max_speed_hz  = 24000000,
		.bus_num       = 1,
		.chip_select   = 0,
	},
};

static struct pinmux_config ebtft_spi_pin_mux[] = {
	/* SPI0 */
	{"mcasp0_aclkx.spi1_sclk",	AM33XX_PIN_INPUT_PULLUP },
	{"mcasp0_fsx.spi1_d0",		AM33XX_PIN_INPUT_PULLUP },
	{"mcasp0_axr0.spi1_d1",		AM33XX_PIN_INPUT_PULLUP },
	{"rmii1_refclk.spi1_cs0",	AM33XX_PIN_INPUT_PULLUP }, /* RFID */
	{NULL, 0},
};

static struct spi_board_info ebtft_spi_info[] = {
	{
		.modalias      = "spidev",
		.controller_data = &spi_d0_mosi_cfg,
		.max_speed_hz  = 2000000,
		.bus_num       = 2,
		.chip_select   = 0,
		.irq           = -1, /* spidev doesn't support interrupts */
	},
};

static struct pinmux_config apc_spi_pin_mux[] = {
	/* SPI0 - other signals defined on PM */
	{"spi0_cs1.spi0_cs1",	AM33XX_PIN_INPUT_PULLUP },
	{NULL, 0},
};
static struct spi_board_info apc_spi_info[] = {
	{
		.modalias      = "spidev",
		.controller_data = &spi_d0_mosi_cfg,
		.max_speed_hz  = 25000000,
		.bus_num       = 1,
		.chip_select   = 1,
		.irq           = -1, /* spidev doesn't support interrupts */
	},
};

static struct pinmux_config em_spi_pin_mux[] = {
	{"spi0_sclk.spi0_sclk",	AM33XX_PIN_INPUT_PULLUP },
	{"spi0_d0.spi0_d0",	AM33XX_PIN_INPUT_PULLUP },
	/*{"spi0_d1.spi0_d1",	AM33XX_PIN_INPUT_PULLUP },*/
	{"spi0_cs1.spi0_cs1",	AM33XX_PIN_INPUT_PULLUP }, /* Display */
	{ NULL, 0 },
};

static struct spi_board_info em_st7586s_info[] __initdata = {
	{
		.modalias = "st7586s",
		.mode = SPI_MODE_3,
		.bus_num = 1,
		.chip_select = 1,
		.max_speed_hz = 10000000,
		.controller_data = &spi_d0_mosi_cfg,
	},
};

static void spi_init(int boardid)
{
	pr_info("piA335x: %s: %d\n", __func__, boardid);

	switch (boardid) {
	case PIA335_PM:
		setup_pin_mux(pm_spi_pin_mux);
		spi_register_board_info(pm_spi0_info,
				ARRAY_SIZE(pm_spi0_info));
		break;
	case PIA335_BB_EBTFT:
		setup_pin_mux(ebtft_spi_pin_mux);
		spi_register_board_info(ebtft_spi_info,
				ARRAY_SIZE(ebtft_spi_info));
		break;
	case PIA335_BB_APC:
		setup_pin_mux(apc_spi_pin_mux);
		spi_register_board_info(apc_spi_info,
			ARRAY_SIZE(ebtft_spi_info));
		break;
	case PIA335_KM_E2:
		setup_pin_mux(km_e2_spi_pin_mux);
		spi_register_board_info(km_e2_spi_aps_info,
				ARRAY_SIZE(km_e2_spi_aps_info));

#ifdef CONFIG_PIAAM335X_PROTOTYPE
		if (am33xx_piarev == 1) {
			/* CAN device only on rev 0.01 */
			spi_register_board_info(km_e2_spi_mcp2515_info,
					ARRAY_SIZE(km_e2_spi_mcp2515_info));
			/* quad encoder only on rev 0.01 */
			spi_register_board_info(km_e2_spi_qenc_info,
					ARRAY_SIZE(km_e2_spi_qenc_info));
		} else {
#endif
			/* expansion header */
			spi_register_board_info(km_e2_spi1_0_info,
					ARRAY_SIZE(km_e2_spi1_0_info));
#ifdef CONFIG_PIAAM335X_PROTOTYPE
		}
#endif
		/* expansion header - all revisions */
		spi_register_board_info(km_e2_spi1_1_info,
				ARRAY_SIZE(km_e2_spi1_1_info));
		break;
	case PIA335_LOKISA_EM:
		setup_pin_mux(em_spi_pin_mux);
		spi_register_board_info(em_st7586s_info,
				ARRAY_SIZE(em_st7586s_info));
		break;
	case PIA335_KM_MMI:
	default:
		break;
	}
}

static void km_e2_rs485_init(void)
{
	pr_info("piA335x: %s\n", __func__);

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
	if (gpio_request(E2_GPIO_RS485_DE, "te_reg") < 0) {
		pr_err("Failed to request gpio for led_oe");
		return;
	}

	pr_info("Configure RS485 TE GPIO\n");
	/* enable receiver by default */
	gpio_direction_output(E2_GPIO_RS485_DE, 0);
	gpio_export(E2_GPIO_RS485_DE, 0);
}

static void km_e2_uart4_init(void)
{
	pr_info("piA335x: %s\n", __func__);

	setup_pin_mux(km_e2_uart4_pin_mux);
	/* */
	if (gpio_request(E2_GPIO_BOOT0_E1, "boot0_e1") < 0) {
		pr_err("Failed to request gpio for boot0_e1");
		return;
	}

	pr_info("Configure BOOT0_E1 GPIO\n");
	/* enable receiver by default */
	gpio_direction_output(E2_GPIO_BOOT0_E1, 0);
	gpio_export(E2_GPIO_BOOT0_E1, 0);
}

#ifdef CONFIG_PIAAM335X_PROTOTYPE
static void km_e2_ls7366_init(void)
{
	pia335x_clkout2_enable();
}
#endif

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

	pr_info("piA335x: %s\n", __func__);

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
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 *
	 * pia: we don't really need the 32k external OSC
	 */
	writel(0x48, base + 0x54);

	iounmap(base);

	// pia335x_rtc_info.pm_off = true;

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

static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 */
	.mode           = (MUSB_OTG << 4) | MUSB_HOST,
	.power		= 500,
	.instances	= 1,
};

/* Audio */
/* MMI McASP1 */
static struct pinmux_config mcasp0_pin_mux[] = {
	/* Audio.BCLK */
	{"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio.FSX */
	{"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio.DIN */
	{"mcasp0_aclkr.mcasp0_axr2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio.DOUT */
	{"mcasp0_ahclkx.mcasp0_axr3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
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

static struct i2c_board_info tlv320aic3x_i2c_boardinfo = {
	I2C_BOARD_INFO("tlv320aic3x", 0x1b),
};

static void km_mmi_tlv320aic3x_init(void)
{
	pr_info("piA335x: %s\n", __func__);

	/* Enable clkout1 */
	setup_pin_mux(clkout1_pin_mux);

	pia335x_add_i2c_device(1, &tlv320aic3x_i2c_boardinfo);

	mcasp0_init(PIA335_KM_MMI);
}

static struct pwmss_platform_data  pwm_pdata[3] = {
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
	{
		.version = PWM_VERSION_1,
	},
};
static void ecap_init(int boardid)
{
	int idx = 0;

	pr_info("piA335x: %s\n", __func__);

	switch (boardid) {
	case PIA335_BB_EBTFT:
		idx = 1;
		pwm_pdata[idx].chan_attrib[0].max_freq = 20000;
		am33xx_register_ecap(idx, &pwm_pdata[idx]);
		// TODO use something like "pwm-beeper" @ ecap.1
		break;
	default:
		break;
	}
}

#include <linux/platform_data/ti_adc.h>
static struct adc_data em_adc_data = {
	.adc_channels = 8,
};
static struct mfd_tscadc_board em_tscadc = {
	.tsc_init = NULL,
	.adc_init = &em_adc_data,
};
static void em_dac_init(int boardid)
{
	int err = 0;
	pr_info("piA335x: %s\n", __func__);

	switch (boardid) {
	case PIA335_LOKISA_EM:
		err = am33xx_register_mfd_tscadc(&em_tscadc);

		break;
	default:
		break;
	}

	if (err)
		pr_warn("piA3335x: failed to initialize TI-ADC\n");
}

static struct regulator_init_data pia335x_tps_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

static struct tps65910_board pia335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &pia335x_tps_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &pia335x_tps_dummy,
	.gpio_base = OMAP_MAX_GPIO_LINES,
	.irq = 0, // set this in board specific setup
	.irq_base = TWL4030_IRQ_BASE,
};

/* taken from board-am335xevm.c */
#define AM33XX_VDD_CORE_OPP50_UV	1100000
#define AM33XX_OPP120_FREQ		600000000
#define AM33XX_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	1000000000

static void am335x_opp_update(void)
{
	u32 rev;
	int voltage_uv = 0;
	struct device *core_dev, *mpu_dev;
	struct regulator *core_reg;

	core_dev = omap_device_get_by_hwmod_name("l3_main");
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev || !core_dev) {
		pr_err("%s: Aiee.. no mpu/core devices? %p %p\n", __func__,
		       mpu_dev, core_dev);
		return;
	}

	core_reg = regulator_get(core_dev, "vdd_core");
	if (IS_ERR(core_reg)) {
		pr_err("%s: unable to get core regulator\n", __func__);
		return;
	}

	/*
	 * Ensure physical regulator is present.
	 * (e.g. could be dummy regulator.)
	 */
	voltage_uv = regulator_get_voltage(core_reg);
	if (voltage_uv < 0) {
		pr_err("%s: physical regulator not present for core" \
		       "(%d)\n", __func__, voltage_uv);
		regulator_put(core_reg);
		return;
	}

	pr_info("%s: core regulator value %d\n", __func__, voltage_uv);
	if (voltage_uv > 0) {
		rev = omap_rev();
		switch (rev) {
		case AM335X_REV_ES1_0:
			if (voltage_uv <= AM33XX_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev, AM33XX_OPP120_FREQ);
				opp_disable(mpu_dev, AM33XX_OPPTURBO_FREQ);
			}
			break;
		case AM335X_REV_ES2_0:
			if (voltage_uv <= AM33XX_ES2_0_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPNITRO_FREQ);
			}
			break;
		case AM335X_REV_ES2_1:
		/* FALLTHROUGH */
		default:
			if (voltage_uv <= AM33XX_ES2_1_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPNITRO_FREQ);
			}
			break;
		}
	}
}

static char tps65910_core_vg_scale_sleep_seq[] = {
	0x64, 0x00,             /* i2c freq in khz */
	0x02, 0x2d, 0x25, 0x1f, /* Set VDD2 to 0.95V */
	0x0,
};

static char tps65910_core_vg_scale_wake_seq[] = {
	0x64, 0x00,             /* i2c freq in khz */
	0x02, 0x2d, 0x25, 0x2b, /* Set VDD2 to 1.1V */
	0x0,
};

static void pmic_init(int boardid)
{
	pr_info("piA335x: %s: %d\n", __func__, boardid);
	switch (boardid) {
		case PIA335_KM_E2:
			pia335x_tps65910_info.irq =
					OMAP_GPIO_IRQ(E2_GPIO_PMIC_INT);
			break;
		case PIA335_KM_MMI:
			pia335x_tps65910_info.irq =
					OMAP_GPIO_IRQ(MMI_GPIO_PMIC_INT);
			break;
		case PIA335_PM:
			pia335x_tps65910_info.irq =
					OMAP_GPIO_IRQ(PM_GPIO_PMIC_INT);
			break;
		case PIA335_LOKISA_EM:
			pia335x_tps65910_info.irq = OMAP_GPIO_IRQ(PM_GPIO_PMIC_INT);
			//pia335x_tps65910_info.irq_base = 0;
			pia335x_tps65910_info.irq_base = 0;

			break;
		default:
			break;
	}
}

static void km_e2_setup(void)
{
	if (pia335x_main_id.rev < 1 || pia335x_main_id.rev > 3) {
		pr_warn("PIA335E2: Unknown board revision %.4s, using "
				"rev 3 configuration\n",
				config.version);
		pia335x_main_id.rev = 3;
	}
	pr_info("piA335x: Setup KM E2 rev %d\n", pia335x_main_id.rev);

	setup_pin_mux(km_e2_board_pin_mux);

	pmic_init(pia335x_main_id.id);

	pia335x_rtc_init();
	i2c1_init(pia335x_main_id.id);

	ethernet_init(pia335x_main_id.id);
#ifdef CONFIG_PIAAM335X_PROTOTYPE
	if (pia335x_main_id.rev == 1) {
	} else {
#endif
		/* since 0.02 only USB0, we have to init musb_board_data
		 * before usb core driver is initialized */
		usb_init(pia335x_main_id.id);
#ifdef CONFIG_PIAAM335X_PROTOTYPE
	}
#endif
	nand_init();
	mmc_init(pia335x_main_id.id);

	can_init(pia335x_main_id.id);
	spi_init(pia335x_main_id.id);
	km_e2_rs485_init();
	if (pia335x_main_id.rev >= 3) {
		km_e2_uart4_init();
	}
	pia335x_gpios_init(pia335x_main_id.id);

#ifdef CONFIG_PIAAM335X_PROTOTYPE
	if (am33xx_piarev == 1)
		km_e2_ls7366_init();
#endif

	pr_info("piA335x: cpsw_init\n");
	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:0f", "0:00");
}

static void km_mmi_setup(int variant)
{
	pr_info("piA335x MMI: Setup KM MMI rev %d\n", pia335x_main_id.rev);
	if (pia335x_main_id.rev < 1 || pia335x_main_id.rev > 2) {
		pr_info("PIA335MI: Unknown board revision %.4s\n",
				config.version);
		pia335x_main_id.rev = 2;
	}

	setup_pin_mux(km_mmi_board_pin_mux);

	pmic_init(pia335x_main_id.id);

	i2c1_init(pia335x_main_id.id);

	ethernet_init(pia335x_main_id.id);

	//clkout2_32k_enable();

	km_mmi_tlv320aic3x_init();

	pr_info("piA335x: cpsw_init\n");
	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, "0:0f");

	mmc_init(pia335x_main_id.id);
	pia335x_gpios_init(pia335x_main_id.id);
	leds_init(pia335x_main_id.id);
	if (variant == 'X') {
		/* only on eXtended variant */
		usb_init(pia335x_main_id.id);
		can_init(pia335x_main_id.id);
		/* special 24V GPIOs */
		pia335x_gpios_export(km_mmi_24v_gpios, ARRAY_SIZE(km_mmi_24v_gpios));
	}
}

/* only procesor module related parts, see expansion_setup() for the rest */
static void pm_setup(void)
{
	pr_info("piA335x-PM: Setup PM rev %d\n", pia335x_main_id.rev);

	setup_pin_mux(pm_board_pin_mux);
	pmic_init(pia335x_main_id.id);
	i2c1_init(pia335x_main_id.id);

	/* prepare eMMC, will be initialized in baseboard setup */
	mmc_init(pia335x_main_id.id);

	pia335x_gpios_init(pia335x_main_id.id);

	// OMAP RTC should be setup with expansion if needed
	pm_setup_done = 1;
}

static void ebtft_setup(void)
{
	pr_info("piA335x-EB_TFT: ebtft_setup rev %d\n", pia335x_exp_id.rev);

	i2c1_init(pia335x_exp_id.id);
	pia335x_gpios_init(pia335x_exp_id.id);
	leds_init(pia335x_exp_id.id);

	mmc_init(pia335x_exp_id.id);
	ethernet_init(pia335x_exp_id.id);
	can_init(pia335x_exp_id.id);

	ecap_init(pia335x_exp_id.id);

	pia335x_lcd_exp_id.id = PIA335_LCD_EBTFT;
	pia335x_lcd_exp_id.rev = 1;
	pia335x_lcd_init(pia335x_lcd_exp_id.id);

	/* connected to slave 1, slave 0 is not active */
	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:0f", "0:00");
	usb_init(pia335x_exp_id.id);

	spi_init(pia335x_main_id.id);
	spi_init(pia335x_exp_id.id);
}

static void sk_setup(void)
{
	pr_info("piA-AM335x-StarterKit: sk_setup rev %d\n", pia335x_exp_id.rev);
	pia335x_gpios_init(pia335x_exp_id.id); /* TODO */
	leds_init(pia335x_exp_id.id);

	mmc_init(pia335x_exp_id.id);
	ethernet_init(pia335x_exp_id.id);
	can_init(pia335x_exp_id.id);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:0f", "0:00");
	usb_init(pia335x_exp_id.id);

	spi_init(pia335x_main_id.id);
	spi_init(pia335x_exp_id.id);
}

static void apc_setup(void)
{
	pr_info("piA-AM335x-APC: apc_setup rev %d\n", pia335x_exp_id.rev);
	setup_pin_mux(apc_board_pin_mux);
	pia335x_gpios_init(pia335x_exp_id.id);
	leds_init(pia335x_exp_id.id);

	mmc_init(pia335x_exp_id.id);
	ethernet_init(pia335x_exp_id.id);
	can_init(pia335x_exp_id.id);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:0f", "0:00");
	usb_init(pia335x_exp_id.id);

	spi_init(pia335x_main_id.id);
	spi_init(pia335x_exp_id.id);
}

static void em_setup(void)
{
	pr_info("Lokisa Energy Manager: em_setup rev %d\n",
		pia335x_main_id.rev);

	setup_pin_mux(em_board_pin_mux);
	pmic_init(pia335x_main_id.id);
	i2c1_init(pia335x_main_id.id);

	pia335x_gpios_init(pia335x_main_id.id);

	ethernet_init(pia335x_main_id.id);
	can_init(pia335x_main_id.id);

	mmc_init(pia335x_main_id.id);
	em_dac_init(pia335x_main_id.id);

	spi_init(pia335x_main_id.id);

	/* connected to slave 1, slave 0 is not active */
	am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, "0:1e", "0:05");

	/* setup sleep/wake sequence for core voltage scalling */
	am33xx_core_vg_scale_i2c_seq_fillup(tps65910_core_vg_scale_sleep_seq,
				ARRAY_SIZE(tps65910_core_vg_scale_sleep_seq),
				tps65910_core_vg_scale_wake_seq,
				ARRAY_SIZE(tps65910_core_vg_scale_wake_seq));
}

static void expansion_setup(struct memory_accessor *mem_acc, void *context)
{
	pr_info("piA335x: expansion setup\n");

	if (pia335x_read_eeprom(mem_acc, &pia335x_exp_id) != 0) {
		/* no readable EEPROM, check if we have a baseboard already
		 * setup, otherwise halt.*/
		if (pm_setup_done) {
			pr_info("PIA335x: no expansion board detected\n");
			return;
		}

		goto out;
	}
	pia335x_parse_eeprom(&pia335x_exp_id);

	/* Workaround for PM module without EEPROM
	 * pm_setup_done will be only set, if there was an EEPROM on I2C0
	 * and its setup function was called
	 * Explicitly call pm_setup() otherwise */
	if (pm_setup_done == 0) {
		pr_warn("piA335x: assume PM module without EEPROM\n");
		/* assume PM module without EEPROM here, use fake EEPROM */
		config.header = 0xEE3355AA;
		strncpy(config.name, "PIA335PM", 8);
		strncpy(config.version, "0.01", 4);
		pia335x_parse_eeprom(&pia335x_main_id);

		/* now call real pm_setup before doing expansion init */
		pm_setup();
	}

	/* TODO merge the single expansion setup functions here and use
	 * the optional setup for special case handling */
	switch (pia335x_exp_id.id) {
		case PIA335_BB_EBTFT:
			ebtft_setup();
			break;
		case PIA335_BB_SK:
			sk_setup();
			break;
		case PIA335_BB_APC:
			apc_setup();
			break;
		default:
			pr_err("PIA335x: Expansion board identification "
					"failed...\n");
			goto out;
	}
	return;

out:
	pr_err("PIA335x: Neither base nor expansion board detected, "
			"halting...\n");
	machine_halt();
}

static void pia335x_setup(struct memory_accessor *mem_acc, void *context)
{
	int res = 0;
	/* generic board detection triggered by eeprom init */
	pr_info("piA335x: main setup\n");
	res = pia335x_read_eeprom(mem_acc, &pia335x_main_id);
	if (res == -ETIMEDOUT || res == -1) {
		pr_warn("None or empty EEPROM, try with expansion setup");
		/* no EEPROM, we might have a PM board without EEPROM
		 * do nothing here, because eventually the expansion_setup
		 * will run pm_setup() if the board is a PM */
		return;
	}

	if (res != 0)
		goto out;

	/* EEPROM found, branch into specific board setups */
	pia335x_parse_eeprom(&pia335x_main_id);

	switch (pia335x_main_id.id) {
	case PIA335_KM_E2:
		km_e2_setup();
		break;
	case PIA335_KM_MMI:
		km_mmi_setup(config.opt[0]);
		break;
	case PIA335_PM:
		pm_setup();
		break;
	case PIA335_LOKISA_EM:
		em_setup();
		break;
	default:
		pr_err("PIA335x: Unknown board, "
				"check EEPROM configuration...\n");
		goto out;
	}
	/* we only care about this in case of a PM board with expansion,
	 * we need to make sure, not to run pm_setup() in a configuration
	 * with an EEPROM on I2C1 @0x50 */
	pm_setup_done = 1;

	am335x_opp_update();

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

/**
 * I2C devices
 */
static struct at24_platform_data pia335x_eeprom_info = {
	.byte_len       = 256,
	.page_size      = 8,
	.flags          = AT24_FLAG_READONLY,
	/* Setup gets called, even if there is no eeprom on the board.
	 * Make sure to handle this case! */
	.setup          = pia335x_setup,
	.context        = (void *)NULL,
};

static struct i2c_board_info __initdata pia335x_i2c0_boardinfo[] = {
	{
		/* Board ID EEPROM, KM E2 Rev 0.01 & 0.02 use 24c00,
		 * 0.03 uses 24cs01 (taking only 1 address 0x50) */
		I2C_BOARD_INFO("24c01", PIA335X_EEPROM_I2C_ADDR),
		.platform_data  = &pia335x_eeprom_info,
	},
};
/* 24AA02E48T, expansion/base board ID EEPROM */
static struct at24_platform_data expansion_eeprom_info = {
	.byte_len       = 256,
	.page_size      = 8,
	.flags          = 0,
	.setup          = expansion_setup,
	.context        = (void *)NULL,
};

static struct i2c_board_info __initdata pia335x_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("at24", PIA335X_EEPROM_I2C_ADDR),
		.platform_data  = &expansion_eeprom_info,
	},
};

static void __init pia335x_i2c_init(void)
{
	/* I2C1 must be muxed in u-boot */
	pr_info("piA335x: %s", __func__);
	omap_register_i2c_bus(1, 400, pia335x_i2c0_boardinfo,
				ARRAY_SIZE(pia335x_i2c0_boardinfo));

	/* initialize the second bus as well, in case we have an expansion
	 * board/base board with another id eeprom
	 * We expect the bootloader to initialize the correct pinmux for
	 * the second bus!
	 */
	omap_register_i2c_bus(2, 400, pia335x_i2c1_boardinfo,
				ARRAY_SIZE(pia335x_i2c1_boardinfo));
}

#ifdef CONFIG_MACH_AM335XEVM
/* for some reason board specific stuff is called from mach code
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
	pia335x_i2c_init();
	pr_info("piA335x: sdrc_init\n");
	omap_sdrc_init(NULL, NULL);
	pr_info("piA335x: musb_init\n");
	usb_musb_init(&musb_board_data);

	omap_board_config = pia335x_config;
	omap_board_config_size = ARRAY_SIZE(pia335x_config);
	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
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
