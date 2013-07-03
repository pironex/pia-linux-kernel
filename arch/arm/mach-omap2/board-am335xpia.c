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
#include <linux/if_ether.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/tps65910.h>
#include <linux/reboot.h>

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
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW |
			AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif


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
	{"gpmc_be1n.gpio1_28", 		OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
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
	int ret;
	int use_lcd = 1;

	setup_pin_mux(lcdc_pin_mux);

	pia335x_dss_data.default_device = &pia335x_lcd_device;

	/* backlight GPIO */
	if ((ret = gpio_request_one(GPIO_LCD_BACKLIGHT,
			GPIOF_DIR_OUT | GPIOF_INIT_LOW, "lcd-backlight")) != 0) {
		pr_err("%s: GPIO_LCD_BACKLIGHT request failed: %d\n", __func__, ret);
		return;
	} else {
		//gpio_direction_output(GPIO_LCD_BACKLIGHT, 0);
		omap_mux_init_gpio(GPIO_LCD_BACKLIGHT, OMAP_PIN_INPUT_PULLDOWN);
		gpio_export(GPIO_LCD_BACKLIGHT, true);
	}

	/* DISPLAY_EN GPIO */
	if ((ret = gpio_request_one(GPIO_LCD_DISP,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "lcd-disp")) != 0) {
		pr_err("%s: GPIO_LCD_DISP request failed: %d\n", __func__, ret);
		gpio_free(GPIO_LCD_BACKLIGHT);
		return;
	} else {
		//gpio_direction_output(GPIO_LCD_DISP, 1);
		omap_mux_init_gpio(GPIO_LCD_DISP, OMAP_PIN_INPUT_PULLDOWN);
		gpio_export(GPIO_LCD_DISP, true);
	}

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
		.gpio_wp        = GPIO_TO_PIN(3, 9),
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

static void setup_e2(void)
{
	pr_info("piA335x: Setup KM E2.\n");
	/* EVM - Starter Kit */
/*	static struct evm_dev_cfg evm_sk_dev_cfg[] = {
		{mmc1_wl12xx_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{rgmii1_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{rgmii2_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{lcdc_init,     DEV_ON_BASEBOARD, PROFILE_ALL},
		{enable_ecap2,     DEV_ON_BASEBOARD, PROFILE_ALL},
		{mfd_tscadc_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{gpio_keys_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
		{gpio_led_init,  DEV_ON_BASEBOARD, PROFILE_ALL},
		{lis331dlh_init, DEV_ON_BASEBOARD, PROFILE_ALL},
		{mcasp1_init,   DEV_ON_BASEBOARD, PROFILE_ALL},
		{uart1_wl12xx_init, DEV_ON_BASEBOARD, PROFILE_ALL},
		{wl12xx_init,       DEV_ON_BASEBOARD, PROFILE_ALL},
		{gpio_ddr_vtt_enb_init,	DEV_ON_BASEBOARD, PROFILE_ALL},
		{NULL, 0, 0},
	};*/
	pia335x_rtc_init();

	mmc0_init(PIA335_KM_E2);

	pr_info("piA335x: cpsw_init\n");
	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, NULL, NULL);
}

#if 0	//FIXME: Do we need CLKOUT2?
/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void clkout2_enable(int evm_id, int profile)
{
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

	//TODO: add DaVinci Ethernet init
	//pr_info("piA335x: cpsw_init\n");
	//am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);

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
			pr_info("piA335x: Unknown board revision %.4s\n",
					config.version);
		}
	} else if(!strncmp("PIA335MI", config.name, 8)) {
		if(!strncmp("0.01", config.version, 4)) {
			setup_mmi();
		} else {
			pr_info("piA335x: Unknown board revision %.4s\n",
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
	pia335x_i2c_init();
	omap_sdrc_init(NULL, NULL);
	gpio_led_init();
	lcdc_init();


	//mmc0_init();

	//am33xx_evmid_fillup(PIA335_KM_E2);
	//am33xx_cpsw_init(0);
	//am33xx_cpsw_init(AM33XX_CPSW_MODE_MII, NULL, NULL);
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
