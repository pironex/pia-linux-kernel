/*
 * Support for AM3505 piA
 * by pironex GmbH -- http://www.pironex.de
 *
 * Copyright (C) 2011 pironex GmbH <info@pironex.de>
 * Author: Bjoern Krombholz <b.krombholz@pironex.de>
 *
 * Ideas taken from mach-omap2/board-am3517crane.c
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
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/davinci_emac.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/i2c/tsc2007.h>
#include <linux/mfd/tps6507x.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6507x.h>
#include <linux/regulator/fixed.h>
#include <linux/wl12xx.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/leds.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/clock.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <video/omap-panel-dvi.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"



enum {
	PIA_AM3505,
	PIA_X_AM3517,
	PIA_UNKNOWN = 0xff,
};
static u8 pia35x_version = PIA_UNKNOWN;
static char expansionboard_name[32];
static char lcdboard_name[16];

/*
 * GSM: Telit GE864 Quad-V2
 */
#define GPIO_EN_GSM_POWER   29    /* GSM power supply voltage */
#define GPIO_GSM_NRESET    126    /* GSM reset low active */
#define GPIO_GSM_ONOFF     127    /* GSM on/off low active*/
static int __init pia35x_gsm_init(void)
{
	int ret;

	/* piAx doesn't have a GSM socket */
	if (pia35x_version == PIA_X_AM3517) return 0;

	pr_info("pia35x_init: init GSM\n");

	/* GSM GPIOs are low active */
	if ((ret = gpio_request_one(GPIO_GSM_NRESET,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "gsm-reset")) != 0) {
		pr_warning("%s: GPIO 126 request failed: %d\n", __func__, ret);
	} else {
		// GPIO 126 is available on 2 pins
		omap_mux_init_signal("sdmmc1_dat4.gpio_126", OMAP_PIN_OUTPUT);
		gpio_export(GPIO_GSM_NRESET, false);
	}

	if ((ret = gpio_request_one(GPIO_GSM_ONOFF,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH,"gsm-onoff")) != 0) {
		pr_warning("%s: GPIO 127 request failed, %d\n", __func__, ret);
	} else {
		omap_mux_init_gpio(GPIO_GSM_ONOFF,    OMAP_PIN_OUTPUT);
		gpio_export(GPIO_GSM_ONOFF, false);
	}

	if ((ret = gpio_request_one(GPIO_EN_GSM_POWER,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH,"gsm-power")) != 0) {
		pr_warning("%s: GPIO_EN_GSM_POWER request failed: %d\n", __func__, ret);
		return -1;
	} else {
		omap_mux_init_gpio(GPIO_EN_GSM_POWER, OMAP_PIN_OUTPUT);
		gpio_export(GPIO_EN_GSM_POWER, false);
	}

	return 0;
}

/** piA-LCD **/
#define GPIO_LCD_DISP		99
#define GPIO_LCD_BACKLIGHT 101
#define GPIO_LCDDVI_SWITCH 140 /* 1 = DVI, 0 = LCD */

#if defined(CONFIG_PANEL_GENERIC_DPI) || \
		defined(CONFIG_PANEL_GENERIC_DPI_MODULE)
static int pia35x_lcd_enable(struct omap_dss_device *dssdev)
{
	gpio_set_value(GPIO_LCD_DISP, 1);
	//msleep(1000);
	pr_info("pia35x: enabling LCD\n");
	gpio_set_value(GPIO_LCD_BACKLIGHT, 1);
	gpio_set_value(GPIO_LCDDVI_SWITCH, 0);

	return 0;
}

static void pia35x_lcd_disable(struct omap_dss_device *dssdev)
{
	gpio_set_value(GPIO_LCD_BACKLIGHT, 0);
	gpio_set_value(GPIO_LCDDVI_SWITCH, 1);
	pr_info("pia35x: disabling LCD\n");
	gpio_set_value(GPIO_LCD_DISP, 0);
}

static struct panel_generic_dpi_data pia35x_lcd_panel_dem = {
	.name               = "dem_480272d",
	.platform_enable    = pia35x_lcd_enable,
	.platform_disable   = pia35x_lcd_disable,
};

static struct panel_generic_dpi_data pia35x_lcd_panel_sharp = {
	.name               = "sharp_lq",
	.platform_enable    = pia35x_lcd_enable,
	.platform_disable   = pia35x_lcd_disable,
};

static struct omap_dss_device pia35x_lcd_device = {
	.type               = OMAP_DISPLAY_TYPE_DPI,
	.name               = "lcd",
	.driver_name        = "generic_dpi_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio         = -EINVAL,
	.data               = &pia35x_lcd_panel_sharp,
};
#define PIA_LCD
#else
static struct omap_dss_device pia35x_lcd_device = {
		.type = OMAP_DISPLAY_TYPE_NONE,
		.name = "lcd",
		.driver_name = "none",
		.reset_gpio = -EINVAL,
};
#endif /* CONFIG_PANEL_SHARP_LQ043T1DG01 */

#if defined(CONFIG_PANEL_DVI) || defined(CONFIG_PANEL_DVI_MODULE)
static int pia35x_dvi_enable(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 1);
	gpio_set_value(GPIO_LCDDVI_SWITCH, 1);
	pr_info("pia35x: enabling DVI");
	return 0;
}
static void pia35x_dvi_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 0);
	gpio_set_value(GPIO_LCDDVI_SWITCH, 0);
	pr_info("pia35x: disabling DVI");
	return;
}
#define PIA_DVI

static struct panel_dvi_platform_data dvi_panel = {
	.platform_enable    = pia35x_dvi_enable,
	.platform_disable   = pia35x_dvi_disable,
	.i2c_bus_num		= 3,
};

static struct omap_dss_device pia35x_dvi_device = {
	.type               = OMAP_DISPLAY_TYPE_DPI,
	.name               = "dvi",
	.driver_name        = "dvi",
	.phy.dpi.data_lines = 24,
	.reset_gpio         = GPIO_LCDDVI_SWITCH,
	.data               = &dvi_panel,
};
#else
static struct omap_dss_device pia35x_dvi_device = {
		.type = OMAP_DISPLAY_TYPE_NONE,
		.name = "dvi",
		.driver_name = "none",
		.reset_gpio = -EINVAL,
};
#endif /* CONFIG_PANEL_GENERIC */

static struct omap_dss_device *pia35x_dss_devices[] = {
	&pia35x_lcd_device,
	&pia35x_dvi_device,
};

static struct omap_dss_board_info pia35x_dss_data = {
	.num_devices     = ARRAY_SIZE(pia35x_dss_devices),
	.devices         = pia35x_dss_devices,
	.default_device  = &pia35x_lcd_device,
};

#if defined(PIA_LCD) || defined(PIA_DVI)

/* Touch interface */
#if defined(CONFIG_INPUT_TOUCHSCREEN) && \
    defined(CONFIG_TOUCHSCREEN_TSC2007)
/* Pen Down IRQ, low active */
#define GPIO_LCD_PENDOWN 100
static int pia35x_tsc2007_pendown(void)
{
	return !gpio_get_value(GPIO_LCD_PENDOWN);
}

static int pia35x_tsc2007_init_hw(void)
{
	int gpio = GPIO_LCD_PENDOWN;
	int ret = 0;
	pr_info("pia35x_init: init TSC2007\n");
	ret = gpio_request_one(gpio, GPIOF_DIR_IN, "tsc2007_pen_down");
	if (ret < 0) {
		pr_err("Failed to request GPIO_LCD_PENDOWN: %d\n", ret);
		return ret;
	}
	gpio_set_debounce(gpio, 0xa);
	omap_mux_init_gpio(GPIO_LCD_PENDOWN, OMAP_PIN_INPUT_PULLUP);
	irq_set_irq_type(OMAP_GPIO_IRQ(GPIO_LCD_PENDOWN), IRQ_TYPE_EDGE_FALLING);

	return ret;
}

static struct tsc2007_platform_data tsc2007_info = {
	.model = 2007,
	.x_plate_ohms = 180,
	.get_pendown_state = pia35x_tsc2007_pendown,
	.init_platform_hw = pia35x_tsc2007_init_hw,
};

static struct i2c_board_info __initdata pia35x_i2c3_tsc2007[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x4B),
		.irq = OMAP_GPIO_IRQ(GPIO_LCD_PENDOWN),
		.platform_data = &tsc2007_info,
	},
};
#else
static struct i2c_board_info __initdata pia35x_i2c3_tsc2007[] = {};
#endif

static void __init pia35x_touch_init(void)
{
	pr_info("pia35x_init: init touchscreen TSC2007\n");
	i2c_register_board_info(3, pia35x_i2c3_tsc2007,
			ARRAY_SIZE(pia35x_i2c3_tsc2007));
}

static void __init pia35x_display_init(void)
{
	int ret;
	int use_lcd = 1;
	char *sub = 0;
	int rev = 0;

	sub = strrchr(lcdboard_name, '-');
	if (sub != NULL)
		rev = 0 - simple_strtol(sub, 0, 10);
	/* all lcd board names should start with "pia_lcd"
	 * default panel is Sharp LQ */
	if (0 != strncmp(lcdboard_name, "pia_lcd", 7)) {
		use_lcd = 0;
	} else if (0 == strncmp(lcdboard_name, "pia_lcd_dem", 11)) {
		pia35x_lcd_device.data = &pia35x_lcd_panel_dem;
	} else if (0 == strncmp(lcdboard_name, "pia_lcd_dt028", 13)) {
	}

	/* the tsc device address changed for revision 1 display expansions */
	if (use_lcd && rev == 1) {
		pia35x_i2c3_tsc2007[0].addr = 0x48;
	}

	if (0 == use_lcd)
		pia35x_dss_data.default_device = &pia35x_dvi_device;

	/* don't initialize DSS on piA-AM3505 when no piA-LCD attached */
	if ((pia35x_version == PIA_AM3505) && (0 == use_lcd))
		return;

	pr_info("pia35x_init: init DSS\n");

	/* LCD_DVI switch */
	if (pia35x_version == PIA_X_AM3517 &&
			(ret = gpio_request_one(GPIO_LCDDVI_SWITCH,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "lcddvi.switch")) != 0) {
		pr_err("%s: GPIO_LCDDVI_SWITCH request failed: %d\n", __func__, ret);
		return;
	} else {
		//gpio_direction_output(GPIO_LCDDVI_SWITCH, 1);
		omap_mux_init_gpio(GPIO_LCDDVI_SWITCH, OMAP_PIN_OUTPUT);
		gpio_export(GPIO_LCDDVI_SWITCH, true);
	}

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

	pr_info("pia35x_init: init LCD\n");

	/* initialize touch interface only for LCD display */
	if (use_lcd)
		pia35x_touch_init();

	return;
}
#else
inline static void __init pia35x_display_init(void) { }
#endif /* PIA_DVI || PIA_LCD */

/** SPI GPIO CS Hack **/
//int mcspi1_cs_gpios[4];
int pia_mcspi2_cs_gpios[4];
extern int *mcspi2_cs_gpios;

/** piA-MotorControl **/
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
#include <plat/mcspi.h>
#include <linux/spi/spi.h>

//#define SYS_CLKOUT2_PARENT	"omap_54m_fck"
#define SYS_CLKOUT2_PARENT	"cm_96m_fck"
static struct omap2_mcspi_device_config pia35x_motor_x_cfg = {
	.turbo_mode       = 0,
	.single_channel   = 1,
};
static struct omap2_mcspi_device_config pia35x_motor_y_cfg = {
	.turbo_mode       = 0,
	.single_channel   = 1,
};
static struct omap2_mcspi_device_config pia35x_motor_z_cfg = {
	.turbo_mode       = 0,
	.single_channel   = 1,
};
static struct spi_board_info pia35x_spi_mc_info[] __initdata = {
	/* Motor X */
	{
		.modalias        = "spidev",
		.bus_num         = 1,
		.chip_select     = 1,
		.max_speed_hz    = 1000000, /* 1MHz */
		.controller_data = &pia35x_motor_x_cfg,
	},
	/* Motor Y */
	{
		.modalias        = "spidev",
		.bus_num         = 1,
		.chip_select     = 2,
		.max_speed_hz    = 1000000, /* 1MHz */
		.controller_data = &pia35x_motor_y_cfg,
	},
	/* Motor Z */
	{
		.modalias        = "spidev",
		.bus_num         = 1,
		.chip_select     = 3,
		.max_speed_hz    = 1000000, /* 1MHz */
		.controller_data = &pia35x_motor_z_cfg,
	},
};

static int __init pia35x_sys_clkout2_init(void)
{
	struct clk *sys_clkout2;
	struct clk *parent_clk;
	struct clk *sys_clkout2_src;
	pr_info("pia35x: Initializing SYS_CLKOUT2");
	sys_clkout2_src = clk_get(NULL, "clkout2_src_ck");
	if (IS_ERR(sys_clkout2_src)) {
		pr_err("pia35x: Could not get clkout2_src_ck");
		return -1;
	}

	sys_clkout2 = clk_get(NULL, "sys_clkout2");
	if (IS_ERR(sys_clkout2)) {
		pr_err("pia35x: Could not get sys_clkout2");
		clk_put(sys_clkout2_src);
		return -2;
	}

	parent_clk = clk_get(NULL, SYS_CLKOUT2_PARENT);
	if (IS_ERR(parent_clk)) {
		pr_err("pia35x: Could not get " SYS_CLKOUT2_PARENT);
		clk_put(sys_clkout2);
		clk_put(sys_clkout2_src);
		return -3;
	}

	clk_set_parent(sys_clkout2_src, parent_clk);
	//clk_set_rate(sys_clkout2, 13500000);
	clk_set_rate(sys_clkout2, 12000000);

	pr_info("pia35x: parent of SYS_CLKOUT2 %s ", parent_clk->name);
	pr_info("pia35x: CLK - enabling SYS_CLKOUT2 with %lu MHz",
			clk_get_rate(sys_clkout2));
	clk_enable(sys_clkout2);

	return 0;
}

#define GPIO_MOTOR_X_EN    12
#define GPIO_MOTOR_X_DIR   14
#define GPIO_MOTOR_X_STEP  13
#define GPIO_MOTOR_X_SG    18
#define GPIO_MOTOR_Y_EN    16
#define GPIO_MOTOR_Y_DIR   21
#define GPIO_MOTOR_Y_STEP  17
#define GPIO_MOTOR_Y_SG    15
#define GPIO_MOTOR_Z_EN   162
#define GPIO_MOTOR_Z_DIR  160
#define GPIO_MOTOR_Z_STEP 161
#define GPIO_MOTOR_Z_SG   157
static struct gpio pia35x_motorcontrol_gpios[] = {
	/* X */
	{ GPIO_MOTOR_X_EN,  GPIOF_DIR_OUT | GPIOF_INIT_HIGH,"motorX.nen"   },
	{ GPIO_MOTOR_X_DIR, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motorX.dir"  },
	{ GPIO_MOTOR_X_STEP,GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motorX.step" },
	{ GPIO_MOTOR_X_SG,  GPIOF_DIR_IN,                   "motorX.sg"   },
	/* Y */
	{ GPIO_MOTOR_Y_EN,  GPIOF_DIR_OUT | GPIOF_INIT_HIGH,"motorY.nen"   },
	{ GPIO_MOTOR_Y_DIR, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motorY.dir"  },
	{ GPIO_MOTOR_Y_STEP,GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motorY.step" },
	{ GPIO_MOTOR_Y_SG,  GPIOF_DIR_IN,                   "motorY.sg"   },
	/* Z */
	{ GPIO_MOTOR_Z_EN,  GPIOF_DIR_OUT | GPIOF_INIT_HIGH,"motorZ.nen"   },
	{ GPIO_MOTOR_Z_DIR, GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motorZ.dir"  },
	{ GPIO_MOTOR_Z_STEP,GPIOF_DIR_OUT | GPIOF_INIT_LOW, "motorZ.step" },
	{ GPIO_MOTOR_Z_SG,  GPIOF_DIR_IN,                   "motorZ.sg"   },
};

static int __init pia35x_motorcontrol_init(void)
{
	int err = 0, i;
	unsigned int gpio;
	unsigned long flags;

	pr_info("pia35x: Initializing piA-MotorControl board");
	if (0 != pia35x_sys_clkout2_init()) {
		pr_warn("pia35x: Could not initialize MotorControl Clock!");
		return -1;
	}
	/* GPIOs for EN/DIR/STEP/StallDetect */
	if (0 != (err = gpio_request_array(pia35x_motorcontrol_gpios,
			ARRAY_SIZE(pia35x_motorcontrol_gpios)))) {
		pr_warning("pia35x: unable to request MotorControl GPIOs: %d", err);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(pia35x_motorcontrol_gpios); i++) {
		gpio  = pia35x_motorcontrol_gpios[i].gpio;
		flags = pia35x_motorcontrol_gpios[i].flags;

		/* GPIOF_DIR_IN is 1 */
		omap_mux_init_gpio(gpio, OMAP_MUX_MODE4	| (flags & GPIOF_DIR_IN) ?
						OMAP_PIN_INPUT_PULLDOWN : OMAP_PIN_OUTPUT);
		gpio_export(gpio, false);
	}

	spi_register_board_info(pia35x_spi_mc_info,
			ARRAY_SIZE(pia35x_spi_mc_info));

	return 0;
}
#else
static inline void int __init pia35x_motorcontrol_init(void) { return; }
#endif /* CONFIG_SPI_SPIDEV */


/** piA-Wireless **/
/* WIFI/BT: TiWi-R2 (WL1271) */
#if defined(CONFIG_WL1271_SDIO) || defined(CONFIG_WL1271_SDIO_MODULE)
#define GPIO_WLAN_IRQ	137
#define GPIO_WLAN_PMENA	139
#define GPIO_BT_EN      138

static struct regulator_consumer_supply pia35x_vmmc2_consumers[] = {
	//REGULATOR_SUPPLY("vmmc2", "mmci-omap-hs.1");
		REGULATOR_SUPPLY("vwl1271", "wl1271"),
};

static struct regulator_init_data pia35x_vmmc2_data = {
	.constraints = {
		.valid_ops_mask   = REGULATOR_CHANGE_STATUS,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.min_uV           = 1800000,
		.max_uV           = 1800000,
		.apply_uV         = true,
		.always_on        = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(pia35x_vmmc2_consumers),
	.consumer_supplies     = pia35x_vmmc2_consumers,
};

static struct fixed_voltage_config pia35x_vmmc2_config = {
	.supply_name     = "vwl1271",
	.microvolts      = 1800000,
	.gpio            = GPIO_WLAN_PMENA,
	.startup_delay   = 70000,
	.enable_high     = 1,   /* gpio = 1 means wlan_en active */
	.enabled_at_boot = 0,   /* was the module enabled before linux boot */
	.init_data       = &pia35x_vmmc2_data,
};

static struct platform_device pia35x_vwlan_device = {
	.name           = "reg-fixed-voltage",
	.id             = -1,
	.dev = {
		.platform_data = &pia35x_vmmc2_config,
	},
};

#define WL12XX_REFCLOCK_26      1 /* 26 MHz */
#define WL12XX_REFCLOCK_38      2 /* 38.4 MHz */
static struct wl12xx_platform_data pia35x_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
	/* internal ref clock is 38 MHz */
	.board_ref_clock = WL12XX_REFCLOCK_38, /* 2, internal refclock of the  */
};

static int __init pia35x_wlan_init(void)
{
	u32 reg;
	int ret = 0;

	omap_mux_init_gpio(GPIO_WLAN_IRQ, OMAP_PIN_INPUT);
	omap_mux_init_gpio(GPIO_WLAN_PMENA, OMAP_PIN_OUTPUT);

	//if ((ret = gpio_request(GPIO_WLAN_PMENA, "wlan-power")))
	//	pr_warning("%s: GPIO_WLAN_PMENA request failed: %d\n", __func__, ret);
	//gpio_direction_output(GPIO_WLAN_PMENA, 0);
	//gpio_export(GPIO_WLAN_PMENA, false);

	if ((ret = wl12xx_set_platform_data(&pia35x_wlan_data)) != 0)
		pr_err("%s: error setting wl12xx data: %d\n", __func__, ret);

	// we need to enable the internal clock loopback on MMC2!
	reg = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
	reg |= OMAP2_MMCSDIO2ADPCLKISEL;
	omap_ctrl_writel(reg, OMAP343X_CONTROL_DEVCONF1);

	//pia35x_vmmc2_consumers[0].dev = mmc[1].dev;

	platform_device_register(&pia35x_vwlan_device);

	return ret;
}

/* BlueTooth */
static void __init pia35x_bt_init(void)
{
	int ret = 0;
	//gpio_request(136, "bt.wu");
	//gpio_direction_output(136, 0);
	/* just enable the BT module */
	if ((ret = gpio_request_one(GPIO_BT_EN,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "bt-en")) != 0) {
		pr_warning("GPIO 138 (BT.EN) request failed\n");
	} else {
		omap_mux_init_gpio(GPIO_BT_EN, OMAP_PIN_INPUT);
		gpio_export(GPIO_BT_EN, false);
	}
}
#else
static inline void __init pia35x_wlan_init(void) { return; }
static inline void __init pia35x_bt_init(void) { return; }
#endif /* CONFIG_WL1271_SDIO */

/* RFID */
#include <plat/mcspi.h>
#include <linux/spi/spi.h>

#define GPIO_RFID_IRQ    14
#define GPIO_RFID_POWER  18
#define GPIO_RFID_SPI_CS 174

static struct omap2_mcspi_device_config pia35x_rfid_cfg = {
	.turbo_mode       = 0,
	.single_channel   = 1,
};

static struct spi_board_info  __initdata pia35x_spi_rfid_info[] = {
	{
		.modalias        = "spidev",
		.bus_num         = 1,
		.chip_select     = 0,
		.max_speed_hz    = 1000000, /* 1MHz */
		.controller_data = &pia35x_rfid_cfg,
	},
};
static struct gpio pia35x_rfid_gpios[] = {
	{ GPIO_RFID_IRQ,    GPIOF_DIR_IN, "rfid.irq"   },
	{ GPIO_RFID_POWER,  GPIOF_DIR_OUT | GPIOF_INIT_LOW, "rfid.power" },
	{ GPIO_RFID_SPI_CS, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "motorX.nen" },
};

static void __init pia35x_rfid_init(void)
{
	int i;
	unsigned gpio;
	if (0 != gpio_request_array(pia35x_rfid_gpios,
			ARRAY_SIZE(pia35x_rfid_gpios))) {
		pr_warning("pia35x_init: RFID GPIO request failed\n");
		return;
	}

	for (i = 0; i < ARRAY_SIZE(pia35x_rfid_gpios); ++i) {
		gpio = pia35x_rfid_gpios[i].gpio;
		gpio_export(gpio, false);
		omap_mux_init_gpio(gpio,
			OMAP_MUX_MODE4 |
			((pia35x_rfid_gpios[i].flags & GPIOF_DIR_IN) ?
				OMAP_PIN_INPUT_PULLDOWN : OMAP_PIN_OUTPUT));
		gpio_export(gpio, false);
	}

	spi_register_board_info(pia35x_spi_rfid_info,
			ARRAY_SIZE(pia35x_spi_rfid_info));
}

#if defined(CONFIG_AD799X) || defined(CONFIG_AD799X_MODULE)
#include "../../../drivers/staging/iio/adc/ad799x.h"
static struct ad799x_platform_data pia35x_ad799x_info = {
	.vref_mv = 3000,
};

static struct i2c_board_info __initdata pia35x_i2c2_ad799x[] = {
	{
		I2C_BOARD_INFO("ad7994", 0x20),
		.platform_data = &pia35x_ad799x_info,
	},
};

static char * pia35x_trigger_data[2] = { "rtc0", NULL };

#if defined(CONFIG_IIO_PERIODIC_RTC_TRIGGER) || \
		defined(CONFIG_IIO_PERIODIC_RTC_TRIGGER_MODULE)
static struct platform_device pia35x_iio_rtc_trigger = {
	.name = "iio_prtc_trigger",
	.dev = {
		.platform_data = &pia35x_trigger_data,
	},
};
#endif

#else
static struct i2c_board_info __initdata pia35x_i2c2_ad799x[] = {};
#endif


static void __init pia35x_ad799x_init(void)
{
	pr_info("pia35x_init: init AD converter AD799x\n");
#if defined(CONFIG_IIO_PERIODIC_RTC_TRIGGER) || \
		defined(CONFIG_IIO_PERIODIC_RTC_TRIGGER_MODULE)
	platform_device_register(&pia35x_iio_rtc_trigger);
#endif
	i2c_register_board_info(2, pia35x_i2c2_ad799x,
			ARRAY_SIZE(pia35x_i2c2_ad799x));
}

/** piA-IO **/
#if defined(CONFIG_GPIO_PCF857X) || defined (CONFIG_GPIO_PCF857X_MODULE)
/* expander GPIOs after OMAP GPIOs */
#define PIAIO_GPIO_BASE(x) (OMAP_MAX_GPIO_LINES + ((x) * 8))

#include <linux/i2c/pcf857x.h>
/* IO Expander 2 x PCA9672PW: 8 x IN, 8 x OUT */
#define GPIO_IO_OUT_RESET	14
#define GPIO_IO_OUT_INT		15
#define GPIO_IO_IN_RESET	16
#define GPIO_IO_IN_INT		17
static struct gpio pia35x_io_gpios[] = {
	{ GPIO_IO_OUT_RESET, GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
			"piaio.out_reset" },
	{ GPIO_IO_OUT_INT,   GPIOF_DIR_IN | GPIOF_INIT_HIGH,
			"piaio.out_int"  },
	{ GPIO_IO_IN_RESET,  GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
			"piaio.in_reset"},
	{ GPIO_IO_IN_INT,    GPIOF_DIR_IN | GPIOF_INIT_HIGH,
			"piaio.in_int"  },
};
static char *piaio_names[16] = {
	/* first expander, outputs */
	0,
	"piaio.out6",
	"piaio.out5",
	"piaio.out4",
	"piaio.out1",
	"piaio.out2",
	"piaio.out3",
	0,
	/* second expander, inputs */
	0,
	"piaio.in6",
	"piaio.in5",
	"piaio.in4",
	"piaio.in3",
	"piaio.in2",
	"piaio.in1",
	0,
};

static int pia35x_io_gpio_setup(
		struct i2c_client *client,
		int gpio, unsigned ngpio, void *c)
{
	int i = 0;
	int base = gpio - PIAIO_GPIO_BASE(0);

	for (; i < 8; ++i) {
		if (piaio_names[base+i] == 0)
			continue;

		gpio_request(gpio + i, piaio_names[base+i]);
		/* second expander has inputs */
		if (base > 0)
			gpio_direction_input(gpio + i);
		else
			gpio_direction_output(gpio + i, 1);

		if (0 != gpio_export(gpio + i, false))
			pr_err("piAx: error while exporting GPIO%d\n", (gpio+i));
	}

	return 0;
}

static int pia35x_io_gpio_teardown(
		struct i2c_client *client,
		int gpio, unsigned ngpio, void *c)
{
	int i = 0;

	for (; i < 8; ++i)
		gpio_free(gpio + i);

	return 0;
}

static struct pcf857x_platform_data pia35x_io_out_data = {
		.gpio_base = PIAIO_GPIO_BASE(0),
		.setup     = pia35x_io_gpio_setup,
		.teardown  = pia35x_io_gpio_teardown,
};
static struct pcf857x_platform_data pia35x_io_in_data = {
		.gpio_base = PIAIO_GPIO_BASE(1),
		.setup     = pia35x_io_gpio_setup,
		.teardown  = pia35x_io_gpio_teardown,
};

static struct i2c_board_info pia35x_i2c_io_data[] = {
		{
				I2C_BOARD_INFO("pca9672", 0x20),
				.platform_data	= &pia35x_io_out_data,
		},
		{
				I2C_BOARD_INFO("pca9672", 0x21),
				.platform_data	= &pia35x_io_in_data,
		},
};

static void __init pia35x_ioexp_init(void)
{
	int err, i;
	unsigned int gpio;
	unsigned long flags;

	pr_info("pia35x: Initializing piA-IO board");
	if (0 != (err = gpio_request_array(pia35x_io_gpios,
			ARRAY_SIZE(pia35x_io_gpios)))) {
		pr_warning("pia35x: unable to request IO expander GPIOs: %d", err);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(pia35x_io_gpios); i++) {
		gpio  = pia35x_io_gpios[i].gpio;
		flags = pia35x_io_gpios[i].flags;

		omap_mux_init_gpio(gpio, OMAP_MUX_MODE4	| (flags & GPIOF_DIR_IN) ?
						OMAP_PIN_INPUT_PULLDOWN : OMAP_PIN_OUTPUT);
		gpio_export(gpio, false);
	}

	i2c_register_board_info(2, pia35x_i2c_io_data,
			ARRAY_SIZE(pia35x_i2c_io_data));
}
#else
static inline void __init pia35x_ioexp_init(void) {
	pr_err("pia35x: piA-IO Expander driver PCA9672 missing\n");
}
#endif /* CONFIG_GPIO_PCF857X */

/** piA-EMS_IO **/
#if (defined(CONFIG_GPIO_PCF857X) || defined(CONFIG_GPIO_PCF857X_MODULE)) && \
	(defined(CONFIG_CAN_MCP251X) || defined(CONFIG_CAN_MCP251X_MODULE)) && \
	(defined(CONFIG_SERIAL_MAX3100) || defined(CONFIG_SERIAL_MAX3100_MODULE) || \
	defined(CONFIG_SERIAL_MAX3140) || defined(CONFIG_SERIAL_MAX3140_MODULE))
#include <linux/can/platform/mcp251x.h>
#include <linux/serial_max3140hd.h>

#define EMS_IO_CAN_DEV(bus, cs, irqgpio, id) \
{	.modalias      = "mcp2515", \
	.bus_num       = bus, \
	.chip_select   = cs, \
	.max_speed_hz  = 4E6, \
	.mode          = SPI_MODE_0, \
	.irq           = OMAP_GPIO_IRQ(irqgpio), \
	.controller_data = &ems_io_mcp2515_cfg[id], \
	.platform_data = &ems_io_mcp2515_data[id], \
}
#define EMS_IO_485_DEV(bus, cs, irqgpio, id) \
{ \
	.modalias      = "max3140hd", \
	.bus_num       = bus, \
	.chip_select   = cs, \
	.max_speed_hz  = 4E6, /* DS min 238ns period */ \
	.mode          = SPI_MODE_0, \
	.irq           = OMAP_GPIO_IRQ(irqgpio), \
	.controller_data = &ems_io_max3140_cfg[id], \
	.platform_data = &ems_io_max3140_data[id], \
}
#define EMS_IO_GPIO_DEV(ad, irqgpio, id) \
{	I2C_BOARD_INFO("pca9672", ad), \
	.irq           = OMAP_GPIO_IRQ(irqgpio), \
	.platform_data = &ems_io_pca9672_data[id], \
}
#define EMS_IO_GPIO_DEV_DATA(id) \
{	.gpio_base = PIAIO_GPIO_BASE(id), \
	.setup     = ems_io_gpio_setup, \
	.teardown  = ems_io_gpio_teardown \
}

static struct mcp251x_platform_data ems_io_mcp2515_data[3] = {
	{ .oscillator_frequency = 25E6 }, /* CAN 1 */
	{ .oscillator_frequency = 25E6 }, /* CAN 2 */
	{ .oscillator_frequency = 25E6 }, /* CAN 3 */
};

static struct plat_max3140hd ems_io_max3140_data[4] = {
	{ .loopback = 0, .crystal = 1, .poll_time = 0, .invert_rts = 1 },
	{ .loopback = 0, .crystal = 1, .poll_time = 0, .invert_rts = 1 },
	{ .loopback = 0, .crystal = 1, .poll_time = 0, .invert_rts = 1 },
	{ .loopback = 0, .crystal = 1, .poll_time = 0, .invert_rts = 1 },
};

/* 3 CAN + 4 RS485 on SPI busses 1+2 */
static struct omap2_mcspi_device_config ems_io_mcp2515_cfg[] = {
	{ .turbo_mode	= 0, .single_channel	= 1 },
	{ .turbo_mode	= 0, .single_channel	= 1 },
	{ .turbo_mode	= 0, .single_channel	= 1 },
};
static struct omap2_mcspi_device_config ems_io_max3140_cfg[] = {
	{ .turbo_mode	= 0, .single_channel	= 1 },
	{ .turbo_mode	= 0, .single_channel	= 1 },
	{ .turbo_mode	= 0, .single_channel	= 1 },
	{ .turbo_mode	= 0, .single_channel	= 1 },
};
static struct spi_board_info pia35x_ems_io_spi_info[] = {
	EMS_IO_CAN_DEV(1, 0, 132, 0),
	EMS_IO_CAN_DEV(1, 2, 133, 1),
	EMS_IO_CAN_DEV(2, 0, 134, 2),
	EMS_IO_485_DEV(1, 1, 135, 0),
	EMS_IO_485_DEV(1, 3, 136, 1),
	EMS_IO_485_DEV(2, 1, 137, 2),
	EMS_IO_485_DEV(2, 2, 138, 3),
};

enum {
	EMS_IO_DOUT,
	EMS_IO_DIN1,
	EMS_IO_DIN2,
	EMS_IO_TERM,
	EMS_IO_DISP,
};
/* mapping of expander GPIOs to names */
#define EMS_IO_GPIO(e, n, o, ex) \
{	.expander = e, .name = n, .out = o, .export = ex }
struct ems_io_gpio_config {
	unsigned expander; /* expander id */
	char    *name;     /* exported name */
	unsigned out;      /* is output? */
	unsigned export;   /* auto export? */
};

/* all gpios on expanders ordered by expander + pin */
static struct ems_io_gpio_config ems_io_gpios[5*8] = {
	/* DOUT 0..7 */
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.1", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.2", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.3", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.4", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.5", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.6", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.7", 1, 1),
	EMS_IO_GPIO(EMS_IO_DOUT, "dout.8", 1, 1),
	/* DIN 8..19 */
	EMS_IO_GPIO(EMS_IO_DIN1, "din.1", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.2", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.3", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.4", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.5", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.6", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.7", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN1, "din.8", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN2, "din.9", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN2, "din.10", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN2, "din.11", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN2, "din.12", 0, 1),
	EMS_IO_GPIO(EMS_IO_DIN2, "display.fehler", 1, 1),
	EMS_IO_GPIO(EMS_IO_DIN2, NULL, 0, 0),
	EMS_IO_GPIO(EMS_IO_DIN2, NULL, 0, 0),
	EMS_IO_GPIO(EMS_IO_DIN2, NULL, 0, 0),
	/* TERM */
	EMS_IO_GPIO(EMS_IO_TERM, "term.can.1", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, "term.rs485.1", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, "term.can.2", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, "term.rs485.2", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, "term.can3", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, "term.rs485.3", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, "term.rs485.4", 1, 1),
	EMS_IO_GPIO(EMS_IO_TERM, NULL, 1, 1),
	/* DISPLAY */
	EMS_IO_GPIO(EMS_IO_DISP, "display.enter", 0, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.esc", 0, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.rechts", 0, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.links", 0, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.ab", 0, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.auf", 0, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.led_einspeis", 1, 1),
	EMS_IO_GPIO(EMS_IO_DISP, "display.led_netz", 1, 1),
};

static int ems_io_gpio_setup(
		struct i2c_client *client,
		int gpio, unsigned ngpio, void *c)
{
	int i;
	int base = gpio - OMAP_MAX_GPIO_LINES;
	struct ems_io_gpio_config *cur;

	for (i = 0; i < 8; ++i) {
		cur = &ems_io_gpios[base+i];
		if ((NULL == cur) || (NULL == cur->name))
			continue;

		gpio_request(gpio + i, cur->name);
		if (cur->out)
			gpio_direction_output(gpio + i, 0);
		else
			gpio_direction_input(gpio + i);

		if (0 == cur->export)
			continue;

		if (0 != gpio_export(gpio + i, false))
			pr_err("piAx: EMS IO couldn't export GPIO %d on expander %d\n",
					(gpio + i), cur->expander);
	}

	return 0;
}

static int ems_io_gpio_teardown(
		struct i2c_client *client,
		int gpio, unsigned ngpio, void *c)
{
	int i = 0;

	for (; i < 8; ++i)
		gpio_free(gpio + i);

	return 0;
}

static struct pcf857x_platform_data ems_io_pca9672_data[5] = {
	EMS_IO_GPIO_DEV_DATA(0),
	EMS_IO_GPIO_DEV_DATA(1),
	EMS_IO_GPIO_DEV_DATA(2),
	EMS_IO_GPIO_DEV_DATA(3),
	EMS_IO_GPIO_DEV_DATA(4),
};

static struct i2c_board_info pia35x_ems_io_i2c_info[] = {
	EMS_IO_GPIO_DEV(0x20,  0, EMS_IO_DOUT),
	EMS_IO_GPIO_DEV(0x21, 21, EMS_IO_DIN1),
	EMS_IO_GPIO_DEV(0x22, 19, EMS_IO_DIN2),
	EMS_IO_GPIO_DEV(0x23,  0, EMS_IO_TERM),
	EMS_IO_GPIO_DEV(0x12, 17, EMS_IO_DISP),
};

#define GPIO_EMS_IO_RESET    14
#define GPIO_EMS_IO_DIN1_INT 21
#define GPIO_EMS_IO_DIN2_INT 19
#define GPIO_EMS_IO_DISP_INT 17
#define GPIO_EMS_IO_SPI2_CS0 181
#define GPIO_EMS_IO_SPI2_CS1 182
#define GPIO_EMS_IO_SPI2_CS2 12
static struct gpio pia35x_ems_io_gpios[] = {
	{ GPIO_EMS_IO_RESET, GPIOF_DIR_OUT | GPIOF_INIT_LOW,
			"emsio.reset" },
	{ GPIO_EMS_IO_DIN1_INT, GPIOF_DIR_IN, "emsio.din1_int"  },
	{ GPIO_EMS_IO_DIN2_INT, GPIOF_DIR_IN, "emsio.din2_int"},
	{ GPIO_EMS_IO_DISP_INT, GPIOF_DIR_IN, "emsio.disp_int"  },
	{ GPIO_EMS_IO_SPI2_CS0, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, NULL },
	{ GPIO_EMS_IO_SPI2_CS1, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, NULL },
	{ GPIO_EMS_IO_SPI2_CS2, GPIOF_DIR_OUT | GPIOF_INIT_HIGH, NULL },
};

static void __init pia35x_ems_io_init(void) {
	unsigned i;

	pr_info("pia35x: Initializing piA-EMS_IO board");

	/* GPIOs */
	gpio_request_array(pia35x_ems_io_gpios, ARRAY_SIZE(pia35x_ems_io_gpios));
	usleep_range(4, 10); /* reset must be held for at least 4us */
	gpio_set_value(GPIO_EMS_IO_RESET, 1);

	for (i = 0; i < ARRAY_SIZE(pia35x_ems_io_gpios); ++i) {
		if (NULL == pia35x_ems_io_gpios[i].label)
			continue;

		if (0 != gpio_export(pia35x_ems_io_gpios[i].gpio, false))
			pr_err("piAx: EMS IO couldn't export %s\n",
					pia35x_ems_io_gpios[i].label);
	}

	/* IO expander */
	i2c_register_board_info(2, pia35x_ems_io_i2c_info,
			ARRAY_SIZE(pia35x_ems_io_i2c_info));

	/* setup SPI2 GPIO CS */
	mcspi2_cs_gpios[0] = GPIO_EMS_IO_SPI2_CS0;
	mcspi2_cs_gpios[1] = GPIO_EMS_IO_SPI2_CS1;
	mcspi2_cs_gpios[2] = GPIO_EMS_IO_SPI2_CS2;
	mcspi2_cs_gpios[3] = 0;

	/* prototype boards used inverted rts logic */
	if (0 == strcmp(expansionboard_name, "pia_ems_io")) {
		for (i = 0; i < 4; ++i)
			ems_io_max3140_data[i].invert_rts = 0;
	}

//	for (i = 132; i <= 138; ++i)
//		set_irq_type(OMAP_GPIO_IRQ(i), IRQ_TYPE_EDGE_FALLING);

	/* SPI */
	spi_register_board_info(pia35x_ems_io_spi_info,
			ARRAY_SIZE(pia35x_ems_io_spi_info));
}
#else
static inline void __init pia35x_ems_io_init(void) {
	pr_err("pia35x: piA-EMS_IO driver PCA9672|MCP2515|MAX3140 missing\n");
}
#endif

/** Integrated Devices **/
#define GPIO_EN_VCC_5V_PER  28    /* expansion supply voltage */
#define GPIO_USB_SW        116    /* USB switch OTG/GSM-UMTS, piA only */
#define GPIO_STATUS_LED    117    /* Status LED with heartbeat functionality */
/** same for piAx variant */
#define GPIOX_EN_VCC_5V_PER 28    /* expansion supply voltage */
#define GPIOX_STATUS_LED    26

/* Board initialization */
static struct omap_board_config_kernel pia35x_config[] __initdata = {
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* only fixed MUXes for all board variants here,
	   don't add anything on expansions */

	/* MMC1_CD        GPIO 041, low == card in slot */
	OMAP3_MUX(GPMC_A8, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

	/* UART2.485/#232 GPIO 128, low = RS232 */
	OMAP3_MUX(SDMMC1_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
	/* UART2.SLEW     GPIO 129 */
	OMAP3_MUX(SDMMC1_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* INPUT_GPIO1    GPIO 055 */
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	/* INPUT_GPIO2    GPIO 056 */
	OMAP3_MUX(GPMC_NCS5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),

	/*        GPIO 136 */
	//FIXME OMAP3_MUX(SDMMC2_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* UART 4 tx */
	OMAP3_MUX(SAD2D_MCAD1, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),
	/* UART 4 rx */
	OMAP3_MUX(SAD2D_MCAD4, OMAP_MUX_MODE2 | OMAP_PIN_INPUT),
	/* UART 4 rts */
	OMAP3_MUX(SAD2D_MCAD2, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),
	/* UART 4 cts */
	OMAP3_MUX(SAD2D_MCAD3, OMAP_MUX_MODE2 | OMAP_PIN_INPUT),

	/* TTLIOs 1+2, not present in revision 0.1.0 */
	OMAP3_MUX(GPMC_A3, OMAP_MUX_MODE7),
	OMAP3_MUX(GPMC_A4, OMAP_MUX_MODE7),
	OMAP3_MUX(GPMC_A5, OMAP_MUX_MODE7),
	OMAP3_MUX(GPMC_A6, OMAP_MUX_MODE7),

	/* TERMINATOR */
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

#if 0 /* USB EHCI port only on expansion port */
static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};
#endif

/*
 * Audio TLV320AIC3204IRHB
 */
#if defined(CONFIG_SND_SOC_TLV320AIC32X4) || \
	defined(CONFIG_SND_SOC_TLV320AIC32X4_MODULE)
#include <sound/tlv320aic32x4.h>
static struct aic32x4_pdata pia35x_aic320_data __initdata = {
	.power_cfg = (
		AIC32X4_PWR_CMMODE_LDOIN_RANGE_18_36 |
		AIC32X4_PWR_CMMODE_HP_LDOIN_POWERED |
		AIC32X4_PWR_AVDD_DVDD_WEAK_DISABLE |
		AIC32X4_PWR_AIC32X4_LDO_ENABLE
	),
};

static struct i2c_board_info __initdata pia35x_i2c2_aic3x[] = {
	{
		I2C_BOARD_INFO("tlv320aic32x4", 0x18),
		.platform_data = &pia35x_aic320_data,
	},
};
#else
static struct i2c_board_info __initdata pia35x_i2c2_aic3x[] = {};
#endif /* CONFIG_SND_SOC_TLV320AIC23 */

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux_audio[] __initdata = {
	/* I2S codec port pins for McBSP block */
	/* SYS_CLKOUT1 connected to SYS_CLKOUT2 */
	OMAP3_MUX(SYS_CLKOUT1, OMAP_MUX_MODE7 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_FSX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_CLKX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DR, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	OMAP3_MUX(MCBSP2_DX, OMAP_MUX_MODE0 | OMAP_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux_audio NULL
#endif /* CONFIG_OMAP_MUX */

static int __init pia35x_audio_init(void)
{
	int ret = 0;

	if (pia35x_version == PIA_AM3505) return 0;

	pr_info("pia35x: init audio device TLV320-AIC3204");

	if (0 != pia35x_sys_clkout2_init()) {
		pr_warn("pia35x: Could not initialize Audio Clock!");
		return -1;
	}

	if ((ret = omap3_mux_init(board_mux_audio, OMAP_PACKAGE_CBB)) != 0) {
		pr_warn("pia35x: failed to init audio mux!");
		return ret;
	}

	i2c_register_board_info(2, pia35x_i2c2_aic3x,
			ARRAY_SIZE(pia35x_i2c2_aic3x));

	return ret;
}

/*
 * Ethernet (internal) & PHY (SMSC LAN8720A-CP)
 */
#define GPIO_ETHERNET_NRST  65    /* Ethernet RESET */
#define AM35XX_EVM_MDIO_FREQUENCY	(1000000)

static struct mdio_platform_data pia35x_evm_mdio_pdata = {
	.bus_freq = AM35XX_EVM_MDIO_FREQUENCY,
};

static struct resource pia35x_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET + SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device pia35x_mdio_device = {
	.name              = "davinci_mdio",
	.id                = -1,
	.num_resources     = ARRAY_SIZE(pia35x_mdio_resources),
	.resource          = pia35x_mdio_resources,
	.dev.platform_data = &pia35x_evm_mdio_pdata,
};

static struct emac_platform_data pia35x_emac_pdata = {
	.rmii_en = 1,
};

static struct resource pia35x_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device pia35x_emac_device = {
	.name           = "davinci_emac",
	.id	            = -1,
	.num_resources  = ARRAY_SIZE(pia35x_emac_resources),
	.resource       = pia35x_emac_resources,
};

/*
 * initialize MAC address from boot parameter eth=<MAC>
 */
static int __init eth_addr_setup(char *str)
{
	int i;

	if (str == NULL)
		return 0;
	for (i = 0; i < ETH_ALEN; i++)
		pia35x_emac_pdata.mac_addr[i] = simple_strtol(&str[i*3],
				(char **)NULL, 16);
	return 1;
}
/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

/*
 * disable ETH interrupt
 */
static void pia35x_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR |
			AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
			AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

/*
 * enable ETH interrupt
 */
static void pia35x_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

/*
 * ETH init
 * make sure ETH module is powered
 * initialize MAC, PHY and configurationn
 */
static void __init pia35x_ethernet_init(struct emac_platform_data *pdata)
{
	u32 regval, mac_lo, mac_hi;
	int res;

	pr_info("pia35x_init: init ETH\n");
	/* unset reset */
	if ((res = gpio_request_one(GPIO_ETHERNET_NRST,
			GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "ethernet-nrst")) != 0) {
		pr_warn("%s : Unable to request ETHERNET_nRST GPIO: %d\n",
				__func__, res);
	} else {
		omap_mux_init_gpio(GPIO_ETHERNET_NRST, OMAP_PIN_OUTPUT);
		gpio_export(GPIO_ETHERNET_NRST, false);
	}
	mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
	mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

	pdata->mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
	pdata->mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00)   >> 8);
	pdata->mac_addr[2] = (u_int8_t)((mac_hi & 0xFF)     >> 0);
	pdata->mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
	pdata->mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00)   >> 8);
	pdata->mac_addr[5] = (u_int8_t)((mac_lo & 0xFF)     >> 0);

	pdata->ctrl_reg_offset      = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset  = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset      = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size        = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version              = EMAC_VERSION_2;
	pdata->hw_ram_addr          = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable     = pia35x_enable_ethernet_int;
	pdata->interrupt_disable    = pia35x_disable_ethernet_int;
	pia35x_emac_device.dev.platform_data = pdata;
	platform_device_register(&pia35x_mdio_device);
	platform_device_register(&pia35x_emac_device);
	clk_add_alias(NULL, dev_name(&pia35x_mdio_device.dev),
		      NULL, &pia35x_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
}

/*
 * CAN - HECC
 */
static struct resource pia35x_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device pia35x_hecc_device = {
	.name           = "ti_hecc",
	.id	            = -1,
	.num_resources  = ARRAY_SIZE(pia35x_hecc_resources),
	.resource       = pia35x_hecc_resources,
};

static struct ti_hecc_platform_data pia35x_hecc_pdata = {
	.scc_hecc_offset = AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset  = AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset = AM35XX_HECC_RAM_OFFSET,
	.mbx_offset      = AM35XX_HECC_MBOX_OFFSET,
	.int_line        = AM35XX_HECC_INT_LINE,
	.version         = AM35XX_HECC_VERSION,
	//.transceiver_switch     = hecc_phy_control,
};

#define GPIO_CAN_RES        26    /* resistor switch for CAN */
#define GPIOX_CAN_RES       36
static void __init pia35x_can_init(struct ti_hecc_platform_data *pdata)
{
	pr_info("pia35x_init: init CAN");

	pia35x_hecc_device.dev.platform_data = pdata;
	platform_device_register(&pia35x_hecc_device);

	if (pia35x_version == PIA_AM3505) {
		if (gpio_request_one(GPIO_CAN_RES,
				GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "can.res") != 0) {
			pr_warning("pia35x: unable to request CAN_RES");
		} else {
			omap_mux_init_gpio(GPIO_CAN_RES, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIO_CAN_RES, false);
		}
	} else if (pia35x_version == PIA_X_AM3517) {
		if (gpio_request_one(GPIOX_CAN_RES,
				GPIOF_DIR_OUT | GPIOF_INIT_HIGH, "can.res") != 0) {
			pr_warning("pia35x: unable to request CAN_RES");
		} else {
			omap_mux_init_gpio(GPIOX_CAN_RES, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIOX_CAN_RES, false);
		}
	} else {
		pr_warn("pia35x_init: CAN resistor uninitialized: unknown piA version");
	}
}

/*
 * MMC
 */
/* MMC1 has fixed power supply */
static struct regulator_consumer_supply pia35x_vmmc1_consumers[] = {
	REGULATOR_SUPPLY("vmmc1", "mmci-omap-hs.0"),
};

static struct regulator_init_data pia35x_vmmc1_data = {
	.constraints = {
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.always_on		  = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(pia35x_vmmc1_consumers),
	.consumer_supplies	    = pia35x_vmmc1_consumers,
};

static struct fixed_voltage_config pia35x_vmmc1_config = {
	.supply_name     = "vmmc1",
	.microvolts      = 3300000,  /* 3.3V */
	.gpio            = -EINVAL,
	//.enabled_at_boot = 1,
	.init_data       = &pia35x_vmmc1_data,
};

static struct platform_device pia35x_vmmc1_device = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data = &pia35x_vmmc1_config,
	},
};

/*
 * Voltage Regulator
 */
#if defined(CONFIG_REGULATOR_TPS6507X)
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
			.min_uV           = 1800000,
			.max_uV           = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask   = REGULATOR_CHANGE_STATUS,
			.always_on        = true,
			.apply_uV         = true,
		},
		.num_consumer_supplies = ARRAY_SIZE(pia35x_vdd1_consumers),
		.consumer_supplies     = &pia35x_vdd1_consumers[0],
	},
	/* dcdc2: VDDSHV_3V3 */
	{
		.constraints = {
			.min_uV           = 3300000,
			.max_uV           = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask   = REGULATOR_CHANGE_STATUS,
			.always_on        = true,
			.apply_uV         = true
		},
		.num_consumer_supplies = ARRAY_SIZE(pia35x_vdd2_consumers),
		.consumer_supplies = &pia35x_vdd2_consumers[0],
		/* select high = 3.3V (low is 1.8) */
		.driver_data = &pia35x_tps_vdd2_platform_data,
	},
	/* dcdc3: VDDCORE_1V2 */
	{
		.constraints = {
			.min_uV           = 1200000,
			.max_uV           = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask   = REGULATOR_CHANGE_STATUS,
			.always_on        = true,
			.apply_uV         = true
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
			.always_on        = true,
			.apply_uV         = true,
			//.boot_on          = 1,
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
			.always_on        = true,
			.apply_uV         = true,
			//.boot_on          = 1,
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
#endif /* CONFIG_REGULATOR_TPS6507X */

/*
 * MMC
 */
static struct omap2_hsmmc_info mmc_single[] = {
	/* first MMC port used for system MMC modules */
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = 41,
		.gpio_wp        = -EINVAL, /* we don't have a WP pin connected, was: 40 */
		//.ocr_mask       = MMC_VDD_33_34,
	},
	{}/* Terminator */
};

static struct omap2_hsmmc_info mmc_wlan[] = {
	/* first MMC port used for system MMC modules */
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = 41,
		.gpio_wp        = -EINVAL, /* we don't have a WP pin connected, was: 40 */
		//.ocr_mask       = MMC_VDD_33_34,
	},
#if defined(CONFIG_WL1271) || defined (CONFIG_WL1271_MODULE)
	{
		.name           = "wl1271",
		.mmc            = 2,
		.caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_wp        = -EINVAL,
		.gpio_cd        = -EINVAL,
		.nonremovable   = true,
		.ext_clock      = false,
		//.ocr_mask       = MMC_VDD_165_195 | MMC_VDD_33_34,
	},
#endif /* CONFIG_WL12XX */
	{}/* Terminator */
};
static struct omap2_hsmmc_info *mmc;

static void __init pia35x_mmc_init(void)
{
	pr_info("pia35x_init: init MMC\n");

	/* predefined mmc configs depending on expansion board */
	if (0 == strcmp(expansionboard_name, "pia_wifi"))
		mmc = mmc_wlan;
	else
		mmc = mmc_single;

	omap2_hsmmc_init(mmc);
	/* link regulator to on-board MMC adapter */
	//TODO pia35x_vmmc1_consumers[0].dev = mmc[0].dev;
	platform_device_register(&pia35x_vmmc1_device);
}

/*
 * MUSB (USB OTG)
 */
static struct omap_musb_board_data pia35x_musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_UNDEFINED,
	.power                  = 500,
	.set_phy_power		= am35x_musb_phy_power,
	.clear_irq		= am35x_musb_clear_irq,
	.set_mode		= am35x_set_mode,
	.reset			= am35x_musb_reset,
};

static __init void pia35x_musb_init(void)
{
	u32 devconf2;
	int err;

	pr_info("pia35x_init: init USB OTG\n");
	/* Set up USB clock/mode in the DEVCONF2 register. */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	switch (pia35x_version) {
	case PIA_AM3505:
		/* set USB OTG-UMTS switch to OTG by default */
		err = gpio_request_one(GPIO_USB_SW,
				GPIOF_DIR_OUT | GPIOF_INIT_LOW, "usb.sw");
		if (err != 0) {
			pr_warning("pia35x: unable to request USB_SW");
		} else {
			omap_mux_init_gpio(GPIO_USB_SW,
					OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN);
			gpio_export(GPIO_USB_SW, false);
		}
		/* automatical detection in OTG mode doesn't work atm */
		pia35x_musb_board_data.mode = MUSB_HOST;

		break;
	case PIA_X_AM3517:
		pia35x_musb_board_data.mode = MUSB_HOST;

		break;
	}

	usb_musb_init(&pia35x_musb_board_data);
}

/*
 * NAND
 * we use GPMC CS 0
 */
#define PIA35X_NAND_CS 0
static struct mtd_partition pia35x_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{ /* 0x00000000 - 0x00080000 */
		.name           = "xloader-nand",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{ /* 0x00080000 - 0x00260000 */
		.name           = "uboot-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 15*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{ /* 0x00260000 - 0x00280000 */
		.name           = "params-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 1*(SZ_128K)
	},
	{ /* 0x00280000 - 0x00680000 */
		.name           = "linux-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 32*(SZ_128K)
	},
	{ /* */
		.name           = "jffs2-nand",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};


static void __init pia35x_flash_init(void)
{
	pr_info("pia35x_init: init NAND\n");

	board_nand_init(pia35x_nand_partitions,
			ARRAY_SIZE(pia35x_nand_partitions),
			PIA35X_NAND_CS, NAND_BUSWIDTH_16);
}


/*
 * Serial ports RS232/485
 */
#define GPIO_RS485_RES      27    /* resistor switch for RS485 */
#define GPIO_RS232_RXEN    144    /* enable RS232/RS485 receiver */
#define GPIO_RS232_DEN     145    /* driver enable for RS485, low for RS232 */
#define GPIOX_RS485_RES     42
#define GPIOX_RS232_RXEN    34
#define GPIOX_RS232_DEN     35
static void __init pia35x_serial_init(void)
{
	pr_info("pia35x_init: init serial ports\n");

	if (pia35x_version == PIA_AM3505) {
		if (gpio_request_one(GPIO_RS485_RES,
				GPIOF_DIR_OUT | GPIOF_OUT_INIT_LOW, "rs485.res") != 0) {
			pr_warning("pia35x: unable to request RS485_RES");
		} else {
			omap_mux_init_gpio(GPIO_RS485_RES, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIO_RS485_RES, false);
		}
		if (gpio_request_one(GPIOX_RS232_DEN,
				GPIOF_DIR_OUT | GPIOF_OUT_INIT_LOW, "rs232x.den") != 0) {
			pr_warning("pia35x: unable to request piAx RS232_DEN");
		} else {
			omap_mux_init_gpio(GPIOX_RS232_DEN, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIOX_RS232_DEN, false);
		}
		if (gpio_request_one(GPIO_RS232_RXEN,
				GPIOF_DIR_OUT | GPIOF_OUT_INIT_HIGH, "rs232.rxen") != 0) {
			pr_warning("pia35x: unable to request RS232_RXEN");
		} else {
			omap_mux_init_gpio(GPIO_RS485_RES, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIO_RS232_RXEN, false);
		}
	} else if (pia35x_version == PIA_X_AM3517) {
		/* piAx */
		if (gpio_request_one(GPIOX_RS485_RES,
				GPIOF_DIR_OUT | GPIOF_OUT_INIT_LOW, "rs485.res") != 0) {
			pr_warning("pia35x: unable to request RS485_RES");
		} else {
			omap_mux_init_gpio(GPIOX_RS485_RES, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIOX_RS485_RES, false);
		}
		if (gpio_request_one(GPIOX_RS232_RXEN,
				GPIOF_DIR_OUT | GPIOF_OUT_INIT_HIGH, "rs232x.rxen") != 0) {
			pr_warning("pia35x: unable to request piAx RS232_RXEN");
		} else {
			omap_mux_init_gpio(GPIOX_RS232_RXEN, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIOX_RS232_RXEN, false);
		}
		if (gpio_request_one(GPIOX_RS232_DEN,
				GPIOF_DIR_OUT | GPIOF_OUT_INIT_LOW, "rs232x.den") != 0) {
			pr_warning("pia35x: unable to request piAx RS232_DEN");
		} else {
			omap_mux_init_gpio(GPIOX_RS232_DEN, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
			gpio_export(GPIOX_RS232_DEN, false);
		}
	}
	omap_serial_init();
}

/*
 * I2C
 */
#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
#include <linux/i2c/at24.h>
static struct at24_platform_data m24c01_exp = {
	.byte_len       = SZ_1K / 8,
	.page_size      = 16,
};

static struct at24_platform_data m24c01_lcd = {
	.byte_len       = SZ_1K / 8,
	.page_size      = 16,
};

static struct at24_platform_data eeprom_piax_data = {
	.byte_len       = SZ_2K / 8, /* 128 bytes */
	.page_size      = 8,         /* 8 bytes pages */
	.flags          = AT24_FLAG_TAKE8ADDR, /* no addr pins */
};
#endif /* CONFIG_EEPROM_AT24 */

static int ds1374_keep_wace = 1;

static struct i2c_board_info __initdata pia35x_i2c1_info[] = {
#if defined(CONFIG_REGULATOR_TPS6507X)
	{ /* power regulator TPS650732 */
		I2C_BOARD_INFO("tps6507x", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &pia35x_tps_board,
	},
#endif /* CONFIG_REGULATOR_TPS6507X */
#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
	/* piAx only 24AA02E48 on board eeprom with node ID */
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data  = &eeprom_piax_data,
	},
#endif /* CONFIG_EEPROM_AT24 */
	/* RTC + WDOG */
	{
		I2C_BOARD_INFO("ds1374", 0x68),
		.platform_data = &ds1374_keep_wace,
	},
};

static struct i2c_board_info __initdata pia35x_i2c2_info[] = {
	/* temperature sensor LM75 */
	{
		I2C_BOARD_INFO("lm75", 0x48),
	},
#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
	/* expansion board eeprom */
	{
		I2C_BOARD_INFO("24c01", 0x50),
		.platform_data  = &m24c01_exp,
	},
#endif /* CONFIG_EEPROM_AT24 */
};

static struct i2c_board_info __initdata pia35x_i2c3_info[] = {
#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)
	/* expansion board eeprom */
	{
		I2C_BOARD_INFO("24c01", 0x51),
		.platform_data  = &m24c01_lcd,
	},
#endif /* CONFIG_EEPROM_AT24 */
};

static int __init pia35x_i2c_init(void)
{
	pr_info("pia35x_init: init I2C busses\n");

	omap_register_i2c_bus(1, 400, pia35x_i2c1_info, ARRAY_SIZE(pia35x_i2c1_info));
	omap_register_i2c_bus(2, 400, pia35x_i2c2_info, ARRAY_SIZE(pia35x_i2c2_info));
	omap_register_i2c_bus(3, 100, pia35x_i2c3_info, ARRAY_SIZE(pia35x_i2c3_info));

	return 0;
}

#define GPIO_VERSION_DETECT 151 /* GPIO 151 has an external pull-up on piA */
static int __init pia35x_version_detect(void)
{
	int val;

	pr_info("pia35x_init: detecting piA version\n");

	if (gpio_request_one(GPIO_VERSION_DETECT,
			GPIOF_DIR_IN, "gpio.vdetect") != 0) {
		pr_warning("pia35x: unable to request VERSION_DETECT");
		return -1;
	} else {
		omap_mux_init_gpio(GPIO_VERSION_DETECT, OMAP_MUX_MODE4 | OMAP_PIN_INPUT);
	}
	msleep(10);
	val = gpio_get_value(GPIO_VERSION_DETECT);
	if (val == 0 && omap3_has_sgx()) {
		/* piAx has AM3517, but never GSM module */
		pr_info("pia35x: piAx-AM3517");
		pia35x_version = PIA_X_AM3517;
	} else {
		pia35x_version = PIA_AM3505;
		pr_info("pia35x: piA-AM3505");
	}

	/* reset to mode 0 */
	gpio_free(GPIO_VERSION_DETECT);
	omap_mux_init_signal("uart1_rx.uart1_rx", OMAP_PIN_INPUT);

	return pia35x_version;
}

/* Status LED */
static struct gpio_led gpio_leds_pia[] = {
	{
		.name				= "led1",
		.default_trigger	= "heartbeat",
		.gpio				= GPIO_STATUS_LED,
		.active_low			= false,
	}
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= NULL,
	.num_leds	= 0,
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id		= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static void __init pia35x_status_led_init(void)
{
	if (pia35x_version == PIA_X_AM3517)
		gpio_leds_pia[0].gpio = GPIOX_STATUS_LED;

	pr_info("pia35x_init: activating status heart beat on GPIO %d",
			gpio_leds_pia[0].gpio);

	omap_mux_init_gpio(gpio_leds_pia[0].gpio,
			OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT);
	gpio_led_info.leds = gpio_leds_pia;
	gpio_led_info.num_leds = ARRAY_SIZE(gpio_leds_pia);

	platform_device_register(&leds_gpio);
}

static int __init pia35x_expansion_init(void)
{
	int ret = 0;

	pia35x_gsm_init();

	if (0 == strcmp(expansionboard_name, "pia_wifi")) {
		pr_info("pia35x_init: init WLAN & BT\n");
		pia35x_wlan_init();
		pia35x_bt_init();
		pia35x_ad799x_init();
		pia35x_rfid_init();
		ret++;
	} else if (0 == strcmp(expansionboard_name, "pia_motorcontrol")) {
		pia35x_motorcontrol_init();
		ret++;
	} else if (0 == strcmp(expansionboard_name, "pia_io")) {
		pia35x_ioexp_init();
		ret++;
	} else if (0 == strncmp(expansionboard_name, "pia_ems_io", 10)) {
		pia35x_ems_io_init();
		ret++;
	}

	return ret;
}

/* base initialization function */
static int __init expansionboard_setup(char *str)
{
	if (!str)
		return -EINVAL;

	strncpy(expansionboard_name, str, 32);
	printk(KERN_INFO "pia35x expansionboard: %s\n", expansionboard_name);

	return 0;
}

static int __init lcdboard_setup(char *str)
{
	if (!str)
		return -EINVAL;

	strncpy(lcdboard_name, str, 16);
	printk(KERN_INFO "pia35x LCD: %s\n", lcdboard_name);

	return 0;
}

static void __init pia35x_init(void)
{
	int ret;

	omap_board_config = pia35x_config;
	omap_board_config_size = ARRAY_SIZE(pia35x_config);
	pia35x_version_detect();
	pr_info("pia35x: init pin mux\n");
	ret = omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	if (ret)
		pr_warning("pia35x: MUX init failed: %d\n", ret);

	/* EN_VCC_5V_PER  GPIO 028, low active */
	ret = gpio_request_one(GPIO_EN_VCC_5V_PER, GPIOF_OUT_INIT_HIGH,
			"vccen.per");
	if ( ret != 0) {
		pr_warning("pia35x: unable to request EN_VCC_5V_PER GPIO");
	} else {
		omap_mux_init_gpio(GPIO_EN_VCC_5V_PER,
				OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN);
		gpio_export(GPIO_EN_VCC_5V_PER, false);
	}

	pia35x_i2c_init();
	omap_display_init(&pia35x_dss_data);
	pia35x_serial_init();
	omap_sdrc_init(NULL, NULL);
	pia35x_status_led_init();

	pia35x_display_init();

	pia35x_flash_init();
	pia35x_musb_init();
	pia35x_ethernet_init(&pia35x_emac_pdata);
	pia35x_can_init(&pia35x_hecc_pdata);
	pia35x_mmc_init();

	pia35x_audio_init();

	pia35x_expansion_init();
}

early_param("buddy", expansionboard_setup);
early_param("buddy_lcd", lcdboard_setup);

//FIXME where do I cal gpmc_init?
//static void __init pia35x_init_irq(void)
//{
//	gpmc_init();
//}

MACHINE_START(PIA_AM35X, "PIA AM35X")
	.atag_offset  = 0x100,
	.map_io       = omap3_map_io,
	.init_early   = am35xx_init_early,
	.reserve      = omap_reserve,
	.init_irq     = omap3_init_irq,
	.init_machine = pia35x_init,
	.timer        = &omap3_timer,
MACHINE_END
