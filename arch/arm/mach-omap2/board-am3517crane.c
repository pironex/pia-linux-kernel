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
#include <linux/clk.h>
#include <linux/davinci_emac.h>
#include <linux/gpio.h>
#include <linux/mfd/tps6507x.h>
#include <linux/mmc/host.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/tps6507x.h>
#include <linux/can/platform/ti_hecc.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"

/* Board initialization */
static struct omap_board_config_kernel am3517_crane_config[] __initdata = {
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
		/* only fixed MUXes here, don't add anything on expansions */

		/* MMC1_CD        GPIO 041, low == card in slot */
		OMAP3_MUX(GPMC_A8, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
		/* GSM_nRESET     GPIO 126, low active */
		OMAP3_MUX(SDMMC1_DAT4, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
		/* nGSM_ON/OFF    GPIO 127, low active */
		OMAP3_MUX(SDMMC1_DAT5, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
		/* UART2.485/#232 GPIO 128, low = RS232 */
		OMAP3_MUX(SDMMC1_DAT6, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
		/* UART2.SLEW     GPIO 129 */
		OMAP3_MUX(SDMMC1_DAT7, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),
		/* undocumented */
		//OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
		/* MCBSP_CLKS     GPIO 160 */
		//OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
		/* INPUT_GPIO1    GPIO 055 */
		OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
		/* INPUT_GPIO2    GPIO 056 */
		OMAP3_MUX(GPMC_NCS5, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
		/* ETHERNET_nRST  GPIO 065 */
		OMAP3_MUX(GPMC_WAIT3, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

		/* TERMINATOR */
		{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

#if 0 /* USB EHCI port only on expansion port */
static struct usbhs_omap_board_data usbhs_bdata __initdata = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};
#endif

/*
 * Ethernet (internal) & PHY (SMSC LAN8720A-CP)
 */
#define GPIO_ETHERNET_NRST 65
#define AM35XX_EVM_MDIO_FREQUENCY	(1000000)

static struct mdio_platform_data pia35x_evm_mdio_pdata = {
	.bus_freq	= AM35XX_EVM_MDIO_FREQUENCY,
};

static struct resource pia35x_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device pia35x_mdio_device = {
	.name		= "davinci_mdio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(pia35x_mdio_resources),
	.resource	= pia35x_mdio_resources,
	.dev.platform_data = &pia35x_evm_mdio_pdata,
};

static struct emac_platform_data pia35x_emac_pdata = {
	.rmii_en	= 1,
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

	/* unset reset */
	gpio_request(GPIO_ETHERNET_NRST, "ethernet-nrst");
	gpio_direction_output(GPIO_ETHERNET_NRST, 1);
	msleep(50);

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

	return ;
}

/*
 * CAN - HECC
 */
//#define CAN_STB         214
//static void hecc_phy_control(int on)
//{
//        int r;
//
//        r = gpio_request(CAN_STB, "can_stb");
//        if (r) {
//                printk(KERN_ERR "failed to get can_stb \n");
//                return;
//        }
//
//        gpio_direction_output(CAN_STB, (on==1)?0:1);
//}

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

static void __init pia35x_can_init(struct ti_hecc_platform_data *pdata)
{
	pia35x_hecc_device.dev.platform_data = pdata;
	platform_device_register(&pia35x_hecc_device);
}



/*
 * MMC
 */
#include <linux/regulator/fixed.h>
#include <linux/regulator/tps6507x.h>

/* MMC1 has fixed power supply */
static struct regulator_consumer_supply pia35x_vmmc1_consumers[] = {
		REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.0"),
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
#endif /* CONFIG_REGULATOR_TPS6507X */

/* register our voltage regulator TPS650732 using I2C1 */
static int __init pia35x_pmic_tps65070_init(void)
{
	// does nothing, already registered with i2c inits
	return 0;
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
		.gpio_wp        = -EINVAL, /* we don't have a WP pin connected, was: 40 */
		//.ocr_mask       = MMC_VDD_33_34,
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

static void __init pia35x_mmc_init(void)
{
	pr_info("piA-am35x: registering VMMC1 platform device\n");
	/* handling of different MMC2 expansions here */
	omap2_hsmmc_init(mmc);
	/* link regulator to on-board MMC adapter */
	pia35x_vmmc1_consumers[0].dev = mmc[0].dev;
	platform_device_register(&pia35x_vmmc1_device);
}

/*
 * MUSB (USB OTG)
 */
static struct omap_musb_board_data pia35x_musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 500,
};

static __init void pia35x_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&pia35x_musb_board_data);
}

/*
 * NAND
 * we use GPMC CS 0
 */
#define PIA35X_NAND_CS 0
static struct mtd_partition pia35x_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader-nand",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 28*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "params-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 4*(SZ_128K)
	},
	{
		.name           = "linux-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 80*(SZ_128K)
	},
	{
		.name           = "jffs2-nand",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};

#if 0
static struct omap_nand_platform_data am3517crane_nand_data = {
	.parts          = pia35x_nand_partitions,
	.nr_parts       = ARRAY_SIZE(pia35x_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	.dev_ready      = NULL,
};

static struct resource am3517crane_nand_resource = {
	.flags          = IORESOURCE_MEM,
};

static struct platform_device am3517crane_nand_device = {
	.name           = "omap2-nand",
	.id             = 0,
	.dev            = {
		.platform_data  = &am3517crane_nand_data,
	},
	.num_resources  = 1,
	.resource       = &am3517crane_nand_resource,
};
#endif /* 0 */

void __init pia35x_flash_init(void)
{
#if 0 /* we don't need this in 2.6.37 */
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}
	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				" in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		board_nand_init(pia35x_nand_partitions,
				ARRAY_SIZE(pia35x_nand_partitions), nandcs, NAND_BUSWIDTH_16);
//		am3517crane_nand_data.cs   = nandcs;
//		am3517crane_nand_data.gpmc_cs_baseaddr =
//		(void *)(gpmc_base_add + GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
//
//		am3517crane_nand_data.gpmc_baseaddr = (void *)(gpmc_base_add);
//
//		if (platform_device_register(&am3517crane_nand_device) < 0)
//			printk(KERN_ERR "Unable to register NAND device\n");

	}
#endif /* 0 - old nand detection */
	board_nand_init(pia35x_nand_partitions,
			ARRAY_SIZE(pia35x_nand_partitions),
			PIA35X_NAND_CS, NAND_BUSWIDTH_16);
}

/*
 * I2C
 */
static struct i2c_board_info __initdata pia35x_i2c1_info[] = {
#if defined(CONFIG_REGULATOR_TPS6507X)
		/* power regulator TPS650732 */
		{
				I2C_BOARD_INFO("tps6507x", 0x48),
				.platform_data = &pia35x_tps_board,
		},
#endif /* CONFIG_REGULATOR_TPS6507X */
};

static struct i2c_board_info __initdata pia35x_i2c2_info[] = {
		/* temperature sensor LM75 */
		{
				I2C_BOARD_INFO("lm75", 0x48),
		},
};

static struct i2c_board_info __initdata pia35x_i2c3_info[] = {
};

static int __init pia35x_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, pia35x_i2c1_info, ARRAY_SIZE(pia35x_i2c1_info));
	omap_register_i2c_bus(2, 400, pia35x_i2c2_info, ARRAY_SIZE(pia35x_i2c2_info));
	omap_register_i2c_bus(3, 400, pia35x_i2c3_info, ARRAY_SIZE(pia35x_i2c3_info));

	return 0;
}

static void __init am3517_crane_init_irq(void)
{
	omap_board_config = am3517_crane_config;
	omap_board_config_size = ARRAY_SIZE(am3517_crane_config);

	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

/*
 * base initialisation function
 */
static void __init am3517_crane_init(void)
{
	int ret;

	pr_info("pia35x_init: init pin muc\n");
	ret = omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	if (ret)
		pr_warning("pia35x_init: MUX init failed: %d\n", ret);

	pr_info("pia35x_init: init I2C busses\n");
	pia35x_i2c_init();

	pr_info("pia35x_init: init serial ports\n");
	omap_serial_init();
	omap_sdrc_init(NULL, NULL);

	omap_board_config = am3517_crane_config;
	omap_board_config_size = ARRAY_SIZE(am3517_crane_config);

	pr_info("pia35x_init: init NAND\n");
	pia35x_flash_init();

	pr_info("pia35x_init: init USB OTG\n");
	pia35x_musb_init();

	pr_info("pia35x_init: init ETH\n");
	pia35x_ethernet_init(&pia35x_emac_pdata);
	pr_info("pia35x_init: init CAN\n");
	pia35x_can_init(&pia35x_hecc_pdata);

	pr_info("pia35x_init: init MMC\n");
	pia35x_mmc_init();

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
