/*
 * linux/arch/arm/mach-omap2/board-am3517crane.c
 *
 * Copyright (C) 2010 Mistral Solutions Pvt LtD <www.mistralsolutions.com>
 * Author: Srinath.R <srinath@mistralsolutions.com>
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
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/i2c/tsc2004.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/tca6416_keypad.h>
#include <linux/davinci_emac.h>
#include <linux/i2c/pca953x.h>
#include <linux/regulator/machine.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/i2c/tps65910.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include "mmc-am3517crane.h"
#include "mux.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

static struct mtd_partition am3517crane_nand_partitions[] = {
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

static struct omap_nand_platform_data am3517crane_nand_data = {
	.parts          = am3517crane_nand_partitions,
	.nr_parts       = ARRAY_SIZE(am3517crane_nand_partitions),
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

void __init am3517crane_flash_init(void)
{
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
		am3517crane_nand_data.cs   = nandcs;
		am3517crane_nand_data.gpmc_cs_baseaddr =
		(void *)(gpmc_base_add + GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);

		am3517crane_nand_data.gpmc_baseaddr = (void *)(gpmc_base_add);

		if (platform_device_register(&am3517crane_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");

	}
}


#define AM35XX_EVM_PHY_MASK		(0xF)
#define AM35XX_EVM_MDIO_FREQUENCY    	(1000000)

static struct emac_platform_data am3517_crane_emac_pdata = {
	.phy_mask       = AM35XX_EVM_PHY_MASK,
	.mdio_max_freq  = AM35XX_EVM_MDIO_FREQUENCY,
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str)
{
	int i;

	if (str == NULL)
		return 0;
	for (i = 0; i < ETH_ALEN; i++)
		am3517_crane_emac_pdata.mac_addr[i] = simple_strtol(&str[i*3],
							(char **)NULL, 16);
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
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

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
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

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void am3517_crane_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

	pdata->ctrl_reg_offset          = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset      = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset          = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset          = AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size            = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version                  = EMAC_VERSION_2;
	pdata->hw_ram_addr              = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable 	= am3517_enable_ethernet_int;
	pdata->interrupt_disable 	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data     = pdata;
	platform_device_register(&am3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}

static void __init am3517_crane_display_init(void)
{
	omap_mux_init_gpio(52, OMAP_PIN_OUTPUT);
	gpio_request(52, "dvi_enable");
	gpio_direction_output(52, 1);
}



static struct omap_dss_device am3517_crane_tv_device = {
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.name 			= "tv",
	.driver_name		= "venc",
	.phy.venc.type          = OMAP_DSS_VENC_TYPE_COMPOSITE,
	.platform_enable	= NULL,
	.platform_disable	= NULL,
};

static int am3517_crane_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(52, 1);
	return 0;
}

static void am3517_crane_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(52, 0);
}

static struct omap_dss_device am3517_crane_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= am3517_crane_panel_enable_dvi,
	.platform_disable	= am3517_crane_panel_disable_dvi,
};

static struct omap_dss_device *am3517_crane_dss_devices[] = {
	&am3517_crane_tv_device,
	&am3517_crane_dvi_device,
};

static struct omap_dss_board_info am3517_crane_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_crane_dss_devices),
	.devices	= am3517_crane_dss_devices,
	.default_device	= &am3517_crane_dvi_device,
};

struct platform_device am3517_crane_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_crane_dss_data,
	},
};

static u64 vpfe_capture_dma_mask = DMA_BIT_MASK(32);
static struct resource dm644x_ccdc_resource[] = {
	/* CCDC Base address */
	{
		.start	= AM35XX_IPSS_VPFE_BASE,
		.end	= AM35XX_IPSS_VPFE_BASE + 0xffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device dm644x_ccdc_dev = {
	.name		= "dm644x_ccdc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm644x_ccdc_resource),
	.resource	= dm644x_ccdc_resource,
	.dev = {
		.dma_mask		= &vpfe_capture_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

static struct regulator_consumer_supply am3517_crane_vdd1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

static struct regulator_init_data am3517_crane_regulator_vdd1 = {
	.constraints = {
		.min_uV = 1200000,
		.max_uV = 1200000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vdd1_supplies),
	.consumer_supplies = am3517_crane_vdd1_supplies,
};

static struct regulator_consumer_supply am3517_crane_vdd2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

static struct regulator_init_data am3517_crane_regulator_vdd2 = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vdd2_supplies),
	.consumer_supplies = am3517_crane_vdd2_supplies,
};


static struct regulator_consumer_supply am3517_crane_vio_supplies[] = {
	{
		.supply = "vdds",
	},
};

static struct regulator_init_data am3517_crane_regulator_vio = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vio_supplies),
	.consumer_supplies = am3517_crane_vio_supplies,
};


static struct regulator_consumer_supply am3517_crane_vaux1_supplies[] = {
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

static struct regulator_init_data am3517_crane_regulator_vaux1 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vaux1_supplies),
	.consumer_supplies = am3517_crane_vaux1_supplies,
};


static struct regulator_consumer_supply am3517_crane_vaux2_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
};

static struct regulator_init_data am3517_crane_regulator_vaux2 = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vaux2_supplies),
	.consumer_supplies = am3517_crane_vaux2_supplies,
};


static struct regulator_consumer_supply am3517_crane_vdac_supplies[] = {
	{
		.supply = "vdda_dac",
		.dev    = &am3517_crane_dss_device.dev,
	},
};

static struct regulator_init_data am3517_crane_regulator_vdac = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_MODE,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vdac_supplies),
	.consumer_supplies = am3517_crane_vdac_supplies,
};

static struct regulator_consumer_supply am3517_crane_vmmc_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data am3517_crane_regulator_vmmc = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vmmc_supplies),
	.consumer_supplies = am3517_crane_vmmc_supplies,
};


static struct regulator_consumer_supply am3517_crane_vpll_supplies[] = {
	{
		.supply = "vdds_dpll_mpu_usbhost",
	},
	{
		.supply = "vdds_dpll_per_core",
	},
};

static struct regulator_init_data am3517_crane_regulator_vpll = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies = ARRAY_SIZE(am3517_crane_vpll_supplies),
	.consumer_supplies = am3517_crane_vpll_supplies,
};

static int am3517_crane_tps65910_config(struct tps65910_platform_data *pdata)
{
	u8 val 	= 0;
	int i 	= 0;
	int err = -1;


	/* Configure TPS65910 for am3517_crane board needs */

	/* Set sleep state active high */
	val |= (TPS65910_DEV2_SLEEPSIG_POL);

	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
			TPS65910_REG_DEVCTRL2);
	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_DEVCTRL2 reg\n");
		return -EIO;
	}

	/* Mask ALL interrupts */
	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, 0xFF,
			TPS65910_REG_INT_MSK);
	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_INT_MSK reg\n");
		return -EIO;
	}
	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, 0x03,
			TPS65910_REG_INT_MSK2);
	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_INT_MSK2 reg\n");
		return -EIO;
	}

	/* Set RTC regulator on during sleep */

	val = TPS65910_VRTC_OFFMASK;
	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
			TPS65910_REG_VRTC);

	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_VRTC reg\n");
		return -EIO;
	}
	/* Set RTC Power, disable Smart Reflex in DEVCTRL_REG */
	val = 0;
	val &= ~TPS65910_RTC_PWDNN;
	val |= (TPS65910_CK32K_CTRL | TPS65910_SR_CTL_I2C_SEL);

	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
			TPS65910_REG_DEVCTRL);
	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_DEVCTRL reg\n");
		return -EIO;
	}

	/* Enable and set back-up battery charger control*/

	tps65910_enable_bbch(TPS65910_BBSEL_2P52);

	err = tps65910_i2c_read_u8(TPS65910_I2C_ID0, &val,
			TPS65910_REG_VRTC);
	if (err) {
		printk(KERN_ERR "Unable to read  TPS65910_REG_VRTC reg\n");
		return -EIO;
	}
	val = TPS65910_VRTC_OFFMASK;

	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
			TPS65910_REG_VRTC);
	if (err) {
		printk(KERN_ERR "Unable to write TPS65910_REG_VRTC reg\n");
		return -EIO;
	}

	/* Disable SmartReflex control */
	val &= 0;
	val &= ~TPS65910_RTC_PWDNN;
	val |= (TPS65910_CK32K_CTRL | TPS65910_SR_CTL_I2C_SEL);

	err = tps65910_i2c_write_u8(TPS65910_I2C_ID0, val,
			TPS65910_REG_DEVCTRL);
	if (err) {
		printk(KERN_ERR "Unabale to write TPS65910_REG_DEVCTRL reg\n");
		return -EIO;
	}

	/* initilize all ISR work as NULL, specific driver will
	 * assign function(s) later.
	 */
	for (i = 0; i < TPS65910_MAX_IRQS; i++)
		pdata->handlers[i] = NULL;

	return 0;
}

struct tps65910_platform_data am3517_crane_tps65910_data = {
	.irq_num 	= (unsigned)TPS65910_HOST_IRQ,
	.gpio  		= NULL,
	.vio   		= &am3517_crane_regulator_vio,
	.vdd1  		= &am3517_crane_regulator_vdd1,
	.vdd2  		= &am3517_crane_regulator_vdd2,
	.vdd3  		= NULL,
	.vdig1		= NULL,
	.vdig2		= NULL,
	.vaux33		= NULL,
	.vmmc		= &am3517_crane_regulator_vmmc,
	.vaux1		= &am3517_crane_regulator_vaux1,
	.vaux2		= &am3517_crane_regulator_vaux2,
	.vdac		= &am3517_crane_regulator_vdac,
	.vpll		= &am3517_crane_regulator_vpll,
	.board_tps65910_config = am3517_crane_tps65910_config,
};

static struct i2c_board_info __initdata am3517crane_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID0),
		.flags          = I2C_CLIENT_WAKE,
		.irq            = TPS65910_HOST_IRQ,
		.platform_data  = &am3517_crane_tps65910_data,
	},
};


static int __init am3517_crane_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, am3517crane_i2c1_boardinfo,
			ARRAY_SIZE(am3517crane_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}

/*
 * HECC information
 */
static struct resource am3517_hecc_resources[] = {
	{
		.start  = AM35XX_IPSS_HECC_BASE,
		.end    = AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_HECC0_IRQ,
		.end    = INT_35XX_HECC0_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_hecc_device = {
	.name           = "ti_hecc",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(am3517_hecc_resources),
	.resource       = am3517_hecc_resources,
};

static struct ti_hecc_platform_data am3517_crane_hecc_pdata = {
	.scc_hecc_offset        = AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset         = AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset        = AM35XX_HECC_RAM_OFFSET,
	.mbx_offset            = AM35XX_HECC_MBOX_OFFSET,
	.int_line               = AM35XX_HECC_INT_LINE,
	.version                = AM35XX_HECC_VERSION,
};

static void am3517_crane_hecc_init(struct ti_hecc_platform_data *pdata)
{
	am3517_hecc_device.dev.platform_data = pdata;
	platform_device_register(&am3517_hecc_device);
}


/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_crane_config[] __initdata = {
};

static struct platform_device *am3517_crane_devices[] __initdata = {
	&dm644x_ccdc_dev,
	&am3517_crane_dss_device,
};

static void __init am3517_crane_init_irq(void)
{
	omap_board_config = am3517_crane_config;
	omap_board_config_size = ARRAY_SIZE(am3517_crane_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = 38,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif

static struct am3517_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.wires          = 8,
		.gpio_cd        = 41,
		.gpio_wp        = 40,
	},
	{}      /* Terminator */
};

static void __init am3517_crane_init(void)
{

	am3517_crane_i2c_init();
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	platform_add_devices(am3517_crane_devices,
			ARRAY_SIZE(am3517_crane_devices));

	omap_serial_init();
	am3517crane_flash_init();
	usb_musb_init();

	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(35, OMAP_PIN_OUTPUT);
	gpio_request(35, "usb_ehci_enable");
	gpio_direction_output(35, 1);
	gpio_set_value(35, 1);
	omap_mux_init_gpio(38, OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);

	/* DSS */
	am3517_crane_display_init();

	/*Ethernet*/
	am3517_crane_ethernet_init(&am3517_crane_emac_pdata);
	am3517_crane_hecc_init(&am3517_crane_hecc_pdata);

	/* MMC init function */
	am3517_mmc_init(mmc);

}

static void __init am3517_crane_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(CRANEBOARD, "AM3517/05 CRANEBOARD")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= am3517_crane_map_io,
	.init_irq	= am3517_crane_init_irq,
	.init_machine	= am3517_crane_init,
	.timer		= &omap_timer,
MACHINE_END
