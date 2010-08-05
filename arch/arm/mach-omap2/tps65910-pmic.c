/*
 * tps65910-pmic.c
 *
 * Common regulator supplies and init data structs for TPS65910
 * PMIC for AM35xx based EVMs. They can be used in various board-evm
 * files for Am35xx based platforms using TPS65910.
 *
 * Copyright (C) 2010 Mistral Solutions Pvt Ltd <www.mistralsolutions.com>
 *
 * Based on arch/arm/mach-omap2/twl4030-pmic.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

/* Power domain maping for TPS65910 and AM35XX

	   1.8V
	   VIO -----------> VDDS

	   1.8V
	   VAUX1 ---------> VDDS_SRAM_CORE_BG
				|
				-------> VDDS_SRAM_MPU
				|
				-------> VDDOSC

	   3.3V
	   VDD2 ----------> VDDSHV

	   1.2V
	   VDD1 ----------> VDD_CORE

	   1.8V
	   VPLL ----------> VDDS_DPLL_PRE_CORE
				|
				-------> VDDSPLL_MPU_USBHOST

	   1.8V
	   VDAC ----------> VDDA_DAC

	   1.8V
	   VAUX2 ----------> VDDA1P8V_USBPHY

	   3.3V
	   VMMC ----------> VDDA3P3V_USBPHY

*/
#include <linux/regulator/machine.h>

/* VIO */
struct regulator_consumer_supply tps65910_vio_supply = {
	.supply = "vdds",
};


/* VAUX1 */
struct regulator_consumer_supply tps65910_vaux1_supply[] = {
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
	{
		.supply = "vddosc",
	},
};

/* VPLL */
struct regulator_consumer_supply tps65910_vpll_supply[] = {
	{
		.supply = "vdds_dpll_pre_core",
	},
	{
		.supply = "vddspll_mpu_usbhost",
	},

};

/* VDAC */
struct regulator_consumer_supply tps65910_vdac_supply = {
	.supply = "vdda_dac",
};

/* VAUX2 */
struct regulator_consumer_supply tps65910_vaux2_supply = {
	.supply = "vdda1p8v_usbphy",
};


/* VMMC */
struct regulator_consumer_supply tps65910_vmmc_supply = {
	.supply = "vdda3p3v_usbphy",
};


/* Regulator initialization data */

/* VIO  LDO */
struct regulator_init_data vio_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 1800000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tps65910_vaux1_supply),
	.consumer_supplies      = &tps65910_vaux1_supply,
};



/* VAUX1  LDO */
struct regulator_init_data vaux1_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 2850000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = false,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tps65910_vaux1_supply),
	.consumer_supplies      = &tps65910_vaux1_supply,
};

/* VAUX2  LDO */
struct regulator_init_data vaux2_data = {
	.constraints = {
		.min_uV = 3300000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = true,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tps65910_vaux2_supply),
	.consumer_supplies      = &tps65910_vaux2_supply,

};

/* VMMC  LDO */
struct regulator_init_data vmmc_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 3300000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = false,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tps65910_vmmc_supply),
	.consumer_supplies      = &tps65910_vmmc_supply,

};

/* VPLL  LDO */
struct regulator_init_data vpll_data = {
	.constraints = {
		.min_uV = 100000,
		.max_uV = 2500000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = false,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tps65910_vpll_supply),
	.consumer_supplies      = &tps65910_vpll_supply,
};

/* VDAC  LDO */
struct regulator_init_data vdac_data = {
	.constraints = {
		.min_uV = 1800000,
		.max_uV = 2850000,
		.valid_modes_mask = REGULATOR_MODE_NORMAL
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.always_on = true,
		.apply_uV = false,
	},
	.num_consumer_supplies  = ARRAY_SIZE(tps65910_vdac_supply),
	.consumer_supplies      = &tps65910_vdac_supply,

};


