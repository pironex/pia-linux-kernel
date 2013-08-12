/*
 * CPU idle for AM33XX SoCs
 *
 * Copyright (C) 2011 Texas Instruments Incorporated. http://www.ti.com/
 *
 * Derived from Davinci CPU idle code
 * (arch/arm/mach-davinci/cpuidle.c)
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <linux/sched.h>
#include <asm/proc-fns.h>

#include <plat/emif.h>

#include "pm.h"
#include "cpuidle33xx.h"

#define AM33XX_CPUIDLE_MAX_STATES	2

static struct cpuidle_driver am33xx_idle_driver = {
	.name	= "cpuidle-am33xx",
	.owner	= THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device, am33xx_cpuidle_device);

/* Actual code that puts the SoC in different idle states */
static int am33xx_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	struct timeval before, after;
	int idle_time, ret = 0;

	if (index == 0) {
		local_irq_disable();
		do_gettimeofday(&before);

		/* Wait for interrupt state */
		cpu_do_idle();
	} else {
		ret = am33xx_setup_cpuidle();

		local_irq_disable();
		do_gettimeofday(&before);
		if (!ret)
			am33xx_enter_cpuidle();
	}

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;

	return index;
}

static int __init am33xx_cpuidle_probe(struct platform_device *pdev)
{
	int ret;
	struct cpuidle_device *device;
	struct cpuidle_driver *driver = &am33xx_idle_driver;
	struct am33xx_cpuidle_config *pdata = pdev->dev.platform_data;

	device = &per_cpu(am33xx_cpuidle_device, smp_processor_id());

	if (!pdata) {
		dev_err(&pdev->dev, "cannot get platform data\n");
		return -ENOENT;
	}

	/* Wait for interrupt state */
	driver->states[0].enter = am33xx_enter_idle;
	driver->states[0].exit_latency = 1;
	driver->states[0].target_residency = 10000;
	driver->states[0].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(driver->states[0].name, "WFI");
	strcpy(driver->states[0].desc, "Wait for interrupt");

	/* Wait for interrupt and DDR self refresh state */
	driver->states[1].enter = am33xx_enter_idle;
	driver->states[1].exit_latency = 950 + 150;
	driver->states[1].target_residency = 10000;
	driver->states[1].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(driver->states[1].name, "DDR SR");
	strcpy(driver->states[1].desc, "WFI and MPU Clock Off");

	device->state_count = AM33XX_CPUIDLE_MAX_STATES;
	driver->state_count = AM33XX_CPUIDLE_MAX_STATES;

	ret = cpuidle_register_driver(&am33xx_idle_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed to register driver\n");
		return ret;
	}

	ret = cpuidle_register_device(device);
	if (ret) {
		dev_err(&pdev->dev, "failed to register device\n");
		cpuidle_unregister_driver(&am33xx_idle_driver);
		return ret;
	}

	return 0;
}

static struct platform_driver am33xx_cpuidle_driver = {
	.driver = {
		.name	= "cpuidle-am33xx",
		.owner	= THIS_MODULE,
	},
};

static int __init am33xx_cpuidle_init(void)
{
	return platform_driver_probe(&am33xx_cpuidle_driver,
						am33xx_cpuidle_probe);
}
device_initcall(am33xx_cpuidle_init);
