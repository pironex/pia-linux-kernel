/*
 * linux/drivers/video/st7586s.c
 *
 * Expose OMAP cpuid to userspace via procfs
 *
 * Copyright (C) 2014 Mikhail Burakov <m-burakov@ya.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <mach/id.h>

static struct omap_die_id uuid;

static ssize_t cpuid_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset)
{
	return simple_read_from_buffer(buffer, length, offset, &uuid, sizeof(uuid));
}

static const struct file_operations cpuid_ops = {
	.read = cpuid_read
};

static int __init cpuid_expose_init(void)
{
	omap_get_die_id(&uuid);
	proc_create("cpuid", 0444, NULL, &cpuid_ops);
	return 0;
}

static void __exit cpuid_expose_exit(void)
{
	remove_proc_entry("cpuid", NULL);
}

module_init(cpuid_expose_init);
module_exit(cpuid_expose_exit);

MODULE_AUTHOR("Mikhail Burakov <mikhail.burakov@gmail.com>");
MODULE_DESCRIPTION("cpuid-expose driver");
MODULE_LICENSE("GPL");
