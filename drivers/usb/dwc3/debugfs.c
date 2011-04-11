/**
 * debugfs.c - DesignWare USB3 DRD Controller DebugFS file
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/ptrace.h>
#include <linux/types.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

static int dwc3_regdump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	file->f_pos = 0;

	return 0;
}

static loff_t dwc3_regdump_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t			ret;

	if (offset < 0)
		return -EINVAL;

	mutex_lock(&file->f_path.dentry->d_inode->i_mutex);
	switch (orig) {
	case SEEK_CUR:
		offset += file->f_pos;
	case SEEK_SET:
		file->f_pos = offset;
		ret = file->f_pos;
		force_successful_syscall_return();
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&file->f_path.dentry->d_inode->i_mutex);

	return ret;
}

static ssize_t dwc3_regdump_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	struct dwc3		*dwc = file->private_data;
	size_t			size;

	count = min(count, dwc->regs_size);
	size = count;

	while (size > 0) {
		unsigned long	remaining;

		remaining = copy_to_user(buf, dwc->regs + file->f_pos, count);
		if (remaining == 0)
			break;

		buf += size - remaining;
		size -= remaining;
	}

	return count;
}

static const struct file_operations dwc3_regdump_fops = {
	.open			= dwc3_regdump_open,
	.read			= dwc3_regdump_read,
	.llseek			= dwc3_regdump_lseek,
};

int __devinit dwc3_debugfs_init(struct dwc3 *dwc)
{
	struct dentry		*root;
	struct dentry		*file;
	int			ret;

	root = debugfs_create_dir(dev_name(dwc->dev), NULL);
	if (IS_ERR(root)){
		ret = PTR_ERR(root);
		goto err0;
	}

	dwc->root = root;

	file = debugfs_create_file("regdump", S_IRUGO, root, dwc,
			&dwc3_regdump_fops);
	if (IS_ERR(file)) {
		ret = PTR_ERR(file);
		goto err1;
	}

	return 0;

err1:
	debugfs_remove_recursive(root);

err0:
	return ret;
}

void __devexit dwc3_debugfs_exit(struct dwc3 *dwc)
{
	debugfs_remove_recursive(dwc->root);
	dwc->root = NULL;
}
