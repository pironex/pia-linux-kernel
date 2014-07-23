/*
 * linux/drivers/video/st7586s.c
 *
 * ST7586 framebuffer driver for SPI.
 *
 * Copyright (C) 2014 Mikhail Burakov <m-burakov@ya.ru>
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
#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

/* Define DEBUG if you want traces of all the basic functions */
/* Define CORRECTION if you want to verify the data received from userspace */

#ifdef DEBUG
#define TRACE() \
	printk(KERN_INFO DRVNAME "->%s (%s:%i)\n", __FUNCTION__, __FILE__, __LINE__)
#else
#define TRACE()
#endif

/*
 * Line length here represents length of the line in bytes in accordance with memory layout:
 * the first byte is data flag - always 0x01,
 * the second byte contains values for three pixels - aaabbbcc
 */
#define DRVNAME "st7586s"
#define HEIGHT 160
#define LINE_LENGTH 160
#define WIDTH 240

static struct fb_fix_screeninfo st7586s_fix __devinitdata = {
	.id = "ST7586S",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_PSEUDOCOLOR,
	.line_length = LINE_LENGTH,
	.accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo st7586s_var __devinitdata = {
	.xres = WIDTH,
	.yres = HEIGHT,
	.xres_virtual = WIDTH,
	.yres_virtual = HEIGHT,
	.bits_per_pixel = 2,
	.grayscale = 1,
	.nonstd = 1,
};

#define DATA(arg) ((arg & 0xff) | 0x0100)
#define SPI_FLUSH(spi, buf) spi_write(spi, buf, sizeof(buf))

static int st7586s_prepare_transmission(struct spi_device* spi,
	int x1, int y1, int x2, int y2)
{
	unsigned short buf[] = {
		/* Horizontal window */
		0b000101010, DATA(x1 >> 8), DATA(x1), DATA(x2 >> 8), DATA(x2),
		/* Vertical window   */
		0b000101011, DATA(y1 >> 8), DATA(y1), DATA(y2 >> 8), DATA(y2),
		/* Data transmission */
		0b000101100,
	};

	return SPI_FLUSH(spi, buf);
}

static int st7586s_configure(struct spi_device* spi)
{
	unsigned short buf[] = {
		/* Sleep out mode         */
		0b000010001,
		/* Set VOP                */
		0b011000000, 0b111111111, 0b100000000,
		/* BIAS system            */
		0b011000011, 0b100000011,
		/* Display mode gray      */
		0b000111000,
		/* Inverse pixels         */
		0b000100001,
		/* Enable DDRAM interface */
		0b000111010, 0b100000010,
		/* Display ON             */
		0b000101001,
	};

	return SPI_FLUSH(spi, buf);
}

static int st7586s_violates_boundaries(int x, int y, int w, int h)
{
	return x > WIDTH || x + w < 0 || y > HEIGHT || y + h < 0;
}

void st7586s_deferred_io(struct fb_info* info, struct list_head* pagelist)
{
	int retval;
	TRACE();

	retval = st7586s_prepare_transmission(info->par, 0, 0, (WIDTH / 3) - 1, HEIGHT - 1);
	if (retval) {
		pr_err(DRVNAME ": Failed to prepare display update (%i)\n",
			retval);
		return;
	}

#ifdef CORRECTION
	unsigned int* fb = info->screen_base, i;
	/* Make sure that everything in buffer have valid SPI data type */
	for (i = 0; i < LINE_LENGTH * HEIGHT >> 2; ++i) fb[i] |= 0x01000100;
#endif

	spi_write(info->par, info->screen_base, LINE_LENGTH * HEIGHT);
}

void st7586s_fillrect(struct fb_info* info, const struct fb_fillrect* rect)
{
	int start_x, end_x, x, y, retval;
	int fill_value, start_mask, end_mask;
	unsigned short* dst_line;
	TRACE();

	/* Discard all writes not fitting to the display */
	if (st7586s_violates_boundaries(rect->dx,rect->dy,
			rect->width, rect->height))
		return;

	x = rect->dx + rect->width;
	start_x = rect->dx / 3;
	end_x = x / 3 + !(x % 3);

	/* Prepare for data transmission */
	retval = st7586s_prepare_transmission(info->par,
		start_x, rect->dy, end_x, rect->dy + rect->height - 1);
	if (retval) {
		pr_err(DRVNAME ": Failed to prepare for data transmission (%i)\n",
			retval);
		return;
	}

	/* Prepare filling pattern */
	fill_value = ((rect->color & rect->color >> 1 & 1) | rect->color << 1) & 0b111;
	fill_value = fill_value << 5 | fill_value << 2 | fill_value >> 1;

	/* Masks for the first and last bytes in filled line area */
	start_mask = 0b111 << (2 - rect->dx % 3) * 3 >> 1;
	end_mask = 0b111 << (2 - x % 3) * 3 >> 1;

	for (y = 0; y < rect->height; ++y) {
		/* Target line in framebuffer */
		dst_line = (unsigned short*)(info->screen_base) + (rect->dy + y) * 80;

		/* Set first and last bytes of filled line area */
		dst_line[start_x] = (dst_line[start_x] & ~start_mask) | (fill_value & start_mask) | 0x100;
		dst_line[end_x] = (dst_line[end_x] & ~end_mask) | (fill_value & end_mask) | 0x100;

		/* Fill everything between with prepared pattern */
		for (x = start_x + 1; x < end_x; ++x)
			dst_line[x] = fill_value | 0x100;

		/* Flush line, twice the number of bytes to include data flag bytes */
		spi_write(info->par, dst_line + start_x, (end_x - start_x + 1) << 1);
	}
}

void st7586s_copyarea(struct fb_info* info, const struct fb_copyarea* area)
{
	TRACE();

	/* Discard all writes not fitting to the display */
	if (st7586s_violates_boundaries(area->dx, area->dy,
			area->width, area->height))
		return;

	pr_info(DRVNAME ": Area copying is not implemented yet\n");
}

void st7586s_imageblit(struct fb_info* info, const struct fb_image* image)
{
	int start_x, end_x, x, y, retval;
	int source_shift, target_shift;
	int source, target, color, mask;
	unsigned char* src_line;
	unsigned short* dst_line;
	TRACE();

	/* Discard all writes not fitting to the display */
	if (st7586s_violates_boundaries(image->dx, image->dy,
		image->width, image->height))
		return;

	/* Refuse to blit non-monochrome images */
	if (image->depth != 1) {
		pr_info(DRVNAME ": Refusing to blit non-monochrome image\n");
		return;
	}

	x = image->dx + image->width;
	start_x = image->dx / 3;
	end_x = x / 3 + !(x % 3);

	/* Prepare for data transmission */
	retval = st7586s_prepare_transmission(info->par,
		start_x, image->dy, end_x, image->dy + image->height - 1);
	if (retval) {
		pr_err(DRVNAME ": Failed to prepare for data transmission (%i)\n",
			retval);
		return;
	}

	for (y = 0; y < image->height; ++y) {
		/* Source line from image and destination line in framebuffer */
		dst_line = (unsigned short*)(info->screen_base) + (image->dy + y) * 80;
		src_line = (unsigned char*)(image->data) + y * (image->width >> 3);

		for (x = 0; x < image->width; ++x) {
			source_shift = 7 - (x & 7);
			source = x >> 3;
			target_shift = (2 - (image->dx + x) % 3) * 3;
			target = (image->dx + x) / 3;
			color = src_line[source] >> source_shift & 1;
			mask = 0b111 << target_shift >> 1;
			dst_line[target] = (dst_line[target] & ~mask) | (color * mask) | 0x100;
		}
		spi_write(info->par, dst_line + start_x, (end_x - start_x + 1) << 1);
	}
}

static struct fb_ops st7586s_ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = st7586s_fillrect,
	.fb_copyarea = st7586s_copyarea,
	.fb_imageblit = st7586s_imageblit,
};

static struct fb_deferred_io st7586s_defio = {
	.delay = HZ / 20,
	.deferred_io = st7586s_deferred_io,
};

/******************************************************************************/

static int st7586s_probe(struct spi_device* spi)
{
	int retval;
	int vmem_size = WIDTH * HEIGHT;
	struct fb_info* info = NULL;
	u8* vmem = NULL;
	TRACE();

	spi->bits_per_word = 9;
	retval = spi_setup(spi);
	if (retval) {
		pr_err(DRVNAME ": Failed to switch to 9-bit words\n");
		goto failure;
	}

	retval = st7586s_configure(spi);
	if (retval) {
		pr_err(DRVNAME ": Initial chip configuration failed\n");
		goto failure;
	}

	/* Prepare error for memory allocating functions */
	retval = -ENOMEM;

	if ((vmem = (u8*)kmalloc(vmem_size, GFP_KERNEL)) == NULL) {
		pr_err(DRVNAME ": Failed to allocate memory for framebuffer\n");
		goto failure;
	}

	if ((info = framebuffer_alloc(0, &spi->dev)) == NULL) {
		pr_err(DRVNAME ": Failed to allocate framebuffer\n");
		goto failure;
	}

	info->screen_base = vmem;
	info->fbops = &st7586s_ops;
	info->fix = st7586s_fix;
	info->fix.smem_start = virt_to_phys(vmem);
	info->fix.smem_len = vmem_size;
	info->var = st7586s_var;
	info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;
	info->fbdefio = &st7586s_defio;
	info->par = spi;
	fb_deferred_io_init(info);

	retval = register_framebuffer(info);
	if (retval) {
		pr_err(DRVNAME ": Failed to register framebuffer\n");
		goto failure;
	}

	spi_set_drvdata(spi, info);
	return 0;

failure:
	if (vmem)
		kfree(vmem);
	if (info)
		framebuffer_release(info);
	return retval;
}

static int st7586s_remove(struct spi_device* spi)
{
	struct fb_info* info;
	TRACE();

	if ((info = spi_get_drvdata(spi)) != NULL) {
		unregister_framebuffer(info);
		framebuffer_release(info);
		kfree(info->screen_base);
	}

	spi_set_drvdata(spi, NULL);
	return 0;
}

static struct spi_driver st7586s_driver = {
	.driver = {
		.name = "st7586s",
	},
	.probe = st7586s_probe,
	.remove = st7586s_remove,
};

static int __init st7586s_init(void)
{
	TRACE();
	return spi_register_driver(&st7586s_driver);
}

static void __exit st7586s_exit(void)
{
	TRACE();
	spi_unregister_driver(&st7586s_driver);
}

module_init(st7586s_init);
module_exit(st7586s_exit);

MODULE_AUTHOR("Mikhail Burakov <mikhail.burakov@gmail.com>");
MODULE_DESCRIPTION("SM75867S framebuffer driver");
MODULE_LICENSE("GPL");
