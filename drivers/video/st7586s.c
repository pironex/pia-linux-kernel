#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

/******************************************************************************/

/* Define DEBUG if you want traces of all the basic functions */
/* Defube CORRECTION if you want to verify the data received from userspace */

#ifdef DEBUG
#define TRACE() \
  printk(KERN_INFO DRVNAME "->%s (%s:%i)\n", __FUNCTION__, __FILE__, __LINE__)
#else
#define TRACE()
#endif

/******************************************************************************/

/* Line length here represents length of the line in words, not bytes */
#define DRVNAME "st7586s"
#define HEIGHT 160
#define WIDTH 240

static struct fb_fix_screeninfo st7586s_fix __devinitdata =
{
  .id = "ST7586S",
  .type = FB_TYPE_PACKED_PIXELS,
  .visual = FB_VISUAL_PSEUDOCOLOR,
  .line_length = WIDTH,
  .accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo st7586s_var __devinitdata =
{
  .xres = WIDTH,
  .yres = HEIGHT,
  .xres_virtual = WIDTH,
  .yres_virtual = HEIGHT,
  .bits_per_pixel = 2,
  .grayscale = 1,
  .nonstd = 1,
};

/******************************************************************************/

#define DATA(arg) ((arg & 0xff) | 0x0100)
#define SPI_FLUSH(spi, buf) spi_write(spi, buf, sizeof(buf))

static int st7586s_prepare_transmission(struct spi_device* spi, int x, int y, int w, int h)
{
  int x_end = x + w - 1;
  int y_end = y + h - 1;
  unsigned short buf[] =
  {
    0b000101010, DATA(x >> 8), DATA(x), DATA(x_end >> 8), DATA(x_end), /* Horizontal window */
    0b000101011, DATA(y >> 8), DATA(y), DATA(y_end >> 8), DATA(y_end), /* Vertical window   */
    0b000101100,                                                       /* Data transmission */
  };

  return SPI_FLUSH(spi, buf);
}

static int st7586s_configure(struct spi_device* spi)
{
  unsigned short buf[] =
  {
    0b000010001,                           /* Sleep out mode         */
    0b011000000, 0b100001100, 0b100000001, /* Set VOP                */
    0b011000011, 0b100000011,              /* BIAS system            */
    0b000111000,                           /* Display mode gray      */
    0b000111010, 0b100000010,              /* Enable DDRAM interface */
    0b000101001,                           /* Display ON             */
  };

  return SPI_FLUSH(spi, buf);
}

static int st7586s_violates_boundaries(int x, int y, int w, int h)
{
  return x > WIDTH || x + w < 0 || y > HEIGHT || y + h < 0;
}

/******************************************************************************/

void st7586s_deferred_io(struct fb_info* info, struct list_head* pagelist)
{
  int retval;
  TRACE();

  if ((retval = st7586s_prepare_transmission(info->par, 0, 0, WIDTH >> 1, HEIGHT)) != 0)
  {
    pr_err(DRVNAME ": Failed to prepare display update (%i)\n", retval);
    return;
  }

#ifdef CORRECTION
  int i;
  /* Make sure that everything in buffer have valid SPI data type */
  for (i = 1; i < WIDTH * HEIGHT; i += 2) info->screen_base[i] = 1;
#endif

  spi_write(info->par, info->screen_base, WIDTH * HEIGHT);
}

void st7586s_fillrect(struct fb_info* info, const struct fb_fillrect* rect)
{
  int x, y, retval;
  unsigned int value;
  unsigned int* line;
  TRACE();

  /* Discard all writes not fitting to the display */
  if (st7586s_violates_boundaries(rect->dx, rect->dy, rect->width, rect->height))
    return;

  /* Refuse to fill unaligned area */
  if (rect->dx & 1)
  {
    pr_info(DRVNAME ": Refusing to fill unaligned area\n");
    return;
  }

  /* Refuse to fill areas with size not aligned to 4 pix */
  if (rect->width & 0x3)
  {
    pr_info(DRVNAME ": Refusing to fill unproperly sized area\n");
    return;
  }

  /* Prepare for data transmission */
  if ((retval = st7586s_prepare_transmission(info->par, rect->dx >> 1, rect->dy, rect->width >> 1, rect->height)) != 0)
  {
    pr_err(DRVNAME ": Failed to prepare for data transmission (%i)\n", retval);
    return;
  }

  /* Prepare fill value in advance */
  value =
    ((rect->color & 0b111) <<  5) |
    ((rect->color & 0b111) <<  2) |
    ((rect->color & 0b111) << 21) |
    ((rect->color & 0b111) << 18) |
    0b00000001000000000000000100000000;

  /* Write the data to the framebuffer and SPI at once */
  for (y = rect->dy; y < rect->dy + rect->height; ++y)
  {
    line = info->screen_base + y * WIDTH + rect->dx;
    for (x = 0; x < rect->width >> 2; ++x) line[x] = value;
    spi_write(info->par, line, rect->width);
  }
}

void st7586s_copyarea(struct fb_info* info, const struct fb_copyarea* area)
{
  TRACE();

  /* Discard all writes not fitting to the display */
  if (st7586s_violates_boundaries(area->dx, area->dy, area->width, area->height))
    return;

  pr_info(DRVNAME ": Area copying is not implemented yet\n");
}

void st7586s_imageblit(struct fb_info* info, const struct fb_image* image)
{
  int x, y, retval;
  unsigned int* line;
  unsigned char byte;
  TRACE();

  /* Discard all writes not fitting to the display */
  if (st7586s_violates_boundaries(image->dx, image->dy, image->width, image->height))
    return;

  /* Refuse to blit non-monochrome images */
  if (image->depth != 1)
  {
    pr_info(DRVNAME ": Refusing to blit non-monochrome image\n");
    return;
  }

  /* Refuse to blit unaligned images */
  if (image->dx & 1)
  {
    pr_info(DRVNAME ": Refusing to blit unaligned image\n");
    return;
  }

  /* Refuse to blit images with size not aligned to 8 pix */
  if (image->width & 0x7)
  {
    pr_info(DRVNAME ": Refusing to blit unproperly sized image\n");
    return;
  }

  /* Prepare for data transmission */
  if ((retval = st7586s_prepare_transmission(info->par, image->dx >> 1, image->dy, image->width >> 1, image->height)) != 0)
  {
    pr_err(DRVNAME ": Failed to prepare for data transmission (%i)\n", retval);
    return;
  }

  for (y = 0; y < image->height; ++y)
  {
    line = info->screen_base + (y + image->dy) * WIDTH + image->dx;
    for (x = 0; x < image->width >> 3; ++x)
    {
      byte = image->data[x + y * (image->width >> 3)];
      line[(x << 1) | 0] =
        ((0b111 * (byte >> 7 & 1)) <<  5) |
        ((0b111 * (byte >> 6 & 1)) <<  2) |
        ((0b111 * (byte >> 5 & 1)) << 21) |
        ((0b111 * (byte >> 4 & 1)) << 18) |
        0b00000001000000000000000100000000;
      line[(x << 1) | 1] =
        ((0b111 * (byte >> 3 & 1)) <<  5) |
        ((0b111 * (byte >> 2 & 1)) <<  2) |
        ((0b111 * (byte >> 1 & 1)) << 21) |
        ((0b111 * (byte >> 0 & 1)) << 18) |
        0b00000001000000000000000100000000;
    }
    spi_write(info->par, line, image->width);
  }
}

static struct fb_ops st7586s_ops =
{
  .owner = THIS_MODULE,
  .fb_fillrect = st7586s_fillrect,
  .fb_copyarea = st7586s_copyarea,
  .fb_imageblit = st7586s_imageblit,
};

static struct fb_deferred_io st7586s_defio =
{
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
  if ((retval = spi_setup(spi)) != 0)
  {
    pr_err(DRVNAME ": Failed to switch to 9-bit words\n");
    goto failure;
  }

  if ((retval = st7586s_configure(spi)) != 0)
  {
    pr_err(DRVNAME ": Initial chip configuration failed\n");
    goto failure;
  }

  /* Prepare error for memory allocating functions */
  retval = -ENOMEM;

  if ((vmem = (u8*)kmalloc(vmem_size, GFP_KERNEL)) == NULL)
  {
    pr_err(DRVNAME ": Failed to allocate memory for framebuffer\n");
    goto failure;
  }

  if ((info = framebuffer_alloc(0, &spi->dev)) == NULL)
  {
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

  if ((retval = register_framebuffer(info)) != 0)
  {
    pr_err(DRVNAME ": Failed to register framebuffer\n");
    goto failure;
  }

  spi_set_drvdata(spi, info);
  return 0;

failure:

  if (vmem) kfree(vmem);
  if (info) framebuffer_release(info);
  return retval;
}

static int st7586s_remove(struct spi_device* spi)
{
  struct fb_info* info;
  TRACE();

  if ((info = spi_get_drvdata(spi)) != NULL)
  {
    unregister_framebuffer(info);
    framebuffer_release(info);
    kfree(info->screen_base);
  }

  spi_set_drvdata(spi, NULL);
  return 0;
}

static struct spi_driver st7586s_driver =
{
  .driver =
  {
    .name = "st7586s",
  },
  .probe = st7586s_probe,
  .remove = st7586s_remove,
};

static int __init st7586s_init()
{
  TRACE();
  return spi_register_driver(&st7586s_driver);
}

static void __exit st7586s_exit()
{
  TRACE();
  spi_unregister_driver(&st7586s_driver);
}

module_init(st7586s_init);
module_exit(st7586s_exit);

MODULE_AUTHOR("Mikhail Burakov <mikhail.burakov@gmail.com>");
MODULE_DESCRIPTION("SM75867S framebuffer driver");
MODULE_LICENSE("GPL");
