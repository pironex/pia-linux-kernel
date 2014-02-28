#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

/******************************************************************************/

#ifdef DEBUG
#define TRACE() \
  printk(KERN_INFO DRVNAME "->%s (%s:%i)\n", __FUNCTION__, __FILE__, __LINE__)
#else
#define TRACE()
#endif

/******************************************************************************/

#define DRVNAME "st7586s"
#define WIDTH 384
#define HEIGHT 160

static struct fb_fix_screeninfo st7586s_fix __devinitdata =
{
  .id = "ST7586S",
  .type = FB_TYPE_PACKED_PIXELS,
  .visual = FB_VISUAL_PSEUDOCOLOR,
  .line_length = (WIDTH / 2),
  .accel = FB_ACCEL_NONE,
};

static struct fb_var_screeninfo st7586s_var __devinitdata =
{
  .xres = WIDTH,
  .yres = HEIGHT,
  .xres_virtual = WIDTH,
  .yres_virtual = HEIGHT,
  .bits_per_pixel = 2,
  .nonstd = 1,
};

/******************************************************************************/

#define DATA(arg) ((arg & 0xff) | 0x0100)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (b) : (a))
#define SPI_FLUSH(spi, buf) spi_write(spi, buf, sizeof(buf))

static int st7586s_prepare_transmission(struct spi_device* spi, int x, int y, int w, int h)
{
  int x_end, y_end;
  
  x_end = x + w - 1;
  y_end = y + h - 1;
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

void st7586s_fillrect(struct fb_info* info, const struct fb_fillrect* rect)
{
  int x, y;
  int retval;
  TRACE();

  /* Discard all writes not fitting to the display */
  if (st7586s_violates_boundaries(rect->dx, rect->dy, rect->width, rect->height))
    return;

  /* Prepare for data transmission */
  if ((retval = st7586s_prepare_transmission(info->par, rect->dx >> 1, rect->dy, rect->width >> 1, rect->height)) != 0)
  {
    pr_err(DRVNAME ": Failed to prepare for data transmission (%i)\n", retval);
    return;
  }

  /* TODO: Optimize */
  const unsigned short pattern[] = { DATA(rect->color) };
  for (y = 0; y < rect->height; ++y)
    for (x = 0; x < rect->width; x += 2)
      SPI_FLUSH(info->par, pattern);
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
  int i, retval;
  unsigned short byte;
  TRACE();

  /* Refuse to blit non-monochrome images */
  if (image->depth != 1)
  {
    pr_info(DRVNAME ": Refusing to blit non-monochrome image\n");
    return;
  }

  /* Discard all writes not fitting to the display */
  if (st7586s_violates_boundaries(image->dx, image->dy, image->width, image->height))
    return;

  /* Prepare for data transmission */
  if ((retval = st7586s_prepare_transmission(info->par, image->dx >> 1, image->dy, image->width >> 1, image->height)) != 0)
  {
    pr_err(DRVNAME ": Failed to prepare for data transmission (%i)\n", retval);
    return;
  }

  /* TODO: Optimize */
  for (i = 0; i < image->width * image->height >> 3; ++i)
  {
    byte = image->data[i];
    const unsigned short unpacked[] =
    {
      0x100 | (0b111 * (byte >> 7 & 1)) << 5 | (0b111 * (byte >> 6 & 1)) << 2,
      0x100 | (0b111 * (byte >> 5 & 1)) << 5 | (0b111 * (byte >> 4 & 1)) << 2,
      0x100 | (0b111 * (byte >> 3 & 1)) << 5 | (0b111 * (byte >> 2 & 1)) << 2,
      0x100 | (0b111 * (byte >> 1 & 1)) << 5 | (0b111 * (byte >> 0 & 1)) << 2,
    };
    SPI_FLUSH(info->par, unpacked);
  }
}

static struct fb_ops st7586s_ops =
{
  .owner = THIS_MODULE,
  .fb_fillrect = st7586s_fillrect,
  .fb_copyarea = st7586s_copyarea,
  .fb_imageblit = st7586s_imageblit,
};

/******************************************************************************/

static int st7586s_probe(struct spi_device* spi)
{
  int retval;
  struct fb_info* info;
  TRACE();

  spi->bits_per_word = 9;
  if ((retval = spi_setup(spi)) != 0)
  {
    pr_err(DRVNAME ": Failed to switch to 9-bit words\n");
    return retval;
  }

  if ((retval = st7586s_configure(spi)) != 0)
  {
    pr_err(DRVNAME ": Initial chip configuration failed\n");
    return retval;
  }

  if ((info = framebuffer_alloc(0, &spi->dev)) == NULL)
  {
    pr_err(DRVNAME ": Failed to allocate memory for framebuffer\n");
    return -ENOMEM;
  }

  info->fbops = &st7586s_ops;
  info->fix = st7586s_fix;
  info->var = st7586s_var;
  info->flags = FBINFO_FLAG_DEFAULT | FBINFO_VIRTFB;
  info->par = spi;

  if ((retval = register_framebuffer(info)) != 0)
  {
    pr_err(DRVNAME ": Failed to register framebuffer\n");
    framebuffer_release(info);
    return retval;
  }

  spi_set_drvdata(spi, info);
  return 0;
}

static int st7586s_remove(struct spi_device* spi)
{
  struct fb_info* info;
  TRACE();

  if ((info = spi_get_drvdata(spi)) != NULL)
  {
    unregister_framebuffer(info);
    framebuffer_release(info);
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
