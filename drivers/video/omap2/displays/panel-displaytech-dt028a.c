/*
 * LCD panel driver for Displaytech DT028ATFT
 * based on Sharp LQ043T1DG01 & ACX565AKM drivers
 *
 * Copyright (C) 2012 pironex GmbH
 * Author: Bjoern Krombhol <b.krombholz@pironex.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/spi/spi.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#include <plat/display.h>

/* ILI9341 registers (incomplete) */
#define READ_DISPLAY_ID			0x04
#define READ_DISPLAY_PIXEL_FORMAT	0x0C
#define SLEEP_ENTER			0x10
#define SLEEP_OUT			0x11
#define GAMMA_SET			0x26
#define DISPLAY_OFF			0x28
#define DISPLAY_ON			0x29
#define SET_COLUMN_ADDRESS		0x2A
#define SET_PAGE_ADDRESS		0x2B
#define WRITE_MEMORY			0x2C
#define READ_MEMORY			0x2E
#define MEMORY_ACCESS_CONTROL		0x36
#define WRITE_MEMORY_CONTINUE		0x3C
#define READ_MEMORY_CONTINUE		0x3E
#define PIXEL_FORMAT_SET		0x3A
#define READ_DISPLAY_BRIGHTNESS		0x52
#define WRITE_CTRL_DISPLAY		0x53
#define READ_CTRL_DISPLAY		0x54
#define FRAME_RATE_CONTROL		0xB1
#define DISPLAY_INVERSION_CONTROL	0xB4
#define DISPLAY_FUNCTION_CONTROL	0xB6
#define BACKLIGHT_CONTROL_1		0xB8
#define BACKLIGHT_CONTROL_2		0xB9
#define BACKLIGHT_CONTROL_3		0xBA
#define BACKLIGHT_CONTROL_4		0xBB
#define BACKLIGHT_CONTROL_5		0xBC
#define BACKLIGHT_CONTROL_6		0xBD
#define BACKLIGHT_CONTROL_7		0xBE
#define BACKLIGHT_CONTROL_8		0xBF
#define POWER_CONTROL_1			0xC0
#define POWER_CONTROL_2			0xC1
#define VCOM_CONTROL_1			0xC5
#define VCOM_CONTROL_2			0xC7
#define POWER_CONTROL_A			0xCB
#define POWER_CONTROL_B			0xCF
#define POSITIVE_GAMMA_CORRECTION	0xE0
#define NEGATIVE_GAMMA_CORRECTION	0xE1
#define DRIVER_TIMING_CONTROL_A		0xE8
#define DRIVER_TIMING_CONTROL_B		0xEA
#define POWER_ON_SEQUENCE_CONTROL	0xED
#define UNDOCUMENTED_0xEF		0xEF
#define ENABLE_3G			0xF2
#define INTERFACE_CONTROL		0xF6
#define UNDOCUMENTED_0xF7		0xF7
#define DELAY				0x00


struct ilicmd {
	int len;
	u8 cmd;
	u16 data[16];
};

static struct ilicmd cmdlist[] = {
	/* 0xEF */
	{3,  UNDOCUMENTED_0xEF, { 0x03, 0x80, 0x02 } },
	/* 0xB1 intern. 1/1 divisor, fps 70 Hz */
	{2,  FRAME_RATE_CONTROL, { 0x00, 0x1B} },
	/* 0x36 row/column order memory -> display, RGB order */
	{1,  MEMORY_ACCESS_CONTROL, {0x48} },
	/* 0xF2 3Gamma function disable */
	{1,  ENABLE_3G, {0x00} },
	/* 0x2A col addresses 0-239 */
	{4,  SET_COLUMN_ADDRESS, {0x00, 0x00, 0x00, 0xEF} },
	/* 0x2B page addresses 0-319 */
	{4,  SET_PAGE_ADDRESS, {0x00, 0x00, 0x01, 0x3F} },
	/* 0x26 Gamma curve selected */
	{1,  GAMMA_SET, {0x01} },
	{1, 0x51, {128} },
	/* 0xE0 positive gamma correction table */
	{15, POSITIVE_GAMMA_CORRECTION, {
			0x0f, 0x27, 0x23, 0x0b,
			0x0f, 0x05, 0x54, 0x74,
			0x45, 0x0a, 0x17, 0x0a,
			0x1c, 0x0e, 0x08} },
	/* 0xE1 negative gamma correction table */
	{15, NEGATIVE_GAMMA_CORRECTION, {
			0x08, 0x1a, 0x1e, 0x03,
			0x0f, 0x05, 0x2e, 0x25,
			0x3b, 0x01, 0x06, 0x05,
			0x25, 0x33, 0x0F} },
	/* 0x3A colmod pixel format RGB-666*/
	{1,  PIXEL_FORMAT_SET, {0x66} },
	/* 0xF6 interface control */
	{3,  INTERFACE_CONTROL, {0x01, 0x01, 0x06} },
	/* 0xB0 interface signal control */
	{1,  0xB0, {0xC0 } },
	/* blank porching control */
	{1,  0xB5, {0x02, 0x02, 0x0A, 0x14 } },
	{0,  SLEEP_OUT, {} },
	/* sleep 120ms after leaving sleep mode */
	{1,  DELAY, {120} },
	{0,  DISPLAY_ON, {} },
	{0, 0xff, {} }, // end marker
};

struct dt028a_device {
	char		*name;
	int		enabled;
	//int		model;
	//int		revision;
	u8		display_id[3];
	unsigned	has_bc:1;
	unsigned	has_cabc:1;
	unsigned	cabc_mode;

	struct spi_device	*spi;
	struct mutex		mutex;

	struct omap_dss_device	*dssdev;
	struct backlight_device *bl_dev;
};
static struct dt028a_device dt_dev;

static void dt028a_transfer(struct dt028a_device *md, int cmd,
			      const u8 *wbuf, int wlen, u8 *rbuf, int rlen)
{
	struct spi_message	m;
	struct spi_transfer	*x, xfer[5];
	int			r;

	BUG_ON(md->spi == NULL);

	spi_message_init(&m);

	memset(xfer, 0, sizeof(xfer));
	x = &xfer[0];

	cmd &=  0xff;
	x->tx_buf = &cmd;
	x->bits_per_word = 9;
	x->len = 2;

	if (rlen > 1 && wlen == 0) {
		/*
		 * Between the command and the response data there is a
		 * dummy clock cycle. Add an extra bit after the command
		 * word to account for this.
		 */
		x->bits_per_word = 10;
		cmd <<= 1;
	}
	spi_message_add_tail(x, &m);

	if (wlen) {
		x++;
		x->tx_buf = wbuf;
		x->len = 2*wlen;
		x->bits_per_word = 9;
		spi_message_add_tail(x, &m);
	}

	if (rlen) {
		x++;
		x->rx_buf	= rbuf;
		x->len		= rlen;
		spi_message_add_tail(x, &m);
	}

	r = spi_sync(md->spi, &m);
	if (r < 0)
		dev_err(&md->spi->dev, "spi_sync %d\n", r);
}

static inline void dt028a_cmd(struct dt028a_device *md, int cmd)
{
	dt028a_transfer(md, cmd, NULL, 0, NULL, 0);
}

static inline void dt028a_write(struct dt028a_device *md,
			       int reg, const u8 *buf, int len)
{
	dt028a_transfer(md, reg, buf, len, NULL, 0);
}

static inline void dt028a_read(struct dt028a_device *md,
			      int reg, u8 *buf, int len)
{
	dt028a_transfer(md, reg, NULL, 0, buf, len);
}

/* Backlight Control */
#define DT_MAX_BRIGHTNESS 0x0F
static void enable_backlight_ctrl(struct dt028a_device *md, int enable)
{
	u16 ctrl;

	dt028a_read(md, READ_CTRL_DISPLAY, (u8 *)&ctrl, 1);
	if (enable) {
		ctrl |= (1 << 5) | (1 << 2);
	} else {
		ctrl &= ~((1 << 5) | (1 << 2));
	}

	ctrl |= 1 << 8;
	dt028a_write(md, WRITE_CTRL_DISPLAY, (u8 *)&ctrl, 1);
}

static void dt_set_brightness(struct dt028a_device *md, int level)
{
	int bv;

	bv = (0x0F - level) | (1 << 8);
	dt028a_write(md, BACKLIGHT_CONTROL_1, (u8 *)&bv, 1);

	if (level)
		enable_backlight_ctrl(md, 1);
	else
		enable_backlight_ctrl(md, 0);
}

static int dt_get_actual_brightness(struct dt028a_device *md)
{
	/* no way to read back BL level */
	u8 bv;

	dt028a_read(md, READ_DISPLAY_BRIGHTNESS, &bv, 1);
	dev_dbg(&md->spi->dev, "actual brightness: %u\n",bv);

	return bv;
}


static int dt_bl_update_status(struct backlight_device *dev)
{
	struct dt028a_device *md = dev_get_drvdata(&dev->dev);
	int r;
	int level;

	dev_dbg(&md->spi->dev, "%s\n", __func__);

	mutex_lock(&md->mutex);

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK)
		level = dev->props.brightness;
	else
		level = 0;

	r = 0;
	if (md->has_bc)
		dt_set_brightness(md, level);
	else if (md->dssdev->set_backlight)
		r = md->dssdev->set_backlight(md->dssdev, level);
	else
		r = -ENODEV;

	mutex_unlock(&md->mutex);

	return r;
}

static int dt_bl_get_intensity(struct backlight_device *dev)
{
	struct dt028a_device *md = dev_get_drvdata(&dev->dev);

	dev_dbg(&dev->dev, "%s\n", __func__);

	if (!md->has_bc && md->dssdev->set_backlight == NULL)
		return -ENODEV;

	if (dev->props.fb_blank == FB_BLANK_UNBLANK &&
			dev->props.power == FB_BLANK_UNBLANK) {
		if (md->has_bc)
			return dt_get_actual_brightness(md);
		else
			return dev->props.brightness;
	}

	return 0;
}

static const struct backlight_ops dt028a_bl_ops = {
	.get_brightness = dt_bl_get_intensity,
	.update_status  = dt_bl_update_status,
};

/* display timings */
static struct omap_video_timings dem_timings = {
	.x_res = 240,
	.y_res = 320,

	.pixel_clock	= 6350,
	.hsw		= 9,
	.hfp		= 9,
	.hbp		= 19,

	.vsw		= 1,
	.vfp		= 3,
	.vbp		= 1,
};

static int dt_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;
	//int i = 1;
	int i = 0;
	int j = 0;
	struct dt028a_device *md = &dt_dev;
	u8 data[5];

	dev_dbg(&md->spi->dev, "Display power on\n");
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	mutex_lock(&md->mutex);

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	if (md->enabled) {
		dev_dbg(&md->spi->dev, "panel already enabled\n");
		mutex_unlock(&md->mutex);
		return 0;
	}


	while (cmdlist[i].cmd != 0xff) {
		//u8 len = ILI9341_init[i];
		//u8 cmd = ILI9341_init[i+1];
		//const u8 *buf = &ILI9341_init[i+2];
		int len = cmdlist[i].len;
		u8 cmd = cmdlist[i].cmd;
		const u8 *buf = (u8 *)cmdlist[i].data;
		if (cmd == DELAY) {
			msleep(buf[0]);
		} else if (len) {
			for (j = 0; j < len; j++)
				cmdlist[i].data[j] |= (1 << 8); // data flag
			dev_dbg(&md->spi->dev,
					"Display set reg:0x%02x len:%d %04x\n",
					cmd, len, buf[0]);
			dt028a_write(md, cmd, buf, len);
		} else {
			dt028a_cmd(md, cmd);
		}
		//i = i + 2 + len;
		i++;
	}
	md->enabled = 1;
//	msleep(120);
//	dt028a_cmd(md, DISPLAY_ON);
//	dt028a_cmd(md, WRITE_MEMORY);
//	msleep(20);

	dt028a_read(md, 0x04, data, 3);
	dev_dbg(&md->spi->dev, "Display id: %02x.%02x.%02x\n",
		data[0], data[1], data[2]);

	dt028a_read(md, 0xDA, data, 1);
	dev_dbg(&md->spi->dev, "Display ID1: %02x\n",
		data[0]);
	dt028a_read(md, 0xDB, data, 1);
	dev_dbg(&md->spi->dev, "Display ID2: %02x\n",
		data[0]);
	dt028a_read(md, 0xDC, data, 1);
	dev_dbg(&md->spi->dev, "Display ID3: %02x\n",
		data[0]);
	dt028a_read(md, 0xD3, data, 3);
	dev_dbg(&md->spi->dev, "Display ID4: %02x.%02x.%02x\n",
		data[0], data[1], data[2]);

	mutex_unlock(&md->mutex);

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	mutex_unlock(&md->mutex);
	return r;
}

static void dt_panel_power_off(struct omap_dss_device *dssdev)
{
	struct dt028a_device *md = &dt_dev;
	dev_dbg(&md->spi->dev, "Display power off\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	mutex_lock(&md->mutex);

	if (!md->enabled) {
		mutex_unlock(&md->mutex);
		return;
	}
	dt028a_cmd(md, SLEEP_ENTER);
	md->enabled = 0;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */
	msleep(100);

	omapdss_dpi_display_disable(dssdev);

	mutex_unlock(&md->mutex);
}

static int dt_panel_probe(struct omap_dss_device *dssdev)
{
	struct dt028a_device *md = &dt_dev;
	struct backlight_device *bldev;
	int max_brightness, brightness;
	struct backlight_properties props;
	//u8 id[3];

	/* V/HSYNC low active, data + sync driven on falling PCLK */
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS | OMAP_DSS_LCD_IPC | OMAP_DSS_LCD_ONOFF;
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = dem_timings;

	dt028a_read(md, READ_DISPLAY_ID, md->display_id, 3);
	dev_dbg(&md->spi->dev, "MIPI display ID: %02x%02x%02x\n",
		md->display_id[0], md->display_id[1], md->display_id[2]);

	mutex_lock(&dt_dev.mutex);
	dt_dev.enabled = 0;
	dt_dev.dssdev = dssdev;
	dt_dev.has_bc = 1;
	dt_dev.has_cabc = 0; /* TODO implement */
	dt_dev.name = "DT028ATFT";
	mutex_unlock(&dt_dev.mutex);

	/* backlight */
	props.fb_blank = FB_BLANK_UNBLANK;
	props.power = FB_BLANK_UNBLANK;

	bldev = backlight_device_register("dt028a", &md->spi->dev,
			md, &dt028a_bl_ops, &props);

	if (md->has_bc)
		max_brightness = 0x0f;
	else
		max_brightness = dssdev->max_backlight_level;

	if (md->has_bc)
		brightness = dt_get_actual_brightness(md);
	else if (dssdev->get_backlight)
		brightness = dssdev->get_backlight(dssdev);
	else
		brightness = 0;

	bldev->props.max_brightness = max_brightness;
	bldev->props.brightness = brightness;

	dt_bl_update_status(bldev);

	return 0;
}

static void dt_panel_remove(struct omap_dss_device *dssdev)
{
}

static int dt_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	dev_dbg(&dt_dev.spi->dev, "Display enable\n");

	r = dt_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void dt_panel_disable(struct omap_dss_device *dssdev)
{
	dev_dbg(&dt_dev.spi->dev, "Display disable\n");

	dt_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int dt_panel_suspend(struct omap_dss_device *dssdev)
{
	dev_dbg(&dt_dev.spi->dev, "Display suspend\n");

	dt_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int dt_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;
	dev_dbg(&dt_dev.spi->dev, "Display resume\n");

	r = dt_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static struct omap_dss_driver dt028a_panel_driver = {
	.probe		= dt_panel_probe,
	.remove		= dt_panel_remove,

	.enable		= dt_panel_enable,
	.disable	= dt_panel_disable,
	.suspend	= dt_panel_suspend,
	.resume		= dt_panel_resume,

	.driver         = {
		.name   = "displaytech_dt028a_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init dt_panel_drv_init(void)
{
	return omap_dss_register_driver(&dt028a_panel_driver);
}

static void __exit dt_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&dt028a_panel_driver);
}

/*--------------------SPI probe-------------------------*/

static int dt028a_spi_probe(struct spi_device *spi)
{
	struct dt028a_device *md = &dt_dev;

	dev_dbg(&spi->dev, "%s\n", __func__);

	spi->mode = SPI_MODE_0; // CPOL+, CPHA rising
	md->spi = spi;
	mutex_init(&md->mutex);
	dev_set_drvdata(&spi->dev, md);

	dt_panel_drv_init();

	return 0;
}

static int dt028a_spi_remove(struct spi_device *spi)
{
	struct dt028a_device *md = dev_get_drvdata(&spi->dev);

	dev_dbg(&md->spi->dev, "%s\n", __func__);
	omap_dss_unregister_driver(&dt028a_panel_driver);

	return 0;
}

static struct spi_driver dt028a_spi_driver = {
	.driver = {
		.name	= "dt028a",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= dt028a_spi_probe,
	.remove	= __devexit_p(dt028a_spi_remove),
};

static int __init dt028a_drv_init(void)
{
	return spi_register_driver(&dt028a_spi_driver);
}

static void __exit dt028a_drv_exit(void)
{
	spi_unregister_driver(&dt028a_spi_driver);
}

module_init(dt028a_drv_init);
module_exit(dt028a_drv_exit);
MODULE_LICENSE("GPL");
