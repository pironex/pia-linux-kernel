/*
 *  max517.c - Support for Maxim MAX536x
 *
 *  Copyright (C) 2014 Bjoern Krombholz <b.krombholz@pironex.de>
 *                (based on max517)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>

#include "../iio.h"
#include "../sysfs.h"
#include "dac.h"

#include "max536.h"

#define MAX517_DRV_NAME	"max536"

enum max517_device_ids {
	ID_MAX5360,
	ID_MAX5361,
	ID_MAX5362,
};

struct max536_data {
	struct iio_dev		*indio_dev;
	struct i2c_client	*client;
	unsigned short		vref_mv;
};

static ssize_t max536_set_raw_value(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct max536_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	u8 out;
	int res;
	long val;

	res = strict_strtol(buf, 10, &val);

	if (res)
		return res;

	if (val < 0 || val > 0x3f)
		return -EINVAL;

	/* MAX536x uses simple 1 byte protocol specifying the DAC value
	 * in the 6 MSBits. The 2 LSB must be 0. */
	out = (val << 2) & 0xfc;

	res = i2c_master_send(client, &out, 1);
	if (res < 0)
		return res;

	return count;
}
static IIO_DEV_ATTR_OUT_RAW(1, max536_set_raw_value, 0);

static ssize_t max517_show_scale(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct max536_data *data = iio_priv(indio_dev);
	/* max value is defined as 0.9 Vref * (63/64)
	 * 0x01 / 2^6 */
	unsigned int scale_uv = (data->vref_mv * 900) >> 6;

	return sprintf(buf, "%d.%03d\n", scale_uv / 1000, scale_uv % 1000);
}

static IIO_DEVICE_ATTR(out_voltage1_scale, S_IRUGO,
		       max517_show_scale, NULL, 0);

/* On MAX517 variant, we have one output */
static struct attribute *max536_attributes[] = {
	&iio_dev_attr_out_voltage1_raw.dev_attr.attr,
	&iio_dev_attr_out_voltage1_scale.dev_attr.attr,
	NULL
};

static struct attribute_group max536_attribute_group = {
	.attrs = max536_attributes,
};

static int max536_suspend(struct i2c_client *client, pm_message_t mesg)
{
	u8 buf;

	/* the I2C address read bit will trigger the shutdown mode */
	return i2c_master_recv(client, &buf, 1);
}

static int max536_resume(struct i2c_client *client)
{
	u8 outbuf = 0;

	/* any write will wake up the device, time to wake up: 50us */
	return i2c_master_send(client, &outbuf, 1);
}

static const struct iio_info max536_info = {
	.attrs = &max536_attribute_group,
	.driver_module = THIS_MODULE,
};

static int max517_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max536_data *data;
	struct iio_dev *indio_dev;
	struct max536_platform_data *pdata = client->dev.platform_data;
	int err;

	indio_dev = iio_allocate_device(sizeof(*data));
	if (indio_dev == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;

	indio_dev->dev.parent = &client->dev;

	indio_dev->info = &max536_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* this is where the variants of MAX536x differ */
	switch (id->driver_data) {
	case ID_MAX5360:
		data->vref_mv = 1000;
		break;
	case ID_MAX5361:
		data->vref_mv = 2000;
		break;
	case ID_MAX5362:
		data->vref_mv = 3300; /* VDD */
		break;
	default:
		break;
	}
	/* override with provided platform data, useful for MAX5362 only */
	if (pdata) {
		data->vref_mv = 3300; /* mV */
	}

	err = iio_device_register(indio_dev);
	if (err)
		goto exit_free_device;

	dev_info(&client->dev, "DAC registered\n");

	return 0;

exit_free_device:
	iio_free_device(indio_dev);
exit:
	return err;
}

static int max517_remove(struct i2c_client *client)
{
	iio_free_device(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id max536_id[] = {
	{ "max5360", ID_MAX5360 },
	{ "max5361", ID_MAX5361 },
	{ "max5362", ID_MAX5362 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max536_id);

static struct i2c_driver max536_driver = {
	.driver = {
		.name	= MAX517_DRV_NAME,
	},
	.probe		= max517_probe,
	.remove		= max517_remove,
	.suspend	= max536_suspend,
	.resume		= max536_resume,
	.id_table	= max536_id,
};

static int __init max536_init(void)
{
	return i2c_add_driver(&max536_driver);
}

static void __exit max536_exit(void)
{
	i2c_del_driver(&max536_driver);
}

MODULE_AUTHOR("Bjoern Krombholz <b.krombholz@pironex.de>");
MODULE_DESCRIPTION("MAX536x 6-bit I2C DAC");
MODULE_LICENSE("GPL");

module_init(max536_init);
module_exit(max536_exit);
