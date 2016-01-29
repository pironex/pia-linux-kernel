/*
 *  MAX3191x - GP Input expander for Maxim MAX3191x series
 *
 *  Copyright (C) 2016 Bjoern Krombholz <b.krombholz@pironex.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/module.h>

#define MAX3191X_NUM_GPIOS	8

struct max3191x_chip {
	u8			buffer[2]; /* [0] contains inputs state */
	struct gpio_chip	gpio_chip;
	struct mutex		lock;
};

static struct max3191x_chip *gpio_to_max3191x_chip(struct gpio_chip *gc)
{
	return container_of(gc, struct max3191x_chip, gpio_chip);
}

static int __max3191x_read_values(struct max3191x_chip *chip)
{
	struct spi_device *spi = to_spi_device(chip->gpio_chip.dev);
	int ret = 0;

	/* read 2 bytes, first byte will contain input values */
	ret = spi_read(spi, &chip->buffer, sizeof(chip->buffer));

	dev_dbg(&spi->dev, "SPI: %02x %02x\n",
		chip->buffer[0], chip->buffer[1]);
	return ret;
}

static int max3191x_get_value(struct gpio_chip *gc, unsigned offset)
{
	struct max3191x_chip *chip = gpio_to_max3191x_chip(gc);

	u8 pin = offset;
	int ret;

	mutex_lock(&chip->lock);
	ret = __max3191x_read_values(chip);
	mutex_unlock(&chip->lock);

	if (unlikely(ret)) {
		return ret;
	}

	ret = (chip->buffer[0] >> pin) & 0x01;

	return ret;
}

static int max3191x_direction_input(struct gpio_chip *gc, unsigned offset)
{
	return 0;
}

static int max3191x_probe(struct spi_device *spi)
{
	struct max3191x_chip *chip;
	struct device *dev;
	int ret;

	dev = &spi->dev;

	/* Word size is actually 8 or 16 depending on mode-pin
	 * we only really care about the first byte containing the
	 * input states for now. This also avoids having to swap around
	 * the bytes when endianess changes. */
	spi->bits_per_word = 8;

	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	spi_set_drvdata(spi, chip);

	chip->gpio_chip.label = spi->modalias;
	chip->gpio_chip.direction_input = max3191x_direction_input;
	chip->gpio_chip.get = max3191x_get_value;
	chip->gpio_chip.base = -1;

	chip->gpio_chip.ngpio = MAX3191X_NUM_GPIOS;
	chip->gpio_chip.can_sleep = true;

#ifdef CONFIG_OF
	chip->gpio_chip.of_gpio_n_cells = 2;
	chip->gpio_chip.of_node = dev->of_node;
#endif
	chip->gpio_chip.dev = dev;
	chip->gpio_chip.owner = THIS_MODULE;

	mutex_init(&chip->lock);

	ret = __max3191x_read_values(chip);
	if (ret < 0) {
		dev_err(dev, "Failed getting input values: %d\n", ret);
		goto exit_destroy;
	}

	ret = gpiochip_add(&chip->gpio_chip);
	if (!ret)
		return 0;

exit_destroy:
	mutex_destroy(&chip->lock);

	return ret;
}

static int max3191x_remove(struct spi_device *spi)
{
	struct max3191x_chip *chip = spi_get_drvdata(spi);

	gpiochip_remove(&chip->gpio_chip);
	mutex_destroy(&chip->lock);

	return 0;
}

static const struct of_device_id max3191x_dt_ids[] = {
	{ .compatible = "maxim,max3191x" },
	{},
};
MODULE_DEVICE_TABLE(of, max3191x_dt_ids);

static struct spi_driver max3191x_driver = {
	.driver = {
		.name		= "max3191x",
		.owner		= THIS_MODULE,
		.of_match_table	= max3191x_dt_ids,
	},
	.probe		= max3191x_probe,
	.remove		= max3191x_remove,
};
module_spi_driver(max3191x_driver);

MODULE_AUTHOR("Bjoern Krombholz <b.krombholz@pironex.de>");
MODULE_DESCRIPTION("GP input expander driver for MAX3191x serializer");
MODULE_LICENSE("GPL v2");
