/*
 * include/media/mt9t111.h
 *
 * mt9t111 sensor driver
 *
 * Copyright (C) 2009 Leopard Imaging
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef	MT9T111_H
#define	MT9T111_H

/*********************************
 * Defines and Macros and globals
 ********************************/

#ifdef	TRUE
#undef	TRUE
#endif

#ifdef	FALSE
#undef	FALSE
#endif

#define	TRUE 	1
#define	FALSE	0

#ifdef DEBUG
#undef DEBUG
#endif

#ifndef TYPES
#define TYPES
#endif

#define MT9T111_I2C_REGISTERED			(1)
#define MT9T111_I2C_UNREGISTERED		(0)

/*i2c adress for MT9T111*/
#define MT9T111_I2C_ADDR  		(0x78 >> 1)

#define MT9T111_CLK_MAX 	(75000000) /* 75MHz */
#define MT9T111_CLK_MIN	(6000000)  /* 6Mhz */

#define MT9T111_I2C_CONFIG		(1)
#define I2C_ONE_BYTE_TRANSFER		(1)
#define I2C_TWO_BYTE_TRANSFER		(2)
#define I2C_THREE_BYTE_TRANSFER		(3)
#define I2C_FOUR_BYTE_TRANSFER		(4)
#define I2C_TXRX_DATA_MASK		(0x00FF)
#define I2C_TXRX_DATA_MASK_UPPER	(0xFF00)
#define I2C_TXRX_DATA_SHIFT		(8)

struct mt9t111_platform_data {
	char *master;
	int (*power_set) (enum v4l2_power on);
	int (*ifparm) (struct v4l2_ifparm *p);
	int (*priv_data_set) (void *);
	/* Interface control params */
	bool clk_polarity;
	bool hs_polarity;
	bool vs_polarity;
};

/**
 * struct capture_size - image capture size information
 * @width: image width in pixels
 * @height: image height in pixels
 */
struct capture_size {
	unsigned long width;
	unsigned long height;
};

#endif				/*for  ifndef MT9T111 */

