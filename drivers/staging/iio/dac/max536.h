/*
 * MAX536 DAC driver
 *
 * Copyright 2014 Bjoern Krombholz <b.krombholz@pironex.de>
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef IIO_DAC_MAX536_H_
#define IIO_DAC_MAX536_H_

struct max536_platform_data {
	u16				vref_mv;
};

#endif /* IIO_DAC_MAX536_H_ */
