/*
 *
 *  Copyright (C) 2012 Björn Krombholz
 *    based on serial_max3100.h by Christian Pellegrin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


#ifndef _LINUX_SERIAL_MAX3140HD_H
#define _LINUX_SERIAL_MAX3140HD_H 1


/**
 * struct plat_max3140hd - MAX3100 SPI UART platform data
 * @loopback:              force MAX3140 in loopback
 * @crystal:               1 for 3.6864 Mhz, 0 for 1.8432
 * @poll_time:             poll time for CTS signal in ms, 0 disables (so no hw
 *                       flow ctrl is possible but you have less CPU usage)
 * @invert_rts           in half-duplex mode, the driver enable is controlled
 *                       by nRTS pin. This flag allows inverting the RTS logic:
 *                       RTS = high => nRTS = low => DE = high
 *                       normally DE is off, when RTS is on
 *
 * You should use this structure in your machine description to specify
 * how the MAX3100 is connected. Example:
 *
 * static struct plat_max3100 max3100_plat_data = {
 *  .loopback = 0,
 *  .crystal = 0,
 *  .poll_time = 100,
 *  .inverrts = 0,
 * };
 *
 * static struct spi_board_info spi_board_info[] = {
 * {
 *  .modalias       = "max3100",
 *  .platform_data  = &max3100_plat_data,
 *  .irq            = IRQ_EINT12,
 *  .max_speed_hz   = 5*1000*1000,
 *  .chip_select    = 0,
 * },
 * };
 *
 **/
struct plat_max3140hd {
	int loopback;
	int crystal;
	int poll_time; /* not yet implemented */
	int invert_rts;
};

#endif
