/*
 * Code for supporting AM335X PIA.
 *
 * Copyright (C) {2013} pironex GmbH - http://www.pironex.de/
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

#ifndef _BOARD_AM335XPIA_H
#define _BOARD_AM335XPIA_H

#define PIA335_KM_E2		20
#define PIA335_KM_MMI		21
#define PIA335_LCD_KM_MMI	22
#define PIA335_PM		22
#define PIA335_LOKISA_EM	23
#define PIA335_BB_EBTFT		30
#define PIA335_BB_SK		31
#define PIA335_BB_APC		32
#define PIA335_LCD_EBTFT	40

#ifndef __ASSEMBLER__
int am335x_pia_get_id(void);

void am33xx_core_vg_scale_i2c_seq_fillup(char *sleep_seq, size_t ssz,
					 char *wake_seq, size_t wsz);

#endif
#endif
