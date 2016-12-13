/*
 * Platform data for the sii5293
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 */


#ifndef __SII_PLATFORM_DATA_H__
#define __SII_PLATFORM_DATA_H__

#define SII5293_PDATA_STRUCT_VERSION        1

struct sii5293_platform_data {
	/* keep version field in first position */
	int     version;

	int     gpio_rst;
	int     gpio_irq;
	int     i2c_bus;

	int     output_format;

	int     input_dev_rap;
	int     input_dev_rcp;
	int     input_dev_ucp;
};

#endif
