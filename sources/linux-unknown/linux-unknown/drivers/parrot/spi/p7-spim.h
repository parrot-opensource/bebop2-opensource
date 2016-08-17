/**
 * linux/drivers/parrot/spi/p7-spim.h - Parrot7 SPI master driver
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author: alvaro.moran@parrot.com
 * @date:   2012-10-01
 *
 * This file is released under the GPL
 */

#ifndef _P7_SPIM_H
#define _P7_SPIM_H

#if defined(CONFIG_SPI_MASTER_PARROT7) || \
	defined(CONFIG_SPI_MASTER_PARROT7_MODULE)

#define P7SPIM_DRV_NAME "p7-spim"

#endif  /* defined(CONFIG_SPI_MASTER_PARROT7) || \
           defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

#endif
