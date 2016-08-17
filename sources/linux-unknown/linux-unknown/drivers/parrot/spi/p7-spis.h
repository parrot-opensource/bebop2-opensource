/**
 * linux/drivers/parrot/spi/p7-spis.h - Parrot7 SPI slave driver
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * @author: alvaro.moran@parrot.com
 * @date:   2012-10-01
 *
 * This file is released under the GPL
 */

#ifndef _P7_SPIS_H
#define _P7_SPIS_H

#if defined(CONFIG_SPI_SLAVE_PARROT7) || \
	defined(CONFIG_SPI_SLAVE_PARROT7_MODULE)

#define P7SPIS_DRV_NAME "p7-spis"

struct spi_master;

/*
 * Flush RX and TX FIFOs of SPI core. 
 * This might be necessary since the slave spi keeps on receiving
 * (thus filling the RX FIFO). 
 */
extern void p7spis_flush(struct spi_master *master);
extern int p7spis_pause(struct spi_master *master);
extern int p7spis_resume(struct spi_master *master);

#endif  /* defined(CONFIG_SPI_SLAVE_PARROT7) || \
           defined(CONFIG_SPI_SLAVE_PARROT7_MODULE) */

#endif
