/*
 * linux/drivers/parrot/media/p7-mpegts.h
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author: Damien Riegel <damien.riegel.ext@parrot.com>
 * @author: Jeremie Samuel <jeremie.samuel.ext@parrot.com>
 *
 * This file is released under the GPL
 */

#ifndef P7_MPEGTS_H_
#define P7_MPEGTS_H_

#include "p7-mpegts_ioctl.h"
#include "p7-mpegts_regs.h"

#define P7MPG_DRV_NAME   "p7-mpegts"
#define P7MPG_CORE_NR    2

/**
 * struct p7mpg_plat_data - Parrot7 MPEGTS per device platform settings
 * @swb:                switchbox settings (see &struct p7spi_swb)
 * @thres_wcnt:         FIFO threshold in number of 32 bits words
 */
struct p7mpg_plat_data {
	struct p7spi_swb const* swb;
	size_t                  fifo_wcnt;
	size_t                  thres_wcnt;
	size_t                  dma_sz;
};

#define P7MPG_MEM_WCNT(_core, _cnt)    \
	(P7SPI_MEM_ ## _cnt ## W << ((4 + _core) * P7SPI_MEM_BITLEN))

#endif
