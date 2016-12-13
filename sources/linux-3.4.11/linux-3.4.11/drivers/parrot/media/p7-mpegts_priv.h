/*
 * linux/drivers/parrot/media/p7-mpegts_priv.h
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author: Damien Riegel <damien.riegel.ext@parrot.com>
 * @author: Jeremie Samuel <jeremie.samuel.ext@parrot.com>
 *
 * This file is released under the GPL
 */

#ifndef P7_MPEGTS_PRIV_H_
#define P7_MPEGTS_PRIV_H_

#if defined(CONFIG_MPEGTS_PARROT7) || \
	defined(CONFIG_MPEGTS_PARROT7_MODULE)

#include "p7-mpegts_ioctl.h"
#include "p7-mpegts_regs.h"

#include <linux/miscdevice.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>

#include <spi/p7-spi.h>
#include <spi/p7-spi_priv.h>


#define P7MPG_DRV_NAME   "p7-mpegts"

struct p7mpg_block_desc;

struct p7mpg_core {
	int                         id;
	struct device*              device;
	struct miscdevice           miscdev;
	
	struct resource const *     regs;
	unsigned long               vaddr;

	/* info about the region reserved for DMA */
	size_t                      dma_region_sz;
	void*                       dma_virt;
	dma_addr_t                  dma_bus;

	struct dma_chan*            dma_chan;
	bool                        map_dma;
	struct dma_slave_config     dma_cfg;
	dma_cookie_t                dma_cook;
	dma_addr_t                  dma_addr;
	size_t                      dma_sz;
	size_t                      dma_thres;
	size_t                      dma_min;

	unsigned long               fifo_paddr;
	size_t                      fifo_sz;
	/* Transfer bytes threshold */
	size_t                      thres_sz;

	/* Driver params */
	unsigned int                pkt_by_blk;
	unsigned char               blk_by_rb;

	void*                       metadata_mem;
	struct p7mpg_metadata*      metadata;
	unsigned int                dma_block;

	struct pinctrl*             pctl;

	/* Pins valid or sync avalaibility */
	int                         valid_pin;
	int                         sync_pin;

	/* Array to save registers during suspend/resume procedure */
	u32                         *saved_regs;
	bool                        started;
};

static inline int p7mpg_core_id(struct p7mpg_core const * core)
{
	return core->id;
}

#endif  /* defined(CONFIG_MPEGTS_PARROT7) || \
           defined(CONFIG_MPEGTS_PARROT7_MODULE) */

#endif
