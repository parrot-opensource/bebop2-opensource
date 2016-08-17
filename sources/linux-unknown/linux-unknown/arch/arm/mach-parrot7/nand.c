/**
 * linux/arch/arm/mach-parrot7/nand.c - Nand controller platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    15-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <nand/cast-nand.h>
#include <linux/mtd/nand.h>

#include "common.h"
#include "clock.h"

/* The NAND controller allocate dma_memory only once in its probe,
 * so the size is known at compilation time
 */
static size_t dma_size = NAND_MAX_PAGESIZE + NAND_MAX_OOBSIZE;
static dma_addr_t dma_addr;

static struct resource p7_nand_res[] = {
	[0] = {
		.start = P7_NAND,
		.end   = P7_NAND + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_NAND_IRQ,
		.end   = P7_NAND_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static u64 p7_nand_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7_nand_dev = {
	.name           = CAST_DRV_NAME,
	.id             = 0,
	.resource       = p7_nand_res,
	.num_resources  = ARRAY_SIZE(p7_nand_res),
	.dev            = {
		.dma_mask           = &p7_nand_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/**
 * p7_init_nand() - Instantiate NAND controller for further driver usage.
 *
 * @pdata:      controller platform specific data
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_nand(struct cast_plat_data* pdata,
                         struct pinctrl_map* pins,
                         size_t pin_cnt)
{
#ifdef DEBUG
	BUG_ON(! pdata);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif

	p7_init_dev(&p7_nand_dev, pdata, pins, pin_cnt);
	if (! dma_declare_coherent_memory(&p7_nand_dev.dev,
					  dma_addr,
					  dma_addr,
					  dma_size,
					  DMA_MEMORY_MAP))
		panic("p7: failed to remap DMA area [%08x:%08x] for device %s\n",
				dma_addr,
				dma_addr + dma_size -1,
				dev_name(&p7_nand_dev.dev));
}

/**
 * p7_reserve_nand_mem() - Reserve DMA memory for the NAND controller
 */
void __init p7_reserve_nand_mem(void)
{
	/* The memory is not allocated in the global pool.
	 * The driver only does a small allocation that cause fragmentation in the global pool
	 */
	p7_reserve_devmem(&p7_nand_dev,&dma_addr,&dma_size);
}
