/*
 * Parrot7 MPEG-TS controller platform implementation
 * 
 *  Copyright (C) 2013 Parrot S.A.
 *  
 *  @author: Damien Riegel <damien.riegel.ext@parrot.com>
 *  
 *  This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <media/p7-mpegts.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "common.h"
#include "dma.h"
#include "spi.h"

static struct resource p7_mpegts0_res[] = {
	[0] = {
		.start = P7_MPEGTS0,
		.end   = P7_MPEGTS0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_MPEGTS0_IRQ,
		.end   = P7_MPEGTS0_IRQ,
		.flags = IORESOURCE_IRQ
	},
	[2] = {
		.start	= P7_MPEGTS0_DMA,
		.end	= P7_MPEGTS0_DMA,
		.flags	= IORESOURCE_DMA,
	}
};

static struct resource p7_mpegts1_res[] = {
	[0] = {
		.start = P7_MPEGTS1,
		.end   = P7_MPEGTS1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_MPEGTS1_IRQ,
		.end   = P7_MPEGTS1_IRQ,
		.flags = IORESOURCE_IRQ
	},
	[2] = {
		.start	= P7_MPEGTS1_DMA,
		.end	= P7_MPEGTS1_DMA,
		.flags	= IORESOURCE_DMA,
	}
};

static u64 p7_mpegts0_dma_mask = DMA_BIT_MASK(32);
static u64 p7_mpegts1_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7_mpegts_dev[] = {
	{
		.name           = P7MPG_DRV_NAME,
		.id             = 0,
		.resource       = p7_mpegts0_res,
		.num_resources  = ARRAY_SIZE(p7_mpegts0_res),
		.dev            = {
			.dma_mask           = &p7_mpegts0_dma_mask,
			.coherent_dma_mask  = DMA_BIT_MASK(32),
		},
	},
	{
		.name           = P7MPG_DRV_NAME,
		.id             = 1,
		.resource       = p7_mpegts1_res,
		.num_resources  = ARRAY_SIZE(p7_mpegts1_res),
		.dev            = {
			.dma_mask           = &p7_mpegts1_dma_mask,
			.coherent_dma_mask  = DMA_BIT_MASK(32),
		},
	}
};

static size_t p7_mpegts_dmasz[ARRAY_SIZE(p7_mpegts_dev)] __initdata;

void __init p7_reserve_mpegtsmem(unsigned int core, size_t size)
{
#ifdef DEBUG
	BUG_ON(! size);
	BUG_ON(core >= ARRAY_SIZE(p7_mpegts_dev));
#endif
	p7_mpegts_dmasz[core] = size;
	p7_reserve_devmem(&p7_mpegts_dev[core], NULL, &p7_mpegts_dmasz[core]);
}

void __init p7_init_mpegts(int core,
                           struct p7mpg_plat_data* pdata,
                           struct pinctrl_map* pins,
                           size_t pin_cnt)
{
	int err;
	if (p7_chiprev() == P7_CHIPREV_R1) {
		printk(KERN_ERR "MPEG-TS not available on P7R1\n");
		return;
	}

#ifdef DEBUG
	BUG_ON(core < 0 || core >= ARRAY_SIZE(p7_mpegts_dev));
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif	
	p7_init_spi();

	pdata->dma_sz = p7_mpegts_dmasz[core];

	err = p7_init_dev(&p7_mpegts_dev[core], pdata, pins, pin_cnt);
	if (! err)
		return;

	panic("p7: failed to init MPEG-TS (%d)\n", err);
}
