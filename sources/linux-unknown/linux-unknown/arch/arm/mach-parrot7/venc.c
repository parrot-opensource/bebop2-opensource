/**
 * linux/arch/arm/mach-parrot7/venc.c - Parrot7 video encoder platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    18-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <uio/hx280-venc.h>
#include "common.h"
#include "clock.h"

static struct resource p7_venc_res[] = {
	[0] = {
		.start  = P7_VENC,
		.end    = P7_VENC + SZ_4K - 1,
		.flags  = IORESOURCE_MEM
	},
	[1] = {
		.start  = P7_VENC_IRQ,
		.end    = P7_VENC_IRQ,
		.flags  = IORESOURCE_IRQ
	}
};

static u64 p7_venc_dma_mask = DMA_BIT_MASK(32);

struct platform_device p7_venc_dev = {
	.name           = HX280_DRV_NAME,
	.id             = 0,
	.resource       = p7_venc_res,
	.num_resources  = ARRAY_SIZE(p7_venc_res),
	.dev            = {
		.dma_mask           = &p7_venc_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/**
 * p7_reserve_vencmem() - Reserve DMA memory region for video encoder usage.
 *
 * @size: size of DMA region in bytes
 */
void __init p7_reserve_vencmem(size_t size)
{
	/* Reserve memory in the global DMA contiguous pool */
	p7_reserve_devmem(&p7_venc_dev, NULL, &size);
}

/**
 * p7_init_venc() - Instantiate Video encoder for further driver usage.
 */
void __init p7_init_venc(void)
{
	int err;

	BUG_ON(! p7_venc_dev.num_resources);

	err = p7_init_dev(&p7_venc_dev, NULL, NULL, 0);
	if (err)
		panic("p7: failed to init venc (%d)\n", err);
}
