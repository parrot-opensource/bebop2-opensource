/**
 * linux/arch/arm/mach-parrot7/vdec.c - Parrot7 video decoder platform
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
#include <uio/hx270-vdec.h>
#include "common.h"
#include "clock.h"

static struct resource p7_vdec_res[] = {
	[0] = {
		.start  = P7_VDEC,
		.end    = P7_VDEC + SZ_4K - 1,
		.flags  = IORESOURCE_MEM
	},
	[1] = {
		.start  = P7_VDEC_IRQ,
		.end    = P7_VDEC_IRQ,
		.flags  = IORESOURCE_IRQ
	}
};

static u64 p7_vdec_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7_vdec_dev = {
	.name           = HX270_DRV_NAME,
	.id             = 0,
	.resource       = p7_vdec_res,
	.num_resources  = ARRAY_SIZE(p7_vdec_res),
	.dev            = {
		.dma_mask           = &p7_vdec_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/**
 * p7_reserve_vdecmem() - Reserve DMA memory region for video decoder usage.
 *
 * @size: size of DMA region in bytes
 */
void __init p7_reserve_vdecmem(size_t size)
{
	p7_reserve_devmem(&p7_vdec_dev, NULL, &size);
}

/**
 * p7_init_vdec() - Instantiate video decoder for further driver usage.
 */
void __init p7_init_vdec(void)
{
	int err;

	BUG_ON(! p7_vdec_dev.num_resources);

	err = p7_init_dev(&p7_vdec_dev, NULL, NULL, 0);
	if (err)
		panic("p7: failed to init vdec (%d)\n", err);
}
