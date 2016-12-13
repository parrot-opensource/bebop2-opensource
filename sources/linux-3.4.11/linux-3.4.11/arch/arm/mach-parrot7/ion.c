/*
 * linux/arch/arm/mach-parrot7/ion.c - Parrot7 ION allocator platform
 *                                     implementation
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    14-Jan-2014
 *
 * This file is released under the GPL
 *
 * See linux/drivers/parrot/gpu/ion/p7-ion.c
 */

#include <linux/platform_device.h>
#include <linux/ion.h>
#include "ion.h"

#define P7ION_DRV_NAME      "p7-ion"
#define P7_ION_HEAPS_MAX    16

static struct ion_platform_heap p7_ion_pheaps[P7_ION_HEAPS_MAX] = {
	{ /* vmalloc based */
		.type   = ION_HEAP_TYPE_SYSTEM,
		.id     = ION_HEAP_TYPE_SYSTEM,
		.name   = "vmalloc"
	},
	{ /* kmalloc based */
		.type   = ION_HEAP_TYPE_SYSTEM_CONTIG,
		.id     = ION_HEAP_TYPE_SYSTEM_CONTIG,
		.name   = "kmalloc"
	},
	{ /* global DMA coherent / CMA based */
		.type   = ION_HEAP_TYPE_DMA,
		.id     = ION_HEAP_TYPE_DMA,
		.name   = "cma",
		.priv   = NULL
	}
};

static struct ion_platform_data p7_ion_pdata = {
	.nr     = 3,
	.heaps  = p7_ion_pheaps
};

static struct platform_device p7_ion_dev = {
	.name               = P7ION_DRV_NAME,
	.id                 = 0,
	.num_resources      = 0,
	.dev.platform_data  = &p7_ion_pdata
};

void __init p7_ionize_dev(struct platform_device* pdev, unsigned int id)
{
	struct ion_platform_heap* const heap = &p7_ion_pheaps[p7_ion_pdata.nr];

#ifdef DEBUG
	BUG_ON(pdev);
	/* dma_declare_coherent_memory MUST have been called before !! */
	BUG_ON(! pdev->dev.dma_mem);
	BUG_ON(id < ION_HEAP_TYPE_CUSTOM);
	{
		unsigned int h;
		for (h = 0; h < p7_ion_pdata.nr; h++)
			BUG_ON(id == p7_ion_pheaps[h].id);
	}
#endif

	heap->type  = ION_HEAP_TYPE_DMA;
	heap->id    = id;
	heap->name  = dev_name(&pdev->dev);
	heap->priv  = &pdev->dev;

	p7_ion_pdata.nr++;
}

void __init p7_init_ion(void)
{
	int err;

#ifdef DEBUG
	BUG_ON(! p7_ion_pdata.nr);
#endif

	pr_debug("p7: registering " P7ION_DRV_NAME ".0...\n");

	err = platform_device_register(&p7_ion_dev);
	if (err)
		panic("p7: failed to register " P7ION_DRV_NAME ".0 (%d)\n", err);
}
