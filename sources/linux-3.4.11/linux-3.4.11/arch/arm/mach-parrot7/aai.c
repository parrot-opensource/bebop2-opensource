/**
 * linux/arch/arm/mach-parrot7/aai.c - Parrot7 aai platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Samir Ammenouche <samir.Ammenouche@parrot.com>
 * date:    28-Sep-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <sound/p7_aai/reg_aai_gbc.h>
#include "common.h"
#include "clock.h"
#include "aai.h"

#if defined(CONFIG_SND_AAI) ||  defined(CONFIG_SND_AAI_MODULE)

static struct resource p7_aai_resource[] = {
	[0] = {
		.start  = P7_AAI,
		.end    = P7_AAI + SZ_1M - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = P7_AAI_IRQ,
		.end    = P7_AAI_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

struct platform_device p7_aai_device = {
	.name		= "aai",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(p7_aai_resource),
	.resource	= p7_aai_resource,
};

void __init p7_init_aai(struct pinctrl_map* pins,
			size_t size,
			struct aai_platform_data *pdata)
{
	if (p7_aai_device.num_resources)
		p7_init_dev(& p7_aai_device, pdata, pins, size);
	pdata->chiprev =  p7_chiprev();
}

#endif 
