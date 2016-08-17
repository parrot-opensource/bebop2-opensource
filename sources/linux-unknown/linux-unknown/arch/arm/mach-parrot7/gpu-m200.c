/**
 * linux/arch/arm/mach-parrot7/gpu-m400.c - Parrot7 MALI200 GPU platform interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    28-05-2013
 *
 * This file is released under the GPL
 */

#if defined(CONFIG_MALI200) || \
    defined(CONFIG_MALI200_MODULE)

#include <mach/p7.h>
#include <mach/irqs.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/ioport.h>
#include "gpu.h"
#include "gpu/mali200/linux/mali_osk_specific.h"
#include "gpu/mali200/common/mali_osk.h"
#include "gpu/mali200/include/linux/mali/mali_utgard_uk_types.h"

#ifndef USING_MMU
#define USING_MMU 1
#endif

#define GPU_BASE		P7_GPU
#define GPU_ADDR(addr)	(GPU_BASE + addr)

_mali_osk_resource_t p7mali200_arch_configuration [] =
{
	{
		.type = MALI200,
		.base = -1, /* will be set in p7_init_gpu_fb_m200 */
		//.base = GPU_ADDR(0x0000),
		.irq = P7_GPU_IRQ0,
		.description = "Mali 200 (GX525)",
		.mmu_id = 1
	},
	{
		.type = MALIGP2,
		.description = "MALI GP2",
		.base = GPU_ADDR(0x2000),
		.irq = P7_GPU_IRQ1,
		.mmu_id = 1
	},
	{
		.type = MMU,
		.base = GPU_ADDR(0x3000),
		.irq = P7_GPU_IRQ2,
		.description = "Mali MMU",
		.mmu_id = 1
	},
	{
		.type = OS_MEMORY,
		.description = "OS Memory",
		.cpu_usage_adjust = 0x00000000,
		.alloc_order = 10, /* Lowest preference for this memory */
		.size = 0x10000000, /* 256MB, to avoid OOM-killer */
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_READABLE | _MALI_PP_WRITEABLE |_MALI_GP_READABLE | _MALI_GP_WRITEABLE
	},
/* 
 * NOTE: Validation memory addresses will be setup at boot time after the AVI FB
 *       memory has been set up.
 */
	{
		.type = MEM_VALIDATION,
		.description = "AVI Framebuffer",
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_WRITEABLE | _MALI_PP_READABLE
	},
/* 
 * NOTE: We created a custom end marker for P7 architecture.
 */
	{
		.description = "End Marker",
		.base = 0xffffffff,
		.size = 0xffffffff,
		.flags = _MALI_CPU_WRITEABLE | _MALI_CPU_READABLE | _MALI_PP_WRITEABLE | _MALI_PP_READABLE
	},
};
EXPORT_SYMBOL(p7mali200_arch_configuration);


int __init p7_init_gpu_fb_m200(unsigned long fb_start, unsigned long fb_size)
{
	int ret = -1;
	int i = 0;

	if (!fb_start || !fb_size)
		return 0;

	p7mali200_arch_configuration[0].base = GPU_ADDR(0x0000);

	for (i = 0; i< ARRAY_SIZE(p7mali200_arch_configuration); i++)
	{
		if (p7mali200_arch_configuration[i].type == MEM_VALIDATION) {
			p7mali200_arch_configuration[i].base = fb_start;
			p7mali200_arch_configuration[i].size = fb_size;
			printk(KERN_INFO "Mali200 GPU Validation: 0x%08x size 0x%x\n",
			       p7mali200_arch_configuration[i].base,
			       p7mali200_arch_configuration[i].size);
			ret = 0;
			break;
		}
	}
	if ( i == ARRAY_SIZE(p7mali200_arch_configuration)) {
		printk(KERN_ERR "Error initializing Mali200 GPU Validation range\n");
		ret = -1;
	}
	return ret;
}

#endif /* defined(CONFIG_MALI200) || \
	  defined(CONFIG_MALI200_MODULE) */


