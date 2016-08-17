/**
 * linux/arch/arm/mach-parrot7/gpu-m400.c - Parrot7 MALI400 GPU platform interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    28-05-2013
 *
 * This file is released under the GPL
 */

#if defined(CONFIG_MALI400) || \
    defined(CONFIG_MALI400_MODULE)

#include <mach/p7.h>
#include <mach/irqs.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include "gpu/mali400/include/linux/mali/mali_utgard_uk_types.h"
#include "gpu/mali400/include/linux/mali/mali_utgard.h"
#include "gpu.h"
#include "common.h"
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

static struct mali_gpu_device_data mali_gpu_data = {
	/* Mali OS memory limit */
	.shared_mem_size = 512 * 1024 * 1024, /* 512MB */

	/* Framebuffer memory */
	/* This will be set after the AVI FB has been set
	.fb_start = <base address of contigous frame buffer memory>,
	.fb_size = <size of frame buffer memory>
	*/

	/* DVFS */
	//.utilization_interval = 1000, /* ms */
	//.utilization_handler = <utilization function>,
	
};

static struct resource mali_gpu_resources[] = {
MALI_GPU_RESOURCES_MALI400_MP4(P7_GPU,
			P7_GPU_GP_IRQ,
			P7_GPU_GPMMU_IRQ,
			P7_GPU_PP0_IRQ,
			P7_GPU_PPMMU0_IRQ,
			P7_GPU_PP1_IRQ,
			P7_GPU_PPMMU1_IRQ,
			P7_GPU_PP2_IRQ,
			P7_GPU_PPMMU2_IRQ,
			P7_GPU_PP3_IRQ,
			P7_GPU_PPMMU3_IRQ)
};

static struct resource mali_gpu_resources_3pix[] = {
MALI_GPU_RESOURCES_MALI400_MP3(P7_GPU,
			P7_GPU_GP_IRQ,
			P7_GPU_GPMMU_IRQ,
			P7_GPU_PP0_IRQ,
			P7_GPU_PPMMU0_IRQ,
			P7_GPU_PP1_IRQ,
			P7_GPU_PPMMU1_IRQ,
			P7_GPU_PP2_IRQ,
			P7_GPU_PPMMU2_IRQ)
};

static struct resource mali_gpu_resources_2pix[] = {
MALI_GPU_RESOURCES_MALI400_MP2(P7_GPU,
			P7_GPU_GP_IRQ,
			P7_GPU_GPMMU_IRQ,
			P7_GPU_PP0_IRQ,
			P7_GPU_PPMMU0_IRQ,
			P7_GPU_PP1_IRQ,
			P7_GPU_PPMMU1_IRQ)
};

static struct resource mali_gpu_resources_1pix[] = {
MALI_GPU_RESOURCES_MALI400_MP1(P7_GPU,
			P7_GPU_GP_IRQ,
			P7_GPU_GPMMU_IRQ,
			P7_GPU_PP0_IRQ,
			P7_GPU_PPMMU0_IRQ)
};

struct platform_device mali_gpu_device = {
	.name = MALI_GPU_NAME_UTGARD,
	.id = 0,
	.num_resources = ARRAY_SIZE(mali_gpu_resources),
	.resource = mali_gpu_resources,
	.dev.platform_data = &mali_gpu_data,
	.dev.coherent_dma_mask	= DMA_BIT_MASK(32),
};

static int __init p7_init_gpu_m400(void) 
{
	int ret = 0;

	/* We have Mali400 only on Revision 3. */
	if (P7_CHIPREV_R3 != p7_chiprev())
		return 0;

	ret = platform_device_register(&mali_gpu_device);
	if (ret) {
		printk(KERN_ERR "Error registering Mali400 GPU device: %d.\n", ret);
		goto out;
#ifdef CONFIG_PM_RUNTIME
	} else {
		pm_runtime_set_autosuspend_delay(&(mali_gpu_device.dev), 1000);
		pm_runtime_use_autosuspend(&(mali_gpu_device.dev));
		pm_runtime_enable(&(mali_gpu_device.dev));
#endif
	}
	printk(KERN_INFO"Mali400 GPU registered.\n");

out:
	return ret;
}

int __init p7_init_gpu_fb_m400(unsigned long fb_start, unsigned long fb_size, int nb_pixcore)
{
	if (nb_pixcore == 1) {
		mali_gpu_device.num_resources = ARRAY_SIZE(mali_gpu_resources_1pix);
		mali_gpu_device.resource = mali_gpu_resources_1pix;
	}
	else if (nb_pixcore == 2) {
		mali_gpu_device.num_resources = ARRAY_SIZE(mali_gpu_resources_2pix);
		mali_gpu_device.resource = mali_gpu_resources_2pix;
	}
	else if (nb_pixcore == 3) {
		mali_gpu_device.num_resources = ARRAY_SIZE(mali_gpu_resources_3pix);
		mali_gpu_device.resource = mali_gpu_resources_3pix;
	}

	/* Setup FB start and size according to given resource */
	if (fb_start && fb_size) {
		mali_gpu_data.fb_start = fb_start;
		mali_gpu_data.fb_size  = fb_size;

		printk(KERN_INFO"Mali400 GPU Framebuffer:  0x%08lx size 0x%lx\n", 
				mali_gpu_data.fb_start,
				mali_gpu_data.fb_size);
	}

	return p7_init_gpu_m400();
}

#endif /* defined(CONFIG_MALI400) || \
	  defined(CONFIG_MALI400_MODULE) */

