/**
 * linux/arch/arm/mach-parrot7/gpu.c - Parrot7 GPU platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    31-10-2012
 *
 * This file is released under the GPL
 */

#if defined(CONFIG_GPU_PARROT7) || \
    defined(CONFIG_GPU_PARROT7_MODULE)

#if !( defined(CONFIG_MALI200) || \
    defined(CONFIG_MALI200_MODULE) ) && \
    !( defined(CONFIG_MALI400) || \
    defined(CONFIG_MALI400_MODULE))
#error You need to select either Mali200 or Mali400 to enable the GPU.
#endif

#include "common.h"
#include "gpu.h"

static struct p7gpu_pm_plat_data p7gpu_pdata;

static struct platform_device p7gpu_pm_dev = {
	.name   = "p7gpu",
	.id     = 0
};

static char* mali200_clocks_names[] = { M200_BUS_CLK, M200_CORE_CLK }; 
static char *mali400_clocks_names[] = {	M400_CLK,
	M400_CLK"_pp0", M400_CLK"_pp1", M400_CLK"_pp2", M400_CLK"_pp3" }; 


static int p7_init_gpu_pm(char** clocks_names, int num)
{
	int ret = 0;

	/* Setup correct clock names */
	if (NULL == clocks_names)
		panic("P7 GPU Clock names not provided.");
	p7gpu_pdata.clocks_names = clocks_names;
	p7gpu_pdata.clocks_number = num;

	p7gpu_pm_dev.dev.platform_data = &p7gpu_pdata;

	/* Add power management device */
	ret = platform_device_register(&p7gpu_pm_dev);
	if (ret) {
		printk(KERN_ERR "Error registering P7 GPU device: %d.\n", ret);
		platform_device_unregister(&p7gpu_pm_dev);
		goto out;
	}
	printk(KERN_INFO"P7 GPU device PM registered.\n");

out:
	return ret;
}

/*
 * Setup GPU's framebuffer, passsing framebuffer buffer information.
 */
int __init p7_init_gpu_fb(unsigned long fb_start, unsigned long fb_size, int nb_pixcore)
{
	int ret = 0;
	int chiprev = p7_chiprev();

	/* Initialize Mali200/400 (and register platform device if needed) */
	switch(chiprev) {
		case P7_CHIPREV_R1:
		case P7_CHIPREV_R2:
			ret = p7_init_gpu_pm(mali200_clocks_names, ARRAY_SIZE(mali200_clocks_names));
			if (ret == 0)
				ret = p7_init_gpu_fb_m200(fb_start, fb_size);
			break;
		case P7_CHIPREV_R3:
			if (nb_pixcore < 0 || nb_pixcore > 4)
				ret = -1;
			/* clock should be registered before gpu driver */
			if (ret == 0)
				ret = p7_init_gpu_pm(mali400_clocks_names, ARRAY_SIZE(mali400_clocks_names) - 4 + nb_pixcore);
			if (ret == 0)
				ret = p7_init_gpu_fb_m400(fb_start, fb_size, nb_pixcore);
			break;
		default:
			ret = -1;
			printk(KERN_ERR "GPU not initialized: Unknown P7 revision.\n");
	}
	return ret;
}

#endif /* defined(CONFIG_GPU_PARROT7) || \
          defined(CONFIG_GPU_PARROT7_MODULE) */

