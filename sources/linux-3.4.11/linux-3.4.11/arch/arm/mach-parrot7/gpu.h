/**
 * linux/arch/arm/mach-parrot7/gpu.h - Parrot7 MALI200 GPU platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    01-11-2012
 *
 * This file is released under the GPL
 */

#include "gpu/p7gpu-pm.h"

#if defined(CONFIG_GPU_PARROT7) || \
    defined(CONFIG_GPU_PARROT7_MODULE)
int p7_init_gpu_fb(unsigned long fb_start, unsigned long fb_size, int nb_pixcore) __init;

#else
#define p7_init_gpu_fb(_fb_start, _fb_size, nb_pixcore) \
	({ -ENOSYS; })
#endif


#if defined(CONFIG_MALI200) || \
    defined(CONFIG_MALI200_MODULE)
int p7_init_gpu_fb_m200(unsigned long fb_start, unsigned long fb_size) __init;
#else
#define p7_init_gpu_fb_m200(_fb_start, _fb_size) \
	({ -ENOSYS; })
#endif


#if defined(CONFIG_MALI400) || \
    defined(CONFIG_MALI400_MODULE)
int p7_init_gpu_fb_m400(unsigned long fb_start, unsigned long fb_size, int nb_pixcore) __init;
#else
#define p7_init_gpu_fb_m400(_fb_start, _fb_size, nb_pixcore) \
	({ -ENOSYS; })
#endif

