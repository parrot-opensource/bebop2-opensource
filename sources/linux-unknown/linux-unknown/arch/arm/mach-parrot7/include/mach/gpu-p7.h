/**
 * linux/arch/arm/mach-parrot7/include/mach/gpu-p7.h - Parrot7 MALI200 GPU 
 *                                                     platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    20-10-2012
 *
 * This file is released under the GPL
 */

#ifndef __ARCH_PARROT7_GPU_H__
#define __ARCH_PARROT7_GPU_H__

#if defined(CONFIG_MALI200) || \
    defined(CONFIG_MALI200_MODULE)

#include "mali_osk_specific.h"
#include "mali_osk.h"

extern _mali_osk_resource_t p7mali200_arch_configuration [];
#define arch_configuration p7mali200_arch_configuration

#endif /* defined(CONFIG_MALI200) || \
	  defined(CONFIG_MALI200_MODULE) */

#endif /* __ARCH_PARROT7_GPU_H__ */
