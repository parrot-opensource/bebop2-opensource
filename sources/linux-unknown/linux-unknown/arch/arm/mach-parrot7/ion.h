/**
 * linux/arch/arm/mach-parrot7/ion.h - Parrot7 ION allocator platform
 *                                     interface
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

#ifndef _ARCH_PARROT7_ION_H
#define _ARCH_PARROT7_ION_H

#if defined(CONFIG_ION) || \
    defined(CONFIG_ION_MODULE)

#include <linux/init.h>

struct platform_device;

extern void p7_init_ion(void) __init;
extern void p7_ionize_dev(struct platform_device*, unsigned int) __init;

#else  /* defined(CONFIG_ION) || \
          defined(CONFIG_ION_MODULE) */

#define p7_init_ion()
#define p7_ionize_dev(_pdev)

#endif /* defined(CONFIG_ION) || \
          defined(CONFIG_ION_MODULE) */

#endif
