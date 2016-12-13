/**
 * linux/arch/arm/mach-parrot7/venc.h - Parrot7 video encoder platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    18-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_VENC_H
#define _ARCH_PARROT7_VENC_H

#if defined(CONFIG_UIO_HX280) || \
    defined(CONFIG_UIO_HX280_MODULE)

#include <linux/init.h>
extern struct platform_device p7_venc_dev;
extern void p7_init_venc(void) __init;
extern void p7_reserve_vencmem(size_t) __init;

#else  /* defined(CONFIG_UIO_HX280) || \
          defined(CONFIG_UIO_HX280_MODULE) */

#define p7_init_venc()
#define p7_reserve_vencmem(_size)

#endif /* defined(CONFIG_UIO_HX280) || \
          defined(CONFIG_UIO_HX280_MODULE) */

#endif
