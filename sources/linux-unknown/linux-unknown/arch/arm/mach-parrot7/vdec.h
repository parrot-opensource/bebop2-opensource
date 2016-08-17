/**
 * linux/arch/arm/mach-parrot7/vdec.h - Parrot7 video decoder platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    18-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_VDEC_H
#define _ARCH_PARROT7_VDEC_H

#if defined(CONFIG_UIO_HX270) || \
    defined(CONFIG_UIO_HX270_MODULE)

#include <linux/init.h>

extern void p7_init_vdec(void) __init;
extern void p7_reserve_vdecmem(size_t) __init;

#else  /* defined(CONFIG_UIO_HX270) || \
          defined(CONFIG_UIO_HX270_MODULE) */

#define p7_init_vdec()
#define p7_reserve_vdecmem(_size)

#endif /* defined(CONFIG_UIO_HX270) || \
          defined(CONFIG_UIO_HX270_MODULE) */

#endif
