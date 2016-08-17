/**
 * linux/arch/arm/mach-parrot7/i2cs.h - Parrot7 i2c slave controller platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    13-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_I2CS_H
#define _ARCH_PARROT7_I2CS_H

struct plds_i2cs_pdata;
struct pinctrl_map;

#if defined(CONFIG_PLDS_I2CS) || \
    defined(CONFIG_PLDS_I2CS_MODULE)

#include <linux/init.h>

extern void p7_init_i2cs(struct plds_i2cs_pdata*,
                         struct pinctrl_map*,
                         size_t) __init;

#else  /* defined(CONFIG_PLDS_I2CS) || \
          defined(CONFIG_PLDS_I2CS_MODULE) */

#define p7_init_i2cs(_pdata, _pins, _pins_nr)

#endif /* defined(CONFIG_PLDS_I2CS) || \
          defined(CONFIG_PLDS_I2CS_MODULE) */

#endif
