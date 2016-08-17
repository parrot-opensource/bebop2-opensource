/**
 * linux/arch/arm/mach-parrot7/p7mu.h - Parrot7 power management unit platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    26-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_P7MU_H
#define _ARCH_PARROT7_P7MU_H

struct p7mu_plat_data;
struct pinctrl_map;

#if defined(CONFIG_MFD_P7MU) || \
    defined(CONFIG_MFD_P7MU_MODULE)

#include <linux/init.h>

extern void p7_init_p7mu(int,
                         struct p7mu_plat_data*,
                         struct pinctrl_map*,
                         size_t) __init;

#else  /* defined(CONFIG_MFD_P7MU) || \
          defined(CONFIG_MFD_P7MU_MODULE) */

#define p7_init_p7mu(_bus, _pdata, _pins, _pins_nr)

#endif /* defined(CONFIG_MFD_P7MU) || \
          defined(CONFIG_MFD_P7MU_MODULE) */

#endif
