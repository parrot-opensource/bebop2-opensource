/**
 * linux/arch/arm/mach-parrot7/nand.h - Nand controller platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    15-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_NAND_H
#define _ARCH_PARROT7_NAND_H

struct cast_plat_data;
struct pinctrl_map;

#if defined(CONFIG_MTD_NAND_CAST) || \
    defined(CONFIG_MTD_NAND_CAST_MODULE)

#include <linux/init.h>

extern void p7_init_nand(struct cast_plat_data*,
                         struct pinctrl_map*,
                         size_t) __init;
extern void p7_reserve_nand_mem(void);
#else  /* defined(CONFIG_MTD_NAND_CAST) || \
          defined(CONFIG_MTD_NAND_CAST_MODULE) */

#define p7_init_nand(_pdata, _pins, _pins_nr)
#define p7_reserve_nand_mem()

#endif /* defined(CONFIG_MTD_NAND_CAST) || \
          defined(CONFIG_MTD_NAND_CAST_MODULE) */

#endif
