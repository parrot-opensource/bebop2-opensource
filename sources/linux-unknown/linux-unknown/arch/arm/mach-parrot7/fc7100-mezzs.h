/*
 * arch/arm/mach-parrot7/fc7100-mezz.h - common structures and declaration
 *                                        for mezzanines of the FC7100 family.
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    2013-07
 *
 * This file is released under the GPL.
 */

#ifndef _ARCH_MEZZS_FC71XX_H
#define _ARCH_MEZZS_FC71XX_H

#include "fc7100-module.h"

#define FC7100_DAIFMT (SND_SOC_DAIFMT_I2S       | \
		       SND_SOC_DAIFMT_NB_NF     | \
		       SND_SOC_DAIFMT_CBS_CFS)

int fc7100_mezz_get_rev(void);

int __init fc7100_init_mezz(unsigned int mod_settings, const char *screen_name);
void __init fc7100_mezz_reserve_mem(void);

#endif /* _ARCH_MEZZS_FC71XX_H */
