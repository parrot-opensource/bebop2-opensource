/**
 * linux/arch/arm/mach-parrot7/sdhci.h - Parrot7 eMMC / SD / SDIO controller
 *                                       platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    14-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_SDHCI_H
#define _ARCH_PARROT7_SDHCI_H

struct acs3_plat_data;
struct pinctrl_map;

#include <linux/init.h>
#include <mmc/acs3-sdhci.h>


#if defined(CONFIG_MMC_SDHCI_ACS3) || \
    defined(CONFIG_MMC_SDHCI_ACS3_MODULE)

extern void p7_init_sdhci(int,
                         struct acs3_plat_data*,
                         struct pinctrl_map*,
                         size_t) __init;

#else  /* defined(CONFIG_MMC_PARROT7) || \
          defined(CONFIG_MMC_PARROT7_MODULE) */

#define p7_init_sdhci(_host, _pdata, _pins, _pins_nr)

#endif  /* defined(CONFIG_MMC_SDHCI_ACS3) || \
           defined(CONFIG_MMC_SDHCI_ACS3_MODULE) */

#endif
