/**
 * linux/arch/arm/mach-parrot7/ether.c - Parrot7 Ethernet platform interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Jimmy Perchet <jimmy.perchet@parrot.com>
 * date:    27-08-2013
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_ETH_H
#define _ARCH_PARROT7_ETH_H

#if defined(CONFIG_STMMAC_ETH) || \
    defined(CONFIG_STMMAC_ETH_MODULE)

#include <linux/init.h>

enum phy_iface {
	PHY_IFACE_MII,
	PHY_IFACE_RGMII,
	PHY_IFACE_MIILITE,
	PHY_IFACE_SSMII,
};

void __init p7_init_ether(enum phy_iface iface,
			  int phy_reset_gpio,
			  unsigned long drive_strength);

#else /*def CONFIG_STMMAC_ETH*/

#define p7_init_ether(_iface,_phy_reset_gpio,_drive_strength) do {} while (0)

#endif /*def CONFIG_STMMAC_ETH*/

#endif
