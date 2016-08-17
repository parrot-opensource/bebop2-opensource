/**
 * linux/driver/parrot/block/phy-brd.h - Physical Block RamDisk interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    18-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _PHY_BRD_H
#define _PHY_BRD_H

#if defined(CONFIG_BLK_DEV_PHYBRD) || \
    defined(CONFIG_BLK_DEV_PHYBRD_MODULE)

#define PHYBRD_DRV_NAME "phy-brd"

#endif

#endif
