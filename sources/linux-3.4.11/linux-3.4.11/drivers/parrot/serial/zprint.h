/**
 * linux/drivers/parrot/zprint.h - Parrot7 ZPrint Zebu transactor
 *                                 interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    20-Aug-2012
 *
 * This file is released under the GPL
 */

#ifndef _ZPRINT_H
#define _ZPRINT_H

#if defined(CONFIG_SERIAL_ZPRINT) || \
    defined(CONFIG_SERIAL_ZPRINT_MODULE)

#define ZPRINT_DRV_NAME "zprint"

#endif /* defined(CONFIG_SERIAL_ZPRINT) || \
          defined(CONFIG_SERIAL_ZPRINT_MODULE) */

#endif
