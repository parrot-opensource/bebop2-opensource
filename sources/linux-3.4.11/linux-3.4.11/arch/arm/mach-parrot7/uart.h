/**
 * linux/arch/arm/mach-parrot7/uart.h - Parrot7 serial port platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    13-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_SERIAL_H
#define _ARCH_PARROT7_SERIAL_H

struct pinctrl_map;

#if defined(CONFIG_SERIAL_PARROTX) || \
    defined(CONFIG_SERIAL_PARROTX_MODULE)

#include <linux/init.h>

extern void p7_init_uart(int,
                         struct pinctrl_map*,
                         size_t) __init;

#else  /* defined(CONFIG_SERIAL_PARROTX) || \
          defined(CONFIG_SERIAL_PARROTX_MODULE) */

#define p7_init_uart(_port, _pins, _pins_nr)

#endif /* defined(CONFIG_SERIAL_PARROTX) || \
          defined(CONFIG_SERIAL_PARROTX_MODULE) */

#endif
