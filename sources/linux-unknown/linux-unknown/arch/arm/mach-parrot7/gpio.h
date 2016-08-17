/**
 * linux/arch/arm/mach-parrot7/gpio.h - Parrot7 gpio pin controller platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    12-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_GPIO_H
#define _ARCH_PARROT7_GPIO_H

struct p7gpio_irq_map;

#if defined(CONFIG_GPIO_PARROT7) || \
    defined(CONFIG_GPIO_PARROT7_MODULE)

#include <linux/init.h>

extern void p7_init_gpio(unsigned const*, size_t) __init;

#else  /* defined(CONFIG_GPIO_PARROT7) || \
          defined(CONFIG_GPIO_PARROT7_MODULE) */

#define p7_init_gpio(_irq_map, _irq_cnt)

#endif /* defined(CONFIG_GPIO_PARROT7) || \
          defined(CONFIG_GPIO_PARROT7_MODULE) */

#endif
