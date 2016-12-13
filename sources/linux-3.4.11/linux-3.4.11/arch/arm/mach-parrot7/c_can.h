/**
 * linux/arch/arm/mach-parrot7/c_can.h - Parrot7 c_can IP Module slave controller platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Karl Leplat <karl.leplat@parrot.com>
 * date:    14-Feb-2014
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_C_CAN_H
#define _ARCH_PARROT7_C_CAN_H

struct pinctrl_map;

#if defined(CONFIG_CAN_C_CAN_PLATFORM) || \
    defined(CONFIG_CAN_C_CAN_PLATFORM_MODULE)

#include <linux/init.h>

extern void p7_init_c_can(int id,
		struct pinctrl_map* pins,
		size_t pin_cnt) __init;

#else  /* defined(CONFIG_CAN_C_CAN_PLATFORM) || \
          defined(CONFIG_CAN_C_CAN_PLATFORM_MODULE) */

static inline void p7_init_c_can(int id,
		struct pinctrl_map *pins,
		size_t pin_cnt) {}

#endif /* defined(CONFIG_CAN_C_CAN_PLATFORM) || \
          defined(CONFIG_CAN_C_CAN_PLATFORM_MODULE) */

#endif
