/**
 * linux/arch/arm/mach-parrot7/usb.h - USB controller platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:	Guangye Tian <guangye.tian@parrot.com>
 * data:	2012-09-11
 */

#ifndef _ARCH_PARROT7_USB_H
#define _ARCH_PARROT7_USB_H

#include <linux/init.h>
#include <linux/platform_device.h>

struct pinctrl_map;

#if defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC)

struct p7_usb2_platform_data;

extern void p7_init_ci_udc(int core,
                        struct p7_usb2_platform_data*,
                        struct pinctrl_map*,
                        size_t) __init;

extern void p7_reserve_usb_mem(int core) __init;

#else /* defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC) */
#define p7_init_ci_udc(_core, _pdata, _pins, _pins_nr)
#define p7_reserve_usb_mem(core)
#endif /* defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC) */

#endif //_ARCH_PARROT7_USB_H
