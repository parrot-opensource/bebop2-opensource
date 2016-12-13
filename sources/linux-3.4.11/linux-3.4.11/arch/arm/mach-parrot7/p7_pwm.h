/**
 * linux/arch/arm/mach-parrot7/p7pwm.h - Parrot7 PWM platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author: Victor Lambret <victor.lambret.ext@parrot.com>
 * date:    29-Nov-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_P7PWM_H
#define _ARCH_PARROT7_P7PWM_H

#include <linux/init.h>
#include <linux/pinctrl/machine.h>
#include <pwm/p7_pwm.h>

#if defined(CONFIG_PWM_PARROT7) || \
    defined(CONFIG_PWM_PARROT7_MODULE)

void __init p7_init_p7pwm(struct p7pwm_pdata *pdata,
                         struct pinctrl_map* pins,
                         size_t pin_cnt) ;

#else  /* defined(CONFIG_PWM_PARROT7) || \
          defined(CONFIG_PWM_PARROT7_MODULE) */

#define p7_init_p7pwm(...)

#endif /* defined(CONFIG_PWM_PARROT7) || \
          defined(CONFIG_PWM_PARROT7_MODULE) */

#endif /*_ARCH_PARROT7_P7PWM_H*/
