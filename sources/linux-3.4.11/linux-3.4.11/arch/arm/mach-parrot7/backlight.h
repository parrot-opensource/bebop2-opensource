/**
 * linux/arch/arm/mach-parrot7/backlight.h - Parrot7 backlight platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    23-Nov-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_BACKLIGHT_H
#define _ARCH_PARROT7_BACKLIGHT_H

#if (defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)) && \
    (defined(CONFIG_BACKLIGHT_PWM) || defined(CONFIG_BACKLIGHT_PWM_MODULE)) && \
    (defined(CONFIG_PWM_P7MU) || defined(CONFIG_PWM_P7MU_MODULE))

void __init p7_init_bkl(unsigned int bkl,
                        struct device const* fbdev,
                        int pwm_id,
                        int rst_gpio,
                        unsigned int en_gpio,
                        unsigned int ns,
                        unsigned int max,
                        unsigned int min);

#else

static inline void __init p7_init_bkl(unsigned int bkl,
                                      struct device const* fbdev,
                                      int pwm_id,
                                      int rst_gpio,
                                      unsigned int en_gpio,
                                      unsigned int ns,
                                      unsigned int max,
                                      unsigned int min)
{
	return;
}

#endif

#endif
