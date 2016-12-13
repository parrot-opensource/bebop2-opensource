/**
 * linux/arch/arm/mach-parrot7/include/mach/pwm.h - Parrot7 pwm platform
 *                                                  specific interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    22-Nov-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_PWM_H_
#define _ARCH_PARROT7_PWM_H_

/*
 * Holds current number of PWMs declared for the global space.
 * Don't use this outside of this file, this is for internal purpose only !
 */
#define __P7_FIRST_PWM__   0

#define P7_PWMS_MAX    16
#define P7MU_PWMS_MAX  2

/*
 * Define pwm base for each controller.
 * Use enum definition to force preprocessor expansion each time
 * __P7_FIRST_PWM__ is re-defined.
 */
enum {
/**********************************
 * Declare Parrot7 chip PWM lines
 **********************************/
#if defined(CONFIG_PWM_PARROT7) || defined(CONFIG_PWM_PARROT7_MODULE)
	P7_FIRST_PWM = __P7_FIRST_PWM__,
#undef __P7_FIRST_PWM__
#define __P7_FIRST_PWM__   P7_PWMS_MAX

#define P7_PWM_NR(_n) (P7_FIRST_PWM + (_n))
#else
#define P7_PWM_NR(_n) (-1)
#endif

/******************************************************************
 * Declare Parrot7 Power Management Unit companion chip PWM lines
 ******************************************************************/

#if defined(CONFIG_PWM_P7MU) || defined(CONFIG_PWM_P7MU_MODULE)
	P7MU_FIRST_PWM = __P7_FIRST_PWM__,
#undef __P7_FIRST_PWM__
#define __P7_FIRST_PWM__   (P7MU_FIRST_PWM + P7MU_PWMS_MAX)

#define P7MU_PWM_NR(_n) (P7MU_FIRST_PWM + (_n))

#else
#define P7MU_PWM_NR(_n) (-1)
#endif
	P7_FIRST_NR = __P7_FIRST_PWM__,
};

/* At last, set the total number of available PWM lines in the global space. */
#define ARCH_NR_PWMS   __P7_FIRST_PWM__

#endif /* _ARCH_PARROT7_PWM_H_ */
