/**
 * linux/drivers/parrot/pwm/p7-pwm.h - Parrot7 PWM driver interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author: Victor Lambret <victor.lambret.ext@parrot.com>
 * date:    28-Nov-2012
 *
 * This file is released under the GPL
 *
 */

#ifndef P7PWM_H
#define P7PWM_H

#include <mach/pwm.h>

#define P7PWM_NUMBER   (P7_PWMS_MAX)

#define P7PWM_DRV_NAME  "p7_pwm"

#define P7PWM_MODE_NORMAL       0
#define P7PWM_MODE_CLOCK        1
#define P7PWM_MODE_PPM_RX       2

/* period_precision :   a per cent value for the max error rate accepted on
 *                      frequency
 * duty_precision :     a per cent value for the max error rate accepted on
 *                      duty cycle (XXX this not really a percent)
 * mode :               P7PWM_MODE_NORMAL for normal PWM mode
 *                      P7PWM_MODE_CLOCK for clock mode
 *  */

struct p7pwm_conf {
	int16_t         period_precision ;
	int16_t         duty_precision ;
	uint8_t         mode ;
} ;

struct p7pwm_pdata {
	size_t                  used ;
	struct p7pwm_conf *    conf[P7PWM_NUMBER] ;
} ;

#endif /*P7PWM_H*/
