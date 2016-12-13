/**
 * linux/drivers/parrot/mfd/p7mu-clk.h - Parrot7 power management unit internal
 *                                       clocks interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    21-Nov-2012
 *
 * This file is released under the GPL
 */

#ifndef _P7MU_CLK_H
#define _P7MU_CLK_H

#include <linux/init.h>

#if defined(CONFIG_P7MU_ADC) || defined(CONFIG_P7MU_ADC_MODULE)

#define P7MU_SPI_HZ_MIN (32000000UL)
#define P7MU_SPI_HZ_MAX (48000000UL)

extern long p7mu_get_spiclk_rate(void);

#endif

#if defined(CONFIG_PWM_P7MU) || defined(CONFIG_PWM_P7MU_MODULE)

#define P7MU_PWM_HZ_MIN (32768UL)
#define P7MU_PWM_HZ_MAX (2000000UL)

static inline long p7mu_round_pwmclk_rate(unsigned long hz)
{
	if (hz > P7MU_PWM_HZ_MAX)
		return -ERANGE;

	/* Divide by 256 to allow finer granularity when using 32kHz clock. */
	return (hz < (P7MU_PWM_HZ_MIN / 256)) ? P7MU_PWM_HZ_MIN : P7MU_PWM_HZ_MAX;
}

extern long p7mu_get_pwmclk_rate(unsigned int);
extern int  p7mu_set_pwmclk_rate(unsigned int, unsigned long);

#endif

extern void p7mu_init_clk(void);
extern void p7mu_exit_clk(void);
extern void p7mu_resume_clk(void);

#endif
