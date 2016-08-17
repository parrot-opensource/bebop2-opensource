/*
 *  drivers/parrot/clk/clock.c
 *
 *  Copyright (C) 2010-2013 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/sched.h>
#include "clock.h"


/**
 * p7_get_clkcfg() - Get clock frequency setup descriptor index for the frequency
 * passed in argument.
 * @config: a table of clock frequency descriptor sorted in strict ascending
 * order.
 * @frequency: frequency in HZ
 * @count: clock frequency table number of element
 *
 * @p7_get_clkcfg is used to find the best configuration of a divisor
 * (or PLL) amongst possible frequency it may handle.
 */
int p7_get_clkcfg(unsigned long const* config, unsigned int frequency,
			 size_t count)
{
	unsigned int    min = 0;
	unsigned int    max = count - 1;

	BUG_ON(! config);
	BUG_ON(! frequency);
	BUG_ON(! count);

	if (frequency < config[0])
		return -ERANGE;

	while (max - min > 1) {
		unsigned int const idx = (min + max) / 2;

		if (frequency == config[idx])
			return idx;
		else if (frequency > config[idx])
			min = idx;
		else
			max = idx;
	}

	if (frequency > config[max])
		return max;
	else if ((config[max] - frequency) < (frequency - config[min]))
		return max;
	else
		return min;
}

/**
 * p7_activate_pll() - Enable PLL
 * @reg_addr: PLL setup register virtual address
 * @config: configuration word used to setup PLL
 * status_msk: mask used to poll for PLL ready condition
 */
int p7_activate_pll(unsigned long status_reg, u32 status_msk)
{
	unsigned long const timeout = jiffies + 10;

	do {
		if (readl(status_reg) & status_msk)
			return 0;

		cpu_relax();
	} while (time_before(jiffies, timeout));

	return -ETIME;
}

unsigned long p7_pll_rate(union p7_pll_reg cfg, unsigned long in_hz)
{
	if (cfg.fields.bypass)
		return in_hz;

	return (((in_hz / 1000 *
	          ((cfg.fields.cfg & ((1UL << 8) - 1)) + 1))         /* DIVf + 1 */
		 /
		 (((cfg.fields.cfg >> 8) & ((1UL << 6) - 1)) + 1))   /* DIVr + 1 */
		>> ((cfg.fields.cfg >> 14) & ((1UL << 3) - 1)))      /* 2 ^ DIVq */
		* 1000;
}

void p7_write_pll_conf(unsigned long address, u32 val)
{
	writel(val, address);
	wmb();
}
