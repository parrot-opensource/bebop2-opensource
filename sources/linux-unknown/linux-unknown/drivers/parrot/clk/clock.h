/*
 *  drivers/parrot/clk/clock.h
 *
 *  Copyright (C) 2010-2013 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @author  Damien Riegel <damien.riegel.ext@parrot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _P7_CLOCK_COMMON_H
#define _P7_CLOCK_COMMON_H

#include "p7clk-provider.h"

/**
 * union p7_pll_reg - PLL register descriptor.
 */
union p7_pll_reg {
	u32 word;
	struct {
		unsigned    cfg:20;
		unsigned    enable:1;
		unsigned    bypass:1;
		unsigned    reset:1;
	} fields;
};

int p7_get_clkcfg(unsigned long const* config, unsigned int frequency, size_t count);
int p7_activate_pll(unsigned long status_reg, u32 status_msk);
unsigned long p7_pll_rate(union p7_pll_reg cfg, unsigned long in_hz);
void p7_write_pll_conf(unsigned long addr, u32 val);

#endif
