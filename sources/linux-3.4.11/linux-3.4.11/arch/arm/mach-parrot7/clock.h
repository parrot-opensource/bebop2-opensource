/*
 *  linux/arch/arm/mach-parrot7/clock.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  05-Nov-2010
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _ARCH_PARROT7_CLOCK_H
#define _ARCH_PARROT7_CLOCK_H

#include <mach/clkdev.h>
#include "gbc.h"


#define P7_PLL_CFG(divf, divr, divq, range)         \
	((((range) & 7) << 17) | (((divq) & 7) << 14)	\
	 | (((divr) & 0x3f) << 8) | (divf & 0xff))

/* helper to fill a clk_div_table */
#define CLK_DIV(_val, _div) { .val = _val, .div = _div }


#ifdef CONFIG_NEON
#include <mach/p7.h>

static inline void p7_enable_neonclk(unsigned int cpu_id)
{
	unsigned long const off = (! cpu_id) ? CPU_GBC_NEON0_RESET :
	                          CPU_GBC_NEON1_RESET;
	BUG_ON(cpu_id > 1);

	/* De-reset NEON unit. */
	__raw_writel(0, __MMIO_P2V(P7_CPU_GBC) + off);
	/* wait ready */
	while (!(__raw_readl(__MMIO_P2V(P7_CPU_GBC) + off) & 2));
}

static inline void p7_disable_neonclk(unsigned int cpu_id)
{
	unsigned long const off = (! cpu_id) ? CPU_GBC_NEON0_RESET :
	                          CPU_GBC_NEON1_RESET;
	BUG_ON(cpu_id > 1);

	/* Put NEON unit into resetting state. */
	__raw_writel(1UL, __MMIO_P2V(P7_CPU_GBC) + off);
}
#else
#define p7_enable_neonclk(_cpu_id)
#define p7_disable_neonclk(_cpu_id)
#endif

extern void p7_init_clk(int low_sysclk) __init;

#endif
