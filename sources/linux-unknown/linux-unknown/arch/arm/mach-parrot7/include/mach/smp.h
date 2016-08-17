/*
 *  linux/arch/arm/mach-parrot7/include/mach/smp.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  28-Oct-2010
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

#ifndef _ARCH_PARROT7_SMP_H
#define _ARCH_PARROT7_SMP_H

#include <asm/hardware/gic.h>

#define hard_smp_processor_id()				\
	({										\
		unsigned int cpunum;				\
		__asm__("mrc p15, 0, %0, c0, c0, 5"	\
				: "=r" (cpunum));			\
		cpunum &= 0x0f;						\
	})

/*
 * We use IRQ1 as the IPI (Inter-Processor Interrupts).
 */
static inline void smp_cross_call(const struct cpumask *mask)
{
	gic_raise_softirq(mask, 1);
}

#endif

/* vim:set ts=4:sw=4:noet:ft=c: */
