/*
 *  linux/arch/arm/mach-parrot7/include/mach/timex.h
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

#ifndef _ARCH_PARROT7_TIMEX_H
#define _ARCH_PARROT7_TIMEX_H

/*
 * CLOCK_TICK_RATE is actually a don't-care value. Make it default to
 * (100 * HZ).
 */
#define CLOCK_TICK_RATE (100 * HZ)

#define ARCH_HAS_READ_CURRENT_TIMER

#endif

/* vim:set ts=4:sw=4:noet:ft=c: */
