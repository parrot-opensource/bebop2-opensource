/*
 *  linux/arch/arm/mach-parrot7/core.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Samir Ammenouche <samir.ammenouche@parrot.com>
 * @date  12-Feb-2013
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


#ifndef _ARCH_PARROT7_CORE_H
#define _ARCH_PARROT7_CORE_H

/******************************
 * Cache L2 setting declaration
 ******************************/

#define L2X0_REFERENCE_CLK_RATE_R1 (195 * 1000 * 1000)
/* L2X0_REFERENCE_CLK_RATE_R2 is not already known */
#define L2X0_REFERENCE_CLK_RATE_R2 (195 * 1000 * 1000)
/* Ditto */
#define L2X0_REFERENCE_CLK_RATE_R3 (195 * 1000 * 1000)
#define L2X0_RAM_LAT_LOW 0x010
#define L2X0_RAM_LAT_HIGH 0x121
#define CPU_GBC_CLK_MASK_R1 0x1
#define CPU_GBC_CLK_MASK_R2 0x3
#define CPU_GBC_CLK_MASK_R3 0x3
#define CPU_GBC_DATACLK_SHIFT_R1 0x0
#define CPU_GBC_DATACLK_SHIFT_R2 0x0
#define CPU_GBC_DATACLK_SHIFT_R3 0x0
#define CPU_GBC_TAGCLK_SHIFT_R1 0x1
#define CPU_GBC_TAGCLK_SHIFT_R2 0x8
#define CPU_GBC_TAGCLK_SHIFT_R3 0x8

extern int p7_is_nonsecure(void);
#endif
