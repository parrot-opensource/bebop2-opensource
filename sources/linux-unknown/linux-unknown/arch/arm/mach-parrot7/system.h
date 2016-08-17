/*
 *  linux/arch/arm/mach-parrot7/system.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  08-Nov-2010
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

#ifndef _ARCH_PARROT7_SYSTEM_H
#define _ARCH_PARROT7_SYSTEM_H

#include <mach/p7.h>

/* Clocks generation. */
#define P7_SYS_CLKGEN				P7_SYS
#define P7_SYS_CLKGEN_STATUS		P7_SYS_CLKGEN
#define P7_SYS_CLKGEN_CFG_CPU_DIV	(P7_SYS_CLKGEN + UL(0x04))
#define P7_SYS_CLKGEN_CFG_SYS_DIV	(P7_SYS_CLKGEN + UL(0x08))
#define P7_SYS_CLKGEN_CFG_USB_DIV	(P7_SYS_CLKGEN + UL(0x0c))
#define P7_SYS_CLKGEN_CFG_CAN_DIV	(P7_SYS_CLKGEN + UL(0x10))
#define P7_SYS_CLKGEN_CFG_FAST		(P7_SYS_CLKGEN + UL(0x14))
#define P7_SYS_CLKGEN_CFG_DDR		(P7_SYS_CLKGEN + UL(0x18))
#define P7_SYS_CLKGEN_CFG_AVI00		(P7_SYS_CLKGEN + UL(0x1c))
#define P7_SYS_CLKGEN_CFG_AVI01		(P7_SYS_CLKGEN + UL(0x20))
#define P7_SYS_CLKGEN_CFG_AVI10		(P7_SYS_CLKGEN + UL(0x24))
#define P7_SYS_CLKGEN_CFG_AVI11		(P7_SYS_CLKGEN + UL(0x28))
#define P7_SYS_CLKGEN_CFG_AAI		(P7_SYS_CLKGEN + UL(0x2c))
#define P7_SYS_CLKGEN_CFG_NAND		(P7_SYS_CLKGEN + UL(0x30))
#define P7_SYS_CLKGEN_CFG_USB       (P7_SYS_CLKGEN + UL(0x34))
#define P7_SYS_CLKGEN_CFG_TRACE		(P7_SYS_CLKGEN + UL(0x38))
#define P7_SYS_CLKGEN_CFG_ETHERNET		(P7_SYS_CLKGEN + UL(0x3c))

/* Multiplexed I/O setup. */
#define P7_SYS_PADCTRL_IO    (P7_SYS + UL(0x1000))
#define P7_SYS_PADCTRL_IO_00 (P7_SYS_PADCTRL_IO + UL(4 * 0))
#define P7_SYS_PADCTRL_IO_01 (P7_SYS_PADCTRL_IO + UL(4 * 1))
#define P7_SYS_PADCTRL_IO_02 (P7_SYS_PADCTRL_IO + UL(4 * 2))
#define P7_SYS_PADCTRL_IO_03 (P7_SYS_PADCTRL_IO + UL(4 * 3))
#define P7_SYS_PADCTRL_IO_04 (P7_SYS_PADCTRL_IO + UL(4 * 4))
#define P7_SYS_PADCTRL_IO_05 (P7_SYS_PADCTRL_IO + UL(4 * 5))
#define P7_SYS_PADCTRL_IO_06 (P7_SYS_PADCTRL_IO + UL(4 * 6))
#define P7_SYS_PADCTRL_IO_07 (P7_SYS_PADCTRL_IO + UL(4 * 7))
#define P7_SYS_PADCTRL_IO_08 (P7_SYS_PADCTRL_IO + UL(4 * 8))
#define P7_SYS_PADCTRL_IO_09 (P7_SYS_PADCTRL_IO + UL(4 * 9))
#define P7_SYS_PADCTRL_IO_10 (P7_SYS_PADCTRL_IO + UL(4 * 10))
#define P7_SYS_PADCTRL_IO_11 (P7_SYS_PADCTRL_IO + UL(4 * 11))
#define P7_SYS_PADCTRL_IO_12 (P7_SYS_PADCTRL_IO + UL(4 * 12))
#define P7_SYS_PADCTRL_IO_13 (P7_SYS_PADCTRL_IO + UL(4 * 13))
#define P7_SYS_PADCTRL_IO_14 (P7_SYS_PADCTRL_IO + UL(4 * 14))
#define P7_SYS_PADCTRL_IO_15 (P7_SYS_PADCTRL_IO + UL(4 * 15))
#define P7_SYS_PADCTRL_IO_16 (P7_SYS_PADCTRL_IO + UL(4 * 16))
#define P7_SYS_PADCTRL_IO_17 (P7_SYS_PADCTRL_IO + UL(4 * 17))
#define P7_SYS_PADCTRL_IO_18 (P7_SYS_PADCTRL_IO + UL(4 * 18))
#define P7_SYS_PADCTRL_IO_19 (P7_SYS_PADCTRL_IO + UL(4 * 19))
#define P7_SYS_PADCTRL_IO_20 (P7_SYS_PADCTRL_IO + UL(4 * 20))
#define P7_SYS_PADCTRL_IO_21 (P7_SYS_PADCTRL_IO + UL(4 * 21))
#define P7_SYS_PADCTRL_IO_22 (P7_SYS_PADCTRL_IO + UL(4 * 22))
#define P7_SYS_PADCTRL_IO_23 (P7_SYS_PADCTRL_IO + UL(4 * 23))
#define P7_SYS_PADCTRL_IO_24 (P7_SYS_PADCTRL_IO + UL(4 * 24))
#define P7_SYS_PADCTRL_IO_25 (P7_SYS_PADCTRL_IO + UL(4 * 25))
#define P7_SYS_PADCTRL_IO_26 (P7_SYS_PADCTRL_IO + UL(4 * 26))
#define P7_SYS_PADCTRL_IO_27 (P7_SYS_PADCTRL_IO + UL(4 * 27))
#define P7_SYS_PADCTRL_IO_28 (P7_SYS_PADCTRL_IO + UL(4 * 28))
#define P7_SYS_PADCTRL_IO_29 (P7_SYS_PADCTRL_IO + UL(4 * 29))
#define P7_SYS_PADCTRL_IO_30 (P7_SYS_PADCTRL_IO + UL(4 * 30))
#define P7_SYS_PADCTRL_IO_31 (P7_SYS_PADCTRL_IO + UL(4 * 31))
#define P7_SYS_PADCTRL_IO_32 (P7_SYS_PADCTRL_IO + UL(4 * 32))
#define P7_SYS_PADCTRL_IO_33 (P7_SYS_PADCTRL_IO + UL(4 * 33))
#define P7_SYS_PADCTRL_IO_34 (P7_SYS_PADCTRL_IO + UL(4 * 34))
#define P7_SYS_PADCTRL_IO_35 (P7_SYS_PADCTRL_IO + UL(4 * 35))
#define P7_SYS_PADCTRL_IO_36 (P7_SYS_PADCTRL_IO + UL(4 * 36))
#define P7_SYS_PADCTRL_IO_37 (P7_SYS_PADCTRL_IO + UL(4 * 37))
#define P7_SYS_PADCTRL_IO_38 (P7_SYS_PADCTRL_IO + UL(4 * 38))
#define P7_SYS_PADCTRL_IO_39 (P7_SYS_PADCTRL_IO + UL(4 * 39))
#define P7_SYS_PADCTRL_IO_40 (P7_SYS_PADCTRL_IO + UL(4 * 40))
#define P7_SYS_PADCTRL_IO_41 (P7_SYS_PADCTRL_IO + UL(4 * 41))
#define P7_SYS_PADCTRL_IO_42 (P7_SYS_PADCTRL_IO + UL(4 * 42))
#define P7_SYS_PADCTRL_IO_43 (P7_SYS_PADCTRL_IO + UL(4 * 43))
#define P7_SYS_PADCTRL_IO_44 (P7_SYS_PADCTRL_IO + UL(4 * 44))
#define P7_SYS_PADCTRL_IO_45 (P7_SYS_PADCTRL_IO + UL(4 * 45))
#define P7_SYS_PADCTRL_IO_46 (P7_SYS_PADCTRL_IO + UL(4 * 46))
#define P7_SYS_PADCTRL_IO_47 (P7_SYS_PADCTRL_IO + UL(4 * 47))
#define P7_SYS_PADCTRL_IO_48 (P7_SYS_PADCTRL_IO + UL(4 * 48))
#define P7_SYS_PADCTRL_IO_49 (P7_SYS_PADCTRL_IO + UL(4 * 49))
#define P7_SYS_PADCTRL_IO_50 (P7_SYS_PADCTRL_IO + UL(4 * 50))
#define P7_SYS_PADCTRL_IO_51 (P7_SYS_PADCTRL_IO + UL(4 * 51))
#define P7_SYS_PADCTRL_IO_52 (P7_SYS_PADCTRL_IO + UL(4 * 52))
#define P7_SYS_PADCTRL_IO_53 (P7_SYS_PADCTRL_IO + UL(4 * 53))
#define P7_SYS_PADCTRL_IO_54 (P7_SYS_PADCTRL_IO + UL(4 * 54))
#define P7_SYS_PADCTRL_IO_55 (P7_SYS_PADCTRL_IO + UL(4 * 55))
#define P7_SYS_PADCTRL_IO_56 (P7_SYS_PADCTRL_IO + UL(4 * 56))
#define P7_SYS_PADCTRL_IO_57 (P7_SYS_PADCTRL_IO + UL(4 * 57))

/* Configuration registers. */
#define P7_SYS_CHIP_ID				(P7_SYS + UL(0x400))
#define P7_SYS_BOOT_MODE			(P7_SYS + UL(0x408))
#define P7_SYS_SCRATCH_REG0			(P7_SYS + UL(0x40c))
#define P7_SYS_SCRATCH_REG1			(P7_SYS + UL(0x410))
#define P7_SYS_WATCHDOG             (P7_SYS + UL(0x418))

/* Timers. */
#define P7_SYS_TIMX_CTL			UL(0x0)
#define P7_SYS_TIMXCTL_ENABLE	(1U << 0)
#define P7_SYS_TIMX_LD			UL(0x4)
#define P7_SYS_TIMX_CNT			UL(0x8)
#define P7_SYS_TIM0_IT			(1U << 0)
#define P7_SYS_TIM1_IT			(1U << 1)
#define P7_SYS_TIM2_IT			(1U << 2)
#define P7_SYS_TIM3_IT			(1U << 3)

#define P7_SYS_TIM			(P7_SYS + UL(0x800))

#define P7_SYS_TIM0			(P7_SYS + UL(0x0))
#define P7_SYS_TIM0_CTL		(P7_SYS_TIM0 + P7_SYS_TIMX_CTL)
#define P7_SYS_TIM0_LD		(P7_SYS_TIM0 + P7_SYS_TIMX_LD)
#define P7_SYS_TIM0_CNT		(P7_SYS_TIM0 + P7_SYS_TIMX_CNT)

#define P7_SYS_TIM1			(P7_SYS_TIM + UL(0x10))
#define P7_SYS_TIM1_CTL		(P7_SYS_TIM1 + P7_SYS_TIMX_CTL)
#define P7_SYS_TIM1_LD		(P7_SYS_TIM1 + P7_SYS_TIMX_LD)
#define P7_SYS_TIM1_CNT		(P7_SYS_TIM1 + P7_SYS_TIMX_CNT)

#define P7_SYS_TIM2			(P7_SYS_TIM + UL(0x20))
#define P7_SYS_TIM2_CTL		(P7_SYS_TIM2 + P7_SYS_TIMX_CTL)
#define P7_SYS_TIM2_LD		(P7_SYS_TIM2 + P7_SYS_TIMX_LD)
#define P7_SYS_TIM2_CNT		(P7_SYS_TIM2 + P7_SYS_TIMX_CNT)

#define P7_SYS_TIM3			(P7_SYS_TIM + UL(0x30))
#define P7_SYS_TIM3_CTL		(P7_SYS_TIM3 + P7_SYS_TIMX_CTL)
#define P7_SYS_TIM3_LD		(P7_SYS_TIM3 + P7_SYS_TIMX_LD)
#define P7_SYS_TIM3_CNT		(P7_SYS_TIM3 + P7_SYS_TIMX_CNT)

#define P7_SYS_TIM_STATUS	(P7_SYS_TIM + UL(0x100))
#define P7_SYS_TIM_ITEN		(P7_SYS_TIM + UL(0x104))
#define P7_SYS_TIM_ITACK	(P7_SYS_TIM + UL(0x108))

#endif

/* vim:set ts=4:sw=4:noet:ft=c: */
