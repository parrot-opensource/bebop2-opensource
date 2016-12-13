/*
 *  linux/arch/arm/mach-parrot7/include/mach/p7.h
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

#ifndef _ARCH_PARROT7_P7_H
#define _ARCH_PARROT7_P7_H

#include <asm/memory.h>
#ifndef __ASSEMBLY__
#include <asm/io.h>
#endif


/*
 * I/O devices static mappings: inside vmalloc region,
 * 48MB below VMALLOC_END.
 */
#define __MMIO_P2V(_phys)   (0xff000000 - (3 * SZ_16M) + ((_phys) & 0x03ffffff))
#define MMIO_P2V(_phys)     __typesafe_io(__MMIO_P2V(_phys))

/* Primary input clock frequency (Hertz). */
#define P7_CLKIN_HZ UL(26000000)

#define P7_INTRAM   UL(0x00100000)
#define P7_GPU      UL(0x00200000)
#define P7_AAI      UL(0x00300000)
#define P7_AVI      UL(0x00400000)
#define P7_VDEC     UL(0x00500000)
#define P7_VENC     UL(0x00600000)
#define P7_DMA      UL(0x00700000)
#define P7_CPU      UL(0x00800000)
#define P7_MPMC     UL(0x00900000)
#define P7_SYS      UL(0x00a00000)
#define P7_NIC      UL(0x00e00000)

/*
 * Core Private Memory Region.
 */
#define P7_CPU_PMR          UL(0x00f00000)
/* Snoop Control Unit registers. */
#define P7_CPU_SCU          (P7_CPU_PMR)
/* Local interrupts controller interface. */
#define P7_CPU_ICC          (P7_CPU_PMR + UL(0x0100))
/* Core private / local timer. */
#define P7_CPU_LOCALTIMER   (P7_CPU_PMR + UL(0x0600))
/* Interrupts distributor. */
#define P7_CPU_ICD          (P7_CPU_PMR + UL(0x1000))

/*
 * Level 2 cache controller (PL310).
 */
#define P7_L2C              (P7_CPU_PMR + SZ_512K)

/*
 * MPMC
 */
/* gate training */
#define P7_MPMC_TRAIN_EN (UL(0x4f75 << 2))

/*
 * High Speed Peripherals.
 */
#define P7_HSP          UL(0x01000000)
#define P7_NAND         P7_HSP
#define P7_SPI0         (P7_HSP + UL(0x100000))
#define P7_SPI1         (P7_HSP + UL(0x101000))
#define P7_SPI2         (P7_HSP + UL(0x102000))
#define P7_SPI3         (P7_HSP + UL(0x103000))
#define P7_MPEGTS0      (P7_HSP + UL(0x104000))
#define P7_MPEGTS1      (P7_HSP + UL(0x10c000))
#define P7_SPI          (P7_HSP + UL(0x10f000))
#define P7_SDIO0        (P7_HSP + UL(0x200000))
#define P7_SDIO1        (P7_HSP + UL(0x300000))
#define P7_SDIO2        (P7_HSP + UL(0x400000))
#define P7_USB0         (P7_HSP + UL(0x500000))
#define P7_USB1         (P7_HSP + UL(0x600000))
#define P7_USB2         (P7_HSP + UL(0x700000))
#define P7_ETHER        (P7_HSP + UL(0x800000))
#define P7_CRYPTO       (P7_HSP + UL(0x900000))

/*
 * Low Speed Peripherals.
 */
#define P7_LSP          UL(0x02000000)
#define P7_PWM          (P7_LSP + UL(0x000000))
#define P7_UART0        (P7_LSP + UL(0x100000))
#define P7_UART1        (P7_LSP + UL(0x110000))
#define P7_UART2        (P7_LSP + UL(0x120000))
#define P7_UART3        (P7_LSP + UL(0x130000))
#define P7_UART4        (P7_LSP + UL(0x140000))
#define P7_UART5        (P7_LSP + UL(0x150000))
#define P7_UART6        (P7_LSP + UL(0x160000))
#define P7_UART7        (P7_LSP + UL(0x170000))
#define P7_I2CM0        (P7_LSP + UL(0x200000))
#define P7_I2CM1        (P7_LSP + UL(0x210000))
#define P7_I2CM2        (P7_LSP + UL(0x220000))
#define P7_I2CMS        (P7_LSP + UL(0x800000))
#define P7_I2CS0        (P7_LSP + UL(0x300000))
#define P7_I2CS1        (P7_LSP + UL(0x310000))
#define P7_I2CS2        (P7_LSP + UL(0x320000))
#define P7_GPIO         (P7_LSP + UL(0x400000))
#define P7_CCAN0        (P7_LSP + UL(0x500000))
#define P7_CCAN1        (P7_LSP + UL(0x510000))

/*
 * AXI Monitor
 */
#define P7_AXIMON       (P7_MPMC + UL(0x80000))
#define P7_MPMC_GBC     (P7_MPMC + UL(0xffc00))
#define P7_MPMC_GBC_PRI (P7_MPMC_GBC + UL(0x1f4))


/*
 * P7 Boot mode register containing the diagnostic channel to use amongst other
 * things
 */
#define P7_BOOT_MODE    (P7_SYS + UL(0x408))

#define P7_BOOT_MODE_DIAG_MASK  (0x7 << 5)
#define P7_BOOT_MODE_DIAG_NONE  (0   << 5)
#define P7_BOOT_MODE_DIAG_USB0  (1   << 5)
#define P7_BOOT_MODE_DIAG_UART0 (2   << 5)
#define P7_BOOT_MODE_DIAG_DCC   (3   << 5)
#define P7_BOOT_MODE_DIAG_UART2 (4   << 5)
#define P7_BOOT_MODE_DIAG_UART1 (5   << 5)
#define P7_BOOT_MODE_DIAG_UART3 (6   << 5)

/*
 * System Debug (only present when running emulated design)
 */

#ifdef CONFIG_MACH_PARROT_ZEBU

#ifdef CONFIG_ARCH_PARROT7_ZEBU_MPW1
#define P7_SYS_DBG          (P7_SYS + UL(0x2000))
/* MPW2 */
//#define P7_SYS_DBG        (P7_SYS + UL(0x3000))

#define P7_ZPRINT           P7_SYS_DBG
#define P7_SYS_DBG_HALT     (P7_SYS_DBG + UL(0x1008))
#endif

#ifdef CONFIG_ARCH_PARROT7_ZEBU_MPW2
#define P7_SYS_DBG          (P7_SYS + UL(0x3000))
#define P7_ZPRINT           (P7_SYS + UL(0x70000))
#define P7_SYS_DBG_HALT     (P7_SYS + UL(0x71008))
#endif

#endif /* CONFIG_MACH_PARROT_ZEBU */

#endif
