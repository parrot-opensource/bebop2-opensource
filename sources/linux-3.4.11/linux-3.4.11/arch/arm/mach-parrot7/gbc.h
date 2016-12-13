/*
 *  linux/arch/arm/mach-parrot7/gbc.h
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

#ifndef _ARCH_PARROT7_GBC_H
#define _ARCH_PARROT7_GBC_H

#include <mach/p7.h>

#define P7_GPU_GBC                  (P7_GPU + UL(0xff000))
/* P7 R1,R2 Mali200 */
#define GPU_GBC_CORE_RESET          UL(0xfe8)
#define GPU_GBC_CORE_CLOCK          UL(0xfec)
#define GPU_GBC_BUS_CLOCK           UL(0xffc)
/* P7 R3 Mali400 */
#define GPU_GBC_POWER          UL(0xff4)
#define GPU_GBC_RESET          UL(0xff8)
#define GPU_GBC_CLOCK          UL(0xffc)

#define P7_AVI_GBC                  (P7_AVI + UL(0xff000))
#define AVI_GBC_RESET               UL(0xff8)
#define AVI_GBC_CLOCK               UL(0xffc)

#define P7_VDEC_GBC                 (P7_VDEC + UL(0xff000))
#define VDEC_GBC_POWER              UL(0xff4)
#define VDEC_GBC_RESET              UL(0xff8)
#define VDEC_GBC_CLOCK              UL(0xffc)


#define P7_VENC_GBC                 (P7_VENC + UL(0xff000))
#define VENC_GBC_POWER              UL(0xff4)
#define VENC_GBC_RESET              UL(0xff8)
#define VENC_GBC_CLOCK              UL(0xffc)


#define P7_DMA_GBC                  (P7_DMA + UL(0xff000))
#define DMA_GBC_RESET               UL(0xff8)
#define DMA_GBC_CLOCK               UL(0xffc)

#define P7_MPMC_GBC                 (P7_MPMC + UL(0xffc00))
#define MPMC_GBC_REQ_PRI            UL(0x1f4)
#define MPMC_GBC_SELF_REFRESH       UL(0x1f8)
#define MPMC_GBC_PWR_DOWN           UL(0x1fc)
#define MPMC_GBC_RESET              UL(0x3f8)

#define P7_CPU_GBC                  (P7_CPU + UL(0xff000))
#define CPU_GBC_CPU0_RESET          UL(0xff8)
#define CPU_GBC_CPU0_POWER          UL(0xff4)
#define CPU_GBC_NEON0_RESET         UL(0xfe8)
#define CPU_GBC_NEON0_POWER         UL(0xfe4)
#define CPU_GBC_CPU1_RESET          UL(0xfd8)
#define CPU_GBC_CPU1_POWER          UL(0xfd4)
#define CPU_GBC_NEON1_RESET         UL(0xfc8)
#define CPU_GBC_NEON1_POWER         UL(0xfc4)
#define CPU_GBC_SCU_CLOCK           UL(0xfbc)
#define CPU_GBC_SCU_RESET           UL(0xfb8)
#define CPU_GBC_SCU_POWER           UL(0xfb4)
#define CPU_GBC_DEBUG_RESET         UL(0xfa8)
#define CPU_GBC_DEBUG_POWER         UL(0xfa4)
#define CPU_GBC_DBGVSOC_POWER       UL(0xf94)
#define CPU_GBC_MBIST_RESET         UL(0xf88)
#define CPU_GBC_MBIST_POWER         UL(0xf84)
#define CPU_GBC_TRACE_CLOCK         UL(0xf7c)
#define CPU_GBC_CORE_STATUS         UL(0xdfc)

#define P7_AAI_GBC                  (P7_AAI + UL(0xf000))
#define P7_AAI_LBC                  (P7_AAI + UL(0xf000))
#define P7_AAI_GBC_R3               (P7_AAI + UL(0xff000))
#define AAI_GBC_RESET               UL(0xff8)
#define AAI_GBC_CLOCK               UL(0xffc)
#define AAI_GBC_POWER               UL(0xff4)

#define P7_HSP_GBC                  (P7_HSP + UL(0xfff000))
#define HSP_GBC_ETHERNET_AXI_LP     UL(0x744)
#define HSP_GBC_ETHERNET_CTRL       UL(0x748)
#define HSP_GBC_ETHERNET_CONFIG     UL(0x74C)
#define HSP_GBC_SDIO2_TDL2_CFG      UL(0x770)
#define HSP_GBC_SDIO2_TDL1_CFG      UL(0x774)
#define HSP_GBC_SDIO2_CTRL          UL(0x778)
#define HSP_GBC_SDIO2_RETUNE        UL(0x77c)
#define HSP_GBC_SDIO1_TDL2_CFG      UL(0x790)
#define HSP_GBC_SDIO1_TDL1_CFG      UL(0x794)
#define HSP_GBC_SDIO1_CTRL          UL(0x798)
#define HSP_GBC_SDIO1_RETUNE        UL(0x79c)
#define HSP_GBC_SDIO0_TDL2_CFG      UL(0x7b0)
#define HSP_GBC_SDIO0_TDL1_CFG      UL(0x7b4)
#define HSP_GBC_SDIO0_CTRL          UL(0x7b8)
#define HSP_GBC_SDIO0_RETUNE        UL(0x7bc)
#define HSP_GBC_CRYPTO_CLOCK        UL(0xf3c)
#define HSP_GBC_CRYPTO_RESET        UL(0xf38)
#define HSP_GBC_ETHERNET_CLOCK      UL(0xf4C)
#define HSP_GBC_ETHERNET_RESET      UL(0xf48)
#define HSP_GBC_PARINT_RESET        UL(0xf58)
#define HSP_GBC_PARINT_AHB_CLOCK    UL(0xf5c)
#define HSP_GBC_SDIO2_XIN_CLOCK     UL(0xf6c)
#define HSP_GBC_SDIO2_RESET         UL(0xf78)
#define HSP_GBC_SDIO2_AHB_CLOCK     UL(0xf7c)
#define HSP_GBC_SDIO1_XIN_CLOCK     UL(0xf8c)
#define HSP_GBC_SDIO1_RESET         UL(0xf98)
#define HSP_GBC_SDIO1_AHB_CLOCK     UL(0xf9c)
#define HSP_GBC_SDIO0_XIN_CLOCK     UL(0xfac)
#define HSP_GBC_SDIO0_RESET         UL(0xfb8)
#define HSP_GBC_SDIO0_AHB_CLOCK     UL(0xfbc)
#define HSP_GBC_USB1_RESET          UL(0xfc8)
#define HSP_GBC_USB1_CLOCK          UL(0xfcc)
#define HSP_GBC_USB0_RESET          UL(0xfd8)
#define HSP_GBC_USB0_CLOCK          UL(0xfdc)
#define HSP_GBC_SPI_RESET           UL(0xfe8)
#define HSP_GBC_SPI_CLOCK           UL(0xfec)
#define HSP_GBC_NAND_CLOCK          UL(0xffc)
#define HSP_GBC_NAND_RESET          UL(0xff8)

#define P7_LSP_GBC                  (P7_LSP + UL(0xfff000))
#define LSP_GBC_I2CMS_RESET         UL(0xee8)
#define LSP_GBC_I2CMS_CLOCK         UL(0xeec)
#define LSP_GBC_CCAN1_RESET         UL(0xef8)
#define LSP_GBC_CCAN1_CLOCK         UL(0xefc)
#define LSP_GBC_CCAN0_RESET         UL(0xf08)
#define LSP_GBC_CCAN0_CLOCK         UL(0xf0c)
#define LSP_GBC_GPIO_RESET          UL(0xf18)
#define LSP_GBC_GPIO_CLOCK          UL(0xf1c)
#define LSP_GBC_I2CS2_RESET         UL(0xf28)
#define LSP_GBC_I2CS2_CLOCK         UL(0xf2c)
#define LSP_GBC_I2CS1_RESET         UL(0xf38)
#define LSP_GBC_I2CS1_CLOCK         UL(0xf3c)
#define LSP_GBC_I2CS0_RESET         UL(0xf48)
#define LSP_GBC_I2CS0_CLOCK         UL(0xf4c)
#define LSP_GBC_I2CM2_RESET         UL(0xf58)
#define LSP_GBC_I2CM2_CLOCK         UL(0xf5c)
#define LSP_GBC_I2CM1_RESET         UL(0xf68)
#define LSP_GBC_I2CM1_CLOCK         UL(0xf6c)
#define LSP_GBC_I2CM0_RESET         UL(0xf78)
#define LSP_GBC_I2CM0_CLOCK         UL(0xf7c)
#define LSP_GBC_UART7_RESET         UL(0xf88)
#define LSP_GBC_UART7_CLOCK         UL(0xf8c)
#define LSP_GBC_UART6_RESET         UL(0xf98)
#define LSP_GBC_UART6_CLOCK         UL(0xf9c)
#define LSP_GBC_UART5_RESET         UL(0xfa8)
#define LSP_GBC_UART5_CLOCK         UL(0xfac)
#define LSP_GBC_UART4_RESET         UL(0xfb8)
#define LSP_GBC_UART4_CLOCK         UL(0xfbc)
#define LSP_GBC_UART3_RESET         UL(0xfc8)
#define LSP_GBC_UART3_CLOCK         UL(0xfcc)
#define LSP_GBC_UART2_RESET         UL(0xfd8)
#define LSP_GBC_UART2_CLOCK         UL(0xfdc)
#define LSP_GBC_UART1_RESET         UL(0xfe8)
#define LSP_GBC_UART1_CLOCK         UL(0xfec)
#define LSP_GBC_UART0_RESET         UL(0xff8)
#define LSP_GBC_UART0_CLOCK         UL(0xffc)

#endif

/* vim:set ts=4:sw=4:noet:ft=c: */
