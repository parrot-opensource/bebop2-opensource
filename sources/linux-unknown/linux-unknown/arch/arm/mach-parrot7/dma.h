/**
 * linux/arch/arm/mach-parrot7/dma.h - Parrot7 DMA controller platform
 *                                     interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    19-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_DMA_H
#define _ARCH_PARROT7_DMA_H

enum p7_dma_chan {
	P7_SPI0_DMA,
	P7_SPI1_DMA,
	P7_SPI2_DMA,
	P7_SPI3_DMA,
	P7_MEM_DMA,
	P7_MPEGTS0_DMA,
	P7_MPEGTS1_DMA,
	P7_PARINT_DMA = 4,
};

dma_addr_t p7_dma_ucode_addr(void);
size_t p7_dma_ucode_sz(void);

#if defined(CONFIG_DMA_PARROT7)
extern int p7_init_dma(void) __init;
#else
#define p7_init_dma()
#endif

#endif
