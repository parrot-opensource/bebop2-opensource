/**
 * linux/arch/arm/mach-parrot7/dma.c - Parrot7 DMA controller platform
 *                                     implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    19-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/clkdev.h>
#include <linux/amba/bus.h>
#include <linux/amba/pl330.h>
#include <linux/dma-mapping.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "common.h"
#include "clock.h"
#include "dma.h"

#define P7DMA_DRV_NAME  "pl330-dma"

static u8 p7_dma_peri[] = {
	P7_SPI0_DMA,
	P7_SPI1_DMA,
	P7_SPI2_DMA,
	P7_SPI3_DMA,
	P7_MEM_DMA,
	P7_MPEGTS0_DMA,
	P7_MPEGTS1_DMA,
};

#define P7_DMA_UCODE_SZ 1024

static struct dma_pl330_platdata p7_dma_pdata = {
	.nr_valid_peri  = ARRAY_SIZE(p7_dma_peri),
	.peri_id        = p7_dma_peri,
	.mcbuf_sz       = P7_DMA_UCODE_SZ
};

static struct amba_device p7_dma_dev = {
	.dev        = __AMBA_DEV(P7DMA_DRV_NAME ".0",
	                         &p7_dma_pdata,
	                         DMA_BIT_MASK(32)),
	.res        = DEFINE_RES_MEM(P7_DMA, SZ_4K),
	.dma_mask   = DMA_BIT_MASK(32),
	.irq        = { P7_DMA_IRQ, P7_DMA_IRQ },
	.periphid   = 0x00041330
};

size_t p7_dma_ucode_sz(void)
{
	return PAGE_ALIGN(p7_dma_pdata.mcbuf_sz * ARRAY_SIZE(p7_dma_peri));
}

dma_addr_t p7_dma_ucode_addr(void)
{
	/*
	 * Locate microcode region at the bottom of internal RAM, i.e. into user
	 * buffer / bootstrap area (see section 4.12.4 INTRAM memory map of
	 * Parrot7 ROM specificationp).
	 * This prevent us from messing around with Secure INTRAM / ROM bss / heap,
	 * etc... and might help debugging (boot failures), just in case...
	 */
#ifdef DEBUG
	BUG_ON(p7_dma_ucode_sz() > SZ_128K);
#endif
	return P7_INTRAM;
}

int __init p7_init_dma(void)
{
	int         err;
	int const   rev = p7_chiprev();

	pr_debug("p7: registering %s...\n", p7_dma_dev.dev.init_name);

	if (rev ==  P7_CHIPREV_R1) {
		/* P7 first revision initializes DMA controller in non secure mode
		 * when coming out of reset but it is not possible to switch back to
		 * secure mode. Since Linux / cores / L2 cache controller run in
		 * secure mode, and all DMA controller transactions going through ACP
		 * port are flagged as non secure, CPU and DMA accesses to the same
		 * address won't point to same L2 cache internal location.
		 * Therefore, we must disable DMA RAM to RAM ACP accesses (and
		 * bypass L2 cache) to perform transactions directly onto main AXI
		 * system bus (the one behind L2 cache).
		 */
#define P7_NIC_REMAP    P7_NIC
#define NIC_NOACP       (1U << 7)
		__raw_writel(NIC_NOACP, MMIO_P2V(P7_NIC_REMAP));

		/* On R1, DMA interrupts are not shared */
		p7_dma_dev.irq[0] = P7_R1_DMA_ABORT_IRQ;
		p7_dma_dev.irq[1] = P7_R1_DMA5_IRQ;
		p7_dma_pdata.flushp = true;
	}
	else if (rev == P7_CHIPREV_R2 ||
	         rev == P7_CHIPREV_R3) {
		/*
		 * P7_NIC_REMAP is write-only, we can't check the REMAP_DRAM bit
		 * value. The assumption is made that it is already set at this point,
		 * so we add it to our bitmask.
		 */
#define NIC_REMAP_DRAM  (1U)
		__raw_writel(NIC_REMAP_DRAM | NIC_NOACP, MMIO_P2V(P7_NIC_REMAP));

		p7_dma_pdata.flushp = true;
	}

	dma_cap_set(DMA_MEMCPY, p7_dma_pdata.cap_mask);
	dma_cap_set(DMA_SLAVE, p7_dma_pdata.cap_mask);
	dma_cap_set(DMA_CYCLIC, p7_dma_pdata.cap_mask);

	err = amba_device_register(&p7_dma_dev, &iomem_resource);
	if (err)
		panic("p7: failed to register %s (%d)\n",
		      p7_dma_dev.dev.init_name,
		      err);

	/*
	 * We want to store controller microcode into internal RAM for performance
	 * reasons.
	 * As amba_device holds a single resource and pl330 driver does not handle
	 * multiple memory resources, we have to reserve microcode memory region here.
	 * Related device must have been initialized (amba_device_register) before
	 * using dma_declare_coherent_memory.
	 * Moreover, dma_declare_coherent_memory must be performed before pl330
	 * driver loaded since it allocates microcode region at probing time.
	 */
	if (! (dma_declare_coherent_memory(&p7_dma_dev.dev,
	                                   p7_dma_ucode_addr(),
	                                   p7_dma_ucode_addr(),
	                                   p7_dma_ucode_sz(),
	                                   DMA_MEMORY_MAP |
	                                   DMA_MEMORY_EXCLUSIVE) &
	       DMA_MEMORY_MAP))
		/* Failure: will use DMA zone located in system RAM. */
		panic("p7: failed to map DMA controller microcode memory region [%08x:%08x]\n",
		      p7_dma_ucode_addr(),
		      p7_dma_ucode_addr() + p7_dma_ucode_sz() - 1);

	dev_info(&p7_dma_dev.dev,
			 "mapped microcode memory region [%08x:%08x]\n",
			 p7_dma_ucode_addr(),
			 p7_dma_ucode_addr() + p7_dma_ucode_sz() - 1);
	return 0;
}
