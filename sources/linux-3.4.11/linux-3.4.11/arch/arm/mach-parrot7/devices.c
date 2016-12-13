/*
 *	linux/arch/arm/mach-parrot7/devices.c
 *
 *	Copyright (C) 2011 Parrot S.A.
 *
 * @author	Gregor Boirie <gregor.boirie@parrot.com>
 *   @date  15-Feb-2011
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

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <asm/setup.h>
#include <asm/mach/map.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/memblock.h>
#include <mach/p7.h>
#include "gbc.h"
#include "iopin.h"
#include "pinctrl.h"
#include "dma.h"
#include "common.h"
#include "board-common.h"
#include "ion.h"
#include "mpmc.h"

#ifdef CONFIG_HW_PERF_EVENTS
/**********************************
 * Performance Monitoring Unit
 **********************************/
#include <asm/pmu.h>

static struct resource p7_pmu_resource[] = {
	[0] = {
		.start  = P7_PMU_IRQ0,
		.end    = P7_PMU_IRQ0,
		.flags  = IORESOURCE_IRQ
	},
	[1] = {
		.start  = P7_PMU_IRQ1,
		.end    = P7_PMU_IRQ1,
		.flags  = IORESOURCE_IRQ
	}
};

static struct platform_device p7_pmu_dev = {
	.name           = "arm-pmu",
	.id             = ARM_PMU_DEVICE_CPU,
	.resource       = p7_pmu_resource,
	.num_resources  = ARRAY_SIZE(p7_pmu_resource)
};
#endif

static struct resource p7_watch_resource[] = {
	/*[0] = {
		.start  = P7_LOCALWDOG_IRQ,
		.end    = P7_LOCALWDOG_IRQ,
		.flags  = IORESOURCE_IRQ
	},*/
	[1] = {
		.start  = P7_CPU_LOCALTIMER,
		.end    = P7_CPU_LOCALTIMER+0xff,
		.flags  = IORESOURCE_MEM
	}
};

static struct platform_device p7_watchdog_dev = {
	.name           = "mpcore_wdt",
	.id             = -1,
	.resource       = p7_watch_resource,
	.num_resources  = ARRAY_SIZE(p7_watch_resource)
};

static struct platform_device* p7_devices[] __initdata = {
#if defined (CONFIG_HW_PERF_EVENTS)
	&p7_pmu_dev,
#endif
	&p7_watchdog_dev,
};

static struct map_desc p7_iomap[] __initdata = {
#if defined CONFIG_DEBUG_LL
	/* any UART can be used for debugging so we need to remap them all */
	{ /* UART0 debugging console */
		.virtual    = __MMIO_P2V(P7_UART0),
		.pfn        = __phys_to_pfn(P7_UART0),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* UART1 debugging console */
		.virtual    = __MMIO_P2V(P7_UART1),
		.pfn        = __phys_to_pfn(P7_UART1),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* UART2 debugging console */
		.virtual    = __MMIO_P2V(P7_UART2),
		.pfn        = __phys_to_pfn(P7_UART2),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* UART3 debugging console */
		.virtual    = __MMIO_P2V(P7_UART3),
		.pfn        = __phys_to_pfn(P7_UART3),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
#endif
	{ /* Internal RAM */
		.virtual    = __MMIO_P2V(P7_INTRAM),
		.pfn        = __phys_to_pfn(P7_INTRAM),
		.length     = SZ_128K,
		.type       = MT_DEVICE,
	},

	{ /* GPU Global Block Controller */
		.virtual    = __MMIO_P2V(P7_GPU_GBC),
		.pfn        = __phys_to_pfn(P7_GPU_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},

	{ /* AAI */
		.virtual    = __MMIO_P2V(P7_AAI_GBC),
		.pfn        = __phys_to_pfn(P7_AAI_GBC),
		.length     = SZ_1M,
		.type       = MT_DEVICE,
	},
	{ /* AVI */
		.virtual    = __MMIO_P2V(P7_AVI_GBC),
		.pfn        = __phys_to_pfn(P7_AVI_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* VDEC */
		.virtual    = __MMIO_P2V(P7_VDEC_GBC),
		.pfn        = __phys_to_pfn(P7_VDEC_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* VENC */
		.virtual    = __MMIO_P2V(P7_VENC_GBC),
		.pfn        = __phys_to_pfn(P7_VENC_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* DMA */
		.virtual    = __MMIO_P2V(P7_DMA_GBC),
		.pfn        = __phys_to_pfn(P7_DMA_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* CPU cores GBC */
		.virtual    = __MMIO_P2V(P7_CPU_GBC),
		.pfn        = __phys_to_pfn(P7_CPU_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* System controller */
		.virtual    = __MMIO_P2V(P7_SYS),
		.pfn        = __phys_to_pfn(P7_SYS),
		.length     = SZ_1M,
		.type       = MT_DEVICE,
	},
	{ /* Core private memory region */
		.virtual    = __MMIO_P2V(P7_CPU_PMR),
		.pfn        = __phys_to_pfn(P7_CPU_PMR),
		.length     = SZ_8K,
		.type       = MT_DEVICE, /* TODO: MT_DEVICE_NONSHARED ? */
	},
	{ /* NIC301 interconnect */
		.virtual    = __MMIO_P2V(P7_NIC),
		.pfn        = __phys_to_pfn(P7_NIC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* PL310 L2 cache controller */
		.virtual    = __MMIO_P2V(P7_L2C),
		.pfn        = __phys_to_pfn(P7_L2C),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* High Speed Peripheral Global Block Controller */
		.virtual    = __MMIO_P2V(P7_HSP_GBC),
		.pfn        = __phys_to_pfn(P7_HSP_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{ /* Low Speed Peripheral Global Block Controller */
		.virtual    = __MMIO_P2V(P7_LSP_GBC),
		.pfn        = __phys_to_pfn(P7_LSP_GBC),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{
		/*
		 * MPMC GBC to access self-refresh register and prio config
		 * Caution: P7_MPMC_GBC is not aligned on 4K
		 */
		.virtual    = __MMIO_P2V(P7_MPMC_GBC & PAGE_MASK),
		.pfn        = __phys_to_pfn(P7_MPMC_GBC & PAGE_MASK),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{
		/*
		 * MPMC registers: we need to disable gate training
		 * or it may get the DDR out of self-refresh
		 */
		.virtual    = __MMIO_P2V((P7_MPMC + P7_MPMC_TRAIN_EN) & PAGE_MASK),
		.pfn        = __phys_to_pfn((P7_MPMC + P7_MPMC_TRAIN_EN) & PAGE_MASK),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
	{       /* I2C0 to communicate with P7MU */
		.virtual    = __MMIO_P2V(P7_I2CM0),
		.pfn        = __phys_to_pfn(P7_I2CM0),
		.length     = SZ_4K,
		.type       = MT_DEVICE,
	},
};

#if defined(CONFIG_DEBUG_LL) && \
    ! defined(CONFIG_PARROT7_DEBUG_LL_ZPRINT)

/* This variable is used very early in the kernel boot to store the physical and
 * virtual addresses of the debug UART. If the addresses are initialized to 0
 * the addruart code will read the value from the P7_SYS_BOOT_MODE register
 * before the MMU is activated (and the register becomes temporarily
 * innaccessible in the early boot). See include/mach/debug-macro.S.
 *
 * We force the data to be stored in the .data segment because the "addruart"
 * macro is first called before the bss segment is cleared. Therefore if this
 * variable is initialized to 0 it'll contain garbage in the first call unless
 * we move it to the .data segment.
 */
u32 p7_early_uart[] __attribute__ ((section (".data"))) = {
#if defined(CONFIG_PARROT7_DEBUG_LL_DIAG_BOOT_MODE) || \
    defined(DEBUG_LL_UART_NONE)
	0,        0 /* Use BOOT_MODE to choose the UART */
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART0)
	P7_UART0, __MMIO_P2V(P7_UART0)
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART1)
	P7_UART1, __MMIO_P2V(P7_UART1)
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART2)
	P7_UART2, __MMIO_P2V(P7_UART2)
#elif defined(CONFIG_PARROT7_DEBUG_LL_UART3)
	P7_UART3, __MMIO_P2V(P7_UART3)
#else
# error Unsupported early printk UART selected!
#endif
};

#endif /* CONFIG_DEBUG_LL */

void __init p7_init_mach(void)
{
	pr_debug("p7: machine setup\n");

	p7_init_ion();

	/* Register platform devices. */
	if (platform_add_devices(p7_devices, ARRAY_SIZE(p7_devices)))
		goto err;

	/* Init I/O pins controller */
	if (p7_init_pctl())
		goto err;

	p7_pm_init();
	p7brd_init_named_gpios();
	p7_init_dma();

	p7_init_mpmc();

	p7brd_export_mpmc_priorities();
	p7brd_export_fb_positions();

	return;

err:
	panic("p7: failed to setup base machine\n");
}

void __init p7_map_io(void)
{
	/*
	 * Note: do not access I/O devices here since we are in the process of
	 * rebuilding MMU tables (see devicemaps_init).
	 */
	iotable_init(p7_iomap, ARRAY_SIZE(p7_iomap));
}

/***************************************************************
 * DMA'ble memory regions reservation for device usage purposes
 ***************************************************************/

dma_addr_t   p7_dma_addr __initdata;
size_t       p7_dma_size __initdata;

void __init p7_reserve_devmem(struct platform_device* pdev,
                              dma_addr_t* bus_addr,
                              size_t* size)
{
	static int  printed __initdata = 0;

#ifdef DEBUG
	BUG_ON(! pdev);
	BUG_ON(! size);
	BUG_ON(! *size);

	/* Check if the DMA masks are properly set */
	WARN_ON((pdev->dev.coherent_dma_mask &&
	         ! is_device_dma_capable(&pdev->dev)) ||
	        (! pdev->dev.coherent_dma_mask &&
	         is_device_dma_capable(&pdev->dev)));
#endif

	if (! printed) {
		pr_info("P7 per device DMA memory regions:\n");
		printed = 1;
	}

	if (bus_addr) {
		/* arm_memblock_steal PANICs if it cannot allocate enough memory. */
		*size = ALIGN(*size, SECTION_SIZE);
		*bus_addr = arm_memblock_steal(*size, SECTION_SIZE);
		pr_info("\t%08x:%08x: %s.%d\n",
		        *bus_addr,
		        *bus_addr + *size - 1,
		        pdev->name,
		        pdev->id);
	}
	else {
		*size = ALIGN(*size, PAGE_SIZE);
		p7_dma_size += *size;
		pr_info("\t00000000:%08x: %s.%d\n",
		        *size - 1,
		        pdev->name,
		        pdev->id);
	}
}

void __init p7_reserve_dmamem(void)
{
	if (! p7_dma_size)
		return;

	/* arm_memblock_steal PANICs if it cannot allocate enough memory. */
	p7_dma_size = ALIGN(p7_dma_size, SECTION_SIZE);
	p7_dma_addr = arm_memblock_steal(p7_dma_size, SECTION_SIZE);
}

static int __init p7_remap_dmamem(void)
{
	if (p7_dma_size) {
		extern void dma_init_coherent_mem(dma_addr_t, dma_addr_t, size_t, int) __init;

#ifdef DEBUG
		/* make sure p7_reserve_dmamem has been called */
		BUG_ON(!p7_dma_addr);
#endif
		dma_init_coherent_mem(p7_dma_addr,
		                      p7_dma_addr,
		                      p7_dma_size,
		                      DMA_MEMORY_MAP);
	}

	return 0;
}
core_initcall(p7_remap_dmamem);

/* errata check */

#ifdef CONFIG_ARM_ERRATA_430973
#error errata for cortex A8
#endif

#ifdef CONFIG_ARM_ERRATA_720789
#error unconditionnal errata for A9 r0 and r1
#endif

#ifdef CONFIG_ARM_ERRATA_754327
#error unconditionnal errata for A9 r0 and r1
#endif
