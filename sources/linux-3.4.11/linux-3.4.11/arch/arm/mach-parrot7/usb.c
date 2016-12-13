/**
 * linux/arch/arm/mach-parrot7/usb.c - USB controller platform implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:	Guangye Tian <guangye.tian@parrot.com>
 * data :	2012-09-10
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <mach/p7.h>
#include <mach/usb-p7.h>
#include <mach/irqs.h>
#include "common.h"
#include "clock.h"
#include <mfd/p7mu.h>

static u64 dma_mask = DMA_BIT_MASK(32);

struct p7_usb_dma{
	dma_addr_t addr;
	size_t	   size;
};


static struct resource p7_usb0_resource[] = {
	[0] = {
		.start = P7_USB0,
		.end   = P7_USB0 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_USB0_IRQ,
		.end   = P7_USB0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource p7_usb1_resource[] = {
	[0] = {
		.start = P7_USB1,
		.end   = P7_USB1 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_USB1_IRQ,
		.end   = P7_USB1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device p7_usb_devs[] = {
	{
		.id             = 0,
		.name           = "ci_hdrc",
		.num_resources	= ARRAY_SIZE(p7_usb0_resource),
		.resource	    = p7_usb0_resource,
		.dev = {
			.dma_mask          = &dma_mask,
			.coherent_dma_mask = DMA_BIT_MASK(32),
		}
	},
	{
		.id             = 1,
		.name           = "ci_hdrc",
		.num_resources	= ARRAY_SIZE(p7_usb1_resource),
		.resource	    = p7_usb1_resource,
		.dev = {
			.dma_mask          = &dma_mask,
			.coherent_dma_mask = DMA_BIT_MASK(32),
		}
	}
};

static struct p7_usb_dma p7_usb_mem[ARRAY_SIZE(p7_usb_devs)] __initdata;

#if defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC)

#include <mach/usb-p7.h>

/**
 * p7_init_ci_udc() - Instantiate USB controller
 *
 * @core:	usb controller number
 */
void __init p7_init_ci_udc(int core,
			struct p7_usb2_platform_data *pdata,
			struct pinctrl_map *pins,
			size_t pin_cnt)
{
#ifdef DEBUG
	BUG_ON(!pdata);
	BUG_ON(!pins);
	BUG_ON(!pin_cnt);
#endif

	p7_init_dev(&p7_usb_devs[core], pdata, pins, pin_cnt);
	if(! dma_declare_coherent_memory(&p7_usb_devs[core].dev,
				    p7_usb_mem[core].addr,
				    p7_usb_mem[core].addr,
				    p7_usb_mem[core].size,
				    DMA_MEMORY_MAP))
		panic("can't declare memeory for usb\n");
}

/**
 * p7_reserve_usb_mem(int) - Reserve coherent memory for the USB controller
 */
void __init p7_reserve_usb_mem(int core)
{
	p7_usb_mem[core].size = 256 * PAGE_SIZE;/*1MB*/

	/*It's very important for the memory to be reserved in a pool attached to the dev.
	 * Do not reserve this memory inside the global pool, it will cause fragmentation due to
	 * the gap between size allocate by the USB (4KB), ans size allocated by a video driver (>1MB)
	 */
	p7_reserve_devmem(&p7_usb_devs[core],
			  &p7_usb_mem[core].addr,
			  &p7_usb_mem[core].size);
}
#endif  /* defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC */

