/**
 * linux/arch/arm/mach-parrot7/spi.c - Parrot7 SPI controller platform
 *                                     implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    27-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/clkdev.h>
#include <linux/spi/spi.h>
#include <spi/p7-spi.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "pinctrl.h"
#include "dma.h"
#include "common.h"
#include "clock.h"

/* SPI Kernel resource */
static struct resource p7_spi_res = {
	.start = P7_SPI,
	.end   = P7_SPI + SZ_4K - 1,
	.flags = IORESOURCE_MEM,
};

static struct p7spi_plat_data p7_spi_pdata = {
	.max_slave_hz = 10000000,
	.wcnt = 128,
};

/* SPI Kernel device */
static struct platform_device p7_spi_dev = {
	.name               = P7SPI_DRV_NAME,
	.id                 = 0,
	.dev.platform_data  = &p7_spi_pdata,
	.resource           = &p7_spi_res,
	.num_resources      = 1,
};

static enum p7spi_core_type p7spi_core_r1[] = {
	P7SPI_CORE_SPI,
	P7SPI_CORE_SPI,
	P7SPI_CORE_SPI,
	P7SPI_CORE_SPI,
};

static enum p7spi_core_type p7spi_core_r23[] = {
	P7SPI_CORE_SPI,
	P7SPI_CORE_SPI,
	P7SPI_CORE_SPI,
	P7SPI_CORE_SPI,
	P7SPI_CORE_MPEG,
	P7SPI_CORE_MPEG,
};

static bool already_init = false;

/**
 * p7_init_spi() - Instantiate SPI shared resources "controller"
 *                 for further driver usage.
 *
 * @pdata:      controller platform specific data
 */
void __init p7_init_spi(void)
{
	int err = p7_chiprev();

	if (already_init)
		return;

	switch (err) {
	case P7_CHIPREV_R1:
		p7_spi_pdata.rev = SPI_REVISION_1;
		break;
	case P7_CHIPREV_R2:
		p7_spi_pdata.rev = SPI_REVISION_2;
		break;
	case P7_CHIPREV_R3:
		p7_spi_pdata.rev = SPI_REVISION_3;
		break;
	default:
		BUG();
		break;
	}

	if (p7_spi_pdata.rev == SPI_REVISION_1) {
		/* MPW1 is limited to 127kHz - 65MHz. */
		p7_spi_pdata.min_hz = 127000;
		p7_spi_pdata.max_master_hz = 65000000;
		p7_spi_pdata.num_pads = 16;
		p7_spi_pdata.cores = p7spi_core_r1;
		p7_spi_pdata.num_cores = ARRAY_SIZE(p7spi_core_r1);
	}
	else {
		/* R2 and R3 to 2kHz - 130 MHz */
		p7_spi_pdata.min_hz = 2000;
		p7_spi_pdata.max_master_hz = 130000000;
		p7_spi_pdata.num_pads = 20;
		p7_spi_pdata.cores = p7spi_core_r23;
		p7_spi_pdata.num_cores = ARRAY_SIZE(p7spi_core_r23);
	}

	err = p7_init_dev(&p7_spi_dev, NULL, NULL, 0);
	if (! err) {
		already_init = true;
		return;
	}

	panic("p7: failed to init SPI kernel (%d)\n", err);
}

#if defined(CONFIG_SPI_MASTER_PARROT7) || \
    defined(CONFIG_SPI_MASTER_PARROT7_MODULE) || \
    defined(CONFIG_SPI_SLAVE_PARROT7) || \
    defined(CONFIG_SPI_SLAVE_PARROT7_MODULE)

/* Bus 0 resources */
static struct resource p7_spi0_res[] = {
	[0] = {
		.start = P7_SPI0,
		.end   = P7_SPI0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_SPI0_IRQ,
		.end   = P7_SPI0_IRQ,
#ifdef CONFIG_ARCH_VEXPRESS_P7FPGA
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
#else
		.flags = IORESOURCE_IRQ,
#endif
	},
	[2] = {
		.start	= P7_SPI0_DMA,
		.end	= P7_SPI0_DMA,
		.flags	= IORESOURCE_DMA,
	}
};

/* Bus 1 resources */
static struct resource p7_spi1_res[] = {
	[0] = {
		.start = P7_SPI1,
		.end   = P7_SPI1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_SPI1_IRQ,
		.end   = P7_SPI1_IRQ,
#ifdef CONFIG_ARCH_VEXPRESS_P7FPGA
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
#else
		.flags = IORESOURCE_IRQ,
#endif
	},
	[2] = {
		.start	= P7_SPI1_DMA,
		.end	= P7_SPI1_DMA,
		.flags	= IORESOURCE_DMA,
	}
};

/* Bus 2 resources */
static struct resource p7_spi2_res[] = {
	[0] = {
		.start = P7_SPI2,
		.end   = P7_SPI2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_SPI2_IRQ,
		.end   = P7_SPI2_IRQ,
#ifdef CONFIG_ARCH_VEXPRESS_P7FPGA
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
#else
		.flags = IORESOURCE_IRQ,
#endif
	},
	[2] = {
		.start	= P7_SPI2_DMA,
		.end	= P7_SPI2_DMA,
		.flags	= IORESOURCE_DMA,
	}
};

/* Bus 3 resources */
static struct resource p7_spi3_res[] = {
	[0] = {
		.start = P7_SPI3,
		.end   = P7_SPI3 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_SPI3_IRQ,
		.end   = P7_SPI3_IRQ,
#ifdef CONFIG_ARCH_VEXPRESS_P7FPGA
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
#else
		.flags = IORESOURCE_IRQ,
#endif
	},
	[2] = {
		.start	= P7_SPI3_DMA,
		.end	= P7_SPI3_DMA,
		.flags	= IORESOURCE_DMA,
	}
};

static struct platform_device p7_spi_devs[] = {
	{
		.id             = 0,
		.resource       = p7_spi0_res,
		.num_resources  = ARRAY_SIZE(p7_spi0_res),
	},
	{
		.id             = 1,
		.resource       = p7_spi1_res,
		.num_resources  = ARRAY_SIZE(p7_spi1_res),
	},
	{
		.id             = 2,
		.resource       = p7_spi2_res,
		.num_resources  = ARRAY_SIZE(p7_spi2_res),
	},
	{
		.id             = 3,
		.resource       = p7_spi3_res,
		.num_resources  = ARRAY_SIZE(p7_spi3_res),
	}
};

#endif  /* defined(CONFIG_SPI_MASTER_PARROT7) || \
           defined(CONFIG_SPI_MASTER_PARROT7_MODULE) || \
           defined(CONFIG_SPI_SLAVE_PARROT7) || \
           defined(CONFIG_SPI_SLAVE_PARROT7_MODULE) */

#if defined(CONFIG_SPI_MASTER_PARROT7) || \
    defined(CONFIG_SPI_MASTER_PARROT7_MODULE)
#include <spi/p7-spim.h>

/**
 * p7_init_spim() - Instantiate SPI master controller identified by @bus
 *                  for further driver usage.
 *
 * @bus:        master controller / bus identifier
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_spim(int bus, struct pinctrl_map* pins, size_t pin_cnt,
                         struct p7spi_swb const * const pdata)
{
	int err;

#ifdef DEBUG
	BUG_ON(bus >= ARRAY_SIZE(p7_spi_devs));
	BUG_ON(p7_spi_devs[bus].name);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
	BUG_ON(! pdata);
#endif
	p7_init_spi();

	p7_spi_devs[bus].name = P7SPIM_DRV_NAME;
	err = p7_init_dev(&p7_spi_devs[bus], (void*)pdata, pins, pin_cnt);
	if (! err)
		return;

	panic("p7: failed to init SPI master controller %d (%d)\n",
	      bus,
	      err);
}

/**
 * p7_init_spim_slave() - Register SPI slave device to master controller
 *                        further driver usage.
 *
 * @bus:        master controller / bus identifier
 * @info:       slave device descriptor
 *
 * Returns: 0 - success, a negative errno like value if failure
 */
int __init p7_init_spim_slave(int bus, struct spi_board_info* info)
{
	int err;

#ifdef DEBUG
	struct p7spi_ctrl_data* const   cdata = info->controller_data;
	BUG_ON(bus >= ARRAY_SIZE(p7_spi_devs));
	BUG_ON(! info);
	BUG_ON(! info->modalias);
	BUG_ON(! cdata);
	BUG_ON(info->chip_select);
	BUG_ON(p7_spi_devs[bus].name && strcmp(p7_spi_devs[bus].name, P7SPIM_DRV_NAME));

	pr_debug("p7: registering %s%sspi%u.0...\n",
	         info->modalias ? info->modalias : "",
	         info->modalias ? " slave as " : "",
	         bus);
#endif

	info->bus_num = bus;
	err = spi_register_board_info(info, 1);
	WARN(err,
		 "p7: failed to register %s%sspi%u.0 (%d)\n",
		 info->modalias ? info->modalias : "",
		 info->modalias ? " slave as " : "",
		 bus,
		 err);

	return err;
}

#endif  /* defined(CONFIG_SPI_MASTER_PARROT7) || \
           defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

#if defined(CONFIG_SPI_SLAVE_PARROT7) || \
    defined(CONFIG_SPI_SLAVE_PARROT7_MODULE)
#include <spi/p7-spis.h>

/**
 * p7_init_spis() - Instantiate SPI slave controller identified by @bus
 *                  for further driver usage.
 *
 * @bus:        slave controller / bus identifier
 * @pins:       array of pin functions and optional settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_spis(int bus, struct pinctrl_map* pins, size_t pin_cnt,
                         struct p7spi_swb const * const pdata)
{
	int err;

#ifdef DEBUG
	BUG_ON(bus >= ARRAY_SIZE(p7_spi_devs));
	BUG_ON(p7_spi_devs[bus].name);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
	BUG_ON(! pdata);
#endif
	p7_init_spi();

	p7_spi_devs[bus].name = P7SPIS_DRV_NAME;
	err = p7_init_dev(&p7_spi_devs[bus], (void*)pdata, pins, pin_cnt);
	if (! err)
		return;

	panic("p7: failed to init SPI slave controller %d (%d)\n",
	      bus,
	      err);
}

/**
 * p7_init_spis_master() - Register SPI master device to slave controller
 *                         for further driver usage.
 *
 * @bus:        slave controller / bus identifier
 * @info:       master device descriptor
 *
 * Returns: 0 - success, a negative errno like value if failure
 */
int __init p7_init_spis_master(int bus, struct spi_board_info* info)
{
	int err;

#ifdef DEBUG
	struct p7spi_ctrl_data* const   cdata = info->controller_data;
	BUG_ON(bus >= ARRAY_SIZE(p7_spi_devs));
	BUG_ON(! info);
	BUG_ON(! info->modalias);
	BUG_ON(! cdata);
	BUG_ON(info->chip_select);
	BUG_ON(strcmp(p7_spi_devs[bus].name, P7SPIS_DRV_NAME));

	pr_debug("p7: registering %s%sspi%u.0...\n",
	         info->modalias ? info->modalias : "",
	         info->modalias ? " master as " : "",
	         bus);
#endif

	info->bus_num = bus;
	err = spi_register_board_info(info, 1);
	WARN(err,
		 "p7: failed to register %s%sspi%u.0 (%d)\n",
		 info->modalias ? info->modalias : "",
		 info->modalias ? " slave as " : "",
		 bus,
		 err);

	return err;
}

#endif  /* defined(CONFIG_SPI_SLAVE_PARROT7) || \
           defined(CONFIG_SPI_SLAVE_PARROT7_MODULE) */
