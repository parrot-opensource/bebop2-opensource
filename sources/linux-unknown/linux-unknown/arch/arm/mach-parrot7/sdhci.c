/**
 * linux/arch/arm/mach-parrot7/sdhci.c - Parrot7 eMMC / SD / SDIO controller
 *                                       platform implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    14-Jun-2012
 *
 * This file is released under the GPL
 *
 * See linux/drivers/parrot/mmc/acs3-sdhci.c file header
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <mach/irqs.h>
#include <mach/p7.h>
#include <mmc/acs3-sdhci.h>
#include "common.h"
#include "clock.h"

/*****************************
 * Host controller instances
 *****************************/

static struct resource p7_sdhci0_res[] = {
	[0] = {
		.start = P7_SDIO0,
		.end   = P7_SDIO0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_SDIO0_IRQ,
		.end   = P7_SDIO0_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_sdhci1_res[] = {
	[0] = {
		.start = P7_SDIO1,
		.end   = P7_SDIO1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_SDIO1_IRQ,
		.end   = P7_SDIO1_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct resource p7_sdhci2_res[] = {
	[0] = {
		.start = P7_SDIO2,
		.end   = P7_SDIO2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM
	},
	[1] = {
		.start = P7_SDIO2_IRQ,
		.end   = P7_SDIO2_IRQ,
		.flags = IORESOURCE_IRQ
	}
};

static struct platform_device p7_sdhci_devs[] = {
	{
		.name           = ACS3_DRV_NAME,
		.id             = 0,
		.resource       = p7_sdhci0_res,
		.num_resources  = ARRAY_SIZE(p7_sdhci0_res)
	},
	{
		.name           = ACS3_DRV_NAME,
		.id             = 1,
		.resource       = p7_sdhci1_res,
		.num_resources  = ARRAY_SIZE(p7_sdhci1_res)
	},
	{
		.name           = ACS3_DRV_NAME,
		.id             = 2,
		.resource       = p7_sdhci2_res,
		.num_resources  = ARRAY_SIZE(p7_sdhci2_res)
	}
};

static struct acs3_regs const p7_sdhci_regs[] = {
	{   /* SDIO controller 0. */
		.tdl1   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO0_TDL1_CFG),
		.tdl2   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO0_TDL2_CFG),
		.ctrl   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO0_CTRL),
		.retune = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO0_RETUNE)
	},
	{   /* SDIO controller 1. */
		.tdl1   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO1_TDL1_CFG),
		.tdl2   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO1_TDL2_CFG),
		.ctrl   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO1_CTRL),
		.retune = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO1_RETUNE)
	},
	{   /* SDIO controller 2. */
		.tdl1   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO2_TDL1_CFG),
		.tdl2   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO2_TDL2_CFG),
		.ctrl   = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO2_CTRL),
		.retune = __MMIO_P2V(P7_HSP_GBC + HSP_GBC_SDIO2_RETUNE)
	}
};

/**
 * p7_init_sdhci() - Instantiate SDHCI host controller identified by @host
 *                   for further driver usage.
 *
 * @host:       SDHCI host controller identifier
 * @pdata:      controller platform specific data
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_sdhci(int host,
                          struct acs3_plat_data* pdata,
                          struct pinctrl_map* pins,
                          size_t pin_cnt)
{
#ifdef DEBUG
	BUG_ON(host < 0);
	BUG_ON(host >= ARRAY_SIZE(p7_sdhci_devs));
	BUG_ON(! pdata);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif

	/*
	 * Set GBC registers addresses so that driver can easily locate them in a
	 * portable way.
	 */
	pdata->regs = &p7_sdhci_regs[host];

	/*
	 * Disable the command confict error, i.e. disable multi-master mode.
	 * This allows us to keep working at high frequencies.
	 */
	__raw_writel(__raw_readl(pdata->regs->ctrl) | ACS3_CTRL_CMD_CONFLICT_DIS_MASK,
	             pdata->regs->ctrl);

	p7_init_dev(&p7_sdhci_devs[host], pdata, pins, pin_cnt);
}
