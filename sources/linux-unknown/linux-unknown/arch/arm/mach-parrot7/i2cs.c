/**
 * linux/arch/arm/mach-parrot7/i2cs.c - Parrot7 i2c slave controller platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    13-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clkdev.h>
#include <i2c/plds_i2cs.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "common.h"
#include "clock.h"

/* Slave 0 resources */
static struct resource plds_i2cs0_res[] = {
	[0] = {
		.start = P7_I2CS0,
		.end   = P7_I2CS0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CS0_IRQ,
		.end   = P7_I2CS0_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

/* Slave 1 resources */
static struct resource plds_i2cs1_res[] = {
	[0] = {
		.start = P7_I2CS1,
		.end   = P7_I2CS1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CS1_IRQ,
		.end   = P7_I2CS1_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

/* Slave 2 resources */
static struct resource plds_i2cs2_res[] = {
	[0] = {
		.start = P7_I2CS2,
		.end   = P7_I2CS2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CS2_IRQ,
		.end   = P7_I2CS2_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct resource *plds_i2cs_res[] = {
	plds_i2cs0_res,
	plds_i2cs1_res,
	plds_i2cs2_res,
};

static struct platform_device plds_i2cs_dev = {
		.name           = PLDS_I2CS_NAME,
		.id             = 0,
		.resource       = plds_i2cs0_res,
		.num_resources  = ARRAY_SIZE(plds_i2cs0_res)
};

/**
 * p7_init_i2cs() - Instantiate I2C slave controller identified by @slave
 *                  for further driver usage.
 *
 * @slave:      slave controller identifier
 * @pdata:      controller platform specific data
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_i2cs( struct plds_i2cs_pdata *pdata,
                         struct pinctrl_map* pins,
                         size_t pin_cnt)
{
	int err;

#ifdef DEBUG
	BUG_ON(! pdata);
	BUG_ON(pdata->i2c_bus > 2);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif

	switch(p7_chiprev()) {
	case P7_CHIPREV_R1:
		pdata->revision = I2CS_REVISION_1;
		break;
	case P7_CHIPREV_R2:
		pdata->revision = I2CS_REVISION_2;
		break;
	case P7_CHIPREV_R3:
		pdata->revision = I2CS_REVISION_3;
		break;
	default:
		BUG();
		break;
	}

	plds_i2cs_dev.id              = pdata->i2c_bus;
	plds_i2cs_dev.resource        = plds_i2cs_res[pdata->i2c_bus];
	plds_i2cs_dev.num_resources   = 2;

	err = p7_init_dev(&plds_i2cs_dev, pdata, pins, pin_cnt);
	if (err)
		panic("p7: failed to init I2C slave controller %d (%d)\n",
		      pdata->i2c_bus, err);
}
