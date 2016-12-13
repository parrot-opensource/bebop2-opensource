/**
 * linux/arch/arm/mach-parrot7/i2cm.c - Parrot7 i2c master controller platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    12-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/clkdev.h>
#include <i2c/p7-i2cm.h>
#include <i2c/muxes/p7-i2cmux.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "pinctrl.h"
#include "common.h"
#include "clock.h"
#include "i2cm.h"

/* Master / bus 0 resources */
static struct resource p7_i2cm0_res[] = {
	[0] = {
		.start = P7_I2CM0,
		.end   = P7_I2CM0 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CM0_IRQ,
		.end   = P7_I2CM0_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

/* Master / bus 1 resources */
static struct resource p7_i2cm1_res[] = {
	[0] = {
		.start = P7_I2CM1,
		.end   = P7_I2CM1 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CM1_IRQ,
		.end   = P7_I2CM1_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

/* Master / bus 2 resources */
static struct resource p7_i2cm2_res[] = {
	[0] = {
		.start = P7_I2CM2,
		.end   = P7_I2CM2 + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CM2_IRQ,
		.end   = P7_I2CM2_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

/* Master / bus 3 resources */
static struct resource p7_i2cm3_res[] = {
	[0] = {
		.start = P7_I2CMS,
		.end   = P7_I2CMS + SZ_4K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_I2CMS_IRQ,
		.end   = P7_I2CMS_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device p7_i2cm_devs[] = {
	{
		.name           = P7I2CM_DRV_NAME,
		.id             = 0,
		.resource       = p7_i2cm0_res,
		.num_resources  = ARRAY_SIZE(p7_i2cm0_res)
	},
	{
		.name           = P7I2CM_DRV_NAME,
		.id             = 1,
		.resource       = p7_i2cm1_res,
		.num_resources  = ARRAY_SIZE(p7_i2cm1_res)
	},
	{
		.name           = P7I2CM_DRV_NAME,
		.id             = 2,
		.resource       = p7_i2cm2_res,
		.num_resources  = ARRAY_SIZE(p7_i2cm2_res)
	},
	{
		.name           = P7I2CM_DRV_NAME,
		.id             = 3,
		.resource       = p7_i2cm3_res,
		.num_resources  = ARRAY_SIZE(p7_i2cm3_res)
	},
};

/**
 * p7_init_i2cm() - Instantiate I2C master controller identified by @bus
 *                  for further driver usage.
 *
 * @bus:        master controller / bus identifier
 * @pdata:      controller platform specific data
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_i2cm(int bus,
                         struct p7i2cm_plat_data* pdata,
                         struct pinctrl_map* pins,
                         size_t pin_cnt)
{
	int err;

#ifdef DEBUG
	BUG_ON(bus >= ARRAY_SIZE(p7_i2cm_devs));
	BUG_ON(! pdata);
	BUG_ON(pin_cnt && !pins);
#endif
	if (bus >= ARRAY_SIZE(p7_i2cm_devs)) {
		return;
	}
	switch(p7_chiprev())
	{
		case P7_CHIPREV_R1:
			pdata->revision = I2CM_REVISION_1;
			break;
		case P7_CHIPREV_R2:
			pdata->revision = I2CM_REVISION_2;
			break;
		case P7_CHIPREV_R3:
			pdata->revision = I2CM_REVISION_3;
			break;
		default:
			BUG();
			break;
	}

	/* Assign pins before registering device in cases driver is already loaded. */
	if (pin_cnt) {
		char buf[10];
		snprintf(buf, 10, "p7-i2cm.%d", bus);
		err = p7_assign_pins(buf, pins, pin_cnt);
		if (err)
			goto err;
	}

	err = p7_init_dev(&p7_i2cm_devs[bus], pdata, NULL, 0);
	if (! err)
		return;

	/*
	 * Pinctrl does not provide us with a way to remove registered pin
	 * mappings...
	 */

err:
	panic("p7: failed to init I2C master controller %d (%d)\n", bus, err);
}

#if defined(CONFIG_I2C_MUX_PARROT7) || defined(CONFIG_I2C_MUX_PARROT7_MODULE)

/* Only I2CM_2 can be muxed on the P7 */
static struct platform_device p7_i2cmux_dev = {
	.name           = P7I2CMUX_DRV_NAME,
	.id             = 0,
};

#include <i2c/muxes/p7-i2cmux.h>

/**
 * p7_init_i2cm2_muxed() - Instantiate I2C master controller 2 handling several
 *                         physical buses. Only I2CM2 can be used that way on
 *                         the P7.
 *
 * @pdata:      platform data for...
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 *
 * Returns: 0 - success, a negative errno like value if failure
 */
void __init p7_init_i2cm_muxed(int bus,
                               struct p7i2cm_plat_data* pdata,
                               struct pinctrl_map* pins,
                               size_t pin_cnt,
                               struct p7i2cmux_plat_data* mux_pdata,
                               struct p7i2cmux_pins const* mux_pins)
{
	char        dname[] = "p7-i2cmux.xx";
	size_t      i;
	int         ret;

#ifdef DEBUG
	BUG_ON(! pdata);
	BUG_ON(! mux_pdata);
	BUG_ON(! mux_pdata->channel_names);
	BUG_ON(! mux_pdata->nr_channels);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
	BUG_ON(! mux_pins);
#endif

	/* Register the I2CM without any associated pins */
	pdata->muxed = 1;
	p7_init_i2cm(bus, pdata, NULL, 0);

	/* We register the pin configurations first in case the driver is
	 * already loaded */

	/* Register default pin configuration */
	snprintf(dname, sizeof(dname), "p7-i2cmux.%d", bus);
	ret = p7_assign_named_pins(dname, NULL, pins, pin_cnt);

	/* Register alternate buses pin configurations */
	for (i = 0; ! ret && i < mux_pdata->nr_channels; i++)
		ret = p7_assign_named_pins(dname,
		                           mux_pdata->channel_names[i],
		                           mux_pins[i].pinmap,
		                           mux_pins[i].sz);
	if (ret)
		goto err;

       /* Register the mux device */
       mux_pdata->parent = bus;
       p7_i2cmux_dev.id  = bus;

       ret = p7_init_dev(&p7_i2cmux_dev, mux_pdata, NULL, 0);
       if (!ret)
               return;

err:
	panic("p7: failed to init muxed I2C master controller %d (%d)\n",
	      bus,
	      ret);
}

#endif /* #if defined(CONFIG_I2C_MUX_PARROT7) || \
              defined(CONFIG_I2C_MUX_PARROT7_MODULE) */

/**
 * p7_init_i2cm_slave() - Register I2C slave device to master controller
 *                        identified by @bus for further driver usage.
 *
 * @bus:        master controller / bus identifier
 * @info:       slave device descriptor
 * @pins:       array of pin functions and optional settings
 * @pin_cnt:    number of element of @pins array
 *
 * Returns: 0 - success, a negative errno like value if failure
 */
int __init p7_init_i2cm_slave(int bus,
                              struct i2c_board_info const* info,
                              struct pinctrl_map* pins,
                              size_t pin_cnt)
{
	struct i2c_adapter* adapt;
	struct i2c_client*  client;
	char*               name = NULL;
	int                 err = 0;

#ifdef DEBUG
	BUG_ON(! info);

	pr_debug("p7: registering %s %d-%04hx...\n", info->type, bus, info->addr);
#endif

	/*
	 * We need to use dynamic slave registration here since master driver
	 * might already be loaded. All static board info would therefore be
	 * ignored.
	 * Get master assigned to bus...
	 */
	adapt = i2c_get_adapter(bus);
	if (! adapt) {
		printk("failed to get adapter\n");
		err = -ENODEV;
		goto err;
	}

	if (pin_cnt) {
		name = kasprintf(GFP_KERNEL,
		                 "%d-%04x",
		                 i2c_adapter_id(adapt),
		                 info->addr | ((info->flags & I2C_CLIENT_TEN)
		                               ? 0xa000 : 0));
		if (! name)
			goto put;

		err = p7_assign_named_pins(name, NULL, pins, pin_cnt);
		if (err) {
			kfree(name);
			goto put;
		}
	}

	/* Register new slave to master retrieved above... */
	client = i2c_new_device(adapt, info);
	if (! client) {
		err = -ENXIO;
		/*
		 * Don't free name since we cannot unregister pin mapping (which would
		 * still holds reference to name).
		 */
		goto put;
	}

	if (err)
		i2c_unregister_device(client);

put:
	i2c_put_adapter(adapt);
err:
	WARN(err,
	     "p7: failed to register %s %d-%04hx (%d)\n",
	     info->type,
	     bus,
	     info->addr,
	     err);
	return err;
}
