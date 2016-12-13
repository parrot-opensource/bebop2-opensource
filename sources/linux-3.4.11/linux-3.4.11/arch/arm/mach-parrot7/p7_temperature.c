/**
 * linux/arch/arm/mach-parrot7/p7_temperature.c - Parrot7 TEMPERATURE platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author: Karl Leplat <karl.leplat@parrot.com>
 * date:   20-Sept-2013
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <mach/p7-adc.h>
#include "pinctrl.h"
#include "p7_temperature.h"

static struct p7_temp_chan p7mu_adc_channels[] = {
	{
		.channel    = 0,
		.freq       = 160000,
		.name       = "p7",
	},
	{
		.channel    = 7,
		.freq       = 160000,
		.name       = "p7mu",
	}
};

static struct p7_temp_chan_data p7mu_adc_chan_data = {
	.channels       = p7mu_adc_channels,
	.num_channels   = ARRAY_SIZE(p7mu_adc_channels),
};

static struct platform_device p7_temp_device = {
	.name               = "p7-temperature",
	.id                 = -1,
	.dev.platform_data  = &p7mu_adc_chan_data,
};

void __init p7_init_temperature(void)
{
	int err;

	p7_config_pin(49, P7CTL_DRV_CFG(5));
	p7_config_pin(50, P7CTL_DRV_CFG(5));

	err = platform_device_register(&p7_temp_device);
	if (err)
		pr_err(KERN_ERR "Error registering P7 temperature device: %d.\n", err);
}
