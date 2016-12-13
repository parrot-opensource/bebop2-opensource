/**
 * linux/arch/arm/mach-parrot7/gpio.c - Parrot7 gpio pin controller platform
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
#include <linux/clkdev.h>
#include <mach/irqs.h>
#include <mach/p7.h>
#include <mach/gpio.h>
#include <gpio/p7-gpio.h>
#include "common.h"
#include "clock.h"

static struct resource p7_gpio_res[] = {
	[0] = {
		.start = P7_GPIO,
		.end   = P7_GPIO + SZ_32K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_GPIO_IRQ,
		.end   = P7_GPIO_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct p7gpio_plat_data p7_gpio_pdata;

static struct platform_device p7_gpio_dev = {
	.name               = P7GPIO_DRV_NAME,
	.id                 = 0,
	.dev.platform_data  = &p7_gpio_pdata,
	.num_resources      = ARRAY_SIZE(p7_gpio_res),
	.resource           = p7_gpio_res
};

/**
 * p7_init_gpio() - Instantiate GPIO controller for further driver usage.
 *
 * @irq_gpios:  array of GPIO numbers to be used as interrupt sources
 * @irq_cnt:    number of entries in array irq_map
 *
 * GPIO -> interrupt mappings are mandatory if drivers need to use one or
 * several GPIO lines as interrupt.
 */
void __init p7_init_gpio(unsigned const* irq_gpios, size_t irq_cnt)
{
	int err = p7_chiprev();

	if (err == P7_CHIPREV_R1)
		/* MPW1 may drive up to 202 GPIOs. */
		p7_gpio_pdata.gpio_nr = 202;
	else
		/* 220 for MPW2 */
		p7_gpio_pdata.gpio_nr = P7_GPIOS_MAX;

	p7_gpio_pdata.irq_gpios = irq_gpios;
	p7_gpio_pdata.irq_gpios_sz = irq_cnt;

	err = p7_init_dev(&p7_gpio_dev, NULL, NULL, 0);
	if (! err)
		return;

	panic("p7: failed to init GPIO controller (%d)", err);
}
