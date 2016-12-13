/**
 * linux/arch/arm/mach-parrot7/c_can.c - Parrot7 c_can IP Module platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Karl Leplat <karl.leplat@parrot.com>
 * date:    14-Feb-2014
 *
 * This file is released under the GPL
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clkdev.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <linux/delay.h>
#include "common.h"
#include "clock.h"

#include "system.h"

/* Slave 0 resources */
static struct resource p7_ccan0_res[] = {
	[0] = {
		.start = P7_CCAN0,
		.end   = P7_CCAN0 + SZ_128 - 1,
		.flags = IORESOURCE_MEM |
		IORESOURCE_MEM_32BIT,
	},
	[1] = {
		.start = P7_CCAN0_IRQ,
		.end   = P7_CCAN0_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

/* Slave 1 resources */
static struct resource p7_ccan1_res[] = {
	[0] = {
		.start = P7_CCAN1,
		.end   = P7_CCAN1 + SZ_128 - 1,
		.flags = IORESOURCE_MEM |
		IORESOURCE_MEM_32BIT,
	},
	[1] = {
		.start = P7_CCAN1_IRQ,
		.end   = P7_CCAN1_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device p7_ccan_devices[] = {
	{
		.id             = 0,
		.resource       = p7_ccan0_res,
		.num_resources  = ARRAY_SIZE(p7_ccan0_res)
	},
	{
		.id             = 1,
		.resource       = p7_ccan1_res,
		.num_resources  = ARRAY_SIZE(p7_ccan1_res)
	}
};

/**
 * p7_init_c_can() - Instantiate C_CAN IP Module Interface
 *
 * @id:      	bus number
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_c_can(int id,
			struct pinctrl_map* pins,
			size_t pin_cnt)
{
	int err;
	struct clk *ctrlclk;
	struct pinctrl *pctl;

	BUG_ON(id > 1);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);

	switch (id) {
	case 0:
		ctrlclk = clk_get_sys("c_can_platform.0", NULL);
		break;
	case 1:
		ctrlclk = clk_get_sys("c_can_platform.1", NULL);
		break;
	default:
		err = -ENODEV;
		goto err;
	}

	if (IS_ERR(ctrlclk)) {
		err = PTR_ERR(ctrlclk);
		goto err;
	}

	err = clk_prepare_enable(ctrlclk);
	if (err)
		goto err;

	/* we want 8MHz */
	err = clk_set_rate(ctrlclk, 8000000);
	if (err)
		goto disable_ctrl_clk;

	p7_ccan_devices[id].name = "c_can_platform";
	err = p7_init_dev(&p7_ccan_devices[id], NULL, pins, pin_cnt);
	if (err)
		goto disable_ctrl_clk;

	pctl = pinctrl_get_select_default(&(p7_ccan_devices[id]).dev);
	if (IS_ERR(pctl)) {
		err = PTR_ERR(pctl);
		goto disable_ctrl_clk;
	}

	return;

disable_ctrl_clk:
	clk_disable_unprepare(ctrlclk);
err:
	pr_err("p7: failed to init C_CAN %d (%d)\n",
			id, err);
}
