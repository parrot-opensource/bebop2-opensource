/**
 * linux/arch/arm/mach-parrot7/crypto.c - P7 crypto IP module platform
 * 						implementation
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
#include <mach/p7.h>
#include <mach/irqs.h>
#include "common.h"

#include "system.h"

/* Slave 0 resources */
static struct resource p7_crypto_res[] = {
	[0] = {
		.start = P7_CRYPTO,
		.end   = P7_CRYPTO + SZ_2K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P7_CRYPTO_IRQ,
		.end   = P7_CRYPTO_IRQ,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device p7_crypto_device = {
	.name		= "p7-crypto",
	.id             = 0,
	.resource       = p7_crypto_res,
	.num_resources  = ARRAY_SIZE(p7_crypto_res)
};

/**
 * p7_init_crypto() - Instantiate crypto IP module interface
 */
void __init p7_init_crypto(void)
{
	int err;

	err = p7_init_dev(&p7_crypto_device,
			NULL,
			NULL,
			0);
	if (err)
		pr_err("p7: failed to init crypto (%d)\n",
				err);
}

