/*
 * linux/arch/arm/mach-parrot7/aximon.c - 
 *      Parrot7 AXI Monitor
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Damien Riegel <damien.riegel.ext@parrot.com>
 * date:    14-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <misc/aximon/p7-aximon.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include "common.h"
#include "aximon.h"



/* Aximon Kernel resource */
static struct resource p7_aximon_res[] = {
    [0] = {
        .start = P7_AXIMON,
        .end   = P7_AXIMON + 0x100 - 1,
        .flags = IORESOURCE_MEM,
    },
    [1] = {
        .start = P7_AXIMON_IRQ,
        .end   = P7_AXIMON_IRQ,
        .flags = IORESOURCE_IRQ,
    }
};

/* Aximon device */
static struct platform_device p7_aximon_dev = {
	.name           = P7AXIMON_DRV_NAME,
	.id             = 0,
	.resource       = p7_aximon_res,
	.num_resources  = ARRAY_SIZE(p7_aximon_res)
};

static struct p7_aximon_master p7_aximon_masters[] = {
    {
        .name   = "cpu0",
        .port   = 0,
        .id_min = 0x000,
        .id_max = 0x0FF,
    },
    {
        .name   = "cpu1",
        .port   = 0,
        .id_min = 0x100,
        .id_max = 0x1FF,
    },
    {
        .name   = "dma",
        .port   = 0,
        .id_min = 0x300,
        .id_max = 0x307,
    },
    {
        .name   = "avi",
        .port   = 1,
        .id_min = 0x000,
        .id_max = 0x00F,
    },
    {
        .name   = "vdec",
        .port   = 2,
        .id_min = 0x000,
        .id_max = 0x0FF,
    },
    {
        .name   = "venc",
        .port   = 2,
        .id_min = 0x100,
        .id_max = 0x1FF,
    },
    {
        .name   = "hsp",
        .port   = 3,
        .id_min = 0x000,
        .id_max = 0x007,
    },
    {
        .name   = "aai",
        .port   = 3,
        .id_min = 0x008,
        .id_max = 0x009,
    },
    /* MPW2
    {
        .name   = "hsp",
        .port   = 3,
        .id_min = 0x000,
        .id_max = 0x03F,
    },
    {
        .name   = "aai",
        .port   = 3,
        .id_min = 0x040,
        .id_max = 0x041,
    },
    */
    {
        .name   = "gpu",
        .port   = 4,
        .id_min = 0x000,
        .id_max = 0x01F,
    },
};

static struct p7_aximon_plat_data p7_aximon_pdata = {
    .masters      = p7_aximon_masters,
    .num_masters  = ARRAY_SIZE(p7_aximon_masters),
    .num_ports    = 5,
};

void __init p7_init_aximon() {
    int err;
    struct p7_aximon_plat_data *pdata = &p7_aximon_pdata;

    err = p7_init_dev(&p7_aximon_dev, pdata, NULL, 0);
    if (err)
        panic("p7: failed to init AXIMON (%d)\n", err);
}
