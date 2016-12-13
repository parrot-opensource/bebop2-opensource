/*
 *  linux/arch/arm/mach-parrot7/iopin.c
 *
 *  Copyright (C) 2011 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  10-Feb-2011
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <mach/p7.h>
#include "system.h"
#include "iopin.h"

/* Uncomment this if running on old P7 releases where the input enable bit is buggy... */
#if 0
#define P7_r1865_IOPIN_WORKAROUND
#endif

static void __init p7_setup_iopin(union p7_iopin const config)
{
    unsigned long const addr = __MMIO_P2V(P7_SYS_PADCTRL_IO) +
                               (config.fields.id * sizeof(u32));
    union p7_iopin      pin = { .word = readl(addr) };

#ifndef P7_r1865_IOPIN_WORKAROUND
    /* Disable pin at first. */
    pin.fields.input_en = 0;
    __raw_writel(pin.word, addr);

    if (config.fields.set_usage)
        pin.fields.usage = config.fields.usage;

    if (config.fields.set_trigger)
        pin.fields.trigger = config.fields.trigger;

    if (config.fields.set_pull)
        pin.fields.pull = config.fields.pull;

    if (config.fields.set_slewrate)
        pin.fields.slewrate = config.fields.slewrate;

    if (config.fields.set_driver)
        pin.fields.driver = config.fields.driver;

    /* Write config. */
    writel(pin.word, addr);
    /* Re-enable pin. */
    pin.fields.input_en = 1;
    writel(pin.word, addr);
#else	/* P7_r1865_IOPIN_WORKAROUND */
    if (config.fields.set_usage)
        pin.fields.usage = config.fields.usage;

    __raw_writel(pin.word, addr);
#endif	/* P7_r1865_IOPIN_WORKAROUND */
}

void __init p7_init_iopin_table(union p7_iopin const* iopins)
{
    union p7_iopin const* pin;

    for (pin = iopins; pin->word; pin++)
        p7_setup_iopin(*pin);
}

/* vim:set ts=4:sw=4:et:ft=c: */
