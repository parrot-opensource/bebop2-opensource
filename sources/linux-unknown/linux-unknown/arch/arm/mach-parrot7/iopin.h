/*
 *  linux/arch/arm/mach-parrot7/iopin.h
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

#ifndef _ARCH_PARROT7_IOPIN_H
#define _ARCH_PARROT7_IOPIN_H

#include <linux/init.h>
#include <asm/types.h>

union p7_iopin {
    u32 word;
    struct {
        unsigned    usage:2;
        unsigned    unused0:1;
        unsigned    set_usage:1;
        unsigned    set_trigger:1;
        unsigned    set_pull:1;
        unsigned    set_slewrate:1;
        unsigned    set_driver:1;
        unsigned    trigger:1;
        unsigned    pull:3;
        unsigned    input_en:1;
        unsigned    unused1:3;
        unsigned    slewrate:2;
        unsigned    driver:6;
        unsigned    id:8;
    } fields;
};

#define P7_IOPIN_USAGE      (1 << 3)
#define P7_IOPIN_TRIGGER    (1 << 4)
#define P7_IOPIN_PULL       (1 << 5)
#define P7_IOPIN_SLEWRATE   (1 << 6)
#define P7_IOPIN_DRIVER     (1 << 7)

#define P7_IOPIN_SMT_NONE   0   /* No trigger */
#define P7_IOPIN_SMT_EN     1   /* Schmidt trigger enabled */

#define P7_IOPIN_PULL_HIGHZ 0   /* High impendence */
#define P7_IOPIN_PULL_UP    1   /* Pull up */
#define P7_IOPIN_PULL_DOWN  2   /* Pull down */
#define P7_IOPIN_PULL_KEEP  4   /* Bus keeper */

#define P7_IOPIN_SLEWRATE_0 0   /* Slowest */
#define P7_IOPIN_SLEWRATE_1 1
#define P7_IOPIN_SLEWRATE_2 2
#define P7_IOPIN_SLEWRATE_3 3   /* Fastest */

#define P7_IOPIN_DRIVER_0   1   /* Weakest */
#define P7_IOPIN_DRIVER_1   3
#define P7_IOPIN_DRIVER_2   7
#define P7_IOPIN_DRIVER_3   0xf
#define P7_IOPIN_DRIVER_4   0x1f
#define P7_IOPIN_DRIVER_5   0x3f/* Strongest */

#define P7_INIT_FULL_IOPIN(_id,                                 \
                           _set,                                \
                           _trigger,                            \
                           _pull,                               \
                           _slewrate,                           \
                           _driver)                             \
    { .fields = {                                               \
            .usage          = (_id) & 0xf,                      \
            .set_usage      = !! ((_set) & P7_IOPIN_USAGE),     \
            .set_trigger    = !! ((_set) & P7_IOPIN_TRIGGER),   \
            .set_pull       = !! ((_set) & P7_IOPIN_PULL),      \
            .set_slewrate   = !! ((_set) & P7_IOPIN_SLEWRATE),  \
            .set_driver     = !! ((_set) & P7_IOPIN_DRIVER),    \
            .trigger        = _trigger,                         \
            .pull           = _pull,                            \
            .slewrate       = _slewrate,                        \
            .driver         = _driver,                          \
            .id             = ((_id) >> 4)                      \
        }                                                       \
    }

#define P7_INIT_IOPIN(_id) \
    P7_INIT_FULL_IOPIN(_id, P7_IOPIN_USAGE, 0, 0, 0, 0)

extern void p7_init_iopin_table(union p7_iopin const*) __init;

#endif

/* vim:set ts=4:sw=4:et:ft=c: */
