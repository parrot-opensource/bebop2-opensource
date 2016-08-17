/*
 * Parrot 7 MPEG-TS controller platform interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author: Damien Riegel <damien.riegel.ext@parrot.com>
 *
 * This file is released under the GPL
 */


#ifndef _ARCH_PARROT7_MPEGTS_H
#define _ARCH_PARROT7_MPEGTS_H

#include <linux/init.h>
#include <media/p7-mpegts.h>

#if defined(CONFIG_MPEGTS_PARROT7) || \
	defined(CONFIG_MPEGTS_PARROT7_MODULE)

extern void p7_init_mpegts(int core, struct p7mpg_plat_data* pdata,
                           struct pinctrl_map* pins, size_t pin_cnt);
extern void p7_reserve_mpegtsmem(unsigned int core, size_t size);

#else

static inline void p7_init_mpegts(int core,
                                  struct p7mpg_plat_data* pdata,
                                  struct pinctrl_map* pins,
                                  size_t pin_cnt)
{
	return;
}

static inline void p7_reserve_mpegtsmem(unsigned int core, size_t size)
{
	return;
}

#endif

#endif
