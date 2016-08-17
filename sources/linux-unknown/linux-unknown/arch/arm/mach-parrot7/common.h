/*
 *  linux/arch/arm/mach-parrot7/common.h
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  24-Nov-2010
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


#ifndef _ARCH_PARROT7_COMMON_H
#define _ARCH_PARROT7_COMMON_H

#include <linux/platform_device.h>

#define P7_CHIPREV_R1 0
#define P7_CHIPREV_R2 1
#define P7_CHIPREV_R3 2
extern int p7_chiprev(void);
extern void p7_restart(char, const char*);

/*************
 * Interrupts
 *************/

extern void p7_init_irq(void) __init;

/*********
 * Timers
 *********/

#include <asm/mach/time.h>

extern struct sys_timer p7_tick_timer;

void p7_init_timer(void) __init;

/******************************
 * Device / mappings resources
 ******************************/

extern void p7_init_mach(void) __init;
extern void p7_map_io(void) __init;

extern void p7_reserve_dmamem(void) __init;
extern void p7_reserve_devmem(struct platform_device*,
                              dma_addr_t*,
                              size_t*) __init;

struct pinctrl_map;
extern int p7_init_dev(struct platform_device*,
                       void* pdata,
                       struct pinctrl_map*,
                       size_t) __init;

/****************
 * Hot plug CPUs
 ****************/

#ifdef CONFIG_HOTPLUG_CPU
extern int p7_poweron_cpu1(void);
extern int p7_poweroff_cpu1(void);
#else
static inline int p7_poweron_cpu1(void) { return 1; }
static inline int p7_poweroff_cpu1(void) { return 1; }
#endif

/*******************
 * Power Management
 *******************/

#ifdef CONFIG_PM
extern void p7_pm_init(void);
#else
static inline void p7_pm_init(void) {}
#endif

extern void v7_cpu_resume(void);

/**********************
 * Machine declaration
 **********************/

#define P7_MACHINE_START(_m, _d)			\
	MACHINE_START(_m, _d)				    \
		.restart_mode   = 's',			    \
		.map_io         = &p7_map_io,		\
		.init_irq       = &p7_init_irq,		\
		.timer          = &p7_tick_timer,	\
		.handle_irq     = &gic_handle_irq,	\
		.restart        = &p7_restart,

#define P7_MACHINE_END MACHINE_END


#endif
