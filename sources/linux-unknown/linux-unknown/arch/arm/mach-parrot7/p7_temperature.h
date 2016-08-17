/**
 * linux/arch/arm/mach-parrot7/p7_temperature.h - Parrot7 TEMPERATURE platform interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author: Karl Leplat <karl.leplat@parrot.com>
 * date:    20-Sept-2013
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_P7TEMPERATURE_H
#define _ARCH_PARROT7_P7TEMPERATURE_H

#if defined(CONFIG_P7_TEMPERATURE) || \
    defined(CONFIG_P7_TEMPERATURE_MODULE)

extern void p7_init_temperature(void) __init;

#else	/* defined(CONFIG_P7_TEMPERATURE) || \
	       defined(CONFIG_P7_TEMPERATURE) */

#define p7_init_temperature() \
	({ -ENOSYS; })

#endif	 /* defined(CONFIG_P7_TEMPERATURE) || \
	        defined(CONFIG_P7_TEMPERATURE_MODULE) */

#endif /*_ARCH_PARROT7_P7TEMPERATURE_H*/
