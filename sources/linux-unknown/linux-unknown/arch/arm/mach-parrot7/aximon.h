#ifndef _ARCH_PARROT7_AXIMON_H
#define _ARCH_PARROT7_AXIMON_H

struct p7_aximon_master;
struct p7_aximon_port;
struct p7_aximon_plat_data;


#if defined(CONFIG_AXIMON_PARROT7) || \
    defined(CONFIG_AXIMON_PARROT7_MODULE)

#include <linux/init.h>
extern void p7_init_aximon(void) __init;

#else

static inline void p7_init_aximon(void)
{
	return;
}

#endif


#endif
