/**
 * linux/arch/arm/mach-parrot7/aai.h - Parrot7 aai platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Samir Ammenouche <samir.ammenouche@parrot.com>
 * date:    04-Oct-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_AAI_H
#define _ARCH_PARROT7_AAI_H

#include <linux/init.h>
#include "pinctrl.h"
#include "sound/p7_aai/aai_platform_data.h"
#include "sound/p7_aai/reg_aai.h"

#if defined(CONFIG_SND_AAI) || \
    defined(CONFIG_SND_AAI_MODULE)

extern void p7_init_aai(struct pinctrl_map*, size_t, struct aai_platform_data*) __init;
extern void p7_init_aai2(struct aai_platform_data*) __init;

#else	/* defined(CONFIG_SND_AAI) || defined(CONFIG_SND_AAI_MODULE) */

static inline void p7_init_aai(struct pinctrl_map	*m,
                               size_t			 s,
                               struct aai_platform_data *p)
{
	return;
}

static inline void p7_init_aai2(struct aai_platform_data *p)
{
	return;
}
#endif	 /* defined(CONFIG_SND_AAI) || defined(CONFIG_SND_AAI_MODULE) */

/*
 * Configure the aai register:  The quadruplet is
 * 	<address, shift, mask, value>
 * */

#define MUS_SYNC_DAC(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x0
#define MUS_SYNC_ADC(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x1
#define MUS_SYNC_AUX(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x2
#define MUS_ASYNC(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x3
#define MUS_TDM_0(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x4
#define MUS_TDM_1(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x5
#define MUS_TDM_2(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x6
#define MUS_TDM_3(channel)		AAI_AAICFG_MUS(channel), 12, 7, 0x7
#define MUS_SPDIF(channel)		AAI_AAICFG_MUS(channel), 16, 1, 0x1

#define AAI_MASTER(channel)			MUS_SYNC_DAC(channel)
#define AAI_SLAVE(channel)			MUS_ASYNC(channel)
/*#define LOOPBACK()*/

#define MIC_MUX_ENABLE(output)		AAI_AAICFG_MIC_MUX, (2+((output)*8)), 1, 0x1
#define MIC_MUX_MUX(mic, output)	AAI_AAICFG_MIC_MUX, ((output)*8), 1, (mic)

#define RESET_VOI_MUX(output)		AAI_AAICFG_VOI_MUX, ((output)*8), 0xffffffff, 0
#define VOI_MUX(output)			AAI_AAICFG_VOI_MUX, ((output)*8), 0xf, 9
#define VOI_MUX_INVERSE(output)		AAI_AAICFG_VOI_MUX, ((output)*8), 0xf, 6
#define VOI_MUX_DISABLE(output)		AAI_AAICFG_VOI_MUX, ((output)*8), 0xf, 0

#endif
