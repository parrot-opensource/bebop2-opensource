/*
 * arch/arm/mach-parrot7/fc7100-module.h - common structures and declaration
 *                                        for FC7100 family.
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    2013-07
 *
 * This file is released under the GPL.
 */

#ifndef _ARCH_MODULE_FC71XX_H
#define _ARCH_MODULE_FC71XX_H

#define FC7100_DAIFMT (SND_SOC_DAIFMT_I2S       | \
		       SND_SOC_DAIFMT_NB_NF     | \
		       SND_SOC_DAIFMT_CBS_CFS)


extern int fc7100_module_rev;

static inline int fc7100_module_get_rev(void)
{
	return fc7100_module_rev;
}
int fc7100_board_get_rev(void);
int fc7100_board_get_id(void);

void fc7100_init_module(unsigned int mod_settings);

__init int fc7100_init_spim_single(int bus, int clk_pin, int ss_pin,
			    int mosi_pin, int miso_pin) ;
__init int fc7100_init_mpegts_single(int bus, int clk_pin, int data_pin);

#define FC7100_MOD_SDCARD1 (1<<0)
#define FC7100_MOD_SDCARD2 (1<<1)
#define FC7100_MOD_CAM5 (1<<2)
#define FC7100_MEZZ_MPEGTS (1<<3)
#define FC7100_MOD_CAM0_16 (1<<4)

#endif /* _ARCH_MODULE_FC71XX_H */
