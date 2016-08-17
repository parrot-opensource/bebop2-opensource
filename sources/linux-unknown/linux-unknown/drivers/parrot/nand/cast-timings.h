/**
 * CAST Nand Controller Driver
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:	G.Tian <guangye.tian@parrot.com>
 * date:	2012-10-24
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */
#ifndef __CAST_TIMINGS_H
#define __CAST_TIMINGS_H

/* Timing parameters corresponding to cast registers (ns) */
struct cast_timings {
	/*
	 * (ASYN) WE# high to RE# low time
	 * (SYN)  Cmd to data output time
	 */
	u16 twhr;
	/*
	 * (ASYN) RE# high to WE# low time
	 * (SYN)  Data output to command, addr, data input time
	 */
	u16 trhw;
	/*
	 * (ASYN) ALE to data start time
	 * (SYN)  ALE to data loading time
	 */
	u16 tadl;
	/*
	 * Change column setup
	 * To be determined with the parameter page for non-zero timing mode
	 */
	u16 tccs;
	/* Read high to read low */
	u16 trr;
	/* Busy time for interface change (asyn<->syn)*/
	u16 twb;
	/* (ASYN) */
	u16 twp;
	u16 twh;
	u16 twc;
	/* (SYN) Command, address and data delay */
	u16 tcad;
	/* (ASYN) */
	u16 trp;
	u16 treh;
	u16 trc;

	/* predefined pll value */
	u32 pll;
};

#define T_SYN	0
#define T_ASYN	1

extern const struct cast_timings cast_modes[2][6];

struct onfi_par_page {
	int syn_mode;
	int asyn_mode;
};

#endif /* __CAST_TIMINGS_H */
