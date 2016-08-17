/*
 * @file	cast_timings.c
 * @brief	CAST timing parameters for mode[0 - 5]
 *
 * @Author 	G.Tian (guangye.tian@parrot.com)
 * @Date	2011-11-18
 *
 * Copyright (C) 2011 Parrot S.A.
 */

#include <linux/types.h>
#include "cast-timings.h"

const struct cast_timings cast_modes[2][6] = {
	/* asynchronous data interface */
	[T_ASYN] = {
		/* mode 0 */
		[0] = {
			.twhr = 120, .trhw = 200, .tadl = 200,
			.tccs = 500, .trr  = 40,  .twb  = 200,
			.twp =  50,  .twh =  30, .twc = 100,  .tcad = 45,
			.trp =  53,  .treh =  30, .trc = 100, .pll  = 0,
		},
		/* mode 1 */
		[1] = {
			.twhr = 80,  .trhw = 100, .tadl = 100,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  25,  .twh =  15, .twc =  45,  .tcad = 45,
			.trp =  43,  .treh =  15, .trc =  58,  .pll  = 0,
		},
		/* mode 2 */
		[2] = {
			.twhr = 80,  .trhw = 100, .tadl = 100,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  17,  .twh =  15, .twc =  35,  .tcad = 45,
			.trp =  38,  .treh =  15, .trc =  53,  .pll  = 0,
		},
		/* mode 3 */
		[3] = {
			.twhr = 60,  .trhw = 100, .tadl = 100,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  15,  .twh =  10, .twc =  30,  .tcad = 45,
			.trp =  33,  .treh =  10, .trc =  43,  .pll  = 0,
		},
		/* mode 4 (edo is not supported by CAST) */
		[4] = {
			.twhr = 60,  .trhw = 100, .tadl = 70,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  12,  .twh =  10, .twc =  25,  .tcad = 45,
			.trp =  33,  .treh =  10, .trc =  43,  .pll  = 0,
		},
		/* mode 5 (edo is not supported by CAST) */
		[5] = {
			.twhr = 60,  .trhw = 100, .tadl = 70,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  10,  .twh =   7, .twc =  20,  .tcad = 45,
			.trp =  29,  .treh =   7, .trc =  36,  .pll  = 0,
		},
	},
	/* synchronous data interface */
	[T_SYN] = {
		/* mode 0 */
		[0] = {
			.twhr = 80,  .trhw = 100, .tadl = 100,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  50,  .twh =  30, .twc = 100,  .tcad = 45,
			.trp =  53,  .treh =  30, .trc = 100, .pll  = (20000000*4),
		},
		/* mode 1 */
		[1] = {
			.twhr = 60,  .trhw = 100, .tadl = 100,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  25,  .twh =  15, .twc =  45,  .tcad = 45,
			.trp =  43,  .treh =  15, .trc =  58,  .pll  = (33000000*4),
		},
		/* mode 2 */
		[2] = {
			.twhr = 60,  .trhw = 100, .tadl = 70,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  17,  .twh =  15, .twc =  35,  .tcad = 45,
			.trp =  38,  .treh =  15, .trc =  53,  .pll  = (50000000*4),
		},
		/* mode 3 */
		[3] = {
			.twhr = 60,  .trhw = 100, .tadl = 70,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  15,  .twh =  10, .twc =  30,  .tcad = 45,
			.trp =  33,  .treh =  10, .trc =  43,  .pll  = (66000000*4),
		},
		/* mode 4 */
		[4] = {
			.twhr = 60,  .trhw = 100, .tadl = 70,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  12,  .twh =  10, .twc =  25,  .tcad = 45,
			.trp =  33,  .treh =  10, .trc =  43,  .pll  = (83000000*4),
		},
		/* mode 5 */
		[5] = {
			.twhr = 60,  .trhw = 100, .tadl = 70,
			.tccs = 500, .trr  = 20,  .twb  = 100,
			.twp =  10,  .twh =   7, .twc =  20,  .tcad = 45,
			.trp =  29,  .treh =   7, .trc =  36,  .pll  = (100000000*4),
		},
	},
};
