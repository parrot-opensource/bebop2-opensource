/**
 * p7gpu-pm.h - Parrot7 GPU PM driver interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Alvaro Moran <alvaro.moran@parrot.com>
 * date:    2013-05-30
 *
 * This file is released under the GPL
 */

#ifndef _P7_GPU_PM_H
#define _P7_GPU_PM_H

#if defined(CONFIG_GPU_PARROT7) || defined(CONFIG_GPU_PARROT7_MODULE)

/* Clock names */
#define M200_BUS_CLK	"bus" 
#define M200_CORE_CLK	"core" 
#define M400_CLK	"gpu"

/* 
 * For now the clock names and number is passed as platform data, we might want
 * to put it somewhere else as soon as we find a better solution.
 */
struct p7gpu_pm_plat_data {
	char**	clocks_names;
	int 	clocks_number;
};

#endif
#endif
