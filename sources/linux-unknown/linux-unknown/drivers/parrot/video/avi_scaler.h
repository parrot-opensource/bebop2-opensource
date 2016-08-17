#ifndef _AVI_SCALER_H_
#define _AVI_SCALER_H_

/*
 *  linux/drivers/parrot/video/avi_scaler.h
 *
 *  Copyright (C) 2013 Parrot S.A.
 *
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date  17-Jun-2013
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

/*
 * This file contains the scaler configuration code
 */

#include "avi_compat.h"
#include "avi_pixfmt.h"

/**************
 * Scalers API
 **************/

struct avi_scal_dimcfg;

/**
 * struct avi_scal_cfg - Scaler configuration
 */
struct avi_scal_cfg {
	u32				conf;
	u32				size;
	u32				woffset0;
	u32				wincr0;
	struct avi_scal_dimcfg const*	wcfg0;
	u32				hoffset0;
	u32				hincr0;
	struct avi_scal_dimcfg const*	hcfg0;
	u32				woffset1;
	u32				wincr1;
	struct avi_scal_dimcfg const*	wcfg1;
	u32				hoffset1;
	u32				hincr1;
	struct avi_scal_dimcfg const*	hcfg1;
};

extern void avi_scal_bypass(struct avi_node const *);
extern void avi_scal_to_cfg(struct avi_scal_cfg*,
                            u16, u16, u16, u16,
                            struct avi_dma_pixfmt,
                            int);

extern void avi_scal_to_interleaced_cfg(struct avi_scal_cfg*,
                                        struct avi_scal_cfg*,
                                        u16, u16, u16, u16,
                                        struct avi_dma_pixfmt,
                                        int);

extern void avi_scal_setup(struct avi_node const*,
                           struct avi_scal_cfg const*);
/* The "slow" configuration that computes a temporary config and applies it
 * directly. If you must reconfigure the scaler several times per second you
 * should probably precompute the configurations once and then use
 * avi_setup_scal when needed. */
extern void avi_scal_setup_oneshot(struct avi_node const *,
                                   u16, u16, u16, u16,
                                   struct avi_dma_pixfmt,
                                   int);

/* Function used to reconfigure the scaler between frames when it's used to drop
 * the unnecessary field for interleaced output */
extern void avi_scal_set_field(struct avi_node const *,
                               u16, u16,
                               enum avi_field);

#endif /* _AVI_SCALER_H_ */
