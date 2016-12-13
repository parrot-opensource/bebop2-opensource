/*
 *      linux/drivers/parrot/video/avi_limit.h
 *
 *      Copyright (C) 2014 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    24-Feb-2014
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

#ifndef _AVI_LIMIT_H_
#define _AVI_LIMIT_H_

#include "avi_dma.h"
#include "avi_pixfmt.h"

/* AVI registers are 16bits wide */
#define AVI_LIMIT_MAX_WIDTH     0xFFFF
#define AVI_LIMIT_MAX_HEIGHT    0xFFFF

/* Maximum frequency for LCD output and camera input clocks in kHz */
#define AVI_LIMIT_MAX_PHY_FREQ  165000

/* Those functions are helpers rounding down a value according to P7R2 and P7R3
 * limitations for both the image frame and the strip used by the FIFOs */

extern void avi_limit_adjust_height(struct avi_dma_pixfmt pixfmt,
				    unsigned *h);
extern void avi_limit_adjust_width(struct avi_dma_pixfmt pixfmt,
				   unsigned *w);
/* This function adjust both left and width strip values */
extern void avi_limit_adjust_w_strip(struct avi_dma_pixfmt pixfmt,
                                     unsigned w,
                                     unsigned *strip_w,
                                     unsigned *left);

/* This function adjust both top and height strip values */
extern void avi_limit_adjust_h_strip(struct avi_dma_pixfmt pixfmt,
                                     unsigned h,
                                     unsigned *strip_h,
                                     unsigned *top);

#endif /* _AVI_LIMIT_H_ */
