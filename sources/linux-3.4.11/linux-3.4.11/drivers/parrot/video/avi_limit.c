/*
 *      linux/drivers/parrot/video/avi_limit.c
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

#include <linux/module.h>
#include "avi_limit.h"

#define AVI_LIMIT_MAX   0x10000U

/* Limitations types according to P7 User Manual */
enum avi_pixfmt_type {
	AVI_LIMIT_UNKNOWN = 0,
	AVI_LIMIT_PACKED_444_16BITS,
	AVI_LIMIT_PACKED_444_24BITS,
	AVI_LIMIT_PACKED_444_32BITS,
	AVI_LIMIT_PACKED_422_16BITS,
	AVI_LIMIT_PLANAR_422_16BITS,
	AVI_LIMIT_PLANAR_420_16BITS,
	AVI_LIMIT_NR,
};

enum avi_limit_type {
	AVI_LIMIT_WIDTH = 0,
	AVI_LIMIT_HEIGHT,
	AVI_LIMIT_WIDTH_STRIP,
	AVI_LIMIT_HEIGHT_STRIP,
	AVI_LIMIT_LEFT,
	AVI_LIMIT_TOP,
};

/* Frame and strip height limitations are the same on P7R2 and P7R3 */
static unsigned avi_limit_height[] = {
	[AVI_LIMIT_UNKNOWN] = 2,
	[AVI_LIMIT_PACKED_444_16BITS] = 1,
	[AVI_LIMIT_PACKED_444_24BITS] = 1,
	[AVI_LIMIT_PACKED_444_32BITS] = 1,
	[AVI_LIMIT_PACKED_422_16BITS] = 1,
	[AVI_LIMIT_PLANAR_422_16BITS] = 1,
	[AVI_LIMIT_PLANAR_420_16BITS] = 2,
};

/* Frame width limitations are the same on P7R2 and P7R3 */
static unsigned avi_limit_width[] = {
	[AVI_LIMIT_UNKNOWN] = 8,
	[AVI_LIMIT_PACKED_444_16BITS] = 4,
	[AVI_LIMIT_PACKED_444_24BITS] = 8,
	[AVI_LIMIT_PACKED_444_32BITS] = 2,
	[AVI_LIMIT_PACKED_422_16BITS] = 4,
	[AVI_LIMIT_PLANAR_422_16BITS] = 8,
	[AVI_LIMIT_PLANAR_420_16BITS] = 8,
};

/* Strip width  */
static unsigned avi_limit_r2_width_strip[] = {
	[AVI_LIMIT_UNKNOWN] = 8,
	[AVI_LIMIT_PACKED_444_16BITS] = 1,
	[AVI_LIMIT_PACKED_444_24BITS] = 1,
	[AVI_LIMIT_PACKED_444_32BITS] = 1,
	[AVI_LIMIT_PACKED_422_16BITS] = 8,
	[AVI_LIMIT_PLANAR_422_16BITS] = 2,
	[AVI_LIMIT_PLANAR_420_16BITS] = 8,
};

static unsigned avi_limit_r3_width_strip[] = {
	[AVI_LIMIT_UNKNOWN] = 8,
	[AVI_LIMIT_PACKED_444_16BITS] = 1,
	[AVI_LIMIT_PACKED_444_24BITS] = 1,
	[AVI_LIMIT_PACKED_444_32BITS] = 1,
	[AVI_LIMIT_PACKED_422_16BITS] = 2,
	[AVI_LIMIT_PLANAR_422_16BITS] = 2,
	[AVI_LIMIT_PLANAR_420_16BITS] = 2,
};

/* left constraints (derivates from start adress limitations) */
static unsigned avi_limit_r2_left_strip[] = {
	[AVI_LIMIT_UNKNOWN]           = 8,
	[AVI_LIMIT_PACKED_444_16BITS] = 4,
	[AVI_LIMIT_PACKED_444_24BITS] = 8,
	[AVI_LIMIT_PACKED_444_32BITS] = 2,
	[AVI_LIMIT_PACKED_422_16BITS] = 4,
	[AVI_LIMIT_PLANAR_422_16BITS] = 4,
	[AVI_LIMIT_PLANAR_420_16BITS] = 4,
};

static unsigned avi_limit_r3_left_strip[] = {
	[AVI_LIMIT_UNKNOWN]           = 8,
	[AVI_LIMIT_PACKED_444_16BITS] = 1,
	[AVI_LIMIT_PACKED_444_24BITS] = 1,
	[AVI_LIMIT_PACKED_444_32BITS] = 1,
	[AVI_LIMIT_PACKED_422_16BITS] = 1,
	[AVI_LIMIT_PLANAR_422_16BITS] = 2,
	[AVI_LIMIT_PLANAR_420_16BITS] = 2,
};

static enum avi_pixfmt_type avi_limit_get_fmttype(struct avi_dma_pixfmt pixfmt)
{
	if (pixfmt.semiplanar == 0) {
		if (pixfmt.subsampling == AVI_FIFO_CFG_422_FMT)
			return AVI_LIMIT_PACKED_422_16BITS;
		else if (pixfmt.subsampling == AVI_FIFO_CFG_444_FMT)
			switch (pixfmt.bitenc) {
			case AVI_FIFO_CFG_4444_BENC:
			case AVI_FIFO_CFG_565_BENC:
			case AVI_FIFO_CFG_LSB555_BENC:
			case AVI_FIFO_CFG_1555_BENC:
				return AVI_LIMIT_PACKED_444_16BITS;
			case AVI_FIFO_CFG_888_BENC:
				return AVI_LIMIT_PACKED_444_24BITS;
			case AVI_FIFO_CFG_8888_BENC:
				return AVI_LIMIT_PACKED_444_32BITS;
			default:
				return AVI_LIMIT_UNKNOWN;
			};
	}
	else if (pixfmt.semiplanar == 1) {
		if (pixfmt.subsampling == AVI_FIFO_CFG_420_FMT)
			return AVI_LIMIT_PLANAR_420_16BITS;
		else if (pixfmt.subsampling == AVI_FIFO_CFG_422_FMT)
			return AVI_LIMIT_PLANAR_422_16BITS;
	}
	return AVI_LIMIT_UNKNOWN;
}

static unsigned *avi_limit_get_limits(enum avi_limit_type type)
{
	int revision = avi_get_revision();

	switch(type) {
	case AVI_LIMIT_HEIGHT:
	case AVI_LIMIT_HEIGHT_STRIP:
	case AVI_LIMIT_TOP:
		return avi_limit_height;
	case AVI_LIMIT_WIDTH:
		return avi_limit_width;
	case AVI_LIMIT_WIDTH_STRIP:
		if (revision == AVI_REVISION_2)
			return avi_limit_r2_width_strip;
		else
			return avi_limit_r3_width_strip;
	case AVI_LIMIT_LEFT:
		if (revision == AVI_REVISION_2)
			return avi_limit_r2_left_strip;
		else
			return avi_limit_r3_left_strip;
	}

	pr_err("%s:unknown type %d with revision %d\n",
	         __func__, type, revision);
	BUG_ON(1);
	return NULL;
}

/* Round down val according to limitations corresponding to pixel format in
 * the valid range for pixel format. */
static void avi_limit_adjust(struct avi_dma_pixfmt pixfmt,
                             unsigned *val,
                             enum avi_limit_type type)
{
	enum avi_pixfmt_type     fmttype = avi_limit_get_fmttype(pixfmt);
	unsigned                *limitations = avi_limit_get_limits(type);
	unsigned                 limit = limitations[fmttype];

	*val  = min(AVI_LIMIT_MAX, round_down(*val,  limit));

	/* We need at least a positive value for width and height */
	if ((type != AVI_LIMIT_LEFT) && (type != AVI_LIMIT_TOP)
	    && (*val < limit))
		*val = limit;

	/* P7R2 semiplanar limitation : width have to be at least 112 pixels */
	if (AVI_REVISION_2 == avi_get_revision() && pixfmt.semiplanar
	    && ((type == AVI_LIMIT_WIDTH) || (type == AVI_LIMIT_WIDTH_STRIP))
	    && (*val < 112))
		*val = 112;
}

/*******************************************************************************
 * Provided helpers
 ******************************************************************************/

void avi_limit_adjust_height(struct avi_dma_pixfmt pixfmt, unsigned *h)
{
	*h = min((unsigned) AVI_LIMIT_MAX_HEIGHT, *h);
	avi_limit_adjust(pixfmt, h, AVI_LIMIT_HEIGHT);
}
EXPORT_SYMBOL(avi_limit_adjust_height);

/* FIXME : Add Semi-planar 420 constraint. As pixels blocs are 2 pixel high, the
 * AVI FIFO need to contain a complete line. The minimal FIFO depth is 4Ko so we
 * cant have more than 4096 width */
void avi_limit_adjust_width(struct avi_dma_pixfmt pixfmt, unsigned *w)
{
	/* P7R2 semiplanar limitation : width have to be at least 112 pixels */
	if ((AVI_REVISION_2 == avi_get_revision())
	     && pixfmt.semiplanar
	     && (*w < 112))
		*w = 112;

	*w = min((unsigned) AVI_LIMIT_MAX_WIDTH, *w);

	avi_limit_adjust(pixfmt, w, AVI_LIMIT_WIDTH);
}
EXPORT_SYMBOL(avi_limit_adjust_width);

/* This function adjust both left and width strip values */
void avi_limit_adjust_w_strip(struct avi_dma_pixfmt pixfmt,
                              unsigned w,
                              unsigned *strip_w,
                              unsigned *left)
{
	/* We cant strip more than image size */
	*strip_w = min(*strip_w, w);
	avi_limit_adjust(pixfmt, strip_w, AVI_LIMIT_WIDTH_STRIP);

	/* if strip is out of bounds we reduce left */
	*left = min(*left, (w - *strip_w));
	avi_limit_adjust(pixfmt, left, AVI_LIMIT_LEFT);
}
EXPORT_SYMBOL(avi_limit_adjust_w_strip);

/* This function adjust both top and height strip values */
void avi_limit_adjust_h_strip(struct avi_dma_pixfmt pixfmt,
                              unsigned h,
                              unsigned *strip_h,
                              unsigned *top)
{
	/* We cant strip more than image size*/
	*strip_h = min(*strip_h, h);
	avi_limit_adjust(pixfmt, strip_h, AVI_LIMIT_HEIGHT_STRIP);

	/* if strip is out of bounds we reduce top */
	*top = min(*top, (h - *strip_h));
	avi_limit_adjust(pixfmt, top, AVI_LIMIT_TOP);
}
EXPORT_SYMBOL(avi_limit_adjust_h_strip);

MODULE_AUTHOR("Victor Lambret <victor.lambret.ext@parrot.com>");
MODULE_DESCRIPTION("Parrot Advanced Video Interface Limitations layer");
MODULE_LICENSE("GPL");
