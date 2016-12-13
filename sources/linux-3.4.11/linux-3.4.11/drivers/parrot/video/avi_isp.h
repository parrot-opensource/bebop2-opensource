#ifndef _AVI_ISP_H_
#define _AVI_ISP_H_

/*
 *  linux/drivers/parrot/video/avi_isp.h
 *
 *  Copyright (C) 2013 Parrot S.A.
 *
 * @author  Eng-Hong Sron <eng-hong.sron@parrot.com>
 * @date  20-Jan-2014
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
 * This file contains the isp configuration code. It can be included from userland.
 */

#include <linux/types.h>
#include <linux/ioctl.h>

struct avi_isp_offsets {
	__u32 chain_bayer;
	__u32 gamma_corrector;
	__u32 chroma;
	__u32 statistics_yuv;
	__u32 chain_yuv;
};

/* ISP nodes offset */
#define AVI_ISP_IOGET_OFFSETS  _IOR('F', 0x33, struct avi_isp_offsets)

#ifdef __KERNEL__

struct avi_node;
struct avi_segment;

extern void avi_isp_chain_bayer_bypass(struct avi_node *);
extern void avi_isp_chain_yuv_bypass(struct avi_node *);

extern void avi_statistics_bayer_configure(struct avi_node *,
					   unsigned,
					   unsigned,
					   unsigned,
					   unsigned);

extern int avi_isp_get_offsets(struct avi_segment *,
			       struct avi_isp_offsets *);

/* Return the capability of the stat node corresponding to an ISP_CHAIN_BAYER
 * contained in the segment. Returns 0 if the segment doesn't contain an
 * ISP_CHAIN_BAYER. */
extern unsigned long avi_isp_bayer_get_stats_cap(struct avi_segment *);

#endif /* __KERNEL__ */

#endif /* _AVI_ISP_H_ */
