#ifndef _AVI_CHROMA_H_
#define _AVI_CHROMA_H_

#include "avi_compat.h"

/*
 *  linux/drivers/parrot/video/avi_chroma.h
 *
 *  Copyright (C) 2014 Parrot S.A.
 *
 * @author  Lionel Flandrin <lionel.flandrin@parrot.com>
 * @date  22-Dec-2014
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


/***********************
 * Chroma converter API
 ***********************/
enum avi_colorspace {
	AVI_NULL_CSPACE = 0,
	AVI_RGB_CSPACE,
	AVI_BT601_CSPACE,
	AVI_BT709_CSPACE,
	AVI_JFIF_CSPACE,
	AVI_GREY_CSPACE,
	AVI_Y10_CSPACE,
	AVI_CSPACE_NR,
};

extern struct avi_isp_chroma_regs const avi_conv_cfg[AVI_CSPACE_NR][AVI_CSPACE_NR];

extern void avi_setup_conv_colorspace(struct avi_node const *,
				      enum avi_colorspace, enum avi_colorspace);


extern void avi_conv_get_registers(struct avi_node const*,
                                   struct avi_isp_chroma_regs*);


/* Convert a 888 YUV or RGB color (depending on colorspace `from`) to the same
 * color in colorspace `to` */
extern u32 avi_conv_convert_color(enum avi_colorspace from,
				  enum avi_colorspace to,
				  u32                 color);

#endif /* _AVI_CHROMA_H_ */
