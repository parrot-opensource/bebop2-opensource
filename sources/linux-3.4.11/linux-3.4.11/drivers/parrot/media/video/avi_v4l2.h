/*
 *      linux/drivers/parrot/media/video/avi_v4l2.h
 *
 *      Copyright (C) 2014 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    04-Feb-2014
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

#ifndef _AVI_V4L2_H_
#define _AVI_V4L2_H_

#include <linux/videodev2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include "video/avi_segment.h"
#include "video/avi_pixfmt.h"
#include "video/avi_limit.h"

#define AVI_V4L2_BUS_VERSION    "platform:parrot7:avi"

/****************************************
 * Set of macros to pretty print FourCCs
 ****************************************/
#define FOURCC_FORMAT "%c%c%c%c"
#define FOURCC_SHOW(_cc)			\
		((_cc)        & 0xff),		\
		(((_cc) >> 8) & 0xff),		\
		(((_cc) >> 16) & 0xff),		\
		(((_cc) >> 24) & 0xff)

enum avi_v4l2_pixtype {
	AVI_PIXTYPE_UNKNOWN,
	AVI_PIXTYPE_RGB,
	AVI_PIXTYPE_YUV,
	AVI_PIXTYPE_BAYER,
	AVI_PIXTYPE_GREY,
};

extern enum avi_v4l2_pixtype avi_v4l2_get_pixfmt_type(u32);

/** convert a V4L2 pixel format (fourcc) to a AVI pixel format
 *
 * @v4l2_fmt: fourcc pixel format
 *
 * Return a struct avi_dma_pixfmt
 */
extern struct avi_dma_pixfmt avi_v4l2_to_avi_pixfmt(u32 v4l2_fmt);

/** Convert V4L2 configuration into AVI segment format
 *
 * Input parameters:
 * @v4l2_pix: struct v4l2_pix_format defining the whole frame
 *
 * @rect: defining the cropped area into whole frame
 *
 * Output parameters:
 * @avi_fmt: AVI segment format filled by this function
 *
 * @plane0_size: size in bytes of plane 0, filled by this function
 * @plane1_size: size in bytes of plane 1, filled by this function
 *               they are useful for computing addresses in buffer
 *               and size of image
 */
extern int
avi_v4l2_to_segment_fmt(const struct v4l2_pix_format    *pix,
                        const struct v4l2_rect          *rect,
                        struct avi_segment_format       *avi_fmt,
                        unsigned                        *plane0_size,
                        unsigned                        *plane1_size);

/** Setup a avi_dma_buffer
 *
 * Input parameters:
 * @rect: defining the cropped area into whole frame
 * @seg_conf: AVI segment format set by avi_v4l2_convert_format()
 * @plane0_size: size in bytes of plane 0, set by avi_v4l2_convert_format()
 * @plane1_size: size in bytes of plane 1, set by avi_v4l2_convert_format()
 * @addr: physical starting address of buffer for the whole frame
 * @priv: value to set in priv field of dma_buf
 *
 * Output parameters:
 * @dma_buf: AVI DMA buffer, filled by this function, suitable to be used
 *           by avi_segment_set_input_buffer() or
 *           avi_segment_set_output_buffer()
 */
extern void avi_v4l2_setup_dmabuf(const struct v4l2_rect          *rect,
				  const struct avi_segment_format *fmt,
				  unsigned                         plane0_size,
				  unsigned                         plane1_size,
				  dma_addr_t                       addr,
				  void                            *priv,
				  struct avi_dma_buffer           *dma_buf);

/* enumerate V4L2 formats */
extern int avi_v4l2_enum_fmt(struct v4l2_fmtdesc* fmt);

/** try a V4L2 format
 * used by IOCTL functions vidioc_try_fmt_xxxx
 *
 * Return -EINVAL if pixel format or colorspace are not available on AVI
 */
extern int avi_v4l2_try_fmt(__u32 type, struct v4l2_pix_format *pix);

extern void avi_v4l2_crop_adjust(struct v4l2_rect       *crop,
				 const struct v4l2_rect *bounds);

static inline enum avi_colorspace avi_v4l2_to_avi_cspace(enum v4l2_colorspace c)
{
	switch (c) {
	case V4L2_COLORSPACE_SMPTE170M:
		return AVI_BT601_CSPACE;
	case V4L2_COLORSPACE_REC709:
		return AVI_BT709_CSPACE;
	case V4L2_COLORSPACE_JPEG:
		return AVI_JFIF_CSPACE;
	case V4L2_COLORSPACE_SRGB:
		return AVI_RGB_CSPACE;
	default:
		/* Unknown colorspace */
		return AVI_NULL_CSPACE;
	}
}

extern struct v4l2_device  *avi_v4l2_get_device(void);
extern void                 avi_v4l2_put_device(struct v4l2_device *);

#endif /* _AVI_V4L2_H_ */
