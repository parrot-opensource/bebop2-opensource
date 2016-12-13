/*
 *      linux/drivers/parrot/video/segment_v4l2.c
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

#include <linux/module.h>
#include "avi_v4l2.h"

#ifdef DEBUG
#define dprintk(format_, args...) \
	pr_debug("%s: " format_, __func__, ##args)

#else /* DEBUG */
#define dprintk(format_, args...)
#endif /* DEBUG */

struct avi_v4l2_pixfmt {
	struct avi_dma_pixfmt	avi_fmt;
	u32			v4l2_fmt;
	char			description[32];
};

static const struct avi_v4l2_pixfmt avi_v4l2_formats[] = {
	{
		.avi_fmt = AVI_PIXFMT_YVYU,
		.v4l2_fmt = V4L2_PIX_FMT_YVYU,
		.description = "16bpp, packed, 4:2:2 YVYU"
	},
	{
		.avi_fmt = AVI_PIXFMT_AYUV,
		.v4l2_fmt = V4L2_PIX_FMT_YUV32,
		.description = "32bpp, 4:4:4 AYUV"
	},
	{
		.avi_fmt = AVI_PIXFMT_YUYV,
		.v4l2_fmt = V4L2_PIX_FMT_YUYV,
		.description = "16bpp, packed, 4:2:2 YUYV"
	},
	{
		.avi_fmt = AVI_PIXFMT_YYUV,
		.v4l2_fmt = V4L2_PIX_FMT_YYUV,
		.description = "16bpp, packed, 4:2:2 YYUV"
	},
	{
		.avi_fmt = AVI_PIXFMT_UYVY,
		.v4l2_fmt = V4L2_PIX_FMT_UYVY,
		.description = "16bpp, packed, 4:2:2 UYVY"
	},
	{
		.avi_fmt = AVI_PIXFMT_VYUY,
		.v4l2_fmt = V4L2_PIX_FMT_VYUY,
		.description = "16bpp, packed, 4:2:2 VYUY"
	},
	{
		.avi_fmt = AVI_PIXFMT_GREY,
		.v4l2_fmt = V4L2_PIX_FMT_GREY,
		.description = "8bpp greyscale"
	},
	{
		.avi_fmt = AVI_PIXFMT_Y10,
		.v4l2_fmt = V4L2_PIX_FMT_Y10,
		.description = "10bpp greyscale"
	},
	{
		.avi_fmt = AVI_PIXFMT_RGBA8888,
		.v4l2_fmt = V4L2_PIX_FMT_RGB32,
		.description = "32bpp, RGBA"
	},
	{
		.avi_fmt = AVI_PIXFMT_BGRA8888,
		.v4l2_fmt = V4L2_PIX_FMT_BGR32,
		.description = "32bpp, BGRA"
	},
	{
		.avi_fmt = AVI_PIXFMT_RGB888,
		.v4l2_fmt = V4L2_PIX_FMT_RGB24,
		.description = "24bpp, RGB"
	},
	{
		.avi_fmt = AVI_PIXFMT_BGR888,
		.v4l2_fmt = V4L2_PIX_FMT_BGR24,
		.description = "24bpp, BGR"
	},
	{
		.avi_fmt = AVI_PIXFMT_RGB565,
		.v4l2_fmt = V4L2_PIX_FMT_RGB565,
		.description = "16bpp, RGB565"
	},
	{
		.avi_fmt = AVI_PIXFMT_RGBA5551,
		.v4l2_fmt = V4L2_PIX_FMT_RGB555,
		.description = "16bpp, RGBA5551"
	},
	{
		.avi_fmt = AVI_PIXFMT_NV16,
		.v4l2_fmt = V4L2_PIX_FMT_NV16,
		.description = "16bpp, semi-planar, 4:2:2 Y/UV"
	},
	{
		.avi_fmt = AVI_PIXFMT_NV61,
		.v4l2_fmt = V4L2_PIX_FMT_NV61,
		.description = "16bpp, semi-planar, 4:2:2 Y/VU"
	},
	{
		.avi_fmt = AVI_PIXFMT_NV12,
		.v4l2_fmt = V4L2_PIX_FMT_NV12,
		.description = "12bpp, semi-planar, 4:2:0 Y/UV"
	},
	{
		.avi_fmt = AVI_PIXFMT_NV21,
		.v4l2_fmt = V4L2_PIX_FMT_NV21,
		.description = "12bpp, semi-planar, 4:2:0 Y/VU"
	},

	/* Bayer formats, only available when we have a raw bayer input. */
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X10_16,
		/* I'm not following the spec here because in this format the
		 * data should be packed on the LSB, the 6 remaining MSB padded
		 * with 0s. However the AVI does not support this format, so I
		 * make do. We could lie and pretend we store 16bits but our
		 * current v4l2 version does not seem to support 16bit raw
		 * pixelcodes. */
		.v4l2_fmt = V4L2_PIX_FMT_SBGGR10,
		.description = "16bpp, 10bit raw bayer [BG/GR]",
	},
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X10_16,
		.v4l2_fmt = V4L2_PIX_FMT_SGBRG10,
		.description = "16bpp, 10bit raw bayer [GB/RG]",
	},
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X10_16,
		.v4l2_fmt = V4L2_PIX_FMT_SGRBG10,
		.description = "16bpp, 10bit raw bayer [GR/BG]",
	},
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X10_16,
		.v4l2_fmt = V4L2_PIX_FMT_SRGGB10,
		.description = "16bpp, 10bit raw bayer [RG/GB]",
	},
	/* 12bit bayer formats */
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X12_16,
		/* This format is completely nonstandard, the bits are shifted
		 * around. */
		.v4l2_fmt = V4L2_PIX_FMT_SBGGR12,
		.description = "16bpp, 12bit raw bayer [BG/GR]",
	},
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X12_16,
		.v4l2_fmt = V4L2_PIX_FMT_SGBRG12,
		.description = "16bpp, 12bit raw bayer [GB/RG]",
	},
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X12_16,
		.v4l2_fmt = V4L2_PIX_FMT_SGRBG12,
		.description = "16bpp, 12bit raw bayer [GR/BG]",
	},
	{
		.avi_fmt = AVI_PIXFMT_BAYER_1X12_16,
		.v4l2_fmt = V4L2_PIX_FMT_SRGGB12,
		.description = "16bpp, 12bit raw bayer [RG/GB]",
	},
};

int avi_v4l2_enum_fmt(struct v4l2_fmtdesc *f)
{
	const struct avi_v4l2_pixfmt *fmt;

	if (f->index >= ARRAY_SIZE(avi_v4l2_formats))
		return -EINVAL;

	fmt = &avi_v4l2_formats[f->index];

	f->flags       = 0;
	f->pixelformat = fmt->v4l2_fmt;

	strlcpy(f->description, fmt->description, sizeof(f->description));

	return 0;
}
EXPORT_SYMBOL(avi_v4l2_enum_fmt);

enum avi_v4l2_pixtype avi_v4l2_get_pixfmt_type(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_YUV32:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_YYUV:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
	case V4L2_PIX_FMT_NV16:
	case V4L2_PIX_FMT_NV61:
		return AVI_PIXTYPE_YUV;
	case V4L2_PIX_FMT_GREY:
	case V4L2_PIX_FMT_Y10:
		return AVI_PIXTYPE_GREY;
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB555:
		return AVI_PIXTYPE_RGB;
	case V4L2_PIX_FMT_SBGGR10:
	case V4L2_PIX_FMT_SGRBG10:
	case V4L2_PIX_FMT_SGBRG10:
	case V4L2_PIX_FMT_SRGGB10:
	case V4L2_PIX_FMT_SBGGR12:
	case V4L2_PIX_FMT_SGRBG12:
	case V4L2_PIX_FMT_SGBRG12:
	case V4L2_PIX_FMT_SRGGB12:
		return AVI_PIXTYPE_BAYER;
	}

	dprintk("unknown format type 0x%08x '" FOURCC_FORMAT "'\n",
		fourcc, FOURCC_SHOW(fourcc));

	return AVI_PIXTYPE_UNKNOWN;
}
EXPORT_SYMBOL(avi_v4l2_get_pixfmt_type);

struct avi_dma_pixfmt avi_v4l2_to_avi_pixfmt(u32 v4l2_fmt)
{
	int i ;

	for (i = 0; i < ARRAY_SIZE(avi_v4l2_formats); i++)
		if (v4l2_fmt == avi_v4l2_formats[i].v4l2_fmt)
			return avi_v4l2_formats[i].avi_fmt;

	return AVI_PIXFMT_INVALID;
}
EXPORT_SYMBOL(avi_v4l2_to_avi_pixfmt);

static u32 avi_v4l2_lnsize(struct avi_dma_pixfmt pixfmt,
			   u32 frm_width,
			   u32 requested_lnsize)
{
	struct avi_segment_format       fmt;

	fmt.pix_fmt = pixfmt;
	avi_dma_setup_line_size(&fmt, frm_width, requested_lnsize);

	return max(fmt.plane0.line_size, fmt.plane1.line_size);
}

static u32 avi_v4l2_frmsize(struct avi_dma_pixfmt pixfmt,
			    unsigned int line_size,
			    u32 frm_height)
{
	switch (avi_pixfmt_get_packing(pixfmt)) {
	case AVI_INTERLEAVED_444_PACKING:
	case AVI_INTERLEAVED_YUV_422_PACKING:
		return frm_height * line_size;
	case AVI_SEMIPLANAR_YUV_420_PACKING:
		return (frm_height + frm_height / 2)* line_size;
	case AVI_SEMIPLANAR_YUV_422_PACKING:
		return frm_height * 2 * line_size;
	default:
		BUG();
	}

	return 0;
}


int avi_v4l2_to_segment_fmt(const struct v4l2_pix_format	*pix,
			    const struct v4l2_rect		*crop,
			    struct avi_segment_format		*avi_fmt,
			    unsigned				*plane0_size,
			    unsigned				*plane1_size)
{
	static const struct avi_segment_format avi_zero_fmt = { 0 };

	*avi_fmt = avi_zero_fmt;

	avi_fmt->pix_fmt = avi_v4l2_to_avi_pixfmt(pix->pixelformat);
	if (avi_fmt->pix_fmt.id == AVI_INVALID_FMTID) {
		dprintk("bad pixel format %d\n", pix->pixelformat);
		return -EINVAL;
	}

	if (crop) {
		avi_fmt->width	    = crop->width;
		avi_fmt->height	    = crop->height;
	} else {
		/* Use the full frame */
		avi_fmt->width	    = pix->width;
		avi_fmt->height	    = pix->height;
	}

	avi_fmt->interlaced = (pix->field != V4L2_FIELD_NONE);

	/* There is no V4L2 colorspace for 'GREY' so we specify it manually */
	if (avi_v4l2_get_pixfmt_type(pix->pixelformat) == AVI_PIXTYPE_GREY) {
		if (avi_fmt->pix_fmt.bytes_per_pixel0 == 0)
			avi_fmt->colorspace = AVI_GREY_CSPACE;
		else
			avi_fmt->colorspace = AVI_Y10_CSPACE;
	}
	else
		avi_fmt->colorspace = avi_v4l2_to_avi_cspace(pix->colorspace);

	if (!avi_fmt->pix_fmt.raw && avi_fmt->colorspace == AVI_NULL_CSPACE) {
		dprintk("bad colorspace %d\n", pix->colorspace);
		return -EINVAL;
	}

	/* pix_fmt is valid only for DMA(in/out) segment format
	 * so line sizes and plane sizes can be computed only in these cases
	 */
	if (avi_fmt->pix_fmt.id == AVI_INVALID_FMTID) {
		*plane0_size = 0;
		*plane1_size = 0;
	} else {
		avi_dma_setup_line_size(avi_fmt, pix->width, pix->bytesperline);
		avi_dma_setup_plane_size(avi_fmt,
		                         pix->height,
		                         plane0_size,
		                         plane1_size);
	}

	if (pix->field == V4L2_FIELD_INTERLACED) {
		/* We will store one field at a time, which means that we want
		 * to jump one line after each line to skip the other field */
		avi_fmt->plane0.line_size *= 2;
		avi_fmt->plane1.line_size *= 2;
	}

	return 0;
}
EXPORT_SYMBOL(avi_v4l2_to_segment_fmt);

void avi_v4l2_setup_dmabuf(const struct v4l2_rect		*crop,
			   const struct avi_segment_format	*fmt,
			   unsigned				 plane0_size,
			   unsigned				 plane1_size,
			   dma_addr_t				 addr,
			   void					*priv,
			   struct avi_dma_buffer		*dma_buf)
{
	dma_addr_t	start0 = addr;
	dma_addr_t	start1 = 0;
	unsigned	top;
	unsigned        left;

	if (crop) {
		top  = crop->top;
		left = crop->left;
	} else {
		top  = 0;
		left = 0;
	}

	/* start address of transfered data in plane 0 */
	start0 += top * fmt->plane0.line_size
		+ left * avi_pixel_size0(fmt->pix_fmt);

	/* start address in plane 1 is valid only for semi-planar format */
	if (avi_pixfmt_is_planar(fmt->pix_fmt)) {
		/* Plane 1 follows Plane 0 */
		start1 = addr + plane0_size;
		/* in case of NV12 (4:2:0) height of chroma plane (plane 1)
		 * is divided by 2
		 */
		/* start address of transfered data in plane 1: start of line */
		if (avi_pixfmt_get_packing(fmt->pix_fmt) ==
		    AVI_SEMIPLANAR_YUV_420_PACKING)
			start1 += top / 2 * fmt->plane1.line_size;
		else
			start1 += top * fmt->plane1.line_size;
		/* add offset in line */
		start1 += left * avi_pixel_size1(fmt->pix_fmt);
	}

	dma_buf->plane0.dma_addr = start0;
	dma_buf->plane1.dma_addr = start1;
	dma_buf->priv		 = priv;
	dma_buf->status		 = AVI_BUFFER_READY;
}
EXPORT_SYMBOL(avi_v4l2_setup_dmabuf);

int avi_v4l2_try_fmt(__u32 type, struct v4l2_pix_format *pix)
{
	struct avi_dma_pixfmt	pixfmt = AVI_PIXFMT_INVALID;
	enum avi_colorspace	cspace = AVI_NULL_CSPACE;

	/* check if pixelformat can deal with AVI */
	pixfmt = avi_v4l2_to_avi_pixfmt(pix->pixelformat);
	if (pixfmt.id == AVI_INVALID_FMTID) {
		dprintk("unknown pixel format " FOURCC_FORMAT
		        ", replaced by default\n",
			FOURCC_SHOW(pix->pixelformat));
		/* Format is unknown, we replace it by a default format */
		pixfmt = avi_v4l2_formats[0].avi_fmt;
		pix->pixelformat = avi_v4l2_formats[0].v4l2_fmt;
	}

	if (pixfmt.raw) {
		/* No meaningful colorspace for raw formats */
		cspace		= AVI_NULL_CSPACE;
		pix->colorspace = 0;

	} else {

#ifdef CONFIG_AVI_V4L2_GST_COLORSPACE_HACK
		/* gstreamer v4l2sink and src do not handle the colorspace
		 * correctly, they set bad values in the pix structure
		 * (actually, they don't set anything, they use whatever they
		 * get in g_fmt). Since we cannot trust the value we force the
		 * heuristic below to guess the colorspace */
		cspace = AVI_NULL_CSPACE;
#else
		cspace = avi_v4l2_to_avi_cspace(pix->colorspace);
#endif

		if (cspace == AVI_NULL_CSPACE) {
			/* Unknown or unsupported colorspace. Let's try to guess
			 * a sane value. */
			switch (avi_v4l2_get_pixfmt_type(pix->pixelformat)) {
			case AVI_PIXTYPE_RGB:
				pix->colorspace = V4L2_COLORSPACE_SRGB;
				break;
			case AVI_PIXTYPE_YUV:
				pix->colorspace = (pix->height <= 576) ?
					/* Use SMPTE170M for SD output */
					V4L2_COLORSPACE_SMPTE170M :
					V4L2_COLORSPACE_REC709;
				break;
			case AVI_PIXTYPE_GREY:
				if (pixfmt.bytes_per_pixel0 == 0)
					cspace = AVI_GREY_CSPACE;
				else
					cspace = AVI_Y10_CSPACE;
				break;
			default:
				return -EINVAL;
			}
		}
	}

	/* Adjust image size */
	avi_limit_adjust_height(pixfmt, &pix->height);
	avi_limit_adjust_width(pixfmt, &pix->width);

	pix->priv = 0;

	pix->bytesperline = avi_v4l2_lnsize(pixfmt,
	                                    pix->width,
	                                    pix->bytesperline);
	pix->sizeimage = avi_v4l2_frmsize(pixfmt,
	                                  pix->bytesperline,
	                                  pix->height);

	return 0;
}
EXPORT_SYMBOL(avi_v4l2_try_fmt);

static inline void avi_v4l2_crop_fit(int *pos, int *size, int posmin, int bound)
{
	if (*pos < posmin)
		*pos = posmin;

	if (*size <= 0)
		*size = bound;

	if (*size > bound)
		*size = bound;

	if ((*pos + *size) > bound)
		*pos = bound - *size;
}

void avi_v4l2_crop_adjust(struct v4l2_rect       *crop,
			  const struct v4l2_rect *bounds)
{
	avi_v4l2_crop_fit(&crop->left,
			  &crop->width,
			  bounds->left,
			  bounds->width);

	avi_v4l2_crop_fit(&crop->top,
			  &crop->height,
			  bounds->top,
			  bounds->height);
}
EXPORT_SYMBOL(avi_v4l2_crop_adjust);

struct avi_v4l2_device
{
	struct mutex        lock;
	unsigned            refcount;
	struct v4l2_device  dev;
	struct media_device media;
}avi_v4l2_device;

struct v4l2_device *avi_v4l2_get_device(void)
{
	struct v4l2_device	*dev   = &avi_v4l2_device.dev;
	struct media_device	*media = &avi_v4l2_device.media;
	int			 ret;

	if (avi_v4l2_device.refcount == 0) {
		memset(&avi_v4l2_device, 0, sizeof(avi_v4l2_device));
		strlcpy(avi_v4l2_device.media.bus_info, "AVI", sizeof(avi_v4l2_device.media.bus_info));
		strlcpy(avi_v4l2_device.media.model, "AVI", sizeof(avi_v4l2_device.media.model));

		mutex_init(&avi_v4l2_device.lock);
		mutex_lock(&avi_v4l2_device.lock);
		media->hw_revision = avi_get_revision();
		media->dev         = avi_ctrl.dev;

		ret = media_device_register(media);
		if (ret) {
			dev = NULL;
			goto unlock;
		}

		dev->mdev = media;

		ret = v4l2_device_register(avi_ctrl.dev, dev);
		if (ret) {
			dev = NULL;
			media_device_unregister(media);
			goto unlock;
		}
	}
	else
		mutex_lock(&avi_v4l2_device.lock);

	avi_v4l2_device.refcount++;

 unlock:
	mutex_unlock(&avi_v4l2_device.lock);

	return dev;
}
EXPORT_SYMBOL(avi_v4l2_get_device);

void avi_v4l2_put_device(struct v4l2_device *dev)
{
	struct media_device *media = &avi_v4l2_device.media;

	BUG_ON(dev != &avi_v4l2_device.dev);

	mutex_lock(&avi_v4l2_device.lock);

	avi_v4l2_device.refcount--;

	if (avi_v4l2_device.refcount == 0) {
		v4l2_device_unregister(dev);
		dev->mdev = NULL;
		media_device_unregister(media);
	}

	mutex_unlock(&avi_v4l2_device.lock);
}
EXPORT_SYMBOL(avi_v4l2_put_device);

MODULE_AUTHOR("Victor Lambret <victor.lambret.ext@parrot.com>");
MODULE_DESCRIPTION("Parrot AVI V4L2 layer");
MODULE_LICENSE("GPL");
