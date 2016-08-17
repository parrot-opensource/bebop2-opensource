#include <linux/module.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <parrot/avicam_dummy_dev.h>

#include "gpu/p7ump.h"

#include "avicam_v4l2.h"

static struct v4l2_subdev *
avicam_remote_subdev(struct avicam *avicam, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_source(&avicam->pad);

	if (remote == NULL ||
	    media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int avicam_queue_setup(struct vb2_queue *vq,
                        const struct v4l2_format *fmt,
                        unsigned int *count, unsigned int *num_planes,
                        unsigned int sizes[], void *alloc_ctxs[])
{
	struct avicam	*avicam = vb2_get_drv_priv(vq);

	sizes[0] = avicam->pix.sizeimage;
	alloc_ctxs[0] = avicam->alloc_ctx;
	*num_planes = 1;

	if (*count == 0)
		/* Default to 4 buffers */
		*count = 4;

	return 0;
}

static int avicam_vbuf_init(struct vb2_buffer* vb)
{
	struct avicam *avicam = vb2_get_drv_priv(vb->vb2_queue);

	/* Allow GPU to access this memory through UMP */
	p7ump_add_map(vb2_dma_contig_plane_dma_addr(vb, 0),
	              PAGE_ALIGN(avicam->pix.sizeimage));

	return 0;
}

static int avicam_vbuf_prepare(struct vb2_buffer* vb)
{
	struct avicam	*avicam = vb2_get_drv_priv(vb->vb2_queue);

	vb2_set_plane_payload(vb, 0, avicam->pix.sizeimage);
	vb->v4l2_buf.field = V4L2_FIELD_NONE;

	return 0;
}

/* This function is called with avicam->vbq_lock held (no need to protect
 * ourselves when we play with the dmaqueue) */
static void avicam_vbuf_queue(struct vb2_buffer *vb)
{
	struct avicam *avicam = vb2_get_drv_priv(vb->vb2_queue);
	struct avicam_vbuf *buf = to_avicam_vbuf(vb);
	unsigned long flags = 0;

	spin_lock_irqsave(&avicam->vbq_lock, flags);
	list_add_tail(&buf->list, &avicam->bufqueue);
	spin_unlock_irqrestore(&avicam->vbq_lock, flags);
}

static void avicam_vbuf_release(struct vb2_buffer *vb)
{
	struct avicam	*avicam = vb2_get_drv_priv(vb->vb2_queue);

	p7ump_remove_map(vb2_dma_contig_plane_dma_addr(vb, 0),
			 PAGE_ALIGN(avicam->pix.sizeimage));
}

struct avicam_mbus_desc {
	enum v4l2_mbus_pixelcode	code;
	enum avi_v4l2_pixtype		type;
	unsigned                        format_control;
	/* bits per pixel */
	char				bpp;
	/* clock per pixel (less is better) */
	char				cpp;
};

static const struct avicam_mbus_desc avicam_mbus_descs[] =
{
	/* YUV mbus formats */
	{
		.code		= V4L2_MBUS_FMT_YUYV8_1X16,
		.type		= AVI_PIXTYPE_YUV,
		.format_control = AVI_FORMAT_CONTROL_YUYV_1X16,
		.bpp		= 16,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_YUYV8_2X8,
		.type		= AVI_PIXTYPE_YUV,
		.format_control = AVI_FORMAT_CONTROL_YUYV_2X8,
		.bpp		= 16,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_YVYU8_2X8,
		.type		= AVI_PIXTYPE_YUV,
		.format_control = AVI_FORMAT_CONTROL_YVYU_2X8,
		.bpp		= 16,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_UYVY8_2X8,
		.type		= AVI_PIXTYPE_YUV,
		.format_control = AVI_FORMAT_CONTROL_UYVY_2X8,
		.bpp		= 16,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_VYUY8_2X8,
		.type		= AVI_PIXTYPE_YUV,
		.format_control = AVI_FORMAT_CONTROL_VYUY_2X8,
		.bpp		= 16,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_Y8_1X8,
		.type		= AVI_PIXTYPE_GREY,
		.format_control = AVI_FORMAT_CONTROL_YUV888_1X24,
		.bpp		= 8,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_Y10_1X10,
		.type		= AVI_PIXTYPE_GREY,
		.format_control = AVI_FORMAT_CONTROL_YUV888_1X24,
		.bpp		= 10,
		.cpp		= 1,
	},
	/* RGB formats */
	{
		.code		= V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
		.type		= AVI_PIXTYPE_RGB,
		.format_control = AVI_FORMAT_CONTROL_RGB555_2X8,
		.bpp		= 15,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_RGB555_2X8_PADHI_BE,
		.type		= AVI_PIXTYPE_RGB,
		.format_control = AVI_FORMAT_CONTROL_BGR555_2X8,
		.bpp		= 15,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_RGB565_2X8_LE,
		.type		= AVI_PIXTYPE_RGB,
		.format_control = AVI_FORMAT_CONTROL_RGB565_2X8,
		.bpp		= 16,
		.cpp		= 2,
	},
	{
		.code		= V4L2_MBUS_FMT_RGB565_2X8_BE,
		.type		= AVI_PIXTYPE_RGB,
		.format_control = AVI_FORMAT_CONTROL_BGR565_2X8,
		.bpp		= 16,
		.cpp		= 2,
	},

	/* Bayer formats: 10bits */
	{
		.code		= V4L2_MBUS_FMT_SBGGR10_1X10,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUV888_1X24,
		.bpp		= 10,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_SGRBG10_1X10,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUV888_1X24,
		.bpp		= 10,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_SGBRG10_1X10,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUV888_1X24,
		.bpp		= 10,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_SRGGB10_1X10,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUV888_1X24,
		.bpp		= 10,
		.cpp		= 1,
	},

	/* Bayer formats: 12bits.  */
	{
		.code		= V4L2_MBUS_FMT_SBGGR12_1X12,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUYV_1X16,
		.bpp		= 12,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_SGRBG12_1X12,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUYV_1X16,
		.bpp		= 12,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_SGBRG12_1X12,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUYV_1X16,
		.bpp		= 12,
		.cpp		= 1,
	},
	{
		.code		= V4L2_MBUS_FMT_SRGGB12_1X12,
		.type		= AVI_PIXTYPE_BAYER,
		.format_control = AVI_FORMAT_CONTROL_YUYV_1X16,
		.bpp		= 12,
		.cpp		= 1,
	},
};

static const struct avicam_mbus_desc *
avicam_get_desc(enum v4l2_mbus_pixelcode c)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(avicam_mbus_descs); i++)
		if (avicam_mbus_descs[i].code == c)
			return &avicam_mbus_descs[i];

	/* Not found */
	return NULL;
}

/* This function attempts to find a bus configuration between the subdevice and
 * the avicam. In order do to that we iterate over the subdevice's formats and
 * we try to find the most suitable one (least transformations, best picture
 * quality etc...) */
static int avicam_pixfmt_to_mbus(struct avicam			*avicam,
                                 u32				 fmt,
                                 enum v4l2_mbus_pixelcode	*c)
{
	struct v4l2_subdev		*sd;
	int				 ret;
	enum avi_v4l2_pixtype		 dst_type;
	const struct avicam_mbus_desc	*best = NULL;
	struct v4l2_subdev_mbus_code_enum code;

	dst_type = avi_v4l2_get_pixfmt_type(fmt);
	if (dst_type == AVI_PIXTYPE_UNKNOWN)
		/* Unsupported or unknown pixel format */
		return -EINVAL;

	sd = avicam_remote_subdev(avicam, &code.pad);
	if(!sd)
		return -ENODEV;

	for (code.index = 0;; code.index++) {
		const struct avicam_mbus_desc *desc;

		ret = v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &code);
		if (ret)
			/* We're done enumerating the formats */
			break;

		desc = avicam_get_desc(code.code);
		if (desc == NULL)
			/* Unsupported or unknown bus format */
			continue;

		/* Check if the bus is wide enough to support this format. I
		 * just divide the bits per pixel with the number of clock ticks
		 * per pixel, it seems to work with all the formats I've
		 * considered but if it breaks somehow with more exotic
		 * subdevices we should add a "bus_width" member to the
		 * avicam_mbus_desc structure. */
		if (desc->bpp > avicam->pdata->bus_width * desc->cpp)
			/* We can't handle this bus width */
			continue;

		if (best == NULL) {
			/* Anything is better than nothing... */
			best = desc;
			continue;
		}

		/* Always favour similar types to avoid useless conversions */
		if (best->type != desc->type) {
			if (desc->type == dst_type)
				best = desc;
			continue;
		}

		if (best->bpp != desc->bpp) {
			if (best->bpp < desc->bpp)
				best = desc;
			continue;
		}

		/* Favour the format with the least clock ticks per pixel in an
		 * effort to reduce the clock frequency. */
		if (best->cpp != desc->cpp) {
			if (best->cpp > desc->cpp)
				best = desc;
			continue;
		}
	}

	if (best == NULL) {
		dev_err(avicam->dev,
			"Could not find any valid subdev bus configuration");
		return -EREMOTEIO;
	}

	if (dst_type == AVI_PIXTYPE_BAYER && best->type != dst_type)
		/* We can't substitute a YUV/RGB format for bayer output, it
		 * doesn't make any sense to convert from these formats into
		 * bayer. */
		best = NULL;

	if (c)
		*c = best->code;

	return 0;
}

static int avicam_querycap(struct file			*file,
			   void				*fh,
			   struct v4l2_capability	*cap)
{
	struct avicam *avicam = video_drvdata(file);

	mutex_lock(&avicam->lock);

	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "avicam.%d",
		 avicam_instance(avicam));

	cap->version	  = DRIVER_VERSION;
	cap->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	mutex_unlock(&avicam->lock);

	return 0;
}

static int avicam_enum_fmt_vid_cap(struct file		*file,
				   void			*priv,
				   struct v4l2_fmtdesc	*f)
{
	return avi_v4l2_enum_fmt(f);
}

static int avicam_g_fmt_vid_cap(struct file		*file,
                                void			*priv,
                                struct v4l2_format	*f)
{
	struct avicam	*avicam = video_drvdata(file);
	int		 ret;

	mutex_lock(&avicam->lock);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ret = -EINVAL;
		goto unlock;
	}

	f->fmt.pix = avicam->pix;

	ret = 0;

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_try_fmt_vid_cap_unlocked(struct avicam	*avicam,
                                           struct v4l2_format	*f)
{
	struct v4l2_pix_format		*pix = &f->fmt.pix;
	struct v4l2_subdev		*sd;
	const struct avicam_mbus_desc	*mbus_desc;
	int                              bayer_output;
	struct v4l2_subdev_format	 format;
	int				 ret;

	sd = avicam_remote_subdev(avicam, &format.pad);
	if(!sd)	{
		dprintk(avicam, "no subdev found\n");
		return -ENODEV;
	}

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dprintk(avicam, "bad format type %d\n", f->type);
		return -EINVAL;
	}

	/* force driver to set colorspace as specified in V4L2:
	 * Colorspace information supplements the pixelformat
	 * and must be set by the driver for capture streams
	 */
	pix->colorspace = 0;

	ret = avi_v4l2_try_fmt(f->type, pix);
	if (ret) {
		dprintk(avicam, "avi_v4l2_try_fmt failed\n");
		return ret;
	}

	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	bayer_output = (avi_v4l2_get_pixfmt_type(pix->pixelformat) ==
			AVI_PIXTYPE_BAYER);

	/* Try to get a good default format */
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &format);
	if (ret)
		return ret;

	mbus_desc = avicam_get_desc(format.format.code);
	if (mbus_desc == NULL) {
		dprintk(avicam, "invalid mbus format\n");
		return -EINVAL;
	}

	if (bayer_output && mbus_desc->type != AVI_PIXTYPE_BAYER) {
		dprintk(avicam, "can't capture raw when sensor is not raw(%d)\n",
			format.format.code);
		return -EINVAL;
	}

	if (format.format.field == V4L2_FIELD_NONE ||
	    format.format.field == V4L2_FIELD_ANY) {
		f->fmt.pix.field = V4L2_FIELD_NONE;
	} else {
		/* Input is interlaced, therefore so is the output. We only
		 * support ALTERNATE and INTERLACED so far */
		if (f->fmt.pix.field != V4L2_FIELD_ALTERNATE &&
		    f->fmt.pix.field != V4L2_FIELD_INTERLACED)
			f->fmt.pix.field = V4L2_FIELD_INTERLACED;
	}

	return 0;
}

static int avicam_try_fmt_vid_cap(struct file		*file,
                                  void			*priv,
                                  struct v4l2_format	*f)
{
	struct avicam	*avicam = video_drvdata(file);
	int		 ret;

	mutex_lock(&avicam->lock);

	ret = avicam_try_fmt_vid_cap_unlocked(avicam, f);

	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_s_fmt_vid_cap_unlocked(struct avicam		*avicam,
					 struct v4l2_format	*f)
{
	const struct v4l2_pix_format	*pix = &f->fmt.pix;
	u32				 code;
	int				 ret;

	if (avicam->streaming)
		return -EBUSY;

	ret = avicam_try_fmt_vid_cap_unlocked(avicam, f);
	if (ret)
		return ret;

	ret = avicam_pixfmt_to_mbus(avicam, pix->pixelformat, &code);
	if (ret) {
		dprintk(avicam, "unsupported pixfmt\n");
		return ret;
	}

	avicam->pix	       = *pix;
	/* Reset compose selection to take the whole output buffer */
	avicam->compose.top    = 0;
	avicam->compose.left   = 0;
	avicam->compose.width  = pix->width;
	avicam->compose.height = pix->height;

	return 0;
}

static int avicam_s_fmt_vid_cap(struct file		*file,
                                void			*priv,
                                struct v4l2_format	*f)
{
	struct avicam	*avicam	= video_drvdata(file);
	int		 ret;

	mutex_lock(&avicam->lock);

	ret = avicam_s_fmt_vid_cap_unlocked(avicam, f);

	mutex_unlock(&avicam->lock);

	return ret;
}

/* Return the cropping boundary for the source video. */
static int avicam_get_bounds(struct avicam    *avicam,
			     struct v4l2_rect *r)
{
	struct v4l2_subdev		*sd;
	struct v4l2_subdev_format	 format;
	int				 ret;

	sd = avicam_remote_subdev(avicam, &format.pad);
	if(!sd)
		return -ENODEV;

	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	/* Try to get a good default format */
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &format);
	if (ret)
		/* Looks like there's nothing we can do */
		return -ENOIOCTLCMD;

	r->top    = 0;
	r->left	  = 0;
	r->width  = format.format.width;
	r->height = format.format.height;

	return 0;
}

/* Returt the cropping boundary for the destination buffer. For now it's simply
 * the size of the buffer */
static int avicam_get_compose_bounds(struct avicam    *avicam,
				     struct v4l2_rect *r)
{
	r->top    = 0;
	r->left   = 0;
	r->width  = avicam->pix.width;
	r->height = avicam->pix.height;

	return 0;
}

static int avicam_g_selection(struct file           *file,
			      void                  *priv,
			      struct v4l2_selection *s)
{
	struct avicam *avicam = video_drvdata(file);
	int            ret;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return -EINVAL;
	}

	mutex_lock(&avicam->lock);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		ret = avicam_get_bounds(avicam, &s->r);
		break;
	case V4L2_SEL_TGT_CROP_ACTIVE:
		s->r = avicam->rect;
		ret = 0;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		ret = avicam_get_compose_bounds(avicam, &s->r);
		break;
	case V4L2_SEL_TGT_COMPOSE_ACTIVE:
		s->r = avicam->compose;
		ret = 0;
		break;
	default:
		ret = -ENOSYS;
	}

	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_s_selection(struct file           *file,
			      void                  *priv,
			      struct v4l2_selection *s)
{
	struct avicam    *avicam = video_drvdata(file);
	struct v4l2_rect  bounds;
	int               ret;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return -EINVAL;
	}

	mutex_lock(&avicam->lock);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_ACTIVE:
		avicam_get_bounds(avicam, &bounds);

		avi_v4l2_crop_adjust(&s->r, &bounds);

		if (s->r.width == 0 || s->r.height == 0) {
			ret = -EINVAL;
			break;
		}

		/* We don't yet allow changing the source resolution while
		 * streaming because we can't add/remove the scaler on the
		 * fly. */
		if (avicam->streaming &&
		    (s->r.width  != avicam->rect.width ||
		     s->r.height != avicam->rect.height)) {
			ret = -EBUSY;
			break;
		}

		avicam->rect = s->r;

		ret = 0;
		break;
	case V4L2_SEL_TGT_COMPOSE_ACTIVE:
		avicam_get_compose_bounds(avicam, &bounds);

		avi_v4l2_crop_adjust(&s->r, &bounds);

		if (s->r.width == 0 || s->r.height == 0) {
			ret = -EINVAL;
			break;
		}

		/* We don't yet allow changing the source resolution while
		 * streaming because we can't add/remove the scaler on the
		 * fly. */
		if (avicam->streaming &&
		    (s->r.width  != avicam->compose.width ||
		     s->r.height != avicam->compose.height)) {
			ret = -EBUSY;
			break;
		}

		avicam->compose = s->r;
		ret = 0;
		break;
	default:
		ret = -ENOSYS;
	}

	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_enum_input(struct file	*file,
                             void		*priv,
                             struct v4l2_input	*inp)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	if (inp->index != 0) {
		ret = -EINVAL;
		goto unlock;
	}

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		dev_err(avicam->dev, "no subdev found\n");
		ret = -ENODEV;
		goto unlock;
	}

	/* Ugly hack for dv */
	ret = v4l2_subdev_call(sd, video, s_dv_timings, NULL);
	if(ret == -ENODEV || ret == -ENOIOCTLCMD)
		inp->capabilities &= ~V4L2_IN_CAP_CUSTOM_TIMINGS;

	ret = v4l2_subdev_call(sd, core, s_std, 0);
	if(ret == -ENODEV || ret == -ENOIOCTLCMD)
		inp->capabilities &= ~V4L2_IN_CAP_STD;

	/* default is camera */
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->std  = V4L2_STD_UNKNOWN;
	strlcpy(inp->name, "Camera", sizeof(inp->name));

	ret = 0;

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int avicam_s_input(struct file *file, void *priv, unsigned int i)
{
	struct avicam	*avicam = video_drvdata(file);

	if (avicam->streaming)
		return -EBUSY;

	if (i > 0)
		return -EINVAL;

	return 0;
}

static int avicam_s_std(struct file *file, void *priv, v4l2_std_id *a)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	if (avicam->streaming) {
		ret = -EBUSY;
		goto unlock;
	}

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, core, s_std, *a);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_g_std(struct file *file, void *priv, v4l2_std_id *a)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, core, g_std, a);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_g_chip_ident(struct file			*file,
                               void				*fh,
                               struct v4l2_dbg_chip_ident	*id)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, core, g_chip_ident, id);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static void avicam_done(struct avi_capture_context	*ctx,
                        struct avi_dma_buffer		*frame,
                        enum avi_field			 f)
{
	struct avicam		*avicam = ctx->priv;
	struct avicam_vbuf	*vbuf	= frame->priv;
	enum vb2_buffer_state    state  = VB2_BUF_STATE_DONE;

	if (vbuf->last_capture == 0) {
		/* We wait until we get the bottom field in the
		 * buffer */
		vbuf->last_capture = 1;
		return;
	}

	/* Frame count is actually field count because videobuf is retarded */
	if (avicam->pix.field == V4L2_FIELD_ALTERNATE)
		avicam->frame_count++;
	else
		avicam->frame_count += 2;

	v4l2_get_timestamp(&vbuf->vb.v4l2_buf.timestamp);
	vbuf->vb.v4l2_buf.sequence = avicam->frame_count / 2;

	if (frame->status == AVI_BUFFER_ERROR ||
	    frame->status == AVI_BUFFER_TIMEOUT)
		state = VB2_BUF_STATE_ERROR;

	if (avicam->pix.field == V4L2_FIELD_ALTERNATE)
		vbuf->vb.v4l2_buf.field = (f == AVI_TOP_FIELD) ?
			V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM;

	vb2_buffer_done(&vbuf->vb, state);
}

static void avicam_next(struct avi_capture_context	*ctx,
			struct avi_dma_buffer		*frame,
			enum avi_field			 f)
{
	struct avicam		*avicam	       = ctx->priv;
	struct v4l2_subdev	*sd;
	unsigned		 bottom_offset = 0;
	struct avicam_vbuf	*vbuf	       = NULL;
	dma_addr_t		 dma_addr;
	unsigned long		 flags;
	bool			 handled       = false;
	struct v4l2_event event;

#ifdef CONFIG_AVICAM_USE_ISP
	/* Do blanking ISP task */
	avi_v4l2_isp_blanking(avicam->ctrl_isp);
#endif

	if (avicam->pix.field == V4L2_FIELD_INTERLACED) {
		vbuf = avicam->bottom_vbuf;

		if (vbuf) {
			/* We should be capturing the bottom field */
			bottom_offset = avicam->pix.bytesperline;
			avicam->bottom_vbuf = NULL;
		} else if (f == AVI_BOT_FIELD) {
			/* We must wait until we get a top field to start
			 * capturing a new buffer */
			return;
		}
	}

	/* Propagate interrupt to subdev */
	sd = avicam_remote_subdev(avicam, NULL);
	v4l2_subdev_call(sd, core, interrupt_service_routine, 0, &handled);

	/* Queue V4L2 frame_sync event */
	memset(&event, 0, sizeof(event));
	event.type = V4L2_EVENT_FRAME_SYNC;
	event.u.frame_sync.frame_sequence = avicam->frame_count / 2;
	v4l2_event_queue(avicam->vdev, &event);

	if (vbuf == NULL) {
		spin_lock_irqsave(&avicam->vbq_lock, flags);

		if (!list_empty(&avicam->bufqueue)) {
			vbuf = list_entry(avicam->bufqueue.next,
			                        struct avicam_vbuf,
			                        list);

			list_del(&vbuf->list);

			vbuf->vb.state = VB2_BUF_STATE_ACTIVE;
		}

		spin_unlock_irqrestore(&avicam->vbq_lock, flags);

		if (vbuf == NULL) {
			dprintk_l(avicam, "Exhausted buffer queue\n");
			return;
		}

		/* We'll have to capture two fields in a row if the input is
		 * interlaced */
		if (avicam->pix.field == V4L2_FIELD_INTERLACED) {
			avicam->bottom_vbuf = vbuf;
			vbuf->last_capture = 0;
		} else {
			vbuf->last_capture = 1;
		}
	}

	dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0) + bottom_offset;

	avi_v4l2_setup_dmabuf(&avicam->compose,
			      &avicam->segment_format,
			      avicam->plane0_size,
			      avicam->plane1_size,
			      dma_addr,
			      vbuf,
			      frame);
}

static int avicam_clean_buffers(struct avicam *avicam, int state) {
	unsigned long       flags;
	struct avicam_vbuf *buf, *node;

	spin_lock_irqsave(&avicam->vbq_lock, flags);

	list_for_each_entry_safe(buf, node, &avicam->bufqueue, list) {
		vb2_buffer_done(&buf->vb, state);
		list_del(&buf->list);
	}

	spin_unlock_irqrestore(&avicam->vbq_lock, flags);

	return 0;
}

static unsigned long avicam_get_timeout(struct v4l2_subdev *sd) {
	struct v4l2_subdev_frame_interval fi;
	int ret;
	u32 min_timeout = HZ / 2; /* Minimum 500ms timeout */
	u32 timeout;

	ret = v4l2_subdev_call(sd, video, g_frame_interval, &fi);

	if (ret)
		return 3 * HZ; /* Default 3s timeout */

	/* Set timeout to the duration of 3 frames */
	timeout = div_u64((u64)fi.interval.numerator * 3 * HZ,
	                  fi.interval.denominator);

	if (timeout < min_timeout)
		timeout = min_timeout;

	return timeout;
}

static int avicam_streamon(struct vb2_queue* vq, unsigned int count)
{
	struct avicam			*avicam	 = vb2_get_drv_priv(vq);
	const struct v4l2_pix_format	*pix     = &avicam->pix;
	const struct v4l2_rect          *compose = &avicam->compose;
	const struct v4l2_rect		*rect    = &avicam->rect;
	struct avi_capture_context	*ctx	 = &avicam->capture_ctx;
	struct avi_segment_format	*fmt     = &avicam->segment_format;
	int				 collect_stats;
	struct v4l2_subdev		*sd;
	const struct avicam_mbus_desc	*mbus_desc;
	unsigned long			 caps;
	int				 bayer_input;
	int				 bayer_output;
	struct v4l2_subdev_format	 format;
	struct avi_capture_crop		 crop;
	int				 ret;
	unsigned                         sensor_width;
	unsigned                         sensor_height;
#ifdef CONFIG_AVICAM_USE_ISP
	struct avi_isp_offsets		 isp_offsets;
#endif

	if (avicam->streaming) {
		ret = -EBUSY;
		goto busy;
	}

	avicam->frame_count = 0;

	sd = avicam_remote_subdev(avicam, &format.pad);
	if (!sd) {
		ret = -ENODEV;
		goto no_subdev;
	}

	media_entity_pipeline_start(&avicam->vdev->entity, &avicam->pipe);

	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	/* Try to get a good default format */
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &format);
	if (ret)
		goto get_fmt_failed;

	mbus_desc = avicam_get_desc(format.format.code);
	if (mbus_desc == NULL)
		goto bad_mbus;

	crop.x	    = rect->left;
	crop.y	    = rect->top;

	if (rect->width && rect->height) {
		crop.width  = rect->width;
		crop.height = rect->height;
	} else {
		/* Use the whole sensor's resolution when no crop window is
		 * set */
		crop.width  = format.format.width  - crop.x;
		crop.height = format.format.height - crop.y;
	}

	bayer_input  = (mbus_desc->type == AVI_PIXTYPE_BAYER);
	bayer_output = (avi_v4l2_get_pixfmt_type(pix->pixelformat) ==
			AVI_PIXTYPE_BAYER);

	caps = avicam->pdata->cam_cap;

	if (crop.width  != compose->width ||
	    crop.height != compose->height)
		caps |= AVI_CAP_SCAL;

	if (bayer_output) {
		if (caps & AVI_CAP_SCAL) {
			dev_err(avicam->dev,
				"bad format: can't rescale a raw image "
				"[%ux%u -> %ux%u]\n",
				crop.width, crop.height,
				compose->width, compose->height);
			ret = -EINVAL;
			goto bad_bayer;
		}

		if (!bayer_input) {
			/* Can't output raw from non-raw input */
			ret = -EINVAL;
			goto bad_bayer;
		}
	} else {
		/* Output is not raw. */
		if (bayer_input)
			/* We need to convert the RAW image  */
			caps |= AVI_CAPS_ISP;
		else if (format.format.colorspace != pix->colorspace)
			/* We need to do chroma conversion */
			caps |= AVI_CAP_CONV;
	}

	/* We collect the stats when we have an ISP */
	collect_stats = avicam->pdata->enable_stats &&
		(caps & AVI_CAP_ISP_CHAIN_BAYER);

	ret = avi_capture_init(ctx,
			       avicam->dev,
			       avicam_instance(avicam),
			       caps,
			       collect_stats);
	if (ret)
		goto capture_init_failed;

#ifdef CONFIG_AVICAM_USE_ISP
	/* Activate ISP chain controls */
	if ((caps & AVI_CAPS_ISP) &&
	    avi_isp_get_offsets(ctx->dma_segment, &isp_offsets) == 0) {
		avi_v4l2_isp_activate(avicam->ctrl_isp, &isp_offsets);
		avi_v4l2_isp_blanking(avicam->ctrl_isp);
	}
#endif

	ctx->priv = avicam;

	ctx->measure_to_timings       = avicam->pdata->measure_to_timings;
	ctx->interface		      = avicam->pdata->interface;
	ctx->interface.format_control = mbus_desc->format_control;

	if (mbus_desc->type == AVI_PIXTYPE_BAYER && mbus_desc->bpp == 10)
		ctx->interface.raw10 = AVI_CAP_RAW10_1X10;
	else if (mbus_desc->type == AVI_PIXTYPE_GREY && mbus_desc->bpp == 8)
		ctx->interface.unpacker = AVI_CAP_RAW8_1X8;
	else if (mbus_desc->type == AVI_PIXTYPE_GREY && mbus_desc->bpp == 10)
		ctx->interface.raw10 = AVI_CAP_RAW10_1X10;

	ctx->interlaced	= (pix->field != V4L2_FIELD_NONE);

	/* There is no V4L2 colorspace for 'GREY' so we specify it manually */
	if (mbus_desc->type == AVI_PIXTYPE_GREY) {
		if (mbus_desc->bpp == 8 ||
		    pix->pixelformat == V4L2_PIX_FMT_GREY) {
			ctx->capture_cspace = AVI_GREY_CSPACE;
		} else {
			ctx->capture_cspace = AVI_Y10_CSPACE;
		}
	} else {
		ctx->capture_cspace =
			avi_v4l2_to_avi_cspace(format.format.colorspace);
	}

	if (mbus_desc->bpp == 10 && ctx->capture_cspace == AVI_GREY_CSPACE) {
		/* We have a 10 bit interface but we only want to capture 8
		 * bits. We can do this by using ror-lsb to shift the 8MSB to
		 * the first 8bits and then set the unpacker to move those 8bits
		 * to the Y component in the VL */
		ctx->interface.ror_lsb = 1;
		ctx->interface.raw10 = 0;
		ctx->interface.unpacker = AVI_CAP_RAW8_1X8;
	}

	ctx->next	= &avicam_next;
	ctx->done	= &avicam_done;
	ctx->stats_next = &avicam_stats_next;
	ctx->stats_done = &avicam_stats_done;

	/* Make sure we don't have any stale vbuf laying around */
	avicam->bottom_vbuf = NULL;

	ctx->timeout_jiffies = avicam_get_timeout(sd);

	/* Start the subdevice */
	ret = v4l2_subdev_call(sd, video, s_stream, 1);
	if (ret)
		goto subdev_streamon_failed;
#ifdef CONFIG_AVICAM_TIMINGS_AUTO
	ret = avi_capture_prepare(ctx);
	if (ret) {
		dev_err(avicam->dev, "failed to get sensor timings\n");
		goto subdev_streamon_failed;
	}
	avi_capture_resolution(ctx, &sensor_width, &sensor_height);
#else
	ret = avi_capture_prepare_timings(ctx, format.format.width, format.format.height);
	if (ret)
		goto prepare_failed;
	sensor_width = format.format.width;
	sensor_height = format.format.height;
#endif
	dprintk(avicam,
	        "sensor resolution is %dx%d%c\n",
	        sensor_width, sensor_height,
	        ctx->interlaced ? 'i' : 'p');

	ret = avi_capture_set_crop(ctx, &crop);
	if (ret) {
		dev_err(avicam->dev,
			"cropping failed. Resolution: %dx%d. "
			"Crop window: %dx%d @%dx%d\n",
			sensor_width, sensor_height,
			crop.width,   crop.height,
			crop.x,       crop.y);
		goto crop_failed;
	}

	if ((caps & AVI_CAPS_ISP) &&
	    (crop.width > 2048 || crop.height > 4096)) {
		dev_err(avicam->dev,
			"AVI ISP doesn't handle resolutions above 2048x4096\n");
		ret = -ERANGE;
		goto bad_resolution;
	}

	/* XXX Handle output crop ? */
	ret = avi_v4l2_to_segment_fmt(pix,
				      compose,
				      fmt,
				      &avicam->plane0_size,
				      &avicam->plane1_size);
	if (ret)
		goto format_conversion_failed;

	ret = avi_capture_set_format(ctx, fmt);
	if (ret) {
		dev_err(avicam->dev, "failed to set capture format\n");
		goto set_format_failed;
	}

	avicam->streaming = 1;

	ret = avi_capture_streamon(ctx);
	if (ret)
		goto capture_streamon_failed;

	/* Success */
	return 0;

 capture_streamon_failed:
 set_format_failed:
 format_conversion_failed:
 crop_failed:
 bad_resolution:
 prepare_failed:
 subdev_streamon_failed:
	v4l2_subdev_call(sd, video, s_stream, 0);
	avi_capture_destroy(ctx);
 capture_init_failed:
 bad_bayer:
 bad_mbus:
 get_fmt_failed:
 no_subdev:
	avicam->streaming = 0;
	avicam_clean_buffers(avicam, VB2_BUF_STATE_QUEUED);
 busy:
	media_entity_pipeline_stop(&avicam->vdev->entity);
	return ret;
}

static int avicam_do_streamoff(struct avicam *avicam)
{
	struct v4l2_subdev	*sd;

#ifdef CONFIG_AVICAM_USE_ISP
	/* Deactivate ISP chains controls */
	avi_v4l2_isp_deactivate(avicam->ctrl_isp);
#endif

	if (!avicam->streaming)
		return -EINVAL;

	avicam->streaming = 0;

	sd = avicam_remote_subdev(avicam, NULL);
	if (!sd)
		return -ENODEV;

	avi_capture_streamoff(&avicam->capture_ctx);

	avi_capture_destroy(&avicam->capture_ctx);

	v4l2_subdev_call(sd, video, s_stream, 0);

	avicam_clean_buffers(avicam, VB2_BUF_STATE_ERROR);

	media_entity_pipeline_stop(&avicam->vdev->entity);

	return 0;
}

static int avicam_streamoff(struct vb2_queue* vq)
{
	struct avicam	*avicam = vb2_get_drv_priv(vq);
	struct v4l2_subdev	*sd;

	sd = avicam_remote_subdev(avicam, NULL);
	v4l2_subdev_call(sd, core, s_power, 0);

	return avicam_do_streamoff(avicam);
}

static struct vb2_ops avicam_vqueue_ops = {
	.queue_setup	= avicam_queue_setup,
	.buf_init	= avicam_vbuf_init,
	.buf_prepare	= avicam_vbuf_prepare,
	.buf_queue	= avicam_vbuf_queue,
	.buf_cleanup	= avicam_vbuf_release,
	.start_streaming = avicam_streamon,
	.stop_streaming = avicam_streamoff,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int avicam_s_dv_timings(struct file *file,
                               void *fh,
                               struct v4l2_dv_timings *timings)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, video, s_dv_timings, timings);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_g_dv_timings(struct file		*file,
                               void			*fh,
                               struct v4l2_dv_timings	*timings)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, video, g_dv_timings, timings);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_query_dv_timings(struct file			*file,
                                   void				*fh,
                                   struct v4l2_dv_timings	*timings)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, video, query_dv_timings, timings);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_enum_dv_timings(struct file			*file,
                                  void				*fh,
                                  struct v4l2_enum_dv_timings	*timings)
{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, video, enum_dv_timings, timings);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_dv_timings_cap(struct file *file,
                                 void *fh,
                                 struct v4l2_dv_timings_cap *cap)

{
	struct avicam		*avicam = video_drvdata(file);
	struct v4l2_subdev	*sd;
	int			 ret;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, NULL);
	if(!sd) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = v4l2_subdev_call(sd, video, dv_timings_cap, cap);

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static long avicam_default(struct file	*file,
                           void		*p,
                           bool		 valid_prio,
                           int		 cmd,
                           void		*arg)
{
	struct avicam		*avicam = video_drvdata(file);
	int			 ret = -ENOIOCTLCMD;

	mutex_lock(&avicam->lock);

	if (cmd == AVI_ISP_IOGET_OFFSETS) {

		if (!avicam->streaming) {
			ret = -EINVAL;
			goto unlock;
		}

		BUG_ON(!avicam->capture_ctx.dma_segment);

		/* The v4l2 layer allocates "arg" in kernel space based on
		 * _IOC_SIZE instead of directly passing the user pointer. It
		 * means we don't have to copy_to_user or check for overflows
		 * here. See the code of video_usercopy in v4l2-ioctl.c. */
		ret = avi_isp_get_offsets(avicam->capture_ctx.dma_segment,
					  (struct avi_isp_offsets *)arg);
	}

 unlock:
	mutex_unlock(&avicam->lock);

	return ret;
}

static int avicam_subscribe_event(struct v4l2_fh *fh,
                                  struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_FRAME_SYNC)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub, 0, NULL);
}

static int avicam_unsubscribe_event(struct v4l2_fh *fh,
                                     struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static const struct v4l2_ioctl_ops avicam_ioctl_ops = {
	.vidioc_querycap	 = avicam_querycap,
	.vidioc_enum_fmt_vid_cap = avicam_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	 = avicam_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	 = avicam_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	 = avicam_s_fmt_vid_cap,

	.vidioc_g_selection      = avicam_g_selection,
	.vidioc_s_selection      = avicam_s_selection,

	.vidioc_enum_input	 = avicam_enum_input,
	.vidioc_g_input		 = avicam_g_input,
	.vidioc_s_input		 = avicam_s_input,
	.vidioc_s_std		 = avicam_s_std,
	.vidioc_g_std		 = avicam_g_std,
	.vidioc_reqbufs		 = vb2_ioctl_reqbufs,
	.vidioc_querybuf	 = vb2_ioctl_querybuf,
	.vidioc_qbuf		 = vb2_ioctl_qbuf,
	.vidioc_dqbuf		 = vb2_ioctl_dqbuf,
	.vidioc_expbuf		 = vb2_ioctl_expbuf,
	.vidioc_g_chip_ident	 = avicam_g_chip_ident,
	.vidioc_streamon	 = vb2_ioctl_streamon,
	.vidioc_streamoff	 = vb2_ioctl_streamoff,
	.vidioc_default		 = avicam_default,

	.vidioc_subscribe_event   = avicam_subscribe_event,
	.vidioc_unsubscribe_event = avicam_unsubscribe_event,

	/* These functions just call the subdev methods of the same name. They
	 * are only used by the adv7611 userland tool for debugging
	 * purposes. Once they are not needed anymore we should remove them from
	 * here (or re-implement them if they make sense in the context of this
	 * driver)
	 */
	.vidioc_dv_timings_cap		= avicam_dv_timings_cap,
	.vidioc_s_dv_timings		= avicam_s_dv_timings,
	.vidioc_g_dv_timings		= avicam_g_dv_timings,
	.vidioc_query_dv_timings	= avicam_query_dv_timings,
	.vidioc_enum_dv_timings		= avicam_enum_dv_timings,
};

static int avicam_open(struct file *file)
{
	struct avicam			*avicam = video_drvdata(file);
	struct v4l2_format		 default_fmt = { 0 };
	struct v4l2_subdev		*sd;
	struct v4l2_subdev_format	 format;
	const struct avicam_mbus_desc 	*desc;
	int				 ret;
	struct vb2_queue*                q = &avicam->vb_vidq;
	struct avicam_fh                *fh;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh) {
		ret = -ENOMEM;
		goto no_mem;
	}

	v4l2_fh_init(&fh->vfh, avicam->vdev);
	v4l2_fh_add(&fh->vfh);
	file->private_data = &fh->vfh;

	mutex_lock(&avicam->lock);

	sd = avicam_remote_subdev(avicam, &format.pad);
	if (!sd) {
		dev_err(avicam->dev, "no subdev found\n");
		ret = -ENODEV;
		goto no_subdev;
	}

	if (avicam->use_count > 0) {
		ret = 0;
		goto unlock;
	}

	memset(&avicam->pix,  0, sizeof(avicam->pix));
	memset(&avicam->rect, 0, sizeof(avicam->rect));

	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	/* Try to get a good default format */
	ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &format);
	if (ret) {
		dev_err(avicam->dev, "can't get sensor bus format\n");
		goto unlock;
	}

	default_fmt.type	       = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	default_fmt.fmt.pix.width      = format.format.width;
	default_fmt.fmt.pix.height     = format.format.height;
	default_fmt.fmt.pix.colorspace = format.format.colorspace;
	default_fmt.fmt.pix.field      = format.format.field;

	/* Select a sane default pixelformat */
	default_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	desc = avicam_get_desc(format.format.code);
	if (desc)
		switch (desc->type) {
		case AVI_PIXTYPE_BAYER:
			default_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR10;
			break;
		case AVI_PIXTYPE_RGB:
			default_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;
			break;
		case AVI_PIXTYPE_YUV:
		default:
			default_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
			break;
		}

	ret = avicam_s_fmt_vid_cap_unlocked(avicam, &default_fmt);
	if (ret) {
		dev_err(avicam->dev, "couldn't set the default format\n");
		goto unlock;
	}

	v4l2_subdev_call(sd, core, s_power, 1);

	memset(q, 0, sizeof(struct vb2_queue));

	q->type              = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes          = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->mem_ops           = &vb2_dma_contig_memops;
	q->lock              = &avicam->lock;
	q->ops               = &avicam_vqueue_ops;
	q->drv_priv          = avicam;
	q->buf_struct_size   = sizeof(struct avicam_vbuf);
	q->cache_flags       = avicam->pdata->vb2_cache_flags;
	vb2_queue_init(q);

	ret = 0;

	goto unlock;

 no_subdev:
	file->private_data = NULL;
	v4l2_fh_del(&fh->vfh);
	v4l2_fh_exit(&fh->vfh);
	kfree(fh);

 unlock:
	if (ret == 0)
		avicam->use_count++;

	mutex_unlock(&avicam->lock);

	dprintk(avicam, "camera device opened\n");
 no_mem:
	return ret;
}

static int avicam_release(struct file *file)
{
	struct avicam *avicam = video_drvdata(file);
	int            ret = 0;

	mutex_lock(&avicam->lock);

	avicam->use_count--;
	if (!avicam->use_count) {
		/*
		 * Release all resources (cleaning/freeing)
		 * v4l2_fh_release called in vb2_fop_release
		 */
		ret = vb2_fop_release(file);
	}
	else {
		ret = v4l2_fh_release(file);
	}

	/*
	 * file->private_data can not be reseted before :
	 * it is used in vb2_fop_release
	 */
	file->private_data = NULL;

	mutex_unlock(&avicam->lock);
	return ret;
}

static struct v4l2_file_operations avicam_fops = {
	.owner		= THIS_MODULE,
	.open		= avicam_open,
	.release	= avicam_release,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.read           = vb2_fop_read,
};

static int __devinit avicam_register_subdevs(struct avicam *avicam,
                                             struct avicam_subdevs *devs,
					     struct media_entity *entity,
					     u32 sink_pad)
{
	struct v4l2_subdev	*subdev = NULL;
	struct i2c_adapter	*adapter;
	struct avicam_subdevs	*dev;
	int ret = 0;

	if (devs == NULL || devs->board_info == NULL) {
		dev_err(avicam->dev, "no board info found\n");
		return -ENODEV;
	}

	for (dev = devs; dev->board_info; dev++) {
		unsigned int flags = MEDIA_LNK_FL_ENABLED;
		unsigned int pad = 0;
		int i;

		dprintk(avicam, "found one i2c board info\n");

		adapter = i2c_get_adapter(dev->i2c_adapter_id);
		if (adapter == NULL) {
			dev_err(avicam->dev,
			        "Unable to get I2C adapter %d for device %s\n",
				dev->i2c_adapter_id,
				dev->board_info->type);
			continue;
		}

		subdev = v4l2_i2c_new_subdev_board(avicam->v4l2_dev,
		                                   adapter,
		                                   dev->board_info,
		                                   NULL);
		if (subdev == NULL) {
			dprintk(avicam,
			        "Unable to register subdev %s\n",
			        dev->board_info->type);
			continue;
		}

		/* This just returns 0 if either of the two args is NULL */
		v4l2_ctrl_add_handler(avicam->vdev->ctrl_handler,
				      subdev->ctrl_handler);

		if(dev->subdevs) {
			pad = 0;

			for (i = 0; i < subdev->entity.num_pads; i++)
				if(subdev->entity.pads[i].flags & MEDIA_PAD_FL_SINK) {
					pad = i;
					break;
				}

			if (i == subdev->entity.num_pads) {
				dprintk(avicam, "No sink pad found on subdev\n");
				break;
			}

			ret = avicam_register_subdevs(avicam, dev->subdevs,
						      &subdev->entity, pad);
			if(ret < 0) {
				dprintk(avicam, "Error registering subdevices\n");
				/* XXX We should cleanup whe previous registered
				   subdevices before returning here */
				return ret;
			}
		}

		pad = 0;
		for (i = 0; i < subdev->entity.num_pads; i++)
			if (subdev->entity.pads[i].flags & MEDIA_PAD_FL_SOURCE) {
				pad = i;
				break;
			}

		if (i == subdev->entity.num_pads) {
			dprintk(avicam, "No source pad found on subdev\n");
			break;
		}

		ret = media_entity_create_link(&subdev->entity, pad, entity, sink_pad,
					       flags);
		if (ret < 0) {
			dprintk(avicam, "Failed to create media link(sd)\n");
			break;;
		}
	}

	if (!subdev)
		/* Couldn't initialise any device succesfully.  XXX We should
		   cleanup whe previous registered subdevices before returning
		   here */
		return -EINVAL;

	return ret;
}


static int __devinit avicam_init_subdevs(struct avicam *avicam)
{
	struct device			*dev = avicam->dev;
	int				 ret;

	/* register i2c devices
	 *
	 * if board_info isn't present, register dummy driver currently, only
	 * one device per i2c bus is supported but this is intended to change to
	 * be able to support several the media devices should be configured
	 * here */
	if(!avicam->pdata->subdevs && !avicam->pdata->dummy_driver_info) {
		dev_err(dev, "no dev config found\n");
		return -EINVAL;
	}

	if (avicam->pdata->subdevs) {
		ret = avicam_register_subdevs(avicam, avicam->pdata->subdevs,
					     &avicam->vdev->entity, 0);
		if (ret)
			dprintk(avicam, "Error registering subdevs\n");

	}

	if (avicam->pdata->dummy_driver_info) {
		unsigned int flags = MEDIA_LNK_FL_ENABLED;
		/* only one pad on avicam right now */
		unsigned int pad   = 0;
		struct v4l2_subdev *subdev;
		ret = request_module(AVICAM_DUMMY_NAME);

		ret = avicam_dummy_add(avicam->v4l2_dev,
		                       &avicam->dummy_pdev,
		                       avicam->pdata->dummy_driver_info);
		if (ret) {
			dev_err(dev, "Unable to register dummy dev\n");
			return ret;
		}

		if(!avicam->dummy_pdev->dev.driver ||
		   !try_module_get(avicam->dummy_pdev->dev.driver->owner)) {
			dev_err(dev, "cannot get dummy module\n");
			return ret;
		}
		subdev = platform_get_drvdata(avicam->dummy_pdev);
		if(!subdev) {
			dprintk(avicam, "Cannot get a dummy subdev\n");
			return ret;
		}
		ret = media_entity_create_link(&subdev->entity, 0, &avicam->vdev->entity, pad,
					       flags);
		if(ret < 0) {
			dprintk(avicam, "Failed to create media link(dummy)\n");
			return ret;
		}
	}

	return 0;
}

static void avicam_destroy_subdevs(struct avicam *avicam)
{
	struct v4l2_subdev *sd, *next;

	if (avicam->dummy_pdev) {
		if(avicam->dummy_pdev->dev.driver)
			module_put(avicam->dummy_pdev->dev.driver->owner);
		avicam_dummy_del(avicam->dummy_pdev);
	}

	list_for_each_entry_safe(sd, next, &avicam->v4l2_dev->subdevs, list) {
		if (sd->flags & V4L2_SUBDEV_FL_IS_I2C) {
			struct i2c_client *client = v4l2_get_subdevdata(sd);

			/* We need to unregister the i2c client explicitly.
			   We cannot rely on i2c_del_adapter to always
			   unregister clients for us, since if the i2c bus
			   is a platform bus, then it is never deleted. */
			if (client)
				i2c_unregister_device(client);
		}
	}
}

static int __devinit avicam_v4l2_init(struct avicam *avicam)
{
	struct video_device	*vdev;
	int			 ret;

	avicam->v4l2_dev = avi_v4l2_get_device();
	if (!avicam->v4l2_dev) {
		ret = -ENODEV;
		goto no_v4l2_dev;
	}

	vdev = video_device_alloc();
	if (vdev == NULL) {
		ret = -ENODEV;
		goto vdev_alloc_failed;
	}

	avicam->vdev = vdev;

	avicam->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&avicam->vdev->entity, 1, &avicam->pad, 0);
	if(ret < 0) {
		dprintk(avicam, "Error initializing media entity\n");
		goto media_init_failed;
	}

	mutex_init(&avicam->lock);

	strlcpy(vdev->name, dev_name(avicam->dev), sizeof(vdev->name));

	vdev->parent	   = avicam->dev;
	vdev->current_norm = V4L2_STD_UNKNOWN;
	vdev->fops	   = &avicam_fops;
	vdev->ioctl_ops	   = &avicam_ioctl_ops;
	vdev->release	   = &video_device_release;
	vdev->vfl_type	   = VFL_TYPE_GRABBER;
	vdev->tvnorms	   = V4L2_STD_UNKNOWN;
	vdev->queue        = &avicam->vb_vidq;
	/* We handle the locking ourselves */
	vdev->lock	   = NULL;

	video_set_drvdata(vdev, avicam);

#ifndef AVI_V4L2_ISP_CONTROL_HINT
#define AVI_V4L2_ISP_CONTROL_HINT 0
#endif
	/* Register ctrl_handler for inheriting controls from sub-devices */
	ret = v4l2_ctrl_handler_init(&avicam->ctrl_handler,
				     16 + AVI_V4L2_ISP_CONTROL_HINT);
	if (ret < 0)
		goto init_ctrl_handler_failed;

#ifdef CONFIG_AVICAM_USE_ISP
	/* Add ISP chain controls to V4L2 device */
	ret = avi_v4l2_isp_init(&avicam->ctrl_isp, &avicam->ctrl_handler);
	if (ret)
		goto init_isp_failed;
#endif

	avicam->vdev->ctrl_handler = &avicam->ctrl_handler;

	vdev->v4l2_dev = avicam->v4l2_dev;
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto video_register_failed;

	ret = avicam_init_subdevs(avicam);
	if (ret)
		goto subdev_register_failed;

	ret = avicam_stats_init(avicam);
	if (ret)
		goto init_stats_failed;

	return 0;

 init_stats_failed:
	avicam_destroy_subdevs(avicam);
 subdev_register_failed:
 video_register_failed:
#ifdef CONFIG_AVICAM_USE_ISP
	avi_v4l2_isp_free(avicam->ctrl_isp);
 init_isp_failed:
#endif
	v4l2_ctrl_handler_free(&avicam->ctrl_handler);
 init_ctrl_handler_failed:
	media_entity_cleanup(&avicam->vdev->entity);
 media_init_failed:
	/* unregister calls video_device_release */
	video_unregister_device(avicam->vdev);
 vdev_alloc_failed:
	avi_v4l2_put_device(avicam->v4l2_dev);
 no_v4l2_dev:
	return ret;
}

static int avicam_register_v4l2_subdevs(void)
{
	struct v4l2_device	*v4l2_dev;
	int			 ret;

	/* Make sure the AVI has been succesfully probed */
	if (!avi_probed())
		return -ENODEV;

	pr_info("registering avicam V4L2 subdevice nodes.\n");

	v4l2_dev = avi_v4l2_get_device();

	if (!v4l2_dev) {
		pr_err("no v4l2 device available\n");
		return -ENODEV;
	}

	ret = v4l2_device_register_subdev_nodes(v4l2_dev);
	if (ret)
		pr_err("v4l2_device_register_subdev_nodes failed [%d]\n", ret);

	avi_v4l2_put_device(v4l2_dev);
	return ret;
}

static void __devexit avicam_v4l2_destroy(struct avicam *avicam)
{
	avicam_stats_destroy(avicam);
	avicam_destroy_subdevs(avicam);
#ifdef CONFIG_AVICAM_USE_ISP
	avi_v4l2_isp_free(avicam->ctrl_isp);
#endif
	v4l2_ctrl_handler_free(&avicam->ctrl_handler);
	video_unregister_device(avicam->vdev);
	avi_v4l2_put_device(avicam->v4l2_dev);
}

static int __devinit avicam_probe(struct platform_device *pdev)
{
	int		    ret;
	struct avicam	   *avicam;
	struct vb2_dc_conf *alloc_ctx;

	avicam = kzalloc(sizeof(*avicam), GFP_KERNEL);
	if (avicam == NULL) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	avicam->dev = &pdev->dev;

	avicam->pdata = dev_get_platdata(&pdev->dev);
	if (!avicam->pdata) {
		ret = -EINVAL;
		goto no_pdata;
	}

	avicam->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(avicam->pctl)) {
		ret = PTR_ERR(avicam->pctl);
		goto pinctrl_failed;
	}

	spin_lock_init(&avicam->vbq_lock);
	INIT_LIST_HEAD(&avicam->bufqueue);

	ret = avicam_v4l2_init(avicam);
	if(ret)
		goto v4l2_init_failed;

	platform_set_drvdata(pdev, avicam);
	dev_set_drvdata(&pdev->dev, avicam);

	dev_info(avicam->dev,
	         "video device successfuly registered as %s\n",
	         video_device_node_name(avicam->vdev));

	alloc_ctx = (struct vb2_dc_conf *)vb2_dma_contig_init_ctx(&pdev->dev);
	alloc_ctx->cache_flags = avicam->pdata->vb2_cache_flags;

	avicam->alloc_ctx = alloc_ctx;
	return 0;

 v4l2_init_failed:
	pinctrl_put(avicam->pctl);
 pinctrl_failed:
 no_pdata:
	kfree(avicam);
 alloc_failed:

	return ret;
}

static int __devexit avicam_remove(struct platform_device *pdev)
{
	struct avicam	*avicam = platform_get_drvdata(pdev);

	if (!avicam)
		return -ENODEV;

	avicam_v4l2_destroy(avicam);
	pinctrl_put(avicam->pctl);
	vb2_dma_contig_cleanup_ctx(avicam->alloc_ctx);
	kfree(avicam);

	dev_set_drvdata(&pdev->dev, NULL);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int avicam_resume(struct device *dev)
{
	struct avicam	*avicam = dev_get_drvdata(dev);

	if (!avicam)
		return -ENODEV;

	if (!avicam->streaming)
		/* Nothing to do */
		return 0;

	return avi_capture_resume(&avicam->capture_ctx);
}

static struct dev_pm_ops avicam_dev_pm_ops = {
       .resume  = &avicam_resume,
};

static struct platform_driver avicam_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm     = &avicam_dev_pm_ops,
	},
	.probe		= avicam_probe,
	.remove		= __devexit_p(avicam_remove),
};

static int __init avicam_init(void)
{
	int ret;

	ret = platform_driver_register(&avicam_driver);

	if (ret)
		return ret;

	return avicam_register_v4l2_subdevs();
}

static void __exit avicam_exit(void)
{
	platform_driver_unregister(&avicam_driver);
}

module_init(avicam_init);
module_exit(avicam_exit);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Host driver for P7 Advanced Video Interface");
MODULE_LICENSE("GPL");
