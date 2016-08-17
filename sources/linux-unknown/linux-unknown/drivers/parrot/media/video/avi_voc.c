/*
 *      linux/drivers/parrot/video/avi_voc.c
 *
 *      Copyright (C) 2011 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 *          Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    03-Mar-2011
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

#include <linux/list.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/gcd.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-common.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-ctrls.h>

#include "avi_v4l2.h"
#include "video/avi_limit.h"
#include "avi_voc.h"
#include "avi_voc_core.h"
#include "avi_voc_ctrl.h"

/*******************************************************************************
 * HELPERS
 ******************************************************************************/

static int avi_voc_instance(struct avi_voc *voc)
{
	struct platform_device *pdev;

	pdev = container_of(voc->dev, struct platform_device, dev);

	return pdev->id;
}

static inline struct avi_voc_fh *avi_voc_vfh_get_fh(struct v4l2_fh *vfh)
{
	return container_of(vfh, struct avi_voc_fh, vfh);
}

static void __maybe_unused avi_voc_print_debug(const char *func, struct avi_voc *voc)
{
	printk("=== VOC %d (%s)===\n", avi_voc_instance(voc), func);
	printk("        FMT={w=%4u, h=%4u}\n",
	       voc->pix.width, voc->pix.height);
	printk("        OVL={w=%4u, h=%4u, t=%4d, l=%4d }\n",
	       voc->win.w.width, voc->win.w.height,
	       voc->win.w.top, voc->win.w.left);
	printk("        CRP={w=%4u, h=%4u, t=%4d, l=%4d }\n",
	       voc->crop.c.width, voc->crop.c.height,
	       voc->crop.c.top, voc->crop.c.left);
	if (voc->display) {
		struct avi_segment_format	 display_fmt;
		avi_segment_get_input_format(voc->display, &display_fmt);
		printk("        OUT={w=%4u, h=%4u}\n",
		display_fmt.width, display_fmt.height);
	} else {
		printk("        OUTPUT NOT CONNECTED\n");
	}
}

/* Must be called with vbuf_lock held */
static void avi_voc_display(struct avi_voc *voc, struct avi_voc_vbuf *b)
{
	struct avi_segment_format	in_fmt;
	dma_addr_t			dma_addr;
	struct avi_dma_buffer           frame;

	dma_addr = vb2_dma_contig_plane_dma_addr(&b->vb, 0);

	avi_segment_get_input_format(voc->segment, &in_fmt);

	avi_v4l2_setup_dmabuf(&voc->win.w,
			      &in_fmt,
			      voc->plane0_size,
			      voc->plane1_size,
			      dma_addr,
			      b,
			      &frame);
	avi_segment_set_input_buffer(voc->segment, &frame);

	/* In case we're just starting and aren't visible yet */
	voc->hw_streaming = 1;
	if (!voc->hide)
		avi_segment_unhide(voc->segment);
}

/* Configure the segment after a crop/win change or on streamon
 *
 * Must be called with voc->vbuf_lock held
 */
static int avi_voc_update_segment_layout(struct avi_voc *voc)
{
	struct avi_segment_format dma_fmt;
	struct avi_segment_format out_fmt;
	struct avi_segment_layout layout;
	int                       ret;

	ret = avi_v4l2_to_segment_fmt(&voc->pix,
				      &voc->win.w,
				      &dma_fmt,
				      &voc->plane0_size,
				      &voc->plane1_size);
	if (ret)
		return ret;

	/* Since we have a scaler we can always have a progressive input (the
	 * scaler will do the rest)
	 * XXX: with s_ctrl scaler can now be disabled */
	dma_fmt.interlaced = 0;

	avi_segment_get_input_format(voc->display, &out_fmt);

	out_fmt.width  = voc->crop.c.width;
	out_fmt.height = voc->crop.c.height;

	layout.x      = voc->crop.c.left;
	layout.y      = voc->crop.c.top;

	if (avi_pixfmt_have_alpha(dma_fmt.pix_fmt))
		layout.alpha  = AVI_ALPHA_OSD;
	else
		layout.alpha  = voc->win.global_alpha;

	layout.hidden = 0;

	ret = avi_segment_set_format_and_layout(voc->segment,
	                                        &dma_fmt,
	                                        &out_fmt,
	                                        &layout);
	if (ret) {
		dev_err(voc->dev, "couldn't set segment format\n");
		return ret;
	}

	/* The config changed, we need to reconfigure the displayed frame */
	if (voc->displayed_frame)
		avi_voc_display(voc, voc->displayed_frame);

	return 0;
}

static int avi_voc_querycap(struct file			*file,
                            void			*vfh,
                            struct v4l2_capability	*cap)
{
	struct avi_voc	*voc = video_drvdata(file);

	strcpy(cap->driver, DRIVER_NAME);

	snprintf(cap->card, sizeof(cap->card), "avi-voc.%d",
		 avi_voc_instance(voc));
	snprintf(cap->bus_info, sizeof(cap->card), AVI_V4L2_BUS_VERSION);

	cap->device_caps  = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_OUTPUT |
	                    V4L2_CAP_VIDEO_OUTPUT_OVERLAY;

	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS ;

	return 0;
}

static void avi_voc_fill_outputs(struct avi_voc *voc) {
	int i;
	struct avi_segment *seg;
	__u8 name[32];

	voc->out_nb = 0;

	for (i = 0; i < AVI_VOC_ENUM_NB; i++) {
		snprintf(name, 32, "lcd.%d", i);
		seg = avi_segment_find(name);
		if (seg) {
			strlcpy(voc->outputs[voc->out_nb], name, 32);
			voc->out_nb++;
		}
	}

	for (i = 0; i < AVI_VOC_ENUM_NB; i++) {
		snprintf(name, 32, "r2r.%d", i);
		seg = avi_segment_find(name);
		if (seg) {
			strlcpy(voc->outputs[voc->out_nb], name, 32);
			voc->out_nb++;
		}
	}
}

static int avi_voc_enum_output(struct file		*file,
                               void			*vfh,
                               struct v4l2_output	*output)
{
	struct avi_voc *voc = video_drvdata(file);
	__u32 index = output->index;

	/* In our case possible ouputs are segments. While segments can be
	 * dynamically created we have a race condition between the enumeration
	 * and s_output due to a segment change decaling the indexes.
	 * Our solution to fix it is to create a table associating an index with
	 * an segment.
	 * The output list have to be recreated sometimes in order to be up to
	 * date. We update this list at each new enumation*/
	if (index == 0) {
		int i;
		avi_voc_fill_outputs(voc);

		/* Find index value of the current output */
		voc->out_id = -1;
		for (i = 0; i < voc->out_nb; i++)
			if (0 == strncmp(voc->outputs[i], voc->output,
					 sizeof(voc->output))) {
				voc->out_id = i;
				break;
			}
	}


	if (index >= voc->out_nb)
		return -EINVAL;


	memset(output, 0, sizeof(*output));

	output->index = index;
	strlcpy(output->name, voc->outputs[index], sizeof(output->name));
	/* XXX Analog? where does that come from? */
	output->type         = V4L2_OUTPUT_TYPE_ANALOG;
	/* No support for video standards.*/
	output->audioset     = 0;
	output->modulator    = 0;
	output->std          = 0;
	output->capabilities = 0;

	return 0;
}

static int avi_voc_s_output(struct file *file, void *priv, unsigned int index)
{
	struct avi_voc *voc = video_drvdata(file);

	if (index >= voc->out_nb)
		return -EINVAL;

	/* At the moment we dont change output while streaming */
	if (vb2_is_streaming(&voc->vb2_vidq))
		return -EBUSY;

	strlcpy(voc->output, voc->outputs[index], sizeof(voc->output));
	voc->out_id = index;
	voc->display = avi_segment_find(voc->output);

	return 0;
}

static int avi_voc_g_output(struct file *file, void *priv, unsigned int *index)
{
	struct avi_voc *voc = video_drvdata(file);

	/* Current output segment cant be find so it no longer exists */
	if (voc->out_id == -1)
		return -EINVAL;

	*index = voc->out_id;
	return 0;
}

static int avi_voc_g_parm(struct file			*file,
                          void				*vfh,
                          struct v4l2_streamparm	*parms)
{
	struct avi_voc		*voc = video_drvdata(file);
	struct v4l2_outputparm	*p   = &parms->parm.output;
	unsigned		 g;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	p->capability		    = V4L2_CAP_TIMEPERFRAME;
	p->timeperframe.numerator   = atomic_read(&voc->period_us);
	p->timeperframe.denominator = USEC_PER_SEC;

	/* I could return there but I prefer to reduce the fraction first */
	g = gcd(p->timeperframe.numerator, p->timeperframe.denominator);

	p->timeperframe.numerator   /= g;
	p->timeperframe.denominator /= g;

	return 0;
}


static int avi_voc_s_parm(struct file			*file,
                          void				*vfh,
                          struct v4l2_streamparm	*parms)
{
	struct avi_voc		*voc = video_drvdata(file);
	struct v4l2_outputparm	*p   = &parms->parm.output;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	/* Protect against division by 0. */
	if (p->timeperframe.denominator == 0)
		p->timeperframe.denominator = 1;

	if (p->timeperframe.numerator == 0)
		p->timeperframe.numerator = 25;

	/* Would overflow */
	if (p->timeperframe.numerator > (INT_MAX / USEC_PER_SEC))
		p->timeperframe.numerator = (INT_MAX / USEC_PER_SEC);

	/* Compute the value of the fraction in us, rounded to the nearest */
	atomic_set(&voc->period_us,
	           ((USEC_PER_SEC * p->timeperframe.numerator) +
		    (p->timeperframe.denominator / 2)) /
	           p->timeperframe.denominator);

	/* Update parms in case of rounding */
	return avi_voc_g_parm(file, vfh, parms);
}

static int avi_voc_enum_fmt_vid_out(struct file		*file,
                                    void		*vfh,
                                    struct v4l2_fmtdesc *fmt)
{
	return avi_v4l2_enum_fmt(fmt);
}

static int avi_voc_try_fmt_vid_out(struct file		*file,
                                   void			*vfh,
                                   struct v4l2_format	*f)
{
	struct avi_voc		*voc = video_drvdata(file);
	int			 ret;

	ret = avi_v4l2_try_fmt(f->type, &f->fmt.pix);
	if (ret) {
		dprintk(voc, "avi_v4l2_try_fmt failed\n");
		return ret;
	}

	if (f->fmt.pix.field != V4L2_FIELD_NONE &&
	    f->fmt.pix.field != V4L2_FIELD_INTERLACED) {
		/* We don't support the other interlacing formats yet */
		f->fmt.pix.field  = V4L2_FIELD_NONE;
	}

	return 0;
}

#define AVI_VOC_DISPLAY_V4L2_FORMAT(_voc, _f) do {			\
	switch ((_f)->type) {						\
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:				\
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:				\
		dprintk((_voc),						\
			"VIDEO_CAPTURE/OUTPUT: width: %d, height: %d, "	\
			"pixelformat: " FOURCC_FORMAT ", field: %d, "	\
			"bytesperline: %d, sizeimage: %d, "		\
			"colorspace: %d, "				\
			"priv: %p\n",					\
			(_f)->fmt.pix.width,				\
			(_f)->fmt.pix.height,				\
			FOURCC_SHOW((_f)->fmt.pix.pixelformat),		\
			(_f)->fmt.pix.field,				\
			(_f)->fmt.pix.bytesperline,			\
			(_f)->fmt.pix.sizeimage,			\
			(_f)->fmt.pix.colorspace,			\
			(void*)(_f)->fmt.pix.priv);			\
		break;							\
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:			\
		dprintk((_voc), "VIDEO_CAPTURE_MPLANE\n");		\
		break;							\
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:				\
		dprintk((_voc),						\
			"VIDEO_OVERLAY: left: %d, top: %d, "		\
			"width: %d, height: %d, field: %d, "		\
			"chromakey: %08x, clips: %pK, clipcount: %u "	\
			"bitmap: %p, global_alpha: %d\n",		\
			(_f)->fmt.win.w.left,				\
			(_f)->fmt.win.w.top,				\
			(_f)->fmt.win.w.width,				\
			(_f)->fmt.win.w.height,				\
			(_f)->fmt.win.field,				\
			(_f)->fmt.win.chromakey,			\
			(_f)->fmt.win.clips,				\
			(_f)->fmt.win.clipcount,			\
			(_f)->fmt.win.bitmap,				\
			(_f)->fmt.win.global_alpha);			\
		break;							\
	case V4L2_BUF_TYPE_VBI_CAPTURE:					\
		dprintk((_voc), "VBI_CAPTURE\n");			\
		break;							\
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:				\
		dprintk((_voc), "SLICED_VBI_CAPTURE\n");		\
		break;							\
	default:							\
		dprintk((_voc), "invalid format type: %d\n",		\
			(_f)->type);					\
	};								\
} while (0)

static int avi_voc_s_fmt_vid_out_unlocked(struct file		*file,
                                          void			*vfh,
                                          struct v4l2_format    *f)
{
	struct avi_voc			*voc = video_drvdata(file);
	struct avi_segment_format	 display_fmt;
	int				 ret;

	AVI_VOC_DISPLAY_V4L2_FORMAT(voc, f);

	if (vb2_is_streaming(&voc->vb2_vidq))
		/* Can't change format while streaming */
		return -EBUSY;

	ret = avi_voc_try_fmt_vid_out(file, vfh, f);
	if (ret)
		return ret;

	voc->pix  = f->fmt.pix;

	avi_segment_get_input_format(voc->display, &display_fmt);

	/* Reset overlay */
	voc->win.w.left	      = 0;
	voc->win.w.top	      = 0;
	voc->win.global_alpha = 0xff;
	voc->win.field        = V4L2_FIELD_NONE;

	voc->crop.type     = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	voc->crop.c.left   = 0;
	voc->crop.c.top    = 0;

	if (voc->no_scaler) {
		/* Without scaler we cant display more than the output size */
		voc->win.w.width   = min(voc->pix.width, display_fmt.width);
		voc->win.w.height  = min(voc->pix.height, display_fmt.height);
		/* crop and overlay have to be equals */
		voc->crop.c.width  = voc->win.w.width;
		voc->crop.c.height = voc->win.w.height;
	} else {
		voc->crop.c.width  = display_fmt.width;
		voc->crop.c.height = display_fmt.height;
		voc->win.w.width   = voc->pix.width;
		voc->win.w.height  = voc->pix.height;
	}

	return ret;
}

static int avi_voc_s_fmt_vid_out(struct file		*file,
                                 void			*vfh,
                                 struct v4l2_format     *f)
{
	struct avi_voc			*voc = video_drvdata(file);
	int				 ret;

	mutex_lock(&voc->lock);

	ret = avi_voc_s_fmt_vid_out_unlocked(file, vfh, f);

	mutex_unlock(&voc->lock);

	return ret;
};

static int avi_voc_g_fmt_vid_out(struct file		*file,
                                 void			*vfh,
                                 struct v4l2_format	*f)
{
	struct avi_voc	*voc = video_drvdata(file);

	mutex_lock(&voc->lock);
	f->fmt.pix = voc->pix;
	mutex_unlock(&voc->lock);

	return 0;
}

static int avi_voc_try_fmt_vid_overlay_unlocked(struct avi_voc		*voc,
						struct v4l2_format	*f)
{
	struct v4l2_window	*win = &f->fmt.win;
	struct avi_dma_pixfmt    pixfmt;

	pixfmt = avi_v4l2_to_avi_pixfmt(voc->pix.pixelformat);
	if (pixfmt.id == AVI_INVALID_FMTID)
		return -EINVAL;

	avi_limit_adjust_w_strip(pixfmt, voc->pix.width,
	                         &win->w.width, &win->w.left);
	avi_limit_adjust_h_strip(pixfmt, voc->pix.height,
	                         &win->w.height, &win->w.top);

	win->field = V4L2_FIELD_NONE;

	return 0;
}

static int avi_voc_try_fmt_vid_overlay(struct file		*file,
				       void			*vfh,
				       struct v4l2_format	*f)
{
	struct avi_voc		*voc = video_drvdata(file);
	int			 ret;

	mutex_lock(&voc->lock);

	ret = avi_voc_try_fmt_vid_overlay_unlocked(voc, f);

	mutex_unlock(&voc->lock);

	return ret;
}


static int avi_voc_s_fmt_vid_overlay(struct file	*file,
                                     void		*vfh,
                                     struct v4l2_format *f)
{
	struct avi_voc		*voc = video_drvdata(file);
	struct v4l2_window	 old_win;
	struct v4l2_crop	 old_crop;
	int			 ret;

	mutex_lock(&voc->lock);

	AVI_VOC_DISPLAY_V4L2_FORMAT(voc, f);

	ret = avi_voc_try_fmt_vid_overlay_unlocked(voc, f);
	if (ret)
		goto unlock;

	spin_lock_irq(&voc->vbuf_lock);

	old_win	 = voc->win;
	voc->win = f->fmt.win;

	/* Without scaler we have to keep overlay and crop sizes equals */
	if (voc->no_scaler) {
		struct avi_segment_format	 display_fmt;
		avi_segment_get_input_format(voc->display, &display_fmt);

		voc->win.w.width   = min((unsigned) voc->win.w.width,
		                         display_fmt.width);
		voc->win.w.height  = min((unsigned) voc->win.w.height,
		                         display_fmt.height);

		old_crop = voc->crop;
		voc->crop.c.width  = voc->win.w.width;
		voc->crop.c.height = voc->win.w.height;
	}

	if (vb2_is_streaming(&voc->vb2_vidq)) {
		ret = avi_voc_update_segment_layout(voc);
		if (ret) {
			voc->win = old_win;
			if (voc->no_scaler)
				voc->crop = old_crop;
			goto unlock_spin;
		}
	}

	ret = 0;

unlock_spin:
	spin_unlock_irq(&voc->vbuf_lock);
unlock:
	mutex_unlock(&voc->lock);

	return ret;
}

static int avi_voc_g_fmt_vid_overlay(struct file	*file,
                                 void			*vfh,
                                 struct v4l2_format	*f)
{
	struct avi_voc *voc = video_drvdata(file);

	mutex_lock(&voc->lock);

	f->fmt.win = voc->win;

	mutex_unlock(&voc->lock);

	return 0;
}

static int avi_voc_cropcap_unlocked(struct avi_voc      *voc,
                                    struct v4l2_cropcap *cap)
{
	struct avi_segment_format display_fmt;

	avi_segment_get_input_format(voc->display, &display_fmt);

	cap->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	cap->bounds.top	   = 0;
	cap->bounds.left   = 0;
	cap->bounds.width  = display_fmt.width;
	cap->bounds.height = display_fmt.height;

	cap->defrect = cap->bounds;

	cap->pixelaspect.numerator   = 1;
	cap->pixelaspect.denominator = 1;

	return 0;
}

static int avi_voc_cropcap(struct file		*file,
                           void			*vfh,
                           struct v4l2_cropcap	*cap)
{
	struct avi_voc			*voc = video_drvdata(file);
	int				 ret;

	mutex_lock(&voc->lock);

	ret = avi_voc_cropcap_unlocked(voc, cap);

	mutex_unlock(&voc->lock);

	return 0;
}

static int avi_voc_g_crop(struct file		*file,
                          void			*vfh,
                          struct v4l2_crop	*crop)
{
	struct avi_voc	*voc = video_drvdata(file);

	mutex_lock(&voc->lock);

	*crop = voc->crop;

	mutex_unlock(&voc->lock);

	return 0;
}

static int avi_voc_s_crop(struct file		*file,
                          void			*vfh,
                          struct v4l2_crop	*crop)
{
	struct avi_voc		*voc = video_drvdata(file);
	struct v4l2_cropcap	 ccap;
	struct v4l2_crop	 old_crop;
	int			 ret;

	mutex_lock(&voc->lock);

	dprintk(voc,
		"type: %d, left: %d, top: %d, width: %d, height: %d\n",
		crop->type,
		crop->c.left,
		crop->c.top,
		crop->c.width,
		crop->c.height);

	if (crop->type != V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		ret = -EINVAL;
		goto unlock;
	}

	/* When no scaler is used we cant crop. Instead of returning an error
	 * we adjust configuration with no cropping. */
	if (voc->no_scaler) {
		struct avi_segment_format display_fmt;
		avi_segment_get_input_format(voc->display, &display_fmt);

		voc->crop = *crop;
		/* S_CROP is an write only so we dont modify parameter values */
		voc->crop.c.width  = voc->win.w.width;
		voc->crop.c.height = voc->win.w.height;
		/* Left and top are adjusted to avoid part of the image to be out of display.
		 * Such configuration can be achieved by setting overlay window first. */
		voc->crop.c.left   = min((unsigned) voc->crop.c.left,
		                         display_fmt.width - voc->crop.c.width);
		voc->crop.c.top    = min((unsigned)voc->crop.c.top,
		                         display_fmt.height - voc->crop.c.height);
		if (voc->crop.c.left < 0)
			voc->crop.c.left = 0;
		if (voc->crop.c.top < 0)
			voc->crop.c.top = 0;

		ret = 0;
		goto unlock;
	}

	ret = avi_voc_cropcap_unlocked(voc, &ccap);
	if (ret)
		goto unlock;

	avi_v4l2_crop_adjust(&crop->c, &ccap.bounds);

	if (crop->c.width == 0 || crop->c.height == 0) {
		ret = -EINVAL;
		goto unlock;
	}

	spin_lock_irq(&voc->vbuf_lock);

	old_crop  = voc->crop;
	voc->crop = *crop;

	if (vb2_is_streaming(&voc->vb2_vidq)) {
		ret = avi_voc_update_segment_layout(voc);
		if (ret) {
			voc->crop = old_crop;
			goto unlock_spin;
		}
	}

	ret = 0;

unlock_spin:
	spin_unlock_irq(&voc->vbuf_lock);
unlock:
	mutex_unlock(&voc->lock);
	return ret;
}

static inline void timeval_add_usec(struct timeval* to, unsigned int usecs)
{
	to->tv_usec += usecs;
	if (to->tv_usec >= USEC_PER_SEC) {
		to->tv_usec -= USEC_PER_SEC;
		to->tv_sec++;
	}
}

static inline int timeval_isnull(const struct timeval *tv)
{
	return tv->tv_sec == 0 && tv->tv_usec == 0;
}

/* Must be called with voc->vbuf_lock held */
static inline void avi_voc_handle_expired_frame(struct avi_voc *voc)
{
// XXX : is it usefull with videobuf2 ?
#if 1
	if (voc->expired_frame) {
		/* This frame is no longer displayed, we can safely send it back
		 * to the user. */
		vb2_buffer_done(&voc->expired_frame->vb, VB2_BUF_STATE_DONE);
		voc->expired_frame = NULL;
	}
#endif
}

/* Main interrupt while streaming */
static unsigned avi_voc_interrupt(struct avi_segment *s, enum avi_field* f)
{
	struct avi_voc		*voc  = s->priv;
	struct avi_voc_vbuf     *candidate;
	struct avi_voc_vbuf     *next = NULL;
	struct timeval		 display_date;
	struct timeval		 target_date;

	v4l2_get_timestamp(&display_date);

	/* The frame will be displayed at the next interrupt, not right now */
	timeval_add_usec(&display_date, voc->display->period_us);

	target_date = display_date;

	/* We put ourselves at the centre of the frame period, we want to
	 * display the frame the closest in time (in the past or future). */
	timeval_add_usec(&target_date, atomic_read(&voc->period_us) / 2);

	spin_lock(&voc->vbuf_lock);

	avi_voc_handle_expired_frame(voc);

	/* Look for a new frame to display. The algorithm is to iterate through
	 * the bufferqueue until we reach the end or a frame that's not supposed
	 * to be displayed yet. We then take the one before that (if any). This
	 * takes care of dropping frames if we're late (we take the most
	 * up-to-date frame). Of course this assumes that the buffers are queued
	 * in order. */
	while (!list_empty(&voc->bufqueue)) {

		candidate = list_first_entry(&voc->bufqueue,
		                             struct avi_voc_vbuf,
		                             list);

		if (!timeval_isnull(&candidate->vb.v4l2_buf.timestamp) &&
		    timeval_compare(&candidate->vb.v4l2_buf.timestamp,
		                    &target_date) > 0)
			/* Too early to display this frame. */
			break;

		/* This frame is ready to be displayed */
		if (next) {
			/* we have to drop the previous candidate, we're running
			 * late! */
			vb2_buffer_done(&next->vb, VB2_BUF_STATE_DONE);
		}

		next = candidate;

		list_del(&next->list);
		next->in_queue = 0;
		voc->frame_count++;

		next->vb.v4l2_buf.sequence  = voc->frame_count;
		/* I wonder if I should set this when we skip the frame? */
		next->vb.v4l2_buf.timestamp = display_date;
	}

	if (next) {
		/* The currently displayed frame (if any) will be released at
		 * the next interrupt */
		voc->expired_frame   = voc->displayed_frame;
		voc->displayed_frame = next;
	}

	if (next)
		avi_voc_display(voc, next);

	spin_unlock(&voc->vbuf_lock);

	return 0;
}

static unsigned avi_voc_stopped(struct avi_segment *s, enum avi_field* f)
{
	struct avi_voc *voc = s->priv;

	spin_lock(&voc->vbuf_lock);

	avi_voc_handle_expired_frame(voc);

	spin_unlock(&voc->vbuf_lock);

	avi_segment_register_irq(voc->segment, NULL);

	/* Streamoff sequence is complete */
	wake_up_interruptible(&voc->waitq);

	return 0;
}

/* IRQ handler called when we want to shutdown the stream */
static unsigned avi_voc_finish(struct avi_segment *s, enum avi_field* f)
{
	struct avi_voc *voc = s->priv;

	spin_lock(&voc->vbuf_lock);

	avi_voc_handle_expired_frame(voc);

	spin_unlock(&voc->vbuf_lock);

	avi_segment_hide(voc->segment);
	avi_segment_deactivate(voc->segment);

	voc->expired_frame	     = voc->displayed_frame;
	voc->displayed_frame = NULL;

	/* At the next interrupt we're sure we're no longer in use */
	avi_segment_register_irq(voc->segment, &avi_voc_stopped);

	return 0;
}

static void avi_voc_cleanup_leftovers(struct avi_voc *voc)
{
	unsigned long flags;

	spin_lock_irqsave(&voc->vbuf_lock, flags);

	/* Make sure left no frame lying around */
	avi_voc_handle_expired_frame(voc);

	voc->expired_frame   = voc->displayed_frame;
	voc->displayed_frame = NULL;

	avi_voc_handle_expired_frame(voc);

	voc->expired_frame = NULL;

	spin_unlock_irqrestore(&voc->vbuf_lock, flags);
}

static struct v4l2_ioctl_ops const avi_voc_ioctl_ops = {
	.vidioc_querycap            = &avi_voc_querycap,
	.vidioc_enum_output         = &avi_voc_enum_output,
	.vidioc_s_output            = &avi_voc_s_output,
	.vidioc_g_output            = &avi_voc_g_output,
	.vidioc_g_parm              = &avi_voc_g_parm,
	.vidioc_s_parm              = &avi_voc_s_parm,
	.vidioc_enum_fmt_vid_out    = &avi_voc_enum_fmt_vid_out,
	.vidioc_try_fmt_vid_out     = &avi_voc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out       = &avi_voc_s_fmt_vid_out,
	.vidioc_g_fmt_vid_out       = &avi_voc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_overlay = &avi_voc_try_fmt_vid_overlay,
	.vidioc_s_fmt_vid_overlay   = &avi_voc_s_fmt_vid_overlay,
	.vidioc_g_fmt_vid_overlay   = &avi_voc_g_fmt_vid_overlay,
	.vidioc_cropcap             = &avi_voc_cropcap,
	.vidioc_g_crop              = &avi_voc_g_crop,
	.vidioc_s_crop              = &avi_voc_s_crop,
	.vidioc_reqbufs             = &vb2_ioctl_reqbufs,
	.vidioc_querybuf            = &vb2_ioctl_querybuf,
	.vidioc_qbuf                = &vb2_ioctl_qbuf,
	.vidioc_dqbuf               = &vb2_ioctl_dqbuf,
	.vidioc_expbuf              = &vb2_ioctl_expbuf,
	.vidioc_streamon            = &vb2_ioctl_streamon,
	.vidioc_streamoff           = &vb2_ioctl_streamoff,
};

/*******************************************************************************
 * VIDEOBUF2 OPERATIONS
 ******************************************************************************/

static int avi_voc_queue_setup(struct vb2_queue *vq,
                               const struct v4l2_format *fmt,
                               unsigned int *nbuffers, unsigned int *nplanes,
                               unsigned int sizes[], void *alloc_ctxs[])
{
	struct avi_voc  *voc = vb2_get_drv_priv(vq);
	//unsigned long size;

	// XXX : fix to update size
	struct avi_segment_format dma_fmt;
        unsigned                         plane0_size;
        unsigned                         plane1_size;
        int ret;
	ret = avi_v4l2_to_segment_fmt(&voc->pix,
				      &voc->win.w,
				      &dma_fmt,
				      &plane0_size,
				      &plane1_size);
	if (ret)
		return ret;

	sizes[0] = plane0_size + plane1_size;

	if (*nbuffers < AVI_VOC_MIN_BUFFERS)
		*nbuffers = AVI_VOC_MIN_BUFFERS;

	// XXX : it would be great to have such a mecanism in avi_voc to adapt
	// the number of buffers
//	while (size * *nbuffers > vid_limit * 4096 * 4096)
//		(*nbuffers)--;

	*nplanes = 1;

	alloc_ctxs[0] = vb2_dma_contig_init_ctx(voc->dev);

	INIT_LIST_HEAD(&voc->bufqueue);

	return 0;
}

/* This function is called with avicam->vbq_lock held (no need to protect
 * ourselves when we play with the dmaqueue) */
static void avi_voc_buffer_queue(struct vb2_buffer *vb)
{
	int ret;
	struct avi_voc  *voc = vb2_get_drv_priv(vb->vb2_queue);
	struct avi_voc_vbuf *buf = container_of(vb, struct avi_voc_vbuf, vb);

	unsigned long flags;

	if (vb->acquire_fence) {
		ret = sync_fence_wait(vb->acquire_fence, 100);
		if(ret < 0){
			dprintk(voc,"Acquire fence timed out (100ms)");
		}
		sync_fence_put(vb->acquire_fence);
		vb->acquire_fence = NULL;
	}

	spin_lock_irqsave(&voc->vbuf_lock, flags);

	list_add_tail(&buf->list, &voc->bufqueue);
	buf->in_queue = 1;
	spin_unlock_irqrestore(&voc->vbuf_lock, flags);
}

static void avi_voc_buffer_cleanup(struct vb2_buffer *vb)
{
	struct avi_voc  *voc = vb2_get_drv_priv(vb->vb2_queue);
	struct avi_voc_vbuf *buf = container_of(vb, struct avi_voc_vbuf, vb);
	unsigned long flags;
	spin_lock_irqsave(&voc->vbuf_lock, flags);
	if(buf->in_queue)
		list_del(&buf->list);
	buf->in_queue = 0;
	spin_unlock_irqrestore(&voc->vbuf_lock, flags);
}

static int avi_voc_buffer_init(struct vb2_buffer *vb)
{
	struct avi_voc  *voc = vb2_get_drv_priv(vb->vb2_queue);
	struct avi_voc_vbuf *buf = container_of(vb, struct avi_voc_vbuf, vb);
	unsigned long flags;
        spin_lock_irqsave(&voc->vbuf_lock, flags);
	INIT_LIST_HEAD(&buf->list);
	buf->in_queue = 0;
        spin_unlock_irqrestore(&voc->vbuf_lock, flags);
	return 0;
}
int avi_voc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct avi_voc *voc = vb2_get_drv_priv(q);
	struct avi_segment_format        display_fmt;
	enum avi_colorspace		 in_cspace;
	struct avi_dma_pixfmt		 in_pixfmt;
	unsigned long			 caps = AVI_CAP_DMA_IN;
	int				 ret;

	mutex_lock(&voc->lock);

	voc->frame_count = 0;

	if (!voc->no_scaler)
		caps |= AVI_CAP_SCAL | AVI_CAP_BUFFER;

	avi_segment_get_input_format(voc->display, &display_fmt);

	in_cspace = avi_v4l2_to_avi_cspace(voc->pix.colorspace);

	if (in_cspace != display_fmt.colorspace)
		caps |= AVI_CAP_CONV;

	in_pixfmt = avi_v4l2_to_avi_pixfmt(voc->pix.pixelformat);
	if (avi_pixfmt_is_planar(in_pixfmt))
		caps |= AVI_CAP_PLANAR;

	voc->segment = avi_segment_build(&caps,
					 "voc",
					 avi_voc_instance(voc),
					 -1,
					 voc->dev);
	if (IS_ERR(voc->segment)) {
		ret = PTR_ERR(voc->segment);
		dev_err(voc->dev, "failed to build overlay segment caps=%lx\n",
		        caps);
		goto no_segment;
	}

	voc->segment->priv = voc;

	/* Configure the segment */
	ret = avi_voc_update_segment_layout(voc);
	if (ret)
		goto bad_segment_fmt;

	/* Wait until a frame is configured to actually display the voc */
	avi_segment_hide(voc->segment);

	ret = avi_segment_activate(voc->segment);
	if (ret)
		goto segment_activate_failed;

	ret = avi_segment_connect(voc->segment, voc->display, voc->zorder);
	if (ret) {
		dev_err(voc->dev,
		        "couldn't connect voc and output segments (%s)\n",
		        voc->display->id);
		goto connect_failed;
	}

	avi_segment_register_irq(voc->segment, &avi_voc_interrupt);

	goto unlock;

 connect_failed:
	avi_segment_deactivate(voc->segment);
 segment_activate_failed:
 bad_segment_fmt:
	avi_segment_teardown(voc->segment);
 no_segment:
	voc->segment = NULL;
	voc->plane0_size = 0;
	voc->plane1_size = 0;
unlock:
	mutex_unlock(&voc->lock);

	return ret;
}

static int avi_voc_is_streaming_ended(struct avi_voc *voc)
{
	return (voc->expired_frame == NULL && voc->displayed_frame == NULL);
}

static int avi_voc_stop_streaming(struct vb2_queue *q)
{
	struct avi_voc *voc = vb2_get_drv_priv(q);

	mutex_lock(&voc->lock);

	avi_segment_register_irq(voc->segment, &avi_voc_finish);

	/* I use an arbitrary timeout value of 1 second. I don't check for
	 * errors, if for some reason the sequence does not finish the worst
	 * that can happen is one or two gargabe frames being displayed (and if
	 * the sequence does not finish in all likelyhood it means that the
	 * display stopped) */
	wait_event_interruptible_timeout(voc->waitq,
					 avi_voc_is_streaming_ended(voc),
					 HZ);

	avi_segment_register_irq(voc->segment, NULL);
	voc->hw_streaming = 0;
	avi_segment_deactivate(voc->segment);

	/* In case the streamoff sequence didn't complete properly, make sure
	 * no frame was left lying around */
	avi_voc_cleanup_leftovers(voc);

	avi_segment_disconnect(voc->segment, voc->display);
	avi_segment_teardown(voc->segment);
	voc->segment = NULL;

	voc->plane0_size = 0;
	voc->plane1_size = 0;

	mutex_unlock(&voc->lock);

	return 0;
}

static void avi_voc_lock(struct vb2_queue *q)
{
	struct avi_voc *voc = vb2_get_drv_priv(q);
	mutex_lock(&voc->vb2_lock);
}

static void avi_voc_unlock(struct vb2_queue *q)
{
	struct avi_voc *voc = vb2_get_drv_priv(q);
	mutex_unlock(&voc->vb2_lock);
}

static struct vb2_ops const avi_voc_vqueue_ops = {
	.queue_setup            = &avi_voc_queue_setup,
	.buf_queue              = &avi_voc_buffer_queue,
	.stop_streaming         = &avi_voc_stop_streaming,
	.start_streaming        = &avi_voc_start_streaming,
	.wait_prepare           = &avi_voc_unlock,
	.wait_finish            = &avi_voc_lock,
	.buf_cleanup		= &avi_voc_buffer_cleanup,
	.buf_init 		= &avi_voc_buffer_init,
};

/*******************************************************************************
 * V4L2 FILE OPERATIONS
 ******************************************************************************/

static int avi_voc_open(struct file *file)
{
	struct avi_voc			*voc = video_drvdata(file);
	struct avi_voc_fh		*fh;
	struct avi_segment_format	 display_fmt;
	struct v4l2_format		 v4l2_fmt = {0};
	int				 ret;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (!fh) {
		ret = -ENOMEM;
		goto no_mem;
	}

	v4l2_fh_init(&fh->vfh, voc->vdev);
	v4l2_fh_add(&fh->vfh);
	file->private_data = &fh->vfh;

	voc->user_count++;

	/* With multi open we dont need to go further if VOC is already
	 * initialized */
	if (voc->user_count > 1)
		return 0;

	mutex_lock(&voc->lock);

	/* Fresh open, let's reinitialize everything to sane values */

	init_waitqueue_head(&voc->waitq);

	voc->display = avi_segment_find(voc->output);
	if (!voc->display) {
		dev_err(voc->dev,
		        "Couldn't get display segment \"%s\"",
		        voc->output);
		ret = -ENODEV;
		goto no_display;
	}

	/* Use sane value for default format  */
	avi_segment_get_input_format(voc->display, &display_fmt);

	v4l2_fmt.type		     = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	v4l2_fmt.fmt.pix.width	     = display_fmt.width;
	v4l2_fmt.fmt.pix.height	     = display_fmt.height;
	v4l2_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB32;
	v4l2_fmt.fmt.pix.field	     = V4L2_FIELD_NONE;
	v4l2_fmt.fmt.pix.colorspace  = V4L2_COLORSPACE_SRGB;

	ret = avi_voc_s_fmt_vid_out_unlocked(file, &fh->vfh, &v4l2_fmt);
	if (ret)
		goto bad_fmt;

	/* Set the default framerate to that of the display. I'm not sure if
	 * that's the right thing to do but it seems that the v4l2 compliance
	 * tool complains if I set the framerate (G_PARM) to 0 */
	atomic_set(&voc->period_us, voc->display->period_us);

	voc->hw_streaming = 0;
	voc->hide = 0;
	avi_voc_ctrl_unhide(voc);

	ret = 0;
	goto unlock;

 bad_fmt:
	if (voc->user_count == 1)
		voc->display = NULL;
 no_display:
	v4l2_fh_del(&fh->vfh);
	file->private_data = NULL;
	kfree(fh);
	voc->user_count--;
 unlock:
	mutex_unlock(&voc->lock);
 no_mem:

	return ret;
}

static int avi_voc_release(struct file *file)
{
	struct avi_voc  *voc = video_drvdata(file);
	int ret;

	ret = vb2_fop_release(file);

	voc->user_count--;
	if (voc->user_count == 0)
		voc->display = NULL;

	return ret;
}

static struct v4l2_file_operations const avi_voc_fops = {
	.owner          = THIS_MODULE,
	.open           = &avi_voc_open,
	.release        = &avi_voc_release,
	.mmap           = &vb2_fop_mmap,
	.poll           = &vb2_fop_poll,
	.unlocked_ioctl = &video_ioctl2,
};

/*******************************************************************************
 * SYSFS INTERFACE
 ******************************************************************************/

static ssize_t avi_voc_show_zorder(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
	struct avi_voc *voc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", voc->zorder);
}

static ssize_t avi_voc_store_zorder(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	struct avi_voc *voc = dev_get_drvdata(dev);
	long    zorder;
	int     status;

	status = strict_strtol(buf, 0, &zorder);
	if (status < 0)
		return -EINVAL;

	/* We dont change zorder of an already connected segment */
	if(voc->segment && avi_segment_connected(voc->segment))
		return -EBUSY;


	voc->zorder = zorder;

	return count;
}

static ssize_t avi_voc_show_frame_count(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct avi_voc *voc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", voc->frame_count);
}

static ssize_t avi_voc_show_no_scaler(struct device *dev,
                                   struct device_attribute *attr,
                                   char *buf)
{
	struct avi_voc *voc = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", voc->no_scaler);
}

static ssize_t avi_voc_store_no_scaler(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
	struct avi_voc  *voc = dev_get_drvdata(dev);
	long             no_scaler;
	int              status;
	int              ret;

	status = strict_strtol(buf, 0, &no_scaler);
	if (status < 0)
		return -EINVAL;

	mutex_lock(&voc->lock);
	/* We cant remove scaler is segment is already built */
	if (voc->segment) {
		ret = -EBUSY;
		goto unlock;
	}

	ret = count;
	voc->no_scaler = no_scaler;
unlock:
	mutex_unlock(&voc->lock);

	return ret;
}

static struct device_attribute avi_voc_sysfs_attrs[] = {
	__ATTR(zorder, S_IRUGO | S_IWUSR,
	       avi_voc_show_zorder, avi_voc_store_zorder),
	__ATTR(frame_count, S_IRUGO,
	       avi_voc_show_frame_count, NULL),
	__ATTR(no_scaler, S_IRUGO | S_IWUSR,
	       avi_voc_show_no_scaler, avi_voc_store_no_scaler),
};

/*******************************************************************************
 * INIT / REMOVE
 ******************************************************************************/

static int avi_voc_v4l2_init(struct avi_voc *voc)
{
	struct video_device	*vdev;
	int			 i;
	int			 ret;

	spin_lock_init(&voc->vbuf_lock);
	mutex_init(&voc->lock);

	voc->v4l2_dev = avi_v4l2_get_device();
	if (!voc->v4l2_dev) {
		ret = -ENODEV;
		goto no_v4l2_dev;
	}

	memset(&voc->vb2_vidq, 0, sizeof(voc->vb2_vidq));
	voc->vb2_vidq.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	voc->vb2_vidq.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	voc->vb2_vidq.drv_priv = voc;
	voc->vb2_vidq.buf_struct_size = sizeof(struct avi_voc_vbuf);
	voc->vb2_vidq.ops = &avi_voc_vqueue_ops;
	voc->vb2_vidq.mem_ops = &vb2_dma_contig_memops;
	ret = vb2_queue_init(&voc->vb2_vidq);
	if (ret)
		goto vb2_queue_init_failed;

	vdev = video_device_alloc();
	if (!vdev) {
		ret = -ENOMEM;
		goto vdev_alloc_failed;
	}

	voc->vdev = vdev;
	mutex_init(&voc->vb2_lock);

	strlcpy(vdev->name, dev_name(voc->dev), sizeof(vdev->name));

	vdev->parent	   = voc->dev;
	vdev->current_norm = V4L2_STD_UNKNOWN;
	vdev->fops	   = &avi_voc_fops;
	vdev->ioctl_ops	   = &avi_voc_ioctl_ops;
	vdev->release	   = &video_device_release;
	vdev->vfl_type	   = VFL_TYPE_GRABBER;
	vdev->tvnorms	   = V4L2_STD_UNKNOWN;
	/* We handle the locking ourselves */
	vdev->lock	   = &voc->vb2_lock;
	vdev->queue        = &voc->vb2_vidq;

	video_set_drvdata(vdev, voc);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto video_register_failed;

	if (avi_voc_ctrl_create(voc))
		goto ctrl_failed;
	v4l2_ctrl_handler_setup(&voc->ctrl_handler);

	for (i = 0; i < ARRAY_SIZE(avi_voc_sysfs_attrs); i++) {
		ret = device_create_file(&voc->vdev->dev,
					 &avi_voc_sysfs_attrs[i]);
		if (ret) {
			while (i--) {
				device_remove_file(&voc->vdev->dev,
						   &avi_voc_sysfs_attrs[i]);
				goto sysfs_failed;
			}
		}
	}

	return 0;

 sysfs_failed:
	video_unregister_device(voc->vdev);
 ctrl_failed:
	avi_voc_ctrl_free(voc);
 video_register_failed:
	video_device_release(voc->vdev);
 vdev_alloc_failed:
 vb2_queue_init_failed:
	avi_v4l2_put_device(voc->v4l2_dev);
 no_v4l2_dev:
	return ret;
}

static void __devexit avi_voc_v4l2_destroy(struct avi_voc *voc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(avi_voc_sysfs_attrs); i++)
		device_remove_file(&voc->vdev->dev, &avi_voc_sysfs_attrs[i]);
	avi_voc_ctrl_free(voc);
	video_unregister_device(voc->vdev);
	video_device_release(voc->vdev);
	avi_v4l2_put_device(voc->v4l2_dev);
}

static int __devinit avi_voc_probe(struct platform_device* pdev)
{
	struct avi_voc			*voc;
	const struct avi_voc_plat_data	*pdata;
	int				 ret;

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "cant get platform data\n");
		ret = -EINVAL;
		goto no_pdata;
	}

	voc = kzalloc(sizeof(*voc), GFP_KERNEL);
	if (!voc) {
		dev_err(&pdev->dev, "failed to allocate voc struct\n");
		ret = -ENOMEM;
		goto alloc_failed;
	}

	voc->dev   = &pdev->dev;
	strlcpy(voc->output, pdata->display, sizeof(voc->output));

	ret = avi_voc_v4l2_init(voc);
	if (ret)
		goto v4l2_init_failed;

	dev_info(&pdev->dev,
	         "V4L2 device successfuly registered as %s\n",
	         video_device_node_name(voc->vdev));

	dev_set_drvdata(&pdev->dev, voc);
	platform_set_drvdata(pdev, voc);

	return 0;

 v4l2_init_failed:
	kfree(voc);
 alloc_failed:
 no_pdata:
	return ret;
}

static int __devexit avi_voc_remove(struct platform_device* pdev)
{
	struct avi_voc *voc = platform_get_drvdata(pdev);

	if (!voc)
		return -ENODEV;

	avi_voc_v4l2_destroy(voc);
	kfree(voc);

	platform_set_drvdata(pdev, NULL);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static int avi_voc_suspend(struct device *dev)
{
	dev_info(dev, "voc shutting down\n");

	return 0;
}

static int avi_voc_resume(struct device *dev)
{
	struct avi_voc *voc = dev_get_drvdata(dev);

	dev_info(dev, "voc resuming\n");

	if (voc->segment) {
		if (voc->displayed_frame)
			avi_voc_display(voc, voc->displayed_frame);
		avi_segment_activate(voc->segment);
	}

	return 0;
}

static struct dev_pm_ops avi_voc_dev_pm_ops = {
       .suspend = &avi_voc_suspend,
       .resume  = &avi_voc_resume,
};

static struct platform_driver avi_voc_driver = {
	.driver = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.pm     = &avi_voc_dev_pm_ops,
	},
	.probe  = &avi_voc_probe,
	.remove = __devexit_p(&avi_voc_remove),
};

static int __init avi_init_voc(void)
{
	return platform_driver_register(&avi_voc_driver);
}
module_init(avi_init_voc);

static void __exit avi_exit_voc(void)
{
	platform_driver_unregister(&avi_voc_driver);
}
module_exit(avi_exit_voc);

MODULE_AUTHOR("Gregor Boirie <gregor.boirie@parrot.com>");
MODULE_AUTHOR("Victor Lambret <victor.lambret.ext@parrot.com>");
MODULE_DESCRIPTION("Parrot Advanced Video Interface video output overlays");
MODULE_LICENSE("GPL");
