#include <linux/module.h>
#include <media/v4l2-ioctl.h>

#include "avi_stats.h"

#define AVI_STATS_DEFAULT_THUMB_WIDTH	64
#define AVI_STATS_DEFAULT_THUMB_HEIGHT	48
#define AVI_STATS_DEFAULT_WIDTH		(AVI_STATS_DEFAULT_THUMB_WIDTH * 6)
#define AVI_STATS_DEFAULT_HEIGHT	(AVI_STATS_DEFAULT_THUMB_HEIGHT + 1)

void avi_stats_done(struct avi_stats *stats, struct avi_dma_buffer *frame,
		    u32 sequence, int requeue)
{
	enum vb2_buffer_state state = VB2_BUF_STATE_DONE;
	struct avi_vbuf *vbuf = frame->priv;
	unsigned long flags;

	if (!frame->plane0.dma_addr || !stats->streaming)
		return;

	/* Bad buffer */
	if (frame->status != AVI_BUFFER_DONE) {
		/* Requeue buffer instead of returning to V4L2 */
		if (requeue) {
			spin_lock_irqsave(&stats->vbq_lock, flags);
			list_add_tail(&vbuf->list, &stats->bufqueue);
			spin_unlock_irqrestore(&stats->vbq_lock, flags);
			return;
		}
		dev_err(stats->vdev->parent, "couldn't capture stats\n");
	}

	v4l2_get_timestamp(&vbuf->vb.v4l2_buf.timestamp);
	vbuf->vb.v4l2_buf.sequence = sequence;

	if (frame->status != AVI_BUFFER_DONE)
		state = VB2_BUF_STATE_ERROR;

	vb2_buffer_done(&vbuf->vb, state);
}
EXPORT_SYMBOL(avi_stats_done);

void avi_stats_next(struct avi_stats *stats, struct avi_dma_buffer *frame)
{
	struct avi_vbuf *vbuf = NULL;
	unsigned long flags;
	dma_addr_t dma_addr;

	if (!stats->streaming)
		return;

	spin_lock_irqsave(&stats->vbq_lock, flags);

	if (stats->streaming && !list_empty(&stats->bufqueue)) {
		vbuf = list_first_entry(&stats->bufqueue, struct avi_vbuf,
					list);

		list_del(&vbuf->list);

		vbuf->vb.state = VB2_BUF_STATE_ACTIVE;
	}

	spin_unlock_irqrestore(&stats->vbq_lock, flags);

	if (!vbuf)
		/* exhausted buffer queue */
		return;

	/* Get DMA address from vb2 buffer */
	dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0);

	/* Setup AVI DMA buffer with crop */
	avi_v4l2_setup_dmabuf(&stats->crop, &stats->avi_fmt, stats->plane0_size,
			      stats->plane1_size, dma_addr, vbuf, frame);
}
EXPORT_SYMBOL(avi_stats_next);

int avi_stats_apply_format(struct avi_stats *stats,
			   struct avi_node *bayer_node,
			   struct avi_segment *stats_seg,
			   const struct avi_segment_format *src_fmt, int force)
{
	struct avi_segment_format stats_fmt = {
		.width = stats->avi_fmt.width,
		.height = stats->avi_fmt.height,
		.interlaced = 0,
		.colorspace = 0,
	};
	struct avi_segment_format dstats_fmt;
	unsigned width, height;
	int ret = 0;

	/* Configuration has not changed */
	if (!force && !stats->updated_fmt)
		return 0;

	/* Apply new configuration */
	stats->updated_fmt = 0;

	/* Calculate thumbnail
	 * In order to get histogram in statistics, we need to store 256 bytes
	 * in last lines of captured frame, so thumbnail height is
	 * frame height - ((256 + line_width - 1) / line width).
	 */
	width = stats_fmt.width / 6;
	height = stats_fmt.height - ((255 + stats_fmt.width) / stats_fmt.width);

	/* Configure bayer stats */
	avi_statistics_bayer_configure(bayer_node, src_fmt->width,
				       src_fmt->height, width, height);

	/* Configure segment format */
	dstats_fmt = stats_fmt;
	dstats_fmt.pix_fmt = AVI_PIXFMT_BGRA8888;
	dstats_fmt.plane0.line_size = stats->v4l2_fmt.width * 4;

	/* Set format */
	ret = avi_segment_set_format(stats_seg, &stats_fmt,
						&dstats_fmt);
	if (ret) {
		dev_err(stats->vdev->parent, "Failed to set stats format!\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(avi_stats_apply_format);

static int avi_stats_queue_setup(struct vb2_queue *vq,
				 const struct v4l2_format *fmt,
				 unsigned int *count, unsigned int *num_planes,
				 unsigned int sizes[], void *alloc_ctxs[])
{
	struct avi_stats *stats = vb2_get_drv_priv(vq);

	sizes[0] = stats->v4l2_fmt.sizeimage;
	alloc_ctxs[0] = stats->alloc_ctx;
	*num_planes = 1;

	if (*count == 0)
		/* Default to 4 buffers */
		*count = 4;

	return 0;
}

static int avi_stats_vbuf_prepare(struct vb2_buffer* vb)
{
	struct avi_stats *stats = vb2_get_drv_priv(vb->vb2_queue);

	vb2_set_plane_payload(vb, 0, stats->v4l2_fmt.sizeimage);
	vb->v4l2_buf.field = V4L2_FIELD_NONE;

	return 0;
}

/* This function is called with stats->vbq_lock held (no need to protect
 * ourselves when we play with the dmaqueue) */
static void avi_stats_vbuf_queue(struct vb2_buffer *vb)
{
	struct avi_stats *stats = vb2_get_drv_priv(vb->vb2_queue);
	struct avi_vbuf *vbuf = to_avi_vbuf(vb);
	unsigned long flags;

	spin_lock_irqsave(&stats->vbq_lock, flags);

	list_add_tail(&vbuf->list, &stats->bufqueue);

	spin_unlock_irqrestore(&stats->vbq_lock, flags);
}

static int avi_stats_querycap(struct file *file, void *fh,
			      struct v4l2_capability *cap)
{
	struct avi_stats *stats = video_drvdata(file);

	mutex_lock(&stats->lock);

	strlcpy(cap->driver, stats->vdev->name, sizeof(cap->driver));

	snprintf(cap->card, sizeof(cap->card), "avi-stats.%d",
		 stats->vdev->parent->id);

	cap->version	  = stats->version;
	cap->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	mutex_unlock(&stats->lock);

	return 0;
}

static int avi_stats_enum_fmt_vid_cap(struct file *file, void *priv,
				      struct v4l2_fmtdesc *f)
{
	/* We only have one hardcoded format for the stats  */
	if (f->index > 0)
		return -EINVAL;

	f->flags       = 0;
	f->pixelformat = V4L2_PIX_FMT_BGR32;

	strlcpy(f->description, "bayer statistics", sizeof(f->description));

	return 0;
}

static int avi_stats_g_fmt_vid(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct avi_stats *stats = video_drvdata(file);

	/* Check type */
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Lock format access */
	mutex_lock(&stats->lock);

	/* Get V4L2 format */
	f->fmt.pix = stats->v4l2_fmt;

	/* Unlock format access */
	mutex_unlock(&stats->lock);

	return 0;
}

static int avi_stats_try_fmt_vid_unlocked(struct avi_stats *stats,
					  struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	/* Only V4L2_PIX_FMT_BGR32 is supported */
	if (pix->pixelformat != V4L2_PIX_FMT_RGB32 &&
	    pix->pixelformat != V4L2_PIX_FMT_BGR32)
		return -EINVAL;

	/* Check resolution */
	avi_limit_adjust_height(AVI_PIXFMT_BGRA8888, &pix->height);
	avi_limit_adjust_width(AVI_PIXFMT_BGRA8888, &pix->width);

	/* Update values */
	pix->priv = 0;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width * 4;
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

static int avi_stats_try_fmt_vid(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct avi_stats *stats = video_drvdata(file);
	int ret;

	/* Set format */
	mutex_lock(&stats->lock);
	ret = avi_stats_try_fmt_vid_unlocked(stats, f);
	mutex_unlock(&stats->lock);

	return 0;
}

static void avi_stats_adjust(struct avi_stats *stats, struct v4l2_rect *crop)
{
	struct v4l2_rect bounds = {
		.left = 0,
		.top = 0,
		.width = stats->v4l2_fmt.width,
		.height = stats->v4l2_fmt.height,
	};
	int32_t max_height;

	/* Align width to a multiple of 6 */
	crop->width -= crop->width % 6;

	/* Max resolution width is 64 * 6 pixels */
	if (crop->width > 384)
		crop->width = 384;

	/* Adjust crop rectangle into output frame */
	avi_v4l2_crop_adjust(crop, &bounds);

	/* Adjust height size:
	 *   max resolution is 48 + ((256 + width - 1) / width)
	 */
	max_height = 48 + ((255 + crop->width) / crop->width);
	if (crop->height > max_height)
		crop->height = max_height;
}

static int avi_stats_s_fmt_vid_unlocked(struct avi_stats *stats,
					struct v4l2_format *f)
{
	int ret;

	/* Try format before setting */
	ret = avi_stats_try_fmt_vid_unlocked(stats, f);
	if (ret < 0)
		return ret;

	/* Update format */
	stats->v4l2_fmt = f->fmt.pix;
	stats->crop.left = 0;
	stats->crop.top = 0;
	stats->crop.width = f->fmt.pix.width;
	stats->crop.height = f->fmt.pix.height;

	/* Adjust crop to best value */
	avi_stats_adjust(stats, &stats->crop);

	/* Update AVI segment format */
	avi_v4l2_to_segment_fmt(&stats->v4l2_fmt, &stats->crop, &stats->avi_fmt,
				&stats->plane0_size, &stats->plane1_size);

	return 0;
}

static int avi_stats_s_fmt_vid(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct avi_stats *stats = video_drvdata(file);
	int ret;

	/* Check type */
	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Already streaming */
	if (stats->streaming)
		return -EBUSY;

	/* Set format */
	mutex_lock(&stats->lock);
	ret = avi_stats_s_fmt_vid_unlocked(stats, f);
	stats->updated_fmt = 1;
	mutex_unlock(&stats->lock);

	return ret;
}

static int avi_stats_g_selection(struct file *file, void *priv,
				 struct v4l2_selection *s)
{
	struct avi_stats *stats = video_drvdata(file);
	int ret = 0;

	/* Check type */
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Lock format access */
	mutex_lock(&stats->lock);

	/* Get selection */
	switch(s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = stats->crop;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = stats->v4l2_fmt.width;
		s->r.height = stats->v4l2_fmt.height;
		break;
	default:
		ret = -EINVAL;
	}

	/* Unlock format access */
	mutex_unlock(&stats->lock);

	return ret;
}

static int avi_stats_s_selection(struct file *file, void *priv,
				 struct v4l2_selection *s)
{
	struct avi_stats *stats = video_drvdata(file);
	int ret = 0;

	/* Check type */
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Only CROP is supported */
	if (s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	/* Lock format access */
	mutex_lock(&stats->lock);

	/* Adjust crop */
	avi_stats_adjust(stats, &s->r);

	/* Update crop and compose rect */
	stats->crop = s->r;

	/* Update AVI segment format */
	avi_v4l2_to_segment_fmt(&stats->v4l2_fmt, &stats->crop, &stats->avi_fmt,
				&stats->plane0_size, &stats->plane1_size);

	/* Selection has been updated */
	stats->updated_fmt = 1;

	/* Unlock format access */
	mutex_unlock(&stats->lock);

	return ret;
}

static int avi_stats_streamon(struct vb2_queue* vq, unsigned int count)
{
	struct avi_stats *stats = vb2_get_drv_priv(vq);

	/* Already streaming */
	if (stats->streaming)
		return -EBUSY;

	/* Nothing else to do here, the complicated stuff is done in streamon
	 * of main capture device.
	 */
	stats->updated_fmt = 1;
	stats->streaming = 1;

	return 0;
}

static int avi_stats_streamoff(struct vb2_queue* vq)
{
	struct avi_stats *stats = vb2_get_drv_priv(vq);
	struct avi_vbuf *node;
	struct avi_vbuf *buf;
	unsigned long flags;

	/* Already stopped */
	if (!stats->streaming)
		return -EINVAL;

	/* Mark every buffer as DONE */
	spin_lock_irqsave(&stats->vbq_lock, flags);
	list_for_each_entry_safe(buf, node, &stats->bufqueue, list) {
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&stats->vbq_lock, flags);

	/* Stopped */
	stats->streaming = 0;

	return 0;
}

static struct vb2_ops avi_stats_vqueue_ops = {
	.queue_setup	 = avi_stats_queue_setup,
	.buf_prepare	 = avi_stats_vbuf_prepare,
	.buf_queue	 = avi_stats_vbuf_queue,
	.start_streaming = avi_stats_streamon,
	.stop_streaming  = avi_stats_streamoff,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
};

static const struct v4l2_ioctl_ops avi_stats_ioctl_ops = {
	.vidioc_querycap	 = avi_stats_querycap,
	.vidioc_enum_fmt_vid_cap = avi_stats_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	 = avi_stats_g_fmt_vid,
	.vidioc_try_fmt_vid_cap	 = avi_stats_try_fmt_vid,
	.vidioc_s_fmt_vid_cap	 = avi_stats_s_fmt_vid,
	.vidioc_g_selection	 = avi_stats_g_selection,
	.vidioc_s_selection	 = avi_stats_s_selection,
	.vidioc_reqbufs		 = vb2_ioctl_reqbufs,
	.vidioc_querybuf	 = vb2_ioctl_querybuf,
	.vidioc_qbuf		 = vb2_ioctl_qbuf,
	.vidioc_dqbuf		 = vb2_ioctl_dqbuf,
	.vidioc_streamon	 = vb2_ioctl_streamon,
	.vidioc_streamoff	 = vb2_ioctl_streamoff,
};

static int avi_stats_open(struct file *file)
{
	struct avi_stats *stats = video_drvdata(file);
	struct vb2_queue *q = &stats->vb_vidq;
	struct vb2_dc_conf *alloc_ctx = (struct vb2_dc_conf *)stats->alloc_ctx;

	mutex_lock(&stats->lock);

	if (stats->use_count != 0)
		goto done;

	stats->updated_fmt = 1;

	memset(q, 0, sizeof(*q));

	q->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes        = VB2_MMAP | VB2_USERPTR;
	q->mem_ops         = &vb2_dma_contig_memops;
	q->lock            = &stats->lock;
	q->ops             = &avi_stats_vqueue_ops;
	q->drv_priv        = stats;
	q->buf_struct_size = sizeof(struct avi_vbuf);
	q->cache_flags     = alloc_ctx->cache_flags;

	vb2_queue_init(q);

 done:
	stats->use_count++;

	mutex_unlock(&stats->lock);

	return 0;
}

static int avi_stats_release(struct file *file)
{
	struct avi_stats *stats= video_drvdata(file);
	int ret = 0;

	mutex_lock(&stats->lock);

	stats->use_count--;
	if (!stats->use_count)
		ret = vb2_fop_release(file);

	mutex_unlock(&stats->lock);

	return ret;
}

static struct v4l2_file_operations avi_stats_fops = {
	.owner		= THIS_MODULE,
	.open		= avi_stats_open,
	.release	= avi_stats_release,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.read           = vb2_fop_read,
};

int __devinit avi_stats_init(struct avi_stats *stats,
			     struct video_device *pvdev, u32 version)
{
	struct v4l2_format default_fmt = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width = AVI_STATS_DEFAULT_WIDTH,
			.height = AVI_STATS_DEFAULT_HEIGHT,
			.pixelformat = V4L2_PIX_FMT_BGR32,
		},
	};
	struct video_device *vdev;
	struct vb2_dc_conf *alloc_ctx;
	int ret;

	spin_lock_init(&stats->vbq_lock);
	INIT_LIST_HEAD(&stats->bufqueue);

	vdev = video_device_alloc();
	if (vdev == NULL) {
		ret = -ENODEV;
		goto vdev_alloc_failed;
	}

	stats->vdev = vdev;
	stats->version = version;

	mutex_init(&stats->lock);

	snprintf(vdev->name, sizeof(vdev->name), "%s-stats" , pvdev->name);

	vdev->parent	   = pvdev->parent;
	vdev->current_norm = V4L2_STD_UNKNOWN;;
	vdev->fops	   = &avi_stats_fops;
	vdev->ioctl_ops	   = &avi_stats_ioctl_ops;
	vdev->release	   = &video_device_release;
	vdev->vfl_type	   = VFL_TYPE_GRABBER;
	vdev->tvnorms	   = V4L2_STD_UNKNOWN;
	vdev->queue        = &stats->vb_vidq;
	/* We handle the locking ourselves */
	vdev->lock	   = NULL;

	video_set_drvdata(vdev, stats);

	vdev->v4l2_dev = pvdev->v4l2_dev;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto video_register_failed;

	/* Set default format to 64x48 thumbnail */
	avi_stats_s_fmt_vid_unlocked(stats, &default_fmt);
	stats->updated_fmt = 1;

	alloc_ctx = vb2_dma_contig_init_ctx(&vdev->dev);
	alloc_ctx->cache_flags = stats->stat_vb2_cache_flags;
	stats->alloc_ctx = (void *)alloc_ctx;

	dev_info(vdev->parent,
	         "stats video device successfuly registered as %s\n",
	         video_device_node_name(stats->vdev));
	return 0;

 video_register_failed:
	video_unregister_device(stats->vdev);
 vdev_alloc_failed:
	return ret;
}
EXPORT_SYMBOL(avi_stats_init);

void __devexit avi_stats_destroy(struct avi_stats *stats)
{
	vb2_dma_contig_cleanup_ctx(stats->alloc_ctx);
	video_unregister_device(stats->vdev);
}
EXPORT_SYMBOL(avi_stats_destroy);

MODULE_AUTHOR("Alexandre Dilly <alexandre.dilly@parrot.com>");
MODULE_DESCRIPTION("V4L2 stats capture interface");
MODULE_LICENSE("GPL");
