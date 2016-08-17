#include <linux/module.h>
#include <media/v4l2-ioctl.h>

#include "avicam_v4l2.h"

void avicam_stats_done(struct avi_capture_context	*ctx,
		       struct avi_dma_buffer		*frame,
		       enum avi_field			 f)
{
	struct avicam		*avicam = ctx->priv;
	struct avicam_vbuf	*vbuf	= frame->priv;
	enum vb2_buffer_state    state  = VB2_BUF_STATE_DONE;

	if (frame->status != AVI_BUFFER_DONE)
		dev_err(avicam->dev, "couldn't capture stats\n");

	v4l2_get_timestamp(&vbuf->vb.v4l2_buf.timestamp);
	vbuf->vb.v4l2_buf.sequence = avicam->frame_count >> 1;

	if (frame->status != AVI_BUFFER_DONE)
		state = VB2_BUF_STATE_ERROR;

	vb2_buffer_done(&vbuf->vb, state);
}

void avicam_stats_next(struct avi_capture_context	*ctx,
		       struct avi_dma_buffer		*frame,
		       enum avi_field			 f)
{
	struct avicam		*avicam	= ctx->priv;
	struct avicam_stats	*stats	= &avicam->stats;
	struct avicam_vbuf	*vbuf	= NULL;
	unsigned long		 flags;

	spin_lock_irqsave(&stats->vbq_lock, flags);

	if (stats->streaming && !list_empty(&stats->bufqueue)) {
		vbuf = list_first_entry(&stats->bufqueue,
					struct avicam_vbuf,
					list);

		list_del(&vbuf->list);

		vbuf->vb.state = VB2_BUF_STATE_ACTIVE;
	}

	spin_unlock_irqrestore(&stats->vbq_lock, flags);

	if (!vbuf)
		/* exhausted buffer queue */
		return;

	/* XXX add YUV offset */
	frame->plane0.dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0);
	frame->priv	       = vbuf;
}

static int avicam_stats_queue_setup(struct vb2_queue *vq,
                        const struct v4l2_format *fmt,
                        unsigned int *count, unsigned int *num_planes,
                        unsigned int sizes[], void *alloc_ctxs[])
{
	struct avicam	*avicam = vb2_get_drv_priv(vq);

	sizes[0] = AVI_CAPTURE_STATS_FRAMESIZE;
	alloc_ctxs[0] = avicam->stats.alloc_ctx;
	*num_planes = 1;

	if (*count == 0)
		/* Default to 4 buffers */
		*count = 4;

	return 0;
}

static int avicam_stats_vbuf_prepare(struct vb2_buffer* vb)
{
	vb2_set_plane_payload(vb, 0, AVI_CAPTURE_STATS_FRAMESIZE);
	vb->v4l2_buf.field = V4L2_FIELD_NONE;
	return 0;
}

/* This function is called with stats->vbq_lock held (no need to protect
 * ourselves when we play with the dmaqueue) */
static void avicam_stats_vbuf_queue(struct vb2_buffer *vb)
{
	struct avicam           *avicam = vb2_get_drv_priv(vb->vb2_queue);
	struct avicam_vbuf      *buf    = to_avicam_vbuf(vb);
	struct avicam_stats	*stats	= &avicam->stats;
	unsigned long            flags;

	spin_lock_irqsave(&stats->vbq_lock, flags);

	list_add_tail(&buf->list, &stats->bufqueue);

	spin_unlock_irqrestore(&stats->vbq_lock, flags);
}

static int avicam_stats_querycap(struct file		*file,
				 void			*fh,
				 struct v4l2_capability	*cap)
{
	struct avicam		*avicam	= video_drvdata(file);
	struct avicam_stats	*stats	= &avicam->stats;

	mutex_lock(&stats->lock);

	strlcpy(cap->driver, DRIVER_NAME,       sizeof(cap->driver));

	snprintf(cap->card, sizeof(cap->card), "avi-stats.%d",
		 avicam_instance(avicam));

	cap->version	  = DRIVER_VERSION;
	cap->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	mutex_unlock(&stats->lock);

	return 0;
}

static int avicam_stats_enum_fmt_vid_cap(struct file		*file,
					 void			*priv,
					 struct v4l2_fmtdesc	*f)
{
	/* We only have one hardcoded format for the stats  */
	if (f->index != 0)
		return -EINVAL;

	f->flags       = 0;
	f->pixelformat = V4L2_PIX_FMT_BGR32;

	strlcpy(f->description, "bayer statistics", sizeof(f->description));

	return 0;
}

static int avicam_stats_fill_fmt(struct file		*file,
				 void			*priv,
				 struct v4l2_format	*f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix->pixelformat = V4L2_PIX_FMT_BGR32;
	pix->colorspace  = 0;

	pix->width	  = AVI_CAPTURE_STATS_WIDTH;
	pix->height	  = AVI_CAPTURE_STATS_HEIGHT;
	pix->bytesperline = pix->width * 4;
	pix->sizeimage    = pix->bytesperline * pix->height;
	pix->field	  = V4L2_FIELD_NONE;

	return 0;
}

static int avicam_stats_streamon(struct vb2_queue* vq, unsigned int count)
{
	struct avicam		*avicam	= vb2_get_drv_priv(vq);
	struct avicam_stats	*stats	= &avicam->stats;

	if (stats->streaming)
		return -EBUSY;

	/* Nothing else to do here, the complicated stuff is done in avicam_streamon */
	stats->streaming = 1;

	return 0;
}

static int avicam_stats_do_streamoff(struct avicam *avicam)
{
	struct avicam_stats *stats = &avicam->stats;
	struct avicam_vbuf  *buf;
	struct avicam_vbuf  *node;
	unsigned long        flags;

	if (!stats->streaming)
		return -EINVAL;

	spin_lock_irqsave(&stats->vbq_lock, flags);
	list_for_each_entry_safe(buf, node, &stats->bufqueue, list) {
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&stats->vbq_lock, flags);

	stats->streaming = 0;

	return 0;
}


static int avicam_stats_streamoff(struct vb2_queue* vq)
{
	struct avicam		*avicam = vb2_get_drv_priv(vq);

	return avicam_stats_do_streamoff(avicam);
}

static struct vb2_ops avicam_stats_vqueue_ops = {
	.queue_setup	 = avicam_stats_queue_setup,
	.buf_prepare	 = avicam_stats_vbuf_prepare,
	.buf_queue	 = avicam_stats_vbuf_queue,
	.start_streaming = avicam_stats_streamon,
	.stop_streaming  = avicam_stats_streamoff,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
};

static const struct v4l2_ioctl_ops avicam_stats_ioctl_ops = {
	.vidioc_querycap	 = avicam_stats_querycap,
	.vidioc_enum_fmt_vid_cap = avicam_stats_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	 = avicam_stats_fill_fmt,
	.vidioc_try_fmt_vid_cap	 = avicam_stats_fill_fmt,
	.vidioc_s_fmt_vid_cap	 = avicam_stats_fill_fmt,
	.vidioc_reqbufs		 = vb2_ioctl_reqbufs,
	.vidioc_querybuf	 = vb2_ioctl_querybuf,
	.vidioc_qbuf		 = vb2_ioctl_qbuf,
	.vidioc_dqbuf		 = vb2_ioctl_dqbuf,
	.vidioc_streamon	 = vb2_ioctl_streamon,
	.vidioc_streamoff	 = vb2_ioctl_streamoff,
};

static int avicam_stats_open(struct file *file)
{
	struct avicam           *avicam = video_drvdata(file);
	struct avicam_stats     *stats  = &avicam->stats;
	struct vb2_queue        *q      = &stats->vb_vidq;

	mutex_lock(&stats->lock);

	if (stats->use_count != 0)
		goto done;

	memset(q, 0, sizeof(*q));

	q->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes        = VB2_MMAP | VB2_USERPTR;
	q->mem_ops         = &vb2_dma_contig_memops;
	q->lock            = &stats->lock;
	q->ops             = &avicam_stats_vqueue_ops;
	q->drv_priv        = avicam;
	q->buf_struct_size = sizeof(struct avicam_vbuf);

	vb2_queue_init(q);

 done:
	stats->use_count++;

	mutex_unlock(&stats->lock);

	return 0;
}

static int avicam_stats_release(struct file *file)
{
	struct avicam		*avicam = video_drvdata(file);
	struct avicam_stats     *stats  = &avicam->stats;
	int ret = 0;
	
	mutex_lock(&stats->lock);
	
	stats->use_count--;
	if (!stats->use_count)
		ret = vb2_fop_release(file);

	mutex_unlock(&stats->lock);
	return ret;
}

static struct v4l2_file_operations avicam_stats_fops = {
	.owner		= THIS_MODULE,
	.open		= avicam_stats_open,
	.release	= avicam_stats_release,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.read           = vb2_fop_read,
};

int __devinit avicam_stats_init(struct avicam *avicam)
{
	struct avicam_stats	*stats = &avicam->stats;
	struct video_device	*vdev;
	int			 ret;

	if (!avicam->pdata->enable_stats)
		/* Nothing to do */
		return 0;

	spin_lock_init(&stats->vbq_lock);
	INIT_LIST_HEAD(&stats->bufqueue);

	vdev = video_device_alloc();
	if (vdev == NULL) {
		ret = -ENODEV;
		goto vdev_alloc_failed;
	}

	stats->vdev = vdev;

	mutex_init(&stats->lock);

	strlcpy(vdev->name, DRIVER_NAME "-stats", sizeof(vdev->name));

	vdev->parent	   = avicam->dev;
	vdev->current_norm = V4L2_STD_UNKNOWN;;
	vdev->fops	   = &avicam_stats_fops;
	vdev->ioctl_ops	   = &avicam_stats_ioctl_ops;
	vdev->release	   = &video_device_release;
	vdev->vfl_type	   = VFL_TYPE_GRABBER;
	vdev->tvnorms	   = V4L2_STD_UNKNOWN;
	vdev->queue        = &stats->vb_vidq;
	/* We handle the locking ourselves */
	vdev->lock	   = NULL;

	video_set_drvdata(vdev, avicam);

	vdev->v4l2_dev = avicam->v4l2_dev;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto video_register_failed;

	stats->alloc_ctx = vb2_dma_contig_init_ctx(&vdev->dev);

	dev_info(avicam->dev,
	         "stats video device successfuly registered as %s\n",
	         video_device_node_name(stats->vdev));
	return 0;

 video_register_failed:
	video_unregister_device(stats->vdev);
 vdev_alloc_failed:
	return ret;
}

void __devexit avicam_stats_destroy(struct avicam *avicam)
{
	if (!avicam->pdata->enable_stats)
		/* Nothing to do */
		return;

	vb2_dma_contig_cleanup_ctx(avicam->stats.alloc_ctx);
	video_unregister_device(avicam->stats.vdev);
}
