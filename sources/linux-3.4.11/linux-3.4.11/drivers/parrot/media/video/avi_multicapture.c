/**
 * AVI Multicapture driver
 *
 * Its purpose is to create one big video (mosaic) made of multiple camera streams coming
 * into the P7 AVI.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <parrot/avicam_dummy_dev.h>
#include "avicam_v4l2.h"
#include "avi_multicapture.h"

#include <video/avi_pixfmt.h>
#include "avi_capture.h"

/**
 * XXX: don't hardcode this.
 * Multicapture driver should be able to handle different cameras with different
 * byte/pixel values
 */
#define CAM_BYTES_PER_PIXEL  2

/**
 * Videobuf2 parts
 */
struct avimulti_buffer {
	struct vb2_buffer vb;
	struct list_head  list;
	int               needs_unmap;
	unsigned          done_needed;
	unsigned          done_current;
	void              *metadata_addr;
};

/*
 * One instance per capturing camera
 */
struct avi_multicapt_context {
	unsigned                         i;
	struct avi_capture_context	 capture_ctx;
	struct avicam_platform_data*     avicam_pdata;

	/**
	 * V4L2 subdevice associated with this camera
	 */
	struct v4l2_subdev               *subdev;

	/**
	 * Whether the probe for this camera module was successful or not
	 */
	unsigned                         valid;

	/*
	 * Offset bytes in the final buffer where the frame for this camera will be stored
	 */
	unsigned			 offset_bytes;
	void				*priv_top;

	/*
	 * Whether the camera is in full framerate (and thus needs 2 frames per mosaic)
	 */
	unsigned                         alternate;

	/*
	 * These 3 variables control the state of the camera when capturing
	 */
	unsigned                         cur_alt;

	/*
	 * Whether this camera is active or not
	 * Unlike "streaming", this can become 0 during the multicapture
	 * if the camera goes offline or gets corrupted
	 */
	unsigned                         enabled;

	/*
	 * Whether we started an avi capture on this camera
	 */
	unsigned                         streaming;

	struct avimulti_buffer          *cur_buf;
};

struct avi_multicapt_top {
	struct device                    *dev;
	struct avimulti_platform_data    *pdata;
	spinlock_t                       vbq_lock;
	struct v4l2_device               *v4l2_dev;
	struct video_device              *vdev;
	struct list_head                 bufqueue;
	struct list_head                 bufqueue_active;
	struct list_head                 lookup_address;

	/* serialization lock */
	struct mutex                     lock;
	struct media_pad                 pad;
	int                              use_count;
	int                              streaming;

	struct avi_multicapt_context     multicapt_contexts[6];
	unsigned int                     frame_count;

	struct pinctrl                  *pctl;

	/**
	 * Videobuf2 parts
	 */
	void                            *alloc_ctx;
	struct vb2_queue                 vb_vidq;
};

struct avi_multicapt_lookup_address {
	struct list_head list;
	dma_addr_t       dma_addr;
	void             *metadata_addr;
};

static struct avimulti_buffer* to_avimulti_buffer(struct vb2_buffer* vb2)
{
	return container_of(vb2, struct avimulti_buffer, vb);
}

static int avimulticapture_instance(struct avi_multicapt_top *multicapt_top)
{
	struct platform_device *pdev;

	pdev = container_of(multicapt_top->dev, struct platform_device, dev);

	return pdev->id;
}

static inline int avimulti_instance(struct avi_multicapt_top *top)
{
        struct platform_device *pdev;

        pdev = container_of(top->dev, struct platform_device, dev);

        return pdev->id;
}

/**
 * Returns the mosaic image height
 */
static unsigned avimulti_image_height(struct avi_multicapt_top *top)
{
	return top->pdata->height * (top->pdata->nb_cameras+top->pdata->nb_full_framerate);
}

/**
 * Return the total mosaic size, in bytes
 */
static unsigned avimulti_image_size(struct avi_multicapt_top *top)
{
	return avimulti_image_height(top) * top->pdata->width * CAM_BYTES_PER_PIXEL;
}

/**
 * Number of metadata lines necessary (if enabled)
 */
static int avimulti_metadata_height(struct avi_multicapt_top *top) {
	struct avimulti_platform_data* pdata = top->pdata;

	if(!pdata->enable_metadata)
		return 0;

	return  (sizeof(struct avimulti_metadata) *
	        (pdata->nb_cameras + pdata->nb_full_framerate)) /
		(pdata->width * CAM_BYTES_PER_PIXEL) + 1;
}

/**
 * Size of metadata in bytes (if enabled)
 */
static int avimulti_metadata_size(struct avi_multicapt_top *top) {
	return top->pdata->width * avimulti_metadata_height(top) * CAM_BYTES_PER_PIXEL;
}

static unsigned avimulti_buffer_size(struct avi_multicapt_top *top)
{
	return avimulti_image_size(top) +
		avimulti_metadata_size(top);
}

/**
 * VB2 callback: setup queue plane number & payload size
 */
static int avimulti_queue_setup(struct vb2_queue *vq,
                        const struct v4l2_format *fmt,
                        unsigned int *count, unsigned int *num_planes,
                        unsigned int sizes[], void *alloc_ctxs[])
{
	struct avi_multicapt_top *multicapt_top = vb2_get_drv_priv(vq);
	unsigned int size = avimulti_image_size(multicapt_top) +
			    avimulti_metadata_size(multicapt_top);

	sizes[0] = size;
	alloc_ctxs[0] = multicapt_top->alloc_ctx;
	*num_planes = 1;
	if (*count == 0)
		/* Default to 4 buffers */
		*count = 4;

	return 0;
}

static void avimulti_get_meta_addr(struct vb2_buffer* vb)
{
	struct avi_multicapt_top *multicapt_top = vb2_get_drv_priv(vb->vb2_queue);
	struct avimulti_buffer *vbuf = to_avimulti_buffer(vb);
	struct list_head *pos;
	void * meta_addr = NULL;
	struct avi_multicapt_lookup_address *lookup;
	unsigned long flags = 0;

	if (multicapt_top->pdata->enable_metadata) {
		dma_addr_t dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0);

		spin_lock_irqsave(&multicapt_top->vbq_lock, flags);

		list_for_each(pos, &multicapt_top->lookup_address) {
			lookup = list_entry(pos, struct avi_multicapt_lookup_address, list);
			if (lookup->dma_addr == dma_addr) {
				meta_addr = lookup->metadata_addr;
				break;
			}
		}

		spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);

		if (meta_addr == NULL) {
			lookup = kzalloc(sizeof (struct avi_multicapt_lookup_address), GFP_KERNEL);

			if (!lookup) {
				dev_err(multicapt_top->dev,
					"alloc failed for lookup address structure\n");
			} else {
				meta_addr = ioremap(dma_addr+ avimulti_image_size(multicapt_top),
						    sizeof(struct avimulti_metadata)*5);
				lookup->dma_addr = dma_addr;
				lookup->metadata_addr = meta_addr;

				spin_lock_irqsave(&multicapt_top->vbq_lock, flags);
				list_add_tail(&lookup->list,  &multicapt_top->lookup_address);
				spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);
			}
		}
		vbuf->metadata_addr = meta_addr;
	}

	return;
}

static int avimulti_vbuf_prepare(struct vb2_buffer* vb)
{
	struct avi_multicapt_top *multicapt_top = vb2_get_drv_priv(vb->vb2_queue);
	unsigned int size = avimulti_buffer_size(multicapt_top);

	avimulti_get_meta_addr(vb);

	vb2_set_plane_payload(vb, 0, size);
	vb->v4l2_buf.field = V4L2_FIELD_NONE;
	return 0;
}

static void avimulti_vbuf_queue(struct vb2_buffer* vb)
{
	struct avi_multicapt_top *multicapt_top = vb2_get_drv_priv(vb->vb2_queue);
	struct avimulti_buffer *vbuf = to_avimulti_buffer(vb);
	unsigned long flags = 0;

	spin_lock_irqsave(&multicapt_top->vbq_lock, flags);
	list_add_tail(&vbuf->list, &multicapt_top->bufqueue);
	spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);
}

static int avimulti_fill_fmt(struct file	*file,
			     void		*priv,
			     struct v4l2_format	*f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct avi_multicapt_top *top = video_drvdata(file);

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->colorspace  = V4L2_COLORSPACE_SMPTE170M;

	pix->width	  = top->pdata->width;
	pix->height	  = avimulti_image_height(top) + avimulti_metadata_height(top);
	pix->bytesperline = top->pdata->width * CAM_BYTES_PER_PIXEL;
	pix->sizeimage    = pix->bytesperline * pix->height;
	pix->field	  = V4L2_FIELD_NONE;

	return 0;
}

/**
 * Tells whether the mosaic is ready (i.e all frames are captured)
 * for the current buffer
 */
static unsigned avimulti_ready(struct avimulti_buffer *buf)
{
	if (buf->done_current >= buf->done_needed)
		return 1;

	return 0;
}

static void avimulti_reset_counter_done(struct avimulti_buffer *buf)
{
	buf->done_needed = 0;
	buf->done_current = 0;
}

/**
 * This is called by the avi_capture layer when one of the camera finished
 * capturing a frame
 */
static void avimulti_done(struct avi_capture_context	*ctx,
                          struct avi_dma_buffer		*frame,
                          enum avi_field                f)
{
	struct avimulti_buffer           *vbuf	= frame->priv;
	struct avi_multicapt_context     *multicapt = ctx->priv;
	struct avi_multicapt_top         *multicapt_top = multicapt->priv_top;
	struct avimulti_platform_data    *pdata = multicapt_top->pdata;
	unsigned long                     flags = 0;
	u32                               was_enabled = multicapt->enabled;

	if (!multicapt_top->streaming)
		return;

	if (frame->status == AVI_BUFFER_ERROR ||
	    frame->status == AVI_BUFFER_TIMEOUT)
		/* Disable the camera in case it goes corrupt */
		multicapt->enabled = 0;
	else
		multicapt->enabled  = 1;

	++multicapt_top->frame_count;

	/* Write meta-data at the end of the mosaic */
	if (multicapt_top->pdata->enable_metadata) {
		struct avimulti_metadata metadata;
		unsigned int index =
			multicapt->offset_bytes / (pdata->width * pdata->height * CAM_BYTES_PER_PIXEL);
		struct avimulti_metadata *metadata_addr = (struct avimulti_metadata *) vbuf->metadata_addr;

		metadata.timestamp = ktime_get().tv64;
		metadata.enabled = multicapt->enabled;
		memcpy(&metadata_addr[index], &metadata, sizeof(metadata));
	}

	spin_lock_irqsave(&multicapt_top->vbq_lock, flags);

	/* This camera is corrupt and thus we shouldn't update the buffers */
	if (vbuf->done_needed == 0) {
		spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);
		return;
	}

	/* If the camera was corrupt before, we aren't waiting for it. So don't
	 * increment the counter */
	if (was_enabled)
		vbuf->done_current++;

	/* If this was the last frame needed for the full mosaic, mark the buffer as done
	   and wake up any existing process waiting on it */
	if (avimulti_ready(vbuf)) {
		while (!list_empty(&multicapt_top->bufqueue_active)) {
			struct avimulti_buffer *buf;
			buf = list_entry(multicapt_top->bufqueue_active.next, struct avimulti_buffer, list);
			list_del(&buf->list);

			buf->vb.v4l2_buf.sequence = multicapt_top->frame_count;
			v4l2_get_timestamp(&buf->vb.v4l2_buf.timestamp);

			vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE);
			avimulti_reset_counter_done(buf);

			if (buf == vbuf)
				break;
		}
	}

	spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);
}

static void avimulti_clean_metadata(struct avi_multicapt_top *multicapt_top,
                                    struct avimulti_buffer *vbuf)
{
	if (!multicapt_top->pdata->enable_metadata)
		return;

	memset(vbuf->metadata_addr, 0,
	       sizeof(struct avimulti_metadata) *
	       multicapt_top->pdata->nb_cameras);
}

/**
 * This is called by the avi_capture layer to configure
 * the next frame's buffer for a camera
 */
static void avimulti_next(struct avi_capture_context    *ctx,
                          struct avi_dma_buffer         *frame,
                          enum avi_field                f)
{
	struct avi_multicapt_context	*multicapt     = ctx->priv;
	struct avi_multicapt_top	*multicapt_top = multicapt->priv_top;
	struct avimulti_buffer		*vbuf = NULL;
	unsigned long			 flags = 0;

	spin_lock_irqsave(&multicapt_top->vbq_lock, flags);

	if (!multicapt_top->streaming) {
		spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);
		return;
	}

	/* Get the latest buffer that was required */
	if (!list_empty(&multicapt_top->bufqueue_active))
		vbuf = list_entry(multicapt_top->bufqueue_active.prev,
						struct avimulti_buffer,
						list);

	/* Get ahead with the next buffer if all the next() have been called for every
	   camera. */
	if (!vbuf || multicapt->cur_buf == vbuf) {
		if(!list_empty(&multicapt_top->bufqueue)) {
			vbuf = list_first_entry(&multicapt_top->bufqueue,
						struct avimulti_buffer,
						list);
			list_del(&vbuf->list);

			vbuf->vb.state = VB2_BUF_STATE_ACTIVE;

			list_add_tail(&vbuf->list,
			              &multicapt_top->bufqueue_active);

			avimulti_clean_metadata(multicapt_top, vbuf);
		} else {
			frame->priv = NULL;
			spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);
			return;
		}
	}

	multicapt->cur_buf = vbuf;

	/* We don't want corrupt cameras to be waited for */
	if (multicapt->enabled)
		vbuf->done_needed++;

	spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);

	/* Find out the correct offset for this frame in the current mosaic buffer
	   It depends on whether the current camera is in full framerate or not */
	if (multicapt->alternate && multicapt->cur_alt) {
		frame->plane0.dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0);
		frame->plane0.dma_addr += multicapt->offset_bytes;
		frame->plane0.dma_addr += (multicapt_top->pdata->width * multicapt_top->pdata->height * CAM_BYTES_PER_PIXEL);
	} else {
		frame->plane0.dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0);
		frame->plane0.dma_addr += multicapt->offset_bytes;
	}

	multicapt->cur_alt = !multicapt->cur_alt;
	frame->priv = vbuf;
}

static int avimulti_querycap(struct file		*file,
			     void			*fh,
			     struct v4l2_capability	*cap)
{
	struct avi_multicapt_top *multicapt_top = video_drvdata(file);

	mutex_lock(&multicapt_top->lock);

	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "avi-cam.%d",
		 avimulticapture_instance(multicapt_top));

	cap->version	  = DRIVER_VERSION;
	cap->device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	mutex_unlock(&multicapt_top->lock);

	return 0;
}

static int avimulti_enum_fmt_vid_cap(struct file	 *file,
				     void		 *priv,
				     struct v4l2_fmtdesc *f)
{
	return avi_v4l2_enum_fmt(f);
}

static int avimulti_streamon(struct vb2_queue* vq, unsigned int count)
{
	struct avi_multicapt_top	*multicapt_top = vb2_get_drv_priv(vq);
	struct avimulti_platform_data   *pdata = multicapt_top->pdata;
	int				 ret = 0;
	unsigned       			 i;

	struct avi_segment_format dma_fmt = {
		.width		  = pdata->width,
		.height		  = pdata->height,
		.interlaced	  = 0,
		.colorspace	  = AVI_BT601_CSPACE,
		.pix_fmt          = AVI_PIXFMT_YUYV,
		.plane0           = {
			.line_size = pdata->width * CAM_BYTES_PER_PIXEL,
		}
	};

	if (multicapt_top->streaming) {
		return -EBUSY;
	}

	multicapt_top->frame_count = 0;

	for (i = 0; i < pdata->nb_cameras; i++) {
		struct avi_multicapt_context	*ctx =
			&multicapt_top->multicapt_contexts[i];

		if(!ctx->valid) {
			goto capture_init_failed;
		}

		ctx->offset_bytes = (i+min(pdata->nb_full_framerate, i))
			* (pdata->width * pdata->height * CAM_BYTES_PER_PIXEL);

		ret = avi_capture_init(&ctx->capture_ctx,
				       multicapt_top->dev,
				       avimulti_instance(multicapt_top) + i,
				       ctx->avicam_pdata->cam_cap,
				       0);
		if (ret)
			goto capture_init_failed;

		if(i < pdata->nb_full_framerate)
			ctx->alternate              = 1;
		else
			ctx->alternate              = 0;

		ctx->i                              = i;
		ctx->cur_alt                        = 0;
		ctx->priv_top			    = multicapt_top;
		ctx->cur_buf                        = NULL;
		ctx->capture_ctx.priv               = ctx;
		ctx->capture_ctx.interface          = ctx->avicam_pdata->interface;
		ctx->capture_ctx.capture_cspace     = AVI_BT601_CSPACE;
		ctx->capture_ctx.interlaced	    = 0;
		ctx->capture_ctx.measure_to_timings = ctx->avicam_pdata->measure_to_timings;

		ctx->capture_ctx.next = &avimulti_next;
		ctx->capture_ctx.done = &avimulti_done;

		ctx->capture_ctx.timeout_jiffies = pdata->timeout;

		/* enable streaming on the V4L2 subdev driver */
		ret = v4l2_subdev_call(ctx->subdev, video, s_stream, 1);
		if (ret) {
			dev_err(multicapt_top->dev, "Failed to call s_stream for %d\n", i);
			goto s_stream_failed;
		}
#ifdef CONFIG_AVICAM_TIMINGS_AUTO
	{
		unsigned			 cam_width;
		unsigned			 cam_height;

		/* Setup the avi_capture layer */
		ret = avi_capture_prepare(&ctx->capture_ctx);
		if (ret) {
			dev_err(multicapt_top->dev, "failed to get sensor timings for %d\n", i);
			goto prepare_failed;
		}

		avi_capture_resolution(&ctx->capture_ctx, &cam_width, &cam_height);

		if (cam_width != pdata->width || cam_height != pdata->height) {
			dev_err(multicapt_top->dev,
				"Invalid sensor resolution for %d, "
				"expected %ux%u but measured %ux%u\n",
				i, pdata->width, pdata->height, cam_width, cam_height);
			ret = -EINVAL;
			goto bad_resolution;
		}
	}
#else
		ret = avi_capture_prepare_timings(&ctx->capture_ctx, pdata->width, pdata->height);
		if (ret)
			goto prepare_failed;

#endif
		ret = avi_capture_set_format(&ctx->capture_ctx, &dma_fmt);
		if (ret)
			goto set_format_failed;

		ctx->enabled = 1;

		ret = 0;
		continue;

	set_format_failed:
#ifdef CONFIG_AVICAM_TIMINGS_AUTO
	bad_resolution:
#endif
	prepare_failed:
		v4l2_subdev_call(ctx->subdev, video, s_stream, 0);
	s_stream_failed:
		avi_capture_destroy(&ctx->capture_ctx);
	capture_init_failed:
		ctx->enabled = 0;
		ctx->streaming = 0;
		dev_err(multicapt_top->dev, "Couldn't initialize camera %d", i);
	}

	/* Do all capture streamon in a dedicated loop to prevent crash */
	for (i = 0; i < pdata->nb_cameras; i++) {
		struct avi_multicapt_context	*ctx =
			&multicapt_top->multicapt_contexts[i];

		if (ctx->enabled ) {
			ret = avi_capture_streamon(&ctx->capture_ctx);
			if (ret) {
				v4l2_subdev_call(ctx->subdev, video, s_stream, 0);
				avi_capture_destroy(&ctx->capture_ctx);
				ctx->enabled = 0;
				dev_err(multicapt_top->dev, "Couldn't initialize camera %d", i);
			}
			else {
				/* Success */
				ctx->streaming = 1;
			}
		}
	}

	multicapt_top->streaming = 1;
	return 0;
}

static int avimulti_streamoff(struct vb2_queue* vq)
{
	struct avi_multicapt_top *multicapt_top = vb2_get_drv_priv(vq);
	int		          i;
	unsigned long             flags = 0;

	if (!multicapt_top->streaming) {
		return -EINVAL;
	}

	spin_lock_irqsave(&multicapt_top->vbq_lock, flags);
	multicapt_top->streaming = 0;

	while (!list_empty(&multicapt_top->bufqueue)) {
		struct avimulti_buffer *buf;
		buf = list_entry(multicapt_top->bufqueue.next, struct avimulti_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&multicapt_top->bufqueue_active)) {
		struct avimulti_buffer *buf;
		buf = list_entry(multicapt_top->bufqueue_active.next, struct avimulti_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);

	/*
	 * Note : multicapt_top->lookup_address list
	 * is released through avimulti_buf_cleanup call
	 */

	for (i = 0; i < multicapt_top->pdata->nb_cameras; i++) {
		struct avi_multicapt_context *ctx = &multicapt_top->multicapt_contexts[i];

		if (!ctx->streaming)
			continue;

		v4l2_subdev_call(ctx->subdev, video, s_stream, 0);
		avi_capture_streamoff(&ctx->capture_ctx);
		avi_capture_destroy (&ctx->capture_ctx);
	}

	return 0;
}

static void avimulti_buf_cleanup(struct vb2_buffer *vb)
{
	struct avi_multicapt_top *multicapt_top = vb2_get_drv_priv(vb->vb2_queue);
	struct avimulti_buffer *vbuf = to_avimulti_buffer(vb);
	struct list_head *pos;
	struct avi_multicapt_lookup_address *lookup = NULL;
	unsigned long flags = 0;

	dma_addr_t dma_addr = vb2_dma_contig_plane_dma_addr(&vbuf->vb, 0);

	spin_lock_irqsave(&multicapt_top->vbq_lock, flags);
	list_for_each(pos, &multicapt_top->lookup_address) {
		lookup = list_entry(pos, struct avi_multicapt_lookup_address, list);
		if (lookup->dma_addr == dma_addr) {
			list_del(&lookup->list);
			break;
		}
	}
	spin_unlock_irqrestore(&multicapt_top->vbq_lock, flags);

	if (lookup && lookup->dma_addr == dma_addr) {
		iounmap(lookup->metadata_addr);
		kfree(lookup);
	}
}

static struct vb2_ops avimulti_qops = {
	.queue_setup     = avimulti_queue_setup,
	.buf_prepare     = avimulti_vbuf_prepare,
	.buf_queue       = avimulti_vbuf_queue,
	.start_streaming = avimulti_streamon,
	.stop_streaming  = avimulti_streamoff,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
	.buf_cleanup     = avimulti_buf_cleanup,
};

static int avimulti_open(struct file *file)
{
	struct avi_multicapt_top *multicapt_top = video_drvdata(file);

	mutex_lock(&multicapt_top->lock);

	if (multicapt_top->use_count == 0) {
		struct vb2_queue* q = &multicapt_top->vb_vidq;

		memset(q, 0, sizeof(struct vb2_queue));
		q->type            = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		q->io_modes        = VB2_MMAP | VB2_USERPTR;
		q->mem_ops         = &vb2_dma_contig_memops;
		q->lock            = &multicapt_top->lock;
		q->ops             = &avimulti_qops;
		q->drv_priv        = multicapt_top;
		q->buf_struct_size = sizeof(struct avimulti_buffer);
		q->cache_flags     = multicapt_top->pdata->vb2_cache_flags;
		vb2_queue_init(q);
	}
	multicapt_top->use_count++;

	mutex_unlock(&multicapt_top->lock);

	return 0;
}

static int avimulti_release(struct file *file)
{
	struct avi_multicapt_top *top = video_drvdata(file);
	int			  ret = 0;

	mutex_lock(&top->lock);

	top->use_count--;

	if (top->use_count == 0) {
		ret = vb2_fop_release(file);
	}

	mutex_unlock(&top->lock);

	return ret;
}

static struct v4l2_file_operations avimulti_fops = {
	.owner		= THIS_MODULE,
	.open		= avimulti_open,
	.release	= avimulti_release,
	.mmap		= vb2_fop_mmap,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.read           = vb2_fop_read,
};


static const struct v4l2_ioctl_ops avimulti_ioctl_ops = {
	.vidioc_querycap	     = avimulti_querycap,
	.vidioc_enum_fmt_vid_cap     = avimulti_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	     = avimulti_fill_fmt,
	.vidioc_try_fmt_vid_cap	     = avimulti_fill_fmt,
	.vidioc_s_fmt_vid_cap	     = avimulti_fill_fmt,
	.vidioc_reqbufs		     = vb2_ioctl_reqbufs,
	.vidioc_querybuf	     = vb2_ioctl_querybuf,
	.vidioc_qbuf		     = vb2_ioctl_qbuf,
	.vidioc_dqbuf		     = vb2_ioctl_dqbuf,
	.vidioc_streamon	     = vb2_ioctl_streamon,
	.vidioc_streamoff	     = vb2_ioctl_streamoff,
};

/**
 * Register one V4L2 subdev controlling one of the mosaic's cameras
 */
static struct v4l2_subdev* __devinit avimulti_register_subdev(
				     struct avicam_platform_data* camera,
                                     struct v4l2_device *v4l2_dev)
{
	struct i2c_adapter	*adapter;
	struct avicam_subdevs   *devs = camera->subdevs;

	if (devs == NULL || devs->board_info == NULL) {
		return NULL;
	}

	adapter = i2c_get_adapter(devs->i2c_adapter_id);
	if (adapter == NULL) {
		return NULL;
	}

	return v4l2_i2c_new_subdev_board(v4l2_dev,
					 adapter,
					 devs->board_info,
					 NULL);
}

/**
 * Register all the V4L2 subdev modules controlling the mosaic's cameras
 */
static int __devinit avimulti_register_cameras(struct avi_multicapt_top *top,
                                               struct v4l2_device *v4l2_dev)
{
	struct avicam_platform_data *cameras = top->pdata->cam_interfaces;
	int i, ok = 0;

	for (i = 0; i < top->pdata->nb_cameras; ++i) {
		struct avi_multicapt_context *ctx =
			&top->multicapt_contexts[i];

		ctx->subdev = avimulti_register_subdev(&cameras[i], v4l2_dev);

		if(ctx->subdev != NULL) {
			ok = 1;
			ctx->valid = 1;
			ctx->avicam_pdata = &cameras[i];
		} else {
			ctx->valid = 0;
		}
	}

	if (!ok)
		return -ENODEV;

	return v4l2_device_register_subdev_nodes(v4l2_dev);
}

static int __devinit avimulti_v4l2_init(struct avi_multicapt_top *multicapt_top)
{
	struct video_device	*vdev;
	int			 ret;

	vdev = video_device_alloc();
	if (vdev == NULL) {
		ret = -ENODEV;
		goto vdev_alloc_failed;
	}

	multicapt_top->v4l2_dev = avi_v4l2_get_device();
	if (!multicapt_top->v4l2_dev) {
		ret = -ENODEV;
		goto no_v4l2_dev;
	}

	multicapt_top->vdev = vdev;

	multicapt_top->pad.flags  = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&multicapt_top->vdev->entity, 1, &multicapt_top->pad, 0);
	if (ret < 0) {
		dprintk(multicapt_top, "Error initializing media entity\n");
		goto media_init_failed;
	}

	mutex_init(&multicapt_top->lock);

	strlcpy(vdev->name, dev_name(multicapt_top->dev), sizeof(vdev->name));

	vdev->parent	   = multicapt_top->dev;
	vdev->current_norm = V4L2_STD_UNKNOWN;
	vdev->fops         = &avimulti_fops;
	vdev->ioctl_ops	   = &avimulti_ioctl_ops;
	vdev->release	   = &video_device_release;
	vdev->vfl_type	   = VFL_TYPE_GRABBER;
	vdev->tvnorms	   = V4L2_STD_UNKNOWN;
	vdev->queue        = &multicapt_top->vb_vidq;
	/* We handle the locking ourselves */
	vdev->lock	   = NULL;

	video_set_drvdata(vdev, multicapt_top);

	vdev->v4l2_dev = multicapt_top->v4l2_dev;
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto video_register_failed;

	ret = avimulti_register_cameras(multicapt_top, multicapt_top->v4l2_dev);

	if (ret < 0) {
		v4l2_err(vdev, "Couldn't register at least one camera subdev\n");
		goto camera_register_failed;
	}

	return 0;

 camera_register_failed:
	video_unregister_device(multicapt_top->vdev);
 video_register_failed:
	media_entity_cleanup(&multicapt_top->vdev->entity);
 media_init_failed:
	avi_v4l2_put_device(multicapt_top->v4l2_dev);
 no_v4l2_dev:
	video_device_release(multicapt_top->vdev);
 vdev_alloc_failed:
	return ret;
}

static int avimulti_resume(struct device *dev)
{
	struct avicam	*avicam = dev_get_drvdata(dev);

	if (!avicam)
		return -ENODEV;

	if (!avicam->streaming)
		return 0;

	return avi_capture_resume(&avicam->capture_ctx);
}

static struct dev_pm_ops avimulti_dev_pm_ops = {
       .resume  = &avimulti_resume,
};

static int __devinit avimulti_probe(struct platform_device *pdev)
{
	int				 ret;
	struct avi_multicapt_top	*multicapt_top;
	struct vb2_dc_conf              *alloc_ctx;

	multicapt_top = kzalloc(sizeof(*multicapt_top), GFP_KERNEL);
	if (multicapt_top == NULL) {
	  	ret = -ENOMEM;
	  	goto alloc_failed;
	}

	multicapt_top->dev = &pdev->dev;

	multicapt_top->pdata =  dev_get_platdata(&pdev->dev);
	if (!multicapt_top->pdata) {
		ret = -EINVAL;
		goto no_pdata;
	}

	multicapt_top->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(multicapt_top->pctl)) {
		ret = PTR_ERR(multicapt_top->pctl);
		goto pinctrl_failed;
	}

	spin_lock_init(&multicapt_top->vbq_lock);
	INIT_LIST_HEAD(&multicapt_top->bufqueue);
	INIT_LIST_HEAD(&multicapt_top->bufqueue_active);
	INIT_LIST_HEAD(&multicapt_top->lookup_address);

	ret = avimulti_v4l2_init(multicapt_top);
	if (ret)
		goto v4l2_init_failed;


	platform_set_drvdata(pdev, multicapt_top);
	dev_set_drvdata(&pdev->dev, multicapt_top);

	dev_info(multicapt_top->dev,
	         "video device successfuly registered as %s\n",
	         video_device_node_name(multicapt_top->vdev));

	alloc_ctx = (struct vb2_dc_conf *)vb2_dma_contig_init_ctx(&pdev->dev);

	multicapt_top->alloc_ctx = alloc_ctx;
	return 0;

 v4l2_init_failed:
	pinctrl_put(multicapt_top->pctl);
 pinctrl_failed:
 no_pdata:
	kfree(multicapt_top);
 alloc_failed:

	return ret;
}

static void avimulti_unregister_camera(struct avi_multicapt_top	*top)
{
	int i;

	for (i = 0; i < top->pdata->nb_cameras; ++i) {
		struct avi_multicapt_context *ctx =
			&top->multicapt_contexts[i];

		if (ctx->subdev) {
			struct i2c_client *client = v4l2_get_subdevdata(ctx->subdev);
			if(client)
				i2c_unregister_device(client);
		}
	}
}

static void __devexit avicam_v4l2_destroy(struct avi_multicapt_top	*multicapt_top)
{
	avimulti_unregister_camera(multicapt_top);
	video_unregister_device(multicapt_top->vdev);
	media_entity_cleanup(&multicapt_top->vdev->entity);
	avi_v4l2_put_device(multicapt_top->v4l2_dev);
}

static int __devexit avimulti_remove(struct platform_device *pdev)
{
	struct avi_multicapt_top *multicapt_top = platform_get_drvdata(pdev);

	if (!multicapt_top)
		return -ENODEV;

	avicam_v4l2_destroy(multicapt_top);
	pinctrl_put(multicapt_top->pctl);
	vb2_dma_contig_cleanup_ctx(multicapt_top->alloc_ctx);
	kfree(multicapt_top);

	dev_set_drvdata(&pdev->dev, NULL);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver avimulticapt_driver = {
	.driver		= {
		.name	= "avi_multicapture",
		.owner	= THIS_MODULE,
		.pm     = &avimulti_dev_pm_ops,
	},
	.probe		= avimulti_probe,
	.remove		= __devexit_p(avimulti_remove),
};
static int __init avimulti_init(void)
{
	return platform_driver_register(&avimulticapt_driver);
}
module_init(avimulti_init);

static void __exit avimulti_exit(void)
{
	platform_driver_unregister(&avimulticapt_driver);
}
module_exit(avimulti_exit);

MODULE_AUTHOR("Misha Nesaratnam <misha.nesaratnam@parrot.com>");
MODULE_DESCRIPTION("P7 AVI multicapture mosaic driver");
MODULE_LICENSE("GPL");
