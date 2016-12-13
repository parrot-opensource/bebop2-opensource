/*
 *  linux/drivers/parrot/video/media/avi_m2m.c
 *
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author  Alexandre Dilly <alexandre.dilly@parrot.com>
 * @date  03-Aug-2015
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
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <linux/platform_device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#include "gpu/p7ump.h"

#include "avi_v4l2.h"
#include "avi_stats.h"
#include "avi_m2m.h"

#ifdef CONFIG_M2M_AVI_USE_ISP
#include "avi_v4l2_isp.h"
#endif

#define DRIVER_NAME "avi_m2m"
#define DRIVER_VERSION KERNEL_VERSION(0, 2, 0)

struct avi_m2m_format {
	struct v4l2_pix_format		 v4l2_fmt;
	struct v4l2_rect		 bounds;
	struct v4l2_rect		 rect;
	struct avi_segment_format	 avi_fmt;
	unsigned			 plane0_size;
	unsigned			 plane1_size;
};

struct avi_m2m_dev {
	struct device			*dev;
	int				 id;
	struct mutex			 mutex;
	spinlock_t			 lock;
	atomic_t			 num_inst;
	atomic_t			 activated;
	unsigned long			 streaming;
	unsigned long			 sequence;
	/* V4L2 device */
	struct v4l2_device 		*v4l2_dev;
	struct video_device		*vdev;
	/* Videobuf2 */
	struct vb2_alloc_ctx		*alloc_ctx;
	/* MEM2MEM device */
	struct v4l2_m2m_dev		*m2m_dev;
	struct v4l2_m2m_ctx		*m2m_ctx;
	/* Input/output formats */
	struct avi_m2m_format		 src_format;
	struct avi_m2m_format		 dst_format;
	unsigned long			 configuration_changed;
	/* Composition */
	struct v4l2_rect		 compose_rect;
	int				 compose_alpha;
	bool				 use_compose;
	/* Control handler and ISP chain controls */
#ifdef CONFIG_M2M_AVI_USE_ISP
	struct v4l2_ctrl_handler	 ctrl_handler;
	struct avi_v4l2_isp		*ctrl_isp;
#endif
	/* Stats */
	struct avi_stats		 stats;
	bool				 stats_enabled;
	bool				 use_stats;
	/* Caps */
	unsigned long			 caps;
	unsigned long			 current_caps;
	/* Segments */
	struct avi_segment		*in_seg;
	struct avi_segment		*out_seg;
	struct avi_segment		*stats_seg;
	/* Timer */
	struct timer_list		timer;
	/* Nodes */
	struct avi_node			*in_fifo_node;
	struct avi_node			*in_fifo_planar_node;
	struct avi_node			*out_fifo_node;
	struct avi_node			*stats_fifo_node;
	struct avi_node			*isp_node;
	/* Buffers */
	struct avi_dma_buffer		 src_buf;
	struct avi_dma_buffer		 dst_buf;
	struct avi_dma_buffer		 stats_buf;
};

/* Default values */
#define AVI_M2M_DEFAULT_PIX_FMT		V4L2_PIX_FMT_NV12
#define AVI_M2M_DEFAULT_COLORSPACE	V4L2_COLORSPACE_REC709
#define AVI_M2M_DEFAULT_WIDTH		2048
#define AVI_M2M_DEFAULT_HEIGHT		2048
#define AVI_M2M_DEFAULT_ALPHA		0xFF

static struct avi_m2m_format avi_m2m_default_format = {
	.v4l2_fmt = {
		.width	      = AVI_M2M_DEFAULT_WIDTH,
		.height	      = AVI_M2M_DEFAULT_HEIGHT,
		.pixelformat  = AVI_M2M_DEFAULT_PIX_FMT,
		.field	      = V4L2_FIELD_NONE,
		.bytesperline = 0,
		.sizeimage    = 0,
		.colorspace   = AVI_M2M_DEFAULT_COLORSPACE,
		.priv	      = 0,
	},
	.rect = {
		.top    = 0,
		.left   = 0,
		.width  = AVI_M2M_DEFAULT_WIDTH,
		.height = AVI_M2M_DEFAULT_HEIGHT,
	},
	.bounds = {
		.top    = 0,
		.left   = 0,
		.width  = AVI_M2M_DEFAULT_WIDTH,
		.height = AVI_M2M_DEFAULT_HEIGHT,
	},
};

/* Old R2R driver compatibily */
static int z_order = 1;
module_param(z_order, int, S_IRUGO);
MODULE_PARM_DESC(z_order, "Z order of plane managed by this driver: default 1");

/* Helpers */
#define AVI_M2M_CHECK_TYPE(_type) (_type == V4L2_BUF_TYPE_VIDEO_OUTPUT\
				   || _type == V4L2_BUF_TYPE_VIDEO_CAPTURE)

static inline bool avi_m2m_is_streaming(struct avi_m2m_dev *avi_m2m,
					enum v4l2_buf_type type)
{
	return !!(test_bit(type, &avi_m2m->streaming));
}

static inline void avi_m2m_set_streaming(struct avi_m2m_dev *avi_m2m,
					 enum v4l2_buf_type type,
					 bool stream_on)
{
	if (stream_on)
		set_bit(type, &avi_m2m->streaming);
	else
		clear_bit(type, &avi_m2m->streaming);
}

/*
 * AVI configuration and helpers
 */
static unsigned avi_m2m_aux_interrupt(struct avi_segment *segment,
				      enum avi_field *field)
{
	/* To remove? */
	return 0;
}

static unsigned avi_m2m_interrupt(struct avi_segment *segment,
				  enum avi_field *field)
{
	struct avi_m2m_dev *avi_m2m = segment->priv;
	struct vb2_buffer *src_vb, *dst_vb;
	unsigned long flags;

	/* Lock segment access */
	spin_lock_irqsave(&avi_m2m->lock, flags);

	/* Check if segment is activated */
	if (!atomic_read(&avi_m2m->activated))
		goto unlock;
	atomic_set(&avi_m2m->activated, 0);

	/* Reset timer */
	del_timer(&avi_m2m->timer);

	/* Update output FIFO status */
	avi_fifo_get_status(avi_m2m->out_fifo_node);

	/* Ack node IRQ for multi planar mode */
	avi_fifo_get_status(avi_m2m->in_fifo_node);
	if (avi_node_get_irq_flag(avi_m2m->in_fifo_node))
		avi_ack_node_irq(avi_m2m->in_fifo_node);

	/* Ack node IRQ for multi planar mode */
	if (avi_m2m->in_fifo_planar_node) {
		avi_fifo_get_status(avi_m2m->in_fifo_planar_node);
		if (avi_node_get_irq_flag(avi_m2m->in_fifo_planar_node))
			avi_ack_node_irq(avi_m2m->in_fifo_planar_node);
	}

	/* Check stats buffer output */
	if (avi_m2m->stats_seg) {
		if (avi_node_get_irq_flag(avi_m2m->stats_fifo_node))
			avi_m2m->stats_buf.status = AVI_BUFFER_DONE;
		else
			/* Looks like the stats are not ready... */
			avi_m2m->stats_buf.status = AVI_BUFFER_ERROR;

		/* Deactivate segment since IRQ for this segment is disabled and
		 * it is activated in oneshot mode.
		 */
		avi_segment_deactivate(avi_m2m->stats_seg);

		/* Make sure we never leave the IT flag raised */
		avi_ack_node_irq(avi_m2m->stats_fifo_node);
	}

	/* Mark buffer has done */
	avi_m2m->src_buf.status = AVI_BUFFER_DONE;
	avi_m2m->dst_buf.status = AVI_BUFFER_DONE;

	/* Get current buffers from MEM2MEM context */
	src_vb = v4l2_m2m_src_buf_remove(avi_m2m->m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(avi_m2m->m2m_ctx);

	/* Frame is too late and it has already been processed by timeout
	 * interrupt. In this case, the timeout should be increased.
	 */
	if (!src_vb || !dst_vb) {
		dev_warn(avi_m2m->dev, "Frame is too late...\n");
		goto unlock;
	}

	/* Release buffers */
	v4l2_m2m_buf_done(src_vb, avi_m2m->src_buf.status == AVI_BUFFER_DONE ?
							   VB2_BUF_STATE_DONE :
							   VB2_BUF_STATE_ERROR);
	v4l2_m2m_buf_done(dst_vb, avi_m2m->dst_buf.status == AVI_BUFFER_DONE ?
							   VB2_BUF_STATE_DONE :
							   VB2_BUF_STATE_ERROR);

	/* Return stats buffer to V4L2
	 * When buffer status is AVI_BUFFER_ERROR, it means that bayer
	 * statistics segment has not raised an interrupt which can be due to
	 * that module is not used or interrupt is raised later. In this case,
	 * we requeue buffer internaly instead of returning to videobuf2 (see
	 * avi_stats code).
	 */
	if (avi_m2m->stats_enabled && avi_m2m->stats_buf.plane0.dma_addr)
		avi_stats_done(&avi_m2m->stats, &avi_m2m->stats_buf,
			       src_vb->v4l2_buf.sequence, 1);

	/* Update V4L2 buffer fields */
	if (avi_m2m->src_buf.status == AVI_BUFFER_DONE &&
	    avi_m2m->dst_buf.status == AVI_BUFFER_DONE) {
		/* Update timestamp */
		v4l2_get_timestamp(&dst_vb->v4l2_buf.timestamp);

		/* Copy timecode when provided */
		if (src_vb->v4l2_buf.flags & V4L2_BUF_FLAG_TIMECODE) {
			dst_vb->v4l2_buf.flags |= V4L2_BUF_FLAG_TIMECODE;
			dst_vb->v4l2_buf.timecode = src_vb->v4l2_buf.timecode;
		}

		/* Copy sequence number */
		dst_vb->v4l2_buf.sequence =  src_vb->v4l2_buf.sequence;
	}

	/* Job is finished */
	v4l2_m2m_job_finish(avi_m2m->m2m_dev, avi_m2m->m2m_ctx);

unlock:
	/* Unlock segment access */
	spin_unlock_irqrestore(&avi_m2m->lock, flags);

	return 0;
}

static void avi_m2m_timeout(unsigned long data)
{
	struct avi_m2m_dev *avi_m2m = (struct avi_m2m_dev *) data;
	struct vb2_buffer *src_vb, *dst_vb;
	unsigned long flags;

	/* Check if AVI is streaming */
	if (!avi_m2m_is_streaming(avi_m2m, V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return;

	/* Lock segment access */
	spin_lock_irqsave(&avi_m2m->lock, flags);

	/* Check if segment is activated */
	if (!atomic_read(&avi_m2m->activated))
		goto unlock;
	atomic_set(&avi_m2m->activated, 0);

	/* Warn a frame has been dropped */
	dev_warn(avi_m2m->dev, "Timeout, dropping frame...\n");

	/* Clean the stats chain and deactivate its segment */
	if (avi_m2m->stats_seg)  {
		/*avi_fifo_force_clear*/
		avi_segment_deactivate(avi_m2m->stats_seg);
		avi_fifo_force_clear(avi_m2m->stats_fifo_node);
		avi_apply_node(avi_m2m->stats_fifo_node);
	}

	/* Deactivate segments */
	avi_segment_deactivate(avi_m2m->out_seg);
	avi_segment_deactivate(avi_m2m->in_seg);

	/* Clean the whole chain */
	avi_fifo_force_clear(avi_m2m->in_fifo_node);
	avi_fifo_force_clear(avi_m2m->out_fifo_node);
	avi_apply_node(avi_m2m->out_fifo_node);
	avi_apply_node(avi_m2m->in_fifo_node);

	/* Get current buffers from MEM2MEM context */
	src_vb = v4l2_m2m_src_buf_remove(avi_m2m->m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(avi_m2m->m2m_ctx);

	/* Check buffers */
	if (!src_vb || !dst_vb) {
		dev_err(avi_m2m->dev, "No V4L2 buffer available!\n");
		goto unlock;
	}

	/* Release buffers with error status */
	v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_ERROR);
	v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_ERROR);

	/* Release stats buffer with error status */
	if (avi_m2m->stats_enabled && avi_m2m->stats_buf.plane0.dma_addr) {
		avi_m2m->stats_buf.status = AVI_BUFFER_ERROR;
		avi_stats_done(&avi_m2m->stats, &avi_m2m->stats_buf,
			       src_vb->v4l2_buf.sequence, 1);
	}

	/* Job is finished */
	v4l2_m2m_job_finish(avi_m2m->m2m_dev, avi_m2m->m2m_ctx);

unlock:
	/* Unlock segment access */
	spin_unlock_irqrestore(&avi_m2m->lock, flags);
}

static int avi_m2m_build_segments(struct avi_m2m_dev *avi_m2m)
{
	struct avi_dma_pixfmt src_pixfmt;
	enum avi_colorspace src_cspace;
	enum avi_colorspace dst_cspace;
	unsigned long requirements;
	unsigned long caps;
	int ret = 0;

	/* Convert V4L2 formats to AVI formats */
	src_pixfmt = avi_v4l2_to_avi_pixfmt(
				      avi_m2m->src_format.v4l2_fmt.pixelformat);
	src_cspace = avi_v4l2_to_avi_cspace(
				       avi_m2m->src_format.v4l2_fmt.colorspace);
	dst_cspace = avi_v4l2_to_avi_cspace(
				       avi_m2m->dst_format.v4l2_fmt.colorspace);

	/* Determine needed caps */
	if (src_pixfmt.raw) {
		/* We need an ISP */
		requirements = AVI_CAPS_ISP;

		/* Stats can be used */
		avi_m2m->use_stats = 1;
	} else {
		/* We need a scaler */
		requirements = AVI_CAP_SCAL;

		/* We need a colorspace converter */
		if (src_cspace != dst_cspace)
			requirements |= AVI_CAP_CONV;

		/* We use planar */
		if (avi_pixfmt_is_planar(src_pixfmt))
			requirements |= AVI_CAP_PLANAR;

		/* Stats are not available */
		avi_m2m->use_stats = 0;
	}

	/* Check caps availability */
	if ((avi_m2m->caps & requirements) != requirements) {
		dev_err(avi_m2m->dev, "Capabilities are not available!\n");
		return -EINVAL;
	}

	/* Build output segment (needs only DMA_OUT) */
	caps = AVI_CAP_DMA_OUT;
	avi_m2m->out_seg = avi_segment_build(&caps, "m2m", avi_m2m->id, -1,
					     avi_m2m->dev);
	if (IS_ERR(avi_m2m->out_seg)) {
		ret = PTR_ERR(avi_m2m->out_seg);
		goto out_seg_failed;
	}

	/* Build input segment (all with DMA_IN) */
	caps = requirements | AVI_CAP_DMA_IN;
	avi_m2m->in_seg = avi_segment_build(&caps, "m2m-in", avi_m2m->id, -1,
					    avi_m2m->dev);
	if (IS_ERR(avi_m2m->in_seg)) {
		ret = PTR_ERR(avi_m2m->in_seg);
		goto in_seg_failed;
	}

	/* Set current caps used */
	avi_m2m->current_caps = requirements | AVI_CAP_DMA_IN | AVI_CAP_DMA_OUT;

	/* Build a segment for stats if ISP is used */
	if (avi_m2m->stats_enabled &&
	    (requirements & AVI_CAP_ISP_CHAIN_BAYER)) {
		/* Check if bayer stats are available in input segment */
		caps = avi_isp_bayer_get_stats_cap(avi_m2m->in_seg);
		if (!caps) {
			dev_err(avi_m2m->dev, "can't find stats cap");
			goto bayer_caps_failed;
		}

		/* Build stats segment */
		caps |= AVI_CAP_DMA_OUT;
		avi_m2m->stats_seg = avi_segment_build(&caps, "m2m-stats",
						       avi_m2m->id, -1,
						       avi_m2m->dev);
		if (IS_ERR(avi_m2m->stats_seg)) {
			ret = PTR_ERR(avi_m2m->stats_seg);
			goto stats_seg_failed;
		}

		/* Get FIFO node of stats segment */
		avi_m2m->stats_fifo_node = avi_segment_get_node(
							     avi_m2m->stats_seg,
							     AVI_CAP_DMA_OUT);
		if (!avi_m2m->stats_fifo_node) {
			dev_err(avi_m2m->dev,
				"Failed to get stats FIFO node!\n");
			goto stats_node_failed;
		}

		/* Get BAYER ISP node */
		avi_m2m->isp_node = avi_segment_get_node(avi_m2m->in_seg,
						       AVI_CAP_ISP_CHAIN_BAYER);
		if (!avi_m2m->isp_node) {
			dev_err(avi_m2m->dev, "Failed to get bayer node!\n");
			goto isp_node_failed;
		}

	} else {
		/* No stats needed */
		avi_m2m->stats_seg = NULL;
	}

	/* Connect segments */
	ret = avi_segment_connect(avi_m2m->in_seg, avi_m2m->out_seg, z_order);
	if (ret) {
		dev_err(avi_m2m->dev, "Cannot connect segments %s to %s",
			avi_m2m->in_seg->id, avi_m2m->out_seg->id);
		goto connect_failed;
	}

	/* Get FIFO node of input segment */
	avi_m2m->in_fifo_node = avi_segment_get_node(avi_m2m->in_seg,
						     AVI_CAP_DMA_IN);
	if (!avi_m2m->in_fifo_node) {
		dev_err(avi_m2m->dev, "Failed to get input FIFO node!\n");
		goto in_node_failed;
	}

	/* Get FIFO node of output segment */
	avi_m2m->out_fifo_node = avi_segment_get_node(avi_m2m->out_seg,
						      AVI_CAP_DMA_OUT);
	if (!avi_m2m->out_fifo_node) {
		dev_err(avi_m2m->dev, "Failed to get output FIFO node!\n");
		goto out_node_failed;
	}

	/* Get FIFO node of planar source  */
	if (requirements & AVI_CAP_PLANAR) {
		avi_m2m->in_fifo_planar_node = avi_segment_get_node(
								avi_m2m->in_seg,
								AVI_CAP_PLANAR);
		if (!avi_m2m->in_fifo_planar_node) {
			dev_err(avi_m2m->dev,
				"Failed to get planar FIFO node!\n");
			goto planar_node_failed;
		}
	}

	/* Set private in segments for IRQ */
	avi_m2m->in_seg->priv = avi_m2m;
	avi_m2m->out_seg->priv = avi_m2m;

	/* Request segments' IRQ */
	avi_segment_register_irq(avi_m2m->in_seg, &avi_m2m_interrupt);
	avi_segment_register_irq(avi_m2m->out_seg, &avi_m2m_aux_interrupt);

	/* Create timer for timeout handling */
	init_timer(&avi_m2m->timer);
	setup_timer(&avi_m2m->timer, avi_m2m_timeout, (long unsigned) avi_m2m);

	/* Enable segment IRQ (DMA_OUT) */
	avi_segment_enable_irq(avi_m2m->out_seg);

	return 0;

planar_node_failed:
out_node_failed:
in_node_failed:
	avi_segment_disconnect(avi_m2m->in_seg, avi_m2m->out_seg);
connect_failed:
isp_node_failed:
stats_node_failed:
stats_seg_failed:
	if (avi_m2m->stats_seg)
		avi_segment_teardown(avi_m2m->stats_seg);
	avi_segment_teardown(avi_m2m->in_seg);
bayer_caps_failed:
in_seg_failed:
	avi_segment_teardown(avi_m2m->out_seg);
out_seg_failed:
	return ret;
}

static int avi_m2m_activate_oneshot(struct avi_m2m_dev *avi_m2m)
{
	int ret;

	/* Setup buffers */
	avi_segment_set_input_buffer(avi_m2m->in_seg, &avi_m2m->src_buf);
	avi_segment_set_output_buffer(avi_m2m->out_seg, &avi_m2m->dst_buf);

	/* Setup stats buffer and activate its segment */
	if (avi_m2m->stats_seg && avi_m2m->stats_buf.plane0.dma_addr) {
		/* Setup stats buffer */
		avi_segment_set_output_buffer(avi_m2m->stats_seg,
					      &avi_m2m->stats_buf);

		/* Activate stats segment */
		ret = avi_segment_activate_oneshot(avi_m2m->stats_seg);
		if (ret) {
			dev_err(avi_m2m->dev,
				"Failed to activate stats segment!\n");
			goto activate_stats_failed;
		}
	}

	/* Activate input segment */
	ret = avi_segment_activate_oneshot(avi_m2m->in_seg);
	if (ret) {
		dev_err(avi_m2m->dev, "Failed to activate input segment!\n");
		goto activate_input_failed;
	}

	/* Activate output segment */
	ret = avi_segment_activate_oneshot(avi_m2m->out_seg);
	if (ret) {
		dev_err(avi_m2m->dev, "Failed to activate output segment!\n");
		goto activate_output_failed;
	}

	/* Segment are activatded */
	atomic_set(&avi_m2m->activated, 1);

	/* Setup timer for capture timeout (1s) */
	mod_timer(&avi_m2m->timer, jiffies + HZ);

	return 0;

activate_output_failed:
	avi_segment_deactivate(avi_m2m->in_seg);
activate_input_failed:
	if (avi_m2m->stats_seg && avi_m2m->stats_buf.plane0.dma_addr)
		avi_segment_deactivate(avi_m2m->stats_seg);
activate_stats_failed:
	return ret;
}

static int avi_m2m_deactivate_segments(struct avi_m2m_dev *avi_m2m)
{
	/* Lock segment access */
	spin_lock(&avi_m2m->lock);

	/* Segment are deactivated */
	atomic_set(&avi_m2m->activated, 0);

	/* Destroy stats */
	if (avi_m2m->stats_seg) {
		/* Deactivate and teardown */
		avi_segment_deactivate(avi_m2m->stats_seg);

		/* Force fifo clear FIFO node of stats segment */
		avi_fifo_force_clear(avi_m2m->stats_fifo_node);
		avi_apply_node(avi_m2m->stats_fifo_node);
	}

	/* Deactivate segments */
	avi_segment_deactivate(avi_m2m->in_seg);
	avi_segment_deactivate(avi_m2m->out_seg);

	/* Recover in case of freeze */
	avi_fifo_force_clear(avi_m2m->out_fifo_node);

	/* Unlock segment access */
	spin_unlock(&avi_m2m->lock);

	return 0;
}

static int avi_m2m_destroy_segments(struct avi_m2m_dev *avi_m2m)
{
	/* Disable segment IRQ (DMA_OUT) */
	avi_segment_disable_irq(avi_m2m->out_seg);

	/* Remove timer */
	del_timer(&avi_m2m->timer);

	/* Destroy stats */
	if (avi_m2m->stats_seg)
		avi_segment_teardown(avi_m2m->stats_seg);

	/* Disconnect segments */
	avi_segment_disconnect(avi_m2m->in_seg, avi_m2m->out_seg);

	/* Destroy segments */
	avi_segment_teardown(avi_m2m->in_seg);
	avi_segment_teardown(avi_m2m->out_seg);

	/* Reset segments */
	avi_m2m->in_seg = NULL;
	avi_m2m->out_seg = NULL;
	avi_m2m->stats_seg = NULL;

	return 0;
}

static int avi_m2m_check_buffer(struct avi_m2m_dev *avi_m2m,
				struct avi_m2m_format *fmt,
				struct avi_dma_buffer *buf)
{
	/* Check buffer status */
	if (buf->status != AVI_BUFFER_READY) {
		dev_err(avi_m2m->dev, "Buffer not ready!\n");
		return -EINVAL;
	}

	/* Check buffer DMA address */
	if (buf->plane0.dma_addr == 0) {
		dev_err(avi_m2m->dev, "Buffer plane #0 address is zero!\n");
		return -EINVAL;
	}

	/* Check buffer DMA address for second plane (multi-planar mode) */
	if (fmt && fmt->avi_fmt.plane1.line_size != 0 &&
	    buf->plane1.dma_addr == 0) {
		dev_err(avi_m2m->dev, "Buffer plane #1 address is zero!\n");
		return -EINVAL;
	}

	return 0;
}

/* Adjust rectangle into image */
static void avi_m2m_adjust_strip_into_image(struct v4l2_rect *rect,
					    struct v4l2_pix_format *fmt)
{
	struct avi_dma_pixfmt pix_fmt =
				       avi_v4l2_to_avi_pixfmt(fmt->pixelformat);

	avi_limit_adjust_w_strip(pix_fmt, fmt->width,
				 &rect->width, &rect->left);
	avi_limit_adjust_h_strip(pix_fmt, fmt->height,
				 &rect->height, &rect->top);
}
/* Adjust rectangle into bounds: rectangle is truncated into bounds */
static void avi_m2m_adjust_crop_into_bounds(struct v4l2_rect *rect,
					    const struct v4l2_rect *bounds)
{
	u32 val, limit;

	/* Left side */
	if (rect->left < bounds->left)
		rect->left = bounds->left;

	/* Top side */
	if (rect->top < bounds->top)
		rect->top = bounds->top;

	/* Right side */
	limit = bounds->left + bounds->width - 1;
	val = rect->left + rect->width - 1;
	if (val > limit)
		rect->width = val - limit;

	/* Bottom side */
	limit = bounds->top + bounds->height - 1;
	val = rect->top + rect->height - 1;
	if (val > limit)
		rect->height = val - limit;
}

static void avi_m2m_setup_avi_dma_buffer(struct vb2_buffer *vb,
					 struct avi_m2m_format *fmt,
					 struct avi_dma_buffer *buf)
{
	dma_addr_t dma_addr;

	/* Get DMA address from vb2 buffer */
	dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	/* Setup AVI DMA buffer */
	avi_v4l2_setup_dmabuf(&fmt->rect, &fmt->avi_fmt, fmt->plane0_size,
			      fmt->plane1_size, dma_addr, vb, buf);
}

static inline void avi_m2m_to_avi_segment_format(struct avi_m2m_format *fmt)
{
	avi_v4l2_to_segment_fmt(&fmt->v4l2_fmt, &fmt->rect, &fmt->avi_fmt,
				&fmt->plane0_size, &fmt->plane1_size);

	/* We support only progressive mode */
	fmt->avi_fmt.interlaced = 0;
}

static int avi_m2m_check_format(struct avi_m2m_dev *avi_m2m,
				 struct avi_segment_format *fmt)
{
	enum avi_pixel_packing packing;
	int w;

	/* Check resolution */
	if (fmt->width == 0 || fmt->height == 0) {
		dev_err(avi_m2m->dev, "Bad resolution!\n");
		return -EINVAL;
	}

	/* Get packing */
	packing = avi_pixfmt_get_packing(fmt->pix_fmt);

	/* Width must be even to match the subsampling */
	if ((packing == AVI_SEMIPLANAR_YUV_422_PACKING ||
	     packing == AVI_SEMIPLANAR_YUV_420_PACKING ||
	     packing == AVI_INTERLEAVED_YUV_422_PACKING) &&
	    fmt->width % 2) {
		dev_err(avi_m2m->dev, "Width is not event (%d)!\n", fmt->width);
		return -EINVAL;
	}

	/* Height must be even to match the subsampling */
	if (packing == AVI_SEMIPLANAR_YUV_420_PACKING &&
	    fmt->height % 2) {
		dev_err(avi_m2m->dev, "Hidth is not event (%d)!\n",
			fmt->height);
		return -EINVAL;
	}

	/* Check line size */
	w = fmt->width * avi_pixel_size0(fmt->pix_fmt);
	if (fmt->plane0.line_size == 0 || w > fmt->plane0.line_size) {
		dev_err(avi_m2m->dev, "Bad line size for plane #0 (%d / %d)!\n",
			fmt->plane0.line_size, w);
		return -EINVAL;
	}

	/* Check line size in mutli planar */
	if (avi_pixel_size1(fmt->pix_fmt)) {
		w = fmt->width * avi_pixel_size1(fmt->pix_fmt);
		if (fmt->plane1.line_size == 0 || w > fmt->plane1.line_size) {
			dev_err(avi_m2m->dev,
				"Bad line size for plane #1 (%d / %d)!\n",
				fmt->plane1.line_size, w);
			return -EINVAL;
		}
	}

	return 0;
}

static int avi_m2m_apply_format(struct avi_m2m_dev *avi_m2m)
{
	struct avi_segment_format *src_fmt = &avi_m2m->src_format.avi_fmt;
	struct avi_segment_format *dst_fmt = &avi_m2m->dst_format.avi_fmt;
	struct avi_segment_format left_fmt, right_fmt;
	struct avi_segment_layout compose_layout;
	struct avi_segment_format compose_fmt;
	int ret;

	/* Prepare compose format */
	if (avi_m2m->use_compose) {
		compose_fmt.width = avi_m2m->compose_rect.width;
		compose_fmt.height = avi_m2m->compose_rect.height;
		compose_fmt.colorspace = dst_fmt->colorspace;
		compose_layout.x = avi_m2m->compose_rect.left;
		compose_layout.y = avi_m2m->compose_rect.top;
		compose_layout.alpha = avi_m2m->compose_alpha;
		compose_layout.hidden = 0;
	} else {
		compose_fmt = *dst_fmt;
		compose_layout.x = 0;
		compose_layout.y = 0;
		compose_layout.alpha = 0xFF;
		compose_layout.hidden = 0;
	}

	/* Force non-DMA and progressive mode for compose */
	compose_fmt.pix_fmt = AVI_PIXFMT_INVALID;
	compose_fmt.plane0.line_size = 0;
	compose_fmt.plane1.line_size = 0;
	compose_fmt.interlaced = 0;

	/* Check formats */
	if (avi_m2m_check_format(avi_m2m, src_fmt) ||
	    avi_m2m_check_format(avi_m2m, dst_fmt))
		return -EINVAL;

	/* Check if scaler is available */
	if ((src_fmt->width != dst_fmt->width ||
	     src_fmt->height != dst_fmt->height) &&
	    !(avi_m2m->current_caps & AVI_CAP_SCAL))
		return -EINVAL;

	/* Determine formats between input and output segments
	 * Composing: left = compose, right = target
	 * +--------+             +------+-------+               +--------+
	 * | source |-- INPUT --> | left | right |--> OUTPUT --> | target |
	 * +--------+             +------+-------+               +--------+
	 * Not composing: left = target, right = target
	 */
	right_fmt = *dst_fmt;
	left_fmt = avi_m2m->use_compose ? compose_fmt : right_fmt;

	/* Force non-DMA and progressive mode for compose */
	right_fmt.pix_fmt = AVI_PIXFMT_INVALID;
	left_fmt.pix_fmt = AVI_PIXFMT_INVALID;
	right_fmt.plane0.line_size = 0;
	left_fmt.plane0.line_size = 0;
	right_fmt.plane1.line_size = 0;
	left_fmt.plane1.line_size = 0;

#define CHECK_FMTS(a, b) memcmp(a, b, sizeof(struct avi_segment_format))
#define CHECK_LAYS(a, b) memcmp(a, b, sizeof(struct avi_segment_layout))

	/* Update output segment format */
	if ((CHECK_FMTS(&avi_m2m->out_seg->input_format, &right_fmt) != 0) ||
	    (CHECK_FMTS(&avi_m2m->out_seg->output_format, dst_fmt) != 0) ) {
		ret = avi_segment_set_format(avi_m2m->out_seg, &right_fmt,
					     dst_fmt);
		if (ret) {
			dev_err(avi_m2m->dev, "Failed to set output format!\n");
			return ret;
		}
	}

	/* Update input segment */
	if ((CHECK_FMTS(&avi_m2m->in_seg->input_format, src_fmt) != 0) ||
	    (CHECK_FMTS(&avi_m2m->in_seg->output_format, &left_fmt) != 0) ||
	    (CHECK_LAYS(&avi_m2m->in_seg->layout, &compose_layout) != 0)) {
		ret = avi_segment_set_format_and_layout(avi_m2m->in_seg,
							src_fmt, &left_fmt,
							&compose_layout);
		if (ret) {
			dev_err(avi_m2m->dev, "Failed to set input format!\n");
			return ret;
		}

		/* Update stats format in any case */
		if (avi_m2m->stats_seg && avi_m2m->use_stats) {
			ret = avi_stats_apply_format(&avi_m2m->stats,
						     avi_m2m->isp_node,
						     avi_m2m->stats_seg,
						     src_fmt, 1);
			if (ret) {
				dev_err(avi_m2m->dev,
					"Failed to set stats format!\n");
				return ret;
			}
		}
	}

#undef CHECK_LAYS
#undef CHECK_FMTS

	/* Update stats format */
	if (avi_m2m->stats_seg && avi_m2m->use_stats) {
		ret = avi_stats_apply_format(&avi_m2m->stats, avi_m2m->isp_node,
					     avi_m2m->stats_seg, src_fmt, 0);
		if (ret) {
			dev_err(avi_m2m->dev, "Failed to set stats format!\n");
			return ret;
		}
	}

	/* Set background when composing */
	if (avi_m2m->use_compose) {
		avi_segment_set_background(avi_m2m->out_seg, 0);
	}

	return 0;
}

/*
 * MEM2MEM callbacks
 */
static void avi_m2m_device_run(void *priv)
{
	struct avi_m2m_dev *avi_m2m = priv;
	struct vb2_buffer *src_vb, *dst_vb;
	int ret;

#ifdef CONFIG_M2M_AVI_USE_ISP
	/* Do blanking ISP task */
	avi_v4l2_isp_blanking(avi_m2m->ctrl_isp);
#endif

	/* Update format and AVI configuration */
	if (avi_m2m->configuration_changed) {
		/* Update AVI format from V4L2 format */
		avi_m2m_to_avi_segment_format(&avi_m2m->src_format);
		avi_m2m_to_avi_segment_format(&avi_m2m->dst_format);
		avi_m2m->configuration_changed = 0;
	}

	/* Get next input buffer */
	src_vb = v4l2_m2m_next_src_buf(avi_m2m->m2m_ctx);
	avi_m2m_setup_avi_dma_buffer(src_vb, &avi_m2m->src_format,
				     &avi_m2m->src_buf);

	/* Get next output buffer */
	dst_vb = v4l2_m2m_next_dst_buf(avi_m2m->m2m_ctx);
	avi_m2m_setup_avi_dma_buffer(dst_vb, &avi_m2m->dst_format,
				     &avi_m2m->dst_buf);

	/* Get next stats buffer */
	avi_m2m->stats_buf.plane0.dma_addr = 0;
	if (avi_m2m->stats_enabled && avi_m2m->use_stats)
		avi_stats_next(&avi_m2m->stats, &avi_m2m->stats_buf);

	/* Check DMA buffers */
	if (avi_m2m_check_buffer(avi_m2m, &avi_m2m->src_format,
							   &avi_m2m->src_buf) ||
	    avi_m2m_check_buffer(avi_m2m, &avi_m2m->dst_format,
							     &avi_m2m->dst_buf))
		goto error;

	/* Check stats DMA buffer */
	if (avi_m2m->stats_buf.plane0.dma_addr != 0 &&
	    avi_m2m_check_buffer(avi_m2m, NULL, &avi_m2m->stats_buf)) {
		dev_err(avi_m2m->dev, "Bad stats buffer!\n");
		goto error;
	}

	/* Update V4L2 buffer fields */
	avi_m2m->sequence++;
	src_vb->v4l2_buf.sequence = avi_m2m->sequence;
	v4l2_get_timestamp(&src_vb->v4l2_buf.timestamp);

	/* Apply format */
	ret = avi_m2m_apply_format(avi_m2m);
	if (ret) {
		dev_err(avi_m2m->dev, "Failed to apply format!\n");
		goto error;
	}

	/* Activate segments (One shot mode) */
	ret = avi_m2m_activate_oneshot(avi_m2m);
	if (ret) {
		dev_err(avi_m2m->dev, "Failed to activate segments!\n");
		goto error;
	}

	return;

error:
	/* Get current buffers from MEM2MEM context */
	src_vb = v4l2_m2m_src_buf_remove(avi_m2m->m2m_ctx);
	dst_vb = v4l2_m2m_dst_buf_remove(avi_m2m->m2m_ctx);

	/* Failed to get buffer */
	v4l2_m2m_buf_done(src_vb, VB2_BUF_STATE_ERROR);
	v4l2_m2m_buf_done(dst_vb, VB2_BUF_STATE_ERROR);

	/* Failed to get stats buffer */
	avi_m2m->stats_buf.status = AVI_BUFFER_ERROR;
	if (avi_m2m->stats_enabled && avi_m2m->stats_buf.plane0.dma_addr)
		avi_stats_done(&avi_m2m->stats, &avi_m2m->stats_buf,
				src_vb->v4l2_buf.sequence, 1);
	/* Job is finished */
	v4l2_m2m_job_finish(avi_m2m->m2m_dev, avi_m2m->m2m_ctx);
}

static int avi_m2m_job_ready(void *priv)
{
	struct avi_m2m_dev *avi_m2m = priv;

	/* Check if input and output buffers are available */
	if (v4l2_m2m_num_src_bufs_ready(avi_m2m->m2m_ctx) &&
	    v4l2_m2m_num_dst_bufs_ready(avi_m2m->m2m_ctx))
		return 1;

	return 0;
}

static void avi_m2m_job_abort(void *priv)
{
	struct avi_m2m_dev *avi_m2m = priv;
	v4l2_m2m_job_finish(avi_m2m->m2m_dev, avi_m2m->m2m_ctx);
}

static void avi_m2m_lock(void *priv)
{
	struct avi_m2m_dev *avi_m2m = priv;
	mutex_lock(&avi_m2m->mutex);
}

static void avi_m2m_unlock(void *priv)
{
	struct avi_m2m_dev *avi_m2m = priv;
	mutex_unlock(&avi_m2m->mutex);
}

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= avi_m2m_device_run,
	.job_ready	= avi_m2m_job_ready,
	.job_abort	= avi_m2m_job_abort,
	.lock		= avi_m2m_lock,
	.unlock		= avi_m2m_unlock,
};

/*
 * video ioctls
 */
static int avi_m2m_querycap(struct file *file, void *priv,
			    struct v4l2_capability *cap)
{
	/* Set string infos */
	strlcpy(cap->driver, DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, AVI_V4L2_BUS_VERSION, sizeof(cap->bus_info));
	snprintf(cap->card, sizeof(cap->card) - 1, "avi-m2m");

	/* Set capabilites */
	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	/* Reset reserved part */
	memset(cap->reserved, 0, sizeof(cap->reserved));

	return 0;
}

/* Formats ops */
static int avi_m2m_enum_fmt_vid(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(f->type))
		return -EINVAL;

	return avi_v4l2_enum_fmt(f);
}

static int avi_m2m_try_fmt_vid(struct file *file, void *priv,
			       struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret = 0;

	/* MEM2MEM works only in progressive */
	pix->field = V4L2_FIELD_NONE;

	/* Check if AVI support the format */
	ret = avi_v4l2_try_fmt(f->type, pix);
	if (ret < 0)
	{
		/* Change with default format */
		pix->pixelformat = AVI_M2M_DEFAULT_PIX_FMT;
		pix->colorspace  = AVI_M2M_DEFAULT_COLORSPACE;
		ret = avi_v4l2_try_fmt(f->type, pix);
	}

	return ret;
}

static int avi_m2m_g_fmt_vid(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);

	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(f->type))
		return -EINVAL;

	/* Lock format access */
	mutex_lock(&avi_m2m->mutex);

	/* Get V4L2 format */
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		f->fmt.pix = avi_m2m->src_format.v4l2_fmt;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		f->fmt.pix = avi_m2m->dst_format.v4l2_fmt;
		break;
	default:
		BUG();
	}

	/* Unlock format access */
	mutex_unlock(&avi_m2m->mutex);

	return 0;
}

static int avi_m2m_s_fmt_vid_unlocked(struct file *file, void *priv,
				      struct v4l2_format *f)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	struct avi_m2m_format *format;
	int ret = 0;

	/* Do not change format during streaming */
	if (avi_m2m_is_streaming(avi_m2m, f->type))
		return -EBUSY;

	/**
	 * V4L2 requirement says "[colorspace] must be set
	 * by the driver for capture streams and by the application
	 * for output streams". Thus for our m2m driver, we will
	 * force the CAPTURE colorspace same as OUTPUT.
	 *
	 * <<<Warning>>> wierd things can occurs, if pixel format differ
	 * between OUTPUT and CAPTURE: i.e. OUPUT=YUV and CAPTURE=RGB,
	 * colorspace is different...
	 */
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    f->fmt.pix.colorspace == AVI_NULL_CSPACE)
		f->fmt.pix.colorspace = avi_m2m->src_format.v4l2_fmt.colorspace;

	/* Try format before setting */
	ret = avi_m2m_try_fmt_vid(file, priv, f);
	if (ret < 0)
		return ret;

	/* Get format to set from type */
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		format = &avi_m2m->src_format;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		format = &avi_m2m->dst_format;
		break;
	default:
		BUG();
	}

	/* Update format */
	format->v4l2_fmt    = f->fmt.pix;
	format->rect.left   = 0;
	format->rect.top    = 0;
	format->rect.width  = f->fmt.pix.width;
	format->rect.height = f->fmt.pix.height;
	format->bounds      = format->rect;
	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		avi_m2m->compose_rect = format->rect;

	/* Get AVI segment format */
	avi_m2m_to_avi_segment_format(format);

	/* Format has changed */
	set_bit(f->type, &avi_m2m->configuration_changed);

	return 0;
}

static int avi_m2m_s_fmt_vid(struct file *file, void *priv,
			     struct v4l2_format *f)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(f->type))
		return -EINVAL;

	/* Set format */
	mutex_lock(&avi_m2m->mutex);
	ret = avi_m2m_s_fmt_vid_unlocked(file, priv, f);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

/* Buffer ops */
static int avi_m2m_create_bufs(struct file *file, void *priv,
			       struct v4l2_create_buffers *create)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	struct vb2_queue *vq;
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(create->format.type))
		return -EINVAL;

	/* Create buffer */
	mutex_lock(&avi_m2m->mutex);
	vq  = v4l2_m2m_get_vq(avi_m2m->m2m_ctx, create->format.type);
	ret = vb2_create_bufs(vq, create);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_prepare_buf(struct file *file, void *priv,
			       struct v4l2_buffer *buf)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	struct vb2_queue *vq;
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(buf->type))
		return -EINVAL;

	/* Prepare buffer */
	mutex_lock(&avi_m2m->mutex);
	vq  = v4l2_m2m_get_vq(avi_m2m->m2m_ctx, buf->type);
	ret = vb2_prepare_buf(vq, buf);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_reqbufs(struct file *file, void *priv,
			   struct v4l2_requestbuffers *reqbufs)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(reqbufs->type))
		return -EINVAL;

	/* Request buffer */
	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_reqbufs(file, avi_m2m->m2m_ctx, reqbufs);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_querybuf(struct file *file, void *priv,
			    struct v4l2_buffer *buf)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(buf->type))
		return -EINVAL;

	/* Query buffer */
	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_querybuf(file, avi_m2m->m2m_ctx, buf);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(buf->type))
		return -EINVAL;

	/* Queue buffer */
	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_qbuf(file, avi_m2m->m2m_ctx, buf);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(buf->type))
		return -EINVAL;

	/* Dequeue buffer */
	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_dqbuf(file, avi_m2m->m2m_ctx, buf);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_expbuf(struct file *file, void *priv,
			  struct v4l2_exportbuffer *ebuf)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	/* Check buffer type */
	if (!AVI_M2M_CHECK_TYPE(ebuf->type))
		return -EINVAL;

	/* Export buffer */
	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_expbuf(file, avi_m2m->m2m_ctx, ebuf);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

/* Streaming ops */
static int avi_m2m_streamon(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
#ifdef CONFIG_M2M_AVI_USE_ISP
	struct avi_isp_offsets offsets;
#endif
	int ret = 0;

	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(type))
		return -EINVAL;

	/* Lock MEM2MEM context access */
	mutex_lock(&avi_m2m->mutex);

	/* Already streaming */
	if (avi_m2m_is_streaming(avi_m2m, type)) {
		/* Unlock MEM2MEM context access */
		mutex_unlock(&avi_m2m->mutex);
		return -EBUSY;
	}

	/* Start capturing */
	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		/* Build segments */
		ret = avi_m2m_build_segments(avi_m2m);
		if (ret)
			goto build_failed;

		/* Reload configuration for first frame */
		set_bit(type, &avi_m2m->configuration_changed);

#ifdef CONFIG_M2M_AVI_USE_ISP
		/* Add ISP controls */
		if (avi_m2m->current_caps & AVI_CAPS_ISP) {
			/* Get ISP offsets */
			ret = avi_isp_get_offsets(avi_m2m->in_seg, &offsets);
			if (ret)
				goto isp_failed;

			/* Activate ISP controls */
			avi_v4l2_isp_activate(avi_m2m->ctrl_isp, &offsets);
			avi_v4l2_isp_blanking(avi_m2m->ctrl_isp);
		}
 #endif
	}

	/* Stream on */
	ret = v4l2_m2m_streamon(file, avi_m2m->m2m_ctx, type);
	if (ret)
		goto streamon_failed;

	/* Set streaming bit */
	avi_m2m_set_streaming(avi_m2m, type, 1);

	/* Unlock MEM2MEM context access */
	mutex_unlock(&avi_m2m->mutex);

	return 0;

streamon_failed:
#ifdef CONFIG_M2M_AVI_USE_ISP
	if (avi_m2m->current_caps & AVI_CAPS_ISP)
		avi_v4l2_isp_deactivate(avi_m2m->ctrl_isp);
isp_failed:
#endif
	avi_m2m_destroy_segments(avi_m2m);
build_failed:
	/* Unlock MEM2MEM context access */
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_streamoff(struct file *file, void *priv,
			     enum v4l2_buf_type type)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret = 0;

	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(type))
		return -EINVAL;

	/* Lock MEM2MEM context access */
	mutex_lock(&avi_m2m->mutex);

	/* Stream off */
	if (avi_m2m_is_streaming(avi_m2m, type)) {
		/* Stream off MEM2MEM context */
		v4l2_m2m_streamoff(file, avi_m2m->m2m_ctx, type);

		/* Stop capturing */
		if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
#ifdef CONFIG_M2M_AVI_USE_ISP
			/* Deactivate ISP chains controls */
			if (avi_m2m->current_caps & AVI_CAPS_ISP) {
				avi_v4l2_isp_deactivate(avi_m2m->ctrl_isp);
			}
#endif

			/* Force deactivation of segments */
			avi_m2m_deactivate_segments(avi_m2m);

			/* Destroy segments */
			avi_m2m_destroy_segments(avi_m2m);
		}

		/* Clear streaming bit */
		avi_m2m_set_streaming(avi_m2m, type, 0);
	}

	/* Unlock MEM2MEM context access */
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

/* Selection ops */
static int avi_m2m_g_selection(struct file *file, void *priv,
			       struct v4l2_selection *s)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret = 0;

	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(s->type))
		return -EINVAL;

	/* Lock format access */
	mutex_lock(&avi_m2m->mutex);

	/* Get selection */
	switch (s->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		switch(s->target) {
		case V4L2_SEL_TGT_CROP:
		case V4L2_SEL_TGT_COMPOSE:
		case V4L2_SEL_TGT_COMPOSE_PADDED:
			s->r = avi_m2m->src_format.rect;
			break;
		case V4L2_SEL_TGT_CROP_BOUNDS:
		case V4L2_SEL_TGT_CROP_DEFAULT:
		case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		case V4L2_SEL_TGT_COMPOSE_DEFAULT:
			s->r = avi_m2m->src_format.bounds;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		switch(s->target) {
		case V4L2_SEL_TGT_CROP:
			s->r = avi_m2m->dst_format.rect;
			break;
		case V4L2_SEL_TGT_COMPOSE:
		case V4L2_SEL_TGT_COMPOSE_PADDED:
			s->r = avi_m2m->compose_rect;
			break;
		case V4L2_SEL_TGT_CROP_BOUNDS:
		case V4L2_SEL_TGT_CROP_DEFAULT:
		case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		case V4L2_SEL_TGT_COMPOSE_DEFAULT:
			s->r = avi_m2m->dst_format.bounds;
			break;
		default:
			ret = -EINVAL;
		}
		break;
	default:
		BUG();
	}

	/* Unlock format access */
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_s_selection(struct file *file, void *priv,
			       struct v4l2_selection *s)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	struct avi_m2m_format *fmt;
	int ret = 0;

	/* Check type */
	if (!AVI_M2M_CHECK_TYPE(s->type))
		return -EINVAL;

	/* Only CROP and COMPOSE are supported */
	if (s->target != V4L2_SEL_TGT_CROP && s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	/* Lock format access */
	mutex_lock(&avi_m2m->mutex);

	/* Set selection */
	switch (s->type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		fmt = &avi_m2m->src_format;
		/* Adjust crop rectangle into bounds rectangle */
		avi_m2m_adjust_crop_into_bounds(&s->r, &fmt->bounds);

		/* Adjust strip into image */
		avi_m2m_adjust_strip_into_image(&s->r, &fmt->v4l2_fmt);

		/* Update crop */
		fmt->rect = s->r;
		set_bit(s->type, &avi_m2m->configuration_changed);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		fmt = &avi_m2m->dst_format;
		switch (s->target) {
		case V4L2_SEL_TGT_CROP:
			/* Adjust crop rectangle into bounds rectangle */
			avi_m2m_adjust_crop_into_bounds(&s->r, &fmt->bounds);

			/* Adjust strip into image */
			avi_m2m_adjust_strip_into_image(&s->r, &fmt->v4l2_fmt);

			/* Update crop and compose rect */
			fmt->rect = s->r;
			avi_m2m->compose_rect = s->r;
			break;
		case V4L2_SEL_TGT_COMPOSE:
			/* Adjust compose rectangle into crop rectangle */
			avi_m2m_adjust_crop_into_bounds(&s->r, &fmt->rect);

			/* Update compose rect */
			avi_m2m->compose_rect = s->r;
			break;
		default:
			ret = -EINVAL;
			goto error;
		}

		/* Use compose if compose rect is bigger than crop */
		avi_m2m->use_compose =
		   ((avi_m2m->compose_rect.left > fmt->rect.left) ||
		    (avi_m2m->compose_rect.top > fmt->rect.top)   ||
		    (avi_m2m->compose_rect.left + avi_m2m->compose_rect.width) <
					   (fmt->rect.left + fmt->rect.width) ||
		    (avi_m2m->compose_rect.top + avi_m2m->compose_rect.height) <
					    (fmt->rect.top + fmt->rect.height));

		/* Configuration has changed */
		set_bit(s->type, &avi_m2m->configuration_changed);
		break;
	default:
		BUG();
	}

error:
	/* Unlock format access */
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

/* Deprecated ops */
static long avi_m2m_default(struct file *file, void *priv, bool valid_prio,
			    int cmd, void *arg)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret = -ENOIOCTLCMD;

	/* Lock access */
	mutex_lock(&avi_m2m->mutex);

	/* Check command */
	switch (cmd) {
	case AVI_ISP_IOGET_OFFSETS:
		/* ISP is not available in caps */
		if ((avi_m2m->caps & AVI_CAPS_ISP) != AVI_CAPS_ISP) {
			ret = -EINVAL;
			goto unlock;
		}

		/* Not streaming */
		if (!avi_m2m->in_seg ||
		    !avi_m2m_is_streaming(avi_m2m,
						 V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
			ret = -EINVAL;
			goto unlock;
		}

		/* Get ISP offsets from AVI segment */
		ret = avi_isp_get_offsets(avi_m2m->in_seg,
					  (struct avi_isp_offsets *) arg);
		break;
	default:
		v4l2_err(avi_m2m->v4l2_dev, "Not supported command!\n");
	}

unlock:
	/* Unlock access */
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_log_status(struct file *file, void *priv)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);

	/* Deprected IOCTL */
	dev_info(avi_m2m->dev, "/!\\ DEPRECATED IOCTL!\n");

	return 0;
}

static const struct v4l2_ioctl_ops avi_m2m_ioctl_ops = {
	/* Caps */
	.vidioc_querycap		= avi_m2m_querycap,

	/* Source format (input) */
	.vidioc_enum_fmt_vid_out	= avi_m2m_enum_fmt_vid,
	.vidioc_g_fmt_vid_out		= avi_m2m_g_fmt_vid,
	.vidioc_try_fmt_vid_out		= avi_m2m_try_fmt_vid,
	.vidioc_s_fmt_vid_out		= avi_m2m_s_fmt_vid,

	/* Target format (output) */
	.vidioc_enum_fmt_vid_cap	= avi_m2m_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap		= avi_m2m_g_fmt_vid,
	.vidioc_try_fmt_vid_cap		= avi_m2m_try_fmt_vid,
	.vidioc_s_fmt_vid_cap		= avi_m2m_s_fmt_vid,

	/* Buffers */
	.vidioc_create_bufs		= &avi_m2m_create_bufs,
	.vidioc_prepare_buf		= &avi_m2m_prepare_buf,
	.vidioc_reqbufs			= avi_m2m_reqbufs,
	.vidioc_querybuf		= avi_m2m_querybuf,
	.vidioc_qbuf			= avi_m2m_qbuf,
	.vidioc_dqbuf			= avi_m2m_dqbuf,
	.vidioc_expbuf			= avi_m2m_expbuf,

	/* Streaming */
	.vidioc_streamon		= avi_m2m_streamon,
	.vidioc_streamoff		= avi_m2m_streamoff,

	/* Selection (+ crop) */
	.vidioc_g_selection		= avi_m2m_g_selection,
	.vidioc_s_selection		= avi_m2m_s_selection,

	/* Used for getting ISP offset
	 * /!\ Deprecated: use ISP V4L2 controls
	 */
	.vidioc_default			= avi_m2m_default,

	/* [DEPRECATED] Log status */
	.vidioc_log_status		= avi_m2m_log_status,
};

/*
 * Queue operations
 */
static int avi_m2m_vb2_queue_setup(struct vb2_queue *vq,
				   const struct v4l2_format *fmt,
				   unsigned int *nbuffers,
				   unsigned int *nplanes, unsigned int sizes[],
				   void *alloc_ctxs[])
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vq);

	/* Setup queue with correct planes (assume only one plane) */
	*nplanes = 1;
	switch (vq->type)
	{
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		sizes[0] = avi_m2m->src_format.v4l2_fmt.sizeimage;
		alloc_ctxs[0] = avi_m2m->alloc_ctx;
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		sizes[0] = avi_m2m->dst_format.v4l2_fmt.sizeimage;
		alloc_ctxs[0] = avi_m2m->alloc_ctx;
		break;
	default:
		BUG();
	}

	return 0;
}

static int avi_m2m_vb2_buffer_init(struct vb2_buffer *vb)
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vb->vb2_queue);

	/* Allow GPU to access this memory through UMP */
	switch (vb->v4l2_buf.type)
	{
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		p7ump_add_map(vb2_dma_contig_plane_dma_addr(vb, 0),
			    PAGE_ALIGN(avi_m2m->src_format.v4l2_fmt.sizeimage));
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		p7ump_add_map(vb2_dma_contig_plane_dma_addr(vb, 0),
			    PAGE_ALIGN(avi_m2m->dst_format.v4l2_fmt.sizeimage));
		break;
	default:
		BUG();
	}

	return 0;
}

static int avi_m2m_vb2_buffer_prepare(struct vb2_buffer *vb)
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vb->vb2_queue);

	/* Prepate buffer with correct size */
	switch (vb->v4l2_buf.type)
	{
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		vb2_set_plane_payload(vb, 0,
				      avi_m2m->src_format.v4l2_fmt.sizeimage);
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		vb2_set_plane_payload(vb, 0,
				      avi_m2m->dst_format.v4l2_fmt.sizeimage);
		break;
	default:
		BUG();
	}

	return 0;
}

static void avi_m2m_vb2_buffer_queue(struct vb2_buffer *vb)
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vb->vb2_queue);
	v4l2_m2m_buf_queue(avi_m2m->m2m_ctx, vb);
}

static void avi_m2m_vb2_buffer_cleanup(struct vb2_buffer *vb)
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vb->vb2_queue);

	/* No buffer to remove */
	if (!vb->planes[0].mem_priv)
		return;

	/* Remove GPU access to this memory through UMP */
	switch (vb->v4l2_buf.type)
	{
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		p7ump_remove_map(vb2_dma_contig_plane_dma_addr(vb, 0),
			    PAGE_ALIGN(avi_m2m->src_format.v4l2_fmt.sizeimage));
		break;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		p7ump_remove_map(vb2_dma_contig_plane_dma_addr(vb, 0),
			    PAGE_ALIGN(avi_m2m->dst_format.v4l2_fmt.sizeimage));
		break;
	default:
		BUG();
	}
}

static void avi_m2m_vb2_wait_prepare(struct vb2_queue *vq)
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vq);
	mutex_unlock(&avi_m2m->mutex);
}

static void avi_m2m_vb2_wait_finish(struct vb2_queue *vq)
{
	struct avi_m2m_dev *avi_m2m = vb2_get_drv_priv(vq);
	mutex_lock(&avi_m2m->mutex);
}

static struct vb2_ops avi_m2m_video_qops = {
	.queue_setup	= avi_m2m_vb2_queue_setup,
	.buf_init	= avi_m2m_vb2_buffer_init,
	.buf_prepare	= avi_m2m_vb2_buffer_prepare,
	.buf_queue	= avi_m2m_vb2_buffer_queue,
	.buf_cleanup	= avi_m2m_vb2_buffer_cleanup,
	.wait_prepare	= avi_m2m_vb2_wait_prepare,
	.wait_finish	= avi_m2m_vb2_wait_finish,
};

/*
 * Queue init operations
 */
static inline int avi_m2m_vb2_queue_init(struct vb2_queue *vq,
					 enum v4l2_buf_type type,
					 void *priv)
{
	struct avi_m2m_dev *avi_m2m = (struct avi_m2m_dev *)priv;
	struct vb2_dc_conf *alloc_ctx = (struct vb2_dc_conf *)avi_m2m->alloc_ctx;
	memset(vq, 0, sizeof(*vq));
	vq->type = type;
	vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	vq->drv_priv = priv;
	vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	vq->ops = &avi_m2m_video_qops;
	vq->mem_ops = &vb2_dma_contig_memops;
	vq->cache_flags = alloc_ctx->cache_flags;

	return vb2_queue_init(vq);
}

static int avi_m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	int ret;

	/* Init input vb2 queue */
	ret = avi_m2m_vb2_queue_init(src_vq, V4L2_BUF_TYPE_VIDEO_OUTPUT, priv);
	if (ret)
		return ret;

	/* Init output vb2 queue */
	return avi_m2m_vb2_queue_init(dst_vq, V4L2_BUF_TYPE_VIDEO_CAPTURE,
				      priv);
}

/*
 * File operations
 */
static int avi_m2m_open(struct file *file)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);

	/* Lock MEM2MEM context access */
	mutex_lock(&avi_m2m->mutex);

	if (!avi_m2m->m2m_ctx) {
		/* Init MEM2MEM context (shared by all instances) */
		avi_m2m->m2m_ctx = v4l2_m2m_ctx_init(avi_m2m->m2m_dev, avi_m2m,
						     &avi_m2m_queue_init);
		if (IS_ERR(avi_m2m->m2m_ctx)) {
			/* Unlock and exit */
			mutex_unlock(&avi_m2m->mutex);
			return PTR_ERR(avi_m2m->m2m_ctx);
		}

		/* Init formats with default values */
		avi_m2m->src_format = avi_m2m_default_format;
		avi_m2m->dst_format = avi_m2m_default_format;

		/* Init composition to default values */
		avi_m2m->compose_rect = avi_m2m_default_format.rect;
		avi_m2m->compose_alpha = AVI_M2M_DEFAULT_ALPHA;
		avi_m2m->use_compose = 0;

		/* Get AVI segment formats */
		avi_m2m_to_avi_segment_format(&avi_m2m->src_format);
		avi_m2m_to_avi_segment_format(&avi_m2m->dst_format);

		/* Set bytes per line */
		avi_m2m->src_format.v4l2_fmt.bytesperline = max(
				  avi_m2m->src_format.avi_fmt.plane0.line_size,
				  avi_m2m->src_format.avi_fmt.plane1.line_size);
		avi_m2m->dst_format.v4l2_fmt.bytesperline = max(
				  avi_m2m->dst_format.avi_fmt.plane0.line_size,
				  avi_m2m->dst_format.avi_fmt.plane1.line_size);

		/* Set size image */
		avi_m2m->src_format.v4l2_fmt.sizeimage =
					       avi_m2m->src_format.plane0_size +
					       avi_m2m->src_format.plane1_size;
		avi_m2m->dst_format.v4l2_fmt.sizeimage =
					       avi_m2m->dst_format.plane0_size +
					       avi_m2m->dst_format.plane1_size;

		/* Force configuration for first time */
		set_bit(V4L2_BUF_TYPE_VIDEO_OUTPUT,
					       &avi_m2m->configuration_changed);
		set_bit(V4L2_BUF_TYPE_VIDEO_CAPTURE,
					       &avi_m2m->configuration_changed);
	}

	/* Unlock MEM2MEM context access */
	mutex_unlock(&avi_m2m->mutex);

	/* Increment instance number */
	atomic_inc(&avi_m2m->num_inst);

	return 0;
}

static int avi_m2m_release(struct file *file)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);

	/* Decrement instance number and check instance number */
	if (atomic_dec_and_test(&avi_m2m->num_inst)) {

		/* Force streamoff */
		avi_m2m_streamoff(file, avi_m2m, V4L2_BUF_TYPE_VIDEO_CAPTURE);
		avi_m2m_streamoff(file, avi_m2m, V4L2_BUF_TYPE_VIDEO_OUTPUT);

		/* Lock MEM2MEM context access */
		mutex_lock(&avi_m2m->mutex);

		/* Release MEM2MEM context */
		if (avi_m2m->m2m_ctx)
			v4l2_m2m_ctx_release(avi_m2m->m2m_ctx);
		avi_m2m->m2m_ctx = NULL;

		/* Unlock MEM2MEM context access */
		mutex_unlock(&avi_m2m->mutex);
	}

	return 0;
}

static unsigned int avi_m2m_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_poll(file, avi_m2m->m2m_ctx, wait);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static int avi_m2m_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct avi_m2m_dev *avi_m2m = video_drvdata(file);
	int ret;

	mutex_lock(&avi_m2m->mutex);
	ret = v4l2_m2m_mmap(file, avi_m2m->m2m_ctx, vma);
	mutex_unlock(&avi_m2m->mutex);

	return ret;
}

static const struct v4l2_file_operations avi_m2m_fops = {
	.owner		= THIS_MODULE,
	.open		= avi_m2m_open,
	.release	= avi_m2m_release,
	.poll		= avi_m2m_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= avi_m2m_mmap,
};

static int __devinit avi_m2m_v4l2_init(struct avi_m2m_dev *avi_m2m)
{
	struct video_device *vdev;
	int ret;

	/* Get shared V4L2 device instance */
	avi_m2m->v4l2_dev = avi_v4l2_get_device();
	if (!avi_m2m->v4l2_dev)
		return -ENODEV;

	/* Allocate new video device */
	vdev = video_device_alloc();
	if (!vdev) {
		v4l2_err(avi_m2m->v4l2_dev,
			 "Failed to allocate video device\n");
		ret = -ENODEV;
		goto vdev_alloc_failed;
	}
	avi_m2m->vdev = vdev;

	/* Init video device */
	strlcpy(vdev->name, dev_name(avi_m2m->dev), sizeof(vdev->name) - 1);

	vdev->parent	   = avi_m2m->dev;
	vdev->v4l2_dev	   = avi_m2m->v4l2_dev;
	vdev->current_norm = V4L2_STD_UNKNOWN;
	vdev->fops	   = &avi_m2m_fops;
	vdev->ioctl_ops	   = &avi_m2m_ioctl_ops;
	vdev->release	   = &video_device_release;
	vdev->vfl_type	   = VFL_TYPE_GRABBER;
	vdev->tvnorms	   = V4L2_STD_UNKNOWN;
	/* We handle the locking ourselves */
	vdev->lock	   = NULL;

	video_set_drvdata(vdev, avi_m2m);

	/* Add V4L2 controls for ISP configuration */
#ifdef CONFIG_M2M_AVI_USE_ISP
	if (avi_m2m->caps & AVI_CAPS_ISP) {
		/* Init the control handler */
		ret = v4l2_ctrl_handler_init(&avi_m2m->ctrl_handler,
					     AVI_V4L2_ISP_CONTROL_HINT);
		if (ret)
			goto init_ctrl_handler_failed;

		/* Add ISP chain controls */
		ret = avi_v4l2_isp_init(&avi_m2m->ctrl_isp,
					&avi_m2m->ctrl_handler);
		if (ret)
			goto init_isp_failed;

		/* Add control handler to video device */
		vdev->ctrl_handler = &avi_m2m->ctrl_handler;
	}
#endif

	/* Register video device */
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		goto video_register_failed;

	/* Init MEM2MEM device */
	avi_m2m->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(avi_m2m->m2m_dev)) {
		v4l2_err(avi_m2m->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(avi_m2m->m2m_dev);
		goto v4l2_m2m_init_failed;
	}

	/* Init stats video device */
	if (avi_m2m->stats_enabled && (avi_m2m->caps & AVI_CAPS_ISP)) {
		ret = avi_stats_init(&avi_m2m->stats, vdev, DRIVER_VERSION);
		if (ret)
			goto init_stats_failed;
	}

	return 0;

init_stats_failed:
	v4l2_m2m_release(avi_m2m->m2m_dev);
v4l2_m2m_init_failed:
	video_unregister_device(vdev);
video_register_failed:
#ifdef CONFIG_M2M_AVI_USE_ISP
	if (avi_m2m->caps & AVI_CAPS_ISP)
		avi_v4l2_isp_free(avi_m2m->ctrl_isp);
init_isp_failed:
	if (avi_m2m->caps & AVI_CAPS_ISP)
		v4l2_ctrl_handler_free(&avi_m2m->ctrl_handler);
init_ctrl_handler_failed:
#endif
	video_unregister_device(avi_m2m->vdev);
vdev_alloc_failed:
	avi_v4l2_put_device(avi_m2m->v4l2_dev);
	return ret;
}

static void __devexit avi_m2m_v4l2_destroy(struct avi_m2m_dev *avi_m2m)
{
	/* Remove stats video device */
	if (avi_m2m->stats_enabled && (avi_m2m->caps & AVI_CAPS_ISP))
		avi_stats_destroy(&avi_m2m->stats);

	/* Release V4L2 MEM2MEM device */
	v4l2_m2m_release(avi_m2m->m2m_dev);

	/* Remove V4L2 controls */
#ifdef CONFIG_M2M_AVI_USE_ISP
	if (avi_m2m->caps & AVI_CAPS_ISP) {
		avi_v4l2_isp_free(avi_m2m->ctrl_isp);
		v4l2_ctrl_handler_free(&avi_m2m->ctrl_handler);
	}
#endif

	/* Unregister video device */
	video_unregister_device(avi_m2m->vdev);
	avi_v4l2_put_device(avi_m2m->v4l2_dev);
}

static int __devinit avi_m2m_probe(struct platform_device *pdev)
{
	struct avi_m2m_platform_data *pdata;
	struct avi_m2m_dev *avi_m2m;
	struct vb2_dc_conf *alloc_ctx;
	int ret;

	/* Get platform data */
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "Failed to find platform data!\n");
		return -EINVAL;
	}

	/* Allocate private device structure */
	avi_m2m = kzalloc(sizeof *avi_m2m, GFP_KERNEL);
	if (!avi_m2m) {
		dev_err(&pdev->dev, "Failed to allocate!\n");
		return -ENOMEM;
	}

	/* Save platform device */
	avi_m2m->dev = &pdev->dev;
	avi_m2m->id = pdev->id;
	avi_m2m->caps = pdata->caps;
	avi_m2m->stats_enabled = pdata->enable_stats;

	/* Gives cache flags to stats */
	avi_m2m->stats.stat_vb2_cache_flags =
		pdata->stat_vb2_cache_flags;

	/* Init spin lock and mutex */
	mutex_init(&avi_m2m->mutex);
	spin_lock_init(&avi_m2m->lock);
	atomic_set(&avi_m2m->num_inst, 0);

	/* Init V4L2 device */
	ret = avi_m2m_v4l2_init(avi_m2m);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init V4L2 device!\n");
		goto error;
	}

	/* Register driver data */
	platform_set_drvdata(pdev, avi_m2m);
	dev_set_drvdata(&pdev->dev, avi_m2m);

	/* Video device is registered */
	dev_info(avi_m2m->dev,
	         "video device successfuly registered as %s\n",
	         video_device_node_name(avi_m2m->vdev));

	/* Init videobuf2 DMA contig context */
	alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(alloc_ctx)) {
		dev_err(&pdev->dev, "Failed to init Videobuf2 DMA contig!\n");
		ret = PTR_ERR(alloc_ctx);
		goto error;
	}
	alloc_ctx->cache_flags = pdata->vb2_cache_flags;
	avi_m2m->alloc_ctx = (void *)alloc_ctx;

	return 0;

error:
	kfree(avi_m2m);
	return ret;
}

static int __devexit avi_m2m_remove(struct platform_device *pdev)
{
	struct avi_m2m_dev *avi_m2m = dev_get_drvdata(&pdev->dev);

	/* Destroy V4L2 device */
	avi_m2m_v4l2_destroy(avi_m2m);

	/* Clean videobuf2 */
	vb2_dma_contig_cleanup_ctx(avi_m2m->alloc_ctx);

	/* Free device */
	dev_set_drvdata(&pdev->dev, NULL);
	platform_set_drvdata(pdev, NULL);
	kfree(avi_m2m);

	return 0;
}

static struct platform_driver avi_m2m_driver = {
	.probe		= avi_m2m_probe,
	.remove		= __devexit_p(avi_m2m_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init avi_m2m_init(void)
{
	/* Register driver */
	return platform_driver_register(&avi_m2m_driver);
}

static void __exit avi_m2m_exit(void)
{
	/* Unregister driver */
	platform_driver_unregister(&avi_m2m_driver);
}

module_init(avi_m2m_init);
module_exit(avi_m2m_exit);

MODULE_AUTHOR("Alexandre Dilly <alexandre.dilly@parrot.com>");
MODULE_DESCRIPTION("V4L2 MEM2MEM driver for P7 Advanced Video Interface");
MODULE_LICENSE("GPL");
