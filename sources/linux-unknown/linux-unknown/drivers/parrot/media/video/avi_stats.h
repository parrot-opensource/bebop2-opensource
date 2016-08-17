#ifndef _AVI_STATS_H_
#define _AVI_STATS_H_

#include <media/v4l2-common.h>
#include <media/videobuf2-core.h>

#include "avi_v4l2.h"

#define AVI_STATS_WIDTH		384
#define AVI_STATS_HEIGHT	49

struct avi_vbuf {
	/* *must* be first */
	struct vb2_buffer	 vb;
	struct list_head	 list;
	/* For interlaced streams we capture two fields in a single frame */
	int			 last_capture;
};

struct avi_stats {
	/* Buffer queue */
	spinlock_t			 vbq_lock;
	struct list_head		 bufqueue;
	/* Video device */
	struct video_device		*vdev;
	struct vb2_queue		 vb_vidq;
	struct vb2_alloc_ctx		*alloc_ctx;
	/* Capture format */
	struct v4l2_pix_format		 v4l2_fmt;
	struct v4l2_rect		 crop;
	struct avi_segment_format	 avi_fmt;
	unsigned			 plane0_size;
	unsigned			 plane1_size;
	int				 updated_fmt;
	/* Serialization lock */
	struct mutex			 lock;
	int				 streaming;
	int				 use_count;
	u32				 version;
	enum vb2_cache_flags             stat_vb2_cache_flags;
};

static inline struct avi_vbuf* to_avi_vbuf(struct vb2_buffer* vb2)
{
	return container_of(vb2, struct avi_vbuf, vb);
}

/* avi_stats_done() - Return a filled buffer to V4L2 capture interface.
 * @stats:	The stats V4L2 capture interface handler.
 * @frame:	The filled buffer (can be empty with AVI_BUFFER_ERROR flag).
 * @sequence:	Sequence number of V4L2 frame.
 * @requeue:	If AVI_BUFFER_ERROR flag is set and requeue is not equel to zero
 *              the buffer is not returned to V4L2 interface and just requeued
 *              internaly.
 *
 * Return a processed buffer to V4L2 capture interface dedicated to bayer
 * statistics output.
 */
extern void avi_stats_done(struct avi_stats *stats,
			   struct avi_dma_buffer *frame, u32 sequence,
			   int requeue);

/* avi_stats_next() - Get a new buffer from V4L2 capture interface.
 * @stats:	The stats V4L2 capture interface handler.
 * @frame:	The buffer from V4L2.
 *
 * If no buffer are available from V4L2 interface, the avi_dma_buffer structure
 * is not modified. In order to detect if a buffer is available, initialize the
 * avi_dma_buffer passed with zeros and check if plane0.dma_addr is set.
 */
extern void avi_stats_next(struct avi_stats *stats,
			   struct avi_dma_buffer *frame);

/* avi_stats_apply_format() - Apply format on stats segment output.
 * @stats:	The stats V4L2 capture interface handler.
 * @bayer_node:	The ISP bayer node of ISP segment on which stats are connected.
 * @stats_seg:	The stats segment.
 * @src_fmt:	The input format in ISP.
 * @force:	Force format segment update. 
 *
 * Apply V4L2 format to the AVI segment format.
 */
int avi_stats_apply_format(struct avi_stats *stats,
			   struct avi_node *bayer_node,
			   struct avi_segment *stats_seg,
			   const struct avi_segment_format *src_fmt, int force);

/* avi_stats_init() - Initialize V4L2 capture interface for bayer statistics.
 * @stats:	The stats V4L2 capture interface handler.
 * @pvdev:	The video_device used in parallel of this V4L2 interface (some
 *              informations are extracted from it like driver name and parent
 *              device).
 * @version:	Driver version.
 *
 * Initialize and create a new V4L2 video device for bayer statistics output.
 */
extern int __devinit avi_stats_init(struct avi_stats *stats,
				    struct video_device *pvdev, u32 version);

/* avi_stats_destroy() -  Destroy V4L2 capture interface for bayer statistics.
 * @stats:	The stats V4L2 capture interface handler.
 *
 * Destroy the V4L2 video device dedicated for bayer statistics output.
 */
extern void __devexit avi_stats_destroy(struct avi_stats *stats);

#endif // #ifdef AVI_STATS_H
