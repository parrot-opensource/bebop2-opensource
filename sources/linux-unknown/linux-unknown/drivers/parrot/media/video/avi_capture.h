#ifndef _AVI_CAPTURE_H_
#define _AVI_CAPTURE_H_

#include "video/avi_segment.h"
#include "avi_capture_timings.h"

/* Hardcoded stats output config */
#define AVI_CAPTURE_THUMB_STATS_WIDTH	64
#define AVI_CAPTURE_THUMB_STATS_HEIGHT	48
#define AVI_CAPTURE_STATS_WIDTH		(AVI_CAPTURE_THUMB_STATS_WIDTH * 6)
#define AVI_CAPTURE_STATS_HEIGHT	(AVI_CAPTURE_THUMB_STATS_HEIGHT + 1)
/* Size of a stat buffer (in bytes) */
#define AVI_CAPTURE_STATS_FRAMESIZE     (AVI_CAPTURE_STATS_WIDTH  * \
					 AVI_CAPTURE_STATS_HEIGHT * \
					 4)

struct avi_capture_crop {
	unsigned x;
	unsigned y;
	unsigned width;
	unsigned height;
};

struct avi_capture_context;

typedef void (*avi_capture_frame_cback)(struct avi_capture_context *ctx,
					struct avi_dma_buffer      *frame,
					enum avi_field              field);

struct avi_capture_context {
        /* Callback when a frame is done capturing */
	avi_capture_frame_cback     done;
        /* Callback for the configuration of the next frame */
	avi_capture_frame_cback     next;
	/* Ditto for the stats frame. Those functions are *always* called after
	 * the equivalent video callbacks above. This means that these stats
	 * always match the last frame passed to done() or next(). */
	avi_capture_frame_cback     stats_done;
	avi_capture_frame_cback     stats_next;

        /* Private client data. */
        void                       *priv;

        /* These members need to be correctly initialized before the first call
         * to avi_capture_get_measures */
        union avi_cam_interface     interface;
        int                         interlaced;
        enum avi_colorspace         capture_cspace;
	/* If set to NULL we use a builtin measure-to-timings conversion routine
	 * that should work for standard-compliant 656 streams at
	 * least. Unfortunately many sensors use non-standard timings and need
	 * adjustment, for those we need custom code (usually in the BSP). */
	avi_capture_get_timings     measure_to_timings;

        struct device              *dev;

        struct avi_capture_timings  timings_top;
        struct avi_capture_timings  timings_bot;

        struct avi_capture_crop     crop;

        atomic_t                    timings_valid;
        wait_queue_head_t           waitq;

        /* We have two segments: one containing the camera and one containing
         * the DMA out. We need that in order to use either IRQ
         * independently. */
        struct avi_segment         *cam_segment;
        struct avi_segment         *dma_segment;
        struct avi_segment         *stats_segment;

	struct avi_node            *stats_fifo;

        struct avi_node            *cam_node;

        /* We need a rotation of two buffers: at any given moment while
         * streaming one will be receiving the frame while the next one will be
         * configured for receiving the next frame. */
        struct avi_dma_buffer       buffers[2];
        struct avi_dma_buffer      *configured_buffer;
        struct avi_dma_buffer      *capture_buffer;

	/* Ditto for stats */
	struct avi_dma_buffer       stats[2];
	struct avi_dma_buffer      *configured_stats;
        struct avi_dma_buffer      *capture_stats;

        atomic_t                    streaming;

	/* Give up while waiting for a frame for more than timeout_jiffies
	 * jiffies. The frame is returned in the done() callback with status set
	 * to AVI_BUFFER_TIMEOUT. If 0 the timeout is ignored. */
        struct timer_list           timer;
        unsigned long               timeout_jiffies;
        struct timeval              last_frame;
        struct timeval              last_captured_frame;
        struct timeval              last_stats_frame;
        struct timeval              last_captured_stats_frame;

        spinlock_t                  lock;
};

/* Allocate the resources needed for the capture. If stats is true and caps
 * contains an ISP we also allocate the resources for capturing the stat
 * buffers. */
extern int avi_capture_init(struct avi_capture_context	*ctx,
                            struct device		*dev,
                            int				 major,
                            unsigned long		 caps,
                            int				 enable_stats);

extern void avi_capture_destroy(struct avi_capture_context *ctx);

/* Configure the capture interface and get the video timings. This has to be
 * called after any change to the input stream timings or the capture
 * configuration in ctx.
 *
 * This call will block for a few frames' worth of time. */
extern int avi_capture_prepare(struct avi_capture_context *ctx);

/* After a call to avi_capture_get_measures this function can be used to get the
 * resolution of the video. */
extern void avi_capture_resolution(const struct avi_capture_context *ctx,
                                   unsigned *width,
                                   unsigned *height);

extern int avi_capture_set_crop(struct avi_capture_context *ctx,
                                struct avi_capture_crop    *crop);

extern int avi_capture_try_format(const struct avi_capture_context *ctx,
                                  struct avi_segment_format *fmt);

extern int avi_capture_set_format(struct avi_capture_context *ctx,
                                  struct avi_segment_format *fmt);

extern int avi_capture_streamon (struct avi_capture_context *ctx);
extern int avi_capture_streamoff(struct avi_capture_context *ctx);

/* Resume capture after suspend to ram */
extern int avi_capture_resume(struct avi_capture_context *ctx);

extern void avi_capture_configure_interface(struct avi_capture_context *ctx);

extern int avi_capture_prepare_timings(struct avi_capture_context *ctx,
                                       unsigned width, unsigned height);

struct avi_multicapt {
  struct avi_capture_context *ctx;
  dma_addr_t fb_offset_addr;
};
#endif /* _AVI_CAPTURE_H_ */
