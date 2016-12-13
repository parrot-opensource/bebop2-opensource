#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/device.h>

#include "video/avi_pixfmt.h"
#include "avi_capture.h"

#ifdef DEBUG
#define dprintk(ctx, format_, args...) \
	dev_dbg((ctx)->dev, "%s: " format_, __func__, ##args)

#else /* DEBUG */
#define dprintk(ctx_, format_, args...) (void)ctx_
#endif /* DEBUG */

int avi_capture_init(struct avi_capture_context *ctx,
                     struct device		*dev,
                     int			 major,
                     unsigned long		 caps,
                     int			 enable_stats)
{
	unsigned long	cam_caps;
	int		ret = 0;

	memset(ctx, 0, sizeof(*ctx));

	/* Extract the camera number from the capabilities */
	cam_caps = caps & AVI_CAPS_CAM_ALL;
	caps    &= ~cam_caps;

	ctx->dev = dev;

	if (!cam_caps) {
		/* We need a camera */
		dprintk(ctx, "missing camera capability\n");
		ret = -EINVAL;
		goto no_cam;
	}

	/* Build the camera segment */
	ctx->cam_segment = avi_segment_build(&cam_caps, "cam", major, -1, dev);
	if (IS_ERR(ctx->cam_segment)) {
		dprintk(ctx, "cam_segment build failed\n");
		ret = PTR_ERR(ctx->cam_segment);
		goto no_cam_segment;
	}

	ctx->cam_node = avi_segment_get_node(ctx->cam_segment,
	                                     AVI_CAPS_CAM_ALL);
	BUG_ON(ctx->cam_node == NULL);

	/* Build the DMA out segment. */
	caps |= AVI_CAP_DMA_OUT;

	ctx->dma_segment = avi_segment_build(&caps, "cam_dma", major, -1, dev);
	if (IS_ERR(ctx->dma_segment)) {
		dprintk(ctx, "dma_segment build failed\n");
		ret = PTR_ERR(ctx->dma_segment);
		goto no_dma_segment;
	}

	if (enable_stats) {
		caps = avi_isp_bayer_get_stats_cap(ctx->dma_segment);

		if (!caps) {
			dprintk(ctx, "can't find stats cap\n");
			ret = -ENODEV;
			goto no_stats_cap;
		}

		caps |= AVI_CAP_DMA_OUT;

		ctx->stats_segment = avi_segment_build(&caps,
						       "stats",
						       major,
						       -1,
						       dev);
		if (IS_ERR(ctx->stats_segment)) {
			dprintk(ctx, "stats_segment build failed\n");
			ret = PTR_ERR(ctx->stats_segment);
			ctx->stats_segment = NULL;
			goto no_stats_segment;
		}

		ctx->stats_fifo = avi_segment_get_node(ctx->stats_segment,
						       AVI_CAP_DMA_OUT);
		BUG_ON(!ctx->stats_fifo);
	}

	ctx->cam_segment->priv = ctx;
	ctx->dma_segment->priv = ctx;

	init_waitqueue_head(&ctx->waitq);
	atomic_set(&ctx->streaming, 0);

	avi_segment_enable_irq(ctx->dma_segment);

	init_timer(&ctx->timer);
	spin_lock_init(&ctx->lock);

	return 0;

 no_stats_segment:
 no_stats_cap:
	avi_segment_teardown(ctx->dma_segment);
 no_dma_segment:
	avi_segment_teardown(ctx->cam_segment);
 no_cam_segment:
 no_cam:
	return ret;
}
EXPORT_SYMBOL(avi_capture_init);

void avi_capture_destroy(struct avi_capture_context *ctx)
{
	/* interrupt is registered on dma_segment. Disable it first
	   If not we can do the streamoff on cpu1, disable stat
	   and got an irq on cpu0.
	 */
	avi_segment_teardown(ctx->dma_segment);
	avi_segment_teardown(ctx->cam_segment);
	if (ctx->stats_segment)
		avi_segment_teardown(ctx->stats_segment);
}
EXPORT_SYMBOL(avi_capture_destroy);

static enum avi_field avi_capture_get_field(struct avi_capture_context *ctx)
{
	union avi_cam_status status;

	if (!ctx->interlaced)
		return AVI_NONE_FIELD;

	/* XXX interlaced with external synchros not yet supported (software
	 * limitation) */
	BUG_ON(!ctx->interface.itu656);

	status = avi_cam_get_status(ctx->cam_node);

	return status.field ? AVI_BOT_FIELD : AVI_TOP_FIELD;
}

static struct avi_capture_timings *
avi_capture_field_timings(struct avi_capture_context *ctx, enum avi_field f)
{
	enum avi_field	cur_field;

	if (ctx->interlaced) {
		cur_field = avi_capture_get_field(ctx);

		switch(f) {
		case AVI_TOP_FIELD:
			return &ctx->timings_top;
		case AVI_BOT_FIELD:
			return &ctx->timings_bot;
		default:
			BUG();
		}
	}

	return &ctx->timings_top;
}

static unsigned avi_capture_timings_init(struct avi_segment *,
                                         enum avi_field *);

/* Default measure-to-timings function, should work fine for standard-compliant
 * 656 timings. Assumes ivs and ihs are set in cam interface. */
static void avi_capture_656_to_timings(struct avi_capture_timings  *t,
				       struct avi_cam_measure_regs *m)
{
	/* We always invert the polarity of the synchronisation signals,
	 * therefore the active video is mesured by the cam module as a sync
	 * pulse.
	 *
	 * We do this in order to be able to capture stable video even if the
	 * input stream has variable blanking durations. */
	t->ht.hactive_on  = m->hactive_on;
	t->ht.hactive_off = m->hsync_off;
	t->vt.vactive_on  = m->vactive_on;
	t->vt.vactive_off = m->vsync_voff;
}

/* Default measure-to-timings function, should work fine for standard-compliant
 * non-656 timings. Assumes ivs and ihs are set in cam interface. */
static void avi_capture_to_timings(struct avi_capture_timings  *t,
				       struct avi_cam_measure_regs *m)
{
	t->ht.hactive_on  = m->hactive_on;
	t->ht.hactive_off = m->hsync_off;
	t->vt.vactive_on  = m->vactive_on+1;
	t->vt.vactive_off = m->vsync_voff+1;
}

/* Step 3: We compare the timings retreived in step 2 with the current mesured
 * values. If they're the same we're done and wake up the calling
 * task. Otherwise we go back to step 1 and try again. */
static unsigned avi_capture_timings_validate(struct avi_segment *s,
                                             enum avi_field *field)
{
	struct avi_capture_context	*ctx = s->priv;
	struct avi_capture_timings	*expected;
	enum avi_field			 f = avi_capture_get_field(ctx);
	struct avi_cam_measure_regs	 m;
	struct avi_capture_timings       t;

	expected = avi_capture_field_timings(ctx, f);

	avi_cam_get_measure(ctx->cam_node, &m);

	ctx->measure_to_timings(&t, &m);

	if (memcmp(expected, &t, sizeof(t)) == 0) {
		/* XXX for interlaced mode I should maybe check the consistency
		 * of both timings ? */
		atomic_set(&ctx->timings_valid, 1);
		avi_segment_register_irq(ctx->cam_segment, NULL);
		wake_up_interruptible(&ctx->waitq);
		return 0;
	}

	/* Got inconsistent timings */
	if (printk_ratelimit()) {
		dev_warn(ctx->dev,
		         "got inconsistent timings during bootstrap\n");

#define CHECK_TIMING(_timing)  do {                                    \
		if (expected->_timing != t._timing)                    \
			dev_warn(ctx->dev,                             \
			         #_timing ": expected %u, got %u\n",   \
			         expected->_timing, t._timing);        \
		} while(0);

		/* Display the inconsistencies */
		CHECK_TIMING(ht.hactive_on);
		CHECK_TIMING(ht.hactive_off);
		CHECK_TIMING(vt.vactive_on);
		CHECK_TIMING(vt.vactive_off);
#undef CHECK_TIMING
	}

	/* Restart the whole bootstrapping procedure to try and get clean
	 * timings */
	return avi_capture_timings_init(s, field);
}

/* Step 2: We mesure the timings and store them in the context. Wait for two frames
 * if we're interlaced in order to get the timings for both fields. */
static unsigned avi_capture_timings_measure(struct avi_segment *s,
                                            enum avi_field *field)
{
	struct avi_capture_context *ctx  = s->priv;
	enum avi_field	            f    = avi_capture_get_field(ctx);
	struct avi_capture_timings *t;
	struct avi_cam_measure_regs m;

	avi_cam_get_measure(ctx->cam_node, &m);

	t = avi_capture_field_timings(ctx, f);

	ctx->measure_to_timings(t, &m);

	if (ctx->timings_top.vt.vactive_off &&
	    (!ctx->interlaced || ctx->timings_bot.vt.vactive_off))
		/* Switch to validation mode */
		avi_segment_register_irq(ctx->cam_segment,
		                         &avi_capture_timings_validate);

	return 0;
}

/* Step 1: At this point the measurements are probably garbage since it's likely
 * we've been enabled in the middle of a frame. We'll start measuring starting
 * with the next frame. */
static unsigned avi_capture_timings_init(struct avi_segment *s,
                                         enum avi_field *field)
{
	struct avi_capture_context *ctx = s->priv;

	/* Mark the timings as invalid */
	ctx->timings_top.vt.vactive_off = 0;
	ctx->timings_bot.vt.vactive_off = 0;

	avi_segment_register_irq(ctx->cam_segment,
	                         &avi_capture_timings_measure);

	return 0;
}

int avi_capture_prepare_timings(struct avi_capture_context *ctx,
                                unsigned width, unsigned height)
{
	struct avi_segment_format sensor_fmt = {
		.interlaced		     = ctx->interlaced,
		.colorspace		     = ctx->capture_cspace,
		.width                       = width,
		.height                      = height
	};
	struct avi_cam_measure_regs m = {
		.hactive_on = 0,
		.hsync_off  = width,
		.vactive_on = 0,
		.vsync_voff = height,
	};

	if (ctx->interface.itu656) {
		m.hactive_on = 2;
		m.hsync_off += 2;
		m.vactive_on = 1;
		m.vsync_voff += 1;
	}

	avi_capture_configure_interface(ctx);

	ctx->crop.x	 = 0;
	ctx->crop.y	 = 0;
	ctx->crop.width  = width;
	ctx->crop.height = height;

	if(ctx->interlaced)
		m.vsync_voff = height/2 + height%2
			+ (ctx->interface.itu656 ? 1 : 0);

	ctx->measure_to_timings(&ctx->timings_top, &m);

	if(ctx->interlaced) {
		m.vsync_voff -= height % 2;
		ctx->measure_to_timings(&ctx->timings_bot, &m);
	}

	return avi_segment_set_format(ctx->cam_segment, &sensor_fmt, &sensor_fmt);
}
EXPORT_SYMBOL(avi_capture_prepare_timings);

/* Step 0: We ignore the first interrupt because the measures are not yet valid
 * (we always get the mesures of the preceding frame).*/
static unsigned avi_capture_timings_sync(struct avi_segment *s,
                                         enum avi_field *field)
{
	struct avi_capture_context *ctx = s->priv;

	avi_segment_register_irq(ctx->cam_segment,
	                         &avi_capture_timings_init);

	return 0;
}

void avi_capture_configure_interface(struct avi_capture_context *ctx)
{
	struct avi_cam_registers regs = {{{ 0 }}};

	regs.itsource.done_en = 1;

	regs.interface = ctx->interface;

	if (ctx->measure_to_timings == NULL) {
		/* 656-to-timings needs inversed syncs */
		if (ctx->interface.itu656) {
			ctx->measure_to_timings = &avi_capture_656_to_timings;
			regs.interface.ivs = 1;
			regs.interface.ihs = 1;
		} else {
			ctx->measure_to_timings = &avi_capture_to_timings;
			regs.interface.ivs = !ctx->interface.ivs;
			regs.interface.ihs = !ctx->interface.ihs;
		}
	}

	avi_cam_set_registers(ctx->cam_node, &regs);
}
EXPORT_SYMBOL(avi_capture_configure_interface);

int avi_capture_prepare(struct avi_capture_context *ctx)
{
	struct avi_segment_format sensor_fmt = {
		.interlaced		     = ctx->interlaced,
		.colorspace		     = ctx->capture_cspace,
	};
	unsigned long             timeout;
	int                       ret;

	avi_capture_configure_interface(ctx);

	atomic_set(&ctx->timings_valid, 0);

	avi_segment_register_irq(ctx->cam_segment, &avi_capture_timings_sync);

	avi_segment_enable_irq(ctx->cam_segment);

	avi_segment_activate(ctx->cam_segment);

	/* Force the activation of the camera alone */
	avi_apply_node(ctx->cam_node);

	timeout = ctx->timeout_jiffies;
	if (timeout == 0)
		/* default to 1 second */
		timeout = HZ;

	/* Give us some headroom since we'll have to capture a few frames to get
	 * the timings right. I chose this value completely arbitrarily, but it
	 * should be at least 3 since we'll need at least 3 frames before the
	 * timings are validated. The rest is in case  */
	timeout *= 4;

	/* Wait for the notification by the interrupt handler that the timings
	 * are valid. */
	ret = wait_event_interruptible_timeout(ctx->waitq,
	                                       atomic_read(&ctx->timings_valid),
	                                       timeout);

	/* Cleanup everything related to the sensor's interrupt. If we start a
	 * capture after that we'll only care about the DMA interrupt */
	avi_segment_deactivate(ctx->cam_segment);
	/* Force the deactivation of the camera alone */
	avi_apply_node(ctx->cam_node);

	avi_segment_disable_irq(ctx->cam_segment);
	avi_segment_register_irq(ctx->cam_segment, NULL);

	if (ret == 0)
		return -ETIMEDOUT;

	if (ret < 0)
		return ret;

	avi_capture_resolution(ctx, &sensor_fmt.width, &sensor_fmt.height);
	/* Default crop is the full frame */
	ctx->crop.x	 = 0;
	ctx->crop.y	 = 0;
	ctx->crop.width  = sensor_fmt.width;
	ctx->crop.height = sensor_fmt.height;

	return avi_segment_set_format(ctx->cam_segment, &sensor_fmt, &sensor_fmt);
}
EXPORT_SYMBOL(avi_capture_prepare);

void avi_capture_resolution(const struct avi_capture_context *ctx,
                            unsigned *width,
                            unsigned *height)
{
	if (width)
		*width = ctx->timings_top.ht.hactive_off -
			ctx->timings_top.ht.hactive_on;

	if (height) {
		*height = ctx->timings_top.vt.vactive_off -
			ctx->timings_top.vt.vactive_on;

		if (ctx->interlaced)
			*height += ctx->timings_bot.vt.vactive_off -
				ctx->timings_bot.vt.vactive_on;
	}
}
EXPORT_SYMBOL(avi_capture_resolution);

int avi_capture_set_crop(struct avi_capture_context *ctx,
                         struct avi_capture_crop    *crop)
{
	unsigned			max_x = crop->x + crop->width;
	unsigned			max_y = crop->y + crop->height;
	unsigned			width;
	unsigned			height;
	struct avi_segment_format	sensor_fmt;
	int				ret;

	avi_capture_resolution(ctx, &width, &height);

	if (max_x > width || max_y > height)
		/* The crop window is outside of the active video */
		return -ERANGE;

	/* Force the cropping to keep the field ordering and have identical top
	 * and bottom heights */
	if (ctx->interlaced && (crop->y % 2 || crop->height % 2))
		return -EINVAL;

	if (crop->width  == ctx->crop.width &&
	    crop->height == ctx->crop.height) {
		/* We only need to update the crop coordinates, the segment
		 * formats is unchanged */
		ctx->crop = *crop;

		return 0;
	}

	/* If we reach this point it means we change the crop window
	 * dimensions */

	if (atomic_read(&ctx->streaming))
		/* We can't change the capture dimensions while streaming */
		return -EBUSY;

	avi_segment_get_input_format(ctx->cam_segment, &sensor_fmt);

	sensor_fmt.width  = crop->width;
	sensor_fmt.height = crop->height;

	ret = avi_segment_set_format(ctx->cam_segment,
	                             &sensor_fmt,
	                             &sensor_fmt);
	if (ret)
		return ret;

	ctx->crop = *crop;

	return 0;
}
EXPORT_SYMBOL(avi_capture_set_crop);

int avi_capture_try_format(const struct avi_capture_context *ctx,
                           struct avi_segment_format *fmt)
{
	struct avi_segment_format sensor_fmt;

	avi_segment_get_input_format(ctx->cam_segment, &sensor_fmt);

	return avi_segment_try_format(ctx->dma_segment,
	                              &sensor_fmt,
	                              fmt,
	                              &ctx->dma_segment->layout);
}
EXPORT_SYMBOL(avi_capture_try_format);

int avi_capture_set_format(struct avi_capture_context *ctx,
                           struct avi_segment_format *fmt)
{
	struct avi_segment_format sensor_fmt;

	avi_segment_get_input_format(ctx->cam_segment, &sensor_fmt);

	return avi_segment_set_format(ctx->dma_segment,
	                              &sensor_fmt,
	                              fmt);
}
EXPORT_SYMBOL(avi_capture_set_format);

static void avi_capture_set_timings(struct avi_capture_context *ctx,
                                    struct avi_capture_timings *t)
{
	union avi_cam_h_timing		 ht;
	union avi_cam_v_timing		 vt;
	const struct avi_capture_crop	*crop = &ctx->crop;

	ht.hactive_on = t->ht.hactive_on + crop->x;
	ht.hactive_off = ht.hactive_on + crop->width;

	if (ctx->interlaced) {
		vt.vactive_on = t->vt.vactive_on + crop->y / 2;
		vt.vactive_off = vt.vactive_on + crop->height / 2;
	} else {
		vt.vactive_on = t->vt.vactive_on + crop->y;
		vt.vactive_off = vt.vactive_on + crop->height;
	}

	avi_cam_set_timings(ctx->cam_node, ht, vt);
}

static const struct avi_dma_buffer avi_capture_null_buffer = {
	.plane0.dma_addr = 0,
	.plane1.dma_addr = 0,
	.status		 = AVI_BUFFER_READY,
	.priv		 = NULL,
};

static inline int avi_capture_is_null_buffer(struct avi_dma_buffer *buf)
{
	return !buf->plane0.dma_addr;
}

static inline int avi_capture_stats_enabled(struct avi_capture_context *ctx)
{
	return ctx->stats_segment != NULL;
}

static void avi_capture_update_period(long           *period_us,
                                      struct timeval *cur_date,
                                      struct timeval *last_frame)
{
	long cur_period_us;

	if (last_frame->tv_sec == 0 && last_frame->tv_usec == 0) {
		/* Our first frame, can't compute the period yet */
		*last_frame = *cur_date;
		return;
	}

	cur_period_us = (cur_date->tv_sec  - last_frame->tv_sec) * USEC_PER_SEC
		      +  cur_date->tv_usec - last_frame->tv_usec;

	if (*period_us) {
		long delta = cur_period_us - *period_us;

		/* Use an exponential moving average for the framerate. The
		 * shift value says how "fast" the mean changes when new samples
		 * come in, so basically the bigger it is the smoother will be
		 * the variation, */
		*period_us += delta >> 4;
	} else {
		/* We start the mean at the first value */
		*period_us = cur_period_us;
	}

	*last_frame = *cur_date;
}

static inline void avi_capture_get_time(struct timeval *date)
{
	struct timespec ts;

	ktime_get_ts(&ts);
	date->tv_sec = ts.tv_sec;
	date->tv_usec = ts.tv_nsec / NSEC_PER_USEC;
}

static void avi_capture_handle_new_frame(struct avi_capture_context	*ctx,
                                         enum avi_field			*f,
                                         enum avi_dma_buffer_status      s)
{
	struct avi_dma_buffer	*buf   = ctx->capture_buffer;
	struct avi_dma_buffer	*stats = ctx->capture_stats;
	unsigned long		 flags;
	union avi_cam_status	 status;
	struct timeval		 cur_date;

	spin_lock_irqsave(&ctx->lock, flags);

	avi_capture_get_time(&cur_date);

	avi_capture_update_period(&ctx->dma_segment->stream_period_us,
				  &cur_date,
				  &ctx->last_frame);

	if (!avi_capture_is_null_buffer(buf))
		avi_capture_update_period(&ctx->dma_segment->period_us,
				          &cur_date,
				          &ctx->last_captured_frame);
	else
		ctx->dma_segment->frame_dropped++;

	*f = avi_capture_get_field(ctx);

	if (ctx->done && !avi_capture_is_null_buffer(buf)) {
		if (s == AVI_BUFFER_TIMEOUT) {
			buf->status = s;
		} else {
			status = avi_cam_get_status(ctx->cam_node);

			if (status.fof)
				/* An overflow occured during the capture */
				buf->status = AVI_BUFFER_ERROR;
			else
				buf->status = AVI_BUFFER_DONE;

			/* Make sure all flags are reset */
			avi_ack_node_irq(ctx->cam_node);
		}

		ctx->done(ctx, buf, *f);
	}

	/* Reset the buffer */
	*buf = avi_capture_null_buffer;

	/* The configured_buffer is not receiving the captured frame, so we swap
	 * them around */
	ctx->capture_buffer    = ctx->configured_buffer;
	ctx->configured_buffer = buf;

	/* Stats handling. We never actually enable the stats_fifo interrupt at
	 * the CFG level, we just use the itflg flag to see if they were
	 * correctly written in memory. This way we avoid having two interrupts
	 * per frame at the CPU level since we synchronize ourselves on the
	 * video output. */
	if (avi_capture_stats_enabled(ctx)) {
		avi_capture_update_period(&ctx->stats_segment->stream_period_us,
					  &cur_date,
					  &ctx->last_stats_frame);

		if (!avi_capture_is_null_buffer(stats))
			avi_capture_update_period(
					       &ctx->stats_segment->period_us,
					       &cur_date,
					       &ctx->last_captured_stats_frame);
		else
			ctx->stats_segment->frame_dropped++;

		if (ctx->stats_done && !avi_capture_is_null_buffer(stats)) {

			if (avi_node_get_irq_flag(ctx->stats_fifo))
				stats->status = AVI_BUFFER_DONE;
			else
				/* Looks like the stats are not ready... */
				stats->status = AVI_BUFFER_ERROR;

			ctx->stats_done(ctx, stats, *f);
		}

		/* Make sure we never leave the IT flag raised */
		avi_ack_node_irq(ctx->stats_fifo);

		/* Same shuffling as above, this time for the stat buffers */
		*stats		      = avi_capture_null_buffer;
		ctx->capture_stats    = ctx->configured_stats;
		ctx->configured_stats = stats;
	}

	if (ctx->timeout_jiffies)
		mod_timer(&ctx->timer, jiffies + ctx->timeout_jiffies);

	spin_unlock_irqrestore(&ctx->lock, flags);
}

#ifndef CONFIG_AVICAM_INSTANT_STREAMOFF
/* Final interrupt when the last frame is done capturing */
static unsigned avi_capture_stopped(struct avi_segment *s, enum avi_field *f)
{
	struct avi_capture_context *ctx = s->priv;

	/* Last frame */
	avi_capture_handle_new_frame(ctx, f, AVI_BUFFER_DONE);

	/* Make sure we apply everything on stats FIFO */
	if (avi_capture_stats_enabled(ctx)) {
		struct avi_node *dma_fifo;

		dma_fifo = avi_segment_get_node(ctx->stats_segment,
						AVI_CAP_DMA_OUT);
		BUG_ON(dma_fifo == NULL);
		avi_apply_node(dma_fifo);
	}

	atomic_set(&ctx->streaming, 0);
	wake_up(&ctx->waitq);

	return 0;
}

/* Interrupt callback for disabling the stream */
static unsigned avi_capture_stop(struct avi_segment *s, enum avi_field *f)
{
	struct avi_capture_context *ctx = s->priv;

	avi_capture_handle_new_frame(ctx, f, AVI_BUFFER_DONE);

	avi_segment_deactivate(ctx->dma_segment);
	avi_segment_deactivate(ctx->cam_segment);

	if (avi_capture_stats_enabled(ctx))
		avi_segment_deactivate(ctx->stats_segment);

	/* We should receive a final interrupt when the last frame is
	 * captured. At that moment the deactivation will take effect. */
	avi_segment_register_irq(ctx->dma_segment, &avi_capture_stopped);

	return 0;
}
#endif

/* Interrupt callback while streaming */
static unsigned avi_capture_interrupt(struct avi_segment *s, enum avi_field *f)
{
	struct avi_capture_context *ctx = s->priv;

	avi_capture_handle_new_frame(ctx, f, AVI_BUFFER_DONE);

	if (ctx->interlaced) {
		/* We're preparing for the next frame to be captured (not the
		 * one being captured right now) so it's the same field as the
		 * previous frame's field. Yes, it's complicated. */
		avi_capture_set_timings(ctx, avi_capture_field_timings(ctx, *f));
	}

	/* Ask the client to provide us with a new buffer */
	if (ctx->next)
		ctx->next(ctx, ctx->configured_buffer, *f);

	avi_segment_set_output_buffer(ctx->dma_segment, ctx->configured_buffer);

	if (avi_capture_stats_enabled(ctx)) {
		if (ctx->stats_next)
			ctx->stats_next(ctx, ctx->configured_stats, *f);

		avi_segment_set_output_buffer(ctx->stats_segment,
					      ctx->configured_stats);
	}

	avi_cam_run(ctx->cam_node);

	return 0;
}

/* Triggered by timer */
static void avi_capture_timeout(unsigned long data)
{
	struct avi_capture_context	*ctx;
	enum avi_field			 dummy_field;
	struct avi_node                 *dma_fifo;

	ctx = (struct avi_capture_context *)data;
	if (!atomic_read(&ctx->streaming))
		/* We're probably in the process of shutting down, ignore this
		 * timeout */
		return;

	dev_warn(ctx->dev, "timeout, dropping frame...\n");

	/* If the AVI locked itself because of a corrupted frame for instance,
	 * attempt to restart the process by forcing the camera run */
	avi_cam_freerun(ctx->cam_node);

	/* And then we clean the whole chain by forcing a clear */
	if (avi_capture_stats_enabled(ctx)) {
		dma_fifo = avi_segment_get_node(ctx->stats_segment,
						AVI_CAP_DMA_OUT);
		BUG_ON(dma_fifo == NULL);
		avi_fifo_force_clear(dma_fifo);
	}

	dma_fifo = avi_segment_get_node(ctx->dma_segment, AVI_CAP_DMA_OUT);
	BUG_ON(dma_fifo == NULL);
	avi_fifo_force_clear(dma_fifo);

	avi_capture_handle_new_frame(ctx, &dummy_field, AVI_BUFFER_TIMEOUT);
}

static int avi_capture_stats_streamon(struct avi_capture_context *ctx)
{
	/* Hardcoded stats format. */
	struct avi_segment_format vl_fmt = {
		.width		  = AVI_CAPTURE_STATS_WIDTH,
		.height		  = AVI_CAPTURE_STATS_HEIGHT,
		.interlaced	  = 0,
		.colorspace	  = 0,
	};
	struct avi_segment_format dma_fmt = vl_fmt;
	struct avi_node          *chain_bayer;
	int                       ret;

	if (!avi_capture_stats_enabled(ctx))
		/* Nothing to do */
		return 0;

	chain_bayer = avi_segment_get_node(ctx->dma_segment,
					   AVI_CAP_ISP_CHAIN_BAYER);

	BUG_ON(!chain_bayer);
	avi_statistics_bayer_configure(chain_bayer,
				       ctx->crop.width,
				       ctx->crop.height,
				       AVI_CAPTURE_THUMB_STATS_WIDTH,
				       AVI_CAPTURE_THUMB_STATS_HEIGHT);

	dma_fmt			 = vl_fmt;
	dma_fmt.pix_fmt		 = AVI_PIXFMT_BGRA8888;
	dma_fmt.plane0.line_size = dma_fmt.width * 4;

	ret = avi_segment_set_format(ctx->stats_segment, &vl_fmt, &dma_fmt);
	if (ret)
		return ret;

	/* Initialize buffer rotation */
	ctx->configured_stats = &ctx->stats[0];
	ctx->capture_stats    = &ctx->stats[1];

	*ctx->configured_stats = avi_capture_null_buffer;
	*ctx->capture_stats    = avi_capture_null_buffer;

	avi_segment_set_output_buffer(ctx->stats_segment,
				      ctx->configured_stats);

	avi_segment_activate(ctx->stats_segment);

	return 0;
}

int avi_capture_streamon(struct avi_capture_context *ctx)
{
	enum avi_field	dummy;
	int		ret;

	if (atomic_read(&ctx->streaming))
		return -EBUSY;

	ret = avi_segment_connect(ctx->cam_segment, ctx->dma_segment, -1);
	if (ret)
		goto connect_failed;

	/* We can configure the CAM module once and for all for progressive
	 * mode. We arbitrarily select the top config for interlaced input. */
	avi_capture_set_timings(ctx, &ctx->timings_top);

	/* Initialize buffer rotation */
	ctx->configured_buffer = &ctx->buffers[0];
	ctx->capture_buffer    = &ctx->buffers[1];

	*ctx->configured_buffer = avi_capture_null_buffer;
	*ctx->capture_buffer    = avi_capture_null_buffer;

	avi_segment_set_output_buffer(ctx->dma_segment, ctx->capture_buffer);

	avi_cam_run(ctx->cam_node);

	atomic_set(&ctx->streaming, 1);

	avi_segment_register_irq(ctx->dma_segment, &avi_capture_interrupt);

	ret = avi_capture_stats_streamon(ctx);
	if (ret)
		goto stats_streamon_failed;

	ret = avi_segment_activate(ctx->cam_segment);
	if (ret)
		goto cam_activate_failed;

	ret = avi_segment_activate(ctx->dma_segment);
	if (ret)
		goto dma_activate_failed;

	setup_timer(&ctx->timer, avi_capture_timeout, (long unsigned)ctx);

	/* The capture has started, we manually call the interrupt routine to
	 * let it configure the first frame to capture. */
	avi_capture_interrupt(ctx->dma_segment, &dummy);

	return 0;

 dma_activate_failed:
	avi_segment_deactivate(ctx->cam_segment);
 cam_activate_failed:
	avi_segment_disconnect(ctx->cam_segment, ctx->dma_segment);
 connect_failed:
 stats_streamon_failed:
	atomic_set(&ctx->streaming, 0);

	return ret;
}
EXPORT_SYMBOL(avi_capture_streamon);

int avi_capture_streamoff(struct avi_capture_context *ctx)
{
	unsigned long	timeout;
	int		ret = -1;

	if (atomic_read(&ctx->streaming) == 0)
		return -EINVAL;

	timeout = ctx->timeout_jiffies;
	ctx->timeout_jiffies = 0;
	del_timer(&ctx->timer);

#ifndef CONFIG_AVICAM_INSTANT_STREAMOFF
	avi_segment_register_irq(ctx->dma_segment, &avi_capture_stop);

	if (timeout == 0)
		/* Default to one second */
		timeout = HZ;

	ret = wait_event_timeout(ctx->waitq,
	                         !atomic_read(&ctx->streaming),
	                         timeout);
#endif

	if (ret > 0) {
		/* The interrupt handler deactivated everything and woke us
		 * up */
		ret = 0;
	} else {
		struct avi_node *dma_fifo;

		if (ret == 0)
			ret = -ETIMEDOUT;

		/* The IT handler wasn't called so force the deactivation from
		 * here. This can happen if the input stream stopped before
		 * us. */

		atomic_set(&ctx->streaming, 0);

		avi_segment_deactivate(ctx->dma_segment);
		avi_segment_deactivate(ctx->cam_segment);
		if (avi_capture_stats_enabled(ctx)) {
			avi_segment_deactivate(ctx->stats_segment);

			dma_fifo = avi_segment_get_node(ctx->stats_segment,
							AVI_CAP_DMA_OUT);
			BUG_ON(dma_fifo == NULL);
			avi_apply_node(dma_fifo);
		}

		/* We apply the new configuration to make sure the fifo will not
		 * send random garbage if a stream comes up */
		dma_fifo = avi_segment_get_node(ctx->dma_segment, AVI_CAP_DMA_OUT);
		BUG_ON(dma_fifo == NULL);

		/* Because there is no input stream, no clear will be sent
		 * 'naturally' so we need to force it */
		avi_fifo_force_clear(dma_fifo);

		avi_apply_node(dma_fifo);
	}

	avi_segment_disconnect(ctx->cam_segment, ctx->dma_segment);

	return ret;
}
EXPORT_SYMBOL(avi_capture_streamoff);

int avi_capture_resume(struct avi_capture_context *ctx)
{
	avi_capture_configure_interface(ctx);

	if (avi_capture_stats_enabled(ctx)) {
		avi_segment_set_output_buffer(ctx->stats_segment,
					      ctx->configured_stats);
		avi_segment_activate(ctx->stats_segment);
	}

	avi_segment_set_output_buffer(ctx->dma_segment, ctx->capture_buffer);
	avi_capture_set_timings(ctx, &ctx->timings_top);
	avi_cam_run(ctx->cam_node);
	avi_segment_enable_irq(ctx->dma_segment);

	avi_segment_activate(ctx->cam_segment);
	avi_segment_activate(ctx->dma_segment);

	return 0;
}
EXPORT_SYMBOL(avi_capture_resume);
