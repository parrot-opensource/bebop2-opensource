/**
 * @file linux/drivers/parrot/media/video/avi_r2r.c
 *  Parrot AVI RAM to RAM intra-kernel driver.
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * @author     didier.leymarie.ext@parrot.com
 * @date       2014-02-06
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

#include "avi_r2r_core.h"

#include "avi_stats.h"

/**
 * command line parameters
 */

#ifdef CONFIG_AVI_DEBUG
/* size of timings saved */
#define _NR_TIMINGS_SAVED_DEFAULT (PAGE_SIZE / sizeof(struct timings))
static unsigned int nr_timings_saved = _NR_TIMINGS_SAVED_DEFAULT;
module_param(nr_timings_saved, uint, S_IRUGO);
MODULE_PARM_DESC(nr_timings_saved, "number of timings saved");
#endif /* CONFIG_AVI_DEBUG */

static int z_order = 1;
module_param(z_order, int, S_IRUGO);
MODULE_PARM_DESC(z_order, "Z order of plane managed by this driver : default 1");

#define _WORKS_POOL_DEFAULT (2 * PAGE_SIZE / sizeof(struct avi_r2r_work))
static unsigned int works_pool = _WORKS_POOL_DEFAULT;
module_param(works_pool, uint, S_IRUGO);
MODULE_PARM_DESC(works_pool, "size of works pool");

#define AVI_PIXFMT_PLANAR_DEFAULT AVI_PIXFMT_NV12
#define PLANAR_PIXEL_SIZE0 1
#define PLANAR_PIXEL_SIZE1 1
#define AVI_CSPACE_PLANAR_DEFAULT AVI_BT709_CSPACE

#define AVI_PIXFMT_RGB_DEFAULT AVI_PIXFMT_RGB565
#define RGB_PIXEL_SIZE0 2
#define RGB_PIXEL_SIZE1 0
#define AVI_CSPACE_RGB_DEFAULT AVI_RGB_CSPACE

#define AVI_PIXFMT_ISP_DEFAULT AVI_PIXFMT_BAYER_1X10_16
#define ISP_PIXEL_SIZE0 2
#define ISP_PIXEL_SIZE1 0
#define AVI_CSPACE_ISP_DEFAULT AVI_NULL_CSPACE

#define WIDTH_DEFAULT 2048
#define HEIGHT_DEFAULT 2048
#define ALPHA_DEFAULT 0xFF
#define BACKGROUND_RED_DEFAULT 0x00
#define BACKGROUND_GREEN_DEFAULT 0x00
#define BACKGROUND_BLUE_DEFAULT 0x00
#define BACKGROUND_DEFAULT (BACKGROUND_RED_DEFAULT<<16  | \
			    BACKGROUND_GREEN_DEFAULT<<8 | \
			    BACKGROUND_BLUE_DEFAULT) /* RGB */

static struct avi_r2r_user_format const planar_default_format = {
	.source = {
		.width = WIDTH_DEFAULT,
		.height = HEIGHT_DEFAULT,
		.colorspace = AVI_CSPACE_PLANAR_DEFAULT,
		.pix_fmt = AVI_PIXFMT_PLANAR_DEFAULT,
		.interlaced = 0,
		.plane0 = {
			.line_size = WIDTH_DEFAULT*PLANAR_PIXEL_SIZE0
		},
		.plane1 = {
			.line_size = WIDTH_DEFAULT*PLANAR_PIXEL_SIZE1
		},
	},
	.target = {
		.width = WIDTH_DEFAULT,
		.height = HEIGHT_DEFAULT,
		.colorspace = AVI_CSPACE_PLANAR_DEFAULT,
		.pix_fmt = AVI_PIXFMT_PLANAR_DEFAULT,
		.interlaced = 0,
		.plane0 = {
			.line_size = WIDTH_DEFAULT*PLANAR_PIXEL_SIZE0
		},
		.plane1 = {
			.line_size = WIDTH_DEFAULT*PLANAR_PIXEL_SIZE1
		},
	},
	.compose= {
		.width = 0,
		.height = 0,
		.colorspace = 0,
		.pix_fmt = AVI_PIXFMT_INVALID,
		.interlaced = 0,
		.plane0 = {
			.line_size = 0
		},
		.plane1 = {
			.line_size = 0
		},
		},
	.use_compose = 0,
	.layout = {
		.x =0,
		.y =0,
		.hidden = 0,
		.alpha = ALPHA_DEFAULT,
	},
	.background = BACKGROUND_DEFAULT
};

static struct avi_r2r_user_format const rgb_default_format = {
	.source = {
		.width = WIDTH_DEFAULT,
		.height = HEIGHT_DEFAULT,
		.colorspace = AVI_CSPACE_RGB_DEFAULT,
		.pix_fmt = AVI_PIXFMT_RGB_DEFAULT,
		.interlaced = 0,
		.plane0 = {
			.line_size = WIDTH_DEFAULT*RGB_PIXEL_SIZE0
		},
		.plane1 = {
			.line_size = WIDTH_DEFAULT*RGB_PIXEL_SIZE1
		},
	},
	.target = {
		.width = WIDTH_DEFAULT,
		.height = HEIGHT_DEFAULT,
		.colorspace = AVI_CSPACE_RGB_DEFAULT,
		.pix_fmt = AVI_PIXFMT_RGB_DEFAULT,
		.interlaced = 0,
		.plane0 = {
			.line_size = WIDTH_DEFAULT*RGB_PIXEL_SIZE0
		},
		.plane1 = {
			.line_size = WIDTH_DEFAULT*RGB_PIXEL_SIZE1
		},
	},
	.compose= {
		.width = 0,
		.height = 0,
		.colorspace = 0,
		.pix_fmt = AVI_PIXFMT_INVALID,
		.interlaced = 0,
		.plane0 = {
			.line_size = 0
		},
		.plane1 = {
			.line_size = 0
		},
		},
	.use_compose = 0,
	.layout = {
		.x =0,
		.y =0,
		.hidden = 0,
		.alpha = ALPHA_DEFAULT,
	},
	.background = BACKGROUND_DEFAULT
};

static struct avi_r2r_user_format const isp_default_format = {
	.source = {
		.width = WIDTH_DEFAULT,
		.height = HEIGHT_DEFAULT,
		.colorspace = AVI_CSPACE_ISP_DEFAULT,
		.pix_fmt = AVI_PIXFMT_ISP_DEFAULT,
		.interlaced = 0,
		.plane0 = {
			.line_size = WIDTH_DEFAULT*ISP_PIXEL_SIZE0
		},
		.plane1 = {
			.line_size = WIDTH_DEFAULT*ISP_PIXEL_SIZE1
		},
	},
	.target = {
		.width = WIDTH_DEFAULT,
		.height = HEIGHT_DEFAULT,
		.colorspace = AVI_CSPACE_PLANAR_DEFAULT,
		.pix_fmt = AVI_PIXFMT_PLANAR_DEFAULT,
		.interlaced = 0,
		.plane0 = {
			.line_size = WIDTH_DEFAULT*PLANAR_PIXEL_SIZE0
		},
		.plane1 = {
			.line_size = WIDTH_DEFAULT*PLANAR_PIXEL_SIZE1
		},
	},
	.compose= {
		.width = 0,
		.height = 0,
		.colorspace = 0,
		.pix_fmt = AVI_PIXFMT_INVALID,
		.interlaced = 0,
		.plane0 = {
			.line_size = 0
		},
		.plane1 = {
			.line_size = 0
		},
		},
	.use_compose = 0,
	.layout = {
		.x =0,
		.y =0,
		.hidden = 0,
		.alpha = ALPHA_DEFAULT,
	},
	.background = BACKGROUND_DEFAULT
};

static struct avi_r2r avi_r2r_data;

/**
 * User Context management
 */

static inline void avi_r2r__user_context_free(
				struct avi_r2r_user_context *user_context)
{
#ifdef CONFIG_AVI_DEBUG
	avi_r2r__user_destroy_debugfs(user_context);
#endif /* CONFIG_AVI_DEBUG */
	user_context->magic     = 0;
	user_context->magic_end = 0;
	kfree(user_context);
}

static struct avi_r2r_user_context * avi_r2r__user_context_alloc(struct avi_r2r *data)
{
	struct avi_r2r_user_context *new_ctx;
	new_ctx=kzalloc(sizeof(struct avi_r2r_user_context), GFP_KERNEL);
	if (new_ctx) {
		new_ctx->magic     = _AVI_R2R_CTX_ID_MAGIC;
		new_ctx->magic_end = _AVI_R2R_CTX_ID_MAGIC;
	}
	return new_ctx;
}

/**
 * Work pool management
 * a linked list of free work
 */
static int avi_r2r__work_pool_init(struct avi_r2r_channel_data *chan,
				   unsigned int pool_size)
{
	struct avi_r2r_work	*tmp;
	unsigned int		i;

	BUG_ON(!pool_size);

	chan->work_pool = kzalloc(sizeof(struct avi_r2r_work) * pool_size,
				  GFP_KERNEL);
	if (!chan->work_pool)
		return -ENOMEM;

	INIT_LIST_HEAD(&(chan->work_pool->list));
	chan->work_pool_size = pool_size;
	chan->work_pool_free = pool_size;

	for (i=0; i < pool_size; i++)
	{
		tmp = &chan->work_pool[i];
		list_add_tail(&(tmp->list),&(chan->work_pool->list));
	}

	return 0;
}

static struct avi_r2r_work *avi_r2r__work_pool_pull_slot(
					struct avi_r2r_channel_data *chan)
{
	struct avi_r2r_work	*tmp = NULL;
	struct list_head	*pos;

	if (list_empty(&(chan->work_pool->list)))
		return NULL;

	pos = chan->work_pool->list.next; /* first item in list */
	chan->work_pool_free --;

	list_del(pos);

	tmp = (struct avi_r2r_work *)pos;

	return tmp;
}

static void avi_r2r__work_pool_push_slot(struct avi_r2r_channel_data *chan,
					 struct avi_r2r_work *work)
{
	list_add_tail(&(work->list),&(chan->work_pool->list));
	chan->work_pool_free ++;
}
/**
 * Work queue management
 * a linked list is used as a FIFO
 */
static inline void avi_r2r__work_init(struct avi_r2r_work *work)
{
	work->magic = _AVI_R2R_WORK_MAGIC;
	ktime_get_ts(&work->ts_push);
}

#define BUG_ON_BAD_WORK(_work) BUG_ON((_work)->magic != _AVI_R2R_WORK_MAGIC || \
				      !(_work)->channel || \
				      !(_work)->user);

static inline void avi_r2r__work_cpy(struct avi_r2r_work *dst,
				     struct avi_r2r_work *src)
{
	dst->magic   = src->magic;
	dst->channel = src->channel;
	dst->user    = src->user;
	dst->source_buffer = src->source_buffer;
	dst->target_buffer = src->target_buffer;
	dst->stats_buffer = src->stats_buffer;
	dst->ts_push = src->ts_push;
}

static inline int avi_r2r__work_queue_empty(struct avi_r2r_work_queue *wq)
{
	return list_empty(&(wq->works.list));
}

static void avi_r2r__work_queue_init(struct avi_r2r_work_queue *wq)
{
	INIT_LIST_HEAD(&(wq->works.list));
	wq->works_nr = 0;
}

static void avi_r2r__work_queue_delete(struct avi_r2r_channel_data *chan)
{
	struct avi_r2r_work_queue	*wq = &chan->work_queue;
	struct avi_r2r_work		*tmp;
	struct list_head		*pos, *q;

	list_for_each_safe(pos, q, &(wq->works.list)) {
		tmp = list_entry(pos, struct avi_r2r_work, list);
		list_del(pos);
		avi_r2r__work_pool_push_slot(chan, tmp);
	}
}


static void avi_r2r__work_queue_push(struct avi_r2r_channel_data *chan,
				     struct avi_r2r_work *work)
{
	struct avi_r2r_work_queue	*wq = &chan->work_queue;
	struct avi_r2r_work		*tmp;

	tmp = avi_r2r__work_pool_pull_slot(chan);
	BUG_ON(!tmp);

	avi_r2r__work_init(work);

	avi_r2r__work_cpy(tmp, work);

	BUG_ON_BAD_WORK(tmp);

	list_add_tail(&(tmp->list),&(wq->works.list));
	wq->works_nr++;
}

static int avi_r2r__work_queue_pull(struct avi_r2r_channel_data *chan,
				    struct avi_r2r_work *work)
{
	struct avi_r2r_work_queue	*wq = &chan->work_queue;
	struct avi_r2r_work	*tmp;
	struct list_head	*pos;
	int			ret = 0;

	if (!avi_r2r__work_queue_empty(wq)) {

		pos=wq->works.list.next;/* first item in list */
		list_del(pos);
		wq->works_nr--;

		tmp = (struct avi_r2r_work *)pos;
		avi_r2r__work_cpy(work, tmp);

		avi_r2r__work_pool_push_slot(chan, tmp);

		BUG_ON_BAD_WORK(work);

		ret=1;
	}

	return ret;
}

/**
 * Gamma correction configuration management
 */
static inline void avi_r2r__gamma_set_registers(
		struct avi_r2r_gamma_correction_config *in_config,
		struct avi_r2r_gamma_correction_config *registers)
{
	*registers = *in_config;
}

static void avi_r2r__gamma_init_configuration(
		struct avi_r2r_gamma_correction_config *in_config,
		struct avi_r2r_gamma_correction_config *registers)
{
	memset(in_config, 0, sizeof(struct avi_r2r_gamma_correction_config));
	in_config->palette    = 0;
	in_config->bypass     = 1;
	in_config->comp_width = 0;
	avi_r2r__gamma_set_registers(in_config, registers);
}

static void avi_r2r__write_gamma_registers(
			struct avi_node const* node,
			struct avi_r2r_gamma_correction_config *user_registers,
			struct avi_r2r_gamma_correction_config *chan_registers)
{
	int do_write=0;
	if (0 != memcmp(chan_registers,
			user_registers,
			sizeof(struct avi_r2r_gamma_correction_config)))
		do_write= 1;
	if (do_write) {
		*chan_registers = *user_registers;
		avi_gam_setup(	node,
				chan_registers->bypass,
				chan_registers->palette,
				chan_registers->comp_width);
		if (!chan_registers->bypass) {
			avi_gam_set_cmap(node, &chan_registers->cmap);
		}
	}
}

static inline enum avi_r2r_channel_status avi_r2r__get_channel_status(struct avi_r2r_channel_data *channel)
{
	enum avi_r2r_channel_status	ret;

	ret = atomic_read(&channel->status);
	return ret;
}

static inline bool avi_r2r__get_channel_is_running(struct avi_r2r_channel_data *channel)
{
	return (avi_r2r__get_channel_status(channel) == AVI_R2R_CHANNEL_RUNNING);
}

static inline bool avi_r2r__get_channel_is_active(struct avi_r2r_channel_data *channel)
{
	bool		ret = 0;

	ret = (channel->output_segment->active == AVI_SEGMENT_ACTIVE_ONESHOT);
	return ret;
}

static inline bool avi_r2r__channel_is_ok(struct avi_r2r_channel_data *channel)
{
	if (channel)
		return
			((channel->magic & _AVI_R2R_CHANNEL_MAGIC_MASK) ==
				_AVI_R2R_CHANNEL_MAGIC) &&
			((channel->magic_end & _AVI_R2R_CHANNEL_MAGIC_MASK) ==
				_AVI_R2R_CHANNEL_MAGIC) &&
			(channel->magic_end == channel->magic);
	return 0;
}

static inline int avi_r2r__get_user_status(struct avi_r2r_user_context *user)
{
	struct avi_r2r	*avi_r2r = user->avi_r2r;

	int		ret = atomic_read(&user->context->stacked_nr);

	AVIR2R_LOG_DEBUG(avi_r2r,
			 "%s() user %p running=%d stacked=%d",
			 __func__,
			 user,
			 atomic_read(&user->context->running), ret);

	if (atomic_read(&user->context->running))
		ret++;
	return ret;
}

static void avi_r2r__requirements_mask(u32 requirements,
					union avi_r2r_configuration_mask *mask)
{
	mask->_mask=0;
	/* mandatory */
	mask->buffers=1;
	mask->formats=1;
	if (requirements & AVI_CAP_CONV) mask->converter=1;
	if (requirements & AVI_CAP_GAM) mask->gamma=1;
	if (requirements & AVI_CAP_SCAL) mask->scaler=1;
	if (requirements & AVI_CAP_ROT) mask->rotator=1;
}

/**
 * Initialize user data
 */
static void avi_r2r__user_initialize(
		struct avi_r2r_user_context *user,
		u32 requirements,
		const u32 channels_compliant,
		const int channels_compliant_nr)
{
	user->requirements= requirements;
	user->config_done._mask=0;

	avi_r2r__requirements_mask(
			user->requirements,
			&user->requirements_mask);

	user->channels_compliant    = channels_compliant;
	user->channels_compliant_nr = channels_compliant_nr;

	avi_r2r__gamma_init_configuration(
			&user->gamma_user_config,
			&user->registers.gam);

#ifdef CONFIG_AVI_DEBUG
	avi_r2r__user_setup_debugfs(user);
#endif /* CONFIG_AVI_DEBUG */

	return;
}
#define CHECK_ITEM(_test, _msg, arg...)				\
	if (_test) {						\
		AVIR2R_LOG_DEBUG_EXT(&avi_r2r_data, _msg, ##arg);	\
		return false;					\
	}
static bool avi_r2r__check_format(struct avi_segment_format *fmt, bool use_dma)
{
	int			w;
	enum avi_pixel_packing	packing;

	CHECK_ITEM(fmt->width == 0,"width is zero");
	CHECK_ITEM(fmt->height == 0,"height is zero");

	if (use_dma) {
		CHECK_ITEM(fmt->plane0.line_size == 0, "line size #0 is zero");

		packing = avi_pixfmt_get_packing(fmt->pix_fmt);
		if (packing == AVI_SEMIPLANAR_YUV_422_PACKING ||
		    packing == AVI_SEMIPLANAR_YUV_420_PACKING ||
		    packing == AVI_INTERLEAVED_YUV_422_PACKING)
			/* width must be even to match the subsampling */
			CHECK_ITEM(fmt->width % 2,
				  "width not even (%d)",
				  fmt->width);

		if (packing == AVI_SEMIPLANAR_YUV_420_PACKING)
			/* height must be even to match the subsampling */
			CHECK_ITEM(fmt->height % 2,
				  "height not even (%d)",
				  fmt->height);

		w = fmt->width * avi_pixel_size0(fmt->pix_fmt);
		CHECK_ITEM(w > fmt->plane0.line_size,
			  "line size #0 too large (%d > %d)",
			  w, fmt->plane0.line_size);

		if (avi_pixel_size1(fmt->pix_fmt)) {

			CHECK_ITEM(fmt->plane1.line_size == 0,
				  "line size #1 is zero");

			w = fmt->width * avi_pixel_size1(fmt->pix_fmt);

			CHECK_ITEM(w > fmt->plane1.line_size,
				  "line size #1 too large (%d > %d)",
				  w, fmt->plane1.line_size);
		}
	}
	return true;
}

static bool avi_r2r__check_buffer(struct avi_segment_format *fmt,
		         struct avi_dma_buffer *buf)
{
	CHECK_ITEM(buf->status != AVI_BUFFER_READY,
		   "buffer not ready");

	CHECK_ITEM(buf->plane0.dma_addr == 0,
		   "buffer plane #0 address is zero");

	if (fmt->plane1.line_size != 0)
		CHECK_ITEM(buf->plane1.dma_addr == 0,
			   "buffer plane #1 address is zero");

	return true;
}

static bool avi_r2r__check_formats_and_requirements(struct avi_r2r_context *ctx,
			struct avi_segment_format *infmt,
			struct avi_segment_format *outfmt,
			struct avi_segment_format *composefmt,
			struct avi_segment_layout *layout)
{
	u32 l_req=(AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT);
	bool ret = false;

	if (avi_r2r__check_format(infmt, 1) &&
	    avi_r2r__check_format(outfmt, 1)) {
		if (infmt->width  != outfmt->width ||
		    infmt->height != outfmt->height)
			l_req |= AVI_CAP_SCAL;

		if ((ctx->caps & l_req) == l_req)
			ret = true;
	}

	if (composefmt && layout) {
		if (avi_r2r__check_format(composefmt, 0)) {
			ret = true;
		}
	}

	return ret;
}
#undef CHECK_ITEM

/**
 * Segment formats management
 * Composing: left = compose, right = target
 * +--------+             +------+-------+               +--------+
 * | source |-- INPUT --> | left | right |--> OUTPUT --> | target |
 * +--------+             +------+-------+               +--------+
 *
 * Not composing: left = target, right = target
 */
static int avi_r2r_apply_format(struct avi_r2r_channel_data *channel,
				struct avi_r2r_user_format  *format)
{
	struct avi_segment_format	left_fmt, right_fmt;
	struct avi_r2r			*avi_r2r;
	int				ret = 0;

	avi_r2r = channel->avi_r2r;

	/* setup formats on segments */
	right_fmt = format->target;
	/* force pixel format and line sizes for non-DMA format */
	right_fmt.pix_fmt          = AVI_PIXFMT_INVALID;
	right_fmt.plane0.line_size = 0;
	right_fmt.plane1.line_size = 0;

	if (format->use_compose)
		left_fmt = format->compose;
	else
		left_fmt = right_fmt;

	/* force non-DMA */
	left_fmt.pix_fmt          = AVI_PIXFMT_INVALID;
	left_fmt.plane0.line_size = 0;
	left_fmt.plane1.line_size = 0;

	if ((memcmp(&channel->output_segment->input_format,
		    &right_fmt,
		    sizeof(struct avi_segment_format))!=0) ||
	    (memcmp(&channel->output_segment->output_format,
		    &format->target,
		    sizeof(struct avi_segment_format))!=0) ) {
		ret = avi_segment_set_format(channel->output_segment,
					     &right_fmt,
					     &format->target);

		if (ret) {
			AVIR2R_LOG_ERROR(avi_r2r, "%s() Channel %s "
				"avi_segment_set_format output(%d)",
				__func__,
				channel->output_segment->id,
				ret);
			return ret;
		}
	}

	if ((memcmp(&channel->input_segment->input_format,
		    &format->source,
		    sizeof(struct avi_segment_format))!=0) ||
	    (memcmp(&channel->input_segment->output_format,
		    &left_fmt,
		    sizeof(struct avi_segment_format))!=0) ||
	    (memcmp(&channel->input_segment->layout,
		    &format->layout,
		    sizeof(struct avi_segment_layout))!=0)) {
		ret = avi_segment_set_format_and_layout(channel->input_segment,
							&format->source,
							&left_fmt,
							&format->layout);
		if (ret) {
			AVIR2R_LOG_ERROR(avi_r2r,"%s() Channel %s "
				"avi_segment_set_format_and_layout input(%d)",
				__func__,
				channel->input_segment->id,
				ret);
			return ret;
		}
	}

	if (format->use_compose) {
		avi_segment_set_background(channel->output_segment,
					   format->background);
	}

	return 0;
}

static int avi_r2r_apply_stats_format(struct avi_r2r_channel_data *channel,
				      struct avi_r2r_user_format  *format)
{
	struct avi_segment_format stats_fmt = {
		.width	    = AVI_STATS_WIDTH,
		.height	    = AVI_STATS_HEIGHT,
		.interlaced = 0,
		.colorspace = 0,
	};
	struct avi_segment_format dstats_fmt;
	struct avi_r2r *avi_r2r = channel->avi_r2r;
	int ret = 0;

	/* Configure bayer stats */
	avi_statistics_bayer_configure(
			      (struct avi_node *) channel->avi_nodes.bayer_node,
						  format->source.width,
						  format->source.height,
						  64, 48);

	/* Configure segment format */
	dstats_fmt		    = stats_fmt;
	dstats_fmt.pix_fmt	    = AVI_PIXFMT_BGRA8888;
	dstats_fmt.plane0.line_size = dstats_fmt.width * 4;

	/* Set format */
	ret = avi_segment_set_format(channel->stats_segment, &stats_fmt,
				     &dstats_fmt);
	if (ret) {
		AVIR2R_LOG_ERROR(avi_r2r, "%s() Channel %s "
				 "avi_segment_set_stats_format stats(%d)",
				 __func__,
				 channel->stats_segment->id,
				 ret);
		return ret;
	}

	return 0;
}

static int avi_r2r_set_default_format(struct avi_r2r_channel_data *channel)
{
	struct avi_r2r_user_format  *format;
	int ret;

	if ((channel->input_segment->caps & AVI_CAPS_ISP) == AVI_CAPS_ISP)
		format = (struct avi_r2r_user_format *)&isp_default_format;
	else
	if (channel->input_segment->caps & AVI_CAP_PLANAR)
		format = (struct avi_r2r_user_format *)&planar_default_format;
	else
		format = (struct avi_r2r_user_format *)&rgb_default_format;

	ret = avi_r2r_apply_format(channel, format);

	if (channel->stats_segment)
		ret |= avi_r2r_apply_stats_format(channel, format);

	return ret;
}

static int avi_r2r__activate_channel(struct avi_r2r_work *work)
{
	struct avi_r2r			*avi_r2r;
	int				ret = 0;

	BUG_ON_BAD_WORK(work);
	BUG_ON(!avi_r2r__channel_is_ok(work->channel));
	BUG_ON(!avi_r2r__user_context_is_ok(work->user));

	BUG_ON(avi_r2r__get_channel_is_running(work->channel));
	BUG_ON(avi_r2r__get_channel_is_active(work->channel));

	avi_r2r = work->channel->avi_r2r;

	AVIR2R_LOG_DEBUG_EXT(avi_r2r, ">>> %s() Channel %s user %p",
			 __func__,
			 work->channel->output_segment->id,
			 work->user);

	/* callback configure; user lock will locked/unlocked */
	BUG_ON(!work->user->context->ops->configure);
	work->user->context->ops->configure(work->user->context,
					    &work->source_buffer,
					    &work->target_buffer,
					    work->user->context->private);

	BUG_ON(!avi_r2r__check_buffer(&work->user->format.source,
			     &work->source_buffer));
	BUG_ON(!avi_r2r__check_buffer(&work->user->format.target,
			     &work->target_buffer));

	ret = avi_r2r_apply_format(work->channel, &work->user->format);
	if (ret) {
		AVIR2R_LOG_ERROR(avi_r2r, "%s() Channel %s "
			"avi_r2r_apply_format output(%d)",
			__func__,
			work->channel->output_segment->id,
			ret);
		goto activate_error;
	}

	/* Apply format for stats output */
	if (work->channel->stats_segment &&
	    work->stats_buffer.plane0.dma_addr &&
	    avi_r2r_apply_stats_format(work->channel, &work->user->format)) {
		AVIR2R_LOG_ERROR(avi_r2r, "%s() Channel %s "
				 "avi_r2r_apply_stats_format output(%d)",
				 __func__,
				 work->channel->stats_segment->id,
				 ret);
		goto activate_error;
	}

	/* setup nodes not managed by segment API */
	if (work->channel->avi_nodes.gam_node && work->user->config_done.gamma)
		avi_r2r__write_gamma_registers(
			work->channel->avi_nodes.gam_node,
			&work->user->registers.gam,
			&work->channel->registers.gam);

	/* set up buffers */
	avi_segment_set_input_buffer(work->channel->input_segment,
				     &work->source_buffer);

	avi_segment_set_output_buffer(work->channel->output_segment,
				      &work->target_buffer);

	if (work->channel->stats_segment &&
	    work->stats_buffer.plane0.dma_addr)
		avi_segment_set_output_buffer(work->channel->stats_segment,
					      &work->stats_buffer);

	atomic_set(&work->channel->status, AVI_R2R_CHANNEL_RUNNING);
	atomic_set(&work->user->context->running, 1);
	work->user->config_done.buffers = 1;

	avi_r2r__work_cpy(&work->channel->current_work, work);

	BUG_ON_BAD_WORK(&work->channel->current_work);
	BUG_ON(work->channel->current_work.user != work->user);
	BUG_ON(work->channel->current_work.channel != work->channel);

	work->channel->ts_push = work->channel->current_work.ts_push;

	/* Activate stats segment */
	if (work->channel->stats_segment && work->stats_buffer.plane0.dma_addr)
	{
		ret = avi_segment_activate_oneshot(
						  work->channel->stats_segment);
		if (ret) {
			AVIR2R_LOG_ERROR(avi_r2r,"%s() Channel %s "
				"avi_segment_activate stats(%d) %d %s %p",
				__func__,
				work->channel->stats_segment->id,
				ret,
				work->channel->stats_segment->active,
				dev_name(work->user->context->dev),
				work->user);
			goto activate_error;
		}
	}

	/* activate segments:
	 * - enable all nodes
	 * - set FIFOs in single mode
	 * - apply dma_out */
	ret = avi_segment_activate_oneshot(work->channel->input_segment);
	if (ret) {
		AVIR2R_LOG_ERROR(avi_r2r,"%s() Channel %s "
			"avi_segment_activate input(%d) %d %s %p",
			__func__,
			work->channel->input_segment->id,
			ret,
			work->channel->input_segment->active,
			dev_name(work->user->context->dev),
			work->user);
		if (work->channel->stats_segment)
			avi_segment_deactivate(work->channel->stats_segment);
		goto activate_error;
	}

	ret = avi_segment_activate_oneshot(work->channel->output_segment);
	if (ret) {
		AVIR2R_LOG_ERROR(avi_r2r,"%s() Channel %s "
			"avi_segment_activate output(%d) %d %s %p",
			__func__,
			work->channel->output_segment->id,
			ret,
			work->channel->output_segment->active,
			dev_name(work->user->context->dev),
			work->user);
		if (work->channel->stats_segment)
			avi_segment_deactivate(work->channel->stats_segment);
		avi_segment_deactivate(work->channel->input_segment);
		goto activate_error;
	}

	ktime_get_ts(&work->channel->ts_start);
	work->channel->apply_counter++;

	AVIR2R_LOG_DEBUG_EXT(avi_r2r,"<<< %s() Channel %s",
			__func__,
			work->channel->output_segment->id);
	return 0;

activate_error:
	/* done processing call back */
	AVIR2R_LOG_DEBUG(avi_r2r,
		"%s() channel %s error: calling callback done()",
		__func__,
		work->channel->output_segment->id);
	work->source_buffer.status = AVI_BUFFER_ERROR;
	work->target_buffer.status = AVI_BUFFER_ERROR;
	work->stats_buffer.status = AVI_BUFFER_ERROR;
	BUG_ON(!work->user->context->ops->done);
	work->user->context->ops->done(work->user->context,
				       &work->source_buffer,
				       &work->target_buffer,
				       &work->stats_buffer,
				       work->user->context->private);
	AVIR2R_LOG_DEBUG(avi_r2r,"%s() Channel %s FAIL (%d)",
			__func__,
			work->channel->output_segment->id,
			ret);
	return ret;
}

/**
 * Interrupt function
 * Called at the end of the Source FIFO DMA transfer
 */
static unsigned avi_r2r__aux_interrupt(struct avi_segment *segment,
				   enum avi_field *field)
{
	struct avi_r2r_channel_data	*channel = segment->priv;
	struct avi_r2r			*avi_r2r;
	BUG_ON(!avi_r2r__channel_is_ok(channel));

	spin_lock(&channel->lock);

	BUG_ON(!avi_r2r__get_channel_is_running(channel));

	/* output segment (=active channel) has been deactivated
	 * by segment interrupt handler before calling of this handler */
	BUG_ON(avi_r2r__get_channel_is_active(channel));

	avi_r2r = channel->avi_r2r;

	if (segment == channel->input_segment) {
		channel->input_segment_interrupts_counter++;
		AVIR2R_LOG_DEBUG_EXT(avi_r2r,
				 "%s() segment %s",
				 __func__,
				 channel->input_segment->id);
	}
	if (segment == channel->output_segment) {
		channel->output_segment_interrupts_counter++;
		AVIR2R_LOG_DEBUG_EXT(avi_r2r,
				 "%s() segment %s",
				 __func__,
				 channel->output_segment->id);
	}

	spin_unlock(&channel->lock);

	return 0;
}
/**
 * Interrupt function
 * Called at the end of the Target FIFO DMA transfer
 */
static unsigned avi_r2r__interrupt(struct avi_segment *segment,
				   enum avi_field *field)
{
	struct avi_r2r_channel_data	*channel = segment->priv;
	struct avi_r2r			*avi_r2r;
	struct avi_r2r_user_context	*current_user;
	struct avi_r2r_context		*ctx;
	struct avi_r2r_work		work;
	int				ret;

	BUG_ON(!avi_r2r__channel_is_ok(channel));

	spin_lock(&channel->lock);

	BUG_ON(!avi_r2r__get_channel_is_running(channel));

	/* output segment (=active channel) has been deactivated
	 * by segment interrupt handler before calling of this handler */
	BUG_ON(avi_r2r__get_channel_is_active(channel));

	BUG_ON_BAD_WORK(&channel->current_work);

	BUG_ON(!avi_r2r__channel_is_ok(channel->current_work.channel));
	BUG_ON(channel->magic != channel->current_work.channel->magic);

	BUG_ON(!avi_r2r__user_context_is_ok(channel->current_work.user));
	BUG_ON(!channel->current_work.user->context);

	BUG_ON(!avi_r2r__user_context_is_ok(channel->current_work.user));

	BUG_ON(!channel->current_work.user->context);

	avi_r2r = channel->avi_r2r;

	current_user = channel->current_work.user;
	ctx=current_user->context;

	AVIR2R_LOG_DEBUG_EXT(avi_r2r,
		">>> %s() channel %s user %p",
		__func__,
		channel->output_segment->id,
		channel->current_work.user);

	ktime_get_ts(&channel->ts_stop);

	channel->interrupts_counter++;
	if (segment == channel->input_segment) {
		channel->input_segment_interrupts_counter++;
		AVIR2R_LOG_DEBUG_EXT(avi_r2r,
				 "%s() segment %s",
				 __func__,
				 channel->input_segment->id);
	}
	if (segment == channel->output_segment) {
		channel->output_segment_interrupts_counter++;
		AVIR2R_LOG_DEBUG_EXT(avi_r2r,
				 "%s() segment %s",
				 __func__,
				 channel->output_segment->id);
	}
	if (avi_node_get_irq_flag(channel->avi_nodes.target_fifo_node)) {
		/* This interrupt has been acknowledged normally
		 * by avi_segment_irq_handler. */
		channel->spurious_interrupts_counter++;
	}

#ifdef CONFIG_AVI_DEBUG
	avi_r2r__channel_add_timings(channel);
#endif /* CONFIG_AVI_DEBUG */

	avi_fifo_get_status(channel->avi_nodes.target_fifo_node);

	avi_fifo_get_status(channel->avi_nodes.source_fifo_node);
	if (avi_node_get_irq_flag(channel->avi_nodes.source_fifo_node))
		avi_ack_node_irq(channel->avi_nodes.source_fifo_node);
	else
		channel->missing_source_interrupts_counter++;

	if (channel->avi_nodes.source_fifo_planar_node &&
	    avi_pixfmt_is_planar(current_user->format.source.pix_fmt)) {
		avi_fifo_get_status(channel->avi_nodes.source_fifo_planar_node);
		if (avi_node_get_irq_flag(channel->avi_nodes.source_fifo_planar_node))
			avi_ack_node_irq(channel->avi_nodes.source_fifo_planar_node);
		else
			channel->missing_planar_interrupts_counter++;
	}

	/* Do stats checking */
	if (channel->stats_segment &&
	    channel->current_work.stats_buffer.plane0.dma_addr) {
		if (avi_node_get_irq_flag(channel->avi_nodes.stats_fifo_node))
			channel->current_work.stats_buffer.status =
								AVI_BUFFER_DONE;
		else
			/* Looks like the stats are not ready... */
			channel->current_work.stats_buffer.status =
							       AVI_BUFFER_ERROR;

		/* Deactivate segment since IRQ for this segment is disabled and
		 * it is activated in oneshot mode.
		 */
		avi_segment_deactivate(channel->stats_segment);

		/* Make sure we never leave the IT flag raised */
		avi_ack_node_irq(channel->avi_nodes.stats_fifo_node);
	}

	/* if all works have been unstacked, so user is not longer running */
	if (atomic_read(&ctx->stacked_nr) == 0)
		atomic_set(&ctx->running, 0);

	/* done processing call back */
	AVIR2R_LOG_DEBUG_EXT(avi_r2r,
		"%s() channel %s calling callback done()",
		__func__,
		channel->output_segment->id);
	channel->current_work.source_buffer.status = AVI_BUFFER_DONE;
	channel->current_work.target_buffer.status = AVI_BUFFER_DONE;
	BUG_ON(!ctx->ops->done);
	ctx->ops->done(ctx,
		       &channel->current_work.source_buffer,
		       &channel->current_work.target_buffer,
		       &channel->current_work.stats_buffer,
		       ctx->private);

	/* save current user (future features?) */
	channel->previous_user      = current_user;
	/* deactivate channel */
	atomic_set(&channel->status, AVI_R2R_CHANNEL_INACTIVE);
	/* clear finished current work */
	memset(&channel->current_work, 0, sizeof(channel->current_work));

	if (avi_r2r->suspended) {
		/* if suspend requested
		 * do not start further processing
		 */
		AVIR2R_LOG_DEBUG(avi_r2r,"%s() suspended", __func__);
		goto int_unlock;
	}

	do {
		ret = 0;
		/* check queue: if jobs, pull one, activate it */
		if (!avi_r2r__work_queue_empty(&channel->work_queue) &&
		    avi_r2r__work_queue_pull(channel, &work)) {
			BUG_ON_BAD_WORK(&work);
			AVIR2R_LOG_DEBUG_EXT(avi_r2r,
					 "%s() channel %s user %p %d "
					 "activate new work",
					 __func__,
					 channel->output_segment->id,
					 work.user,
					 atomic_read(&work.user->context->stacked_nr));
			BUG_ON(atomic_dec_return(&work.user->context->stacked_nr) < 0);
			ret = avi_r2r__activate_channel(&work);
		}
	} while (ret != 0);

int_unlock:
	AVIR2R_LOG_DEBUG_EXT(avi_r2r,
			 "<<< %s() channel %s",
			 __func__,
			 channel->output_segment->id);
	spin_unlock(&channel->lock);

	/* Call finish callback since we are outside from spinlock */
	BUG_ON(!ctx->ops->finish);
	ctx->ops->finish(ctx, ctx->private);

	return 0;
}

/* TODO: complete support for suspend / resume. */
static int avi_r2r_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct avi_r2r	*avi_r2r = dev_get_drvdata(&pdev->dev);

	BUG_ON(!avi_r2r);

	AVIR2R_LOG_NOTICE(avi_r2r, ">>> %s()", __func__);

	avi_r2r->suspended = 1;

	AVIR2R_LOG_NOTICE(avi_r2r, "<<< %s()", __func__);
	return 0;
}

static int avi_r2r_resume(struct platform_device *pdev)
{
	struct avi_r2r			*avi_r2r = dev_get_drvdata(&pdev->dev);
	struct avi_r2r_channel_data	*channel;
	struct avi_r2r_work		work;
	int 				i, ret=0;
	unsigned long			flags;

	BUG_ON(!avi_r2r);

	AVIR2R_LOG_NOTICE(avi_r2r, ">>> %s()", __func__);

	avi_r2r->suspended = 0;
	for (i = 0; i < avi_r2r->channels_nr; i++) {
		channel = avi_r2r->channels[i];
		BUG_ON(!channel);
		spin_lock_irqsave(&channel->lock, flags);
		do {
			/* check queue: if jobs, pull one, activate it */
			if (!avi_r2r__work_queue_empty(&channel->work_queue) &&
			    avi_r2r__work_queue_pull(channel, &work)) {
				BUG_ON_BAD_WORK(&work);
				AVIR2R_LOG_DEBUG(avi_r2r,
						 "%s() channel %s user %p %d "
						 "activate new work",
						 __func__,
						 channel->output_segment->id,
						 work.user,
						 atomic_read(&work.user->context->stacked_nr));
				BUG_ON(atomic_dec_return(&work.user->context->stacked_nr) < 0);
				ret = avi_r2r__activate_channel(&work);
			}
		} while ( ret != 0);
		spin_unlock_irqrestore(&channel->lock, flags);
	}

	/* An error has occured in channel activation and done callback has been
	 * called, so we must call finish callback
	 */
	if (ret) {
		BUG_ON(!work.user->context->ops->finish);
		work.user->context->ops->finish(work.user->context,
						work.user->context->private);
	}

	AVIR2R_LOG_NOTICE(avi_r2r, "<<< %s()", __func__);
	return 0;
}

static int avi_r2r__check_requirements(struct avi_r2r *data,
					u32 requirements,
					u32 *channels_compliant)
{
	int ret=-1,i,n=0;

	*channels_compliant = 0;
	for (i=0;i<data->channels_nr;i++) {
		if ((data->channels[i]->available & requirements) == requirements) {
//			AVIR2R_LOG_DEBUG(data,
//				"%s() channel #%d 0x%08X=0x%08X->%d",
//				__func__,
//				i,requirements,data->channels[i]->available,n);
			*channels_compliant |= (1<<i);
			n++;
		}
	}
	if (n) ret=n;
	return ret;
}

/**
 * Exported functions
 */


/** check if ram2ram driver is available and if requirements are supported.
 */
int avi_r2r_is_available(u32 requirements)
{
	u32	channels_compliant = 0;
	int	n;

	if (!avi_r2r_data.dev)
		return -ENODEV;

	requirements |= (AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT);

	if ((n=avi_r2r__check_requirements(&avi_r2r_data,
			requirements,
			&channels_compliant))== 0) {
		AVIR2R_LOG_NOTICE(&avi_r2r_data,
			"%s() can not find channel suitable "
			"for requirements 0x%08X",
			__func__, requirements);
		return -ENOENT;
	}

	return 0;
}
EXPORT_SYMBOL(avi_r2r_is_available);

/** Initiate the session with the ram2ram driver.
 */
int avi_r2r_request(struct avi_r2r_context *ctx,
		    struct device *dev,
		    u32 requirements,
		    struct avi_r2r_ops *ops)
{
#ifdef DEBUG
	char				buffer[64];
	int				c, i;
#endif /* DEBUG */
	u32				channels_compliant = 0;
	struct avi_r2r_user_context	*user_context;
	int				n;

	BUG_ON(!ops);
	BUG_ON(!dev);
	BUG_ON(!ctx);

	BUG_ON(!ops->configure || !ops->done || !ops->finish);

	if (!avi_r2r_data.dev)
		return -ENODEV;

	requirements |= (AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT);

	if ((n=avi_r2r__check_requirements(&avi_r2r_data,
			requirements,
			&channels_compliant))== 0) {
		AVIR2R_LOG_NOTICE(&avi_r2r_data,
			"%s() can not find channel suitable "
			"for requirements 0x%08X",
			__func__, requirements);
		return -ENOENT;
	}

#ifdef DEBUG
	c=0;
	c+=sprintf(&buffer[c],"[");
	for (i=0; i < avi_r2r_data.channels_nr; i++)
		if (channels_compliant & (1<<i))
			c += sprintf(&buffer[c]," %d", i);
	c+=sprintf(&buffer[c]," ]");

	AVIR2R_LOG_DEBUG(&avi_r2r_data,
			"%s() found %d channels %s suitable"
			" for requirements 0x%08X of %s",
		__func__,
		n, buffer, requirements,
		dev_name(dev));
#endif /* DEBUG */

	user_context = avi_r2r__user_context_alloc(&avi_r2r_data);

	if (!user_context) {
		AVIR2R_LOG_ERROR(&avi_r2r_data,
			"%s() could not create a new user",
			__func__);
		return -ENOMEM;
	}

	user_context->context = ctx;
	user_context->avi_r2r = &avi_r2r_data;
	avi_r2r__user_initialize(
			user_context,
			requirements,
			channels_compliant,
			n);
	ctx->avi_r2r_private = user_context;
	ctx->caps = requirements;
	ctx->dev = dev;
	ctx->ops = ops;
	atomic_set(&ctx->stacked_nr, 0);
	atomic_set(&ctx->running, 0);

	AVIR2R_LOG_DEBUG(&avi_r2r_data,
		"%s() created user %p (%s)",
		__func__,
		user_context,
		dev_name(ctx->dev));

	return 0;

}
EXPORT_SYMBOL(avi_r2r_request);

/** Destroys a context initialized by a successful avi_r2r_request
 */
int avi_r2r_destroy(struct avi_r2r_context *ctx)
{
	struct avi_r2r_user_context *user = ctx->avi_r2r_private;
	struct avi_r2r	*avi_r2r;
	int 		err;

	BUG_ON(!avi_r2r__user_context_is_ok(user));

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return -ENODEV;

	if ((err = avi_r2r__get_user_status(user)) != 0) {
		AVIR2R_LOG_WARN(avi_r2r,
			"%s() user %p (%s): work queue not empty (%d)",
			__func__,
			user,
			dev_name(ctx->dev),
			err);
		return -EBUSY;
	}

	AVIR2R_LOG_DEBUG(avi_r2r,
		"%s() user %p (%s)",
		__func__,
		user,
		dev_name(ctx->dev));

	avi_r2r__user_context_free(user);
	ctx->avi_r2r_private = NULL;
	return 0;

}
EXPORT_SYMBOL(avi_r2r_destroy);

int avi_r2r_get_status(struct avi_r2r_context *ctx,
		       int *stacked_nr,
		       int *running)
{
	struct avi_r2r_user_context	*user = ctx->avi_r2r_private;
	struct avi_r2r			*avi_r2r;

	BUG_ON(!avi_r2r__user_context_is_ok(user));

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return -ENODEV;

	*stacked_nr = atomic_read(&user->context->stacked_nr);

	*running = atomic_read(&user->context->running);

	AVIR2R_LOG_DEBUG(avi_r2r,
		"%s() user %p (%s) running=%d stacked=%d",
		__func__,
		user,
		dev_name(ctx->dev),
		*running,
		*stacked_nr);

	return 0;
}
EXPORT_SYMBOL(avi_r2r_get_status);

/**
 * Gamma conversion
 */
int avi_r2r_setup_gam(struct avi_r2r_context *ctx,
			bool bypass,
			bool palette,
			bool comp_width,
			struct avi_cmap *cmap)
{
	struct avi_r2r_user_context *user = ctx->avi_r2r_private;
	struct avi_r2r	*avi_r2r;

	BUG_ON(!avi_r2r__user_context_is_ok(user));

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return -ENODEV;

	if (!user->requirements_mask.gamma)
		return -EINVAL;

	AVIR2R_LOG_DEBUG(avi_r2r,
		"%s() user %p (%s)",
		__func__,
		user,
		dev_name(ctx->dev));

	user->gamma_user_config.bypass = bypass;
	user->gamma_user_config.palette = palette;
	user->gamma_user_config.comp_width = comp_width;
	user->gamma_user_config.cmap = *cmap;
	avi_r2r__gamma_set_registers(
		&user->gamma_user_config,
		&user->registers.gam);
	user->config_done.gamma = 1;

	return 0;
}
EXPORT_SYMBOL(avi_r2r_setup_gam);

/* rotator */
int avi_r2r_setup_rotator(struct avi_r2r_context *ctx,
			  enum avi_rotation angle, bool flip)
{
	struct avi_r2r_user_context *user = ctx->avi_r2r_private;
	struct avi_r2r	*avi_r2r;

	BUG_ON(!avi_r2r__user_context_is_ok(user));

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return -ENODEV;

	if (!user->requirements_mask.rotator)
		return -EINVAL;

	AVIR2R_LOG_DEBUG(avi_r2r,
		"%s() user %p (%s)",
		__func__,
		user,
		dev_name(ctx->dev));

	user->rotator_user_config.angle = angle;
	user->rotator_user_config.flip  = flip;
	AVIR2R_LOG_WARN(avi_r2r,
		"Rotator setup required");
	user->config_done.rotator = 1;

	return 0;
}
EXPORT_SYMBOL(avi_r2r_setup_rotator);

/** Input and output format configuration.
 */
int avi_r2r_set_format(struct avi_r2r_context *ctx,
		       struct avi_segment_format *infmt,
		       struct avi_segment_format *outfmt,
		       struct avi_segment_format *composefmt,
		       struct avi_segment_layout *layout)
{
	struct avi_r2r_user_context *user = ctx->avi_r2r_private;
	struct avi_r2r		*avi_r2r;

	BUG_ON(!avi_r2r__user_context_is_ok(user));

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return -ENODEV;

	AVIR2R_LOG_DEBUG(avi_r2r,
		"%s() user %p (%s)",
		__func__,
		user,
		dev_name(ctx->dev));

	if (!avi_r2r__check_formats_and_requirements(ctx,
						     infmt,
						     outfmt,
						     composefmt,
						     layout))
		return -EINVAL;

	user->format.source = *infmt;
	user->format.target = *outfmt;

	/* we are working only in progressive mode */
	user->format.source.interlaced = 0;
	user->format.target.interlaced = 0;

	if (composefmt && layout) {
		/* composing is used */
		user->format.use_compose		= 1;
		user->format.layout		= *layout;
		user->format.compose		= *composefmt;
	}
	else {
		/* if composing is not used,
		 * 	force layout to neutral
		 * 	force compose format to target format
		 */
		user->format.use_compose	= 0;
		user->format.layout.x		= 0;
		user->format.layout.y		= 0;
		user->format.layout.hidden	= 0;
		user->format.layout.alpha	= 0xFF;
		user->format.compose		= user->format.target;
	}

	/* force compose format to non-DMA and progressive */
	user->format.compose.interlaced		= 0;
	user->format.compose.plane0.line_size	= 0;
	user->format.compose.plane1.line_size	= 0;
	user->format.compose.pix_fmt		= AVI_PIXFMT_INVALID;

	user->config_done.formats = 1;

	if (user->requirements_mask.scaler)
		user->config_done.scaler = 1;

	if (user->requirements_mask.converter)
		user->config_done.converter = 1;

	return 0;
}
EXPORT_SYMBOL(avi_r2r_set_format);

/** Put DMA buffers into queue for processing.
 */
int avi_r2r_buf_queue(struct avi_r2r_context *ctx,
			struct avi_dma_buffer *in_dma,
			struct avi_dma_buffer *out_dma,
			struct avi_dma_buffer *stats_dma)
{
	struct avi_r2r_user_context	*user = ctx->avi_r2r_private;
	struct avi_r2r			*avi_r2r;
	struct avi_r2r_channel_data	*channel;
	struct avi_r2r_work		work;
	unsigned long			chan_flags[MAX_CHANNELS_AVI_R2R];
	int				found, c, ret = 0;
	int				i, lvl, min_lvl, idx_min;

	/* is a valid user ? */
	BUG_ON(!avi_r2r__user_context_is_ok(user));

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return -ENODEV;

	AVIR2R_LOG_DEBUG_EXT(avi_r2r, ">>> %s()", __func__);

	/* check formats set */
	if (!user->config_done.formats) {
		AVIR2R_LOG_WARN(avi_r2r,
			"%s() user %p (%s) formats not set",
				__func__,
			user,
			dev_name(ctx->dev));
		ret = -EINVAL;
		goto on_error;
	}

	/* check buffers */
	if (!avi_r2r__check_buffer(&user->format.source, in_dma) ||
	    !avi_r2r__check_buffer(&user->format.target, out_dma) ) {
		AVIR2R_LOG_WARN(avi_r2r,
			"%s() user %p (%s) invalid buffers",
				__func__,
			user,
			dev_name(ctx->dev));
		ret = -EINVAL;
		goto on_error;
	}

	/* lock all channels */
	for (c=0; c < avi_r2r->channels_nr; c++) {
		spin_lock_irqsave(&avi_r2r->channels[c]->lock, chan_flags[c]);
	}
	/* look for a channel compliant */
	idx_min = -1;
	min_lvl   = INT_MAX;
	for (i = avi_r2r->channels_nr - 1; i >= 0 ; i-- ) {
		if (user->channels_compliant & (1<<i)) {
			channel = avi_r2r->channels[i];
			lvl = avi_r2r__work_queue_level(&channel->work_queue);
			if (avi_r2r__get_channel_is_running(channel))
				lvl += 1;
			if (lvl < min_lvl) {
				min_lvl   = lvl;
				idx_min = i;
			}
		}
	}
	found = idx_min;

	if (found < 0) {
		/* unlock all channels */
		for (c=0; c < avi_r2r->channels_nr; c++) {
			spin_unlock_irqrestore(&avi_r2r->channels[c]->lock,
					       chan_flags[c]);
		}
		ret = -ENODEV;
		goto on_error;
	}

	/* unlock unused channels */
	for (c=0; c < avi_r2r->channels_nr; c++) {
		if (c != found)
			spin_unlock_irqrestore(&avi_r2r->channels[c]->lock,
				               chan_flags[c]);
	}

	channel = avi_r2r->channels[found];
	work.user = user;
	work.channel = channel;
	work.source_buffer = *in_dma;
	work.target_buffer = *out_dma;
	work.stats_buffer = *stats_dma;

	ret = 0;
	if (avi_r2r__work_queue_empty(&channel->work_queue) &&
	    !avi_r2r__get_channel_is_running(channel) &&
	    !avi_r2r__get_channel_is_active(channel)) {
		/* channel compliant
		 * 	and inactive
		 * 	and queue empty
		 * --> activate it
		 */
		avi_r2r__work_init(&work);

		AVIR2R_LOG_DEBUG_EXT(avi_r2r,
				 "%s() user %p (%s) "
				 "Activate channel #%d",
				 __func__,
				 user,
				 dev_name(ctx->dev),
				 found);

		ret = avi_r2r__activate_channel(&work);
	}
	else {
		/* channel compliant
		 * 	or running
		 * 	or queue not empty
		 * --> push work on it
		 */

		AVIR2R_LOG_DEBUG_EXT(avi_r2r,
				 "%s() user %p (%s) "
				 "Push on channel #%d",
				 __func__,
				 user,
				 dev_name(ctx->dev),
				 found);

		avi_r2r__work_queue_push(channel, &work);
		atomic_inc(&ctx->stacked_nr);
	}

	/* unlock compliant channel */
	spin_unlock_irqrestore(&channel->lock,
			       chan_flags[found]);

	/* An error has occured in channel activation and done callback has been
	 * called, so we must call finish callback
	 */
	if (ret) {
		BUG_ON(!ctx->ops->finish);
		ctx->ops->finish(ctx, ctx->private);
	}

on_error:
	AVIR2R_LOG_DEBUG_EXT(avi_r2r, "<<< %s() ret=%d", __func__, ret);

	return ret;
}
EXPORT_SYMBOL(avi_r2r_buf_queue);

struct avi_segment * avi_r2r_get_segment(struct avi_r2r_context *ctx,
					 int in_out,
					 int *ret_index)
{
	struct avi_r2r_user_context	*user = ctx->avi_r2r_private;
	struct avi_r2r			*avi_r2r;
	struct avi_r2r_channel_data	*channel;
	struct avi_segment		*segment= NULL;
	int				i;

	/* is a valid user ? */
	BUG_ON(!avi_r2r__user_context_is_ok(user));
	*ret_index = -1;

	avi_r2r = user->avi_r2r;
	if (!avi_r2r->dev)
		return NULL;

	for (i = 0; i < avi_r2r->channels_nr; i++ ) {
		if (user->channels_compliant & (1<<i)) {
			channel = avi_r2r->channels[i];
			if (in_out)
				segment = channel->output_segment;
			else
				segment = channel->input_segment;
			*ret_index = i;
			break;
		}
	}

	if (!segment)
		*ret_index = -1;
	return segment;
}
EXPORT_SYMBOL(avi_r2r_get_segment);

/**
 * Init/Exit module functions
 */
static void avi_r2r_destroy_channels(struct avi_r2r *avi_r2r)
{
	struct avi_r2r_channel_data	*chan_data;
	int				c;

	for (c = 0; c < MAX_CHANNELS_AVI_R2R; c++)
	{
		chan_data = avi_r2r->channels[c];
		if (chan_data) {
			if (atomic_read(&chan_data->status) >=
						AVI_R2R_CHANNEL_INACTIVE) {
				/* Clean remaining works */
				avi_r2r__work_queue_delete(chan_data);

				/* Disable segment IRQ (DMA_OUT) */
				avi_segment_disable_irq(chan_data->output_segment);

				/* to recover in case of freeze r2r */
				if (chan_data->stats_segment)
					avi_apply_node(chan_data->avi_nodes.stats_fifo_node);
				avi_fifo_force_clear(chan_data->avi_nodes.target_fifo_node);

				/* Deactivate stats */
				if (chan_data->stats_segment)
					avi_segment_deactivate(chan_data->stats_segment);

				avi_segment_deactivate(chan_data->input_segment);

				avi_segment_deactivate(chan_data->output_segment);

				/* Release segments */
				avi_segment_disconnect(chan_data->input_segment,
						       chan_data->output_segment);

				/* Destroy stats */
				if (chan_data->stats_segment)
					avi_segment_teardown(chan_data->stats_segment);

				avi_segment_teardown(chan_data->input_segment);
				avi_segment_teardown(chan_data->output_segment);

				atomic_set(&chan_data->status, AVI_R2R_CHANNEL_UNUSED);
			}

#ifdef CONFIG_AVI_DEBUG
			avi_r2r__channel_destroy_debugfs(chan_data);
#endif /* CONFIG_AVI_DEBUG */
			if (chan_data->work_pool)
				kfree(chan_data->work_pool);

			kfree(chan_data);
		}
	}
}

static int __devinit avi_r2r__check_channel_nodes(
		struct avi_r2r_channel_data	*chan_data,
		const unsigned long caps)
{
	struct avi_r2r	*avi_r2r = chan_data->avi_r2r;
	struct avi_node	*the_node = NULL, *scaler_node = NULL;

	chan_data->avi_nodes.nodes_nr = 0;
	/* check nodes */
	chan_data->avi_nodes.source_fifo_node =
			avi_segment_get_node(chan_data->input_segment,
					     AVI_CAP_DMA_IN);
	if (!chan_data->avi_nodes.source_fifo_node) {
		AVIR2R_LOG_ERROR(avi_r2r,
				"Source FIFO is missing");
		return -ENODEV;
	}
	else {
		AVIR2R_LOG_DEBUG(avi_r2r,
				"-- %s (Source)",
				chan_data->avi_nodes.source_fifo_node->name);
		chan_data->available |= AVI_CAP_DMA_IN;
		chan_data->avi_nodes.nodes_nr++;
	}

	chan_data->avi_nodes.target_fifo_node =
			avi_segment_get_node(chan_data->output_segment,
					     AVI_CAP_DMA_OUT);
	if (!chan_data->avi_nodes.target_fifo_node) {
		AVIR2R_LOG_ERROR(avi_r2r,
				"Target FIFO is missing");
		return -ENODEV;
	}
	else {
		AVIR2R_LOG_DEBUG(avi_r2r,
				"-- %s (Target)",
				chan_data->avi_nodes.target_fifo_node->name);
		chan_data->available |= AVI_CAP_DMA_OUT;
		chan_data->avi_nodes.nodes_nr++;
	}

	/* Check stats fifo node */
	if (chan_data->stats_segment) {
		chan_data->avi_nodes.stats_fifo_node = avi_segment_get_node(
						       chan_data->stats_segment,
						       AVI_CAP_DMA_OUT);
		if (!chan_data->avi_nodes.stats_fifo_node) {
			AVIR2R_LOG_ERROR(avi_r2r, "Stats FIFO is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r, "-- %s (Stats)",
				    chan_data->avi_nodes.stats_fifo_node->name);
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if (caps & AVI_CAP_ISP_CHAIN_BAYER) {
		chan_data->avi_nodes.bayer_node =
			avi_segment_get_node(chan_data->input_segment,
					AVI_CAP_ISP_CHAIN_BAYER);
		if (!chan_data->avi_nodes.bayer_node) {
			AVIR2R_LOG_ERROR(avi_r2r,
					"ISP Bayer Chain is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					chan_data->avi_nodes.bayer_node->name);
			chan_data->available |= AVI_CAP_ISP_CHAIN_BAYER;
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if (caps & AVI_CAP_ISP_CHAIN_YUV) {
		the_node =
			avi_segment_get_node(chan_data->input_segment,
					AVI_CAP_ISP_CHAIN_YUV);
		if (!the_node) {
			AVIR2R_LOG_ERROR(avi_r2r,
					"ISP YUV Chain is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					the_node->name);
			chan_data->available |= AVI_CAP_ISP_CHAIN_YUV;
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if (caps & AVI_CAP_STATS_YUV) {
		the_node =
			avi_segment_get_node(chan_data->input_segment,
					AVI_CAP_STATS_YUV);
		if (!the_node) {
			AVIR2R_LOG_ERROR(avi_r2r,
					"Statistics YUV is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					the_node->name);
			chan_data->available |= AVI_CAP_STATS_YUV;
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if (caps & AVI_CAP_CONV) {
		the_node =
			avi_segment_get_node(chan_data->input_segment,
					     AVI_CAP_CONV);
		if (!the_node) {
			AVIR2R_LOG_ERROR(avi_r2r,
					"Color Space Converter is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					the_node->name);
			chan_data->available |= AVI_CAP_CONV;
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if (caps & AVI_CAP_GAM) {
		chan_data->avi_nodes.gam_node =
			avi_segment_get_node(chan_data->input_segment,
					     AVI_CAP_GAM);
		if (!chan_data->avi_nodes.gam_node) {
			AVIR2R_LOG_ERROR(avi_r2r,
					"Gamma Correction is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					chan_data->avi_nodes.gam_node->name);
			chan_data->available |= AVI_CAP_GAM;
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if (caps & AVI_CAP_SCAL) {
		scaler_node =
			avi_segment_get_node(chan_data->input_segment, AVI_CAP_SCAL);
		if (!scaler_node)
		{
			AVIR2R_LOG_ERROR(avi_r2r,
					"Scaler is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					scaler_node->name);
			chan_data->available |= AVI_CAP_SCAL;
			chan_data->avi_nodes.nodes_nr++;
		}
	}

	if ( (caps & AVI_CAP_PLANAR) && scaler_node) {
		chan_data->avi_nodes.source_fifo_planar_node =
			avi_segment_get_node(chan_data->input_segment,
					     AVI_CAP_PLANAR);
		if (chan_data->avi_nodes.source_fifo_planar_node)
		{
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s (Source Plane 1)",
					chan_data->avi_nodes.source_fifo_planar_node->name);
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					avi_get_node(scaler_node->node_id+1)->name);
			chan_data->available |= AVI_CAP_PLANAR;
			chan_data->avi_nodes.nodes_nr++;
		}
		else
		{
			AVIR2R_LOG_WARN(avi_r2r,
					"Planar is missing");
		}
	}

	if (caps & AVI_CAP_ROT) {
		chan_data->avi_nodes.rotator_node =
			avi_segment_get_node(chan_data->input_segment, AVI_CAP_ROT);
		if (!chan_data->avi_nodes.rotator_node)
		{
			AVIR2R_LOG_ERROR(avi_r2r,
					"Rotator is missing");
			return -ENODEV;
		}
		else {
			AVIR2R_LOG_DEBUG(avi_r2r,
					"-- %s",
					chan_data->avi_nodes.rotator_node->name);
			chan_data->available |= AVI_CAP_ROT;
			chan_data->avi_nodes.nodes_nr++;
		}
	}
	return 0;
}

#define DMA_OUT_CAPS (AVI_CAP_DMA_OUT)
static int __devinit avi_r2r_setup_channels(struct avi_r2r *avi_r2r,
					    struct avi_r2r_platform_data *pdata)
{
	struct avi_segment		*out_seg = NULL, *in_seg = NULL;
	struct avi_segment		*stats_seg = NULL;
	char				gen_id[AVI_SEGMENT_ID_LEN];
	unsigned long			required_caps, missing_caps;
	struct avi_r2r_channel_data	*chan_data;
	int				ret = 0, c;

	avi_r2r->channels_nr=0;
	for (c = 0;pdata->channels[c].caps; c++) {
		BUG_ON(c >= MAX_CHANNELS_AVI_R2R);

		if (avi_r2r->channels_nr>MAX_CHANNELS_AVI_R2R) {
			AVIR2R_LOG_ERROR(avi_r2r,
					"Maximum number of channels reached");
			return -EINVAL;
		}

		/* force capabilities */
		pdata->channels[c].caps |= AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT;

		/* build output segment */
		required_caps = missing_caps = pdata->channels[c].caps & DMA_OUT_CAPS;
		out_seg = avi_segment_build(&missing_caps,
					    "r2r",
					    c,
					    -1,
					    avi_r2r->dev);

		if (IS_ERR(out_seg)) {
			AVIR2R_LOG_ERROR(avi_r2r,
				"couldn't build output segment %d,"
				" caps=%08lX, missing caps=%08lX",
				c,
				required_caps,
				missing_caps);
			ret = PTR_ERR(out_seg);
			goto seg_error;
		}

		/* build input segment */
		required_caps = missing_caps  = pdata->channels[c].caps & ~DMA_OUT_CAPS;
		in_seg = avi_segment_build(&missing_caps,
					   "r2r-in",
					   c,
					   -1,
					   avi_r2r->dev);

		if (IS_ERR(in_seg)) {
			AVIR2R_LOG_ERROR(avi_r2r,
				"couldn't build input segment %s,"
				" caps=%08lX, missing caps=%08lX",
				gen_id,
				required_caps,
				missing_caps);
			ret = PTR_ERR(in_seg);
			goto seg_error;
		}

		/* connect segments */
		ret = avi_segment_connect(in_seg, out_seg, z_order);
		if (ret<0) {
			AVIR2R_LOG_ERROR(avi_r2r,
				"couldn't connect segments %s to %s",
				in_seg->id,
				out_seg->id);
			goto seg_error;
		}

		/* Add a segment for stats if ISP is used */
		if (required_caps & AVI_CAP_ISP_CHAIN_BAYER) {
			/* Find bayer stats */
			required_caps = avi_isp_bayer_get_stats_cap(in_seg);
			if (!required_caps) {
				AVIR2R_LOG_ERROR(avi_r2r,
						 "can't find stats cap");
				goto seg_error;
			}

			/* Build stats segment */
			required_caps |= AVI_CAP_DMA_OUT;
			stats_seg = avi_segment_build(&required_caps,
						      "r2r-stats",
						      c,
						      -1,
						      avi_r2r->dev);

			if (IS_ERR(stats_seg)) {
				AVIR2R_LOG_ERROR(avi_r2r,
					      "couldn't build stats segment %d",
					      c);
				ret = PTR_ERR(stats_seg);
				goto seg_error;
			}
		}
		else
			stats_seg = NULL;

		/* allocate data for channel */
		chan_data = kzalloc(sizeof(struct avi_r2r_channel_data),
				    GFP_KERNEL);
		if (!chan_data) {
			AVIR2R_LOG_ERROR(avi_r2r,
				"couldn't allocate struct avi_r2r_channel_data");
			ret = -ENOMEM;
			goto seg_error;
		}

		/* initialize channel data */

		chan_data->avi_r2r = avi_r2r;

		out_seg->priv			= chan_data;
		in_seg->priv			= chan_data;
		chan_data->output_segment	= out_seg;
		chan_data->input_segment	= in_seg;
		chan_data->stats_segment	= stats_seg;
		atomic_set(&chan_data->status, AVI_R2R_CHANNEL_UNUSED);

		AVIR2R_LOG_DEBUG(avi_r2r,
				". Channel #%d",avi_r2r->channels_nr);
		ret=avi_r2r__check_channel_nodes(chan_data,
						 pdata->channels[c].caps);
		if (ret < 0)
			goto seg_error;

		ret = avi_r2r_set_default_format(chan_data);
		if (ret < 0)
			goto seg_error;

		/* Request segments' IRQ */
		avi_segment_register_irq(chan_data->input_segment,
					 &avi_r2r__interrupt);

		avi_segment_register_irq(chan_data->output_segment,
					 &avi_r2r__aux_interrupt);

		/* Enable segment IRQ (DMA_OUT) */
		avi_segment_enable_irq(chan_data->output_segment);

		spin_lock_init(&chan_data->lock);
		ret = avi_r2r__work_pool_init(chan_data, works_pool);
		if (ret)
			goto seg_error;
		avi_r2r__work_queue_init(&chan_data->work_queue);
		atomic_set(&chan_data->status, AVI_R2R_CHANNEL_INACTIVE);
		chan_data->magic     = _AVI_R2R_CHANNEL_MAGIC + c;
		chan_data->magic_end = _AVI_R2R_CHANNEL_MAGIC + c;

#ifdef CONFIG_AVI_DEBUG
		chan_data->nr_timings_saved = nr_timings_saved;
		ret = avi_r2r__channel_setup_debugfs(chan_data);
		if (ret) {
			if (chan_data->work_pool)
				kfree(chan_data->work_pool);
			goto seg_error;
		}
#endif /* CONFIG_AVI_DEBUG */

		AVIR2R_LOG_DEBUG(avi_r2r,
				"- Channel #%d: %d nodes,"
				" IRQ Node %s, IRQ n° %d,"
				" Pool size %d",
				c,
				chan_data->avi_nodes.nodes_nr,
				chan_data->output_segment->irq_node->name,
				avi_node_irq(chan_data->output_segment->irq_node),
				works_pool);


		avi_r2r->channels[c] = chan_data;
		avi_r2r->channels_nr++;
	}
	if (!ret)
	{
		int i;
		AVIR2R_LOG_INFO(avi_r2r,
			"Found %d channels", avi_r2r->channels_nr);
		for (i=0; i<avi_r2r->channels_nr; i++)
			AVIR2R_LOG_INFO(avi_r2r,
				"Channel #%d: %s", i, avi_r2r->channels[i]->output_segment->id);

	}
	return ret;
seg_error:
	if (out_seg)
		avi_segment_teardown(out_seg);
	if (in_seg)
		avi_segment_teardown(in_seg);
	if (stats_seg)
		avi_segment_teardown(stats_seg);
	return ret;
}

static int __devinit avi_r2r_probe(struct platform_device *pdev)
{
	struct avi_r2r_platform_data	*platform_data;
	int				ret;

	dev_dbg(&pdev->dev, "Probing...\n");

#define ERROR_OUT(lbl, msg, errno)						\
	do { ret = errno;							\
		dev_err(&pdev->dev, "Probing failed: %s [%d]\n", msg, ret);	\
		goto lbl; }							\
	while (0)

	platform_data = dev_get_platdata(&pdev->dev);

	if (!platform_data)
		ERROR_OUT(exit, "failed to find platform data", -EINVAL);

	dev_set_drvdata(&pdev->dev, &avi_r2r_data);

	avi_r2r_data.dev = &pdev->dev;

#ifdef CONFIG_AVI_DEBUG
	if (nr_timings_saved < _NR_TIMINGS_SAVED_DEFAULT)
		nr_timings_saved = _NR_TIMINGS_SAVED_DEFAULT;

	avi_r2r_data.debugfs_root =
		debugfs_create_dir(dev_name(&pdev->dev), NULL);
	if (IS_ERR_OR_NULL(avi_r2r_data.debugfs_root))
		ERROR_OUT(exit,
				"debugfs_create_dir failed",
				avi_r2r_data.debugfs_root ?
				PTR_ERR(avi_r2r_data.debugfs_root) : -ENOMEM);

	dev_info(&pdev->dev, "DebugFS enabled\n");
#endif /* CONFIG_AVI_DEBUG */

	ret = avi_r2r_setup_channels(&avi_r2r_data, platform_data);
	if (ret)
		ERROR_OUT(avi_unregister,
			"setup channels failed", ret);

#ifdef CONFIG_AVI_DEBUG
	dev_dbg(&pdev->dev, "Saved timings: %d\n", nr_timings_saved);
#endif /* CONFIG_AVI_DEBUG */

	dev_dbg(&pdev->dev, "Probed\n");

	return 0;

avi_unregister:
	avi_r2r_destroy_channels(&avi_r2r_data);
#ifdef CONFIG_AVI_DEBUG
        if (avi_r2r_data.debugfs_root)
		debugfs_remove_recursive(avi_r2r_data.debugfs_root);
	avi_r2r_data.debugfs_root = NULL;
#endif /* CONFIG_AVI_DEBUG */
exit:
	return ret;
}

static int __devexit avi_r2r_remove(struct platform_device *pdev)
{
	struct avi_r2r	*avi_r2r = dev_get_drvdata(&pdev->dev);

	dev_dbg(&pdev->dev, "Removing...\n");

	BUG_ON(!avi_r2r);

	avi_r2r_destroy_channels(avi_r2r);
#ifdef CONFIG_AVI_DEBUG
	if (avi_r2r->debugfs_root)
		debugfs_remove_recursive(avi_r2r->debugfs_root);
	avi_r2r->debugfs_root = NULL;
#endif /* CONFIG_AVI_DEBUG */
	dev_set_drvdata(&pdev->dev, NULL);
	avi_r2r->dev = NULL;

	dev_dbg(&pdev->dev, "Removed\n");

	return 0;
}

static struct platform_driver avi_r2r_driver = {
	.driver         = {
		.name   = AVI_R2R_DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
	.probe          = &avi_r2r_probe,
	.remove         = __devexit_p(avi_r2r_remove),
	.suspend        = &avi_r2r_suspend,
	.resume         = &avi_r2r_resume
};

static int __init avi_r2r_init(void)
{
	int ret;
	pr_debug(AVI_R2R_DRIVER_NAME " Initializing...\n");
	ret = platform_driver_register(&avi_r2r_driver);
	if (ret) {
		pr_err(AVI_R2R_DRIVER_NAME " init fail (%d)\n", ret);
	}
	else {
		pr_info(AVI_R2R_DRIVER_NAME " Initialized\n");
	}
	return ret;
}

module_init(avi_r2r_init);

static void __exit avi_r2r_exit(void)
{
	pr_debug(AVI_R2R_DRIVER_NAME " Exiting...\n");
	platform_driver_unregister(&avi_r2r_driver);
	pr_debug(AVI_R2R_DRIVER_NAME " Exited\n");
}

module_exit(avi_r2r_exit);

MODULE_AUTHOR("Didier Leymarie <didier.leymarie.ext@parrot.com>");
MODULE_DESCRIPTION("RAM to RAM driver for Parrot7 Advanced Video Interface");
MODULE_LICENSE("GPL");

