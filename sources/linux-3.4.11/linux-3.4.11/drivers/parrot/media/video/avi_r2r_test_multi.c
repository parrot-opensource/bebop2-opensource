/**
 * @file avi_r2r_test_multi.c
 *  Parrot AVI ram2ram driver (test module with multiple clients support).
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * @author     didier.leymarie.ext@parrot.com
 * @date       2013-11-07
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

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/hrtimer.h>
#include <linux/atomic.h>
#include <linux/printk.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#include "video/avi_segment.h"
#include "video/avi_dma.h"
#include "video/avi_debug.h"
#include "video/avi_pixfmt.h"
#include "avi_r2r.h"


#define DRIVER_NAME "avi_r2r_test_multi"
#define DRIVER_VERSION KERNEL_VERSION(0, 0, 0)

#define AVIR2R_LOG_INFO(data, fmt, arg...)	\
	dev_info((data)->dev, fmt "\n", ##arg)
#define AVIR2R_LOG_WARN(data, fmt, arg...)	\
	dev_warn((data)->dev, fmt "\n", ##arg)
#define AVIR2R_LOG_NOTICE(data, fmt, arg...)	\
	dev_notice((data)->dev, fmt "\n", ##arg)
#define AVIR2R_LOG_ERROR(data, fmt, arg...)	\
	dev_err((data)->dev, fmt "\n", ##arg)
#define AVIR2R_LOG_DEBUG(data, fmt, arg...)	\
	dev_dbg((data)->dev, fmt "\n", ##arg)
#define AVIR2R_LOG_DEBUG_EXT(data, fmt, arg...)	\
	dev_dbg((data)->dev, fmt "\n", ##arg)

/* command line parameters */

/* source */
static unsigned int src_pixfmt = 1;
module_param(src_pixfmt, uint, S_IRUGO);

static unsigned int src_frame_width = 1280;
module_param(src_frame_width, uint, S_IRUGO);

static unsigned int src_frame_height = 720;
module_param(src_frame_height, uint, S_IRUGO);

static unsigned int src_top = 0;
module_param(src_top, uint, S_IRUGO);

static unsigned int src_left = 0;
module_param(src_left, uint, S_IRUGO);

static unsigned int src_width = 1280;
module_param(src_width, uint, S_IRUGO);

static unsigned int src_height = 720;
module_param(src_height, uint, S_IRUGO);

static unsigned int src_delta_left = 0;
module_param(src_delta_left, int, S_IRUGO);

static unsigned int src_delta_top = 0;
module_param(src_delta_top, int, S_IRUGO);

static unsigned int src_sa0 = 0;
module_param(src_sa0, uint, S_IRUGO);

static unsigned int src_sa1 = 0;
module_param(src_sa1, uint, S_IRUGO);

static unsigned int src_cspace = AVI_RGB_CSPACE;
module_param(src_cspace, uint, S_IRUGO);

static unsigned int src_nbuffers = 1;
module_param(src_nbuffers, uint, S_IRUGO);

/* target */
static unsigned int tgt_pixfmt = 2;
module_param(tgt_pixfmt, uint, S_IRUGO);

static unsigned int tgt_frame_width = 1280;
module_param(tgt_frame_width, uint, S_IRUGO);

static unsigned int tgt_frame_height = 720;
module_param(tgt_frame_height, uint, S_IRUGO);

static unsigned int tgt_top = 0;
module_param(tgt_top, uint, S_IRUGO);

static unsigned int tgt_left = 0;
module_param(tgt_left, uint, S_IRUGO);

static unsigned int tgt_width = 1280;
module_param(tgt_width, uint, S_IRUGO);

static unsigned int tgt_height = 720;
module_param(tgt_height, uint, S_IRUGO);

static unsigned int tgt_delta_left = 0;
module_param(tgt_delta_left, int, S_IRUGO);

static unsigned int tgt_delta_top = 0;
module_param(tgt_delta_top, int, S_IRUGO);

static unsigned int tgt_sa0 = 0;
module_param(tgt_sa0, uint, S_IRUGO);

static unsigned int tgt_sa1 = 0;
module_param(tgt_sa1, uint, S_IRUGO);

static unsigned int tgt_cspace = AVI_RGB_CSPACE;
module_param(tgt_cspace, uint, S_IRUGO);

static unsigned int tgt_nbuffers = 1;
module_param(tgt_nbuffers, uint, S_IRUGO);

static unsigned int gam_bypass = 1;
module_param(gam_bypass, int, S_IRUGO);

static unsigned int gam_palette = 0;
module_param(gam_palette, int, S_IRUGO);

static unsigned int config_run_time = 0;
module_param(config_run_time, int, S_IRUGO);

static unsigned int tag_src = 0;
module_param(tag_src, int, S_IRUGO);

/* number of operations */
static unsigned int delta_count = 1;
module_param(delta_count, uint, S_IRUGO);

/* time out (ms) for done */
#ifndef CONFIG_ARCH_VEXPRESS
#define DONE_TMOUT 100 /* 100 ms: 1/10th of a sec */
#else
#define DONE_TMOUT 2500 /* 2500 ms: VExpress vs real P7 : x25 slower */
#endif
static unsigned int time_out = DONE_TMOUT;
module_param(time_out, uint, S_IRUGO);

/* period (microseconds) */
static unsigned int period = 0;
module_param(period, uint, S_IRUGO);

/* start up delay (milliseconds) */
static unsigned int startup_delay = 0;
module_param(startup_delay, uint, S_IRUGO);

/* verbose display */
static unsigned int verbose = 1;
module_param(verbose, uint, S_IRUGO);

/* size of timings saved */
#define _NR_TIMINGS_SAVED 20
static unsigned int nr_timings_saved = _NR_TIMINGS_SAVED;
module_param(nr_timings_saved, uint, S_IRUGO);

/**
 * struct avi_frame - AVI whole frame descriptor .
 * @width: frame width in pixels
 * @height: frame height in pixels
 */
struct avi_frame {
	u16 width;
	u16 height;
};
/**
 * Image configuration
 */
struct avi_r2r_image_configuration {
	/* whole frame */
	struct avi_frame	frame;
	/* rectangle within frame */
	struct avi_rect		rect;
	/* pixel format AVI_xxxx_FMT in avi.h */
	struct avi_dma_pixfmt 	pixel_format;

	enum avi_colorspace	colorspace;
	struct {
		unsigned int line_size;
		unsigned int frame_size;
	}plane0, plane1;
};
/**
 * Source/Target Image configuration
 */
struct avi_r2r_images_config {
	/* definition of source picture */
	struct avi_r2r_image_configuration source;
	/* definition of target picture */
	struct avi_r2r_image_configuration target;
} ;
/**
 * Buffers definition
 */
/**
 * Parameters for submit: pair of buffer addresses (Plane 0, Plane 1)
 */
struct avi_r2r_frame_physical_addresses {
	dma_addr_t plane_0;
	dma_addr_t plane_1;
};

struct avi_r2r_buffers_config {
	struct avi_r2r_frame_physical_addresses source;
	struct avi_r2r_frame_physical_addresses target;
};

/**
 * Gamma correction
 */
struct avi_r2r_gamma_correction_config {
	struct avi_cmap cmap;
	struct
	{
		unsigned bypass		:1;
		unsigned palette	:1;
		unsigned comp_width	:1;
	};
};

/**
 *  AVI RAM to RAM data: global test driver data
 */
struct kt_period {
	ktime_t		begin;
	ktime_t		end;
	int		overrun;
};
#define NR_REQUIREMENTS 5
struct avi_r2r_test {
	struct device			*dev;
	struct platform_device		*platform_device;

	void				*mem_start;
	void				*phys_mem_start;
	size_t				mem_size;
	struct resource			*vmem;
	struct {
		struct avi_segment_format	source;
		struct avi_segment_format	target;
	} formats;
	struct {
		struct avi_dma_buffer	source;
		struct avi_dma_buffer	target;
	} buffers;
	struct avi_r2r_images_config	images_config;
	struct avi_r2r_buffers_config	buffers_config;
	struct avi_r2r_buffers_config	l_buffers_config;
	unsigned int			src_nbuffers;
	unsigned int			tgt_nbuffers;
	unsigned int			buffer_in_idx;
	unsigned int			buffer_out_idx;

	struct avi_r2r_gamma_correction_config	gamma_config;
	u32 				requirements[NR_REQUIREMENTS];
	struct avi_r2r_context 		context_id;
	struct avi_r2r_ops		ops;
	struct {
		unsigned		number;
		struct {
			int left;
			int top;
		}source,target;
	} deltas;
	unsigned long			period; /* microseconds*/
	ktime_t				ktime_period_ns;
	unsigned			time_out;/* milliseconds */
	unsigned			startup_delay;/* milliseconds */

	int 				run;
	int 				running;
	int 				config_run_time;
	int 				tag_src;
	int 				verbose;

	unsigned			counter_buf_queue;
	unsigned			counter_done;
	unsigned int			frame_index_src, frame_index_tgt;
	unsigned int			mismatch_frame_index_src,
					mismatch_frame_index_tgt,
					mismatch_frame_index;
	bool				finished;
	bool				aborted;
	bool				done_periodic;
	struct kt_period		*kt_period;
	int				kt_period_index,kt_period_nr,kt_period_counter;
	wait_queue_head_t		wait_done;
	atomic_t			callback_done_ran;
	atomic_t			callback_configure_ran;
	atomic_t			done_counter;
	atomic_t			all_done;
	struct hrtimer			timer;
	spinlock_t			lock;
	/* The root of our debugfs filesystem. */
	struct dentry 			*debugfs_root;
};

static void setup_formats(struct avi_r2r_test *data)
{
	data->formats.source.pix_fmt = data->images_config.source.pixel_format;
	data->formats.source.width   = data->images_config.source.rect.width;
	data->formats.source.height  = data->images_config.source.rect.height;
	data->formats.source.interlaced = 0;
	data->formats.source.colorspace = data->images_config.source.colorspace;
	avi_dma_setup_line_size(&data->formats.source,
				data->images_config.source.frame.width,
				0);
	data->images_config.source.plane0.line_size =
			data->formats.source.plane0.line_size;
	data->images_config.source.plane1.line_size =
			data->formats.source.plane1.line_size;
	data->images_config.source.plane0.frame_size =
			data->images_config.source.plane0.line_size *
				data->images_config.source.frame.height;
	data->images_config.source.plane1.frame_size =
			data->images_config.source.plane1.line_size *
				data->images_config.source.frame.height;

	data->formats.target.pix_fmt = data->images_config.target.pixel_format;
	data->formats.target.width   = data->images_config.target.rect.width;
	data->formats.target.height  = data->images_config.target.rect.height;
	data->formats.target.interlaced = 0;
	data->formats.target.colorspace = data->images_config.target.colorspace;
	avi_dma_setup_line_size(&data->formats.target,
				data->images_config.target.frame.width,
				0);
	data->images_config.target.plane0.line_size =
			data->formats.target.plane0.line_size;
	data->images_config.target.plane1.line_size =
			data->formats.target.plane1.line_size;
	data->images_config.target.plane0.frame_size =
			data->images_config.target.plane0.line_size *
				data->images_config.target.frame.height;
	data->images_config.target.plane1.frame_size =
			data->images_config.target.plane1.line_size *
				data->images_config.target.frame.height;
}

static void setup_dma(struct avi_dma_buffer *buf,
		      struct avi_segment_format *fmt,
		      struct avi_r2r_frame_physical_addresses *physaddrs,
		      unsigned left,
		      unsigned top)
{
	unsigned pixsz,l_top;

	buf->plane0.dma_addr = physaddrs->plane_0;
	if (buf->plane0.dma_addr==0) {
		buf->status=AVI_BUFFER_INVAL;
		return;
	}
	pixsz = avi_pixel_size0(fmt->pix_fmt);
	buf->plane0.dma_addr += fmt->plane0.line_size * top;
	buf->plane0.dma_addr += left * pixsz;

	buf->plane1.dma_addr = 0;

	pixsz = avi_pixel_size1(fmt->pix_fmt);

	if (pixsz > 0) {
		buf->plane1.dma_addr = physaddrs->plane_1;
		if (buf->plane1.dma_addr == 0) {
			buf->status = AVI_BUFFER_INVAL;
			return;
		}
		l_top = top;

		if (fmt->pix_fmt.id == AVI_NV12_FMTID ||
		    fmt->pix_fmt.id == AVI_NV21_FMTID)
				l_top = l_top/2;

		buf->plane1.dma_addr +=	fmt->plane1.line_size * l_top;
		buf->plane1.dma_addr += left * pixsz;
	}
	buf->status=AVI_BUFFER_READY;
}

static inline uint32_t timeval_to_usec(const struct timeval *tv)
{
	return ((uint32_t)tv->tv_sec * 1000000) + (uint32_t)tv->tv_usec;
}

static void report_status(struct avi_r2r_test *data)
{
	if (data->aborted)
		AVIR2R_LOG_INFO(data,"Aborted");
	if (data->finished)
		AVIR2R_LOG_INFO(data,"Finished");

	if (data->verbose) {
		AVIR2R_LOG_INFO(data,"Callback done:\t%d",
				atomic_read(&data->callback_done_ran));
		AVIR2R_LOG_INFO(data,"Callback configure:\t%d",
				atomic_read(&data->callback_configure_ran));
		AVIR2R_LOG_INFO(data,"Counter Buf queue:\t%d",
				data->counter_buf_queue);
		AVIR2R_LOG_INFO(data,"Counter Done:\t%d",
				data->counter_done);
	}
}

static inline unsigned ms_to_jiffies(unsigned ms)
{
	return (ms/(1000/HZ));
}
static int wait_for_done(struct avi_r2r_test *data)
{
	int			 ret;

	ret = wait_event_interruptible_timeout(data->wait_done,
			atomic_read(&data->done_counter)
				!= atomic_read(&data->all_done),
			data->deltas.number*ms_to_jiffies(data->time_out)
							*(data->period==0?1:2));

	if (data->verbose)
		AVIR2R_LOG_INFO(data,"%s() ret=%d!", __FUNCTION__, ret);
	if (ret == 0)
		return -ETIMEDOUT;

	if (ret < 0)
		return ret;

	return 0;
}

static void batch_queue(struct avi_r2r_test *data)
{
	struct avi_dma_buffer stats_buf = {
		.plane0.dma_addr = 0,
	};
	int ret;

	if (0 == data->buffer_in_idx)
		data->l_buffers_config.source = data->buffers_config.source;
	if (0 == data->buffer_out_idx)
		data->l_buffers_config.target = data->buffers_config.target;

	setup_dma(&data->buffers.source,
		  &data->formats.source,
		  &data->l_buffers_config.source,
		  data->images_config.source.rect.left,
		  data->images_config.source.rect.top);

	setup_dma(&data->buffers.target,
		  &data->formats.target,
		  &data->l_buffers_config.target,
		  data->images_config.target.rect.left,
		  data->images_config.target.rect.top);

	data->buffers.source.priv = (void *)(0x42000000|data->counter_buf_queue);
	data->buffers.target.priv = (void *)(0x62000000|data->counter_buf_queue);

	ret=avi_r2r_buf_queue(&data->context_id,
			&data->buffers.source,
			&data->buffers.target,
			&stats_buf);
	if (ret>=0) {
		if (data->verbose)
			AVIR2R_LOG_INFO(data,
				"%s() #%d/%d Buf Queue successful",
				__FUNCTION__,
				data->counter_buf_queue,
				data->deltas.number);
	}
	else {
		AVIR2R_LOG_INFO(data,"%s() #%d/%d "
				"Buf Queue failed (%d)",
				__FUNCTION__,
				data->counter_buf_queue,
				data->deltas.number,
				ret);
		data->aborted=1;
		return;
	}

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s() #%d/%d",__FUNCTION__,
			data->counter_buf_queue,
			data->deltas.number);

	data->counter_buf_queue++;
	data->images_config.source.rect.left += data->deltas.source.left;
	data->images_config.source.rect.top  += data->deltas.source.top;
	data->images_config.target.rect.left += data->deltas.target.left;
	data->images_config.target.rect.top  += data->deltas.target.top;

	data->buffer_in_idx++;
	if (data->buffer_in_idx > data->src_nbuffers)
		data->buffer_in_idx = 0;
	else {
		data->l_buffers_config.source.plane_0 += data->images_config.source.plane0.frame_size;
		data->l_buffers_config.source.plane_1 += data->images_config.source.plane1.frame_size;
	}
	data->buffer_out_idx++;
	if (data->buffer_out_idx > data->tgt_nbuffers)
		data->buffer_out_idx = 0;
	else {
		data->l_buffers_config.target.plane_0 += data->images_config.target.plane0.frame_size;
		data->l_buffers_config.target.plane_1 += data->images_config.target.plane1.frame_size;
	}
}

static void my_callback_done(struct avi_r2r_context *ctx,
				struct avi_dma_buffer *in_buf,
				struct avi_dma_buffer *out_buf,
				struct avi_dma_buffer *stats_buf,
				void *priv)
{
	struct avi_r2r_test *data=ctx->private;
	unsigned int index_src, index_tgt;

	spin_lock(&data->lock);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) IN",__FUNCTION__,
			atomic_read(&data->callback_done_ran));

	atomic_inc(&data->callback_done_ran);

	index_src=(unsigned int)in_buf->priv & 0x00FFFFFF;
	index_tgt=(unsigned int)out_buf->priv & 0x00FFFFFF;

	if (index_src != index_tgt) {
		data->mismatch_frame_index++;
		AVIR2R_LOG_INFO(data, "%s(%d) index mismatch src=%u tgt=%u",
				__FUNCTION__, data->counter_done,
				index_src, index_tgt);
	}

	if (index_src != (data->frame_index_src+1)) {
		data->mismatch_frame_index_src++;
		AVIR2R_LOG_INFO(data, "%s(%d) SRC index mismatch read=%u expected=%u",
				__FUNCTION__, data->counter_done,
				index_src, data->frame_index_src+1);
	}

	if (index_tgt != (data->frame_index_tgt+1)) {
		data->mismatch_frame_index_tgt++;
		AVIR2R_LOG_INFO(data, "%s(%d) TGT index mismatch read=%u expected=%u",
				__FUNCTION__, data->counter_done,
				index_tgt, data->frame_index_tgt+1);
	}

	data->frame_index_src=index_src;
	data->frame_index_tgt=index_tgt;

	data->counter_done++;

	if (data->counter_done>=data->deltas.number) {
		data->finished=1;
		AVIR2R_LOG_INFO(data, "%s(%d) finished",
				__FUNCTION__,
				atomic_read(&data->callback_done_ran));
		atomic_inc(&data->all_done);
		/* Wake up any task waiting for done */
		wake_up_interruptible(&data->wait_done);
	}
	else {
		if (data->verbose)
			AVIR2R_LOG_INFO(data, "%s(%d) #%d/%d",
				__FUNCTION__,
				atomic_read(&data->callback_done_ran),
				data->counter_done,
				data->deltas.number);
	}

	if (data->aborted) {
		AVIR2R_LOG_INFO(data, "%s() aborted",__FUNCTION__);
		atomic_inc(&data->all_done);
		/* Wake up any task waiting for done */
		wake_up_interruptible(&data->wait_done);
	}

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) OUT",__FUNCTION__,
			atomic_read(&data->callback_done_ran));

	spin_unlock(&data->lock);
}

static inline void * physical_to_virtual(struct avi_r2r_test *data, dma_addr_t phys)
{
	u32 offset = (u32)phys - (u32)data->phys_mem_start;
	return (void * )((u32)data->mem_start + offset);
}

static void my_callback_configure(struct avi_r2r_context *ctx,
				struct avi_dma_buffer *in_buf,
				struct avi_dma_buffer *out_buf,
				void *priv)
{
	struct avi_r2r_test		*data = ctx->private;
	struct avi_segment_format	out_fmt, in_fmt;
	int				ret;
	unsigned int			l, o;
	u32				*buffer;

	spin_lock(&data->lock);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) IN",__FUNCTION__,
			atomic_read(&data->callback_configure_ran));

	atomic_inc(&data->callback_configure_ran);
	in_fmt = data->formats.source ;
	out_fmt = data->formats.target ;

	switch (data->config_run_time) {
		case 1:
			in_fmt.colorspace = AVI_BT601_CSPACE;
			out_fmt.colorspace = AVI_BT709_CSPACE;
			if ((ret=avi_r2r_set_format(&data->context_id,
					&in_fmt,
					&out_fmt,
					0,
					0)) < 0)
				AVIR2R_LOG_ERROR(data,
					"%s(%d) avi_r2r_set_format(conv) fail (%d)",
					__FUNCTION__,
					atomic_read(&data->callback_configure_ran),
					ret);

		break;
		case 2:
			if ((ret=avi_r2r_setup_gam(&data->context_id,
					data->gamma_config.bypass,
					data->gamma_config.palette,
					data->gamma_config.comp_width,
					&data->gamma_config.cmap)) < 0)
				AVIR2R_LOG_ERROR(data,
					"%s(%d) avi_r2r_setup_gam fail (%d)",
					__FUNCTION__,
					atomic_read(&data->callback_configure_ran),
					ret);
		break;
		case 3:
			out_fmt.height /= 2;
			out_fmt.width /= 2;
			if ((ret=avi_r2r_set_format(&data->context_id,
					&in_fmt,
					&out_fmt,
					0,
					0)) < 0)
				AVIR2R_LOG_ERROR(data,
					"%s(%d) avi_r2r_set_format(out_size) fail (%d)",
					__FUNCTION__,
					atomic_read(&data->callback_configure_ran),
					ret);
		break;
		case 4:
			if (((unsigned)in_buf->priv & 1) == 0) {
				out_fmt.height /= 2;
				out_fmt.width /= 2;
				if ((ret=avi_r2r_set_format(&data->context_id,
						&in_fmt,
						&out_fmt,
						0,
						0)) < 0)
					AVIR2R_LOG_ERROR(data,
						"%s(%d) avi_r2r_set_format fail (%d)",
						__FUNCTION__,
						atomic_read(&data->callback_configure_ran),
						ret);
			}
		break;
		default:
		break;
	}

	if (data->tag_src) {
		buffer = (u32 *)physical_to_virtual(data, in_buf->plane0.dma_addr);
		for (l = 0; l < in_fmt.height; l++) {
			o = l * in_fmt.plane0.line_size / 4;
			buffer[o] = (u32)in_buf->priv;
		}
		o = in_fmt.height * in_fmt.plane0.line_size / 4;
		buffer[o-1] = (u32)in_buf->priv;
	}
	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) %08X %08X",__FUNCTION__,
			atomic_read(&data->callback_configure_ran),
			(unsigned)in_buf->priv, (unsigned)out_buf->priv);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) OUT",__FUNCTION__,
			atomic_read(&data->callback_configure_ran));

	spin_unlock(&data->lock);
}

static int run_batch(struct avi_r2r_test *data)
{
	int	ret=0, i;
	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s() IN",__FUNCTION__);

	data->ops.done  = &my_callback_done;
	data->ops.configure  = &my_callback_configure;

	data->counter_buf_queue = 0;
	data->counter_done = 0;
	data->frame_index_src = -1;
	data->frame_index_tgt = -1;
	data->buffer_in_idx = 0;
	data->buffer_out_idx = 0;

	for (i = 0; i < data->deltas.number ;i++ ) {
		batch_queue(data);
	}

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s() OUT",__FUNCTION__);

	return ret;
}

static void add_kt_period(struct avi_r2r_test *data,
		ktime_t kt_begin,
		ktime_t kt_end,
		int over)
	{
	data->kt_period[data->kt_period_index].begin   = kt_begin;
	data->kt_period[data->kt_period_index].end     = kt_end;
	data->kt_period[data->kt_period_index].overrun = over;

	data->kt_period_counter++;
	data->kt_period_index++;
	if (data->kt_period_index >= nr_timings_saved)
		data->kt_period_index = 0;

	if (data->kt_period_nr < nr_timings_saved)
		data->kt_period_nr++;
}

/* Timer callback: called at the end of period. */
static enum hrtimer_restart periodic_timer(struct hrtimer *timer)
{
	struct avi_dma_buffer	stats = {
		.plane0.dma_addr = 0,
	};
	struct avi_dma_buffer	source;
	struct avi_dma_buffer	target;
	struct avi_r2r_test	*data;
	unsigned long		tjnow;
	ktime_t			kt_now, kt_now1;
	int			err;
	enum hrtimer_restart	ret = HRTIMER_NORESTART;

	data = container_of(timer, struct avi_r2r_test, timer);

	if (data->counter_buf_queue<data->deltas.number
			&& !data->finished
			&& !data->aborted) {
		tjnow = jiffies;
		kt_now = hrtimer_cb_get_time(&data->timer);
		err = hrtimer_forward(&data->timer,
				kt_now,
				data->ktime_period_ns);
		if (data->verbose)
			AVIR2R_LOG_INFO(data,
					"%s() "
					"jiffies %lu ;"
					" ret: %d ;"
					" ktnsec: %lld ;"
					" count %d",
					__FUNCTION__,
					tjnow,
					err,
					ktime_to_ns(kt_now),
					data->counter_buf_queue);

		if (0 == data->buffer_in_idx)
			data->l_buffers_config.source = data->buffers_config.source;
		if (0 == data->buffer_out_idx)
			data->l_buffers_config.target = data->buffers_config.target;

		setup_dma(&source,
			  &data->formats.source,
			  &data->l_buffers_config.source,
			  data->images_config.source.rect.left,
			  data->images_config.source.rect.top);

		setup_dma(&target,
			  &data->formats.target,
			  &data->l_buffers_config.target,
			  data->images_config.target.rect.left,
			  data->images_config.target.rect.top);

		source.priv = (void *)(0x50000000|data->counter_buf_queue);
		target.priv = (void *)(0x70000000|data->counter_buf_queue);

		ret=avi_r2r_buf_queue(&data->context_id,
				&source,
				&target,
				&stats);
		if (ret>=0) {
			if (data->verbose)
				AVIR2R_LOG_INFO(data, "%s() #%d/%d"
						" Buf Queue successful",
						__FUNCTION__,
						data->counter_buf_queue,
				data->deltas.number);

			data->counter_buf_queue++;
			data->images_config.source.rect.left +=
					data->deltas.source.left;
			data->images_config.source.rect.top  +=
					data->deltas.source.top;
			data->images_config.target.rect.left +=
					data->deltas.target.left;
			data->images_config.target.rect.top  +=
					data->deltas.target.top;

			data->buffer_in_idx++;
			if (data->buffer_in_idx > data->src_nbuffers)
				data->buffer_in_idx = 0;
			else {
				data->l_buffers_config.source.plane_0 += data->images_config.source.plane0.frame_size;
				data->l_buffers_config.source.plane_1 += data->images_config.source.plane1.frame_size;
			}
			data->buffer_out_idx++;
			if (data->buffer_out_idx > data->tgt_nbuffers)
				data->buffer_out_idx = 0;
			else {
				data->l_buffers_config.target.plane_0 += data->images_config.target.plane0.frame_size;
				data->l_buffers_config.target.plane_1 += data->images_config.target.plane1.frame_size;
			}

			ret=HRTIMER_RESTART;
		}
		else {
			AVIR2R_LOG_INFO(data,"%s() #%d/%d "
					"Buf Queue failed (%d). Abort !!",
					__FUNCTION__,
					data->counter_buf_queue,
					data->deltas.number,
					ret);
			data->aborted=1;
		}

		kt_now1 = hrtimer_cb_get_time(&data->timer);
		add_kt_period(data,kt_now,kt_now1,err);
	}
	else
		AVIR2R_LOG_INFO(data,
			"%s() NORESTART #%d/%d end:%d abort:%d",
			__FUNCTION__,
			data->counter_buf_queue,
			data->deltas.number,
			data->finished, data->aborted);

	if (data->aborted) {
		AVIR2R_LOG_INFO(data, "%s() aborted",__FUNCTION__);
		atomic_inc(&data->all_done);
		/* Wake up any task waiting for done */
		wake_up_interruptible(&data->wait_done);
	}

	return ret;
}

static void my_periodic_callback_done(struct avi_r2r_context *ctx,
				struct avi_dma_buffer *in_buf,
				struct avi_dma_buffer *out_buf,
				struct avi_dma_buffer *stats_buf,
				void *priv)
{
	struct avi_r2r_test *data=ctx->private;
	unsigned int index_src, index_tgt;

	spin_lock(&data->lock);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) IN",__FUNCTION__,
			atomic_read(&data->callback_done_ran));

	atomic_inc(&data->callback_done_ran);

	index_src=(unsigned int)in_buf->priv & 0x00FFFFFF;
	index_tgt=(unsigned int)out_buf->priv & 0x00FFFFFF;

	if (index_src != index_tgt) {
		data->mismatch_frame_index++;
		AVIR2R_LOG_INFO(data, "%s(%d) index mismatch src=%u tgt=%u",
				__FUNCTION__, data->counter_done,
				index_src, index_tgt);
	}

	if (index_src != (data->frame_index_src+1)) {
		data->mismatch_frame_index_src++;
		AVIR2R_LOG_INFO(data, "%s(%d) SRC index mismatch read=%u expected=%u",
				__FUNCTION__, data->counter_done,
				index_src, data->frame_index_src+1);
	}

	if (index_tgt != (data->frame_index_tgt+1)) {
		data->mismatch_frame_index_tgt++;
		AVIR2R_LOG_INFO(data, "%s(%d) TGT index mismatch read=%u expected=%u",
				__FUNCTION__, data->counter_done,
				index_tgt, data->frame_index_tgt+1);
	}

	data->frame_index_src=index_src;
	data->frame_index_tgt=index_tgt;

	data->counter_done++;

	if (data->counter_done>=data->deltas.number) {
		data->finished=1;
		AVIR2R_LOG_INFO(data, "%s(%d) finished",
				__FUNCTION__,
				atomic_read(&data->callback_done_ran));
		atomic_inc(&data->all_done);
		/* Wake up any task waiting for done */
		wake_up_interruptible(&data->wait_done);
	}
	else
		if (data->verbose)
			AVIR2R_LOG_INFO(data, "%s(%d) #%d/%d",
				__FUNCTION__,
				atomic_read(&data->callback_done_ran),
				data->counter_done,
				data->deltas.number);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) OUT",__FUNCTION__,
			atomic_read(&data->callback_done_ran));

	spin_unlock(&data->lock);
}

static void my_periodic_callback_configure(struct avi_r2r_context *ctx,
				struct avi_dma_buffer *in_buf,
				struct avi_dma_buffer *out_buf,
				void *priv)
{
	struct avi_r2r_test		*data = ctx->private;
	struct avi_segment_format	out_fmt, in_fmt;
	u32				*buffer;
	unsigned int			l, o;
	int				ret;

	spin_lock(&data->lock);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) IN",__FUNCTION__,
			atomic_read(&data->callback_configure_ran));

	atomic_inc(&data->callback_configure_ran);

	in_fmt = data->formats.source ;
	out_fmt = data->formats.target ;
	switch (data->config_run_time) {
		case 1:
			in_fmt.colorspace = AVI_BT601_CSPACE;
			out_fmt.colorspace = AVI_BT709_CSPACE;
			if ((ret=avi_r2r_set_format(&data->context_id,
					&in_fmt,
					&out_fmt,
					0,
					0)) < 0)
				AVIR2R_LOG_ERROR(data,
					"%s(%d) avi_r2r_set_format(conv) fail (%d)",
					__FUNCTION__,
					atomic_read(&data->callback_configure_ran),
					ret);

		break;
		case 2:
			if ((ret=avi_r2r_setup_gam(&data->context_id,
					data->gamma_config.bypass,
					data->gamma_config.palette,
					data->gamma_config.comp_width,
					&data->gamma_config.cmap)) < 0)
				AVIR2R_LOG_ERROR(data,
					"%s(%d) avi_r2r_setup_gam fail (%d)",
					__FUNCTION__,
					atomic_read(&data->callback_configure_ran),
					ret);
		break;
		case 3:
			out_fmt.height /= 2;
			out_fmt.width /= 2;
			if ((ret=avi_r2r_set_format(&data->context_id,
					&in_fmt,
					&out_fmt,
					0,
					0)) < 0)
				AVIR2R_LOG_ERROR(data,
					"%s(%d) avi_r2r_set_format fail (%d)",
					__FUNCTION__,
					atomic_read(&data->callback_configure_ran),
					ret);
		break;
		case 4:
			if (((unsigned)in_buf->priv & 1) == 0) {
				out_fmt.height /= 2;
				out_fmt.width /= 2;
				if ((ret=avi_r2r_set_format(&data->context_id,
						&in_fmt,
						&out_fmt,
						0,
						0)) < 0)
					AVIR2R_LOG_ERROR(data,
						"%s(%d) avi_r2r_set_format fail (%d)",
						__FUNCTION__,
						atomic_read(&data->callback_configure_ran),
						ret);
			}
		break;
		default:
		break;
	}

	if (data->tag_src) {
		buffer = (u32 *)physical_to_virtual(data, in_buf->plane0.dma_addr);
		for (l = 0; l < in_fmt.height; l++) {
			o = l * in_fmt.plane0.line_size / 4;
			buffer[o] = (u32)in_buf->priv;
		}
		o = in_fmt.height * in_fmt.plane0.line_size / 4;
		buffer[o-1] = (u32)in_buf->priv;
	}

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) %08X %08X",__FUNCTION__,
			atomic_read(&data->callback_configure_ran),
			(unsigned)in_buf->priv, (unsigned)out_buf->priv);

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s(%d) OUT",__FUNCTION__,
			atomic_read(&data->callback_configure_ran));

	spin_unlock(&data->lock);
}

static int run_periodic(struct avi_r2r_test *data)
{
	int ret=0;
	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s() IN",__FUNCTION__);

	data->ops.done  = &my_periodic_callback_done;
	data->ops.configure  = &my_periodic_callback_configure;

	data->ktime_period_ns = ktime_set(0, data->period*1000);
	data->timer.function = &periodic_timer;
	data->done_periodic = 0;
	data->counter_buf_queue = 0;
	data->counter_done = 0;
	data->frame_index_src = -1;
	data->frame_index_tgt = -1;
	data->buffer_in_idx = 0;
	data->buffer_out_idx = 0;

	if (!hrtimer_active(&data->timer)) {
		unsigned long tjnow=jiffies;
		struct timespec ts,ts1;
		ktime_get_ts(&ts);
		hrtimer_start(&data->timer,
				data->ktime_period_ns,
				HRTIMER_MODE_REL);
		ktime_get_ts(&ts1);
		add_kt_period(data,
				timespec_to_ktime(ts),
				timespec_to_ktime(ts1),
				-1);

		if (data->verbose)
			AVIR2R_LOG_INFO(data,
				"%s() hrtimer_start %lu %lld",
				__FUNCTION__,
				tjnow,
				ktime_to_ns(timespec_to_ktime(ts)));
	}

	if (data->verbose)
		AVIR2R_LOG_INFO(data, "%s() OUT",__FUNCTION__);

	return ret;
}

static void get_test_config(struct avi_r2r_test *data)
{
	int i,avi_cmap_sz;

	data->images_config.source.pixel_format = avi_pixfmt_by_id(src_pixfmt);
	data->images_config.source.frame.width=src_frame_width;
	data->images_config.source.frame.height=src_frame_height;
	data->images_config.source.rect.width=src_width;
	data->images_config.source.rect.height=src_height;
	data->images_config.source.rect.left=src_left;
	data->images_config.source.rect.top=src_top;
	data->src_nbuffers=src_nbuffers;

	data->images_config.target.pixel_format= avi_pixfmt_by_id(tgt_pixfmt);
	data->images_config.target.frame.width=tgt_frame_width;
	data->images_config.target.frame.height=tgt_frame_height;
	data->images_config.target.rect.width=tgt_width;
	data->images_config.target.rect.height=tgt_height;
	data->images_config.target.rect.left=tgt_left;
	data->images_config.target.rect.top=tgt_top;
	data->tgt_nbuffers=tgt_nbuffers;

	data->buffers_config.source.plane_0=src_sa0;
	data->buffers_config.source.plane_1=src_sa1;

	data->buffers_config.target.plane_0=tgt_sa0;
	data->buffers_config.target.plane_1=tgt_sa1;

	data->deltas.source.left=src_delta_left;
	data->deltas.source.top=src_delta_top;
	data->deltas.target.left=tgt_delta_left;
	data->deltas.target.top=tgt_delta_top;
	data->deltas.number=delta_count;

	data->time_out=time_out;
	data->period=period;
	data->startup_delay=startup_delay;

	data->images_config.source.colorspace=src_cspace;
	data->images_config.target.colorspace=tgt_cspace;

	memset(&data->gamma_config.cmap,0,sizeof(struct avi_cmap));
	avi_cmap_sz=AVI_CMAP_SZ;
#ifdef AVI_BACKWARD_COMPAT
	if (avi_get_revision() < AVI_REVISION_3)
		avi_cmap_sz=AVI_CMAP_SZ/4;
#endif /* AVI_BACKWARD_COMPAT */

	for (i=0;i<avi_cmap_sz;i++) {
		data->gamma_config.cmap.red[i]=i;
		data->gamma_config.cmap.green[i]=i;
		data->gamma_config.cmap.blue[i]=i;
	}
	data->gamma_config.bypass=gam_bypass;
	data->gamma_config.palette=gam_palette;
	data->gamma_config.comp_width=0;

	data->config_run_time = config_run_time;
	data->tag_src = tag_src;


	data->requirements[0]=(AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT);
	data->requirements[1]=(AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT|AVI_CAP_CONV|AVI_CAP_GAM);
	data->requirements[2]=(AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT|AVI_CAP_CONV|AVI_CAP_SCAL);
	data->requirements[3]=(AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT|AVI_CAP_CONV|AVI_CAP_SCAL|AVI_CAP_PLANAR);
	data->requirements[4]=(AVI_CAP_DMA_IN|AVI_CAP_DMA_OUT|AVI_CAP_CONV);

	data->run=0;
	data->running=0;
	data->verbose=verbose;
	data->kt_period_index=0;
	data->kt_period_counter=0;
	data->kt_period_nr=0;
}

static void print_test_config(struct avi_r2r_test *data)
{
	AVIR2R_LOG_INFO(data,
		"\n"
		" - Source: %s[%d] %dx%d (%d,%d) %dx%d 0x%p|0x%p\n"
		" - Target: %s[%d] %dx%d (%d,%d) %dx%d 0x%p|0x%p\n"
		" - Delta: Source (%d,%d), Target (%d,%d), Number %d",
	        avi_debug_pixfmt_to_str(data->images_config.source.pixel_format),
		data->images_config.source.pixel_format.id,
		data->images_config.source.frame.width,
		data->images_config.source.frame.height,
		data->images_config.source.rect.left,
		data->images_config.source.rect.top,
		data->images_config.source.rect.width,
		data->images_config.source.rect.height,
		(void *)data->buffers_config.source.plane_0,
		(void *)data->buffers_config.source.plane_1,

	        avi_debug_pixfmt_to_str(data->images_config.target.pixel_format),
		data->images_config.target.pixel_format.id,
		data->images_config.target.frame.width,
		data->images_config.target.frame.height,
		data->images_config.target.rect.left,
		data->images_config.target.rect.top,
		data->images_config.target.rect.width,
		data->images_config.target.rect.height,
		(void *)data->buffers_config.target.plane_0,
		(void *)data->buffers_config.target.plane_1,

		data->deltas.source.left,
		data->deltas.source.top,
		data->deltas.target.left,
		data->deltas.target.top,
		data->deltas.number);

}

static void cancel_timer(struct avi_r2r_test *data)
{
	int ret_cancel = 0;

	if (hrtimer_callback_running(&data->timer)) {
		AVIR2R_LOG_WARN(data,
				"hrtimer callback is running");
	}
	if (hrtimer_active(&data->timer) != 0) {
		ret_cancel = hrtimer_cancel(&data->timer);
		AVIR2R_LOG_INFO(data,
				"active hrtimer cancelled: %d (%d, %d)",
				ret_cancel,
				data->counter_buf_queue,
				data->counter_done);
	}
	if (hrtimer_is_queued(&data->timer) != 0) {
		ret_cancel = hrtimer_cancel(&data->timer);
		AVIR2R_LOG_INFO(data,
				"queued hrtimer cancelled: %d (%d, %d)",
				ret_cancel,
				data->counter_buf_queue,
				data->counter_done);
	}
}

static void run_test(struct avi_r2r_test *data, int new_run)
{
	int ret,seq;
//	int count;

	if (new_run==0) {
		AVIR2R_LOG_INFO(data,"Run is zero!!");
		return;
	}

	if (new_run>NR_REQUIREMENTS) new_run=NR_REQUIREMENTS;
	data->running=data->run=new_run;

	AVIR2R_LOG_INFO(data,"Running test #%d",data->running);
	print_test_config(data);

	data->context_id.private=data;

	data->kt_period_counter = 0;
	data->kt_period_index = 0;
	data->kt_period_nr = 0;
	memset(data->kt_period,0,sizeof(data->kt_period));

	if (data->period == 0) {
		data->ops.done  = &my_callback_done;
		data->ops.configure  = &my_callback_configure;
	}
	else {
		data->ops.done  = &my_periodic_callback_done;
		data->ops.configure  = &my_periodic_callback_configure;
		AVIR2R_LOG_INFO(data,"Period %lu microseconds",data->period);
	}
	if ((ret=avi_r2r_request(&data->context_id,
			data->dev,
			data->requirements[data->running-1],
			&data->ops))==0) {
		if (data->verbose)
			AVIR2R_LOG_INFO(data,"Request successful");

		for (seq=0;seq<10 && ret==0;seq++)
		switch (seq) {
			case 0:
				setup_formats(data);
				ret=avi_r2r_set_format(&data->context_id,
							&data->formats.source,
							&data->formats.target,
							0,
							0);
				if (ret==0)
				{
					if (data->verbose)
						AVIR2R_LOG_INFO(data,"Set formats successful");
				}
				else
					AVIR2R_LOG_INFO(data,"Set formats failed (%d)",ret);
			break;

			case 1:
			break;

			case 2:
				if (data->requirements[data->running-1] & AVI_CAP_GAM) {
					ret=avi_r2r_setup_gam(&data->context_id,
							data->gamma_config.bypass,
							data->gamma_config.palette,
							data->gamma_config.comp_width,
							&data->gamma_config.cmap);
					if (ret==0) {
						if (data->verbose)
							AVIR2R_LOG_INFO(data,"Setup Gamma successful");
					}
					else
						AVIR2R_LOG_INFO(data,"Setup Gamma failed (%d)",ret);
				}
			break;
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			default:
				break;

			case 8:
				if (data->startup_delay)
					msleep(data->startup_delay);
				break;

			case 9:
				data->finished=0;
				data->aborted=0;
				atomic_set(&data->done_counter,
					   atomic_read(&data->all_done));

				if (data->period==0)
					ret=run_batch(data);
				else
					ret=run_periodic(data);

				if (ret>=0) {
					if (data->verbose)
						AVIR2R_LOG_INFO(data,"Run successful");
					ret=wait_for_done(data);
					if (ret!=0) {
						data->aborted=1;
						AVIR2R_LOG_INFO(data,"Done failed (%d)",ret);
						break;
					}
					else
						report_status(data);
				}
				else
					AVIR2R_LOG_INFO(data,"Run failed (%d)",ret);
			break;
		}

//		count=0;
//		do {
			if ((ret=avi_r2r_destroy(&data->context_id))==0) {
				if (data->verbose)
					AVIR2R_LOG_INFO(data,
							"Destroy successful");
			}
			else {
				AVIR2R_LOG_INFO(data,"Destroy failed (%d)",ret);
//				if (count<20)
//				{
//					msleep_interruptible(10);
//					count++;
//				}
//				else
//				{
//					AVIR2R_LOG_INFO(data,
//							"cannot destroy");
//					break;
//				}
			}
//		} while(ret!=0);
	}
	else
		AVIR2R_LOG_INFO(data,"Request failed (%d)",ret);

	if (data->period != 0)
		cancel_timer(data);

	data->running=0;
	data->run=0;
}

/****************************************************************/
static int src_frame_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%ux%u\n",
		data->images_config.source.frame.width,
		data->images_config.source.frame.height);

	return 0;
}

static ssize_t src_frame_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			width,height;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	width = simple_strtoul(_buf, &last, 0);
	if (*last == 'x')
	{
		last++;
		height = simple_strtoul(last, &last, 0);

		data->images_config.source.frame.width=width;
		data->images_config.source.frame.height=height;
	}
	return size;
}

static int src_frame_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_frame_show ,inode->i_private);
}

static const struct file_operations src_frame_debugfs_fops = {
	.open		= src_frame_open,
	.read		= seq_read,
	.write		= src_frame_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_frame_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%ux%u\n",
		data->images_config.target.frame.width,
		data->images_config.target.frame.height);

	return 0;
}

static ssize_t tgt_frame_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			width,height;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	width = simple_strtoul(_buf, &last, 0);
	if (*last == 'x') {
		last++;
		height = simple_strtoul(last, &last, 0);

		data->images_config.target.frame.width=width;
		data->images_config.target.frame.height=height;
	}
	return size;
}

static int tgt_frame_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_frame_show ,inode->i_private);
}

static const struct file_operations tgt_frame_debugfs_fops = {
	.open		= tgt_frame_open,
	.read		= seq_read,
	.write		= tgt_frame_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_rect_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%ux%u\n",
		data->images_config.source.rect.width,
		data->images_config.source.rect.height);

	return 0;
}

static ssize_t src_rect_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			width,height;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	width = simple_strtoul(_buf, &last, 0);
	if (*last == 'x') {
		last++;
		height = simple_strtoul(last, &last, 0);

		data->images_config.source.rect.width=width;
		data->images_config.source.rect.height=height;
	}
	return size;
}

static int src_rect_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_rect_show ,inode->i_private);
}

static const struct file_operations src_rect_debugfs_fops = {
	.open		= src_rect_open,
	.read		= seq_read,
	.write		= src_rect_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_rect_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%ux%u\n",
		data->images_config.target.rect.width,
		data->images_config.target.rect.height);

	return 0;
}

static ssize_t tgt_rect_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			width,height;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	width = simple_strtoul(_buf, &last, 0);
	if (*last == 'x') {
		last++;
		height = simple_strtoul(last, &last, 0);

		data->images_config.target.rect.width=width;
		data->images_config.target.rect.height=height;
	}
	return size;
}

static int tgt_rect_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_rect_show ,inode->i_private);
}

static const struct file_operations tgt_rect_debugfs_fops = {
	.open		= tgt_rect_open,
	.read		= seq_read,
	.write		= tgt_rect_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_pos_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%u,%u\n",
		data->images_config.source.rect.left,
		data->images_config.source.rect.top);

	return 0;
}

static ssize_t src_pos_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			left,top;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	left = simple_strtoul(_buf, &last, 0);
	if (*last == ',') {
		last++;
		top = simple_strtoul(last, &last, 0);

		data->images_config.source.rect.left=left;
		data->images_config.source.rect.top=top;
	}
	return size;
}

static int src_pos_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_pos_show ,inode->i_private);
}

static const struct file_operations src_pos_debugfs_fops = {
	.open		= src_pos_open,
	.read		= seq_read,
	.write		= src_pos_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_pos_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%u,%u\n",
		data->images_config.target.rect.left,
		data->images_config.target.rect.top);

	return 0;
}

static ssize_t tgt_pos_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			left,top;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	left = simple_strtoul(_buf, &last, 0);
	if (*last == ',') {
		last++;
		top = simple_strtoul(last, &last, 0);

		data->images_config.target.rect.left=left;
		data->images_config.target.rect.top=top;
	}
	return size;
}

static int tgt_pos_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_pos_show ,inode->i_private);
}

static const struct file_operations tgt_pos_debugfs_fops = {
	.open		= tgt_pos_open,
	.read		= seq_read,
	.write		= tgt_pos_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_addr_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "0x%08X,0x%08X\n",
		data->buffers_config.source.plane_0,
		data->buffers_config.source.plane_1);

	return 0;
}

static ssize_t src_addr_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			addr[2];

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	addr[0] = simple_strtoul(_buf, &last, 0);
	if (*last == ',') {
		last++;
		addr[1] = simple_strtoul(last, &last, 0);

		data->buffers_config.source.plane_0=addr[0];
		data->buffers_config.source.plane_1=addr[1];
	}
	return size;
}

static int src_addr_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_addr_show ,inode->i_private);
}

static const struct file_operations src_addr_debugfs_fops = {
	.open		= src_addr_open,
	.read		= seq_read,
	.write		= src_addr_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_addr_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "0x%08X,0x%08X\n",
		data->buffers_config.target.plane_0,
		data->buffers_config.target.plane_1);

	return 0;
}

static ssize_t tgt_addr_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			addr[2];

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	addr[0] = simple_strtoul(_buf, &last, 0);
	if (*last == ',') {
		last++;
		addr[1] = simple_strtoul(last, &last, 0);

		data->buffers_config.target.plane_0=addr[0];
		data->buffers_config.target.plane_1=addr[1];
	}
	return size;
}

static int tgt_addr_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_addr_show ,inode->i_private);
}

static const struct file_operations tgt_addr_debugfs_fops = {
	.open		= tgt_addr_open,
	.read		= seq_read,
	.write		= tgt_addr_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_pixfmt_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d (%s)\n",
		data->images_config.source.pixel_format.id,
	        avi_debug_pixfmt_to_str(data->images_config.source.pixel_format));

	return 0;
}

static ssize_t src_pixfmt_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->images_config.source.pixel_format = avi_pixfmt_by_id(fmt);
	return size;
}

static int src_pixfmt_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_pixfmt_show ,inode->i_private);
}

static const struct file_operations src_pixfmt_debugfs_fops = {
	.open		= src_pixfmt_open,
	.read		= seq_read,
	.write		= src_pixfmt_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_pixfmt_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d (%s)\n",
		data->images_config.target.pixel_format.id,
	        avi_debug_pixfmt_to_str(data->images_config.target.pixel_format));

	return 0;
}

static ssize_t tgt_pixfmt_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->images_config.target.pixel_format = avi_pixfmt_by_id(fmt);
	return size;
}

static int tgt_pixfmt_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_pixfmt_show ,inode->i_private);
}

static const struct file_operations tgt_pixfmt_debugfs_fops = {
	.open		= tgt_pixfmt_open,
	.read		= seq_read,
	.write		= tgt_pixfmt_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_delta_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%u,%u\n",
		data->deltas.source.left,
		data->deltas.source.top);

	return 0;
}

static ssize_t src_delta_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			left,top;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	left = simple_strtoul(_buf, &last, 0);
	if (*last == ',') {
		last++;
		top = simple_strtoul(last, &last, 0);

		data->deltas.source.left=left;
		data->deltas.source.top=top;
	}
	return size;
}

static int src_delta_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_delta_show ,inode->i_private);
}

static const struct file_operations src_delta_debugfs_fops = {
	.open		= src_delta_open,
	.read		= seq_read,
	.write		= src_delta_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_delta_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%u,%u\n",
		data->deltas.target.left,
		data->deltas.target.top);

	return 0;
}

static ssize_t tgt_delta_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			left,top;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	left = simple_strtoul(_buf, &last, 0);
	if (*last == ',') {
		last++;
		top = simple_strtoul(last, &last, 0);

		data->deltas.target.left=left;
		data->deltas.target.top=top;
	}
	return size;
}

static int tgt_delta_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_delta_show ,inode->i_private);
}

static const struct file_operations tgt_delta_debugfs_fops = {
	.open		= tgt_delta_open,
	.read		= seq_read,
	.write		= tgt_delta_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int delta_count_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->deltas.number);

	return 0;
}

static ssize_t delta_count_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->deltas.number=fmt;
	return size;
}

static int delta_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, delta_count_show ,inode->i_private);
}

static const struct file_operations delta_count_debugfs_fops = {
	.open		= delta_count_open,
	.read		= seq_read,
	.write		= delta_count_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_cspace_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d (%s)\n", data->images_config.source.colorspace,
	           avi_debug_colorspace_to_str(data->images_config.source.colorspace));

	return 0;
}

static ssize_t src_cspace_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->images_config.source.colorspace=fmt;
	return size;
}

static int src_cspace_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_cspace_show ,inode->i_private);
}

static const struct file_operations src_cspace_debugfs_fops = {
	.open		= src_cspace_open,
	.read		= seq_read,
	.write		= src_cspace_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_cspace_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d (%s)\n", data->images_config.target.colorspace,
	           avi_debug_colorspace_to_str(data->images_config.target.colorspace));

	return 0;
}

static ssize_t tgt_cspace_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->images_config.target.colorspace=fmt;
	return size;
}

static int tgt_cspace_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_cspace_show ,inode->i_private);
}

static const struct file_operations tgt_cspace_debugfs_fops = {
	.open		= tgt_cspace_open,
	.read		= seq_read,
	.write		= tgt_cspace_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int src_nbuffers_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->src_nbuffers);

	return 0;
}

static ssize_t src_nbuffers_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->src_nbuffers=fmt;
	return size;
}

static int src_nbuffers_open(struct inode *inode, struct file *file)
{
	return single_open(file, src_nbuffers_show ,inode->i_private);
}

static const struct file_operations src_nbuffers_debugfs_fops = {
	.open		= src_nbuffers_open,
	.read		= seq_read,
	.write		= src_nbuffers_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tgt_nbuffers_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->tgt_nbuffers);

	return 0;
}

static ssize_t tgt_nbuffers_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->tgt_nbuffers=fmt;
	return size;
}

static int tgt_nbuffers_open(struct inode *inode, struct file *file)
{
	return single_open(file, tgt_nbuffers_show ,inode->i_private);
}

static const struct file_operations tgt_nbuffers_debugfs_fops = {
	.open		= tgt_nbuffers_open,
	.read		= seq_read,
	.write		= tgt_nbuffers_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int run_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->run);

	return 0;
}

static ssize_t run_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (data->running)
	{
		AVIR2R_LOG_INFO(data,"Already running test #%d", data->running);
		return size;
	}

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	run_test(data, fmt);

	return size;
}

static int run_open(struct inode *inode, struct file *file)
{
	return single_open(file, run_show ,inode->i_private);
}

static const struct file_operations run_debugfs_fops = {
	.open		= run_open,
	.read		= seq_read,
	.write		= run_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int running_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->running);

	return 0;
}

static int running_open(struct inode *inode, struct file *file)
{
	return single_open(file, running_show ,inode->i_private);
}

static const struct file_operations running_debugfs_fops = {
	.open		= running_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int gam_bypass_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->gamma_config.bypass);

	return 0;
}

static ssize_t gam_bypass_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->gamma_config.bypass=!!fmt;
	return size;
}

static int gam_bypass_open(struct inode *inode, struct file *file)
{
	return single_open(file, gam_bypass_show ,inode->i_private);
}

static const struct file_operations gam_bypass_debugfs_fops = {
	.open		= gam_bypass_open,
	.read		= seq_read,
	.write		= gam_bypass_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int gam_palette_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->gamma_config.palette);

	return 0;
}

static ssize_t gam_palette_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);
	data->gamma_config.palette=!!fmt;
	return size;
}

static int gam_palette_open(struct inode *inode, struct file *file)
{
	return single_open(file, gam_palette_show ,inode->i_private);
}

static const struct file_operations gam_palette_debugfs_fops = {
	.open		= gam_palette_open,
	.read		= seq_read,
	.write		= gam_palette_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int mismatchs_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "SRC=%u, TGT=%u, %u\n",
			data->mismatch_frame_index_src,
			data->mismatch_frame_index_tgt,
			data->mismatch_frame_index);

	return 0;
}

static int mismatchs_open(struct inode *inode, struct file *file)
{
	return single_open(file, mismatchs_show ,inode->i_private);
}

static const struct file_operations mismatchs_debugfs_fops = {
	.open		= mismatchs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int counters_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d, %d\n",
			data->counter_buf_queue,
			data->counter_done);

	return 0;
}

static int counters_open(struct inode *inode, struct file *file)
{
	return single_open(file, counters_show ,inode->i_private);
}

static const struct file_operations counters_debugfs_fops = {
	.open		= counters_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int verbose_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->verbose);

	return 0;
}

static ssize_t verbose_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->verbose=fmt;

	return size;
}

static int verbose_open(struct inode *inode, struct file *file)
{
	return single_open(file, verbose_show ,inode->i_private);
}

static const struct file_operations verbose_debugfs_fops = {
	.open		= verbose_open,
	.read		= seq_read,
	.write		= verbose_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};


/*-----------*/
static int time_out_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->time_out);

	return 0;
}

static ssize_t time_out_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->time_out=fmt;

	return size;
}

static int time_out_open(struct inode *inode, struct file *file)
{
	return single_open(file, time_out_show ,inode->i_private);
}

static const struct file_operations time_out_debugfs_fops = {
	.open		= time_out_open,
	.read		= seq_read,
	.write		= time_out_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int period_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%lu\n", data->period);

	return 0;
}

static ssize_t period_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->period=fmt;

	return size;
}

static int period_open(struct inode *inode, struct file *file)
{
	return single_open(file, period_show ,inode->i_private);
}

static const struct file_operations period_debugfs_fops = {
	.open		= period_open,
	.read		= seq_read,
	.write		= period_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int config_run_time_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->config_run_time);

	return 0;
}

static ssize_t config_run_time_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->config_run_time=fmt;

	return size;
}

static int config_run_time_open(struct inode *inode, struct file *file)
{
	return single_open(file, config_run_time_show ,inode->i_private);
}

static const struct file_operations config_run_time_debugfs_fops = {
	.open		= config_run_time_open,
	.read		= seq_read,
	.write		= config_run_time_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int tag_src_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->tag_src);

	return 0;
}

static ssize_t tag_src_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->tag_src=fmt;

	return size;
}

static int tag_src_open(struct inode *inode, struct file *file)
{
	return single_open(file, tag_src_show ,inode->i_private);
}

static const struct file_operations tag_src_debugfs_fops = {
	.open		= tag_src_open,
	.read		= seq_read,
	.write		= tag_src_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int startup_delay_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->startup_delay);

	return 0;
}

static ssize_t startup_delay_store(struct file *file,
                                      const char __user *buf,
                                      size_t size,
                                      loff_t *ppos)
{
	struct seq_file         *m = file->private_data;
	struct avi_r2r_test *data = m->private;
	char			*last;
	char			_buf[32];
	u32			fmt;

	if (*ppos > 0)
		return -EOPNOTSUPP;

	if (size > ARRAY_SIZE(_buf) - 1)
		return -ERANGE;

	if (copy_from_user(_buf, buf, size))
		return -EFAULT;

	_buf[size] = '\0';

	fmt = simple_strtoul(_buf, &last, 0);

	data->startup_delay=fmt;

	return size;
}

static int startup_delay_open(struct inode *inode, struct file *file)
{
	return single_open(file, startup_delay_show ,inode->i_private);
}

static const struct file_operations startup_delay_debugfs_fops = {
	.open		= startup_delay_open,
	.read		= seq_read,
	.write		= startup_delay_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int timings_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;
	struct kt_period kt_prev;
	s64 delta;

	int i,n,c;

	n=data->kt_period_nr;
	seq_printf(s, "%d/%d (periodic)\n", n, data->kt_period_counter);
	if (n>0) {
		i = data->kt_period_index-n;
		if (i<0) i += nr_timings_saved;
		kt_prev = data->kt_period[i];
		for (c=0;c<n;c++) {
			delta=ktime_us_delta(data->kt_period[i].begin,
						kt_prev.begin);
			seq_printf(s,
				"client-%d: %18lld, %18lld, %2d,"
				" %10lld microseconds, %+6lld\n",
				data->platform_device->id,
				ktime_to_us(data->kt_period[i].begin),
				ktime_to_us(data->kt_period[i].end),
				data->kt_period[i].overrun,
				delta,
				delta-data->period);
			kt_prev=data->kt_period[i];
			i++;
			if (i>=nr_timings_saved) i=0;
		}
	}
	return 0;
}


static int timings_open(struct inode *inode, struct file *file)
{
	return single_open(file, timings_show ,inode->i_private);
}

static const struct file_operations timings_debugfs_fops = {
	.open		= timings_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int mem_start_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "0x%08X\n",
			(unsigned int)data->platform_device->resource->start);

	return 0;
}

static int mem_start_open(struct inode *inode, struct file *file)
{
	return single_open(file, mem_start_show ,inode->i_private);
}

static const struct file_operations mem_start_debugfs_fops = {
	.open		= mem_start_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/*-----------*/
static int mem_size_show(struct seq_file *s, void *unused)
{
	struct avi_r2r_test *data = s->private;

	seq_printf(s, "%d\n", data->mem_size);

	return 0;
}

static int mem_size_open(struct inode *inode, struct file *file)
{
	return single_open(file, mem_size_show ,inode->i_private);
}

static const struct file_operations mem_size_debugfs_fops = {
	.open		= mem_size_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/****************************************************************/

static void avi_r2r_test_data_delete(struct avi_r2r_test *data)
{
	if (!IS_ERR_OR_NULL(data->mem_start))
		iounmap(data->mem_start);

	if (!IS_ERR_OR_NULL((void *)data->vmem->start))
		release_mem_region(data->platform_device->resource->start,
			resource_size(data->platform_device->resource));

	if (!IS_ERR_OR_NULL(data->debugfs_root))
		debugfs_remove_recursive(data->debugfs_root);
	data->debugfs_root = NULL;

	cancel_timer(data);
}

#define MY_DEBUGFS_REGISTER(_name)				\
	file = debugfs_create_file(#_name,			\
				   S_IRWXUGO,			\
				   data->debugfs_root,		\
				   data,			\
				   &_name ## _debugfs_fops);	\
	if (IS_ERR_OR_NULL(file)) {				\
		ret = PTR_ERR(file) ?: -ENOMEM;			\
		goto rm;					\
	}							\
	/*AVIR2R_LOG_INFO(data, "debugfs : %s",#_name)*/


static int __devinit avi_r2r_test_data_init(struct avi_r2r_test *data)
{
	struct dentry	*file;
	int		ret = 0;

	AVIR2R_LOG_INFO(data, "Starting...");

	init_waitqueue_head(&data->wait_done);
	atomic_set(&data->callback_done_ran,0);
	atomic_set(&data->callback_configure_ran,0);
	atomic_set(&data->all_done,0);
	spin_lock_init(&data->lock);

	hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	/* Map the DMA memory. */
	data->vmem = request_mem_region(data->platform_device->resource->start,
			resource_size(data->platform_device->resource),
			dev_name(data->dev));
	if (!data->vmem) {
		ret = -EBUSY;
		AVIR2R_LOG_ERROR(data,
			"request_mem_region %pR failed",
			data->platform_device->resource);
		goto mem_request_failed;
	}
	data->mem_size  = resource_size(data->platform_device->resource);

	data->mem_start = ioremap(data->platform_device->resource->start,
			resource_size(data->platform_device->resource));

	if (!data->mem_start) {
		ret = -ENOMEM;
		AVIR2R_LOG_ERROR(data,
			"ioremap %pR failed",
			data->platform_device->resource);
		goto ioremap_failed;
	}

	data->phys_mem_start = (void *)data->platform_device->resource->start;

	AVIR2R_LOG_INFO(data,
		"Workspace %pR",data->platform_device->resource);

	data->debugfs_root = debugfs_create_dir(dev_name(data->dev), NULL);
	if (IS_ERR_OR_NULL(data->debugfs_root)) {
		ret=data->debugfs_root ? PTR_ERR(data->debugfs_root) : -ENOMEM;
		goto rm;
	}
	MY_DEBUGFS_REGISTER(src_frame);
	MY_DEBUGFS_REGISTER(tgt_frame);
	MY_DEBUGFS_REGISTER(src_rect);
	MY_DEBUGFS_REGISTER(tgt_rect);
	MY_DEBUGFS_REGISTER(src_pos);
	MY_DEBUGFS_REGISTER(tgt_pos);
	MY_DEBUGFS_REGISTER(src_addr);
	MY_DEBUGFS_REGISTER(tgt_addr);
	MY_DEBUGFS_REGISTER(src_nbuffers);
	MY_DEBUGFS_REGISTER(tgt_nbuffers);
	MY_DEBUGFS_REGISTER(src_pixfmt);
	MY_DEBUGFS_REGISTER(tgt_pixfmt);
	MY_DEBUGFS_REGISTER(src_delta);
	MY_DEBUGFS_REGISTER(tgt_delta);
	MY_DEBUGFS_REGISTER(delta_count);
	MY_DEBUGFS_REGISTER(src_cspace);
	MY_DEBUGFS_REGISTER(tgt_cspace);
	MY_DEBUGFS_REGISTER(run);
	MY_DEBUGFS_REGISTER(running);
	MY_DEBUGFS_REGISTER(counters);
	MY_DEBUGFS_REGISTER(mismatchs);
	MY_DEBUGFS_REGISTER(verbose);
	MY_DEBUGFS_REGISTER(time_out);
	MY_DEBUGFS_REGISTER(startup_delay);
	MY_DEBUGFS_REGISTER(period);
	MY_DEBUGFS_REGISTER(config_run_time);
	MY_DEBUGFS_REGISTER(tag_src);
	MY_DEBUGFS_REGISTER(gam_bypass);
	MY_DEBUGFS_REGISTER(gam_palette);
	MY_DEBUGFS_REGISTER(mem_start);
	MY_DEBUGFS_REGISTER(mem_size);
	MY_DEBUGFS_REGISTER(timings);

	get_test_config(data);

	AVIR2R_LOG_INFO(data, "Ready");
	return 0;
rm:
ioremap_failed:
mem_request_failed:
	avi_r2r_test_data_delete(data);
	return ret;
}
static int __devinit avi_r2r_test_probe(struct platform_device *pdev)
{
	struct avi_r2r_test	*avi_r2r_test;
	int			ret;

	dev_info(&pdev->dev, "Probing...\n");
#define ERROR_OUT(lbl, msg, errno)						\
	do { ret = errno;							\
		dev_err(&pdev->dev, "Probing failed: %s [%d]\n", msg, ret);	\
		goto lbl; }							\
	while (0)

	avi_r2r_test = kzalloc(sizeof(*avi_r2r_test), GFP_KERNEL);
	if (!avi_r2r_test)
		ERROR_OUT(exit, "couldn't allocate avi_r2r_test data", -ENOMEM);

	avi_r2r_test->kt_period = kzalloc(sizeof(struct kt_period)*nr_timings_saved, GFP_KERNEL);
	if (!avi_r2r_test->kt_period)
		ERROR_OUT(clean, "couldn't allocate kt_period data", -ENOMEM);

	avi_r2r_test->dev = &pdev->dev;
	avi_r2r_test->platform_device = pdev;

	dev_set_drvdata(&pdev->dev, avi_r2r_test);

	if ((ret=avi_r2r_test_data_init(avi_r2r_test))!=0)
		ERROR_OUT(clean, "couldn't initialise avi_r2r_test data", ret);

	dev_info(&pdev->dev, "Probed\n");

	return 0;
clean:
	avi_r2r_test_data_delete(avi_r2r_test);
	if (avi_r2r_test->kt_period)
		kfree(avi_r2r_test->kt_period);
	kfree(avi_r2r_test);
exit:
	return ret;
}


static int __devexit avi_r2r_test_remove(struct platform_device *pdev)
{
	struct avi_r2r_test	*avi_r2r_test = dev_get_drvdata(&pdev->dev);

	dev_info(&pdev->dev, "Removing...\n");	\
	dev_set_drvdata(&pdev->dev, NULL);
	avi_r2r_test_data_delete(avi_r2r_test);
	kfree(avi_r2r_test);
	dev_info(&pdev->dev, "Removed\n");
	return 0;
}

static struct platform_driver avi_r2r_test_driver = {
	.driver         = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
	.probe          = avi_r2r_test_probe,
	.remove         = __devexit_p(avi_r2r_test_remove),
};
/**
 * init function
 */
static int __init avi_r2r_test_init( void )
{
	int ret;
	pr_info(DRIVER_NAME " Initializing...\n");
	ret =  platform_driver_register(&avi_r2r_test_driver);
	if (ret)
		pr_err(DRIVER_NAME " init fail (%d)\n", ret);
	else
		pr_info(DRIVER_NAME " Initialized\n");
	return ret;
}

/**
 * exit function
 */
static void __exit avi_r2r_test_exit( void )
{
	pr_info(DRIVER_NAME " Exiting...\n");
	platform_driver_unregister(&avi_r2r_test_driver);
	pr_info(DRIVER_NAME " Exited\n");
}

module_init( avi_r2r_test_init );
module_exit( avi_r2r_test_exit );

MODULE_AUTHOR("Didier Leymarie <didier.leymarie.ext@parrot.com>");
MODULE_DESCRIPTION("RAM to RAM driver for Parrot7 Advanced Video Interface (test module multiple clients");
MODULE_LICENSE( "GPL" );

