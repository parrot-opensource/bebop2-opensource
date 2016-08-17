#ifndef _AVI_R2R_CORE_H_
#define _AVI_R2R_CORE_H_
/**
 * @file avi_r2r_core.h
 *  Parrot AVI RAM to RAM driver.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/atomic.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>

#include "avi_r2r.h"
#include "video/avi_pixfmt.h"
#include "video/avi_dma.h"
#include "video/avi_debug.h"

#define AVI_R2R_VERSION KERNEL_VERSION(0, 1, 0)

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

enum avi_r2r_channel_status {
	AVI_R2R_CHANNEL_UNUSED,
	AVI_R2R_CHANNEL_INACTIVE,
	AVI_R2R_CHANNEL_RUNNING,
	AVI_R2R_CHANNEL_STATUS_NR
};

/**
 *  Data structures definition
 */

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

struct avi_r2r_rotator_config {
	/* rotation angle */
	enum  avi_rotation	angle;
	/* horizontal mirror (flip) */
	bool			flip;
};

struct avi_r2r_registers {

	struct avi_r2r_gamma_correction_config	gam;
};

union avi_r2r_configuration_mask {
	struct {
		unsigned buffers:1;
		unsigned formats:1;
		unsigned gamma:1;
		unsigned converter:1;
		unsigned scaler:1;
		unsigned rotator:1;
		unsigned isp_bayer:1;
		unsigned isp_yuv:1;
	};
	u32 _mask;
};

struct avi_r2r;
struct avi_r2r_channel_data;
struct avi_r2r_user_context;

/**
 * Operation Queue item
 */
#define _AVI_R2R_WORK_MAGIC	0x52324432
struct avi_r2r_work {
	struct list_head		list;
	unsigned int			magic;
	struct avi_r2r_user_context	*user;
	struct avi_r2r_channel_data	*channel;
	struct timespec			ts_push;
	struct avi_dma_buffer		source_buffer;
	struct avi_dma_buffer		target_buffer;
	struct avi_dma_buffer		stats_buffer;
};

struct avi_r2r_work_queue {
	struct avi_r2r_work		works;
	int				works_nr;
};

static inline int avi_r2r__work_queue_level(struct avi_r2r_work_queue *wq)
{
	return wq->works_nr;
}

struct avi_r2r_user_format {
	struct avi_segment_format			source;
	struct avi_segment_format			target;
	struct avi_segment_format			compose;
	struct avi_segment_layout			layout;
	u32						background;
	bool						use_compose;
};
/**
 * User context definition
 */
#define _AVI_R2R_CTX_ID_MAGIC   0x52325255
#define _AVI_R2R_CTX_UUID_MASK  0x00FFFFFF
#define _AVI_R2R_CTX_UUID_MAGIC 0x55000000

struct avi_r2r_user_context {
	unsigned int					magic;
#ifdef CONFIG_AVI_DEBUG
	struct dentry 					*debugfs_root;
#endif /* CONFIG_AVI_DEBUG */

	u32						requirements;
	u32 						channels_compliant;
	int						channels_compliant_nr;

	struct avi_r2r					*avi_r2r;

	/* Configuration of elements (user) */
	struct avi_r2r_context				*context;
	struct avi_r2r_user_format			format;

	union avi_r2r_configuration_mask		requirements_mask;
	union avi_r2r_configuration_mask		config_done;

	struct avi_r2r_gamma_correction_config		gamma_user_config;
	struct avi_r2r_rotator_config			rotator_user_config;

	struct avi_r2r_registers			registers;
	unsigned int					magic_end;
};

static inline bool avi_r2r__user_context_is_ok(
		const struct avi_r2r_user_context *user_context)
{
	if (user_context)
		return (user_context->magic     == _AVI_R2R_CTX_ID_MAGIC &&
			user_context->magic_end == _AVI_R2R_CTX_ID_MAGIC );
	return 0;
}

/**
 * Channel definition
 */
struct avi_r2r_channel_nodes {
	/* AVI Nodes */
	int				nodes_nr;
	/* Source FIFO */
	const struct avi_node		*source_fifo_node;

	const struct avi_node		*source_fifo_planar_node;

	/* Target FIFO */
	const struct avi_node		*target_fifo_node;

	/* Stats FIFO */
	const struct avi_node		*stats_fifo_node;

	/* optional rotator for rotation */
	const struct avi_node		*rotator_node;

	/* optional gamma correction module */
	const struct avi_node		*gam_node;

	/* optional ISP bayer chain */
	const struct avi_node		*bayer_node;
};

#define _AVI_R2R_CHANNEL_MAGIC      0x52324330
#define _AVI_R2R_CHANNEL_MAGIC_MASK 0xFFFFFFF0

struct timings {
	struct timespec	ts_push;
	struct timespec	ts_start;
	struct timespec	ts_stop;
	unsigned int	uuid;
	int		index;
};

struct avi_r2r_channel_data {
	unsigned int			magic;
	struct avi_segment		*output_segment;
	struct avi_segment		*input_segment;
	struct avi_r2r			*avi_r2r;
	struct avi_r2r_user_context	*previous_user;
	struct avi_r2r_work 		current_work;
	spinlock_t			lock;

	atomic_t			status;
	/* enum avi_r2r_channel_status */

	struct timespec			ts_start;
	struct timespec			ts_stop;
	struct timespec			ts_push;
#ifdef CONFIG_AVI_DEBUG
	struct timings			*timings;
	int				timings_index,
					timings_nr,
					timings_counter,
					nr_timings_saved;
	struct dentry 			*debugfs_root;
#endif /* CONFIG_AVI_DEBUG */
	unsigned int			interrupts_counter;
	unsigned int			spurious_interrupts_counter;
	unsigned int			missing_source_interrupts_counter;
	unsigned int			missing_planar_interrupts_counter;
	unsigned int			input_segment_interrupts_counter;
	unsigned int			output_segment_interrupts_counter;
	unsigned int			apply_counter;

	struct avi_r2r_work_queue	work_queue;
	struct avi_r2r_work		*work_pool;
	unsigned int			work_pool_size;
	unsigned int			work_pool_free;

	struct avi_r2r_channel_nodes	avi_nodes;
	u32				available;

	/* Configuration of elements (cache) */
	struct avi_r2r_registers	registers;

	unsigned int			magic_end;

	/* Bayer stats output */
	struct avi_segment		*stats_segment;

};

/**
 *  AVI RAM2RAM data: global driver data
 */
struct avi_r2r {
	struct device				*dev;

#ifdef CONFIG_AVI_DEBUG
	/* The root of our debugfs filesystem. */
	struct dentry 				*debugfs_root;
#endif /* CONFIG_AVI_DEBUG */

	/* Channels */
	struct avi_r2r_channel_data		*channels[MAX_CHANNELS_AVI_R2R];
	int					channels_nr;

	int					suspended;
};

#ifdef CONFIG_AVI_DEBUG
/*
 * debugfs support
 */
extern void avi_r2r__user_setup_debugfs
					(struct avi_r2r_user_context *user);
extern void avi_r2r__user_destroy_debugfs
					(struct avi_r2r_user_context *user);

extern int __devinit avi_r2r__channel_setup_debugfs
					(struct avi_r2r_channel_data *channel);
extern void avi_r2r__channel_destroy_debugfs
					(struct avi_r2r_channel_data *channel);

extern void avi_r2r__channel_add_timings(struct avi_r2r_channel_data *data);

#endif /* CONFIG_AVI_DEBUG */

#endif /* _AVI_R2R_CORE_H_ */
