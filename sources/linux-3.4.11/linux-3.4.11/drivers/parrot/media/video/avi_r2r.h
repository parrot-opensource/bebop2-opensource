#ifndef _AVI_R2R_H_
#define _AVI_R2R_H_
/**
 * @file avi_r2r.h
 *  Parrot AVI RAM to RAM driver.
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author     didier.leymarie.ext@parrot.com
 * @date       2013-10-23
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

#include "video/avi_segment.h"

#define MAX_CHANNELS_AVI_R2R 6
#define AVI_R2R_DRIVER_NAME "avi_r2r"

struct avi_r2r_platform_data {
	struct {
		unsigned long			caps;
	} channels[MAX_CHANNELS_AVI_R2R];
};

/**
 *  Inter Driver communication section
 */

/**
 * Maximum number of elements per channel
 * 2x2 FIFOS, 1 SCALER, 1 ROTATOR, 4 CONV, 2 GAMMA, 1 ISP_BAYER, 1 ISP_YUV
 */
#define AVI_R2R_MAX_ELEMENTS_PER_CHANNEL 16

struct avi_r2r_context;

/* Structure defining operations (callbacks) */
struct avi_r2r_ops
{
	/* Callback to notify the client after processing frame */
	void (*done)(struct avi_r2r_context *, struct avi_dma_buffer *, struct avi_dma_buffer *, struct avi_dma_buffer *, void *);
	/* Callback to notify the client after processing frame and that
	 * internal spinlock has been released
	 */
	void (*finish)(struct avi_r2r_context *, void *);
	/* Callback to setup the client before processing frame */
	void (*configure)(struct avi_r2r_context *, struct avi_dma_buffer *, struct avi_dma_buffer *, void *);
};

/* Structure containing the ram2ram context */
struct avi_r2r_context
{
	/* operations */
	struct avi_r2r_ops	*ops;
	/* device */
	struct device		*dev;

	/* capabilities required */
	u32			caps;

	/* number of stacked jobs not processed yet */
	atomic_t 		stacked_nr;

	/* use has job(s) in progress (bool) */
	atomic_t		running;

	/* private part managed by driver (driver privacy) */
	void			*avi_r2r_private;

	/* private data for user,
	 * driver uses it as 4th parameter for callbacks
	 */
	void 			*private;
};


/**
 * Exported functions
 */

/** check if ram2ram driver is available and if requirements are supported.
 *
 * @requirements is a bitmask of the capabilities requested.
 * Returns 0 on success, negative errno on error.
 * 	-ENODEV if driver is not available
 * 	-ENOENT if no suitable channel was found
 *
 *
 */
extern
int avi_r2r_is_available(u32 requirements);

/** Initiate the session with the ram2ram driver.
 *
 * @ctx is the context
 * @dev is the device requesting the channel
 * @requirements is a bitmask of the capabilities requested.
 * @ops are operations
 *
 * /!\ Prior to this call
 *
 * Returns 0 on success, negative errno on error.
 * 	-ENODEV if driver is not available
 * 	-ENOENT if no suitable channel was found
 * 	-ENOMEM if new user could not be allocated
 *
 * By default the channel is initialized to "sane" default values: the
 * converter/gamma are bypassed, no scaling or rotation etc...
 *
 * It's then up to the client driver to initialize the elements it needs.
 *
 * The "DMA_IN" and "DMA_OUT" capabilities don't have to be specified since
 * they're mandatory in ram2ram operation and are therefore always implied.
 *
 * /!| the same client (same "dev") is allowed to make several calls to this
 * function to request several ram2ram contexts (one for scaling, one for
 * rotation etc...).
 *
 */
extern
int avi_r2r_request(struct avi_r2r_context *ctx,
		    struct device *dev,
		    u32 requirements,
		    struct avi_r2r_ops *ops);

/** Destroys a context initialized by a successful avi_r2r_request
 *
 * @ctx is the context
 *
 * Returns 0 on success, negative errno on error.
 * 	-ENODEV if driver is not available
 * 	-ENOENT if user context is not found
 * 	-EBUSY if some jobs have not been processed yet
 */
extern
int avi_r2r_destroy(struct avi_r2r_context *ctx);

/** return status of user's jobs.
 *
 * /!\ This function can be called at any moment on a valid context.
 *
 * @ctx is the context
 * @stacked_nr will provide number of jobs on stack
 * @running will provide a booleen if a job is in progress
 * Returns 0 on success, negative errno on error.
 * 	-ENODEV if driver is not available
 *
 */
extern
int avi_r2r_get_status(struct avi_r2r_context *ctx,
		       int *stacked_nr,
		       int *running);

/* The various setup functions used to configure the ram2ram operation
 *
 * /!\ These functions can be called at any moment on a valid context.
 *
 * @ctx is the context
 *
 * Returns 0 on success, negative errno on error.
 * 	-ENODEV if driver is not available
 * 	-ENOENT if user context is not found
 * 	-EINVAL if parameters can not be processed with capabilities requested
 * 	-EBUSY  if requests have not been processed yet
 *
 * The operation is defined by the following parameters; input format, output
 * format, internal transformations.
 *
 * The internal transformations supported are those requested by the driver in
 * the "cap" parameter of avi_r2r_request. For now those are: gamma correction
 * and colorspace conversion.
 */

/**
 * Gamma conversion
 */
extern
int avi_r2r_setup_gam(struct avi_r2r_context *ctx,
			bool bypass,
			bool palette,
			bool comp_width,
			struct avi_cmap *cmap);

/**
 * Rotator
 */
extern
int avi_r2r_setup_rotator(struct avi_r2r_context *ctx,
			  enum avi_rotation angle, bool flip);

/** Input and output format configuration.
 *
 * /!\ This function can be called at any moment on a valid context.
 */
extern
int avi_r2r_set_format(struct avi_r2r_context *ctx,
		       struct avi_segment_format *infmt,
		       struct avi_segment_format *outfmt,
		       struct avi_segment_format *composefmt,
		       struct avi_segment_layout *layout);


/** Enqueue buffers for processing.
 * *
 * @ctx is the context
 * @in_dma is input buffer
 * @out_dma is output buffer
 *
 * Returns
 * 0 on success,
 * 1 if user already streaming
 * negative errno on error.
 * 	-ENODEV if driver is not available
 * 	-ENOENT if user context is not found
 * 	-EINVAL if parameters can not be processed with capabilities requested
 *
 */

extern
int avi_r2r_buf_queue(struct avi_r2r_context *ctx,
			struct avi_dma_buffer *in_dma,
			struct avi_dma_buffer *out_dma,
			struct avi_dma_buffer *stats_dma);

extern
struct avi_segment * avi_r2r_get_segment(struct avi_r2r_context *ctx,
					 int in_out,
					 int *index);

static inline struct avi_segment * avi_r2r_get_dma_in_segment(
		struct avi_r2r_context *ctx,
		int *index)
{
	return avi_r2r_get_segment(ctx, 0, index);
}

static inline struct avi_segment * avi_r2r_get_dma_out_segment(
		struct avi_r2r_context *ctx,
		int *index)
{
	return avi_r2r_get_segment(ctx, 1, index);
}
#endif /* _AVI_R2R_H_ */
