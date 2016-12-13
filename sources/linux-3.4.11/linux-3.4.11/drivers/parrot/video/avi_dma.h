#ifndef _AVI_DMA_H_
#define _AVI_DMA_H_

/*
 *  linux/drivers/parrot/video/avi_dma.h
 *
 *  Copyright (C) 2014 Parrot S.A.
 *
 * @author  Didier Leymarie <didier.leymarie.ext@parrot.com>
 * @date  15-Jan-2014
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

#include "avi_compat.h"

/* This struct has been packed to fit in 32bits. This way we can store it and
 * pass it by value everywhere instead of using an indirection through an
 * array. */
struct avi_dma_pixfmt {
	/* Unique format ID (the rest of the config might not be unique, for
	 * instance bayer 3x10 and rgb565 share the same configuration)*/
	unsigned id               : 6;
	/* Bit encoding (565, 888, 8888 etc...) */
	unsigned bitenc           : 4;
	/* Chroma subsampling: 4:4:4/4:2:2/4:2:0 */
	unsigned subsampling      : 2;
	/* Component reorganisation for 16 and 24bit formats. */
	unsigned reorg            : 2;
	/* set if the format is semiplanar */
	unsigned semiplanar       : 1;
	/* RAW bayer 'space' */
	unsigned raw              : 1;
	/* Width of the pixels in both planes (in bytes - 1) */
	unsigned bytes_per_pixel0 : 2;
	unsigned bytes_per_pixel1 : 2;
	/* swap value. The value of the last byte is implied as
	 * b0 ^ b1 ^ b2.
	 *
	 * If the format is planar, swap0 is plane0 and swap1 is
	 * plane1.
	 *
	 * Otherwise swap0 is the dma_in swap value and swap0
	 * the dma out. That should take care of all weird
	 * cases. */
	unsigned swap0_b0         : 2;
	unsigned swap0_b1         : 2;
	unsigned swap0_b2         : 2;
	unsigned swap1_b0         : 2;
	unsigned swap1_b1         : 2;
	unsigned swap1_b2         : 2;
};

/* Dummy function to validate the struct is correctly packed */
static inline void avi_dma_check_pixfmt_packing__dummy(void)
{
	BUILD_BUG_ON(sizeof(struct avi_dma_pixfmt) != sizeof(u32));
}

enum avi_dma_buffer_status
{
	AVI_BUFFER_INVAL = 0,
	AVI_BUFFER_READY,
	AVI_BUFFER_DONE,
	AVI_BUFFER_ERROR,
	AVI_BUFFER_TIMEOUT,
};

/* AVI Buffer struct */
struct avi_dma_buffer
{
	struct {
		dma_addr_t dma_addr;
	} plane0, plane1;

	enum avi_dma_buffer_status status;

	/* Private client data */
	void *priv;
};

/* Forward declare avi_segment_format to prevent circular includes */
struct avi_segment_format;

/* Setup FIFO registers for DMA input according to avi_segment_format
 * @fifo_plane0: avi_node for FIFO on Plane 0
 * @fifo_plane1: avi_node for FIFO on Plane 1, Planar configuration, optional
 * @in: input avi_segment_format
 * Note: FIFOs registers StartAddress dmasa0, dmasa1
 * 	and LineStride dmalstr0,dmalstr1 are not set by this function
 */
extern
void avi_dma_setup_input(const struct avi_node *fifo_plane0,
			 const struct avi_node *fifo_plane1,
			 const struct avi_segment_format *in);

/* Setup FIFO registers for DMA output according to avi_segment_format
 * @fifo: avi_node for FIFO
 * @out: output avi_segment_format
 * Note: FIFOs registers StartAddress dmasa0, dmasa1
 * 	and LineStride dmalstr0,dmalstr1 are not set by this function
 */
extern
void avi_dma_setup_output(const struct avi_node *fifo,
			  const struct avi_segment_format *out);

/* Setup FIFO registers for DMA input
 * 	according to avi_segment_format and avi_dma_buffer
 * @fifo_plane0: avi_node for FIFO on Plane 0
 * @fifo_plane1: avi_node for FIFO on Plane 1, Planar configuration, optional
 * @in: input avi_segment_format
 * @buff: input avi_dma_buffer
 * This function is the counterpart of avi_dma_setup_input()
 * 	which must be called before.
 * This function set FIFOs registers StartAddress dmasa0, dmasa1
 * 	and LineStride dmalstr0,dmalstr1 in compliance of AVI constraints
 */
extern
void avi_dma_set_input_buffer(const struct avi_node *fifo_plane0,
			      const struct avi_node *fifo_plane1,
			      const struct avi_segment_format *in,
			      const struct avi_dma_buffer *buff);

/* Setup FIFO registers for DMA output
 * 	according to avi_segment_format and avi_dma_buffer
 * @fifo: avi_node for FIFO
 * @out: output avi_segment_format
 * @buff: output avi_dma_buffer
 * This function is the counterpart of avi_dma_setup_output()
 * 	which must be called before.
 * This function set FIFOs registers StartAddress dmasa0, dmasa1
 * 	and LineStride dmalstr0,dmalstr1 in compliance of AVI constraints
 */
extern
void avi_dma_set_output_buffer(const struct avi_node *fifo,
			       const struct avi_segment_format *out,
			       const struct avi_dma_buffer *buff);

/* Setup line_size fields in avi_segment_format
 * 	according to avi_segment_format and AVI constraints
 * @fmt: avi_segment_format
 * @frame_width: width in pixels of whole frame
 * Following fmt fields must be set before calling:
 * 	pix_fmt: pixel format
 * This function set fmt fields:
 * 	plane0.line_size
 * 	plane1.line_size
 */
extern
void avi_dma_setup_line_size(struct avi_segment_format *fmt,
			     const unsigned frame_width,
			     const unsigned int requested_lnsize0);

/* Compute plane sizes for a given avi_segment_format
 * 	according AVI constraints
 * @fmt (input): avi_segment_format
 * @frame_height (input): height in rows of whole frame
 * @plane0_size (output): pointer to returned size of plane 0
 * @plane1_size (output): pointer to returned size of plane 1
 * Following fmt fields must be set before calling (use avi_dma_setup_line_size)
 * 	pix_fmt: pixel format
 * 	plane0.line_size
 * 	plane1.line_size
 */
extern
void avi_dma_setup_plane_size(struct avi_segment_format *fmt,
			     const unsigned frame_height,
			     unsigned *plane0_size,
			     unsigned *plane1_size);
#endif /* _AVI_DMA_H_ */
