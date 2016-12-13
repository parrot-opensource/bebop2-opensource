#include <linux/module.h>
#include "avi_dma.h"
#include "avi_segment.h"
#include "avi_pixfmt.h"

/**
 * Tales of AVI: DMA
 *
 * Definition:
 * - whole frame: a complete frame
 * - stripped frame: a part of a whole frame
 * - left address:
 *      physical address of first byte of left pixel of stripped frame
 * - right address:
 *      physical address of last byte of right pixel of stripped frame
 *
 * Whole frames in memory have to be compliant to AVI constraints:
 * - start addresses must be 64 bits aligned
 * - line sizes must be 64 bits aligned
 * - line sizes must be a multiple of pixel width
 * - for specific pixel formats, width and/or height must be even
 *
 * Stripped frames have also to be compliant to AVI constraints:
 * - for specific pixel formats, width and/or height must be even
 * - for P7R1/P7R2, left and right addresses must be pixel aligned and 64 bits aligned
 * - for P7R3, left and right addresses must be pixel aligned.
 */

/* Swap calculations:
 *
 * The swap value is made of 8 values, each the index of a byte to pick.
 *
 * The top 4 bytes are always the same as the top bottom bytes + 4 (we don't
 * have any format where we apply a different swapping for the top and bottom
 * 32bits.
 *
 * A same byte is never repeated (picked twice).
 *
 * Therefore, we can represent the swapping by only 3 values: the first three
 * values for the bottom 32bits.
 *
 * The 4th value is found by elimination (if the values are 0, 1, 3 then the 4th
 * can only be 2).
 */

/* compute register value for swap0 */
static inline u32 avi_dma_pixfmt_swap0(struct avi_dma_pixfmt pixfmt)
{
	unsigned b0 = pixfmt.swap0_b0;
	unsigned b1 = pixfmt.swap0_b1;
	unsigned b2 = pixfmt.swap0_b2;
	unsigned b3 = b0 ^ b1 ^ b2;

	return AVI_FIFO_MAKE_SWAP(b0, b1, b2, b3);
}

/* compute register value for swap1 */
static inline u32 avi_dma_pixfmt_swap1(struct avi_dma_pixfmt pixfmt)
{
	unsigned b0 = pixfmt.swap1_b0;
	unsigned b1 = pixfmt.swap1_b1;
	unsigned b2 = pixfmt.swap1_b2;
	unsigned b3 = b0 ^ b1 ^ b2;

	return AVI_FIFO_MAKE_SWAP(b0, b1, b2, b3);
}

/**
 * Returns the maximal value of pincr for a given FIFO and AVI revision.
 *
 * In P7R3, maximal value for pincr is 255.
 * In P7R2, maximal value for pincr is
 * 64 for FIFO00 & FIFO01, 128 for other FIFOs
 * In P7R1, maximal value for pincr is 64.
 */
static inline unsigned avi_fifo_max_pincr(const struct avi_node *fifo)
{
	switch (avi_get_revision())
	{
		default:
		case AVI_REVISION_3:
			return 255;
		break;
		case AVI_REVISION_1:
		case AVI_REVISION_2:
			switch(fifo->node_id)
			{
				case AVI_FIFO00_NODE:
				case AVI_FIFO01_NODE:
					return 64;
					break;
				default:
					return 128;
					break;
			}
		break;
	}

}

/**
 * Returns the value of pincr for a given bandwidth (in bytes/s).
 *
 * The highest the value of pincr the lowest the priority.
 * The maximal priority is 1 (0 is invalid), the minimal 255.
 *
 * I attribute maximal value to 0B/s (minimal bandwidth) and 1 to 474MB/s
 * (bandwidth of 1080p60 @ 32bits). I use a linear scale between those two
 * values.
 */
static inline unsigned avi_fifo_bw_to_pincr(unsigned long bw, unsigned max_pincr)
{
	/* max is bw of 1080p60 32bpp */
	static const unsigned long bwmax = 1920 * 1080 * 4 * 60 / 1024;
	unsigned max_pincr_minus_one, ret;

        /* convert bandwidth to KB to make sure the calculations will
           fit in 32 bits */
        bw /= 1024;

	if (bw >= bwmax)
		return 1;

	if (max_pincr > 255)
		max_pincr = 255;
	if (max_pincr < 255)
		max_pincr_minus_one = max_pincr -1;
	else
		max_pincr_minus_one = max_pincr;

	/* extrapolate the value of pincr linearly */
	ret = max_pincr - (max_pincr_minus_one * bw / bwmax);
	if (ret < 1)
		ret = 1;

	return ret;
}

void avi_dma_setup_input(const struct avi_node *fifo_plane0,
			 const struct avi_node *fifo_plane1,
			 const struct avi_segment_format *in)
{
	struct avi_fifo_registers	 fifo_reg_0, fifo_reg_1;
	struct avi_dma_pixfmt const	 pixfmt = in->pix_fmt;

	/* Clear overall configuration. */
	memset(&fifo_reg_0, 0, sizeof(fifo_reg_0));
	memset(&fifo_reg_1, 0, sizeof(fifo_reg_1));

	/* Setup interrupts. */
	fifo_reg_0.cfg.itline = 0x0F;
	fifo_reg_0.cfg.itsource = 1;

	/* Input DMA from memory. */
	fifo_reg_0.cfg.srctype = AVI_FIFO_CFG_DMA_TYPE;

	/* Input data depicted by format. */
	fifo_reg_0.cfg.srcorg    = pixfmt.semiplanar;
	fifo_reg_0.cfg.srcformat = pixfmt.subsampling;
	fifo_reg_0.cfg.srcbitenc = pixfmt.bitenc;

	/*
	 * Destination is always a video link in packed 4:4:4
	 * 32bits format.
	 */
	fifo_reg_0.cfg.dsttype   = AVI_FIFO_CFG_VL_TYPE;
	fifo_reg_0.cfg.dstorg    = AVI_FIFO_CFG_PACKED_ORG;
	fifo_reg_0.cfg.dstformat = AVI_FIFO_CFG_444_FMT,
	fifo_reg_0.cfg.dstbitenc = AVI_FIFO_CFG_8888_BENC;

	fifo_reg_0.cfg.dithering = AVI_FIFO_CFG_TRUNC_RND;

	fifo_reg_0.cfg.cusdis = AVI_FIFO_CFG_DISABLE_CUS;
	fifo_reg_0.cfg.cdsdis = AVI_FIFO_CFG_DISABLE_CDS;

	fifo_reg_0.cfg.reorg       = pixfmt.reorg;

	fifo_reg_0.swap0._register = avi_dma_pixfmt_swap0(pixfmt);

	if (pixfmt.semiplanar)
		fifo_reg_0.swap1._register = avi_dma_pixfmt_swap1(pixfmt);
	else
		fifo_reg_0.swap1._register = 0;

	/* Use some know-to-work values for the timeout settings */
	fifo_reg_0.timeout.vblank = 0;
	fifo_reg_0.timeout.hblank = 0;
	/* TODO: We should probably set the pincr as well, but for that we need
	 * the framerate */
	fifo_reg_0.timeout.pincr  = 1;

	fifo_reg_0.timeout.issuing_capability = 7;
	fifo_reg_0.timeout.burst = 0;


	/* We send the whole frame at once for now */
	fifo_reg_0.dmafxycfg.dmafxnb	= 1;
	fifo_reg_0.dmafxycfg.dmafynb	= 1;
	fifo_reg_0.dmafxycfg.dmafxymode = 0;
	fifo_reg_0.dmafxstr0		= 0;
	fifo_reg_0.dmafxstr1		= 0;
	fifo_reg_0.dmafystr0		= 0;
	fifo_reg_0.dmafystr1		= 0;

	if (in->interlaced)
		fifo_reg_0.framesize.height = in->height / 2 - 1;
	else
		fifo_reg_0.framesize.height = in->height - 1;

	if (unlikely(in->pix_fmt.id == AVI_BAYER_3X10_32_FMTID))
		/* In this mode we put 3 10-bit pixels in one 32bit word.
		 * TODO: this calculation is bogus, it does not work for
		 * cropping.
		 */
		fifo_reg_0.framesize.width = roundup(in->width, 3) - 1;
	else
		fifo_reg_0.framesize.width = in->width  - 1;

	/* TODO : interlaced management */

	if (avi_pixfmt_is_planar(in->pix_fmt) && fifo_plane1) {
		/* use 2 FIFOs and a Scaler to manage SEMI-PLANAR formats
		 * both FIFOs are set to AVI_FIFO_CFG_422_FMT (src/dst)
		 * never less of desired format (420 or 422)
		 * FIFO plane0 is set to AVI_FIFO_CFG_Y_SCALPLANE
		 * FIFO plane1 is set to AVI_FIFO_CFG_UV_SCALPLANE
		 * Frame height is divided by 2 for FIFO plane1 in case of 420.
		 * This configuration allows:
		 * - management of SEMI-PLANAR formats in P7R1/R2
		 *   where these formats are bugged: workaround
		 * - a better quality for chrominance (UV) scaling
		 */
		fifo_reg_1  = fifo_reg_0;

		fifo_reg_0.cfg.dstformat = AVI_FIFO_CFG_422_FMT;
		/* If dstorg is planar the FIFOs will only fetch the plane they
		 * have to. FIFO0 will fetch the plane0 (DMASA0) and FIFO1 will
		 * fetch plane1 (DMASA1) */
		fifo_reg_0.cfg.dstorg    = AVI_FIFO_CFG_PLANAR_ORG;
		fifo_reg_0.cfg.srcformat = AVI_FIFO_CFG_422_FMT;
		fifo_reg_0.cfg.srcorg    = AVI_FIFO_CFG_PLANAR_ORG;
		fifo_reg_0.cfg.scalplane = AVI_FIFO_CFG_Y_SCALPLANE;
		fifo_reg_0.cfg.cusdis = AVI_FIFO_CFG_ENABLE_CUS;
		fifo_reg_0.cfg.cdsdis = AVI_FIFO_CFG_ENABLE_CDS;

		fifo_reg_1.cfg.dstformat = AVI_FIFO_CFG_422_FMT;
		fifo_reg_1.cfg.dstorg    = AVI_FIFO_CFG_PLANAR_ORG;
		fifo_reg_1.cfg.srcformat = AVI_FIFO_CFG_422_FMT;
		fifo_reg_1.cfg.srcorg    = AVI_FIFO_CFG_PLANAR_ORG;
		fifo_reg_1.cfg.scalplane = AVI_FIFO_CFG_UV_SCALPLANE;
		fifo_reg_1.cfg.cusdis = AVI_FIFO_CFG_ENABLE_CUS;
		fifo_reg_1.cfg.cdsdis = AVI_FIFO_CFG_ENABLE_CDS;

		if (avi_pixfmt_get_packing(in->pix_fmt) ==
		    AVI_SEMIPLANAR_YUV_420_PACKING) {
			fifo_reg_1.framesize.width  = in->width  - 1;
			fifo_reg_1.framesize.height = (in->height / 2) - 1;

		}

		avi_fifo_set_registers(fifo_plane0, &fifo_reg_0);
		avi_fifo_set_registers(fifo_plane1, &fifo_reg_1);
	}
	else
		avi_fifo_set_registers(fifo_plane0, &fifo_reg_0);
}
EXPORT_SYMBOL(avi_dma_setup_input);

void avi_dma_setup_output(const struct avi_node *fifo,
			  const struct avi_segment_format *out)
{
	struct avi_fifo_registers	fifo_reg;
	struct avi_dma_pixfmt           pixfmt = out->pix_fmt;

	/* Clear overall configuration. */
	memset(&fifo_reg, 0, sizeof(struct avi_fifo_registers));

	/* Setup interrupts. */
	fifo_reg.cfg.itline   = 0x0F;
	fifo_reg.cfg.itsource = 1;

	/* Output DMA to memory. */
	fifo_reg.cfg.dsttype = AVI_FIFO_CFG_DMA_TYPE;

	/* Output data depicted by format. */
	fifo_reg.cfg.dstorg    = pixfmt.semiplanar;
	fifo_reg.cfg.dstformat = pixfmt.subsampling;
	fifo_reg.cfg.dstbitenc = pixfmt.bitenc;

	/*
	 * Source is always a video link in packed 4:4:4
	 * 32bits format.
	 */
	fifo_reg.cfg.srctype   = AVI_FIFO_CFG_VL_TYPE;
	fifo_reg.cfg.srcorg    = AVI_FIFO_CFG_PACKED_ORG;
	fifo_reg.cfg.srcformat = AVI_FIFO_CFG_444_FMT,
	fifo_reg.cfg.srcbitenc = AVI_FIFO_CFG_8888_BENC;

	fifo_reg.cfg.dithering = AVI_FIFO_CFG_TRUNC_RND;

	fifo_reg.cfg.cusdis = AVI_FIFO_CFG_DISABLE_CUS;
	fifo_reg.cfg.cdsdis = AVI_FIFO_CFG_DISABLE_CDS;

	fifo_reg.cfg.reorg       = pixfmt.reorg;

	if (pixfmt.semiplanar) {
		/* if the format is semiplanar, swap0 and swap1 in pixfmt
		 * contain the format for both planes */
		fifo_reg.swap0._register = avi_dma_pixfmt_swap0(pixfmt);
		fifo_reg.swap1._register = avi_dma_pixfmt_swap1(pixfmt);
	} else {
		/* otherwise swap0 and swap1 in pixfmt contain the dma in and
		 * dma out swap values */
		fifo_reg.swap0._register = avi_dma_pixfmt_swap1(pixfmt);
		fifo_reg.swap1._register = 0;
	}

	/* Use some know-to-work values for the timeout settings */
	fifo_reg.timeout.vblank = 0;
	fifo_reg.timeout.hblank = 0;
	/* TODO: We should probably set the pincr as well, but for that we need
	 * the framerate */
	fifo_reg.timeout.pincr  = 1;

	fifo_reg.timeout.issuing_capability = 0;
	fifo_reg.timeout.burst = 0;

	/* We send the whole frame at once for now */
	fifo_reg.dmafxycfg.dmafxnb = 1;
	fifo_reg.dmafxycfg.dmafynb = 1;
	fifo_reg.dmafxycfg.dmafxymode = 0;
	fifo_reg.dmafxstr0 = 0;
	fifo_reg.dmafxstr1 = 0;
	fifo_reg.dmafystr0 = 0;
	fifo_reg.dmafystr1 = 0;

	if (out->interlaced)
		fifo_reg.framesize.height = out->height / 2 - 1;
	else
		fifo_reg.framesize.height = out->height - 1;

	if (unlikely(out->pix_fmt.id == AVI_BAYER_3X10_32_FMTID))
		/* In this mode we put 3 10-bit pixels in one 32bit word.
		 * TODO: this calculation is bogus, it does not work for
		 * cropping.
		 */
		fifo_reg.framesize.width = roundup(out->width, 3) - 1;
	else
		fifo_reg.framesize.width = out->width  - 1;

	/* TODO : interlaced management */

	avi_fifo_set_registers(fifo, &fifo_reg);
}
EXPORT_SYMBOL(avi_dma_setup_output);

void avi_dma_setup_inner_buffer(const struct avi_node *fifo,
                          const struct avi_segment_format *out)
{
	struct avi_fifo_registers       fifo_reg;

	/* Clear overall configuration. */
	memset(&fifo_reg, 0, sizeof(struct avi_fifo_registers));

	fifo_reg.cfg.dstbitenc = AVI_FIFO_CFG_8888_BENC;
	fifo_reg.cfg.srcbitenc = AVI_FIFO_CFG_8888_BENC;

	fifo_reg.framesize.height = out->height - 1;
	fifo_reg.framesize.width = out->width  - 1;

	avi_fifo_set_registers(fifo, &fifo_reg);
}
EXPORT_SYMBOL(avi_dma_setup_inner_buffer);

/**
 * Computation of line stride
 *
 * Line stride is depending on several parameters:
 * - whole frame line size (see avi_segment.planeX.line_size) in bytes
 * - stripped frame width size: width * pixel_size
 * - progressive or interlaced mode
 * - left address
 * - right address
 *
 * For all revision of P7, first part of computation is the same:
 * line_stride = line_size - width_size
 *
 * Second part of computation only for P7R3.
 * P7R3 is able to transfer data even if left and right addresses are not
 * 64 bits aligned, but its internal address pointers work only
 * on 64 bits addresses. So, line stride must be tuned to take account
 * of these "overflows".
 * During transfer, for each line, effective addresses start before
 * the left address and end after the right address.
 * The left padding is the modulo 8 of left address, (8 bytes -> 64 bits)
 * the right padding is the complement of module 8 of right address.
 * Module 8 of an address can have the values:
 * - 0, 4 for 32 bits pixel format
 * - 0, 2, 4, 6 for 16 bits pixel format
 * - 0-7 for 8 bits pixel format
 * For 24 bits pixel format, this is a special case: modulo 8 is replaced
 * by a "modulo 8-24": effective addresses are 64 bits aligned (8 bytes)
 * and 24 bits aligned (3 bytes): common multiple 24.
 * To compute "modulo 8-24", first modulo 8 is computed as usual (result in 0-7)
 * second it is converted to module 24.
 * This "modulo 8-24" is used in replacement of modulo 8
 * for left and right paddings
 */

static const int avi_dma_24bit_align[] =
{
	[0] = 0,
	[1] = 9,
	[2] = 18,
	[3] = 3,
	[4] = 12,
	[5] = 21,
	[6] = 6,
	[7] = 15
};

static inline int avi_dma_alignment(unsigned bytes_per_pixel)
{
	/* The alignment constraint is 8 bytes for all modes except the 24bit
	 * ones. There we have to have a multiple of 24bytes to reach the
	 * correct 64bit alignment over a full pixel. */
	return (bytes_per_pixel == 3) ? 24 : 8;
}

static void avi_dma_setup_memory_addresses(const struct avi_node *fifo,
					   const struct avi_segment_format *fmt,
					   const struct avi_dma_buffer *buffer)
{
	struct avi_dma_pixfmt   pixfmt;
	int			width_bytes;
	int			pad_left  = 0;
	int			pad_right = 0;
	int			tmp;
	int			align;
	int			left;
	int			right;
	unsigned		line_size0;
	unsigned		line_size1;
	u32			dmasa0;
	u32			dmasa1;
	u32			dmalstr0;
	u32			dmalstr1;

	pixfmt = fmt->pix_fmt;

	line_size0 = fmt->plane0.line_size;
	line_size1 = fmt->plane1.line_size;

	dmasa0 = buffer->plane0.dma_addr;
	dmasa1 = buffer->plane1.dma_addr;

	width_bytes = fmt->width * avi_pixel_size0(pixfmt);

	dmalstr0 = line_size0 - width_bytes;

#ifdef AVI_BACKWARD_COMPAT
	if (avi_get_revision() >= AVI_REVISION_3) {
#endif /* AVI_BACKWARD_COMPAT */

		/* The alignment constraint is 8 bytes for all modes except the
		 * 24bit ones. There we have to have a multiple of 24bytes to
		 * reach the correct 64bit alignment over a full pixel. */
		align = avi_dma_alignment(avi_pixel_size0(pixfmt));

		pad_left = 0;
		left = dmasa0 % 8;
		if (left != 0) {
			if (align == 24)
				left = avi_dma_24bit_align[left];
			pad_left = left;
		}

		pad_right = 0;
		right = left + width_bytes;
		tmp = right % 8;
		if (tmp != 0) {
			if (align == 24)
				tmp = avi_dma_24bit_align[tmp];
			pad_right = align - tmp;
		}
		dmalstr0 -= pad_left;
		dmalstr0 -= pad_right;
#ifdef AVI_BACKWARD_COMPAT
	}
#endif /* AVI_BACKWARD_COMPAT */

	dmalstr1 = 0;
	if (avi_pixfmt_is_planar(fmt->pix_fmt)) {
		width_bytes = fmt->width * avi_pixel_size1(pixfmt);
		dmalstr1 = line_size1 - width_bytes;
#ifdef AVI_BACKWARD_COMPAT
		if (avi_get_revision() >= AVI_REVISION_3) {
#endif /* AVI_BACKWARD_COMPAT */
			align = avi_dma_alignment(avi_pixel_size1(pixfmt));
			pad_left = 0;
			left = dmasa1 % 8;
			if (left != 0)
				pad_left = left;

			pad_right = 0;
			right = left + width_bytes;
			tmp = right % 8;
			if (tmp != 0) {
				pad_right = align - tmp;
			}
			dmalstr1 -= pad_left;
			dmalstr1 -= pad_right;
#ifdef AVI_BACKWARD_COMPAT
		}
#endif /* AVI_BACKWARD_COMPAT */
	}

	avi_fifo_set_dmasa(fifo, dmasa0, dmasa1);
	avi_fifo_set_dmalstr(fifo, dmalstr0, dmalstr1);
}

void avi_dma_set_input_buffer(const struct avi_node		*fifo_plane0,
			      const struct avi_node		*fifo_plane1,
			      const struct avi_segment_format	*in,
			      const struct avi_dma_buffer	*buff)
{
	avi_dma_setup_memory_addresses(fifo_plane0, in, buff);

	if (avi_pixfmt_is_planar(in->pix_fmt) && fifo_plane1)
		avi_dma_setup_memory_addresses(fifo_plane1, in, buff);
}
EXPORT_SYMBOL(avi_dma_set_input_buffer);

void avi_dma_set_output_buffer(const struct avi_node		*fifo,
			       const struct avi_segment_format	*out,
			       const struct avi_dma_buffer	*buff)
{
	avi_dma_setup_memory_addresses(fifo, out, buff);
}
EXPORT_SYMBOL(avi_dma_set_output_buffer);

void avi_dma_setup_line_size(struct avi_segment_format	*fmt,
			     const unsigned		 frame_width,
			     const unsigned int		 requested_lnsize0)
{
	unsigned	pixsz0, pixsz1, align, rem;

	BUG_ON(!fmt);
	BUG_ON(fmt->pix_fmt.id == AVI_INVALID_FMTID);
	BUG_ON(!frame_width);

	fmt->plane0.line_size = 0;
	fmt->plane1.line_size = 0;

	pixsz0 = avi_pixel_size0(fmt->pix_fmt);
	pixsz1 = avi_pixel_size1(fmt->pix_fmt);

	fmt->plane0.line_size = max(frame_width * pixsz0, requested_lnsize0);

	align = avi_dma_alignment(pixsz0);

	rem = fmt->plane0.line_size % 8;
	if (rem != 0) {
		if (align == 24 )
			rem = avi_dma_24bit_align[rem];

		fmt->plane0.line_size += align - rem;
	}

	/* Plane 1 */
	if (pixsz1 > 0) {
		unsigned int requested_lnsize1;

		/* derive requested_lnsize1 from requested_lnsize0 using
		 * format subsampling */
		requested_lnsize1 = (requested_lnsize0 * pixsz1) / pixsz0;
		fmt->plane1.line_size = max(frame_width * pixsz1, requested_lnsize1);
		align = avi_dma_alignment(pixsz1);

		rem = fmt->plane1.line_size % align;

		if (rem != 0) {
			if (align == 24 )
				rem = avi_dma_24bit_align[rem];

			fmt->plane1.line_size += align - rem;
		}
	}
}
EXPORT_SYMBOL(avi_dma_setup_line_size);

void avi_dma_setup_plane_size(struct avi_segment_format *fmt,
			     const unsigned		 frame_height,
			     unsigned			*plane0_size,
			     unsigned			*plane1_size)
{
	BUG_ON(!fmt);

	*plane0_size = frame_height * fmt->plane0.line_size;

	switch (avi_pixfmt_get_packing(fmt->pix_fmt)) {
		case AVI_INTERLEAVED_444_PACKING:
		case AVI_INTERLEAVED_YUV_422_PACKING:
			*plane1_size = 0;
		break;
		case AVI_SEMIPLANAR_YUV_422_PACKING:
			*plane1_size = frame_height * fmt->plane1.line_size;
		break;
		case AVI_SEMIPLANAR_YUV_420_PACKING:
			*plane1_size = frame_height * fmt->plane1.line_size / 2;
		break;
		default:
			BUG();
	}
}
EXPORT_SYMBOL(avi_dma_setup_plane_size);

MODULE_AUTHOR("Didier Leymarie <didier.leymarie.ext@parrot.com>");
MODULE_DESCRIPTION("Parrot Advanced Video Interface DMA layer");
MODULE_LICENSE("GPL");
