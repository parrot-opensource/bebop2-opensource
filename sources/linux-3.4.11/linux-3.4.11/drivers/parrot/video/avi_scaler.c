#include <linux/module.h>
#include "avi_scaler.h"

/**************************
 * Scalers low-level setup
 **************************/

struct avi_scal_dimcfg {
	unsigned	ntaps;
	unsigned	nphases;
	char		coeffs[32];
};

/* Configuration for scaling factor 1 / 1 using lanczos
 * with 2 lobes.
 *
 * This configuration uses 4 taps and 8 phases.
 *
 *                +
 *               +|+
 *                | +
 *              + |
 *                |
 *             +  |  +
 *                |
 *            +   |   +
 *                |
 *                |
 *           +    |    +
 *                |
 *                |
 *          +     |     +
 *                |
 *         +      |      +
 * -------+---------------+------++
 * ++++  +        |        +  +++
 *     ++         |         ++
 */
static const struct avi_scal_dimcfg avi_scalcfg_1_1 = {
	.ntaps   = AVI_SCAL_NTAPS_4,
	.nphases = AVI_SCAL_NPHASES_8,
	.coeffs = {
		  0,  -4,  -5,  -5,  -4,  -2,  -1,  -1, /* tap 0 */
		 64,  62,  55,  46,  36,  24,  14,   6, /* tap 1 */
		  0,   6,  15,  25,  36,  47,  56,  62, /* tap 2 */
		  0,   0,  -1,  -2,  -4,  -5,  -5,  -3, /* tap 3 */
	},
};

/* Configuration for scaling factor 1 / 2 using lanczos
 * with 2 lobes.
 *
 * This configuration uses 8 taps and 4 phases.
 *
 *               +++
 *              + | +
 *             +  |  +
 *            +   |   +
 *                |
 *           +    |    +
 *          +     |     +
 *         +      |      +
 * ++++-+++---------------+++++++++
 *     +          |
 */
static const struct avi_scal_dimcfg avi_scalcfg_1_2 = {
	.ntaps   = AVI_SCAL_NTAPS_8,
	.nphases = AVI_SCAL_NPHASES_4,
	.coeffs = {
		 -2,  -1,  -1,  -1, /* tap 0 */
		  0,  -2,  -2,  -3, /* tap 1 */
		 18,  13,   7,   3, /* tap 2 */
		 32,  31,  28,  23, /* tap 3 */
		 18,  23,  28,  31, /* tap 4 */
		  0,   3,   7,  13, /* tap 5 */
		 -1,  -2,  -2,  -1, /* tap 6 */
		 -1,  -1,  -1,  -1, /* tap 7 */
	},
};

/* Configuration for scaling factor 1 / 3 using lanczos
 * with 2 lobes.
 *
 * This configuration uses 16 taps and 2 phases.
 *
 *               +++
 *              + | +
 *             +  |  +
 *            +   |   +
 *           +    |    +
 * ++++++++++-----------+++++++++++
 */
static const struct avi_scal_dimcfg avi_scalcfg_1_3 = {
	.ntaps   = AVI_SCAL_NTAPS_16,
	.nphases = AVI_SCAL_NPHASES_2,
	.coeffs = {
		 -1,  -1, /* tap 0 */
		 -1,  -1, /* tap 1 */
		  0,   0, /* tap 2 */
		 -1,   0, /* tap 3 */
		  0,  -1, /* tap 4 */
		  7,   3, /* tap 5 */
		 17,  12, /* tap 6 */
		 21,  20, /* tap 7 */
		 17,  20, /* tap 8 */
		  7,  12, /* tap 9 */
		  0,   3, /* tap 10 */
		 -1,  -1, /* tap 11 */
		  0,   0, /* tap 12 */
		  0,   0, /* tap 13 */
		  0,  -1, /* tap 14 */
		 -1,  -1, /* tap 15 */
	},
};

/* Configuration for scaling factor 2 / 3 using lanczos
 * with 2 lobes.
 *
 * This configuration uses 8 taps and 4 phases.
 *
 *                +
 *               +|+
 *                |
 *              + | +
 *                |
 *             +  |  +
 *                |
 *            +   |   +
 *                |
 *           +    |    +
 *                |
 * +++++++-++-----------++-++++++++
 *        +       |       +
 */
static const struct avi_scal_dimcfg avi_scalcfg_2_3 = {
	.ntaps   = AVI_SCAL_NTAPS_8,
	.nphases = AVI_SCAL_NPHASES_4,
	.coeffs = {
		  0,  -1,  -1,  -1, /* tap 0 */
		 -3,  -2,   0,   0, /* tap 1 */
		 14,   6,   0,  -2, /* tap 2 */
		 42,  40,  33,  24, /* tap 3 */
		 14,  24,  33,  40, /* tap 4 */
		 -3,  -2,   0,   6, /* tap 5 */
		  0,   0,   0,  -2, /* tap 6 */
		  0,  -1,  -1,  -1, /* tap 7 */
	},
};

/* Configuration for scaling factor 3 / 4 using lanczos
 * with 2 lobes.
 *
 * This configuration uses 8 taps and 4 phases.
 *
 *                +
 *               +|+
 *                |
 *              + | +
 *                |
 *                |
 *             +  |  +
 *                |
 *                |
 *            +   |   +
 *                |
 *           +    |    +
 * ++++++++-+-----------+-+++++++++
 *         +      |      +
 */
static const struct avi_scal_dimcfg avi_scalcfg_3_4 = {
	.ntaps   = AVI_SCAL_NTAPS_8,
	.nphases = AVI_SCAL_NPHASES_4,
	.coeffs = {
		 -1,  -1,  -1,  -1, /* tap 0 */
		 -2,   0,   0,   0, /* tap 1 */
		 11,   2,  -2,  -3, /* tap 2 */
		 48,  44,  35,  23, /* tap 3 */
		 11,  23,  35,  44, /* tap 4 */
		 -2,  -3,  -2,   2, /* tap 5 */
		  0,   0,   0,   0, /* tap 6 */
		 -1,  -1,  -1,  -1, /* tap 7 */
	},
};

/* Configuration for scaling factor 1 / 4 using lanczos
 * with 2 lobes.
 *
 * This configuration uses 16 taps and 2 phases.
 *
 *              +++++
 *             +  |  +
 *           ++   |   ++
 *         ++     |     ++
 * ++++++++---------------+++++++++
 */
static const struct avi_scal_dimcfg avi_scalcfg_1_4 = {
	.ntaps   = AVI_SCAL_NTAPS_16,
	.nphases = AVI_SCAL_NPHASES_2,
	.coeffs = {
		 -1,  -1, /* tap 0 */
		 -1,  -1, /* tap 1 */
		 -1,  -1, /* tap 2 */
		  0,   0, /* tap 3 */
		  4,   2, /* tap 4 */
		  9,   6, /* tap 5 */
		 14,  12, /* tap 6 */
		 16,  15, /* tap 7 */
		 14,  15, /* tap 8 */
		  9,  12, /* tap 9 */
		  4,   6, /* tap 10 */
		  0,   2, /* tap 11 */
		  0,   0, /* tap 12 */
		 -1,  -1, /* tap 13 */
		 -1,  -1, /* tap 14 */
		 -1,  -1, /* tap 15 */
	},
};

#define SCAL_RATIO(_to, _from) ((_to) << 8 / (_from))

struct avi_scal_ratio_config {
	unsigned int                    ratio;
	struct avi_scal_dimcfg const*   cfg;
};

static const struct avi_scal_ratio_config avi_scal_ratios[] = {
#define ADD_SCAL_CFG(_to, _from)		\
	{ .ratio = SCAL_RATIO(_to, _from), &avi_scalcfg_ ##_to##_##_from }
	/* the matrix is the same for all upscaling ratios. It's also the default ratio */
	ADD_SCAL_CFG(1, 1),
	/* Various downscaling ratios. Sorted from smaller to bigger. */
	ADD_SCAL_CFG(3, 4),
	ADD_SCAL_CFG(2, 3),
	ADD_SCAL_CFG(1, 2),
	ADD_SCAL_CFG(1, 3),
	ADD_SCAL_CFG(1, 4),
#undef ADD_SCAL_CFG
};

static inline unsigned avi_scal_delta(unsigned a, unsigned b)
{
	if (a > b)
		return a - b;

	return b - a;
}

/*
 * Return scaler's FIR parameters according to sizes passed in argument for one
 * dimension.
 */
static struct avi_scal_dimcfg const* avi_scal_size2dimcfg(unsigned from,
							  unsigned to,
							  unsigned max_taps)
{
	unsigned long				 ratio = SCAL_RATIO(to, from);
	const struct avi_scal_ratio_config	*best  = &avi_scal_ratios[0];
	int					 i;

	for (i = 1; i < ARRAY_SIZE(avi_scal_ratios); i++) {
		const struct avi_scal_ratio_config *cur = &avi_scal_ratios[i];

		if (cur->cfg->ntaps > max_taps)
			continue;

		if (avi_scal_delta(cur->ratio, ratio) <
		    avi_scal_delta(best->ratio, ratio))
			best = cur;
	}

        return best->cfg;
}

static inline void avi_write_scal_coeffs(unsigned long base,
					 const char *coeffs)
{
#define SCAL_COEFFS_AT(_x) (coeffs[_x]             |		\
			    (coeffs[_x + 1] << 8)  |		\
			    (coeffs[_x + 2] << 16) |		\
			    (coeffs[_x + 3] << 24))

	AVI_WRITE(SCAL_COEFFS_AT(0),  base + AVI_SCAL_COEFF0300);
	AVI_WRITE(SCAL_COEFFS_AT(4),  base + AVI_SCAL_COEFF0704);
	AVI_WRITE(SCAL_COEFFS_AT(8),  base + AVI_SCAL_COEFF1108);
	AVI_WRITE(SCAL_COEFFS_AT(12), base + AVI_SCAL_COEFF1512);
	AVI_WRITE(SCAL_COEFFS_AT(16), base + AVI_SCAL_COEFF1916);
	AVI_WRITE(SCAL_COEFFS_AT(20), base + AVI_SCAL_COEFF2320);
	AVI_WRITE(SCAL_COEFFS_AT(24), base + AVI_SCAL_COEFF2724);
	AVI_WRITE(SCAL_COEFFS_AT(28), base + AVI_SCAL_COEFF3128);

#undef SCAL_COEFFS_AT
}

/*
 * Setup scaler's parameters for resizing a single plane (handling both
 * dimensions if required).
 * A NULL increment means no resize operation is requested for the given
 * dimension.
 * A NULL dimension configuration requests to leave coefficients unchanged.
 */
static void avi_setup_scalplane(unsigned long base,
				u32 woffset,
				u32 wincr,
				struct avi_scal_dimcfg const* wcfg,
				u32 hoffset,
				u32 hincr,
				struct avi_scal_dimcfg const* hcfg)
{
	u32 tp = 0;

	if (wincr) {
		/* Horizontal resizing requested. */
		tp = wcfg->ntaps << AVI_SCAL_NTAPSX_SHIFT |
			wcfg->nphases << AVI_SCAL_NPHASESX_SHIFT;

		AVI_WRITE(woffset,
			  base + AVI_SCAL_HORIZ_DIM + AVI_SCAL_OFFSET);
		AVI_WRITE(wincr, base + AVI_SCAL_HORIZ_DIM + AVI_SCAL_INCR);

		/* Setup FIR coefficients for horizontal dimension. */
		avi_write_scal_coeffs(base + AVI_SCAL_HORIZ_DIM, wcfg->coeffs);
	}

	if (hincr) {
		/* Vertical resizing requested. */
		tp |= hcfg->ntaps << AVI_SCAL_NTAPSY_SHIFT |
			hcfg->nphases << AVI_SCAL_NPHASESY_SHIFT;

		AVI_WRITE(hoffset,
			  base + AVI_SCAL_VERT_DIM + AVI_SCAL_OFFSET);
		AVI_WRITE(hincr, base + AVI_SCAL_VERT_DIM + AVI_SCAL_INCR);

		AVI_WRITE(hoffset,
			  base + AVI_SCAL_VERT_DIM + AVI_SCAL_OFFSET);
		AVI_WRITE(hincr, base + AVI_SCAL_VERT_DIM + AVI_SCAL_INCR);
		/* Setup FIR coefficients for vertical dimension. */
		avi_write_scal_coeffs(base + AVI_SCAL_VERT_DIM, hcfg->coeffs);
	}

	/* Setup FIR number of phases and taps for both dimensions. */
	AVI_WRITE(tp, base + AVI_SCAL_NTAPS_NPHASES);
}

/* Compute the base offset value in Q16.8.
 *
 * This value depends solely on the scaling factor.
 */
static inline unsigned avi_scal_offset(u16 from, u16 to)
{
	/* The offset is (F - 1) / 2, where F is the downscaling factor. This
	 * means that for upscaling (when F is smaller than 1) the offset is
	 * negative.
	 *
	 * The offset is stored in hardware in Q16.8, so we have to multiply of
	 * final offset value by 256. We can simplify ((F - 1) * 256 / 2) as
	 * (128 * F - 128) */
	unsigned offset = ((from << 7) + to - 1) / to;

	/* Here if offset is < 128 it means we are upscaling and the offset
	 * should be negative. However, the hardware doesn't handle negative
	 * offsets, so in this case I cheat by moving one pixel right and making
	 * the effective offset (1 + real_offset). This means we're skewed by
	 * one pixel but the phases still fall in the right places */
	if (offset < 128)
		return offset + 128; /* move one pixel to the right to get a
				      * positive value with the same phase */

	return offset - 128; /* true value */
}

unsigned avi_scal_line_taps(unsigned buffer_line_sz)
{
	unsigned ntaps;

	/* See how many lines we can buffer in the internal scaler RAM */
	ntaps = AVI_SCAL_RAMSZ / (buffer_line_sz * 4);

	if (ntaps < 2)
		/* We can't scale lines that big! */
		BUG();

	if (ntaps <= 4)
		return AVI_SCAL_NTAPS_4;
	if (ntaps <= 8)
		return AVI_SCAL_NTAPS_8;

	return AVI_SCAL_NTAPS_16;
}

static void avi_scal_calc_cfg(struct avi_scal_cfg*	config,
                              unsigned			from_width,
                              unsigned			to_width,
                              unsigned			from_height,
                              unsigned			to_height,
                              struct avi_dma_pixfmt	pixfmt,
                              int			have_planar)
{
	int			planar	= avi_pixfmt_is_planar(pixfmt)	&&
					      have_planar;
	enum avi_pixel_packing	packing = avi_pixfmt_get_packing(pixfmt);
	unsigned		bufln_sz;

	/* We can't bypass the scaler when it's configured in planar mode even
	 * if the frame size stays the same: we still have to upscale the chroma
	 * plane. */
	if (unlikely(to_width  == from_width  &&
	             to_height == from_height &&
	             !planar)) {
		/* no scaling needed, use bypass config */
		memset(config, 0, sizeof(*config));

		config->conf = AVI_SCAL_IN_SRC << AVI_SCAL_CONF_OUTSRC_SHIFT;

		return;
	}

	/* The vertical scaler has a line buffer. To optimize memory usage we
	 * have to make it store the smallest lines. */
	if (to_width > from_width) {
		/* We are enlarging the picture, put the vertical scaler
		 * first */
		config->conf = AVI_SCAL_IN_SRC << AVI_SCAL_CONF_VERTSRC_SHIFT
			| AVI_SCAL_VERT_SRC << AVI_SCAL_CONF_HORIZSRC_SHIFT
			| AVI_SCAL_HORIZ_SRC << AVI_SCAL_CONF_OUTSRC_SHIFT;
		bufln_sz = from_width;
	} else {
		/* We are thinning the picture, put the vertical scaler last */
		config->conf = AVI_SCAL_IN_SRC << AVI_SCAL_CONF_HORIZSRC_SHIFT
			| AVI_SCAL_HORIZ_SRC << AVI_SCAL_CONF_VERTSRC_SHIFT
			| AVI_SCAL_VERT_SRC << AVI_SCAL_CONF_OUTSRC_SHIFT;
		bufln_sz = to_width;
	}

	config->size = (to_height - 1) << AVI_SCAL_HEIGHT_SHIFT |
		(to_width - 1) << AVI_SCAL_WIDTH_SHIFT;

	config->wcfg0 = avi_scal_size2dimcfg(from_width,
					     to_width,
					     AVI_SCAL_NTAPS_16);

	config->wincr0	  = (from_width - config->wcfg0->ntaps) << 8;
	config->wincr0	 /= max(to_width - 1, 1U);
	config->woffset0  = avi_scal_offset(from_width, to_width);

	config->hcfg0 = avi_scal_size2dimcfg(from_height,
					     to_height,
					     avi_scal_line_taps(bufln_sz));

	config->hincr0	  = (from_height - config->hcfg0->ntaps) << 8;
	config->hincr0	 /= max(to_height - 1, 1U);
	config->hoffset0  = avi_scal_offset(from_height, to_height);

	config->conf |= (!!planar) << AVI_SCAL_CONF_PLANAR_SHIFT;

	if (planar) {
		switch(packing) {
		case AVI_INTERLEAVED_444_PACKING:
		case AVI_INTERLEAVED_YUV_422_PACKING:
		default:
			config->wincr1   = 0;
			config->wcfg1    = 0;
			config->woffset1 = 0;
			config->hincr1   = 0;
			config->hcfg1    = 0;
			config->hoffset1 = 0;
			break;
		case AVI_SEMIPLANAR_YUV_420_PACKING:
			config->wincr1   = config->wincr0 / 2;
			config->wcfg1    = config->wcfg0;
			config->woffset1 = config->woffset0;
			config->hincr1   = config->hincr0 / 2;
			config->hcfg1    = config->hcfg0;
			config->hoffset1 = config->hoffset0;
			break;
			/* NV16 */
		case AVI_SEMIPLANAR_YUV_422_PACKING:
			config->wincr1   = config->wincr0 / 2;
			config->wcfg1    = config->wcfg0;
			config->woffset1 = config->woffset0;
			config->hincr1   = config->hincr0;
			config->hcfg1    = config->hcfg0;
			config->hoffset1 = config->hoffset0;
			break;
		}
	} else {
		config->wincr1   = 0;
		config->wcfg1    = 0;
		config->woffset1 = 0;
		config->hincr1   = 0;
		config->hcfg1    = 0;
		config->hoffset1 = 0;
	}
}

/**
 * avi_scal_bypass() - Configure a scaler node as passthrough
 */
void avi_scal_bypass(struct avi_node const *scaler)
{
	unsigned long const base = avi_node_base(scaler);

	AVI_WRITE(AVI_SCAL_IN_SRC << AVI_SCAL_CONF_OUTSRC_SHIFT,
		  base + AVI_SCAL_CONF);
}
EXPORT_SYMBOL(avi_scal_bypass);

/**
 * avi_scal_to_cfg() - Compute scaler configuration
 * @config: the structure we'll fill with the computed config
 * @from_width: frame's original width
 * @to_width: width to resize frame to
 * @from_height: frame's original height
 * @to_height: height to resize frame to
 */
void avi_scal_to_cfg(struct avi_scal_cfg* config,
                     u16 from_width,
                     u16 to_width,
                     u16 from_height,
                     u16 to_height,
                     struct avi_dma_pixfmt pixfmt,
                     int have_planar)
{
	avi_scal_calc_cfg(config,
			  from_width, to_width,
			  from_height, to_height,
			  pixfmt,
			  have_planar);
}
EXPORT_SYMBOL(avi_scal_to_cfg);

/**
 * avi_scal_to_interleaced_cfg() - Compute scaler configuration for interleaced
 *                                 output.
 * @top_config: the structure we'll fill with the computed config for the top
 *              field
 * @bot_config: the structure we'll fill with the computed config for the bottom
 *              field
 * @from_width: frame's original width
 * @to_width: width to resize frame to
 * @from_height: frame's original height
 * @to_height: height to resize frame to (the complete frame size, not just the
 *             field's).
 */
void avi_scal_to_interleaced_cfg(struct avi_scal_cfg *top_config,
                                 struct avi_scal_cfg *bot_config,
                                 u16 from_width,
                                 u16 to_width,
                                 u16 from_height,
                                 u16 to_height,
                                 struct avi_dma_pixfmt pixfmt,
                                 int have_planar)
{
	avi_scal_calc_cfg(top_config,
			  from_width, to_width,
			  from_height, roundup(to_height, 2) / 2,
			  pixfmt,
			  have_planar);

	*bot_config = *top_config;

	/* We offset the bottom field by one output line. Incr moves two output
	 * lines at a time (since we divided to_height by two above) so we can
	 * use that to speed up the computation. */
	bot_config->hoffset0 += bot_config->hincr0 / 2;
	bot_config->hoffset1 += bot_config->hincr1 / 2;
}
EXPORT_SYMBOL(avi_scal_to_interleaced_cfg);

/*
 * avi_setup_scal() - Apply configuration to scaler.
 * @scaler: node pointing to the scaler to setup
 * @width: output width after resize operation
 * @height: output height after resize operation
 * @config: configuration to apply
 */
void avi_scal_setup(struct avi_node const* scaler,
                    struct avi_scal_cfg const* config)
{
	unsigned long const base = avi_node_base(scaler);

	/* Setup format and resizing stages. */
	AVI_WRITE(config->conf, base + AVI_SCAL_CONF);

	/* Setup dimensions sizes. */
	AVI_WRITE(config->size, base + AVI_SCAL_SIZEOUT);

	/* Setup plane 0. */
	avi_setup_scalplane(base + AVI_SCAL_PLANE0,
			    config->woffset0,
			    config->wincr0,
			    config->wcfg0,
			    config->hoffset0,
			    config->hincr0,
			    config->hcfg0);

	if (config->wincr1 || config->hincr1)
		avi_setup_scalplane(base + AVI_SCAL_PLANE1,
				    config->woffset1,
				    config->wincr1,
				    config->wcfg1,
				    config->hoffset1,
				    config->hincr1,
				    config->hcfg1);
	else
		/* Otherwise we use the same config for both planes */
		avi_setup_scalplane(base + AVI_SCAL_PLANE1,
				    config->woffset0,
				    config->wincr0,
				    config->wcfg0,
				    config->hoffset0,
				    config->hincr0,
				    config->hcfg0);
}
EXPORT_SYMBOL(avi_scal_setup);

void avi_scal_setup_oneshot(struct avi_node const *scaler,
                            u16 from_width,  u16 to_width,
                            u16 from_height, u16 to_height,
                            struct avi_dma_pixfmt pixfmt,
                            int have_planar)
{
	struct avi_scal_cfg scalcfg;

	avi_scal_to_cfg(&scalcfg,
			from_width, to_width,
			from_height, to_height,
			pixfmt,
			have_planar);
	avi_scal_setup(scaler, &scalcfg);
}
EXPORT_SYMBOL(avi_scal_setup_oneshot);

void avi_scal_set_field(struct avi_node const	*scaler,
                        u16			 from_height,
                        u16			 to_height,
                        enum avi_field		 field)
{
	/* I'm not sure if this code works for planar but I'm pretty sure the
	 * rest of the code in this file is broken for planar anyway. */
	unsigned long	base = avi_node_base(scaler) + AVI_SCAL_PLANE0;
	unsigned	hoffset;
	u32		hincr;

	hoffset = avi_scal_offset(from_height, to_height);

	if (field == AVI_BOT_FIELD) {
		hincr = AVI_READ(base + AVI_SCAL_VERT_DIM + AVI_SCAL_INCR);
		/* Offset by one line */
		hoffset += hincr / 2;
	}

	AVI_WRITE(hoffset, base + AVI_SCAL_VERT_DIM + AVI_SCAL_OFFSET);

	/* XXX update plan1 hoffset if needed */
}
EXPORT_SYMBOL(avi_scal_set_field);
