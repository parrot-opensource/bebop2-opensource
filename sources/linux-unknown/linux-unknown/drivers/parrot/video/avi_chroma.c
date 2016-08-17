#include <linux/module.h>
#include "avi_chroma.h"

/************************************
 * Chroma converters low-level setup
 ************************************/

/* The Q311 macro converts from a float to a fixed point 3.11 2's complement
 * integer.
 *
 * Q311(1.0)  => 0x0800
 * Q311(0.0)  => 0x0000
 * Q311(1.42) => 0x0b5c
 * Q311(-3.3) => 0x259a
 */
#define ABS(i) ((i) < 0 ? -(i) : (i))
#define COMPLEMENT_2(i, r) (((i) >= 0) ? (r) : (~(r) + 1) & 0x3fff)
#define Q311(i) (COMPLEMENT_2(i, (unsigned)(((ABS(i)) * (1 << 11)) + 0.5)))

/*
 * Chroma converter parameters to convert input stream from a given
 * color space to another.
 * See http://www.fourcc.org/fccyvrgb.php
 */

#define AVI_CONV_MATRIX(_c00, _c01, _c02,				     \
			_c10, _c11, _c12,				     \
			_c20, _c21, _c22)				     \
	.coeff_01_00 = {{ .coeff_00 = Q311(_c00), .coeff_01 = Q311(_c01) }}, \
	.coeff_10_02 = {{ .coeff_02 = Q311(_c02), .coeff_10 = Q311(_c10) }}, \
	.coeff_12_11 = {{ .coeff_11 = Q311(_c11), .coeff_12 = Q311(_c12) }}, \
	.coeff_21_20 = {{ .coeff_20 = Q311(_c20), .coeff_21 = Q311(_c21) }}, \
	.coeff_22    = {{ .coeff_22 = Q311(_c22) }}

#define AVI_CONV_OFFSETS(_ryin, _ryout,					\
			 _guin, _guout,					\
			 _bvin, _bvout)					\
	.offset_ry = {{ .offset_in = _ryin, .offset_out = _ryout }},	\
	.offset_gu = {{ .offset_in = _guin, .offset_out = _guout }},	\
	.offset_bv = {{ .offset_in = _bvin, .offset_out = _bvout }}

#define AVI_CONV_CLIPS(_rymin, _rymax,					\
		       _gumin, _gumax,					\
		       _bvmin, _bvmax)					\
	.clip_ry = {{ .clip_min = _rymin, .clip_max = _rymax }},	\
	.clip_gu = {{ .clip_min = _gumin, .clip_max = _gumax }}, 	\
	.clip_bv = {{ .clip_min = _bvmin, .clip_max = _bvmax }}

/* Identity config: no conversion */
#define AVI_CONV_CONF_ID				\
	AVI_CONV_MATRIX(1, 0, 0,			\
			0, 1, 0,			\
			0, 0, 1),			\
	AVI_CONV_OFFSETS(0, 0, 0, 0, 0, 0),		\
	AVI_CONV_CLIPS(0, 0xff, 0, 0xff, 0, 0xff)

struct avi_isp_chroma_regs const avi_conv_cfg[AVI_CSPACE_NR][AVI_CSPACE_NR] = {
	/*
	 * No / transparent converter
	 * digital RGB[0-255] To digital RGB[0-255]
	 */
	[AVI_NULL_CSPACE] = {
		[AVI_NULL_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_RGB_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_BT601_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_BT709_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_JFIF_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_GREY_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
	},
	/*
	 * BT.601 (usually SDTV) to digital RGB
	 * Y[16-235]/CbCr[16-240] To digital RGB[0-255]
	 * R' = 1.164(Y-16)                 + 1.596(Cr-128)
	 * G' = 1.164(Y-16) - 0.391(Cb-128) - 0.813(Cr-128)
	 * B' = 1.164(Y-16) + 2.018(Cb-128)
	 */
	[AVI_BT601_CSPACE] = {
		[AVI_NULL_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_RGB_CSPACE] = {
			AVI_CONV_MATRIX(1.164,  0.0  ,  1.596,
					1.164, -0.391, -0.813,
					1.164,  2.018,  0.0),
			AVI_CONV_OFFSETS(16 , 0,
					 128, 0,
					 128, 0),
			AVI_CONV_CLIPS(0, 0xff,
				       0, 0xff,
				       0, 0xff),
		},
		[AVI_BT601_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_BT709_CSPACE] = {
			AVI_CONV_MATRIX(1.146, 0.015, 0.003,
					0.008, 1.153, 0.003,
					0.008, 0.015, 1.142),
			AVI_CONV_OFFSETS(16 , 16 ,
					 128, 128,
					 128, 128),
			AVI_CONV_CLIPS(16, 235,
				       16, 240,
				       16, 240),
		},
		[AVI_GREY_CSPACE] = {
			AVI_CONV_MATRIX(1, 0, 0,
					0, 1, 0,
					0, 0, 1),
			AVI_CONV_OFFSETS(0 , 0,
					 0, 0,
					 0, 0),
			AVI_CONV_CLIPS(0, 0xff,
				       0, 0,
				       0, 0),
		},
	},
	/*
	 * BT.709 (usually HDTV) to digital RGB
	 * Y[16-235]/CbCr[16-240] To digital RGB[0-255]
	 * R' = 1.164(Y-16)                 + 1.793(Cr-128)
	 * G' = 1.164(Y-16) - 0.213(Cb-128) - 0.534(Cr-128)
	 * B' = 1.164(Y-16) + 2.115(Cb-128)
	 */
	[AVI_BT709_CSPACE] = {
		[AVI_NULL_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_RGB_CSPACE] = {
			AVI_CONV_MATRIX(1.0,  0.0  ,  1.402,
					1.0, -0.344, -0.714,
					1.0,  1.772,  0.0),
			AVI_CONV_OFFSETS(16 , 0,
					 128, 0,
					 128, 0),
			AVI_CONV_CLIPS(0, 0xff,
				       0, 0xff,
				       0, 0xff),
		},
		[AVI_BT601_CSPACE] = {
			AVI_CONV_MATRIX( 0.873, -0.012, -0.002,
					-0.006,  0.867, -0.002,
					-0.006, -0.012,  0.876),
			AVI_CONV_OFFSETS(16 , 16 ,
					 128, 128,
					 128, 128),
			AVI_CONV_CLIPS(16, 235,
				       16, 240,
				       16, 240),
		},
		[AVI_BT709_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_GREY_CSPACE] = {
			AVI_CONV_MATRIX( 0.873, -0.012, -0.002,
					-0.006,  0.867, -0.002,
					-0.006, -0.012,  0.876),
			AVI_CONV_OFFSETS(16 , 16 ,
					 128, 128,
					 128, 128),
			AVI_CONV_CLIPS(16, 235,
				       0, 0,
				       0, 0),
		},
	},
	[AVI_RGB_CSPACE] = {
		[AVI_NULL_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_RGB_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_BT601_CSPACE] = {
			AVI_CONV_MATRIX( 0.257,  0.504,  0.098,
					-0.148, -0.291,  0.439,
					 0.439, -0.368, -0.071),
			AVI_CONV_OFFSETS(0, 16,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(16, 235,
				       16, 240,
				       16, 240),
		},
		[AVI_BT709_CSPACE] = {
			AVI_CONV_MATRIX(  0.213,  0.715,  0.072,
					 -0.100, -0.336,  0.436,
					  0.615, -0.515, -0.100),
			AVI_CONV_OFFSETS(0, 16,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(16, 235,
				       16, 240,
				       16, 240),
		},
		[AVI_JFIF_CSPACE] = {
			AVI_CONV_MATRIX( 0.299,   0.587,   0.114,
					-0.1697, -0.3313,  0.5,
					 0.5,    -0.4187, -0.0813),
			AVI_CONV_OFFSETS(0, 0,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_GREY_CSPACE] = {
			AVI_CONV_MATRIX( 0.257,  0.504,  0.098,
					-0.148, -0.291,  0.439,
					 0.439, -0.368, -0.071),
			AVI_CONV_OFFSETS(0, 16,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(16, 235,
				       0, 0,
				       0, 0),
		},
	},
	[AVI_GREY_CSPACE] = {
		[AVI_NULL_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_RGB_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0, 0,
					 1, 0, 0,
					 1, 0, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 0,
					 0, 0),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_BT601_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0, 0,
					 0, 0, 0,
					 0, 0, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_BT709_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0, 0,
					 0, 0, 0,
					 0, 0, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_GREY_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
	},
	[AVI_Y10_CSPACE] = {
		[AVI_NULL_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
		[AVI_RGB_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0.03125, 0,
					 1, 0.03125, 0,
					 1, 0.03125, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 0,
					 0, 0),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_BT601_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0.03125, 0,
					 0, 0, 0,
					 0, 0, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_BT709_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0.03125, 0,
					 0, 0, 0,
					 0, 0, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 128,
					 0, 128),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_GREY_CSPACE] = {
			AVI_CONV_MATRIX( 1, 0.03125, 0,
					 0, 0, 0,
					 0, 0, 0),
			AVI_CONV_OFFSETS(0, 0,
					 0, 0,
					 0, 0),
			AVI_CONV_CLIPS(0, 255,
				       0, 255,
				       0, 255),
		},
		[AVI_Y10_CSPACE] = {
			AVI_CONV_CONF_ID,
		},
	},
};
EXPORT_SYMBOL(avi_conv_cfg);

/***********************
 * Color space handling
 ***********************/

static inline void memcpy_to_registers(unsigned long addr,
				       const void *reg_base,
				       size_t s)
{
	const u32 *reg = reg_base;
	unsigned  i;

	BUG_ON(s % sizeof(u32) != 0);
	s /= sizeof(u32); /* we write one register at a time */

	for (i = 0; i < s; i++)
		AVI_WRITE(reg[i], addr + i * sizeof(u32));
	wmb();
}

static void avi_setup_conv(struct avi_node const* conv,
			   struct avi_isp_chroma_regs const* regs)
{
	unsigned long base = avi_node_base(conv);

	BUG_ON(avi_node_type(conv) != AVI_CONV_NODE_TYPE);

	memcpy_to_registers(base, regs, sizeof(*regs));
}

static const struct avi_isp_chroma_regs *
avi_conv_get_matrix(enum avi_colorspace from,
		    enum avi_colorspace to)
{
	const struct avi_isp_chroma_regs *matrix;

	BUG_ON(from >= AVI_CSPACE_NR);
	BUG_ON(to   >= AVI_CSPACE_NR);

	matrix = &avi_conv_cfg[from][to];

	// Check if the conversion matrix is initialized.
	BUG_ON(matrix->coeff_01_00.coeff_00 == 0);

	return matrix;
}

void avi_setup_conv_colorspace(struct avi_node const *conv,
			       enum avi_colorspace from,
			       enum avi_colorspace to)
{
	avi_setup_conv(conv, avi_conv_get_matrix(from, to));
}
EXPORT_SYMBOL(avi_setup_conv_colorspace);

void avi_conv_get_registers(struct avi_node const* conv,
			    struct avi_isp_chroma_regs* regs)
{
	memcpy_from_registers(regs, avi_node_base(conv), sizeof(*regs));
}
EXPORT_SYMBOL(avi_conv_get_registers);

/* Convert Q3.11 into Q21.11 to use the native two complement's
   representation. Without that arithmetics on negative Q3.11 values yield
   incorrect results. */
static s32 avi_q311_to_q2111(u32 q)
{
	/* Check if the sign bit is set, and if so extend it to 32bits */
	if (q & (1 << 13)) {
		q |= 0xffffc000;
	}

	/* We can now cast it as a native s32 since the two's complement
	   representation is correct. */
	return (s32)q;
}

/* Compute the dot product of two vectors, the 2nd being expressed as
   Q3.11 fixed point. The result is Q22.11 (really Q6.11 with sign
   extension, makes sense right?) */
static s32 avi_q311_dot_product(s32 r,  s32 g,  s32 b,
				u32 qr, u32 qg, u32 qb)
{
	s32 fr = avi_q311_to_q2111(qr);
	s32 fg = avi_q311_to_q2111(qg);
	s32 fb = avi_q311_to_q2111(qb);

	return r * fr + g * fg + b * fb;
}

/* Round Q3.11 integer representation to the nearest value. Not sure
   if that works properly for negative values but those will get
   clipped afterwards anyway. */
static s32 avi_q311_to_s32(s32 q) {
	return (q + (1 << 10)) >> 11;
}

static u32 avi_conv_clip(s32 c, u32 clip_min, u32 clip_max)
{
	u32 uc = c;

	if (c <= 0) {
		return clip_min;
	}

	if (uc < clip_min) {
		return clip_min;
	} else if (uc > clip_max) {
		return clip_max;
	}

	return uc;
}

/* Simulate CONV color conversion in software. The only complexity
   comes from the fact that the CONV factors are stored as Q3.11 which
   requires a fair bit of fixed point arithmetics. I haven't tested
   this against the actual C model so I don't garantee bit
   accuracy. */
u32 avi_conv_convert_color(enum avi_colorspace from,
			   enum avi_colorspace to,
			   u32                 color)
{
	const struct avi_isp_chroma_regs *matrix;

	s32 ry = (color >> 16) & 0xff;
	s32 gu = (color >> 8)  & 0xff;
	s32 bv = (color >> 0)  & 0xff;

	s32 o_ry, o_gu, o_bv;

	matrix = avi_conv_get_matrix(from, to);

	/* Substract input offsets */
	ry -= matrix->offset_ry.offset_in;
	gu -= matrix->offset_gu.offset_in;
	bv -= matrix->offset_bv.offset_in;

	/* Handle matrix transformation */
	o_ry = avi_q311_dot_product(ry,
				    gu,
				    bv,
				    matrix->coeff_01_00.coeff_00,
				    matrix->coeff_01_00.coeff_01,
				    matrix->coeff_10_02.coeff_02);

	o_gu = avi_q311_dot_product(ry,
				    gu,
				    bv,
				    matrix->coeff_10_02.coeff_10,
				    matrix->coeff_12_11.coeff_11,
				    matrix->coeff_12_11.coeff_12);

	o_bv = avi_q311_dot_product(ry,
				    gu,
				    bv,
				    matrix->coeff_21_20.coeff_20,
				    matrix->coeff_21_20.coeff_21,
				    matrix->coeff_22   .coeff_22);

	/* Convert from fixed point to integer by rounding */
	o_ry = avi_q311_to_s32(o_ry);
	o_gu = avi_q311_to_s32(o_gu);
	o_bv = avi_q311_to_s32(o_bv);

	/* Add output offsets */
	o_ry += matrix->offset_ry.offset_out;
	o_gu += matrix->offset_gu.offset_out;
	o_bv += matrix->offset_bv.offset_out;

	/* Clip */
	o_ry = avi_conv_clip(o_ry,
			     matrix->clip_ry.clip_min,
			     matrix->clip_ry.clip_max);
	o_gu = avi_conv_clip(o_gu,
			     matrix->clip_gu.clip_min,
			     matrix->clip_gu.clip_max);
	o_bv = avi_conv_clip(o_bv,
			     matrix->clip_bv.clip_min,
			     matrix->clip_bv.clip_max);

	return (o_ry << 16) | (o_gu << 8) | o_bv;
}
EXPORT_SYMBOL(avi_conv_convert_color);
