#ifndef _AVI_PIXFMT_H_
#define _AVI_PIXFMT_H_

#include "avi_dma.h"
#include "reg_avi.h"

#define AVI_PIXEL_FORMATS(EXPANDER)	\
	EXPANDER(INVALID),		\
	EXPANDER(RGBA8888),		\
	EXPANDER(BGRA8888),		\
	EXPANDER(ARGB8888),		\
	EXPANDER(ABGR8888),		\
	EXPANDER(RGB888),		\
	EXPANDER(BGR888),		\
	EXPANDER(RGB565),		\
	EXPANDER(BGR565),		\
	EXPANDER(RGBA5551),		\
	EXPANDER(AYUV),			\
	EXPANDER(YUYV),			\
	EXPANDER(YYUV),			\
	EXPANDER(YVYU),			\
	EXPANDER(UYVY),			\
	EXPANDER(VYUY),			\
	EXPANDER(NV16),			\
	EXPANDER(NV61),			\
	EXPANDER(NV12),			\
	EXPANDER(NV21),			\
	EXPANDER(GREY),			\
	EXPANDER(Y10),			\
	EXPANDER(BAYER_8),		\
	EXPANDER(BAYER_1X10_16),	\
	EXPANDER(BAYER_1X10_24),	\
	EXPANDER(BAYER_1X10_32),	\
	EXPANDER(BAYER_3X10_32),	\
	EXPANDER(BAYER_1X12_16)

/* Do not use these tags directly, use the corresponding avi_dma_pixfmt declared
 * below instead */
#define AVI_PIXEL_FORMAT_AS_ENUM(_x) AVI_ ## _x ## _FMTID
enum avi_dma_pixfmt_ids {
	AVI_PIXEL_FORMATS(AVI_PIXEL_FORMAT_AS_ENUM),
	AVI_PIXFMT_NR
};

static inline unsigned avi_pixel_size0(struct avi_dma_pixfmt pixfmt)
{
	return pixfmt.bytes_per_pixel0 + 1;
}

static inline unsigned avi_pixel_size1(struct avi_dma_pixfmt pixfmt)
{
	/* Certain format pretend they're planar but don't actually
	 * use the chroma plane. */
	if (pixfmt.semiplanar) {
		if (pixfmt.raw || pixfmt.id == AVI_GREY_FMTID)
			return 0;
		else
			return pixfmt.bytes_per_pixel1 + 1;
	}

	return 0;
}

enum avi_pixel_packing {
	AVI_INTERLEAVED_444_PACKING,
	AVI_INTERLEAVED_YUV_422_PACKING,
	AVI_SEMIPLANAR_YUV_420_PACKING,
	AVI_SEMIPLANAR_YUV_422_PACKING,
};

static inline int avi_pixfmt_is_planar(struct avi_dma_pixfmt pixfmt)
{
	return !!avi_pixel_size1(pixfmt);
}

static inline enum avi_pixel_packing
avi_pixfmt_get_packing(struct avi_dma_pixfmt pixfmt)
{
	if (!avi_pixfmt_is_planar(pixfmt))
		return (pixfmt.subsampling == AVI_FIFO_CFG_444_FMT)?
				AVI_INTERLEAVED_444_PACKING:
				AVI_INTERLEAVED_YUV_422_PACKING;

	switch (pixfmt.subsampling) {
	case AVI_FIFO_CFG_422_FMT:
		return AVI_SEMIPLANAR_YUV_422_PACKING;
	case AVI_FIFO_CFG_420_FMT:
		return AVI_SEMIPLANAR_YUV_420_PACKING;
	case AVI_FIFO_CFG_444_FMT:
		/* Doesn't make sense to have 4:4:4 planar */
	default:
		BUG();
		break;
	}
}

extern int avi_pixfmt_have_alpha(struct avi_dma_pixfmt pixfmt);

#define AVI_PIXFMT_INVALID ((struct avi_dma_pixfmt) {            \
	.id               = AVI_INVALID_FMTID,                   \
})

#define AVI_PIXFMT_RGBA8888 ((struct avi_dma_pixfmt) {           \
	.id               = AVI_RGBA8888_FMTID,                  \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 0,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 0,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BGRA8888 ((struct avi_dma_pixfmt) {           \
	.id               = AVI_BGRA8888_FMTID,                  \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_ARGB8888 ((struct avi_dma_pixfmt) {           \
	.id               = AVI_ARGB8888_FMTID,                  \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 0,                                   \
	.swap0_b1         = 1,                                   \
	.swap0_b2         = 2,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 1,                                   \
	.swap1_b2         = 2,                                   \
})

#define AVI_PIXFMT_ABGR8888 ((struct avi_dma_pixfmt) {           \
	.id               = AVI_ABGR8888_FMTID,                  \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 0,                                   \
	.swap0_b1         = 3,                                   \
	.swap0_b2         = 2,                                   \
	                                                         \
	.swap1_b0         = 2,                                   \
	.swap1_b1         = 1,                                   \
	.swap1_b2         = 0,                                   \
})

#define AVI_PIXFMT_RGB888 ((struct avi_dma_pixfmt) {             \
	.id               = AVI_RGB888_FMTID,                    \
	.bitenc           = AVI_FIFO_CFG_888_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_VUY_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 3 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BGR888 ((struct avi_dma_pixfmt) {             \
	.id               = AVI_BGR888_FMTID,                    \
	.bitenc           = AVI_FIFO_CFG_888_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 3 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_RGB565 ((struct avi_dma_pixfmt) {             \
	.id               = AVI_RGB565_FMTID,                    \
	.bitenc           = AVI_FIFO_CFG_565_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BGR565 ((struct avi_dma_pixfmt) {             \
	.id               = AVI_BGR565_FMTID,                    \
	.bitenc           = AVI_FIFO_CFG_565_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_VUY_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_RGBA5551 ((struct avi_dma_pixfmt) {           \
	.id               = AVI_RGBA5551_FMTID,                  \
	.bitenc           = AVI_FIFO_CFG_1555_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_AYUV ((struct avi_dma_pixfmt) {               \
	.id               = AVI_AYUV_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 0,                                   \
	.swap0_b1         = 1,                                   \
	.swap0_b2         = 2,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 1,                                   \
	.swap1_b2         = 2,                                   \
})

#define AVI_PIXFMT_YUYV ((struct avi_dma_pixfmt) {               \
	.id               = AVI_YUYV_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 2,                                   \
	.swap0_b1         = 1,                                   \
	.swap0_b2         = 0,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 3,                                   \
	.swap1_b2         = 2,                                   \
})

#define AVI_PIXFMT_YYUV ((struct avi_dma_pixfmt) {               \
	.id               = AVI_YYUV_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 1,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 0,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 3,                                   \
})

#define AVI_PIXFMT_YVYU ((struct avi_dma_pixfmt) {               \
	.id               = AVI_YVYU_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 2,                                   \
	.swap0_b1         = 3,                                   \
	.swap0_b2         = 0,                                   \
	                                                         \
	.swap1_b0         = 2,                                   \
	.swap1_b1         = 3,                                   \
	.swap1_b2         = 0,                                   \
})

#define AVI_PIXFMT_UYVY ((struct avi_dma_pixfmt) {               \
	.id               = AVI_UYVY_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 0,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 0,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_VYUY ((struct avi_dma_pixfmt) {               \
	.id               = AVI_VYUY_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_NV16 ((struct avi_dma_pixfmt) {               \
	.id               = AVI_NV16_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 1,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 1 - 1,                               \
	.bytes_per_pixel1 = 1 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 2,                                   \
	.swap1_b1         = 3,                                   \
	.swap1_b2         = 0,                                   \
})

#define AVI_PIXFMT_NV61 ((struct avi_dma_pixfmt) {               \
	.id               = AVI_NV61_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 1,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 1 - 1,                               \
	.bytes_per_pixel1 = 1 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_NV12 ((struct avi_dma_pixfmt) {               \
	.id               = AVI_NV12_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_420_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 1,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 1 - 1,                               \
	.bytes_per_pixel1 = 1 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 2,                                   \
	.swap1_b1         = 3,                                   \
	.swap1_b2         = 0,                                   \
})

#define AVI_PIXFMT_NV21 ((struct avi_dma_pixfmt) {               \
	.id               = AVI_NV21_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_420_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 1,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 1 - 1,                               \
	.bytes_per_pixel1 = 1 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_GREY ((struct avi_dma_pixfmt) {               \
	.id               = AVI_GREY_FMTID,                      \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 1,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 1 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 0,                                   \
	.swap1_b2         = 0,                                   \
})

#define AVI_PIXFMT_Y10 ((struct avi_dma_pixfmt) {                \
	.id               = AVI_Y10_FMTID,                       \
	.bitenc           = AVI_FIFO_CFG_565_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 0,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})



#define AVI_PIXFMT_BAYER_8 ((struct avi_dma_pixfmt) {            \
	.id               = AVI_BAYER_8_FMTID,                   \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 1,                                   \
	.raw              = 1,                                   \
	.bytes_per_pixel0 = 1 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 0,                                   \
	.swap1_b2         = 0,                                   \
})

#define AVI_PIXFMT_BAYER_1X10_16 ((struct avi_dma_pixfmt) {      \
	.id               = AVI_BAYER_1X10_16_FMTID,             \
	.bitenc           = AVI_FIFO_CFG_565_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 1,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BAYER_1X10_24 ((struct avi_dma_pixfmt) {      \
	.id               = AVI_BAYER_1X10_24_FMTID,             \
	.bitenc           = AVI_FIFO_CFG_888_BENC,               \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 1,                                   \
	.bytes_per_pixel0 = 3 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BAYER_1X10_32 ((struct avi_dma_pixfmt) {      \
	.id               = AVI_BAYER_1X10_32_FMTID,             \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 1,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BAYER_3X10_32 ((struct avi_dma_pixfmt) {      \
	.id               = AVI_BAYER_3X10_32_FMTID,             \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_444_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 1,                                   \
	.bytes_per_pixel0 = 4 - 1,                               \
	                                                         \
	.swap0_b0         = 3,                                   \
	.swap0_b1         = 2,                                   \
	.swap0_b2         = 1,                                   \
	                                                         \
	.swap1_b0         = 3,                                   \
	.swap1_b1         = 2,                                   \
	.swap1_b2         = 1,                                   \
})

#define AVI_PIXFMT_BAYER_1X12_16 ((struct avi_dma_pixfmt) {      \
	.id               = AVI_BAYER_1X12_16_FMTID,             \
	.bitenc           = AVI_FIFO_CFG_8888_BENC,              \
	.subsampling      = AVI_FIFO_CFG_422_FMT,                \
	.reorg            = AVI_FIFO_CFG_YUV_REORG,              \
	.semiplanar       = 0,                                   \
	.raw              = 1,                                   \
	.bytes_per_pixel0 = 2 - 1,                               \
	                                                         \
	.swap0_b0         = 2,                                   \
	.swap0_b1         = 1,                                   \
	.swap0_b2         = 0,                                   \
	                                                         \
	.swap1_b0         = 0,                                   \
	.swap1_b1         = 3,                                   \
	.swap1_b2         = 2,                                   \
})

extern const struct avi_dma_pixfmt avi_pixfmt_by_id(enum avi_dma_pixfmt_ids id);

#endif /* _AVI_PIXFMT_H_ */
