/*
 *  linux/drivers/parrot/video/reg_avi.h
 *
 *  Copyright (C) 2011 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  01-Feb-2011
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

#ifndef _REG_AVI_H
#define _REG_AVI_H

#include <asm/memory.h>

/***********************
 * Configuration module
 ***********************/
#define AVI_CFG UL(0)

#include "avi_regmap/avi_cfg.h"

/**********************
 * Interconnect module
 **********************/
#define AVI_INTER   UL(0x100)

#define AVI_INTER_SINKA UL(0x00)
#define AVI_INTER_SINKB UL(0x04)
#define AVI_INTER_SINKC UL(0x08)
#define AVI_INTER_SINKD UL(0x0c)
#define AVI_INTER_SINKE UL(0x10)
#define AVI_INTER_SINKF UL(0x14)
#define AVI_INTER_SINKG UL(0x18)
#define AVI_INTER_SINGH UL(0x1C)
/* Not present in MPW1 (GAM -> LCD connection is hardwired) */
#define AVI_INTER_SINKI UL(0x20)
#define AVI_INTER_SINKJ UL(0x24)
#define AVI_INTER_SINKK UL(0x28)
#define AVI_INTER_SINKL UL(0x2C)
#define AVI_INTER_NSINK 12

#define AVI_INTER_SRC_MASK UL(0x3F)

/**************
 * FIFO module
 **************/
#define AVI_FIFO    UL(0x1000)
#define AVI_FIFO0   AVI_FIFO
#define AVI_FIFO1   (AVI_FIFO + UL(0x100))
#define AVI_FIFO2   (AVI_FIFO + UL(0x200))
#define AVI_FIFO3   (AVI_FIFO + UL(0x300))
#define AVI_FIFO4   (AVI_FIFO + UL(0x400))
#define AVI_FIFO5   (AVI_FIFO + UL(0x500))
#define AVI_FIFO6   (AVI_FIFO + UL(0x600))
#define AVI_FIFO7   (AVI_FIFO + UL(0x700))
#define AVI_FIFO8   (AVI_FIFO + UL(0x800))
#define AVI_FIFO9   (AVI_FIFO + UL(0x900))
#define AVI_FIFO10  (AVI_FIFO + UL(0xa00))
#define AVI_FIFO11  (AVI_FIFO + UL(0xb00))

#define AVI_FIFO_CFG         UL(0x00)
#define AVI_FIFO_FRAMESIZE   UL(0x04)
#define AVI_FIFO_MONITOR     UL(0x08)
#define AVI_FIFO_TIMEOUT     UL(0x0C)
#define AVI_FIFO_SWAP0       UL(0x10)
#define AVI_FIFO_SWAP1       UL(0x14)
#define AVI_FIFO_DMASA0      UL(0x20)
#define AVI_FIFO_DMASA1      UL(0x24)
#define AVI_FIFO_DMALSTR0    UL(0x28)
#define AVI_FIFO_DMALSTR1    UL(0x2C)
#define AVI_FIFO_DMAFXYCFG   UL(0x30)
#define AVI_FIFO_DMAFXSTR0   UL(0x34)
#define AVI_FIFO_DMAFXSTR1   UL(0x38)
#define AVI_FIFO_DMAFYSTR0   UL(0x3C)
#define AVI_FIFO_DMAFYSTR1   UL(0x40)
#define AVI_FIFO_KEYINGMIN   UL(0x50)
#define AVI_FIFO_KEYINGMAX   UL(0x54)
#define AVI_FIFO_KEYINGALPHA UL(0x58)
#define AVI_FIFO_VLIN        UL(0x60)
#define AVI_FIFO_STATUS      UL(0x64)
#define AVI_FIFO_REG_COUNT   UL(0x68/4)

/*
 * FIFO configuration register bitfields
 */
#define AVI_FIFO_CFG_VL_TYPE        0   /* Video link to/from other AVI module. */
#define AVI_FIFO_CFG_DMA_TYPE       1   /* DMA to/from memory */

#define AVI_FIFO_CFG_PACKED_ORG     0   /* Packed frame data organization. */
#define AVI_FIFO_CFG_PLANAR_ORG     1   /* Semi-planar frame data organization. */

#define AVI_FIFO_CFG_444_FMT        0
#define AVI_FIFO_CFG_422_FMT        1
#define AVI_FIFO_CFG_420_FMT        3

#define AVI_FIFO_CFG_4444_BENC      0
#define AVI_FIFO_CFG_565_BENC       1
#define AVI_FIFO_CFG_LSB555_BENC    2
#define AVI_FIFO_CFG_1555_BENC      3
#define AVI_FIFO_CFG_888_BENC       4
#define AVI_FIFO_CFG_8888_BENC      8

#define AVI_FIFO_CFG_ARGB_REORG     0
#define AVI_FIFO_CFG_RGB_REORG      0
#define AVI_FIFO_CFG_YUV_REORG      0
#define AVI_FIFO_CFG_ABGR_REORG     1
#define AVI_FIFO_CFG_BGR_REORG      1
#define AVI_FIFO_CFG_VUY_REORG      1
#define AVI_FIFO_CFG_RGBA_REORG     2
#define AVI_FIFO_CFG_GRB_REORG      2
#define AVI_FIFO_CFG_UYV_REORG      2
#define AVI_FIFO_CFG_BGRA_REORG     3
#define AVI_FIFO_CFG_BRG_REORG      3
#define AVI_FIFO_CFG_VYU_REORG      3

#define AVI_FIFO_CFG_TRUNC_RND      0
#define AVI_FIFO_CFG_CONV_RND       1
#define AVI_FIFO_CFG_DYNTRUNC_RND   2
#define AVI_FIFO_CFG_DYNCONV_RND    3

#define AVI_FIFO_CFG_TRUNC_RND      0
#define AVI_FIFO_CFG_CONV_RND       1
#define AVI_FIFO_CFG_DYNTRUNC_RND   2
#define AVI_FIFO_CFG_DYNCONV_RND    3

#define AVI_FIFO_CFG_ENABLE_CUS     0   /* Enable chroma up-sampling. */
#define AVI_FIFO_CFG_DISABLE_CUS    1   /* Disable chroma up-sampling. */

#define AVI_FIFO_CFG_ENABLE_CDS     0   /* Enable chroma down-sampling. */
#define AVI_FIFO_CFG_DISABLE_CDS    1   /* Disable chroma down-sampling. */

#define AVI_FIFO_CFG_0_PHASE        0   /* O° chroma up-sampling phase. */
#define AVI_FIFO_CFG_180_PHASE      0   /* 180° chroma up-sampling phase. */

#define AVI_FIFO_CFG_Y_SCALPLANE    0
#define AVI_FIFO_CFG_UV_SCALPLANE   1

#define AVI_FIFO_CFG_BURST16        0   /* 16 words DMA bursts */
#define AVI_FIFO_CFG_BURST32        1   /* 32 words DMA bursts */

/**************************
 * Scaler / Rotator module
 **************************/

#define AVI_SCALROT     UL(0x7000)

/* Offsets 0x0 -> 0x100 of the ScalRot block are used for
 * reserving memory slots for a specific SCAL/ROT module
 * instead of dynamic allocation */
#define AVI_ISP_SCAL_ROT_SCAL0_RESERVE    (AVI_SCALROT + UL(0x00))
#define AVI_ISP_SCAL_ROT_ROT0_RESERVE     (AVI_SCALROT + UL(0x04))
#define AVI_ISP_SCAL_ROT_SCAL1_RESERVE    (AVI_SCALROT + UL(0x08))
#define AVI_ISP_SCAL_ROT_ROT1_RESERVE     (AVI_SCALROT + UL(0x0C))

#define AVI_SCALROT0    (AVI_SCALROT + UL(0x100))
#define AVI_SCALROT1    (AVI_SCALROT + UL(0x300))

/*********************
 * Scaler sub-modules
 *********************/

#define AVI_SCAL0   (AVI_SCALROT + UL(0x100))   /* Scaler 0 */
#define AVI_SCAL1   (AVI_SCALROT + UL(0x300))   /* Scaler 1 */

/*
 * Per-scaler register definitions
 */
#define AVI_SCAL_CONF                   UL(0x00)
#define AVI_SCAL_CONF_OUTSRC_SHIFT      0
#define AVI_SCAL_CONF_HORIZSRC_SHIFT    2
#define AVI_SCAL_CONF_VERTSRC_SHIFT     4
#define AVI_SCAL_NULL_SRC               0
#define AVI_SCAL_IN_SRC                 1
#define AVI_SCAL_HORIZ_SRC              2
#define AVI_SCAL_VERT_SRC               3
#define AVI_SCAL_CONF_PLANAR_SHIFT      6
#define AVI_SCAL_CONF_OUTFMT_SHIFT      7

#define AVI_SCAL_SIZEOUT                UL(0x04)
#define AVI_SCAL_WIDTH_SHIFT            0
#define AVI_SCAL_HEIGHT_SHIFT           16

#define AVI_SCAL_PLANE0                 UL(0x10)
#define AVI_SCAL_PLANE1                 UL(0x80)

/*
 * Per-plane register definitions (i.e., add plane base adress to offsets below
 * when accessing registers).
 */
#define AVI_SCAL_NTAPS_NPHASES  UL(0x00)
#define AVI_SCAL_NTAPSX_SHIFT   0
#define AVI_SCAL_NTAPSY_SHIFT   2
#define AVI_SCAL_NPHASESX_SHIFT 4
#define AVI_SCAL_NPHASESY_SHIFT 6

#define AVI_SCAL_NTAPS_2        0
#define AVI_SCAL_NTAPS_4        1
#define AVI_SCAL_NTAPS_8        2
#define AVI_SCAL_NTAPS_16       3

#define AVI_SCAL_NPHASES_2      0
#define AVI_SCAL_NPHASES_4      1
#define AVI_SCAL_NPHASES_8      2
#define AVI_SCAL_NPHASES_16     3

#define AVI_SCAL_HORIZ_DIM      UL(0x10)    /* Horizontal dimension */
#define AVI_SCAL_VERT_DIM       UL(0x40)    /* Vertical dimension */

/*
 * Per-dimension coefficients register definitions (i.e., add dimension base
 * address to offsets below when accessing registers.
 */
#define AVI_SCAL_OFFSET         UL(0x00)
#define AVI_SCAL_INCR           UL(0x04)
#define AVI_SCAL_COEFF          UL(0x08)    /* Coefficients base address */
                                            /* within dimension */
#define AVI_SCAL_COEFF0300      (AVI_SCAL_COEFF + UL(0x00))
#define AVI_SCAL_COEFF0704      (AVI_SCAL_COEFF + UL(0x04))
#define AVI_SCAL_COEFF1108      (AVI_SCAL_COEFF + UL(0x08))
#define AVI_SCAL_COEFF1512      (AVI_SCAL_COEFF + UL(0x0c))
#define AVI_SCAL_COEFF1916      (AVI_SCAL_COEFF + UL(0x10))
#define AVI_SCAL_COEFF2320      (AVI_SCAL_COEFF + UL(0x14))
#define AVI_SCAL_COEFF2724      (AVI_SCAL_COEFF + UL(0x18))
#define AVI_SCAL_COEFF3128      (AVI_SCAL_COEFF + UL(0x1c))

/* There's a 64KB RAM shared by both scalers, when only one is in use it can
 * take the entire memory. To simplify things a bit I just assume both scalers
 * are always in use and can only take 32K of RAM. */
#define AVI_SCAL_RAMSZ          (32 * SZ_1K)

/**********************
 * Rotator sub-modules
 **********************/

#define AVI_ROT0   (AVI_SCALROT + UL(0x200))
#define AVI_ROT1   (AVI_SCALROT + UL(0x400))

#define AVI_ROT_ANGLE UL(0x00)

#define AVI_ROT_ANGLE_0		0
#define AVI_ROT_ANGLE_90	1
#define AVI_ROT_ANGLE_270	2
#define AVI_ROT_ANGLE_180	3
#define AVI_ROT_ANGLE_MASK	0x3
#define AVI_ROT_HORIZ_FLIP	(1 << 2)

#define AVI_ROT_SIZE		UL(0x04)
#define AVI_ROT_HEIGHTOUT_SHIFT 16
#define AVI_ROT_HEIGHTOUT_MASK  (0xFFFF)
#define AVI_ROT_NSTRIPE_SHIFT   0
#define AVI_ROT_NSTRIPE_MASK    (0xFFFF)

/**************************
 * Chroma converter module
 **************************/
#define AVI_CONV    UL(0x3000)
#define AVI_CONV0   AVI_CONV
#define AVI_CONV1   (AVI_CONV + UL(0x100))
#define AVI_CONV2   (AVI_CONV + UL(0x200))
#define AVI_CONV3   (AVI_CONV + UL(0x300))

#include "avi_regmap/avi_isp_chroma.h"

/*****************
 * Blender module
 *****************/
#define AVI_BLEND   UL(0x4000)
#define AVI_BLEND0  AVI_BLEND
#define AVI_BLEND1  (AVI_BLEND + UL(0x100))

#define AVI_BLEND_BACKGROUND    UL(0x00)
#define AVI_BLEND_FRAMEOUTSIZE  UL(0x04)
#define AVI_BLEND_OFFSET0       UL(0x08)
#define AVI_BLEND_ALPHA0        UL(0x0C)
#define AVI_BLEND_OFFSET1       UL(0x10)
#define AVI_BLEND_ALPHA1        UL(0x14)
#define AVI_BLEND_OFFSET2       UL(0x18)
#define AVI_BLEND_ALPHA2        UL(0x1C)
#define AVI_BLEND_OFFSET3       UL(0x20)
#define AVI_BLEND_ALPHA3        UL(0x24)

#define AVI_BLEND_NINPUTS       4

/************************
 * LCD controller module
 ************************/
#define AVI_LCD     UL(0x5000)
#define AVI_LCD0    AVI_LCD
#define AVI_LCD1    (AVI_LCD + UL(0x100))

enum avi_pad_select {
	AVI_PAD_0  = 0,
	AVI_PAD_8  = 1,
	AVI_PAD_16 = 2,
};

#define AVI_LCD_FORCE_CLEAR_NORMAL      0 /* Normal clear behaviour */
#define AVI_LCD_FORCE_CLEAR_INACTIVE    1 /* Force clear low */
#define AVI_LCD_FORCE_CLEAR_ACTIVE      2 /* Force clear high */

#include "avi_regmap/avi_lcd.h"

/**************************
 * Camera / Capture module
 **************************/
#define AVI_CAM     UL(0x6000)
#define AVI_CAM0    AVI_CAM
#define AVI_CAM1    (AVI_CAM + UL(0x100))
#define AVI_CAM2    (AVI_CAM + UL(0x200))
#define AVI_CAM3    (AVI_CAM + UL(0x300))
#define AVI_CAM4    (AVI_CAM + UL(0x400))
#define AVI_CAM5    (AVI_CAM + UL(0x500))
#define AVI_CAM6    (AVI_CAM + UL(0x600))
#define AVI_CAM7    (AVI_CAM + UL(0x700))

#define AVI_CAM_STATUS              UL(0x00)
#define AVI_CAM_ITSOURCE            UL(0x04)
#define AVI_CAM_INTERFACE           UL(0x08)
#define AVI_CAM_RUN                 UL(0x0C)
#define AVI_CAM_H_TIMING            UL(0x10)
#define AVI_CAM_V_TIMING            UL(0x20)
#define AVI_CAM_MESURE_H_TIMING0    UL(0x30)
#define AVI_CAM_MESURE_H_TIMING1    UL(0x34)
#define AVI_CAM_MESURE_V_TIMING0    UL(0x40)
#define AVI_CAM_MESURE_V_TIMING1    UL(0x44)
#define AVI_CAM_MESURE_V_TIMING2    UL(0x48)
#define AVI_CAM_MESURE_V_TIMING3    UL(0x4C)

#define AVI_CAP_RAW10_NONE          (0)
#define AVI_CAP_RAW10_1X10          (1)
#define AVI_CAP_RAW10_3X10          (2)

#define AVI_CAP_RAW8_NONE           (0)
#define AVI_CAP_RAW8_1X8            (1)
#define AVI_CAP_RAW8_2X8            (2)
#define AVI_CAP_RAW8_3X8            (3)

/******************************************************
 * Format control definitions. Used by LCDC and CAMIF.
 ******************************************************/
#define AVI_FORMAT_CONTROL_RGB888_1X24	((0 << 2) | 0)
#define AVI_FORMAT_CONTROL_YUV888_1X24	((0 << 2) | 0)
#define AVI_FORMAT_CONTROL_YUYV_1X16	((1 << 2) | 0)
#define AVI_FORMAT_CONTROL_YVYU_1X16	((1 << 2) | 1)
#define AVI_FORMAT_CONTROL_UYVY_2X8	((2 << 2) | 0)
#define AVI_FORMAT_CONTROL_VYUY_2X8	((2 << 2) | 1)
#define AVI_FORMAT_CONTROL_YUYV_2X8	((2 << 2) | 2)
#define AVI_FORMAT_CONTROL_YVYU_2X8	((2 << 2) | 3)
#define AVI_FORMAT_CONTROL_RGB888_3X8	((3 << 2) | 0)
#define AVI_FORMAT_CONTROL_BGR888_3X8	((3 << 2) | 1)
/* X is a dummy byte put at the end of the pixel to pad it to 32 bits */
#define AVI_FORMAT_CONTROL_RGBX8888_4X8	((4 << 2) | 0)
#define AVI_FORMAT_CONTROL_BGRX8888_4X8	((4 << 2) | 1)
#define AVI_FORMAT_CONTROL_RGB565_2X8	((5 << 2) | 0)
#define AVI_FORMAT_CONTROL_BGR565_2X8	((5 << 2) | 1)
/* The padding bit is the MSB of the first byte. So those two formats look like
 * [XRRRRRGG|GGGBBBBB] and [XBBBBBGG|GGGRRRRR] respectively */
#define AVI_FORMAT_CONTROL_RGB555_2X8	((6 << 2) | 0)
#define AVI_FORMAT_CONTROL_BGR555_2X8	((6 << 2) | 1)
/* The padding nibble are the MSB of the first byte. So those two formats look
 * like [XXXXRRRR|GGGGBBBB] and [XXXXBBBB|GGGGRRRR] respectively */
#define AVI_FORMAT_CONTROL_RGB444_2X8	((7 << 2) | 0)
#define AVI_FORMAT_CONTROL_BGR444_2X8	((7 << 2) | 1)

/*************************
 * Gamma corrector module
 *************************/
#define AVI_GAM     UL(0x8000)
#define AVI_GAM0    AVI_GAM
#define AVI_GAM1    (AVI_GAM + 0x4000)

#include "avi_regmap/avi_isp_gamma_corrector.h"

/**
 * Make sure the C structure representation is coherent with the AVI registers
 */
#define AVI_ASSERT_REGISTERS_SIZE(_struct, _nregs)                            \
        static inline void __avi_check_##_struct##_size(void)                 \
        {                                                                     \
              BUILD_BUG_ON(sizeof(struct _struct) != (_nregs) * sizeof(u32)); \
        }

union avi_interconnect_registers
{
	union
	{
		struct
		{
			u8 fifo00_src;
			u8 fifo01_src;
			u8 fifo02_src;
			u8 fifo03_src;
		};
		u32 srcsel0;
	};
	union
	{
		struct
		{
			u8 fifo04_src;
			u8 fifo05_src;
			u8 fifo06_src;
			u8 fifo07_src;
		};
		u32 srcsel1;
	};
	union
	{
		struct
		{
			u8 fifo08_src;
			u8 fifo09_src;
			u8 fifo10_src;
			u8 fifo11_src;
		};
		u32 srcsel2;
	};
	union
	{
		struct
		{
			u8 conv0_src;
			u8 conv1_src;
			u8 conv2_src;
			u8 conv3_src;
		};
		u32 srcsel3;
	};
	union
	{
		struct
		{
			u8 blend0_src0;
			u8 blend0_src1;
			u8 blend0_src2;
			u8 blend0_src3;
		};
		u32 srcsel4;
	};
	union
	{
		struct
		{
			u8 blend1_src0;
			u8 blend1_src1;
			u8 blend1_src2;
			u8 blend1_src3;
		};
		u32 srcsel5;
	};
	union
	{
		struct
		{
			u8 gam0_src;
			u8 gam1_src;
			u8 scal0_src0;
			u8 scal0_src1;
		};
		u32 srcsel6;
	};
	union
	{
		struct
		{
			u8 rot0_src;
			u8 scal1_src0;
			u8 scal1_src1;
			u8 rot1_src;
		};
		u32 srcsel7;
	};
	u32 srcsel[8];
};

/**
 * AVI_FIFO_CFG
 */
union avi_fifo_cfg
{
        struct
        {
                unsigned srctype     : 1;
                unsigned srcorg      : 1;
                unsigned srcformat   : 2;
                unsigned srcbitenc   : 4;
                unsigned dsttype     : 1;
                unsigned dstorg      : 1;
                unsigned dstformat   : 2;
                unsigned dstbitenc   : 4;
                unsigned reorg       : 2;
                unsigned dithering   : 2;
                unsigned cdsdis      : 1;
                unsigned cusdis      : 1;
                unsigned phase       : 1;
                unsigned scalplane   : 1;
                unsigned single      : 1;/* P7 Release >=2 */
                unsigned keying      : 1;
                unsigned itsource    : 2;
                unsigned itline      : 4;
        };
        u32 _register;
};

/**
 * AVI_FIFO_FRAMESIZE
 */
union avi_fifo_framesize
{
        struct
        {
                u16 width;
                u16 height;
        };
        u32 _register;
};

/**
 * AVI_FIFO_MONITOR
 */
union avi_fifo_monitor
{
        struct
        {
                u16 lvlstarting;
                u16 lvlwarning;
        };
        u32 _register;
};

/**
 * AVI_FIFO_TIMEOUT
 */
union avi_fifo_timeout
{
        struct
        {
                u8       vblank;
                u8       hblank;
                u8       pincr;
                unsigned issuing_capability : 3;
                unsigned burst              : 2;
                unsigned /* unused */       : 3;
        };
        u32 _register;
};

/**
 * AVI_FIFO_SWAP{0,1}
 */
union avi_fifo_swap
{
        struct
        {
                unsigned sel0        : 3;
                unsigned /* undef */ : 1;
                unsigned sel1        : 3;
                unsigned /* undef */ : 1;
                unsigned sel2        : 3;
                unsigned /* undef */ : 1;
                unsigned sel3        : 3;
                unsigned /* undef */ : 1;
                unsigned sel4        : 3;
                unsigned /* undef */ : 1;
                unsigned sel5        : 3;
                unsigned /* undef */ : 1;
                unsigned sel6        : 3;
                unsigned /* undef */ : 1;
                unsigned sel7        : 3;
                unsigned /* undef */ : 1;
        };
        u32 _register;
};

/**
 * This macro can be used to set avi_fifo_swap._register when doing colour
 * swapping. Output format is ARGB, so if you want to convert RGBA you can do:
 * AVI_FIFO_SWAP(3, 0, 1, 2) (alpha is the 4th byte, red the first etc...)
 */
#define AVI_FIFO_MAKE_SWAP16(_a, _b, _c, _d) (_a << 12 | _b << 8 | _c << 4 | _d)

#define AVI_FIFO_MAKE_SWAP(_a, _b, _c, _d)			   \
	(((AVI_FIFO_MAKE_SWAP16(_a, _b, _c, _d) | 0x4444) << 16) | \
	 AVI_FIFO_MAKE_SWAP16(_a, _b, _c, _d))

/* This configures a NO-OP swap */
#define AVI_FIFO_NULL_SWAP AVI_FIFO_MAKE_SWAP(3, 2, 1, 0)

/**
 * AVI FIFO DMA FRAME XY configuration
 */
union avi_fifo_dmafxycfg
{
        struct
        {
                unsigned dmafxnb      : 12;
                unsigned /* unused */ : 4;
                unsigned dmafynb      : 12;
                unsigned /* unused */ : 3;
                unsigned dmafxymode   : 1;
        };
        u32 _register;
};

union avi_fifo_keying
{
        struct
        {
                u8 bv;
                u8 gu;
                u8 ry;
        };
        u32 _register;
};

/**
 * AVI FIFO Video Link In control
 * Available in P7 Release >=3
 */
union avi_fifo_vlin
{
	struct
	{
		unsigned force_clear  : 2;
		unsigned /* unused */ : 30;
	};
	u32 _register;
};
/*
 * force_clear field
 * possible values
 */
enum avi_fifo_vlin_force_clear {
	/* no force clear signal */
	AVI_FIFO_VLIN_FORCE_CLEAR_NORMAL=0,
	/* force clear signal to inactive level */
	AVI_FIFO_VLIN_FORCE_CLEAR_INACTIVE=1,
	/* force clear signal to active level */
	AVI_FIFO_VLIN_FORCE_CLEAR_ACTIVE=2
};

/* Available in P7 Release >=3 */
union avi_fifo_status
{
	struct
	{
		unsigned it_all:1;
		unsigned it_nl0:1;
		unsigned it_nl1:1;
		unsigned :13;
		unsigned it_monitor0:1;
		unsigned it_monitor1:1;
		unsigned :6;
		unsigned running:1;
		unsigned :7;
	};
	u32 _register;
};

struct avi_fifo_registers
{
        union avi_fifo_cfg       cfg;
        union avi_fifo_framesize framesize;
        union avi_fifo_monitor   monitor;
        union avi_fifo_timeout   timeout;
        union avi_fifo_swap      swap0;
        union avi_fifo_swap      swap1;
        unsigned                 /* unused */ : 32;
        unsigned                 /* unused */ : 32;
        u32                      dmasa0;
        u32                      dmasa1;
        s32                      dmalstr0;
        s32                      dmalstr1;
        union avi_fifo_dmafxycfg dmafxycfg;
        s32                      dmafxstr0;
        s32                      dmafxstr1;
        s32                      dmafystr0;
        s32                      dmafystr1;
        unsigned                 /* unused */ : 32;
        unsigned                 /* unused */ : 32;
        unsigned                 /* unused */ : 32;
        union avi_fifo_keying    keyingmin;
        union avi_fifo_keying    keyingmax;
	unsigned                 keyingalpha  : 8;
	unsigned                 /* unused */ : 24;
	unsigned                 /* unused */ : 32;
	union avi_fifo_vlin      vlin;    /* Available in P7 Release >=3 */
	union avi_fifo_status    status;  /* Available in P7 Release >=3 */
};
AVI_ASSERT_REGISTERS_SIZE(avi_fifo_registers, AVI_FIFO_REG_COUNT);

union avi_scal_coeff {
	struct {
		unsigned    coeff00:8;
		unsigned    coeff01:8;
		unsigned    coeff02:8;
		unsigned    coeff03:8;

		unsigned    coeff04:8;
		unsigned    coeff05:8;
		unsigned    coeff06:8;
		unsigned    coeff07:8;

		unsigned    coeff08:8;
		unsigned    coeff09:8;
		unsigned    coeff10:8;
		unsigned    coeff11:8;

		unsigned    coeff12:8;
		unsigned    coeff13:8;
		unsigned    coeff14:8;
		unsigned    coeff15:8;

		unsigned    coeff16:8;
		unsigned    coeff17:8;
		unsigned    coeff18:8;
		unsigned    coeff19:8;

		unsigned    coeff20:8;
		unsigned    coeff21:8;
		unsigned    coeff22:8;
		unsigned    coeff23:8;

		unsigned    coeff24:8;
		unsigned    coeff25:8;
		unsigned    coeff26:8;
		unsigned    coeff27:8;

		unsigned    coeff28:8;
		unsigned    coeff29:8;
		unsigned    coeff30:8;
		unsigned    coeff31:8;
	};
	struct {
		u32         coeff0300;
		u32         coeff0704;
		u32         coeff1108;
		u32         coeff1512;
		u32         coeff1916;
		u32         coeff2320;
		u32         coeff2724;
		u32         coeff3128;
	} words;
	u8 bytes[32];
};

union avi_cam_status
{
        struct
        {
                unsigned done  : 1;
                unsigned fof   : 1;
                unsigned field : 1;
        };
        u32 _register;
};

union avi_cam_itsource
{
        struct
        {
                unsigned done_en  : 1;
                unsigned fof_en   : 1;
        };
        u32 _register;
};

union avi_cam_interface
{
        struct
        {
                unsigned ivs            : 1;
                unsigned ihs            : 1;
                unsigned ipc            : 1;
                unsigned ioe            : 1;
                unsigned psync_rf       : 1;
                unsigned psync_en       : 1;
                unsigned itu656         : 1;
                unsigned timing_auto    : 1;
                unsigned format_control : 5;
                unsigned pad_select     : 2;
                unsigned unpacker       : 2;
                unsigned raw10          : 2;
                unsigned rol_lsb        : 1;
                unsigned ror_lsb        : 1;
        };
        u32 _register;
};

union avi_cam_run
{
        struct
        {
                unsigned run      : 1;
                unsigned free_run : 1;
        };
        u32 _register;
};

union avi_cam_h_timing
{
        struct
        {
                u16 hactive_on;
                u16 hactive_off;
        };
        u32 _register;
};

union avi_cam_v_timing
{
        struct
        {
                u16 vactive_on;
                u16 vactive_off;
        };
        u32 _register;
};

struct avi_cam_registers
{
        union avi_cam_status    status;
        union avi_cam_itsource  itsource;
        union avi_cam_interface interface;
        union avi_cam_run       run;
        union avi_cam_h_timing  h_timing;
        unsigned /* unused */ : 32;
        unsigned /* unused */ : 32;
        unsigned /* unused */ : 32;
        union avi_cam_v_timing  v_timing;
};
AVI_ASSERT_REGISTERS_SIZE(avi_cam_registers, 0x24 / 4)

/* Those are the read-only timing registers of the cam interface */
struct avi_cam_measure_regs
{
        /* MESURE_H_TIMING0 */
        u16 hsync_off;
        u16 hactive_on;
        /* MESURE_H_TIMING1 */
        u16 hactive_off;
        u16 htotal;
        unsigned /* unused */ : 32;
        unsigned /* unused */ : 32;
        /* MESURE_V_TIMING0 */
        u16 vsync_hon;
        u16 vsync_von;
        /* MESURE_V_TIMING1 */
        u16 vsync_hoff;
        u16 vsync_voff;
        /* MESURE_V_TIMING2 */
        u16 vactive_on;
        u16 vactive_off;
        /* MESURE_V_TIMING3 */
        u16 vtotal;
        unsigned /* unused */ : 16;
};
AVI_ASSERT_REGISTERS_SIZE(avi_cam_measure_regs, (0x50 - 0x30) / 4)

/**************
 * ISP modules
 **************/
/* ISP Chain Bayer */
#define AVI_ISP_CHAIN_BAYER    UL(0x20000)
#define AVI_ISP_CHAIN_BAYER0   AVI_ISP_CHAIN_BAYER
#define AVI_ISP_CHAIN_BAYER1  (AVI_ISP_CHAIN_BAYER + UL(0x10000))

#define AVI_ISP_CHAIN_BAYER_INTER       UL(0x0000)
#define AVI_ISP_VL_FORMAT_32_TO_40      UL(0x1000)
#define AVI_ISP_PEDESTAL                UL(0x2000)
#define AVI_ISP_GREEN_IMBALANCE         UL(0x4000)
#define AVI_ISP_DEAD_PIXEL_CORRECTION   UL(0x6000)
#define AVI_ISP_DENOISING               UL(0x7000)
#define AVI_ISP_STATS_BAYER             UL(0x8000)
#define AVI_ISP_LENS_SHADING_CORRECTION UL(0xA000)
#define AVI_ISP_CHROMATIC_ABERRATION    UL(0xC000)
#define AVI_ISP_BAYER                   UL(0xD000)
#define AVI_ISP_COLOR_CORRECTION        UL(0xE000)
#define AVI_ISP_VL_FORMAT_40_T0_32      UL(0xF000)

#include "avi_regmap/avi_isp_chain_bayer_inter.h"
#include "avi_regmap/avi_isp_vlformat_32to40.h"
#include "avi_regmap/avi_isp_pedestal.h"
#include "avi_regmap/avi_isp_green_imbalance.h"
#include "avi_regmap/avi_isp_dead_pixel_correction.h"
#include "avi_regmap/avi_isp_denoising.h"
#include "avi_regmap/avi_isp_statistics_bayer.h"
#include "avi_regmap/avi_isp_lens_shading_correction.h"
#include "avi_regmap/avi_isp_chromatic_aberration.h"
#include "avi_regmap/avi_isp_bayer.h"
#include "avi_regmap/avi_isp_color_correction.h"
#include "avi_regmap/avi_isp_vlformat_40to32.h"

#define AVI_STATS_YUV	 UL(0x12000)
#define AVI_STATS_YUV0	 AVI_STATS_YUV
#define AVI_STATS_YUV1	(AVI_STATS_YUV + UL(0x1000))

/* ISP Chain YUV */
#define AVI_ISP_CHAIN_YUV    UL(0x40000)
#define AVI_ISP_CHAIN_YUV0   AVI_ISP_CHAIN_YUV
#define AVI_ISP_CHAIN_YUV1  (AVI_ISP_CHAIN_YUV + UL(0x10000))

#define AVI_ISP_CHAIN_YUV_INTER       UL(0x0000)
#define AVI_ISP_EDGE_ENHANCEMENT      UL(0x1000)
#define AVI_ISP_I3D_LUT               UL(0x2000)
#define AVI_ISP_DROP                  UL(0x3000)

#include "avi_regmap/avi_isp_chain_yuv_inter.h"
#include "avi_regmap/avi_isp_statistics_yuv.h"
#include "avi_regmap/avi_isp_edge_enhancement_color_reduction_filter.h"
#include "avi_regmap/avi_isp_i3d_lut.h"
#include "avi_regmap/avi_isp_drop.h"

#endif /* _REG_AVI_H */
