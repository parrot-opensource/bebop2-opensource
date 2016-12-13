/*
 *  linux/drivers/parrot/video/media/avi_v4l2_isp.c
 *
 *  Copyright (C) 2015 Parrot S.A.
 *
 * @author  Alexandre Dilly <alexandre.dilly@parrot.com>
 * @date  01-Jan-2015
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
#include <linux/slab.h>

#include "../../video/avi_compat.h"
#include "../../video/reg_avi.h"

#include "avi_v4l2_isp_ctrls.h"
#include "avi_v4l2_isp.h"

/** Boolean control types
 * @id:		Control ID
 * @name:	Name of control (and description)
 * @addr:	Offest of assocaited register
 * @len:	Length of regsiter field (in bits)
 * @shift:	Position of filed in register
 * @read:	Read register before writing
 * @flags:	Flags (RO, WO, ...)
 */
#define _BOOLEAN_CTRL(_id, _name, _addr, _len, _shift, _read, _flags) \
{ \
	.cfg = { \
		.ops = &avi_v4l2_isp_ops, \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.id = _id, \
		.name = _name, \
		.def = 0, \
		.min = 0, \
		.max = 1, \
		.step = 1, \
		.flags = _flags, \
	}, \
	.reg = { \
		.addr = _addr, \
		.count = 1, \
		.mask = ((1L << _len)-1) << _shift, \
		.shift = _shift, \
		.read = _read, \
	}, \
},
#define BOOLEAN_CTRL(_id, _name, _addr, _len, _shift, _read) \
	_BOOLEAN_CTRL(_id, _name, _addr, _len, _shift, _read, 0)

/** Array control types
 * @id:		Control ID
 * @name:	Name of control (and description)
 * @min:	Minimal value for control
 * @max:	Maximal value for control
 * @def:	Default value for control
 * @size:	Size of array
 * @addr:	Offest of assocaited register
 * @len:	Length of regsiter field (in bits)
 * @shift:	Position of filed in register
 * @read:	Read register before writing
 * @flags:	Flags (RO, WO, ...)
 */
#define _ARRAY_CTRL(_type, _id, _name, _min, _max, _def, _size, _addr, _len, \
		    _shift, _read, _flags) \
{ \
	.cfg = { \
		.ops = &avi_v4l2_isp_ops, \
		.type = _type, \
		.id = _id, \
		.name = _name, \
		.def = _def, \
		.min = _min, \
		.max = _max, \
		.step = 1, \
		.flags = _flags, \
		.dims = { _size } \
	}, \
	.reg = { \
		.addr = _addr, \
		.count = _size, \
		.mask = ((1L << _len)-1) << _shift, \
		.shift = _shift, \
		.read = _read, \
	}, \
},
#define _ARRAY8_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, _shift, \
		    _read, _flags) \
	_ARRAY_CTRL(V4L2_CTRL_TYPE_U8, _id, _name, _min, _max, _def, _size, \
		   _addr, _len, _shift, _read, _flags)
#define ARRAY8_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, _shift, \
		    _read) \
	_ARRAY8_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, _shift, \
		     _read, 0)

#define _ARRAY16_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, \
		      _shift, _read, _flags) \
	_ARRAY_CTRL(V4L2_CTRL_TYPE_U16, _id, _name, _min, _max, _def, _size, \
		    _addr, _len, _shift, _read, _flags)
#define ARRAY16_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, _shift, \
		     _read) \
	_ARRAY16_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, \
		      _shift, _read, 0)

#define _ARRAY32_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, \
		      _shift, _read, _flags) \
	_ARRAY_CTRL(V4L2_CTRL_TYPE_INTEGER, _id, _name, _min, _max, _def, \
		    _size, _addr, _len, _shift, _read, _flags)
#define ARRAY32_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, _shift, \
		     _read) \
	_ARRAY32_CTRL(_id, _name, _min, _max, _def, _size, _addr, _len, \
		      _shift, _read, 0)

/** Basic control types
 * @id:		Control ID
 * @name:	Name of control (and description)
 * @min:	Minimal value for control
 * @max:	Maximal value for control
 * @def:	Default value for control
 * @addr:	Offest of assocaited register
 * @len:	Length of regsiter field (in bits)
 * @shift:	Position of filed in register
 * @read:	Read register before writing
 * @flags:	Flags (RO, WO, ...)
 */
#define _U32_CTRL(_id, _name, _min, _max, _def, _addr, _len, _shift, _read, \
		  _flags) \
{ \
	.cfg = { \
		.ops = &avi_v4l2_isp_ops, \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.id = _id, \
		.name = _name, \
		.def = _def, \
		.min = _min, \
		.max = _max, \
		.step = 1, \
		.flags = _flags, \
	}, \
	.reg = { \
		.addr = _addr, \
		.count = 1, \
		.mask = ((1L << _len)-1) << _shift, \
		.shift = _shift, \
		.read = _read, \
	}, \
},
#define U32_CTRL(_id, _name, _min, _max, _def, _addr, _len, _shift, _read) \
		_U32_CTRL(_id, _name, _min, _max, _def, _addr, _len, _shift, \
			  _read, 0)
#define _U16_CTRL _U32_CTRL
#define U16_CTRL U32_CTRL
#define _U8_CTRL _U32_CTRL
#define U8_CTRL U32_CTRL

static int avi_v4l2_isp_g_volatile_ctrl(struct v4l2_ctrl *ctrl);
static int avi_v4l2_isp_s_ctrl(struct v4l2_ctrl *ctrl);

struct v4l2_ctrl_ops avi_v4l2_isp_ops = {
	.g_volatile_ctrl = avi_v4l2_isp_g_volatile_ctrl,
	.s_ctrl = avi_v4l2_isp_s_ctrl,
};

struct avi_v4l2_isp_ctrl_config {
	struct v4l2_ctrl_config cfg;
	struct avi_v4l2_isp_reg {
		unsigned long addr;
		unsigned long count;
		u32 mask;
		u8 shift;
		u8 read;
	} reg;
};

static const struct avi_v4l2_isp_ctrl_config isp_bayer_ctrls[] = {
	/* ISP_BYPASS */
	BOOLEAN_CTRL(P7_CID_BYPASS_PEDESTAL, "Bypass Pedestal ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 0, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_GRIM, "Bypass GRIM ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 1, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_RIP, "Bypass RIP ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 2, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_DENOISE, "Bypass Denoise ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 3, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_LSC, "Bypass LSC ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 4, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_CHROMA_ABER,
		     "Bypass Chroma Aberation ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 5, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_BAYER, "Bypass Bayer ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 6, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_COLOR_MATRIX,
		     "Bypass Color Matrix ISP Module",
		     AVI_ISP_CHAIN_BAYER_INTER, 1, 7, 1)
	/* ISP_VLF_32TO40 */
	U8_CTRL(P7_CID_ISP_VLF_32_TO_40, "VL Format 32 to 40", 0, 4, 0,
		AVI_ISP_VL_FORMAT_32_TO_40 + AVI_ISP_VLFORMAT_32TO40_FORMAT,
		3, 0, 0)
	/* ISP_PEDESTAL */
	U16_CTRL(P7_CID_ISP_PEDESTAL_SUB_R,
		 "Value subtracted from red bayer pixels",
		 0, 1023, 0,
		 AVI_ISP_PEDESTAL + AVI_ISP_PEDESTAL_SUB_R, 10, 0, 0)
	U16_CTRL(P7_CID_ISP_PEDESTAL_SUB_GB,
		 "Value subtracted from green bayer pixels on blue row",
		 0, 1023, 0,
		 AVI_ISP_PEDESTAL + AVI_ISP_PEDESTAL_SUB_GB, 10, 0, 0)
	U16_CTRL(P7_CID_ISP_PEDESTAL_SUB_GR,
		 "Value subtracted from green bayer pixels on red row",
		 0, 1023, 0,
		 AVI_ISP_PEDESTAL + AVI_ISP_PEDESTAL_SUB_GR, 10, 0, 0)
	U16_CTRL(P7_CID_ISP_PEDESTAL_SUB_B,
		 "Value subtracted from blue bayer pixels",
		 0, 1023, 0,
		 AVI_ISP_PEDESTAL + AVI_ISP_PEDESTAL_SUB_B, 10, 0, 0)
	U8_CTRL(P7_CID_ISP_PEDESTAL_CFA,
		"Color Filter Array of Pedestal",
		0, 3, 0,
		AVI_ISP_PEDESTAL + AVI_ISP_PEDESTAL_CFA, 2, 0, 0)
	/* ISP_GRIM */
	U16_CTRL(P7_CID_ISP_GRIM_OFFSET_X,
		 "Offset of top left pixel from top left vertice of cell",
		 0, 511, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_OFFSET_X_Y, 9, 0, 1)
	U16_CTRL(P7_CID_ISP_GRIM_OFFSET_Y,
		 "Offset of top left pixel from top left vertice of cell",
		 0, 1023, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_OFFSET_X_Y, 10, 16, 1)
	U8_CTRL(P7_CID_ISP_GRIM_CELL_ID_X,
		 "Horizontal cell index to which top left pixel belongs",
		 0, 15, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_CELL_ID_X_Y, 4, 0, 1)
	U8_CTRL(P7_CID_ISP_GRIM_CELL_ID_Y,
		 "Vertical cell index to which top left pixel belongs",
		 0, 15, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_CELL_ID_X_Y, 4, 16, 1)
	U16_CTRL(P7_CID_ISP_GRIM_CELL_W,
		 "Mesh cells width in pixel", 0, 511, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_CELL_W, 9, 0, 0)
	U16_CTRL(P7_CID_ISP_GRIM_CELL_H,
		 "Mesh cells height in pixel", 0, 1023, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_CELL_H, 10, 0, 0)
	U32_CTRL(P7_CID_ISP_GRIM_CELL_W_INV,
		 "Mesh cells inverse width", 0, 131071, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_CELL_W_INV, 17, 0, 0)
	U32_CTRL(P7_CID_ISP_GRIM_CELL_H_INV,
		 "Mesh cells inverse height", 0, 131071, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_CELL_H_INV, 17, 0, 0)
	U32_CTRL(P7_CID_ISP_GRIM_ALPHA,
		 "Normalised horizontal offset of top left pixel from top left "
		 "vertice of cell",
		 0, 131071, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_ALPHA, 17, 0, 0)
	U32_CTRL(P7_CID_ISP_GRIM_BETA,
		 "Normalised vertical offset of top left pixel from top left "
		 "vertice of cell",
		 0, 131071, 0,
		 AVI_ISP_GREEN_IMBALANCE +
		 AVI_ISP_GREEN_IMBALANCE_BETA, 17, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_GRIM_GR_COEFF,
		    "Red coefficients: Correction mesh for R channel",
		    0, 255, 0, 221,
		    AVI_ISP_GREEN_IMBALANCE +
		    AVI_ISP_GREEN_IMBALANCE_GREEN_RED_COEFF_MEM, 8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_GRIM_GB_COEFF,
		    "Blue coefficients: Correction mesh for B channel",
		    0, 255, 0, 221,
		    AVI_ISP_GREEN_IMBALANCE +
		    AVI_ISP_GREEN_IMBALANCE_GREEN_BLUE_COEFF_MEM, 8, 0, 0)
	U8_CTRL(P7_CID_ISP_GRIM_CFA,
		"Color Filter Array of Green Imbalance",
		0, 3, 0,
		AVI_ISP_GREEN_IMBALANCE +
		AVI_ISP_GREEN_IMBALANCE_BAYER_CFA, 2, 0, 0)
	/* ISP_RIP */
	ARRAY16_CTRL(P7_CID_ISP_RIP_LIST_MEM_X, "Abscissa of known dead pixel",
		     0, 2047, 0, 256,
		     AVI_ISP_DEAD_PIXEL_CORRECTION +
		     AVI_ISP_DEAD_PIXEL_CORRECTION_LIST_MEM, 11, 3, 1)
	ARRAY16_CTRL(P7_CID_ISP_RIP_LIST_MEM_Y, "Ordinate of known dead pixel",
		     0, 4095, 0, 256,
		     AVI_ISP_DEAD_PIXEL_CORRECTION +
		     AVI_ISP_DEAD_PIXEL_CORRECTION_LIST_MEM, 12, 14, 1)
	ARRAY8_CTRL(P7_CID_ISP_RIP_LIST_MEM_OP,
		    "List of suitable corrections for each corresponding dead "
		    "pixel position",
		    0, 7, 0, 256,
		    AVI_ISP_DEAD_PIXEL_CORRECTION +
		    AVI_ISP_DEAD_PIXEL_CORRECTION_LIST_MEM, 3, 0, 1)
	BOOLEAN_CTRL(P7_CID_ISP_RIP_BYPASS_LIST, "Deactivates list mode",
		     AVI_ISP_DEAD_PIXEL_CORRECTION +
		     AVI_ISP_DEAD_PIXEL_CORRECTION_BYPASS, 1, 0, 1)
	BOOLEAN_CTRL(P7_CID_ISP_RIP_BYPASS_AUTO_DETECTION,
		     "Deactivates the live mode",
		     AVI_ISP_DEAD_PIXEL_CORRECTION +
		     AVI_ISP_DEAD_PIXEL_CORRECTION_BYPASS, 1, 1, 1)
	BOOLEAN_CTRL(P7_CID_ISP_RIP_BYPASS_RGRIM,
		     "Deactivates the RGRIM filter list mode, which in turn "
		     "takes precedence over the RGRIM filter",
		     AVI_ISP_DEAD_PIXEL_CORRECTION +
		     AVI_ISP_DEAD_PIXEL_CORRECTION_BYPASS, 1, 2, 1)
	U16_CTRL(P7_CID_ISP_RIP_THRESHOLD,
		 "Used in Live mode to compare one pixel to its neighbours",
		 0, 1023, 0,
		 AVI_ISP_DEAD_PIXEL_CORRECTION +
		 AVI_ISP_DEAD_PIXEL_CORRECTION_THRESHOLD, 10, 0, 0)
	U8_CTRL(P7_CID_ISP_RIP_RGRIM_CONF_TIM_1, "Imbalance 1", 0, 255, 0,
		AVI_ISP_DEAD_PIXEL_CORRECTION +
		AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_CONF, 8, 0, 1)
	U8_CTRL(P7_CID_ISP_RIP_RGRIM_CONF_TIM_2, "Imbalance 2", 0, 255, 0,
		AVI_ISP_DEAD_PIXEL_CORRECTION +
		AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_CONF, 8, 8, 1)
	U8_CTRL(P7_CID_ISP_RIP_RGRIM_CONF_TSAT, "Saturation", 0, 255, 0,
		AVI_ISP_DEAD_PIXEL_CORRECTION +
		AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_CONF, 8, 16, 1)
	U8_CTRL(P7_CID_ISP_RIP_RGRIM_CONF_TCON, "Contrast", 0, 255, 0,
		AVI_ISP_DEAD_PIXEL_CORRECTION +
		AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_CONF, 8, 24, 1)
	U8_CTRL(P7_CID_ISP_RIP_RGRIM_GAIN,
		"The gain obeys the formula : actual gain = 20 * gain / 4096",
		0, 127, 0,
		AVI_ISP_DEAD_PIXEL_CORRECTION +
		AVI_ISP_DEAD_PIXEL_CORRECTION_RGRIM_GAIN, 7, 0, 0)
	U8_CTRL(P7_CID_ISP_RIP_CFA,
		"Color Filter Array of Dead Pixel Correction",
		0, 3, 0,
		AVI_ISP_DEAD_PIXEL_CORRECTION +
		AVI_ISP_DEAD_PIXEL_CORRECTION_CFA, 2, 0, 0)
	/* ISP_DENOISE */
	ARRAY8_CTRL(P7_CID_ISP_DENOISE_LUMOCOEFF_RED,
		    "Ordinates of lumo coeffs Red", 0, 255, 0, 14,
		    AVI_ISP_DENOISING + AVI_ISP_DENOISING_LUMOCOEFF_R_03_00,
		    8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_DENOISE_LUMOCOEFF_GREEN,
		    "Ordinates of lumo coeffs Green", 0, 255, 0, 14,
		    AVI_ISP_DENOISING + AVI_ISP_DENOISING_LUMOCOEFF_G_03_00,
		    8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_DENOISE_LUMOCOEFF_BLUE,
		    "Ordinates of lumo coeffs Blue", 0, 255, 0, 14,
		    AVI_ISP_DENOISING + AVI_ISP_DENOISING_LUMOCOEFF_B_03_00,
		    8, 0, 0)
	U8_CTRL(P7_CID_ISP_DENOISE_CFA,
		"Color Filter Array of Denoising",
		0, 3, 0,
		AVI_ISP_DENOISING + AVI_ISP_DENOISING_CFA, 2, 0, 0)
	/* ISP_BAYER_STAT */
	_BOOLEAN_CTRL(P7_CID_ISP_BSTAT_MEASURE_REQ_CLEAR,
		      "Force clear the histogram, usually done the first time "
		      "before enabling the stats bayer module",
		      AVI_ISP_STATS_BAYER +
		      AVI_ISP_STATISTICS_BAYER_MEASURE_REQ, 1, 0, 0,
		      V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_X_WIDTH,
		  "Width of a thumbnail pixel in sensor pixels (how many "
		  "columns are averaged into one)",
		  0, 2047, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_WINDOW_X, 11, 16, 1,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_X_OFFSET,
		  "Origin abcissa of the statistics window in the captured "
		  "image",
		  0, 8191, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_WINDOW_X, 13, 0, 1,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_Y_HEIGHT,
		  "Height of a thumbnail pixel in sensor pixels (how many rows "
		  "are averaged into one).",
		  0, 2047, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_WINDOW_Y, 11, 16, 1,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_Y_OFFSET,
		  "Origin abcissa of the statistics window in the captured "
		  "image",
		  0, 8191, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_WINDOW_Y, 13, 0, 1,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_CIRCLE_POS_X_CENTER,
		  "Center abcissa of the image circle in sensor pixels",
		  0, 16383, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_X_CENTER, 14, 0, 0,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U32_CTRL(P7_CID_ISP_BSTAT_CIRCLE_POS_X_SQUARED,
		  "Center abcissa of the image circle in sensor pixels value "
		  "squared",
		  0, 67108863, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_X_SQUARED, 26, 0, 0,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_CIRCLE_POS_Y_CENTER,
		  "Center ordinate of the image circle in sensor pixels",
		  0, 16383, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_Y_CENTER, 14, 0, 0,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U32_CTRL(P7_CID_ISP_BSTAT_CIRCLE_POS_Y_SQUARED,
		  "Center ordinate of the image circle in sensor pixels value "
		  "squared",
		  0, 67108863, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_CIRCLE_POS_Y_SQUARED, 26, 0, 0,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U32_CTRL(P7_CID_ISP_BSTAT_CIRCLE_RADIUS_SQUARED,
		  "Squared radius of the image circle", 0, 536870911, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_CIRCLE_RADIUS_SQUARED, 29, 0, 0,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U8_CTRL(P7_CID_ISP_BSTAT_INC_X_LOG2_INC,
		 "Vertical skipping/binning parameter", 0, 7, 0,
		 AVI_ISP_STATS_BAYER +
		 AVI_ISP_STATISTICS_BAYER_INCREMENTS_LOG2, 3, 0, 1,
		 V4L2_CTRL_FLAG_VOLATILE)
	_U8_CTRL(P7_CID_ISP_BSTAT_INC_Y_LOG2_INC,
		 "Horizontal skipping/binning parameter", 0, 7, 0,
		 AVI_ISP_STATS_BAYER +
		 AVI_ISP_STATISTICS_BAYER_INCREMENTS_LOG2, 3, 16, 1,
		 V4L2_CTRL_FLAG_VOLATILE)
	_U16_CTRL(P7_CID_ISP_BSTAT_SAT_THRESHOLD,
		  "Below this threshold a pixel is considered unsaturated, a "
		  "Bayer block is saturated if any of its 4 pixels is saturated",
		  0, 1023, 0,
		  AVI_ISP_STATS_BAYER +
		  AVI_ISP_STATISTICS_BAYER_SAT_THRESHOLD, 10, 0, 0,
		  V4L2_CTRL_FLAG_VOLATILE)
	_U8_CTRL(P7_CID_ISP_BSTAT_MAX_X_WINDOW_COUNT,
		 "Width of thumbnail", 0, 127, 0,
		 AVI_ISP_STATS_BAYER +
		 AVI_ISP_STATISTICS_BAYER_MAX_NB_WINDOWS, 7, 0, 1,
		 V4L2_CTRL_FLAG_VOLATILE)
	_U8_CTRL(P7_CID_ISP_BSTAT_MAX_Y_WINDOW_COUNT,
		 "Height of thumbnail", 0, 127, 0,
		 AVI_ISP_STATS_BAYER +
		 AVI_ISP_STATISTICS_BAYER_MAX_NB_WINDOWS, 7, 16, 1,
		 V4L2_CTRL_FLAG_VOLATILE)
	_U8_CTRL(P7_CID_ISP_BSTAT_CFA,
		 "Color Filter Array of Statistics bayer",
		 0, 3, 0,
		 AVI_ISP_STATS_BAYER + AVI_ISP_STATISTICS_BAYER_CFA, 2, 0, 0,
		 V4L2_CTRL_FLAG_VOLATILE)
	/* ISP_LSC */
	U16_CTRL(P7_CID_ISP_LSC_OFFSET_X,
		 "Offset of top left pixel from top left vertice of cell",
		 0, 511, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_OFFSET_X_Y, 9, 0, 1)
	U16_CTRL(P7_CID_ISP_LSC_OFFSET_Y,
		 "Offset of top left pixel from top left vertice of cell",
		 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_OFFSET_X_Y, 10, 16, 1)
	U8_CTRL(P7_CID_ISP_LSC_CELL_ID_X,
		 "Horizontal cell index to which top left pixel belongs",
		 0, 15, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_CELL_ID_X_Y, 4, 0, 1)
	U8_CTRL(P7_CID_ISP_LSC_CELL_ID_Y,
		 "Vertical cell index to which top left pixel belongs",
		 0, 15, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_CELL_ID_X_Y, 4, 16, 1)
	U16_CTRL(P7_CID_ISP_LSC_CELL_W,
		 "Mesh cells width in pixel", 0, 511, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_CELL_W, 9, 0, 0)
	U16_CTRL(P7_CID_ISP_LSC_CELL_H,
		 "Mesh cells height in pixel", 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_CELL_H, 10, 0, 0)
	U32_CTRL(P7_CID_ISP_LSC_CELL_W_INV,
		 "Mesh cells inverse width", 0, 131071, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_CELL_W_INV, 17, 0, 0)
	U32_CTRL(P7_CID_ISP_LSC_CELL_H_INV,
		 "Mesh cells inverse height", 0, 131071, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_CELL_H_INV, 17, 0, 0)
	U32_CTRL(P7_CID_ISP_LSC_ALPHA,
		 "Normalised horizontal offset of top left pixel from top left "
		 "vertice of cell",
		 0, 131071, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_ALPHA, 17, 0, 0)
	U32_CTRL(P7_CID_ISP_LSC_BETA,
		 "Normalised vertical offset of top left pixel from top left "
		 "vertice of cell",
		 0, 131071, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_BETA, 17, 0, 0)
	U16_CTRL(P7_CID_ISP_LSC_THRESHOLD_B,
		 "Threshold to avoid declipping", 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_THRESHOLD, 10, 20, 1)
	U16_CTRL(P7_CID_ISP_LSC_THRESHOLD_G,
		 "Threshold to avoid declipping", 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_THRESHOLD, 10, 10, 1)
	U16_CTRL(P7_CID_ISP_LSC_THRESHOLD_R,
		 "Threshold to avoid declipping", 0, 1023, 0,
		  AVI_ISP_LENS_SHADING_CORRECTION +
		  AVI_ISP_LENS_SHADING_CORRECTION_THRESHOLD, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_LSC_GAIN_B,
		 "White balance gain Blue", 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_GAIN, 10, 20, 1)
	U16_CTRL(P7_CID_ISP_LSC_GAIN_G,
		 "White balance gain Green", 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_GAIN, 10, 10, 1)
	U16_CTRL(P7_CID_ISP_LSC_GAIN_R,
		 "White balance gain Red", 0, 1023, 0,
		 AVI_ISP_LENS_SHADING_CORRECTION +
		 AVI_ISP_LENS_SHADING_CORRECTION_GAIN, 10, 0, 1)
	ARRAY8_CTRL(P7_CID_ISP_LSC_RED_COEFF_MEM,
		    "Correction mesh for Red channel (array 15x13)",
		    0, 255, 0, 221,
		    AVI_ISP_LENS_SHADING_CORRECTION +
		    AVI_ISP_LENS_SHADING_CORRECTION_RED_COEFF_MEM, 8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_LSC_GREEN_COEFF_MEM,
		    "Correction mesh for Green channel (array 15x13)",
		    0, 255, 0, 221,
		    AVI_ISP_LENS_SHADING_CORRECTION +
		    AVI_ISP_LENS_SHADING_CORRECTION_GREEN_COEFF_MEM, 8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_LSC_BLUE_COEFF_MEM,
		    "Correction mesh for Blue channel (array 15x13)",
		    0, 255, 0, 221,
		    AVI_ISP_LENS_SHADING_CORRECTION +
		    AVI_ISP_LENS_SHADING_CORRECTION_BLUE_COEFF_MEM, 8, 0, 0)
	U8_CTRL(P7_CID_ISP_LSC_CFA,
		"Color Filter Array of Lens Shading correction",
		0, 3, 0,
		AVI_ISP_LENS_SHADING_CORRECTION +
		AVI_ISP_LENS_SHADING_CORRECTION_BAYER_CFA, 2, 0, 0)
	/* ISP_CHROMA_ABER */
	ARRAY32_CTRL(P7_CID_ISP_CA_RADIUS_SQUARED,
		     "Abscissa of the Look Up Table", 0, 16777215, 0, 20,
		     AVI_ISP_CHROMATIC_ABERRATION +
		     AVI_ISP_CHROMATIC_ABERRATION_RADIUS_SQUARED, 24, 0, 0)
	ARRAY16_CTRL(P7_CID_ISP_CA_DISPLACEMENT_BLUE_COEFF,
		     "Displacement for the BLUE channel", 0, 65535, 0, 20,
		     AVI_ISP_CHROMATIC_ABERRATION +
		     AVI_ISP_CHROMATIC_ABERRATION_DISPLACEMENT_COEFFS, 16, 0, 1)
	ARRAY16_CTRL(P7_CID_ISP_CA_DISPLACEMENT_RED_COEFF,
		     "Displacement for the RED channel", 0, 65535, 0, 20,
		     AVI_ISP_CHROMATIC_ABERRATION +
		     AVI_ISP_CHROMATIC_ABERRATION_DISPLACEMENT_COEFFS,
		     16, 16, 1)
	U16_CTRL(P7_CID_ISP_CA_CIRCLE_POS_X_CENTER,
		 "Center abcissa of the image circle in sensor pixels",
		 0, 16383, 0,
		 AVI_ISP_CHROMATIC_ABERRATION +
		 AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_X_CENTER, 14, 0, 0)
	U32_CTRL(P7_CID_ISP_CA_CIRCLE_POS_X_SQUARED,
		 "Center abcissa of the image circle in sensor pixels value "
		 "squared.",
		 0, 67108863, 0,
		 AVI_ISP_CHROMATIC_ABERRATION +
		 AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_X_SQUARED, 26, 0, 0)
	U16_CTRL(P7_CID_ISP_CA_CIRCLE_POS_Y_CENTER,
		 "Center ordinate of the image circle in sensor pixels",
		 0, 16383, 0,
		 AVI_ISP_CHROMATIC_ABERRATION +
		 AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_Y_CENTER, 14, 0, 0)
	U32_CTRL(P7_CID_ISP_CA_CIRCLE_POS_Y_SQUARED,
		 "Center ordinate of the image circle in sensor pixels value "
		 "squared",
		 0, 67108863, 0,
		 AVI_ISP_CHROMATIC_ABERRATION +
		 AVI_ISP_CHROMATIC_ABERRATION_CIRCLE_POS_Y_SQUARED, 26, 0, 0)
	BOOLEAN_CTRL(P7_CID_ISP_CA_GREEN_VARIATION,
		     "Green variation is taken into account for the correction",
		     AVI_ISP_CHROMATIC_ABERRATION +
		     AVI_ISP_CHROMATIC_ABERRATION_GREEN_VARIATION, 1, 0, 0)
	U8_CTRL(P7_CID_ISP_CA_X_LOG2_INC,
		"Horizontal skipping/binning parameter", 0, 7, 0,
		AVI_ISP_CHROMATIC_ABERRATION +
		AVI_ISP_CHROMATIC_ABERRATION_INCREMENTS_LOG2, 3, 0, 1)
	U8_CTRL(P7_CID_ISP_CA_Y_LOG2_INC,
		"Vertical skipping/binning parameter", 0, 7, 0,
		AVI_ISP_CHROMATIC_ABERRATION +
		AVI_ISP_CHROMATIC_ABERRATION_INCREMENTS_LOG2, 3, 16, 1)
	U8_CTRL(P7_CID_ISP_CA_CFA,
		"Color Filter Array of Chromatic Aberration",
		0, 3, 0,
		AVI_ISP_CHROMATIC_ABERRATION +
		AVI_ISP_CHROMATIC_ABERRATION_CFA, 2, 0, 0)
	/* ISP_BAYER */
	U16_CTRL(P7_CID_ISP_BAYER_THRESHOLD_1, "Low threshold value",
		 0, 8191, 0,
		 AVI_ISP_BAYER + AVI_ISP_BAYER_THRESHOLD_1, 13, 0, 0)
	U16_CTRL(P7_CID_ISP_BAYER_THRESHOLD_2, "High threshold value",
		 0, 8191, 0,
		 AVI_ISP_BAYER + AVI_ISP_BAYER_THRESHOLD_2, 13, 0, 0)
	U8_CTRL(P7_CID_ISP_BAYER_CFA,
		"Color Filter Array of Bayer",
		0, 3, 0,
		AVI_ISP_BAYER + AVI_ISP_BAYER_CFA, 2, 0, 0)
	/* ISP_CONV */
	U16_CTRL(P7_CID_ISP_CC_COEFF_00, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_01_00, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_01, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_01_00, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_02, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_10_02, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_10, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_10_02, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_11, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_12_11, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_12, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_12_11, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_20, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_21_20, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_21, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION +
		 AVI_ISP_COLOR_CORRECTION_COEFF_21_20, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_COEFF_22, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0,
		 AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_COEFF_22,
		 14, 0, 0)
	U16_CTRL(P7_CID_ISP_CC_OFFSET_RY_IN,
		 "Red or Y component offset: before matrix multiplication, the "
		 "pixel vector is subtracted by ’OFFSET_IN’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_OFFSET_RY,
		  10, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_OFFSET_RY_OUT,
		 "Red or Y component offsets: after the matrix multiplication, "
		 "the product is added with ’OFFSET_OUT’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_OFFSET_RY,
		  10, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_CLIP_RY_MIN,
		 "Red or Y component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_CLIP_RY,
		  10, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_CLIP_RY_MAX,
		 "Red or Y component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_CLIP_RY,
		  10, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_OFFSET_GU_IN,
		 "Green or U component offset: before matrix multiplication, "
		 "the pixel vector is subtracted by ’OFFSET_IN’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_OFFSET_GU,
		  10, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_OFFSET_GU_OUT,
		 "Green or U component offsets: after the matrix "
		 "multiplication, the product is added with ’OFFSET_OUT’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_OFFSET_GU,
		  10, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_CLIP_GU_MIN,
		 "Green or U component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_CLIP_GU,
		  10, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_CLIP_GU_MAX,
		 "Green or U component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_CLIP_GU,
		  10, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_OFFSET_BV_IN,
		 "Blue or V component offset: before matrix multiplication, "
		 "the pixel vector is subtracted by ’OFFSET_IN’ value",
		  0, 1023, 0,
		 AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_OFFSET_BV,
		  10, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_OFFSET_BV_OUT,
		 "Blue or V component offsets: after the matrix "
		 "multiplication, the product is added with ’OFFSET_OUT’ value",
		  0, 1023, 0,
		 AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_OFFSET_BV,
		  10, 16, 1)
	U16_CTRL(P7_CID_ISP_CC_CLIP_BV_MIN,
		 "Blue or V component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_CLIP_BV,
		  10, 0, 1)
	U16_CTRL(P7_CID_ISP_CC_CLIP_BV_MAX,
		 "Blue or V component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		  0, 1023, 0,
		  AVI_ISP_COLOR_CORRECTION + AVI_ISP_COLOR_CORRECTION_CLIP_BV,
		  10, 16, 1)
	/* ISP_VLF_40TO32 */
	U8_CTRL(P7_CID_ISP_VLF_40_TO_32, "VL Format 40 to 32", 0, 4, 0,
		AVI_ISP_VL_FORMAT_40_T0_32 + AVI_ISP_VLFORMAT_40TO32_FORMAT,
		3, 0, 0)
};

static const struct avi_v4l2_isp_ctrl_config isp_gamma_ctrls[] = {
	/* ISP_GAMMA_CORRECTION */
	BOOLEAN_CTRL(P7_CID_ISP_GAMMA_BYPASS, "Bypass Gamma correction",
		     AVI_ISP_GAMMA_CORRECTOR_CONF, 1, 0, 1)
	BOOLEAN_CTRL(P7_CID_ISP_GAMMA_PALETTE,
		     "Palette mode (0 = non-linear, 1 = palette)",
		     AVI_ISP_GAMMA_CORRECTOR_CONF, 1, 1, 1)
	BOOLEAN_CTRL(P7_CID_ISP_GAMMA_COMP_WIDTH,
		     "Component width (0 = 8bits, 1 = 10bits)",
		     AVI_ISP_GAMMA_CORRECTOR_CONF, 1, 2, 1)
	ARRAY8_CTRL(P7_CID_ISP_GAMMA_RY_LUT, "Red or Y curve",
		    0, 255, 0, 1024, AVI_ISP_GAMMA_CORRECTOR_RY_LUT, 8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_GAMMA_GU_LUT, "Green or U curve",
		    0, 255, 0, 1024, AVI_ISP_GAMMA_CORRECTOR_GU_LUT, 8, 0, 0)
	ARRAY8_CTRL(P7_CID_ISP_GAMMA_BV_LUT, "Blue or V curve",
		    0, 255, 0, 1024, AVI_ISP_GAMMA_CORRECTOR_BV_LUT, 8, 0, 0)
};

static const struct avi_v4l2_isp_ctrl_config isp_conv_ctrls[] = {
	/* ISP_YUV_CONV */
	U16_CTRL(P7_CID_ISP_CONV_COEFF_00, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_01_00, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_01, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_01_00, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_02, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_10_02, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_10, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_10_02, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_11, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_12_11, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_12, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_12_11, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_20, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_21_20, 14, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_21, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_21_20, 14, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_COEFF_22, "Coded in signed fixed-point Q2.11",
		 0, 16383, 0, AVI_ISP_CHROMA_COEFF_22, 14, 0, 0)
	U16_CTRL(P7_CID_ISP_CONV_OFFSET_RY_IN,
		 "Red or Y component offset: before matrix multiplication, the "
		 "pixel vector is subtracted by ’OFFSET_IN’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_OFFSET_RY, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_OFFSET_RY_OUT,
		 "Red or Y component offsets: after the matrix multiplication, "
		 "the product is added with ’OFFSET_OUT’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_OFFSET_RY, 10, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_CLIP_RY_MIN,
		 "Red or Y component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_CLIP_RY, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_CLIP_RY_MAX,
		 "Red or Y component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_CLIP_RY, 10, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_OFFSET_GU_IN,
		 "Green or U component offset: before matrix multiplication, "
		 "the pixel vector is subtracted by ’OFFSET_IN’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_OFFSET_GU, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_OFFSET_GU_OUT,
		 "Green or U component offsets: after the matrix "
		 "multiplication, the product is added with ’OFFSET_OUT’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_OFFSET_GU, 10, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_CLIP_GU_MIN,
		 "Green or U component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_CLIP_GU, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_CLIP_GU_MAX,
		 "Green or U component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_CLIP_GU, 10, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_OFFSET_BV_IN,
		 "Blue or V component offset: before matrix multiplication, "
		 "the pixel vector is subtracted by ’OFFSET_IN’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_OFFSET_BV, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_OFFSET_BV_OUT,
		 "Blue or V component offsets: after the matrix "
		 "multiplication, the product is added with ’OFFSET_OUT’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_OFFSET_BV, 10, 16, 1)
	U16_CTRL(P7_CID_ISP_CONV_CLIP_BV_MIN,
		 "Blue or V component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_CLIP_BV, 10, 0, 1)
	U16_CTRL(P7_CID_ISP_CONV_CLIP_BV_MAX,
		 "Blue or V component clips: the output result is clipped "
		 "between ’CLIP_MIN’ and ’CLIP_MAX’ value",
		 0, 1023, 0, AVI_ISP_CHROMA_CLIP_BV, 10, 16, 1)
};

static const struct avi_v4l2_isp_ctrl_config isp_stats_ctrls[] = {
	_BOOLEAN_CTRL(P7_CID_ISP_YSTAT_MEASURE_REQ,
		      "Will measure at the beginning of the next frame",
		      AVI_ISP_STATISTICS_YUV_MEASURE_REQ,
		      1, 0, 1, V4L2_CTRL_FLAG_VOLATILE)
	_BOOLEAN_CTRL(P7_CID_ISP_YSTAT_CLEAR, "Clearing the histogram",
		      AVI_ISP_STATISTICS_YUV_MEASURE_REQ,
		      1, 1, 1, V4L2_CTRL_FLAG_VOLATILE)
	_BOOLEAN_CTRL(P7_CID_ISP_YSTAT_DONE,
		      "Asserted at the end of each measured frame",
		      AVI_ISP_STATISTICS_YUV_MEASURE_STATUS,
		      1, 0, 1, V4L2_CTRL_FLAG_VOLATILE)
	_BOOLEAN_CTRL(P7_CID_ISP_YSTAT_ERROR,
		      "Asserted if there is valid input stream during the "
		      "histogram clearing process",
		      AVI_ISP_STATISTICS_YUV_MEASURE_STATUS,
		      1, 1, 1, V4L2_CTRL_FLAG_VOLATILE)
	U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_X_START,
		 "Beginning of the measured window X",
		 0, 8191, 0,
		 AVI_ISP_STATISTICS_YUV_WINDOW_POS_X,
		 13, 0, 1)
	U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_X_END, "End of the measured window X",
		 0, 8191, 0,
		 AVI_ISP_STATISTICS_YUV_WINDOW_POS_X,
		 13, 16, 1)
	U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_Y_START,
		 "Beginning of the measured window Y",
		 0, 8191, 0,
		 AVI_ISP_STATISTICS_YUV_WINDOW_POS_Y,
		 13, 0, 1)
	U16_CTRL(P7_CID_ISP_BSTAT_WINDOW_Y_END, "End of the measured window Y",
		 0, 8191, 0,
		 AVI_ISP_STATISTICS_YUV_WINDOW_POS_Y,
		 13, 16, 1)
	U16_CTRL(P7_CID_ISP_YSTAT_CIRCLE_POS_X_CENTER, "Circle center X",
		 0, 16383, 0,
		 AVI_ISP_STATISTICS_YUV_CIRCLE_POS_X_CENTER,
		 14, 0, 0)
	U32_CTRL(P7_CID_ISP_YSTAT_CIRCLE_POS_X_SQUARED, "X center squared",
		 0, 67108863, 0,
		 AVI_ISP_STATISTICS_YUV_CIRCLE_POS_X_SQUARED,
		 26, 0, 0)
	U16_CTRL(P7_CID_ISP_YSTAT_CIRCLE_POS_Y_CENTER, "Circle center Y",
		 0, 16383, 0,
		 AVI_ISP_STATISTICS_YUV_CIRCLE_POS_Y_CENTER,
		 14, 0, 0)
	U32_CTRL(P7_CID_ISP_YSTAT_CIRCLE_POS_Y_SQUARED, "Y center squared",
		 0, 67108863, 0,
		 AVI_ISP_STATISTICS_YUV_CIRCLE_POS_Y_SQUARED,
		 26, 0, 0)
	U32_CTRL(P7_CID_ISP_YSTAT_CIRCLE_RADIUS_SQUARED,
		"Circle radius squared",
		 0, 536870911, 0,
		 AVI_ISP_STATISTICS_YUV_CIRCLE_RADIUS_SQUARED,
		 29, 0, 0)
	U8_CTRL(P7_CID_ISP_YSTAT_INC_X_LOG2_INC,
		"Horizontal skipping/binning parameter",
		0, 7, 0,
		AVI_ISP_STATISTICS_YUV_INCREMENTS_LOG2,
		3, 0, 1)
	U8_CTRL(P7_CID_ISP_YSTAT_INC_Y_LOG2_INC,
		"Vertical skipping/binning parameter",
		0, 7, 0,
		AVI_ISP_STATISTICS_YUV_INCREMENTS_LOG2,
		3, 16, 1)
	_U32_CTRL(P7_CID_ISP_YSTAT_AE_NB_VALID_Y, "Number of valid luma pixels",
		 0, 4194303, 0,
		 AVI_ISP_STATISTICS_YUV_AE_NB_VALID_Y,
		 22 , 0, 0, V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)
	U8_CTRL(P7_CID_ISP_YSTAT_AWB_THRESHOLD,
		"Auto white balancing threshold",
		0, 255, 0,
		AVI_ISP_STATISTICS_YUV_AWB_THRESHOLD,
		8, 0, 0)
	_U32_CTRL(P7_CID_ISP_YSTAT_AWB_SUM_Y, "White balancing sum of Y",
		 0, 1073741823, 0,
		 AVI_ISP_STATISTICS_YUV_AWB_SUM_Y,
		 30, 0, 0, V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)
	_U32_CTRL(P7_CID_ISP_YSTAT_AWB_SUM_U, "White balancing sum of U",
		 0, 1073741823, 0,
		 AVI_ISP_STATISTICS_YUV_AWB_SUM_U,
		 30, 0, 0, V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)
	_U32_CTRL(P7_CID_ISP_YSTAT_AWB_SUM_V, "White balancing sum of V",
		 0, 1073741823, 0,
		 AVI_ISP_STATISTICS_YUV_AWB_SUM_V,
		 30, 0, 0, V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)
	_U32_CTRL(P7_CID_ISP_YSTAT_AWB_NB_GREY_PIXELS,
		 "Number of valid grey pixels",
		 0, 4194303, 0,
		 AVI_ISP_STATISTICS_YUV_AWB_NB_GREY_PIXELS,
		 22, 0, 0, V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)
	_ARRAY32_CTRL(P7_CID_ISP_YSTAT_AE_HISTOGRAM_Y, "Value of each bin",
		     0, 4194303, 0, 256,
		     AVI_ISP_STATISTICS_YUV_AE_HISTOGRAM_Y,
		     22, 0, 0,
		     V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_VOLATILE)
};

static const struct avi_v4l2_isp_ctrl_config isp_yuv_ctrls[] = {
	/* ISP EE_CRF_I3D_LUT_DROP */
	BOOLEAN_CTRL(P7_CID_BYPASS_EE_CRF, "Bypass EE CRF ISP Module",
		     AVI_ISP_CHAIN_YUV_INTER +
		     AVI_ISP_CHAIN_YUV_INTER_MODULE_BYPASS, 1, 0, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_I3D_LUT, "Bypass I3D LUT ISP Module",
		     AVI_ISP_CHAIN_YUV_INTER +
		     AVI_ISP_CHAIN_YUV_INTER_MODULE_BYPASS, 1, 1, 1)
	BOOLEAN_CTRL(P7_CID_BYPASS_DROP, "Bypass Drop ISP Module",
		     AVI_ISP_CHAIN_YUV_INTER +
		     AVI_ISP_CHAIN_YUV_INTER_MODULE_BYPASS, 1, 2, 1)
	/* ISP_EE_CRF */
	ARRAY8_CTRL(P7_CID_ISP_EE_CRF_EE_LUT, "EE LUT table", 0, 63, 0, 256,
		    AVI_ISP_EDGE_ENHANCEMENT +
		    AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_EE_LUT,
		    6, 0, 0)
	ARRAY16_CTRL(P7_CID_ISP_EE_CRF_EE_KERNEL_COEFF, "EE kernel",
		     0, 2047, 0, 6,
		     AVI_ISP_EDGE_ENHANCEMENT +
		AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_EE_KERNEL_COEFF,
		     11, 0, 0)
	ARRAY16_CTRL(P7_CID_ISP_EE_CRF_CRF_KERNEL_COEFF, "CRF kernel",
		     0, 2047, 0, 6,
		     AVI_ISP_EDGE_ENHANCEMENT +
	       AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_CRF_KERNEL_COEFF,
		     11, 0, 0)
	U8_CTRL(P7_CID_ISP_EE_CRF_M_COEFF, "Sobel threshold",
		0, 255, 0,
		AVI_ISP_EDGE_ENHANCEMENT +
		AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_M_COEFF,
		8, 0, 0)
	U16_CTRL(P7_CID_ISP_EE_CRF_D_COEFF, "Normalization factor", 0, 2047, 0,
		 AVI_ISP_EDGE_ENHANCEMENT +
		 AVI_ISP_EDGE_ENHANCEMENT_COLOR_REDUCTION_FILTER_D_COEFF,
		 11, 0, 0)
	/* ISP_DROP */
	U16_CTRL(P7_CID_ISP_DROP_OFFSET_X, "Horizontal offset", 0, 65535, 0,
		 AVI_ISP_DROP + AVI_ISP_DROP_OFFSET_X, 16, 0, 0)
	U16_CTRL(P7_CID_ISP_DROP_OFFSET_Y, "Vertical offset", 0, 65535, 0,
		 AVI_ISP_DROP + AVI_ISP_DROP_OFFSET_Y, 16, 0, 0)
};

static const struct avi_v4l2_isp_ctrl_config isp_i3d_ctrls[] = {
	/* ISP_I3D_LUT */
	ARRAY8_CTRL(P7_CID_ISP_I3D_LUT_OUTSIDE_RY, "Red or Y component value",
		    0, 255, 0, 125,
		    AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_OUTSIDE, 8, 16, 1)
	ARRAY8_CTRL(P7_CID_ISP_I3D_LUT_OUTSIDE_GU, "Green or U component value",
		    0, 255, 0, 125,
		    AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_OUTSIDE, 8, 8, 1)
	ARRAY8_CTRL(P7_CID_ISP_I3D_LUT_OUTSIDE_BV, "Blue or V component value",
		    0, 255, 0, 125,
		    AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_OUTSIDE, 8, 0, 1)
	ARRAY8_CTRL(P7_CID_ISP_I3D_LUT_INSIDE_RY, "Red or Y component value",
		    0, 255, 0, 125,
		    AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_INSIDE, 8, 16, 1)
	ARRAY8_CTRL(P7_CID_ISP_I3D_LUT_INSIDE_GU, "Green or U component value",
		    0, 255, 0, 125,
		    AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_INSIDE, 8, 8, 1)
	ARRAY8_CTRL(P7_CID_ISP_I3D_LUT_INSIDE_BV, "Blue or V component value",
		    0, 255, 0, 125,
		    AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_LUT_INSIDE, 8, 0, 1)
	BOOLEAN_CTRL(P7_CID_ISP_I3D_LUT_CLIP_MODE, "YUV Clip",
		     AVI_ISP_I3D_LUT + AVI_ISP_I3D_LUT_CLIP_MODE, 1, 0, 0)
};

struct avi_v4l2_isp_node {
	int is_activate;
	u32 base_node;
};

struct avi_v4l2_isp_priv {
	const struct avi_v4l2_isp_reg *reg;
	struct avi_v4l2_isp_node *node;
	int blanking_update;
};

struct avi_v4l2_isp {
	/* ISP chain bayer */
	struct v4l2_ctrl *bayer_ctrls[ARRAY_SIZE(isp_bayer_ctrls)];
	struct avi_v4l2_isp_priv bayer_priv[ARRAY_SIZE(isp_bayer_ctrls)];
	struct avi_v4l2_isp_node bayer_node;
	/* ISP Gamma correction */
	struct v4l2_ctrl *gamma_ctrls[ARRAY_SIZE(isp_gamma_ctrls)];
	struct avi_v4l2_isp_priv gamma_priv[ARRAY_SIZE(isp_gamma_ctrls)];
	struct avi_v4l2_isp_node gamma_node;
	/* ISP Converter */
	struct v4l2_ctrl *conv_ctrls[ARRAY_SIZE(isp_conv_ctrls)];
	struct avi_v4l2_isp_priv conv_priv[ARRAY_SIZE(isp_conv_ctrls)];
	struct avi_v4l2_isp_node conv_node;
	/* ISP statistics YUV */
	struct v4l2_ctrl *stats_ctrls[ARRAY_SIZE(isp_stats_ctrls)];
	struct avi_v4l2_isp_priv stats_priv[ARRAY_SIZE(isp_stats_ctrls)];
	struct avi_v4l2_isp_node stats_node;
	/* ISP chain YUV */
	struct v4l2_ctrl *yuv_ctrls[ARRAY_SIZE(isp_yuv_ctrls)];
	struct avi_v4l2_isp_priv yuv_priv[ARRAY_SIZE(isp_yuv_ctrls)];
	struct avi_v4l2_isp_node yuv_node;
	/* ISP chain YUV: I3D LUT */
	struct v4l2_ctrl *i3d_ctrls[ARRAY_SIZE(isp_i3d_ctrls)];
	struct avi_v4l2_isp_priv i3d_priv[ARRAY_SIZE(isp_i3d_ctrls)];
	struct avi_v4l2_isp_node i3d_node;
};

static int avi_v4l2_isp_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avi_v4l2_isp_priv *priv = ctrl->priv;
	const struct avi_v4l2_isp_reg *isp_reg = priv->reg;
	u32 base_node = priv->node->base_node;
	u32 value, i;

	/* Do not change ISP configuration when controls are not activated */
	if (!priv->node->is_activate)
		return 0;

	/* Read values from registers */
	for (i = 0; i < isp_reg->count; i++) {
		/* Get register value */
		value = (AVI_READ(base_node + isp_reg->addr + (i * sizeof(u32)))
			 & isp_reg->mask) >> isp_reg->shift;

		/* Get new value */
		if (ctrl->is_array)
			ctrl->p_new.p_s32[i] = value;
		else
			ctrl->val = value;
	}

	return 0;
}

static int avi_v4l2_isp_set(struct v4l2_ctrl *ctrl,
			    const struct avi_v4l2_isp_reg *isp_reg,
			    u32 base_node)
{
	u32 reg, value, i;

	/* Special case for Denoising registers */
	if (ctrl->id == P7_CID_ISP_DENOISE_LUMOCOEFF_RED ||
	    ctrl->id == P7_CID_ISP_DENOISE_LUMOCOEFF_GREEN ||
	    ctrl->id == P7_CID_ISP_DENOISE_LUMOCOEFF_BLUE) {
		for (i = 0; i < isp_reg->count; i += 4) {
			/* Prepare value */
			reg = (ctrl->p_new.p_u8[i+1] << 8) |
			      ctrl->p_new.p_u8[i];
			if (i < 12)
				reg |= (ctrl->p_new.p_u8[i+3] << 24) |
				       (ctrl->p_new.p_u8[i+2] << 16);

			/* Write to register */
			AVI_WRITE(reg, base_node + isp_reg->addr + i);
		}
		return 0;
	}

	/* Special case for chroma abberation registers */
	if (ctrl->id == P7_CID_ISP_CA_RADIUS_SQUARED ||
	    ctrl->id == P7_CID_ISP_CA_DISPLACEMENT_BLUE_COEFF ||
	    ctrl->id == P7_CID_ISP_CA_DISPLACEMENT_RED_COEFF) {
		for (i = 0; i < isp_reg->count; i++) {
			/* Prepare new value */
			if (ctrl->id != P7_CID_ISP_CA_RADIUS_SQUARED) {
				reg = AVI_READ(base_node + isp_reg->addr +
							 (2 * i * sizeof(u32)));
				reg = ((ctrl->p_new.p_u16[i] << isp_reg->shift)
				      & isp_reg->mask) | (reg & ~isp_reg->mask);
			} else {
				reg = ctrl->p_new.p_s32[i] & 0xFFFFFF;
			}

			/* Write value to registers */
			AVI_WRITE(reg, base_node + isp_reg->addr +
							 (2 * i * sizeof(u32)));
		}
		return 0;
	}

	/* Write values to registers */
	for (i = 0; i < isp_reg->count; i++) {
		/* Read register before */
		if (isp_reg->read)
			reg = AVI_READ(base_node + isp_reg->addr +
							     (i * sizeof(u32)));
		else
			reg = 0;

		/* Get new value */
		if (ctrl->is_array) {
			switch (ctrl->type) {
			case V4L2_CTRL_TYPE_U16:
				value = ctrl->p_new.p_u16[i];
				break;
			case V4L2_CTRL_TYPE_U8:
				value = ctrl->p_new.p_u8[i];
				break;
			default:
				value = 0;
			}
		} else
			value = ctrl->val;

		/* Prepare value for write */
		reg = ((value << isp_reg->shift) & isp_reg->mask) |
			       (reg & ~isp_reg->mask);

		/* Write to register */
		AVI_WRITE(reg, base_node + isp_reg->addr + (i * sizeof(u32)));
	}

	return 0;
}

static int avi_v4l2_isp_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct avi_v4l2_isp_priv *priv = ctrl->priv;
	const struct avi_v4l2_isp_reg *isp_reg = priv->reg;
	u32 base_node = priv->node->base_node;

	/* Do not change ISP configuration when controls are not activated */
	if (!priv->node->is_activate)
		return 0;

	/* These registers are not written */
	if (ctrl->id == P7_CID_ISP_YSTAT_AE_NB_VALID_Y ||
	    ctrl->id == P7_CID_ISP_YSTAT_AWB_SUM_Y ||
	    ctrl->id == P7_CID_ISP_YSTAT_AWB_SUM_U ||
	    ctrl->id == P7_CID_ISP_YSTAT_AWB_SUM_V ||
	    ctrl->id == P7_CID_ISP_YSTAT_AWB_NB_GREY_PIXELS ||
	    ctrl->id == P7_CID_ISP_YSTAT_AE_HISTOGRAM_Y)
		return -EINVAL;

	/* Workaround: I3D LUT must be set during blanking period */
	if (ctrl->id == P7_CID_ISP_I3D_LUT_OUTSIDE_RY ||
	    ctrl->id == P7_CID_ISP_I3D_LUT_OUTSIDE_GU ||
	    ctrl->id == P7_CID_ISP_I3D_LUT_OUTSIDE_BV ||
	    ctrl->id == P7_CID_ISP_I3D_LUT_INSIDE_RY ||
	    ctrl->id == P7_CID_ISP_I3D_LUT_INSIDE_GU ||
	    ctrl->id == P7_CID_ISP_I3D_LUT_INSIDE_BV ||
	    ctrl->id == P7_CID_ISP_I3D_LUT_CLIP_MODE) {
		/* Update configuration at next blanking call */
		priv->blanking_update = 1;
		return 0;
	}

	/* Set control */
	return avi_v4l2_isp_set(ctrl, isp_reg, base_node);
}

/* An hardware bug in I3D LUT block imply its configuration must be done during
 * blanking period or when stream is off.
 */
int avi_v4l2_isp_blanking(struct avi_v4l2_isp *isp)
{
	int i;

	/* Return immediatly when ISP is not activated */
	if (!isp->i3d_node.is_activate)
		return 0;

	/* Set I3D LUT configuration */
	for (i = 0; i < ARRAY_SIZE(isp_i3d_ctrls); i++) {
		if (isp->i3d_priv[i].blanking_update) {
			avi_v4l2_isp_set(isp->i3d_ctrls[i],
					 isp->i3d_priv[i].reg,
					 isp->i3d_node.base_node);
			isp->i3d_priv[i].blanking_update = 0;
		}
	}

	return 0;
}
EXPORT_SYMBOL(avi_v4l2_isp_blanking);

int avi_v4l2_isp_activate(struct avi_v4l2_isp *isp,
			  struct avi_isp_offsets *offsets)
{
	int ret = 0, i;

#define RESTORE_CONTROLS(_off, _name) \
for (i = 0; i < ARRAY_SIZE(isp_ ## _name ## _ctrls); i++) { \
	isp->_name ## _node.is_activate = 1; \
	isp->_name ## _node.base_node = avi_ctrl.base + offsets->_off; \
	if (isp->_name ## _ctrls[i] != NULL) \
			ret |= avi_v4l2_isp_s_ctrl(isp->_name ## _ctrls[i]); \
}

	/* Set ISP offest and restore all control values */
	RESTORE_CONTROLS(chain_bayer, bayer);
	RESTORE_CONTROLS(gamma_corrector, gamma);
	RESTORE_CONTROLS(chroma, conv);
	RESTORE_CONTROLS(statistics_yuv, stats);
	RESTORE_CONTROLS(chain_yuv, yuv);
	RESTORE_CONTROLS(chain_yuv, i3d);

#undef RESTORE_CONTROLS

	return ret;
}
EXPORT_SYMBOL(avi_v4l2_isp_activate);

void avi_v4l2_isp_deactivate(struct avi_v4l2_isp *isp)
{
	/* Deactivate ISP chain */
	isp->bayer_node.is_activate = 0;
	isp->gamma_node.is_activate = 0;
	isp->conv_node.is_activate = 0;
	isp->stats_node.is_activate = 0;
	isp->yuv_node.is_activate = 0;
	isp->i3d_node.is_activate = 0;
}
EXPORT_SYMBOL(avi_v4l2_isp_deactivate);

int avi_v4l2_isp_init(struct avi_v4l2_isp **_isp,
		      struct v4l2_ctrl_handler *ctrl_handler)
{
	struct avi_v4l2_isp *isp;
	int i;

	BUG_ON(!ctrl_handler);

	/* Allocate a new ISP context */
	*_isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (*_isp == NULL)
		return -ENOMEM;
	isp = *_isp;

#define ADD_CONTROLS(_name) \
for (i = 0; i < ARRAY_SIZE(isp_ ## _name ## _ctrls); i++) { \
	isp->_name ## _priv[i].node = &isp->_name ## _node; \
	isp->_name ## _priv[i].reg = &isp_ ## _name ## _ctrls[i].reg; \
	isp->_name ## _ctrls[i] = v4l2_ctrl_new_custom(ctrl_handler, \
					      &isp_ ## _name ## _ctrls[i].cfg, \
					      &isp->_name ## _priv[i]); \
}

	/* Add ISP controls to control handler */
	ADD_CONTROLS(bayer);
	ADD_CONTROLS(gamma);
	ADD_CONTROLS(conv);
	ADD_CONTROLS(stats);
	ADD_CONTROLS(yuv);
	ADD_CONTROLS(i3d);

#undef ADD_CONTROLS

	return 0;
}
EXPORT_SYMBOL(avi_v4l2_isp_init);

void avi_v4l2_isp_free(struct avi_v4l2_isp *isp)
{
	if (!isp)
		return;
	kfree(isp);
}
EXPORT_SYMBOL(avi_v4l2_isp_free);

MODULE_AUTHOR("Alexandre Dilly <alexandre.dilly@parrot.com>");
MODULE_DESCRIPTION("V4L2 controls for ISP chains of Advanced Video Interface");
MODULE_LICENSE("GPL");
