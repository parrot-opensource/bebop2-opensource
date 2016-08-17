/*
 *  linux/drivers/parrot/video/media/avi_v4l2_isp_ctrls.h
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

#ifndef _AVI_V4L2_ISP_CTRLS_H_
#define _AVI_V4L2_ISP_CTRLS_H_

#include <linux/videodev2.h>

/* Definition of controls
 *  All V4L2 controls are defined in this header with the following syntax:
 *    NAME:	(TYPE)		Description
 *  Where NAME is the name of control to pass as ID in v4l2_ext_control
 *  structure, TYPE is one of type provided by V4L2 control API:
 *    - bool	for a boolean (V4L2_CTRL_TYPE_BOOLEAN)
 *    - int	for an integer (V4L2_CTRL_TYPE_INTEGER)
 *      The syntax format is (int Xb [MIN:MAX]) where X is number of bytes and
 *      MIN/MAX are respectively the minimum and maximum value allowed by the
 *      control. If the value passed in v4l2_ext_control is out of range, all
 *      set of controls in v4l2_ext_controls fail.
 *    - ary[Y]	for an array (multi types (U8 to S32) with 1 dimension: matrix
 *		of 1xY)
 *      The syntax is the same as integer.
 *      To use with an v4l2_ext_control, the local array is passed through its
 *      pointer with one of the following field:
 *       - p_u8		for an U8. Size is equal to Y (count of value in the
 *			array),
 *       - p_u16	for an U16. Size is equal to Y*2,
 *       - ptr		for an S/U32. Size is equal to Y*4.
 *
 * For more informations about the ext controls, see videodev2.h and
 * v4l2_ext_control and v4l2_ext_controls structures.
 */

/* Color Filter Array */
enum p7_isp_cfa {
	P7_ISP_CFA_BGGR = 0,
	P7_ISP_CFA_RGGB = 1,
	P7_ISP_CFA_GBRG = 2,
	P7_ISP_CFA_GRBG = 3,
};

#define P7_CID_ISP_BASE (V4L2_CID_IMAGE_PROC_CLASS_BASE + 10)

/* Bypass ISP module
 *  BYPASS_PEDESTAL:	 (bool)	Bypass Pedestal block
 *  BYPASS_GRIM:	 (bool)	Bypass Green Imbalance block
 *  BYPASS_RIP:		 (bool)	Bypass Dead Pixel Correction block
 *  BYPASS_DENOISE:	 (bool)	Bypass Denoising block
 *  BYPASS_LSC:		 (bool)	Bypass Lens Shading Correction block
 *  BYPASS_CHROMA_ABER:	 (bool)	Bypass Chromatic Aberration block
 *  BYPASS_BAYER:	 (bool)	Bypass Bayer block
 *  BYPASS_COLOR_MATRIX: (bool)	Byapss Color Correction block
 */
#define P7_CID_BYPASS_BASE		(P7_CID_ISP_BASE + 1)
#define P7_CID_BYPASS_PEDESTAL		(P7_CID_BYPASS_BASE + 1)
#define P7_CID_BYPASS_GRIM		(P7_CID_BYPASS_BASE + 2)
#define P7_CID_BYPASS_RIP		(P7_CID_BYPASS_BASE + 3)
#define P7_CID_BYPASS_DENOISE		(P7_CID_BYPASS_BASE + 4)
#define P7_CID_BYPASS_LSC		(P7_CID_BYPASS_BASE + 5)
#define P7_CID_BYPASS_CHROMA_ABER	(P7_CID_BYPASS_BASE + 6)
#define P7_CID_BYPASS_BAYER		(P7_CID_BYPASS_BASE + 7)
#define P7_CID_BYPASS_COLOR_MATRIX	(P7_CID_BYPASS_BASE + 8)

/* Bypass YUV ISP module
 *  BYPASS_EE_CRF:	(bool)	Bypass Edge Enhancement block
 *  BYPASS_I3D_LUT:	(bool)	Bypass I3D block
 *  BYPASS_DROP:	(bool)	Bypass Drop block
 */
#define P7_CID_BYPASS_YUV_BASE	(P7_CID_BYPASS_BASE + 10)
#define P7_CID_BYPASS_EE_CRF	(P7_CID_BYPASS_YUV_BASE + 1)
#define P7_CID_BYPASS_I3D_LUT	(P7_CID_BYPASS_YUV_BASE + 2)
#define P7_CID_BYPASS_DROP	(P7_CID_BYPASS_YUV_BASE + 3)

/* VL Format 32 to 40 ISP module
 *  VLF_32_TO_40:	(int 3b [0:4])	Convert format from 32 to 40
 *    see enum p7_vlf_32to40 for types.
 */
#define P7_CID_ISP_VLF_32_TO_40		(P7_CID_BYPASS_YUV_BASE + 5)
enum p7_vlf_32to40 {
	P7_VLF_32_RAW10_TO_RAW10,
	P7_VLF_32_RAW8_TO_RAW10,
	P7_VLF_32_RAW3x10_TO_RAW10,
	P7_VLF_32_RGB3x10_TO_RGB3x10,
	P7_VLF_32_ARGB_TO_RGB3x10,
};

/* Pedestal ISP module
 *  PEDESTAL_SUB_R:	(int 10b [0:1023])	Substract X to Red pixel
 *  PEDESTAL_SUB_GB:	(int 10b [0:1023])	Substract X to Green red pixel
 *  PEDESTAL_SUB_GR:	(int 10b [0:1023])	Substract X to Green blue pixel
 *  PEDESTAL_SUB_B:	(int 10b [0:1023])	Substract X to Blue pixel
 *  PEDESTAL_CFA:	(int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_PEDESTAL_BASE	(P7_CID_BYPASS_YUV_BASE + 10)
#define P7_CID_ISP_PEDESTAL_SUB_R	(P7_CID_ISP_PEDESTAL_BASE + 1)
#define P7_CID_ISP_PEDESTAL_SUB_GB	(P7_CID_ISP_PEDESTAL_BASE + 2)
#define P7_CID_ISP_PEDESTAL_SUB_GR	(P7_CID_ISP_PEDESTAL_BASE + 3)
#define P7_CID_ISP_PEDESTAL_SUB_B	(P7_CID_ISP_PEDESTAL_BASE + 4)
#define P7_CID_ISP_PEDESTAL_CFA		(P7_CID_ISP_PEDESTAL_BASE + 5)

/* Green Imbalance ISP module
 *  GRIM_OFFSET_X:	(int 9b [0:511])	Offset of top left pixel from
 *						top left vertice of cell
 *  GRIM_OFFSET_Y:	(int 10b [0:1023])	Offset of top left pixel from
 *						top left vertice of cell
 *  GRIM_CELL_ID_X:	(int 4b [0:15])		Horizontal cell index to which
 *						top left pixel belongs
 *  GRIM_CELL_ID_Y:	(int 4b [0:15])		Vertical cell index to which top
 *						left pixel belongs
 *  GRIM_CELL_W:	(int 9b [0:511])	Mesh cells width in pixel
 *  GRIM_CELL_H:	(int 10b [0:1023])	Mesh cells height in pixel
 *  GRIM_CELL_W_INV:	(int 17b [0:131071])	Mesh cells inverse width
 *  GRIM_CELL_H_INV:	(int 17b [0:131071])	Mesh cells inverse height
 *  GRIM_ALPHA:		(int 17b [0:131071])	Normalised horizontal offset of
 *						top left pixel from top left
 *						vertice of cell
 *  GRIM_BETA:		(int 17b [0:131071])	Normalised vertical offset of
 *						top left pixel from top left
 *						vertice of cell
 *  GRIM_GR_COEFF:	(ary[221] 8b [0:255])	Red coefficients: Correction
 *						mesh for R channel
 *  GRIM_GB_COEFF:	(ary[221] 8b [0:255])	Blue coefficients: Correction
 *						mesh for B channel
 *  GRIM_CFA:		(int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_GRIM_BASE		(P7_CID_ISP_PEDESTAL_BASE + 10)
#define P7_CID_ISP_GRIM_OFFSET_X	(P7_CID_ISP_GRIM_BASE + 1)
#define P7_CID_ISP_GRIM_OFFSET_Y	(P7_CID_ISP_GRIM_BASE + 2)
#define P7_CID_ISP_GRIM_CELL_ID_X	(P7_CID_ISP_GRIM_BASE + 3)
#define P7_CID_ISP_GRIM_CELL_ID_Y	(P7_CID_ISP_GRIM_BASE + 4)
#define P7_CID_ISP_GRIM_CELL_W		(P7_CID_ISP_GRIM_BASE + 5)
#define P7_CID_ISP_GRIM_CELL_H		(P7_CID_ISP_GRIM_BASE + 6)
#define P7_CID_ISP_GRIM_CELL_W_INV	(P7_CID_ISP_GRIM_BASE + 7)
#define P7_CID_ISP_GRIM_CELL_H_INV	(P7_CID_ISP_GRIM_BASE + 8)
#define P7_CID_ISP_GRIM_ALPHA		(P7_CID_ISP_GRIM_BASE + 9)
#define P7_CID_ISP_GRIM_BETA		(P7_CID_ISP_GRIM_BASE + 10)
#define P7_CID_ISP_GRIM_GR_COEFF	(P7_CID_ISP_GRIM_BASE + 11)
#define P7_CID_ISP_GRIM_GB_COEFF	(P7_CID_ISP_GRIM_BASE + 12)
#define P7_CID_ISP_GRIM_CFA		(P7_CID_ISP_GRIM_BASE + 13)

/* Reconstruction Incorrect Pixel ISP module
 *  LIST_MEM_X:			(ary[256] 11b [0:2047])	Abscissa of known dead
 *							pixel
 *  LIST_MEM_Y:			(ary[256] 12b [0:4095])	Ordinate of known dead
 *							pixel
 *  LIST_MEM_OP:		(ary[256] 3bits [0:7])	List of suitable
 *							corrections for each
 *							corresponding dead pixel
 *							position
 *  BYPASS_RGRIM:		(bool)			Deactivates list mode
 *  BYPASS_AUTO_DETECTION:	(bool)			Deactivates live mode
 *  BYPASS_LIST:		(bool)			Deactivates the RGRIM
 *							filter list mode, which
 *							in turn takes precedence
 *							over the RGRIM filter
 *  THRESHOLD:			(int 10b [0:1023])	Used in Live mode to
 *							compare one pixel to its
 *							neighbours
 *  RGRIM_CONF_TCON:		(int 8b [0:255])	Contrast
 *  RGRIM_CONF_TSAT:		(int 8b [0:255])	Saturation
 *  RGRIM_CONF_TIM_2:		(int 8b [0:255])	Imbalance 2
 *  RGRIM_CONF_TIM_1:		(int 8b [0:255])	Imbalance 1
 *  RGRIM_GAIN:			(int 7b [0:127])	The gain obeys the
 *							formula : actual gain =
 *							20 * gain / 4096
 *  RIP_CFA:			(int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_RIP_BASE			(P7_CID_ISP_GRIM_BASE + 20)
#define P7_CID_ISP_RIP_LIST_MEM_X		(P7_CID_ISP_RIP_BASE + 1)
#define P7_CID_ISP_RIP_LIST_MEM_Y		(P7_CID_ISP_RIP_BASE + 2)
#define P7_CID_ISP_RIP_LIST_MEM_OP		(P7_CID_ISP_RIP_BASE + 3)
#define P7_CID_ISP_RIP_BYPASS_RGRIM		(P7_CID_ISP_RIP_BASE + 4)
#define P7_CID_ISP_RIP_BYPASS_AUTO_DETECTION	(P7_CID_ISP_RIP_BASE + 5)
#define P7_CID_ISP_RIP_BYPASS_LIST		(P7_CID_ISP_RIP_BASE + 6)
#define P7_CID_ISP_RIP_THRESHOLD		(P7_CID_ISP_RIP_BASE + 7)
#define P7_CID_ISP_RIP_RGRIM_CONF_TCON		(P7_CID_ISP_RIP_BASE + 8)
#define P7_CID_ISP_RIP_RGRIM_CONF_TSAT		(P7_CID_ISP_RIP_BASE + 9)
#define P7_CID_ISP_RIP_RGRIM_CONF_TIM_2		(P7_CID_ISP_RIP_BASE + 10)
#define P7_CID_ISP_RIP_RGRIM_CONF_TIM_1		(P7_CID_ISP_RIP_BASE + 11)
#define P7_CID_ISP_RIP_RGRIM_GAIN		(P7_CID_ISP_RIP_BASE + 12)
#define P7_CID_ISP_RIP_CFA			(P7_CID_ISP_RIP_BASE + 13)

/* Denoising ISP module
 *  DENOISE_LUMOCOEFF_RED:	(ary[14] 8b [0:255])	Ordinates of lumo coeffs
 *							Red
 *  DENOISE_LUMOCOEFF_BLUE:	(ary[14] 8b [0:255])	Ordinates of lumo coeffs
 *							Blue
 *  DENOISE_LUMOCOEFF_GREEN:	(ary[14] 8b [0:255])	Ordinates of lumo coeffs
 *							Green
 *  DENOISE_CFA:		(int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_DENOISE_BASE			(P7_CID_ISP_RIP_BASE + 15)
#define P7_CID_ISP_DENOISE_LUMOCOEFF_RED	(P7_CID_ISP_DENOISE_BASE + 1)
#define P7_CID_ISP_DENOISE_LUMOCOEFF_BLUE	(P7_CID_ISP_DENOISE_BASE + 2)
#define P7_CID_ISP_DENOISE_LUMOCOEFF_GREEN	(P7_CID_ISP_DENOISE_BASE + 3)
#define P7_CID_ISP_DENOISE_CFA			(P7_CID_ISP_DENOISE_BASE + 4)

/* Bayer Statistics ISP module
 *  BSTAT_MEASURE_REQ_CLEAR:	 (bool)			Force clear the
 *							histogram, usually done
 *							the first time before
 *							enabling the stats bayer
 *							module
 *  BSTAT_WINDOW_X_WIDTH:	 (int 11b [0:2047])	Width of a thumbnail
 *							pixel in sensor pixels
 *							(how many columns are
 *							averaged into one)
 *  BSTAT_WINDOW_X_OFFSET:	 (int 13b [0:8191])	Origin abcissa of the
 *							statistics window in the
 *							captured image
 *  BSTAT_WINDOW_Y_HEIGHT:	 (int 11b [0:2047])	Height of a thumbnail
 *							pixel in sensor pixels
 *							(how many rows are
 *							averaged into one)
 *  BSTAT_WINDOW_Y_OFFSET:	 (int 13b [0:8191])	Origin abcissa of the
 *							statistics window in the
 *							captured image
 *  BSTAT_CIRCLE_POS_X_CENTER:	 (int 14b [0:16383])	Center abcissa of the
 *							image circle in sensor
 *							pixels
 *  BSTAT_CIRCLE_POS_X_SQUARED:	 (int 26b [0:67108863])	Center abcissa of the
 *							image circle in sensor
 *							pixels value squared
 *  BSTAT_CIRCLE_POS_Y_CENTER:	 (int 14b [0:16383])	Center ordinate of the
 *							image circle in sensor
 *							pixels
 *  BSTAT_CIRCLE_POS_Y_SQUARED:	 (int 26b [0:67108863])	Center ordinate of the
 *							image circle in sensor
 *							pixels value squared
 *  BSTAT_CIRCLE_RADIUS_SQUARED: (int 29b [0:536870911])Squared radius of the
 *							image circle
 *  BSTAT_INC_X_LOG2_INC:	 (int 3b [0:7])		Vertical
 *							skipping/binning
 *							parameter
 *  BSTAT_INC_Y_LOG2_INC:	 (int 3b [0:7])		Horizontal
 *							skipping/binning
 *							parameter
 *  BSTAT_SAT_THRESHOLD:	 (int 10b [0:1023])	Below this threshold a
 *							pixel is considered
 *							unsaturated, a Bayer
 *							block is saturated if
 *							any of its 4 pixels is
 *							saturated
 *  BSTAT_MAX_X_WINDOW_COUNT:	 (int 7b [0:127])	Width of thumbnail
 *  BSTAT_MAX_Y_WINDOW_COUNT:	 (int 7b [0:127])	Height of thumbnail
 *  BSTAT_CFA:			 (int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_BSTAT_BASE			(P7_CID_ISP_DENOISE_BASE + 5)
#define P7_CID_ISP_BSTAT_MEASURE_REQ_CLEAR	(P7_CID_ISP_BSTAT_BASE + 1)
#define P7_CID_ISP_BSTAT_WINDOW_X_WIDTH		(P7_CID_ISP_BSTAT_BASE + 2)
#define P7_CID_ISP_BSTAT_WINDOW_X_OFFSET	(P7_CID_ISP_BSTAT_BASE + 3)
#define P7_CID_ISP_BSTAT_WINDOW_Y_HEIGHT	(P7_CID_ISP_BSTAT_BASE + 4)
#define P7_CID_ISP_BSTAT_WINDOW_Y_OFFSET	(P7_CID_ISP_BSTAT_BASE + 5)
#define P7_CID_ISP_BSTAT_CIRCLE_POS_X_CENTER	(P7_CID_ISP_BSTAT_BASE + 6)
#define P7_CID_ISP_BSTAT_CIRCLE_POS_X_SQUARED	(P7_CID_ISP_BSTAT_BASE + 7)
#define P7_CID_ISP_BSTAT_CIRCLE_POS_Y_CENTER	(P7_CID_ISP_BSTAT_BASE + 8)
#define P7_CID_ISP_BSTAT_CIRCLE_POS_Y_SQUARED	(P7_CID_ISP_BSTAT_BASE + 9)
#define P7_CID_ISP_BSTAT_CIRCLE_RADIUS_SQUARED	(P7_CID_ISP_BSTAT_BASE + 10)
#define P7_CID_ISP_BSTAT_INC_X_LOG2_INC		(P7_CID_ISP_BSTAT_BASE + 11)
#define P7_CID_ISP_BSTAT_INC_Y_LOG2_INC		(P7_CID_ISP_BSTAT_BASE + 12)
#define P7_CID_ISP_BSTAT_SAT_THRESHOLD		(P7_CID_ISP_BSTAT_BASE + 13)
#define P7_CID_ISP_BSTAT_MAX_X_WINDOW_COUNT	(P7_CID_ISP_BSTAT_BASE + 14)
#define P7_CID_ISP_BSTAT_MAX_Y_WINDOW_COUNT	(P7_CID_ISP_BSTAT_BASE + 15)
#define P7_CID_ISP_BSTAT_CFA			(P7_CID_ISP_BSTAT_BASE + 16)

/* Lens Shading Correction ISP module
 *  LSC_OFFSET_X:	 (int 9b [0:511])	Offset of top left pixel from
 *						top left vertice of cell
 *  LSC_OFFSET_Y:	 (int 10b [0:1023])	Offset of top left pixel from
 *						top left vertice of cell
 *  LSC_CELL_ID_X:	 (int 4b [0:15])	Horizontal cell index to which
 *						top left pixel belongs
 *  LSC_CELL_ID_Y:	 (int 4b [0:15])	Vertical cell index to which top
 *						left pixel belongs
 *  LSC_CELL_W:		 (int 9b [0:511])	Mesh cells width in pixel
 *  LSC_CELL_H:		 (int 10b [0:1023])	Mesh cells height in pixel
 *  LSC_CELL_W_INV:	 (int 17b [0:131071])	Mesh cells inverse width
 *  LSC_CELL_H_INV:	 (int 17b [0:131071])	Mesh cells inverse height
 *  LSC_ALPHA:		 (int 17b [0:131071])	Normalised horizontal offset of
 *						top left pixel from top left
 *						vertice of cell
 *  LSC_BETA:		 (int 17b [0:131071])	Normalised vertical offset of
 *						top left pixel from top left
 *						vertice of cell
 *  LSC_THRESHOLD_B:	 (int 10b [0:1023])	Threshold to avoid declipping on
 *						Blue
 *  LSC_THRESHOLD_G:	 (int 10b [0:1023])	Threshold to avoid declipping on
 *						Green
 *  LSC_THRESHOLD_R:	 (int 10b [0:1023])	Threshold to avoid declipping on
 *						Blue
 *  LSC_GAIN_B:		 (int 10b [0:1023])	White balance gain Blue
 *  LSC_GAIN_G:		 (int 10b [0:1023])	White balance gain Green
 *  LSC_GAIN_R:		 (int 10b [0:1023])	White balance gain Red
 *  LSC_RED_COEFF_MEM:	 (ary[221] 8b [0:255])	Correction mesh for Red channel
 *						(array 15x13)
 *  LSC_GREEN_COEFF_MEM: (ary[221] 8b [0:255])	Correction mesh for Green
 *						channel (array 15x13)
 *  LSC_BLUE_COEFF_MEM:	 (ary[221] 8b [0:255])	Correction mesh for Blue channel
 *						(array 15x13)
 *  LSC_CFA:		 (int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_LSC_BASE		(P7_CID_ISP_BSTAT_BASE + 20)
#define P7_CID_ISP_LSC_OFFSET_X		(P7_CID_ISP_LSC_BASE + 1)
#define P7_CID_ISP_LSC_OFFSET_Y		(P7_CID_ISP_LSC_BASE + 2)
#define P7_CID_ISP_LSC_CELL_ID_X	(P7_CID_ISP_LSC_BASE + 3)
#define P7_CID_ISP_LSC_CELL_ID_Y	(P7_CID_ISP_LSC_BASE + 4)
#define P7_CID_ISP_LSC_CELL_W		(P7_CID_ISP_LSC_BASE + 5)
#define P7_CID_ISP_LSC_CELL_H		(P7_CID_ISP_LSC_BASE + 6)
#define P7_CID_ISP_LSC_CELL_W_INV	(P7_CID_ISP_LSC_BASE + 7)
#define P7_CID_ISP_LSC_CELL_H_INV	(P7_CID_ISP_LSC_BASE + 8)
#define P7_CID_ISP_LSC_ALPHA		(P7_CID_ISP_LSC_BASE + 9)
#define P7_CID_ISP_LSC_BETA		(P7_CID_ISP_LSC_BASE + 10)
#define P7_CID_ISP_LSC_THRESHOLD_B	(P7_CID_ISP_LSC_BASE + 11)
#define P7_CID_ISP_LSC_THRESHOLD_G	(P7_CID_ISP_LSC_BASE + 12)
#define P7_CID_ISP_LSC_THRESHOLD_R	(P7_CID_ISP_LSC_BASE + 13)
#define P7_CID_ISP_LSC_GAIN_B		(P7_CID_ISP_LSC_BASE + 14)
#define P7_CID_ISP_LSC_GAIN_G		(P7_CID_ISP_LSC_BASE + 15)
#define P7_CID_ISP_LSC_GAIN_R		(P7_CID_ISP_LSC_BASE + 16)
#define P7_CID_ISP_LSC_RED_COEFF_MEM	(P7_CID_ISP_LSC_BASE + 17)
#define P7_CID_ISP_LSC_GREEN_COEFF_MEM	(P7_CID_ISP_LSC_BASE + 18)
#define P7_CID_ISP_LSC_BLUE_COEFF_MEM	(P7_CID_ISP_LSC_BASE + 19)
#define P7_CID_ISP_LSC_CFA		(P7_CID_ISP_LSC_BASE + 20)

/* Chromatic Aberation ISP module
 *  CA_RADIUS_SQUARED:		(ary[20] 24b [0:16777215])Abscissa of the Look
 *							Up Table
 *  CA_DISPLACEMENT_RED_COEFF:	(ary[20] 16b [0:65535])	Displacement for the
 *							BLUE channel
 *  CA_DISPLACEMENT_BLUE_COEFF:	(ary[20] 16b [0:65535])	Displacement for the
 *							RED channel
 *  CA_CIRCLE_POS_X_CENTER:	(int 14b [0:16383])	Center abcissa of the
 *							image circle in sensor
 *							pixels
 *  CA_CIRCLE_POS_X_SQUARED:	(int 26b [0:67108863])	Center abcissa of the
 *							image circle in sensor
 *							pixels value squared
 *  CA_CIRCLE_POS_Y_CENTER:	(int 14b [0:16383])	Center ordinate of the
 *							image circle in sensor
 *							pixels
 *  CA_CIRCLE_POS_Y_SQUARED:	(int 26b [0:67108863])	Center ordinate of the
 *							image circle in sensor
 *							pixels value squared
 *  CA_GREEN_VARIATION:		(bool)			Green variation is
 *							taken into account for
 *							the correction
 *  CA_X_LOG2_INC:		(int 3b [0:7])		Horizontal
 *							skipping/binning
 *							parameter
 *  CA_Y_LOG2_INC:		(int 3b [0:7])		Vertical
 *							skipping/binning
 *							parameter
 *  CA_CFA:			(int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_CA_BASE			(P7_CID_ISP_LSC_BASE + 25)
#define P7_CID_ISP_CA_RADIUS_SQUARED		(P7_CID_ISP_CA_BASE + 1)
#define P7_CID_ISP_CA_DISPLACEMENT_RED_COEFF	(P7_CID_ISP_CA_BASE + 2)
#define P7_CID_ISP_CA_DISPLACEMENT_BLUE_COEFF	(P7_CID_ISP_CA_BASE + 3)
#define P7_CID_ISP_CA_CIRCLE_POS_X_CENTER	(P7_CID_ISP_CA_BASE + 4)
#define P7_CID_ISP_CA_CIRCLE_POS_X_SQUARED	(P7_CID_ISP_CA_BASE + 5)
#define P7_CID_ISP_CA_CIRCLE_POS_Y_CENTER	(P7_CID_ISP_CA_BASE + 6)
#define P7_CID_ISP_CA_CIRCLE_POS_Y_SQUARED	(P7_CID_ISP_CA_BASE + 7)
#define P7_CID_ISP_CA_GREEN_VARIATION		(P7_CID_ISP_CA_BASE + 8)
#define P7_CID_ISP_CA_X_LOG2_INC		(P7_CID_ISP_CA_BASE + 9)
#define P7_CID_ISP_CA_Y_LOG2_INC		(P7_CID_ISP_CA_BASE + 10)
#define P7_CID_ISP_CA_CFA			(P7_CID_ISP_CA_BASE + 11)

/* Bayer ISP module
 *  BAYER_THRESHOLD_1	(int 13b [0:8191])	Low threshold value
 *  BAYER_THRESHOLD_2	(int 13b [0:8191])	High threshold value
 *  BAYER_CFA:		(int 2b [0:3])		Color Filter Array
 *     see enum p7_isp_cfa for types.
 */
#define P7_CID_ISP_BAYER_BASE		(P7_CID_ISP_CA_BASE + 15)
#define P7_CID_ISP_BAYER_THRESHOLD_1	(P7_CID_ISP_BAYER_BASE + 1)
#define P7_CID_ISP_BAYER_THRESHOLD_2	(P7_CID_ISP_BAYER_BASE + 2)
#define P7_CID_ISP_BAYER_CFA		(P7_CID_ISP_BAYER_BASE + 3)

/* Color correction ISP module
 *  CC_COEFF_00:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_01:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_02:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_10:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_11:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_12:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_20:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_21:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_COEFF_22:	(int 14b [0:16383])	Coded in signed fixed-pointQ2.11
 *  CC_OFFSET_RY_IN:	(int 10b [0:1023])	Red or Y component offset:
 *						before matrix multiplication,
 *						the pixel vector is subtracted
 *						by 'OFFSET_IN' value
 *  CC_OFFSET_RY_OUT:	(int 10b [0:1023])	Red or Y component offsets:
 *						after the matrix multiplication,
 *						the product is added with
 *						'OFFSET_OUT' value
 *  CC_CLIP_RY_MIN:	(int 10b [0:1023])	Red or Y component clips: the
 *						output result is clipped between
 *						'CLIP_MIN' and 'CLIP_MAX' value
 *  CC_CLIP_RY_MAX:	(int 10b [0:1023])	Red or Y component clips: the
 *						output result is clipped between
 *						'CLIP_MIN' and 'CLIP_MAX' value
 *  CC_OFFSET_GU_IN:	(int 10b [0:1023])	Green or U component offset:
 *						before matrix multiplication,
 *						the pixel vector is subtracted
 *						by 'OFFSET_IN' value
 *  CC_OFFSET_GU_OUT:	(int 10b [0:1023])	Green or U component offsets:
 *						after the matrix multiplication,
 *						the product is added with
 *						'OFFSET_OUT' value
 *  CC_CLIP_GU_MIN:	(int 10b [0:1023])	Green or U component clips: the
 *						output result is clipped between
 *						'CLIP_MIN' and 'CLIP_MAX' value
 *  CC_CLIP_GU_MAX:	(int 10b [0:1023])	Green or U component clips: the
 *						output result is clipped between
 *						'CLIP_MIN' and 'CLIP_MAX' value
 *  CC_OFFSET_BV_IN:	(int 10b [0:1023])	Blue or V component offset:
 *						before matrix multiplication,
 *						the pixel vector is subtracted
 *						by 'OFFSET_IN' value
 *  CC_OFFSET_BV_OUT:	(int 10b [0:1023])	Blue or V component offsets:
 *						after the matrix multiplication,
 *						the product is added with
 *						'OFFSET_OUT' value
 *  CC_CLIP_BV_MIN:	(int 10b [0:1023])	Blue or V component clips: the
 *						output result is clipped between
 *						'CLIP_MIN' and 'CLIP_MAX' value
 *  CC_CLIP_BV_MAX:	(int 10b [0:1023])	Blue or V component clips: the
 *						output result is clipped between
 *						'CLIP_MIN' and 'CLIP_MAX' value
 */
#define P7_CID_ISP_CC_BASE		(P7_CID_ISP_BAYER_BASE + 5)
#define P7_CID_ISP_CC_COEFF_00		(P7_CID_ISP_CC_BASE + 1)
#define P7_CID_ISP_CC_COEFF_01		(P7_CID_ISP_CC_BASE + 2)
#define P7_CID_ISP_CC_COEFF_02		(P7_CID_ISP_CC_BASE + 3)
#define P7_CID_ISP_CC_COEFF_10		(P7_CID_ISP_CC_BASE + 4)
#define P7_CID_ISP_CC_COEFF_11		(P7_CID_ISP_CC_BASE + 5)
#define P7_CID_ISP_CC_COEFF_12		(P7_CID_ISP_CC_BASE + 6)
#define P7_CID_ISP_CC_COEFF_20		(P7_CID_ISP_CC_BASE + 7)
#define P7_CID_ISP_CC_COEFF_21		(P7_CID_ISP_CC_BASE + 8)
#define P7_CID_ISP_CC_COEFF_22		(P7_CID_ISP_CC_BASE + 9)
#define P7_CID_ISP_CC_OFFSET_RY_IN	(P7_CID_ISP_CC_BASE + 10)
#define P7_CID_ISP_CC_OFFSET_RY_OUT	(P7_CID_ISP_CC_BASE + 11)
#define P7_CID_ISP_CC_CLIP_RY_MIN	(P7_CID_ISP_CC_BASE + 12)
#define P7_CID_ISP_CC_CLIP_RY_MAX	(P7_CID_ISP_CC_BASE + 13)
#define P7_CID_ISP_CC_OFFSET_GU_IN	(P7_CID_ISP_CC_BASE + 14)
#define P7_CID_ISP_CC_OFFSET_GU_OUT	(P7_CID_ISP_CC_BASE + 15)
#define P7_CID_ISP_CC_CLIP_GU_MIN	(P7_CID_ISP_CC_BASE + 16)
#define P7_CID_ISP_CC_CLIP_GU_MAX	(P7_CID_ISP_CC_BASE + 17)
#define P7_CID_ISP_CC_OFFSET_BV_IN	(P7_CID_ISP_CC_BASE + 18)
#define P7_CID_ISP_CC_OFFSET_BV_OUT	(P7_CID_ISP_CC_BASE + 19)
#define P7_CID_ISP_CC_CLIP_BV_MIN	(P7_CID_ISP_CC_BASE + 20)
#define P7_CID_ISP_CC_CLIP_BV_MAX	(P7_CID_ISP_CC_BASE + 21)

/* VL Format 40 to 32 ISP module
 *  VLF_40_TO_32:	(int 3b [0:4])	Convert format from 40 to 32
 *    see enum p7_vlf_40to32 for types.
 */
#define P7_CID_ISP_VLF_40_TO_32		(P7_CID_ISP_CC_BASE + 25)
enum p7_vlf_40to32 {
	P7_VLF_40_RAW10_TO_RAW8,
	P7_VLF_40_RAW10_TO_RAW10,
	P7_VLF_40_RAW10_TO_RAW3x10,
	P7_VLF_40_RGB3x10_TO_RGB3x10,
	P7_VLF_40_RGB3x10_TO_ARGB,
};

/* Gamma Correction YUV ISP module
 *  GAMMA_BYPASS:	(bool)			Bypass Gamma correction
 *  GAMMA_PALETTE:	(bool)			Palette mode
 *						(0 = non-linear, 1 = palette)
 *  GAMMA_COMP_WIDTH:	(bool)			Component width
 *						(0 = 8bits, 1 = 10bits)
 *  GAMMA_RY_LUT:	(ary[1024] 8b [0:255])	Red or Y curve
 *  GAMMA_GU_LUT:	(ary[1024] 8b [0:255])	Green or U curve
 *  GAMMA_BV_LUT:	(ary[1024] 8b [0:255])	Blue or V curve
 */
#define P7_CID_ISP_GAMMA_BASE		(P7_CID_ISP_CC_BASE + 30)
#define P7_CID_ISP_GAMMA_BYPASS		(P7_CID_ISP_GAMMA_BASE + 1)
#define P7_CID_ISP_GAMMA_PALETTE	(P7_CID_ISP_GAMMA_BASE + 2)
#define P7_CID_ISP_GAMMA_COMP_WIDTH	(P7_CID_ISP_GAMMA_BASE + 3)
#define P7_CID_ISP_GAMMA_RY_LUT		(P7_CID_ISP_GAMMA_BASE + 4)
#define P7_CID_ISP_GAMMA_GU_LUT		(P7_CID_ISP_GAMMA_BASE + 5)
#define P7_CID_ISP_GAMMA_BV_LUT		(P7_CID_ISP_GAMMA_BASE + 6)

/* Chroma converter YUV ISP module
 *  See definition of Color Correction ISP module (P7_CID_ISP_CC_*)
 */
#define P7_CID_ISP_CONV_BASE		(P7_CID_ISP_GAMMA_BASE + 10)
#define P7_CID_ISP_CONV_COEFF_00	(P7_CID_ISP_CONV_BASE + 1)
#define P7_CID_ISP_CONV_COEFF_01	(P7_CID_ISP_CONV_BASE + 2)
#define P7_CID_ISP_CONV_COEFF_02	(P7_CID_ISP_CONV_BASE + 3)
#define P7_CID_ISP_CONV_COEFF_10	(P7_CID_ISP_CONV_BASE + 4)
#define P7_CID_ISP_CONV_COEFF_11	(P7_CID_ISP_CONV_BASE + 5)
#define P7_CID_ISP_CONV_COEFF_12	(P7_CID_ISP_CONV_BASE + 6)
#define P7_CID_ISP_CONV_COEFF_20	(P7_CID_ISP_CONV_BASE + 7)
#define P7_CID_ISP_CONV_COEFF_21	(P7_CID_ISP_CONV_BASE + 8)
#define P7_CID_ISP_CONV_COEFF_22	(P7_CID_ISP_CONV_BASE + 9)
#define P7_CID_ISP_CONV_OFFSET_RY_IN	(P7_CID_ISP_CONV_BASE + 10)
#define P7_CID_ISP_CONV_OFFSET_RY_OUT	(P7_CID_ISP_CONV_BASE + 11)
#define P7_CID_ISP_CONV_CLIP_RY_MIN	(P7_CID_ISP_CONV_BASE + 12)
#define P7_CID_ISP_CONV_CLIP_RY_MAX	(P7_CID_ISP_CONV_BASE + 13)
#define P7_CID_ISP_CONV_OFFSET_GU_IN	(P7_CID_ISP_CONV_BASE + 14)
#define P7_CID_ISP_CONV_OFFSET_GU_OUT	(P7_CID_ISP_CONV_BASE + 15)
#define P7_CID_ISP_CONV_CLIP_GU_MIN	(P7_CID_ISP_CONV_BASE + 16)
#define P7_CID_ISP_CONV_CLIP_GU_MAX	(P7_CID_ISP_CONV_BASE + 17)
#define P7_CID_ISP_CONV_OFFSET_BV_IN	(P7_CID_ISP_CONV_BASE + 18)
#define P7_CID_ISP_CONV_OFFSET_BV_OUT	(P7_CID_ISP_CONV_BASE + 19)
#define P7_CID_ISP_CONV_CLIP_BV_MIN	(P7_CID_ISP_CONV_BASE + 20)
#define P7_CID_ISP_CONV_CLIP_BV_MAX	(P7_CID_ISP_CONV_BASE + 21)

/* Statistics YUV ISP module
 *  YSTAT_MEASURE_REQ:		 (bool)			Will measure at the
 *							beginning of the next
 *							frame
 *  YSTAT_CLEAR:		 (bool)			Clearing the histogram
 *  YSTAT_DONE:			 (bool)			Asserted at the end of
 *							each measured frame
 *  YSTAT_ERROR:		 (bool)			Asserted if there is
 *							valid input stream
 *							during the histogram
 *							clearing process
 *  BSTAT_WINDOW_X_START:	 (int 13b [0:8191])	Beginning of the
 *							measured window X
 *  BSTAT_WINDOW_X_END:		 (int 13b [0:8191])	End of the measured
 *							window X
 *  BSTAT_WINDOW_Y_START:	 (int 13b [0:8191])	Beginning of the
 *							measured window Y
 *  BSTAT_WINDOW_Y_END:		 (int 13b [0:8191])	End of the measured
 *							window Y
 *  YSTAT_CIRCLE_POS_X_CENTER:	 (int 14b [0:16383])	Circle center X
 *  YSTAT_CIRCLE_POS_X_SQUARED:	 (int 26b [0:67108863])	X center squared
 *  YSTAT_CIRCLE_POS_Y_CENTER:	 (int 14b [0:16383])	Circle center Y
 *  YSTAT_CIRCLE_POS_Y_SQUARED:	 (int 26b [0:67108863])	Y center squared
 *  YSTAT_CIRCLE_RADIUS_SQUARED: (int 29b [0:536870911])Circle radius squared
 *  YSTAT_INC_X_LOG2_INC:	 (int 3b [0:7])		Horizontal
 *							skipping/binning
 *							parameter
 *  YSTAT_INC_Y_LOG2_INC:	 (int 3b [0:7])		Vertical
 *							skipping/binning
 *							parameter
 *  YSTAT_AWB_THRESHOLD:	 (int 8b [0:255])	Auto white balancing
 *							threshold
 *
 * Read Only controls:
 *  YSTAT_AE_NB_VALID_Y:	 (int 22b [0:4194303])		Number of valid
 *								luma pixels
 *  YSTAT_AWB_SUM_Y:		 (int 30b [0:1073741823])	White balancing
 *								sum of Y
 *  YSTAT_AWB_SUM_U:		 (int 30b [0:1073741823])	White balancing
 *								sum of U
 *  YSTAT_AWB_SUM_V:		 (int 30b [0:1073741823])	White balancing
 *								sum of V
 *  YSTAT_AWB_NB_GREY_PIXELS:	 (int 22b [0:4194303])		Number of valid
 *								grey pixels
 *  YSTAT_AE_HISTOGRAM_Y:	 (ary[256] 22b [0:4194303])	Value of each
 *								bin
 */
#define P7_CID_ISP_YSTAT_BASE			(P7_CID_ISP_CONV_BASE + 25)
#define P7_CID_ISP_YSTAT_MEASURE_REQ		(P7_CID_ISP_YSTAT_BASE + 1)
#define P7_CID_ISP_YSTAT_CLEAR			(P7_CID_ISP_YSTAT_BASE + 2)
#define P7_CID_ISP_YSTAT_DONE			(P7_CID_ISP_YSTAT_BASE + 3)
#define P7_CID_ISP_YSTAT_ERROR			(P7_CID_ISP_YSTAT_BASE + 4)
#define P7_CID_ISP_BSTAT_WINDOW_X_START		(P7_CID_ISP_YSTAT_BASE + 5)
#define P7_CID_ISP_BSTAT_WINDOW_X_END		(P7_CID_ISP_YSTAT_BASE + 6)
#define P7_CID_ISP_BSTAT_WINDOW_Y_START		(P7_CID_ISP_YSTAT_BASE + 7)
#define P7_CID_ISP_BSTAT_WINDOW_Y_END		(P7_CID_ISP_YSTAT_BASE + 8)
#define P7_CID_ISP_YSTAT_CIRCLE_POS_X_CENTER	(P7_CID_ISP_YSTAT_BASE + 9)
#define P7_CID_ISP_YSTAT_CIRCLE_POS_X_SQUARED	(P7_CID_ISP_YSTAT_BASE + 10)
#define P7_CID_ISP_YSTAT_CIRCLE_POS_Y_CENTER	(P7_CID_ISP_YSTAT_BASE + 11)
#define P7_CID_ISP_YSTAT_CIRCLE_POS_Y_SQUARED	(P7_CID_ISP_YSTAT_BASE + 12)
#define P7_CID_ISP_YSTAT_CIRCLE_RADIUS_SQUARED	(P7_CID_ISP_YSTAT_BASE + 13)
#define P7_CID_ISP_YSTAT_INC_X_LOG2_INC		(P7_CID_ISP_YSTAT_BASE + 14)
#define P7_CID_ISP_YSTAT_INC_Y_LOG2_INC		(P7_CID_ISP_YSTAT_BASE + 15)
#define P7_CID_ISP_YSTAT_AE_NB_VALID_Y		(P7_CID_ISP_YSTAT_BASE + 16)
#define P7_CID_ISP_YSTAT_AWB_THRESHOLD		(P7_CID_ISP_YSTAT_BASE + 17)
#define P7_CID_ISP_YSTAT_AWB_SUM_Y		(P7_CID_ISP_YSTAT_BASE + 18)
#define P7_CID_ISP_YSTAT_AWB_SUM_U		(P7_CID_ISP_YSTAT_BASE + 19)
#define P7_CID_ISP_YSTAT_AWB_SUM_V		(P7_CID_ISP_YSTAT_BASE + 20)
#define P7_CID_ISP_YSTAT_AWB_NB_GREY_PIXELS	(P7_CID_ISP_YSTAT_BASE + 21)
#define P7_CID_ISP_YSTAT_AE_HISTOGRAM_Y		(P7_CID_ISP_YSTAT_BASE + 22)

/* Edge enhancement color reduction filter ISP module
 *  EE_CRF_EE_LUT:		(ary[256] 6b [0:63])	EE LUT table
 *  EE_CRF_EE_KERNEL_COEFF:	(ary[6] 11b [0:2047])	EE kernel
 *  EE_CRF_CRF_KERNEL_COEFF:	(ary[6] 11b [0:2047])	CRF kernel
 *  EE_CRF_M_COEFF:		(int 8b [0:255])	Sobel threshold
 *  EE_CRF_D_COEFF:		(int 11b [0:2047])	Normalization factor
 */
#define P7_CID_ISP_EE_CRF_BASE			(P7_CID_ISP_YSTAT_BASE + 25)
#define P7_CID_ISP_EE_CRF_EE_LUT		(P7_CID_ISP_EE_CRF_BASE + 1)
#define P7_CID_ISP_EE_CRF_EE_KERNEL_COEFF	(P7_CID_ISP_EE_CRF_BASE + 2)
#define P7_CID_ISP_EE_CRF_CRF_KERNEL_COEFF	(P7_CID_ISP_EE_CRF_BASE + 3)
#define P7_CID_ISP_EE_CRF_M_COEFF		(P7_CID_ISP_EE_CRF_BASE + 4)
#define P7_CID_ISP_EE_CRF_D_COEFF		(P7_CID_ISP_EE_CRF_BASE + 5)

/* I3D Lut ISP module
 *  I3D_LUT_OUTSIDE_RY:	(ary[125] 8b [0:255])	Red or Y component value
 *  I3D_LUT_OUTSIDE_GU:	(ary[125] 8b [0:255])	Green or U component value
 *  I3D_LUT_OUTSIDE_BV:	(ary[125] 8b [0:255])	Blue or V component value
 *  I3D_LUT_INSIDE_RY:	(ary[125] 8b [0:255])	Red or Y component value
 *  I3D_LUT_INSIDE_GU:	(ary[125] 8b [0:255])	Green or U component value
 *  I3D_LUT_INSIDE_BV:	(ary[125] 8b [0:255])	Blue or V component value
 *  I3D_LUT_CLIP_MODE:	(bool)			YUV Clip
 */
#define P7_CID_ISP_I3D_LUT_BASE		(P7_CID_ISP_EE_CRF_BASE + 10)
#define P7_CID_ISP_I3D_LUT_OUTSIDE_RY	(P7_CID_ISP_I3D_LUT_BASE + 1)
#define P7_CID_ISP_I3D_LUT_OUTSIDE_GU	(P7_CID_ISP_I3D_LUT_BASE + 2)
#define P7_CID_ISP_I3D_LUT_OUTSIDE_BV	(P7_CID_ISP_I3D_LUT_BASE + 3)
#define P7_CID_ISP_I3D_LUT_INSIDE_RY	(P7_CID_ISP_I3D_LUT_BASE + 4)
#define P7_CID_ISP_I3D_LUT_INSIDE_GU	(P7_CID_ISP_I3D_LUT_BASE + 5)
#define P7_CID_ISP_I3D_LUT_INSIDE_BV	(P7_CID_ISP_I3D_LUT_BASE + 6)
#define P7_CID_ISP_I3D_LUT_CLIP_MODE	(P7_CID_ISP_I3D_LUT_BASE + 7)

/* Drop ISP module
 *  DROP_OFFSET_X:	(int 16b [0:65535])	Horizontal offset
 *  DROP_OFFSET_Y:	(int 16b [0:65535])	Vertical offset
 */
#define P7_CID_ISP_DROP_BASE		(P7_CID_ISP_I3D_LUT_BASE + 10)
#define P7_CID_ISP_DROP_OFFSET_X	(P7_CID_ISP_DROP_BASE + 1)
#define P7_CID_ISP_DROP_OFFSET_Y	(P7_CID_ISP_DROP_BASE + 2)

#endif /* _AVI_V4L2_ISP_CTRLS_H_ */
