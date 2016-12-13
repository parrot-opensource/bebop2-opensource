/*
 *      linux/drivers/parrot/video/ov16825_ctrl.c
 *
 *      Copyright (C) 2014 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    20-Oct-2014
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

#include <linux/i2c.h>
#include <media/v4l2-ctrls.h>
#include "ov16825_ctrl.h"

#define OV16825_CUSTOM_CID(_n) (V4L2_CID_CAMERA_CLASS_BASE + 0x100 + (_n))

#define V4L2_CID_OV16825_ISP_EN            OV16825_CUSTOM_CID(0)
#define V4L2_CID_OV16825_AVERAGE_EN        OV16825_CUSTOM_CID(1)
#define V4L2_CID_OV16825_WHITE_DPC_EN      OV16825_CUSTOM_CID(2)
#define V4L2_CID_OV16825_BLACK_DPC_EN      OV16825_CUSTOM_CID(3)
#define V4L2_CID_OV16825_WB_GAIN_EN        OV16825_CUSTOM_CID(4)
#define V4L2_CID_OV16825_OTP_CC_EN         OV16825_CUSTOM_CID(5)
#define V4L2_CID_OV16825_DBC_EN            OV16825_CUSTOM_CID(6)
#define V4L2_CID_OV16825_SCALE_EN          OV16825_CUSTOM_CID(7)
#define V4L2_CID_OV16825_BLC_EN            OV16825_CUSTOM_CID(8)
#define V4L2_CID_OV16825_AVERAGE_BEFORE    OV16825_CUSTOM_CID(9)
#define V4L2_CID_OV16825_DIGITAL_GAIN_EN   OV16825_CUSTOM_CID(10)
#define V4L2_CID_OV16825_RED_GAIN       OV16825_CUSTOM_CID(11)
#define V4L2_CID_OV16825_GREEN_GAIN     OV16825_CUSTOM_CID(12)
#define V4L2_CID_OV16825_BLUE_GAIN      OV16825_CUSTOM_CID(13)



static u8 const ov16825_test_pattern_registers[] = {
       PATTERN_DISABLE,
       PATTERN_BLACK,
       PATTERN_COLOR_BAR,
       PATTERN_COLOR_BAR_DARK_TOP_BOTTOM,
       PATTERN_COLOR_BAR_DARK_LEFT_RIGHT,
       PATTERN_COLOR_BAR_DARK_BOTTOM_TOP,
       PATTERN_SQUARE_BW,
       PATTERN_SQUARE_COLOR,
       PATTERN_RANDOM,
};

static const char * const ov16825_test_pattern_menu[] = {
       "Disabled",
       "Black image",
       "Color bar standard",
       "Color bar top-bottom dark",
       "Color bar left-right dark",
       "Color bar bottom-top dark",
       "Square Black-White",
       "Square Color",
       "Random",
};

static struct ov16825_gain {
	u8 major;
	u8 minor;
	/* Gain multiplied by 256 */
	u32 gain;
} ov16825_gain_table[] = {
#define GAIN(v) ((u32)(v * 256))
	{ 0x0, 0x80, GAIN(1.0000) },
	{ 0x0, 0x81, GAIN(1.0078) },
	{ 0x0, 0x82, GAIN(1.0156) },
	{ 0x0, 0x83, GAIN(1.0234) },
	{ 0x0, 0x84, GAIN(1.0313) },
	{ 0x0, 0x85, GAIN(1.0391) },
	{ 0x0, 0x86, GAIN(1.0469) },
	{ 0x0, 0x87, GAIN(1.0547) },
	{ 0x0, 0x88, GAIN(1.0625) },
	{ 0x0, 0x89, GAIN(1.0703) },
	{ 0x0, 0x8A, GAIN(1.0781) },
	{ 0x0, 0x8B, GAIN(1.0859) },
	{ 0x0, 0x8C, GAIN(1.0938) },
	{ 0x0, 0x8D, GAIN(1.1016) },
	{ 0x0, 0x8E, GAIN(1.1094) },
	{ 0x0, 0x8F, GAIN(1.1172) },
	{ 0x0, 0x90, GAIN(1.1250) },
	{ 0x0, 0x91, GAIN(1.1328) },
	{ 0x0, 0x92, GAIN(1.1406) },
	{ 0x0, 0x93, GAIN(1.1484) },
	{ 0x0, 0x94, GAIN(1.1563) },
	{ 0x0, 0x95, GAIN(1.1641) },
	{ 0x0, 0x96, GAIN(1.1719) },
	{ 0x0, 0x97, GAIN(1.1797) },
	{ 0x0, 0x98, GAIN(1.1875) },
	{ 0x0, 0x99, GAIN(1.1953) },
	{ 0x0, 0x9A, GAIN(1.2031) },
	{ 0x0, 0x9B, GAIN(1.2109) },
	{ 0x0, 0x9C, GAIN(1.2188) },
	{ 0x0, 0x9D, GAIN(1.2266) },
	{ 0x0, 0x9E, GAIN(1.2344) },
	{ 0x0, 0x9F, GAIN(1.2422) },
	{ 0x0, 0xA0, GAIN(1.2500) },
	{ 0x0, 0xA1, GAIN(1.2578) },
	{ 0x0, 0xA2, GAIN(1.2656) },
	{ 0x0, 0xA3, GAIN(1.2734) },
	{ 0x0, 0xA4, GAIN(1.2813) },
	{ 0x0, 0xA5, GAIN(1.2891) },
	{ 0x0, 0xA6, GAIN(1.2969) },
	{ 0x0, 0xA7, GAIN(1.3047) },
	{ 0x0, 0xA8, GAIN(1.3125) },
	{ 0x0, 0xA9, GAIN(1.3203) },
	{ 0x0, 0xAA, GAIN(1.3281) },
	{ 0x0, 0xAB, GAIN(1.3359) },
	{ 0x0, 0xAC, GAIN(1.3438) },
	{ 0x0, 0xAD, GAIN(1.3516) },
	{ 0x0, 0xAE, GAIN(1.3594) },
	{ 0x0, 0xAF, GAIN(1.3672) },
	{ 0x0, 0xB0, GAIN(1.3750) },
	{ 0x0, 0xB1, GAIN(1.3828) },
	{ 0x0, 0xB2, GAIN(1.3906) },
	{ 0x0, 0xB3, GAIN(1.3984) },
	{ 0x0, 0xB4, GAIN(1.4063) },
	{ 0x0, 0xB5, GAIN(1.4141) },
	{ 0x0, 0xB6, GAIN(1.4219) },
	{ 0x0, 0xB7, GAIN(1.4297) },
	{ 0x0, 0xB8, GAIN(1.4375) },
	{ 0x0, 0xB9, GAIN(1.4453) },
	{ 0x0, 0xBA, GAIN(1.4531) },
	{ 0x0, 0xBB, GAIN(1.4609) },
	{ 0x0, 0xBC, GAIN(1.4688) },
	{ 0x0, 0xBD, GAIN(1.4766) },
	{ 0x0, 0xBE, GAIN(1.4844) },
	{ 0x0, 0xBF, GAIN(1.4922) },
	{ 0x0, 0xC0, GAIN(1.5000) },
	{ 0x0, 0xC1, GAIN(1.5078) },
	{ 0x0, 0xC2, GAIN(1.5156) },
	{ 0x0, 0xC3, GAIN(1.5234) },
	{ 0x0, 0xC4, GAIN(1.5313) },
	{ 0x0, 0xC5, GAIN(1.5391) },
	{ 0x0, 0xC6, GAIN(1.5469) },
	{ 0x0, 0xC7, GAIN(1.5547) },
	{ 0x0, 0xC8, GAIN(1.5625) },
	{ 0x0, 0xC9, GAIN(1.5703) },
	{ 0x0, 0xCA, GAIN(1.5781) },
	{ 0x0, 0xCB, GAIN(1.5859) },
	{ 0x0, 0xCC, GAIN(1.5938) },
	{ 0x0, 0xCD, GAIN(1.6016) },
	{ 0x0, 0xCE, GAIN(1.6094) },
	{ 0x0, 0xCF, GAIN(1.6172) },
	{ 0x0, 0xD0, GAIN(1.6250) },
	{ 0x0, 0xD1, GAIN(1.6328) },
	{ 0x0, 0xD2, GAIN(1.6406) },
	{ 0x0, 0xD3, GAIN(1.6484) },
	{ 0x0, 0xD4, GAIN(1.6563) },
	{ 0x0, 0xD5, GAIN(1.6641) },
	{ 0x0, 0xD6, GAIN(1.6719) },
	{ 0x0, 0xD7, GAIN(1.6797) },
	{ 0x0, 0xD8, GAIN(1.6875) },
	{ 0x0, 0xD9, GAIN(1.6953) },
	{ 0x0, 0xDA, GAIN(1.7031) },
	{ 0x0, 0xDB, GAIN(1.7109) },
	{ 0x0, 0xDC, GAIN(1.7188) },
	{ 0x0, 0xDD, GAIN(1.7266) },
	{ 0x0, 0xDE, GAIN(1.7344) },
	{ 0x0, 0xDF, GAIN(1.7422) },
	{ 0x0, 0xE0, GAIN(1.7500) },
	{ 0x0, 0xE1, GAIN(1.7578) },
	{ 0x0, 0xE2, GAIN(1.7656) },
	{ 0x0, 0xE3, GAIN(1.7734) },
	{ 0x0, 0xE4, GAIN(1.7813) },
	{ 0x0, 0xE5, GAIN(1.7891) },
	{ 0x0, 0xE6, GAIN(1.7969) },
	{ 0x0, 0xE7, GAIN(1.8047) },
	{ 0x0, 0xE8, GAIN(1.8125) },
	{ 0x0, 0xE9, GAIN(1.8203) },
	{ 0x0, 0xEA, GAIN(1.8281) },
	{ 0x0, 0xEB, GAIN(1.8359) },
	{ 0x0, 0xEC, GAIN(1.8438) },
	{ 0x0, 0xED, GAIN(1.8516) },
	{ 0x0, 0xEE, GAIN(1.8594) },
	{ 0x0, 0xEF, GAIN(1.8672) },
	{ 0x0, 0xF0, GAIN(1.8750) },
	{ 0x0, 0xF1, GAIN(1.8828) },
	{ 0x0, 0xF2, GAIN(1.8906) },
	{ 0x0, 0xF3, GAIN(1.8984) },
	{ 0x0, 0xF4, GAIN(1.9063) },
	{ 0x0, 0xF5, GAIN(1.9141) },
	{ 0x0, 0xF6, GAIN(1.9219) },
	{ 0x0, 0xF7, GAIN(1.9297) },
	{ 0x0, 0xF8, GAIN(1.9375) },
	{ 0x0, 0xF9, GAIN(1.9453) },
	{ 0x0, 0xFA, GAIN(1.9531) },
	{ 0x0, 0xFB, GAIN(1.9609) },
	{ 0x0, 0xFC, GAIN(1.9688) },
	{ 0x0, 0xFD, GAIN(1.9766) },
	{ 0x0, 0xFE, GAIN(1.9844) },
	{ 0x0, 0xFF, GAIN(1.9922) },
	{ 0x4, 0x80, GAIN(2.0000) },
	{ 0x4, 0x81, GAIN(2.0156) },
	{ 0x4, 0x82, GAIN(2.0313) },
	{ 0x4, 0x83, GAIN(2.0469) },
	{ 0x4, 0x84, GAIN(2.0625) },
	{ 0x4, 0x85, GAIN(2.0781) },
	{ 0x4, 0x86, GAIN(2.0938) },
	{ 0x4, 0x87, GAIN(2.1094) },
	{ 0x4, 0x88, GAIN(2.1250) },
	{ 0x4, 0x89, GAIN(2.1406) },
	{ 0x4, 0x8A, GAIN(2.1563) },
	{ 0x4, 0x8B, GAIN(2.1719) },
	{ 0x4, 0x8C, GAIN(2.1875) },
	{ 0x4, 0x8D, GAIN(2.2031) },
	{ 0x4, 0x8E, GAIN(2.2188) },
	{ 0x4, 0x8F, GAIN(2.2344) },
	{ 0x4, 0x90, GAIN(2.2500) },
	{ 0x4, 0x91, GAIN(2.2656) },
	{ 0x4, 0x92, GAIN(2.2813) },
	{ 0x4, 0x93, GAIN(2.2969) },
	{ 0x4, 0x94, GAIN(2.3125) },
	{ 0x4, 0x95, GAIN(2.3281) },
	{ 0x4, 0x96, GAIN(2.3438) },
	{ 0x4, 0x97, GAIN(2.3594) },
	{ 0x4, 0x98, GAIN(2.3750) },
	{ 0x4, 0x99, GAIN(2.3906) },
	{ 0x4, 0x9A, GAIN(2.4063) },
	{ 0x4, 0x9B, GAIN(2.4219) },
	{ 0x4, 0x9C, GAIN(2.4375) },
	{ 0x4, 0x9D, GAIN(2.4531) },
	{ 0x4, 0x9E, GAIN(2.4688) },
	{ 0x4, 0x9F, GAIN(2.4844) },
	{ 0x4, 0xA0, GAIN(2.5000) },
	{ 0x4, 0xA1, GAIN(2.5156) },
	{ 0x4, 0xA2, GAIN(2.5313) },
	{ 0x4, 0xA3, GAIN(2.5469) },
	{ 0x4, 0xA4, GAIN(2.5625) },
	{ 0x4, 0xA5, GAIN(2.5781) },
	{ 0x4, 0xA6, GAIN(2.5938) },
	{ 0x4, 0xA7, GAIN(2.6094) },
	{ 0x4, 0xA8, GAIN(2.6250) },
	{ 0x4, 0xA9, GAIN(2.6406) },
	{ 0x4, 0xAA, GAIN(2.6563) },
	{ 0x4, 0xAB, GAIN(2.6719) },
	{ 0x4, 0xAC, GAIN(2.6875) },
	{ 0x4, 0xAD, GAIN(2.7031) },
	{ 0x4, 0xAE, GAIN(2.7188) },
	{ 0x4, 0xAF, GAIN(2.7344) },
	{ 0x4, 0xB0, GAIN(2.7500) },
	{ 0x4, 0xB1, GAIN(2.7656) },
	{ 0x4, 0xB2, GAIN(2.7813) },
	{ 0x4, 0xB3, GAIN(2.7969) },
	{ 0x4, 0xB4, GAIN(2.8125) },
	{ 0x4, 0xB5, GAIN(2.8281) },
	{ 0x4, 0xB6, GAIN(2.8438) },
	{ 0x4, 0xB7, GAIN(2.8594) },
	{ 0x4, 0xB8, GAIN(2.8750) },
	{ 0x4, 0xB9, GAIN(2.8906) },
	{ 0x4, 0xBA, GAIN(2.9063) },
	{ 0x4, 0xBB, GAIN(2.9219) },
	{ 0x4, 0xBC, GAIN(2.9375) },
	{ 0x4, 0xBD, GAIN(2.9531) },
	{ 0x4, 0xBE, GAIN(2.9688) },
	{ 0x4, 0xBF, GAIN(2.9844) },
	{ 0x4, 0xC0, GAIN(3.0000) },
	{ 0x4, 0xC1, GAIN(3.0156) },
	{ 0x4, 0xC2, GAIN(3.0313) },
	{ 0x4, 0xC3, GAIN(3.0469) },
	{ 0x4, 0xC4, GAIN(3.0625) },
	{ 0x4, 0xC5, GAIN(3.0781) },
	{ 0x4, 0xC6, GAIN(3.0938) },
	{ 0x4, 0xC7, GAIN(3.1094) },
	{ 0x4, 0xC8, GAIN(3.1250) },
	{ 0x4, 0xC9, GAIN(3.1406) },
	{ 0x4, 0xCA, GAIN(3.1563) },
	{ 0x4, 0xCB, GAIN(3.1719) },
	{ 0x4, 0xCC, GAIN(3.1875) },
	{ 0x4, 0xCD, GAIN(3.2031) },
	{ 0x4, 0xCE, GAIN(3.2188) },
	{ 0x4, 0xCF, GAIN(3.2344) },
	{ 0x4, 0xD0, GAIN(3.2500) },
	{ 0x4, 0xD1, GAIN(3.2656) },
	{ 0x4, 0xD2, GAIN(3.2813) },
	{ 0x4, 0xD3, GAIN(3.2969) },
	{ 0x4, 0xD4, GAIN(3.3125) },
	{ 0x4, 0xD5, GAIN(3.3281) },
	{ 0x4, 0xD6, GAIN(3.3438) },
	{ 0x4, 0xD7, GAIN(3.3594) },
	{ 0x4, 0xD8, GAIN(3.3750) },
	{ 0x4, 0xD9, GAIN(3.3906) },
	{ 0x4, 0xDA, GAIN(3.4063) },
	{ 0x4, 0xDB, GAIN(3.4219) },
	{ 0x4, 0xDC, GAIN(3.4375) },
	{ 0x4, 0xDD, GAIN(3.4531) },
	{ 0x4, 0xDE, GAIN(3.4688) },
	{ 0x4, 0xDF, GAIN(3.4844) },
	{ 0x4, 0xE0, GAIN(3.5000) },
	{ 0x4, 0xE1, GAIN(3.5156) },
	{ 0x4, 0xE2, GAIN(3.5313) },
	{ 0x4, 0xE3, GAIN(3.5469) },
	{ 0x4, 0xE4, GAIN(3.5625) },
	{ 0x4, 0xE5, GAIN(3.5781) },
	{ 0x4, 0xE6, GAIN(3.5938) },
	{ 0x4, 0xE7, GAIN(3.6094) },
	{ 0x4, 0xE8, GAIN(3.6250) },
	{ 0x4, 0xE9, GAIN(3.6406) },
	{ 0x4, 0xEA, GAIN(3.6563) },
	{ 0x4, 0xEB, GAIN(3.6719) },
	{ 0x4, 0xEC, GAIN(3.6875) },
	{ 0x4, 0xED, GAIN(3.7031) },
	{ 0x4, 0xEE, GAIN(3.7188) },
	{ 0x4, 0xEF, GAIN(3.7344) },
	{ 0x4, 0xF0, GAIN(3.7500) },
	{ 0x4, 0xF1, GAIN(3.7656) },
	{ 0x4, 0xF2, GAIN(3.7813) },
	{ 0x4, 0xF3, GAIN(3.7969) },
	{ 0x4, 0xF4, GAIN(3.8125) },
	{ 0x4, 0xF5, GAIN(3.8281) },
	{ 0x4, 0xF6, GAIN(3.8438) },
	{ 0x4, 0xF7, GAIN(3.8594) },
	{ 0x4, 0xF8, GAIN(3.8750) },
	{ 0x4, 0xF9, GAIN(3.8906) },
	{ 0x4, 0xFA, GAIN(3.9063) },
	{ 0x4, 0xFB, GAIN(3.9219) },
	{ 0x4, 0xFC, GAIN(3.9375) },
	{ 0x4, 0xFD, GAIN(3.9531) },
	{ 0x4, 0xFE, GAIN(3.9688) },
	{ 0x4, 0xFF, GAIN(3.9844) },
	{ 0x8, 0x80, GAIN(4.0000) },
	{ 0x8, 0x81, GAIN(4.0313) },
	{ 0x8, 0x82, GAIN(4.0625) },
	{ 0x8, 0x83, GAIN(4.0938) },
	{ 0x8, 0x84, GAIN(4.1250) },
	{ 0x8, 0x85, GAIN(4.1563) },
	{ 0x8, 0x86, GAIN(4.1875) },
	{ 0x8, 0x87, GAIN(4.2188) },
	{ 0x8, 0x88, GAIN(4.2500) },
	{ 0x8, 0x89, GAIN(4.2813) },
	{ 0x8, 0x8A, GAIN(4.3125) },
	{ 0x8, 0x8B, GAIN(4.3438) },
	{ 0x8, 0x8C, GAIN(4.3750) },
	{ 0x8, 0x8D, GAIN(4.4063) },
	{ 0x8, 0x8E, GAIN(4.4375) },
	{ 0x8, 0x8F, GAIN(4.4688) },
	{ 0x8, 0x90, GAIN(4.5000) },
	{ 0x8, 0x91, GAIN(4.5313) },
	{ 0x8, 0x92, GAIN(4.5625) },
	{ 0x8, 0x93, GAIN(4.5938) },
	{ 0x8, 0x94, GAIN(4.6250) },
	{ 0x8, 0x95, GAIN(4.6563) },
	{ 0x8, 0x96, GAIN(4.6875) },
	{ 0x8, 0x97, GAIN(4.7188) },
	{ 0x8, 0x98, GAIN(4.7500) },
	{ 0x8, 0x99, GAIN(4.7813) },
	{ 0x8, 0x9A, GAIN(4.8125) },
	{ 0x8, 0x9B, GAIN(4.8438) },
	{ 0x8, 0x9C, GAIN(4.8750) },
	{ 0x8, 0x9D, GAIN(4.9063) },
	{ 0x8, 0x9E, GAIN(4.9375) },
	{ 0x8, 0x9F, GAIN(4.9688) },
	{ 0x8, 0xA0, GAIN(5.0000) },
	{ 0x8, 0xA1, GAIN(5.0313) },
	{ 0x8, 0xA2, GAIN(5.0625) },
	{ 0x8, 0xA3, GAIN(5.0938) },
	{ 0x8, 0xA4, GAIN(5.1250) },
	{ 0x8, 0xA5, GAIN(5.1563) },
	{ 0x8, 0xA6, GAIN(5.1875) },
	{ 0x8, 0xA7, GAIN(5.2188) },
	{ 0x8, 0xA8, GAIN(5.2500) },
	{ 0x8, 0xA9, GAIN(5.2813) },
	{ 0x8, 0xAA, GAIN(5.3125) },
	{ 0x8, 0xAB, GAIN(5.3438) },
	{ 0x8, 0xAC, GAIN(5.3750) },
	{ 0x8, 0xAD, GAIN(5.4063) },
	{ 0x8, 0xAE, GAIN(5.4375) },
	{ 0x8, 0xAF, GAIN(5.4688) },
	{ 0x8, 0xB0, GAIN(5.5000) },
	{ 0x8, 0xB1, GAIN(5.5313) },
	{ 0x8, 0xB2, GAIN(5.5625) },
	{ 0x8, 0xB3, GAIN(5.5938) },
	{ 0x8, 0xB4, GAIN(5.6250) },
	{ 0x8, 0xB5, GAIN(5.6563) },
	{ 0x8, 0xB6, GAIN(5.6875) },
	{ 0x8, 0xB7, GAIN(5.7188) },
	{ 0x8, 0xB8, GAIN(5.7500) },
	{ 0x8, 0xB9, GAIN(5.7813) },
	{ 0x8, 0xBA, GAIN(5.8125) },
	{ 0x8, 0xBB, GAIN(5.8438) },
	{ 0x8, 0xBC, GAIN(5.8750) },
	{ 0x8, 0xBD, GAIN(5.9063) },
	{ 0x8, 0xBE, GAIN(5.9375) },
	{ 0x8, 0xBF, GAIN(5.9688) },
	{ 0x8, 0xC0, GAIN(6.0000) },
	{ 0x8, 0xC1, GAIN(6.0313) },
	{ 0x8, 0xC2, GAIN(6.0625) },
	{ 0x8, 0xC3, GAIN(6.0938) },
	{ 0x8, 0xC4, GAIN(6.1250) },
	{ 0x8, 0xC5, GAIN(6.1563) },
	{ 0x8, 0xC6, GAIN(6.1875) },
	{ 0x8, 0xC7, GAIN(6.2188) },
	{ 0x8, 0xC8, GAIN(6.2500) },
	{ 0x8, 0xC9, GAIN(6.2813) },
	{ 0x8, 0xCA, GAIN(6.3125) },
	{ 0x8, 0xCB, GAIN(6.3438) },
	{ 0x8, 0xCC, GAIN(6.3750) },
	{ 0x8, 0xCD, GAIN(6.4063) },
	{ 0x8, 0xCE, GAIN(6.4375) },
	{ 0x8, 0xCF, GAIN(6.4688) },
	{ 0x8, 0xD0, GAIN(6.5000) },
	{ 0x8, 0xD1, GAIN(6.5313) },
	{ 0x8, 0xD2, GAIN(6.5625) },
	{ 0x8, 0xD3, GAIN(6.5938) },
	{ 0x8, 0xD4, GAIN(6.6250) },
	{ 0x8, 0xD5, GAIN(6.6563) },
	{ 0x8, 0xD6, GAIN(6.6875) },
	{ 0x8, 0xD7, GAIN(6.7188) },
	{ 0x8, 0xD8, GAIN(6.7500) },
	{ 0x8, 0xD9, GAIN(6.7813) },
	{ 0x8, 0xDA, GAIN(6.8125) },
	{ 0x8, 0xDB, GAIN(6.8438) },
	{ 0x8, 0xDC, GAIN(6.8750) },
	{ 0x8, 0xDD, GAIN(6.9063) },
	{ 0x8, 0xDE, GAIN(6.9375) },
	{ 0x8, 0xDF, GAIN(6.9688) },
	{ 0x8, 0xE0, GAIN(7.0000) },
	{ 0x8, 0xE1, GAIN(7.0313) },
	{ 0x8, 0xE2, GAIN(7.0625) },
	{ 0x8, 0xE3, GAIN(7.0938) },
	{ 0x8, 0xE4, GAIN(7.1250) },
	{ 0x8, 0xE5, GAIN(7.1563) },
	{ 0x8, 0xE6, GAIN(7.1875) },
	{ 0x8, 0xE7, GAIN(7.2188) },
	{ 0x8, 0xE8, GAIN(7.2500) },
	{ 0x8, 0xE9, GAIN(7.2813) },
	{ 0x8, 0xEA, GAIN(7.3125) },
	{ 0x8, 0xEB, GAIN(7.3438) },
	{ 0x8, 0xEC, GAIN(7.3750) },
	{ 0x8, 0xED, GAIN(7.4063) },
	{ 0x8, 0xEE, GAIN(7.4375) },
	{ 0x8, 0xEF, GAIN(7.4688) },
	{ 0x8, 0xF0, GAIN(7.5000) },
	{ 0x8, 0xF1, GAIN(7.5313) },
	{ 0x8, 0xF2, GAIN(7.5625) },
	{ 0x8, 0xF3, GAIN(7.5938) },
	{ 0x8, 0xF4, GAIN(7.6250) },
	{ 0x8, 0xF5, GAIN(7.6563) },
	{ 0x8, 0xF6, GAIN(7.6875) },
	{ 0x8, 0xF7, GAIN(7.7188) },
	{ 0x8, 0xF8, GAIN(7.7500) },
	{ 0x8, 0xF9, GAIN(7.7813) },
	{ 0x8, 0xFA, GAIN(7.8125) },
	{ 0x8, 0xFB, GAIN(7.8438) },
	{ 0x8, 0xFC, GAIN(7.8750) },
	{ 0x8, 0xFD, GAIN(7.9063) },
	{ 0x8, 0xFE, GAIN(7.9375) },
	{ 0x8, 0xFF, GAIN(7.9688) },
	{ 0xC, 0x80, GAIN(8.0000) },
	{ 0xC, 0x81, GAIN(8.0625) },
	{ 0xC, 0x82, GAIN(8.1250) },
	{ 0xC, 0x83, GAIN(8.1875) },
	{ 0xC, 0x84, GAIN(8.2500) },
	{ 0xC, 0x85, GAIN(8.3125) },
	{ 0xC, 0x86, GAIN(8.3750) },
	{ 0xC, 0x87, GAIN(8.4375) },
	{ 0xC, 0x88, GAIN(8.5000) },
	{ 0xC, 0x89, GAIN(8.5625) },
	{ 0xC, 0x8A, GAIN(8.6250) },
	{ 0xC, 0x8B, GAIN(8.6875) },
	{ 0xC, 0x8C, GAIN(8.7500) },
	{ 0xC, 0x8D, GAIN(8.8125) },
	{ 0xC, 0x8E, GAIN(8.8750) },
	{ 0xC, 0x8F, GAIN(8.9375) },
	{ 0xC, 0x90, GAIN(9.0000) },
	{ 0xC, 0x91, GAIN(9.0625) },
	{ 0xC, 0x92, GAIN(9.1250) },
	{ 0xC, 0x93, GAIN(9.1875) },
	{ 0xC, 0x94, GAIN(9.2500) },
	{ 0xC, 0x95, GAIN(9.3125) },
	{ 0xC, 0x96, GAIN(9.3750) },
	{ 0xC, 0x97, GAIN(9.4375) },
	{ 0xC, 0x98, GAIN(9.5000) },
	{ 0xC, 0x99, GAIN(9.5625) },
	{ 0xC, 0x9A, GAIN(9.6250) },
	{ 0xC, 0x9B, GAIN(9.6875) },
	{ 0xC, 0x9C, GAIN(9.7500) },
	{ 0xC, 0x9D, GAIN(9.8125) },
	{ 0xC, 0x9E, GAIN(9.8750) },
	{ 0xC, 0x9F, GAIN(9.9375) },
	{ 0xC, 0xA0, GAIN(10.0000) },
	{ 0xC, 0xA1, GAIN(10.0625) },
	{ 0xC, 0xA2, GAIN(10.1250) },
	{ 0xC, 0xA3, GAIN(10.1875) },
	{ 0xC, 0xA4, GAIN(10.2500) },
	{ 0xC, 0xA5, GAIN(10.3125) },
	{ 0xC, 0xA6, GAIN(10.3750) },
	{ 0xC, 0xA7, GAIN(10.4375) },
	{ 0xC, 0xA8, GAIN(10.5000) },
	{ 0xC, 0xA9, GAIN(10.5625) },
	{ 0xC, 0xAA, GAIN(10.6250) },
	{ 0xC, 0xAB, GAIN(10.6875) },
	{ 0xC, 0xAC, GAIN(10.7500) },
	{ 0xC, 0xAD, GAIN(10.8125) },
	{ 0xC, 0xAE, GAIN(10.8750) },
	{ 0xC, 0xAF, GAIN(10.9375) },
	{ 0xC, 0xB0, GAIN(11.0000) },
	{ 0xC, 0xB1, GAIN(11.0625) },
	{ 0xC, 0xB2, GAIN(11.1250) },
	{ 0xC, 0xB3, GAIN(11.1875) },
	{ 0xC, 0xB4, GAIN(11.2500) },
	{ 0xC, 0xB5, GAIN(11.3125) },
	{ 0xC, 0xB6, GAIN(11.3750) },
	{ 0xC, 0xB7, GAIN(11.4375) },
	{ 0xC, 0xB8, GAIN(11.5000) },
	{ 0xC, 0xB9, GAIN(11.5625) },
	{ 0xC, 0xBA, GAIN(11.6250) },
	{ 0xC, 0xBB, GAIN(11.6875) },
	{ 0xC, 0xBC, GAIN(11.7500) },
	{ 0xC, 0xBD, GAIN(11.8125) },
	{ 0xC, 0xBE, GAIN(11.8750) },
	{ 0xC, 0xBF, GAIN(11.9375) },
	{ 0xC, 0xC0, GAIN(12.0000) },
	{ 0xC, 0xC1, GAIN(12.0625) },
	{ 0xC, 0xC2, GAIN(12.1250) },
	{ 0xC, 0xC3, GAIN(12.1875) },
	{ 0xC, 0xC4, GAIN(12.2500) },
	{ 0xC, 0xC5, GAIN(12.3125) },
	{ 0xC, 0xC6, GAIN(12.3750) },
	{ 0xC, 0xC7, GAIN(12.4375) },
	{ 0xC, 0xC8, GAIN(12.5000) },
	{ 0xC, 0xC9, GAIN(12.5625) },
	{ 0xC, 0xCA, GAIN(12.6250) },
	{ 0xC, 0xCB, GAIN(12.6875) },
	{ 0xC, 0xCC, GAIN(12.7500) },
	{ 0xC, 0xCD, GAIN(12.8125) },
	{ 0xC, 0xCE, GAIN(12.8750) },
	{ 0xC, 0xCF, GAIN(12.9375) },
	{ 0xC, 0xD0, GAIN(13.0000) },
	{ 0xC, 0xD1, GAIN(13.0625) },
	{ 0xC, 0xD2, GAIN(13.1250) },
	{ 0xC, 0xD3, GAIN(13.1875) },
	{ 0xC, 0xD4, GAIN(13.2500) },
	{ 0xC, 0xD5, GAIN(13.3125) },
	{ 0xC, 0xD6, GAIN(13.3750) },
	{ 0xC, 0xD7, GAIN(13.4375) },
	{ 0xC, 0xD8, GAIN(13.5000) },
	{ 0xC, 0xD9, GAIN(13.5625) },
	{ 0xC, 0xDA, GAIN(13.6250) },
	{ 0xC, 0xDB, GAIN(13.6875) },
	{ 0xC, 0xDC, GAIN(13.7500) },
	{ 0xC, 0xDD, GAIN(13.8125) },
	{ 0xC, 0xDE, GAIN(13.8750) },
	{ 0xC, 0xDF, GAIN(13.9375) },
	{ 0xC, 0xE0, GAIN(14.0000) },
	{ 0xC, 0xE1, GAIN(14.0625) },
	{ 0xC, 0xE2, GAIN(14.1250) },
	{ 0xC, 0xE3, GAIN(14.1875) },
	{ 0xC, 0xE4, GAIN(14.2500) },
	{ 0xC, 0xE5, GAIN(14.3125) },
	{ 0xC, 0xE6, GAIN(14.3750) },
	{ 0xC, 0xE7, GAIN(14.4375) },
	{ 0xC, 0xE8, GAIN(14.5000) },
	{ 0xC, 0xE9, GAIN(14.5625) },
	{ 0xC, 0xEA, GAIN(14.6250) },
	{ 0xC, 0xEB, GAIN(14.6875) },
	{ 0xC, 0xEC, GAIN(14.7500) },
	{ 0xC, 0xED, GAIN(14.8125) },
	{ 0xC, 0xEE, GAIN(14.8750) },
	{ 0xC, 0xEF, GAIN(14.9375) },
	{ 0xC, 0xF0, GAIN(15.0000) },
	{ 0xC, 0xF1, GAIN(15.0625) },
	{ 0xC, 0xF2, GAIN(15.1250) },
	{ 0xC, 0xF3, GAIN(15.1875) },
	{ 0xC, 0xF4, GAIN(15.2500) },
	{ 0xC, 0xF5, GAIN(15.3125) },
	{ 0xC, 0xF6, GAIN(15.3750) },
	{ 0xC, 0xF7, GAIN(15.4375) },
	{ 0xC, 0xF8, GAIN(15.5000) },
	{ 0xC, 0xF9, GAIN(15.5625) },
	{ 0xC, 0xFA, GAIN(15.6250) },
	{ 0xC, 0xFB, GAIN(15.6875) },
	{ 0xC, 0xFC, GAIN(15.7500) },
	{ 0xC, 0xFD, GAIN(15.8125) },
	{ 0xC, 0xFE, GAIN(15.8750) },
	{ 0xC, 0xFF, GAIN(15.9375) },
#undef GAIN
};

static void ov16825_configure_gain(struct ov16825_info *info, s32 val)
{
	struct ov16825_gain	*gain;
	int			 i;

	if (!info->streaming) {
		return;
	}

	for (i = 0; i < ARRAY_SIZE(ov16825_gain_table); i++) {
		gain = &ov16825_gain_table[i];

		if (gain->gain >= val) {
			break;
		}
	}


	sensor_write(&info->sd, GAIN0, gain->major);
	sensor_write(&info->sd, GAIN1, gain->minor);
}

static void ov16825_configure_exposure(struct ov16825_info *info)
{
	s32			 expo = info->exposure->val;
	u32                      line_duration = info->config->line_duration_us;
	u32                      reg_val;

	if (!info->streaming) {
		return;
	}

	/* Register value has 4 fractional bits but it seems that setting those
	 * causes the camera to stop working. For now I just force them to 0
	 * which means that we don't have sub-line granularity. */
	reg_val = (expo / line_duration) << 4;

	sensor_write(&info->sd, EXPO0, (reg_val >> 16) & 0xff);
	sensor_write(&info->sd, EXPO1, (reg_val >> 8) & 0xff);
	sensor_write(&info->sd, EXPO2, (reg_val >> 0) & 0xff);
}

void ov16825_force_gain_reconfig(struct ov16825_info *info)
{
	/* Force gain reconfiguration by changing the value and reverting back
	 * to the original */
	ov16825_configure_gain(info, info->gain->val * 2);
	ov16825_configure_gain(info, info->gain->val);
}

void ov16825_set_isp_ctrl0(struct ov16825_info *info, int bit, int val)
{
	u8 reg;

	if (!info->streaming) {
		return;
	}

	sensor_read(&info->sd, ISP_CTRL00, &reg);
	reg &= ~(1 << bit);
	reg |= (!!val) << bit;
	sensor_write(&info->sd, ISP_CTRL00, reg);

	ov16825_force_gain_reconfig(info);
}

void ov16825_set_isp_ctrl1(struct ov16825_info * info, int bit, int val)
{
	u8 reg;

	if (!info->streaming) {
		return;
	}

	sensor_read(&info->sd, ISP_CTRL01, &reg);
	reg &= ~(1 << bit);
	reg |= (!!val) << bit;
	sensor_write(&info->sd, ISP_CTRL01, reg);

	ov16825_force_gain_reconfig(info);
}

void ov16825_set_digital_gain_en(struct ov16825_info * info, int en)
{
	sensor_write(&info->sd, ISP_CTRL04, !!en);

	ov16825_force_gain_reconfig(info);
}

void ov16825_set_component_gain(struct ov16825_info *info, u16 reg, u16 val)
{
	u8 high = val >> 8;
	u8 low  = val & 0xff;

	sensor_write(&info->sd, reg, high);
	sensor_write(&info->sd, reg + 1, low);
}

static void ov16825_set_test_pattern(struct ov16825_info *info)
{
	u8 pattern = ov16825_test_pattern_registers[info->test_pattern->val];

	if (!info->streaming) {
		return;
	}

	sensor_write(&info->sd, TEST_PATTERN, pattern);
}

static void ov16825_configure_vflip(struct ov16825_info *info)
{

	int flip = !!info->vflip->val;
	u8 reg;

	if (!info->streaming) {
		return;
	}

	sensor_read(&info->sd, FORMAT0, &reg);

	if (flip) {
		reg |= 0x6;
	} else {
		reg &= ~0x6;
	}

	sensor_write(&info->sd, FORMAT0, reg);
}

static void ov16825_configure_hflip(struct ov16825_info *info)
{
	int flip = !!info->hflip->val;
	u8 reg;

	if (!info->streaming) {
		return;
	}

	sensor_read(&info->sd, FORMAT1, &reg);

	if (flip) {
		reg |= 0x6;
	} else {
		reg &= ~0x6;
	}

	sensor_write(&info->sd, FORMAT1, reg);
}

#define VCM_ADDR 0xc

int vcm_write(struct ov16825_info *info, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&info->sd);
	int                ret;
	u8                 data[2];
	struct i2c_msg     msg = {
		.addr  = VCM_ADDR,
		.flags = 0,
		.len   = sizeof(data),
		.buf   = data,
	};

	data[0] =  val >> 8;
	data[1] =  val & 0xff;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		v4l2_err(&info->sd,
			 "VCM I2C write failed [%04x: %d]\n",
			 val, ret);
		return ret;
	}

	return 0;
}


static void ov16825_set_focus(struct ov16825_info *info)
{
	unsigned	focus = info->focus->val;

	if (!info->streaming) {
		return;
	}

	// Protection off
	vcm_write(info, 0xECA3);
	// Setting
        vcm_write(info, 0xF200|(0x0F<<3));
	// Protection On
        vcm_write(info, 0xDC51);
	// Position
        vcm_write(info, (focus << 4 |
			 (0x3 << 2 ) |
			 (0x0 << 0)));
}

static int ov16825_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov16825_info *info = container_of(ctrl->handler,
	                                         struct ov16825_info,
	                                         ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ov16825_configure_gain(info, info->gain->val);
		break;
	case V4L2_CID_EXPOSURE:
		ov16825_configure_exposure(info);
		break;
	case V4L2_CID_VFLIP:
		ov16825_configure_vflip(info);
		break;
	case V4L2_CID_HFLIP:
		ov16825_configure_hflip(info);
		break;
	case V4L2_CID_OV16825_ISP_EN:
		ov16825_set_isp_ctrl0(info, 0, info->isp_en->val);
		break;
	case V4L2_CID_OV16825_AVERAGE_EN:
		ov16825_set_isp_ctrl0(info, 1, info->average_en->val);
		break;
	case V4L2_CID_OV16825_WHITE_DPC_EN:
		ov16825_set_isp_ctrl0(info, 2, info->white_dpc_en->val);
		break;
	case V4L2_CID_OV16825_BLACK_DPC_EN:
		ov16825_set_isp_ctrl0(info, 3, info->black_dpc_en->val);
		break;
	case V4L2_CID_OV16825_WB_GAIN_EN:
		ov16825_set_isp_ctrl0(info, 4, info->wb_gain_en->val);
		break;
	case V4L2_CID_OV16825_OTP_CC_EN:
		ov16825_set_isp_ctrl0(info, 5, info->otp_cc_en->val);
		break;
	case V4L2_CID_OV16825_DBC_EN:
		ov16825_set_isp_ctrl0(info, 6, info->dbc_en->val);
		break;
	case V4L2_CID_OV16825_SCALE_EN:
		ov16825_set_isp_ctrl0(info, 7, info->scale_en->val);
		break;
	case V4L2_CID_OV16825_BLC_EN:
		ov16825_set_isp_ctrl1(info, 0, info->blc_en->val);
		break;
	case V4L2_CID_OV16825_AVERAGE_BEFORE:
		ov16825_set_isp_ctrl1(info, 5, info->average_before->val);
		break;
	case V4L2_CID_OV16825_DIGITAL_GAIN_EN:
		ov16825_set_digital_gain_en(info, info->digital_gain_en->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ov16825_set_test_pattern(info);
		break;
	case V4L2_CID_OV16825_RED_GAIN:
		ov16825_set_component_gain(info,
					   RED_GAIN,
					   info->red_gain->val);
		break;
	case V4L2_CID_OV16825_GREEN_GAIN:
		ov16825_set_component_gain(info,
					   GREEN_GAIN,
					   info->green_gain->val);
		break;
	case V4L2_CID_OV16825_BLUE_GAIN:
		ov16825_set_component_gain(info,
					   BLUE_GAIN,
					   info->blue_gain->val);
		break;
	case V4L2_CID_FOCUS_ABSOLUTE:
		ov16825_set_focus(info);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ov16825_ctrl_ops = {
	.s_ctrl = ov16825_s_ctrl,
};

void ov16825_apply_ctrl(struct ov16825_info *info)
{
	ov16825_s_ctrl(info->focus);
	ov16825_s_ctrl(info->gain);
	ov16825_s_ctrl(info->exposure);
	ov16825_s_ctrl(info->vflip);
	ov16825_s_ctrl(info->hflip);
	ov16825_s_ctrl(info->isp_en);
	ov16825_s_ctrl(info->average_en);
	ov16825_s_ctrl(info->white_dpc_en);
	ov16825_s_ctrl(info->black_dpc_en);
	ov16825_s_ctrl(info->wb_gain_en);
	ov16825_s_ctrl(info->otp_cc_en);
	ov16825_s_ctrl(info->dbc_en);
	ov16825_s_ctrl(info->scale_en);
	ov16825_s_ctrl(info->blc_en);
	ov16825_s_ctrl(info->average_before);
	ov16825_s_ctrl(info->digital_gain_en);
	ov16825_s_ctrl(info->red_gain);
	ov16825_s_ctrl(info->green_gain);
	ov16825_s_ctrl(info->blue_gain);
}

#define OV16825_CUSTOM_BOOL_CTRL(_desc, _cid, _default) { \
	.ops  = &ov16825_ctrl_ops,                        \
	.id   = _cid,                                     \
	.name = _desc,                                    \
	.type = V4L2_CTRL_TYPE_BOOLEAN,                   \
	.min  = false,                                    \
	.max  = true,                                     \
	.step = 1,                                        \
	.def  = _default,                                 \
}

static const struct v4l2_ctrl_config ov16825_isp_en =
	OV16825_CUSTOM_BOOL_CTRL("ISP enable",
				 V4L2_CID_OV16825_ISP_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_average_en =
	OV16825_CUSTOM_BOOL_CTRL("Average function enable",
				 V4L2_CID_OV16825_AVERAGE_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_white_dpc_en =
	OV16825_CUSTOM_BOOL_CTRL("White DPC enable",
				 V4L2_CID_OV16825_WHITE_DPC_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_black_dpc_en =
	OV16825_CUSTOM_BOOL_CTRL("Black DPC enable",
				 V4L2_CID_OV16825_BLACK_DPC_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_wb_gain_en =
	OV16825_CUSTOM_BOOL_CTRL("White balance gain enable",
				 V4L2_CID_OV16825_WB_GAIN_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_otp_cc_en =
	OV16825_CUSTOM_BOOL_CTRL("OTP cluster cancellation enable",
				 V4L2_CID_OV16825_OTP_CC_EN,
				 false);

static const struct v4l2_ctrl_config ov16825_dbc_en =
	OV16825_CUSTOM_BOOL_CTRL("Digital binning compensation enable",
				 V4L2_CID_OV16825_DBC_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_scale_en =
	OV16825_CUSTOM_BOOL_CTRL("Digital scale function enable",
				 V4L2_CID_OV16825_SCALE_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_blc_en =
	OV16825_CUSTOM_BOOL_CTRL("BLC function enable",
				 V4L2_CID_OV16825_BLC_EN,
				 true);

static const struct v4l2_ctrl_config ov16825_average_before =
	OV16825_CUSTOM_BOOL_CTRL("Average before ISP",
				 V4L2_CID_OV16825_AVERAGE_BEFORE,
				 false);

static const struct v4l2_ctrl_config ov16825_digital_gain_en =
	OV16825_CUSTOM_BOOL_CTRL("Digital gain enable",
				 V4L2_CID_OV16825_DIGITAL_GAIN_EN,
				 false);

static const struct v4l2_ctrl_config ov16825_test_pattern = {
	.ops            = &ov16825_ctrl_ops,
	.id             = V4L2_CID_TEST_PATTERN,
	.type           = V4L2_CTRL_TYPE_MENU,
	.name           = "Test Pattern",
	.min            = 0,
	.max            = ARRAY_SIZE(ov16825_test_pattern_menu) - 1,
	.step           = 0,
	.def            = 0,
	.flags          = 0,
	.menu_skip_mask = 0,
	.qmenu          = ov16825_test_pattern_menu,
};

static const struct v4l2_ctrl_config ov16825_red_gain = {
	.ops            = &ov16825_ctrl_ops,
	.id             = V4L2_CID_OV16825_RED_GAIN,
	.type           = V4L2_CTRL_TYPE_INTEGER,
	.name           = "Red gain",
	.min            = 0,
	.max            = 0xfff,
	.step           = 1,
	.def            = 0x400,
};

static const struct v4l2_ctrl_config ov16825_green_gain = {
	.ops            = &ov16825_ctrl_ops,
	.id             = V4L2_CID_OV16825_GREEN_GAIN,
	.type           = V4L2_CTRL_TYPE_INTEGER,
	.name           = "Green gain",
	.min            = 0,
	.max            = 0xfff,
	.step           = 1,
	.def            = 0x400,
};

static const struct v4l2_ctrl_config ov16825_blue_gain = {
	.ops            = &ov16825_ctrl_ops,
	.id             = V4L2_CID_OV16825_BLUE_GAIN,
	.type           = V4L2_CTRL_TYPE_INTEGER,
	.name           = "Blue gain",
	.min            = 0,
	.max            = 0xfff,
	.step           = 1,
	.def            = 0x400,
};

int ov16825_ctrl_create(struct ov16825_info *info)
{
	int rt;
	u32 gain_min, gain_max;

	if ((rt = v4l2_ctrl_handler_init(&info->ctrl_handler, 32)))
		return rt;

	gain_min = ov16825_gain_table[0].gain;
	gain_max = ov16825_gain_table[ARRAY_SIZE(ov16825_gain_table) - 1].gain;

	info->gain = v4l2_ctrl_new_std(&info->ctrl_handler,
				       &ov16825_ctrl_ops,
	                               V4L2_CID_GAIN,
				       gain_min,
				       gain_max,
				       1, gain_min);

	info->exposure = v4l2_ctrl_new_std(&info->ctrl_handler,
					   &ov16825_ctrl_ops,
					   V4L2_CID_EXPOSURE,
					   1,
					   100000,
					   1,
					   10000);

	info->vflip = v4l2_ctrl_new_std(&info->ctrl_handler,
					&ov16825_ctrl_ops,
					V4L2_CID_VFLIP,
					0,
					1,
					1,
					0);

	info->hflip = v4l2_ctrl_new_std(&info->ctrl_handler,
					&ov16825_ctrl_ops,
					V4L2_CID_HFLIP,
					0,
					1,
					1,
					1);

	info->red_gain = v4l2_ctrl_new_custom(&info->ctrl_handler,
					      &ov16825_red_gain,
					      NULL);

	info->green_gain = v4l2_ctrl_new_custom(&info->ctrl_handler,
						&ov16825_green_gain,
						NULL);

	info->blue_gain = v4l2_ctrl_new_custom(&info->ctrl_handler,
					       &ov16825_blue_gain,
					       NULL);

	info->isp_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
					    &ov16825_isp_en,
					    NULL);
	info->average_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
						&ov16825_average_en,
						NULL);
	info->white_dpc_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
						  &ov16825_white_dpc_en,
						  NULL);
	info->black_dpc_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
						  &ov16825_black_dpc_en,
						  NULL);
	info->wb_gain_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
						&ov16825_wb_gain_en,
						NULL);
	info->otp_cc_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
					       &ov16825_otp_cc_en,
					       NULL);
	info->dbc_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
					    &ov16825_dbc_en,
					    NULL);
	info->scale_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
					      &ov16825_scale_en,
					      NULL);
	info->blc_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
					    &ov16825_blc_en,
					    NULL);
	info->average_before = v4l2_ctrl_new_custom(&info->ctrl_handler,
						    &ov16825_average_before,
						    NULL);
	info->digital_gain_en = v4l2_ctrl_new_custom(&info->ctrl_handler,
						     &ov16825_digital_gain_en,
						     NULL);
	info->test_pattern = v4l2_ctrl_new_custom(&info->ctrl_handler,
						  &ov16825_test_pattern,
						  NULL);

	info->focus = v4l2_ctrl_new_std(&info->ctrl_handler,
					&ov16825_ctrl_ops,
					V4L2_CID_FOCUS_ABSOLUTE,
					50,
					1000,
					1,
					220);

	info->sd.ctrl_handler = &info->ctrl_handler;

	return 0;
}

void ov16825_ctrl_free(struct ov16825_info *info)
{
	v4l2_ctrl_handler_free(&info->ctrl_handler);
}
