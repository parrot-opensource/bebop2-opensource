/*
 * sunxi sensor header file
 * Author:raymonxiu
 */
#ifndef __OV16825_CAMERA__H__
#define __OV16825_CAMERA__H__

#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <linux/videodev2.h>
#include <media/ov16825.h>

#include "ov16825_registers.h"

#define DRIVER_NAME "ov16825"

struct ov16825_info {
	struct v4l2_subdev               sd;
	struct ov16825_platform_data    *pdata;
	struct media_pad                 pad;
	struct v4l2_mbus_framefmt        format;
	struct ov16825_config           *config;
	int                              streaming;

	/* Ctrl specific */
	struct v4l2_ctrl_handler         ctrl_handler;
	struct v4l2_ctrl                *gain;
	/* Exposure control in us */
	struct v4l2_ctrl                *exposure;
	struct v4l2_ctrl                *vflip;
	struct v4l2_ctrl                *hflip;
	struct v4l2_ctrl                *isp_en;
	struct v4l2_ctrl                *average_en;
	struct v4l2_ctrl                *white_dpc_en;
	struct v4l2_ctrl                *black_dpc_en;
	struct v4l2_ctrl                *wb_gain_en;
	struct v4l2_ctrl                *otp_cc_en;
	struct v4l2_ctrl                *dbc_en;
	struct v4l2_ctrl                *scale_en;
	struct v4l2_ctrl                *blc_en;
	struct v4l2_ctrl                *average_before;
	struct v4l2_ctrl                *digital_gain_en;
	struct v4l2_ctrl                *test_pattern;
	struct v4l2_ctrl                *red_gain;
	struct v4l2_ctrl                *green_gain;
	struct v4l2_ctrl                *blue_gain;
	struct v4l2_ctrl                *focus;
};

struct regval_list {
	unsigned short addr;
	unsigned char data;
};

struct ov16825_config {
	u32                 width;
	u32                 height;
	struct regval_list *regs;
	u32                 regs_len;
	u32                 line_duration_us;
};

struct ov16825_info *to_state(struct v4l2_subdev *sd);

int sensor_write(struct v4l2_subdev *sd, u16 reg, u8 val);

int sensor_write16(struct v4l2_subdev *sd, u16 reg, u16 val);

int sensor_read(struct v4l2_subdev *sd, u16 reg, u8 *val);

int sensor_write_array(struct v4l2_subdev *sd,
                       struct regval_list *regs,
                       int array_size);

#endif //__OV16825_CAMERA__H__
