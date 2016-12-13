/*
 * adv7604 - Analog Devices ADV7604 video decoder driver
 *
 * Copyright 2012 Cisco Systems, Inc. and/or its affiliates. All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*
 * References (c = chapter, p = page):
 * REF_01 - Analog devices, ADV7604, Register Settings Recommendations,
 *		Revision 2.5, June 2010
 * REF_02 - Analog devices, Register map documentation, Documentation of
 *		the register maps, Software manual, Rev. F, June 2010
 * REF_03 - Analog devices, ADV7604, Hardware Manual, Rev. F, August 2010
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/adv7604.h>
#include <linux/interrupt.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

MODULE_DESCRIPTION("Analog Devices ADV7604 video decoder driver");
MODULE_AUTHOR("Hans Verkuil <hans.verkuil@cisco.com>");
MODULE_AUTHOR("Mats Randgaard <mats.randgaard@cisco.com>");
MODULE_LICENSE("GPL");

/* ADV7604 system clock frequency */
#define ADV7604_fsc (28636360)

#define DIGITAL_INPUT (state->mode == ADV7604_MODE_HDMI)

enum adv7604_type {
	ADV7604,
	ADV7611,
};

struct adv7604_reg_seq {
	unsigned int reg;
	u8 val;
};

struct adv7604_chip_info {
	unsigned int edid_ctrl_reg;
	unsigned int edid_status_reg;
	unsigned int lcf_reg;

	unsigned int cable_det_mask;
	unsigned int tdms_lock_mask;
	unsigned int fmt_change_digital_mask;

	bool has_afe;

	void (*set_termination)(struct v4l2_subdev *sd, bool enable);
	void (*setup_irqs)(struct v4l2_subdev *sd);
	unsigned int (*read_hdmi_pixelclock)(struct v4l2_subdev *sd);

	/* 0 = AFE, 1 = HDMI */
	const struct adv7604_reg_seq *recommended_settings[2];
	unsigned int num_recommended_settings[2];

	unsigned long page_mask;
};

/*
 **********************************************************************
 *
 *  Arrays with configuration parameters for the ADV7604
 *
 **********************************************************************
 */
struct adv7604_state {
	const struct adv7604_chip_info *info;
	struct adv7604_platform_data pdata;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler hdl;
	enum adv7604_mode mode;
	struct v4l2_dv_timings timings;
	u8 edid[256];
	unsigned edid_blocks;
	struct v4l2_fract aspect_ratio;
	u32 rgb_quantization_range;
	struct workqueue_struct *work_queues;
	struct delayed_work delayed_work_enable_hotplug;
	bool connector_hdmi;
	bool restart_stdi_once;

	/* i2c clients */
	struct i2c_client *i2c_avlink;
	struct i2c_client *i2c_cec;
	struct i2c_client *i2c_infoframe;
	struct i2c_client *i2c_esdp;
	struct i2c_client *i2c_dpp;
	struct i2c_client *i2c_afe;
	struct i2c_client *i2c_repeater;
	struct i2c_client *i2c_edid;
	struct i2c_client *i2c_hdmi;
	struct i2c_client *i2c_test;
	struct i2c_client *i2c_cp;
	struct i2c_client *i2c_vdp;

	/* controls */
	struct v4l2_ctrl *detect_tx_5v_ctrl;
	struct v4l2_ctrl *analog_sampling_phase_ctrl;
	struct v4l2_ctrl *free_run_color_manual_ctrl;
	struct v4l2_ctrl *free_run_color_ctrl;
	struct v4l2_ctrl *rgb_quantization_range_ctrl;

	/* sysfs */
	struct sysfs_dirent *cable;
	struct sysfs_dirent *signal;
	struct sysfs_dirent *fmt;
	struct sysfs_dirent *audio;

	/* v4l */
	enum v4l2_mbus_type	 mbus_type;
	enum v4l2_mbus_pixelcode code;

	unsigned int	fmt_change;
	int		irq;

	struct v4l2_mbus_framefmt format;
};


struct adv7604_mbus_pixelcode {
	/* Bus rotation and reordering */
	enum adv7604_op_ch_sel op_ch_sel;
	/* Select output format */
	enum adv7604_op_format_sel op_format_sel;
	unsigned int invert_cbcr;

	enum v4l2_mbus_pixelcode code;
};


//For details see http://linuxtv.org/downloads/v4l-dvb-apis/subdev.html
struct adv7604_mbus_pixelcode pxcode [] = {
	{ ADV7604_OP_CH_SEL_RGB, ADV7604_OP_FORMAT_SEL_SDR_ITU656_8,  0, V4L2_MBUS_FMT_UYVY8_2X8  },
	{ ADV7604_OP_CH_SEL_RGB, ADV7604_OP_FORMAT_SEL_SDR_ITU656_8,  1, V4L2_MBUS_FMT_VYUY8_2X8  },
	{ ADV7604_OP_CH_SEL_RGB, ADV7604_OP_FORMAT_SEL_SDR_ITU656_16, 0, V4L2_MBUS_FMT_YUYV8_1X16 },
	{ ADV7604_OP_CH_SEL_RGB, ADV7604_OP_FORMAT_SEL_SDR_ITU656_16, 1, V4L2_MBUS_FMT_YVYU8_1X16 },
	{ ADV7604_OP_CH_SEL_RBG, ADV7604_OP_FORMAT_SEL_SDR_ITU656_16, 0, V4L2_MBUS_FMT_UYVY8_1X16 },
	{ ADV7604_OP_CH_SEL_RBG, ADV7604_OP_FORMAT_SEL_SDR_ITU656_16, 1, V4L2_MBUS_FMT_VYUY8_1X16 },
};


static inline bool no_signal(struct v4l2_subdev *sd);

static bool adv7604_has_afe(struct adv7604_state *state)
{
	return state->info->has_afe;
}

/* Supported CEA and DMT timings */
static const struct v4l2_dv_timings adv7604_timings[] = {
	V4L2_DV_BT_CEA_720X480I59_94,
	V4L2_DV_BT_CEA_720X480P59_94,
	V4L2_DV_BT_CEA_720X576P50,
	V4L2_DV_BT_CEA_1280X720P24,
	V4L2_DV_BT_CEA_1280X720P25,
	V4L2_DV_BT_CEA_1280X720P50,
	V4L2_DV_BT_CEA_1280X720P60,
	V4L2_DV_BT_CEA_1920X1080P24,
	V4L2_DV_BT_CEA_1920X1080P25,
	V4L2_DV_BT_CEA_1920X1080P30,
	V4L2_DV_BT_CEA_1920X1080I50,
	V4L2_DV_BT_CEA_1920X1080P50,
	V4L2_DV_BT_CEA_1920X1080I60,
	V4L2_DV_BT_CEA_1920X1080P60,

	/* sorted by DMT ID */
	V4L2_DV_BT_DMT_640X350P85,
	V4L2_DV_BT_DMT_640X400P85,
	V4L2_DV_BT_DMT_720X400P85,
	V4L2_DV_BT_DMT_640X480P60,
	V4L2_DV_BT_DMT_640X480P72,
	V4L2_DV_BT_DMT_640X480P75,
	V4L2_DV_BT_DMT_640X480P85,
	V4L2_DV_BT_DMT_800X600P56,
	V4L2_DV_BT_DMT_800X600P60,
	V4L2_DV_BT_DMT_800X600P72,
	V4L2_DV_BT_DMT_800X600P75,
	V4L2_DV_BT_DMT_800X600P85,
	V4L2_DV_BT_DMT_848X480P60,
	V4L2_DV_BT_DMT_1024X768P60,
	V4L2_DV_BT_DMT_1024X768P70,
	V4L2_DV_BT_DMT_1024X768P75,
	V4L2_DV_BT_DMT_1024X768P85,
	V4L2_DV_BT_DMT_1152X864P75,
	V4L2_DV_BT_DMT_1280X768P60_RB,
	V4L2_DV_BT_DMT_1280X768P60,
	V4L2_DV_BT_DMT_1280X768P75,
	V4L2_DV_BT_DMT_1280X768P85,
	V4L2_DV_BT_DMT_1280X800P60_RB,
	V4L2_DV_BT_DMT_1280X800P60,
	V4L2_DV_BT_DMT_1280X800P75,
	V4L2_DV_BT_DMT_1280X800P85,
	V4L2_DV_BT_DMT_1280X960P60,
	V4L2_DV_BT_DMT_1280X960P85,
	V4L2_DV_BT_DMT_1280X1024P60,
	V4L2_DV_BT_DMT_1280X1024P75,
	V4L2_DV_BT_DMT_1280X1024P85,
	V4L2_DV_BT_DMT_1360X768P60,
	V4L2_DV_BT_DMT_1400X1050P60_RB,
	V4L2_DV_BT_DMT_1400X1050P60,
	V4L2_DV_BT_DMT_1400X1050P75,
	V4L2_DV_BT_DMT_1400X1050P85,
	V4L2_DV_BT_DMT_1440X900P60_RB,
	V4L2_DV_BT_DMT_1440X900P60,
	V4L2_DV_BT_DMT_1600X1200P60,
	V4L2_DV_BT_DMT_1680X1050P60_RB,
	V4L2_DV_BT_DMT_1680X1050P60,
	V4L2_DV_BT_DMT_1792X1344P60,
	V4L2_DV_BT_DMT_1856X1392P60,
	V4L2_DV_BT_DMT_1920X1200P60_RB,
	V4L2_DV_BT_DMT_1366X768P60,
	V4L2_DV_BT_DMT_1920X1080P60,
	{ },
};

struct adv7604_video_standards {
	struct v4l2_dv_timings timings;
	u8 vid_std;
	u8 v_freq;
};

/* sorted by number of lines */
static const struct adv7604_video_standards adv7604_prim_mode_comp[] = {
	{ V4L2_DV_BT_CEA_720X480I59_94, 0x00, 0x00 },
	{ V4L2_DV_BT_CEA_720X480P59_94, 0x0a, 0x00 },
	{ V4L2_DV_BT_CEA_720X576I50, 0x01, 0x01 },
	{ V4L2_DV_BT_CEA_720X576P50, 0x0b, 0x00 },
	{ V4L2_DV_BT_CEA_1280X720P50, 0x19, 0x01 },
	{ V4L2_DV_BT_CEA_1280X720P60, 0x19, 0x00 },
	{ V4L2_DV_BT_CEA_1920X1080I50, 0x14, 0x01 },
	{ V4L2_DV_BT_CEA_1920X1080I60, 0x14, 0x00 },
	{ V4L2_DV_BT_CEA_1920X1080P24, 0x1e, 0x04 },
	{ V4L2_DV_BT_CEA_1920X1080P25, 0x1e, 0x03 },
	{ V4L2_DV_BT_CEA_1920X1080P30, 0x1e, 0x02 },
	{ V4L2_DV_BT_CEA_1920X1080P50, 0x1e, 0x01 },
	{ V4L2_DV_BT_CEA_1920X1080P60, 0x1e, 0x00 },
	/* TODO add 1920x1080P60_RB (CVT timing) */
	{ },
};

/* sorted by number of lines */
static const struct adv7604_video_standards adv7604_prim_mode_gr[] = {
	{ V4L2_DV_BT_DMT_640X480P60, 0x08, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P72, 0x09, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P75, 0x0a, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P85, 0x0b, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P56, 0x00, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P60, 0x01, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P72, 0x02, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P75, 0x03, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P85, 0x04, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P60, 0x0c, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P70, 0x0d, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P75, 0x0e, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P85, 0x0f, 0x00 },
	{ V4L2_DV_BT_DMT_1280X1024P60, 0x05, 0x00 },
	{ V4L2_DV_BT_DMT_1280X1024P75, 0x06, 0x00 },
	{ V4L2_DV_BT_DMT_1360X768P60, 0x12, 0x00 },
	{ V4L2_DV_BT_DMT_1366X768P60, 0x13, 0x00 },
	{ V4L2_DV_BT_DMT_1400X1050P60, 0x14, 0x00 },
	{ V4L2_DV_BT_DMT_1400X1050P75, 0x15, 0x00 },
	{ V4L2_DV_BT_DMT_1600X1200P60, 0x16, 0x00 }, /* TODO not tested */
	/* TODO add 1600X1200P60_RB (not a DMT timing) */
	{ V4L2_DV_BT_DMT_1680X1050P60, 0x18, 0x00 },
	{ V4L2_DV_BT_DMT_1920X1200P60_RB, 0x19, 0x00 }, /* TODO not tested */
	{ },
};

/* sorted by number of lines */
static const struct adv7604_video_standards adv7604_prim_mode_hdmi_comp[] = {
	{ V4L2_DV_BT_CEA_720X480I59_94, 0x00, 0x00 },
	{ V4L2_DV_BT_CEA_720X480P59_94, 0x0a, 0x00 },
	{ V4L2_DV_BT_CEA_720X576I50, 0x01, 0x01 },
	{ V4L2_DV_BT_CEA_720X576P50, 0x0b, 0x00 },
	{ V4L2_DV_BT_CEA_1280X720P50, 0x13, 0x01 },
	{ V4L2_DV_BT_CEA_1280X720P60, 0x13, 0x00 },
	{ V4L2_DV_BT_CEA_1920X1080I50, 0x14, 0x01 },
	{ V4L2_DV_BT_CEA_1920X1080I60, 0x14, 0x00 },
	{ V4L2_DV_BT_CEA_1920X1080P24, 0x1e, 0x04 },
	{ V4L2_DV_BT_CEA_1920X1080P25, 0x1e, 0x03 },
	{ V4L2_DV_BT_CEA_1920X1080P30, 0x1e, 0x02 },
	{ V4L2_DV_BT_CEA_1920X1080P50, 0x1e, 0x01 },
	{ V4L2_DV_BT_CEA_1920X1080P60, 0x1e, 0x00 },
	{ },
};

/* sorted by number of lines */
static const struct adv7604_video_standards adv7604_prim_mode_hdmi_gr[] = {
	{ V4L2_DV_BT_DMT_640X480P60, 0x08, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P72, 0x09, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P75, 0x0a, 0x00 },
	{ V4L2_DV_BT_DMT_640X480P85, 0x0b, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P56, 0x00, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P60, 0x01, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P72, 0x02, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P75, 0x03, 0x00 },
	{ V4L2_DV_BT_DMT_800X600P85, 0x04, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P60, 0x0c, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P70, 0x0d, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P75, 0x0e, 0x00 },
	{ V4L2_DV_BT_DMT_1024X768P85, 0x0f, 0x00 },
	{ V4L2_DV_BT_DMT_1280X1024P60, 0x05, 0x00 },
	{ V4L2_DV_BT_DMT_1280X1024P75, 0x06, 0x00 },
	{ },
};

/* ----------------------------------------------------------------------- */

static inline struct adv7604_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct adv7604_state, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct adv7604_state, hdl)->sd;
}

static inline unsigned hblanking(const struct v4l2_bt_timings *t)
{
	return t->hfrontporch + t->hsync + t->hbackporch;
}

static inline unsigned htotal(const struct v4l2_bt_timings *t)
{
	return t->width + t->hfrontporch + t->hsync + t->hbackporch;
}

static inline unsigned vblanking(const struct v4l2_bt_timings *t)
{
	return t->vfrontporch + t->vsync + t->vbackporch;
}

static inline unsigned vtotal(const struct v4l2_bt_timings *t)
{
	return t->height + t->vfrontporch + t->vsync + t->vbackporch;
}

/* ----------------------------------------------------------------------- */

static s32 adv_smbus_read_byte_data_check(struct i2c_client *client,
		u8 command, bool check)
{
	union i2c_smbus_data data;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data)){
		if(sd)
			v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
					client->addr, command, data.byte);
		return data.byte;
	}
	if (check)
		v4l_err(client, "error reading 0x%02x, 0x%02x\n",
				client->addr, command);
	return -EIO;
}

static s32 adv_smbus_read_byte_data(struct i2c_client *client, u8 command)
{
	return adv_smbus_read_byte_data_check(client, command, true);
}

static s32 adv_smbus_write_byte_data(struct i2c_client *client,
					u8 command, u8 value)
{
	union i2c_smbus_data data;
	int err;
	int i;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	data.byte = value;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
		if (!err)
			break;
	}
	if (err < 0)
		v4l_err(client, "error writing 0x%02x, 0x%02x, 0x%02x\n",
				client->addr, command, value);
	else if(sd)
		v4l2_dbg(2, debug, sd, "writing 0x%02x, 0x%02x, 0x%02x\n",
				client->addr, command, value);

	return err;
}

static s32 adv_smbus_write_i2c_block_data(struct i2c_client *client,
	       u8 command, unsigned length, const u8 *values)
{
	union i2c_smbus_data data;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	memcpy(data.block + 1, values, length);
	return i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			      I2C_SMBUS_WRITE, command,
			      I2C_SMBUS_I2C_BLOCK_DATA, &data);
}

/* ----------------------------------------------------------------------- */

static inline int io_read(struct v4l2_subdev *sd, u8 reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return adv_smbus_read_byte_data(client, reg);
}

static inline int io_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return adv_smbus_write_byte_data(client, reg, val);
}

static inline int io_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return io_write(sd, reg, (io_read(sd, reg) & mask) | val);
}

static inline int avlink_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_avlink, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_avlink->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_avlink->addr, reg);
	return ret;
}

static inline int avlink_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_avlink, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret >= 0)? "w" : "error w",
			state->i2c_avlink->addr, reg, val);

	return ret;
}

static inline int cec_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_cec, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_cec->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_cec->addr, reg);
	return ret;
}

static inline int cec_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_cec, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_cec->addr, reg, val);
	return ret;
}

static inline int cec_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return cec_write(sd, reg, (cec_read(sd, reg) & mask) | val);
}

static inline int infoframe_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_infoframe, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_infoframe->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_infoframe->addr, reg);
	return ret;
}

static inline int infoframe_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_infoframe, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_infoframe->addr, reg, val);
	return ret;
}

static inline int esdp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_esdp, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_esdp->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_esdp->addr, reg);
	return ret;
}

static inline int esdp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_esdp, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_esdp->addr, reg, val);
	return ret;
}

static inline int dpp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_dpp, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_dpp->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_dpp->addr, reg);
	return ret;
}

static inline int dpp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_dpp, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_dpp->addr, reg, val);
	return ret;
}

static inline int afe_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_afe, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_afe->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_afe->addr, reg);
	return ret;
}

static inline int afe_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_afe, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_afe->addr, reg, val);
	return ret;
}

static inline int rep_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_repeater, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_repeater->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_repeater->addr, reg);
	return ret;
}

static inline int rep_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_repeater, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_repeater->addr, reg, val);
	return ret;
}

static inline int rep_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return rep_write(sd, reg, (rep_read(sd, reg) & mask) | val);
}

static inline int edid_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_edid, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_edid->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_edid->addr, reg);
	return ret;
}

static inline int edid_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_edid, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_edid->addr, reg, val);
	return ret;
}

static inline int edid_read_block(struct v4l2_subdev *sd, unsigned len, u8 *val)
{
	struct adv7604_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_edid;
	u8 msgbuf0[1] = { 0 };
	u8 msgbuf1[256];
	struct i2c_msg msg[2] = { { client->addr, 0, 1, msgbuf0 },
				  { client->addr, 0 | I2C_M_RD, len, msgbuf1 }
				};

	if (i2c_transfer(client->adapter, msg, 2) < 0)
		return -EIO;
	memcpy(val, msgbuf1, len);
	return 0;
}

static void adv7604_delayed_work_enable_hotplug(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct adv7604_state *state = container_of(dwork, struct adv7604_state,
						delayed_work_enable_hotplug);
	struct v4l2_subdev *sd = &state->sd;

	v4l2_dbg(2, debug, sd, "%s: enable hotplug\n", __func__);

	v4l2_subdev_notify(sd, ADV7604_HOTPLUG, (void *)1);
}

static inline int edid_write_block(struct v4l2_subdev *sd,
					unsigned len, const u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	int err = 0;
	int i;

	v4l2_dbg(2, debug, sd, "%s: write EDID block (%d byte)\n", __func__, len);

	v4l2_subdev_notify(sd, ADV7604_HOTPLUG, (void *)0);

	/* Disables I2C access to internal EDID ram from DDC port */
	rep_write_and_or(sd, info->edid_ctrl_reg, 0xf0, 0x0);

	for (i = 0; !err && i < len; i += I2C_SMBUS_BLOCK_MAX)
		err = adv_smbus_write_i2c_block_data(state->i2c_edid, i,
				I2C_SMBUS_BLOCK_MAX, val + i);
	if (err)
		return err;

	/* adv7604 calculates the checksums and enables I2C access to internal
	   EDID ram from DDC port. */
	rep_write_and_or(sd, info->edid_ctrl_reg, 0xf0, 0x1);

	for (i = 0; i < 1000; i++) {
		if (rep_read(sd, info->edid_status_reg) & 1)
			break;
		mdelay(1);
	}
	if (i == 1000) {
		v4l_err(client, "error enabling edid\n");
		return -EIO;
	}

	/* enable hotplug after 100 ms */
	queue_delayed_work(state->work_queues,
			&state->delayed_work_enable_hotplug, HZ / 10);
	return 0;
}

static inline int hdmi_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_hdmi, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_hdmi->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_hdmi->addr, reg);
	return ret;
}

static u16 hdmi_read16(struct v4l2_subdev *sd, u8 reg, u16 mask)
{
	return ((hdmi_read(sd, reg) << 8) | hdmi_read(sd, reg + 1)) & mask;
}

static inline int hdmi_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_hdmi, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_hdmi->addr, reg, val);
	return ret;
}

static inline int test_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_test, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_test->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_test->addr, reg);
	return ret;

}

static inline int test_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_test, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_test->addr, reg, val);
	return ret;
}

static inline int cp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_cp, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_cp->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_cp->addr, reg);
	return ret;
}

static u16 cp_read16(struct v4l2_subdev *sd, u8 reg, u16 mask)
{
	return ((cp_read(sd, reg) << 8) | cp_read(sd, reg + 1)) & mask;
}

static inline int cp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_cp, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_cp->addr, reg, val);
	return ret;
}

static inline int cp_write_and_or(struct v4l2_subdev *sd, u8 reg, u8 mask, u8 val)
{
	return cp_write(sd, reg, (cp_read(sd, reg) & mask) | val);
}

static inline int vdp_read(struct v4l2_subdev *sd, u8 reg)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_read_byte_data(state->i2c_vdp, reg);

	if(ret >= 0)
		v4l2_dbg(2, debug, sd, "reading 0x%02x, 0x%02x, 0x%02x\n",
				state->i2c_vdp->addr, reg, ret);
	else
		v4l2_dbg(2, debug, sd, "error reading 0x%02x, 0x%02x\n",
				state->i2c_vdp->addr, reg);
	return ret;
}

static inline int vdp_write(struct v4l2_subdev *sd, u8 reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	int ret = adv_smbus_write_byte_data(state->i2c_vdp, reg, val);

	v4l2_dbg(2, debug, sd, "%sritting 0x%02x, 0x%02x, 0x%02x\n",
			(ret>=0)? "w" : "error w",
			state->i2c_vdp->addr, reg, val);
	return ret;
}

enum {
	ADV7604_PAGE_IO,
	ADV7604_PAGE_AVLINK,
	ADV7604_PAGE_CEC,
	ADV7604_PAGE_INFOFRAME,
	ADV7604_PAGE_ESDP,
	ADV7604_PAGE_DPP,
	ADV7604_PAGE_AFE,
	ADV7604_PAGE_REP,
	ADV7604_PAGE_EDID,
	ADV7604_PAGE_HDMI,
	ADV7604_PAGE_TEST,
	ADV7604_PAGE_CP,
	ADV7604_PAGE_VDP,
};

#define ADV7604_REG(page, offset) (((page) << 8) | (offset))

static int __maybe_unused adv7604_read_reg(struct v4l2_subdev *sd,
                                           unsigned int reg)
{
	struct adv7604_state *state = to_state(sd);
	unsigned int page = reg >> 8;

	if (!(BIT(page) & state->info->page_mask))
		return -EINVAL;

	reg &= 0xff;

	switch (page) {
	case ADV7604_PAGE_IO:
		return io_read(sd, reg);
	case ADV7604_PAGE_AVLINK:
		return avlink_read(sd, reg);
	case ADV7604_PAGE_CEC:
		return cec_read(sd, reg);
	case ADV7604_PAGE_INFOFRAME:
		return infoframe_read(sd, reg);
	case ADV7604_PAGE_ESDP:
		return esdp_read(sd, reg);
	case ADV7604_PAGE_DPP:
		return dpp_read(sd, reg);
	case ADV7604_PAGE_AFE:
		return afe_read(sd, reg);
	case ADV7604_PAGE_REP:
		return rep_read(sd, reg);
	case ADV7604_PAGE_EDID:
		return edid_read(sd, reg);
	case ADV7604_PAGE_HDMI:
		return hdmi_read(sd, reg);
	case ADV7604_PAGE_TEST:
		return test_read(sd, reg);
	case ADV7604_PAGE_CP:
		return cp_read(sd, reg);
	case ADV7604_PAGE_VDP:
		return vdp_read(sd, reg);
	}

	return -EINVAL;
}

static int adv7604_write_reg(struct v4l2_subdev *sd, unsigned int reg, u8 val)
{
	struct adv7604_state *state = to_state(sd);
	unsigned int page = reg >> 8;

	if (!(BIT(page) & state->info->page_mask))
		return -EINVAL;

	reg &= 0xff;

	switch (page) {
	case ADV7604_PAGE_IO:
		return io_write(sd, reg, val);
	case ADV7604_PAGE_AVLINK:
		return avlink_write(sd, reg, val);
	case ADV7604_PAGE_CEC:
		return cec_write(sd, reg, val);
	case ADV7604_PAGE_INFOFRAME:
		return infoframe_write(sd, reg, val);
	case ADV7604_PAGE_ESDP:
		return esdp_write(sd, reg, val);
	case ADV7604_PAGE_DPP:
		return dpp_write(sd, reg, val);
	case ADV7604_PAGE_AFE:
		return afe_write(sd, reg, val);
	case ADV7604_PAGE_REP:
		return rep_write(sd, reg, val);
	case ADV7604_PAGE_EDID:
		return edid_write(sd, reg, val);
	case ADV7604_PAGE_HDMI:
		return hdmi_write(sd, reg, val);
	case ADV7604_PAGE_TEST:
		return test_write(sd, reg, val);
	case ADV7604_PAGE_CP:
		return cp_write(sd, reg, val);
	case ADV7604_PAGE_VDP:
		return vdp_write(sd, reg, val);
	}

	return -EINVAL;
}

static void adv7604_write_reg_seq(struct v4l2_subdev *sd,
	const struct adv7604_reg_seq *reg_seq, unsigned int reg_seq_size)
{
	unsigned int i;

	for (i = 0; i < reg_seq_size; i++)
		adv7604_write_reg(sd, reg_seq[i].reg, reg_seq[i].val);
}



static enum v4l2_mbus_pixelcode adv7604_get_mbus_pixelcode( enum adv7604_op_ch_sel op_ch_sel,
		enum adv7604_op_format_sel op_format_sel, unsigned int invert_cbcr)
{
	int loop;
	for( loop = 0; loop < ARRAY_SIZE(pxcode); loop++){

		if( (pxcode[loop].op_ch_sel == op_ch_sel)  &&
			(pxcode[loop].op_format_sel == op_format_sel) &&
			(pxcode[loop].invert_cbcr == invert_cbcr) ){
			break;
		}
	}

	if( loop == ARRAY_SIZE(pxcode) ){
		loop = 0;
		pr_err("%s: op_ch_sel/op_format_sel : bad configuration\n", __func__);
	}
	return pxcode[loop].code;
}

/* ----------------------------------------------------------------------- */
static ssize_t cable_detect_show(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(device);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
        return snprintf(buf, PAGE_SIZE,
			"%d\n", io_read(sd, 0x6f) /* & 0x1 */);
}

static ssize_t signal_detect_show(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(device);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
        return snprintf(buf, PAGE_SIZE,
			"%d\n", (no_signal(sd)));
}

static ssize_t fmt_change_show(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(device);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7604_state *state = to_state(sd);
	ssize_t val;

        val = snprintf(buf, PAGE_SIZE,
			"%s\n",	(state->fmt_change)? "1": "0");

	if(state->fmt_change)
		state->fmt_change = 0;

	return val;
}

static ssize_t audio_change_show(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	struct i2c_client *client = to_i2c_client(device);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	int audio_ch_md_raw, audio_channel_mode = 0, audio_status = 0;
	ssize_t size = 0;
	unsigned char data[6] = {0};
	char * freq = NULL;
	const char *clock_accuracy[4] = {
		"Level II, ±1000 ppm",
		"Level I, ±50 ppm",
		"Level III, variable pitch shifted",
		"Reserved",
	};

	data[5] = io_read(sd, 0x65);

	if( data[5] & 0x40 )//audio is mute
		return snprintf(buf, PAGE_SIZE, "Mute\n");

	audio_ch_md_raw = data[5] & 0x10;
	audio_channel_mode = hdmi_read(sd, 0x7) & 0x40;
	audio_status = hdmi_read(sd, 0x18);

	if( data[5] & 0x80 ){
		data[0] = hdmi_read(sd, 0x36);
		data[1] = hdmi_read(sd, 0x37);
		data[2] = hdmi_read(sd, 0x38);
		data[3] = hdmi_read(sd, 0x39);
		data[4] = hdmi_read(sd, 0x3a);
	}

	if( audio_status & 0x2){
		size += snprintf(buf + size, PAGE_SIZE - size, "\t%s-channel audio\n",
			audio_ch_md_raw? "8": "2");
	}

	size += snprintf(buf + size, PAGE_SIZE - size, "\t%s\n",
			audio_channel_mode? "Multichannel uncompressed audio" : "Stereo audio");

	if( audio_status & 0x1)
		size += snprintf(buf + size, PAGE_SIZE - size, "\tL_PCM or IEC 61937\n");
	else
		return snprintf(buf + size, PAGE_SIZE - size, "\tNo L_PCM or IEC 61937\n");

	if( audio_status & 0x2)
		size += snprintf(buf + size, PAGE_SIZE - size, "\tDSD packet\n");

	if( audio_status & 0x4)
		size += snprintf(buf + size, PAGE_SIZE - size, "\tDST packet\n");

	if( audio_status & 0x4)
		size += snprintf(buf + size, PAGE_SIZE - size, "\tHBR packet\n");

	if( data[5] & 0x80 ){
		switch( ( data[3] & 0xf) ){
			case 0 : freq = "44.1 kHz";break;
			case 2 : freq = "48 kHz"; break;
			case 3 : freq = "32 kHz"; break;
			case 8 : freq = "88.2 kHz"; break;
			case 9 : freq = "768 kHz"; break;
			case 10 : freq = "96 kHz"; break;
			case 12 : freq = "176 kHz"; break;
			case 14 : freq = "192 kHz"; break;
		};

		if(debug){
			size += snprintf(buf + size, PAGE_SIZE - size,
				"\t%s application\n"
				"\t%sopyright is asserted\n"
				"\t%sPCM Audio samples\n"
				"\tCategory code       : %d\n"
				"\tSource number       : %d\n"
				"\tChannel number      : %d\n",
				(data[0] & 0x1)? "Consumer":"Professional",
				(data[0] & 0x4)? "No c":"C",
				(data[0] & 0x2)? "Non-":"",
				data[1],
				data[2] & 0xf,
				(data[2] & 0xf0) >> 4);

			if( !(data[0] & 0x2) ){
				size += snprintf(buf + size, PAGE_SIZE - size,
					"\tTwo audio channels %s\n"
					"\tSampling Frequency  : %s\n"
					"\tClock Accuracy      : %s\n",
					(data[0] & 0x8)? "with 50/15 pre-emphasis":"without pre-emphasis",
					freq? freq : "Unknown",
					clock_accuracy[(( data[3] & 0x30)>>4)]);
			}

		}else{
			size += snprintf(buf + size, PAGE_SIZE - size,
				"\t%sPCM Audio samples\n",
				(data[0] & 0x2)? "Non-":"");

			if( !(data[0] & 0x2) )
				size += snprintf(buf + size, PAGE_SIZE - size,
					"\tSampling Frequency  : %s\n",
					freq? freq : "Unknown");
		}

	}

	return size;
}


static struct device_attribute adv7611_sysfs_attrs[] = {
        __ATTR(cable, S_IRUGO, cable_detect_show, NULL),
        __ATTR(signal, S_IRUGO, signal_detect_show, NULL),
        __ATTR(fmt, S_IRUGO, fmt_change_show, NULL),
        __ATTR(audio, S_IRUGO, audio_change_show, NULL),
};



/* ----------------------------------------------------------------------- */


#ifdef CONFIG_VIDEO_ADV_DEBUG
static void adv7604_inv_register(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "0x000-0x0ff: IO Map\n");
	v4l2_info(sd, "0x100-0x1ff: AVLink Map\n");
	v4l2_info(sd, "0x200-0x2ff: CEC Map\n");
	v4l2_info(sd, "0x300-0x3ff: InfoFrame Map\n");
	v4l2_info(sd, "0x400-0x4ff: ESDP Map\n");
	v4l2_info(sd, "0x500-0x5ff: DPP Map\n");
	v4l2_info(sd, "0x600-0x6ff: AFE Map\n");
	v4l2_info(sd, "0x700-0x7ff: Repeater Map\n");
	v4l2_info(sd, "0x800-0x8ff: EDID Map\n");
	v4l2_info(sd, "0x900-0x9ff: HDMI Map\n");
	v4l2_info(sd, "0xa00-0xaff: Test Map\n");
	v4l2_info(sd, "0xb00-0xbff: CP Map\n");
	v4l2_info(sd, "0xc00-0xcff: VDP Map\n");
}

static int adv7604_g_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	ret = adv7604_read_reg(sd, reg->reg);
	if (ret < 0) {
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		adv7604_inv_register(sd);
		return ret;
	}

	reg->size = 1;
	reg->val = ret;

	return 0;
}

static int adv7604_s_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (!v4l2_chip_match_i2c_client(client, &reg->match))
		return -EINVAL;
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	ret = adv7604_write_reg(sd, reg->reg, reg->val);
	if (ret < 0) {
		v4l2_info(sd, "Register %03llx not supported\n", reg->reg);
		adv7604_inv_register(sd);
		return ret;
	}

	return 0;
}
#endif

static int adv7604_s_detect_tx_5v_ctrl(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	s32 val = io_read(sd, 0x6f);

	if(val < 0)
		return val;

	/* port A only */
	return v4l2_ctrl_s_ctrl(state->detect_tx_5v_ctrl,
				(bool)(io_read(sd, 0x6f) & info->cable_det_mask));
}

static int find_and_set_predefined_video_timings(struct v4l2_subdev *sd,
		u8 prim_mode,
		const struct adv7604_video_standards *predef_vid_timings,
		const struct v4l2_dv_timings *timings)
{
	struct adv7604_state *state = to_state(sd);
	int i;

	for (i = 0; predef_vid_timings[i].timings.bt.width; i++) {
		if (!v4l_match_dv_timings(timings, &predef_vid_timings[i].timings,
					DIGITAL_INPUT ? 250000 : 1000000))
			continue;
		io_write(sd, 0x00, predef_vid_timings[i].vid_std); /* video std */
		io_write(sd, 0x01, (predef_vid_timings[i].v_freq << 4) +
				prim_mode); /* v_freq and prim mode */
		return 0;
	}

	return -1;
}

static int configure_predefined_video_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct adv7604_state *state = to_state(sd);
	int err;

	v4l2_dbg(1, debug, sd, "%s", __func__);

	if (adv7604_has_afe(state)) {
		/* reset to default values */
		io_write(sd, 0x16, 0x43);
		io_write(sd, 0x17, 0x5a);
	}
	/* disable embedded syncs for auto graphics mode */
	if (adv7604_has_afe(state)){
		cp_write_and_or(sd, 0x81, 0xef, 0x00);
		cp_write(sd, 0x8f, 0x00);
		cp_write(sd, 0x90, 0x00);
		cp_write(sd, 0xa2, 0x00);
		cp_write(sd, 0xa3, 0x00);
		cp_write(sd, 0xa4, 0x00);
		cp_write(sd, 0xa5, 0x00);
		cp_write(sd, 0xa6, 0x00);
		cp_write(sd, 0xa7, 0x00);
		cp_write(sd, 0xab, 0x00);
		cp_write(sd, 0xac, 0x00);
	}

	switch (state->mode) {
	case ADV7604_MODE_COMP:
	case ADV7604_MODE_GR:
		err = find_and_set_predefined_video_timings(sd,
				0x01, adv7604_prim_mode_comp, timings);
		if (err)
			err = find_and_set_predefined_video_timings(sd,
					0x02, adv7604_prim_mode_gr, timings);
		break;
	case ADV7604_MODE_HDMI:
		err = find_and_set_predefined_video_timings(sd,
				0x05, adv7604_prim_mode_hdmi_comp, timings);
		if (err)
			err = find_and_set_predefined_video_timings(sd,
					0x06, adv7604_prim_mode_hdmi_gr, timings);

		break;
	default:
		v4l2_dbg(2, debug, sd, "%s: Unknown mode %d\n",
				__func__, state->mode);
		err = -1;
		break;
	}


	return err;
}

static void configure_custom_video_timings(struct v4l2_subdev *sd,
		const struct v4l2_bt_timings *bt) /* OK */
{
	struct adv7604_state *state = to_state(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 width = htotal(bt);
	u32 height = vtotal(bt);
	u16 cp_start_sav = bt->hsync + bt->hbackporch - 4;
	u16 cp_start_eav = width - bt->hfrontporch;
	u16 cp_start_vbi = height - bt->vfrontporch;
	u16 cp_end_vbi = bt->vsync + bt->vbackporch;
	u16 ch1_fr_ll = (((u32)bt->pixelclock / 100) > 0) ?
		((width * (ADV7604_fsc / 100)) / ((u32)bt->pixelclock / 100)) : 0;
	const u8 pll[2] = {
		0xc0 | ((width >> 8) & 0x1f),
		width & 0xff
	};

	v4l2_dbg(1, debug, sd, "%s\n", __func__);

	switch (state->mode) {
	case ADV7604_MODE_COMP:
	case ADV7604_MODE_GR:
		/* auto graphics */
		io_write(sd, 0x00, 0x07); /* video std */
		io_write(sd, 0x01, 0x02); /* prim mode */
		/* enable embedded syncs for auto graphics mode */
		cp_write_and_or(sd, 0x81, 0xef, 0x10);

		/* Should only be set in auto-graphics mode [REF_02, p. 91-92] */
		/* setup PLL_DIV_MAN_EN and PLL_DIV_RATIO */
		/* IO-map reg. 0x16 and 0x17 should be written in sequence */
		if (adv_smbus_write_i2c_block_data(client, 0x16, 2, pll)) {
			v4l2_err(sd, "writing to reg 0x16 and 0x17 failed\n");
			break;
		}

		/* active video - horizontal timing */
		cp_write(sd, 0xa2, (cp_start_sav >> 4) & 0xff);
		cp_write(sd, 0xa3, ((cp_start_sav & 0x0f) << 4) |
					((cp_start_eav >> 8) & 0x0f));
		cp_write(sd, 0xa4, cp_start_eav & 0xff);

		/* active video - vertical timing */
		cp_write(sd, 0xa5, (cp_start_vbi >> 4) & 0xff);
		cp_write(sd, 0xa6, ((cp_start_vbi & 0xf) << 4) |
					((cp_end_vbi >> 8) & 0xf));
		cp_write(sd, 0xa7, cp_end_vbi & 0xff);
		break;
	case ADV7604_MODE_HDMI:
		/* set default prim_mode/vid_std for HDMI
		   accoring to [REF_03, c. 4.2] */
		io_write(sd, 0x00, 0x02); /* video std */
		io_write(sd, 0x01, 0x06); /* prim mode */
		break;
	default:
		v4l2_dbg(2, debug, sd, "%s: Unknown mode %d\n",
				__func__, state->mode);
		break;
	}

	if (adv7604_has_afe(state)){
		cp_write(sd, 0x8f, (ch1_fr_ll >> 8) & 0x7);
		cp_write(sd, 0x90, ch1_fr_ll & 0xff);
		cp_write(sd, 0xab, (height >> 4) & 0xff);
		cp_write(sd, 0xac, (height & 0x0f) << 4);
	}
}

static void set_rgb_quantization_range(struct v4l2_subdev *sd) /* OK */
{
	struct adv7604_state *state = to_state(sd);

	if( (state->pdata.inp_color_space != ADV7604_INP_COLOR_SPACE_LIM_RGB )
		|| (state->pdata.inp_color_space != ADV7604_INP_COLOR_SPACE_FULL_RGB) )
		return; //let default value for input color space

	switch (state->rgb_quantization_range) {
	case V4L2_DV_RGB_RANGE_AUTO:
		/* automatic */
		if (DIGITAL_INPUT && !(hdmi_read(sd, 0x05) & 0x80)) {
			/* receiving DVI-D signal */

			/* ADV7604 selects RGB limited range regardless of
			   input format (CE/IT) in automatic mode */
			if (state->timings.bt.standards & V4L2_DV_BT_STD_CEA861) {
				/* RGB limited range (16-235) */
				io_write_and_or(sd, 0x02, 0x0f, 0x00);

			} else {
				/* RGB full range (0-255) */
				io_write_and_or(sd, 0x02, 0x0f, 0x10);
			}
		} else {
			/* receiving HDMI or analog signal, set automode */
			io_write_and_or(sd, 0x02, 0x0f, 0xf0);
		}
		break;
	case V4L2_DV_RGB_RANGE_LIMITED:
		/* RGB limited range (16-235) */
		io_write_and_or(sd, 0x02, 0x0f, 0x00);
		break;
	case V4L2_DV_RGB_RANGE_FULL:
		/* RGB full range (0-255) */
		io_write_and_or(sd, 0x02, 0x0f, 0x10);
		break;
	}
}

static int adv7604_s_ctrl(struct v4l2_ctrl *ctrl) /* OK */
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct adv7604_state *state = to_state(sd);

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		cp_write(sd, 0x3c, ctrl->val);
		return 0;
	case V4L2_CID_CONTRAST:
		cp_write(sd, 0x3a, ctrl->val);
		return 0;
	case V4L2_CID_SATURATION:
		cp_write(sd, 0x3b, ctrl->val);
		return 0;
	case V4L2_CID_HUE:
		cp_write(sd, 0x3d, ctrl->val);
		return 0;
	case  V4L2_CID_DV_RX_RGB_RANGE:
		state->rgb_quantization_range = ctrl->val;
		if( state->pdata.inp_color_space <= ADV7604_INP_COLOR_SPACE_FULL_RGB )
			set_rgb_quantization_range(sd);
		return 0;
	case V4L2_CID_ADV_RX_ANALOG_SAMPLING_PHASE:
		if (!adv7604_has_afe(state))
			return -EINVAL;
		/* Set the analog sampling phase. This is needed to find the
		   best sampling phase for analog video: an application or
		   driver has to try a number of phases and analyze the picture
		   quality before settling on the best performing phase. */
		afe_write(sd, 0xc8, ctrl->val);
		return 0;
	case V4L2_CID_ADV_RX_FREE_RUN_COLOR_MANUAL:
		/* Use the default blue color for free running mode,
		   or supply your own. */
		cp_write_and_or(sd, 0xbf, ~0x04, (ctrl->val << 2));
		return 0;
	case V4L2_CID_ADV_RX_FREE_RUN_COLOR:
		cp_write(sd, 0xc0, (ctrl->val & 0xff0000) >> 16);
		cp_write(sd, 0xc1, (ctrl->val & 0x00ff00) >> 8);
		cp_write(sd, 0xc2, (u8)(ctrl->val & 0x0000ff));
		return 0;
	}
	return -EINVAL;
}

static int adv7604_g_chip_ident(struct v4l2_subdev *sd,
					struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_ADV7604, 0);
}

/* ----------------------------------------------------------------------- */

static inline bool no_power(struct v4l2_subdev *sd) /* OK */
{
	/* Entire chip or CP powered off */
	return io_read(sd, 0x0c) & 0x24;
}

static inline bool no_signal_tmds(struct v4l2_subdev *sd) /* OK */
{
	/* TODO port B, C and D */
	return !(io_read(sd, 0x6a) & 0x10);
}

static inline bool no_lock_tmds(struct v4l2_subdev *sd) /* OK */
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;

	return (io_read(sd, 0x6a) & info->tdms_lock_mask) != info->tdms_lock_mask;
}

static inline bool no_lock_sspd(struct v4l2_subdev *sd) /* OK */
{
	struct adv7604_state *state = to_state(sd);

	/*
	 * Chips without a AFE don't expose registers for the SSPD, so just assume
	 * that we have a lock.
	 */
	if (!adv7604_has_afe(state))
		return false;

	/* TODO channel 2 */
	return ((cp_read(sd, 0xb5) & 0xd0) != 0xd0);
}

static inline bool no_lock_stdi(struct v4l2_subdev *sd) /* OK */
{
	/* TODO channel 2 */
	return !(cp_read(sd, 0xb1) & 0x80);
}

static inline bool no_signal(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);
	bool ret;

	ret = no_power(sd);

	ret |= no_lock_stdi(sd);
	ret |= no_lock_sspd(sd);

	if (DIGITAL_INPUT) {
		ret |= no_lock_tmds(sd);
		ret |= no_signal_tmds(sd);
	}

	return ret;
}

static inline bool no_lock_cp(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);

	if (!adv7604_has_afe(state))
		return false;

	/* CP has detected a non standard number of lines on the incoming
	   video compared to what it is configured to receive by s_dv_timings */
	return io_read(sd, 0x12) & 0x01;
}

static int adv7604_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct adv7604_state *state = to_state(sd);

	*status = 0;
	*status |= no_power(sd) ? V4L2_IN_ST_NO_POWER : 0;
	*status |= no_signal(sd) ? V4L2_IN_ST_NO_SIGNAL : 0;
	if (no_lock_cp(sd))
		*status |= DIGITAL_INPUT ? V4L2_IN_ST_NO_SYNC : V4L2_IN_ST_NO_H_LOCK;

	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

/* ----------------------------------------------------------------------- */

static void adv7604_print_timings(struct v4l2_subdev *sd,
	struct v4l2_dv_timings *timings, const char *txt, bool detailed)
{
	struct v4l2_bt_timings *bt = &timings->bt;
	u32 htot, vtot;

	if (timings->type != V4L2_DV_BT_656_1120)
		return;

	htot = htotal(bt);
	vtot = vtotal(bt);

	v4l2_info(sd, "%s %dx%d%s%d (%dx%d)",
			txt, bt->width, bt->height, bt->interlaced ? "i" : "p",
			(htot * vtot) > 0 ? ((u32)bt->pixelclock /
				(htot * vtot)) : 0,
			htot, vtot);

	if (detailed) {
		v4l2_info(sd, "    horizontal: fp = %d, %ssync = %d, bp = %d\n",
				bt->hfrontporch,
				(bt->polarities & V4L2_DV_HSYNC_POS_POL) ? "+" : "-",
				bt->hsync, bt->hbackporch);
		v4l2_info(sd, "    vertical: fp = %d, %ssync = %d, bp = %d\n",
				bt->vfrontporch,
				(bt->polarities & V4L2_DV_VSYNC_POS_POL) ? "+" : "-",
				bt->vsync, bt->vbackporch);
		v4l2_info(sd, "    pixelclock: %lld, flags: 0x%x, standards: 0x%x\n",
				bt->pixelclock, bt->flags, bt->standards);
	}
}

struct stdi_readback {
	u16 bl, lcf, lcvs;
	u8 hs_pol, vs_pol;
	bool interlaced;
};

static int stdi2dv_timings(struct v4l2_subdev *sd,
		struct stdi_readback *stdi,
		struct v4l2_dv_timings *timings)
{
	struct adv7604_state *state = to_state(sd);
	u32 hfreq = (ADV7604_fsc * 8) / stdi->bl;
	u32 pix_clk;
	int i;

	for (i = 0; adv7604_timings[i].bt.height; i++) {
		if (vtotal(&adv7604_timings[i].bt) != stdi->lcf + 1)
			continue;
		if (adv7604_timings[i].bt.vsync != stdi->lcvs)
			continue;

		pix_clk = hfreq * htotal(&adv7604_timings[i].bt);

		if ((pix_clk < adv7604_timings[i].bt.pixelclock + 1000000) &&
		    (pix_clk > adv7604_timings[i].bt.pixelclock - 1000000)) {
			*timings = adv7604_timings[i];
			return 0;
		}
	}

	if (v4l2_detect_cvt(stdi->lcf + 1, hfreq, stdi->lcvs,
			(stdi->hs_pol == '+' ? V4L2_DV_HSYNC_POS_POL : 0) |
			(stdi->vs_pol == '+' ? V4L2_DV_VSYNC_POS_POL : 0),
			timings))
		return 0;
	if (v4l2_detect_gtf(stdi->lcf + 1, hfreq, stdi->lcvs,
			(stdi->hs_pol == '+' ? V4L2_DV_HSYNC_POS_POL : 0) |
			(stdi->vs_pol == '+' ? V4L2_DV_VSYNC_POS_POL : 0),
			state->aspect_ratio, timings))
		return 0;

	v4l2_dbg(2, debug, sd,
		"%s: No format candidate found for lcvs = %d, lcf=%d, bl = %d, %chsync, %cvsync\n",
		__func__, stdi->lcvs, stdi->lcf, stdi->bl,
		stdi->hs_pol, stdi->vs_pol);
	return -1;
}


static int read_stdi(struct v4l2_subdev *sd, struct stdi_readback *stdi)
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;

	if (no_lock_stdi(sd) || no_lock_sspd(sd)) {
		v4l2_dbg(2, debug, sd, "%s: STDI and/or SSPD not locked\n", __func__);
		return -1;
	}

	/* read STDI */
	stdi->bl = cp_read16(sd, 0xb1, 0x3fff);
	stdi->lcf = cp_read16(sd, info->lcf_reg, 0x7ff);
	stdi->lcvs = cp_read(sd, 0xb3) >> 3;
	stdi->interlaced = io_read(sd, 0x12) & 0x10;

	if (adv7604_has_afe(state)) {
		/* read SSPD */
		if ((cp_read(sd, 0xb5) & 0x03) == 0x01) {
			stdi->hs_pol = ((cp_read(sd, 0xb5) & 0x10) ?
					((cp_read(sd, 0xb5) & 0x08) ? '+' : '-') : 'x');
			stdi->vs_pol = ((cp_read(sd, 0xb5) & 0x40) ?
					((cp_read(sd, 0xb5) & 0x20) ? '+' : '-') : 'x');
		} else {
			stdi->hs_pol = 'x';
			stdi->vs_pol = 'x';
		}
	} else {
		stdi->hs_pol = (hdmi_read(sd, 0x05) & 0x20) ? '+' : '-';
		stdi->vs_pol = (hdmi_read(sd, 0x05) & 0x10) ? '+' : '-';
	}

	if (no_lock_stdi(sd) || no_lock_sspd(sd)) {
		v4l2_dbg(2, debug, sd,
			"%s: signal lost during readout of STDI/SSPD\n", __func__);
		return -1;
	}

	if (stdi->lcf < 239 || stdi->bl < 8 || stdi->bl == 0x3fff) {
		v4l2_dbg(2, debug, sd, "%s: invalid signal\n", __func__);
		memset(stdi, 0, sizeof(struct stdi_readback));
		return -1;
	}

	v4l2_dbg(2, debug, sd,
		"%s: lcf (frame height - 1) = %d, bl = %d, lcvs (vsync) = %d, %chsync, %cvsync, %s\n",
		__func__, stdi->lcf, stdi->bl, stdi->lcvs,
		stdi->hs_pol, stdi->vs_pol,
		stdi->interlaced ? "interlaced" : "progressive");

	return 0;
}

static int adv7604_enum_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_enum_dv_timings *timings)
{
	if (timings->index >= ARRAY_SIZE(adv7604_timings) - 1)
		return -EINVAL;
	memset(timings->reserved, 0, sizeof(timings->reserved));
	timings->timings = adv7604_timings[timings->index];
	return 0;
}

static int adv7604_dv_timings_cap(struct v4l2_subdev *sd,
			struct v4l2_dv_timings_cap *cap)
{
	struct adv7604_state *state = to_state(sd);

	cap->type = V4L2_DV_BT_656_1120;
	cap->bt.max_width = 1920;
	cap->bt.max_height = 1200;
	cap->bt.min_pixelclock = 27000000;
	if (DIGITAL_INPUT)
		cap->bt.max_pixelclock = 225000000;
	else
		cap->bt.max_pixelclock = 170000000;
	cap->bt.standards = V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			 V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT;
	cap->bt.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE |
		V4L2_DV_BT_CAP_REDUCED_BLANKING | V4L2_DV_BT_CAP_CUSTOM;
	return 0;
}

/* Fill the optional fields .standards and .flags in struct v4l2_dv_timings
   if the format is listed in adv7604_timings[] */
static void adv7604_fill_optional_dv_timings_fields(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct adv7604_state *state = to_state(sd);
	int i;

	for (i = 0; adv7604_timings[i].bt.width; i++) {
		if (v4l_match_dv_timings(timings, &adv7604_timings[i],
					DIGITAL_INPUT ? 250000 : 1000000)) {
			*timings = adv7604_timings[i];
			break;
		}
	}
}

static unsigned int adv7604_read_hdmi_pixelclock(struct v4l2_subdev *sd)
{
	int a, b;

	a = hdmi_read(sd, 0x06);
	b = hdmi_read(sd, 0x3b);
	if (a < 0 || b < 0)
		return 0;
	return a * 1000000 + ((b & 0x30) >> 4) * 250000;
}

static unsigned int adv7611_read_hdmi_pixelclock(struct v4l2_subdev *sd)
{
	int a, b;

	a = hdmi_read(sd, 0x51);
	b = hdmi_read(sd, 0x52);
	if (a < 0 || b < 0)
		return 0;
	return ((a << 1) | (b >> 7)) * 1000000 + (b & 0x7f) * 1000000 / 128;
}

static int adv7604_query_dv_timings(struct v4l2_subdev *sd,
			struct v4l2_dv_timings *timings) /* OK */
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	struct v4l2_bt_timings *bt = &timings->bt;
	struct stdi_readback stdi;

	if (!timings){
		v4l2_dbg(1, debug, sd, "%s: invalide timings\n", __func__);
		return -EINVAL;
	}

	memset(timings, 0, sizeof(struct v4l2_dv_timings));

	if (no_signal(sd)) {
		v4l2_dbg(1, debug, sd, "%s: no valid signal\n", __func__);
		return -ENOLINK;
	}

	/* read STDI */
	if (read_stdi(sd, &stdi)) {
		v4l2_dbg(1, debug, sd, "%s: STDI/SSPD not locked\n", __func__);
		return -ENOLINK;
	}
	bt->interlaced = stdi.interlaced ?
		V4L2_DV_INTERLACED : V4L2_DV_PROGRESSIVE;

	if (DIGITAL_INPUT) {
		timings->type = V4L2_DV_BT_656_1120;

		bt->width = hdmi_read16(sd, 0x07, 0xfff); /* mask */
		bt->height = hdmi_read16(sd, 0x09, 0xfff);
		bt->pixelclock = info->read_hdmi_pixelclock(sd);
		bt->hfrontporch = hdmi_read16(sd, 0x20, 0x3ff); /* mask */
		bt->hsync = hdmi_read16(sd, 0x22, 0x3ff);
		bt->hbackporch = hdmi_read16(sd, 0x24, 0x3ff);
		bt->vfrontporch = hdmi_read16(sd, 0x2a, 0x1fff) / 2;
		bt->vsync = hdmi_read16(sd, 0x2e, 0x1fff) / 2;
		bt->vbackporch = hdmi_read16(sd, 0x32, 0x1fff) / 2;
		bt->polarities = ((hdmi_read(sd, 0x05) & 0x10) ? V4L2_DV_VSYNC_POS_POL : 0) |
			((hdmi_read(sd, 0x05) & 0x20) ? V4L2_DV_HSYNC_POS_POL : 0);
		if (bt->interlaced == V4L2_DV_INTERLACED) {
			bt->height += hdmi_read16(sd, 0x0b, 0xfff);
			bt->il_vfrontporch = hdmi_read16(sd, 0x2c, 0x1fff) / 2;
			bt->il_vsync = hdmi_read16(sd, 0x30, 0x1fff) / 2;
			bt->vbackporch = hdmi_read16(sd, 0x34, 0x1fff) / 2;
		}
		adv7604_fill_optional_dv_timings_fields(sd, timings);
	} else {
		/* find format
		 * Since LCVS values are inaccurate [REF_03, p. 275-276],
		 * stdi2dv_timings() is called with lcvs +-1 if the first attempt fails.
		 */
		if (!stdi2dv_timings(sd, &stdi, timings))
			goto found;
		stdi.lcvs += 1;
		v4l2_dbg(1, debug, sd, "%s: lcvs + 1 = %d\n", __func__, stdi.lcvs);
		if (!stdi2dv_timings(sd, &stdi, timings))
			goto found;
		stdi.lcvs -= 2;
		v4l2_dbg(1, debug, sd, "%s: lcvs - 1 = %d\n", __func__, stdi.lcvs);
		if (stdi2dv_timings(sd, &stdi, timings)) {
			/*
			 * The STDI block may measure wrong values, especially
			 * for lcvs and lcf. If the driver can not find any
			 * valid timing, the STDI block is restarted to measure
			 * the video timings again. The function will return an
			 * error, but the restart of STDI will generate a new
			 * STDI interrupt and the format detection process will
			 * restart.
			 */
			if (state->restart_stdi_once) {
				v4l2_dbg(1, debug, sd, "%s: restart STDI\n", __func__);
				/* TODO restart STDI for Sync Channel 2 */
				/* enter one-shot mode */
				cp_write_and_or(sd, 0x86, 0xf9, 0x00);
				/* trigger STDI restart */
				cp_write_and_or(sd, 0x86, 0xf9, 0x04);
				/* reset to continuous mode */
				cp_write_and_or(sd, 0x86, 0xf9, 0x02);
				state->restart_stdi_once = false;
				return -ENOLINK;
			}
			v4l2_dbg(1, debug, sd, "%s: format not supported\n", __func__);
			return -ERANGE;
		}
		state->restart_stdi_once = true;
	}
found:

	if (no_signal(sd)) {
		v4l2_dbg(1, debug, sd, "%s: signal lost during readout\n", __func__);
		memset(timings, 0, sizeof(struct v4l2_dv_timings));
		return -ENOLINK;
	}

	if ((!DIGITAL_INPUT && bt->pixelclock > 170000000) ||
			(DIGITAL_INPUT && bt->pixelclock > 225000000)) {
		v4l2_dbg(1, debug, sd, "%s: pixelclock out of range %d\n",
				__func__, (u32)bt->pixelclock);
		return -ERANGE;
	}

	if (debug > 1)
		adv7604_print_timings(sd, timings,
				"adv7604_query_dv_timings:", true);

	return 0;
}

static int adv7604_s_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings) /* OK */
{
	struct adv7604_state *state = to_state(sd);
	struct v4l2_bt_timings *bt;
	int err;

	if (!timings)
		return -EINVAL;

	bt = &timings->bt;

	if ((!DIGITAL_INPUT && bt->pixelclock > 170000000) ||
			(DIGITAL_INPUT && bt->pixelclock > 225000000)) {
		v4l2_dbg(1, debug, sd, "%s: pixelclock out of range %d\n",
				__func__, (u32)bt->pixelclock);
		return -ERANGE;
	}

	adv7604_fill_optional_dv_timings_fields(sd, timings);

	state->timings = *timings;

	cp_write_and_or(sd, 0x91, 0x40, bt->interlaced ? 0x40 : 0x00);

	/* Use prim_mode and vid_std when available */
	err = configure_predefined_video_timings(sd, timings);
	if (err) {
		/* custom settings when the video format
		 does not have prim_mode/vid_std */
		configure_custom_video_timings(sd, bt);
	}

	if( state->pdata.inp_color_space <= ADV7604_INP_COLOR_SPACE_FULL_RGB )
		set_rgb_quantization_range(sd);


	if (debug > 1)
		adv7604_print_timings(sd, timings,
				"adv7604_s_dv_timings:", true);
	return 0;
}

static int adv7604_g_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	struct adv7604_state *state = to_state(sd);

	*timings = state->timings;
	return 0;
}

static void adv7611_set_termination(struct v4l2_subdev *sd, bool enable)
{
	hdmi_write(sd, 0x83, enable ? 0xfe : 0xff);
}

static void adv7604_set_termination(struct v4l2_subdev *sd, bool enable)
{
	hdmi_write(sd, 0x01, enable ? 0x00 : 0x78);
}

static void enable_input(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);

	switch (state->mode) {
	case ADV7604_MODE_COMP:
	case ADV7604_MODE_GR:
		/* enable */
		io_write(sd, 0x15, 0xb0);   /* Disable Tristate of Pins (no audio) */
		break;
	case ADV7604_MODE_HDMI:
		/* enable */
		hdmi_write(sd, 0x1a, 0x0a); /* Unmute audio */
		state->info->set_termination(sd, true);
		io_write(sd, 0x15, adv7604_has_afe(state)? 0xa0: 0x80);   /* Disable Tristate of Pins */
		break;
	default:
		v4l2_dbg(2, debug, sd, "%s: Unknown mode %d\n",
				__func__, state->mode);
		break;
	}
}

static void disable_input(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);

	/* disable */
	io_write(sd, 0x15, 0xbe);   /* Tristate all outputs from video core */
	hdmi_write(sd, 0x1a, 0x1a); /* Mute audio */
	state->info->set_termination(sd, false);
}

static void select_input(struct v4l2_subdev *sd) /* OK */
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;

	switch (state->mode) {
	case ADV7604_MODE_COMP:
	case ADV7604_MODE_GR:
		adv7604_write_reg_seq(sd, info->recommended_settings[0],
				info->num_recommended_settings[0]);

		afe_write(sd, 0x00, 0x08); /* power up ADC */
		afe_write(sd, 0x01, 0x06); /* power up Analog Front End */
		afe_write(sd, 0xc8, 0x00); /* phase control */
		break;

	case ADV7604_MODE_HDMI:
		adv7604_write_reg_seq(sd, info->recommended_settings[1],
				info->num_recommended_settings[1]);

		if (adv7604_has_afe(state)) {
			afe_write(sd, 0x00, 0xff); /* power down ADC */
			afe_write(sd, 0x01, 0xfe); /* power down Analog Front End */
			afe_write(sd, 0xc8, 0x40); /* phase control */

		}

		cp_write(sd, 0x3e, 0x00); /* CP core pre-gain control */
		cp_write(sd, 0xc3, 0x39); /* CP coast control. Graphics mode */
		cp_write(sd, 0x40, 0x80); /* CP core pre-gain control. Graphics mode */

		break;
	default:
		v4l2_dbg(2, debug, sd, "%s: Unknown mode %d\n",
				__func__, state->mode);
		break;
	}
}

static int adv7604_s_routing(struct v4l2_subdev *sd,
		u32 input, u32 output, u32 config)
{
	struct adv7604_state *state = to_state(sd);

	v4l2_dbg(2, debug, sd, "%s: input %d", __func__, input);

	if (!adv7604_has_afe(state) && input != ADV7604_MODE_HDMI)
		return -EINVAL;

	state->mode = input;

	disable_input(sd);

	select_input(sd);

	enable_input(sd);

	return 0;
}

static int adv7604_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct adv7604_state *state = to_state(sd);
	struct adv7604_platform_data *pdata = &state->pdata;
	u8 llc_dll_double = 0;

	if (adv7604_has_afe(state))
		return 0;

	if( (state->mbus_type == V4L2_MBUS_BT656)
		&&
		((pdata->op_format_sel == ADV7604_OP_FORMAT_SEL_SDR_ITU656_8)
		||(pdata->op_format_sel == ADV7604_OP_FORMAT_SEL_DDR_422_8)
		||(pdata->op_format_sel == ADV7604_OP_FORMAT_SEL_SDR_ITU656_12_MODE2)) ){

		llc_dll_double = 1<<6;
	}

	// set or unset LLC_DLL_EN, LLC_DLL_DOUBLE, LLC_DLL_PHASE
	if( enable ){
		// TODO must be modified according to the output fromat selected
		// and perhaps output parallell ou BT656
		//see OP_FORMAT_SEL and LLC_DLL_PHASE for more details
		io_write(sd, 0x19, llc_dll_double | 0x86);

		io_write_and_or(sd, 0x33, 0xbf, 1<<6);//LLC_DLL_MUX
	}else{
		io_write(sd, 0x19, 0x0);
		io_write_and_or(sd, 0x33, 0xbf, 0x0);
	}
	return 0;
}

static int adv7604_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct adv7604_state *state = to_state(sd);

	if (code->index)
		return -EINVAL;

	//For BT656
	code->code = state->code;

	v4l2_dbg(1, debug, sd, "%s: code = 0x%04x\n",
		__func__, code->code);

	return 0;
}

static int adv7604_find_matching_dv_timings
	( struct v4l2_subdev *sd,unsigned int width, unsigned int height )
{
	int loop;
	int err = -1;
	struct v4l2_dv_timings timings;
	struct adv7604_state *state = to_state(sd);

	//Looks for available format format that matches
	//Set default standard to enable free run if there is no signal
	//According to what has been set as default width and height
	for (loop = 0; loop < ARRAY_SIZE(adv7604_prim_mode_comp); loop++) {

		if( (adv7604_prim_mode_comp[loop].timings.bt.width
				== width)
			&& ( adv7604_prim_mode_comp[loop].timings.bt.height
				== height) ){

			v4l2_dbg(1, debug, sd,
				"%s : found a matching value (comp, %d)\n",
					__func__, loop );

			timings = adv7604_prim_mode_comp[loop].timings;
			timings.bt.pixelclock += 125000;

			err = find_and_set_predefined_video_timings(sd,
				0x05, adv7604_prim_mode_hdmi_comp, &timings);
		}
	}

	if(loop == ARRAY_SIZE(adv7604_prim_mode_comp)){

		for (loop = 0; loop < ARRAY_SIZE(adv7604_prim_mode_gr); loop++) {

			if( (adv7604_prim_mode_gr[loop].timings.bt.width
					== width)
				&& ( adv7604_prim_mode_gr[loop].timings.bt.height
					== height) ){

				v4l2_dbg(1, debug, sd,
					"%s : found a matching value (gr, %d)\n",
					__func__, loop );

				timings = adv7604_prim_mode_gr[loop].timings;
				timings.bt.pixelclock += 125000;

				err = find_and_set_predefined_video_timings(sd,
					0x06, adv7604_prim_mode_hdmi_gr, &timings);
			}
		}
	}

	if( !err ) //Update default timings
		state->timings = timings;

	return err;
}


static int adv7604_s_mbus_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct adv7604_state *state = to_state(sd);

	if (fmt == NULL)
		return -EINVAL;

	if( !adv7604_has_afe(state)
		&& (fmt->width != state->timings.bt.width)
		&& (fmt->height != state->timings.bt.height) ){

		if( adv7604_find_matching_dv_timings( sd,
				fmt->width, fmt->height ) ){

				v4l2_dbg(2, debug, sd, "Unable to set default video standard\n");
		}
	}

	fmt->width = state->timings.bt.width;
	fmt->height = state->timings.bt.height;
	fmt->code = state->code;
	if (state->timings.bt.interlaced != V4L2_DV_INTERLACED)
		fmt->field = V4L2_FIELD_NONE;
	else
		fmt->field = V4L2_FIELD_INTERLACED;
	if (state->timings.bt.standards & V4L2_DV_BT_STD_CEA861) {
		fmt->colorspace = (state->timings.bt.height <= 576) ?
			V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;
	}

	v4l2_dbg(1, debug, sd, "%s: (width = %d, height = %d, field = %d, code=0x%04x)\n",
		__func__, fmt->width, fmt->height, fmt->field, fmt->code);

	return 0;
}

static struct v4l2_mbus_framefmt *
__adv7604_get_fmt(struct adv7604_state *state,
		  struct v4l2_subdev_fh *fh,
		  unsigned int pad,
		  enum v4l2_subdev_format_whence which)
{
	if(pad)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &state->format;
}

static int adv7604_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct adv7604_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	format = __adv7604_get_fmt(state, fh, fmt->pad, fmt->which);
	if(!format)
		return -EINVAL;

	ret = adv7604_s_mbus_fmt(sd, &fmt->format);
	if(ret < 0)
		return ret;

	*format = fmt->format;
	return 0;
}

static int adv7604_g_mbus_fmt(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct adv7604_state *state = to_state(sd);
	struct v4l2_dv_timings timings;

	if (fmt == NULL)
		return -EINVAL;

	//Ugly hack : Force detection and set DV timings
	memset(&timings, 0, sizeof(struct v4l2_dv_timings));
        if(!adv7604_query_dv_timings(sd, &timings))
		if( memcmp(&state->timings, &timings,
			sizeof(struct v4l2_dv_timings)) ){
			adv7604_s_dv_timings(sd, &timings);
		}

	fmt->width = state->timings.bt.width;
	fmt->height = state->timings.bt.height;
	fmt->code = state->code;
	if (state->timings.bt.interlaced != V4L2_DV_INTERLACED)
		fmt->field = V4L2_FIELD_NONE;
	else
		fmt->field = V4L2_FIELD_INTERLACED;
	if (state->timings.bt.standards & V4L2_DV_BT_STD_CEA861) {
		fmt->colorspace = (state->timings.bt.height <= 576) ?
			V4L2_COLORSPACE_SMPTE170M : V4L2_COLORSPACE_REC709;
	}

	v4l2_dbg(1, debug, sd, "%s: (width = %d, height = %d, field = %d, code=0x%04x)\n",
		__func__, fmt->width, fmt->height, fmt->field, fmt->code);

	return 0;
}

static int adv7604_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct adv7604_state *state = to_state(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	format = __adv7604_get_fmt(state, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	ret = adv7604_g_mbus_fmt(sd, &fmt->format);
	if(ret < 0)
		return ret;

	*format = fmt->format;
	return 0;
}

static int adv7604_g_mbus_config(struct v4l2_subdev *sd,
					struct v4l2_mbus_config *cfg)
{
	//fake for the moment
	//struct i2c_client *client   = v4l2_get_subdevdata(sd);
	struct adv7604_state *state = to_state(sd);

	cfg->flags =	V4L2_MBUS_PCLK_SAMPLE_RISING |
			V4L2_MBUS_VSYNC_ACTIVE_HIGH  |
			V4L2_MBUS_HSYNC_ACTIVE_HIGH  |
			V4L2_MBUS_DATA_ACTIVE_HIGH   |
			V4L2_MBUS_MASTER;

	cfg->type  = state->mbus_type;

	v4l2_dbg(2, debug, sd, "%s: v4l2_mbus_config type %s\n flags = 0x%x ( %s%s%s%s%s%s%s%s )\n",
		__func__,
		(cfg->type==V4L2_MBUS_PARALLEL) ? "V4L2_MBUS_PARALLEL":"V4L2_MBUS_BT656",
		cfg->flags,
		(cfg->flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)? " V4L2_MBUS_HSYNC_ACTIVE_HIGH ": "",
		(cfg->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)? "| V4L2_MBUS_HSYNC_ACTIVE_LOW ": "",
		(cfg->flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)? "| V4L2_MBUS_VSYNC_ACTIVE_HIGH ": "",
		(cfg->flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)? "| V4L2_MBUS_VSYNC_ACTIVE_LOW ": "",
		(cfg->flags & V4L2_MBUS_PCLK_SAMPLE_RISING)? "| V4L2_MBUS_PCLK_SAMPLE_RISING ": "",
		(cfg->flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)? "| V4L2_MBUS_PCLK_SAMPLE_FALLING ": "",
		(cfg->flags & V4L2_MBUS_DATA_ACTIVE_HIGH)? "| V4L2_MBUS_DATA_ACTIVE_HIGH ": "",
		(cfg->flags & V4L2_MBUS_DATA_ACTIVE_LOW)? "| V4L2_MBUS_DATA_ACTIVE_LOW ": "");

	return 0;
}

static int adv7604_s_mbus_config(struct v4l2_subdev *sd,
				const struct v4l2_mbus_config *cfg)
{
	//fake for the moment
	if( (cfg->type != V4L2_MBUS_PARALLEL) &&
		(cfg->type != V4L2_MBUS_BT656) )
		return -EINVAL;

	//TODO change the values of register to be compliant with the configuration
	v4l2_dbg(2, debug, sd, "%s: v4l2_mbus_config type %s\n flags = 0x%x ( %s%s%s%s%s%s%s%s )\n",
		__func__,
		(cfg->type==V4L2_MBUS_PARALLEL) ? "V4L2_MBUS_PARALLEL":"V4L2_MBUS_BT656",
		cfg->flags,
		(cfg->flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH)? " V4L2_MBUS_HSYNC_ACTIVE_HIGH ": "",
		(cfg->flags & V4L2_MBUS_HSYNC_ACTIVE_LOW)? "| V4L2_MBUS_HSYNC_ACTIVE_LOW ": "",
		(cfg->flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH)? "| V4L2_MBUS_VSYNC_ACTIVE_HIGH ": "",
		(cfg->flags & V4L2_MBUS_VSYNC_ACTIVE_LOW)? "| V4L2_MBUS_VSYNC_ACTIVE_LOW ": "",
		(cfg->flags & V4L2_MBUS_PCLK_SAMPLE_RISING)? "| V4L2_MBUS_PCLK_SAMPLE_RISING ": "",
		(cfg->flags & V4L2_MBUS_PCLK_SAMPLE_FALLING)? "| V4L2_MBUS_PCLK_SAMPLE_FALLING ": "",
		(cfg->flags & V4L2_MBUS_DATA_ACTIVE_HIGH)? "| V4L2_MBUS_DATA_ACTIVE_HIGH ": "",
		(cfg->flags & V4L2_MBUS_DATA_ACTIVE_LOW)? "| V4L2_MBUS_DATA_ACTIVE_LOW ": "");

	//io_write(sd, 0x06, 0x2f);   /* Field output on VS/FIELD pin, Positive polarity HS, VS, Field, LLC */
	// INV_LLC_POL, IO Map, Address 0x06, [0] A control to select the polarity of the LLC.
	// INV_HS_POL, IO, Address 0x06[1]	A control to select the polarity of the HS signal.
	// INV_VS_POL, IO, Address 0x06[2]	A control to select the polarity of the VS/FIELD/ALSB signal.
	// INV_F_POL, IO, Address 0x06[3]	A control to select the polarity of the DE signal.
	return 0;
}

static int adv7604_isr(struct v4l2_subdev *sd, u32 status, bool *handled)
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	u8 fmt_change, fmt_change_digital, tx_5v;

	/* format change */
	fmt_change = io_read(sd, 0x43) & 0x98;
	if (fmt_change)
		io_write(sd, 0x44, fmt_change);
	fmt_change_digital = DIGITAL_INPUT ? (io_read(sd, 0x6b) & info->fmt_change_digital_mask) : 0;
	if (fmt_change_digital)
		io_write(sd, 0x6c, fmt_change_digital);
	if (fmt_change || fmt_change_digital) {
		v4l2_dbg(1, debug, sd,
			"%s: ADV7604_FMT_CHANGE, fmt_change = 0x%x, fmt_change_digital = 0x%x\n",
			__func__, fmt_change, fmt_change_digital);
		v4l2_subdev_notify(sd, ADV7604_FMT_CHANGE, NULL);
		if (handled)
			*handled = true;
	}
	/* tx 5v detect */
	tx_5v = io_read(sd, 0x70) & info->cable_det_mask;
	if (tx_5v) {
		v4l2_dbg(1, debug, sd, "%s: tx_5v: 0x%x\n", __func__, tx_5v);
		io_write(sd, 0x71, tx_5v);
		adv7604_s_detect_tx_5v_ctrl(sd);
		if (handled)
			*handled = true;
	}
	return 0;
}

static int adv7604_get_edid(struct v4l2_subdev *sd, struct v4l2_subdev_edid *edid)
{
	struct adv7604_state *state = to_state(sd);

	if (edid->pad != 0)
		return -EINVAL;
	if (edid->blocks == 0)
		return -EINVAL;
	if (edid->start_block >= state->edid_blocks)
		return -EINVAL;
	if (edid->start_block + edid->blocks > state->edid_blocks)
		edid->blocks = state->edid_blocks - edid->start_block;
	if (!edid->edid)
		return -EINVAL;
	memcpy(edid->edid + edid->start_block * 128,
	       state->edid + edid->start_block * 128,
	       edid->blocks * 128);
	return 0;
}

static int adv7604_set_edid(struct v4l2_subdev *sd, struct v4l2_subdev_edid *edid)
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	int err;

	if (edid->pad != 0)
		return -EINVAL;

	if (edid->start_block != 0)
		return -EINVAL;

	if (edid->blocks == 0) {
		/* Pull down the hotplug pin */
		v4l2_subdev_notify(sd, ADV7604_HOTPLUG, (void *)0);
		/* Disables I2C access to internal EDID ram from DDC port */
		rep_write_and_or(sd, info->edid_ctrl_reg, 0xf0, 0x0);
		state->edid_blocks = 0;
		/* Fall back to a 16:9 aspect ratio */
		state->aspect_ratio.numerator = 16;
		state->aspect_ratio.denominator = 9;
		return 0;
	}

	if (edid->blocks > 2)
		return -E2BIG;

	if (!edid->edid)
		return -EINVAL;

	memcpy(state->edid, edid->edid, 128 * edid->blocks);
	state->edid_blocks = edid->blocks;
	state->aspect_ratio = v4l2_calc_aspect_ratio(edid->edid[0x15],
			edid->edid[0x16]);
	err = edid_write_block(sd, 128 * edid->blocks, state->edid);
	if (err < 0)
		v4l2_err(sd, "error %d writing edid\n", err);
	else
		v4l2_dbg(2, debug, sd, "Writing edid succed\n");

	return err;
}

/*********** avi info frame CEA-861-E **************/

static void print_avi_infoframe(struct v4l2_subdev *sd)
{
	int i;
	u8 buf[14];
	u8 avi_len;
	u8 avi_ver;

	if (!(hdmi_read(sd, 0x05) & 0x80)) {
		v4l2_info(sd, "receive DVI-D signal (AVI infoframe not supported)\n");
		return;
	}
	if (!(io_read(sd, 0x60) & 0x01)) {
		v4l2_info(sd, "AVI infoframe not received\n");
		return;
	}

	if (io_read(sd, 0x83) & 0x01) {
		v4l2_info(sd, "AVI infoframe checksum error has occurred earlier\n");
		io_write(sd, 0x85, 0x01); /* clear AVI_INF_CKS_ERR_RAW */
		if (io_read(sd, 0x83) & 0x01) {
			v4l2_info(sd, "AVI infoframe checksum error still present\n");
			io_write(sd, 0x85, 0x01); /* clear AVI_INF_CKS_ERR_RAW */
		}
	}

	avi_len = infoframe_read(sd, 0xe2);
	avi_ver = infoframe_read(sd, 0xe1);
	v4l2_info(sd, "AVI infoframe version %d (%d byte)\n",
			avi_ver, avi_len);

	if (avi_ver != 0x02)
		return;

	for (i = 0; i < 14; i++)
		buf[i] = infoframe_read(sd, i);

	v4l2_info(sd,
		"\t%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]);
}

static int adv7604_log_status(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	struct v4l2_dv_timings timings;
	struct stdi_readback stdi;
	u8 reg_io_0x02 = io_read(sd, 0x02);

	static const char * const csc_coeff_sel_rb[16] = {
		"bypassed", "YPbPr601 -> RGB", "reserved", "YPbPr709 -> RGB",
		"reserved", "RGB -> YPbPr601", "reserved", "RGB -> YPbPr709",
		"reserved", "YPbPr709 -> YPbPr601", "YPbPr601 -> YPbPr709",
		"reserved", "reserved", "reserved", "reserved", "manual"
	};
	static const char * const input_color_space_txt[16] = {
		"RGB limited range (16-235)", "RGB full range (0-255)",
		"YCbCr Bt.601 (16-235)", "YCbCr Bt.709 (16-235)",
		"XvYCC Bt.601", "XvYCC Bt.709",
		"YCbCr Bt.601 (0-255)", "YCbCr Bt.709 (0-255)",
		"invalid", "invalid", "invalid", "invalid", "invalid",
		"invalid", "invalid", "automatic"
	};
	static const char * const rgb_quantization_range_txt[] = {
		"Automatic",
		"RGB limited range (16-235)",
		"RGB full range (0-255)",
	};

	v4l2_info(sd, "-----Chip status-----\n");
	v4l2_info(sd, "Chip power: %s\n", no_power(sd) ? "off" : "on");
	v4l2_info(sd, "Connector type: %s\n", state->connector_hdmi ?
			"HDMI" : (DIGITAL_INPUT ? "DVI-D" : "DVI-A"));
	v4l2_info(sd, "EDID: %s\n", ((rep_read(sd, info->edid_ctrl_reg) & 0x01) &&
			(rep_read(sd, info->edid_status_reg) & 0x01)) ? "enabled" : "disabled ");
	v4l2_info(sd, "CEC: %s\n", !!(cec_read(sd, 0x2a) & 0x01) ?
			"enabled" : "disabled");

	v4l2_info(sd, "-----Signal status-----\n");
	v4l2_info(sd, "Cable detected (+5V power): %s\n",
			(io_read(sd, 0x6f) & info->cable_det_mask) ? "true" : "false");
	v4l2_info(sd, "TMDS signal detected: %s\n",
			no_signal_tmds(sd) ? "false" : "true");
	v4l2_info(sd, "TMDS signal locked: %s\n",
			no_lock_tmds(sd) ? "false" : "true");
	v4l2_info(sd, "SSPD locked: %s\n", no_lock_sspd(sd) ? "false" : "true");
	v4l2_info(sd, "STDI locked: %s\n", no_lock_stdi(sd) ? "false" : "true");
	v4l2_info(sd, "CP locked: %s\n", no_lock_cp(sd) ? "false" : "true");
	v4l2_info(sd, "CP free run: %s\n",
			(!!(cp_read(sd, 0xff) & 0x10) ? "on" : "off"));
	v4l2_info(sd, "Prim-mode = 0x%x, video std = 0x%x, v_freq = 0x%x\n",
			io_read(sd, 0x01) & 0x0f, io_read(sd, 0x00) & 0x3f,
			(io_read(sd, 0x01) & 0x70) >> 4);

	v4l2_info(sd, "-----Video Timings-----\n");
	if (read_stdi(sd, &stdi))
		v4l2_info(sd, "STDI: not locked\n");
	else
		v4l2_info(sd, "STDI: lcf (frame height - 1) = %d, bl = %d, lcvs (vsync) = %d, %s, %chsync, %cvsync\n",
				stdi.lcf, stdi.bl, stdi.lcvs,
				stdi.interlaced ? "interlaced" : "progressive",
				stdi.hs_pol, stdi.vs_pol);
	if (adv7604_query_dv_timings(sd, &timings))
		v4l2_info(sd, "No video detected\n");
	else
		adv7604_print_timings(sd, &timings, "Detected format:", true);
	adv7604_print_timings(sd, &state->timings, "Configured format:", true);

	v4l2_info(sd, "-----Color space-----\n");
	v4l2_info(sd, "RGB quantization range ctrl: %s\n",
			rgb_quantization_range_txt[state->rgb_quantization_range]);
	v4l2_info(sd, "Input color space: %s\n",
			input_color_space_txt[reg_io_0x02 >> 4]);
	v4l2_info(sd, "Output color space: %s %s, saturator %s\n",
			(reg_io_0x02 & 0x02) ? "RGB" : "YCbCr",
			(reg_io_0x02 & 0x04) ? "(16-235)" : "(0-255)",
			((reg_io_0x02 & 0x04) ^ (reg_io_0x02 & 0x01)) ?
					"enabled" : "disabled");
	v4l2_info(sd, "Color space conversion: %s\n",
			csc_coeff_sel_rb[cp_read(sd, 0xfc) >> 4]);

	/* Digital video */
	if (DIGITAL_INPUT) {
		v4l2_info(sd, "-----HDMI status-----\n");
		v4l2_info(sd, "HDCP encrypted content: %s\n",
				hdmi_read(sd, 0x05) & 0x40 ? "true" : "false");

		print_avi_infoframe(sd);
	}

	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops adv7604_ctrl_ops = {
	.s_ctrl = adv7604_s_ctrl,
};

static const struct v4l2_subdev_core_ops adv7604_core_ops = {
	.log_status = adv7604_log_status,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
	.g_chip_ident = adv7604_g_chip_ident,
	.interrupt_service_routine = adv7604_isr,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = adv7604_g_register,
	.s_register = adv7604_s_register,
#endif
};

static const struct v4l2_subdev_video_ops adv7604_video_ops = {
	.s_stream = adv7604_s_stream,
	.s_routing = adv7604_s_routing,
	.g_input_status = adv7604_g_input_status,
	.s_dv_timings = adv7604_s_dv_timings,
	.g_dv_timings = adv7604_g_dv_timings,
	.query_dv_timings = adv7604_query_dv_timings,
	.enum_dv_timings = adv7604_enum_dv_timings,
	.dv_timings_cap = adv7604_dv_timings_cap,
	.g_mbus_config = adv7604_g_mbus_config,
	.s_mbus_config = adv7604_s_mbus_config,
};

static const struct v4l2_subdev_pad_ops adv7604_pad_ops = {
	.get_edid = adv7604_get_edid,
	.set_edid = adv7604_set_edid,
	.get_fmt  = adv7604_get_fmt,
	.set_fmt  = adv7604_set_fmt,
	.enum_mbus_code = adv7604_enum_mbus_code,
};

static const struct v4l2_subdev_ops adv7604_ops = {
	.core = &adv7604_core_ops,
	.video = &adv7604_video_ops,
	.pad = &adv7604_pad_ops,
};

/* -------------------------- custom ctrls ---------------------------------- */

static const struct v4l2_ctrl_config adv7604_ctrl_analog_sampling_phase = {
	.ops = &adv7604_ctrl_ops,
	.id = V4L2_CID_ADV_RX_ANALOG_SAMPLING_PHASE,
	.name = "Analog Sampling Phase",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0x1f,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config adv7604_ctrl_free_run_color_manual = {
	.ops = &adv7604_ctrl_ops,
	.id = V4L2_CID_ADV_RX_FREE_RUN_COLOR_MANUAL,
	.name = "Free Running Color, Manual",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = false,
	.max = true,
	.step = 1,
	.def = false,
};

static const struct v4l2_ctrl_config adv7604_ctrl_free_run_color = {
	.ops = &adv7604_ctrl_ops,
	.id = V4L2_CID_ADV_RX_FREE_RUN_COLOR,
	.name = "Free Running Color",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0x0,
	.max = 0xffffff,
	.step = 0x1,
	.def = 0x0,
};

/* ----------------------------------------------------------------------- */

static int adv7604_core_init(struct v4l2_subdev *sd)
{
	struct adv7604_state *state = to_state(sd);
	const struct adv7604_chip_info *info = state->info;
	struct adv7604_platform_data *pdata = &state->pdata;

	hdmi_write(sd, 0x48,
		(pdata->disable_pwrdnb ? 0x80 : 0) |
		(pdata->disable_cable_det_rst ? 0x40 : 0));

	disable_input(sd);

	/* power */
	io_write(sd, 0x0c, 0x42);   /* Power up part and power down VDP */
	io_write(sd, 0x0b, 0x44);   /* Power down ESDP block */
	cp_write(sd, 0xcf, 0x01);   /* Power down macrovision */

	/* video format */
	io_write_and_or(sd, 0x02, 0xf0,
			pdata->alt_gamma << 3 |
			pdata->op_656_range << 2 |
			pdata->rgb_out << 1 |
			pdata->alt_data_sat << 0);
	io_write(sd, 0x03, pdata->op_format_sel);
	io_write_and_or(sd, 0x04, 0x1f, pdata->op_ch_sel << 5);
	io_write_and_or(sd, 0x05, 0xf0, pdata->blank_data << 3 |
					pdata->insert_av_codes << 2 |
					pdata->replicate_av_codes << 1 |
					pdata->invert_cbcr << 0);

	/* TODO from platform data */
	if (adv7604_has_afe(state)){
		cp_write(sd, 0x69, 0x30);   /* Enable CP CSC */
		io_write(sd, 0x06, 0xa6);   /* positive VS and HS */
	}else
		io_write(sd, 0x06, 0x2f);   /* Field output on VS/FIELD pin, Positive polarity HS, VS, Field, LLC */

	io_write(sd, 0x14, 0x7f);   /* Drive strength adjusted to max */
	cp_write(sd, 0xba, (pdata->hdmi_free_run_mode << 1) | 0x01); /* HDMI free run */
	cp_write(sd, 0xf3, 0xdc); /* Low threshold to enter/exit free run mode */
	cp_write(sd, 0xf9, 0x23); /*  STDI ch. 1 - LCVS change threshold -
				      ADI recommended setting [REF_01, c. 2.3.3] */

	if (adv7604_has_afe(state)){
		cp_write(sd, 0x45, 0x23); /*  STDI ch. 2 - LCVS change threshold -
					      ADI recommended setting [REF_01, c. 2.3.3] */
	}

	cp_write(sd, 0xc9, 0x2d); /* use prim_mode and vid_std as free run resolution
					for digital formats */

	/* TODO from platform data */
	afe_write(sd, 0xb5, 0x01);  /* Setting MCLK to 256Fs */

	if (adv7604_has_afe(state)) {
		afe_write(sd, 0x02, pdata->ain_sel); /* Select analog input muxing mode */
		io_write_and_or(sd, 0x30, ~(1 << 4), pdata->output_bus_lsb_to_msb << 4);
	}

	/* interrupts */
	//io_write(sd, 0x40, 0xc0 | pdata->int1_config); /* Configure INT1, remains untill flags is cleared */
	io_write(sd, 0x40, 0x80 | pdata->int1_config); /* Configure INT1, 64 Xtal periods */
	io_write(sd, 0x73, info->cable_det_mask); /* Enable CABLE_DET_A_ST (+5v) interrupt */

	if (!adv7604_has_afe(state)){ /* Audio */
		io_write(sd, 0x69, 0x80); /* Enable CS_DATA_VALID_ST interrupt */
		io_write(sd, 0x87, 0x8); /* Enable NEW_SAMP_RT_ST interrupt */
	}

	io_write(sd, 0x46, 0x98); /* Enable SSPD, STDI and CP unlocked interrupts */
	io_write(sd, 0x6e, info->fmt_change_digital_mask); /* Enable V_LOCKED and DE_REGEN_LCK interrupts */
	info->setup_irqs(sd);

	return v4l2_ctrl_handler_setup(sd->ctrl_handler);
}

static void adv7604_setup_irqs(struct v4l2_subdev *sd)
{
	io_write(sd, 0x41, 0xd7); /* STDI irq for any change, disable INT2 */
}

static void adv7611_setup_irqs(struct v4l2_subdev *sd)
{
	io_write(sd, 0x41, 0xd0); /* STDI irq for any change, disable INT2 */
}

static void adv7604_unregister_clients(struct adv7604_state *state)
{
	if (state->i2c_avlink)
		i2c_unregister_device(state->i2c_avlink);
	if (state->i2c_cec)
		i2c_unregister_device(state->i2c_cec);
	if (state->i2c_infoframe)
		i2c_unregister_device(state->i2c_infoframe);
	if (state->i2c_esdp)
		i2c_unregister_device(state->i2c_esdp);
	if (state->i2c_dpp)
		i2c_unregister_device(state->i2c_dpp);
	if (state->i2c_afe)
		i2c_unregister_device(state->i2c_afe);
	if (state->i2c_repeater)
		i2c_unregister_device(state->i2c_repeater);
	if (state->i2c_edid)
		i2c_unregister_device(state->i2c_edid);
	if (state->i2c_hdmi)
		i2c_unregister_device(state->i2c_hdmi);
	if (state->i2c_test)
		i2c_unregister_device(state->i2c_test);
	if (state->i2c_cp)
		i2c_unregister_device(state->i2c_cp);
	if (state->i2c_vdp)
		i2c_unregister_device(state->i2c_vdp);
}

static struct i2c_client *adv7604_dummy_client(struct v4l2_subdev *sd,
							u8 addr, u8 io_reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (addr)
		if( io_write(sd, io_reg, addr << 1) < 0)
			return NULL; //no device detected
	return i2c_new_dummy(client->adapter, io_read(sd, io_reg) >> 1);
}

static const struct adv7604_reg_seq adv7604_recommended_settings_afe[] = {
		/* reset ADI recommended settings for HDMI: */
		/* "ADV7604 Register Settings Recommendations (rev. 2.5, June 2010)" p. 4. */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x0d), 0x04 }, /* HDMI filter optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x0d), 0x04 }, /* HDMI filter optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x3d), 0x00 }, /* DDC bus active pull-up control */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x3e), 0x74 }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x4e), 0x3b }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x57), 0x74 }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x58), 0x63 }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x8d), 0x18 }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x8e), 0x34 }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x93), 0x88 }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x94), 0x2e }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x96), 0x00 }, /* enable automatic EQ changing */

		/* set ADI recommended settings for digitizer */
		/* "ADV7604 Register Settings Recommendations (rev. 2.5, June 2010)" p. 17. */
		{ ADV7604_REG(ADV7604_PAGE_AFE, 0x12), 0x7b }, /* ADC noise shaping filter controls */
		{ ADV7604_REG(ADV7604_PAGE_AFE, 0x0c), 0x1f }, /* CP core gain controls */
		{ ADV7604_REG(ADV7604_PAGE_CP, 0x3e), 0x04 }, /* CP core pre-gain control */
		{ ADV7604_REG(ADV7604_PAGE_CP, 0xc3), 0x39 }, /* CP coast control. Graphics mode */
		{ ADV7604_REG(ADV7604_PAGE_CP, 0x40), 0x5c }, /* CP core pre-gain control. Graphics mode */
};

static const struct adv7604_reg_seq adv7604_recommended_settings_hdmi[] = {
		/* set ADI recommended settings for HDMI: */
		/* "ADV7604 Register Settings Recommendations (rev. 2.5, June 2010)" p. 4. */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x0d), 0x84 }, /* HDMI filter optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x3d), 0x10 }, /* DDC bus active pull-up control */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x3e), 0x39 }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x4e), 0x3b }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x57), 0xb6 }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x58), 0x03 }, /* TMDS PLL optimization */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x8d), 0x18 }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x8e), 0x34 }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x93), 0x8b }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x94), 0x2d }, /* equaliser */
		{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x96), 0x01 }, /* enable automatic EQ changing */

		/* reset ADI recommended settings for digitizer */
		/* "ADV7604 Register Settings Recommendations (rev. 2.5, June 2010)" p. 17. */
		{ ADV7604_REG(ADV7604_PAGE_AFE, 0x12), 0xfb }, /* ADC noise shaping filter controls */
		{ ADV7604_REG(ADV7604_PAGE_AFE, 0x0c), 0x0d }, /* CP core gain controls */
};

static const struct adv7604_reg_seq adv7611_recommended_settings_hdmi[] = {
	{ ADV7604_REG(ADV7604_PAGE_CP, 0x6c), 0x00 },
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x03), 0x98 },
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x4c), 0x44 },
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x8d), 0x04 },
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x8e), 0x1e },
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x6c), 0x26 }, /* HPA_AUTO_INT_EDID, HPA_DELAY_SEL */
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x9b), 0x03 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc1), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc2), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc3), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc4), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc5), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc6), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc7), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc8), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xc9), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xca), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xcb), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0xcc), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x6f), 0x0c },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x85), 0x1f },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x87), 0x70 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x57), 0xda },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x58), 0x01 },// ADI recommended setting
	{ ADV7604_REG(ADV7604_PAGE_HDMI, 0x75), 0x10 },// DDC drive strength
};

static const struct adv7604_chip_info adv7604_chip_info[] = {
	[ADV7604] = {
		.has_afe = true,
		.tdms_lock_mask = 0xe0,
		.fmt_change_digital_mask = 0xc0,
		.set_termination = adv7604_set_termination,
		.setup_irqs = adv7604_setup_irqs,
		.read_hdmi_pixelclock = adv7604_read_hdmi_pixelclock,
		.recommended_settings = {
		    [0] = adv7604_recommended_settings_afe,
		    [1] = adv7604_recommended_settings_hdmi,
		},
		.num_recommended_settings = {
		    [0] = ARRAY_SIZE(adv7604_recommended_settings_afe),
		    [1] = ARRAY_SIZE(adv7604_recommended_settings_hdmi),
		},
		.page_mask = BIT(ADV7604_PAGE_IO) | BIT(ADV7604_PAGE_AVLINK) |
			BIT(ADV7604_PAGE_CEC) | BIT(ADV7604_PAGE_INFOFRAME) |
			BIT(ADV7604_PAGE_ESDP) | BIT(ADV7604_PAGE_DPP) |
			BIT(ADV7604_PAGE_AFE) | BIT(ADV7604_PAGE_REP) |
			BIT(ADV7604_PAGE_EDID) | BIT(ADV7604_PAGE_HDMI) |
			BIT(ADV7604_PAGE_TEST) | BIT(ADV7604_PAGE_CP) |
			BIT(ADV7604_PAGE_VDP),
	},
	[ADV7611] = {
		.has_afe = false,
		.edid_ctrl_reg = 0x74,
		.edid_status_reg = 0x76,
		.lcf_reg = 0xa3,
		.tdms_lock_mask = 0x43,
		.cable_det_mask = 0x01,
		.fmt_change_digital_mask = 0x03,
		.set_termination = adv7611_set_termination,
		.read_hdmi_pixelclock = adv7611_read_hdmi_pixelclock,
		.setup_irqs = adv7611_setup_irqs,
		.recommended_settings = {
		    [1] = adv7611_recommended_settings_hdmi,
		},
		.num_recommended_settings = {
		    [1] = ARRAY_SIZE(adv7611_recommended_settings_hdmi),
		},
		.page_mask = BIT(ADV7604_PAGE_IO) | BIT(ADV7604_PAGE_CEC) |
			BIT(ADV7604_PAGE_INFOFRAME) | BIT(ADV7604_PAGE_AFE) |
			BIT(ADV7604_PAGE_REP) |  BIT(ADV7604_PAGE_EDID) |
			BIT(ADV7604_PAGE_HDMI) | BIT(ADV7604_PAGE_CP),
	},
};


static irqreturn_t adv7604_irq(int irq, void *dev_id)
{
	struct adv7604_state *state = dev_id;
	struct v4l2_subdev *sd = &state->sd;
	const struct adv7604_chip_info *info = state->info;
	u8 fmt_change = 0, fmt_change_digital = 0, tx_5v = 0, regval = 0;
	u8 cs_data_valid_st = 0, new_samp_rt_st = 0;

	/* format change */
	fmt_change = io_read(sd, 0x43) & 0x98;
	if (fmt_change)
		io_write(sd, 0x44, fmt_change);

	regval = io_read(sd, 0x6b);
	fmt_change_digital = DIGITAL_INPUT ? (io_read(sd, 0x6b) & info->fmt_change_digital_mask) : 0;

	//Signal
	//TMDS_CLK_A_ST, IO, Address 0x6B[4] // see TMDS_CLK_A_RAW  no_signal_tmds
	//V_LOCKED_ST, IO, Address 0x6B[1]
	//TMDSPLL_LCK_A_ST, IO, Address 0x6B[6] //see TMDSPLL_LCK_A_RAW, IO, Address 0x6A[6]
	//

	//no_lock_tmds see Table 29. HDMI Flags in IO Map Register 0x6A
	//V_LOCKED_RAW, IO, Address 0x6A[1]
	//DE_REGEN_LCK_RAW, IO, Address 0x6A[0]
	//TMDSPLL_LCK_A_RAW, IO, Address 0x6A[6]

	if (fmt_change_digital)
		io_write(sd, 0x6c, fmt_change_digital);
	if (fmt_change || fmt_change_digital) {
		state->fmt_change = 1;
		sysfs_notify_dirent(state->fmt);
	}
	/* tx 5v detect */
	tx_5v = io_read(sd, 0x70) & info->cable_det_mask;
	if (tx_5v) {
		io_write(sd, 0x71, tx_5v);
		//adv7604_s_detect_tx_5v_ctrl(sd);//TODO perhaps should try a delayed work
		sysfs_notify_dirent(state->cable);
	}

	/* Audio  */
	cs_data_valid_st = io_read(sd, 0x66) & 0x80;
	if( cs_data_valid_st ){
		io_write(sd, 0x67, cs_data_valid_st );//no necessary to clear
		sysfs_notify_dirent(state->audio);
	}

	new_samp_rt_st = io_read(sd, 0x84) & 0x8;
	if( new_samp_rt_st ){
		io_write(sd, 0x85, new_samp_rt_st );
		sysfs_notify_dirent(state->audio);
	}

	return IRQ_HANDLED;
}


static int adv7604_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct adv7604_state *state;
	struct adv7604_platform_data *pdata = NULL;
	struct v4l2_ctrl_handler *hdl;
	struct v4l2_subdev *sd;
	int err, loop;
	char gpio_name[50] = { 0 };
	pdata = client->dev.platform_data;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_dbg(1, debug, client, "detecting %s client on address 0x%x\n",
			id->name,
			client->addr << 1);

	state = kzalloc(sizeof(struct adv7604_state), GFP_KERNEL);
	if (!state) {
		v4l_err(client, "Could not allocate adv7604_state memory!\n");
		return -ENOMEM;
	}

	state->info = &adv7604_chip_info[id->driver_data];

	if( id->driver_data == ADV7611 )
		state->mode = ADV7604_MODE_HDMI;

	/* platform data */
	if (!pdata) {
		v4l_err(client, "No platform data!\n");
		err = -ENODEV;
		goto err_state;
	}
	memcpy(&state->pdata, pdata, sizeof(state->pdata));

	if (pdata->power_on) {
		err = pdata->power_on();
		if (err) {
			v4l_err(client, "power on failed\n");
			goto err_state;
		}
	}

	state->timings.bt.width = pdata->default_width;
	state->timings.bt.height = pdata->default_height;
	if (!adv7604_has_afe(state))
		state->timings.bt.standards = V4L2_DV_BT_STD_CEA861;

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &adv7604_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->connector_hdmi = pdata->connector_hdmi;

#if 0
	/* i2c access to adv7604? */
	if (adv_smbus_read_byte_data_check(client, 0xfb, false) != 0x68) {
		v4l2_info(sd, "not an adv7604 on address 0x%x\n",
				client->addr << 1);
		err = -ENODEV;
		goto err_pwr;
	}
#endif

	/* control handlers */
	hdl = &state->hdl;
	v4l2_ctrl_handler_init(hdl, adv7604_has_afe(state) ? 9 : 8);

	v4l2_ctrl_new_std(hdl, &adv7604_ctrl_ops,
			V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &adv7604_ctrl_ops,
			V4L2_CID_CONTRAST, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &adv7604_ctrl_ops,
			V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &adv7604_ctrl_ops,
			V4L2_CID_HUE, 0, 128, 1, 0);

	/* private controls */
	state->detect_tx_5v_ctrl = v4l2_ctrl_new_std(hdl, NULL,
			V4L2_CID_DV_RX_POWER_PRESENT, 0, 1, 0, 0);
	state->detect_tx_5v_ctrl->is_private = true;

	if( state->pdata.inp_color_space <= ADV7604_INP_COLOR_SPACE_FULL_RGB ){
		state->rgb_quantization_range_ctrl =
			v4l2_ctrl_new_std_menu(hdl, &adv7604_ctrl_ops,
				V4L2_CID_DV_RX_RGB_RANGE, V4L2_DV_RGB_RANGE_FULL,
				0, V4L2_DV_RGB_RANGE_AUTO);
		state->rgb_quantization_range_ctrl->is_private = true;
	}

	/* custom controls */
	if (adv7604_has_afe(state)) {
		state->analog_sampling_phase_ctrl =
			v4l2_ctrl_new_custom(hdl, &adv7604_ctrl_analog_sampling_phase, NULL);
		state->analog_sampling_phase_ctrl->is_private = true;
	}
	state->free_run_color_manual_ctrl =
		v4l2_ctrl_new_custom(hdl, &adv7604_ctrl_free_run_color_manual, NULL);
	state->free_run_color_manual_ctrl->is_private = true;
	state->free_run_color_ctrl =
		v4l2_ctrl_new_custom(hdl, &adv7604_ctrl_free_run_color, NULL);
	state->free_run_color_ctrl->is_private = true;

	sd->ctrl_handler = hdl;
	if (hdl->error) {
		err = hdl->error;
		goto err_hdl;
	}
	if (adv7604_s_detect_tx_5v_ctrl(sd)) {
		err = -ENODEV;
		goto err_hdl;
	}

	state->i2c_cec = adv7604_dummy_client(sd, pdata->i2c_cec, 0xf4);
	state->i2c_infoframe = adv7604_dummy_client(sd, pdata->i2c_infoframe, 0xf5);
	state->i2c_afe = adv7604_dummy_client(sd, pdata->i2c_afe, 0xf8);
	state->i2c_repeater = adv7604_dummy_client(sd, pdata->i2c_repeater, 0xf9);
	state->i2c_edid = adv7604_dummy_client(sd, pdata->i2c_edid, 0xfa);
	state->i2c_hdmi = adv7604_dummy_client(sd, pdata->i2c_hdmi, 0xfb);
	state->i2c_cp = adv7604_dummy_client(sd, pdata->i2c_cp, 0xfd);
	if (!state->i2c_cec || !state->i2c_infoframe || !state->i2c_afe ||
	    !state->i2c_repeater || !state->i2c_edid || !state->i2c_hdmi ||
	    !state->i2c_cp) {
		err = -ENOMEM;
		v4l2_err(sd, "failed to create all i2c clients\n");
		goto err_i2c;
	}
	if (adv7604_has_afe(state)) {
		state->i2c_avlink = adv7604_dummy_client(sd, pdata->i2c_avlink, 0xf3);
		state->i2c_esdp = adv7604_dummy_client(sd, pdata->i2c_esdp, 0xf6);
		state->i2c_dpp = adv7604_dummy_client(sd, pdata->i2c_dpp, 0xf7);
		state->i2c_test = adv7604_dummy_client(sd, pdata->i2c_test, 0xfc);
		state->i2c_vdp = adv7604_dummy_client(sd, pdata->i2c_vdp, 0xfe);
		if (!state->i2c_avlink || !state->i2c_esdp || !state->i2c_dpp ||
			!state->i2c_test || !state->i2c_vdp) {
			err = -ENOMEM;
			v4l2_err(sd, "failed to create all i2c clients\n");
			goto err_i2c;
		}
	}
	state->restart_stdi_once = true;

	/* work queues */
	state->work_queues = create_singlethread_workqueue(client->name);
	if (!state->work_queues) {
		v4l2_err(sd, "Could not create work queue\n");
		err = -ENOMEM;
		goto err_i2c;
	}

	INIT_DELAYED_WORK(&state->delayed_work_enable_hotplug,
			adv7604_delayed_work_enable_hotplug);

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_init(&sd->entity, 1, &state->pad, 0);
	if (err)
		goto err_work_queues;

	if(pdata->insert_av_codes ){
		state->mbus_type = V4L2_MBUS_BT656;

		if(!pdata->op_656_range)//Force range
			pdata->op_656_range = 1;
	}else
		state->mbus_type = V4L2_MBUS_PARALLEL;

	state->code = adv7604_get_mbus_pixelcode( pdata->op_ch_sel,
			pdata->op_format_sel,
			pdata->invert_cbcr);

	err = adv7604_core_init(sd);
	if (err)
		goto err_entity;
	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
			client->addr << 1, client->adapter->name);

	if(sd && !adv7604_has_afe(state)) // Force routing
		adv7604_s_routing(sd, ADV7604_MODE_HDMI, 0, 0);

        for (loop = 0; loop < ARRAY_SIZE(adv7611_sysfs_attrs); loop++) {
                err = device_create_file(&client->dev, &adv7611_sysfs_attrs[loop]);
                if (err) {
			goto sysfs_err;
                }
        }

	state->cable = sysfs_get_dirent(client->dev.kobj.sd, NULL, "cable");
	if(!state->cable){
		err = -ENODEV;
		goto sysfs_err;
	}
	state->signal = sysfs_get_dirent(client->dev.kobj.sd, NULL, "signal");
	if(!state->signal){
		err = -ENODEV;
		goto sysfs_err;
	}
	state->fmt = sysfs_get_dirent(client->dev.kobj.sd, NULL, "fmt");
	if(!state->fmt){
		err = -ENODEV;
		goto sysfs_err;
	}

	state->audio = sysfs_get_dirent(client->dev.kobj.sd, NULL, "audio");
	if(!state->audio){
		err = -ENODEV;
		goto sysfs_err;
	}

	if( pdata->cam_it != -1 ){
		v4l2_err(sd, "request IRQ gpio %d\n", pdata->cam_it);
		if (!gpio_is_valid(pdata->cam_it)){
			err = -EINVAL;
			v4l2_err(sd, "Invalide GPIO %d\n",
					pdata->cam_it);
			goto sysfs_err;
		}
		snprintf(gpio_name, ARRAY_SIZE(gpio_name), "%s cam_it", client->name);
		state->irq = gpio_to_irq(pdata->cam_it);


		err = request_threaded_irq(state->irq, NULL, adv7604_irq,
			          IRQF_TRIGGER_FALLING,
				  client->name, state);

		if (err < 0) {
			v4l2_err(sd, "request IRQ %d failed\n",
					state->irq);
			goto erqstirq;
		}
	}

	state->fmt_change = 0;

	return 0;

erqstirq:
sysfs_err:
	if(state->cable)
		sysfs_put(state->cable);

	if(state->signal)
		sysfs_put(state->signal);

	if(state->fmt)
		sysfs_put(state->fmt);

	if(state->audio)
		sysfs_put(state->audio);

        for (loop = 0; loop < ARRAY_SIZE(adv7611_sysfs_attrs); loop++) {
                device_remove_file(&client->dev,
					   &adv7611_sysfs_attrs[loop]);
	}

err_entity:
	media_entity_cleanup(&sd->entity);
err_work_queues:
	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	destroy_workqueue(state->work_queues);
err_i2c:
	adv7604_unregister_clients(state);
err_hdl:
	v4l2_ctrl_handler_free(hdl);
err_state:
	kfree(state);
	return err;
}

/* ----------------------------------------------------------------------- */

static int adv7604_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct adv7604_state *state = to_state(sd);
	int loop;

	if( state->pdata.cam_it != -1 ){
		free_irq(state->irq, state);
	}

	if(state->cable)
		sysfs_put(state->cable);

	if(state->signal)
		sysfs_put(state->signal);

	if(state->fmt)
		sysfs_put(state->fmt);

	if(state->audio)
		sysfs_put(state->audio);

        for (loop = 0; loop < ARRAY_SIZE(adv7611_sysfs_attrs); loop++) {
                device_remove_file(&client->dev,
					   &adv7611_sysfs_attrs[loop]);
	}

	cancel_delayed_work(&state->delayed_work_enable_hotplug);
	destroy_workqueue(state->work_queues);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	adv7604_unregister_clients(to_state(sd));
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	if (state->pdata.power_off)
		state->pdata.power_off();
	kfree(to_state(sd));
	return 0;
}

/* ----------------------------------------------------------------------- */

static struct i2c_device_id adv7604_id[] = {
	{ "adv7604", ADV7604 },
	{ "adv7611", ADV7611 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7604_id);

static struct i2c_driver adv7604_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "adv7604",
	},
	.probe = adv7604_probe,
	.remove = adv7604_remove,
	.id_table = adv7604_id,
};

module_i2c_driver(adv7604_driver);
