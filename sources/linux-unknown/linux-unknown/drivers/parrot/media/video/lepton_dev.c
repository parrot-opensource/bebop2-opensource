/*
 * Lepton I2C driver
 *
 * Copyright 2014 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include "lepton.h"

MODULE_AUTHOR("Damien Riegel <damien.riegel.ext@parrot.com>");
MODULE_DESCRIPTION("Lepton camera I2C controller");
MODULE_LICENSE("GPL");

#define LEPTON_STATUS_REG       ((u16)0x0002)
#define LEPTON_CMD_REG          ((u16)0x0004)
#define LEPTON_DATA_LENGTH_REG  ((u16)0x0006)
#define LEPTON_DATA0_REG        ((u16)0x0008)

#define LEPTON_CMD_MOD_AGC      ((u16) (0x100))
#define LEPTON_CMD_MOD_SYS      ((u16) (0x200))
#define LEPTON_CMD_MOD_VID      ((u16) (0x300))
#define LEPTON_CMD_MOD_OEM      ((u16) (0x800 | (1 << 14)))
#define LEPTON_CMD_MOD_RAD      ((u16) (0xE00 | (1 << 14)))

#define LEPTON_CMD_TYP_GET      ((u16) 0x0)
#define LEPTON_CMD_TYP_SET      ((u16) 0x1)
#define LEPTON_CMD_TYP_RUN      ((u16) 0x2)

#define LEPTON_CMD_OEM_SEL_FMT  (LEPTON_CMD_MOD_OEM | 0x28)
#define LEPTON_CMD_OEM_SEL_SRC  (LEPTON_CMD_MOD_OEM | 0x2C)
#define LEPTON_CMD_AGC_ENABLE   (LEPTON_CMD_MOD_AGC | 0x00)
#define LEPTON_CMD_AGC_POLICY   (LEPTON_CMD_MOD_AGC | 0x04)
#define LEPTON_CMD_SYS_TELE_ENABLE      (LEPTON_CMD_MOD_SYS | 0x18)
#define LEPTON_CMD_SYS_TELE_LOC         (LEPTON_CMD_MOD_SYS | 0x1C)
#define LEPTON_CMD_RAD_ENABLE	(LEPTON_CMD_MOD_RAD | 0x10)
#define LEPTON_CMD_RAD_SHUTTER_MODE		(LEPTON_CMD_MOD_RAD | 0x24)
#define LEPTON_CMD_RAD_SHUTTER_TEMP		(LEPTON_CMD_MOD_RAD | 0x28)
#define LEPTON_CMD_RAD_FFC		(LEPTON_CMD_MOD_RAD | 0x2C)

#define LEPTON_CMD_ID_SHIFT     2

enum lepton_telemetry {
	LEPTON_TELEMETRY_DISABLED,
	LEPTON_TELEMETRY_FIRST,
	LEPTON_TELEMETRY_LAST,
};

struct lepton_subdev {
	struct v4l2_subdev              sd;
	struct v4l2_ctrl_handler        ctrl_handler;

	struct i2c_client               *client;
	struct v4l2_mbus_framefmt       format;

	enum lepton_telemetry           tele;
	struct v4l2_ctrl                *tele_ctrl;

	int                             gpio_pwr;
	int                             gpio_rst;
	int                             gpio_clken;
};

#define to_lepton(_sd) container_of(_sd, struct lepton_subdev, sd)

struct lepton_subdev_format {
	enum v4l2_mbus_pixelcode        code;
	/* conf is the value to be given to the sensor to switch
	 * to this format */
	u32                             conf;
};

#define LEPTON_IMAGE_WIDTH    (80)
#define LEPTON_IMAGE_HEIGHT   (60)
#define LEPTON_TELEMETRY_LINES (3)

struct lepton_subdev_format formats[] = {
	/* format in first position is the default format */
	{
		.code = V4L2_MBUS_FMT_Y14_2X8_PADHI_LE, /* RAW14 */
		.conf = 0x7,
	},
	{
		.code = V4L2_MBUS_FMT_Y8_1X8,           /* RAW8  */
		.conf = 0x0,
	},
	{
		.code = V4L2_MBUS_FMT_RGB888_1X24,      /* RGB24 */
		.conf = 0x3,
	},
};

u32 lepton_init_regs[] = {
	LEPTON_CMD_OEM_SEL_FMT,     7,  /* set format to RAW14 */
	LEPTON_CMD_OEM_SEL_SRC,     1,  /* set output source to COOKED */
	LEPTON_CMD_SYS_TELE_ENABLE, 1,  /* enable telemetry */
	LEPTON_CMD_SYS_TELE_LOC,	1,  /* set telemetry location to footer */
};

static u32 get_image_height(u32 mbus_code, bool telemetry)
{
	u32 h = LEPTON_IMAGE_HEIGHT;

	if (telemetry) {
		h += LEPTON_TELEMETRY_LINES;
		if (mbus_code == V4L2_MBUS_FMT_Y8_1X8)
			h += LEPTON_TELEMETRY_LINES;
	}

	return h;
}

/*************
 * I2C helpers
 *************/
static int lepton_transfer(struct lepton_subdev *dev, u16 reg_addr,
                           u8 *data, size_t cnt, bool read)
{
	struct i2c_client *cl = dev->client;
	struct i2c_msg  msg[2];
	u8 reg[] = { (reg_addr >> 8) & 0xFF, reg_addr & 0xFF };
	int err;

	msg[0].addr = cl->addr;
	msg[0].flags = 0;
	msg[0].len = (u16) sizeof(reg_addr);
	msg[0].buf = (u8*) &reg;

	msg[1].addr = cl->addr;
	msg[1].flags = read ? I2C_M_RD : I2C_M_NOSTART;
	msg[1].len = cnt;
	msg[1].buf = data;

	err = i2c_transfer(cl->adapter, msg, ARRAY_SIZE(msg));
	if (err == ARRAY_SIZE(msg))
		return 0;
	else if (err < 0) {
		return err;
	}
	else
		return -EIO;
}

static int lepton_transfer16(struct lepton_subdev *dev, u16 reg_addr,
                             u16 *data, size_t cnt, bool read)
{
	u8 buff[cnt * sizeof(*data)];
	int err, i;

	if (!read) {
		for (i = 0; i < cnt; i++) {
			buff[2 * i] = (data[i] >> 8) & 0xFF;
			buff[2 * i + 1] = data[i] & 0xFF;
		}
	}

	err = lepton_transfer(dev, reg_addr, buff, cnt * sizeof(*data), read);
	if (err)
		return err;

	if (read) {
		for (i = 0; i < cnt; i++) {
			data[i] = buff[2 * i] << 8 | buff[2 * i + 1];
		}
	}

	return err;
}

static int lepton_poll_busy(struct lepton_subdev *dev, s8 *error_code)
{
	unsigned long timeout = jiffies + 2 * HZ;
	u16 status;
	int err;

	do {
		if (time_is_before_jiffies(timeout)) {
			err = -ETIMEDOUT;
			break;
		}

		err = lepton_transfer16(dev, LEPTON_STATUS_REG, &status, 1, true);
		if (err)
			break;


		if (status & 0x1)
			err = -EBUSY;
		else if (error_code)
			*error_code = status >> 8;

	} while (err);

	return err;
}

static int lepton_get(struct lepton_subdev *dev, u16 cmd, u16 *data, size_t cnt)
{
	int err;
	s8 error_code;
	u16 length = (u16) cnt;

	err = lepton_poll_busy(dev, NULL);
	if (err)
		return err;

	err = lepton_transfer16(dev, LEPTON_DATA_LENGTH_REG, &length, 1, false);
	if (err)
		return err;

	err = lepton_transfer16(dev, LEPTON_CMD_REG, &cmd, 1, false);
	if (err)
		return err;

	err = lepton_poll_busy(dev, &error_code);
	if (err)
		return err;
	else if (error_code)
		return error_code;

	err = lepton_transfer16(dev, LEPTON_DATA0_REG, data, cnt, true);
	return err;
}

static int lepton_get32(struct lepton_subdev *dev, u16 cmd, u32 *data, size_t cnt)
{
	u16 buff[2 * cnt];
	int err, i;

	err = lepton_get(dev, cmd, buff, 2 * cnt);
	if (err)
		return err;

	for (i = 0; i < cnt; i++) {
		data[i] = buff[2 * i + 1] << 16 | buff[2 * i];
	}

	return 0;
}

static int lepton_set(struct lepton_subdev *dev, u16 cmd, u16 *data, size_t cnt)
{
	int err;
	s8 error_code;
	u16 length = (u16) cnt;

	err = lepton_poll_busy(dev, NULL);
	if (err)
		return err;

	err = lepton_transfer16(dev, LEPTON_DATA0_REG, data, cnt, false);
	if (err)
		return err;

	err = lepton_transfer16(dev, LEPTON_DATA_LENGTH_REG, &length, 1, false);
	if (err)
		return err;

	err = lepton_transfer16(dev, LEPTON_CMD_REG, &cmd, 1, false);
	if (err)
		return err;

	err = lepton_poll_busy(dev, &error_code);
	if (err)
		return err;
	else
		return error_code;
}

static int lepton_set32(struct lepton_subdev *dev, u16 cmd, u32 *data, size_t cnt)
{
	u16 buff[2 * cnt];
	int i;

	for (i = 0; i < cnt; i++) {
		buff[2 * i] = data[i] & 0xFFFF;
		buff[2 * i + 1] = (data[i] >> 16) & 0xFFFF;
	}

	return lepton_set(dev, cmd, buff, 2 * cnt);
}

#if 0
static int lepton_send_cmd(struct lepton_subdev *dev, u16 cmd)
{
	int err;
	s8 error_code;

	err = lepton_poll_busy(dev, NULL);
	if (err)
		return err;

	err = lepton_transfer16(dev, LEPTON_CMD_REG, &cmd, 1, false);
	if (err)
		return err;

	err = lepton_poll_busy(dev, &error_code);
	if (err)
		return err;
	else
		return error_code;
}
#endif

/*****************
 * V4L2 operations
 *****************/
static void lepton_power_on(struct lepton_subdev *lepton)
{
	if (lepton->gpio_pwr != -1) {
		gpio_set_value(lepton->gpio_pwr, 1);
		/* must wait 5000 clock cycles after power up,
		 * round up to 1ms */
		msleep(1);
	}
}

static void lepton_power_off(struct lepton_subdev *lepton)
{
	if (lepton->gpio_pwr != -1)
		gpio_set_value(lepton->gpio_pwr, 0);
}

static int lepton_hard_reset(struct v4l2_subdev *sd, u32 val)
{
	struct lepton_subdev *lepton = to_lepton(sd);

	if (lepton->gpio_rst != -1) {
		// Reset
		gpio_set_value(lepton->gpio_rst, 0);
		msleep(1);
		// Release reset
		gpio_set_value(lepton->gpio_rst, 1);
		msleep(1200);
	}

	return 0;
}

static int lepton_set_sensor_format(struct lepton_subdev *lepton)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].code == lepton->format.code) {
			u32 buff = formats[i].conf;
			u16 cmd;

			cmd = LEPTON_CMD_OEM_SEL_FMT;
			cmd |= LEPTON_CMD_TYP_SET;
			return lepton_set32(lepton, cmd, &buff, 1);
		}
	}

	return -EINVAL;
}

static struct lepton_subdev_format* get_format(enum v4l2_mbus_pixelcode code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].code == code)
			return &formats[i];
	}

	return NULL;
}

static struct lepton_subdev_format* get_format_by_conf(u32 conf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].conf == conf)
			return &formats[i];
	}

	return NULL;
}

static int lepton_init(struct lepton_subdev *lepton)
{
	int i, err;
	u16 cmd;
	u32 buff, read_buff;

	for (i = 0; i < ARRAY_SIZE(lepton_init_regs); i += 2) {
		cmd = lepton_init_regs[i];
		buff = lepton_init_regs[i + 1];

		cmd |= LEPTON_CMD_TYP_SET;
		err = lepton_set32(lepton, cmd, &buff, 1);
		if (err) {
			v4l2_err(&lepton->sd,
				 "failed to set lepton reg 0x%X\n", cmd);
			return err;
		}

		cmd = lepton_init_regs[i];
		cmd |= LEPTON_CMD_TYP_GET;
		err = lepton_get32(lepton, cmd, &read_buff, 1);
		if (err) {
			v4l2_err(&lepton->sd,
				 "failed to read lepton reg 0x%X\n", cmd);
			return err;
		}

		if (read_buff != buff) {
			v4l2_err(&lepton->sd, "failed to set lepton register "
				 "value [0x%X] is 0x%X instead of 0x%X\n",
				 cmd, read_buff, buff);
			return -EINVAL;
		}
	}

	cmd = LEPTON_CMD_OEM_SEL_FMT | LEPTON_CMD_TYP_GET;
	err = lepton_get32(lepton, cmd, &buff, 1);
	if (err) {
		v4l2_err(&lepton->sd, "failed to get format (%d)\n", err);
		return err;
	}
	else {
		struct v4l2_mbus_framefmt *fmt = &lepton->format;
		struct lepton_subdev_format *sub_fmt;

		sub_fmt = get_format_by_conf(buff);
		if (!sub_fmt) {
			v4l2_warn(&lepton->sd, "unknown format found %d\n",
				  buff);
			fmt->code = formats[0].code;
			err = lepton_set_sensor_format(lepton);
			if (err) {
				v4l2_err(&lepton->sd,
				         "can't set a valid sensor format\n");
				return -EINVAL;
			}

		}
		else {
			fmt->code = sub_fmt->code;
		}
	}

	// Save initializing telemetry value (lepton_init_regs)
	lepton->tele = LEPTON_TELEMETRY_LAST;

	return 0;
}

static int lepton_reset(struct v4l2_subdev *sd, u32 val)
{
	struct lepton_subdev *lepton = to_lepton(sd);

	lepton_hard_reset(sd, val);
	lepton_init(lepton);

	return 0;
}

static int lepton_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct lepton_subdev *lepton = to_lepton(sd);
	int err = 0;

	if (enable) {
		err = lepton_set_sensor_format(lepton);
	}
	v4l2_ctrl_grab(lepton->tele_ctrl, enable);

	return err;
}

static int lepton_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
                                enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(formats))
		return -EINVAL;

	*code = formats[index].code;
	return 0;
}

static int lepton_g_mbus_fmt(struct v4l2_subdev *sd,
                             struct v4l2_mbus_framefmt *fmt)
{
	struct lepton_subdev *lepton = to_lepton(sd);

	if (fmt == NULL)
		return -EINVAL;

	/* update image height, that's one thing that can
	 * have changed if telemetry has been toggled */
	lepton->format.height = get_image_height(lepton->format.code,
	                                         lepton->tele_ctrl->cur.val);
	*fmt = lepton->format;
	return 0;
}

static int lepton_try_mbus_fmt(struct v4l2_subdev *sd,
	                       struct v4l2_mbus_framefmt *fmt)
{
	struct lepton_subdev *lepton = to_lepton(sd);
	struct lepton_subdev_format *lep_fmt;

	fmt->width = LEPTON_IMAGE_WIDTH;
	fmt->height = get_image_height(lepton->format.code,
	                               lepton->tele_ctrl->cur.val);
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	lep_fmt = get_format(fmt->code);
	if (!lep_fmt)
		lep_fmt = &formats[0];

	fmt->code = lep_fmt->code;
	return 0;
}

static int lepton_s_mbus_fmt(struct v4l2_subdev *sd,
                             struct v4l2_mbus_framefmt *fmt)
{
	struct lepton_subdev *lepton = to_lepton(sd);
	int err;

	err = lepton_try_mbus_fmt(sd, fmt);
	if (err)
		return err;

	lepton->format = *fmt;

	return 0;
}

/***************
 * V4L2 controls
 ***************/
static int enable_agc(struct lepton_subdev *lepton, s32 enable)
{
	u16 cmd = LEPTON_CMD_AGC_ENABLE | LEPTON_CMD_TYP_SET;
	u32 value = enable ? 1 : 0;

	return lepton_set32(lepton, cmd, &value, 1);
}

static int set_agc(struct lepton_subdev *lepton, s32 val)
{
	u16 cmd = LEPTON_CMD_AGC_POLICY | LEPTON_CMD_TYP_SET;
	u32 value = val;

	return lepton_set32(lepton, cmd, &value, 1);
}

static int enable_telemetry(struct lepton_subdev *lepton, s32 enable)
{
	u16 cmd = LEPTON_CMD_SYS_TELE_ENABLE | LEPTON_CMD_TYP_SET;
	u32 value = enable ? 1 : 0;

	return lepton_set32(lepton, cmd, &value, 1);
}

static int set_telemetry(struct lepton_subdev *lepton, s32 val)
{
	u16 cmd = LEPTON_CMD_SYS_TELE_LOC | LEPTON_CMD_TYP_SET;
	u32 value = val;

	return lepton_set32(lepton, cmd, &value, 1);
}

static int enable_rad(struct lepton_subdev *lepton, s32 enable)
{
	u16 cmd = LEPTON_CMD_RAD_ENABLE | LEPTON_CMD_TYP_SET;
	u32 value = enable ? 1 : 0;

	return lepton_set32(lepton, cmd, &value, 1);
}

static int do_ffc(struct lepton_subdev *lepton)
{
	u16 cmd = LEPTON_CMD_RAD_FFC | LEPTON_CMD_TYP_RUN;
	u32 value = 0;

	return lepton_set32(lepton, cmd, &value, 0);
}

static int set_shutter_mode(struct lepton_subdev *lepton, s32 mode)
{
	u16 cmd = LEPTON_CMD_RAD_SHUTTER_MODE | LEPTON_CMD_TYP_SET;
	u32 value = (u32)mode;

	return lepton_set32(lepton, cmd, &value, 1);
}

static int set_shutter_temp(struct lepton_subdev *lepton, s32 temperature)
{
	u16 cmd = LEPTON_CMD_RAD_SHUTTER_TEMP | LEPTON_CMD_TYP_SET;
	u16 value = (u16) temperature;

	return lepton_set(lepton, cmd, &value, 1);
}

#define V4L2_CID_LEPTON_BASE    (V4L2_CID_USER_BASE | 0xf000)
#define V4L2_CID_LEPTON_AGC_POLICY (V4L2_CID_LEPTON_BASE + 1)
#define V4L2_CID_LEPTON_TELEMETRY  (V4L2_CID_LEPTON_BASE + 2)
#define V4L2_CID_LEPTON_FFC  	   (V4L2_CID_LEPTON_BASE + 3)

static int lepton_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int err = 0;
	struct lepton_subdev *lepton =
		container_of(ctrl->handler, struct lepton_subdev, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_AUTOGAIN:
		err = enable_agc(lepton, ctrl->val);
		break;
	case V4L2_CID_LEPTON_AGC_POLICY:
		err = set_agc(lepton, ctrl->val);
		break;
	case V4L2_CID_LEPTON_TELEMETRY:
		err = enable_telemetry(lepton, ctrl->val);
		if (err)
			break;
		if (ctrl->val)
			err = set_telemetry(lepton, ctrl->val - 1);

		lepton->tele =  ctrl->val; // Save the telemetry configuration
		break;
	case V4L2_CID_LEPTON_FFC:
		err = enable_rad(lepton, 1); // Enable radiometry
		if(err)
			break;
		err = set_shutter_mode(lepton, 0); // Set shutter to user mode
		if(err)
			break;
		err = set_shutter_temp(lepton, ctrl->val); // Set calib shutter temp
		if(err)
			break;
		err = do_ffc(lepton); // Trigger FFC calibration

		// Reset control value so that the handler is called next time
		ctrl->cur.val = ctrl->val = 0;
		break;
	default:
		err = -EINVAL;
	}
	return err;
}

const struct v4l2_ctrl_ops lepton_ctrl_ops = {
	.s_ctrl = lepton_s_ctrl,
};

static const char* agc_policy_opts[] = {
	"linear",
	"histogram equalization",
	NULL,
};

static struct v4l2_ctrl_config lepton_agc_policy = {
	.ops    = &lepton_ctrl_ops,
	.id     = V4L2_CID_LEPTON_AGC_POLICY,
	.name   = "AGC Policy",
	.type   = V4L2_CTRL_TYPE_MENU,
	.min    = 0,
	.max    = 1,
	.def    = 0,
	.qmenu  = agc_policy_opts,
};

static const char* telemetry_opts[] = {
	"disable",
	"first",
	"last",
	NULL,
};

static struct v4l2_ctrl_config lepton_telemetry_cfg = {
	.ops    = &lepton_ctrl_ops,
	.id     = V4L2_CID_LEPTON_TELEMETRY,
	.name   = "Telemetry",
	.type   = V4L2_CTRL_TYPE_MENU,
	.min    = 0,
	.max    = 2,
	.def    = 2,
	.qmenu  = telemetry_opts,
};

static struct v4l2_ctrl_config lepton_ffc_cfg = {
	.ops    = &lepton_ctrl_ops,
	.id     = V4L2_CID_LEPTON_FFC,
	.name   = "FFC",
	.type   = V4L2_CTRL_TYPE_INTEGER,
	.min    = 0,
	.max    = 65535,
	.step   = 1,
};

static int lepton_init_ctrl(struct lepton_subdev *lepton)
{
	int err;

	lepton->sd.ctrl_handler = &lepton->ctrl_handler;
	err = v4l2_ctrl_handler_init(&lepton->ctrl_handler, 5);
	if (err)
	    return err;
	v4l2_ctrl_new_std(&lepton->ctrl_handler, &lepton_ctrl_ops,
	                  V4L2_CID_AUTOGAIN, 0, 1, 1, 0);
	v4l2_ctrl_new_custom(&lepton->ctrl_handler,
	                     &lepton_ffc_cfg, NULL);
	v4l2_ctrl_new_custom(&lepton->ctrl_handler,
	                     &lepton_agc_policy, NULL);
	lepton->tele_ctrl = v4l2_ctrl_new_custom(&lepton->ctrl_handler,
	                                         &lepton_telemetry_cfg, NULL);

	err = lepton->ctrl_handler.error;
	if (err)
		v4l2_ctrl_handler_free(&lepton->ctrl_handler);

	return err;
}

static void lepton_free_ctrl(struct lepton_subdev *lepton)
{
	v4l2_ctrl_handler_free(&lepton->ctrl_handler);
}

static int lepton_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct lepton_subdev *lepton = to_lepton(sd);

	param->parm.raw_data[0] = lepton->tele;

	return 0;
}

static const struct v4l2_subdev_core_ops lepton_core_ops = {
	.reset          = lepton_reset,
	.queryctrl      = v4l2_subdev_queryctrl,
	.querymenu      = v4l2_subdev_querymenu,
	.g_ctrl         = v4l2_subdev_g_ctrl,
	.s_ctrl         = v4l2_subdev_s_ctrl,
	.g_ext_ctrls    = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls  = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls    = v4l2_subdev_s_ext_ctrls,
};

static const struct v4l2_subdev_video_ops lepton_video_ops = {
	.s_stream       = lepton_s_stream,
	.enum_mbus_fmt  = lepton_enum_mbus_fmt,
	.try_mbus_fmt   = lepton_try_mbus_fmt,
	.g_mbus_fmt     = lepton_g_mbus_fmt,
	.s_mbus_fmt     = lepton_s_mbus_fmt,
	.g_parm         = lepton_g_parm,
};

static const struct v4l2_subdev_ops lepton_ops = {
	.core   = &lepton_core_ops,
	.video  = &lepton_video_ops,
};

static int lepton_init_gpio(struct lepton_subdev *lepton, int pwr, int rst)
{
	int err = 0;

	if (pwr != -1) {
		err = gpio_request_one(pwr, GPIOF_OUT_INIT_LOW,
				       "lepton-ctrl_pwr");
		if (err)
			goto err;
	}
	lepton->gpio_pwr = pwr;

	if (rst != -1) {
		err = gpio_request_one(rst, GPIOF_OUT_INIT_LOW,
				       "lepton-ctrl_rst");
		if (err)
			goto free_pwr_gpio;
	}
	lepton->gpio_rst = rst;

free_pwr_gpio:
	if (err && pwr)
		gpio_free(pwr);
err:
	return err;
}

static void lepton_free_gpio(struct lepton_subdev *lepton)
{
	if (lepton->gpio_pwr)
		gpio_free(lepton->gpio_pwr);

	if (lepton->gpio_rst)
		gpio_free(lepton->gpio_rst);
}

static int lepton_dev_probe(struct i2c_client *client,
                            const struct i2c_device_id *id)
{
	struct lepton_i2c_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lepton_subdev *lep_sub;
	struct v4l2_subdev *sd;
	struct v4l2_mbus_framefmt *fmt;
	int err = -EIO;

	if (!i2c_check_functionality(client->adapter,
	                              I2C_FUNC_SMBUS_WORD_DATA))
		goto err;

	err = -ENOMEM;
	lep_sub = kzalloc(sizeof(struct lepton_subdev), GFP_KERNEL);
	if (!lep_sub)
		goto err;

	lep_sub->client = client;

	sd = &lep_sub->sd;
	v4l2_i2c_subdev_init(sd, client, &lepton_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	fmt = &lep_sub->format;
	fmt->width = LEPTON_IMAGE_WIDTH;
	fmt->height = LEPTON_IMAGE_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;

	err = lepton_init_gpio(lep_sub, pdata ? pdata->gpio_pwr : -1,
	                       pdata ? pdata->gpio_rst : -1);

	if (err) {
		v4l2_err(sd, "failed to request gpio(s) (%d)\n", err);
		goto free_subdev;
	}

	lepton_power_on(lep_sub);
	lepton_hard_reset(sd, 0);

	err = lepton_init(lep_sub);
	if (err) {
		v4l2_err(sd, "init failed (%d)\n", err);
		goto free_gpio;
	}

	err = lepton_init_ctrl(lep_sub);
	if (err)
		goto free_gpio;

	return 0;

free_gpio:
	lepton_free_gpio(lep_sub);
free_subdev:
	kfree(lep_sub);
err:
	return err;
}

static int lepton_dev_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sub = i2c_get_clientdata(client);
	struct lepton_subdev *dev = to_lepton(sub);

	v4l2_device_unregister_subdev(sub);
	lepton_power_off(dev);
	lepton_free_ctrl(dev);
	lepton_free_gpio(dev);
	kfree(dev);
	return 0;
}

static const struct i2c_device_id lepton_dev_id[] = {
	{ "lepton-ctrl", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, lepton_dev_id);

static struct i2c_driver lepton_dev_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "lepton-ctrl",
	},
	.probe = lepton_dev_probe,
	.remove = lepton_dev_remove,
	.id_table = lepton_dev_id,
};

module_i2c_driver(lepton_dev_driver);

