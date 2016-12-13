/*
 * A V4L2 driver for OV16825 cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>

#include "ov16825_common.h"
#include "ov16825_confs.h"
#include "ov16825_ctrl.h"
#include "ov16825_clk.h"

MODULE_AUTHOR("Victor Lambret");
MODULE_DESCRIPTION("A low-level driver for OV16825 sensors");
MODULE_LICENSE("GPL");

static void ov16825_configure(struct v4l2_subdev *sd)
{
	struct ov16825_info             *info = to_state(sd);

	sensor_write(sd, SC_CTRL0103, SC_CTRL0103_SOFTWARE_RESET);

	/* This 20 ms delay comes from the official scripts */
	msleep(20);

	sensor_write_array(sd, info->config->regs, info->config->regs_len);

	if (info->format.code == V4L2_MBUS_FMT_SBGGR12_1X12) {
		/* Change PLL config to increase MIPI clock frequency */
		sensor_write(&info->sd, 0x0302, 0x8b);
		/* Set 12 bit mode */
		sensor_write(&info->sd, 0x3031, 0x0c);
		/* Change mipi clock period */
		sensor_write(&info->sd, 0x4837, 0x35);
	}

	ov16825_apply_ctrl(info);
}

static int ov16825_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov16825_info             *info  = to_state(sd);
	struct ov16825_platform_data    *pdata = info->pdata;

	if (enable) {
		if (pdata->set_power)
			pdata->set_power(1);
		info->streaming = 1;
		ov16825_configure(sd);
	} else {
		if (pdata->set_power)
			pdata->set_power(0);
		info->streaming = 0;
	}

	return 0;
}

static int ov16825_g_dv_timings(struct v4l2_subdev *sd,
                                struct v4l2_dv_timings *timings)
{
	struct ov16825_info          *info = to_state(sd);
	struct v4l2_mbus_framefmt   *fmt = &info->format;
	struct v4l2_bt_timings      *bt = &timings->bt;

	memset(timings, 0, sizeof(*timings));

	bt->width      = fmt->width;
	bt->height     = fmt->height;
	//bt->pixelclock = mt9m021->pll.pix_clock;
	bt->pixelclock = 100000000;

	return 0;
}

static int sensor_get_fmt(struct v4l2_subdev *sd,
                          struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov16825_info          *info = to_state(sd);
	struct v4l2_mbus_framefmt   *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	fmt->format = info->format;
	return 0;
}

static int sensor_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov16825_info		*info = to_state(sd);
	struct v4l2_mbus_framefmt	*f;
	struct ov16825_config		*config = NULL;
	u32				 requested_pixels;
	u32                              cur_delta = UINT_MAX;
	unsigned                         i;

	f = &fmt->format;
	requested_pixels = f->width * f->height;

	if (f->code != V4L2_MBUS_FMT_SBGGR12_1X12 &&
	    f->code != V4L2_MBUS_FMT_SBGGR10_1X10) {
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(ov16825_configs); i++) {
		struct ov16825_config	*c = &ov16825_configs[i];
		u32			 delta;

		if (c->width == f->width && c->height == f->height) {
			config = c;
			break;
		}


		// Don't consider smaller resolutions (we can always crop/scale higher
		// resolutions later)
		if (c->width < f->width || c->height < f->height) {
			continue;
		}

		// Attempt to find the closest higher resolution by number of
		// pixels
		delta = (c->width * c->height) - requested_pixels;

		if (delta < cur_delta) {
			// We found a closer resolution
			config = c;
			cur_delta = delta;
		}
	}

	if (config == NULL) {
		// No config found, use the first one as default
		config = &ov16825_configs[0];
	}

	f->width = config->width;
	f->height = config->height;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt   *mf;

		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		*mf = *f;
		return 0;
	}

	info->format = *f;
	info->config = config;

	return 0;
}

static int sensor_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov16825_info          *info = to_state(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = info->format.code;

	return 0;
}

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.s_stream         = ov16825_s_stream,
	.g_dv_timings     = ov16825_g_dv_timings,
};

static const struct v4l2_subdev_pad_ops sensor_pad_ops = {
	.get_fmt = sensor_get_fmt,
	.set_fmt = sensor_set_fmt,
	.enum_mbus_code = sensor_enum_mbus_code,
};

static const struct v4l2_subdev_ops ov16825_ops = {
	.video = &sensor_video_ops,
	.pad = &sensor_pad_ops,
};

static int ov16825_detect_chip(struct v4l2_subdev *sd)
{
	struct ov16825_info		*info  = to_state(sd);
	struct ov16825_platform_data	*pdata = info->pdata;
	u8                               id1, id2, id3;
	int				 ret;

	if (pdata->set_power) {
		ret = pdata->set_power(1);

		if (ret) {
			v4l2_err(sd, "Couldn't set camera power on\n");
			return ret;
		}
	}

	ret = sensor_read(sd, SC_CMMN_CHIP_ID1, &id1);
	if (ret) {
		goto power_off;
	}

	ret = sensor_read(sd, SC_CMMN_CHIP_ID2, &id2);
	if (ret) {
		goto power_off;
	}

	ret = sensor_read(sd, SC_CMMN_CHIP_ID3, &id3);
	if (ret) {
		goto power_off;
	}

	if (id1 != 0x01 || id2 != 0x68 || id3 != 0x20) {
		v4l2_err(sd,
			 "Invalid camera ID: expected 016820, got %02x%02x%02x\n",
			 id1, id2, id3);
		ret = -EINVAL;
		goto power_off;
	}

	v4l2_info(sd, "Found OV16825 chip\n");

	ret = 0;

power_off:
	if (pdata->set_power) {
		pdata->set_power(0);
	}

	return ret;
}

static int ov16825_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	struct v4l2_subdev              *sd;
	struct ov16825_info              *info;
	struct ov16825_platform_data    *pdata = client->dev.platform_data;
	int                              ret;

	if (pdata == NULL) {
		dev_err(&client->dev, "platform data not specified\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c adapdter does not support SMBUS\n");
		return -EINVAL;
	}

	info = kzalloc(sizeof(struct ov16825_info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	info->pdata = pdata;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &ov16825_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	info->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &info->pad, 0);
	if (ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto error_media_entity_init;
	}

	/* Default configuration */
	info->config = &ov16825_configs[0];
	info->format.width  = info->config->width;
	info->format.height = info->config->height;
	info->format.code   = V4L2_MBUS_FMT_SBGGR10_1X10;

	ret = ov16825_ctrl_create(info);
	if (ret)
		goto error_ctrls;

	ret = ov16825_detect_chip(sd);
	if (ret) {
		goto chip_not_found;
	}

	v4l2_info(sd, "OV16825 sensor succesfully probed");

	return 0;

chip_not_found:
	ov16825_ctrl_free(info);
error_ctrls:
error_media_entity_init:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(info);

	return ret;
}


static int ov16825_remove(struct i2c_client *client)
{
	struct v4l2_subdev      *sd = i2c_get_clientdata(client);

	ov16825_ctrl_free(to_state(sd));

	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));

	return 0;
}

static const struct i2c_device_id ov16825_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe = ov16825_probe,
	.remove = ov16825_remove,
	.id_table = ov16825_id,
};

static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
	i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
