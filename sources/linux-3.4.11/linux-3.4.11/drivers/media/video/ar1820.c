/*
 * ar1820 - Aptina CMOS Digital Image Sensor
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Tue Aug 26 15:55:49 CEST 2014
 *
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>

#include <media/media-entity.h>
#include <media/ar1820.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "aptina-pll.h"

MODULE_AUTHOR("Eng-Hong SRON <eng-hong.sron@parrot.com>");
MODULE_DESCRIPTION("Aptina AR1820 driver");
MODULE_LICENSE("GPL");

#define DRIVER_NAME "ar1820"

#define AR1820_CHIP_VERSION			0x0000
#define 	AR1820_CHIP_VERSION_VALUE	0x2E01

struct ar1820 {
	struct v4l2_subdev           sd;
	struct media_pad             pad;
	struct v4l2_mbus_framefmt    format;
	struct ar1820_platform_data *pdata;
};

static inline struct ar1820 *to_ar1820(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ar1820, sd);
}

static int ar1820_read16(struct v4l2_subdev *sd, u16 reg, u16 *val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = (u8 *)&reg,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = (u8 *)val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	*val = swab16(*val);

	return 0;
}

static int ar1820_write8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	int                ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg     msg;

	struct {
		u16 reg;
		u8  val;
	} __packed buf;

	reg = swab16(reg);

	buf.reg = reg;
	buf.val = val;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int ar1820_write16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u16 buf[] = {
		swab16(reg),
		swab16(val),
	};

	struct i2c_msg msg = {
		.addr  = client->addr,
		.flags = 0,
		.len   = 4,
		.buf   = (u8 *)&buf,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int ar1820_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct ar1820            *ar1820 = to_ar1820(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, 0);
		fmt->format = *mf;
		return 0;
	}

	fmt->format = ar1820->format;

	return 0;
}

static int ar1820_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct ar1820            *ar1820 = to_ar1820(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	ar1820->format = *mf;

	return 0;
}

static int ar1820_enum_mbus_code(struct v4l2_subdev                *sd,
				  struct v4l2_subdev_fh             *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar1820 *ar1820 = to_ar1820(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = ar1820->format.code;

	return 0;
}

static int ar1820_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ar1820               *ar1820 = to_ar1820(sd);
	struct ar1820_platform_data *pdata = ar1820->pdata;
	int                          ret = 0;

	if (enable == 0) {
		if (pdata->set_power)
			pdata->set_power(0);

		return 0;
	}

	if (pdata->set_power) {
		ret = pdata->set_power(1);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	/* TODO: Need to clean this up.
	 * Sequence taken for dragon-prog... */
	ar1820_write8(sd, 0x0103, 0x01);
	msleep(1);

	ar1820_write16(sd, 0x301a, 0x0118);
	ar1820_write8(sd, 0x0100, 0x00);
	ar1820_write16(sd, 0x31ae, 0x0202);
	ar1820_write16(sd, 0x0112, 0x0a0a);
	ar1820_write16(sd, 0x0300, 0x0005);
	ar1820_write16(sd, 0x0302, 0x0001);
	ar1820_write16(sd, 0x0304, 0x0006);
	ar1820_write16(sd, 0x0306, 0x007d);
	ar1820_write16(sd, 0x0308, 0x000a);
	ar1820_write16(sd, 0x030a, 0x0002);
	ar1820_write16(sd, 0x3064, 0x0945);
	ar1820_write16(sd, 0x3016, 0x0111);
	ar1820_write16(sd, 0x3064, 0x0045);
	ar1820_write16(sd, 0x034c, 0x0500);
	ar1820_write16(sd, 0x034e, 0x02d0);
	ar1820_write16(sd, 0x0344, 0x0000);
	ar1820_write16(sd, 0x0348, 0x04ff);
	ar1820_write16(sd, 0x0346, 0x0000);
	ar1820_write16(sd, 0x034a, 0x02cf);
	msleep(1);

	ar1820_write16(sd, 0x0342, 0x3d20);
	ar1820_write16(sd, 0x0340, 0x0362);
	ar1820_write16(sd, 0x0202, 0x017f);
	ar1820_write16(sd, 0x3014, 0x1aa0);
	ar1820_write16(sd, 0x3056, 0x1440);
	ar1820_write16(sd, 0x3058, 0x1440);
	ar1820_write16(sd, 0x305a, 0x1440);
	ar1820_write16(sd, 0x305c, 0x1440);
	ar1820_write16(sd, 0x3018, 0x0000);
	ar1820_write16(sd, 0x301a, 0x0118);
	ar1820_write16(sd, 0x3edc, 0x68cf);
	ar1820_write16(sd, 0x3ee2, 0xe363);
	msleep(1);

	ar1820_write8(sd, 0x0104, 0x01);
	ar1820_write16(sd, 0x0342, 0x3d20);
	ar1820_write16(sd, 0x0340, 0x0362);
	ar1820_write16(sd, 0x0202, 0x017f);
	ar1820_write16(sd, 0x3014, 0x1aa0);
	ar1820_write8(sd, 0x0104, 0x00);
	ar1820_write16(sd, 0x0342, 0x3d20);
	ar1820_write16(sd, 0x0340, 0x0362);
	ar1820_write8(sd, 0x0100, 0x01);
	msleep(1);

	return 0;
}

static int ar1820_g_dv_timings(struct v4l2_subdev     *sd,
			       struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings *bt = &timings->bt;

	bt->pixelclock = 40000000;
	return 0;
}

static const struct v4l2_subdev_video_ops ar1820_video_ops = {
	.s_stream     = ar1820_s_stream,
	.g_dv_timings = ar1820_g_dv_timings,
};

static const struct v4l2_subdev_pad_ops ar1820_pad_ops = {
	.get_fmt        = ar1820_get_fmt,
	.set_fmt        = ar1820_set_fmt,
	.enum_mbus_code = ar1820_enum_mbus_code,
};

static const struct v4l2_subdev_ops ar1820_ops = {
	.core  = NULL,
	.video = &ar1820_video_ops,
	.pad   = &ar1820_pad_ops,
};

static int ar1820_detect_chip(struct v4l2_subdev *sd)
{
	struct ar1820               *ar1820 = to_ar1820(sd);
	struct ar1820_platform_data *pdata = ar1820->pdata;
	int                           ret = 0;
	u16                           id = 0;

	if (pdata->set_power) {
		ret = pdata->set_power(1);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	ar1820_read16(sd, AR1820_CHIP_VERSION, &id);

	if (pdata->set_power)
		pdata->set_power(0);

	if (id != AR1820_CHIP_VERSION_VALUE) {
		v4l2_err(sd, "Error Chip ID = 0x%04x instead of 0x%04x\n",
				id, AR1820_CHIP_VERSION_VALUE);
		return -ENODEV;
	}

	v4l2_info(sd, "Found " DRIVER_NAME " chip\n");

	return 0;
}

static int ar1820_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ar1820               *ar1820;
	struct ar1820_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev           *sd;
	int                           ret = 0;

	if (pdata == NULL) {
		dev_err(&client->dev, "platform data not specified\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c not available\n");
		return -ENODEV;
	}

	ar1820 = kzalloc(sizeof(*ar1820), GFP_KERNEL);
	if (!ar1820) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	ar1820->pdata = pdata;

	sd = &ar1820->sd;
	v4l2_i2c_subdev_init(sd, client, &ar1820_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ar1820->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ar1820->pad, 0);

	if (ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto error_media_entity_init;
	}

	/* Set default configuration: 720p60 */
	ar1820->format.width  = 1280;
	ar1820->format.height =  720;
	ar1820->format.code   = V4L2_MBUS_FMT_SBGGR10_1X10;

	/* Check if the chip is present */
	ret = ar1820_detect_chip(sd);
	if (ret < 0)
		goto error_detect;

	return 0;

error_detect:
error_media_entity_init:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(ar1820);

	return ret;
}

static int ar1820_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar1820     *ar1820 = to_ar1820(sd);

	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(ar1820);

	return 0;
}

static const struct i2c_device_id ar1820_id[] = {
	{DRIVER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, ar1820_id);

static struct i2c_driver ar1820_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe = ar1820_probe,
	.remove = ar1820_remove,
	.id_table = ar1820_id,
};

module_i2c_driver(ar1820_driver);
