#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define DRIVER_NAME "galileo2"

struct galileo2 {
	struct v4l2_subdev             sd;
	struct media_pad               pad;
};

static inline struct galileo2 *sd_to_galileo2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct galileo2, sd);
}

static int galileo2_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	fmt->format.width  = 1280;
	fmt->format.height = 720;
	fmt->format.code   = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int galileo2_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	/* For now we support only one code */
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGBRG10_1X10;

	return 0;
}

static const struct v4l2_subdev_pad_ops galileo2_pad_ops = {
	.get_fmt        = galileo2_get_fmt,
	.set_fmt        = galileo2_get_fmt,
	.enum_mbus_code = galileo2_enum_mbus_code,
	/*.get_selection  = galileo2_get_selection,
	.set_selection  = galileo2_set_selection,*/
};

static int galileo2_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int galileo2_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct v4l2_bt_timings        *bt = &timings->bt;

	bt->width      = 1280;
	bt->height     = 720;
	bt->pixelclock = 100 * 1000 * 1000;

	/* Consider HSYNC and VSYNC as HACTIVE and VACTIVE*/
	bt->polarities = 0;

	/* Because we are in HACTIVE/VACTIVE mode, the blanking size does not
	 * matter for the capture device.
	 */

	return 0;
}

static const struct v4l2_subdev_video_ops galileo2_video_ops = {
	.s_stream         = galileo2_s_stream,
	/*.g_frame_interval = galileo2_g_frame_interval,
	  .s_frame_interval = galileo2_s_frame_interval,*/
	.g_dv_timings     = galileo2_g_dv_timings,
};

static const struct v4l2_subdev_ops galileo2_ops = {
/*	.core  = &galileo2_core_ops,*/
	.video = &galileo2_video_ops,
	.pad   = &galileo2_pad_ops,
};

static int galileo2_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct galileo2               *galileo2;
	/*struct galileo2_platform_data *pdata = client->dev.platform_data;*/
	struct v4l2_subdev            *sd;
	int                            ret = 0;

	/*if (pdata == NULL) {
		dev_err(&client->dev, "platform data not specified\n");
		return -EINVAL;
	}*/

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c not available\n");
		return -ENODEV;
	}

	galileo2 = kzalloc(sizeof(*galileo2), GFP_KERNEL);
	if (!galileo2) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	/*galileo2->pdata = pdata;*/

	sd = &galileo2->sd;
	v4l2_i2c_subdev_init(sd, client, &galileo2_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	galileo2->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &galileo2->pad, 0);

	if (ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto emedia;
	}


	return 0;

emedia:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(galileo2);

	return ret;
}

static int galileo2_remove(struct i2c_client *client)
{
	struct v4l2_subdev            *sd = i2c_get_clientdata(client);
	struct galileo2               *galileo2 = sd_to_galileo2(sd);
	/*struct galileo2_platform_data *pdata = client->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	if (pdata->set_power)
		pdata->set_power(GALILEO2_POWER_OFF);

	if (galileo2->i2c_pmic)
		i2c_unregister_device(galileo2->i2c_pmic);

	device_remove_file(&client->dev, &dev_attr_nvm);


	galileo2_free_controls(sd);
	*/
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(galileo2);

	return 0;
}

static const struct i2c_device_id galileo2_id[] = {
	{DRIVER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, galileo2_id);

static struct i2c_driver galileo2_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe    = galileo2_probe,
	.remove   = galileo2_remove,
	.id_table = galileo2_id,
};

module_i2c_driver(galileo2_driver);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Nokia Galileo2 driver");
MODULE_LICENSE("GPL");
