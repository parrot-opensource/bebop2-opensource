
/*
 * Dummy subdev driver for avicam
 *
 * Based(copied) on soc_camera_platform
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <parrot/avicam_dummy_dev.h>

struct avicam_dummy_priv {
	struct v4l2_subdev subdev;
};

static struct avicam_dummy_priv *get_priv(struct platform_device *pdev)
{
	struct v4l2_subdev *subdev = platform_get_drvdata(pdev);
	return container_of(subdev, struct avicam_dummy_priv, subdev);
}

static int avicam_dummy_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int avicam_dummy_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct avicam_dummy_info *p = v4l2_get_subdevdata(sd);

	if (!fi ||
		(p->fi.interval.numerator == 0) ||
		(p->fi.interval.denominator == 0))
		return -EINVAL;

	fi->interval.numerator = p->fi.interval.numerator;
	fi->interval.denominator = p->fi.interval.denominator;
	return 0;
}

static int avicam_dummy_s_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *fi)
{
	struct avicam_dummy_info *p = v4l2_get_subdevdata(sd);

	if (!fi)
		return -EINVAL;

	p->fi.interval.numerator = fi->interval.numerator;
	p->fi.interval.denominator = fi->interval.denominator;
	return 0;
}

static struct v4l2_mbus_framefmt *
__avicam_dummy_get_fmt(struct avicam_dummy_info *dummy, struct v4l2_subdev_fh *fh,
			  unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &dummy->format;
}

static int avicam_dummy_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct avicam_dummy_info *p = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __avicam_dummy_get_fmt(p, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int avicam_dummy_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct avicam_dummy_info *p = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __avicam_dummy_get_fmt(p, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	*format = fmt->format;
	return 0;
}

static int avicam_dummy_enum_mbus_code(struct v4l2_subdev *sd,
				       struct v4l2_subdev_fh *fh,
				       struct v4l2_subdev_mbus_code_enum *code)
{
	struct avicam_dummy_info *p = v4l2_get_subdevdata(sd);

	if (code->index)
		return -EINVAL;

	code->code = p->format.code;
	return 0;
}

static struct v4l2_subdev_core_ops avicam_dummy_subdev_core_ops;

static struct v4l2_subdev_video_ops avicam_dummy_subdev_video_ops = {
	.s_stream	= avicam_dummy_s_stream,
	.g_frame_interval = avicam_dummy_g_frame_interval,
	.s_frame_interval = avicam_dummy_s_frame_interval,
};

static struct v4l2_subdev_pad_ops avicam_dummy_subdev_pad_ops	= {
	.get_fmt = avicam_dummy_get_fmt,
	.set_fmt = avicam_dummy_set_fmt,
	.enum_mbus_code = avicam_dummy_enum_mbus_code,
};

static struct v4l2_subdev_ops avicam_dummy_subdev_ops = {
	.core	= &avicam_dummy_subdev_core_ops,
	.video	= &avicam_dummy_subdev_video_ops,
	.pad	= &avicam_dummy_subdev_pad_ops,
};

static int avicam_dummy_probe(struct platform_device *pdev)
{
	struct avicam_dummy_priv *priv;
	struct avicam_dummy_info *p = pdev->dev.platform_data;
	int ret;
	struct v4l2_subdev *sd;

	if (!p)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	sd = &priv->subdev;

	platform_set_drvdata(pdev, sd);

	v4l2_subdev_init(sd, &avicam_dummy_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	v4l2_set_subdevdata(sd, p);
	strlcpy(sd->name, dev_name(&pdev->dev),
		V4L2_SUBDEV_NAME_SIZE);

	ret = v4l2_device_register_subdev(p->v4l2_dev, sd);
	if (ret)
	{
		dev_err(&pdev->dev, "couldn't register subdev\n");
		goto evdrs;
	}

	p->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 1, &p->pad, 0);

	if(ret < 0)
	{
		dev_err(&pdev->dev, "couldn't init media entity\n");
		goto evdrs;
	}
	else
		dev_info(&pdev->dev, "entity intialized, dummy\n");

	return ret;

evdrs:
	platform_set_drvdata(pdev, NULL);
	kfree(priv);
	return ret;
}

static int avicam_dummy_remove(struct platform_device *pdev)
{
	struct avicam_dummy_priv *priv = get_priv(pdev);

	v4l2_device_unregister_subdev(&priv->subdev);
	media_entity_cleanup(&priv->subdev.entity);
	platform_set_drvdata(pdev, NULL);
	kfree(priv);
	return 0;
}

static struct platform_driver avicam_dummy_driver = {
	.driver 	= {
		.name	= AVICAM_DUMMY_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= avicam_dummy_probe,
	.remove		= avicam_dummy_remove,
};

module_platform_driver(avicam_dummy_driver);

MODULE_DESCRIPTION("avicam dummy dev");
MODULE_AUTHOR("Julien BERAUD");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:avicam_dummy");
