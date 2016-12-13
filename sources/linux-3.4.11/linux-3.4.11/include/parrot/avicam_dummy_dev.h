
/*
 * Avicam dummy device driver
 *
 * Based(copied) on soc_camera_platform
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __AVICAM_DUMMY_DEV_H__
#define __AVICAM_DUMMY_DEV_H__

#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/video/avicam.h>
#include <media/v4l2-mediabus.h>

struct device;

struct avicam_dummy_info {
	struct v4l2_device		*v4l2_dev;
	int				 dev_id;
	struct v4l2_mbus_framefmt	 format;
	struct v4l2_subdev_frame_interval fi;
	struct media_pad		 pad;
};

static inline void avicam_platform_release(struct device *pdev)
{
	dev_info(pdev, "release device\n");
}

static inline int avicam_dummy_add(struct v4l2_device *v4l2_dev,
                                   struct platform_device **pdev,
                                   struct avicam_dummy_info *info)
{
	int ret;

	if (*pdev)
		return -EBUSY;

	*pdev = platform_device_alloc(AVICAM_DUMMY_NAME, info->dev_id);
	if (!*pdev)
		return -ENOMEM;

	info->v4l2_dev = v4l2_dev;

	(*pdev)->dev.platform_data = info;
	(*pdev)->dev.release = avicam_platform_release;

	ret = platform_device_add(*pdev);
	if (ret < 0) {
		platform_device_put(*pdev);
		*pdev = NULL;
	}

	return ret;
}

static inline void avicam_dummy_del(struct platform_device *pdev)
{
	if (!pdev)
		return;

	platform_device_unregister(pdev);
}

#endif /* __AVICAM_DUMMY_DEV_H__ */
