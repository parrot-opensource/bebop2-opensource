/**
 * linux/arch/arm/mach-parrot7/camera-sensor-common.h
 * - Parrot7 based camera sensors common interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Didier Leymarie <didier.leymarie.ext@parrot.com>
 * date:    16-May-2013
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_CAMERA_SENSOR_COMMON_H
#define _ARCH_CAMERA_SENSOR_COMMON_H

/**
 * Camera sensors and video capture devices
 */

#include <linux/platform_device.h>
#include <media/video/avicam.h>
#include <media/soc_camera_platform.h>
#include <media/soc_camera.h>

/**
 * APTINA MT9F002 1/2.3-Inch 14Mp CMOS
 * Digital Image Sensor
 */

/* Resolutions */
/* maximum */
#define MT9F002_CAMERA_WIDTH_MAX 4640U
#define MT9F002_CAMERA_HEIGHT_MAX 3320U
/* entire array */
#define MT9F002_CAMERA_WIDTH_WHOLE 4608U
#define MT9F002_CAMERA_HEIGHT_WHOLE 3288U
/* 4:3, still mode */
#define MT9F002_CAMERA_WIDTH_STILL 4384U
#define MT9F002_CAMERA_HEIGHT_STILL 3288U
/* 16:9 video mode */
#define MT9F002_CAMERA_WIDTH_VIDEO 4608U
#define MT9F002_CAMERA_HEIGHT_VIDEO 2592U
/* 1080p mode  (1080p +17%EIS) */
#define MT9F002_CAMERA_WIDTH_1080P 2256U
#define MT9F002_CAMERA_HEIGHT_1080P 1268U

#define MT9F002_CAMERA_WIDTH MT9F002_CAMERA_WIDTH_STILL
#define MT9F002_CAMERA_HEIGHT MT9F002_CAMERA_HEIGHT_STILL

/* pixel in Bayer 8 bits */
#define MT9F002_CAMERA_PIXEL_SIZE 1
/* number of v4L2 buffers */
#define MT9F002_CAMERA_N_BUFFERS 4

#define MT9F002_CAMERA_NAME "mt9f002"

extern
size_t mt9f002_camera_avi_ram_size;

extern
struct soc_camera_platform_info mt9f002_camera_platform_info;

extern
struct i2c_board_info mt9f002_camera_i2c_board_info;

/**
 * APTINA MT9V117 1/6-Inch VGA System-On-A-Chip (SOC) CMOS
 * Digital Image Sensor
 */

/* Resolution VGA */
#define MT9V117_CAMERA_WIDTH 640U
#define MT9V117_CAMERA_HEIGHT 480U

/* pixel in YUV 4:2:2 16 bits */
#define MT9V117_CAMERA_PIXEL_SIZE 2
/* number of v4L2 buffers */
#define MT9V117_CAMERA_N_BUFFERS 4

#define MT9V117_CAMERA_NAME "mt9v117"

extern
size_t mt9v117_camera_avi_ram_size;

extern
struct soc_camera_platform_info mt9v117_camera_platform_info;

extern
struct i2c_board_info mt9v117_camera_i2c_board_info;

/**
 * APTINA AS0260 1/6-Inch 1080P High-Definition (HD)
 * System-On-A-Chip (SOC) Digital Image Sensor
 */
#include <media/video/as0260.h>

/* Resolution Full HD */
#define AS0260_CAMERA_WIDTH 1920U
#define AS0260_CAMERA_HEIGHT 1080U

/* pixel in YUV 4:2:2 16 bits */
#define AS0260_CAMERA_PIXEL_SIZE 2

/* number of v4L2 buffers */
#define AS0260_CAMERA_N_BUFFERS 4

#define AS0260_CAMERA_NAME "as0260"

extern
size_t as0260_camera_avi_ram_size;

extern
struct i2c_board_info as0260_camera_i2c_board_info;

/**
 * APTINA MT9M114 1/6-Inch 720p High-Definition (HD)
 * System-On-A-Chip (SOC) Digital Image Sensor
 */
#include <media/video/mt9m114.h>

/* Resolution HD */
#define MT9M114_CAMERA_WIDTH 1280U
#define MT9M114_CAMERA_HEIGHT 720U

/* pixel in YUV 4:2:2 16 bits */
#define MT9M114_CAMERA_PIXEL_SIZE 2

/* number of v4L2 buffers */
#define MT9M114_CAMERA_N_BUFFERS 4

#define MT9M114_CAMERA_NAME "mt9m114"

extern
size_t mt9m114_camera_avi_ram_size;

extern
struct i2c_board_info mt9m114_camera_i2c_board_info;

/**
 * Omnivision OV9740 1/6-Inch 720p High-Definition (HD)
 * System-On-A-Chip (SOC) Digital Image Sensor
 */

/* Resolution HD */
#define OV9740_CAMERA_WIDTH 1280U
#define OV9740_CAMERA_HEIGHT 720U

/* pixel in YUV 4:2:2 16 bits */
#define OV9740_CAMERA_PIXEL_SIZE 2

/* number of v4L2 buffers */
#define OV9740_CAMERA_N_BUFFERS 4

#define OV9740_CAMERA_NAME "ov9740"

extern
size_t ov9740_camera_avi_ram_size;

extern
struct i2c_board_info ov9740_camera_i2c_board_info;

/**
 * ANALOG DEVICES Advantiv ADV7611 HDMI Receiver
 */

/* Resolution (Maximum Full HD) */
#define ADV7611_HDMI_RECEIVER_WIDTH 1920
#define ADV7611_HDMI_RECEIVER_HEIGHT 1080

/* pixel in YUV 4:2:2 16 bits */
#define ADV7611_HDMI_RECEIVER_PIXEL_SIZE 2

/* number of v4L2 buffers */
#define ADV7611_HDMI_RECEIVER_N_BUFFERS 4

#define ADV7611_HDMI_RECEIVER_NAME "adv7611"

extern
size_t adv7611_hdmi_receiver_avi_ram_size;

extern
struct soc_camera_platform_info adv7611_hdmi_receiver_platform_info;

extern
struct i2c_board_info adv7611_hdmi_receiver_i2c_board_info;

/*
 * Camera sensor types
 */
#define SOC_CAM_TYPE_NONE 	0x00000000
#define SOC_CAM_TYPE_MT9F002 0x4D549002
#define SOC_CAM_TYPE_MT9M114 0x4D549114
#define SOC_CAM_TYPE_MT9V117 0x4D549117
#define SOC_CAM_TYPE_AS0260  0x41530260

/*
 * Synchronization types
 */
#define CAM_SYNCHRO_ITU_BT656 0x42363536
#define CAM_SYNCHRO_ITU_BT601 0x42363031

/*
 * some useful macros
 */
#define __str(s) #s
#define AVI_CONV_NODE(_x) (AVI_CONV0_NODE+_x)
/*
 * definition of structures required for avicam driver
 */
#define AVICAM_PLATFORM_DEVICE(_name, camif, camirq, fifoirq,		\
			       _bustype, _padselect, _overlay)		\
	static struct avi_chan _name ## _avi_chan = {			\
		.type       = AVI_CAPTURE_CHAN_TYPE,			\
		.nodes      = _name ## _avi_nodes,			\
		.nodes_nr   = ARRAY_SIZE(_name ## _avi_nodes),		\
	};								\
	static struct avi_group _name ## _avi_group = {			\
		.name       = "avicam.cam" __str(camif),		\
		.lck        = 						\
			__SPIN_LOCK_UNLOCKED(_name ## _avi_group.lck),	\
		.irq        = fifoirq,					\
		.channels   = &_name ## _avi_chan,			\
		.chan_nr    = 1,					\
	};								\
	static struct avicam_platform_data _name ## _pdata = {		\
		.avicam      = NULL,					\
		.avi_group   = &_name ## _avi_group,			\
		.mbus_type   = _bustype,				\
		.pad_select  = _padselect,				\
		.overlay_chan = _overlay,				\
	};								\
	static struct resource _name ## _res[] = { 			\
		{							\
			.flags = IORESOURCE_IRQ, 			\
			.start = camirq,				\
			.end   = camirq,				\
		},							\
		{							\
			.flags = IORESOURCE_MEM,			\
		},							\
	};								\
	static u64 _name ## _dma_mask = DMA_BIT_MASK(32);		\
	static struct platform_device _name ## _dev = {			\
		.name           = "avicam",				\
		.id             = camif,				\
		.resource       = _name ## _res,			\
		.num_resources  = ARRAY_SIZE(_name ## _res),		\
		.dev            = {					\
			.platform_data      = &_name ## _pdata,		\
		/* Todo: try to set a narrower mask */			\
			.dma_mask           = &_name ## _dma_mask,	\
			.coherent_dma_mask  = DMA_BIT_MASK(32)		\
		}							\
	};								\
	/* Register devboard sensors */					\
	static struct platform_device _name ## _soc_pdev = {		\
		.name = "soc-camera-pdrv",				\
		.id   = camif,						\
		.dev            = {					\
			.platform_data      = &_name ## _iclink,	\
		}							\
	}

/*
 * definition of structures required for a SoC-camera driver
 */
#define SOC_CAMERA_LINK(_name, camif, cam_i2c_dev,			\
			cam_i2c_bus, querybus, module, pdata)		\
	static struct soc_camera_link _name ## _iclink =		\
	{								\
		.bus_id		 = camif, /* id of the camif */		\
		.board_info	 = &cam_i2c_dev,			\
		.i2c_adapter_id	 = cam_i2c_bus,				\
		.query_bus_param = querybus,				\
		.module_name	 = module,				\
		.priv            = &pdata				\
	}

/*
 * definition of structures required for soc-platform-camera driver
 */
#define SOC_CAMERA_PLATFORM_LINK(_name, camif, cam_i2c_dev,		\
				 cam_i2c_bus, querybus, pdata)		\
	static int							\
		_name ## _add_soc_cam(struct soc_camera_device *);	\
	static void							\
		_name ## _del_soc_cam(struct soc_camera_device *);	\
	static struct platform_device *_name ## _device = NULL;		\
	struct soc_camera_link _name ## _iclink = {			\
		.bus_id          = camif, /* id of the camif */		\
	.module_name     = "soc_camera_platform", \
		.add_device      = &_name ## _add_soc_cam,		\
		.del_device      = &_name ## _del_soc_cam,		\
	.priv            = &pdata, \
	/*.board_info	    = &cam_i2c_dev ,*/ \
	.query_bus_param = querybus, \
	.i2c_adapter_id	 = cam_i2c_bus \
}; \
	static void _name ## _soc_cam_release(struct device *dev)	\
{ \
		soc_camera_platform_release(&_name ## _device);		\
} \
	static int _name ## _add_soc_cam(struct soc_camera_device *d)	\
{ \
	return soc_camera_platform_add(d, \
					&_name ## _device,		\
					&_name ## _iclink,		\
					_name ## _soc_cam_release,	\
									camif /* camif */); \
} \
	static void _name ## _del_soc_cam(struct soc_camera_device *d)	\
{ \
		soc_camera_platform_del(d, 				\
				_name ## _device, &_name ## _iclink);	\
		_name ## _device = NULL;				\
}

#define SOC_CAMERA_PLATFORM_LINK_MT9F002(_name ,camif,		\
					 cam_i2c_bus, querybus)	\
	SOC_CAMERA_PLATFORM_LINK(_name,camif,			\
				 mt9f002_camera_i2c_board_info,	\
				 cam_i2c_bus,querybus,		\
				 mt9f002_camera_platform_info)

#define SOC_CAMERA_PLATFORM_LINK_MT9V117(_name, camif,		\
					 cam_i2c_bus, querybus) \
	SOC_CAMERA_PLATFORM_LINK(_name,camif,			\
				 mt9v117_camera_i2c_board_info,	\
				 cam_i2c_bus,querybus,		\
				 mt9v117_camera_platform_info)

#define SOC_CAMERA_PLATFORM_LINK_ADV7611(_name, camif,			\
					 cam_i2c_bus, querybus)		\
	SOC_CAMERA_PLATFORM_LINK(_name,camif,				\
				 adv7611_hdmi_receiver_i2c_board_info,	\
				 cam_i2c_bus,querybus,			\
				 adv7611_hdmi_receiver_platform_info)

#define SOC_CAMERA_LINK_MT9M114(_name, camif,				\
				cam_i2c_bus, querybus, mbustype)	\
	static struct mt9m114_platform_data				\
		_name ## _mt9m114_camera_platform_data = {		\
	.mbus_type = mbustype, \
}; \
	SOC_CAMERA_LINK(_name,camif,mt9m114_camera_i2c_board_info,	\
			cam_i2c_bus,querybus,MT9M114_CAMERA_NAME,	\
			_name ## _mt9m114_camera_platform_data)

#define SOC_CAMERA_LINK_AS0260(_name, camif, cam_i2c_bus, 	\
				querybus, mbustype)		\
	static struct as0260_platform_data			\
		_name ## _as0260_camera_platform_data = {	\
	.mbus_type = mbustype, \
}; \
	SOC_CAMERA_LINK(_name,camif,				\
			as0260_camera_i2c_board_info,		\
			cam_i2c_bus,querybus,			\
			AS0260_CAMERA_NAME,			\
			_name ## _as0260_camera_platform_data)

#define SOC_CAMERA_LINK_OV9740(Name,camif,cam_i2c_bus,querybus,mbustype) \
SOC_CAMERA_LINK(Name,camif,ov9740_camera_i2c_board_info,cam_i2c_bus,querybus,OV9740_CAMERA_NAME,NULL)

#endif /* _ARCH_CAMERA_SENSOR_COMMON_H */

