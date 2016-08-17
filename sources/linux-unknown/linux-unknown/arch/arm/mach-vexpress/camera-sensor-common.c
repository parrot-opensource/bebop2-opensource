/**
 * linux/arch/arm/mach-parrot7/camera-sensor-common.c
 * - Parrot7 based camera sensors common interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Didier Leymarie <didier.leymarie.ext@parrot.com>
 * date:    16-May-2013
 *
 * This file is released under the GPL
 */
#include <linux/mm.h>
#include "camera-sensor-common.h"

/**
 * Camera sensors and video capture devices
 */

/**
 * APTINA MT9F002 1/2.3-Inch 14Mp CMOS
 * Digital Image Sensor
 */

#define MT9F002_CAMERA_WIDTH_ALLOC 4096U
#define MT9F002_CAMERA_HEIGHT_ALLOC 4096U
#define MT9F002_CAMERA_FRAME_SIZE PAGE_ALIGN(MT9F002_CAMERA_WIDTH_ALLOC*MT9F002_CAMERA_HEIGHT_ALLOC*MT9F002_CAMERA_PIXEL_SIZE)
#define MT9F002_CAMERA_AVI_RAM_SIZE (MT9F002_CAMERA_FRAME_SIZE*MT9F002_CAMERA_N_BUFFERS)

size_t mt9f002_camera_avi_ram_size=MT9F002_CAMERA_AVI_RAM_SIZE;

static int mt9f002_camera_set_capture(struct soc_camera_platform_info *info,
                              int enable)
{
	return 0; /* Input configured in userland */
}

struct soc_camera_platform_info mt9f002_camera_platform_info = {
	.format_name = "SGRBG8",/* Raw Bayer 8 GR/BG */
	.format_depth = 8,
	.format = {
		.code       = V4L2_MBUS_FMT_SGRBG8_1X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field      = V4L2_FIELD_NONE,
		.width      = MT9F002_CAMERA_WIDTH,
		.height     = MT9F002_CAMERA_HEIGHT,
	},
	.mbus_param = V4L2_MBUS_PCLK_SAMPLE_RISING| V4L2_MBUS_MASTER |
			  V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_HSYNC_ACTIVE_LOW |
			  V4L2_MBUS_DATA_ACTIVE_HIGH,
	/* this sensor supports only this bus type */
	.mbus_type = V4L2_MBUS_PARALLEL,
	.set_capture = mt9f002_camera_set_capture,
};

struct i2c_board_info mt9f002_camera_i2c_board_info = {
	I2C_BOARD_INFO(MT9F002_CAMERA_NAME, 0x10),
};


/**
 * APTINA MT9V117 1/6-Inch VGA System-On-A-Chip (SOC) CMOS
 * Digital Image Sensor
 */
#define MT9V117_CAMERA_FRAME_SIZE PAGE_ALIGN(MT9V117_CAMERA_WIDTH*MT9V117_CAMERA_HEIGHT*MT9V117_CAMERA_PIXEL_SIZE)
#define MT9V117_CAMERA_AVI_RAM_SIZE (MT9V117_CAMERA_FRAME_SIZE*MT9V117_CAMERA_N_BUFFERS)

size_t mt9v117_camera_avi_ram_size=MT9V117_CAMERA_AVI_RAM_SIZE;

static int mt9v117_camera_set_capture(struct soc_camera_platform_info *info,
                             int enable)
{
	return 0; /* Input configured in userland */
}

struct soc_camera_platform_info mt9v117_camera_platform_info = {
	.format_name = "UYVY",/* UYVY */
	.format_depth = 8,
	.format = {
		.code       = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field      = V4L2_FIELD_NONE,
		.width      = MT9V117_CAMERA_WIDTH,
		.height     = MT9V117_CAMERA_HEIGHT,
	},
	.mbus_param = V4L2_MBUS_PCLK_SAMPLE_RISING| V4L2_MBUS_MASTER |
				  V4L2_MBUS_VSYNC_ACTIVE_LOW | V4L2_MBUS_HSYNC_ACTIVE_LOW |
				  V4L2_MBUS_DATA_ACTIVE_HIGH,
	.mbus_type = V4L2_MBUS_PARALLEL,
	.set_capture = mt9v117_camera_set_capture,
};

struct i2c_board_info mt9v117_camera_i2c_board_info = {
	I2C_BOARD_INFO(MT9V117_CAMERA_NAME, 0x5d),
};

/**
 * APTINA AS0260 1/6-Inch 1080P High-Definition (HD)
 * System-On-A-Chip (SOC) Digital Image Sensor
 */
#define AS0260_CAMERA_FRAME_SIZE PAGE_ALIGN(AS0260_CAMERA_WIDTH*AS0260_CAMERA_HEIGHT*AS0260_CAMERA_PIXEL_SIZE)
#define AS0260_CAMERA_AVI_RAM_SIZE (AS0260_CAMERA_FRAME_SIZE*AS0260_CAMERA_N_BUFFERS)

size_t as0260_camera_avi_ram_size=AS0260_CAMERA_AVI_RAM_SIZE;

struct i2c_board_info as0260_camera_i2c_board_info = {
	I2C_BOARD_INFO(AS0260_CAMERA_NAME, 0x48),
};

/**
 * APTINA MT9M114 1/6-Inch 720p High-Definition (HD)
 * System-On-A-Chip (SOC) Digital Image Sensor
 */
#define MT9M114_CAMERA_FRAME_SIZE PAGE_ALIGN(MT9M114_CAMERA_WIDTH*MT9M114_CAMERA_HEIGHT*MT9M114_CAMERA_PIXEL_SIZE)
#define MT9M114_CAMERA_AVI_RAM_SIZE (MT9M114_CAMERA_FRAME_SIZE*MT9M114_CAMERA_N_BUFFERS)

size_t mt9m114_camera_avi_ram_size=MT9M114_CAMERA_AVI_RAM_SIZE;

struct i2c_board_info mt9m114_camera_i2c_board_info = {
	I2C_BOARD_INFO(MT9M114_CAMERA_NAME, 0x48),
};

/**
 * Omnivision OV9740 1/6-Inch 720p High-Definition (HD)
 * System-On-A-Chip (SOC) Digital Image Sensor
 */
#define OV9740_CAMERA_FRAME_SIZE PAGE_ALIGN(OV9740_CAMERA_WIDTH*OV9740_CAMERA_HEIGHT*OV9740_CAMERA_PIXEL_SIZE)
#define OV9740_CAMERA_AVI_RAM_SIZE (OV9740_CAMERA_FRAME_SIZE*OV9740_CAMERA_N_BUFFERS)

size_t ov6740_camera_avi_ram_size=OV9740_CAMERA_AVI_RAM_SIZE;

struct i2c_board_info ov6740_camera_i2c_board_info = {
	I2C_BOARD_INFO(OV9740_CAMERA_NAME, 0x10),
};

/**
 * ANALOG DEVICES Advantiv ADV7611 HDMI Receiver
 */
#define ADV7611_HDMI_RECEIVER_FRAME_SIZE PAGE_ALIGN(ADV7611_HDMI_RECEIVER_WIDTH*ADV7611_HDMI_RECEIVER_HEIGHT*ADV7611_HDMI_RECEIVER_PIXEL_SIZE)
#define ADV7611_HDMI_RECEIVER_AVI_RAM_SIZE PAGE_ALIGN(ADV7611_HDMI_RECEIVER_FRAME_SIZE*ADV7611_HDMI_RECEIVER_N_BUFFERS)

size_t adv7611_hdmi_receiver_avi_ram_size=ADV7611_HDMI_RECEIVER_AVI_RAM_SIZE;

static int adv7611_hdmi_receiver_set_capture(struct soc_camera_platform_info *info,
				    int enable)
{
	return 0; /* Input configured in userland */
}

struct soc_camera_platform_info adv7611_hdmi_receiver_platform_info = {
	.format_name = "UYVY",
	.format_depth = 8,
	.format = {
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field = V4L2_FIELD_NONE,
		.width = ADV7611_HDMI_RECEIVER_WIDTH,
		.height = ADV7611_HDMI_RECEIVER_HEIGHT,
	},
	.mbus_param = V4L2_MBUS_PCLK_SAMPLE_RISING | V4L2_MBUS_MASTER |
				  V4L2_MBUS_VSYNC_ACTIVE_HIGH | V4L2_MBUS_HSYNC_ACTIVE_HIGH |
				  V4L2_MBUS_DATA_ACTIVE_HIGH,
	.mbus_type = V4L2_MBUS_BT656,
	.set_capture = adv7611_hdmi_receiver_set_capture,
};

struct i2c_board_info adv7611_hdmi_receiver_i2c_board_info = {
	I2C_BOARD_INFO(ADV7611_HDMI_RECEIVER_NAME, 0x4C),
};

