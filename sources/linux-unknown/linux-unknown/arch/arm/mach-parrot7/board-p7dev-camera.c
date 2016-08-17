/**
 * linux/arch/arm/mach-parrot7/camera-board.c - P7Dev camera daughter board
 *                                              implementation
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Didier Leymarie <didier.leymarie.ext@parrot.com>
 * date:    31-Jan-2013
 *
 * This file is released under the GPL
 */

#include <video/avi.h>
#include <video/avifb.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "board.h"
#include "board-common.h"
#include "pinctrl.h"
#include "avi.h"
#include "spi.h"
#include "gpu.h"
#include "venc.h"
#include <media/video/avicam.h>
#include <media/mt9f002.h>
#include <media/ar0330.h>
#include <parrot/avicam_dummy_dev.h>

#define CAMERA_BRD_NAME   "camera"

/******
 * PWM
 ******/

#include "p7_pwm.h"

static struct p7pwm_conf p7dev_conf_clock_camera = {
	.period_precision = 5,          /* Precision for mt9f002 */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_pdata p7dev_pwm_pdata = {
	.conf = {
		[9] = &p7dev_conf_clock_camera,/* CAM4 */
		[10] = &p7dev_conf_clock_camera,/* CAM5 */
	}
};

static struct pinctrl_map p7dev_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_09),
	P7_INIT_PINMAP(P7_PWM_10),
};

static struct pinctrl_map cam4_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_4_CLK),
	P7_INIT_PINMAP(P7_CAM_4_HS),
	P7_INIT_PINMAP(P7_CAM_4_VS),
	P7_INIT_PINMAP(P7_CAM_4_DATA00),
	P7_INIT_PINMAP(P7_CAM_4_DATA01),
	P7_INIT_PINMAP(P7_CAM_4_DATA02),
	P7_INIT_PINMAP(P7_CAM_4_DATA03),
	P7_INIT_PINMAP(P7_CAM_4_DATA04),
	P7_INIT_PINMAP(P7_CAM_4_DATA05),
	P7_INIT_PINMAP(P7_CAM_4_DATA06),
	P7_INIT_PINMAP(P7_CAM_4_DATA07),
	P7_INIT_PINMAP(P7_CAM_4_DATA08),
	P7_INIT_PINMAP(P7_CAM_4_DATA09),
};

static struct avicam_dummy_info cam4_dummy_driver_info = {
	.dev_id = 4,
	.format = {
		.code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.field = V4L2_FIELD_NONE,
		.width = 4640,
		.height = 3320,
	},
};

/* Aptina AR0330 Subdev */
#define CAM4_PWM_PERIOD_NS		38 /* 27 MHz approx */
#define CAM4_PWM			9
static struct pwm_device *cam4_pwm_device = NULL;
static int cam4_power_on(void)
{
	int ret = 0;

	if (cam4_pwm_device == NULL) {
		cam4_pwm_device = pwm_request(CAM4_PWM, "cam4-pwm");
		if (IS_ERR(cam4_pwm_device)) {
			ret = PTR_ERR(cam4_pwm_device);
			goto err_alloc;
		}
	}

	ret = pwm_config(cam4_pwm_device,
			 0,
			 CAM4_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	ret = pwm_enable(cam4_pwm_device);
	if (ret)
		goto err_config;

	return 0;

err_config:
	pwm_free(cam4_pwm_device);
	cam4_pwm_device = NULL;
err_alloc:
	pr_warning("failed to set clock for mt9f002 chip\n");
	return ret;
}

static int cam4_power_off(void)
{
	pwm_disable(cam4_pwm_device);
	return 0;
}

static int cam4_set_power(int on)
{
	if (on == AR0330_POWER_ON)
		return cam4_power_on();
	else
		return cam4_power_off();
}
static struct ar0330_platform_data cam4_ar0330_platform_data = {
	.clk_rate = 26000000,
	.max_vco_rate = 588000000,
	.max_op_clk_rate = 98000000,
	.hw_bus = AR0330_HW_BUS_PARALLEL,
	.set_power = &cam4_set_power,
};

static struct i2c_board_info cam4_ar0330_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ar0330", 0x10),
		.platform_data = &cam4_ar0330_platform_data,
	}
};

static struct mt9f002_platform_data cam4_mt9f002_platform_data = {
	.interface = MT9F002_Parallel,
	.pixel_depth = MT9F002_10bits,
	.number_of_lanes = MT9F002_2lane,
	.ext_clk_freq_mhz = 26315000000,
	.output_clk_freq_mhz = 96000000000,
	.set_power = &cam4_set_power,
};

static struct i2c_board_info cam4_mt9f002_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9f002", 0x10),
		.platform_data = &cam4_mt9f002_platform_data,
	}
};

#define CAM4_I2C_BUS	1
static struct avicam_subdevs cam4_ar0330_subdevs[] = {
	{
		.board_info = &cam4_ar0330_i2c_devices[0],
		.i2c_adapter_id = CAM4_I2C_BUS,
	},
	{
		.board_info = &cam4_mt9f002_i2c_devices[0],
		.i2c_adapter_id = CAM4_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_platform_data cam4_ar0330_pdata = {
	.cam_cap	   = AVI_CAP_CAM_4,
	.enable_stats      = 1,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 1,
		.ihs        = 1,
		.ipc        = 1,
		.psync_en   = 1,
		.ror_lsb    = 1,
	},
	.bus_width	   = 16,
	.subdevs	   = cam4_ar0330_subdevs,
	.dummy_driver_info = &cam4_dummy_driver_info,
};

#define CAM4_AVI_RAM_SIZE ((4640 * 3320 * 2 + 2048) * 3)

static u64 cam4_dma_mask = DMA_BIT_MASK(32);

static struct platform_device cam4_dev = {
	.name           = "avicam",
	.id             = 4,
	.dev            = {
		.dma_mask           = &cam4_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct pinctrl_map cam5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINMAP(P7_CAM_5_HS),
	P7_INIT_PINMAP(P7_CAM_5_VS),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
	P7_INIT_PINMAP(P7_CAM_5_DATA08),
	P7_INIT_PINMAP(P7_CAM_5_DATA09),
};

static struct avicam_dummy_info cam5_dummy_driver_info = {
	.dev_id = 5,
	.format = {
		.code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.field = V4L2_FIELD_NONE,
		.width = 4640,
		.height = 3320,
	},
};

#define CAM5_PWM_PERIOD_NS		38 /* 27 MHz approx */
#define CAM5_PWM			10
#define CAM5_NRST_GPIO 			207
static struct pwm_device *cam5_pwm_device = NULL;
static int cam5_power_on(void)
{
	int ret = 0;

	if (cam5_pwm_device == NULL) {
		cam5_pwm_device = pwm_request(CAM5_PWM, "cam5-pwm");
		if (IS_ERR(cam5_pwm_device)) {
			ret = PTR_ERR(cam5_pwm_device);
			goto err_alloc;
		}
	}

	ret = pwm_config(cam5_pwm_device,
			 0,
			 CAM5_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	ret = pwm_enable(cam5_pwm_device);
	if (ret)
		goto err_config;

	gpio_set_value_cansleep(CAM5_NRST_GPIO, 1);

	return 0;

err_config:
	pwm_free(cam5_pwm_device);
	cam5_pwm_device = NULL;
err_alloc:
	pr_warning("failed to set clock for mt9f002 chip\n");
	return ret;
}

static int cam5_power_off(void)
{
	gpio_set_value_cansleep(CAM5_NRST_GPIO, 0);
	pwm_disable(cam5_pwm_device);
	return 0;
}

static int cam5_set_power(int on)
{
	if (on == MT9F002_POWER_ON)
		return cam5_power_on();
	else
		return cam5_power_off();
}

static struct mt9f002_platform_data cam5_mt9f002_platform_data = {
	.interface = MT9F002_Parallel,
	.pixel_depth = MT9F002_10bits,
	.number_of_lanes = MT9F002_2lane,
	.ext_clk_freq_mhz = 26315000000,
	.output_clk_freq_mhz = 96000000000,
	.set_power = &cam5_set_power,
};

static struct i2c_board_info cam5_mt9f002_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9f002", 0x10),
		.platform_data = &cam5_mt9f002_platform_data,
	}
};

static struct ar0330_platform_data cam5_ar0330_platform_data = {
	.clk_rate = 26000000,
	.max_vco_rate = 588000000,
	.max_op_clk_rate = 98000000,
	.hw_bus = AR0330_HW_BUS_PARALLEL,
	.set_power = &cam5_set_power,
};

static struct i2c_board_info cam5_ar0330_i2c_devices[] = {
	{
		I2C_BOARD_INFO("ar0330", 0x10),
		.platform_data = &cam5_ar0330_platform_data,
	}
};

#define CAM5_I2C_BUS	0
static struct avicam_subdevs cam5_subdevs[] = {
	{
		.board_info = &cam5_ar0330_i2c_devices[0],
		.i2c_adapter_id = CAM5_I2C_BUS,
	},
	{
		.board_info = &cam5_mt9f002_i2c_devices[0],
		.i2c_adapter_id = CAM5_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_platform_data cam5_pdata = {
	.cam_cap	   = AVI_CAP_CAM_5,
	.enable_stats      = 1,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 1,
		.ihs        = 1,
		.ipc        = 0,
		.psync_en   = 1,
		.ror_lsb    = 1,
	},
	.bus_width	   = 16,
	.subdevs	   = cam5_subdevs,
	.dummy_driver_info = &cam5_dummy_driver_info,
};

#define CAM5_AVI_RAM_SIZE ((4640 * 3320 * 2 + 2048) * 3)

static u64 cam5_dma_mask = DMA_BIT_MASK(32);

static struct platform_device cam5_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

void __init cameradb_probe(struct p7_board const* board)
{
	p7_init_p7pwm(&p7dev_pwm_pdata,
	              p7dev_pwm_pins,
		      ARRAY_SIZE(p7dev_pwm_pins));

	p7_reserve_avicammem(&cam4_dev, CAM4_AVI_RAM_SIZE);
	p7_reserve_avicammem(&cam5_dev, CAM5_AVI_RAM_SIZE);

	p7_init_avicam(&cam4_dev,
		       &cam4_ar0330_pdata,
		       cam4_pins,
		       ARRAY_SIZE(cam4_pins));

	p7_init_avicam(&cam5_dev,
		       &cam5_pdata,
		       cam5_pins,
		       ARRAY_SIZE(cam5_pins));
	p7brd_export_gpio(127, GPIOF_OUT_INIT_HIGH, "cam4-nrst");
	p7brd_export_gpio(207, GPIOF_OUT_INIT_HIGH, "cam5-nrst");

	p7_init_gpu_fb(0, 0, 4);
}

