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
#include <linux/delay.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "board.h"
#include "board-common.h"
#include "pinctrl.h"
#include "avi.h"
#include "spi.h"
#include <media/video/avicam.h>

#define CAMERA_BRD_NAME   "sicilia"

/*
 * PWM
 */
#include "p7_pwm.h"

static struct p7pwm_conf sicilia_conf_clock_camera = {
	.period_precision = 10,          /* Precision for sensors */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_pdata sicilia_pwm_pdata = {
	.conf = {
		[5]  = &sicilia_conf_clock_camera, /* CAM2 */
		[7]  = &sicilia_conf_clock_camera, /* CAM3 */
		[9]  = &sicilia_conf_clock_camera, /* CAM4 */
		[10] = &sicilia_conf_clock_camera, /* CAM5 */
		[12] = &sicilia_conf_clock_camera, /* CAM0 */
	}
};

static struct pinctrl_map sicilia_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_05),
	P7_INIT_PINMAP(P7_PWM_07),
	P7_INIT_PINMAP(P7_PWM_09),
	P7_INIT_PINMAP(P7_PWM_10),
	P7_INIT_PINMAP(P7_PWM_12),
};

static int sicilia_pwm_power_on(struct pwm_device **dev,
				int                 pwm,
				const char         *label,
				int                 period_ns)
{
	int ret = 0;

	if (*dev == NULL) {
		*dev = pwm_request(pwm, label);
		if (IS_ERR(*dev)) {
			*dev = NULL;
			ret = PTR_ERR(*dev);
			pr_err("Unable to request PWM for '%s' (%d)\n",
				label, ret);
			goto err_request;
		}
	}

	ret = pwm_config(*dev, 0, period_ns);
	if (ret) {
		pr_err("Unable to config PWM for '%s' (%d)\n", label, ret);
		goto err_config;
	}

	ret = pwm_enable(*dev);
	if (ret) {
		pr_err("Unable to enable PWM for '%s' (%d)\n", label, ret);
		goto err_config;
	}

	return 0;

err_config:
	pwm_free(*dev);
	*dev = NULL;

err_request:
	return ret;
}

static int sicilia_pwm_power_off(struct pwm_device **dev)
{
	pwm_disable(*dev);
	pwm_put(*dev);
	*dev = NULL;

	return 0;
}

/*
 * Multispec cameras
 */
#include <media/mt9m021.h>

static struct pinctrl_map sicilia_cam_red_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_3_CLK),
	P7_INIT_PINMAP(P7_CAM_3_HS),
	P7_INIT_PINMAP(P7_CAM_3_VS),
	P7_INIT_PINMAP(P7_CAM_3_DATA00),
	P7_INIT_PINMAP(P7_CAM_3_DATA01),
	P7_INIT_PINMAP(P7_CAM_3_DATA02),
	P7_INIT_PINMAP(P7_CAM_3_DATA03),
	P7_INIT_PINMAP(P7_CAM_3_DATA04),
	P7_INIT_PINMAP(P7_CAM_3_DATA05),
	P7_INIT_PINMAP(P7_CAM_3_DATA06),
	P7_INIT_PINMAP(P7_CAM_3_DATA07),
	P7_INIT_PINMAP(P7_CAM_3_DATA08a),
	P7_INIT_PINMAP(P7_CAM_3_DATA09a),
};

static struct pinctrl_map sicilia_cam_red_edge_pins[] __initdata = {
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

static struct pinctrl_map sicilia_cam_near_infrared_pins[] __initdata = {
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

struct pinctrl_map_size {
	struct pinctrl_map* p_map;
	size_t size;
};

static struct pinctrl_map_size sicilia_multispec_cam_map_size[] __initdata = {
	{
		.p_map = sicilia_cam_red_pins,
		.size  = ARRAY_SIZE(sicilia_cam_red_pins),
	},
	{
		.p_map = sicilia_cam_red_edge_pins,
		.size  = ARRAY_SIZE(sicilia_cam_red_edge_pins),
	},
	{
		.p_map = sicilia_cam_near_infrared_pins,
		.size  = ARRAY_SIZE(sicilia_cam_near_infrared_pins),
	},
};

#define MULTISPEC_CAM_WIDTH          1280
#define MULTISPEC_CAM_HEIGHT          720
#define MULTISPEC_CAM_PIXEL_SIZE        2
#define MULTISPEC_CAM_N_BUFFERS         4
#define MULTISPEC_CAM_REFCLK     27000000
#define MULTISPEC_CAM_TGTCLK     74250000
#define MULTISPEC_CAM_I2C_BUS           1

#define MULTISPEC_CAM_RAM_SIZE PAGE_ALIGN(MULTISPEC_CAM_WIDTH *      \
					  MULTISPEC_CAM_HEIGHT *     \
					  MULTISPEC_CAM_PIXEL_SIZE * \
					  MULTISPEC_CAM_N_BUFFERS)

static u64 sicilia_multispec_cam_dma_mask = DMA_BIT_MASK(32);

static union avi_cam_interface sicilia_multispec_cam_interface = {
	.itu656	    = 0,
	.pad_select = 0,
	.ivs        = 1,
	.ihs        = 1,
	.ipc        = 0,
	.psync_en   = 0,
	.ror_lsb    = 0,
};

struct sicilia_multispec_cam {
	/* PWM */
	struct pwm_device           *pwm_device;
	int                          pwm_id;
	int                          gpio_id;
	char                        *label;
	int                        (*set_power)(int on);
	/* AVI */
	struct platform_device       cam_dev;
	struct avicam_platform_data  cam_pdata;
	/* Subdevs */
	struct avicam_subdevs        subdevs[2];
	struct i2c_board_info        subdev_info;
	struct mt9m021_platform_data subdev_pdata;
};

static int sicilia_cam_red_set_power(int on);
static int sicilia_cam_red_edge_set_power(int on);
static int sicilia_cam_near_infrared_set_power(int on);

static struct sicilia_multispec_cam sicilia_multispec_cams[] = {
	{
		.pwm_device = NULL,
		.pwm_id     = 7,
		.gpio_id    = 125,
		.label      = "red",
		.set_power  = &sicilia_cam_red_set_power,
	},
	{
		.pwm_device = NULL,
		.pwm_id     = 9,
		.gpio_id    = 127,
		.label      = "red-edge",
		.set_power  = &sicilia_cam_red_edge_set_power,
	},
	{
		.pwm_device = NULL,
		.pwm_id     = 10,
		.gpio_id    = 207,
		.label      = "near-infrared",
		.set_power  = &sicilia_cam_near_infrared_set_power,
	},
};

static int sicilia_multispec_cam_power_on(struct pwm_device **dev,
					  int                 pwm,
					  const char         *label)
{
	int ret = sicilia_pwm_power_on(dev,
				       pwm,
				       label,
				       1000000000 / MULTISPEC_CAM_REFCLK);

	/* Make sure the power is on */
	if (ret == 0)
		msleep(1);

	return ret;
}

static int sicilia_multispec_cam_power_off(struct pwm_device **dev)
{
	return sicilia_pwm_power_off(dev);
}

static int sicilia_cam_red_set_power(int on)
{
	struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[0];

	if (on)
		return sicilia_multispec_cam_power_on(&cam->pwm_device,
						      cam->pwm_id,
						      cam->label);
	else
		return sicilia_multispec_cam_power_off(&cam->pwm_device);
}

static int sicilia_cam_red_edge_set_power(int on)
{
	struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[1];

	if (on)
		return sicilia_multispec_cam_power_on(&cam->pwm_device,
						      cam->pwm_id,
						      cam->label);
	else
		return sicilia_multispec_cam_power_off(&cam->pwm_device);
}

static int sicilia_cam_near_infrared_set_power(int on)
{
	struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[2];

	if (on)
		return sicilia_multispec_cam_power_on(&cam->pwm_device,
						      cam->pwm_id,
						      cam->label);
	else
		return sicilia_multispec_cam_power_off(&cam->pwm_device);
}

static int __init sicilia_i2c_write(struct i2c_adapter *adap, u8 id,
				    u16 addr, u16 val)
{
	union i2c_smbus_data data = {
		.block = {
			3,                 /* Length */
			addr & 0xff,       /* addr LSB */
			(val >> 8) & 0xff, /* val  MSB */
			val & 0xff,        /* val  LSB */
		},
	};

	return i2c_smbus_xfer(adap, id, 0,
			     I2C_SMBUS_WRITE, (addr >> 8) & 0xff,
			     I2C_SMBUS_I2C_BLOCK_DATA, &data);
}

static int __init sicilia_aptina_change_id(u8 new_id)
{
	int ret;

	/* Unlock */
	ret = sicilia_i2c_write(i2c_get_adapter(MULTISPEC_CAM_I2C_BUS),
				0x10, 0x301a, 0xd0);
	if (ret < 0)
		return ret;

	/* Change ID */
	ret = sicilia_i2c_write(i2c_get_adapter(MULTISPEC_CAM_I2C_BUS),
				0x10, 0x31fc, new_id << 1);
	if (ret < 0)
		return ret;

	/* Lock back */
	ret = sicilia_i2c_write(i2c_get_adapter(MULTISPEC_CAM_I2C_BUS),
				new_id, 0x301a, 0xd8);
	if (ret < 0)
		return ret;

	return 0;
}

static void __init sicilia_init_multispec_cams(void)
{
	struct platform_device      *dev;
	struct avicam_subdevs       *subdevs;
	struct avicam_platform_data *pdata;
	struct i2c_board_info       *info;
	int                          i;

	for (i = 0 ; i < ARRAY_SIZE(sicilia_multispec_cams) ; i++) {
		struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[i];
		int                           cam_id = i + 2;

		/* De-reset cam */
		gpio_request_one(P7_GPIO_NR(cam->gpio_id),
				 GPIOF_OUT_INIT_HIGH,
				 cam->label);

		/* All cameras are on the same i2c bus, thus at reset they all
		 * have the same slave address.  Here we power-on each camera
		 * one by one, and we change its slave address.
		 */
		sicilia_multispec_cam_power_on(&cam->pwm_device,
					       cam->pwm_id,
					       cam->label);

		if (sicilia_aptina_change_id(0x10 | cam_id) < 0) {
			pr_warn("failed to changed ID for camera %d\n", cam_id);
			goto power_off;
		}

		dev     = &cam->cam_dev;
		subdevs = cam->subdevs;
		info    = &cam->subdev_info;
		pdata   = &cam->cam_pdata;

		/* Camera device */
		dev->id                    = cam_id;
		dev->name                  = "avicam";
		dev->dev.dma_mask          = &sicilia_multispec_cam_dma_mask;
		dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

		/* AVI subdev */
		cam->subdev_pdata.set_power     = cam->set_power;
		cam->subdev_pdata.ext_clock     = MULTISPEC_CAM_REFCLK;
		cam->subdev_pdata.pix_clock     = MULTISPEC_CAM_TGTCLK;
		/* We cannot reset the sensor since we have changed all i2c
		 * slave addresses. Resetting the sensor will put back the
		 * default slave address.
		 */
		cam->subdev_pdata.no_soft_reset = true;

		strlcpy(info->type, "mt9m021", sizeof(info->type));
		info->addr = 0x10 | cam_id;
		info->platform_data = &cam->subdev_pdata;

		subdevs->board_info     = info;
		subdevs->i2c_adapter_id = MULTISPEC_CAM_I2C_BUS;

		/* AVI cam platform data */
		pdata->cam_cap           = AVI_CAP_CAM_0 << dev->id;
		pdata->interface         = sicilia_multispec_cam_interface;
		pdata->bus_width         = 16;
		pdata->subdevs           = subdevs;

		p7_reserve_avicammem(dev, MULTISPEC_CAM_RAM_SIZE);

		p7_init_avicam(dev, pdata,
			       sicilia_multispec_cam_map_size[i].p_map,
			       sicilia_multispec_cam_map_size[i].size);

power_off:
		sicilia_multispec_cam_power_off(&cam->pwm_device);
	}
}

/*
 * RGB camera
 */
#include <media/ov16825.h>
#include <media/tc358746a.h>

#define MAINCAM_NRST_GPIO      123
#define MAINCAM_PWM            5
#define MAINCAM_REFCLK    26000000
#define MAINCAM_I2C_BUS          1

static struct pinctrl_map sicilia_maincam_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_2_CLK),
	P7_INIT_PINMAP(P7_CAM_2_HS),
	P7_INIT_PINMAP(P7_CAM_2_VS),
	P7_INIT_PINMAP(P7_CAM_2_DATA00),
	P7_INIT_PINMAP(P7_CAM_2_DATA01),
	P7_INIT_PINMAP(P7_CAM_2_DATA02),
	P7_INIT_PINMAP(P7_CAM_2_DATA03),
	P7_INIT_PINMAP(P7_CAM_2_DATA04),
	P7_INIT_PINMAP(P7_CAM_2_DATA05),
	P7_INIT_PINMAP(P7_CAM_2_DATA06),
	P7_INIT_PINMAP(P7_CAM_2_DATA07),
	P7_INIT_PINMAP(P7_CAM_2_DATA08),
	P7_INIT_PINMAP(P7_CAM_2_DATA09),
};

static struct pwm_device *sicilia_maincam_pwm = NULL;

static int sicilia_maincam_power_on(void)
{
	int ret = sicilia_pwm_power_on(&sicilia_maincam_pwm,
				       MAINCAM_PWM,
				       "maincam",
				       1000000000 / MAINCAM_REFCLK);

	gpio_set_value_cansleep(MAINCAM_NRST_GPIO, 0);

	/* Make sure the power is on */
	if (ret == 0)
		msleep(50);

	return ret;
}

static int sicilia_maincam_power_off(void)
{
	sicilia_pwm_power_off(&sicilia_maincam_pwm);
	gpio_set_value_cansleep(MAINCAM_NRST_GPIO, 1);

	return 0;
}

/* The TC358746A and AR1820 are sharing the same PWM and nRST GPIO, thus
 * before powering off, we need to check if the 2nd device is not on.
 */
static bool sicilia_maincam_tc358746a_on = 0;
static bool sicilia_maincam_ov16825_on = 0;

static int sicilia_maincam_ov16825_set_power(int on)
{
	sicilia_maincam_ov16825_on = (on != 0);

	if (on) {
		return sicilia_maincam_power_on();
	} else {
		if (sicilia_maincam_tc358746a_on)
			return 0;
		else
			return sicilia_maincam_power_off();
	}
}

static int sicilia_maincam_tc358746a_set_power(int on)
{
	sicilia_maincam_tc358746a_on = (on != 0);

	if (on) {
		return sicilia_maincam_power_on();
	} else {
		if (sicilia_maincam_ov16825_on)
			return 0;
		else
			return sicilia_maincam_power_off();
	}
}

static struct ov16825_platform_data sicilia_maincam_ov16825_subdev_pdata = {
	.set_power = &sicilia_maincam_ov16825_set_power,
	.ext_clk   = MAINCAM_REFCLK,
};

static struct i2c_board_info sicilia_maincam_ov16825_i2c_device = {
	I2C_BOARD_INFO("ov16825", OV16825_I2C_ADDR),
	.platform_data = &sicilia_maincam_ov16825_subdev_pdata,
};

static struct avicam_subdevs sicilia_maincam_ov16825_subdevs[] = {
	{
		.board_info     = &sicilia_maincam_ov16825_i2c_device,
		.i2c_adapter_id = MAINCAM_I2C_BUS,
		.subdevs        = NULL,
	},
	{ NULL, 0 },
};

static struct tc358746a_platform_data sicilia_maincam_tc358746a_subdev_pdata = {
	.set_power = &sicilia_maincam_tc358746a_set_power,
	.refclk    = MAINCAM_REFCLK,
	.lanes     = 1,
};

static struct i2c_board_info sicilia_maincam_tc358746a_i2c_device = {
	I2C_BOARD_INFO("tc358746a", TC358746A_I2C_ADDR),
	.platform_data = &sicilia_maincam_tc358746a_subdev_pdata,
};

static struct avicam_subdevs sicilia_maincam_tc358746a_subdevs[] = {
	{
		.board_info     = &sicilia_maincam_tc358746a_i2c_device,
		.i2c_adapter_id = MAINCAM_I2C_BUS,
		.subdevs        = sicilia_maincam_ov16825_subdevs,
	},
	{ NULL, 0 },
};

static struct avicam_platform_data sicilia_maincam_pdata = {
	.cam_cap	   = AVI_CAP_CAM_2,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 1,
		.ihs        = 1,
		.ipc        = 0,
		.psync_en   = 0,
		.ror_lsb    = 0,
	},
	.bus_width	   = 16,
	.subdevs	   = sicilia_maincam_tc358746a_subdevs,
};

#define MAINCAM_AVI_RAM_SIZE PAGE_ALIGN(1280 * 720 * 2 * 4)

static u64 sicilia_maincam_dma_mask = DMA_BIT_MASK(32);

static struct platform_device sicilia_maincam_dev = {
	.name = "avicam",
	.id   = 0,
	.dev  = {
		.dma_mask          = &sicilia_maincam_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

/*
 * SPI
 */
static struct pinctrl_map sicilia_s6spi_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12),   /* CLK */
	P7_INIT_PINMAP(P7_SPI_15),   /* MOSI */
};

/*
 * We'll need only MOSI and CLK signals, together with 2 GPIOs to program
 * Spartan6 FPGA.
 */
static struct p7spi_swb const sicilia_s6spi_swb[] = {
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_OUT,     P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data sicilia_s6spi_cdata = {
	.half_duplex        = true,
	.read               = false,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.fifo_wcnt          = 16,
	.thres_wcnt         = 12,
	.tsetup_ss_ns       = 1,
	.thold_ss_ns        = 1,
	.toffclk_ns         = 1,
	.toffspi_ns         = 1,
	.tcapture_delay_ns  = 0,
};

static P7_DECLARE_SPIM_SLAVE(sicilia_s6spi_dev,
                             "spidev",
                             NULL,
                             &sicilia_s6spi_cdata,
                             10000000,
                             SPI_MODE_0);

void __init siciliadb_probe(struct p7_board const* board)
{
	int err;

	p7_init_p7pwm(&sicilia_pwm_pdata,
	              sicilia_pwm_pins,
		      ARRAY_SIZE(sicilia_pwm_pins));

	/* RGB Camera */
	p7_reserve_avicammem(&sicilia_maincam_dev, MAINCAM_AVI_RAM_SIZE);
	p7_init_avicam(&sicilia_maincam_dev,
		       &sicilia_maincam_pdata,
		       sicilia_maincam_pins,
		       ARRAY_SIZE(sicilia_maincam_pins));
	p7brd_export_gpio(MAINCAM_NRST_GPIO,
			  GPIOF_OUT_INIT_HIGH, "maincam-nrst");

	/* Multispec Cameras */
	sicilia_init_multispec_cams();

	p7_init_spim(2,
	             sicilia_s6spi_pins,
	             ARRAY_SIZE(sicilia_s6spi_pins),
	             sicilia_s6spi_swb);

	err = p7_init_spim_slave(2, &sicilia_s6spi_dev);
	WARN(err, "%s: failed to initialize Spartan6 SPI interface (%d)\n",
		  board->name, err);
}
