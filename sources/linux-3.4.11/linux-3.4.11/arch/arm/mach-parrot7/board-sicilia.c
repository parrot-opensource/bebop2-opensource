/**
 * linux/arch/arm/mach-parrot7/board-sicilia.c - Sicilia board implementation
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Oct 2014
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <video/avi.h>
#include <video/avifb.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/delay.h>
#include <spi/p7-spi.h>
#include <linux/init.h>
#include <linux/ramoops.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <asm/system_info.h>
#include <mach/p7.h>
#include "common.h"
#include "system.h"
#include "board.h"
#include "board-common.h"
#include "pinctrl.h"
#include "avi.h"
#include "spi.h"
#include "venc.h"
#include "usb.h"
#include "gpio.h"
#include "p7mu.h"
#include "crypto.h"
#include "wl18xx.h"
#include "p7_temperature.h"

#include "board-drone-common.h"

#include <media/video/avicam.h>

#define SICILIA_BRD_NAME   "sicilia"

#define CAM_POWER_EN_HW00        P7_GPIO_NR(66)
#define CAM_POWER_EN_HW01        P7_GPIO_NR(86)
#define PUSH_BUTTON              P7_GPIO_NR(75)
/* output VSYNC on id pin of usb0 */
#define MAINCAM_VS_OUTPUT        P7_GPIO_NR(81)
#define CAM_SYNCH_OUTPUT         P7_GPIO_NR(88)
#define USB_IRR_PWEN_GPIO        P7_GPIO_NR(91)
#define USB_IRR_OC_GPIO          P7_GPIO_NR(92)
#define MAINCAM_BRIDGE_NRST_GPIO P7_GPIO_NR(143)
#define MAINCAM_NRST_GPIO        P7_GPIO_NR(144)
#define MONO_TRIGGER_GPIO     P7_GPIO_NR(174)

/*
 * GPIO
 */
static unsigned const sicilia_default_irq_gpios[] = {
	PUSH_BUTTON,		/* PUSH_BUTTON */
	USB_IRR_OC_GPIO,	/* USB OC */
};

/* p7gpio_filter_phase  */
/* cam sync
	153 : cam0
	152 : cam1
	175 : cam4
	207 : cam5
	210 : cam3
 */
static struct p7gpio_filter_phase sicilia_irq_gpios_filter[] = {
	{
		P7_GPIO_NR(153),	/* IMU_FSYNC = CAM_HD_VS */
		P7_GPIO_NR(76),	/* GYRO_INT */
		20, /* filter */
		GPIO_MEASURE_STOP
	}
};

static int __init sicilia_board_get_rev(void)
{
	static int board_rev = -1;
	int gpios[] = {71, 72, 73, 74};
	int i;

	if (board_rev != -1)
		return board_rev;

	board_rev = 0;
	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		int err, val;
		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
			0,
			"sicilia rev");
		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			board_rev |= (val << i);
			gpio_free(P7_GPIO_NR(gpios[i]));
		}
	}
	return board_rev;
}

/*
 * PWM
 */
#include "p7_pwm.h"

static struct p7pwm_conf sicilia_conf_clock_camera = {
	.period_precision = 5,        /* Precision for sensors */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf sicilia_conf_pwm_leds = {
	.period_precision = 5,
	.duty_precision   = 5,
	.mode             = P7PWM_MODE_NORMAL,
};

#define SICILIA_PWM_LED_RED   8
#define SICILIA_PWM_LED_GREEN 7
#define SICILIA_PWM_LED_BLUE  9

static struct p7pwm_pdata sicilia_pwm_pdata = {
	.conf = {
		[0]  = &sicilia_conf_clock_camera, /* SPECTRAL CAM */
		[1]  = &sicilia_conf_clock_camera, /* MIPI CAM */
		[5]  = &sicilia_conf_clock_camera, /* IMU CLKIN */
		[6]  = &sicilia_conf_pwm_leds,     /* IMU THERMAL */
		[7]  = &sicilia_conf_pwm_leds,     /* GREEN LED */
		[8]  = &sicilia_conf_pwm_leds,     /* RED   LED */
		[9]  = &sicilia_conf_pwm_leds,     /* BLUE  LED */
	}
};

static unsigned long sicilia_pwm_pinconfig[] = {
	P7CTL_DRV_CFG(0),      /* Drive strength 0 (reg=1) */
};

static unsigned long sicilia_pwm_mclk_pinconfig[] = {
	P7CTL_DRV_CFG(4),
};

static struct pinctrl_map sicilia_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_00),
	P7_INIT_PINCFG(P7_PWM_00, sicilia_pwm_mclk_pinconfig),
	P7_INIT_PINMAP(P7_PWM_01),
	P7_INIT_PINCFG(P7_PWM_01, sicilia_pwm_mclk_pinconfig),
	P7_INIT_PINMAP(P7_PWM_05),
	P7_INIT_PINMAP(P7_PWM_06),
	P7_INIT_PINMAP(P7_PWM_07),
	P7_INIT_PINCFG(P7_PWM_07, sicilia_pwm_pinconfig),
	P7_INIT_PINMAP(P7_PWM_08),
	P7_INIT_PINCFG(P7_PWM_08, sicilia_pwm_pinconfig),
	P7_INIT_PINMAP(P7_PWM_09),
	P7_INIT_PINCFG(P7_PWM_09, sicilia_pwm_pinconfig),
};

static int sicilia_pwm_power_on(struct pwm_device **dev,
				int                 pwm,
				const char         *label,
				int                 freq_hz)
{
	int ret       = 0;
	int period_ns = 1000000000 / freq_hz;

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

static struct pinctrl_map sicilia_cam_green_pins[] __initdata = {
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
		.p_map = sicilia_cam_green_pins,
		.size  = ARRAY_SIZE(sicilia_cam_green_pins),
	},
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
#define MULTISPEC_CAM_HEIGHT          960
#define MULTISPEC_CAM_EXTRA_DATA       10
#define MULTISPEC_CAM_PIXEL_SIZE        2
#define MULTISPEC_CAM_N_BUFFERS         4
#define MULTISPEC_CAM_REFCLK     26000000
#define MULTISPEC_CAM_TGTCLK     71500000
#define MULTISPEC_CAM_I2C_BUS           2

#define MULTISPEC_CAM_RAM_SIZE PAGE_ALIGN(MULTISPEC_CAM_WIDTH *      \
					  (MULTISPEC_CAM_HEIGHT + \
					  MULTISPEC_CAM_EXTRA_DATA) * \
					  MULTISPEC_CAM_PIXEL_SIZE * \
					  MULTISPEC_CAM_N_BUFFERS)

static u64 sicilia_multispec_cam_dma_mask = DMA_BIT_MASK(32);

static union avi_cam_interface sicilia_multispec_cam_interface = {
	.itu656	    = 0,
	.pad_select = 0,
	.ivs        = 1,
	.ihs        = 1,
	.ipc        = 0,
	.psync_en   = 1,
	.psync_rf   = 1,
	.ror_lsb    = 0,
};

static struct pwm_device *sicilia_multispec_cam_pwm = NULL;

struct sicilia_multispec_cam {
	/* PWM */
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

static int sicilia_cam_green_set_power(int on);
static int sicilia_cam_red_set_power(int on);
static int sicilia_cam_red_edge_set_power(int on);
static int sicilia_cam_near_infrared_set_power(int on);

static struct sicilia_multispec_cam sicilia_multispec_cams[] = {
	{
		.gpio_id    = 134,
		.label      = "green",
		.set_power  = &sicilia_cam_green_set_power,
	},
	{
		.gpio_id    = 135,
		.label      = "red",
		.set_power  = &sicilia_cam_red_set_power,
	},
	{
		.gpio_id    = 136,
		.label      = "red-edge",
		.set_power  = &sicilia_cam_red_edge_set_power,
	},
	{
		.gpio_id    = 137,
		.label      = "near-infrared",
		.set_power  = &sicilia_cam_near_infrared_set_power,
	},
};

#define MULTISPEC_PWM_ID 0
#define MULTISPEC_PWM_NAME "multispec"

static int sicilia_i2c_write(struct i2c_adapter *adap, u8 id,
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

static int sicilia_aptina_change_id(u8 new_id)
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

static struct mutex sicilia_multispec_lock;

static int sicilia_multispec_cam_power_on(int cam_id, struct sicilia_multispec_cam *cam)
{
	int ret = 0;

	mutex_lock(&sicilia_multispec_lock);
	/* De-reset cam */
	gpio_request_one(P7_GPIO_NR(cam->gpio_id),
			 GPIOF_OUT_INIT_HIGH,
			 cam->label);
	msleep(10);

	ret = sicilia_aptina_change_id(0x10 | cam_id);
	if (ret < 0) {
		pr_warn("failed to changed ID for camera mt9m021 %d\n", cam_id);
		goto unlock;
	}

unlock:
	mutex_unlock(&sicilia_multispec_lock);
	return ret;
}

static int sicilia_multispec_cam_power_off(struct sicilia_multispec_cam *cam)
{
	gpio_free(cam->gpio_id);

	return 0;
}

static int sicilia_multispec_cam_set_power(int on, int cam_id)
{
	struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[cam_id];

	if (on)
		return sicilia_multispec_cam_power_on(cam_id + 2, cam);

	return sicilia_multispec_cam_power_off(cam);
}

static int sicilia_cam_green_set_power(int on) {
	return sicilia_multispec_cam_set_power(on, 0);
}

static int sicilia_cam_red_set_power(int on) {
	return sicilia_multispec_cam_set_power(on, 1);
}

static int sicilia_cam_red_edge_set_power(int on) {
	return sicilia_multispec_cam_set_power(on, 2);
}

static int sicilia_cam_near_infrared_set_power(int on) {
	return sicilia_multispec_cam_set_power(on, 3);
}

static void __init sicilia_init_multispec_cams(void)
{
	struct platform_device      *dev;
	struct avicam_subdevs       *subdevs;
	struct avicam_platform_data *pdata;
	struct i2c_board_info       *info;
	int                          i;
	int ret;

	ret = sicilia_pwm_power_on(&sicilia_multispec_cam_pwm,
				   MULTISPEC_PWM_ID,
				   MULTISPEC_PWM_NAME,
				   MULTISPEC_CAM_REFCLK);

	if (ret < 0) {
		pr_err("failed to initialize PWM %d\n", MULTISPEC_PWM_ID);
		return;
	}

	gpio_request_one(P7_GPIO_NR(2),
			 GPIOF_OUT_INIT_LOW,
			 "CAM MONO I2C SELECT");

	gpio_request_one(P7_GPIO_NR(147),
			 GPIOF_OUT_INIT_LOW,
			 "CAM MONO I2C STDBY");

	p7brd_export_gpio(P7_GPIO_NR(MONO_TRIGGER_GPIO),
			 GPIOF_OUT_INIT_LOW,
			 "CAM MONO TRIGGER");

	msleep(1);
	mutex_init(&sicilia_multispec_lock);

	for (i = 0 ; i < ARRAY_SIZE(sicilia_multispec_cams) ; i++) {
		struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[i];
		int                           cam_id = i + 2;

		/* De-reset cam */
		if (sicilia_multispec_cam_power_on(cam_id, cam) < 0) {
			pr_err("Camera %d power on failed\n", cam_id);
			goto power_off;
		}

		dev     = &cam->cam_dev;
		subdevs = cam->subdevs;
		info    = &cam->subdev_info;
		pdata   = &cam->cam_pdata;

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
		pdata->vb2_cache_flags   = VB2_CACHE_DMA_CONTIG;

		p7_init_avicam(dev, pdata,
			       sicilia_multispec_cam_map_size[i].p_map,
			       sicilia_multispec_cam_map_size[i].size);
	power_off:
		sicilia_multispec_cam_power_off(cam);
	}
}


static void __init sicilia_multispec_cams_reserve_mem(void)
{
	struct platform_device      *dev;
	int                          i;

	for (i = 0; i < ARRAY_SIZE(sicilia_multispec_cams); i++) {
		struct sicilia_multispec_cam *cam = &sicilia_multispec_cams[i];
		int                           cam_id = i + 2;

		dev     = &cam->cam_dev;

		/* Camera device */
		dev->id                    = cam_id;
		dev->name                  = "avicam";
		dev->dev.dma_mask          = &sicilia_multispec_cam_dma_mask;
		dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

		p7_reserve_avicammem(dev, MULTISPEC_CAM_RAM_SIZE);
	}
}

/*
 * RGB camera
 */
#include <media/ov16825.h>
#include <media/tc358746a.h>

#define MAINCAM_REFCLK      26000000
#define MAINCAM_BRIDGE_I2C_ADDR  0xe
#define MAINCAM_VCM_I2C_ADDR     0xc
#define MAINCAM_I2C_BUS            2
#define MAINCAM_LANES_NB           4
#define MAINCAM_VCM_FOCUS        220

static struct pinctrl_map sicilia_maincam_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_HS),
	P7_INIT_PINMAP(P7_CAM_0_VS),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINMAP(P7_CAM_0_DATA16),
	P7_INIT_PINMAP(P7_CAM_0_DATA17),
};

static struct pwm_device *sicilia_maincam_pwm = NULL;

static int sicilia_tc358746a_write(struct i2c_adapter *adapter,
				   u8 addr,
				   u16 reg, u16 val)
{
	int            ret;
	struct i2c_msg msg;
	u8             buf[4];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr  = addr;
	msg.flags = 0;
	msg.len   = sizeof(buf);
	msg.buf   = buf;

	ret = i2c_transfer(adapter, &msg, 1);

	if (ret < 0) {
		return ret;
	}

	return 0;
}


static int sicilia_vcm_configure(struct i2c_adapter *adapter)
{
	int            ret;
	struct i2c_msg msg;
	u8             buf[2];
	unsigned       position = MAINCAM_VCM_FOCUS;

	msg.addr  = MAINCAM_VCM_I2C_ADDR;
	msg.flags = 0;
	msg.len   = sizeof(buf);
	msg.buf   = buf;


	// Protection OFF
	buf[0] = 0xec;
	buf[1] = 0xa3;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret < 0) {
		return ret;
	}

	// Setting
	buf[0] = 0xf2;
	buf[1] = 0x78;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret < 0) {
		return ret;
	}

	// Protection ON
	buf[0] = 0xdc;
	buf[1] = 0x51;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret < 0) {
		return ret;
	}

	// Position
	position = (position << 4) | 0xc;

	buf[0] = (position >> 8) & 0xff;
	buf[1] = position & 0xff;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int sicilia_maincam_power_on(void)
{
	struct i2c_adapter *adapter;
	int                 ret;

	ret = sicilia_pwm_power_on(&sicilia_maincam_pwm,
				   1,
				   "maincam",
				   MAINCAM_REFCLK);

	if (ret != 0) {
		goto done;
	}

	/* In the current hardware it's not a reset, it's a power down and it's
	   active high */
	gpio_set_value_cansleep(MAINCAM_NRST_GPIO, 0);
	gpio_set_value_cansleep(MAINCAM_BRIDGE_NRST_GPIO, 1);

	/* Make sure the power is on */
	msleep(50);

	/* CAM reset is wired on the bridge's GPIO 2. */
	adapter = i2c_get_adapter(MAINCAM_I2C_BUS);
	if (adapter == NULL) {
		ret = -EINVAL;
		goto done;
	}

	/* Set GPIO 2 output */
	ret = sicilia_tc358746a_write(adapter,
				      MAINCAM_BRIDGE_I2C_ADDR,
				      0x0010, 0xfffb);
	if (ret != 0) {
		goto done;
	}

	/* Set GPIO 2 high */
	ret = sicilia_tc358746a_write(adapter,
				      MAINCAM_BRIDGE_I2C_ADDR,
				      0x0014, 0x0004);
	if (ret != 0) {
		goto done;
	}

	/* Configure camera focus using VCM. */
	ret = sicilia_vcm_configure(adapter);
	if (ret != 0) {
		/* I ignore errors because this requires a hardware patch that
		 * not everybody has */
		printk(KERN_WARNING
		       "Could not configure maincam VCM."
		       " You probably need a hardware patch");
		ret = 0;
	}
 done:
	return ret;
}

static int sicilia_maincam_power_off(void)
{
	sicilia_pwm_power_off(&sicilia_maincam_pwm);
	gpio_set_value_cansleep(MAINCAM_NRST_GPIO, 1);
	gpio_set_value_cansleep(MAINCAM_BRIDGE_NRST_GPIO, 0);

	return 0;
}

/* The TC358746A and AR1820 are sharing the same PWM and nRST GPIO, thus
 * before powering off, we need to check if the 2nd device is not on.
 *
 * This implementation assumes that the calls to set_power() will be
 * symmetrical.
 */
static int sicilia_maincam_set_power(int on)
{
	static DEFINE_MUTEX(lock);
	static int use_count = 0;
	int        ret;

	mutex_lock(&lock);

	if (on) {
		if (use_count == 0) {
			printk(KERN_INFO "Powering on maincam\n");

			ret = sicilia_maincam_power_on();

			if (ret == 0) {
				use_count++;
			}
		} else {
			use_count++;
			ret = 0;
		}
	} else {
		use_count--;

		if (use_count == 0) {
			printk(KERN_INFO "Powering off maincam\n");
			ret = sicilia_maincam_power_off();
		} else {
			ret = 0;
		}
	}

	mutex_unlock(&lock);

	return ret;
}

static struct ov16825_platform_data sicilia_maincam_ov16825_subdev_pdata = {
	.set_power = &sicilia_maincam_set_power,
	.ext_clk   = MAINCAM_REFCLK,
	.lanes     = MAINCAM_LANES_NB,
};

static struct i2c_board_info sicilia_maincam_ov16825_i2c_device = {
	I2C_BOARD_INFO("ov16825", 0x10),
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
	.set_power            = &sicilia_maincam_set_power,
	.refclk               = MAINCAM_REFCLK,
	.force_subdev_pixcode = V4L2_MBUS_FMT_SBGGR10_1X10,
	.lanes                = MAINCAM_LANES_NB,
	.calibration_delay_ms = 300,
	.phytimdly            = 10,
};

static struct i2c_board_info sicilia_maincam_tc358746a_i2c_device = {
	I2C_BOARD_INFO("tc358746a", MAINCAM_BRIDGE_I2C_ADDR),
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
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 1,
		.ivs        = 1,
		.ihs        = 1,
		.ipc        = 0,
		.psync_en   = 1,
		.psync_rf   = 1,
		.ror_lsb    = 0,
	},
	.bus_width	   = 10,
	.subdevs	   = sicilia_maincam_tc358746a_subdevs,
};

#define MAINCAM_WIDTH           4608
#define MAINCAM_HEIGHT          3456
#define MAINCAM_PIXEL_SIZE         2
#define MAINCAM_N_BUFFERS          4

#define MAINCAM_AVI_RAM_SIZE PAGE_ALIGN(MAINCAM_WIDTH * \
					MAINCAM_HEIGHT * \
					MAINCAM_PIXEL_SIZE * \
					MAINCAM_N_BUFFERS)

static u64 sicilia_maincam_dma_mask = DMA_BIT_MASK(32);

static struct platform_device sicilia_maincam_dev = {
	.name = "avicam",
	.id   = 0,
	.dev  = {
		.dma_mask          = &sicilia_maincam_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

#if defined (CONFIG_LEDS_PWM) && defined(CONFIG_LEDS_TRIGGERS)
static struct led_pwm pwm_leds[] = {
	{
		.name = "sicilia:green",
		.default_trigger = "none",
		.pwm_id = SICILIA_PWM_LED_GREEN,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "sicilia:red",
		.default_trigger = "none",
		.pwm_id = SICILIA_PWM_LED_RED,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "sicilia:blue",
		.default_trigger = "none",
		.pwm_id = SICILIA_PWM_LED_BLUE,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
};

static struct led_pwm_platform_data pwm_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device sicilia_leds_pwm = {
	.name = "leds_pwm",
	.id = -1,
	.dev = {
		.platform_data = &pwm_data,
	},
};
#endif

/*****************************
 * P7MU Power Management Unit
 *****************************/

#include "p7mu.h"
#include <mfd/p7mu.h>
#include <mach/p7-adc.h>
#include <mach/gpio.h>
#include <spi/p7-spim.h>

#define SICILIA_SPI_SLAVE_P7MU    2

#if 0
static struct pinctrl_map sicilia_spi_slave_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_02), /* CLK */
	P7_INIT_PINMAP(P7_SPI_01), /* MOSI */
	P7_INIT_PINMAP(P7_SPI_00)  /* MISO */
};
static struct p7spi_swb const sicilia_spi_slave_p7mu_swb[] = {
	P7SPI_INIT_SWB(02,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(01,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(00,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spis_ctrl_data sicilia_spi_slave_p7mu_cdata = {
	.common = {
		.half_duplex        = true,
		.read               = true,
		.write              = true,
		.xfer_mode          = P7SPI_SINGLE_XFER,
		.fifo_wcnt          = 16,
		.thres_wcnt         = 8,
		.tsetup_ss_ns       = 1,
		.thold_ss_ns        = 1,
		.toffclk_ns         = 1,
		.toffspi_ns         = 1,
		.tcapture_delay_ns  = 0,
	}
};

static P7_DECLARE_SPIS_MASTER(sicilia_spi_master_p7mu_info,
			      "p7mu-adc",
			      NULL,
			      &sicilia_spi_slave_p7mu_cdata,
			      10 * 1000 * 1000,
			      SPI_MODE_0|SPI_LSB_FIRST);
#endif

static struct pinctrl_map sicilia_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

static struct p7mu_plat_data sicilia_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(80),   /* GPIO 80 is P7MU -> P7 interrupt source */
	.pwm_pins   = {
		[1] = 3                     /* PWM1 multiplexed onto IO_4 pin. */
	},
	.int_32k    = false,            /* External 32kHz clock. */
	.int_32m    = true,             /* No External 48mHz clock. */
};


/* External temperature sensors */
static struct p7_temp_chan p7mu_adc_channels[] = {
	/* 3.3V ref */
	{
		.channel = 3,
		.freq = 160000,
		.name = "ref",
	},
	/* DDR3 */
	{
		.channel = 1,
		.freq = 160000,
		.name = "ddr",
	},
	/* WIFI */
	{
		.channel = 2,
		.freq = 160000,
		.name = "wifi",
	},
};

static struct p7_temp_chan_data p7mu_adc_chan_data = {
        .channels               = p7mu_adc_channels,
        .num_channels           = ARRAY_SIZE(p7mu_adc_channels),
        .temp_mode              = P7_TEMP_FC7100_HW08,
};

static void __init sicilia_init_spi_p7mu(void)
{
	/*
	p7_init_spis(SICILIA_SPI_SLAVE_P7MU,
			sicilia_spi_slave_p7mu_pins,
			ARRAY_SIZE(sicilia_spi_slave_p7mu_pins),
			sicilia_spi_slave_p7mu_swb);

	if (p7_init_spis_master(SICILIA_SPI_SLAVE_P7MU,
				&sicilia_spi_master_p7mu_info))
		pr_err(SICILIA_BRD_NAME ": failed to initialize SPI slave.\n");
	*/

	/* Set DDR voltage to 1.35V */
	/* DIRTY IT HAS TO BE REMOVED FOR DV */
	p7mu_write16(0x6, 0x7023);
	p7mu_write16(0x7, 0x0c5e);

}

/* Reuse the fc7100 driver since it's the same config. We should really rename
 * that */
static struct platform_device sicilia_temp_device = {
	.name = "fc7100-temperature",
	.id = -1,
	.dev.platform_data = &p7mu_adc_chan_data,
};

/***********************
 * SDHCI hosts handling
 ***********************/

#include "sdhci.h"

#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>
#include "gbc.h"

static unsigned long sicilia_sdhci_pins_config_clk[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
	P7CTL_SLR_CFG(2),
};

static unsigned long sicilia_sdhci_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
	P7CTL_SLR_CFG(2),
};

static struct pinctrl_map sicilia_sdhci0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_0_CLK),
	P7_INIT_PINCFG(P7_SD_0_CLK, sicilia_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_0_CMD),
	P7_INIT_PINCFG(P7_SD_0_CMD, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT00),
	P7_INIT_PINCFG(P7_SD_0_DAT00, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT01),
	P7_INIT_PINCFG(P7_SD_0_DAT01, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT02),
	P7_INIT_PINCFG(P7_SD_0_DAT02, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT03),
	P7_INIT_PINCFG(P7_SD_0_DAT03, sicilia_sdhci_pins_config)
};

static struct acs3_plat_data sicilia_sdhci0_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = P7_GPIO_NR(142),
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,     /* emmc is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE|MMC_CAP2_CACHE_CTRL,  /* bus voltage is fixed in hardware */
};

/**********************
 * TI wl18xx wlan
 **********************/

static struct pinctrl_map sicilia_sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, sicilia_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, sicilia_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, sicilia_sdhci_pins_config)
};

static __initdata struct wl18xx_resources sicilia_wl18xx_res_hw00 = {
	.wlirq_gpio = P7_GPIO_NR(4),
	.wlen_gpio = P7_GPIO_NR(3),
	.wl_sdhci_slot = 1,
};

static void __init sicilia_init_wl18xx(void)
{
	init_wl18xx(&sicilia_wl18xx_res_hw00,
		    sicilia_sdhci1_pins,
		    ARRAY_SIZE(sicilia_sdhci1_pins));
}

/******
 * Ram2Ram Scaler/ISP
 ******/

/* buffers in NV12 (4:2:0) */
#define M2M_STAT_DUMMY_BUFFER	(MAINCAM_WIDTH * \
				MAINCAM_HEIGHT * 3 / 2)
#define M2M_STAT_RAM_SIZE	((64 * 50 * 6 * MAINCAM_N_BUFFERS) + \
				M2M_STAT_DUMMY_BUFFER)

#define M2M_ISP_RAM_SIZE	(MAINCAM_WIDTH * \
				MAINCAM_HEIGHT * \
				MAINCAM_N_BUFFERS * 3 / 2)

#define SICILIA_AVI_M2M_SIZE	PAGE_ALIGN(M2M_ISP_RAM_SIZE + \
					   M2M_STAT_RAM_SIZE)

static struct avi_m2m_platform_data sicilia_avi_m2m_pdata[] = {
	{
		.caps = AVI_CAPS_ISP,
		.enable_stats = 1,
		.stat_vb2_cache_flags = VB2_CACHE_DMA_CONTIG |
					VB2_CACHE_FLUSH,
	},
	{
		.caps = AVI_CAPS_ISP,
		.enable_stats = 0,
		.vb2_cache_flags = VB2_CACHE_DMA_CONTIG,
	},
	{ .caps = 0 },
};


static void sicilia_configure_leds(void)
{
	struct pwm_device *pwm;

	pwm = pwm_request(SICILIA_PWM_LED_RED, "sicilia BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 2000000, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(SICILIA_PWM_LED_GREEN, "sicilia BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 0, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(SICILIA_PWM_LED_BLUE, "sicilia BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 0, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}
}

static void __init sicilia_reserve_mem(void)
{
	drone_common_reserve_mem_ramoops();

	p7_reserve_avicammem(&sicilia_maincam_dev, MAINCAM_AVI_RAM_SIZE);
	sicilia_multispec_cams_reserve_mem();

	p7_reserve_avi_m2m_mem(SICILIA_AVI_M2M_SIZE);

#define SICILIA_HX280_SIZE (CONFIG_ARCH_PARROT7_SICILIA_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(SICILIA_HX280_SIZE);

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

static void __init sicilia_init_mach(void)
{
	int cam_power_en;
	int i;
	int ret;

	/* Initialize ramoops */
	drone_common_init_ramoops();

	p7_init_mach();

	p7_init_gpio(NULL, 0);

	system_rev = sicilia_board_get_rev();
	pr_info("sicilia rev %d\n", system_rev);

	/* debug uart */
	p7brd_init_uart(0,0);

	p7brd_init_i2cm(0, 400); /* P7MU */
	/*
	 * i2c-1:
	 *     IMU MPU6050 : Fast-mode supported
	 *     EEPROM M24C64 : Fast-mode supported
	 */
	p7brd_init_i2cm(1, 400);
	/*
	 * i2c-2:
	 *     magnetometer AK8963 : Fast-mode supported
	 *     cameras mt9m031 : Fast-mode supported
	 *     bridge TC358746 : Fast-mode supported
	 *     camera ov16825 : Fast-mode supported
	 */
	p7brd_init_i2cm(2, 400);

	/* p7mu */
	p7_gpio_interrupt_register(sicilia_p7mu_pdata.gpio);
	p7_init_p7mu(0,
		     &sicilia_p7mu_pdata,
		     sicilia_p7mu_pins,
		     ARRAY_SIZE(sicilia_p7mu_pins));
	sicilia_init_spi_p7mu();

	p7_init_temperature();

	/* gpio */
	for (i = 0; i < ARRAY_SIZE(sicilia_default_irq_gpios); i++)
		p7_gpio_interrupt_register(sicilia_default_irq_gpios[i]);

	for (i = 0; i < ARRAY_SIZE(sicilia_irq_gpios_filter); i++)
		p7_gpio_filter_interrupt_register(sicilia_irq_gpios_filter[i]);

	if (system_rev == 0) {
		/* HW00 has no 3.3V ADC ref */
		p7mu_adc_chan_data.channels = &p7mu_adc_channels[1];
		p7mu_adc_chan_data.num_channels -= 1;
		p7mu_adc_chan_data.temp_mode = P7_TEMP_SICILIA;
	}

	platform_device_register(&sicilia_temp_device);

	/* emmc */
	p7brd_init_sdhci(0, &sicilia_sdhci0_pdata, NULL, NULL, NULL,
			 sicilia_sdhci0_pins, ARRAY_SIZE(sicilia_sdhci0_pins));

	/* Enable camera power */
	switch (system_rev) {
	case 0:
		cam_power_en = CAM_POWER_EN_HW00;
		break;
	default:
		cam_power_en = CAM_POWER_EN_HW01;
		// The maincam uses an other starting with HW01 in order not to
		// conflict with the multispectral cameras
		sicilia_maincam_ov16825_i2c_device.addr = 0x36;
		break;
	}

	ret = gpio_request_one(cam_power_en,
			       GPIOF_OUT_INIT_HIGH,
			       "cam_power_en");

	p7_init_p7pwm(&sicilia_pwm_pdata,
	              sicilia_pwm_pins,
		      ARRAY_SIZE(sicilia_pwm_pins));

	sicilia_configure_leds();

	/* pwn leds */
#if defined (CONFIG_LEDS_PWM) && defined(CONFIG_LEDS_TRIGGERS)
	platform_device_register(&sicilia_leds_pwm);
#endif
	/* RGB Camera */
	p7_init_avicam(&sicilia_maincam_dev,
		       &sicilia_maincam_pdata,
		       sicilia_maincam_pins,
		       ARRAY_SIZE(sicilia_maincam_pins));
	p7brd_export_gpio(MAINCAM_BRIDGE_NRST_GPIO,
			  GPIOF_OUT_INIT_LOW, "maincam-bridge-nrst");
	p7brd_export_gpio(MAINCAM_NRST_GPIO,
			  GPIOF_OUT_INIT_LOW, "maincam-nrst");

	/* Multispec Cameras */
	sicilia_init_multispec_cams();

	p7brd_init_udc(0, -1);

	/* mini-USB host port to irradiance board */
	p7brd_init_hcd(1, USB_IRR_PWEN_GPIO);

	p7_init_venc();

	p7_init_avi_m2m(sicilia_avi_m2m_pdata);

	/* Eeprom WP */
	p7brd_export_gpio(P7_GPIO_NR(65), GPIOF_OUT_INIT_HIGH, "eeprom-wp");
	drone_common_init_inv_mpu6050(1, P7_GPIO_NR(76), 5, 5/* pwm */);
	drone_common_export_gpio(P7_GPIO_NR(76), GPIOF_IN, "IMU irq");

	/* misc gpios */
	p7brd_export_gpio(P7_GPIO_NR(82), GPIOF_IN, "COM_Debug_On");
	p7brd_export_gpio(P7_GPIO_NR(152), GPIOF_IN, "CAM2_VS");
	p7brd_export_gpio(P7_GPIO_NR(153), GPIOF_IN, "CAM0_VS");
	p7brd_export_gpio(P7_GPIO_NR(177), GPIOF_IN, "CAM4_VS");
	p7brd_export_gpio(P7_GPIO_NR(207), GPIOF_IN, "CAM5_VS");
	p7brd_export_gpio(P7_GPIO_NR(210), GPIOF_IN, "CAM3_VS");
	p7brd_export_gpio(P7_GPIO_NR(219), GPIOF_IN, "CAM0_BRIDGE_INT_in");

	/* gpio for cam external synchro */
	if (sicilia_board_get_rev() >= 3) {
		/* This gpio will be used to generate a sync signal for eBee.
		 * It will be raised during 10ms each time sicilia captures
		 * a frame to record.
		 */
		p7brd_export_gpio(CAM_SYNCH_OUTPUT, GPIOF_OUT_INIT_LOW, "CAM_SYNCH");
		/* This gpio allows to enable this signal when eBee is present,
		 * or to disable it otherwise
		 */
		p7brd_export_gpio(MAINCAM_VS_OUTPUT, GPIOF_OUT_INIT_LOW, "EN_CAM_SYNC");
	} else {
		/* This gpio will be used to generate a sync signal for eBee
		 * by enabling the maincam vsync signal during 10ms.
		 */
		p7brd_export_gpio(MAINCAM_VS_OUTPUT, GPIOF_OUT_INIT_LOW, "CAM_SYNCH");
	}


	/*
	 * Init Wilink8
	 */
	sicilia_init_wl18xx();
	p7brd_export_gpio(PUSH_BUTTON, GPIOF_IN, "wifi-push-button");
}

extern struct sys_timer p7_low_sysclk_tick_timer;

P7_MACHINE_START(PARROT_SICILIA, "Sicilia board")
  .reserve        = &sicilia_reserve_mem,
  .init_machine   = &sicilia_init_mach,
  .timer          = &p7_low_sysclk_tick_timer,
P7_MACHINE_END
