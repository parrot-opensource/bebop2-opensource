/*
 * linux/arch/arm/mach-parrot7/board-sfx1b.c - Parrot7 sfx1b board implementation
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Christian Rosalie <christian.rosalie@parrot.com>
 * date:    14-Avril-2014
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/pwm.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <media/ov7740.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/usb-p7.h>
#include <mach/ether.h>
#include <spi/p7-spi.h>
#include "board-common.h"
#include "common.h"
#include "pinctrl.h"
#include "venc.h"
#include "spi.h"
#include "usb.h"
#include "avi.h"
#include "p7_temperature.h"
#include "pinctrl.h"
#include "lcd-monspecs.h"
#include "nand.h"
#include "usb.h"
#include "board-drone-common.h"

/* p7mu */
#include <mfd/p7mu.h>
#include <i2c/p7-i2cm.h>
#include "p7mu.h"


#define SFX1B_BRD_NAME "sfx1b"

/*
 * We shall chose Galileo1 or Galileo2 as maicam
 */
#if defined(CONFIG_VIDEO_GALILEO1) && defined(CONFIG_VIDEO_GALILEO2)
#error "Error : Galileo1 and Galileo2 defined !"
#endif

/* Macro to simplify definition of GPIO keys */
#define RX_KEYS_GPIO(_gpio, _code, _active_low, _desc)\
{\
	.gpio						= (P7_GPIO_NR(_gpio)),\
	.type						= EV_KEY,\
	.code						= (_code),\
	.can_disable					= false,\
	.wakeup						= false,\
	/* Not a real button, no need to debounce */ \
	.debounce_interval				= 0,\
	.active_low					= (_active_low),\
	.desc						= (_desc)\
}

/*********
 * GPIOs
 *********/

#include "gpio.h"

static unsigned const sfx1b_irq_gpios[] = {
	73,   /* P7MU interrupt source */
	161,  /* FSTROBE interrupt source */
	81,   /* GPS timepulse interrupt source */
};

#include "p7_pwm.h"

#if defined(CONFIG_PWM_PARROT7) || \
    defined(CONFIG_PWM_PARROT7_MODULE)

static struct p7pwm_conf sfx1b_conf_clock_OV7740 = {
	.period_precision = 5,
	.duty_precision   = 0,            /* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf sfx1b_conf_clock_tc358746a_p7_tegra = {
	.period_precision = 5,
	.duty_precision   = 0,            /* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};


static struct p7pwm_conf sfx1b_conf_clock_maincam_galileo = {
	.period_precision = 5,
	.duty_precision   = 0,
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf sfx1b_conf_clock_maincam_tc358746a = {
	.period_precision = 5,
	.duty_precision   = 0,
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf sfx1b_conf_front_led = {
	.period_precision = 5,
	.duty_precision   = 0,
	.mode             = P7PWM_MODE_NORMAL,
};

static struct p7pwm_pdata sfx1b_pwm_pdata = {
	.conf = {
		[ 3] = &sfx1b_conf_front_led,
		[ 4] = &sfx1b_conf_clock_maincam_galileo,
		[ 5] = &sfx1b_conf_clock_OV7740, /* right camera */
		[ 6] = &sfx1b_conf_clock_OV7740, /* left camera */
		[ 7] = &sfx1b_conf_clock_OV7740, /* bottom camera */
		[ 8] = &sfx1b_conf_clock_OV7740, /* back camera */
		[ 9] = &sfx1b_conf_clock_OV7740, /* head camera */
		[10] = &sfx1b_conf_clock_tc358746a_p7_tegra, /* MCLK bridge toshiba */
		[11] = &sfx1b_conf_clock_maincam_tc358746a,
	}
};

static unsigned long sfx1b_pwm_drivecfg[] = {
	P7CTL_DRV_CFG(0),
};

static struct pinctrl_map sfx1b_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_03),
	P7_INIT_PINCFG(P7_PWM_03, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_04),
	P7_INIT_PINCFG(P7_PWM_04, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_05),
	P7_INIT_PINCFG(P7_PWM_05, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_06),
	P7_INIT_PINCFG(P7_PWM_06, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_07),
	P7_INIT_PINCFG(P7_PWM_07, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_08),
	P7_INIT_PINCFG(P7_PWM_08, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_09),
	P7_INIT_PINCFG(P7_PWM_09, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_10),
	P7_INIT_PINCFG(P7_PWM_10, sfx1b_pwm_drivecfg),
	P7_INIT_PINMAP(P7_PWM_11),
	P7_INIT_PINCFG(P7_PWM_11, sfx1b_pwm_drivecfg),
};

#endif

#define SFX1B_STROBE_GPIO    P7_GPIO_NR(161)
#define SFX1B_GPS_PULSE_GPIO P7_GPIO_NR(81)

/* GPIO keys */
static struct gpio_keys_button p7_gpio_keys[] = {
/*      GPIO                               Code      Active low  Description */
	RX_KEYS_GPIO(SFX1B_STROBE_GPIO,    KEY_UP,   false,      "Galileo1 FSTROBE line"),
	RX_KEYS_GPIO(SFX1B_GPS_PULSE_GPIO, KEY_DOWN, false,      "GPS timepulse"),
};

static struct gpio_keys_platform_data p7_gpio_keys_platform_data = {
	.buttons	= p7_gpio_keys,
	.nbuttons	= ARRAY_SIZE(p7_gpio_keys),
};

static struct platform_device p7_gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= 0,
	.dev = {
		.platform_data = &p7_gpio_keys_platform_data,
	},
};

/*************
 * AVI
 *************/
/* Nokia Galielo1 Sensor */
#define GALILEO_WIDTH      7728
#define GALILEO_HEIGHT     5368
#define GALILEO_PIXEL_SIZE    2
#define GALILEO_N_BUFFERS     4

#define P7_GALILEO_AVI_RAM_SIZE PAGE_ALIGN(GALILEO_WIDTH *      \
					   GALILEO_HEIGHT *     \
					   GALILEO_PIXEL_SIZE * \
					   GALILEO_N_BUFFERS)

static struct pinctrl_map sfx1b_maincam_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_VS),
	P7_INIT_PINMAP(P7_CAM_0_HS),
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
};

static struct pwm_device *sfx1b_maincam_tc358746a_pwm = NULL;

static int sfx1b_pwm_power_on(struct pwm_device **dev,
			      int pwm,
			      const char *label,
			      int period_ns)
{
	int ret = 0;

	if (*dev == NULL) {
		*dev = pwm_request(pwm, label);
		if (IS_ERR(*dev)) {
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

static int sfx1b_pwm_power_off(struct pwm_device **dev)
{
	pwm_disable(*dev);
	pwm_put(*dev);
	*dev = NULL;

	return 0;
}

#if defined(CONFIG_VIDEO_GALILEO2)
#include <media/galileo2.h>
#define MAINCAM_GALILEO_REFCLK      8928571
#define GALILEO_POWER_ON            GALILEO2_POWER_ON
#define GALILEO_POWER_OFF           GALILEO2_POWER_OFF
#else
#include <media/galileo1.h>
#define MAINCAM_GALILEO_REFCLK      9200000
#define GALILEO_POWER_ON            GALILEO1_POWER_ON
#define GALILEO_POWER_OFF           GALILEO1_POWER_OFF
#endif

#define GALILEO_SENSOR_I2C_ADDR     0x10
#define MAINCAM_GALILEO_I2C_BUS        1
#define MAINCAM_GALILEO_MIPI_LANES     2

static struct pwm_device *sfx1b_maincam_galileo_pwm = NULL;

static int sfx1b_maincam_galileo_power_on(void)
{
	int ret = sfx1b_pwm_power_on(&sfx1b_maincam_galileo_pwm,
				     4,
				     "maincam-galileo",
				     1000000000 / MAINCAM_GALILEO_REFCLK);

	/* Make sure the power is on */
	if (ret == 0)
		msleep(1);

	return ret;
}

static int sfx1b_maincam_galileo_power_off(void)
{
	return sfx1b_pwm_power_off(&sfx1b_maincam_galileo_pwm);
}

static int sfx1b_maincam_galileo_set_power(int on)
{
	if (on == GALILEO_POWER_ON)
		return sfx1b_maincam_galileo_power_on();
	else
		return sfx1b_maincam_galileo_power_off();
}

#if defined(CONFIG_VIDEO_GALILEO2)
static struct galileo2_platform_data sfx1b_maincam_galileo_pdata = {
#else
static struct galileo1_platform_data sfx1b_maincam_galileo_pdata = {
#endif
	.set_power = &sfx1b_maincam_galileo_set_power,
	.refclk    = MAINCAM_GALILEO_REFCLK,
	.lanes     = MAINCAM_GALILEO_MIPI_LANES,
};

static struct i2c_board_info sfx1b_maincam_galileo_i2c_device = {
#if defined(CONFIG_VIDEO_GALILEO2)
	I2C_BOARD_INFO("galileo2", GALILEO_SENSOR_I2C_ADDR),
#else
	I2C_BOARD_INFO("galileo1", GALILEO_SENSOR_I2C_ADDR),
#endif
	.platform_data = &sfx1b_maincam_galileo_pdata,
};

/* EEPROM M24C64 */
#include <linux/i2c/at24.h>
static struct at24_platform_data at24c64_pdata = {
	.byte_len	= SZ_64K / 8,
	.page_size	= 32,
	.flags		= AT24_FLAG_ADDR16 | AT24_FLAG_IRUGO,
};

static struct i2c_board_info at24c64_i2c_device = {
	I2C_BOARD_INFO("24c64", 0x50),
	.platform_data = &at24c64_pdata,
};

#include <media/as3636.h>

static struct as3636_platform_data as3636_pdata = {
	.set_power = NULL,
	.xenon_pulse_length = 255,
	.switch_selection = 900,
	.charge_max_voltage = 285,
	.dcdc_peak = 400,
	.auto_charge = 0,
};

static struct i2c_board_info as3636_i2c_device = {
	I2C_BOARD_INFO(AS3636_NAME, AS3636_I2C_ADDR),
	.platform_data = &as3636_pdata,
};

static struct avicam_subdevs sfx1b_maincam_galileo_subdevs[] = {
	{
		.board_info     = &sfx1b_maincam_galileo_i2c_device,
		.i2c_adapter_id = MAINCAM_GALILEO_I2C_BUS,
		.subdevs        = NULL,
	},
	{ NULL, 0 },
};

#include <media/tc358746a.h>

#define MAINCAM_TC358746A_NRST_GPIO       115
#if defined(CONFIG_VIDEO_GALILEO2)
#define MAINCAM_TC358746A_REFCLK      8928571
#else
#define MAINCAM_TC358746A_REFCLK      9200000
#endif
#define MAINCAM_TC358746A_I2C_BUS          20
#define MAINCAM_TC358746A_MIPI_LANES MAINCAM_GALILEO_MIPI_LANES

static int sfx1b_maincam_tc358746a_power_on(void)
{
	int ret = 0;

	ret = sfx1b_pwm_power_on(&sfx1b_maincam_tc358746a_pwm,
				 11,
				 "maincam-tc358746a",
				 1000000000 / MAINCAM_TC358746A_REFCLK);

	if (ret)
		return ret;

	gpio_set_value_cansleep(
			MAINCAM_TC358746A_NRST_GPIO, TC358746A_POWER_ON);

	return 0;
}

static int sfx1b_maincam_tc358746a_power_off(void)
{
	sfx1b_pwm_power_off(&sfx1b_maincam_tc358746a_pwm);

	gpio_set_value_cansleep(
			MAINCAM_TC358746A_NRST_GPIO, TC358746A_POWER_OFF);
	return 0;
}

static int sfx1b_maincam_tc358746a_set_power(int on)
{
	if (on == TC358746A_POWER_ON)
		return sfx1b_maincam_tc358746a_power_on();
	else
		return sfx1b_maincam_tc358746a_power_off();
}

static struct tc358746a_platform_data sfx1b_maincam_tc358746a_pdata = {
	.set_power = &sfx1b_maincam_tc358746a_set_power,
	.refclk    = MAINCAM_TC358746A_REFCLK,
	.lanes     = MAINCAM_TC358746A_MIPI_LANES,
#if defined(CONFIG_VIDEO_GALILEO2)
	.calibration_delay_ms = 50,
	.phytimdly            = 27,
#else
	.phytimdly = 9,
#endif
};

static struct i2c_board_info sfx1b_maincam_tc358746a_i2c_device = {
	I2C_BOARD_INFO("tc358746a", TC358746A_I2C_ADDR),
	.platform_data = &sfx1b_maincam_tc358746a_pdata,
};

static struct avicam_subdevs sfx1b_maincam_tc358746a_subdevs[] = {
	{
		.board_info     = &sfx1b_maincam_tc358746a_i2c_device,
		.i2c_adapter_id = MAINCAM_TC358746A_I2C_BUS,
		.subdevs        = sfx1b_maincam_galileo_subdevs,
	},
	{
		.board_info     = &as3636_i2c_device,
		.i2c_adapter_id = 24,
		.subdevs        = NULL,
	},
	{ NULL, 0 },
};

/* Galileo1 sensor follows the SMIA specification, hence the first 4 lines
 * contains its registers values. So we crop it directly here.
 */
static void sfx1b_galileo_get_timings(struct avi_capture_timings  *t,
				      struct avi_cam_measure_regs *m)
{
	t->ht.hactive_on  = 0;
	t->ht.hactive_off = m->hsync_off;
	t->vt.vactive_on  = 5;
	t->vt.vactive_off = m->vsync_voff + 5;
}

static struct avicam_platform_data sfx1b_maincam_pdata = {
	.cam_cap	    = AVI_CAP_CAM_0,
	.enable_stats       = 1,
	.interface          = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 0,
		.ihs        = 0,
		.ipc        = 0,
		.psync_en   = 1,
		.psync_rf   = 1,
	},
	.bus_width	    = 10,
	.subdevs	    = sfx1b_maincam_tc358746a_subdevs,
	.measure_to_timings = &sfx1b_galileo_get_timings,
	.vb2_cache_flags    = VB2_CACHE_DMA_CONTIG | VB2_CACHE_WRITETHROUGH,
};

static u64 sfx1b_maincam_dma_mask = DMA_BIT_MASK(32);

static struct platform_device sfx1b_maincam_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &sfx1b_maincam_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct platform_device sfx1b_multicapture = {
	.name = "avi_multicapture",
	.id   = 1,
	.dev            = {
		.dma_mask           = &sfx1b_maincam_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct pinctrl_map sfx1b_cam12345_pins[] __initdata = {
	/* CAM_1[7:0] itu656, pad_select = 0*/ 
	P7_INIT_PINMAP(P7_CAM_1_DATA00),
	P7_INIT_PINMAP(P7_CAM_1_DATA01),
	P7_INIT_PINMAP(P7_CAM_1_DATA02),
	P7_INIT_PINMAP(P7_CAM_1_DATA03),
	P7_INIT_PINMAP(P7_CAM_1_DATA04),
	P7_INIT_PINMAP(P7_CAM_1_DATA05),
	P7_INIT_PINMAP(P7_CAM_1_DATA06),
	P7_INIT_PINMAP(P7_CAM_1_DATA07),
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	/* CAM_2[7:0] itu656, pad_select = 0 */
	P7_INIT_PINMAP(P7_CAM_2_CLK),
	P7_INIT_PINMAP(P7_CAM_2_DATA00),
	P7_INIT_PINMAP(P7_CAM_2_DATA01),
	P7_INIT_PINMAP(P7_CAM_2_DATA02),
	P7_INIT_PINMAP(P7_CAM_2_DATA03),
	P7_INIT_PINMAP(P7_CAM_2_DATA04),
	P7_INIT_PINMAP(P7_CAM_2_DATA05),
	P7_INIT_PINMAP(P7_CAM_2_DATA06),
	P7_INIT_PINMAP(P7_CAM_2_DATA07),
	/* CAM_3[7:0] itu656, pad_select = 0 */
	P7_INIT_PINMAP(P7_CAM_3_CLK),
	P7_INIT_PINMAP(P7_CAM_3_DATA00),
	P7_INIT_PINMAP(P7_CAM_3_DATA01),
	P7_INIT_PINMAP(P7_CAM_3_DATA02),
	P7_INIT_PINMAP(P7_CAM_3_DATA03),
	P7_INIT_PINMAP(P7_CAM_3_DATA04),
	P7_INIT_PINMAP(P7_CAM_3_DATA05),
	P7_INIT_PINMAP(P7_CAM_3_DATA06),
	P7_INIT_PINMAP(P7_CAM_3_DATA07),
	/* CAM_4[7:0] itu656, pad_select = 0 */
	P7_INIT_PINMAP(P7_CAM_4_CLK),
	P7_INIT_PINMAP(P7_CAM_4_DATA00),
	P7_INIT_PINMAP(P7_CAM_4_DATA01),
	P7_INIT_PINMAP(P7_CAM_4_DATA02),
	P7_INIT_PINMAP(P7_CAM_4_DATA03),
	P7_INIT_PINMAP(P7_CAM_4_DATA04),
	P7_INIT_PINMAP(P7_CAM_4_DATA05),
	P7_INIT_PINMAP(P7_CAM_4_DATA06),
	P7_INIT_PINMAP(P7_CAM_4_DATA07),
	/* CAM_5[7:0] itu656, pad_select = 0 */
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
};

static const struct v4l2_mbus_framefmt sfx1b_ov_cam_format = {
	.code	      = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_SMPTE170M,
	.field	    = V4L2_FIELD_NONE,
	.width	    = 640,
	.height	    = 480,
};

/* Custom measure-to-timings conversion for omnivision camera in 656 timings */
static void sfx1b_ov_get_timings(struct avi_capture_timings  *t,
				 struct avi_cam_measure_regs *m)
{
	t->ht.hactive_on  = 2;
	t->ht.hactive_off = m->hactive_off - 2;
	t->vt.vactive_on  = m->vactive_on - 1;
	t->vt.vactive_off = m->vsync_voff - 1;

	/* Due to the way the hardware counts the lines we have to add one to
	 * the value if the hsync and vsync don't rise on the same cycle */
	if (m->vsync_hon != 0) {
		t->vt.vactive_on++;
		t->vt.vactive_off++;
	}
}

#define OV7740_REFCLK 6000000
#define TC358746A_P7_TEGRA_REFCLK 26000000

static struct pwm_device *sfx1b_ov7740_pwm[6] = { NULL };

static struct ov7740_platform_data ov7740_pdata = {
	.set_power = NULL,
};

static struct i2c_board_info sfx1b_ov7740_i2c_device = {
	I2C_BOARD_INFO("ov7740", 0x21),
	.platform_data = &ov7740_pdata,
};

static struct avicam_subdevs sfx1b_ov_cam_subdevs[][2] = {
	{
		{
			.board_info     = &sfx1b_ov7740_i2c_device,
			.i2c_adapter_id = 20,
			.subdevs        = NULL,
		},
		{ NULL, 0 },
	},
	{
		{
			.board_info     = &sfx1b_ov7740_i2c_device,
			.i2c_adapter_id = 21,
			.subdevs        = NULL,
		},
		{ NULL, 0 },
	},
	{
		{
			.board_info     = &sfx1b_ov7740_i2c_device,
			.i2c_adapter_id = 22,
			.subdevs        = NULL,
		},
		{ NULL, 0 },
	},
	{
		{
			.board_info     = &sfx1b_ov7740_i2c_device,
			.i2c_adapter_id = 23,
			.subdevs        = NULL,
		},
		{ NULL, 0 },
	},
	{
		{
			.board_info     = &sfx1b_ov7740_i2c_device,
			.i2c_adapter_id = 24,
			.subdevs        = NULL,
		},
		{ NULL, 0 },
	},
};

static struct avicam_platform_data sfx1b_ov_cams_pdata[] = {
	{
		.cam_cap	    = AVI_CAP_CAM_1,
		.interface          = {
			.itu656	    = 1,
			.pad_select = 0,
			.ivs        = 1,
			.ihs        = 1,
			.ipc        = 0,
			.psync_en   = 0,
			.format_control = AVI_FORMAT_CONTROL_YUYV_2X8,
		},
		.bus_width	    = 8,
		.subdevs	    = sfx1b_ov_cam_subdevs[0],
		.dummy_driver_info  = NULL,
		.measure_to_timings = &sfx1b_ov_get_timings,
	},
	{
		.cam_cap	    = AVI_CAP_CAM_2,
		.interface          = {
			.itu656	    = 1,
			.pad_select = 0,
			.ivs        = 1,
			.ihs        = 1,
			.ipc        = 0,
			.psync_en   = 0,
			.format_control = AVI_FORMAT_CONTROL_YUYV_2X8,
		},
		.bus_width	    = 8,
		.subdevs	    = sfx1b_ov_cam_subdevs[1],
		.dummy_driver_info  = NULL,
		.measure_to_timings = &sfx1b_ov_get_timings,
	},
	{
		.cam_cap	    = AVI_CAP_CAM_3,
		.interface          = {
			.itu656	    = 1,
			.pad_select = 0,
			.ivs        = 1,
			.ihs        = 1,
			.ipc        = 0,
			.psync_en   = 0,
			.format_control = AVI_FORMAT_CONTROL_YUYV_2X8,
		},
		.bus_width	    = 8,
		.subdevs	    = sfx1b_ov_cam_subdevs[2],
		.dummy_driver_info  = NULL,
		.measure_to_timings = &sfx1b_ov_get_timings,
	},
	{
		.cam_cap	    = AVI_CAP_CAM_4,
		.interface          = {
			.itu656	    = 1,
			.pad_select = 0,
			.ivs        = 1,
			.ihs        = 1,
			.ipc        = 0,
			.psync_en   = 0,
			.format_control = AVI_FORMAT_CONTROL_YUYV_2X8,
		},
		.bus_width	    = 8,
		.subdevs	    = sfx1b_ov_cam_subdevs[3],
		.dummy_driver_info  = NULL,
		.measure_to_timings = &sfx1b_ov_get_timings,
	},
	{
		.cam_cap	    = AVI_CAP_CAM_5,
		.interface          = {
			.itu656	    = 1,
			.pad_select = 0,
			.ivs        = 1,
			.ihs        = 1,
			.ipc        = 0,
			.psync_en   = 0,
			.format_control = AVI_FORMAT_CONTROL_YUYV_2X8,
		},
		.bus_width	    = 8,
		.subdevs	    = sfx1b_ov_cam_subdevs[4],
		.dummy_driver_info  = NULL,
		.measure_to_timings = &sfx1b_ov_get_timings,
	},
};

static struct avimulti_platform_data sfx1b_multicapture_pdata  = {
	.cam_interfaces    = sfx1b_ov_cams_pdata,
	.nb_full_framerate = 0,
	.nb_cameras        = ARRAY_SIZE(sfx1b_ov_cams_pdata),
	.width             = 640,
	.height            = 480,
	.enable_metadata   = 1,
	.timeout           = 3 * HZ,
	.vb2_cache_flags   = VB2_CACHE_DMA_CONTIG | VB2_CACHE_WRITETHROUGH,
};


/*********************
 * Framebuffer config
 *********************/

static unsigned long sfx1b_fb0_drivecfg[] = {
	P7CTL_DRV_CFG(0),
};

static struct pinctrl_map sfx1b_fb0_pins[] __initdata = {
	/* LCD0[23:16] itu656, pad_select = 2 */
	P7_INIT_PINMAP(P7_LCD_0_CLK),
	P7_INIT_PINCFG(P7_LCD_0_CLK, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA16),
	P7_INIT_PINCFG(P7_LCD_0_DATA16, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA17),
	P7_INIT_PINCFG(P7_LCD_0_DATA17, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA18),
	P7_INIT_PINCFG(P7_LCD_0_DATA18, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA19),
	P7_INIT_PINCFG(P7_LCD_0_DATA19, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA20),
	P7_INIT_PINCFG(P7_LCD_0_DATA20, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA21),
	P7_INIT_PINCFG(P7_LCD_0_DATA21, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA22),
	P7_INIT_PINCFG(P7_LCD_0_DATA22, sfx1b_fb0_drivecfg),
	P7_INIT_PINMAP(P7_LCD_0_DATA23),
	P7_INIT_PINCFG(P7_LCD_0_DATA23, sfx1b_fb0_drivecfg),
};

#define SFX1B_VOC_SIZE (640 * 480 * 2 * 4 * 5 + 640 * 2)

static struct avifb_overlay sfx1b_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = SFX1B_VOC_SIZE,
		.zorder		 = 0,
	},
};

static struct avifb_platform_data sfx1b_avifb0_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_YUYV_2X8,
	.lcd_default_videomode = &sfx1_tegra_640x2401p32_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 1,
    		.ipc           = 1,
    		.pad_select    = 2,
	}},
	.output_cspace         = AVI_BT601_CSPACE,
	.caps                 = AVI_CAP_LCD_0,
	.background    = 0x00ff00,
	.overlays	     = sfx1b_avi_lcd0_overlays,
	.overlay_nr	   = ARRAY_SIZE(sfx1b_avi_lcd0_overlays),
};

static struct avi_voc_plat_data sfx1b_avi_voc_param0 = {
	.display = "lcd.1",
};

static u64 sfx1b_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device sfx1b_avifb0_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
	/* Todo: try to set a narrower mask */
	.dma_mask           = &sfx1b_avifb0_dma_mask,
	.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};
/*****************************
 * P7MU Power Management Unit
 *****************************/
static unsigned long sfx1b_p7mu_reboot_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN)  | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map sfx1b_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
	P7_INIT_PINCFG(P7_REBOOT_P7MU, sfx1b_p7mu_reboot_pinconf),
};

static struct p7mu_plat_data sfx1b_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(73),
	.int_32k    = false,    /* External 32kHz clock. */
	.int_32m    = false,    /* External 48mHz clock. */
};

/*************
 * I2C
 *************/
#include <i2c/muxes/p7-i2cmux.h>
#include "i2cm.h"

/* The alternate buses. There will be one additional "default" bus.*/
static char const* const  sfx1b_i2cm_mux_names[] = {
	"mux-a", "mux-b", "mux-c", "mux-d", "mux-e", "mux-all"
};

static unsigned long sfx1b_i2cm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

/* I2CM3 pins */
static struct pinctrl_map p7brd_i2cm3_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKa),
	P7_INIT_PINCFG(P7_I2C_2_CLKa, sfx1b_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATa),
	P7_INIT_PINCFG(P7_I2C_2_DATa, sfx1b_i2cm_pinconf)
};

/* I2CM4 pins */
static struct pinctrl_map p7brd_i2cm4_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKb),
	P7_INIT_PINCFG(P7_I2C_2_CLKb, sfx1b_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATb),
	P7_INIT_PINCFG(P7_I2C_2_DATb, sfx1b_i2cm_pinconf)
};

/* I2CM5 pins */
static struct pinctrl_map p7brd_i2cm5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKc),
	P7_INIT_PINCFG(P7_I2C_2_CLKc, sfx1b_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATc),
	P7_INIT_PINCFG(P7_I2C_2_DATc, sfx1b_i2cm_pinconf)
};

/* I2CM6 pins */
static struct pinctrl_map p7brd_i2cm6_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKd),
	P7_INIT_PINCFG(P7_I2C_2_CLKd, sfx1b_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATd),
	P7_INIT_PINCFG(P7_I2C_2_DATd, sfx1b_i2cm_pinconf)
};

/* I2CM7 pins */
static struct pinctrl_map p7brd_i2cm7_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKe),
	P7_INIT_PINCFG(P7_I2C_2_CLKe, sfx1b_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATe),
	P7_INIT_PINCFG(P7_I2C_2_DATe, sfx1b_i2cm_pinconf)
};

/*
 * Pseudo port for talking to all ports
 */
static struct pinctrl_map p7brd_i2cmall_pins[] __initdata = {

	P7_INIT_PINMAP(P7_I2C_2_CLKa),
	P7_INIT_PINMAP(P7_I2C_2_DATa),
	P7_INIT_PINMAP(P7_I2C_2_CLKb),
	P7_INIT_PINMAP(P7_I2C_2_DATb),
	P7_INIT_PINMAP(P7_I2C_2_CLKc),
	P7_INIT_PINMAP(P7_I2C_2_DATc),
	P7_INIT_PINMAP(P7_I2C_2_CLKd),
	P7_INIT_PINMAP(P7_I2C_2_DATd),
	P7_INIT_PINMAP(P7_I2C_2_CLKe),
	P7_INIT_PINMAP(P7_I2C_2_DATe),
};

static struct p7i2cmux_pins  p7brd_i2cm_mux_pins[] __initdata = {
	{
		.pinmap = p7brd_i2cm3_pins,
		.sz     = ARRAY_SIZE(p7brd_i2cm3_pins)
	},
	{
		.pinmap = p7brd_i2cm4_pins,
		.sz     = ARRAY_SIZE(p7brd_i2cm4_pins)
	},
	{
		.pinmap = p7brd_i2cm5_pins,
		.sz     = ARRAY_SIZE(p7brd_i2cm5_pins)
	},
	{
		.pinmap = p7brd_i2cm6_pins,
		.sz     = ARRAY_SIZE(p7brd_i2cm6_pins)
	},
	{
		.pinmap = p7brd_i2cm7_pins,
		.sz     = ARRAY_SIZE(p7brd_i2cm7_pins)
	},
	{
		.pinmap = p7brd_i2cmall_pins,
		.sz     = ARRAY_SIZE(p7brd_i2cmall_pins)
	}
};

static struct p7i2cmux_plat_data  p7brd_i2cm_mux_pdata = {
	.channel_names = sfx1b_i2cm_mux_names,
	.nr_channels   = ARRAY_SIZE(sfx1b_i2cm_mux_names)
};


/*************
 * SPI
 *************/
#define SFX1B_SPI_SLAVE_P7MU    3

static struct pinctrl_map sfx1b_spi_slave_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_13b), /* SS */
	P7_INIT_PINMAP(P7_SPI_12b), /* CLK */
	P7_INIT_PINMAP(P7_SPI_14b), /* MOSI */
};

static struct p7spi_swb const sfx1b_spi_slave_p7mu_swb[] = {
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
};

static struct p7spis_ctrl_data sfx1b_spi_slave_p7mu_cdata = {
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
	},
	.circ_buf_period = 128,
	.periods         = 9,
};

static P7_DECLARE_SPIS_MASTER(sfx1b_spi_master_p7mu_info,
		"p7mu-adc",
		NULL,
		&sfx1b_spi_slave_p7mu_cdata,
		10 * 1000 * 1000,
		SPI_MODE_0|SPI_LSB_FIRST);


static void __init sfx1_init_spi_p7mu(void)
{
	p7_init_spis(SFX1B_SPI_SLAVE_P7MU,
			sfx1b_spi_slave_p7mu_pins,
			ARRAY_SIZE(sfx1b_spi_slave_p7mu_pins),
			sfx1b_spi_slave_p7mu_swb);

	if (p7_init_spis_master(SFX1B_SPI_SLAVE_P7MU,
				&sfx1b_spi_master_p7mu_info))
		pr_err(SFX1B_BRD_NAME ": failed to initialize SPI slave.\n");
}

static unsigned long sfx1b_spi_slave_flir_pinconf[] = {
       P7CTL_DRV_CFG(2),      /* Drive strength to 7 (CEM), lower is no more functionnal */
};

static struct pinctrl_map sfx1b_spi_slave_flir_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_00), /* CLK */
	P7_INIT_PINCFG(P7_SPI_00, sfx1b_spi_slave_flir_pinconf),
	P7_INIT_PINMAP(P7_SPI_02), /* MISO */
	P7_INIT_PINMAP(P7_SPI_03), /* SS */
};

static struct p7spi_swb const sfx1b_spi_master_flir_swb[] = {
	P7SPI_INIT_SWB( 0,  P7_SWB_DIR_OUT,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB( 2,  P7_SWB_DIR_IN, P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB( 3,  P7_SWB_DIR_OUT,  P7_SWB_SPI_SS),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data sfx1b_spi_master_flir_cdata = {
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
};

#include <media/video/lepton.h>

#define LEPTON_NRST_GPIO 84

static struct lepton_i2c_platform_data flir_i2c_pdata = {
	.gpio_pwr = -1,
	.gpio_rst = LEPTON_NRST_GPIO,
};

static struct lepton_platform_data flir_pdata = {
	.board_info = {
		I2C_BOARD_INFO("lepton-ctrl", 0x2A),
		.platform_data = &flir_i2c_pdata,
	},
	.i2c_adapter_nr = 1,
};

static P7_DECLARE_SPIM_SLAVE(sfx1b_flir_info,
		"lepton",
		&flir_pdata,
		&sfx1b_spi_master_flir_cdata,
		10 * 1000 * 1000,
		SPI_MODE_3);

static void __init __maybe_unused sfx1b_i2c_int(void)
{
	int err, fail = 3;

	err = p7_assign_named_pins("p7-i2cm.3",NULL,
                                p7brd_i2cm3_pins,ARRAY_SIZE(p7brd_i2cm3_pins));

	if(err)
		goto finish;

	fail++;

	err = p7_assign_named_pins("p7-i2cm.4",NULL,
                                p7brd_i2cm4_pins,ARRAY_SIZE(p7brd_i2cm4_pins));

	if(err)
		goto finish;

	fail++;


	err = p7_assign_named_pins("p7-i2cm.5", NULL,
                                p7brd_i2cm5_pins, ARRAY_SIZE(p7brd_i2cm5_pins));

	if(err)
		goto finish;

	fail++;


	err = p7_assign_named_pins("p7-i2cm.6",NULL,
                                p7brd_i2cm6_pins,ARRAY_SIZE(p7brd_i2cm6_pins));

	if(err)
		goto finish;

	fail++;


	err = p7_assign_named_pins("p7-i2cm.7",NULL,
                                p7brd_i2cm7_pins,ARRAY_SIZE(p7brd_i2cm7_pins));

	if(err)
		goto finish;

finish:
	if(err){
		pr_err("%s: p7-i2cm.%d init failed\n",SFX1B_BRD_NAME, fail);
	}
}

static void __init sfx1b_init_mach(void)
{
	p7_init_mach();

	/* GPIO init */
	p7_init_gpio(sfx1b_irq_gpios,
	             ARRAY_SIZE(sfx1b_irq_gpios));

	/* Remove pull-down from STROBE pin */
	p7_config_pin(SFX1B_STROBE_GPIO, P7CTL_PUD_CFG(HIGHZ));

	p7_init_dev(&p7_gpio_keys_device, 
			&p7_gpio_keys_platform_data, NULL, 0);

	/* P7MU init */
	p7brd_init_i2cm(0, 100);

	p7_gpio_interrupt_register(sfx1b_p7mu_pdata.gpio);
	p7_init_p7mu(0,
			&sfx1b_p7mu_pdata,
			sfx1b_p7mu_pins,
			ARRAY_SIZE(sfx1b_p7mu_pins));

	p7brd_export_gpio(MAINCAM_TC358746A_NRST_GPIO,
			GPIOF_OUT_INIT_LOW, "maincam-tc358746a-nrst");
	/*
	p7brd_export_gpio(LEPTON_NRST_GPIO,
			GPIOF_OUT_INIT_HIGH, "lepton-nrst");
	*/

	/* P7MU/ADC */
	sfx1_init_spi_p7mu();
        p7_init_temperature();

	/* NAND init */
	p7brd_init_nand(0);

	/* UART init */
	p7brd_init_uart(0,1);

	/* PWM init */ // TEST
	p7_init_p7pwm(&sfx1b_pwm_pdata,
	              sfx1b_pwm_pins,
		      ARRAY_SIZE(sfx1b_pwm_pins));

	/* USB init */
	//if (parrot_force_usb_device)
		p7brd_init_udc(0, -1);
	//else
	//	p7brd_init_hcd(0, -1);

	/* Ethernet init */
	/* Reset managed by APALIS module */
	p7_init_ether(PHY_IFACE_RGMII,-1, P7CTL_DRV_CFG(4));

	/* I2C init (remaining) */
	p7brd_init_i2cm(1, 400);
	//p7brd_init_i2cm(2, 400);
	p7brd_init_i2cm_muxed(2,
			      100,
			      NULL,
			      0,
			      &p7brd_i2cm_mux_pdata,
			      p7brd_i2cm_mux_pins);

	/* FLIR */
	p7_init_spim(2, sfx1b_spi_slave_flir_pins,
			ARRAY_SIZE(sfx1b_spi_slave_flir_pins),
			sfx1b_spi_master_flir_swb);

	p7_init_spim_slave(2, &sfx1b_flir_info);

	/* AVI init */
	p7_init_avicam(&sfx1b_maincam_dev,
		       &sfx1b_maincam_pdata,
		       sfx1b_maincam_pins,
		       ARRAY_SIZE(sfx1b_maincam_pins));

	/* EEPROM */
	p7_init_i2cm_slave(1, &at24c64_i2c_device, NULL, 0);

	/* PWMs for the 5 CVCams & P7->Tegra bridge */
	sfx1b_pwm_power_on(&sfx1b_ov7740_pwm[0],
	                   5,
	                   "ov7740-right",
	                   1000000000 / OV7740_REFCLK);
	sfx1b_pwm_power_on(&sfx1b_ov7740_pwm[1],
	                   6,
	                   "ov7740-left",
	                   1000000000 / OV7740_REFCLK);
	sfx1b_pwm_power_on(&sfx1b_ov7740_pwm[2],
	                   7,
	                   "ov7740-bottom",
	                   1000000000 / OV7740_REFCLK);
	sfx1b_pwm_power_on(&sfx1b_ov7740_pwm[3],
	                   8,
	                   "ov7740-back",
	                   1000000000 / OV7740_REFCLK);
	sfx1b_pwm_power_on(&sfx1b_ov7740_pwm[4],
	                   9,
	                   "ov7740-front",
	                   1000000000 / OV7740_REFCLK);
	sfx1b_pwm_power_on(&sfx1b_ov7740_pwm[5],
	                   10,
	                   "tc358746a-p7-tegra",
	                   1000000000 / TC358746A_P7_TEGRA_REFCLK);

	p7_init_dev(&sfx1b_multicapture,
		    &sfx1b_multicapture_pdata,
		    sfx1b_cam12345_pins,
		    ARRAY_SIZE(sfx1b_cam12345_pins));

	p7_init_avifb(&sfx1b_avifb0_dev, &sfx1b_avifb0_pdata,
	              sfx1b_fb0_pins, ARRAY_SIZE(sfx1b_fb0_pins));

	p7_init_avi_voc(0, &sfx1b_avi_voc_param0);

	/* IMU */
	drone_common_init_inv_mpu6050(24,
				      P7_GPIO_NR(159),
				      5, -1);

	p7_init_venc();
}

static void __init sfx1b_reserve_mem(void)
{
	size_t	ov_cam_sz;

#define SFX1_HX280_SIZE (CONFIG_ARCH_PARROT7_SFX1_HX280_SIZE << 20)
	p7_reserve_vencmem(SFX1_HX280_SIZE);

	p7_reserve_avicammem(&sfx1b_maincam_dev,
			     P7_GALILEO_AVI_RAM_SIZE);

	/* Reserve enough memory for 8 buffers at 16bpp */
	ov_cam_sz = sfx1b_ov_cam_format.width *
		sfx1b_ov_cam_format.height *
		8 * 2;

	p7_reserve_avicammem(&sfx1b_multicapture, ov_cam_sz);

	p7_reserve_avifbmem(&sfx1b_avifb0_dev,
	                    sfx1b_avi_lcd0_overlays,
	                    ARRAY_SIZE(sfx1b_avi_lcd0_overlays));

	p7_reserve_avi_voc_mem(0, SFX1B_VOC_SIZE);

        p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);

	p7_reserve_dmamem();
}


P7_MACHINE_START(PARROT_SFX1B, "Sfx1b board")
	.reserve        = &sfx1b_reserve_mem,
	.init_machine   = &sfx1b_init_mach,
P7_MACHINE_END
