/**
 * linux/arch/arm/mach-parrot7/galileo2-board.c - P7Dev ISP GALILEO2 daughter board
 *                                            implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    05-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/pwm.h>
#include <spi/p7-spi.h>
#include <i2c/muxes/p7-i2cmux.h>
#include <i2c/muxes/p7-i2cmux.h>
#include <media/tc358746a.h>
#include <media/galileo2.h>
#include <linux/pwm.h>
#include "common.h"
#include "pinctrl.h"
#include "spi.h"
#include "board.h"
#include "board-common.h"
#include "lcd-monspecs.h"
#include "avi.h"
#include "backlight.h"
#include "i2cm.h"
#include "p7_pwm.h"


static unsigned long galileo2_i2c_pinconf[] = {
       P7CTL_PUD_CFG(UP)
};


/* The alternate buses. There will be one additional "default" bus.*/
static char const* const galileo2db_i2cm2_mux_names[] = {
	"mux-a", "mux-b", "mux-c", "mux-d", "mux-e"
};

static struct pinctrl_map galileo2db_i2cm2_muxa_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKa),
	P7_INIT_PINCFG(P7_I2C_2_CLKa, galileo2_i2c_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATa),
	P7_INIT_PINCFG(P7_I2C_2_DATa, galileo2_i2c_pinconf),
};

static struct pinctrl_map galileo2db_i2cm2_muxb_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKb),
	P7_INIT_PINCFG(P7_I2C_2_CLKb, galileo2_i2c_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATb),
	P7_INIT_PINCFG(P7_I2C_2_DATb, galileo2_i2c_pinconf),
};

static struct pinctrl_map galileo2db_i2cm2_muxc_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKc),
	P7_INIT_PINCFG(P7_I2C_2_CLKc, galileo2_i2c_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATc),
	P7_INIT_PINCFG(P7_I2C_2_DATc, galileo2_i2c_pinconf),
};

static struct pinctrl_map galileo2db_i2cm2_muxd_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKd),
	P7_INIT_PINCFG(P7_I2C_2_CLKd, galileo2_i2c_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATd),
	P7_INIT_PINCFG(P7_I2C_2_DATd, galileo2_i2c_pinconf),
};

static struct pinctrl_map galileo2db_i2cm2_muxe_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKe),
	P7_INIT_PINCFG(P7_I2C_2_CLKe, galileo2_i2c_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATe),
	P7_INIT_PINCFG(P7_I2C_2_DATe, galileo2_i2c_pinconf),
};

static struct p7i2cmux_pins galileo2db_i2cm2_mux_pins[] __initdata = {
	{
		.pinmap = galileo2db_i2cm2_muxa_pins,
		.sz     = ARRAY_SIZE(galileo2db_i2cm2_muxa_pins)
	},
	{
		.pinmap = galileo2db_i2cm2_muxb_pins,
		.sz     = ARRAY_SIZE(galileo2db_i2cm2_muxb_pins)
	},
	{
		/* TOSH_I2C */
		.pinmap = galileo2db_i2cm2_muxc_pins,
		.sz     = ARRAY_SIZE(galileo2db_i2cm2_muxc_pins)
	},
	{
		.pinmap = galileo2db_i2cm2_muxd_pins,
		.sz     = ARRAY_SIZE(galileo2db_i2cm2_muxd_pins)
	},
	{
		.pinmap = galileo2db_i2cm2_muxe_pins,
		.sz     = ARRAY_SIZE(galileo2db_i2cm2_muxe_pins)
	},
};

static struct p7i2cmux_plat_data galileo2db_i2cm2_mux_pdata = {
	.channel_names = galileo2db_i2cm2_mux_names,
	.nr_channels   = ARRAY_SIZE(galileo2db_i2cm2_mux_names)
};

static struct p7pwm_conf galileo2_conf_cam_mclk = {
	/* Precision for mt9f002 */
	.period_precision = 5,
	/* Not used in clock mode */
	.duty_precision = 0,
	.mode = P7PWM_MODE_CLOCK,
};

static struct pinctrl_map galileo2db_pwm_pins[] __initdata = {
	/* Galileo 2 mclk */
	P7_INIT_PINMAP(P7_PWM_05),
	/* Toshiba bridge mclk */
	P7_INIT_PINMAP(P7_PWM_11),
};

static struct p7pwm_pdata galileo2db_pwm_pdata = {
	.conf = {
		[5] = &galileo2_conf_cam_mclk,
		[11] = &galileo2_conf_cam_mclk,
	}
};

#include <media/tc358746a.h>

static struct pinctrl_map galileo2_avicam_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_HS),
	P7_INIT_PINMAP(P7_CAM_0_VS),
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
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
};

/* Configure a PWM to output a clock signal with a frequency given in
 * megaherz */
static struct pwm_device *galileo2_pwm_clock(int pwm,
					     const char *label,
					     unsigned freq_khz)
{
	struct pwm_device *pwm_dev;
	unsigned period_ns = (1000000 + freq_khz / 2) / freq_khz;
	int ret;

	pwm_dev = pwm_request(pwm, label);
	if (IS_ERR(pwm_dev)) {
		return pwm_dev;
	}

	ret = pwm_config(pwm_dev, 0, period_ns);
	if (ret) {
		goto free;
	}

	ret = pwm_enable(pwm_dev);
	if (ret) {
		goto free;
	}

	return pwm_dev;

 free:
	pwm_free(pwm_dev);

	return ERR_PTR(ret);
}

static void galileo2_pwm_free(struct pwm_device *pwm_dev)
{
	if (!IS_ERR_OR_NULL(pwm_dev)) {
		pwm_disable(pwm_dev);
		pwm_free(pwm_dev);
	}
}

#define GALILEO2_TC358746A_MCLK_KHZ 9200
static struct pwm_device *galileo2_tc358746a_pwm = NULL;

static int galileo2_tc358746a_set_power(int on)
{
	if (on) {
		galileo2_tc358746a_pwm =
			galileo2_pwm_clock(P7_PWM_NR(11),
					   "bridge mclk",
					   GALILEO2_TC358746A_MCLK_KHZ);
	} else {
		galileo2_pwm_free(galileo2_tc358746a_pwm);
	}

	if (IS_ERR(galileo2_tc358746a_pwm)) {
		return PTR_ERR(galileo2_tc358746a_pwm);
	}

	gpio_set_value_cansleep(P7_GPIO_NR(132), !!on);

	return 0;
}

#define GALILEO2_CAM_MCLK_KHZ 10000
static struct pwm_device *galileo2_cam_pwm = NULL;

static int galileo2_cam_set_power(int on)
{
	if (on) {
		galileo2_cam_pwm =
			galileo2_pwm_clock(P7_PWM_NR(5),
					   "galileo2 mclk",
					   GALILEO2_CAM_MCLK_KHZ);
	} else {
		galileo2_pwm_free(galileo2_cam_pwm);
	}

	if (IS_ERR(galileo2_cam_pwm)) {
		return PTR_ERR(galileo2_cam_pwm);
	}

	msleep(10);

	return 0;
}

struct galileo2_platform_data galileo2_cam_pdata = {
	.set_power = galileo2_cam_set_power,
	.refclk = GALILEO2_CAM_MCLK_KHZ * 1000,
	.lanes = 4,
};

static struct i2c_board_info galileo2_cam_i2c_device = {
	I2C_BOARD_INFO("galileo2", 0x10),
	.platform_data = &galileo2_cam_pdata,
};

static struct avicam_subdevs galileo2_cam_subdevs[] = {
	{
		.board_info     = &galileo2_cam_i2c_device,
		.i2c_adapter_id = 1,
		.subdevs        = NULL,
	},
	{ NULL, 0 },
};

static struct tc358746a_platform_data galileo2_tc358746a_subdev_pdata = {
	.set_power            = &galileo2_tc358746a_set_power,
	.refclk               = GALILEO2_TC358746A_MCLK_KHZ * 1000,
	/* Do we need that? */
	/*.force_subdev_pixcode = V4L2_MBUS_FMT_SBGGR10_1X10,*/
	.lanes                = 4,
	.calibration_delay_ms = 500,
	.phytimdly            = 39,
};

static struct i2c_board_info galileo2_tc358746a_i2c_device = {
	I2C_BOARD_INFO("tc358746a", 0x7),
	.platform_data = &galileo2_tc358746a_subdev_pdata,
};

static struct avicam_subdevs galileo2_tc358746a_subdevs[] = {
	{
		.board_info     = &galileo2_tc358746a_i2c_device,
		.i2c_adapter_id = 22,
		.subdevs = galileo2_cam_subdevs,
	},
	{ NULL, 0 },
};

/* Galileo2 sensor follows the SMIA specification, hence the first 4 lines
 * contains its registers values. So we crop it directly here.
 */
static void galileo2_get_timings(struct avi_capture_timings  *t,
				 struct avi_cam_measure_regs *m)
{
	t->ht.hactive_on  = 0;
	t->ht.hactive_off = m->hsync_off;
	t->vt.vactive_on  = 5;
	t->vt.vactive_off = m->vsync_voff + 5;
}


static struct avicam_platform_data galileo2_avicam_pdata = {
	.cam_cap	    = AVI_CAP_CAM_0,
	.interface          = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 0,
		.ihs        = 0,
		.ipc        = 0,
		.psync_en   = 1,
		.psync_rf   = 1,
		.ror_lsb    = 0,
	},
	.bus_width	    = 10,
	.subdevs	    = galileo2_tc358746a_subdevs,
	.measure_to_timings = &galileo2_get_timings,
};

#define GALILEO2_CAM_RAM_SIZE PAGE_ALIGN(7716 * 5364 * 2 * 4)

static u64 galileo2_avicam_dma_mask = DMA_BIT_MASK(32);

static struct platform_device galileo2_avicam_dev = {
	.name = "avicam",
	.id   = 0,
	.dev  = {
		.dma_mask          = &galileo2_avicam_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

void __init galileo2db_rsvmem(struct p7_board const* board)
{
	p7_reserve_avicammem(&galileo2_avicam_dev,
			     GALILEO2_CAM_RAM_SIZE);
	p7_reserve_dmamem();
}

void __init galileo2db_probe(struct p7_board const* board)
{
	p7brd_init_i2cm_muxed(2, 100, NULL, 0,
			      &galileo2db_i2cm2_mux_pdata,
			      galileo2db_i2cm2_mux_pins);

	p7_init_avicam(&galileo2_avicam_dev,
		       &galileo2_avicam_pdata,
		       galileo2_avicam_pins,
		       ARRAY_SIZE(galileo2_avicam_pins));

	p7_init_p7pwm(&galileo2db_pwm_pdata,
	              galileo2db_pwm_pins,
		      ARRAY_SIZE(galileo2db_pwm_pins));
}
