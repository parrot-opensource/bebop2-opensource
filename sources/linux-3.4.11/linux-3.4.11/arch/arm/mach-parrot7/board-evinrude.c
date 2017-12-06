/**
 * linux/arch/arm/mach-parrot7/board-evinrude.c - Parrot7 Evinrude platform
 *                                                implementation
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Alexandre Dilly <alexandre.dilly@parrot.com>
 * date:    11-May-2015
 *
 * This file is released under the GPL
 * Copyright (C) 2015 Parrot S.A.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/ramoops.h>
#include <linux/memblock.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <asm/system_info.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <video/avi.h>
#include <spi/p7-spi.h>
#include <spi/p7-spim.h>
#include <linux/reboot.h>
#include "board-common.h"
#include "common.h"
#include "pinctrl.h"
#include "venc.h"
#include "spi.h"
#include "usb.h"
#include "avi.h"
#include "p7_temperature.h"
#include "gpu.h"
#include "nand.h"
#include "usb.h"

/* USB HUB support */
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>

/* HDMI input support */
#include "sii_platform_data.h"

#include "board-common.h"
#include "common.h"
#include "p7_pwm.h"
#include "venc.h"
#include "gpio.h"
#include "gpu.h"

#include "board-drone-common.h"

/* Evinrude hardware revision */
enum {
	EV_HW00 = 0,
	EV_HW01,
	EV_HW02,
};

/********
 * HSIS *
 ********/

/* P7MU */
#define HSIS_HWxx__INT_P7MU		73

/* Sensors */
#define HSIS_HWxx__MAGNETO_INT_P7	81
#define HSIS_HWxx__FSYNC_CAM_V_GYRO_P7	89
#define HSIS_HWxx__FSYNC_CAM_H_GYRO_P7	90
#define HSIS_HWxx__FSYNC_GYRO_P7	HSIS_HWxx__FSYNC_CAM_H_GYRO_P7
#define HSIS_HWxx__GYRO_INT_P7		91
#define HSIS_HWxx__THERMAL_PWM		10
#define HSIS_HWxx__CLKIN_GYRO		11
#define HSIS_HWxx_PWM__CLKIN_GYRO	P7_PWM_11
#define HSIS_HWxx_PWM__THERMAL_PWM	P7_PWM_10

/* Horizontal camera MT9F002 */
#define HSIS_HWxx__CAMERA_H_MCLK	13
#define HSIS_HWxx__CAMERA_H_PWDN	133
#define HSIS_HWxx_PWM__CAMERA_H_MCLK	P7_PWM_13

/* Vertical camera MT9V117 */
#define HSIS_HWxx__CAMERA_V_MCLK	12
#define HSIS_HWxx__CAMERA_V_PWDN	78
#define HSIS_HWxx_PWM__CAMERA_V_MCLK	P7_PWM_12

/* HDMI input */
#define HSIS_HWxx__MHL_INT		197

/* USB */
#define HSIS_HWxx__HOST_MODE_3V3	202
#define HSIS_HWxx__HOST_MODE_ON		203
#define HSIS_HWxx__USB0_OC		204

/* UltraSound */
#define HSIS_HWxx__SELECT_ALIM_US	200

/* Motors */
#define HSIS_HWxx__MOTOR_PWM		0
#define HSIS_HWxx_PWM__MOTOR_PWM	P7_PWM_00
#define HSIS_HWxx__SERVO1_PWM		4
#define HSIS_HWxx__SERVO2_PWM		3
#define HSIS_HWxx__SERVO3_PWM		5
#define HSIS_HWxx__SERVO4_PWM		2
#define HSIS_HWxx__SERVO5_PWM		6
#define HSIS_HWxx__SERVO6_PWM		1
#define HSIS_HWxx_PWM__SERVO1_PWM	P7_PWM_04
#define HSIS_HWxx_PWM__SERVO2_PWM	P7_PWM_03
#define HSIS_HWxx_PWM__SERVO3_PWM	P7_PWM_05
#define HSIS_HWxx_PWM__SERVO4_PWM	P7_PWM_02
#define HSIS_HWxx_PWM__SERVO5_PWM	P7_PWM_06
#define HSIS_HWxx_PWM__SERVO6_PWM	P7_PWM_01

#define HSIS_HWxx__RC_PWM       7

/* Fan */
#define HSIS_HWxx__FANS_EN		85

/* Reset */
#define HSIS_HWxx__RESET_PSOC		199
#define HSIS_HWxx__RESET_WIFI		9
#define HSIS_HWxx__GNSS_RESET		84
#define HSIS_HWxx__HUB_RST		80
#define HSIS_HWxx__MHL_NRST		196

/* User */
#define HSIS_HWxx__USER_ON_OFF		201

/* Leds */
#define HSIS_HWxx_LED_RED_PWM   9
#define HSIS_HWxx_LED_GREEN_PWM 8
#define HSIS_HWxx_LED_BLUE_PWM  15

/* I2C bus */
#define AK8963_I2C_BUS			1
#define MPU6050_I2C_BUS			2
#define MS5607_I2C_BUS			1
#define BLDC_I2C_BUS			1
#define HUB_I2C_BUS			0
#define GNSS_I2C_BUS			1
#define HDMI_I2C_BUS			0

/* Evinrude HSIS */
struct ev_hsis {
	int p7rev;
	int pcbrev;
	int hwrev;
	/* P7MU */
	int p7mu_int;
	/* Cameras */
	int camera_h_mclk;
	int camera_h_pwdn;
	int camera_v_mclk;
	int camera_v_pwdn;
	/* HDMI/MHL input */
	int hdmi_int;
	/* Sensors */
	int thermal_pwm;
	int led_red_pwm;
	int led_green_pwm;
	int led_blue_pwm;
	int gyro_int_p7;
	int magneto_int_p7;
	int fsync_gyro_p7;
	int clkin_gyro;
	/* Motors */
	int motor_pwm;
	int servo1_pwm;
	int servo2_pwm;
	int servo3_pwm;
	int servo4_pwm;
	int servo5_pwm;
	int servo6_pwm;
	/* Reset */
	int reset_psoc;
	int reset_wifi;
	int reset_gnss;
	int reset_hub;
	int reset_hdmi;
	/* USB */
	int host_mode_3v3;
	int host_mode_on;
	int usb0_oc;
	/* Fan */
	int fan;
	/* UltraSound */
	int select_alim_us;
	/* User */
	int user_on_off;
} static ev_hsis = {
	/* No default values */
	.p7rev		 = -1,
	.pcbrev		 = -1,
	.hwrev		 = -1,
	/* P7MU */
	.p7mu_int	 = HSIS_HWxx__INT_P7MU,
	/* Cameras */
	.camera_h_mclk	 = HSIS_HWxx__CAMERA_H_MCLK,
	.camera_h_pwdn	 = HSIS_HWxx__CAMERA_H_PWDN,
	.camera_v_mclk	 = HSIS_HWxx__CAMERA_V_MCLK,
	.camera_v_pwdn	 = HSIS_HWxx__CAMERA_V_PWDN,
	/* HDMI/MHL input */
	.hdmi_int	 = HSIS_HWxx__MHL_INT,
	/* Sensors */
	.thermal_pwm	 = HSIS_HWxx__THERMAL_PWM,
	.led_red_pwm	 = HSIS_HWxx_LED_RED_PWM,
	.led_green_pwm	 = HSIS_HWxx_LED_GREEN_PWM,
	.led_blue_pwm	 = HSIS_HWxx_LED_BLUE_PWM,
	.gyro_int_p7	 = HSIS_HWxx__GYRO_INT_P7,
	.magneto_int_p7	 = HSIS_HWxx__MAGNETO_INT_P7,
	.fsync_gyro_p7	 = HSIS_HWxx__FSYNC_GYRO_P7,
	.clkin_gyro	 = HSIS_HWxx__CLKIN_GYRO,
	/* Motors */
	.motor_pwm	 = HSIS_HWxx__MOTOR_PWM,
	.servo1_pwm	 = HSIS_HWxx__SERVO1_PWM,
	.servo2_pwm	 = HSIS_HWxx__SERVO2_PWM,
	.servo3_pwm	 = HSIS_HWxx__SERVO3_PWM,
	.servo4_pwm	 = HSIS_HWxx__SERVO4_PWM,
	.servo5_pwm	 = HSIS_HWxx__SERVO5_PWM,
	.servo6_pwm	 = HSIS_HWxx__SERVO6_PWM,
	/* Reset */
	.reset_psoc	 = HSIS_HWxx__RESET_PSOC,
	.reset_wifi	 = HSIS_HWxx__RESET_WIFI,
	.reset_gnss	 = HSIS_HWxx__GNSS_RESET,
	.reset_hub	 = HSIS_HWxx__HUB_RST,
	.reset_hdmi	 = HSIS_HWxx__MHL_NRST,
	/* USB */
	.host_mode_3v3	 = HSIS_HWxx__HOST_MODE_3V3,
	.host_mode_on	 = HSIS_HWxx__HOST_MODE_ON,
	.usb0_oc	 = HSIS_HWxx__USB0_OC,
	/* Fan */
	.fan		 = HSIS_HWxx__FANS_EN,
	/* UltraSound */
	.select_alim_us	 = HSIS_HWxx__SELECT_ALIM_US,
	/* User */
	.user_on_off	 = HSIS_HWxx__USER_ON_OFF,
};

static void __init ev_board_probe(void)
{
	int gpios[] = {138, 139, 140};
	int pcbrev = 0;
	int p7rev;

	/* Get P7 revision */
	p7rev = p7_chiprev();
	ev_hsis.p7rev = p7rev + 1;

	/* Only P7R3 is supported */
	if (p7rev != P7_CHIPREV_R3)
		panic("this chip is not supported anymore\n");

	/* Get baord revision */
	pcbrev = drone_common_get_rev(gpios, ARRAY_SIZE(gpios), "Evinrude rev");
	ev_hsis.pcbrev = pcbrev;
	/* Get hardware revision */
	ev_hsis.hwrev = EV_HW00 + pcbrev;

	/* Displau revision during boot */
	pr_info("**************** HARDWARE HW %02d ***********************\n",
		ev_hsis.hwrev);
	pr_info("****************   PCB REV %02d   ***********************\n",
		pcbrev);
}

/***********************
 * Interrupts handling *
 ***********************/

static unsigned const ev_irq_gpios[] = {
	HSIS_HWxx__INT_P7MU,	   /* P7MU interrupt source */
	HSIS_HWxx__MAGNETO_INT_P7, /* Magneto interrupt */
	HSIS_HWxx__USER_ON_OFF,	   /* user button as the name */
	HSIS_HWxx__HOST_MODE_3V3,  /* detection of device/host USB dongle */
	HSIS_HWxx__USB0_OC,	   /* detection of overcurrent on USB */
	HSIS_HWxx__MHL_INT,	   /* HDMI input */
};

/* p7gpio_filter_phase  */
#define FSYNC_GYRO_FILTER 5
static struct p7gpio_filter_phase ev_irq_gpios_filter[] = {
	{
		.start	= HSIS_HWxx__FSYNC_GYRO_P7,
		.stop	= HSIS_HWxx__GYRO_INT_P7,
		.filter	= FSYNC_GYRO_FILTER,
		.mode	= GPIO_MEASURE_STOP
	}
};

/****************
 * Video memory *
 ****************/

/* Video memory is used for:
 *   video encoder,
 *   horizontal camera,
 *   vertival camera,
 *   MEM2MEM device (scaler and ISP)
 */
#define EVINRUDE_VIDEO_MEMORY_SIZE (206*1024*1024)

/*******
 * PWM *
 *******/

/* Clock configuration used for both cameras and Gyro */
static struct p7pwm_conf ev_conf_clock = {
	.period_precision = 5,
	.duty_precision   = 0,
	.mode		  = P7PWM_MODE_CLOCK,
};

/* PWM used to warm IMU sensors through a heating resistor */
static struct p7pwm_conf ev_conf_heating_resistor = {
	.period_precision = 5,
	.duty_precision   = 1,
	.mode		  = P7PWM_MODE_NORMAL
};

static struct p7pwm_conf ev_conf_pwm_leds = {
	.period_precision = 5,
	.duty_precision   = 5,
	.mode             = P7PWM_MODE_NORMAL,
};

/* PWM for motor and servo-motors */
static struct p7pwm_conf ev_conf_motor = {
	.period_precision = 5,
	.duty_precision   = 0,
	.mode		  = P7PWM_MODE_NORMAL
};

/* PWM for rc input*/
static struct p7pwm_conf ev_conf_rc = {
	.mode		  = P7PWM_MODE_PPM_RX
};

static struct p7pwm_pdata ev_pwm_pdata = {
	.conf = {
		[HSIS_HWxx__CAMERA_H_MCLK] = &ev_conf_clock,
		[HSIS_HWxx__CAMERA_V_MCLK] = &ev_conf_clock,
		[HSIS_HWxx__THERMAL_PWM]   = &ev_conf_heating_resistor,
		[HSIS_HWxx__CLKIN_GYRO]    = &ev_conf_clock,
		/* Motors */
		[HSIS_HWxx__MOTOR_PWM]     = &ev_conf_motor,
		[HSIS_HWxx__SERVO1_PWM]    = &ev_conf_motor,
		[HSIS_HWxx__SERVO2_PWM]    = &ev_conf_motor,
		[HSIS_HWxx__SERVO3_PWM]    = &ev_conf_motor,
		[HSIS_HWxx__SERVO4_PWM]    = &ev_conf_motor,
		[HSIS_HWxx__SERVO5_PWM]    = &ev_conf_motor,
		[HSIS_HWxx__SERVO6_PWM]    = &ev_conf_motor,
		/* RC in */
		[HSIS_HWxx__RC_PWM]        = &ev_conf_rc,
		/* Leds */
		[HSIS_HWxx_LED_RED_PWM]     = &ev_conf_pwm_leds,
		[HSIS_HWxx_LED_GREEN_PWM]   = &ev_conf_pwm_leds,
		[HSIS_HWxx_LED_BLUE_PWM]    = &ev_conf_pwm_leds,
	}
};

static unsigned long ev_cam_clock_config[] = {
	P7CTL_SLR_CFG(0) | /* Slew rate 0  */
	P7CTL_DRV_CFG(0), /* Drive strength 1 */
};

static unsigned long ev_led_pinconfig[] = {
	P7CTL_DRV_CFG(0),      /* Drive strength 0 (reg=1) */
};

static struct pinctrl_map ev_pwm_pins[] __initdata = {
	/* Camera H Master Clock */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CAMERA_H_MCLK),
	P7_INIT_PINCFG(HSIS_HWxx_PWM__CAMERA_H_MCLK, ev_cam_clock_config),
	/* Camera V Master Clock */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CAMERA_V_MCLK),
	P7_INIT_PINCFG(HSIS_HWxx_PWM__CAMERA_V_MCLK, ev_cam_clock_config),
	/* Gyroscope Clock Input */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CLKIN_GYRO),
	/* Thermal Resistor */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__THERMAL_PWM),
	/* Motors */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__MOTOR_PWM),
	P7_INIT_PINMAP(HSIS_HWxx_PWM__SERVO1_PWM),
	P7_INIT_PINMAP(HSIS_HWxx_PWM__SERVO2_PWM),
	P7_INIT_PINMAP(HSIS_HWxx_PWM__SERVO3_PWM),
	P7_INIT_PINMAP(HSIS_HWxx_PWM__SERVO4_PWM),
	P7_INIT_PINMAP(HSIS_HWxx_PWM__SERVO5_PWM),
	P7_INIT_PINMAP(HSIS_HWxx_PWM__SERVO6_PWM),
	P7_INIT_PINMAP(P7_PWM_07),
	/* Leds */
	P7_INIT_PINMAP(P7_PWM_08),
	P7_INIT_PINCFG(P7_PWM_08, ev_led_pinconfig),
	P7_INIT_PINMAP(P7_PWM_09),
	P7_INIT_PINCFG(P7_PWM_09, ev_led_pinconfig),
	P7_INIT_PINMAP(P7_PWM_15),
	P7_INIT_PINCFG(P7_PWM_15, ev_led_pinconfig),
};

/*************
 * LED pwm
 *************/

static struct led_pwm pwm_leds[] = {
	{
		.name = "evinrude:green",
		.default_trigger = "none",
		.pwm_id = HSIS_HWxx_LED_GREEN_PWM,
		.active_low = 0,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "evinrude:red",
		.default_trigger = "none",
		.pwm_id = HSIS_HWxx_LED_RED_PWM,
		.active_low = 0,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "evinrude:blue",
		.default_trigger = "none",
		.pwm_id = HSIS_HWxx_LED_BLUE_PWM,
		.active_low = 0,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
};

static struct led_pwm_platform_data pwm_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device evinrude_leds_pwm = {
	.name = "leds_pwm",
	.id = -1,
	.dev = {
		.platform_data = &pwm_data,
	},
};

/*************
 * USB hub
 *************/

static struct smsc82514_pdata ev_smsc82512_pdata = {
	.us_port   = DS_HIGH,
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = P7_GPIO_NR(HSIS_HWxx__HUB_RST),
};

static struct i2c_board_info smsc82512_info = {
	I2C_BOARD_INFO("smsc82512", 0x2c),
	.platform_data = &ev_smsc82512_pdata,
};

/**************
 * HDMI Input *
 **************/

/* HDMI input (on CAM1) */
static unsigned long ev_hdmi_cam1_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map ev_hdmi_cam1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINCFG(P7_CAM_1_CLK, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA08),
	P7_INIT_PINCFG(P7_CAM_1_DATA08, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA09),
	P7_INIT_PINCFG(P7_CAM_1_DATA09, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA10),
	P7_INIT_PINCFG(P7_CAM_1_DATA10, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA11),
	P7_INIT_PINCFG(P7_CAM_1_DATA11, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA12),
	P7_INIT_PINCFG(P7_CAM_1_DATA12, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA13),
	P7_INIT_PINCFG(P7_CAM_1_DATA13, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA14),
	P7_INIT_PINCFG(P7_CAM_1_DATA14, ev_hdmi_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA15),
	P7_INIT_PINCFG(P7_CAM_1_DATA15, ev_hdmi_cam1_pinconf),
};

/* Dummy camera for HDMI input */
static struct avicam_dummy_info ev_hdmi_cam_dummy_driver_info = {
	.dev_id = 2,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = 1280,
		.height	    = 720,
	},
};

static struct avicam_platform_data ev_hdmi_cam_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &ev_hdmi_cam_dummy_driver_info,
};

static u64 ev_hdmi_cam_dma_mask = DMA_BIT_MASK(32);
static struct platform_device ev_hdmi_cam_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &ev_hdmi_cam_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/* SII5293 driver */
static struct sii5293_platform_data ev_hdmi_pdata = {
	.version        = SII5293_PDATA_STRUCT_VERSION,
	.i2c_bus        = HDMI_I2C_BUS,
	.gpio_rst       = P7_GPIO_NR(HSIS_HWxx__MHL_NRST),
	.gpio_irq       = P7_GPIO_NR(HSIS_HWxx__MHL_INT),
	.output_format  = 8, /* YCbCr 4:2:2, embedded sync output */
	.input_dev_rap  = 1,
	.input_dev_rcp  = 1,
	.input_dev_ucp  = 1,
};

static struct platform_device ev_hdmi_pdev = {
	.name = "sii-5293",
	.id   = 0,
};

/* CAM0 */
static unsigned long ev_cam_h_cam0_pinconf[] = {
	P7CTL_SMT_CFG(OFF) /* no shimmt trigger */
};

/* Horizontal camera (on CAM0) */
static struct pinctrl_map cam_h_mt9f002_evinrude_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_HS),
	P7_INIT_PINCFG(P7_CAM_0_HS, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_VS),
	P7_INIT_PINCFG(P7_CAM_0_VS, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINCFG(P7_CAM_0_DATA08, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINCFG(P7_CAM_0_DATA09, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINCFG(P7_CAM_0_DATA10, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINCFG(P7_CAM_0_DATA11, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINCFG(P7_CAM_0_DATA12, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINCFG(P7_CAM_0_DATA13, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINCFG(P7_CAM_0_DATA14, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINCFG(P7_CAM_0_DATA15, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA16),
	P7_INIT_PINCFG(P7_CAM_0_DATA16, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA17),
	P7_INIT_PINCFG(P7_CAM_0_DATA17, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA18),
	P7_INIT_PINCFG(P7_CAM_0_DATA18, ev_cam_h_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA19),
	P7_INIT_PINCFG(P7_CAM_0_DATA19, ev_cam_h_cam0_pinconf),
};

static union avi_cam_interface cam_h_interface_evinrude = {
	.itu656     = 0,
	.pad_select = 1,
	.ivs	    = 1,
	.ihs	    = 1,
	.ipc	    = 0,
	.psync_en   = 1,
	.psync_rf   = 1,
};

/***************
 * GPIO export *
 ***************/

#define EV_GPIO(gpio, flags, label)\
				DRONE_COMMON_GPIO(&ev_hsis.gpio, flags, label)
struct drone_common_gpio ev_gpios[] = {
	EV_GPIO(fsync_gyro_p7, GPIOF_IN, "FSYNC_GYRO_P7"),
	EV_GPIO(reset_wifi, GPIOF_OUT_INIT_LOW, "RESET_WIFI"),
	EV_GPIO(user_on_off, GPIOF_IN, "USER_ON_OFF"),
	EV_GPIO(select_alim_us, GPIOF_OUT_INIT_LOW, "SELECT_ALIM_US"),
	{ .gpio = NULL, }
};

/**************
 * HSIS Sysfs *
 **************/

#define EV_HSIS_SYSFS_ATTR(x) DRONE_COMMON_HSIS_SYSFS_ATTR(x, &ev_hsis.x)
struct drone_common_hsis_sysfs_attr ev_hsis_sysfs[] = {
	EV_HSIS_SYSFS_ATTR(select_alim_us),
	EV_HSIS_SYSFS_ATTR(reset_psoc),
	EV_HSIS_SYSFS_ATTR(reset_wifi),
	EV_HSIS_SYSFS_ATTR(reset_gnss),
	EV_HSIS_SYSFS_ATTR(reset_hub),
	EV_HSIS_SYSFS_ATTR(user_on_off),
	EV_HSIS_SYSFS_ATTR(fsync_gyro_p7),
	EV_HSIS_SYSFS_ATTR(gyro_int_p7),
	EV_HSIS_SYSFS_ATTR(magneto_int_p7),
	EV_HSIS_SYSFS_ATTR(thermal_pwm),
	EV_HSIS_SYSFS_ATTR(led_red_pwm),
	EV_HSIS_SYSFS_ATTR(led_green_pwm),
	EV_HSIS_SYSFS_ATTR(led_blue_pwm),
	EV_HSIS_SYSFS_ATTR(fan),
#ifndef DRIVER_VIDEO_MT9V117
	EV_HSIS_SYSFS_ATTR(camera_v_mclk),
#endif
	EV_HSIS_SYSFS_ATTR(camera_h_mclk),
	EV_HSIS_SYSFS_ATTR(camera_v_pwdn),
	EV_HSIS_SYSFS_ATTR(camera_h_pwdn),
	EV_HSIS_SYSFS_ATTR(clkin_gyro),
	EV_HSIS_SYSFS_ATTR(motor_pwm),
	EV_HSIS_SYSFS_ATTR(servo1_pwm),
	EV_HSIS_SYSFS_ATTR(servo2_pwm),
	EV_HSIS_SYSFS_ATTR(servo3_pwm),
	EV_HSIS_SYSFS_ATTR(servo4_pwm),
	EV_HSIS_SYSFS_ATTR(servo5_pwm),
	EV_HSIS_SYSFS_ATTR(servo6_pwm),
	EV_HSIS_SYSFS_ATTR(host_mode_3v3),
	EV_HSIS_SYSFS_ATTR(host_mode_on),
	EV_HSIS_SYSFS_ATTR(usb0_oc),
	EV_HSIS_SYSFS_ATTR(p7rev),
	EV_HSIS_SYSFS_ATTR(hwrev),
	EV_HSIS_SYSFS_ATTR(pcbrev),
	{ .value = NULL, } 
};

/*******
 * BSP *
 *******/

static void __init evinrude_reserve_mem(void)
{
	drone_common_reserve_mem_ramoops();
	drone_common_reserve_mem(EVINRUDE_VIDEO_MEMORY_SIZE);
}

static void evinrude_configure_leds(void)
{
	struct pwm_device *pwm;

	pwm = pwm_request(HSIS_HWxx_LED_RED_PWM, "evinrude BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 0, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(HSIS_HWxx_LED_GREEN_PWM, "evinrude BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 0, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	if (ev_hsis.hwrev >= EV_HW02) {
		pwm = pwm_request(HSIS_HWxx_LED_BLUE_PWM, "evinrude BSP");

		if (!IS_ERR(pwm)) {
			pwm_config(pwm, 1000000, 2000000);
			pwm_enable(pwm);
			pwm_free(pwm);
		}
	}
}

static int evinrude_rst_notify_sys(struct notifier_block *this,
		unsigned long code, void *unused)
{
	/* reset wifi chip, otherwise sometimes it doesn't work after reboot */
	if (gpio_is_valid(ev_hsis.reset_wifi)) {
		gpio_set_value(ev_hsis.reset_wifi, 1);
		msleep(50);
	}

	return 0;
}

static struct notifier_block evinrude_rst_notifier = {
	.notifier_call = evinrude_rst_notify_sys,
};


static void __init evinrude_init_mach(void)
{
	int i;

	/* Initialize ramoops */
	drone_common_init_ramoops();

	/* Init hardware revision independent stuff */
	p7_init_mach();

	/* Init GPIOs */
	p7_init_gpio(NULL, 0);

	/* Get PCB/HW revision and update HSIS */
	ev_board_probe();

	/* Init debug UART */
	p7brd_init_uart(0, 0);

	/* Init NAND */
	p7brd_init_nand(0);

	/* Init I2C master
	 * I2CM-0:
	 *     P7MU
	 *     Vertical Camera
	 *     Horizontal Camera
	 *     USB HUB
	 *     HDMI Input (with EDID EEPROM)
	 * I2CM-1:
	 *     AKM8963 (magneto)
	 *     MS5607 (Pressure/Temperature)
	 *     BLDC (Motors)
	 *     GNSS
	 * I2CM-2:
	 *     MPU6050 (gyro/accel)
	 */
	p7brd_init_i2cm(0, 400);
	p7brd_init_i2cm(1, 400);
	p7brd_init_i2cm(2, 400);

	/* Init GPS */
	p7brd_init_uart(1, 0);

	/* Init remote RC RF */
	p7brd_init_uart(2, 0); /* SBUS */
	p7brd_init_uart(3, 0); /* SUMD */

	/* Add GPIO interrupts */
	for (i = 0; i < ARRAY_SIZE(ev_irq_gpios); i++)
		p7_gpio_interrupt_register(ev_irq_gpios[i]);

	/* Add GPIO interrupt filters */
	for (i = 0; i < ARRAY_SIZE(ev_irq_gpios_filter) ; i++)
		p7_gpio_filter_interrupt_register(ev_irq_gpios_filter[i]);

	/* Initialize P7MU */
	drone_common_init_p7mu(ev_hsis.p7mu_int, 0);

	/* Init PWM */
	p7_init_p7pwm(&ev_pwm_pdata, ev_pwm_pins, ARRAY_SIZE(ev_pwm_pins));

	/* PWM leds */
	evinrude_configure_leds();
	platform_device_register(&evinrude_leds_pwm);

	/* Init eMMC (use default mapping) */
	drone_common_init_sdhci(NULL, 0);

	/* Init Ultrasound */
	drone_common_init_us_tx();

	/* Init AVI */
	p7_init_avi();

	/* Init cameras
	 *     mt9v117 for vertical camera on CAM2
	 *     mt9f002 for horizontal camera on CAM0
	 */
	drone_common_init_cam_v_mt9v117(ev_hsis.camera_v_mclk,
					ev_hsis.camera_v_pwdn);
	drone_common_init_cam_h_mt9f002(ev_hsis.camera_h_mclk,
					ev_hsis.camera_h_pwdn, &cam_h_interface_evinrude,
					cam_h_mt9f002_evinrude_pins, ARRAY_SIZE(cam_h_mt9f002_evinrude_pins));

	/* Init MEM2MEM (use default) */
	drone_common_init_m2m(NULL);

	/* Init GPU */
	p7_init_gpu_fb(0, 0, 4);

	/* Init video encoder */
	p7_init_venc();

	/* Init USB */
	drone_common_init_usb(ev_hsis.host_mode_on, ev_hsis.host_mode_3v3,
			      ev_hsis.usb0_oc, 0);
	/* Init EHCI 1 */
	p7brd_init_usb(1, -1, CI_UDC_DR_HOST);

	/* Init sensors */
	drone_common_init_ak8963(AK8963_I2C_BUS, ev_hsis.magneto_int_p7);
	drone_common_init_inv_mpu6050(MPU6050_I2C_BUS, ev_hsis.gyro_int_p7,
				      FSYNC_GYRO_FILTER, ev_hsis.clkin_gyro);
	drone_common_init_ms5607(MS5607_I2C_BUS);

	/* Init BLDC */
	drone_common_init_bldc(BLDC_I2C_BUS, ev_hsis.reset_psoc);

	/* Init FAN */
	drone_common_init_fan(ev_hsis.fan);

	/* Init USB Hub */
	gpio_request_one(P7_GPIO_NR(ev_hsis.reset_hub), GPIOF_OUT_INIT_LOW,
			 "RESET_USB_HUB");
	parrot_init_i2c_slave(HUB_I2C_BUS, &smsc82512_info, "smsc 82512",
			      P7_I2C_NOIRQ);

	/* Init HDMI input */
	p7_init_avicam(&ev_hdmi_cam_dev, &ev_hdmi_cam_pdata, ev_hdmi_cam1_pins,
		       ARRAY_SIZE(ev_hdmi_cam1_pins));
	p7_init_dev(&ev_hdmi_pdev, &ev_hdmi_pdata, NULL, 0);

	/* Export default and custom GPIOs */
	pr_info("Evinrude board : exporting GPIOs\n");
	if (ev_hsis.hwrev >= EV_HW02)
		drone_common_export_gpio(ev_hsis.reset_gnss,GPIOF_OUT_INIT_LOW, "RESET_GNSS");
	else
		drone_common_export_gpio(ev_hsis.reset_gnss,GPIOF_OUT_INIT_HIGH, "RESET_GNSS");
	drone_common_export_gpios(ev_gpios);

	/* Create sysfs entries (in /sys/kernel/hsis/...) */
	pr_info(
	    "Evinrude board : exporting HSIS to userspace in /sys/kernel/hsis");
	drone_common_init_sysfs(ev_hsis_sysfs);

	register_reboot_notifier(&evinrude_rst_notifier);

	/* End of initialization */
	pr_info("Evinrude board : init done\n");
}

P7_MACHINE_START(PARROT_CHIMERA, "Chimera board")
	.reserve        = &evinrude_reserve_mem,
	.init_machine   = &evinrude_init_mach,
P7_MACHINE_END

P7_MACHINE_START(PARROT_EVINRUDE, "Evinrude board")
	.reserve        = &evinrude_reserve_mem,
	.init_machine   = &evinrude_init_mach,
P7_MACHINE_END
