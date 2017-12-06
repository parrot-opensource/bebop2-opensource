/**
 * linux/arch/arm/mach-parrot7/mykonos3-board.c - Parrot7 Mykonos3 platform
 *                                                implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Florent Bayendrian <florent.bayendrian@parrot.com>
 * date:    08-Oct-2012
 *
 * This file is released under the GPL
 * Copyright (C) 2012 Parrot S.A.
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
#include "p7_pwm.h"
#include "vdec.h"
#include "venc.h"
#include "gpio.h"
#include "gpu.h"

#include "board-drone-common.h"

/* Mykono3 hardware revision */
enum {
	MK3_HW01 = 1,
	MK3_HW02,
	MK3_HW03,
	MK3_HW04,
	MK3_HW05,
	MK3_HW06,
	MK3_HW07,
	MK3_HW08,
	MK3_HW09, /* RIP */
	MK3_HW10,
	MK3_HW11,
};

enum mk3_hardware_board {
	MK3X_MK3,
	MK3X_MILOS,
};

/********
 * HSIS *
 ********/

/* P7MU */
#define HSIS_HWxx__INT_P7MU		73

/* Sensors */
#define HSIS_HWxx__MAGNETO_INT_P7	124
#define HSIS_HWxx__FSYNC_CAM_V_GYRO_P7	89
#define HSIS_HWxx__FSYNC_CAM_H_GYRO_P7	90
#define HSIS_HWxx__FSYNC_GYRO_P7	HSIS_HWxx__FSYNC_CAM_H_GYRO_P7
#define HSIS_HWxx__GYRO_INT_P7		91
#define HSIS_HWxx__THERMAL_PWM		6
#define HSIS_HWxx__CLKIN_GYRO		8
#define HSIS_HWxx_PWM__CLKIN_GYRO	P7_PWM_08
#define HSIS_HWxx_PWM__THERMAL_PWM	P7_PWM_06

/* Horizontal camera MT9F002 */
#define HSIS_HWxx__CAMERA_H_MCLK	11
#define HSIS_HWxx__CAMERA_H_PWDN	132
#define HSIS_HWxx_PWM__CAMERA_H_MCLK	P7_PWM_11

/* Vertical camera MT9V117 */
#define HSIS_HWxx__CAMERA_V_MCLK	9
#define HSIS_HWxx__CAMERA_V_PWDN	129
#define HSIS_HWxx_PWM__CAMERA_V_MCLK	P7_PWM_09

/* GPS */
#define HSIS_HWxx__POWER_EN_GNSS		81
#define HSIS_HWxx__GNSS_FLASH	82

/* USB */
#define HSIS_HWxx__HOST_MODE_3V3	202
#define HSIS_HWxx__HOST_MODE_ON		203
#define HSIS_HWxx__USB0_OC		204

/* UltraSound */
#define HSIS_HWxx__SELECT_ALIM_US	200

/* Fan */
#define HSIS_HW08__FANS_EN		6
#define HSIS_HW10__FANS_EN		85

/* Reset */
#define HSIS_HWxx__RESET_PSOC		199
#define HSIS_HW11__RESET_WIFI		9

/* User */
#define HSIS_HWxx__USER_ON_OFF		201

/* Milos */
#define HSIS_HWxx_SUPER_LED_PWM		0
#define HSIS_HWxx__GNSS_RESET		83

/* I2C bus */
#define AK8963_I2C_BUS		1
#define MPU6050_I2C_BUS		2
#define MS5607_I2C_BUS		1
#define BLDC_I2C_BUS		1

/* Mykonos3 HSIS */
struct mk3_hsis {
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
	/* Sensors */
	int thermal_pwm;
	int super_led_pwm;
	int gyro_int_p7;
	int magneto_int_p7;
	int fsync_gyro_p7;
	int clkin_gyro;
	/* Reset */
	int reset_psoc;
	int reset_wifi;
	/* USB */
	int host_mode_3v3;
	int host_mode_on;
	int usb0_oc;
	/* GPS */
	int gps_power_en;
	int gps_flash;
	int reset_gnss;
	/* Fan */
	int fan;
	/* UltraSound */
	int select_alim_us;
	/* User */
	int user_on_off;
} static mk3_hsis = {
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
	/* Sensors */
	.thermal_pwm	 = HSIS_HWxx__THERMAL_PWM,
	.super_led_pwm	 = -1,
	.gyro_int_p7	 = HSIS_HWxx__GYRO_INT_P7,
	.magneto_int_p7	 = HSIS_HWxx__MAGNETO_INT_P7,
	.fsync_gyro_p7	 = HSIS_HWxx__FSYNC_GYRO_P7,
	.clkin_gyro	 = HSIS_HWxx__CLKIN_GYRO,
	/* Reset */
	.reset_psoc	 = HSIS_HWxx__RESET_PSOC,
	.reset_wifi	 = HSIS_HW11__RESET_WIFI,
	/* USB */
	.host_mode_3v3	 = HSIS_HWxx__HOST_MODE_3V3,
	.host_mode_on	 = HSIS_HWxx__HOST_MODE_ON,
	.usb0_oc	 = HSIS_HWxx__USB0_OC,
	/* GPS */
	.reset_gnss = -1,
	.gps_power_en	 = HSIS_HWxx__POWER_EN_GNSS,
	.gps_flash		= HSIS_HWxx__GNSS_FLASH,
	/* Fan */
	.fan		 = HSIS_HW10__FANS_EN,
	/* UltraSound */
	.select_alim_us	 = HSIS_HWxx__SELECT_ALIM_US,
	/* User */
	.user_on_off	 = HSIS_HWxx__USER_ON_OFF,
};

static int __init mk3_drone_set_fan(char *option)
{
	return drone_common_set_fan(option);
}
early_param("mk3_fan", mk3_drone_set_fan);

static void __init mk3_board_probe(void)
{
	int gpios[] = {138, 139, 140};
	int hwrev = MK3_HW01;
	int pcbrev = 0;
	int p7rev;

	/* Get P7 revision */
	p7rev = p7_chiprev();
	mk3_hsis.p7rev = p7rev + 1;

	/* Only P7R3 is supported */
	if (p7rev != P7_CHIPREV_R3)
		panic("this chip is not supported anymore\n");

	/* Get baord revision */
	pcbrev = drone_common_get_rev(gpios, ARRAY_SIZE(gpios), "mykonos3 rev");
	mk3_hsis.pcbrev = pcbrev;

	/* Get hardware revision */
	if (pcbrev == 0) {
		hwrev = MK3_HW11;
	} else if (pcbrev == 0x6 || pcbrev == 0x7) {
		hwrev = MK3_HW10;
	} else {
		/* hw04 ... hw08 */
		hwrev = pcbrev + MK3_HW03;
	}
	mk3_hsis.hwrev = hwrev;

	/* We don't support HW older than HW06 */
	if (hwrev < MK3_HW06)
		panic("unsupported board\n");

	/* Display revision during boot */
	pr_info("**************** HARDWARE HW %02d ***********************\n",
		hwrev);
	pr_info("****************   PCB REV %02d   ***********************\n",
		pcbrev);

	/* Compatibility issue when flashed with SW <= 1.98 and updated to 3.x */
	if(system_rev != mk3_hsis.hwrev) {
		pr_warn("system_rev != mk3_hsis.hwrev\n");
		pr_warn("may be caused by an old bootloader\n");
		system_rev = mk3_hsis.hwrev;
	}

	/* Configure HSIS with PCB/HW revision */
	if (hwrev < MK3_HW08) {
		mk3_hsis.gps_power_en = -1;
		mk3_hsis.fan = -1;
		mk3_hsis.usb0_oc = -1;
	} else if (hwrev < MK3_HW10) {
		/* TP for test ... */
		mk3_hsis.fan = HSIS_HW08__FANS_EN;
	}
	if (hwrev < MK3_HW10) {
		mk3_hsis.gps_flash = -1;
	}
	if (hwrev < MK3_HW11) {
		mk3_hsis.reset_wifi = -1;
	}
}

static void __init milos_board_probe(void)
{
	int gpios[] = {138, 139, 140};
	int hwrev = MK3_HW01;
	int pcbrev = 0;
	int p7rev;

	/* Get P7 revision */
	p7rev = p7_chiprev();
	mk3_hsis.p7rev = p7rev + 1;

	/* Only P7R3 is supported */
	if (p7rev != P7_CHIPREV_R3)
		panic("this chip is not supported anymore\n");

	/* Get baord revision */
	pcbrev = drone_common_get_rev(gpios, ARRAY_SIZE(gpios), "mykonos3 rev");
	mk3_hsis.pcbrev = pcbrev;

	/* Get hardware revision : we follow bebop rev to avoid to break
	 scripts */
	hwrev = MK3_HW11 + pcbrev;
	mk3_hsis.hwrev = hwrev;

	/* Display revision during boot */
	pr_info("**************** HARDWARE HW %02d ***********************\n",
		hwrev);
	pr_info("****************   PCB REV %02d   ***********************\n",
		pcbrev);
}

/***********************
 * Interrupts handling *
 ***********************/

static unsigned const mk3_irq_gpios[] = {
	HSIS_HWxx__INT_P7MU,	   /* P7MU interrupt source */
	HSIS_HWxx__MAGNETO_INT_P7, /* Magneto interrupt */
	HSIS_HWxx__USER_ON_OFF,	   /* user button as the name */
	HSIS_HWxx__HOST_MODE_3V3,  /* detection of device/host USB dongle */
	HSIS_HWxx__USB0_OC,	   /* detection of overcurrent on USB */
};

/* p7gpio_filter_phase  */
#define FSYNC_GYRO_FILTER 5
static struct p7gpio_filter_phase mk3_irq_gpios_filter[] = {
	{
		.start		= HSIS_HWxx__FSYNC_GYRO_P7,
		.stop		= HSIS_HWxx__GYRO_INT_P7,
		.filter		= FSYNC_GYRO_FILTER,
		.mode		= GPIO_MEASURE_STOP,
		.export_reset 	= 1
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
#define MYKONOS3_VIDEO_MEMORY_SIZE (206*1024*1024)

/*******
 * PWM *
 *******/

/* Clock configuration used for both cameras and Gyro */
static struct p7pwm_conf mk3_conf_clock = {
	.period_precision = 5,
	.duty_precision   = 0,
	.mode		  = P7PWM_MODE_CLOCK,
};

/* PWM used to warm IMU sensors through a heating resistor */
static struct p7pwm_conf mk3_conf_heating_resistor = {
	.period_precision = 5,
	.duty_precision   = 1,
	.mode		  = P7PWM_MODE_NORMAL
};

static struct p7pwm_conf milos_conf_pwm_leds = {
	.period_precision = 5,
	.duty_precision   = 5,
	.mode             = P7PWM_MODE_NORMAL,
};

static struct p7pwm_pdata mk3_pwm_pdata = {
	.conf = {
		[HSIS_HWxx__CAMERA_H_MCLK] = &mk3_conf_clock,
		[HSIS_HWxx__CAMERA_V_MCLK] = &mk3_conf_clock,
		[HSIS_HWxx__THERMAL_PWM]   = &mk3_conf_heating_resistor,
		[HSIS_HWxx__CLKIN_GYRO]    = &mk3_conf_clock,
	}
};

static struct p7pwm_pdata milos_pwm_pdata = {
	.conf = {
		[HSIS_HWxx__CAMERA_H_MCLK] = &mk3_conf_clock,
		[HSIS_HWxx__CAMERA_V_MCLK] = &mk3_conf_clock,
		[HSIS_HWxx__THERMAL_PWM]   = &mk3_conf_heating_resistor,
		[HSIS_HWxx__CLKIN_GYRO]    = &mk3_conf_clock,
		/* Leds */
		[HSIS_HWxx_SUPER_LED_PWM]  = &milos_conf_pwm_leds,
	}
};

static unsigned long mk3_cam_clock_config[] = {
	P7CTL_SLR_CFG(0) | /* Slew rate 0  */
	P7CTL_DRV_CFG(0), /* Drive strength 1 */
};

static unsigned long milos_led_pinconfig[] = {
	P7CTL_DRV_CFG(0),      /* Drive strength 0 (reg=1) */
};

static struct pinctrl_map mk3_pwm_pins[] __initdata = {
	/* Camera H Master Clock */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CAMERA_H_MCLK),
	P7_INIT_PINCFG(HSIS_HWxx_PWM__CAMERA_H_MCLK, mk3_cam_clock_config),
	/* Camera V Master Clock */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CAMERA_V_MCLK),
	P7_INIT_PINCFG(HSIS_HWxx_PWM__CAMERA_V_MCLK, mk3_cam_clock_config),
	/* Gyroscope Clock Input */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CLKIN_GYRO),
	/* Thermal Resistor */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__THERMAL_PWM),
};

static struct pinctrl_map milos_pwm_pins[] __initdata = {
	/* Camera H Master Clock */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CAMERA_H_MCLK),
	P7_INIT_PINCFG(HSIS_HWxx_PWM__CAMERA_H_MCLK, mk3_cam_clock_config),
	/* Camera V Master Clock */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CAMERA_V_MCLK),
	P7_INIT_PINCFG(HSIS_HWxx_PWM__CAMERA_V_MCLK, mk3_cam_clock_config),
	/* Gyroscope Clock Input */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__CLKIN_GYRO),
	/* Thermal Resistor */
	P7_INIT_PINMAP(HSIS_HWxx_PWM__THERMAL_PWM),
	/* Leds */
	P7_INIT_PINMAP(P7_PWM_00),
	P7_INIT_PINCFG(P7_PWM_00, milos_led_pinconfig),
};

/*************
 * LED pwm
 *************/

static struct led_pwm pwm_leds[] = {
	{
		.name = "milos:super_led",
		.default_trigger = "none",
		.pwm_id = HSIS_HWxx_SUPER_LED_PWM,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
};

static struct led_pwm_platform_data pwm_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device milos_leds_pwm = {
	.name = "leds_pwm",
	.id = -1,
	.dev = {
		.platform_data = &pwm_data,
	},
};
/*************************
 * SDHCI (for HW07 only) *
 *************************/

static unsigned long mk3_hw07_sdhci_pins_config_clk[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(5),
	P7CTL_SLR_CFG(2),
};

static unsigned long mk3_hw07_sdhci_pins_config[] = {
	P7CTL_PUD_CFG(UP),
	P7CTL_DRV_CFG(4),
};

static struct pinctrl_map mk3_hw07_sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, mk3_hw07_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, mk3_hw07_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, mk3_hw07_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, mk3_hw07_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, mk3_hw07_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, mk3_hw07_sdhci_pins_config)
};

/***************
 * GPIO export *
 ***************/

#define MK3_GPIO(gpio, flags, label)\
				DRONE_COMMON_GPIO(&mk3_hsis.gpio, flags, label)
struct drone_common_gpio mk3_gpios[] = {
	MK3_GPIO(fsync_gyro_p7, GPIOF_IN, "FSYNC_GYRO_P7"),
	MK3_GPIO(reset_wifi, GPIOF_OUT_INIT_LOW, "RESET_WIFI"),
	MK3_GPIO(reset_gnss, GPIOF_OUT_INIT_LOW, "RESET_GNSS"),
	MK3_GPIO(gps_flash, GPIOF_OUT_INIT_LOW, "GPS_FLASH"),
	MK3_GPIO(gps_power_en, GPIOF_OUT_INIT_HIGH, "GPS_POWER_EN"),
	MK3_GPIO(user_on_off, GPIOF_IN, "USER_ON_OFF"),
	MK3_GPIO(select_alim_us, GPIOF_OUT_INIT_LOW, "SELECT_ALIM_US"),
	{ .gpio = NULL, }
};

struct drone_common_gpio milos_gpios[] = {
	MK3_GPIO(fsync_gyro_p7, GPIOF_IN, "FSYNC_GYRO_P7"),
	MK3_GPIO(reset_wifi, GPIOF_OUT_INIT_LOW, "RESET_WIFI"),
	MK3_GPIO(reset_gnss, GPIOF_OUT_INIT_LOW, "RESET_GNSS"),
	MK3_GPIO(gps_flash, GPIOF_OUT_INIT_HIGH, "GPS_FLASH"),
	MK3_GPIO(gps_power_en, GPIOF_OUT_INIT_HIGH, "GPS_POWER_EN"),
	MK3_GPIO(user_on_off, GPIOF_IN, "USER_ON_OFF"),
	MK3_GPIO(select_alim_us, GPIOF_OUT_INIT_LOW, "SELECT_ALIM_US"),
	{ .gpio = NULL, }
};

/**************
 * HSIS Sysfs *
 **************/

#define MK3_HSIS_SYSFS_ATTR(x) DRONE_COMMON_HSIS_SYSFS_ATTR(x, &mk3_hsis.x)
struct drone_common_hsis_sysfs_attr mk3_hsis_sysfs[] = {
	/* gpio */
	MK3_HSIS_SYSFS_ATTR(select_alim_us),
	MK3_HSIS_SYSFS_ATTR(reset_psoc),
	MK3_HSIS_SYSFS_ATTR(reset_wifi),
	MK3_HSIS_SYSFS_ATTR(user_on_off),
	MK3_HSIS_SYSFS_ATTR(fsync_gyro_p7),
	MK3_HSIS_SYSFS_ATTR(gyro_int_p7),
	MK3_HSIS_SYSFS_ATTR(magneto_int_p7),
	MK3_HSIS_SYSFS_ATTR(fan),
	MK3_HSIS_SYSFS_ATTR(camera_v_pwdn),
	MK3_HSIS_SYSFS_ATTR(camera_h_pwdn),
	MK3_HSIS_SYSFS_ATTR(host_mode_3v3), /* vbus detect */
	MK3_HSIS_SYSFS_ATTR(host_mode_on), /* vbus on */
	MK3_HSIS_SYSFS_ATTR(usb0_oc),
	MK3_HSIS_SYSFS_ATTR(gps_power_en),
	/* pwm */
	MK3_HSIS_SYSFS_ATTR(thermal_pwm),
	MK3_HSIS_SYSFS_ATTR(super_led_pwm),
#ifndef DRIVER_VIDEO_MT9V117
	MK3_HSIS_SYSFS_ATTR(camera_v_mclk),
#endif
	MK3_HSIS_SYSFS_ATTR(camera_h_mclk),
	MK3_HSIS_SYSFS_ATTR(clkin_gyro),
	/* rev */
	MK3_HSIS_SYSFS_ATTR(p7rev),
	MK3_HSIS_SYSFS_ATTR(hwrev),
	MK3_HSIS_SYSFS_ATTR(pcbrev),
	{ .value = NULL, }
};


/**************
 * Motors     *
 **************/

/* motors lut */
#define BLDC_LUT_DEFAULT {0, 1, 2, 3}
static const u8 bldc_lut_default[4] = BLDC_LUT_DEFAULT;

#define BLDC_LUT_MILOS_DV {0, 1, 3, 2}
static const u8 bldc_lut_milos_dv[4] = BLDC_LUT_MILOS_DV;

/* motors spin direction */
#define BLDC_SPIN_DIR_DEFAULT 0x5 /* 0b0101, CW/CCW/CW/CCW */
#define BLDC_SPIN_DIR_MILOS 0xA   /* 0b1010, CCW/CW/CCW/CW */

/* bebop bldc platform data */
static struct bldc_cypress_platform_data bldc_cypress_pdata = {
	.lut = BLDC_LUT_DEFAULT,
	.spin_dir = BLDC_SPIN_DIR_DEFAULT,
};

/*******
 * IIO *
 *******/
static struct platform_device mykonos3_iio_device = {
	.name = "iio_mykonos3",
	.id = -1,
};

static void __init mykonos3_iio_init(void)
{
	platform_device_register(&mykonos3_iio_device);
}

/*****************
 * Sensor AK8975 *
 *****************/

#ifdef DRIVER_PARROT_IIO_AK8975

#include <linux/platform_data/ak8975.h>

#define DRONE_MAG_ROTATION_MATRIX\
	"-0.984807753012208, 0, -0.173648177666930; "\
	"0, -1, 0; "\
	"-0.173648177666930, 0, 0.984807753012208\n"

static struct ak8975_platform_data ak8963_pdata = {
	.orientation = DRONE_MAG_ROTATION_MATRIX,
	.drdy_gpio = -1, /* GPIO connected to DRDY pin exclusive with I2C irq */
	.trg_gpio = -1,  /* GPIO connected to TRG pin (AK8963)  */
};
#else
#define IIO_MAGNETOMETER_AK8963 "ak8963"
#endif

static struct i2c_board_info ak8963_info = {
	I2C_BOARD_INFO(IIO_MAGNETOMETER_AK8963, 0x0d),
#ifdef DRIVER_PARROT_IIO_AK8975
	.platform_data = &ak8963_pdata,
#endif
};

static void __init milos_init_ak8963(int i2c_bus, int irq)
{
#ifdef DRIVER_PARROT_IIO_AK8975
	ak8963_info.irq = P7_GPIO_NR(irq);
	parrot_init_i2c_slave(i2c_bus, &ak8963_info, "Magnetometer",
			      irq > 0 ? P7_I2C_IRQ : P7_I2C_NOIRQ);
#else
	p7brd_export_i2c_hw_infos(i2c_bus, ak8963_info.addr, "Magnetometer",
				  ak8963_info.type);
	drone_common_export_gpio(irq, GPIOF_IN, "MAGNETO_INT_P7");
#endif
}

/*******
 * BSP *
 *******/

static void __init mykonos3_reserve_mem(void)
{
	drone_common_reserve_mem_ramoops();
	drone_common_reserve_mem(MYKONOS3_VIDEO_MEMORY_SIZE);
}

static void milos_configure_leds(void)
{
	struct pwm_device *pwm;

	pwm = pwm_request(HSIS_HWxx_SUPER_LED_PWM, "milos BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 100000, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}
}

static int mykonos3x_rst_notify_sys(struct notifier_block *this,
		unsigned long code, void *unused)
{
	/* reset wifi chip, otherwise sometimes it doesn't work after reboot */
	if (gpio_is_valid(mk3_hsis.reset_wifi)) {
		gpio_set_value(mk3_hsis.reset_wifi, 1);
		msleep(50);
	}
	return 0;
}

static struct notifier_block mykonos3x_rst_notifier = {
	.notifier_call = mykonos3x_rst_notify_sys,
};

static void __init mykonos3x_init_mach(enum mk3_hardware_board board)
{
	int i;

	/* Initialize ramoops */
	drone_common_init_ramoops();

	/* Init hardware revision independent stuff */
	p7_init_mach();

	/* Init GPIOs */
	p7_init_gpio(NULL, 0);

	/* Get PCB/HW revision and update HSIS */
	if (board == MK3X_MK3)
		mk3_board_probe();
	else
		milos_board_probe();

	/* Init debug UART */
	p7brd_init_uart(0, 0);

	/* Init NAND */
	p7brd_init_nand(0);

	/* Init I2C master
	 * I2CM-0:
	 *     P7MU
	 *     Vertical Camera
	 *     Horizontal Camera
	 * I2CM-1:
	 *     AKM8963 (magneto)
	 *     MS5607 (Pressure/Temperature)
	 *     BLDC (Motors)
	 * I2CM-2:
	 *     MPU6050 (gyro/accel)
	 */
	p7brd_init_i2cm(0, 400);
	p7brd_init_i2cm(1, 400);
	p7brd_init_i2cm(2, 400);

	/* Init GPS */
	p7brd_init_uart(1, 0);

	/* Add GPIO interrupts */
	for (i = 0; i < ARRAY_SIZE(mk3_irq_gpios); i++)
		p7_gpio_interrupt_register(mk3_irq_gpios[i]);

	/* Add GPIO interrupt filters */
	for (i = 0; i < ARRAY_SIZE(mk3_irq_gpios_filter) ; i++)
		p7_gpio_filter_interrupt_register(mk3_irq_gpios_filter[i]);

	/* Initialize P7MU */
	drone_common_init_p7mu(mk3_hsis.p7mu_int, 1);

	/* Init PWM */
	if (board == MK3X_MILOS) {
		p7_init_p7pwm(&milos_pwm_pdata, milos_pwm_pins, ARRAY_SIZE(milos_pwm_pins));
		/* PWM leds */
		milos_configure_leds();
		platform_device_register(&milos_leds_pwm);
	}
	else {
		p7_init_p7pwm(&mk3_pwm_pdata, mk3_pwm_pins, ARRAY_SIZE(mk3_pwm_pins));
	}

	/* Init eMMC
	 *     For HW07 revision, use old config
	 *     For newer revision, use default mapping
	 */
	if (mk3_hsis.hwrev < MK3_HW08 && board == MK3X_MK3)
		drone_common_init_sdhci(mk3_hw07_sdhci1_pins,
					ARRAY_SIZE(mk3_hw07_sdhci1_pins));
	else
		drone_common_init_sdhci(NULL, 0);

	/* Init Ultrasound */
	drone_common_init_us_tx();

	/* Init AVI */
	p7_init_avi();

	/* Init cameras
	 *     mt9v117 for vertical camera on CAM2
	 *     mt9f002 for horizontal camera on CAM0
	 */
	drone_common_init_cam_v_mt9v117(mk3_hsis.camera_v_mclk,
					mk3_hsis.camera_v_pwdn);
	drone_common_init_cam_h_mt9f002(mk3_hsis.camera_h_mclk,
					mk3_hsis.camera_h_pwdn, NULL, NULL, 0);

	/* Init MEM2MEM (use default) */
	drone_common_init_m2m(NULL);

	/* Init GPU */
	p7_init_gpu_fb(0, 0, 4);

	/* Init video encoder */
	p7_init_venc();

	/* Init video decoder
	 * As the decoder is used only when streaming thermal data,
	 * the front camera is shut down when using the decoder.
	 * That's why we don't need to allocate extra memory for the decoder.
	 */
	p7_init_vdec();

	/* Init USB */
	drone_common_init_usb(mk3_hsis.host_mode_on, mk3_hsis.host_mode_3v3,
			      mk3_hsis.usb0_oc, 0);

	p7brd_init_usb(1, -1, CI_UDC_DR_HOST);

	/* Init sensors */
	if (board == MK3X_MILOS)
		milos_init_ak8963(AK8963_I2C_BUS,
				mk3_hsis.magneto_int_p7);
	else
		drone_common_init_ak8963(AK8963_I2C_BUS,
				mk3_hsis.magneto_int_p7);
	drone_common_init_inv_mpu6050(MPU6050_I2C_BUS, mk3_hsis.gyro_int_p7,
				      FSYNC_GYRO_FILTER, mk3_hsis.clkin_gyro);
	drone_common_init_ms5607(MS5607_I2C_BUS);

	/* Init BLDC platform data for Milos board */
	if (board == MK3X_MILOS) {
		bldc_cypress_pdata.spin_dir = BLDC_SPIN_DIR_MILOS;
		if (mk3_hsis.pcbrev < MK3_HW02)
			memcpy(bldc_cypress_pdata.lut, bldc_lut_milos_dv,
			       sizeof(bldc_cypress_pdata.lut));
	}

	/* Init BLDC */
	drone_common_init_bldc_with_lut(BLDC_I2C_BUS, mk3_hsis.reset_psoc,
					&bldc_cypress_pdata);

	mykonos3_iio_init();

	/* Init FAN */
	drone_common_init_fan(mk3_hsis.fan);

	/* Export default and custom GPIOs */

	if (board == MK3X_MK3) {
		pr_info("Mykonos3 board : exporting GPIOs\n");
		drone_common_export_gpios(mk3_gpios);
	} else {
		pr_info("Milos board : exporting GPIOs\n");
		drone_common_export_gpios(milos_gpios);
	}

	/* Create sysfs entries (in /sys/kernel/hsis/...) */
	pr_info(
	    "Mykonos3 board : exporting HSIS to userspace in /sys/kernel/hsis");
	drone_common_init_sysfs(mk3_hsis_sysfs);

	register_reboot_notifier(&mykonos3x_rst_notifier);

	/* End of initialization */
	pr_info("Mykonos3 board : init done\n");
}

static void __init mykonos3_init_mach(void)
{
	mykonos3x_init_mach(MK3X_MK3);
}

P7_MACHINE_START(PARROT_MYKONOS3, "Mykonos3 board")
	.reserve        = &mykonos3_reserve_mem,
	.init_machine   = &mykonos3_init_mach,
P7_MACHINE_END

static void __init milos_init_mach(void)
{
	mk3_hsis.reset_gnss = HSIS_HWxx__GNSS_RESET;
	mk3_hsis.super_led_pwm = HSIS_HWxx_SUPER_LED_PWM;
	mykonos3x_init_mach(MK3X_MILOS);
}

P7_MACHINE_START(PARROT_MILOS, "Milos board")
	.reserve        = &mykonos3_reserve_mem,
	.init_machine   = &milos_init_mach,
P7_MACHINE_END
