/**
 * linux/arch/arm/mach-parrot7/board-galilei.c - Galilei board implementation
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Romuald Despres <romuald.despres@parrot.com>
 * date:    Dec 2015
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
#include <mach/p7.h>
#include <mach/pwm.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
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
#include "p7_temperature.h"
#include "p7_pwm.h"

#include "board-drone-common.h"

#include <media/video/avicam.h>


#define GALILEI_BRD_NAME   "galilei"

/*****************************
 * GPIO
 *****************************/

#define GPIO_12V_PWR_GOOD_N	58  /* Exported */
#define GPIO_3V3_CAM_EN		65  /* Exported */
#define GPIO_TOSH_CSN		66  /* No needed to drive it : pull down on P7 */
#define GPIO_1V2_MIPI_EN	67  /* Exported */
#define GPIO_2V8_CAM_EN		68  /* Exported */
#define GPIO_INT_P7MU		72
#define GPIO_1V2_PWR_GOOD	73  /* Exported */
#define GPIO_SD_WP		81
#define GPIO_SD_CD 		82
#define GPIO_FLIR_RESET_N	84
#define GPIO_IMU_INT		91
#define GPIO_PCA_OE_N		120 /* Exported */
#define GPIO_FSYNC_MPU		126 /* Exported */
#define GPIO_CAM_STROBE		128
#define GPIO_FSYNC_IMU		127
#define GPIO_TOSHIBA_RESET_N	132
#define GPIO_PCB_VERSION_00	138
#define GPIO_PCB_VERSION_01	139
#define GPIO_EMMC_RESETN	142

/*****************************
 * PWM
 *****************************/

#define PWM_PCA_CLK		0 /* Exported */
#define PWM_CLK_IN_IMU		2
#define	PWM_THERMAL_PWM_2	3 /* Exported for Peltier module */
#define PWM_LED_GREEN		4
#define PWM_GALILEO_MCLK	5
#define	PWM_THERMAL_PWM_1	6 /* Exported for Peltier module */
#define PWM_TOSH_MCLK		11
#define PWM_LED_RED		12
#define PWM_LED_BLUE		14
#define	PWM_THERMAL_PWM_3	15 /* Exported for Peltier module */
#define	PWM_THERMAL_PWM_4	10 /* Exported for Peltier module */


/*****************************
 * HSIS
 *****************************/

struct galilei_hsis {
	int p7rev;
	int pcbrev;
	int hwrev;
	/* P7MU */
	int p7mu_int;
	/* Camera */
	int camera_3v3_en;
	int mipi_1v2_en;
	int camera_2v8_en;
	int toshiba_reset_n;
	int galileo_mclk;
	int tosh_mclk;
	/* Power good */
	int pwr_12v_good_n;
	int pwr_1v2_good;
	/* Sensors */
	int fsync_mpu;
	int imu_int;
	int fsync_imu;
	int clk_in_imu;
	int flir_reset_n;
	int cam_strobe;
	/* Thermistor */
	int thermal1;
	int thermal2;
	int thermal3;
	int thermal4;
	/* Led */
	int led_green;
	int led_red;
	int led_blue;
	/* Servo */
	int pca_oe_en;
	int pca_clk;
	/* SD */
	int sd_wp;
	int sd_cd;
	/* EMMC */
	int emmc_reset_n;
} static galilei_hsis = {
	/* No default values */
	.p7rev			= -1,
	.pcbrev			= -1,
	.hwrev			= -1,
	/* P7MU */
	.p7mu_int		= GPIO_INT_P7MU,
	/* Camera */
	.camera_3v3_en		= GPIO_3V3_CAM_EN,
	.mipi_1v2_en		= GPIO_1V2_MIPI_EN,
	.camera_2v8_en		= GPIO_2V8_CAM_EN,
	.toshiba_reset_n	= GPIO_TOSHIBA_RESET_N,
	.galileo_mclk		= PWM_GALILEO_MCLK,
	.tosh_mclk		= PWM_TOSH_MCLK,
	/* Power good */
	.pwr_12v_good_n		= GPIO_12V_PWR_GOOD_N,
	.pwr_1v2_good		= GPIO_1V2_PWR_GOOD,
	/* Sensors */
	.fsync_mpu		= GPIO_FSYNC_MPU,
	.imu_int		= GPIO_IMU_INT,
	.fsync_imu		= GPIO_FSYNC_IMU,
	.clk_in_imu		= PWM_CLK_IN_IMU,
	.flir_reset_n		= GPIO_FLIR_RESET_N,
	.cam_strobe		= GPIO_CAM_STROBE,
	/* Thermistor */
	.thermal1		= PWM_THERMAL_PWM_1,
	.thermal2		= PWM_THERMAL_PWM_2,
	.thermal3		= PWM_THERMAL_PWM_3,
	.thermal4		= PWM_THERMAL_PWM_4,
	/* Led */
	.led_green		= PWM_LED_GREEN,
	.led_red		= PWM_LED_RED,
	.led_blue		= PWM_LED_BLUE,
	/* Servo */
	.pca_oe_en		= GPIO_PCA_OE_N,
	.pca_clk		= PWM_PCA_CLK,
	/* SD */
	.sd_wp			= GPIO_SD_WP,
	.sd_cd			= GPIO_SD_CD,
	/* EMMC */
	.emmc_reset_n		= GPIO_EMMC_RESETN,
};

/***************
 * GPIO export *
 ***************/

#define GALILEI_GPIO(gpio, flags, label)\
	DRONE_COMMON_GPIO(&galilei_hsis.gpio, flags, label)

struct drone_common_gpio galilei_gpios[] = {
	GALILEI_GPIO(camera_3v3_en, GPIOF_OUT_INIT_HIGH, "3v3_cam_en"),
	GALILEI_GPIO(mipi_1v2_en, GPIOF_OUT_INIT_HIGH, "1v2_mipi_en"),
	GALILEI_GPIO(camera_2v8_en, GPIOF_OUT_INIT_HIGH, "2v8_cam_en"),
	GALILEI_GPIO(pwr_12v_good_n, GPIOF_IN, "12v_pwr_good_n"),
	GALILEI_GPIO(pwr_1v2_good,  GPIOF_IN, "1v2_pwr_good"),
	GALILEI_GPIO(fsync_mpu, GPIOF_IN, "Fsync_mpu"),
	GALILEI_GPIO(pca_oe_en, GPIOF_OUT_INIT_HIGH, "PCA_OE_en"), // Deactivate PCA, PWM from P7 directly used
	{ .gpio = NULL, }
};

/***************
 * GPIO keys *
 ***************/

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

static struct gpio_keys_button p7_gpio_keys[] = {
/*      GPIO                               Code      Active low  Description */
	RX_KEYS_GPIO(P7_GPIO_NR(GPIO_CAM_STROBE), KEY_UP,   false,      "Galileo2 FSTROBE line"),
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

/**************
 * HSIS Sysfs *
 **************/

#define GALILEI_HSIS_SYSFS_ATTR(x) DRONE_COMMON_HSIS_SYSFS_ATTR(x, &galilei_hsis.x)
struct drone_common_hsis_sysfs_attr galilei_hsis_sysfs[] = {
	/* gpio */
	GALILEI_HSIS_SYSFS_ATTR(camera_3v3_en),
	GALILEI_HSIS_SYSFS_ATTR(mipi_1v2_en),
	GALILEI_HSIS_SYSFS_ATTR(camera_2v8_en),
	GALILEI_HSIS_SYSFS_ATTR(toshiba_reset_n),
	GALILEI_HSIS_SYSFS_ATTR(pwr_12v_good_n),
	GALILEI_HSIS_SYSFS_ATTR(pwr_1v2_good),
	GALILEI_HSIS_SYSFS_ATTR(fsync_mpu),
	GALILEI_HSIS_SYSFS_ATTR(imu_int),
	GALILEI_HSIS_SYSFS_ATTR(fsync_imu),
	GALILEI_HSIS_SYSFS_ATTR(flir_reset_n),
	GALILEI_HSIS_SYSFS_ATTR(pca_oe_en),
	GALILEI_HSIS_SYSFS_ATTR(cam_strobe),
	/* pwm */
	GALILEI_HSIS_SYSFS_ATTR(galileo_mclk),
	GALILEI_HSIS_SYSFS_ATTR(tosh_mclk),
	GALILEI_HSIS_SYSFS_ATTR(clk_in_imu),
	GALILEI_HSIS_SYSFS_ATTR(thermal1),
	GALILEI_HSIS_SYSFS_ATTR(thermal2),
	GALILEI_HSIS_SYSFS_ATTR(thermal3),
	GALILEI_HSIS_SYSFS_ATTR(thermal4),
	GALILEI_HSIS_SYSFS_ATTR(led_green),
	GALILEI_HSIS_SYSFS_ATTR(led_red),
	GALILEI_HSIS_SYSFS_ATTR(led_blue),
	GALILEI_HSIS_SYSFS_ATTR(pca_clk),
	/* rev */
	GALILEI_HSIS_SYSFS_ATTR(p7rev),
	GALILEI_HSIS_SYSFS_ATTR(hwrev),
	GALILEI_HSIS_SYSFS_ATTR(pcbrev),
	{ .value = NULL, }
};

/*****************************
 * I2C
 *****************************/
#define I2C_ADDR_GALILEO2 0x10
#define I2C_ADDR_TOSHIBA  0x0e
#define I2C_ADDR_LEPTON   0x2A

#if 0
/*
** This parts could be used to drive pca9685 pwm generator
** It is need whereas the chip is on muxed i2c_2 b
** No need for now : PWM_PCA_CLK is used instead
*/

#include <i2c/muxes/p7-i2cmux.h>
#include "i2cm.h"

/* The alternate buses. There will be one additional "default" bus.*/
static char const* const  galilei_i2cm2_mux_names[] = {
	"mux-b"
};

static unsigned long galilei_i2cm2_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};


static struct pinctrl_map galilei_i2cm2_muxb_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKb),
	P7_INIT_PINCFG(P7_I2C_2_CLKb, galilei_i2cm2_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DATb),
	P7_INIT_PINCFG(P7_I2C_2_DATb, galilei_i2cm2_pinconf),
};

static struct p7i2cmux_pins galilei_i2cm2_mux_pins[] __initdata = {
	{
		.pinmap = galilei_i2cm2_muxb_pins,
		.sz     = ARRAY_SIZE(galilei_i2cm2_muxb_pins)
	}
};

static struct p7i2cmux_plat_data galilei_i2cm2_mux_pdata = {
	.channel_names = galilei_i2cm2_mux_names,
	.nr_channels   = ARRAY_SIZE(galilei_i2cm2_mux_names)
};
#endif

static struct p7pwm_conf galilei_conf_cam_mclk = {
	/* Precision for mt9f002 */
	.period_precision = 5,
	/* Not used in clock mode */
	.duty_precision = 0,
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf galilei_conf_pwm_leds = {
	.period_precision = 5,
	.duty_precision   = 5,
	.mode             = P7PWM_MODE_NORMAL,
};

static struct p7pwm_conf galilei_conf_pwm_thermal = {
	.period_precision = 5,
	/* Not used in clock mode */
	.duty_precision	  = 0,
	.mode             = P7PWM_MODE_NORMAL,
};

static struct p7pwm_conf galilei_conf_pca_clk = {
	.period_precision = 5,
	/* Not used in clock mode */
	.duty_precision = 0,
	.mode = P7PWM_MODE_NORMAL,
};

static struct p7pwm_conf galilei_conf_clk_in_imu = {
	.period_precision = 5,
	/* Not used in clock mode */
	.duty_precision = 0,
	.mode = P7PWM_MODE_CLOCK,
};

static unsigned long galilei_pwm_pinconfig[] = {
	P7CTL_DRV_CFG(0),      /* Drive strength 0 (reg=1) */
};

static struct pinctrl_map galilei_pwm_pins[] __initdata = {
	/* Galileo 2 mclk */
	P7_INIT_PINMAP(P7_PWM_05),
	/* Toshiba bridge mclk */
	P7_INIT_PINMAP(P7_PWM_11),
	/* Green led */
	P7_INIT_PINMAP(P7_PWM_04),
	P7_INIT_PINCFG(P7_PWM_04, galilei_pwm_pinconfig),
	/* Red led */
	P7_INIT_PINMAP(P7_PWM_12),
	P7_INIT_PINCFG(P7_PWM_12, galilei_pwm_pinconfig),
	/* Blue led */
	P7_INIT_PINMAP(P7_PWM_14),
	P7_INIT_PINCFG(P7_PWM_14, galilei_pwm_pinconfig),
	/* Thermal 1 */
	P7_INIT_PINMAP(P7_PWM_06),
	/* Thermal 2 */
	P7_INIT_PINMAP(P7_PWM_03),
	/* Thermal 3 */
	P7_INIT_PINMAP(P7_PWM_15),
	/* Thermal 4 */
	P7_INIT_PINMAP(P7_PWM_10),
	/* PCA clk*/
	P7_INIT_PINMAP(P7_PWM_00),
	/* Clk in IMU*/
	P7_INIT_PINMAP(P7_PWM_02),
};

static struct p7pwm_pdata galilei_pwm_pdata = {
	.conf = {
		[PWM_GALILEO_MCLK]  = &galilei_conf_cam_mclk,
		[PWM_TOSH_MCLK]     = &galilei_conf_cam_mclk,
		[PWM_LED_GREEN]     = &galilei_conf_pwm_leds,
		[PWM_LED_RED]       = &galilei_conf_pwm_leds,
		[PWM_LED_BLUE]      = &galilei_conf_pwm_leds,
		[PWM_THERMAL_PWM_1] = &galilei_conf_pwm_thermal,
		[PWM_THERMAL_PWM_2] = &galilei_conf_pwm_thermal,
		[PWM_THERMAL_PWM_3] = &galilei_conf_pwm_thermal,
		[PWM_THERMAL_PWM_4] = &galilei_conf_pwm_thermal,
		[PWM_PCA_CLK]       = &galilei_conf_pca_clk,      // Exported, used in place of pca9685
		[PWM_CLK_IN_IMU]    = &galilei_conf_clk_in_imu,
        }
};

/*****************************
 * Galileo2 cam
 * Bridge tc358746a
 *****************************/

#include <media/galileo2.h>
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
};

/* Configure a PWM to output a clock signal with a frequency given in
 * megaherz */
static struct pwm_device *galileo2_pwm_clock(int pwm,
					     const char *label,
					     unsigned freq_hz)
{
	struct pwm_device *pwm_dev;
	unsigned period_ns = 1000000000/freq_hz;
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


#define GALILEO2_TC358746A_MCLK_HZ 8928571

static struct pwm_device *galileo2_tc358746a_pwm = NULL;

static int galileo2_tc358746a_set_power(int on)
{
	if (on) {
		galileo2_tc358746a_pwm =
			galileo2_pwm_clock(P7_PWM_NR(galilei_hsis.tosh_mclk),
					   "bridge mclk",
					   GALILEO2_TC358746A_MCLK_HZ);
	} else {
		galileo2_pwm_free(galileo2_tc358746a_pwm);
	}

	if (IS_ERR(galileo2_tc358746a_pwm)) {
		return PTR_ERR(galileo2_tc358746a_pwm);
	}

	gpio_set_value(P7_GPIO_NR(galilei_hsis.toshiba_reset_n), !!on);

	return 0;
}

#define GALILEO2_CAM_MCLK_HZ 8928571
static struct pwm_device *galileo2_cam_pwm = NULL;

static int galileo2_cam_set_power(int on)
{
	if (on) {
		galileo2_cam_pwm =
			galileo2_pwm_clock(P7_PWM_NR(galilei_hsis.galileo_mclk),
					   "galileo2 mclk",
					   GALILEO2_CAM_MCLK_HZ);
	} else {
		galileo2_pwm_free(galileo2_cam_pwm);
	}

	if (IS_ERR(galileo2_cam_pwm)) {
		return PTR_ERR(galileo2_cam_pwm);
	}

	msleep(10);

	return 0;
}

#define GALILEO2_NB_LANES 2

struct galileo2_platform_data galileo2_cam_pdata = {
	.set_power = galileo2_cam_set_power,
	.refclk = GALILEO2_CAM_MCLK_HZ,
	.lanes = GALILEO2_NB_LANES,
};

static struct i2c_board_info galileo2_cam_i2c_device = {
	I2C_BOARD_INFO("galileo2", I2C_ADDR_GALILEO2),
	.platform_data = &galileo2_cam_pdata,
};

static struct avicam_subdevs galileo2_cam_subdevs[] = {
	{
		.board_info     = &galileo2_cam_i2c_device,
		.i2c_adapter_id = 2,
		.subdevs        = NULL,
	},
	{ NULL, 0 },
};

#if CONFIG_VIDEO_MAX14574
/*****************************
 * LENS Driver
 *****************************/

#include <media/max14574.h>

static struct max14574_platform_data galileo2_max14574_subdev_pdata = {
	.set_power = NULL,
	/* normal operation */
	.mode = (USERMODE_ACTIVE | USERMODE_SM),
	/* Check if chip is OK before initialization */
	.control =  CMND_CHK_FAIL,
	.voltage = CHARGE_MAX_VOLT_MIN,
};

static struct i2c_board_info galileo2_max14574_i2c_device = {
	I2C_BOARD_INFO(MAX14574_NAME, MAX14574_I2C_ADDR),
	.platform_data = &galileo2_max14574_subdev_pdata,
};
#endif

static struct tc358746a_platform_data galileo2_tc358746a_subdev_pdata = {
	.set_power            = &galileo2_tc358746a_set_power,
	.refclk               = GALILEO2_TC358746A_MCLK_HZ,
	/* Do we need that? */
	/*.force_subdev_pixcode = V4L2_MBUS_FMT_SBGGR10_1X10,*/
	.lanes                = GALILEO2_NB_LANES,
	.calibration_delay_ms = 50,
	.phytimdly            = 27,
};

static struct i2c_board_info galileo2_tc358746a_i2c_device = {
	I2C_BOARD_INFO("tc358746a", I2C_ADDR_TOSHIBA),
	.platform_data = &galileo2_tc358746a_subdev_pdata,
};

static struct avicam_subdevs galileo2_tc358746a_subdevs[] = {
	{
		.board_info     = &galileo2_tc358746a_i2c_device,
		.i2c_adapter_id = 1,
		.subdevs = galileo2_cam_subdevs,
	},
#if CONFIG_VIDEO_MAX14574
	{
		.board_info     = &galileo2_max14574_i2c_device,
		.i2c_adapter_id = 2,
		.subdevs = NULL,
	},
#endif
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
	.enable_stats       = 1,
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
	.vb2_cache_flags    = VB2_CACHE_DMA_CONTIG | VB2_CACHE_WRITETHROUGH,
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

/*****************************
 * FLIR
 *****************************/

#if 0

/*
** FLIR is not used for now
*/

static struct pinctrl_map galilei_spi_slave_flir_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_00), /* CLK */
	P7_INIT_PINMAP(P7_SPI_02), /* MISO */
	P7_INIT_PINMAP(P7_SPI_03), /* SS */
};

static struct p7spi_swb const galilei_spi_master_flir_swb[] = {
	P7SPI_INIT_SWB( 0,  P7_SWB_DIR_OUT,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB( 2,  P7_SWB_DIR_IN,   P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB( 3,  P7_SWB_DIR_OUT,  P7_SWB_SPI_SS),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data galilei_spi_master_flir_cdata = {
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

static struct lepton_i2c_platform_data flir_i2c_pdata = {
	.gpio_pwr = -1,
	.gpio_rst = GPIO_FLIR_RESET_N,
};

static struct lepton_platform_data flir_pdata = {
	.board_info = {
		I2C_BOARD_INFO("lepton-ctrl", I2C_ADDR_LEPTON),
		.platform_data = &flir_i2c_pdata,
	},
	.i2c_adapter_nr = 2,
};

static P7_DECLARE_SPIM_SLAVE(galilei_flir_info,
		"lepton",
		&flir_pdata,
		&galilei_spi_master_flir_cdata,
		10 * 1000 * 1000,
		SPI_MODE_3);

#endif

/*****************************
 * PCA9685 12 bits PWM generator
 *****************************/

#if 0
/*
** This parts could be used to drive pca9685 pwm generator
** No need for now : PWM_PCA_CLK is used instead
** Need to import the pca9685 driver
*/

static struct i2c_board_info pca9685_i2c_device = {
	I2C_BOARD_INFO("pca9685", 0x40),
	.platform_data = NULL,
};
#endif

/*****************************
 * MPU6000
 *****************************/

/*
** We export the spidev
** For now the MPU6000 is driven in user space
*/

static struct pinctrl_map galilei_spi_slave_mpu6000_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_08), /* SS */
	P7_INIT_PINMAP(P7_SPI_09), /* CLK */
	P7_INIT_PINMAP(P7_SPI_10), /* MOSI */
	P7_INIT_PINMAP(P7_SPI_11), /* MISO */
};

static struct p7spi_swb const galilei_spi_master_mpu6000_swb[] = {
	P7SPI_INIT_SWB(  8,  P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(  9,  P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB( 10,  P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB( 11,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data galilei_spi_mpu6000_cdata = {
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

/* Equivalent of the P7_DECLARE_SPIS_MASTER macro but with more parameters exposed */
static struct spi_board_info galilei_mpu6000_spi_dev = {
		.modalias           = "spidev",
		.platform_data      = NULL,
		.controller_data    = &galilei_spi_mpu6000_cdata,
		.irq                = -1,
		.max_speed_hz       = 8 * 1000 * 1000,
		.chip_select        = 0,
		.mode               = SPI_MODE_3
	};

#define MPU6000_PWM_PERIOD_NS 31510

static int galilei_enable_clock_mpu6000(int clkin_pwm)
{
	struct pwm_device *mpu6000_pwm_device;
	int ret = 0;

	/* check pwm pin is valid */
	if (!gpio_is_valid(clkin_pwm))
		return -EINVAL;

	/* Request PWM */
	mpu6000_pwm_device = pwm_request(clkin_pwm, "MPU6000 clk");
	if (IS_ERR(mpu6000_pwm_device)) {
		ret = PTR_ERR(mpu6000_pwm_device);
		goto err_alloc;
	}

	/* Configure PWM */
	ret = pwm_config(mpu6000_pwm_device, 0, MPU6000_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	/* Enable PWM */
	ret = pwm_enable(mpu6000_pwm_device);
	if (ret)
		goto err_config;

	return 0;

err_config:
	pwm_free(mpu6000_pwm_device);
err_alloc:
	pr_warn("failed to set clock for mpu6000 chip\n");
	return ret;
}

static int galilei_init_spi_slave_mpu6000(
					  int spi_bus,
					  struct spi_board_info *info,
					  const char *desc,
					  int irq_type)
{
	int gpio;
	int err;

	if (irq_type != P7_I2C_NOIRQ) {
		gpio = info->irq;

		if (!gpio_is_valid(gpio)) {
			pr_err("%s : Invalid gpio %d\n", __func__, gpio);
			return -EINVAL;
		}

		if (irq_type == P7_I2C_IRQ) {
			err = gpio_request(gpio, desc);
			if (err) {
				pr_err("%s : couldn't request GPIO %d "
				       "for %s [%d]\n", __func__,
				       gpio, desc, err);
				return -EINVAL;
			}
			p7_gpio_interrupt_register(gpio);
			info->irq = gpio_to_irq(gpio);
		} else if (irq_type == P7_I2C_SHAREDIRQ) {

			info->irq = gpio_to_irq(gpio);

			if (info->irq < 0) {
				pr_err("%s : couldn't find an IRQ for gpio %d\n",
				       __func__, gpio);
				gpio_free(gpio);
				return -EINVAL;
			}
		} else {
			pr_err("%s : invalid i2c irq mode\n", __func__);
			return -EINVAL;
		}
	}

	pr_info("Registered interrupt GPIO %d for SPI device %s",gpio,desc);
	p7_init_spim_slave(spi_bus, info);

	return 0;
}


/*****************************
 * Board revision
 *****************************/

static int __init galilei_board_get_rev(void)
{
	static int board_rev = -1;
	int gpios[] = {GPIO_PCB_VERSION_00, GPIO_PCB_VERSION_01};
	int i;

	if (board_rev != -1)
		return board_rev;

	board_rev = 0;
	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		int err, val;
		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
			0,
			"galilei rev");
		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			board_rev |= (val << i);
			gpio_free(P7_GPIO_NR(gpios[i]));
		}
	}
	return board_rev;
}

/*****************************
 * P7MU Power Management Unit
 *****************************/

#include "p7mu.h"
#include <mfd/p7mu.h>
#include <mach/p7-adc.h>
#include <mach/gpio.h>
#include <spi/p7-spim.h>

static struct pinctrl_map galilei_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

/* External temperature sensor */
static struct p7_temp_chan p7mu_adc_channels[] = {
	/* 3.3V ref */
	{
		.channel = 2,
		.freq = 160000,
		.name = "ref",
	},
	/* THERMISTOR */
	{
		.channel = 1,
		.freq = 160000,
		.name = "thermistor",
	},
};

static struct p7_temp_chan_data p7mu_adc_chan_data = {
        .channels               = p7mu_adc_channels,
        .num_channels           = ARRAY_SIZE(p7mu_adc_channels),
        .temp_mode              = P7_TEMP_FC7100_HW08,
};

static struct p7mu_plat_data galilei_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(GPIO_INT_P7MU), /* P7 interrupt source */
	.int_32k    = false,                     /* External 32kHz clock. */
	.int_32m    = true,                      /* No External 48mHz clock. */
};

/* Reuse the fc7100 driver since it's the same config. We should really rename
 * that */
static struct platform_device galilei_temp_device = {
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

static unsigned long galilei_sdhci_pins_config_clk[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
	P7CTL_SLR_CFG(2),
};

static unsigned long galilei_sdhci_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
	P7CTL_SLR_CFG(2),
};

static struct pinctrl_map galilei_sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, galilei_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, galilei_sdhci_pins_config)
};

static struct acs3_plat_data galilei_sdhci0_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = GPIO_SD_WP,                       /* write protect status */
	.cd_gpio    = GPIO_SD_CD,                       /* card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

static struct acs3_plat_data galilei_sdhci1_pdata = {
	.led_gpio   = -1,                                           /* No activity led */
	.wp_gpio    = -1,                                           /* No write protect */
	.cd_gpio    = -1,                                           /* No card detect */
	.rst_gpio   = P7_GPIO_NR(GPIO_EMMC_RESETN),
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |               /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,                         /* emmc is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE|MMC_CAP2_CACHE_CTRL,  /* bus voltage is fixed in hardware */
};

/***********************
 * LEDS
 ***********************/

static void galilei_configure_leds(void)
{
	struct pwm_device *pwm;

	pwm = pwm_request(galilei_hsis.led_green, "galilei BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 50000, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(galilei_hsis.led_red, "galilei BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 1000000, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(galilei_hsis.led_blue, "galilei BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 0, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}
}

static struct led_pwm pwm_leds[] = {
	{
		.name = "galilei:green",
		.default_trigger = "none",
		.pwm_id = PWM_LED_GREEN,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "galilei:red",
		.default_trigger = "none",
		.pwm_id = PWM_LED_RED,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "galilei:blue",
		.default_trigger = "none",
		.pwm_id = PWM_LED_BLUE,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
};

static struct led_pwm_platform_data pwm_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device galilei_leds_pwm = {
	.name = "leds_pwm",
	.id = -1,
	.dev = {
		.platform_data = &pwm_data,
	},
};

/***********************
 * Memory
 ***********************/

static void __init galilei_reserve_mem(void)
{
	drone_common_reserve_mem_ramoops();

#define GALILEI_HX280_SIZE (CONFIG_ARCH_PARROT7_GALILEI_HX280_SIZE * SZ_1M)

	p7_reserve_avicammem(&galileo2_avicam_dev,
			     GALILEO2_CAM_RAM_SIZE);

	p7_reserve_vencmem(GALILEI_HX280_SIZE);

	p7_reserve_usb_mem(0);

	p7_reserve_dmamem();
}

/***********************
 * Interrupts handling *
 ***********************/

/* p7gpio_filter_phase  */
#define FSYNC_GYRO_FILTER 5
static struct p7gpio_filter_phase galilei_irq_gpios_filter[] = {
	{
		.start		= GPIO_FSYNC_IMU,
		.stop		= GPIO_IMU_INT,
		.filter		= FSYNC_GYRO_FILTER,
		.mode		= GPIO_MEASURE_STOP,
		.export_reset 	= 1
	}
};

static unsigned const galilei_irq_gpios[] = {
	GPIO_CAM_STROBE,  /* FSTROBE interrupt source */
};

/***********************
 * Init
 ***********************/

static void __init galilei_init_mach(void)
{
	int i;

	/* Initialize ramoops */
	drone_common_init_ramoops();

	p7_init_mach();

	/* GPIO init */
	p7_init_gpio(galilei_irq_gpios,
		     ARRAY_SIZE(galilei_irq_gpios));

	galilei_hsis.p7rev  = p7_chiprev() + 1;
	galilei_hsis.pcbrev = galilei_board_get_rev();
	galilei_hsis.hwrev  = 1;

	pr_info("galilei p7rev %d\n", galilei_hsis.p7rev);
	pr_info("galilei rev %d\n", galilei_hsis.pcbrev);

	/* Remove pull-down from STROBE pin */
	p7_config_pin(galilei_hsis.cam_strobe, P7CTL_PUD_CFG(HIGHZ));

	/* GPIO keys */
	p7_init_dev(&p7_gpio_keys_device,
		    &p7_gpio_keys_platform_data, NULL, 0);

	/* debug uart */
	p7brd_init_uart(0,0);

	/* eBee autopilot uart */
	p7brd_init_uart(1,0);

	/*
	 * i2c-0: P7MU
	 */
	p7brd_init_i2cm(0, 400);

	/*
	 * i2c-1: tc358746a
	 */
	p7brd_init_i2cm(1, 400);

	/*
	 * i2c-2: galileo2 cam
	          pca
		  lepton3
	 */
#if 1
	p7brd_init_i2cm(2, 400);
#else
	/*
	** This parts could be used to drive pca9685 pwm generator
	** It is need whereas the chip is on muxed i2c_2 b
	** No need for now : PWM_PCA_CLK is used instead
	*/

	p7brd_init_i2cm_muxed(2,
			      400,
			      NULL,
			      0,
			      &galilei_i2cm2_mux_pdata,
			      galilei_i2cm2_mux_pins);
#endif

	/* p7mu */
	p7_gpio_interrupt_register(galilei_p7mu_pdata.gpio);
	p7_init_p7mu(0,
		     &galilei_p7mu_pdata,
		     galilei_p7mu_pins,
		     ARRAY_SIZE(galilei_p7mu_pins));

	/* Add GPIO interrupt filters */
	for (i = 0; i < ARRAY_SIZE(galilei_irq_gpios_filter) ; i++)
		p7_gpio_filter_interrupt_register(galilei_irq_gpios_filter[i]);

	/* P7MU/ADC */
	p7_init_temperature();

	platform_device_register(&galilei_temp_device);

	/* PWM */
	p7_init_p7pwm(&galilei_pwm_pdata,
		      galilei_pwm_pins,
		      ARRAY_SIZE(galilei_pwm_pins));


	/* emmc */
	p7brd_init_sdhci(1, &galilei_sdhci1_pdata, NULL, NULL, NULL,
			 galilei_sdhci1_pins, ARRAY_SIZE(galilei_sdhci1_pins));


	/* SD */
	p7_gpio_interrupt_register(galilei_sdhci0_pdata.cd_gpio);
	p7brd_init_sdhci(0, &galilei_sdhci0_pdata, NULL, NULL,
			 NULL, NULL, 0);

	p7brd_init_udc(0, -1);

	p7_init_venc();

#if 0
	/*
	** FLIR is not used for now
	*/

	/* FLIR */


	p7_init_spim(1, galilei_spi_slave_flir_pins,
		     ARRAY_SIZE(galilei_spi_slave_flir_pins),
		     galilei_spi_master_flir_swb);

	p7_init_spim_slave(1, &galilei_flir_info);
#endif

#if 0
	/*
	** This parts could be used to drive pca9685 pwm generator
	** No need for now : PWM_PCA_CLK is used instead
	** Need to import the pca9685 driver
	*/

	/* pca9685 */
	parrot_init_i2c_slave(20 /*1*/,
			      &pca9685_i2c_device,
			      "pca9685",
			      P7_I2C_NOIRQ);
#endif

#if 0
	/*
	** To manage the MPU6000
	** We need spi driver
	** For now not available and not needed :
	** This chip is driven in user space
	*/

	/* MPU6000 */
	drone_common_init_inv_mpu6000(-1,
				      galilei_hsis.imu_int,
				      FSYNC_GYRO_FILTER,
				      galilei_hsis.clk_in_imu);

#else
	/* MPU6000 */
	galilei_mpu6000_spi_dev.irq = P7_GPIO_NR(galilei_hsis.imu_int);

	galilei_enable_clock_mpu6000(P7_PWM_NR(galilei_hsis.clk_in_imu));

	p7_init_spim(0,
		     galilei_spi_slave_mpu6000_pins,
		     ARRAY_SIZE(galilei_spi_slave_mpu6000_pins),
		     galilei_spi_master_mpu6000_swb);

	galilei_init_spi_slave_mpu6000(0,
				       &galilei_mpu6000_spi_dev,
				       "IMU",
				       P7_I2C_IRQ);

#endif

	/* Toshiba bridge (tc358746a) */
	gpio_request_one(P7_GPIO_NR(galilei_hsis.toshiba_reset_n),
			 GPIOF_OUT_INIT_LOW,
			 "Toshiba_reset_n");


	/* galileo2 */
	p7_init_avicam(&galileo2_avicam_dev,
		       &galileo2_avicam_pdata,
		       galileo2_avicam_pins,
		       ARRAY_SIZE(galileo2_avicam_pins));

	galilei_configure_leds();

	/* pwn leds */
	platform_device_register(&galilei_leds_pwm);

	/* Toshiba bridge (tc358746a) */

	/* Exported gpios :
	** 3v3 cam en
	** 1v2 mipi en
	** 2v8 cam en
	** 12v pwr good n
	** 1v2 pwr good
	** Fsync mpu
	** PCA_OE_en
	*/
	drone_common_export_gpios(galilei_gpios);

	/* Create sysfs entries (in /sys/kernel/hsis/...) */
	pr_info(
	    "Galilei board : exporting HSIS to userspace in /sys/kernel/hsis");
	drone_common_init_sysfs(galilei_hsis_sysfs);

}

P7_MACHINE_START(PARROT_GALILEI, "galilei board")
  .reserve        = &galilei_reserve_mem,
  .init_machine   = &galilei_init_mach,
P7_MACHINE_END
