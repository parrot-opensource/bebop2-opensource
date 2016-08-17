/**
 * linux/arch/arm/mach-parrot7/paros-board.c - Parrot7 Paros platform
 *                                                implementation
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Christian ROSALIE <christian.rosalie@parrot.com>
 * date:    17-Oct-2014
 *
 * This file is released under the GPL
 * Copyright (C) 2014 Parrot S.A.
 */
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <mach/ether.h>
#include <video/avi.h>
#include <spi/p7-spi.h>
#include <spi/p7-spim.h>
#include <i2c/p7-i2cm.h>
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

/******
 * GPIO
 ******/

/* P7 GPIOs */
//#define CSB_DAC_WLAN		71
#define CLR_DAC_WLAN		72

#define INT_P7MU		73
#define NREBOOT_P7MU		94
#define INT_ETH			136
#define NRESET_ETH		137
#define IRQ_DAC_WLAN		174
#define LDAC_DAC_WLAN		175
#define HOST_MODE_3V3		126
#define HOST_MODE_ON		127
#define USB0_OC			128
#define _3V3_WLAN_EN		134
#define _1V2_WLAN_EN		135
#define LED_R_P7		160
#define LED_G_P7		161
#define VENTILLO_EN		213
#define	CAM_PWR_EN		214
#define	HDMI_IN_NRST		215
#define	HDMI_IN_INT		216
#define	HDMI_IN_CLK		217

#define	PSOC_ON			6
#define	FAULT			7
#define	PD_RESET		8
#define	INT_MAGNETO_0		9
#define	INT_MAGNETO_1		10
#define	GYRO_INT		11
#define	SELECT_ALIM_US_TOP	12
#define	SELECT_ALIM_US_BOTTOM	13
#define	USER_ON_OFF		14


/* P7MU GPIOs */
#define _3V3_KALAMOS_PWRON	5
/*
 IO_5_P7MU	3V3_KALAMOS_PWRON
 IO_3_P7MU	WAKE_UP_WL
 IO_2_P7MU      IO_2_P7MU/RESET_DDR3_GPIO
*/

struct gpio_setting {
	int gpio;
	char *name;
	int default_value;
	int interrupt;
	int bidir;
};

#define P7_GPIO_SETTING(nbr, id, val, it, bdir)	\
	{	.gpio = P7_GPIO_NR(nbr), \
		.name = id, \
		.default_value = val, \
		.interrupt = it, \
		.bidir = bdir, }

#define P7MU_GPIO_SETTING(nbr, id, val, it, bdir) \
	{	.gpio = P7MU_IO_NR(nbr), \
		.name = id, \
		.default_value = val, \
		.interrupt = it, \
		.bidir = bdir, }
	
struct gpio_setting paros_gpios[] __initdata = {

	P7_GPIO_SETTING(CLR_DAC_WLAN, "clr_dac_wlan",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(INT_ETH, "int_eth",
			GPIOF_DIR_IN, 1, 0),
	P7_GPIO_SETTING(IRQ_DAC_WLAN, "irq_dac_wlan",
			GPIOF_DIR_IN, 1, 0),
	P7_GPIO_SETTING(LDAC_DAC_WLAN, "ldac_dac_wlan",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(HOST_MODE_3V3, "host_mode_3v3",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(USB0_OC, "usb0_oc",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(_3V3_WLAN_EN, "3v3_wlan_en",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7_GPIO_SETTING(_1V2_WLAN_EN, "1v2_wlan_en",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7_GPIO_SETTING(LED_R_P7, "led_r_p7",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(LED_G_P7, "led_g_p7",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(VENTILLO_EN, "ventillo_en",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7_GPIO_SETTING(CAM_PWR_EN, "cam_pwr_en",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7_GPIO_SETTING(HDMI_IN_NRST, "hdmi_in_nrst",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7_GPIO_SETTING(HDMI_IN_INT, "hdmi_in_int",
			GPIOF_DIR_IN, 1, 0),
	P7_GPIO_SETTING(HDMI_IN_CLK, "hdmi_in_clk",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(PSOC_ON, "psoc_on",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7_GPIO_SETTING(FAULT, "fault",
			GPIOF_DIR_IN, 0, 0),
	P7_GPIO_SETTING(INT_MAGNETO_0, "int_magneto_0",
			GPIOF_DIR_IN, 1, 0),
	P7_GPIO_SETTING(INT_MAGNETO_1, "int_magneto_1",
			GPIOF_DIR_IN, 1, 0),
	P7_GPIO_SETTING(GYRO_INT, "gyro_int",
			GPIOF_DIR_IN, 1, 0),
	P7_GPIO_SETTING(SELECT_ALIM_US_TOP, "select_alim_us_top",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(SELECT_ALIM_US_BOTTOM, "select_alim_us_bottom",
			GPIOF_OUT_INIT_LOW, 0, 0),
	P7_GPIO_SETTING(USER_ON_OFF, "user_on_off",
			GPIOF_OUT_INIT_HIGH, 0, 0),
	P7MU_GPIO_SETTING(_3V3_KALAMOS_PWRON, "3v3_kalamos_pwron",
			GPIOF_OUT_INIT_HIGH, 0, 0),
};


/******
 * PWM
 ******/
#include "p7_pwm.h"
#define	CAM_H_MCLK	0
#define	CAM_V_MCLK	1
#define	CAM_FPV_1_MCLK	2
#define	CAM_FPV_2_MCLK	3
#define	CLKIN_GYRO	4
#define	PWM_US_BOTTOM	5
#define	THERMAL_PWM	6

static struct p7pwm_conf paros_cam_h_mclk = {
	.period_precision = 5,		/* precision for MT9V117 */
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf paros_cam_v_mclk = {
	.period_precision = 5,		/* precision for MT9F001 */
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf paros_cam_fpv_1_mclk = {
	.period_precision = 5,
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf paros_cam_fpv_2_mclk = {
	.period_precision = 5,
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf paros_clkin_gyro = {
	.period_precision = 5,
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf paros_pwm_us_bottom = {
	.period_precision = 5,
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf paros_thermal_pwm = {
	.period_precision = 5,
	.duty_precision	  = 0,		/* Not used in clock mode */
	.mode             = P7PWM_MODE_CLOCK,
};

static struct p7pwm_pdata paros_default_pwm_pdata = {
	.conf = {
		[CAM_H_MCLK]     = &paros_cam_h_mclk,
		[CAM_V_MCLK]     = &paros_cam_v_mclk,
		[CAM_FPV_1_MCLK] = &paros_cam_fpv_1_mclk,
		[CAM_FPV_2_MCLK] = &paros_cam_fpv_2_mclk,
		[CLKIN_GYRO]     = &paros_clkin_gyro,
		[PWM_US_BOTTOM]  = &paros_pwm_us_bottom,
		[THERMAL_PWM]    = &paros_thermal_pwm,
	}
};

static struct pinctrl_map paros_default_pwm_pins[] __initdata = {
		P7_INIT_PINMAP(P7_PWM_00), /* CAM_H_MCLK     */
		P7_INIT_PINMAP(P7_PWM_01), /* CAM_V_MCLK     */
		P7_INIT_PINMAP(P7_PWM_02), /* CAM_FPV_1_MCLK */
		P7_INIT_PINMAP(P7_PWM_03), /* CAM_FPV_2_MCLK */
		P7_INIT_PINMAP(P7_PWM_04), /* CLKIN_GYRO     */
		P7_INIT_PINMAP(P7_PWM_05), /* PWM_US_BOTTOM  */
		P7_INIT_PINMAP(P7_PWM_06), /* THERMAL_PWM    */
};


/******
 * Cameras
 ******/
#include <media/video/avicam.h>
#include <parrot/avicam_dummy_dev.h>

/* 12-bit cam */
/* horizontal cam (cam 0) */
static struct pinctrl_map paros_camh_pins[] __initdata = {
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
	P7_INIT_PINMAP(P7_CAM_0_DATA18),
	P7_INIT_PINMAP(P7_CAM_0_DATA19),
};


static struct avicam_dummy_info paros_camh_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.field = V4L2_FIELD_NONE,
		.width = 4640,
		.height = 3320,
	},
};

static struct avicam_platform_data paros_camh_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.enable_stats      = 1,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 1,
		.ivs        = 1,
		.ihs        = 1,
		.ipc        = 1,
		.psync_en   = 1,
	},
	.bus_width	   = 16,
	.subdevs	   = NULL,
	.dummy_driver_info = &paros_camh_dummy_driver_info,
};

static u64 paros_camh_dma_mask = DMA_BIT_MASK(32);

static struct platform_device paros_camh_dev = {
	.name	= "avicam",
	.id	= 0,
	.dev	= {
		.dma_mask	    = &paros_camh_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


/* cam hdmi (cam 1 / ADV7611) */
#define CAM_HDMI_WIDTH		1080
#define CAM_HDMI_HEIGHT		720
#define CAM_HDMI_PIXEL_SIZE	2
#define CAM_HDMI_N_BUFFERS	16

static struct pinctrl_map paros_camhdmi_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINMAP(P7_CAM_1_HS),
	P7_INIT_PINMAP(P7_CAM_1_VS),
	P7_INIT_PINMAP(P7_CAM_1_DATA00),
	P7_INIT_PINMAP(P7_CAM_1_DATA01),
	P7_INIT_PINMAP(P7_CAM_1_DATA02),
	P7_INIT_PINMAP(P7_CAM_1_DATA03),
	P7_INIT_PINMAP(P7_CAM_1_DATA04),
	P7_INIT_PINMAP(P7_CAM_1_DATA05),
	P7_INIT_PINMAP(P7_CAM_1_DATA06),
	P7_INIT_PINMAP(P7_CAM_1_DATA07),
};


/* vertical cam (cam 2) */
#define CAMV_WIDTH	640
#define CAMV_HEIGHT	480
#define CAMV_PIXEL_SIZE (3/2)
#define CAMV_N_BUFFERS	16

static struct pinctrl_map paros_camv_pins[] __initdata = {
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
};

static struct avicam_dummy_info paros_camv_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field = V4L2_FIELD_NONE,
		.width = CAMV_WIDTH,
		.height = CAMV_HEIGHT,
	},
};

static struct avicam_platform_data paros_camv_pdata = {
	.cam_cap	    = AVI_CAP_CAM_2,
	.interface	    = {
		.itu656	    = 1,
		.pad_select = 0,
		.ipc	    = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &paros_camv_dummy_driver_info,
};


static u64 paros_camv_dma_mask = DMA_BIT_MASK(32);
static struct platform_device paros_camv_dev = {
	.name		= "avicam",
	.id		= 2,
	.dev		= {
		.dma_mask	    = &paros_camv_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


/* cam fvp 1 (cam 3) */
static struct pinctrl_map paros_cam_fvp1_pins[] __initdata = {
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

/* cam fvp 2 (cam 4) */
static struct pinctrl_map paros_cam_fvp2_pins[] __initdata = {
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

/*************
 * Regulator
 *************/
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/fixed.h>
//#include <regulator/switch-voltage-regulator.h>
#define ADAU1977_I2C_BUS	1
#define ADAU1977_ADDR		0x11 /* 0x31, 0x51, 0x71 */

static struct regulator_consumer_supply paros_adau1977_consumer_supplies[] = {
	REGULATOR_SUPPLY("AVDD", "1-0011"),
	REGULATOR_SUPPLY("DVDD", "1-0011"),
};

static struct regulator_init_data paros_adau1977_reg_init_data = {
	.constraints	= {
		.name	= "3V3",
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.consumer_supplies = paros_adau1977_consumer_supplies,
	.num_consumer_supplies = ARRAY_SIZE(paros_adau1977_consumer_supplies),
};

static struct fixed_voltage_config paros_adau1977_fvolt_pdata = {
	.supply_name	= "board-3V3",
	.microvolts	= 3300000,
	.gpio		= -EINVAL,
	.enabled_at_boot = 0,
	.init_data	= &paros_adau1977_reg_init_data,
};

static struct platform_device paros_adau1977_voltage_regulator = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev		= {
		.platform_data	= &paros_adau1977_fvolt_pdata,
	},
};


/*************
 * Audio
 *************/
#include <linux/platform_data/adau1977.h>
#include <sound/soc.h>
#include <sound/soc-dai.h>

static struct adau1977_platform_data paros_adau1977_pdata = {
	.micbias	= ADAU1977_MICBIAS_5V0,
	.reset_gpio	= P7_GPIO_NR(PD_RESET),
};

static struct i2c_board_info __initdata paros_adau1977_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("adau1977", 0x11),
		.platform_data = &paros_adau1977_pdata,
	},
};

static struct snd_soc_dai_link paros_adau1977_dai[] = {
	{
		.name           = "adau1977",
		.codec_name     = "adau1977.1-0011",
		.codec_dai_name = "adau1977-hifi",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.dai_fmt        = (SND_SOC_DAIFMT_I2S		|
					SND_SOC_DAIFMT_NB_NF	|
					SND_SOC_DAIFMT_CBS_CFS),
	},
};

/* FC7100 ASoC device */
static struct platform_device paros_asoc_dev = {
	.name           = "parrot-fc7100-audio",
	.id             = 0,
	.dev		= {
		.platform_data = paros_adau1977_dai,
	}
};


#include "aai.h"

static struct aai_pad_t parrot_aai_pads[] =
{
	/* MCLK, BCLK, FCLK (out clocks) */
	{AAI_SIG_MAIN_I2S_FRAME,    11,  PAD_OUT}, /* I2S_FSYNC */
	{AAI_SIG_DAC_BIT_CLOCK,     12,  PAD_OUT}, /* I2S_BLK   */
	{AAI_SIG_MCLK,              10,  PAD_OUT}, /* I2S_MCLK  */

	/* I2S Input / Codec Audio (ADAU1977WBCPZ) */
	{AAI_SIG_I2S0_IN,           13,  PAD_IN}, /* SDATAOUT1 */
	{AAI_SIG_I2S1_IN,           14,  PAD_IN}, /* SDATAOUT2 */

	{-1,                       -1,  0}
};

static char * aai_dev_list[] =
{
	/* Input Channels/ Codec Audio (ADAU1977WBCPZ) */
	"music-in-stereo0",
	"music-in-stereo1",

	/*Don't remove*/
	NULL,
};

static struct aai_conf_set paros_aai_conf_set[] =
{
	/* This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},

	/*Don't remove*/
	{-1, 0, 0, 0},
};

static unsigned long paros_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long paros_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static struct pinctrl_map paros_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, paros_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, paros_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, paros_aai_pinconf),

	P7_INIT_PINMAP(P7_AAI_13),
	P7_INIT_PINCFG(P7_AAI_13, paros_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_14),
	P7_INIT_PINCFG(P7_AAI_14, paros_aai_pinconf),
};
static struct aai_platform_data paros_aai_pdata = {
	.pad = parrot_aai_pads,
	.aai_conf = paros_aai_conf_set,
	.device_list = aai_dev_list,
};

/*************
 * SD
 *************/

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>

static struct acs3_plat_data paros_sdhci_emmc_pdata = {
	.led_gpio   = -1,  /* No activity led GPIO */
	.wp_gpio    = -1,  /* No write protect */
	.cd_gpio    = -1,  /* No card detect */
	.rst_gpio   = -1,
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,     /* emmc is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE,  /* bus voltage is fixed in hardware */
};

/*************
 * SPI
 *************/
#include <mach/p7-adc.h>
#define PAROS_SPI_SLAVE_P7MU    3

static struct pinctrl_map paros_spi_slave_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12b), /* CLK */
	P7_INIT_PINMAP(P7_SPI_13b), /* SS */
	P7_INIT_PINMAP(P7_SPI_14b), /* MOSI */
};

static struct p7spi_swb const paros_spi_slave_p7mu_swb[] = {
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
};

static struct p7spis_ctrl_data paros_spi_slave_p7mu_cdata = {
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

static P7_DECLARE_SPIS_MASTER(paros_spi_master_p7mu_info,
		"p7mu-adc",
		NULL,
		&paros_spi_slave_p7mu_cdata,
		10 * 1000 * 1000,
		SPI_MODE_0|SPI_LSB_FIRST);


static void __init paros_init_spi_p7mu(void)
{
	p7_init_spis(PAROS_SPI_SLAVE_P7MU,
			paros_spi_slave_p7mu_pins,
			ARRAY_SIZE(paros_spi_slave_p7mu_pins),
			paros_spi_slave_p7mu_swb);

	if (p7_init_spis_master(PAROS_SPI_SLAVE_P7MU,
				&paros_spi_master_p7mu_info))
		pr_err("Paros: failed to initialize SPI slave.\n");
}

#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>

/*
SPI_09b     SS0 WLAN
SPI_11      SS1 Codec ADAU
SPI_08      SCLK
SPI_09      MISO
SPI_10      MOSI

Not possible yet to connect Max5725 and ADAU1977
According  to ASIC, an spi core can manage several chip select.
Moreover SPI_09b and SPI_09 are link internally and
can't be used with different fuction.

On patched board the folowing setup will work.
*/

static struct pinctrl_map paros_max5735_spim_single_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_08),
	P7_INIT_PINMAP(P7_SPI_09),
	P7_INIT_PINMAP(P7_SPI_10),
	P7_INIT_PINMAP(P7_SPI_11),
};

static struct p7spi_swb paros_max5735_spim_swb[] = {
	{
		.pad       = 8 /* P7_SPI_08 */,
		.direction = P7_SWB_DIR_OUT,
		.function  = P7_SWB_SPI_CLK,
	},
	{
		.pad       = 9 /* P7_SSPI_09 */,
		.direction = P7_SWB_DIR_IN,
		.function  = P7_SWB_SPI_DATA0,
	},
	{
		.pad       = 10 /* P7_SPI_10 */,
		.direction = P7_SWB_DIR_OUT,
		.function  = P7_SWB_SPI_DATA0,
	},
	{
		.pad       = 11 /* P7_SPI_11 */,
		.direction = P7_SWB_DIR_OUT,
		.function  = P7_SWB_SPI_SS,
	},
	{
		.pad       = -1,
	}
};

static struct p7spi_ctrl_data paros_max5735_spim_cdata = {
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
	.tcapture_delay_ns  = 0
};

static struct spi_board_info paros_max5735_spim_dev = {
	.modalias           = "spidev",
	.platform_data      = NULL,
	.controller_data    = &paros_max5735_spim_cdata,
	.irq                = -1,
	.max_speed_hz       = 40000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};

/*****************************
 * P7MU Power Management Unit
 *****************************/
#include <mfd/p7mu.h>
#include "p7mu.h"
static unsigned long paros_p7mu_reboot_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN)  | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map paros_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(NREBOOT_P7MU),    /* P7 -> P7MU reset request */
	P7_INIT_PINCFG(NREBOOT_P7MU, paros_p7mu_reboot_pinconf),
};

static struct p7_adc_chan paros_adc_p7mu_channels[] = {
	{
		.type = P7MUADC_IIO_RING_BUFFER,
		.channel = 1,
		.freq = 160000,
		.samples = 1000,
	},
	{
		.type = P7MUADC_IIO_RING_BUFFER,
		.channel = 2,
		.freq = 160000,
		.samples = 1000,
	},
};

static struct p7_adc_chan_data paros_adc_p7mu_data = {
	.channels	= paros_adc_p7mu_channels,
	.num_channels	= ARRAY_SIZE(paros_adc_p7mu_channels),
};

static struct p7mu_plat_data paros_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(INT_P7MU),
	.int_32k    = true,     /* Internal 32kHz clock. */
	.int_32m    = false,    /* External 48mHz clock. */
	.chan_data  = &paros_adc_p7mu_data,

	/* Wakeup on gpio3 edge rising event
	  WAKE_UP_WL */
	.gpio_event_sel = P7MU_PMC_GPIO3_EDGE_RISING,
};



static void __init paros_init_mach(void)
{
	int loop, p7rev   = p7_chiprev();

	/* Init hardware revision independant stuff */
	p7_init_mach();

	/* UARTs */
	p7brd_init_uart(1,1);	/* Debug */
	p7brd_init_uart(2,0);	/* BLDC */
	p7brd_init_uart(3,0);	/* NACELLLE */
	p7brd_init_uart(4,0);	/* GNSS2 */
	p7brd_init_uart(5,0);	/* GNSS1 */

	/* I2C init */
	/* I2C 0 : P7MU 0x31, Cam_H, Nacelle, MAGNETO_0 */
	p7brd_init_i2cm(0, 400); /* P7MU */
	p7_gpio_interrupt_register(paros_p7mu_pdata.gpio);
	p7_init_p7mu(0,
			&paros_p7mu_pdata,
			paros_p7mu_pins,
			ARRAY_SIZE(paros_p7mu_pins));

	/* P7MU/ADC */
	paros_init_spi_p7mu();
	p7_init_temperature();

	if (parrot_force_usb_device)
		p7brd_init_udc(0, -1);
	else
		p7brd_init_hcd(0, HOST_MODE_ON);

	p7brd_init_hcd(1, -1);


	/* I2C 1 : Cam_FPV1, Nacelle, MAGNETO_1 */
	p7brd_init_i2cm(1, 400);
	parrot_init_i2c_slave(1,
				paros_adau1977_i2c_board_info,
				"ADAU1977",
				P7_I2C_IRQ);
	p7_init_dev(&paros_asoc_dev, NULL, NULL, 0);

	/* I2C 2: Cam_V, Cam_FPV2, GYRO, HDMI IN / adv7611 */
	p7brd_init_i2cm(2, 400);

	/* GPIOs */
	for( loop = 0; loop < ARRAY_SIZE(paros_gpios); loop ++){

		if( paros_gpios[loop].interrupt )
			p7_gpio_interrupt_register(paros_gpios[loop].gpio);

		p7brd_export_gpio(paros_gpios[loop].gpio,
					paros_gpios[loop].default_value,

					paros_gpios[loop].name);

		if( paros_gpios[loop].bidir ){
			gpio_unexport(paros_gpios[loop].gpio);
			BUG_ON(gpio_export(paros_gpios[loop].gpio, 1));
		}
	}

	p7brd_init_nand(0);

	/* PWM */
	p7_init_p7pwm(&paros_default_pwm_pdata,
		paros_default_pwm_pins,
		ARRAY_SIZE(paros_default_pwm_pins));

	/* eMMC init */
	p7brd_init_sdhci(0, &paros_sdhci_emmc_pdata,
					NULL, NULL, NULL, NULL, 0);

	/* Audio init */
	/* AAI --> Codec Audio (ADAU1977WBCPZ) --> MIC & LINE IN */
	p7_init_aai(paros_aai_pins,
		ARRAY_SIZE(paros_aai_pins), &paros_aai_pdata);

	/* Ethernet */
	p7_init_ether(PHY_IFACE_RGMII,
		      P7_GPIO_NR(NRESET_ETH),
		      P7CTL_DRV_CFG(5));

	/* spi: max5735 */
	p7_init_spim_slave(0, &paros_max5735_spim_dev);
	p7_init_spim(0, paros_max5735_spim_single_pins_cfg,
		ARRAY_SIZE(paros_max5735_spim_single_pins_cfg),
		paros_max5735_spim_swb);
}


static void __init paros_reserve_mem(void)
{
	p7_reserve_nand_mem();
	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);
	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_PAROS, "Paros board")
	.reserve        = &paros_reserve_mem,
	.init_machine   = &paros_init_mach,
P7_MACHINE_END
