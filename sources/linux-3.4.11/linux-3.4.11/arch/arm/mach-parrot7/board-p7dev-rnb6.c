/**
 * linux/arch/arm/mach-parrot7/rnb6-board.c - P7Dev RNB6 daughter board
 *                                            implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    02-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <mach/pwm.h>
#include "board-common.h"
#include "common.h"
#include "board.h"
#include "pinctrl.h"
#include "lcd-monspecs.h"
#include "avi.h"
#include "backlight.h"
#include "gpu.h"
#include <media/video/avicam.h>
#include <media/mt9f002.h>
#include <parrot/avicam_dummy_dev.h>

/***************** Configuration section ******************/
/* drivers for camera sensors*/

/* define this if you use a sensor Aptina MT9V117
 * wired on J801 connector CAM1/Road
 * you have to select driver MT9V117 in kernel configuration
 */
#define USE_MT9V117_DRIVER

/* define this if you use a sensor Aptina MT9F002
 * wired on J801 connector CAM1/Road
 * you have to select driver MT9F002 in kernel configuration
 */
//#define USE_MT9F002_DRIVER

#if defined(USE_MT9V117_DRIVER) && defined(USE_MT9F002_DRIVER)
#error "USE_MT9F002_DRIVER && USE_MT9F002_DRIVER must be exclusive."
#endif
/* define this if you use driver for a HDMI receiver Advantech ADV7611
 * wired on HDMI input connector
 * you have to select driver ADV7604 in kernel configuration
 */
//#define USE_ADV7611_DRIVER

/* define this if you use driver for a Video Decoder TI TVP5151
 * wired on RCA Yellow input connector
 * you have to select driver TVP5150 in kernel configuration
 */
//#define USE_TVP5150_DRIVER

/* define this if you use HS/VS for CAM1 on J801 connector CAM1/Road
 * beware that this is incompatible with usage of LCD_0
 */
//#define USE_CAM1_HS_VS

/* GPU framebuffer */
/* only one frame buffer can be used by the GPU
 * it can use frame buffer on LCD0 or on LCD1 or none
 * select one of 3 following defines
 */
/* define this if you want GPU not to use a frame buffer */
#define GPU_FRAME_BUFFER -1
/* define this if you want GPU to use frame buffer on LCD0 */
//#define GPU_FRAME_BUFFER 0
/* define this if you want GPU to use frame buffer on LCD1 */
//#define GPU_FRAME_BUFFER 1
/**********************************************************/

/******
 * PWM
 ******/

#include "p7_pwm.h"

static struct p7pwm_conf p7dev_conf_clock_camera = {
	.period_precision = 5,          /* Precision for MT9V117 */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_pdata p7dev_pwm_pdata = {
	.conf = {
		[15] = &p7dev_conf_clock_camera,/* XMCLK CAM0/REAR CAM1/ROAD */
	}
};

static struct pinctrl_map p7dev_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_15),
};

/**********************
 * PWM Backlight
 **********************/

#if defined(CONFIG_PWM_PARROT7) ||		\
	defined(CONFIG_PWM_PARROT7_MODULE)
void __init rnb6_init_bkl(void)
{
	p7_init_bkl(0,
	            NULL,
		    /* PWM mapped to IO_4_P7MU in rnb6_board.c */
		    P7MU_PWM_NR(1),
		    /* the enable pin is shared with multiple components on
		     * the board, so we don't let the backlight driver manage
		     * it */
		    -1,
		    P7MU_IO_NR(3),
	            NSEC_PER_SEC / 200,    /* 200 Hz */
	            256,                   /* 100% duty cycle / brightness */
	            3);                    /* 1% duty cycle / brightness */
}

#else

static inline void rnb6_init_bkl(void)
{
	return;
}

#endif /*CONFIG_PWM_PARROT7*/

/***********************
 * AVI configuration
 ***********************/

/***********************************************************************/
/** LCD0
 * PANEL kyocera
 */
static struct pinctrl_map rnb6_avifb0_pins[] __initdata = {
	/* LCD0 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_0_CLK),
	P7_INIT_PINMAP(P7_LCD_0_DATA00),
	P7_INIT_PINMAP(P7_LCD_0_DATA01),
	P7_INIT_PINMAP(P7_LCD_0_DATA02),
	P7_INIT_PINMAP(P7_LCD_0_DATA03),
	P7_INIT_PINMAP(P7_LCD_0_DATA04),
	P7_INIT_PINMAP(P7_LCD_0_DATA05),
	P7_INIT_PINMAP(P7_LCD_0_DATA06),
	P7_INIT_PINMAP(P7_LCD_0_DATA07),
	P7_INIT_PINMAP(P7_LCD_0_DATA08),
	P7_INIT_PINMAP(P7_LCD_0_DATA09),
	P7_INIT_PINMAP(P7_LCD_0_DATA10),
	P7_INIT_PINMAP(P7_LCD_0_DATA11),
	P7_INIT_PINMAP(P7_LCD_0_DATA12),
	P7_INIT_PINMAP(P7_LCD_0_DATA13),
	P7_INIT_PINMAP(P7_LCD_0_DATA14),
	P7_INIT_PINMAP(P7_LCD_0_DATA15),
	P7_INIT_PINMAP(P7_LCD_0_DATA16),
	P7_INIT_PINMAP(P7_LCD_0_DATA17),
	P7_INIT_PINMAP(P7_LCD_0_DATA18),
	P7_INIT_PINMAP(P7_LCD_0_DATA19),
	P7_INIT_PINMAP(P7_LCD_0_DATA20),
	P7_INIT_PINMAP(P7_LCD_0_DATA21),
	P7_INIT_PINMAP(P7_LCD_0_DATA22),
	P7_INIT_PINMAP(P7_LCD_0_DATA23),
	P7_INIT_PINMAP(P7_LCD_0_DEN),
	P7_INIT_PINMAP(P7_LCD_0_HS),
	P7_INIT_PINMAP(P7_LCD_0_VS),
};

#define P7_AVIFB0_BUFFER_SIZE PAGE_ALIGN(1280*800*4*2)
static struct avifb_overlay rnb6_avi_lcd0_overlays[] = {
	{
		.layout = {
			.alpha = AVI_ALPHA(100),
			.enabled = 1,
		},
		.dma_memory.end = P7_AVIFB0_BUFFER_SIZE,
		.caps		 = 0,
		.zorder		 = -1,
	},
};

static struct avifb_platform_data rnb6_avifb0_pdata = {

	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &kyocera_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_0,
	.overlays	       = rnb6_avi_lcd0_overlays,
	.overlay_nr	       = ARRAY_SIZE(rnb6_avi_lcd0_overlays),
};

/* Should we bother setting a narrower mask here? */
static u64 rnb6_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_avifb0_dev = {
	.name           = "avifb",
	.id             = 0,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &rnb6_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/***********************************************************************/
/** LCD1
 * HDMI ADV7511W
 */
static struct pinctrl_map rnb6_avifb1_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINMAP(P7_LCD_1_VS),
};

#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1920*1080*4*2)

static struct avifb_overlay rnb6_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.enabled = 1,
		},
		.dma_memory.end	 = P7_AVIFB1_BUFFER_SIZE,
		.caps		 = 0,
		.zorder		 = -1,
	},
};

static struct avifb_platform_data rnb6_avifb1_pdata = {
	/* HDMI output chip is wired in Data 8 bits */
	.lcd_format_control    = AVI_FORMAT_CONTROL_UYVY_2X8,
	.output_cspace         = AVI_BT709_CSPACE,
	.lcd_default_videomode = &hdmi_1280x720p60_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_1 | AVI_CAP_CONV,

	.overlays	       = rnb6_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(rnb6_avi_lcd1_overlays),
};

static u64 rnb6_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &rnb6_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/***********************************************************************/
/** CAM1
 * Connectors J800 and J801 "ROAD"
 */

#define RNB6_CAM1_I2C_BUS		2
#define RNB6_CAM1_PWM			15
#define RNB6_CAM1_PWM_PERIOD_NS		37 /* 27 MHz approx */

static struct pwm_device *rnb6_cam1_pwm_device = NULL;
static int rnb6_cam1_power_on(void)
{
	int ret = 0;

	if (rnb6_cam1_pwm_device == NULL ) {
		rnb6_cam1_pwm_device = pwm_request(RNB6_CAM1_PWM, "cam1-pwm");
		if (IS_ERR(rnb6_cam1_pwm_device)) {
			ret = PTR_ERR(rnb6_cam1_pwm_device);
			goto err_alloc;
		}
	}

	ret = pwm_config(rnb6_cam1_pwm_device,
			 0,
			 RNB6_CAM1_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	ret = pwm_enable(rnb6_cam1_pwm_device);
	if (ret)
		goto err_config;

	return 0;

err_config:
	pwm_free(rnb6_cam1_pwm_device);
	rnb6_cam1_pwm_device = NULL;
err_alloc:
	pr_warning("failed to set clock for mt9v117 chip\n");
	return ret;
}

static int rnb6_cam1_power_off(void)
{
	pwm_disable(rnb6_cam1_pwm_device);
	return 0;
}

static int rnb6_cam1_set_power(int on)
{
	if(on == MT9F002_POWER_ON)
		return rnb6_cam1_power_on();
	else
		return rnb6_cam1_power_off();
}

#ifdef USE_MT9V117_DRIVER
/* Aptina MT9V117 Subdev */
#include <media/mt9v117.h>
#define MT9V117_INPUT_FREQ_HZ	27000000
static struct mt9v117_platform_data rnb6_cam1_mt9v117_platform_data = {
	.set_power = &rnb6_cam1_set_power,
	.enable_bt656    = 1,
	.ext_clk_freq_hz = MT9V117_INPUT_FREQ_HZ,
};

static struct i2c_board_info rnb6_cam1_mt9v117_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9v117", 0x5D),
		.platform_data = &rnb6_cam1_mt9v117_platform_data,
	}
};

#elif defined(USE_MT9F002_DRIVER)
/* Aptina MT9F002 subdev */
#include <media/mt9f002.h>

#ifndef USE_CAM1_HS_VS
#define 	USE_CAM1_HS_VS
#endif

#define MT9F002_INPUT_FREQ_mHZ		26000000000
#define MT9F002_OUTPUT_FREQ_mHZ		96000000000

static struct mt9f002_platform_data cam1_mt9f002_platform_data = {
	.interface = MT9F002_Parallel,
	.pixel_depth = MT9F002_10bits,
	.number_of_lanes = MT9F002_2lane,
	.ext_clk_freq_mhz = MT9F002_INPUT_FREQ_mHZ,
	.output_clk_freq_mhz = MT9F002_OUTPUT_FREQ_mHZ,
	.set_power = &rnb6_cam1_set_power,
};

static struct i2c_board_info cam1_mt9f002_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9f002", 0x10),
		.platform_data = &cam1_mt9f002_platform_data,
	}
};

#endif

static struct pinctrl_map rnb6_cam1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINMAP(P7_CAM_1_DATA00),
	P7_INIT_PINMAP(P7_CAM_1_DATA01),
	P7_INIT_PINMAP(P7_CAM_1_DATA02),
	P7_INIT_PINMAP(P7_CAM_1_DATA03),
	P7_INIT_PINMAP(P7_CAM_1_DATA04),
	P7_INIT_PINMAP(P7_CAM_1_DATA05),
	P7_INIT_PINMAP(P7_CAM_1_DATA06),
	P7_INIT_PINMAP(P7_CAM_1_DATA07),
#ifdef USE_CAM1_HS_VS
	P7_INIT_PINMAP(P7_CAM_1_HS),
	P7_INIT_PINMAP(P7_CAM_1_VS),
#endif
};

#ifdef USE_MT9V117_DRIVER
/* useful for Aptina MT9M114 */
#define RNB6_CAM1_HD_WIDTH 1280
#define RNB6_CAM1_HD_HEIGHT 720
/* 2 bytes/pixel (UYVY), 4 buffers */
#define RNB6_CAM1_SIZE (RNB6_CAM1_HD_WIDTH * RNB6_CAM1_HD_HEIGHT * 2 * 4)

static struct avicam_dummy_info rnb6_cam1_dummy_driver_info = {
	.dev_id		    = 1,
	.format		    = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = RNB6_CAM1_HD_WIDTH,
		.height	    = RNB6_CAM1_HD_HEIGHT,
	},
};

#elif defined(USE_MT9F002_DRIVER)
/* useful for Aptina MT9M114 */
#define RNB6_CAM1_HD_WIDTH 4640
#define RNB6_CAM1_HD_HEIGHT 3320

static struct avicam_dummy_info rnb6_cam1_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code = V4L2_MBUS_FMT_SGRBG10_1X10,
		.field = V4L2_FIELD_NONE,
		.width = RNB6_CAM1_HD_WIDTH,
		.height = RNB6_CAM1_HD_HEIGHT,
	},
};
#define RNB6_CAM1_SIZE ((RNB6_CAM1_HD_WIDTH * RNB6_CAM1_HD_HEIGHT * 2 + 2048) * 3)

#endif

static struct avicam_subdevs rnb6_cam1_subdevs[] = {
#ifdef USE_MT9V117_DRIVER
	{
		.board_info = &rnb6_cam1_mt9v117_i2c_devices[0],
		.i2c_adapter_id = RNB6_CAM1_I2C_BUS,
	},
#elif defined(USE_MT9F002_DRIVER)
	{
		.board_info = &cam1_mt9f002_i2c_devices[0],
		.i2c_adapter_id = RNB6_CAM1_I2C_BUS,
	},
#endif
	{ NULL, 0, },
};

#if defined(USE_MT9F002_DRIVER) || defined(USE_MT9V117_DRIVER)
#if 0
static struct avicam_platform_data rnb6_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.pad_select = 0,
#ifndef USE_CAM1_HS_VS
		.itu656	    = 1,
		.ipc        = 1,
#else
		.itu656	    = 0,
		.ivs	    = 1,
		.ihs	    = 0,
		.ipc        = 0,
#endif
	},
#if defined(USE_MT9V117_DRIVER)
	.bus_width	   = 8,
#else
	.bus_width	   = 16,
#endif
	.subdevs	   = rnb6_cam1_subdevs,
	.dummy_driver_info = &rnb6_cam1_dummy_driver_info,
};
#endif

static struct avicam_platform_data rnb6_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.enable_stats      = 0,
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
	.subdevs	   = rnb6_cam1_subdevs,
	.dummy_driver_info = &rnb6_cam1_dummy_driver_info,
};

static u64 rnb6_cam1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_cam1_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &rnb6_cam1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};
#endif

/***************************************************/

/***********************************************************************/
/** CAM4
 * Connector J300 "HDMI IN" ADV7611
 */
static struct pinctrl_map rnb6_cam4_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_4_CLK),
	P7_INIT_PINMAP(P7_CAM_4_DATA00),
	P7_INIT_PINMAP(P7_CAM_4_DATA01),
	P7_INIT_PINMAP(P7_CAM_4_DATA02),
	P7_INIT_PINMAP(P7_CAM_4_DATA03),
	P7_INIT_PINMAP(P7_CAM_4_DATA04),
	P7_INIT_PINMAP(P7_CAM_4_DATA05),
	P7_INIT_PINMAP(P7_CAM_4_DATA06),
	P7_INIT_PINMAP(P7_CAM_4_DATA07),
};

#define RNB6_CAM4_I2C_BUS		1

/* default resolution HD 1280x720 */
#define RNB6_CAM4_HD_WIDTH 1280
#define RNB6_CAM4_HD_HEIGHT 720
/* 2 bytes/pixel (UYVY), 4 buffers */
#define RNB6_CAM4_HD_SIZE (RNB6_CAM4_HD_WIDTH * RNB6_CAM4_HD_HEIGHT * 2 * 4)

/* max resolution Full HD 1920x1080 */
#define RNB6_CAM4_FULL_HD_WIDTH 1920
#define RNB6_CAM4_FULL_HD_HEIGHT 1080
/* 2 bytes/pixel (UYVY), 4 buffers */
#define RNB6_CAM4_FULL_HD_SIZE (RNB6_CAM4_FULL_HD_WIDTH * RNB6_CAM4_FULL_HD_HEIGHT * 2 * 4)

static struct avicam_dummy_info rnb6_cam4_dummy_driver_info = {
	.dev_id		    = 4,
	.format		    = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = RNB6_CAM4_HD_WIDTH,
		.height	    = RNB6_CAM4_HD_HEIGHT,
	},
};

#define RNB6_CAM4_SIZE RNB6_CAM4_FULL_HD_SIZE

#ifdef USE_ADV7611_DRIVER
#include <media/adv7604.h>
/* ADV7611 HDMI Input: I2C Subdevice addresses */
#define ADV7611_IO_SLAVE_ADDRESS		0x98
#define ADV7611_CP_SLAVE_ADDRESS		0x44
#define ADV7611_HDMI_SLAVE_ADDRESS		0x68
#define ADV7611_EDID_SLAVE_ADDRESS		0x6C
#define ADV7611_REPEATER_SLAVE_ADDRESS		0x64
#define ADV7611_DPLL_SLAVE_ADDRESS		0x4c
#define ADV7611_INFOFRAME_SLAVE_ADDRESS		0x7C
#define ADV7611_CEC_SLAVE_ADDRESS		0x80

static struct adv7604_platform_data rnb6_adv7604_platform_data = {
	.disable_pwrdnb		     = 0,
	.op_ch_sel		     = ADV7604_OP_CH_SEL_RGB,
	.alt_gamma		     = 0,
	.op_656_range		     = 1,
	.rgb_out		     = 0,
	.alt_data_sat		     = 1,
	.op_format_sel		     = ADV7604_OP_FORMAT_SEL_SDR_ITU656_8,
	.int1_config		     = ADV7604_INT1_CONFIG_ACTIVE_LOW,
	.inp_color_space	     = ADV7604_INP_COLOR_SPACE_AUTO,
	.connector_hdmi		     = 1,
	.insert_av_codes	     = 1,
	.blank_data		     = 1,
	.i2c_cec		     = (ADV7611_CEC_SLAVE_ADDRESS >> 1),
	.i2c_infoframe		     = (ADV7611_INFOFRAME_SLAVE_ADDRESS >> 1),
	.i2c_afe		     = (ADV7611_DPLL_SLAVE_ADDRESS >> 1),
	.i2c_repeater		     = (ADV7611_REPEATER_SLAVE_ADDRESS >> 1),
	.i2c_edid		     = (ADV7611_EDID_SLAVE_ADDRESS >> 1),
	.i2c_hdmi		     = (ADV7611_HDMI_SLAVE_ADDRESS >> 1),
	.i2c_cp			     = (ADV7611_CP_SLAVE_ADDRESS >> 1),
	.default_width		     = RNB6_CAM4_HD_WIDTH,
	.default_height		     = RNB6_CAM4_HD_HEIGHT,
	.cam_it			     = -1,
	.power_on                    = NULL,
	.power_off                   = NULL
};

static struct i2c_board_info rnb6_cam4_i2c_devices[] = {
	{
		I2C_BOARD_INFO("adv7611", (ADV7611_IO_SLAVE_ADDRESS >> 1)),
		.platform_data = &rnb6_adv7604_platform_data,
	}
};

static struct avicam_subdevs rnb6_cam4_subdevs[] = {
	{
		.board_info = &rnb6_cam4_i2c_devices[0],
		.i2c_adapter_id = RNB6_CAM4_I2C_BUS,
	},
	{ NULL, 0, },
};
#endif /* USE_ADV7611_DRIVER */

static struct avicam_platform_data rnb6_cam4_pdata = {
	.cam_cap	   = AVI_CAP_CAM_4,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
		.ipc        = 0,
	},
	.bus_width	   = 8,
#ifdef USE_ADV7611_DRIVER
	.subdevs	   = rnb6_cam4_subdevs,
#endif /* USE_ADV7611_DRIVER */
	.dummy_driver_info = &rnb6_cam4_dummy_driver_info,
};

static u64 rnb6_cam4_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_cam4_dev = {
	.name           = "avicam",
	.id             = 4,
	.dev            = {
		.dma_mask           = &rnb6_cam4_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/***********************************************************************/
/** CAM0
 * Connector J700 / J701 RCA Yellow "REAR"
 * TVP5151 Ultralow-Power NTSC/PAL/SECAM Video Decoder
 */
static struct pinctrl_map rnb6_cam0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
};

#define RNB6_CAM0_I2C_BUS		2

/* default and max resolution SD 720x576 */
#define RNB6_CAM0_WIDTH 720
#define RNB6_CAM0_HEIGHT 576
/* 2 bytes/pixel, 4 buffers */
#define RNB6_CAM0_SIZE (RNB6_CAM0_WIDTH * RNB6_CAM0_HEIGHT * 2 * 4)

static struct avicam_dummy_info rnb6_cam0_dummy_driver_info = {
	.dev_id		    = 0,
	.format		    = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = RNB6_CAM0_WIDTH,
		.height	    = RNB6_CAM0_HEIGHT,
	},
};

#ifdef USE_TVP5150_DRIVER
#include <media/tvp5150.h>

static struct tvp5150_platform_data rnb6_tvp5150_platform_data = {
	.mbus_type = V4L2_MBUS_BT656
};

static struct i2c_board_info rnb6_cam0_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tvp5150", 0x5C),
		.platform_data = &rnb6_tvp5150_platform_data,
	}
};

static struct avicam_subdevs rnb6_cam0_subdevs[] = {
	{
		.board_info = &rnb6_cam0_i2c_devices[0],
		.i2c_adapter_id = RNB6_CAM0_I2C_BUS,
	},
	{ NULL, 0, },
};
#endif /* USE_TVP5150_DRIVER */

static struct avicam_platform_data rnb6_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
		.ipc        = 0,
	},
	.bus_width	   = 8,
#ifdef USE_TVP5150_DRIVER
	.subdevs	   = rnb6_cam0_subdevs,
#endif /* USE_TVP5150_DRIVER */
	.dummy_driver_info = &rnb6_cam0_dummy_driver_info,
};

static u64 rnb6_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &rnb6_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/***********************************************************************/
/** M2M
 *
 */
static struct avi_m2m_platform_data rnb6_avi_m2m_pdata[] = {
	{ .caps = AVI_CAP_PLANAR | AVI_CAP_SCAL | AVI_CAP_CONV },
	{ .caps = AVI_CAPS_ISP },
	{ .caps = 0 },
};

/***********************************************************************/
/** VOC
 * VOC #0 connected to LCD0 PANEL
 * VOC #1 connected to LCD1 HDMI
 * VOC #2 connected to RAM2RAM
 */
static struct avi_voc_plat_data rnb6_avi_voc_lcd0 = {
	.display = AVIFB_LCD_0_ID
};

static struct avi_voc_plat_data rnb6_avi_voc_lcd1 = {
	.display = AVIFB_LCD_1_ID
};

static struct avi_voc_plat_data rnb6_avi_voc_m2m = {
	.display = "m2m.0"
};

/***********************************************************************/
void __init rnb6db_rsvmem(struct p7_board const* board)
{
	p7_reserve_avifbmem(&rnb6_avifb0_dev,
			    rnb6_avi_lcd0_overlays,
			    ARRAY_SIZE(rnb6_avi_lcd0_overlays));

	p7_reserve_avifbmem(&rnb6_avifb1_dev,
			    rnb6_avi_lcd1_overlays,
			    ARRAY_SIZE(rnb6_avi_lcd1_overlays));

#if defined(USE_MT9F002_DRIVER) || defined(USE_MT9V117_DRIVER)
	p7_reserve_avicammem(&rnb6_cam1_dev, RNB6_CAM1_SIZE);
#endif

	p7_reserve_avicammem(&rnb6_cam4_dev, RNB6_CAM4_SIZE);

	p7_reserve_avicammem(&rnb6_cam0_dev, RNB6_CAM0_SIZE);

#define RNB6_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(0, RNB6_VOC_SIZE);

	p7_reserve_avi_voc_mem(1, RNB6_VOC_SIZE);

	p7_reserve_avi_voc_mem(2, RNB6_VOC_SIZE);

#define P7DEV_AVI_M2M_SIZE (1920 * 1080 * 4 * 4 * 2)
	p7_reserve_avi_m2m_mem(P7DEV_AVI_M2M_SIZE);

}

void __init rnb6db_probe(struct p7_board const* board)
{
	unsigned long fb_start = 0;
	unsigned long fb_size = 0;
	p7brd_init_i2cm(2, 100);

	p7_init_p7pwm(&p7dev_pwm_pdata,
	              p7dev_pwm_pins,
		      ARRAY_SIZE(p7dev_pwm_pins));

	rnb6_init_bkl();

	/* On the RNB6 board, this pin (IO_5_P7MU) deresets all the
	 * controlers on the daughterboard. */
	if (gpio_request_one(P7MU_IO_NR(5), GPIOF_OUT_INIT_HIGH,
			     "RNB6 board nRST"))
		pr_err("RNB6: failed to de-reset board\n");
	else
		pr_info("RNB6: board de-reset; LCD backlight enabled\n");

	p7_init_avi();

#if !defined(USE_CAM1_HS_VS)
	p7_init_avifb(&rnb6_avifb0_dev, &rnb6_avifb0_pdata,
			rnb6_avifb0_pins, ARRAY_SIZE(rnb6_avifb0_pins));
#endif

	p7_init_avifb(&rnb6_avifb1_dev, &rnb6_avifb1_pdata,
			rnb6_avifb1_pins, ARRAY_SIZE(rnb6_avifb1_pins));

#if GPU_FRAME_BUFFER == 0
	fb_start = rnb6_avifb0_pdata.overlays[0].dma_memory.start;
	fb_size  = rnb6_avifb0_pdata.overlays[0].dma_memory.end - fb_start + 1;
#elif GPU_FRAME_BUFFER == 1
	fb_start = rnb6_avifb1_pdata.overlays[0].dma_memory.start;
	fb_size = rnb6_avifb1_pdata.overlays[0].dma_memory.end - fb_start + 1;
#endif /* GPU_FRAME_BUFFER - If no frame buffer, leave fb_start and fb_size at 0*/

	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &rnb6_avi_voc_lcd0);

	p7_init_avi_voc(1, &rnb6_avi_voc_lcd1);

	p7_init_avi_voc(2, &rnb6_avi_voc_m2m);

#if defined(USE_MT9F002_DRIVER) || defined(USE_MT9V117_DRIVER)
	p7_init_avicam(&rnb6_cam1_dev,
	               &rnb6_cam1_pdata,
	               rnb6_cam1_pins,
	               ARRAY_SIZE(rnb6_cam1_pins));
#endif

	p7_init_avicam(&rnb6_cam4_dev,
	               &rnb6_cam4_pdata,
	               rnb6_cam4_pins,
	               ARRAY_SIZE(rnb6_cam4_pins));

	p7_init_avicam(&rnb6_cam0_dev,
	               &rnb6_cam0_pdata,
	               rnb6_cam0_pins,
	               ARRAY_SIZE(rnb6_cam0_pins));

	p7_init_avi_m2m(rnb6_avi_m2m_pdata);
}
