/*
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include "spi.h"
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>
#include <mfd/p7mu.h>
#include <p7mu.h>
#include <mach/ether.h>
#include <asm/system.h>

#include <media/adv7604.h>

#include "gpio.h"
#include "gpu.h"
#include "common.h"
#include "system.h"
#include "lcd-monspecs.h"
#include "avi.h"
#include "mpegts.h"
#include "venc.h"
#include "vdec.h"
#include "p7_temperature.h"
#include "wl18xx.h"

#include "board-common.h"
#include "nand.h"
#include "usb.h"

#include <media/video/avicam.h>
#include <parrot/avicam_dummy_dev.h>

/*********************
 * Framebuffer config
 *********************/

static unsigned long bebop_fb1_drivecfg[] = {
	P7CTL_DRV_CFG(1),
};

static struct pinctrl_map bebop_fb1_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, bebop_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, bebop_fb1_drivecfg),
};


static struct avifb_overlay bebop_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = (1920 * 1080 * 4 * 2),
		.zorder		 = 0,
	},
};

static struct avifb_platform_data bebop_v1_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &hdmi_1920x1080p60_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_1 | AVI_CAP_GAM,
	.background            = 0x00ff00,
	.overlays	       = bebop_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(bebop_avi_lcd1_overlays),
};

static struct avifb_platform_data bebop_v2_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &kyocera_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_1 | AVI_CAP_GAM,
	.background            = 0x00ff00,
	.overlays	       = bebop_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(bebop_avi_lcd1_overlays),
};


static u64 bebop_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device bebop_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &bebop_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

 /********************
 * HDMI Output config
 ********************/

#include "../../../drivers/video/mxc/siihdmi.h"
#include "i2cm.h"

static struct siihdmi_platform_data bebop_siihdmi_data = {
// XXX
//	.reset       = mx51_efikamx_display_reset,

	.vendor      = "Genesi",
	.description = "Efika MX",

	.lcd_id = "lcd.1",

//XXX
/*	.hotplug     = {
		.start = IOMUX_TO_IRQ(MX51_PIN_DISPB2_SER_DIO),
		.end   = IOMUX_TO_IRQ(MX51_PIN_DISPB2_SER_DIO),
		.name  = "video-hotplug",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},*/

// XXX
//	.pixclock    = KHZ2PICOS(133000L),
};

static struct i2c_board_info __initdata bebop_siihdmi_board_info[] = {
	{
		I2C_BOARD_INFO("siihdmi", 0x3b),
		.platform_data = &bebop_siihdmi_data,
	},
};

/****************************
 * Bebop v1 HDMI Input config
 ****************************/

static struct pinctrl_map bebop_v1_cam0_pins[] __initdata = {
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
	P7_INIT_PINMAP(P7_CAM_0_DATA16),
	P7_INIT_PINMAP(P7_CAM_0_DATA17),
	P7_INIT_PINMAP(P7_CAM_0_DATA18),
	P7_INIT_PINMAP(P7_CAM_0_DATA19),
	P7_INIT_PINMAP(P7_CAM_0_DATA20),
	P7_INIT_PINMAP(P7_CAM_0_DATA21),
	P7_INIT_PINMAP(P7_CAM_0_DATA22),
	P7_INIT_PINMAP(P7_CAM_0_DATA23),
};

#define ADV7611_CP_SLAVE_ADDRESS		0x44
#define ADV7611_HDMI_SLAVE_ADDRESS		0x68
#define ADV7611_EDID_SLAVE_ADDRESS		0x6C
#define ADV7611_REPEATER_SLAVE_ADDRESS		0x64
#define ADV7611_DPLL_SLAVE_ADDRESS		0x4c
#define ADV7611_INFOFRAME_SLAVE_ADDRESS		0x7C
#define ADV7611_CEC_SLAVE_ADDRESS		0x80

#define ADV7611_HDMI_RECEIVER_WIDTH             FULL_HD_WIDTH
#define ADV7611_HDMI_RECEIVER_HEIGHT            FULL_HD_HEIGHT

#define CAM0_NRST                               P7_GPIO_NR(12)

static struct adv7604_platform_data bebop_adv7604_platform_data = {
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
	.default_width		     = 1920,
	.default_height		     = 1080,
	.cam_it			     = -1,
	.power_on                    = NULL,
	.power_off                   = NULL,
};

static struct i2c_board_info bebop_cam0_i2c_devices[] = {
	{
		I2C_BOARD_INFO("adv7611", 0x4C),
		.platform_data = &bebop_adv7604_platform_data,
	}
};

static struct avicam_subdevs bebop_cam0_subdevs[] = {
	{
		.board_info	= &bebop_cam0_i2c_devices[0],
		.i2c_adapter_id = 2l,
	},
	{ NULL, 0, },
};

#define BEBOP_V1_CAM0_SIZE (1920 * 1080 * 4 * 2)

static struct avicam_platform_data bebop_v1_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = bebop_cam0_subdevs,
	.dummy_driver_info = NULL,
};

static u64 bebop_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device bebop_v1_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &bebop_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


/******************************
 * Bebop v2 camera Input config
 ******************************/

#include "p7_pwm.h"

static struct p7pwm_conf bebop_v2_conf_clock_camera = {
	.period_precision = 5,          /* Precision for MT9V117 */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_pdata bebop_v2_pwm_pdata = {
	.conf = {
		[11] = &bebop_v2_conf_clock_camera,
	}
};

static unsigned long bebop_v2_pwm_drivecfg[] = {
	P7CTL_DRV_CFG(3),
};

static struct pinctrl_map bebop_v2_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_11),
	P7_INIT_PINCFG(P7_PWM_11, bebop_v2_pwm_drivecfg),
};

static struct pinctrl_map bebop_v2_cam0_pins[] __initdata = {
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
};

static struct avicam_dummy_info bebop_v2_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.field = V4L2_FIELD_NONE,
		.width = 4640,
		.height = 3320,
	},
};

#define MT9F002_INPUT_FREQ_mHZ		26000000000
#define MT9F002_OUTPUT_FREQ_mHZ		96000000000
#define MT9F002_PWM_PERIOD_NS		38 /* 27 MHz approx */
/* Aptina MT9F002 Subdev */
#include <media/mt9f002.h>
#define CAM0_I2C_BUS			2
#define CAM0_PWM			11
#define CAM0_NRST_GPIO 			131
static struct pwm_device *cam0_pwm_device = NULL;
static int cam0_mt9f002_power_on(void)
{
	int ret = 0;

	if (cam0_pwm_device == NULL) {
		cam0_pwm_device = pwm_request(CAM0_PWM, "cam0-pwm");
		if (IS_ERR(cam0_pwm_device)) {
			ret = PTR_ERR(cam0_pwm_device);
			goto err_alloc;
		}
	}

	ret = pwm_config(cam0_pwm_device,
			 0,
			 MT9F002_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	ret = pwm_enable(cam0_pwm_device);
	if (ret)
		goto err_config;

	gpio_set_value_cansleep(CAM0_NRST_GPIO, 1);

	return 0;

err_config:
	pwm_free(cam0_pwm_device);
	cam0_pwm_device = NULL;
err_alloc:
	pr_warning("failed to set clock for mt9f002 chip\n");
	return ret;
}

static int cam0_mt9f002_power_off(void)
{
	gpio_set_value_cansleep(CAM0_NRST_GPIO, 0);
	pwm_disable(cam0_pwm_device);
	return 0;
}

static int cam0_mt9f002_set_power(int on)
{
	if (on == MT9F002_POWER_ON)
		return cam0_mt9f002_power_on();
	else
		return cam0_mt9f002_power_off();
}

static struct mt9f002_platform_data cam0_mt9f002_platform_data = {
	.interface = MT9F002_Parallel,
	.pixel_depth = MT9F002_10bits,
	.number_of_lanes = MT9F002_2lane,
	.ext_clk_freq_mhz = MT9F002_INPUT_FREQ_mHZ,
	.output_clk_freq_mhz = MT9F002_OUTPUT_FREQ_mHZ,
	.set_power = &cam0_mt9f002_set_power,
};

static struct i2c_board_info cam0_mt9f002_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9f002", 0x10),
		.platform_data = &cam0_mt9f002_platform_data,
	}
};

static struct avicam_subdevs cam0_mt9f002_subdevs[] = {
	{
		.board_info = &cam0_mt9f002_i2c_devices[0],
		.i2c_adapter_id = CAM0_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_platform_data bebop_v2_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.enable_stats      = 1,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 1,
		.ihs        = 1,
		.ipc        = 1,
		.psync_en   = 1,
//		.ror_lsb    = 1,
	},
	.bus_width	   = 16,
	.subdevs	   = cam0_mt9f002_subdevs,
	.dummy_driver_info = &bebop_v2_cam0_dummy_driver_info,
};

#define BEBOP_V2_CAM0_SIZE ((4640 * 3320 * 2 + 2048) * 3)

static u64 cam0_dma_mask = DMA_BIT_MASK(32);

static struct platform_device bebop_v2_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/************************************
 * Video Output overlay (VOC) config
 ************************************/

static struct avi_voc_plat_data bebop_avi_voc_param = {
	.display = "lcd.1"
};

static struct avi_m2m_platform_data bebop_avi_m2m_pdata[] = {
	{ .caps = AVI_CAPS_ISP },
	{ .caps = AVI_CAP_PLANAR |AVI_CAP_SCAL | AVI_CAP_CONV },
	{ .caps = 0 },
};

/*
 * SPI master to control Octopus
 */
#define BEBOP_SPI_TUNER_CTRL_BUS        0

static struct pinctrl_map bebop_tuner_ctrl_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_08),
	P7_INIT_PINMAP(P7_SPI_09),
	P7_INIT_PINMAP(P7_SPI_10),
	P7_INIT_PINMAP(P7_SPI_11),
};

static struct p7spi_swb const bebop_tuner_ctrl_swb[] = {
	P7SPI_INIT_SWB(8,  P7_SWB_DIR_OUT,   P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(9,  P7_SWB_DIR_IN,    P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(10, P7_SWB_DIR_OUT,   P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(11, P7_SWB_DIR_OUT,   P7_SWB_SPI_SS),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data bebop_tuner_ctrl_cdata = {
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

static P7_DECLARE_SPIM_SLAVE(bebop_tuner_ctrl_info,
			     "spidev",
			     NULL,
			     &bebop_tuner_ctrl_cdata,
			     40 * 1000 * 1000,
			     SPI_MODE_0);

/*
 * core 0
 */
static struct pinctrl_map bebop_tuner0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_16),
	P7_INIT_PINMAP(P7_SPI_17),
};

static struct p7spi_swb const bebop_tuner0_swb[] = {
	P7SPI_INIT_SWB(16, P7_SWB_DIR_IN,   P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB(17, P7_SWB_DIR_IN,   P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

struct p7mpg_plat_data bebop_tuner0_pdata = {
	.swb         = bebop_tuner0_swb,
	.fifo_wcnt   = 16,
	.thres_wcnt  = 4,
};

/*
 * core 1
 */
static struct pinctrl_map bebop_tuner1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_18),
	P7_INIT_PINMAP(P7_SPI_19),
};

static struct p7spi_swb const bebop_tuner1_swb[] = {
	P7SPI_INIT_SWB(18, P7_SWB_DIR_IN,   P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB(19, P7_SWB_DIR_IN,   P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

struct p7mpg_plat_data bebop_tuner1_pdata = {
	.swb         = bebop_tuner1_swb,
	.fifo_wcnt   = 16,
	.thres_wcnt  = 4,
};

/***********************
 * SDHCI hosts handling
 ***********************/

#include "sdhci.h"
#include <linux/mmc/host.h>

static struct acs3_regulator_gpios bebop_sdhci1_regulator_gpios = {
	.bus_1v8_gpio = 72,                             /* GPIO 72 is 1.8V switch
	                                                 * command */
	.bus_1v8_active_low = 0,                        /* 1.8V switch is active high */
};

static struct acs3_plat_data bebop_sdhci1_pdata = {
	.led_gpio   = 2,                                /* GPIO 02 drives activity led */
	.wp_gpio    = 0,                                /* GPIO 00 is write protect
	                                                 * status */
	.cd_gpio    = 1,                                /* GPIO 01 is card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

/*
 * microSD on MMC2: as indicated in BEBOP board's schematics clock frequency is
 * limited to 25MHz.
 */
static struct acs3_mode_config bebop_sdhci2_25MHz = {
	.mode   = ACS3_HIGH_SPEED,
	.max_Hz = 25 * 1000 * 1000,
	.tdl1   = 0x31f,
	.tdl2   = 0,
};

static __maybe_unused struct acs3_plat_data bebop_sdhci2_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
	.mode_config = &bebop_sdhci2_25MHz,
	.nb_config   = 1,
};

/**********************
+ * TI wl18xx wlan
+ **********************/
static __initdata struct wl18xx_resources bebop_v1_wl18xx_res = {
	.wlirq_gpio = P7_GPIO_NR(67),
	.wlen_gpio = P7MU_IO_NR(4),
	.bt_rst_gpio = P7MU_IO_NR(3),
	.bt_uart_slot = 1,
	.wl_sdhci_slot = 0,
};

/**********************
+ * TI wl18xx wlan
+ **********************/
static __initdata struct wl18xx_resources bebop_v2_wl18xx_res = {
	.wlirq_gpio = P7_GPIO_NR(7),
	.wlen_gpio = P7MU_IO_NR(4),
	.bt_rst_gpio = P7MU_IO_NR(3),
	.bt_uart_slot = 1,
	.wl_sdhci_slot = 0,
};



/*****************************
 * P7MU Power Management Unit
 *****************************/

static struct pinctrl_map bebop_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

static struct p7mu_plat_data bebop_r23_p7mu_pdata = {
	.gpio       = -1,       /* GPIO 75 is P7MU -> P7 interrupt source */
	.int_32k    = false,    /* External 32kHz clock. */
	.int_32m    = false,    /* External 48mHz clock. */
};

static void __init bebop_init_p7mu(void)
{
	bebop_r23_p7mu_pdata.gpio = P7_GPIO_NR(73);
	p7_gpio_interrupt_register(bebop_r23_p7mu_pdata.gpio);

	/* I2CM0: P7MU */
	p7brd_init_i2cm(0, 100);

	p7_init_p7mu(0,
			&bebop_r23_p7mu_pdata,
			bebop_p7mu_pins,
			ARRAY_SIZE(bebop_p7mu_pins));
}

static int bebop_use_marvel_phy = 0;

static int __init bebop_activate_marvel_phy(char *unused)
{
	printk(KERN_INFO "Bebop: using Marvel phy\n");

	bebop_use_marvel_phy = 1;

	return 0;
}

early_param("bebop_use_marvel_phy", bebop_activate_marvel_phy);

static int bebop_use_nxp_phy = 0;

static int __init bebop_activate_nxp_phy(char *unused)
{
	printk(KERN_INFO "Bebop: using NXP phy\n");

	bebop_use_nxp_phy = 1;

	return 0;
}

early_param("bebop_use_nxp_phy", bebop_activate_nxp_phy);

static int bebop_use_broadcom_phy = 0;

static int __init bebop_activate_broadcom_phy(char *unused)
{
	printk(KERN_INFO "Bebop: using Broadcom phy\n");

	bebop_use_broadcom_phy = 1;

	return 0;
}

early_param("bebop_use_broadcom_phy", bebop_activate_broadcom_phy);


static int bebop_rev;

static void __init init_board(void)
{
	unsigned long fb_start = 0;
	unsigned long fb_size = 0;
	struct avifb_platform_data *bebop_avifb1_pdata;

	if (system_rev == 0) {
		/* bebop v2 by default */
		bebop_rev = 2;
	} else {
		bebop_rev = system_rev;
	}

	printk("Bebop revision: %d\n", bebop_rev);

	p7_init_mach();
	/* first init gpio controler to have gpio_to_irq working */
	p7_init_gpio(NULL, 0);

	bebop_init_p7mu();

	p7brd_init_nand(1);

	p7brd_init_uart(0,1);

	p7brd_init_i2cm(1, 200);
	p7brd_init_i2cm(2, 200);

	p7_gpio_interrupt_register(bebop_sdhci1_pdata.cd_gpio);
	p7brd_init_sdhci(1, &bebop_sdhci1_pdata, NULL, NULL,
			 &bebop_sdhci1_regulator_gpios, NULL, 0);
	p7brd_init_sdhci(2, &bebop_sdhci2_pdata, NULL, NULL, NULL, NULL, 0);

	/* USB0 is device only */
	p7brd_init_udc(0, -1);

	/* USB1 is host only */
	p7brd_init_hcd(1, -1);

	p7_init_avi();
	if (bebop_rev == 2) {
		p7_init_avifb(&bebop_avifb1_dev, &bebop_v2_avifb1_pdata,
								bebop_fb1_pins, ARRAY_SIZE(bebop_fb1_pins));
		bebop_avifb1_pdata = &bebop_v2_avifb1_pdata;
	} else {
		p7_init_avifb(&bebop_avifb1_dev, &bebop_v1_avifb1_pdata,
								bebop_fb1_pins, ARRAY_SIZE(bebop_fb1_pins));
		bebop_avifb1_pdata = &bebop_v1_avifb1_pdata;
	}
#if defined(CONFIG_GPU_PARROT7) || defined(CONFIG_GPU_PARROT7_MODULE)
	fb_start = bebop_avifb1_pdata->overlays[0].dma_memory.start;
	fb_size = bebop_avifb1_pdata->overlays[0].dma_memory.end - fb_start + 1;
	p7_init_gpu_fb(fb_start, fb_size, 4);
#endif
	p7_init_avi_voc(0, &bebop_avi_voc_param);

	p7_init_avi_m2m(bebop_avi_m2m_pdata);

	if (bebop_rev == 2) {
		p7_init_p7pwm(&bebop_v2_pwm_pdata,
								bebop_v2_pwm_pins,
								ARRAY_SIZE(bebop_v2_pwm_pins));
		p7_reserve_avicammem(&bebop_v2_cam0_dev, BEBOP_V2_CAM0_SIZE);
		p7_init_avicam(&bebop_v2_cam0_dev,
								&bebop_v2_cam0_pdata,
								bebop_v2_cam0_pins,
								ARRAY_SIZE(bebop_v2_cam0_pins));

		p7brd_export_gpio(P7_GPIO_NR(131), GPIOF_OUT_INIT_LOW, "cam0-nrst");

	} else {
		p7brd_export_gpio(P7_GPIO_NR(12), GPIOF_OUT_INIT_HIGH, "HDMI_IN_nRST");

		p7_init_avicam(&bebop_v1_cam0_dev,
								&bebop_v1_cam0_pdata,
								bebop_v1_cam0_pins,
								ARRAY_SIZE(bebop_v1_cam0_pins));
	}

	if (bebop_rev == 2) {
		if (bebop_use_nxp_phy) {

		} else if (bebop_use_broadcom_phy) {

		} else {
			/* Marvell phy */
			p7_init_ether(PHY_IFACE_RGMII,3, P7CTL_DRV_CFG(5));
		}
	} else {

		if (bebop_use_marvel_phy) {
			/* marvell phy */
			gpio_request_one(P7_GPIO_NR(217),
							GPIOF_OUT_INIT_LOW,
							"SWITCH ETH BR");
			p7_init_ether(PHY_IFACE_RGMII,3, P7CTL_DRV_CFG(5));
		} else {
			gpio_request_one(P7_GPIO_NR(217),
							GPIOF_OUT_INIT_HIGH,
							"SWITCH ETH BR");
			p7_init_ether(PHY_IFACE_RGMII,5, P7CTL_DRV_CFG(5));
		}
	}

	p7brd_export_gpio(P7_GPIO_NR(218), GPIOF_OUT_INIT_LOW, "ipod-rst");

	p7_init_spim(BEBOP_SPI_TUNER_CTRL_BUS, bebop_tuner_ctrl_pins,
	             ARRAY_SIZE(bebop_tuner_ctrl_pins), bebop_tuner_ctrl_swb);

	if (p7_init_spim_slave(BEBOP_SPI_TUNER_CTRL_BUS, &bebop_tuner_ctrl_info)) {
		pr_err("bebop: failed to initialize Tuner SPI master.\n");
	} else {
		p7_init_mpegts(0, &bebop_tuner0_pdata, bebop_tuner0_pins,
						ARRAY_SIZE(bebop_tuner0_pins));
		p7_init_mpegts(1, &bebop_tuner1_pdata, bebop_tuner1_pins,
						ARRAY_SIZE(bebop_tuner1_pins));
		p7brd_export_gpio(P7_GPIO_NR(92), GPIOF_OUT_INIT_LOW,
				  "octopus-pwr");
		p7brd_export_gpio(P7_GPIO_NR(219), GPIOF_OUT_INIT_LOW,
				  "octopus-rst");
	}

	p7_init_venc();
	p7_init_vdec();

	p7_init_temperature();

	if (bebop_rev == 2) {
		init_wl18xx(&bebop_v2_wl18xx_res, NULL, 0);
	} else {
		init_wl18xx(&bebop_v1_wl18xx_res, NULL, 0);
	}

	if (bebop_rev == 1) {
		i2c_register_board_info(2,
							bebop_siihdmi_board_info,
							ARRAY_SIZE(bebop_siihdmi_board_info));
	}
}

void __init bebop_reserve_mem(void)
{
	p7_reserve_avifbmem(&bebop_avifb1_dev,
	                    bebop_avi_lcd1_overlays,
	                    ARRAY_SIZE(bebop_avi_lcd1_overlays));

	if (bebop_rev == 1) {
		p7_reserve_avicammem(&bebop_v1_cam0_dev, BEBOP_V1_CAM0_SIZE);
	}

#define BEBOP_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(0, BEBOP_VOC_SIZE);

#define BEBOP_HX280_SIZE (CONFIG_ARCH_PARROT7_BEBOP_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(BEBOP_HX280_SIZE);

#define BEBOP_HX270_SIZE (CONFIG_ARCH_PARROT7_BEBOP_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(BEBOP_HX270_SIZE);

#define BEBOP_MPGTS_SIZE (CONFIG_ARCH_PARROT7_BEBOP_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, BEBOP_MPGTS_SIZE);
	p7_reserve_mpegtsmem(1, BEBOP_MPGTS_SIZE);

#define BEBOP_AVI_M2M_SIZE (1920 * 1080 * 4 * 4 * 2)
	p7_reserve_avi_m2m_mem(BEBOP_AVI_M2M_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_BEBOP, "bebop")
	.reserve        = &bebop_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END
