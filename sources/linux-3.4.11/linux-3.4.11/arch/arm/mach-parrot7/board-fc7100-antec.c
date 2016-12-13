/*
 * linux/arch/arm/mach-parrot7/board-antec.c - Antec board implementation
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Damien Riegel <damien.riegel.ext@parrot.com>
 * date:    15-May-2013
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
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
#include <media/adv7604.h>

#include "common.h"
#include "system.h"
#include "venc.h"
#include "vdec.h"
#include "avi.h"
#include "gpu.h"

#include "common.h"
#include "board-common.h"
#include "fc7100-module.h"
#include "fc7100-module-lcd.h"
#include "fc7100-mezzs.h" /* to allow to boot on mezz */
#include "lcd-monspecs.h"

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>
#include "aai.h"
#include "nand.h"
#include "usb.h"

/* spi */
static struct p7spi_ctrl_data fc7100_spim_cdata = {
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

static struct spi_board_info fc7100_spim_dev = {
	.modalias           = "spidev",
	.platform_data      = NULL,
	.controller_data    = &fc7100_spim_cdata,
	.irq                = -1,
	.max_speed_hz       = 100000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};


/* MMC1 on FC7100 R2 */
static struct acs3_plat_data fc7100_sdhci1_r23_pdata = {
	.led_gpio   = -1,                               /* No activity led GPIO */
	.wp_gpio    = -1,
	.cd_gpio    = -1,
	.rst_gpio   = 150,
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  =  MMC_CAP2_BOOTPART_NOACC,
};

/* aai */
static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",
	"music-out-stereo1",
	"voice-out-stereo",
	"pcm0-out",

	/* Input Channels */
	"music-in-stereo0",
	"mic0-8k",
	"mic0-16k",
	"pcm0-in",
	"loopback-8k",
	"loopback-16k",

	/* Don't remove */
	NULL,
};

static struct aai_pad_t antec_aai_pads[] = {
	{AAI_SIG_MCLK,		 24, PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME, 26, PAD_OUT},
	{AAI_SIG_DAC_BIT_CLOCK,  25, PAD_OUT},
	{AAI_SIG_I2S0_IN,	 12, PAD_IN },
	{AAI_SIG_IN_MIC0, 	 12, PAD_IN },
	{AAI_SIG_OUT_DAC0,	 22, PAD_OUT},
	{AAI_SIG_OUT_DAC1,	 14, PAD_OUT},

	/* PCM1 */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	{-1,			 -1,       0}
};

static struct aai_conf_set antec_aai_conf_set[] = {
	{RESET_VOI_MUX(0)},
	{VOI_MUX(0)},

	/* Don't remove */
	{-1, 0, 0, 0},
};

static struct aai_platform_data antec_aai_pdata = {
	.pad         = antec_aai_pads,
	.aai_conf    = antec_aai_conf_set,
	.device_list = aai_dev_list,

};

static unsigned long antec_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long antec_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long antec_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static unsigned long antec_aai_pcm_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static struct pinctrl_map antec_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, antec_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, antec_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, antec_aai_pcm_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, antec_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_14),
	P7_INIT_PINCFG(P7_AAI_14, antec_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, antec_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_24),
	P7_INIT_PINCFG(P7_AAI_24, antec_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_25),
	P7_INIT_PINCFG(P7_AAI_25, antec_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_26),
	P7_INIT_PINCFG(P7_AAI_26, antec_aai_frame_pinconf),
};

/* touchscreen */
static struct i2c_board_info __initdata fc7100_antec_touch = {
	I2C_BOARD_INFO("uc6511", 0x0a),
	.irq = 83,
};

/* lcd */
static struct avifb_overlay antec_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
	},
};

/***********************
 * Camera 0 configuration
 ***********************/

/**
 * HDMI Input configuration
 */
#define P7_CAM0_AVI_RAM_SIZE		(1280 * 720 * 4 * 4)

static unsigned long antec_cam_schmitt_off[] = {
	P7CTL_SMT_CFG(OFF),
};

static struct pinctrl_map antec_r3_cam0_pins_cfg_0[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINCFG(P7_CAM_0_CLK, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINCFG(P7_CAM_0_DATA08, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINCFG(P7_CAM_0_DATA09, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINCFG(P7_CAM_0_DATA10, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINCFG(P7_CAM_0_DATA11, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINCFG(P7_CAM_0_DATA12, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINCFG(P7_CAM_0_DATA13, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINCFG(P7_CAM_0_DATA14, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINCFG(P7_CAM_0_DATA15, antec_cam_schmitt_off),
};

static struct pinctrl_map antec_r3_cam0_pins_cfg_1[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINCFG(P7_CAM_0_CLKa, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA08a),
	P7_INIT_PINCFG(P7_CAM_0_DATA08a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA09a),
	P7_INIT_PINCFG(P7_CAM_0_DATA09a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA10a),
	P7_INIT_PINCFG(P7_CAM_0_DATA10a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA11a),
	P7_INIT_PINCFG(P7_CAM_0_DATA11a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA12a),
	P7_INIT_PINCFG(P7_CAM_0_DATA12a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA13a),
	P7_INIT_PINCFG(P7_CAM_0_DATA13a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA14a),
	P7_INIT_PINCFG(P7_CAM_0_DATA14a, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA15a),
	P7_INIT_PINCFG(P7_CAM_0_DATA15a, antec_cam_schmitt_off),
};

static struct avicam_dummy_info antec_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field = V4L2_FIELD_NONE,
		.width = 1280,
		.height = 720,
	},
};

static struct avicam_platform_data antec_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &antec_cam0_dummy_driver_info,
};

static u64 antec_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device antec_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &antec_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/* ADV7611 HDMI settings for FC7100 workbench */
#define ADV7611_CP_SLAVE_ADDRESS		0x44
#define ADV7611_HDMI_SLAVE_ADDRESS		0x68
#define ADV7611_EDID_SLAVE_ADDRESS		0x6C
#define ADV7611_REPEATER_SLAVE_ADDRESS		0x64
#define ADV7611_DPLL_SLAVE_ADDRESS		0x4c
#define ADV7611_INFOFRAME_SLAVE_ADDRESS		0x7C
#define ADV7611_CEC_SLAVE_ADDRESS		0x80

#define ADV7611_HDMI_NRST                       FC7100_IOEXPAND0_GPIO_NR(9)
#define ADV7611_HDMI_PWDN                       FC7100_IOEXPAND0_GPIO_NR(10)
#define ADV7611_HDMI_PWR_EN                     FC7100_IOEXPAND0_GPIO_NR(11)

static int adv7611_power_on(void)
{
	int ret;


	ret = gpio_request_one(ADV7611_HDMI_PWR_EN,
				GPIOF_OUT_INIT_HIGH,
				"cam0-5v_pwr_en");
	if (ret)
		goto bad_pwr;

	ret = gpio_request_one(ADV7611_HDMI_PWDN,
				GPIOF_OUT_INIT_HIGH,
				"cam0-pwdn");
	if (ret)
		goto bad_pwdn;


	ret = gpio_request_one(ADV7611_HDMI_NRST,
				GPIOF_OUT_INIT_HIGH,
				"cam0-nrst");
	if (ret)
		goto bad_reset;

	return 0;

 bad_reset:
	gpio_free(ADV7611_HDMI_PWDN);
 bad_pwdn:
	gpio_free(ADV7611_HDMI_PWR_EN);
 bad_pwr:
	pr_warning("failed to power adv7611 chip on\n");
	return ret;
}

static int adv7611_power_off(void)
{
	gpio_set_value_cansleep(ADV7611_HDMI_NRST, 0);
	gpio_free(ADV7611_HDMI_NRST);
	gpio_set_value_cansleep(ADV7611_HDMI_PWDN, 0);
	gpio_free(ADV7611_HDMI_PWDN);
	gpio_set_value_cansleep(ADV7611_HDMI_PWR_EN, 0);
	gpio_free(ADV7611_HDMI_PWR_EN);

	return 0;
}

static struct adv7604_platform_data fc7100_adv7604_platform_data = {
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
	.default_width		     = 1280,
	.default_height		     = 720,
	.cam_it			     = -1,
	.power_on                    = &adv7611_power_on,
	.power_off                   = &adv7611_power_off,
};

static struct i2c_board_info fc7100_mezz2_cam0_i2c_devices[] = {
	{
		I2C_BOARD_INFO("adv7611", 0x4C),
		.platform_data = &fc7100_adv7604_platform_data,
	}
};

static struct avicam_subdevs fc7100_mezz2_cam0_subdevs[] = {
	{
		.board_info = &fc7100_mezz2_cam0_i2c_devices[0],
		.i2c_adapter_id = 1,
	},
	{ NULL, 0, },
};


/***********************
 * Camera 1 configuration
 ***********************/

#define P7_CAM1_AVI_RAM_SIZE		(720 * 576 * 4 * 4)

static struct pinctrl_map antec_r3_cam1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINCFG(P7_CAM_1_CLK, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA08),
	P7_INIT_PINCFG(P7_CAM_1_DATA08, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA09),
	P7_INIT_PINCFG(P7_CAM_1_DATA09, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA10),
	P7_INIT_PINCFG(P7_CAM_1_DATA10, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA11),
	P7_INIT_PINCFG(P7_CAM_1_DATA11, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA12),
	P7_INIT_PINCFG(P7_CAM_1_DATA12, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA13),
	P7_INIT_PINCFG(P7_CAM_1_DATA13, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA14),
	P7_INIT_PINCFG(P7_CAM_1_DATA14, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA15),
	P7_INIT_PINCFG(P7_CAM_1_DATA15, antec_cam_schmitt_off),
};

static struct avicam_dummy_info antec_cam1_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field	    = V4L2_FIELD_NONE,
		.width	    = 720,
		.height	    = 576,
	},
};

static struct avicam_platform_data antec_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
		.ipc        = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &antec_cam1_dummy_driver_info,
};

static u64 antec_cam1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device antec_cam1_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &antec_cam1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/*******************************
 * Camera 5 input configuration
 *******************************/
#define P7_CAM5_AVI_RAM_SIZE (720 * 576 * 4 * 4)

static struct pinctrl_map antec_cam5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINCFG(P7_CAM_5_CLK, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINCFG(P7_CAM_5_DATA00, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINCFG(P7_CAM_5_DATA01, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINCFG(P7_CAM_5_DATA02, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINCFG(P7_CAM_5_DATA03, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINCFG(P7_CAM_5_DATA04, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINCFG(P7_CAM_5_DATA05, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINCFG(P7_CAM_5_DATA06, antec_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
	P7_INIT_PINCFG(P7_CAM_5_DATA07, antec_cam_schmitt_off),
};


static struct avicam_dummy_info antec_cam5_dummy_driver_info = {
	.dev_id = 5,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = 704,
		.height	    = 480,
	},
};

static struct avicam_platform_data antec_cam5_pdata = {
	.cam_cap	    = AVI_CAP_CAM_5,
	.interface	    = {
		.itu656	    = 1,
		.pad_select = 0,
		.ipc        = 1,
	},
	.bus_width	    = 8,
	.subdevs	    = NULL,
	.dummy_driver_info  = &antec_cam5_dummy_driver_info,
};

static u64 antec_cam5_dma_mask = DMA_BIT_MASK(32);
static struct platform_device antec_cam5_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &antec_cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/* Register devboard sensors */

void __init antec_avi_init_cameras(void)
{
	p7_init_avi();

	if (p7_chiprev() == P7_CHIPREV_R3)
		p7_init_dev(&antec_cam0_dev, &antec_cam0_pdata,
			    antec_r3_cam0_pins_cfg_1, ARRAY_SIZE(antec_r3_cam0_pins_cfg_1));
	else
		p7_init_dev(&antec_cam0_dev, &antec_cam0_pdata,
			    antec_r3_cam0_pins_cfg_0, ARRAY_SIZE(antec_r3_cam0_pins_cfg_0));

	p7_init_dev(&antec_cam1_dev, &antec_cam1_pdata,
		    antec_r3_cam1_pins, ARRAY_SIZE(antec_r3_cam1_pins));

	p7_init_dev(&antec_cam5_dev, &antec_cam5_pdata,
            antec_cam5_pins, ARRAY_SIZE(antec_cam5_pins));
}

/***************************
 * Framebuffer configuration
 ***************************/
static unsigned long antec_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};
static unsigned long antec_lcd_pinconf_hdrive[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(4),      /* Drive strength 4(reg=31) */
};

static struct pinctrl_map antec_avifb_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, antec_lcd_pinconf_hdrive),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, antec_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, antec_lcd_pinconf),
};

static struct avifb_overlay antec_avifb_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = 800 * 480 * 4 * 2,
	},
};

static struct avifb_platform_data antec_avifb_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
			.ipc      = 1,
		}},
	.caps			  = AVI_CAP_LCD_1,
        .lcd_format_control	  = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	  = &chimei_antec_video_mode,
	.lcd_videomodes		  = p7_all_video_modes,
	/* default overlays */
        .overlays		  = antec_avifb_overlays,
	.overlay_nr		  = ARRAY_SIZE(antec_avifb_overlays),
};

/* Should we bother setting a narrower mask here? */
static u64 antec_avifb_dma_mask = DMA_BIT_MASK(32);
static struct platform_device antec_avifb_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &antec_avifb_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avi_voc_plat_data antec_avi_voc_param0 = {
	.display = "lcd.1",
};

static struct avi_voc_plat_data antec_avi_voc_param1 = {
	.display = "lcd.1",
};

#define SCREEN_ON_GPIO P7_GPIO_NR(145)
static void fc7100_antec_power_off(void)
{
	gpio_set_value(SCREEN_ON_GPIO, 0);
}

static void __init init_board_antec(void)
{
	unsigned long fb_start = 0;
	unsigned long fb_size = 0;
	/* fc7100_init_module is allready done */

	/* i2c1 is init in board autodetection */
	p7brd_init_i2cm(2, 100);

	p7brd_init_uart(0,1);

	p7brd_export_gpio(141, GPIOF_OUT_INIT_LOW, "ipod-rst");
	p7brd_init_sdhci(1, &fc7100_sdhci1_r23_pdata,
			 NULL, NULL, NULL, NULL, 0);

	/* USB0 is device only */
	p7brd_init_udc(0, -1);

	/* USB1 is host only */
	p7brd_init_hcd(1, 139);

	p7_init_aai(antec_aai_pins,
		    ARRAY_SIZE(antec_aai_pins), &antec_aai_pdata);
	/* CODEC are driven by client */

	parrot_init_i2c_slave(2,
	                      &fc7100_antec_touch,
	                      "Antec touchscreen",
			      P7_I2C_IRQ);

	/* manual cs */
	fc7100_init_spim_single(0, 16, -1/*18*/, 17, 19);
	p7brd_export_gpio(P7_GPIO_NR(138), GPIOF_IN, "mcu-irq");
	p7_gpio_interrupt_register(P7_GPIO_NR(138));
	p7brd_export_gpio(P7_GPIO_NR(136), GPIOF_INIT_HIGH, "mcu-cs");

	p7_init_spim_slave(0, &fc7100_spim_dev);

	p7_init_vdec();

	antec_avi_init_cameras();

	p7_init_avifb(&antec_avifb_dev, &antec_avifb_pdata,
		      antec_avifb_pins, ARRAY_SIZE(antec_avifb_pins));

	fb_start = antec_avifb_pdata.overlays[0].dma_memory.start;
	fb_size =  antec_avifb_pdata.overlays[0].dma_memory.end - fb_start + 1;
	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &antec_avi_voc_param0);
	p7_init_avi_voc(1, &antec_avi_voc_param1);

	/* Set screen on , and register power off callback */
	p7brd_export_gpio(SCREEN_ON_GPIO, GPIOF_OUT_INIT_HIGH, "screen-on");
	pm_power_off = fc7100_antec_power_off;
}

static void __init init_board(void)
{
	unsigned int flags = FC7100_MOD_SDCARD1;
	int err;

	fc7100_init_module(flags);
	err = fc7100_init_mezz(flags, NULL);
	if (err < 0) {
		init_board_antec();
		return;
	}

	pr_notice("Antec board is on dev. board\n");
	antec_cam0_pdata.dummy_driver_info = NULL;
	antec_cam0_pdata.subdevs = fc7100_mezz2_cam0_subdevs;
	antec_avi_init_cameras();
	fc7100_mezz_avi_init_lcd_screen("chimei-antec", NULL, NULL);
	fc7100_mezz_avi_init_lcd(antec_avi_lcd0_overlays,
				 ARRAY_SIZE(antec_avi_lcd0_overlays));

	/* spi init with comm with antec mcu */
	fc7100_init_spim_single(0, 11, -1/*10*/, 8, 9);
	p7brd_export_gpio(P7_GPIO_NR(154), GPIOF_IN, "mcu-irq");
	p7_gpio_interrupt_register(P7_GPIO_NR(154));
	p7brd_export_gpio(P7_GPIO_NR(87), GPIOF_INIT_HIGH, "mcu-cs");
	p7_init_spim_slave(0, &fc7100_spim_dev);
}

static void __init antec_reserve_mem(void)
{
	/* lcd */
	fc7100_mezz_reserve_mem_for_lcd(antec_avi_lcd0_overlays, ARRAY_SIZE(antec_avi_lcd0_overlays));
	p7_reserve_avifbmem(&antec_avifb_dev,
			    antec_avifb_overlays,
			    ARRAY_SIZE(antec_avifb_overlays));

#define ANTEC_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(0, ANTEC_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, ANTEC_VOC_SIZE);

	/* camera */
	p7_reserve_avicammem(&antec_cam0_dev, P7_CAM0_AVI_RAM_SIZE);
	p7_reserve_avicammem(&antec_cam1_dev, P7_CAM1_AVI_RAM_SIZE);
	p7_reserve_avicammem(&antec_cam5_dev, P7_CAM5_AVI_RAM_SIZE);

#define FC7100_HX280_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(FC7100_HX280_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}


P7_MACHINE_START(PARROT_ANTEC, "Antec")
	.reserve        = &antec_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END

