/**
 * linux/arch/arm/mach-parrot7/board-porsche-rse.c - Parrot7 Porsche RSE
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Dec-2013
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
#include "common.h"
#include "system.h"
#include <gpio/p7-gpio.h>
#include "spi.h"
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>

#include "common.h"
#include "system.h"
#include "i2cm.h"
#include "gpio.h"
#include "sdhci.h"
#include "aai.h"
#include "venc.h"
#include "gpu.h"
#include "nand.h"
#include "usb.h"

#include "board-common.h"
#include "fc7100-module.h"
#include "fc7100-module-lcd.h"
#include "lcd-monspecs.h"
#include "gpio.h"
#include "sii_platform_data.h"

#include "vdec.h"

#include "avi.h"

#include <media/tw8834.h>
#include <media/mt9v117.h>


/*************
 * Audio (AAI)
 *************/

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",
        "pcm0-out",

	/*
	 * music-in-stereo0 is the codec (line_in and tv_audio)
	 * music-in-stereo1 is the dvd
	 * music-in-stereo2 is the cam
	 */
	"music-in-stereo0",
	"music-in-stereo1",
	"music-in-stereo2",
	"mic0-8k",
        "pcm0-in",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set prse_aai_conf_set[] = {
	/*
	 * P7 i2s is master for the codec
	 * P7 i2s is slave for the dvd
	 * P7 i2s is master for the cam
	 */
	{AAI_MASTER(0)},
	{AAI_SLAVE(1)},
	{AAI_SLAVE(2)},

	/* Don't remove */
	{-1, 0, 0, 0},
};

static struct aai_pad_t prse_aai_pads[] = {
	/* CODEC OUT CLKs */
	{AAI_SIG_DAC_BIT_CLOCK,	 10, PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME, 11, PAD_OUT},
	{AAI_SIG_MCLK,		 12, PAD_OUT},

	/* codec IN/OUT */
	{AAI_SIG_I2S0_IN,	 18, PAD_IN },
	{AAI_SIG_IN_MIC0,	 18, PAD_IN },
	{AAI_SIG_OUT_DAC0,	 24, PAD_OUT},

	/* dvd IN */
	{AAI_SIG_I2S1_FRAME,	 23, PAD_IN},
	{AAI_SIG_I2S1_IN,	 25, PAD_IN},

	/* CAM0 IN */
	{AAI_SIG_I2S2_FRAME,	 17, PAD_IN},
	{AAI_SIG_I2S2_IN,	 26, PAD_IN},

	/* PCM1 */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	{-1,			 -1,       0}
};

static struct aai_platform_data prse_aai_pdata = {
	.pad         = prse_aai_pads,
	.aai_conf    = prse_aai_conf_set,
	.device_list = aai_dev_list,
};

static unsigned long prse_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long prse_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long prse_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static unsigned long prse_aai_pcm_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static unsigned long prse_aai_in_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static unsigned long prse_aai_in_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static struct pinctrl_map prse_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, prse_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, prse_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, prse_aai_pcm_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, prse_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, prse_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, prse_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, prse_aai_in_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, prse_aai_in_pinconf),
	P7_INIT_PINMAP(P7_AAI_23),
	P7_INIT_PINCFG(P7_AAI_23, prse_aai_in_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_24),
	P7_INIT_PINCFG(P7_AAI_24, prse_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_25),
	P7_INIT_PINCFG(P7_AAI_25, prse_aai_in_pinconf),
	P7_INIT_PINMAP(P7_AAI_26),
	P7_INIT_PINCFG(P7_AAI_26, prse_aai_in_pinconf),
};

/*****************
 * WAU8822
 *****************/

#include <sound/soc.h>
static struct i2c_board_info __initdata fc7100_wau8822_board_info = {
	I2C_BOARD_INFO("wau8822", 0x1a),
	.irq = -1,
};

static struct snd_soc_dai_link parrot_fc7100_dai[] = {
	{
		.name           = "wau8822",
		.codec_name     = "wau8822.1-001a",
		.codec_dai_name = "wau8822-hifi",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.dai_fmt        = FC7100_DAIFMT,
	},
};

/* FC7100 ASoC device */
static struct platform_device fc7100_asoc_dev = {
	.name           = "parrot-fc7100-audio",
	.id             = 0,
	.dev		= {
		.platform_data = parrot_fc7100_dai,
	}
};

static struct acs3_plat_data fc7100_sdhci1_pdata = {
	.led_gpio   = -1,                               /* No activity led GPIO */
	.wp_gpio    = 134,
	.cd_gpio    = 135,
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

/* I2C Slave */
#include <i2c/plds_i2cs.h>

static struct plds_i2cs_pdata porsche_rse_i2cs_pdata = {
	.gpio_request   = P7_GPIO_NR(142),
	.gpio_reset     = P7_GPIO_NR(140),
	.i2c_bus        = 2,
};

/** Touchscreen **/

#include "input/touchscreen/atmel_mxt_ts.h"

static struct mxt_platform_data fc7100_atmel_mxt_pdata = {
	//.orient = MXT_NORMAL,
	.cfg_name = "maxtouch-prse.cfg",
	.fw_name = "maxtouch.fw",
	.irqflags = IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata fc7100_atmel_mxt_board_info = {
	I2C_BOARD_INFO("atmel_mxt_ts", 0x4c),
	.platform_data = &fc7100_atmel_mxt_pdata,
	.irq = P7_GPIO_NR(77),
};

/*******
  cam 5 : camera/dvd
  *****/
#define CAM5_HEIGHT	576
#define CAM5_WIDTH	720
#define CAM5_N_BUFFERS	16

#define CAM5_PIXEL_SIZE 2
#define CAM5_FRAME_SIZE PAGE_ALIGN(CAM5_WIDTH*CAM5_HEIGHT*CAM5_PIXEL_SIZE)
#define CAM5_AVI_RAM_SIZE PAGE_ALIGN(CAM5_FRAME_SIZE*CAM5_N_BUFFERS)

static unsigned long fc7100_prse_cam5_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(3),      /* Drive strength 3(reg=15) */
};

/* Cam 5 input declarations */
static struct pinctrl_map porsche_rse_cam5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINCFG(P7_CAM_5_CLK, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINCFG(P7_CAM_5_DATA00, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINCFG(P7_CAM_5_DATA01, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINCFG(P7_CAM_5_DATA02, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINCFG(P7_CAM_5_DATA03, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINCFG(P7_CAM_5_DATA04, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINCFG(P7_CAM_5_DATA05, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINCFG(P7_CAM_5_DATA06, fc7100_prse_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
	P7_INIT_PINCFG(P7_CAM_5_DATA07, fc7100_prse_cam5_pinconf)
};

#define TW8834_NRST_GPIO 150
#define CAM5_SWITCH_GPIO 138
#define CAM5_NRST_GPIO	 151
#define CAM5_SUBDEVS_I2C_BUS 1

static struct avicam_dummy_info porsche_rse_cam5_dummy_driver_info = {
	.dev_id = 5,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = CAM5_WIDTH,
		.height	    = CAM5_HEIGHT,
	},
};
/* mt9v117 subdev */
static int prse_mt9v117_power_on(void)
{
	gpio_set_value_cansleep(CAM5_NRST_GPIO, 1);
	return 0;
}

static int prse_mt9v117_power_off(void)
{
	gpio_set_value_cansleep(CAM5_NRST_GPIO, 0);
	return 0;
}

static int prse_mt9v117_set_power(int on)
{
	if(on == MT9V117_POWER_ON)
		return prse_mt9v117_power_on();
	else
		return prse_mt9v117_power_off();
}

#define MT9V117_INPUT_FREQ_HZ	27000000
static struct mt9v117_platform_data prse_mt9v117_platform_data = {
	.ext_clk_freq_hz = MT9V117_INPUT_FREQ_HZ,
	.enable_bt656    = 1,
	.set_power = &prse_mt9v117_set_power,
};

static struct i2c_board_info porsche_rse_mt9v117_i2c_devices[] = {
	{
		I2C_BOARD_INFO("mt9v117", 0x5D),
		.platform_data = &prse_mt9v117_platform_data,
	}
};

static struct avicam_subdevs porsche_rse_mt9v117_subdevs[] = {
	{
		.board_info = &porsche_rse_mt9v117_i2c_devices[0],
		.i2c_adapter_id = CAM5_SUBDEVS_I2C_BUS,
	},
	{ NULL, 0, },
};

/*tw8834 subdev */
static int prse_tw8834_power_on(void)
{
	gpio_set_value_cansleep(TW8834_NRST_GPIO, 1);
	return 0;
}

static int prse_tw8834_power_off(void)
{
	gpio_set_value_cansleep(TW8834_NRST_GPIO, 0);
	return 0;
}

/*tw8834 subdev */
static int prse_tw8834_power_input_enable(void)
{
	gpio_set_value_cansleep(CAM5_SWITCH_GPIO, 1);
	return 0;
}

static int prse_tw8834_power_input_disable(void)
{
	gpio_set_value_cansleep(CAM5_SWITCH_GPIO, 0);
	return 0;
}

static int prse_tw8834_set_power(int on)
{
	switch(on)	{
	case TW8834_POWER_OFF:
		return prse_tw8834_power_off();
	case TW8834_POWER_ON:
		return prse_tw8834_power_on();
	case TW8834_POWER_INPUT_DISABLE:
		return prse_tw8834_power_input_disable();
	case TW8834_POWER_INPUT_ENABLE:
		return prse_tw8834_power_input_enable();
	default:
		printk(KERN_ERR"bad power state for tw8834\n");
		break;
	}
	return -1;
}

static struct tw8834_platform_data prse_tw8834_platform_data = {
	.set_power = &prse_tw8834_set_power,
};

static struct i2c_board_info porsche_rse_cam5_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tw8834", 0x45),
		.platform_data = &prse_tw8834_platform_data,
	}
};

static struct avicam_subdevs porsche_rse_cam5_subdevs[] = {
	{
		.board_info = &porsche_rse_cam5_i2c_devices[0],
		.i2c_adapter_id = CAM5_SUBDEVS_I2C_BUS,
		.subdevs = porsche_rse_mt9v117_subdevs,
	},
	{ NULL, 0, },
};

static struct avicam_platform_data porsche_rse_cam5_pdata = {
	.cam_cap	   = AVI_CAP_CAM_5,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
		.ivs        = 1,
		.ihs        = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = porsche_rse_cam5_subdevs,
	.dummy_driver_info = &porsche_rse_cam5_dummy_driver_info,
};

static u64 porsche_rse_cam5_dma_mask = DMA_BIT_MASK(32);
static struct platform_device porsche_rse_cam5_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &porsche_rse_cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/**********
   cam0 : MHL
 *********/
#define CAM0_HEIGHT 1080
#define CAM0_WIDTH 1920
#define CAM0_N_BUFFERS 16

#define CAM0_PIXEL_SIZE 2
#define CAM0_FRAME_SIZE PAGE_ALIGN(CAM0_WIDTH * CAM0_HEIGHT * CAM0_PIXEL_SIZE)
#define CAM0_AVI_RAM_SIZE PAGE_ALIGN(CAM0_FRAME_SIZE * CAM0_N_BUFFERS)

#define HDMI_IN_CAMERA_INTERFACE	0
#define HDMI_IN_CAMERA_NODE		AVI_CAM0_NODE
#define HDMI_IN_CAMERA_IRQ		P7_CAM0_IRQ
#define HDMI_IN_CAMERA_I2C_BUS		1

static unsigned long fc7100_prse_cam0_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(3),      /* Drive strength 3(reg=15) */
};
static unsigned long fc7100_prse_cam0_clk_pinconf[] = {
       P7CTL_SMT_CFG(ON)    | /* shimmt trigger */
       P7CTL_PUD_CFG(DOWN)  | /* pull down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(3),      /* Drive strength 3(reg=15) */
};

static struct pinctrl_map porsche_rse_cam0_pins_cfg_1[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLKa),
	P7_INIT_PINCFG(P7_CAM_0_CLKa, fc7100_prse_cam0_clk_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINCFG(P7_CAM_0_DATA00, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINCFG(P7_CAM_0_DATA01, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINCFG(P7_CAM_0_DATA02, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINCFG(P7_CAM_0_DATA03, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINCFG(P7_CAM_0_DATA04, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINCFG(P7_CAM_0_DATA05, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINCFG(P7_CAM_0_DATA06, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINCFG(P7_CAM_0_DATA07, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA08a),
	P7_INIT_PINCFG(P7_CAM_0_DATA08a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA09a),
	P7_INIT_PINCFG(P7_CAM_0_DATA09a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA10a),
	P7_INIT_PINCFG(P7_CAM_0_DATA10a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA11a),
	P7_INIT_PINCFG(P7_CAM_0_DATA11a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA12a),
	P7_INIT_PINCFG(P7_CAM_0_DATA12a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA13a),
	P7_INIT_PINCFG(P7_CAM_0_DATA13a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA14a),
	P7_INIT_PINCFG(P7_CAM_0_DATA14a, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA15a),
	P7_INIT_PINCFG(P7_CAM_0_DATA15a, fc7100_prse_cam0_pinconf)
};

static struct pinctrl_map porsche_rse_cam0_pins_cfg_0[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINCFG(P7_CAM_0_CLK, fc7100_prse_cam0_clk_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINCFG(P7_CAM_0_DATA00, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINCFG(P7_CAM_0_DATA01, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINCFG(P7_CAM_0_DATA02, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINCFG(P7_CAM_0_DATA03, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINCFG(P7_CAM_0_DATA04, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINCFG(P7_CAM_0_DATA05, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINCFG(P7_CAM_0_DATA06, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINCFG(P7_CAM_0_DATA07, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINCFG(P7_CAM_0_DATA08, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINCFG(P7_CAM_0_DATA09, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINCFG(P7_CAM_0_DATA10, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINCFG(P7_CAM_0_DATA11, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINCFG(P7_CAM_0_DATA12, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINCFG(P7_CAM_0_DATA13, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINCFG(P7_CAM_0_DATA14, fc7100_prse_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINCFG(P7_CAM_0_DATA15, fc7100_prse_cam0_pinconf)
};

static struct avicam_dummy_info porsche_rse_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code	    = V4L2_MBUS_FMT_YUYV8_1X16,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = CAM0_WIDTH,
		.height	    = CAM0_HEIGHT,
	},
};

static struct avicam_platform_data porsche_rse_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 16,
	.subdevs	   = NULL,
	.dummy_driver_info = &porsche_rse_cam0_dummy_driver_info,
};


static u64 porsche_rse_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device porsche_rse_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &porsche_rse_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

void __init porsche_rse_avi_init_cameras(void)
{
	p7_init_avicam(&porsche_rse_cam5_dev,
	               &porsche_rse_cam5_pdata,
	               porsche_rse_cam5_pins,
	               ARRAY_SIZE(porsche_rse_cam5_pins));

	if (p7_chiprev() == P7_CHIPREV_R3)
		p7_init_avicam(&porsche_rse_cam0_dev,
		               &porsche_rse_cam0_pdata,
		               porsche_rse_cam0_pins_cfg_1,
		               ARRAY_SIZE(porsche_rse_cam0_pins_cfg_1));
	else
		p7_init_avicam(&porsche_rse_cam0_dev,
		               &porsche_rse_cam0_pdata,
		               porsche_rse_cam0_pins_cfg_0,
		               ARRAY_SIZE(porsche_rse_cam0_pins_cfg_0));
}

/********
  lcd
 *******/

static unsigned long prse_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};
static unsigned long prse_lcd_clk_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(4),      /* Drive strength 4(reg=31) */
};

static struct pinctrl_map prse_avifb0_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, prse_lcd_clk_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, prse_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, prse_lcd_pinconf),
};

/* Should we bother setting a narrower mask here? */
static u64 prse_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device prse_avifb0_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &prse_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avi_voc_plat_data prse_avi_voc_param0 = {
	.display = "lcd.1",
};

static struct avi_voc_plat_data prse_avi_voc_param1 = {
	.display = "lcd.1",
};

static struct avifb_overlay prse_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
	},
};
static struct avifb_platform_data prse_avifb0_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
			.ipc	  = 1, /* data on falling edge of pclk */
			.psync_en = 1, /* HS/VS follow psync_rf config */
			.psync_rf = 0, /* HS/VS on falling edge of pclk */
		}},
        .lcd_format_control	  = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	  = &porsche_rse_video_mode,
	.lcd_videomodes		  = p7_all_video_modes,
	/* default overlays */
        .overlays		  = prse_avi_lcd0_overlays,
	.overlay_nr		  = ARRAY_SIZE(prse_avi_lcd0_overlays),
	.caps                     = AVI_CAP_LCD_1 | AVI_CAP_GAM,
};

static unsigned long prse_sdhci_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(4),
	P7CTL_SLR_CFG(2),
};

static struct pinctrl_map prse_sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, prse_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, prse_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, prse_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, prse_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, prse_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, prse_sdhci_pins_config)
};

static struct platform_device virtual_wakeup_button = {
	.name           = "virtual_wakeup_button",
};

static struct sii5293_platform_data sii5293_pdata = {
	.version        = SII5293_PDATA_STRUCT_VERSION,
	.i2c_bus        = 1,
	.gpio_rst       = P7_GPIO_NR(146),
	.gpio_irq       = P7_GPIO_NR(148),
	.output_format  = 4,
	.input_dev_rap  = 1,
	.input_dev_rcp  = 1,
	.input_dev_ucp  = 1,
};

static struct platform_device sii5293_pdev = {
	.name           = "sii-5293",
	.id             = 0,
};

static void __init init_board(void)
{
	unsigned int flags = FC7100_MOD_SDCARD1;
	unsigned int fb_nr = ARRAY_SIZE(prse_avi_lcd0_overlays);
	unsigned long fb_start = prse_avi_lcd0_overlays[fb_nr - 1].dma_memory.start;
	unsigned long fb_size = prse_avi_lcd0_overlays[0].dma_memory.end - fb_start + 1;


	fc7100_init_module(flags);
	p7brd_init_i2cm(1, 100);

	/* tms uart */
	p7brd_init_uart(0, 0);

	p7_gpio_interrupt_register(fc7100_sdhci1_pdata.cd_gpio);
	p7brd_init_sdhci(1, &fc7100_sdhci1_pdata,
			 NULL, NULL, NULL,
			 prse_sdhci1_pins, ARRAY_SIZE(prse_sdhci1_pins));

	/* EHCI controller 0 is by default host and can be
	 * forced to device by cmdline option passed by installer
	 */
	//XXX gpio 84 control power for both ports
	//but according to hardware it is ok to keep it...
	p7brd_init_usb(0, -1, parrot_force_usb_device ? CI_UDC_DR_DEVICE : CI_UDC_DR_HOST);

	/* USB1 is host only */
	p7brd_init_usb(1, 84, CI_UDC_DR_HOST | CI_UDC_DR_DISABLE_HOST_WORKAROUND);
	/* hub nrst is inverted we can't pass it to p7brd_init_hcd */
	p7brd_export_gpio(144, GPIOF_OUT_INIT_LOW, "hub");

	/* Init sound */
	p7_init_aai(prse_aai_pins, ARRAY_SIZE(prse_aai_pins), &prse_aai_pdata);
	parrot_init_i2c_slave(1, &fc7100_wau8822_board_info,
			      "wau8822", P7_I2C_NOIRQ);
	p7_init_dev(&fc7100_asoc_dev, NULL, NULL, 0);

	/* Init P7-I2C Slave and PLDS DVD reader */
	gpio_request_one(P7_GPIO_NR(140), GPIOF_OUT_INIT_HIGH, "plds-rst");
	p7brd_init_i2cs(&porsche_rse_i2cs_pdata);

	/* user gpio */
	p7brd_export_gpio(133, GPIOF_OUT_INIT_LOW, "ipod-rst");

	/*Touchsreen */
	p7brd_export_gpio(155, GPIOF_OUT_INIT_HIGH, "touch-rst-n");
	parrot_init_i2c_slave(1,
	                      &fc7100_atmel_mxt_board_info,
	                      "Atmel maXTouch",
			      P7_I2C_IRQ);

	/* for debug, maybe to be removed later */
	p7brd_export_gpio(146, GPIOF_OUT_INIT_LOW, "cam0-nrst");
	p7brd_export_gpio(TW8834_NRST_GPIO, GPIOF_OUT_INIT_HIGH, "tw8834-nrst");
	p7brd_export_gpio(CAM5_NRST_GPIO, GPIOF_OUT_INIT_HIGH, "cam5-nrst");
	p7brd_export_gpio(CAM5_SWITCH_GPIO, GPIOF_OUT_INIT_LOW, "cam5-switch");
	p7brd_export_gpio(P7_GPIO_NR(136), GPIOF_IN, "tw8834-irq");

	/* mhl/hdmi chip SiI5293 */
	p7_gpio_interrupt_register(148);
	p7brd_export_gpio(82, GPIOF_OUT_INIT_LOW, "gpio82_mhl_pwr_en");

	p7_gpio_interrupt_register(P7_GPIO_NR(136));
	porsche_rse_cam5_i2c_devices[0].irq = gpio_to_irq(P7_GPIO_NR(136));

	gpio_request_one(P7_GPIO_NR(161), GPIOF_OUT_INIT_HIGH, "LCD stdy");
	msleep(50);
	gpio_request_one(P7_GPIO_NR(160), GPIOF_OUT_INIT_HIGH, "LCD nrst");

	p7_init_venc();
	p7_init_vdec();

	p7_init_avifb(&prse_avifb0_dev, &prse_avifb0_pdata,
		      prse_avifb0_pins, ARRAY_SIZE(prse_avifb0_pins));

	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &prse_avi_voc_param0);
	p7_init_avi_voc(1, &prse_avi_voc_param1);

	/*init cameras*/
	porsche_rse_avi_init_cameras();
	p7_init_avi();

	/* Export unconfigured devices informations */
	p7brd_export_i2c_hw_infos(1, 0x10, "2C", "ipod");

	p7brd_export_gpio(57, GPIOF_OUT_INIT_HIGH, "fc7100_ready");
	p7brd_export_gpio(58, GPIOF_IN, "tms_request");

	// Registering virtual wakeup button for resume
	// Needed by android
	platform_device_register(&virtual_wakeup_button);
	p7_init_dev(&sii5293_pdev, &sii5293_pdata, NULL, 0);
}

static void __init prse_reserve_mem(void)
{
#define FC7100_HX280_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(FC7100_HX280_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);

	p7_reserve_avifbmem(&prse_avifb0_dev, prse_avi_lcd0_overlays, ARRAY_SIZE(prse_avi_lcd0_overlays));

#define FC7100_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(0, FC7100_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, FC7100_VOC_SIZE);

	p7_reserve_avicammem(&porsche_rse_cam5_dev, CAM5_AVI_RAM_SIZE);
	p7_reserve_avicammem(&porsche_rse_cam0_dev, CAM0_AVI_RAM_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_PORSCHE_RSE, "PRSE")
	.reserve        = &prse_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END
