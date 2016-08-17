/*
 * linux/arch/arm/mach-parrot7/board-fc7100-vtna.c - VTNA board implementation
 *
 * Copyright 2015 (c) Parrot Automotive
 *
 * author:  Girardin Yann <yann.girardin.ext@parrot.com>
 * date:    26-January-2016
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
#include <mach/gpio.h>
#include <mach/ether.h>
#include "spi.h"
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>

#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>

#include "common.h"
#include "system.h"
#include "i2cm.h"
#include "gpio.h"
#include "sdhci.h"
#include "aai.h"
#include "mpegts.h"
#include "nand.h"
#include "usb.h"

#include "board-common.h"
#include "fc7100-module.h"
#include "fc7100-module-lcd.h"
#include "lcd-monspecs.h"
#include "vdec.h"
#include "avi.h"
#include "gpu.h"
#include "pinctrl.h"

static int hw_rev;

/*************
 * Versions
 *************/

#define EU_VERSION		1
#define US_VERSION		(1<<1)
#define ALL_VERSION		(EU_VERSION | US_VERSION)

/*************
 * GPIO
 *************/

#define IPOD_RST_N		160

#define O3_PWDN_N		161	/* Power down of Octopus 3 for FM/AM/WeatherBand */

#define XM_RST_N		56
#define XM_SHDN			155

#define	FC7_CODEC_RST_N		86	/* See FC7_CODEC_RST_N in HSIS */

#define USB_1_CP_EN		55
#define USB_HUB_RST_N		57

#define	FC7100_SERIALIZER_RST_N	88
#define CAM_FC7_RESET		154
#define IT_SCREEN		12	/* See 2C_TMS_IT_SCREEN on schematics
					   Shared with TMS */
struct gpio_setting {
	int gpio;
	char *name;
	int default_value;
	int interrupt;
	int version;
	int bidir;
};

struct gpio_setting vtna_gpios[] __initdata = {
	/* Audio codec (CS4245)*/
	{
		.gpio		= P7_GPIO_NR(FC7_CODEC_RST_N),
		.name		= "audio-codec-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
		.version	= ALL_VERSION,
	},
	/* Camera/TW8836 init */
	{
		.gpio		= P7_GPIO_NR(CAM_FC7_RESET),
		.name		= "cvbs-cam-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
		.version	= ALL_VERSION,
	},
	/* O3P2 Tuner init */
	{
		.gpio		= P7_GPIO_NR(O3_PWDN_N),
		.name		= "octopus-pwr",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
		.version	= ALL_VERSION,
	},
	/* XM Sirius Tuner init */
	{
		.gpio		= P7_GPIO_NR(XM_RST_N),
		.name		= "xms-tuner-rst-n",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
		.version	= US_VERSION,
	},
	{
		.gpio		= P7_GPIO_NR(XM_SHDN),
		.name		= "xms-shdn",
		.bidir		= 0,
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.version	= US_VERSION,
	},
	/* Ipod */
	{
		.gpio		= P7_GPIO_NR(IPOD_RST_N),
		.name		= "ipod-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
		.version	= ALL_VERSION,
	},
};

/*************
 * AVI CAM
 *************/
#define CAM_N_BUFFERS	8
#define CAM_PIXEL_SIZE	2

static unsigned long vtna_cam_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static u64 vtna_cam_dma_mask = DMA_BIT_MASK(32);


/**********************
 *  CVBS INPUT (IT656)
 **********************/
#define CAM0_HEIGHT	480
#define CAM0_WIDTH	720
#define CAM0_FRAME_SIZE PAGE_ALIGN(CAM0_WIDTH * CAM0_HEIGHT * CAM_PIXEL_SIZE)
#define CAM0_AVI_RAM_SIZE PAGE_ALIGN(CAM0_FRAME_SIZE * CAM_N_BUFFERS)

static struct pinctrl_map vtna_cvbs_input_pins_cfg_1[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLKa),
	P7_INIT_PINCFG(P7_CAM_0_CLKa, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA08a),
	P7_INIT_PINCFG(P7_CAM_0_DATA08a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA09a),
	P7_INIT_PINCFG(P7_CAM_0_DATA09a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA10a),
	P7_INIT_PINCFG(P7_CAM_0_DATA10a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA11a),
	P7_INIT_PINCFG(P7_CAM_0_DATA11a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA12a),
	P7_INIT_PINCFG(P7_CAM_0_DATA12a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA13a),
	P7_INIT_PINCFG(P7_CAM_0_DATA13a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA14a),
	P7_INIT_PINCFG(P7_CAM_0_DATA14a, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA15a),
	P7_INIT_PINCFG(P7_CAM_0_DATA15a, vtna_cam_pinconf),
};

static struct avicam_dummy_info vtna_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = CAM0_WIDTH,
		.height	    = CAM0_HEIGHT,
	},
	.fi = {
		.interval = {
			.numerator = 1,
			.denominator = 30,
		},
	},
};

static struct avicam_platform_data vtna_cvbs_input_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &vtna_cam0_dummy_driver_info,
};


static struct platform_device vtna_cvbs_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &vtna_cam_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


/**************************
 * OUT VD (LVDS IN, IT656)
 **************************/

#define CAM1_HEIGHT	576
#define CAM1_WIDTH	720
#define CAM1_FRAME_SIZE PAGE_ALIGN(CAM1_WIDTH * CAM1_HEIGHT * CAM_PIXEL_SIZE)
#define CAM1_AVI_RAM_SIZE PAGE_ALIGN(CAM1_FRAME_SIZE * CAM_N_BUFFERS)


static struct pinctrl_map vtna_lvds_input_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINCFG(P7_CAM_1_CLK, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA08),
	P7_INIT_PINCFG(P7_CAM_1_DATA08, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA09),
	P7_INIT_PINCFG(P7_CAM_1_DATA09, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA10),
	P7_INIT_PINCFG(P7_CAM_1_DATA10, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA11),
	P7_INIT_PINCFG(P7_CAM_1_DATA11, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA12),
	P7_INIT_PINCFG(P7_CAM_1_DATA12, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA13),
	P7_INIT_PINCFG(P7_CAM_1_DATA13, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA14),
	P7_INIT_PINCFG(P7_CAM_1_DATA14, vtna_cam_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA15),
	P7_INIT_PINCFG(P7_CAM_1_DATA15, vtna_cam_pinconf),
};

static struct avicam_dummy_info vtna_lvds_input_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = CAM1_WIDTH,
		.height	    = CAM1_HEIGHT,
	},
	.fi = {
		.interval = {
			.numerator = 1,
			.denominator = 30,
		},
	},
};

static struct avicam_platform_data vtna_lvds_input_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &vtna_lvds_input_dummy_driver_info,
};


static struct platform_device vtna_lvds_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &vtna_cam_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/*************
 * AVI : LCD
 *************/
static unsigned long vtna_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};
static unsigned long vtna_lcd_pinconf_hdrive[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(0)     | /* Slew rate 0 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static struct pinctrl_map vtna_avifb0_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, vtna_lcd_pinconf_hdrive),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, vtna_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, vtna_lcd_pinconf),
};

static struct avifb_overlay vtna_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.x	 = 64,
			.width	 = 800-64,
			.enabled = 1,
		},
		.zorder		 = 1,
		.dma_memory.end	 = (800-64) * 480 * 4 * 2,
	},
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.width	 = 64,
			.enabled = 1,
		},
		.zorder		 = 2,
		.dma_memory.end	 = 64 * 480 * 4 * 2,
	},
	{
		.layout 	= {
			.alpha 	 = AVI_ALPHA_OSD,
			.x	 = 64,
			.width	 = 800-64,
			.enabled = 1,
		},
		.zorder		= 0,
		.dma_memory.end	= (800-64) * 480 * 4 * 2,
	},
};

static struct avifb_platform_data vtna_avifb0_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
			.ihs	  = 1,
			.ivs	  = 1,
			.ipc	  = 1,
			.psync_en = 1,
			.psync_rf = 0,

		}},
        .lcd_format_control	= AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	= &tft800480_video_mode,
	.lcd_videomodes		= p7_all_video_modes,
	.caps			= AVI_CAP_LCD_1,

	/* default overlays */
        .overlays		= vtna_avi_lcd0_overlays,
	.overlay_nr		= ARRAY_SIZE(vtna_avi_lcd0_overlays),
	/* Use RGB black default pixel */
	.dpd                    = 0,
};

/* Should we bother setting a narrower mask here? */
static u64 vtna_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device vtna_avifb0_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &vtna_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avi_voc_plat_data vtna_avi_voc_param0 = {
	.display = "lcd.1",
};

static struct avi_voc_plat_data vtna_avi_voc_param1 = {
	.display = "lcd.1",
};

/*************
 * SD
 *************/

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>

static unsigned long vtna_sdhci_emmc_pinconf[] __initdata = {
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(0) | /* Slew rate 0 */
	P7CTL_DRV_CFG(2),  /* Drive strength 7 */
};
static struct pinctrl_map vtna_sdhci_emmc_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_2_CLK),
	P7_INIT_PINCFG(P7_SD_2_CLK, vtna_sdhci_emmc_pinconf),
	P7_INIT_PINMAP(P7_SD_2_CMD),
	P7_INIT_PINCFG(P7_SD_2_CMD, vtna_sdhci_emmc_pinconf),
	P7_INIT_PINMAP(P7_SD_2_DAT00),
	P7_INIT_PINCFG(P7_SD_2_DAT00, vtna_sdhci_emmc_pinconf),
	P7_INIT_PINMAP(P7_SD_2_DAT01),
	P7_INIT_PINCFG(P7_SD_2_DAT01, vtna_sdhci_emmc_pinconf),
	P7_INIT_PINMAP(P7_SD_2_DAT02),
	P7_INIT_PINCFG(P7_SD_2_DAT02, vtna_sdhci_emmc_pinconf),
	P7_INIT_PINMAP(P7_SD_2_DAT03),
	P7_INIT_PINCFG(P7_SD_2_DAT03, vtna_sdhci_emmc_pinconf)
};

static struct acs3_plat_data vtna_sdhci_emmc_pdata = {
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
 * SPIM
 *************/

static struct p7spi_ctrl_data o3p2_tuner_spim_cdata = {
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

static struct spi_board_info o3p2_tuner_spim_dev = {
	.modalias           = "spidev",
	.platform_data      = NULL,
	.controller_data    = &o3p2_tuner_spim_cdata,
	.irq                = -1,
	.max_speed_hz       = 100000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};

/*************
 * USB hub
 *************/

static struct smsc82514_pdata hub_init = {
	.us_port   = DS_HIGH,
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.ds_port_3 = DS_HIGH,
	.ds_port_4 = DS_HIGH,
	.reset_pin = P7_GPIO_NR(USB_HUB_RST_N),
};

static struct i2c_board_info __initdata vtna_smsc_82512_board_info[] = {
	{
		/* USB HUB SMSC 82514 */
		I2C_BOARD_INFO("smsc82514", 0x2c),
		.platform_data = &hub_init,
		.irq = -1,
	}
};

/*************
 * Audio
 *************/

static struct aai_pad_t vtna_aai_pads[] = {
	/* Codec & DAC out clocks */
	{AAI_SIG_MAIN_I2S_FRAME, 21, PAD_OUT}, /* I2S_FSYNC */
	{AAI_SIG_DAC_BIT_CLOCK,	 14, PAD_OUT}, /* I2S_BLK   */
	{AAI_SIG_MCLK,		 22, PAD_OUT}, /* I2S_MCLK  */

	/* I2S Input / Output */
	{AAI_SIG_I2S0_IN,	 23, PAD_IN },
	{AAI_SIG_IN_MIC0,	 23, PAD_IN }, /* CS4245 Line in/mic */
	{AAI_SIG_OUT_DAC0,	 10, PAD_OUT}, /* CS4245 Line out */
	{AAI_SIG_OUT_DAC1,	 19, PAD_OUT}, /* Front DAC out */
	{AAI_SIG_OUT_DAC2,	 11, PAD_OUT}, /* Rear  DAC out */
	{AAI_SIG_I2S2_IN,	 20, PAD_IN }, /* XM Sirius */

	/* PCM1 (Bluetooth for FC7100 HW04, HW05, HW06) */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	{-1,			 -1,       0}
};

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",	/* CS4245 Line out */
	"music-out-quad2",	/* Front&Rear DAC out */
        "voice-out-stereo",
        "pcm0-out",

	/* Input Channels */
	"music-in-stereo0",	/* CS4245 */
	"music-in-stereo2",	/* XM Sirius */
	"mic0-8k",
	"mic0-16k",
        "pcm0-in",
	"loopback1-8k",
	"loopback1-16k",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set vtna_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},
	{AAI_MASTER(1)},
	{AAI_MASTER(2)},
	/* voice output on DAC1 only, for front speakers */
	{VOI_MUX_DISABLE(0)},
	{VOI_MUX_DISABLE(2)},
	{VOI_MUX_DISABLE(3)},

	/*Don't remove*/
	{-1, 0, 0, 0},
};

static struct aai_platform_data vtna_aai_pdata = {
	.pad         = vtna_aai_pads,
	.aai_conf    = vtna_aai_conf_set,
	.device_list = aai_dev_list,
};

static unsigned long vtna_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long vtna_aai_ts_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long vtna_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static unsigned long vtna_aai_pcm_ts_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static struct pinctrl_map vtna_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, vtna_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, vtna_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, vtna_aai_pcm_ts_pinconf),
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, vtna_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, vtna_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_14),
	P7_INIT_PINCFG(P7_AAI_14, vtna_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, vtna_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, vtna_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_21),
	P7_INIT_PINCFG(P7_AAI_21, vtna_aai_ts_pinconf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, vtna_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_23),
	P7_INIT_PINCFG(P7_AAI_23, vtna_aai_pinconf),
};

/************************
 * LVDS serialiser config
 ************************/

#include <i2c/ti_ub925_lvds.h>

static struct dsr_i2c_cmd command_vvgt[] = {
	{ .reg = 0x21, .data = 0x1  }, /* GPO_REG7 set to '0'  */
	{ .reg = 0x20, .data = 0x10 }, /* GPO_REG6 set to '0'  */
	{ .reg = 0x2c, .data = 0x0f }, /* SSCG Enable, SSCG Frequency Deviation: +/- 2.5 */
	{ .reg = 0, .data = 0 },
};

static struct ti_lvds_platform_data ti_lvs_pdata = {
	.cmd = command_vvgt,
	.premap = {
		.slave_id = 0x4d,
		.slave_alias = 0x4d,
	},
	.nb_i2c_slave = 1,
	.clock_rising = 1,
};

static struct i2c_board_info __initdata vtna_lvds_board_info = {
	I2C_BOARD_INFO("lvds", 0xc),
	.irq = P7_GPIO_NR(IT_SCREEN),
	.platform_data = &ti_lvs_pdata,
};

/************************
 * Touchscreens
 ************************/

#include "input/touchscreen/atmel_mxt_ts.h"

static struct mxt_platform_data vtna_atmel_mxt_pdata = {
	.cfg_name = "maxtouch.cfg",
	.fw_name = "maxtouch.fw",
	.irqflags = IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata vtna_atmel_mxt_board_info = {
	I2C_BOARD_INFO("atmel_mxt_ts", 0x4d),
	.platform_data = &vtna_atmel_mxt_pdata,
	.irq = -1,
};

static int __init vtna_get_rev(void)
{
	int board_rev = 0, i, err, val;
	static const int gpios[] = {203, 204, 205, 206};

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {

		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
				P7CTL_PUD_CFG(DOWN), "volvo mb rev");

		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			board_rev |= (val << i);
			gpio_free(P7_GPIO_NR(gpios[i]));
		}
	}

	return board_rev;
}

static void __init init_board(void)
{
	int loop;

	unsigned int fb_nr = ARRAY_SIZE(vtna_avi_lcd0_overlays);
	unsigned long fb_start = vtna_avi_lcd0_overlays[fb_nr - 1].dma_memory.start;
	unsigned long fb_size = vtna_avi_lcd0_overlays[0].dma_memory.end - fb_start + 1;

	fc7100_init_module(0);

	hw_rev = vtna_get_rev();

	/* UART */
	p7brd_init_uart(7,0); /* Debug */

	/* I2C init */
	p7brd_init_i2cm(1, 200); /* CAM0/Video In, Ipod, GPS Gyro */
	p7brd_init_i2cm(2, 200); /* Audio Codec, SMSC 82512 USB Hub, LVDS, Atmel mxt224 */

	/* USB init */
	if (parrot_force_usb_device)
		p7brd_init_udc(0, -1);
	else
		p7brd_init_hcd(0, -1);

	p7brd_init_hcd(1, P7_GPIO_NR(USB_1_CP_EN));

	gpio_request_one(P7_GPIO_NR(USB_HUB_RST_N), GPIOF_OUT_INIT_LOW, "HUB USB RST");

	parrot_init_i2c_slave(2, &vtna_smsc_82512_board_info[0],
			      "smsc 82512", P7_I2C_NOIRQ);

	/* LCD / LVDS / Touchsreen / LCD */
	gpio_request_one(P7_GPIO_NR(FC7100_SERIALIZER_RST_N), GPIOF_OUT_INIT_HIGH, "ETH");
	parrot_init_i2c_slave(2,
	                      &vtna_lvds_board_info,
	                      "LVDS serializer",
			      P7_I2C_IRQ);
	parrot_init_i2c_slave(2,
	                      &vtna_atmel_mxt_board_info,
	                      "Atmel maXTouch",
			      P7_I2C_NOIRQ);

	p7_init_avi();
	/* AVI init */
	/* CAM0/Video In */
	p7_init_avicam(&vtna_cvbs_dev,
		       &vtna_cvbs_input_pdata,
		       vtna_cvbs_input_pins_cfg_1,
		       ARRAY_SIZE(vtna_cvbs_input_pins_cfg_1));

	/* LVDS IN */
	p7_init_avicam(&vtna_lvds_dev,
	               &vtna_lvds_input_pdata,
	               vtna_lvds_input_pins,
	               ARRAY_SIZE(vtna_lvds_input_pins));


	/* LCD */
	//fc7100_mezz_avi_init_lcd(vtna_avi_lcd0_overlays,
	//			 ARRAY_SIZE(vtna_avi_lcd0_overlays));
	p7_init_avifb(&vtna_avifb0_dev, &vtna_avifb0_pdata,
			vtna_avifb0_pins, ARRAY_SIZE(vtna_avifb0_pins));

	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &vtna_avi_voc_param0);
	p7_init_avi_voc(1, &vtna_avi_voc_param1);

	/* Audio init */
	/* AAI --> AUDIO CODEC (CS4245) --> MIC & LINE IN */
	/* AAI --> DAC (NAU84U02W G) --> AUDIO AMPLIFIER (TDA75610S) */
	p7_init_aai(vtna_aai_pins,
		    ARRAY_SIZE(vtna_aai_pins), &vtna_aai_pdata);

	/* eMMC init */
	p7brd_init_sdhci(2, &vtna_sdhci_emmc_pdata,
			 NULL, NULL, NULL,
			 vtna_sdhci_emmc_pins, ARRAY_SIZE(vtna_sdhci_emmc_pins));

	/* AM/FM Tuner init
	   SPI1 (spi0) : AM/FM Tuner */
	p7_init_spim_slave(0, &o3p2_tuner_spim_dev);
	fc7100_init_spim_single(0, 16, 17, 18, 19);

	/* XM Sirius Tuner init */
	p7brd_init_uart(6,0);

	/* Ethernet */
	p7_init_ether(PHY_IFACE_RGMII,-1, P7CTL_DRV_CFG(5));


	/* TMS */
	p7brd_init_uart(5,0);

	/* Miscelllaneous */
	p7_init_vdec();

	/* GPIO */
	/* Must be done after all IP init because some pin can be reconfigure as GPIO */
	/* See amfm-cs as example */
	for( loop = 0; loop < ARRAY_SIZE(vtna_gpios); loop ++){

		if( vtna_gpios[loop].interrupt )
			p7_gpio_interrupt_register(vtna_gpios[loop].gpio);

		p7brd_export_gpio(vtna_gpios[loop].gpio,
					vtna_gpios[loop].default_value,

					vtna_gpios[loop].name);

		if( vtna_gpios[loop].bidir ){
			gpio_unexport(vtna_gpios[loop].gpio);
			BUG_ON(gpio_export(vtna_gpios[loop].gpio, 1));
		}
	}

	/* Export unconfigured devices informations */
	p7brd_export_i2c_hw_infos(1, 0x10, "2C", "ipod");
}

static void __init vtna_reserve_mem(void)
{
	p7_reserve_avifbmem(&vtna_avifb0_dev, vtna_avi_lcd0_overlays,
			ARRAY_SIZE(vtna_avi_lcd0_overlays));

#define FC7100_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(0, FC7100_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, FC7100_VOC_SIZE);

	p7_reserve_avicammem(&vtna_cvbs_dev, CAM0_AVI_RAM_SIZE);
	p7_reserve_avicammem(&vtna_lvds_dev, CAM1_AVI_RAM_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);
#define FC7100_MPGTS_SIZE (CONFIG_ARCH_PARROT7_FC7100_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, FC7100_MPGTS_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}


P7_MACHINE_START(PARROT_VTNA, "VTNA")
	.reserve        = &vtna_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END
