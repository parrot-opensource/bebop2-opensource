/**
 * linux/arch/arm/mach-parrot7/board-dtv.c - Parrot7 DTV
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Nov-2013
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
#include <linux/persistent_ram.h>
#include <linux/ramoops.h>
#include "common.h"
#include "system.h"
#include "i2cm.h"
#include "gpio.h"
#include "sdhci.h"
#include "aai.h"
#include "avi.h"
#include "mpegts.h"
#include "vdec.h"
#include "gpu.h"
#include "nand.h"
#include "usb.h"

#include "board-common.h"
#include "fc7100-module.h"
#include "lcd-monspecs.h"
#include <media/tw9990.h>
#include <media/adv7391.h>

/******************************/
/* Ramoops and persistent ram */
/******************************/

#define RAMOOPS_SIZE  SZ_512K
#define RAMOOPS_RECORD_SIZE  SZ_128K

static struct ramoops_platform_data ramoops_data = {
        .mem_size               = RAMOOPS_SIZE,
        .record_size            = RAMOOPS_RECORD_SIZE, /* size of each dump done on oops/panic */
        .dump_oops              = 1,      /* set to 1 to dump oopses, 0 to only dump panics */
};

static struct platform_device ramoops_dev = {
        .name = "ramoops",
        .dev = {
                .platform_data = &ramoops_data,
        },
};

/*************
 * Audio (AAI)
 *************/

static struct aai_pad_t dtv_aai_pads[] = {
	/* CODEC OUT CLKs */
	{AAI_SIG_DAC_BIT_CLOCK,  10,  PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME, 11,  PAD_OUT},
	{AAI_SIG_MCLK,           12,  PAD_OUT},

	/* codec IN/OUT */
	{AAI_SIG_OUT_DAC0,       14,  PAD_OUT},

	{-1,			 -1,        0}
};

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set dtv_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},

	/* Don't remove */
	{-1, 0, 0, 0},
};

static struct aai_platform_data dtv_aai_pdata = {
	.pad         = dtv_aai_pads,
	.aai_conf    = dtv_aai_conf_set,
	.device_list = aai_dev_list,
};

static unsigned long dtv_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long dtv_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static struct pinctrl_map dtv_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, dtv_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, dtv_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, dtv_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_14),
	P7_INIT_PINCFG(P7_AAI_14, dtv_aai_pinconf),
};

/*************
 * SPIM
 *************/

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
	.max_speed_hz       = 40000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};

/**********
  Cam 5: Video 2 input
 **********/
#define CAM1_HEIGHT 576
#define CAM1_WIDTH	720
#define CAM1_N_BUFFERS 4

#define CAM1_PIXEL_SIZE 2
#define CAM1_FRAME_SIZE PAGE_ALIGN(CAM1_WIDTH*CAM1_HEIGHT*CAM1_PIXEL_SIZE)
#define CAM1_AVI_RAM_SIZE PAGE_ALIGN(CAM1_FRAME_SIZE*CAM1_N_BUFFERS)

static struct pinctrl_map dtv_cam1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINMAP(P7_CAM_1_DATA08),
	P7_INIT_PINMAP(P7_CAM_1_DATA09),
	P7_INIT_PINMAP(P7_CAM_1_DATA10),
	P7_INIT_PINMAP(P7_CAM_1_DATA11),
	P7_INIT_PINMAP(P7_CAM_1_DATA12),
	P7_INIT_PINMAP(P7_CAM_1_DATA13),
	P7_INIT_PINMAP(P7_CAM_1_DATA14),
	P7_INIT_PINMAP(P7_CAM_1_DATA15),
};

#define CAM1_NRST                       P7_GPIO_NR(141)
#define CAM1_PWDN                       P7_GPIO_NR(145)
#define CAM1_I2C_BUS			1

static int cam1_power_on(void)
{
	gpio_set_value_cansleep(CAM1_NRST, 1);
	gpio_set_value_cansleep(CAM1_PWDN, 0);
	return 0;
}

static int cam1_power_off(void)
{
	gpio_set_value_cansleep(CAM1_NRST, 0);
	gpio_set_value_cansleep(CAM1_PWDN, 1);

	return 0;
}

static int cam1_set_power(int on)
{
	if(on)
		return cam1_power_on();
	else
		return cam1_power_off();
}

static struct tw9990_platform_data cam1_tw9990_platform_data = {
	.set_power    = cam1_set_power,
	.differential_input = TW9990_DIFFERENTIAL_ENABLED,
	.anti_aliasing = TW9990_ANTI_ALIAS_ENABLED,
	.power_saving = TW9990_POWER_SAVE_ENABLED,
	.synct_mode = TW9990_SYNCT_FORCED,
	.sync_pulse_amplitude = 0x2E,
	.common_mode_clamp = TW9990_CM_RESTORE | TW9990_CM_RSTR_FULL_TIME
			   | TW9990_CM_CUR_MODE | TW9990_CM_TOGGLE,
};

static struct i2c_board_info cam1_tw9990_device[] = {
	{
		I2C_BOARD_INFO("tw9990", 0x44),
		.platform_data = &cam1_tw9990_platform_data,
	},
};

static struct avicam_subdevs dtv_cam1_subdevs[] = {
	{
		.board_info = &cam1_tw9990_device[0],
		.i2c_adapter_id = CAM1_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_dummy_info dtv_cam1_dummy_driver_info = {
	.dev_id = 5,
	.format = {
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field = V4L2_FIELD_INTERLACED,
		.width = CAM1_WIDTH,
		.height = CAM1_HEIGHT,
	},
};

static struct avicam_platform_data dtv_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &dtv_cam1_dummy_driver_info,
	.subdevs	   = dtv_cam1_subdevs,
};

static u64 dtv_cam1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device dtv_cam1_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &dtv_cam1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/******
 Camera 1 : Video 1 Input
 *****/
#define CAM5_HEIGHT 576
#define CAM5_WIDTH  720
#define CAM5_N_BUFFERS 4

#define CAM5_PIXEL_SIZE 2

#define CAM5_FRAME_SIZE PAGE_ALIGN(CAM5_WIDTH*CAM5_HEIGHT*CAM5_PIXEL_SIZE)
#define CAM5_AVI_RAM_SIZE PAGE_ALIGN(CAM5_FRAME_SIZE*CAM5_N_BUFFERS)


#define CAM5_NRST                       P7_GPIO_NR(142)
#define CAM5_PWDN                       P7_GPIO_NR(146)
#define CAM5_I2C_BUS			1

static struct pinctrl_map dtv_cam5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
};

static int cam5_power_on(void)
{
	gpio_set_value_cansleep(CAM5_NRST, 1);
	gpio_set_value_cansleep(CAM5_PWDN, 0);
	return 0;
}

static int cam5_power_off(void)
{
	gpio_set_value_cansleep(CAM5_NRST, 0);
	gpio_set_value_cansleep(CAM5_PWDN, 1);

	return 0;
}

static int cam5_set_power(int on)
{
	if(on)
		return cam5_power_on();
	else
		return cam5_power_off();
}

static struct tw9990_platform_data cam5_tw9990_platform_data = {
	.set_power    = cam5_set_power,
	.differential_input = TW9990_DIFFERENTIAL_ENABLED,
	.anti_aliasing = TW9990_ANTI_ALIAS_ENABLED,
	.power_saving = TW9990_POWER_SAVE_ENABLED,
	.synct_mode = TW9990_SYNCT_FORCED,
	.sync_pulse_amplitude = 0x2E,
	.common_mode_clamp = TW9990_CM_RESTORE | TW9990_CM_RSTR_FULL_TIME
			   | TW9990_CM_CUR_MODE | TW9990_CM_TOGGLE,
};

static struct i2c_board_info cam5_tw9990_device[] = {
	{
		I2C_BOARD_INFO("tw9990", 0x45),
		.platform_data = &cam5_tw9990_platform_data,
	},
};

static struct avicam_subdevs dtv_cam5_subdevs[] = {
	{
		.board_info = &cam5_tw9990_device[0],
		.i2c_adapter_id = CAM5_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_dummy_info dtv_cam5_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field = V4L2_FIELD_INTERLACED,
		.width = CAM5_WIDTH,
		.height = CAM5_HEIGHT,
	},
};

static struct avicam_platform_data dtv_cam5_pdata = {
	.cam_cap	   = AVI_CAP_CAM_5,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &dtv_cam5_dummy_driver_info,
	.subdevs	   = dtv_cam5_subdevs,
};

static u64 dtv_cam5_dma_mask = DMA_BIT_MASK(32);
static struct platform_device dtv_cam5_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &dtv_cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/*****
  video out
 ****/
static unsigned long dtv_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* pull up/down disable */
       P7CTL_DRV_CFG(4),      /* Drive strength */
};

static struct pinctrl_map dtv_avifb1_pins[] __initdata = {
	/* LCD0 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK,    dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, dtv_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, dtv_lcd_pinconf),
};

static struct avifb_overlay dtv_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = 720 * 480 * 4 * 2,
	},
};

static struct avifb_platform_data dtv_avifb1_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 1,
		}},
        .lcd_format_control	  = AVI_FORMAT_CONTROL_UYVY_2X8,
	.lcd_default_videomode	  = &ntsc_720_video_mode,
	.lcd_videomodes		  = p7_all_video_modes,
	.caps                     = AVI_CAP_LCD_1,
	.output_cspace		  = AVI_BT601_CSPACE,
	/* default overlays */
        .overlays		  = dtv_avi_lcd0_overlays,
	.overlay_nr		  = ARRAY_SIZE(dtv_avi_lcd0_overlays),
};

static u64 dtv_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device dtv_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		.dma_mask           = &dtv_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avi_voc_plat_data dtv_avi_voc_param0 = {
	.display = "lcd.1",
};

static struct avi_voc_plat_data dtv_avi_voc_param1 = {
	.display = "lcd.1",
};

/* config come from datasheet :
   Table 79. 8-Bit NTSC Square Pixel YCrCb In (EAV/SAV), CVBS/Y-C Out
 */
struct adv7391_platform_data adv7391_pdata = {
	.dac_pwr = ADV7391_FLG_DAC1_PWR, /* enable DAC1 only */
	.dac_gain = 3, /* gain = + 0.054 % */
	.sd_mode1 = ADV7391_FLG_LUMA_SSAF,
	.sd_mode2 = (ADV7391_FLG_PRPB_SSAF_FILT
			| ADV7391_FLG_DAC_OUT1
			| ADV7391_FLG_PEDESTAL
			| ADV7391_FLG_PIX_DT_VALID
			| ADV7391_FLG_VID_EDG_CTL),
	.sd_mode3 = ADV7391_FLG_Y_OUT_LVL,
	.scale_lsb = 0x3C,
};

static struct i2c_board_info lcd_adv7391_device[] = {
	{
		I2C_BOARD_INFO(ADV7391_NAME, 0x2A),
		.platform_data = &adv7391_pdata,
	},
};

static void dtv_init_lcd(void)
{
	gpio_request_one(147, GPIOF_OUT_INIT_HIGH, "lcd nrst");
	parrot_init_i2c_slave(1, &lcd_adv7391_device[0],
			"adv7391", P7_I2C_NOIRQ);
}

/******
 * Ram to Ram configuration
 *****/
static struct avi_m2m_platform_data fc7100_dtv_avi_m2m_pdata[] = {
	{ .caps = AVI_CAP_PLANAR | AVI_CAP_SCAL | AVI_CAP_CONV },
	{ .caps = 0 },
};

/* VOC connected to RAM2RAM */
static struct avi_voc_plat_data fc7100_dtv_avi_voc_m2m = {
	.display = "m2m.0",
};


static void __init init_board(void)
{
	unsigned int			 flags = 0;
	unsigned long fb_start = 0;
	unsigned long fb_size  = 0;
	int ret=0;

	fc7100_init_module(flags);
	
	/* Need CONFIG_RAMOOPS in kernel config */
	ret = platform_device_register(&ramoops_dev);
	if (ret) {
		printk(KERN_ERR "unable to register ramoops platform device\n");
	}


	/* tms uart */
	p7brd_init_uart(0, 0);
	p7brd_export_gpio(57, GPIOF_OUT_INIT_HIGH, "fc7100_ready");
	p7brd_export_gpio(58, GPIOF_IN, "tms_request");

	if (parrot_force_usb_device)
		p7brd_init_udc(0, P7_GPIO_NR(84));
	else
		p7brd_init_hcd(0, P7_GPIO_NR(84));
	p7brd_init_i2cm(1, 200);


	/* Init sound */
	p7_init_aai(dtv_aai_pins, ARRAY_SIZE(dtv_aai_pins), &dtv_aai_pdata);
	p7brd_export_gpio(P7_GPIO_NR(137), GPIOF_OUT_INIT_HIGH, "codec-mute");

	/* octopus pwdn gpio */
	p7_init_spim_slave(0, &fc7100_spim_dev);
	fc7100_init_spim_single(0, 1, 0, 3, 2);
	fc7100_init_mpegts_single(0, 11, 10);
	fc7100_init_mpegts_single(1, 9, 8);
	p7brd_export_gpio(P7_GPIO_NR(82), GPIOF_OUT_INIT_LOW,  "octopus-pwr");
	p7brd_export_gpio(P7_GPIO_NR(81), GPIOF_DIR_IN,        "octopus-int");

	p7_init_vdec();

	dtv_init_lcd();
	p7_init_avifb(&dtv_avifb1_dev, &dtv_avifb1_pdata,
	              dtv_avifb1_pins, ARRAY_SIZE(dtv_avifb1_pins));

	fb_start = dtv_avi_lcd0_overlays[0].dma_memory.start;
	fb_size = dtv_avi_lcd0_overlays[0].dma_memory.end - fb_start + 1;
	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &dtv_avi_voc_param0);
	p7_init_avi_voc(1, &dtv_avi_voc_param1);

	p7_init_avi();
	p7_init_avi_m2m(fc7100_dtv_avi_m2m_pdata);
	p7_init_avi_voc(2, &fc7100_dtv_avi_voc_m2m);

	p7brd_export_gpio(P7_GPIO_NR(CAM5_PWDN), GPIOF_OUT_INIT_LOW, "cam5-pwdn");
	p7brd_export_gpio(P7_GPIO_NR(CAM5_NRST), GPIOF_OUT_INIT_LOW, "cam5-nrst");
	p7brd_export_gpio(P7_GPIO_NR(140), GPIOF_IN, "cam5-irq");
	p7brd_export_gpio(P7_GPIO_NR(CAM1_PWDN), GPIOF_OUT_INIT_LOW, "cam1-pwdn");
	p7brd_export_gpio(P7_GPIO_NR(CAM1_NRST), GPIOF_OUT_INIT_LOW, "cam1-nrst");
	p7brd_export_gpio(P7_GPIO_NR(139), GPIOF_IN, "cam1-irq");
	p7_gpio_interrupt_register(P7_GPIO_NR(140));
	p7_gpio_interrupt_register(P7_GPIO_NR(139));

	cam5_tw9990_device[0].irq = gpio_to_irq(P7_GPIO_NR(140));
	cam1_tw9990_device[0].irq = gpio_to_irq(P7_GPIO_NR(139));
	p7_init_avicam(&dtv_cam1_dev, &dtv_cam1_pdata,
	               dtv_cam1_pins, ARRAY_SIZE(dtv_cam1_pins));

	p7_init_avicam(&dtv_cam5_dev, &dtv_cam5_pdata,
	               dtv_cam5_pins, ARRAY_SIZE(dtv_cam5_pins));

}

static void __init dtv_reserve_mem(void)
{
	struct membank *bank = &meminfo.bank[0];
	ramoops_data.mem_address = bank->start + bank->size-RAMOOPS_SIZE;
	p7_reserve_devmem(&ramoops_dev,
			  (dma_addr_t *)&ramoops_data.mem_address,
			  (size_t *)&ramoops_data.mem_size);

	p7_reserve_avifbmem(&dtv_avifb1_dev, dtv_avi_lcd0_overlays, ARRAY_SIZE(dtv_avi_lcd0_overlays));

#define FC7100_VOC_SIZE (640 * 480 * 4 * 2)
	p7_reserve_avi_voc_mem(0, FC7100_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, FC7100_VOC_SIZE);
	p7_reserve_avi_voc_mem(2, FC7100_VOC_SIZE);

#define FC7100_AVI_M2M_SIZE (640 * 480 * 4 * 2)
	p7_reserve_avi_m2m_mem(FC7100_AVI_M2M_SIZE);

	p7_reserve_avicammem(&dtv_cam5_dev, CAM5_AVI_RAM_SIZE);
	p7_reserve_avicammem(&dtv_cam1_dev, CAM1_AVI_RAM_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);

#define FC7100_MPGTS_SIZE (CONFIG_ARCH_PARROT7_FC7100_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, FC7100_MPGTS_SIZE);
	p7_reserve_mpegtsmem(1, FC7100_MPGTS_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);

	p7_reserve_dmamem();
}


P7_MACHINE_START(PARROT_DTV, "DTV")
	.reserve        = &dtv_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END
