/*
 * linux/arch/arm/mach-parrot7/board-fc7100-testbench.c -FC7100 testbench board
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Avr-2014
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
#include <linux/i2c/pca953x.h>
#include <linux/mmc/host.h>

#include "common.h"
#include "system.h"
#include "i2cm.h"
#include "gpio.h"
#include "sdhci.h"
#include "aai.h"
#include "avi.h"
#include "gpu.h"
#include "mpegts.h"
#include "nand.h"
#include "usb.h"

#include "board-common.h"
#include "fc7100-module.h"
#include "lcd-monspecs.h"

#define FC7100_BRD_NAME "FC7100 TESTBENCH"

static __maybe_unused struct acs3_plat_data testbench_sdhci1_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
	.nb_config   = 1,
};

/********
  lcd
 *******/

#ifdef CONFIG_MACH_PARROT_FC7100_TB_ANDROID
static unsigned long fake_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map fake_avifb0_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, fake_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, fake_lcd_pinconf),
};

/* Should we bother setting a narrower mask here? */
static u64 fake_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device fake_avifb0_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &fake_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avifb_overlay fake_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
	},
};
static struct avifb_platform_data fake_avifb0_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
		}},
        .lcd_format_control	  = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	  = &hdmi_1280x720p60_video_mode,
	.lcd_videomodes		  = p7_all_video_modes,
	.lcd_interface.ipc        = 1,
	/* default overlays */
        .overlays		  = fake_avi_lcd0_overlays,
	.overlay_nr		  = ARRAY_SIZE(fake_avi_lcd0_overlays),
	.caps                     = AVI_CAP_LCD_1 | AVI_CAP_GAM,
};
#else
static unsigned long testbench_fb_drivecfg[] = {
       P7CTL_SMT_CFG(OFF)   | /* no schmitt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(2)     | /* Slew rate 2 */
       P7CTL_DRV_CFG(2),      /* Drive strength 2 (reg=7) */
};

static struct pinctrl_map testbench_fb0_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_0_CLK),
	P7_INIT_PINCFG(P7_LCD_0_CLK, testbench_fb_drivecfg),
};

static struct pinctrl_map testbench_fb1_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, testbench_fb_drivecfg),
};

static struct avifb_platform_data testbench_avifb0_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	/// 27Mhz PCLK
	.lcd_default_videomode = &hdmi_720x576p50_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
		.ipc           = 1,
	}},
	.caps		       = AVI_CAP_LCD_0,
	.background            = 0x00ff00,
	.overlays	       = NULL,
	.overlay_nr	       = 0,
};

static struct avifb_platform_data testbench_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	/// 27Mhz PCLK
	.lcd_default_videomode = &hdmi_720x576p50_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
		.ipc           = 1,
	}},
	.caps		       = AVI_CAP_LCD_1,
	.background            = 0x00ff00,
	.overlays	       = NULL,
	.overlay_nr	       = 0,
};

static u64 testbench_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device testbench_avifb0_dev = {
	.name           = "avifb",
	.id             = 0,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &testbench_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static u64 testbench_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device testbench_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &testbench_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

#endif


/*************
 * Audio
 *************/

#ifndef CONFIG_MACH_PARROT_FC7100_TB_ANDROID
static struct aai_pad_t test_bench_aai_pads[] =
{
	/* PCM1 (Bluetooth for FC7100 HW04, HW05, HW06) */
	{AAI_SIG_PCM1_OUT,          0,  PAD_OUT},
	{AAI_SIG_PCM1_IN,           2,  PAD_IN},
	{AAI_SIG_PCM1_FRAME,        3,  PAD_IN},

	{-1,                       -1,  0}
};


static char * aai_dev_list[] =
{
	/*Output channels*/

   "pcm0-out",

	/*Input Channels*/

   "pcm0-in",
	//"loopback-8k",
	//"loopback-16k",

	/*Don't remove*/
	NULL,
};

static struct aai_conf_set test_bench_aai_conf_set[] =
{
	/* This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},
	{AAI_MASTER(1)},
	{AAI_MASTER(2)},

	/*Don't remove*/
	{-1, 0, 0, 0},
};

static struct aai_platform_data test_bench_aai_pdata = {
	.pad = test_bench_aai_pads,
	.aai_conf = test_bench_aai_conf_set,
	.device_list = aai_dev_list,
};


static unsigned long testbench_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static unsigned long testbench_aai_pcm_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static struct pinctrl_map testbench_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, testbench_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, testbench_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, testbench_aai_pcm_frame_pinconf),
};

#endif

static void __init init_board(void)
{
	unsigned int mod_settings = FC7100_MOD_SDCARD1;

	fc7100_init_module(mod_settings);

	p7brd_init_uart(0,0);

#ifndef CONFIG_MACH_PARROT_FC7100_TB_ANDROID
	p7_init_aai(testbench_aai_pins, ARRAY_SIZE(testbench_aai_pins),
		    &test_bench_aai_pdata);
#endif

	p7brd_init_sdhci(1, &testbench_sdhci1_pdata, NULL, NULL, NULL, NULL, 0);

	p7_init_avi();
#ifdef CONFIG_MACH_PARROT_FC7100_TB_ANDROID
	p7_init_avifb(&fake_avifb0_dev, &fake_avifb0_pdata,
		      fake_avifb0_pins, ARRAY_SIZE(fake_avifb0_pins));
	p7_init_gpu_fb(fake_avi_lcd0_overlays[0].dma_memory.start,
		       fake_avi_lcd0_overlays[0].dma_memory.end -
		       fake_avi_lcd0_overlays[0].dma_memory.start + 1,
			   4);
#else
  p7_init_avifb(&testbench_avifb0_dev, &testbench_avifb0_pdata,
                testbench_fb0_pins, ARRAY_SIZE(testbench_fb0_pins));
  p7_init_avifb(&testbench_avifb1_dev, &testbench_avifb1_pdata,
                testbench_fb1_pins, ARRAY_SIZE(testbench_fb1_pins));
#endif

	/* USB0 is device only */
#ifdef CONFIG_MACH_PARROT_FC7100_TB_ANDROID
	p7brd_init_udc(0, -1);
#else
	p7brd_init_usb(0, -1, CI_UDC_DR_DUAL_DEVICE);
#endif
	p7brd_init_hcd(1, -1);
}

static void __init fc7100_tb_reserve_mem(void)
{
	p7_reserve_nand_mem();
	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);
	p7_reserve_dmamem();

#ifdef CONFIG_MACH_PARROT_FC7100_TB_ANDROID
	p7_reserve_avifbmem(&fake_avifb0_dev, fake_avi_lcd0_overlays,
			    ARRAY_SIZE(fake_avi_lcd0_overlays));
#endif
}

P7_MACHINE_START(PARROT_FC7100_TB, "FC7100_TB")
	.reserve        = &fc7100_tb_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END

