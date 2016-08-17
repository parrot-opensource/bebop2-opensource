/**
 * linux/arch/arm/mach-parrot7/board-p7dev-hdmi.c - P7Dev HDMI daughter board
 *                                                  implementation
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    27-May-2013
 *
 * This file is released under the GPL
 */

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <mach/pwm.h>
#include "board-common.h"
#include "common.h"
#include "board.h"
#include "pinctrl.h"
#include "lcd-monspecs.h"
#include "avi.h"
#include "backlight.h"

/***********************
 * AVI configuration
 ***********************/
#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)

/*********************
 *    OUT0 Config
 *********************/

static unsigned long hdmi_out0_clkcfg[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
};

static struct pinctrl_map hdmi_avifb0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_LCD_0_CLK),

	P7_INIT_PINCFG(P7_LCD_0_CLK, hdmi_out0_clkcfg),

	P7_INIT_PINMAP(P7_LCD_0_DEN),
	P7_INIT_PINMAP(P7_LCD_0_HS),
	P7_INIT_PINMAP(P7_LCD_0_VS),
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
};

static struct avifb_overlay hdmi_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.enabled = 1,
		},
		.dma_memory.end	 = (1920 * 1080 * 4 * 2),
		.zorder		 = 0,
	},
};

static struct avifb_platform_data hdmi_avifb0_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_UYVY_2X8,
	.lcd_default_videomode = &hdmi_1280x720p60_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_0 | AVI_CAP_CONV,
	.overlays	       = hdmi_avi_lcd0_overlays,
	.overlay_nr	       = ARRAY_SIZE(hdmi_avi_lcd0_overlays),
};

/* Should we bother setting a narrower mask here? */
static u64 hdmi_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device hdmi_avifb0_dev = {
	.name           = "avifb",
	.id             = 0,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &hdmi_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/*********************
 *    OUT1 Config
 *********************/

static unsigned long hdmi_out1_clkcfg[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
};

static struct pinctrl_map hdmi_avifb1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_LCD_1_CLK),

	P7_INIT_PINCFG(P7_LCD_1_CLK, hdmi_out1_clkcfg),

	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
};

#define P7_AVIFB1_BUFFER_SIZE PAGE_ALIGN(1280*720*4*2)

static struct avifb_overlay hdmi_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.enabled = 1,
		},
		.dma_memory.end	 = (1920 * 1080 * 4 * 2),
		.zorder		 = 0,
	},
};

static struct avifb_platform_data hdmi_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_UYVY_2X8,
	.lcd_default_videomode = &hdmi_1280x720p60_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_1 | AVI_CAP_CONV,
	.overlays	       = hdmi_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(hdmi_avi_lcd1_overlays),
};

/* Should we bother setting a narrower mask here? */
static u64 hdmi_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device hdmi_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &hdmi_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

#endif /* defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */

#if defined(CONFIG_AVI) || defined(CONFIG_AVI_MODULE)

#endif /* defined(CONFIG_AVI) || defined(CONFIG_AVI_MODULE) */

void __init hdmidb_rsvmem(struct p7_board const* board)
{
#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
	p7_reserve_devmem(&hdmi_avifb0_dev,
	                  &hdmi_avi_lcd0_overlays[0].dma_memory.start,
	                  &hdmi_avi_lcd0_overlays[0].dma_memory.end);
	hdmi_avi_lcd0_overlays[0].dma_memory.end += hdmi_avi_lcd0_overlays[0].dma_memory.start - 1;
	p7_reserve_devmem(&hdmi_avifb1_dev,
	                  &hdmi_avi_lcd1_overlays[0].dma_memory.start,
	                  &hdmi_avi_lcd1_overlays[0].dma_memory.end);
	hdmi_avi_lcd1_overlays[0].dma_memory.end += hdmi_avi_lcd1_overlays[0].dma_memory.start - 1;
#endif /* CONFIG_ARCH_PARROT7_P7DEV_HDMI_HDMI_ENABLE */
}

void __init hdmidb_probe(struct p7_board const* board)
{
#if defined(CONFIG_AVI) || defined(CONFIG_AVI_MODULE)
	p7_init_avi();
#endif

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
	p7_init_avifb(&hdmi_avifb0_dev, &hdmi_avifb0_pdata,
		      hdmi_avifb0_pins, ARRAY_SIZE(hdmi_avifb0_pins));
	p7_init_avifb(&hdmi_avifb1_dev, &hdmi_avifb1_pdata,
		      hdmi_avifb1_pins, ARRAY_SIZE(hdmi_avifb1_pins));
#endif

	if (gpio_request_one(P7MU_IO_NR(5), GPIOF_OUT_INIT_HIGH, "HDMI inputs nRST"))
		pr_err("HDMI: failed to de-reset inputs\n");
	else
		pr_info("HDMI: inputs de-reset\n");

	p7brd_init_i2cm(2, 100);
}
