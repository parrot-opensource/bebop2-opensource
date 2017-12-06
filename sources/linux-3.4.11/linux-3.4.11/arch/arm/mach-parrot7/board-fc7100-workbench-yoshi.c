/*
 * CES demo yoshi board (FC7100 workbench + portrait screen) specific code
 *
 * Author: Ivan Djelic <ivan.djelic@parrot.com>
 * Copyright (C) 2013 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
#include <mach/p7.h>
#include "common.h"
#include "system.h"
#include <gpio/p7-gpio.h>
#include <video/avifb.h>

#include "board-common.h"
#include "fc7100-mezzs.h"
#include "gpio.h"
#include "avi.h"
#include "lcd-monspecs.h"
#include "backlight.h"
#include "gpu.h"

#include "fc7100-mezzs2-avi.h"

static unsigned long yoshi_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* pull up/down disable */
       P7CTL_DRV_CFG(3),      /* Drive strength */
};

static struct pinctrl_map yoshi_avifb1_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, yoshi_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, yoshi_lcd_pinconf),
};

/* position and sizes when the fb is at its max size after parrotVTK plays with
 * values, this forces xres_virt and line_length to be really constant after. */
static struct avifb_overlay yoshi_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.x       = 72,
			/* should be 1138 but issue in android if not multiple
			 * of 16, parrotVTK will  */
			.width   = 1152,
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
		.zorder          = 0,
		.color_depth     = 32,
	},
	{
		.layout          = {
			.x       = 792,
			.width   = 488,
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end  = 1280 * 800 * 4 * 2,
		.zorder          = 1,
		.color_depth     = 32,
	},
	{
		.layout          = {
			.x       = 0,
			.width   = 1280,
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.dma_memory.end  = 1280 * 800 * 4 * 2,
		.zorder          = 2,
		.color_depth     = 32,
	},
};

static struct avifb_platform_data yoshi_avifb1_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
		}},
	.lcd_format_control	  = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	  = &kyocera_video_mode,
	.lcd_videomodes		  = p7_all_video_modes,
	.caps                     = AVI_CAP_LCD_1,
	.output_cspace		  = AVI_RGB_CSPACE,
	/* default overlays */
        .overlays		  = yoshi_avi_lcd1_overlays,
	.overlay_nr		  = ARRAY_SIZE(yoshi_avi_lcd1_overlays),
};

static u64 yoshi_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device yoshi_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		.dma_mask           = &yoshi_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avi_voc_plat_data yoshi_avi_voc_param = {
	.display = "lcd.1",
};


void __init yoshi_lcd_mem(void)
{
	fc7100_mezz_reserve_mem_for_lcd(yoshi_avi_lcd1_overlays,
	                                ARRAY_SIZE(yoshi_avi_lcd1_overlays));
}

void __init yoshi_init_lcd(void)
{
#if defined(CONFIG_GPU_PARROT7) || \
    defined(CONFIG_GPU_PARROT7_MODULE)
	/* Fb mem is contiguous and goes from fb2 to fb0 */
	unsigned int fb_nr = ARRAY_SIZE(yoshi_avi_lcd1_overlays);
	unsigned long fb_start = yoshi_avi_lcd1_overlays[fb_nr - 1].dma_memory.start;
	unsigned long fb_size = yoshi_avi_lcd1_overlays[0].dma_memory.end - fb_start + 1;
#endif
	/* downscale kiocera lcd pixel clock for P7R3 on FC7100 HW06 & HW07.
	 * TODO: On p7dev board with P7R3, this patch is not needed so
	 * FC7100 Hardware investigation must be performed ...  by someone !
	 */
	if ((p7_chiprev() == P7_CHIPREV_R3) &&
	    (fc7100_module_get_rev() == 6 ||
	     fc7100_module_get_rev() == 7))
		kyocera_video_mode.pixclock = 59400;

	p7_init_avifb(&yoshi_avifb1_dev, &yoshi_avifb1_pdata,
	              yoshi_avifb1_pins, ARRAY_SIZE(yoshi_avifb1_pins));

	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &yoshi_avi_voc_param);
}
