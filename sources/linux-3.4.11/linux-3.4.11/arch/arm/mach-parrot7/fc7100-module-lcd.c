#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>

#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <mach/gpio.h>
#include <gpio/p7-gpio.h>
#include <i2c/p7-i2cm.h>
#include <linux/i2c/pca953x.h>
#include <host/sdhci.h>
#include <mmc/acs3-sdhci.h>

#include "common.h"
#include "system.h"
#include "aai.h"
#include "avi.h"
#include "backlight.h"
#include "gbc.h"
#include "gpio.h"
#include "gpu.h"
#include "i2cm.h"
#include "lcd-monspecs.h"
#include "p7_pwm.h"
#include "p7mu.h"
#include "pinctrl.h"
#include "sdhci.h"
#include "spi.h"
#include "uart.h"

#include "system.h"
#include "board-common.h"
#include "fc7100-module.h"

#define FC7100_BRD_NAME "FC7100 mezz avi"

static unsigned long fc7100_lcd_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};
static unsigned long fc7100_lcd_pinconf_hdrive[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(4),      /* Drive strength 4(reg=31) */
};

/***********************
 * AVI configuration
 ***********************/

/***************************
 * Framebuffer configuration
 ***************************/
static struct pinctrl_map fc7100_r3_avifb0_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, fc7100_lcd_pinconf_hdrive),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, fc7100_lcd_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, fc7100_lcd_pinconf),
};

static struct avifb_platform_data fc7100_avifb0_r12_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
		}},
        .lcd_format_control	  = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	  = &kyocera_video_mode,
	.lcd_videomodes		  = p7_all_video_modes,
	/* default overlays */
        .overlays		  = NULL,
	.overlay_nr		  = 0,
	.caps                     = AVI_CAP_GAM,
};

/* Should we bother setting a narrower mask here? */
static u64 fc7100_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device fc7100_avifb0_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &fc7100_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct avi_voc_plat_data fc7100_avi_voc_param0 = {
	.display = "lcd.1",
};

static struct avi_voc_plat_data fc7100_avi_voc_param1 = {
	.display = "lcd.1",
};

/** VOC connected to RAM2RAM*/
static struct avi_voc_plat_data fc7100_avi_voc_m2m = {
	.display = "m2m.0",
};

static void fixup_kiocera_pixelclock(void)
{
	/* downscale kiocera lcd pixel clock for P7R3 on FC7100 HW06 & HW07.
	 * TODO: On p7dev board with P7R3, this patch is not needed so
	 * FC7100 Hardware investigation must be performed ...  by someone !
	 */
	if ((p7_chiprev() == P7_CHIPREV_R3) &&
	    (fc7100_module_get_rev() == 6 ||
	     fc7100_module_get_rev() == 7))
		kyocera_video_mode.pixclock = 59400;
}

void fc7100_mezz_avi_init_lcd_screen(const char *screen_name, int *xres, int *yres)
{
	if (screen_name)
		pr_info("%s: Registering avi for screen %s\n",
			FC7100_BRD_NAME, screen_name);
	else
		pr_info("%s: Registering avi without specified screen\n",
			FC7100_BRD_NAME);
	//XXX this need to move to lcd-monspecs configs
	if (screen_name && strcmp(screen_name, "porsche_rse") == 0)
		fc7100_avifb0_r12_pdata.lcd_interface.ipc = 1;

	if (screen_name && strcmp(screen_name, "kyocera") == 0)
		fixup_kiocera_pixelclock();

	if (screen_name && strcmp(screen_name, "sii-hdmi") == 0) {
		fc7100_avifb0_r12_pdata.lcd_default_videomode = &hdmi_1024x768p60_video_mode;
		fc7100_avifb0_r12_pdata.lcd_interface.ipc = 0;
		fc7100_avifb0_r12_pdata.width = 304;
		fc7100_avifb0_r12_pdata.height = 228;
	}

	if (screen_name && strcmp(screen_name, "2K31715") == 0) {
		fc7100_avifb0_r12_pdata.lcd_default_videomode = &tft800480_video_mode;
		fc7100_avifb0_r12_pdata.lcd_interface.ihs = 1;
		fc7100_avifb0_r12_pdata.lcd_interface.ivs = 1;
		fc7100_avifb0_r12_pdata.lcd_interface.ipc = 1;
	}

	if (screen_name != NULL) {
		int	i;
		const struct avi_videomode *lcd_videomode = NULL;

		for (i = 0; p7_all_video_modes[i] != NULL; i++) {

			lcd_videomode = p7_all_video_modes[i];

			if (!strcmp(lcd_videomode->name, screen_name)) {
				fc7100_avifb0_r12_pdata.lcd_default_videomode = lcd_videomode;
				break;
			}
		}

		/* requested screen name not found */
		BUG_ON(!lcd_videomode);
	}

	if (xres != NULL)
		*xres = fc7100_avifb0_r12_pdata.lcd_default_videomode->xres;
	if (yres != NULL)
		*yres = fc7100_avifb0_r12_pdata.lcd_default_videomode->yres;
}

void __init fc7100_mezz_avi_init_lcd(struct avifb_overlay *overlay, int overlay_size)
{
	unsigned long fb_start;
	unsigned long fb_size;

	fc7100_avifb0_r12_pdata.overlays = overlay;
	fc7100_avifb0_r12_pdata.overlay_nr = overlay_size;

	fb_start = fc7100_avifb0_r12_pdata.overlays[0].dma_memory.start;
	fb_size = fc7100_avifb0_r12_pdata.overlays[0].dma_memory.end - fb_start + 1;

	fc7100_avifb0_r12_pdata.caps |= AVI_CAP_LCD_1;

	p7_init_avifb(&fc7100_avifb0_dev, &fc7100_avifb0_r12_pdata,
		      fc7100_r3_avifb0_pins, ARRAY_SIZE(fc7100_r3_avifb0_pins));

	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &fc7100_avi_voc_param0);
	p7_init_avi_voc(1, &fc7100_avi_voc_param1);
	p7_init_avi_voc(2, &fc7100_avi_voc_m2m);
}

void __init fc7100_mezz_reserve_mem_for_lcd(struct avifb_overlay *overlays, int noverlays)
{
	p7_reserve_avifbmem(&fc7100_avifb0_dev, overlays, noverlays);

#define FC7100_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(0, FC7100_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, FC7100_VOC_SIZE);
	p7_reserve_avi_voc_mem(2, FC7100_VOC_SIZE);
}
