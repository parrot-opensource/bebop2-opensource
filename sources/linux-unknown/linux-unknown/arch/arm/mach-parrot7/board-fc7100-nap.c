/**
 * linux/arch/arm/mach-parrot7/board-nap.c - Parrot7 NAP board
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Feb-2014
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <linux/input/mpu6050.h>
#include <linux/input/akm8975.h>

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
#include <gpio/p7-gpio.h>

#include "gpu.h"
#include "common.h"
#include "system.h"
#include "board-common.h"
#include "fc7100-module.h"
#include "gpio.h"
#include "mpegts.h"
#include "lcd-monspecs.h"
#include "avi.h"

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>
#include "vdec.h"
#include "aai.h"
#include "nand.h"
#include "usb.h"

extern dma_addr_t   p7_dma_addr __initdata;
extern size_t       p7_dma_size __initdata;

/*********************
 * Framebuffer config
 *********************/

static unsigned long nap_fb1_drivecfg[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(2)     | /* Slew rate 2 */
       P7CTL_DRV_CFG(2),      /* Drive strength 2 (reg=7) */
};

static unsigned long nap_fb1_clk_drivecfg[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(2)     | /* Slew rate 2 */
       P7CTL_DRV_CFG(2),      /* Drive strength 2 (reg=7) */
};

static struct pinctrl_map nap_fb1_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, nap_fb1_clk_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, nap_fb1_drivecfg),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, nap_fb1_drivecfg),
};


static struct avifb_overlay nap_avi_lcd1_overlays[] = {};

static struct avifb_platform_data nap_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &hdmi_1280x720p60_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		/* HSYNC/VSYNC must be generated at same edge as DATA with
		 * SIL9024A HDMI transmitter. By default it reads signals at
		 * rising edge. */
		.psync_en      = 1,
		.psync_rf      = 0,
		.free_run      = 1,
		.itu656        = 0,
		.ipc           = 1,
	}},
	.caps		       = AVI_CAP_LCD_1,
	.background            = 0x000000,
	.overlays	       = nap_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(nap_avi_lcd1_overlays),
};

static u64 nap_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device nap_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &nap_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/************************************
 * Video Output overlay (VOC) config
 ************************************/

static struct avi_voc_plat_data nap_avi_voc_param0 = {
	.display = "lcd.1"
};

static struct avi_voc_plat_data nap_avi_voc_param1 = {
	        .display = "lcd.1"
};

static struct avi_voc_plat_data nap_avi_voc_param2 = {
	                .display = "lcd.1"
};

/********************
 * HDMI Output config
 ********************/

#include "../../../drivers/video/mxc/siihdmi.h"

static void nap_on_hotplug(int screen_attached){
	gpio_set_value(136, !screen_attached);
}

static void nap_reset_hdmi_out(void)
{
	printk("Resetting HDMI chip\n");
	gpio_set_value(154, 0);
	msleep(1);/*TRST Reset pulse width 50us*/
	gpio_set_value(154, 1);
	msleep(2);/*TACC Time from reset until host can access chip registers max 1ms*/
}

static struct siihdmi_platform_data fc7100_siihdmi_data = {
// XXX
	.reset       = nap_reset_hdmi_out,

	.vendor      = "Genesi",
	.description = "Efika MX",

	.lcd_id = "lcd.1",

	.hotplug     = {
		/*.start = IOMUX_TO_IRQ(MX51_PIN_DISPB2_SER_DIO),
		  .end   = IOMUX_TO_IRQ(MX51_PIN_DISPB2_SER_DIO),*/
		.name  = "video-hotplug",
		.flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
	},
	.on_hotplug = &nap_on_hotplug,

// XXX
//	.pixclock    = KHZ2PICOS(133000L),
};

static struct i2c_board_info __initdata fc7100_siihdmi_board_info[] = {
	{
		I2C_BOARD_INFO("siihdmi", 0x3b),
		.platform_data = &fc7100_siihdmi_data,
		.irq = P7_GPIO_NR(140),
	},
};

/*************/
/* MPU6050   */
/*************/
static struct mpu6050_platform_data mpu6050_platform_data = {
        .fifo_mode = 0,
        .flags = 0,
        .x_axis = 2,
        .y_axis = 2,
        .z_axis = 2,
        .accel_fsr = MPU6050_RANGE_4G,
        .gyro_fsr = MPU6050_GYRO_FSR_2000,
};

static struct i2c_board_info __initdata fc7100_mpu6050_board_info[] = {
	{
		I2C_BOARD_INFO("mpu6050", 0x68),
		.irq = P7_GPIO_NR(146),
		.platform_data = &mpu6050_platform_data,
	},
};

/*************/
/* AKM8975   */
/*************/
int akm8975_setup(void) { return 0; }
void akm8975_shutdown(void) {}
void akm8975_hw_config(int enable) {}
void akm8975_power_mode(int enable) {}

static struct akm8975_platform_data akm8975_data = {
    .setup = akm8975_setup,
    .shutdown = akm8975_shutdown,
    .hw_config = akm8975_hw_config,
    .power_mode = akm8975_power_mode
};

/*static struct akm8975_platform_data akm8975_data = {
    .intr = -1,
    };*/

static struct i2c_board_info __initdata fc7100_akm8975_board_info[] = {
	{
		I2C_BOARD_INFO("akm8975", 0x0c),
		.irq = P7_GPIO_NR(142),
		.platform_data = &akm8975_data,
	},
};

/*************
 * Audio
 *************/
static struct aai_pad_t nap_aai_pads[] = {
	/* HDMI clocks */
	{AAI_SIG_MCLK,		 12, PAD_OUT}, /* HDMI I2S MCLK  */
	{AAI_SIG_MAIN_I2S_FRAME, 11, PAD_OUT}, /* HDMI I2S LEFT RIGHT / FSYNC */
	{AAI_SIG_DAC_BIT_CLOCK,	 10, PAD_OUT}, /* HDMI I2S BIT CLK */


	/* I2S Output */
	{AAI_SIG_OUT_DAC0,	 20, PAD_OUT}, /* HDMI TRANSMITTER I2S "0" */
	{AAI_SIG_OUT_DAC1,	 19, PAD_OUT}, /* HDMI TRANSMITTER I2S "1" */
	{AAI_SIG_OUT_DAC2,	 16, PAD_OUT}, /* HDMI TRANSMITTER I2S "2" */
	{AAI_SIG_OUT_DAC3,	 22, PAD_OUT}, /* HDMI TRANSMITTER I2S "3" */
	{AAI_SIG_SPDIF_TX,	 21, PAD_OUT}, /* HDMI TRANSMITTER SPDIF */

	{-1,			 -1,	   0}
};

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",
	"music-out-stereo1",
	"music-out-stereo2",
	"music-out-stereo3",
	"spdif-out",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set nap_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_SLAVE(0)},
	{AAI_SLAVE(1)},
	{AAI_SLAVE(2)},
	{AAI_SLAVE(3)},

	/* Don't remove */
	{-1, 0, 0, 0},
};

static struct aai_platform_data nap_aai_pdata = {
	.pad         = nap_aai_pads,
	.aai_conf    = nap_aai_conf_set,
	.device_list = aai_dev_list,
};

static unsigned long nap_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long nap_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static struct pinctrl_map nap_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, nap_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, nap_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, nap_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_16),
	P7_INIT_PINCFG(P7_AAI_16, nap_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, nap_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, nap_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_21),
	P7_INIT_PINCFG(P7_AAI_21, nap_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, nap_aai_pinconf),
};

static int __init nap_get_rev(void)
{
	int board_rev = 0, i, err, val;
	static const int gpios[] =
		{183, 184, 185, 186, 187, 189, 191, 203, 204, 205, 206};

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {

		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
				P7CTL_PUD_CFG(DOWN), "nap mb rev");

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
	unsigned int mod_settings = 0;
	int board_rev;

	fc7100_init_module(mod_settings);

	board_rev = nap_get_rev();
	pr_info("NAP hw 0x%08x", board_rev);
	/* USB0 is device only */
	p7brd_init_udc(0, -1);

	/* USB1 is host only */
	p7brd_init_hcd(1, -1);

	p7brd_init_i2cm(1, 200);
	p7brd_init_i2cm(2, 100); /* /!\ HDMI DDC which uses I2C protocol,
				  *     requires a clock lower or equal to
				  *     to 100kHz! */

	parrot_init_i2c_slave(1,
						  fc7100_mpu6050_board_info,
						  "MPU6050 board",
						  P7_I2C_IRQ);
	parrot_init_i2c_slave(2,
						  fc7100_siihdmi_board_info,
						  "sii HDMI 9024A tx",
						  P7_I2C_IRQ);
	parrot_init_i2c_slave(2,
						  fc7100_akm8975_board_info,
						  "AKM8975 board",
						  P7_I2C_IRQ);

	p7_init_avi();
	p7_init_avifb(&nap_avifb1_dev, &nap_avifb1_pdata,
	
	nap_fb1_pins, ARRAY_SIZE(nap_fb1_pins));

	p7_init_avi_voc(0, &nap_avi_voc_param0);
	p7_init_avi_voc(1, &nap_avi_voc_param1);
	p7_init_avi_voc(2, &nap_avi_voc_param2);

	p7_init_gpu_fb(p7_dma_addr, p7_dma_size, 4);

	p7_init_vdec();

	/* Audio init */
	p7_init_aai(nap_aai_pins, ARRAY_SIZE(nap_aai_pins), &nap_aai_pdata);

	/* misc gpio export */
	p7brd_export_gpio(P7_GPIO_NR(55), GPIOF_OUT_INIT_HIGH, "LED_CLK");
	p7brd_export_gpio(P7_GPIO_NR(56), GPIOF_OUT_INIT_HIGH, "LED_LATCH");
	p7brd_export_gpio(P7_GPIO_NR(57), GPIOF_OUT_INIT_HIGH, "LED_BLANK");
	p7brd_export_gpio(P7_GPIO_NR(58), GPIOF_OUT_INIT_HIGH, "LED_SIN");
	p7brd_export_gpio(P7_GPIO_NR(150), GPIOF_OUT_INIT_LOW, "ATMEG_UPDATE");

	p7_gpio_interrupt_register(P7_GPIO_NR(140));
	p7brd_export_gpio(P7_GPIO_NR(140), GPIOF_DIR_IN, "hdmi-it");
	p7_config_pin(P7_GPIO_NR(140), P7CTL_PUD_CFG(HIGHZ));
	p7brd_export_gpio(P7_GPIO_NR(154), GPIOF_OUT_INIT_LOW, "hdmi-nrst");

	p7_gpio_interrupt_register(P7_GPIO_NR(142));
	p7brd_export_gpio(P7_GPIO_NR(142), GPIOF_DIR_IN, "magneto-it");

	p7_gpio_interrupt_register(P7_GPIO_NR(146));
	p7brd_export_gpio(P7_GPIO_NR(146), GPIOF_DIR_IN, "gyro-it");

	p7brd_export_gpio(P7_GPIO_NR(148), GPIOF_OUT_INIT_LOW, "wifi-rst");
	p7brd_export_gpio(P7_GPIO_NR(138), GPIOF_OUT_INIT_LOW, "wifi-wake-up");

	p7brd_export_gpio(P7_GPIO_NR(151), GPIOF_OUT_INIT_HIGH, "namp-pwdn");

	p7brd_export_gpio(P7_GPIO_NR(134), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_A");
	p7brd_export_gpio(P7_GPIO_NR(135), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_B");
	p7brd_export_gpio(P7_GPIO_NR(136), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_C");
	p7brd_export_gpio(P7_GPIO_NR(137), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_D");

	p7brd_export_gpio(P7_GPIO_NR(179), GPIOF_OUT_INIT_LOW, "usb-rst");

}

static void __init nap_reserve_mem(void)
{
#define NAP_VOC_SIZE (1920 * 1080 * 4 * 3) //3 1080p BGRA buffers max
	p7_reserve_avi_voc_mem(0, NAP_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, NAP_VOC_SIZE);
	p7_reserve_avi_voc_mem(2, NAP_VOC_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);
	p7_reserve_nand_mem();
	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);
	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_NAP, "NAP")
	.reserve        = &nap_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END
