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
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "aai.h"
#include "avi.h"
#include "board-common.h"
#include "common.h"
#include "fc7100-module.h"
#include "gpu.h"
#include "lcd-monspecs.h"
#include "nand.h"
#include "usb.h"
#include "vdec.h"

/* platform data */
#include "../../../drivers/video/mxc/siihdmi.h"
#include <linux/platform_data/ak8975.h>
#include <linux/platform_data/invensense_mpu6050.h>

/**********************
 * Framebuffer config *
 **********************/

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

static struct avifb_overlay nap_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.width   = 1920,
			.height  = 1080,
			.x       = 0,
			.alpha	 = AVI_ALPHA(50),
			.enabled = 1,
		},
		.dma_memory.end	 = (1920 * 1200 * 4 * 2),
		.zorder		 = 0,
	},
};

static struct avifb_platform_data nap_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &hdmi_1920x1080p60_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {
		{
		/* HSYNC/VSYNC must be generated at same edge as DATA with
		 * SIL9024A HDMI transmitter. By default it reads signals at
		 * rising edge. */
		.psync_en      = 1,
		.psync_rf      = 0,
		.free_run      = 1,
		.itu656        = 0,
		.ipc           = 1,
		}
	},
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


/*************************************
 * Video Output overlay (VOC) config *
 *************************************/

static struct avi_voc_plat_data nap_avi_voc_param0 = {
	.display = "lcd.1"
};

static struct avi_voc_plat_data nap_avi_voc_param1 = {
	.display = "lcd.1"
};

static struct avi_voc_plat_data nap_avi_voc_param2 = {
	.display = "lcd.1"
};


/**********************
 * HDMI Output config *
 **********************/

static void nap_on_hdmi_hotplug(int screen_attached)
{
	gpio_set_value(136, !screen_attached);
}

static void nap_reset_hdmi_out(void)
{
	printk(KERN_INFO "Resetting HDMI chip\n");

	gpio_set_value(154, 0);

	/* TRST Reset pulse width 50us */
	msleep(1);

	gpio_set_value(154, 1);

	/* TACC Time from reset until host can access chip registers max 1ms */
	msleep(2);
}


/*****************
 * SII : HDMI *
 *****************/

#define NAP_HDMI_NAME        "siihdmi"
#define NAP_HDMI_DESCRIPTION "sii HDMI 9024A tx"
#define NAP_HDMI_I2C_BUS     2
#define NAP_HDMI_I2C_ADDR    0x3b
#define NAP_HDMI_IRQ_GPIO    140

static struct siihdmi_platform_data nap_hdmi_pdata = {
	.reset       = nap_reset_hdmi_out,

	.vendor      = "Genesi",
	.description = "Efika MX",

	.lcd_id = "lcd.1",

	.hotplug     = {
		.name  = "video-hotplug",
		.flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
	},

	.on_hotplug = &nap_on_hdmi_hotplug,
};

static struct i2c_board_info __initdata nap_hdmi_board_info = {
	I2C_BOARD_INFO(NAP_HDMI_NAME, NAP_HDMI_I2C_ADDR),
	.platform_data = &nap_hdmi_pdata,
	.irq = P7_GPIO_NR(NAP_HDMI_IRQ_GPIO),
};


/*****************
 * MPU6050 : IMU *
 *****************/

#define NAP_IMU_NAME        "mpu6050"
#define NAP_IMU_DESCRIPTION "IMU mpu6050"
#define NAP_IMU_I2C_BUS     1
#define NAP_IMU_I2C_ADDR    0x68
#define NAP_IMU_IRQ_GPIO    146
#define NAP_IMU_ORIENTATION { 0, 1, 0, 1, 0, 0, 0, 0, -1 }

static struct inv_mpu6050_platform_data nap_imu_pdata = {
	.orientation = NAP_IMU_ORIENTATION,
};

static struct i2c_board_info __initdata nap_imu_board_info = {
	I2C_BOARD_INFO(NAP_IMU_NAME, NAP_IMU_I2C_ADDR),
	.irq = P7_GPIO_NR(NAP_IMU_IRQ_GPIO),
	.platform_data = &nap_imu_pdata,
};


/*************************
 * AK8963 : Magnetometer *
 *************************/

#define NAP_MAGNETO_NAME        "ak8963"
#define NAP_MAGNETO_DESCRIPTION "Magneto ak8963"
#define NAP_MAGNETO_I2C_BUS     2
#define NAP_MAGNETO_I2C_ADDR    0x0c
#define NAP_MAGNETO_IRQ_GPIO    142
#define NAP_MAGNETO_ORIENTATION "0, -1, 0; -1, 0, 0; 0, 0, -1;"

static struct ak8975_platform_data nap_magneto_pdata = {
	.orientation = NAP_MAGNETO_ORIENTATION,
};

static struct i2c_board_info __initdata nap_magneto_board_info = {
	I2C_BOARD_INFO(NAP_MAGNETO_NAME, NAP_MAGNETO_I2C_ADDR),
	.platform_data = &nap_magneto_pdata,
	.irq = P7_GPIO_NR(NAP_MAGNETO_IRQ_GPIO),
};


/*********
 * Audio *
 *********/
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
	/* This configuration is used to set music in channel MASTER or SLAVE */
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

static int __init nap_get_board_revision(void)
{
	int i;
	int val;
	int err;
	int board_revision = 0;
	static const int gpios[] = { 183, 184, 185, 186, 187, 189,
				     191, 203, 204, 205, 206 };

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {

		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
				P7CTL_PUD_CFG(DOWN), "nap mb rev");

		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			board_revision |= (val << i);
			gpio_free(P7_GPIO_NR(gpios[i]));
		}
	}

	return board_revision;
}

static void __init nap_initialize_machine(void)
{
	int board_revision;
	unsigned long fb_start = 0;
	unsigned long fb_size = 0;
	unsigned int mod_settings = 0;

	/* initialize fc7100 module */
	fc7100_init_module(mod_settings);

	/* print board revision */
	board_revision = nap_get_board_revision();
	pr_info("NAP hw 0x%08x", board_revision);

	/* USB0 is device only */
	p7brd_init_udc(0, -1);

	/* USB1 is host only */
	p7brd_init_hcd(1, -1);

	/* initialize i2c bus 1 */
	p7brd_init_i2cm(1, 200);

	/* initialize i2c bus 2
	 * HDMI DDC using I2C requires a clock <= 100KHz */
	p7brd_init_i2cm(2, 100);

	/* initialize IMU */
	parrot_init_i2c_slave(NAP_IMU_I2C_BUS,
			      &nap_imu_board_info,
			      NAP_IMU_DESCRIPTION,
			      P7_I2C_IRQ);

	/* initialize HDMI */
	parrot_init_i2c_slave(NAP_HDMI_I2C_BUS,
			      &nap_hdmi_board_info,
			      NAP_HDMI_DESCRIPTION,
			      P7_I2C_IRQ);

	p7_config_pin(P7_GPIO_NR(NAP_HDMI_IRQ_GPIO), P7CTL_PUD_CFG(HIGHZ));

	/* initialize Magneto */
	parrot_init_i2c_slave(NAP_MAGNETO_I2C_BUS,
			      &nap_magneto_board_info,
			      NAP_MAGNETO_DESCRIPTION,
			      P7_I2C_IRQ);

	/* initialize framebuffer */
	p7_init_avi();
	p7_init_avifb(&nap_avifb1_dev, &nap_avifb1_pdata,
		      nap_fb1_pins, ARRAY_SIZE(nap_fb1_pins));

	p7_init_avi_voc(0, &nap_avi_voc_param0);
	p7_init_avi_voc(1, &nap_avi_voc_param1);
	p7_init_avi_voc(2, &nap_avi_voc_param2);

	fb_start = nap_avi_lcd1_overlays[0].dma_memory.start;
	fb_size = nap_avi_lcd1_overlays[0].dma_memory.end - fb_start + 1;
	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_vdec();

	/* initialize audio */
	p7_init_aai(nap_aai_pins, ARRAY_SIZE(nap_aai_pins), &nap_aai_pdata);

	/* export GPIO */
	p7brd_export_gpio(P7_GPIO_NR(55), GPIOF_OUT_INIT_HIGH, "LED_CLK");
	p7brd_export_gpio(P7_GPIO_NR(56), GPIOF_OUT_INIT_HIGH, "LED_LATCH");
	p7brd_export_gpio(P7_GPIO_NR(57), GPIOF_OUT_INIT_HIGH, "LED_BLANK");
	p7brd_export_gpio(P7_GPIO_NR(58), GPIOF_OUT_INIT_HIGH, "LED_SIN");
	p7brd_export_gpio(P7_GPIO_NR(150), GPIOF_OUT_INIT_HIGH, "ATMEG_UPDATE");

	p7brd_export_gpio(P7_GPIO_NR(154), GPIOF_OUT_INIT_LOW, "hdmi-nrst");

	p7brd_export_gpio(P7_GPIO_NR(148), GPIOF_OUT_INIT_LOW, "wifi-rst");
	p7brd_export_gpio(P7_GPIO_NR(138), GPIOF_OUT_INIT_LOW, "wifi-wake-up");

	p7brd_export_gpio(P7_GPIO_NR(151), GPIOF_OUT_INIT_HIGH, "namp-pwdn");

	p7brd_export_gpio(P7_GPIO_NR(134), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_A");
	p7brd_export_gpio(P7_GPIO_NR(135), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_B");
	p7brd_export_gpio(P7_GPIO_NR(136), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_C");
	p7brd_export_gpio(P7_GPIO_NR(137), GPIOF_OUT_INIT_LOW, "ATMEG_GPIO_D");

	p7brd_export_gpio(P7_GPIO_NR(179), GPIOF_OUT_INIT_LOW, "usb-rst");
}

static void __init nap_reserve_memory(void)
{
	p7_reserve_avifbmem(&nap_avifb1_dev, nap_avi_lcd1_overlays,
			    ARRAY_SIZE(nap_avi_lcd1_overlays));

/* 3 1080p BGRA buffers max */
#define NAP_VOC_SIZE (1920 * 1080 * 4 * 3)
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
	.reserve      = &nap_reserve_memory,
	.init_machine = &nap_initialize_machine,
P7_MACHINE_END
