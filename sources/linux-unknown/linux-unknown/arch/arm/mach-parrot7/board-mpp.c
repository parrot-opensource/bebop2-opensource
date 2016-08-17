/**
 * linux/arch/arm/mach-parrot7/board-mpp.c
 *
 * Copyright (C) 2016 Parrot S.A.
 *
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#include <linux/leds_pwm.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <linux/ramoops.h>
#include <linux/memblock.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <asm/system_info.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <video/avi.h>
#include <spi/p7-spi.h>
#include <spi/p7-spim.h>
#include <mfd/p7mu.h>
#include "board-common.h"
#include "common.h"
#include "pinctrl.h"
#include "venc.h"
#include "spi.h"
#include "usb.h"
#include "avi.h"
#include "p7_temperature.h"
#include "gpu.h"
#include "nand.h"
#include "usb.h"
#include "lcd-monspecs.h"
#include "aai.h"

/* USB HUB support */
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>

/* HDMI input support */
#include "sii_platform_data.h"

#include "board-common.h"
#include "common.h"
#include "p7_pwm.h"
#include "vdec.h"
#include "gpio.h"
#include "gpu.h"

#include "board-drone-common.h"

//XXX ugly from nap
extern dma_addr_t   p7_dma_addr __initdata;
extern size_t       p7_dma_size __initdata;


/*************
 * USB hub
 *************/

#if 0
static struct smsc82514_pdata ev_smsc82512_pdata = {
	.us_port   = DS_HIGH,
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = P7_GPIO_NR(HSIS_HWxx__HUB_RST),
};

static struct i2c_board_info smsc82512_info = {
	I2C_BOARD_INFO("smsc82512", 0x2c),
	.platform_data = &ev_smsc82512_pdata,
};
#endif


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
	/* XXX ATMEG_GPIO_C on nap */
	//gpio_set_value(136, !screen_attached);
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

static struct i2c_board_info __initdata fc7100_siihdmi_board_info = {
		I2C_BOARD_INFO("siihdmi", 0x3b),
		.platform_data = &fc7100_siihdmi_data,
		.irq = P7_GPIO_NR(90),
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
static void __init mpp_init_mach(void)
{
	/* Initialize ramoops */
	drone_common_init_ramoops();

	/* Init hardware revision independent stuff */
	p7_init_mach();

	/* Init GPIOs */
	p7_init_gpio(NULL, 0);

	/* Get PCB/HW revision and update HSIS */
	//ev_board_probe();

	/* Init debug UART */
	p7brd_init_uart(3, 0);

	/* Init NAND */
	p7brd_init_nand(0);

	/* Init I2C master
	 * I2CM-0:
	 *     P7MU
	 *     HDMI Input (with EDID EEPROM)
	 * I2CM-2:
	 *     AKM8963 (magneto)
	 *     MS5607 (Pressure/Temperature)
	 *     HDMI Input (with EDID EEPROM)
	 * I2CM-1:
	 *     MPU6050 (gyro/accel)
	 * XXX connect hub ?
	 *     USB HUB
	 */
	p7brd_init_i2cm(0, 100);
	p7brd_init_i2cm(1, 200);
	/* /!\ HDMI DDC which uses I2C protocol,
	 *     requires a clock lower or equal to
	 *     to 100kHz! */
	p7brd_init_i2cm(2, 100);

	/* Init BT */
	p7brd_init_uart(1, 1);
	p7brd_export_gpio(119, GPIOF_OUT_INIT_LOW, "bt-rst");

	/* Initialize P7MU */
	p7_gpio_interrupt_register(73);
	drone_common_init_p7mu(73, 0);

	/* Init Video out */
	p7_config_pin(fc7100_siihdmi_board_info.irq, P7CTL_PUD_CFG(HIGHZ));
	p7brd_export_gpio(P7_GPIO_NR(154), GPIOF_OUT_INIT_LOW, "hdmi-nrst");

	parrot_init_i2c_slave(2,
						  &fc7100_siihdmi_board_info,
						  "sii HDMI 9024A tx",
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
	p7brd_export_gpio(P7_GPIO_NR(125), GPIOF_OUT_INIT_HIGH, "namp-pwdn");

	/* Init USB */
	if (1) {
		/* HW0 are buggy */
		if (parrot_force_usb_device)
			p7brd_init_usb(0, -1, CI_UDC_DR_DEVICE);
		else {
			void __iomem *sysctl = (void __iomem *)__MMIO_P2V(P7_SYS);
			u32 tmp;
			/* put usb 0 ulpi in HiZ on p7mu */
			p7mu_write16(0xa00, 2);
			p7mu_write16(0xb00, 6);

			/* gpio input on p7 */
			/* 0xa00000 + 0x1000 */
			tmp = readl(sysctl + 0x1000 + 99*4);
			tmp |= 1;
			tmp &= (3<<9);
			writel(tmp, sysctl + 0x1000 + 99*4);

			tmp = readl(sysctl + 0x1000 + 101*4);
			tmp |= 1;
			tmp &= (3<<9);
			writel(tmp, sysctl + 0x1000 + 101*4);

			p7brd_init_usb(1, -1, CI_UDC_DR_HOST);
		}
	}
	else {
		drone_common_init_usb(86 /* VBUS_TABLET_EN */, 87 /* USB_VBUS_TABLET */,
			      88 /* #OC_TABLET */);
		/* Init EHCI 1 */
		p7brd_init_usb(1, -1, CI_UDC_DR_HOST);
	}


	/* Init USB Hub */
	gpio_request_one(179, GPIOF_OUT_INIT_LOW,
			 "RESET_USB_HUB");
#if 0
	parrot_init_i2c_slave(HUB_I2C_BUS, &smsc82512_info, "smsc 82512",
			      P7_I2C_NOIRQ);
#endif

	/* Init sensors */
#if 0
	drone_common_init_ak8963(AK8963_I2C_BUS, ev_hsis.magneto_int_p7);
	drone_common_init_inv_mpu6050(MPU6050_I2C_BUS, ev_hsis.gyro_int_p7,
				      FSYNC_GYRO_FILTER, ev_hsis.clkin_gyro);
	drone_common_init_ms5607(MS5607_I2C_BUS);
#endif

	/* Init FAN */
	drone_common_init_fan(85);

	p7brd_export_gpio(P7_GPIO_NR(124), GPIOF_OUT_INIT_LOW, "POWER_KEEP");
	p7brd_export_gpio(9, GPIOF_OUT_INIT_LOW, "RESET_WIFI");
	/* End of initialization */
	pr_info("Mpp board : init done\n");
}

static void __init mpp_reserve_mem(void)
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

P7_MACHINE_START(PARROT_MPP, "Mpp board")
	.reserve        = &mpp_reserve_mem,
	.init_machine   = &mpp_init_mach,
P7_MACHINE_END
