/*
 * linux/arch/arm/mach-parrot7/board-fc7100-mezz-dev.c - Mezz dev,
 *   sharing EMC workbench.
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Jan-2015
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

#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>

#include "common.h"
#include "system.h"
#include "i2cm.h"
#include "gpio.h"
#include "sdhci.h"
#include "aai.h"
#include "avi.h"
#include "mpegts.h"
#include "nand.h"
#include "usb.h"

#include "board-common.h"
#include "fc7100-module.h"
#include "fc7100-module-lcd.h"
#include "lcd-monspecs.h"
#include "venc.h"
#include "vdec.h"
#include "avi.h"
#include <media/adv7604.h>

#define FC7100_BRD_NAME "FC7100 MEZZ DEV"


/*************
 * Audio (AAI)
 *************/

static struct aai_pad_t mezz_dev_aai_pads[] = {
	/* CODEC OUT CLKs */
	{AAI_SIG_DAC_BIT_CLOCK,	 10, PAD_OUT},
	{AAI_SIG_MCLK,		 12, PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME, 11, PAD_OUT},
	/* CODEC IN CLKs */
	{AAI_SIG_DAC_BIT_CLOCK,	 10, PAD_IN },
	{AAI_SIG_MCLK,		 12, PAD_IN },

	/* codec IN/OUT */
	{AAI_SIG_I2S0_IN,	 21, PAD_IN },
	{AAI_SIG_OUT_DAC0,	 20, PAD_OUT},
	/* DB */
	{AAI_SIG_I2S1_IN,	 16, PAD_IN },
	{AAI_SIG_OUT_DAC1,	 15, PAD_OUT},
	/* CAM0 */
	{AAI_SIG_I2S2_FRAME,	 22, PAD_IN },
	{AAI_SIG_I2S2_IN,	 17, PAD_IN },
	/* CAM5 */
	{AAI_SIG_I2S3_FRAME,	 19, PAD_IN },
	{AAI_SIG_I2S3_IN,	 18, PAD_IN },

	{-1,			 -1,       0}
};

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",
	"music-out-stereo1",
	"music-in-stereo0",
	"music-in-stereo1",
	"music-in-stereo2",
	"music-in-stereo3",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set mezz_dev_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},
	{AAI_SLAVE(1)},

	/* Don't remove */
	{-1, 0, 0, 0},
};

static struct aai_platform_data mezz_dev_aai_pdata = {
	.pad         = mezz_dev_aai_pads,
	.aai_conf    = mezz_dev_aai_conf_set,
	.device_list = aai_dev_list,
};

static unsigned long mezz_dev_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long mezz_dev_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static struct pinctrl_map mezz_dev_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, mezz_dev_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, mezz_dev_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, mezz_dev_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, mezz_dev_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, mezz_dev_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, mezz_dev_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, mezz_dev_aai_pinconf),
};

/*****************
 * WAU8822
 *****************/

#include <sound/soc.h>
static struct i2c_board_info __initdata mezz_dev_wau8822_board_info = {
	I2C_BOARD_INFO("wau8822", 0x1a),
	.irq = -1,
};

static struct snd_soc_dai_link parrot_fc7100_dai[] = {
	{
		.name           = "wau8822",
		.codec_name     = "wau8822.2-001a",
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
	.max_speed_hz       = 100000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};


/*************
 * SD1
 *************/

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>
#include "fc7100-mezzs.h"

static struct acs3_plat_data fc7100_sdhci1_pdata = {
	.led_gpio   = -1,      /* No activity led GPIO */
	.wp_gpio    = FC7100DEV_IOEXPAND_GPIO_NR(5), /* GPIO 5 on IOEXPAND0 is write protect status */
	.cd_gpio    = FC7100DEV_IOEXPAND_GPIO_NR(4), /* GPIO 4  on IOEXPAND0 is card detect */
	.rst_gpio   = -1,
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

static struct acs3_regulator_gpios fc7100_sdhci1_regulator_gpios = {
	.bus_1v8_gpio = P7_GPIO_NR(217),
	.bus_1v8_active_low = 1,
};

static struct acs3_plat_data fc7100_sdhci1_pdata_extern = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};


/*************
 * IO Expander
 *************/


static char const* const mezz_dev_ioexpand0_names[24] = {
	"IPod_RSTn",
	"USB0_nFault",
	"USB0_nEn",
	"USB0_HUB_RSTn",
	"SD_CDn",
	"SD_WPn",
	"SD_3V3_EN",
	"LCD_BKL_EN",
	"LCD1_RST_TS",
	"LCD1_nRST",
	"Octopus_PWR_EN",
	"OCTO_RSTn",
	"ETH_RSTn",
	"VDD_SD_EN",
	"CAM0_nRST",
	"CAM0_5V_PWR_EN",
	"CAM0_PWDN",
	"CAM5_nRST",
	"CAM5_5V_PWR_EN",
	"CAM5_PWDN",
	"TP722",
	"TP723",
	"TP724",
	"TP725",
};

static int mezz_dev_ioexpand0_setup(struct i2c_client *client,
					   unsigned gpio, unsigned ngpio,
					   void *context)
{
	char *screen_name = "kyocera";
	union i2c_smbus_data data;
	int xres, yres;

	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(0), GPIOF_OUT_INIT_LOW, "ipod-rst");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(1), GPIOF_OUT_INIT_LOW, "USB0-nFault");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(2), GPIOF_OUT_INIT_LOW, "USB0-nEn");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(3), GPIOF_OUT_INIT_HIGH, "USB0-HUB_RSTn");
	/* 4 : SD_CDn */
	/* 5 : SD_WPn */
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(6), GPIOF_IN, "HOST_MODE_3V3");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(7), GPIOF_OUT_INIT_HIGH, "lcd-bkl-en");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(8), GPIOF_OUT_INIT_LOW, "LCD1_RST_TS");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(9), GPIOF_OUT_INIT_HIGH, "LCD1_nRST");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(10), GPIOF_OUT_INIT_HIGH, "octopus-pwr-en");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(11), GPIOF_OUT_INIT_HIGH, "octopus-rstn");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(12), GPIOF_OUT_INIT_HIGH, "eth_RSTn");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(13), GPIOF_OUT_INIT_HIGH, "vdd-sd-en");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(14), GPIOF_OUT_INIT_HIGH, "cam0-nrst");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(15), GPIOF_OUT_INIT_HIGH, "cam0-5v_pwr_en");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(16), GPIOF_OUT_INIT_HIGH, "cam0-pwdn");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(17), GPIOF_OUT_INIT_HIGH, "cam5-nrst");
	p7brd_export_gpio(FC7100DEV_IOEXPAND_GPIO_NR(18), GPIOF_OUT_INIT_HIGH, "cam5-5v_pwr_en");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(19),  GPIOF_OUT_INIT_HIGH, "cam5-nrst");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(20),  GPIOF_OUT_INIT_LOW, "TP722");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(21),  GPIOF_OUT_INIT_LOW, "TP723");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(22),  GPIOF_OUT_INIT_LOW, "TP724");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(23),  GPIOF_OUT_INIT_LOW, "TP725");

	/* If avi chip is pluged on mezz, force configuration */
	if(i2c_smbus_xfer(i2c_get_adapter(2), 0x3b, 0,
			  I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE_DATA, &data) >= 0) {
		pr_info("Detected sii hdmi chip %d-0x%02X.\n", 2, 0x3b);
		screen_name = "sii-hdmi";
	}

	if (screen_name != NULL)
		//	fc7100_mezz_avi_update(screen_name, 0);
		fc7100_mezz_avi_init_lcd_screen(screen_name, &xres, &yres);

	return 0;
}

#define P7_EXTERNAL_IRQ_BASE (NR_IRQS - 24)

static struct pca953x_platform_data mezz_dev_pca953x_pdata[] = {
	{
		/* We put it on top of the "internal" GPIOs */
		.gpio_base = FC7100_IOEXPAND0_FIRST_GPIO,
		.irq_base  = P7_EXTERNAL_IRQ_BASE,
		/*.names = mezz_dev_ioexpand0_names,*/
		.setup = mezz_dev_ioexpand0_setup,
	},
};

static struct i2c_board_info __initdata mezz_dev_i2c_infos_ioexpands[] = {
	{
		I2C_BOARD_INFO("tca6424", 0x23),
		.platform_data = &mezz_dev_pca953x_pdata[0],
		.irq =P7_GPIO_NR(160),
	},
};


/*************
 * USB hub
 *************/


static struct smsc82514_pdata hub_init = {
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = 0, /* FC7100DEV_IOEXPAND_GPIO_NR(3) not accepted*/
};

static struct i2c_board_info __initdata mezz_dev_i2c_usb_board_info[] = {
	{
		/* USB HUB SMSC 82514 */
		I2C_BOARD_INFO("smsc82514", 0x2c),
		.platform_data = &hub_init,
		.irq = -1,
	}
};


/************************
 * LVDS serialiser config
 ************************/

#include <i2c/ti_ub925_lvds.h>

/* Extra command for deserializer : enable gpio0 as output high (touchscreen nreset) */
static struct dsr_i2c_cmd ti_lvds_dsr_command[] = {
	{ .reg = 0x1d, .data  = 0x9 },
	{ .reg = 0, .data  = 0 },
};

static struct ti_lvds_platform_data ti_lvs_pdata = {
	.cmd = ti_lvds_dsr_command,
	.premap = {
		.slave_id = 0x4c,
		.slave_alias = 0x4c,
	},
	.nb_i2c_slave = 1,
};

static struct i2c_board_info __initdata fc7100_lvds_board_info = {
	I2C_BOARD_INFO("lvds", 0xc),
	.irq = P7_GPIO_NR(161),
	.platform_data = &ti_lvs_pdata,
};

/** Touchscreens **/

#include "input/touchscreen/atmel_mxt_ts.h"

static struct mxt_platform_data fc7100_atmel_mxt_pdata = {
	//.orient = MXT_NORMAL,
	.cfg_name = "maxtouch.cfg",
	.fw_name = "maxtouch.fw",
	.irqflags = IRQF_TRIGGER_FALLING,
};

static struct i2c_board_info __initdata fc7100_atmel_mxt_board_info = {
	I2C_BOARD_INFO("atmel_mxt_ts", 0x4c),
	.platform_data = &fc7100_atmel_mxt_pdata,
	/* We use an IRQ muxed over the LVDS. This is a very ugly
	 * hack. The clean version would be to implement an irq_chip in
	 * the lvds driver, but I don't think it can discriminate
	 * multiple interrupt sources. So it would only be one shared
	 * IRQ. Let's keep it simple for now. */
	.irq = -1,
};

/***********************
 * AVI configuration
 ***********************/

/***********************
 * Camera configuration
 ***********************/
#define HD_WIDTH 1280
#define HD_HEIGHT 720
#define FULL_HD_WIDTH 1920
#define FULL_HD_HEIGHT 1080

/**
 * HDMI Input configuration
 */

#define CAM0_I2C_BUS		1

#define P7_CAM0_AVI_RAM_SIZE		(FULL_HD_WIDTH * FULL_HD_HEIGHT * 4 * 4)

static unsigned long mezz_dev_cam0_pincfg[] = {
	P7CTL_SMT_CFG(OFF),
};

static struct pinctrl_map mezz_dev_cam0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINCFG(P7_CAM_0_CLK, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINCFG(P7_CAM_0_DATA00, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINCFG(P7_CAM_0_DATA01, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINCFG(P7_CAM_0_DATA02, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINCFG(P7_CAM_0_DATA03, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINCFG(P7_CAM_0_DATA04, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINCFG(P7_CAM_0_DATA05, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINCFG(P7_CAM_0_DATA06, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINCFG(P7_CAM_0_DATA07, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINCFG(P7_CAM_0_DATA08, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINCFG(P7_CAM_0_DATA09, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINCFG(P7_CAM_0_DATA10, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINCFG(P7_CAM_0_DATA11, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINCFG(P7_CAM_0_DATA12, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINCFG(P7_CAM_0_DATA13, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINCFG(P7_CAM_0_DATA14, mezz_dev_cam0_pincfg),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINCFG(P7_CAM_0_DATA15, mezz_dev_cam0_pincfg)
};

/* ADV7611 HDMI input I2C subdevice addresses */
#define ADV7611_CP_SLAVE_ADDRESS		0x44
#define ADV7611_HDMI_SLAVE_ADDRESS		0x68
#define ADV7611_EDID_SLAVE_ADDRESS		0x6C
#define ADV7611_REPEATER_SLAVE_ADDRESS		0x64
#define ADV7611_DPLL_SLAVE_ADDRESS		0x4c
#define ADV7611_INFOFRAME_SLAVE_ADDRESS		0x7C
#define ADV7611_CEC_SLAVE_ADDRESS		0x80

#define ADV7611_HDMI_RECEIVER_WIDTH             FULL_HD_WIDTH
#define ADV7611_HDMI_RECEIVER_HEIGHT            FULL_HD_HEIGHT

#define CAM0_NRST                       FC7100DEV_IOEXPAND_GPIO_NR(16)
#define CAM0_IRQ                        P7_GPIO_NR(154)

static int cam0_power_on(void)
{
	gpio_set_value_cansleep(CAM0_NRST, 1);
	return 0;
}

static int cam0_power_off(void)
{
	gpio_set_value_cansleep(CAM0_NRST, 0);
	return 0;
}

static struct adv7604_platform_data cam0_adv7604_platform_data = {
	.disable_pwrdnb		     = 0,
	.op_ch_sel		     = ADV7604_OP_CH_SEL_RGB,
	.alt_gamma		     = 0,
	.op_656_range		     = 1,
	.rgb_out		     = 0,
	.alt_data_sat		     = 1,
	.op_format_sel		     = ADV7604_OP_FORMAT_SEL_SDR_ITU656_8,
	.int1_config		     = ADV7604_INT1_CONFIG_ACTIVE_LOW,
	.inp_color_space	     = ADV7604_INP_COLOR_SPACE_AUTO,
	.connector_hdmi		     = 1,
	.insert_av_codes	     = 1,
	.blank_data		     = 1,
	.i2c_cec		     = (ADV7611_CEC_SLAVE_ADDRESS >> 1),
	.i2c_infoframe		     = (ADV7611_INFOFRAME_SLAVE_ADDRESS >> 1),
	.i2c_afe		     = (ADV7611_DPLL_SLAVE_ADDRESS >> 1),
	.i2c_repeater		     = (ADV7611_REPEATER_SLAVE_ADDRESS >> 1),
	.i2c_edid		     = (ADV7611_EDID_SLAVE_ADDRESS >> 1),
	.i2c_hdmi		     = (ADV7611_HDMI_SLAVE_ADDRESS >> 1),
	.i2c_cp			     = (ADV7611_CP_SLAVE_ADDRESS >> 1),
	.default_width		     = ADV7611_HDMI_RECEIVER_WIDTH,
	.default_height		     = ADV7611_HDMI_RECEIVER_HEIGHT,
	.connector_hdmi		     = 1,
	.power_on                    = &cam0_power_on,
	.power_off                   = &cam0_power_off,
	.cam_it                      = -1,
};

static struct i2c_board_info cam0_adv7611_device[] = {
	{
		I2C_BOARD_INFO("adv7611", 0x4C),
		.platform_data = &cam0_adv7604_platform_data,
	},
};

static struct avicam_subdevs mezz_dev_cam0_subdevs[] = {
	{
		.board_info = &cam0_adv7611_device[0],
		.i2c_adapter_id = CAM0_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_dummy_info mezz_dev_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code	    = V4L2_MBUS_FMT_YUYV8_1X16,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = HD_WIDTH,
		.height	    = HD_HEIGHT,
	},
};

static struct avicam_platform_data mezz_dev_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 10,
	.dummy_driver_info = &mezz_dev_cam0_dummy_driver_info,
	.subdevs	   = mezz_dev_cam0_subdevs,
};

static u64 mezz_dev_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device mezz_dev_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &mezz_dev_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


#include <media/tvp5150.h>

#define CAM1_WIDTH	720
#define CAM1_HEIGHT	576
#define CAM1_N_BUFFERS    4    /* #define TVP5151_VIDEO_DECODER_N_BUFFERS 4 */
#define CAM1_PIXEL_SIZE   4
#define CAM1_FRAME_SIZE PAGE_ALIGN(CAM1_WIDTH * CAM1_HEIGHT * CAM1_PIXEL_SIZE)
#define P7_CAM1_AVI_RAM_SIZE PAGE_ALIGN(CAM1_FRAME_SIZE * CAM1_N_BUFFERS)

#define CAM1_I2C_BUS		2

#define CAM1_RST                        P7_GPIO_NR(12)
#define CAM1_IRQ                        P7_GPIO_NR(13)

static unsigned long mezz_dev_cam1_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};


static struct pinctrl_map mezz_dev_cam1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINCFG(P7_CAM_1_CLK, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA08),
	P7_INIT_PINCFG(P7_CAM_1_DATA08, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA09),
	P7_INIT_PINCFG(P7_CAM_1_DATA09, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA10),
	P7_INIT_PINCFG(P7_CAM_1_DATA10, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA11),
	P7_INIT_PINCFG(P7_CAM_1_DATA11, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA12),
	P7_INIT_PINCFG(P7_CAM_1_DATA12, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA13),
	P7_INIT_PINCFG(P7_CAM_1_DATA13, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA14),
	P7_INIT_PINCFG(P7_CAM_1_DATA14, mezz_dev_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA15),
	P7_INIT_PINCFG(P7_CAM_1_DATA15, mezz_dev_cam1_pinconf),
};

static struct tvp5150_platform_data mezz_dev_tvp5150_platform_data = {
	.mbus_type = V4L2_MBUS_BT656
};


static struct i2c_board_info mezz_dev_cam1_i2c_devices[] = {
	{
		I2C_BOARD_INFO("tvp5150", 0x5C),
		.platform_data = &mezz_dev_tvp5150_platform_data,
	}
};

static struct avicam_subdevs mezz_dev_cam1_subdevs[] = {
	{
		.board_info = &mezz_dev_cam1_i2c_devices[0],
		.i2c_adapter_id = CAM1_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_dummy_info mezz_dev_cam1_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = CAM1_WIDTH,
		.height	    = CAM1_HEIGHT,
	},
};


static struct avicam_platform_data mezz_dev_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
		.ipc        = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = mezz_dev_cam1_subdevs,
	.dummy_driver_info = &mezz_dev_cam1_dummy_driver_info,
};


static u64 mezz_dev_cam1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device mezz_dev_cam1_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &mezz_dev_cam1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


void __init mezz_dev_avi_init_cameras(void)
{
	/* Direct interrupt for CAM0 (gpio expander bypass) */
	p7_gpio_interrupt_register(CAM0_IRQ);
	p7_gpio_interrupt_register(CAM1_IRQ);
	cam0_adv7604_platform_data.cam_it = CAM0_IRQ;
	p7brd_export_gpio(P7_GPIO_NR(CAM1_RST), GPIOF_OUT_INIT_LOW, "cam1_rst");

	p7_init_avicam(&mezz_dev_cam0_dev,
		       &mezz_dev_cam0_pdata,
		       mezz_dev_cam0_pins,
		       ARRAY_SIZE(mezz_dev_cam0_pins));

	p7_init_avicam(&mezz_dev_cam1_dev,
		       &mezz_dev_cam1_pdata,
		       mezz_dev_cam1_pins,
		       ARRAY_SIZE(mezz_dev_cam1_pins));
}

static struct avifb_overlay mezz_dev_avi_lcd0_overlays_novtk[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.zorder		 = -1,
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
	},
};


/********************
 * HDMI Output config
 ********************/

#include "../../../drivers/video/mxc/siihdmi.h"

static struct siihdmi_platform_data fc7100_siihdmi_data = {
// XXX
//	.reset       = mx51_efikamx_display_reset,

	.vendor      = "Genesi",
	.description = "Efika MX",

	.lcd_id = "lcd.1",

	.hotplug     = {
	/* .start = P7_GPIO_NR(77), irq is dynamically read from i2c_client */
		//.end   = IOMUX_TO_IRQ(MX51_PIN_DISPB2_SER_DIO),
		.name  = "video-hotplug",
		.flags = IRQF_ONESHOT | IRQF_TRIGGER_LOW,
	},

// XXX
//	.pixclock    = KHZ2PICOS(133000L),
};

static struct i2c_board_info __initdata fc7100_siihdmi_board_info[] = {
	{
		I2C_BOARD_INFO("siihdmi", 0x3b),
		.platform_data = &fc7100_siihdmi_data,
		.irq = 161,
	},
};

#define MEZZ_DEV_ON_DB    1 << 0 /* signal on Daughter board */
#define MEZZ_DEV_ON_OCTO  1 << 1 /* signal on Octpus  */
#define MEZZ_DEV_ON_BOARD 1 << 2 /* SD signal on SD slot */

#define MEZZ_DEV_CONF_WHERE(config) (config|MEZZ_DEV_ON_BOARD)?"on board": \
	((config|MEZZ_DEV_ON_OCTO)?"on octopus":				 \
	 ((config|MEZZ_DEV_ON_DB)?"on dauther Board ":"nowhere"))

static int __init get_gpio_value(unsigned gpio)
{
	int value;
	gpio_request(gpio, "gpio");
	gpio_direction_input(gpio);
	value = gpio_get_value(gpio);
	gpio_free(gpio);
	return value;
}

static void __init get_config(int *sd_conf, int *spi_conf, int *i2s_conf)
{
	int data05 = get_gpio_value(188);
	int data07 = get_gpio_value(190);
	int data09 = get_gpio_value(192);
	int data10 = get_gpio_value(194);

	if (data05 == 0 && data07 == 0)
		*sd_conf = MEZZ_DEV_ON_BOARD;
	else if (data05 == 0 && data07 == 1)
		*sd_conf = MEZZ_DEV_ON_DB;
	else if (data05 == 1 && data07 == 0)
		*sd_conf = MEZZ_DEV_ON_OCTO;

	if (data09 != 0)
		*spi_conf = MEZZ_DEV_ON_OCTO;
	else
		*spi_conf = MEZZ_DEV_ON_DB;

	if (data10 != 0)
		*i2s_conf = MEZZ_DEV_ON_DB;
}

static void __init init_board(void)
{
	int xres, yres;
	unsigned int mod_settings = 0;
	int sd_conf, spi_conf, i2s_conf;

	fc7100_init_module(mod_settings);

	get_config(&sd_conf, &spi_conf, &i2s_conf);
	pr_info("Mezz Dev SD is %s ", MEZZ_DEV_CONF_WHERE(sd_conf));
	pr_info("Mezz Dev SPI is %s ", MEZZ_DEV_CONF_WHERE(spi_conf));
	pr_info("Mezz Dev I2S is %s ", MEZZ_DEV_CONF_WHERE(i2s_conf));

	p7brd_init_i2cm(1, 200);
	p7brd_init_i2cm(2, 200);

	p7brd_init_uart(0,1);

	/* SD inits */
	if (sd_conf | MEZZ_DEV_ON_BOARD)
		p7brd_init_sdhci(1, &fc7100_sdhci1_pdata, NULL, NULL,
				 &fc7100_sdhci1_regulator_gpios, NULL, 0);
	else
		p7brd_init_sdhci(1, &fc7100_sdhci1_pdata_extern, NULL, NULL,
				 NULL, NULL, 0);

	/* I2C init */
	parrot_init_i2c_slave(1, &mezz_dev_i2c_infos_ioexpands[0], "IO-Expander", P7_I2C_IRQ);
	parrot_init_i2c_slave(1, &mezz_dev_i2c_usb_board_info[0], "usb hub", P7_I2C_NOIRQ);

	/* USB0 is device only */
	p7brd_init_udc(0, -1);

	/* USB1 is host only */
	p7brd_init_hcd(1, -1);

	p7_init_spim_slave(0, &fc7100_spim_dev);

	p7_init_ether(PHY_IFACE_RGMII, -1, P7CTL_DRV_CFG(5));

	/* Init sound */
	p7_init_aai(mezz_dev_aai_pins, ARRAY_SIZE(mezz_dev_aai_pins), &mezz_dev_aai_pdata);
	parrot_init_i2c_slave(2, &mezz_dev_wau8822_board_info, "wau8822", P7_I2C_NOIRQ);
	p7_init_dev(&fc7100_asoc_dev, NULL, NULL, 0);

	/* LVDS / Touchsreen / LCD */
	parrot_init_i2c_slave(2,
	                      &fc7100_lvds_board_info,
	                      "LVDS serializer",
			      P7_I2C_IRQ);
	/* -1 for mxt irq is for mxt driver use only (means soft lvds irq) */
	parrot_init_i2c_slave(2,
	                      &fc7100_atmel_mxt_board_info,
	                      "Atmel maXTouch",
			      P7_I2C_NOIRQ);
	parrot_init_i2c_slave(2,
			      fc7100_siihdmi_board_info,
			      "sii HDMI 9024A tx",
			      P7_I2C_SHAREDIRQ);

	p7_init_avi();
	fc7100_mezz_avi_init_lcd_screen("kyocera", &xres, &yres);
	fc7100_mezz_avi_init_lcd(mezz_dev_avi_lcd0_overlays_novtk,
				 ARRAY_SIZE(mezz_dev_avi_lcd0_overlays_novtk));
	mezz_dev_avi_init_cameras();

	p7_init_venc();
	p7_init_vdec();

	fc7100_init_spim_single(0, 11, 10, 8, 9);
	if (spi_conf | MEZZ_DEV_ON_OCTO) {
		fc7100_init_mpegts_single(0, 16, 17);
		fc7100_init_mpegts_single(1, 18, 19);
	}

	/* Export unconfigured devices informations */
	p7brd_export_i2c_hw_infos(1, 0x10, "2C", "ipod");
}

static void __init mezz_dev_reserve_mem(void)
{
	fc7100_mezz_reserve_mem_for_lcd(mezz_dev_avi_lcd0_overlays_novtk, ARRAY_SIZE(mezz_dev_avi_lcd0_overlays_novtk));
	p7_reserve_avicammem(&mezz_dev_cam0_dev, P7_CAM0_AVI_RAM_SIZE);
	p7_reserve_avicammem(&mezz_dev_cam1_dev, P7_CAM1_AVI_RAM_SIZE);

#define FC7100_HX280_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(FC7100_HX280_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);
#define FC7100_MPGTS_SIZE (CONFIG_ARCH_PARROT7_FC7100_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, FC7100_MPGTS_SIZE);
	p7_reserve_mpegtsmem(1, FC7100_MPGTS_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_MEZZ_DEV, "Mezz Dev")
	.reserve        = &mezz_dev_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END

