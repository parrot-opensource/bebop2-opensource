/*
 * linux/arch/arm/mach-parrot7/board-rnb6.c - Parrot7 RNB6 board implementation
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * author:  Christian Rosalie <christian.rosalie@parrot.com>
 * date:    21-juillet-2014
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <asm/system_info.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/usb-p7.h>
#include <mach/ether.h>
#include <spi/p7-spi.h>
#include <media/tc358764.h>
#include "board-common.h"
#include "common.h"
#include "pinctrl.h"
#include "venc.h"
#include "spi.h"
#include "usb.h"
#include "avi.h"
#include "gpu.h"
#include "p7_temperature.h"
#include "pinctrl.h"
#include "lcd-monspecs.h"
#include "vdec.h"
#include <media/tw9990.h>
#include "sii_platform_data.h"
#include "nand.h"
#include "usb.h"

#define RNB6_BRD_NAME "rnb6"

enum rnb6_tuner_type {
	RNB6_TUNER_NONE = 0,
	RNB6_TUNER_OCTOPUS,
	RNB6_TUNER_HIRSCHMANN,
};

enum rnb6_tuner_type rnb6_tuner_device;

static int __init parrot_rnb6_tuner(char *str)
{
	if (!strcmp(str, "hirschmann")) {
		rnb6_tuner_device = RNB6_TUNER_HIRSCHMANN;
	}
	else {
		/* Tuner octopus is set by default,
			rnb6_tuner=octopus */
		rnb6_tuner_device = RNB6_TUNER_OCTOPUS;
	}

	return 1;
}
__setup("rnb6_tuner=", parrot_rnb6_tuner);


/*********
 * GPIOs
 *********/

#define UC_INT		P7_GPIO_NR(57)

#define P7MU_INT	P7_GPIO_NR(72)
#define ETH_NRST	P7_GPIO_NR(73)
#define EMMC_NRST	P7_GPIO_NR(74)
#define IPOD_RST_N	P7_GPIO_NR(75)

#define CMD_RELAY	P7_GPIO_NR(76)
#define AMPLI_MUTE	P7_GPIO_NR(90)
#define AMPLI_STBY	P7_GPIO_NR(91)
#define DAC_MUTE_P7	P7_GPIO_NR(92)

#define IRQ_WL8		P7_GPIO_NR(120)
#define WL_EN		P7_GPIO_NR(123)
#define USB_HUB_RST_N	P7_GPIO_NR(124)
#define VBUS_0_FAULT	P7_GPIO_NR(125)
#define EN_5V_USB0	P7_GPIO_NR(126)	/* See EN_5V_USB0 in schematics */

#define CAN_SILENCE_CONT	P7_GPIO_NR(129)
#define K_LINE_NSLP		P7_GPIO_NR(130)
#define ENABLE_BSL		P7_GPIO_NR(131)
#define EXTERNAL_RESET_BSL	P7_GPIO_NR(132)

#define NRST_TFT	P7_GPIO_NR(152)
#define EN_TFT		P7_GPIO_NR(153)

#define NRST_TOUCH	P7_GPIO_NR(154)
#define INT_TOUCH	P7_GPIO_NR(155)

#define INT_MIPI	P7_GPIO_NR(156)
#define NRST_MIPI	P7_GPIO_NR(157)
#define MHL_CAM_NRST	P7_GPIO_NR(158)
#define MHL_CAM_INT1	P7_GPIO_NR(159)
#define MHL_CAM_EN_CD	P7_GPIO_NR(160)

#define HDMI_INT	P7_GPIO_NR(171)
#define HDMI_RESET	P7_GPIO_NR(172)
#define CVBS_RSTN_A	P7_GPIO_NR(173)
#define CVBS_PDN_A	P7_GPIO_NR(174)
#define CVBS_INT_B	P7_GPIO_NR(175)
#define MHL_NRST	P7_GPIO_NR(176)
#define MHL_INT		P7_GPIO_NR(177)
#define MHL_5V_FAULT	P7_GPIO_NR(178)

#define RDS_TUNER_OCTO_INT	P7_GPIO_NR(33)
#define QSI_TUNER_OCTO_PWDOWN	P7_GPIO_NR(42)

#define EEPRM_WP		P7_GPIO_NR(160)
#define DASHCAM_BOOT_USB	P7_GPIO_NR(161)

/* See Tuner_Hirch_Bootsel in schematic Due to a peculiar use of SPI, and the
   fact that SPI_00a cannot be set as GPIO.  CS is attached to
   Tuner_Hirch_Bootsel and the boosel pin of Hirschmann tuner is controlled by
   the external MCU. */
#define AMFM_TUNER_CS		P7_GPIO_NR(128)

#define AMFM_TUNER_RST_N	QSI_TUNER_OCTO_PWDOWN
#define AMFM_TUNER_IT		RDS_TUNER_OCTO_INT
/*#define AMFM_TUNER_CS */

/**************
 * Pins config
 **************/
#define	ETH_MII_TXER	134
#define	ETH_MII_RXER	135
#define	ETH_MII_COL	137
#define	ETH_MII_RXC	144
#define	ETH_MII_RXD_0	145
#define	ETH_MII_RXD_1	146
#define	ETH_MII_RXD_2	147
#define	ETH_MII_RXD_3	148

#include "gpio.h"

struct gpio_setting {
	int gpio;
	char *name;
	int default_value;
	int interrupt;
	int bidir;
};
struct gpio_setting rnb6_gpios[] __initdata = {
	/* Audio */
	{
		.gpio		= P7_GPIO_NR(AMPLI_MUTE),
		.name		= "amp-mute",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(AMPLI_STBY),
		.name		= "amp-stby",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(DAC_MUTE_P7),
		.name		= "dac-mute",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(CMD_RELAY),
		.name		= "cmd-relay",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	/* Touchscreen */
	{
		.gpio		= P7_GPIO_NR(NRST_TOUCH),
		.name		= "touch-en",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(INT_TOUCH),
		.name		= "touch-it",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 1,
		.bidir		= 0,
	},
	/* MIPI */
	{
		.gpio		= P7_GPIO_NR(NRST_MIPI),
		.name		= "mipi-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(INT_MIPI),
		.name		= "mipi-it",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 1,
		.bidir		= 0,
	},
	/* CVBS */
	{
		.gpio		= P7_GPIO_NR(CVBS_RSTN_A),
		.name		= "cvbs-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(CVBS_PDN_A),
		.name		= "cvbs-pdn",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(CVBS_INT_B),
		.name		= "cvbs-it",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 1,
		.bidir		= 0,
	},
	/* MHL */
	/*{
		.gpio		= P7_GPIO_NR(MHL_NRST),
		.name		= "mhl-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(MHL_INT),
		.name		= "mhl-it",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 1,
		.bidir		= 0,
	},*/
	{
		.gpio		= P7_GPIO_NR(MHL_5V_FAULT),
		.name		= "mhl-flt",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 0,
		.bidir		= 0,
	},
	/* UC */
	{
		.gpio		= P7_GPIO_NR(UC_INT),
		.name		= "uc-it",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 1,
		.bidir		= 0,
	},
	/* Ipod */
	{
		.gpio		= P7_GPIO_NR(IPOD_RST_N),
		.name		= "ipod-rst",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	/* CAN */
	{
		.gpio		= P7_GPIO_NR(CAN_SILENCE_CONT),
		.name		= "can-silent-cont",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(K_LINE_NSLP),
		.name		= "k-line-nslp",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
};

struct gpio_setting rnb6_gpios_hw00[] __initdata = {
	/* MHL : dashcam */
	{
		.gpio		= P7_GPIO_NR(MHL_CAM_EN_CD),
		.name		= "dashcam-encd",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	/* UC */
	{
		.gpio		= P7_GPIO_NR(ENABLE_BSL),
		.name		= "uc-en",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(EXTERNAL_RESET_BSL),
		.name		= "uc-rst",
		.default_value	= GPIOF_OUT_INIT_HIGH,
		.interrupt	= 0,
		.bidir		= 0,
	},
};


struct gpio_setting rnb6_gpios_hw01[] __initdata = {
	{
		.gpio		= P7_GPIO_NR(EEPRM_WP),
		.name		= "eeprom-wp",
		.default_value	= GPIOF_OUT_INIT_HIGH,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(DASHCAM_BOOT_USB),
		.name		= "dashcam-boot-usb",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},
};

struct gpio_setting rnb6_gpios_hirschmann[] __initdata = {
	{
		.gpio		= P7_GPIO_NR(AMFM_TUNER_RST_N),
		.name		= "amfm-tuner-rst-n",
		.default_value	= GPIOF_INIT_HIGH,
		.interrupt	= 0,
		.bidir		= 0,
	},
	{
		.gpio		= P7_GPIO_NR(AMFM_TUNER_IT),
		.name		= "amfm-it",
		.default_value	= GPIOF_DIR_IN,
		.interrupt	= 1,
		.bidir		= 1,
	},
	/* This pin is manage by the external MCU (not P7)
	{
		.gpio		= P7_GPIO_NR(AMFM_TUNER_BOOTSEL),
		.name		= "amfm-tuner-bootsel",
		.default_value	= GPIOF_OUT_INIT_LOW,
		.interrupt	= 0,
		.bidir		= 0,
	},*/
	{//TODO Find a gpio for CS
		.gpio		= P7_GPIO_NR(AMFM_TUNER_CS),
		.name		= "amfm-cs",
		.default_value	= GPIOF_INIT_HIGH,
		.interrupt	= 0,
		.bidir		= 0,
	},
};

/*****************************
 * P7MU Power Management Unit
 *****************************/

#include <mfd/p7mu.h>
#include <i2c/p7-i2cm.h>
#include "p7mu.h"

static unsigned long rnb6_p7mu_reboot_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN)  | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
	P7_INIT_PINCFG(P7_REBOOT_P7MU, rnb6_p7mu_reboot_pinconf),
};

static struct p7mu_plat_data rnb6_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(P7MU_INT),
	/* Il n'y a pas de 32K externe sur la P7MU car c'est le µC qui fait la RTC puisque nous coupons complètements les alims de la P7MU en mode  veille. */
	.int_32k    = true,    /* No External 32kHz clock. */
	.int_32m    = false,    /* External 48mHz clock. */
	.gpio_event_sel = P7MU_PMC_GPIO5_EDGE_RISING, /* Wakeup on gpio5 edge rising event */
};


/* ADC */
#include <mach/p7-adc.h>
static struct p7_temp_chan p7mu_adc_channels[] = {
	{
		.channel = 3,
		.freq = 160000,
		.name = "ref",
	},

	/* ADC_01 | DDR 3 */
	{
		.channel = 1,
		.freq    = 160000,
		.name = "ddr",
	},
	/* ADC_02 | PCB BOTTOM */
	{
		.channel = 2,
		.freq    = 160000,
		.name = "pcb_bot",
	},
	/* ADC_04 | MOS 1 */
	{
		.channel = 4,
		.freq    = 160000,
		.name = "mos1",
	},
	/* ADC_05 | DC/DC */
	{
		.channel = 5,
		.freq    = 160000,
		.name = "dc_dc",
	},
	/* ADC_06 | PCB_1 [Wilink8] */
	{
		.channel = 6,
		.freq    = 160000,
		.name = "wifi",
	},
};

static struct p7_temp_chan_data p7mu_adc_chan_data = {
        .channels               = p7mu_adc_channels,
        .num_channels           = ARRAY_SIZE(p7mu_adc_channels),
		.temp_mode				= P7_TEMP_FC7100_HW08,
};

static struct platform_device fc7100_temp_device = {
	.name = "fc7100-temperature",
	.id = -1,
	.dev.platform_data = &p7mu_adc_chan_data,
};

/*************
 * I2C
 *************/
#include <i2c/muxes/p7-i2cmux.h>
#include "i2cm.h"

void rnb6_reset_hdmi_out(void)
{
	//TODO Does not work but gpio seems to be driven
	gpio_set_value(HDMI_RESET, 0);
	mdelay(10);
	gpio_set_value(HDMI_RESET, 1);
}

/* HDMI Output config */
#include "../../../drivers/video/mxc/siihdmi.h"
static struct siihdmi_platform_data rnb6_siihdmi_data = {
	.vendor      = "Genesi",
	.description = "Efika MX",

	.lcd_id = "lcd.0",

	.hotplug     = {
		.name  = "video-hotplug",
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWEDGE,
	},

	. input_format = YUV_422_8_MUX,

	//.reset = rnb6_reset_hdmi_out,
};

static struct i2c_board_info __initdata rnb6_siihdmi_board_info[] = {
	{
		I2C_BOARD_INFO("siihdmi", 0x39),
		.platform_data = &rnb6_siihdmi_data,
		.irq = P7_GPIO_NR(HDMI_INT),
	},
};



/*************
 * SD
 *************/

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>
#include "wl18xx.h"

static struct acs3_plat_data rnb6_sdhci_emmc_pdata = {
	.led_gpio   = -1,  /* No activity led GPIO */
	.wp_gpio    = -1,  /* No write protect */
	.cd_gpio    = -1,  /* No card detect */
	.rst_gpio   = P7_GPIO_NR(EMMC_NRST),
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,     /* emmc is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE,  /* bus voltage is fixed in hardware */
};


static __initdata struct wl18xx_resources rnb6_wl18xx_res = {
	.wlirq_gpio     = P7_GPIO_NR(IRQ_WL8),
	.wlen_gpio      = P7_GPIO_NR(WL_EN),
	.bt_rst_gpio    = P7MU_IO_NR(4),
	.bt_uart_slot   = 1,
	.wl_sdhci_slot  = 0,
};


/*************
 * Audio
 *************/
#include "aai.h"

static struct aai_pad_t rnb6_aai_pads[] = {
	/* Codec & DAC out clocks */
	{AAI_SIG_MAIN_I2S_FRAME, 17, PAD_OUT}, /* I2S_FSYNC Audio A */
	{AAI_SIG_DAC_BIT_CLOCK,	 16, PAD_OUT}, /* I2S_BLK Audio A  */
	{AAI_SIG_MCLK,		 15, PAD_OUT}, /* I2S_MCLK Audio A */

	/* I2S Input / Output */
	{AAI_SIG_I2S0_IN,	 21, PAD_IN },
	{AAI_SIG_IN_MIC0,	 21, PAD_IN }, /* I2C_SD0_IN_MIC_P7 */
	{AAI_SIG_OUT_DAC0,	 18, PAD_OUT}, /* I2S_SD0_DAC_FRONT_P7 */
	{AAI_SIG_OUT_DAC1,	 19, PAD_OUT}, /* I2S_SD1_DAC_REAR_P7 */
	{AAI_SIG_OUT_DAC2,	 20, PAD_OUT}, /* I2S_SD2_CODEC_P7 (sub) */

	/* PCM1 (Bluetooth) */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	/* SPDIF IN (dashcam) */
	{AAI_SIG_SPDIF_RX,	 27, PAD_IN },
	{AAI_SIG_SPDIF_TX,	 26, PAD_OUT},

	{-1,			 -1,       0}
};

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-quad0",	/* I2S_SD0_DAC_FRONT_P7 + REAR */
	"music-out-stereo2",	/* I2S_SD1_DAC_SUB */
	"voice-out-stereo",
	"pcm0-out",
	"spdif-out",

	/* Input Channels */
	"music-in-stereo0",	/* I2C_SD0_IN_MIC_P7 */
	"mic0-8k",
	"mic0-16k",
	"pcm0-in",
	"loopback-8k",
	"loopback-16k",
	"spdif-in",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set rnb6_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},
	{AAI_MASTER(1)},
	{AAI_MASTER(2)},

	/*Don't remove*/
	{-1, 0, 0, 0},
};

static struct aai_platform_data rnb6_aai_pdata = {
	.pad         = rnb6_aai_pads,
	.aai_conf    = rnb6_aai_conf_set,
	.device_list = aai_dev_list,
};

static struct aai_pad_t rnb6_hirschmann_aai_pads[] = {
	/* Codec & DAC out clocks */
	{AAI_SIG_MAIN_I2S_FRAME, 17, PAD_OUT}, /* I2S_FSYNC Audio A */
	{AAI_SIG_DAC_BIT_CLOCK,	 16, PAD_OUT}, /* I2S_BLK Audio A  */
	{AAI_SIG_MCLK,		 15, PAD_OUT}, /* I2S_MCLK Audio A */

	/* I2S Input / Output */
	{AAI_SIG_I2S0_IN,	 21, PAD_IN },
	{AAI_SIG_IN_MIC0,	 21, PAD_IN }, /* I2C_SD0_IN_MIC_P7 */
	{AAI_SIG_OUT_DAC0,	 18, PAD_OUT}, /* I2S_SD0_DAC_FRONT_P7 */
	{AAI_SIG_OUT_DAC1,	 19, PAD_OUT}, /* I2S_SD1_DAC_REAR_P7 */
	{AAI_SIG_OUT_DAC2,	 20, PAD_OUT}, /* I2S_SD2_CODEC_P7 (sub) */

	/* PCM1 (Bluetooth) */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	/* SPDIF IN (dashcam) */
	{AAI_SIG_SPDIF_RX,	 27, PAD_IN },
	{AAI_SIG_SPDIF_TX,	 26, PAD_OUT},

	/* for hirschmann tuner */
	{AAI_SIG_I2S3_FRAME,	 23, PAD_IN },
	{AAI_SIG_I2S3_IN,	 24, PAD_IN },

	{-1,			 -1,       0}
};

static char *aai_hirschmann_dev_list[] = {
	/* Output channels */
	"music-out-quad0",	/* I2S_SD0_DAC_FRONT_P7 + REAR */
	"music-out-stereo2",	/* I2S_SD1_DAC_SUB */
	"voice-out-stereo",
	"pcm0-out",
	"spdif-out",

	/* Input Channels */
	"music-in-stereo0",	/* I2C_SD0_IN_MIC_P7 */
	"mic0-8k",
	"mic0-16k",
	"pcm0-in",
	"loopback-8k",
	"loopback-16k",
	"spdif-in",
	"music-in-stereo3",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set rnb6_hirschmann_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},
	{AAI_MASTER(1)},
	{AAI_MASTER(2)},
	{AAI_SLAVE(3)},

	/*Don't remove*/
	{-1, 0, 0, 0},
};

static struct aai_platform_data rnb6_hirschmann_aai_pdata = {
	.pad         = rnb6_hirschmann_aai_pads,
	.aai_conf    = rnb6_hirschmann_aai_conf_set,
	.device_list = aai_hirschmann_dev_list,
};

static unsigned long rnb6_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long rnb6_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long rnb6_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static unsigned long rnb6_aai_pcm_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)   | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static struct pinctrl_map rnb6_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, rnb6_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, rnb6_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, rnb6_aai_pcm_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_15),
	P7_INIT_PINCFG(P7_AAI_15, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_16),
	P7_INIT_PINCFG(P7_AAI_16, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, rnb6_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_21),
	P7_INIT_PINCFG(P7_AAI_21, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_26),
	P7_INIT_PINCFG(P7_AAI_26, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_27),
	P7_INIT_PINCFG(P7_AAI_27, rnb6_aai_pinconf),
};

static struct pinctrl_map rnb6_hirschmann_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, rnb6_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, rnb6_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, rnb6_aai_pcm_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_15),
	P7_INIT_PINCFG(P7_AAI_15, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_16),
	P7_INIT_PINCFG(P7_AAI_16, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, rnb6_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_21),
	P7_INIT_PINCFG(P7_AAI_21, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_23),
	P7_INIT_PINCFG(P7_AAI_23, rnb6_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_24),
	P7_INIT_PINCFG(P7_AAI_24, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_26),
	P7_INIT_PINCFG(P7_AAI_26, rnb6_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_27),
	P7_INIT_PINCFG(P7_AAI_27, rnb6_aai_pinconf),
};

/* WAU8822 */

#include <sound/soc.h>
static struct i2c_board_info __initdata rnb6_wau8822_board_info = {
	I2C_BOARD_INFO("wau8822", 0x1a),
	.irq = -1,
};

static struct snd_soc_dai_link rnb6_dai[] = {
	{
		.name           = "wau8822",
		.codec_name     = "wau8822.0-001a",
		.codec_dai_name = "wau8822-hifi",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.dai_fmt        = SND_SOC_DAIFMT_I2S |
					SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS,
	},
};

/* FC7100 ASoC device */
static struct platform_device rnb6_asoc_dev = {
	.name           = "parrot-fc7100-audio",
	.id             = 0,
	.dev		= {
		.platform_data = rnb6_dai,
	}
};


/*************
 * USB hub
 *************/
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>

static struct smsc82514_pdata hub_init = {
	.us_port   = DS_HIGH,
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = P7_GPIO_NR(USB_HUB_RST_N),
};

static struct i2c_board_info __initdata rnb6_smsc_82512_board_info[] = {
	{
		/* USB HUB SMSC 82514 */
		I2C_BOARD_INFO("smsc82512", 0x2c),
		.platform_data = &hub_init,
		.irq = -1,
	}
};


/* AVI */
/*************
 * Framebuffer
 *************/
#define LCD0_WIDTH	1024
#define LCD0_HEIGHT	768
#define LCD0_PIX_SIZE	2
#define LCD0_N_BUFFERS	4

static unsigned long rnb6_avifb0_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_avifb0_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_LCD_0_CLK),
	P7_INIT_PINCFG(P7_LCD_0_CLK, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA08),
	P7_INIT_PINCFG(P7_LCD_0_DATA08, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA09),
	P7_INIT_PINCFG(P7_LCD_0_DATA09, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA10),
	P7_INIT_PINCFG(P7_LCD_0_DATA10, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA11),
	P7_INIT_PINCFG(P7_LCD_0_DATA11, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA12),
	P7_INIT_PINCFG(P7_LCD_0_DATA12, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA13),
	P7_INIT_PINCFG(P7_LCD_0_DATA13, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA14),
	P7_INIT_PINCFG(P7_LCD_0_DATA14, rnb6_avifb0_pinconf),
	P7_INIT_PINMAP(P7_LCD_0_DATA15),
	P7_INIT_PINCFG(P7_LCD_0_DATA15, rnb6_avifb0_pinconf),
};

static struct avifb_overlay rnb6_avi_lcd0_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.width	 = LCD0_WIDTH,
			.enabled = 0,
		},
		.zorder		 = -1,
		.dma_memory.end	 = LCD0_WIDTH * LCD0_HEIGHT
					* LCD0_N_BUFFERS * LCD0_PIX_SIZE,
	},
};

static struct avifb_platform_data rnb6_avifb0_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 1,
			.ipc	  = 1,
			.pad_select = 1,
		}},
        .lcd_format_control	= AVI_FORMAT_CONTROL_UYVY_2X8,
	.lcd_default_videomode	= &hdmi_1024x768p60_video_mode,
	.lcd_videomodes		= p7_all_video_modes,
	.output_cspace		= AVI_BT601_CSPACE,
	.caps			= AVI_CAP_LCD_0,
	.background    = 0x00ff00,

	/* default overlays */
        .overlays		= rnb6_avi_lcd0_overlays,
	.overlay_nr		= ARRAY_SIZE(rnb6_avi_lcd0_overlays),
	.width                  = 304,
	.height                 = 228,
};

/* Should we bother setting a narrower mask here? */
static u64 rnb6_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_avifb0_dev = {
	.name           = "avifb",
	.id             = 0,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &rnb6_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

#define LCD1_HEIGHT	1280
#define LCD1_WIDTH	720
#define LCD1_PIX_SIZE	2
#define LCD1_N_BUFFERS	4

static unsigned long rnb6_avifb1_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_avifb1_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINCFG(P7_LCD_1_CLK, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINCFG(P7_LCD_1_DATA00, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINCFG(P7_LCD_1_DATA01, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINCFG(P7_LCD_1_DATA02, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINCFG(P7_LCD_1_DATA03, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINCFG(P7_LCD_1_DATA04, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINCFG(P7_LCD_1_DATA05, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINCFG(P7_LCD_1_DATA06, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINCFG(P7_LCD_1_DATA07, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINCFG(P7_LCD_1_DATA08, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINCFG(P7_LCD_1_DATA09, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINCFG(P7_LCD_1_DATA10, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINCFG(P7_LCD_1_DATA11, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINCFG(P7_LCD_1_DATA12, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINCFG(P7_LCD_1_DATA13, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINCFG(P7_LCD_1_DATA14, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINCFG(P7_LCD_1_DATA15, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINCFG(P7_LCD_1_DATA16, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINCFG(P7_LCD_1_DATA17, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINCFG(P7_LCD_1_DATA18, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINCFG(P7_LCD_1_DATA19, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINCFG(P7_LCD_1_DATA20, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINCFG(P7_LCD_1_DATA21, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINCFG(P7_LCD_1_DATA22, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINCFG(P7_LCD_1_DATA23, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINCFG(P7_LCD_1_DEN, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINCFG(P7_LCD_1_HS, rnb6_avifb1_pinconf),
	P7_INIT_PINMAP(P7_LCD_1_VS),
	P7_INIT_PINCFG(P7_LCD_1_VS, rnb6_avifb1_pinconf),
};

static struct avifb_overlay rnb6_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.width	 = LCD1_WIDTH,
			.enabled = 0,
		},
		.zorder		 = -1,
		.dma_memory.end	 = LCD1_WIDTH * LCD1_HEIGHT
		* LCD1_N_BUFFERS * LCD1_PIX_SIZE,
	},
};

static struct avifb_platform_data rnb6_avifb1_pdata = {
	.lcd_interface		  = {{
			.free_run = 1,
			.itu656	  = 0,
			.ipc      = 1,
		}},
        .lcd_format_control	= AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode	= &rnb6_truly_video_mode,
	.lcd_videomodes		= p7_all_video_modes,
	.caps			= AVI_CAP_LCD_1,

	/* default overlays */
        .overlays		= rnb6_avi_lcd1_overlays,
	.overlay_nr		= ARRAY_SIZE(rnb6_avi_lcd1_overlays),
	/* Panel dimensions in mm */
	.width                  = 87,
	.height                 = 155,
};

/* Should we bother setting a narrower mask here? */
static u64 rnb6_avifb1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &rnb6_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


static struct avi_voc_plat_data rnb6_avi_voc0 = {
	.display = AVIFB_ID(0),
};

static struct avi_voc_plat_data rnb6_avi_voc1 = {
	.display = AVIFB_ID(0),
};

static struct avi_voc_plat_data rnb6_avi_voc2 = {
	.display = AVIFB_ID(0),
};


/***********************
   CAM 0 : MHL / HDMI IN
 ***********************/
/* EDID provided  */
#define CAM0_HEIGHT	720
#define CAM0_WIDTH	1280
#define CAM0_N_BUFFERS	8

#define CAM0_PIXEL_SIZE 2
#define CAM0_FRAME_SIZE PAGE_ALIGN(CAM0_WIDTH * CAM0_HEIGHT * CAM0_PIXEL_SIZE)
#define CAM0_AVI_RAM_SIZE PAGE_ALIGN(CAM0_FRAME_SIZE * CAM0_N_BUFFERS)

static unsigned long rnb6_mhl_cam0_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_mhl_cam0_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINCFG(P7_CAM_0_CLK, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINCFG(P7_CAM_0_DATA00, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINCFG(P7_CAM_0_DATA01, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINCFG(P7_CAM_0_DATA02, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINCFG(P7_CAM_0_DATA03, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINCFG(P7_CAM_0_DATA04, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINCFG(P7_CAM_0_DATA05, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINCFG(P7_CAM_0_DATA06, rnb6_mhl_cam0_pinconf),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINCFG(P7_CAM_0_DATA07, rnb6_mhl_cam0_pinconf),
};

static struct avicam_dummy_info rnb6_mhl_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = CAM0_WIDTH,
		.height	    = CAM0_HEIGHT,
	},
};

static struct avicam_platform_data rnb6_mhl_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &rnb6_mhl_cam0_dummy_driver_info,
};


static u64 rnb6_mhl_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_mhl_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &rnb6_mhl_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct sii5293_platform_data sii_mhl_pdata = {
	.version        = SII5293_PDATA_STRUCT_VERSION,
	.i2c_bus        = 0,
	.gpio_rst       = P7_GPIO_NR(MHL_NRST),
	.gpio_irq       = P7_GPIO_NR(MHL_INT),
	.output_format  = 8, /* YCbCr 4:2:2, embedded sync output */
	.input_dev_rap  = 1,
	.input_dev_rcp  = 1,
	.input_dev_ucp  = 1,
};

static struct platform_device sii_mhl_pdev = {
	.name           = "sii-5293",
	.id             = 0,
};

/******************
   CAM 1 : Dashcam
 ******************/
/* EDID need to be provided  */

#define CAM1_HEIGHT	720
#define CAM1_WIDTH	1280
#define CAM1_N_BUFFERS	8

#define CAM1_PIXEL_SIZE 2
#define CAM1_FRAME_SIZE PAGE_ALIGN(CAM1_WIDTH * CAM1_HEIGHT * CAM1_PIXEL_SIZE)
#define CAM1_AVI_RAM_SIZE PAGE_ALIGN(CAM1_FRAME_SIZE * CAM1_N_BUFFERS)

static unsigned long rnb6_dashcam_cam1_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static u64 rnb6_dashcam_cam1_dma_mask = DMA_BIT_MASK(32);

static struct pinctrl_map rnb6_dashcam_cam1_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINCFG(P7_CAM_1_CLK, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA00),
	P7_INIT_PINCFG(P7_CAM_1_DATA00, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA01),
	P7_INIT_PINCFG(P7_CAM_1_DATA01, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA02),
	P7_INIT_PINCFG(P7_CAM_1_DATA02, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA03),
	P7_INIT_PINCFG(P7_CAM_1_DATA03, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA04),
	P7_INIT_PINCFG(P7_CAM_1_DATA04, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA05),
	P7_INIT_PINCFG(P7_CAM_1_DATA05, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA06),
	P7_INIT_PINCFG(P7_CAM_1_DATA06, rnb6_dashcam_cam1_pinconf),
	P7_INIT_PINMAP(P7_CAM_1_DATA07),
	P7_INIT_PINCFG(P7_CAM_1_DATA07, rnb6_dashcam_cam1_pinconf),
};

static struct avicam_dummy_info rnb6_dashcam_cam1_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = CAM1_WIDTH,
		.height	    = CAM1_HEIGHT,
	},
};

static struct avicam_platform_data rnb6_dashcam_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &rnb6_dashcam_cam1_dummy_driver_info,
};


static struct platform_device rnb6_dashcam_cam1_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &rnb6_dashcam_cam1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static struct sii5293_platform_data sii_dashcam_pdata = {
	.version        = SII5293_PDATA_STRUCT_VERSION,
	.i2c_bus        = 2,
	.gpio_rst       = P7_GPIO_NR(MHL_CAM_NRST),
	.gpio_irq       = P7_GPIO_NR(MHL_CAM_INT1),
	.output_format  = 8, /* YCbCr 4:2:2, embedded sync output */
	.input_dev_rap  = 1,
	.input_dev_rcp  = 1,
	.input_dev_ucp  = 1,
};

static struct platform_device sii_dashcam_pdev = {
	.name           = "sii-5293",
	.id             = 1,
};


/***************
   CAM 5 : CVBS
 ***************/
#define CAM5_HEIGHT	576
#define CAM5_WIDTH	720
#define CAM5_N_BUFFERS	4

#define CAM5_PIXEL_SIZE 2
#define CAM5_FRAME_SIZE PAGE_ALIGN(CAM5_WIDTH*CAM5_HEIGHT*CAM5_PIXEL_SIZE)
#define CAM5_AVI_RAM_SIZE PAGE_ALIGN(CAM5_FRAME_SIZE*CAM5_N_BUFFERS)

static unsigned long rnb6_cvbs_cam5_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_cvbs_cam5_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINCFG(P7_CAM_5_CLK, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINCFG(P7_CAM_5_DATA00, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINCFG(P7_CAM_5_DATA01, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINCFG(P7_CAM_5_DATA02, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINCFG(P7_CAM_5_DATA03, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINCFG(P7_CAM_5_DATA04, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINCFG(P7_CAM_5_DATA05, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINCFG(P7_CAM_5_DATA06, rnb6_cvbs_cam5_pinconf),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
	P7_INIT_PINCFG(P7_CAM_5_DATA07, rnb6_cvbs_cam5_pinconf)
};

static struct avicam_dummy_info rnb6_cvbs_cam5_dummy_driver_info = {
	.dev_id = 5,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = CAM5_WIDTH,
		.height	    = CAM5_HEIGHT,
	},
};

static int cam5_power_on(void)
{
	gpio_set_value_cansleep(CVBS_RSTN_A, 1);
	gpio_set_value_cansleep(CVBS_PDN_A, 0);
	return 0;
}

static int cam5_power_off(void)
{
	gpio_set_value_cansleep(CVBS_RSTN_A, 0);
	gpio_set_value_cansleep(CVBS_PDN_A, 1);

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
	.differential_input = TW9990_DIFFERENTIAL_ENABLED
};

static struct i2c_board_info cam5_tw9990_device[] = {
	{
		I2C_BOARD_INFO("tw9990", 0x45),
		.platform_data = &cam5_tw9990_platform_data,
	},
};

static struct avicam_subdevs rnb6_cvbs_cam5_subdevs[] = {
	{
		.board_info = &cam5_tw9990_device[0],
		.i2c_adapter_id = 2,
	},
	{ NULL, 0, },
};

static struct avicam_platform_data rnb6_cvbs_cam5_pdata = {
	.cam_cap	   = AVI_CAP_CAM_5,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 8,
	.subdevs	   = rnb6_cvbs_cam5_subdevs,
	.dummy_driver_info = &rnb6_cvbs_cam5_dummy_driver_info,
};

static u64 rnb6_cvbs_cam5_dma_mask = DMA_BIT_MASK(32);
static struct platform_device rnb6_cvbs_cam5_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &rnb6_cvbs_cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};


void __init rnb6_avi_init_cameras(void)
{
	p7_gpio_interrupt_register(CVBS_INT_B);
	cam5_tw9990_device[0].irq = gpio_to_irq(CVBS_INT_B);
	p7_init_avicam(&rnb6_mhl_cam0_dev,
	               &rnb6_mhl_cam0_pdata,
	               rnb6_mhl_cam0_pins_cfg,
	               ARRAY_SIZE(rnb6_mhl_cam0_pins_cfg));

	p7_init_avicam(&rnb6_dashcam_cam1_dev,
	               &rnb6_dashcam_cam1_pdata,
	               rnb6_dashcam_cam1_pins_cfg,
	               ARRAY_SIZE(rnb6_dashcam_cam1_pins_cfg));

	p7_init_avicam(&rnb6_cvbs_cam5_dev,
	               &rnb6_cvbs_cam5_pdata,
	               rnb6_cvbs_cam5_pins_cfg,
	               ARRAY_SIZE(rnb6_cvbs_cam5_pins_cfg));
}

void __init rnb6_avi_init(void)
{
	unsigned long fb_start = 0;
	unsigned long fb_size = 0;

	p7_init_avi();

	/* Camera inits */
	rnb6_avi_init_cameras();

	/* LCD init */
	gpio_request_one(P7_GPIO_NR(HDMI_RESET), GPIOF_OUT_INIT_HIGH, "hdmi-rst");
	parrot_init_i2c_slave(1,
				rnb6_siihdmi_board_info,
				"sii HDMI 9024A tx",
				P7_I2C_IRQ);

	p7_init_avifb(&rnb6_avifb0_dev, &rnb6_avifb0_pdata,
			rnb6_avifb0_pins_cfg,
			ARRAY_SIZE(rnb6_avifb0_pins_cfg));

	p7_init_avifb(&rnb6_avifb1_dev, &rnb6_avifb1_pdata,
			rnb6_avifb1_pins_cfg,
			ARRAY_SIZE(rnb6_avifb1_pins_cfg));

	fb_start = rnb6_avifb0_pdata.overlays[0].dma_memory.start;
	fb_size = rnb6_avifb0_pdata.overlays[0].dma_memory.end - fb_start + 1;
	p7_init_gpu_fb(fb_start, fb_size, 4);

	p7_init_avi_voc(0, &rnb6_avi_voc0);
	p7_init_avi_voc(1, &rnb6_avi_voc1);
	p7_init_avi_voc(2, &rnb6_avi_voc2);
}

/******
 * PWM
 ******/
#include "p7_pwm.h"
#include "backlight.h"
#define	PWM_14 14

static struct p7pwm_conf rnb6_conf_backlight_lcd = {
	.period_precision = 5,
	.duty_precision = 1,
	.mode = P7PWM_MODE_NORMAL,
};

static struct p7pwm_conf rnb6_conf_fan = {
	.period_precision = 5,
	.duty_precision = 1,
	.mode = P7PWM_MODE_NORMAL,
};


static struct p7pwm_pdata rnb6_pwm_pdata = {
	.conf = {
		[8]	 = &rnb6_conf_fan,
		[PWM_14] = &rnb6_conf_backlight_lcd,

	}
};
static unsigned long rnb6_pwm_pinconfig[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_pwm_pin_cfg[] __initdata= {
	P7_INIT_PINMAP(P7_PWM_08),
	P7_INIT_PINCFG(P7_PWM_08, rnb6_pwm_pinconfig),
	P7_INIT_PINMAP(P7_PWM_14),
	P7_INIT_PINCFG(P7_PWM_14, rnb6_pwm_pinconfig),
};

static void __init rnb6_init_pwm_backlight(void)
{
	p7_init_p7pwm(&rnb6_pwm_pdata, rnb6_pwm_pin_cfg, ARRAY_SIZE(rnb6_pwm_pin_cfg));
	p7_init_bkl(0, NULL,
		    P7_PWM_NR(PWM_14),          /* PWM Id */
		    -1,                          /* reset */
		    P7_GPIO_NR(EN_TFT),         /* TFT Enable */
		    NSEC_PER_SEC / 200,         /* 200 Hz */
	            256,                        /* 100% duty cycle / brightness */
	            3);                         /* 1% duty cycle / brightness */
}

/**************************
 * Parallel -> MIPI bridge
 **************************/

int rnb6_tc358764_set_power(int enable) {
	gpio_set_value_cansleep(NRST_MIPI, !!enable);
	gpio_set_value_cansleep(NRST_TFT,  !!enable);

	/* Not sure if it's entirely necessary but better safe than sorry */
	msleep(10);

	return 0;
}

struct tc358764_pdata rnb6_tc358764_pdata = {
	.set_power = rnb6_tc358764_set_power,
};

static struct i2c_board_info rnb6_tc358764_board_info = {
	I2C_BOARD_INFO("tc358764", 0x07),
	.platform_data = &rnb6_tc358764_pdata,
};

/***************
 * OCTOPUS Tuner
 ***************/
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>

static unsigned long rnb6_spim_single_pinconf[] = {
       P7CTL_SMT_CFG(OFF)	| /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN)	| /* no pull up/down unable */
       P7CTL_SLR_CFG(3)		| /* Slew rate 3 */
       P7CTL_DRV_CFG(3),	  /* Drive strength 3 (reg=0xf)*/
};

static struct pinctrl_map rnb6_spim_single_pins_cfg[8] __initdata = {
	P7_INIT_PINMAP(P7_SPI_01a),
	P7_INIT_PINCFG(P7_SPI_01a, rnb6_spim_single_pinconf),
	P7_INIT_PINMAP(P7_SPI_02a),
	P7_INIT_PINCFG(P7_SPI_02a, rnb6_spim_single_pinconf),
	P7_INIT_PINMAP(P7_SPI_03a),
	P7_INIT_PINCFG(P7_SPI_03a, rnb6_spim_single_pinconf),
	P7_INIT_PINMAP(P7_SPI_00a),
	P7_INIT_PINCFG(P7_SPI_00a, rnb6_spim_single_pinconf),
};

static struct p7spi_swb rnb6_spim_swb[5] =
{
	{
		.pad       = 1 /* P7_SPI_01a */,
		.direction = P7_SWB_DIR_OUT,
		.function  = P7_SWB_SPI_CLK,
	},
	{
		.pad       = 3 /* P7_SPI_03a */,
		.direction = P7_SWB_DIR_IN,
		.function  = P7_SWB_SPI_DATA0,
	},
	{
		.pad       = 2 /* P7_SPI_02a */,
		.direction = P7_SWB_DIR_OUT,
		.function  = P7_SWB_SPI_DATA0,
	},
	{
		.pad       = -1,
	},
	{
		.pad       = -1,
	}
};

static struct p7spi_ctrl_data rnb6_spim_cdata = {
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

static struct spi_board_info rnb6_spim_dev = {
	.modalias           = "spidev",
	.platform_data      = NULL,
	.controller_data    = &rnb6_spim_cdata,
	.irq                = -1,
	.max_speed_hz       = 40000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};

/* MPEG/TS */
#include <media/p7-mpegts.h>
#include "mpegts.h"

struct pinctrl_map rnb6_mpegts_tuner_pins_cfg[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_08),
	P7_INIT_PINCFG(P7_SPI_08, rnb6_spim_single_pinconf),
	P7_INIT_PINMAP(P7_SPI_09),
	P7_INIT_PINCFG(P7_SPI_09, rnb6_spim_single_pinconf),
};

struct p7spi_swb rnb6_mpegts_tuner_swb[] __initdata = {
	P7SPI_INIT_SWB( 8, P7_SWB_DIR_IN, P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB( 9, P7_SWB_DIR_IN, P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

static struct p7mpg_plat_data rnb6_mpegts_tuner_pdata __initdata = {
	.swb = rnb6_mpegts_tuner_swb,
	.fifo_wcnt = 16,
	.thres_wcnt = 8,
};


/******
 * CAN
 ******/
#include "c_can.h"
static unsigned long rnb6_can_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN)  | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map rnb6_can_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAN1_TXb),
	P7_INIT_PINCFG(P7_CAN1_TXb, rnb6_can_pinconf),
	P7_INIT_PINMAP(P7_CAN1_RXb),
	P7_INIT_PINCFG(P7_CAN1_RXb, rnb6_can_pinconf),
};


static struct platform_device virtual_wakeup_button = {
	.name           = "virtual_wakeup_button",
};

static void __init rnb6_init_mach(void)
{
	int loop, size = 0;
	struct gpio_setting *rnb6_gpios_hwxx = NULL;

	p7_init_mach();
	p7_init_gpio(NULL, 0);

	/* UART init */
	p7brd_init_uart(3,0); /* Uart Debug */
	p7brd_init_uart(0,0); /* UC */
	p7brd_init_uart(4,0); /* K LINE */

	/* I2C init */
	/* P7MU, MHL IN (Sil9293CNUC), CODEC (NAU8822L), IPOD chips*/
	if (!system_rev)
		p7brd_init_i2cm(0, 200);
	else
		p7brd_init_i2cm(0, 100);

	/* TouchScreen, RGB to MIPI (TC358768AXBG)
	   HDMI OUT (Sil9024AYBT), USB HUB (USB82514) */
	p7brd_init_i2cm(1, 100);

	/* TUNER Lithio (TEF6686), VIDEO CVBS,
	   HDMI MHL IN EDID (MC24C02-RMC6TG), MHL IN CAM (Sil9293CNUC) */
	if (!system_rev)
		p7brd_init_i2cm(2, 100);
	else
		p7brd_init_i2cm(2, 400);

	/* p7mu init */
	p7_gpio_interrupt_register(rnb6_p7mu_pdata.gpio);
	p7_init_p7mu(0,
			&rnb6_p7mu_pdata,
			rnb6_p7mu_pins,
			ARRAY_SIZE(rnb6_p7mu_pins));

	/* P7MU/ADC */
	p7_init_temperature();
	platform_device_register(&fc7100_temp_device);


	/* USB init */
	if (parrot_force_usb_device)
		p7brd_init_usb(0, P7_GPIO_NR(EN_5V_USB0), CI_UDC_DR_DUAL_DEVICE);
	else
		p7brd_init_usb(0, P7_GPIO_NR(EN_5V_USB0), CI_UDC_DR_DUAL_HOST);

	/* Configure vbus fault input */
	p7_config_pin(P7_GPIO_NR(VBUS_0_FAULT), P7CTL_PUD_CFG(HIGHZ));
	p7brd_export_gpio(P7_GPIO_NR(VBUS_0_FAULT),
			  GPIOF_DIR_IN,
			  "vbus0-fault");

	/* hub rst is driven by hub i2c driver */
	p7brd_init_hcd(1, -1);

	gpio_request_one(P7_GPIO_NR(USB_HUB_RST_N), GPIOF_OUT_INIT_LOW, "USB HUB RST");

	parrot_init_i2c_slave(1, &rnb6_smsc_82512_board_info[0],
					"smsc 82512", P7_I2C_NOIRQ);

	/* GPIOs */
	/* Common */
	for (loop = 0; loop < ARRAY_SIZE(rnb6_gpios); loop++) {

		if( rnb6_gpios[loop].interrupt )
			p7_gpio_interrupt_register(rnb6_gpios[loop].gpio);

		p7brd_export_gpio(rnb6_gpios[loop].gpio,
					rnb6_gpios[loop].default_value,
					rnb6_gpios[loop].name);

		if( rnb6_gpios[loop].bidir ){
			gpio_unexport(rnb6_gpios[loop].gpio);
			BUG_ON(gpio_export(rnb6_gpios[loop].gpio, 1));
		}
	}

	/* HW dependant */
	switch (system_rev) {
	case 0x0:
			rnb6_gpios_hwxx = rnb6_gpios_hw00;
			size = ARRAY_SIZE(rnb6_gpios_hw00);
		break;

	case 0x1:
	default:
			rnb6_gpios_hwxx = rnb6_gpios_hw01;
			size = ARRAY_SIZE(rnb6_gpios_hw01);
		break;
	};

	for (loop = 0; loop < size; loop++) {

		if (rnb6_gpios_hwxx[loop].interrupt)
			p7_gpio_interrupt_register(rnb6_gpios_hwxx[loop].gpio);

		p7brd_export_gpio(rnb6_gpios_hwxx[loop].gpio,
					rnb6_gpios_hwxx[loop].default_value,
					rnb6_gpios_hwxx[loop].name);

		if (rnb6_gpios_hwxx[loop].bidir) {
			gpio_unexport(rnb6_gpios_hwxx[loop].gpio);
			BUG_ON(gpio_export(rnb6_gpios_hwxx[loop].gpio, 1));
		}
	}


	/* AVI init */
	rnb6_avi_init();

	rnb6_init_pwm_backlight();

	p7brd_export_gpio(NRST_MIPI, GPIOF_OUT_INIT_LOW, "mipi_bridge_nrst");
	p7brd_export_gpio(NRST_TFT, GPIOF_OUT_INIT_LOW, "tft_controller_nrst");

	parrot_init_i2c_slave(1,
			      &rnb6_tc358764_board_info,
			      "tc358764",
			      P7_I2C_NOIRQ);

	/* Wifi / Bluetooth */
	init_wl18xx(&rnb6_wl18xx_res, NULL, 0);
	if (system_rev) /* HW01 and HW02 */
		p7_config_pin(P7_GPIO_NR(21), P7CTL_PUD_CFG(UP));

	/* Ethernet */
	if (system_rev) {/* HW01 and HW02 */
		p7_config_pin(P7_GPIO_NR(ETH_MII_TXER), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(ETH_MII_RXER), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(ETH_MII_COL), P7CTL_PUD_CFG(HIGHZ));

		p7_config_pin(P7_GPIO_NR(ETH_MII_RXC), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(ETH_MII_RXD_0), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(ETH_MII_RXD_1), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(ETH_MII_RXD_2), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(ETH_MII_RXD_3), P7CTL_PUD_CFG(HIGHZ));
	}

	p7_init_ether(PHY_IFACE_MII, P7_GPIO_NR(ETH_NRST), P7CTL_DRV_CFG(5));

	/* eMMC init */
	//TODO : not clear, it seems that SD_1_CLK need a pull down resistance
	p7brd_init_sdhci(1, &rnb6_sdhci_emmc_pdata,
					NULL, NULL, NULL, NULL, 0);
	/* Tuner */
	if (rnb6_tuner_device == RNB6_TUNER_OCTOPUS) {
		/* octopus */
		p7_gpio_interrupt_register(P7_GPIO_NR(RDS_TUNER_OCTO_INT));
		p7brd_export_gpio(P7_GPIO_NR(RDS_TUNER_OCTO_INT),
					GPIOF_DIR_IN,
					"octopus-int");

		p7brd_export_gpio(P7_GPIO_NR(QSI_TUNER_OCTO_PWDOWN),
			GPIOF_OUT_INIT_HIGH, "octopus-pwr");

		rnb6_spim_swb[3].pad       = 0 /* P7_SPI_00a */,
		rnb6_spim_swb[3].direction = P7_SWB_DIR_OUT,
		rnb6_spim_swb[3].function  = P7_SWB_SPI_SS,

		p7_init_mpegts(0, &rnb6_mpegts_tuner_pdata,
			rnb6_mpegts_tuner_pins_cfg,
			ARRAY_SIZE(rnb6_mpegts_tuner_pins_cfg));

		size = ARRAY_SIZE(rnb6_spim_single_pins_cfg);

		/* Audio init */
		p7_init_aai(rnb6_aai_pins, ARRAY_SIZE(rnb6_aai_pins), &rnb6_aai_pdata);

	} else if (rnb6_tuner_device == RNB6_TUNER_HIRSCHMANN) {

		for (loop = 0;
			loop < ARRAY_SIZE(rnb6_gpios_hirschmann);
			loop++) {

			if (rnb6_gpios_hirschmann[loop].interrupt)
				p7_gpio_interrupt_register(rnb6_gpios_hirschmann[loop].gpio);

			p7brd_export_gpio(rnb6_gpios_hirschmann[loop].gpio,
					rnb6_gpios_hirschmann[loop].default_value,
					rnb6_gpios_hirschmann[loop].name);

			if (rnb6_gpios_hirschmann[loop].bidir) {
				gpio_unexport(rnb6_gpios_hirschmann[loop].gpio);
				BUG_ON(gpio_export(rnb6_gpios_hirschmann[loop].gpio, 1));
			}
		}

		size = 6;

		/* Audio init */
		p7_init_aai(rnb6_hirschmann_aai_pins,
			    ARRAY_SIZE(rnb6_hirschmann_aai_pins),
			    &rnb6_hirschmann_aai_pdata);
	}

	p7_init_spim_slave(0, &rnb6_spim_dev);
	p7_init_spim(0, rnb6_spim_single_pins_cfg, size, rnb6_spim_swb);

	/* Audio init */
	parrot_init_i2c_slave(0, &rnb6_wau8822_board_info,
					"wau8822", P7_I2C_NOIRQ);
	p7_init_dev(&rnb6_asoc_dev, NULL, NULL, 0);

	/* CAN */
	p7_init_c_can(1, rnb6_can_pins, ARRAY_SIZE(rnb6_can_pins));

	/* Miscelllaneous */
	p7_init_vdec();
	p7_init_venc();
	p7_gpio_interrupt_register(P7_GPIO_NR(MHL_INT)); /* MHL in IT */
	p7_gpio_interrupt_register(P7_GPIO_NR(MHL_CAM_INT1)); /* MHL in Dashcam IT */

	if (system_rev) { /* HW01 and HW02 */
		p7_config_pin(P7_GPIO_NR(HDMI_INT), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(MHL_CAM_INT1), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(CVBS_INT_B), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(MHL_INT), P7CTL_PUD_CFG(HIGHZ));
		p7_config_pin(P7_GPIO_NR(MHL_5V_FAULT), P7CTL_PUD_CFG(UP));
	}

	/* Ipod */
	p7brd_export_i2c_hw_infos(0, 0x10, "2C", "ipod");

	/* virtual wakeup button needed for android resume */
	platform_device_register(&virtual_wakeup_button);
	p7_init_dev(&sii_mhl_pdev, &sii_mhl_pdata, NULL, 0);
	p7_init_dev(&sii_dashcam_pdev, &sii_dashcam_pdata, NULL, 0);
}

static void __init rnb6_reserve_mem(void)
{
	p7_reserve_avifbmem(&rnb6_avifb0_dev,
		rnb6_avi_lcd0_overlays,
		ARRAY_SIZE(rnb6_avi_lcd0_overlays));

	p7_reserve_avifbmem(&rnb6_avifb1_dev,
		rnb6_avi_lcd1_overlays,
		ARRAY_SIZE(rnb6_avi_lcd1_overlays));

#define RNB6_VOC_SIZE (LCD1_HEIGHT * LCD1_WIDTH * LCD1_N_BUFFERS * LCD1_PIX_SIZE)
	p7_reserve_avi_voc_mem(0, RNB6_VOC_SIZE);
	p7_reserve_avi_voc_mem(1, RNB6_VOC_SIZE);
	p7_reserve_avi_voc_mem(2, RNB6_VOC_SIZE);

#define RNB6_HX280_SIZE (CONFIG_ARCH_PARROT7_RNB6_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(RNB6_HX280_SIZE);

#define RNB6_HX270_SIZE (CONFIG_ARCH_PARROT7_RNB6_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(RNB6_HX270_SIZE);

#define RNB6_MPGTS_SIZE (CONFIG_ARCH_PARROT7_RNB6_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, RNB6_MPGTS_SIZE);

	p7_reserve_avicammem(&rnb6_mhl_cam0_dev,     CAM0_AVI_RAM_SIZE);
	p7_reserve_avicammem(&rnb6_dashcam_cam1_dev, CAM1_AVI_RAM_SIZE);
	p7_reserve_avicammem(&rnb6_cvbs_cam5_dev,    CAM5_AVI_RAM_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_RNB6, "rnb6")
	.reserve        = &rnb6_reserve_mem,
	.init_machine   = &rnb6_init_mach,
P7_MACHINE_END
