/*
 *
 * Copyright (C) 2013 Parrot S.A.
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
#include "spi.h"
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>

#include "common.h"
#include "system.h"

#include "common.h"
#include "board-common.h"
#include "fc7100-module.h"
#include "lcd-monspecs.h"
#include "aai.h"
#include "avi.h"
#include <mach/ether.h>
#include "nand.h"
#include "usb.h"

#include <linux/i2c/pca953x.h>
#define P7_EXTERNAL_IRQ_BASE (NR_IRQS - 16 * 2)
/* Mezz HW01 & HW02 have two 16 IO expanders */
/*
 * Export gpios to userspace
 * This MUST be done after all gpio drivers / chips were probed and properly
 * initialized ! Otherwise we won't be able to request gpio usage...
 * gpio expander is done on subsys_initcall
 * p7mu and p7 on postcore
 */

static int soca_r12_ioexpand1_setup(struct i2c_client *client,
					   unsigned gpio, unsigned ngpio,
					   void *context)
{
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(10), GPIOF_OUT_INIT_LOW, "ipod-rst");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(9), GPIOF_OUT_INIT_LOW, "cam0-nrst");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(13), GPIOF_OUT_INIT_LOW, "cam1-nrst");
	return 0;
}

static struct pca953x_platform_data fc7100_pca953x_pdata[] = {
	{
		/* We put it on top of the "internal" GPIOs */
		.gpio_base = FC7100_IOEXPAND0_FIRST_GPIO,
		.irq_base  = P7_EXTERNAL_IRQ_BASE,
		.setup = soca_r12_ioexpand1_setup,
	},
};

static struct i2c_board_info __initdata fc7100_i2c_infos_ioexpands[] = {
	{
		I2C_BOARD_INFO("pca9535", 0x24),
		.platform_data = &fc7100_pca953x_pdata[0],
		.irq =P7_GPIO_NR(160),
	},
};

static void __init init_gpio_expanders(void)
{
		parrot_init_i2c_slave(1, &fc7100_i2c_infos_ioexpands[0],
				      "IO-Expander", P7_I2C_IRQ);
}

static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",
	"voice-8k",
	"pcm0-out",

	/* Input Channels */
	"music-in-stereo0",
	"mic1-8k",
	"mic1-16k",
	"pcm0-in",
	"loopback-8k",
	"loopback-16k",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set soca_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{MUS_SYNC_DAC(0)},

	/*Don't remove*/
	{-1, 0, 0, 0},
};

static struct aai_pad_t soca_aai_pads[] = {
	{AAI_SIG_MCLK,		 12, PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME, 11, PAD_OUT},
	{AAI_SIG_DAC_BIT_CLOCK,	 10, PAD_OUT},

	/* codec2 wm8594 */
	{AAI_SIG_IN_MIC0,	 19, PAD_IN },
	{AAI_SIG_OUT_DAC0,	 22, PAD_OUT},

	/* codec1 wau8822 */
	{AAI_SIG_IN_MIC1,	 18, PAD_IN },
	{AAI_SIG_OUT_DAC1,	 20, PAD_OUT},

	/* PCM1 */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	{-1,			 -1,       0}
};

static struct aai_platform_data soca_aai_pdata = {
	.pad         = soca_aai_pads,
	.aai_conf    = soca_aai_conf_set,
	.device_list = aai_dev_list,
};

static unsigned long soca_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long soca_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg = 3) */
};

static unsigned long soca_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static unsigned long soca_aai_pcm_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shmitt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg = 15) */
};

static struct pinctrl_map soca_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, soca_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, soca_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, soca_aai_pcm_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, soca_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, soca_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, soca_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, soca_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, soca_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, soca_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, soca_aai_pinconf),
};

#include <sound/soc.h>
static struct i2c_board_info __initdata fc7100_wau8822_board_info = {
	I2C_BOARD_INFO("wau8822", 0x1a),
	.irq = -1,
};

static struct i2c_board_info __initdata fc7100_wm8594_board_info = {
	I2C_BOARD_INFO("wm8594", 0x1b),
	.irq = -1,
};


static struct snd_soc_dai_link parrot_fc7100_dai[] = {
	{
		.name           = "wau8822",
		.codec_name     = "wau8822.1-001a",
		.codec_dai_name = "wau8822-hifi",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.dai_fmt        = FC7100_DAIFMT,
	},
	{
		.name           = "wm8594",
		.codec_name     = "wm8594.2-001b",
		.codec_dai_name = "wm8594-hifi",
		.cpu_dai_name   = "snd-soc-dummy-dai",
		.dai_fmt        = FC7100_DAIFMT,
	},
};

/* FC7100 ASoC device */
static struct platform_device fc7100_asoc_dev[] = {
	{
		.name           = "parrot-fc7100-audio",
		.id             = 0,
		.dev		= {
			.platform_data = &parrot_fc7100_dai[0],
		},
	},
	{
		.name           = "parrot-fc7100-audio",
		.id             = 1,
		.dev		= {
			.platform_data = &parrot_fc7100_dai[1],
		},
	},
};

static void __init init_board(void)
{
	unsigned int mod_settings = 0;

	fc7100_init_module(mod_settings);
	p7brd_init_i2cm(1, 100);
	p7brd_init_i2cm(2, 50);

	p7brd_init_uart(0,1);

	init_gpio_expanders();

	// power supply enable (USB PHY, LCD, etc..)
	p7brd_export_gpio(206, GPIOF_OUT_INIT_HIGH, "board-power");

	// codec mute
	p7brd_export_gpio(202, GPIOF_OUT_INIT_HIGH, "mute");
	// RST tuner/codec/amplifier, active low
	p7brd_export_gpio(205, GPIOF_OUT_INIT_LOW, "tct");
	// Power down codec (active low)
	p7brd_export_gpio(201, GPIOF_OUT_INIT_HIGH, "codec pow");

	// Standby ampli (active = 0)
	p7brd_export_gpio(203, GPIOF_OUT_INIT_LOW, "ampli");

	p7brd_export_gpio(204, GPIOF_OUT_INIT_HIGH, "hub");

	if (parrot_force_usb_device || 1)
		p7brd_init_udc(0, FC7100_IOEXPAND0_GPIO_NR(4));
	else
		p7brd_init_hcd(0, FC7100_IOEXPAND0_GPIO_NR(4));

	p7brd_init_hcd(1, FC7100_IOEXPAND0_GPIO_NR(6));

	p7_init_aai(soca_aai_pins, ARRAY_SIZE(soca_aai_pins), &soca_aai_pdata);
	/* CODEC ... */
	parrot_init_i2c_slave(1, &fc7100_wau8822_board_info,
			      "wau8822", P7_I2C_NOIRQ);
	parrot_init_i2c_slave(2, &fc7100_wm8594_board_info,
			      "wm8594", P7_I2C_NOIRQ);
	p7_init_dev(&fc7100_asoc_dev[0], NULL, NULL, 0);
	p7_init_dev(&fc7100_asoc_dev[1], NULL, NULL, 0);

	/* stuff to check with rnb5tb ... */
	gpio_set_value(205, 1);
	gpio_set_value(202, 0);

	p7_init_ether(PHY_IFACE_RGMII,
		      FC7100_IOEXPAND0_GPIO_NR(11),
		      P7CTL_DRV_CFG(5));

	/* Export unconfigured devices informations */
	p7brd_export_i2c_hw_infos(2, 0x11, "2B", "ipod");
}

static void __init soca_reserve_mem(void)
{
	p7_reserve_nand_mem();
	p7_reserve_usb_mem(0);
	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_SOCA, "socadense")
	.reserve        = &soca_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END

