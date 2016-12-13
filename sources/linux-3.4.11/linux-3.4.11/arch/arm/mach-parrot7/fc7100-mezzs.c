#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <linux/persistent_ram.h>
#include <linux/ramoops.h>
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
#include "mpegts.h"
#include "uart.h"
#include "venc.h"
#include "vdec.h"
#include "nand.h"
#include "usb.h"

#include "system.h"
#include "board-common.h"
#include "fc7100-module.h"
#include "fc7100-mezzs.h"
#include "fc7100-mezzs2-avi.h"

#define FC7100_BRD_NAME "FC7100 mezz"

/**ram2ram*/

static struct avi_m2m_platform_data fc7100_avi_m2m_pdata[] = {
	{ .caps = AVI_CAP_PLANAR | AVI_CAP_SCAL | AVI_CAP_CONV },
	{ .caps = 0 },
};


/***** TODO : make it generic *******/


/****** */
static int mezz_rev = -1;
int fc7100_mezz_get_rev(void)
{
	if (mezz_rev == -1) {
		if (fc7100_module_get_rev() <= 3) {
			mezz_rev = fc7100_module_get_rev() - 1;
		}
		else {
			mezz_rev = fc7100_board_get_rev();
			if (mezz_rev == -1) {
				/* XXX hw03 and hw04 can't be detected.
				   add +1 when there is no more hw03
				 */
				mezz_rev = 4;
			}
		}
		pr_notice(FC7100_BRD_NAME" version %d\n", mezz_rev);
		BUG_ON(mezz_rev <= 1);
	}
	return mezz_rev;
}

/* sdcard 1 on workbench */
/* MMC1 on FC7100 R2 */
static struct acs3_plat_data fc7100_sdhci1_pdata = {
	.led_gpio   = -1,                               /* No activity led GPIO */
	.wp_gpio    = FC7100_IOEXPAND0_GPIO_NR(6),      /* GPIO 6 on IOEXPAND0 is
	                                                 * write protect status */
	.cd_gpio    = FC7100_IOEXPAND0_GPIO_NR(5),      /* GPIO 5  on IOEXPAND0 is card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

/* sdcard 2 on mezz */
//TODO make this a real regulator
static struct acs3_regulator_gpios fc7100_sdhci2_regulator_gpios = {
	.card_enable_gpio = 0,
	.card_enable_active_low = 0,			/* Card VDD is active high */
	.bus_1v8_gpio = 0,
	.bus_1v8_active_low = 1,                        /* 1V8 swicth GPIO is active low */
	.bus_switch_delay = 200,                     /* After the switch, a delay of 200ms is needed */
	.bus_switch_disabled = 1,                       /* The switch need to be done while disabled */
	.bus_enable_gpio = 0,
	.bus_enable_active_low = 0,                     /* Bus VDD is active high */
};

/* MMC2 on FC7100 R2 */
static struct acs3_plat_data fc7100_sdhci2_pdata = {
	.led_gpio   = -1,                               /* No activity led GPIO */
	.wp_gpio    = P7_GPIO_NR(12),                   /* GPIO 12 is write protect
	                                                 * status */
	.cd_gpio    = P7_GPIO_NR(13),                   /* GPIO 13 is card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

static struct acs3_mode_config fc7100_p7r3_sdhci2_sdr_65MHz = {
	.mode   = ACS3_UHS_SDR50 | ACS3_UHS_SDR104,
	.max_Hz = 65 * 1000 * 1000,
	.tdl1   = 0,
	.tdl2   = 1,
	.tuning_success_count = 10,
};

void __init fc7100_init_sd(unsigned int mod_settings)
{
	if (mod_settings & FC7100_MOD_SDCARD1) {
		pr_info(FC7100_BRD_NAME": Registering SDCARD1\n");
		p7_gpio_interrupt_register(fc7100_sdhci1_pdata.cd_gpio);
		p7brd_init_sdhci(1, &fc7100_sdhci1_pdata, NULL, NULL, NULL,
				 NULL, 0);
	}

	if (mod_settings & FC7100_MOD_SDCARD2) {
		pr_info(FC7100_BRD_NAME": Registering SDCARD2\n");
		/* forced to high in fc7100_mezz_r12_ioexpand1_setup */
		fc7100_sdhci2_regulator_gpios.card_enable_gpio = 0;
		/* not connected on hw03-04. Connected on hw05. */
		//fc7100_sdhci2_regulator_gpios.bus_enable_gpio = FC7100_IOEXPAND1_GPIO_NR(15);
		fc7100_sdhci2_regulator_gpios.bus_enable_gpio = 0;

		if (fc7100_mezz_get_rev() < 6)
			fc7100_sdhci2_regulator_gpios.bus_1v8_gpio = P7_GPIO_NR(161);
		else
			fc7100_sdhci2_regulator_gpios.bus_1v8_gpio = FC7100_IOEXPAND0_GPIO_NR(12);

		if (p7_chiprev() == P7_CHIPREV_R3) {
			fc7100_sdhci2_pdata.mode_config = &fc7100_p7r3_sdhci2_sdr_65MHz;
			fc7100_sdhci2_pdata.nb_config = 1;
		}
		p7_gpio_interrupt_register(fc7100_sdhci2_pdata.cd_gpio);
		p7brd_init_sdhci(2, &fc7100_sdhci2_pdata, NULL, NULL,
				 &fc7100_sdhci2_regulator_gpios, NULL, 0);
	}
}

/*************** IO expender ****************/

#define P7_EXTERNAL_IRQ_BASE (NR_IRQS - 16 * 2)

/*
 * Export gpios to userspace
 * This MUST be done after all gpio drivers / chips were probed and properly
 * initialized ! Otherwise we won't be able to request gpio usage...
 * gpio expander is done on subsys_initcall
 * p7mu and p7 on postcore
 */
static int fc7100_export_gpios(void)
{
	/* this is the ipod chip on workbench, there is also another chip on mezz ... */
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(11), GPIOF_OUT_INIT_LOW, "ipod-rst");
	/* workbench ipod chip is not connected anymore */
	//p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(0), GPIOF_OUT_INIT_LOW, "ipod2-rst");

	/* XXX allow direction change */
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(1), GPIOF_IN, "wb-gpio0");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(2), GPIOF_IN, "wb-gpio1");

	/* octopus */
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(2), GPIOF_OUT_INIT_HIGH, "octopus-pwr");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(3), GPIOF_OUT_INIT_HIGH, "octopus-rst");

	/* cameras */
	/* FC7100_IOEXPAND0_GPIO_NR(12) : cam1-irq */
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(13), GPIOF_OUT_INIT_LOW, "cam1-nrst");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(14), GPIOF_OUT_INIT_LOW, "cam1-pwdn");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(15), GPIOF_OUT_INIT_LOW, "cam1-5v_pwr_en");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(9), GPIOF_OUT_INIT_HIGH, "cam0-nrst");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(10), GPIOF_OUT_INIT_LOW, "cam0-pwdn");
	p7brd_export_gpio(FC7100_IOEXPAND0_GPIO_NR(11), GPIOF_OUT_INIT_HIGH, "cam0-5v_pwr_en");
	p7brd_export_gpio(FC7100_IOEXPAND1_GPIO_NR(12),  GPIOF_OUT_INIT_LOW, "cam5-nrst");
	p7brd_export_gpio(P7_GPIO_NR(154), GPIOF_IN, "cam0-irq");

	p7_gpio_interrupt_register(P7_GPIO_NR(154));
	return 0;
}

static int fc7100_mezz_r12_ioexpand1_setup(struct i2c_client *client,
					   unsigned gpio, unsigned ngpio,
					   void *context)
{
	union i2c_smbus_data data;
	char *screen_name=NULL;

	gpio_request_one(FC7100_IOEXPAND1_GPIO_NR(14), GPIOF_OUT_INIT_HIGH, "LCD-nrst");

	gpio_request_one(FC7100_IOEXPAND0_GPIO_NR(4), GPIOF_OUT_INIT_HIGH, "Touch-nrst");
	/* force sd2 power and ldo en */
	gpio_request_one(FC7100_IOEXPAND0_GPIO_NR(7), GPIOF_OUT_INIT_HIGH, "sd2 power");
	gpio_request_one(FC7100_IOEXPAND1_GPIO_NR(15), GPIOF_OUT_INIT_HIGH, "sd2 bus en");

	fc7100_export_gpios();

	/* Setup avi, now that chipsets are powered */
	/* If avi chip is pluged on mezz, force configuration */
	if(i2c_smbus_xfer(i2c_get_adapter(2), 0x3b, 0,
			  I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE_DATA, &data) >= 0) {
		pr_info("Detected sii hdmi chip %d-0x%02X.\n", 2, 0x3b);
		screen_name = "sii-hdmi";
	}

	if (screen_name != NULL)
		fc7100_mezz_avi_update(screen_name, 0);

	return 0;
}

static struct pca953x_platform_data fc7100_pca953x_pdata[] = {
	{
		/* We put it on top of the "internal" GPIOs */
		.gpio_base = FC7100_IOEXPAND0_FIRST_GPIO,
		.irq_base  = P7_EXTERNAL_IRQ_BASE,
	},
	{
		.gpio_base = FC7100_IOEXPAND1_FIRST_GPIO,
		.irq_base  = P7_EXTERNAL_IRQ_BASE + 16,
		.setup = fc7100_mezz_r12_ioexpand1_setup,
	},
};

static struct i2c_board_info __initdata fc7100_i2c_infos_ioexpands[] = {
	{
		I2C_BOARD_INFO("pca9535", 0x24),
		.platform_data = &fc7100_pca953x_pdata[0],
		.irq =P7_GPIO_NR(86),
	},
	{
		I2C_BOARD_INFO("pca9535", 0x27),
		.platform_data = &fc7100_pca953x_pdata[1],
		.irq =P7_GPIO_NR(87),
	}
};

static void __init init_gpio_expanders(unsigned int mod_settings)
{
	fc7100_i2c_infos_ioexpands[0].irq = P7_GPIO_NR(160);
	p7_config_pin(fc7100_i2c_infos_ioexpands[0].irq, P7CTL_PUD_CFG(HIGHZ));
	parrot_init_i2c_slave(1, &fc7100_i2c_infos_ioexpands[0],
			      "IO-Expander-0", P7_I2C_IRQ);
	if (fc7100_mezz_get_rev() <= 4) {
		/* XXX HW04 miss a pull-up on gpio 155, use 160
		   to fake irq */
		fc7100_i2c_infos_ioexpands[1].irq = P7_GPIO_NR(160);
		p7_config_pin(fc7100_i2c_infos_ioexpands[1].irq, P7CTL_PUD_CFG(HIGHZ));
		parrot_init_i2c_slave(1, &fc7100_i2c_infos_ioexpands[1],
				      "IO-Expander", P7_I2C_SHAREDIRQ);
	}
	else {
		fc7100_i2c_infos_ioexpands[1].irq = P7_GPIO_NR(155);
		p7_config_pin(fc7100_i2c_infos_ioexpands[1].irq, P7CTL_PUD_CFG(HIGHZ));
		parrot_init_i2c_slave(1, &fc7100_i2c_infos_ioexpands[1],
				      "IO-Expander-1", P7_I2C_IRQ);
	}
}

/************************
 * LVDS serialiser config
 ************************/

#include <i2c/ti_ub925_lvds.h>
static int update_screen(unsigned deser_addr, const struct ti_lvds_platform_data **altconf);

/* Extra command for deserializer : enable gpio0 as output high (touchscreen nreset) */
static struct dsr_i2c_cmd vgtt_7dis_hw00_cmd[] = {
	{ .reg = 0x21, .data = 0x1  }, /* Unreset ATMEL MXT  */
	{ .reg = 0x20, .data = 0x10 }, /* Unreset PWM LP8860 */
	{ .reg = 0,    .data  = 0   },
};

const struct ti_lvds_platform_data vgtt_7dis_conf = {
	.cmd = vgtt_7dis_hw00_cmd,
	.premap = {
		.slave_id = 0x4d,
		.slave_alias = 0x4c,
	},
	.nb_i2c_slave = 4,
	.clock_rising = 1,
};

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
	.deser_callback = update_screen,
};

static struct i2c_board_info __initdata fc7100_lvds_board_info = {
	I2C_BOARD_INFO("lvds", 0xc),
	.irq = -1, /* fill by code */
	.platform_data = &ti_lvs_pdata,
};

static int update_screen(unsigned deser_addr, const struct ti_lvds_platform_data **altconf)
{
	char *screen_name = NULL;

	if (deser_addr == 0x2e) {
		pr_info("Detected Volvo truck 2K31715 screen over lvds\n");
		screen_name = "2K31715";
		*altconf = &vgtt_7dis_conf;
	} else if (deser_addr == 0x3a) {
		pr_info("Detected kyocera screen over lvds\n");
		screen_name = "kyocera";
	}

	if (screen_name != NULL)
		fc7100_mezz_avi_update(screen_name, 0);

	return 0;
}

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


static struct i2c_board_info __initdata fc7100_antec_board_info = {
	I2C_BOARD_INFO("uc6511", 0x0a),
	.irq = -1, /* fill by code */
};

static void fc7100_init_touchscreen(void)
{
	fc7100_lvds_board_info.irq = P7_GPIO_NR(77);
	fc7100_antec_board_info.irq = P7_GPIO_NR(77);

	parrot_init_i2c_slave(2,
	                      &fc7100_lvds_board_info,
	                      "LVDS serializer",
			      P7_I2C_IRQ);
	/* Init shared gpio : antec ts, lvds and sii hdmi */
	parrot_init_i2c_slave(2,
	                      &fc7100_antec_board_info,
			      "Antec TS",
			      P7_I2C_SHAREDIRQ);
	parrot_init_i2c_slave(2,
	                      &fc7100_atmel_mxt_board_info,
	                      "Atmel maXTouch",
			      P7_I2C_NOIRQ);
}


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
		.irq = P7_GPIO_NR(77),
	},
};


/*************
 * Audio (AAI)
 *************/
static char *aai_dev_list[] = {
	/* Output channels */
	"music-out-stereo0",
	"music-out-stereo1",
	"voice-out-stereo",
	"pcm0-out",
	"pcm1-out",

	/* Input Channels */
	"music-in-stereo0",
	"music-in-stereo1",
	"music-in-stereo2",
	"mic0-8k",
	"mic0-16k",
	"mic1-8k",
	"mic1-16k",
	"pcm0-in",
	"pcm1-in",
	"loopback-8k",
	"loopback-16k",

	/* Don't remove */
	NULL,
};

static struct aai_conf_set fc7100_aai_conf_set[] = {
	/*
	 * This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{AAI_MASTER(0)},
	{AAI_MASTER(1)},
	{AAI_SLAVE(2)},
	{RESET_VOI_MUX(0)},
	{VOI_MUX(0)},

	/* Don't remove */
	{-1, 0, 0, 0},
};

static struct aai_pad_t fc7100_aai_pads[] = {
	{AAI_SIG_MCLK,		 12, PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME, 11, PAD_OUT},
	{AAI_SIG_DAC_BIT_CLOCK,	 10, PAD_OUT},
	/* codec 1 */
	{AAI_SIG_I2S0_IN,	 18, PAD_IN },
	{AAI_SIG_IN_MIC0,	 18, PAD_IN },
	{AAI_SIG_OUT_DAC0,	 20, PAD_OUT},
	/* codec 2 */
	{AAI_SIG_I2S1_IN,	 22, PAD_IN },
	{AAI_SIG_IN_MIC1,	 22, PAD_IN },
	{AAI_SIG_OUT_DAC1,	 19, PAD_OUT},
	/* hdmi in */
	{AAI_SIG_I2S2_IN,	 21, PAD_IN },
	{AAI_SIG_I2S2_FRAME,	 17, PAD_IN },
	/* PCM1 */
	{AAI_SIG_PCM1_OUT,	  0, PAD_OUT},
	{AAI_SIG_PCM1_IN,	  2, PAD_IN },
	{AAI_SIG_PCM1_FRAME,	  3, PAD_IN },

	{-1,			 -1,       0}
};

static unsigned long fc7100_aai_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long fc7100_aai_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long fc7100_aai_pcm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static unsigned long fc7100_aai_pcm_frame_pinconf[] = {
	P7CTL_SMT_CFG(ON)    | /* enable shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=15) */
};

static struct pinctrl_map fc7100_aai_pins[] __initdata = {
	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, fc7100_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, fc7100_aai_pcm_pinconf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, fc7100_aai_pcm_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_10),
	P7_INIT_PINCFG(P7_AAI_10, fc7100_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_11),
	P7_INIT_PINCFG(P7_AAI_11, fc7100_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, fc7100_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, fc7100_aai_frame_pinconf),
	P7_INIT_PINMAP(P7_AAI_18),
	P7_INIT_PINCFG(P7_AAI_18, fc7100_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_19),
	P7_INIT_PINCFG(P7_AAI_19, fc7100_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_20),
	P7_INIT_PINCFG(P7_AAI_20, fc7100_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_21),
	P7_INIT_PINCFG(P7_AAI_21, fc7100_aai_pinconf),
	P7_INIT_PINMAP(P7_AAI_22),
	P7_INIT_PINCFG(P7_AAI_22, fc7100_aai_pinconf),
};

static struct aai_platform_data fc7100_aai_pdata = {
	.pad         = fc7100_aai_pads,
	.aai_conf    = fc7100_aai_conf_set,
	.device_list = aai_dev_list,
};

/*****************
 * WAU8822
 *****************/

#include <sound/soc.h>
static struct i2c_board_info __initdata fc7100_wau8822_board_info = {
	I2C_BOARD_INFO("wau8822", 0x1a),
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
		.name           = "pillow_wau8822",
		.codec_name     = "wau8822.2-001a",
		.codec_dai_name = "wau8822-hifi",
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

static void __init fc7100_init_audio(void)
{
	p7_init_aai(fc7100_aai_pins,
		    ARRAY_SIZE(fc7100_aai_pins), &fc7100_aai_pdata);
	parrot_init_i2c_slave(1, &fc7100_wau8822_board_info,
			      "wau8822", P7_I2C_NOIRQ);
	parrot_init_i2c_slave(2, &fc7100_wau8822_board_info,
			      "wau8822", P7_I2C_NOIRQ);
	p7_init_dev(&fc7100_asoc_dev[0], NULL, NULL, 0);
	p7_init_dev(&fc7100_asoc_dev[1], NULL, NULL, 0);
}

static void __init fc7100_init_mpegts(void)
{
	fc7100_init_mpegts_single(0, 16, 17);
	fc7100_init_mpegts_single(1, 18, 19);
}


/******
 * PWM
 ******/

#include "p7_pwm.h"

static struct p7pwm_conf fc7100_conf_backlight_lcd = {
	.period_precision = 5,
	.duty_precision = 1,
	.mode = P7PWM_MODE_NORMAL,
};

static struct p7pwm_pdata fc7100_hw2_pwm_pdata = {
	.conf = {
	}
};
static unsigned long fc7100_pwd_pinconfig[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map fc7100_hw4_pwm_pins[] __initdata= {
	P7_INIT_PINMAP(P7_PWM_14),
	P7_INIT_PINCFG(P7_PWM_14, fc7100_pwd_pinconfig),
};

static void __init fc7100_init_backlight(void)
{
	int pwm_id, blk_en;
	/* TODO : make pwm alloc dynamic */
	pwm_id = 14;
	blk_en = FC7100_IOEXPAND0_GPIO_NR(3);

	fc7100_hw2_pwm_pdata.conf[pwm_id] = &fc7100_conf_backlight_lcd;
	p7_init_p7pwm(&fc7100_hw2_pwm_pdata, fc7100_hw4_pwm_pins, ARRAY_SIZE(fc7100_hw4_pwm_pins));
	p7_init_bkl(0, NULL,
		    P7_PWM_NR(pwm_id),
		    -1,
		    blk_en,
		    NSEC_PER_SEC / 200,    /* 200 Hz */
	            256,                   /* 100% duty cycle / brightness */
	            3);                    /* 1% duty cycle / brightness */
}


/******************************/
/* Ramoops and persistent ram */
/******************************/
/* reserved RAM used to store oops/panic logs */
#define RAMOOPS_SIZE  SZ_512K
#define RAMOOPS_RECORD_SIZE  SZ_128K

static struct ramoops_platform_data ramoops_data = {
        .mem_size               = RAMOOPS_SIZE,
        .record_size            = RAMOOPS_RECORD_SIZE, /* size of each dump done on oops/panic */
        .dump_oops              = 1,      /* set to 1 to dump oopses, 0 to only dump panics */
};

static struct platform_device ramoops_dev = {
        .name = "ramoops",
        .dev = {
                .platform_data = &ramoops_data,
        },
};

/*********************/
int __init fc7100_init_mezz(unsigned int mod_settings, const char *screen_name)
{
	int ret;
	p7brd_init_i2cm(1, 200);

	/* check we are on a mezz */
	if (i2c_smbus_xfer(i2c_get_adapter(1), 0x24, 0, I2C_SMBUS_WRITE, 0, I2C_SMBUS_QUICK, NULL) < 0) {
		pr_info("no gpio expander at 0x24\n");
		return -ENODEV;
	}
	if (i2c_smbus_xfer(i2c_get_adapter(1), 0x27, 0, I2C_SMBUS_WRITE, 0, I2C_SMBUS_QUICK, NULL) < 0) {
		pr_info("no gpio expander at 0x27\n");
		return -ENODEV;
	}

	/* Need CONFIG_RAMOOPS in kernel config */
	ret = platform_device_register(&ramoops_dev);
	if (ret) {
		printk(KERN_ERR "unable to register ramoops platform device\n");
		return ret;
	}
	p7brd_init_i2cm(2, 100);

	p7brd_init_uart(0,1);

	if (!(mod_settings & FC7100_MOD_SDCARD1)) {
		p7brd_init_uart(5,0);
		p7brd_init_uart(6,0);
		p7brd_init_uart(7,0);
	}

	p7_init_venc();
	p7_init_vdec();

	/* In all cases, CAM5 uses SD2 pins, CAM5 has priority */
	if (mod_settings & FC7100_MOD_CAM5) {
		if (mod_settings & FC7100_MOD_SDCARD2)
			pr_warning(FC7100_BRD_NAME": CAM5 selected, can't use SD2 so removing it\n");
		mod_settings &= ~FC7100_MOD_SDCARD2;
	}

	init_gpio_expanders(mod_settings);

	fc7100_init_sd(mod_settings);


	/* Init USB (module attributes) */
	/* USB0 is dual mode booting on device */
	p7brd_init_usb(0, FC7100_IOEXPAND1_GPIO_NR(7), CI_UDC_DR_DUAL_DEVICE);

	/* USB1 is host only */
	p7brd_init_hcd(1, FC7100_IOEXPAND1_GPIO_NR(9));

	p7_init_ether(PHY_IFACE_RGMII,
		      FC7100_IOEXPAND1_GPIO_NR(13),
		      P7CTL_DRV_CFG(5));

	if (mod_settings & FC7100_MEZZ_MPEGTS) {
		fc7100_init_spim_single(0, 11, 10, 8, 9);
		fc7100_init_mpegts();
	}

	fc7100_init_touchscreen();

	fc7100_init_audio();

	if (screen_name)
		fc7100_mezz_avi_init(screen_name, 0, mod_settings);

	fc7100_init_backlight();

	p7_init_avi_m2m(fc7100_avi_m2m_pdata);

	/* Note : The irq is same than for lvds and antec ts *
	 * wich are previously registered in fc7100_init_touchscreen */
	parrot_init_i2c_slave(2,
			      fc7100_siihdmi_board_info,
			      "sii HDMI 9024A tx",
			      P7_I2C_SHAREDIRQ);
 	return 0;
}



/********************************
 * Fc7100 initialization functions
 ********************************/

void __init fc7100_mezz_reserve_mem(void)
{
	struct membank *bank = &meminfo.bank[0];

	ramoops_data.mem_address = bank->start + bank->size-RAMOOPS_SIZE;
	p7_reserve_devmem(&ramoops_dev,
			  (dma_addr_t *)&ramoops_data.mem_address,
			  (size_t *)&ramoops_data.mem_size);

	/* TODO make these alloc conditionnal */
	fc7100_mezz_reserve_mem_lcd();

#ifdef CONFIG_CAM_AVI
	fc7100_mezz_reserve_mem_for_camera();
#endif

#define FC7100_HX280_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(FC7100_HX280_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);

#define FC7100_MPGTS_SIZE (CONFIG_ARCH_PARROT7_FC7100_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, FC7100_MPGTS_SIZE);
	p7_reserve_mpegtsmem(1, FC7100_MPGTS_SIZE);

#ifdef CONFIG_M2M_AVI
#define FC7100_AVI_M2M_SIZE (1920 * 1080 * 2 * 4 * 4)
	p7_reserve_avi_m2m_mem(FC7100_AVI_M2M_SIZE);
#endif
	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}
