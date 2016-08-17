/**
 * linux/arch/arm/mach-parrot7/board-galilei.c - Galilei board implementation
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Romuald Despres <romuald.despres@parrot.com>
 * date:    Dec 2015
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <video/avi.h>
#include <video/avifb.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/leds_pwm.h>
#include <linux/delay.h>
#include <spi/p7-spi.h>
#include <linux/init.h>
#include <linux/ramoops.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/pwm.h>
#include "common.h"
#include "system.h"
#include "board.h"
#include "board-common.h"
#include "pinctrl.h"
#include "avi.h"
#include "spi.h"
#include "venc.h"
#include "usb.h"
#include "gpio.h"
#include "p7mu.h"
#include "p7_temperature.h"
#include "p7_pwm.h"

#include "board-drone-common.h"

#include <media/video/avicam.h>


#define GALILEI_BRD_NAME   "galilei"

/*****************************
 * PWM
 *****************************/

static struct p7pwm_conf galileo2_conf_cam_mclk = {
	/* Precision for mt9f002 */
	.period_precision = 5,
	/* Not used in clock mode */
	.duty_precision = 0,
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf galilei_conf_pwm_leds = {
	.period_precision = 5,
	.duty_precision   = 5,
	.mode             = P7PWM_MODE_NORMAL,
};

static unsigned long galilei_pwm_pinconfig[] = {
	P7CTL_DRV_CFG(0),      /* Drive strength 0 (reg=1) */
};

static struct pinctrl_map galileo2db_pwm_pins[] __initdata = {
	/* Galileo 2 mclk */
	P7_INIT_PINMAP(P7_PWM_05),
	/* Toshiba bridge mclk */
	P7_INIT_PINMAP(P7_PWM_11),
	/* Green led */
	P7_INIT_PINMAP(P7_PWM_04),
	P7_INIT_PINCFG(P7_PWM_04, galilei_pwm_pinconfig),
	/* Red led */
	P7_INIT_PINMAP(P7_PWM_12),
	P7_INIT_PINCFG(P7_PWM_12, galilei_pwm_pinconfig),
	/* Blue led */
	P7_INIT_PINMAP(P7_PWM_14),
	P7_INIT_PINCFG(P7_PWM_14, galilei_pwm_pinconfig),
};

static struct p7pwm_pdata galileo2db_pwm_pdata = {
	.conf = {
		[5]   = &galileo2_conf_cam_mclk,
		[11]  = &galileo2_conf_cam_mclk,
		[4]   = &galilei_conf_pwm_leds,     /* GREEN LED */
		[12]  = &galilei_conf_pwm_leds,     /* RED   LED */
		[14]  = &galilei_conf_pwm_leds,     /* BLUE  LED */
        }
};

/*****************************
 * Galileo2 cam
 * Bridge tc358746a
 *****************************/

#include <media/galileo2.h>
#include <media/tc358746a.h>


static struct pinctrl_map galileo2_avicam_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_HS),
	P7_INIT_PINMAP(P7_CAM_0_VS),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
};

/* Configure a PWM to output a clock signal with a frequency given in
 * megaherz */
static struct pwm_device *galileo2_pwm_clock(int pwm,
					     const char *label,
					     unsigned freq_khz)
{
	struct pwm_device *pwm_dev;
	unsigned period_ns = (1000000 + freq_khz / 2) / freq_khz;
	int ret;

	pwm_dev = pwm_request(pwm, label);
	if (IS_ERR(pwm_dev)) {
		return pwm_dev;
	}

	ret = pwm_config(pwm_dev, 0, period_ns);
	if (ret) {
		goto free;
	}

	ret = pwm_enable(pwm_dev);
	if (ret) {
		goto free;
	}

	return pwm_dev;

 free:
	pwm_free(pwm_dev);

	return ERR_PTR(ret);
}

static void galileo2_pwm_free(struct pwm_device *pwm_dev)
{
	if (!IS_ERR_OR_NULL(pwm_dev)) {
		pwm_disable(pwm_dev);
		pwm_free(pwm_dev);
	}
}


#define GALILEO2_TC358746A_MCLK_KHZ 9200

static struct pwm_device *galileo2_tc358746a_pwm = NULL;

static int galileo2_tc358746a_set_power(int on)
{
	if (on) {
		galileo2_tc358746a_pwm =
			galileo2_pwm_clock(P7_PWM_NR(11),
					   "bridge mclk",
					   GALILEO2_TC358746A_MCLK_KHZ);
	} else {
		galileo2_pwm_free(galileo2_tc358746a_pwm);
	}

	if (IS_ERR(galileo2_tc358746a_pwm)) {
		return PTR_ERR(galileo2_tc358746a_pwm);
	}

	gpio_set_value(P7_GPIO_NR(132), !!on);

	return 0;
}

#define GALILEO2_CAM_MCLK_KHZ 10000
static struct pwm_device *galileo2_cam_pwm = NULL;

static int galileo2_cam_set_power(int on)
{
	if (on) {
		galileo2_cam_pwm =
			galileo2_pwm_clock(P7_PWM_NR(5),
					   "galileo2 mclk",
					   GALILEO2_CAM_MCLK_KHZ);
	} else {
		galileo2_pwm_free(galileo2_cam_pwm);
	}

	if (IS_ERR(galileo2_cam_pwm)) {
		return PTR_ERR(galileo2_cam_pwm);
	}

	msleep(10);

	return 0;
}

struct galileo2_platform_data galileo2_cam_pdata = {
	.set_power = galileo2_cam_set_power,
	.refclk = GALILEO2_CAM_MCLK_KHZ * 1000,
	.lanes = 4,
};

static struct i2c_board_info galileo2_cam_i2c_device = {
	I2C_BOARD_INFO("galileo2", 0x10),
	.platform_data = &galileo2_cam_pdata,
};

static struct avicam_subdevs galileo2_cam_subdevs[] = {
	{
		.board_info     = &galileo2_cam_i2c_device,
		.i2c_adapter_id = 2,
		.subdevs        = NULL,
	},
	{ NULL, 0 },
};

static struct tc358746a_platform_data galileo2_tc358746a_subdev_pdata = {
	.set_power            = &galileo2_tc358746a_set_power,
	.refclk               = GALILEO2_TC358746A_MCLK_KHZ * 1000,
	/* Do we need that? */
	/*.force_subdev_pixcode = V4L2_MBUS_FMT_SBGGR10_1X10,*/
	.lanes                = 4,
	.calibration_delay_ms = 500,
	.phytimdly            = 39,
};

static struct i2c_board_info galileo2_tc358746a_i2c_device = {
	I2C_BOARD_INFO("tc358746a", 0xe),
	.platform_data = &galileo2_tc358746a_subdev_pdata,
};

static struct avicam_subdevs galileo2_tc358746a_subdevs[] = {
	{
		.board_info     = &galileo2_tc358746a_i2c_device,
		.i2c_adapter_id = 1,
		.subdevs = galileo2_cam_subdevs,
	},
	{ NULL, 0 },
};

/* Galileo2 sensor follows the SMIA specification, hence the first 4 lines
 * contains its registers values. So we crop it directly here.
 */
static void galileo2_get_timings(struct avi_capture_timings  *t,
				 struct avi_cam_measure_regs *m)
{
	t->ht.hactive_on  = 0;
	t->ht.hactive_off = m->hsync_off;
	t->vt.vactive_on  = 5;
	t->vt.vactive_off = m->vsync_voff + 5;
}


static struct avicam_platform_data galileo2_avicam_pdata = {
	.cam_cap	    = AVI_CAP_CAM_0,
	.interface          = {
		.itu656	    = 0,
		.pad_select = 0,
		.ivs        = 0,
		.ihs        = 0,
		.ipc        = 0,
		.psync_en   = 1,
		.psync_rf   = 1,
		.ror_lsb    = 0,
	},
	.bus_width	    = 10,
	.subdevs	    = galileo2_tc358746a_subdevs,
	.measure_to_timings = &galileo2_get_timings,
};

#define GALILEO2_CAM_RAM_SIZE PAGE_ALIGN(7716 * 5364 * 2 * 4)

static u64 galileo2_avicam_dma_mask = DMA_BIT_MASK(32);

static struct platform_device galileo2_avicam_dev = {
	.name = "avicam",
	.id   = 0,
	.dev  = {
		.dma_mask          = &galileo2_avicam_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32)
	}
};

/*****************************
 * Board revision
 *****************************/

static int __init galilei_board_get_rev(void)
{
	static int board_rev = -1;
	int gpios[] = {138, 139};
	int i;

	if (board_rev != -1)
		return board_rev;

	board_rev = 0;
	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		int err, val;
		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
			0,
			"galilei rev");
		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			board_rev |= (val << i);
			gpio_free(P7_GPIO_NR(gpios[i]));
		}
	}
	return board_rev;
}

/*****************************
 * P7MU Power Management Unit
 *****************************/

#include "p7mu.h"
#include <mfd/p7mu.h>
#include <mach/p7-adc.h>
#include <mach/gpio.h>
#include <spi/p7-spim.h>


static struct pinctrl_map galilei_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

static struct p7mu_plat_data galilei_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(72),   /* GPIO 72 is P7MU -> P7 interrupt source */
	.int_32k    = false,            /* External 32kHz clock. */
	.int_32m    = true,             /* No External 48mHz clock. */
};

/***********************
 * SDHCI hosts handling
 ***********************/

#include "sdhci.h"

#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>
#include "gbc.h"

static unsigned long galilei_sdhci_pins_config_clk[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
	P7CTL_SLR_CFG(2),
};

static unsigned long galilei_sdhci_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(3),
	P7CTL_SLR_CFG(2),
};

static struct pinctrl_map galilei_sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, galilei_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, galilei_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, galilei_sdhci_pins_config)
};

static struct acs3_plat_data galilei_sdhci0_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = 81,                               /* GPIO 81 is write protect status */
	.cd_gpio    = 82,                               /* GPIO 82 is card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

static struct acs3_plat_data galilei_sdhci1_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = P7_GPIO_NR(142),
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,             /* emmc is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE|MMC_CAP2_CACHE_CTRL,  /* bus voltage is fixed in hardware */
};

/***********************
 * LEDS
 ***********************/

#define GALILEI_PWM_LED_GREEN  4
#define GALILEI_PWM_LED_RED   12
#define GALILEI_PWM_LED_BLUE  14

static void galilei_configure_leds(void)
{
	struct pwm_device *pwm;
	pwm = pwm_request(GALILEI_PWM_LED_RED, "galilei BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 1000000, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(GALILEI_PWM_LED_GREEN, "galilei BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 50000, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}

	pwm = pwm_request(GALILEI_PWM_LED_BLUE, "galilei BSP");

	if (!IS_ERR(pwm)) {
		pwm_config(pwm, 0, 2000000);
		pwm_enable(pwm);
		pwm_free(pwm);
	}
}

static struct led_pwm pwm_leds[] = {
	{
		.name = "galilei:green",
		.default_trigger = "none",
		.pwm_id = GALILEI_PWM_LED_GREEN,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "galilei:blue",
		.default_trigger = "none",
		.pwm_id = GALILEI_PWM_LED_BLUE,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
	{
		.name = "galilei:red",
		.default_trigger = "none",
		.pwm_id = GALILEI_PWM_LED_RED,
		.active_low = 1,
		.max_brightness = 255,
		.pwm_period_ns = 100000,
	},
};

static struct led_pwm_platform_data pwm_data = {
	.num_leds = ARRAY_SIZE(pwm_leds),
	.leds = pwm_leds,
};

static struct platform_device galilei_leds_pwm = {
	.name = "leds_pwm",
	.id = -1,
	.dev = {
		.platform_data = &pwm_data,
	},
};

/***********************
 * Memory
 ***********************/

static void __init galilei_reserve_mem(void)
{
    drone_common_reserve_mem_ramoops();

#define GALILEI_HX280_SIZE (CONFIG_ARCH_PARROT7_GALILEI_HX280_SIZE * SZ_1M)

	p7_reserve_avicammem(&galileo2_avicam_dev,
			     GALILEO2_CAM_RAM_SIZE);

	p7_reserve_vencmem(GALILEI_HX280_SIZE);

	p7_reserve_usb_mem(0);

	p7_reserve_dmamem();
}

/***********************
 * Init
 ***********************/

static void __init galilei_init_mach(void)
{
	/* Initialize ramoops */
	drone_common_init_ramoops();

	p7_init_mach();

	p7_init_gpio(NULL, 0);

	pr_info("galilei rev %d\n", galilei_board_get_rev());

	/* debug uart */
	p7brd_init_uart(0,0);

	/* eBee autopilot uart */
	p7brd_init_uart(1,0);

	/*
	 * i2c-0: P7MU
	 */
	p7brd_init_i2cm(0, 400);
	/*
	 * i2c-1: tc358746a
	 */
	p7brd_init_i2cm(1, 400);
	/*
	 * i2c-2: galileo2 cam
	 */
	p7brd_init_i2cm(2, 400);

	/* p7mu */
	p7_gpio_interrupt_register(galilei_p7mu_pdata.gpio);
	p7_init_p7mu(0,
		     &galilei_p7mu_pdata,
		     galilei_p7mu_pins,
		     ARRAY_SIZE(galilei_p7mu_pins));

	p7_init_temperature();

	/* SD */
	p7_gpio_interrupt_register(galilei_sdhci0_pdata.cd_gpio);
	p7brd_init_sdhci(0, &galilei_sdhci0_pdata, NULL, NULL,
			 NULL, NULL, 0);

	/* emmc */
	p7brd_init_sdhci(1, &galilei_sdhci1_pdata, NULL, NULL, NULL,
			 galilei_sdhci1_pins, ARRAY_SIZE(galilei_sdhci1_pins));


	p7brd_init_udc(0, -1);

	p7_init_venc();

	/* Power on galileo 2 cam and tc358746a */
	gpio_request_one(P7_GPIO_NR(65),
			 GPIOF_OUT_INIT_HIGH, "Power on Galileo2 cam");

	gpio_request_one(P7_GPIO_NR(132),
			 GPIOF_OUT_INIT_LOW, "Power on tc358746a");

	/* galileo2 */
	p7_init_avicam(&galileo2_avicam_dev,
		       &galileo2_avicam_pdata,
		       galileo2_avicam_pins,
		       ARRAY_SIZE(galileo2_avicam_pins));

	p7_init_p7pwm(&galileo2db_pwm_pdata,
		      galileo2db_pwm_pins,
		      ARRAY_SIZE(galileo2db_pwm_pins));

	galilei_configure_leds();

	/* pwn leds */
	platform_device_register(&galilei_leds_pwm);

}

P7_MACHINE_START(PARROT_GALILEI, "galilei board")
  .reserve        = &galilei_reserve_mem,
  .init_machine   = &galilei_init_mach,
P7_MACHINE_END
