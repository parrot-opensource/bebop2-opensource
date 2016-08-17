/**
 * linux/arch/arm/mach-parrot7/sfx1-board.c - Parrot7 sfx1 platform
 *                                                implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Florent Bayendrian <florent.bayendrian@parrot.com>
 * date:    08-Oct-2012
 *
 * This file is released under the GPL
 * Copyright (C) 2012 Parrot S.A.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <spi/p7-spi.h>
#include "board-common.h"
#include "common.h"
#include "pinctrl.h"
#include "venc.h"
#include "spi.h"
#include "usb.h"
#include "avi.h"
#include "p7_temperature.h"
#include "gpu.h"

#define SFX1_BRD_NAME "sfx1"

/*****************
 * GPIOs handling
 *****************/

#include "gpio.h"

static unsigned const sfx1_irq_gpios[] = {
	10,   /* Wi-Fi wl18xx IT */
	73,   /* P7MU interrupt source */
	132,  /* SDCard 1 Card Detect */
	91,   /* mpu6050 Data Ready signal */
};

/******************
 * SPI controllers
 ******************/

#include <spi/p7-spim.h>

static struct pinctrl_map sfx1_s6spi_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_04),  /* CLK */
	P7_INIT_PINMAP(P7_SPI_05)   /* DIN */
};

/*
 * We'll need only MOSI and CLK signals, together with 2 GPIOs to program
 * Spartan6 FPGA. It is connected to Master 2.
 */
static struct p7spi_swb const sfx1_s6spi_swb[] = {
	P7SPI_INIT_SWB(4,   P7_SWB_DIR_OUT,     P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(5,   P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data sfx1_s6spi_cdata = {
	.half_duplex        = true,
	.read               = false,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.fifo_wcnt          = 16,
	.thres_wcnt         = 12,
	.tsetup_ss_ns       = 1,
	.thold_ss_ns        = 1,
	.toffclk_ns         = 1,
	.toffspi_ns         = 1,
	.tcapture_delay_ns  = 0,
};

static P7_DECLARE_SPIM_SLAVE(sfx1_s6spi_dev,
                             "spidev",
                             NULL,
                             &sfx1_s6spi_cdata,
                             10000000,
                             SPI_MODE_0);


/*****************************
 * P7MU Power Management Unit
 *****************************/
#include "p7mu.h"

#if defined(CONFIG_MFD_P7MU) || defined(CONFIG_MFD_P7MU_MODULE)
#include <mfd/p7mu.h>
#include <mach/gpio.h>

static struct pinctrl_map sfx1_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

static struct p7mu_plat_data sfx1_p7mu_pdata = {
	.gpio       = 73,   /* GPIO 73 is P7MU -> P7 interrupt source */
	.int_32k    = false,/* External 32kHz clock. */
	.int_32m    = false,/* External 48mHz clock. */
};

#endif  /* defined(CONFIG_MFD_P7MU) || defined(CONFIG_MFD_P7MU_MODULE) */

/******
 * PWM
 ******/

#include "p7_pwm.h"

#if defined(CONFIG_PWM_PARROT7) || \
    defined(CONFIG_PWM_PARROT7_MODULE)

static struct p7pwm_conf sfx1_conf_clock_camera = {
	.period_precision = 5,          /* precision for MT9V117 */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};

static struct p7pwm_conf sfx1_conf_clock_Hcamera = {
	.period_precision = 5,          /* precision for MT9F001 */
	.duty_precision = 0,            /* Not used in clock mode */
	.mode = P7PWM_MODE_CLOCK,
};
static struct p7pwm_pdata sfx1_pwm_pdata = {
	.conf = {
		[0] = &sfx1_conf_clock_Hcamera,
		[9] = &sfx1_conf_clock_camera,
	}
};

static struct pinctrl_map sfx1_pwm_pins[] __initdata = {
	P7_INIT_PINMAP(P7_PWM_00),
	P7_INIT_PINMAP(P7_PWM_09),
	P7_INIT_PINMAP(P7_PWM_13),
};

#endif /*CONFIG_PWM_PARROT7 || CONFIG_PWM_PARROT7_MODULE*/

/***********************
 * SDHCI hosts handling
 ***********************/

#include "sdhci.h"

#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>
#include "gbc.h"

/* wl18xx wifi chip */
static struct acs3_plat_data sfx1_sdhci0_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,
	.cd_gpio    = -1,
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,             /* wl18xx chip is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE,          /* sdio bus voltage is fixed in hardware */
};

static struct acs3_regulator_gpios sfx1_sdhci1_regulator_gpios = {
	.bus_1v8_gpio = P7_GPIO_NR(133),                /* GPIO 133 is 1.8V switch
	                                                 * command */
	.bus_1v8_active_low = 0,                        /* 1V8 swicth GPIO is active high */
};

static struct acs3_plat_data sfx1_sdhci1_pdata = {
	.led_gpio   = -1,                               /* No activity led GPIO */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = P7_GPIO_NR(132),                  /* GPIO 132 is card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

/**********************
 * TI wl18xx wlan
 **********************/

#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>

#define GPIO_WIFI_PMENA		12
#define GPIO_WIFI_IRQ		10

static struct regulator_consumer_supply acs3_sdhci0_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "acs3-sdhci.0");

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data acs3_sdhci0_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &acs3_sdhci0_vmmc2_supply,
};

static struct fixed_voltage_config acs3_sdhci0_vwlan = {
	.supply_name            = "vwl18xx",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = GPIO_WIFI_PMENA,
	.startup_delay          = 70000, /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &acs3_sdhci0_vmmc2,
};

static struct platform_device acs3_sdhci0_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data  = &acs3_sdhci0_vwlan,
	},
};

#include <linux/wl12xx.h>

static struct wl12xx_platform_data sfx1_wl18xx_pdata = {
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = WL12XX_TCXOCLOCK_26,
};


static void __init sfx1_init_wl18xx(void)
{
	int gpio;
	int err;

	gpio = GPIO_WIFI_IRQ;

	err = gpio_request(gpio, "wl18xx IT");
	if (err) {
		pr_err(SFX1_BRD_NAME
			": couldn't request GPIO %d "
			"for wl18xx [%d]\n",
			gpio, err);
	}

	sfx1_wl18xx_pdata.irq = gpio_to_irq(gpio);
	if (sfx1_wl18xx_pdata.irq < 0) {
		pr_err(SFX1_BRD_NAME
			":wl18xx:couldn't find an IRQ for gpio %d [%d]\n",
			gpio, sfx1_wl18xx_pdata.irq);
		gpio_free(gpio);
	}

	err = wl12xx_set_platform_data(&sfx1_wl18xx_pdata);
	if (err)
		pr_err(SFX1_BRD_NAME
			": error setting wl12xx data: %d\n", err);
}

/******
 * Cameras
 ******/

#define P7_CAMH_AVI_RAM_SIZE (1280 * 2500 * 4 * 4)

/* horizontal cam (cam0) */
static struct pinctrl_map sfx1_camh_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINMAP(P7_CAM_0_HS),
	P7_INIT_PINMAP(P7_CAM_0_VS),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINMAP(P7_CAM_0_DATA16),
	P7_INIT_PINMAP(P7_CAM_0_DATA17),
	P7_INIT_PINMAP(P7_CAM_0_DATA18),
	P7_INIT_PINMAP(P7_CAM_0_DATA19),
	P7_INIT_PINMAP(P7_CAM_0_DATA20),
	P7_INIT_PINMAP(P7_CAM_0_DATA21),
	P7_INIT_PINMAP(P7_CAM_0_DATA22),
	P7_INIT_PINMAP(P7_CAM_0_DATA23),
};


static struct avicam_dummy_info sfx1_camh_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code = V4L2_MBUS_FMT_YUYV8_1X16,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field = V4L2_FIELD_NONE,
		.width = 4640,
		.height = 3320,
	},
};

static struct avicam_platform_data sfx1_camh_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 0,
		.pad_select = 1,
		.ivs        = 1,
		.ihs        = 1,
	},
	.bus_width	   = 16,
	.subdevs	   = NULL,
	.dummy_driver_info = &sfx1_camh_dummy_driver_info,
};

static u64 sfx1_camh_dma_mask = DMA_BIT_MASK(32);
static struct platform_device sfx1_camh_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &sfx1_camh_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/* vertical cam (cam2) */

/* Those are the pins for the "ROAD CAM" connector on the RNB6 devboard
 * (connectors J800 and J801). The VS and HS signals are connected on the same
 * pins as the LCD0's DEN and HS, so in order to use both at the same time we
 * can't use the camera's external sync lines and use ITU656 embedded AV codes
 * instead. */
static struct pinctrl_map sfx1_cam2_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_2_CLK),
	P7_INIT_PINMAP(P7_CAM_2_DATA00),
	P7_INIT_PINMAP(P7_CAM_2_DATA01),
	P7_INIT_PINMAP(P7_CAM_2_DATA02),
	P7_INIT_PINMAP(P7_CAM_2_DATA03),
	P7_INIT_PINMAP(P7_CAM_2_DATA04),
	P7_INIT_PINMAP(P7_CAM_2_DATA05),
	P7_INIT_PINMAP(P7_CAM_2_DATA06),
	P7_INIT_PINMAP(P7_CAM_2_DATA07),
};

#define CAM2_WIDTH 640
#define CAM2_HEIGHT 480
#define CAM2_PIXEL_SIZE 4
#define CAM2_N_BUFFERS 8
#define P7_CAM2_AVI_RAM_SIZE PAGE_ALIGN(CAM2_WIDTH * CAM2_HEIGHT * \
					CAM2_PIXEL_SIZE * CAM2_N_BUFFERS)

static struct avicam_dummy_info sfx1_cam2_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field = V4L2_FIELD_NONE,
		.width = CAM2_WIDTH,
		.height = CAM2_HEIGHT,
	},
};

static struct avicam_platform_data sfx1_cam2_pdata = {
	.cam_cap	   = AVI_CAP_CAM_2,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &sfx1_cam2_dummy_driver_info,
};

static u64 sfx1_cam2_dma_mask = DMA_BIT_MASK(32);
static struct platform_device sfx1_cam2_dev = {
	.name           = "avicam",
	.id             = 2,
	.dev            = {
		.dma_mask           = &sfx1_cam2_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static void __init sfx1_reserve_mem(void)
{
#define SFX1_HX280_SIZE (CONFIG_ARCH_PARROT7_SFX1_HX280_SIZE << 20)
	p7_reserve_vencmem(SFX1_HX280_SIZE);

	p7_reserve_avicammem(&sfx1_camh_dev, P7_CAMH_AVI_RAM_SIZE);
	p7_reserve_avicammem(&sfx1_cam2_dev, P7_CAM2_AVI_RAM_SIZE);

	p7_reserve_dmamem();
}

#if defined(CONFIG_P7MU_ADC) || defined(CONFIG_P7MU_ADC_MODULE)
#define SFX1_SPI_SLAVE_P7MU    3

static struct pinctrl_map sfx1_spi_slave_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_13b), /* SS */
	P7_INIT_PINMAP(P7_SPI_12b), /* CLK */
	P7_INIT_PINMAP(P7_SPI_14b), /* MOSI */
};

static struct p7spi_swb const sfx1_spi_slave_p7mu_swb[] = {
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
};

static struct p7spis_ctrl_data sfx1_spi_slave_p7mu_cdata = {
	.common = {
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
		.tcapture_delay_ns  = 0,
	},
	.circ_buf_period = 128,
	.periods         = 9,
};

static P7_DECLARE_SPIS_MASTER(sfx1_spi_master_p7mu_info,
		"p7mu-adc",
		NULL,
		&sfx1_spi_slave_p7mu_cdata,
		10 * 1000 * 1000,
		SPI_MODE_0|SPI_LSB_FIRST);


static void __init sfx1_init_spi_p7mu(void)
{
	p7_init_spis(SFX1_SPI_SLAVE_P7MU,
			sfx1_spi_slave_p7mu_pins,
			ARRAY_SIZE(sfx1_spi_slave_p7mu_pins),
			sfx1_spi_slave_p7mu_swb);

	if (p7_init_spis_master(SFX1_SPI_SLAVE_P7MU,
				&sfx1_spi_master_p7mu_info))
		pr_err(SFX1_BRD_NAME ": failed to initialize SPI slave.\n");
}
#else
#define sfx1_init_spi_p7mu()
#endif



static void __init sfx1_init_mach(void)
{
	p7_init_mach();

	p7_init_gpio(sfx1_irq_gpios,
	             ARRAY_SIZE(sfx1_irq_gpios));

	/* I2CM0: P7MU, Vertical Camera & FPGA */
	p7brd_init_i2cm(0, 400);
	p7_init_p7mu(0,
	             &sfx1_p7mu_pdata,
	             sfx1_p7mu_pins,
	             ARRAY_SIZE(sfx1_p7mu_pins));

	p7brd_init_uart(0,1);
	p7brd_init_uart(1,1);

	p7brd_init_nand(1);

	/* I2CM1: MPU6050(gyro), AKM8963 (magneto) */
	p7brd_init_i2cm(1, 400);
	/* I2CM2: BLC & pressure sensor */
	p7brd_init_i2cm(2, 400);

	p7_init_p7pwm(&sfx1_pwm_pdata,
	              sfx1_pwm_pins,
		      ARRAY_SIZE(sfx1_pwm_pins));

	p7brd_init_sdhci(0, &sfx1_sdhci0_pdata, &acs3_sdhci0_wlan_regulator,
			 NULL, NULL, NULL, 0);
	p7brd_init_sdhci(1, &sfx1_sdhci1_pdata, NULL, NULL,
			 &sfx1_sdhci1_regulator_gpios, NULL, 0);

	p7_init_spim(2,
	             sfx1_s6spi_pins,
	             ARRAY_SIZE(sfx1_s6spi_pins),
	             sfx1_s6spi_swb);
	p7_init_spim_slave(2, &sfx1_s6spi_dev);

	sfx1_init_spi_p7mu();
        p7_init_temperature();

	p7_init_avi();

	p7_init_avicam(&sfx1_cam2_dev,
	               &sfx1_cam2_pdata,
	               sfx1_cam2_pins,
	               ARRAY_SIZE(sfx1_cam2_pins));

	p7_init_avicam(&sfx1_camh_dev,
	               &sfx1_camh_pdata,
	               sfx1_camh_pins,
	               ARRAY_SIZE(sfx1_camh_pins));
	p7_init_gpu_fb(0, 0, 4);

	sfx1_init_wl18xx();

	/* EHCI controller 0 is by default host and can be
	 * forced to device by cmdline option passed by installer
	 */
	if (parrot_force_usb_device)
		p7brd_init_udc(0, -1);
	else
		p7brd_init_hcd(0, -1);

	p7_init_venc();
}

#ifdef CONFIG_SERIAL_PARROTX_CONSOLE
#include <linux/console.h>

static int __init sfx1_init_cons(void)
{
	if (machine_desc->nr == MACH_TYPE_PARROT_SFX1)
		return add_preferred_console("ttyPA", 0, "115200n8");

	return 0;
}
console_initcall(sfx1_init_cons);

#endif

P7_MACHINE_START(PARROT_SFX1, "Sfx1 board")
	.reserve        = &sfx1_reserve_mem,
	.init_machine   = &sfx1_init_mach,
P7_MACHINE_END
