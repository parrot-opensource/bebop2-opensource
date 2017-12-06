/**
 * linux/arch/arm/mach-parrot7/board-drone-common.c - Parrot7 based boards
 *                                                    drone common interface
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Alexandre Dilly <alexandre.dilly@parrot.com>
 * date:    7-May-2015
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
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


/* GPIO support */
#include <gpio/p7-gpio.h>
#include "gpio.h"
/* P7MU support */
#include <mach/p7-adc.h>
#include <mfd/p7mu.h>
#include "p7mu.h"
/* PWM support */
#include "p7_pwm.h"
/* SDHCI support */
#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>
#include "sdhci.h"
#include "gbc.h"
/* Cameras support */
#include <media/video/avicam.h>
#include <parrot/avicam_dummy_dev.h>
/* Sysfs support */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#include "board-drone-common.h"

/*****************************
 * P7MU Power Management Unit
 *****************************/

#define SPI_SLAVE_P7MU		3

static struct pinctrl_map p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU), /* P7 -> P7MU reset request */
};

static struct p7_adc_chan p7_adc_p7mu_channels[] = {
	{
		.type	 = P7MUADC_IIO_RING_BUFFER,
		.channel = 2,
		.freq	 = 160000,
		.samples = 16*1024,
	},
};

static struct p7_adc_chan_data p7_adc_p7mu_data = {
	.channels     = p7_adc_p7mu_channels,
	.num_channels = ARRAY_SIZE(p7_adc_p7mu_channels),
};

static struct p7mu_plat_data drone_p7mu_pdata = {
	.gpio	   = 73,
	.int_32k   = true,
	.int_32m   = false,
	.chan_data = &p7_adc_p7mu_data,
};

static struct pinctrl_map spi_slave_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_13b), /* SS */
	P7_INIT_PINMAP(P7_SPI_12b), /* CLK */
	P7_INIT_PINMAP(P7_SPI_14b), /* MOSI */
};

static struct p7spi_swb const spi_slave_p7mu_swb[] = {
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spis_ctrl_data spi_slave_p7mu_cdata = {
	.common = {
		.half_duplex	    = true,
		.read		    = true,
		.write		    = true,
		.xfer_mode	    = P7SPI_SINGLE_XFER,
		.fifo_wcnt	    = 16,
		.thres_wcnt	    = 8,
		.tsetup_ss_ns	    = 1,
		.thold_ss_ns	    = 1,
		.toffclk_ns	    = 1,
		.toffspi_ns	    = 1,
		.tcapture_delay_ns  = 0,
	},
	.circ_buf_period = 8192,
	.periods	 = 4,
};

static P7_DECLARE_SPIS_MASTER(spi_master_p7mu_info,
			      "p7mu-adc",
			      NULL,
			      &spi_slave_p7mu_cdata,
			      10 * 1000 * 1000,
			      SPI_MODE_0 | SPI_LSB_FIRST);

void __init drone_common_init_p7mu(int gpio_int, int check_availability)
{
	union i2c_smbus_data data;
	int p7mu_i2c_bus = 0;

	/* Set GPIO */
	if (gpio_is_valid(gpio_int))
		drone_p7mu_pdata.gpio = gpio_int;

	/* Check availability of P7MU bus 0 */
	if (check_availability &&
	    i2c_smbus_xfer(i2c_get_adapter(0), 0x31, 0, I2C_SMBUS_READ, 0,
			   I2C_SMBUS_BYTE_DATA, &data) < 0) {
		/* P7MU is on bus 1 */
		p7mu_i2c_bus = 1;
	}

	/* Init P7MU */
	p7_init_p7mu(p7mu_i2c_bus, &drone_p7mu_pdata, p7mu_pins,
		     ARRAY_SIZE(p7mu_pins));

	/* Init SPI slave for P7MU */
	p7_init_spis(SPI_SLAVE_P7MU, spi_slave_p7mu_pins,
		     ARRAY_SIZE(spi_slave_p7mu_pins), spi_slave_p7mu_swb);
	if (p7_init_spis_master(SPI_SLAVE_P7MU, &spi_master_p7mu_info))
		pr_err("failed to initialize SPI slave.\n");

	/* Init P7 temperature */
	p7_init_temperature();
}

/****************
 * SDHCI (eMMC) *
 ****************/

static struct acs3_plat_data sdhci1_pdata = {
	.led_gpio  = -1, /* No activity led GPIO */
	.wp_gpio   = -1, /* No write protect */
	.cd_gpio   = -1, /* GPIO ??? is card detect */
	.rst_gpio  = -1, /* No eMMC hardware reset */
	/* 3.3V ~ 3.0V card Vdd only */
	.brd_ocr   = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_29_30 |
		     MMC_VDD_30_31,
	.mmc_caps  = MMC_CAP_NONREMOVABLE,
	.mmc_caps2 = MMC_CAP2_BROKEN_VOLTAGE,
};

static unsigned long sdhci_pins_config_clk[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(5),
	P7CTL_SLR_CFG(2),
};

static unsigned long sdhci_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(4),
	P7CTL_SLR_CFG(2),
};

static struct pinctrl_map sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, sdhci_pins_config)
};

void __init drone_common_init_sdhci(struct pinctrl_map *pins, int size)
{
	/* Set pin mapping */
	if (pins == NULL) {
		/* Use default mapping */
		pins = sdhci1_pins;
		size = ARRAY_SIZE(sdhci1_pins);
	}

	/* Initialize SDHCI */
	p7brd_init_sdhci(1, &sdhci1_pdata, NULL, NULL, NULL, pins, size);
}

/******************************
 * Ultrasound slave SPI pulse *
 ******************************/

static struct pinctrl_map us_spi_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_15) /* Data only */
};

static struct p7spi_swb const us_spi_swb[] = {
	P7SPI_INIT_SWB(15, P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST
};

static struct p7spi_ctrl_data us_spi_cdata = {
	.read		   = false,
	.write		   = true,
	.xfer_mode	   = P7SPI_SINGLE_XFER,
	.fifo_wcnt	   = 16,
	.thres_wcnt	   = 12,
	.tsetup_ss_ns	   = 1,
	.thold_ss_ns	   = 1,
	/* No offset between each octet sent */
	.toffclk_ns	   = 0,
	.toffspi_ns	   = 1,
	.tcapture_delay_ns = 0,
};

static P7_DECLARE_SPIM_SLAVE(us_spi_dev,
			     "spidev",
			     NULL,
			     &us_spi_cdata,
			     10000000,
			     SPI_MODE_0);

void __init drone_common_init_us_tx(void)
{
	/* Init SPI for ultrasound */
	p7_init_spim(1, us_spi_pins, ARRAY_SIZE(us_spi_pins), us_spi_swb);
	p7_init_spim_slave(1, &us_spi_dev);
}

/*******************************
 * Horizontal camera (MT9F002) *
 *******************************/

#define CAMERA_H_MT9F002_I2C_INFO		I2C_BOARD_INFO("mt9f002", 0x10)
#define CAMERA_H_MT9F002_I2C_BUS		0

/* Horizontal camera (on CAM0) */
static struct pinctrl_map cam_h_mt9f002_pins[] __initdata = {
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
};

/* When driver is available */
#ifdef DRIVER_VIDEO_MT9F002

/* MT9F002 input clock cofniguration: 13MHz (~77ns) */
#define MT9F002_INPUT_FREQ_mHZ		13000000000
#define MT9F002_OUTPUT_FREQ_mHZ		96000000000
#define MT9F002_PWM_PERIOD_NS		77

/* Aptina MT9F002 Subdev */
#include <media/mt9f002.h>

/* PWM out and Enable out */
static struct pwm_device *cam_h_mt9f002_pwm_device;
static int cam_h_mt9f002_pwm;
static int cam_h_mt9f002_en;

static int cam_h_mt9f002_power_on(void)
{
	int ret = 0;

	/* No PWM output */
	if (!gpio_is_valid(cam_h_mt9f002_pwm))
		goto enable;

	/* Request PWM out */
	if (cam_h_mt9f002_pwm_device == NULL) {
		cam_h_mt9f002_pwm_device = pwm_request(cam_h_mt9f002_pwm,
						       "CAMERA_H_MCLK");
		if (IS_ERR(cam_h_mt9f002_pwm_device)) {
			ret = PTR_ERR(cam_h_mt9f002_pwm_device);
			goto err_alloc;
		}
	}

	/* Configure PWM */
	ret = pwm_config(cam_h_mt9f002_pwm_device, 0, MT9F002_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	/* Enable PWM */
	ret = pwm_enable(cam_h_mt9f002_pwm_device);
	if (ret)
		goto err_config;

enable:
	/* Enable camera */
	if (gpio_is_valid(cam_h_mt9f002_en))
		gpio_set_value_cansleep(P7_GPIO_NR(cam_h_mt9f002_en), 1);

	return 0;

err_config:
	pwm_free(cam_h_mt9f002_pwm_device);
	cam_h_mt9f002_pwm_device = NULL;
err_alloc:
	pr_warn("failed to set clock for mt9f002 chip\n");
	return ret;
}

static int cam_h_mt9f002_power_off(void)
{
	/* Disable camera */
	if (gpio_is_valid(cam_h_mt9f002_en))
		gpio_set_value_cansleep(P7_GPIO_NR(cam_h_mt9f002_en), 0);

	/* Disable PWM */
	if (cam_h_mt9f002_pwm_device != NULL) {
		pwm_disable(cam_h_mt9f002_pwm_device);
		pwm_free(cam_h_mt9f002_pwm_device);
	}

	return 0;
}

static int cam_h_mt9f002_set_power(int on)
{
	if (on == MT9F002_POWER_ON)
		return cam_h_mt9f002_power_on();
	else
		return cam_h_mt9f002_power_off();
}

static struct mt9f002_platform_data cam_h_mt9f002_platform_data = {
	.interface	     = MT9F002_Parallel,
	.pixel_depth	     = MT9F002_10bits,
	.number_of_lanes     = MT9F002_2lane,
	.ext_clk_freq_mhz    = MT9F002_INPUT_FREQ_mHZ,
	.output_clk_freq_mhz = MT9F002_OUTPUT_FREQ_mHZ,
	.set_power	     = &cam_h_mt9f002_set_power,
};

static struct i2c_board_info cam_h_mt9f002_i2c_devices[] = {
	{
		CAMERA_H_MT9F002_I2C_INFO,
		.platform_data = &cam_h_mt9f002_platform_data,
	}
};

static struct avicam_subdevs cam_h_mt9f002_subdevs[] = {
	{
		.board_info     = &cam_h_mt9f002_i2c_devices[0],
		.i2c_adapter_id = CAMERA_H_MT9F002_I2C_BUS,
	},
	{ NULL, 0 },
};
#endif /* #ifdef DRIVER_VIDEO_MT9F002 */

/* MT9F002 dummy camera */
static struct avicam_dummy_info cam_mt9f002_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.field  = V4L2_FIELD_NONE,
		.width  = 4640,
		.height = 3320,
	},
};

static struct avicam_platform_data cam_h_mt9f002_pdata = {
	.interface = {
		.itu656     = 0,
		.pad_select = 1,
		.ivs	    = 1,
		.ihs	    = 1,
		.ipc	    = 1,
		.psync_en   = 1,
	},
	.cam_cap	   = AVI_CAP_CAM_0,
	.enable_stats	   = 1,
	.bus_width	   = 16,
#ifdef DRIVER_VIDEO_MT9F002
	.subdevs	   = cam_h_mt9f002_subdevs,
#endif /* #ifdef DRIVER_VIDEO_MT9F002 */
	.dummy_driver_info = &cam_mt9f002_dummy_driver_info,
};

static u64 cam_h_mt9f002_dma_mask = DMA_BIT_MASK(32);
static struct platform_device cam_h_mt9f002_dev = {
	.name = "avicam",
	.id   = 0,
	.dev  = {
		.dma_mask	   = &cam_h_mt9f002_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

static struct i2c_board_info cam_h_mt9f002_info = {
	CAMERA_H_MT9F002_I2C_INFO
};

void __init drone_common_init_cam_h_mt9f002(int gpio_pwm, int gpio_en, union avi_cam_interface *cam_interface, struct pinctrl_map *cam_pins, size_t pin_cnt)
{
#ifdef DRIVER_VIDEO_MT9F002
	/* Set GPIOs */
	cam_h_mt9f002_pwm = gpio_pwm;
	cam_h_mt9f002_en = gpio_en;
#endif

	if (cam_interface)
		memcpy(&cam_h_mt9f002_pdata.interface, cam_interface, sizeof(*cam_interface));
	if (!cam_pins) {
		cam_pins = cam_h_mt9f002_pins;
		pin_cnt = ARRAY_SIZE(cam_h_mt9f002_pins);
	}
	/* Add MT9F002 horizontal camera */
	p7_init_avicam(&cam_h_mt9f002_dev, &cam_h_mt9f002_pdata,
		       cam_pins, pin_cnt);

	/* Init I2C camera device */
	p7brd_export_i2c_hw_infos(CAMERA_H_MT9F002_I2C_BUS,
				  cam_h_mt9f002_info.addr, "Video",
				  cam_h_mt9f002_info.type);

	/* Export PWDN GPIO */
	drone_common_export_gpio(gpio_en, GPIOF_OUT_INIT_LOW,
				 "CAMERA_H_PWDN");
}

/*******************************
 * Vertical camera (MT9V117) *
 *******************************/

#define CAMERA_V_MT9V117_I2C_INFO I2C_BOARD_INFO("mt9v117", 0x5D)
#define CAMERA_V_MT9V117_I2C_BUS		0

/* Vertical camera (on CAM2) */
static struct pinctrl_map cam_v_mt9v117_pins[] __initdata = {
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

#ifdef DRIVER_VIDEO_MT9V117

/* MT9V117 input clock cofniguration: 43.3MHz (~23ns) */
#define MT9V117_INPUT_FREQ_HZ		43333333
#define MT9V117_PWM_PERIOD_NS		23

/* Aptina MT9V117 Subdev */
#include <media/mt9v117.h>

/* PWM out and Enable out */
static struct pwm_device *cam_v_mt9v117_pwm_device;
static int cam_v_mt9v117_pwm;
static int cam_v_mt9v117_en;

static int cam_v_mt9v117_power_on(void)
{
	int ret = 0;

	/* No PWM output */
	if (!gpio_is_valid(cam_v_mt9v117_pwm))
		goto enable;

	/* Request PWM out */
	if (cam_v_mt9v117_pwm_device == NULL) {
		cam_v_mt9v117_pwm_device = pwm_request(cam_v_mt9v117_pwm,
						       "CAMERA_V_MCLK");
		if (IS_ERR(cam_v_mt9v117_pwm_device)) {
			ret = PTR_ERR(cam_v_mt9v117_pwm_device);
		goto err_alloc;
		}
	}

	/* Configure PWM */
	ret = pwm_config(cam_v_mt9v117_pwm_device, 0, MT9V117_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	/* Enable PWM */
	ret = pwm_enable(cam_v_mt9v117_pwm_device);
	if (ret)
		goto err_config;

enable:
	/* Enable camera */
	if (gpio_is_valid(cam_v_mt9v117_en))
		gpio_set_value_cansleep(P7_GPIO_NR(cam_v_mt9v117_en), 1);

	return 0;

err_config:
	pwm_free(cam_v_mt9v117_pwm_device);
	cam_v_mt9v117_pwm_device = NULL;
err_alloc:
	pr_warn("failed to set clock for mt9v117 chip\n");
	return ret;
}

static int cam_v_mt9v117_power_off(void)
{
	/* Disable camera */
	if (gpio_is_valid(cam_v_mt9v117_en))
		gpio_set_value_cansleep(P7_GPIO_NR(cam_v_mt9v117_en), 0);

	/* Disable PWM */
	if (cam_v_mt9v117_pwm_device != NULL)
		pwm_disable(cam_v_mt9v117_pwm_device);

	return 0;
}

static int cam_v_mt9v117_set_power(int on)
{
	if (on == MT9V117_POWER_ON)
		return cam_v_mt9v117_power_on();
	else
		return cam_v_mt9v117_power_off();
}

static struct mt9v117_platform_data cam_v_mt9v117_platform_data = {
	.ext_clk_freq_hz = MT9V117_INPUT_FREQ_HZ,
	.enable_bt656    = 1,
	.set_power	 = &cam_v_mt9v117_set_power,
};

static struct i2c_board_info cam_v_mt9v117_i2c_devices[] = {
	{
		CAMERA_V_MT9V117_I2C_INFO,
		.platform_data = &cam_v_mt9v117_platform_data,
	}
};

static struct avicam_subdevs cam_v_mt9v117_subdevs[] = {
	{
		.board_info     = &cam_v_mt9v117_i2c_devices[0],
		.i2c_adapter_id = CAMERA_V_MT9V117_I2C_BUS,
	},
	{ NULL, 0 },
};
#else
/* MT9V117 dummy camera */
static struct avicam_dummy_info cam_mt9v117_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field	    = V4L2_FIELD_NONE,
		.width	    = 640,
		.height	    = 480,
	},
};
#endif /* #ifdef DRIVER_VIDEO_MT9V117 */

static struct avicam_platform_data cam_v_mt9v117_pdata = {
	.interface = {
		.itu656	    = 1,
		.pad_select = 0,
		.ipc	    = 1,
	},
	.cam_cap	   = AVI_CAP_CAM_2,
	.bus_width	   = 8,
#ifdef DRIVER_VIDEO_MT9V117
	.subdevs	   = cam_v_mt9v117_subdevs,
#else
	.dummy_driver_info = &cam_mt9v117_dummy_driver_info,
#endif /* #ifdef DRIVER_VIDEO_MT9V117 */
	.vb2_cache_flags = VB2_CACHE_DMA_CONTIG,
};

static u64 cam_v_mt9v117_dma_mask = DMA_BIT_MASK(32);
static struct platform_device cam_v_mt9v117_dev = {
	.name = "avicam",
	.id   = 2,
	.dev  = {
		.dma_mask	   = &cam_v_mt9v117_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

static struct i2c_board_info cam_v_mt9v117_info = {
	CAMERA_V_MT9V117_I2C_INFO
};

void __init drone_common_init_cam_v_mt9v117(int gpio_pwm, int gpio_en)
{
#ifdef DRIVER_VIDEO_MT9V117
	/* Set GPIOs */
	cam_v_mt9v117_pwm = gpio_pwm;
	cam_v_mt9v117_en = gpio_en;
#endif

	/* Add MT9V117 vertical camera */
	p7_init_avicam(&cam_v_mt9v117_dev, &cam_v_mt9v117_pdata,
		       cam_v_mt9v117_pins, ARRAY_SIZE(cam_v_mt9v117_pins));

	/* Init I2C camera device */
	p7brd_export_i2c_hw_infos(CAMERA_V_MT9V117_I2C_BUS,
				  cam_v_mt9v117_info.addr, "Vision",
				  cam_v_mt9v117_info.type);

	/* Export PWDN GPIO */
	drone_common_export_gpio(gpio_en, GPIOF_OUT_INIT_LOW,
				 "CAMERA_V_PWDN");
}

/***********************
 * MEM2MEM and RAM2RAM *
 ***********************/

static struct avi_m2m_platform_data avi_m2m_pdata[] = {
	{ .caps = AVI_CAP_PLANAR | AVI_CAP_SCAL | AVI_CAP_CONV, },
	{ .caps = AVI_CAPS_ISP, },
	{ .caps = AVI_CAP_PLANAR | AVI_CAP_SCAL | AVI_CAP_CONV, },
	{ .caps = 0 },
};

void __init drone_common_init_m2m(struct avi_m2m_platform_data *pdata)
{
	/* Use default configuration */
	if (pdata == NULL)
		pdata = avi_m2m_pdata;

	/* Init MEM2MEM */
	p7_init_avi_m2m(avi_m2m_pdata);
}

/*******
 * USB *
 *******/

void __init drone_common_init_usb(int gpio_on, int gpio_host_mode_3v3,
				  int gpio_usb0_oc, int is_host)
{
	/* Export USB GPIOs */
	drone_common_export_gpio(gpio_host_mode_3v3, GPIOF_IN, "HOST_MODE_3V3");
	drone_common_export_gpio(gpio_usb0_oc, GPIOF_IN, "USB0_OC");

	/* Init EHCI 0
	 * EHCI controller 0 is by default dual host/device and can be
	 * forced to device by cmdline option passed by installer
	 */
	if (parrot_force_usb_device) {
		p7brd_init_usb(0, -1, CI_UDC_DR_DEVICE);
		return;
	}

	if (is_host && gpio_get_value(gpio_host_mode_3v3)) {
		pr_warn("Connected as device, don't initialize as host\n");
		is_host = 0;
	}
	p7brd_init_usb(0, gpio_on,
		       is_host ? CI_UDC_DR_DUAL_HOST :
		       CI_UDC_DR_DUAL_DEVICE);

}

/*****************
 * Sensor AK8975 *
 *****************/

#ifdef DRIVER_PARROT_IIO_AK8975
#include <iio/platform_data/mykonos3.h>
#include <linux/platform_data/ak8975.h>

#define DRONE_MAG_ROTATION_MATRIX\
		"0, -1, 0; "\
		"1, 0, 0; "\
		"0, 0, 1\n"

static struct ak8975_platform_data ak8963_pdata = {
	.orientation = DRONE_MAG_ROTATION_MATRIX,
	.drdy_gpio = -1, /* GPIO connected to DRDY pin exclusive with I2C irq */
	.trg_gpio = -1,  /* GPIO connected to TRG pin (AK8963)  */
};
#else
#define IIO_MAGNETOMETER_AK8963 "ak8963"
#endif

static struct i2c_board_info ak8963_info = {
	I2C_BOARD_INFO(IIO_MAGNETOMETER_AK8963, 0x0d),
#ifdef DRIVER_PARROT_IIO_AK8975
	.platform_data = &ak8963_pdata,
#endif
};

void __init drone_common_init_ak8963(int i2c_bus, int irq)
{
#ifdef DRIVER_PARROT_IIO_AK8975
	ak8963_info.irq = P7_GPIO_NR(irq);
	parrot_init_i2c_slave(i2c_bus, &ak8963_info, "Magnetometer",
			      irq > 0 ? P7_I2C_IRQ : P7_I2C_NOIRQ);
#else
	p7brd_export_i2c_hw_infos(i2c_bus, ak8963_info.addr, "Magnetometer",
				  ak8963_info.type);
	drone_common_export_gpio(irq, GPIOF_IN, "MAGNETO_INT_P7");
#endif
}

/**********************
 * Sensor INV_MPU6050 *
 **********************/

#ifdef DRIVER_PARROT_IIO_INV_MPU6050

#include <iio/platform_data/invensense_mpu6050.h>

#define DRONE_IMU_ROTATION_MATRIX\
	{{  1,  0,  0},\
	 {  0, -1,  0},\
	 {  0,  0, -1} }

static struct inv_mpu6050_platform_data inv_mpu6050_pdata = {
	.id		= 0,
	.orientation	= DRONE_IMU_ROTATION_MATRIX,
	.fsync		= true,
	.clkin		= true,
	.filter_rate	= 5,
};
#endif

static struct i2c_board_info inv_mpu6050_info = {
	I2C_BOARD_INFO("mpu6050", 0x68),
#ifdef DRIVER_PARROT_IIO_INV_MPU6050
	.platform_data = &inv_mpu6050_pdata,
#endif
};

#ifdef DRIVER_PARROT_IIO_INV_MPU6050
/*
 * Set the MPU6050 (gyro/accell) input clock
 * Desired frequency is 32768 Hz with 50% duty cycle (period=30517ns)
 * Period was set empirically to 31517 to get a 5ms data ready period
 * Desired frequency is slightly modified to synchronize camera and IMU
 */
#define MPU6050_PWM_PERIOD_NS 31510
static int mpu6050_pwm_enable(int clkin_pwm)
{
	struct pwm_device *mpu6050_pwm_device;
	int ret = 0;

	/* check pwm pin is valid */
	if (!gpio_is_valid(clkin_pwm))
		return -EINVAL;

	/* Request PWM */
	mpu6050_pwm_device = pwm_request(clkin_pwm, "MPU6050_CLK");
	if (IS_ERR(mpu6050_pwm_device)) {
		ret = PTR_ERR(mpu6050_pwm_device);
		goto err_alloc;
	}

	/* Configure PWM */
	ret = pwm_config(mpu6050_pwm_device, 0, MPU6050_PWM_PERIOD_NS);
	if (ret)
		goto err_config;

	/* Enable PWM */
	ret = pwm_enable(mpu6050_pwm_device);
	if (ret)
		goto err_config;

	return 0;

err_config:
	pwm_free(mpu6050_pwm_device);
err_alloc:
	pr_warn("failed to set clock for mpu6050 chip\n");
	return ret;
}
#endif

void __init drone_common_init_inv_mpu6050(int i2c_bus, int irq, int filter_rate,
					 int clkin_pwm)
{
#ifdef DRIVER_PARROT_IIO_INV_MPU6050
	mpu6050_pwm_enable(clkin_pwm);
	inv_mpu6050_pdata.filter_rate = filter_rate;
	inv_mpu6050_info.irq = P7_GPIO_NR(irq);
	parrot_init_i2c_slave(i2c_bus, &inv_mpu6050_info, "IMU",
			      irq > 0 ? P7_I2C_IRQ : P7_I2C_NOIRQ);
#else
	p7brd_export_i2c_hw_infos(i2c_bus, inv_mpu6050_info.addr, "IMU",
				  inv_mpu6050_info.type);
	/* we need to fix libHAL before exporting it here :
	   libHAL want direction file.
	 */
	//drone_common_export_gpio(irq, GPIOF_IN, "GYRO_INT_P7");
#endif
}

/*****************
 * Sensor MS5607 *
 *****************/

static struct i2c_board_info ms5607_info = {
	I2C_BOARD_INFO("ms5607", 0x77),
	.irq = -1
};

void __init drone_common_init_ms5607(int i2c_bus)
{
#ifdef DRIVER_PARROT_IIO_MS5607
	parrot_init_i2c_slave(i2c_bus, &ms5607_info, "Pressure/Temperature",
			      P7_I2C_NOIRQ);
#else
	p7brd_export_i2c_hw_infos(i2c_bus, ms5607_info.addr,
				  "Pressure/Temperature", ms5607_info.type);
#endif
}

/********
 * BLDC *
 ********/

static struct i2c_board_info bldc_info = {
#ifdef DRIVER_PARROT_IIO_BLDC_CYPRESS
	I2C_BOARD_INFO("bldc-cypress-i2c", 0x08),
	.platform_data = NULL,
#else
	I2C_BOARD_INFO("BLDC", 0x08)
#endif
};

void __init drone_common_init_bldc_with_lut(int i2c_bus, int gpio_reset,
				struct bldc_cypress_platform_data *pdata)
{
#ifdef DRIVER_PARROT_IIO_BLDC_CYPRESS
	if (pdata)
		bldc_info.platform_data = pdata;

	parrot_init_i2c_slave(i2c_bus, &bldc_info, "Motors",
			      P7_I2C_NOIRQ);
#else
	p7brd_export_i2c_hw_infos(i2c_bus, bldc_info.addr, "Motors",
				  bldc_info.type);
#endif
	/* Export reset GPIO */
	drone_common_export_gpio(gpio_reset, GPIOF_OUT_INIT_LOW, "RESET_PSOC");
}

void __init drone_common_init_bldc(int i2c_bus, int gpio_reset)
{
	drone_common_init_bldc_with_lut(i2c_bus, gpio_reset, NULL);
}

/*************************
 * FAN control (on boot) *
 *************************/

static bool drone_common_fan_en = 1;

int __init drone_common_set_fan(char *option)
{
	char *opt;

	/* Remove leading and trailing white spaces. */
	opt = strim(option);

	if (!strcmp(opt, "1") || !strcmp(opt, "on"))
		drone_common_fan_en = 1;
	if (!strcmp(opt, "0") || !strcmp(opt, "off"))
		drone_common_fan_en = 0;

	pr_info("Drone: FAN %s\n", drone_common_fan_en ? "on" : "off");

	return 0;
}
early_param("drone_fan", drone_common_set_fan);

void __init drone_common_init_fan(int gpio)
{
	drone_common_export_gpio(gpio,
				 drone_common_fan_en ? GPIOF_OUT_INIT_HIGH :
						       GPIOF_OUT_INIT_LOW,
				 "FAN");
}

/***********
 * RamOOPS *
 ***********/

#ifdef CONFIG_RAMOOPS

/* Reserved RAM used to store oops/panic logs */
#define RAMOOPS_SIZE		SZ_1M
#define RAMOOPS_RECORD_SIZE	SZ_128K

static struct ramoops_platform_data ramoops_data = {
	.mem_size    = RAMOOPS_SIZE,
	.record_size = RAMOOPS_RECORD_SIZE, /* size of each dump */
	.dump_oops   = 1, /* set to 1 to dump oopses, 0 to only dump panics */
};

static struct platform_device ramoops_dev = {
	.name = "ramoops",
	.dev  = {
		.platform_data = &ramoops_data,
	},
};
#endif

void __init drone_common_init_ramoops(void)
{
#ifdef CONFIG_RAMOOPS
	int ret;

	/* Need CONFIG_RAMOOPS in kernel config */
	ret = platform_device_register(&ramoops_dev);
	if (ret)
		pr_err("unable to register ramoops platform device\n");
	else
		pr_info("Ram Oops registered with memory from %08lx-%08lx\n",
			(long) ramoops_data.mem_address,
			(long) (ramoops_data.mem_address +
				ramoops_data.mem_size - 1));
#endif
}

void __init drone_common_reserve_mem_ramoops(void)
{
#ifdef CONFIG_RAMOOPS
	ramoops_data.mem_address = meminfo.bank[0].start +
				   meminfo.bank[0].size - RAMOOPS_SIZE;

	/* Reserve memory for RAM OOPS */
	p7_reserve_devmem(&ramoops_dev, (dma_addr_t *)&ramoops_data.mem_address,
			  (size_t *)&ramoops_data.mem_size);
#endif
}

/******************************
 * Reserved/Persistent memory *
 ******************************/

/* This platform device is only used for DMA pool reservation */
static u64 dummy_cam_dma_mask = DMA_BIT_MASK(32);
static struct platform_device dummy_cam_dev = {
	.name = "avicam",
	.id   = 0,
	.dev  = {
		.dma_mask	   = &dummy_cam_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	}
};

void __init drone_common_reserve_mem(size_t avi_size)
{
	/* Reserve memory for mem */
	p7_reserve_nand_mem();

	/* Reserve memory for USB */
	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	/* Reserve memory for AVI (cameras, m2m and venc) in global DMA pool */
	p7_reserve_avicammem(&dummy_cam_dev, avi_size);

	/* Reserve global DMA pool */
	p7_reserve_dmamem();
}

/***********
 * Helpers *
 ***********/

int __init drone_common_get_rev(int *gpios, int n_gpio, const char *name)
{
	int board_rev = 0;
	int i;

	/* Calculate board revision */
	for (i = 0; i < n_gpio; i++) {
		int err, val;

		/* Init GPIO */
		pr_info("Reading gpio-%d\n", gpios[i]);
		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]), 0, name);

		/* Get GPIO value */
		if (err == 0) {
			/* Read GPIO value */
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			pr_info("  > %d\n", val);

			/* Update board revision */
			board_rev |= (val << i);

			/* Free GPIO */
			gpio_free(P7_GPIO_NR(gpios[i]));
		} else {
			/* GPIO not available: board revision is unknown */
			pr_info("  > fail %d\n", gpios[i]);
			board_rev = -1;
			break;
		}
	}

	return board_rev;
}

/****************
 * GPIOs export *
 ****************/

void __init drone_common_export_gpio(int gpio, int flags, const char *label)
{
	/* Export GPIO */
	if (gpio_is_valid(gpio) &&
	    !p7brd_export_gpio(P7_GPIO_NR(gpio), flags, label))
		pr_info("Export GPIO-%d as %s\n", gpio, label);
}

void __init drone_common_export_gpios(struct drone_common_gpio *gpios)
{
	/* Export every GPIO */
	for (; gpios->gpio != NULL; gpios++) {
		drone_common_export_gpio(*gpios->gpio, gpios->flags, gpios->label);
	}
}

/**************
 * HSIS sysfs *
 **************/

static struct kobject *hsis_kobject;

ssize_t drone_common_hsis_sysfs_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	struct drone_common_hsis_sysfs_attr *hs = container_of(attr,
					    struct drone_common_hsis_sysfs_attr,
					    attr);
	return sprintf(buf, "%d\n", *hs->value);
}

ssize_t drone_common_hsis_sysfs_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	return count;
}

int __init drone_common_init_sysfs(struct drone_common_hsis_sysfs_attr *attrs)
{
	int err = 0;

	/* Create a new HSIS kobject for sysfs entry */
	if (hsis_kobject == NULL) {
		hsis_kobject = kobject_create_and_add("hsis", kernel_kobj);
		if (!hsis_kobject)
			return -ENOMEM;
	}

	/* Add values to sysfs */
	for (; attrs->value != NULL && !err; attrs++)
		err = sysfs_create_file(hsis_kobject, &attrs->attr.attr);

	return 0;
}
