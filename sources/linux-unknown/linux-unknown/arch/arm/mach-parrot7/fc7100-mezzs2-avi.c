#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>

#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <mach/gpio.h>
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
#include "uart.h"
#include <media/adv7604.h>
#include <media/tw9990.h>

#include "system.h"
#include "board-common.h"
#include "fc7100-module.h"
#include "fc7100-mezzs.h"
#include "fc7100-mezzs2-avi.h"
#include "fc7100-module-lcd.h"
#include "board-fc7100-workbench-variant.h"

#define FC7100_BRD_NAME "FC7100 mezz avi"

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
#define SD_WIDTH 720
#define SD_HEIGHT 576

/**
 * HDMI Input configuration
 */
#define CAM0_I2C_BUS		1
#define CAM1_I2C_BUS		2
#define CAM5_I2C_BUS		2

#define P7_CAM0_AVI_RAM_SIZE		(FULL_HD_WIDTH * FULL_HD_HEIGHT * 4 * 4)

static unsigned long fc7100_cam_schmitt_off[] = {
	P7CTL_SMT_CFG(OFF),
};

static struct pinctrl_map fc7100_r7_cam0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLKa),
	P7_INIT_PINCFG(P7_CAM_0_CLKa, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA08a),
	P7_INIT_PINCFG(P7_CAM_0_DATA08a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA09a),
	P7_INIT_PINCFG(P7_CAM_0_DATA09a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA10a),
	P7_INIT_PINCFG(P7_CAM_0_DATA10a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA11a),
	P7_INIT_PINCFG(P7_CAM_0_DATA11a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA12a),
	P7_INIT_PINCFG(P7_CAM_0_DATA12a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA13a),
	P7_INIT_PINCFG(P7_CAM_0_DATA13a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA14a),
	P7_INIT_PINCFG(P7_CAM_0_DATA14a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA15a),
	P7_INIT_PINCFG(P7_CAM_0_DATA15a, fc7100_cam_schmitt_off),
};

static struct pinctrl_map fc7100_r7_cam0_16bits_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLKa),
	P7_INIT_PINCFG(P7_CAM_0_CLKa, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA00),
	P7_INIT_PINCFG(P7_CAM_0_DATA00, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA01),
	P7_INIT_PINCFG(P7_CAM_0_DATA01, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA02),
	P7_INIT_PINCFG(P7_CAM_0_DATA02, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA03),
	P7_INIT_PINCFG(P7_CAM_0_DATA03, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA04),
	P7_INIT_PINCFG(P7_CAM_0_DATA04, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA05),
	P7_INIT_PINCFG(P7_CAM_0_DATA05, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA06),
	P7_INIT_PINCFG(P7_CAM_0_DATA06, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA07),
	P7_INIT_PINCFG(P7_CAM_0_DATA07, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA08a),
	P7_INIT_PINCFG(P7_CAM_0_DATA08a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA09a),
	P7_INIT_PINCFG(P7_CAM_0_DATA09a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA10a),
	P7_INIT_PINCFG(P7_CAM_0_DATA10a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA11a),
	P7_INIT_PINCFG(P7_CAM_0_DATA11a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA12a),
	P7_INIT_PINCFG(P7_CAM_0_DATA12a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA13a),
	P7_INIT_PINCFG(P7_CAM_0_DATA13a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA14a),
	P7_INIT_PINCFG(P7_CAM_0_DATA14a, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA15a),
	P7_INIT_PINCFG(P7_CAM_0_DATA15a, fc7100_cam_schmitt_off),
};


static struct pinctrl_map fc7100_r3_cam0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_0_CLK),
	P7_INIT_PINCFG(P7_CAM_0_CLK, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA08),
	P7_INIT_PINCFG(P7_CAM_0_DATA08, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA09),
	P7_INIT_PINCFG(P7_CAM_0_DATA09, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA10),
	P7_INIT_PINCFG(P7_CAM_0_DATA10, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA11),
	P7_INIT_PINCFG(P7_CAM_0_DATA11, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA12),
	P7_INIT_PINCFG(P7_CAM_0_DATA12, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA13),
	P7_INIT_PINCFG(P7_CAM_0_DATA13, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA14),
	P7_INIT_PINCFG(P7_CAM_0_DATA14, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_0_DATA15),
	P7_INIT_PINCFG(P7_CAM_0_DATA15, fc7100_cam_schmitt_off),
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

#define CAM0_NRST                       FC7100_IOEXPAND0_GPIO_NR(9)
#define CAM0_PWDN                       FC7100_IOEXPAND0_GPIO_NR(10)
#define CAM0_PWR_EN                     FC7100_IOEXPAND0_GPIO_NR(11)

static int cam0_power_on(void)
{
	gpio_set_value_cansleep(CAM0_NRST, 1);
	gpio_set_value_cansleep(CAM0_PWDN, 0);
	gpio_set_value_cansleep(CAM0_PWR_EN, 1);
	return 0;
}

static int cam0_power_off(void)
{
	gpio_set_value_cansleep(CAM0_NRST, 0);
	gpio_set_value_cansleep(CAM0_PWDN, 1);
	gpio_set_value_cansleep(CAM0_PWR_EN, 0);

	return 0;
}

static int cam0_set_power(int on)
{
	if(on)
		return cam0_power_on();
	else
		return cam0_power_off();
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
	.cam_it			     = -1,
	.power_on                    = &cam0_power_on,
	.power_off                   = &cam0_power_off,
};

static struct tw9990_platform_data cam0_tw9990_platform_data = {
	.set_power    = cam0_set_power,
	.differential_input = TW9990_DIFFERENTIAL_DISABLED
};

static struct i2c_board_info cam0_adv7611_device[] = {
	{
		I2C_BOARD_INFO("adv7611", 0x4C),
		.platform_data = &cam0_adv7604_platform_data,
	},
};

static struct i2c_board_info cam0_tw9990_device[] = {
	{
		I2C_BOARD_INFO("tw9990", 0x44),
		.platform_data = &cam0_tw9990_platform_data,
	},
};

static struct avicam_subdevs fc7100_mezz2_cam0_subdevs[] = {
	{
		.board_info = &cam0_adv7611_device[0],
		.i2c_adapter_id = CAM0_I2C_BUS,
	},
	{
		.board_info = &cam0_tw9990_device[0],
		.i2c_adapter_id = CAM0_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct avicam_dummy_info fc7100_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = HD_WIDTH,
		.height	    = HD_HEIGHT,
	},
};

static struct avicam_platform_data fc7100_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.dummy_driver_info = &fc7100_cam0_dummy_driver_info,
	.subdevs	   = fc7100_mezz2_cam0_subdevs,
};

static struct avicam_platform_data fc7100_cam0_16bits_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
	},
	.bus_width	   = 16,
	.dummy_driver_info = &fc7100_cam0_dummy_driver_info,
	.subdevs	   = fc7100_mezz2_cam0_subdevs,
};

static u64 fc7100_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device fc7100_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &fc7100_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

#define P7_CAM1_AVI_RAM_SIZE		(HD_WIDTH * HD_HEIGHT * 4 * 4)

/* ADV7611 HDMI input I2C subdevice addresses */

#define CAM1_NRST                       FC7100_IOEXPAND0_GPIO_NR(13)
#define CAM1_PWDN                       FC7100_IOEXPAND0_GPIO_NR(14)
#define CAM1_PWR_EN                     FC7100_IOEXPAND0_GPIO_NR(15)

static int cam1_power_on(void)
{
	gpio_set_value_cansleep(CAM1_NRST, 1);
	gpio_set_value_cansleep(CAM1_PWDN, 1);
	gpio_set_value_cansleep(CAM1_PWR_EN, 1);
	return 0;
}

static int cam1_power_off(void)
{
	gpio_set_value_cansleep(CAM1_NRST, 0);
	gpio_set_value_cansleep(CAM1_PWDN, 0);
	gpio_set_value_cansleep(CAM1_PWR_EN, 0);

	return 0;
}

static struct adv7604_platform_data cam1_adv7604_platform_data = {
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
	.cam_it			     = -1,
	.power_on                    = &cam1_power_on,
	.power_off                   = &cam1_power_off,
};

static struct i2c_board_info fc7100_mezz2_cam1_i2c_devices[] = {
	{
		I2C_BOARD_INFO("adv7611", 0x4C),
		.platform_data = &cam1_adv7604_platform_data,
	}
};

static struct avicam_subdevs fc7100_mezz2_cam1_subdevs[] = {
	{
		.board_info = &fc7100_mezz2_cam1_i2c_devices[0],
		.i2c_adapter_id = CAM1_I2C_BUS,
	},
	{ NULL, 0, },
};

static struct pinctrl_map fc7100_r3_cam1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_1_CLK),
	P7_INIT_PINCFG(P7_CAM_1_CLK, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA08),
	P7_INIT_PINCFG(P7_CAM_1_DATA08, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA09),
	P7_INIT_PINCFG(P7_CAM_1_DATA09, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA10),
	P7_INIT_PINCFG(P7_CAM_1_DATA10, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA11),
	P7_INIT_PINCFG(P7_CAM_1_DATA11, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA12),
	P7_INIT_PINCFG(P7_CAM_1_DATA12, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA13),
	P7_INIT_PINCFG(P7_CAM_1_DATA13, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA14),
	P7_INIT_PINCFG(P7_CAM_1_DATA14, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_1_DATA15),
	P7_INIT_PINCFG(P7_CAM_1_DATA15, fc7100_cam_schmitt_off),
};

static struct avicam_dummy_info fc7100_cam1_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_SMPTE170M,
		.field	    = V4L2_FIELD_INTERLACED,
		.width	    = SD_WIDTH,
		.height	    = SD_HEIGHT,
	},
};

static struct avicam_platform_data fc7100_cam1_pdata = {
	.cam_cap	   = AVI_CAP_CAM_1,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = fc7100_mezz2_cam1_subdevs,
	.dummy_driver_info = &fc7100_cam1_dummy_driver_info,
};


static u64 fc7100_cam1_dma_mask = DMA_BIT_MASK(32);
static struct platform_device fc7100_cam1_dev = {
	.name           = "avicam",
	.id             = 1,
	.dev            = {
		.dma_mask           = &fc7100_cam1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/* CAM5 */
#define P7_CAM5_WIDTH		HD_WIDTH
#define P7_CAM5_HEIGHT		HD_HEIGHT

#define P7_CAM5_AVI_RAM_SIZE		(P7_CAM5_WIDTH * P7_CAM5_HEIGHT * 4 * 4)

static struct pinctrl_map fc7100_r3_cam5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINCFG(P7_CAM_5_CLK, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINCFG(P7_CAM_5_DATA00, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINCFG(P7_CAM_5_DATA01, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINCFG(P7_CAM_5_DATA02, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINCFG(P7_CAM_5_DATA03, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINCFG(P7_CAM_5_DATA04, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINCFG(P7_CAM_5_DATA05, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINCFG(P7_CAM_5_DATA06, fc7100_cam_schmitt_off),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
	P7_INIT_PINCFG(P7_CAM_5_DATA07, fc7100_cam_schmitt_off),
};

static struct avicam_dummy_info fc7100_cam5_dummy_driver_info = {
	.dev_id = 5,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = P7_CAM5_WIDTH,
		.height	    = P7_CAM5_HEIGHT,
	},
};

static struct avicam_platform_data fc7100_cam5_pdata = {
	.cam_cap	   = AVI_CAP_CAM_5,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 0,
		.ipc        = 0,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &fc7100_cam5_dummy_driver_info,
};


static u64 fc7100_cam5_dma_mask = DMA_BIT_MASK(32);
static struct platform_device fc7100_cam5_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &fc7100_cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static void __init fc7100_mezz_avi_init_cameras(int mod_settings)
{

	if (fc7100_mezz_get_rev() >= 6)
	{
		/* Direct interrupt for CAM0 (gpio expander bypass) */
		p7_gpio_interrupt_register(P7_GPIO_NR(154));
		/* Direct interrupt for CAM1 (gpio expander bypass) */
		p7_gpio_interrupt_register(P7_GPIO_NR(161));

		/* driver adv7604 performs gpio_to_irq itself,
		 * so give it gpio number and not gpio_to_irq()
		 */
		cam0_adv7604_platform_data.cam_it = P7_GPIO_NR(154);
		cam1_adv7604_platform_data.cam_it = P7_GPIO_NR(161);

		cam0_tw9990_device[0].irq = gpio_to_irq(P7_GPIO_NR(154));
	}
	else
	{
		cam0_adv7604_platform_data.cam_it = FC7100_IOEXPAND0_GPIO_NR(8); /* CAM0_INT_1 */
	}

	pr_info(FC7100_BRD_NAME": Registering AVI CAM0\n");
	if (p7_chiprev() == P7_CHIPREV_R3)
		if (!(mod_settings & FC7100_MOD_CAM0_16)) {
			p7_init_avicam(&fc7100_cam0_dev,
					&fc7100_cam0_pdata,
					fc7100_r7_cam0_pins,
					ARRAY_SIZE(fc7100_r7_cam0_pins));
		}else{
			/* CAM0 16 bits */
			p7_init_avicam(&fc7100_cam0_dev,
					&fc7100_cam0_16bits_pdata,
					fc7100_r7_cam0_16bits_pins,
					ARRAY_SIZE(fc7100_r7_cam0_16bits_pins));

			cam0_adv7604_platform_data.op_format_sel = ADV7604_OP_FORMAT_SEL_SDR_ITU656_16;
		}
	else
		p7_init_avicam(&fc7100_cam0_dev,
				&fc7100_cam0_pdata,
				fc7100_r3_cam0_pins,
				ARRAY_SIZE(fc7100_r3_cam0_pins));

	/* we have cam1 if cam0 is not 16 bits */
	if (!(mod_settings & FC7100_MOD_CAM0_16)) {
		pr_info(FC7100_BRD_NAME": Registering AVI CAM1\n");
		p7_init_avicam(&fc7100_cam1_dev,
				&fc7100_cam1_pdata,
				fc7100_r3_cam1_pins,
				ARRAY_SIZE(fc7100_r3_cam1_pins));
	}

	if (mod_settings & FC7100_MOD_CAM5) {
		pr_info(FC7100_BRD_NAME": Registering AVI CAM5\n");
		p7_init_avicam(&fc7100_cam5_dev,
				&fc7100_cam5_pdata,
				fc7100_r3_cam5_pins,
				ARRAY_SIZE(fc7100_r3_cam5_pins));
	}
}

void __init fc7100_mezz_reserve_mem_for_camera(void)
{
	p7_reserve_avicammem(&fc7100_cam0_dev, P7_CAM0_AVI_RAM_SIZE);
	p7_reserve_avicammem(&fc7100_cam1_dev, P7_CAM1_AVI_RAM_SIZE);
	p7_reserve_avicammem(&fc7100_cam5_dev, P7_CAM5_AVI_RAM_SIZE);
}

static int fc7100_lcd0_novtk = 0;

static int __init fc7100_setup_lcd0_novtk(char *options)
{
	pr_info("%s: Disabling Parrot VTK overlay\n", FC7100_BRD_NAME);

	fc7100_lcd0_novtk = 1;

	return 0;
}

early_param("fc7100_novtk", fc7100_setup_lcd0_novtk);

/* Reserve enough memory for a single dual-buffered 32bpp full HD
 * screen. Probably a lot more than we'll actually need. */

static struct avifb_overlay fc7100_avi_lcd0_overlays_vtk[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.x	 = 64,
			.width	 = 1216,
			.enabled = 1,
		},
		.zorder		 = -1,
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
	},
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.width	 = 64,
			.enabled = 1,
		},
		.zorder		 = -1,
		.dma_memory.end	 = 64 * 800 * 4 * 2,
	},
};

static struct avifb_overlay fc7100_avi_lcd0_overlays_novtk[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA_OSD,
			.enabled = 1,
		},
		.zorder		 = -1,
		.dma_memory.end	 = 1280 * 800 * 4 * 2,
	},
};

void __init fc7100_mezz_avi_init(const char *screen_name,
				 int force_fc7100_novtk, int mod_settings)
{
	int xres, yres;

	p7_init_avi();

	if (fc7100_variant && fc7100_variant->init_lcd) {
		fc7100_variant->init_lcd();
	} else {
		fc7100_mezz_avi_init_lcd_screen(screen_name, &xres, &yres);
		if (force_fc7100_novtk || fc7100_lcd0_novtk) {
			fc7100_mezz_avi_init_lcd(fc7100_avi_lcd0_overlays_novtk,
						 ARRAY_SIZE(fc7100_avi_lcd0_overlays_novtk));
		} else {
			fc7100_avi_lcd0_overlays_vtk[0].layout.width = xres - 64;
			pr_info("%s: fb0 width set to %d\n", FC7100_BRD_NAME, xres - 64);
			fc7100_mezz_avi_init_lcd(fc7100_avi_lcd0_overlays_vtk,
						 ARRAY_SIZE(fc7100_avi_lcd0_overlays_vtk));
		}
	}

	fc7100_mezz_avi_init_cameras(mod_settings);

}

void fc7100_mezz_avi_update(const char *screen_name, int force_fc7100_novtk)
{
	int xres, yres;

	fc7100_mezz_avi_init_lcd_screen(screen_name, &xres, &yres);
	if (!force_fc7100_novtk && !fc7100_lcd0_novtk &&
	    fc7100_avi_lcd0_overlays_vtk[0].layout.width != xres - 64) {
		pr_info("%s: fb0 width updated to %d for %s %dx%d\n",
			FC7100_BRD_NAME, xres - 64, screen_name, xres, yres);
		fc7100_avi_lcd0_overlays_vtk[0].layout.width = xres - 64;
	}
}

void __init fc7100_mezz_reserve_mem_lcd_novtk(void)
{
	if (fc7100_variant && fc7100_variant->reserve_lcd_mem) {
		fc7100_variant->reserve_lcd_mem();
		return;
	}

	fc7100_mezz_reserve_mem_for_lcd(fc7100_avi_lcd0_overlays_novtk, ARRAY_SIZE(fc7100_avi_lcd0_overlays_novtk));
}

void __init fc7100_mezz_reserve_mem_lcd(void)
{
	if (fc7100_variant && fc7100_variant->reserve_lcd_mem) {
		fc7100_variant->reserve_lcd_mem();
		return;
	}

	if (fc7100_lcd0_novtk)
		fc7100_mezz_reserve_mem_for_lcd(fc7100_avi_lcd0_overlays_novtk, ARRAY_SIZE(fc7100_avi_lcd0_overlays_novtk));
	else
		fc7100_mezz_reserve_mem_for_lcd(fc7100_avi_lcd0_overlays_vtk, ARRAY_SIZE(fc7100_avi_lcd0_overlays_vtk));

}
