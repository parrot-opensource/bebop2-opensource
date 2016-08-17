/**
 * linux/arch/arm/mach-parrot7/fpga-board.c - P7Dev ISP FPGA daughter board
 *                                            implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    05-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <mach/pwm.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "pinctrl.h"
#include "spi.h"
#include "board.h"
#include "board-common.h"
#include "lcd-monspecs.h"
#include "avi.h"
#include "backlight.h"

#define FPGA_BRD_NAME   "fpga"

#if ! (defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) || \
       defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE))
#error Parrot7 AVI cam and / or frame buffer drivers disabled
#endif
#if ! (defined(CONFIG_SPI_MASTER_PARROT7) || \
	   defined(CONFIG_SPI_MASTER_PARROT7_MODULE))
#error Parrot7 SPI master driver disabled
#endif

static struct pinctrl_map fpgadb_s6spi_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12),   /* CLK */
	P7_INIT_PINMAP(P7_SPI_13),   /* MOSI */
	P7_INIT_PINMAP(P7_SPI_15),   /* MISO */
	P7_INIT_PINMAP(P7_SPI_00a),
	P7_INIT_PINMAP(P7_SPI_01a),
	P7_INIT_PINMAP(P7_SPI_02a),
	P7_INIT_PINMAP(P7_SPI_03a),
	P7_INIT_PINMAP(P7_SPI_04a),
	P7_INIT_PINMAP(P7_SPI_05a),
	P7_INIT_PINMAP(P7_SPI_06a),
	P7_INIT_PINMAP(P7_SPI_07a),
	P7_INIT_PINMAP(P7_SPI_08a),
	P7_INIT_PINMAP(P7_SPI_09a),
};

/*
 * We'll need only MOSI and CLK signals, together with 2 GPIOs to program
 * Spartan6 FPGA. It is connected to Master 2.
 */
static struct p7spi_swb const fpgadb_s6spi_swb[] = {
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_OUT,     P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_IN,      P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data fpgadb_s6spi_cdata = {
	.half_duplex        = true,
	.read               = true,
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

static P7_DECLARE_SPIM_SLAVE(fpgadb_s6spi_dev,
                             "spidev",
                             NULL,
                             &fpgadb_s6spi_cdata,
                             10000000,
                             SPI_MODE_0);

/***********************
 * AVI configuration
 ***********************/

static struct pinctrl_map p7dev_avifb1_pins[] __initdata = {
	/* LCD1 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_1_CLK),
	P7_INIT_PINMAP(P7_LCD_1_DATA00),
	P7_INIT_PINMAP(P7_LCD_1_DATA01),
	P7_INIT_PINMAP(P7_LCD_1_DATA02),
	P7_INIT_PINMAP(P7_LCD_1_DATA03),
	P7_INIT_PINMAP(P7_LCD_1_DATA04),
	P7_INIT_PINMAP(P7_LCD_1_DATA05),
	P7_INIT_PINMAP(P7_LCD_1_DATA06),
	P7_INIT_PINMAP(P7_LCD_1_DATA07),
	P7_INIT_PINMAP(P7_LCD_1_DATA08),
	P7_INIT_PINMAP(P7_LCD_1_DATA09),
	P7_INIT_PINMAP(P7_LCD_1_DATA10),
	P7_INIT_PINMAP(P7_LCD_1_DATA11),
	P7_INIT_PINMAP(P7_LCD_1_DATA12),
	P7_INIT_PINMAP(P7_LCD_1_DATA13),
	P7_INIT_PINMAP(P7_LCD_1_DATA14),
	P7_INIT_PINMAP(P7_LCD_1_DATA15),
	P7_INIT_PINMAP(P7_LCD_1_DATA16),
	P7_INIT_PINMAP(P7_LCD_1_DATA17),
	P7_INIT_PINMAP(P7_LCD_1_DATA18),
	P7_INIT_PINMAP(P7_LCD_1_DATA19),
	P7_INIT_PINMAP(P7_LCD_1_DATA20),
	P7_INIT_PINMAP(P7_LCD_1_DATA21),
	P7_INIT_PINMAP(P7_LCD_1_DATA22),
	P7_INIT_PINMAP(P7_LCD_1_DATA23),
	P7_INIT_PINMAP(P7_LCD_1_DEN),
	P7_INIT_PINMAP(P7_LCD_1_HS),
	P7_INIT_PINMAP(P7_LCD_1_VS),
};

static struct avifb_overlay p7dev_avi_lcd1_overlays[] = {
	{
		.layout		 = {
			.alpha	 = AVI_ALPHA(100),
			.enabled = 1,
		},
		.dma_memory.end	 = (1920 * 1080 * 4 * 2),
		.zorder		 = 0,
	},
};

static struct avifb_platform_data p7dev_avifb1_pdata = {
	.lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &lt104ac_video_mode,
	.lcd_videomodes        = p7_all_video_modes,
	.lcd_interface	       = {{
		.free_run      = 1,
		.itu656        = 0,
	}},
	.caps		       = AVI_CAP_LCD_0 | AVI_CAP_CONV,
	.overlays	       = p7dev_avi_lcd1_overlays,
	.overlay_nr	       = ARRAY_SIZE(p7dev_avi_lcd1_overlays),
};

static u64 p7dev_avifb1_dma_mask = DMA_BIT_MASK(32);

static struct platform_device p7dev_avifb1_dev = {
	.name           = "avifb",
	.id             = 1,
	.dev            = {
		.dma_mask           = &p7dev_avifb1_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/***********************
 * Camera configuration
 ***********************/
#if defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE)


static struct pinctrl_map p7dev_cam0_pins[] __initdata = {
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

static struct pinctrl_map p7dev_cam5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAM_5_CLK),
	P7_INIT_PINMAP(P7_CAM_5_HS),
	P7_INIT_PINMAP(P7_CAM_5_VS),
	P7_INIT_PINMAP(P7_CAM_5_DATA00),
	P7_INIT_PINMAP(P7_CAM_5_DATA01),
	P7_INIT_PINMAP(P7_CAM_5_DATA02),
	P7_INIT_PINMAP(P7_CAM_5_DATA03),
	P7_INIT_PINMAP(P7_CAM_5_DATA04),
	P7_INIT_PINMAP(P7_CAM_5_DATA05),
	P7_INIT_PINMAP(P7_CAM_5_DATA06),
	P7_INIT_PINMAP(P7_CAM_5_DATA07),
};

static struct avicam_dummy_info p7dev_cam0_dummy_driver_info = {
	.dev_id = 0,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = 1920,
		.height	    = 1080,
	},
};

static struct avicam_dummy_info p7dev_cam5_dummy_driver_info = {
	.dev_id = 1,
	.format = {
		.code	    = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace = V4L2_COLORSPACE_REC709,
		.field	    = V4L2_FIELD_NONE,
		.width	    = 1920,
		.height	    = 1080,
	},
};

static struct avicam_platform_data p7dev_cam0_pdata = {
	.cam_cap	   = AVI_CAP_CAM_0,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &p7dev_cam0_dummy_driver_info,
};

static struct avicam_platform_data p7dev_cam5_pdata = {
	.cam_cap	   = AVI_CAP_CAM_5,
	.interface         = {
		.itu656	    = 1,
		.pad_select = 1,
	},
	.bus_width	   = 8,
	.subdevs	   = NULL,
	.dummy_driver_info = &p7dev_cam5_dummy_driver_info,
};

static u64 p7dev_cam0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7dev_cam0_dev = {
	.name           = "avicam",
	.id             = 0,
	.dev            = {
		.dma_mask           = &p7dev_cam0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

static u64 p7dev_cam5_dma_mask = DMA_BIT_MASK(32);
static struct platform_device p7dev_cam5_dev = {
	.name           = "avicam",
	.id             = 5,
	.dev            = {
		.dma_mask           = &p7dev_cam5_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

/* Bayer FPGA input configuration */

/* Cresyn input configuration */
/* Register devboard sensors */

#endif /* defined(CONFIG_CAM_AVI) || defined(CONFIG_CAM_AVI_MODULE) */

#if defined(CONFIG_AVI) || defined(CONFIG_AVI_MODULE)

#endif /* defined(CONFIG_AVI) || defined(CONFIG_AVI_MODULE) */

#include <i2c/muxes/p7-i2cmux.h>
#include "i2cm.h"

/* The alternate buses. There will be one additional "default" bus.*/
static char const* const fpgadb_i2cm2_mux_names[] = {
	"mux-a", "mux-b", "mux-c", "mux-d", "mux-e", "mux-all"
};

static struct pinctrl_map fpgadb_i2cm2_muxa_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKa),
	P7_INIT_PINMAP(P7_I2C_2_DATa)
};

static struct pinctrl_map fpgadb_i2cm2_muxb_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKb),
	P7_INIT_PINMAP(P7_I2C_2_DATb)
};

static struct pinctrl_map fpgadb_i2cm2_muxc_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKc),
	P7_INIT_PINMAP(P7_I2C_2_DATc)
};

static struct pinctrl_map fpgadb_i2cm2_muxd_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKd),
	P7_INIT_PINMAP(P7_I2C_2_DATd)
};

static struct pinctrl_map fpgadb_i2cm2_muxe_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKe),
	P7_INIT_PINMAP(P7_I2C_2_DATe)
};

/*
 * Pseudo port for talking to all ports
 */
static struct pinctrl_map fpgadb_i2cm2_muxall_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLKa),
	P7_INIT_PINMAP(P7_I2C_2_DATa),

	P7_INIT_PINMAP(P7_I2C_2_CLKb),
	P7_INIT_PINMAP(P7_I2C_2_DATb),

	P7_INIT_PINMAP(P7_I2C_2_CLKc),
	P7_INIT_PINMAP(P7_I2C_2_DATc),

	P7_INIT_PINMAP(P7_I2C_2_CLKd),
	P7_INIT_PINMAP(P7_I2C_2_DATd),

	P7_INIT_PINMAP(P7_I2C_2_CLKe),
	P7_INIT_PINMAP(P7_I2C_2_DATe),
};

static struct p7i2cmux_pins fpgadb_i2cm2_mux_pins[] __initdata = {
	{
		.pinmap = fpgadb_i2cm2_muxa_pins,
		.sz     = ARRAY_SIZE(fpgadb_i2cm2_muxa_pins)
	},
	{
		.pinmap = fpgadb_i2cm2_muxb_pins,
		.sz     = ARRAY_SIZE(fpgadb_i2cm2_muxb_pins)
	},
	{
		.pinmap = fpgadb_i2cm2_muxc_pins,
		.sz     = ARRAY_SIZE(fpgadb_i2cm2_muxc_pins)
	},
	{
		.pinmap = fpgadb_i2cm2_muxd_pins,
		.sz     = ARRAY_SIZE(fpgadb_i2cm2_muxd_pins)
	},
	{
		.pinmap = fpgadb_i2cm2_muxe_pins,
		.sz     = ARRAY_SIZE(fpgadb_i2cm2_muxe_pins)
	},
	{
		.pinmap = fpgadb_i2cm2_muxall_pins,
		.sz     = ARRAY_SIZE(fpgadb_i2cm2_muxall_pins)
	}
};

static struct p7i2cmux_plat_data fpgadb_i2cm2_mux_pdata = {
	.channel_names = fpgadb_i2cm2_mux_names,
	.nr_channels   = ARRAY_SIZE(fpgadb_i2cm2_mux_names)
};

void __init fpgadb_rsvmem(struct p7_board const* board)
{
	p7_reserve_avifbmem(&p7dev_avifb1_dev,
	                    p7dev_avi_lcd1_overlays,
	                    ARRAY_SIZE(p7dev_avi_lcd1_overlays));

	/* XXX I have no idea how much memory I should reserve for the
	 * cameras */
	p7_reserve_avicammem(&p7dev_cam0_dev, 0);
	p7_reserve_avicammem(&p7dev_cam5_dev, 0);
}

void __init fpgadb_probe(struct p7_board const* board)
{
	int err;

	p7_init_avi();

	p7_init_avifb(&p7dev_avifb1_dev, &p7dev_avifb1_pdata,
		      p7dev_avifb1_pins, ARRAY_SIZE(p7dev_avifb1_pins));
	p7_init_avicam(&p7dev_cam0_dev,
	               &p7dev_cam0_pdata,
	               p7dev_cam0_pins,
	               ARRAY_SIZE(p7dev_cam0_pins));

	p7_init_avicam(&p7dev_cam5_dev,
	               &p7dev_cam5_pdata,
	               p7dev_cam5_pins,
	               ARRAY_SIZE(p7dev_cam5_pins));

	p7brd_init_i2cm_muxed(2, 100, NULL, 0,
			      &fpgadb_i2cm2_mux_pdata,
			      fpgadb_i2cm2_mux_pins);

	p7_init_spim(2,
	             fpgadb_s6spi_pins,
	             ARRAY_SIZE(fpgadb_s6spi_pins),
	             fpgadb_s6spi_swb);

	err = p7_init_spim_slave(2, &fpgadb_s6spi_dev);
	WARN(err,
	     "%s: failed to initialize Spartan6 SPI interface (%d)\n",
		 board->name,
		 err);

	p7_init_bkl(0,
	            NULL,
	            P7MU_FIRST_PWM + 1,
	            -1,
	            P7MU_FIRST_GPIO + 2,
	            NSEC_PER_SEC / 200,    /* 200 Hz */
	            256,                   /* 100% duty cycle / brightness */
	            3);                    /* 1% duty cycle / brightness */

	/*
	 * On the FPGA board, "IO_5_P7MU" pin deresets all LCD0/1 related
	 * controllers.
	 */
	err = gpio_request_one(P7MU_FIRST_GPIO + 4,
	                       GPIOF_OUT_INIT_HIGH,
	                       "FPGA board nRST");
	WARN(err,
	     FPGA_BRD_NAME ": failed to de-reset board (%d)\n",
		 err);
}
