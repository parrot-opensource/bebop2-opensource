/**
 * linux/arch/arm/mach-parrot7/fc7100-board.c - Parrot7 FC7100 platform
 *                                              implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    31-Aug-2012
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
#include "common.h"
#include "system.h"
#include <gpio/p7-gpio.h>
#include "spi.h"
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>

#include "board-common.h"
#include "fc7100-mezzs.h"
#include "gpio.h"
#include "sii_platform_data.h"

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
	.max_speed_hz       = 40000000,
	.chip_select        = 0,
	.mode               = SPI_MODE_0,
};

static struct platform_device virtual_wakeup_button = {
	.name           = "virtual_wakeup_button",
};

static struct sii5293_platform_data sii5293_pdata = {
	.version        = SII5293_PDATA_STRUCT_VERSION,
	.i2c_bus        = 1,
	.gpio_rst       = FC7100_IOEXPAND0_GPIO_NR(9),
	.gpio_irq       = P7_GPIO_NR(154),
	.output_format  = 8,
	.input_dev_rap  = 1,
	.input_dev_rcp  = 1,
	.input_dev_ucp  = 1,
};

static struct platform_device sii5293_pdev = {
	.name           = "sii-5293",
	.id             = 0,
};

/* hardware variation :
   sd1 or uart[5-7]
   cam0+cam1 or cam0 x16
   sd2 or cam5
 */
static int fc7100_mod_cam5 = 0;

#define FC7100_BRD_NAME "FC7100 workbench"

static int __init fc7100_setup_mod_cam5(char *options)
{
	pr_info("%s: Enabling AVI CAM5\n", FC7100_BRD_NAME);

	fc7100_mod_cam5 = 1;

	return 0;
}

early_param("fc7100_mod_cam5", fc7100_setup_mod_cam5);

static void __init init_board(void)
{
	int flags = FC7100_MOD_SDCARD1|FC7100_MOD_SDCARD2;
	int err;

	if (fc7100_mod_cam5) {
		flags |= FC7100_MOD_CAM5;
		flags &= ~FC7100_MOD_SDCARD2;
	}

	fc7100_init_module(flags);

	err = fc7100_init_mezz(flags|FC7100_MEZZ_MPEGTS, "kyocera");
	WARN_ON(err);
	/* for octopus control */
	p7_init_spim_slave(0, &fc7100_spim_dev);

	/* Export unconfigured devices informations */
	p7brd_export_i2c_hw_infos(2, 0x10, "2C", "ipod");

	// Registering virtual wakeup button for resume
	// Needed by android
	platform_device_register(&virtual_wakeup_button);
	p7_init_dev(&sii5293_pdev, &sii5293_pdata, NULL, 0);
}

P7_MACHINE_START(PARROT_FC7100, "FC7100 Workbench")
	.reserve        = &fc7100_mezz_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END


