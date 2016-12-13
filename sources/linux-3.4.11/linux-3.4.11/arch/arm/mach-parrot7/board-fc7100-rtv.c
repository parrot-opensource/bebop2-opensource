/**
 * linux/arch/arm/mach-parrot7/board-rtv.c - Parrot7 RTV board
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    Nov-2013
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
#include <linux/persistent_ram.h>
#include <linux/ramoops.h>
#include "spi.h"
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>
#include <gpio/p7-gpio.h>

#include "common.h"
#include "system.h"
#include "board-common.h"
#include "fc7100-module.h"
#include "gpio.h"
#include "mpegts.h"

#include "avi.h"
#include "venc.h"
#include "vdec.h"
#include "nand.h"
#include "usb.h"

static struct avi_m2m_platform_data fc7100_rtv_avi_m2m_pdata[] = {
	{ .caps = AVI_CAP_PLANAR | AVI_CAP_SCAL | AVI_CAP_CONV },
	{ .caps = 0 },
};

/** VOC connected to RAM2RAM*/
static struct avi_voc_plat_data fc7100_rtv_avi_voc_m2m = {
	.display = "m2m.0",
};

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

#include <mmc/acs3-sdhci.h>
#include <host/sdhci.h>

static struct acs3_plat_data fc7100_sdhci_emmc_pdata = {
	.led_gpio   = -1,  /* No activity led GPIO */
	.wp_gpio    = -1,  /* No write protect */
	.cd_gpio    = -1,  /* No card detect */
	.rst_gpio   = P7_GPIO_NR(188),  /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = MMC_CAP_NONREMOVABLE,     /* emmc is non removable */
	.mmc_caps2  = MMC_CAP2_BROKEN_VOLTAGE,  /* bus voltage is fixed in hardware */
	.nb_config   = 1,
};

/******************************/
/* Ramoops and persistent ram */
/******************************/

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

static void __init init_board(void)
{
	unsigned int mod_settings = 0;
	int ret=0;

	fc7100_init_module(mod_settings);

	/* Need CONFIG_RAMOOPS in kernel config */
	ret = platform_device_register(&ramoops_dev);
	if (ret) {
		printk(KERN_ERR "unable to register ramoops platform device\n");
	}

	/* tms uart */
	p7brd_init_uart(0, 0);
	p7brd_export_gpio(57, GPIOF_OUT_INIT_HIGH, "fc7100_ready");
	p7brd_export_gpio(58, GPIOF_IN, "tms_request");

	/* SD inits */
	p7brd_init_sdhci(2, &fc7100_sdhci_emmc_pdata,
			 NULL, NULL, NULL, NULL, 0);

	/* USB0 is device only */
	p7brd_init_udc(0, P7_GPIO_NR(84));

	/* USB1 is host only */
	p7brd_init_hcd(1, P7_GPIO_NR(82));


	/* ethernet */
	p7_init_ether(PHY_IFACE_MII, -1, P7CTL_DRV_CFG(5));
	p7brd_export_gpio(P7_GPIO_NR(133), GPIOF_OUT_INIT_LOW, "ethernet_reset");

	/* octopus */
	p7_init_spim_slave(0, &fc7100_spim_dev);
	fc7100_init_spim_single(0, 1, 0, 3, 2);
	fc7100_init_mpegts_single(0, 11, 10);
	fc7100_init_mpegts_single(1, 9, 8);
	p7brd_export_gpio(P7_GPIO_NR(130), GPIOF_OUT_INIT_LOW, "octopus-pwr");
	p7brd_export_gpio(P7_GPIO_NR(129), GPIOF_DIR_IN,        "octopus-int");

	p7_init_venc();
	p7_init_vdec();

	p7_init_avi();
	p7_init_avi_m2m(fc7100_rtv_avi_m2m_pdata);
	p7_init_avi_voc(2, &fc7100_rtv_avi_voc_m2m);

	/* SIM is locked */
	p7brd_export_gpio(P7_GPIO_NR(191), GPIOF_DIR_IN, "sim_locked");
}

static void __init rtv_reserve_mem(void)
{
	struct membank *bank = &meminfo.bank[0];

	ramoops_data.mem_address = bank->start + bank->size-RAMOOPS_SIZE;
	p7_reserve_devmem(&ramoops_dev,
			  (dma_addr_t *)&ramoops_data.mem_address,
			  (size_t *)&ramoops_data.mem_size);

#define FC7100_HX280_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(FC7100_HX280_SIZE);

#define FC7100_HX270_SIZE (CONFIG_ARCH_PARROT7_FC7100_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(FC7100_HX270_SIZE);

#define FC7100_MPGTS_SIZE (CONFIG_ARCH_PARROT7_FC7100_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, FC7100_MPGTS_SIZE);
	p7_reserve_mpegtsmem(1, FC7100_MPGTS_SIZE);

#define FC7100_AVI_M2M_SIZE (1920 * 1080 * 2 * 4 *2 )
	p7_reserve_avi_m2m_mem(FC7100_AVI_M2M_SIZE);

#define FC7100_VOC_SIZE (1920 * 1080 * 4 * 2)
	p7_reserve_avi_voc_mem(2, FC7100_VOC_SIZE);

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_RTV, "RTV")
	.reserve        = &rtv_reserve_mem,
	.init_machine   = &init_board,
P7_MACHINE_END


