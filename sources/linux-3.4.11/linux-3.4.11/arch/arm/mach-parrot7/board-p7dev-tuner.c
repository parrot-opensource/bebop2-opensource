/*
 * linux/arch/arm/mach-parrot7/board-p7dev-tuner.c - Octopus board
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Damien Riegel <damien.riegel.ext@parrot.com>
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/gpio.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "pinctrl.h"
#include "board.h"
#include "spi.h"
#include "mpegts.h"
#include "mach/gpio.h"
#include "board-common.h"

/*
 * The Octopus has two MPEG-TS cores and is controlled with SPI.
 */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_TUNER
/*
 * SPI master to control mpegts cores
 */
#define P7DEV_SPI_TUNER_CTRL_BUS        0

static struct pinctrl_map p7dev_tuner_ctrl_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_00),
	P7_INIT_PINMAP(P7_SPI_01),
	P7_INIT_PINMAP(P7_SPI_02),
	P7_INIT_PINMAP(P7_SPI_03),
};

static struct p7spi_swb const p7dev_tuner_ctrl_swb[] = {
	P7SPI_INIT_SWB(0, P7_SWB_DIR_OUT,   P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(1, P7_SWB_DIR_OUT,   P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(2, P7_SWB_DIR_OUT,   P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(3, P7_SWB_DIR_IN,    P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data p7dev_tuner_ctrl_cdata = {
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
};

static P7_DECLARE_SPIM_SLAVE(p7dev_tuner_ctrl_info,
                             "spidev",
                             NULL,
                             &p7dev_tuner_ctrl_cdata,
                             40 * 1000 * 1000,
                             SPI_MODE_0);

/*
 * core 0
 */
static struct pinctrl_map p7dev_tuner0_hw00_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_04),
	P7_INIT_PINMAP(P7_SPI_05),
};

static struct p7spi_swb const p7dev_tuner0_hw00_swb[] = {
	P7SPI_INIT_SWB(5, P7_SWB_DIR_IN,   P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB(4, P7_SWB_DIR_IN,   P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

struct p7mpg_plat_data p7dev_tuner0_hw00_pdata = {
	.swb         = p7dev_tuner0_hw00_swb,
	.fifo_wcnt   = 16,
	.thres_wcnt  = 4,
};

static struct pinctrl_map p7dev_tuner0_hw01_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_16),
	P7_INIT_PINMAP(P7_SPI_17),
};

static struct p7spi_swb const p7dev_tuner0_hw01_swb[] = {
	P7SPI_INIT_SWB(16, P7_SWB_DIR_IN,   P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB(17, P7_SWB_DIR_IN,   P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

struct p7mpg_plat_data p7dev_tuner0_hw01_pdata = {
	.swb         = p7dev_tuner0_hw01_swb,
	.fifo_wcnt   = 16,
	.thres_wcnt  = 4,
};

/*
 * core 1
 */
static struct pinctrl_map p7dev_tuner1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_18),
	P7_INIT_PINMAP(P7_SPI_19),
};

static struct p7spi_swb const p7dev_tuner1_swb[] = {
	P7SPI_INIT_SWB(18, P7_SWB_DIR_IN,   P7_SWB_MPEG_CLK),
	P7SPI_INIT_SWB(19, P7_SWB_DIR_IN,   P7_SWB_MPEG_DATA),
	P7SPI_SWB_LAST,
};

struct p7mpg_plat_data p7dev_tuner1_pdata = {
	.swb         = p7dev_tuner1_swb,
	.fifo_wcnt   = 16,
	.thres_wcnt  = 4,
};


void __init tunerdb_probe(struct p7_board const* brd)
{
	p7_init_spim(P7DEV_SPI_TUNER_CTRL_BUS, p7dev_tuner_ctrl_pins,
	             ARRAY_SIZE(p7dev_tuner_ctrl_pins), p7dev_tuner_ctrl_swb);

	if (p7_init_spim_slave(P7DEV_SPI_TUNER_CTRL_BUS, &p7dev_tuner_ctrl_info)) {
		pr_err("%s: failed to initialize SPI master.\n", brd->name);
		return;
	}

	if (brd->conn->id == 550) {
		p7_init_mpegts(0, &p7dev_tuner0_hw00_pdata,
			       p7dev_tuner0_hw00_pins,
		               ARRAY_SIZE(p7dev_tuner0_hw00_pins));
	} else if (brd->conn->id == 2000) {
		p7_init_mpegts(0, &p7dev_tuner0_hw01_pdata,
			       p7dev_tuner0_hw01_pins,
		               ARRAY_SIZE(p7dev_tuner0_hw01_pins));
		p7_init_mpegts(1, &p7dev_tuner1_pdata, p7dev_tuner1_pins,
		               ARRAY_SIZE(p7dev_tuner1_pins));
	}
	
	/* octopus pwdn gpio */
	p7brd_export_gpio(P7_GPIO_NR(85), GPIOF_OUT_INIT_LOW, "octopus-pwr");
}

void __init tunerdb_rsvmem(struct p7_board const* brd)
{
#define P7DEV_TUNER_MPGTS_SIZE (CONFIG_ARCH_PARROT7_P7DEV_TUNER_MPGTS_SIZE * SZ_1K)
	p7_reserve_mpegtsmem(0, P7DEV_TUNER_MPGTS_SIZE);
	p7_reserve_mpegtsmem(1, P7DEV_TUNER_MPGTS_SIZE);
}


#else /* CONFIG_ARCH_PARROT7_P7DEV_TUNER */
#define tunerdb_probe(_brd)
#define tunerdb_rsvmem(_brd)
#endif
