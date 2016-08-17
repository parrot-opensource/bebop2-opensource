/**
 * linux/arch/arm/mach-parrot7/nor-board.c - P7Dev SPI Nor flash daughter board
 *                                           implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    31-Oct-2012
 *
 * This file is released under the GPL
 */

#include <spi/p7-spi.h>
#include "pinctrl.h"
#include "spi.h"
#include "board.h"

#if ! (defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE))
#error m25p80 spi nor flash driver disabled
#endif
#if ! (defined(CONFIG_SPI_MASTER_PARROT7) || \
	   defined(CONFIG_SPI_MASTER_PARROT7_MODULE))
#error Parrot7 SPI master driver disabled
#endif

static unsigned long nordb_sscfg[] = {
	P7CTL_DRV_CFG(3)
};

static unsigned long nordb_clkcfg[] = {
	P7CTL_DRV_CFG(4)
};

static unsigned long nordb_datcfg[] = {
	P7CTL_PUD_CFG(DOWN),
	P7CTL_DRV_CFG(4)
};

static struct pinctrl_map nordb_pins[4][8] __initdata = {
	[0] = { /* Master 0 */
		P7_INIT_PINMAP(P7_SPI_02), /* SS */
		P7_INIT_PINCFG(P7_SPI_02, nordb_sscfg),
		P7_INIT_PINMAP(P7_SPI_01), /* CLK */
		P7_INIT_PINCFG(P7_SPI_01, nordb_clkcfg),
		P7_INIT_PINMAP(P7_SPI_00), /* MOSI */
		P7_INIT_PINCFG(P7_SPI_00, nordb_datcfg),
		P7_INIT_PINMAP(P7_SPI_03), /* MISO */
		P7_INIT_PINCFG(P7_SPI_03, nordb_datcfg)
	},
	[1] = { /* Master 1 */
		P7_INIT_PINMAP(P7_SPI_06), /* SS */
		P7_INIT_PINCFG(P7_SPI_06, nordb_sscfg),
		P7_INIT_PINMAP(P7_SPI_05), /* CLK */
		P7_INIT_PINCFG(P7_SPI_05, nordb_clkcfg),
		P7_INIT_PINMAP(P7_SPI_04), /* MOSI */
		P7_INIT_PINCFG(P7_SPI_04, nordb_datcfg),
		P7_INIT_PINMAP(P7_SPI_07), /* MISO */
		P7_INIT_PINCFG(P7_SPI_07, nordb_datcfg)
	},
	[2] = { /* Master 2 */
		P7_INIT_PINMAP(P7_SPI_10), /* SS */
		P7_INIT_PINCFG(P7_SPI_10, nordb_sscfg),
		P7_INIT_PINMAP(P7_SPI_09), /* CLK */
		P7_INIT_PINCFG(P7_SPI_09, nordb_clkcfg),
		P7_INIT_PINMAP(P7_SPI_08), /* MOSI */
		P7_INIT_PINCFG(P7_SPI_08, nordb_datcfg),
		P7_INIT_PINMAP(P7_SPI_11), /* MISO */
		P7_INIT_PINCFG(P7_SPI_11, nordb_datcfg)
	},
	[3] = { /* Master 3 */
		P7_INIT_PINMAP(P7_SPI_14), /* SS */
		P7_INIT_PINCFG(P7_SPI_14, nordb_sscfg),
		P7_INIT_PINMAP(P7_SPI_13), /* CLK */
		P7_INIT_PINCFG(P7_SPI_13, nordb_clkcfg),
		P7_INIT_PINMAP(P7_SPI_12), /* MOSI */
		P7_INIT_PINCFG(P7_SPI_12, nordb_datcfg),
		P7_INIT_PINMAP(P7_SPI_15), /* MISO */
		P7_INIT_PINCFG(P7_SPI_15, nordb_datcfg)
	}
};

/*
 * Single SPI half-duplex mode SPI controller internal multiplexing setup:
 * the same line (DATA0) is used alternativly for MISO and MOSI functions.
 */
static struct p7spi_swb const nordb_swb[4][5] = {
	[0] = { /* Master 0 */
		P7SPI_INIT_SWB(2,   P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
		P7SPI_INIT_SWB(1,   P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
		P7SPI_INIT_SWB(0,   P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
		P7SPI_INIT_SWB(3,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
		P7SPI_SWB_LAST,
	},
	[1] = { /* Master 1 */
		P7SPI_INIT_SWB(6,   P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
		P7SPI_INIT_SWB(5,   P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
		P7SPI_INIT_SWB(4,   P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
		P7SPI_INIT_SWB(7,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
		P7SPI_SWB_LAST,
	},
	[2] = { /* Master 2 */
		P7SPI_INIT_SWB(10,  P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
		P7SPI_INIT_SWB(9,   P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
		P7SPI_INIT_SWB(8,   P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
		P7SPI_INIT_SWB(11,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
		P7SPI_SWB_LAST,
	},
	[3] = { /* Master 3 */
		P7SPI_INIT_SWB(14,  P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
		P7SPI_INIT_SWB(13,  P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
		P7SPI_INIT_SWB(12,  P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
		P7SPI_INIT_SWB(15,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
		P7SPI_SWB_LAST,
	}
};

#define NORDB_INIT_CDATA(_id)                   \
	{                                           \
		.half_duplex        = true,             \
		.read               = true,             \
		.write              = true,             \
		.xfer_mode          = P7SPI_SINGLE_XFER,\
		.fifo_wcnt          = 16,               \
		.thres_wcnt         = 12,               \
		.tsetup_ss_ns       = 5,                \
		.thold_ss_ns        = 5,                \
		.toffclk_ns         = 0,                \
		.toffspi_ns         = 0,                \
		.tcapture_delay_ns  = 7                 \
	}

static struct p7spi_ctrl_data nordb_cdata[] = {
	NORDB_INIT_CDATA(0),
	NORDB_INIT_CDATA(1),
	NORDB_INIT_CDATA(2),
	NORDB_INIT_CDATA(3)
};

#define NORDB_INIT_DEV(_id)  \
	P7_INIT_SPIM_SLAVE("m25p32", NULL, &nordb_cdata[_id], 50000000, SPI_MODE_0)

static struct spi_board_info nordb_devs[] __initdata = {
	NORDB_INIT_DEV(0),
	NORDB_INIT_DEV(1),
	NORDB_INIT_DEV(2),
	NORDB_INIT_DEV(3)
};

void __init nordb_probe(struct p7_board const* board)
{
	unsigned int    id = 0;
	int             err;

	for (; id < 4; id++) {
		p7_init_spim(id,
		             nordb_pins[id],
		             ARRAY_SIZE(nordb_pins[id]),
		             nordb_swb[id]);

		err = p7_init_spim_slave(id, &nordb_devs[id]);
		WARN(err,
		     "%s: failed to initialize serial NOR flash %d (%d)\n",
			 board->name,
			 id,
			 err);
	}
}
