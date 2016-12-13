/*
 * linux/arch/arm/mach-parrot7/board-p7dev-spi.c - SPI master and slave board(s)
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Damien Riegel <damien.riegel.ext@parrot.com>
 * date:    30-04-2013
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "pinctrl.h"
#include "spi.h"
#include "board.h"



#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI
#include <spi/p7-spis.h>
#define P7DEV_SPI_MASTER_TEST_BUS   2
#define P7DEV_SPI_SLAVE_TEST_BUS    3

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_J550
/************************ slave ************************/
static struct pinctrl_map p7dev_spi_slave_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_09), /* MOSI */
	P7_INIT_PINMAP(P7_SPI_11), /* MISO */
	P7_INIT_PINMAP(P7_SPI_13), /* CLK */
	P7_INIT_PINMAP(P7_SPI_15), /* SS */
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD
	P7_INIT_PINMAP(P7_SPI_03),
	P7_INIT_PINMAP(P7_SPI_01),
#endif
};

static struct p7spi_swb const p7dev_spi_slave_swb[] = {
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_IN,      P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,      P7_SWB_SPI_CLK),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_SINGLE)
	P7SPI_INIT_SWB(11,  P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(9,   P7_SWB_DIR_IN,      P7_SWB_SPI_DATA0),
#else
	P7SPI_INIT_SWB(9,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(11,  P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA1),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
	P7SPI_INIT_SWB(1,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA2),
	P7SPI_INIT_SWB(3,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA3),
#endif
#endif
	P7SPI_SWB_LAST,
};


/************************ master ************************/
static struct pinctrl_map p7dev_spi_master_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_08), /* MOSI */
	P7_INIT_PINMAP(P7_SPI_10), /* MISO */
	P7_INIT_PINMAP(P7_SPI_12), /* CLK */
	P7_INIT_PINMAP(P7_SPI_14), /* SS */
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD
	P7_INIT_PINMAP(P7_SPI_02),
	P7_INIT_PINMAP(P7_SPI_00),
#endif
};

static struct p7spi_swb const p7dev_spi_master_swb[] = {
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_OUT,     P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_OUT,     P7_SWB_SPI_CLK),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_SINGLE)
	P7SPI_INIT_SWB(8,   P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(10,  P7_SWB_DIR_IN,      P7_SWB_SPI_DATA0),
#else
	P7SPI_INIT_SWB(8,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(10,  P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA1),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
	P7SPI_INIT_SWB(0,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA2),
	P7SPI_INIT_SWB(2,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA3),
#endif
#endif
	P7SPI_SWB_LAST,
};
#endif /* CONFIG_ARCH_PARROT7_P7DEV_SPI_J550 */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_J1030
/************************ slave ************************/
static struct pinctrl_map p7dev_spi_slave_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12), /* CLK */
	P7_INIT_PINMAP(P7_SPI_13), /* SS */
	P7_INIT_PINMAP(P7_SPI_14), /* MOSI/Data0 */
	P7_INIT_PINMAP(P7_SPI_15), /* MISO/Data1 */
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD
	P7_INIT_PINMAP(P7_SPI_04),
	P7_INIT_PINMAP(P7_SPI_05),
#endif
};

static struct p7spi_swb const p7dev_spi_slave_swb[] = {
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,      P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,      P7_SWB_SPI_SS),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_SINGLE)
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,      P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
#else
    /* common PADs in DUAL and QUAD modes */
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA1),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
	P7SPI_INIT_SWB(4,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA2),
	P7SPI_INIT_SWB(5,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA3),
#endif
#endif
	P7SPI_SWB_LAST,
};

/************************ master ************************/
static unsigned long p7dev_spi_master_cfg[] = {
	P7CTL_DRV_CFG(1)
};

static struct pinctrl_map p7dev_spi_master_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12), /* CLK */
	P7_INIT_PINCFG(P7_SPI_12, p7dev_spi_master_cfg),
	P7_INIT_PINMAP(P7_SPI_13), /* SS */
	P7_INIT_PINCFG(P7_SPI_13, p7dev_spi_master_cfg),
	P7_INIT_PINMAP(P7_SPI_14), /* MOSI */
	P7_INIT_PINCFG(P7_SPI_14, p7dev_spi_master_cfg),
	P7_INIT_PINMAP(P7_SPI_15), /* MISO */
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_DUAL) || \
	defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
	P7_INIT_PINCFG(P7_SPI_15, p7dev_spi_master_cfg),
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD
	P7_INIT_PINMAP(P7_SPI_04),
	P7_INIT_PINCFG(P7_SPI_04, p7dev_spi_master_cfg),
	P7_INIT_PINMAP(P7_SPI_05),
	P7_INIT_PINCFG(P7_SPI_05, p7dev_spi_master_cfg),
#endif
};

static struct p7spi_swb const p7dev_spi_master_swb[] = {
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_OUT,     P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_OUT,     P7_SWB_SPI_CLK),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_SINGLE)
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_OUT,     P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_IN,      P7_SWB_SPI_DATA0),
#else
    /* common PADs in DUAL and QUAD modes */
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA1),
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
	P7SPI_INIT_SWB(4,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA2),
	P7SPI_INIT_SWB(5,   P7_SWB_DIR_BOTH,    P7_SWB_SPI_DATA3),
#endif
#endif
	P7SPI_SWB_LAST,
};
#endif /* CONFIG_ARCH_PARROT7_P7DEV_SPI_J1030 */

static struct p7spis_ctrl_data p7dev_spi_slave_cdata = {
	.common = {
		.half_duplex        = true,
		.read               = true,
		.write              = true,
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
		.xfer_mode          = P7SPI_QUAD_XFER,
#elif defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_DUAL)
		.xfer_mode          = P7SPI_DUAL_XFER,
#else
		.xfer_mode          = P7SPI_SINGLE_XFER,
#endif
		.fifo_wcnt          = 16,
		.thres_wcnt         = 8,
		.tsetup_ss_ns       = 1,
		.thold_ss_ns        = 1,
		.toffclk_ns         = 1,
		.toffspi_ns         = 1,
		.tcapture_delay_ns  = 0,
	},
	.circ_buf_period        = 64 * 1024,
	.periods                = 4,
};

static P7_DECLARE_SPIS_MASTER(p7dev_spi_master_info,
		"spistest",
		NULL,
		&p7dev_spi_slave_cdata,
		10 * 1000 * 1000,
		SPI_MODE_0);


static struct p7spi_ctrl_data p7dev_spi_master_cdata = {
	.half_duplex        = true,
	.read               = true,
	.write              = true,
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_QUAD)
	.xfer_mode          = P7SPI_QUAD_XFER,
#elif defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_DUAL)
	.xfer_mode          = P7SPI_DUAL_XFER,
#else
	.xfer_mode          = P7SPI_SINGLE_XFER,
#endif
	.fifo_wcnt          = 16,
	.thres_wcnt         = 8,
	.tsetup_ss_ns       = 1,
	.thold_ss_ns        = 1,
	.toffclk_ns         = 1,
	.toffspi_ns         = 1,
	.tcapture_delay_ns  = 0,
};

static P7_DECLARE_SPIM_SLAVE(p7dev_spi_slave_info,
		"spidev",
		NULL,
		&p7dev_spi_master_cdata,
		10 * 1000 * 1000,
		SPI_MODE_0);

void __init spisdb_probe(struct p7_board const* brd)
{
	p7_init_spis(P7DEV_SPI_SLAVE_TEST_BUS,
			p7dev_spi_slave_pins,
			ARRAY_SIZE(p7dev_spi_slave_pins),
			p7dev_spi_slave_swb);

	if (p7_init_spis_master(P7DEV_SPI_SLAVE_TEST_BUS,
				&p7dev_spi_master_info))
		pr_err("%s: failed to initialize SPI slave.\n", brd->name);
}

void __init spimdb_probe(struct p7_board const* brd)
{
	p7_init_spim(P7DEV_SPI_MASTER_TEST_BUS,
			p7dev_spi_master_pins,
			ARRAY_SIZE(p7dev_spi_master_pins),
			p7dev_spi_master_swb);

	if (p7_init_spim_slave(P7DEV_SPI_MASTER_TEST_BUS,
				&p7dev_spi_slave_info))
		pr_err("%s: failed to initialize SPI master.\n", brd->name);
}

void __init spidb_probe(struct p7_board const* brd)
{
	spisdb_probe(brd);
	spimdb_probe(brd);
}
#else

static inline void spisdb_probe(struct p7_board const* brd)
{
	return;
}

static inline void spimdb_probe(struct p7_board const* brd)
{
	return;
}

static inline void spidb_probe(struct p7_board const* brd)
{
	return;
}

#endif
