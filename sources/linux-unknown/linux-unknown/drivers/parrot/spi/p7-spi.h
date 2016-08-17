/**
 * linux/drivers/parrot/spi/p7-spi.h - Parrot7 SPI shared resources
 *                                     driver interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    04-Jul-2012
 *
 * This file is released under the GPL
 */

#ifndef _P7_SPI_H
#define _P7_SPI_H

#include <linux/mutex.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/spi/spi.h>
#include <spi/p7-spi_regs.h>
#include "p7-swb.h"

#define P7SPI_DRV_NAME  "p7-spi"

/**************
 * IP revisions
 **************/
enum spi_revision {
	SPI_REVISION_1  = 1,
	SPI_REVISION_2  = 2,
	SPI_REVISION_3  = 3,
	SPI_REVISION_NR,
};

/***********
 * Core type
 ***********/
enum p7spi_core_type {
	P7SPI_CORE_SPI,
	P7SPI_CORE_MPEG,
};


/****************
 * Transfer mode
 ****************/

enum p7spi_xfer_mode {
	P7SPI_SINGLE_XFER   = 0,
	P7SPI_DUAL_XFER     = 1,
	P7SPI_QUAD_XFER     = 3
};

/*************************************************************
 * Internal multiplexing handling: switchbox and pads control
 *************************************************************/

struct p7spi_swb {
	int                     pad;
	enum p7swb_direction    direction;
	enum p7swb_function     function;
};

#define P7SPI_INIT_SWB(_pad, _dir, _func) \
	{ .pad = _pad, .direction = _dir, .function = _func }
#define P7SPI_SWB_LAST \
	{ .pad = - 1 }

/**
 * struct p7spi_ctrl_data - Parrot7 SPI controller per device platform
 *                          settings
 * @half_duplex:        self explanatory
 * @read:               controller can receive data
 * @write:              controller can send data
 * @xfer_mode:          transfer mode (one of %P7SPI_SINGLE_XFER,
 *                      %P7SPI_DUAL_XFER or %P7SPI_QUAD_XFER)
 * @swb:                switchbox settings (see &struct p7spi_swb)
 * @thres_wcnt:         FIFO threshold in number of 32 bits words
 * @tsetup_ss_ns:       slave select setup time (nanoseconds)
 * @thold_ss_ns:        slave select hold time (nanoseconds)
 * @toffclk_ns:         idle time between two bytes when the slave select is
 *                      maintained low (nanoseconds)
 * @toffspi_ns:         idle time between two bytes when the slave select is
 *                      maintained high (nanoseconds)
 * @tcapture_delay_ns:  delay between the capture edge of SPI clock sent to the
 *                      slave and the capture the incoming data (nanoseconds)
 */
struct p7spi_ctrl_data {
	bool                    half_duplex;
	bool                    read;
	bool                    write;
	enum p7spi_xfer_mode    xfer_mode;
	struct p7spi_swb const* swb;
	size_t                  fifo_wcnt;
	size_t                  thres_wcnt;
	u32                     tsetup_ss_ns;
	u32                     thold_ss_ns;
	u32                     toffclk_ns;
	u32                     toffspi_ns;
	u32                     tcapture_delay_ns;
};


/*
 * struct p7spis_ctrl_data - Parrot7 SPI controller for slave devices
 * 
 * @common:             common settings for slave and master
 * @circ_buf_period:    circular buffer period length
 * @periods:            number of periods
 */
struct p7spis_ctrl_data {
	struct p7spi_ctrl_data  common;
	size_t                  circ_buf_period;
	u32                     periods;
};

/***************************
 * Internal FIFOs handling.
 ***************************/

struct p7spi_plat_data {
	unsigned int    wcnt;
	unsigned int    max_master_hz;
	unsigned int    max_slave_hz;
	unsigned int    min_hz;
	unsigned int    rev;

	/* variables to handle switchbox */
	unsigned int            num_pads;
	enum p7spi_core_type*   cores;
	unsigned int            num_cores;
};

#define P7SPI_MEM_WCNT(_core, _cnt)    \
	(P7SPI_MEM_ ## _cnt ## W << (_core * P7SPI_MEM_BITLEN))


#endif
