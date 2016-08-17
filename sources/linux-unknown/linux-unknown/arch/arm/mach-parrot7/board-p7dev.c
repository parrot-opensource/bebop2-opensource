/**
 * linux/arch/arm/mach-parrot7/p7dev-board.c - Parrot7 development board platform
 *                                             implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    28-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
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
#include <mach/ether.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "pinctrl.h"
#include "spi.h"
#include "usb.h"
#include "venc.h"
#include "vdec.h"
#include "aai.h"
#include "gpu.h"
#include "board.h"
#include "backlight.h"
#include "board-common.h"
#include "aai.h"
#include "p7_temperature.h"
#include "wl18xx.h"
#include "crypto.h"
#include "usb.h"
#include "nand.h"

#define P7DEV_BRD_NAME   "p7dev"

int p7dev_rev = 1;  //Default HW : 01

/* On HW01, microsd can be used only without mother board and daughter board
 * modified. */
int use_microsd = 0;

/* conn id is the connector identifier */
P7_DEFINE_BOARD_CONN(p7dev_j100_conn, 100);
P7_DEFINE_BOARD_CONN(p7dev_j550_conn, 550);
P7_DEFINE_BOARD_CONN(p7dev_j801_conn, 801);
P7_DEFINE_BOARD_CONN(p7dev_j1030_conn, 1030);
P7_DEFINE_BOARD_CONN(p7dev_j2000_conn, 2000);

#ifdef CONFIG_ARCH_PARROT7_P7DEV_NOR
P7_DEFINE_BOARD(p7dev_nordb,
                "nor",
                &p7dev_j550_conn,
                nordb_probe,
                NULL);

static int p7dev_nordb_there(void)
{
	return p7_brd_there(&p7dev_nordb);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_NOR */
static bool p7dev_nordb_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_NOR */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_QUAD_NOR
P7_DEFINE_BOARD(p7dev_quadnordb,
                "quadnor",
                &p7dev_j1030_conn,
                spiquadb_probe,
                NULL);
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_FPGA
P7_DEFINE_BOARD(p7dev_fpgadb,
                "fpga",
                &p7dev_j100_conn,
                fpgadb_probe,
                fpgadb_rsvmem);
static int p7dev_fpgadb_there(void)
{
	return p7_brd_there(&p7dev_fpgadb);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_FPGA */
static bool p7dev_fpgadb_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_FPGA */


#ifdef CONFIG_ARCH_PARROT7_P7DEV_RNB6
P7_DEFINE_BOARD(p7dev_rnb6db,
                "rnb6",
                &p7dev_j100_conn,
                rnb6db_probe,
                rnb6db_rsvmem);
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_HDMI
P7_DEFINE_BOARD(p7dev_hdmidb,
                "hdmi",
                &p7dev_j100_conn,
                hdmidb_probe,
                hdmidb_rsvmem);
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_CAMERA
P7_DEFINE_BOARD(p7dev_cameradb,
                "camera",
                &p7dev_j100_conn,
                cameradb_probe,
                NULL);

static int p7dev_cameradb_there(void)
{
	return p7_brd_there(&p7dev_cameradb);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_CAMERA */
static bool p7dev_cameradb_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_CAMERA */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_GALILEO2
P7_DEFINE_BOARD(p7dev_galileo2db,
                "galileo2",
                &p7dev_j100_conn,
                galileo2db_probe,
                galileo2db_rsvmem);

static int p7dev_galileo2db_there(void)
{
	return p7_brd_there(&p7dev_galileo2db);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_GALILEO2 */
#if 0
static bool p7dev_galileo2db_there(void)
{
	return false;
}
#endif
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_GALILEO2 */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SICILIA
P7_DEFINE_BOARD(p7dev_siciliadb,
                "sicilia",
                &p7dev_j100_conn,
                siciliadb_probe,
                NULL);

static int p7dev_siciliadb_there(void)
{
	return p7_brd_there(&p7dev_siciliadb);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_SICILIA */
static bool p7dev_siciliadb_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_SICILIA */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_TUNER
static struct p7_board p7dev_tunerdb __initdata;

static int p7dev_tunerdb_there(void)
{
	return p7_brd_there(&p7dev_tunerdb);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_TUNER */
static bool p7dev_tunerdb_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_TUNER */

#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI)
#if defined(CONFIG_ARCH_PARROT7_P7DEV_SPI_J1030)
P7_DEFINE_BOARD(p7dev_spimdb,
                "spim",
                &p7dev_j1030_conn,
                spimdb_probe,
                NULL);

P7_DEFINE_BOARD(p7dev_spisdb,
                "spis",
                &p7dev_j1030_conn,
                spisdb_probe,
                NULL);
#else
P7_DEFINE_BOARD(p7dev_spidb,
                "spi",
                &p7dev_j550_conn,
                spidb_probe,
                NULL);
#endif
#endif

#ifdef CONFIG_ARCH_PARROT7_P7DEV_SND_AAI
P7_DEFINE_BOARD(p7dev_aaidb,
		"aai",
		&p7dev_j550_conn,
		aaidb_probe,
		NULL);

static int p7dev_aaidb_there(void)
{
	return p7_brd_there(&p7dev_aaidb);
}
#else   /* CONFIG_ARCH_PARROT7_P7DEV_SND_AAI */
static bool p7dev_aaidb_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_SND_AAI */

#ifdef CONFIG_ARCH_PARROT7_P7DEV_WILINK8
static __init p7_init_brd_fn p7dev_init_wl18xx;

P7_DEFINE_BOARD(p7dev_wilink8db,
		"wilink8",
		/*  */
		&p7dev_j801_conn,
		p7dev_init_wl18xx,
		NULL);

static int p7dev_wilink8db_there(void)
{
	return p7_brd_there(&p7dev_wilink8db);
}
#else
static int p7dev_wilink8db_there(void)
{
	return false;
}
#endif  /* CONFIG_ARCH_PARROT7_P7DEV_WILINK8 */

static void __init p7dev_define_boards(void)
{
#ifdef CONFIG_ARCH_PARROT7_P7DEV_TUNER
	if (p7dev_rev == 0)
		p7_define_board(&p7dev_tunerdb,
				"tuner",
				&p7dev_j550_conn,
				tunerdb_probe,
				tunerdb_rsvmem);
	else
		p7_define_board(&p7dev_tunerdb,
				"tuner",
				&p7dev_j2000_conn,
				tunerdb_probe,
				tunerdb_rsvmem);
#endif /* CONFIG_ARCH_PARROT7_P7DEV_TUNER */
}

static struct p7_board* const p7dev_boards[] __initconst = {
#ifdef CONFIG_ARCH_PARROT7_P7DEV_NOR
	  &p7dev_nordb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_FPGA
	  &p7dev_fpgadb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_RNB6
	  &p7dev_rnb6db,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_HDMI
	  &p7dev_hdmidb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_CAMERA
	  &p7dev_cameradb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_GALILEO2
	  &p7dev_galileo2db,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SICILIA
	  &p7dev_siciliadb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SPI_J1030
	  &p7dev_spimdb,
	  &p7dev_spisdb,
#else
	  &p7dev_spidb,
#endif
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_QUAD_NOR
	  &p7dev_quadnordb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_SND_AAI
	  &p7dev_aaidb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_TUNER
	  &p7dev_tunerdb,
#endif
#ifdef CONFIG_ARCH_PARROT7_P7DEV_WILINK8
	  &p7dev_wilink8db,
#endif
};

static int __init p7dev_get_boards(char *options)
{
	char *opt;
	char opt2[strlen(options)];

	opt2[0] = '\0';

	/* Remove leading and trailing white spaces. */
	opt = strim(options);

	/* Now parse each comma separated token and init boards accordingly. */
	while (opt && *opt != '\0') {
		char*           o = strchr(opt, ',');

		if (o)
			*(o++) = '\0';

		if (!strcmp(opt, "hw00"))
			p7dev_rev = 0;
		else if (!strcmp(opt, "hw01"))
			p7dev_rev = 1;
		else if (!strcmp(opt, "microsd"))
			use_microsd = 1;
		else {
			strcat(opt2, opt);
			if (o && *o != '\0')
				strcat(opt2, ",");
		}

		/* Jump to next board token. */
		opt = o;
	}

	p7dev_define_boards();

	p7_setup_brd(p7dev_boards, ARRAY_SIZE(p7dev_boards), opt2);
	return 0;
}
early_param("p7dev_boards", p7dev_get_boards);

static unsigned long p7dev_aai_conf[] ={
	P7CTL_DRV_CFG(4),
};



static char * aai_dev_list[] =
{
	"music-out-stereo0",
	"music-out-stereo1",
	"music-out-stereo2",
	"music-out-stereo3",
	NULL,
};


static struct pinctrl_map p7dev_aai_pins[] __initdata = {

	P7_INIT_PINMAP(P7_AAI_00),
	P7_INIT_PINCFG(P7_AAI_00, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_01),
	P7_INIT_PINCFG(P7_AAI_01, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_02),
	P7_INIT_PINCFG(P7_AAI_02, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_03),
	P7_INIT_PINCFG(P7_AAI_03, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_05),
	P7_INIT_PINCFG(P7_AAI_05, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_09),
	P7_INIT_PINCFG(P7_AAI_09, p7dev_aai_conf),
	/*P7_INIT_PINMAP(P7_AAI_11),
	  P7_INIT_PINCFG(P7_AAI_11, p7dev_aai_conf),*/
	P7_INIT_PINMAP(P7_AAI_12),
	P7_INIT_PINCFG(P7_AAI_12, p7dev_aai_conf),
	/*P7_INIT_PINMAP(P7_AAI_13),
	  P7_INIT_PINCFG(P7_AAI_13, p7dev_aai_conf),*/
	P7_INIT_PINMAP(P7_AAI_14),
	P7_INIT_PINCFG(P7_AAI_14, p7dev_aai_conf),

	P7_INIT_PINMAP(P7_AAI_15),
	P7_INIT_PINCFG(P7_AAI_15, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_16),
	P7_INIT_PINCFG(P7_AAI_16, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_17),
	P7_INIT_PINCFG(P7_AAI_17, p7dev_aai_conf),
	P7_INIT_PINMAP(P7_AAI_23),
	P7_INIT_PINCFG(P7_AAI_23, p7dev_aai_conf),
};

static struct aai_pad_t p7dev_aai_pads[] =
{
    {AAI_SIG_MCLK,              0,  PAD_OUT},
    {AAI_SIG_MAIN_I2S_FRAME,    1,  PAD_OUT},
    {AAI_SIG_DAC_BIT_CLOCK,     2,  PAD_OUT},

    {AAI_SIG_PCM1_CLK,         11,  PAD_OUT},
    {AAI_SIG_PCM1_FRAME,       12,  PAD_OUT},
    {AAI_SIG_PCM1_OUT,         13,  PAD_OUT},
    {AAI_SIG_PCM1_IN,          14,  PAD_IN },

    /*{AAI_SIG_I2S0_IN,        21,  PAD_IN },*/
    {AAI_SIG_IN_MIC0,          5,  PAD_IN },
    {AAI_SIG_OUT_DAC0,         9,  PAD_OUT},

    {AAI_SIG_OUT_DAC1,         15,  PAD_OUT},
    {AAI_SIG_I2S1_IN,          16,  PAD_IN },

    {AAI_SIG_OUT_DAC2,         17,  PAD_OUT},
    {AAI_SIG_I2S2_IN,          18,  PAD_IN },

    {AAI_SIG_OUT_DAC3,         23,  PAD_OUT},
    {AAI_SIG_I2S3_IN,          24,  PAD_IN },
    {AAI_SIG_I2S0_IN, 	       16,  PAD_IN },

/*
    {AAI_SIG_TDM_OUT,          24,  PAD_OUT},
    {AAI_SIG_TDM_OUT_BIT_CLOCK,13,  PAD_OUT},
    {AAI_SIG_TDM_IN_BIT_CLOCK, 18,  PAD_OUT },
    {AAI_SIG_TDM_IN,	       3,   PAD_IN },
*/
    {AAI_SIG_SPDIF_TX,         10,  PAD_OUT},
    {AAI_SIG_SPDIF_RX,          6,  PAD_IN },

    {-1,                       -1,  0}
};

static struct aai_conf_set p7dev_aai_conf_set[] =
{
	/* This configuration is used to set
	 * music in channel MASTER or SLAVE
	 */
	{MUS_SYNC_DAC(0)},
	{MUS_SYNC_DAC(1)},
	{MUS_SYNC_DAC(2)},
	{MUS_SYNC_DAC(3)},
/*Don't remove*/
	{-1, 0, 0, 0},
};

static struct aai_platform_data p7dev_aai_pdata = {
	.pad = p7dev_aai_pads,
	.aai_conf = p7dev_aai_conf_set,
	.device_list = aai_dev_list,
};

static struct aai_pad_t p7dev_aai_pads_r2[] =
{
	{AAI_SIG_MCLK,               11,  PAD_OUT},
	{AAI_SIG_MAIN_I2S_FRAME,     12,  PAD_OUT},
	{AAI_SIG_DAC_BIT_CLOCK,      13,  PAD_OUT},

	{AAI_SIG_PCM1_CLK,	      0,  PAD_IN },
	{AAI_SIG_PCM1_FRAME,          1,  PAD_IN },
	{AAI_SIG_PCM1_IN,             2,  PAD_IN },
	{AAI_SIG_PCM1_OUT,            3,  PAD_OUT},

	{AAI_SIG_IN_MIC0,           5,  PAD_IN },
	{AAI_SIG_OUT_DAC0,          9,  PAD_OUT},

	{AAI_SIG_OUT_DAC1,         15,  PAD_OUT},
	{AAI_SIG_I2S1_IN,          16,  PAD_IN },

	{AAI_SIG_OUT_DAC2,         17,  PAD_OUT},
	{AAI_SIG_I2S2_IN,          18,  PAD_IN },

	{AAI_SIG_OUT_DAC3,         23,  PAD_OUT},
	{AAI_SIG_I2S3_IN,          24,  PAD_IN },
	{AAI_SIG_I2S0_IN,          16,  PAD_IN },

	/*
	 * {AAI_SIG_TDM_OUT,          24,  PAD_OUT},
	 * {AAI_SIG_TDM_OUT_BIT_CLOCK,13,  PAD_OUT},
	 * {AAI_SIG_TDM_IN_BIT_CLOCK, 18,  PAD_OUT },
	 * {AAI_SIG_TDM_IN,           3,   PAD_IN },
	 */
	{AAI_SIG_SPDIF_TX,         10,  PAD_OUT},
	{AAI_SIG_SPDIF_RX,          6,  PAD_IN },

	{-1,                       -1,  0}
};

static struct aai_platform_data p7dev_aai_pdata_r2 = {
	.pad = p7dev_aai_pads_r2,
	.aai_conf = p7dev_aai_conf_set,
	.device_list = aai_dev_list,
};



struct pinctrl_map *p7dev_aai_pin_ptr __initdata = p7dev_aai_pins;
unsigned long p7_aai_pin_sz = ARRAY_SIZE(p7dev_aai_pins);

/*****************
 * GPIOs handling
 *****************/

#include "gpio.h"

#if defined(CONFIG_GPIO_PARROT7) || \
    defined(CONFIG_GPIO_PARROT7_MODULE)

#include <gpio/p7-gpio.h>

static unsigned const p7dev_irq_gpios[] = {
	73,   /* P7MU interrupt source */
	82,   /* SDHCI.1 card detect interrupt source */
};

#endif


/*****************************
 * P7MU Power Management Unit
 *****************************/

#include "p7mu.h"

#if defined(CONFIG_MFD_P7MU) || defined(CONFIG_MFD_P7MU_MODULE)
#include <mfd/p7mu.h>
#include <mach/p7-adc.h>
#include <mach/gpio.h>

static struct pinctrl_map p7dev_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

static struct p7_adc_chan p7mu_adc_channels[] = {
	{
		.type = P7MUADC_IIO_RING_BUFFER,
		.channel = 2,
		.freq = 80000,
		.samples = 1000,
	},
	{
		.type = P7MUADC_IIO_MARKER,
		.channel = 6,
		.freq = P7MUADC_MARKER_FREQ,
	}
};

static struct p7_adc_chan_data p7mu_adc_chan_data = {
        .channels               = p7mu_adc_channels,
        .num_channels           = ARRAY_SIZE(p7mu_adc_channels),
};

static struct p7mu_plat_data p7dev_p7mu_pdata = {
	.gpio       = P7_GPIO_NR(73),   /* GPIO 73 is P7MU -> P7 interrupt source */
	.pwm_pins   = {
		[1] = 3                     /* PWM1 multiplexed onto IO_4 pin. */
	},
	.int_32k    = false,            /* External 32kHz clock. */
	.int_32m    = false,             /* External 48mHz clock. */
	.chan_data = &p7mu_adc_chan_data,
};

#endif  /* defined(CONFIG_MFD_P7MU) || defined(CONFIG_MFD_P7MU_MODULE) */

/******************
 * SPI controllers
 ******************/

#if defined(CONFIG_SPI_MASTER_PARROT7) || \
    defined(CONFIG_SPI_MASTER_PARROT7_MODULE)
#include <spi/p7-spim.h>
#endif  /* defined(CONFIG_SPI_MASTER_PARROT7) || \
           defined(CONFIG_SPI_MASTER_PARROT7_MODULE) */

#if ((defined(CONFIG_MTD_M25P80) || \
     defined(CONFIG_MTD_M25P80_MODULE)) && \
	(defined(CONFIG_SPI_MASTER_PARROT7) || \
	 defined(CONFIG_SPI_MASTER_PARROT7_MODULE))) && \
		!defined(CONFIG_MTD_NAND_CAST)
#include <linux/spi/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#define P7DEV_SPINOR_BUS 0
#define P7DEV_SPINOR_NAME \
	P7DEV_BRD_NAME " serial NOR"

static struct mtd_partition p7dev_spinor_parts[] = {
	{
		.name   = P7DEV_SPINOR_NAME " [0]",
		.offset = 0,
		.size   = 512 * SZ_1K,
	},
	{
		.name   = P7DEV_SPINOR_NAME " [1]",
		.offset = MTDPART_OFS_NXTBLK,
		.size   = MTDPART_SIZ_FULL
	}
};

static struct flash_platform_data const p7dev_spinor_pdata = {
	.parts      = p7dev_spinor_parts,
	.nr_parts   = ARRAY_SIZE(p7dev_spinor_parts)
};

static unsigned long p7dev_spinor_cmdcfg[] = {
	P7CTL_DRV_CFG(3)
};

static unsigned long p7dev_spinor_datcfg[] = {
	P7CTL_PUD_CFG(DOWN),
	P7CTL_DRV_CFG(4)
};

static struct pinctrl_map p7dev_spinor_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_01a), /* SS */
	P7_INIT_PINCFG(P7_SPI_01a, p7dev_spinor_cmdcfg),
	P7_INIT_PINMAP(P7_SPI_02a), /* CLK */
	P7_INIT_PINCFG(P7_SPI_02a, p7dev_spinor_cmdcfg),
	P7_INIT_PINMAP(P7_SPI_03a), /* MOSI */
	P7_INIT_PINCFG(P7_SPI_03a, p7dev_spinor_datcfg),
	P7_INIT_PINMAP(P7_SPI_04a), /* MISO */
	P7_INIT_PINCFG(P7_SPI_04a, p7dev_spinor_datcfg)
};

/*
 * Single SPI half-duplex mode SPI controller internal multiplexing setup:
 * the same line (DATA0) is used alternativly for MISO and MOSI functions.
 */
static struct p7spi_swb const p7dev_spinor_swb[] = {
	P7SPI_INIT_SWB(1, P7_SWB_DIR_OUT,   P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(2, P7_SWB_DIR_OUT,   P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(3, P7_SWB_DIR_OUT,   P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(4, P7_SWB_DIR_IN,    P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data p7dev_spinor_cdata = {
	.half_duplex        = true,
	.read               = true,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.fifo_wcnt          = 16,
	.thres_wcnt         = 12,
	.tsetup_ss_ns       = 5,
	.thold_ss_ns        = 5,
	.toffclk_ns         = 0,
	.toffspi_ns         = 0,
	.tcapture_delay_ns  = 7
};

static P7_DECLARE_SPIM_SLAVE(p7dev_spinor_dev,
                             "m25p32",
                             &p7dev_spinor_pdata,
                             &p7dev_spinor_cdata,
                             50000000,
                             SPI_MODE_0);

static void __init p7dev_init_spinor(void)
{
	int err;

	p7_init_spim(P7DEV_SPINOR_BUS,
	             p7dev_spinor_pins,
	             ARRAY_SIZE(p7dev_spinor_pins),
	             p7dev_spinor_swb);

	err = p7_init_spim_slave(P7DEV_SPINOR_BUS, &p7dev_spinor_dev);

	WARN(err,
	     P7DEV_BRD_NAME ": failed to initialize serial NOR flash (%d)\n",
		 err);
}

#else

static inline void __init p7dev_init_spinor(void)
{
	return;
}

#endif  /* ((defined(CONFIG_MTD_M25P80) || \
             defined(CONFIG_MTD_M25P80_MODULE)) && \
            (defined(CONFIG_SPI_MASTER_PARROT7) || \
             defined(CONFIG_SPI_MASTER_PARROT7_MODULE))) &&
	    !defined(CONFIG_MTD_NAND_CAST) */

#if defined(CONFIG_P7MU_ADC) || defined(CONFIG_P7MU_ADC_MODULE)
#define P7DEV_SPI_SLAVE_P7MU    2

static struct pinctrl_map p7dev_spi_slave_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_13b), /* SS */
	P7_INIT_PINMAP(P7_SPI_12b), /* CLK */
	P7_INIT_PINMAP(P7_SPI_14b), /* MOSI */
};

static struct p7spi_swb const p7dev_spi_slave_p7mu_swb[] = {
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};


static struct p7spis_ctrl_data p7dev_spi_slave_p7mu_cdata = {
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
	}
};

static P7_DECLARE_SPIS_MASTER(p7dev_spi_master_p7mu_info,
		"p7mu-adc",
		NULL,
		&p7dev_spi_slave_p7mu_cdata,
		10 * 1000 * 1000,
		SPI_MODE_0|SPI_LSB_FIRST);


static void __init p7dev_init_spi_p7mu(void)
{
	p7_init_spis(P7DEV_SPI_SLAVE_P7MU,
			p7dev_spi_slave_p7mu_pins,
			ARRAY_SIZE(p7dev_spi_slave_p7mu_pins),
			p7dev_spi_slave_p7mu_swb);

	if (p7_init_spis_master(P7DEV_SPI_SLAVE_P7MU,
				&p7dev_spi_master_p7mu_info))
		pr_err(P7DEV_BRD_NAME ": failed to initialize SPI slave.\n");
}
#else

static inline void __init p7dev_init_spi_p7mu(void)
{
	return;
}

#endif

/***********************
 * SDHCI hosts handling
 ***********************/

#include "sdhci.h"

#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>
#include "gbc.h"

/*
 * microSD on MMC0: as indicated in P7 CPU daughter board's schematics
 * clock frequency is limited to 25MHz.
 */
static struct acs3_mode_config p7dev_sdhci0_25MHz = {
	.mode   = ACS3_HIGH_SPEED,
	.max_Hz = 25 * 1000 * 1000,
	.tdl1   = 0x31f,
	.tdl2   = 0,
};

static __maybe_unused struct acs3_plat_data p7dev_sdhci0_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
	.mode_config = &p7dev_sdhci0_25MHz,
	.nb_config   = 1,
};

static __maybe_unused struct acs3_regulator_gpios p7dev_sdhci1_regulator_gpios = {
	.bus_1v8_gpio = 83,                             /* GPIO 83 is 1.8V switch
	                                                 * command */
	.bus_1v8_active_low = 0,                        /* 1.8V switch is active high */
};

static __maybe_unused struct acs3_plat_data p7dev_sdhci1_pdata = {
	.led_gpio   = 68,                               /* GPIO 68 drives activity led */
	.wp_gpio    = 81,                               /* GPIO 81 is write protect
	                                                 * status */
	.cd_gpio    = 82,                               /* GPIO 82 is card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

static __maybe_unused struct acs3_plat_data p7dev_sdhci2_pdata = {
	.led_gpio   = -1,                               /* No activity led */
	.wp_gpio    = -1,                               /* No write protect */
	.cd_gpio    = -1,                               /* No card detect */
	.rst_gpio   = -1,                               /* No eMMC hardware reset */
	.brd_ocr    = MMC_VDD_32_33 | MMC_VDD_33_34 |   /* 3.3V ~ 3.0V card Vdd only */
	              MMC_VDD_29_30 | MMC_VDD_30_31,
	.mmc_caps   = 0, .mmc_caps2  = 0,               /* No specific capability */
};

/*************
 * I2C EEPROM
 *************/

#if defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE)

#include <linux/i2c/at24.h>

static struct at24_platform_data p7dev_eeprom_pdata = {
	.byte_len  = SZ_64K / 8,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
};

static struct i2c_board_info p7dev_eeprom_board_info = {
	I2C_BOARD_INFO("at24", 0x50), /* E0 = E1 = E2 = 0 */
	.platform_data = &p7dev_eeprom_pdata,
};

static inline void p7dev_register_eeprom(void)
{
	/* EEPROM is connected to I2C-0 */
	parrot_init_i2c_slave(0, &p7dev_eeprom_board_info, "I2C EEPROM", P7_I2C_NOIRQ);
}
#else /* defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE) */

static inline void p7dev_register_eeprom(void)
{
	return;
}

#endif /* defined(CONFIG_EEPROM_AT24) || defined(CONFIG_EEPROM_AT24_MODULE) */

/**********************
 * TI wl18xx wlan
 **********************/
static __initdata struct wl18xx_resources p7dev_wl18xx_res_hw00 = {
	.wlirq_gpio = P7_GPIO_NR(67),
	.wlen_gpio = P7MU_IO_NR(5),
	.bt_rst_gpio = P7MU_IO_NR(2),
	.bt_uart_slot = 1,
	.wl_sdhci_slot = 0,
};

static __initdata struct wl18xx_resources p7dev_wl18xx_res_hw01 = {
	.wlirq_gpio = P7_GPIO_NR(67),
	.wlen_gpio = P7MU_IO_NR(5),
	.bt_rst_gpio = P7MU_IO_NR(3),
	.bt_uart_slot = 1,
	.wl_sdhci_slot = 0,
};

static void __init p7dev_init_wl18xx(struct p7_board const *board)
{
	if (p7dev_rev == 0)
		init_wl18xx(&p7dev_wl18xx_res_hw00, NULL, 0);
	else
		init_wl18xx(&p7dev_wl18xx_res_hw01, NULL, 0);
}

static void __init p7dev_reserve_mem(void)
{
#define P7DEV_HX280_SIZE (CONFIG_ARCH_PARROT7_P7DEV_HX280_SIZE * SZ_1M)
	p7_reserve_vencmem(P7DEV_HX280_SIZE);
#define P7DEV_HX270_SIZE (CONFIG_ARCH_PARROT7_P7DEV_HX270_SIZE * SZ_1M)
	p7_reserve_vdecmem(P7DEV_HX270_SIZE);

	p7_reserve_brdmem(p7dev_boards, ARRAY_SIZE(p7dev_boards));

	p7_reserve_nand_mem();

	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);

	p7_reserve_dmamem();
}

static void __init p7dev_init_mach(void)
{
	p7_init_mach();

	p7_init_gpio(p7dev_irq_gpios,
	             ARRAY_SIZE(p7dev_irq_gpios));

	p7brd_init_i2cm(0, 100);
	/* I2CM2: iPod ACP, GPIO expanders */
	p7brd_init_i2cm(1, 100);

	p7_init_p7mu(0,
		     &p7dev_p7mu_pdata,
		     p7dev_p7mu_pins,
		     ARRAY_SIZE(p7dev_p7mu_pins));

	p7_probe_brd(p7dev_boards, ARRAY_SIZE(p7dev_boards));

	/*
	 * Write protect pin is shared by NAND and NOR flashes. It is active
	 * low for both devices: enable writes.
	 * Don't reserve WP pin usage so that userland may use both devices
	 * alternatively.
	 */
	if (gpio_request_one(42, GPIOF_OUT_INIT_HIGH, P7DEV_BRD_NAME " flash wp"))
		pr_warn(P7DEV_BRD_NAME ": failed to disable NAND / NOR flash write protect\n");

	p7brd_init_uart(0,1);
	if (p7dev_rev == 0 && !p7dev_wilink8db_there())
		p7brd_init_uart(1,1);
	p7brd_init_uart(2,1);

	if (! p7dev_nordb_there() && ! p7dev_tunerdb_there())
		p7dev_init_spinor();

	p7dev_init_spi_p7mu();
        p7_init_temperature();

	if (!p7dev_fpgadb_there())
		p7brd_init_nand(1);

	if (use_microsd && !p7dev_wilink8db_there())
		p7brd_init_sdhci(0, &p7dev_sdhci0_pdata, NULL, NULL, NULL,
				 NULL, 0);
	if (p7dev_tunerdb_there() || p7dev_aaidb_there()) {
		/* The GPIO 81, 82 & 83 are shared with AAI or SPI pins
		 * of tuner & aai daughter boards */
		p7dev_sdhci1_regulator_gpios.bus_1v8_gpio = 0;
		p7dev_sdhci1_pdata.wp_gpio = -1;
		p7dev_sdhci1_pdata.cd_gpio = -1;
	}
	p7brd_init_sdhci(1, &p7dev_sdhci1_pdata, NULL, NULL,
			 &p7dev_sdhci1_regulator_gpios, NULL, 0);
	if (!p7dev_cameradb_there() &&
	    !p7dev_fpgadb_there() &&
	    !p7dev_siciliadb_there())
		/* SD2 pins conflict with CAM5 ones */
		p7brd_init_sdhci(2, &p7dev_sdhci2_pdata, NULL, NULL, NULL,
				 NULL, 0);


#if 0

	/*
	 * USB0 role is host only so that we may use first controller with daughter
	 * boards (pins conflict with I2CM2).
	 * Role can forced to device by cmdline option passed by installer.
	 */
	if (parrot_force_usb_device)
		p7brd_init_udc(0, P7_GPIO_NR(72));
	else
		p7brd_init_usb(0, P7_GPIO_NR(72), CI_UDC_DR_DUAL_HOST);

	if (!p7dev_cameradb_there() &&
	    !p7dev_fpgadb_there() &&
	    !p7dev_galileo2db_there()) {
		/*
		 * I2C muxed buses are used by camera daughter board and conflict with
		 * USB1 pins. USB1 role is host only.
		 * A single gpio is used to drive VBUS and it is shared with USB0
		 * controller. Leave VBUS management reponsability to USB0 just above.
		 */
		p7brd_init_hcd(1, -1);
	}

#endif
	p7_init_venc();
	p7_init_vdec();
	if (p7dev_rev == 0)
		p7_init_aai(p7dev_aai_pin_ptr, p7_aai_pin_sz, &p7dev_aai_pdata);
	else if (p7dev_rev == 1)
		p7_init_aai(p7dev_aai_pin_ptr, p7_aai_pin_sz, &p7dev_aai_pdata_r2);

	p7dev_register_eeprom();

	/*
	 * Init Wilink8 on HW01
	 */
	if ((p7dev_rev > 0) && !p7dev_wilink8db_there() && !use_microsd &&
	    !p7dev_tunerdb_there())
		p7dev_init_wl18xx(NULL);

	/*
	 * On P7DEV HW01 board, gpio P7MU_IO_NR(5) is used to de-reset several
	 * devices.
	 */
	gpio_request_one(P7MU_IO_NR(5), GPIOF_OUT_INIT_HIGH, "De-Reset Multi devices");

	if (p7dev_rev > 0)
		p7_init_ether(PHY_IFACE_RGMII, -1, P7CTL_DRV_CFG(5));

#if (defined(CONFIG_CAN_C_CAN_PLATFORM) || \
		defined(CONFIG_CAN_C_CAN_PLATFORM_MODULE)) && \
		!defined(CONFIG_SND_AAI) && \
		!defined(CONFIG_SND_AAI_MODULE) && \
		!defined(CONFIG_WILINK_PLATFORM_DATA)
	gpio_request_one(P7_GPIO_NR(66), GPIOF_OUT_INIT_HIGH, "CAN en");
	gpio_request_one(P7_GPIO_NR(67), GPIOF_IN, "CAN err");
	gpio_request_one(P7_GPIO_NR(84), GPIOF_OUT_INIT_HIGH, "CAN stb");
	p7brd_init_ccan(0);
	p7brd_init_ccan(1);
#endif

	p7_init_crypto();
}

#ifdef CONFIG_SERIAL_PARROTX_CONSOLE
#include <linux/console.h>

static int __init p7dev_init_cons(void)
{
	if (machine_desc->nr == MACH_TYPE_PARROT_P7DEV)
		return add_preferred_console("ttyPA", 0, "115200n8");

	return 0;
}
console_initcall(p7dev_init_cons);

#endif

P7_MACHINE_START(PARROT_P7DEV, "Parrot7 development board")
	.reserve        = &p7dev_reserve_mem,
	.init_machine   = &p7dev_init_mach,
P7_MACHINE_END
