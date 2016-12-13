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


#define P7DEV_SPIQUAD_BUS 2
#define P7DEV_SPIQUAD_NAME \
	P7DEV_BRD_NAME " Quad NOR"


static struct pinctrl_map p7dev_spiquad_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12), /* CLK */
	P7_INIT_PINMAP(P7_SPI_13), /* SS */
	P7_INIT_PINMAP(P7_SPI_14), /* MOSI */
	P7_INIT_PINMAP(P7_SPI_15), /* MISO */
};


static struct p7spi_swb const p7dev_spiquad_swb[] = {
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data p7dev_spiquad_cdata = {
	.half_duplex        = true,
	.read               = true,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.fifo_wcnt          = 16,
	.thres_wcnt         = 12,
	.tsetup_ss_ns       = 5,
	.thold_ss_ns        = 5,
	.toffclk_ns         = 0,
	.toffspi_ns         = 30,
	.tcapture_delay_ns  = 7
};

static P7_DECLARE_SPIM_SLAVE(p7dev_spiquad_dev,
                             "a25lq032",
                             NULL,
                             &p7dev_spiquad_cdata,
                             50000000,
                             SPI_MODE_0);

void __init spiquadb_probe(struct p7_board const* board)
{
	int err;

	p7_init_spim(P7DEV_SPIQUAD_BUS,
	             p7dev_spiquad_pins,
	             ARRAY_SIZE(p7dev_spiquad_pins),
	             p7dev_spiquad_swb);

	err = p7_init_spim_slave(P7DEV_SPIQUAD_BUS, &p7dev_spiquad_dev);

	WARN(err,
	     "%s: failed to initialize quad NOR flash (%d)\n",
	     board->name, err);
}
