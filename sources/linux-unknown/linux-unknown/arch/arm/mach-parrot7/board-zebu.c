/*
 *	linux/arch/arm/mach-parrot7/board-zebu.c
 *
 *	Copyright (C) 2010 Parrot S.A.
 *
 * @author	Gregor Boirie <gregor.boirie@parrot.com>
 * @date  28-Oct-2010
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <spi/p7-spi.h>
#include "common.h"
#include "pinctrl.h"
#include "venc.h"
#include "vdec.h"
#include "avi.h"
#include "lcd-monspecs.h"
#include "iopin.h"
#include "clock.h"
#include "board-common.h"
#include "gpu.h"
#include "spi.h"
#include "nand.h"

#define P7ZB_BRD_NAME   "p7zebu"

/*******************
 * Physical ramdisk
 *******************/

#if defined(CONFIG_BLK_DEV_PHYBRD) || \
    defined(CONFIG_BLK_DEV_PHYBRD_MODULE)

#include <block/phy-brd.h>
#include "common.h"

static struct resource zebu_brd_res = {
	.start = P7_SPI0,
	.end   = P7_SPI0 + SZ_4K - 1,
	.flags = IORESOURCE_MEM
};

static struct platform_device zebu_brd_dev = {
	.name           = PHYBRD_DRV_NAME,
	.id             = 0,
	.num_resources  = 1,
	.resource       = &zebu_brd_res
};

void __init zebu_reserve_brdmem(size_t size)
{
	p7_reserve_devmem(&zebu_brd_dev,
	                  &zebu_brd_res.start,
	                  &size);
	zebu_brd_res.end = zebu_brd_res.start + size - 1;
}

void __init zebu_init_brd(void)
{
	p7_init_dev(&zebu_brd_dev, NULL, NULL, 0);
}

#else   /* defined(CONFIG_BLK_DEV_PHYBRD) || \
           defined(CONFIG_BLK_DEV_PHYBRD_MODULE) */

#define zebu_reserve_brdmem(_size)
#define zebu_init_brd()

#endif  /* defined(CONFIG_BLK_DEV_PHYBRD) || \
           defined(CONFIG_BLK_DEV_PHYBRD_MODULE) */

/***********************
 * SDHCI hosts handling
 ***********************/

#include "sdhci.h"

#if defined(CONFIG_MMC_SDHCI_ACS3) || \
    defined(CONFIG_MMC_SDHCI_ACS3_MODULE)

#include <linux/mmc/host.h>
#include <mmc/acs3-sdhci.h>

static struct acs3_plat_data zebu_sdhci0_pdata = {
	.led_gpio       = -1,                           /* No activity led GPIO */
	.wp_gpio        = -1,                           /* No write protect GPIO */
	.cd_gpio        = -1,                           /* No card detect GPIO */
	.rst_gpio       = -1,                           /* No eMMC hardware reset */
	.brd_ocr        = MMC_VDD_32_33 | MMC_VDD_33_34,/* 3.3V card Vdd only */
	.mmc_caps       = MMC_CAP_NONREMOVABLE,         /* non removable */
	.mmc_caps2      = 0,
};

#endif  /* defined(CONFIG_MMC_SDHCI_ACS3) || \
           defined(CONFIG_MMC_SDHCI_ACS3_MODULE) */


/*****************
 * GPIOs handling
 *****************/

#include "gpio.h"

#if defined(CONFIG_GPIO_PARROT7) || \
    defined(CONFIG_GPIO_PARROT7_MODULE)

#include <gpio/p7-gpio.h>

static unsigned const zebu_irq_gpios[] = {
	9,   /* P7MU interrupt source */
};

#endif

/*****************************
 * P7MU Power Management Unit
 *****************************/

#include "p7mu.h"

#if defined(CONFIG_MFD_P7MU) || defined(CONFIG_MFD_P7MU_MODULE)
#include <mfd/p7mu.h>

static struct pinctrl_map zebu_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
};

static struct p7mu_plat_data zebu_p7mu_pdata = {
	.gpio       = 9,    /* GPIO 9 is P7MU -> P7 interrupt source */
	.int_32k    = false,
	.int_32m    = true
};

#endif

/********************
 * I2C slave devices
 ********************/

#include "i2cs.h"

#if defined(CONFIG_PLDS_I2CS) || \
    defined(CONFIG_PLDS_I2CS_MODULE)

#include <i2c/p7-i2cs.h>

static struct p7i2cs_plat_data zebu_i2cs1_pdata = {
	.address = 0x11,
	.size    = SZ_256K
};

static struct pinctrl_map zebu_i2cs1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_1_SL_CLK),
	P7_INIT_PINMAP(P7_I2C_1_SL_DAT)
};

static struct p7i2cs_plat_data zebu_i2cs2_pdata = {
	.address = 0x12,
	.size    = SZ_256K
};

static struct pinctrl_map zebu_i2cs2_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_SL_CLK),
	P7_INIT_PINMAP(P7_I2C_2_SL_DAT)
};

#endif

/***********************
 * AVI configuration
 ***********************/

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)

/***************************
 * Framebuffer configuration
 ***************************/
static struct pinctrl_map zebu_avifb0_pins[] __initdata = {
	/* LCD0 related I/O pins */
	P7_INIT_PINMAP(P7_LCD_0_CLK),
	P7_INIT_PINMAP(P7_LCD_0_DATA00),
	P7_INIT_PINMAP(P7_LCD_0_DATA01),
	P7_INIT_PINMAP(P7_LCD_0_DATA02),
	P7_INIT_PINMAP(P7_LCD_0_DATA03),
	P7_INIT_PINMAP(P7_LCD_0_DATA04),
	P7_INIT_PINMAP(P7_LCD_0_DATA05),
	P7_INIT_PINMAP(P7_LCD_0_DATA06),
	P7_INIT_PINMAP(P7_LCD_0_DATA07),
	P7_INIT_PINMAP(P7_LCD_0_DATA08),
	P7_INIT_PINMAP(P7_LCD_0_DATA09),
	P7_INIT_PINMAP(P7_LCD_0_DATA10),
	P7_INIT_PINMAP(P7_LCD_0_DATA11),
	P7_INIT_PINMAP(P7_LCD_0_DATA12),
	P7_INIT_PINMAP(P7_LCD_0_DATA13),
	P7_INIT_PINMAP(P7_LCD_0_DATA14),
	P7_INIT_PINMAP(P7_LCD_0_DATA15),
	P7_INIT_PINMAP(P7_LCD_0_DATA16),
	P7_INIT_PINMAP(P7_LCD_0_DATA17),
	P7_INIT_PINMAP(P7_LCD_0_DATA18),
	P7_INIT_PINMAP(P7_LCD_0_DATA19),
	P7_INIT_PINMAP(P7_LCD_0_DATA20),
	P7_INIT_PINMAP(P7_LCD_0_DATA21),
	P7_INIT_PINMAP(P7_LCD_0_DATA22),
	P7_INIT_PINMAP(P7_LCD_0_DATA23),
	P7_INIT_PINMAP(P7_LCD_0_DEN),
	P7_INIT_PINMAP(P7_LCD_0_HS),
	P7_INIT_PINMAP(P7_LCD_0_VS),
};

static unsigned int const zebu_avi_lcd0_nodes[] = {
	AVI_BLEND0_NODE,
	AVI_GAM0_NODE,
	AVI_LCD0_NODE
};

static unsigned int const zebu_avi_lcd0_fb0_nodes[] = {
	AVI_FIFO00_NODE,
	AVI_SCAL00_NODE,
	AVI_FIFO07_NODE,
	AVI_BLENDIN01_NODE
};

/* Video overlay channel used by the video decoder */
static unsigned int const zebu_avi_voc_nodes1[] = {
	AVI_FIFO02_NODE,
	AVI_SCAL10_NODE,
	AVI_FIFO09_NODE,
	AVI_CONV1_NODE,
	AVI_BLENDIN02_NODE
};

static unsigned int const zebu_avi_voc_nodes2[] = {
	AVI_FIFO06_NODE,
	AVI_SCAL11_NODE,
};

/*
 * AVI channels for display group 1.
 * Warning: complete channels MUST come first !
 */
static struct avi_chan zebu_avi_chans_lcd0[] = {
	{
		.type       = AVI_DISPLAY_CHAN_TYPE,
		.nodes_nr   = ARRAY_SIZE(zebu_avi_lcd0_nodes),
		.nodes      = zebu_avi_lcd0_nodes
	},
	{
		.type       = AVI_FB_OVERLAY_CHAN_TYPE,
		.nodes      = zebu_avi_lcd0_fb0_nodes,
		.nodes_nr   = ARRAY_SIZE(zebu_avi_lcd0_fb0_nodes),
	},
	{
		.type       = AVI_VIDOUT_CHAN_TYPE,
		.nodes      = zebu_avi_voc_nodes1,
		.nodes_nr   = ARRAY_SIZE(zebu_avi_voc_nodes1),
	},
	{
		.type       = AVI_VIDOUT_CHAN_TYPE,
		.nodes      = zebu_avi_voc_nodes2,
		.nodes_nr   = ARRAY_SIZE(zebu_avi_voc_nodes2),
	},
};

/* Reserve enough memory for a single dual-buffered 32bpp full HD
 * screen. Probably a lot more than we'll actually need. */
#define P7_FB0_AVI_RAM_SIZE (1280 * 720 * 4 * 2)

static struct avifb_overlay zebu_avi_lcd0_overlays[] = {
	{
		.layout = {
			.alpha = AVIFB_ALPHA(100),
			.enabled = 1,
		},
		.dma_memory.end = P7_FB0_AVI_RAM_SIZE,
		.avi_chan = &zebu_avi_chans_lcd0[1],
	},
};

/* AVI display group 1. */
static struct avi_group zebu_avi_group_lcd0 = {
	.name       = "avifb.lcd0",
	.lck        = __SPIN_LOCK_UNLOCKED(zebu_avi_group_lcd0.lck),
	/* This group will be synchronised on the IRQ from LCD0. We consider
	 * that LCD0 is the "master" of the group. */
	.irq        = P7_LCD0_IRQ,
	.chan_nr    = ARRAY_SIZE(zebu_avi_chans_lcd0),
	.channels   = zebu_avi_chans_lcd0
};

static struct avi_videomode zebu_videomode = {
	.name	      = "zebu",
	.xres	      = 720,
	.yres	      = 576,
	.hsync_len    = 64,
	.left_margin  = 68,
	.right_margin = 12,
	.vsync_len    = 5,
	.upper_margin = 39,
	.lower_margin = 5,
	.pixclock     = 27000,
};

static struct avifb_platform_data zebu_avifb0_pdata = {
	.avi_group          = &zebu_avi_group_lcd0,
        .lcd_interface={{
          .free_run      = 1,
          .itu656        = 0,
        }},
        .lcd_format_control    = AVI_FORMAT_CONTROL_RGB888_1X24,
	.lcd_default_videomode = &zebu_videomode,
	.overlays	       = zebu_avi_lcd0_overlays,
	.overlay_nr	       = ARRAY_SIZE(zebu_avi_lcd0_overlays),
};

/* Should we bother setting a narrower mask here? */
static u64 zebu_avifb0_dma_mask = DMA_BIT_MASK(32);
static struct platform_device zebu_avifb0_dev = {
	.name           = "avifb",
	.id             = 0,
	.dev            = {
		/* Todo: try to set a narrower mask */
		.dma_mask           = &zebu_avifb0_dma_mask,
		.coherent_dma_mask  = DMA_BIT_MASK(32)
	}
};

#if defined(CONFIG_VIDOUT_AVI) || defined(CONFIG_VIDOUT_AVI_MODULE)
static struct avi_voc_plat_data zebu_avi_voc_param = {
	.chan0   = &zebu_avi_chans_lcd0[2],
	.chan1   = &zebu_avi_chans_lcd0[3]
};
#endif /*defined(CONFIG_VIDOUT_AVI) || defined(CONFIG_VIDOUT_AVI_MODULE)*/

#endif /* defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */


/****************************************************************************
 * SPI slave devices (i.e., devices connected to Parrot7 master controllers)
 ****************************************************************************/

#if (defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)) && \
	(defined(CONFIG_SPI_MASTER_PARROT7) || \
	 defined(CONFIG_SPI_MASTER_PARROT7_MODULE))
#include <linux/spi/spi.h>
#include <spi/p7-spim.h>

static struct pinctrl_map zebu_spidev_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_00), /* SS */
	P7_INIT_PINMAP(P7_SPI_01), /* CLK */
	P7_INIT_PINMAP(P7_SPI_02), /* MOSI */
	P7_INIT_PINMAP(P7_SPI_03)  /* MISO */
};

/*
 * Single SPI half-duplex mode SPI controller internal multiplexing setup:
 * the same line (DATA0) is used alternativly for MISO and MOSI functions.
 */
static struct p7spi_swb const zebu_spidev_swb[] = {
	P7SPI_INIT_SWB(0,   P7_SWB_DIR_OUT, P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(1,   P7_SWB_DIR_OUT, P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(2,   P7_SWB_DIR_OUT, P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(3,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_SWB_LAST,
};

static struct p7spi_ctrl_data zebu_spidev_cdata = {
	.half_duplex        = true,
	.read               = true,
	.write              = true,
	.xfer_mode          = P7SPI_SINGLE_XFER,
	.fifo_wcnt          = 64,
	.thres_wcnt         = 48,
	.tsetup_ss_ns       = 10,
	.thold_ss_ns	    = 10,
	.toffclk_ns	        = 10,
	.tcapture_delay_ns  = 40
};

static P7_DECLARE_SPIM_SLAVE(zebu_spidev_dev,
                             "spidev",
                             NULL,
                             &zebu_spidev_cdata,
                             10000000,
                             SPI_MODE_0);


static void __init zebu_init_spidev(void)
{
	int err;

	p7_init_spim(0,
	             zebu_spidev_pins,
	             ARRAY_SIZE(zebu_spidev_pins),
	             zebu_spidev_swb);

	err = p7_init_spim_slave(0, &zebu_spidev_dev);
	WARN(err,
	     P7ZB_BRD_NAME ": failed to init userspace SPI device (%d)\n",
	     err);
}

#else   /* (defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)) && \
	       (defined(CONFIG_SPI_MASTER_PARROT7) || \
	        defined(CONFIG_SPI_MASTER_PARROT7_MODULE)) */
#define zebu_init_spidev()
#endif  /* (defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)) && \
	       (defined(CONFIG_SPI_MASTER_PARROT7) || \
	        defined(CONFIG_SPI_MASTER_PARROT7_MODULE)) */

#if (defined(CONFIG_SPI_SLAVE_PARROT7) || \
     defined(CONFIG_SPI_SLAVE_PARROT7_MODULE))
#include <spi/p7-spis.h>
#define ZEBU_SPI_SLAVE_BUS  3

#if defined(CONFIG_ARCH_PARROT7_ZEBU_MPW1) /* runs on elephant/MPW1 */
static struct pinctrl_map zebu_spi_slave_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_02a), /* CLK */
	P7_INIT_PINMAP(P7_SPI_01a), /* SS */
	P7_INIT_PINMAP(P7_SPI_03a), /* mosi */
	P7_INIT_PINMAP(P7_SPI_04a), /* miso */
	P7_INIT_PINMAP(P7_SPI_00a), /* data 2 */
	P7_INIT_PINMAP(P7_SPI_05a), /* data 2 */
};

static struct p7spi_swb const zebu_spi_slave_swb[] = {
	P7SPI_INIT_SWB(2,   P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(1,   P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(5,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(0,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA1),
	P7SPI_INIT_SWB(4,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA2),
	P7SPI_INIT_SWB(3,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA3),
	P7SPI_SWB_LAST,
};
#else /* runs on hawk/R3 */
static struct pinctrl_map zebu_spi_slave_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SPI_12), /* CLK */
	P7_INIT_PINMAP(P7_SPI_13), /* SS */
	P7_INIT_PINMAP(P7_SPI_15), /* mosi */
	P7_INIT_PINMAP(P7_SPI_14), /* miso */
	P7_INIT_PINMAP(P7_SPI_04), /* data 2 */
	P7_INIT_PINMAP(P7_SPI_05), /* data 3 */
};

static struct p7spi_swb const zebu_spi_slave_swb[] = {
	P7SPI_INIT_SWB(12,  P7_SWB_DIR_IN,  P7_SWB_SPI_CLK),
	P7SPI_INIT_SWB(13,  P7_SWB_DIR_IN,  P7_SWB_SPI_SS),
	P7SPI_INIT_SWB(5,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA0),
	P7SPI_INIT_SWB(4,   P7_SWB_DIR_IN,  P7_SWB_SPI_DATA1),
	P7SPI_INIT_SWB(15,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA2),
	P7SPI_INIT_SWB(14,  P7_SWB_DIR_IN,  P7_SWB_SPI_DATA3),
	P7SPI_SWB_LAST,
};
#endif

static struct p7spis_ctrl_data zebu_spi_slave_cdata = {
	.common = {
		.half_duplex        = true,
		.read               = true,
		.write              = true,
		.xfer_mode          = P7SPI_QUAD_XFER,
		.fifo_wcnt          = 16,
		.thres_wcnt         = 12,
		.tsetup_ss_ns       = 1,
		.thold_ss_ns        = 1,
		.toffclk_ns         = 1,
		.toffspi_ns         = 1,
		.tcapture_delay_ns  = 0,
	}
};

static P7_DECLARE_SPIS_MASTER(zebu_spi_slave_info,
               "spistest",
               NULL,
               &zebu_spi_slave_cdata,
               26 * 1000 * 1000, /* 26 MHz (1/10 of low clk) */
               SPI_MODE_0);

static void __init zebu_init_spi_slave(void)
{
	int err;

	p7_init_spis(ZEBU_SPI_SLAVE_BUS,
	             zebu_spi_slave_pins,
	             ARRAY_SIZE(zebu_spi_slave_pins),
	             zebu_spi_slave_swb);

	err = p7_init_spis_master(ZEBU_SPI_SLAVE_BUS,
	                          &zebu_spi_slave_info);
	WARN(err,
	     "Failed to init SPI slave (%d)\n",
	      err);
}

#else /* CONFIG_SPI_SLAVE_PARROT7 || CONFIG_SPI_SLAVE_PARROT7_MODULE */
#define zebu_init_spi_slave()
#endif /* CONFIG_SPI_SLAVE_PARROT7 || CONFIG_SPI_SLAVE_PARROT7_MODULE */


/******************************
 * Machine base initialization
 ******************************/

static void __init p7_fixup_zebu(struct tag* tags,
                                 char** cmdline,
                                 struct meminfo* mi)
{
	pr_debug(P7ZB_BRD_NAME ": RAM fixup --> 512MB @ " __stringify(PHYS_OFFSET) "\n");
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = 512 * SZ_1M;

	/* Just to speed up booting process with Zebu Only!!!! */
	lpj_fine = CONFIG_ARCH_PARROT7_ZEBU_FREQ / HZ;
}

static void p7_poweroff_zebu(void)
{
	writel(1, __MMIO_P2V(P7_SYS_DBG_HALT));
}

static void __init zebu_reserve_mem(void)
{
#define ZEBU_PHYBRD_SIZE (CONFIG_ARCH_PARROT7_ZEBU_PHYBRD_SIZE << 20)
	zebu_reserve_brdmem(ZEBU_PHYBRD_SIZE);
#define ZEBU_HX280_SIZE (CONFIG_ARCH_PARROT7_ZEBU_HX280_SIZE << 20)
	p7_reserve_vencmem(ZEBU_HX280_SIZE);
#define ZEBU_HX270_SIZE (CONFIG_ARCH_PARROT7_ZEBU_HX270_SIZE << 20)
	p7_reserve_vdecmem(ZEBU_HX270_SIZE);
#define P7_AVI_VOC_RAM_SIZE (1920 * 1080 * 4 * 4)
	p7_reserve_avi_voc_mem(&zebu_avi_voc_param,P7_AVI_VOC_RAM_SIZE);

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
	{
		int i;

		for (i = 0; i < ARRAY_SIZE(zebu_avi_lcd0_overlays); i++)
			p7_reserve_devmem(
				zebu_avi_lcd0_overlays[i].dma_memory.end,
				&zebu_avifb0_dev,
				&zebu_avi_lcd0_overlays[i].dma_memory);
	}
#endif /* defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */

	p7_reserve_nand_mem();

	p7_reserve_dmamem();
}

static void __init zebu_init_mach(void)
{
	unsigned long fb_start = 0;
	unsigned long fb_size = 0;
	p7_init_mach();

	p7_init_gpio(zebu_irq_gpios,
	             ARRAY_SIZE(zebu_irq_gpios));

	p7brd_init_uart(0,1);
	/* Why do we need UART1 on zebu ? */
#if 0
	p7brd_init_uart(1,1);
#endif

	p7brd_init_i2cm(0, 400);
	p7_init_p7mu(0,
		     &zebu_p7mu_pdata,
		     zebu_p7mu_pins,
		     ARRAY_SIZE(zebu_p7mu_pins));
	p7_init_i2cs(&zebu_i2cs1_pdata,
				 zebu_i2cs1_pins,
				 ARRAY_SIZE(zebu_i2cs1_pins));
	p7_init_i2cs(&zebu_i2cs2_pdata,
				 zebu_i2cs2_pins,
				 ARRAY_SIZE(zebu_i2cs2_pins));
	zebu_init_brd();
	p7brd_init_nand(1);

	zebu_init_spidev();
	zebu_init_spi_slave();

	p7brd_init_sdhci(0, &zebu_sdhci0_pdata, NULL, NULL, NULL, NULL, 0);

	p7_init_venc();
	p7_init_vdec();
	p7_init_avi_voc(0, &zebu_avi_voc_param);

	pm_power_off = &p7_poweroff_zebu;

	p7_init_avi();

#if defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE)
	p7_init_avifb(&zebu_avifb0_dev, &zebu_avifb0_pdata,
		      zebu_avifb0_pins, ARRAY_SIZE(zebu_avifb0_pins));
#if defined(CONFIG_GPU_PARROT7) || defined(CONFIG_GPU_PARROT7_MODULE)
	fb_start = zebu_avifb0_pdata.overlays[0].dma_memory.start;
	fb_size = zebu_avifb0_pdata.overlays[0].dma_memory.end - fb_start + 1;
	p7_init_gpu_fb(fb_start, fb_size, 4);
#endif

#endif /* defined(CONFIG_FB_AVI) || defined(CONFIG_FB_AVI_MODULE) */
}

#ifdef CONFIG_SERIAL_ZPRINT
#include <serial/zprint.h>

static struct platform_device zebu_zprint_dev = {
	.name           = ZPRINT_DRV_NAME,
	.id             = 0,
};

static int __init zebu_init_zprint(void)
{
	if (machine_desc->nr == MACH_TYPE_PARROT_ZEBU)
		return p7_init_dev(&zebu_zprint_dev, NULL, NULL, 0);

	return 0;
}
arch_initcall(zebu_init_zprint);

#endif

#ifdef CONFIG_SERIAL_PARROTX_CONSOLE
#include <linux/console.h>

static int __init zebu_init_cons(void)
{
	if (machine_desc->nr == MACH_TYPE_PARROT_ZEBU)
		return add_preferred_console("ttyPA", 0, "115200n8");

	return 0;
}
console_initcall(zebu_init_cons);

#endif

MACHINE_START(PARROT_ZEBU, "Parrot7 Zebu emulated design")
	.restart_mode   = 's',
	.fixup          = &p7_fixup_zebu,
	.reserve        = &zebu_reserve_mem,
	.map_io         = &p7_map_io,
	.init_irq       = &p7_init_irq,
	.timer          = &p7_tick_timer,
	.init_machine   = &zebu_init_mach,
	.handle_irq     = &gic_handle_irq,
	.restart        = &p7_restart,
MACHINE_END
