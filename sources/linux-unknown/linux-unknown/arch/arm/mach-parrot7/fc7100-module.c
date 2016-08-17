/*
 * linux/arch/arm/mach-parrot7/fc7100-modules.c - FC7100 Modules
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Thomas Poussevin <thomas.poussevin@parrot.com>
 * date:    2013-07
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/mmc/host.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>

#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <mach/gpio.h>
#include <gpio/p7-gpio.h>
#include <i2c/p7-i2cm.h>
#include <mfd/p7mu.h>
#include <host/sdhci.h>
#include <mmc/acs3-sdhci.h>

#include <asm/mach-types.h>
#include <asm/system.h>

#include "common.h"
#include "system.h"
#include "aai.h"
#include "avi.h"
#include "backlight.h"
#include "gbc.h"
#include "gpio.h"
#include "gpu.h"
#include "i2cm.h"
#include "lcd-monspecs.h"
#include "p7_pwm.h"
#include "p7mu.h"
#include "pinctrl.h"
#include "sdhci.h"
#include "spi.h"
#include "mpegts.h"
#include "uart.h"
#include "venc.h"
#include "vdec.h"
#include "p7_temperature.h"
#include "wl18xx.h"

/* p7mu */
#include <mfd/p7mu.h>
#include <i2c/p7-i2cm.h>

#include "board-common.h"
#include "fc7100-module.h"

#define FC7100_BRD_NAME "FC7100 module"


/***********************
 * Board identification
 ***********************/
int fc7100_module_rev = -1;

static int ddr_rev(void)
{
		int gpios[] = {2, 3, 4};
		int i;
		int ddr_rev = 0;
		for (i = 0; i < ARRAY_SIZE(gpios); i++) {
			int err, val;
			err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
					P7CTL_PUD_CFG(DOWN), "fc7100-ddr rev");
			if (err == 0) {
				val = gpio_get_value(P7_GPIO_NR(gpios[i]));
				ddr_rev |= (val << i);
			}
		}
		printk(KERN_WARNING"fc7100 ddr config 0x%x\n", ddr_rev);
		return ddr_rev;
}

static int rf_config(void)
{
		int gpios[] = {123, 124, 125};
		int i;
		int rf_conf = 0;
		for (i = 0; i < ARRAY_SIZE(gpios); i++) {
			int err, val;
			err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
					P7CTL_PUD_CFG(DOWN), "fc7100-rf conf");
			if (err == 0) {
				val = gpio_get_value(P7_GPIO_NR(gpios[i]));
				rf_conf |= (val << i);
			}
		}
		printk(KERN_WARNING"fc7100 rf config 0x%x\n", rf_conf);
		return rf_conf;
}

static void init_module_rev(void)
{
	if (p7_chiprev() == P7_CHIPREV_R1) {
		u32 boot_pins;
		boot_pins = (__raw_readl(MMIO_P2V(P7_SYS_BOOT_MODE)) >> 8) & 0xf;

		switch (boot_pins) {
			default:
				panic("p7r1 revision not supported anymore %x\n", boot_pins);
		}
	}
	else if (system_rev) {
		fc7100_module_rev = system_rev;
	}
	else {
		/* use CAM1 DATA[0:3] to get the revision */
		int gpios[] = {218,219,0,1};
		int i;
		int module_rev = 0;
		for (i = 0; i < ARRAY_SIZE(gpios); i++) {
			int err, val;
			err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
					P7CTL_PUD_CFG(DOWN), "fc7100 rev");
			if (err == 0) {
				val = gpio_get_value(P7_GPIO_NR(gpios[i]));
				module_rev |= (val << i);
			}
		}

		fc7100_module_rev = module_rev + 4;
		if (fc7100_module_rev < 8) {
			/* moved to fuse */
			ddr_rev();
			rf_config();
		}
	}
	printk(KERN_WARNING"fc7100 : detected module %d\n", fc7100_module_rev);
	printk(KERN_WARNING"fc7100 : using board %d rev %d\n", fc7100_board_get_id(),
			fc7100_board_get_rev());

	/* check if board detection is working */
	if (fc7100_module_get_rev() >= 6) {
		/* for antec board we need to wait there is no more hw04 */
		BUG_ON(fc7100_board_get_id() == 0 && !(machine_is_fc7100() || machine_is_antec()));
	}
	else {
		WARN_ON(fc7100_board_get_id() == 0 && !machine_is_fc7100());
	}

	/* export rev to cpuinfo :
	   bit 0-7 : module rev
	   bit 8-11 : not used
	   bit 12-15 : board rev
	 */
	system_rev = (fc7100_board_get_rev() << 12) | fc7100_module_rev;
}

int fc7100_board_get_rev(void)
{
	static int board_rev = -1;

	if (fc7100_module_get_rev() <= 3)
		board_rev = 0;

	if (board_rev == -1) {
		static const int rev_mapping [16] = {
			[0] = -1,
			/* 1 bit */
			[1] = 0,
			[2] = 1,
			[4] = 2,
			[8] = 3,
			/* 2 bit */
			[3] = 4,
			[5] = 5,
			[9] = 6,
			[6] = 7,
			[10] = 8,
			[12] = 9,
			[7] = 10,
			[11] = 11,
			[13] = 12,
			[14] = 13,
			[15] = 14,
		};
		/* use LCD1 DATA[20:23] to get the revision */
		static const int gpios[] = {203, 204, 205, 206};
		int i;
		board_rev = 0;
		for (i = 0; i < ARRAY_SIZE(gpios); i++) {
			int err, val;
			err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
					P7CTL_PUD_CFG(DOWN), "fc7100 mb rev");
			if (err == 0) {
				val = gpio_get_value(P7_GPIO_NR(gpios[i]));
				board_rev |= (val << i);
				gpio_free(P7_GPIO_NR(gpios[i]));
			}
		}
		board_rev = rev_mapping[board_rev & 0xf];
	}
	return board_rev;
}

int fc7100_board_get_id(void)
{
	static int board_id = -1;

	if (fc7100_module_get_rev() <= 3)
		board_id = 0;

	if (board_id == -1) {
		/* use LCD1 DATA[0:12] to get the id */
		int gpios[] = {183, 184, 185, 186, 187, 189, 191, 193, 195};
		int i;
		board_id = 0;
		for (i = 0; i < ARRAY_SIZE(gpios); i++) {
			int err, val;
			err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
					P7CTL_PUD_CFG(DOWN), "fc7100 mb id");
			if (err == 0) {
				val = gpio_get_value(P7_GPIO_NR(gpios[i]));
				board_id |= (val << i);
				gpio_free(P7_GPIO_NR(gpios[i]));
			}
		}
	}
	return board_id;
}

#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>
static struct pinctrl_map spim_pins[4][8];
static struct p7spi_swb spim_swb[4][5];
static int spim_pin_name[] = {
	P7_SPI_00,
	P7_SPI_01,
	P7_SPI_02,
	P7_SPI_03,
	P7_SPI_04,
	P7_SPI_05,
	P7_SPI_06,
	P7_SPI_07,
	P7_SPI_08,
	P7_SPI_09,
	P7_SPI_10,
	P7_SPI_11,
	P7_SPI_12,
	P7_SPI_13,
	P7_SPI_14,
	P7_SPI_15,
	P7_SPI_16,
	P7_SPI_17,
	P7_SPI_18,
	P7_SPI_19,
};

static unsigned long fc7100_spim_single_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=0xf), octopus board don't work @ 1 (reg=3) */
};

__init int fc7100_init_spim_single(int bus, int clk_pin, int ss_pin,
				   int mosi_pin, int miso_pin)
{
	int idx = 0;
	if (bus < 0 || bus > 4)
		return -EINVAL;

	if (clk_pin >= 0) {
		struct pinctrl_map pin_map = P7_INIT_PINMAP(spim_pin_name[clk_pin]);
		struct pinctrl_map pin_cfg = P7_INIT_PINCFG(spim_pin_name[clk_pin],
			fc7100_spim_single_pinconf);
		spim_swb[bus][idx].pad = clk_pin;
		spim_swb[bus][idx].direction = P7_SWB_DIR_OUT;
		spim_swb[bus][idx].function = P7_SWB_SPI_CLK;
		spim_pins[bus][idx*2] = pin_map;
		spim_pins[bus][idx*2+1] = pin_cfg;
		idx++;
	}
	if (ss_pin >= 0) {
		struct pinctrl_map pin_map = P7_INIT_PINMAP(spim_pin_name[ss_pin]);
		struct pinctrl_map pin_cfg = P7_INIT_PINCFG(spim_pin_name[ss_pin],
			fc7100_spim_single_pinconf);
		spim_swb[bus][idx].pad = ss_pin;
		spim_swb[bus][idx].direction = P7_SWB_DIR_OUT;
		spim_swb[bus][idx].function = P7_SWB_SPI_SS;
		spim_pins[bus][idx*2] = pin_map;
		spim_pins[bus][idx*2+1] = pin_cfg;
		idx++;
	}
	if (miso_pin >= 0) {
		struct pinctrl_map pin_map = P7_INIT_PINMAP(spim_pin_name[miso_pin]);
		struct pinctrl_map pin_cfg = P7_INIT_PINCFG(spim_pin_name[miso_pin],
			fc7100_spim_single_pinconf);
		spim_swb[bus][idx].pad = miso_pin;
		spim_swb[bus][idx].direction = P7_SWB_DIR_IN;
		spim_swb[bus][idx].function = P7_SWB_SPI_DATA0;
		spim_pins[bus][idx*2] = pin_map;
		spim_pins[bus][idx*2+1] = pin_cfg;
		idx++;
	}
	if (mosi_pin >= 0) {
		struct pinctrl_map pin_map = P7_INIT_PINMAP(spim_pin_name[mosi_pin]);
		struct pinctrl_map pin_cfg = P7_INIT_PINCFG(spim_pin_name[mosi_pin],
			fc7100_spim_single_pinconf);
		spim_swb[bus][idx].pad = mosi_pin;
		spim_swb[bus][idx].direction = P7_SWB_DIR_OUT;
		spim_swb[bus][idx].function = P7_SWB_SPI_DATA0;
		spim_pins[bus][idx*2] = pin_map;
		spim_pins[bus][idx*2+1] = pin_cfg;
		idx++;
	}
	spim_swb[bus][idx].pad = -1; /* end marker */

	/* XXX this function panic in case of error */
	p7_init_spim(bus, spim_pins[bus], idx*2, spim_swb[bus]);
	return 0;
}

/*****
 * MPEGTS/tunner
 *****/
#include <spi/p7-spim.h>
#include <spi/p7-spi.h>
#include <linux/spi/spi.h>

#define MAX_MPEGTS_BUS 2
static struct p7mpg_plat_data fc7100_tuner_pdata[MAX_MPEGTS_BUS];
static struct pinctrl_map fc7100_tuner_pins[MAX_MPEGTS_BUS][4];
static struct p7spi_swb fc7100_tuner_swb[MAX_MPEGTS_BUS][3];

static unsigned long fc7100_spim_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(3),      /* Drive strength 3 (reg=0xf), octopus board don't work @ 1 (reg=3) */
};

int __init fc7100_init_mpegts_single(int bus, int clk_pin, int data_pin)
{
	if (bus < 0 || bus+1 > MAX_MPEGTS_BUS || clk_pin < 0 || data_pin < 0) {
		pr_err(FC7100_BRD_NAME "Bad parameters in mpegts init.\n");
		return -EINVAL;
	}
	else {
		struct pinctrl_map tmp_fc7100_tuner_pins[] = {
			P7_INIT_PINMAP(spim_pin_name[clk_pin]),
			P7_INIT_PINCFG(spim_pin_name[clk_pin], fc7100_spim_pinconf),
			P7_INIT_PINMAP(spim_pin_name[data_pin]),
			P7_INIT_PINCFG(spim_pin_name[data_pin], fc7100_spim_pinconf),
		};
		struct p7spi_swb tmp_fc7100_tuner_swb[] = {
			P7SPI_INIT_SWB(clk_pin, P7_SWB_DIR_IN, P7_SWB_MPEG_CLK),
			P7SPI_INIT_SWB(data_pin, P7_SWB_DIR_IN,
				       P7_SWB_MPEG_DATA),
			P7SPI_SWB_LAST,
		};
		memcpy(fc7100_tuner_pins[bus], &tmp_fc7100_tuner_pins,
		       sizeof(tmp_fc7100_tuner_pins));
		memcpy(fc7100_tuner_swb[bus], &tmp_fc7100_tuner_swb,
		       sizeof(tmp_fc7100_tuner_swb));
		fc7100_tuner_pdata[bus].swb = fc7100_tuner_swb[bus];
		fc7100_tuner_pdata[bus].fifo_wcnt = 16;
		fc7100_tuner_pdata[bus].thres_wcnt = 8;

		p7_init_mpegts(bus, &fc7100_tuner_pdata[bus], fc7100_tuner_pins[bus],
			       ARRAY_SIZE(fc7100_tuner_pins[bus]));
		return 0;
	}
}

/**********************
 * TI wl18xx wlan
 **********************/
static __initdata struct wl18xx_resources fc7100_wl18xx_res_hw45 = {
	.wlirq_gpio = P7_GPIO_NR(177),
	.wlen_gpio = P7_GPIO_NR(175),
	.bt_rst_gpio = P7_GPIO_NR(176),
	.wl_bt_ant_gpio = P7_GPIO_NR(173),
	.wl5g_ant_gpio = P7_GPIO_NR(174),
	.bt_uart_slot = 1,
	.wl_sdhci_slot = 0,
};

static __initdata struct wl18xx_resources fc7100_wl18xx_res_hw6 = {
	.wlirq_gpio = P7_GPIO_NR(33),
	.wlen_gpio = P7_GPIO_NR(175),
	.bt_rst_gpio = P7MU_IO_NR(4),
	.wl_bt_ant_gpio = P7_GPIO_NR(173),
	.wl5g_ant_gpio = P7_GPIO_NR(174),
	.bt_uart_slot = 1,
	.wl_sdhci_slot = 0,
};

static void __init fc7100_init_wl18xx(void)
{
	if (fc7100_module_get_rev() >= 8)
		fc7100_wl18xx_res_hw6.wlirq_gpio = P7_GPIO_NR(120);

	if (fc7100_module_get_rev() <= 5)
		init_wl18xx(&fc7100_wl18xx_res_hw45, NULL, 0);
	else
		init_wl18xx(&fc7100_wl18xx_res_hw6, NULL, 0);
}


/*****************************
 * P7MU Power Management Unit
 *****************************/

static unsigned long fc7100_p7mu_reboot_pinconf[] = {
       P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
       P7CTL_PUD_CFG(DOWN) | /* no pull up/down unable */
       P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
       P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map fc7100_p7mu_pins[] __initdata = {
	P7_INIT_PINMAP(P7_REBOOT_P7MU),    /* P7 -> P7MU reset request */
	P7_INIT_PINCFG(P7_REBOOT_P7MU, fc7100_p7mu_reboot_pinconf)
};

static struct p7mu_plat_data fc7100_r23_p7mu_pdata = {
	.gpio       = -1,       /* GPIO 75 is P7MU -> P7 interrupt source */
	.int_32k    = false,    /* External 32kHz clock. */
	.int_32m    = false,     /* External 48mHz clock. */
	.gpio_event_sel = P7MU_PMC_GPIO5_EDGE_RISING, /* Wakeup on gpio5 edge rising event */
};

static void __init fc7100_init_p7mu(void)
{
	fc7100_r23_p7mu_pdata.gpio = P7_GPIO_NR(72);
	p7_gpio_interrupt_register(fc7100_r23_p7mu_pdata.gpio);

	/* I2CM0: P7MU */
	p7brd_init_i2cm(0, 100);
	/* I2CM3: for test */
	//p7brd_init_i2cm(3, 100);

	p7_init_p7mu(0,
			&fc7100_r23_p7mu_pdata,
			fc7100_p7mu_pins,
			ARRAY_SIZE(fc7100_p7mu_pins));

	if (fc7100_module_get_rev() >= 5) {
		/* Switch p7mu CLK_OUT_32k to SPI_SS output */
		p7mu_write16(0x16, 0xAB);
	}

	if (fc7100_module_get_rev() >= 9) {
		/* Switch DCDC_0v9 "Power Good" signal to p7mu C12 pin */
		p7mu_write16(0x14, 0x83);
	}
}

/* temp sensors */
static struct p7_temp_chan p7mu_adc_channels[] = {
	/* DDR3 */
	{
		.channel = 1,
		.freq = 160000,
		.name = "ddr",
	},
	/* MOS1 */
	{
		.channel = 4,
		.freq = 160000,
		.name = "mos1",
	},
	/* PCB1 */
	{
		.channel = 6,
		.freq = 160000,
		.name = "pcb1",
	},
};

static struct p7_temp_chan_data p7mu_adc_chan_data = {
        .channels               = p7mu_adc_channels,
        .num_channels           = ARRAY_SIZE(p7mu_adc_channels),
        .temp_mode              = P7_TEMP_FC7100_HW04,
};

static struct p7_temp_chan p7mu_adc_channels_hw08[] = {
	/* HW08 : V2/ref */
	{
		.channel = 2,
		.freq = 160000,
		.name = "ref",
	},
	/* HW08 : V1/DDR3 */
	{
		.channel = 1,
		.freq = 160000,
		.name = "ddr",
	},
	/* HW08 : V1/MOS1 */
	{
		.channel = 4,
		.freq = 160000,
		.name = "mos1",
	},
	/* HW08 : V1/PCB1 */
	{
		.channel = 6,
		.freq = 160000,
		.name = "pcb1",
	},
};

static struct p7_temp_chan_data p7mu_adc_chan_data_hw08 = {
        .channels               = p7mu_adc_channels_hw08,
        .num_channels           = ARRAY_SIZE(p7mu_adc_channels_hw08),
		.temp_mode				= P7_TEMP_FC7100_HW08,
};

static struct platform_device fc7100_temp_device = {
	.name = "fc7100-temperature",
	.id = -1,
	.dev.platform_data = &p7mu_adc_chan_data,
};

void __init fc7100_init_temperature(void)
{
	p7_init_temperature();
	if (fc7100_module_get_rev() >= 8)
		fc7100_temp_device.dev.platform_data = &p7mu_adc_chan_data_hw08;
	platform_device_register(&fc7100_temp_device);
}

/*******************/

void __init fc7100_init_module(unsigned int mod_settings)
{
	p7_init_mach();
	/* first init gpio controler to have gpio_to_irq working */
	p7_init_gpio(NULL, 0);

	/* need gpio */
	init_module_rev();

	/* provide gpio */
	fc7100_init_p7mu();

	fc7100_init_wl18xx();

	p7brd_init_nand(0);

	/* debug uart */
	if (fc7100_module_get_rev() >= 4)
		p7brd_init_uart(3,0);

	fc7100_init_temperature();
}
