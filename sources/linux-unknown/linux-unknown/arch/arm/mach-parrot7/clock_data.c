/*
 * Copyright (C) 2013 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * P7 clock tree
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clkdev.h>

#include <clk/p7clk-provider.h>
#include <mach/p7.h>

#include "common.h"
#include "system.h"
#include "clock.h"


#define P7_DEFINE_LSPCLK(_name, _clk_reg, _reset_reg)                   \
	DEFINE_CLK_P7_RESET_GATE(_name, "p7_low_clk", &p7_low_clk,      \
	                         _clk_reg, 1, _reset_reg)
#define P7_DEFINE_XINCLK(_name, _clk_reg)                               \
	DEFINE_CLK_P7_XIN(_name, "p7_fast_clk", &p7_fast_clk, _clk_reg) \

struct clk* clk_register_lspclk(const char *name, unsigned long clk_reg,
				unsigned long reset_reg)
{
	return clk_register_gbc(NULL, name, "low_clk", 0, clk_reg, 1,
	                               reset_reg, 0, 0, NULL);
}

struct clk* clk_register_hspclk(const char *name, unsigned long clk_reg, u32 clk_mask,
				unsigned long reset_reg)
{
	return clk_register_gbc(NULL, name, "low_clk", 0, clk_reg, clk_mask,
	                               reset_reg, 0, 0, NULL);
}


struct clk* p7clk_register_pll(const char *name,
                               const bool gateable, const unsigned long flags,
                               const unsigned long pll_id,
                               unsigned long const * const cfg_idx,
                               unsigned long const * const pll_cfg,
                               const size_t cfg_nr, spinlock_t *lock)
{
	unsigned long const addr = __MMIO_P2V(P7_SYS_CLKGEN_CFG_FAST) +
	                                   (pll_id * sizeof(u32));

	return clk_register_pll(NULL, name, "clkin", gateable, flags,
	                        __MMIO_P2V(P7_SYS_CLKGEN_STATUS), (u8) pll_id,
	                        addr, cfg_idx, pll_cfg, cfg_nr, lock);
}


enum p7_clocks {
	clkin,
	pll_fast_clk, pll_fast_clk_div2,
	fast_clk, sys_clk, cpu_clk, low_clk, proc_clk, twd_clk,
	pll_ddr_clk, pll_aai_clk,
	pll_avi0_clk,
	pll_avi1_clk,
	pll_nand_clk,
	pll_usb_clk,
	eth_tx_clk,
	/* IPs */
	i2cm0_clk, i2cm1_clk, i2cm2_clk, i2cms_clk,
	i2cs0_clk, i2cs1_clk, i2cs2_clk,
	gpio_clk,
	uart0_clk, uart1_clk, uart2_clk, uart3_clk, uart4_clk,
	uart5_clk, uart6_clk, uart7_clk,
	venc_clk, vdec_clk,
	usb0_clk, usb1_clk,
	dma_clk,
	sdhci0_hostclk, sdhci0_xinclk, sdhci0_sdclk,
	sdhci1_hostclk, sdhci1_xinclk, sdhci1_sdclk,
	sdhci2_hostclk, sdhci2_xinclk, sdhci2_sdclk,
	nand_ctrlclk, nand_phyclk,
	gpu_clk, gpu_pp0_clk, gpu_pp1_clk, gpu_pp2_clk, gpu_pp3_clk,/* mali 400 */
	gpu_busclk, gpu_coreclk, /* mali 200 */
	spi_clk, spi0_clk, spi1_clk, spi2_clk, spi3_clk,
	aai_ctrlclk, i2s_clk, pcm1_clk, pcm2_clk, spdif_clk,
	avi_ctrlclk, lcd0_clk, lcd1_clk,
	eth_ctrlclk,
	can_clk, can0_clk, can1_clk,
	crypto_clk,
	clk_max
};

static struct clk *clk[clk_max];

/* fast clk */
static const char *p7_fast_clk_parent_names_r1[] = {
	"clkin",
	"pll_fast_clk",
};
static const char *p7_fast_clk_parent_names_r2[] = {
	"clkin",
	"pll_fast_clk_div2",
};
static DEFINE_SPINLOCK(p7_fast_clk_lock);

/* sys clk */
static struct clk_div_table sys_clk_div[] = {
	CLK_DIV(0, 2),
	CLK_DIV(1, 4),
	CLK_DIV(2, 8),
	CLK_DIV(3, 16),
	CLK_DIV(4, 0),
};
static DEFINE_SPINLOCK(sys_clk_lock);

/* can clk */
static struct clk_div_table can_clk_div[] = {
	CLK_DIV(0, 78),
	CLK_DIV(1, 52),
	CLK_DIV(2, 40),
	CLK_DIV(3, 0),
};
static DEFINE_SPINLOCK(can_clk_lock);

/* cpu clk */
static DEFINE_SPINLOCK(cpu_clk_lock);

/*
 * NAND PHY PLL possible frequencies computed with CLK_IN (26 MHz).
 * Configuration word is calculated for nand_phy_clkx4 since it comes out from
 * PLL (and not nand_phy_clk).
 * This means the real frequency the PLL operates at is 4 times
 * p7_nand_phyclk->freq.
 *
 * Warning: keep this in strict ascending order with no duplicates !!
 */
static unsigned long const p7_nandphy_cfgidx[] = {
	/*
	 * pll_nand max frequency 400MHz
	 * out frequency / 4
	 */
	/* 26MHz as a minimum attainable frequency for calculation purpose */
	26000000,
	/* NAND synchronous mode 0 (20 Mhz) */
	79625000,
	/* mpw1 mode 0 asynchronous */
	118625000,
	/* NAND synchronous mode 1 (33 Mhz) */
	131625000,
	/* NAND synchronous mode 2 (50 Mhz) */
	199875000,
	/* mpw1 mode 1-5 asynchronous */
	237250000,
	/* tccs = 500ns asynchronous */
	253500000,
	/* NAND synchronous mode 3 (66 Mhz) */
	263250000,
	/* NAND synchronous mode 4 (83 Mhz) */
	331500000,
	359125000,
	/* Nand asynchronous mode + synchronous mode 5 (100 Mhz) */
	399750000,
	/* 400MHz as a maximum attainable frequency for calculation purpose */
	400000000,
};

static unsigned long const p7_nandphy_pllcfg[] = {
	/* 26MHz as a minimum attainable frequency for calculation purpose */
	P7_PLL_CFG(0, 0, 0, 0),
	/* NAND synchronous mode 0 (20 Mhz) */
	P7_PLL_CFG(97, 0, 5, 3),
	/* mpw1 mode 0 (118625000Hz) */
	P7_PLL_CFG(72, 0, 4, 3),
	/* NAND synchronous mode 1 (33 Mhz) */
	P7_PLL_CFG(80, 0, 4, 3),
	/* NAND synchronous mode 2 (50 Mhz) */
	P7_PLL_CFG(122, 0, 4, 3),
	/* mpw1 mode 1-5 (237250000Hz) */
	P7_PLL_CFG(72, 0, 3, 3),
	/* tccs = 500ns (253500000Hz) */
	P7_PLL_CFG(77, 0, 3, 3),
	/* NAND synchronous mode 3 (66 Mhz) */
	P7_PLL_CFG(80, 0, 3, 3),
	/* NAND synchronous mode 4 (83 Mhz) */
	P7_PLL_CFG(101, 0, 3, 3),
	P7_PLL_CFG(220, 1, 3, 3),
	/* Nand asynchronous mode + synchronous mode 5 (100 Mhz) */
	P7_PLL_CFG(122, 0, 3, 3),
	/* 400MHz which will not be attained */
	P7_PLL_CFG(122, 0, 3, 3),
};

/* AAI */
static unsigned long const p7_aai_cfgidx[] = {
    /* this table must be filled in ascending order! */
    /* aai_clk frequency 786.5 MHz */
    786500000,
    /* aai_clk frequency 858 MHz */
    858000000,
};

static unsigned long const p7_aai_pllcfg[] = {
	/* aai_clk frequency 786.5 MHz */
	P7_PLL_CFG(120, 0, 2, 3),
	/* aai_clk frequency 858 MHz */
	P7_PLL_CFG(65,  0, 1, 3),
};

/* AVI PLL clock frequencies. Those are the frequencies at the PLL
 * output. There's a factor 2 divisor between the PLL output and the AVI LCD
 * module. */
static unsigned long const p7_avipll_cfgidx[] = {
	25000000,
	25952204,
	27000000,
	35642000,
	48000000,
	49092000,
	50352468,
	54000000,
	54054000,
	60000000,
	66520000,
	100800000,
	108000000,
	108108000,
	113400000,
	118800000,
	126000000,
	127220000,
	132000000,
	132824000,
	142200000,
	144000000,
	145000000,
	148352000,
	148500000,
	154600000,
	161000000,
	167294000,
	189000000,
	198000000,
	216000000,
	216216000,
	225000000,
	237600000,
	250000000,
	260012000,
	270000000,
	296704000,
	297000000,
	315208000,
	330000000,
};

static unsigned long const p7_avipll_mastercfg[] = {
	P7_PLL_CFG(63,  0, 2, 3), /*	 25000000 */
	P7_PLL_CFG(180, 1, 2, 3), /*	 25952204 */
	P7_PLL_CFG(63,  0, 2, 3), /*     27000000 */
	P7_PLL_CFG(148, 1, 2, 3), /*	 35642000 */
	P7_PLL_CFG(127, 1, 3, 1), /*	 48000000 */
	P7_PLL_CFG(81,  0, 2, 3), /*	 49092000 */
	P7_PLL_CFG(68,  0, 2, 3), /*	 50352468 */
	P7_PLL_CFG(63,  0, 2, 3), /*	 54000000 */
	P7_PLL_CFG(81,  0, 2, 3), /*	 54054000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	 60000000 */
	P7_PLL_CFG(71,  0, 2, 3), /*	 66520000 */
	P7_PLL_CFG(162, 1, 2, 3), /*	100800000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	108000000 */
	P7_PLL_CFG(81,  0, 2, 3), /*	108108000 */
	P7_PLL_CFG(136, 1, 3, 3), /*	113400000 */
	P7_PLL_CFG(68,  0, 2, 3), /*	118800000 */
	P7_PLL_CFG(62,  0, 2, 3), /*	126000000 */
	P7_PLL_CFG(67,  0, 2, 3), /*	127220000 */
	P7_PLL_CFG(127, 1, 3, 1), /*	132000000 */
	P7_PLL_CFG(232, 1, 3, 3), /*	132824000 */
	P7_PLL_CFG(196, 1, 3, 3), /*	142200000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	144000000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	145000000 */
	P7_PLL_CFG(73,  0, 2, 3), /*	148352000 */
	P7_PLL_CFG(65,  0, 2, 3), /*	148500000 */
	P7_PLL_CFG(148, 1, 2, 3), /*	154600000 */
	P7_PLL_CFG(63,  0, 2, 3), /*    161000000 */
	P7_PLL_CFG(70,  0, 2, 3), /*    167294000 */
	P7_PLL_CFG(62,  0, 2, 3), /*	189000000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	198000000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	216000000 */
	P7_PLL_CFG(81,  0, 2, 3), /*	216216000 */
	P7_PLL_CFG(71,  0, 2, 3), /*	225000000 */
	P7_PLL_CFG(68,  0, 2, 3), /*	237600000 */
	P7_PLL_CFG(127, 1, 3, 1), /*	250000000 */
	P7_PLL_CFG(130, 1, 2, 3), /*	260012000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	270000000 */
	P7_PLL_CFG(73,  0, 2, 3), /*	296704000 */
	P7_PLL_CFG(65,  0, 2, 3), /*	297000000 */
	P7_PLL_CFG(84,  0, 2, 3), /*	315208000 */
	P7_PLL_CFG(63,  0, 2, 3), /*	330000000 */
};

static unsigned long const p7_avipll_slavecfg[] = {
	P7_PLL_CFG(49,  12, 6, 7), /*	 25000000 */
	P7_PLL_CFG(47,  16, 6, 7), /*	 25952204 */
	P7_PLL_CFG(53,  12, 6, 7), /*    27000000 */
	P7_PLL_CFG(178, 37, 6, 7), /*	 35642000 */
	P7_PLL_CFG(191, 12, 6, 2), /*	 48000000 */
	P7_PLL_CFG(111, 18, 6, 7), /*	 49092000 */
	P7_PLL_CFG(96,  26, 5, 7), /*	 50352468 */
	P7_PLL_CFG(53,  12, 5, 7), /*	 54000000 */
	P7_PLL_CFG(171, 52, 5, 7), /*	 54054000 */
	P7_PLL_CFG(59,  12, 5, 7), /*	 60000000 */
	P7_PLL_CFG(140, 30, 5, 7), /*	 66520000 */
	P7_PLL_CFG(136, 44, 4, 7), /*	100800000 */
	P7_PLL_CFG(53,  12, 4, 7), /*	108000000 */
	P7_PLL_CFG(171, 52, 4, 7), /*	108108000 */
	P7_PLL_CFG(162, 39, 4, 7), /*	113400000 */
	P7_PLL_CFG(88,  20, 4, 7), /*	118800000 */
	P7_PLL_CFG(63,  12, 4, 7), /*	126000000 */
	P7_PLL_CFG(174, 37, 4, 7), /*	127220000 */
	P7_PLL_CFG(131, 12, 4, 2), /*	132000000 */
	P7_PLL_CFG(173, 30, 4, 7), /*	132824000 */
	P7_PLL_CFG(198, 27, 4, 7), /*	142200000 */
	P7_PLL_CFG(71,  12, 4, 7), /*	144000000 */
	P7_PLL_CFG(144, 25, 4, 7), /*	145000000 */
	P7_PLL_CFG(226, 45, 4, 7), /*	148352000 */
	P7_PLL_CFG(71,  12, 4, 7), /*	148500000 */
	P7_PLL_CFG(188, 36, 4, 7), /*	154600000 */
	P7_PLL_CFG(160, 25, 4, 7), /*   161000000 */
	P7_PLL_CFG(28,   4, 4, 7), /*   167294000 */
	P7_PLL_CFG(95,  12, 4, 7), /*	189000000 */
	P7_PLL_CFG(98,  12, 4, 7), /*	198000000 */
	P7_PLL_CFG(53,  12, 3, 7), /*	216000000 */
	P7_PLL_CFG(171, 52, 3, 7), /*	216216000 */
	P7_PLL_CFG(49,  12, 3, 7), /*	225000000 */
	P7_PLL_CFG(88,  20, 3, 7), /*	237600000 */
	P7_PLL_CFG(124, 12, 3, 2), /*	250000000 */
	P7_PLL_CFG(170, 34, 3, 7), /*	260012000 */
	P7_PLL_CFG(134, 25, 3, 7), /*	270000000 */
	P7_PLL_CFG(226, 45, 3, 7), /*	296704000 */
	P7_PLL_CFG(71,  12, 3, 7), /*	297000000 */
	P7_PLL_CFG(177, 38, 3, 7), /*	315208000 */
	P7_PLL_CFG(164, 25, 3, 7), /*	330000000 */
};

/* PLL USB
 * It's the legacy name, but it's actually used for ethernet.
 */
static unsigned long const p7_usbpll_cfgidx[] = {
	65000000, /* ether 65/26 : 2.5 */
	250250000, /* ether 250/2 : 125 */
	624000000, /* can  624 / 78 : 8 */
	650000000, /* ether 650/26 : 25 */
};

static unsigned long const p7_usbpll_pllcfg[] = {
	P7_PLL_CFG(4, 0, 1, 3),
	P7_PLL_CFG(76, 0, 3, 3),
	P7_PLL_CFG(47, 0, 1, 3),
	P7_PLL_CFG(49, 0, 1, 3),
};


static DEFINE_SPINLOCK(p7_spi_clk_lock);

static DEFINE_SPINLOCK(pll_fast_clk_lock);
static DEFINE_SPINLOCK(pll_aai_clk_lock);
static DEFINE_SPINLOCK(pll_avi0_clk_lock);
static DEFINE_SPINLOCK(pll_avi1_clk_lock);
static DEFINE_SPINLOCK(pll_nand_clk_lock);
static DEFINE_SPINLOCK(pll_usb_clk_lock);

static struct clk_fixed_factor p7_avi_pll_div_hw = {
	.mult = 1,
	.div = 2,
};
static struct clk_gate p7_avi0_gate_hw = {
	.reg = (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI00),
	.bit_idx = 20,
	/*
	 * Enable bit is in the PLL config register, so they share
	 * the same lock
	 */
	.lock = &pll_avi0_clk_lock,
};
static struct clk_gate p7_avi1_gate_hw = {
	.reg = (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI10),
	.bit_idx = 20,
	.lock = &pll_avi1_clk_lock,
};
static const char * p7_avi_lcd0clk_parents[] = {
	"pll_avi0_clk",
};
static const char * p7_avi_lcd1clk_parents[] = {
	"pll_avi1_clk",
};

static DEFINE_SPINLOCK(eth_dll_reg_lock);
static u32 eth_tx_mux_parents_table[] = {
	0, /* clkin. This is a dummy clock to avoid init errors */
	1, /* USB PLL */
	3, /* AVI PLL */
	3, /* AVI PLL */
};
static const char* eth_tx_parent_names[] = {
	"clkin",
	"pll_usb_clk",
	"pll_avi0_clk", //P7R3
	"pll_avi1_clk", //P7R2
};
static struct clk_mux p7_eth_tx_mux_hw = {
	.reg    = (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_ETHERNET),
	.table  = eth_tx_mux_parents_table,
	.mask   = 0x3,
	.shift  = 9,
	.lock   = &eth_dll_reg_lock,
};
static struct clk_div_table eth_clk_div[] = {
	CLK_DIV(0, 2),
	CLK_DIV(1, 4),
	CLK_DIV(4, 10),
	CLK_DIV(12, 26),
	CLK_DIV(49, 100),
	CLK_DIV(50, 0),
};
static struct clk_divider p7_eth_tx_div_hw = {
	.reg    = (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_ETHERNET),
	.shift  = 3,
	.width  = 6,
	.table  = eth_clk_div,
	.lock   = &eth_dll_reg_lock,
};
static struct clk_gate p7_eth_tx_gate_hw = {
	.reg    = (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_ETHERNET),
	.bit_idx = 1,
	.lock   = &eth_dll_reg_lock,
};

int __init p7_populate_clock_tree(int low_sysclk)
{
	int i;
	int rev = p7_chiprev();
	unsigned long boot_mode = __raw_readl(__MMIO_P2V(P7_SYS_BOOT_MODE));

	if ((boot_mode >> 8) & 0x1)
		clk[clkin] = clk_register_fixed_rate(NULL, "clkin", NULL, CLK_IS_ROOT, 25000000);
	else
		clk[clkin] = clk_register_fixed_rate(NULL, "clkin", NULL, CLK_IS_ROOT, 26000000);
	clk[pll_fast_clk] = p7clk_register_pll("pll_fast_clk", false, CLK_SET_RATE_GATE,
	                                       0, NULL, NULL, 0, &pll_fast_clk_lock);
	if (rev == P7_CHIPREV_R1 || rev == P7_CHIPREV_R3) {
		clk[fast_clk] = clk_register_mux(
					NULL, "fast_clk", p7_fast_clk_parent_names_r1,
					ARRAY_SIZE(p7_fast_clk_parent_names_r1), 0,
					(void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_FAST), 20, 1, 0,
					&p7_fast_clk_lock);
		clk[proc_clk] = clk_register_fixed_factor(NULL, "proc_clk", "fast_clk", 0,
		                                          2, 5);
	} else if (rev == P7_CHIPREV_R2) {
		clk[pll_fast_clk_div2] = clk_register_fixed_factor(
						NULL, "pll_fast_clk_div2", "pll_fast_clk", 0,
						1, 2);
		clk[fast_clk] = clk_register_mux(
					NULL, "fast_clk", p7_fast_clk_parent_names_r2,
					ARRAY_SIZE(p7_fast_clk_parent_names_r2), 0,
					(void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_FAST), 20, 1, 0,
					&p7_fast_clk_lock);
		clk[proc_clk] = clk_register_fixed_factor(NULL, "proc_clk", "pll_fast_clk", 0,
		                                          1, 5);
	}
	else {
		BUG();
	}

	/*
	 * Use sys_clk = fast_clk / 8, this allows to reach lower
	 * frequencies with cpufreq-p7
	 */
	if (low_sysclk) {
		writel(2, __MMIO_P2V(P7_SYS_CLKGEN_CFG_SYS_DIV));
	}
	clk[sys_clk] = clk_register_divider_table(
	            NULL, "sys_clk", "fast_clk", 0,
	            (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_SYS_DIV),
	            0, 2, 0,
	            sys_clk_div, &sys_clk_lock);
	clk[cpu_clk] = clk_register_divider(NULL, "cpu_clk", "fast_clk", 0,
				   (void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_CPU_DIV), 0, 2,
				   CLK_DIVIDER_POWER_OF_TWO, &cpu_clk_lock);
	clk[twd_clk] = clk_register_fixed_factor(NULL, "twd_clk", "cpu_clk", 0, 1, 2);
	clk[low_clk] = clk_register_fixed_factor(NULL, "low_clk", "fast_clk", 0, 1, 3);


	clk[pll_nand_clk] = p7clk_register_pll("pll_nand_clk", true, 0,
	                                 7, p7_nandphy_cfgidx, p7_nandphy_pllcfg,
	                                 ARRAY_SIZE(p7_nandphy_cfgidx), &pll_nand_clk_lock);
	clk[pll_aai_clk] = p7clk_register_pll("pll_aai_clk", true, 0, 6,
	                                      p7_aai_cfgidx, p7_aai_pllcfg, ARRAY_SIZE(p7_aai_cfgidx),
	                                      &pll_aai_clk_lock);
	clk[pll_usb_clk] = p7clk_register_pll("pll_usb_clk", true, 0, 8,
	                                    p7_usbpll_cfgidx, p7_usbpll_pllcfg, ARRAY_SIZE(p7_usbpll_cfgidx),
	                                    &pll_usb_clk_lock);
	clk[pll_avi0_clk] = clk_register_avipll(NULL, "pll_avi0_clk", "clkin", 0,
				     __MMIO_P2V(P7_SYS_CLKGEN_STATUS), 2,
				     __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI00),
				     p7_avipll_cfgidx, p7_avipll_mastercfg, p7_avipll_slavecfg,
				     ARRAY_SIZE(p7_avipll_cfgidx), &pll_avi0_clk_lock);
	clk[pll_avi1_clk] = clk_register_avipll(NULL, "pll_avi1_clk", "clkin", rev == P7_CHIPREV_R2 ? CLK_SET_RATE_GATE : 0,
				     __MMIO_P2V(P7_SYS_CLKGEN_STATUS), 4,
				     __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI10),
				     p7_avipll_cfgidx, p7_avipll_mastercfg, p7_avipll_slavecfg,
				     ARRAY_SIZE(p7_avipll_cfgidx), &pll_avi1_clk_lock);

	/* IPs */
	clk[i2cm0_clk] = clk_register_lspclk("i2cm0_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CM0_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CM0_RESET);
	clk[i2cm1_clk] = clk_register_lspclk("i2cm1_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CM1_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CM1_RESET);
	clk[i2cm2_clk] = clk_register_lspclk("i2cm2_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CM2_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CM2_RESET);
	clk[i2cs0_clk] = clk_register_lspclk("i2cs0_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CS0_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CS0_RESET);
	clk[i2cs1_clk] = clk_register_lspclk("i2cs1_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CS1_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CS1_RESET);
	clk[i2cs2_clk] = clk_register_lspclk("i2cs2_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CS2_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CS2_RESET);
	clk[gpio_clk] = clk_register_lspclk("gpio_clk",
	                                    __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_GPIO_CLOCK,
	                                    __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_GPIO_RESET);
	clk[uart0_clk] = clk_register_lspclk("uart0_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART0_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART0_RESET);
	clk[uart1_clk] = clk_register_lspclk("uart1_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART1_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART1_RESET);
	clk[uart2_clk] = clk_register_lspclk("uart2_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART2_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART2_RESET);
	clk[uart3_clk] = clk_register_lspclk("uart3_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART3_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART3_RESET);
	clk[uart4_clk] = clk_register_lspclk("uart4_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART4_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART4_RESET);
	clk[uart5_clk] = clk_register_lspclk("uart5_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART5_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART5_RESET);
	clk[uart6_clk] = clk_register_lspclk("uart6_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART6_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART6_RESET);
	clk[uart7_clk] = clk_register_lspclk("uart7_clk",
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART7_CLOCK,
	                                     __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_UART7_RESET);
	if (rev == P7_CHIPREV_R3) {
		clk[i2cms_clk] = clk_register_lspclk("i2cm3_clk",
				__MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CMS_CLOCK,
				__MMIO_P2V(P7_LSP_GBC) + LSP_GBC_I2CMS_RESET);
	}

	if (rev == P7_CHIPREV_R1 || rev == P7_CHIPREV_R2)
		clk[venc_clk] = clk_register_gbc(NULL, "venc_clk", "proc_clk", 0,
				__MMIO_P2V(P7_VENC_GBC) + VENC_GBC_CLOCK, 1,
				__MMIO_P2V(P7_VENC_GBC) + VENC_GBC_RESET, 0,
				0, NULL);
	else
		clk[venc_clk] = clk_register_gbc(NULL, "venc_clk", "proc_clk", 0,
				__MMIO_P2V(P7_VENC_GBC) + VENC_GBC_CLOCK, 1,
				__MMIO_P2V(P7_VENC_GBC) + VENC_GBC_RESET, 0,
				__MMIO_P2V(P7_VENC_GBC) + VENC_GBC_POWER,
				NULL);

	if (rev == P7_CHIPREV_R1 || rev == P7_CHIPREV_R2)
		clk[vdec_clk] = clk_register_gbc(NULL, "vdec_clk", "proc_clk", 0,
				__MMIO_P2V(P7_VDEC_GBC) + VDEC_GBC_CLOCK, 1,
				__MMIO_P2V(P7_VDEC_GBC) + VDEC_GBC_RESET, 0,
				0, NULL);
	else
		clk[vdec_clk] = clk_register_gbc(NULL, "vdec_clk", "proc_clk", 0,
				__MMIO_P2V(P7_VDEC_GBC) + VDEC_GBC_CLOCK, 1,
				__MMIO_P2V(P7_VDEC_GBC) + VDEC_GBC_RESET, 0,
				__MMIO_P2V(P7_VDEC_GBC) + VDEC_GBC_POWER,
				NULL);

	clk[usb0_clk] = clk_register_hspclk("usb0_clk",
	                                 __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_USB0_CLOCK, 3UL,
	                                 __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_USB0_RESET);
	clk[usb1_clk] = clk_register_hspclk("usb1_clk",
	                                 __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_USB1_CLOCK, 3UL,
	                                 __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_USB1_RESET);
	clk[dma_clk] = clk_register_hspclk("dma_clk",
	                                   __MMIO_P2V(P7_DMA_GBC) + DMA_GBC_CLOCK, 1,
	                                   __MMIO_P2V(P7_DMA_GBC) + DMA_GBC_RESET);
	clk[sdhci0_hostclk] = clk_register_hspclk("sdhci0_hostclk",
	                                          __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO0_AHB_CLOCK, 1,
	                                          __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO0_RESET);
	clk[sdhci0_xinclk] = clk_register_xin(NULL, "sdhci0_xinclk", "fast_clk",
	                                      __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO0_XIN_CLOCK);
	clk[sdhci0_sdclk] = clk_register_sd(NULL, "sdhci0_sdclk", "sdhci0_xinclk");
	clk[sdhci1_hostclk] = clk_register_hspclk("sdhci1_hostclk",
	                                          __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO1_AHB_CLOCK, 1,
	                                          __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO1_RESET);
	clk[sdhci1_xinclk] = clk_register_xin(NULL, "sdhci1_xinclk", "fast_clk",
	                                      __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO1_XIN_CLOCK);
	clk[sdhci1_sdclk] = clk_register_sd(NULL, "sdhci1_sdclk", "sdhci1_xinclk");
	clk[sdhci2_hostclk] = clk_register_hspclk("sdhci2_hostclk",
	                                          __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO2_AHB_CLOCK, 1,
	                                          __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO2_RESET);
	clk[sdhci2_xinclk] = clk_register_xin(NULL, "sdhci2_xinclk", "fast_clk",
	                                      __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SDIO2_XIN_CLOCK);
	clk[sdhci2_sdclk] = clk_register_sd(NULL, "sdhci2_sdclk", "sdhci2_xinclk");
	clk[nand_ctrlclk] = clk_register_hspclk("nand_ctrlclk",
	                                        __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_NAND_CLOCK, 1,
	                                        __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_NAND_RESET);
	clk[nand_phyclk] = clk_register_gbc(NULL, "nand_phyclk", "pll_nand_clk",
		                                CLK_SET_RATE_PARENT,
	                                    __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_NAND_CLOCK, 6,
										__MMIO_P2V(P7_HSP_GBC) + HSP_GBC_NAND_RESET, 1,
		                                0, NULL);

	if (rev == P7_CHIPREV_R1 || rev == P7_CHIPREV_R2) {
		clk[gpu_busclk] = clk_register_gate(NULL, "gpu_busclk", "proc_clk", 0,
		                                   (void __iomem *)__MMIO_P2V(P7_GPU_GBC) + GPU_GBC_BUS_CLOCK, 0,
		                                   0, NULL);
		clk[gpu_coreclk] = clk_register_gbc(NULL, "gpu_coreclk", "proc_clk", 0,
		                                    __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CORE_CLOCK, 3,
		                                    __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CORE_RESET, 0,
		                                    0, NULL);
	} else {
		clk[gpu_clk] = clk_register_gbc(NULL, "gpu_clk", "proc_clk", 0,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CLOCK, 0x1,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_RESET, 0,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_POWER, NULL);
		clk[gpu_pp0_clk] = clk_register_gbc(NULL, "gpu_pp0_clk", "gpu_clk", 0,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CLOCK, 0x2,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_RESET, 1,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_POWER, NULL);
		clk[gpu_pp1_clk] = clk_register_gbc(NULL, "gpu_pp1_clk", "gpu_clk", 0,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CLOCK, 0x4,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_RESET, 2,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_POWER, NULL);
		clk[gpu_pp2_clk] = clk_register_gbc(NULL, "gpu_pp2_clk", "gpu_clk", 0,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CLOCK, 0x8,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_RESET, 3,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_POWER, NULL);
		clk[gpu_pp3_clk] = clk_register_gbc(NULL, "gpu_pp3_clk", "gpu_clk", 0,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_CLOCK, 0x10,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_RESET, 4,
		                                __MMIO_P2V(P7_GPU_GBC) + GPU_GBC_POWER, NULL);
	}
	clk[spi_clk] = clk_register_gbc(NULL, "spi_clk", "low_clk", 0,
	                                __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SPI_CLOCK, 1,
	                                __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SPI_RESET, 0,
	                                0, &p7_spi_clk_lock);
	clk[spi0_clk] = clk_register_gate(NULL, "spi0_clk", "spi_clk", 0,
	                                  (void __iomem *) __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SPI_RESET, 8,
	                                  CLK_GATE_SET_TO_DISABLE, &p7_spi_clk_lock);
	clk[spi1_clk] = clk_register_gate(NULL, "spi1_clk", "spi_clk", 0,
	                                  (void __iomem *) __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SPI_RESET, 9,
	                                  CLK_GATE_SET_TO_DISABLE, &p7_spi_clk_lock);
	clk[spi2_clk] = clk_register_gate(NULL, "spi2_clk", "spi_clk", 0,
	                                  (void __iomem *) __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SPI_RESET, 10,
	                                  CLK_GATE_SET_TO_DISABLE, &p7_spi_clk_lock);
	clk[spi3_clk] = clk_register_gate(NULL, "spi3_clk", "spi_clk", 0,
	                                  (void __iomem *) __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_SPI_RESET, 11,
	                                  CLK_GATE_SET_TO_DISABLE, &p7_spi_clk_lock);

	if (rev == P7_CHIPREV_R1 || rev == P7_CHIPREV_R2){
		/* XXX locking */
		clk[aai_ctrlclk] = clk_register_gbc(NULL, "aai_ctrlclk", "low_clk", 0,
		                                    __MMIO_P2V(P7_AAI_GBC) + AAI_GBC_CLOCK, 1 << 16,
		                                    __MMIO_P2V(P7_AAI_GBC) + AAI_GBC_RESET, 0,
		                                    0, NULL);
		/* XXX reset : logic inverted ??? */
		clk[spdif_clk] = clk_register_gate(NULL, "spdif_clk", "pll_aai_clk",
		                                  CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
		                                 (void __iomem *) __MMIO_P2V(P7_AAI_GBC) + AAI_GBC_RESET, 1, /* in reality it is RESET_SPDIF, not a clk enable */
										  CLK_GATE_SET_TO_DISABLE, NULL);
		clk[i2s_clk] = clk_register_gate(NULL, "i2s_clk", "pll_aai_clk",
		                                CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
		                                (void __iomem *)__MMIO_P2V(P7_AAI_GBC) + AAI_GBC_CLOCK, 0, 0, NULL);
		clk[pcm1_clk] = clk_register_gate(NULL, "pcm1_clk", "fast_clk", 0,
		                                 (void __iomem *)__MMIO_P2V(P7_AAI_GBC) + AAI_GBC_CLOCK, 8, 0, NULL);
		clk[pcm2_clk] = clk_register_gate(NULL, "pcm2_clk", "fast_clk", 0,
		                                 (void __iomem *)__MMIO_P2V(P7_AAI_GBC) + AAI_GBC_CLOCK, 9, 0, NULL);
	}else{
		/* XXX locking */
		clk[aai_ctrlclk] = clk_register_gbc(NULL, "aai_ctrlclk",
		                                    "low_clk", 0,
		                                    __MMIO_P2V(P7_AAI_GBC_R3) + AAI_GBC_CLOCK, 1,
		                                    __MMIO_P2V(P7_AAI_GBC_R3) + AAI_GBC_RESET, 0,
		                                    __MMIO_P2V(P7_AAI_GBC_R3) + AAI_GBC_POWER,
		                                    NULL);

		clk[spdif_clk] = clk_register_gbc(NULL, "spdif_clk", "pll_aai_clk",
		                                CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
		                                __MMIO_P2V(P7_AAI_GBC_R3) + AAI_GBC_CLOCK, 1 << 1,
		                                __MMIO_P2V(P7_AAI_GBC_R3) + AAI_GBC_RESET, 1,
		                                0, NULL);
		clk[i2s_clk] = clk_register_gate(NULL, "i2s_clk", "pll_aai_clk",
		                                CLK_SET_RATE_PARENT | CLK_SET_RATE_GATE,
		                                (void __iomem *)__MMIO_P2V(P7_AAI_LBC) + AAI_GBC_CLOCK, 0, 0, NULL);
		clk[pcm1_clk] = clk_register_gate(NULL, "pcm1_clk", "fast_clk", 0,
		                                 (void __iomem *)__MMIO_P2V(P7_AAI_LBC) + AAI_GBC_CLOCK, 8, 0, NULL);
		clk[pcm2_clk] = clk_register_gate(NULL, "pcm2_clk", "fast_clk", 0,
		                                 (void __iomem *)__MMIO_P2V(P7_AAI_LBC) + AAI_GBC_CLOCK, 9, 0, NULL);
	}

	clk[avi_ctrlclk] = clk_register_gbc(NULL, "avi_ctrlclk", "proc_clk", 0,
	                                    __MMIO_P2V(P7_AVI_GBC) + AVI_GBC_CLOCK, 1,
	                                    __MMIO_P2V(P7_AVI_GBC) + AVI_GBC_RESET, 0,
	                                    0, NULL);
	clk[lcd0_clk] = clk_register_composite(NULL, "avi_lcd0clk",
				p7_avi_lcd0clk_parents, ARRAY_SIZE(p7_avi_lcd0clk_parents),
				NULL, NULL,
				&p7_avi_pll_div_hw.hw, &clk_fixed_factor_ops,
				&p7_avi0_gate_hw.hw, &clk_gate_ops,
				CLK_SET_RATE_PARENT);
	clk[lcd1_clk] = clk_register_composite(NULL, "avi_lcd1clk",
				p7_avi_lcd1clk_parents, ARRAY_SIZE(p7_avi_lcd1clk_parents),
				NULL, NULL,
				&p7_avi_pll_div_hw.hw, &clk_fixed_factor_ops,
				&p7_avi1_gate_hw.hw, &clk_gate_ops,
				CLK_SET_RATE_PARENT);
	if (rev >= P7_CHIPREV_R2) {
		clk[eth_tx_clk] = clk_register_composite(
						NULL, "eth_tx_clk",
						eth_tx_parent_names, ARRAY_SIZE(eth_tx_parent_names),
						&p7_eth_tx_mux_hw.hw, &clk_mux_ops,
						&p7_eth_tx_div_hw.hw, &clk_divider_ops,
						&p7_eth_tx_gate_hw.hw, &clk_gate_ops,
						CLK_SET_PARENT_GATE | CLK_SET_RATE_GATE |
						CLK_SET_RATE_PARENT);
		/* XXX bit 1 and 2 is not for ctrl clk */
		clk[eth_ctrlclk] = clk_register_hspclk("eth_ctrlclk",
		                                    __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_ETHERNET_CLOCK, 7,
		                                    __MMIO_P2V(P7_HSP_GBC) + HSP_GBC_ETHERNET_RESET);
	}
	clk[can_clk] = clk_register_divider_table(
			NULL, "can_clk", "pll_usb_clk", CLK_SET_RATE_PARENT,
			(void __iomem *) __MMIO_P2V(P7_SYS_CLKGEN_CFG_CAN_DIV),
			0, 2, CLK_DIVIDER_ALLOW_ZERO,
			can_clk_div, &can_clk_lock);

	clk[can0_clk] = clk_register_gbc(NULL, "can0_clk", "can_clk",
	                                 CLK_SET_RATE_PARENT,
	                                 __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_CCAN0_CLOCK, 1,
	                                 __MMIO_P2V(P7_LSP_GBC) + LSP_GBC_CCAN0_RESET, 0, 0, NULL);


	clk[can1_clk] = clk_register_gbc(NULL, "can1_clk", "can_clk",
				CLK_SET_RATE_PARENT,
				__MMIO_P2V(P7_LSP_GBC) + LSP_GBC_CCAN1_CLOCK, 1,
				__MMIO_P2V(P7_LSP_GBC) + LSP_GBC_CCAN1_RESET, 0, 0,
				NULL);

	if (rev == P7_CHIPREV_R3) {
		clk[crypto_clk] = clk_register_hspclk("crypto_clk",
				__MMIO_P2V(P7_HSP_GBC) + HSP_GBC_CRYPTO_CLOCK, 1,
				__MMIO_P2V(P7_HSP_GBC) + HSP_GBC_CRYPTO_RESET);
	}

	for(i = 0; i < ARRAY_SIZE(clk); i++) {
		if (clk[i] && IS_ERR(clk[i]))
			pr_err("failed to register clk %d (%ld)\n",
			       i, PTR_ERR(clk[i]));
	}

	clk_register_clkdev(clk[sys_clk],        NULL,          "sys_clk");
	clk_register_clkdev(clk[cpu_clk],        NULL,          "cpu_clk");
	clk_register_clkdev(clk[i2cm0_clk],      NULL,          "p7-i2cm.0");
	clk_register_clkdev(clk[i2cm1_clk],      NULL,          "p7-i2cm.1");
	clk_register_clkdev(clk[i2cm2_clk],      NULL,          "p7-i2cm.2");
	clk_register_clkdev(clk[i2cs0_clk],      NULL,          "p7-i2cs.0");
	clk_register_clkdev(clk[i2cs1_clk],      NULL,          "p7-i2cs.1");
	clk_register_clkdev(clk[i2cs2_clk],      NULL,          "p7-i2cs.2");
	clk_register_clkdev(clk[gpio_clk],       NULL,          "p7-gpio.0");
	clk_register_clkdev(clk[uart0_clk],      NULL,          "px-uart.0");
	clk_register_clkdev(clk[uart1_clk],      NULL,          "px-uart.1");
	clk_register_clkdev(clk[uart2_clk],      NULL,          "px-uart.2");
	clk_register_clkdev(clk[uart3_clk],      NULL,          "px-uart.3");
	clk_register_clkdev(clk[uart4_clk],      NULL,          "px-uart.4");
	clk_register_clkdev(clk[uart5_clk],      NULL,          "px-uart.5");
	clk_register_clkdev(clk[uart6_clk],      NULL,          "px-uart.6");
	clk_register_clkdev(clk[uart7_clk],      NULL,          "px-uart.7");
	clk_register_clkdev(clk[venc_clk],       NULL,          "hx280-venc.0");
	clk_register_clkdev(clk[vdec_clk],       NULL,          "hx270-vdec.0");
	clk_register_clkdev(clk[usb0_clk],       "usb_clk.0",   NULL);
	clk_register_clkdev(clk[usb1_clk],       "usb_clk.1",   NULL);
	clk_register_clkdev(clk[dma_clk],        NULL,          "pl330-dma.0");
	clk_register_clkdev(clk[sdhci0_hostclk], "host",        "acs3-sdhci.0");
	clk_register_clkdev(clk[sdhci0_xinclk],  "xin",         "acs3-sdhci.0");
	clk_register_clkdev(clk[sdhci0_sdclk],   "sd",          "acs3-sdhci.0");
	clk_register_clkdev(clk[sdhci1_hostclk], "host",        "acs3-sdhci.1");
	clk_register_clkdev(clk[sdhci1_xinclk],  "xin",         "acs3-sdhci.1");
	clk_register_clkdev(clk[sdhci1_sdclk],   "sd",          "acs3-sdhci.1");
	clk_register_clkdev(clk[sdhci2_hostclk], "host",        "acs3-sdhci.2");
	clk_register_clkdev(clk[sdhci2_xinclk],  "xin",         "acs3-sdhci.2");
	clk_register_clkdev(clk[sdhci2_sdclk],   "sd",          "acs3-sdhci.2");
	clk_register_clkdev(clk[nand_phyclk],    "phy",         "cast-nand.0");
	clk_register_clkdev(clk[nand_ctrlclk],   "ctrl",        "cast-nand.0");
	clk_register_clkdev(clk[low_clk],        NULL,          "p7_pwm.0");
	clk_register_clkdev(clk[spi_clk],        "p7-spi",      "common");
	clk_register_clkdev(clk[spi0_clk],       "p7-spi",      "0");
	clk_register_clkdev(clk[spi1_clk],       "p7-spi",      "1");
	clk_register_clkdev(clk[spi2_clk],       "p7-spi",      "2");
	clk_register_clkdev(clk[spi3_clk],       "p7-spi",      "3");
	clk_register_clkdev(clk[aai_ctrlclk],    "ctrl_clk",    "aai.0");
	clk_register_clkdev(clk[i2s_clk],        "i2s_clk",     "aai.0");
	clk_register_clkdev(clk[pcm1_clk],       "pcm1_clk",    "aai.0");
	clk_register_clkdev(clk[pcm2_clk],       "pcm2_clk",    "aai.0");
	clk_register_clkdev(clk[spdif_clk],      "spdif_clk",   "aai.0");
	clk_register_clkdev(clk[avi_ctrlclk],    "ctrl",        "avi.0");
	clk_register_clkdev(clk[lcd0_clk],       "lcd0",        "avi.0");
	clk_register_clkdev(clk[lcd1_clk],       "lcd1",        "avi.0");
	clk_register_clkdev(clk[twd_clk],        NULL,          "smp_twd");
	clk_register_clkdev(clk[can0_clk], 	 NULL, 		"c_can_platform.0");
	clk_register_clkdev(clk[can1_clk], 	 NULL, 		"c_can_platform.1");
	if (rev == P7_CHIPREV_R3) {
		clk_register_clkdev(clk[i2cms_clk],      NULL,          "p7-i2cm.3");
		clk_register_clkdev(clk[crypto_clk],     NULL,          "p7-crypto.0");
	}

	if (rev >= P7_CHIPREV_R2) {
		clk_register_clkdev(clk[eth_tx_clk],      "eth_tx_clk", "stmmaceth");
		clk_register_clkdev(clk[pll_usb_clk],     NULL,         "pll_usb_clk");
		clk_register_clkdev(clk[eth_ctrlclk],     "stmmaceth",  "stmmaceth");

	}

	if (rev == P7_CHIPREV_R3) {
		clk_register_clkdev(clk[gpu_clk],        "gpu",         "p7gpu.0");
		clk_register_clkdev(clk[gpu_pp0_clk],    "gpu_pp0",     "p7gpu.0");
		clk_register_clkdev(clk[gpu_pp1_clk],    "gpu_pp1",     "p7gpu.0");
		clk_register_clkdev(clk[gpu_pp2_clk],    "gpu_pp2",     "p7gpu.0");
		clk_register_clkdev(clk[gpu_pp3_clk],    "gpu_pp3",     "p7gpu.0");
		clk_register_clkdev(clk[pll_avi0_clk],     NULL,         "pll_avi0_clk");
	}
	else {
		clk_register_clkdev(clk[gpu_busclk],     "bus",         "p7gpu.0");
		clk_register_clkdev(clk[gpu_coreclk],    "core",        "p7gpu.0");
		if  (rev == P7_CHIPREV_R2)
			clk_register_clkdev(clk[pll_avi1_clk],     NULL,         "pll_avi1_clk");
	}

	return 0;
}

