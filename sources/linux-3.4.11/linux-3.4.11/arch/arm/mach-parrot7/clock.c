/*
 *	linux/arch/arm/mach-parrot7/clock.c
 *
 *	Copyright (C) 2010 Parrot S.A.
 *
 * @author	Gregor Boirie <gregor.boirie@parrot.com>
 * @date  05-Nov-2010
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

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include <asm/io.h>
#include <mach/p7.h>
#include "common.h"
#include "system.h"
#include "clock.h"
#include "gpu.h"
#include "clk/clock.h"

int p7_populate_clock_tree(int low_sysclk) __init;

#ifdef CONFIG_PM_SLEEP

static u32 clks_reg_save[13];

static int p7_clk_suspend(void)
{
	int idx = 0;
	/* pll_fast_clk */
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_FAST));
	/* sys_clk */
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_SYS_DIV));
	/* cpu_clk */
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_CPU_DIV));
	/* pll */
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI00));
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI01));
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI10));
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI11));
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_AAI));
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_NAND));
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_USB));

	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_TRACE));
	/* dll clk (ethernet) */
	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_ETHERNET));

	clks_reg_save[idx++] = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_CAN_DIV));

	WARN_ON(idx != ARRAY_SIZE(clks_reg_save));
	return 0;
}

static void p7_clk_wait_pll(int pll_reg, int status_bit)
{
	int count = 0;

	/* ignore reset or bypass mode */
	if ((pll_reg & (1<<22)) || (pll_reg & (1<<21)))
			return;

	/* ignore disabled pll */
	if (status_bit != 0 && !(pll_reg & (1<<20)))
		return;

	while (!(readl(__MMIO_P2V(P7_SYS_CLKGEN_STATUS)) & (1<<status_bit))) {
		if (count++ > 10000) {
			printk("timeout while waiting for pll %d (cfg=%x stat %x)\n",
					status_bit, pll_reg,
					readl(__MMIO_P2V(P7_SYS_CLKGEN_STATUS)));
			udelay(1);
			break;
		}
	}
}

static void p7_clk_resume(void)
{
	int idx = 0;
	/* Disable fast clk : switch to clkin */
	clks_reg_save[0] &= ~(1 << 20);
	p7_write_pll_conf(__MMIO_P2V(P7_SYS_CLKGEN_CFG_FAST), clks_reg_save[idx]);
	/* Wait for PLL stabilization */
	p7_clk_wait_pll(clks_reg_save[idx], 0);
	/* Re-enable fast clock */
	clks_reg_save[0] |= 1 << 20;
	p7_write_pll_conf(__MMIO_P2V(P7_SYS_CLKGEN_CFG_FAST), clks_reg_save[idx]);
	idx++;

	/* sys_clk */
	writel(clks_reg_save[idx++], __MMIO_P2V(P7_SYS_CLKGEN_CFG_SYS_DIV));
	/* cpu_clk */
	writel(clks_reg_save[idx++], __MMIO_P2V(P7_SYS_CLKGEN_CFG_CPU_DIV));

	/* pll conf */
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI00));
	p7_clk_wait_pll(clks_reg_save[idx], 2);
	idx++;
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI01));
	p7_clk_wait_pll(clks_reg_save[idx]|(clks_reg_save[idx-1] & (1<<20)), 3);
	idx++;
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI10));
	p7_clk_wait_pll(clks_reg_save[idx], 4);
	idx++;
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_AVI11));
	p7_clk_wait_pll(clks_reg_save[idx]|(clks_reg_save[idx-1] & (1<<20)), 5);
	idx++;
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_AAI));
	p7_clk_wait_pll(clks_reg_save[idx], 6);
	idx++;
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_NAND));
	p7_clk_wait_pll(clks_reg_save[idx], 7);
	idx++;
	writel(clks_reg_save[idx], __MMIO_P2V(P7_SYS_CLKGEN_CFG_USB));
	p7_clk_wait_pll(clks_reg_save[idx], 8);
	idx++;

	writel(clks_reg_save[idx++], __MMIO_P2V(P7_SYS_CLKGEN_CFG_TRACE));
	/* dll clk (ethernet) */
	writel(clks_reg_save[idx++], __MMIO_P2V(P7_SYS_CLKGEN_CFG_ETHERNET));

	/* need usb pll */
	writel(clks_reg_save[idx++], __MMIO_P2V(P7_SYS_CLKGEN_CFG_CAN_DIV));

	WARN_ON(idx != ARRAY_SIZE(clks_reg_save));
}

struct syscore_ops p7_clk_syscore_ops = {
	.suspend	= p7_clk_suspend,
	.resume		= p7_clk_resume,
};

#endif /* CONFIG_PM_SLEEP */

void __init p7_init_clk(int low_sysclk)
{
	p7_populate_clock_tree(low_sysclk);

#ifdef CONFIG_PM_SLEEP
	register_syscore_ops(&p7_clk_syscore_ops);
#endif
}
