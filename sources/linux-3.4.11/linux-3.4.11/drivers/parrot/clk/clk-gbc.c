/*
 * drivers/parrot/clk/clk-gbc.c
 *
 * Copyright (C) 2010-2013 Parrot S.A.
 *
 * @author	Gregor Boirie <gregor.boirie@parrot.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

//#define DEBUG
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include "p7clk-provider.h"
#include "clock.h"

#define to_p7clk_gbc(_hw) container_of(_hw, struct clk_p7_gbc, hw)

/**
 * p7_enable_clkgbc() - Enable clock at the GBC level and reset child block if
 * required.
 * @gbc: GBC clock descriptor
 */
static int p7_enable_clkgbc(struct p7_clk_gbc const* gbc)
{
	/* /!\ WARNING /!\
	 *
	 * This code could be written using __raw_(read|write)l, however, due to
	 * a hardware bug, bursts to the AVI R2 GBC registers can cause a lock
	 * up of the AXI interconnect. Other revisions of the IP are not
	 * concerned.
	 */
	u32	clk;
	u32	clk_on;
	u32	clk_off;

	pr_devel("p7 gdc enable %lx\n", gbc->clk_reg);
	if (!gbc->clk_reg)
		return 0;

	clk = readl(gbc->clk_reg);
	clk_on = clk | gbc->clk_msk;
	clk_off = clk & ~gbc->clk_msk;

	if (gbc->reset_reg) {
		u32	reset;
		u32	reset_on;
		u32	reset_off;

		reset = readl(gbc->reset_reg);
		reset_on = reset | (1 << gbc->reset_offset);
		reset_off = reset & ~(1 << gbc->reset_offset);

		if (clk == clk_on && reset == reset_off)
			/* if the clock is already enabled and the IP is not
			 * reset we don't have anything left to do. */
			return 0;

		/* We have to disable the clock when we (un)assert the reset
		 * (see section 3.2.4: Clock and reset sequence of the user
		 * manual). */
		writel(clk_off, gbc->clk_reg);

		/* Force reset of the GBC child block */
		writel(reset_on, gbc->reset_reg);

		/* Enable clock to make the synchronous reset take effect */
		writel(clk_on, gbc->clk_reg);
		cpu_relax();
		/* Then disable once again to de-reset the block */
		writel(clk_off, gbc->clk_reg);
		writel(reset_off, gbc->reset_reg);

		/* At this point we're sure the block is correctly reset and the
		 * clock is disabled */
	}

	/* Enable clock */
	writel(clk_on, gbc->clk_reg);

	return 0;
}

/**
 * p7_disable_clkgbc() - Disable clock at the GBC level.
 * @gbc: GBC clock descriptor
 */
static void p7_disable_clkgbc(struct p7_clk_gbc const* gbc)
{
	pr_devel("p7 gdc disable %lx\n", gbc->clk_reg);
	/* Disable clock. */
	writel(readl(gbc->clk_reg) & ~gbc->clk_msk, gbc->clk_reg);
}

/**
 * p7_gbc_enable_clk() - Enable a gbc clock.
 * @clk_hw: a pointer to a clk_hw embedded in a @clk_p7_gbc struct.
 */
static int p7_gbc_enable_clk(struct clk_hw* clk_hw)
{
	struct clk_p7_gbc *clk = to_p7clk_gbc(clk_hw);
	int ret;

	if (clk->lock)
		spin_lock(clk->lock);
	ret = p7_enable_clkgbc(&clk->gbc);
	if (clk->lock)
		spin_unlock(clk->lock);

	return ret;
}

/**
 * p7_gbc_disable_clk() - Disable a gbc clock.
 * @clk_hw: a pointer to a clk_hw embedded in a @clk_p7_gbc struct.
 */
static void p7_gbc_disable_clk(struct clk_hw* clk_hw)
{
	struct clk_p7_gbc *clk = to_p7clk_gbc(clk_hw);

	if (clk->lock)
		spin_lock(clk->lock);
	p7_disable_clkgbc(&clk->gbc);
	if (clk->lock)
		spin_unlock(clk->lock);
}

static int p7_gbc_is_enabled_clk(struct clk_hw *clk_hw)
{
	struct clk_p7_gbc *clk = to_p7clk_gbc(clk_hw);
	struct p7_clk_gbc const* gbc = &clk->gbc;
	int ret = 0;

	/* this callback is used to disable unused clock, return
	 1 if at least one bit is enabled
	 */
	if (readl(gbc->clk_reg) & gbc->clk_msk)
		ret = 1;

	pr_devel("p7 is gbc %s %lx\n", ret?"enabled":"disabled",
			gbc->clk_reg);
	return ret;
}

static int p7_gbc_is_prepared_clk(struct clk_hw *clk_hw)
{
	struct clk_p7_gbc *clk = to_p7clk_gbc(clk_hw);
	struct p7_clk_gbc const* gbc = &clk->gbc;
	int ret = 0;
	u32 power;

	if (!gbc->power_reg)
	   return 1;

	power = readl(gbc->power_reg);

	if (power & (P7_POWER_ON << (gbc->reset_offset * 4)))
		ret = 1;
	else if (power & (P7_POWER_OFF << (gbc->reset_offset * 4)))
		ret = 0;
	else {
		ret = 1;
		pr_warn("p7 gbc invalid prepared state 0x%x %lx\n",
				power, gbc->clk_reg);
	}

	pr_devel("p7 is gbc %s %lx\n", ret?"prepared":"unprepared",
			gbc->clk_reg);

	return ret;
}

static int p7_gbc_prepare_clk(struct clk_hw *clk_hw)
{
	struct clk_p7_gbc *clk = to_p7clk_gbc(clk_hw);
	struct p7_clk_gbc const* gbc = &clk->gbc;
	unsigned int timeout = 0;
	u32 power;

	if (!gbc->power_reg)
		return 0;

	pr_devel("p7 gbc prepare 0x%lx\n", gbc->power_reg);
	power = readl(gbc->power_reg);
	if ((power & 0x6) == 0)
		pr_warn("p7 gbc invalid prepared state 0x%x %lx\n",
				power, gbc->clk_reg);

	writel(power |
			(P7_REQUEST_POWER_ON << (gbc->reset_offset * 4)),
			gbc->power_reg);
	/*wait until power on flag is on*/
	do {
		if (timeout++ > 0x8ffff) {
			pr_warn("p7 gbc prepare timeout 0x%lx = 0x%x\n",
					gbc->power_reg, readl(gbc->power_reg));
			return -ENODEV;
		}
		cpu_relax();
	} while (!(readl(gbc->power_reg) & (P7_POWER_ON << (gbc->reset_offset * 4))));

	return 0;
}

static void p7_gbc_unprepare_clk(struct clk_hw *clk_hw)
{
	struct clk_p7_gbc *clk = to_p7clk_gbc(clk_hw);
	struct p7_clk_gbc const* gbc = &clk->gbc;
	unsigned int timeout = 0;

	if (!gbc->power_reg)
		return;

	pr_devel("p7 gbc unprepare 0x%lx\n", gbc->power_reg);
	writel(readl(gbc->power_reg) &
			~(P7_REQUEST_POWER_ON << (gbc->reset_offset * 4)),
			gbc->power_reg);
	/*wait until power off flag is on*/
	do {
		if (timeout++ > 0x8ffff) {
			pr_warn("p7 gbc unprepare timeout 0x%lx = 0x%x\n",
					gbc->power_reg, readl(gbc->power_reg));
			break;
		}
		cpu_relax();
	} while (!(readl(gbc->power_reg) & (P7_POWER_OFF << (gbc->reset_offset * 4))));
}


struct clk_ops const p7_clk_gbc_ops = {
	.enable	    = p7_gbc_enable_clk,
	.disable    = p7_gbc_disable_clk,
	.is_enabled =  p7_gbc_is_enabled_clk,
	.prepare    = p7_gbc_prepare_clk,
	.unprepare  = p7_gbc_unprepare_clk,
	.is_prepared=  p7_gbc_is_prepared_clk,
};

struct clk *clk_register_gbc(struct device *dev, const char *name,
		const char *parent_name, unsigned long flags,
		unsigned long clk_reg, u32 clk_msk,
		unsigned long reset_reg, u32 reset_offset,
		unsigned long power_reg,
		 spinlock_t* lock)
{
	struct clk_p7_gbc *gbc;
	struct clk *clk;
	struct clk_init_data init;

	gbc = kzalloc(sizeof(struct clk_p7_gbc), GFP_KERNEL);
	if (!gbc) {
		pr_err("%s: could not allocate gbc clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &p7_clk_gbc_ops;
	init.flags = flags;
	init.parent_names = (parent_name ? &parent_name: NULL);
	init.num_parents = (parent_name ? 1 : 0);

	gbc->gbc.clk_reg = clk_reg;
	gbc->gbc.clk_msk = clk_msk;
	gbc->gbc.power_reg = power_reg;
	gbc->gbc.reset_reg = reset_reg;
	gbc->gbc.reset_offset = reset_offset;
	gbc->lock = lock;
	gbc->hw.init = &init;

	pr_devel("p7 gbc register %s 0x%lx 0x%lx\n", name, reset_reg, power_reg);
	/* if there is no reset : use clk_register_gate ! */
	WARN_ON(reset_reg == 0);

	clk = clk_register(dev, &gbc->hw);

	if (IS_ERR(clk))
		kfree(gbc);

	return clk;
}
EXPORT_SYMBOL_GPL(clk_register_gbc);
