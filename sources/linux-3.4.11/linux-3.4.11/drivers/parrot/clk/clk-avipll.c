
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include "p7clk-provider.h"
#include "clock.h"


#define to_p7clk_avipll(_hw) container_of(_hw, struct clk_p7_avipll, hw)

/**
 * p7_prepare_avipll() - Prepare a two-level PLL
 * @clk_hw: a pointer to @clk_hw in a @clk_p7_avipll.
 */
static int p7_prepare_avipll(struct clk_hw* clk_hw)
{
	struct clk_p7_avipll const* const pll = to_p7clk_avipll(clk_hw);
	unsigned long const             master_msk = BIT(pll->lock_bit_idx);
	unsigned long const             slave_msk = BIT(pll->lock_bit_idx + 1);
	unsigned long const             status_addr = pll->lock_reg;
	unsigned long const             master_addr = pll->pll_conf_reg;
	unsigned long const             slave_addr = master_addr + sizeof(u32);
	union p7_pll_reg                master = { .word = readl(master_addr) };
	union p7_pll_reg                slave = { .word = readl(slave_addr) };
	int err = 0;

	if (! master.fields.reset &&
	    (readl(status_addr) & master_msk)) {
		/* Master PLL is powered on, and locked: already enabled.
		 * Note that the pll should not be gated at this point.
		 */
		if (! slave.fields.reset &&
		    (readl(status_addr) & slave_msk))
			/* Slave is also enabled: nothing to do. */
			return err;
	}
	else {
		master.fields.reset = 0;    /* power on */
		p7_write_pll_conf(master_addr, master.word);
		if (! master.fields.bypass) {
			err = p7_activate_pll(status_addr, master_msk);
			if (err)
				return err;
		}
	}

	/* Setup slave PLL. */
	slave.fields.reset = 0; /* power on */
	p7_write_pll_conf(slave_addr, slave.word);
	if (! slave.fields.bypass)
		err = p7_activate_pll(status_addr, slave_msk);

	return err;
}

/**
 * p7_unprepare_avipll() - Unrepare a two-level PLL
 * @clk_hw: a pointer to @clk_hw in a @clk_p7_avipll.
 */
static void p7_unprepare_avipll(struct clk_hw* clk_hw)
{
	struct clk_p7_avipll const* const pll = to_p7clk_avipll(clk_hw);
	unsigned long const             master_addr = pll->pll_conf_reg;
	unsigned long const             slave_addr = master_addr + sizeof(u32);
	union p7_pll_reg                master = { .word = readl(master_addr) };
	union p7_pll_reg                slave = { .word = readl(slave_addr) };

	/* power off slave PLL. */
	slave.fields.reset = 1;
	writel(slave.word, slave_addr);

	/* Power off master PLL. */
	master.fields.reset = 1;
	writel(master.word, master_addr);
}

/**
 * p7_recalc_avipll_rate() - Return two level/AVI PLL rate.
 * @clk_hw: a pointer to @clk_hw in a @clk_p7_avipll.
 * @parent_rate: rate of the parent clock.
 */
static unsigned long p7_recalc_avipll_rate(struct clk_hw* clk_hw,
                                           unsigned long parent_rate)
{
	struct clk_p7_avipll * const pll = to_p7clk_avipll(clk_hw);
	unsigned long const     master_addr = pll->pll_conf_reg;
	unsigned long const     slave_addr = master_addr + sizeof(u32);
	union p7_pll_reg        master;
	union p7_pll_reg        slave;
	unsigned long                   rate;
	unsigned long                   flags;

	spin_lock_irqsave(pll->lock, flags);
	master.word = readl(master_addr);
	slave.word = readl(slave_addr);
	spin_unlock_irqrestore(pll->lock, flags);
	rate = p7_pll_rate(master, parent_rate);
	rate = p7_pll_rate(slave, rate);

	return rate;
}

/**
 * p7_set_avipll_rate() - Set two level/AVI PLL frequency.
 * @clk_hw: a pointer to @clk_hw in a @clk_p7_avipll.
 * @rate: frequency to configure.
 * @parent_rate: parent clock's rate.
 *
 * Warning: frequency passed in argument will likely be rounded to a possible
 * PLL hierarchy operation point.
 */
static int p7_set_avipll_rate(struct clk_hw* clk_hw, unsigned long rate,
                              unsigned long parent_rate)
{
	struct clk_p7_avipll * const pll = to_p7clk_avipll(clk_hw);
	unsigned long const     master_addr = pll->pll_conf_reg;
	unsigned long const     slave_addr = master_addr + sizeof(u32);
	union p7_pll_reg        master, new_master;
	union p7_pll_reg        slave, new_slave;
	int                     idx, gate;
	unsigned long           flags = 0;
	int                     err = 0;

	idx = p7_get_clkcfg(pll->cfg_idx, rate, pll->cfg_nr);
	if (idx < 0)
		return idx;
	new_master.word = pll->master_cfg[idx];
	new_slave.word = pll->slave_cfg[idx];

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);
	master.word = readl(master_addr);
	slave.word = readl(slave_addr);


	if (master.fields.cfg == new_master.fields.cfg &&
	    master.fields.bypass == new_master.fields.bypass &&
	    slave.fields.cfg == new_slave.fields.cfg &&
	    slave.fields.bypass == new_slave.fields.bypass)
		goto unlock;

	gate = !! master.fields.enable;
	if (gate) {
		/*
		 * Drivers are responsible for calling clk_disable (gating the
		 * clock) before changing the rate. We do as sanity check here,
		 * but we shouldn't even look at this bit here...
		 */
		err = -EBUSY;
		goto unlock;
	}

	/*
	 * write the config in both PLL and activate after
	 * the spinlock is released
	 */
	if (master.fields.cfg != new_master.fields.cfg ||
	    master.fields.bypass != new_master.fields.bypass) {
		master.fields.cfg = new_master.fields.cfg;
		master.fields.bypass = new_master.fields.bypass;
		p7_write_pll_conf(master_addr, master.word);
	}

	if (slave.fields.cfg != new_slave.fields.cfg ||
	    slave.fields.bypass != new_slave.fields.bypass) {
		slave.fields.cfg = new_slave.fields.cfg;
		slave.fields.bypass = new_slave.fields.bypass;
		p7_write_pll_conf(slave_addr, slave.word);
	}

	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
	if (__clk_get_prepare_count(clk_hw->clk)) {
		err = p7_activate_pll(pll->lock_reg, BIT(pll->lock_bit_idx));
		if (err)
			return err;
		err = p7_activate_pll(pll->lock_reg, BIT(pll->lock_bit_idx + 1));
	}
	return err;

unlock:
	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
	return err;
}

/**
 * p7_round_avipll_rate() - Round two level/AVI PLL frequency.
 * @clk_hw: a pointer to @clk_hw in a @clk_p7_avipll.
 * @rate: frequency to configure.
 * @parent_rate: parent clock's rate pointer.
 * 
 * Round rate to the closest supported value given the parent clock rate.
 */
static long p7_round_avipll_rate(struct clk_hw* clk_hw, unsigned long rate,
                              unsigned long* parent_rate)
{
	struct clk_p7_avipll* const    pll = to_p7clk_avipll(clk_hw);
	int const               idx = p7_get_clkcfg(pll->cfg_idx,
	                                            rate,
	                                            pll->cfg_nr);
	if (idx < 0)
		return idx;

	return (long) pll->cfg_idx[idx];
}


struct clk_ops const p7_clk_avipll_ops = {
	.prepare     = &p7_prepare_avipll,
	.unprepare   = &p7_unprepare_avipll,
	.recalc_rate = &p7_recalc_avipll_rate,
	.set_rate    = &p7_set_avipll_rate,
	.round_rate  = &p7_round_avipll_rate,
};

struct clk *clk_register_avipll(struct device *dev, const char *name,
                                const char *parent_name, const unsigned long flags,
                                const unsigned long lock_reg, const u8 lock_bit_idx,
				const unsigned long pll_reg, unsigned long const * const cfg_idx,
				unsigned long const * const master_cfg, unsigned long const * const slave_cfg,
				const size_t cfg_nr, spinlock_t *lock)
{
	struct clk_p7_avipll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(struct clk_p7_pll), GFP_KERNEL);
	if (!pll) {
		pr_err("%s: could not allocate clk pll\n", __func__);
		clk = ERR_PTR(-ENOMEM);
		goto err;
	}

	init.name = name;
	init.ops = &p7_clk_avipll_ops;
	init.flags = flags | CLK_SET_RATE_GATE;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->lock_reg = lock_reg;
	pll->lock_bit_idx = lock_bit_idx;
	pll->pll_conf_reg = pll_reg;
	pll->cfg_idx = cfg_nr ? kmemdup(cfg_idx, cfg_nr * sizeof(unsigned long), GFP_KERNEL) : NULL;
	pll->master_cfg = cfg_nr ? kmemdup(master_cfg, cfg_nr * sizeof(unsigned long), GFP_KERNEL) : NULL;
	pll->slave_cfg = cfg_nr ? kmemdup(slave_cfg, cfg_nr * sizeof(unsigned long), GFP_KERNEL) : NULL;
	pll->cfg_nr = cfg_nr;
	pll->lock = lock;
	pll->hw.init = &init;

	if (cfg_nr && ((!pll->cfg_idx) || (!pll->master_cfg) || (!pll->slave_cfg))) {
		pr_err("%s: could not allocate memory for pll info\n", __func__);
		clk = ERR_PTR(-ENOMEM);
		goto free_pll_content;
	}

	clk = clk_register(dev, &pll->hw);
	if (! IS_ERR(clk))
		return clk;

free_pll_content:
	if (pll->cfg_idx)
		kfree(pll->cfg_idx);
	if (pll->master_cfg)
		kfree(pll->master_cfg);
	if (pll->slave_cfg)
		kfree(pll->slave_cfg);
	kfree(pll);
err:
	return clk;
}
EXPORT_SYMBOL(clk_register_avipll);
