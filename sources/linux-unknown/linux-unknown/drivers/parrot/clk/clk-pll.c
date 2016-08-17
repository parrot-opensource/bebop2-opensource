
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/spinlock.h>
#include <linux/clk-provider.h>
#include "p7clk-provider.h"
#include "clock.h"


#define to_p7clk_pll(_hw) container_of(_hw, struct clk_p7_pll, hw)

/**
 * p7_prepare_pll() - Prepare PLL
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 */
static int p7_prepare_pll(struct clk_hw* clk_hw)
{
	struct clk_p7_pll const* const  pll = to_p7clk_pll(clk_hw);
	unsigned long const         msk = BIT(pll->lock_bit_idx);
	unsigned long const         addr = pll->pll_conf_reg;
	union p7_pll_reg            curr = { .word = readl(addr) };
	int err = 0;

	pr_debug("p7_prepare_pll %p %x\n", clk_hw, curr.word);
	if (curr.fields.reset ||                  /* powered on ? */
	    ! (readl(pll->lock_reg) & msk)) {     /* locked ? */
		curr.fields.reset = 0;
		p7_write_pll_conf(addr, curr.word);
		if (curr.fields.bypass)
			return err;

		err = p7_activate_pll(pll->lock_reg, msk);
		if (err) {
			/*
			 * The PLL is supposed to be locked when they are not
			 * reset and not bypassed, but we may have to update
			 * the config bits to restart the lock sequence. Toggle
			 * the last bit of config field to do so.
			 */
			curr.fields.cfg ^= 1;
			p7_write_pll_conf(addr, curr.word);
			curr.fields.cfg ^= 1;
			p7_write_pll_conf(addr, curr.word);
			err = p7_activate_pll(pll->lock_reg, msk);
		}

	}

	return err;
}

/**
 * p7_unprepare_pll() - Unprepare PLL
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 */
static void p7_unprepare_pll(struct clk_hw* clk_hw)
{
	struct clk_p7_pll const* const  pll = to_p7clk_pll(clk_hw);
	unsigned long const         addr = pll->pll_conf_reg;
	union p7_pll_reg            curr = { .word = readl(addr) };

	pr_debug("p7_unprepare_pll %p\n", clk_hw);
	/* power off */
	curr.fields.reset = 1;
	writel(curr.word, addr);
}

/**
 * p7_enable_pll() - Enable PLL
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 *
 * If the PLL is not gateable, no-op.
 */
static int p7_enable_pll(struct clk_hw *clk_hw)
{
	struct clk_p7_pll const* const pll = to_p7clk_pll(clk_hw);
	unsigned long const addr = pll->pll_conf_reg;
	union p7_pll_reg pll_conf;
	unsigned long flags = 0;

	pr_debug("p7_enable_pll %p\n", clk_hw);
	if (! pll->gateable)
		goto ret;

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);
	pll_conf.word = readl(addr);
	if (pll_conf.fields.enable)
		goto unlock;
	pll_conf.fields.enable = 1;
	writel(pll_conf.word, addr);

unlock:
	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
ret:
	return 0;
}

/**
 * p7_disable_pll() - Disable PLL
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 */
static void p7_disable_pll(struct clk_hw *clk_hw)
{
	struct clk_p7_pll const* const pll = to_p7clk_pll(clk_hw);
	unsigned long const addr = pll->pll_conf_reg;
	union p7_pll_reg pll_conf;
	unsigned long flags = 0;

	pr_debug("p7_disable_pll %p\n", clk_hw);
	if (! pll->gateable)
		return;

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);
	pll_conf.word = readl(addr);
	if (! pll_conf.fields.enable)
		goto ret;
	pll_conf.fields.enable = 0;
	writel(pll_conf.word, addr);
ret:
	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
}

/**
 * p7_get_pll_rate() - Return master / one level pll rate in HZ.
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 * @parent_rate: rate of the parent clock
 */
static unsigned long p7_get_pll_rate(struct clk_hw* clk_hw,
                                     unsigned long parent_rate)
{
	struct clk_p7_pll* const    pll = to_p7clk_pll(clk_hw);
	unsigned long           rate;
	union p7_pll_reg curr;

	curr.word = readl(pll->pll_conf_reg);
	rate = p7_pll_rate(curr, parent_rate);

	pr_debug("p7_get_pll_rate %p rate=%lu (word=%x)\n", clk_hw, rate, curr.word);
	return rate;
}

/**
 * p7_set_pll_rate() - Set master / one level pll frequency in HZ.
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 * @rate: rate to configure in HZ.
 * @parent_rate: rate of the parent clock
 *
 * Warning: frequency passed in argument will likely be rounded to a possible
 * PLL operation point.
 */
static int p7_set_pll_rate(struct clk_hw* clk_hw, unsigned long rate,
                           unsigned long parent_rate)
{
	struct clk_p7_pll       *pll = to_p7clk_pll(clk_hw);
	unsigned long const     addr = pll->pll_conf_reg;
	union p7_pll_reg        curr, new;
	int                     idx;
	unsigned long           flags = 0;
	int                     err = 0;

	pr_debug("p7_set_pll_rate %p rate=%lu parent=%lu\n",
			clk_hw, rate, parent_rate);
	idx = p7_get_clkcfg(pll->cfg_idx, rate, pll->cfg_nr);
	if (idx < 0)
		return idx;
	new.word = pll->pll_cfg[idx];

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);
	curr.word = readl(addr);
	if (curr.fields.cfg == new.fields.cfg &&
	    curr.fields.bypass == new.fields.bypass)
		goto unlock;

	curr.fields.cfg = new.fields.cfg;
	curr.fields.bypass = new.fields.bypass;
	p7_write_pll_conf(addr, curr.word);
	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
	/*
	 * If the PLL is reset, the lock bit will never be set to 1. So
	 * we avoid locking if this operation doesn't make sense.
	 */
	if ((!curr.fields.reset) && (!curr.fields.bypass))
		err = p7_activate_pll(pll->lock_reg, BIT(pll->lock_bit_idx));
	return err;

unlock:
	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
	return err;
}

/**
 * p7_set_pll_rate() - Set master / one level pll frequency in HZ.
 * @clk_hw: a @clk_hw in a @clk_p7_pll
 * @rate: rate to configure in HZ.
 * @parent_rate: pointer to the rate of the parent clock
 */
static long p7_round_pll_rate(struct clk_hw* clk_hw, unsigned long rate,
                              unsigned long* parent_rate)
{
	struct clk_p7_pll* const    pll = to_p7clk_pll(clk_hw);
	int const               idx = p7_get_clkcfg(pll->cfg_idx,
	                                            rate,
	                                            pll->cfg_nr);
	pr_debug("p7_round_pll_rate %p rate=%lu parent=%lu : idx=%d\n",
			clk_hw, rate, *parent_rate, idx);
	if (idx < 0)
		return 0;

	return (long) pll->cfg_idx[idx];
}

static void p7_pll_init(struct clk_hw *clk_hw)
{
	struct clk_p7_pll* const pll = to_p7clk_pll(clk_hw);
	unsigned long const     addr = pll->pll_conf_reg;
	union p7_pll_reg        curr;
	unsigned long flags = 0;


	if (!pll->gateable)
		return;

	if (pll->lock)
		spin_lock_irqsave(pll->lock, flags);
	curr.word = readl(addr);
	if (curr.fields.enable) {
		curr.fields.enable = 0;
		writel(curr.word, addr);
	}

	if (pll->lock)
		spin_unlock_irqrestore(pll->lock, flags);
}


struct clk_ops const p7_clk_pll_ops = {
	.prepare        = &p7_prepare_pll,
	.unprepare      = &p7_unprepare_pll,
	.enable         = &p7_enable_pll,
	.disable        = &p7_disable_pll,
	.recalc_rate    = &p7_get_pll_rate,
	.set_rate       = &p7_set_pll_rate,
	.round_rate     = &p7_round_pll_rate,
	.init           = &p7_pll_init,
};

struct clk_ops const p7_clk_fixed_pll_ops = {
	.prepare        = &p7_prepare_pll,
	.unprepare      = &p7_unprepare_pll,
	.enable         = &p7_enable_pll,
	.disable        = &p7_disable_pll,
	.recalc_rate    = &p7_get_pll_rate,
	.init           = &p7_pll_init,
};

struct clk *clk_register_pll(struct device *dev, const char *name,
                             const char *parent_name, const bool gateable, const unsigned long flags,
                             const unsigned long lock_reg, const u8 lock_bit_idx,
                             const unsigned long pll_reg, unsigned long const * const cfg_idx,
                             unsigned long const * const pll_cfg, const size_t cfg_nr,
                             spinlock_t *lock)
{
	struct clk_p7_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(struct clk_p7_pll), GFP_KERNEL);
	if (!pll) {
		pr_err("%s: could not allocate clk pll\n", __func__);
		clk = ERR_PTR(-ENOMEM);
		goto err;
	}

	init.name = name;
	if (cfg_nr)
		init.ops = &p7_clk_pll_ops;
	else
		init.ops = &p7_clk_fixed_pll_ops;
	init.flags = flags;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->gateable = gateable;
	pll->lock_reg = lock_reg;
	pll->lock_bit_idx = lock_bit_idx;
	pll->pll_conf_reg = pll_reg;
	pll->cfg_idx = cfg_nr ? kmemdup(cfg_idx, cfg_nr * sizeof(unsigned long), GFP_KERNEL) : NULL;
	pll->pll_cfg = cfg_nr ? kmemdup(pll_cfg, cfg_nr * sizeof(unsigned long), GFP_KERNEL) : NULL;
	pll->cfg_nr = cfg_nr;
	pll->lock = lock;
	pll->hw.init = &init;

	if (cfg_nr && ((!pll->cfg_idx) || (!pll->pll_cfg))) {
		pr_err("%s: could not allocate memory for pll info\n", __func__);
		clk = ERR_PTR(-ENOMEM);
		goto free_pll_content;
	}

	clk = clk_register(dev, &pll->hw);

	pr_debug("clk_register_pll %p for %s\n", &pll->hw, name);
	if (! IS_ERR(clk))
		return clk;

free_pll_content:
	if (pll->cfg_idx)
		kfree(pll->cfg_idx);
	if (pll->pll_cfg)
		kfree(pll->pll_cfg);
	kfree(pll);
err:
	return clk;
}
EXPORT_SYMBOL(clk_register_pll);
