
#include <linux/export.h>
#include <linux/slab.h>
#include "p7clk-provider.h"

/*
 * SD clock (ACS3 internal clock derived from XIN clock)
 */

/*
 * Given a GBC (pre-)divisor value, compute XIN clock rate in Hz depending on
 * the parent clock frequency passed in argument.
 */
static unsigned long p7_sdclk_div2rate(unsigned long prate,
				       u32 xin_div)
{
	unsigned long const xrate = (prate + xin_div) / (xin_div + 1);

#ifdef DEBUG
	BUG_ON(xrate > SDIOx_XIN_CLK_RATE_MAX);
#endif
	return xrate;
}

/*
 * Given a parent clock frequency in Hz, compute the GBC (pre-)divisor value
 * to reach the XIN clock rate passed in argument.
 */
static u32 p7_sdclk_rate2div(unsigned long prate, unsigned long rate,
			     unsigned long *newrate)
{
	unsigned long   nrate = 0;
	unsigned long   sdiv;               /* SDIO controller divisor. */
	u32             xdiv = 10, xdiv_;   /* SDIO XIN clock GBC (pre-)divisor. */

	/* 
	 * We test the different values of the SDIO controller divisor.
	 * With sdiv = 1, we cannot control the duty cycle 
	 */
	for (sdiv = 2; sdiv <= 256; sdiv = sdiv * 2) {
		/* If we cannot a higher rate with this new divisor,
		 * we stop the loop. */
		if ((prate / sdiv) < nrate) break;
 
		/* We test the different values of the SDIO XIN clock GBC 
		 * (pre-)divisor. */
		for (xdiv_ = 0; xdiv_ <= 10; xdiv_ ++) {
			unsigned long xin_rate = prate / (xdiv_ + 1);
			unsigned long sd_rate = xin_rate / sdiv;

			/* SDIO XIN clock can't be higher than 260MHz */
			if (xin_rate > SDIOx_XIN_CLK_RATE_MAX) continue;

			/* While the rate calculated is too high, we try a new
			 * of sdiv. */
			if (sd_rate <= rate) {
				if (sd_rate >= nrate) {
					nrate = sd_rate;
					xdiv = xdiv_;
				}
				break;
			}
		}
	}
	*newrate = nrate;
	return xdiv;
}

/*
 * Determine SD clock rate and XIN clock rate.
 */
static long p7_determine_sdclk_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *best_parent_rate,
				    struct clk **best_parent_clk)
{
	u32 xin_div;
	long nrate;
	unsigned long fastrate = clk_get_rate(clk_get_parent(*best_parent_clk));

	xin_div = p7_sdclk_rate2div(fastrate, rate, &nrate);
	*best_parent_rate = p7_sdclk_div2rate(fastrate, xin_div);

	return nrate;
}

/*
 * SD clock operations.
 */
struct clk_ops const p7_clk_sd_ops = {
	.determine_rate = p7_determine_sdclk_rate,
};

struct clk *clk_register_sd(struct device *dev, const char *name,
			    const char *parent_name)
{
	struct clk_hw *hw;
	struct clk *clk;
	struct clk_init_data init;

	hw = kzalloc(sizeof(struct clk_hw), GFP_KERNEL);
	if (! hw) {
		pr_err("%s: could not allocate sdclk hw\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &p7_clk_sd_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	hw->init = &init;

	clk = clk_register(dev, hw);
	if (IS_ERR(clk))
		kfree(hw);

	return clk;
}
EXPORT_SYMBOL(clk_register_sd);
