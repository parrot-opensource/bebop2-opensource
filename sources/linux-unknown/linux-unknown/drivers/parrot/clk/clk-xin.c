
#include <linux/export.h>
#include <linux/slab.h>
#include "p7clk-provider.h"

/*
 * SDIO GBC based XIN clock.
 *
 * Depicts a leaf clock in the overall clock hierarchy. It behaves as a
 * a gated clock divider, providing SDIO controller with an
 * ASYMETRIC input clock used to derive the external SDIO bus clock.
 * The parent clock MUST support enable/disable counting.
 *
 * Warning: the asymetric nature of SDIO XIN clock enforces it to be further divided
 * (using the SDIO controller divisor) to generate the needed symetric bus clock.
 */

#define to_p7clk_xin(_hw) container_of(_hw, struct clk_p7_xin, hw)

/*
 * Enable GBC based SDIO XIN clock.
 */
static int p7_enable_xinclk(struct clk_hw* clk_hw)
{
	struct clk_p7_xin const* const xin = to_p7clk_xin(clk_hw);

	/* Enable clock */
	__raw_writel(readl(xin->clk_reg) | SDIOx_XIN_CLK_EN, xin->clk_reg);
	return 0;
}

/*
 * Disable GBC based SDIO XIN clock.
 */
static void p7_disable_xinclk(struct clk_hw* clk_hw)
{
	struct clk_p7_xin const* const xin = to_p7clk_xin(clk_hw);

	/* Disable clock */
	__raw_writel(readl(xin->clk_reg) & ~SDIOx_XIN_CLK_EN, xin->clk_reg);
}

/*
 * Return current SDIO XIN clock frequency in HZ.
 */
static unsigned long p7_recalc_xinclk_rate(struct clk_hw* clk_hw,
                                           unsigned long parent_rate)
{
	struct clk_p7_xin* const    xin = to_p7clk_xin(clk_hw);
	unsigned int const div = (readl(xin->clk_reg) >> SDIOx_XIN_CLK_DIV_SFT) &
							 SDIOx_XIN_CLK_DIV_MSK;
	return (parent_rate + div) / (div + 1);
}

/*
 * Return best rounded XIN clock frequency in Hz.
 *
 * Round frequency passed in argument so that once divided using SD host divisor
 * the produced SD clock complies with following constraints:
 *  - clock is symetric (XIN clock is not) ;
 *  - SD clock output (i.e., once devided by host controller) is as close as
 *    possible to desired rate.
 * Host controller divisor values are limited to powers of 2 to comply with legacy
 * SD specifications.
 */
static long p7_round_xinclk_rate(struct clk_hw* clk_hw, unsigned long rate,
                                 unsigned long *parent_rate)
{
	unsigned long const prate = *parent_rate;

	if (rate && !(prate % rate))
		return rate;
	else {
		int i;
		for (i = 0; i <= 10; i++) {
			unsigned long nrate = prate / (i + 1);
			if ((rate > nrate) && (nrate <= SDIOx_XIN_CLK_RATE_MAX))
				return nrate;
		}

		return prate / 11;
	}
}

/*
 * Set SDIO XIN cloxk frequency in HZ.
 *
 * Warning: frequency passed in argument will be rounded using the SDIO
 * controller and GBC divisors combination closest to targeted
 * frequency. Be aware that SDIO XIN clock is asymetric (see @p7_sin_clk).
 */
static int p7_set_xinclk_rate(struct clk_hw* clk_hw, unsigned long rate,
                              unsigned long parent_rate)
{
	struct clk_p7_xin* const xin = to_p7clk_xin(clk_hw);
	u32 const xdiv = parent_rate / rate - 1;
	int const enabled = __raw_readl(xin->clk_reg) & SDIOx_XIN_CLK_EN;

	/* Clock must be disabled before modifying divisor. */
	if (enabled)
		/* Disable clock. */
		__raw_writel(0, xin->clk_reg);

	/* Setup divisor. */
	__raw_writel(xdiv << SDIOx_XIN_CLK_DIV_SFT, xin->clk_reg);

	/*
	 * Re-enabling the clock has to be implemented using a separate write
	 * instruction.
	 */
	if (enabled)
		/* Re-enable clock if needed. */
		__raw_writel((xdiv << SDIOx_XIN_CLK_DIV_SFT) | SDIOx_XIN_CLK_EN,
		             xin->clk_reg);

	return 0;
}

/*
 * XIN clock operations
 */
struct clk_ops const p7_clk_xin_ops = {
	.enable         = &p7_enable_xinclk,
	.disable        = &p7_disable_xinclk,
	.recalc_rate    = &p7_recalc_xinclk_rate,
	.set_rate       = &p7_set_xinclk_rate,
	.round_rate     = &p7_round_xinclk_rate
};

struct clk *clk_register_xin(struct device *dev, const char *name,
                             const char *parent_name, const unsigned long clk_reg)
{
	struct clk_p7_xin *xin;
	struct clk *clk;
	struct clk_init_data init;

	xin = kzalloc(sizeof(struct clk_p7_xin), GFP_KERNEL);
	if (! xin) {
		pr_err("%s: could not allocate clk xin\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &p7_clk_xin_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	xin->clk_reg = clk_reg;
	xin->hw.init = &init;

	clk = clk_register(dev, &xin->hw);
	if (IS_ERR(clk))
		kfree(xin);

	return clk;
}
EXPORT_SYMBOL(clk_register_xin);
