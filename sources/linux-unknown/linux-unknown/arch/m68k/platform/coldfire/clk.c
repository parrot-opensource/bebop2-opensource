/***************************************************************************/

/*
 *	clk.c -- general ColdFire CPU kernel clk handling
 *
 *	Copyright (C) 2009, Greg Ungerer (gerg@snapgear.com)
 */

/***************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <asm/coldfire.h>

/***************************************************************************/

struct clk *clk_get(struct device *dev, const char *id)
{
	return NULL;
}
EXPORT_SYMBOL(clk_get);

int clk_enable(struct clk *clk)
{
	return 0;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_disable);

void clk_put(struct clk *clk)
{
}
EXPORT_SYMBOL(clk_put);

unsigned long clk_get_rate(struct clk *clk)
{
	return MCF_CLK;
}
EXPORT_SYMBOL(clk_get_rate);
/***************************************************************************/

void __clk_init_enabled(struct clk *clk)
{
	clk->enabled = 1;
	clk->clk_ops->enable(clk);
}

void __clk_init_disabled(struct clk *clk)
{
	clk->enabled = 0;
	clk->clk_ops->disable(clk);
}

static void __clk_enable0(struct clk *clk)
{
	__raw_writeb(clk->slot, MCFPM_PPMCR0);
}

static void __clk_disable0(struct clk *clk)
{
	__raw_writeb(clk->slot, MCFPM_PPMSR0);
}

struct clk_ops clk_ops0 = {
	.enable		= __clk_enable0,
	.disable	= __clk_disable0,
};

#ifdef MCFPM_PPMCR1
static void __clk_enable1(struct clk *clk)
{
	__raw_writeb(clk->slot, MCFPM_PPMCR1);
}

static void __clk_disable1(struct clk *clk)
{
	__raw_writeb(clk->slot, MCFPM_PPMSR1);
}

struct clk_ops clk_ops1 = {
	.enable		= __clk_enable1,
	.disable	= __clk_disable1,
};
#endif /* MCFPM_PPMCR1 */
#endif /* MCFPM_PPMCR0 */
