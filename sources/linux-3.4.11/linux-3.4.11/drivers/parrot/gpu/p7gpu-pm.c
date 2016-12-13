/*
 * @file p7gpu-pm.c
 * @brief Driver for handling Parrot's Mali200/400 GPU power management
 *
 * Copyright (C) 2011 Parrot S.A.
 *
 * @author     alvaro.moran@parrot.com
 * @date       2011-01-25
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
 *
 *
 *
 * This driver covers the power management for the possible GPUs handled in 
 * Parrot7, Mali200 and Mali400. The clocks are different, so we have to check 
 * out the revision.
 *
 *
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <asm/io.h>
#include <mach/clkdev.h>
#include <linux/clk.h>
#include <linux/module.h> 
#include "p7gpu-pm.h"

MODULE_AUTHOR( "Alvaro Moran Parrot S.A. <alvaro.moran@parrot.com>" );
MODULE_DESCRIPTION( "Parrot7 Mali200/400 power management" );
MODULE_LICENSE( "GPL" );

static int inline p7gpu_pm_activate_clk (struct platform_device *pdev, const char* name)
{
	int ret = 0;
	struct clk* clock;

	clock = clk_get(&pdev->dev, name);
	if (IS_ERR(clock))
	{
		printk(KERN_ERR "Parrot7 GPU error getting %s clock\n",name);
		return PTR_ERR(clock);
	}
	ret = clk_prepare_enable(clock);
	if (ret)
	{
		printk(KERN_ERR "Parrot7 GPU error enabling %s clock\n",name);
	}
	return ret;
}

static void inline p7gpu_pm_disable_clk (struct platform_device *pdev, const char* name)
{
	struct clk* clock;

	clock = clk_get(&pdev->dev, name);
	// if there is an error getting the clock, return silently
	if (IS_ERR(clock))
    		return;
	clk_disable_unprepare(clock);
	/* put ref from clk_get from disable */
	clk_put(clock);
	/* put ref from clk_get from activate */
	clk_put(clock);
	return;
}

static int p7gpu_pm_clocks_on(struct platform_device *pdev)
{
	int ret = 0;
	int i;
	struct p7gpu_pm_plat_data const* const pdata = 
		(struct p7gpu_pm_plat_data const*) dev_get_platdata(&pdev->dev);
	
	for (i = 0; i < pdata->clocks_number; i++) {
		ret = p7gpu_pm_activate_clk(pdev, pdata->clocks_names[i]);
		if (ret != 0)
			goto fail_probe;
	}

	printk(KERN_INFO "Parrot7 GPU clocks started\n");
	return 0;

fail_probe:
	return ret;
}

static int p7gpu_pm_clocks_off(struct platform_device *pdev)
{
	int i;
	struct p7gpu_pm_plat_data const* const pdata = 
		(struct p7gpu_pm_plat_data const*) dev_get_platdata(&pdev->dev);

	printk(KERN_INFO "Parrot7 GPU clocks stopping\n");
	/* Disable clocks in inversed order */
	for (i = pdata->clocks_number - 1; i >= 0; i--)	
		p7gpu_pm_disable_clk(pdev, pdata->clocks_names[i]);

	return 0;
}

static int p7gpu_pm_probe(struct platform_device *pdev)
{
	return p7gpu_pm_clocks_on(pdev);
}

static int p7gpu_pm_remove(struct platform_device *pdev)
{
	return p7gpu_pm_clocks_off(pdev);
}


#ifdef CONFIG_PM
static int p7gpu_pm_suspend(struct platform_device* pdev, pm_message_t state)
{
	return p7gpu_pm_clocks_off(pdev);
}

static int p7gpu_pm_resume(struct platform_device* pdev)
{
	return p7gpu_pm_clocks_on(pdev);
}
#else

#define p7gpu_pm_suspend    NULL
#define p7gpu_pm_resume     NULL

#endif  /* CONFIG_PM */

static struct platform_driver p7gpu_pm_driver = {
	.probe		= p7gpu_pm_probe,
	.remove		= p7gpu_pm_remove,
	.suspend	= p7gpu_pm_suspend,
	.resume		= p7gpu_pm_resume,
	.driver		= {
		.name	= "p7gpu",
	},
};

/// Module initialization function
static int __init p7gpu_pm_init( void ) 
{ 
	int ret;
	ret = platform_driver_register(&p7gpu_pm_driver);
	return ret;
}

/// Module Cleanup Function
static void __exit p7gpu_pm_exit( void ) 
{ 
	platform_driver_unregister(&p7gpu_pm_driver);
}

module_init( p7gpu_pm_init ); 
module_exit( p7gpu_pm_exit ); 
