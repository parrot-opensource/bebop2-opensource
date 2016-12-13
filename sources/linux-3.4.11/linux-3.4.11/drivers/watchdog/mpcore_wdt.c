/*
 *	Watchdog driver for the mpcore watchdog timer
 *
 *	(c) Copyright 2004 ARM Limited
 *	(c) Copyright 2014 Parrot S.A.
 *
 *	Based on the SoftDog driver:
 *	(c) Copyright 1996 Alan Cox <alan@lxorguk.ukuu.org.uk>,
 *						All Rights Reserved.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Neither Alan Cox nor CymruNet Ltd. admit liability nor provide
 *	warranty for any of this software. This material is provided
 *	"AS-IS" and at no charge.
 *
 *	(c) Copyright 1995    Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#include <asm/smp_twd.h>

#define TIMER_MARGIN 60
#define TIMER_MAX_MARGIN 65536
static unsigned int mpcore_margin = TIMER_MARGIN;
module_param(mpcore_margin, uint, 0);
MODULE_PARM_DESC(mpcore_margin,
	"Watchdog mpcore_margin in seconds. (0 < mpcore_margin < 65536, default="
					__MODULE_STRING(TIMER_MARGIN) ")");

static bool mpcore_nowayout = WATCHDOG_NOWAYOUT;
module_param(mpcore_nowayout, bool, 0);
MODULE_PARM_DESC(mpcore_nowayout,
		"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int mpcore_noboot = 0;
module_param(mpcore_noboot, int, 0);
MODULE_PARM_DESC(mpcore_noboot,
	"MPcore watchdog action, set to 1 to ignore reboots, 0 to reboot (default=0)");

static struct platform_device *mpcore_wdt_pdev;

struct mpcore_wdt_drvdata {
	struct watchdog_device wdt;
	void __iomem *base;
	unsigned int perturb;
	struct clk *clk;
	unsigned long clk_rate;
	unsigned long timer_alive;
	spinlock_t lock;
};

struct mpcore_wdt_write_smp_data {
	unsigned long value;
	void __iomem *addr;
};

static void __mpcore_wdt_writel_smp(void *data)
{
	struct mpcore_wdt_write_smp_data *w = data;
	writel(w->value, w->addr);
}

static inline void _mpcore_wdt_writel_smp(int cpu, unsigned long value, void __iomem *addr)
{
	struct mpcore_wdt_write_smp_data w = {
		.value = value,
		.addr = addr
	};

	if (cpu == -1)
		on_each_cpu(__mpcore_wdt_writel_smp, &w, 1);
	else
		smp_call_function_single(cpu, __mpcore_wdt_writel_smp, &w, 1);
}

static inline void _mpcore_wdt_load(int cpu, struct mpcore_wdt_drvdata *drvdata)
{
	unsigned int count;

	/* Assume prescale is set to 256 */
	count = (drvdata->clk_rate / 256) * (drvdata->wdt.timeout);

	/* Reload the counter */
	_mpcore_wdt_writel_smp(cpu, count + drvdata->perturb, drvdata->base + TWD_WDOG_LOAD);
	drvdata->perturb = drvdata->perturb ? 0 : 1;
}

static inline void _mpcore_wdt_enable(int cpu, struct mpcore_wdt_drvdata *drvdata)
{
	if (mpcore_noboot)
	{
		/* Enable watchdog - prescale=256, watchdog mode=0, enable=1 */
		_mpcore_wdt_writel_smp(cpu, 0x0000FF01, drvdata->base + TWD_WDOG_CONTROL);
	}
	else
	{
		/* Enable watchdog - prescale=256, watchdog mode=1, enable=1 */
		_mpcore_wdt_writel_smp(cpu, 0x0000FF09, drvdata->base + TWD_WDOG_CONTROL);
	}
}

static inline void _mpcore_wdt_disable(int cpu, struct mpcore_wdt_drvdata *drvdata)
{
	_mpcore_wdt_writel_smp(cpu, 0x12345678, drvdata->base + TWD_WDOG_DISABLE);
	_mpcore_wdt_writel_smp(cpu, 0x87654321, drvdata->base + TWD_WDOG_DISABLE);
	_mpcore_wdt_writel_smp(cpu, 0x0, drvdata->base + TWD_WDOG_CONTROL);
}

static int mpcore_wdt_ping(struct watchdog_device *w)
{
	struct mpcore_wdt_drvdata *drvdata = watchdog_get_drvdata(w);

	spin_lock(&drvdata->lock);
	_mpcore_wdt_load(-1, drvdata);
	spin_unlock(&drvdata->lock);

	return 0;
}

static int mpcore_wdt_start(struct watchdog_device *w)
{
	struct mpcore_wdt_drvdata *drvdata = watchdog_get_drvdata(w);

	spin_lock(&drvdata->lock);
	set_bit(0, &drvdata->timer_alive);
	_mpcore_wdt_load(-1, drvdata);
	_mpcore_wdt_enable(-1, drvdata);
	spin_unlock(&drvdata->lock);

	return 0;
}

static int mpcore_wdt_stop(struct watchdog_device *w)
{
	struct mpcore_wdt_drvdata *drvdata = watchdog_get_drvdata(w);

	spin_lock(&drvdata->lock);
	clear_bit(0, &drvdata->timer_alive);
	_mpcore_wdt_disable(-1, drvdata);
	spin_unlock(&drvdata->lock);

	return 0;
}

static int mpcore_wdt_set_timeout(struct watchdog_device *w, unsigned int t)
{
	struct mpcore_wdt_drvdata *drvdata = watchdog_get_drvdata(w);

	spin_lock(&drvdata->lock);
	w->timeout = t;
	spin_unlock(&drvdata->lock);

	return 0;
}

static struct watchdog_info mpcore_wdt_info = {
	.identity = "MPCore Watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static struct watchdog_ops mpcore_wdt_ops = {
	.owner = THIS_MODULE,
	.start = mpcore_wdt_start,
	.stop = mpcore_wdt_stop,
	.ping = mpcore_wdt_ping,
	.set_timeout = mpcore_wdt_set_timeout,
};

static int mpcore_wdt_cpu_callback(struct notifier_block *nfb,
		unsigned long action, void *hcpu)
{
	struct mpcore_wdt_drvdata *drvdata = platform_get_drvdata(mpcore_wdt_pdev);
	unsigned int cpu = (unsigned long)hcpu;

	switch (action) {
	case CPU_ONLINE:
		spin_lock(&drvdata->lock);
		if (test_bit(0, &drvdata->timer_alive))
		{
			_mpcore_wdt_load(cpu, drvdata);
			_mpcore_wdt_enable(cpu, drvdata);
		}
		spin_unlock(&drvdata->lock);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block mpcore_wdt_cpu_notifier =
{
	.notifier_call = mpcore_wdt_cpu_callback,
};

#ifdef CONFIG_CPU_FREQ
static int mpcore_wdt_cpufreq_transition(struct notifier_block *nb,
	unsigned long state, void *data)
{
	struct cpufreq_freqs *freqs = data;
	struct mpcore_wdt_drvdata *drvdata = platform_get_drvdata(mpcore_wdt_pdev);
	unsigned long rate;

	/* Retrieve new clock freqyency */
	rate = clk_get_rate(drvdata->clk);

	spin_lock(&drvdata->lock);

	drvdata->clk_rate = rate;

	/* Reload cpu watchdog if frequency has changed */
	if (state == CPUFREQ_POSTCHANGE || state == CPUFREQ_RESUMECHANGE)
		if (test_bit(0, &drvdata->timer_alive))
			_mpcore_wdt_load(freqs->cpu, drvdata);

	spin_unlock(&drvdata->lock);

	return NOTIFY_OK;
}

static struct notifier_block mpcore_wdt_cpufreq_nb = {
	.notifier_call = mpcore_wdt_cpufreq_transition,
};
#endif

static int __devinit mpcore_wdt_probe(struct platform_device *pdev)
{
	struct mpcore_wdt_drvdata *drvdata;
	struct watchdog_device *mpcore_wdt;
	struct resource *res;
	int ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct mpcore_wdt_drvdata),
			       GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "Unable to allocate watchdog device\n");
		return -ENOMEM;
	}

	if (mpcore_margin < 1 || mpcore_margin > TIMER_MAX_MARGIN)
		mpcore_margin = TIMER_MARGIN;

	mpcore_wdt = &drvdata->wdt;
	mpcore_wdt->info = &mpcore_wdt_info;
	mpcore_wdt->ops = &mpcore_wdt_ops;
	mpcore_wdt->timeout = mpcore_margin;
	mpcore_wdt->min_timeout = 1;
	mpcore_wdt->max_timeout = TIMER_MAX_MARGIN;
	watchdog_set_nowayout(mpcore_wdt, mpcore_nowayout);
	watchdog_set_drvdata(mpcore_wdt, drvdata);

	spin_lock_init(&drvdata->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	drvdata->base = devm_request_and_ioremap(&pdev->dev, res);

	if (drvdata->base == NULL) {
		dev_err(&pdev->dev, "Could not get memory\n");
		return -EBUSY;
	}

	drvdata->clk = clk_get_sys("smp_twd", NULL);
	if (IS_ERR(drvdata->clk)) {
		dev_err(&pdev->dev, "Could not get watchdog clock\n");
		return -EINVAL;
	}

	drvdata->clk_rate = clk_get_rate(drvdata->clk);

	ret = watchdog_register_device(&drvdata->wdt);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register watchdog device\n");
		goto err_release_clk;
	}

	register_hotcpu_notifier(&mpcore_wdt_cpu_notifier);
#ifdef CONFIG_CPU_FREQ
	cpufreq_register_notifier(&mpcore_wdt_cpufreq_nb, CPUFREQ_TRANSITION_NOTIFIER);
#endif

	platform_set_drvdata(pdev, drvdata);

	mpcore_wdt_pdev = pdev;

	mpcore_wdt_stop(&drvdata->wdt);

	return 0;

err_release_clk:
	clk_put(drvdata->clk);

	return ret;
}

static int __devexit mpcore_wdt_remove(struct platform_device *pdev)
{
	struct mpcore_wdt_drvdata *drvdata = platform_get_drvdata(pdev);

	mpcore_wdt_stop(&drvdata->wdt);
	watchdog_unregister_device(&drvdata->wdt);
	clk_put(drvdata->clk);

	return 0;
}

#ifdef CONFIG_PM
int mpcore_wdt_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mpcore_wdt_drvdata *drvdata = platform_get_drvdata(pdev);

	if (test_bit(0, &drvdata->timer_alive))
		mpcore_wdt_ping(&drvdata->wdt);
	return 0;
}

int mpcore_wdt_resume(struct platform_device *pdev)
{
	struct mpcore_wdt_drvdata *drvdata = platform_get_drvdata(pdev);

	if (test_bit(0, &drvdata->timer_alive))
		mpcore_wdt_start(&drvdata->wdt);

	return 0;
}
#endif

static struct platform_driver mpcore_wdt_driver = {
	.probe = mpcore_wdt_probe,
	.remove = __devexit_p(mpcore_wdt_remove),
#ifdef CONFIG_PM
	.suspend = mpcore_wdt_suspend,
	.resume = mpcore_wdt_resume,
#endif
	.driver = {
		.name = "mpcore_wdt",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(mpcore_wdt_driver);

MODULE_AUTHOR("ARM Limited");
MODULE_AUTHOR("Aurelien Lefebvre <aurelien.lefebvre@parrot.com>");
MODULE_DESCRIPTION("MPcore Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
