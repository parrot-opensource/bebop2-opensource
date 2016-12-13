/**
 * linux/drivers/parrot/rtc/rtc-p7mu.c - Parrot7 power management unit RTC
 *                                       block implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    16-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <mfd/p7mu.h>

#define P7MU_RTC_NAME "p7mu-rtc"

/*
 * RTC register offset from page address
 */
#define P7MU_RTC_TIME_H		((u16) 0)
#define P7MU_RTC_TIME_L		((u16) 1)
#define P7MU_RTC_MATCH_H	((u16) 2)
#define P7MU_RTC_MATCH_L	((u16) 3)
#define P7MU_RTC_LOAD_H		((u16) 4)
#define P7MU_RTC_LOAD_L		((u16) 5)
#define P7MU_RTC_CTRL		((u16) 6)

#define P7MU_RTC_TIME       P7MU_RTC_TIME_H
#define P7MU_RTC_MATCH      P7MU_RTC_MATCH_H
#define P7MU_RTC_LOAD       P7MU_RTC_LOAD_H

#define P7MU_RTC_EN         ((u16) (1U << 0))
#define P7MU_RTC_LOAD_EN    ((u16) (1U << 1))
#define P7MU_RTC_ALRM_EN    ((u16) (1U << 2))

static struct resource const*   p7mu_rtc_res;
static int                      p7mu_rtc_virq;
static struct rtc_device*       p7mu_rtc_dev;

static int p7mu_read_rtc_time(struct device* dev, struct rtc_time* time)
{
	u32         sec;
	/* there is no race condition here :
	   32bit value from "rtc_cnt" counter is copied into two 16bit
	   "APB registers" RTC_TIME - 32bit
	   RTC_TIME_H and RTC_TIME_L when read access to RTC_TIME_H occurred.
	 */
	int const   err = p7mu_read32(p7mu_rtc_res->start + P7MU_RTC_TIME, &sec);

	if (err)
		return err;

	rtc_time_to_tm(sec, time);
	return 0;
}

static int p7mu_set_rtc_time(struct device* dev, struct rtc_time* time)
{
	unsigned long   now;
	int             err;

	rtc_tm_to_time(time, &now);

	err = p7mu_write32(p7mu_rtc_res->start + P7MU_RTC_LOAD, now);
	if (err)
		return err;

	return p7mu_mod16(p7mu_rtc_res->start + P7MU_RTC_CTRL,
	                  P7MU_RTC_LOAD_EN | P7MU_RTC_EN,
	                  P7MU_RTC_LOAD_EN | P7MU_RTC_EN);
}

static int p7mu_read_rtc_alrm(struct device* dev, struct rtc_wkalrm* alarm)
{
	unsigned long   sec;
	int             err;
	u16 reg;

	dev_dbg(dev, "getting RTC alarm state...\n");

	err = p7mu_read32(p7mu_rtc_res->start + P7MU_RTC_MATCH,
			(u32*) &sec);
	if (err)
		goto err;

	err = p7mu_read16(p7mu_rtc_res->start + P7MU_RTC_CTRL,
			&reg);
	if (err < 0)
		goto err;

	alarm->enabled = !!(reg & P7MU_RTC_ALRM_EN);


	rtc_time_to_tm(sec, &alarm->time);

	return 0;

err:
	return err;
}

static int __p7mu_enable_rtc_alrm(int enabled)
{
	int mask = P7MU_RTC_EN;

	if (enabled)
		mask |= P7MU_RTC_ALRM_EN;
	return p7mu_mod16(p7mu_rtc_res->start + P7MU_RTC_CTRL,
	                  mask,
	                  P7MU_RTC_ALRM_EN | P7MU_RTC_EN);
}

static int p7mu_set_rtc_alrm(struct device* dev, struct rtc_wkalrm* alarm)
{
	int             err;
	unsigned long   sec;

	dev_dbg(dev, "setting up RTC alarm...\n");

	rtc_tm_to_time(&alarm->time, &sec);

	disable_irq(p7mu_rtc_virq);

	/* 32bit "internal" register is update by values from RTC_MATCH_H and
	   RTC_MATCH_L "APB registers" when RTC_MATCH_L register is updated
	 */
	err = p7mu_write32(p7mu_rtc_res->start + P7MU_RTC_MATCH, sec);
	if (! err)
		__p7mu_enable_rtc_alrm(alarm->enabled);

	enable_irq(p7mu_rtc_virq);
	return err;
}

static int p7mu_enable_rtc_alrm(struct device* dev, unsigned int enabled)
{
	dev_dbg(dev, "%sabling RTC alarm irq...\n", enabled ? "en" : "dis");
	return __p7mu_enable_rtc_alrm(enabled);
}

static struct rtc_class_ops const p7mu_rtc_ops = {
	.read_time              = &p7mu_read_rtc_time,
	.set_time               = &p7mu_set_rtc_time,
	.read_alarm             = &p7mu_read_rtc_alrm,
	.set_alarm              = &p7mu_set_rtc_alrm,
	.alarm_irq_enable       = &p7mu_enable_rtc_alrm
};

static irqreturn_t p7mu_handle_rtc_irq(int irq, void* dev_id)
{
	/* Alarm interrupts are one-shot. */
	rtc_update_irq(p7mu_rtc_dev, 1, RTC_IRQF | RTC_AF);
	return IRQ_HANDLED;
}

static int __devinit p7mu_probe_rtc(struct platform_device* pdev)
{
	int                             err;
	struct resource *resirq;

	dev_dbg(&pdev->dev, "probing...\n");

	/* Reserve region from P7MU I2C registers space. */
	p7mu_rtc_res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (! p7mu_rtc_res) {
		dev_err(&pdev->dev, "failed to find I/O region address\n");
		err = -ENXIO;
		goto err;
	}

	p7mu_rtc_res = p7mu_request_region(pdev, p7mu_rtc_res);
	if (IS_ERR(p7mu_rtc_res)) {
		err = PTR_ERR(p7mu_rtc_res);
		goto err;
	}
	
	/* Allow RTC to wake up system */
	err = device_init_wakeup(&pdev->dev, true);
	if (err)
		goto region;

	/* Register device to RTC framework. */
	p7mu_rtc_dev = rtc_device_register(P7MU_RTC_NAME,
	                                   &pdev->dev,
	                                   &p7mu_rtc_ops,
	                                   THIS_MODULE);
	if (IS_ERR(p7mu_rtc_dev)) {
		dev_err(&pdev->dev, "failed to register device\n");
		err = PTR_ERR(p7mu_rtc_dev);
		goto wakeup;
	}

	/* Reserve interrupt for alarm usage. */
	resirq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (!resirq) {
		dev_err(&pdev->dev, "failed to find hardware interrupt number\n");
		err = ENODEV;
		goto dev;
	}

	p7mu_rtc_virq = resirq->start;
	err = request_threaded_irq(p7mu_rtc_virq, NULL, &p7mu_handle_rtc_irq, IRQF_TRIGGER_RISING, "p7mu rtc", NULL);
	if (err < 0) {
		printk("p7mu rtc irq %d %d\n", p7mu_rtc_virq, err);
		goto dev;
	}

	/* Enable counting. */
	err = p7mu_mod16(p7mu_rtc_res->start + P7MU_RTC_CTRL,
	                 P7MU_RTC_EN,
	                 P7MU_RTC_EN);
	if (err) {
		dev_err(&pdev->dev, "failed to start counting\n");
		goto irq;
	}

	return 0;

irq:
	free_irq(p7mu_rtc_virq, NULL);
dev:
	rtc_device_unregister(p7mu_rtc_dev);
wakeup:
	device_init_wakeup(&pdev->dev, false);
region:
	p7mu_release_region(p7mu_rtc_res);
err:
	dev_err(&pdev->dev, "failed to probe (%d)\n", err);
	return err;
}

static int __devexit p7mu_remove_rtc(struct platform_device* pdev)
{
	int err;

	dev_dbg(&pdev->dev, "removing...\n");

	err = p7mu_write16(p7mu_rtc_res->start + P7MU_RTC_CTRL, 0);
	free_irq(p7mu_rtc_virq, NULL);
	rtc_device_unregister(p7mu_rtc_dev);
	device_init_wakeup(&pdev->dev, false);
	p7mu_release_region(p7mu_rtc_res);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int p7mu_suspend_rtc(struct device* dev)
{
	return 0;
}

static int p7mu_resume_rtc(struct device *dev)
{
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(p7mu_rtc_pm_ops,
                         p7mu_suspend_rtc,
                         p7mu_resume_rtc);

static struct platform_driver p7mu_rtc_driver = {
	.driver = {
		.name	= P7MU_RTC_NAME,
		.owner	= THIS_MODULE,
		.pm     = &p7mu_rtc_pm_ops,
	},
	.probe	= &p7mu_probe_rtc,
	.remove	= __devexit_p(&p7mu_remove_rtc),
};

module_platform_driver(p7mu_rtc_driver);

MODULE_DESCRIPTION("Parrot Power Management Unit RTC driver");
MODULE_AUTHOR("Grégor Boirie <gregor.boirie@parrot.com>");
MODULE_LICENSE("GPL");
