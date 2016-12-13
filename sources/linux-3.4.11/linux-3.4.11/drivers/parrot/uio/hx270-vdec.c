/**
 * linux/driver/parrot/uio/hx270-vdec.c - On2 / Hantro user I/O video decoder
 *                                        implementation
 *
 * Copyright (C) 2011 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    31-Aug-2011
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "hx-uio.h"
#include "hx270-vdec.h"

#define DRV_VERSION "0.1"

#define HX270_REGS_NR           101
#define HX270_ID                0x0
#define HX270_ID_PRODUCT_SHIFT  16
#define HX270_DECIRQ            0x4
#define HX270_DECIRQ_IRQDISABLE (1U << 4)
#define HX270_DECIRQ_IRQ        (1U << 8)
#define HX270_PPIRQ             0xf0
#define HX270_PPIRQ_IRQDISABLE  (1U << 4)
#define HX270_PPIRQ_IRQ         (1U << 8)

static int hx270_handle_decirq(struct hx_device const* hxdev)
{
	unsigned long const reg = hxdev->regs_virt + HX270_DECIRQ;
	u32 const           status = readl_relaxed(reg);

	if (! (status & HX270_DECIRQ_IRQ))
		return 0;

	/* Acknowledge interrupt. */
	writel_relaxed(status & ~HX270_DECIRQ_IRQ, reg);
	return 1;
}

static void hx270_setup_decirq(struct hx_device const* hxdev, int command)
{
	unsigned long const reg = hxdev->regs_virt + HX270_DECIRQ;
	u32 const           word = readl(reg);

	if (! command)
		/* Disable interrupts generation. */
		writel_relaxed(word | HX270_DECIRQ_IRQDISABLE, reg);
	else
		/* Enable interrupts generature. */
		writel_relaxed(word & ~HX270_DECIRQ_IRQDISABLE, reg);
}

static int hx270_handle_ppirq(struct hx_device const* hxdev)
{
	unsigned long const reg = hxdev->regs_virt + HX270_PPIRQ;
	u32 const           status = readl_relaxed(reg);

	if (! (status & HX270_PPIRQ_IRQ))
		return 0;

	/* Acknowledge interrupt. */
	writel_relaxed(status & ~HX270_PPIRQ_IRQ, reg);
	return 1;
}

static void hx270_setup_ppirq(struct hx_device const* hxdev, int command)
{
	unsigned long const reg = hxdev->regs_virt + HX270_PPIRQ;
	u32 const           word = readl(reg);

	if (! command)
		/* Disable interrupts generation. */
		writel_relaxed(word | HX270_PPIRQ_IRQDISABLE, reg);
	else
		/* Enable interrupts generature. */
		writel_relaxed(word & ~HX270_PPIRQ_IRQDISABLE, reg);
}

static int __devinit hx270_init_hw(struct hx_device const* hxdev)
{
	static unsigned short const ids[] = { 0x8190, 0x8170, 0x9170, 0x9190, 0x6731 };
	unsigned long const         regs = hxdev->regs_virt;
	u32 const                   id = readl(regs + HX270_ID);
	unsigned int                i;

	/* Check hardware id. */
	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		if (((unsigned short) (id >> HX270_ID_PRODUCT_SHIFT)) == ids[i])
			break;
	}
	if (i >= ARRAY_SIZE(ids)) {
		dev_err(hxdev->device.parent,
		        "failed to find valid hardware id (0x%08x).\n",
		        id);
		return -ENODEV;
	}
	dev_info(hxdev->device.parent,
	         "found hx270 video decoder (0x%08x).\n",
	         id);

	/* Disable & acknowledge decoder interrupts, stop decoder. */
	writel(HX270_DECIRQ_IRQDISABLE, regs + HX270_DECIRQ);
	/* Disable & acknowledge post-processor interrupts, stop post-processor. */
	writel(HX270_PPIRQ_IRQDISABLE, regs + HX270_PPIRQ);

	/* FIXME: is this really necessary ?
	 * Clear all remaining decoder registers... */
	for (i = 2 * sizeof(u32);
	     i < (sizeof(u32) * 60);
	     i += sizeof(u32))
		writel_relaxed(0, regs + i);
	/* ...and post-processor registers. */
	for (i = 61 * sizeof(u32);
	     i < (sizeof(u32) * HX270_REGS_NR);
	     i += sizeof(u32))
		writel_relaxed(0, regs + i);
	wmb();

	return 0;
}

static int __devinit hx270_probe(struct platform_device* pdev)
{
	int                     err;
	struct hx_unit* const	units = kmalloc(2 * sizeof(*units), GFP_KERNEL);

	if (! units)
		return -ENOMEM;

	hx_init_unit(&units[0], &hx270_handle_decirq, &hx270_setup_decirq);
	hx_init_unit(&units[1], &hx270_handle_ppirq, &hx270_setup_ppirq);
	err = hx_probe(pdev, "hx270", units, 2, &hx270_init_hw);
	if (err)
		kfree(units);

	return err;
}

static int __devexit hx270_remove(struct platform_device* pdev)
{
	int                     err;
	struct hx_unit* const	units = ((struct hx_device*)
	                                 dev_get_platdata(&pdev->dev))->units;

	err = hx_remove(pdev);
	if (! err)
		kfree(units);

	return err;
}
static int hx270_suspend(struct platform_device *pdev, pm_message_t state)
{
	return hx_suspend(pdev);
}
static int hx270_resume(struct platform_device *pdev)
{
	return hx_resume(pdev, &hx270_init_hw);
}

static struct platform_driver hx270_driver = {
	.probe      = hx270_probe,
	.remove     = __devexit_p(hx270_remove),
	.driver     = {
		.name   = HX270_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	/* TODO: add support for optional suspend / resume. */
	.suspend    = hx270_suspend,
	.resume     = hx270_resume
};
module_platform_driver(hx270_driver);

MODULE_AUTHOR("Gregor Boirie");
MODULE_DESCRIPTION("On2 / Hantro video decoder / post-processor "
                   "userspace I/O platform driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
