/**
 * linux/driver/parrot/uio/hx280-venc.c - On2 / Hantro user I/O video encoder
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
#include "hx280-venc.h"

#define DRV_VERSION "0.1"

#define HX280_PRODUCT_ID_R1     (0x72800000U)
#define HX280_PRODUCT_ID_R2     (0x48310000U)

#define HX280_REGS_NR           14
#define HX280_ID                0x0
#define HX280_ID_PRODUCT_MSK    (0xffff0000U)
#define HX280_IRQ               0x4
#define HX280_IRQ_HINTENC       (1U << 0)
#define HX280_IRQ_INTDISABLE    (1U << 1)
#define HX280_IRQ_SLICE_STATUS  (1U << 8)
#define HX280_COMCTRL           0x38

static int hx280_handle_irq(struct hx_device const* hxdev)
{
	unsigned long const reg = hxdev->regs_virt + HX280_IRQ;
	u32 const status = readl_relaxed(reg);

	if (unlikely(! (status & HX280_IRQ_HINTENC)))
		/* Spurious ? */
		return 0;

	writel_relaxed(status & ~(HX280_IRQ_SLICE_STATUS | HX280_IRQ_HINTENC),
			reg);

	return 1;
}

static void hx280_setup_irq(struct hx_device const* hxdev, int command)
{
	unsigned long const reg = hxdev->regs_virt + HX280_IRQ;
	u32 const           word = readl(reg);

	if (! command)
		/* Disable interrupts generation. */
		writel_relaxed(word | HX280_IRQ_INTDISABLE, reg);
	else
		/* Enable interrupts generature. */
		writel_relaxed(word & ~HX280_IRQ_INTDISABLE, reg);
}

static int __devinit hx280_init_hw(struct hx_device const* hxdev)
{
	unsigned long const regs = hxdev->regs_virt;
	u32 const           id = readl(regs + HX280_ID);
	unsigned int        r;

	/* Check hardware id. */
	if ((id & HX280_ID_PRODUCT_MSK) != HX280_PRODUCT_ID_R1 &&
	 (id & HX280_ID_PRODUCT_MSK) != HX280_PRODUCT_ID_R2) {
		dev_err(hxdev->device.parent,
		        "failed to find valid hardware id (0x%08x).\n",
		        id);
		return -ENODEV;
	}
	dev_info(hxdev->device.parent,
	         "found hx280 video encoder (0x%08x).\n",
	         id);

	/* Stop processing. */
	writel_relaxed(0, regs + HX280_COMCTRL);
	/* Disable & acknowledge interrupts. */
	writel(HX280_IRQ_INTDISABLE, regs + HX280_IRQ);
	/* Clear all remaining registers. FIXME: is this really necessary ? */
	for (r = 2 * sizeof(u32);
	     r < (sizeof(u32) * HX280_REGS_NR);
	     r += sizeof(u32))
		writel_relaxed(0, regs + r);
	wmb();

	return 0;
}

static int __devinit hx280_probe(struct platform_device* pdev)
{
	int                     err;
	struct hx_unit* const	unit = kmalloc(sizeof(*unit), GFP_KERNEL);

	if (! unit)
		return -ENOMEM;

	hx_init_unit(unit, &hx280_handle_irq, &hx280_setup_irq);
	err = hx_probe(pdev, "hx280", unit, 1, &hx280_init_hw);
	if (err)
		kfree(unit);

	return err;
}

static int __devexit hx280_remove(struct platform_device* pdev)
{
	int                     err;
	struct hx_unit* const	unit = ((struct hx_device*)
									dev_get_platdata(&pdev->dev))->units;

	err = hx_remove(pdev);
	if (! err)
		kfree(unit);

	return err;
}

static int hx280_suspend(struct platform_device *pdev, pm_message_t state)
{
	return hx_suspend(pdev);
}
static int hx280_resume(struct platform_device *pdev)
{
	return hx_resume(pdev, &hx280_init_hw);
}

static struct platform_driver hx280_driver = {
	.probe      = hx280_probe,
	.remove     = __devexit_p(hx280_remove),
	.driver     = {
		.name   = HX280_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.suspend    = &hx280_suspend,
	.resume     = &hx280_resume
};
module_platform_driver(hx280_driver);

MODULE_AUTHOR("Gregor Boirie <gregor.boirie@parrot.com>");
MODULE_DESCRIPTION("On2 / Hantro video encoder "
                   "userspace I/O platform driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
