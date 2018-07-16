/*
 * core.c - ChipIdea USB IP core family device controller
 *
 * Copyright (C) 2008 Chipidea - MIPS Technologies, Inc. All rights reserved.
 *
 * Author: David Lopo
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/*
 * Description: ChipIdea USB IP core family device controller
 *
 * This driver is composed of several blocks:
 * - HW:     hardware interface
 * - DBG:    debug facilities (optional)
 * - UTIL:   utilities
 * - ISR:    interrupts handling
 * - ENDPT:  endpoint operations (Gadget API)
 * - GADGET: gadget operations (Gadget API)
 * - BUS:    bus glue code, bus abstraction layer
 *
 * Compile Options
 * - CONFIG_USB_GADGET_DEBUG_FILES: enable debug facilities
 * - STALL_IN:  non-empty bulk-in pipes cannot be halted
 *              if defined mass storage compliance succeeds but with warnings
 *              => case 4: Hi >  Dn
 *              => case 5: Hi >  Di
 *              => case 8: Hi <> Do
 *              if undefined usbtest 13 fails
 * - TRACE:     enable function tracing (depends on DEBUG)
 *
 * Main Features
 * - Chapter 9 & Mass Storage Compliance with Gadget File Storage
 * - Chapter 9 Compliance with Gadget Zero (STALL_IN undefined)
 * - Normal & LPM support
 *
 * USBTEST Report
 * - OK: 0-12, 13 (STALL_IN defined) & 14
 * - Not Supported: 15 & 16 (ISO)
 *
 * TODO List
 * - OTG
 * - Interrupt Traffic
 * - Handle requests which spawns into several TDs
 * - GET_STATUS(device) - always reports 0
 * - Gadget API (majority of optional features)
 * - Suspend & Remote Wakeup
 */
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/chipidea.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/jiffies.h>

#include "ci.h"
#include "udc.h"
#include "bits.h"
#include "host.h"
#include "debug.h"

/* Controller register map */
static uintptr_t ci_regs_nolpm[] = {
	[CAP_CAPLENGTH]		= 0x000UL,
	[CAP_HCCPARAMS]		= 0x008UL,
	[CAP_DCCPARAMS]		= 0x024UL,
	[CAP_TESTMODE]		= 0x038UL,
	[OP_USBCMD]		= 0x000UL,
	[OP_USBSTS]		= 0x004UL,
	[OP_USBINTR]		= 0x008UL,
	[OP_DEVICEADDR]		= 0x014UL,
	[OP_ENDPTLISTADDR]	= 0x018UL,
	[OP_VIEWPORT]		= 0x030UL,
	[OP_PORTSC]		= 0x044UL,
	[OP_DEVLC]		= 0x084UL,
	[OP_OTGSC]		= 0x064UL,
	[OP_USBMODE]		= 0x068UL,
	[OP_ENDPTSETUPSTAT]	= 0x06CUL,
	[OP_ENDPTPRIME]		= 0x070UL,
	[OP_ENDPTFLUSH]		= 0x074UL,
	[OP_ENDPTSTAT]		= 0x078UL,
	[OP_ENDPTCOMPLETE]	= 0x07CUL,
	[OP_ENDPTCTRL]		= 0x080UL,
};

static uintptr_t ci_regs_lpm[] = {
	[CAP_CAPLENGTH]		= 0x000UL,
	[CAP_HCCPARAMS]		= 0x008UL,
	[CAP_DCCPARAMS]		= 0x024UL,
	[CAP_TESTMODE]		= 0x0FCUL,
	[OP_USBCMD]		= 0x000UL,
	[OP_USBSTS]		= 0x004UL,
	[OP_USBINTR]		= 0x008UL,
	[OP_DEVICEADDR]		= 0x014UL,
	[OP_ENDPTLISTADDR]	= 0x018UL,
	[OP_PORTSC]		= 0x044UL,
	[OP_DEVLC]		= 0x084UL,
	[OP_OTGSC]		= 0x0C4UL,
	[OP_USBMODE]		= 0x0C8UL,
	[OP_ENDPTSETUPSTAT]	= 0x0D8UL,
	[OP_ENDPTPRIME]		= 0x0DCUL,
	[OP_ENDPTFLUSH]		= 0x0E0UL,
	[OP_ENDPTSTAT]		= 0x0E4UL,
	[OP_ENDPTCOMPLETE]	= 0x0E8UL,
	[OP_ENDPTCTRL]		= 0x0ECUL,
};

static int hw_alloc_regmap(struct ci13xxx *ci, bool is_lpm)
{
	int i;

	kfree(ci->hw_bank.regmap);

	ci->hw_bank.regmap = kzalloc((OP_LAST + 1) * sizeof(void *),
				     GFP_KERNEL);
	if (!ci->hw_bank.regmap)
		return -ENOMEM;

	for (i = 0; i < OP_ENDPTCTRL; i++)
		ci->hw_bank.regmap[i] =
			(i <= CAP_LAST ? ci->hw_bank.cap : ci->hw_bank.op) +
			(is_lpm ? ci_regs_lpm[i] : ci_regs_nolpm[i]);

	for (; i <= OP_LAST; i++)
		ci->hw_bank.regmap[i] = ci->hw_bank.op +
			4 * (i - OP_ENDPTCTRL) +
			(is_lpm
			 ? ci_regs_lpm[OP_ENDPTCTRL]
			 : ci_regs_nolpm[OP_ENDPTCTRL]);

	return 0;
}

/**
 * hw_port_test_set: writes port test mode (execute without interruption)
 * @mode: new value
 *
 * This function returns an error code
 */
int hw_port_test_set(struct ci13xxx *ci, u8 mode)
{
	const u8 TEST_MODE_MAX = 7;

	if (mode > TEST_MODE_MAX)
		return -EINVAL;

	hw_write(ci, OP_PORTSC, PORTSC_PTC, mode << __ffs(PORTSC_PTC));
	return 0;
}

/**
 * hw_port_test_get: reads port test mode value
 *
 * This function returns port test mode value
 */
u8 hw_port_test_get(struct ci13xxx *ci)
{
	return hw_read(ci, OP_PORTSC, PORTSC_PTC) >> __ffs(PORTSC_PTC);
}

static int hw_device_init(struct ci13xxx *ci, void __iomem *base)
{
	u32 reg;

	/* bank is a module variable */
	ci->hw_bank.abs = base;

	ci->hw_bank.cap = ci->hw_bank.abs;
	ci->hw_bank.cap += ci->udc_driver->capoffset;
	ci->hw_bank.op = ci->hw_bank.cap + (ioread32(ci->hw_bank.cap) & 0xff);

	hw_alloc_regmap(ci, false);
	reg = hw_read(ci, CAP_HCCPARAMS, HCCPARAMS_LEN) >>
		__ffs(HCCPARAMS_LEN);
	ci->hw_bank.lpm  = reg;
	hw_alloc_regmap(ci, !!reg);
	ci->hw_bank.size = ci->hw_bank.op - ci->hw_bank.abs;
	ci->hw_bank.size += OP_LAST;
	ci->hw_bank.size /= sizeof(u32);

	reg = hw_read(ci, CAP_DCCPARAMS, DCCPARAMS_DEN) >>
		__ffs(DCCPARAMS_DEN);
	ci->hw_ep_max = reg * 2;   /* cache hw ENDPT_MAX */

	if (ci->hw_ep_max > ENDPT_MAX)
		return -ENODEV;

	dev_dbg(ci->dev, "ChipIdea HDRC found, lpm: %d; cap: %p op: %p\n",
		ci->hw_bank.lpm, ci->hw_bank.cap, ci->hw_bank.op);

	/* setup lock mode ? */

	/* ENDPTSETUPSTAT is '0' by default */

	/* HCSPARAMS.bf.ppc SHOULD BE zero for device */

	return 0;
}

/**
 * hw_device_reset: resets chip (execute without interruption)
 * @ci: the controller
  *
 * This function returns an error code
 */
int hw_device_reset(struct ci13xxx *ci, u32 mode)
{
	/* should flush & stop before reset */
	hw_write(ci, OP_ENDPTFLUSH, ~0, ~0);
	hw_write(ci, OP_USBCMD, USBCMD_RS, 0);

	hw_write(ci, OP_USBCMD, USBCMD_RST, USBCMD_RST);
	while (hw_read(ci, OP_USBCMD, USBCMD_RST))
		udelay(10);		/* not RTOS friendly */


	if (ci->udc_driver->notify_event)
		ci->udc_driver->notify_event(ci,
			CI13XXX_CONTROLLER_RESET_EVENT);

	if (ci->udc_driver->flags & CI13XXX_DISABLE_STREAMING)
		hw_write(ci, OP_USBMODE, USBMODE_CI_SDIS, USBMODE_CI_SDIS);

	/* USBMODE should be configured step by step */
	hw_write(ci, OP_USBMODE, USBMODE_CM, USBMODE_CM_IDLE);
	hw_write(ci, OP_USBMODE, USBMODE_CM, mode);
	/* HW >= 2.3 */
	hw_write(ci, OP_USBMODE, USBMODE_SLOM, USBMODE_SLOM);

	if (hw_read(ci, OP_USBMODE, USBMODE_CM) != mode) {
		pr_err("cannot enter in %s mode", ci_role(ci)->name);
		pr_err("lpm = %i", ci->hw_bank.lpm);
		return -ENODEV;
	}

	return 0;
}

/**
 * ci_otg_role - pick role based on ID pin state
 * @ci: the controller
 */
static enum ci_role ci_otg_role(struct ci13xxx *ci)
{
	u32 sts = hw_read(ci, OP_OTGSC, ~0);
	enum ci_role role = sts & OTGSC_ID
		? CI_ROLE_GADGET
		: CI_ROLE_HOST;

	return role;
}

/**
 * ci_role_work - perform role changing based on ID pin
 * @work: work struct
 */
static void ci_role_work(struct work_struct *work)
{
	struct ci13xxx *ci = container_of(work, struct ci13xxx, work);
	enum ci_role role = ci_otg_role(ci);

	hw_write(ci, OP_OTGSC, OTGSC_IDIS, OTGSC_IDIS);

	if (role != ci->role) {
		dev_dbg(ci->dev, "switching from %s to %s\n",
			ci_role(ci)->name, ci->roles[role]->name);

		ci_role_switch(ci, role);
	}
}

static ssize_t show_role(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct ci13xxx *ci = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", ci_role(ci)->name);
}

static ssize_t store_role(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct ci13xxx *ci = dev_get_drvdata(dev);
	struct platform_device *pdev = to_platform_device(dev);
	enum ci_role role;
	int ret;

	for (role = CI_ROLE_HOST; role < CI_ROLE_END; role++)
		if (ci->roles[role] && !strcmp(buf, ci->roles[role]->name))
			break;

	if (role == CI_ROLE_END || role == ci->role)
		return -EINVAL;

	ret = ci_role_switch(ci, role);
	if (ret)
		return ret;

	// set vbus if new role is host, unset if role is device

	// Apple role switch don't like power to be cut down between
	// role_stop & start so do ithere, it will not cause much harm
	// take note that vbus will not be disabled if vbus_always_on = 1
	ci->udc_driver->set_vbus(pdev,
		((role == CI_ROLE_HOST) || ci->vbus_always_on) ? 1 : 0);

	return count;
}

static DEVICE_ATTR(role, S_IRUSR | S_IWUSR, show_role, store_role);

static ssize_t store_vbus(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct ci13xxx *ci = dev_get_drvdata(dev);
	struct platform_device *pdev = to_platform_device(dev);

	if(buf[0] != '0' && buf[0] != '1')
		return -EINVAL;

	if (ci->vbus_always_on && (buf[0] == '0'))
		return -EINVAL;

	ci->udc_driver->set_vbus(pdev, buf[0] - '0');

	return count;
}

static DEVICE_ATTR(vbus, S_IWUSR, NULL, store_vbus);

static ssize_t show_vbus_always_on(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct ci13xxx *ci = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", ci->vbus_always_on);
}

static ssize_t store_vbus_always_on(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ci13xxx *ci = dev_get_drvdata(dev);

	if (buf[0] == '0')
		ci->vbus_always_on = 0;
	else if (buf[0] == '1')
		ci->vbus_always_on = 1;
	else
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(vbus_always_on, S_IRUGO | S_IWUSR,
		show_vbus_always_on, store_vbus_always_on);

static irqreturn_t ci_irq(int irq, void *data)
{
	struct ci13xxx *ci = data;
	irqreturn_t ret = IRQ_NONE;

	/*If a switch is ongoing, this is really a spurious interrupt */
	if(!spin_trylock(&ci->switch_lock))
		return ret;

	if (ci->is_otg) {
		u32 sts = hw_read(ci, OP_OTGSC, ~0);

		if (sts & OTGSC_IDIS) {
			queue_work(ci->wq, &ci->work);
			ret = IRQ_HANDLED;
		}
	}

	ret = ci->role_itr == CI_ROLE_END ?
			ret : ci->roles[ci->role_itr]->irq(ci);

	spin_unlock(&ci->switch_lock);

	return ret;
}

int ci_ulpi_write(struct platform_device *pdev, u8 addr, u8 val)
{
	struct ci13xxx *ci = platform_get_drvdata(pdev);
	u32 data = VIEWPORT_ULPIRUN		|
			VIEWPORT_ULPIRW			|
			VIEWPORT_ULPIADDR(addr)	|
			VIEWPORT_ULPIDATWR(val);
	unsigned long timeout = jiffies + HZ;

	hw_write(ci, OP_VIEWPORT, ~0, data);

	do {
		data = hw_read(ci, OP_VIEWPORT, ~0);
		schedule();
	} while ((data & VIEWPORT_ULPIRUN)
		 && time_before(jiffies, timeout));

	return (data & VIEWPORT_ULPIRUN) ? -1 : 0;
}

static inline void ci_role_destroy(struct ci13xxx *ci)
{
	ci_hdrc_gadget_destroy(ci);
	ci_hdrc_host_destroy(ci);
}

static int __devinit ci_hdrc_probe(struct platform_device *pdev)
{
	struct device	*dev = &pdev->dev;
	struct ci13xxx	*ci;
	struct resource	*res;
	void __iomem	*base;
	int		ret;

	if (!dev->platform_data) {
		dev_err(dev, "platform data missing\n");
		return -ENODEV;
	}

	{
		static char clk_name[10];
		struct clk *usb_clk;
		snprintf(clk_name, 10, "usb_clk.%d", pdev->id);

		/* enable clock */
		usb_clk = clk_get(&pdev->dev, clk_name);
		if (IS_ERR(usb_clk)) {
			dev_err(&pdev->dev, "failed to get clock\n");
			return -EIO;
		}
		clk_prepare_enable(usb_clk);
		clk_put(usb_clk);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing resource\n");
		return -ENODEV;
	}

	base = devm_request_and_ioremap(dev, res);
	if (!res) {
		dev_err(dev, "can't request and ioremap resource\n");
		return -ENOMEM;
	}

	ci = devm_kzalloc(dev, sizeof(*ci), GFP_KERNEL);
	if (!ci) {
		dev_err(dev, "can't allocate device\n");
		return -ENOMEM;
	}

	ci->dev = dev;
	ci->udc_driver = dev->platform_data;

	spin_lock_init(&ci->switch_lock);

	ret = hw_device_init(ci, base);
	if (ret < 0) { 
		dev_err(dev, "can't initialize hardware\n");
		return -ENODEV;
	}

	ci->hw_bank.phys = res->start;

	ci->irq = platform_get_irq(pdev, 0);
	if (ci->irq < 0) {
		dev_err(dev, "missing IRQ\n");
		return -ENODEV;
	}

	INIT_WORK(&ci->work, ci_role_work);
	ci->wq = create_singlethread_workqueue("ci_otg");
	if (!ci->wq) {
		dev_err(dev, "can't create workqueue\n");
		return -ENODEV;
	}

	/* initialize role(s) before the interrupt is requested */
	if(ci->udc_driver->operating_mode == CI_UDC_DR_HOST ||
		ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_DEVICE ||
		ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_HOST)
	{
		ret = ci_hdrc_host_init(ci);
		if (ret)
			dev_info(dev, "doesn't support host\n");
	}

	if(ci->udc_driver->operating_mode == CI_UDC_DR_DEVICE ||
		ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_DEVICE ||
		ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_HOST)
	{
		ret = ci_hdrc_gadget_init(ci);
		if (ret)
			dev_info(dev, "doesn't support gadget\n");
	}

	if (!ci->roles[CI_ROLE_HOST] && !ci->roles[CI_ROLE_GADGET]) {
		dev_err(dev, "no supported roles\n");
		ret = -ENODEV;
		goto rm_wq;
	}

	if (ci->roles[CI_ROLE_HOST] && ci->roles[CI_ROLE_GADGET]) {
//		ci->is_otg = true;
//		ci->role = ci_otg_role(ci);
		if(ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_DEVICE)
			ci->role = CI_ROLE_GADGET;
		else
			ci->role = CI_ROLE_HOST;
	} else {
		ci->role = ci->roles[CI_ROLE_HOST]
			? CI_ROLE_HOST
			: CI_ROLE_GADGET;
	}

	ci->vbus_active = 1;

	platform_set_drvdata(pdev, ci);

	if (ci->udc_driver->init && ci->udc_driver->init(pdev, ci_ulpi_write)) // Call Parrot specific init (auto calibration, p7mu ...) need platform_set_drvdata
		goto stop;

	if(ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_HOST)
		ci->roles[CI_ROLE_GADGET]->stop(ci); // workaround needed since device is enabled so stop it to be able to start host

	if(ci->udc_driver->operating_mode == CI_UDC_DR_DUAL_HOST ||
		ci->udc_driver->operating_mode == CI_UDC_DR_HOST) // device is already initialized so avoid to do it a second time
	{
		ret = ci_role_switch(ci, ci->role);
		if (ret) {
			dev_err(dev, "can't start host role\n");
			ret = -ENODEV;
			goto de_init;
		}
	}

	ci->role_itr = ci->role;

	ret = request_irq(ci->irq, ci_irq, IRQF_SHARED, ci->udc_driver->name,
			  ci);
	if (ret)
		goto de_init;

	ret = device_create_file(dev, &dev_attr_role);
	if (ret)
		goto de_init;

	ret = device_create_file(dev, &dev_attr_vbus);
	if (ret)
		goto rm_attr;

	ret = device_create_file(dev, &dev_attr_vbus_always_on);
	if (ret)
		goto rm_attr2;

	if (ci->is_otg)
		hw_write(ci, OP_OTGSC, OTGSC_IDIE, OTGSC_IDIE);

	return ret;

rm_attr2:
	device_remove_file(dev, &dev_attr_vbus);
rm_attr:
	device_remove_file(dev, &dev_attr_role);
de_init:
	ci->udc_driver->exit(pdev);
stop:
	ci_role_destroy(ci);
rm_wq:
	flush_workqueue(ci->wq);
	destroy_workqueue(ci->wq);

	return ret;
}

static int __devexit ci_hdrc_remove(struct platform_device *pdev)
{
	struct ci13xxx *ci = platform_get_drvdata(pdev);

	flush_workqueue(ci->wq);
	destroy_workqueue(ci->wq);
	ci_role_destroy(ci);
	ci->udc_driver->exit(pdev);
	device_remove_file(ci->dev, &dev_attr_role);
	device_remove_file(ci->dev, &dev_attr_vbus);
	device_remove_file(ci->dev, &dev_attr_vbus_always_on);
	free_irq(ci->irq, ci);

	{
		static char clk_name[10];
		struct clk *usb_clk;
		snprintf(clk_name, 10, "usb_clk.%d", pdev->id);

		/* disable clock */
		usb_clk = clk_get(&pdev->dev, clk_name);
		if (IS_ERR(usb_clk)) {
			dev_err(&pdev->dev, "failed to get clock\n");
			return -EIO;
		}
		clk_disable_unprepare(usb_clk);
		clk_put(usb_clk);
	}

	return 0;
}

#ifdef CONFIG_PM
int ci_hdrc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ci13xxx *ci = platform_get_drvdata(pdev);
	static char clk_name[10];
	struct clk *usb_clk;

	dev_dbg(dev, "suspending...\n");

	ci_hdrc_host_suspend(ci);
	ci_hdrc_gadget_suspend(ci);

	snprintf(clk_name, 10, "usb_clk.%d", pdev->id);

	usb_clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(usb_clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return -EIO;
	}

	clk_disable_unprepare(usb_clk);
	clk_put(usb_clk);

	return 0;
}

int ci_hdrc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct ci13xxx *ci = platform_get_drvdata(pdev);
	static char clk_name[10];
	struct clk *usb_clk;

	dev_dbg(dev, "resuming...\n");

	snprintf(clk_name, 10, "usb_clk.%d", pdev->id);

	usb_clk = clk_get(&pdev->dev, clk_name);
	if (IS_ERR(usb_clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return -EIO;
	}

	clk_prepare_enable(usb_clk);
	clk_put(usb_clk);

	ci_hdrc_host_resume(ci);
	ci_hdrc_gadget_resume(ci);

	return 0;
}

static struct dev_pm_ops ci_hdrc_dev_pm_ops = {
	.suspend = ci_hdrc_suspend,
	.resume = ci_hdrc_resume,
};
#endif


static struct platform_driver ci_hdrc_driver = {
	.probe	= ci_hdrc_probe,
	.remove	= __devexit_p(ci_hdrc_remove),
	.driver	= {
		.name	= "ci_hdrc",
#ifdef CONFIG_PM
		.pm = &ci_hdrc_dev_pm_ops,
#endif
	},
};

module_platform_driver(ci_hdrc_driver);

MODULE_ALIAS("platform:ci_hdrc");
MODULE_ALIAS("platform:ci13xxx");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Lopo <dlopo@chipidea.mips.com>");
MODULE_DESCRIPTION("ChipIdea HDRC Driver");
