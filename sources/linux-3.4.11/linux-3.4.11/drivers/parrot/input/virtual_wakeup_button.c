/*
 * virtual_wakeup_button.c - Power Button which is pushed on resume from standby.
 *
 * Copyright (c) 2011 Stefan Seidel
 * Copyright (c) 2014 Aurelien Lefebvre
 *
 * This file is released under the GPLv2 or later.
 */
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/suspend.h>
#include <linux/pm_runtime.h>

#define DRIVER_NAME "virtual_wakeup_button"

static int __devinit virtual_wakeup_button_probe(struct platform_device* pdev)
{
	struct input_dev *input;
	int error;

	dev_info(&pdev->dev, "probe\n");

	input = input_allocate_device();

	platform_set_drvdata(pdev, input);

	input->name = DRIVER_NAME;
	input->evbit[0] = BIT_MASK(EV_KEY);

	set_bit(KEY_POWER, input->keybit);

	error = input_register_device(input);

	if (error)
		input_free_device(input);

	return 0;
}

static int __devexit virtual_wakeup_button_remove(struct platform_device* pdev)
{
	struct input_dev *input;

	input = platform_get_drvdata(pdev);

	if (input)
		input_free_device(input);

	return 0;
}

static int virtual_wakeup_button_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct input_dev *input;

	input = platform_get_drvdata(pdev);

	if (!input)
		return 0;

	dev_info(dev, "resuming... sending KEY_POWER event\n");

	input_report_key(input, KEY_POWER, 1);
	input_sync(input);
	input_report_key(input, KEY_POWER, 0);
	input_sync(input);

	return 0;
}

static struct dev_pm_ops virtual_wakeup_button_dev_pm_ops = {
	.resume = &virtual_wakeup_button_resume,
};

static struct platform_driver virtual_wakeup_button_driver = {
	.driver         = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.pm     = &virtual_wakeup_button_dev_pm_ops,
	},
	.probe          = &virtual_wakeup_button_probe,
	.remove         = __devexit_p(&virtual_wakeup_button_remove),
};

static int __init virtual_wakeup_button_init(void)
{
	return platform_driver_register(&virtual_wakeup_button_driver);
}
module_init(virtual_wakeup_button_init);

static void __exit virtual_wakeup_button_exit(void)
{
	platform_driver_unregister(&virtual_wakeup_button_driver);
}
module_exit(virtual_wakeup_button_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stefan Seidel <android@stefanseidel.info>");
MODULE_AUTHOR("Aurelien Lefebvre <aurelien.lefebvre@parrot.com>");
MODULE_DESCRIPTION("Sets up a virtual input device and sends a power key event during resume.");


