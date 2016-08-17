/**
 * linux/driver/parrot/mmc/switch_voltage_regulator.c - Voltage switch
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Jeremie Samuel <jeremie.samuel.ext@parrot.com>
 * date:    06-Jun-2013
 *
 * based on gpio-regulator.c
 *
 * Copyright 2011 Heiko Stuebner <heiko@sntech.de>
 *
 * This file is released under the GPL
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/driver.h>
#include "switch-voltage-regulator.h"

struct switch_voltage_data {
	struct regulator_desc desc;
	struct regulator_dev *dev;

	bool is_enabled;
	struct switch_voltage_config *config;
};

static int switch_voltage_is_enabled(struct regulator_dev *dev)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	return data->is_enabled;
}

static int switch_voltage_enable(struct regulator_dev *dev)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	if (gpio_is_valid(data->config->enable_gpio)) {
		gpio_set_value_cansleep(data->config->enable_gpio,
					data->config->enable_high);
		data->is_enabled = true;
	}

	return 0;
}

static int switch_voltage_disable(struct regulator_dev *dev)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	if (gpio_is_valid(data->config->enable_gpio)) {
		gpio_set_value_cansleep(data->config->enable_gpio,
					!data->config->enable_high);
		data->is_enabled = false;
	}

	return 0;
}

static int switch_voltage_enable_time(struct regulator_dev *dev)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	return data->config->startup_delay;
}

static int switch_voltage_get_voltage(struct regulator_dev *dev)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	if (gpio_is_valid(data->config->switch_gpio)) {
		int gpio_value = gpio_get_value_cansleep(data->config->switch_gpio);
		int ptr;
		for (ptr = 0; ptr < 2; ptr++)
			if (data->config->states[ptr].gpio == gpio_value)
				return data->config->states[ptr].value;
	} else
		return data->config->states[0].value;

	return -EINVAL;
}

static int switch_voltage_set_voltage(struct regulator_dev *dev,
					int min_uV, int max_uV,
					unsigned *selector)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	if (gpio_is_valid(data->config->switch_gpio)) {
		unsigned int ptr, target;
		bool was_enabled = data->is_enabled;

		target = -1;
		for (ptr = 0; ptr < 2; ptr++)
			if (data->config->states[ptr].value >= min_uV &&
			    data->config->states[ptr].value <= max_uV)
				target = data->config->states[ptr].gpio;

		if (target < 0)
			return -EINVAL;

		if (was_enabled && data->config->switch_disabled)
			switch_voltage_disable(dev);

		gpio_set_value_cansleep(data->config->switch_gpio, target);

		if (data->config->switch_delay)
			msleep(data->config->switch_delay);

		if (was_enabled && data->config->switch_disabled)
			switch_voltage_enable(dev);
	}

	return 0;
}

static int switch_voltage_list_voltage(struct regulator_dev *dev,
				      unsigned selector)
{
	struct switch_voltage_data *data = rdev_get_drvdata(dev);

	if (selector >= 2)
		return -EINVAL;

	if (gpio_is_valid(data->config->switch_gpio))
		return data->config->states[selector].value;
	else
		return data->config->states[0].value;
}

static struct regulator_ops switch_voltage_ops = {
	.is_enabled = switch_voltage_is_enabled,
	.enable = switch_voltage_enable,
	.disable = switch_voltage_disable,
	.enable_time = switch_voltage_enable_time,
	.get_voltage = switch_voltage_get_voltage,
	.set_voltage = switch_voltage_set_voltage,
	.list_voltage = switch_voltage_list_voltage,
};

static int __devinit switch_voltage_probe(struct platform_device *pdev)
{
	struct switch_voltage_config *config = pdev->dev.platform_data;
	struct switch_voltage_data *drvdata;
	int ret;

	drvdata = kzalloc(sizeof(struct switch_voltage_data), GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&pdev->dev, "Failed to allocate device data\n");
		return -ENOMEM;
	}

	drvdata->desc.name = config->supply_name;

	drvdata->desc.owner = THIS_MODULE;
	drvdata->desc.type = REGULATOR_VOLTAGE;
	drvdata->desc.ops = &switch_voltage_ops;

	drvdata->config = config;

	if (gpio_is_valid(config->enable_gpio)) {
		ret = gpio_request(config->enable_gpio, config->supply_name);
		if (ret) {
			dev_err(&pdev->dev,
			   "Could not obtain regulator enable GPIO %d: %d\n",
						config->enable_gpio, ret);
			goto err;
		}

		/* set output direction without changing state
		 * to prevent glitch
		 */
		if (config->enabled_at_boot) {
			drvdata->is_enabled = true;
			ret = gpio_direction_output(config->enable_gpio,
						    config->enable_high);
		} else {
			drvdata->is_enabled = false;
			ret = gpio_direction_output(config->enable_gpio,
						    !config->enable_high);
		}

		if (ret) {
			dev_err(&pdev->dev,
			   "Could not configure regulator enable GPIO %d direction: %d\n",
						config->enable_gpio, ret);
			goto err_enablegpio;
		}
	} else {
		/* Regulator without GPIO control is considered
		 * always enabled
		 */
		drvdata->is_enabled = true;
	}

	if (gpio_is_valid(config->switch_gpio)) {
		ret = gpio_request(config->switch_gpio, config->supply_name);
		if (ret) {
			dev_err(&pdev->dev,
			   "Could not obtain regulator switch GPIO %d: %d\n",
						config->switch_gpio, ret);
			goto err_enablegpio;
		}

		ret = gpio_direction_output(config->switch_gpio,
					    config->states[0].gpio);
		if (ret) {
			dev_err(&pdev->dev,
			   "Could not configure regulator switch GPIO %d direction: %d\n",
						config->switch_gpio, ret);
			goto err_switchgpio;
		}

		drvdata->desc.n_voltages = 2;
	} else
		drvdata->desc.n_voltages = 1;

	drvdata->dev = regulator_register(&drvdata->desc, &pdev->dev,
					  config->init_data, drvdata, NULL);
	if (IS_ERR(drvdata->dev)) {
		ret = PTR_ERR(drvdata->dev);
		dev_err(&pdev->dev, "Failed to register regulator: %d\n", ret);
		goto err_switchgpio;
	}

	platform_set_drvdata(pdev, drvdata);

	return 0;

err_switchgpio:
	if (gpio_is_valid(config->switch_gpio))
		gpio_free(config->switch_gpio);
err_enablegpio:
	if (gpio_is_valid(config->enable_gpio))
		gpio_free(config->enable_gpio);
err:
	kfree(drvdata);
	return ret;
}

static int __devexit switch_voltage_remove(struct platform_device *pdev)
{
	struct switch_voltage_data *drvdata = platform_get_drvdata(pdev);

	regulator_unregister(drvdata->dev);

	if (gpio_is_valid(drvdata->config->switch_gpio))
		gpio_free(drvdata->config->switch_gpio);
	if (gpio_is_valid(drvdata->config->enable_gpio))
		gpio_free(drvdata->config->enable_gpio);

	kfree(drvdata->desc.name);
	kfree(drvdata);

	return 0;
}

static struct platform_driver switch_voltage_driver = {
	.probe		= switch_voltage_probe,
	.remove		= __devexit_p(switch_voltage_remove),
	.driver		= {
		.name		= "switch-voltage",
		.owner		= THIS_MODULE,
	},
};

static int __init switch_voltage_init(void)
{
	return platform_driver_register(&switch_voltage_driver);
}
subsys_initcall(switch_voltage_init);

static void __exit switch_voltage_exit(void)
{
	platform_driver_unregister(&switch_voltage_driver);
}
module_exit(switch_voltage_exit);

MODULE_AUTHOR("Jeremie Samuel<jeremie.samuel.ext@parrot.com>");
MODULE_DESCRIPTION("switch voltage regulator");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:switch-voltage");
