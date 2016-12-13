/**
 * linux/drivers/parrot/gpio/p7mu-gpio.c - Parrot7 power management unit GPIOs
 *                                         implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 *
 * This file is released under the GPL
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <mfd/p7mu-pin.h>

DEFINE_MUTEX(                   p7mu_gpio_lck);
static struct resource const*   p7mu_gpio_res;

struct p7mu_gpio_chip {
	struct gpio_chip gpio_chip;
};


#ifdef DEBUG
#define P7MU_ASSERT_GPIO(_chip, _gpio)  \
    BUG_ON(! p7mu_gpio_res);            \
    BUG_ON(_gpio >= _chip->ngpio)
#else
#define P7MU_ASSERT_GPIO(_chip, _gpio)
#endif

/*
 * Reserve GPIO line for future usage.
 */
static int p7mu_request_gpio(struct gpio_chip* chip, unsigned int gpio)
{
	P7MU_ASSERT_GPIO(chip, gpio);

	/* Forward call to P7MU pin management layer. */
	return p7mu_request_pin(gpio, P7MU_INGPIO_MUX);
}

/*
 * Release GPIO line previously reserved with p7mu_request_gpio.
 */
static void p7mu_free_gpio(struct gpio_chip* chip, unsigned int gpio)
{
	P7MU_ASSERT_GPIO(chip, gpio);

	/* Forward call to P7MU pin management layer. */
	p7mu_free_pin(gpio);
}

/*
 * Switch GPIO line to input mode.
 */
static int p7mu_gpio_input(struct gpio_chip* chip, unsigned int gpio)
{
	P7MU_ASSERT_GPIO(chip, gpio);

	/* Forward call to P7MU pin management layer. */
	return p7mu_setup_pin(gpio, P7MU_INGPIO_MUX);
}

/*
 * Retrieve current value of GPIO line passed in argument.
 */
static int p7mu_get_gpio(struct gpio_chip* chip, unsigned int gpio)
{
	int val;
	int err;
	u16 p7mu_gpio_val;

	P7MU_ASSERT_GPIO(chip, gpio);

	err = p7mu_read16(p7mu_gpio_res->start, &p7mu_gpio_val);

	if (err)
		return err;

	val = !! (p7mu_gpio_val & (u16) (1U << gpio));

	return val;
}

/*
 * Set current value of GPIO line passed in argument.
 */
static void p7mu_set_gpio(struct gpio_chip* chip,
                          unsigned int gpio,
                          int value)
{
	int err;
	u16 p7mu_gpio_val;

	P7MU_ASSERT_GPIO(chip, gpio);

	mutex_lock(&p7mu_gpio_lck);

	/* Fetch all GPIO lines values. */
	err = p7mu_read16(p7mu_gpio_res->start, &p7mu_gpio_val);
	if (err) {
		goto err;
	}

	/* Update GPIO line value. */
	if (value)
		p7mu_gpio_val |= 1U << gpio;
	else
		p7mu_gpio_val &= ~(1U << gpio);

	err = p7mu_write16(p7mu_gpio_res->start, p7mu_gpio_val);
	if (! err)
		/* Success */
		goto unlock;

err:
	dev_warn(chip->dev,
	         "failed to set GPIO %d value (%d)\n",
	         gpio,
	         err);
unlock:
	mutex_unlock(&p7mu_gpio_lck);
}

/*
 * Switch GPIO line to output mode.
 */
static int p7mu_gpio_output(struct gpio_chip* chip, unsigned int gpio, int value)
{
	int err;

	P7MU_ASSERT_GPIO(chip, gpio);

	err = p7mu_setup_pin(gpio, P7MU_OUTGPIO_MUX);
	if (err)
		return err;

	p7mu_set_gpio(chip, gpio, value);
	return 0;
}

static int p7mu_gpio2irq(struct gpio_chip* chip, unsigned int gpio)
{
	return p7mu_gpio_to_irq(gpio);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

/*
 * Show requested GPIO lines current state.
 */
static void p7mu_show_gpio(struct seq_file* seq, struct gpio_chip* chip)
{
	unsigned int gpio;

	for (gpio = 0; gpio < chip->ngpio; gpio++) {
		char const* const label = gpiochip_is_requested(chip, gpio);

		P7MU_ASSERT_GPIO(chip, gpio);

		if (label) {
			char const* dir;
			int         irq;

			switch (p7mu_pin_mux(gpio)) {
			case P7MU_INGPIO_MUX:
				dir = "in ";
				break;

			case P7MU_OUTGPIO_MUX:
				dir = "out";
				break;
			default:
				dir = "?";
#ifdef DEBUG
				BUG();
#endif
			}

			seq_printf(seq,
			           " gpio-%-3d (%-20.20s) %s %s",
			           chip->base + gpio,
			           label,
			           dir,
			           p7mu_get_gpio(chip, gpio) ? "hi" : "lo");

			irq = gpio_to_irq(chip->base + gpio);
			if (irq >= 0)
				seq_printf(seq, " [IRQ %d]", irq);

			seq_printf(seq, "\n");
		}
	}
}

#define P7MU_SHOW_GPIO  p7mu_show_gpio

#else   /* CONFIG_DEBUG_FS */

#define P7MU_SHOW_GPIO  NULL

#endif  /* CONFIG_DEBUG_FS */

#define P7MU_GPIO_DRV_NAME  "p7mu-gpio"

static struct p7mu_gpio_chip p7mu_gpio_chip
= {
	.gpio_chip = {
		.label              = P7MU_GPIO_DRV_NAME,
		.request            = p7mu_request_gpio,
		.free               = p7mu_free_gpio,
		.direction_input    = p7mu_gpio_input,
		.get                = p7mu_get_gpio,
		.direction_output   = p7mu_gpio_output,
		.set                = p7mu_set_gpio,
		.base               = P7MU_FIRST_GPIO,
		.to_irq             = p7mu_gpio2irq,
		.ngpio              = P7MU_GPIOS_MAX,
		.dbg_show           = P7MU_SHOW_GPIO,
		.can_sleep          = 1,
	},
};

static int __devinit p7mu_probe_gpio(struct platform_device* pdev)
{
	int err;

	p7mu_gpio_res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (! p7mu_gpio_res) {
		dev_err(&pdev->dev, "failed to find I/O region address\n");
		err = -ENXIO;
		goto err;
	}

	p7mu_gpio_res = p7mu_request_region(pdev, p7mu_gpio_res);
	if (IS_ERR(p7mu_gpio_res)) {
		err = PTR_ERR(p7mu_gpio_res);
		goto err;
	}

	p7mu_gpio_chip.gpio_chip.dev = &pdev->dev;
	err = gpiochip_add(&p7mu_gpio_chip.gpio_chip);
	if (! err)
		return 0;

	p7mu_release_region(p7mu_gpio_res);

err:
	dev_err(&pdev->dev,
	        "failed to register gpio chip (%d)\n",
	        err);
	return err;
}

static int __devexit p7mu_remove_gpio(struct platform_device* pdev)
{
	int const err = gpiochip_remove(&p7mu_gpio_chip.gpio_chip);
	if (err) {
		dev_err(&pdev->dev,
		        "failed to unregister gpio chip (%d)\n",
		        err);
		return err;
	}

#ifdef DEBUG
	BUG_ON(! p7mu_gpio_res);
	BUG_ON(IS_ERR(p7mu_gpio_res));
#endif
	p7mu_release_region(p7mu_gpio_res);

	return 0;
}

static struct platform_driver p7mu_gpio_driver = {
	.driver = {
		.name   = P7MU_GPIO_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = p7mu_probe_gpio,
	.remove = __devexit_p(&p7mu_remove_gpio),
};

static int __init p7mu_init_gpio(void)
{
	return platform_driver_register(&p7mu_gpio_driver);
}
postcore_initcall(p7mu_init_gpio);

static void __exit p7mu_exit_gpio(void)
{
	platform_driver_unregister(&p7mu_gpio_driver);
}
module_exit(p7mu_exit_gpio);

MODULE_DESCRIPTION("Parrot Power Management Unit GPIO driver");
MODULE_AUTHOR("Grégor Boirie <gregor.boirie@parrot.com>");
MODULE_LICENSE("GPL");
