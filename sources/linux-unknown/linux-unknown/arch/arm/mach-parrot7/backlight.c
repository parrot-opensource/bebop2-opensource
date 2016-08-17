/**
 * linux/arch/arm/mach-parrot7/backlight.h - Parrot7 backlight platform
 *                                           implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    23-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/pwm_backlight.h>
#include <linux/fb.h>
#include "common.h"

#define P7_PWMBKL_NAME_MAX  32

struct p7_pwm_bkl {
	struct platform_pwm_backlight_data  data;
	unsigned int                        en_gpio;
	int                                 rst_gpio;
	struct device const*                fbdev;
	char                                en_name[P7_PWMBKL_NAME_MAX];
	char                                rst_name[P7_PWMBKL_NAME_MAX];
};

static int p7_init_pwmbkl(struct device* dev)
{
	struct p7_pwm_bkl* const    pdata = dev_get_platdata(dev);
	int                         err;

	if (gpio_is_valid(pdata->rst_gpio)) {
		/* Reset backlight controller. */
		int err;

		snprintf(pdata->rst_name, sizeof(pdata->rst_name), "%s rst", dev_name(dev));
		pdata->rst_name[P7_PWMBKL_NAME_MAX - 1] = '\0';
		err = gpio_request_one(pdata->rst_gpio,
		                       GPIOF_OUT_INIT_HIGH,
		                       pdata->rst_name);
		if (err)
			return err;
	}

	/* Enable backlight. */
	snprintf(pdata->en_name, sizeof(pdata->en_name), "%s en", dev_name(dev));
	pdata->en_name[P7_PWMBKL_NAME_MAX - 1] = '\0';
	err = gpio_request_one(pdata->en_gpio,
	                       GPIOF_OUT_INIT_HIGH,
	                       pdata->en_name);
	if (! err)
		return 0;

	if (gpio_is_valid(pdata->rst_gpio)) {
		gpio_set_value_cansleep(pdata->rst_gpio, 0);
		gpio_free(pdata->rst_gpio);
	}

	return err;
}

static int p7_notify_pwmbkl(struct device* dev, int brightness)
{
	struct p7_pwm_bkl const* const pdata = dev_get_platdata(dev);

	if (brightness <= pdata->data.lth_brightness)
		brightness = 0;

	gpio_set_value_cansleep(pdata->en_gpio, brightness);
	return brightness;
}

static void p7_exit_pwmbkl(struct device* dev)
{
	struct p7_pwm_bkl const* const pdata = dev_get_platdata(dev);

	/* Disable backlight. */
	gpio_set_value_cansleep(pdata->en_gpio, 0);
	gpio_free(pdata->en_gpio);

	if (gpio_is_valid(pdata->rst_gpio)) {
		gpio_set_value_cansleep(pdata->rst_gpio, 0);
		gpio_free(pdata->rst_gpio);
	}
}

static int p7_check_pwmblk_fb(struct device* dev, struct fb_info* fb)
{
	struct p7_pwm_bkl const* const pdata = dev_get_platdata(dev);

	/* Is this backlight assigned to framebuffer ? */
	if (pdata->fbdev)
		return pdata->fbdev == fb->device;

	return 0;
}

static struct p7_pwm_bkl p7_pwmbkl_pdata[] = {
	[0] = {
		.data = {
			.init           = p7_init_pwmbkl,
			.notify         = p7_notify_pwmbkl,
			.exit           = p7_exit_pwmbkl,
			.check_fb       = p7_check_pwmblk_fb
		}
	},
	[1] = {
		.data = {
			.init           = p7_init_pwmbkl,
			.notify         = p7_notify_pwmbkl,
			.exit           = p7_exit_pwmbkl,
			.check_fb       = p7_check_pwmblk_fb
		}
	}
};

static struct platform_device p7_pwmbkl_devs[] = {
	[0] = {
		.name			= "pwm-backlight",
		.id             = 0,
	},
	[1] = {
		.name			= "pwm-backlight",
		.id             = 1,
	}
};

void __init p7_init_bkl(unsigned int bkl,
                        struct device const* fbdev,
                        int pwm_id,
                        int rst_gpio,
                        unsigned int en_gpio,
                        unsigned int ns,
                        unsigned int max,
                        unsigned int min)
{
	int err;

#ifdef DEBUG
	BUG_ON(bkl >= ARRAY_SIZE(p7_pwmbkl_devs));
	BUG_ON(! ns);
	BUG_ON(! max);
	BUG_ON(max <= min);
#endif

	p7_pwmbkl_pdata[bkl].data.pwm_id = pwm_id;
	p7_pwmbkl_pdata[bkl].data.max_brightness = max;
	p7_pwmbkl_pdata[bkl].data.dft_brightness = max / 2;
	p7_pwmbkl_pdata[bkl].data.lth_brightness = min;
	p7_pwmbkl_pdata[bkl].data.pwm_period_ns = ns;
	p7_pwmbkl_pdata[bkl].en_gpio = en_gpio;
	p7_pwmbkl_pdata[bkl].rst_gpio = rst_gpio;
	p7_pwmbkl_pdata[bkl].fbdev = fbdev;

	err = p7_init_dev(&p7_pwmbkl_devs[bkl], &p7_pwmbkl_pdata[bkl].data, NULL, 0);
	WARN(err,
	     "p7: failed to init pwm backlight %d (%d)\n",
	     bkl,
		 err);
}
