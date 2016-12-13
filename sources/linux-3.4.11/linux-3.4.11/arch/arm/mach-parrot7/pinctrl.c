/**
 * linux/arch/arm/mach-parrot7/pinctrl.c - Parrot7 pin controller platform
 *                                         implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    06-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <gpio/p7-gpio.h>
#include "system.h"
#include "common.h"
#include "pinctrl.h"

static const struct p7_pinctrl_config *p7_pctl_chip_configs[] = {
	[P7_CHIPREV_R1] = &p7_pinctrl_config_r1,
	[P7_CHIPREV_R2] = &p7_pinctrl_config_r2,
	[P7_CHIPREV_R3] = &p7_pinctrl_config_r3,
};

static inline const struct p7_pinctrl_config *p7_pctl_get_chip_config(void)
{
	int rev = p7_chiprev();

	/* Check if we have a valid pin config for this chip rev */
	BUG_ON(rev >= ARRAY_SIZE(p7_pctl_chip_configs) ||
	       p7_pctl_chip_configs[rev] == NULL);

	return p7_pctl_chip_configs[rev];
}


/**
 * p7_config_pin() - Configure a pin.
 *
 * @pin:        pin index (ie gpio)
 * @config:       configuration to apply
 *
 * Returns: 0 - success, a negative errno like value if failure
 */
int p7_config_pin(int pin, int config)
{
	const struct p7_pinctrl_config	*cfg;
	const char *pin_name;

	cfg = p7_pctl_get_chip_config();
	if (pin < 0 || pin >= cfg->pin_descs_sz) {
		pr_err("p7: invalid pin number (%d)\n", pin);
		return -EINVAL;
	}

	pin_name = cfg->pin_descs[pin].name;
	pr_debug("p7: config pin %d (%s)\n", pin, pin_name);
	return pin_config_set(P7CTL_DRV_NAME ".0", pin_name, config);
}


/**
 * p7_assign_named_pins() - Assign a set of pins to device for further usage.
 *
 * @dev_name:   device name
 * @name:       optional pins set name
 * @pins:       array of pin functions and optional settings
 * @pin_cnt:    number of element of @pins array
 *
 * Returns: 0 - success, a negative errno like value if failure
 *
 * Register a set of multiplexed functions / pins to pinctrl core for device
 * usage. @pins array element is either:
 * - one multiplexed function / physical pin mapping initialized using
 *   P7_INIT_PINMAP(), or
 * - one physical pin set of properties (pull up / down, drive strength...)
 *   initialized using P7_INIT_PINCFG().
 *
 * Device drivers will further use registered pins by requesting them from the
 * pinctrl core.
 * Keep in mind that PIN_MAP_CONFIGS_GROUPS mappings are not handled
 * since Parrot7 has no notion of pin groups at hardware level.
 *
 * Note:
 * @pins may be located into initdata section (or on stack) as pinctrl core
 * (shallow) copies structures content at registering time. However, embedded
 * pointers are not copied, meaning all pin settings cannot live in kernel's
 * init sections.
 */
int __init p7_assign_named_pins(char const* dev_name,
                                char const* name,
                                struct pinctrl_map* pins,
                                size_t pin_cnt)
{
	const struct p7_pinctrl_config	*cfg;
	unsigned int			 m;
	int				 err;

	cfg = p7_pctl_get_chip_config();

#ifdef DEBUG
	pr_debug("p7: registering %s I/O pins...\n", dev_name);
	BUG_ON(! dev_name);
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif

	for (m = 0; m < pin_cnt; m++) {
		enum p7_pin_func func_id;

		switch (pins[m].type) {
		case PIN_MAP_TYPE_MUX_GROUP:
			func_id = (enum p7_pin_func) pins[m].data.mux.function;

			BUG_ON((unsigned int) func_id >= P7_N_FUNCTIONS);
			/* Make sure the function is available on this chip rev */
			BUG_ON(cfg->pin_funcs[func_id].name == NULL);

			pins[m].data.mux.function = cfg->pin_funcs[func_id].name;

			pr_debug("p7:\tinstall %s I/O pin function\n",
			         pins[m].data.mux.function);
			break;

		case PIN_MAP_TYPE_CONFIGS_PIN:
			func_id = (enum p7_pin_func) pins[m].data.configs.group_or_pin;

			BUG_ON((unsigned int) func_id >= P7_N_FUNCTIONS);
			/* Make sure the function is available on this chip rev */
			BUG_ON(cfg->pin_funcs[func_id].name == NULL);
			BUG_ON((unsigned int) cfg->pin_funcs[func_id].pin_id >=
			       cfg->pin_descs_sz);

			pins[m].data.configs.group_or_pin =
				cfg->pin_descs[cfg->pin_funcs[func_id].pin_id].name;

			pr_debug("p7:\tinstall %s I/O pin settings\n",
			         pins[m].data.configs.group_or_pin);
			break;

		default:
			BUG();
		}

		/* Assign pins to device. */
		pins[m].dev_name = dev_name;

		/* Do not override default name if none given in argument. */
		if (name)
			pins[m].name = name;
	}

	/* Register pins to pinctrl core. */
	err = pinctrl_register_mappings(pins, pin_cnt);
	if (err)
		pr_err("p7: failed to register %s I/O pins (%d)\n", dev_name, err);

	return err;
}

/*
 * Pin controller hardware registers space
 */
static struct resource p7_pctl_res = {
	.start = P7_SYS_PADCTRL_IO,
	.end   = P7_SYS_PADCTRL_IO + SZ_4K - 1,
	.flags = IORESOURCE_MEM,
};

/*
 * Pin controller platform specific data, allowing to manage Parrot7 variants
 * with different set of pins and functions. This one is meant for MPW1.
 */
static struct p7ctl_plat_data p7_pctl_pdata = {
	.gpio_drv   = P7GPIO_DRV_NAME
};

/*
 * Pin controller device
 */
static struct platform_device p7_pctl_dev = {
	.name               = P7CTL_DRV_NAME,
	.id                 = 0,
	.dev.platform_data  = &p7_pctl_pdata,
	.num_resources      = 1,
	.resource           = &p7_pctl_res
};

int __init p7_init_pctl(void)
{
	const struct p7_pinctrl_config	*cfg = p7_pctl_get_chip_config();

	p7_pctl_pdata.funcs    = cfg->pin_funcs;
	p7_pctl_pdata.funcs_nr = P7_N_FUNCTIONS;
	p7_pctl_pdata.pins     = cfg->pin_descs;
	p7_pctl_pdata.pins_nr  = cfg->pin_descs_sz;
	p7_pctl_pdata.gpios    = cfg->gpio_pins;
	p7_pctl_pdata.gpios_nr = cfg->gpio_pins_sz;

	return p7_init_dev(&p7_pctl_dev, NULL, NULL, 0);
}
