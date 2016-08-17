/**
 * linux/driver/parrot/mmc/switch_voltage_regulator.h - Voltage switch
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Jeremie Samuel <jeremie.samuel.ext@parrot.com>
 * date:    06-Jun-2013
 *
 * This file is released under the GPL
 */

#ifndef __SWITCH_VOLTAGE_H
#define __SWITCH_VOLTAGE_H

#include <linux/regulator/driver.h>

struct regulator_init_data;

enum regulator_type;

/**
 * struct switch_voltage_state - state description
 * @value:		microvolts or microamps
 * @gpios:		gpio target-state for the value
 *
 * This structure describes a supported setting of the regulator
 * and the necessary gpio-state to achieve it.
 */
struct switch_voltage_state {
	int value;
	int gpio;
};

/**
 * struct switch_voltage_config - config structure
 * @supply_name:	Name of the regulator supply
 * @enable_gpio:	GPIO to use for enable control
 *			set to -EINVAL if not used
 * @enable_high:	Polarity of enable GPIO
 *			1 = Active high, 0 = Active low
 * @enabled_at_boot:	Whether regulator has been enabled at
 *			boot or not. 1 = Yes, 0 = No
 *			This is used to keep the regulator at
 *			the default state
 * @startup_delay:	Start-up time in microseconds
 * @switch_gpio:	Switch GPIO
 * @switch_disabled:	Need to disable the regulator during
 *			the switch
 * @switch_delay:	Switch time in microseconds
 * @states:		Array of gpio_regulator_state entries describing
 *			the gpio state for specific voltages
 *			The first state is the default state.
 * @init_data:		regulator_init_data
 *
 * This structure contains gpio-voltage regulator configuration
 * information that must be passed by platform code to the
 * gpio-voltage regulator driver.
 */
struct switch_voltage_config {
	const char *supply_name;

	int enable_gpio;
	unsigned enable_high:1;
	unsigned enabled_at_boot:1;
	unsigned startup_delay;

	int switch_gpio;
	unsigned switch_disabled:1;
	unsigned switch_delay;
	struct switch_voltage_state *states;

	struct regulator_init_data *init_data;
};

#endif
