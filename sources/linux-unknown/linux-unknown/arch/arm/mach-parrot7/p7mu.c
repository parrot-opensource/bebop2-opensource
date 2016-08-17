/**
 * linux/arch/arm/mach-parrot7/p7mu.c - Parrot7 power management unit platform
 *                                      implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    22-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mfd/p7mu.h>
#include "common.h"
#include "pinctrl.h"
#include "i2cm.h"

static struct i2c_board_info p7mu_dev __initdata = {
	I2C_BOARD_INFO(P7MU_DRV_NAME, 0x31),
};

/**
 * p7_init_p7mu() - Instantiate P7MU device on I2C bus identified by @bus
 *                  for further driver usage.
 *
 * @bus:        master controller / bus identifier
 * @pdata:      controller platform specific data
 * @pins:       array of pin functions and settings
 * @pin_cnt:    number of element of @pins array
 */
void __init p7_init_p7mu(int bus,
                         struct p7mu_plat_data* pdata,
                         struct pinctrl_map* pins,
                         size_t pin_cnt)
{
#ifdef DEBUG
	BUG_ON(! pdata);
	BUG_ON(! gpio_is_valid(pdata->gpio));
	BUG_ON(! pins);
	BUG_ON(! pin_cnt);
#endif

	p7mu_dev.platform_data = pdata;
	p7_init_i2cm_slave(bus, &p7mu_dev, pins, pin_cnt);
}
