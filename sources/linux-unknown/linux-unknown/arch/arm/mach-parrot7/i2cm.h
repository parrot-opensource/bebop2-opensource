/**
 * linux/arch/arm/mach-parrot7/i2cm.h - Parrot7 I2C master controller platform
 *                                      interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    12-Jun-2012
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_I2CM_H
#define _ARCH_PARROT7_I2CM_H

struct p7i2cm_plat_data;
struct i2c_board_info;
struct pinctrl_map;

#if defined(CONFIG_I2CM_PARROT7) || \
    defined(CONFIG_I2CM_PARROT7_MODULE)

#include <linux/init.h>

extern void p7_init_i2cm(int,
                         struct p7i2cm_plat_data*,
                         struct pinctrl_map*,
                         size_t) __init;


extern int p7_init_i2cm_slave(int,
                              struct i2c_board_info const*,
                              struct pinctrl_map*,
                              size_t) __init;

#else  /* defined(CONFIG_I2CM_PARROT7) || \
          defined(CONFIG_I2CM_PARROT7_MODULE) */

#define p7_init_i2cm(_bus, _pdata, _pins, _pins_nr)
#define p7_init_i2cm_slave(_bus, _info, _pins, _pins_nr) \
	({ -ENOSYS; })

#endif /* defined(CONFIG_I2CM_PARROT7) || \
          defined(CONFIG_I2CM_PARROT7_MODULE) */

#if defined(CONFIG_I2C_MUX_PARROT7) || \
    defined(CONFIG_I2C_MUX_PARROT7_MODULE)

struct p7i2cmux_plat_data;

struct p7i2cmux_pins {
	struct pinctrl_map* pinmap;
	size_t			    sz;
};

extern void p7_init_i2cm_muxed(int,
                               struct p7i2cm_plat_data*,
                               struct pinctrl_map*,
                               size_t,
                               struct p7i2cmux_plat_data*,
                               struct p7i2cmux_pins const*) __init;

#else

struct p7i2cmux_plat_data;

struct p7i2cmux_pins {
	struct pinctrl_map* pinmap;
	size_t			    sz;
};

#endif  /* #if defined(CONFIG_I2C_MUX_PARROT7) || \
               defined(CONFIG_I2C_MUX_PARROT7_MODULE) */

#endif
