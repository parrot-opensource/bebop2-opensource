/**
 * linux/drivers/parrot/pinctrl/p7-pinctrl.h - Parrot7 pin controller driver
 *                                             interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    04-Jun-2012
 *
 * This file is released under the GPLv2
 */

#ifndef _P7_PINCTRL_H
#define _P7_PINCTRL_H

#ifdef CONFIG_PINCTRL_PARROT7

#include <linux/stringify.h>
#include <linux/pinctrl/pinctrl.h>

#define P7CTL_DRV_NAME "p7-pinctrl"

/**
 * union p7ctl_setting - Parrot7 pin controller pin setting
 *
 * @func:       multiplexing FUNCtion
 * @set_smt:    this setting contains a SchMitt Trigger configuration
 * @set_pud:    this setting contains a Pull Up / Down configuration
 * @set_slr:    this setting contains a SLew Rate configuration
 * @set_drv:    this setting contains a DRiVe strength configuration
 * @smt:        SchMitt Trigger configuration
 * @pud:        Pull Up / Down configuration
 * @ie:         Input Enable. If 1 the input takes the pad value, otherwise 0.
 * @slr:        SLew Rate configuration
 * @drv:        DRiVe strength configuration
 *
 * Holds physical pin configuration. The union is designed in a such way that
 * configuration values are directly mappable to hardware registers of pin
 * controller.
 *
 * The set_* members are mapped to unused bits in the register. We use them to
 * tell the pinctrl interface what we want to modify.
 */
union p7ctl_setting {
    u32 word;
    struct {
        unsigned    func:2;
        unsigned    :2;
        unsigned    set_smt:1;
        unsigned    set_pud:1;
        unsigned    set_slr:1;
        unsigned    set_drv:1;
        unsigned    smt:1;
        unsigned    pud:3;
        unsigned    ie:1;
        unsigned    :3;
        unsigned    slr:2;
        unsigned    drv:6;
    } fields;
};

/*
 * Pad configuration bit field shifts
 */
#define P7CTL_FUNC_SFT      0   /* IO Mux control */
#define P7CTL_SMT_SFT       8   /* schmitt trigger enable */
#define P7CTL_PUD_SFT       9   /* Pull Up enable */
#define P7CTL_SLR_SFT       16  /* Slew Rate configuration */
#define P7CTL_DRV_SFT       18  /* Drive Strength */

#define P7CTL_SET_SMT_SFT   4   /* Enable function setting */
#define P7CTL_SET_PUD_SFT   5   /* Enable pull up / down setting */
#define P7CTL_SET_SLR_SFT   6   /* Enable slew rate setting */
#define P7CTL_SET_DRV_SFT   7   /* Enable drive strength setting */

/***************************************************
 * Helper macros for generating a pad configuration
 ***************************************************/

/* Enable/disable Schmitt trigger */
#define P7CTL_SMT_ON    (1U)
#define P7CTL_SMT_OFF   (0U)

/**
 * P7CTL_SMT_CFG() - Define a Parrot7 pin Schmitt trigger setting
 *
 * @_on: %OFF, %ON
 */
#define P7CTL_SMT_CFG(_on)                  \
    ((P7CTL_SMT_ ## _on << P7CTL_SMT_SFT) | \
     (1U << P7CTL_SET_SMT_SFT))

/*
 * Pull up / pull down /
 * bus keeper / high impedence settings
 */
#define P7CTL_PUD_HIGHZ (0U)    /* High impedence */
#define P7CTL_PUD_UP    (1U)    /* Pull up */
#define P7CTL_PUD_DOWN  (2U)    /* Pull down */
#define P7CTL_PUD_KEEP  (4U)    /* Bus keeper */

/**
 * P7CTL_PUD_CFG() - Define a Parrot7 pin Pull UP / Down setting
 *
 * @_pud: %HIGHZ, %UP, %DOWN, %KEEP
 */
#define P7CTL_PUD_CFG(_pud)   \
    ((P7CTL_PUD_ ## _pud << P7CTL_PUD_SFT) | \
     (1U << P7CTL_SET_PUD_SFT))

/* Slew rate settings */
#define P7CTL_SLR_0 (0U)    /* Slowest */
#define P7CTL_SLR_1 (1U)
#define P7CTL_SLR_2 (2U)
#define P7CTL_SLR_3 (3U)    /* Fastest */

/**
 * P7CTL_SLR_CFG() - Define a Parrot7 pin slew rate setting
 *
 * @_rate: %0, %1, %2, %3
 */
#define P7CTL_SLR_CFG(_slr) \
    ((P7CTL_SLR_ ## _slr << P7CTL_SLR_SFT) | \
     (1U << P7CTL_SET_SLR_SFT))

#define P7CTL_DRV_TRI   (0U)    /* Tri-state */
#define P7CTL_DRV_0     (1U)    /* Weakest */
#define P7CTL_DRV_1     (3U)
#define P7CTL_DRV_2     (7U)
#define P7CTL_DRV_3     (0xfU)
#define P7CTL_DRV_4     (0x1fU)
#define P7CTL_DRV_5     (0x3fU) /* Strongest */

/**
 * P7CTL_DRV_CFG() - Define a Parrot7 pin slew rate setting
 *
 * @_strength: %TRI, %0, %1, %2, %3, %4, %5
 */
#define P7CTL_DRV_CFG(_drv) \
    ((P7CTL_DRV_ ## _drv << P7CTL_DRV_SFT) | \
     (1 << P7CTL_SET_DRV_SFT))

/**
 * P7CTL_INIT_PIN() - Define a Parrot7 physical pin
 *
 * @_pid:   pin identifier (same as GPIO number)
 * @_name:  pin name
 */
#define P7CTL_INIT_PIN(_pid, _name) PINCTRL_PIN(_pid, _name)

/**
 * struct p7ctl_function - Parrot7 pin multiplexing function
 *
 * @name:   function name
 * @pin_id: physical pin identifier (same as GPIO number)
 * @mux_id: hardware multiplexing function assigned to pin
 */
struct p7ctl_function {
    char const*         name;
    unsigned short      pin_id;
    unsigned short      mux_id;
};

/**
 * P7CTL_INIT_FUNC() - Define a Parrot7 multiplexed pin function
 *
 * @_name:  function name
 * @pin_id: physical pin identifier (same as GPIO number)
 * @mux_id: hardware multiplexing function to assign to physical pin
 */
#define P7CTL_INIT_FUNC(_func_id, _pin_id, _mux_id) \
    {                                               \
        .name       = __stringify(_func_id),        \
        .pin_id     = _pin_id,                      \
        .mux_id     = _mux_id                       \
    }

/**
 * struct p7ctl_pdata - Parrot7 pins controller platform specific data
 *
 * @funcs:      table of multiplexing functions
 * @funcs_nr:   number of multiplexing functions
 * @pins:       table of hardware pins
 * @pins_nr:    number of hardware pins
 * @gpios:      table of hardware pins usable as GPIOs
 * @gpios_nr:   number of entries of @gpios array
 * @gpio_drv:   GPIO driver name
 *
 * Holds platform specific pinctrl descriptors to allow Parrot7 chip
 * variants to customize their own I/O pins space.
 */
struct p7ctl_plat_data {
	struct p7ctl_function const*	funcs;
	size_t				funcs_nr;
	struct pinctrl_pin_desc const*	pins;
	size_t				pins_nr;
	unsigned long const*		gpios;
	size_t				gpios_nr;
	char const*			gpio_drv;
};

#endif  /* CONFIG_PINCTRL_PARROT7 */

#endif  /* _P7_PINCTRL_H */
