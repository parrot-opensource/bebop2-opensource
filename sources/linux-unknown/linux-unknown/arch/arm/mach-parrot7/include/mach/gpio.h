/**
 * linux/arch/arm/mach-parrot7/include/mach/gpio.h - Parrot7 gpiolib platform
 *                                                   specific interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    06-Apr-2012
 *
 * This file is released under the GPL
 *
 * The whole purpose of this file is to provide gpiolib and Parrot7's common
 * platform code with GPIO space definitions, i.e., number of GPIO lines to
 * manage, and, for each GPIO controller present in the system, their respective
 * first GPIO within the global space (so-called "gpio base").
 * We want to achieve this in a flexible enough manner to prevent from changing
 * platform and / or driver code each time a new GPIO controller (or expander)
 * is added.
 */

#ifndef _ARCH_PARROT7_GPIO_H_
#define _ARCH_PARROT7_GPIO_H_

/*
 * Holds current number of GPIOs declared for the global space.
 * Don't use this outside of this file, this is for internal purpose only !
 */
#define __P7_FIRST_GPIO__   0

/*
 * Define gpio base for each controller.
 * Use enum definition to force preprocessor expansion each time
 * __P7_FIRST_GPIO__ is re-defined.
 */
enum {
/**********************************
 * Declare Parrot7 chip GPIO lines
 **********************************/
#if defined(CONFIG_GPIO_PARROT7) || defined(CONFIG_GPIO_PARROT7_MODULE)
	P7_FIRST_GPIO = __P7_FIRST_GPIO__,
#define P7_GPIOS_MAX    220
#undef __P7_FIRST_GPIO__
#define __P7_FIRST_GPIO__   P7_GPIOS_MAX

#define P7_GPIO_NR(_n) (P7_FIRST_GPIO + (_n))

#endif

/******************************************************************
 * Declare Parrot7 Power Management Unit companion chip GPIO lines
 ******************************************************************/
#if defined(CONFIG_GPIO_P7MU) || defined(CONFIG_GPIO_P7MU_MODULE)
	P7MU_FIRST_GPIO = __P7_FIRST_GPIO__,
#define P7MU_GPIOS_MAX  5
#undef __P7_FIRST_GPIO__
#define __P7_FIRST_GPIO__   (P7MU_FIRST_GPIO + P7MU_GPIOS_MAX)

/* P7MU IOs are starting with IO1, so we substract 1 to be 0 indexed */
#define P7MU_IO_NR(_n) (P7MU_FIRST_GPIO + (_n) - 1)

#endif

/***********************************************
 * Declare first FC7100 I/O expander GPIO lines
 ***********************************************/
#if (defined(CONFIG_GPIO_PCA953X) ||            \
     defined(CONFIG_GPIO_PCA953X_MODULE)) &&    \
    defined(CONFIG_MACH_PARROT_FC7100_FAMILY)
	FC7100_IOEXPAND0_FIRST_GPIO = __P7_FIRST_GPIO__,
#define FC7100_IOEXPAND0_GPIOS_MAX  16
#undef __P7_FIRST_GPIO__
#define __P7_FIRST_GPIO__   \
    (FC7100_IOEXPAND0_FIRST_GPIO + FC7100_IOEXPAND0_GPIOS_MAX)

#define FC7100_IOEXPAND0_GPIO_NR(_n) (FC7100_IOEXPAND0_FIRST_GPIO + (_n))
#endif

/************************************************
 * Declare second FC7100 I/O expander GPIO lines
 ************************************************/
#if (defined(CONFIG_GPIO_PCA953X) ||            \
     defined(CONFIG_GPIO_PCA953X_MODULE)) &&    \
    defined(CONFIG_MACH_PARROT_FC7100_FAMILY)
	FC7100_IOEXPAND1_FIRST_GPIO = __P7_FIRST_GPIO__,
#define FC7100_IOEXPAND1_GPIOS_MAX  16
#undef __P7_FIRST_GPIO__
#define __P7_FIRST_GPIO__   \
    (FC7100_IOEXPAND1_FIRST_GPIO + FC7100_IOEXPAND1_GPIOS_MAX)

#define FC7100_IOEXPAND1_GPIO_NR(_n) (FC7100_IOEXPAND1_FIRST_GPIO + (_n))
#endif

/****************************************************
 * Declare 24 bits FC7100DEV I/O expander GPIO lines
 ****************************************************/
#define FC7100DEV_IOEXPAND_GPIO_NR FC7100_IOEXPAND0_GPIO_NR
};

/* At last, set the total number of available GPIO lines in the global space.
   We add some headroom for potential "hotplug" GPIO chips (used on Sicilia to
   add the external GPIOs of the irradiance module on the Hook).
 */
#define ARCH_NR_GPIOS   (__P7_FIRST_GPIO__ + 50)

#include <asm-generic/gpio.h>

/*
 * We use the gpiolib framework. Those functions will call the corresponding
 * callbacks defined in the GPIO driver's struct gpio_chip. (see for instance
 * drivers/parrot/gpio/p7gpio.c).
 *
 * If it turns out that the gpiolib framework is too slow for a given product,
 * we could inline the code accessing the GPIO here and save a couple cycles
 * (for instance for heavy bitbangig).
 */
#define gpio_get_value  __gpio_get_value
#define gpio_set_value  __gpio_set_value
#define gpio_cansleep   __gpio_cansleep
#define gpio_to_irq     __gpio_to_irq

/* When use filtering and/or phase measure, use the following struct
 * @start and @stop are gpios
 * @filter if the counter ratio is set. This ratio devides irqs.
 *
 */

#define GPIO_NO_MEASURE			(-1)
#define GPIO_MEASURE_START		0
#define GPIO_MEASURE_STOP		1
#define GPIO_MEASURE_START_AFTER_STOP	2
#define GPIO_MEASURE_STOP_AFTER_START	3

struct p7gpio_filter_phase{
	u32		start;
	u32		stop;
	u8		filter;
	s8	 	mode;
	u8		export_reset;
};

/* custom fonction for mapping a gpio to an irq :
   should be called after the p7 gpio is registered, but before gpio_to_irq
 */
int p7_gpio_interrupt_register(unsigned gpio);
int p7_gpio_filter_interrupt_register(struct p7gpio_filter_phase);
#endif /* _ARCH_PARROT7_GPIO_H_ */
