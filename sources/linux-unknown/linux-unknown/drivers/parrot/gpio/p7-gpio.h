/**
 * linux/drivers/parrot/gpio/p7-gpio.h - Parrot7 GPIO pin controller driver
 *                                       interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    06-Apr-2012
 *
 * This file is released under the GPL
 */

#ifndef _P7_GPIO_H_
#define _P7_GPIO_H_

#define P7GPIO_DRV_NAME "p7-gpio"

#if defined(CONFIG_GPIO_PARROT7) || defined(CONFIG_GPIO_PARROT7_MODULE)

/**
 * struct p7gpio_irq_map - Parrot7 GPIO controller platform specific data
 *
 * @irq_gpios:    array of GPIOs to be used as IRQs.
 * @irq_gpios_sz: number of entries in array irq_gpios
 *
 * Note: @irq_map_sz is limited to %P7_GPIO_IRQS
 */
struct p7gpio_plat_data {
	const unsigned	*irq_gpios;
	unsigned	    irq_gpios_sz;
	u16             gpio_nr;
};

#endif  /* defined(CONFIG_GPIO_PARROT7) ||
           defined(CONFIG_GPIO_PARROT7_MODULE) */

#endif /* _P7_GPIO_H_ */
