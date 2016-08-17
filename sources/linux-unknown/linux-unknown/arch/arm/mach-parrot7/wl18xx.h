/**
 * linux/arch/arm/mach-parrot7/wl18xx.h - wl18xx platform interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Arnaud Boulan <arnaud.boulan@parrot.com>
 * date:    12-Dec-2013
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_PARROT7_WIFI_WL18XX_H
#define _ARCH_PARROT7_WIFI_WL18XX_H

struct wl18xx_resources {
	int wlirq_gpio;
	int wlen_gpio;
	int bt_rst_gpio;
	int wl_bt_ant_gpio;
	int wl5g_ant_gpio;
	int bt_uart_slot;
	int wl_sdhci_slot;
};

#ifdef CONFIG_WILINK_PLATFORM_DATA
void __init init_wl18xx(struct wl18xx_resources* res,
			struct pinctrl_map *pins,
			size_t pins_cnt);
#else
static inline void __init init_wl18xx(struct wl18xx_resources* res,
			struct pinctrl_map *pins,
			size_t pins_cnt) { }
#endif

#endif
