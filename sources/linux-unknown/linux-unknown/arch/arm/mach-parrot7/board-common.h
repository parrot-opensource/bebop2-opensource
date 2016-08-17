/**
 * linux/arch/arm/mach-parrot7/board-common.c - Parrot7 based boards common
 *                                              interface
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    12-Apr-2013
 *
 * This file is released under the GPL
 */

#ifndef _ARCH_BOARD_COMMON_H
#define _ARCH_BOARD_COMMON_H

#include <linux/init.h>
#include "pinctrl.h"
#include "board-sysfs.h"
#include "i2cs.h"
#include <linux/i2c.h>
#include "mach/usb-p7.h"

extern int parrot_force_usb_device;

#if defined(CONFIG_I2CM_PARROT7) || \
    defined(CONFIG_I2CM_PARROT7_MODULE)

extern void p7brd_init_i2cm(int, unsigned int) __init;
#else

static inline void p7brd_init_i2cm(int, unsigned int)
{
	return;
}

#endif

#if defined(CONFIG_I2C_MUX_PARROT7) || \
    defined(CONFIG_I2C_MUX_PARROT7_MODULE)

struct p7i2cmux_plat_data;
struct p7i2cmux_pins;

extern void p7brd_init_i2cm_muxed(int,
                                  unsigned int,
                                  struct pinctrl_map* main_pins,
                                  size_t main_pin_cnt,
                                  struct p7i2cmux_plat_data*,
                                  struct p7i2cmux_pins const*) __init;
#else

struct p7i2cmux_plat_data;
struct p7i2cmux_pins;

static inline void p7brd_init_i2cm_muxed(int bus,
					 unsigned int hz,
					 struct pinctrl_map* main_pins,
					 size_t main_pin_cnt,
					 struct p7i2cmux_plat_data* mux_pdata,
					 struct p7i2cmux_pins const *mux_pins)
{
	return;
}

#endif

#define P7_I2C_NOIRQ     0
#define P7_I2C_IRQ       1
#define P7_I2C_SHAREDIRQ 2

int __init parrot_init_i2c_slave(int i2c_bus,
				 struct i2c_board_info *info,
				 const char *desc,
				 int irq_type);

#if defined(CONFIG_PLDS_I2CS) || \
    defined(CONFIG_PLDS_I2CS_MODULE)

extern void p7brd_init_i2cs(struct plds_i2cs_pdata *pdata) __init;

#else

static inline void p7brd_init_i2cs(struct plds_i2cs_pdata* pdata)
{
	return;
}

#endif

#if defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC)
extern void p7brd_init_usb(int, int, enum ci_udc_operating_modes mode) __init;
#else

static inline void p7brd_init_usb(int i, int j, enum ci_udc_operating_modes mode)
{
	return;
}

#endif  /* #if defined(CONFIG_USB_CHIPIDEA_HOST) ||       \
               defined(CONFIG_USB_CHIPIDEA_UDC) */
#if defined(CONFIG_USB_CHIPIDEA_HOST)
extern void p7brd_init_hcd(int, int) __init;
#else

static inline void p7brd_init_hcd(int i, int j)
{
	return;
}

#endif  /* #if defined(CONFIG_USB_CHIPIDEA_HOST) */

#if defined(CONFIG_USB_CHIPIDEA_UDC)
extern void p7brd_init_udc(int, int) __init;
#else

static inline void p7brd_init_udc(int i, int j)
{
}

#endif  /* #if defined(CONFIG_USB_CHIPIDEA_UDC) */

/**
 * struct acs3_regulator_gpios - Regulators GPIOs
 * @card_enable_gpio:       card power enable GPIO
 * @card_enable_active_low: card GPIO active high
 */

struct acs3_regulator_gpios {
	int card_enable_gpio;
	int card_enable_active_low;
	int card_enable_delay;
	int bus_1v8_gpio;
	int bus_1v8_active_low;
	int bus_switch_delay;
	int bus_switch_disabled;
	int bus_enable_gpio;
	int bus_enable_active_low;
	int bus_enable_delay;
};

struct acs3_plat_data;

#if defined(CONFIG_MMC_SDHCI_ACS3) || \
    defined(CONFIG_MMC_SDHCI_ACS3_MODULE)
extern void p7brd_init_sdhci(int, struct acs3_plat_data*,
				struct platform_device*,
				struct platform_device*,
				struct acs3_regulator_gpios*,
				struct pinctrl_map*,
				size_t) __init;
#else

static inline void p7brd_init_sdhci(int				 i,
				    struct acs3_plat_data	*p,
				    struct platform_device	*pd1,
				    struct platform_device	*pd2,
				    struct acs3_regulator_gpios *g,
				    struct pinctrl_map		*pm,
				    size_t			 c)
{
	return;
}


#endif  /* defined(CONFIG_MMC_SDHCI_ACS3) || \
           defined(CONFIG_MMC_SDHCI_ACS3_MODULE) */

#if defined(CONFIG_MTD_NAND_CAST) || \
    defined(CONFIG_MTD_NAND_CAST_MODULE)
extern void p7brd_init_nand(int synchrone_nand) __init;
#else

static inline void p7brd_init_nand(int synchrone_nand)
{
	return;
}

#endif  /* defined(CONFIG_MTD_NAND_CAST) || \
           defined(CONFIG_MTD_NAND_CAST_MODULE) */

#if defined(CONFIG_SERIAL_PARROTX) || \
    defined(CONFIG_SERIAL_PARROTX_MODULE)
extern void p7brd_init_uart(int uart, int rtscts) __init;
#else

static inline void p7brd_init_uart(int uart, int rtscts)
{
	return;
}

#endif

extern void p7brd_init_named_gpios(void) __init;
extern int p7brd_export_gpio(int, unsigned int, char const*);
extern void p7brd_unexport_gpio(int) __init;
int parrot_gpio_user_in_init(int gpio, int pincfg, const char *alias);
int parrot_gpio_in_init(int gpio, int pincfg, const char *name);

#endif

extern void __init p7brd_init_ccan(int bus) __init;
