/**
 * linux/drivers/parrot/mfd/p7mu.h - Parrot7 power management unit core driver
 *                                   interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    16-May-2012
 *
 * This file is released under the GPL
 */

#ifndef _P7MU_H
#define _P7MU_H

#if defined(CONFIG_MFD_P7MU) || \
    defined(CONFIG_MFD_P7MU_MODULE)

#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/i2c.h>
#include <mach/pwm.h>
#include <mach/p7-adc.h>

#define P7MU_DRV_NAME   "p7mu"

/* Number of controllable I/O pins */
#define P7MU_PIN_NR     5

#define P7MU_CFG_PAGE   ((u16) 0x0000)

extern struct i2c_client*   p7mu_i2c;

struct p7mu_plat_data {
	/* GPIO line used as PxMU interrupt source */
	int             gpio;
	unsigned int    pwm_pins[P7MU_PWMS_MAX];
	bool            int_32k;
	bool            int_32m;
	bool			overide_otp;
	int             gpio_event_sel;

        /* p7mu adc platform data */
         struct p7_adc_chan_data *chan_data;
};

#define P7MU_PMC_GPIO1_EDGE_RISING ((u16) (1U << 10)|(2U << 0))
#define P7MU_PMC_GPIO2_EDGE_RISING ((u16) (1U << 11)|(2U << 2))
#define P7MU_PMC_GPIO3_EDGE_RISING ((u16) (1U << 12)|(2U << 4))
#define P7MU_PMC_GPIO4_EDGE_RISING ((u16) (1U << 13)|(2U << 6))
#define P7MU_PMC_GPIO5_EDGE_RISING ((u16) (1U << 14)|(2U << 8))

static inline struct p7mu_plat_data const* p7mu_pdata(void)
{
	return dev_get_platdata(&p7mu_i2c->dev);
}

struct platform_device;

extern struct resource const*   p7mu_request_region(struct platform_device*,
													struct resource const* res);
extern void                     p7mu_release_region(struct resource const*);

int p7mu_gpio_to_irq(unsigned int gpio);

/////////////////

extern int p7mu_transfer(u16, u16*, size_t, bool);

static inline int p7mu_read16(u16 offset, u16* val)
{
	int const err = p7mu_transfer(cpu_to_be16(offset), (u16*)val, 1, true);

	if (err)
		return err;

	be16_to_cpus(val);
	return 0;
}

static inline int p7mu_write16(u16 offset, u16 val)
{
	val = cpu_to_be16(val);
	return p7mu_transfer(cpu_to_be16(offset), &val, 1, false);
}

extern int p7mu_mod16(u16 offset, u16 val, u16 mask);

static inline int p7mu_read32(u16 offset, u32* buff)
{
	int const err = p7mu_transfer(cpu_to_be16(offset), (u16*)buff, 2, true);

	if (err)
		return err;

	be32_to_cpus(buff);
	return 0;
}

static inline int p7mu_write32(u16 offset, u32 val)
{
	val = cpu_to_be32(val);
	return p7mu_transfer(cpu_to_be16(offset),(u16*) &val, 2, false);
}

int p7mu_save_cpu_jump(void *jump_addr);

#endif  /* defined(CONFIG_MFD_P7MU) || \
           defined(CONFIG_MFD_P7MU_MODULE) */

#endif
