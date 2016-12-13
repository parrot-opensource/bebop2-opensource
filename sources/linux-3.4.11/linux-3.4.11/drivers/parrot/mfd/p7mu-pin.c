/**
 * linux/drivers/parrot/mfd/p7mu-pin.c - Parrot7 power management unit I/O pins
 *                                       internal implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    18-Sep-2012
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <asm/errno.h>
#include "p7mu-pin.h"

/*
 * Multiplexing function register configuration values
 */
static u16 const p7mu_mux_cfg[] = {
	[P7MU_INGPIO_MUX]   = 0,
	[P7MU_PWM_MUX]      = 2,
	[P7MU_OUTGPIO_MUX]  = 3
};

/*
 * Build  a bit mask of available multiplexing function for a pin.
 */
#define P7MU_MUX_MSK(_mux)  \
	((u16) (1U << P7MU_ ## _mux ## _MUX))

/*
 * Available multiplexing functions per pin
 */
static u16 const p7mu_avail_mux[P7MU_PIN_NR] = {
	[0] = P7MU_MUX_MSK(INGPIO) | P7MU_MUX_MSK(OUTGPIO) |
	      P7MU_MUX_MSK(PWM),
	[1] = P7MU_MUX_MSK(INGPIO) | P7MU_MUX_MSK(OUTGPIO) |
	      P7MU_MUX_MSK(PWM),
	[2] = P7MU_MUX_MSK(INGPIO) | P7MU_MUX_MSK(OUTGPIO) |
	      P7MU_MUX_MSK(PWM),
	[3] = P7MU_MUX_MSK(INGPIO) | P7MU_MUX_MSK(OUTGPIO) |
	      P7MU_MUX_MSK(PWM),
	[4] = P7MU_MUX_MSK(INGPIO) | P7MU_MUX_MSK(OUTGPIO) |
	      P7MU_MUX_MSK(PWM)
};

/*
 * GPIO configuration register definitions.
 */
#define P7MU_PIN_SFT    2
#define P7MU_PIN_MSK    ((u16) ((1U << P7MU_PIN_SFT) - 1))

static struct p7mu_pin  p7mu_pins[P7MU_PIN_NR];
static DEFINE_MUTEX(    p7mu_pin_lck);

#ifdef DEBUG

#define CHAR_BIT    8

/*
 * Sanity checks:
 *  - there must exist one available multiplexing function mask per pin ;
 *  - we cannot handle more pins (configuration) than a full 16 bits register
 *    can hold (which is the width of P7MU_PIN_CFG register).
 */
#define P7MU_ASSERT_MUX(_mux)                                       \
	BUG_ON(_mux < 0);                                               \
	BUG_ON(_mux >= ARRAY_SIZE(p7mu_mux_cfg));                       \
	BUG_ON(ARRAY_SIZE(p7mu_avail_mux) != ARRAY_SIZE(p7mu_pins));    \
	BUG_ON(ARRAY_SIZE(p7mu_avail_mux) >                             \
	       ((sizeof(u16) * CHAR_BIT) / P7MU_PIN_SFT));              \
	BUG_ON(p7mu_mux_cfg[_mux] & ~P7MU_PIN_MSK)
#else   /* DEBUG */

#define P7MU_ASSERT_MUX(_mux)

#endif  /* DEBUG */

/*
 * Return computed register configuration word to assign "mux" multiplexing
 * function to "pin" pin.
 */
static int p7mu_pin_cfg(unsigned int pin, enum p7mu_mux mux)
{
	P7MU_ASSERT_MUX(mux);

	if (pin >= ARRAY_SIZE(p7mu_pins))
		return -EINVAL;

	if (! ((u16) (1U << mux) & p7mu_avail_mux[pin]))
		return -EPERM;

	return (int) (p7mu_mux_cfg[mux] << (pin * P7MU_PIN_SFT));
}

/*
 * Reserve "pin" pin for usage with multiplexing function "mux".
 */
int p7mu_request_pin(unsigned int pin, enum p7mu_mux mux)
{
	struct p7mu_pin* const  p = &p7mu_pins[pin];
	int                     cfg;
	int                     err = 0;

	cfg = p7mu_pin_cfg(pin, mux);
	if (cfg < 0)
		return cfg;

	mutex_lock(&p7mu_pin_lck);

	if (p->booked) {
		err = -EBUSY;
		goto unlock;
	}

	if (p->mux != mux)
		/* If I/O is not already setup as requested perform I2C access. */
		err = p7mu_mod16(P7MU_PIN_CFG,
		                 (u16) cfg,
		                 P7MU_PIN_MSK << (pin * P7MU_PIN_SFT));
	if (! err) {
		p->booked = true;
		p->mux = mux;
	}

unlock:
	mutex_unlock(&p7mu_pin_lck);
	return err;
}
EXPORT_SYMBOL(p7mu_request_pin);

/*
 * Release "pin" pin previously reserved by p7mu_request_pin.
 * This will reset pin to a side-effects free default state.
 */
void p7mu_free_pin(unsigned int pin)
{
	struct p7mu_pin* const  p = &p7mu_pins[pin];
	int                     cfg;
	int                     err = 0;

	cfg = p7mu_pin_cfg(pin, P7MU_INGPIO_MUX);
#ifdef DEBUG
	/*
	 * Input GPIO must always be supported since this is the default state
	 * mentionned above.
	 */
	BUG_ON(cfg < 0);
#endif

	mutex_lock(&p7mu_pin_lck);

	if (! p->booked) {
		err = -EBADFD;
		goto unlock;
	}

	if (p->mux != P7MU_INGPIO_MUX) {
		err = p7mu_mod16(P7MU_PIN_CFG,
		                 (u16) cfg,
		                 P7MU_PIN_MSK << (pin * P7MU_PIN_SFT));
	}

	if (! err) {
		p->booked = false;
		p->mux = P7MU_INGPIO_MUX;
	}

unlock:
	mutex_unlock(&p7mu_pin_lck);

	if (err)
		dev_warn(&p7mu_i2c->dev,
		         "failed to release pin %d (%d)\n",
		         pin,
		         err);
}
EXPORT_SYMBOL(p7mu_free_pin);

int p7mu_setup_pin(unsigned int pin, enum p7mu_mux mux)
{
	struct p7mu_pin* const  p = &p7mu_pins[pin];
	int                     cfg;
	int                     err = 0;

	cfg = p7mu_pin_cfg(pin, mux);
	if (cfg < 0)
		return cfg;

	mutex_lock(&p7mu_pin_lck);

	if (! p->booked) {
		err = -EBADFD;
		goto unlock;
	}

	if (p->mux != mux)
		/* If I/O is not already setup as requested perform I2C access. */
		err = p7mu_mod16(P7MU_PIN_CFG,
		                 (u16) cfg,
		                 P7MU_PIN_MSK << (pin * P7MU_PIN_SFT));
	if (! err)
		p->mux = mux;

unlock:
	mutex_unlock(&p7mu_pin_lck);
	return err;
}
EXPORT_SYMBOL(p7mu_setup_pin);

int p7mu_pin_mux(unsigned int pin)
{
	struct p7mu_pin const*  p;
	int                     err;

	if (pin >= ARRAY_SIZE(p7mu_pins))
		return -EINVAL;

	p = &p7mu_pins[pin];

	mutex_lock(&p7mu_pin_lck);

	if (! p->booked) {
		err = -EBADFD;
		goto unlock;
	}

	err = p->mux;
	P7MU_ASSERT_MUX(err);

unlock:
	mutex_unlock(&p7mu_pin_lck);
	return err;
}
EXPORT_SYMBOL(p7mu_pin_mux);
