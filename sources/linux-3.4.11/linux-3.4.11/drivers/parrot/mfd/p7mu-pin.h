/**
 * linux/drivers/parrot/mfd/p7mu-pin.h.c - Parrot7 power management unit I/O pins
 *                                         internal interface
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    18-Sep-2012
 *
 * This file is released under the GPL
 */

#ifndef _P7MU_PIN_H
#define _P7MU_PIN_H

#include "p7mu.h"

/* I/O pins configuration register */
#define P7MU_PIN_CFG    (P7MU_CFG_PAGE + 0x0b)

/*
 * Multiplexing functions which may be assigned to I/O pins
 */
enum p7mu_mux {
	P7MU_INGPIO_MUX,    /* Input GPIO */
	P7MU_OUTGPIO_MUX,   /* Output GPIO */
	P7MU_PWM_MUX        /* PWM output */
};

/*
 * Pin descriptor
 */
struct p7mu_pin {
	/* Pin usage marker */
	bool            booked;
	/* Multiplexing function currently assigned to pin */
	enum p7mu_mux   mux;
};

extern int      p7mu_request_pin(unsigned int, enum p7mu_mux);
extern void     p7mu_free_pin(unsigned int);
extern int      p7mu_setup_pin(unsigned int, enum p7mu_mux);
extern int      p7mu_pin_mux(unsigned int);

#endif  /* _P7MU_PIN_H */
