/**
 * linux/arch/arm/mach-parrot7/p7pwm.c - Parrot7 PWM platform implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author: Victor Lambret <victor.lambret.ext@parrot.com>
 * date:    29-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/platform_device.h>
#include <linux/clkdev.h>
#include <pwm/p7_pwm.h>
#include <mach/pwm.h>
#include "p7_pwm.h"
#include <mach/p7.h>
#include "common.h"
#include "clock.h"
#include "pinctrl.h"

static struct resource p7pwm_res[] = {
	[0] = {
		.start = P7_PWM,
		.end   = P7_PWM + SZ_64K - 1,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device p7_p7pwm_dev = {
	.name           = P7PWM_DRV_NAME,
	.id             = 0,
	.resource       = p7pwm_res,
	.num_resources  = ARRAY_SIZE(p7pwm_res)
};

/*This part deduce the used_pwms from pinmaps*/

struct p7pwm_pinpwm {
	uint16_t    pin;
	uint16_t    mask_pwm;
};


const static struct p7pwm_pinpwm p7pwm_numpins[] __initconst = {
	{.pin=P7_PWM_00, .mask_pwm=(1 << 0)},
	{.pin=P7_PWM_01, .mask_pwm=(1 << 1)},
	{.pin=P7_PWM_02, .mask_pwm=(1 << 2)},
	{.pin=P7_PWM_03, .mask_pwm=(1 << 3)},
	{.pin=P7_PWM_04, .mask_pwm=(1 << 4)},
	{.pin=P7_PWM_05, .mask_pwm=(1 << 5)},
	{.pin=P7_PWM_06, .mask_pwm=(1 << 6)},
	{.pin=P7_PWM_07, .mask_pwm=(1 << 7)},
	{.pin=P7_PWM_08, .mask_pwm=(1 << 8)},
	{.pin=P7_PWM_09, .mask_pwm=(1 << 9)},
	{.pin=P7_PWM_10, .mask_pwm=(1 << 10)},
	{.pin=P7_PWM_11, .mask_pwm=(1 << 11)},
	{.pin=P7_PWM_12, .mask_pwm=(1 << 12)},
	{.pin=P7_PWM_13, .mask_pwm=(1 << 13)},
	{.pin=P7_PWM_14, .mask_pwm=(1 << 14)},
	{.pin=P7_PWM_15, .mask_pwm=(1 << 15)},
	{.pin=P7_PWM_15a,.mask_pwm=(1 << 15)},
};

/*PWM default Configuration : No precision control, normal mode*/
static struct p7pwm_conf p7pwm_default_conf = {
	.period_precision = 100,
	.duty_precision = 100,
	.mode = P7PWM_MODE_NORMAL
};

/**
 * p7_init_p7pwm() - Instantiate P7 PWM dev for further driver usage.
 *
 * @pdata:  platform data
 * @pins:   array of pinmaps, each pinmap contain function & props for one PWM
 * @cnt:    number of PWM described  in pins
 */
void __init p7_init_p7pwm(struct p7pwm_pdata *pdata,
                         struct pinctrl_map* pins,
                         size_t pin_cnt)
{
	int err;
	unsigned int used = 0;
	int i,j;

#ifdef DEBUG
	BUG_ON(! pins);
	BUG_ON(! pdata);
#endif

	/*Compute dynamically the used pwms*/
	for (i=0; i< pin_cnt; i++)
		for (j=0; j < ARRAY_SIZE(p7pwm_numpins); j++)
			if ((unsigned int)(pins[i].data.mux.function) == p7pwm_numpins[j].pin)
				used |= p7pwm_numpins[j].mask_pwm;
	pdata->used = used;

	/*Replace NULL conf pointers by default conf*/
	for (i=0 ; i<P7PWM_NUMBER; i++)
		if (NULL == pdata->conf[i])
			pdata->conf[i] = &p7pwm_default_conf;

	/* Assign pins before registering device in cases driver is already
	 * loaded. */
	if (pin_cnt) {
		char buf[10];
		snprintf(buf, 10, "p7_pwm.%d", p7_p7pwm_dev.id);
		err = p7_assign_pins(buf, pins, pin_cnt);
		if (err)
			goto err;
	}

	/* Create platform device */
	err = p7_init_dev(&p7_p7pwm_dev, pdata, NULL, 0);
	if (err)
		panic("p7: failed to init P7 PWM %d (%d)\n", 0, err);

	return;

	/*
	 * Pinctrl does not provide us with a way to remove registered pin
	 * mappings...
	 */
err:
	panic("p7: failed to init PWM master controller %d (%d)\n",
	      p7_p7pwm_dev.id, err);
}
