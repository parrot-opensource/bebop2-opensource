/**
 * linux/drivers/parrot/mfd/p7mu-clk.c - Parrot7 power management unit internal
 *                                       clocks implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    21-Nov-2012
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <mach/pwm.h>
#include "p7mu.h"
#include "p7mu-clk.h"

/*
 * Internal oscillator calibration registers (configuration block)
 */
#define P7MU_CFG_OSC_CAL_REG    (P7MU_CFG_PAGE + 0x0c)

#define P7MU_OSC_CAL_EN         ((u16) 1U << 0)
#define P7MU_OSC_CAL_SEL        ((u16) 1U << 1)
#define P7MU_OSC_SSM_ENABLE     ((u16) 1U << 2)
#define P7MU_OSC_CAL_OUT_SFT    9
#define P7MU_OSC_CAL_OUT_MSK    ((u16) 0x3fU << P7MU_OSC_CAL_OUT_SFT)
#define P7MU_OSC_CAL_RDY        ((u16) 1U << 15)

/*
 * Clock generator block register
 */
#define P7MU_CLK_PAGE       ((u16) 0x0c00)
#define P7MU_CLK_CFG_REG    (P7MU_CLK_PAGE + 0x0)

#define P7MU_CLK_EXT_EN         ((u16) (1U << 0))
#define P7MU_CLK_FAST_CLK4PWM_1 ((u16) (1U << 1))
#define P7MU_CLK_EXT_USED       ((u16) (1U << 2))

/*
 * Clock generator backup block register
 */
#define P7MU_CLKBKUP_PAGE       ((u16) 0x0d00)
#define P7MU_CLKBKUP_CFG_REG    (P7MU_CLKBKUP_PAGE + 0x0)

#define P7MU_CLKBKUP_EXT32K_EN          ((u16) (1U << 0))
#define P7MU_CLKBKUP_FAST_CLK4PWM_0     ((u16) (1U << 1))
#define P7MU_CLKBKUP_EXT32K_USED        ((u16) (1U << 2))
#define P7MU_CLKBKUP_LDO_OCD_MASK_PGO0  ((u16) (1U << 7))

/****************************************************************************
 * SPI clock
 ****************************************************************************/
#if defined(CONFIG_P7MU_ADC) || defined(CONFIG_P7MU_ADC_MODULE)
static unsigned long p7mu_spiclk_rate;
long p7mu_get_spiclk_rate(void)
{
	int     err;
	u16     reg, msk, val;

	if (p7mu_spiclk_rate)
		goto out;

	reg = P7MU_CLK_CFG_REG;
	msk = P7MU_CLK_EXT_USED; 
	err = p7mu_read16(reg, &val);
	if (err)
		return (long) err;

	if (val & msk)
		p7mu_spiclk_rate = P7MU_SPI_HZ_MAX;
	else
		p7mu_spiclk_rate = P7MU_SPI_HZ_MIN;

out:
	return (long) p7mu_spiclk_rate;
}
EXPORT_SYMBOL(p7mu_get_spiclk_rate);
#endif


/****************************************************************************
 * PWM clocks
 *
 * Function prototypes are kept compliant with common clock framework (if we
 * ever need to switch to it).
 ****************************************************************************/

#if defined(CONFIG_PWM_P7MU) || defined(CONFIG_PWM_P7MU_MODULE)

static unsigned long p7mu_pwmclk_rates[P7MU_PWMS_MAX];

long p7mu_get_pwmclk_rate(unsigned int pwm_id)
{
	int     err;
	u16     reg, msk, val;

#ifdef DEBUG
	BUG_ON(pwm_id >= P7MU_PWMS_MAX);
#endif

	if (p7mu_pwmclk_rates[pwm_id])
		goto out;

	if (! pwm_id) {
		reg = P7MU_CLKBKUP_CFG_REG;
		msk = P7MU_CLKBKUP_FAST_CLK4PWM_0;
	}
	else {
		reg = P7MU_CLK_CFG_REG;
		msk = P7MU_CLK_FAST_CLK4PWM_1;
	}

	err = p7mu_read16(reg, &val);
	if (err)
		return (long) err;

	if (val & msk)
		p7mu_pwmclk_rates[pwm_id] = P7MU_PWM_HZ_MAX;
	else
		p7mu_pwmclk_rates[pwm_id] = P7MU_PWM_HZ_MIN;

out:
	return (long) p7mu_pwmclk_rates[pwm_id];
}
EXPORT_SYMBOL(p7mu_get_pwmclk_rate);

int p7mu_set_pwmclk_rate(unsigned int pwm_id, unsigned long hz)
{
	int     err;
	u16     reg, msk, val;

#ifdef DEBUG
	BUG_ON(pwm_id >= P7MU_PWMS_MAX);
	BUG_ON(hz != P7MU_PWM_HZ_MIN && hz != P7MU_PWM_HZ_MAX);
#endif

	if (hz == p7mu_pwmclk_rates[pwm_id])
		return 0;

	if (! pwm_id) {
		reg = P7MU_CLKBKUP_CFG_REG;
		msk = P7MU_CLKBKUP_FAST_CLK4PWM_0;
	}
	else {
		reg = P7MU_CLK_CFG_REG;
		msk = P7MU_CLK_FAST_CLK4PWM_1;
	}

	if (hz == P7MU_PWM_HZ_MIN)
		val = 0;
	else
		val = msk;

	err = p7mu_mod16(reg, val, msk);
	if (err)
		return (long) err;

	p7mu_pwmclk_rates[pwm_id] = hz;
	return 0;
}
EXPORT_SYMBOL(p7mu_set_pwmclk_rate);

#endif  /* defined(CONFIG_PWM_P7MU) || defined(CONFIG_PWM_P7MU_MODULE) */

/********************************************
 * Clock tree initialization implementation.
 ********************************************/

#define P7MU_CALIB_JIFFIES  (HZ * 60 * 15)  /* 15 mn calibration period */

struct p7mu_clock_work {
	struct delayed_work wk;
	bool                int_32k;
	bool                int_32m;
};

static struct p7mu_clock_work   p7mu_clk_wk;

static void p7mu_start_32m_calib(struct work_struct* work);

static void p7mu_32m_calib_done(struct work_struct* work)
{
	u16 cfg;
	int err = p7mu_read16(P7MU_CFG_OSC_CAL_REG, &cfg);

	if (! err)
		err = (cfg & P7MU_OSC_CAL_RDY) ? 0 : -ETIME;

	if (err) {
		/* Cancel calibration and restore default coefficients. */
		p7mu_write16(P7MU_CFG_OSC_CAL_REG, P7MU_OSC_CAL_SEL);

		dev_warn(&p7mu_i2c->dev,
		         "internal 32mHz calibration failed, "
		         "retrying in %d mn...\n",
		         P7MU_CALIB_JIFFIES / (60 * HZ));
	}
	else
		dev_dbg(&p7mu_i2c->dev,
		        "internal 32mHz oscillator calibration completed "
		        "(coeff = %02x).\n",
		        (cfg & P7MU_OSC_CAL_OUT_MSK) >> P7MU_OSC_CAL_OUT_SFT);

	/* Next run will start a new calibration process. */
	PREPARE_DELAYED_WORK((struct delayed_work*) &p7mu_clk_wk,
	                     &p7mu_start_32m_calib);
	schedule_delayed_work((struct delayed_work*) &p7mu_clk_wk,
	                      P7MU_CALIB_JIFFIES);
}

static void p7mu_start_32m_calib(struct work_struct* work)
{
	int const err = p7mu_write16(P7MU_CFG_OSC_CAL_REG, P7MU_OSC_CAL_EN);

	dev_dbg(&p7mu_i2c->dev,
	        "starting internal 32mHz oscillator calibration...\n");

	if (err) {
		dev_warn(&p7mu_i2c->dev,
		         "failed to start internal 32mHz calibration, "
		         "retrying in %d mn...\n",
		         P7MU_CALIB_JIFFIES / (60 * HZ));
		schedule_delayed_work((struct delayed_work*) &p7mu_clk_wk,
		                      P7MU_CALIB_JIFFIES);
		return;
	}

	/*
	 * Calibration process is about 215 usecs long: schedule check at
	 * next jiffie to allow other queued works to run.
	 */
	PREPARE_DELAYED_WORK((struct delayed_work*) &p7mu_clk_wk,
	                     &p7mu_32m_calib_done);
	schedule_delayed_work((struct delayed_work*) &p7mu_clk_wk, 1);
}

static void __devinit p7mu_x32k_switch_done(struct work_struct* work)
{
	/*
	 * For whatever reason, we need to perform 3 retries most of the time to
	 * succeed in switching to external xtal.
	 * 
	 */
	u16 cfg_32k;
	static int  retries = 5;
	int         err = p7mu_read16(P7MU_CLKBKUP_CFG_REG, &cfg_32k);

	if (err) {
		/*
		 * Internal clock still in use: failure.
		 * Revert back to internal.
		 */
		cfg_32k &= ~(P7MU_CLKBKUP_EXT32K_EN |
		                         P7MU_CLKBKUP_EXT32K_USED);
		p7mu_write16(P7MU_CLKBKUP_CFG_REG, cfg_32k);

		goto err;
	}

	if (cfg_32k & P7MU_CLKBKUP_EXT32K_USED) {
		/* External clock ready to be used. */
		if (! p7mu_clk_wk.int_32k && p7mu_clk_wk.int_32m)
			p7mu_start_32m_calib(NULL);

		return;
	}

	if (retries) {
		/* Retry in one more second. */
		retries--;
		schedule_delayed_work((struct delayed_work*) &p7mu_clk_wk, HZ);
		return;
	}
	else
		err = -EIO;

err:
	dev_warn(&p7mu_i2c->dev,
	         "failed to switch to external 32kHz xtal (%d)\n",
	         err);
	p7mu_clk_wk.int_32k = true;
}

void p7mu_init_clk(void)
{
	struct p7mu_plat_data const* const  pdata = dev_get_platdata(&p7mu_i2c->dev);
	int                                 err;
	u16 cfg_32k;
	u16 cfg_32m;

	p7mu_clk_wk.int_32k = pdata->int_32k;
	p7mu_clk_wk.int_32m = pdata->int_32m;

	/*** start 32mHz / 48 mHz oscillator cfg ***/
	err = p7mu_read16(P7MU_CLK_CFG_REG, &cfg_32m);
	if (err) {
		dev_warn(&p7mu_i2c->dev,
		         "failed to setup 32mHz / 48 mHz oscillator (%d)\n",
		         err);
		return;
	}

	err = p7mu_read16(P7MU_CLKBKUP_CFG_REG, &cfg_32k);
	if (err) {
		dev_warn(&p7mu_i2c->dev,
		         "failed to setup 32kHz xtal (%d)\n",
		         err);
		return;
	}

	if (!pdata->overide_otp) {
		/* we should trust otp by default. only check if config
		 look correct */
		if (p7mu_clk_wk.int_32m != !(cfg_32m & P7MU_CLK_EXT_USED))
			dev_err(&p7mu_i2c->dev, "32 Mhz clock do not match wanted %s got %s\n",
					p7mu_clk_wk.int_32m?"int":"ext", cfg_32m & P7MU_CLK_EXT_USED ? "ext":"int");

		if (p7mu_clk_wk.int_32k != !(cfg_32k & P7MU_CLK_EXT_USED))
			dev_warn(&p7mu_i2c->dev, "32 Khz clock do not match wanted %s got %s\n",
					p7mu_clk_wk.int_32k?"int":"ext", cfg_32k & P7MU_CLK_EXT_USED ? "ext":"int");

		return;
	}

	if (! p7mu_clk_wk.int_32m &&
	    ! (cfg_32m & P7MU_CLK_EXT_USED)) {
		/* pdata want external but it is not enabled */
		dev_dbg(&p7mu_i2c->dev, "switching to external 48mHz oscillator...\n");

		p7mu_write16(P7MU_CLK_CFG_REG,
		             cfg_32m | P7MU_CLK_EXT_EN);

		err = p7mu_read16(P7MU_CLK_CFG_REG, &cfg_32m);
		if (! err && ! (cfg_32m & P7MU_CLK_EXT_USED)) {
			/*
			 * Internal clock still in use: failure.
			 * Revert back to internal.
			 */
			cfg_32m &= ~(P7MU_CLK_EXT_EN | P7MU_CLK_EXT_USED);
			p7mu_write16(P7MU_CLK_CFG_REG, cfg_32m);
			err = -EIO;
		}

		if (err) {
			dev_warn(&p7mu_i2c->dev,
			         "failed to switch to external 48mHz oscillator (%d)\n",
			         err);

			p7mu_clk_wk.int_32m = true;
		}
	}

	if (p7mu_clk_wk.int_32m && (cfg_32m & P7MU_CLK_EXT_USED)) {
		/* pdata want internal but it is external enabled */
		dev_dbg(&p7mu_i2c->dev, "switching to internal 32mHz oscillator...\n");

		err = p7mu_write16(P7MU_CLK_CFG_REG,
		                   cfg_32m & ~P7MU_CLK_EXT_EN);
		if (err) {
			dev_warn(&p7mu_i2c->dev,
			         "failed to switch to internal 32mHz oscillator (%d)\n",
			         err);

			p7mu_clk_wk.int_32m = false;
		}
	}

	/*** Setup 32kHz xtal. ***/

	INIT_DELAYED_WORK_DEFERRABLE(&p7mu_clk_wk.wk, NULL);

	if (! p7mu_clk_wk.int_32k &&
	    ! (cfg_32k & P7MU_CLKBKUP_EXT32K_USED)) {
		/* pdata want external but it is not enabled */
		dev_dbg(&p7mu_i2c->dev, "switching to external 32kHz xtal...\n");

		err = p7mu_write16(P7MU_CLKBKUP_CFG_REG,
		                   cfg_32k | P7MU_CLKBKUP_EXT32K_EN);
		if (! err) {
			/* Switching to external clock may long up to 1 second. */
			PREPARE_DELAYED_WORK((struct delayed_work*) &p7mu_clk_wk,
			                     &p7mu_x32k_switch_done);
			schedule_delayed_work((struct delayed_work*) &p7mu_clk_wk, HZ + 1);

			return;
		}

		dev_warn(&p7mu_i2c->dev,
		         "failed to switch to external 32kHz xtal (%d)\n",
		         err);

		p7mu_clk_wk.int_32k = true;
	}

	if (p7mu_clk_wk.int_32k &&
	    (cfg_32k & P7MU_CLKBKUP_EXT32K_USED)) {
		/* pdata want internal but it is external enabled */
		dev_dbg(&p7mu_i2c->dev, "switching to internal 32kHz xtal...\n");

		err = p7mu_write16(P7MU_CLKBKUP_CFG_REG,
		                     cfg_32k & ~P7MU_CLKBKUP_EXT32K_EN);
		if (err)
			dev_warn(&p7mu_i2c->dev,
			         "failed to switch to internal 32kHz xtal (%d)\n",
			         err);

		p7mu_clk_wk.int_32k = false;
	}

	/*
	 * When external 32kHz xtal is used, it serves as a reference clock for
	 * internal 32mHz oscillator and needs periodic calibration.
	 */
	if (! p7mu_clk_wk.int_32k && p7mu_clk_wk.int_32m)
		p7mu_start_32m_calib(NULL);
}

void p7mu_exit_clk(void)
{
	cancel_delayed_work_sync((struct delayed_work*) &p7mu_clk_wk);
}

void p7mu_resume_clk(void)
{
	struct p7mu_plat_data const* const  pdata = dev_get_platdata(&p7mu_i2c->dev);
	if (!pdata->overide_otp) {
		u16 otp;
		/* restore 32Mhz */
		p7mu_read16(0xe19, &otp);
		p7mu_write16(P7MU_CLK_CFG_REG,
				otp);
		return;
	}
	p7mu_init_clk();
}
