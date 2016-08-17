/**
 * linux/drivers/parrot/pwm/p7mu-pwm.c - Parrot7 power management unit PWM
 *                                       implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    16-Nov-2012
 *
 * This file is released under the GPL
 *
 * TODO:
 *  - review
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/seq_file.h>
#include <linux/pwm.h>
#include <mfd/p7mu-pin.h>
#include <mfd/p7mu-clk.h>

#define P7MU_PWM_RATIO  ((u16) 0)
#define P7MU_PWM_DIV    ((u16) 1)
#define P7MU_PWM_CTRL   ((u16) 2)
#define P7MU_PWM_EN     ((u16) (1U << 0))
#define P7MU_PWM_LOAD   ((u16) (1U << 1))

#ifdef DEBUG
#define P7MU_ASSERT_PWM(_chip, _pwm)            \
    BUG_ON(! _chip);                            \
    BUG_ON(! _pwm);                             \
    BUG_ON(_chip != &p7mu_pwm_chip);            \
    BUG_ON(_chip != _pwm->chip);                \
    BUG_ON(_chip->npwm != P7MU_PWMS_MAX);       \
    BUG_ON(! _chip->pwms);                      \
    BUG_ON(_pwm->hwpwm >= chip->npwm);          \
	BUG_ON(! p7mu_pwm_data[_pwm->hwpwm].res);   \
    BUG_ON(IS_ERR(p7mu_pwm_data[_pwm->hwpwm].res))
#else
#define P7MU_ASSERT_PWM(_chip, _pwm)
#endif

struct p7mu_pwm {
	u16                     div;
	u16                     ratio;
	struct resource const*  res;
};

static struct pwm_chip  p7mu_pwm_chip;
static struct p7mu_pwm  p7mu_pwm_data[P7MU_PWMS_MAX];

/* Multiplex requested PWM block to pin setup by platform. */
static int p7mu_request_pwm(struct pwm_chip* chip, struct pwm_device* pwm)
{
	int err;

	P7MU_ASSERT_PWM(chip, pwm);

	/* Forward call to P7MU pin management layer. */
	err = p7mu_request_pin((unsigned int) p7mu_pdata()->pwm_pins[pwm->hwpwm],
	                       P7MU_PWM_MUX);
	if (! err) {
		dev_dbg(chip->dev, "requested %d[%d]\n", pwm->pwm, pwm->hwpwm);
		return 0;
	}

	dev_err(chip->dev,
			"failed to request %d[%d] (%d)\n",
			pwm->pwm,
			pwm->hwpwm,
			err);
	return err;
}

static void p7mu_free_pwm(struct pwm_chip* chip, struct pwm_device* pwm)
{
	struct p7mu_pwm* const  data = &p7mu_pwm_data[pwm->hwpwm];
	int                     err;

	P7MU_ASSERT_PWM(chip, pwm);

	err = p7mu_write16(data->res->start + P7MU_PWM_RATIO, 0);
	if (err) {
		dev_dbg(chip->dev, "failed to clear ratio (%d)\n", err);
		goto err;
	}

	err = p7mu_mod16(data->res->start + P7MU_PWM_CTRL,
	                 P7MU_PWM_LOAD,
	                 P7MU_PWM_LOAD);
	if (err) {
		dev_dbg(chip->dev, "failed to reload ratio (%d)\n", err);
		goto err;
	}
	else
		data->ratio = 0;

	err = p7mu_write16(data->res->start + P7MU_PWM_CTRL, 0);
	if (err) {
		dev_dbg(chip->dev, "failed to stop (%d)\n", err);
		goto err;
	}

	/* Forward call to P7MU pin management layer. */
	p7mu_free_pin((unsigned int) p7mu_pdata()->pwm_pins[pwm->hwpwm]);
	dev_dbg(chip->dev, "released %d[%d]\n", pwm->pwm, pwm->hwpwm);

err:
	dev_err(chip->dev,
			"failed to release %d[%d] (%d)\n",
			pwm->pwm,
			pwm->hwpwm,
			err);
}

static unsigned long p7mu_pwm_period(uint16_t div, unsigned long hz)
{
	return ((256 * ((unsigned long) div + 1)) *
	        ((unsigned long) USEC_PER_SEC / 100UL)) /
	       (hz / 100UL);
}

static unsigned long p7mu_pwm_duty(unsigned long period, uint16_t ratio)
{
	return (period * (unsigned long) ratio) / 256;
}

/*
 * PWM layer guarantees duty_ns <= period_ns & period_ns > 0.
 * Compute using usecs to avoid overflows. Periods range:
 *   128000 nsec -> 0xffffffff nsec (~ 4.3 sec)
 *     7128 Hz   ->     ~ 0.23 Hz
 */
static int p7mu_config_pwm(struct pwm_chip* chip,
                           struct pwm_device* pwm,
                           int duty_ns,
                           int period_ns)
{
	struct p7mu_pwm* const  data = &p7mu_pwm_data[pwm->hwpwm];
	unsigned long const     us = (unsigned long) period_ns / NSEC_PER_USEC;
	uint16_t                div, ratio;
	unsigned long           phz;
	int                     err;

	P7MU_ASSERT_PWM(chip, pwm);

	if (! us) {
		dev_dbg(chip->dev, "invalid period requested\n");
		err = -EINVAL;
		goto err;
	}

	/*
	 * Get parent clock frequency in Hz. It must be (at least) 256 times the target
	 * frequency since this is the minimum divisor value.
	 */
	phz = p7mu_round_pwmclk_rate((256UL * (unsigned long) USEC_PER_SEC) / us);
	if ((int) phz < 0) {
		dev_dbg(chip->dev, "out of range period requested\n");
		err = -ERANGE;
		goto err;
	}

	/*
	 * Compute in msecs since this migth otherwise overflow numerator. This
	 * implies we may very rarely compute a frequency a bit higher than the
	 * requested one because numerator is rounded down when dividing (in
	 * addition to loosing precision).
	 * Consider this as acceptable for now...
	 */
	div = (uint16_t) DIV_ROUND_UP((us / 10UL) * (phz / 100UL),
	                              256UL * (unsigned long) MSEC_PER_SEC) - 1;
	ratio = (uint16_t) DIV_ROUND_CLOSEST((unsigned long) duty_ns,
	                                     (unsigned long) period_ns / 256UL);
#ifdef DEBUG
	BUG_ON(ratio & ~((u16) 0x1ff));
#endif

	if (div != data->div || ratio != data->ratio) {
		/* Avoid useless I2C accesses. */
		err = p7mu_write16(data->res->start + P7MU_PWM_RATIO, ratio);
		if (err) {
			dev_dbg(chip->dev, "failed to set ratio\n");
			goto err;
		}

		err = p7mu_write16(data->res->start + P7MU_PWM_DIV, div);
		if (err) {
			dev_dbg(chip->dev, "failed to set period\n");
			goto err;
		}

		err = p7mu_set_pwmclk_rate(pwm->hwpwm,  phz);
		if (err) {
			dev_dbg(chip->dev, "failed to set parent clock frequency\n");
			goto err;
		}

		err = p7mu_mod16(data->res->start + P7MU_PWM_CTRL,
						 P7MU_PWM_LOAD,
						 P7MU_PWM_LOAD);
		if (err) {
			dev_dbg(chip->dev, "failed to load ratop / period\n");
			goto err;
		}

#ifdef DEBUG
		{
			unsigned long const t = p7mu_pwm_period(div, phz);

			dev_dbg(chip->dev, "configured %d[%d] period=%luusec, duty=%luusec\n",
					pwm->pwm,
					pwm->hwpwm,
					t,
					p7mu_pwm_duty(t, ratio));
		}
#endif
	}

	data->div = div;
	data->ratio = ratio;
	return 0;

err:
	dev_err(chip->dev,
			"failed to setup %d[%d] (%d)\n",
			pwm->pwm,
			pwm->hwpwm,
			err);
	return err;
}

static int p7mu_enable_pwm(struct pwm_chip* chip, struct pwm_device* pwm)
{
	int                             err;
	struct p7mu_pwm const* const    data = &p7mu_pwm_data[pwm->hwpwm];

	P7MU_ASSERT_PWM(chip, pwm);

	err = p7mu_mod16(data->res->start + P7MU_PWM_CTRL,
	                 P7MU_PWM_EN,
	                 P7MU_PWM_EN);
	if (err)
		dev_err(chip->dev,
				"failed to enable %d[%d] (%d)\n",
				pwm->pwm,
				pwm->hwpwm,
				err);
	else
		dev_dbg(chip->dev,
				"enabled %d[%d]\n",
				pwm->pwm,
				pwm->hwpwm);
	return err;
}

static void p7mu_disable_pwm(struct pwm_chip* chip, struct pwm_device* pwm)
{
	int                             err;
	struct p7mu_pwm const* const    data = &p7mu_pwm_data[pwm->hwpwm];

	P7MU_ASSERT_PWM(chip, pwm);

	err = p7mu_mod16(data->res->start + P7MU_PWM_CTRL,
	                 0,
	                 P7MU_PWM_EN);

	if (err)
		dev_err(chip->dev,
				"failed to disable %d[%d] (%d)\n",
				pwm->pwm,
				pwm->hwpwm,
				err);
	else
		dev_dbg(chip->dev,
				"disabled %d[%d]\n",
				pwm->pwm,
				pwm->hwpwm);
}

#ifdef CONFIG_DEBUG_FS

static int p7mu_pwm_info(struct pwm_device const* pwm,
                         unsigned long* period,
                         unsigned long* duty)
{
	struct p7mu_pwm const* const    data = &p7mu_pwm_data[pwm->hwpwm];
	unsigned long const             phz = p7mu_get_pwmclk_rate(pwm->hwpwm);

	if ((int) phz < 0) {
		dev_warn(pwm->chip->dev,
		         "failed to get parent clock rate for %d[%d] (%d)\n",
		         pwm->pwm,
		         pwm->hwpwm,
		         (int) phz);
		return (int) phz;
	}

	if (data->ratio == (u16) ~(0U))
		/* Not configured yet. */
		return -EBADFD;

	*period = p7mu_pwm_period(data->div, phz);
	*duty = p7mu_pwm_duty(*period, data->ratio);

	return 0;
}

static void p7mu_show_pwm(struct pwm_chip* chip, struct seq_file* s)
{
	unsigned int p;

	for (p = 0; p < chip->npwm; p++) {
		struct pwm_device const* const  pwm = &chip->pwms[p];
		bool const                      req = test_bit(PWMF_REQUESTED, &pwm->flags);
		bool const                      en = test_bit(PWMF_ENABLED, &pwm->flags);
		unsigned long                   period, duty;

		P7MU_ASSERT_PWM(chip, pwm);

		seq_printf(s, " pwm-%-3d (%-20.20s)", p, pwm->label);
		if (req || en)  {
			int const err = p7mu_pwm_info(pwm, &period, &duty);

			if (! err) {
				if (req)
					seq_printf(s, ": requested @ %lu / %lu usec", duty, period);
				else if (en)
					seq_printf(s, ": enabled @ %lu / %lu usec", duty, period);
			}
		}
		seq_printf(s, "\n");
	}
}

#define P7MU_SHOW_PWM   p7mu_show_pwm

#else   /* CONFIG_DEBUG_FS */

#define P7MU_SHOW_PWM   NULL

#endif  /* CONFIG_DEBUG_FS */

static struct pwm_ops const p7mu_pwm_ops = {
	.request    = p7mu_request_pwm,
	.free       = p7mu_free_pwm,
	.config     = p7mu_config_pwm,
	.enable     = p7mu_enable_pwm,
	.disable    = p7mu_disable_pwm,
#ifdef CONFIG_DEBUG_FS
	.dbg_show   = P7MU_SHOW_PWM,
#endif
	.owner      = THIS_MODULE
};

static struct pwm_chip p7mu_pwm_chip = {
	.ops    = &p7mu_pwm_ops,
	.base   = P7MU_FIRST_PWM,
	.npwm   = P7MU_PWMS_MAX
};

static int __devinit p7mu_probe_pwm(struct platform_device* pdev)
{
	unsigned int    r;
	unsigned int    nr;
	int             err;

	for (r = 0, nr = 0; r < pdev->num_resources; r++) {
		struct resource const* const res = &pdev->resource[r];

		if (resource_type(res) != IORESOURCE_IO)
			continue;

		p7mu_pwm_data[nr].ratio = (u16) ~(0U);
		p7mu_pwm_data[nr].res = p7mu_request_region(pdev, res);
		if (! IS_ERR(p7mu_pwm_data[nr].res))
			nr++;
	}

	if (nr != P7MU_PWMS_MAX) {
		dev_err(&pdev->dev, "failed to find I/O regions\n");
		err = -ENXIO;
		goto err;
	}

	p7mu_pwm_chip.dev = &pdev->dev;
	err = pwmchip_add(&p7mu_pwm_chip);
	if (! err)
		return 0;

err:
	for (r = 0; r < nr; r++)
		p7mu_release_region(p7mu_pwm_data[r].res);

	dev_err(&pdev->dev,
	        "failed to register chip (%d)\n",
	        err);
	return err;
}

static int __devexit p7mu_remove_pwm(struct platform_device* pdev)
{
	unsigned int    r;
	int const       err = pwmchip_remove(&p7mu_pwm_chip);

	if (err) {
		dev_err(&pdev->dev,
		        "failed to unregister chip (%d)\n",
		        err);
		return err;
	}

	for (r = 0; r < P7MU_PWMS_MAX; r++) {
#ifdef DEBUG
		BUG_ON(! p7mu_pwm_data[r].res);
		BUG_ON(IS_ERR(p7mu_pwm_data[r].res));
#endif
		p7mu_release_region(p7mu_pwm_data[r].res);
	}

	return 0;
}

#define P7MU_PWM_DRV_NAME   "p7mu-pwm"

static struct platform_driver p7mu_pwm_driver = {
	.driver = {
		.name   = P7MU_PWM_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = p7mu_probe_pwm,
	.remove = __devexit_p(&p7mu_remove_pwm),
};

module_platform_driver(p7mu_pwm_driver);

MODULE_DESCRIPTION("Parrot Power Management Unit PWM driver");
MODULE_AUTHOR("Grégor Boirie <gregor.boirie@parrot.com>");
MODULE_LICENSE("GPL");
