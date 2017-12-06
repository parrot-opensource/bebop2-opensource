/**
 * linux/drivers/parrot/pwm/p7-pwm.c - Parrot7 PWM driver implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author: Victor Lambret <victor.lambret.ext@parrot.com>
 * date:    28-Nov-2012
 *
 * This file is released under the GPL
 */

#if defined(CONFIG_PWM_PARROT7_DEBUG)
#define DEBUG
#define P7PWM_ASSERT(_chip, _pwm)      \
    BUG_ON(! _chip);                    \
    BUG_ON(! _pwm);                     

#else /*Empty debug macro if debug is OFF*/
#define P7PWM_ASSERT(_chip, _pwm)

#endif /*CONFIG_PWM_PARROT7_DEBUG*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <mach/hardware.h>
#include <linux/pinctrl/consumer.h>
#include "p7_pwm.h"


/******************************************************************************
 *  DRIVER DATA
******************************************************************************/


/* Comment on the usage of the mutex :
 * The linux PWM framework already deal with concurrency problems so the
 * drivers dont need to use mutex. At the moment, this mutex is useless, but
 * we may have to write another pwm driver to use all P7 PWM capabilities not
 * managed by linux PWM framework like servomotors. The usage of the mutex is a
 * guaranty that the access to PWM registers will be shared without conflicts
 * between those drivers.
 *
 * This is not true for enable/disable/configure where no lock is taken by
 * the framework.
 */

static DEFINE_SPINLOCK(p7pwm_spinlock);

struct p7pwm_config {
	int             min_ratio;
	int             speed_regval;
	int             ratio_regval;
	int             duty_ns;
	int             period_ns;
	struct pinctrl  *p;
};

static struct p7pwm_pdata *    pdata;
static unsigned long            clk_freq;
static struct clk *             clk;
static void __iomem *           mmio_base;
static struct p7pwm_config     config[P7PWM_NUMBER];
static struct pinctrl *         pins;
static struct input_dev *       input_dev;

int servo_rx_no_filter = 1;
module_param(servo_rx_no_filter, int, S_IRUGO);
MODULE_PARM_DESC(servo_rx_no_filter, "0/1(default) : if 1 the event won't be" 
		" filtered and repeated every 22 ms.");

#define P7PWM_SPEED(i)          (0x0+0x10*(i))
#define P7PWM_RATIO(i)          (0x4+0x10*(i))
#define P7PWM_START             (0xFF00)
#define P7PWM_MODE              (0xFF04)

#define P7PWM_MUX(i)            (0xa00 + 4*((i)/4))
#define P7PWM_SERVO_RX_IN(i)    (0x900 + 4*(i))
#define P7PWM_SERVO_RX_RESYNC   (0x920)
#define P7PWM_SERVO_RX_MODE     (0x940)
#define P7PWM_SERVO_RX_DIV      (0x960)
#define P7PWM_SERVO_RX_ITEN     (0x980)
#define P7PWM_SERVO_RX_IT_CFG   (0x9c0)

/******************************************************************************
 *  DRIVER HELPERS
******************************************************************************/

static int     is_mode(int pwm, int mode)
{
	if (NULL == pdata->conf[pwm])
		return 0;
	else
		return (mode ==pdata->conf[pwm]->mode);
}

#define is_used(_pwm)     (pdata->used & (1 << (_pwm)))

#define DIV64_ROUND_CLOSEST(number, divisor)(                   \
{                                                               \
	typeof(number) __number = (number) + ((divisor) / 2);   \
	typeof(divisor) __div = (divisor);                      \
	do_div((__number),(__div));                             \
	__number;                                               \
}                                                               \
)

static void write_config(uint32_t pwm, uint32_t speed, uint32_t ratio, struct p7pwm_config *config)
{
	if (!config || config->speed_regval != speed)
		__raw_writel(speed, mmio_base + P7PWM_SPEED(pwm));
	if (!config || config->ratio_regval != ratio)
		__raw_writel(ratio, mmio_base + P7PWM_RATIO(pwm));
}

static unsigned long freq(uint32_t speed_reg, uint32_t ratio, unsigned long mode)
{
	unsigned long res;
	res = DIV64_ROUND_CLOSEST(clk_freq, 2*(speed_reg +1));
	if (P7PWM_MODE_NORMAL == mode)
		res = res >> ratio;
	return res;
}

static unsigned long duty(uint32_t ratio_reg, unsigned long mode)
{
	unsigned long rt;
	if (1 == mode)
		rt = 50;
	else {
		uint32_t ratio, hratio;
		ratio = (ratio_reg >> 16 );
		hratio = ratio_reg & 0xFFFF;
		rt = DIV64_ROUND_CLOSEST(100*hratio, (1 << ratio));
	}
	return rt;
}

static int to_min_ratio(uint16_t precision)
{
	int r = 0;
	if (precision < 1)
		r = 16;
	else if (precision < 2)
		r = 7;
	else if (precision < 4)
		r = 6;
	else if (precision < 7)
		r = 5;
	else if (precision < 13)
		r = 4;
	else if (precision < 25)
		r = 3;
	else if (precision < 50)
		r = 2;
	else
		r = 1;

	return r;
}

/******************************************************************************
 *  PWM FRAMEWORK INTERFACE
******************************************************************************/

static int      p7pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	int         rc = -ENOENT;

	P7PWM_ASSERT(chip, pwm);

	dev_dbg(chip->dev, "%s:requesting pwm %d\n", __func__,pwm->hwpwm);

	if (is_used(pwm->hwpwm) && !is_mode(pwm->hwpwm, P7PWM_MODE_PPM_RX)) {
		config[pwm->hwpwm].speed_regval  = -1;
		config[pwm->hwpwm].ratio_regval  = -1;
		config[pwm->hwpwm].min_ratio     = to_min_ratio(pdata->conf[pwm->hwpwm]->duty_precision);
		rc = 0;
	}

	return rc;
}

static void     p7pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	P7PWM_ASSERT(chip, pwm);

	dev_dbg(chip->dev,"p7pwm_free: P7 pwm %d\n",pwm->hwpwm);
}



/* Calculation :
 *
 * Refer to the Pulse with Modulation section of the P7 User Manual for more
 * the original specification.
 *
 * The goal of the calculus is to compute the best speed and ratio register
 * values to reach demanded period and duty cycle.
 *
 * Basicly we have to compute three values noted :
 *
 * speed : The speed value
 * ratio : The bits [20:16] of the ratio register.
 * hratio : The bits [15:0] of the ratio register.
 *
 * The period formula is :
 *      (period_ns / 10^9) = (2*(Speed+1)*2^ratio) / clk_freq
 *
 * With a little transformation it is equivalent to :
 *      (Speed+1)*2^ratio = (period_ns * clk_freq) / (2*10^9)
 *
 * We call ideal_div = (period_ns * clk_freq) / (2*10^9) the ideal divider
 *
 * many couples (Speed,ratio) are solution of this equation.
 *
 *
 * Frequency computation Steps
 * ===========================
 *
 *      1) Compute the ideal divider
 *      2) The initial ratio is the binary logarithm of the 2 most significant
 *         bytes.
 *      3) With the ratio fixed, initial speed can be computed with :
 *         Speed = (ideal_div / (2^ratio)) -1
 *
 * This couple is the couple maximizing frequency. As Frequency and duty cycle
 * accuracy are linked, it may not be sufficient to get the demanded accuracy
 * on duty cycle. If a minimal ratio is specified with platform-data we try to
 * increase ratio and keep the frequency close to the initial one.
 *
 * If initial ratio r1 is inferior to minimal ratio r2 then we have :
 *      r2 = r1 + n
 * If we set ratio to r2, we have a new speed Speed2 computed with :
 *
 *      Speed2 = ((Speed1 + 1) / 2^n) -1
 *
 *      4) If needed set Speed2 as new speed and minimal ratio as ratio. 
 *
 *
 * Duty cycle computations steps
 * =============================
 *
 * The duty cycle is given by the formula :
 *
 * (duty_ns / period_ns) = hratio / 2^ratio
 *
 * so it means that hratio = (duty_ns * 2^ratio) / period
 *
 *      5) Compute hratio
 *      6) We keep extreme values only if they are really the extremes :
 *         - If hratio is 0 and duty_ns > 0 then set hratio to 1.
 *         - If hratio is max and duty-ns < period_ns then substract one to
 *           hratio.
 */

static int p7pwm_configure(struct pwm_chip *chip,
		struct pwm_device *pwm, int duty_ns, int period_ns)
{

	uint64_t        divider;
	uint32_t        speed, ratio, hratio = 0;
	int             rc = 0;
	unsigned long   f, res ;

	P7PWM_ASSERT(chip, pwm);
	dev_dbg(chip->dev,"%s: PWM_%d, modeclock=%d, period=%d, ratio=%d\n",
		__func__,
		pwm->hwpwm,
		pdata->conf[pwm->hwpwm]->mode,
		period_ns,
		duty_ns);

	/* Check invalid parameters values */
	if ((period_ns <= 0) || ((duty_ns < 0) && is_mode(pwm->hwpwm,P7PWM_MODE_NORMAL))) {
		dev_warn(chip->dev,"%s:Invalid argument\n",__func__);
		rc = -EINVAL;
		goto error;
	}

	if (duty_ns > 0 && is_mode(pwm->hwpwm,P7PWM_MODE_CLOCK))
		dev_warn(chip->dev,"%s:non zero duty with PWM %d configured in clock mode !\n",
		         __func__,
		         pwm->hwpwm);

	/* Step 1 : Compute ideal divider */
	divider = ((uint64_t)period_ns) * clk_freq;
	dev_dbg(chip->dev,"STEP1 : divider1=%llu\n",divider);
	divider = DIV64_ROUND_CLOSEST(divider, 2*1000000000);
	dev_dbg(chip->dev,"STEP1 : divider2=%llu\n",divider);
	/* A divider of zero cause problems, force divider to be 1 at least */
	if (0 == divider)
		divider = 1;

	/* Step 2 : Compute initial ratio */ 
	if (is_mode(pwm->hwpwm,P7PWM_MODE_NORMAL)) {
		ratio = ilog2((divider >> 16 ) & 0xFFFF) + 1;
	} else {
		ratio = 0;
	}
	dev_dbg(chip->dev,"STEP2 : ratio=0x%x\n",ratio);

	/* Step 3 : Compute initial speed */
	speed = (divider >> ratio) - 1;
	dev_dbg(chip->dev,"STEP3 : speed=0x%x\n",speed);

	if (is_mode(pwm->hwpwm,P7PWM_MODE_NORMAL)) {
		int             min_ratio;
		min_ratio = config[pwm->hwpwm].min_ratio;
		/* Step 4 : If needed, increase ratio and recompute speed with new ratio */
		if (ratio < min_ratio) {
			unsigned int delta_div= 1 << (min_ratio - ratio);
			dev_dbg(chip->dev,"%s: Adjusting for a minimal ratio of %d\n",
				__func__,
				min_ratio);
			speed = DIV64_ROUND_CLOSEST((uint64_t)speed+1, delta_div) - 1;
			ratio = min_ratio;
		}

		/* Step 5 : Compute hratio */
		hratio = 0xFFFF & DIV64_ROUND_CLOSEST((uint64_t)(1<<ratio)*duty_ns, period_ns);

		/* Step 6 : Deal with hratio extreme values */
		if ((0 == hratio) && (duty_ns > 0))
		{
			dev_dbg(chip->dev,"%s: set hratio to 1 because duty cycle is positive\n",
				__func__);
			hratio = 1;
		}
		else if (((1 << ratio) <= hratio) && (duty_ns < period_ns))
		{
			dev_dbg(chip->dev,"%s: reduce hratio by 1 because duty cycle is not 100\n",
				__func__);
			hratio--;
		}
	}
	speed = min(speed, 0xFFFFU);

	/* Test if frequency precision is high enough */
	dev_dbg(chip->dev,"PRECISION0: speed=%d,ratio=%d,mode=%d\n",speed,ratio,pdata->conf[pwm->hwpwm]->mode);
	f = freq(speed,ratio,pdata->conf[pwm->hwpwm]->mode);
	dev_dbg(chip->dev,"PRECISION1: f=%lu\n",f);
	if (f == 0)
		f = 1;
	res = DIV64_ROUND_CLOSEST(1000000000,f);
	dev_dbg(chip->dev,"PRECISION1: res1=%lu\n",res);
	if (res > period_ns) {
		res = DIV64_ROUND_CLOSEST(100*(res - period_ns), period_ns);
		dev_dbg(chip->dev,"PRECISION2: res2=%lu\n",res);
	}
	else {
		dev_dbg(chip->dev,"PRECISION2: res3=%lu\n",res);
		res = DIV64_ROUND_CLOSEST(100*(period_ns - res), period_ns);
	}
	dev_dbg(chip->dev,"%s: period precision computed :%lu/100 (needed : %d/100)\n",
		__func__,
		res,
		pdata->conf[pwm->hwpwm]->period_precision);
	if (res > pdata->conf[pwm->hwpwm]->period_precision) {
		dev_err(chip->dev,"%s: ERROR : insuffisant precision of %lu/100 (needed : %d/100)\n",
			__func__,
			res,
			pdata->conf[pwm->hwpwm]->period_precision);
		rc = -EINVAL;
		goto error;
	}

	/* Write the values in registers */
	speed = speed & 0xFFFF;
	ratio = ((ratio & 0x1F) << 16) | (hratio & 0xFFFF);
	
	dev_dbg(chip->dev,"%s: computed values: period=0x%x, ratio=0x%x\n",
		__func__,
		speed,
		ratio);

	write_config(pwm->hwpwm, speed, ratio, &config[pwm->hwpwm]);

	config[pwm->hwpwm].speed_regval = speed;
	config[pwm->hwpwm].ratio_regval = ratio;
	config[pwm->hwpwm].duty_ns = duty_ns;
	config[pwm->hwpwm].period_ns = period_ns;


error:
	return rc;
}

static int p7pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	uint32_t start;
	int rc = 0;
	
	P7PWM_ASSERT(chip, pwm) ;
	dev_dbg(chip->dev,"%s: PWM_%d\n",__func__,
		pwm->hwpwm);
	spin_lock(&p7pwm_spinlock);
	/* start of MPW1 workaround */
	/* restore configure overwritten in disable */
	if (config[pwm->hwpwm].speed_regval != -1 && config[pwm->hwpwm].ratio_regval != -1) {
		/* disable counter */
		start = __raw_readl(mmio_base + P7PWM_START);
		start = start & ~(0x1 << pwm->hwpwm);
		__raw_writel(start, mmio_base + P7PWM_START);

		write_config(pwm->hwpwm, config[pwm->hwpwm].speed_regval, config[pwm->hwpwm].ratio_regval, NULL);
	}
	/* end of MPW1 workaround */

	
	if (is_mode(pwm->hwpwm,P7PWM_MODE_CLOCK)) {
		uint32_t mode ;
		mode = __raw_readl(mmio_base + P7PWM_MODE);
		mode = mode | (0x1 << pwm->hwpwm);
		__raw_writel(mode, mmio_base + P7PWM_MODE);
	}

	/* Set the pwm start bit */
	start = __raw_readl(mmio_base + P7PWM_START);
	start = start | (0x1 << pwm->hwpwm);
	__raw_writel(start, mmio_base + P7PWM_START);
	spin_unlock(&p7pwm_spinlock);

	return rc ;
}


static void p7pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	uint32_t start;

	P7PWM_ASSERT(chip, pwm);
	dev_dbg(chip->dev,"p7pwm_disable: P7 pwm %d\n",pwm->hwpwm);

	spin_lock(&p7pwm_spinlock);

	start = __raw_readl(mmio_base + P7PWM_START);
	dev_dbg(chip->dev,"%s: start init = 0x%x\n",__func__,start);

	/* Disable counters */
	start = start & ~(0x1 << pwm->hwpwm);
	dev_dbg(chip->dev,"%s: start modified = 0x%x\n",__func__,start);
	__raw_writel(start, mmio_base + P7PWM_START);
	
	/* If clock mode, switch on normal mode */
	if (is_mode(pwm->hwpwm,P7PWM_MODE_CLOCK)) {
		uint32_t mode;
		mode = __raw_readl(mmio_base + P7PWM_MODE);
		mode = mode & ~(0x1 << pwm->hwpwm);
		__raw_writel(mode, mmio_base + P7PWM_MODE);
	}

	/* It's not possible to disable the pwm using the START bit in MPW1, so it just
	 * configure with a duty cycle of 0 and a at speed MAX */

	/* Configured speed at maximum and ratio to have a low state signal */
	__raw_writel(0, mmio_base + P7PWM_SPEED(pwm->hwpwm));
	__raw_writel(0x10000, mmio_base + P7PWM_RATIO(pwm->hwpwm));

	/* Restart counters */
	start = start | (0x1 << pwm->hwpwm);
	__raw_writel(start, mmio_base + P7PWM_START);
	/* end of MPW1 workaround */


	spin_unlock(&p7pwm_spinlock);
}

/******************************************************************************
 *  DEBUGFS INTERFACE
******************************************************************************/

#ifdef CONFIG_DEBUG_FS

static void p7pwm_info(struct pwm_device const* pwm,
                         unsigned long* frequency,
                         unsigned long* dutycycle,
                         unsigned long* dutyns,
                         unsigned long mode)
{
	int     n= pwm->hwpwm;
	if (config[n].speed_regval == -1 || config[n].ratio_regval == -1) {
		*frequency = 1;
		*dutycycle = 0;
		*dutyns = 0;
	}
	else {
		unsigned long freqdiv = freq(config[n].speed_regval, 0, P7PWM_MODE_CLOCK);

		*frequency = freq(   config[n].speed_regval,
				(config[n].ratio_regval >> 16),
				mode);
		*dutycycle = duty(   config[n].ratio_regval,
				mode);
		*dutyns = (config[n].ratio_regval & 0xffff) * (1000000000 / freqdiv);
	}
}

static void p7_show_pwm(struct pwm_chip* chip, struct seq_file* s)
{
	unsigned int p;

	for (p = 0; p < chip->npwm; p++) {
		struct pwm_device const* const  pwm = &chip->pwms[p];
		bool const                      req = test_bit(PWMF_REQUESTED, &pwm->flags);
		bool const                      en = test_bit(PWMF_ENABLED, &pwm->flags);
		unsigned long                   freq, duty, duty_ns ;
		int clock ;

		P7PWM_ASSERT(chip, pwm);
	

		/* Get CLOCK MODE infos*/
		if (is_mode(p,P7PWM_MODE_CLOCK))
			clock=1 ;
		else
			clock=0 ;

		seq_printf(s, " pwm-%-3d (%-20.20s)", p, pwm->label);
		if (req || en)  {
			p7pwm_info(pwm, &freq, &duty, &duty_ns, clock);

			seq_printf(s, "requested=%d enabled=%d clock_mode=%d freq=%luHz duty=%lu/100(%luns)\n",
			           req,
			           en,
			           clock,
			           freq,
			           duty,
			           duty_ns);
			seq_printf(s,"                               period_precision=%d, duty_precision=%d (min_ratio=%d)\n",
			           pdata->conf[pwm->hwpwm]->period_precision,
			           pdata->conf[pwm->hwpwm]->duty_precision,
			           config[pwm->hwpwm].min_ratio);
			seq_printf(s,"                               period error=%d/1000, duty error=%d/1000\n",
			           (int)(config[pwm->hwpwm].period_ns-1000000000/freq)*1000/config[pwm->hwpwm].period_ns,
			           config[pwm->hwpwm].duty_ns ? (int)(config[pwm->hwpwm].duty_ns-duty_ns)*1000/config[pwm->hwpwm].duty_ns : 0);
			seq_printf(s,"                               REGS={speed=0x%x, ratio=0x%x}\n",
			           config[pwm->hwpwm].speed_regval,
			           config[pwm->hwpwm].ratio_regval);
			seq_printf(s,"                               user={period=%dns, duty=%dns}\n",
			           config[pwm->hwpwm].period_ns,
			           config[pwm->hwpwm].duty_ns);
		} else if (is_mode(p, P7PWM_MODE_PPM_RX)) {
			seq_printf(s, "servo_rx\n");
		} else
			seq_printf(s, "\n");
	}
}

#define P7_SHOW_PWM   p7_show_pwm

#else   /* CONFIG_DEBUG_FS */

#define P7_SHOW_PWM   NULL

#endif  /* CONFIG_DEBUG_FS */

static struct pwm_ops p7pwm_ops = {
	.request        = p7pwm_request,
	.free           = p7pwm_free,
	.enable         = p7pwm_enable,
	.disable        = p7pwm_disable,
	.config         = p7pwm_configure,
#ifdef CONFIG_DEBUG_FS
	.dbg_show       = P7_SHOW_PWM,
#endif
	.owner          = THIS_MODULE,
};


static struct pwm_chip p7pwm_chip = {
	.ops    = &p7pwm_ops,
	.base   = P7_FIRST_PWM,
	.npwm   = P7PWM_NUMBER
};

static const signed short servo_rx_abs[] = {
	ABS_X, ABS_Y,
	ABS_RX, ABS_RY,
	ABS_Z, ABS_RZ,
	ABS_HAT0X, ABS_HAT0Y,
};


static irqreturn_t servo_rx_irq(int irq, void *dev_id)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(servo_rx_abs); i++) {
		/* reading data ack irq */
		int val = __raw_readl(mmio_base + P7PWM_SERVO_RX_IN(i));
		/*
		   XXX the center is 1520 on futaba RC
		 */
		val -= 1500;
		/* XXX hack to not filter value */
		if (servo_rx_no_filter)
			input_dev->absinfo[servo_rx_abs[i]].value = val - 10;
		input_report_abs(input_dev, servo_rx_abs[i], val);
	}
	input_sync(input_dev);
	return IRQ_HANDLED;
}

static int servo_rx_open(struct input_dev *dev)
{
	/* start irq */
	__raw_writel(1, mmio_base + P7PWM_SERVO_RX_ITEN);
	return 0;
}

static void servo_rx_close(struct input_dev *dev)
{
	/* stop irq */
	__raw_writel(0, mmio_base + P7PWM_SERVO_RX_ITEN);
}

static int __devinit servo_rx_probe(struct platform_device *pdev)
{
	unsigned int pwm;

	for (pwm = 0; pwm < P7PWM_NUMBER; pwm++) {
		if (is_used(pwm) && is_mode(pwm, P7PWM_MODE_PPM_RX)) {
			dev_info(&pdev->dev, "servo rx on pwm %d\n", pwm);
			break;
		}
	}

	if (pwm != P7PWM_NUMBER) {
		/* muxing in servo rx serial */
		__raw_writel(7 << ((pwm % 4)*8), mmio_base + P7PWM_MUX(pwm));
		__raw_writel(0, mmio_base + P7PWM_SERVO_RX_ITEN);
		/* data register are 12 bits (4095).
		   2ms (max value) fit on 11 bits)
		   but with trim we can get 2066 on futaba rc
		 */
		__raw_writel(clk_freq/1000000, mmio_base + P7PWM_SERVO_RX_DIV);
		/* servo rx sync len : 3ms */
		__raw_writel(3000, mmio_base + P7PWM_SERVO_RX_RESYNC);
		/* serial mode, failling mode
		   The mode configuration rising or failling don't change the result because we are measuring the whole pulse and the low pulse is always 500 us in our case.
		 */
		__raw_writel(0x200, mmio_base + P7PWM_SERVO_RX_MODE);

		input_dev = input_allocate_device();
		if (input_dev) {
			int i;
			int ret;
			input_dev->name = pdev->name;
			input_dev->dev.parent = &pdev->dev;
			__set_bit(EV_ABS, input_dev->evbit);
			for (i = 0; i < ARRAY_SIZE(servo_rx_abs); i++) {
				signed short abs = servo_rx_abs[i];
				set_bit(abs, input_dev->absbit);
				input_set_abs_params(input_dev, abs, -500, 500, 3, 0);
			}
			ret = request_irq(P7_PWM_SERVO_RX_IRQ, servo_rx_irq, 0, pdev->name, pdata);
			if (ret) {
				dev_err(&pdev->dev,"can't register pwm irq");
				input_free_device(input_dev);
				input_dev = NULL;
			}
			else {
				__raw_writel(0xf, mmio_base + P7PWM_SERVO_RX_IT_CFG);
				input_dev->open = servo_rx_open;
				input_dev->close = servo_rx_close;
			}
		}
	}
	return 0;
}
static int __devinit servo_rx_register(void)
{
	int r;
	/* we need to wait that input layer is ready : register later */
	if (input_dev)
		r = input_register_device(input_dev);
	return 0;
}

static int __devinit p7pwm_probe(struct platform_device *pdev)
{
	struct resource * r ;
	int rc;
	
	dev_dbg(&pdev->dev,"p7pwm_probe:\n") ;
	
	rc = 0 ;

	/* get the platform_data */
	pdata = (struct p7pwm_pdata *) dev_get_platdata(&pdev->dev) ;

	if (NULL == pdata) {
		dev_warn(&pdev->dev,"p7pwm_probe: Cant get minimal ratio : no platform data\n") ;
		rc = -EINVAL ;
		goto err_exit ;
	}

	/*Enable Clock*/
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_warn(&pdev->dev,"p7pwm_probe:Failed : Cant get clock\n") ;
		rc = PTR_ERR(clk);
		goto err_exit ;
	}
	/* Get clock rate */
	clk_freq = clk_get_rate(clk);
	dev_dbg(&pdev->dev,"p7pwm_probe: clk rate is %ld\n",clk_freq) ;

	rc = clk_prepare_enable(clk);
	if (rc) {
		dev_warn(&pdev->dev,"Pp7pwm_probe:Failed :Cant start clock\n") ;
		goto err_putclk ;
	}

	/*Get base adress*/
	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (r == NULL) {
		dev_warn(&pdev->dev,"p7pwm_probe:Failed : No memory resource defined\n");
		rc = -ENODEV;
		goto err_disclk ;
	}

	mmio_base = devm_request_and_ioremap(&pdev->dev, r);
	dev_dbg(&pdev->dev,"%s: mmio_base = 0x%x\n",__func__,(unsigned int) mmio_base) ;
	if (mmio_base == NULL) {
		dev_warn(&pdev->dev,"p7pwm_probe:Failed:Cant get memory base\n");
		rc = -EADDRNOTAVAIL;
		goto err_disclk ;
	}
	
	/*Get Pins*/
	pins = pinctrl_get_select_default(&pdev->dev) ;
	if (IS_ERR(pins)) {
		dev_err(&pdev->dev, "%s:failed to request pins\n", __func__);
		rc = PTR_ERR(pins);
		goto err_disclk ;
	}

	/*Fill pwm_chip*/
	p7pwm_chip.dev = &pdev->dev ;

	servo_rx_probe(pdev);

	rc = pwmchip_add(&p7pwm_chip);
	if (rc < 0) {
		dev_warn(&pdev->dev,"p7pwm_probe:Failed :Cant add chip\n");
	}

goto err_exit ;

err_disclk:
	clk_disable(clk) ;
err_putclk:
	clk_put(clk) ;
err_exit:
	return rc;
}


static int __devexit p7pwm_remove(struct platform_device *pdev)
{
	int rc ;

	dev_dbg(&pdev->dev,"p7pwm_remove:\n") ;

	if (input_dev) {
		free_irq(P7_PWM_SERVO_RX_IRQ, pdata);
		input_unregister_device(input_dev);
		input_free_device(input_dev);
	}
	rc = 0 ;
	pinctrl_put(pins);

	rc = pwmchip_remove(&p7pwm_chip) ;
	if (rc) {
		dev_warn(&pdev->dev,"p7pwm_remove:Removing Driver with a PWM still in use\n") ;
	}

	clk_disable(clk) ;
	clk_put(clk) ;

	return rc ;
}


/* XXX there is no suspend/resume support */
static struct platform_driver p7pwm_driver = {
	.driver		= {
		.name	= P7PWM_DRV_NAME,
	},
	.probe		= p7pwm_probe,
	.remove		= __devexit_p(p7pwm_remove),
};


static int __init p7pwm_init(void)
{
	return platform_driver_register(&p7pwm_driver);
}
postcore_initcall(p7pwm_init);
module_init(servo_rx_register);


static void __exit p7pwm_exit(void)
{
	platform_driver_unregister(&p7pwm_driver);
}
module_exit(p7pwm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Victor Lambret <victor.lambret.ext@parrot.com>");
