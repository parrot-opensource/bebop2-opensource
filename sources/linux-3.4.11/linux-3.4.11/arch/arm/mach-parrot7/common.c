/*
 *  linux/arch/arm/mach-parrot7/common.c
 *
 *  Copyright (C) 2010 Parrot S.A.
 *
 * @author  Gregor Boirie <gregor.boirie@parrot.com>
 * @date  24-Nov-2010
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/cnt32_to_63.h>
#include <asm/smp_twd.h>
#include <asm/pmu.h>
#include <asm/sched_clock.h>
#include <asm/localtimer.h>
#include <asm/hardware/gic.h>
#include <mach/irqs.h>
#include <mach/p7.h>
#include <mach/timex.h>
#include "system.h"
#include "clock.h"
#include "common.h"
#include "pinctrl.h"

/***********************
 * Timers & system tick
 ***********************/

static u32 p7_clocksource_val;
static u32 tmr3_val;

#ifdef ARCH_HAS_READ_CURRENT_TIMER
static unsigned int sys_to_cpu_clk_shift;

static void __devinit p7_calc_sys_to_cpu_clk_shift(void)
{
	unsigned long sys_div = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_SYS_DIV));
	unsigned long cpu_div = readl(__MMIO_P2V(P7_SYS_CLKGEN_CFG_CPU_DIV));

	sys_to_cpu_clk_shift = (sys_div + 1) - cpu_div;
}

int __devinit read_current_timer(unsigned long* timer_val)
{
	*timer_val = (~readl(MMIO_P2V(P7_SYS_TIM2_CNT)))
	                    << sys_to_cpu_clk_shift;
	return 0;
}
#else
#define p7_calc_sys_to_cpu_clk_shift()	do {} while(0)
#endif

static cycle_t p7_get_cycles(struct clocksource* clksrc)
{
	return ~readl(MMIO_P2V(P7_SYS_TIM2_CNT));
}

static u32 notrace p7_read_sched_clock(void)
{
	return ~readl(MMIO_P2V(P7_SYS_TIM2_CNT));
}

static void p7_suspend_clock(struct clocksource *cs)
{
	/* Stop the clock and store current counter value */
	writel(0, MMIO_P2V(P7_SYS_TIM2_CTL));
	p7_clocksource_val = readl(MMIO_P2V(P7_SYS_TIM2_CNT));
}

static void p7_resume_clock(struct clocksource *cs)
{
	/* Write back the stored value and start counting */
	__raw_writel(p7_clocksource_val, MMIO_P2V(P7_SYS_TIM2_CNT));
	writel(P7_SYS_TIMXCTL_ENABLE, MMIO_P2V(P7_SYS_TIM2_CTL));
}

static struct clocksource p7_clksrc = {
	.name       = "timer2",
	.rating     = 200,
	.read       = &p7_get_cycles,
	.mask       = CLOCKSOURCE_MASK(32),
	.flags      = CLOCK_SOURCE_IS_CONTINUOUS,
	.suspend    = p7_suspend_clock,
	.resume     = p7_resume_clock,
};

static enum clock_event_mode p7_tick_mode;

static void p7_set_clkevt_mode(enum clock_event_mode mode,
                               struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		/* timer load already set up */
		__raw_writel(P7_SYS_TIM3_IT, MMIO_P2V(P7_SYS_TIM_ITEN));

	case CLOCK_EVT_MODE_ONESHOT:
		/* period set, and interrupt enabled in 'next_event' hook */
		writel(P7_SYS_TIMXCTL_ENABLE, MMIO_P2V(P7_SYS_TIM3_CTL));
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		writel(0, MMIO_P2V(P7_SYS_TIM_ITEN));
		writel(0, MMIO_P2V(P7_SYS_TIM3_CTL));
		tmr3_val = __raw_readl(MMIO_P2V(P7_SYS_TIM3_LD));
		break;
	case CLOCK_EVT_MODE_RESUME:
		writel(0, MMIO_P2V(P7_SYS_TIM3_CTL));
		/* Setup boot time tick timer rate. */
		__raw_writel(tmr3_val, MMIO_P2V(P7_SYS_TIM3_LD));
		writel((1U << 3), MMIO_P2V(P7_SYS_TIM_ITACK));
		break;
	default:
		BUG();
	}

	p7_tick_mode = mode;
}

#ifdef CONFIG_TICK_ONESHOT
static int p7_set_clkevt_nextevt(unsigned long next,
                                 struct clock_event_device* evt)
{
	__raw_writel(next, MMIO_P2V(P7_SYS_TIM3_CNT));
	writel(P7_SYS_TIM3_IT, MMIO_P2V(P7_SYS_TIM_ITEN));
	return 0;
}
#endif

static struct clock_event_device p7_clkevt = {
	.name           = "timer3",
	.set_mode       = &p7_set_clkevt_mode,
#ifdef CONFIG_TICK_ONESHOT
	.set_next_event = &p7_set_clkevt_nextevt,
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
#else
	.features       = CLOCK_EVT_FEAT_PERIODIC,  /* FIXME: CS3 for WFI ? */
#endif
	.rating         = 100,
	.irq            = P7_TIM3_IRQ,
};

static irqreturn_t p7_handle_timer_irq(int irq, void* dev_id)
{
#ifdef CONFIG_TICK_ONESHOT
	if (p7_tick_mode == CLOCK_EVT_MODE_ONESHOT)
		/* Disable & clear interrupt. */
		writel(0, MMIO_P2V(P7_SYS_TIM_ITEN));
	else
#endif
		/* Clear interrupt */
		writel(P7_SYS_TIM3_IT, MMIO_P2V(P7_SYS_TIM_ITACK));

	p7_clkevt.event_handler(&p7_clkevt);

	return IRQ_HANDLED;
}

static struct irqaction p7_timer_irq = {
	.name       = "timer",
	.flags      = IRQF_TIMER | IRQF_IRQPOLL,
	.handler    = &p7_handle_timer_irq,
	.dev_id     = &p7_clkevt,
};

#ifdef CONFIG_HAVE_ARM_TWD
static DEFINE_TWD_LOCAL_TIMER(p7_twd,
                              P7_CPU_LOCALTIMER,
                              P7_LOCALTIMER_IRQ);

static void __init p7_init_twd(void)
{
	int err = twd_local_timer_register(&p7_twd);
	if (err)
		pr_err("p7: local timer register failed %d\n", err);
}
#else
#define p7_init_twd()	do {} while(0)
#endif


void __init inline _p7_init_timer(void)
{
	unsigned long sys_hz;
	struct clk* p7_sys_clk;

	pr_debug("p7: tick setup\n");

	p7_sys_clk = clk_get_sys("sys_clk", NULL);
	sys_hz = clk_get_rate(p7_sys_clk);
	clk_put(p7_sys_clk);

	/* sched clock */
	setup_sched_clock(&p7_read_sched_clock, 32, sys_hz);

	/* clk source */

	/* Disable timers. */
	__raw_writel(0, MMIO_P2V(P7_SYS_TIM2_CTL));
	/* Setup clock source timer rate. */
	__raw_writel(0xffffffff, MMIO_P2V(P7_SYS_TIM2_LD));
	/* Now launch available clock source and event timers. */
	writel(P7_SYS_TIMXCTL_ENABLE, MMIO_P2V(P7_SYS_TIM2_CTL));
	/* register */
	clocksource_register_hz(&p7_clksrc, sys_hz);


	/* Disable timers. */
	writel(0, MMIO_P2V(P7_SYS_TIM3_CTL));
	/* Setup boot time tick timer rate. */
	__raw_writel(sys_hz / HZ, MMIO_P2V(P7_SYS_TIM3_LD));
	/* Finalize boot time tick timer setup.*/
	__raw_writel(0xf, MMIO_P2V(P7_SYS_TIM_ITACK));
	setup_irq(p7_clkevt.irq, &p7_timer_irq);

	/* Setup boot time periodic tick timer (clock event device). */
	p7_clkevt.cpumask = cpumask_of(0);
	clockevents_calc_mult_shift(&p7_clkevt, sys_hz, 5);
	p7_clkevt.max_delta_ns = clockevent_delta2ns(0xffffffff, &p7_clkevt);
	p7_clkevt.min_delta_ns = clockevent_delta2ns(0xf, &p7_clkevt);

	clockevents_register_device(&p7_clkevt);


	p7_calc_sys_to_cpu_clk_shift();
	p7_init_twd();
}


void __init p7_init_timer(void)
{
	/* Setup PLLs & clocks. */
	p7_init_clk(0);

	_p7_init_timer();
}

void __init p7_low_sysclk_init_timer(void)
{
	/* Setup PLLs & clocks. */
	p7_init_clk(1);

	_p7_init_timer();
}

struct sys_timer p7_tick_timer = {
	.init   = p7_init_timer,
};

struct sys_timer p7_low_sysclk_tick_timer = {
	.init   = p7_low_sysclk_init_timer,
};

/*************
 * Interrupts
 *************/

void __init p7_init_irq(void)
{
	pr_debug("p7: IRQ setup\n");
	gic_init(0, P7_LOCALTIMER_IRQ, MMIO_P2V(P7_CPU_ICD), MMIO_P2V(P7_CPU_ICC));
}

/**********************
 * Device registration
 **********************/

int __init p7_init_dev(struct platform_device* pdev,
                       void* pdata,
                       struct pinctrl_map* pins,
                       size_t pin_cnt)
{
	int err;

	if (pdata)
		pdev->dev.platform_data = pdata;

	pr_debug("p7: registering %s.%d...\n", pdev->name, pdev->id);

	err = platform_device_register(pdev);
	if (! err && pin_cnt) {
		err = p7_assign_pins(dev_name(&pdev->dev), pins, pin_cnt);
		if (err)
			platform_device_unregister(pdev);
	}

	if (err)
		pr_err("p7: failed to register %s.%d (%d)\n",
		       pdev->name,
		       pdev->id,
		       err);

	return err;
}

/**
 * p7_chiprev() - Retrieve Parrot7 chip revision number
 *
 * Returns: -1 if not a Parrot7, revision number otherwise
 */
int p7_chiprev(void)
{
	static int rev __initdata = INT_MAX;

	if (rev == INT_MAX) {
		char const id[sizeof(u32)];

		*((u32*) id) = __raw_readl(MMIO_P2V(P7_SYS_CHIP_ID));
		if (! memcmp(&id[0], "P70", sizeof("P70") - 1))
			rev = id[3] - '0';
		else
			/* This is not a Parrot7 chip. */
			panic("p7: not a Parrot7 chip\n");

		if (rev >= 0)
			pr_debug("p7: found Parrot7 chip v%d [0x%08x]\n",
			         rev + 1,
			         be32_to_cpu(*(u32*) id));
		else
			panic("p7: found invalid Parrot7 chip revision 0x%08x\n",
			      be32_to_cpu(*(u32*) id));
	}

	return rev;
}
