/**
 * linux/drivers/parrot/gpio/p7-gpio.c - Parrot7 GPIO pin controller driver
 *                                       implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    06-Apr-2012
 *
 * This file is released under the GPL
 *
 * TODO:
 *  - implement power management support
 *  - optional: implement driver removal once irq_domain deletion is supported
 *  - optional: implement a better GPIO <-> interrupt mapping scheme to prevent
 *              platforms from providing mandatory mappings (using
 *              irq_create_mapping() to perform dynamic mapping at gpio_to_irq()
 *              time would have the side effect of creating mappings each time
 *              gpio_export() is called, particularly when browsing sysfs
 *              tree...).
 */

#include <linux/gpio.h>
#include <asm/gpio.h>
#include <asm/mach/irq.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/sysfs.h>
#include <mach/irqs.h>
#include "p7-gpio.h"

#define P7_GPIO_FILTER

#define P7_GPIO_START_FILTER_IRQ (0x8)
#define P7_GPIO_PORT_DATA_BASE  (0x0)
#define P7_GPIO_PORT_DIR_BASE   (0x7000)
#define P7_GPIO_STATUS          (0x7400)
#define P7_GPIO_DEBOUNCE        (0x7404)
#define P7_GPIO_PHASE_IT        (0x7420)
#define P7_GPIO_PHASE           (0x7600)
#define P7_GPIO_PHASE_FROM_IRQ(IRQ)	((IRQ - P7_GPIO_START_FILTER_IRQ) / 2)
#define P7_GPIO_PHASE_VAL(PHASE)	(P7_GPIO_PHASE + (PHASE * 0x20))
#define P7_GPIO_PHASE_ITEN(PHASE)	(P7_GPIO_PHASE + (PHASE * 0x20) + 0x4)
#define P7_GPIO_PHASE_ITACK(PHASE)	(P7_GPIO_PHASE + (PHASE * 0x20) + 0x8)
#define P7_GPIO_PHASE_IFILTER(PHASE)	(P7_GPIO_PHASE + (PHASE * 0x20) + 0xC)
#define P7_GPIO_INTC_BASE       (0x7800)

#define P7_GPIO_INTC_POSEDGE    (0 << 16)
#define P7_GPIO_INTC_ALLEDGE    (1 << 16)
#define P7_GPIO_INTC_NEGEDGE    (2 << 16)
#define P7_GPIO_INTC_LEVEL      (3 << 16)
#define P7_GPIO_INTC_HIGH       (P7_GPIO_INTC_LEVEL | (0 << 18))
#define P7_GPIO_INTC_LOW        (P7_GPIO_INTC_LEVEL | (1 << 18))
#define P7_GPIO_INTC_TYPE_MASK  (7 << 16)
#define P7_GPIO_INTC_ITCLR      (1 << 19)
#define P7_GPIO_INTC_ITEN       (1 << 20)
#define P7_GPIO_INTC_DEBOUNCE   (1 << 21)
#define P7_GPIO_INTC_FILTER_MODE   (1 << 22)
#define P7_GPIO_INTC_ANA_MEASURE   (1 << 23)
#define P7_GPIO_INTC_PHASE_MEASURE (1 << 24)
#define P7_GPIO_ITFILTER_MODE_POS 24

#define P7_GPIO_STATUS_IRQ_MASK ((1 << P7_GPIO_IRQS) - 1)
#define P7_GPIO_STATUS_PHASE    (1 << 22)

#define P7_GPIO_INVAL   -1

struct gpio_irq {
	int gpio;
	u8  filter;
	u32 measure;
	struct timespec ts;
};

struct p7gpio {
	unsigned long                   base;
	struct resource                 *ioarea;
	struct clk                      *clk;
	int                             irq;
	struct resource                 *mux_irqs;
	struct gpio_chip                chip;
	spinlock_t                      lock;
	const struct p7gpio_plat_data   *pdata;
	struct gpio_irq                 irq_map[P7_GPIO_IRQS];
	struct irq_domain               *irq_domain;
#ifdef CONFIG_PM_SLEEP
	u32                             *saved_regs;
#endif
};

/*
 * GPIO chip callbacks
 */

static int p7gpio_get(struct gpio_chip *c, unsigned gpio)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	u32             port    = p7gpio->base + P7_GPIO_PORT_DATA_BASE;

	/* Access to the data register is a bit peculiar: each port contains 8
	 * GPIOs and can be accessed through 255 different addresses. The bits 2
	 * to 10 of these addresses is actually a mask applied to the value
	 * present in this register (bits [1:0] are not part of the mask and are
	 * always 0).
	 */

	/* first we compute the base address for the port containing this
	 * GPIO */
	port += (gpio / 8) * 0x400;
	/* Then we add the mask */
	port |= 1U << ((gpio % 8) + 2);

	/* since all the other GPIOs in this port are masked, we don't have to
	 * check for the specific bit */
	return !!(__raw_readl(port) & 0xff);
}

static void p7gpio_set(struct gpio_chip *c, unsigned gpio, int v)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	u32             port    = p7gpio->base + P7_GPIO_PORT_DATA_BASE;

	port += (gpio / 8) * 0x400;
	port |= 1U << ((gpio % 8) + 2);

	/* since all the other GPIOs in this port are masked, we don't have to
	 * only set the specific bit */
	__raw_writel(v ? 0xff : 0x00, port);
}

/* spinlock is taken by caller */
static int p7gpio_direction_input(struct gpio_chip *c, unsigned gpio)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	u32             port    = p7gpio->base + P7_GPIO_PORT_DIR_BASE;
	u32             mask;

	/* There are 8 GPIOs per port */
	port += (gpio / 8) * 4;
	mask = ~(1U << (gpio % 8)) & 0xff;

	__raw_writel(__raw_readl(port) & mask, port);

	return 0;
}

/* spinlock is taken by caller */
static int p7gpio_direction_output(struct gpio_chip *c, unsigned gpio, int v)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	u32             port    = p7gpio->base + P7_GPIO_PORT_DIR_BASE;
	u32             mask;

	/* There are 8 GPIOs per port */
	port += (gpio / 8) * 4;
	mask = 1U << (gpio % 8);

	/* Set the GPIO value before we enable it. */
	p7gpio_set(c, gpio, v);

	__raw_writel((__raw_readl(port) | mask) & 0xff, port);

	return 0;
}

static int p7gpio_is_output(struct gpio_chip *c, unsigned gpio)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	u32             port    = p7gpio->base + P7_GPIO_PORT_DIR_BASE;
	u32             mask;

	/* There are 8 GPIOs per port */
	port += (gpio / 8) * 4;
	mask = 1U << (gpio % 8);

	return !!(__raw_readl(port) & mask);
}

/*
 * Recover the internal interrupt number for a gpio.
 */
static inline int p7gpio_to_hw_irq(struct p7gpio *p7gpio, unsigned gpio)
{
	unsigned i;
	unsigned long   flags;

	spin_lock_irqsave(&p7gpio->lock, flags);
	for (i = 0; i < P7_GPIO_IRQS; i++)
		if (p7gpio->irq_map[i].gpio == gpio)
			break;
	spin_unlock_irqrestore(&p7gpio->lock, flags);

	return i == P7_GPIO_IRQS ? -1: i;
}

/*
 * Configure the debounce delay.
 * d is the debounce delay in microseconds.
 *
 * Hardware limitations: only the first 8 interruptions can be debounced, and
 * the debouncing delay is global for all those GPIOs. This means that the
 * debounce value for *all* debounced GPIO will in fact be the latest non-zero
 * value given to this function.
 *
 * spinlock is taken by caller
 */
/* XXX this not working for p7r3 : we have a 8 bit prediv and per gpio debounce */
static int p7gpio_set_debounce(struct gpio_chip *c, unsigned gpio, unsigned d)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	u32             *addr   = (u32 *)(p7gpio->base + P7_GPIO_INTC_BASE);
	unsigned        hw_irq;

	hw_irq = p7gpio_to_hw_irq(p7gpio, gpio);
	if (hw_irq == -1)
		/* This GPIO is not mapped to any IRQ */
		return -EINVAL;

	/* Only the first 8 interrupts are debounce-capable */
	if (hw_irq >= 8)
		return -EINVAL;

	addr += hw_irq;

	if (d != 0) {
		unsigned long   clk_khz = clk_get_rate(p7gpio->clk) / 1000;
		u32             ticks;

		/* convert the delay in a number of ticks from the GPIO clock */
		ticks = d * clk_khz / 1000;

		/* debounce register is only 24 bit wide */
		if (ticks > 0xffffff || ticks == 0)
			return -EINVAL;

		/* reconfigure the global debounce delay with the new value. */
		__raw_writel(ticks, p7gpio->base + P7_GPIO_DEBOUNCE);

		/* enable debouncing on this interrupt */
		__raw_writel(__raw_readl(addr) | P7_GPIO_INTC_DEBOUNCE,
		               addr);
	} else {
		/* if delay is 0 we simply remove the debounce flag from the
		 * interrupt register and leave the debounce value untouched. */
		__raw_writel(__raw_readl(addr) & ~P7_GPIO_INTC_DEBOUNCE,
		               addr);
	}

	return 0;
}

static int p7gpio_to_irq(struct gpio_chip *c, unsigned gpio)
{
	struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
	unsigned        hw_irq;

	hw_irq = p7gpio_to_hw_irq(p7gpio, gpio);

	if (hw_irq == -1)
		return -ENXIO;

	return irq_linear_revmap(p7gpio->irq_domain, hw_irq);
}

#ifdef CONFIG_DEBUG_FS
#include <linux/seq_file.h>

static void p7gpio_dbg_show(struct seq_file *s, struct gpio_chip *c)
{
	unsigned	 i;
	const char	*label;

	for (i = 0; i < c->ngpio; i++) {
		label = gpiochip_is_requested(c, i);

		if (label) {
			int irq;

			seq_printf(s, " gpio-%-3d (%-20.20s) %s %s",
				   c->base + i, label,
				   p7gpio_is_output(c, i) ? "out" : "in ",
				   p7gpio_get(c, i) ? "hi" : "lo");

			irq = gpio_to_irq(c->base + i);
			if (irq >= 0)
				seq_printf(s, " [IRQ %d]", irq);

			seq_printf(s, "\n");
		}
	}
}

#define P7GPIO_DBG_SHOW p7gpio_dbg_show

#else   /* CONFIG_DEBUG_FS */

#define P7GPIO_DBG_SHOW NULL

#endif  /* CONFIG_DEBUG_FS */

#ifdef CONFIG_PINCTRL

#include <linux/pinctrl/consumer.h>

/* Request pinctrl subsystem to switch pin as gpio. */
static int p7gpio_request(struct gpio_chip *chip,
                          unsigned int offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

/* Tell pinctrl subsystem we are not using pin as a gpio anymore. */
static void p7gpio_free(struct gpio_chip *chip,
                        unsigned int offset)
{
	/* When a PAD is disabled it's put back into GPIO function. We have to
	 * make sure we're not driving anything. */
	p7gpio_direction_input(chip, offset);
	pinctrl_free_gpio(chip->base + offset);
}

#else   /* CONFIG_PINCTRL */

#define p7gpio_request  NULL
#define p7gpio_free     NULL

#endif  /* CONFIG_PINCTRL */

static const struct gpio_chip p7gpio_chip_defaults = {
	.request            = p7gpio_request,
	.free               = p7gpio_free,
	.get                = p7gpio_get,
	.set                = p7gpio_set,
	.direction_input    = p7gpio_direction_input,
	.direction_output   = p7gpio_direction_output,
	.set_debounce       = p7gpio_set_debounce,
	.to_irq             = p7gpio_to_irq,
	.dbg_show           = P7GPIO_DBG_SHOW,
	.base               = P7_FIRST_GPIO,
	.label              = P7GPIO_DRV_NAME,
	.can_sleep          = 0,  /* P7 GPIO functions are IRQ-safe */
};

/*
 * GPIO IRQ callbacks
 */


/*
 * p7gpio_filter_irq_reg: Enable, disable, ack filtered gpios
 */
static inline void p7gpio_filter_irq_ack(struct p7gpio *p7gpio, struct irq_data *d)
{
	unsigned long	flags;
	unsigned int	reg;
	u32		addr;
	int		phase;

	phase = P7_GPIO_PHASE_FROM_IRQ(d->hwirq);
	addr  = p7gpio->base + P7_GPIO_PHASE_ITACK(phase);
	/*
	 * even  hwirq means measure done ->
	 *		bit 0 of gpio_phasex_itack register
	 * odd hwirq means filter it available ->
	 *		bit 1 if gpio_phasex_itack register
	 */

	spin_lock_irqsave(&p7gpio->lock, flags);
	reg = __raw_readl(addr);
	reg |= 1 << d->hwirq % 2;
	__raw_writel(reg, addr);

	if (!(d->hwirq % 2)) {	/*measure done*/
		addr  = p7gpio->base + P7_GPIO_PHASE_VAL(phase);
		p7gpio->irq_map[d->hwirq + 1].measure = __raw_readl(addr);
		ktime_get_ts(&p7gpio->irq_map[d->hwirq + 1].ts);
	}

	spin_unlock_irqrestore(&p7gpio->lock, flags);
}

static inline void p7gpio_filter_irq_msk(struct p7gpio *p7gpio, struct irq_data *d, u8 val)
{
	unsigned long	flags;
	unsigned int	reg;
	u32		addr;
	int		phase;

	phase = P7_GPIO_PHASE_FROM_IRQ(d->hwirq);
	addr = p7gpio->base + P7_GPIO_PHASE_ITEN(phase);
	/*
	 * even  hwirq means measure done ->
	 *		bit 0 of gpio_phasex_itack register
	 * odd hwirq means filter it available ->
	 *		bit 1 if gpio_phasex_itack register
	 */

	spin_lock_irqsave(&p7gpio->lock, flags);
	reg = __raw_readl(addr);
	reg &= ~(val << (d->hwirq % 2));
	reg |= !!val << (d->hwirq % 2);
	__raw_writel(reg, addr);

	spin_unlock_irqrestore(&p7gpio->lock, flags);
}

static void p7gpio_irq_ack(struct irq_data *d)
{
	struct p7gpio   *p7gpio = irq_data_get_irq_chip_data(d);
	u32             *addr   = (u32 *)(p7gpio->base + P7_GPIO_INTC_BASE);
	unsigned long   flags;

	BUG_ON(d->hwirq >= P7_GPIO_IRQS);

	if (p7gpio->irq_map[d->hwirq].filter){
		p7gpio_filter_irq_ack(p7gpio, d);
		return;
	}
	addr += d->hwirq;

	spin_lock_irqsave(&p7gpio->lock, flags);
	/* in order to clear the interrupt, we have to write a 1 to INTCLR and
	 * then a 0 to re-enable the interrupt. */
	__raw_writel(__raw_readl(addr) |  P7_GPIO_INTC_ITCLR, addr);
	__raw_writel(__raw_readl(addr) & ~P7_GPIO_INTC_ITCLR, addr);
	spin_unlock_irqrestore(&p7gpio->lock, flags);
}

static void p7gpio_irq_unmask(struct irq_data *d)
{
	struct p7gpio   *p7gpio = irq_data_get_irq_chip_data(d);
	u32             *addr   = (u32 *)(p7gpio->base + P7_GPIO_INTC_BASE);
	unsigned long   flags;

	BUG_ON(d->hwirq >= P7_GPIO_IRQS);

	if (p7gpio->irq_map[d->hwirq].filter) {
		p7gpio_filter_irq_msk(p7gpio, d, 1);
		return;
	}

	addr += d->hwirq;

	spin_lock_irqsave(&p7gpio->lock, flags);
	__raw_writel(__raw_readl(addr) | P7_GPIO_INTC_ITEN, addr);
	spin_unlock_irqrestore(&p7gpio->lock, flags);
}

static void p7gpio_irq_mask(struct irq_data *d)
{
	struct p7gpio   *p7gpio = irq_data_get_irq_chip_data(d);
	u32             *addr;
	unsigned long   flags;

	BUG_ON(d->hwirq >= P7_GPIO_IRQS);
	BUG_ON(!p7gpio);

	if (p7gpio->irq_map[d->hwirq].filter) {
		p7gpio_filter_irq_msk(p7gpio, d, 0);
		return;
	}

	addr = (u32 *)(p7gpio->base + P7_GPIO_INTC_BASE) + d->hwirq;

	spin_lock_irqsave(&p7gpio->lock, flags);
	__raw_writel(__raw_readl(addr) & ~P7_GPIO_INTC_ITEN, addr);
	spin_unlock_irqrestore(&p7gpio->lock, flags);
}

static int p7gpio_irq_type(struct irq_data *d, unsigned trigger)
{
	struct p7gpio   *p7gpio = irq_data_get_irq_chip_data(d);
	u32             *addr   = (u32 *)(p7gpio->base + P7_GPIO_INTC_BASE);
	u32             val;
	unsigned long   flags;
	irq_flow_handler_t   irqf;

	BUG_ON(d->hwirq >= P7_GPIO_IRQS);

	addr += d->hwirq;

	switch (trigger & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_BOTH:
		val = P7_GPIO_INTC_ALLEDGE;
		irqf = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_RISING:
		val = P7_GPIO_INTC_POSEDGE;
		irqf = handle_edge_irq;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		val = P7_GPIO_INTC_NEGEDGE;
		irqf = handle_edge_irq;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		val = P7_GPIO_INTC_HIGH;
		irqf = handle_level_irq;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		val = P7_GPIO_INTC_LOW;
		irqf = handle_level_irq;
		break;
	default:
		/* uh? */
		return -EINVAL;
	}

	spin_lock_irqsave(&p7gpio->lock, flags);
	/* set the appropriate IRQ handler */
	__irq_set_handler_locked(d->irq, irqf);
	/* configure the GPIO controller's interrupt */
	__raw_writel((__raw_readl(addr) & ~P7_GPIO_INTC_TYPE_MASK) | val,
		       addr);

	spin_unlock_irqrestore(&p7gpio->lock, flags);

	return 0;
}

static inline void p7gpio_irq_phase_handler(struct p7gpio *p7gpio, u32 status)
{
	unsigned long   hw_irq = 8;
	u32             phase_status;

	if (status & P7_GPIO_STATUS_PHASE) {
		phase_status = __raw_readl(p7gpio->base + P7_GPIO_PHASE_IT);
	} else
		return;

	while (phase_status) {
		int     first_bit = ffs(phase_status);
		unsigned    virq;

		hw_irq += first_bit;
		phase_status >>= first_bit;

		virq = irq_linear_revmap(p7gpio->irq_domain, hw_irq - 1);
		generic_handle_irq(virq);
	}
}

static void p7gpio_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct p7gpio   *p7gpio = irq_desc_get_handler_data(desc);
	struct irq_chip *chip   = irq_desc_get_chip(desc);
	unsigned long   hw_irq = 0;
	u32             status;

	chained_irq_enter(chip, desc);

	status  = __raw_readl(p7gpio->base + P7_GPIO_STATUS);

	p7gpio_irq_phase_handler(p7gpio, status);

	status &= P7_GPIO_STATUS_IRQ_MASK;

	while (status) {
		int     first_bit = ffs(status);
		unsigned    virq;

		hw_irq += first_bit;
		status >>= first_bit;

		virq = irq_linear_revmap(p7gpio->irq_domain, hw_irq - 1);

		generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

static struct irq_chip p7gpio_irq_chip = {
	.name         = P7GPIO_DRV_NAME,
	.irq_ack      = p7gpio_irq_ack,
	.irq_unmask   = p7gpio_irq_unmask,
	.irq_mask     = p7gpio_irq_mask,
	.irq_set_type = p7gpio_irq_type,
};

/*
 * IRQ domain interface
 */

static int p7gpio_irq_domain_map(struct irq_domain *domain,
                                 unsigned int virq,
                                 irq_hw_number_t hw_irq)
{
	struct p7gpio *p7gpio = domain->host_data;

	/* Register the GPIO interrupt */
	irq_set_chip_data(virq, p7gpio);
	/* By default the interrupt triggers on POSEDGE */
	irq_set_chip_and_handler(virq, &p7gpio_irq_chip, handle_edge_irq);
	set_irq_flags(virq, IRQF_VALID);

	return 0;
}

static void p7gpio_irq_domain_unmap(struct irq_domain *domain,
                                    unsigned int virq)
{
	irq_set_chip(virq, NULL);
	set_irq_flags(virq, 0);
}

static struct irq_domain_ops p7gpio_irq_domain_ops = {
	.map    = p7gpio_irq_domain_map,
	.unmap  = p7gpio_irq_domain_unmap,
};

/*
 * GPIO <=> IRQ mapping
 *
 * The P7 GPIO Controller can monitor up to 20 interruption conditions. Each one
 * of these interrupts can be mapped to any single GPIO.
 *
 * So for instance we can have GPIO interrupt 2 mapped to the falling edge of
 * GPIO 40, and interrupt 5 mapped to a high level of gpio 8, or any other
 * combination. This mapping is static and should be declared in the
 * platform_data. In this fictional case it'd give us:
 *
 * static struct p7gpio_irq_map my_gpio_irq_map[] = {
 *    { .hw_irq = 2, .gpio = 40 },
 *    { .hw_irq = 5, .gpio = 8  },
 * };
 *
 * The linux gpiolib interface doesn't let us change this mapping at runtime,
 * but the gpio_to_irq function can be used to find the IRQ mapped to a GPIO (if
 * it exists).
 */
static int p7gpio_interrupt_register(struct p7gpio *p7gpio, unsigned gpio)
{
	unsigned        hw_irq;
	unsigned long   flags;
	int             ret = 0;
	u32             addr    = p7gpio->base + P7_GPIO_INTC_BASE;
	u32             reg;

	if (gpio >= p7gpio->chip.ngpio) {
		return -EINVAL;
	}

	/* check if it is already mapped */
	if (p7gpio_to_hw_irq(p7gpio, gpio) >= 0)
		return 0;

	spin_lock_irqsave(&p7gpio->lock, flags);
	for (hw_irq = 0; hw_irq < P7_GPIO_IRQS; hw_irq++) {
		if (p7gpio->irq_map[hw_irq].gpio == P7_GPIO_INVAL)
			break;
	}
	if (hw_irq == P7_GPIO_IRQS) {
		ret = -EBUSY;
		spin_unlock_irqrestore(&p7gpio->lock, flags);
		goto undo;
	}

	/* update the IRQ <=> GPIO mapping */
	p7gpio->irq_map[hw_irq].gpio = gpio;
	p7gpio->irq_map[hw_irq].filter = 0;

	/* Compute the address of the interrupt register */
	addr += hw_irq * 4;

	/* configure the interrupt in the GPIO controller and clear any
	 * pending IT */
	reg = (__raw_readl(addr) & ~0xff) | gpio | P7_GPIO_INTC_ITCLR;
	__raw_writel(reg, addr);

	/* We have to put the ITCLR bit back to 0 (see
	 * p7gpio_irq_ack above) */
	reg &= ~P7_GPIO_INTC_ITCLR;
	__raw_writel(reg, addr);

	spin_unlock_irqrestore(&p7gpio->lock, flags);

	/* Create the mapping for this interrupt in our irq_domain */
	if (irq_create_mapping(p7gpio->irq_domain, hw_irq) == 0) {
		ret = -EINVAL;
		goto undo;
	}
	return 0;

undo:
	return ret;
}

static void p7gpio_interrupt_free(struct p7gpio *p7gpio, unsigned gpio)
{
	unsigned        hw_irq;
	unsigned long   flags;

	if (gpio >= p7gpio->chip.ngpio) {
		return;
	}

	spin_lock_irqsave(&p7gpio->lock, flags);
	for (hw_irq = 0; hw_irq < P7_GPIO_IRQS; hw_irq++) {
		if (p7gpio->irq_map[hw_irq].gpio == gpio) {
			p7gpio->irq_map[hw_irq].gpio = P7_GPIO_INVAL;
			irq_dispose_mapping(irq_find_mapping(
					p7gpio->irq_domain, hw_irq));
			break;
		}
	}

	spin_unlock_irqrestore(&p7gpio->lock, flags);
}

static int p7gpio_match_chip (struct gpio_chip *chip, const void *data)
{
	return !strcmp(chip->label, P7GPIO_DRV_NAME);
}

int p7_gpio_interrupt_register(unsigned gpio)
{
	/* we use gpiochip_find instead of gpio_to_chip because it take a lock
	   and can be used on a gpio that is not requested
	 */
	struct gpio_chip *c = gpiochip_find(NULL, p7gpio_match_chip);
	if (c) {
		struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
		if (gpio >= c->base && gpio < c->base + c->ngpio)
			return p7gpio_interrupt_register(p7gpio, gpio - c->base);
	}
	return -EINVAL;
}

#ifdef P7_GPIO_FILTER
static inline void p7gpio_interrupt_register_gpio(struct p7gpio *p7gpio,
		 int gpio, u8 filter, s8 mode, unsigned hw_irq)
{
	u32	addr_intC;
	u32	intC = 0;

	addr_intC = p7gpio->base + P7_GPIO_INTC_BASE + (hw_irq * 4);

	/* read regs */
	intC = (__raw_readl(addr_intC) &  P7_GPIO_INTC_TYPE_MASK);

	/* set gpio */
	intC |= (gpio & 0xff);

	/* mode */
	if (mode != GPIO_NO_MEASURE) {
		intC |= P7_GPIO_INTC_PHASE_MEASURE;
		__raw_writel(intC, addr_intC);
	}

	p7gpio->irq_map[hw_irq].gpio = gpio;
	p7gpio->irq_map[hw_irq].filter = filter;
}

static inline void p7gpio_interrupt_configure(struct p7gpio *p7gpio,
		int phase, u8 filter, s8 mode)
{
	u32	addr_itFilter, addr_itEn;

	/* regs to write */
	u32	itFilter = 0;
	u32	itEn = 0;

	addr_itFilter = p7gpio->base + P7_GPIO_PHASE_IFILTER(phase);
	addr_itEn = p7gpio->base + P7_GPIO_PHASE_ITEN(phase);

	// itEn no
	itFilter = __raw_readl(addr_itFilter);

	/* mode */
	if (mode != GPIO_NO_MEASURE) {
		itFilter |= (mode & 0x3) << P7_GPIO_ITFILTER_MODE_POS;
		itEn = 1 << 0;
	}

	/* filter */
	if (filter) {
		itFilter |= (filter & 0xff);
		itEn |= 1 << 1;
	}

	__raw_writel(itFilter, addr_itFilter);
	__raw_writel(itEn, addr_itEn);
}

static int p7gpio_measure_interrupt_register(struct p7gpio *p7gpio,
			unsigned gpio1, unsigned gpio2, u16 filter, s8 mode)
{
	unsigned        hw_irq;
	unsigned long   flags;
	int             ret = 0;
	int             phase;

	if (gpio1 >= p7gpio->chip.ngpio) {
		return -EINVAL;
	}

	/* check if it is already mapped */
	if (p7gpio_to_hw_irq(p7gpio, gpio1) >= 0)
		return 0;


	spin_lock_irqsave(&p7gpio->lock, flags);

	/* If gpio2 is set, it means that phase measure is enabled.
	 * If phase measure is enabled, it should be associated to irq range [8-19]
	 * Phase measure uses two consecutives irqs.
	 */
	for (hw_irq = 8; hw_irq < P7_GPIO_IRQS; hw_irq+=2) {
		if ((p7gpio->irq_map[hw_irq].gpio == P7_GPIO_INVAL) &&
			(p7gpio->irq_map[hw_irq+1].gpio == P7_GPIO_INVAL))
			break;
	}

	if (hw_irq == P7_GPIO_IRQS)
		return -EBUSY;

	p7gpio_interrupt_register_gpio(p7gpio, gpio1,
					filter, mode, hw_irq);

	p7gpio_interrupt_register_gpio(p7gpio, gpio2,
			filter, mode, hw_irq + 1);

	phase = P7_GPIO_PHASE_FROM_IRQ(hw_irq);
	p7gpio_interrupt_configure(p7gpio, phase, filter, mode);

	spin_unlock_irqrestore(&p7gpio->lock, flags);

	/* Create the mapping for this interrupt in our irq_domain */
	if (irq_create_mapping(p7gpio->irq_domain, hw_irq) == 0) {
		ret = -EINVAL;
	}
	/* Create the mapping for this interrupt in our irq_domain */
	if (irq_create_mapping(p7gpio->irq_domain, hw_irq+1) == 0) {
		ret = -EINVAL;
	}

	return ret;
}

/*
 * This function is called instead of p7gpio_interrupt_register when need to
 * register filtred gpios
 */
static int p7gpio_filter_interrupt_register(struct p7gpio *p7gpio,
			unsigned gpio1, u16 filter, s8 mode)
{
	unsigned        hw_irq;
	unsigned long   flags;
	int             ret = 0;
	int             phase;

	if (gpio1 >= p7gpio->chip.ngpio) {
		return -EINVAL;
	}

	/* check if it is already mapped */
	if (p7gpio_to_hw_irq(p7gpio, gpio1) >= 0)
		return 0;

	spin_lock_irqsave(&p7gpio->lock, flags);

	/* If phase measure is enabled, it should be associated to irq range [8-19]
	 * Phase measure uses two consecutives irqs.
	 */

	for (hw_irq = 8; hw_irq < P7_GPIO_IRQS; hw_irq++) {
		if (p7gpio->irq_map[hw_irq].gpio == P7_GPIO_INVAL)
			break;
	}

	if (hw_irq == P7_GPIO_IRQS)
		return -EBUSY;

	p7gpio_interrupt_register_gpio(p7gpio, gpio1,
					filter, mode, hw_irq);

	phase = P7_GPIO_PHASE_FROM_IRQ(hw_irq);
	p7gpio_interrupt_configure(p7gpio, phase, filter, mode);

	spin_unlock_irqrestore(&p7gpio->lock, flags);

	if (irq_create_mapping(p7gpio->irq_domain, hw_irq) == 0) {
		ret = -EINVAL;
	}

	return ret;
}

static int p7gpio_sysfs_get_hwirq(struct p7gpio *p7gpio, const char *str_gpio)
{
	int hw_irq;
	int gpio;

	gpio = simple_strtoul(str_gpio, NULL, 10);
	for (hw_irq = 0; hw_irq < P7_GPIO_IRQS; hw_irq++) {
		if (p7gpio->irq_map[hw_irq].gpio == gpio)
			break;
	}

	/*if not reach end of table*/
	if (hw_irq == P7_GPIO_IRQS)
		return -1;

	return hw_irq;
}

static ssize_t p7gpio_sysfs_reset_show(struct device *device,
				       struct device_attribute *attr,
				       char *buf)
{
	struct p7gpio *p7gpio = dev_get_drvdata(device);
	int            hw_irq;

	/* Attribute name format is reset_%d */
	hw_irq = p7gpio_sysfs_get_hwirq(p7gpio, attr->attr.name + 6);
	if (hw_irq < 0)
		return 0;

	return snprintf(buf,PAGE_SIZE, "%d", p7gpio->irq_map[hw_irq].filter);
}

static ssize_t p7gpio_sysfs_reset_store(struct device *device,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
        struct p7gpio           *p7gpio = dev_get_drvdata(device);
        int                     hw_irq;
        u32                     addr_itFilter;
        u32                     addr_itAck;
        u32                     itFilter = 0;
        u32                     itAck = 0;
        int                     phase;
        unsigned long           flags;

	/* Fetch HW IRQ */
	hw_irq = p7gpio_sysfs_get_hwirq(p7gpio, attr->attr.name + 6);
	if (hw_irq < 0)
		return count;

	phase = P7_GPIO_PHASE_FROM_IRQ(hw_irq);
	addr_itFilter = p7gpio->base + P7_GPIO_PHASE_IFILTER(phase);
	addr_itAck = p7gpio->base + P7_GPIO_PHASE_ITACK(phase);

	spin_lock_irqsave(&p7gpio->lock, flags);

	/* Ack any pending interrupt */
	itAck = __raw_readl(addr_itAck);
	itAck |= 1 << (hw_irq % 2);
	__raw_writel(itAck, addr_itAck);

	/* Reset filter counter */
	itFilter = __raw_readl(addr_itFilter);
	__raw_writel(itFilter, addr_itFilter);

	spin_unlock_irqrestore(&p7gpio->lock, flags);

	return count;
}

static struct device_attribute p7gpio_sysfs_reset_attr = {
	.attr = { .mode = S_IRUGO | S_IWUSR },
	.show = &p7gpio_sysfs_reset_show,
	.store = &p7gpio_sysfs_reset_store
};

static int p7_gpio_filter_get_reset_gpio(struct p7gpio_filter_phase *data,
		u32 *gpio)
{
	int err = 0;

	/* Find GPIO to export for reset */
	switch (data->mode) {
	case GPIO_MEASURE_START:
	case GPIO_MEASURE_START_AFTER_STOP:
		*gpio = data->start;
		break;

	case GPIO_MEASURE_STOP:
	case GPIO_MEASURE_STOP_AFTER_START:
		*gpio = data->stop;
		break;

	default:
		err = -EINVAL;
		break;
	}

	return err;
}

static int p7_gpio_filter_interrupt_export_reset(
		struct p7gpio_filter_phase *data)
{
	int err;
	u32 gpio;
	const int len = 20;
	char *name;
	struct gpio_chip *c;
	struct p7gpio *p7gpio;

	c = gpiochip_find(NULL, p7gpio_match_chip);
	if (!c)
		return -EINVAL;

	p7gpio = container_of(c, struct p7gpio, chip);

	err = p7_gpio_filter_get_reset_gpio(data, &gpio);
	if (err < 0)
		return err;

	name = kzalloc(len, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	snprintf(name, len, "reset_%d", gpio);
	p7gpio_sysfs_reset_attr.attr.name = name;

	err = device_create_file(p7gpio->chip.dev, &p7gpio_sysfs_reset_attr);
	if (err)
		printk("Erreur de creation de device file\n");

	return 0;
}

static ssize_t p7gpio_sysfs_show(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct p7gpio *p7gpio = dev_get_drvdata(device);
	int            hw_irq;

	/* Attribute name format is phase_%d */
	hw_irq = p7gpio_sysfs_get_hwirq(p7gpio, attr->attr.name + 6);
	if (hw_irq < 0)
		return 0;

	return snprintf(buf, PAGE_SIZE, "0x%08x:%16lld",
			p7gpio->irq_map[hw_irq].measure,
			timespec_to_ns(&p7gpio->irq_map[hw_irq].ts));
}

static ssize_t p7gpio_sysfs_store(struct device *device,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
	u32 reg, err = 0;
	err = kstrtoul(buf, 0, (unsigned long*) &reg);
	return err;
}

static struct device_attribute p7gpio_sysfs_attr = {
    .attr = {
          .mode = S_IRUGO | S_IWUSR},
     .show = &p7gpio_sysfs_show,
     .store = &p7gpio_sysfs_store
};

/*DEVICE_ATTR(phase, 0644, p7gpio_sysfs_show, p7gpio_sysfs_store);*/

/* If filterring is performed on only one gpio,
 * leave start or stop field with 0.
 */
int p7_gpio_filter_interrupt_register(struct p7gpio_filter_phase data)
{
	int err = 0;

	char *name1, *name2;
	const int len = 20;
	struct gpio_chip *c = gpiochip_find(NULL, p7gpio_match_chip);


	if (c) {
		struct p7gpio   *p7gpio = container_of(c, struct p7gpio, chip);
		name1 = (char *) kzalloc(len, GFP_KERNEL);
		if (name1 == NULL) {
			return -ENOMEM;
		}

		/*FIXME kfree is never performed! Built-in driver*/
		snprintf(name1, len, "phase_%d", data.start);
		p7gpio_sysfs_attr.attr.name = name1;
		err = device_create_file(p7gpio->chip.dev, &p7gpio_sysfs_attr);
		if (err){
			printk("Erreur de creation de device file\n");
		}
		if (data.stop){
			name2 = (char *) kzalloc(len, GFP_KERNEL);
			if (name2 == NULL) {
				return -ENOMEM;
			}

			snprintf(name2, len, "phase_%d", data.stop);
			p7gpio_sysfs_attr.attr.name = name2;
			err = device_create_file(p7gpio->chip.dev, &p7gpio_sysfs_attr);
			if (err){
				printk("Erreur de creation de device file\n");
			}
		}

		/* Test gpio range */
		if (data.start >= c->base && data.start < c->base + c->ngpio){
			if((data.stop) &&
			  (data.stop >= c->base && data.stop < c->base + c->ngpio)){
				err = p7gpio_measure_interrupt_register(
						p7gpio,
						data.start - c->base,
						data.stop - c->base,
						data.filter, data.mode);
			} else {
				err = p7gpio_filter_interrupt_register(
						p7gpio,
						data.start - c->base,
						data.filter, data.mode);
			}

			if (err < 0)
				return err;

			if (data.export_reset)
				return p7_gpio_filter_interrupt_export_reset(
						&data);
			else
				return 0;
		}
	}
	return -EINVAL;
}
#endif

static void p7gpio_unmap_irqs(struct p7gpio *p7gpio)
{
	unsigned        hw_irq;

	BUG_ON(p7gpio->pdata->irq_gpios_sz > P7_GPIO_IRQS);

	for (hw_irq = 0; hw_irq < p7gpio->pdata->irq_gpios_sz; hw_irq++) {
		unsigned long   gpio    = p7gpio->pdata->irq_gpios[hw_irq];

		p7gpio_interrupt_free(p7gpio, gpio);
	}
}

static int p7gpio_map_irqs(struct p7gpio *p7gpio)
{
	unsigned        hw_irq;
	int             ret = 0;

	if (p7gpio->pdata->irq_gpios_sz > P7_GPIO_IRQS)
		/* We can't have that many GPIOs as interrupt */
		return -ERANGE;

	for (hw_irq = 0; hw_irq < p7gpio->pdata->irq_gpios_sz; hw_irq++) {
		unsigned long   gpio    = p7gpio->pdata->irq_gpios[hw_irq];

		ret = p7gpio_interrupt_register(p7gpio, gpio);
		if (ret < 0) {
			goto undo;
		}
	}

	return 0;

undo:
	/* Undo any partial initialization */
	p7gpio_unmap_irqs(p7gpio);
	return ret;
}

static int __devinit p7gpio_probe(struct platform_device *pdev)
{
	struct p7gpio   *p7gpio;
	struct resource *res;
	int             ret;
	unsigned        i;
	unsigned int num_saved_regs __maybe_unused;

	p7gpio = kzalloc(sizeof(*p7gpio), GFP_KERNEL);
	if (!p7gpio) {
		ret = -ENOMEM;
		goto exit;
	}

	spin_lock_init(&p7gpio->lock);

	p7gpio->pdata = dev_get_platdata(&pdev->dev);
	if (!p7gpio->pdata) {
		ret = -ENODEV;
		goto kfree;
	}

	p7gpio->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(p7gpio->clk)) {
		ret = PTR_ERR(p7gpio->clk);
		goto kfree;
	}

	ret = clk_prepare_enable(p7gpio->clk);
	if (ret)
		goto put_clock;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENOENT;
		goto stop_clock;
	}

	p7gpio->ioarea = request_mem_region(res->start,
	                                    resource_size(res),
	                                    pdev->name);
	if (!p7gpio->ioarea) {
		ret = -EBUSY;
		goto stop_clock;
	}

	p7gpio->base = (unsigned long)ioremap(res->start, resource_size(res));
	if (!p7gpio->base) {
		ret = -EBUSY;
		goto release_region;
	}

	/* Register the GPIO chip callbacks */
	p7gpio->chip        = p7gpio_chip_defaults;
	p7gpio->chip.ngpio  = p7gpio->pdata->gpio_nr;
	p7gpio->chip.dev    = &pdev->dev;

	for (i = 0; i < P7_GPIO_IRQS; i++) {
		p7gpio->irq_map[i].gpio = P7_GPIO_INVAL;
		p7gpio->irq_map[i].filter = 0;
	}

	/* XXX this should be done last
	   in theory, user can try to use irq before we finish
	 */
	ret = gpiochip_add(&p7gpio->chip);
	if (ret)
		goto unmap;

	p7gpio->irq = platform_get_irq(pdev, 0);

	if (p7gpio->irq < 0) {
		ret = -ENOENT;
		goto gpio_remove;
	}

	p7gpio->irq_domain =
		irq_domain_add_linear(NULL,
				P7_GPIO_IRQS,
				&p7gpio_irq_domain_ops,
				p7gpio);

	if (!p7gpio->irq_domain) {
		ret = -ENOMEM;
		goto gpio_remove;
	}

	/* p7gpio->irq is the only IRQ line going out of the GPIO
	 * controller. The per-GPIO interrupt will be triggered in
	 * software from this IRQ handler. */
	irq_set_chained_handler(p7gpio->irq, p7gpio_irq_handler);
	irq_set_handler_data(p7gpio->irq, p7gpio);

	ret = p7gpio_map_irqs(p7gpio);
	if (ret)
		goto remove_irq;

#ifdef CONFIG_PM_SLEEP
	num_saved_regs = 2 * roundup(p7gpio->chip.ngpio, 8) +
	                 P7_GPIO_IRQS + 1; /* 1 is for debounce reg */

	p7gpio->saved_regs = kmalloc(num_saved_regs * sizeof(u32), GFP_KERNEL);
	if (! p7gpio->saved_regs) {
		dev_err(&pdev->dev, "Failed to allocate memory"
		        "to save registers\n");
		ret = -ENOMEM;
		goto unmap_irqs;
	}
#endif

	platform_set_drvdata(pdev, p7gpio);

	dev_info(&pdev->dev, "GPIO driver initialized\n");

	return 0;

#ifdef CONFIG_PM_SLEEP
unmap_irqs:
	p7gpio_unmap_irqs(p7gpio);
#endif
remove_irq:
	irq_set_chained_handler(p7gpio->irq, NULL);
gpio_remove:
	/* this shouldn't fail since the gpiochip shouldn't be busy by this
	 * point */
	BUG_ON(gpiochip_remove(&p7gpio->chip));
unmap:
	iounmap((void *)p7gpio->base);
release_region:
	release_mem_region(p7gpio->ioarea->start, resource_size(p7gpio->ioarea));
stop_clock:
	clk_disable_unprepare(p7gpio->clk);
put_clock:
	clk_put(p7gpio->clk);
kfree:
	kfree(p7gpio);
exit:
	dev_info(&pdev->dev, "Init failed: %d\n", ret);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int p7gpio_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct p7gpio          *p7gpio = platform_get_drvdata(pdev);
	struct gpio_chip       *chip = &p7gpio->chip;
	int i, j = 0;

	for (i = 0; i < roundup(chip->ngpio, 8); i++) {
		u32 data_port = p7gpio->base + P7_GPIO_PORT_DATA_BASE;
		u32 dir_port = p7gpio->base + P7_GPIO_PORT_DIR_BASE;

		data_port += (i / 8) * 0x400;
		data_port |= 0xffU << 2; /* read 8 bits at once */

		dir_port += i * 4;

		p7gpio->saved_regs[j++] = __raw_readl(data_port);
		p7gpio->saved_regs[j++] = __raw_readl(dir_port);
	}

	for (i = 0; i < P7_GPIO_IRQS; i++) {
		u32 port = p7gpio->base + P7_GPIO_INTC_BASE;
		port += i * 4;
		p7gpio->saved_regs[j++] = __raw_readl(port);
	}

	p7gpio->saved_regs[j++] = __raw_readl(p7gpio->base + P7_GPIO_DEBOUNCE);

	clk_disable_unprepare(p7gpio->clk);

	return 0;
}

static int p7gpio_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct p7gpio          *p7gpio = platform_get_drvdata(pdev);
	struct gpio_chip       *chip = &p7gpio->chip;
	int i, j = 0;

	clk_prepare_enable(p7gpio->clk);

	for (i = 0; i < roundup(chip->ngpio, 8); i++) {
		u32 data_port = p7gpio->base + P7_GPIO_PORT_DATA_BASE;
		u32 dir_port = p7gpio->base + P7_GPIO_PORT_DIR_BASE;

		data_port += (i / 8) * 0x400;
		data_port |= 0xff << 2;

		dir_port += i * 4;

		__raw_writel(p7gpio->saved_regs[j++], data_port);
		__raw_writel(p7gpio->saved_regs[j++], dir_port);
	}

	for (i = 0; i < P7_GPIO_IRQS; i++) {
		u32 port = p7gpio->base + P7_GPIO_INTC_BASE;
		port += i * 4;
		__raw_writel(p7gpio->saved_regs[j++], port);
	}

	__raw_writel(p7gpio->saved_regs[j++], p7gpio->base + P7_GPIO_DEBOUNCE);

	return 0;
}
#else
#define p7gpio_suspend_noirq    NULL
#define p7gpio_resume_noirq     NULL
#endif

static struct dev_pm_ops p7gpio_dev_pm_ops = {
	.suspend_noirq  = p7gpio_suspend_noirq,
	.resume_noirq   = p7gpio_resume_noirq,
};

static struct platform_driver p7gpio_driver = {
	.driver = {
		.name   = P7GPIO_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = &p7gpio_dev_pm_ops,
	},
	.probe  = p7gpio_probe,
};

static int __init p7gpio_init(void)
{
	return platform_driver_register(&p7gpio_driver);
}
postcore_initcall(p7gpio_init);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("P7 GPIO controller driver");
MODULE_LICENSE("GPL");
