/**
 * linux/drivers/parrot/mfd/p7mu.c - Parrot7 power management unit core driver
 *                                   implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    10-May-2012
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/ratelimit.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/pm.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/mfd/core.h>
#include <linux/fs.h>
#include <asm/system_misc.h>
#include <asm/proc-fns.h>
#include <asm/smp_plat.h>
#include "p7mu.h"
#include "p7mu-pin.h"
#include "p7mu-clk.h"

/*
 * Configuration block registers
 */
#define P7MU_CFG_DCDC0_CFG_0_REG	(P7MU_CFG_PAGE + 0x00)
#define P7MU_CFG_DCDC0_CFG_1_REG	(P7MU_CFG_PAGE + 0x01)
#define P7MU_CFG_DCDC1_CFG_0_REG	(P7MU_CFG_PAGE + 0x02)
#define P7MU_CFG_DCDC1_CFG_1_REG	(P7MU_CFG_PAGE + 0x03)
#define P7MU_CFG_DCDC2_CFG_0_REG	(P7MU_CFG_PAGE + 0x04)
#define P7MU_CFG_DCDC2_CFG_1_REG	(P7MU_CFG_PAGE + 0x05)
#define P7MU_CFG_DCDC3_CFG_0_REG	(P7MU_CFG_PAGE + 0x06)
#define P7MU_CFG_DCDC3_CFG_1_REG	(P7MU_CFG_PAGE + 0x07)
#define P7MU_CFG_DCDC_PGATE_DEL_REG (P7MU_CFG_PAGE + 0x08)
#define P7MU_CFG_DCDC_NGATE_DEL_REG (P7MU_CFG_PAGE + 0x09)
#define P7MU_CFG_LDOS_CFG_REG		(P7MU_CFG_PAGE + 0x0a)
#define P7MU_CFG_TS_CAL_REG			(P7MU_CFG_PAGE + 0x0d)
#define P7MU_CFG_TS_POINT_0_REG		(P7MU_CFG_PAGE + 0x0e)
#define P7MU_CFG_TS_POINT_1_REG		(P7MU_CFG_PAGE + 0x0f)
#define P7MU_CFG_CORE_CFG_REG		(P7MU_CFG_PAGE + 0x10)
#define P7MU_CFG_TEST_MUX0_REG		(P7MU_CFG_PAGE + 0x11)
#define P7MU_CFG_TEST_MUX1_REG		(P7MU_CFG_PAGE + 0x12)
#define P7MU_CFG_TEST_MUX2_REG		(P7MU_CFG_PAGE + 0x13)
#define P7MU_CFG_TEST_MUX3_REG		(P7MU_CFG_PAGE + 0x14)
#define P7MU_CFG_TEST_MUX4_REG		(P7MU_CFG_PAGE + 0x15)
#define P7MU_CFG_TEST_MUX5_REG		(P7MU_CFG_PAGE + 0x16)
#define P7MU_CFG_TEST_MUX6_REG		(P7MU_CFG_PAGE + 0x17)
#define P7MU_CFG_TEST_MUX7_REG		(P7MU_CFG_PAGE + 0x18)
#define P7MU_CFG_TEST_DDR_CFG_REG	(P7MU_CFG_PAGE + 0x19)
#define P7MU_CFG_ID_0_REG			(P7MU_CFG_PAGE + 0x1a)
#define P7MU_CFG_ID_1_REG			(P7MU_CFG_PAGE + 0x1b)
#define P7MU_CFG_ID_2_REG			(P7MU_CFG_PAGE + 0x1c)
#define P7MU_CFG_ID_3_REG			(P7MU_CFG_PAGE + 0x1d)

#define P7MU_CFG_ID                 P7MU_CFG_ID_0_REG

/*
 * Power Management Control block registers
 */
#define P7MU_PMC_PAGE                       ((u16) 0x0100)
#define P7MU_PMC_POWER_SUPPLY_CTRL_REG		((u16) P7MU_PMC_PAGE + 0x00)
#define P7MU_PMC_GPIO_EVENT_SEL_REG			((u16) P7MU_PMC_PAGE + 0x01)
#define P7MU_PMC_GPIO_REG					((u16) P7MU_PMC_PAGE + 0x02)
#define P7MU_PMC_INTERRUPT_0_REG			((u16) P7MU_PMC_PAGE + 0x03)
#define P7MU_PMC_INTERRUPT_1_REG			((u16) P7MU_PMC_PAGE + 0x04)
#define P7MU_PMC_INTERRUPT_EN_0_REG			((u16) P7MU_PMC_PAGE + 0x05)
#define P7MU_PMC_INTERRUPT_EN_1_REG			((u16) P7MU_PMC_PAGE + 0x06)
#define P7MU_PMC_CMD_REG					((u16) P7MU_PMC_PAGE + 0x07)
#define P7MU_PMC_DEBUG_REG					((u16) P7MU_PMC_PAGE + 0x08)
#define P7MU_PMC_STATUS_REG					((u16) P7MU_PMC_PAGE + 0x09)
#define P7MU_PMC_TEMP_ERR_CNT_REG			((u16) P7MU_PMC_PAGE + 0x0a)
#define P7MU_PMC_TEMP_ERR_CTRL_REG			((u16) P7MU_PMC_PAGE + 0x0b)
#define P7MU_PMC_TEMP_ERR_TIME_REG			((u16) P7MU_PMC_PAGE + 0x0c)
#define P7MU_PMC_POWER_SUPPLY_MASK_REG		((u16) P7MU_PMC_PAGE + 0x0d)
#define P7MU_PMC_POWER_SUPPLY_FILTER_REG	((u16) P7MU_PMC_PAGE + 0x0e)
#define P7MU_PMC_POWER_SUPPLY_STATUS_REG	((u16) P7MU_PMC_PAGE + 0x0f)
#define P7MU_PMC_POWER_SUPPLY_CTRL_IDLE_REG ((u16) P7MU_PMC_PAGE + 0x10)

#define P7MU_CMD_SHUTDOWN	((u16) (1U << 0))
#define P7MU_CMD_STANDBY	((u16) (1U << 1))
#define P7MU_CMD_REBOOT		((u16) (1U << 2))
#define P7MU_CMD_RESET		((u16) (1U << 3))
#define P7MU_CMD_RUN		((u16) (1U << 4))

#define P7MU_INT_PWREN					((u16) (1U << 0))
#define P7MU_INT_TEMP_WARN				((u16) (1U << 1))
#define P7MU_INT_TEMP_ERR				((u16) (1U << 2))
#define P7MU_INT_GPIO_1					((u16) (1U << 3))
#define P7MU_INT_GPIO_2					((u16) (1U << 4))
#define P7MU_INT_GPIO_3					((u16) (1U << 5))
#define P7MU_INT_RTC					((u16) (1U << 6))
#define P7MU_INT_MEASUREMENT_ERROR		((u16) (1U << 7))
#define P7MU_INT_FIFO_OVERFLOW			((u16) (1U << 8))
#define P7MU_INT_DET0_DOWN				((u16) (1U << 9))
#define P7MU_INT_DET0_UP				((u16) (1U << 10))
#define P7MU_INT_DET1_DOWN				((u16) (1U << 11))
#define P7MU_INT_DET1_UP				((u16) (1U << 12))
#define P7MU_INT_POWER					((u16) (1U << 13))
#define P7MU_INT_REBOOT					((u16) (1U << 14))
#define P7MU_INT_EN						((u16) (1U << 15))

#define P7MU_INT_GPIO_4		((u16) (1U << 0))
#define P7MU_INT_GPIO_5		((u16) (1U << 1))
#define P7MU_INT_DET2_DOWN	((u16) (1U << 2))
#define P7MU_INT_DET2_UP	((u16) (1U << 3))
#define P7MU_INT_DET3_DOWN	((u16) (1U << 4))
#define P7MU_INT_DET3_UP	((u16) (1U << 5))

/*
 * RTC block registers
 */
#define P7MU_RTC_PAGE   ((u16) 0x0600)
#define P7MU_RTC_SZ		7

/*
 * PWM block registers
 */
#define P7MU_PWM0_PAGE  ((u16) 0x0700)
#define P7MU_PWM1_PAGE	((u16) 0x0800)
#define P7MU_PWM_SZ		3

/*
 * ADC block registers
 */
#define P7MU_ADC_PAGE  ((u16) 0x0500)
#define P7MU_ADC_SZ		0x34

/*
 * SPI block register
 */
#define P7MU_SPI_PAGE   ((u16) 0x0400)
#define P7MU_SPI_SZ		2

/*
 * Application register
 */
#define P7MU_APPL_PAGE	((u16) 0x900)
#define P7MU_APPL_SZ		16

#define P7MU_APPL_DATA00_REG	(P7MU_APPL_PAGE + 0x00)
#define P7MU_APPL_DATA01_REG	(P7MU_APPL_PAGE + 0x01)
#define P7MU_APPL_DATA02_REG	(P7MU_APPL_PAGE + 0x02)
#define P7MU_APPL_DATA03_REG	(P7MU_APPL_PAGE + 0x03)
#define P7MU_APPL_DATA04_REG	(P7MU_APPL_PAGE + 0x04)
#define P7MU_APPL_DATA05_REG	(P7MU_APPL_PAGE + 0x05)
#define P7MU_APPL_DATA06_REG	(P7MU_APPL_PAGE + 0x06)
#define P7MU_APPL_DATA07_REG	(P7MU_APPL_PAGE + 0x07)
#define P7MU_APPL_DATA08_REG	(P7MU_APPL_PAGE + 0x08)
#define P7MU_APPL_DATA09_REG	(P7MU_APPL_PAGE + 0x09)
#define P7MU_APPL_DATA10_REG	(P7MU_APPL_PAGE + 0x0a)
#define P7MU_APPL_DATA11_REG	(P7MU_APPL_PAGE + 0x0b)
#define P7MU_APPL_DATA12_REG	(P7MU_APPL_PAGE + 0x0c)
#define P7MU_APPL_DATA13_REG	(P7MU_APPL_PAGE + 0x0d)
#define P7MU_APPL_DATA14_REG	(P7MU_APPL_PAGE + 0x0e)
#define P7MU_APPL_DATA15_REG	(P7MU_APPL_PAGE + 0x0f)


#define P7MU_REG_MAX	((u16) 0x0f0c)

/***********************
 * Global declarations.
 ***********************/

#define P7MU_PWREN_IRQ              0
#define P7MU_TEMP_WARN_IRQ          1
#define P7MU_TEMP_ERR_IRQ           2
#define P7MU_GPIO_1_IRQ             3
#define P7MU_GPIO_2_IRQ             4
#define P7MU_GPIO_3_IRQ             5
#define P7MU_RTC_IRQ                6
#define P7MU_MEASUREMENT_ERROR_IRQ  7
#define P7MU_FIFO_OVERFLOW_IRQ      8
#define P7MU_DET0_DOWN_IRQ          9
#define P7MU_DET0_UP_IRQ            10
#define P7MU_DET1_DOWN_IRQ          11
#define P7MU_DET1_UP_IRQ            12
#define P7MU_POWER_IRQ              13
#define P7MU_REBOOT_IRQ             14
#define P7MU_GPIO_4_IRQ             16
#define P7MU_GPIO_5_IRQ             17
#define P7MU_DET2_DOWN_IRQ          18
#define P7MU_DET2_UP_IRQ            19
#define P7MU_DET3_DOWN_IRQ          20
#define P7MU_DET3_UP_IRQ            21

#define P7MU_IRQ_NR                 22
#define DEFAULT_IRQ	(P7MU_INT_EN|P7MU_INT_TEMP_WARN|P7MU_INT_TEMP_ERR|P7MU_INT_POWER|P7MU_INT_PWREN|P7MU_INT_REBOOT|P7MU_INT_POWER)

static struct resource p7mu_iores = {
	.name	= P7MU_DRV_NAME " io",
	.start	= 0,
	.end	= P7MU_REG_MAX,
	.flags	= IORESOURCE_IO | IORESOURCE_BUSY
};

static struct resource p7mu_irqres = {
	.name	= P7MU_DRV_NAME " irq",
	.start	= 0,
	.end	= P7MU_IRQ_NR - 1,
	.flags	= IORESOURCE_IO | IORESOURCE_BUSY
};

static struct resource p7mu_rtc_res[] __devinitdata = {
	{
		.start	= P7MU_RTC_IRQ,
		.end	= P7MU_RTC_IRQ,
		.flags	= IORESOURCE_IRQ,
		.parent = &p7mu_irqres
	},
	{
		.start	= P7MU_RTC_PAGE,
		.end	= P7MU_RTC_PAGE + P7MU_RTC_SZ - 1,
		.flags	= IORESOURCE_IO,
		.parent = &p7mu_iores
	}
};

static struct resource p7mu_gpio_res[] __devinitdata = {
	{
		.start	= P7MU_PMC_GPIO_REG,
		.end	= P7MU_PMC_GPIO_REG + sizeof(u16) - 1,
		.flags	= IORESOURCE_IO,
		.parent = &p7mu_iores
	}
};

static struct resource p7mu_adc_res[] __devinitdata = {
	{   /* ADC */
		.start	= P7MU_ADC_PAGE,
		.end	= P7MU_ADC_PAGE + P7MU_ADC_SZ - 1,
		.flags	= IORESOURCE_IO,
		.parent = &p7mu_iores
	},
	{   /* SPI */
		.start	= P7MU_SPI_PAGE,
		.end	= P7MU_SPI_PAGE + sizeof(u16) - 1,
		.flags	= IORESOURCE_IO,
		.parent = &p7mu_iores
	}
};

static struct resource p7mu_pwm_res[] __devinitdata = {
	{   /* PWM0 */
		.start	= P7MU_PWM0_PAGE,
		.end	= P7MU_PWM0_PAGE + P7MU_PWM_SZ - 1,
		.flags	= IORESOURCE_IO,
		.parent = &p7mu_iores
	},
	{   /* PWM1 */
		.start	= P7MU_PWM1_PAGE,
		.end	= P7MU_PWM1_PAGE + P7MU_PWM_SZ - 1,
		.flags	= IORESOURCE_IO,
		.parent = &p7mu_iores
	}
};

static struct mfd_cell p7mu_cells[] __devinitdata = {
	{	/* RTC */
		.name			= P7MU_DRV_NAME "-rtc",
		.id				= -1,
		.resources		= p7mu_rtc_res,
		.num_resources	= ARRAY_SIZE(p7mu_rtc_res)
	},
	{	/* GPIO */
		.name			= P7MU_DRV_NAME "-gpio",
		.id				= -1,
		.resources		= p7mu_gpio_res,
		.num_resources	= ARRAY_SIZE(p7mu_gpio_res)
	},
	{	/* ADC */
		.name			= P7MU_DRV_NAME "-adc",
		.id				= -1,
		.resources		= p7mu_adc_res,
		.num_resources	= ARRAY_SIZE(p7mu_adc_res)
	},
	{	/* PWM */
		.name			= P7MU_DRV_NAME "-pwm",
		.id				= -1,
		.resources		= p7mu_pwm_res,
		.num_resources	= ARRAY_SIZE(p7mu_pwm_res)
	}
};

struct p7mu_irq {
	u16 const       msk;
	unsigned const  reg:1;
	const char *name;
};

#define P7MU_INIT_IRQ(_name, _reg)	\
	[P7MU_ ## _name ## _IRQ] = { .msk = P7MU_INT_ ## _name, .reg = _reg, .name = #_name }

static struct p7mu_irq p7mu_irqs[] = {
	/* Interrupt register 0 */
	P7MU_INIT_IRQ(PWREN, 0),
	P7MU_INIT_IRQ(TEMP_WARN, 0),
	P7MU_INIT_IRQ(TEMP_ERR, 0),
	P7MU_INIT_IRQ(GPIO_1, 0),
	P7MU_INIT_IRQ(GPIO_2, 0),
	P7MU_INIT_IRQ(GPIO_3, 0),
	P7MU_INIT_IRQ(RTC, 0),
	P7MU_INIT_IRQ(MEASUREMENT_ERROR, 0),
	P7MU_INIT_IRQ(FIFO_OVERFLOW, 0),
	P7MU_INIT_IRQ(DET0_DOWN, 0),
	P7MU_INIT_IRQ(DET0_UP, 0),
	P7MU_INIT_IRQ(DET1_DOWN, 0),
	P7MU_INIT_IRQ(DET1_UP, 0),
	P7MU_INIT_IRQ(POWER, 0),
	P7MU_INIT_IRQ(REBOOT, 0),
	/* Interrupt register 1 */
	P7MU_INIT_IRQ(GPIO_4, 1),
	P7MU_INIT_IRQ(GPIO_5, 1),
	P7MU_INIT_IRQ(DET2_DOWN, 1),
	P7MU_INIT_IRQ(DET2_UP, 1),
	P7MU_INIT_IRQ(DET3_DOWN, 1),
	P7MU_INIT_IRQ(DET3_UP, 1)
};

struct i2c_client*          p7mu_i2c;
EXPORT_SYMBOL(p7mu_i2c);
static struct pinctrl*      p7mu_pctl;

struct p7mu_chip {
	struct mutex irq_lock;
	uint32_t irq_mask;
	uint32_t irq_en;
	int	 irq_base;
	u16 gpio_ev_update;
	u16 gpio_ev;
} p7mu_chip;

#define P7MU_BOOT_REASON_FIRST_BOOT 0
#define P7MU_BOOT_REASON_SOFT_REBOOT 1
#define P7MU_BOOT_REASON_HARD_REBOOT 2
#define P7MU_BOOT_REASON_P7MU_PWREN 3
#define P7MU_BOOT_REASON_P7MU_REBOOT 4
#define P7MU_BOOT_REASON_P7MU_POWER 5
#define P7MU_BOOT_REASON_OTHER 6
static u16 boot_reason;
static u16 irq_before_reboot = 0;
static u16 pending_irq = 0;
static u16 power_supply_status = 0;
static u16 status_register = 0;
static u16 temperature_error_count = 0;

static char *boot_reasons_str[] = {
	"first boot",
	"soft reboot",
	"hard reboot",
	"pwr_en boot",
	"p7mu reboot",
	"power reboot",
	"other",
};

int p7mu_transfer(u16 offset, u16* buff, size_t cnt, bool read)
{
	struct i2c_msg  msg[2];
	int             err;

#ifdef DEBUG
	BUG_ON(! p7mu_i2c);
	BUG_ON((cnt * sizeof(*buff)) > (size_t) ((u16) (~0U)));
#endif

	msg[0].addr = p7mu_i2c->addr;
	msg[0].flags = 0;
	msg[0].len = (u16) sizeof(offset);
	msg[0].buf = (u8*) &offset;

	msg[1].addr = p7mu_i2c->addr;
	msg[1].flags = read ? I2C_M_RD : I2C_M_NOSTART;
	msg[1].len = (u16) (cnt * sizeof(*buff));
	msg[1].buf = (u8*) buff;

	err = i2c_transfer(p7mu_i2c->adapter, msg, ARRAY_SIZE(msg));
	if (err == ARRAY_SIZE(msg))
		return 0;
	else if (err < 0)
		return err;
	else
		return -EIO;
}
EXPORT_SYMBOL(p7mu_transfer);

int p7mu_mod16(u16 off, u16 val, u16 msk)
{
	u16         v;
	int const   err = p7mu_read16(off, &v);

	if (err)
		return err;

	if ((v & msk) == val)
		return 0;

	return p7mu_write16(off, (v & ~msk) | val);
}
EXPORT_SYMBOL(p7mu_mod16);

/********************************
 * Interrupt controller handling
 ********************************/

static void p7mu_irq_mask(struct irq_data *d)
{
	struct p7mu_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask &= ~(1 << (d->irq - chip->irq_base));
}

static void p7mu_irq_unmask(struct irq_data *d)
{
	struct p7mu_chip *chip = irq_data_get_irq_chip_data(d);

	chip->irq_mask |= 1 << (d->irq - chip->irq_base);
}

static void p7mu_irq_bus_lock(struct irq_data *d)
{
	struct p7mu_chip *chip = irq_data_get_irq_chip_data(d);

	mutex_lock(&chip->irq_lock);
}

static void p7mu_irq_bus_sync_unlock(struct irq_data *d)
{
	struct p7mu_chip *chip = irq_data_get_irq_chip_data(d);

	if (chip->irq_en != chip->irq_mask) {
		//printk("irq sync %x %x\n", chip->irq_mask, chip->irq_en);
		p7mu_write16(P7MU_PMC_INTERRUPT_EN_0_REG, chip->irq_mask & 0xffff);
		p7mu_write16(P7MU_PMC_INTERRUPT_EN_1_REG, chip->irq_mask >> 16);

		chip->irq_en = chip->irq_mask;
	}

	if (chip->gpio_ev_update != chip->gpio_ev) {
		p7mu_write16(0x101, chip->gpio_ev_update);
		chip->gpio_ev = chip->gpio_ev_update;
	}

	mutex_unlock(&chip->irq_lock);
}

static int p7mu_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct p7mu_chip *chip = irq_data_get_irq_chip_data(d);
	int num = d->irq - chip->irq_base;
	uint16_t level;
	u16 p7mu_gpio_ev;

	switch(num) {
		case 3:
			level = 0;
			break;
		case 4:
			level = 1;
			break;
		case 5:
			level = 2;
			break;
		case 16:
			level = 3;
			break;
		case 17:
			level = 4;
			break;
		default:
			return 0;
	};
	p7mu_gpio_ev = chip->gpio_ev_update;
	p7mu_gpio_ev &= ~(3 << (level*2));

	if ((type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH)
		p7mu_gpio_ev |= 2 << (level*2);
	else if (type & IRQ_TYPE_EDGE_FALLING)
		p7mu_gpio_ev |= 3 << (level*2);
	else if (type & IRQ_TYPE_EDGE_RISING)
		return -EINVAL;
	else if (type & IRQ_TYPE_LEVEL_HIGH)
		p7mu_gpio_ev |= 0 << (level*2);
	else if (type & IRQ_TYPE_LEVEL_LOW)
		p7mu_gpio_ev |= 1 << (level*2);

	chip->gpio_ev_update = p7mu_gpio_ev;

	return 0;
}

static struct irq_chip p7mu_irq_chip = {
	.name			= "p7mu",
	.irq_enable		= p7mu_irq_unmask,
	.irq_disable		= p7mu_irq_mask,
	.irq_bus_lock		= p7mu_irq_bus_lock,
	.irq_bus_sync_unlock	= p7mu_irq_bus_sync_unlock,
	.irq_set_type		= p7mu_gpio_irq_set_type,
};

static int p7mu_setup_irq(struct p7mu_chip *chip)
{
	int lvl;
	const int irq_base = P7_IRQS;
	const int irq_number = P7MU_GPIO_IRQS;

	mutex_init(&chip->irq_lock);

	chip->irq_base = irq_alloc_descs(-1, irq_base, irq_number, -1);
	if (chip->irq_base < 0)
		goto out_failed;

	for (lvl = 0; lvl < irq_number; lvl++) {
		int irq = lvl + chip->irq_base;

		irq_clear_status_flags(irq, IRQ_NOREQUEST);
		irq_set_chip_data(irq, chip);
		irq_set_chip(irq, &p7mu_irq_chip);
		irq_set_nested_thread(irq, true);
#ifdef CONFIG_ARM
		set_irq_flags(irq, IRQF_VALID);
#else
		irq_set_noprobe(irq);
#endif
	}

	p7mu_read16(0x101, &chip->gpio_ev);
	chip->gpio_ev_update = chip->gpio_ev;


	p7mu_rtc_res[0].start = p7mu_rtc_res[0].end += chip->irq_base;
	dev_info(&p7mu_i2c->dev, "p7mu irq %d %d\n", chip->irq_base, chip->irq_base+irq_number);
	return 0;

out_failed:
	return -ENODEV;
}

static irqreturn_t p7mu_handle_irq(int irq, void* dev_id)
{
	struct p7mu_chip *chip = dev_id;
	u16			                    stat_raw[2];
	u16			                    stat[2];
	u16			                    status;
	irqreturn_t                     ret = IRQ_HANDLED;
	int			                    tmp, tmp1;

	tmp = p7mu_read16(P7MU_PMC_INTERRUPT_0_REG, &stat_raw[0]);
	tmp1 = p7mu_read16(P7MU_PMC_INTERRUPT_1_REG, &stat_raw[1]);
	if (tmp || tmp1) {
		dev_warn(&p7mu_i2c->dev,
		                    "failed to get interrupts status (%d %d)\n",
		                    tmp, tmp1);
		ret = IRQ_NONE;
	}

	if (stat_raw[0] == 0 && stat_raw[1] == 0) {
		dev_warn(&p7mu_i2c->dev, "spurious interrupt ?\n");

		ret = IRQ_NONE;
	}

	if (ret == IRQ_NONE)
		return IRQ_NONE;

	ret = IRQ_NONE;

	/* XXX we could have race here. irq is triggered on p7mu, but while we are
	   handling p7mu irq (i2c sleep), we disable irq.
	 */
	/* Serve enabled interrutps only. */
	stat[0] = chip->irq_en & stat_raw[0];
	stat[1] = (chip->irq_en >> 16) & stat_raw[1];

	pending_irq = 0;
	for (tmp = 0; tmp < ARRAY_SIZE(p7mu_irqs); tmp++) {
		struct p7mu_irq const* const i = &p7mu_irqs[tmp];

		if (!(stat_raw[i->reg] & i->msk))
			continue;

		if (stat[i->reg] & i->msk) {
			if (i->reg == 0 && (i->msk & DEFAULT_IRQ)) {
				p7mu_read16(P7MU_PMC_POWER_SUPPLY_STATUS_REG, &power_supply_status);
				p7mu_read16(P7MU_PMC_STATUS_REG, &status_register);
				p7mu_read16(P7MU_PMC_TEMP_ERR_CNT_REG, &temperature_error_count);

				if (i->msk == P7MU_INT_PWREN)
					p7mu_write16(P7MU_APPL_DATA02_REG, P7MU_BOOT_REASON_P7MU_PWREN);
				else if (i->msk == P7MU_INT_REBOOT)
					p7mu_write16(P7MU_APPL_DATA02_REG, P7MU_BOOT_REASON_P7MU_REBOOT);
				else if (i->msk == P7MU_INT_POWER) {
					p7mu_write16(P7MU_APPL_DATA06_REG, power_supply_status);
					p7mu_read16(P7MU_PMC_POWER_SUPPLY_FILTER_REG, &status);
					p7mu_write16(P7MU_APPL_DATA07_REG, status);
					p7mu_read16(P7MU_PMC_POWER_SUPPLY_MASK_REG, &status);
					p7mu_write16(P7MU_APPL_DATA08_REG, status);
					p7mu_write16(P7MU_APPL_DATA02_REG, P7MU_BOOT_REASON_P7MU_POWER);
				}

				dev_warn(&p7mu_i2c->dev, "irq : %s\n", i->name);
				pending_irq |= i->msk;
			}
			else
				handle_nested_irq(chip->irq_base + tmp);

			ret = IRQ_HANDLED;
		}
		else
			dev_warn(&p7mu_i2c->dev, "spurious irq : %s\n", i->name);
	}

	/* Acknowledge interrupts all at once. */
	tmp = p7mu_write16(P7MU_PMC_INTERRUPT_0_REG, ~stat_raw[0]);
	tmp1 = p7mu_write16(P7MU_PMC_INTERRUPT_1_REG, ~stat_raw[1]);
	if (tmp || tmp1) {
		dev_warn(&p7mu_i2c->dev,
							"failed to acknowledge interrupts (%d %d)\n",
		                    tmp, tmp1);
	}

	if (pending_irq)
		sysfs_notify(&p7mu_i2c->dev.kobj, NULL, "irq");

	return ret;
}

static int __devinit p7mu_init_irqs(void)
{
	struct p7mu_plat_data const* const  pdata = p7mu_pdata();
	int								    err;
	struct p7mu_chip *chip = &p7mu_chip; //XXX

	BUG_ON(! pdata);

	/* Retrieve interrupt line the GPIO will act as. */
	p7mu_i2c->irq = gpio_to_irq(pdata->gpio);
	if (p7mu_i2c->irq < 0) {
		/*
		 * GPIO platform data probably did not provide a
		 * suitable GPIO <-> IRQ mapping.
		 */
		dev_err(&p7mu_i2c->dev, "failed to find a valid GPIO interrupt\n");
		return p7mu_i2c->irq;
	}

	/* Disable all interrupts before enabling GPIO pin as interrupt source. */
	err = p7mu_write16(P7MU_PMC_INTERRUPT_EN_0_REG, 0);
	if (err) {
		dev_err(&p7mu_i2c->dev, "failed to disable interrupts\n");
		return err;
	}
	err = p7mu_write16(P7MU_PMC_INTERRUPT_EN_1_REG, 0);
	if (err) {
		dev_err(&p7mu_i2c->dev, "failed to disable interrupts\n");
		return err;
	}

	/* Setup GPIO for interrupt usage. */
	err = gpio_request_one(pdata->gpio, GPIOF_IN, P7MU_DRV_NAME "_irq");
	if (err) {
		dev_err(&p7mu_i2c->dev, "failed to request GPIO interrupt\n");
		return err;
	}

	/* Ack all interrupts if any: might be caused by calibration ?? */
	p7mu_write16(P7MU_PMC_INTERRUPT_0_REG, 0);
	p7mu_write16(P7MU_PMC_INTERRUPT_1_REG, 0);

	/* Register base interrupt handler in threaded context. */
	err = request_threaded_irq(p7mu_i2c->irq,
							   NULL,
							   &p7mu_handle_irq,
							   IRQF_TRIGGER_HIGH|IRQF_ONESHOT,
							   P7MU_DRV_NAME,
							   chip);
	if (err) {
		dev_err(&p7mu_i2c->dev, "failed to request base irq %d\n", p7mu_i2c->irq);
		goto free_domain;
	}

	err = p7mu_setup_irq(chip);
	if (err) {
		goto free_irq;
	}

	/* Enable hardware interrupts handling. */
	chip->irq_mask = chip->irq_en = DEFAULT_IRQ;
	err = p7mu_write16(P7MU_PMC_INTERRUPT_EN_0_REG, chip->irq_mask);
	if (!err)
		err = p7mu_write16(P7MU_PMC_INTERRUPT_EN_1_REG, chip->irq_mask >> 16);
	if (! err)
		return 0;

free_irq:
	dev_err(&p7mu_i2c->dev, "failed to enable interrupts\n");
	free_irq(p7mu_i2c->irq, NULL);

free_domain:
/* Re-enable module once fixed. */
	gpio_free(pdata->gpio);
	return err;
}

static void p7mu_exit_irqs(void)
{
	struct p7mu_plat_data const* const pdata = p7mu_pdata();

	/* For sake of security: irqs should have been freed using p7mu_free_irq. */
	p7mu_write16(P7MU_PMC_INTERRUPT_EN_0_REG, 0);
	p7mu_write16(P7MU_PMC_INTERRUPT_EN_1_REG, 0);


	irq_free_descs(p7mu_chip.irq_base, P7MU_GPIO_IRQS);
	free_irq(p7mu_i2c->irq, NULL);
	gpio_free(pdata->gpio);
}

int p7mu_gpio_to_irq(unsigned int gpio)
{
	struct p7mu_chip *chip = &p7mu_chip; //XXX
	int gpio_to_irq[] = {3, 4, 5, 16, 17};
	if (gpio >= ARRAY_SIZE(gpio_to_irq))
		return -EINVAL;
	return gpio_to_irq[gpio] + chip->irq_base;
}
EXPORT_SYMBOL_GPL(p7mu_gpio_to_irq);
/***************************
 * MFD sub-devices handling
 ***************************/

struct resource const* p7mu_request_region(struct platform_device* pdev,
                                           struct resource const* res)
{
	resource_size_t const start = res->start;

	res = __request_region(&p7mu_iores,
						   start,
	                       resource_size(res),
	                       dev_name(&pdev->dev),
	                       IORESOURCE_EXCLUSIVE);
	if (res)
		return res;

	dev_warn(&pdev->dev, "failed to reserve I/O region 0x%04x\n", start);
	return ERR_PTR(-EBUSY);
}
EXPORT_SYMBOL_GPL(p7mu_request_region);

void p7mu_release_region(struct resource const* res)
{
	__release_region(&p7mu_iores, res->start, resource_size(res));
}
EXPORT_SYMBOL_GPL(p7mu_release_region);

static void (*p7mu_restart)(char, char const*);

static void p7mu_reboot(char mode, const char *cmd)
{
	p7mu_write16(0xa00, 6);
	p7mu_write16(0xb00, 6);
	if (mode == 's') {
		p7mu_write16(P7MU_APPL_DATA02_REG, P7MU_BOOT_REASON_SOFT_REBOOT);
		if (! p7mu_write16(P7MU_PMC_CMD_REG, P7MU_CMD_REBOOT))
			mdelay(100);
		dev_warn(&p7mu_i2c->dev, "soft reset failed, trying hard reset...\n");
	}

	p7mu_write16(P7MU_APPL_DATA02_REG, P7MU_BOOT_REASON_HARD_REBOOT);
	p7mu_restart(mode, cmd);
}

static void (*p7mu_pwroff)(void);

static void p7mu_shutdown(void)
{
	int ret;
	p7mu_write16(0xa00, 6);
	p7mu_write16(0xb00, 6);
	dev_err(&p7mu_i2c->dev, "p7mu power off\n");
	ret = p7mu_write16(P7MU_PMC_CMD_REG, P7MU_CMD_SHUTDOWN);

 	/* wait power off : p7mu take 5s before going off ... */
	if (ret == 0)
		mdelay(6000);

	dev_err(&p7mu_i2c->dev, "power off failed %d.\n", ret);
	while (1)
		cpu_do_idle();
}

/* We will store the jump address in the first 2 application regs */
int p7mu_save_cpu_jump(void *jump_addr)
{
	u32 addr = virt_to_phys(jump_addr);
	u16 addr_hi = (u16)(addr >> 16);
	u16 addr_lo = (u16)(addr & 0xffff);
	u16 val;

	printk("p7mu_save_cpu_jump storing 0x%x\n", addr);

	if ((0 != p7mu_write16(P7MU_APPL_DATA00_REG, addr_hi))
		|| (0 != p7mu_write16(P7MU_APPL_DATA01_REG, addr_lo))) {
		dev_err(&p7mu_i2c->dev, "cpu jump set fail.\n");
		return -EIO;
	}
	if (p7mu_read16(P7MU_APPL_DATA00_REG, &val) || addr_hi != val) {
		dev_err(&p7mu_i2c->dev, "DATA00_REG is wrong 0x%x\n", val);
		return -EIO;
	}
	if (p7mu_read16(P7MU_APPL_DATA01_REG, &val) || addr_lo != val) {
		dev_err(&p7mu_i2c->dev, "DATA01_REG is wrong 0x%x\n", val);
		return -EIO;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(p7mu_save_cpu_jump);

static void p7mu_wakeup_setup(void)
{
	struct p7mu_plat_data const* const  pdata = p7mu_pdata();

	p7mu_write16(P7MU_PMC_GPIO_EVENT_SEL_REG, pdata->gpio_event_sel);
}

static void p7mu_print_idstr(char const* id, size_t size)
{
	print_hex_dump(KERN_ERR,
				   "\t",
				   DUMP_PREFIX_NONE,
				   16,
				   1,
				   id,
				   size,
				   false);
}

static inline int p7mu_read_regs(u16 offset,
		u16* buff,
		size_t cnt)
{
	/* there is no endian conversion on data ! */
	return p7mu_transfer(cpu_to_be16(offset), buff, cnt, true);
}

static ssize_t show_boot_reason(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE, "%s\n", boot_reasons_str[boot_reason]);
}
DEVICE_ATTR(boot_reason,0444,show_boot_reason,NULL);

static ssize_t show_irq_mask(int mask, char *buf)
{
	int writeidx = 0;
	int tmp;

	for (tmp = 0; tmp < ARRAY_SIZE(p7mu_irqs); tmp++) {
		struct p7mu_irq const* const i = &p7mu_irqs[tmp];

		if (i->reg != 0 || !(i->msk & DEFAULT_IRQ))
			continue;

		if (mask & i->msk) {
			writeidx += snprintf(buf+writeidx,PAGE_SIZE-writeidx,
						"%s\n", i->name);
		}
	}

	return writeidx;
}

static ssize_t show_irq(struct device *d, struct device_attribute *attr, char *buf)
{
	int ret;

	ret = show_irq_mask(pending_irq, buf);
	pending_irq = 0;

	return ret;
}
DEVICE_ATTR(irq,0444,show_irq,NULL);

static ssize_t show_irq_before_reboot(struct device *d, struct device_attribute *attr, char *buf)
{
	int ret;

	ret = show_irq_mask(irq_before_reboot, buf);

	return ret;
}
DEVICE_ATTR(irq_before_reboot,0444,show_irq_before_reboot,NULL);

static ssize_t show_power_supply_status(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE, "%d\n", power_supply_status);
}
DEVICE_ATTR(power_supply_status,0444,show_power_supply_status,NULL);

static ssize_t show_status_register(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE, "%d\n", status_register);
}
DEVICE_ATTR(status_register,0444,show_status_register,NULL);

static ssize_t show_temperature_error_count(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE, "%d\n", temperature_error_count);
}
DEVICE_ATTR(temperature_error_count,0444,show_temperature_error_count,NULL);

#ifdef CONFIG_SUSPEND
extern int p7_resume_cnt;
static long p7_resume_time;
static ssize_t show_resume_cnt(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE, "%d", p7_resume_cnt);
}
DEVICE_ATTR(resume_cnt,0644,show_resume_cnt,NULL);

static ssize_t show_resume_time(struct device *d, struct device_attribute *attr, char *buf)
{
	return snprintf(buf,PAGE_SIZE, "%ld", p7_resume_time);
}
DEVICE_ATTR(resume_time,0644,show_resume_time,NULL);
#endif

static struct attribute *p7mu_attrs[] = {
	&dev_attr_boot_reason.attr,
	&dev_attr_irq.attr,
	&dev_attr_irq_before_reboot.attr,
	&dev_attr_power_supply_status.attr,
	&dev_attr_status_register.attr,
	&dev_attr_temperature_error_count.attr,
#ifdef CONFIG_SUSPEND
	&dev_attr_resume_cnt.attr,
	&dev_attr_resume_time.attr,
#endif
	NULL
};

static const struct attribute_group p7mu_attr_group = {
	.attrs = p7mu_attrs,
};

static char const p7mu_idstr[] __devinitconst = { '\0', 'P', '7', 'M', 'U', '_', 'R' };

static int __devinit p7mu_probe_i2c(struct i2c_client* client,
									struct i2c_device_id const* dev_id)
{
	struct i2c_adapter* const   adapt = client->adapter;
	char						id[4 * sizeof(u16)];
	int							vers;
	int							err;
	u16 val;
	u16 last_power_status[4];

	/*
	 * For instance, depend on I2C driver supporting I2C_M_NOSTART so that
	 * we can aggregate multiple writes in a single transfer. We would
	 * otherwise need to allocate temporary buffers and perform an additional
	 * mem copy to serve client requests.
	 */
	if (! i2c_check_functionality(adapt, I2C_FUNC_PROTOCOL_MANGLING))
		return -ENOTSUPP;

	p7mu_i2c = client;

	err = p7mu_read_regs(P7MU_CFG_ID, (u16*) id, 4);
	if (err) {
		dev_err(&client->dev, "failed to get id string\n");
		goto err;
	}
	if (memcmp(p7mu_idstr, id, sizeof(p7mu_idstr))) {
		/* XXX on some board the first read doesn't work, retry */
		dev_err(&client->dev, "found invalid chip with wrong id string : retrying\n");
		p7mu_print_idstr(id, ARRAY_SIZE(id));
		p7mu_read_regs(P7MU_CFG_ID, (u16*) id, 4);
	}

	err = -ENODEV;
	if (memcmp(p7mu_idstr, id, sizeof(p7mu_idstr))) {
		dev_err(&client->dev, "found invalid chip with wrong id string\n");
		p7mu_print_idstr(id, ARRAY_SIZE(id));
		goto err;
	}

	vers = (int) id[7] - (int) '0';
	if (vers < 0 || vers > 0xf) {
		dev_err(&client->dev, "found invalid chip with wrong version string\n");
		p7mu_print_idstr(id, ARRAY_SIZE(id));
		goto err;
	}

	err = sysfs_create_group(&client->dev.kobj, &p7mu_attr_group);
	if (err)
		goto err;

	p7mu_init_clk();

	err = p7mu_read16(P7MU_APPL_DATA02_REG, &val);
	if (err || val < P7MU_BOOT_REASON_FIRST_BOOT || val > P7MU_BOOT_REASON_OTHER)
	{
		dev_err(&client->dev, "failed to read boot reason\n");
		boot_reason = P7MU_BOOT_REASON_FIRST_BOOT;
	}
	else
	{
		boot_reason = val;
		dev_info(&client->dev, "boot reason: %s\n", boot_reasons_str[boot_reason]);
		if (boot_reason == P7MU_BOOT_REASON_P7MU_POWER) {
			p7mu_read16(P7MU_APPL_DATA06_REG, &last_power_status[0]);
			p7mu_read16(P7MU_APPL_DATA07_REG, &last_power_status[1]);
			p7mu_read16(P7MU_APPL_DATA08_REG, &last_power_status[2]);
			dev_info(&client->dev, "satus before reboot %d, filter %04X, mask %04X\n",
				last_power_status[0], last_power_status[1], last_power_status[2]);
		}
	}
	err = p7mu_read16(0xf05, &val);
	if (!err) {
		dev_info(&client->dev, "p7mu event before reboot : %s%s%s%s%s\n",
				(val & P7MU_INT_REBOOT) ? "pwren pulse,":"",
				(val & P7MU_INT_POWER) ? "regulation failure,":"",
				(val & P7MU_INT_TEMP_ERR) ? "temp error,":"",
				(val & P7MU_INT_TEMP_WARN) ? "temp warn,":"",
				(val & P7MU_INT_PWREN) ? "pwren failling,":""
				);
		irq_before_reboot = val;
	}

	// Set boot reason to other by default
	p7mu_write16(P7MU_APPL_DATA02_REG, P7MU_BOOT_REASON_OTHER);

	err = p7mu_init_irqs();
	if (err) {
		dev_err(&client->dev, "failed to initialize interrupts (%d)\n", err);
		goto clk;
	}

	p7mu_pctl = pinctrl_get_select_default(&client->dev);
	if (IS_ERR(p7mu_pctl)) {
		dev_err(&client->dev, "failed to initialize pins (%ld)\n",
				PTR_ERR(p7mu_pctl));
		goto irq;
	}

	/* Now register cells (future platform_device) to MFD layer. */
	err = mfd_add_devices(&client->dev,
						  0,
						  p7mu_cells,
						  ARRAY_SIZE(p7mu_cells),
						  NULL,
						  0);
	if (err) {
		dev_err(&client->dev, "failed to register sub-devices (%d)\n", err);
		goto pin;
	}


	BUG_ON(! arm_pm_restart);
	p7mu_restart = arm_pm_restart;
	arm_pm_restart = p7mu_reboot;
	p7mu_pwroff = pm_power_off;
	pm_power_off = p7mu_shutdown;
	p7mu_wakeup_setup();

	dev_info(&client->dev, "attached device v%d to I2C bus %s.%d\n",
			 vers,
			 adapt->name,
			 adapt->nr);
	return 0;

pin:
	pinctrl_put(p7mu_pctl);
irq:
	p7mu_exit_irqs();
clk:
	p7mu_exit_clk();
	sysfs_remove_group(&client->dev.kobj, &p7mu_attr_group);
err:
	return err;
}

static int __devexit p7mu_remove_i2c(struct i2c_client* client)
{
	struct i2c_adapter const* const adapt = client->adapter;

	sysfs_remove_group(&client->dev.kobj, &p7mu_attr_group);

	BUG_ON(! p7mu_restart);
	arm_pm_restart = p7mu_restart;
	BUG_ON(! pm_power_off);
	pm_power_off = p7mu_pwroff;

	mfd_remove_devices(&client->dev);

	pinctrl_put(p7mu_pctl);
	p7mu_exit_irqs();
	p7mu_exit_clk();

	dev_info(&adapt->dev, "detached from I2C bus %s.%d\n",
			 adapt->name,
			 adapt->nr);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int p7mu_suspend(struct i2c_client *client, pm_message_t mesg)
{
	p7mu_write16(0xa00, 6);
	p7mu_write16(0xb00, 6);

	return 0;
}
static int p7mu_resume(struct i2c_client *client)
{
	static int resume_cnt;
	//struct p7mu_data *data = i2c_get_clientdata(client);
	p7mu_resume_clk();
	p7mu_write16(0xa00, 4);
	p7mu_write16(0xb00, 4);

	if (resume_cnt != p7_resume_cnt) {
		struct timespec tv;
		do_posix_clock_monotonic_gettime(&tv);

		p7_resume_time = tv.tv_sec;
		resume_cnt = p7_resume_cnt;
	}
	return 0;
}
#else
#define p7mu_suspend  NULL
#define p7mu_resume   NULL
#endif

static struct i2c_device_id const p7mu_i2c_id[] = {
	{ P7MU_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, p7mu_i2c_id);

static struct i2c_driver p7mu_i2c_driver = {
	.driver = {
		.name	= P7MU_DRV_NAME,
	},
	.probe		= &p7mu_probe_i2c,
	.remove		= __devexit_p(&p7mu_remove_i2c),
	.suspend	= p7mu_suspend,
	.resume		= p7mu_resume,
	.id_table	= p7mu_i2c_id
};

static int __init p7mu_init_core(void)
{
	return i2c_add_driver(&p7mu_i2c_driver);
}
postcore_initcall(p7mu_init_core);

static void __exit p7mu_exit_core(void)
{
	i2c_del_driver(&p7mu_i2c_driver);
}
module_exit(p7mu_exit_core);

MODULE_DESCRIPTION("Parrot Power Management Unit core driver");
MODULE_AUTHOR("Grégor Boirie <gregor.boirie@parrot.com>");
MODULE_LICENSE("GPL");
