/**
 * linux/arch/arm/mach-parrot7/board-common.c - Parrot7 based boards common
 *                                              implementation
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    12-Apr-2013
 *
 * This file is released under the GPL
 */

#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/io.h>
#include "common.h"
#include "pinctrl.h"
#include "board-common.h"

int parrot_force_usb_device;
void __init p7brd_export_uart_hw_infos(int uart, int rts_cts, char *version);

static int __init parrot_force_usbd(char *str)
{
	parrot_force_usb_device = 1;
	return 1;
}
__setup("parrot_force_usbd", parrot_force_usbd);

/*********************************
 * I2C master buses & controllers
 *********************************/

#include <i2c/p7-i2cm.h>
#include "i2cm.h"

#if defined(CONFIG_I2CM_PARROT7) || \
    defined(CONFIG_I2CM_PARROT7_MODULE)

/*
 * Untill now, no used slave device are high speed compliant.
 */
static struct p7i2cm_plat_data p7brd_i2cm_pdata[] = {
	{   /* I2CM0 platform data holder */
		.high_speed = false
	},
	{   /* I2CM1 platform data holder */
		.high_speed = false
	},
	{   /* I2CM2 platform data holder */
		.high_speed = false
	},
	{   /* I2CM3 platform data holder */
		.high_speed = false
	}
};

static unsigned long fc7100_i2cm_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

/* I2CM0 pins */
static struct pinctrl_map p7brd_i2cm0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_0_CLK),
	P7_INIT_PINCFG(P7_I2C_0_CLK, fc7100_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_0_DAT),
	P7_INIT_PINCFG(P7_I2C_0_DAT, fc7100_i2cm_pinconf)
};

/* I2CM1 pins */
static struct pinctrl_map p7brd_i2cm1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_1_CLK),
	P7_INIT_PINCFG(P7_I2C_1_CLK, fc7100_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_1_DAT),
	P7_INIT_PINCFG(P7_I2C_1_DAT, fc7100_i2cm_pinconf)
};

/* I2CM2 pins */
static struct pinctrl_map p7brd_i2cm2_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_CLK),
	P7_INIT_PINCFG(P7_I2C_2_CLK, fc7100_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_DAT),
	P7_INIT_PINCFG(P7_I2C_2_DAT, fc7100_i2cm_pinconf)
};

/* I2CM3 pins */
static struct pinctrl_map p7brd_i2cm3_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_SECURE_CLKa),
	P7_INIT_PINCFG(P7_I2C_SECURE_CLKa, fc7100_i2cm_pinconf),
	P7_INIT_PINMAP(P7_I2C_SECURE_DAT),
	P7_INIT_PINCFG(P7_I2C_SECURE_DAT, fc7100_i2cm_pinconf)
};

/**
 * p7brd_init_i2cm() - Instantiate I2C master controller identified by @bus
 *                     for further driver usage.
 *
 * @bus:    master controller / bus identifier
 * @hz:     bus speed in Hertz
 */
void __init p7brd_init_i2cm(int bus, unsigned int hz)
{
	struct p7i2cm_plat_data* const pdata = &p7brd_i2cm_pdata[bus];

#ifdef DEBUG
	BUG_ON(bus >= ARRAY_SIZE(p7brd_i2cm_pdata));
	BUG_ON(bus < 0);
	BUG_ON(! hz);
#endif
	if (bus < 0 || bus >= ARRAY_SIZE(p7brd_i2cm_pdata))
		return;

	pdata->bus_freq = hz;

	switch (bus) {
	case 0:
		p7_init_i2cm(0,
		             pdata,
		             p7brd_i2cm0_pins,
		             ARRAY_SIZE(p7brd_i2cm0_pins));
		break;
	case 1:
		p7_init_i2cm(1,
		             pdata,
		             p7brd_i2cm1_pins,
		             ARRAY_SIZE(p7brd_i2cm1_pins));
		break;
	case 2:
		p7_init_i2cm(2,
		             pdata,
		             p7brd_i2cm2_pins,
		             ARRAY_SIZE(p7brd_i2cm2_pins));
		break;
	case 3:
		p7_init_i2cm(3,
		             pdata,
		             p7brd_i2cm3_pins,
		             ARRAY_SIZE(p7brd_i2cm3_pins));
		break;
	}
}

#if defined(CONFIG_I2C_MUX_PARROT7) || \
    defined(CONFIG_I2C_MUX_PARROT7_MODULE)

void __init p7brd_init_i2cm_muxed(int bus,
                                  unsigned int hz,
                                  struct pinctrl_map* main_pins,
                                  size_t main_pin_cnt,
                                  struct p7i2cmux_plat_data* mux_pdata,
                                  struct p7i2cmux_pins const* mux_pins)
{
	struct p7i2cm_plat_data* const pdata = &p7brd_i2cm_pdata[bus];

	struct pinctrl_map* pins = NULL;
	size_t pin_cnt = 0;

#ifdef DEBUG
	BUG_ON(bus >= ARRAY_SIZE(p7brd_i2cm_pdata));
	BUG_ON(bus < 0);
	BUG_ON(! hz);
	BUG_ON((main_pins != NULL) && (main_pin_cnt == 0));
#endif

	pdata->bus_freq = hz;

	if (main_pins != NULL) {
		pins = main_pins;
		pin_cnt = main_pin_cnt;
	} else {

		switch (bus) {
			case 0:
				pins = p7brd_i2cm0_pins;
				pin_cnt = ARRAY_SIZE(p7brd_i2cm0_pins);
				break;
			case 1:
				pins = p7brd_i2cm1_pins;
				pin_cnt = ARRAY_SIZE(p7brd_i2cm1_pins);
				break;
			case 2:
				pins = p7brd_i2cm2_pins;
				pin_cnt = ARRAY_SIZE(p7brd_i2cm2_pins);
				break;
		}
	}
	return p7_init_i2cm_muxed(bus,
	                          pdata,
	                          pins,
	                          pin_cnt,
	                          mux_pdata,
	                          mux_pins);
}

#endif  /* defined(CONFIG_I2C_MUX_PARROT7) || \
           defined(CONFIG_I2C_MUX_PARROT7_MODULE) */

#endif  /* defined(CONFIG_I2CM_PARROT7) || \
           defined(CONFIG_I2CM_PARROT7_MODULE) */

int __init parrot_init_i2c_slave(int i2c_bus,
				 struct i2c_board_info *info,
				 const char *desc,
				 int irq_type)
{
	int gpio;
	int err;

	if (irq_type != P7_I2C_NOIRQ) {
		gpio = info->irq;

		if (!gpio_is_valid(gpio)) {
			pr_err("%s : Invalid gpio %d\n", __func__, gpio);
			return -EINVAL;
		}

		if (irq_type == P7_I2C_IRQ) {
			err = gpio_request(gpio, desc);
			if (err) {
				pr_err("%s : couldn't request GPIO %d "
				       "for %s %s [%d]\n", __func__,
				       gpio, desc, info->type, err);
				return -EINVAL;
			}
			p7_gpio_interrupt_register(gpio);
			info->irq = gpio_to_irq(gpio);
		} else if (irq_type == P7_I2C_SHAREDIRQ) {

			info->irq = gpio_to_irq(gpio);

			if (info->irq < 0) {
				pr_err("%s : couldn't find an IRQ for gpio %d\n",
				       __func__, gpio);
				gpio_free(gpio);
				return -EINVAL;
			}
		} else {
			pr_err("%s : invalid i2c irq mode\n", __func__);
			return -EINVAL;
		}
	}

	p7brd_export_i2c_hw_infos(i2c_bus, info->addr, "", desc);
	p7_init_i2cm_slave(i2c_bus, info, NULL, 0);

	return 0;
}


/*********************************
 * I2C slave controllers
 *********************************/
#include <i2c/plds_i2cs.h>
#include "i2cs.h"

#if defined(CONFIG_PLDS_I2CS) || \
    defined(CONFIG_PLDS_I2CS_MODULE)

static unsigned long fc7100_i2cs_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

/* I2CS0 pins */
static struct pinctrl_map p7brd_i2cs0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_0_SL_CLK),
	P7_INIT_PINCFG(P7_I2C_0_SL_CLK, fc7100_i2cs_pinconf),
	P7_INIT_PINMAP(P7_I2C_0_SL_DAT),
	P7_INIT_PINCFG(P7_I2C_0_SL_DAT, fc7100_i2cs_pinconf)
};

/* I2CS1 pins */
static struct pinctrl_map p7brd_i2cs1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_1_SL_CLK),
	P7_INIT_PINCFG(P7_I2C_1_SL_CLK, fc7100_i2cs_pinconf),
	P7_INIT_PINMAP(P7_I2C_1_SL_DAT),
	P7_INIT_PINCFG(P7_I2C_1_SL_DAT, fc7100_i2cs_pinconf)
};

/* I2CS2 pins */
static struct pinctrl_map p7brd_i2cs2_pins[] __initdata = {
	P7_INIT_PINMAP(P7_I2C_2_SL_CLK),
	P7_INIT_PINCFG(P7_I2C_2_SL_CLK, fc7100_i2cs_pinconf),
	P7_INIT_PINMAP(P7_I2C_2_SL_DAT),
	P7_INIT_PINCFG(P7_I2C_2_SL_DAT, fc7100_i2cs_pinconf)
};

/**
 * p7brd_init_i2cs() - Instantiate I2C slave controller identified by @bus
 *                     for further driver usage.
 *
 * @bus:    bus identifier
 */
void __init p7brd_init_i2cs(struct plds_i2cs_pdata *pdata)
{
#ifdef DEBUG
	BUG_ON(pdata->i2c_bus < 0);
	BUG_ON(pdata->i2c_bus >= 3);
#endif
	switch (pdata->i2c_bus) {
	case 0:
		p7_init_i2cs(pdata,
		             p7brd_i2cs0_pins,
		             ARRAY_SIZE(p7brd_i2cs0_pins));
		break;
	case 1:
		p7_init_i2cs(pdata,
		             p7brd_i2cs1_pins,
		             ARRAY_SIZE(p7brd_i2cs1_pins));
		break;
	case 2:
		p7_init_i2cs(pdata,
		             p7brd_i2cs2_pins,
		             ARRAY_SIZE(p7brd_i2cs2_pins));
		break;
	}
}
#endif  /* defined(CONFIG_PLDS_I2CS) || \
           defined(CONFIG_PLDS_I2CS_MODULE) */

/******************
 * USB controllers
 ******************/

#if defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC)
#include <mfd/p7mu.h>
#include <mach/usb-p7.h>
#include "usb.h"

static unsigned long fc7100_usb_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1(reg=3) */
};

static struct pinctrl_map p7brd_usb0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_ULPI_0_DATA00),
	P7_INIT_PINCFG(P7_ULPI_0_DATA00, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA01),
	P7_INIT_PINCFG(P7_ULPI_0_DATA01, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA02),
	P7_INIT_PINCFG(P7_ULPI_0_DATA02, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA03),
	P7_INIT_PINCFG(P7_ULPI_0_DATA03, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA04),
	P7_INIT_PINCFG(P7_ULPI_0_DATA04, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA05),
	P7_INIT_PINCFG(P7_ULPI_0_DATA05, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA06),
	P7_INIT_PINCFG(P7_ULPI_0_DATA06, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DATA07),
	P7_INIT_PINCFG(P7_ULPI_0_DATA07, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_NXT),
	P7_INIT_PINCFG(P7_ULPI_0_NXT, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_STP),
	P7_INIT_PINCFG(P7_ULPI_0_STP, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_DIR),
	P7_INIT_PINCFG(P7_ULPI_0_DIR, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_0_CLK),
	P7_INIT_PINCFG(P7_ULPI_0_CLK, fc7100_usb_pinconf)
};

static struct pinctrl_map p7brd_usb1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_ULPI_1_DATA00),
	P7_INIT_PINCFG(P7_ULPI_1_DATA00, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA01),
	P7_INIT_PINCFG(P7_ULPI_1_DATA01, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA02),
	P7_INIT_PINCFG(P7_ULPI_1_DATA02, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA03),
	P7_INIT_PINCFG(P7_ULPI_1_DATA03, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA04),
	P7_INIT_PINCFG(P7_ULPI_1_DATA04, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA05),
	P7_INIT_PINCFG(P7_ULPI_1_DATA05, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA06),
	P7_INIT_PINCFG(P7_ULPI_1_DATA06, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DATA07),
	P7_INIT_PINCFG(P7_ULPI_1_DATA07, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_NXT),
	P7_INIT_PINCFG(P7_ULPI_1_NXT, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_STP),
	P7_INIT_PINCFG(P7_ULPI_1_STP, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_DIR),
	P7_INIT_PINCFG(P7_ULPI_1_DIR, fc7100_usb_pinconf),
	P7_INIT_PINMAP(P7_ULPI_1_CLK),
	P7_INIT_PINCFG(P7_ULPI_1_CLK, fc7100_usb_pinconf)
};

static int p7_init_usb(struct platform_device *pdev, int (*ulpi_write)(struct platform_device *pdev, u8 addr, u8 val))
{
	struct p7_usb2_platform_data* const pdata = dev_get_platdata(&pdev->dev);
	int ret;

	/* disable usb phy */
	ret = p7mu_write16(pdata->phy_enable, 6);
	if (ret) {
		dev_err(&pdev->dev, "failed to disable phy (%d)\n", ret);
		return ret;
	}

	/* enable usb phy */
	ret = p7mu_write16(pdata->phy_enable, 4);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable phy (%d)\n", ret);
		return ret;
	}

	pdata->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pdata->pctl)) {
		dev_err(&pdev->dev, "failed to initialize pins\n");
		ret = -EIO;
		goto disable_usb;
	}

	/* FIXME do detection on p7mu, not p7 */
	if (pdata->chip_rev == P7_CHIPREV_R1 ||
	    pdata->chip_rev == P7_CHIPREV_R2) {
		if(ulpi_write) {
			ret = ulpi_write(pdev, 0x33, 0xd0);
			dev_dbg(&pdev->dev, "disabling auto calibration (%i)\n", ret);
		}
		else
			dev_err(&pdev->dev, "Unable to disable auto calibration\n");
	}

	/*  There is a known bug where the
		USB ULPI interface on USB controller freezes when voltage on
		Vbus crosses the Vbus valid threshold
		http://www.xilinx.com/support/answers/61313.html
		Since we don't use the VbusValid interrupt we deactivate it in the PHY */
	if(ulpi_write) {
		ret = ulpi_write(pdev, 0x0d, 0x1d); // Rising edge
		ret = ulpi_write(pdev, 0x10, 0x1d); // Falling edge
		dev_dbg(&pdev->dev, "disabling VbusValid interrupt (%i)\n", ret);
	}
	else
		dev_err(&pdev->dev, "Unable to disable VbusValid interrupts\n");

	/* deliver VBUS to connector if role is host (or dual, needed for ios in the car)*/
	if (gpio_is_valid(pdata->gpio_vbus)) {
		if (pdata->ci_udc.operating_mode == CI_UDC_DR_DUAL_HOST ||
			pdata->ci_udc.operating_mode == CI_UDC_DR_HOST)
			ret = gpio_request_one(pdata->gpio_vbus,
			                       GPIOF_OUT_INIT_HIGH,
			                       USB_DRV_NAME);
		else
			ret = gpio_request_one(pdata->gpio_vbus,
			                       GPIOF_OUT_INIT_LOW,
			                       USB_DRV_NAME);
		if (ret) {
			/*
			 * Set gpio_vbus to an invalid gpio line so that
			 * p7_exit_usb won't try to free it
			 * (since we failed to request it).
			 */
			pdata->gpio_vbus = -1;
			dev_err(&pdev->dev, "failed to enable vbus (%d)\n", ret);
			goto disable_usb;
		}
	}

	return 0;

disable_usb:
	p7mu_write16(pdata->phy_enable, 6);

	return ret;
}

static void p7_set_vbus(struct platform_device *pdev, int value)
{
	struct p7_usb2_platform_data const* const pdata = dev_get_platdata(&pdev->dev);

	if (gpio_is_valid(pdata->gpio_vbus))
		gpio_set_value_cansleep(pdata->gpio_vbus, value);
}

static void p7_exit_usb(struct platform_device *pdev)
{
	struct p7_usb2_platform_data const* const pdata = dev_get_platdata(&pdev->dev);

	if (!IS_ERR(pdata->pctl))
		pinctrl_put(pdata->pctl);

	if (gpio_is_valid(pdata->gpio_vbus)) {
		/* stop delivering VBUS to connector */
		gpio_set_value_cansleep(pdata->gpio_vbus, 0);
		gpio_free(pdata->gpio_vbus);
	}

	/* disable usb phy */
	p7mu_write16(pdata->phy_enable, 6);
};

static struct p7_usb2_platform_data p7brd_usb_pdata[] = {
	{   /* USB0 */
		.ci_udc.name = "ci_pdata",
		.ci_udc.capoffset = DEF_CAPOFFSET,
		.ci_udc.init = p7_init_usb,
		.ci_udc.exit = p7_exit_usb,
		.ci_udc.set_vbus = p7_set_vbus,
		.ci_udc.flags = CI13XXX_PULLUP_ON_VBUS,
		.phy_enable = 0xa00,
	},
	{   /* USB1 */
		.ci_udc.name = "ci_pdata",
		.ci_udc.capoffset = DEF_CAPOFFSET,
		.ci_udc.init = p7_init_usb,
		.ci_udc.exit = p7_exit_usb,
		.ci_udc.set_vbus = p7_set_vbus,
		.ci_udc.flags = CI13XXX_PULLUP_ON_VBUS,
		.phy_enable = 0xb00,
	}
};

#endif  /* #if defined(CONFIG_USB_CHIPIDEA_HOST) ||       \
               defined(CONFIG_USB_CHIPIDEA_UDC) */

#if defined(CONFIG_USB_CHIPIDEA_HOST)

/**
 * p7brd_init_hcd() - Instantiate USB controller identified by @bus
 *                    in host mode.
 *
 * @bus:        controller / bus identifier
 * @vbus_gpio:  optional gpio used enable downstream VBUS delivery
 */
void __init p7brd_init_hcd(int bus, int vbus_gpio)
{
	p7brd_init_usb(bus, vbus_gpio, CI_UDC_DR_HOST);
}

#endif  /* #if defined(CONFIG_USB_CHIPIDEA_HOST) */

#if defined(CONFIG_USB_CHIPIDEA_UDC)

/**
 * p7brd_init_udc() - Instantiate USB controller identified by @bus
 *                    in device mode.
 *
 * @bus:        controller / bus identifier
 * @vbus_gpio:  optional gpio used disable downstream VBUS delivery
 */
void __init p7brd_init_udc(int bus, int vbus_gpio)
{
	p7brd_init_usb(bus, vbus_gpio, CI_UDC_DR_DEVICE);
}
#endif  /* defined(CONFIG_USB_CHIPIDEA_UDC) */

#if defined(CONFIG_USB_CHIPIDEA_HOST) ||          \
    defined(CONFIG_USB_CHIPIDEA_UDC)
/**
 * p7brd_init_usb() - Instantiate USB controller identified by @bus.
 *
 * @bus:        controller / bus identifier
 * @mode:	host/device/dual
 * @vbus_gpio:  optional gpio used disable downstream VBUS delivery
 */
void __init p7brd_init_usb(int bus, int vbus_gpio,
			   enum ci_udc_operating_modes mode)
{
	struct p7_usb2_platform_data *const pdata = &p7brd_usb_pdata[bus];
	struct pinctrl_map *usb_pins;
	int nb_pins;

#ifdef DEBUG
	BUG_ON(bus >= ARRAY_SIZE(p7brd_usb_pdata));
	BUG_ON(bus < 0);
#endif

	//pdata->u.hcd.phy_mode = P7_USB2_PHY_ULPI;
	pdata->gpio_vbus = vbus_gpio;
	pdata->chip_rev = p7_chiprev();
	if(mode & CI_UDC_DR_DISABLE_HOST_WORKAROUND)
		pdata->ci_udc.workaround_reset_nb = 0; // Warn only mode
	else
		pdata->ci_udc.workaround_reset_nb = 5; // do 5 reset in a session, else let the controller die

	pdata->ci_udc.operating_mode = mode & ~CI_UDC_DR_DISABLE_HOST_WORKAROUND;

	if (bus == 0) {
		usb_pins = p7brd_usb0_pins;
		nb_pins = ARRAY_SIZE(p7brd_usb0_pins);
	} else {
		usb_pins = p7brd_usb1_pins;
		nb_pins = ARRAY_SIZE(p7brd_usb1_pins);
	}

	p7_init_ci_udc(bus, pdata, usb_pins, nb_pins);
}
#endif  /* #if defined(CONFIG_USB_CHIPIDEA_HOST) ||       \
               defined(CONFIG_USB_CHIPIDEA_UDC) */

/*******************
 * SDIO controllers
 *******************/

#if defined(CONFIG_MMC_SDHCI_ACS3) || \
    defined(CONFIG_MMC_SDHCI_ACS3_MODULE)
#include "sdhci.h"
#include "gbc.h"

/* Pins configurations, compatible with P7R3 backend static timing analysis */
static unsigned long p7_sdhci_pins_config_clk[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(5),
	P7CTL_SLR_CFG(2),
};

static unsigned long p7_sdhci_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
	P7CTL_DRV_CFG(4),
	P7CTL_SLR_CFG(2),
};

/* Pins maps */
static struct pinctrl_map p7_sdhci0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_0_CLK),
	P7_INIT_PINCFG(P7_SD_0_CLK, p7_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_0_CMD),
	P7_INIT_PINCFG(P7_SD_0_CMD, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT00),
	P7_INIT_PINCFG(P7_SD_0_DAT00, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT01),
	P7_INIT_PINCFG(P7_SD_0_DAT01, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT02),
	P7_INIT_PINCFG(P7_SD_0_DAT02, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_0_DAT03),
	P7_INIT_PINCFG(P7_SD_0_DAT03, p7_sdhci_pins_config)
};

static struct pinctrl_map p7_sdhci1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_1_CLK),
	P7_INIT_PINCFG(P7_SD_1_CLK, p7_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_1_CMD),
	P7_INIT_PINCFG(P7_SD_1_CMD, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT00),
	P7_INIT_PINCFG(P7_SD_1_DAT00, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT01),
	P7_INIT_PINCFG(P7_SD_1_DAT01, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT02),
	P7_INIT_PINCFG(P7_SD_1_DAT02, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_1_DAT03),
	P7_INIT_PINCFG(P7_SD_1_DAT03, p7_sdhci_pins_config)
};

static struct pinctrl_map p7_sdhci2_pins[] __initdata = {
	P7_INIT_PINMAP(P7_SD_2_CLK),
	P7_INIT_PINCFG(P7_SD_2_CLK, p7_sdhci_pins_config_clk),
	P7_INIT_PINMAP(P7_SD_2_CMD),
	P7_INIT_PINCFG(P7_SD_2_CMD, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_2_DAT00),
	P7_INIT_PINCFG(P7_SD_2_DAT00, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_2_DAT01),
	P7_INIT_PINCFG(P7_SD_2_DAT01, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_2_DAT02),
	P7_INIT_PINCFG(P7_SD_2_DAT02, p7_sdhci_pins_config),
	P7_INIT_PINMAP(P7_SD_2_DAT03),
	P7_INIT_PINCFG(P7_SD_2_DAT03, p7_sdhci_pins_config)
};

/* Regulators definitions */

#if (!defined(CONFIG_REGULATOR) || !defined(CONFIG_REGULATOR_SWITCH_VOLTAGE))
#error "CONFIG_REGULATOR && CONFIG_REGULATOR_SWITCH_VOLTAGE must be defined."
#endif

#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <regulator/switch-voltage-regulator.h>

static struct switch_voltage_state sdhci_no_switch_states[] = {
	{ .value = 3300000, .gpio = 0 },
};

static struct switch_voltage_state sdhci_switch_states[] = {
        { .value = 3300000, .gpio = 0 },
        { .value = 1800000, .gpio = 1 },
};

static struct switch_voltage_state sdhci_switch_low_states[] = {
        { .value = 3300000, .gpio = 1 },
        { .value = 1800000, .gpio = 0 },
};

static struct regulator_consumer_supply p7_sdhci0_vmmc_supply =
	REGULATOR_SUPPLY("vmmc", ACS3_DRV_NAME ".0");

static struct regulator_init_data p7_sdhci0_vmmc = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.min_uV         = 3300000,
		.max_uV         = 3300000,
		.always_on      = 1,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &p7_sdhci0_vmmc_supply,
};

static struct switch_voltage_config p7_sdhci0_vmmc_config = {
	.supply_name           = "sdhci0_card_vdd",
	.enable_gpio           = -1,
	.enable_high           = 1,
	.enabled_at_boot       = 0,
	.startup_delay         = 0,
	.switch_gpio           = -1,
	.states                = sdhci_no_switch_states,
	.init_data              = &p7_sdhci0_vmmc,
};

static struct platform_device p7_sdhci0_vmmc_pdev = {
	.name = "switch-voltage",
	.id   = 0,
	.dev  = {
		.platform_data = &p7_sdhci0_vmmc_config,
	},
};

static struct regulator_consumer_supply p7_sdhci1_vmmc_supply =
	REGULATOR_SUPPLY("vmmc", ACS3_DRV_NAME ".1");

static struct regulator_init_data p7_sdhci1_vmmc = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.min_uV         = 3300000,
		.max_uV         = 3300000,
		.always_on      = 1,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &p7_sdhci1_vmmc_supply,
};

static struct switch_voltage_config p7_sdhci1_vmmc_config = {
	.supply_name           = "sdhci1_card_vdd",
	.enable_gpio           = -1,
	.enable_high           = 1,
	.enabled_at_boot       = 0,
	.startup_delay         = 0,
	.switch_gpio           = -1,
	.states                = sdhci_no_switch_states,
	.init_data              = &p7_sdhci1_vmmc,
};

static struct platform_device p7_sdhci1_vmmc_pdev = {
	.name = "switch-voltage",
	.id   = 1,
	.dev  = {
		.platform_data = &p7_sdhci1_vmmc_config,
	},
};

static struct regulator_consumer_supply p7_sdhci2_vmmc_supply =
	REGULATOR_SUPPLY("vmmc", ACS3_DRV_NAME ".2");

static struct regulator_init_data p7_sdhci2_vmmc = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
		.min_uV         = 3300000,
		.max_uV         = 3300000,
		.always_on      = 1,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &p7_sdhci2_vmmc_supply,
};

static struct switch_voltage_config p7_sdhci2_vmmc_config = {
	.supply_name           = "sdhci2_card_vdd",
	.enable_gpio           = -1,
	.enable_high           = 1,
	.enabled_at_boot       = 0,
	.startup_delay         = 0,
	.switch_gpio           = -1,
	.states                = sdhci_no_switch_states,
	.init_data              = &p7_sdhci2_vmmc,
};

static struct platform_device p7_sdhci2_vmmc_pdev = {
	.name = "switch-voltage",
	.id   = 2,
	.dev  = {
		.platform_data = &p7_sdhci2_vmmc_config,
	},
};

static struct regulator_consumer_supply p7_sdhci0_vqmmc_supply =
       REGULATOR_SUPPLY("vqmmc", ACS3_DRV_NAME ".0");

static struct regulator_init_data p7_sdhci0_vqmmc = {
       .constraints = {
               .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				 REGULATOR_CHANGE_STATUS,
               .min_uV         = 3300000,
               .max_uV         = 3300000,
               .always_on       = 1,
       },
       .num_consumer_supplies  = 1,
       .consumer_supplies = &p7_sdhci0_vqmmc_supply,
};

static struct switch_voltage_config p7_sdhci0_vqmmc_config = {
	.supply_name           = "sdhci0_bus_vdd",
	.enable_gpio           = -1,
	.enable_high           = 1,
	.enabled_at_boot       = 0,
	.startup_delay         = 0,
	.switch_gpio           = -1,
	.switch_disabled       = 0,
	.switch_delay          = 0,
	.states                = sdhci_no_switch_states,
	.init_data             = &p7_sdhci0_vqmmc,
};

static struct platform_device p7_sdhci0_vqmmc_pdev = {
        .name = "switch-voltage",
        .id   = 3,
        .dev  = {
                .platform_data = &p7_sdhci0_vqmmc_config,
        },
};

static struct regulator_consumer_supply p7_sdhci1_vqmmc_supply =
       REGULATOR_SUPPLY("vqmmc", ACS3_DRV_NAME ".1");

static struct regulator_init_data p7_sdhci1_vqmmc = {
       .constraints = {
               .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				 REGULATOR_CHANGE_STATUS,
               .min_uV         = 3300000,
               .max_uV         = 3300000,
               .always_on       = 1,
       },
       .num_consumer_supplies  = 1,
       .consumer_supplies = &p7_sdhci1_vqmmc_supply,
};

static struct switch_voltage_config p7_sdhci1_vqmmc_config = {
	.supply_name           = "sdhci1_bus_vdd",
	.enable_gpio           = -1,
	.enable_high           = 1,
	.enabled_at_boot       = 0,
	.startup_delay         = 0,
	.switch_gpio           = -1,
	.switch_disabled       = 0,
	.switch_delay          = 0,
	.states                = sdhci_no_switch_states,
	.init_data             = &p7_sdhci1_vqmmc,
};

static struct platform_device p7_sdhci1_vqmmc_pdev = {
        .name = "switch-voltage",
        .id   = 4,
        .dev  = {
                .platform_data = &p7_sdhci1_vqmmc_config,
        },
};

static struct regulator_consumer_supply p7_sdhci2_vqmmc_supply =
       REGULATOR_SUPPLY("vqmmc", ACS3_DRV_NAME ".2");

static struct regulator_init_data p7_sdhci2_vqmmc = {
       .constraints = {
               .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE |
				 REGULATOR_CHANGE_STATUS,
               .min_uV         = 3300000,
               .max_uV         = 3300000,
               .always_on       = 1,
       },
       .num_consumer_supplies  = 1,
       .consumer_supplies = &p7_sdhci2_vqmmc_supply,
};

static struct switch_voltage_config p7_sdhci2_vqmmc_config = {
	.supply_name           = "sdhci2_bus_vdd",
	.enable_gpio           = -1,
	.enable_high           = 1,
	.enabled_at_boot       = 0,
	.startup_delay         = 0,
	.switch_gpio           = -1,
	.switch_disabled       = 0,
	.switch_delay          = 0,
	.states                = sdhci_no_switch_states,
	.init_data             = &p7_sdhci2_vqmmc,
};

static struct platform_device p7_sdhci2_vqmmc_pdev = {
        .name = "switch-voltage",
        .id   = 5,
        .dev  = {
                .platform_data = &p7_sdhci2_vqmmc_config,
        },
};

/**
 * p7brd_init_sdhci_card_regulator() - Instantiate regulator for SDHCI driver if no
 *                                specific was defined
 *
 * @reg_gpios:  regulators GPIOs
 * @pdev:       regulator pdev
 * @pdata:      platform data
 */
void __init p7brd_init_sdhci_card_regulator(struct acs3_regulator_gpios *reg_gpios,
					struct platform_device *pdev,
					struct acs3_plat_data *pdata)
{
	struct switch_voltage_config *vmmc_config = (pdev->dev).platform_data;

	if (reg_gpios && reg_gpios->card_enable_gpio) {
		vmmc_config->enable_gpio = reg_gpios->card_enable_gpio;
		vmmc_config->enable_high = !reg_gpios->card_enable_active_low;
		vmmc_config->startup_delay = reg_gpios->card_enable_delay;
		vmmc_config->init_data->constraints.always_on = 0;

		pdata->brd_ocr = MMC_VDD_32_33 | MMC_VDD_33_34;

		p7_init_dev(pdev, NULL, NULL, 0);
	}
}

/**
 * p7brd_init_sdhci_bus_regulator() - Instantiate regulator for SDHCI driver if no
 *                                specific was defined
 *
 * @reg_gpios:  GPIOs for gpio-regulator
 * @pdev:       regulator pdev
 */
void __init p7brd_init_sdhci_bus_regulator(struct acs3_regulator_gpios* reg_gpios,
					struct platform_device* pdev)
{
	struct switch_voltage_config *vqmmc_config = (pdev->dev).platform_data;

	if (reg_gpios) {
		struct regulator_init_data *vqmmc_init_data = vqmmc_config->init_data;

		if (reg_gpios->bus_1v8_gpio) {
			vqmmc_init_data->constraints.min_uV = 1800000;
			vqmmc_config->switch_gpio = reg_gpios->bus_1v8_gpio;
			vqmmc_config->switch_delay = reg_gpios->bus_switch_delay;
			vqmmc_config->switch_disabled = reg_gpios->bus_switch_disabled;
			if (reg_gpios->bus_1v8_active_low) {
				vqmmc_config->states = sdhci_switch_low_states;
			} else {
				vqmmc_config->states = sdhci_switch_states;
			}
		}
		if (reg_gpios->bus_enable_gpio) {
			vqmmc_config->enable_gpio = reg_gpios->bus_enable_gpio;
			vqmmc_config->enable_high = !reg_gpios->bus_enable_active_low;
		}
	}

	p7_init_dev(pdev, NULL, NULL, 0);
}

/********************************
 * SDHCI configuration functions
 *******************************/

/**
 * acs3_default_config_r1 - array of default modes configurations for P7R1
 */
static struct acs3_mode_config acs3_default_config_r1[] = {
	{
		.mode          = ACS3_DEFAULT_SPEED,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_HIGH_SPEED,
		.max_Hz        = 0,
		.tdl1          = 0x31f,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_UHS_SDR,
		.max_Hz        = 0,
		.tdl1          = 0x311,
		.tdl2          = 1,
		.tuning_success_count = 10,
	},
	{
		.mode          = ACS3_UHS_DDR50,
		.max_Hz        = 0,
		.tdl1          = 0x319,
		.tdl2          = 0x329,
	},
};

/**
 * acs3_default_config_r2_sdhci0 - array of default modes configurations for
 * P7R2 HOST 0
 */
static struct acs3_mode_config acs3_default_config_r2_sdhci0[] = {
	{
		.mode          = ACS3_DEFAULT_SPEED | ACS3_UHS_SDR12 |
				 ACS3_UHS_SDR25,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_HIGH_SPEED,
		.max_Hz        = 0,
		.tdl1          = 0x219,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_UHS_SDR50 | ACS3_UHS_SDR104,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 1,
		.tuning_success_count = 10,
	},
	{
		.mode          = ACS3_UHS_DDR50,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0x03D,
	},
};

/**
 * acs3_default_config_r2_sdhci1 - array of default modes configurations for
 * P7R2 HOST 1
 */
static struct acs3_mode_config acs3_default_config_r2_sdhci1[] = {
	{
		.mode          = ACS3_DEFAULT_SPEED | ACS3_UHS_SDR12 |
				 ACS3_UHS_SDR25,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_HIGH_SPEED,
		.max_Hz        = 0,
		.tdl1          = 0x219,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_UHS_SDR50 | ACS3_UHS_SDR104,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 1,
		.tuning_success_count = 10,
	},
	{
		.mode          = ACS3_UHS_DDR50,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0x03F,
	},
};

/**
 * acs3_default_config_r2_sdhci2 - array of default modes configurations for
 * P7R2 HOST 2
 */
static struct acs3_mode_config acs3_default_config_r2_sdhci2[] = {
	{
		.mode          = ACS3_DEFAULT_SPEED | ACS3_UHS_SDR12 |
				 ACS3_UHS_SDR25,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_HIGH_SPEED,
		.max_Hz        = 0,
		.tdl1          = 0x219,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_UHS_SDR50 | ACS3_UHS_SDR104,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 1,
		.tuning_success_count = 10,
	},
	{
		.mode          = ACS3_UHS_DDR50,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0x045,
	},
};

/**
 * acs3_default_config_r3 - array of default modes configurations for P7R3
 */
static struct acs3_mode_config acs3_default_config_r3[] = {
	{
		.mode          = ACS3_DEFAULT_SPEED | ACS3_UHS_SDR12 |
				 ACS3_UHS_SDR25,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_HIGH_SPEED,
		.max_Hz        = 0,
		.tdl1          = 0x219,
		.tdl2          = 0,
	},
	{
		.mode          = ACS3_UHS_SDR50 | ACS3_UHS_SDR104,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 1,
		.tuning_success_count = 10,
	},
	{
		.mode          = ACS3_UHS_DDR50,
		.max_Hz        = 0,
		.tdl1          = 0,
		.tdl2          = 0x7,
	},
};

#ifdef CONFIG_MMC_SDHCI_ACS3_DEBUGFS
#include <linux/debugfs.h>

static int sdhci_tdl1_get(void *data, u64 *val)
{
	struct acs3_mode_config *config = (struct acs3_mode_config *)data;
	*val = config->tdl1;
	return 0;
}

static int sdhci_tdl1_set(void *data, u64 val)
{
	struct acs3_mode_config *config = (struct acs3_mode_config *)data;
	config->tdl1 = val;
	return 0;
}

static int sdhci_tdl2_get(void *data, u64 *val)
{
	struct acs3_mode_config *config = (struct acs3_mode_config *)data;
	*val = config->tdl2;
	return 0;
}

static int sdhci_tdl2_set(void *data, u64 val)
{
	struct acs3_mode_config *config = (struct acs3_mode_config *)data;
	config->tdl2 = val;
	return 0;
}

static int sdhci_maxclk_get(void *data, u64 *val)
{
	struct acs3_mode_config *config = (struct acs3_mode_config *)data;
	*val = config->max_Hz;
	return 0;
}

static int sdhci_maxclk_set(void *data, u64 val)
{
	struct acs3_mode_config *config = (struct acs3_mode_config *)data;
	config->max_Hz = val;
	return 0;
}

#define THIS_MODULE ((struct module *)0)
DEFINE_SIMPLE_ATTRIBUTE(sdhci_tdl1_fops, sdhci_tdl1_get, sdhci_tdl1_set,
			"0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(sdhci_tdl2_fops, sdhci_tdl2_get, sdhci_tdl2_set,
			"0x%llx\n");
DEFINE_SIMPLE_ATTRIBUTE(sdhci_maxclk_fops, sdhci_maxclk_get, sdhci_maxclk_set,
			"%lld\n");

static void sdhci_params_mode_create_debugfs(char *name,
					     struct acs3_mode_config *config,
					     struct dentry *parent)
{
	struct dentry *root;
	root = debugfs_create_dir(name, parent);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		printk("failed to initialize debugfs\n");

	if (!debugfs_create_file("tdl1", S_IRUSR | S_IWUSR, root, config,
			&sdhci_tdl1_fops))
		debugfs_remove_recursive(root);

	if (!debugfs_create_file("tdl2", S_IRUSR | S_IWUSR, root, config,
			&sdhci_tdl2_fops))
		debugfs_remove_recursive(root);

	if (!debugfs_create_file("maxclk", S_IRUSR | S_IWUSR, root, config,
			&sdhci_maxclk_fops))
		debugfs_remove_recursive(root);
}

static void sdhci_params_root_create_debugfs(char *name,
					     struct acs3_mode_config *config)
{
	struct dentry *root;
	root = debugfs_create_dir(name, NULL);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		return;

	sdhci_params_mode_create_debugfs("default_mode", &config[0] ,root);
	sdhci_params_mode_create_debugfs("hs_mode",      &config[1] ,root);
	sdhci_params_mode_create_debugfs("sdr104_mode",  &config[2] ,root);
	sdhci_params_mode_create_debugfs("ddr50_mode",   &config[3] ,root);
}

#else

static inline void sdhci_params_root_create_debugfs(char *name,
                                                    struct acs3_mode_config *config)
{
	return;
}

#endif

/**
 * p7brd_init_sdhci() - Instantiate SDHCI host controller identified by @host
 *                      for further driver usage.
 *
 * @host:            host controller
 * @pdata:           platform data
 * @card_regulator:  specific card regulator
 * @reg_gpios:       regulators GPIOs
 */
void __init p7brd_init_sdhci(int host, struct acs3_plat_data *pdata,
				struct platform_device *card_regulator,
				struct platform_device *bus_regulator,
				struct acs3_regulator_gpios *reg_gpios,
				struct pinctrl_map *board_pins,
				size_t board_pins_cnt)
{
	struct pinctrl_map* pins = board_pins;
	size_t pins_cnt = board_pins_cnt;

	if (card_regulator) p7_init_dev(card_regulator, NULL, NULL, 0);
	if (bus_regulator)  p7_init_dev(bus_regulator, NULL, NULL, 0);

	switch(p7_chiprev()) {
	case P7_CHIPREV_R1:
		pdata->default_mode_config = acs3_default_config_r1;
		sdhci_params_root_create_debugfs("sdhci_params",
						 acs3_default_config_r1);
		break;
	case P7_CHIPREV_R2:
		pdata->disable_ddr50 = 1;
		break;
	case P7_CHIPREV_R3:
		pdata->default_mode_config = acs3_default_config_r3;
		sdhci_params_root_create_debugfs("sdhci_params",
						 acs3_default_config_r3);
		break;
	}

	switch (host) {
	case 0:
		if (p7_chiprev() == P7_CHIPREV_R2) {
			pdata->default_mode_config = acs3_default_config_r2_sdhci0;
			sdhci_params_root_create_debugfs("sdhci0_params",
							 acs3_default_config_r2_sdhci0);
		}

		if (!card_regulator)
			p7brd_init_sdhci_card_regulator(reg_gpios,
							&p7_sdhci0_vmmc_pdev,
							pdata);
		if (!bus_regulator)
			p7brd_init_sdhci_bus_regulator(reg_gpios,
							&p7_sdhci0_vqmmc_pdev);
		if (!pins) {
			pins = p7_sdhci0_pins;
			pins_cnt = ARRAY_SIZE(p7_sdhci0_pins);
		}
		break;
	case 1:
		if (p7_chiprev() == P7_CHIPREV_R2) {
			pdata->default_mode_config = acs3_default_config_r2_sdhci1;
			sdhci_params_root_create_debugfs("sdhci1_params",
							 acs3_default_config_r2_sdhci1);
		}

		if (!card_regulator)
			p7brd_init_sdhci_card_regulator(reg_gpios,
							&p7_sdhci1_vmmc_pdev,
							pdata);
		if (!bus_regulator)
			p7brd_init_sdhci_bus_regulator(reg_gpios,
							&p7_sdhci1_vqmmc_pdev);
		if (!pins) {
			pins = p7_sdhci1_pins;
			pins_cnt = ARRAY_SIZE(p7_sdhci1_pins);
		}
		break;
	case 2:
		if (p7_chiprev() == P7_CHIPREV_R2) {
			pdata->default_mode_config = acs3_default_config_r2_sdhci2;
			sdhci_params_root_create_debugfs("sdhci2_params",
							 acs3_default_config_r2_sdhci2);
		}

		if (!card_regulator)
			p7brd_init_sdhci_card_regulator(reg_gpios,
							&p7_sdhci2_vmmc_pdev,
							pdata);
		if (!bus_regulator)
			p7brd_init_sdhci_bus_regulator(reg_gpios,
							&p7_sdhci2_vqmmc_pdev);
		if (!pins) {
			pins = p7_sdhci2_pins;
			pins_cnt = ARRAY_SIZE(p7_sdhci2_pins);
		}
		break;
	}
	return p7_init_sdhci(host, pdata, pins, pins_cnt);
}

#endif  /* defined(CONFIG_MMC_SDHCI_ACS3) || \
           defined(CONFIG_MMC_SDHCI_ACS3_MODULE) */

/******************
 * NAND controller
 ******************/

#if defined(CONFIG_MTD_NAND_CAST) || \
    defined(CONFIG_MTD_NAND_CAST_MODULE)
#include <linux/mtd/partitions.h>
#include <nand/cast-nand.h>
#include "nand.h"

static struct mtd_partition const p7brd_nand_parts[] = {
	{
		.name   = "bootloader",
		.offset = 0,
		.size   = SZ_8M
	},
	{
		.name   = "kernel",
		.offset = MTDPART_OFS_NXTBLK,
		.size   = SZ_16M
	},
	{
		.name   = "factory",
		.offset = MTDPART_OFS_NXTBLK,
		.size   = 12 * SZ_1M
	},
	{
		.name   = "system",
		.offset = MTDPART_OFS_NXTBLK,
		.size   = SZ_128M
	},
	{
		.name   = "app",
		.offset = MTDPART_OFS_NXTBLK,
		.size   = MTDPART_SIZ_FULL
	}
};

static struct cast_plat_data p7brd_nand_pdata = {
	.parts      = p7brd_nand_parts,
	.parts_nr   = ARRAY_SIZE(p7brd_nand_parts)
};

/* Pull up needed on bebop */
static unsigned long fc7100_nand_pinconf_rnb[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(UP)    | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3(reg=15) */
};

static unsigned long fc7100_nand_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(3),      /* Drive strength 3(reg=15) */
};

static struct pinctrl_map p7brd_nand_pins[] __initdata = {
	P7_INIT_PINMAP(P7_NAND_NCE),
	P7_INIT_PINCFG(P7_NAND_NCE, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_NR),
	P7_INIT_PINCFG(P7_NAND_NR, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_NW),
	P7_INIT_PINCFG(P7_NAND_NW, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_AL),
	P7_INIT_PINCFG(P7_NAND_AL, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_CL),
	P7_INIT_PINCFG(P7_NAND_CL, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_RNB),
	P7_INIT_PINCFG(P7_NAND_RNB, fc7100_nand_pinconf_rnb),
	P7_INIT_PINMAP(P7_NAND_DATA_00),
	P7_INIT_PINCFG(P7_NAND_DATA_00, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_01),
	P7_INIT_PINCFG(P7_NAND_DATA_01, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_02),
	P7_INIT_PINCFG(P7_NAND_DATA_02, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_03),
	P7_INIT_PINCFG(P7_NAND_DATA_03, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_04),
	P7_INIT_PINCFG(P7_NAND_DATA_04, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_05),
	P7_INIT_PINCFG(P7_NAND_DATA_05, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_06),
	P7_INIT_PINCFG(P7_NAND_DATA_06, fc7100_nand_pinconf),
	P7_INIT_PINMAP(P7_NAND_DATA_07),
	P7_INIT_PINCFG(P7_NAND_DATA_07, fc7100_nand_pinconf),
	/*P7_INIT_PINMAP(P7_NAND_WP),
	  P7_INIT_PINCFG(P7_NAND_WP, fc7100_nand_pinconf),*/
	P7_INIT_PINMAP(P7_NAND_DQS),
	P7_INIT_PINCFG(P7_NAND_DQS, fc7100_nand_pinconf)
};

/**
 * p7brd_init_nand - Instantiate NAND flash controller
 *                     for further driver usage.
 */
void __init p7brd_init_nand(int synchrone_nand)
{
	p7brd_nand_pdata.chip_rev = p7_chiprev();
	p7_init_nand(&p7brd_nand_pdata,
	             p7brd_nand_pins,
	             ARRAY_SIZE(p7brd_nand_pins) + (synchrone_nand ? 0: -2));
	/*
	 * Write protect pin is shared by NAND and NOR flashes. It is active
	 * low for both devices: enable writes.
	 * Don't reserve WP pin usage so that userland may use both devices
	 * alternatively.
	 */
	if (gpio_request_one(42, GPIOF_OUT_INIT_HIGH, "flash wp"))
		pr_warn("failed to disable NAND / NOR flash write protect\n");
}

#endif  /* defined(CONFIG_MTD_NAND_CAST) || \
           defined(CONFIG_MTD_NAND_CAST_MODULE) */

/*******
 * UARTs
 *******/

#if defined(CONFIG_SERIAL_PARROTX) || \
    defined(CONFIG_SERIAL_PARROTX_MODULE)

#include "uart.h"

static unsigned long fc7100_uart_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(HIGHZ) | /* no pull up/down unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static unsigned long fc7100_uart_up_pinconf[] = {
	P7CTL_SMT_CFG(OFF)   | /* no shimmt trigger */
	P7CTL_PUD_CFG(UP) | /* pull up unable */
	P7CTL_SLR_CFG(3)     | /* Slew rate 3 */
	P7CTL_DRV_CFG(1),      /* Drive strength 1 (reg=3) */
};

static struct pinctrl_map p7brd_uart0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_0_RX),
	P7_INIT_PINCFG(P7_UART_0_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_0_TX),
	P7_INIT_PINCFG(P7_UART_0_TX, fc7100_uart_pinconf),
	P7_INIT_PINMAP(P7_UART_0_RTS),
	P7_INIT_PINCFG(P7_UART_0_RTS, fc7100_uart_pinconf),
	P7_INIT_PINMAP(P7_UART_0_CTS),
	P7_INIT_PINCFG(P7_UART_0_CTS, fc7100_uart_up_pinconf)
};

static struct pinctrl_map p7brd_uart1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_1_RX),
	P7_INIT_PINCFG(P7_UART_1_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_1_TX),
	P7_INIT_PINCFG(P7_UART_1_TX, fc7100_uart_pinconf),
	P7_INIT_PINMAP(P7_UART_1_RTS),
	P7_INIT_PINCFG(P7_UART_1_RTS, fc7100_uart_pinconf),
	P7_INIT_PINMAP(P7_UART_1_CTS),
	P7_INIT_PINCFG(P7_UART_1_CTS, fc7100_uart_up_pinconf)
};

static struct pinctrl_map p7brd_uart2_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_2_RX),
	P7_INIT_PINCFG(P7_UART_2_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_2_TX),
	P7_INIT_PINCFG(P7_UART_2_TX, fc7100_uart_pinconf)
};

static struct pinctrl_map p7brd_uart3_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_3_RX),
	P7_INIT_PINCFG(P7_UART_3_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_3_TX),
	P7_INIT_PINCFG(P7_UART_3_TX, fc7100_uart_pinconf)
};

static struct pinctrl_map p7brd_uart4_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_4_RX),
	P7_INIT_PINCFG(P7_UART_4_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_4_TX),
	P7_INIT_PINCFG(P7_UART_4_TX, fc7100_uart_pinconf)
};

static struct pinctrl_map p7brd_uart5_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_5_RX),
	P7_INIT_PINCFG(P7_UART_5_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_5_TX),
	P7_INIT_PINCFG(P7_UART_5_TX, fc7100_uart_pinconf)
};

static struct pinctrl_map p7brd_uart6_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_6_RX),
	P7_INIT_PINCFG(P7_UART_6_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_6_TX),
	P7_INIT_PINCFG(P7_UART_6_TX, fc7100_uart_pinconf)
};

static struct pinctrl_map p7brd_uart7_pins[] __initdata = {
	P7_INIT_PINMAP(P7_UART_7_RX),
	P7_INIT_PINCFG(P7_UART_7_RX, fc7100_uart_up_pinconf),
	P7_INIT_PINMAP(P7_UART_7_TX),
	P7_INIT_PINCFG(P7_UART_7_TX, fc7100_uart_pinconf)
};

void __init p7brd_init_uart(int uart, int rts_cts)
{
	switch(uart) {
	case 0:
		p7_init_uart(0, p7brd_uart0_pins, rts_cts?ARRAY_SIZE(p7brd_uart0_pins):ARRAY_SIZE(p7brd_uart0_pins)/2);
		break;
	case 1:
		p7_init_uart(1, p7brd_uart1_pins, rts_cts?ARRAY_SIZE(p7brd_uart1_pins):ARRAY_SIZE(p7brd_uart1_pins)/2);
		break;
	case 2:
		p7_init_uart(2, p7brd_uart2_pins, ARRAY_SIZE(p7brd_uart2_pins));
		break;
	case 3:
		p7_init_uart(3, p7brd_uart3_pins, ARRAY_SIZE(p7brd_uart3_pins));
		break;
	case 4:
		p7_init_uart(4, p7brd_uart4_pins, ARRAY_SIZE(p7brd_uart4_pins));
		break;
	case 5:
		p7_init_uart(5, p7brd_uart5_pins, ARRAY_SIZE(p7brd_uart5_pins));
		break;
	case 6:
		p7_init_uart(6, p7brd_uart6_pins, ARRAY_SIZE(p7brd_uart6_pins));
		break;
	case 7:
		p7_init_uart(7, p7brd_uart7_pins, ARRAY_SIZE(p7brd_uart7_pins));
		break;
	}

	p7brd_export_uart_hw_infos(uart, rts_cts, "");
}

#endif


struct platform_device user_gpio = {
	.name          = "user_gpio",
	.id            = -1,
	.dev = {
		.platform_data = NULL,
	},
};

/**
 * p7brd_init_named_gpios() - Instantiate pseudo-driver for named_gpios
 *
 */
void __init p7brd_init_named_gpios()
{
	p7_init_dev(&user_gpio, NULL, NULL, 0);
}


/**
 * p7brd_export_gpio() - Expose a gpio line to userspace through sysfs
 *
 * @gpio:   gpio line
 * @flags:  GPIOF_* flags
 * @label:  arbitrary name given to gpio line
 *
 * An invalid @gpio will be silently ignored purposedly.
 */
int p7brd_export_gpio(int gpio, unsigned int flags, char const* label)
{
	int err;

	if (! gpio_is_valid(gpio)) {
		return -EINVAL;
	}

	err = gpio_request_one((unsigned) gpio, flags, label);
	if (err) {
		pr_err("p7brd: failed to request GPIO %d for \"%s\" (%d)\n",
		       gpio,
		       label,
		       err);
		return err;
	}

	/* gpio line MUST refer to a valid & instantiated gpio chip ! */
	BUG_ON(gpio_export((unsigned) gpio, 0));

	if (label) {
		/* Export GPIO using peudo-driver "user_gpio" */
		gpio_export_link(&user_gpio.dev, label, gpio);
	}

	return 0;
}

void __init p7brd_unexport_gpio(int gpio)
{
	if (gpio_is_valid(gpio)) {
		gpio_unexport(gpio);
		gpio_free(gpio);
	}
}

int parrot_gpio_in_init(int gpio, int pincfg, const char *name)
{
	int r;

	r = gpio_request_one((unsigned) gpio, GPIOF_IN, name);
	if (r) {
		pr_warn("Could not request GPIO %d\n", gpio);
		return r;
	}
	r = p7_config_pin(gpio, pincfg);
	if (r)
		pr_warn("Could not set GPIO %d pull-up/down\n", gpio);
	return r;
}

int parrot_gpio_user_in_init(int gpio, int pincfg, const char *alias)
{
	int r;
	r = parrot_gpio_in_init(gpio, pincfg, alias);
	if (r)
	  return r;

	gpio_export(gpio, 0);
	if (alias != NULL) {
		printk(KERN_INFO "Export user in GPIO %d with alias '%s'\n", gpio, alias);
		gpio_export_link(&user_gpio.dev, alias, gpio);
	}
	else {
		printk(KERN_INFO "Export user in GPIO %d without alias\n", gpio);
	}
	return 0;
}

#include "c_can.h"

/* disable internal pull up */
static unsigned long ccan_pins_config[] = {
	P7CTL_PUD_CFG(HIGHZ),
};

/* CCAN0 pins */
static struct pinctrl_map p7brd_ccan0_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAN0_TX),
	P7_INIT_PINCFG(P7_CAN0_TX, ccan_pins_config),
	P7_INIT_PINMAP(P7_CAN0_RX),
	P7_INIT_PINCFG(P7_CAN0_RX, ccan_pins_config),
};

/* CCAN1 pins */
static struct pinctrl_map p7brd_ccan1_pins[] __initdata = {
	P7_INIT_PINMAP(P7_CAN1_TX),
	P7_INIT_PINCFG(P7_CAN1_TX, ccan_pins_config),
	P7_INIT_PINMAP(P7_CAN1_RX),
	P7_INIT_PINCFG(P7_CAN1_RX, ccan_pins_config),
};

/**
 * p7brd_init_ccan() - Instantiate C_CAN IP Module Interface indentified by bus
 * identifier
 *
 * @bus:    ccan bus identifier
 */
void __init p7brd_init_ccan(int bus)
{
	BUG_ON(bus < 0);

	switch (bus) {
	case 0:
		p7_init_c_can(0,
		             p7brd_ccan0_pins,
		             ARRAY_SIZE(p7brd_ccan0_pins));
		break;
	case 1:
		p7_init_c_can(1,
		             p7brd_ccan1_pins,
		             ARRAY_SIZE(p7brd_ccan1_pins));
		break;
	}
}
