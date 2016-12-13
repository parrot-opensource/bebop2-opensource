/**
 * linux/drivers/parrot/pinctrl/p7-pinctrl.c - Parrot7 pin controller driver
 *                                             implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Gregor Boirie <gregor.boirie@parrot.com>
 * date:    01-Jun-2012
 *
 * This file is released under the GPL
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include "p7-pinctrl.h"
#include "../../pinctrl/core.h"

/*****************************************************************************
 * Parrot7 chips pinctrl driver: implements I/O pin space handling.
 *
 * Pin / group mapping is particular here since our hardware allows
 * us to multiplex every controllable pins using up to 4 mutually exclusive
 * functions. There is no notion of per group multiplexing at hardware level.
 * However, pinctrl subsystem requires driver to provide groups of pins to
 * support I/O multiplexing.
 * Hence, an identity mapping between hardware pin and corresponding fake
 * software / logical group is carried out through the use of pinctrl_ops
 * operation pointers.
 * Hardware pin multiplexing is managed by assigning up to 4 pinmux functions
 * for each logical group (i.e., per physical pin thanks to identity map).
 *
 * Althought I could have grouped pins in an arbitrary manner to reduce number
 * of multiplexing entities, this would not fit current needs since we rely on
 * a flexible way to support future boards (we have not heard of yet) without
 * breaking existing platform dependent pinctrl code which instantiates
 * pins / groups / functions.
 * Moreover, I wanted to avoid misconfigurations between drivers and platform
 * pinctrl code for IPs which have the ability to multiplex their own outputs
 * to chip pins (like AAI, SPI...). It seems to me the simplest way to achieve
 * this, is to enforce a direct group / pin mapping at pinctrl level and
 * perform complex multiplexing inside the IP / driver (since it is needed any
 * way).
 ****************************************************************************/

/* I/O remapped address of register space */
static u32 volatile __iomem*            p7ctl_vaddr;
/* Physical address of register space */
static phys_addr_t                      p7ctl_paddr;
/* Size of register space */
static resource_size_t                  p7ctl_sz;
/* Pinctrl platform device */
static struct platform_device const*    p7ctl_pdev;
/* Pinctrl device */
static struct pinctrl_dev*              p7ctl_dev;
/* Platform specific data given at loading time */
static struct p7ctl_plat_data const*    p7ctl_pdata;
#ifdef CONFIG_PM_SLEEP
/* Memory area to store registers */
static u32*                             p7ctl_saved_regs;
#endif

#ifdef DEBUG
#define P7CTL_ASSERT_PIN(_pin_id)                           \
    BUG_ON(! p7ctl_pdata->pins);                            \
    BUG_ON(! p7ctl_pdata->pins_nr);                         \
    BUG_ON(_pin_id >= p7ctl_pdata->pins_nr);                \
    BUG_ON(p7ctl_pdata->pins[_pin_id].number != _pin_id);   \
    BUG_ON(! p7ctl_pdata->pins[_pin_id].name)
#else
#define P7CTL_ASSERT_PIN(_pin_id)
#endif

/************************
 * "Dummy" functions handling
 ************************/

/* Not all functions exist on all chip revisions, to simplify things we just
 * fill the blanks with "dummy" pins */
static inline int p7ctl_func_available(int func_id)
{
	BUG_ON(func_id >= p7ctl_pdata->funcs_nr);

	if (p7ctl_pdata->funcs[func_id].name != NULL) {
#ifdef DEBUG
		BUG_ON(! p7ctl_pdata->funcs[func_id].name);
		BUG_ON(p7ctl_pdata->funcs[func_id].mux_id >= 4);
		BUG_ON(p7ctl_pdata->funcs[func_id].mux_id == 1);
		P7CTL_ASSERT_PIN(p7ctl_pdata->funcs[func_id].pin_id);
#endif
		return 1;
	}

	return 0;
}

/******************
 * Fake pin groups
 ******************/

/* Does fake group exists (i.e., physical pin) ? */
static int p7ctl_list_grp(struct pinctrl_dev* pctl, unsigned int grp_id)
{
	if (grp_id >= p7ctl_pdata->pins_nr)
		return -EINVAL;

	P7CTL_ASSERT_PIN(grp_id);
	return 0;
}

/* Returns fake group name, i.e., physical pin name provided by platform. */
static char const* p7ctl_grp_name(struct pinctrl_dev* pctl, unsigned int grp_id)
{
	if (grp_id >= p7ctl_pdata->pins_nr)
		return NULL;

	P7CTL_ASSERT_PIN(grp_id);
	return p7ctl_pdata->pins[grp_id].name;
}

/* Return the single physical pin attached to fake group. */
static int p7ctl_grp_pins(struct pinctrl_dev* pctl,
                          unsigned int grp_id,
                          unsigned int const** pins,
                          unsigned int* pins_nr)
{
	if (grp_id >= p7ctl_pdata->pins_nr)
		return -EINVAL;

	P7CTL_ASSERT_PIN(grp_id);

	*pins = &p7ctl_pdata->pins[grp_id].number;
	*pins_nr = 1;
	return 0;
}

#ifdef CONFIG_DEBUG_FS

static void p7ctl_show_pin(struct pinctrl_dev* pctl,
                           struct seq_file* file,
                           unsigned pin_id)
{
	seq_printf(file, " " P7CTL_DRV_NAME);
}

#else
#define p7ctl_show_pin  NULL
#endif

static struct pinctrl_ops p7ctl_grp_ops = {
	.list_groups    = p7ctl_list_grp,
	.get_group_name = p7ctl_grp_name,
	.get_group_pins = p7ctl_grp_pins,
	.pin_dbg_show   = p7ctl_show_pin
};

/********************
 * Pin configuration
 ********************/

static union p7ctl_setting p7ctl_cfg2set(unsigned long config)
{
	return *((union p7ctl_setting*) (&config));
}

/* Check pin setting consistency. */
static int p7ctl_check_set(union p7ctl_setting settings)
{
       /* Check pull up / down consistency. */
       switch (settings.fields.pud) {
       case P7CTL_PUD_HIGHZ:
       case P7CTL_PUD_UP:
       case P7CTL_PUD_DOWN:
       case P7CTL_PUD_KEEP:
               break;

       default:
               return -EINVAL;
       }

       /* Check slewrate consistency. */
       if (settings.fields.slr > P7CTL_SLR_3)
               return -EINVAL;

       /* Check drive strength consistency. */
       switch (settings.fields.drv) {
       case P7CTL_DRV_TRI:
       case P7CTL_DRV_0:
       case P7CTL_DRV_1:
       case P7CTL_DRV_2:
       case P7CTL_DRV_3:
       case P7CTL_DRV_4:
       case P7CTL_DRV_5:
               break;

       default:
               return -EINVAL;
       }

       return 0;
}

static int p7ctl_get_cfg(struct pinctrl_dev* pctl,
                         unsigned int pin_id,
                         unsigned long* config)
{
	P7CTL_ASSERT_PIN(pin_id);

	*config = (unsigned long) __raw_readl(p7ctl_vaddr + pin_id);
	return 0;
}

static int p7ctl_set_cfg(struct pinctrl_dev* pctl,
                         unsigned int pin_id,
                         unsigned long config)
{
	u32 volatile __iomem* const addr = p7ctl_vaddr + pin_id;
	union p7ctl_setting         cmd = { .word = __raw_readl(addr) };
	union p7ctl_setting const   set = p7ctl_cfg2set(config);
	int const                   err = p7ctl_check_set(set);

	P7CTL_ASSERT_PIN(pin_id);
	if (err)
		return err;

	/* Setup Schmitt trigger. */
	if (set.fields.set_smt)
		cmd.fields.smt = set.fields.smt;

	/* Setup pull up/down. */
	if (set.fields.set_pud)
		cmd.fields.pud = set.fields.pud;

	/* Setup slew rate. */
	if (set.fields.set_slr)
		cmd.fields.slr = set.fields.slr;

	/* Setup drive strength. */
	if (set.fields.set_drv)
		cmd.fields.drv = set.fields.drv;

	__raw_writel(cmd.word, addr);
	return 0;
}

#ifdef CONFIG_DEBUG_FS

static void p7ctl_show_pincfg(struct pinctrl_dev* pctl,
                              struct seq_file* file,
                              unsigned int pin_id)
{
	union p7ctl_setting const   set = { .word = __raw_readl(p7ctl_vaddr +
	                                                          pin_id) };

	P7CTL_ASSERT_PIN(pin_id);

#define ON_OFF(_b) ((_b) ? "on" : "off")

	seq_printf(file, " schmitt(%s) pull_up(%s) pull_down(%s) bus_keeper(%s)",
		   ON_OFF(set.fields.smt),
		   ON_OFF(set.fields.pud & 1),
		   ON_OFF(set.fields.pud & 2),
		   ON_OFF(set.fields.pud & 4));

	seq_printf(file, " slew_rate(%u)", set.fields.slr);
	seq_printf(file, " drive_strength(%u)", set.fields.drv);
}

/*
 * Callback for writes to the pincfg debugfs.
 *
 * buf is a null terminated string
 *
 * Format samples:
 *
 * echo '170:drive_strength(31)' > pinconf-pins
 * echo '170:drive_strength:31'  > pinconf-pins
 * echo '170:pull_up:on'         > pinconf-pins
 */
static int p7ctl_set_pincfg(struct pinctrl_dev* pctl, char *buf)
{
	union p7ctl_setting 	 set;
	char			*last;
	char			*function;
	unsigned		 pin;
	unsigned		 val;

	pin = simple_strtoul(buf, &last, 0);
	if (*last++ != ':')
		return -EINVAL;

	function = last;

	for (;;) {
		if (*last == '\0')
			/* Unexpected end of string */
			return -EINVAL;

		if (*last == '(' || *last == ':') {
			*last = '\0';
			break;
		}

		last++;
	}

	last++;

	switch (*last) {
	case '0':
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		val = simple_strtoul(last, &last, 0);
		break;
	case 'o':
		/* Discriminate between "on" and "off" */
		if (last[1] == 'n')
			val = 1;
		else if (last[1] == 'f')
			val = 0;
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (pin >= p7ctl_pdata->pins_nr)
		return -EINVAL;

	if (strcmp(function, "drive_strength") == 0) {
		set.fields.set_drv = 1;

		if (val >= P7CTL_DRV_5)
			val = P7CTL_DRV_5;
		else if (val >= P7CTL_DRV_4)
			val = P7CTL_DRV_4;
		else if (val >= P7CTL_DRV_3)
			val = P7CTL_DRV_3;
		else if (val >= P7CTL_DRV_2)
			val = P7CTL_DRV_2;
		else if (val >= P7CTL_DRV_1)
			val = P7CTL_DRV_1;
		else if (val >= P7CTL_DRV_0)
			val = P7CTL_DRV_0;

		set.fields.drv = val;
	} else if (strcmp(function, "schmitt") == 0) {
		set.fields.set_smt = 1;
		set.fields.smt = val;
	} else if (strcmp(function, "pull_up") == 0) {
		set.fields.set_pud = 1;
		set.fields.pud = val ? P7CTL_PUD_UP : 0;
	} else if (strcmp(function, "pull_down") == 0) {
		set.fields.set_pud = 1;
		set.fields.pud = val ? P7CTL_PUD_DOWN : 0;
	} else if (strcmp(function, "bus_keeper") == 0) {
		set.fields.set_pud = 1;
		set.fields.pud = val ? P7CTL_PUD_KEEP : 0;
	} else if (strcmp(function, "slew_rate") == 0) {
		set.fields.set_slr = 1;

		if (val >= P7CTL_SLR_3)
			val = P7CTL_SLR_3;
		else if (val >= P7CTL_SLR_2)
			val = P7CTL_SLR_2;
		else if (val >= P7CTL_SLR_1)
			val = P7CTL_SLR_1;
		else if (val >= P7CTL_SLR_0)
			val = P7CTL_SLR_0;

		set.fields.slr = val;
	} else
		return -EINVAL;

	return p7ctl_set_cfg(pctl, pin, set.word);
}

#endif  /* CONFIG_DEBUG_FS */

static struct pinconf_ops p7ctl_cfg_ops = {
	.pin_config_get             = p7ctl_get_cfg,
	.pin_config_set             = p7ctl_set_cfg,

#ifdef CONFIG_DEBUG_FS
	.pin_config_dbg_show        = p7ctl_show_pincfg,
	.pin_config_dbg_set         = p7ctl_set_pincfg,
	.pin_config_group_dbg_show  = p7ctl_show_pincfg
#endif  /* CONFIG_DEBUG_FS */
};

/************************
 * Multiplexing function
 ************************/

/* Does multiplexing function exists ? */
static int p7ctl_list_func(struct pinctrl_dev* pctl,
                           unsigned int func_id)
{
	if (func_id >= p7ctl_pdata->funcs_nr)
		return -EINVAL;

	return 0;
}

/* Go and get function name provided by platform. */
static char const* p7ctl_func_name(struct pinctrl_dev* pctl,
                                   unsigned int func_id)
{
	if (func_id >= p7ctl_pdata->funcs_nr)
		return NULL;

	if (p7ctl_func_available(func_id))
		return p7ctl_pdata->funcs[func_id].name;

	return "NOT_AVAILABLE";
}

/* Go and get list of groups (i.e., physical pins) attached to function. */
static int p7ctl_func_grp(struct pinctrl_dev* pctl,
                          unsigned func_id,
                          char const* const** groups,
                          unsigned int* group_nr)
{
	if (func_id >= p7ctl_pdata->funcs_nr)
		return -EINVAL;

	if (!p7ctl_func_available(func_id))
		return -EINVAL;

	*groups = &p7ctl_pdata->pins[p7ctl_pdata->funcs[func_id].pin_id].name;
	*group_nr = 1;
	return 0;
}

/* Enable physical pin and activate multiplexing function specified for it. */
static void p7ctl_enable_pin(unsigned int pin_id,
                             unsigned int mux_id)
{
	u32 volatile __iomem* const addr = p7ctl_vaddr + pin_id;
	union p7ctl_setting         cmd = { .word = __raw_readl(addr) };

	if (cmd.fields.ie) {
		if (cmd.fields.func == mux_id)
			return;

		/* Disable pin at first. */
		cmd.fields.ie = 0;
		__raw_writel(cmd.word, addr);
	}

	/* Apply settings. */
	cmd.fields.func = mux_id;
	writel(cmd.word, addr);

	/* Re-enable pin. */
	cmd.fields.ie = 1;
	writel(cmd.word, addr);
}

/* Enable physical pin and activate multiplexing function specified for it. */
static int p7ctl_enable_func(struct pinctrl_dev* pctl,
                             unsigned int func_id,
                             unsigned int grp_id)
{
	struct p7ctl_function const* const  func = &p7ctl_pdata->funcs[func_id];

	if (!p7ctl_func_available(func_id))
		return -EINVAL;

#ifdef DEBUG
	BUG_ON(grp_id != func->pin_id);
#endif

	p7ctl_enable_pin(grp_id, func->mux_id);

	dev_dbg(&p7ctl_pdev->dev,
	        "enabled %s pin as %s\n",
	        p7ctl_pdata->pins[grp_id].name,
	        func->name);
	return 0;
}

/* Disable physical pin. */
static void p7ctl_disable_pin(unsigned int pin_id)
{
	u32 volatile __iomem* const addr = p7ctl_vaddr + pin_id;
	union p7ctl_setting         cmd;

	cmd.word = __raw_readl(addr);
	cmd.fields.ie = 0;
	/* Setting ie to 0 just forces the input value reported by the pad to
	 * low. It still allows for the IP selected by the function to drive the
	 * PAD in output. There's no way to really "disable" the PAD. As a
	 * workaround, I switch back the PAD to function 1. It's either a GPIO
	 * (and unless it's buggy the GPIO controller should have unused PADs
	 * configured as inputs) or nothing. */
	cmd.fields.func = 1;

	__raw_writel(cmd.word, addr);

	dev_dbg(&p7ctl_pdev->dev,
	        "disabled %s pin\n",
	        p7ctl_pdata->pins[pin_id].name);
}

/* Disable multiplexing function and turn off associated physical pin. */
static void p7ctl_disable_func(struct pinctrl_dev* pctl,
                               unsigned int func_id,
                               unsigned int grp_id)
{
#ifdef DEBUG
	struct p7ctl_function const* const  func = &p7ctl_pdata->funcs[func_id];

	if (!p7ctl_func_available(func_id))
		return;

	BUG_ON(grp_id != func->pin_id);
#endif

	p7ctl_disable_pin(grp_id);
}

static int p7ctl_request(struct pinctrl_dev* pctl, unsigned int pin)
{
	struct pin_desc *desc;
	desc = pin_desc_get(pctl, pin);
	if (desc->gpio_owner)
		return -EBUSY;

	return 0;
}

#define CHAR_BIT    8

static int p7ctl_enable_gpio(struct pinctrl_dev* pctl,
                             struct pinctrl_gpio_range* range,
                             unsigned int gpio)
{
	struct pin_desc *desc;
	unsigned int const word = gpio / (sizeof(p7ctl_pdata->gpios[0]) *
	                                  CHAR_BIT);

#ifdef DEBUG
	P7CTL_ASSERT_PIN(gpio);
	BUG_ON(word >= p7ctl_pdata->gpios_nr);
#endif

	desc = pin_desc_get(pctl, gpio);
	if (desc->mux_owner)
		return -EBUSY;

	if (! (p7ctl_pdata->gpios[word] &
	       (1UL << (gpio % (sizeof(p7ctl_pdata->gpios[0]) * CHAR_BIT)))))
		return -EINVAL;

	p7ctl_enable_pin(gpio, 1);

	dev_dbg(&p7ctl_pdev->dev,
	        "enabled %s pin as gpio %d\n",
	        p7ctl_pdata->pins[gpio].name,
	        gpio);
	return 0;
}

static void p7ctl_disable_gpio(struct pinctrl_dev* pctl,
                               struct pinctrl_gpio_range* range,
                               unsigned int gpio)
{
	unsigned int const word = gpio / (sizeof(p7ctl_pdata->gpios[0]) *
	                                  CHAR_BIT);

	P7CTL_ASSERT_PIN(gpio);
	BUG_ON(word >= p7ctl_pdata->gpios_nr);

	if (! (p7ctl_pdata->gpios[word] &
	       (1UL << (gpio % (sizeof(p7ctl_pdata->gpios[0]) * CHAR_BIT))))) {
		dev_warn(&p7ctl_pdev->dev,
		         "trying to disable non gpio %d pin %s\n",
		         gpio,
		         p7ctl_pdata->pins[gpio].name);
		return;
	}

	p7ctl_disable_pin(gpio);
}

static struct pinmux_ops p7ctl_mux_ops = {
	.request                = p7ctl_request,
	.list_functions         = p7ctl_list_func,
	.get_function_name      = p7ctl_func_name,
	.get_function_groups    = p7ctl_func_grp,
	.enable                 = p7ctl_enable_func,
	.disable                = p7ctl_disable_func,
	.gpio_request_enable    = p7ctl_enable_gpio,
	.gpio_disable_free      = p7ctl_disable_gpio
};

/*****************
 * Pin controller
 *****************/

static struct pinctrl_gpio_range p7ctl_gpio_range = {
	.id         = 0,
	.base       = 0,
	.pin_base   = 0
};

static struct pinctrl_desc p7ctl_desc = {
	.name       = P7CTL_DRV_NAME,
	.pctlops    = &p7ctl_grp_ops,
	.pmxops     = &p7ctl_mux_ops,
	.confops    = &p7ctl_cfg_ops,
	.owner      = THIS_MODULE
};

static int __devinit p7ctl_probe(struct platform_device* pdev)
{
	int                                 err;
	char const*                         msg;
	struct p7ctl_plat_data const* const pdata = dev_get_platdata(&pdev->dev);
	struct resource const* const        res = platform_get_resource(pdev,
	                                                                IORESOURCE_MEM,
	                                                                0);
#ifdef DEBUG
	BUG_ON(! pdata);
	BUG_ON(! pdata->funcs);
	BUG_ON(! pdata->funcs_nr);
	BUG_ON(! pdata->pins);
	BUG_ON(! pdata->pins_nr);
	BUG_ON(! pdata->gpios);
	BUG_ON(! pdata->gpios_nr);
	BUG_ON(pdata->gpios_nr !=
	       ((pdata->pins_nr + (sizeof(p7ctl_pdata->gpios[0]) * CHAR_BIT) - 1) /
	        (sizeof(p7ctl_pdata->gpios[0]) * CHAR_BIT)));
	BUG_ON(! pdata->gpio_drv);
	BUG_ON(! res);
	dev_dbg(&pdev->dev, "probing...\n");
#endif

#ifdef CONFIG_PM_SLEEP
	p7ctl_saved_regs = kmalloc(pdata->pins_nr * sizeof(u32), GFP_KERNEL);
	if (! p7ctl_saved_regs) {
		dev_err(&pdev->dev, "Failed to allocate memory to save registers\n");
		return -ENOMEM;
	}
#endif
	if (!res)
		return -ENODEV;

	p7ctl_paddr = res->start;
	p7ctl_sz = resource_size(res);
	if (! request_mem_region(p7ctl_paddr,
	                         p7ctl_sz,
	                         dev_name(&pdev->dev))) {
		msg = "failed to request memory";
		err = -EBUSY;
		goto err;
	}

	p7ctl_vaddr = ioremap(p7ctl_paddr, p7ctl_sz);
	if (! p7ctl_vaddr) {
		msg = "failed to remap memory";
		err = -ENOMEM;
		goto release;
	}

	p7ctl_pdev = pdev;
	p7ctl_pdata = pdata;
	p7ctl_desc.pins = pdata->pins,
	p7ctl_desc.npins = pdata->pins_nr;
	p7ctl_dev = pinctrl_register(&p7ctl_desc, &pdev->dev, NULL);
	if (p7ctl_dev) {
		p7ctl_gpio_range.npins = pdata->pins_nr;
		p7ctl_gpio_range.name = pdata->gpio_drv;
		pinctrl_add_gpio_range(p7ctl_dev, &p7ctl_gpio_range);
		dev_info(&pdev->dev, "loaded\n");
		return 0;
	}

	iounmap(p7ctl_vaddr);

	err = -EINVAL;
	msg = "failed to register pins controller";

release:
	release_mem_region(p7ctl_paddr, p7ctl_sz);
err:
	dev_err(&pdev->dev, "%s (%d)\n", msg, err);
	return err;
}

static int __devexit p7ctl_remove(struct platform_device* pdev)
{
	pinctrl_unregister(p7ctl_dev);
	iounmap(p7ctl_vaddr);
	release_mem_region(p7ctl_paddr, p7ctl_sz);
#ifdef CONFIG_PM_SLEEP
	kfree(p7ctl_saved_regs);
#endif

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int p7ctl_suspend_noirq(struct device *dev)
{
	memcpy_fromio(p7ctl_saved_regs, p7ctl_vaddr,
	              p7ctl_pdata->pins_nr * sizeof(u32));
	return 0;
}

static int p7ctl_resume_noirq(struct device *dev)
{
	memcpy_toio(p7ctl_vaddr, p7ctl_saved_regs,
	            p7ctl_pdata->pins_nr * sizeof(u32));
	return 0;
}

#else
#define p7ctl_suspend_noirq     NULL
#define p7ctl_resume_noirq      NULL
#endif

static struct dev_pm_ops p7ctl_dev_pm_ops = {
	.suspend_noirq  = p7ctl_suspend_noirq,
	.resume_noirq   = p7ctl_resume_noirq,
};

static struct platform_driver p7ctl_driver = {
	.driver = {
		.name   = P7CTL_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = &p7ctl_dev_pm_ops,
	},
	.probe  = &p7ctl_probe,
	.remove = __devexit_p(&p7ctl_remove),
};

static int __init p7ctl_init(void)
{
	return platform_driver_register(&p7ctl_driver);
}
postcore_initcall(p7ctl_init);

static void __exit p7ctl_exit(void)
{
	platform_driver_unregister(&p7ctl_driver);
}
module_exit(p7ctl_exit);

MODULE_DESCRIPTION("Parrot7 I/O pins control driver");
MODULE_AUTHOR("Grégor Boirie <gregor.boirie@parrot.com>");
MODULE_LICENSE("GPL");
