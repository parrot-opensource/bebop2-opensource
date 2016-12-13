/*
 * P7 AVI LCD vsync generator
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>

#include "avi_vsync_gen.h"

#define DEVICE_NAME "avi_vsync_gen"

struct avivsync_data {
	struct avivsync_timings  timings;
	int			 enabled;
	const struct avi_node	*lcd_node;
	struct clk		*pixclock;
	struct mutex		 lock;
	struct pinctrl          *pctl;
};

static int avivsync_configure(struct avivsync_data *data)
{
	struct avi_lcd_avi_vsync_gen_regs	vsync_gen_regs;
	u32                                    *regs;
	int					i;
	int                                     ret;
	struct avi_lcd_regs                     lcd_reg = {
		.itsource = { ._register = 0 },
		.interface = {{
				.free_run  = 1,
				.itu656	   = 0,
				.ivs	   = 0,
				.psync_en  = 0,
				.vsync_gen = 1,
				.prog	   = 1,
			}},
		.top_format_ctrl = { ._register = 0 },
		.top_h_timing0 = {{
				.top_hactive_on = 0,
				.top_hsync_off  = 0,
			}},
		.top_h_timing1 = {{
				.top_hactive_off = 0,
			}},
		.top_v_timing2 = {{
				.top_vactive_on  = 0,
				.top_vactive_off = 0,
			}},
	};

	/* XXX sanity check the values ? */

	/*
	 * Master sync configuration
	 */
	lcd_reg.top_h_timing1.top_htotal    = data->timings.htotal;

	lcd_reg.top_v_timing0.top_vsync_von = data->timings.vsync_von;
	lcd_reg.top_v_timing0.top_vsync_hon = data->timings.vsync_hon;

	lcd_reg.top_v_timing1.top_vsync_voff = data->timings.vsync_voff;
	lcd_reg.top_v_timing1.top_vsync_hoff = data->timings.vsync_hoff;

	lcd_reg.top_v_timing3.top_vtotal = data->timings.vtotal;

	avi_lcd_set_registers(data->lcd_node, &lcd_reg);

	regs = (u32 *)&vsync_gen_regs;

	/*
	 * Slave sync configuration
	 */
	for (i = 0; i < AVIVSYNC_OFFSET_NR; i++) {
		*regs++ = data->timings.offsets[i].vsync_on._register;
		*regs++ = data->timings.offsets[i].vsync_off._register;
	}

	avi_vsync_gen_set_registers(data->lcd_node, &vsync_gen_regs);

	/*
	 * Pixclock configuration
	 */
	if (data->timings.pixclock_freq) {
		data->timings.pixclock_freq =
			clk_round_rate(data->pixclock,
				       data->timings.pixclock_freq);

		ret = clk_set_rate(data->pixclock, data->timings.pixclock_freq);
		if (ret)
			return ret;
	}

	return 0;
}

static int avivsync_enable(struct avivsync_data *data)
{
	int ret;

	if (data->enabled)
		return 0;

	if (data->timings.pixclock_freq == 0)
		return -EINVAL;

	/* We have to activate the pixclock before we enable the node otherwise
	 * the logic won't be de-reset properly. */
	ret = clk_enable(data->pixclock);
	if (ret)
		return ret;

	avi_enable_node(data->lcd_node);
	avi_apply_node(data->lcd_node);

	data->enabled = 1;
	return 0;
}

static void avivsync_disable(struct avivsync_data *data)
{
	if (data->enabled) {
		clk_disable(data->pixclock);
		avi_disable_node(data->lcd_node);
	}
}

/*
 * SysFS interface
 */

#define AVIVSYNC_SYSFS_ACCESSORS(_t)					\
	static ssize_t avivsync_show_##_t(struct device *device,	\
					  struct device_attribute *a,	\
					  char *buf)			\
	{								\
		const struct avivsync_data *data;			\
		data = dev_get_drvdata(device);				\
									\
		return snprintf(buf, PAGE_SIZE,				\
				"%u\n", data->timings._t);		\
	}								\
									\
	static ssize_t avivsync_store_##_t(struct device *device,	\
					   struct device_attribute *a,	\
					   const char *buf,		\
					   size_t count)		\
	{								\
		struct avivsync_data	*data;				\
		unsigned long		 v;				\
		char			*last;				\
									\
		data = dev_get_drvdata(device);				\
									\
		v = simple_strtoul(buf, &last, 0);			\
									\
		if (*last != '\0' && *last != '\n')			\
			/* invalid input */				\
			return -EINVAL;					\
									\
		mutex_lock(&data->lock);				\
									\
		data->timings._t = v;					\
		avivsync_configure(data);				\
									\
		mutex_unlock(&data->lock);				\
									\
		return count;						\
	}

#define AVIVSYNC_SYSFS_ATTRS(_t)				\
	__ATTR(_t, S_IRUGO|S_IWUSR,				\
	       avivsync_show_##_t, avivsync_store_##_t)

AVIVSYNC_SYSFS_ACCESSORS(htotal)
AVIVSYNC_SYSFS_ACCESSORS(vtotal)
AVIVSYNC_SYSFS_ACCESSORS(vsync_von)
AVIVSYNC_SYSFS_ACCESSORS(vsync_voff)
AVIVSYNC_SYSFS_ACCESSORS(vsync_hon)
AVIVSYNC_SYSFS_ACCESSORS(vsync_hoff)
AVIVSYNC_SYSFS_ACCESSORS(pixclock_freq)

#define AVIVSYNC_OFFSET_SYSFS_ACCESSORS(_n)				\
	static ssize_t avivsync_show_offset##_n(struct device *device,	\
					    struct device_attribute *a, \
					    char *buf)			\
	{								\
		const struct avivsync_data	*data;			\
		const struct avivsync_offsets	*o;			\
									\
		data = dev_get_drvdata(device);				\
		o = &data->timings.offsets[_n];				\
									\
		return snprintf(buf, PAGE_SIZE, "%u:%u,%u:%u\n",	\
				o->vsync_on.vsync_gen_von,		\
				o->vsync_on.vsync_gen_hon,		\
				o->vsync_off.vsync_gen_voff,		\
				o->vsync_off.vsync_gen_hoff);		\
	}								\
									\
	static ssize_t avivsync_store_offset##_n(			\
		struct device *device,					\
		struct device_attribute *a,				\
		const char *buf,					\
		size_t count)						\
	{								\
		struct avivsync_data		*data;			\
		struct avivsync_offsets		 off;			\
		struct avivsync_offsets		*o;			\
		char				*last;			\
									\
									\
		data = dev_get_drvdata(device);				\
		o = &data->timings.offsets[_n];				\
									\
		off.vsync_on.vsync_gen_von =				\
			simple_strtoul(buf, &last, 0);			\
		if (*last != ':')					\
			goto error;					\
									\
		last++;							\
		off.vsync_on.vsync_gen_hon =				\
			simple_strtoul(last, &last, 0);			\
		if (*last != ',')	 				\
			goto error;					\
									\
		last++;							\
		off.vsync_off.vsync_gen_voff =				\
			simple_strtoul(last, &last, 0);			\
		if (*last != ':')					\
			goto error;					\
									\
		last++;							\
		off.vsync_off.vsync_gen_hoff =				\
			simple_strtoul(last, &last, 0);			\
		if (*last != '\0' && *last != '\n') 			\
			goto error;					\
									\
		*o = off;						\
		avivsync_configure(data);				\
									\
	error:								\
		return count;						\
	}

#define AVIVSYNC_OFFSET_SYSFS_ATTRS(_n)					\
	__ATTR(offset##_n, S_IRUGO|S_IWUSR,				\
	       avivsync_show_offset##_n, avivsync_store_offset##_n)

AVIVSYNC_OFFSET_SYSFS_ACCESSORS(0)
AVIVSYNC_OFFSET_SYSFS_ACCESSORS(1)
AVIVSYNC_OFFSET_SYSFS_ACCESSORS(2)
AVIVSYNC_OFFSET_SYSFS_ACCESSORS(3)
AVIVSYNC_OFFSET_SYSFS_ACCESSORS(4)

static ssize_t avivsync_show_enabled(struct device *device,
				     struct device_attribute *attr,
				     char *buf)
{
	const struct avivsync_data *data = dev_get_drvdata(device);

	return snprintf(buf, PAGE_SIZE, "%u\n", data->enabled);
}

static ssize_t avivsync_store_enabled(struct device *device,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct avivsync_data	*data = dev_get_drvdata(device);
	int			 v;

	if (buf[0] == '1')
		v = 1;
	else if (buf[0] == '0')
		v = 0;
	else
		/* invalid input */
		return -EINVAL;

	mutex_lock(&data->lock);

	if (v)
		avivsync_enable(data);
	else
		avivsync_disable(data);

	mutex_unlock(&data->lock);

	return count;
}

static struct device_attribute avivsync_sysfs_attrs[] = {
	AVIVSYNC_SYSFS_ATTRS(htotal),
	AVIVSYNC_SYSFS_ATTRS(vtotal),
	AVIVSYNC_SYSFS_ATTRS(vsync_von),
	AVIVSYNC_SYSFS_ATTRS(vsync_voff),
	AVIVSYNC_SYSFS_ATTRS(vsync_hon),
	AVIVSYNC_SYSFS_ATTRS(vsync_hoff),
	AVIVSYNC_SYSFS_ATTRS(pixclock_freq),

	AVIVSYNC_OFFSET_SYSFS_ATTRS(0),
	AVIVSYNC_OFFSET_SYSFS_ATTRS(1),
	AVIVSYNC_OFFSET_SYSFS_ATTRS(2),
	AVIVSYNC_OFFSET_SYSFS_ATTRS(3),
	AVIVSYNC_OFFSET_SYSFS_ATTRS(4),

	__ATTR(enabled, S_IRUGO|S_IWUSR,
	       avivsync_show_enabled, avivsync_store_enabled),
};

static int __devinit avivsync_probe(struct platform_device *pdev)
{
	struct avivsync_platform_data	*pdata;
	struct avivsync_data		*data;
	int				 i;
	int				 ret = 0;

	if (!pdev) {
		ret = -ENODEV;
		goto no_pdata;
	}

	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		ret = -ENODEV;
		goto no_pdata;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	platform_set_drvdata(pdev, data);

	mutex_init(&data->lock);

	data->pixclock = clk_get(&pdev->dev, "lcd");
	if (IS_ERR(data->pixclock)) {
		ret = PTR_ERR(data->pixclock);
		goto no_clock;
	}

	data->pctl = pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(data->pctl)) {
		ret = PTR_ERR(data->pctl);
		goto no_pins;
	}

#if 0
#warning "This code needs to be ported to the segment API"
	const struct avi_chan		*chan;
	if (!pdata->avi_group || pdata->avi_group->chan_nr == 0) {
		ret = -ENODEV;
		goto no_group;
	}

	chan = &pdata->avi_group->channels[0];

	if (chan->type != AVI_VSYNC_GEN_CHAN_TYPE ||
	    chan->nodes_nr != 1) {
		ret = -EINVAL;
		goto bad_chan;
	}

	data->lcd_node = avi_get_channode(chan, 0);
	if (data->lcd_node->type != AVI_LCD_NODE_TYPE) {
		ret = -EINVAL;
		goto bad_chan;
	}
#endif
	/* Copy the defaults */
	data->timings = pdata->timings;

	ret = avivsync_configure(data);
	if (ret)
		goto configure_failed;

	if (pdata->enabled)
		avivsync_enable(data);

	dev_set_drvdata(&pdev->dev, data);

	for (i = 0; i < ARRAY_SIZE(avivsync_sysfs_attrs); i++) {
		ret = device_create_file(&pdev->dev, &avivsync_sysfs_attrs[i]);
		if (ret) {
                        for (--i; i >= 0; i--)
                                device_remove_file(&pdev->dev,
						   &avivsync_sysfs_attrs[i]);
			goto sysfs_register_failed;
                }
	}

	dev_info(&pdev->dev, "VSync generator driver initialized\n");

	return 0;

sysfs_register_failed:
	dev_set_drvdata(&pdev->dev, NULL);
configure_failed:
#if 0
bad_chan:
no_group:
#endif
	pinctrl_put(data->pctl);
no_pins:
	clk_put(data->pixclock);
no_clock:
	mutex_destroy(&data->lock);
	kfree(data);
	platform_set_drvdata(pdev, NULL);
alloc_failed:
no_pdata:
	return ret;
}

static int __devexit avivsync_remove(struct platform_device *pdev)
{
	struct avivsync_data	*data = platform_get_drvdata(pdev);
	int			 i;

	if (data) {
		for (i = 0; i < ARRAY_SIZE(avivsync_sysfs_attrs); i++)
			device_remove_file(&pdev->dev, &avivsync_sysfs_attrs[i]);
		dev_set_drvdata(&pdev->dev, NULL);

		avivsync_disable(data);
		pinctrl_put(data->pctl);
		clk_put(data->pixclock);
		mutex_destroy(&data->lock);
		kfree(data);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

static struct platform_driver avivsync_driver = {
        .driver         = {
                .name   = DEVICE_NAME,
                .owner  = THIS_MODULE,
        },
        .probe          = avivsync_probe,
        .remove         = __devexit_p(avivsync_remove),
};

static int __init avivsync_init(void)
{
        return platform_driver_register(&avivsync_driver);
}

module_init(avivsync_init);

static void __exit avivsync_exit(void)
{
        platform_driver_unregister(&avivsync_driver);
}

module_exit(avivsync_exit);

MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_DESCRIPTION("Driver for the Advanced Video Interface VSync generator");
MODULE_LICENSE("GPL");
