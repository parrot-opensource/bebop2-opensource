/**
 * linux/drivers/parrot/gpio/mux/p7-i2cmux.c - Parrot7 I2CM bus muxing
 *                                             implementation
 *
 * Copyright (C) 2012 Parrot S.A.
 *
 * author:  Lionel Flandrin <lionel.flandrin@parrot.com>
 * date:    19-Sep-2012
 *
 * This file is released under the GPL
 */

#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/module.h>
#include "p7-i2cmux.h"

struct p7i2cmux_channel {
	/* I2C adapter for this channel */
	struct i2c_adapter      *adap;
	/* PIN state for this adapter */
	struct pinctrl_state    *pstate;
};

struct p7i2cmux {
	struct i2c_adapter       *parent;
	struct pinctrl           *pctl;
	struct pinctrl_state     *default_pstate;
	struct p7i2cmux_channel  *channels;
	unsigned                  nchannels;
};

static int p7i2cmux_select(struct i2c_adapter *adap, void *data, u32 chan)
{
	struct p7i2cmux *mux = data;

	BUG_ON(chan >= mux->nchannels);

	/* Select the channel mux configuration */
	return pinctrl_select_state(mux->pctl, mux->channels[chan].pstate);
}

static int p7i2cmux_deselect(struct i2c_adapter *adap, void *data, u32 chan)
{
	struct p7i2cmux *mux = data;

	BUG_ON(chan >= mux->nchannels);

	/* Switch back to the default channel (i.e. the parent's channel) */
	return pinctrl_select_state(mux->pctl, mux->default_pstate);
}

static int __devinit p7i2cmux_probe(struct platform_device *pdev)
{
	struct p7i2cmux                 *mux;
	const struct p7i2cmux_plat_data *pdata = pdev->dev.platform_data;
	int                              i;
	int                              ret   = 0;

	printk("%s\n", __func__);

	if (!pdata)
		return -ENODEV;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	mux->nchannels = pdata->nr_channels;

	if (mux->nchannels == 0) {
		ret = -ENODEV;
		goto free_mux;
	}

	mux->channels = kzalloc(mux->nchannels * sizeof(*(mux->channels)),
	                        GFP_KERNEL);

	if (!mux->channels) {
		ret = -ENOMEM;
		goto free_mux;
	}

	mux->parent = i2c_get_adapter(pdata->parent);
	if (!mux->parent) {
		dev_err(&pdev->dev, "Couldn't find parent I2C adapter %d\n",
		        pdata->parent);
		ret = -ENODEV;
		goto free;
	}

	mux->pctl = pinctrl_get(&pdev->dev);

	if (IS_ERR(mux->pctl)) {
		dev_err(&pdev->dev, "Couldn't get pin controller\n");
		ret = PTR_ERR(mux->pctl);
		goto put_parent;
	}

	mux->default_pstate =
		pinctrl_lookup_state(mux->pctl, PINCTRL_STATE_DEFAULT);

	if (IS_ERR(mux->default_pstate)) {
		dev_err(&pdev->dev, "Failed to find default pin state\n");
		ret = PTR_ERR(mux->default_pstate);
		/* Force the removal of this adapter */
		goto put_pctl;
	}

	/* Add muxed adapters */
	for (i = 0; i < mux->nchannels; i++) {
		mux->channels[i].adap =
			i2c_add_mux_adapter(mux->parent, &pdev->dev, mux,
			                    pdata->parent * 10 + i,
			                    i, 0,
			                    p7i2cmux_select, p7i2cmux_deselect);

		if (mux->channels[i].adap == NULL) {
			dev_err(&pdev->dev,
			        "Failed to add mux %s for I2C adapter %s\n",
			        pdata->channel_names[i], mux->parent->name);
			ret = -EINVAL;
			goto remove_mux;
		}

		mux->channels[i].pstate =
			pinctrl_lookup_state(mux->pctl,
			                     pdata->channel_names[i]);

		if (IS_ERR(mux->channels[i].pstate)) {
			dev_err(&pdev->dev, "Failed to find pin state for %s\n",
			        pdata->channel_names[i]);
			ret = PTR_ERR(mux->channels[i].pstate);
			/* Force the removal of this adapter */
			i++;
			goto remove_mux;
		}
	}

	/* switch to default bus */
	ret = pinctrl_select_state(mux->pctl, mux->default_pstate);
	if (ret) {
		dev_err(&pdev->dev, "Failed to switch to default mux config\n");
		goto remove_mux;
	}

	dev_info(&pdev->dev, "Initialized %d mux channels for adapter %s\n",
	         mux->nchannels, mux->parent->name);

	platform_set_drvdata(pdev, mux);

	return 0;

remove_mux:
	while (--i)
		i2c_del_mux_adapter(mux->channels[i].adap);
put_pctl:
	pinctrl_put(mux->pctl);
put_parent:
	i2c_put_adapter(mux->parent);
free:
	kfree(mux->channels);
free_mux:
	kfree(mux);

	return ret;
}

static int __devexit p7i2cmux_remove(struct platform_device *pdev)
{
	struct p7i2cmux *mux = platform_get_drvdata(pdev);
	int              i;

	if (!mux)
		return -ENODEV;

	for (i = 0; i < mux->nchannels; i++)
		i2c_del_mux_adapter(mux->channels[i].adap);
	pinctrl_put(mux->pctl);
	i2c_put_adapter(mux->parent);
	kfree(mux->channels);
	kfree(mux);

	return 0;
}

static struct platform_driver p7i2cmux_driver = {
	.probe  = p7i2cmux_probe,
	.remove = __devexit_p(p7i2cmux_remove),
	.driver = {
		.owner  = THIS_MODULE,
		.name   = P7I2CMUX_DRV_NAME,
	},
};

#ifdef MODULE
module_platform_driver(p7i2cmux_driver);
#else
static int __init p7_i2cmux_init(void)
{
	return platform_driver_register(&p7i2cmux_driver);
}

postcore_initcall(p7_i2cmux_init);
#endif /* MODULE */

MODULE_DESCRIPTION("Pin muxing support for multiple I2C buses "
                   "on the same master");
MODULE_AUTHOR("Lionel Flandrin <lionel.flandrin@parrot.com>");
MODULE_LICENSE("GPL");
