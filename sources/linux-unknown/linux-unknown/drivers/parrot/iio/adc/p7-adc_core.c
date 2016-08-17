/**
 ************************************************
 * @file p7-adc_core.c
 * @brief P7 Analogic to Digital Converter driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2013-09-19
 ************************************************
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include "p7-adc_core.h"

static LIST_HEAD(iiodev_list);
static DEFINE_MUTEX(list_lock);		/* Protects the list of hosts */

static struct p7_adc_iio_chip_info *p7_adc_iio_get_chip_info(unsigned int type)
{
	struct p7_adc_iio_chip_info *chip = NULL, *cix;

	mutex_lock(&list_lock);
	list_for_each_entry(cix, &iiodev_list, list) {
		if (cix->type == type) {
			chip = cix;
			break;
		}
	}
	mutex_unlock(&list_lock);

	return chip;
}

int p7_adc_register_iio_chip(struct p7_adc_iio_chip_info *ch_info)
{
	struct p7_adc_iio_chip_info *cix;

	if (!ch_info->ops->enable ||
			!ch_info->ops->disable ||
			!ch_info->ops->config ||
			!ch_info->ops->read)
		return -EINVAL;

	mutex_lock(&list_lock);
	list_for_each_entry(cix, &iiodev_list, list) {
		if (cix->type == ch_info->type) {
			mutex_unlock(&list_lock);
			return -EBUSY;
		}
	}

	list_add_tail(&ch_info->list, &iiodev_list);
	mutex_unlock(&list_lock);

	return 0;
}
EXPORT_SYMBOL(p7_adc_register_iio_chip);

int p7_adc_unregister_iio_chip(struct p7_adc_iio_chip_info *ch_info)
{
	mutex_lock(&list_lock);
	list_del(&ch_info->list);
	mutex_unlock(&list_lock);

	return 0;
}
EXPORT_SYMBOL(p7_adc_unregister_iio_chip);

int p7_adc_iio_chip_get_channel_template(unsigned type, struct iio_chan_spec **ch_templ)
{
	struct p7_adc_iio_chip_info *ci;

	ci = p7_adc_iio_get_chip_info(type);
	if (!ci)
		return -EPROTONOSUPPORT;

	*ch_templ = &ci->channel_template;

	return 0;
}
EXPORT_SYMBOL(p7_adc_iio_chip_get_channel_template);

int p7_adc_iio_chip_get_channel_ops(unsigned type, struct
		p7_adc_iio_chip_channel_ops **ops)
{
	struct p7_adc_iio_chip_info *ci;

	ci = p7_adc_iio_get_chip_info(type);
	if (!ci)
		return -EPROTONOSUPPORT;

	*ops = ci->ops;

	return 0;
}
EXPORT_SYMBOL(p7_adc_iio_chip_get_channel_ops);

int p7_adc_get_iio_chip(unsigned int type)
{
	struct p7_adc_iio_chip_info *ci;

	ci = p7_adc_iio_get_chip_info(type);
	if (!ci)
		return -EPROTONOSUPPORT;

	if (!try_module_get(ci->owner))
		return -ENODEV;

	return 0;
}
EXPORT_SYMBOL(p7_adc_get_iio_chip);

int p7_adc_put_iio_chip(unsigned int type)
{
	struct p7_adc_iio_chip_info *ci;

	ci = p7_adc_iio_get_chip_info(type);
	if (!ci)
		return -EPROTONOSUPPORT;

	module_put(ci->owner);

	return 0;
}
EXPORT_SYMBOL(p7_adc_put_iio_chip);

