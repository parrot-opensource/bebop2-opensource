/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include "inv_mpu_iio.h"

static void inv_scan_query(struct iio_dev *indio_dev)
{
	struct inv_mpu6050_state  *st = iio_priv(indio_dev);

	if (!indio_dev->active_scan_mask) {
		dev_err(&st->client->dev,
			"mpu6050 should be started "
			"before using trig-mpu6050\n");
		st->chip_config.gyro_fifo_enable = false;
		st->chip_config.accl_fifo_enable = false;
		st->chip_config.temp_fifo_enable = false;
		return;
	}

	st->chip_config.gyro_fifo_enable =
		test_bit(INV_MPU6050_SCAN_GYRO_X,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_MPU6050_SCAN_GYRO_Y,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_MPU6050_SCAN_GYRO_Z,
			 indio_dev->active_scan_mask);

	st->chip_config.temp_fifo_enable =
		test_bit(INV_MPU6050_SCAN_TEMP,
			 indio_dev->active_scan_mask);

	st->chip_config.accl_fifo_enable =
		test_bit(INV_MPU6050_SCAN_ACCL_X,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_MPU6050_SCAN_ACCL_Y,
			 indio_dev->active_scan_mask) ||
		test_bit(INV_MPU6050_SCAN_ACCL_Z,
			indio_dev->active_scan_mask);
}

/**
 *  inv_mpu6050_get_enable() - check if chip is enabled.
 *  @indio_dev:	Device driver instance.
 *  @returns: enable/disable
 */
static unsigned int inv_mpu6050_get_enable(struct iio_dev *indio_dev)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	return st->chip_config.enable;
}

/**
 *  inv_mpu6050_set_enable() - enable chip functions.
 *  @indio_dev:	Device driver instance.
 *  @enable: enable/disable
 */
static int inv_mpu6050_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	int result;

	if (enable) {
		result = inv_mpu6050_set_power_itg(st, true);
		if (result)
			return result;
		inv_scan_query(indio_dev);
		if (st->chip_config.gyro_fifo_enable) {
			result = inv_mpu6050_switch_engine(st, true,
					INV_MPU6050_BIT_PWR_GYRO_STBY);
			if (result)
				return result;
		}
		if (st->chip_config.accl_fifo_enable) {
			result = inv_mpu6050_switch_engine(st, true,
					INV_MPU6050_BIT_PWR_ACCL_STBY);
			if (result)
				return result;
		}
		result = inv_mpu6050_write_reg(st,
					       st->reg->int_pin_cfg,
					       st->reg->int_pin_cfg_set);
		if (result)
		    return result;

		result = inv_reset_fifo(indio_dev);
		if (result)
			return result;
	} else {
		result = inv_mpu6050_write_reg(st, st->reg->fifo_en, 0);
		if (result)
			return result;

		result = inv_mpu6050_write_reg(st, st->reg->int_enable, 0);
		if (result)
			return result;

		result = inv_mpu6050_write_reg(st, st->reg->user_ctrl, 0);
		if (result)
			return result;

		result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_GYRO_STBY);
		if (result)
			return result;

		result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_ACCL_STBY);
		if (result)
			return result;
		result = inv_mpu6050_set_power_itg(st, false);
		if (result)
			return result;
	}
	st->chip_config.enable = enable;

	return 0;
}

/**
 * inv_mpu_data_rdy_trigger_set_state() - set data ready interrupt state
 * @trig: Trigger instance
 * @state: Desired trigger state
 */
static int inv_mpu_data_rdy_trigger_set_state(struct iio_trigger *trig,
						bool state)
{
	return 0;
}

static const struct iio_trigger_ops inv_mpu_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &inv_mpu_data_rdy_trigger_set_state,
};

/* Device trigger attributes */
static ssize_t inv_mpu6050_trigger_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	unsigned int enabled;

	enabled = inv_mpu6050_get_enable(indio_dev);
	return sprintf(buf, "%u\n", enabled);
}

static ssize_t inv_mpu6050_trigger_enable_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf,
						size_t len)
{
	struct iio_trigger *trig = to_iio_trigger(dev);
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	bool requested_state;
	int ret;

	ret = strtobool(buf, &requested_state);
	if (ret < 0)
		return ret;

	ret = inv_mpu6050_set_enable(indio_dev, requested_state);
	return (ret < 0) ? ret : len;
}

static IIO_DEVICE_ATTR(enable,
		       S_IRUGO | S_IWUSR,
		       inv_mpu6050_trigger_enable_show,
		       inv_mpu6050_trigger_enable_store,
		       TRIGGER_ATTR_ENABLE);

static struct attribute *inv_trigger_attributes[] = {
	&iio_dev_attr_enable.dev_attr.attr,
	NULL
};

static const struct attribute_group inv_trigger_attribute_group = {
	.attrs = inv_trigger_attributes
};

static const struct attribute_group *inv_trigger_attribute_groups[] = {
	&inv_trigger_attribute_group,
	NULL
};

int inv_mpu6050_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	st->trig = devm_iio_trigger_alloc(&indio_dev->dev,
					  "trig-%s",
					  st->name);
	if (!st->trig) {
		dev_err(&st->client->dev,
			"devm_iio_trigger_alloc(%s) failed ENOMEM\n", st->name);
		return -ENOMEM;
	}

	st->trig->dev.groups = inv_trigger_attribute_groups;

	ret = devm_request_irq(&st->client->dev,
			       st->client->irq,
			       &iio_trigger_generic_data_rdy_poll,
			       IRQF_TRIGGER_RISING,
			       st->name,
			       st->trig);
	if (ret)
		return ret;

	dev_info(&st->client->dev, "requested IRQ %d\n", st->client->irq);

	st->trig->dev.parent = &st->client->dev;
	st->trig->ops = &inv_mpu_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = iio_trigger_register(st->trig);
	if (ret)
		return ret;
	indio_dev->trig = iio_trigger_get(st->trig);

	return 0;
}

void inv_mpu6050_remove_trigger(struct inv_mpu6050_state *st)
{
	iio_trigger_unregister(st->trig);
}
