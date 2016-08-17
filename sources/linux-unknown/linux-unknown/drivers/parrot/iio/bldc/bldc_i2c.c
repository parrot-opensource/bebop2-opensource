/**
 ************************************************
 * @file bldc_i2c.c
 * @brief BLDC IIO driver
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2015-06-22
 *************************************************
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/iio/iio.h>

#include <linux/iio/bldc/parrot_bldc_iio.h>

static int bldc_i2c_read_multiple_byte(struct device *dev,
			u8 reg_addr, int len, u8 *data)
{
	return i2c_smbus_read_i2c_block_data(to_i2c_client(dev),
			reg_addr,
			len, data);
}

static int bldc_i2c_write_multiple_byte(struct device *dev,
		u8 reg_addr, int len, u8 *data)
{
	return i2c_smbus_write_i2c_block_data(to_i2c_client(dev),
			reg_addr, len, data);
}

static const struct bldc_transfer_function bldc_tf_i2c = {
	.write_multiple_byte = bldc_i2c_write_multiple_byte,
	.read_multiple_byte = bldc_i2c_read_multiple_byte,
};

void bldc_i2c_configure(struct iio_dev *indio_dev,
		struct i2c_client *client, struct bldc_state *st)
{
	i2c_set_clientdata(client, indio_dev);

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = client->name;

	st->dev = &client->dev;
	st->tf = &bldc_tf_i2c;
}
EXPORT_SYMBOL(bldc_i2c_configure);

MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_DESCRIPTION("Parrot BLDC IIO driver");
MODULE_LICENSE("GPL");
