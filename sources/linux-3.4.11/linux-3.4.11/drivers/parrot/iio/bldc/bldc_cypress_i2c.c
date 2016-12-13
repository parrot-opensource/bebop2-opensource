/**
 ************************************************
 * @file bldc_cypress_core.c
 * @brief BLDC cypress IIO driver
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2015-06-22
 *************************************************
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/iio/iio.h>

#include <linux/iio/bldc/parrot_bldc_cypress_iio.h>
#include <linux/iio/bldc/parrot_bldc_iio.h>

#define BLDC_CYPRESS_DRV_NAME "bldc-cypress-i2c"

static const struct bldc_cypress_platform_data default_pdata = {
	.lut = {0, 1, 2, 3},
	.spin_dir = 0x5,	/* 0b0101, which is CW/CCW/CW/CCW */
};

static int bldc_cypress_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct bldc_state *st;
	const struct bldc_cypress_platform_data *pdata;

	struct iio_dev *indio_dev;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENOSYS;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	pdata = dev_get_platdata(&client->dev);
	if (!pdata)
		pdata = &default_pdata;

	memcpy(&st->pdata, pdata, sizeof(st->pdata));
	dev_info(&client->dev, "lut={%d, %d, %d, %d} spin_dir=0x%X\n",
		st->pdata.lut[0], st->pdata.lut[1], st->pdata.lut[2],
		st->pdata.lut[3], st->pdata.spin_dir);

	bldc_i2c_configure(indio_dev, client, st);

	return bldc_cypress_probe(indio_dev);
}

static int bldc_cypress_i2c_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	bldc_cypress_remove(indio_dev);
	devm_iio_device_free(&client->dev, indio_dev);
	return 0;
}

static void bldc_cypress_i2c_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	bldc_cypress_shutdown(indio_dev);
}

/*
 * device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id bldc_cypress_id[] = {
	{BLDC_CYPRESS_DRV_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, bldc_cypress_id);

static struct i2c_driver bldc_cypress_driver = {
	.probe		=	bldc_cypress_i2c_probe,
	.remove		=	bldc_cypress_i2c_remove,
	.shutdown	=	bldc_cypress_i2c_shutdown,
	.id_table	=	bldc_cypress_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	BLDC_CYPRESS_DRV_NAME,
	},
};

module_i2c_driver(bldc_cypress_driver);

MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_DESCRIPTION("Parrot BLDC cypress IIO i2c driver");
MODULE_LICENSE("GPL");
