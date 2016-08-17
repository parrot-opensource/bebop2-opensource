/**
 * Kernel IIO driver for the TSL2591 illuminance I2C chip.
 * Datasheet : http://www.adafruit.com/datasheets/TSL25911_Datasheet_EN_v1.pdf
 *
 * Author : Maxime Jourdan <maxime.jourdan@parrot.com>
 * Date   : 14/01/2015
 * Author : Luis Mario Domenzain <ld@airinov.fr>
 * Date   : 12/10/2015
 */
#include "tsl2591.h"

struct tsl2591 {
	struct i2c_client *i2c;

	// ADC channels
	u16 data0;
	u16 data1;

	// Integration time (100 -> 600ms)
	u16 int_time;

	// Gain. 0 = Low, [..] 3 = Max
	u8 gain;
};

static int tsl2591_read8(struct tsl2591 *tsl2591, u8 reg, u8 *val)
{
	int ret;
	struct i2c_client *client = tsl2591->i2c;
	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = 1,
			.buf   = (u8 *)&reg,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = val,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

static int tsl2591_read_consecutive(
		struct tsl2591 *tsl2591,
		u8 reg,
		u8 *val,
		u16 bytes)
{
	int ret;
	struct i2c_client *client = tsl2591->i2c;
	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = 1,
			.buf   = (u8 *)&reg,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = bytes,
			.buf   = val,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading %d bytes at 0x%02x!\n",
				bytes, reg);
		return ret;
	}

	return 0;
}

static int tsl2591_write8(struct tsl2591 *tsl2591, u8 reg, u8 val)
{
	int            ret;
	struct i2c_client *client = tsl2591->i2c;
	struct i2c_msg msg;
	struct {
		u8 reg;
		u8 val;
	} __packed buf;

	buf.reg = reg;
	buf.val = val;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 2;
	msg.buf   = (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

static int tsl2591_is_powered(struct tsl2591 *tsl2591)
{
	u8 val;

	if (tsl2591_read8(tsl2591, TSL2591_ENABLE_RW, &val))
		return 0;

	val &= TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN;
	return val == (TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN);
}

static int tsl2591_set_integration(struct tsl2591 *tsl2591)
{
	int ret;
	u8 val  = TSL2591_CONFIG_SET_AGAIN(tsl2591->gain);
	val    |= TSL2591_CONFIG_SET_ATIME(tsl2591->int_time);

	/* Turn off integration, set the integration time and gain, then turn it
	 * on. This seems to reset the ADC for the next integration. Otherwise
	 * it carries over state from before and gives false results.
	 */
	ret = tsl2591_write8(tsl2591, TSL2591_ENABLE_RW, TSL2591_ENABLE_PON);
	if (!ret) {
		ret = tsl2591_write8(tsl2591, TSL2591_CONFIG_RW, val);
		if (!ret) {
			ret = tsl2591_write8(tsl2591, TSL2591_ENABLE_RW,
				TSL2591_ENABLE_AEN | TSL2591_ENABLE_PON);
		}
	}

	return ret;
}

static int tsl2591_power(struct tsl2591 *tsl2591, int on)
{
	int ret;

	if (on) {
		ret = tsl2591_write8(tsl2591, TSL2591_ENABLE_RW,
				TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN);
		if (!ret)
			ret = tsl2591_set_integration(tsl2591);

		return ret;
	}

	return tsl2591_write8(tsl2591, TSL2591_ENABLE_RW, TSL2591_ENABLE_OFF);
}

static int tsl2591_get_adc(struct tsl2591 *tsl2591)
{
	u8 all[5];
	u8 valid = 0;
	u8 sleep = 0;
	int ret;

	if (!tsl2591_is_powered(tsl2591)) {
		ret = tsl2591_power(tsl2591, 1);

		if (ret)
			return ret;
	}

	while (!valid) {
		if (sleep)
			msleep(30);

		//Read status and ADCs in a single transaction: 5 bytes.
		ret = tsl2591_read_consecutive(tsl2591,
					TSL2591_ADCDATA_R, all, 5);
		if (ret)
			return ret;

		valid = all[0] & TSL2591_STATUS_AVALID;
		sleep |= 1;
	}

	tsl2591->data0 = all[2] << 8 | all[1];
	tsl2591->data1 = all[4] << 8 | all[3];

	return 0;
}

static const struct iio_chan_spec tsl2591_channels[] = {
	{
		.type      = IIO_INTENSITY,
		.modified  = 1,
		.channel   = 0,
		.channel2  = IIO_MOD_LIGHT_BOTH,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}, {
		.type      = IIO_INTENSITY,
		.modified  = 1,
		.channel   = 1,
		.channel2  = IIO_MOD_LIGHT_IR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static int tsl2591_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct tsl2591 *tsl2591 = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case 0:
		switch (chan->type) {
		case IIO_INTENSITY:
			ret = tsl2591_get_adc(tsl2591);

			if (!ret) {
				if (chan->channel == 0)
					*val = tsl2591->data0;
				else
					*val = tsl2591->data1;

				ret = IIO_VAL_INT;
			}
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}

	return ret;
}

static ssize_t tsl2591_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl2591 *tsl2591 = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", tsl2591->gain);
}

static ssize_t tsl2591_gain_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl2591 *tsl2591 = iio_priv(indio_dev);
	unsigned long value;
	size_t ret;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	// Only allow 0->3 range
	if ((value < 0) || (value > 3))
		return -EINVAL;

	tsl2591->gain = value;
	ret = (size_t) tsl2591_set_integration(tsl2591);

	return (ret) ? ret : len;
}

static ssize_t tsl2591_int_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl2591 *tsl2591 = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", tsl2591->int_time);
}

static ssize_t tsl2591_int_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tsl2591 *tsl2591 = iio_priv(indio_dev);
	unsigned long value;
	size_t ret;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	// Only allow 100->600 range
	if ((value < 100) || (value > 600))
		return -EINVAL;

	// With 100 step
	if (value % 100)
		return -EINVAL;

	tsl2591->int_time = value;
	ret = (size_t) tsl2591_set_integration(tsl2591);

	return (ret) ? ret : len;
}

static DEVICE_ATTR(illuminance0_gain, S_IRUGO | S_IWUSR,
		tsl2591_gain_show, tsl2591_gain_store);

static DEVICE_ATTR(illuminance0_integration_time, S_IRUGO | S_IWUSR,
		tsl2591_int_time_show, tsl2591_int_time_store);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_illuminance0_integration_time.attr,	/* I time*/
	&dev_attr_illuminance0_gain.attr,  /* gain */
	NULL
};

static struct attribute_group tsl2591_attribute_group = {
	.attrs = sysfs_attrs_ctrl,
};

static const struct iio_info tsl2591_info = {
	.driver_module = THIS_MODULE,
	.read_raw      = &tsl2591_read_raw,
	.attrs         = &tsl2591_attribute_group,
};

static int tsl2591_identify(struct tsl2591* tsl2591)
{
	u8 val;
	int ret;

	ret = tsl2591_read8(tsl2591, TSL2591_ID_R, &val);
	if (ret)
		return ret;

	if (val != TSL2591_DEVICE_ID_VALUE)
		return -ENODEV;

	return 0;
}

static int tsl2591_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct tsl2591 *tsl2591;
	struct iio_dev *indio_dev;

	indio_dev = iio_device_alloc(sizeof(*tsl2591));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "iio allocation failed\n");
		return -ENOMEM;
	}

	tsl2591 = iio_priv(indio_dev);
	tsl2591->i2c = client;
	tsl2591->int_time = 100;
	tsl2591->gain = 0;
	i2c_set_clientdata(client, indio_dev);

	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = tsl2591->i2c->name;
	indio_dev->info = &tsl2591_info;
	indio_dev->channels = tsl2591_channels;
	indio_dev->num_channels = ARRAY_SIZE(tsl2591_channels);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&client->dev, "iio registration failed\n");
		goto fail;
	}

	ret = tsl2591_power(tsl2591, 1);
	if (ret < 0) {
		dev_err(&client->dev, "powerup failed\n");
		goto fail2;
	}

	ret = tsl2591_identify(tsl2591);
	if (ret < 0) {
		dev_err(&client->dev, "identification failed\n");
		goto fail2;
	}

	return 0;

fail2:
	iio_device_unregister(indio_dev);
fail:
	iio_device_free(indio_dev);
	return ret;
}

static int tsl2591_remove(struct i2c_client *client)
{
	iio_device_unregister(i2c_get_clientdata(client));
	iio_device_free(i2c_get_clientdata(client));

	return 0;
}

static const struct i2c_device_id tsl2591_id[] = {
	{"tsl2591", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, tsl2591_id);

static struct i2c_driver tsl2591_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "tsl2591",
	},
	.probe    = tsl2591_probe,
	.remove   = tsl2591_remove,
	.id_table = tsl2591_id,
};

module_i2c_driver(tsl2591_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maxime Jourdan <maxime.jourdan@parrot.com>");
MODULE_AUTHOR("Luis Mario Domenzain <ld@airinov.fr>");
MODULE_DESCRIPTION("TSL2591 illuminance sensor driver");
