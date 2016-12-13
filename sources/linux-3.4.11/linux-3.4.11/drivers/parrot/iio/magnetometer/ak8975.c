/*
 * A sensor driver for the magnetometer AK8975/AK8963.
 *
 * Magnetic compass sensor driver for monitoring magnetic flux information.
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>
#ifdef CONFIG_PARROT_AK8963_NOTRIGGER
#include <linux/iio/kfifo_buf.h>
#endif
#include <iio/platform_data/mykonos3.h>
#include <linux/platform_data/ak8975.h>
#include "ak8975_regs.h"

/* module parameters */
static unsigned mode = 1;
module_param(mode, int, S_IRUGO);

static unsigned precision = 16;
module_param(precision, int, S_IRUGO);

enum ak8975_timestamps_list {
  TS_trigger,
  TS_request,
  TS_data_ready,
  TS_push,
  TS_NR
};

/* Compatible Asahi Kasei Compass parts */
enum asahi_compass_chipset {
	AK8975,
	AK8963,
};

/*
 * Per-instance context data for the device.
 */
struct ak8975_data {
	struct i2c_client		*client;
	struct ak8975_platform_data	*pdata;
	struct attribute_group		attrs;
	struct mutex			lock;
	u8				asa[3];
	long long			raw_to_gauss[3];
	int 				clamp[2];
	bool				mode;
	u8				reg_cache[AK8975_MAX_REGS];
	int				drdy_gpio;/* DRDY pin */
	int				drdy_irq; /* DRDY interrupt */
	int				trg_gpio;  /* TRG pin on AK8963 */
	wait_queue_head_t		data_ready_queue;
	unsigned long			flags;
	char 				name[32];
	enum asahi_compass_chipset	chipset;
	s64				timestamps[TS_NR];
};

static const int ak8975_index_to_reg[] = {
	AK8975_REG_HXL, AK8975_REG_HYL, AK8975_REG_HZL,
};

/*
 * Helper function to write to the I2C device's registers.
 */
static int ak8975_write_data(struct i2c_client *client,
			     u8 reg, u8 val, u8 mask, u8 shift)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ak8975_data *data = iio_priv(indio_dev);
	u8 regval;
	int ret;

	regval = (data->reg_cache[reg] & ~mask) | (val << shift);
	ret = i2c_smbus_write_byte_data(client, reg, regval);
	if (ret < 0) {
		dev_err(&client->dev, "Write to device fails status %x\n", ret);
		return ret;
	}
	data->reg_cache[reg] = regval;

	return 0;
}

#ifdef CONFIG_PARROT_AK8963_NOTRIGGER
/*
 * Handle data ready irq
 */
static irqreturn_t ak8975_irq_handler(int irq, void *data)
{
	struct ak8975_data *ak8975 = data;
	struct iio_dev *indio_dev = i2c_get_clientdata(ak8975->client);

	set_bit(0, &ak8975->flags);
	ak8975->timestamps[TS_data_ready] = iio_get_time_ns(indio_dev);

	return IRQ_HANDLED;
}
#else
static irqreturn_t ak8975_irq_handler(int irq, void *data)
{
	struct ak8975_data *ak8975 = data;

	set_bit(0, &ak8975->flags);
	wake_up(&ak8975->data_ready_queue);

	return IRQ_HANDLED;
}

#endif

/*
 * Install data ready interrupt handler
 */
static int ak8975_setup_irq(struct ak8975_data *data)
{
	struct i2c_client *client = data->client;
	int rc;
	int irq;

	if (client->irq >= 0)
		irq = client->irq;
	else
		irq = gpio_to_irq(data->drdy_gpio);

	rc = devm_request_irq(&client->dev, irq,
			      ak8975_irq_handler,
			      IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			      data->name, data);
	if (rc < 0) {
		dev_err(&client->dev,
			"irq %d request failed: %d\n",
			irq, rc);
		return rc;
	}
	dev_info(&client->dev, "requested IRQ %d\n", irq);

	init_waitqueue_head(&data->data_ready_queue);
	clear_bit(0, &data->flags);
	data->drdy_irq = irq;

	return rc;
}

/*
 * Check device id
 */
static int ak8975_check_id(struct i2c_client *client)
{
	u8 device_id;
	int ret;
	/* Confirm that the device we're talking to is really an AK8975. */
	ret = i2c_smbus_read_byte_data(client, AK8975_REG_WIA);
	if (ret < 0) {
		dev_err(&client->dev, "Error reading WIA\n");
		return ret;
	}
	device_id = ret;
	if (device_id != AK8975_DEVICE_ID) {
		dev_err(&client->dev, "Device ak8975 not found\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * Perform some start-of-day setup, including reading the asa calibration
 * values and caching them.
 */
static int ak8975_setup(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ak8975_data *data = iio_priv(indio_dev);
	bool output_setting;
	int ret;

	if ((ret=ak8975_check_id(client))<0)
		return ret;

	/* Write the fused rom access mode. */
	ret = ak8975_write_data(client,
				AK8975_REG_CNTL,
				AK8975_REG_CNTL_MODE_FUSE_ROM,
				AK8975_REG_CNTL_MODE_MASK,
				AK8975_REG_CNTL_MODE_SHIFT);
	if (ret < 0) {
		dev_err(&client->dev, "Error in setting fuse access mode\n");
		return ret;
	}

	/* Get asa data and store in the device data. */
	ret = i2c_smbus_read_i2c_block_data(client, AK8975_REG_ASAX,
					    3, data->asa);
	if (ret < 0) {
		dev_err(&client->dev, "Not able to read ASA data\n");
		return ret;
	}

	/* After reading fuse ROM data set power-down mode */
	ret = ak8975_write_data(client,
				AK8975_REG_CNTL,
				AK8975_REG_CNTL_MODE_POWER_DOWN,
				AK8975_REG_CNTL_MODE_MASK,
				AK8975_REG_CNTL_MODE_SHIFT);

	if (ret < 0) {
		dev_err(&client->dev, "Error in setting power-down mode\n");
		return ret;
	}

	/* We may not have a GPIO based IRQ to scan, that is fine, we will
	 *	   poll if so */
	if (gpio_is_valid(data->drdy_gpio)) {
		ret = gpio_request_one(data->drdy_gpio, GPIOF_IN, "ak8975_drdy");
		if (ret < 0) {
			dev_err(&client->dev,
				"failed to request DRDY GPIO %d, error %d\n",
				data->drdy_gpio, ret);
			return ret;
		}
		dev_info(&client->dev, "requested DRDY GPIO %d\n", data->drdy_gpio);
	}

	if (data->drdy_gpio >= 0 || client->irq >= 0) {
		ret = ak8975_setup_irq(data);
		if (ret < 0) {
			dev_err(&client->dev,
				"Error setting data ready interrupt\n");
			return ret;
		}
	}

	if (data->pdata) {
		if (data->chipset == AK8963) {
			if (gpio_is_valid(data->pdata->trg_gpio)) {
				int err = gpio_request_one(data->pdata->trg_gpio, GPIOF_IN, "ak8963_trg");
				if (err < 0) {
					dev_err(&client->dev,
						"failed to request TRG GPIO %d, error %d\n",
						data->pdata->trg_gpio, err);
				}
				else {
					data->trg_gpio = data->pdata->trg_gpio;
					dev_info(&client->dev,
						 "requested TRG GPIO %d\n",
						 data->trg_gpio);
				}
			}
		}
	}
/*
 * Precalculate scale factor (in Gauss units) for each axis and
 * store in the device data.
 *
 * This scale factor is axis-dependent, and is derived from 3 calibration
 * factors ASA(x), ASA(y), and ASA(z).
 *
 * These ASA values are read from the sensor device at start of day, and
 * cached in the device context struct.
 *
 * Adjusting the flux value with the sensitivity adjustment value should be
 * done via the following formula:
 *
 * Hadj = H * ( ( ( (ASA-128)*0.5 ) / 128 ) + 1 )
 *
 * where H is the raw value, ASA is the sensitivity adjustment, and Hadj
 * is the resultant adjusted value.
 *
 * We reduce the formula to:
 *
 * Hadj = H * (ASA + 128) / 256
 *
 * Chip   | precision | H range         | Flux range (uT) | Scale
 * AK8975 | 13        |  -4096 to  4095 | -1229 to 1229   | 1229/4095  0,300122100122 |
 * AK8963 | 14        |  -8190 to  8190 | -4912 to 4912   | 4912/8190  0,599755799756 |
 * AK8963 | 16        | -32760 to 32760 | -4912 to 4912   | 4912/32760 0,149938949939 |
 *
 *
 * To go from the raw value to uT is:
 *
 * HuT = H * scale.
 *
 * Since 1uT = 0.01 gauss, and to have better precision with mG,
 * our final scale factor becomes:
 * *10000:  1 T = 10000 G
 * *1000: for mG
 * *1000: to 10e-9 precision
 * Hadj = H * ((ASA + 128) / 256) * scale * 10000*1000
 *
 * Since ASA doesn't change, we cache the resultant scale factor into the
 * device context in ak8975_setup().
 */
#define RAW_TO_mGAUSS_8975(asa)        ((((asa) + 128) * (1229*10000000000LL/4095)) / 256)
#define RAW_TO_mGAUSS_8963_14BITS(asa) ((((asa) + 128) * (4912*10000000000LL/8190)) / 256)
#define RAW_TO_mGAUSS_8963_16BITS(asa) ((((asa) + 128) * (4912*10000000000LL/32760)) / 256)
	if (data->chipset == AK8963) {
		if ( precision == 16 ) {
			output_setting = 1;
			data->raw_to_gauss[0] = RAW_TO_mGAUSS_8963_16BITS(data->asa[0]);
			data->raw_to_gauss[1] = RAW_TO_mGAUSS_8963_16BITS(data->asa[1]);
			data->raw_to_gauss[2] = RAW_TO_mGAUSS_8963_16BITS(data->asa[2]);
			data->clamp[0] = -32760;
			data->clamp[1] = +32760;
			precision = 16;
			dev_info(&client->dev,
				"AK8963 16 bits precision\n");
		}
		else {
			output_setting = 0;
			data->raw_to_gauss[0] = RAW_TO_mGAUSS_8963_14BITS(data->asa[0]);
			data->raw_to_gauss[1] = RAW_TO_mGAUSS_8963_14BITS(data->asa[1]);
			data->raw_to_gauss[2] = RAW_TO_mGAUSS_8963_14BITS(data->asa[2]);
			data->clamp[0] = -8190;
			data->clamp[1] = +8190;
			precision = 14;
			dev_info(&client->dev,
				 "AK8963 14 bits precision\n");
		}
		/* Set up precision on ak8963. */
		ret = ak8975_write_data(client,
					AK8975_REG_CNTL,
					output_setting,
					AK8963_REG_CNTL_BIT_MASK,
					AK8963_REG_CNTL_BIT_SHIFT);
		if (ret < 0) {
		  dev_err(&client->dev, "Error in setting AK8963 precision mode\n");
		  return ret;
		}
	} else {
		data->raw_to_gauss[0] = RAW_TO_mGAUSS_8975(data->asa[0]);
		data->raw_to_gauss[1] = RAW_TO_mGAUSS_8975(data->asa[1]);
		data->raw_to_gauss[2] = RAW_TO_mGAUSS_8975(data->asa[2]);
		data->clamp[0] = -4096;
		data->clamp[1] = +4095;
		precision = 13;
		dev_info(&client->dev,
			 "AK8975 13 bits precision\n");
	}
	dev_info(&client->dev,
		 "Scale X %Ld, Y %Ld, Z %Ld\n", data->raw_to_gauss[0], data->raw_to_gauss[1], data->raw_to_gauss[2]);

	return 0;
}

/*
 * Shows the device's mode.  0 = off, 1 = on.
 */
static ssize_t show_mode(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ak8975_data *data = iio_priv(indio_dev);

	return sprintf(buf, "%u\n", data->mode);
}

/*
 * Sets the device's mode.  0 = off, 1 = on.  The device's mode must be on
 * for the magn raw attributes to be available.
 */
static ssize_t store_mode(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ak8975_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	bool value;
	int ret;

	/* Convert mode string and do some basic sanity checking on it.
	   only 0 or 1 are valid. */
	ret = strtobool(buf, &value);
	if (ret < 0)
		return ret;

	mutex_lock(&data->lock);

	/* Write the mode to the device. */
	if (data->mode != value) {
		ret = ak8975_write_data(client,
					AK8975_REG_CNTL,
					(u8)value,
					AK8975_REG_CNTL_MODE_MASK,
					AK8975_REG_CNTL_MODE_SHIFT);

		if (ret < 0) {
			dev_err(&client->dev, "Error in setting mode\n");
			mutex_unlock(&data->lock);
			return ret;
		}
		data->mode = value;
	}

	mutex_unlock(&data->lock);

	return count;
}

/*
 * Shows the device's orientation matrix.
 */
static ssize_t show_in_magn_matrix(struct device *dev, struct device_attribute *devattr,
				   char *buf)
{
  struct iio_dev *indio_dev = dev_get_drvdata(dev);
  struct ak8975_data *data = iio_priv(indio_dev);

  return sprintf(buf, "%s", data->pdata->orientation);
}

#ifdef PARROT_IIO_AK8975_TIMESTAMPS
/*
 * Shows the device's timestamps.
 */
static ssize_t show_timestamps(struct device *dev, struct device_attribute *devattr,
			       char *buf)
{
  struct iio_dev *indio_dev = dev_get_drvdata(dev);
  struct ak8975_data *data = iio_priv(indio_dev);
  int i, c=0;

  for (i=TS_trigger; i< TS_NR; i++) {
	  c+=sprintf(&buf[c],"%Ld ", data->timestamps[i]);
  }
  return c;
}
#endif /* PARROT_IIO_AK8975_TIMESTAMPS */

static int wait_conversion_complete_gpio(struct ak8975_data *data)
{
	struct i2c_client *client = data->client;
	u32 timeout_ms = AK8975_MAX_CONVERSION_TIMEOUT;
	int ret;

	/* Wait for the conversion to complete. */
	dev_dbg(&client->dev, "%s on GPIO %d\n", __func__, data->drdy_gpio);
	while (timeout_ms) {
		msleep(AK8975_CONVERSION_DONE_POLL_TIME);
		if (gpio_get_value(data->drdy_gpio))
			break;
		timeout_ms -= AK8975_CONVERSION_DONE_POLL_TIME;
	}
	if (!timeout_ms) {
		dev_err(&client->dev, "Conversion timeout happened\n");
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(client, AK8975_REG_ST1);
	if (ret < 0)
		dev_err(&client->dev, "Error in reading ST1\n");

	return ret;
}

static int wait_conversion_complete_polled(struct ak8975_data *data)
{
	struct i2c_client *client = data->client;
	u8 read_status;
	u32 timeout_ms = AK8975_MAX_CONVERSION_TIMEOUT;
	int ret;

	/* Wait for the conversion to complete. */
	dev_dbg(&client->dev, "%s\n", __func__);
	while (timeout_ms) {
		msleep(AK8975_CONVERSION_DONE_POLL_TIME);
		ret = i2c_smbus_read_byte_data(client, AK8975_REG_ST1);
		if (ret < 0) {
			dev_err(&client->dev, "Error in reading ST1\n");
			return ret;
		}
		read_status = ret;
		if (read_status & AK8975_REG_ST1_DRDY_MASK)
			break;
		timeout_ms -= AK8975_CONVERSION_DONE_POLL_TIME;
	}
	if (!timeout_ms) {
		dev_err(&client->dev, "Conversion timeout happened\n");
		return -EINVAL;
	}
	return read_status;
}

/* Returns 0 if the end of conversion interrupt occured or -ETIME otherwise */
static int wait_conversion_complete_interrupt(struct ak8975_data *data)
{
	int ret;

	dev_dbg(&data->client->dev, "%s on IRQ %d\n", __func__, data->drdy_irq);

	ret = wait_event_timeout(data->data_ready_queue,
				 test_bit(0, &data->flags),
				 AK8975_DATA_READY_TIMEOUT);
	clear_bit(0, &data->flags);

	return ret > 0 ? 0 : -ETIME;
}

#ifdef CONFIG_PARROT_AK8963_NOTRIGGER
static int ak8975_request_data(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct ak8975_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;

	data->timestamps[TS_request] = iio_get_time_ns(indio_dev);
	dev_dbg(&client->dev, "%s\n", __func__);

	/* Set up the device for taking a sample. */
	ret = ak8975_write_data(client,
				AK8975_REG_CNTL,
				AK8975_REG_CNTL_MODE_ONCE,
				AK8975_REG_CNTL_MODE_MASK,
				AK8975_REG_CNTL_MODE_SHIFT);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s: Error in setting operating mode\n", __func__);
		goto exit;
	}

	dev_dbg(&client->dev, "%s: Single measurement mode\n", __func__);

exit:
	return ret;
}

static int ak8975_fetch_data(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct ak8975_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	u8 buffer[16];

	if (test_bit(0, &data->flags)) {
		clear_bit(0, &data->flags);
	} else {
		ret = -EAGAIN;
		goto exit;
	}

	ret = i2c_smbus_read_byte_data(client, AK8975_REG_ST1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Error in reading ST1\n", __func__);
		goto exit;
	}

	ret = i2c_smbus_read_i2c_block_data(data->client,
					    AK8975_REG_HXL,
					    3 * sizeof(u16),
					    buffer);
	if (ret < 0)
		goto exit;

	data->timestamps[TS_push] = iio_get_time_ns(indio_dev);
	iio_push_to_buffers_with_timestamp(indio_dev, buffer,
					   data->timestamps[TS_push]);

exit:
	return ret;
}
#endif

/* trigger single measurement */
static int ak8975_request_single(struct ak8975_data *data)
{
	struct i2c_client *client = data->client;
	int ret;
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	data->timestamps[TS_request] = iio_get_time_ns(indio_dev);
	dev_dbg(&client->dev, "%s\n", __func__);

	/* Set up the device for taking a sample. */
	ret = ak8975_write_data(client,
				AK8975_REG_CNTL,
				AK8975_REG_CNTL_MODE_ONCE,
				AK8975_REG_CNTL_MODE_MASK,
				AK8975_REG_CNTL_MODE_SHIFT);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Error in setting operating mode\n", __func__);
		return ret;
	}

	dev_dbg(&client->dev, "%s: Single measurement mode\n", __func__);

	/* Wait for the conversion to complete. */
	if (data->drdy_irq >=0 ) {
		ret = wait_conversion_complete_interrupt(data);
		if (ret == 0) {
			ret = i2c_smbus_read_byte_data(client, AK8975_REG_ST1);
			if (ret < 0) {
				dev_err(&client->dev, "%s: Error in reading ST1\n", __func__);
				return ret;
			}
		}
	}
	else if (gpio_is_valid(data->drdy_gpio))
		ret = wait_conversion_complete_gpio(data);
	else
		ret = wait_conversion_complete_polled(data);
	if (ret < 0)
		return ret;
	data->timestamps[TS_data_ready] = iio_get_time_ns(indio_dev);

	dev_dbg(&client->dev, "%s: ST1: %02X\n", __func__, ret);

	if (ret & AK8975_REG_ST1_DRDY_MASK) {
		ret = i2c_smbus_read_byte_data(client, AK8975_REG_ST2);
		if (ret < 0) {
			dev_err(&client->dev, "%s: Error in reading ST2\n", __func__);
			return ret;
		}

		dev_dbg(&client->dev, "%s: ST2: %02X\n", __func__, ret);

		if (ret & (AK8975_REG_ST2_DERR_MASK |
			AK8975_REG_ST2_HOFL_MASK)) {
			dev_err(&client->dev, "%s: ST2 status error 0x%x\n", __func__, ret);
			ret = -EINVAL;
			return ret;
		}
	}

	return 0;
}

/*
 * Emits the raw flux value for the x, y, or z axis.
 */
static int ak8975_read_axis(struct iio_dev *indio_dev, int index, int *val)
{
	struct ak8975_data *data = iio_priv(indio_dev);
	struct i2c_client *client = data->client;
	int ret;

	data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);
	mutex_lock(&data->lock);

	if (data->mode == 0) {
		dev_err(&client->dev, "Operating mode is in power down mode\n");
		ret = -EBUSY;
		goto exit;
	}

	ret = ak8975_request_single(data);
	if (ret < 0)
		goto exit;

	/* Read the flux value from the appropriate register
	 *	   (the register is specified in the iio device attributes). */
	ret = i2c_smbus_read_word_data(client, ak8975_index_to_reg[index]);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Read axis data fails\n", __func__);
		goto exit;
	}

	mutex_unlock(&data->lock);

	/* Clamp to valid range. */
	*val = clamp_t(s16, ret, data->clamp[0], data->clamp[1]);
	data->timestamps[TS_push] = iio_get_time_ns(indio_dev);
	return IIO_VAL_INT;

exit:
	mutex_unlock(&data->lock);
	return ret;
}

static int ak8975_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2,
			   long mask)
{
	struct ak8975_data *data = iio_priv(indio_dev);
	int tmp;

	switch (mask) {
	  case IIO_CHAN_INFO_RAW:
		*val2 = 0;
		return ak8975_read_axis(indio_dev, chan->address, val);
	case IIO_CHAN_INFO_SCALE:
		tmp   = data->raw_to_gauss[chan->address];
		*val  = tmp / 1000000000L;
		*val2 = tmp % 1000000000L;
		return IIO_VAL_INT_PLUS_NANO;
	}
	return -EINVAL;
}

#ifndef CONFIG_PARROT_AK8963_NOTRIGGER
static int ak8975_read(struct ak8975_data *data, u16 buf[3])
{
  int ret;

  mutex_lock(&data->lock);
  ret = ak8975_request_single(data);
  if (ret < 0) {
	  mutex_unlock(&data->lock);
	  return ret;
  }
  ret = i2c_smbus_read_i2c_block_data(data->client,
				      AK8975_REG_HXL, 3 * sizeof(u16), (u8 *) buf);
  mutex_unlock(&data->lock);

  return ret;
}

static irqreturn_t ak8975_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ak8975_data *data = iio_priv(indio_dev);
	u8 buffer[16]; /* 3 16-bit channels + padding + ts */
	int ret;

	data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);

	ret = ak8975_read(data, (u16 *) buffer);
	if (ret < 0)
		goto done;

	data->timestamps[TS_push] = iio_get_time_ns(indio_dev);
	iio_push_to_buffers_with_timestamp(indio_dev, buffer,
					   data->timestamps[TS_push]);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}
#endif

enum ak8975_channel_index {
      CHAN_MAGN_X=0,
      CHAN_MAGN_Y,
      CHAN_MAGN_Z,
      CHAN_MAGN_TIMESTAMP,
      CHAN_MAGN_NR
};

#define AK8975_CHANNEL(axis)					\
	{								\
		.type = IIO_MAGN,					\
		.modified = 1,						\
		.channel2 = IIO_MOD_##axis,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			     BIT(IIO_CHAN_INFO_SCALE),			\
		.address = CHAN_MAGN_##axis,				\
		.scan_index = CHAN_MAGN_##axis,				\
		.scan_type = {						\
				.sign = 's',				\
				.storagebits = 16,			\
				.realbits = 16,				\
				.shift = 0,				\
				.endianness = IIO_LE,			\
			     }						\
	}

static const struct iio_chan_spec ak8975_channels[] = {
	AK8975_CHANNEL(X),
	AK8975_CHANNEL(Y),
	AK8975_CHANNEL(Z),
	/*
	 * Convenience macro for timestamps.
	 * CHAN_MAGN_TIMESTAMP is the index in the buffer.
	 */
	IIO_CHAN_SOFT_TIMESTAMP(CHAN_MAGN_TIMESTAMP)
};

static IIO_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR, show_mode, store_mode,  0);
static IIO_DEVICE_ATTR(in_magn_matrix, S_IRUGO, show_in_magn_matrix, NULL, 1);

#ifdef PARROT_IIO_AK8975_TIMESTAMPS
static IIO_DEVICE_ATTR(timestamps, S_IRUGO, show_timestamps, NULL, 2);
#endif /* PARROT_IIO_AK8975_TIMESTAMPS */

static struct attribute *ak8975_attr[] = {
	&iio_dev_attr_mode.dev_attr.attr,
	&iio_dev_attr_in_magn_matrix.dev_attr.attr,
#ifdef PARROT_IIO_AK8975_TIMESTAMPS
	&iio_dev_attr_timestamps.dev_attr.attr,
#endif /* PARROT_IIO_AK8975_TIMESTAMPS */
	NULL
};

static struct attribute_group ak8975_attr_group = {
	.attrs = ak8975_attr,
};

static const struct iio_info ak8975_info = {
	.attrs = &ak8975_attr_group,
	.read_raw = &ak8975_read_raw,
	.driver_module = THIS_MODULE,
};

static const unsigned long ak8975_scan_masks[] = {0x7, 0}; /* 3 channels X Y Z */

#ifdef CONFIG_PARROT_AK8963_NOTRIGGER
static const struct iio_buffer_setup_ops ak8975_iio_buffer_setup_ops = {
};

struct iio_mykonos3_ops	ak8975_iio_mykonos3_ops = {
	.request_data = ak8975_request_data,
	.fetch_data   = ak8975_fetch_data,
};
#endif

static int ak8975_iio_buffer_new(struct iio_dev *indio_dev)
{
	int err = 0;
#ifdef CONFIG_PARROT_AK8963_NOTRIGGER
	struct iio_buffer *buffer = NULL;

	indio_dev->modes     = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ak8975_iio_buffer_setup_ops;

	buffer = iio_kfifo_allocate(indio_dev);
	if (!buffer) {
		err = -ENOMEM;
		goto exit;
	}

	iio_device_attach_buffer(indio_dev, buffer);

	err = iio_buffer_register(indio_dev, ak8975_channels,
				  ARRAY_SIZE(ak8975_channels));
	if (err < 0)
		goto exit;

	iio_mykonos3_register(IIO_MYKONOS3_MAGNETO,
			      indio_dev, &ak8975_iio_mykonos3_ops);
#else
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_triggered_buffer_setup(indio_dev, NULL,
					 ak8975_trigger_handler, NULL);
	if (err < 0)
		goto exit;
#endif
exit:
	return err;
}

static void ak8975_iio_buffer_cleanup(struct iio_dev *indio_dev)
{
#ifdef CONFIG_PARROT_AK8963_NOTRIGGER
	iio_kfifo_free(indio_dev->buffer);
	iio_mykonos3_unregister(IIO_MYKONOS3_MAGNETO, indio_dev);
#else
	iio_triggered_buffer_cleanup(indio_dev);
#endif
	iio_buffer_unregister(indio_dev);
}

static int ak8975_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ak8975_data		*data = NULL;
	struct ak8975_platform_data	*pdata = NULL;
	struct iio_dev			*indio_dev = NULL;
	int				drdy_gpio = -1;
	int				err;
	const char			*name = NULL;

	if ((err=ak8975_check_id(client))<0)
	  return err;

	/* Grab and set up the supplied GPIO for DRDY. */
	pdata = dev_get_platdata(&client->dev);
	if (pdata)
		drdy_gpio = pdata->drdy_gpio;

	if (drdy_gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	/* Register with IIO */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (indio_dev == NULL)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	data->pdata = pdata;
	data->drdy_gpio = drdy_gpio;
	data->drdy_irq = -1;
	data->trg_gpio = -1;
	data->mode = mode;

	if (id) {
		data->chipset = (enum asahi_compass_chipset)(id->driver_data);
		name = id->name;
	} else {
		return -ENOSYS;
	}
	dev_info(&client->dev, "Asahi Kasei Compass chip %s\n", name);

	snprintf(data->name,
		 sizeof(data->name),
		 "%s_%s",
		 name,
		 dev_name(&client->dev));

	/* Perform some basic start-of-day setup of the device. */
	err = ak8975_setup(client);
	if (err < 0) {
		dev_err(&client->dev, "AK8975 initialization fails\n");
		goto exit_gpio;
	}

	mutex_init(&data->lock);
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = ak8975_channels;
	indio_dev->num_channels = ARRAY_SIZE(ak8975_channels);
	indio_dev->info = &ak8975_info;
	indio_dev->available_scan_masks = ak8975_scan_masks;
	indio_dev->name = name;

	err = ak8975_iio_buffer_new(indio_dev);
	if (err < 0)
		goto exit_gpio;

	err = devm_iio_device_register(&client->dev, indio_dev);
	if (err < 0)
		goto buffer_cleanup;

	return 0;

buffer_cleanup:
	ak8975_iio_buffer_cleanup(indio_dev);
exit_gpio:
	if (data->drdy_irq >= 0)
		devm_free_irq(&client->dev, data->drdy_irq, data);
	if (gpio_is_valid(data->drdy_gpio))
		gpio_free(data->drdy_gpio);
	if (gpio_is_valid(data->trg_gpio))
		gpio_free(data->trg_gpio);
	return err;
}

static int ak8975_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ak8975_data *data = iio_priv(indio_dev);

	ak8975_iio_buffer_cleanup(indio_dev);
	devm_iio_device_unregister(&client->dev, indio_dev);

	if (data->drdy_irq >= 0)
		devm_free_irq(&client->dev, data->drdy_irq, data);
	if (gpio_is_valid(data->drdy_gpio))
		gpio_free(data->drdy_gpio);
	if (gpio_is_valid(data->trg_gpio))
		gpio_free(data->trg_gpio);

	return 0;
}

static const struct i2c_device_id ak8975_id[] = {
	{IIO_MAGNETOMETER_AK8975, AK8975},
	{IIO_MAGNETOMETER_AK8963, AK8963},
	{"AK8963",                AK8963},
	{}
};

MODULE_DEVICE_TABLE(i2c, ak8975_id);

static struct i2c_driver ak8975_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= IIO_MAGNETOMETER_AK8975,
	},
	.probe		= ak8975_probe,
	.remove		= __devexit_p(ak8975_remove),
	.id_table	= ak8975_id,
};
module_i2c_driver(ak8975_driver);

MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_DESCRIPTION("AK8975/AK8963 magnetometer driver");
MODULE_LICENSE("GPL");
