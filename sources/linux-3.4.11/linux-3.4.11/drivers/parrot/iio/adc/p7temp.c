/**
 ***********************************************
 * @file p7-temperature.c
 * @brief P7 Temperature driver
 *
 * Copyright (C) 2013 Parrot S.A.
 * Check https://smet/issues/10348 for more information
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @author François MULLER <francois.muller@parrot.com>
 *
 * @date 2013-09-19
 ***********************************************
 */
// #define DEBUG
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/pinctrl/consumer.h>

#include <linux/sort.h>
#include <asm/io.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>

#include <mfd/p7mu.h>
#include "p7-adc_regs.h"
#include "p7-adc_core.h"

#define P7_TEMP_DRV_NAME "p7-temperature"

#define P7_ADC_CHANNEL			0
#define P7MU_ADC_CHANNEL		7

#define FIXED_POINT_DIV_FACTOR	8
#define ALPHA_DIV_FACTOR		0x100
#define TEMP_NUM_ROUND			(4*4) // Need to be a multiple of 4
#define HALF_TEMP_NUM_ROUND		(TEMP_NUM_ROUND/2)
#define GPIO_V1					49
#define GPIO_V2					50
#define PINCTRL_STATE_VOLTAGE1 "v1"
#define PINCTRL_STATE_VOLTAGE2 "v2"
#define GPIO_TIME       (8000) /* value in us : There are a big capacitor. This capacitor takes a least 8 ms to be charged or discharged */

#define P7MU_OTP_PAGE             ((u16) 0x0e00)
#define P7MU_OTP_TS_POINT_0       (P7MU_OTP_PAGE + 0xc)
#define P7MU_OTP_TS_POINT_1       (P7MU_OTP_PAGE + 0xd)

#define P7MU_OTP_INTERRUPT_EN_1   (P7MU_OTP_PAGE + 0x12) // P7MU_ADC_OFFSET_0V5_TEMP_0 (val >> 8)
#define P7MU_OTP_TEMP_ERR_TIME    (P7MU_OTP_PAGE + 0x13) // P7MU_ADC_OFFSET_2V7_TEMP_0 (val >> 8)
#define P7MU_OTP_IC_SETUP         (P7MU_OTP_PAGE + 0x18) // P7MU_ADC_OFFSET_0V5_TEMP_1 (val >> 8)
#define P7MU_OTP_CLK_GEN_BCKP_CFG (P7MU_OTP_PAGE + 0x1A) // P7MU_ADC_OFFSET_2V7_TEMP_1 (val >> 8)

struct p7_temp_state
{
	unsigned int                     type;
	int                              num_channels;
	int                              gpio_time;
	struct timer_list                poll_timer;
	struct p7_adc_iio_chip_channel_ops *ops;
	// p7mu temperature calibration
	u16                              ts_temp_0;
	int                              ts_adc_read_temp_0;
	u16                              ts_temp_1;
	int                              ts_adc_read_temp_1;
	// p7 temperature calibration
	s8                               offset_0v5_0;
	s8                               offset_2v7_0;
	s8                               offset_0v5_1;
	s8                               offset_2v7_1;
	s8                               calibration_valid;
};

int compare (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

static int p7_temperature_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long m)
{
	int ret;
	int v1, v2, round, sum = 0, alpha = 0;
	int adc_offset_0v5, adc_offset_2v7;
	int values[TEMP_NUM_ROUND];
	u16 data;
	struct p7_temp_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	if(m != IIO_CHAN_INFO_RAW || chan->type != IIO_TEMP) {
		ret = -EINVAL;
		goto out;
	}/* else Reading the current channel */

	switch (chan->channel) {

	case P7_ADC_CHANNEL:
	case P7MU_ADC_CHANNEL:
/***************** P7MU_ADC_CHANNEL *************************/
		for (round = 0; round < TEMP_NUM_ROUND; round++) {
			ret = st->ops->read(P7MU_ADC_CHANNEL, &data);
			if (ret) {
				dev_err(&indio_dev->dev, "p7mu ADC read not valid. Got 0x%x\n", data);
				ret = -EINVAL;
				goto out;
			}
			values[round] = data >> chan->scan_type.shift;
			values[round] &= (1 << chan->scan_type.realbits) - 1;
		}
		// Sort values so that we remove first quarter and last quarter results
		sort (values, TEMP_NUM_ROUND, sizeof(int), compare, NULL);
		// With that we keep only "correct" values and we remove "spike" values from mean calculation
		for(round = TEMP_NUM_ROUND/4; round < (3*TEMP_NUM_ROUND)/4; round++)
			sum += values[round];

		// Check if OTP value are ok
		if(st->calibration_valid) {
			alpha = ALPHA_DIV_FACTOR * (sum - st->ts_adc_read_temp_0 * HALF_TEMP_NUM_ROUND) /
				((st->ts_adc_read_temp_1 - st->ts_adc_read_temp_0) * HALF_TEMP_NUM_ROUND);
		}

		dev_dbg(&indio_dev->dev, "alpha = %i\n", alpha);

		if(chan->channel == P7MU_ADC_CHANNEL) {
			if(st->calibration_valid)
				*val = st->ts_temp_0 + ((st->ts_temp_1 - st->ts_temp_0)*alpha + (int)(0.5*ALPHA_DIV_FACTOR)) / ALPHA_DIV_FACTOR;
			else
				*val = (sum * 185 / (4095 * HALF_TEMP_NUM_ROUND)) - 50;

			ret = IIO_VAL_INT;
			goto out;
		}
		//no break;
/***************** P7_ADC_CHANNEL *************************/
		adc_offset_0v5 = st->offset_0v5_0*FIXED_POINT_DIV_FACTOR +
			alpha * (st->offset_0v5_1 - st->offset_0v5_0) /
			(ALPHA_DIV_FACTOR/FIXED_POINT_DIV_FACTOR);
		adc_offset_2v7 = st->offset_2v7_0*FIXED_POINT_DIV_FACTOR +
			alpha * (st->offset_2v7_1 - st->offset_2v7_0) /
			(ALPHA_DIV_FACTOR/FIXED_POINT_DIV_FACTOR);

		dev_dbg(&indio_dev->dev, "adc_offset_0v5=%i adc_offset_2v7%i\n", adc_offset_0v5, adc_offset_2v7);

		sum = 0;

		gpio_direction_output(GPIO_V1, 1);
		gpio_direction_input(GPIO_V2);
		usleep_range(st->gpio_time, st->gpio_time);

		for (round = 0; round < TEMP_NUM_ROUND; round++) {
			ret = st->ops->read(chan->address, &data);
			if (ret) {
				dev_err(&indio_dev->dev, "v1 ADC read not valid. Got 0x%x\n", data);
				goto out;
			}
			v1 = data >> chan->scan_type.shift;
			v1 &= (1 << chan->scan_type.realbits) - 1;

			values[round] = v1;
		}
		gpio_direction_output(GPIO_V2, 1);
		gpio_direction_input(GPIO_V1);
		usleep_range(st->gpio_time, st->gpio_time);

		for (round = 0; round < TEMP_NUM_ROUND; round++) {
			ret = st->ops->read(chan->address, &data);
			if (ret) {
				dev_err(&indio_dev->dev, "v2 ADC read not valid. Got 0x%x\n", data);
				ret = -EINVAL;
				goto out;
			}
			v2 = data >> chan->scan_type.shift;
			v2 &= (1 << chan->scan_type.realbits) - 1;

			// It's better to "randomize" the substract part than to sort each v1 v2 and substract at the end
			values[round] = values[round] - v2;
		}
		// Sort values so that we remove first quarter and last quarter results
		sort (values, TEMP_NUM_ROUND, sizeof(int), compare, NULL);
		// With that we keep only "correct" values and we remove "spike" values from mean calculation
		for(round = TEMP_NUM_ROUND/4; round < (3*TEMP_NUM_ROUND)/4; round++)
			sum += values[round];

		dev_dbg(&indio_dev->dev, "%i %i %i\n", sum,
			(sum * (int)(4313.72 * FIXED_POINT_DIV_FACTOR)),
			((sum * (int)(4313.72 * FIXED_POINT_DIV_FACTOR) /
			(adc_offset_2v7 - adc_offset_0v5 + 3003*FIXED_POINT_DIV_FACTOR))));

		// sum is a multiple of HALF_TEMP_NUM_ROUND
		*val = (((sum * (int)((4313.72) * FIXED_POINT_DIV_FACTOR) /
				(adc_offset_2v7 - adc_offset_0v5 + 3003*FIXED_POINT_DIV_FACTOR)) -
				(int)(304.7*HALF_TEMP_NUM_ROUND)) + (int)(0.5*HALF_TEMP_NUM_ROUND))
				/ (HALF_TEMP_NUM_ROUND);

		ret = IIO_VAL_INT;
		break;

	default:
		ret = -EINVAL;
		break;
	}
out:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static ssize_t p7_temp_show_t_alarm_high(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return 0;
}

static inline ssize_t p7_temp_set_t_alarm_high(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return 0;
}

static ssize_t p7_temp_show_t_alarm_low(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return 0;
}

static inline ssize_t p7_temp_set_t_alarm_low(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return 0;
}

static IIO_DEVICE_ATTR(t_alarm_high,
		S_IRUGO | S_IWUSR,
		p7_temp_show_t_alarm_high, p7_temp_set_t_alarm_high, 0);
static IIO_DEVICE_ATTR(t_alarm_low,
		S_IRUGO | S_IWUSR,
		p7_temp_show_t_alarm_low, p7_temp_set_t_alarm_low, 0);

static struct attribute *p7_temp_event_int_attributes[] = {
	&iio_dev_attr_t_alarm_high.dev_attr.attr,
	&iio_dev_attr_t_alarm_low.dev_attr.attr,
	NULL,
};

static struct attribute_group p7_temp_event_attribute_group = {
	.attrs = p7_temp_event_int_attributes,
	.name = "events",
};

static const struct iio_info p7_temperature_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &p7_temperature_read_raw,
	.event_attrs = &p7_temp_event_attribute_group,
};

static void poll_temperature(unsigned long data)
{
	struct iio_dev *indio_dev = (struct iio_dev *)data;
	struct p7_temp_state *st = iio_priv(indio_dev);
	s64 timestamp = iio_get_time_ns(indio_dev);

	iio_push_event(indio_dev,
			IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
				IIO_EV_TYPE_THRESH,
				IIO_EV_DIR_RISING),
			timestamp);

	mod_timer(&st->poll_timer, jiffies + (10*HZ));
}

static int p7temp_alloc_channels(struct iio_dev *indio_dev,
		struct p7_temp_chan *channels_info)
{
	struct p7_temp_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *channels;
	struct iio_chan_spec *ch_templ;
	unsigned int i, err;

	err = p7_adc_iio_chip_get_channel_template(st->type,
			&ch_templ);
	if (err)
		return err;

	channels = kcalloc(st->num_channels,
			sizeof(struct iio_chan_spec), GFP_KERNEL);

	if (!channels)
		return -ENOMEM;

	for (i = 0; i < st->num_channels; ++i) {
		channels[i] = *ch_templ;
		channels[i].channel = channels_info[i].channel;
		channels[i].address = channels_info[i].channel;
		channels[i].scan_index = channels_info[i].channel;
		channels[i].extend_name = (char *)channels_info[i].name;

		st->ops->config(channels_info[i].channel,
				channels_info[i].freq);
	}

	indio_dev->channels = channels;
	indio_dev->num_channels = st->num_channels;

	return 0;
}

static int __devinit p7_temp_probe(struct platform_device* pdev)
{
	int ret;
	u16 tmp;
	struct p7_temp_state *st;
	struct iio_dev *indio_dev;
	const struct p7_temp_chan_data *pdata = dev_get_platdata(&pdev->dev);

	if (!pdata)
		return -ENODEV;
	if (pdata->num_channels != 2)
		return -ENODEV;

	ret = p7_adc_get_iio_chip(P7MUADC_IIO_TEMP);
	if (ret) {
		  dev_err(&pdev->dev, "%s():p7_adc_get_iio_chip returns %d\n", __func__, ret );
		  return ret;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		goto put_chip;

	st = iio_priv(indio_dev);
	dev_set_drvdata(&pdev->dev, indio_dev);
	p7_adc_iio_chip_get_channel_ops(P7MUADC_IIO_TEMP, &st->ops);
	st->type = P7MUADC_IIO_TEMP;
	st->gpio_time = GPIO_TIME;
	st->num_channels = pdata->num_channels;

/*************** Read P7MU OTP values *****************/
	// P7MU temperature calibration
	p7mu_read16(P7MU_OTP_TS_POINT_0, &tmp);
	st->ts_temp_0 = ((u8)(tmp & 0x7F) * 185 / 127) - 50;
	st->ts_adc_read_temp_0 = (((s16)(tmp & 0xFF10) >> 7) * 8);
	st->ts_adc_read_temp_0 += (u8)(tmp & 0x7F) * 4095 / 127;
	dev_dbg(&indio_dev->dev, "ts0 0x%x temp_0=%i adc_read_temp0=%i\n", tmp, st->ts_temp_0, st->ts_adc_read_temp_0);

	p7mu_read16(P7MU_OTP_TS_POINT_1, &tmp);
	st->ts_temp_1 = ((u8)(tmp & 0x7F) * 185 / 127) - 50;
	st->ts_adc_read_temp_1 = (((s16)(tmp & 0xFF10) >> 7) * 8);
	st->ts_adc_read_temp_1 += (u8)(tmp & 0x7F) * 4095 / 127;
	dev_dbg(&indio_dev->dev, "ts1 0x%x temp_1=%i adc_read_temp1=%i\n", tmp, st->ts_temp_1, st->ts_adc_read_temp_1);

	// P7 temperature calibration
	p7mu_read16(P7MU_OTP_INTERRUPT_EN_1, &tmp);
	st->offset_0v5_0 = (s8)(tmp >> 8);
	dev_dbg(&indio_dev->dev, "offset_0v5_0=%i\n", st->offset_0v5_0);

	p7mu_read16(P7MU_OTP_TEMP_ERR_TIME, &tmp);
	st->offset_2v7_0 = (s8)(tmp >> 8);
	dev_dbg(&indio_dev->dev, "offset_2v7_0=%i\n", st->offset_2v7_0);

	p7mu_read16(P7MU_OTP_IC_SETUP, &tmp);
	st->offset_0v5_1 = (s8)(tmp >> 8);
	dev_dbg(&indio_dev->dev, "offset_0v5_1=%i\n", st->offset_0v5_1);

	p7mu_read16(P7MU_OTP_CLK_GEN_BCKP_CFG, &tmp);
	st->offset_2v7_1 = (s8)(tmp >> 8);
	dev_dbg(&indio_dev->dev, "offset_2v7_1=%i\n", st->offset_2v7_1);

	if(st->ts_adc_read_temp_1 - st->ts_adc_read_temp_0 == 0)
	{
		st->calibration_valid = 0;
		dev_err(&pdev->dev, "P7 temperature is not calibrated\n");
	}
	else
		st->calibration_valid = 1;

	ret = gpio_request_one(GPIO_V1,
			       GPIOF_OUT_INIT_LOW,
			       PINCTRL_STATE_VOLTAGE1);
	if (ret) {
		dev_err(&pdev->dev, "failed to request GPIO-%d %s\n",
			GPIO_V1, PINCTRL_STATE_VOLTAGE1);
		goto free_device;
	}

	ret = gpio_request_one(GPIO_V2,
			       GPIOF_OUT_INIT_LOW,
			       PINCTRL_STATE_VOLTAGE2);
	if (ret) {
		dev_err(&pdev->dev, "failed to request GPIO-%d %s\n",
			GPIO_V2, PINCTRL_STATE_VOLTAGE2);
		goto free_gpio_v1;
	}

	ret = p7temp_alloc_channels(indio_dev,
			pdata->channels);
	if (ret)
		goto free_gpio_v2;

	/* Schedule this module to run every 10 seconds */
	init_timer(&st->poll_timer);
	st->poll_timer.expires = jiffies + (10*HZ);
	st->poll_timer.function = poll_temperature;
	st->poll_timer.data = (u_long)indio_dev;
	add_timer(&st->poll_timer);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &p7_temperature_info;
	indio_dev->name = P7_TEMP_DRV_NAME;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		goto free_channels;

	dev_info(&pdev->dev, "temperature sensor and ADC registered.\n");

	return 0;

free_channels:
	kfree(indio_dev->channels);
free_gpio_v2:
	gpio_free(GPIO_V2);
free_gpio_v1:
	gpio_free(GPIO_V1);
free_device:
	devm_iio_device_free(&pdev->dev, indio_dev);
put_chip:
	p7_adc_put_iio_chip(P7MUADC_IIO_TEMP);

	return ret;
}

static int __devexit p7_temp_remove(struct platform_device* pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);
	struct p7_temp_state *st = iio_priv(indio_dev);

	p7_adc_put_iio_chip(st->type);
	devm_iio_device_unregister(&pdev->dev, indio_dev);
	dev_set_drvdata(&pdev->dev, NULL);

	del_timer(&st->poll_timer);
	gpio_free(GPIO_V1);
	gpio_free(GPIO_V2);
	kfree(indio_dev->channels);
	devm_iio_device_free(&pdev->dev, indio_dev);

	return 0;
}

static struct platform_driver p7_temp_driver = {
	.driver = {
		.name   = P7_TEMP_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = p7_temp_probe,
	.remove = __devexit_p(p7_temp_remove),
};

module_platform_driver(p7_temp_driver);

MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_DESCRIPTION("P7 Temperature driver");
MODULE_LICENSE("GPL");
