/**
********************************************************************************
* @file p7tempgen.c
* @brief P7 Temperature driver
*
* Copyright (C) 2013 Parrot S.A.
*
* @author Karl Leplat <karl.leplat@parrot.com>
* @date 2013-09-19
********************************************************************************
*/

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/pinctrl/consumer.h>

#include <asm/io.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>

#include "p7-adc_regs.h"
#include "p7-adc_core.h"

#define TEMP_DRV_NAME "fc7100-temperature"

struct temp_gen_state
{
	unsigned int			 type;
	int                              num_channels;
	int                              gpio_time;
	struct timer_list                poll_timer;
	struct p7_adc_iio_chip_channel_ops *ops;
};

/********* HW04 ************/
#define  TEMP_NUM_ROUND         16
#define  TEMP_NUM_ROUND_SHIFT   4

/*
 * parameters of the cubic equation a.x^3 + b.x^2 + c.x + d
 */

/* a = 3.65938.10^-9 ~> 4024 * 2^-40 */
#define P7MU_TEMP_A       4024
#define P7MU_TEMP_A_SHIFT 40

/* b = -2.19291.10^-5 ~> -5887 * 2^-28 */
#define P7MU_TEMP_B       -5887
#define P7MU_TEMP_B_SHIFT 28

/* c = 6.544126.10^-2 ~> 4289 * 2^-16 */
#define P7MU_TEMP_C       4289
#define P7MU_TEMP_C_SHIFT 16

/* d = -45.27694 ~> -11591 * 2^-8 */
#define P7MU_TEMP_D       -11591
#define P7MU_TEMP_D_SHIFT 8

/* Since the value of the adc has to be cubed we downscale it a bit
   before the multiplications */
#define P7MU_TEMP_ADC_SHIFT 3

/* We keep 8 bit of fractional precision before the final sum */
#define P7MU_TEMP_SUM_SHIFT 8

static inline int p7_get_temp_hw04(int adc_val)
{
	int x = adc_val >> P7MU_TEMP_ADC_SHIFT;
	int x_2 = x * x;
	int x_3 = x * x_2;

	/* We shift x_3 a bit so that the multiplication with a doesn't overflow */
	int a = ((x_3 >> 10) * P7MU_TEMP_A)
		>> (P7MU_TEMP_A_SHIFT - P7MU_TEMP_ADC_SHIFT * 3
				- P7MU_TEMP_SUM_SHIFT - 10);

	int b = ((x_2 >> 7) * P7MU_TEMP_B)
		>> (P7MU_TEMP_B_SHIFT - P7MU_TEMP_ADC_SHIFT * 2
				- P7MU_TEMP_SUM_SHIFT - 7);

	int c = (x * P7MU_TEMP_C)
		>> (P7MU_TEMP_C_SHIFT - P7MU_TEMP_ADC_SHIFT * 1
				- P7MU_TEMP_SUM_SHIFT);

	int d = P7MU_TEMP_D >> (P7MU_TEMP_D_SHIFT - P7MU_TEMP_SUM_SHIFT);


	return (a + b + c + d) >> P7MU_TEMP_SUM_SHIFT;
}


static int p7_temperature_read_raw_hw04(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long m)
{
	int ret;
	int v1, round, sum;
	u16 data;
	struct temp_gen_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	switch (m) {
		case IIO_CHAN_INFO_RAW: /* Reading the current channel */

			switch (chan->type) {
				case IIO_TEMP:
					sum = 0;
					for (round = 0; round < TEMP_NUM_ROUND; round++) {
						ret = st->ops->read(chan->address,
								&data);
						if (ret) {
							dev_err(&indio_dev->dev, "v1 ADC read not valid. Got 0x%x\n", data);
							goto out;
						}
						v1 = data >> chan->scan_type.shift;
						v1 &= (1 << chan->scan_type.realbits) - 1;

						sum += v1;
					}

					*val = p7_get_temp_hw04(sum >> TEMP_NUM_ROUND_SHIFT);
					ret = IIO_VAL_INT;
					break;
				default:
					ret = -EINVAL;
					break;
			}
			break;
		default:
			ret = -EINVAL;
			break;

	}
out:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

/********** HW08 *************/
typedef struct ntc_characteristics_t {
	int temp;
	int r_ctn;
} ntc_characteristics_t;

# define RCTN_KOHM(val)	(val * 1000)
static const ntc_characteristics_t ntc[] = {
	{ -40,	RCTN_KOHM(1227.2628) },
	{ -35,	RCTN_KOHM(874.4491) },
	{ -30,	RCTN_KOHM(630.8514) },
	{ -25,	RCTN_KOHM(460.4568) },
	{ -20,	RCTN_KOHM(339.7972) },
	{ -15,	RCTN_KOHM(253.3626) },
	{ -10,	RCTN_KOHM(190.7661) },
	{ -5,	RCTN_KOHM(144.9635) },
	{ 0,	RCTN_KOHM(111.0867) },
	{ 5,	RCTN_KOHM(85.8417) },
	{ 10,	RCTN_KOHM(66.8613) },
	{ 15,	RCTN_KOHM(52.4701) },
	{ 20,	RCTN_KOHM(41.4709) },
	{ 25,	RCTN_KOHM(33) },
	{ 30,	RCTN_KOHM(26.4303) },
	{ 35,	RCTN_KOHM(21.2983) },
	{ 40,	RCTN_KOHM(17.2658) },
	{ 45,	RCTN_KOHM(14.0761) },
	{ 50,	RCTN_KOHM(11.5377) },
	{ 55,	RCTN_KOHM(9.5058) },
	{ 60,	RCTN_KOHM(7.8702) },
	{ 65,	RCTN_KOHM(6.5494) },
	{ 70,	RCTN_KOHM(5.4751) },
	{ 75,	RCTN_KOHM(4.595) },
	{ 80,	RCTN_KOHM(3.8742) },
	{ 85,	RCTN_KOHM(3.2815) },
	{ 90,	RCTN_KOHM(2.7887) },
	{ 95,	RCTN_KOHM(2.3787) },
	{ 100,	RCTN_KOHM(2.0375) },
	{ 105,	RCTN_KOHM(1.7513) },
	{ 110,	RCTN_KOHM(1.5093) },
	{ 115,	RCTN_KOHM(1.3058) },
	{ 120,	RCTN_KOHM(1.1341) },
	{ 125,	RCTN_KOHM(0.9872) },
};

static inline int get_index_min(int rctn)
{
	int i;
	for (i = ARRAY_SIZE(ntc) - 1; i >= 0; i--)
		if ( ntc[i].r_ctn >= rctn)
			return i;
	return 0;
}

#define R1 2400
static inline int p7_get_temp_hw08(int v1, int v2)
{
	int r_ctn = ((R1 * 2 * v2) / v1) - R1;
	int min, max;
	if (!v1)
		v1 = 1;

	r_ctn = ((R1 * 2 * v2) / v1) - R1;

	min = get_index_min(r_ctn);
	if (min == ARRAY_SIZE(ntc) - 1)
		return ntc[min].temp;

	max = min + 1;
	/* linerar interpolation with data from table */
	return ntc[min].temp +
		((ntc[max].temp - ntc[min].temp) * (ntc[min].r_ctn - r_ctn)) /
		(ntc[min].r_ctn - ntc[max].r_ctn);
}

static int p7_temperature_read_raw_hw08(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long m)
{
	int ret;
	int v1, v2;
	u16 data;
	struct temp_gen_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	switch (m) {
	  case IIO_CHAN_INFO_RAW: /* Reading the current channel */

			switch (chan->type) {
				case IIO_TEMP:
					ret = st->ops->read(chan->address,
							&data);
					if (ret) {
						dev_err(&indio_dev->dev, "v1 ADC read not valid. Got 0x%x\n", data);
						goto out;
					}
					v1 = data >> chan->scan_type.shift;
					v1 &= (1 << chan->scan_type.realbits) - 1;

					/* ref is requested, return direct value from ADC */
					if (chan->address == chan->channel2) {
						*val = v1;
						ret = IIO_VAL_INT;
						break;
					}

					ret = st->ops->read(chan->channel2,
							&data);
					if (ret) {
						dev_err(&indio_dev->dev, "v2 ADC read not valid. Got 0x%x\n", data);
						goto out;
					}
					v2 = data >> chan->scan_type.shift;
					v2 &= (1 << chan->scan_type.realbits) - 1;

					*val = p7_get_temp_hw08(v1, v2);
					ret = IIO_VAL_INT;
					break;
				default:
					ret = -EINVAL;
					break;
			}
			break;
		default:
			ret = -EINVAL;
			break;

	}
out:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int p7_temperature_read_raw_sicilia(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long m)
{
	int ret;
	int v1;
	u16 data;
	struct temp_gen_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);

	switch (m) {
		case IIO_CHAN_INFO_RAW: /* Reading the current channel */

			switch (chan->type) {
				case IIO_TEMP:
					ret = st->ops->read(chan->address,
							&data);
					if (ret) {
						dev_err(&indio_dev->dev, "v1 ADC read not valid. Got 0x%x\n", data);
						goto out;
					}
					v1 = data >> chan->scan_type.shift;
					v1 &= (1 << chan->scan_type.realbits) - 1;

					/* No ADC on the 3.3V reference, use a
					 * hardcoded value of 4096 */
					*val = p7_get_temp_hw08(v1, 4096);
					ret = IIO_VAL_INT;
					break;
				default:
					ret = -EINVAL;
					break;
			}
			break;
		default:
			ret = -EINVAL;
			break;

	}
out:
	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static ssize_t p7mu_adc_show_t_alarm_high(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return 0;
}

static inline ssize_t p7mu_adc_set_t_alarm_high(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return 0;
}

static ssize_t p7mu_adc_show_t_alarm_low(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return 0;
}

static inline ssize_t p7mu_adc_set_t_alarm_low(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return 0;
}

static IIO_DEVICE_ATTR(t_alarm_high,
		S_IRUGO | S_IWUSR,
		p7mu_adc_show_t_alarm_high, p7mu_adc_set_t_alarm_high, 0);
static IIO_DEVICE_ATTR(t_alarm_low,
		S_IRUGO | S_IWUSR,
		p7mu_adc_show_t_alarm_low, p7mu_adc_set_t_alarm_low, 0);

static struct attribute *p7mu_adc_event_int_attributes[] = {
	&iio_dev_attr_t_alarm_high.dev_attr.attr,
	&iio_dev_attr_t_alarm_low.dev_attr.attr,
	NULL,
};

static struct attribute_group p7mu_adc_event_attribute_group = {
	.attrs = p7mu_adc_event_int_attributes,
	.name = "events",
};

static struct iio_info p7_temperature_info = {
	.driver_module = THIS_MODULE,
	.event_attrs = &p7mu_adc_event_attribute_group,
};

static void poll_temperature(unsigned long data)
{
	struct iio_dev *indio_dev = (struct iio_dev *)data;
	struct temp_gen_state *st = iio_priv(indio_dev);
	s64 timestamp = iio_get_time_ns(indio_dev);

	iio_push_event(indio_dev,
			IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
				IIO_EV_TYPE_THRESH,
				IIO_EV_DIR_RISING),
			timestamp);

	mod_timer(&st->poll_timer, jiffies + (10*HZ));
}

static int p7tempgen_alloc_channels(struct iio_dev *indio_dev,
		struct p7_temp_chan *channels_info, int channel2)
{
	struct temp_gen_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *channels;
	struct iio_chan_spec *ch_templ;
	unsigned int i, err;

	err = p7_adc_iio_chip_get_channel_template(st->type, &ch_templ);
	if (err)
		return err;

	channels = kcalloc(st->num_channels,
			sizeof(struct iio_chan_spec), GFP_KERNEL);

	if (!channels)
		return -ENOMEM;

	for (i = 0; i < st->num_channels; ++i) {
		channels[i] = *ch_templ;
		channels[i].channel = channels_info[i].channel;
		channels[i].channel2 = channel2;
		channels[i].address = channels_info[i].channel;
		channels[i].scan_index = channels_info[i].channel;
		channels[i].extend_name = (char *)channels_info[i].name;

		st->ops->config(channels_info[i].channel,
				channels_info[i].freq);
	}

	indio_dev->channels = channels;

	return 0;
}

static int __devinit temp_gen_probe(struct platform_device* pdev)
{
	int ret;
	struct temp_gen_state *st;
	struct iio_dev *indio_dev;
	struct p7_temp_chan *channels_info;
	int channel2;
	struct p7_temp_chan_data *pdata
		= dev_get_platdata(&pdev->dev);

	if (!pdata)
		return -ENODEV;

	ret = p7_adc_get_iio_chip(P7MUADC_IIO_TEMP);

	if (ret)
		return ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		goto put_chip;
	}

	st = iio_priv(indio_dev);
	dev_set_drvdata(&pdev->dev, indio_dev);
	p7_adc_iio_chip_get_channel_ops(P7MUADC_IIO_TEMP,
			&st->ops);
	st->type = P7MUADC_IIO_TEMP;

	channels_info = pdata->channels;
	if (pdata->temp_mode == P7_TEMP_FC7100_HW08) {
		p7_temperature_info.read_raw = p7_temperature_read_raw_hw08;
		channel2 = channels_info[0].channel;
		st->ops->config(channels_info[0].channel,
			channels_info[0].freq);

		st->num_channels = pdata->num_channels;
	}
	else if (pdata->temp_mode == P7_TEMP_FC7100_HW04) {
		p7_temperature_info.read_raw = p7_temperature_read_raw_hw04;
		channel2 = 0;
		st->num_channels = pdata->num_channels;
	}
	else if (pdata->temp_mode == P7_TEMP_SICILIA) {
		p7_temperature_info.read_raw = p7_temperature_read_raw_sicilia;
		channel2 = 0;
		st->num_channels = pdata->num_channels;
	}
	else {
		goto free_device;
	}

	ret = p7tempgen_alloc_channels(indio_dev,
			channels_info, channel2);
	if(ret)
		goto free_device;

	/* Schedule this module to run every 10 seconds */
	init_timer(&st->poll_timer);
	st->poll_timer.expires = jiffies + (10*HZ);
	st->poll_timer.function = poll_temperature;
	st->poll_timer.data = (u_long)indio_dev;
	add_timer(&st->poll_timer);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->num_channels = st->num_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &p7_temperature_info;
	indio_dev->name = TEMP_DRV_NAME;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		goto free_channels;

	dev_info(&pdev->dev, "%s temperature sensor and ADC registered.\n",
			indio_dev->name);


	return 0;

free_channels:
	kfree(indio_dev->channels);
free_device:
	devm_iio_device_free(&pdev->dev, indio_dev);
put_chip:
	p7_adc_put_iio_chip(P7MUADC_IIO_TEMP);

	return ret;
}

static int __devexit temp_gen_remove(struct platform_device* pdev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(&pdev->dev);
	struct temp_gen_state *st = iio_priv(indio_dev);

	p7_adc_put_iio_chip(st->type);
	devm_iio_device_unregister(&pdev->dev, indio_dev);
	dev_set_drvdata(&pdev->dev, NULL);

	del_timer(&st->poll_timer);
	kfree(indio_dev->channels);
	devm_iio_device_free(&pdev->dev, indio_dev);

	return 0;
}

static struct platform_driver temp_gen_driver = {
	.driver = {
		.name   = TEMP_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = temp_gen_probe,
	.remove = __devexit_p(temp_gen_remove),
};

module_platform_driver(temp_gen_driver);

MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_DESCRIPTION("Temperature generic driver");
MODULE_LICENSE("GPL");
