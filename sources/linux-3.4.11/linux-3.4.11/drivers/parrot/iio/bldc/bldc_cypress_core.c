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
#ifdef CONFIG_PARROT_BLDC_CYPRESS_NOTRIGGER
#include <linux/iio/kfifo_buf.h>
#endif

#include <linux/iio/bldc/parrot_bldc_cypress_iio.h>
#include <linux/iio/bldc/parrot_bldc_iio.h>
#include <iio/platform_data/bldc_cypress.h>

#include <iio/platform_data/mykonos3.h>

static const struct iio_chan_spec bldc_cypress_channels[] = {
	/* keep 4th first motors channels at first table place */
	PARROT_BLDC_OBS_CHAN         (SPEED_MOTOR1, 0, IIO_ANGL_VEL, 16, 16),
	PARROT_BLDC_OBS_CHAN         (SPEED_MOTOR2, 1, IIO_ANGL_VEL, 16, 16),
	PARROT_BLDC_OBS_CHAN         (SPEED_MOTOR3, 2, IIO_ANGL_VEL, 16, 16),
	PARROT_BLDC_OBS_CHAN         (SPEED_MOTOR4, 3, IIO_ANGL_VEL, 16, 16),
	PARROT_BLDC_OBS_CHAN         (BATT_VOLTAGE, 0, IIO_VOLTAGE, 16, 16),
	PARROT_BLDC_OBS_CHAN_EXT_NAME(STATUS,       0, IIO_CURRENT, 8, 8),
	PARROT_BLDC_OBS_CHAN_EXT_NAME(ERRNO,        1, IIO_CURRENT, 8, 8),
	PARROT_BLDC_OBS_CHAN_EXT_NAME(FAULT_MOTOR,  2, IIO_CURRENT, 4, 8),
	PARROT_BLDC_OBS_CHAN         (TEMP,         0, IIO_TEMP, 8, 8),
	IIO_CHAN_SOFT_TIMESTAMP(PARROT_BLDC_SCAN_TIMESTAMP),
};

static ssize_t bldc_cypress_store_clear_error(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	int ret;

	ret = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_CLEAR_ERROR, 0, NULL);

	return ret == 0 ? count : ret;
}

static ssize_t bldc_cypress_store_led(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	u8 data;
	int ret;

	/**
	 * data is a bitmask :
	 *  b0: Red LED on/off
	 *  b1: Green LED on/off
	 *  b2: linux reboot strategy: reboot/power off
	 */
	ret = kstrtou8(buf, 10, &data);
	if (ret)
		return -EINVAL;

	data &= 0x3; /* Set P7 reboot gpio bit to zero */
	data |= 0x4; /* Keep psoc_on */

	ret = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_LED, 1, &data);

	return ret == 0 ? count : ret;
}

static ssize_t bldc_cypress_store_motors_speed(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	int ret, i;
	u8 data[10];
	u8 crc;
	unsigned short m[4];
	unsigned short *pm = (__be16 *)&data[0];

	ret = sscanf(buf, "%hu %hu %hu %hu",
			&m[0],
			&m[1],
			&m[2],
			&m[3]);

	if (ret != 4)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		*pm++ = cpu_to_be16(m[st->pdata.lut[i]]);
	}

	data[8] = 0; /* force enable security to 0 */

	crc = PARROT_BLDC_REG_REF_SPEED;
	for (i = 0; i < 9; i++)
		crc ^= data[i];
	data[9] = crc;

	ret = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_REF_SPEED, 10, data);

	return ret == 0 ? count : ret;
}

static ssize_t bldc_cypress_start_with_dir(struct bldc_state *st, u8 spin_dir)
{
	int ret, i;
	u8 data;

	/* apply lut on spin direction */
	data = 0;
	for (i = 0; i < 4; i++) {
		if (spin_dir & (1 << st->pdata.lut[i]))
			data |= 1 << i;
	}

	ret = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_START_PROP, 1, &data);

	return ret;
}

static ssize_t bldc_cypress_store_start(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	int ret;

	/* use spin direction given in platform data */
	ret = bldc_cypress_start_with_dir(st, st->pdata.spin_dir);
	return ret == 0 ? count : ret;
}

static ssize_t bldc_cypress_store_start_with_dir(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	u8 spin_dir;
	int ret;

	ret = kstrtou8(buf, 10, &spin_dir);
	if (ret)
		return -EINVAL;

	/* use spin direction given in argument */
	ret = bldc_cypress_start_with_dir(st, spin_dir);
	return ret == 0 ? count : ret;
}

static ssize_t bldc_cypress_store_stop(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	int ret;

	ret = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_STOP_PROP, 0, NULL);

	return ret == 0 ? count : ret;
}

static ssize_t bldc_cypress_show_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result;
	__u8 *ibuf;
	unsigned short val16;
	unsigned int val32;

	ibuf = kzalloc(PARROT_BLDC_GET_INFO_LENGTH, GFP_KERNEL);

	result = st->tf->read_multiple_byte(st->dev,
			PARROT_BLDC_REG_INFO,
			PARROT_BLDC_GET_INFO_LENGTH, ibuf);

	if (result != PARROT_BLDC_GET_INFO_LENGTH) {
		result = -EINVAL;
		goto free_buf;
	}

	switch ((u32)this_attr->address) {
	case ATTR_INFO_SOFT_VERSION:
		sprintf(buf, "%d %d %c %d\n", ibuf[0],
				ibuf[1],
				ibuf[2],
				ibuf[3]);
		break;
	case ATTR_INFO_FT_NB_FLIGHTS:
		val16 = ibuf[4] << 8 | ibuf[5];
		sprintf(buf, "%hu\n", val16);
		break;
	case ATTR_INFO_FT_PREVIOUS_TIME:
		val16 = ibuf[6] << 8 | ibuf[7];
		sprintf(buf, "%hu\n", val16);
		break;
	case ATTR_INFO_FT_TOTAL_TIME:
		val32 = ibuf[8] << 24 | ibuf[9] << 16 | ibuf[10] << 8 | ibuf[11];
		sprintf(buf, "%u\n", val32);
		break;
	case ATTR_INFO_FT_LAST_ERROR:
		sprintf(buf, "%d\n", ibuf[12]);
		break;
	default:
		result = -ENODEV;
	}

free_buf:
	kfree(ibuf);

	return result;
}

static ssize_t bldc_cypress_show_cached_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result;
	unsigned short nb_flights;
	unsigned short previous_time;
	unsigned int total_time;
	int is_updated;

	switch ((u32)this_attr->address) {
	case ATTR_INFO_FT_ALL:
		is_updated = (st->fd_state == PARROT_BLDC_FD_UPDATED);
		nb_flights = st->cache[4] << 8 | st->cache[5];
		previous_time = st->cache[6] << 8 | st->cache[7];
		total_time = st->cache[8] << 24 |
				st->cache[9] << 16 |
				st->cache[10] << 8 |
				st->cache[11];
		result = sprintf(buf, "%d %d %d %c %d, %hu, %hu, %u, %d\n",
				is_updated,
				st->cache[0],
				st->cache[1],
				st->cache[2],
				st->cache[3],
				nb_flights,
				previous_time,
				total_time,
				st->cache[12]);
		if (is_updated)
			st->fd_state = PARROT_BLDC_FD_OLD;
		break;
	default:
		result = -ENODEV;
	}

	return result;
}

static ssize_t bldc_update_flight_info(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);

	if (!st)
		return -ENOENT;

	st->fd_state = PARROT_BLDC_FD_UPDATE_REQ;

	return count;
}

static ssize_t bldc_cypress_show_spin_dir(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bldc_state *st = iio_priv(indio_dev);

	if (!st)
		return -ENOENT;

	return sprintf(buf, "%d\n", st->pdata.spin_dir);
}

static IIO_DEVICE_ATTR(start,
		       S_IWUSR,
		       NULL,
		       bldc_cypress_store_start,
		       ATTR_START_MOTORS);

static IIO_DEVICE_ATTR(start_with_dir,
		       S_IWUSR,
		       NULL,
		       bldc_cypress_store_start_with_dir,
		       ATTR_START_MOTORS_WITH_DIR);

static IIO_DEVICE_ATTR(stop,
		       S_IWUSR,
		       NULL,
		       bldc_cypress_store_stop,
		       ATTR_STOP_MOTORS);

static IIO_DEVICE_ATTR(led,
		       S_IWUSR,
		       NULL,
		       bldc_cypress_store_led,
		       ATTR_LED);

static IIO_DEVICE_ATTR(clear_error,
		       S_IWUSR,
		       NULL,
		       bldc_cypress_store_clear_error,
		       ATTR_CLEAR_ERROR);

static IIO_DEVICE_ATTR(motors_speed,
		       S_IWUSR,
		       NULL,
		       bldc_cypress_store_motors_speed,
		       ATTR_MOTORS_SPEED);

static IIO_DEVICE_ATTR(soft_version,
		       S_IRUGO,
		       bldc_cypress_show_info,
		       NULL,
		       ATTR_INFO_SOFT_VERSION);

static IIO_DEVICE_ATTR(nb_flights,
		       S_IRUGO,
		       bldc_cypress_show_info,
		       NULL,
		       ATTR_INFO_FT_NB_FLIGHTS);

static IIO_DEVICE_ATTR(previous_flight_time,
		       S_IRUGO,
		       bldc_cypress_show_info,
		       NULL,
		       ATTR_INFO_FT_PREVIOUS_TIME);

static IIO_DEVICE_ATTR(total_flight_time,
		       S_IRUGO,
		       bldc_cypress_show_info,
		       NULL,
		       ATTR_INFO_FT_TOTAL_TIME);

static IIO_DEVICE_ATTR(last_flight_error,
		       S_IRUGO,
		       bldc_cypress_show_info,
		       NULL,
		       ATTR_INFO_FT_LAST_ERROR);

static IIO_DEVICE_ATTR(flight_all_infos,
		       S_IRUGO,
		       bldc_cypress_show_cached_info,
		       NULL,
		       ATTR_INFO_FT_ALL);

static IIO_DEVICE_ATTR(update_flight_info,
		       S_IWUGO,
		       NULL,
		       bldc_update_flight_info,
		       ATTR_INFO_FT_UPDATE);

static IIO_DEVICE_ATTR(spin_dir,
		       S_IRUGO,
		       bldc_cypress_show_spin_dir,
		       NULL,
		       ATTR_INFO_SPIN_DIR);

static struct attribute *inv_attributes[] = {
	&iio_dev_attr_start.dev_attr.attr,
	&iio_dev_attr_start_with_dir.dev_attr.attr,
	&iio_dev_attr_stop.dev_attr.attr,
	&iio_dev_attr_led.dev_attr.attr,
	&iio_dev_attr_clear_error.dev_attr.attr,
	&iio_dev_attr_motors_speed.dev_attr.attr,
	&iio_dev_attr_soft_version.dev_attr.attr,
	&iio_dev_attr_nb_flights.dev_attr.attr,
	&iio_dev_attr_previous_flight_time.dev_attr.attr,
	&iio_dev_attr_total_flight_time.dev_attr.attr,
	&iio_dev_attr_last_flight_error.dev_attr.attr,
	&iio_dev_attr_flight_all_infos.dev_attr.attr,
	&iio_dev_attr_update_flight_info.dev_attr.attr,
	&iio_dev_attr_spin_dir.dev_attr.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.attrs = inv_attributes
};

static int bldc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct bldc_state *st = iio_priv(indio_dev);

	kfree(st->buffer);

	st->buffer = kmalloc(indio_dev->scan_bytes +
			PARROT_BLDC_OBS_DATA_LENGTH,
			GFP_KERNEL);
	if (st->buffer == NULL)
		return -ENOMEM;

	return 0;
}

static const struct iio_info bldc_cypress_info = {
	.update_scan_mode	= bldc_update_scan_mode,
	.driver_module		= THIS_MODULE,
	.attrs			= &inv_attribute_group,
};

#ifdef CONFIG_PARROT_BLDC_CYPRESS_NOTRIGGER
static const struct iio_buffer_setup_ops bldc_cypress_iio_buffer_setup_ops = {
};

struct iio_mykonos3_ops bldc_cypress_mykonos3_ops = {
	.request_data = NULL,
	.fetch_data   = bldc_cypress_fetch_data,
};
#endif

static int bldc_cypress_iio_buffer_new(struct iio_dev *indio_dev)
{
	int err = 0;

#ifdef CONFIG_PARROT_BLDC_CYPRESS_NOTRIGGER
	struct iio_buffer *buffer = NULL;

	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &bldc_cypress_iio_buffer_setup_ops;

	buffer = iio_kfifo_allocate(indio_dev);
	if (!buffer) {
		err = -ENOMEM;
		goto exit;
	}

	iio_device_attach_buffer(indio_dev, buffer);
	err = iio_buffer_register(indio_dev, bldc_cypress_channels,
				  ARRAY_SIZE(bldc_cypress_channels));
	if (err < 0)
		goto exit;

	iio_mykonos3_register(IIO_MYKONOS3_MOTORS,
			      indio_dev, &bldc_cypress_mykonos3_ops);
#else
	struct bldc_state *st = iio_device_get_drvdata(indio_dev);

	indio_dev->modes = INDIO_BUFFER_TRIGGERED;
	err = iio_triggered_buffer_setup(indio_dev,
					 NULL,
					 bldc_cypress_read_fifo,
					 NULL);
	if (err < 0) {
		dev_err(st->dev, "configure buffer fail %d\n", err);
		goto exit;
	}
#endif
exit:
	return err;
}

static void bldc_cypress_iio_buffer_cleanup(struct iio_dev *indio_dev)
{
#ifdef CONFIG_PARROT_BLDC_CYPRESS_NOTRIGGER
	iio_kfifo_free(indio_dev->buffer);
	iio_mykonos3_unregister(IIO_MYKONOS3_MOTORS, indio_dev);
#else
	iio_triggered_buffer_cleanup(indio_dev);
#endif
	iio_buffer_unregister(indio_dev);
}

int bldc_cypress_probe(struct iio_dev *indio_dev)
{
	struct bldc_state *st;
	int result, i;
	u8 data;

	st = iio_priv(indio_dev);

	mutex_init(&st->mutex);

	/* init flight data cache */
	st->cache = kzalloc(PARROT_BLDC_GET_INFO_LENGTH, GFP_KERNEL);
	if (st->cache == NULL)
		return -ENOMEM;
	st->fd_state = PARROT_BLDC_FD_OLD;

	/* copy default channels */
	memcpy(st->channels, bldc_cypress_channels, sizeof(st->channels));

	/* update first 4th channels scan_index given motor lut */
	for (i = 0; i < 4; i++)
		st->channels[i].scan_index = st->pdata.lut[i];

	indio_dev->channels = st->channels;
	indio_dev->num_channels = ARRAY_SIZE(st->channels);
	indio_dev->info = &bldc_cypress_info;

	result = bldc_cypress_iio_buffer_new(indio_dev);
	if (result < 0)
		goto out_remove_trigger;

	result = devm_iio_device_register(st->dev, indio_dev);
	if (result) {
		dev_err(st->dev, "IIO register fail %d\n", result);
		goto out_remove_trigger;
	}

	/* Set psoc_on */
	data = 0x4;
	result = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_LED, sizeof(data), &data);
	if (result != sizeof(data))
		dev_err(st->dev, "fail to set psoc_on : %d\n", result);

	dev_info(st->dev,
		 "PARROT BLDC (%s) registered\n",
		 indio_dev->name);

	return 0;

out_remove_trigger:
	bldc_cypress_iio_buffer_cleanup(indio_dev);
	return result;
}
EXPORT_SYMBOL(bldc_cypress_probe);

void bldc_cypress_remove(struct iio_dev *indio_dev)
{
	struct bldc_state *st = iio_priv(indio_dev);

	devm_iio_device_unregister(st->dev, indio_dev);
	bldc_cypress_iio_buffer_cleanup(indio_dev);
	kfree(st->buffer);
	kfree(st->cache);
}
EXPORT_SYMBOL(bldc_cypress_remove);

void bldc_cypress_shutdown(struct iio_dev *indio_dev)
{
	struct bldc_state *st = iio_priv(indio_dev);
	u8 data = 0;
	int ret;

	ret = st->tf->write_multiple_byte(st->dev,
			PARROT_BLDC_REG_LED, sizeof(data), &data);
	if (ret != sizeof(data))
		dev_err(st->dev, "shutdown fail %d\n", ret);
}
EXPORT_SYMBOL(bldc_cypress_shutdown);

MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_DESCRIPTION("Parrot BLDC cypress IIO driver");
MODULE_LICENSE("GPL");
