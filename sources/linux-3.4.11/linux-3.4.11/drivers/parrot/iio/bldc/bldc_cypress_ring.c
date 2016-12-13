/**
 ************************************************
 * @file bldc_cypress_ring.c
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
#include <linux/poll.h>

#include <linux/iio/bldc/parrot_bldc_cypress_iio.h>
#include <linux/iio/bldc/parrot_bldc_iio.h>

#ifdef CONFIG_PARROT_BLDC_CYPRESS_NOTRIGGER
int bldc_cypress_fetch_data(struct iio_dev *indio_dev)
{
	const struct iio_chan_spec *chan;
	struct bldc_state *st = iio_priv(indio_dev);
	int result, i, crc;
	u8 *tx, *rx;

	mutex_lock(&st->mutex);

	tx = st->buffer;
	rx = tx + indio_dev->scan_bytes;

	result = st->tf->read_multiple_byte(st->dev,
			PARROT_BLDC_REG_OBS_DATA,
			PARROT_BLDC_OBS_DATA_LENGTH, rx);

	if (result != PARROT_BLDC_OBS_DATA_LENGTH) {
		mutex_unlock(&st->mutex);
		result = -EINVAL;
		goto err_read;
	}

	crc = 0;
	for (i = 0; i < PARROT_BLDC_OBS_DATA_LENGTH - 1; ++i)
		crc ^= *rx++;

	if (crc != *rx) {
		mutex_unlock(&st->mutex);
		result = -EINVAL;
		goto err_read;
	}

	rx = tx + indio_dev->scan_bytes;
	chan = indio_dev->channels;
	for (i = 0; i < indio_dev->num_channels; i++, chan++) {
		if (!test_bit(chan->scan_index, indio_dev->buffer->scan_mask)) {
			if (chan->scan_type.storagebits == 16)
				rx++;
			rx++;
			continue;
		}

		if (chan->scan_type.storagebits == 16) {
			__be16 data = *((__be16 *)rx);
			*tx++ = data & 0xff;
			*tx++ = (data >> 8) & 0xff;
			rx += 2;
		} else {
			if (chan->scan_index == PARROT_BLDC_SCAN_OBS_DATA_FAULT_MOTOR) {
				/* apply lut for fault motors */
				uint8_t j, new_fmot, fmot = *rx++;
				for (new_fmot = 0, j = 0; j < 4; j++, fmot >>= 1)
					new_fmot |= (fmot & 0x01) << st->pdata.lut[j];
				*tx++ = new_fmot;
			} else {
				*tx++ = *rx++;
			}
		}
	}

	mutex_unlock(&st->mutex);

	result = iio_push_to_buffers_with_timestamp(indio_dev,
			st->buffer,
			iio_get_time_ns(indio_dev));
	if (result) {
		/* only warn once in overflow (no iio reader) */
		if (result != -EBUSY || !st->is_overflow) {
			dev_err(st->dev, "IIO push data failed(%d)\n", result);
		}

		/* update overflow flag */
		if (result == -EBUSY && !st->is_overflow)
			st->is_overflow = 1;

	} else if (st->is_overflow) {
		st->is_overflow = 0;
	}

	if (st->fd_state == PARROT_BLDC_FD_UPDATE_REQ) {
		result = st->tf->read_multiple_byte(st->dev,
				PARROT_BLDC_REG_INFO,
				PARROT_BLDC_GET_INFO_LENGTH,
				st->cache);

		if (result == PARROT_BLDC_GET_INFO_LENGTH)
			st->fd_state = PARROT_BLDC_FD_UPDATED;
	}

err_read:
	return result;
}
#endif

/**
 * bldc_cypress_read_fifo() - Transfer data from hardware FIFO to KFIFO.
 */
irqreturn_t bldc_cypress_read_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	const struct iio_chan_spec *chan;
	struct bldc_state *st = iio_priv(indio_dev);
	int result, i, crc;
	u8 *tx, *rx;

	mutex_lock(&st->mutex);

	tx = st->buffer;
	rx = tx + indio_dev->scan_bytes;

	result = st->tf->read_multiple_byte(st->dev,
			PARROT_BLDC_REG_OBS_DATA,
			PARROT_BLDC_OBS_DATA_LENGTH, rx);

	if (result != PARROT_BLDC_OBS_DATA_LENGTH) {
		mutex_unlock(&st->mutex);
		goto err_read;
	}

	crc = 0;
	for (i = 0; i < PARROT_BLDC_OBS_DATA_LENGTH - 1; ++i)
		crc ^= *rx++;

	if (crc != *rx) {
		mutex_unlock(&st->mutex);
		goto err_read;
	}

	rx = tx + indio_dev->scan_bytes;
	chan = indio_dev->channels;
	for (i = 0; i < indio_dev->num_channels; i++, chan++) {
		if (!test_bit(chan->scan_index, indio_dev->buffer->scan_mask)) {
			if (chan->scan_type.storagebits == 16)
				rx++;
			rx++;
			continue;
		}

		if (chan->scan_type.storagebits == 16) {
			__be16 data = *((__be16 *)rx);
			*tx++ = data & 0xff;
			*tx++ = (data >> 8) & 0xff;
			rx += 2;
		} else {
			if (chan->scan_index == PARROT_BLDC_SCAN_OBS_DATA_FAULT_MOTOR) {
				/* apply lut for fault motors */
				uint8_t j, new_fmot, fmot = *rx++;
				for (new_fmot = 0, j = 0; j < 4; j++, fmot >>= 1)
					new_fmot |= (fmot & 0x01) << st->pdata.lut[j];
				*tx++ = new_fmot;
			} else {
				*tx++ = *rx++;
			}
		}
	}

	mutex_unlock(&st->mutex);

	result = iio_push_to_buffers_with_timestamp(indio_dev,
			st->buffer,
			iio_get_time_ns(indio_dev));
	if (result) {
		/* only warn once in overflow (no iio reader) */
		if (result != -EBUSY || !st->is_overflow)
			dev_err(st->dev, "IIO push data failed(%d)\n", result);

		/* update overflow flag */
		if (result == -EBUSY && !st->is_overflow)
			st->is_overflow = 1;

	} else if (st->is_overflow) {
		st->is_overflow = 0;
	}

	if (st->fd_state == PARROT_BLDC_FD_UPDATE_REQ) {
		result = st->tf->read_multiple_byte(st->dev,
				PARROT_BLDC_REG_INFO,
				PARROT_BLDC_GET_INFO_LENGTH,
				st->cache);

		if (result == PARROT_BLDC_GET_INFO_LENGTH)
			st->fd_state = PARROT_BLDC_FD_UPDATED;
	}

err_read:
	/* Flush HW and SW FIFOs. */
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

