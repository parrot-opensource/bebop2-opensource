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
#include "inv_mpu_iio.h"

static void inv_clear_kfifo(struct inv_mpu6050_state *st)
{
	unsigned long flags;

	/* take the spin lock sem to avoid interrupt kick in */
	spin_lock_irqsave(&st->time_stamp_lock, flags);
	kfifo_reset(&st->timestamps);
	spin_unlock_irqrestore(&st->time_stamp_lock, flags);
}

int inv_reset_fifo(struct iio_dev *indio_dev)
{
	int result;
	u8 d;
	struct inv_mpu6050_state  *st = iio_priv(indio_dev);

	/* disable interrupt */
	result = inv_mpu6050_write_reg(st, st->reg->int_enable, 0);
	if (result) {
		dev_err(&st->client->dev, "int_enable failed %d\n", result);
		return result;
	}
	/* disable the sensor output to FIFO */
	result = inv_mpu6050_write_reg(st, st->reg->fifo_en, 0);
	if (result)
		goto reset_fifo_fail;
	/* disable fifo reading */
	result = inv_mpu6050_write_reg(st, st->reg->user_ctrl, 0);
	if (result)
		goto reset_fifo_fail;

	/* reset FIFO*/
	result = inv_mpu6050_write_reg(st, st->reg->user_ctrl,
					INV_MPU6050_BIT_FIFO_RST);
	if (result)
		goto reset_fifo_fail;

	/* clear timestamps fifo */
	inv_clear_kfifo(st);

	/* enable FIFO reading and I2C master interface*/
	result = inv_mpu6050_write_reg(st, st->reg->user_ctrl,
					INV_MPU6050_BIT_FIFO_EN);
	if (result)
		goto reset_fifo_fail;
	/* enable sensor output to FIFO */
	d = 0;
	if (st->chip_config.gyro_fifo_enable)
		d |= INV_MPU6050_BITS_GYRO_OUT;
	if (st->chip_config.accl_fifo_enable)
		d |= INV_MPU6050_BIT_ACCEL_OUT;
	if (st->chip_config.temp_fifo_enable)
		d |= INV_MPU6050_BIT_TEMP_OUT;
	result = inv_mpu6050_write_reg(st, st->reg->fifo_en, d);
	if (result)
		goto reset_fifo_fail;

	/* enable interrupt */
	if (d) {
		result = inv_mpu6050_write_reg(st, st->reg->int_enable,
					INV_MPU6050_BIT_DATA_RDY_EN);
		if (result)
			return result;
	}

	return 0;

reset_fifo_fail:
	dev_err(&st->client->dev, "reset fifo failed %d\n", result);
	result = inv_mpu6050_write_reg(st, st->reg->int_enable,
					INV_MPU6050_BIT_DATA_RDY_EN);

	return result;
}

/**
 * inv_mpu6050_irq_handler() - Cache a timestamp at each data ready interrupt.
 */
irqreturn_t inv_mpu6050_irq_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	s64 timestamp;

	timestamp = ktime_to_ns(ktime_get());
	/* if interrupts are filtered, compensate delay */
	timestamp -= st->chip_config.filter_period_ns;
	kfifo_in_spinlocked(&st->timestamps, &timestamp, 1,
				&st->time_stamp_lock);

	return IRQ_WAKE_THREAD;
}

/**
 * inv_mpu6050_read_samples() - Transfer data from hardware FIFO to a buffer.
 *
 * Samples are stored in st->rx_buffer
 */
static int inv_mpu6050_read_samples(struct inv_mpu6050_state *st,
				    u16 fifo_count)
{
	u16 remaining_size;
	u8 *dest;
	int result;

	/* Grow buffer if required */
	if (st->rx_buffer_size < fifo_count) {
		st->rx_buffer = krealloc(st->rx_buffer, fifo_count, GFP_KERNEL);
		if (!st->rx_buffer) {
			st->rx_buffer_size = 0;
			return -ENOMEM;
		}

		st->rx_buffer_size = fifo_count;
	}

	dest = st->rx_buffer;
	remaining_size = fifo_count;
	while (remaining_size > 0) {
		result = i2c_smbus_read_i2c_block_data(st->client,
						       st->reg->fifo_r_w,
						       remaining_size, dest);
		if (result < 0)
			return result;

		dest += result;
		remaining_size -= result;
	}

	return fifo_count;
}

/**
 * inv_mpu6050_create_timestamped_buffer() - create buffer with timestamps
 *
 * Samples are stored in st->rx_buffer_ts
 */
static int inv_mpu6050_create_timestamped_buffer(struct inv_mpu6050_state *st,
						 size_t bytes_per_datum,
						 u16 samples)
{
	s64 timestamp = 0;
	u8 *src;
	u8 *dest;
	size_t ts_padding;
	size_t buff_size;
	size_t i;
	int result;

	/* Compute required buffer size */
	ts_padding = sizeof(s64) - (bytes_per_datum % sizeof(s64));
	buff_size = (bytes_per_datum + ts_padding + sizeof(s64)) * samples;

	/* Resize buffer if required */
	if (buff_size > st->rx_buffer_ts_size) {
		st->rx_buffer_ts = krealloc(st->rx_buffer_ts, buff_size,
					    GFP_KERNEL);
		if (!st->rx_buffer_ts) {
			st->rx_buffer_size = 0;
			return -ENOMEM;
		}

		st->rx_buffer_size = buff_size;
	}

	dest = st->rx_buffer_ts;
	src = st->rx_buffer;

	for (i = 0; i < samples; i++) {
		if (i == 0) {
			result = kfifo_out(&st->timestamps, &timestamp, 1);
			if (result == 0)
				dev_err(&st->client->dev, "no timestamp\n");
		} else {
			/* estimate timestamp
			 * by adding fifo fifo period
			 * this is the case
			 * when interrupts are filtered at source
			 */
			timestamp += st->chip_config.fifo_period_ns;
		}

		/* Store sample as-is, no padding required */
		memcpy(dest, src, bytes_per_datum);
		src += bytes_per_datum;
		dest += bytes_per_datum;

		/* Store timestamp */
		memcpy(dest + ts_padding, &timestamp, sizeof(s64));
		dest += ts_padding + sizeof(s64);
	}

	return 0;
}

/**
 * inv_mpu6050_read_fifo() - Transfer data from hardware FIFO to KFIFO.
 */
irqreturn_t inv_mpu6050_read_fifo(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	size_t bytes_per_datum;
	size_t samples;
	int result;
	u16 raw_fifo_count;
	u16 fifo_count;

	mutex_lock(&indio_dev->mlock);
	if (!(st->chip_config.accl_fifo_enable |
		st->chip_config.gyro_fifo_enable |
		st->chip_config.temp_fifo_enable))
		goto end_session;

	/* Calculate IMU sample size (depends of enabled channels) */
	bytes_per_datum = 0;
	if (st->chip_config.accl_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_PER_3AXIS_SENSOR;

	if (st->chip_config.gyro_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_PER_3AXIS_SENSOR;

	if (st->chip_config.temp_fifo_enable)
		bytes_per_datum += INV_MPU6050_BYTES_TEMPERATURE;

	/* Fetch FIFO size */
	result = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->fifo_count_h,
				       INV_MPU6050_FIFO_COUNT_BYTE,
				       (u8 *) &raw_fifo_count);
	if (result != INV_MPU6050_FIFO_COUNT_BYTE)
		goto end_session;

	fifo_count = be16_to_cpup((__be16 *)(&raw_fifo_count));
	if (fifo_count < bytes_per_datum)
		goto end_session;

	samples = fifo_count / bytes_per_datum;

	/* fifo count can't be odd number, if it is odd, reset fifo*/
	if (fifo_count & 1)
		goto flush_fifo;

	if (fifo_count >  INV_MPU6050_FIFO_THRESHOLD)
		goto flush_fifo;

	/* Timestamp mismatch. */
	if (kfifo_len(&st->timestamps) > samples + INV_MPU6050_TIME_STAMP_TOR)
			goto flush_fifo;

	/* Read samples from I2C */
	result = inv_mpu6050_read_samples(st, fifo_count);
	if (result < 0)
		goto flush_fifo;

	if (indio_dev->scan_timestamp) {
		result = inv_mpu6050_create_timestamped_buffer(
						st, bytes_per_datum, samples);
		if (result)
			goto flush_fifo;

		result = iio_push_to_buffers_n(indio_dev, st->rx_buffer_ts,
					       samples);
		if (result)
			goto flush_fifo;
	} else {
		result = iio_push_to_buffers_n(indio_dev, st->rx_buffer,
					       samples);
		if (result)
			goto flush_fifo;
	}

end_session:
	mutex_unlock(&indio_dev->mlock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;

flush_fifo:
	/* Flush HW and SW FIFOs. */
	inv_reset_fifo(indio_dev);
	mutex_unlock(&indio_dev->mlock);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}
