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
#ifndef __IIO_PARROT_BLDC_CYPRESS_H__
#define __IIO_PARROT_BLDC_CYPRESS_H__

#include <linux/i2c.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/i2c.h>

int bldc_cypress_probe(struct iio_dev *indio_dev);
void bldc_cypress_remove(struct iio_dev *indio_dev);
void bldc_cypress_shutdown(struct iio_dev *indio_dev);
irqreturn_t bldc_cypress_read_fifo(int irq, void *p);
int bldc_cypress_fetch_data(struct iio_dev *indio_dev);

#endif
