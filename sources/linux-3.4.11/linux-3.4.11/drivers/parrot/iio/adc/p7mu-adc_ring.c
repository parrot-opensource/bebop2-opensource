/**
 *************************************************
 * @file p7mu-adc_ring.c
 * @brief P7 Analogic to Digital Converter driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2013-09-19
 *************************************************
 */
#define DEBUG
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

#include <spi/p7-spis.h> /* Necessary to flush SPIS fifo. */
#include "p7-adc_core.h"
#include "p7mu-adc.h"

/* Timeout before to received the marker */
#define SEND_MARKER_TIMEOUT 5 /* ms */
static int p7mu_adc_send_marker(struct iio_dev *indio_dev, unsigned int ch)
{
	struct p7mu_adc_state *st = iio_priv(indio_dev);
	struct p7mu_adc_chip_info *chip_info = st->chip_info;
	int err = 0;
	int retry = 10;

	mutex_lock(&chip_info->send_mutex);

	do {
		chip_info->ch_masked = ch;
		chip_info->marker_ops->enable(chip_info->ch_marker);
		usleep_range(2000, 2000);
		chip_info->marker_ops->disable(chip_info->ch_marker);

		/* wait for marker to receive */
		err = wait_for_completion_timeout(&chip_info->send_marker_done,
				msecs_to_jiffies(SEND_MARKER_TIMEOUT));
		if (!err) {
			dev_dbg(&indio_dev->dev, "send marker timeout\n");
			err = -ETIMEDOUT;
			continue;
		} else {
			err = 0;
			break;
		}
	} while (retry--);

	mutex_unlock(&chip_info->send_mutex);

	return err;
}

static int iio_setup_buffer_postenable(struct iio_dev *indio_dev)
{
	unsigned int channel;
	struct p7mu_adc_state *st = iio_priv(indio_dev);

	if (bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
		return -EINVAL;

	channel = find_first_bit(indio_dev->active_scan_mask,
			indio_dev->masklength);

	if (st->chip_info->state_mask & P7MU_ADC_STATE_MARKER)
		p7mu_adc_send_marker(indio_dev, channel);
	else {
		int size;
		if (st->chip_info->chan_en_reg_val & 0xf) {
			/* we support only one spi chanel in single mode */
			printk(KERN_WARNING "p7mu adc : only one spi channel supported\n");
			return -EINVAL;
		}
		/* libHAL really expect to have a complete read (and not a partial read) */
		size = indio_dev->buffer->access->get_length(indio_dev->buffer);
		if (size <= P7MU_ADC_BUF_MAX)
			st->chip_info->xfer.len = size;
		else
			st->chip_info->xfer.len = 8192;
	}
	if ((st->chip_info->chan_en_reg_val & 0xf) == 0) {
		/* start transfert */
		int ret;
		p7spis_resume(st->chip_info->spi->master);
		st->chip_info->msg_pending = 1;
		st->chip_info->msg_full = 0;
		ret = spi_async(st->chip_info->spi, &st->chip_info->msg);
		if (ret) {
			dev_err(&st->chip_info->spi->dev, "spi_async --> %d\n", ret);
			return ret;
		}
	}

	st->ops->enable(channel);

// 	dev_dbg(&indio_dev->dev, "%s channel #%d\n", __func__, channel);

	return 0;
}

static int iio_setup_buffer_postdisable(struct iio_dev *indio_dev)
{
	unsigned int val, channel;
	struct p7mu_adc_state *st = iio_priv(indio_dev);

	if (bitmap_empty(indio_dev->active_scan_mask, indio_dev->masklength))
		return -EINVAL;

	channel = find_first_bit(indio_dev->active_scan_mask,
			indio_dev->masklength);

	st->ops->disable(channel);

	/* invalidate dma and buffer */
	if ((st->chip_info->chan_en_reg_val & 0xf) == 0)
		p7spis_pause(st->chip_info->spi->master);

	val = indio_dev->buffer->access->get_bytes_per_datum(indio_dev->buffer);
	indio_dev->buffer->access->set_bytes_per_datum(indio_dev->buffer, 0);
	indio_dev->buffer->access->set_bytes_per_datum(indio_dev->buffer, val);
	indio_dev->buffer->access->request_update(indio_dev->buffer);

// 	dev_dbg(&indio_dev->dev, "%s channel #%d\n", __func__, channel);

	return 0;
}

static const struct iio_buffer_setup_ops p7mu_adc_ring_setup_ops = {
	.postenable = &iio_setup_buffer_postenable,
	.postdisable = &iio_setup_buffer_postdisable,
};

int p7mu_adc_register_ring_funcs_and_init(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;

	buffer = iio_kfifo_allocate(indio_dev);

	if (!buffer)
		return -ENOMEM;

	/* Ring buffer functions - here start/stop new capture related */
	indio_dev->setup_ops = &p7mu_adc_ring_setup_ops;

	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->modes = INDIO_BUFFER_HARDWARE;

	dev_dbg(&indio_dev->dev, "%s\n", __func__);

	return 0;
}

static struct p7mu_adc_iio_dev *p7mu_adc_find_dev_from_ch(struct p7mu_adc_chip_info *chip_info, int ch)
{
	struct p7mu_adc_iio_dev *dev;

	list_for_each_entry(dev, &chip_info->iio_dev_list, l)
		if (dev->channel == ch)
			return dev;

	return NULL;
}

static int p7mu_adc_push_to_buffer(struct p7mu_adc_chip_info* chip_info)
{
	int i, ch;
	u16 data;
	struct p7mu_adc_iio_dev *dev;
	int ret = 0;

// 	pr_debug("%s: len=%d\n", __func__, chip_info->xfer.len);

	if (chip_info->state_mask & P7MU_ADC_STATE_MARKER) {
		/* XXX sending sample by sample is very slow ... */
		for (i = 0; i < chip_info->xfer.len / 2; i++)
		{
			data = le16_to_cpup((const __le16 *)&chip_info->rx[i*2]);
			ch = data & P7MU_ADC_SPI_CHANNEL_MASK;
			data &= ~P7MU_ADC_SPI_CHANNEL_MASK;

			if (ch == chip_info->ch_marker) {

				if (chip_info->ch_masked != -1) {
					chip_info->ch_masked = -1;
					complete(&chip_info->send_marker_done);
				}
				continue;
			}

			if (chip_info->ch_masked != ch) {
				dev = p7mu_adc_find_dev_from_ch(chip_info, ch);
				if (dev != NULL)
					if (iio_buffer_enabled(dev->indio_dev))
						ret |= dev->indio_dev->buffer->access->store_to(dev->indio_dev->buffer,
								(void *)&data);
			}
		}
	}
	else {
		int n;
		data = le16_to_cpup((const __le16 *)&chip_info->rx[0]);
		ch = data & P7MU_ADC_SPI_CHANNEL_MASK;
		dev = p7mu_adc_find_dev_from_ch(chip_info, ch);
		if (!dev)
			return -ENODEV;
		if (!dev->indio_dev->buffer->access->store_to_buffer)
			return -EINVAL;
		n = dev->indio_dev->buffer->access->store_to_buffer(dev->indio_dev->buffer,
				(void *)chip_info->rx, chip_info->xfer.len / 2);
		if (n != chip_info->xfer.len / 2)
			ret = -EBUSY;
	}

	if (ret && !chip_info->msg_full) {
		pr_warn("p7mu_adc_push_to_buffer : fifo is full\n");
		chip_info->msg_full = 1;
	}

	return ret;
}

/*
 * This function may be called from context where it can't sleep,
 * so we must protect it with a spinlock
 */
static void p7mu_read_ring_data(void *data)
{
	struct p7mu_adc_chip_info *chip_info = (struct p7mu_adc_chip_info *) data;
	struct spi_message* mesg = &chip_info->msg;


	if (!(chip_info->state_mask & P7MU_ADC_STATE_RUNNING)) {
		chip_info->msg_pending = 0;
		return;
	}
	if (mesg->status) {
		chip_info->msg_pending = 0;
		return;
	}

	p7mu_adc_push_to_buffer(chip_info);
	spi_async(chip_info->spi, mesg);
}

void p7mu_adc_ring_cleanup(struct iio_dev *indio_dev)
{
	dev_dbg(&indio_dev->dev, "%s\n", __func__);
	if (iio_buffer_enabled(indio_dev)) {
		/* Force update to free ring buffer */
		indio_dev->buffer->access->set_bytes_per_datum(indio_dev->buffer, 0);
		indio_dev->buffer->access->request_update(indio_dev->buffer);
	}
	iio_kfifo_free(indio_dev->buffer);
}

int p7mu_adc_configure_marker(struct p7mu_adc_chip_info *ci,
			unsigned int ch_marker)
{
	int ret;

	ci->ch_marker = ch_marker;
	ret = p7_adc_iio_chip_get_channel_ops(P7MUADC_IIO_MARKER,
			&ci->marker_ops);
	if (ret < 0)
		return ret;
	init_completion(&ci->send_marker_done);
	mutex_init(&ci->send_mutex);
	ci->state_mask |= P7MU_ADC_STATE_MARKER;
	return 0;
}

int p7mu_adc_configure_ring(struct p7mu_adc_chip_info *ci)
{
	spi_message_init(&ci->msg);
	ci->msg.complete = p7mu_read_ring_data;
	ci->msg.context = ci;

	ci->xfer.rx_buf = ci->rx;
	ci->xfer.len = 1024;
	ci->xfer.cs_change = 1;
	spi_message_add_tail(&ci->xfer, &ci->msg);

	return 0;
}
