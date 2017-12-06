/**
 ************************************************
 * @file p7mu-adc_core.c
 * @brief P7 Analogic to Digital Converter driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2013-09-19
 *************************************************
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <mfd/p7mu.h>
#include <mfd/p7mu-clk.h>
#include <spi/p7-spis.h> /* Necessary to flush SPIS fifo. */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/iio/machine.h>
#include <linux/iio/driver.h>

#include "p7-adc_regs.h"
#include "p7-adc_core.h"
#include "p7mu-adc.h"

#define P7MU_ADC_DRV_NAME "p7mu-adc"

typedef enum  {
	CAPTURE_SAMPLING_MODE = 0,
	CAPTURE_ONE_SAMPLE_MODE,
} capture_mode_t;

/* These modules parameters allow to override platform data.
 * Could be used for testing */
static unsigned int spi_freq = 0;
module_param(spi_freq, uint, 0644);

static struct p7mu_adc_chip_info p7mu_adc;

static inline u32 p7mu_get_mod(u32 freq)
{
	u32 mod;
	mod = ((P7MU_ADC_FREQ / freq) - 1);
	return mod;
}

static void p7mu_adc_start(struct p7mu_adc_chip_info* st)
{
	p7mu_write16(st->p7mu_adc_res->start + P7MU_ADC_CH_EN_REG,
			st->chan_en_reg_val);

	/* Enable ADC */
	p7mu_write16(st->p7mu_adc_res->start + P7MU_ADC_CTRL_REG,
			P7MU_ADC_EN|P7MU_ADC_OM_ON);
}

static void p7mu_adc_stop(struct p7mu_adc_chip_info* st)
{
	/* Disable all channels */
	p7mu_write16(st->p7mu_adc_res->start + P7MU_ADC_CH_EN_REG, 0);
	/* Disable ADC and reset */
	p7mu_write16(st->p7mu_adc_res->start + P7MU_ADC_CTRL_REG, 0x20);
}

static void p7mu_adc_config(struct p7mu_adc_chip_info* st)
{
	u32 ch = 0, prio = 0;
	u32 ch_priority_hi = 0, ch_priority_lo = 0;

	/* Set marker channel with the highest priority
	   PRIORITY_SEL_0 Register
	   [15:12] not implemented.
	   [11:9] PRI3_CH_SEL – Channel with priority 3.
	   [8:6] PRI2_CH_SEL – Channel with priority 2.
	   [5:3] PRI1_CH_SEL – Channel with priority 1.
	   [2:0] PRI0_CH_SEL – Channel with priority 0 (the highest priority).

	   PRIORITY_SEL_1 Register
	   [15:12] not implemented.
	   [11:9] PRI7_CH_SEL – Channel with priority 7 (the lowest priority).
	   [8:6] PRI6_CH_SEL – Channel with priority 6.
	   [5:3] PRI5_CH_SEL – Channel with priority 5.
	   [2:0] PRI4_CH_SEL – Channel with priority 4.
	 */
	if (st->ch_marker != -1) {
		ch_priority_hi |= st->ch_marker;
		prio++;
	}

	for (; prio < 4; ++ch)
		if (ch != st->ch_marker)
			ch_priority_hi |= P7MU_ADC_CH_PRIO_SEL(ch, prio++);

	for (prio = 0; prio < 4; ++ch)
		if (ch != st->ch_marker)
			ch_priority_lo |= P7MU_ADC_CH_PRIO_SEL(ch, prio++);

	p7mu_write16(st->p7mu_adc_res->start + P7MU_ADC_PRIORITY_SEL_0_REG,
			ch_priority_hi & P7MU_ADC_PRIO_MASK);
	p7mu_write16(st->p7mu_adc_res->start + P7MU_ADC_PRIORITY_SEL_1_REG,
			ch_priority_lo & P7MU_ADC_PRIO_MASK);

	p7mu_adc_start(st);
}

static int p7mu_adc_channel_if_select(unsigned int ch, int capture_mode)
{
	u16 val;

	mutex_lock(&p7mu_adc.mutex);
	if (capture_mode == CAPTURE_SAMPLING_MODE) {
		p7mu_read16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_IF_SEL_REG,
				&val);
		p7mu_write16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_IF_SEL_REG,
				val | P7MU_ADC_CH_SPI_IF_SEL(ch));
	} else {
		p7mu_read16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_IF_SEL_REG,
				&val);
		p7mu_write16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_IF_SEL_REG,
				val | P7MU_ADC_CH_I2C_IF_SEL(ch));
	}
	mutex_unlock(&p7mu_adc.mutex);

	return 0;
}

static int p7mu_adc_channel_enable(unsigned int ch, int capture_mode)
{
	mutex_lock(&p7mu_adc.mutex);
	if (capture_mode == CAPTURE_SAMPLING_MODE)
		p7mu_adc.chan_en_reg_val |= P7MU_ADC_CH_SAMPLING_EN(ch);
	else
		p7mu_adc.chan_en_reg_val |= P7MU_ADC_CH_ONE_SAMPLE_EN(ch);

	p7mu_write16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_EN_REG,
			p7mu_adc.chan_en_reg_val);
	mutex_unlock(&p7mu_adc.mutex);

	return 0;
}

static int p7mu_adc_sampling_channel_enable(unsigned int ch)
{
	return p7mu_adc_channel_enable(ch, CAPTURE_SAMPLING_MODE);
}

static int p7mu_adc_one_sample_channel_enable(unsigned int ch)
{
	return p7mu_adc_channel_enable(ch, CAPTURE_ONE_SAMPLE_MODE);
}

static int p7mu_adc_channel_disable(unsigned int ch, int capture_mode)
{
	mutex_lock(&p7mu_adc.mutex);
	if (capture_mode == CAPTURE_SAMPLING_MODE)
		p7mu_adc.chan_en_reg_val &= ~P7MU_ADC_CH_SAMPLING_EN(ch);
	else
		p7mu_adc.chan_en_reg_val &= ~P7MU_ADC_CH_ONE_SAMPLE_EN(ch);

	p7mu_write16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_EN_REG,
			p7mu_adc.chan_en_reg_val);
	mutex_unlock(&p7mu_adc.mutex);

	return 0;
}

static inline int p7mu_adc_sampling_channel_disable(unsigned int ch)
{
	return p7mu_adc_channel_disable(ch, CAPTURE_SAMPLING_MODE);
}

static inline int p7mu_adc_one_sample_channel_disable(unsigned int ch)
{
	return p7mu_adc_channel_disable(ch, CAPTURE_ONE_SAMPLE_MODE);
}

static int p7mu_adc_config_channel(unsigned int ch, int capture_mode, u32 freq)
{
	u32 mod;

	if (freq > 0)
	{
		mod = p7mu_get_mod(freq);
		p7mu_write16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_MOD_0_REG(ch),
				(u16)(mod & P7MU_ADC_CH_MOD_0_MASK));
		p7mu_write16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_MOD_1_REG(ch),
				(u16)((mod >> 16) & P7MU_ADC_CH_MOD_1_MASK));
	}

	p7mu_adc_channel_if_select(ch, capture_mode);

	return 0;
}

static inline int p7mu_adc_sampling_channel_config(unsigned int ch, u32 freq)
{
	return p7mu_adc_config_channel(ch, CAPTURE_SAMPLING_MODE, freq);
}

static inline int p7mu_adc_one_sample_channel_config(unsigned int ch, u32 freq)
{
	return p7mu_adc_config_channel(ch, CAPTURE_ONE_SAMPLE_MODE, freq);
}

static int p7mu_adc_read_one_sample_channel(unsigned int ch, unsigned short *val)
{
	p7mu_adc_channel_enable(ch,
			CAPTURE_ONE_SAMPLE_MODE);

	p7mu_read16(p7mu_adc.p7mu_adc_res->start + P7MU_ADC_CH_DATA_REG(ch),
			val);

	if ((*val & P7MU_ADC_I2C_VALID_MASK) == 0)
		return -EINVAL;

	return 0;
}

static int p7mu_adc_read_sampling_channel(unsigned int ch, unsigned short *val)
{
	/* Not supported */
	return -EINVAL;
}

static int p7mu_adc_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *chan,
		int *val,
		int *val2,
		long m)
{
	return 0;
}

static const struct iio_info p7mu_adc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &p7mu_adc_read_raw,
};

static struct p7mu_adc_iio_dev *p7mu_adc_alloc_channel(struct device *dev,
		struct p7mu_adc_chip_info *chip_info,
		const struct p7_adc_chan *chan_info)
{
	struct p7mu_adc_state *st;
	struct p7mu_adc_iio_dev *new;
	struct iio_chan_spec *chan;
	struct iio_chan_spec *ch_templ;
	struct iio_dev *indio_dev;
	int err = 0;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return NULL;

	st = iio_priv(indio_dev);
	st->chip_info = chip_info;
	p7_adc_iio_chip_get_channel_ops(chan_info->type,
			&st->ops);

	chan = kzalloc(sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (chan == NULL) {
		err = -ENOMEM;
		goto free_device;
	}

	p7_adc_iio_chip_get_channel_template(chan_info->type,
			&ch_templ);

	*chan = *ch_templ;
	chan->channel = chan_info->channel;
	chan->address = chan_info->channel;
	chan->scan_index = chan_info->channel;

	indio_dev->dev.parent = dev;
	indio_dev->info = &p7mu_adc_info;
	indio_dev->channels = chan;
	indio_dev->num_channels = 1;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->name = kasprintf(GFP_KERNEL, "%s_%d", P7MU_ADC_DRV_NAME,
			chan->channel);

	err = p7mu_adc_register_ring_funcs_and_init(indio_dev);
	if (err)
		goto free_chan;

	err = iio_buffer_register(indio_dev,
			indio_dev->channels,
			indio_dev->num_channels);
	if (err)
		goto clear_ring;

	/* associate channel and buffer */
	iio_scan_mask_set(indio_dev, indio_dev->buffer, chan->channel);

	/* Set default buffer length */
// 	indio_dev->buffer->access->set_bytes_per_datum(indio_dev->buffer,
// 			indio_dev->channels[0].scan_type.storagebits/8);
	indio_dev->buffer->access->set_length(indio_dev->buffer,
					      chan_info->samples);

	err = devm_iio_device_register(dev, indio_dev);
	if (err)
		goto free_buf;

	new = kzalloc(sizeof(*new), GFP_KERNEL);
	if (!new) {
		goto free_buf;
	}

	new->channel = chan_info->channel;
	new->indio_dev = indio_dev;
	list_add(&new->l, &chip_info->iio_dev_list);

	return new;

free_buf:
	iio_buffer_unregister(indio_dev);
clear_ring:
	p7mu_adc_ring_cleanup(indio_dev);
free_device:
	devm_iio_device_free(dev, indio_dev);
free_chan:
	kfree(indio_dev->channels);
	kfree(indio_dev->name);

	return NULL;
}

static void p7mu_adc_iiodev_delete(struct p7mu_adc_iio_dev *d)
{
	struct iio_dev *indio_dev = d->indio_dev;

	list_del(&d->l);
	devm_iio_device_unregister(&indio_dev->dev, indio_dev);
	iio_buffer_unregister(indio_dev);
	p7mu_adc_ring_cleanup(indio_dev);
	kfree(indio_dev->name);
	kfree(indio_dev->channels);
	devm_iio_device_free(indio_dev->dev.parent, indio_dev);
	kfree(d);
}

int p7mu_adc_remove(struct p7mu_adc_chip_info *ch_info)
{
	struct p7mu_adc_iio_dev *d, *n;

	list_for_each_entry_safe(d, n, &ch_info->iio_dev_list, l)
		p7mu_adc_iiodev_delete(d);

	p7mu_adc_stop(ch_info);

	return 0;
}

#define P7_ADC_CHAN(_type, _shift, _mask) { \
	.type = _type, \
	.indexed = 1, \
	.info_mask_separate = _mask, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = _shift, \
		.endianness	= IIO_CPU, \
	} \
}

static struct p7_adc_iio_chip_channel_ops sampling_chann_ops = {
	.enable = p7mu_adc_sampling_channel_enable,
	.disable = p7mu_adc_sampling_channel_disable,
	.config = p7mu_adc_sampling_channel_config,
	.read = p7mu_adc_read_sampling_channel,
};

static struct p7_adc_iio_chip_channel_ops one_sample_chann_ops = {
	.enable = p7mu_adc_one_sample_channel_enable,
	.disable = p7mu_adc_one_sample_channel_disable,
	.config = p7mu_adc_one_sample_channel_config,
	.read = p7mu_adc_read_one_sample_channel,
};

static struct p7_adc_iio_chip_info tci = {
	.type = P7MUADC_IIO_TEMP,
	.channel_template = P7_ADC_CHAN(IIO_TEMP, 0, BIT(IIO_CHAN_INFO_RAW)),
	.owner = THIS_MODULE,
	.ops = &one_sample_chann_ops,
};

static struct p7_adc_iio_chip_info rbci = {
	.type = P7MUADC_IIO_RING_BUFFER,
	.channel_template = P7_ADC_CHAN(IIO_VOLTAGE, 4, BIT(IIO_CHAN_INFO_RAW)),
	.ops = &sampling_chann_ops,
	.owner = THIS_MODULE,
};

static struct p7_adc_iio_chip_info mci = {
	.type = P7MUADC_IIO_MARKER,
	.channel_template = P7_ADC_CHAN(IIO_VOLTAGE, 4, BIT(IIO_CHAN_INFO_RAW)),
	.ops = &sampling_chann_ops,
	.owner = THIS_MODULE,
};

static int __devinit p7mu_adc_plat_probe(struct platform_device* pdev)
{
	int i, ret;
	struct p7mu_plat_data const* const pdata = p7mu_pdata();

	if (p7mu_adc.p7mu_adc_init)
		return -EBUSY;

	p7mu_adc.p7mu_adc_res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!p7mu_adc.p7mu_adc_res) {
		dev_err(&pdev->dev, "failed to find P7MU ADC I/O region address\n");
		return -ENXIO;
	}
	p7mu_adc.p7mu_adc_res = p7mu_request_region(pdev, p7mu_adc.p7mu_adc_res);
	if (IS_ERR(p7mu_adc.p7mu_adc_res)) {
		dev_err(&pdev->dev, "failed to request P7MU ADC I/O region\n");
		ret = PTR_ERR(p7mu_adc.p7mu_adc_res);
		return -ENXIO;
	}

	p7mu_adc.p7mu_spi_res = platform_get_resource(pdev, IORESOURCE_IO, 1);
	if (!p7mu_adc.p7mu_spi_res) {
		dev_err(&pdev->dev, "failed to find P7MU SPI I/O region address\n");
		p7mu_adc.state_mask |= P7MU_ADC_STATE_FAULT_SPI;
		goto spi_skip;
	}
	p7mu_adc.p7mu_spi_res = p7mu_request_region(pdev, p7mu_adc.p7mu_spi_res);
	if (IS_ERR(p7mu_adc.p7mu_spi_res)) {
		dev_err(&pdev->dev, "failed to request P7MU SPI I/O region\n");
		p7mu_adc.state_mask |= P7MU_ADC_STATE_FAULT_SPI;
	}

spi_skip:
	p7_adc_register_iio_chip(&tci);
	p7_adc_register_iio_chip(&rbci);
	p7_adc_register_iio_chip(&mci);

	mutex_init(&p7mu_adc.mutex);

	INIT_LIST_HEAD(&p7mu_adc.iio_dev_list);
	p7mu_adc.ch_masked = -1;
	p7mu_adc.ch_marker = -1;

	if (pdata && pdata->chan_data) {
		for (i = 0; i < pdata->chan_data->num_channels; i++) {
			const struct p7_adc_chan *chan_pdata =
				&pdata->chan_data->channels[i];

			if (chan_pdata->type == P7MUADC_IIO_MARKER) {
				p7mu_adc_configure_marker(&p7mu_adc,
						chan_pdata->channel);
			} else {
				if (!p7mu_adc_alloc_channel(&pdev->dev,
							&p7mu_adc,
							chan_pdata))
					continue;
			}

			p7mu_adc_config_channel(chan_pdata->channel,
					CAPTURE_SAMPLING_MODE,
					chan_pdata->freq);
		}
	} else {
		dev_warn(&pdev->dev, "No ADC channel pdata, disabling SPI\n");
		p7mu_adc.state_mask |= P7MU_ADC_STATE_FAULT_SPI;
	}

	if (!(p7mu_adc.state_mask & P7MU_ADC_STATE_FAULT_SPI))
		p7mu_adc_configure_ring(&p7mu_adc);

	/* Setup and start ADC */
	p7mu_adc_stop(&p7mu_adc);
	msleep(1);
	p7mu_adc.state_mask |= P7MU_ADC_STATE_RUNNING;
	p7mu_adc_config(&p7mu_adc);

	p7mu_adc.p7mu_adc_init = 1;
	dev_info(&pdev->dev, "P7MU ADC - ready\n");

	return 0;
}

static int __devexit p7mu_adc_plat_remove(struct platform_device* pdev)
{
	p7mu_adc_remove(&p7mu_adc);

	p7_adc_unregister_iio_chip(&tci);
	p7_adc_unregister_iio_chip(&rbci);
	p7_adc_unregister_iio_chip(&mci);

	if (!IS_ERR_OR_NULL(p7mu_adc.p7mu_spi_res))
		p7mu_release_region(p7mu_adc.p7mu_spi_res);

	if (!IS_ERR_OR_NULL(p7mu_adc.p7mu_adc_res))
		p7mu_release_region(p7mu_adc.p7mu_adc_res);

	p7mu_adc.p7mu_adc_init = 0;

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int p7mu_adc_suspend(struct device *dev)
{
	struct p7mu_adc_iio_dev *d;

	list_for_each_entry(d, &p7mu_adc.iio_dev_list, l)
		if (iio_buffer_enabled(d->indio_dev))
			d->indio_dev->setup_ops->postdisable(d->indio_dev);

	p7mu_adc_stop(&p7mu_adc);

	return 0;
}

static int p7mu_adc_resume(struct device *dev)
{
	struct p7mu_adc_iio_dev *d;

	p7mu_adc_start(&p7mu_adc);

	list_for_each_entry(d, &p7mu_adc.iio_dev_list, l)
		if (iio_buffer_enabled(d->indio_dev))
			d->indio_dev->setup_ops->postenable(d->indio_dev);

	return 0;
}
#else
#define p7mu_adc_suspend   NULL
#define p7mu_adc_resume    NULL
#endif

static const struct dev_pm_ops p7mu_adc_dev_pm_ops = {
	.suspend  = p7mu_adc_suspend,
	.resume   = p7mu_adc_resume,
};

/* Driver structure for platform driver, used to get the i2c resources */
static struct platform_driver p7mu_adc_plat_driver = {
	.driver = {
		.name   = P7MU_ADC_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm     = &p7mu_adc_dev_pm_ops,
	},
	.probe  = p7mu_adc_plat_probe,
	.remove = __devexit_p(&p7mu_adc_plat_remove),
};

/* SPI */

static int p7mu_adc_spi_setup(struct p7mu_adc_chip_info* st)
{
	u16 const spi_mode = (u16)(st->spi->mode & P7MU_SPI_CPOL_CPHA_MASK);
	u32 freq = spi_freq? spi_freq : st->spi->max_speed_hz;
	u32 clk_spi_sel;

	/* Round SPI frequency to possible configurations :
	   p7mu spi base clk could be P7MU_SPI_HZ_MIN or P7MU_SPI_HZ_MAX
	   generate correct divisor
	 */
	if (p7mu_get_spiclk_rate() == P7MU_SPI_HZ_MIN) {
		if (freq > P7MU_SPI_8MHZ) {
			st->spi_freq = P7MU_SPI_16MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_16MHZ;
		} else if (freq > P7MU_SPI_4MHZ) {
			st->spi_freq = P7MU_SPI_8MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_8MHZ;
		} else if (freq > P7MU_SPI_2MHZ) {
			st->spi_freq = P7MU_SPI_4MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_4MHZ;
		} else {
			st->spi_freq = P7MU_SPI_2MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_2MHZ;
		}
	} else {
		if (freq > P7MU_SPI_12MHZ) {
			st->spi_freq = P7MU_SPI_24MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_24MHZ;
		} else if (freq > P7MU_SPI_6MHZ) {
			st->spi_freq = P7MU_SPI_12MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_12MHZ;
		} else if (freq > P7MU_SPI_3MHZ) {
			st->spi_freq = P7MU_SPI_6MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_6MHZ;
		} else {
			st->spi_freq = P7MU_SPI_3MHZ;
			clk_spi_sel = P7MU_SPI_CLK_SEL_3MHZ;
		}
	}
	if (st->spi_freq != freq)
		pr_info( "P7MU SPI frequency corrected to: %d Hz (wanted %d Hz)\n",
			st->spi_freq, freq);

	/* Enable spi with given options */
	p7mu_write16(st->p7mu_spi_res->start + P7MU_SPI_CTRL_REG,
			P7MU_SPI_EN|clk_spi_sel|spi_mode);

	return 0;
}

static int p7mu_adc_spi_probe(struct spi_device *spi)
{
	if (!p7mu_adc.p7mu_adc_init)
		return -ENODEV;

	if (p7mu_adc.state_mask & P7MU_ADC_STATE_FAULT_SPI)
		return -ENOTSUPP;

	/* Disable SPI */
	p7mu_write16(p7mu_adc.p7mu_spi_res->start + P7MU_SPI_CTRL_REG, 0);

	p7mu_adc.spi = spi;
	p7mu_adc_spi_setup(&p7mu_adc);
	p7spis_pause(spi->master);

	return 0;
}

static int p7mu_adc_spi_remove(struct spi_device *spi)
{
	unsigned limit = 500;

	p7mu_adc.state_mask &= ~P7MU_ADC_STATE_RUNNING;
	if (p7mu_adc.ch_marker != -1)
		p7mu_adc.marker_ops->enable(p7mu_adc.ch_marker);

	while (p7mu_adc.msg_pending && limit--)
		msleep(10);

	if (p7mu_adc.ch_marker != -1)
		p7mu_adc.marker_ops->disable(p7mu_adc.ch_marker);

	/* Disable SPI */
	p7mu_write16(p7mu_adc.p7mu_spi_res->start + P7MU_SPI_CTRL_REG, 0);
	p7spis_flush(spi->master);

	return 0;
}

static struct spi_device_id const p7mu_adc_spi_id[] = {
	{ P7MU_ADC_DRV_NAME, 0 },
	{ }
};

/* driver structure for the spi interface. */
static struct spi_driver p7mu_adc_spi_driver = {
	.probe		= p7mu_adc_spi_probe,
	.remove		= __devexit_p(p7mu_adc_spi_remove),
	.driver		= {
		.name	= P7MU_ADC_DRV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.id_table = p7mu_adc_spi_id,
};

static int __init p7mu_adc_init(void)
{
	int err;

	err = platform_driver_register(&p7mu_adc_plat_driver);
	if (err)
		return err;

	return spi_register_driver(&p7mu_adc_spi_driver);
}
module_init(p7mu_adc_init);

static void __exit p7mu_adc_exit(void)
{
	spi_unregister_driver(&p7mu_adc_spi_driver);
	platform_driver_unregister(&p7mu_adc_plat_driver);
}
module_exit(p7mu_adc_exit);

MODULE_AUTHOR("Karl Leplat <karl.leplat@parrot.com>");
MODULE_DESCRIPTION("Parrot Power Management Unit ADC driver");
MODULE_LICENSE("GPL");
