/**
 *************************************************
 * @file p7-adc_core.h
 * @brief P7 Analogic to Digital Converter driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2013-09-1
 *************************************************
 */
#ifndef ADC_P7_ADC_CORE_H_
#define ADC_P7_ADC_CORE_H_

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <mach/p7-adc.h>

struct p7_adc_iio_chip_channel_ops
{
	int   (*enable)(unsigned int ch);
	int   (*disable)(unsigned int ch);
	int   (*config)(unsigned int ch,
			unsigned int freq);
	int   (*read)(unsigned int ch,
			unsigned short *val);
};

struct p7_adc_iio_chip_info
{
	struct list_head list;
	unsigned int type;
	struct iio_chan_spec channel_template;
	struct p7_adc_iio_chip_channel_ops *ops;
	struct module *owner;
};

/* Prototype of global function */
int p7_adc_iio_chip_get_channel_ops			(unsigned type,
		struct p7_adc_iio_chip_channel_ops **ops);
int p7_adc_iio_chip_get_channel_template      	        (unsigned type,
		struct iio_chan_spec **ch_templ);
int p7_adc_register_iio_chip  			(struct p7_adc_iio_chip_info *ch_info);
int p7_adc_unregister_iio_chip			(struct p7_adc_iio_chip_info *ch_info);
int p7_adc_get_iio_chip				(unsigned int type);
int p7_adc_put_iio_chip				(unsigned int type);

#endif /* ADC_P7_ADC_CORE_H_ */
