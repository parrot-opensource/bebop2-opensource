/**
********************************************************************************
* @file p7-adc.h
* @brief P7 Analogic to Digital Converter driver
*
* Copyright (C) 2013 Parrot S.A.
*
* @author Karl Leplat <karl.leplat@parrot.com>
* @date 2013-09-19
********************************************************************************
*/
#ifndef ADC_P7_ADC_H_
#define ADC_P7_ADC_H_

#include <linux/kernel.h>

/* Max sampling frequency based on clk_adc 8Mhz - do not change */
#define P7MUADC_MARKER_FREQ 1200000

typedef enum {
	P7MUADC_IIO_TEMP = 0,
	P7MUADC_IIO_RING_BUFFER,
	P7MUADC_IIO_MARKER,
	P7_ADC_IIO_MAX_CHIP_INFO,
} p7_adc_type;

struct p7_adc_chan {
	int type;
	int channel;
	u32 freq;
	u32 samples;
};

struct p7_adc_chan_data {
	struct p7_adc_chan *channels;
	size_t		num_channels;
};

struct p7_temp_chan {
	int channel;
	u32 freq;
	const char *name;
};

enum {
	P7_TEMP_FC7100_HW08,
	P7_TEMP_FC7100_HW04,
	P7_TEMP_SICILIA,
};
struct p7_temp_chan_data {
	struct p7_temp_chan *channels;
	size_t		num_channels;
	int	temp_mode;
};

#endif /* ADC_P7_ADC_H_ */
