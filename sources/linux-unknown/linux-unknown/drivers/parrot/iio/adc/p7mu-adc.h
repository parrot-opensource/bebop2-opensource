/**
 ***************************************************
 * @file p7mu-adc.h
 * @brief P7MU Analogic to Digital Converter driver
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2013-09-19
 ***************************************************
 */
#ifndef ADC_P7MU_ADC_H_
#define ADC_P7MU_ADC_H_

#include <linux/kernel.h>
#include <linux/spi/spi.h>

/* ADC frequency in kHz */
#define P7MU_ADC_FREQ	8000000

/**********************************/
/* P7MU ADC registers and options */
/* Registers address */
#define P7MU_ADC_CTRL_REG		0x00
#define P7MU_ADC_CH_MOD_0_REG(_ch)	(_ch*2 + 0x02)
#define P7MU_ADC_CH_MOD_1_REG(_ch)	(_ch*2 + 0x03)
#define P7MU_ADC_CH0_TRIG_0_REG		0x12
#define P7MU_ADC_CH_EN_REG		0x22
#define P7MU_ADC_CH_DATA_REG(_ch)	(_ch + 0x23)
#define P7MU_ADC_CH_IF_SEL_REG		0x2B
#define P7MU_ADC_PRIORITY_SEL_0_REG	0x2C
#define P7MU_ADC_PRIORITY_SEL_1_REG	0x2D
#define P7MU_ADC_MEASUREMENT_STATUS_REG 0x32
#define P7MU_ADC_MEASUREMENT_ERROR_REG  0x33

/* Options */
#define P7MU_ADC_EN			(1 << 4)
#define P7MU_ADC_ONE_POINT_TEMP_CORR	(1 << 3)
#define P7MU_ADC_OM_ON			(u16)(0x3) /* Operating Mode On */
#define P7MU_ADC_CH_SAMPLING_EN(_ch)	(u16)(1 << _ch)
#define P7MU_ADC_CH_ONE_SAMPLE_EN(_ch)	(u16)(1 << (8+_ch))
#define P7MU_ADC_CH_I2C_IF_SEL(_ch)	(u16)(1 << (8+_ch))
#define P7MU_ADC_CH_SPI_IF_SEL(_ch)	(u16)(1 << _ch)
#define P7MU_ADC_CH_MOD_0_MASK		0xffff
#define P7MU_ADC_CH_MOD_1_MASK		0x7f
#define P7MU_ADC_CH_PRIO_SEL(_ch, _prio)	(_ch << (3*_prio))
#define P7MU_ADC_PRIO_MASK		0x0FFF

/* Buffer max size */
#define P7MU_ADC_BUF_MAX 20000

/*
 * P7MUADC value masks
 */
#define P7MU_ADC_SPI_CHANNEL_MASK 0x7
#define P7MU_ADC_I2C_VALID_MASK   0x8000

/* State */
#define P7MU_ADC_STATE_RUNNING		BIT(0)
#define P7MU_ADC_STATE_FAULT_SPI	BIT(1)
#define P7MU_ADC_STATE_MARKER		BIT(2)

/* one channel per iio_dev */
struct p7mu_adc_iio_dev {
	struct iio_dev   *indio_dev;
	unsigned int     channel;
	struct list_head l;
};

struct p7mu_adc_chip_info
{
	struct spi_device       *spi;       /* spi_device reference */
	u32			spi_freq;
	u32                     msg_pending;
	u32                     msg_full;
	u32                     state_mask;
	u16                     chan_en_reg_val;
	struct resource const*  p7mu_adc_res;
	struct resource const*	p7mu_spi_res;
	int			p7mu_adc_init;

	struct spi_message      msg;        /* spi message for the spi API */
	struct spi_transfer     xfer;       /* only one transfer per message */

	struct list_head        iio_dev_list;

	int                     ch_masked;
	int                     ch_marker;
	struct  completion      send_marker_done;
	struct mutex            send_mutex; /* Serialize send marker access */
	struct p7_adc_iio_chip_channel_ops *marker_ops;
	struct mutex            mutex;      /* Serialize chip info struct access */

	unsigned char           rx[P7MU_ADC_BUF_MAX] ____cacheline_aligned;
};

struct p7mu_adc_state
{
	struct p7mu_adc_chip_info		*chip_info;
	struct p7_adc_iio_chip_channel_ops 	*ops;
};

int p7mu_adc_register_ring_funcs_and_init(struct iio_dev *indio_dev);
void p7mu_adc_ring_cleanup(struct iio_dev *indio_dev);
int p7mu_adc_configure_marker(struct p7mu_adc_chip_info *ci,
			unsigned int ch_marker);
int p7mu_adc_configure_ring(struct p7mu_adc_chip_info *ci);

#endif /* ADC_P7MU_ADC_H_ */
