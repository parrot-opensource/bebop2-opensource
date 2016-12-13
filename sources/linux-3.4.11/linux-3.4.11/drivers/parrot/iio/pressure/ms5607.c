/*
 * ms5607.c - Support for Measurement Specialties MS5607-02BA03 pressure/temperature sensor
 *
 * Copyright (c) 2015 Parrot <didier.leymarie.ext@parrot.com>
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * (7-bit I2C slave address 0x76 or 0x77)
 *
 */
//#define DEBUG

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/math64.h>
#include <linux/module.h>
#ifdef CONFIG_PARROT_MS5607_NOTRIGGER
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#endif

#include <iio/platform_data/mykonos3.h>

#define MS5607_CMD_RESET	0x1E /* ADC reset command */
#define MS5607_CMD_ADC_READ	0x00 /* ADC read command */
#define MS5607_CMD_ADC_CONV	0x40 /* ADC conversion command */
#define MS5607_CMD_ADC_D1	0x00 /* ADC D1 conversion */
#define MS5607_CMD_ADC_D2	0x10 /* ADC D2 conversion */
#define MS5607_CMD_ADC_OSR_256	0x00 /* ADC conversion OSR=256 */
#define MS5607_CMD_ADC_OSR_512	0x02 /* ADC conversion OSR=512 */
#define MS5607_CMD_ADC_OSR_1024	0x04 /* ADC conversion OSR=1024 */
#define MS5607_CMD_ADC_OSR_2048	0x06 /* ADC conversion OSR=2048 */
#define MS5607_CMD_ADC_OSR_4096	0x08 /* ADC conversion OSR=4096 */
#define MS5607_CMD_PROM_RD	0xA0 /* PROM read command */

#define MS5607_PROM_SZ		8 /* size in word of calibration PROM */
enum ms5607_oversampling_ratio {
	OSR_256=0,
	OSR_512,
	OSR_1024,
	OSR_2048,
	OSR_4096,
	OSR_NR
};

struct ms5607_factory_calibration {
	uint32_t	C1;	/* Pressure sensitivity | SENS T1 */
	uint32_t	C2;	/* Pressure offset | OFF T1 */
	uint32_t	C3;	/* Temperature coefficient of pressure sensitivity | TCS */
	uint32_t	C4;	/* Temperature coefficient of pressure offset | TCO */
	uint32_t	C5;	/* Reference temperature | T REF */
	uint32_t	C6;	/* Temperature coefficient of the temperature | TEMPSENS */
};

enum ms5607_attributs {
	ATTR_PRESSURE,
	ATTR_TEMPERATURE
};

enum ms5607_channel_index {
	CHAN_PRESSURE=0,
	CHAN_TEMPERATURE,
	CHAN_TIMESTAMP,
	CHAN_NR,
	CHAN_DATA_NR=2
};

enum ms5607_timestamps_list {
  TS_trigger,

  TS_request_temperature,
  TS_ready_temperature,
  TS_read_temperature,
  TS_compute_temperature,
  TS_push_temperature,

  TS_request_pressure,
  TS_ready_pressure,
  TS_read_pressure,
  TS_compute_pressure,
  TS_push_pressure,

  TS_NR
};

struct ms5607_data {
	struct i2c_client			*client;
	struct mutex				lock;
	enum ms5607_oversampling_ratio		pressure_osr;
	enum ms5607_oversampling_ratio		temperature_osr;
	struct ms5607_factory_calibration	factory_calibration;
	s64					timestamps[TS_NR];
	struct {
		int32_t				data[CHAN_DATA_NR];
		s64				timestamp;
	} processed_buffer;
	struct {
		uint32_t			temperature;
		uint32_t			pressure;
	} raw_buffer;
	int					pressure_fetched;
	int					temperature_fetched;
};

static const int ms5607_oversampling_ratio_values[OSR_NR] = {
	[OSR_256]  = 256,
	[OSR_512]  = 512,
	[OSR_1024] = 1024,
	[OSR_2048] = 2048,
	[OSR_4096] = 4096
};
#define OSR_AVAILABLE		"256 512 1024 2048 4096"

#define PRESSURE_OSR_DEFAULT	OSR_1024
#define TEMPERATURE_OSR_DEFAULT	OSR_1024

/* module parameters */
static unsigned pressure_osr = 1<<(PRESSURE_OSR_DEFAULT+8);
module_param(pressure_osr, int, S_IRUGO);

static unsigned temperature_osr = 1<<(TEMPERATURE_OSR_DEFAULT+8);
module_param(temperature_osr, int, S_IRUGO);

static void setup_oversampling_ratios(struct ms5607_data *data)
{
	enum ms5607_oversampling_ratio new_osr, i;

	new_osr = PRESSURE_OSR_DEFAULT;
	for (i = OSR_256; i < OSR_NR; i++) {
		if (pressure_osr == ms5607_oversampling_ratio_values[i])
		{
			new_osr = i;
			break;
		}
	}
	data->pressure_osr = new_osr;

	new_osr = TEMPERATURE_OSR_DEFAULT;
	for (i = OSR_256; i < OSR_NR; i++) {
		if (temperature_osr == ms5607_oversampling_ratio_values[i])
		{
			new_osr = i;
			break;
		}
	}
	data->temperature_osr = new_osr;
}

/*
 * Shows the oversampling ratio
 */
static ssize_t show_oversampling_ratio(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct iio_dev	*indio_dev = dev_get_drvdata(dev);
	struct iio_dev_attr	*this_attr = to_iio_dev_attr(devattr);
	struct ms5607_data	*data = iio_priv(indio_dev);
	int			value;

	switch (this_attr->address) {
		case ATTR_PRESSURE:
			value = ms5607_oversampling_ratio_values[data->pressure_osr];
			break;
		case ATTR_TEMPERATURE:
			value = ms5607_oversampling_ratio_values[data->temperature_osr];
			break;
		default:
			return -EINVAL;
	}

	return sprintf(buf, "%d\n", value);
}

/*
 * Sets the oversampling ratio
 */
static ssize_t store_oversampling_ratio(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct iio_dev		*indio_dev	= dev_get_drvdata(dev);
	struct ms5607_data	*data		= iio_priv(indio_dev);
	struct iio_dev_attr	*this_attr	= to_iio_dev_attr(devattr);
	int value;
	int ret, f;
	enum ms5607_oversampling_ratio new_osr, prev_osr, i;

	switch (this_attr->address) {
	  case ATTR_PRESSURE:
	    prev_osr = data->pressure_osr;
	    break;
	  case ATTR_TEMPERATURE:
	    prev_osr = data->temperature_osr;
	    break;
	  default:
	    return -EINVAL;
	}

	ret = sscanf(buf, "%d", &value);
	if (ret != 1)
	      return -EINVAL;

	f = 0;
	for (i = OSR_256; i < OSR_NR; i++) {
		if (value == ms5607_oversampling_ratio_values[i])
		{
			new_osr = i;
			f = 1;
			break;
		}
	}
	if (!f)
		return -EINVAL;

	if (new_osr == prev_osr)
		return count;

	dev_dbg(&data->client->dev, "%s: Attribut %Lu, OSR %d -> %d\n", __func__,this_attr->address,
		ms5607_oversampling_ratio_values[prev_osr],
		ms5607_oversampling_ratio_values[new_osr]);

	mutex_lock(&data->lock);

	switch (this_attr->address) {
	  case ATTR_PRESSURE:
	    data->pressure_osr = new_osr;
	    break;
	  case ATTR_TEMPERATURE:
	    data->temperature_osr = new_osr;
	    break;
	}

	mutex_unlock(&data->lock);

	return count;
}

#ifdef CONFIG_PARROT_IIO_MS5607_TIMESTAMPS
/*
 * Shows the device's timestamps.
 */
static ssize_t show_timestamps(struct device *dev, struct device_attribute *devattr,
			       char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ms5607_data *data = iio_priv(indio_dev);
	int i, c=0;

	for (i=TS_trigger; i< TS_NR; i++) {
		c+=sprintf(&buf[c],"%Ld ", data->timestamps[i]);
	}
	return c;
}
#endif /* CONFIG_PARROT_IIO_MS5607_TIMESTAMPS */

#if 0
/*
 * Shows calibration values
 */
static ssize_t show_calibration(struct device *dev,
				struct device_attribute *devattr,
				char *buf)
{
	struct iio_dev		*indio_dev = dev_get_drvdata(dev);
	struct ms5607_data	*data = iio_priv(indio_dev);
	struct iio_dev_attr	*this_attr = to_iio_dev_attr(devattr);
	uint32_t		value;

	switch (this_attr->address) {
		case 0:
			value = data->factory_calibration.C1;
			break;
		case 1:
			value = data->factory_calibration.C2;
			break;
		case 2:
			value = data->factory_calibration.C3;
			break;
		case 3:
			value = data->factory_calibration.C4;
			break;
		case 4:
			value = data->factory_calibration.C5;
			break;
		case 5:
			value = data->factory_calibration.C6;
			break;
		default:
			return -EINVAL;
	}

	return sprintf(buf, "%u\n", value);
}
#endif

static int ms5607_read_calibration(struct ms5607_data *data)
{
	__be16			prom[MS5607_PROM_SZ];
	int			i, ret;
	unsigned int		crc_read, n_rem;
	unsigned char		n_bit;

	for (i = 0; i < MS5607_PROM_SZ; i++) {
		ret = i2c_smbus_write_byte(data->client, MS5607_CMD_PROM_RD+2*i);
		if (ret < 0) {
			dev_err(&data->client->dev, "%s: Command calibration fails (i = %d)\n", __func__, i);
			return ret;
		}
		ret = i2c_master_recv(data->client, (char *)&prom[i], sizeof(s16));
		if (ret < 0) {
			dev_err(&data->client->dev, "%s: Read calibration fails\n", __func__);
			return ret;
		}
		prom[i] = be16_to_cpu(prom[i]);
	}

	/* check CRC4 */
	n_rem = 0;
	crc_read = prom[7];
	prom[7] = prom[7] & 0xFF00;
	for ( i = 0; i < sizeof(prom); i++ ) {
		if ( i % 2 ==1)
			n_rem ^= prom[i>>1] & 0xFF;
		else
			n_rem ^= prom[i>>1] >> 8;
		for ( n_bit = 8; n_bit > 0; n_bit-- ) {
			if (n_rem & 0x8000)
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}
	n_rem = (0x000F & (n_rem >> 12));
	prom[7] = crc_read;
	n_rem = n_rem ^ 0x0;

	if ( n_rem != (crc_read & 0x000F)) {
		dev_err(&data->client->dev,
			"Invalid CRC4. Expected 0x%01X, Compute 0x%01X\n",
			crc_read & 0x000F, n_rem);
		return -EINVAL;
	}

	/* retrieve calibration data */
	i=1;
	data->factory_calibration.C1 = prom[i++];
	dev_info(&data->client->dev, "C1: %u\n", data->factory_calibration.C1);
	data->factory_calibration.C2 = prom[i++];
	dev_info(&data->client->dev, "C2: %u\n", data->factory_calibration.C2);
	data->factory_calibration.C3 = prom[i++];
	dev_info(&data->client->dev, "C3: %u\n", data->factory_calibration.C3);
	data->factory_calibration.C4 = prom[i++];
	dev_info(&data->client->dev, "C4: %u\n", data->factory_calibration.C4);
	data->factory_calibration.C5 = prom[i++];
	dev_info(&data->client->dev, "C5: %u\n", data->factory_calibration.C5);
	data->factory_calibration.C6 = prom[i++];
	dev_info(&data->client->dev, "C6: %u\n", data->factory_calibration.C6);

	return 0;
}

static const u8 ms5607_cmd_adc_conv[OSR_NR] = {
	[OSR_256]  = MS5607_CMD_ADC_CONV|MS5607_CMD_ADC_OSR_256,
	[OSR_512]  = MS5607_CMD_ADC_CONV|MS5607_CMD_ADC_OSR_512,
	[OSR_1024] = MS5607_CMD_ADC_CONV|MS5607_CMD_ADC_OSR_1024,
	[OSR_2048] = MS5607_CMD_ADC_CONV|MS5607_CMD_ADC_OSR_2048,
	[OSR_4096] = MS5607_CMD_ADC_CONV|MS5607_CMD_ADC_OSR_4096
};

/* for usleep_range {min, max} */
static const unsigned long ms5607_conversion_time[OSR_NR][2] = {
	[OSR_256]  = {  620,  650}, /* | 0.48 | 0.54 | 0.60 | ms */
	[OSR_512]  = { 1200, 1300}, /* | 0.95 | 1.06 | 1.17 | ms */
	[OSR_1024] = { 2300, 2300}, /* | 1.88 | 2.08 | 2.28 | ms */
	[OSR_2048] = { 4600, 4600}, /* | 3.72 | 4.13 | 4.54 | ms */
	[OSR_4096] = { 9100, 9100}, /* | 7.40 | 8.22 | 9.04 | ms */

};

#define MS5607_PRESSURE_MAX_VALUE	120000L
#define MS5607_PRESSURE_MIN_VALUE	30000L

#define MS5607_TEMPERATURE_MIN_VALUE	-19L
#define MS5607_TEMPERATURE_MAX_VALUE	100L

#define C_TEMPERATURE_SCALER               (100L)                         /**< x 100 scaler */
#define C_20_DEG_CELSIUS                   (20L * C_TEMPERATURE_SCALER)   /**< +20 degC */
#define C_15_DEG_CELSIUS                   (15L * C_TEMPERATURE_SCALER)   /**< +15 degC */

#define C_PRESSURE_SCALER                  (10LL)                       /**< x 10 scaler */

#define POW_2_4  ((uint64_t) (1U << 4U))
#define POW_2_6  ((uint64_t) (1U << 6U))
#define POW_2_7  ((uint64_t) (1U << 7U))
#define POW_2_8  ((uint64_t) (1U << 8U))
#define POW_2_15 ((uint64_t) (1U << 15U))
#define POW_2_16 ((uint64_t) (1U << 16U))
#define POW_2_17 ((uint64_t) (1U << 17U))
#define POW_2_21 ((uint64_t) (1U << 21U))
#define POW_2_23 ((uint64_t) (1U << 23U))
#define POW_2_31 ((uint64_t) (1U << 31U))

/**
 * See the data-sheet to understand this computation.
 */
static
int32_t ms5607_temperature_computation(struct ms5607_data *data,  const uint32_t D2)
{
	int64_t T2 = 0;
	int64_t dT = D2 - data->factory_calibration.C5 * POW_2_8;
	int32_t T = C_20_DEG_CELSIUS + (dT * data->factory_calibration.C6) / POW_2_23;

	if (T < C_20_DEG_CELSIUS)
	{
		T2 = dT * dT / POW_2_31;
	}

	T = T - T2;
	return T;/* unscaled temperature */
}

/**
 * See the data-sheet to understand this computation.
 * temp = (T - C_20_DEG_CELSIUS);
 * dT = (temp * POW_2_23) / C6;
 * OFF = C2 * POW_2_17 + (dT * C4) / POW_2_6;
 * SENS = C1 * POW_2_16 + (dT * C3) / POW_2_7;
 */
static
int32_t ms5607_pressure_computation(struct ms5607_data *data,
				    const int32_t temperature,
				    const uint32_t D1)
{
#define K1 61LL
#define K2 2LL
#define K3 15LL
#define K4 8LL
	int64_t		OFF2	= 0;
	int64_t		SENS2	= 0;
	const int64_t	T	= temperature;
	int64_t		temp = T - C_20_DEG_CELSIUS;
	int64_t		OFF,SENS;
	int32_t		P;

	OFF = (C_PRESSURE_SCALER * data->factory_calibration.C2 * POW_2_17)
	    + div_u64(C_PRESSURE_SCALER * temp * POW_2_17 * data->factory_calibration.C4,
		      data->factory_calibration.C6);

	 SENS = (C_PRESSURE_SCALER * data->factory_calibration.C1 * POW_2_16)
	      + div_u64(C_PRESSURE_SCALER * temp * POW_2_16 * data->factory_calibration.C3,
			data->factory_calibration.C6);

	if (T < C_20_DEG_CELSIUS)
	{
		temp = temp * temp;
		OFF2 = (C_PRESSURE_SCALER * K1 * temp) / POW_2_4;
		SENS2 = (C_PRESSURE_SCALER * K2 * temp);

		if (T < -C_15_DEG_CELSIUS)
		{
			temp = (T + C_15_DEG_CELSIUS);
			temp = temp * temp;
			OFF2  = OFF2  + (C_PRESSURE_SCALER * K3 * temp);
			SENS2 = SENS2 + (C_PRESSURE_SCALER * K4 * temp);
		}
	}
	SENS = SENS - SENS2;
	OFF = OFF - OFF2;
	P = (((D1 * SENS) / POW_2_21) - OFF) / POW_2_15;

	return P;
}

#define _RAW_PRESSURE data->raw_buffer.pressure
static int ms5607_read_raw_pressure(struct ms5607_data *data)
{
	union {
	      u8	byte[4];
	      __be32	ulong;
	} buffer;
	int 	ret;

	struct iio_dev *indio_dev = i2c_get_clientdata(data->client);

	data->timestamps[TS_request_pressure] = iio_get_time_ns(indio_dev);

	ret = i2c_smbus_write_byte(data->client,
		  MS5607_CMD_ADC_D1 | ms5607_cmd_adc_conv[data->pressure_osr]);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on i2c_smbus_write_byte(MS5607_CMD_ADC_D1)\n",
			__func__, ret);
		return ret;
	}

	usleep_range(ms5607_conversion_time[data->pressure_osr][0],
		     ms5607_conversion_time[data->pressure_osr][1]);

	data->timestamps[TS_ready_pressure] = iio_get_time_ns(indio_dev);

	ret = i2c_smbus_write_byte(data->client, MS5607_CMD_ADC_READ);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on i2c_smbus_write_byte(MS5607_CMD_ADC_READ)\n",
			__func__, ret);
		return ret;
	}

	buffer.ulong = 0;
	ret = i2c_master_recv(data->client, &buffer.byte[1], 3);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on i2c_master_recv\n", __func__, ret);
		return ret;
	}

	_RAW_PRESSURE = be32_to_cpu(buffer.ulong);

	data->timestamps[TS_read_pressure] = iio_get_time_ns(indio_dev);

	dev_dbg(&data->client->dev, "%s: raw_pressure %u\n", __func__, _RAW_PRESSURE);

	return IIO_VAL_INT;
}

#define _RAW_TEMPERATURE data->raw_buffer.temperature
static int ms5607_read_raw_temperature(struct ms5607_data *data)
{
	union {
		u8	byte[4];
		__be32  ulong;
	} buffer;
	int 	ret;

	struct iio_dev *indio_dev = i2c_get_clientdata(data->client);
	data->timestamps[TS_request_temperature] = iio_get_time_ns(indio_dev);

	ret = i2c_smbus_write_byte(data->client,
				   MS5607_CMD_ADC_D2 |
				  ms5607_cmd_adc_conv[data->temperature_osr]);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on i2c_smbus_write_byte(MS5607_CMD_ADC_D2)\n",
			__func__, ret);
		return ret;
	}

	usleep_range(ms5607_conversion_time[data->temperature_osr][0],
		     ms5607_conversion_time[data->temperature_osr][1]);

	data->timestamps[TS_ready_temperature] = iio_get_time_ns(indio_dev);

	ret = i2c_smbus_write_byte(data->client, MS5607_CMD_ADC_READ);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on i2c_smbus_write_byte(MS5607_CMD_ADC_READ)\n",
			__func__, ret);
		return ret;
	}

	buffer.ulong = 0;
	ret = i2c_master_recv(data->client, &buffer.byte[1], 3);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on i2c_master_recv\n", __func__, ret);
		return ret;
	}

	_RAW_TEMPERATURE = be32_to_cpu(buffer.ulong);

	data->timestamps[TS_read_temperature] = iio_get_time_ns(indio_dev);

	dev_dbg(&data->client->dev,
		"%s: raw_temperature %u\n", __func__,
		_RAW_TEMPERATURE);

	return IIO_VAL_INT;
}

#define _TEMPERATURE data->processed_buffer.data[CHAN_TEMPERATURE]
#define _PRESSURE    data->processed_buffer.data[CHAN_PRESSURE]
static int ms5607_read_processed_temperature(struct ms5607_data *data)
{
	int		ret;
	struct iio_dev *indio_dev = i2c_get_clientdata(data->client);

	ret = ms5607_read_raw_temperature(data);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on ms5607_read_raw_temperature\n", __func__, ret);
		return ret;
	}

	_TEMPERATURE = ms5607_temperature_computation(data, _RAW_TEMPERATURE);

	dev_dbg(&data->client->dev, "%s: temperature %d\n",
		__func__, _TEMPERATURE);

	if ( (_TEMPERATURE < (MS5607_TEMPERATURE_MIN_VALUE*C_TEMPERATURE_SCALER)) ||
	     (_TEMPERATURE > (MS5607_TEMPERATURE_MAX_VALUE*C_TEMPERATURE_SCALER)) ) {
		dev_err(&data->client->dev,
			"%s: out of range temperature: %d\n", __func__, _TEMPERATURE);
		return -EINVAL;
	}

	data->timestamps[TS_compute_temperature] = iio_get_time_ns(indio_dev);

	return IIO_VAL_INT;
}

static int ms5607_read_processed_pressure(struct ms5607_data *data)
{
	int ret;
	struct iio_dev *indio_dev = i2c_get_clientdata(data->client);

	ret = ms5607_read_raw_pressure(data);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"%s: error %d on ms5607_read_raw_pressure\n", __func__, ret);
		return ret;
	}

	_PRESSURE = ms5607_pressure_computation(data,
						_TEMPERATURE,
						_RAW_PRESSURE);

	dev_dbg(&data->client->dev, "%s: pressure %d\n", __func__, _PRESSURE);

	if ( (_PRESSURE < (MS5607_PRESSURE_MIN_VALUE * C_PRESSURE_SCALER)) ||
	     (_PRESSURE > (MS5607_PRESSURE_MAX_VALUE * C_PRESSURE_SCALER)) ) {
		dev_err(&data->client->dev,
			"%s: out of range pressure: %d\n", __func__, _PRESSURE);
		return -EINVAL;
	}

	data->timestamps[TS_compute_pressure] = iio_get_time_ns(indio_dev);

	return IIO_VAL_INT;
}

static int ms5607_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ms5607_data *data = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
		case IIO_CHAN_INFO_RAW:
			switch(chan->address) {
				case ATTR_PRESSURE:
					mutex_lock(&data->lock);
					data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);
					ret = ms5607_read_processed_pressure(data);
					*val  = _PRESSURE;
					*val2 = 0;
					data->timestamps[TS_push_pressure] = iio_get_time_ns(indio_dev);
					mutex_unlock(&data->lock);
					break;
				case ATTR_TEMPERATURE:
					mutex_lock(&data->lock);
					data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);
					ret = ms5607_read_processed_temperature(data);
					*val  = _TEMPERATURE;
					*val2 = 0;
					data->timestamps[TS_push_temperature] = iio_get_time_ns(indio_dev);
					mutex_unlock(&data->lock);
					break;
			}
			break;
		case IIO_CHAN_INFO_SCALE:
			*val2 = 0;
			switch(chan->address) {
				case ATTR_PRESSURE:
					*val  = 0;
					*val2 = 1000000/C_PRESSURE_SCALER;
					ret = IIO_VAL_INT_PLUS_MICRO;
					break;
				case ATTR_TEMPERATURE:
					*val  = 0;
					*val2 = 1000000/C_TEMPERATURE_SCALER;
					ret = IIO_VAL_INT_PLUS_MICRO;
					break;
			}
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}

#ifndef CONFIG_PARROT_MS5607_NOTRIGGER
static int ms5607_read(struct ms5607_data *data)
{
	int ret;

	mutex_lock(&data->lock);

	ret = ms5607_read_processed_temperature(data);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		goto exit;
	}

	ret = ms5607_read_processed_pressure(data);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		goto exit;
	}

exit:
	mutex_unlock(&data->lock);
	return ret;
}

static irqreturn_t ms5607_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ms5607_data *data = iio_priv(indio_dev);
	int ret;

	data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);
	ret = ms5607_read(data);
	if (ret < 0)
		goto done;

	data->timestamps[TS_push_pressure] = data->timestamps[TS_push_temperature] = iio_get_time_ns(indio_dev);
	iio_push_to_buffers_with_timestamp(indio_dev, &data->processed_buffer,
					   data->timestamps[TS_push_pressure]);

done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}
#endif

static const struct iio_chan_spec ms5607_channels[] = {
	{
		.type			= IIO_PRESSURE,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW)|BIT(IIO_CHAN_INFO_SCALE),
		.address		= ATTR_PRESSURE,
		.channel		= CHAN_PRESSURE,
		.scan_index		= CHAN_PRESSURE,
		.scan_type = {
			.sign		= 's',
			.storagebits	= 32,
			.realbits	= 32,
			.shift		= 0,
			.endianness	= IIO_CPU,
		},
		.indexed		= 1
	},

	{
		.type			= IIO_TEMP,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_RAW)|BIT(IIO_CHAN_INFO_SCALE),
		.address		= ATTR_TEMPERATURE,
		.channel		= CHAN_TEMPERATURE,
		.scan_index		= CHAN_TEMPERATURE,
		.scan_type = {
			.sign		= 's',
			.storagebits	= 32,
			.realbits	= 32,
			.shift		= 0,
			.endianness	= IIO_CPU,
		},
		.indexed		= 1
	},

	IIO_CHAN_SOFT_TIMESTAMP(CHAN_TIMESTAMP)
};

#if 0
/* calibration values */
static IIO_DEVICE_ATTR(in_calib_C1, S_IRUGO, show_calibration, NULL, 0);
static IIO_DEVICE_ATTR(in_calib_C2, S_IRUGO, show_calibration, NULL, 1);
static IIO_DEVICE_ATTR(in_calib_C3, S_IRUGO, show_calibration, NULL, 2);
static IIO_DEVICE_ATTR(in_calib_C4, S_IRUGO, show_calibration, NULL, 3);
static IIO_DEVICE_ATTR(in_calib_C5, S_IRUGO, show_calibration, NULL, 4);
static IIO_DEVICE_ATTR(in_calib_C6, S_IRUGO, show_calibration, NULL, 5);
#endif

/* constant IIO attribute */
static IIO_CONST_ATTR(oversampling_ratio_available, OSR_AVAILABLE);

/* oversample_ratios */
static IIO_DEVICE_ATTR(oversampling_ratio_pressure,
		       S_IRUGO | S_IWUSR,
		       show_oversampling_ratio,
		       store_oversampling_ratio,
		       ATTR_PRESSURE);
static IIO_DEVICE_ATTR(oversampling_ratio_temperature,
		       S_IRUGO | S_IWUSR,
		       show_oversampling_ratio,
		       store_oversampling_ratio,
		       ATTR_TEMPERATURE);

#ifdef CONFIG_PARROT_IIO_MS5607_TIMESTAMPS
static IIO_DEVICE_ATTR(timestamps, S_IRUGO, show_timestamps, NULL, 2);
#endif /* CONFIG_PARROT_IIO_MS5607_TIMESTAMPS */

static struct attribute *ms5607_attr[] = {
#if 0
	&iio_dev_attr_in_calib_C1.dev_attr.attr,
	&iio_dev_attr_in_calib_C2.dev_attr.attr,
	&iio_dev_attr_in_calib_C3.dev_attr.attr,
	&iio_dev_attr_in_calib_C4.dev_attr.attr,
	&iio_dev_attr_in_calib_C5.dev_attr.attr,
	&iio_dev_attr_in_calib_C6.dev_attr.attr,
#endif
	&iio_dev_attr_oversampling_ratio_pressure.dev_attr.attr,
	&iio_dev_attr_oversampling_ratio_temperature.dev_attr.attr,
	&iio_const_attr_oversampling_ratio_available.dev_attr.attr,
#ifdef CONFIG_PARROT_IIO_MS5607_TIMESTAMPS
	&iio_dev_attr_timestamps.dev_attr.attr,
#endif /* CONFIG_PARROT_IIO_MS5607_TIMESTAMPS */
	NULL
};

static struct attribute_group ms5607_attr_group = {
  .attrs = ms5607_attr,
};

static const struct iio_info ms5607_info = {
	.attrs = &ms5607_attr_group,
	.read_raw = &ms5607_read_raw,
	.driver_module = THIS_MODULE,
};

static const unsigned long ms5607_scan_masks[] = {0x3, 0}; /* 2 channels Pressure & Temperature */

#ifdef CONFIG_PARROT_MS5607_NOTRIGGER
static int ms5607_fetch_temp(struct iio_dev *indio_dev)
{
	struct ms5607_data *data = iio_priv(indio_dev);
	int ret = 0;

	if (!data->pressure_fetched && !data->temperature_fetched)
		data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);

	mutex_lock(&data->lock);

	ret = ms5607_read_processed_temperature(data);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		goto exit;
	}

	mutex_unlock(&data->lock);

	data->timestamps[TS_push_temperature] = iio_get_time_ns(indio_dev);;
	data->temperature_fetched = 1;

	if (data->pressure_fetched && data->temperature_fetched) {
		data->pressure_fetched = 0;
		data->temperature_fetched = 0;
		iio_push_to_buffers_with_timestamp(indio_dev,
					   &data->processed_buffer,
					   data->timestamps[TS_push_pressure]);
	}
exit:
	return ret;
}

static int ms5607_fetch_pressure(struct iio_dev *indio_dev)
{
	struct ms5607_data *data = iio_priv(indio_dev);
	int ret = 0;

	if (!data->pressure_fetched && !data->temperature_fetched)
		data->timestamps[TS_trigger] = iio_get_time_ns(indio_dev);

	mutex_lock(&data->lock);

	ret = ms5607_read_processed_pressure(data);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		goto exit;
	}

	mutex_unlock(&data->lock);

	data->timestamps[TS_push_pressure] = iio_get_time_ns(indio_dev);
	data->pressure_fetched = 1;

	if (data->pressure_fetched && data->temperature_fetched) {
		data->pressure_fetched = 0;
		data->temperature_fetched = 0;
		iio_push_to_buffers_with_timestamp(indio_dev,
					   &data->processed_buffer,
					   data->timestamps[TS_push_pressure]);
	}
exit:
	return ret;
}

static const struct iio_buffer_setup_ops ms5607_iio_buffer_setup_ops = {
};

struct iio_mykonos3_ops	ms5607_iio_mykonos3_ops_temp = {
	.request_data = NULL,
	.fetch_data   = ms5607_fetch_temp,
};

struct iio_mykonos3_ops	ms5607_iio_mykonos3_ops_pressure = {
	.request_data = NULL,
	.fetch_data   = ms5607_fetch_pressure,
};
#endif

static int ms5607_iio_buffer_new(struct iio_dev *indio_dev)
{
	int err = 0;
#ifdef CONFIG_PARROT_MS5607_NOTRIGGER
	struct iio_buffer *buffer = NULL;

	indio_dev->modes     = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ms5607_iio_buffer_setup_ops;

	buffer = iio_kfifo_allocate(indio_dev);
	if (!buffer) {
		err = -ENOMEM;
		goto exit;
	}

	iio_device_attach_buffer(indio_dev, buffer);

	err = iio_buffer_register(indio_dev, ms5607_channels,
				  ARRAY_SIZE(ms5607_channels));
	if (err < 0)
		goto exit;

	iio_mykonos3_register(IIO_MYKONOS3_BAROMETER, indio_dev,
			      &ms5607_iio_mykonos3_ops_pressure);
	iio_mykonos3_register(IIO_MYKONOS3_TEMP, indio_dev,
			      &ms5607_iio_mykonos3_ops_temp);
#else
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_triggered_buffer_setup(indio_dev, NULL,
					 ms5607_trigger_handler, NULL);
	if (err < 0)
		goto exit;
#endif
exit:
	return err;
}

static void ms5607_iio_buffer_cleanup(struct iio_dev *indio_dev)
{
#ifdef CONFIG_PARROT_MS5607_NOTRIGGER
	iio_kfifo_free(indio_dev->buffer);
	iio_mykonos3_unregister(IIO_MYKONOS3_BAROMETER, indio_dev);
#else
	iio_triggered_buffer_cleanup(indio_dev);
#endif
	iio_buffer_unregister(indio_dev);
}

static int ms5607_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ms5607_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	mutex_init(&data->lock);

	i2c_set_clientdata(client, indio_dev);
	indio_dev->info = &ms5607_info;
	indio_dev->name = id->name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = ms5607_channels;
	indio_dev->num_channels = ARRAY_SIZE(ms5607_channels);
	indio_dev->available_scan_masks = ms5607_scan_masks;

	setup_oversampling_ratios(data);

	ret = ms5607_read_calibration(data);
	/* XXX if (ret < 0)
	  	return ret;*/

	ret = ms5607_iio_buffer_new(indio_dev);
	if (ret < 0)
		return ret;

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0)
		goto buffer_cleanup;

	dev_info(&client->dev,
		 "Measurement Specialties MS5607-02BA03 pressure/temperature sensor (%s)\n",
		 indio_dev->name);

	return 0;

buffer_cleanup:
	ms5607_iio_buffer_cleanup(indio_dev);
	return ret;
}

static int ms5607_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	devm_iio_device_unregister(&client->dev, indio_dev);
	ms5607_iio_buffer_cleanup(indio_dev);

	return 0;
}

static const struct i2c_device_id ms5607_id[] = {
	{ "ms5607", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ms5607_id);

static struct i2c_driver ms5607_driver = {
	.driver   = {
		.owner = THIS_MODULE,
		.name  = "ms5607",
	},
	.probe    = ms5607_probe,
	.remove   = ms5607_remove,
	.id_table = ms5607_id,
};
module_i2c_driver(ms5607_driver);

MODULE_AUTHOR("Didier Leymarie <didier.leymarie.ext@parrot.com>");
MODULE_DESCRIPTION("Measurement Specialties MS5607-02BA03 pressure/temperature sensor");
MODULE_LICENSE("GPL");
