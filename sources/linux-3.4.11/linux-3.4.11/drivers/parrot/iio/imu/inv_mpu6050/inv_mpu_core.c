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
*
* For all 'magic' numbers in this file, please refer to datasheets :
*   mpu6050 : RM-MPU-6000A.pdf
*   mpu6500 : RM-MPU-6500A-00.pdf
*/

//#define DEBUG

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
#include "inv_mpu_iio.h"

/*
 * this is the gyro scale translated from dynamic range plus/minus
 * {250, 500, 1000, 2000} to rad/s
 */
/* pi */
# define M_PI           3.14159265358979323846
static const int gyro_scale_6050[NUM_MPU6050_FSR] = {
	[INV_MPU6050_FSR_250DPS]  = (M_PI*1000000000LL)/(180*131.0), /*133090*/
	[INV_MPU6050_FSR_500DPS]  = (M_PI*1000000000LL)/(180*65.5),  /*266181*/
	[INV_MPU6050_FSR_1000DPS] = (M_PI*1000000000LL)/(180*32.8),  /*532362*/
	[INV_MPU6050_FSR_2000DPS] = (M_PI*1000000000LL)/(180*16.4),  /*1064724*/
};

/*
 * this is the accel scale translated from dynamic range plus/minus
 * {2, 4, 8, 16} to m/s^2
 */
/* standard acceleration of gravity (gee, free-fall on Earth) */
#define G_TO_MS2                      9.80665
static const int accel_scale[NUM_ACCL_FSR] = {
	[INV_MPU6050_FS_02G] = (G_TO_MS2*1000000000LL)/16384, /*598*/
	[INV_MPU6050_FS_04G] = (G_TO_MS2*1000000000LL)/8192,  /*1196*/
	[INV_MPU6050_FS_08G] = (G_TO_MS2*1000000000LL)/4096,  /*2392*/
	[INV_MPU6050_FS_16G] = (G_TO_MS2*1000000000LL)/2048,  /*4785*/
};

static const unsigned long temp_scale[INV_NUM_PARTS][2] = {
	[INV_MPU6050] = {0, INV_MPU6050_TEMP_SCALE},
	[INV_MPU6500] = {0, INV_MPU6500_TEMP_SCALE}
};

static const unsigned long temp_offset[INV_NUM_PARTS][2] = {
	[INV_MPU6050] = {INV_MPU6050_TEMP_OFFSET, 0},
	[INV_MPU6500] = {INV_MPU6500_TEMP_OFFSET, 0}
};

static const struct inv_mpu6050_reg_map reg_set_6050 = {
	.sample_rate_div	= INV_MPU6050_REG_SAMPLE_RATE_DIV,
	.config                 = INV_MPU6050_REG_CONFIG,
	.user_ctrl              = INV_MPU6050_REG_USER_CTRL,
	.fifo_en                = INV_MPU6050_REG_FIFO_EN,
	.gyro_config            = INV_MPU6050_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU6050_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_MPU6050_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU6050_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU6050_REG_RAW_GYRO,
	.raw_accl               = INV_MPU6050_REG_RAW_ACCEL,
	.temperature            = INV_MPU6050_REG_TEMPERATURE,
	.int_pin_cfg		= INV_MPU6050_REG_INT_PIN_CFG,
	.int_pin_cfg_set	= 0,
	/* INT_LEVEL=0: active high,
	 * INT_OPEN=0:  push-pull
	 * LATCH_INT_EN=0: 50 us long pulse
	 * INT_RD_CLEAR=1: interrupt status bits are cleared on any read operation
	 * FSYNC_INT_LEVEL=0: FSYNC active high
	 * FSYNC_INT_EN=0: disables FSYNC interrupt
	 * I2C_BYPASS_EN=0: no access the auxiliary i2c bus
	 */
	.int_enable             = INV_MPU6050_REG_INT_ENABLE,
	.pwr_mgmt_1             = INV_MPU6050_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU6050_REG_PWR_MGMT_2,
};

static const struct inv_mpu6050_reg_map reg_set_6500 = {
	.sample_rate_div	= INV_MPU6050_REG_SAMPLE_RATE_DIV,
	.config                 = INV_MPU6050_REG_CONFIG,
	.user_ctrl              = INV_MPU6050_REG_USER_CTRL,
	.fifo_en                = INV_MPU6050_REG_FIFO_EN,
	.gyro_config            = INV_MPU6050_REG_GYRO_CONFIG,
	.accl_config            = INV_MPU6050_REG_ACCEL_CONFIG,
	.fifo_count_h           = INV_MPU6050_REG_FIFO_COUNT_H,
	.fifo_r_w               = INV_MPU6050_REG_FIFO_R_W,
	.raw_gyro               = INV_MPU6050_REG_RAW_GYRO,
	.raw_accl               = INV_MPU6050_REG_RAW_ACCEL,
	.temperature            = INV_MPU6050_REG_TEMPERATURE,
	.int_pin_cfg		= INV_MPU6050_REG_INT_PIN_CFG,
	.int_pin_cfg_set	= INV_MPU6050_BIT_INT_RD_CLEAR,
	/* INT_LEVEL=0: active high,
	 * INT_OPEN=0:  push-pull
	 * LATCH_INT_EN=0: 50 us long pulse
	 * INT_RD_CLEAR=1: interrupt status bits are cleared on any read operation
	 * FSYNC_INT_LEVEL=0: FSYNC active high
	 * FSYNC_INT_EN=0: disables FSYNC interrupt
	 * I2C_BYPASS_EN=0: no access the auxiliary i2c bus
	 */
	.int_enable             = INV_MPU6050_REG_INT_ENABLE,
	.pwr_mgmt_1             = INV_MPU6050_REG_PWR_MGMT_1,
	.pwr_mgmt_2             = INV_MPU6050_REG_PWR_MGMT_2,
};

static const struct inv_mpu6050_chip_config chip_config_6050 = {
	.fsync  = INV_MPU6050_FSYNC_ACCEL_ZOUT,
	.clksel = INV_CLK_EXTERNAL_32_KHZ,
	.fsr = INV_MPU6050_FSR_2000DPS,
	.lpf = INV_MPU6050_FILTER_256HZ_NOLPF2,
	.fifo_rate = INV_MPU6050_INIT_FIFO_RATE,
	.fifo_period_ns = 1000000000LL/INV_MPU6050_INIT_FIFO_RATE,
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.temp_fifo_enable = false,
	.accl_fs = INV_MPU6050_FS_04G,
};

static const struct inv_mpu6050_chip_config chip_config_6500 = {
	.fsync  = INV_MPU6050_FSYNC_DISABLED,
	.clksel = INV_CLK_PLL,
	.fsr = INV_MPU6050_FSR_2000DPS,
	.lpf = INV_MPU6050_FILTER_20HZ,
	.fifo_rate = INV_MPU6050_INIT_FIFO_RATE,
	.fifo_period_ns = 1000000000LL/INV_MPU6050_INIT_FIFO_RATE,
	.gyro_fifo_enable = false,
	.accl_fifo_enable = false,
	.temp_fifo_enable = false,
	.accl_fs = INV_MPU6050_FS_02G,
};

static const struct inv_mpu6050_hw hw_info[INV_NUM_PARTS] = {
	[INV_MPU6050] = {
		.num_reg = 117,
		.name = "MPU6050",
		.reg = &reg_set_6050,
		.config = &chip_config_6050,
	},
	[INV_MPU6500] = {
		.num_reg = 117,
		.name = "MPU6500",
		.reg = &reg_set_6500,
		.config = &chip_config_6500,
	},
};

int inv_mpu6050_write_reg(struct inv_mpu6050_state *st, int reg, u8 d)
{
	return i2c_smbus_write_i2c_block_data(st->client, reg, 1, &d);
}

int inv_mpu6050_switch_engine(struct inv_mpu6050_state *st, bool en, u32 mask)
{
	u8 d, mgmt_1;
	int result;

	/* switch clock needs to be careful. Only when gyro is on, can
	   clock source be switched to gyro. Otherwise, it must be set to
	   internal clock */
	if (INV_MPU6050_BIT_PWR_GYRO_STBY == mask) {
		result = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->pwr_mgmt_1, 1, &mgmt_1);
		if (result != 1)
			return result;

		mgmt_1 &= ~INV_MPU6050_BIT_CLK_MASK;
	}

	if ((INV_MPU6050_BIT_PWR_GYRO_STBY == mask) && (!en)) {
		/* turning off gyro requires switch to internal clock first.
		   Then turn off gyro engine */
		mgmt_1 |= INV_CLK_INTERNAL;
		result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1, mgmt_1);
		if (result)
			return result;
	}

	result = i2c_smbus_read_i2c_block_data(st->client,
				       st->reg->pwr_mgmt_2, 1, &d);
	if (result != 1)
		return result;
	if (en)
		d &= ~mask;
	else
		d |= mask;
	result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_2, d);
	if (result)
		return result;

	if (en) {
		/* Wait for output stabilize */
		msleep(INV_MPU6050_TEMP_UP_TIME);
		if (INV_MPU6050_BIT_PWR_GYRO_STBY == mask) {
			/* switch internal clock to PLL */
			mgmt_1 |= st->chip_config.clksel;
			result = inv_mpu6050_write_reg(st,
					st->reg->pwr_mgmt_1, mgmt_1);
			if (result)
				return result;
		}
	}

	return 0;
}

int inv_mpu6050_set_power_itg(struct inv_mpu6050_state *st, bool power_on)
{
	int result;

	if (power_on)
		result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1, 0);
	else
		result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1,
						INV_MPU6050_BIT_SLEEP);
	if (result)
		return result;

	if (power_on)
		msleep(INV_MPU6050_REG_UP_TIME);

	return 0;
}

/**
 *  inv_mpu6050_init_config() - Initialize hardware, disable FIFO.
 *
 *  Initial configuration:
 *  FSR: ± 2000DPS
 *  DLPF: 20Hz
 *  FIFO rate: 50Hz
 *  Clock source: Gyro PLL
 */
static int inv_mpu6050_init_config(struct iio_dev *indio_dev)
{
	int result;
	u8 d;
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		return result;
	d = (INV_MPU6050_FSR_2000DPS << INV_MPU6050_GYRO_CONFIG_FSR_SHIFT);
	result = inv_mpu6050_write_reg(st, st->reg->gyro_config, d);
	if (result)
		return result;

	d = INV_MPU6050_FILTER_256HZ_NOLPF2 << INV_MPU6050_BIT_DLPF_CFG_SHIFT;
	d |= INV_MPU6050_FSYNC_ACCEL_ZOUT << INV_MPU6050_BIT_EXT_SYNC_SET_SHIFT;
	result = inv_mpu6050_write_reg(st, st->reg->config, d);
	if (result)
		return result;

	/* When DLPF config is INV_MPU6050_FILTER_256HZ_NOLPF2, the sample
	 * rate is 8000Hz */
	d = (8 * INV_MPU6050_ONE_K_HZ) / INV_MPU6050_INIT_FIFO_RATE - 1;
	result = inv_mpu6050_write_reg(st, st->reg->sample_rate_div, d);
	if (result)
		return result;

	d = (INV_MPU6050_FS_04G << INV_MPU6050_ACCL_CONFIG_FSR_SHIFT);
	result = inv_mpu6050_write_reg(st, st->reg->accl_config, d);
	if (result)
		return result;

	memcpy(&st->chip_config, hw_info[st->chip_type].config,
		sizeof(struct inv_mpu6050_chip_config));
	result = inv_mpu6050_set_power_itg(st, false);

	return result;
}

static int inv_mpu6050_sensor_show(struct inv_mpu6050_state  *st, int reg,
				int axis, int *val)
{
	int ind, result;
	__be16 d;

	ind = (axis - IIO_MOD_X) * 2;
	result = i2c_smbus_read_i2c_block_data(st->client, reg + ind,  2,
						(u8 *)&d);
	if (result != 2)
		return -EINVAL;
	*val = (short)be16_to_cpup(&d);

	return IIO_VAL_INT;
}

static int inv_mpu6050_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_mpu6050_state  *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		int ret, result;

		ret = IIO_VAL_INT;
		result = 0;
		mutex_lock(&indio_dev->mlock);
		if (!st->chip_config.enable) {
			result = inv_mpu6050_set_power_itg(st, true);
			if (result)
				goto error_read_raw;
		}
		/* when enable is on, power is already on */
		switch (chan->type) {
		case IIO_ANGL_VEL:
			if (!st->chip_config.gyro_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, true,
						INV_MPU6050_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			ret =  inv_mpu6050_sensor_show(st, st->reg->raw_gyro,
						chan->channel2, val);
			if (!st->chip_config.gyro_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, false,
						INV_MPU6050_BIT_PWR_GYRO_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_ACCEL:
			if (!st->chip_config.accl_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, true,
						INV_MPU6050_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			ret = inv_mpu6050_sensor_show(st, st->reg->raw_accl,
						chan->channel2, val);
			if (!st->chip_config.accl_fifo_enable ||
					!st->chip_config.enable) {
				result = inv_mpu6050_switch_engine(st, false,
						INV_MPU6050_BIT_PWR_ACCL_STBY);
				if (result)
					goto error_read_raw;
			}
			break;
		case IIO_TEMP:
			/* Temp sensor is never disabled,
				just wait for its stablization */
			msleep(INV_MPU6050_SENSOR_UP_TIME);
			inv_mpu6050_sensor_show(st, st->reg->temperature,
							IIO_MOD_X, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
error_read_raw:
		if (!st->chip_config.enable)
			result |= inv_mpu6050_set_power_itg(st, false);
		mutex_unlock(&indio_dev->mlock);
		if (result)
			return result;

		return ret;
	}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val  = 0;
			*val2 = gyro_scale_6050[st->chip_config.fsr];

			return IIO_VAL_INT_PLUS_NANO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = accel_scale[st->chip_config.accl_fs];

			return IIO_VAL_INT_PLUS_NANO;
		case IIO_TEMP:
			*val  = temp_scale[st->chip_type][0];
			*val2 = temp_scale[st->chip_type][1];

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			*val  = temp_offset[st->chip_type][0];
			*val2 = temp_offset[st->chip_type][1];

			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int inv_mpu6050_write_gyro_scale(struct inv_mpu6050_state *st, int val, int val2)
{
	int result, i;
	u8 d;

	if (val)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(gyro_scale_6050); ++i) {
		if (gyro_scale_6050[i] == val2) {
			d = (i << INV_MPU6050_GYRO_CONFIG_FSR_SHIFT);
			result = inv_mpu6050_write_reg(st,
					st->reg->gyro_config, d);
			if (result)
				return result;
			st->chip_config.fsr = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_mpu6050_write_accel_scale(struct inv_mpu6050_state *st, int val, int val2)
{
	int result, i;
	u8 d;

	if (val)
	  return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(accel_scale); ++i) {
		if (accel_scale[i] == val2) {
			d = (i << INV_MPU6050_ACCL_CONFIG_FSR_SHIFT);
			result = inv_mpu6050_write_reg(st,
					st->reg->accl_config, d);
			if (result)
				return result;
			st->chip_config.accl_fs = i;
			return 0;
		}
	}

	return -EINVAL;
}

static int inv_mpu6050_write_raw_get_fmt(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 long mask)
{
	switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			switch (chan->type) {
				case IIO_ANGL_VEL:
					return IIO_VAL_INT_PLUS_NANO;
					break;
				case IIO_ACCEL:
					return IIO_VAL_INT_PLUS_NANO;
					break;
				default:
					return -EINVAL;
					break;
			}
			break;
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
}

static int inv_mpu6050_write_raw(struct iio_dev *indio_dev,
			         struct iio_chan_spec const *chan,
			         int val,
			         int val2,
			         long mask)
{
	struct inv_mpu6050_state  *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	/* we should only update scale when the chip is disabled, i.e.,
		not running */
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto error_write_raw;
	}
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		goto error_write_raw;

	switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			switch (chan->type) {
			case IIO_ANGL_VEL:
				result = inv_mpu6050_write_gyro_scale(st,
								      val,
								      val2);
				break;
			case IIO_ACCEL:
				result = inv_mpu6050_write_accel_scale(st,
								       val,
								       val2);
				break;
			default:
				result = -EINVAL;
				break;
			}
			break;
		default:
			result = -EINVAL;
			break;
	}

error_write_raw:
	result |= inv_mpu6050_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);

	return result;
}

/**
 *  inv_mpu6050_set_lpf() - set low pass filer based on fifo rate.
 *
 *                  Based on the Nyquist principle, the sampling rate must
 *                  exceed twice of the bandwidth of the signal, or there
 *                  would be alising. This function basically search for the
 *                  correct low pass parameters based on the fifo rate, e.g,
 *                  sampling frequency.
 */
static int inv_mpu6050_set_lpf(struct inv_mpu6050_state *st, int rate)
{
	const int hz[] = {188, 98, 42, 20, 10, 5};
	const int d[] = {INV_MPU6050_FILTER_188HZ, INV_MPU6050_FILTER_98HZ,
			INV_MPU6050_FILTER_42HZ, INV_MPU6050_FILTER_20HZ,
			INV_MPU6050_FILTER_10HZ, INV_MPU6050_FILTER_5HZ};
	int i, h, result;
	u8 data;

	result = i2c_smbus_read_i2c_block_data(st->client,
					       st->reg->config, 1, &data);
	if (result != 1)
		return result;
	data &= ~(INV_MPU6050_BIT_DLPF_CFG_MASK<<INV_MPU6050_BIT_DLPF_CFG_SHIFT);
	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d) - 1))
		i++;

	data |= (d[i] & INV_MPU6050_BIT_DLPF_CFG_MASK)<<INV_MPU6050_BIT_DLPF_CFG_SHIFT;

	result = inv_mpu6050_write_reg(st, st->reg->config, data);
	if (result)
		return result;
	st->chip_config.lpf = data;

	return 0;
}

/**
 *  inv_mpu6050_set_fsync() - Set external Frame Synchronization (FSYNC).
 *
 */
static int inv_mpu6050_set_fsync(struct inv_mpu6050_state *st)
{
	int result;
	u8 data;

	result = i2c_smbus_read_i2c_block_data(st->client,
					      st->reg->config, 1, &data);
	if (result != 1)
		return result;
	data &= ~(INV_MPU6050_BIT_EXT_SYNC_SET_MASK<<INV_MPU6050_BIT_EXT_SYNC_SET_SHIFT);

	data |= ( st->chip_config.fsync & INV_MPU6050_BIT_EXT_SYNC_SET_MASK)<<INV_MPU6050_BIT_EXT_SYNC_SET_SHIFT;

	result = inv_mpu6050_write_reg(st, st->reg->config, data);
	if (result)
		return result;

	return 0;
}

/**
 * inv_mpu6050_fifo_rate_store() - Set fifo rate.
 */
static ssize_t inv_mpu6050_fifo_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	s32 fifo_rate;
	u8 d;
	int result;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	if (kstrtoint(buf, 10, &fifo_rate))
		return -EINVAL;
	if (fifo_rate < INV_MPU6050_MIN_FIFO_RATE ||
				fifo_rate > INV_MPU6050_MAX_FIFO_RATE)
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;

	mutex_lock(&indio_dev->mlock);
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto fifo_rate_fail;
	}
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		goto fifo_rate_fail;

	/* When DLPF config is INV_MPU6050_FILTER_256HZ_NOLPF2, the sample
	 * rate is 8000Hz */
	d = (8 * INV_MPU6050_ONE_K_HZ) / INV_MPU6050_INIT_FIFO_RATE - 1;
	result = inv_mpu6050_write_reg(st, st->reg->sample_rate_div, d);
	if (result)
		goto fifo_rate_fail;
	st->chip_config.fifo_rate = fifo_rate;

	/* convert FIFO rate in FIFO period (nanoseconds) */
	st->chip_config.fifo_period_ns = div_u64(1000000000LL, fifo_rate);
	/* filter period (nanoseconds)
	 * from inv_mpu6050_platform_data.filter_rate
	 */
	st->chip_config.filter_period_ns =
		st->plat_data.filter_rate * st->chip_config.fifo_period_ns;

	result = inv_mpu6050_set_lpf(st, fifo_rate);
	if (result)
		goto fifo_rate_fail;

	result = inv_mpu6050_set_fsync(st);
	if (result)
		goto fifo_rate_fail;

fifo_rate_fail:
	result |= inv_mpu6050_set_power_itg(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

/**
 * inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_mpu6050_state *st = iio_priv(dev_to_iio_dev(dev));

	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

#define STR_INV_CLK_INTERNAL		"internal"
#define STR_INV_CLK_PLL_X		"pll_gyro_x"
#define STR_INV_CLK_PLL_Y		"pll_gyro_y"
#define STR_INV_CLK_PLL_Z		"pll_gyro_z"
#define STR_INV_CLK_EXTERNAL_32_KHZ	"external_32k"
#define STR_INV_CLK_EXTERNAL_19_MHZ	"external_19m"
#define STR_INV_CLK_PLL			"pll" /* for MPU6500 */

static const char *inv_mpu6050_clock_sel_e_str[INV_NUM_PARTS][NUM_CLK] = {
	[INV_MPU6050] = {
		[INV_CLK_INTERNAL]		= STR_INV_CLK_INTERNAL,
		[INV_CLK_PLL_X]			= STR_INV_CLK_PLL_X,
		[INV_CLK_PLL_Y]			= STR_INV_CLK_PLL_Y,
		[INV_CLK_PLL_Z]			= STR_INV_CLK_PLL_Z,
		[INV_CLK_EXTERNAL_32_KHZ]	= STR_INV_CLK_EXTERNAL_32_KHZ,
		[INV_CLK_EXTERNAL_19_MHZ]	= STR_INV_CLK_EXTERNAL_19_MHZ
	},
	[INV_MPU6500] = {
		[INV_CLK_INTERNAL]		= STR_INV_CLK_INTERNAL,
		[INV_CLK_PLL]			= STR_INV_CLK_PLL
	}
};

static const int inv_mpu6050_clock_sel_NR[INV_NUM_PARTS] = {
	[INV_MPU6050] = NUM_CLK_MPU6050,
	[INV_MPU6500] = NUM_CLK_MPU6500
};

static const bool inv_mpu6050_clock_sel_available[INV_NUM_PARTS][NUM_CLK] = {
	/* For MPU6050,
	* availability of clksel selector depending on clkin in inv_mpu6050_platform_data
	*/
	[INV_MPU6050] = {
		[INV_CLK_INTERNAL]		= false,	/* always available */
		[INV_CLK_PLL_X]			= false,
		[INV_CLK_PLL_Y]			= false,
		[INV_CLK_PLL_Z]			= false,
		[INV_CLK_EXTERNAL_32_KHZ]	= true,		/* depending on clkin */
		[INV_CLK_EXTERNAL_19_MHZ]	= true
	},
	/* For MPU6500,
	* clksel selector should be INV_CLK_INTERNAL or INV_CLK_PLL only.
	*/
	[INV_MPU6500] = {
		[INV_CLK_INTERNAL]		= false,	/* always available */
		[INV_CLK_PLL_X]			= false,
		[INV_CLK_PLL_Y]			= true,		/* always unvailable */
		[INV_CLK_PLL_Z]			= true,
		[INV_CLK_EXTERNAL_32_KHZ]	= true,
		[INV_CLK_EXTERNAL_19_MHZ]	= true
	}
};

static inline bool inv_mpu_clock_sel_available(struct inv_mpu6050_state *st,
					       enum inv_mpu6050_clock_sel_e sel)
{
	return !inv_mpu6050_clock_sel_available[st->chip_type][sel] ||
	       st->plat_data.clkin;
}

/**
 * inv_mpu6050_clksel_store() - Set clock source.
  */
static ssize_t inv_mpu6050_clksel_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	enum inv_mpu6050_clock_sel_e prev_clksel, new_clksel, i;
	int f;

	prev_clksel = new_clksel = st->chip_config.clksel;
	f=0;
	for (i = 0; i < inv_mpu6050_clock_sel_NR[st->chip_type]; i++)
	{
		if (sysfs_streq(inv_mpu6050_clock_sel_e_str[st->chip_type][i], buf)) {
			new_clksel = i;
			f = 1;
			break;
		}
	}

	if (!f)
		return -EINVAL;

	if (!inv_mpu_clock_sel_available(st, new_clksel))
		return -EINVAL;

	if (prev_clksel == new_clksel)
		return count;

	if (st->chip_config.enable)
		return -EBUSY;

	mutex_lock(&indio_dev->mlock);
	st->chip_config.clksel = new_clksel;
	mutex_unlock(&indio_dev->mlock);

	return count;
}

#define STR_INV_MPU6050_FSYNC_DISABLED		"disabled"
#define STR_INV_MPU6050_FSYNC_TEMP_OUT		"temp_out"
#define STR_INV_MPU6050_FSYNC_GYRO_XOUT		"gyro_x_out"
#define STR_INV_MPU6050_FSYNC_GYRO_YOUT		"gyro_y_out"
#define STR_INV_MPU6050_FSYNC_GYRO_ZOUT		"gyro_z_out"
#define STR_INV_MPU6050_FSYNC_ACCEL_XOUT	"accel_x_out"
#define STR_INV_MPU6050_FSYNC_ACCEL_YOUT	"accel_y_out"
#define STR_INV_MPU6050_FSYNC_ACCEL_ZOUT	"accel_z_out"

static const char *inv_mpu6050_ext_sync_set_e_str[INV_MPU6050_FSYNC_NR] = {
	[INV_MPU6050_FSYNC_DISABLED]	= STR_INV_MPU6050_FSYNC_DISABLED,
	[INV_MPU6050_FSYNC_TEMP_OUT]	= STR_INV_MPU6050_FSYNC_TEMP_OUT,
	[INV_MPU6050_FSYNC_GYRO_XOUT]	= STR_INV_MPU6050_FSYNC_GYRO_XOUT,
	[INV_MPU6050_FSYNC_GYRO_YOUT]	= STR_INV_MPU6050_FSYNC_GYRO_YOUT,
	[INV_MPU6050_FSYNC_GYRO_ZOUT]	= STR_INV_MPU6050_FSYNC_GYRO_ZOUT,
	[INV_MPU6050_FSYNC_ACCEL_XOUT]	= STR_INV_MPU6050_FSYNC_ACCEL_XOUT,
	[INV_MPU6050_FSYNC_ACCEL_YOUT]	= STR_INV_MPU6050_FSYNC_ACCEL_YOUT,
	[INV_MPU6050_FSYNC_ACCEL_ZOUT]	= STR_INV_MPU6050_FSYNC_ACCEL_ZOUT
};

/* For MPU6050, MPU6500,
 * availability of fsync selector depending on fsync in inv_mpu6050_platform_data
 */
static const bool inv_mpu6050_fsync_available[INV_MPU6050_FSYNC_NR] = {
	[INV_MPU6050_FSYNC_DISABLED]	= false, /* always available */
	[INV_MPU6050_FSYNC_TEMP_OUT]	= true,	 /* depending on fsync */
	[INV_MPU6050_FSYNC_GYRO_XOUT]	= true,
	[INV_MPU6050_FSYNC_GYRO_YOUT]	= true,
	[INV_MPU6050_FSYNC_GYRO_ZOUT]	= true,
	[INV_MPU6050_FSYNC_ACCEL_XOUT]	= true,
	[INV_MPU6050_FSYNC_ACCEL_YOUT]	= true,
	[INV_MPU6050_FSYNC_ACCEL_ZOUT]	= true
};

static inline bool inv_mpu_fsync_available(struct inv_mpu6050_state *st,
					   enum inv_mpu6050_ext_sync_set_e sel)
{
	return !inv_mpu6050_fsync_available[sel] || st->plat_data.fsync;
}

/**
 * inv_mpu6050_fsync_store() - Set external Frame Synchronization (FSYNC)
 */
static ssize_t inv_mpu6050_fsync_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct inv_mpu6050_state *st = iio_priv(indio_dev);
	enum inv_mpu6050_ext_sync_set_e prev_fsync, new_fsync, i;
	int f;

	prev_fsync = new_fsync = st->chip_config.fsync;
	f=0;
	for (i = 0; i < INV_MPU6050_FSYNC_NR; i++) {
		if (sysfs_streq(inv_mpu6050_ext_sync_set_e_str[i], buf)) {
			new_fsync = i;
			f = 1;
			break;
		}
	}

	if (!f)
		return -EINVAL;

	if (!inv_mpu_fsync_available(st, new_fsync))
		return -EINVAL;

	if (prev_fsync == new_fsync)
		return count;

	if (st->chip_config.enable)
		return -EBUSY;

	mutex_lock(&indio_dev->mlock);
	st->chip_config.fsync = new_fsync;
	mutex_unlock(&indio_dev->mlock);

	return count;
}

/**
 * inv_attr_show() - calling this function will show current
 *                    parameters.
 */
static ssize_t inv_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_mpu6050_state *st = iio_priv(dev_to_iio_dev(dev));
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	s8 *m;

	switch (this_attr->address) {
	/* In MPU6050, the two matrix are the same because gyro and accel
	   are integrated in one chip */
	case ATTR_GYRO_MATRIX:
	case ATTR_ACCL_MATRIX:
		m = (s8 *)st->plat_data.orientation;

		return sprintf(buf, "%d, %d, %d; %d, %d, %d; %d, %d, %d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
		break;
	/* Get the current clock source. */
	case ATTR_CLOCK_SOURCE:
		return sprintf(buf, "%s\n",
			       inv_mpu6050_clock_sel_e_str[st->chip_type][st->chip_config.clksel]);
		break;
	case ATTR_FRAME_SYNCHRONISATION:
		return sprintf(buf, "%s\n",
			       inv_mpu6050_ext_sync_set_e_str[st->chip_config.fsync]);
		break;

	case ATTR_CLOCK_SOURCE_AVAILABLE:
		{
			enum inv_mpu6050_clock_sel_e i;
			bool av;
			int c = 0;
			for (i = 0; i < NUM_CLK ; i++) {
				av = inv_mpu_clock_sel_available(st, i);
				if (av)
					c += sprintf(&buf[c], "%s ",
						     inv_mpu6050_clock_sel_e_str[st->chip_type][i]);
			}
			if (c>0) c--;
			c += sprintf(&buf[c], "\n");
			return c;
		}
		break;

	case ATTR_FRAME_SYNCHRONISATION_AVAILABLE:
		{
			enum inv_mpu6050_ext_sync_set_e i;
			bool av;
			int c = 0;
			for (i = 0; i < INV_MPU6050_FSYNC_NR ; i++) {
				av = inv_mpu_fsync_available(st, i);
				if (av)
					c += sprintf(&buf[c], "%s ", inv_mpu6050_ext_sync_set_e_str[i]);
			}
			if (c>0) c--;
			c += sprintf(&buf[c], "\n");
			return c;
		}
		break;
	case ATTR_GYRO_SCALE_AVAILABLE:
		{
			int c = 0, i;
			for (i = 0; i < ARRAY_SIZE(gyro_scale_6050); ++i) {
				c += sprintf(&buf[c], "0.%09d ", gyro_scale_6050[i]);
			}
			if (c>0) c--;
			c += sprintf(&buf[c], "\n");
			return c;
		}
		break;
	case ATTR_ACCEL_SCALE_AVAILABLE:
		{
			int c = 0, i;
			for (i = 0; i < ARRAY_SIZE(accel_scale); ++i) {
				c += sprintf(&buf[c], "0.%09d ", accel_scale[i]);
			}
			if (c>0) c--;
			c += sprintf(&buf[c], "\n");
			return c;
		}
		break;
	default:
		return -EINVAL;
	}
}

/**
 * inv_mpu6050_validate_trigger() - validate_trigger callback for invensense
 *                                  MPU6050 device.
 * @indio_dev: The IIO device
 * @trig: The new trigger
 *
 * Returns: 0 if the 'trig' matches the trigger registered by the MPU6050
 * device, -EINVAL otherwise.
 */
static int inv_mpu6050_validate_trigger(struct iio_dev *indio_dev,
					struct iio_trigger *trig)
{
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

#define INV_MPU6050_CHAN(_type, _channel2, _index)                    \
	{                                                             \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask_shared_by_type =  BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
		.scan_index = _index,                                 \
		.scan_type = {                                        \
				.sign = 's',                          \
				.realbits = 16,                       \
				.storagebits = 16,                    \
				.shift = 0 ,                          \
				.endianness = IIO_BE,                 \
			     },                                       \
	}

static const struct iio_chan_spec inv_mpu_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU6050_SCAN_TIMESTAMP),
	INV_MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_MPU6050_SCAN_GYRO_X),
	INV_MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_MPU6050_SCAN_GYRO_Y),
	INV_MPU6050_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_MPU6050_SCAN_GYRO_Z),
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
					| BIT(IIO_CHAN_INFO_SCALE)
					| BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = INV_MPU6050_SCAN_TEMP,
		.scan_type = {
				.sign = 's',
				.realbits = 16,
				.storagebits = 16,
				.shift = 0,
				.endianness = IIO_BE,
			     },
	},
	INV_MPU6050_CHAN(IIO_ACCEL, IIO_MOD_X, INV_MPU6050_SCAN_ACCL_X),
	INV_MPU6050_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_MPU6050_SCAN_ACCL_Y),
	INV_MPU6050_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_MPU6050_SCAN_ACCL_Z),
};


/* get/set sampling frequency */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("5 10 20 50 100 200 500 1000");
static IIO_DEV_ATTR_SAMP_FREQ(S_IRUGO | S_IWUSR,
			      inv_fifo_rate_show,
			      inv_mpu6050_fifo_rate_store);

/* get/set clock source */
static IIO_DEVICE_ATTR(clock_source_available,
		       S_IRUGO,
		       inv_attr_show,
		       NULL,
		       ATTR_CLOCK_SOURCE_AVAILABLE);
static IIO_DEVICE_ATTR(clock_source,
		       S_IRUGO | S_IWUSR,
		       inv_attr_show,
		       inv_mpu6050_clksel_store,
		       ATTR_CLOCK_SOURCE);

/* get/set frame synchronisation */
static IIO_DEVICE_ATTR(frame_synchronisation_available,
		       S_IRUGO,
		       inv_attr_show,
		       NULL,
		       ATTR_FRAME_SYNCHRONISATION_AVAILABLE);
static IIO_DEVICE_ATTR(frame_synchronisation,
		       S_IRUGO | S_IWUSR,
		       inv_attr_show,
		       inv_mpu6050_fsync_store,
		       ATTR_FRAME_SYNCHRONISATION);

/* show gyro(anglvel) and accel available scales */
static IIO_DEVICE_ATTR(in_anglvel_scale_available,
		       S_IRUGO,
		       inv_attr_show,
		       NULL,
		       ATTR_GYRO_SCALE_AVAILABLE);
static IIO_DEVICE_ATTR(in_accel_scale_available,
		       S_IRUGO,
		       inv_attr_show,
		       NULL,
		       ATTR_ACCEL_SCALE_AVAILABLE);

/* show gyro(anglvel) and accel orientation matrixes */
static IIO_DEVICE_ATTR(in_anglvel_matrix,
		       S_IRUGO,
		       inv_attr_show,
		       NULL,
		       ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(in_accel_matrix,
		       S_IRUGO,
		       inv_attr_show,
		       NULL,
		       ATTR_ACCL_MATRIX);

static struct attribute *inv_attributes[] = {
	&iio_dev_attr_in_anglvel_matrix.dev_attr.attr,
	&iio_dev_attr_in_accel_matrix.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_clock_source.dev_attr.attr,
	&iio_dev_attr_clock_source_available.dev_attr.attr,
	&iio_dev_attr_frame_synchronisation.dev_attr.attr,
	&iio_dev_attr_frame_synchronisation_available.dev_attr.attr,
	&iio_dev_attr_in_anglvel_scale_available.dev_attr.attr,
	&iio_dev_attr_in_accel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module		= THIS_MODULE,
	.read_raw		= &inv_mpu6050_read_raw,
	.write_raw		= &inv_mpu6050_write_raw,
	.write_raw_get_fmt	= &inv_mpu6050_write_raw_get_fmt,
	.attrs			= &inv_attribute_group,
	.validate_trigger	= inv_mpu6050_validate_trigger,
};

/**
 *  inv_check_and_setup_chip() - check and setup chip.
 */
static int inv_check_and_setup_chip(struct inv_mpu6050_state *st,
		const struct i2c_device_id *id)
{
	int result;

	st->chip_type	= (enum inv_devices)(id->driver_data);
	st->hw 		= &hw_info[st->chip_type];
	st->reg		= hw_info[st->chip_type].reg;

	if ( st->chip_type == INV_MPU6500 )
		st->plat_data.clkin= false;/* MPU6500 does not have CLKIN pin */

	/* reset to make sure previous state are not there */
	result = inv_mpu6050_write_reg(st, st->reg->pwr_mgmt_1,
					INV_MPU6050_BIT_H_RESET);
	if (result)
		return result;
	msleep(INV_MPU6050_POWER_UP_TIME);
	/* toggle power state. After reset, the sleep bit could be on
		or off depending on the OTP settings. Toggling power would
		make it in a definite state as well as making the hardware
		state align with the software state */
	result = inv_mpu6050_set_power_itg(st, false);
	if (result)
		return result;
	result = inv_mpu6050_set_power_itg(st, true);
	if (result)
		return result;

	result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_ACCL_STBY);
	if (result)
		return result;
	result = inv_mpu6050_switch_engine(st, false,
					INV_MPU6050_BIT_PWR_GYRO_STBY);
	if (result)
		return result;

	return 0;
}

/**
 *  inv_mpu_probe() - probe function.
 *  @client:          i2c client.
 *  @id:              i2c device id.
 *
 *  Returns 0 on success, a negative error code otherwise.
 */
static int inv_mpu_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct inv_mpu6050_state		*st;
	struct iio_dev				*indio_dev;
	struct inv_mpu6050_platform_data	*pdata;
	char 					*name;
	int result;

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENOSYS;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	pdata = dev_get_platdata(&client->dev);
	if (pdata)
		st->plat_data = *pdata;
	/* power is turned on inside check chip type*/
	result = inv_check_and_setup_chip(st, id);
	if (result)
		return result;

	result = inv_mpu6050_init_config(indio_dev);
	if (result) {
		dev_err(&client->dev,
			"Could not initialize device.\n");
		return result;
	}

	memset(st->name, 0, sizeof(st->name));
	/* id will be NULL when enumerated via ACPI */
	if (id)
		name = (char *)id->name;
	else
		name = (char *)dev_name(&client->dev);
	result = snprintf(st->name,
			  sizeof(st->name),
			  "%s.%d",
			  name,
			  st->plat_data.id);
	st->name[result] = '\0';

	i2c_set_clientdata(client, indio_dev);
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = inv_mpu_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_mpu_channels);
	indio_dev->name = st->name;
	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_BUFFER_TRIGGERED;

	result = iio_triggered_buffer_setup(indio_dev,
					    inv_mpu6050_irq_handler,
					    inv_mpu6050_read_fifo,
					    NULL);
	if (result) {
		dev_err(&st->client->dev, "configure buffer fail %d\n",
				result);
		return result;
	}
	result = inv_mpu6050_probe_trigger(indio_dev);
	if (result) {
		dev_err(&st->client->dev, "trigger probe fail %d\n", result);
		goto out_unreg_ring;
	}

	INIT_KFIFO(st->timestamps);
	spin_lock_init(&st->time_stamp_lock);
	result = devm_iio_device_register(&st->client->dev, indio_dev);
	if (result) {
		dev_err(&st->client->dev, "IIO register fail %d\n", result);
		goto out_remove_trigger;
	}

	dev_info(&st->client->dev,
		 "Invensense MPU6050 6-axis gyroscope/accelometer (%s)%s%s\n",
		 indio_dev->name,
		 st->plat_data.fsync?" FSYNC":"",
		 st->plat_data.clkin?" CLKIN":"");

	return 0;

out_remove_trigger:
	inv_mpu6050_remove_trigger(st);
out_unreg_ring:
	iio_triggered_buffer_cleanup(indio_dev);
	return result;
}

static int inv_mpu_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_mpu6050_state *st = iio_priv(indio_dev);

	kfree(st->rx_buffer);
	kfree(st->rx_buffer_ts);
	devm_iio_device_unregister(&client->dev, indio_dev);
	inv_mpu6050_remove_trigger(st);
	iio_triggered_buffer_cleanup(indio_dev);

	return 0;
}
#ifdef CONFIG_PM_SLEEP

static int inv_mpu_resume(struct device *dev)
{
	return inv_mpu6050_set_power_itg(
		iio_priv(i2c_get_clientdata(to_i2c_client(dev))), true);
}

static int inv_mpu_suspend(struct device *dev)
{
	return inv_mpu6050_set_power_itg(
		iio_priv(i2c_get_clientdata(to_i2c_client(dev))), false);
}
static SIMPLE_DEV_PM_OPS(inv_mpu_pmops, inv_mpu_suspend, inv_mpu_resume);

#define INV_MPU6050_PMOPS (&inv_mpu_pmops)
#else
#define INV_MPU6050_PMOPS NULL
#endif /* CONFIG_PM_SLEEP */

/*
 * device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_mpu_id[] = {
	{"mpu6050", INV_MPU6050},
	{"mpu6500", INV_MPU6500},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_mpu_id);

static struct i2c_driver inv_mpu_driver = {
	.probe		=	inv_mpu_probe,
	.remove		=	inv_mpu_remove,
	.id_table	=	inv_mpu_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv-mpu6050",
		.pm     =       INV_MPU6050_PMOPS,
	},
};

module_i2c_driver(inv_mpu_driver);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device MPU6050 driver");
MODULE_LICENSE("GPL");
