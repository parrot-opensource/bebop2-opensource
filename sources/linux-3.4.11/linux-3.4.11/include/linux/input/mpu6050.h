/*
 * INVENSENSE MPU6050 driver
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Kishore Kadiyala <kishore.kadiyala@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _LINUX_MPU6050_H
#define _LINUX_MPU6050_H

#define MPU6050_NAME		"mpu6050"

/* Sensor features enable flags */
#define MPU6050_PASS_THROUGH_EN	(1 << 0)

/* Accelerometer operational modes */
#define MPU605_MODE_FF		0  /*Free Fall mode*/
#define MPU605_MODE_MD		1  /*Motion Detection mode*/
#define MPU605_MODE_ZD		2  /*Zero Motion Detection mode*/

/* Accelerometer full scale range */
#define MPU6050_RANGE_2G	0
#define MPU6050_RANGE_4G	1
#define MPU6050_RANGE_8G	2
#define MPU6050_RANGE_16G	3

/* Gyroscope full scale range */
#define MPU6050_GYRO_FSR_250	0
#define MPU6050_GYRO_FSR_500	1
#define MPU6050_GYRO_FSR_1000	2
#define MPU6050_GYRO_FSR_2000	3

/**
 * struct mpu6050_gyro_platform_data - MPU6050 Platform data
 * @aux_i2c_supply: Auxiliary i2c bus voltage supply level
 * @sample_rate_div: Samplerate of MPU6050
 * @config: fsync and DLPF config for accel
 * @fifo_mode: FIFO Mode enable/disable
 * @flags: sensor feature flags
 * @mpu6050_accel: mpu6050 accel platform data
 * @mpu6050_gyro: mpu6050 gyro platform data
 */

struct mpu6050_platform_data {
	uint8_t fifo_mode;
	uint8_t flags;
	int x_axis;
	int y_axis;
	int z_axis;
	uint8_t accel_fsr;
	uint8_t gyro_fsr;
};

#endif
