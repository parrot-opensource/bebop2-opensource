#ifndef _AK8975_REGS_H_
#define _AK8975_REGS_H_
/*
 * A sensor driver for the magnetometer AK8975/AK8963.
 *
 * Magnetic compass sensor driver for monitoring magnetic flux information.
 *
 * Copyright (c) 2010, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA.
 */
/*
 * Register definitions, as well as various shifts and masks to get at the
 * individual fields of the registers.
 */
/* Register WIA: Device ID */
#define AK8975_REG_WIA			0x00
#define AK8975_DEVICE_ID		0x48

/* Register INFO: Information */
#define AK8975_REG_INFO			0x01

/* Register ST1: Status 1 */
#define AK8975_REG_ST1			0x02

/* DRDY: Data Ready for AK8975/AK8963 */
#define AK8975_REG_ST1_DRDY_SHIFT	0
#define AK8975_REG_ST1_DRDY_MASK	(1 << AK8975_REG_ST1_DRDY_SHIFT)

/* DRDY: Data Ready for AK8963 */
#define AK8963_REG_ST1_DOR_SHIFT	1
#define AK8963_REG_ST1_DOR_MASK		(1 << AK8975_REG_ST1_DOR_SHIFT)

/* Register HXL to HZH: Measurement Data */
#define AK8975_REG_HXL			0x03
#define AK8975_REG_HXH			0x04
#define AK8975_REG_HYL			0x05
#define AK8975_REG_HYH			0x06
#define AK8975_REG_HZL			0x07
#define AK8975_REG_HZH			0x08

/* Register ST2: Status 2 */
#define AK8975_REG_ST2			0x09

/* DERR: Data Error for AK8975 */
#define AK8975_REG_ST2_DERR_SHIFT	2
#define AK8975_REG_ST2_DERR_MASK	(1 << AK8975_REG_ST2_DERR_SHIFT)

/* HOFL: Magnetic sensor overflow for AK8975/AK8963 */
#define AK8975_REG_ST2_HOFL_SHIFT	3
#define AK8975_REG_ST2_HOFL_MASK	(1 << AK8975_REG_ST2_HOFL_SHIFT)

/* BITM: Output bit setting (mirror) for AK8963 */
#define AK8963_REG_ST2_BITM_SHIFT	4
#define AK8963_REG_ST2_BITM_MASK	(1 << AK8975_REG_ST2_BITM_SHIFT)

/* Register CNTL: Control for AK8975 */
/* Register CNTL1: Control 1 for AK8963 */
#define AK8975_REG_CNTL			0x0A
/* MODE[3:0]: Operation mode setting for AK8975/AK8963 */
#define AK8975_REG_CNTL_MODE_SHIFT	0
#define AK8975_REG_CNTL_MODE_MASK	(0xF << AK8975_REG_CNTL_MODE_SHIFT)
#define AK8975_REG_CNTL_MODE_POWER_DOWN	0
#define AK8975_REG_CNTL_MODE_ONCE	1
#define AK8975_REG_CNTL_MODE_SELF_TEST	8
#define AK8975_REG_CNTL_MODE_FUSE_ROM	0xF
/* MODE[3:0]: Operation mode setting for AK8963 */
#define AK8963_REG_CNTL_MODE_CONTINUOUS_1	2
#define AK8963_REG_CNTL_MODE_CONTINUOUS_2	6
#define AK8963_REG_CNTL_MODE_EXTERNAL_TRIGGER	4

/* BIT: Output bit setting for AK8963 */
#define AK8963_REG_CNTL_BIT_SHIFT	4
#define AK8963_REG_CNTL_BIT_MASK	(1 << AK8963_REG_CNTL_BIT_SHIFT)
#define AK8963_REG_CNTL_BIT_14BITS	0
#define AK8963_REG_CNTL_BIT_16BITS	1

/* Register CNTL2: Control 2 for AK8963 */
#define AK8963_REG_CNTL2		0x0B
/* SRST: Soft reset */
#define AK8963_REG_CNTL2_SRST_SHIFT	0
#define AK8963_REG_CNTL2_SRST_MASK	(0x1 << AK8963_REG_CNTL2_SRST_SHIFT)
#define AK8963_REG_CNTL2_SRST_NORMAL	0
#define AK8963_REG_CNTL2_SRST_RESET	1

/* Register RSV: Reserved for AK8975 */
#define AK8975_REG_RSVC			0x0B

/* Register ASTC: Self Test Control for AK8975/AK8963 */
#define AK8975_REG_ASTC			0x0C
#define AK8975_REG_ASTC_SELF_SHIFT	0
#define AK8975_REG_ASTC_SELF_MASK	(0x1 << AK8975_REG_ASTC_SELF_SHIFT)
#define AK8975_REG_ASTC_SELF_NORMAL	0
#define AK8975_REG_ASTC_SELF_GENERATE	1

/* Registers TS1, TS2: Test 1, 2 for AK8975/AK8963 */
#define AK8975_REG_TS1			0x0D
#define AK8975_REG_TS2			0x0E

/* Register I2CDIS: I2C Disable for AK8975/AK8963 */
#define AK8975_REG_I2CDIS		0x0F

/* Registers ASAX, ASAY, ASAZ: Sensitivity Adjustment values for AK8975/AK8963 */
#define AK8975_REG_ASAX			0x10
#define AK8975_REG_ASAY			0x11
#define AK8975_REG_ASAZ			0x12

#define AK8975_MAX_REGS			AK8975_REG_ASAZ

/*
 * Miscellaneous values.
 */
#define AK8975_MAX_CONVERSION_TIMEOUT	500
#define AK8975_CONVERSION_DONE_POLL_TIME 10
#define AK8975_DATA_READY_TIMEOUT	((100*HZ)/1000)

#endif /* _AK8975_REGS_H_ */

