/*
 * include/media/max14574.h
 *
 * Copyright (C) 2016 Parrot S.A.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __MAX14574_H__
#define __MAX14574_H__

#include <media/v4l2-subdev.h>

#define MAX14574_NAME		"max14574"
#define MAX14574_I2C_ADDR	(0x77)

/******** MAX14574 Registers ********* */
#define	M_STATUS		0x00
#define	STATUS_LL_TH		(1 << 4)
#define	STATUS_BST_FAIL		(1 << 2)

#define FAIL			0x01
#define FAIL_OP			(1 << 6)
#define FAIL_SH			(1 << 5)
#define FAIL_COM_FAIL		(1 << 4)
#define FAIL_LL4_FAIL		(1 << 3)
#define FAIL_LL3_FAIL		(1 << 2)
#define FAIL_LL2_FAIL		(1 << 1)
#define FAIL_LL1_FAIL		(1 << 0)

#define T_SENSE			0x02

#define USERMODE	        0x03
#define USERMODE_ACTIVE		(1 << 1)
#define USERMODE_SM		(1 << 0)

#define OIS_LSB		        0x04
#define LLV1		        0x05
#define LLV2		        0x06
#define LLV3		        0x07
#define LLV4		        0x08

#define CMND		        0x09
#define CMND_UPD_OUT		(1 << 1)
#define CMND_CHK_FAIL		(1 << 0)

#define DRIVERCONF	        0x0A
#define DRIVERCONF_TS_INT	(1 << 7)
#define DRIVERCONF_TS_EXT	(1 << 6)
#define DRIVERCONF_FSTU1	(1 << 3)
#define DRIVERCONF_FSTU0	(1 << 2)
#define DRIVERCONF_IL1		(1 << 1)
#define DRIVERCONF_IL0		(1 << 0)

/* Definition of min and max voltage : 24,4V and 70V */
#define CHARGE_MAX_VOLT_MIN 0
#define CHARGE_MAX_VOLT_MAX 1023

/* Definition of V4L2 controls for the MAX14574 chip */
#define	V4L2_CID_VARIOPTIC_LENS	(V4L2_CID_CAMERA_CLASS_BASE + 0x60)
#define	V4L2_CID_PRIVATE_MAX14574_CONTROL_POSITION	\
	(V4L2_CID_CAMERA_CLASS_BASE + 0x61)
#define	V4L2_CID_PRIVATE_MAX14574_GET_INTERRUPT_STATUS	\
	(V4L2_CID_CAMERA_CLASS_BASE + 0x62)
#define	V4L2_CID_PRIVATE_MAX14574_GET_MAIN_STATUS	\
	(V4L2_CID_CAMERA_CLASS_BASE + 0x63)


/* max14574_platform_data - Lens controller platform data
 * @set_power: Set power callback
 * @mode: set to (USERMODE_ACTIVE | USERMODE_SM) to wake-up the driver
 * @control: set to CMND_UPD_OUT for setting LLVx to CHARGE_MAX_VOLT_MIN value
 * @voltage: [CHARGE_MAX_VOLT_MIN;CHARGE_MAX_VOLT_MAX],
 * default: set to CHARGE_MAX_VOLT_MIN

 */
struct max14574_platform_data {
	int 	(*set_power)(struct v4l2_subdev *subdev, int on);
	u8	mode;
	u8	control;
	u16	voltage;
};

#endif /* __MAX14574_H__ */
