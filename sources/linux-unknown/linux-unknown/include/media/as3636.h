/*
 * include/media/as3636.h
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * Contact: Maxime Jourdan <maxime.jourdan@parrot.com>
 *
 * Inspired from AS3636 driver made by Laurent Pinchart
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

#ifndef __AS3636_H__
#define __AS3636_H__

#include <media/v4l2-subdev.h>

#define AS3636_NAME                             "as3636"
#define AS3636_I2C_ADDR                         (0x28)

#define V4L2_CID_PRIVATE_AS3636_TEST 			(V4L2_CID_FLASH_CLASS_BASE + 0x99)
#define V4L2_CID_PRIVATE_AS3636_STDBY 			(V4L2_CID_FLASH_CLASS_BASE + 0x100)
#define V4L2_CID_PRIVATE_AS3636_SET_REGISTER 		(V4L2_CID_FLASH_CLASS_BASE + 0x101)
#define V4L2_CID_PRIVATE_AS3636_GET_CONTROL 		(V4L2_CID_FLASH_CLASS_BASE + 0x102)
#define V4L2_CID_PRIVATE_AS3636_GET_INTERRUPT_STATUS 	(V4L2_CID_FLASH_CLASS_BASE + 0x103)
#define V4L2_CID_PRIVATE_AS3636_GET_XENON_CONTROL 	(V4L2_CID_FLASH_CLASS_BASE + 0x104)

/* as3636_platform_data - Flash controller platform data
 * @set_power:	Set power callback
 * @xenon_pulse_length: [us] 1-255 ; 0 = not used
 * @switch_selection: [mA] 375-900, 75mA step
 * @charge_max_voltage: [dV] 285-348 Capacitor's max charge voltage, 285 = 28.5V
 * @dcdc_peak: [mA] 250-400, 50mA step. DCDC Boost Coil Peak current setting 
 * @auto_charge: [bool] Automatic charging of the capacitor after a flash
 */
struct as3636_platform_data {
	int (*set_power)(struct v4l2_subdev *subdev, int on);
	u8 xenon_pulse_length;
	u16 switch_selection;
	u16 charge_max_voltage;
	u16 dcdc_peak;
	u8 auto_charge;
};

#endif /* __AS3636_H__ */
