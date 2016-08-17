/*
 * tc358746a - Toshiba device TC358746A CSI2.0 <-> Parallel bridge
 *
 * It can perform protocol conversion in both direction, MIPI to parallel or
 * parallel to MIPI.
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Wed Jul  2 09:16:13 CEST 2014
 *
 */
#ifndef __TC358746A_H__
#define __TC358746A_H__

#include <linux/v4l2-mediabus.h>

struct tc358746a_platform_data {
	int (*set_power)(int on);

	/* External clock in Hz */
	unsigned long refclk;
	/* Numer of CSI-2 lanes */
	unsigned int lanes;
	/* Used to force a different pixelcode for the bridge. By default it
	 * copies the code from the sensor subdevice  */
	enum v4l2_mbus_pixelcode force_subdev_pixcode;
	/* Length of the calibration delay loop (0 means default)*/
	unsigned                 calibration_delay_ms;
	/* If different from zero the bridge will use this value directly
	 * instead of using the (slow) calibration loop */
	unsigned                 phytimdly;
};

#define TC358746A_I2C_ADDR  0x0e
#define TC358746A_POWER_ON     1
#define TC358746A_POWER_OFF    0

#endif /* __TC358746A_H__ */
