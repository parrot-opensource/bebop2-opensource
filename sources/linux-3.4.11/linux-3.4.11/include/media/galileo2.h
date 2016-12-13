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
#ifndef __GALILEO2_H__
#define __GALILEO2_H__

struct galileo2_platform_data {
	int (*set_power)(int on);

	unsigned long refclk;
	unsigned int  lanes;     /* Numer of CSI-2 lanes */
};

#define GALILEO2_POWER_ON     1
#define GALILEO2_POWER_OFF    0

#endif /* __GALILEO2_H__ */
