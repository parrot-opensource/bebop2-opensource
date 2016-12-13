/*
 * ar1820 - Aptina CMOS Digital Image Sensor
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Tue Aug 26 15:55:49 CEST 2014
 *
 */
#ifndef __AR1820_H__
#define __AR1820_H__

struct ar1820_platform_data {
	int (*set_power)(int on);

	u32 ext_clk;
};

#endif /* __AR1820_H__ */
