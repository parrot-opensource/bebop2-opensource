/*
 * mt9m021 - Aptina CMOS Digital Image Sensor
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Tue Aug 26 15:55:49 CEST 2014
 *
 */
#ifndef __MT9M021_H__
#define __MT9M021_H__

struct mt9m021_platform_data {
	int (*set_power)(int on);

	u32  ext_clock;
	u32  pix_clock;
	bool no_soft_reset;
};

#endif /* __MT9M021_H__ */
