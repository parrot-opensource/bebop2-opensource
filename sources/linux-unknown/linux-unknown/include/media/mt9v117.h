/*
 * Header for mt9v117 driver
 *
 *
 * Author : Julien BERAUD
 *
 *
 */
#ifndef __MT9V117_H__
#define __MT9V117_H__

struct mt9v117_platform_data {
	unsigned int	ext_clk_freq_hz;
	int (*set_power)(int on);
};

#define MT9V117_POWER_ON 1
#define MT9V117_POWER_OFF 0

#endif
