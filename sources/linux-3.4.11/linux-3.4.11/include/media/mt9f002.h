/*
 * Header for mt9f002 driver
 *
 *
 * Author : Karl Leplat
 *
 *
 */
#ifndef __MT9F002_H__
#define __MT9F002_H__

 enum mt9f002_interface {
          MT9F002_Parallel,
          MT9F002_MIPI,
          MT9F002_HiSPi,
 };

 enum mt9f002_pixel_depth {
          MT9F002_8bits = 8,
          MT9F002_10bits = 10,
          MT9F002_12bits = 12,
 };

enum mt9f002_lane {
	MT9F002_1lane = 1,
	MT9F002_2lane = 2,
	MT9F002_3lane = 3,
	MT9F002_4lane = 4,
};

struct mt9f002_platform_data {
	enum mt9f002_interface 		interface;
	enum mt9f002_pixel_depth	pixel_depth;
	enum mt9f002_lane		number_of_lanes;
	uint64_t			ext_clk_freq_mhz;
	uint64_t			output_clk_freq_mhz;
	int (*set_power)(int on);
};

#define MT9F002_POWER_ON 1
#define MT9F002_POWER_OFF 0

#endif
