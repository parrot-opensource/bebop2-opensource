
#ifndef _BLDC_CYPRESS_PLATFORM_H_
#define _BLDC_CYPRESS_PLATFORM_H_


/**
 * struct bldc_cypress_platform_data - Platform data for the bldc 
 * cypress driver
 *
 * @lut: Motors look up table ex: {0, 1, 2, 3}
 * @spin_dir: Motors spin direction ex: 0b1010, which is CCW/CW/CCW/CW
 */
struct bldc_cypress_platform_data {
	u8	lut[4];
	u8	spin_dir;
};

#endif
