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

/* V4L2 specific controls for MT9V117 */
#define MT9V117_CID_AUTO_EXPOSURE_ZONE	(V4L2_CID_CAMERA_CLASS_BASE + 0x1001)
enum mt9v117_auto_exposure_zone_type {
	MT9V117_AUTO_EXPOSURE_BRIGHT = 0,
	MT9V117_AUTO_EXPOSURE_NORMAL = 1,
	MT9V117_AUTO_EXPOSURE_DARK = 2,
};

struct mt9v117_platform_data {
	unsigned int	ext_clk_freq_hz;
	int		enable_bt656;
	int (*set_power)(int on);
};

#define MT9V117_POWER_ON 1
#define MT9V117_POWER_OFF 0

#define MT9V117_REG_HOST_COMMAND (0x0040)
#define MT9V117_HOST_COMMAND_0   (1<<0)
#define MT9V117_HOST_COMMAND_1   (1<<1)
#define MT9V117_HOST_COMMAND_2   (1<<2)
#define MT9V117_HOST_COMMAND_OK  (1<<15)


#define MT9V117_VAR_AE_TRACK_ALGO  10,4
#define MT9V117_VAR_AWB_ALGO       11,4

#define MT9V117_VAR_CAM_INPUT_SOURCE               18,64
#define MT9V117_VAR_CAM_INPUT_TEST_PATTERN_SELECT  18,65
#define MT9V117_VAR_CAM_INPUT_TEST_PATTERN_RED     18,66
#define MT9V117_VAR_CAM_INPUT_TEST_PATTERN_GREEN   18,67
#define MT9V117_VAR_CAM_INPUT_TEST_PATTERN_BLUE    18,68

#define MT9V117_VAR_SYSMGR_NEXT_STATE  23,0

#endif
