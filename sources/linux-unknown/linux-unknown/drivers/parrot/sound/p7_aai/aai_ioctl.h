
#ifndef _AAI_IOCTL_H_
#define _AAI_IOCTL_H_

#include <linux/ioctl.h>

/*
 * Music-in configuration
 */
struct aai_ioctl_music_conf_t {
	int	cycles_num;
	int	matsushita;
	int	left_frame;
	int	lsb_first;
	int	right_just;
	int	bits24;
};
#define AAI_MUS0_CONF			_IOW('p',  1, int)
#define AAI_MUS1_CONF			_IOW('p',  2, int)
#define AAI_MUS2_CONF			_IOW('p',  3, int)
#define AAI_MUS3_CONF			_IOW('p',  4, int)

/*
 * TDM configuration
 */
struct aai_ioctl_tdm_conf_t {
	int	cycles_num;
	int	left_frame;
};
#define AAI_TDMIN_CONF			_IOW('p',  5, int)
#define AAI_TDMOUT_CONF			_IOW('p',  6, int)

/*
 * ADC, DAC & AUX configuration
 */
struct aai_ioctl_adc_conf_t {
	int	cycles_num;
	int	matsushita;
	int	left_frame;
	int	half_frame;
};
#define AAI_ADC_CONF			_IOW('p',  7, int)
#define AAI_DAC_CONF			_IOW('p',  8, int)
#define AAI_AUX_CONF			_IOW('p',  9, int)

/*
 * PCM configuration
 */
struct aai_ioctl_pcm_conf_t {
	int	start_two;
	int	cycles_pcm_high;
	int	cycles_pcm_low;
	int	low;
	int	oki;
	int	run_dual;
};
#define AAI_PCM1_CONF			_IOW('p', 10, int)
#define AAI_PCM2_CONF			_IOW('p', 11, int)

/*
 * Voice configuration
 */
#define AAI_VOICE_SPEED			_IOW('p', 12, int)
#define AAI_VOICE8K_SPEAKER		_IOW('p', 13, int)
#define AAI_VOICE8K_PHONE		_IOW('p', 14, int)

/*
 * Sync group controls
 */
struct aai_ioctl_group_t {
	char	*name;
	int	group;
};
#define AAI_GROUP_START			_IOW('p', 15, int)
#define AAI_GROUP_SYNC			_IOW('p', 16, int)

/*
 * ASYNC configuration
 */
struct aai_ioctl_asym_conf_t {
	int	enable;
	short	div_high;
	short	div_low;
};
#define AAI_MFRAME_ASYMMETRIC_CONF	_IOW('p', 19, int)
#define AAI_HFRAME_ASYMMETRIC_CONF	_IOW('p', 20, int)

#endif

