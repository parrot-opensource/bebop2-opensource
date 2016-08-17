/*
 * Header for tw9990 driver
 *
 *
 * Author : Julien BERAUD
 *
 *
 */
#ifndef __TW9990_H__
#define __TW9990_H__

struct tw9990_platform_data {
	int (*set_power)(int on);
	int differential_input;
	int anti_aliasing;
	int power_saving;
	int synct_mode; /* default or forced (forced use sync pulse value) */
	int sync_pulse_amplitude; /* default=56 range=[127..0] */
	int common_mode_clamp;
};

/* Differential input flags */
#define TW9990_DIFFERENTIAL_ENABLED 1
#define TW9990_DIFFERENTIAL_DISABLED 0

/* Anti-aliasing flags */
#define TW9990_ANTI_ALIAS_ENABLED 1
#define TW9990_ANTI_ALIAS_DISABLED 0

/* power saving flags */
#define TW9990_POWER_SAVE_ENABLED 1
#define TW9990_POWER_SAVE_DISABLED 0

/* Synct mode flags */
#define TW9990_SYNCT_DEFAULT 1
#define TW9990_SYNCT_FORCED 0

/* Common Mode Clamp flags */
#define TW9990_CM_TOGGLE (0x40)
#define TW9990_CM_CUR_MODE (0x20)
#define TW9990_CM_RSTR_FULL_TIME (0x10)
#define TW9990_CM_RESTORE (0x04)

#define TW9990_POWER_ON 1
#define TW9990_POWER_OFF 0

#endif
