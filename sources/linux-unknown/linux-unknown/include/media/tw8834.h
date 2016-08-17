/*
 * Header for tw8834 driver
 *
 *
 * Author : Julien BERAUD
 *
 *
 */
#ifndef __TW8834_H__
#define __TW8834_H__

struct tw8834_platform_data {
	int (*set_power)(int on);
};

enum {
	TW8834_POWER_OFF,
	TW8834_POWER_ON,
	TW8834_POWER_INPUT_DISABLE,
	TW8834_POWER_INPUT_ENABLE,
};
#endif
