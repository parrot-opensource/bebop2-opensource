/*
 * Header for ov7740 driver
 *
 *
 * Author : Maxime JOURDAN
 *
 *
 */
#ifndef __OV7740_H__
#define __OV7740_H__

struct ov7740_platform_data {
	int (*set_power)(int on);
};

#endif