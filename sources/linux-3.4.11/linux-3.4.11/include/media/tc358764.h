#ifndef _TC358764_H_
#define _TC358764_H_

struct tc358764_pdata {
	// Called with 1 to power on, 0 to power off
	int (*set_power)(int);
};

#endif /* _TC358764_H_ */
