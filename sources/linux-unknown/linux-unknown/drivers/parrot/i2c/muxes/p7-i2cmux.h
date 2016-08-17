#ifndef _P7_I2CMUX_H_
#define _P7_I2CMUX_H_

#define P7I2CMUX_DRV_NAME "p7-i2cmux"

struct p7i2cmux_plat_data {
	/* ID of the parent I2C adapter */
	int                      parent;
	/* base adapter ID we wish to use for the mux channel adapters. 0 to
	 * auto-assign. */
	unsigned                 base;
	/* name of the channels, used to fetch the pin configuration from
	 * pinctrl */
	const char * const      *channel_names;
	/* number of channels */
	size_t                   nr_channels;
};

#endif /* _P7_I2CMUX_H_ */
