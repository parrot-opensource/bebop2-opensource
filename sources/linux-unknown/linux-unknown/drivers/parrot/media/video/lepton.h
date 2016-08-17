
#ifndef _LEPTON_H_
#define _LEPTON_H_

#include <linux/i2c.h>
#include <media/videobuf2-core.h>

struct lepton_platform_data {
	struct i2c_board_info   board_info;
	int                     i2c_adapter_nr;
	enum vb2_cache_flags    vb2_cache_flags;
};

struct lepton_i2c_platform_data {
	int     gpio_pwr;
	int     gpio_rst;
};
#endif /* _LEPTON_H_ */
