#ifndef __OV16825_H__
#define __OV16825_H__

#define OV16825_I2C_ADDR  0x36

struct ov16825_platform_data {
	int (*set_power)(int on);

	u32 ext_clk;
	unsigned int  lanes;     /* Numer of CSI-2 lanes */
};

#endif /* __OV16825_H__ */
