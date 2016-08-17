#ifndef __OV16825_CLK__H__
#define __OV16825_CLK__H__

struct ov16825_pll {
	unsigned char prediv;
	unsigned char multi;

	unsigned char op_sys;
	unsigned char op_pix;
	unsigned char op_2lane;

	unsigned char vt_sys;
	unsigned char vt_pix;
};

struct ov16825_pll2 {
	unsigned char prediv;
	unsigned char rdiv;
	unsigned char multi;
	unsigned char divr;
	unsigned char seld5;
};

struct ov16825_reg_conf {
	struct ov16825_pll              pll;
	struct ov16825_pll2             pll2;
	struct ov16825_pll2             adc;

	/* External */
	unsigned char r3020;
	unsigned char r3030;
	unsigned char r3032;
	unsigned char r3033;
	unsigned char r3106;
};

void ov16825_print_configuration(struct ov16825_reg_conf *c);

int ov16825_current_configuration(struct v4l2_subdev *sd,
                                  struct ov16825_reg_conf *c);

int ov16825_print_clocks(u32 refclk,
                         struct ov16825_reg_conf *c);

int ov16825_set_phy_pclk(u32 refclk,
                         u32 target_clk,
                         struct ov16825_reg_conf *c);

int ov16825_set_pclk(u32 phy_pclk,
                     u32 target_clk,
                     struct ov16825_reg_conf *c);

int ov16825_set_daccclk(u32 refclk,
                        u32 dacclk,
                        struct ov16825_reg_conf *c);

int ov16825_set_pll2_sclk(u32 refclk,
                          u32 target_clk,
                          struct ov16825_reg_conf *c);

int ov16825_apply_configuration(struct v4l2_subdev *sd,
                                struct ov16825_reg_conf *c);

#endif //__OV16825_CLK__H__
