/*
 * mt9m021_reg - Aptina CMOS Digital Image Sensor registers
 *
 * Author : Maxime JOURDAN <maxime.jourdan@parrot.com>
 *
 * Date : 12/03/2015
 *
 */
#ifndef __MT9M021_REG_H__
#define __MT9M021_REG_H__

union ae_ctrl_reg {
	struct {
		u8 ae_enable    : 1;
		u8 auto_ag_en   : 1;
		u8 reserved     : 2;
		u8 auto_dg_en   : 1;
		u8 min_ana_gain : 2;
	};
	
	u16 _register;
};

union dig_test_reg {
	struct {
		u8 reserved     : 4;
		u8 col_gain     : 2;
		u8 reserved_2   : 1;
		u8 mono_chrome  : 1;
	};
	
	u16 _register;
};

#endif /* __MT9M021_REG_H__ */
