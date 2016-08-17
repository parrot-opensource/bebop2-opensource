/**
 ************************************************
 * @file mt9f002_regs.h
 * @brief Driver for Aptina mt9f002
 *
 * Copyright (C) 2013 Parrot S.A.
 *
 * @author Karl Leplat <karl.leplat@parrot.com>
 * @date 2014-09-19
 ************************************************
 */

#ifndef MT9F002_REG_H
#define MT9F002_REGS_H

//#define HAL_MT9F002_BINNING 1
/*
 * PLL constraint (see figure - MTF9002_DS_D.pdf p38), in KHZ
 */
// frequencies range in mHZ
#define EXT_CLK_MIN 		2000000000UL   // camera input clock min value (symbol ext_clk_freq_mhz)
#define EXT_CLK_MAX 		64000000000UL    // camera input clock max value (symbol ext_clk_freq_mhz)

#define PLL_INPUT_CLK_MIN 	2000000000UL // pll input clock min value (symbol pll_ip_clk_freq)
#define PLL_INPUT_CLK_MAX 	24000000000UL  // pll input clock max value (symbol pll_ip_clk_freq)

#define VCO_CLK_MIN 		384000000000UL   // vco clock min value (symbol frequency)
#define VCO_CLK_MAX 		768000000000UL  // vco clock max value (symbol frequency)

#define PIXEL_CLK_MAX 		120000000000UL

// pll registers range
#define PRE_PLL_CLK_DIV_MIN 1
#define PRE_PLL_CLK_DIV_MAX 64

#define PLL_MUL_EVEN_MIN 32
//#define PLL_MUL_EVEN_MAX 384 // TODO according to datasheet max value is 384 but in fact it seems to be 254
#define PLL_MUL_EVEN_MAX 254
#define PLL_MUL_ODD_MIN 17
#define PLL_MUL_ODD_MAX 191
#define PLL_MUL_MIN PLL_MUL_ODD_MIN
#define PLL_MUL_MAX PLL_MUL_EVEN_MAX

#define LINE_LENGTH_MAX  0xFFFF
#define FRAME_LENGTH_MAX 0xFFFF

/*
* Scaler 1:1 (m = 16) to 1:8 (m = 128)
*/
#define SCALER_N 16
#define SCALER_M_MIN 16
#define SCALER_M_MAX 128

#define MT9F002_PIXEL_ARRAY_HEIGHT			3351
#define MT9F002_PIXEL_ARRAY_WIDTH			4647

#define MT9F002_CHIP_VERSION				0x0000
#define		MT9F002_CHIP_ID				0x2E01
#define MT9F002_RESET_REGISTER				0x301A
#define MT9F002_GLOBAL_GAIN				0x305E
#define		MT9F002_GLOBAL_GAIN_MIN			15
#define		MT9F002_GLOBAL_GAIN_MAX			635
#define		MT9F002_GLOBAL_GAIN_DEF			20
#define MT9F002_DAC_LD_14_15				0x3EDA
#define MT9F002_DAC_LD_28_29				0x3EE8
#define MT9F002_DAC_LD_24_25				0x3EE4
#define MT9F002_DAC_LD_30_31				0x3EEA
#define MT9F002_COLUMN_CORRECTION			0x30D4
#define MT9F002_CTX_CONTROL_REG				0x30E8
#define MT9F002_CTX_WR_DATA_REG				0x30EA
#define MT9F002_ANALOG_CONTROL4				0x3176
#define MT9F002_DATA_PEDESTAL_				0x301E
#define MT9F002_SERIAL_FORMAT				0x31AE
#define MT9F002_DATAPATH_SELECT				0x306E
#define MT9F002_SCALING_MODE				0x0400
#define MT9F002_SMIA_TEST				0x3064
#define MT9F002_VT_PIX_CLK_DIV				0x0300
#define MT9F002_VT_SYS_CLK_DIV				0x0302
#define MT9F002_PRE_PLL_CLK_DIV				0x0304
#define MT9F002_PLL_MULTIPLIER				0x0306
#define MT9F002_OP_PIX_CLK_DIV				0x0308
#define MT9F002_OP_SYS_CLK_DIV				0x030A
#define MT9F002_CCP_DATA_FORMAT                         0x0112
#define MT9F002_READ_MODE                       	0x3040
#define MT9F002_MIN_FRAME_BLANKING_LINES		0x114A
#define MT9F002_MIN_FRAME_BLANKING_LINES_DEF		0x92
#define MT9F002_COARSE_INTEGRATION_TIME_MIN		0x1004
#define MT9F002_COARSE_INTEGRATION_TIME_MAX_MARGIN	0x1006
#define MT9F002_ROW_SPEED				0x3016
#define MT9F002_ANALOG_CONTROL5				0x3178
#define MT9F002_DARK_CONTROL3  				0x30EE
#define MT9F002_SCALE_M					0x0404
#define MT9F002_ANALOG_CONTROL7				0x317C
#define MT9F002_COARSE_INTEGRATION_TIME			0x0202
#define MT9F002_DIGITAL_TEST   				0x30B0
#define MT9F002_X_ADDR_START   				0x0344
#define MT9F002_X_ADDR_END     				0x0348
#define		MT9F002_X_ADDR_MIN			24
#define		MT9F002_X_ADDR_MAX			4608
#define MT9F002_X_ODD_INC      				0x0382
#define MT9F002_X_OUTPUT_SIZE  				0x034C
#define MT9F002_Y_ADDR_START   				0x0346
#define MT9F002_Y_ADDR_END     				0x034A
#define		MT9F002_Y_ADDR_MIN			0
#define		MT9F002_Y_ADDR_MAX			3288
#define	MT9F002_WINDOW_HEIGHT_MIN        		1
#define	MT9F002_WINDOW_HEIGHT_MAX			3288
#define	MT9F002_WINDOW_WIDTH_MIN        		1
#define	MT9F002_WINDOW_WIDTH_MAX			4608
#define MT9F002_Y_ODD_INC      				0x0386
#define MT9F002_Y_OUTPUT_SIZE  				0x034E
#define MT9F002_LINE_LENGTH_PCK				0x0342
#define MT9F002_MIN_LINE_BLANKING_PCK			0x1148
#define MT9F002_MIN_LINE_LENGTH_PCK			0x1144
#define MT9F002_FRAME_LENGTH_LINES			0x0340
#define MT9F002_IMAGE_ORIENTATION			0x0101
#define MT9F002_FINE_INTEGRATION_TIME			0x3014
#define MT9F002_FINE_INTEGRATION_TIME_MIN		0x1008
#define MT9F002_FINE_INTEGRATION_TIME_MAX_MARGIN	0x100A
#define MT9F002_FINE_CORRECTION				0x3010
#define		MT9F002_GREEN1_GAIN			0x3056
#define		MT9F002_BLUE_GAIN			0x3058
#define		MT9F002_RED_GAIN			0x305A
#define		MT9F002_GREEN2_GAIN			0x305C
#define		MT9F002_TEST_PATTERN			0x3070
#define		MT9F002_TEST_PATTERN_RED		0x3072
#define		MT9F002_TEST_PATTERN_GREENR		0x3074
#define		MT9F002_TEST_PATTERN_BLUE		0x3076
#define		MT9F002_TEST_PATTERN_GREENB		0x3078
#define 	MT9F002_HISPI_CONTROL_STATUS		0x31C6
#define         MT9F002_EXTRA_DELAY                     0x3018
#define         MT9F002_RESERVED_MFR_3D00		0x3D00
#define         MT9F002_RESERVED_MFR_3D02		0x3D02
#define         MT9F002_RESERVED_MFR_3D04		0x3D04
#define         MT9F002_RESERVED_MFR_3D06		0x3D06
#define         MT9F002_RESERVED_MFR_3D08		0x3D08
#define         MT9F002_RESERVED_MFR_3D0A		0x3D0A
#define         MT9F002_RESERVED_MFR_3D0C		0x3D0C
#define         MT9F002_RESERVED_MFR_3D0E		0x3D0E
#define         MT9F002_RESERVED_MFR_3D10		0x3D10
#define         MT9F002_RESERVED_MFR_3D12		0x3D12
#define         MT9F002_RESERVED_MFR_3D14		0x3D14
#define         MT9F002_RESERVED_MFR_3D16		0x3D16
#define         MT9F002_RESERVED_MFR_3D18		0x3D18
#define         MT9F002_RESERVED_MFR_3D1A		0x3D1A
#define         MT9F002_RESERVED_MFR_3D1C		0x3D1C
#define         MT9F002_RESERVED_MFR_3D1E		0x3D1E
#define         MT9F002_RESERVED_MFR_3D20		0x3D20
#define         MT9F002_RESERVED_MFR_3D22		0x3D22
#define         MT9F002_RESERVED_MFR_3D24		0x3D24
#define         MT9F002_RESERVED_MFR_3D26		0x3D26
#define         MT9F002_RESERVED_MFR_3D28		0x3D28
#define         MT9F002_RESERVED_MFR_3D2A		0x3D2A
#define         MT9F002_RESERVED_MFR_3D2C		0x3D2C
#define         MT9F002_RESERVED_MFR_3D2E		0x3D2E
#define         MT9F002_RESERVED_MFR_3D30		0x3D30
#define         MT9F002_RESERVED_MFR_3D32		0x3D32
#define         MT9F002_RESERVED_MFR_3D34		0x3D34
#define         MT9F002_RESERVED_MFR_3D36		0x3D36
#define         MT9F002_RESERVED_MFR_3D38		0x3D38
#define         MT9F002_RESERVED_MFR_3D3A		0x3D3A
#define         MT9F002_RESERVED_MFR_3D3C		0x3D3C
#define         MT9F002_RESERVED_MFR_3D3E		0x3D3E
#define         MT9F002_RESERVED_MFR_3D40		0x3D40
#define         MT9F002_RESERVED_MFR_3D42		0x3D42
#define         MT9F002_RESERVED_MFR_3D44		0x3D44
#define         MT9F002_RESERVED_MFR_3D46		0x3D46
#define         MT9F002_RESERVED_MFR_3D48		0x3D48
#define         MT9F002_RESERVED_MFR_3D4A		0x3D4A
#define         MT9F002_RESERVED_MFR_3D4C		0x3D4C
#define         MT9F002_RESERVED_MFR_3D4E		0x3D4E
#define         MT9F002_RESERVED_MFR_3D50		0x3D50
#define         MT9F002_RESERVED_MFR_3D52		0x3D52
#define         MT9F002_RESERVED_MFR_3D54		0x3D54
#define         MT9F002_RESERVED_MFR_3D56		0x3D56
#define         MT9F002_RESERVED_MFR_3D58		0x3D58
#define         MT9F002_RESERVED_MFR_3D5A		0x3D5A
#define         MT9F002_RESERVED_MFR_3D5C		0x3D5C
#define         MT9F002_RESERVED_MFR_3D5E		0x3D5E
#define         MT9F002_RESERVED_MFR_3D60		0x3D60
#define         MT9F002_RESERVED_MFR_3D62		0x3D62
#define         MT9F002_RESERVED_MFR_3D64		0x3D64
#define         MT9F002_RESERVED_MFR_3D66		0x3D66
#define         MT9F002_RESERVED_MFR_3D68		0x3D68
#define         MT9F002_RESERVED_MFR_3D6A		0x3D6A
#define         MT9F002_RESERVED_MFR_3D6C		0x3D6C
#define         MT9F002_RESERVED_MFR_3D6E		0x3D6E
#define         MT9F002_RESERVED_MFR_3D70		0x3D70
#define         MT9F002_RESERVED_MFR_3D72		0x3D72
#define         MT9F002_RESERVED_MFR_3D74		0x3D74
#define         MT9F002_RESERVED_MFR_3D76		0x3D76
#define         MT9F002_RESERVED_MFR_3D78		0x3D78
#define         MT9F002_RESERVED_MFR_3D7A		0x3D7A
#define         MT9F002_RESERVED_MFR_3D7C		0x3D7C
#define         MT9F002_RESERVED_MFR_3D7E		0x3D7E
#define         MT9F002_RESERVED_MFR_3D80		0x3D80
#define         MT9F002_RESERVED_MFR_3D82		0x3D82
#define         MT9F002_RESERVED_MFR_3D84		0x3D84
#define         MT9F002_RESERVED_MFR_3D86		0x3D86
#define         MT9F002_RESERVED_MFR_3D88		0x3D88
#define         MT9F002_RESERVED_MFR_3D8A		0x3D8A
#define         MT9F002_RESERVED_MFR_3D8C		0x3D8C
#define         MT9F002_RESERVED_MFR_3D8E		0x3D8E
#define         MT9F002_RESERVED_MFR_3D90		0x3D90
#define         MT9F002_RESERVED_MFR_3D92		0x3D92
#define         MT9F002_RESERVED_MFR_3D94		0x3D94
#define         MT9F002_RESERVED_MFR_3D96		0x3D96
#define         MT9F002_RESERVED_MFR_3D98		0x3D98
#define         MT9F002_RESERVED_MFR_3D9A		0x3D9A
#define         MT9F002_RESERVED_MFR_3D9C		0x3D9C
#define         MT9F002_RESERVED_MFR_3D9E		0x3D9E
#define         MT9F002_RESERVED_MFR_3DA0		0x3DA0
#define         MT9F002_RESERVED_MFR_3DA2		0x3DA2
#define         MT9F002_RESERVED_MFR_3DA4		0x3DA4
#define         MT9F002_RESERVED_MFR_3DA6		0x3DA6
#define         MT9F002_RESERVED_MFR_3DA8		0x3DA8
#define         MT9F002_RESERVED_MFR_3DAA		0x3DAA
#define         MT9F002_RESERVED_MFR_3DAC		0x3DAC
#define         MT9F002_RESERVED_MFR_3DAE		0x3DAE
#define         MT9F002_RESERVED_MFR_3DB0		0x3DB0
#define         MT9F002_RESERVED_MFR_3DB2		0x3DB2
#define         MT9F002_RESERVED_MFR_3DB4		0x3DB4
#define         MT9F002_RESERVED_MFR_3DB6		0x3DB6
#define         MT9F002_RESERVED_MFR_3DB8		0x3DB8
#define         MT9F002_RESERVED_MFR_3DBA		0x3DBA
#define         MT9F002_RESERVED_MFR_3DBC		0x3DBC
#define         MT9F002_RESERVED_MFR_3DBE		0x3DBE
#define         MT9F002_RESERVED_MFR_3DC0		0x3DC0
#define         MT9F002_RESERVED_MFR_3DC2		0x3DC2
#define         MT9F002_RESERVED_MFR_3DC4		0x3DC4
#define         MT9F002_RESERVED_MFR_3DC6		0x3DC6
#define         MT9F002_RESERVED_MFR_3DC8		0x3DC8
#define         MT9F002_RESERVED_MFR_3DCA		0x3DCA
#define         MT9F002_RESERVED_MFR_3DCC		0x3DCC
#define         MT9F002_RESERVED_MFR_3DCE		0x3DCE
#define         MT9F002_RESERVED_MFR_3DD0		0x3DD0
#define         MT9F002_RESERVED_MFR_3DD2		0x3DD2
#define         MT9F002_RESERVED_MFR_3DD4		0x3DD4
#define         MT9F002_RESERVED_MFR_3DD6		0x3DD6
#define         MT9F002_RESERVED_MFR_3DD8		0x3DD8
#define         MT9F002_RESERVED_MFR_3DDA		0x3DDA
#define         MT9F002_RESERVED_MFR_3DDC		0x3DDC
#define         MT9F002_RESERVED_MFR_3DDE		0x3DDE
#define         MT9F002_RESERVED_MFR_3DE0		0x3DE0
#define         MT9F002_RESERVED_MFR_3DE2		0x3DE2
#define         MT9F002_RESERVED_MFR_3DE4		0x3DE4
#define         MT9F002_RESERVED_MFR_3DE6		0x3DE6
#define         MT9F002_RESERVED_MFR_3DE8		0x3DE8
#define         MT9F002_RESERVED_MFR_3DEA		0x3DEA
#define         MT9F002_RESERVED_MFR_3DEC		0x3DEC
#define         MT9F002_RESERVED_MFR_3DEE		0x3DEE
#define         MT9F002_RESERVED_MFR_3DF0		0x3DF0
#define         MT9F002_RESERVED_MFR_3DF2		0x3DF2
#define         MT9F002_RESERVED_MFR_3DF4		0x3DF4
#define         MT9F002_RESERVED_MFR_3DF6		0x3DF6
#define         MT9F002_RESERVED_MFR_3DF8		0x3DF8
#define         MT9F002_RESERVED_MFR_3DFA		0x3DFA
#define         MT9F002_RESERVED_MFR_3DFC		0x3DFC
#define         MT9F002_RESERVED_MFR_3DFE		0x3DFE
#define         MT9F002_RESERVED_MFR_3E00		0x3E00
#define         MT9F002_RESERVED_MFR_3E02		0x3E02
#define         MT9F002_RESERVED_MFR_3E04		0x3E04
#define         MT9F002_RESERVED_MFR_3E06		0x3E06
#define         MT9F002_RESERVED_MFR_3E08		0x3E08
#define         MT9F002_RESERVED_MFR_3E0A		0x3E0A
#define         MT9F002_RESERVED_MFR_3E0C		0x3E0C
#define         MT9F002_RESERVED_MFR_3E0E		0x3E0E
#define         MT9F002_RESERVED_MFR_3E10		0x3E10
#define         MT9F002_RESERVED_MFR_3E12		0x3E12
#define         MT9F002_RESERVED_MFR_3E14		0x3E14
#define         MT9F002_RESERVED_MFR_3E16		0x3E16
#define         MT9F002_RESERVED_MFR_3E18		0x3E18
#define         MT9F002_RESERVED_MFR_3E1A		0x3E1A
#define         MT9F002_RESERVED_MFR_3E1C		0x3E1C
#define         MT9F002_RESERVED_MFR_3E1E		0x3E1E
#define         MT9F002_RESERVED_MFR_3E20		0x3E20
#define         MT9F002_RESERVED_MFR_3E22		0x3E22
#define         MT9F002_RESERVED_MFR_3E24		0x3E24
#define         MT9F002_RESERVED_MFR_3E26		0x3E26
#define         MT9F002_RESERVED_MFR_3E28		0x3E28
#define         MT9F002_RESERVED_MFR_3E2A		0x3E2A
#define         MT9F002_RESERVED_MFR_3E2C		0x3E2C
#define         MT9F002_RESERVED_MFR_3E2E		0x3E2E
#define         MT9F002_RESERVED_MFR_3E30		0x3E30
#define         MT9F002_RESERVED_MFR_3E32		0x3E32
#define         MT9F002_RESERVED_MFR_3E34		0x3E34
#define         MT9F002_RESERVED_MFR_3E36		0x3E36
#define         MT9F002_RESERVED_MFR_3E38		0x3E38
#define         MT9F002_RESERVED_MFR_3E3A		0x3E3A
#define         MT9F002_RESERVED_MFR_3E3C		0x3E3C
#define         MT9F002_RESERVED_MFR_3E3E		0x3E3E
#define         MT9F002_RESERVED_MFR_3E40		0x3E40
#define         MT9F002_RESERVED_MFR_3E42		0x3E42
#define         MT9F002_RESERVED_MFR_3E44		0x3E44
#define         MT9F002_RESERVED_MFR_3E46		0x3E46
#define         MT9F002_RESERVED_MFR_3E48		0x3E48
#define         MT9F002_RESERVED_MFR_3E4A		0x3E4A
#define         MT9F002_RESERVED_MFR_3E4C		0x3E4C
#define         MT9F002_RESERVED_MFR_3E4E		0x3E4E
#define         MT9F002_RESERVED_MFR_3E50		0x3E50
#define         MT9F002_RESERVED_MFR_3E52		0x3E52
#define         MT9F002_RESERVED_MFR_3E54		0x3E54
#define         MT9F002_RESERVED_MFR_3E56		0x3E56
#define         MT9F002_RESERVED_MFR_3E58		0x3E58
#define         MT9F002_RESERVED_MFR_3E5A		0x3E5A
#define         MT9F002_RESERVED_MFR_3E5C		0x3E5C
#define         MT9F002_RESERVED_MFR_3E5E		0x3E5E
#define         MT9F002_RESERVED_MFR_3E60		0x3E60
#define         MT9F002_RESERVED_MFR_3E62		0x3E62
#define         MT9F002_RESERVED_MFR_3E64		0x3E64
#define         MT9F002_RESERVED_MFR_3E66		0x3E66
#define         MT9F002_RESERVED_MFR_3E68		0x3E68
#define         MT9F002_RESERVED_MFR_3E6A		0x3E6A
#define         MT9F002_RESERVED_MFR_3E6C		0x3E6C
#define         MT9F002_RESERVED_MFR_3E6E		0x3E6E
#define         MT9F002_RESERVED_MFR_3E70		0x3E70
#define         MT9F002_RESERVED_MFR_3E72		0x3E72
#define         MT9F002_RESERVED_MFR_3E74		0x3E74
#define         MT9F002_RESERVED_MFR_3E76		0x3E76
#define         MT9F002_RESERVED_MFR_3E78		0x3E78
#define         MT9F002_RESERVED_MFR_3E7A		0x3E7A
#define         MT9F002_RESERVED_MFR_3E7C		0x3E7C
#define         MT9F002_RESERVED_MFR_3E7E		0x3E7E
#define         MT9F002_RESERVED_MFR_3E80		0x3E80
#define         MT9F002_RESERVED_MFR_3E82		0x3E82
#define         MT9F002_RESERVED_MFR_3E84		0x3E84
#define         MT9F002_RESERVED_MFR_3E86		0x3E86
#define         MT9F002_RESERVED_MFR_3E88		0x3E88
#define         MT9F002_RESERVED_MFR_3E8A		0x3E8A
#define         MT9F002_RESERVED_MFR_3E8C		0x3E8C
#define         MT9F002_RESERVED_MFR_3E8E		0x3E8E
#define         MT9F002_RESERVED_MFR_3E90		0x3E90
#define         MT9F002_RESERVED_MFR_3E92		0x3E92
#define         MT9F002_RESERVED_MFR_3E94		0x3E94
#define         MT9F002_RESERVED_MFR_3E96		0x3E96
#define         MT9F002_RESERVED_MFR_3E98		0x3E98
#define         MT9F002_RESERVED_MFR_3E9A		0x3E9A
#define         MT9F002_RESERVED_MFR_3E9C		0x3E9C
#define         MT9F002_RESERVED_MFR_3E9E		0x3E9E
#define         MT9F002_RESERVED_MFR_3EA0		0x3EA0
#define         MT9F002_RESERVED_MFR_3EA2		0x3EA2
#define         MT9F002_RESERVED_MFR_3EA4		0x3EA4
#define         MT9F002_RESERVED_MFR_3EA6		0x3EA6
#define         MT9F002_RESERVED_MFR_3EA8		0x3EA8
#define         MT9F002_RESERVED_MFR_3EAA		0x3EAA
#define         MT9F002_RESERVED_MFR_3EAC		0x3EAC
#define         MT9F002_RESERVED_MFR_3EAE		0x3EAE
#define         MT9F002_RESERVED_MFR_3EB0		0x3EB0
#define         MT9F002_RESERVED_MFR_3EB2		0x3EB2
#define         MT9F002_RESERVED_MFR_3EB4		0x3EB4
#define         MT9F002_RESERVED_MFR_3EB6		0x3EB6
#define         MT9F002_RESERVED_MFR_3EB8		0x3EB8
#define         MT9F002_RESERVED_MFR_3EBA		0x3EBA
#define         MT9F002_RESERVED_MFR_3EBC		0x3EBC
#define         MT9F002_RESERVED_MFR_3EBE		0x3EBE
#define         MT9F002_RESERVED_MFR_3EC0		0x3EC0
#define         MT9F002_RESERVED_MFR_3EC2		0x3EC2
#define         MT9F002_RESERVED_MFR_3EC4		0x3EC4
#define         MT9F002_RESERVED_MFR_3EC6		0x3EC6
#define         MT9F002_RESERVED_MFR_3EC8		0x3EC8
#define         MT9F002_RESERVED_MFR_3ECA		0x3ECA
#define         MT9F002_RESERVED_MFR_3176		0x3176
#define         MT9F002_RESERVED_MFR_317C		0x317C
#define         MT9F002_RESERVED_MFR_3EE6		0x3EE6
#define         MT9F002_RESERVED_MFR_3ED8		0x3ED8
#define         MT9F002_RESERVED_MFR_3EDC		0x3EDC
#define         MT9F002_RESERVED_MFR_3EE2		0x3EE2
#define         MT9F002_RESERVED_MFR_3EE8		0x3EE8
#define         MT9F002_RESERVED_MFR_3064		0x3064

#define 	MT9F002_MODE_SELECT			0x0100
#define 	MT9F002_SOFTWARE_RESET			0x103
#define		MT9F002_GROUPED_PARAMETER_HOLD		0x0104
#define 	MT9F002_MASK_CORRUPTED_FRAME            0x0105


/*
 * Sensor Timing Constants
 * values have been found experimentally
 */
/* binning mode */
#define BINNING_XY_MIN_LINE_LENGTH_PCK			4650
#define BINNING_XY_MIN_LINE_BLANKING_PCK		2950
#define BINNING_XY_MIN_LINE_FIFO_BLANKING_PCK		120
#define BINNING_XY_FINE_INTEGRATION_TIME_MIN		2000
#define BINNING_XY_FINE_INTEGRATION_TIME_MAX_MARGIN	2200

#define BINNING_X_MIN_LINE_LENGTH_PCK			3495
#define BINNING_X_MIN_LINE_BLANKING_PCK			0
#define BINNING_X_MIN_LINE_FIFO_BLANKING_PCK		60
#define BINNING_X_FINE_INTEGRATION_TIME_MIN		1500
#define BINNING_X_FINE_INTEGRATION_TIME_MAX_MARGIN	1900

#define NO_BINNING_NO_SKIPPING 				0x1
#define BINNING_ONLY 					0x3
#define BINNING_AND_SKIPPING 				0x7

/* normal mode */
#define MIN_LINE_LENGTH_PCK				2400
#define MIN_LINE_BLANKING_PCK				1700
#define MIN_LINE_FIFO_BLANKING_PCK			60
#define FINE_INTEGRATION_TIME_MIN			1316
#define FINE_INTEGRATION_TIME_MAX_MARGIN		1032
/* scaler mode */
#define SCALER_MIN_LINE_LENGTH_PCK			2400
#define SCALER_MIN_LINE_BLANKING_PCK			1750
#define SCALER_MIN_LINE_FIFO_BLANKING_PCK		60
#define SCALER_FINE_INTEGRATION_TIME_MIN		1316
#define SCALER_FINE_INTEGRATION_TIME_MAX_MARGIN		1032

#define COARSE_INTEGRATION_TIME_MIN_VAL			0
#define COARSE_INTEGRATION_TIME_MAX_MARGIN_VAL		1

#define MT9F002_FRAMEPERIOD_NS_DEF 			33333333 //~30Fps
#define MT9F002_EXPOSURE_DEF				30000
#define MT9F002_SCALER_DEF 				16

struct mt9f002_clock {
	uint32_t vt_pix_clk;
	uint32_t op_pix_clk;
};

struct mt9f002_color_gain {
	uint32_t greenR;
	uint32_t blue;
	uint32_t red;
	uint32_t greenB;
};

struct mt9f002_pll {
	uint32_t pre_pll_clk_div;
	uint32_t pll_multiplier;
	uint32_t vt_sys_clk_div;
	uint32_t vt_pix_clk_div;
	uint32_t row_speed_2_0;
	uint32_t op_sys_clk_div;
	uint32_t row_speed_10_8;
	uint32_t op_pix_clk_div;
	uint32_t shift_vt_pix_clk_div;
};

/* HV blanking length in µs
 * A frame is equal to (blkStartHV_us + frameLength_us + blkExtraHV_us)
 */
struct mt9f002_timing {
	uint32_t blkStartHV_us;
	uint32_t frameLength_us;
	uint32_t blkExtraHV_us;
#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
	uint32_t exposure_us;
#endif
};

struct mt9f002_blanking {
	uint16_t min_line_blanking_pck;
	uint16_t x_odd_inc;
	uint16_t y_odd_inc;
	uint16_t min_frame_blanking_lines;
	uint16_t min_line_length_pck;
	uint16_t min_line_fifo_pck;

	uint16_t minimum_line_length;
	uint16_t minimum_frame_length_lines;

	uint16_t line_length;
	uint16_t frame_length;
};

struct mt9f002_exposure {
	uint16_t fine_integration_time_min;
	uint16_t fine_integration_time_max_margin;
	uint16_t coarse_integration_time_min;
	uint16_t coarse_integration_time_max_margin;

	uint16_t coarse_integration_time;
	uint16_t fine_integration_time;
};

struct mt9f002_context {
	struct mt9f002_color_gain	colorGain;		/* color gain */
	uint32_t 			frameperiod_ns;		/* frame period in ns */
	uint32_t 			max_frameperiod_ns; 	/* maximum reachable framerate */

	uint32_t 			scaler;			/* ratio between sensor resolution and ouput resolution */

	uint32_t maxExposure_us;                     		/* maximum exposure time in us */

	struct mt9f002_pll pll;
	struct mt9f002_clock clock;
	struct mt9f002_blanking blanking;
	struct mt9f002_exposure exposure;
	struct mt9f002_timing prev_timings;
	struct mt9f002_timing timings;
};

struct mt9f002_timer {
	struct hrtimer			hrtimer;
	struct video_device		*vdev;
	uint32_t			frame;
};

struct mt9f002 {
	struct v4l2_subdev 		subdev;
	struct i2c_client 		*i2c_client;
	struct media_pad 		pad;
	struct mt9f002_context 		ctx;

	struct v4l2_mbus_framefmt 	format;
	struct v4l2_rect 		crop;

	struct  v4l2_ctrl_handler 	ctrls;
	struct 	v4l2_ctrl 		*target_exposure_us;
	struct 	v4l2_ctrl 		*pixel_rate;
	struct 	v4l2_ctrl 		*hblank;
	struct 	v4l2_ctrl 		*vblank;
	struct 	v4l2_ctrl 		*gains[4];

	struct mutex	 		power_lock;
	int 				power_count;

	struct mt9f002_platform_data 	*pdata;
	int (*set_power)(int on);
	int 				isStreaming;

	/* Timers for FRAME_SYNC event */
	uint32_t			fsync_delay_us;
	struct mt9f002_timer		fsync_timers[2];
#ifdef CONFIG_VIDEO_MT9F002_RESTART_BAD_FRAME
	struct task_struct		*fsync_worker_task;
	struct kthread_worker		fsync_worker;
	struct kthread_work		fsync_work;
#endif
	int				next_timer;
	int				drop;
	int				frame;
};

typedef union {
	struct {
		unsigned    analog_gain2	:7;
		unsigned    analog_gain3	:3;
		unsigned    colamp_gain		:2;
		unsigned    digital_gain	:4;
	};
	uint32_t val;
} pixelGain_t;

#endif
