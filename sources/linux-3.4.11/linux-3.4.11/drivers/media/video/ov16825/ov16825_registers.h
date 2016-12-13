/*
 *      drivers/media/video/ov16825/device/ov16825.h
 *
 *      Copyright (C) 2014 Parrot S.A.
 *
 * @author  Victor Lambret <victor.lambret.ext@parrot.com>
 * @date    01-Dec-2014
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __OV16825__H__
#define __OV16825__H__

/* PLL control */
#define PLL1_CTRL0                      0x0300
#define PLL1_CTRL2                      0x0302
#define PLL1_CTRL3                      0x0303
#define PLL1_CTRL4                      0x0304
#define PLL1_CTRL5                      0x0305

#define OP_PIX_DIV4                     0x0
#define OP_PIX_DIV5                     0x1
#define OP_PIX_DIV6                     0x2
#define OP_PIX_DIV8                     0x3
#define PLL1_CTRL6                      0x0306

#define OP_2LANE_NODIV                  0x0
#define OP_2LANE_DIV2                   0x1
#define PLL1_CTRL7                      0x0307

#define PLL2_CTRL0                      0x030B
#define PLL2_CTRL1                      0x030C
#define PLL2_CTRL2                      0x030D
#define PLL2_CTRL3                      0x030E
#define PLL2_CTRL4                      0x030F
#define PLL3_CTRL0                      0x0313
#define PLL3_CTRL1                      0x0314
#define PLL3_CTRL2                      0x0315
#define PLL3_CTRL3                      0x0316
#define PLL3_CTRL4                      0x0317
#define MIPI_BITSEL_CTRL                0x031E
#define LVDS_BIT16                      0x031F

/* system control */
#define SC_CTRL0100_STREAM_OFF          0x0
#define SC_CTRL0100_STREAM_ON           0x1
#define SC_CTRL0100                     0x0100
#define SC_CTRL0103_SOFTWARE_RESET      0x1
#define SC_CTRL0103                     0x0103
#define SC_CMMN_PAD_OEN0                0x3000
#define SC_CMMN_PAD_OEN2                0x3002
#define SCCB_ID                         0x3004
#define CMMN_SCCB_ID2                   0x3006
#define SC_CMMN_PAD_OUT0                0x3008
#define SC_CMMN_CHIP_ID1                0x300A
#define SC_CMMN_CHIP_ID2                0x300B
#define SC_CMMN_CHIP_ID3                0x300C
#define SC_CMMN_PAD_OUT2                0x300D
#define SC_CMMN_PAD_SEL2                0x3010
#define SC_CMMN_PAD_PK                  0x3011
#define SCCB_SECOND_ID                  0x3012

#define RESET_MIPI_PHY_WHEN_SLEEP       0x2
#define MODE_1LANES                     0x00
#define MODE_2LANES                     0x20
#define MODE_4LANES                     0x60
#define MODE_8LANES                     0x70

#define SC_CMMN_MIPI_SC_CTRL            0x3018
#define SC_CMMN_MIPI_SC_CTRL_MANUAL     0x3019
#define SC_CMMN_CLKRST0                 0x301A
#define SC_CMMN_CLKRST1                 0x301B
#define SC_CMMN_CLKRST2                 0x301C
#define SC_CMMN_CLKRST3                 0x301D
#define SC_CMMN_CLKRST4                 0x301E
#define SC_CMMN_CLKRST5                 0x301F
#define SC_CMMN_CLOCK_SEL               0x3020
#define SC_CMMN_MISC_CTRL               0x3021
#define SC_CMMN_MIPI_SC_CTRL2           0x3022
#define SC_CMMN_SUB_ID                  0x302A
#define SC_CMMN_BIT_SEL                 0x3031

#define SEL_SCLK_PLL_SCLK               0x00
#define SEL_SCLK_PLL2                   0x80
#define SC_CMMN_CORE_CTRL0              0x3032

#define SEL_DACCLK_PLL_DACCLK           0x4
#define SEL_DACCLK_PLL_SCLK             0x0
#define SC_CMMN_CORE_CTRL1              0x3033
#define SIF_CTRL0                       0x303C

/* test pattern control */
#define PATTERN_DISABLE                         0x00
#define PATTERN_BLACK                           0x83
#define PATTERN_COLOR_BAR                       0x80
#define PATTERN_COLOR_BAR_DARK_TOP_BOTTOM       0x84
#define PATTERN_COLOR_BAR_DARK_LEFT_RIGHT       0x88
#define PATTERN_COLOR_BAR_DARK_BOTTOM_TOP       0x8C
#define PATTERN_SQUARE_BW                       0x92
#define PATTERN_SQUARE_COLOR                    0x82
#define PATTERN_RANDOM                          0x81

#define RED_GAIN                        0x500C
#define RED_GAIN_LSB                    0x500D
#define GREEN_GAIN                      0x500E
#define GREEN_GAIN_LSB                  0x500F
#define BLUE_GAIN                       0x5010
#define BLUE_GAIN_LSB                   0x5011

#define TEST_PATTERN_0                  0x5040
#define TEST_PATTERN_1                  0x5041

/* exposure/gain control */
#define EXPO0                           0x3500
#define EXPO1                           0x3501
#define EXPO2                           0x3502
#define R_MANUAL                        0x3503
#define GAIN0                           0x3508
#define GAIN1                           0x3509
#define AVG_CTRL00                      0x5680
#define AVG_CTRL01                      0x5681
#define AVG_CTRL02                      0x5682
#define AVG_CTRL03                      0x5683
#define AVG_CTRL04                      0x5684
#define AVG_CTRL05                      0x5685
#define AVG_CTRL06                      0x5686
#define AVG_CTRL07                      0x5687
#define AVG_CTRL08                      0x5688
#define AVG_CTRL09                      0x5689
#define AVG_CTRL0A                      0x568A
#define AVG_CTRL0B                      0x568B
#define AVG_CTRL0C                      0x568C
#define AVG_CTRL0D                      0x568D
#define AVG_CTRL0E                      0x568E
#define AVG_CTRL0F                      0x568F
#define AVG_CTRL10                      0x5690
#define AVG_ROREG2                      0x5693

/* system timing control registers */
#define H_CROP_START0                   0x3800
#define H_CROP_START1                   0x3801
#define V_CROP_START0                   0x3802
#define V_CROP_START1                   0x3803
#define H_CROP_END0                     0x3804
#define H_CROP_END1                     0x3805
#define V_CROP_END0                     0x3806
#define V_CROP_END1                     0x3807
#define H_OUTPUT_SIZE0                  0x3808
#define H_OUTPUT_SIZE1                  0x3809
#define V_OUTPUT_SIZE0                  0x380A
#define V_OUTPUT_SIZE1                  0x380B
#define TIMING_HTS0                     0x380C
#define TIMING_HTS1                     0x380D
#define TIMING_VTS0                     0x380E
#define TIMING_VTS1                     0x380F
#define H_WIN_OFF0                      0x3810
#define H_WIN_OFF1                      0x3811
#define V_WIN_OFF0                      0x3812
#define V_WIN_OFF1                      0x3813
#define H_INC_ODD                       0x3814
#define H_INC_EVEN                      0x3815
#define VTS_EXP_DIFF                    0x381A
#define FORMAT0                         0x3820
#define FORMAT1                         0x3821
#define TIMING_REG23                    0x3823
#define CS_RST_FSIN_HI                  0x3824
#define CS_RST_FSIN_LO                  0x3825
#define R_RST_FSIN_HI                   0x3826
#define R_RST_FSIN_LO                   0x3827
#define TIMING_REG28                    0x3828
#define FORMAT2                         0x3829
#define V_INC_ODD                       0x382A
#define V_INC_EVEN                      0x382B
#define FRACTIONAL_HTS_HI               0x3832
#define FRACTIONAL_HTS_LO               0x3833
#define EXT_DIV_FACTORS                 0x3834
#define GROUP_WRITE_OPTION              0x3835

/* strobe control */
#define STROBE_REG0                     0x3B00
#define STROBE_REG1                     0x3B02
#define STROBE_REG2                     0x3B03

/* FREX control */
#define FREX_REG5                       0x3B85
#define FREX_REG6                       0x3B86
#define FREX_REG7                       0x3B87
#define FREX_REG9                       0x3B89
#define FREX_REGA                       0x3B8A
#define FREX_REGB                       0x3B8B
#define FREX_REGC                       0x3B8C
#define FREX_REGD                       0x3B8D
#define FREX_REGE                       0x3B8E
#define FREX_REGF                       0x3B8F
#define FREX_REG10                      0x3B90
#define FREX_REG11                      0x3B91
#define FREX_REG12                      0x3B92
#define FREX_REG13                      0x3B93
#define FREX_REG14                      0x3B94
#define FREX_REG15                      0x3B95
#define FREX_REG16                      0x3B96
#define FREX_REG1E                      0x3B9E
#define FREX_REG1F                      0x3B9F

/* OTP control */
#define OTP_PROGRAM_CTRL                0x3D80
#define OTP_LOA_CTRL                    0x3D81
#define OTP_MODE_CTRL                   0x3D84
#define OTP_REG85                       0x3D85
#define OTP_START_ADDRESS0              0x3D88
#define OTP_START_ADDRESS1              0x3D89
#define OTP_END_ADDRESS0                0x3D8A
#define OTP_END_ADDRESS1                0x3D8B
#define OTP_SETTING_STT_ADDRESS0        0x3D8C
#define OTP_SETTING_STT_ADDRESS1        0x3D8D
#define OTP_BIST_ERR_ADDRESS0           0x3D8E
#define OTP_BIST_ERR_ADDRESS1           0x3D8F

/* BLC control */
#define BLC_CTRL00                      0x4000
#define BLC_CTRL04                      0x4004
#define BLC_CTRL05                      0x4005

/* frame control */
#define FRAME_CONTROL00                 0x4200
#define FRAME_CONTROL01                 0x4201
#define FRAME_CONTROL02                 0x4202
#define FRAME_CONTROL03                 0x4203

/* clipping */
#define CLIPPING_MAX0                   0x4302
#define CLIPPING_MAX1                   0x4303
#define CLIPPING_MIN0                   0x4304
#define CLIPPING_MIN1                   0x4305
#define DPCM_CTRL                       0x4306
#define MIPI_16BIT_DT                   0x4307

/* VFIFO control */
#define VFIFO_REG2                      0x4602
#define VFIFO_REG3                      0x4603

/* MIPI control */
#define MIPI_CTRL_00                    0x4800
#define MIPI_CTRL_01                    0x4801
#define MIPI_CTRL_02                    0x4802
#define MIPI_CTRL_04                    0x4804
#define MIPI_MAX_FRAME_COUNT_HI         0x4810
#define MIPI_MAX_FRAME_COUNT_LO         0x4811
#define MIPI_CTRL14                     0x4814
#define MIPI_CTRL15                     0x4815
#define HS_ZERO_MIN_HI                  0x4818
#define HS_ZERO_MIN_LO                  0x4819
#define HS_TRAIL_MIN_HI                 0x481A
#define HS_TRAIL_MIN_LO                 0x481B
#define CLK_ZERO_MIN_HI                 0x481C
#define CLK_ZERO_MIN_LO                 0x481D
#define CLK_PREPARE_MIN_HI              0x481E
#define CLK_PREPARE_MIN_LO              0x481F
#define CLK_POST_MIN_HI                 0x4820
#define CLK_POST_MIN_LO                 0x4821
#define CLK_TRAIL_MIN_HI                0x4822
#define CLK_TRAIL_MIN_LO                0x4823
#define LPX_P_MIN_HI                    0x4824
#define LPX_P_MIN_LO                    0x4825
#define HS_PREPARE_MIN_HI               0x4826
#define HS_PREPARE_MIN_LO               0x4827
#define HS_EXIT_MIN_HI                  0x4828
#define HS_EXIT_MIN_LO                  0x4829
#define UI_HS_ZERO_MIN                  0x482A
#define UI_HS_TRAIL_MIN                 0x482B
#define UI_CLK_ZERO_MIN                 0x482C
#define UI_CLK_PREPARE_MIN              0x482D
#define UI_CLK_POST_MIN                 0x482E
#define UI_CLK_TRAIL_MIN                0x482F
#define UI_LPX_P_MIN                    0x4830
#define UI_HS_PREPARE_MIN               0x4831
#define UI_HS_EXIT_MIN                  0x4832
#define PCLK_PERIOD                     0x4837
#define MIPI_LP_GPIO_0                  0x4838
#define MIPI_LP_GPIO_1                  0x483A
#define MIPI_LP_GPIO                    0x483B
#define MIPI_CTRL_3C                    0x483C
#define MIPI_LP_GPIO_3                  0x483D
#define MIPI_CTRL4C                     0x484C
#define TEST_PATTERN                    0x484D
#define FE_DLY                          0x484E
#define CLK_LANE_TEST_PATTERN           0x484F
#define MIPI8LANE_MODE                  0x4850
//#define MIPI_CTRL_00                    0x4A00
//#define MIPI_CTRL_01                    0x4A01
//#define MIPI_CTRL_02                    0x4A02
//#define MIPI_CTRL_04                    0x4A04
//#define MIPI_MAX_FRAME_COUNT_HI         0x4A10
//#define MIPI_MAX_FRAME_COUNT_LO         0x4A11
//#define MIPI_CTRL14                     0x4A14
//#define MIPI_CTRL15                     0x4A15
//#define HS_ZERO_MIN_HI                  0x4A18
//#define HS_ZERO_MIN_LO                  0x4A19
//#define HS_TRAIL_MIN_HI                 0x4A1A
//#define HS_TRAIL_MIN_LO                 0x4A1B
//#define CLK_ZERO_MIN_HI                 0x4A1C
//#define CLK_ZERO_MIN_LO                 0x4A1D
//#define CLK_PREPARE_MIN_HI              0x4A1E
//#define CLK_PREPARE_MIN_LO              0x4A1F
//#define CLK_POST_MIN_HI                 0x4A20
//#define CLK_POST_MIN_LO                 0x4A21
//#define CLK_TRAIL_MIN_HI                0x4A22
//#define CLK_TRAIL_MIN_LO                0x4A23
//#define LPX_P_MIN_HI                    0x4A24
//#define LPX_P_MIN_LO                    0x4A25
//#define HS_PREPARE_MIN_HI               0x4A26
//#define HS_PREPARE_MIN_LO               0x4A27
//#define HS_EXIT_MIN_HI                  0x4A28
//#define HS_EXIT_MIN_LO                  0x4A29
//#define UI_HS_ZERO_MIN                  0x4A2A
//#define UI_HS_TRAIL_MIN                 0x4A2B
//#define UI_CLK_ZERO_MIN                 0x4A2C
//#define UI_CLK_PREPARE_MIN              0x4A2D
//#define UI_CLK_POST_MIN                 0x4A2E
//#define UI_CLK_TRAIL_MIN                0x4A2F
//#define UI_LPX_P_MIN                    0x4A30
//#define UI_HS_PREPARE_MIN               0x4A31
//#define UI_HS_EXIT_MIN                  0x4A32
//#define PCLK_PERIOD                     0x4A37
//#define MIPI_LP_GPIO_0                  0x4A38
//#define MIPI_LP_GPIO_1                  0x4A3A
//#define MIPI_LP_GPIO                    0x4A3B
//#define MIPI_CTRL_3C                    0x4A3C
//#define MIPI_LP_GPIO_3                  0x4A3D
//#define MIPI_CTRL4C                     0x4A4C
//#define TEST_PATTERN                    0x4A4D
//#define FE_DLY                          0x4A4E
#define CLK_LANE_TEST                   0x4A4F

/* ISP frame control */
//#define FRAME_CONTROL00                 0x4900
//#define FRAME_CONTROL01                 0x4901
//#define FRAME_CONTROL02                 0x4902
//#define FRAME_CONTROL03                 0x4903

/* LVDS control */
#define LVDS_REG0                       0x4B00
#define LVDS_REG2                       0x4B02
#define LVDS_REG3                       0x4B03
#define LVDS_REG4                       0x4B04
#define LVDS_REG5                       0x4B05
#define LVDS_REG6                       0x4B06
#define LVDS_REG7                       0x4B07
#define LVDS_REG8                       0x4B08
#define LVDS_REG9                       0x4B09
#define LVDS_REGA                       0x4B0A

/* DSP general control */
#define ISP_CTRL00                      0x5000
#define ISP_CTRL01                      0x5001
#define ISP_CTRL04                      0x5004
#define ISP_CTRL0C                      0x500C
#define ISP_CTRL0D                      0x500D
#define ISP_CTRL0E                      0x500E
#define ISP_CTRL0F                      0x500F
#define ISP_CTRL10                      0x5010
#define ISP_CTRL11                      0x5011

/* OTP cluster correction control */
#define OTP_CTRL00                      0x5500
#define OTP_CTRL01                      0x5501
#define OTP_CTRL02                      0x5502
#define OTP_CTRL03                      0x5503
#define OTP_CTRL05                      0x5505
#define OTP_CTRL06                      0x5506
#define OTP_CTRL07                      0x5507
#define OTP_CTRL08                      0x5508
#define OTP_CTRL09                      0x5509

/* windowing and scaling control */
#define WINC_CTRL00                     0x5980
#define WINC_CTRL01                     0x5981
#define WINC_CTRL02                     0x5982
#define WINC_CTRL03                     0x5983
#define WINC_CTRL04                     0x5984
#define WINC_CTRL05                     0x5985
#define WINC_CTRL06                     0x5986
#define WINC_CTRL07                     0x5987
#define WINC_CTRL08                     0x5988
#define SCALE_CTRL00                    0x5A00
#define SCALE_CTRL01                    0x5A01
#define SCALE_CTRL02                    0x5A02

#endif //__OV16825__H__
