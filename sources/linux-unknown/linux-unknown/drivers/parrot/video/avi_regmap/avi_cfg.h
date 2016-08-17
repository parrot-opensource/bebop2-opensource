/*********************************************************************
 * avi_cfg register map
 *
 * Vendor:   Parrot
 * Library:  AVI
 * Version:  P7R3
 * Gen-date: (Date of generation of this C code, not the IP-Xact file)
 *           2014-01-08
 *
 * WARNING: This code is automatically generated from the hardware
 * IP-Xact XML files. Do not edit directly.
 *********************************************************************/

#ifndef _AVI_CFG_H_
#define _AVI_CFG_H_

#define AVI_CFG_ENABLE1                           0x00
#define AVI_CFG_ENABLE2                           0x04
#define AVI_CFG_APPLY1                            0x08
#define AVI_CFG_APPLY2                            0x0c
#define AVI_CFG_LOCK1                             0x10
#define AVI_CFG_LOCK2                             0x14
#define AVI_CFG_ITFLG1                            0x20
#define AVI_CFG_ITFLG2                            0x24
#define AVI_CFG_ITEN1                             0x28
#define AVI_CFG_ITEN2                             0x2c
#define AVI_CFG_ITACK1                            0x30
#define AVI_CFG_ITACK2                            0x34
#define AVI_CFG_DMACFG                            0x40
#define AVI_CFG_ISP_ENABLE3                       0x44
#define AVI_CFG_ISP_APPLY3                        0x48
#define AVI_CFG_ISP_LOCK3                         0x4c
#define AVI_CFG_ISP_ITFLG3                        0x50
#define AVI_CFG_ISP_ITEN3                         0x54
#define AVI_CFG_ISP_ITACK3                        0x58

union avi_cfg_enable1
{
	struct
	{
		uint32_t cfg_enable      :  1;
		uint32_t inter_enable    :  1;
		uint32_t fifo00_enable   :  1;
		uint32_t fifo01_enable   :  1;
		uint32_t fifo02_enable   :  1;
		uint32_t fifo03_enable   :  1;
		uint32_t fifo04_enable   :  1;
		uint32_t fifo05_enable   :  1;
		uint32_t fifo06_enable   :  1;
		uint32_t fifo07_enable   :  1;
		uint32_t fifo08_enable   :  1;
		uint32_t fifo09_enable   :  1;
		uint32_t fifo10_enable   :  1;
		uint32_t fifo11_enable   :  1;
	};
	uint32_t _register;
};

union avi_cfg_enable2
{
	struct
	{
		uint32_t conv0_enable    :  1;
		uint32_t conv1_enable    :  1;
		uint32_t conv2_enable    :  1;
		uint32_t conv3_enable    :  1;
		uint32_t blend0_enable   :  1;
		uint32_t blend1_enable   :  1;
		uint32_t lcd0_enable     :  1;
		uint32_t lcd1_enable     :  1;
		uint32_t cam0_enable     :  1;
		uint32_t cam1_enable     :  1;
		uint32_t cam2_enable     :  1;
		uint32_t cam3_enable     :  1;
		uint32_t cam4_enable     :  1;
		uint32_t cam5_enable     :  1;
		uint32_t scal0_enable    :  1;
		uint32_t rot0_enable     :  1;
		uint32_t scal1_enable    :  1;
		uint32_t rot1_enable     :  1;
		uint32_t gam0_enable     :  1;
		uint32_t gam1_enable     :  1;
		uint32_t fork0_enable    :  1;
		uint32_t fork1_enable    :  1;
		uint32_t fork2_enable    :  1;
		uint32_t fork3_enable    :  1;
		uint32_t sat0_enable     :  1;
		uint32_t sat1_enable     :  1;
		uint32_t stats_yuv0_enable :  1;
		uint32_t stats_yuv1_enable :  1;
	};
	uint32_t _register;
};

union avi_cfg_apply1
{
	struct
	{
		uint32_t cfg_apply       :  1;
		uint32_t inter_apply     :  1;
		uint32_t fifo00_apply    :  1;
		uint32_t fifo01_apply    :  1;
		uint32_t fifo02_apply    :  1;
		uint32_t fifo03_apply    :  1;
		uint32_t fifo04_apply    :  1;
		uint32_t fifo05_apply    :  1;
		uint32_t fifo06_apply    :  1;
		uint32_t fifo07_apply    :  1;
		uint32_t fifo08_apply    :  1;
		uint32_t fifo09_apply    :  1;
		uint32_t fifo10_apply    :  1;
		uint32_t fifo11_apply    :  1;
	};
	uint32_t _register;
};

union avi_cfg_apply2
{
	struct
	{
		uint32_t conv0_apply     :  1;
		uint32_t conv1_apply     :  1;
		uint32_t conv2_apply     :  1;
		uint32_t conv3_apply     :  1;
		uint32_t blend0_apply    :  1;
		uint32_t blend1_apply    :  1;
		uint32_t lcd0_apply      :  1;
		uint32_t lcd1_apply      :  1;
		uint32_t cam0_apply      :  1;
		uint32_t cam1_apply      :  1;
		uint32_t cam2_apply      :  1;
		uint32_t cam3_apply      :  1;
		uint32_t cam4_apply      :  1;
		uint32_t cam5_apply      :  1;
		uint32_t scal0_apply     :  1;
		uint32_t rot0_apply      :  1;
		uint32_t scal1_apply     :  1;
		uint32_t rot1_apply      :  1;
		uint32_t gam0_apply      :  1;
		uint32_t gam1_apply      :  1;
		uint32_t fork0_apply     :  1;
		uint32_t fork1_apply     :  1;
		uint32_t fork2_apply     :  1;
		uint32_t fork3_apply     :  1;
		uint32_t sat0_apply      :  1;
		uint32_t sat1_apply      :  1;
		uint32_t stats_yuv0_apply :  1;
		uint32_t stats_yuv1_apply :  1;
	};
	uint32_t _register;
};

union avi_cfg_lock1
{
	struct
	{
		uint32_t cfg_lock        :  1;
		uint32_t inter_lock      :  1;
		uint32_t fifo00_lock     :  1;
		uint32_t fifo01_lock     :  1;
		uint32_t fifo02_lock     :  1;
		uint32_t fifo03_lock     :  1;
		uint32_t fifo04_lock     :  1;
		uint32_t fifo05_lock     :  1;
		uint32_t fifo06_lock     :  1;
		uint32_t fifo07_lock     :  1;
		uint32_t fifo08_lock     :  1;
		uint32_t fifo09_lock     :  1;
		uint32_t fifo10_lock     :  1;
		uint32_t fifo11_lock     :  1;
	};
	uint32_t _register;
};

union avi_cfg_lock2
{
	struct
	{
		uint32_t conv0_lock      :  1;
		uint32_t conv1_lock      :  1;
		uint32_t conv2_lock      :  1;
		uint32_t conv3_lock      :  1;
		uint32_t blend0_lock     :  1;
		uint32_t blend1_lock     :  1;
		uint32_t lcd0_lock       :  1;
		uint32_t lcd1_lock       :  1;
		uint32_t cam0_lock       :  1;
		uint32_t cam1_lock       :  1;
		uint32_t cam2_lock       :  1;
		uint32_t cam3_lock       :  1;
		uint32_t cam4_lock       :  1;
		uint32_t cam5_lock       :  1;
		uint32_t scal0_lock      :  1;
		uint32_t rot0_lock       :  1;
		uint32_t scal1_lock      :  1;
		uint32_t rot1_lock       :  1;
		uint32_t gam0_lock       :  1;
		uint32_t gam1_lock       :  1;
		uint32_t fork0_lock      :  1;
		uint32_t fork1_lock      :  1;
		uint32_t fork2_lock      :  1;
		uint32_t fork3_lock      :  1;
		uint32_t sat0_lock       :  1;
		uint32_t sat1_lock       :  1;
		uint32_t stats_yuv0_lock :  1;
		uint32_t stats_yuv1_lock :  1;
	};
	uint32_t _register;
};

union avi_cfg_itflg1
{
	struct
	{
		uint32_t cfg_itflg       :  1;
		uint32_t inter_itflg     :  1;
		uint32_t fifo00_itflg    :  1;
		uint32_t fifo01_itflg    :  1;
		uint32_t fifo02_itflg    :  1;
		uint32_t fifo03_itflg    :  1;
		uint32_t fifo04_itflg    :  1;
		uint32_t fifo05_itflg    :  1;
		uint32_t fifo06_itflg    :  1;
		uint32_t fifo07_itflg    :  1;
		uint32_t fifo08_itflg    :  1;
		uint32_t fifo09_itflg    :  1;
		uint32_t fifo10_itflg    :  1;
		uint32_t fifo11_itflg    :  1;
	};
	uint32_t _register;
};

union avi_cfg_itflg2
{
	struct
	{
		uint32_t conv0_itflg     :  1;
		uint32_t conv1_itflg     :  1;
		uint32_t conv2_itflg     :  1;
		uint32_t conv3_itflg     :  1;
		uint32_t blend0_itflg    :  1;
		uint32_t blend1_itflg    :  1;
		uint32_t lcd0_itflg      :  1;
		uint32_t lcd1_itflg      :  1;
		uint32_t cam0_itflg      :  1;
		uint32_t cam1_itflg      :  1;
		uint32_t cam2_itflg      :  1;
		uint32_t cam3_itflg      :  1;
		uint32_t cam4_itflg      :  1;
		uint32_t cam5_itflg      :  1;
		uint32_t scal0_itflg     :  1;
		uint32_t rot0_itflg      :  1;
		uint32_t scal1_itflg     :  1;
		uint32_t rot1_itflg      :  1;
		uint32_t gam0_itflg      :  1;
		uint32_t gam1_itflg      :  1;
		uint32_t fork0_itflg     :  1;
		uint32_t fork1_itflg     :  1;
		uint32_t fork2_itflg     :  1;
		uint32_t fork3_itflg     :  1;
		uint32_t sat0_itflg      :  1;
		uint32_t sat1_itflg      :  1;
		uint32_t stats_yuv0_itflg :  1;
		uint32_t stats_yuv1_itflg :  1;
	};
	uint32_t _register;
};

union avi_cfg_iten1
{
	struct
	{
		uint32_t cfg_iten        :  1;
		uint32_t inter_iten      :  1;
		uint32_t fifo00_iten     :  1;
		uint32_t fifo01_iten     :  1;
		uint32_t fifo02_iten     :  1;
		uint32_t fifo03_iten     :  1;
		uint32_t fifo04_iten     :  1;
		uint32_t fifo05_iten     :  1;
		uint32_t fifo06_iten     :  1;
		uint32_t fifo07_iten     :  1;
		uint32_t fifo08_iten     :  1;
		uint32_t fifo09_iten     :  1;
		uint32_t fifo10_iten     :  1;
		uint32_t fifo11_iten     :  1;
	};
	uint32_t _register;
};

union avi_cfg_iten2
{
	struct
	{
		uint32_t conv0_iten      :  1;
		uint32_t conv1_iten      :  1;
		uint32_t conv2_iten      :  1;
		uint32_t conv3_iten      :  1;
		uint32_t blend0_iten     :  1;
		uint32_t blend1_iten     :  1;
		uint32_t lcd0_iten       :  1;
		uint32_t lcd1_iten       :  1;
		uint32_t cam0_iten       :  1;
		uint32_t cam1_iten       :  1;
		uint32_t cam2_iten       :  1;
		uint32_t cam3_iten       :  1;
		uint32_t cam4_iten       :  1;
		uint32_t cam5_iten       :  1;
		uint32_t scal0_iten      :  1;
		uint32_t rot0_iten       :  1;
		uint32_t scal1_iten      :  1;
		uint32_t rot1_iten       :  1;
		uint32_t gam0_iten       :  1;
		uint32_t gam1_iten       :  1;
		uint32_t fork0_iten      :  1;
		uint32_t fork1_iten      :  1;
		uint32_t fork2_iten      :  1;
		uint32_t fork3_iten      :  1;
		uint32_t sat0_iten       :  1;
		uint32_t sat1_iten       :  1;
		uint32_t stats_yuv0_iten :  1;
		uint32_t stats_yuv1_iten :  1;
	};
	uint32_t _register;
};

union avi_cfg_itack1
{
	struct
	{
		uint32_t cfg_itack       :  1;
		uint32_t inter_itack     :  1;
		uint32_t fifo00_itack    :  1;
		uint32_t fifo01_itack    :  1;
		uint32_t fifo02_itack    :  1;
		uint32_t fifo03_itack    :  1;
		uint32_t fifo04_itack    :  1;
		uint32_t fifo05_itack    :  1;
		uint32_t fifo06_itack    :  1;
		uint32_t fifo07_itack    :  1;
		uint32_t fifo08_itack    :  1;
		uint32_t fifo09_itack    :  1;
		uint32_t fifo10_itack    :  1;
		uint32_t fifo11_itack    :  1;
	};
	uint32_t _register;
};

union avi_cfg_itack2
{
	struct
	{
		uint32_t conv0_itack     :  1;
		uint32_t conv1_itack     :  1;
		uint32_t conv2_itack     :  1;
		uint32_t conv3_itack     :  1;
		uint32_t blend0_itack    :  1;
		uint32_t blend1_itack    :  1;
		uint32_t lcd0_itack      :  1;
		uint32_t lcd1_itack      :  1;
		uint32_t cam0_itack      :  1;
		uint32_t cam1_itack      :  1;
		uint32_t cam2_itack      :  1;
		uint32_t cam3_itack      :  1;
		uint32_t cam4_itack      :  1;
		uint32_t cam5_itack      :  1;
		uint32_t scal0_itack     :  1;
		uint32_t rot0_itack      :  1;
		uint32_t scal1_itack     :  1;
		uint32_t rot1_itack      :  1;
		uint32_t gam0_itack      :  1;
		uint32_t gam1_itack      :  1;
		uint32_t fork0_itack     :  1;
		uint32_t fork1_itack     :  1;
		uint32_t fork2_itack     :  1;
		uint32_t fork3_itack     :  1;
		uint32_t sat0_itack      :  1;
		uint32_t sat1_itack      :  1;
		uint32_t stats_yuv0_itack :  1;
		uint32_t stats_yuv1_itack :  1;
	};
	uint32_t _register;
};

union avi_cfg_dmacfg
{
	struct
	{
		uint32_t dma_flag_timeout : 16;
		uint32_t dma_flag_number :  4;
		uint32_t dma_max_burst_number :  4;
		uint32_t dma_max_bandwidth :  8;
	};
	uint32_t _register;
};

union avi_cfg_isp_enable3
{
	struct
	{
		uint32_t inter0_enable   :  1;
		uint32_t vlformat_32to40_0_enable :  1;
		uint32_t pedestal0_enable :  1;
		uint32_t grim0_enable    :  1;
		uint32_t rip0_enable     :  1;
		uint32_t denoise0_enable :  1;
		uint32_t stats_bayer0_enable :  1;
		uint32_t lsc0_enable     :  1;
		uint32_t chroma_aber0_enable :  1;
		uint32_t bayer0_enable   :  1;
		uint32_t color_matrix0_enable :  1;
		uint32_t vlformat_40to32_0_enable :  1;
		uint32_t inter1_enable   :  1;
		uint32_t vlformat_32to40_1_enable :  1;
		uint32_t pedestal1_enable :  1;
		uint32_t grim1_enable    :  1;
		uint32_t rip1_enable     :  1;
		uint32_t denoise1_enable :  1;
		uint32_t stats_bayer1_enable :  1;
		uint32_t lsc1_enable     :  1;
		uint32_t chroma_aber1_enable :  1;
		uint32_t bayer1_enable   :  1;
		uint32_t color_matrix1_enable :  1;
		uint32_t vlformat_40to32_1_enable :  1;
		uint32_t ee_crf_i3d_lut_drop_inter0_enable :  1;
		uint32_t ee_crf0_enable  :  1;
		uint32_t i3d_lut0_enable :  1;
		uint32_t drop0_enable    :  1;
		uint32_t ee_crf_i3d_lut_drop_inter1_enable :  1;
		uint32_t ee_crf1_enable  :  1;
		uint32_t i3d_lut1_enable :  1;
		uint32_t drop1_enable    :  1;
	};
	uint32_t _register;
};

union avi_cfg_isp_apply3
{
	struct
	{
		uint32_t inter0_apply    :  1;
		uint32_t vlformat_32to40_0_apply :  1;
		uint32_t pedestal0_apply :  1;
		uint32_t grim0_apply     :  1;
		uint32_t rip0_apply      :  1;
		uint32_t denoise0_apply  :  1;
		uint32_t stats_bayer0_apply :  1;
		uint32_t lsc0_apply      :  1;
		uint32_t chroma_aber0_apply :  1;
		uint32_t bayer0_apply    :  1;
		uint32_t color_matrix0_apply :  1;
		uint32_t vlformat_40to32_0_apply :  1;
		uint32_t inter1_apply    :  1;
		uint32_t vlformat_32to40_1_apply :  1;
		uint32_t pedestal1_apply :  1;
		uint32_t grim1_apply     :  1;
		uint32_t rip1_apply      :  1;
		uint32_t denoise1_apply  :  1;
		uint32_t stats_bayer1_apply :  1;
		uint32_t lsc1_apply      :  1;
		uint32_t chroma_aber1_apply :  1;
		uint32_t bayer1_apply    :  1;
		uint32_t color_matrix1_apply :  1;
		uint32_t vlformat_40to32_1_apply :  1;
		uint32_t ee_crf_i3d_lut_drop_inter0_apply :  1;
		uint32_t ee_crf0_apply   :  1;
		uint32_t i3d_lut0_apply  :  1;
		uint32_t drop0_apply     :  1;
		uint32_t ee_crf_i3d_lut_drop_inter1_apply :  1;
		uint32_t ee_crf1_apply   :  1;
		uint32_t i3d_lut1_apply  :  1;
		uint32_t drop1_apply     :  1;
	};
	uint32_t _register;
};

union avi_cfg_isp_lock3
{
	struct
	{
		uint32_t inter0_lock     :  1;
		uint32_t vlformat_32to40_0_lock :  1;
		uint32_t pedestal0_lock  :  1;
		uint32_t grim0_lock      :  1;
		uint32_t rip0_lock       :  1;
		uint32_t denoise0_lock   :  1;
		uint32_t stats_bayer0_lock :  1;
		uint32_t lsc0_lock       :  1;
		uint32_t chroma_aber0_lock :  1;
		uint32_t bayer0_lock     :  1;
		uint32_t color_matrix0_lock :  1;
		uint32_t vlformat_40to32_0_lock :  1;
		uint32_t inter1_lock     :  1;
		uint32_t vlformat_32to40_1_lock :  1;
		uint32_t pedestal1_lock  :  1;
		uint32_t grim1_lock      :  1;
		uint32_t rip1_lock       :  1;
		uint32_t denoise1_lock   :  1;
		uint32_t stats_bayer1_lock :  1;
		uint32_t lsc1_lock       :  1;
		uint32_t chroma_aber1_lock :  1;
		uint32_t bayer1_lock     :  1;
		uint32_t color_matrix1_lock :  1;
		uint32_t vlformat_40to32_1_lock :  1;
		uint32_t ee_crf_i3d_lut_drop_inter0_lock :  1;
		uint32_t ee_crf0_lock    :  1;
		uint32_t i3d_lut0_lock   :  1;
		uint32_t drop0_lock      :  1;
		uint32_t ee_crf_i3d_lut_drop_inter1_lock :  1;
		uint32_t ee_crf1_lock    :  1;
		uint32_t i3d_lut1_lock   :  1;
		uint32_t drop1_lock      :  1;
	};
	uint32_t _register;
};

union avi_cfg_isp_itflg3
{
	struct
	{
		uint32_t inter0_itflg    :  1;
		uint32_t vlformat_32to40_0_itflg :  1;
		uint32_t pedestal0_itflg :  1;
		uint32_t grim0_itflg     :  1;
		uint32_t rip0_itflg      :  1;
		uint32_t denoise0_itflg  :  1;
		uint32_t stats_bayer0_itflg :  1;
		uint32_t lsc0_itflg      :  1;
		uint32_t chroma_aber0_itflg :  1;
		uint32_t bayer0_itflg    :  1;
		uint32_t color_matrix0_itflg :  1;
		uint32_t vlformat_40to32_0_itflg :  1;
		uint32_t inter1_itflg    :  1;
		uint32_t vlformat_32to40_1_itflg :  1;
		uint32_t pedestal1_itflg :  1;
		uint32_t grim1_itflg     :  1;
		uint32_t rip1_itflg      :  1;
		uint32_t denoise1_itflg  :  1;
		uint32_t stats_bayer1_itflg :  1;
		uint32_t lsc1_itflg      :  1;
		uint32_t chroma_aber1_itflg :  1;
		uint32_t bayer1_itflg    :  1;
		uint32_t color_matrix1_itflg :  1;
		uint32_t vlformat_40to32_1_itflg :  1;
		uint32_t ee_crf_i3d_lut_drop_inter0_itflg :  1;
		uint32_t ee_crf0_itflg   :  1;
		uint32_t i3d_lut0_itflg  :  1;
		uint32_t drop0_itflg     :  1;
		uint32_t ee_crf_i3d_lut_drop_inter1_itflg :  1;
		uint32_t ee_crf1_itflg   :  1;
		uint32_t i3d_lut1_itflg  :  1;
		uint32_t drop1_itflg     :  1;
	};
	uint32_t _register;
};

union avi_cfg_isp_iten3
{
	struct
	{
		uint32_t inter0_iten     :  1;
		uint32_t vlformat_32to40_0_iten :  1;
		uint32_t pedestal0_iten  :  1;
		uint32_t grim0_iten      :  1;
		uint32_t rip0_iten       :  1;
		uint32_t denoise0_iten   :  1;
		uint32_t stats_bayer0_iten :  1;
		uint32_t lsc0_iten       :  1;
		uint32_t chroma_aber0_iten :  1;
		uint32_t bayer0_iten     :  1;
		uint32_t color_matrix0_iten :  1;
		uint32_t vlformat_40to32_0_iten :  1;
		uint32_t inter1_iten     :  1;
		uint32_t vlformat_32to40_1_iten :  1;
		uint32_t pedestal1_iten  :  1;
		uint32_t grim1_iten      :  1;
		uint32_t rip1_iten       :  1;
		uint32_t denoise1_iten   :  1;
		uint32_t stats_bayer1_iten :  1;
		uint32_t lsc1_iten       :  1;
		uint32_t chroma_aber1_iten :  1;
		uint32_t bayer1_iten     :  1;
		uint32_t color_matrix1_iten :  1;
		uint32_t vlformat_40to32_1_iten :  1;
		uint32_t ee_crf_i3d_lut_drop_inter0_iten :  1;
		uint32_t ee_crf0_iten    :  1;
		uint32_t i3d_lut0_iten   :  1;
		uint32_t drop0_iten      :  1;
		uint32_t ee_crf_i3d_lut_drop_inter1_iten :  1;
		uint32_t ee_crf1_iten    :  1;
		uint32_t i3d_lut1_iten   :  1;
		uint32_t drop1_iten      :  1;
	};
	uint32_t _register;
};

union avi_cfg_isp_itack3
{
	struct
	{
		uint32_t inter0_itack    :  1;
		uint32_t vlformat_32to40_0_itack :  1;
		uint32_t pedestal0_itack :  1;
		uint32_t grim0_itack     :  1;
		uint32_t rip0_itack      :  1;
		uint32_t denoise0_itack  :  1;
		uint32_t stats_bayer0_itack :  1;
		uint32_t lsc0_itack      :  1;
		uint32_t chroma_aber0_itack :  1;
		uint32_t bayer0_itack    :  1;
		uint32_t color_matrix0_itack :  1;
		uint32_t vlformat_40to32_0_itack :  1;
		uint32_t inter1_itack    :  1;
		uint32_t vlformat_32to40_1_itack :  1;
		uint32_t pedestal1_itack :  1;
		uint32_t grim1_itack     :  1;
		uint32_t rip1_itack      :  1;
		uint32_t denoise1_itack  :  1;
		uint32_t stats_bayer1_itack :  1;
		uint32_t lsc1_itack      :  1;
		uint32_t chroma_aber1_itack :  1;
		uint32_t bayer1_itack    :  1;
		uint32_t color_matrix1_itack :  1;
		uint32_t vlformat_40to32_1_itack :  1;
		uint32_t ee_crf_i3d_lut_drop_inter0_itack :  1;
		uint32_t ee_crf0_itack   :  1;
		uint32_t i3d_lut0_itack  :  1;
		uint32_t drop0_itack     :  1;
		uint32_t ee_crf_i3d_lut_drop_inter1_itack :  1;
		uint32_t ee_crf1_itack   :  1;
		uint32_t i3d_lut1_itack  :  1;
		uint32_t drop1_itack     :  1;
	};
	uint32_t _register;
};

struct avi_cfg_regs
{
	union avi_cfg_enable1                    enable1;                    /* 0x000 */
	union avi_cfg_enable2                    enable2;                    /* 0x004 */
	union avi_cfg_apply1                     apply1;                     /* 0x008 */
	union avi_cfg_apply2                     apply2;                     /* 0x00c */
	union avi_cfg_lock1                      lock1;                      /* 0x010 */
	union avi_cfg_lock2                      lock2;                      /* 0x014 */
	unsigned                                 /*unused*/ : 32;            /* 0x018 */
	unsigned                                 /*unused*/ : 32;            /* 0x01c */
	union avi_cfg_itflg1                     itflg1;                     /* 0x020 */
	union avi_cfg_itflg2                     itflg2;                     /* 0x024 */
	union avi_cfg_iten1                      iten1;                      /* 0x028 */
	union avi_cfg_iten2                      iten2;                      /* 0x02c */
	union avi_cfg_itack1                     itack1;                     /* 0x030 */
	union avi_cfg_itack2                     itack2;                     /* 0x034 */
	unsigned                                 /*unused*/ : 32;            /* 0x038 */
	unsigned                                 /*unused*/ : 32;            /* 0x03c */
	union avi_cfg_dmacfg                     dmacfg;                     /* 0x040 */
	union avi_cfg_isp_enable3                isp_enable3;                /* 0x044 */
	union avi_cfg_isp_apply3                 isp_apply3;                 /* 0x048 */
	union avi_cfg_isp_lock3                  isp_lock3;                  /* 0x04c */
	union avi_cfg_isp_itflg3                 isp_itflg3;                 /* 0x050 */
	union avi_cfg_isp_iten3                  isp_iten3;                  /* 0x054 */
	union avi_cfg_isp_itack3                 isp_itack3;                 /* 0x058 */
};

#endif /* _AVI_CFG_H_ */
