/*
 * mt9m021 - Aptina CMOS Digital Image Sensor
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Tue Aug 26 15:55:49 CEST 2014
 *
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>

#include <media/media-entity.h>
#include <media/mt9m021.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "aptina-pll.h"
#include "mt9m021_reg.h"

MODULE_AUTHOR("Eng-Hong SRON <eng-hong.sron@parrot.com>");
MODULE_DESCRIPTION("Aptina MT9M021 driver");
MODULE_LICENSE("GPL");

#define DRIVER_NAME "mt9m021"

#define MT9M021_CHIP_VERSION			0x3000
#define 	MT9M021_CHIP_VERSION_VALUE	0x2401
#define MT9M021_Y_ADDR_START			0x3002
#define MT9M021_X_ADDR_START			0x3004
#define MT9M021_Y_ADDR_END			0x3006
#define MT9M021_X_ADDR_END			0x3008
#define MT9M021_FRAME_LENGTH_LINES		0x300A
#define MT9M021_LINE_LENGTH_PCK			0x300C
#define 	MT9M021_MIN_LINE_LENGTH		  1650
#define MT9M021_COARSE_INT_TIME			0x3012
#define MT9M021_FINE_INTEGRATION_TIME		0x3014
#define MT9M021_RESET_REGISTER			0x301A
#define 	MT9M021_RESET			0x00D9
#define 	MT9M021_STREAM_OFF		0x00D8
#define 	MT9M021_STREAM_ON		0x00DC
#define 	MT9M021_TRIGGER_MODE		0x09D8
#define MT9M021_DATA_PEDESTAL			0x301E
#define MT9M021_ROW_SPEED			0x3028
#define MT9M021_VT_PIX_CLK_DIV			0x302A
#define MT9M021_VT_SYS_CLK_DIV			0x302C
#define MT9M021_PRE_PLL_CLK_DIV			0x302E
#define MT9M021_PLL_MULTIPLIER			0x3030
#define MT9M021_DIGITAL_BINNING			0x3032
#define 	MT9M021_HOR_AND_VER_BIN		0x0022
#define 	MT9M021_HOR_BIN			0x0011
#define 	MT9M021_DISABLE_BINNING		0x0000
#define MT9M021_READ_MODE			0x3040
#define MT9M021_DARK_CONTROL			0x3044
#define MT9M021_GLOBAL_GAIN                     0x305E
#define MT9M021_EMBEDDED_DATA_CTRL      	0x3064
#define MT9M021_TEST_RAW_MODE			0x307A
#define MT9M021_SEQ_DATA_PORT			0x3086
#define MT9M021_SEQ_CTRL_PORT			0x3088
#define MT9M021_X_ODD_INC			0x30A2
#define MT9M021_Y_ODD_INC			0x30A6
#define MT9M021_DIGITAL_TEST			0x30B0
#define MT9M021_COLUMN_CORRECTION		0x30D4
#define MT9M021_RESERVED_MFR_30EA		0x30EA
#define MT9M021_AE_CTRL_REGISTER		0x3100
#define 	MT9M021_AE_ENABLE		0x0013
#define MT9M021_AE_MAX_EXPOSURE			0x311C
#define MT9M021_AE_MIN_EXPOSURE			0x311E
#define MT9M021_AE_TARGET_LUMA			0x3102
#define MT9M021_AE_ROI_X_START			0x3140
#define MT9M021_AE_ROI_Y_START			0x3142
#define MT9M021_AE_ROI_X_SIZE			0x3144
#define MT9M021_AE_ROI_Y_SIZE			0x3146
#define MT9M021_AE_AG_EXPOSURE_HI		0x3166
#define MT9M021_AE_AG_EXPOSURE_LO		0x3168
#define MT9M021_AE_MIN_EV_STEP			0x3108
#define MT9M021_AE_MAX_EV_STEP			0x310A
#define MT9M021_RESERVED_MFR_3180		0x3180
#define MT9M021_ANALOG_REG                      0x3ED6
#define MT9M021_TEMPSENS_CTRL                   0x30B4
#define         MT9M021_TEMPSENS_EN             0x0011

/* Embedded data adds 4 lines for register dump and statistics
 * and 6 lines for delta dark rows so we have to remove 4+6 more
 * lines from height. */
#define EMBEDDED_DATA_HEIGHT                    (4)
#define EMBEDDED_DARK_HEIGHT                    (6)

struct mt9m021 {
	struct v4l2_subdev            sd;
	struct media_pad              pad;
	struct v4l2_rect              crop;
	struct v4l2_mbus_framefmt     format;
	struct v4l2_fract             frame_interval;
	struct mt9m021_platform_data *pdata;
	struct aptina_pll             pll;

	struct v4l2_ctrl_handler      ctrl_handler;
	struct v4l2_ctrl             *trigger;
	struct v4l2_ctrl             *exposure_mode;
	struct v4l2_ctrl             *exposure_abs;
	struct v4l2_ctrl             *exposure_ae_max;
	struct v4l2_ctrl             *exposure_ae_min;
	struct v4l2_ctrl             *ae_tgt_luma;
	struct v4l2_ctrl             *ae_ag_exposure_hi;
	struct v4l2_ctrl             *ae_ag_exposure_lo;
	struct v4l2_ctrl             *auto_aag;
	struct v4l2_ctrl             *auto_adg;
	struct v4l2_ctrl             *gain;
	struct v4l2_ctrl             *ana_gain;
	struct v4l2_ctrl             *ae_min_ev_step;
	struct v4l2_ctrl             *ae_max_ev_step;
	/* Auto-Exposure Region Of Interest */
	struct v4l2_ctrl             *ae_roi_x_start;
	struct v4l2_ctrl             *ae_roi_y_start;
	struct v4l2_ctrl             *ae_roi_x_size;
	struct v4l2_ctrl             *ae_roi_y_size;

	u8                            streaming;
};

static inline struct mt9m021 *to_mt9m021(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m021, sd);
}

static inline struct mt9m021 *ctrl_to_mt9m021(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mt9m021, ctrl_handler);
}

static int mt9m021_read(struct v4l2_subdev *sd, u16 reg, u16 *val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = (u8 *)&reg,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = (u8 *)val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	*val = swab16(*val);

	return 0;
}

static int mt9m021_write(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	u16 buf[] = {
		swab16(reg),
		swab16(val),
	};

	struct i2c_msg msg = {
		.addr  = client->addr,
		.flags = 0,
		.len   = 4,
		.buf   = (u8 *)&buf,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9m021_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	int ret;
	u16 val;

	reg->size = 2;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	ret = mt9m021_read(sd, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = (__u64)val;

	return 0;
}

static int mt9m021_set_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	if (reg->reg & ~0xffff || reg->val & ~0xffff)
		return -EINVAL;

	return mt9m021_write(sd, reg->reg, reg->val);
}
#endif

static const struct v4l2_subdev_core_ops mt9m021_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = mt9m021_get_register,
	.s_register = mt9m021_set_register,
#endif
};

static int mt9m021_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct mt9m021            *mt9m021 = to_mt9m021(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, 0);
		fmt->format = *mf;
		return 0;
	}

	fmt->format = mt9m021->format;

	return 0;
}

static int mt9m021_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct mt9m021            *mt9m021 = to_mt9m021(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		fmt->format = *mf;
		return 0;
	}

	mt9m021->format = *mf;

	return 0;
}

static int mt9m021_enum_mbus_code(struct v4l2_subdev                *sd,
				  struct v4l2_subdev_fh             *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9m021 *mt9m021 = to_mt9m021(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = mt9m021->format.code;

	return 0;
}

static int mt9m021_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_selection *sel)
{
	struct mt9m021 *mt9m021 = to_mt9m021(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left   =    0;
		sel->r.top    =    0;
		sel->r.width  = 1280;
		sel->r.height =  960;
		break;

	case V4L2_SEL_TGT_CROP:
		sel->r = mt9m021->crop;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int mt9m021_set_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_selection *sel)
{
	struct mt9m021 *mt9m021 = to_mt9m021(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		mt9m021->crop = sel->r;
		break;

	default:
		v4l2_err(sd, "selection target (%d) not supported yet\n",
			     sel->target);
		return -EINVAL;
	}

	return 0;
}

/*
 * A-1000GS Rev2 Optimized Sequencer 04-07-11
 */
static u16 mt9m021_seq_data[] = {
	0x3227, 0x0101, 0x0F25, 0x0808, 0x0227, 0x0101, 0x0837, 0x2700,
	0x0138, 0x2701, 0x013A, 0x2700, 0x0125, 0x0020, 0x3C25, 0x0040,
	0x3427, 0x003F, 0x2500, 0x2037, 0x2540, 0x4036, 0x2500, 0x4031,
	0x2540, 0x403D, 0x6425, 0x2020, 0x3D64, 0x2510, 0x1037, 0x2520,
	0x2010, 0x2510, 0x100F, 0x2708, 0x0802, 0x2540, 0x402D, 0x2608,
	0x280D, 0x1709, 0x2600, 0x2805, 0x26A7, 0x2807, 0x2580, 0x8029,
	0x1705, 0x2500, 0x4027, 0x2222, 0x1616, 0x2726, 0x2617, 0x3626,
	0xA617, 0x0326, 0xA417, 0x1F28, 0x0526, 0x2028, 0x0425, 0x2020,
	0x2700, 0x2625, 0x0000, 0x171E, 0x2500, 0x0425, 0x0020, 0x2117,
	0x121B, 0x1703, 0x2726, 0x2617, 0x2828, 0x0517, 0x1A26, 0x6017,
	0xAE25, 0x0080, 0x2700, 0x2626, 0x1828, 0x002E, 0x2A28, 0x081E,
	0x4127, 0x1010, 0x0214, 0x6060, 0x0A14, 0x6060, 0x0B14, 0x6060,
	0x0C14, 0x6060, 0x0D14, 0x6060, 0x0217, 0x3C14, 0x0060, 0x0A14,
	0x0060, 0x0B14, 0x0060, 0x0C14, 0x0060, 0x0D14, 0x0060, 0x0811,
	0x2500, 0x1027, 0x0010, 0x2F6F, 0x0F3E, 0x2500, 0x0827, 0x0008,
	0x3066, 0x3225, 0x0008, 0x2700, 0x0830, 0x6631, 0x3D64, 0x2508,
	0x083D, 0xFF3D, 0x2A27, 0x083F, 0x2C00
};

static u16 mt9m021_analog_setting[] = {
	0x00FD, 0x0FFF, 0x0003, 0xF87A, 0xE075, 0x077C, 0xA4EB, 0xD208
};

static void mt9m021_sequencer_settings(struct v4l2_subdev *sd)
{
	int i;

	mt9m021_write(sd, MT9M021_SEQ_CTRL_PORT, 0x8000);

	for (i = 0 ; i < ARRAY_SIZE(mt9m021_seq_data) ; i++)
		mt9m021_write(sd, MT9M021_SEQ_DATA_PORT, mt9m021_seq_data[i]);

}

/*
 * Column Correction ReTriggering
 */
static void mt9m021_col_correction(struct v4l2_subdev *sd)
{
	/* Disable Streaming */
	mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_STREAM_OFF);

	/* Disable column correction */
	mt9m021_write(sd, MT9M021_COLUMN_CORRECTION, 0x0000);

	msleep(200);

	/* Enable Streaming */
	mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_STREAM_ON);

	msleep(200);

	/* Disable Streaming */
	mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_STREAM_OFF);

	/* Enable column correction */
	mt9m021_write(sd, MT9M021_COLUMN_CORRECTION, 0x0001);

	msleep(200);
}

/*
 * A-1000GS REV2 Optimized Settings
 */
static void mt9m021_rev2_settings(struct v4l2_subdev *sd)
{
	int i;

	mt9m021_write(sd, MT9M021_TEST_RAW_MODE,         0x0000);
	mt9m021_write(sd, MT9M021_RESERVED_MFR_30EA,     0x0C00);
	/* row_noise_correction_en + show_dark_extra_rows: */
	mt9m021_write(sd, MT9M021_DARK_CONTROL,          0x0C04);
	mt9m021_write(sd, MT9M021_DATA_PEDESTAL,         0x012C);
	mt9m021_write(sd, MT9M021_RESERVED_MFR_3180,     0x8000);
	mt9m021_write(sd, MT9M021_COLUMN_CORRECTION,     0xE007);
	mt9m021_write(sd, MT9M021_FINE_INTEGRATION_TIME, 0x0000);

	/* Analog Settings */
	for (i = 0 ; i < ARRAY_SIZE(mt9m021_analog_setting) ; i++)
		mt9m021_write(sd, MT9M021_ANALOG_REG + 2 * i,
				  mt9m021_analog_setting[i]);
}

static int mt9m021_pll_setup(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m021    *mt9m021 = to_mt9m021(sd);
	int                ret;

	/* For now, as we are only in parallel mode, we can consider p1 as fixed
	 * to 1. So we compute only p2
	 */
	static const struct aptina_pll_limits limits = {
		.ext_clock_min =   6000000,
		.ext_clock_max =  50000000,
		.int_clock_min =   3000000,
		.int_clock_max =  50000000,
		.out_clock_min = 384000000,
		.out_clock_max = 768000000,
		.pix_clock_max =  74250000,

		.n_min  =   1,
		.n_max  =  63,
		.m_min  =  32,
		.m_max  = 255,
		.p1_min =   4,
		.p1_max =  16,
	};

	ret = aptina_pll_calculate(&client->dev, &limits, &mt9m021->pll);
	if (ret)
		return ret;

	mt9m021_write(sd, MT9M021_VT_SYS_CLK_DIV,  1);
	mt9m021_write(sd, MT9M021_VT_PIX_CLK_DIV,  mt9m021->pll.p1);
	mt9m021_write(sd, MT9M021_PRE_PLL_CLK_DIV, mt9m021->pll.n);
	mt9m021_write(sd, MT9M021_PLL_MULTIPLIER,  mt9m021->pll.m);

	msleep(100);

	return 0;
}

static int mt9m021_set_size(struct v4l2_subdev *sd)
{
	struct mt9m021            *mt9m021 = to_mt9m021(sd);
	struct v4l2_rect          *c   = &mt9m021->crop;
	struct v4l2_mbus_framefmt *fmt = &mt9m021->format;

	int hratio = DIV_ROUND_CLOSEST(c->width,  fmt->width);
	int vratio = DIV_ROUND_CLOSEST(c->height,
	                               fmt->height - EMBEDDED_DATA_HEIGHT);
	u16 binning;
	s32 embedded_data_height = EMBEDDED_DATA_HEIGHT+EMBEDDED_DARK_HEIGHT;

	/* MT9M021 supports only 3 modes:
	 *   - No binning
	 *   - Horizontal only binning
	 *   - Horizontal and Vertical binning
	 */
	if (hratio == 2) {
		if (vratio == 2) {
			binning = MT9M021_HOR_AND_VER_BIN;
			/* In vertical binning, show_dark_extra_rows doesn't work.
			 * show_dark_extra_rows is going to be deactivated
			 */
			embedded_data_height = EMBEDDED_DATA_HEIGHT;
		} else {
			binning = MT9M021_HOR_BIN;
		}
	} else {
		binning = MT9M021_DISABLE_BINNING;
	}

	/* Check there is enough room to output embedded data lines */
	if (((c->height / vratio) + embedded_data_height) > fmt->height)
		return -1;

	if(binning == MT9M021_HOR_AND_VER_BIN) {
		/* In vertical binning, show_dark_extra_rows doesn't work.
		 * Should be a bug in MT9M021.
		 * So, deactivate it.
		 */
		mt9m021_write(sd, MT9M021_DARK_CONTROL, 0x0404);
	}

	mt9m021_write(sd, MT9M021_DIGITAL_BINNING, binning);
	/* Skipping: TODO in function of fmt / crop */
	mt9m021_write(sd, MT9M021_X_ODD_INC, 0x0001);
	mt9m021_write(sd, MT9M021_Y_ODD_INC, 0x0001);

	mt9m021_write(sd, MT9M021_Y_ADDR_START, mt9m021->crop.top);
	mt9m021_write(sd, MT9M021_X_ADDR_START, mt9m021->crop.left);
	mt9m021_write(sd, MT9M021_Y_ADDR_END, mt9m021->crop.top
	              + mt9m021->crop.height - 1);
	mt9m021_write(sd, MT9M021_X_ADDR_END,   mt9m021->crop.left +
						mt9m021->crop.width - 1);

	return 0;
}

static int mt9m021_apply_exposure(struct v4l2_subdev *sd)
{
	struct mt9m021     *mt9m021 = to_mt9m021(sd);
	u32                 exposure_mode     = mt9m021->exposure_mode->val;
	u32                 exposure_abs      = mt9m021->exposure_abs->val;
	u32                 exposure_ae_max   = mt9m021->exposure_ae_max->val;
	u32                 exposure_ae_min   = mt9m021->exposure_ae_min->val;
	u32                 ae_tgt_luma       = mt9m021->ae_tgt_luma->val;
	u32                 ae_ag_exposure_hi = mt9m021->ae_ag_exposure_hi->val;
	u32                 ae_ag_exposure_lo = mt9m021->ae_ag_exposure_lo->val;
	u32                 ae_min_ev_step    = mt9m021->ae_min_ev_step->val;
	u32                 ae_max_ev_step    = mt9m021->ae_max_ev_step->val;
	u32                 ae_roi_x_start    = mt9m021->ae_roi_x_start->val;
	u32                 ae_roi_y_start    = mt9m021->ae_roi_y_start->val;
	u32                 ae_roi_x_size     = mt9m021->ae_roi_x_size->val;
	u32                 ae_roi_y_size     = mt9m021->ae_roi_y_size->val;
	u32                 auto_aag          = mt9m021->auto_aag->val;
	u32                 auto_adg          = mt9m021->auto_adg->val;
	u32                 gain              = mt9m021->gain->val;
	u32                 ana_gain          = mt9m021->ana_gain->val;
	u32                 line_duration_ns;
	u16                 int_time;

	union ae_ctrl_reg   ae_ctrl_reg = { ._register = 0 };
	union dig_test_reg dig_test_reg = {
		.mono_chrome = 1,
		.col_gain    = ana_gain,
	};

	/* AAG and ADG both need Auto Exposure to function */
	if (exposure_mode == V4L2_EXPOSURE_AUTO || auto_aag || auto_adg)
		ae_ctrl_reg.ae_enable = 1;

	if (auto_aag)
		ae_ctrl_reg.auto_ag_en = 1;

	if (auto_adg)
		ae_ctrl_reg.auto_dg_en = 1;

	/* Integration time per line */
	line_duration_ns = div_u64((u64)1000000000 * MT9M021_MIN_LINE_LENGTH,
	                           mt9m021->pll.pix_clock);

	int_time = (exposure_abs * 1000) / line_duration_ns;

	mt9m021_write(sd, MT9M021_COARSE_INT_TIME, int_time);

	/* Max AE time */
	int_time = (exposure_ae_max * 1000) / line_duration_ns;
	mt9m021_write(sd, MT9M021_AE_MAX_EXPOSURE, int_time);

	/* Min AE time */
	int_time = (exposure_ae_min * 1000) / line_duration_ns;
	mt9m021_write(sd, MT9M021_AE_MIN_EXPOSURE, int_time);

	/* AE Luma Target */
	mt9m021_write(sd, MT9M021_AE_TARGET_LUMA, ae_tgt_luma);
	int_time = (ae_ag_exposure_hi * 1000) / line_duration_ns;
	mt9m021_write(sd, MT9M021_AE_AG_EXPOSURE_HI, int_time);
	int_time = (ae_ag_exposure_lo * 1000) / line_duration_ns;
	mt9m021_write(sd, MT9M021_AE_AG_EXPOSURE_LO, int_time);

	/* AE Ev step size */
	mt9m021_write(sd, MT9M021_AE_MIN_EV_STEP, ae_min_ev_step);
	mt9m021_write(sd, MT9M021_AE_MAX_EV_STEP, ae_max_ev_step);

	/* Analogue gain */
	mt9m021_write(sd, MT9M021_DIGITAL_TEST, dig_test_reg._register);

	mt9m021_write(sd, MT9M021_GLOBAL_GAIN, gain);

	/* AE ROI */
	mt9m021_write(&mt9m021->sd, MT9M021_AE_ROI_X_START, ae_roi_x_start);
	mt9m021_write(&mt9m021->sd, MT9M021_AE_ROI_Y_START, ae_roi_y_start);
	mt9m021_write(&mt9m021->sd, MT9M021_AE_ROI_X_SIZE, ae_roi_x_size);
	mt9m021_write(&mt9m021->sd, MT9M021_AE_ROI_Y_SIZE, ae_roi_y_size);

	return mt9m021_write(sd, MT9M021_AE_CTRL_REGISTER,
		                     ae_ctrl_reg._register);
}

static int mt9m021_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9m021               *mt9m021 = to_mt9m021(sd);
	struct mt9m021_platform_data *pdata = mt9m021->pdata;
	struct v4l2_fract            *fi  = &mt9m021->frame_interval;
	int                           ret = 0;
	u16                           frame_length;

	if (enable == 0) {
		/* Stream off */
		mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_STREAM_OFF);

		if (pdata->set_power)
			pdata->set_power(0);

		mt9m021->streaming = 0;
		return 0;
	}

	if (pdata->set_power) {
		ret = pdata->set_power(1);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	/*
	 * Resetting the sensor will put the all sensor registers back to their
	 * default values. In some cases (mainly the i2c slave address), it is
	 * better to avoid this behavior.
	 */
	if (!pdata->no_soft_reset)
		mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_RESET);

	/*
	 * The following functions have been taken directly from Aptina
	 * software, I have no idea what it does...
	 */
	mt9m021_sequencer_settings(sd);
	mt9m021_col_correction(sd);
	mt9m021_rev2_settings(sd);

	ret = mt9m021_pll_setup(sd);
	if (ret < 0)
		goto power_off;

	ret = mt9m021_set_size(sd);
	if (ret < 0)
		goto power_off;

	/* Flip */
	mt9m021_write(sd, MT9M021_READ_MODE, 0x0000);

	/* Blanking:
	 *   HBLANK is minimized
	 *   VBLANK set according to the FPS
	 */
	mt9m021_write(sd, MT9M021_LINE_LENGTH_PCK, MT9M021_MIN_LINE_LENGTH);

	frame_length = div_u64((u64) mt9m021->pll.pix_clock * fi->numerator,
			       MT9M021_MIN_LINE_LENGTH * fi->denominator);

	mt9m021_write(sd, MT9M021_FRAME_LENGTH_LINES, frame_length);

	/* Enable embedded data */
	mt9m021_write(sd, MT9M021_EMBEDDED_DATA_CTRL, 0x1982);

	/* Set exposure mode & integration time */
	mt9m021_apply_exposure(sd);

	/* Temperature sensor enable */
	mt9m021_write(sd, MT9M021_TEMPSENS_CTRL, MT9M021_TEMPSENS_EN);

	/* Stream on */
	if (mt9m021->trigger->val)
		mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_TRIGGER_MODE);
	else
		mt9m021_write(sd, MT9M021_RESET_REGISTER, MT9M021_STREAM_ON);

	mt9m021->streaming = 1;

	return 0;

power_off:
	if (pdata->set_power)
		pdata->set_power(0);

	return ret;
}

static int mt9m021_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct mt9m021 *mt9m021 = to_mt9m021(sd);

	memset(fi, 0, sizeof(*fi));
	fi->interval = mt9m021->frame_interval;

	return 0;
}

static int mt9m021_s_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct mt9m021 *mt9m021 = to_mt9m021(sd);

	mt9m021->frame_interval = fi->interval;

	return 0;
}

static int mt9m021_g_dv_timings(struct v4l2_subdev *sd,
				struct v4l2_dv_timings *timings)
{
	struct mt9m021            *mt9m021 = to_mt9m021(sd);
	struct v4l2_mbus_framefmt *fmt = &mt9m021->format;
	struct v4l2_bt_timings    *bt = &timings->bt;

	memset(timings, 0, sizeof(*timings));

	bt->width      = fmt->width;
	bt->height     = fmt->height;
	bt->pixelclock = mt9m021->pll.pix_clock;

	return 0;
}

static const struct v4l2_subdev_video_ops mt9m021_video_ops = {
	.s_stream         = mt9m021_s_stream,
	.g_frame_interval = mt9m021_g_frame_interval,
	.s_frame_interval = mt9m021_s_frame_interval,
	.g_dv_timings     = mt9m021_g_dv_timings,
};

static const struct v4l2_subdev_pad_ops mt9m021_pad_ops = {
	.get_fmt        = mt9m021_get_fmt,
	.set_fmt        = mt9m021_set_fmt,
	.enum_mbus_code = mt9m021_enum_mbus_code,
	.get_selection  = mt9m021_get_selection,
	.set_selection  = mt9m021_set_selection,
};

static const struct v4l2_subdev_ops mt9m021_ops = {
	.core  = &mt9m021_core_ops,
	.video = &mt9m021_video_ops,
	.pad   = &mt9m021_pad_ops,
};

#define V4L2_CID_MT9M021_TRIGGER            (V4L2_CID_CAMERA_CLASS_BASE + 0x100)
#define V4L2_CID_MT9M021_AUTO_ANAGAIN       (V4L2_CID_CAMERA_CLASS_BASE + 0x101)
#define V4L2_CID_MT9M021_AUTO_DIGGAIN       (V4L2_CID_CAMERA_CLASS_BASE + 0x102)
#define V4L2_CID_MT9M021_EXPOSURE_AUTO_MAX  (V4L2_CID_CAMERA_CLASS_BASE + 0x103)
#define V4L2_CID_MT9M021_EXPOSURE_AUTO_MIN  (V4L2_CID_CAMERA_CLASS_BASE + 0x104)
#define V4L2_CID_MT9M021_EXPOSURE_TGT_LUMA  (V4L2_CID_CAMERA_CLASS_BASE + 0x105)

/* Region of Interest for Auto-Exposure */
#define V4L2_CID_MT9M021_AE_ROI_X_START     (V4L2_CID_CAMERA_CLASS_BASE + 0x106)
#define V4L2_CID_MT9M021_AE_ROI_Y_START     (V4L2_CID_CAMERA_CLASS_BASE + 0x107)
#define V4L2_CID_MT9M021_AE_ROI_X_SIZE      (V4L2_CID_CAMERA_CLASS_BASE + 0x108)
#define V4L2_CID_MT9M021_AE_ROI_Y_SIZE      (V4L2_CID_CAMERA_CLASS_BASE + 0x109)

#define V4L2_CID_MT9M021_AE_AG_EXPOSURE_HI  (V4L2_CID_CAMERA_CLASS_BASE + 0x10a)
#define V4L2_CID_MT9M021_AE_AG_EXPOSURE_LO  (V4L2_CID_CAMERA_CLASS_BASE + 0x10b)

#define V4L2_CID_MT9M021_AE_MIN_EV_STEP     (V4L2_CID_CAMERA_CLASS_BASE + 0x10c)
#define V4L2_CID_MT9M021_AE_MAX_EV_STEP     (V4L2_CID_CAMERA_CLASS_BASE + 0x10d)

static int mt9m021_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9m021 *mt9m021 = ctrl_to_mt9m021(ctrl);

	/* If not streaming, just keep interval structures up-to-date */
	if (!mt9m021->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_MT9M021_AUTO_ANAGAIN:
	case V4L2_CID_MT9M021_AUTO_DIGGAIN:
	case V4L2_CID_EXPOSURE_AUTO:
	case V4L2_CID_EXPOSURE_ABSOLUTE:
	case V4L2_CID_MT9M021_EXPOSURE_AUTO_MAX:
	case V4L2_CID_MT9M021_EXPOSURE_AUTO_MIN:
	case V4L2_CID_MT9M021_AE_AG_EXPOSURE_HI:
	case V4L2_CID_MT9M021_AE_AG_EXPOSURE_LO:
	case V4L2_CID_MT9M021_EXPOSURE_TGT_LUMA:
	case V4L2_CID_GAIN:
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_MT9M021_AE_ROI_X_START:
	case V4L2_CID_MT9M021_AE_ROI_Y_START:
	case V4L2_CID_MT9M021_AE_ROI_X_SIZE:
	case V4L2_CID_MT9M021_AE_ROI_Y_SIZE:
	case V4L2_CID_MT9M021_AE_MIN_EV_STEP:
	case V4L2_CID_MT9M021_AE_MAX_EV_STEP:
		return mt9m021_apply_exposure(&mt9m021->sd);
	}

	return 0;
}

static const struct v4l2_ctrl_ops mt9m021_ctrl_ops = {
	.s_ctrl = mt9m021_s_ctrl,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_trigger = {
	.ops  = &mt9m021_ctrl_ops,
	.id   = V4L2_CID_MT9M021_TRIGGER,
	.name = "Trigger mode",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = false,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_aag = {
	.ops  = &mt9m021_ctrl_ops,
	.id   = V4L2_CID_MT9M021_AUTO_ANAGAIN,
	.name = "Auto analogue gain",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = true,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_adg = {
	.ops  = &mt9m021_ctrl_ops,
	.id   = V4L2_CID_MT9M021_AUTO_DIGGAIN,
	.name = "Auto digital gain",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = true,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_max = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_EXPOSURE_AUTO_MAX,
	.name = "Max allowed Auto Exposure (us)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 1000000,
	.step = 1,
	.def = 30000,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_min = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_EXPOSURE_AUTO_MIN,
	.name = "Min allowed Auto Exposure (us)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 1000000,
	.step = 1,
	.def = 1,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_tgt_luma = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_EXPOSURE_TGT_LUMA,
	.name = "Auto Exposure target luma",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 8192,
	.step = 1,
	.def = 0x500,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_ag_exposure_hi = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_AE_AG_EXPOSURE_HI,
	.name = "Upper exposure limit for increasing the Analog gain (us)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 1000000,
	.step = 1,
	.def = 20000,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_ag_exposure_lo = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_AE_AG_EXPOSURE_LO,
	.name = "Lower exposure limit for increasing the Analog gain (us)",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 1000000,
	.step = 1,
	.def = 10000,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_roi_x_start = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_AE_ROI_X_START,
	.name = "Auto Exposure ROI X start",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 1280 - 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_roi_y_start = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_AE_ROI_Y_START,
	.name = "Auto Exposure ROI Y start",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 960 - 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_roi_x_size = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_AE_ROI_X_SIZE,
	.name = "Auto Exposure ROI width",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 1280,
	.step = 1,
	.def = 1280,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_roi_y_size = {
	.ops = &mt9m021_ctrl_ops,
	.id = V4L2_CID_MT9M021_AE_ROI_Y_SIZE,
	.name = "Auto Exposure ROI height",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 1,
	.max = 960,
	.step = 1,
	.def = 960,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_min_ev_step = {
        .ops = &mt9m021_ctrl_ops,
        .id = V4L2_CID_MT9M021_AE_MIN_EV_STEP,
        .name = "AE Min EV step size",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 1,
        .max = 0xFF,
        .step = 1,
        .def = 0x0080,
};

static const struct v4l2_ctrl_config mt9m021_ctrl_ae_max_ev_step = {
        .ops = &mt9m021_ctrl_ops,
        .id = V4L2_CID_MT9M021_AE_MAX_EV_STEP,
        .name = "AE Max EV step size",
        .type = V4L2_CTRL_TYPE_INTEGER,
        .min = 1,
        .max = 0xFFFF,
        .step = 1,
        .def = 0x0008,
};


static int mt9m021_initialize_controls(struct v4l2_subdev *sd)
{
	struct mt9m021           *mt9m021 = to_mt9m021(sd);
	struct v4l2_ctrl_handler *hdl      = &mt9m021->ctrl_handler;
	int                       ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0) {
		v4l2_err(sd, "failed to init ctrl handler\n");
		goto einit;
	}

	mt9m021->trigger  = v4l2_ctrl_new_custom(hdl, &mt9m021_ctrl_trigger, NULL);
	mt9m021->auto_aag = v4l2_ctrl_new_custom(hdl, &mt9m021_ctrl_aag, NULL);
	mt9m021->auto_adg = v4l2_ctrl_new_custom(hdl, &mt9m021_ctrl_adg, NULL);
	mt9m021->exposure_mode = v4l2_ctrl_new_std_menu(hdl, &mt9m021_ctrl_ops,
	                                                V4L2_CID_EXPOSURE_AUTO,
							3, ~3, V4L2_EXPOSURE_AUTO);

	mt9m021->exposure_abs = v4l2_ctrl_new_std(hdl,
					          &mt9m021_ctrl_ops,
					          V4L2_CID_EXPOSURE_ABSOLUTE,
					          0, 1000000, 1, 20000);

	mt9m021->gain         = v4l2_ctrl_new_std(hdl,
					          &mt9m021_ctrl_ops,
					          V4L2_CID_GAIN,
					          0, 255, 1, 32);

	mt9m021->ana_gain    = v4l2_ctrl_new_std(hdl,
					          &mt9m021_ctrl_ops,
					          V4L2_CID_ANALOGUE_GAIN,
					          0, 3, 1, 0);

	mt9m021->exposure_ae_max = v4l2_ctrl_new_custom(hdl,
	                                                &mt9m021_ctrl_ae_max,
	                                                NULL);

	mt9m021->exposure_ae_min = v4l2_ctrl_new_custom(hdl,
	                                                &mt9m021_ctrl_ae_min,
	                                                NULL);

	mt9m021->ae_ag_exposure_hi = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_ag_exposure_hi,
						NULL);

	mt9m021->ae_ag_exposure_lo = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_ag_exposure_lo,
						NULL);

	mt9m021->ae_tgt_luma = v4l2_ctrl_new_custom(hdl,
	                                            &mt9m021_ctrl_ae_tgt_luma,
						    NULL);

	/* Auto-Exposure Region Of Interest */
	mt9m021->ae_roi_x_start = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_roi_x_start,
						NULL);

	mt9m021->ae_roi_y_start = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_roi_y_start,
						NULL);

	mt9m021->ae_roi_x_size = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_roi_x_size,
						NULL);

	mt9m021->ae_roi_y_size = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_roi_y_size,
						NULL);

	mt9m021->ae_min_ev_step = v4l2_ctrl_new_custom(hdl,
						&mt9m021_ctrl_ae_min_ev_step,
						NULL);

	mt9m021->ae_max_ev_step = v4l2_ctrl_new_custom(hdl,
                                                &mt9m021_ctrl_ae_max_ev_step,
                                                NULL);


	if (hdl->error) {
		v4l2_err(sd, "failed to add new controls\n");
		ret = hdl->error;
		goto ectrl;
	}

	sd->ctrl_handler = hdl;

	return 0;

ectrl:
	v4l2_ctrl_handler_free(hdl);
einit:
	return ret;
}

static int mt9m021_detect_chip(struct v4l2_subdev *sd)
{
	struct mt9m021               *mt9m021 = to_mt9m021(sd);
	struct mt9m021_platform_data *pdata = mt9m021->pdata;
	int                           ret = 0;
	u16                           id = 0;

	if (pdata->set_power) {
		ret = pdata->set_power(1);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	mt9m021_read(sd, MT9M021_CHIP_VERSION, &id);

	if (pdata->set_power)
		pdata->set_power(0);

	if (id != MT9M021_CHIP_VERSION_VALUE) {
		v4l2_err(sd, "Error Chip ID = 0x%04x instead of 0x%04x\n",
				id, MT9M021_CHIP_VERSION_VALUE);
		return -ENODEV;
	}

	v4l2_info(sd, "Found " DRIVER_NAME " chip\n");

	return 0;
}

static int mt9m021_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct mt9m021               *mt9m021;
	struct mt9m021_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev           *sd;
	int                           ret = 0;

	if (pdata == NULL) {
		dev_err(&client->dev, "platform data not specified\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c not available\n");
		return -ENODEV;
	}

	mt9m021 = kzalloc(sizeof(*mt9m021), GFP_KERNEL);
	if (!mt9m021) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	mt9m021->pdata = pdata;

	sd = &mt9m021->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9m021_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	mt9m021->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &mt9m021->pad, 0);

	if (ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto error_media_entity_init;
	}

	/* Set default configuration: Y10 960p30 at center, with 4+6 embedded data
	 * lines
	 */
	mt9m021->crop.width    = 1280;
	mt9m021->crop.height   =  960;
	mt9m021->crop.left     =    0;
	mt9m021->crop.top      =    0;
	mt9m021->format.width  = 1280;
	mt9m021->format.height =  970;
	mt9m021->format.code   = V4L2_MBUS_FMT_Y10_1X10;
	mt9m021->frame_interval.numerator   =  1;
	mt9m021->frame_interval.denominator = 30;

	/* Clocks */
	mt9m021->pll.ext_clock = pdata->ext_clock;
	mt9m021->pll.pix_clock = pdata->pix_clock;

	/* Check if the chip is present */
	ret = mt9m021_detect_chip(sd);
	if (ret < 0)
		goto error_detect;

	ret = mt9m021_initialize_controls(sd);
	if (ret < 0)
		goto error_ctrl;

	return 0;

error_ctrl:
error_detect:
error_media_entity_init:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(mt9m021);

	return ret;
}

static int mt9m021_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd        = i2c_get_clientdata(client);
	struct mt9m021     *mt9m021   = to_mt9m021(sd);
	struct v4l2_ctrl_handler *hdl = &mt9m021->ctrl_handler;

	v4l2_ctrl_handler_free(hdl);

	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(mt9m021);

	return 0;
}

static const struct i2c_device_id mt9m021_id[] = {
	{DRIVER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mt9m021_id);

static struct i2c_driver mt9m021_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe = mt9m021_probe,
	.remove = mt9m021_remove,
	.id_table = mt9m021_id,
};

module_i2c_driver(mt9m021_driver);
