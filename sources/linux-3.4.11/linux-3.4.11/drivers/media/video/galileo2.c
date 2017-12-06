/*
 * galileo2 - Nokia sensor
 *
 * It is a 41 MPix sensor present in the Nokia Lumia 808
 *
 * Author : Eng-Hong SRON <eng-hong.sron@parrot.com>
 *
 * Date : Wed Jul  2 09:16:13 CEST 2014
 *
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/delay.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#include <media/galileo2.h>

#include "galileo2_reg.h"

MODULE_AUTHOR("Eng-Hong SRON <eng-hong.sron@parrot.com>");
MODULE_DESCRIPTION("Nokia Galileo2 driver");
MODULE_LICENSE("GPL");

#define DRIVER_NAME "galileo2"

#define SENSOR_WIDTH   7728
#define SENSOR_HEIGHT  5368

#define GALIELO2_BINING_MIN_Y    38
#define GALIELO2_BINING_MIN_X   512

/* Clocks */
#define GALILEO2_MIN_REFCLK          6000000UL
#define GALILEO2_MAX_REFCLK         27000000UL

#define GALILEO2_MIN_PLL_IN_CLK      3000000UL
#define GALILEO2_MAX_PLL_IN_CLK     27000000UL

#define GALILEO2_TARGET_PLL_CLK   4100000000UL

/* AD5830 shutter driver */
#define GALILEO2_SHUTTER_DRIVER_I2C_ADDR 0xc

/* AF driver : MAX14638 */
#define GALILEO2_AF_MCID 0x64
#define GALILEO2_AF_VID  0x04

/* Temperature sensor output */
#define GALILEO2_TEMP_MIN           (-20)
#define GALILEO2_TEMP_MAX           (80)
#define GALILEO2_TEMP_LOW_CALIB     (25)
#define GALILEO2_TEMP_HIGH_CALIB    (60)
#define GALILEO2_TEMP_AVERAGE_CALIB (42)

struct galileo2 {
	struct v4l2_subdev             sd;
	struct media_pad               pad;
	struct galileo2_platform_data *pdata;

	struct v4l2_mbus_framefmt      format;
	struct v4l2_fract              frame_interval;

	/* Internal states */
	bool                           streaming;
	bool                           timings_uptodate;

	/* Dimensions */
	struct v4l2_rect               crop;
	struct v4l2_rect               video_timing;
	u32                            x_binning;
	u32                            y_binning;
	u32                            bits_per_pixel;

	/* PLLs */
	struct {
		u32 pre_pll_clk_div;
		u32 pll_multiplier;
		u32 vt_sys_clk_div;
		u32 vt_pix_clk_div;
		u32 op_sys_clk_div;
		u32 op_pix_clk_div;
	} pll1;

	struct {
		u32 pre_pll_clk_div;
		u32 pll_multiplier;
	} pll0;

	/* Non-Volatile Memory */
	u8                            *nvm;
	union nvm_memaddr              nvm_addr;
	s8                             nvm_temp_h;
	s8                             nvm_temp_l;


	/* Clocks */
	unsigned long                  vtclk;
	unsigned long                  mipiclk;
	unsigned long                  line_duration_ns;

	u16                            trdy_ctrl;

	/* i2c clients */
	struct i2c_client             *i2c_sensor;

	/* Controls */
	struct v4l2_ctrl_handler       ctrl_handler;
	struct v4l2_ctrl              *hflip;
	struct v4l2_ctrl              *vflip;
	struct v4l2_ctrl              *exposure;
	struct v4l2_ctrl              *focus;
	struct v4l2_ctrl              *gain;
	struct v4l2_ctrl              *nd;
	struct v4l2_ctrl              *ms;
	struct v4l2_ctrl              *gs;
	struct v4l2_ctrl              *strobe_width;
	struct v4l2_ctrl              *strobe_start;
	struct v4l2_ctrl              *strobe_enable;
	struct v4l2_ctrl              *temp_sensor_output;
};

enum mech_shutter_state {
	MS_STATE_SSTROBE,
	MS_STATE_OPEN,
	MS_STATE_CLOSE
};

static inline struct galileo2 *to_galileo2(struct v4l2_subdev *sd)
{
	return container_of(sd, struct galileo2, sd);
}

static inline struct galileo2 *ctrl_to_galileo2(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct galileo2, ctrl_handler);
}

static int galileo2_read8(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8  regbuf[2];

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = regbuf,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 1,
			.buf   = val,
		},
	};


	regbuf[0] = reg >> 8;
	regbuf[1] = reg;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		*val = 0;
		dev_err(&client->dev, "Failed reading register 0x%04x! [%d]\n",
			reg, ret);
		return ret;
	}

	return 0;
}

static int galileo2_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
	u8  regbuf[2];
	u8  valbuf[2];

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = 2,
			.buf   = regbuf,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = 2,
			.buf   = valbuf,
		},
	};

	regbuf[0] = reg >> 8;
	regbuf[1] = reg;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		*val = 0;
		dev_err(&client->dev, "Failed reading register 0x%04x! [%d]\n",
			reg, ret);
		return ret;
	}

	*val = valbuf[1];
	*val |= valbuf[0] << 8;

	return 0;
}

static int galileo2_write8(struct i2c_client *client, u16 reg, u8 val)
{
	int            ret;
	struct i2c_msg msg;
	struct {
		u16 reg;
		u8  val;
	} __packed buf;

	reg = swab16(reg);

	buf.reg = reg;
	buf.val = val;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 3;
	msg.buf   = (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int galileo2_write16(struct i2c_client *client, u16 reg, u16 val)
{
	int            ret;
	struct i2c_msg msg;
	u16            buf[2];

	buf[0] = swab16(reg);
	buf[1] = swab16(val);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 4;
	msg.buf   = (u8 *)&buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int galileo2_shutter_write8(struct i2c_client *client, u8 reg, u8 val)
{
	int            ret;
	struct i2c_msg msg;
	u8             buf[2];

	buf[0] = reg;
	buf[1] = val;

	msg.addr  = GALILEO2_SHUTTER_DRIVER_I2C_ADDR;
	msg.flags = 0;
	msg.len   = ARRAY_SIZE(buf);
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"Failed writing shutter register 0x%02x [%d]\n",
			reg, ret);
		return ret;
	}

	return 0;
}

static int galileo2_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	struct galileo2 *galileo2 = to_galileo2(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, 0);
		fmt->format = *mf;
		return 0;
	}

	fmt->format = galileo2->format;

	return 0;
}

static int galileo2_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct galileo2           *galileo2 = to_galileo2(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		fmt->format = *mf;
	} else {
		galileo2->format = fmt->format;
	}

	galileo2->timings_uptodate = 0;

	return 0;
}

static int galileo2_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	/* For now we support only one code */
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGBRG10_1X10;

	return 0;
}

static int galileo2_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_selection *sel)
{
	struct galileo2 *galileo2 = to_galileo2(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left   = 0;
		sel->r.top    = 0;
		sel->r.width  = SENSOR_WIDTH;
		sel->r.height = SENSOR_HEIGHT;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = galileo2->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* Compute VT timing and binning */
static int galileo2_calc_vt(struct v4l2_subdev *sd)
{
	struct galileo2           *galileo2 = to_galileo2(sd);
	struct v4l2_rect          *vt  = &galileo2->video_timing;
	struct v4l2_rect          *c   = &galileo2->crop;
	struct v4l2_mbus_framefmt *fmt = &galileo2->format;
	struct v4l2_fract         *fi  = &galileo2->frame_interval;
	u32                        min_vt_height;

	/* We bin as much as possible before scaling */
	galileo2->x_binning = c->width / fmt->width;
	galileo2->x_binning = min(galileo2->x_binning, 2U);

	galileo2->y_binning = c->height / fmt->height;
	galileo2->y_binning = min(galileo2->y_binning, 8U);

	if (galileo2->x_binning == 0)
		galileo2->x_binning = 1;

	if (galileo2->y_binning == 0)
		galileo2->y_binning = 1;

	/* Video Timing is working on binned pixels
	 *   min_vt_line_blanking_pck is GALIELO2_BINING_MIN_X
	 *   min_vt_frame_blanking_line is GALIELO2_BINING_MIN_Y
	 */
	vt->width  = (c->width  / galileo2->x_binning) + GALIELO2_BINING_MIN_X;
	vt->height = div_u64((u64) galileo2->vtclk * fi->numerator,
			     vt->width * fi->denominator);

	min_vt_height = c->height / galileo2->y_binning + GALIELO2_BINING_MIN_Y;

	if (vt->height < min_vt_height)
		vt->height = min_vt_height;

	/* Refresh FPS */
	fi->denominator = galileo2->vtclk;
	fi->numerator   = vt->width * vt->height;

	/* Refresh line_duration */
	galileo2->line_duration_ns = div_u64((u64) vt->width * 1000000000,
					     galileo2->vtclk);

	/* It seems there is a minimum VT width which differs from what the
	 * datasheet says (8240). It is an empiric value, I don't know if it is
	 * correct... */
	/*vt->width = max(vt->width, 1920);*/

	return 0;
}

static int galileo2_set_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_selection *sel)
{
	struct galileo2           *galileo2 = to_galileo2(sd);
	struct v4l2_rect          *c   = &galileo2->crop;
	struct v4l2_mbus_framefmt *fmt = &galileo2->format;
	struct i2c_client         *i2c = galileo2->i2c_sensor;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		galileo2->crop = sel->r;
		break;
	default:
		v4l2_err(sd, "selection target (%d) not supported yet\n",
			     sel->target);
		return -EINVAL;
	}

	galileo2->timings_uptodate = 0;

	if (!galileo2->streaming)
		return 0;

	/* We bin as much as possible before scaling */
	galileo2->x_binning = c->width / fmt->width;
	galileo2->x_binning = min(galileo2->x_binning, 2U);

	galileo2->y_binning = c->height / fmt->height;
	galileo2->y_binning = min(galileo2->y_binning, 8U);

	if (galileo2->x_binning == 0)
		galileo2->x_binning = 1;

	if (galileo2->y_binning == 0)
		galileo2->y_binning = 1;

	galileo2_write16(i2c, GROUPED_PARAMETER_HOLD, 0x1);

	galileo2_write16(i2c, X_ADDR_START, c->left);
	galileo2_write16(i2c, Y_ADDR_START, c->top);
	galileo2_write16(i2c, X_ADDR_END,   c->left + c->width  - 1);
	galileo2_write16(i2c, Y_ADDR_END,   c->top  + c->height - 1);

	galileo2_write16(i2c, DIGITAL_CROP_IMAGE_WIDTH,
			      c->width / galileo2->x_binning);
	galileo2_write16(i2c, DIGITAL_CROP_IMAGE_HEIGHT,
			      c->height / galileo2->y_binning);

	/* Binning */
	if (galileo2->x_binning == 1 && galileo2->y_binning == 1) {
		galileo2_write8(i2c, BINNING_MODE, 0x0);
		galileo2_write8(i2c, BINNING_TYPE, 0x0);
	} else {
		galileo2_write8(i2c, BINNING_MODE, 0x1);
		galileo2_write8(i2c, BINNING_TYPE,
				galileo2->x_binning << 4 |
				galileo2->y_binning);
	}

	galileo2_write16(i2c, GROUPED_PARAMETER_HOLD, 0x0);

	return 0;
}

static const struct v4l2_subdev_pad_ops galileo2_pad_ops = {
	.get_fmt        = galileo2_get_fmt,
	.set_fmt        = galileo2_set_fmt,
	.enum_mbus_code = galileo2_enum_mbus_code,
	.get_selection  = galileo2_get_selection,
	.set_selection  = galileo2_set_selection,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int galileo2_get_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct galileo2 *galileo2 = to_galileo2(sd);
	int              ret;
	u8               val;

	reg->size = 2;

	if (reg->reg & ~0xff)
		return -EINVAL;

	ret = galileo2_read8(galileo2->i2c_sensor, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = (__u64)val;

	return 0;
}

static int galileo2_set_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct galileo2 *galileo2 = to_galileo2(sd);

	if (reg->reg & ~0xff || reg->val & ~0xff)
		return -EINVAL;

	return galileo2_write8(galileo2->i2c_sensor, reg->reg, reg->val);
}
#endif

static const struct v4l2_subdev_core_ops galileo2_core_ops = {
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = galileo2_get_register,
	.s_register = galileo2_set_register,
#endif
};

/* Compute minimum clocks in order to reach the FPS */
static int galileo2_calc_clocks(struct v4l2_subdev *sd)
{
	struct galileo2               *galileo2 = to_galileo2(sd);
	struct v4l2_mbus_framefmt     *fmt = &galileo2->format;

	/* Finally, mipiclk will have to transfer all the scaled pixels, but the
	 * vertical scaling need some line buffers, introducing some
	 * 'burstiness'. We can considered the transfered frame as only scaled
	 * horizontally.
	 */
	switch (fmt->code) {
	case V4L2_MBUS_FMT_SBGGR10_1X10:
	case V4L2_MBUS_FMT_SGBRG10_1X10:
	case V4L2_MBUS_FMT_SGRBG10_1X10:
	case V4L2_MBUS_FMT_SRGGB10_1X10:
		galileo2->bits_per_pixel = 10;
		break;
	default:
		v4l2_err(sd, "code not supported yet\n");
		galileo2->vtclk = 0;
		galileo2->mipiclk = 0;
		return -EINVAL;
	}

	return 0;
}

#define IS_BETWEEN(_f, _min, _max) ((_f >= _min) && (_f <= _max))

/* Configure PLL to suport all formats needed
 * FPS will be adjust trough the vertical blanking
 */
static int galileo2_pll_config(struct v4l2_subdev *sd)
{
	struct galileo2               *galileo2 = to_galileo2(sd);
	struct galileo2_platform_data *pdata = galileo2->pdata;
	u32                            index;
	u64                            pll;

	/* Fixed pll divider to 2 */
	galileo2->pll1.pre_pll_clk_div = 2;

	/* PLL shall be ranged between 4-10,16 GHz
	 * -> we target 4,1 GHZ
	 * The multiplier is 8 multiplier
	 */
	for (index = 400; index < 1256; index += 8) {
		pll = div_u64(pdata->refclk, galileo2->pll1.pre_pll_clk_div);
		pll *= index;
		if (pll >= GALILEO2_TARGET_PLL_CLK)
			break;
	}

	galileo2->pll1.pll_multiplier = index;

	/*
	 * VT pixel clock is around 102 MHz (4.1 GHz / 4 / 10)
	 */
	galileo2->pll1.vt_sys_clk_div = 4;
	galileo2->pll1.vt_pix_clk_div = 10;

	/*
	 * OP clock is adapted according to number of used lanes (2 or 4)
	 */
	galileo2->pll1.op_sys_clk_div =  (pdata->lanes == 2) ? 2 : 4;
	galileo2->pll1.op_pix_clk_div = 6;

	/* Refresh clock frequencies */
	galileo2->vtclk =
		div_u64(((u64) pdata->refclk * galileo2->pll1.pll_multiplier),
			galileo2->pll1.pre_pll_clk_div
			* galileo2->pll1.vt_sys_clk_div
			* galileo2->pll1.vt_pix_clk_div);

	galileo2->mipiclk =
		div_u64(((u64) pdata->refclk * galileo2->pll1.pll_multiplier),
			    galileo2->pll1.pre_pll_clk_div
			    * galileo2->pll1.op_sys_clk_div);

	return 0;
}

static int galileo2_calc_plls(struct v4l2_subdev *sd)
{
	struct galileo2               *galileo2 = to_galileo2(sd);
	struct galileo2_platform_data *pdata = galileo2->pdata;

	/* PLL0 parameters */
	const u16 pre_pll_div[] = {1, 2, 4};
	long      best_error = -1;
	u32       p, best_p = 0;
	u32       m, best_m = 0;

	if (!IS_BETWEEN(pdata->refclk,
			GALILEO2_MIN_REFCLK,
			GALILEO2_MAX_REFCLK)) {
		v4l2_err(sd, "refclk (%lu) is out of range [%lu - %lu]\n",
			 galileo2->mipiclk,
			 GALILEO2_MIN_REFCLK,
			 GALILEO2_MAX_REFCLK);
		return -EINVAL;
	}

	/* Try to reach the PLL frequencies while preserving the FPS, but in
	 * case it is not possible, we have to derate it.
	 */
	if (galileo2_pll_config(sd) < 0) {
		v4l2_err(sd, "Unable to find PLL config for:\n");
		v4l2_err(sd, "  vtclk    %lu", galileo2->vtclk);
		v4l2_err(sd, "  mipiclk  %lu", galileo2->mipiclk);
		return -EINVAL;
	}

	/* TOSHIBA register setting
	 * I don't know what frequency is needed for the following BoostCK,
	 * ADC Clock, ck_st and hreg_clk...
	 * So I follow the given spreadsheet...
	 * I also assume the PLL0 constraints are the same as the PLL1.
	 */
	for (p = 0 ; p < ARRAY_SIZE(pre_pll_div) ; p++) {
		unsigned long pll_in_clk = pdata->refclk / pre_pll_div[p];

		if (!IS_BETWEEN(pll_in_clk,
				GALILEO2_MIN_PLL_IN_CLK,
				GALILEO2_MAX_PLL_IN_CLK))
			continue;
		for (m = 40 ; m <= 1256 ; m += 8) {
			unsigned long pll_op_clk = pll_in_clk * m;

			/* Trying to reach 1GHz, again, it seems to work that
			 * way, but I don't know why...
			 */
			long error = 1000000000UL - pll_op_clk;

			if (error < 0)
				error = -error;

			if (error < best_error || best_error < 0) {
				best_error = error;
				best_p = pre_pll_div[p];
				best_m = m;
			}
		}
	}

	galileo2->pll0.pre_pll_clk_div = best_p;
	galileo2->pll0.pll_multiplier  = best_m;

	return 0;
}

#undef  IS_BETWEEN

static int galileo2_update_timings(struct v4l2_subdev *sd)
{
	struct galileo2  *galileo2 = to_galileo2(sd);
	int               ret;

	/* Update clocks */
	ret = galileo2_calc_plls(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate plls\n");
		return ret;
	}

	/* From the crop and the output size, calculate the binning and the
	 * Video Timing.
	 */
	ret = galileo2_calc_vt(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate Video Timing\n");
		return ret;
	}

	/* Calculate the the minimum theorical clock frequency in order to
	 * achieve the frame interval.
	 */
	ret = galileo2_calc_clocks(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate Clocks\n");
		return ret;
	}

	galileo2->timings_uptodate = 1;

	return 0;
}

static int galileo2_apply_plls(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c_sensor = galileo2->i2c_sensor;

	galileo2_write16(i2c_sensor, PRE_PLL_CLK_DIV,
			 galileo2->pll1.pre_pll_clk_div);
	galileo2_write16(i2c_sensor, PLL_MULTIPLIER,
			 galileo2->pll1.pll_multiplier);
	galileo2_write16(i2c_sensor, VT_SYS_CLK_DIV,
			 galileo2->pll1.vt_sys_clk_div);
	galileo2_write16(i2c_sensor, VT_PIX_CLK_DIV,
			 galileo2->pll1.vt_pix_clk_div);
	galileo2_write16(i2c_sensor, OP_SYS_CLK_DIV,
			 galileo2->pll1.op_sys_clk_div);
	galileo2_write16(i2c_sensor, OP_PIX_CLK_DIV,
			 galileo2->pll1.op_pix_clk_div);

#if 0
	galileo2_write8(i2c_sensor, PRE_PLL_CNTL_ST,
					      galileo2->pll0.pre_pll_clk_div);
	galileo2_write16(i2c_sensor, PLL_MULTI_ST,
					       galileo2->pll0.pll_multiplier);

	galileo2_write8(i2c_sensor, AD_CNTL,            0x02);
	galileo2_write8(i2c_sensor, ST_CNTL,            0x07);
	galileo2_write8(i2c_sensor, HREG_CNTL,          0x05);
	galileo2_write8(i2c_sensor, PLL_HRG_CNTL,       0x01);
	galileo2_write8(i2c_sensor, HREG_PLLSEL_SINGLE, 0x10);
	galileo2_write8(i2c_sensor, OPCK_PLLSEL,        0x00);
#endif

	return 0;
}

/*
 * MODEPowerup_and_Initialize
 * Following values are taken directly from Nokia INIT.txt file.
 * I have no idea what it does...
 */
static int galileo2_init(struct v4l2_subdev *sd)
{
	struct galileo2                 *galileo2 = to_galileo2(sd);
	struct galileo2_platform_data   *pdata = galileo2->pdata;
	struct i2c_client               *i2c_sensor = galileo2->i2c_sensor;
	u16                              whole, fract;
	union global_reset_mode_config1  glbrst_cfg1 = {{
		.vf_to_glbrst                = 0, /* complete frame */
		.glbrst_to_vf                = 0,
		.readout_start               = 0, /* Readout start by tRDOUT */
		.long_exposure_mode          = 0,
		.continous_global_reset_mode = 0,
		.flash_strobe                = 0,
		.sstrobe_muxing              = 1,
		.sastrobe_muxing             = 0,
	} };

	/* Sensor MSRs */
#if 0
	galileo2_write8(i2c_sensor, POSBSTSEL,         0x1C);
	galileo2_write8(i2c_sensor, READVDSEL,         0x06);
	galileo2_write8(i2c_sensor, RSTVDSEL,          0x08);
	galileo2_write8(i2c_sensor, BSVBPSEL,          0x18);
	galileo2_write8(i2c_sensor, HREG_TEST,         0x04);
	galileo2_write8(i2c_sensor, DRESET,            0xC8);
	galileo2_write16(i2c_sensor, FRACEXP_TIME1,  0x08FF);
	galileo2_write16(i2c_sensor, PORTGRESET_U,   0x026C);
	galileo2_write8(i2c_sensor, PORTGRESET_W,      0x30);
	galileo2_write8(i2c_sensor, ROREAD,            0xCE);
	galileo2_write8(i2c_sensor, DRCUT,             0x01);
	galileo2_write8(i2c_sensor, GDMOSCNT,          0x2F);
	galileo2_write8(i2c_sensor, CDS_STOPBST,       0x01);
	galileo2_write8(i2c_sensor, BSTCKLFIX_ADC,     0x89);
	galileo2_write8(i2c_sensor, BSC_AGRNG2,        0x30);
	galileo2_write8(i2c_sensor, BSC_AGRNG1,        0x18);
	galileo2_write8(i2c_sensor, BSC_AGRNG0,        0x10);
	galileo2_write8(i2c_sensor, KBIASCNT_RNG32,    0x98);
	galileo2_write8(i2c_sensor, KBIASCNT_RNG10,    0x76);
	galileo2_write8(i2c_sensor, GDMOSHEN,          0x01);
	galileo2_write8(i2c_sensor, BSDIGITAL_MODE,    0x08);
	galileo2_write8(i2c_sensor, PS_VZS_NML_COEF,   0xC7);
	galileo2_write8(i2c_sensor, PS_VZS_NML_INTC,   0x7E);
	galileo2_write8(i2c_sensor, ZSV_IN_LINES,      0x43);
	galileo2_write8(i2c_sensor, FBC_IN_RANGE,      0x10);
	galileo2_write8(i2c_sensor, OB_CLPTHRSH_NEAR,  0x28);
	galileo2_write8(i2c_sensor, OB_CLPTHRSH_FAR,   0x28);
	galileo2_write8(i2c_sensor, WKUP_WAIT_ON,      0xE9);
	galileo2_write8(i2c_sensor, HALF_VTAP_MODE,    0x12);
	galileo2_write8(i2c_sensor, CCP2BLKD,          0xB0);
#endif

	/* Sensor static register */
	whole = pdata->refclk / 1000000;
	fract = ((pdata->refclk - (whole * 1000000)) * 0x100) / 1000000;

	galileo2_write8(i2c_sensor, EXTCLK_FRQ_MHZ,     whole);
	galileo2_write8(i2c_sensor, EXTCLK_FRQ_MHZ + 1, fract);

	galileo2_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
			glbrst_cfg1._register);
	galileo2_write8(i2c_sensor, DPHY_CTRL, 0x01);

	/* Link MBPS seems to influence the bridge, I don't know why, so I let
	 * this value to zero.
	 */
	galileo2_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_31_24, 0x0f);
	galileo2_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_23_16, 0x99);
	galileo2_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_15_8,  0x0);
	galileo2_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_7_0,   0x0);

	return 0;
}

static int galileo2_apply_hflip(struct v4l2_subdev *sd)
{
	struct galileo2         *galileo2 = to_galileo2(sd);
	struct i2c_client       *i2c = galileo2->i2c_sensor;
	union image_orientation reg;

	galileo2_read8(i2c, IMAGE_ORIENTATION, &reg._register);
	reg.h_mirror = galileo2->hflip->val;
	galileo2_write8(i2c, IMAGE_ORIENTATION, reg._register);

	return 0;
}

static int galileo2_apply_vflip(struct v4l2_subdev *sd)
{
	struct galileo2         *galileo2 = to_galileo2(sd);
	struct i2c_client       *i2c = galileo2->i2c_sensor;
	union image_orientation reg;

	galileo2_read8(i2c, IMAGE_ORIENTATION, &reg._register);
	reg.v_mirror = galileo2->vflip->val;
	galileo2_write8(i2c, IMAGE_ORIENTATION, reg._register);

	return 0;
}

static int galileo2_apply_nd(struct v4l2_subdev *sd)
{
	return 0;
}

static int galileo2_drive_shutter(struct v4l2_subdev *sd, int open)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;

	/* Set current 150mA, single shot mode, I2C control */
	galileo2_shutter_write8(i2c, 0x02, 0x15);
	/* Set current 200mA, single shot mode, I2C control */
	/*galileo2_shutter_write8(i2c, 0x02, 0x16);*/

	if (open)
		galileo2_shutter_write8(i2c, 0x06, 0xb4);
	else
		galileo2_shutter_write8(i2c, 0x06, 0xb1);

	return 0;
}

static int galileo2_set_shutter(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;

	switch (galileo2->ms->val) {
	case MS_STATE_SSTROBE:
		/* Set current 200mA, auto-reverse, strobe control */
		galileo2_shutter_write8(i2c, 0x02, 0x0a);
		galileo2_shutter_write8(i2c, 0x06, 0xb4);
		break;
		/* See register 0x0C02 for strobe config */
	case MS_STATE_OPEN:
		return galileo2_drive_shutter(sd, 1);
	case MS_STATE_CLOSE:
		return galileo2_drive_shutter(sd, 0);
	}

	return 0;
}

static int galileo2_apply_ms(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2    = to_galileo2(sd);
	struct i2c_client *i2c_sensor  = galileo2->i2c_sensor;
	u32               *exposure_us = &galileo2->exposure->val;
	u32                ms_state    = galileo2->ms->val;
	struct v4l2_rect  *c           = &galileo2->crop;
	struct v4l2_rect  *vt          = &galileo2->video_timing;
	u8                *nvm         = galileo2->nvm;
	union nvm_memaddr *nvm_addr    = &galileo2->nvm_addr;
	u16                sdelay, sdelay_ctrl;
	u16                trdout_ctrl;
	u16                str_delay_ctrl;
	u16                tgrst_interval_ctrl;
	u32                half_line_duration;

	union global_reset_mode_config1 glbrst_cfg1 = {
		.vf_to_glbrst                = 0,
		.glbrst_to_vf                = 0,
		.readout_start               = 0,
		.long_exposure_mode          = 0,
		.continous_global_reset_mode = 1,
		.flash_strobe                = 0,
		.sstrobe_muxing              = 1,
		.sastrobe_muxing             = 1,
	};

	if (ms_state != MS_STATE_SSTROBE)
		glbrst_cfg1.sastrobe_muxing = 0;

	/* Deactivate GS mode if it was previously enabled */
	if (!galileo2->gs->val) {
		glbrst_cfg1.sastrobe_muxing = 0;
		glbrst_cfg1.continous_global_reset_mode = 0;

		galileo2_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
				    glbrst_cfg1._register);

		galileo2_write8(galileo2->i2c_sensor,
				GLOBAL_RESET_CTRL1, 0x0);

		if (ms_state != MS_STATE_CLOSE)
			return galileo2_set_shutter(sd);

		return 0;
	}

	galileo2->trdy_ctrl = 0x0034;

	/*
	 *  This is used to round further timing computations
	 *   instead of flooring
	 */
	half_line_duration = galileo2->line_duration_ns / 2;

	/* Shutter should close after exposure time, but we need to take into
	 * account the shutter speed stored in the NVM */
	sdelay = swab16(*((u16 *)(nvm + nvm_addr->ms)));
	sdelay_ctrl = ((u32)sdelay * 1000 + half_line_duration)
		/ galileo2->line_duration_ns;

	/* Don't begin reading the pixels until we've waited
	 * for the exposure
	 * time
	 */
	trdout_ctrl = ((u32)(*exposure_us) * 1000 + half_line_duration)
		/ galileo2->line_duration_ns;

	if (sdelay_ctrl > galileo2->trdy_ctrl + trdout_ctrl)
		galileo2->trdy_ctrl = sdelay_ctrl - trdout_ctrl;

	/* Leave the shutter open for some more time so that it closes when we
	 * start reading the pixels */
	str_delay_ctrl = galileo2->trdy_ctrl + trdout_ctrl - sdelay_ctrl;

	/* Configure timer */
	/* Set Global reset ready to its minimum */
	galileo2_write16(i2c_sensor, TRDY_CTRL, galileo2->trdy_ctrl);

	galileo2_write16(i2c_sensor, TSHUTTER_STROBE_DELAY_CTRL,
				     str_delay_ctrl);

	/* Start readout as soon as possible */
	galileo2_write16(i2c_sensor, TRDOUT_CTRL, trdout_ctrl);

	/* Close the shutter during the readout, thus it should last at least
	 * the number of active line.
	 */
	galileo2_write16(i2c_sensor, TSHUTTER_STROBE_WIDTH_CTRL,
				     c->height / galileo2->y_binning
				     + sdelay_ctrl);

	tgrst_interval_ctrl = vt->height + trdout_ctrl + galileo2->trdy_ctrl
		+ sdelay_ctrl + GALIELO2_BINING_MIN_X;
	galileo2_write16(i2c_sensor, TGRST_INTERVAL_CTRL, tgrst_interval_ctrl);

	galileo2_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
			glbrst_cfg1._register);

	/* Mechanical shutter control */
	galileo2_write8(i2c_sensor, GLOBAL_RESET_CTRL1, 0x1);
	/* galileo2_write8(i2c_pmic, MECH_SHUTTER_CONTROL, 0x1); */

	return 0;
}

static int galileo2_apply_exposure(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct v4l2_fract *fi = &galileo2->frame_interval;
	u32               *exposure_us = &galileo2->exposure->val;
	u16                coarse;

	/* Exposure is expressed in us */
	u32 exposure_max_us = div_u64((u64) fi->numerator * 1000000,
				      fi->denominator);

	if (*exposure_us > exposure_max_us) {
		v4l2_warn(sd, "requested exposure (%d) is higher than exposure max (%d)\n",
			      *exposure_us, exposure_max_us);

		*exposure_us = exposure_max_us;
	}

	coarse = (*exposure_us * 1000) / galileo2->line_duration_ns;

	galileo2_write16(galileo2->i2c_sensor, COARSE_INTEGRATION_TIME, coarse);

	return 0;
}

static int galileo2_apply_gain(struct v4l2_subdev *sd)
{
	struct galileo2 *galileo2 = to_galileo2(sd);
	u32              gain     = galileo2->gain->val;

	galileo2_write16(galileo2->i2c_sensor, ANALOG_GAIN_CODE_GLOBAL, gain);

	return 0;
}

static int galileo2_apply_flash_strobe(struct v4l2_subdev *sd)
{
	struct galileo2                 *galileo2 = to_galileo2(sd);
	union global_reset_mode_config1  glbrst_cfg1;
	struct i2c_client               *i2c = galileo2->i2c_sensor;
	int strobe_en;

	strobe_en = galileo2->strobe_enable->val;

	galileo2_read8(i2c, GLOBAL_RESET_MODE_CONFIG1, &glbrst_cfg1._register);

	if (!strobe_en) {
		glbrst_cfg1.flash_strobe = 0;
		galileo2_write8(i2c,
				GLOBAL_RESET_MODE_CONFIG1,
				glbrst_cfg1._register);
		galileo2_write8(i2c, FLASH_TRIGGER_RS, 0x0);
		return 0;
	}

	/* Set the width to 100us, it is an arbitrary value, but the signal
	 * seems to take at least ~30us to go from 0 to 1
	 */
	if (galileo2->gs->val) {
		/* "Global" shutter mode (photo) */
		glbrst_cfg1.flash_strobe = 1;

		galileo2_write8(i2c,
				GLOBAL_RESET_MODE_CONFIG1,
				glbrst_cfg1._register);

		galileo2_write16(i2c,
				 TFLASH_STROBE_WIDTH_HIGH_CTRL,
				 (galileo2->strobe_width->val * 1000) / 100);

	} else {
		/* Rolling shutter mode (video) */
		galileo2_write16(i2c,
				 TFLASH_STROBE_WIDTH_HIGH_RS_CTRL,
				 (galileo2->strobe_width->val * 1000) / 100);
		galileo2_write8(i2c, FLASH_MODE_RS, 0x1);
		galileo2_write8(i2c, FLASH_TRIGGER_RS, 0x1);
	}


	return 0;
}

static int galileo2_get_lens_position(struct v4l2_subdev *sd, u16 *pos)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;
	u8                 LSB_value = 0;
	u8                 MSB_value = 0;
	u8                 value_register_8 = 0x01;

	/* Waiting bit0 = 0 : operation completed (p248) */
	while ((value_register_8 & 0x01) != 0x00)
		galileo2_read8(i2c, (FOCUS_CHANGE_CONTROL+1), &value_register_8);

	/* Write I2C slave address for reading : 0x1D */
	galileo2_write8(i2c, HLAFD_SLV_ADR, 0x1D);
	/* Set read mode */
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x10);

	/* Set the sub_address of ISET_H register */
	galileo2_write8(i2c, HLAFD_SUB_ADR, 0x03);
	/* Start data transfer */
	galileo2_write8(i2c, BYTE_ACCESS_ENABLE, 0x01);
	/* Read transferred data comes from MAX14638 */
	galileo2_read8(galileo2->i2c_sensor, HLAFD_RW_DATA, &MSB_value);

	/* Set the sub_address of ISET_L register */
	galileo2_write8(i2c, HLAFD_SUB_ADR, 0x04);
	/* Start data transfer */
	galileo2_write8(i2c, BYTE_ACCESS_ENABLE, 0x01);
	/* Read transferred data comes from MAX14638 */
	galileo2_read8(galileo2->i2c_sensor, HLAFD_RW_DATA, &LSB_value);

	*pos = ((MSB_value & 0x03) << 8) + LSB_value;

	/* Stop read mode */
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x04);

	return 0;
}

static int galileo2_get_temp_sensor_output(struct v4l2_subdev *sd, s8 *output)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;

	/*
	 * In NVM there is two values for temp calibration : 25 and 60 C
	 * Look for the shift to apply for getting the real value
	 */
	s8 shift_temp_h = galileo2->nvm_temp_h - GALILEO2_TEMP_HIGH_CALIB;
	s8 shift_temp_l = galileo2->nvm_temp_l - GALILEO2_TEMP_LOW_CALIB;

	galileo2_read8(i2c, TEMP_SENSOR_OUTPUT, output);
	/*
	 * Look for the best shift according average temperature (42) :
	 * Take the best shift close the calibrated value
	 */

	if ((*output - shift_temp_h) > GALILEO2_TEMP_AVERAGE_CALIB)
		*output -= shift_temp_h;
	else
		*output -= shift_temp_l;

	if (*output < GALILEO2_TEMP_MIN)
		*output = GALILEO2_TEMP_MIN;

	if (*output > GALILEO2_TEMP_MAX)
		*output = GALILEO2_TEMP_MAX;

	return 0;
}

static int galileo2_apply_focus(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;
	u8                 data2send = 0;
	u32                focus = galileo2->focus->val;

	/* Write 10bits-value in FOCUS_CHANGE 16-bits register */
	data2send = ((focus >> 8) & 0x03);
	galileo2_write8(i2c, FOCUS_CHANGE, data2send);

	data2send = (focus & 0xFF);
	galileo2_write8(i2c, (FOCUS_CHANGE+1), data2send);

	/* Set infinity-to-macro focus */
	data2send = 0x02;
	galileo2_write8(i2c, FOCUS_CHANGE_CONTROL, data2send);

	data2send = 0x01;
	galileo2_write8(i2c, (FOCUS_CHANGE_CONTROL+1), data2send);

	return 0;
}

/* Manually synchronize control values, I'm not sure if it is the right way to
 * do it...
 */
static inline void galileo2_synchronize_ctrl(struct v4l2_ctrl *ctrl)
{
	v4l2_ctrl_lock(ctrl);
	ctrl->cur.val = ctrl->val;
	v4l2_ctrl_unlock(ctrl);
}

static int galileo2_configure(struct v4l2_subdev *sd)
{
	struct galileo2               *galileo2 = to_galileo2(sd);
	struct galileo2_platform_data *pdata = galileo2->pdata;
	struct i2c_client             *i2c = galileo2->i2c_sensor;
	struct v4l2_rect              *vt  = &galileo2->video_timing;
	struct v4l2_rect              *c   = &galileo2->crop;
	struct v4l2_mbus_framefmt     *fmt = &galileo2->format;
	int                            ret;
	u8                             mcid, vid;

	ret = galileo2_init(sd);
	if (ret < 0) {
		v4l2_err(sd, "init failed\n");
		return ret;
	}

	/* CSI2 mode */
	galileo2_write8(i2c, CSI_SIGNALING_MODE, 0x2);

	/* Pixel format */
	galileo2_write8(i2c,
			CSI_DATA_FORMAT_SOURCE,
			galileo2->bits_per_pixel);
	galileo2_write8(i2c,
			CSI_DATA_FORMAT_DESTINATION,
			galileo2->bits_per_pixel);
	galileo2_write8(i2c, CSI_LANE_MODE, pdata->lanes - 1);

	/* Image Size */
	galileo2_write16(i2c, X_OUTPUT_SIZE, fmt->width);
	galileo2_write16(i2c, Y_OUTPUT_SIZE, fmt->height);

	/* Image Scaling */
	/* Horizontal scaling, Bayer sampling */
	galileo2_write16(i2c, SCALING_MODE,     0x0001);
	galileo2_write16(i2c, SPATIAL_SAMPLING, 0x0000);

	/* Scaler */
	galileo2_write16(i2c, OUTPUT_IMAGE_WIDTH,  fmt->width);
#if 0
	galileo2_write16(i2c, OUTPUT_IMAGE_HEIGHT, fmt->height);
	galileo2_write16(i2c, SCALER_BLANKING_PCK, 0x26EC);
#endif

	/* Frame Timing */
	galileo2_write16(i2c, VT_LINE_LENGTH_PCK, vt->width);
	galileo2_write16(i2c, VT_FRAME_LENGTH_LINES, vt->height);

	/* Image area */
	galileo2_write16(i2c, X_ADDR_START, c->left);
	galileo2_write16(i2c, Y_ADDR_START, c->top);
	galileo2_write16(i2c, X_ADDR_END,   c->left + c->width  - 1);
	galileo2_write16(i2c, Y_ADDR_END,   c->top  + c->height - 1);

	/* Digital Crop: We do not crop before the scaler */
	galileo2_write16(i2c, DIGITAL_CROP_X_OFFSET, 0x0000);
	galileo2_write16(i2c, DIGITAL_CROP_Y_OFFSET, 0x0000);
	galileo2_write16(i2c, DIGITAL_CROP_IMAGE_WIDTH,
			      c->width / galileo2->x_binning);
	galileo2_write16(i2c, DIGITAL_CROP_IMAGE_HEIGHT,
			      c->height / galileo2->y_binning);

	/* Binning */
	if (galileo2->x_binning == 1 && galileo2->y_binning == 1) {
		galileo2_write8(i2c, BINNING_MODE, 0x0);
		galileo2_write8(i2c, BINNING_TYPE, 0x0);
	} else {
		galileo2_write8(i2c, BINNING_MODE, 0x1);
		galileo2_write8(i2c, BINNING_TYPE,
				galileo2->x_binning << 4 |
				galileo2->y_binning);
	}

	/* AF Driver */
	/* Write I2C slave address for reading : 0x1D */
	galileo2_write8(i2c, HLAFD_SLV_ADR, 0x1D);
	/* Set read mode */
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x10);
	/* Set the sub_address of MC_ID register (MAX14638 address) */
	galileo2_write8(i2c, HLAFD_SUB_ADR, 0x00);
	/* Start data transfer */
	galileo2_write8(i2c, BYTE_ACCESS_ENABLE, 0x01);
	/* Read transferred data comes from MAX14638 register */
	galileo2_read8(galileo2->i2c_sensor, HLAFD_RW_DATA, &mcid);

	/* Set the sub_address of V_ID register (MAX14638 address) */
	galileo2_write8(i2c, HLAFD_SUB_ADR, 0x01);
	/* Start data transfer */
	galileo2_write8(i2c, BYTE_ACCESS_ENABLE, 0x01);
	/* Read transferred data comes from MAX14638 register */
	galileo2_read8(galileo2->i2c_sensor, HLAFD_RW_DATA, &vid);

	/* Stop reading operation */
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x00);

	/* Check MAX14638 Id */
	if (mcid != GALILEO2_AF_MCID || vid != GALILEO2_AF_VID) {
		v4l2_err(sd, "Failed to detect MAX14638\n");
		return -EIO;
	}

	/* Start MAX14538 Initialization with write operation */
	/* Write I2C slave address for writing : 0x1C */
	galileo2_write8(i2c, HLAFD_SLV_ADR, 0x1C);
	/* Set write mode */
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x34);

	/* bit1 = 1 for ringing compensation enabled */
	galileo2_write8(i2c, SYSCTRL, 0x02);
	/*
	 * ISET[15:0] = 0x3FF (maximum value 100mA) linearly scaled current
	 *  through the VCM
	 */
	galileo2_write8(i2c, ISET_H, 0x01);
	galileo2_write8(i2c, ISET_L, 0xff);
	/* set bit_5 if you want check OPEN, SHORT, TSD bits */
	galileo2_write8(i2c, STAT, 0x00);
	galileo2_write8(i2c, RING_CFG, 0x06);
	galileo2_write8(i2c, VCMTM_H, 0x49);
	galileo2_write8(i2c, VCMTM_L, 0xb4);
	/* DC-DC always on PWM mode, Vout = 1.04V */
	galileo2_write8(i2c, DIGCFG1, 0x00);
	/* F_PWM = 10MHz (default) */
	galileo2_write8(i2c, DIGCFG2, 0x0c);
	/* Nb period acquired = 1 */
	galileo2_write8(i2c, VTCFG, 0x00);
	galileo2_write8(i2c, VTCUR, 0x00);
	/* Start data transfer : 0x34 + Bit0 */
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x35);
	galileo2_write8(i2c, HLAF_WINDOWS_CONTROL, 0x04);

	/* TOSHIBA register setting
	 * Scaler */
#if 0
	galileo2_write8(i2c, GREEN_AVERAGED_BAYER,       0x00);
	galileo2_write8(i2c, HORIZONTAL_DIGITAL_BINNING, 0x00);
	galileo2_write8(i2c, VERTICAL_DIGITAL_BINNING,   0x00);

	/* Row Noise improve setting */
	galileo2_write8(i2c, BLC_SEL, 0x01);
	galileo2_write16(i2c, CSI2_DELAY, 0x0000);
#endif

	/* DPC */
	galileo2_write8(i2c, SINGLE_DEFECT_CORRECT_ENABLE, 0x00);
	galileo2_write8(i2c, COMBINED_COUPLET_SINGLE_DEFECT_CORRECT_ENABLE,
			     0x01);

	/* Controls */
	galileo2_apply_exposure(sd);
	galileo2_apply_gain(sd);
	galileo2_apply_focus(sd);

	/* Synchronize control values */
	galileo2_synchronize_ctrl(galileo2->exposure);
	galileo2_synchronize_ctrl(galileo2->focus);

	return 0;
}

static int galileo2_cam_config(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;
	int ret = 0;

	/* Set VANA to 2.8V */
	ret |= galileo2_write8(i2c, VANA_VOLTAGE, 0x02);
	ret |= galileo2_write8(i2c, VANA_VOLTAGE+1, 0xCD);

	/* Set VDIG to 1.2V */
	ret |= galileo2_write8(i2c, VDIG_VOLTAGE, 0x01);
	ret |= galileo2_write8(i2c, VDIG_VOLTAGE+1, 0x33);

	/* Set VIO to 3.3V */
	ret |= galileo2_write8(i2c, VIO_VOLTAGE, 0x03);
	ret |= galileo2_write8(i2c, VIO_VOLTAGE+1, 0x54);

	/* Sensor (Pixel array) peripheral circuits setup, bias voltage */
	ret |= galileo2_write8(i2c, 0x3000, 0x54);
	ret |= galileo2_write8(i2c, 0x3003, 0x55);
	ret |= galileo2_write8(i2c, 0x3004, 0x54);

	/* ADC amp configuration */
	ret |= galileo2_write8(i2c, 0x3011, 0x04);

	/* For Black dot sun improvement */
	ret |= galileo2_write8(i2c, 0x303a, 0x1B);
	ret |= galileo2_write8(i2c, 0x303b, 0x17);
	ret |= galileo2_write8(i2c, 0x303c, 0x14);
	ret |= galileo2_write8(i2c, 0x303d, 0x11);

	/* ADC optimization for better SNR and pedestral shading */
	ret |= galileo2_write8(i2c, 0x30b5, 0x90);
	ret |= galileo2_write8(i2c, 0x30fe, 0x00);
	ret |= galileo2_write8(i2c, 0x3105, 0x30);
	ret |= galileo2_write8(i2c, 0x3121, 0x20);
	ret |= galileo2_write8(i2c, 0x312B, 0x80);

	/* For Black dot sun improvement */
	ret |= galileo2_write8(i2c, 0x312f, 0x30);

	/* Global black level adjustment */
	ret |= galileo2_write8(i2c, 0x3137, 0x11);
	ret |= galileo2_write8(i2c, 0x313c, 0x10);
	ret |= galileo2_write8(i2c, 0x313d, 0x02);
	ret |= galileo2_write8(i2c, 0x3154, 0x01);
	ret |= galileo2_write8(i2c, 0x3155, 0x07);
	ret |= galileo2_write8(i2c, 0x3156, 0x11);
	ret |= galileo2_write8(i2c, 0x3157, 0x25);
	ret |= galileo2_write8(i2c, 0x3201, 0x00);

	/* Mapped couplet correct enable */
	ret |= galileo2_write8(i2c, MAPPED_COUPLET_CORRECT_ENABLE, 0x00);
	/* Single defect correct enable */
	ret |= galileo2_write8(i2c, SINGLE_DEFECT_CORRECT_ENABLE, 0x01);
	/* Single defect correct weight H */
	ret |= galileo2_write8(i2c, SINGLE_DEFECT_CORRECT_WEIGHT, 0x98);
	/* Couplet correct enable */
	ret |= galileo2_write8(i2c, COMBINED_COUPLET_SINGLE_DEFECT_CORRECT_ENABLE, 0x01);
	/* Single defect correct weight H */
	ret |= galileo2_write8(i2c, COMBINED_COUPLET_SINGLE_DEFECT_CORRECT_WEIGHT, 0x98);


	/* Set White gain (range : 1 to 20) */
	ret |= galileo2_write8(i2c, DM_LOW_GAIN_WHITE, 0x0C);
	ret |= galileo2_write8(i2c, DM_MID_GAIN_WHITE, 0x0A);
	ret |= galileo2_write8(i2c, DM_HIGH_GAIN_WHITE, 0x08);

	/* Set Black gain (range : 1 to 20) */
	ret |= galileo2_write8(i2c, DM_LOW_GAIN_BLACK, 0x40);
	ret |= galileo2_write8(i2c, DM_MID_GAIN_BLACK, 0x80);

	/* Defect correction */
	ret |= galileo2_write8(i2c, 0x3307, 0x2C);
	ret |= galileo2_write8(i2c, 0x3308, 0x20);

	/* Global reset configuration */
	ret |= galileo2_write8(i2c, GLOBAL_RESET_MODE_CONFIG1, 0xC1);

	/* Time clk_post configuration */
	ret |= galileo2_write8(i2c, 0x0800, 0x00);

	/* Disable line black level adjustment */
	ret |= galileo2_write8(i2c, 0x3200, 0x00);

	/* Set global black level adjustment pedestral */
	ret |= galileo2_write8(i2c, 0x3162, 0x00);

	/* Set Test pattern mode */
	ret |= galileo2_write8(i2c, 0x0601, 0x00);

	return ret;
}

static int galileo2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct galileo2               *galileo2 = to_galileo2(sd);
	int                            ret;

	if (enable == 0) {
		/* Nothing to do if we are already off */
		if (galileo2->streaming == 0)
			return 0;

		galileo2->streaming = 0;

		if (galileo2->gs->val) {
			galileo2_write8(galileo2->i2c_sensor,
					GLOBAL_RESET_CTRL1, 0x0);
			/* galileo2_write8(galileo2->i2c_pmic,
					MECH_SHUTTER_CONTROL, 0); */
		}

		galileo2_write8(galileo2->i2c_sensor, MODE_SELECT, 0x00);

		return 0;
	}

	if (!galileo2->timings_uptodate) {
		ret = galileo2_update_timings(sd);
		if (ret < 0) {
			v4l2_err(sd, "Unable to calculate Video Timing\n");
			return ret;
		}
	}

	/* Now that all needed pre-calculations are done, we can configure the
	 * device
	 */

	ret = galileo2_cam_config(sd);
	if (ret < 0) {
		v4l2_err(sd, "raytrix config failed\n");
		return ret;
	}

	ret = galileo2_apply_plls(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to apply plls\n");
		return ret;
	}

	ret = galileo2_configure(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to configure\n");
		return ret;
	}

	/* Stream on */
	galileo2->streaming = 1;
	galileo2_set_shutter(sd);
	galileo2_apply_hflip(sd);
	galileo2_apply_vflip(sd);
	galileo2_apply_ms(sd);
	galileo2_apply_nd(sd);
	galileo2_apply_flash_strobe(sd);
	galileo2_write8(galileo2->i2c_sensor, MODE_SELECT, 0x01);

	return 0;
}

static int galileo2_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct galileo2 *galileo2 = to_galileo2(sd);

	memset(fi, 0, sizeof(*fi));
	fi->interval = galileo2->frame_interval;

	return 0;
}

static int galileo2_s_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct galileo2           *galileo2 = to_galileo2(sd);
	struct i2c_client         *i2c      = galileo2->i2c_sensor;
	struct v4l2_rect          *vt       = &galileo2->video_timing;
	struct v4l2_rect          *c        = &galileo2->crop;
	struct v4l2_fract         *cur_fi   = &galileo2->frame_interval;
	struct v4l2_mbus_framefmt *fmt      = &galileo2->format;
	u32                        min_vt_height;

	*cur_fi = fi->interval;

	galileo2->timings_uptodate = 0;

	if (!galileo2->streaming)
		return 0;

	/* We bin as much as possible before scaling */
	galileo2->x_binning = c->width / fmt->width;
	galileo2->x_binning = min(galileo2->x_binning, 2U);

	galileo2->y_binning = c->height / fmt->height;
	galileo2->y_binning = min(galileo2->y_binning, 8U);

	if (galileo2->x_binning == 0)
		galileo2->x_binning = 1;

	if (galileo2->y_binning == 0)
		galileo2->y_binning = 1;

	/* We are already streaming, so we try to adjust the vertical blanking
	 * in order to match the frame rate.
	 */
	vt->width  = (c->width  / galileo2->x_binning) + GALIELO2_BINING_MIN_X;
	vt->height = div_u64((u64) galileo2->vtclk * cur_fi->numerator,
			     vt->width * cur_fi->denominator);

	/* In case min_vt_frame_blanking is not met, we adjust the frame rate */
	min_vt_height = c->height / galileo2->y_binning + GALIELO2_BINING_MIN_Y;

	if (vt->height < min_vt_height)
		vt->height = min_vt_height;

	/* Refresh FPS */
	cur_fi->denominator = galileo2->vtclk;
	cur_fi->numerator   = vt->width * vt->height;

	galileo2_write16(i2c, X_OUTPUT_SIZE, fmt->width);
	galileo2_write16(i2c, Y_OUTPUT_SIZE, fmt->height);
	galileo2_write16(i2c, OUTPUT_IMAGE_WIDTH,  fmt->width);

	galileo2_write16(i2c, GROUPED_PARAMETER_HOLD, 0x1);

	galileo2_write16(i2c, X_ADDR_START, c->left);
	galileo2_write16(i2c, Y_ADDR_START, c->top);
	galileo2_write16(i2c, X_ADDR_END,   c->left + c->width  - 1);
	galileo2_write16(i2c, Y_ADDR_END,   c->top  + c->height - 1);

	galileo2_write16(i2c, DIGITAL_CROP_IMAGE_WIDTH,
			 c->width / galileo2->x_binning);
	galileo2_write16(i2c, DIGITAL_CROP_IMAGE_HEIGHT,
			 c->height / galileo2->y_binning);

	/* Binning */
	if (galileo2->x_binning == 1 && galileo2->y_binning == 1) {
		galileo2_write8(i2c, BINNING_MODE, 0x0);
		galileo2_write8(i2c, BINNING_TYPE, 0x0);
	} else {
		galileo2_write8(i2c, BINNING_MODE, 0x1);
		galileo2_write8(i2c, BINNING_TYPE,
				galileo2->x_binning << 4 |
				galileo2->y_binning);
	}

	galileo2_write16(i2c, GROUPED_PARAMETER_HOLD, 0x0);

	galileo2_write16(i2c, VT_FRAME_LENGTH_LINES, vt->height);

	galileo2_write16(i2c, VT_LINE_LENGTH_PCK, vt->width);

	/* Refresh line_duration */
	galileo2->line_duration_ns = div_u64((u64) vt->width * 1000000000,
					     galileo2->vtclk);

	galileo2_apply_ms(sd);

	return 0;
}

static int galileo2_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct galileo2               *galileo2 = to_galileo2(sd);
	struct v4l2_mbus_framefmt     *fmt = &galileo2->format;
	struct v4l2_bt_timings        *bt = &timings->bt;
	int                            ret;

	memset(timings, 0, sizeof(*timings));

	/* We update the timing only when we are not streaming. Anyway, while
	 * streaming, it is forbidden to change the pixelcloclk.
	 */
	if (!galileo2->timings_uptodate && !galileo2->streaming) {
		ret = galileo2_update_timings(sd);
		if (ret < 0) {
			v4l2_err(sd, "Unable to calculate Video Timing\n");
			return ret;
		}
	}

	bt->width      = fmt->width;
	bt->height     = fmt->height;

	/* Pixel clock fixed to maximum : 100 MHz */
	bt->pixelclock = 100000000;

	/* Consider HSYNC and VSYNC as HACTIVE and VACTIVE*/
	bt->polarities = 0;

	/* Because we are in HACTIVE/VACTIVE mode, the blanking size does not
	 * matter for the capture device.
	 */

	return 0;
}

static const struct v4l2_subdev_video_ops galileo2_video_ops = {
	.s_stream         = galileo2_s_stream,
	.g_frame_interval = galileo2_g_frame_interval,
	.s_frame_interval = galileo2_s_frame_interval,
	.g_dv_timings     = galileo2_g_dv_timings,
};

static const struct v4l2_subdev_ops galileo2_ops = {
	.core  = &galileo2_core_ops,
	.video = &galileo2_video_ops,
	.pad   = &galileo2_pad_ops,
};

/* Custom ctrls */
#define V4L2_CID_GALILEO2_ND             (V4L2_CID_CAMERA_CLASS_BASE + 0x100)
#define V4L2_CID_GALILEO2_GS             (V4L2_CID_CAMERA_CLASS_BASE + 0x101)
#define V4L2_CID_GALILEO2_STROBE_WIDTH   (V4L2_CID_CAMERA_CLASS_BASE + 0x102)
#define V4L2_CID_GALILEO2_MS             (V4L2_CID_CAMERA_CLASS_BASE + 0x103)
#define V4L2_CID_GALILEO2_FSTROBE_START  (V4L2_CID_CAMERA_CLASS_BASE + 0x104)
#define V4L2_CID_GALILEO2_FSTROBE_ENABLE (V4L2_CID_CAMERA_CLASS_BASE + 0x105)
#define V4L2_CID_GALILEO2_TEMP_SENSOR_OUTPUT \
	(V4L2_CID_CAMERA_CLASS_BASE + 0x106)

static int galileo2_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct galileo2 *galileo2 = ctrl_to_galileo2(ctrl);

	/* If not streaming, just default value */
	if (!galileo2->streaming) {
		ctrl->val = ctrl->default_value;
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		galileo2_get_lens_position(&galileo2->sd, (u16 *)&ctrl->val);
		break;
	case V4L2_CID_GALILEO2_TEMP_SENSOR_OUTPUT:
		galileo2_get_temp_sensor_output(&galileo2->sd,
						(s8 *)&ctrl->val);
		break;
	}

	return 0;
}

static int galileo2_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct galileo2 *galileo2 = ctrl_to_galileo2(ctrl);

	/* If not streaming, just keep interval structures up-to-date */
	if (!galileo2->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		return galileo2_apply_hflip(&galileo2->sd);
	case V4L2_CID_VFLIP:
		return galileo2_apply_vflip(&galileo2->sd);
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		return galileo2_apply_exposure(&galileo2->sd);
	case V4L2_CID_FOCUS_ABSOLUTE:
		return galileo2_apply_focus(&galileo2->sd);
	case V4L2_CID_GALILEO2_ND:
		return galileo2_apply_nd(&galileo2->sd);
	case V4L2_CID_GALILEO2_STROBE_WIDTH:
	case V4L2_CID_GALILEO2_FSTROBE_START:
	case V4L2_CID_GALILEO2_FSTROBE_ENABLE:
		return galileo2_apply_flash_strobe(&galileo2->sd);
	case V4L2_CID_ANALOGUE_GAIN:
		return galileo2_apply_gain(&galileo2->sd);
	case V4L2_CID_GALILEO2_GS:
		return galileo2_apply_ms(&galileo2->sd);
	case V4L2_CID_GALILEO2_MS:
		return galileo2_set_shutter(&galileo2->sd);
	}

	return 0;
}

static const struct v4l2_ctrl_ops galileo2_ctrl_ops = {
	.g_volatile_ctrl = galileo2_g_volatile_ctrl,
	.s_ctrl          = galileo2_s_ctrl,
};

static const struct v4l2_ctrl_config galileo2_ctrl_nd = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_ND,
	.name = "Neutral Density Filter",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = false,
};

static const struct v4l2_ctrl_config galileo2_ctrl_gs = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_GS,
	.name = "Global Shutter",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = false,
};

static const struct v4l2_ctrl_config galileo2_ctrl_ms = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_MS,
	.name = "Mechanical Shutter",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 2,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config galileo2_ctrl_sw = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_STROBE_WIDTH,
	.name = "Flash strobe width, in us",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 1,
	.max  = 50000,
	.step = 1,
	.def  = 100,
};

static const struct v4l2_ctrl_config galileo2_ctrl_ss = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_FSTROBE_START,
	.name = "Flash strobe start delay, in us",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 50000,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config galileo2_ctrl_se = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_FSTROBE_ENABLE,
	.name = "Flash strobe pulse enable state",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 1,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config galileo2_ctrl_tso = {
	.ops  = &galileo2_ctrl_ops,
	.id   = V4L2_CID_GALILEO2_TEMP_SENSOR_OUTPUT,
	.name = "Temperature sensor output",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = GALILEO2_TEMP_MIN,
	.max  = GALILEO2_TEMP_MAX,
	.step = 1,
	.def  = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static int galileo2_initialize_controls(struct v4l2_subdev *sd)
{
	struct galileo2          *galileo2 = to_galileo2(sd);
	struct v4l2_ctrl_handler *hdl      = &galileo2->ctrl_handler;
	int                       ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0) {
		v4l2_err(sd, "failed to init ctrl handler\n");
		goto einit;
	}

	/* Flips */
	galileo2->hflip = v4l2_ctrl_new_std(hdl,
					    &galileo2_ctrl_ops,
					    V4L2_CID_HFLIP,
					    0, 1, 1, 0);

	galileo2->vflip = v4l2_ctrl_new_std(hdl,
					    &galileo2_ctrl_ops,
					    V4L2_CID_VFLIP,
					    0, 1, 1, 0);

	/* Exposure in us */
	galileo2->exposure = v4l2_ctrl_new_std(hdl,
					       &galileo2_ctrl_ops,
					       V4L2_CID_EXPOSURE_ABSOLUTE,
					       0, 1000000, 1, 20000);

	/* Focus */
	/* The initializing values for AF could be read from NVM
	 * For now it is not a priority
	 * range 0 - 1024 step 1 / default 0 (for saving power)
	 */
	galileo2->focus = v4l2_ctrl_new_std(hdl, &galileo2_ctrl_ops,
					    V4L2_CID_FOCUS_ABSOLUTE,
					    0,
					    1023,
					    1,
					    0);

	/* Since the lens can move even if no command has been sent, flag the
	 * control as volatile.
	 */
	galileo2->focus->flags |= V4L2_CTRL_FLAG_VOLATILE;

	/* Neutral Density Filter */
	galileo2->nd = v4l2_ctrl_new_custom(hdl,
					    &galileo2_ctrl_nd, NULL);

	/* Global Shutter */
	galileo2->gs = v4l2_ctrl_new_custom(hdl,
					    &galileo2_ctrl_gs, NULL);

	/* Mechanical shutter control */
	galileo2->ms = v4l2_ctrl_new_custom(hdl,
					    &galileo2_ctrl_ms, NULL);

	/* Flash strobe width */
	galileo2->strobe_width = v4l2_ctrl_new_custom(hdl,
						      &galileo2_ctrl_sw, NULL);

	/* Flash strobe start */
	galileo2->strobe_start = v4l2_ctrl_new_custom(hdl,
						      &galileo2_ctrl_ss, NULL);

	/* Flash strobe enable */
	galileo2->strobe_enable = v4l2_ctrl_new_custom(hdl,
						       &galileo2_ctrl_se, NULL);

	/* Analog gain
	** value based on 'raytrix driver' and checked by i2c sniffing on phone
	*/
	galileo2->gain = v4l2_ctrl_new_std(hdl,
					   &galileo2_ctrl_ops,
					   V4L2_CID_ANALOGUE_GAIN,
					   0, 0x208, 1, 5 * 0x34);

	/* Temperature sensor output */
	galileo2->temp_sensor_output = v4l2_ctrl_new_custom(hdl,
							    &galileo2_ctrl_tso,
							    NULL);

	if (hdl->error) {
		v4l2_err(sd, "failed to add new ctrls\n");
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

static void galileo2_free_controls(struct v4l2_subdev *sd)
{
	v4l2_ctrl_handler_free(sd->ctrl_handler);
}

static int galileo2_detect_chip(struct v4l2_subdev *sd)
{
	struct galileo2 *galileo2 = to_galileo2(sd);
	u16              chip_id;

	galileo2_read16(galileo2->i2c_sensor, SENSOR_MODEL_ID, &chip_id);

	if (chip_id != GALILEO2_CHIPID) {
		v4l2_err(sd, "Error Chipd ID = 0x%04x instead of 0x%04x\n",
			     chip_id, GALILEO2_CHIPID);
		return -ENODEV;
	}

	v4l2_info(sd, "Found " DRIVER_NAME " chip\n");

	return 0;
}

static int galileo2_read_nvm(struct v4l2_subdev *sd)
{
	struct galileo2   *galileo2 = to_galileo2(sd);
	struct i2c_client *i2c = galileo2->i2c_sensor;
	u8                *nvm = galileo2->nvm;
	u8                 page = 0;
	int                ret = 0;

	/* Enable Read */
	galileo2_write8(i2c, DATA_TRANSFER_IF_1_CTRL, 0x1);

	for (page = 0 ; page < NVM_PAGE_NB ; page++) {
		unsigned int i;

		union data_transfer_if_1_status status = {
			.read_if_ready = 0,
		};

		/* Select page */
		galileo2_write8(i2c, DATA_TRANSFER_IF_1_PAGE_SELECT, page);

		/* Check Status */
		while (!status.read_if_ready) {
			galileo2_read8(i2c, DATA_TRANSFER_IF_1_STATUS,
					    &status._register);

			if (status.improper_if_usage || status.data_corrupted) {
				v4l2_err(sd, "NVM Data transfer IF is bad\n");
				ret = -EINVAL;
				goto out;
			}
		}

		/* Read the entire page (64 bytes)
		 * If it is taking too long, it can be optimized into reading
		 * the entire page in one i2c xfer.
		 */
		for (i = 0 ; i < NVM_PAGE_SZ ; i++)
			galileo2_read8(i2c, DATA_TRANSFER_IF_1_DATA + i,
					nvm + NVM_PAGE_SZ * page +  i);
	}

out:
	galileo2_write8(i2c, DATA_TRANSFER_IF_1_CTRL, 0x0);

	/* Check Version */
	if (*nvm != NVM_VERSION) {
		v4l2_err(sd, "NVM Version (0x%02x) is not correct\n", *nvm);
		v4l2_err(sd, "Expecting 0x%02x\n", NVM_VERSION);
		ret = -ENODEV;
	}

	return ret;
}

static ssize_t galileo2_nvm_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct galileo2    *galileo2 = to_galileo2(sd);

	memcpy(buf, galileo2->nvm, NVM_SIZE);

	return NVM_SIZE;
}

DEVICE_ATTR(nvm, S_IRUGO, galileo2_nvm_show, NULL);

static int galileo2_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct galileo2               *galileo2;
	struct galileo2_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev            *sd;
	int                            ret = 0;

	if (pdata == NULL) {
		dev_err(&client->dev, "platform data not specified\n");
		return -EINVAL;
	}

	if (pdata->refclk == 0) {
		dev_err(&client->dev, "refclk frequency is not specified\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "i2c not available\n");
		return -ENODEV;
	}

	galileo2 = kzalloc(sizeof(*galileo2), GFP_KERNEL);
	if (!galileo2) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	galileo2->pdata = pdata;

	sd = &galileo2->sd;
	v4l2_i2c_subdev_init(sd, client, &galileo2_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	galileo2->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &galileo2->pad, 0);

	if (ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto emedia;
	}

	galileo2->i2c_sensor = client;

	/* Set default configuration:
	 *   Max sensor crop into 720p30
	 */
	galileo2->format.width  = 1280;
	galileo2->format.height = 720;
	galileo2->format.code   = V4L2_MBUS_FMT_SGBRG10_1X10;

	/* Center the crop */
	galileo2->crop.width    = 7680;
	galileo2->crop.height   = 4320;
	galileo2->crop.left     = 24;
	galileo2->crop.top      = 524;

	/* 30 FPS */
	galileo2->frame_interval.numerator   =  1;
	galileo2->frame_interval.denominator = 30;

	/* Make sure all clocks info are up-to-date */
	ret = galileo2_update_timings(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate Video Timing\n");
		goto eupdate;
	}

	if (pdata->set_power) {
		ret = pdata->set_power(GALILEO2_POWER_ON);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	/* Check if the chip is preset */
	ret = galileo2_detect_chip(sd);
	if (ret < 0)
		goto edetect;

	/* Make sure the shutter is closed */
	galileo2_drive_shutter(sd, 0);

	/* Non-Volatile Memory */
	ret = device_create_file(&client->dev, &dev_attr_nvm);
	if (ret < 0) {
		v4l2_err(sd, "Sysfs nvm entry creation failed\n");
		goto esysfs;
	}

	galileo2->nvm = kzalloc(NVM_SIZE, GFP_KERNEL);
	if (!galileo2->nvm) {
		v4l2_err(sd, "alloc failed for NVM structure\n");
		ret = -ENOMEM;
		goto enomem;
	}

	ret = galileo2_read_nvm(sd);
	if (ret < 0) {
		v4l2_err(sd, "Failed to read NVM\n");
		goto envm;
	}

	/* Extract NVM Memory map */
	galileo2->nvm_addr._registers =
		swab64(*(u64 *)(galileo2->nvm + NVM_MEMORY_ADDRESS));

	/* Get temperature sensor calibration */
	galileo2->nvm_temp_h = *(galileo2->nvm + 0x0413); /* 60 degrees C */
	galileo2->nvm_temp_l = *(galileo2->nvm + 0x0414); /* 25 degrees C*/

	/* Initialize Control */
	ret = galileo2_initialize_controls(sd);
	if (ret < 0)
		goto einitctrl;

	return 0;

einitctrl:
envm:
	kfree(galileo2->nvm);
enomem:
	device_remove_file(&client->dev, &dev_attr_nvm);
esysfs:
edetect:
	if (pdata->set_power)
		pdata->set_power(GALILEO2_POWER_OFF);
eupdate:
emedia:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(galileo2);

	return ret;
}

static int galileo2_remove(struct i2c_client *client)
{
	struct v4l2_subdev            *sd = i2c_get_clientdata(client);
	struct galileo2               *galileo2 = to_galileo2(sd);
	struct galileo2_platform_data *pdata = client->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	if (pdata->set_power)
		pdata->set_power(GALILEO2_POWER_OFF);

	device_remove_file(&client->dev, &dev_attr_nvm);
	galileo2_free_controls(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(galileo2->nvm);
	kfree(galileo2);

	return 0;
}

static const struct i2c_device_id galileo2_id[] = {
	{DRIVER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, galileo2_id);

static struct i2c_driver galileo2_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe    = galileo2_probe,
	.remove   = galileo2_remove,
	.id_table = galileo2_id,
};

module_i2c_driver(galileo2_driver);
