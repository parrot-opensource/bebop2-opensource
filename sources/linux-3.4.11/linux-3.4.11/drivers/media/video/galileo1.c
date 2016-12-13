/*
 * galileo1 - Nokia sensor
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

#include <media/galileo1.h>

#include "galileo1_reg.h"

static int debug_print;

module_param(debug_print, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(debug_print, "print debug information if > 0");

MODULE_AUTHOR("Eng-Hong SRON <eng-hong.sron@parrot.com>");
MODULE_DESCRIPTION("Nokia Galileo1 driver");
MODULE_LICENSE("GPL");

#define DRIVER_NAME "galileo1"

#define SENSOR_WIDTH   7728
#define SENSOR_HEIGHT  5368

#define MIN_VTCLK          20000000UL
#define MAX_VTCLK         256000000UL

/* Here we take a MIPICLK slightly higher than the specification on purpose,
 * because we are using the TC358746A bridge and it has its own limitation
 * (PPICLK need to be between 66 and 125 MHz).
 * In case of other bridge, we could go back to the specified value (80.0 MHz).
 */
#define MIN_MIPICLK        82500000UL
#define MAX_MIPICLK      1000000000UL

#define MIN_REFCLK          6000000UL
#define MAX_REFCLK         27000000UL

#define MIN_PLL_IN_CLK      3000000UL
#define MAX_PLL_IN_CLK     27000000UL

#define MIN_PLL_OP_CLK   1000000000UL
#define MAX_PLL_OP_CLK   2080000000UL

#define MIN_VT_SYS_CLK     83330000UL
#define MAX_VT_SYS_CLK   2080000000UL

/* Temperature sensor output */
#define TEMP_SENSOR_OUTPUT_MIN           (-20)
#define TEMP_SENSOR_OUTPUT_MAX           (80)
#define TEMP_SENSOR_OUTPUT_LOW_CALIB     (25)
#define TEMP_SENSOR_OUTPUT_HIGH_CALIB    (60)
#define TEMP_SENSOR_OUTPUT_AVERAGE_CALIB (42)

struct galileo1 {
	struct v4l2_subdev             sd;
	struct media_pad               pad;
	struct galileo1_platform_data *pdata;

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
	struct i2c_client             *i2c_pmic;

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

static inline struct galileo1 *to_galileo1(struct v4l2_subdev *sd)
{
	return container_of(sd, struct galileo1, sd);
}

static inline struct galileo1 *ctrl_to_galileo1(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct galileo1, ctrl_handler);
}

static int galileo1_read8(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
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
			.len   = 1,
			.buf   = val,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int galileo1_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
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

static int galileo1_write8(struct i2c_client *client, u16 reg, u8 val)
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

static int galileo1_write16(struct i2c_client *client, u16 reg, u16 val)
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

static int galileo1_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	struct galileo1 *galileo1 = to_galileo1(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, 0);
		fmt->format = *mf;
		return 0;
	}

	fmt->format = galileo1->format;

	return 0;
}

static int galileo1_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct galileo1           *galileo1 = to_galileo1(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(fh, fmt->pad);
		fmt->format = *mf;
	} else {
		galileo1->format = fmt->format;
	}

	galileo1->timings_uptodate = 0;

	return 0;
}

static int galileo1_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	/* For now we support only one code */
	if (code->index)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGBRG10_1X10;

	return 0;
}

static int galileo1_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_selection *sel)
{
	struct galileo1 *galileo1 = to_galileo1(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left   = 0;
		sel->r.top    = 0;
		sel->r.width  = SENSOR_WIDTH;
		sel->r.height = SENSOR_HEIGHT;
		break;
	case V4L2_SEL_TGT_CROP:
		sel->r = galileo1->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
** Only use for DEBUG :
** print the frame format according to SMIA++ specs
*/
static void galileo1_print_frame_format(struct v4l2_subdev *sd)
{
	struct galileo1 *galileo1 = to_galileo1(sd);
	u8              frame_format_model_type, frame_format_model_subtype, descriptors_number;
	u16             idx;
	u16             reg;
	u8              reg1, reg2[4];

	galileo1_read8(galileo1->i2c_sensor, 0x0040, &frame_format_model_type);
	galileo1_read8(galileo1->i2c_sensor, 0x0041, &frame_format_model_subtype);
	descriptors_number = (frame_format_model_subtype >> 4) + (frame_format_model_subtype & 0x0F);

	v4l2_info(sd, "frame format model type %#x - subtype %#x\n",
		frame_format_model_type, frame_format_model_subtype);

	if (frame_format_model_type == 0x01) {
		for (idx = 0x0042; idx < 0x0042 + 2 * descriptors_number; idx += 0x002) {
			galileo1_read16(galileo1->i2c_sensor, 0x0040, &reg);
			v4l2_info(sd, "frame format desciptor %u = %#x\n", (idx - 0x0042)/2, reg);
		}
	}

	if (frame_format_model_type == 0x02) {
		for (idx = 0x0060; idx < 0x0060 + 4 * descriptors_number; idx += 4) {
			galileo1_read8(galileo1->i2c_sensor, idx, &reg2[0]);
			galileo1_read8(galileo1->i2c_sensor, idx+1, &reg2[1]);
			galileo1_read8(galileo1->i2c_sensor, idx+2, &reg2[2]);
			galileo1_read8(galileo1->i2c_sensor, idx+3, &reg2[3]);

			v4l2_info(sd, "frame format desciptor 4_%u = %#x - %#x - %#x - %#x\n",
				(idx - 0x0060)/4, reg2[0], reg2[1], reg2[2], reg2[3]);
		}
	}

	galileo1_read8(galileo1->i2c_sensor, 0x0011, &reg1);
	v4l2_info(sd, "SMIA++ rev. 0.%d.\n", reg1);
	galileo1_read16(galileo1->i2c_sensor, 0x0008, &reg);
	v4l2_info(sd, "Pedestal register reads %04x\n", reg);
	galileo1_read8(galileo1->i2c_sensor, 0x1900, &reg1);
	v4l2_info(sd, "Shading corr. capability reads %04x\n", reg1);
	galileo1_read8(galileo1->i2c_sensor, 0x1901, &reg1);
	v4l2_info(sd, "Green imb. capability reads %04x\n", reg1);
	galileo1_read8(galileo1->i2c_sensor, 0x1902, &reg1);
	v4l2_info(sd, "Black level capability reads %04x\n", reg1);
	galileo1_read8(galileo1->i2c_sensor, 0x1903, &reg1);
	v4l2_info(sd, "Module specific capability reads %04x\n", reg1);
}

/* Compute VT timing and binning */
static int galileo1_calc_vt(struct v4l2_subdev *sd)
{
	struct galileo1           *galileo1 = to_galileo1(sd);
	struct v4l2_rect          *vt  = &galileo1->video_timing;
	struct v4l2_rect          *c   = &galileo1->crop;
	struct v4l2_mbus_framefmt *fmt = &galileo1->format;

	/* We bin as much as possible before scaling */
	galileo1->x_binning = c->width / fmt->width;
	galileo1->x_binning = min(galileo1->x_binning, 2U);

	galileo1->y_binning = c->height / fmt->height;
	galileo1->y_binning = min(galileo1->y_binning, 8U);

	/* Video Timing is working on binned pixels
	 *   min_vt_line_blanking_pck is 512
	 *   min_vt_frame_blanking_line is 38
	 */
	vt->width  = (c->width  / galileo1->x_binning) + 512;
	vt->height = (c->height / galileo1->y_binning) +  42;

	/* It seems there is a minimum VT width which differs from what the
	 * datasheet says (8240). It is an empiric value, I don't know if it is
	 * correct... */
	vt->width = max(vt->width, 1920);

	return 0;
}

static int galileo1_set_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_selection *sel)
{
	struct galileo1           *galileo1 = to_galileo1(sd);
	struct v4l2_rect          *c   = &galileo1->crop;
	struct v4l2_mbus_framefmt *fmt = &galileo1->format;
	struct i2c_client         *i2c = galileo1->i2c_sensor;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		galileo1->crop = sel->r;
		break;
	default:
		v4l2_err(sd, "selection target (%d) not supported yet\n",
			     sel->target);
		return -EINVAL;
	}

	galileo1->timings_uptodate = 0;

	if (!galileo1->streaming)
		return 0;

	/* We bin as much as possible before scaling */
	galileo1->x_binning = c->width / fmt->width;
	galileo1->x_binning = min(galileo1->x_binning, 2U);

	galileo1->y_binning = c->height / fmt->height;
	galileo1->y_binning = min(galileo1->y_binning, 8U);

	galileo1_write16(i2c, GROUPED_PARAMETER_HOLD, 0x1);

	galileo1_write16(i2c, X_ADDR_START, c->left);
	galileo1_write16(i2c, Y_ADDR_START, c->top);
	galileo1_write16(i2c, X_ADDR_END,   c->left + c->width  - 1);
	galileo1_write16(i2c, Y_ADDR_END,   c->top  + c->height - 1);

	galileo1_write16(i2c, DIGITAL_CROP_IMAGE_WIDTH,
			      c->width / galileo1->x_binning);
	galileo1_write16(i2c, DIGITAL_CROP_IMAGE_HEIGHT,
			      c->height / galileo1->y_binning);

	galileo1_write8(i2c, BINNING_TYPE, galileo1->x_binning << 4 |
					   galileo1->y_binning);

	galileo1_write16(i2c, GROUPED_PARAMETER_HOLD, 0x0);

	return 0;
}

static const struct v4l2_subdev_pad_ops galileo1_pad_ops = {
	.get_fmt        = galileo1_get_fmt,
	.set_fmt        = galileo1_set_fmt,
	.enum_mbus_code = galileo1_enum_mbus_code,
	.get_selection  = galileo1_get_selection,
	.set_selection  = galileo1_set_selection,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int galileo1_get_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct galileo1 *galileo1 = to_galileo1(sd);
	int              ret;
	u8               val;

	reg->size = 2;

	if (reg->reg & ~0xff)
		return -EINVAL;

	ret = galileo1_read8(galileo1->i2c_sensor, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = (__u64)val;

	return 0;
}

static int galileo1_set_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct galileo1 *galileo1 = to_galileo1(sd);

	if (reg->reg & ~0xff || reg->val & ~0xff)
		return -EINVAL;

	return galileo1_write8(galileo1->i2c_sensor, reg->reg, reg->val);
}
#endif

static const struct v4l2_subdev_core_ops galileo1_core_ops = {
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = galileo1_get_register,
	.s_register = galileo1_set_register,
#endif
};

/* Compute minimum clocks in order to reach the FPS */
static int galileo1_calc_clocks(struct v4l2_subdev *sd)
{
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = galileo1->pdata;
	struct v4l2_rect              *vt    = &galileo1->video_timing;
	struct v4l2_fract             *fi    = &galileo1->frame_interval;
	struct v4l2_rect              *c     = &galileo1->crop;
	struct v4l2_mbus_framefmt     *fmt   = &galileo1->format;
	u64                            mipiclk_numerator;
	u64                            mipiclk_denominator;

	galileo1->vtclk = div_u64((u64) vt->width * vt->height * fi->denominator,
			          fi->numerator);

	/* In case vtclk is too high, we need to adjust the frame interval */
	if (galileo1->vtclk > MAX_VTCLK) {
		galileo1->vtclk = MAX_VTCLK;

		fi->denominator = galileo1->vtclk;
		fi->numerator   = vt->width * vt->height;

	/* In case vtclk is too low, we just increase the vertical blanking */
	} else if (galileo1->vtclk < MIN_VTCLK) {
		galileo1->vtclk = MIN_VTCLK;

		vt->height = div_u64((u64) galileo1->vtclk * fi->numerator,
			             vt->width * fi->denominator);

	}

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
		galileo1->bits_per_pixel = 10;
		break;
	default:
		v4l2_err(sd, "code not supported yet\n");
		galileo1->vtclk = 0;
		galileo1->mipiclk = 0;
		return -EINVAL;
	}

	mipiclk_numerator   = (u64) galileo1->vtclk *
		                    galileo1->bits_per_pixel *
			            fmt->width *
			            galileo1->x_binning;

	mipiclk_denominator = c->width * pdata->lanes * 2;

	galileo1->mipiclk = div_u64(mipiclk_numerator, mipiclk_denominator);

	/* In case mipiclk is too low, we just strech vtclk and vertical
	 * blanking.
	 */
	if (galileo1->mipiclk < MIN_MIPICLK) {
		vt->height = div_u64((u64) vt->height * MIN_MIPICLK,
				     galileo1->mipiclk);

		galileo1->vtclk = div_u64((u64) galileo1->vtclk * MIN_MIPICLK,
					  galileo1->mipiclk);

		galileo1->mipiclk = MIN_MIPICLK;

	}

	return 0;
}

#define IS_BETWEEN(_f, _min, _max) ((_f >= _min) && (_f <= _max))

/* Try to reach vtclk and mipiclk from the same PLL. We give the 'priority' to
 * vtclk, since it is the processing clock whereas mipiclk is 'just' the output
 * clock.
 * We are also trying to keep the targeted FPS (if specified so)
 */
static int galileo1_pll_brute_force(struct v4l2_subdev *sd, int keep_fps)
{
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = galileo1->pdata;
	struct v4l2_rect              *vt    = &galileo1->video_timing;
	struct v4l2_fract             *fi    = &galileo1->frame_interval;

	const u16 pre_pll_div[] = {1, 2,  4};
	const u16 vt_sys_div[]  = {1, 2,  4,  6,  8, 10, 12};
	const u16 vt_pix_div[]  = {4, 5,  6,  7,  8,  9, 10, 12};
	const u16 op_sys_div[]  = {2, 4, 12, 16, 20, 24};
	long      best_error    = -1;
	int       ret = -EINVAL;

	/* PLL parameters */
	u32 p,   best_p   = 0;
	u32 m,   best_m   = 0;
	u32 vts, best_vts = 0;
	u32 vtp, best_vtp = 0;
	u32 op,  best_op  = 0;

	/* Brute force PLL */
	for (p = 0 ; p < ARRAY_SIZE(pre_pll_div) ; p++) {
		unsigned long pll_in_clk = pdata->refclk / pre_pll_div[p];

		if (!IS_BETWEEN(pll_in_clk, MIN_PLL_IN_CLK, MAX_PLL_IN_CLK))
			continue;

		for (m = 36 ; m <= 832 ; m++) {
			unsigned long pll_op_clk = pll_in_clk * m;

			if (!IS_BETWEEN(pll_op_clk, MIN_PLL_OP_CLK,
						    MAX_PLL_OP_CLK))
				continue;

			for (vts = 0 ; vts < ARRAY_SIZE(vt_sys_div) ; vts++) {
				unsigned long vt_sys_clk;

				vt_sys_clk = pll_op_clk / vt_sys_div[vts];
				if (!IS_BETWEEN(vt_sys_clk, MIN_VT_SYS_CLK, MAX_VT_SYS_CLK))
					continue;

				for (vtp = 0 ; vtp < ARRAY_SIZE(vt_pix_div) ; vtp++) {
					unsigned long vtclk;

					vtclk = vt_sys_clk / vt_pix_div[vtp];

					if (!IS_BETWEEN(vtclk, MIN_VTCLK, MAX_VTCLK))
						continue;

					for (op = 0 ; op < ARRAY_SIZE(op_sys_div) ; op++) {
						unsigned long mipiclk;
						long error;
						long vt_error;
						long mipi_error;

						mipiclk = pll_op_clk / op_sys_div[op] / 2;

						vt_error   = vtclk   - galileo1->vtclk;
						mipi_error = mipiclk - galileo1->mipiclk;

						/* Don't go lower than the
						 * targeted frequencies,
						 * otherwise we won't be able to
						 * reach the FPS.
						 */
						if (keep_fps == 1) {
							if (vt_error < 0)
								continue;

							if (mipi_error < 0)
								continue;
						} else {
							if (vt_error < 0)
								vt_error = -vt_error;

							if (mipi_error < 0)
								mipi_error = -mipi_error;
						}

						/* Try to minimize both error */
						error = mipi_error + vt_error;

						if (error <= best_error || best_error < 0) {
							ret = 0;
							best_error = error;
							best_p     = pre_pll_div[p];
							best_m     = m;
							best_vts   = vt_sys_div[vts];
							best_vtp   = vt_pix_div[vtp];
							best_op    = op_sys_div[op];
						}
					}
				}
			}
		}
	}

	if (ret != 0)
		return ret;

	/* Refresh clock frequencies */
	galileo1->vtclk =   (pdata->refclk * best_m) /
			    (best_p * best_vts * best_vtp);
	galileo1->mipiclk = (pdata->refclk * best_m) /
			    (best_p * best_op * 2);

	/* Refresh FPS */
	fi->denominator = galileo1->vtclk;
	fi->numerator   = vt->width * vt->height;

	/* Refresh line_duration */
	galileo1->line_duration_ns = div_u64((u64) vt->width * 1000000000,
				             galileo1->vtclk);

	galileo1->pll1.pre_pll_clk_div = best_p;
	galileo1->pll1.pll_multiplier  = best_m;
	galileo1->pll1.vt_sys_clk_div  = best_vts;
	galileo1->pll1.vt_pix_clk_div  = best_vtp;
	galileo1->pll1.op_sys_clk_div  = best_op;

	return 0;
}

static int galileo1_calc_plls(struct v4l2_subdev *sd)
{
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = galileo1->pdata;

	/* PLL0 parameters */
	const u16 pre_pll_div[] = {1, 2, 4};
	long      best_error = -1;
	u32       p, best_p = 0;
	u32       m, best_m = 0;


	/* Perform some sanity checks */
	if (!IS_BETWEEN(galileo1->mipiclk, MIN_MIPICLK, MAX_MIPICLK)) {
		v4l2_err(sd, "mipiclk (%lu) is out of range [%lu - %lu]\n",
			 galileo1->mipiclk, MIN_MIPICLK, MAX_MIPICLK);
		return -EINVAL;
	}

	if (!IS_BETWEEN(galileo1->vtclk, MIN_VTCLK, MAX_VTCLK)) {
		v4l2_err(sd, "vtclk (%lu) is out of range [%lu - %lu]\n",
			 galileo1->vtclk, MIN_VTCLK, MAX_VTCLK);
		return -EINVAL;
	}

	if (!IS_BETWEEN(pdata->refclk, MIN_REFCLK, MAX_REFCLK)) {
		v4l2_err(sd, "refclk (%lu) is out of range [%lu - %lu]\n",
			 galileo1->mipiclk, MIN_REFCLK, MAX_REFCLK);
		return -EINVAL;
	}

	/* Try to reach the PLL frequencies while preserving the FPS, but in
	 * case it is not possible, we have to derate it.
	 */
	if (galileo1_pll_brute_force(sd, 1) < 0) {
		if (galileo1_pll_brute_force(sd, 0) < 0) {
			v4l2_err(sd, "Unable to find PLL config for:\n");
			v4l2_err(sd, "  vtclk    %lu", galileo1->vtclk);
			v4l2_err(sd, "  mipiclk  %lu", galileo1->mipiclk);
			return -EINVAL;
		}
	}

	/* TOSHIBA register setting
	 * I don't know what frequency is needed for the following BoostCK,
	 * ADC Clock, ck_st and hreg_clk...
	 * So I follow the given spreadsheet...
	 * I also assume the PLL0 constraints are the same as the PLL1.
	 */
	for (p = 0 ; p < ARRAY_SIZE(pre_pll_div) ; p++) {
		unsigned long pll_in_clk = pdata->refclk / pre_pll_div[p];

		if (!IS_BETWEEN(pll_in_clk, MIN_PLL_IN_CLK, MAX_PLL_IN_CLK))
			continue;
		for (m = 36 ; m <= 832 ; m++) {
			unsigned long pll_op_clk = pll_in_clk * m;

			/* Trying to reach 1GHz, again, it seems to work that
			 * way, but I don't know why...
			 */
			long error = 1000000000UL - pll_op_clk;

			if (error < 0)
				error = -error;

			if (error < best_error || best_error < 0) {
				best_error = error;
				best_p = pre_pll_div[p] - 1;
				best_m = m;
			}
		}
	}

	galileo1->pll0.pre_pll_clk_div = best_p;
	galileo1->pll0.pll_multiplier  = best_m;

	return 0;
}

#undef  IS_BETWEEN

static int galileo1_update_timings(struct v4l2_subdev *sd)
{
	struct galileo1 *galileo1 = to_galileo1(sd);
	int              ret;

	/* From the crop and the output size, calculate the binning and the
	 * Video Timing.
	 */
	ret = galileo1_calc_vt(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate Video Timing\n");
		return ret;
	}

	/* Calculate the the minimum theorical clock frequency in order to
	 * achieve the frame interval.
	 */
	ret = galileo1_calc_clocks(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate Clocks\n");
		return ret;
	}

	/* Update clocks */
	ret = galileo1_calc_plls(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate plls\n");
		return ret;
	}

	galileo1->timings_uptodate = 1;

	return 0;
}

static int galileo1_apply_plls(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c_sensor = galileo1->i2c_sensor;

	galileo1_write16(i2c_sensor, PRE_PLL_CLK_DIV,
			             galileo1->pll1.pre_pll_clk_div);
	galileo1_write16(i2c_sensor, PLL_MULTIPLIER,
			             galileo1->pll1.pll_multiplier);
	galileo1_write16(i2c_sensor, VT_SYS_CLK_DIV,
			             galileo1->pll1.vt_sys_clk_div);
	galileo1_write16(i2c_sensor, VT_PIX_CLK_DIV,
			             galileo1->pll1.vt_pix_clk_div);
	galileo1_write16(i2c_sensor, OP_SYS_CLK_DIV,
			             galileo1->pll1.op_sys_clk_div);
	galileo1_write16(i2c_sensor, OP_PIX_CLK_DIV,
					       0x0008);

	galileo1_write8(i2c_sensor, PRE_PLL_CNTL_ST,
					      galileo1->pll0.pre_pll_clk_div);
	galileo1_write16(i2c_sensor, PLL_MULTI_ST,
					       galileo1->pll0.pll_multiplier);

	galileo1_write8(i2c_sensor, AD_CNTL,            0x02);
	galileo1_write8(i2c_sensor, ST_CNTL,            0x07);
	galileo1_write8(i2c_sensor, HREG_CNTL,          0x05);
	galileo1_write8(i2c_sensor, PLL_HRG_CNTL,       0x01);
	galileo1_write8(i2c_sensor, HREG_PLLSEL_SINGLE, 0x10);
	galileo1_write8(i2c_sensor, OPCK_PLLSEL,        0x00);

	return 0;
}

/*
 * MODEPowerup_and_Initialize
 * Following values are taken directly from Nokia INIT.txt file.
 * I have no idea what it does...
 */
static int galileo1_init(struct v4l2_subdev *sd)
{
	struct galileo1                 *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data   *pdata = galileo1->pdata;
	struct i2c_client               *i2c_sensor = galileo1->i2c_sensor;
	struct i2c_client               *i2c_pmic   = galileo1->i2c_pmic;
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
	}};

	/*
	 * AD5814 settings
	 */
	galileo1_write8(i2c_pmic, CONTROL,           0x00);
	/* DRIVE_CFG Slope 001(275ns) H Bridge,DIV2 */
	galileo1_write8(i2c_pmic, DRIVE_CFG,         0x39);
	galileo1_write8(i2c_pmic, BUCK_CFG,          0x03);
	/* TIMING 20ms */
	galileo1_write8(i2c_pmic, TIMING,            0x58);
	/* CONFIG 2.5V */
	galileo1_write8(i2c_pmic, CONFIG,            0x10);
	/* PERIOD 15.6us */
	galileo1_write8(i2c_pmic, PERIOD,            0x9C);
	/* TAH 0.8us */
	galileo1_write8(i2c_pmic, TAH,               0x08);
	/* TAL 14.8us */
	galileo1_write8(i2c_pmic, TAL,               0x94);
	/* TBH 12.8us */
	galileo1_write8(i2c_pmic, TBH,               0x78);
	/* TBL 1.7us */
	galileo1_write8(i2c_pmic, TBL,               0x11);
	galileo1_write8(i2c_pmic, NR_CFG,            0x04);
	galileo1_write8(i2c_pmic, NR_PERIOD,         0x02);
	galileo1_write8(i2c_pmic, NR_TAH,            0x01);
	galileo1_write8(i2c_pmic, NR_TAL,            0x01);
	galileo1_write8(i2c_pmic, NR_TBH,            0x01);
	galileo1_write8(i2c_pmic, NR_TBL,            0x01);
	galileo1_write16(i2c_pmic, NR_PRE_PULSE,   0x0000);
	galileo1_write16(i2c_pmic, NR_POST_PULSE,  0x0000);
	galileo1_write8(i2c_pmic, NR_RISEFALL,       0x74);
	/* NR_RF_PULSE NR 4 PLS */
	galileo1_write8(i2c_pmic, NR_RF_PULSE,       0x04);
	/* CURRENT 10.5mA */
	galileo1_write8(i2c_pmic, CURRENT,           0x13);
	/* IPOS_TEMPCOMP -0.10%/deg c */
	galileo1_write8(i2c_pmic, IPOS_TEMPCOMP,     0x02);
	/*ADC_CONFIG 16Ave 200uA */
	galileo1_write8(i2c_pmic, ADC_CONFIG,        0x18);
	galileo1_write8(i2c_pmic, ADC_ACT,           0x00);
	galileo1_write8(i2c_pmic, ADC_RES,           0x00);
	galileo1_write8(i2c_pmic, STATUS,            0x00);

	/* Sensor MSRs */
	galileo1_write8(i2c_sensor, POSBSTSEL,         0x1C);
	galileo1_write8(i2c_sensor, READVDSEL,         0x06);
	galileo1_write8(i2c_sensor, RSTVDSEL,          0x08);
	galileo1_write8(i2c_sensor, BSVBPSEL,          0x18);
	galileo1_write8(i2c_sensor, HREG_TEST,         0x04);
	galileo1_write8(i2c_sensor, DRESET,            0xC8);
	galileo1_write16(i2c_sensor, FRACEXP_TIME1,  0x08FF);
	galileo1_write16(i2c_sensor, PORTGRESET_U,   0x026C);
	galileo1_write8(i2c_sensor, PORTGRESET_W,      0x30);
	galileo1_write8(i2c_sensor, ROREAD,            0xCE);
	galileo1_write8(i2c_sensor, DRCUT,             0x01);
	galileo1_write8(i2c_sensor, GDMOSCNT,          0x2F);
	galileo1_write8(i2c_sensor, CDS_STOPBST,       0x01);
	galileo1_write8(i2c_sensor, BSTCKLFIX_ADC,     0x89);
	galileo1_write8(i2c_sensor, BSC_AGRNG2,        0x30);
	galileo1_write8(i2c_sensor, BSC_AGRNG1,        0x18);
	galileo1_write8(i2c_sensor, BSC_AGRNG0,        0x10);
	galileo1_write8(i2c_sensor, KBIASCNT_RNG32,    0x98);
	galileo1_write8(i2c_sensor, KBIASCNT_RNG10,    0x76);
	galileo1_write8(i2c_sensor, GDMOSHEN,          0x01);
	galileo1_write8(i2c_sensor, BSDIGITAL_MODE,    0x08);
	galileo1_write8(i2c_sensor, PS_VZS_NML_COEF,   0xC7);
	galileo1_write8(i2c_sensor, PS_VZS_NML_INTC,   0x7E);
	galileo1_write8(i2c_sensor, ZSV_IN_LINES,      0x43);
	galileo1_write8(i2c_sensor, FBC_IN_RANGE,      0x10);
	galileo1_write8(i2c_sensor, OB_CLPTHRSH_NEAR,  0x28);
	galileo1_write8(i2c_sensor, OB_CLPTHRSH_FAR,   0x28);
	galileo1_write8(i2c_sensor, WKUP_WAIT_ON,      0xE9);
	galileo1_write8(i2c_sensor, HALF_VTAP_MODE,    0x12);
	galileo1_write8(i2c_sensor, CCP2BLKD,          0xB0);

	/* Sensor static register */
	whole = pdata->refclk / 1000000;
	fract = ((pdata->refclk - (whole * 1000000)) * 0x100) / 1000000;

	galileo1_write8(i2c_sensor, EXTCLK_FRQ_MHZ,     whole);
	galileo1_write8(i2c_sensor, EXTCLK_FRQ_MHZ + 1, fract);

	galileo1_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
			            glbrst_cfg1._register);
	galileo1_write8(i2c_sensor, DPHY_CTRL, 0x01);

	/* Link MBPS seems to influence the bridge, I don't know why, so I let
	 * this value to zero.
	 */
	galileo1_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_31_24, 0x0);
	galileo1_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_23_16, 0x0);
	galileo1_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_15_8,  0x0);
	galileo1_write8(i2c_sensor, REQUESTED_LINK_BIT_RATE_MBPS_7_0,   0x0);

	return 0;
}

static int galileo1_apply_hflip(struct v4l2_subdev *sd)
{
	struct galileo1         *galileo1 = to_galileo1(sd);
	struct i2c_client       *i2c = galileo1->i2c_sensor;
	union image_orientation reg;

	galileo1_read8(i2c, IMAGE_ORIENTATION, &reg._register);
	reg.h_mirror = galileo1->hflip->val;
	galileo1_write8(i2c, IMAGE_ORIENTATION, reg._register);

	return 0;
}

static int galileo1_apply_vflip(struct v4l2_subdev *sd)
{
	struct galileo1         *galileo1 = to_galileo1(sd);
	struct i2c_client       *i2c = galileo1->i2c_sensor;
	union image_orientation reg;

	galileo1_read8(i2c, IMAGE_ORIENTATION, &reg._register);
	reg.v_mirror = galileo1->vflip->val;
	galileo1_write8(i2c, IMAGE_ORIENTATION, reg._register);

	return 0;
}

static int galileo1_apply_nd(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c = galileo1->i2c_pmic;

	galileo1_write8(i2c, ACT_STATE_1, galileo1->nd->val);
	galileo1_write8(i2c, OPERATION_MODE, 0x1);

	/* Drive the ND for 10 ms */
	galileo1_write8(i2c, MECH_SHUTTER_CONTROL, 0x01);
	msleep(20);
	galileo1_write8(i2c, MECH_SHUTTER_CONTROL, 0x00);

	return 0;
}

static int galileo1_drive_shutter(struct v4l2_subdev *sd, u8 direction) {
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c = galileo1->i2c_pmic;

	union operation_mode opmode = {{
		.sdirm_cfg = direction,
	}};

	/* Reset shutter usage */
	galileo1_write8(i2c, MECH_SHUTTER_CONTROL, 0x08);
	galileo1_write8(i2c, OPERATION_MODE, opmode._register);

	/* Drive the shutter for 10 ms */
	galileo1_write8(i2c, MECH_SHUTTER_CONTROL, 0x01);
	msleep(20);
	galileo1_write8(i2c, MECH_SHUTTER_CONTROL, 0x00);

	return 0;
}

static int galileo1_shutter_close(struct v4l2_subdev *sd)
{
	return galileo1_drive_shutter(sd, 0);
}

static int galileo1_shutter_open(struct v4l2_subdev *sd)
{
	return galileo1_drive_shutter(sd, 1);
}

static int galileo1_set_shutter(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1   = to_galileo1(sd);
	union global_reset_mode_config1 glbrst_cfg1;
	struct i2c_client *i2c_sensor = galileo1->i2c_sensor;

	switch (galileo1->ms->val) {
		case MS_STATE_OPEN:
			return galileo1_shutter_open(sd);
		case MS_STATE_CLOSE:
			return galileo1_shutter_close(sd);
		default:
			break;
	}

	galileo1_read8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
		       &glbrst_cfg1._register);
	glbrst_cfg1.sastrobe_muxing = 0;

	if (galileo1->gs->val) {
		struct i2c_client *i2c_pmic = galileo1->i2c_pmic;

		union operation_mode            opmode = {
			/* Shutter strobe & ND Strobe */
			/* Shutter */
			.smode     = 1,  /* Strobe mode */
			.sedge     = 1,  /* Edge mode
					  * rising  edge = close
					  * falling edge = open
					  */
			.sdirm_cfg = 0,
			.nmode     = 1,	 /* Strobe mode */
			.nedge     = 0,
			.ndirm_cfg = 0,
		};

		/* Reset shutter usage */
		galileo1_write8(i2c_pmic, MECH_SHUTTER_CONTROL, 0x08);
		galileo1_write8(i2c_pmic, OPERATION_MODE, opmode._register);
		galileo1_write8(i2c_pmic, MECH_SHUTTER_CONTROL, 0x1);

		glbrst_cfg1.sastrobe_muxing = 1;
	}

	galileo1_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
				    glbrst_cfg1._register);

	return 0;
}

static int galileo1_apply_ms(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1    = to_galileo1(sd);
	struct i2c_client *i2c_sensor  = galileo1->i2c_sensor;
	struct i2c_client *i2c_pmic    = galileo1->i2c_pmic;
	u32               *exposure_us = &galileo1->exposure->val;
	u32                ms_state    = galileo1->ms->val;
	struct v4l2_rect  *c           = &galileo1->crop;
	struct v4l2_rect  *vt          = &galileo1->video_timing;
	u8                *nvm         = galileo1->nvm;
	union nvm_memaddr *nvm_addr    = &galileo1->nvm_addr;
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
	if (!galileo1->gs->val) {
		glbrst_cfg1.sastrobe_muxing = 0;
		glbrst_cfg1.continous_global_reset_mode = 0;

		galileo1_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
				    glbrst_cfg1._register);

		galileo1_write8(galileo1->i2c_sensor,
				GLOBAL_RESET_CTRL1, 0x0);

		if (ms_state != MS_STATE_CLOSE)
			return galileo1_shutter_open(sd);

		return 0;
	}

	galileo1->trdy_ctrl = 0x0034;

	/* This is used to round further timing computations instead of flooring */
	half_line_duration = galileo1->line_duration_ns / 2;
	
	/* Shutter should close after exposure time, but we need to take into
	 * account the shutter speed stored in the NVM */
	sdelay = swab16(*((u16 *)(nvm + nvm_addr->ms)));
	sdelay_ctrl = ((u32)sdelay * 1000 + half_line_duration) / galileo1->line_duration_ns;

	/* Don't begin reading the pixels until we've waited for the exposure
	 * time */
	trdout_ctrl = ((u32)(*exposure_us) * 1000 + half_line_duration) / galileo1->line_duration_ns;

	if (sdelay_ctrl > galileo1->trdy_ctrl + trdout_ctrl)
		galileo1->trdy_ctrl = sdelay_ctrl - trdout_ctrl;

	/* Leave the shutter open for some more time so that it closes when we
	 * start reading the pixels */
	str_delay_ctrl = galileo1->trdy_ctrl + trdout_ctrl - sdelay_ctrl;

	/* Configure timer */
	/* Set Global reset ready to its minimum */
	galileo1_write16(i2c_sensor, TRDY_CTRL, galileo1->trdy_ctrl);

	galileo1_write16(i2c_sensor, TSHUTTER_STROBE_DELAY_CTRL,
				     str_delay_ctrl);

	/* Start readout as soon as possible */
	galileo1_write16(i2c_sensor, TRDOUT_CTRL, trdout_ctrl);

	/* Close the shutter during the readout, thus it should last at least
	 * the number of active line.
	 */
	galileo1_write16(i2c_sensor, TSHUTTER_STROBE_WIDTH_CTRL,
				     c->height / galileo1->y_binning
				     + sdelay_ctrl);

	tgrst_interval_ctrl = vt->height + trdout_ctrl + galileo1->trdy_ctrl
		              + sdelay_ctrl + 512;
	galileo1_write16(i2c_sensor, TGRST_INTERVAL_CTRL, tgrst_interval_ctrl);
	
	galileo1_write8(i2c_sensor, GLOBAL_RESET_MODE_CONFIG1,
				    glbrst_cfg1._register);

	/* Mechanical shutter control */
	galileo1_write8(i2c_sensor, GLOBAL_RESET_CTRL1, 0x1);
	galileo1_write8(i2c_pmic, MECH_SHUTTER_CONTROL, 0x1);

	return 0;
}

static int galileo1_apply_exposure(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct v4l2_fract *fi = &galileo1->frame_interval;
	u32               *exposure_us = &galileo1->exposure->val;
	u16                coarse;

	/* Exposure is expressed in us */
	u32 exposure_max_us = div_u64((u64) fi->numerator * 1000000,
				      fi->denominator);

	if (*exposure_us > exposure_max_us) {
		v4l2_warn(sd, "requested exposure (%d) is higher than exposure max (%d)\n",
			      *exposure_us, exposure_max_us);

		*exposure_us = exposure_max_us;
	}

	coarse = (*exposure_us * 1000) / galileo1->line_duration_ns;

	galileo1_write16(galileo1->i2c_sensor, COARSE_INTEGRATION_TIME, coarse);

	return 0;
}

static int galileo1_apply_gain(struct v4l2_subdev *sd)
{
	struct galileo1 *galileo1 = to_galileo1(sd);
	u32              gain     = galileo1->gain->val;

	galileo1_write16(galileo1->i2c_sensor, ANALOG_GAIN_CODE_GLOBAL, gain);

	return 0;
}

static int galileo1_apply_flash_strobe(struct v4l2_subdev *sd)
{
	struct galileo1                 *galileo1 = to_galileo1(sd);
	union global_reset_mode_config1  glbrst_cfg1;
	struct i2c_client               *i2c = galileo1->i2c_sensor;
	int strobe_en;

	strobe_en = galileo1->strobe_enable->val;

	galileo1_read8(i2c, GLOBAL_RESET_MODE_CONFIG1, &glbrst_cfg1._register);

	if (!strobe_en) {
		glbrst_cfg1.flash_strobe = 0;
		galileo1_write8(i2c,
				GLOBAL_RESET_MODE_CONFIG1,
				glbrst_cfg1._register);
		galileo1_write8(i2c, FLASH_TRIGGER_RS, 0x0);
		return 0;
	}


	/* Set the width to 100us, it is an arbitrary value, but the signal
	 * seems to take at least ~30us to go from 0 to 1
	 */
	if (galileo1->gs->val) {
		/* "Global" shutter mode (photo) */
		glbrst_cfg1.flash_strobe = 1;

		galileo1_write8(i2c,
				GLOBAL_RESET_MODE_CONFIG1,
				glbrst_cfg1._register);

		galileo1_write16(i2c, TFLASH_STROBE_WIDTH_HIGH_CTRL, (galileo1->strobe_width->val
		                 				     * 1000) / 108);

	} else {
		/* Reset the RS trigger to be able
		 * to change the config on the fly
		 */
		galileo1_write8(i2c, FLASH_TRIGGER_RS, 0x0);

		/* Rolling shutter mode (video) */
		galileo1_write16(i2c, TFLASH_STROBE_WIDTH_HIGH_RS_CTRL, (galileo1->strobe_width->val
		                 					* 1000) / 108);
		galileo1_write8(i2c, FLASH_MODE_RS, 0x1);
		galileo1_write8(i2c, FLASH_TRIGGER_RS, 0x1);
	}


	return 0;
}

static int galileo1_get_lens_position(struct v4l2_subdev *sd, u16 *pos)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c = galileo1->i2c_pmic;

	union status status = {
		.busy = 1,
	};

	union focus_change_control fcc = {
		.measpos = 1,
	};

	galileo1_write8(i2c, FOCUS_CHANGE_CONTROL, fcc._register);

	while (status.busy)
		galileo1_read8(i2c, STATUS, &status._register);

	galileo1_read16(i2c, POSITION, pos);

	return 0;
}

static int galileo1_get_temp_sensor_output(struct v4l2_subdev *sd, s8 *output)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c = galileo1->i2c_sensor;

	/*
	 * In NVM there is two values for temp calibration : 25 and 60 C
	 * Look for the shift to apply for getting the real value
	 */
	s8 shift_temp_h = galileo1->nvm_temp_h - TEMP_SENSOR_OUTPUT_HIGH_CALIB;
	s8 shift_temp_l = galileo1->nvm_temp_l - TEMP_SENSOR_OUTPUT_LOW_CALIB;

	galileo1_read8(i2c, TEMP_SENSOR_OUTPUT, output);

	/*
	 * Look for the best shift according average temperature (42) :
	 * Take the best shift close the calibrated value
	 */

	if ( (*output - shift_temp_h) > TEMP_SENSOR_OUTPUT_AVERAGE_CALIB)
		*output -= shift_temp_h;
	else
		*output -= shift_temp_l;

	if (*output < TEMP_SENSOR_OUTPUT_MIN)
		*output = TEMP_SENSOR_OUTPUT_MIN;

	if (*output > TEMP_SENSOR_OUTPUT_MAX)
		*output = TEMP_SENSOR_OUTPUT_MAX;

	return 0;
}


static int galileo1_apply_focus(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c = galileo1->i2c_pmic;
	u8                 trial = 0;
	u16                cur_pos;
	u16                tgt_pos = galileo1->focus->val;

	galileo1_get_lens_position(sd, &cur_pos);

	/* We try to reach the tgt_pos with +/- 2 tolerance in 3 trials max */
	while ((cur_pos < tgt_pos - 2 || cur_pos > tgt_pos + 2) && trial < 3) {
		union focus_change_control fcc = { ._register = 0 };
		union status               status;
		s16                        step;

		fcc.enable  = 1;
		fcc.dir     = (tgt_pos < cur_pos);
		fcc.measpos = 1;

		step = 30 * (tgt_pos - cur_pos);
		if (step < 0)
			step = -step;

		galileo1_write16(i2c, FOCUS_CHANGE, step);
		galileo1_write8(i2c, FOCUS_CHANGE_CONTROL, fcc._register);

		status.busy = 1;
		while (status.busy)
			galileo1_read8(i2c, STATUS, &status._register);

		galileo1_read16(i2c, POSITION, &cur_pos);
		trial++;

		/* Avoid locking up the focus mechanics */
		udelay(750);
	}

	galileo1->focus->val     = cur_pos;
	galileo1->focus->cur.val = cur_pos;

	return 0;
}

/* Manually synchronize control values, I'm not sure if it is the right way to
 * do it...
 */
static inline void galileo1_synchronize_ctrl(struct v4l2_ctrl *ctrl)
{
	v4l2_ctrl_lock(ctrl);
	ctrl->cur.val = ctrl->val;
	v4l2_ctrl_unlock(ctrl);
}

static int galileo1_configure(struct v4l2_subdev *sd)
{
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = galileo1->pdata;
	struct i2c_client             *i2c = galileo1->i2c_sensor;
	struct v4l2_rect              *vt  = &galileo1->video_timing;
	struct v4l2_rect              *c   = &galileo1->crop;
	struct v4l2_mbus_framefmt     *fmt = &galileo1->format;
	int                            ret;

	ret = galileo1_init(sd);
	if (ret < 0) {
		v4l2_err(sd, "init failed\n");
		return ret;
	}

	/* CSI2 mode */
	galileo1_write8(i2c, CSI_SIGNALING_MODE, 0x2);

	/* Pixel format */
	galileo1_write8(i2c, CSI_DATA_FORMAT, galileo1->bits_per_pixel << 8 |
					      galileo1->bits_per_pixel);
	galileo1_write8(i2c, CSI_LANE_MODE, pdata->lanes - 1);

	/* Image Size */
	galileo1_write16(i2c, X_OUTPUT_SIZE, fmt->width);
	galileo1_write16(i2c, Y_OUTPUT_SIZE, fmt->height);

	/* Image Scaling */
	/* Full scaling, Bayer sampling */
	galileo1_write16(i2c, SCALING_MODE,     0x0002);
	galileo1_write16(i2c, SPATIAL_SAMPLING, 0x0000);

	/* Scaler */
	galileo1_write16(i2c, OUTPUT_IMAGE_WIDTH,  fmt->width);
	galileo1_write16(i2c, OUTPUT_IMAGE_HEIGHT, fmt->height);
	galileo1_write16(i2c, SCALER_BLANKING_PCK, 0x26EC);

	/* Frame Timing */
	galileo1_write16(i2c, VT_LINE_LENGTH_PCK,    vt->width);
	galileo1_write16(i2c, VT_FRAME_LENGTH_LINES, vt->height);

	/* Image area */
	galileo1_write16(i2c, X_ADDR_START, c->left);
	galileo1_write16(i2c, Y_ADDR_START, c->top);
	galileo1_write16(i2c, X_ADDR_END,   c->left + c->width  - 1);
	galileo1_write16(i2c, Y_ADDR_END,   c->top  + c->height - 1);

	/* Digital Crop: We do not crop before the scaler */
	galileo1_write16(i2c, DIGITAL_CROP_X_OFFSET, 0x0000);
	galileo1_write16(i2c, DIGITAL_CROP_Y_OFFSET, 0x0000);
	galileo1_write16(i2c, DIGITAL_CROP_IMAGE_WIDTH,
			      c->width / galileo1->x_binning);
	galileo1_write16(i2c, DIGITAL_CROP_IMAGE_HEIGHT,
			      c->height / galileo1->y_binning);

	/* Binning */
	galileo1_write8(i2c, BINNING_MODE, 0x1);
	galileo1_write8(i2c, BINNING_TYPE, galileo1->x_binning << 4 |
					   galileo1->y_binning);

	/* TOSHIBA register setting
	 * Scaler */
	galileo1_write8(i2c, GREEN_AVERAGED_BAYER,       0x00);
	galileo1_write8(i2c, HORIZONTAL_DIGITAL_BINNING, 0x00);
	galileo1_write8(i2c, VERTICAL_DIGITAL_BINNING,   0x00);

	/* Row Noise improve setting */
	galileo1_write8(i2c, BLC_SEL, 0x01);
	galileo1_write16(i2c, CSI2_DELAY, 0x0000);

	/* DPC */
	galileo1_write8(i2c, SINGLE_DEFECT_CORRECT_ENABLE, 0x00);
	galileo1_write8(i2c, COMBINED_COUPLET_SINGLE_DEFECT_CORRECT_ENABLE,
			     0x01);

	/* Controls */
	galileo1_apply_exposure(sd);
	galileo1_apply_gain(sd);
	galileo1_apply_focus(sd);

	/* Synchronize control values */
	galileo1_synchronize_ctrl(galileo1->exposure);
	galileo1_synchronize_ctrl(galileo1->focus);

	return 0;
}

static int galileo1_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = galileo1->pdata;
	int                            ret;

	if (enable == 0) {
		/* Nothing to do if we are already off */
		if (galileo1->streaming == 0)
			return 0;

		galileo1->streaming = 0;

		if (galileo1->gs->val) {
			galileo1_write8(galileo1->i2c_sensor,
					GLOBAL_RESET_CTRL1, 0x0);
			galileo1_write8(galileo1->i2c_pmic,
					MECH_SHUTTER_CONTROL, 0);
		}

		galileo1_write8(galileo1->i2c_sensor, MODE_SELECT, 0x00);

		if (pdata->set_power)
			pdata->set_power(GALILEO1_POWER_OFF);

		return 0;
	}

	if (!galileo1->timings_uptodate) {
		ret = galileo1_update_timings(sd);
		if (ret < 0) {
			v4l2_err(sd, "Unable to calculate Video Timing\n");
			return ret;
		}
	}

	/* Now that all needed pre-calculations are done, we can power on the
	 * device and configure it.
	 */
	if (pdata->set_power) {
		ret = pdata->set_power(GALILEO1_POWER_ON);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	ret = galileo1_apply_plls(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to apply plls\n");
		goto power_off;
	}

	ret = galileo1_configure(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to configure\n");
		goto power_off;
	}

	/* Stream on */
	galileo1->streaming = 1;
	galileo1_set_shutter(sd);
	galileo1_apply_hflip(sd);
	galileo1_apply_vflip(sd);
	galileo1_apply_ms(sd);
	galileo1_apply_nd(sd);
	galileo1_apply_flash_strobe(sd);
	galileo1_write8(galileo1->i2c_sensor, MODE_SELECT, 0x01);

	/* Get frame format for debug */
	if (debug_print > 0)
		galileo1_print_frame_format(sd);

	return 0;

power_off:
	if (pdata->set_power)
		pdata->set_power(GALILEO1_POWER_OFF);

	return ret;
}

static int galileo1_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct galileo1 *galileo1 = to_galileo1(sd);

	memset(fi, 0, sizeof(*fi));
	fi->interval = galileo1->frame_interval;

	return 0;
}

static int galileo1_s_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c      = galileo1->i2c_sensor;
	struct v4l2_rect  *vt       = &galileo1->video_timing;
	struct v4l2_rect  *c        = &galileo1->crop;
	struct v4l2_fract *cur_fi   = &galileo1->frame_interval;
	u32                min_vt_height;

	*cur_fi = fi->interval;

	galileo1->timings_uptodate = 0;

	if (!galileo1->streaming)
		return 0;

	/* We are already streaming, so we try to adjust the vertical blanking
	 * in order to match the frame rate.
	 */
	vt->height = div_u64((u64) galileo1->vtclk * cur_fi->numerator,
		             vt->width * cur_fi->denominator);

	/* In case min_vt_frame_blanking is not met, we adjust the frame rate */
	min_vt_height = c->height / galileo1->y_binning + 42;

	if (vt->height < min_vt_height) {
		vt->height = min_vt_height;
		/* Refresh FPS */
		cur_fi->denominator = galileo1->vtclk;
		cur_fi->numerator   = vt->width * vt->height;
	}

	galileo1_write16(i2c, VT_FRAME_LENGTH_LINES, vt->height);

	return 0;
}

static int galileo1_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = galileo1->pdata;
	struct v4l2_mbus_framefmt     *fmt = &galileo1->format;
	struct v4l2_bt_timings        *bt = &timings->bt;
	int                            ret;

	memset(timings, 0, sizeof(*timings));

	/* We update the timing only when we are not streaming. Anyway, while
	 * streaming, it is forbidden to change the pixelcloclk.
	 */
	if (!galileo1->timings_uptodate && !galileo1->streaming) {
		ret = galileo1_update_timings(sd);
		if (ret < 0) {
			v4l2_err(sd, "Unable to calculate Video Timing\n");
			return ret;
		}
	}

	bt->width      = fmt->width;
	bt->height     = fmt->height;
	bt->pixelclock = (galileo1->mipiclk * pdata->lanes * 2) /
	        	 galileo1->bits_per_pixel;

	/* Consider HSYNC and VSYNC as HACTIVE and VACTIVE*/
	bt->polarities = 0;

	/* Because we are in HACTIVE/VACTIVE mode, the blanking size does not
	 * matter for the capture device.
	 */

	return 0;
}

static const struct v4l2_subdev_video_ops galileo1_video_ops = {
	.s_stream         = galileo1_s_stream,
	.g_frame_interval = galileo1_g_frame_interval,
	.s_frame_interval = galileo1_s_frame_interval,
	.g_dv_timings     = galileo1_g_dv_timings,
};

static const struct v4l2_subdev_ops galileo1_ops = {
	.core  = &galileo1_core_ops,
	.video = &galileo1_video_ops,
	.pad   = &galileo1_pad_ops,
};

/* Custom ctrls */
#define V4L2_CID_GALILEO1_ND                 (V4L2_CID_CAMERA_CLASS_BASE + 0x100)
#define V4L2_CID_GALILEO1_GS                 (V4L2_CID_CAMERA_CLASS_BASE + 0x101)
#define V4L2_CID_GALILEO1_STROBE_WIDTH       (V4L2_CID_CAMERA_CLASS_BASE + 0x102)
#define V4L2_CID_GALILEO1_MS                 (V4L2_CID_CAMERA_CLASS_BASE + 0x103)
#define V4L2_CID_GALILEO1_FSTROBE_START      (V4L2_CID_CAMERA_CLASS_BASE + 0x104)
#define V4L2_CID_GALILEO1_FSTROBE_ENABLE     (V4L2_CID_CAMERA_CLASS_BASE + 0x105)
#define V4L2_CID_GALILEO1_TEMP_SENSOR_OUTPUT (V4L2_CID_CAMERA_CLASS_BASE + 0x106)

static int galileo1_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct galileo1 *galileo1 = ctrl_to_galileo1(ctrl);

	/* If not streaming, just default value */
	if (!galileo1->streaming) {
		ctrl->val = ctrl->default_value;
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_FOCUS_ABSOLUTE:
		galileo1_get_lens_position(&galileo1->sd, (u16 *)&ctrl->val);
		break;
	case V4L2_CID_GALILEO1_TEMP_SENSOR_OUTPUT:
		galileo1_get_temp_sensor_output(&galileo1->sd, (s8 *)&ctrl->val);
		break;
	}

	return 0;
}

static int galileo1_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct galileo1 *galileo1 = ctrl_to_galileo1(ctrl);

	/* If not streaming, just keep interval structures up-to-date */
	if (!galileo1->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		return galileo1_apply_hflip(&galileo1->sd);
	case V4L2_CID_VFLIP:
		return galileo1_apply_vflip(&galileo1->sd);
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		return galileo1_apply_exposure(&galileo1->sd);
	case V4L2_CID_FOCUS_ABSOLUTE:
		return galileo1_apply_focus(&galileo1->sd);
	case V4L2_CID_GALILEO1_ND:
		return galileo1_apply_nd(&galileo1->sd);
	case V4L2_CID_GALILEO1_STROBE_WIDTH:
	case V4L2_CID_GALILEO1_FSTROBE_START:
	case V4L2_CID_GALILEO1_FSTROBE_ENABLE:
		return galileo1_apply_flash_strobe(&galileo1->sd);
	case V4L2_CID_ANALOGUE_GAIN:
		return galileo1_apply_gain(&galileo1->sd);
	case V4L2_CID_GALILEO1_GS:
		return galileo1_apply_ms(&galileo1->sd);
	case V4L2_CID_GALILEO1_MS:
		return galileo1_set_shutter(&galileo1->sd);
	}

	return 0;
}

static const struct v4l2_ctrl_ops galileo1_ctrl_ops = {
	.g_volatile_ctrl = galileo1_g_volatile_ctrl,
	.s_ctrl          = galileo1_s_ctrl,
};

static const struct v4l2_ctrl_config galileo1_ctrl_nd = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_ND,
	.name = "Neutral Density Filter",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = false,
};

static const struct v4l2_ctrl_config galileo1_ctrl_gs = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_GS,
	.name = "Global Shutter",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min  = false,
	.max  = true,
	.step = 1,
	.def  = false,
};

static const struct v4l2_ctrl_config galileo1_ctrl_ms = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_MS,
	.name = "Mechanical Shutter",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 2,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config galileo1_ctrl_sw = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_STROBE_WIDTH,
	.name = "Flash strobe width, in us",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 1,
	.max  = 7000,
	.step = 1,
	.def  = 100,
};

static const struct v4l2_ctrl_config galileo1_ctrl_ss = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_FSTROBE_START,
	.name = "Flash strobe start delay, in us",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 50000,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config galileo1_ctrl_se = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_FSTROBE_ENABLE,
	.name = "Flash strobe pulse enable state",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 0,
	.max  = 1,
	.step = 1,
	.def  = 0,
};

static const struct v4l2_ctrl_config galileo1_ctrl_tso = {
	.ops  = &galileo1_ctrl_ops,
	.id   = V4L2_CID_GALILEO1_TEMP_SENSOR_OUTPUT,
	.name = "Temperature sensor output",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = TEMP_SENSOR_OUTPUT_MIN,
	.max  = TEMP_SENSOR_OUTPUT_MAX,
	.step = 1,
	.def  = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static int galileo1_initialize_controls(struct v4l2_subdev *sd)
{
	struct galileo1          *galileo1 = to_galileo1(sd);
	struct v4l2_ctrl_handler *hdl      = &galileo1->ctrl_handler;
	u8                       *nvm      = galileo1->nvm;
	union nvm_memaddr        *nvm_addr = &galileo1->nvm_addr;
	union nvm_af              nvm_af;
	int                       ret;

	ret = v4l2_ctrl_handler_init(hdl, 16);
	if (ret < 0) {
		v4l2_err(sd, "failed to init ctrl handler\n");
		goto einit;
	}

	/* Flips */
	galileo1->hflip = v4l2_ctrl_new_std(hdl,
					    &galileo1_ctrl_ops,
					    V4L2_CID_HFLIP,
					    0, 1, 1, 0);

	galileo1->vflip = v4l2_ctrl_new_std(hdl,
					    &galileo1_ctrl_ops,
					    V4L2_CID_VFLIP,
					    0, 1, 1, 0);

	/* Exposure in us */
	galileo1->exposure = v4l2_ctrl_new_std(hdl,
					       &galileo1_ctrl_ops,
					       V4L2_CID_EXPOSURE_ABSOLUTE,
					       0, 1000000, 1, 20000);

	/* Focus */
	nvm_af._registers =
		swab64(*((u64 *)(nvm + nvm_addr->af + NVM_AF_FAR_END)));

	/* Format the Auto Focus registers */
	nvm_af.infinity += nvm_af.far_end;
	nvm_af.macro    += nvm_af.infinity;
	nvm_af.near_end += nvm_af.macro;

	galileo1->focus = v4l2_ctrl_new_std(hdl, &galileo1_ctrl_ops,
					    V4L2_CID_FOCUS_ABSOLUTE,
					    nvm_af.far_end,
					    nvm_af.near_end,
					    1,
					    nvm_af.infinity);

	/* Since the lens can move even if no command has been sent, flag the
	 * control as volatile.
	 */
	galileo1->focus->flags |= V4L2_CTRL_FLAG_VOLATILE;

	/* Neutral Density Filter */
	galileo1->nd = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_nd, NULL);

	/* Global Shutter */
	galileo1->gs = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_gs, NULL);

	/* Mechanical shutter control */
	galileo1->ms = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_ms, NULL);

	/* Flash strobe width */
	galileo1->strobe_width = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_sw, NULL);

	/* Flash strobe start */
	galileo1->strobe_start = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_ss, NULL);

	/* Flash strobe enable */
	galileo1->strobe_enable = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_se, NULL);

	/* Analog Gain
	 * AGx1.0  = 0x003F
	 * AGx2.0  = 0x007E
	 * AGx8.0  = 0x01F8
	 * AGx12.0 = 0x02F4, this is the max gain code
	 */
	galileo1->gain = v4l2_ctrl_new_std(hdl,
					   &galileo1_ctrl_ops,
					   V4L2_CID_ANALOGUE_GAIN,
					   0, 0x2F4, 1, 5 * 0x3F);

	/* Temperature sensor output */
	galileo1->temp_sensor_output = v4l2_ctrl_new_custom(hdl, &galileo1_ctrl_tso, NULL);

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

static void galileo1_free_controls(struct v4l2_subdev *sd)
{
	v4l2_ctrl_handler_free(sd->ctrl_handler);
}

static int galileo1_detect_chip(struct v4l2_subdev *sd)
{
	struct galileo1 *galileo1 = to_galileo1(sd);
	u16              chip_id;

	galileo1_read16(galileo1->i2c_sensor, SENSOR_MODEL_ID, &chip_id);

	if (chip_id != GALILEO1_CHIPID) {
		v4l2_err(sd, "Error Chipd ID = 0x%04x instead of 0x%04x\n",
			     chip_id, GALILEO1_CHIPID);
		return -ENODEV;
	}

	galileo1_read16(galileo1->i2c_pmic, IC_INFO, &chip_id);

	if (chip_id != PMIC_CHIPID) {
		v4l2_err(sd, "Error Chipd ID = 0x%04x instead of 0x%04x\n",
			     chip_id, PMIC_CHIPID);
		return -ENODEV;
	}

	v4l2_info(sd, "Found " DRIVER_NAME " chip\n");

	return 0;
}

static int galileo1_read_nvm(struct v4l2_subdev *sd)
{
	struct galileo1   *galileo1 = to_galileo1(sd);
	struct i2c_client *i2c = galileo1->i2c_sensor;
	u8                *nvm = galileo1->nvm;
	u8                 page = 0;
	int                ret = 0;

	/* Enable Read */
	galileo1_write8(i2c, DATA_TRANSFER_IF_1_CTRL, 0x1);

	for (page = 0 ; page < NVM_PAGE_NB ; page++) {
		unsigned int i;

		union data_transfer_if_1_status status = {
			.read_if_ready = 0,
		};

		/* Select page */
		galileo1_write8(i2c, DATA_TRANSFER_IF_1_PAGE_SELECT, page);

		/* Check Status */
		while (!status.read_if_ready) {
			galileo1_read8(i2c, DATA_TRANSFER_IF_1_STATUS,
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
			galileo1_read8(i2c, DATA_TRANSFER_IF_1_DATA + i,
					nvm + NVM_PAGE_SZ * page +  i);
	}

out:
	galileo1_write8(i2c, DATA_TRANSFER_IF_1_CTRL, 0x0);

	/* Check Version */
	if (*nvm != NVM_VERSION) {
		v4l2_err(sd, "NVM Version (0x%02x) is not correct\n", *nvm);
		v4l2_err(sd, "Expecting 0x%02x\n", NVM_VERSION);
		ret = -ENODEV;
	}

	return ret;
}

static ssize_t galileo1_nvm_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct galileo1    *galileo1 = to_galileo1(sd);

	memcpy(buf, galileo1->nvm, NVM_SIZE);

	return NVM_SIZE;
}

DEVICE_ATTR(nvm, S_IRUGO, galileo1_nvm_show, NULL);

static int galileo1_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct galileo1               *galileo1;
	struct galileo1_platform_data *pdata = client->dev.platform_data;
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

	galileo1 = kzalloc(sizeof(*galileo1), GFP_KERNEL);
	if (!galileo1) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	galileo1->pdata = pdata;

	sd = &galileo1->sd;
	v4l2_i2c_subdev_init(sd, client, &galileo1_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	galileo1->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &galileo1->pad, 0);

	if (ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto emedia;
	}

	galileo1->i2c_sensor = client;
	galileo1->i2c_pmic   = i2c_new_dummy(client->adapter,
					     GALILEO1_PMIC_I2C_ADDR);
	if (galileo1->i2c_pmic == NULL) {
		v4l2_err(sd, "failed to register pmic i2c client\n");
		ret = -ENODEV;
		goto epmic;
	}

	/* Set default configuration:
	 *   Max sensor crop into 720p30
	 */
	galileo1->format.width  = 1280;
	galileo1->format.height =  720;
	galileo1->format.code   = V4L2_MBUS_FMT_SGBRG10_1X10;

	/* Center the crop */
	galileo1->crop.width    = 7680;
	galileo1->crop.height   = 4320;
	galileo1->crop.left     =   24;
	galileo1->crop.top      =  524;

	/* 30 FPS */
	galileo1->frame_interval.numerator   =  1;
	galileo1->frame_interval.denominator = 30;

	/* Make sure all clocks info are up-to-date */
	ret = galileo1_update_timings(sd);
	if (ret < 0) {
		v4l2_err(sd, "Unable to calculate Video Timing\n");
		goto eupdate;
	}

	if (pdata->set_power) {
		ret = pdata->set_power(GALILEO1_POWER_ON);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	/* Check if the chip is preset */
	ret = galileo1_detect_chip(sd);
	if (ret < 0)
		goto edetect;

	/* Make sure the shutter is closed */
	galileo1_shutter_close(sd);

	/* Non-Volatile Memory */
	ret = device_create_file(&client->dev, &dev_attr_nvm);
	if (ret < 0) {
		v4l2_err(sd, "Sysfs nvm entry creation failed\n");
		goto esysfs;
	}

	galileo1->nvm = kzalloc(NVM_SIZE, GFP_KERNEL);
	if (!galileo1->nvm) {
		v4l2_err(sd, "alloc failed for NVM structure\n");
		ret = -ENOMEM;
		goto enomem;
	}

	ret = galileo1_read_nvm(sd);
	if (ret < 0) {
		v4l2_err(sd, "Failed to read NVM\n");
		goto envm;
	}

	/* Extract NVM Memory map */
	galileo1->nvm_addr._registers =
		swab64(*(u64 *)(galileo1->nvm + NVM_MEMORY_ADDRESS));

	if (pdata->set_power)
		pdata->set_power(GALILEO1_POWER_OFF);

	/* Get temperature sensor calibration */
	galileo1->nvm_temp_h = *(galileo1->nvm + 0x0D); /* 60 degrees C */
	galileo1->nvm_temp_l = *(galileo1->nvm + 0x0E); /* 25 degrees C*/

	/* Initialize Control */
	ret = galileo1_initialize_controls(sd);
	if (ret < 0)
		goto einitctrl;

	return 0;

einitctrl:
envm:
	kfree(galileo1->nvm);
enomem:
	device_remove_file(&client->dev, &dev_attr_nvm);
esysfs:
edetect:
	if (pdata->set_power)
		pdata->set_power(GALILEO1_POWER_OFF);
eupdate:
	i2c_unregister_device(galileo1->i2c_pmic);
epmic:
emedia:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(galileo1);

	return ret;
}

static int galileo1_remove(struct i2c_client *client)
{
	struct v4l2_subdev            *sd = i2c_get_clientdata(client);
	struct galileo1               *galileo1 = to_galileo1(sd);
	struct galileo1_platform_data *pdata = client->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	if (pdata->set_power)
		pdata->set_power(GALILEO1_POWER_OFF);

	if (galileo1->i2c_pmic)
		i2c_unregister_device(galileo1->i2c_pmic);

	device_remove_file(&client->dev, &dev_attr_nvm);
	galileo1_free_controls(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(galileo1->nvm);
	kfree(galileo1);

	return 0;
}

static const struct i2c_device_id galileo1_id[] = {
	{DRIVER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, galileo1_id);

static struct i2c_driver galileo1_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe    = galileo1_probe,
	.remove   = galileo1_remove,
	.id_table = galileo1_id,
};

module_i2c_driver(galileo1_driver);
