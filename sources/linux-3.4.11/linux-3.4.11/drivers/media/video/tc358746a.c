/*
 * tc358746a - Toshiba device TC358746A CSI2.0 <-> Parallel bridge
 *
 * It can perform protocol conversion in both direction, MIPI to parallel or
 * parallel to MIPI.
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

#include <media/tc358746a.h>

#include "tc358746a_reg.h"

MODULE_AUTHOR("Eng-Hong SRON <eng-hong.sron@parrot.com>");
MODULE_DESCRIPTION("Toshiba TC358746A driver");
MODULE_LICENSE("GPL");

#define DRIVER_NAME "tc358746a"

enum {
	TC358746A_IN_PAD = 0,
	TC358746A_OUT_PAD,
	TC358746A_NUM_PADS,
};

struct tc358746a {
	struct v4l2_subdev              sd;
	struct media_pad                pad[TC358746A_NUM_PADS];
	struct tc358746a_platform_data *pdata;
	unsigned long                   pclk;
};

static inline struct tc358746a *to_tc358746a(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358746a, sd);
}

static int tc358746a_read(struct v4l2_subdev *sd, u16 reg, u16 *val)
{
	int			 ret;
	struct i2c_client	*client = v4l2_get_subdevdata(sd);
	u8			 regbuf[2];
	u8			 valbuf[2];

	struct i2c_msg msg[] = {
		[0] = {
			.addr  = client->addr,
			.flags = 0,
			.len   = sizeof(regbuf),
			.buf   = regbuf,
		},
		[1] = {
			.addr  = client->addr,
			.flags = I2C_M_RD,
			.len   = sizeof(valbuf),
			.buf   = valbuf,
		},
	};

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
		return ret;
	}

	*val = (((u16)valbuf[0]) << 8)
		| (u16)valbuf[1];

	return 0;
}

static int tc358746a_write(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	int            ret;
	struct i2c_msg msg;
	u8             buf[4];

	struct i2c_client *client = v4l2_get_subdevdata(sd);

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 4;
	msg.buf   = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static struct v4l2_subdev *tc358746a_get_remote_sd(struct v4l2_subdev *sd)
{
	struct tc358746a   *tc358746a = to_tc358746a(sd);
	struct media_pad   *r_pad;
	struct v4l2_subdev *r_sd;

	r_pad = media_entity_remote_source(&tc358746a->pad[TC358746A_IN_PAD]);

	if (r_pad == NULL ||
	    media_entity_type(r_pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV) {
		v4l2_err(sd, "Nothing connected to input pad\n");
		return NULL;
	}

	r_sd = media_entity_to_v4l2_subdev(r_pad->entity);

	if (!r_sd) {
		v4l2_err(sd, "No connected subdev found\n");
		return NULL;
	}

	return r_sd;
}

/* Since the bridge cannot really change the stream, we simply pass the command
 * to the remote subdev no matter what is the requested pad.
 */
#define TC358746_MK_PAD_OP(_op, _type)                                    \
	static int tc358746a_ ## _op(struct v4l2_subdev    *sd,           \
				     struct v4l2_subdev_fh *fh,           \
				     struct _type          *s)            \
	{                                                                 \
		struct v4l2_subdev *remote = tc358746a_get_remote_sd(sd); \
		if (!remote)                                              \
			return -ENODEV;                                   \
                                                                          \
		return v4l2_subdev_call(remote, pad, _op, fh, s);         \
	}

static int tc358746a_get_fmt(struct v4l2_subdev        *sd,
			     struct v4l2_subdev_fh     *fh,
			     struct v4l2_subdev_format *s)
{
	struct v4l2_subdev	*remote = tc358746a_get_remote_sd(sd);
	struct tc358746a        *tc358746a = to_tc358746a(sd);
	enum v4l2_mbus_pixelcode pixcode;
	int			 ret;

	if (!remote)
		return -ENODEV;

	ret = v4l2_subdev_call(remote, pad, get_fmt, fh, s);
	if (ret)
		return ret;

	pixcode = tc358746a->pdata->force_subdev_pixcode;

	if (pixcode) {
		s->format.code = pixcode;
	}

	return 0;
}

static int tc358746a_enum_mbus_code(struct v4l2_subdev                *sd,
				    struct v4l2_subdev_fh             *fh,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct v4l2_subdev	*remote = tc358746a_get_remote_sd(sd);
	struct tc358746a        *tc358746a = to_tc358746a(sd);
	enum v4l2_mbus_pixelcode pixcode;
	int			 ret;

	if (!remote)
		return -ENODEV;

	ret = v4l2_subdev_call(remote, pad, enum_mbus_code, fh, code);
	if (ret)
		return ret;

	pixcode = tc358746a->pdata->force_subdev_pixcode;

	if (pixcode) {
		code->code = pixcode;
	}

	return 0;
}

TC358746_MK_PAD_OP(set_fmt,        v4l2_subdev_format)

static const struct v4l2_subdev_pad_ops tc358746a_pad_ops = {
	.get_fmt        = tc358746a_get_fmt,
	.set_fmt        = tc358746a_set_fmt,
	.enum_mbus_code = tc358746a_enum_mbus_code,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int tc358746a_get_register(struct v4l2_subdev *sd,
				  struct v4l2_dbg_register *reg)
{
	int ret;
	u16 val;

	reg->size = 2;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	ret = tc358746a_read(sd, reg->reg, &val);
	if (ret)
		return ret;

	reg->val = (__u64)val;

	return 0;
}

static int tc358746a_set_register(struct v4l2_subdev *sd,
				  struct v4l2_dbg_register *reg)
{
	if (reg->reg & ~0xffff || reg->val & ~0xffff)
		return -EINVAL;

	return tc358746a_write(sd, reg->reg, reg->val);
}
#endif

static const struct v4l2_subdev_core_ops tc358746a_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358746a_get_register,
	.s_register = tc358746a_set_register,
#endif
};

static int tc358746a_set_pclk(struct v4l2_subdev *sd)
{
	struct tc358746a               *tc358746a = to_tc358746a(sd);
	struct tc358746a_platform_data *pdata = tc358746a->pdata;

	unsigned int   div[] = {8, 4, 2};
	unsigned long  pll_clk, ppi_clk;
	uint16_t       frs, fbd, prd;
	unsigned long  best_error = -1;
	int            s, p;
	union clkctl   clkctl;
	union pllctl0  pllctl0;
	union pllctl1  pllctl1 = {{
		.pll_en   = 1,
		.resetb   = 1,
		.cken     = 1,
		.bypcken  = 0,
		.lfbren   = 0,
		.pll_lbws = 0,
		.pll_frs  = 0,
	}};

#define IS_BETWEEN(_f, _min, _max) ((_f >= _min) && (_f <= _max))

	/* Check Ref frequency */
	if (!IS_BETWEEN(pdata->refclk, TC358746A_MIN_REFCLK,
			 	       TC358746A_MAX_REFCLK)) {
		v4l2_err(sd, "refclk (%lu) is out of range [%lu - %lu]\n",
			     pdata->refclk,
			     TC358746A_MIN_REFCLK,
			     TC358746A_MAX_REFCLK);
		return -1;
	}

	/* Check PCLK frequency */
	if (!IS_BETWEEN(tc358746a->pclk, TC358746A_MIN_PCLK,
					 TC358746A_MAX_PCLK)) {
		v4l2_err(sd, "pclk (%lu) is out of range [%lu - %lu]\n",
				tc358746a->pclk,
				TC358746A_MIN_PCLK,
				TC358746A_MAX_PCLK);
		return -1;
	}

	/* PPIRxClk is used to in CSIRx for detecting CSI LP-to-HS transition,
	 * so we need to maximize its frequency.
	 */
	for (p = 2 ; p >= 0 ; p--) {
		for (s = 0 ; s < 3 ; s++) {
			pll_clk = tc358746a->pclk * div[s];
			ppi_clk = pll_clk / div[p];

			if (IS_BETWEEN(ppi_clk, TC358746A_MIN_PPICLK,
						TC358746A_MAX_PPICLK)) {
				goto ppi_ok;
			}
		}
	}

#undef IS_BETWEEN

	v4l2_err(sd, "Cannot find PPI_CLK for PCLK at %lu\n", tc358746a->pclk);
	v4l2_err(sd, "PPI_CLK must be between %lu and %lu\n",
		     TC358746A_MIN_PPICLK, TC358746A_MAX_PPICLK);

	return -1;

ppi_ok:

	clkctl.sclkdiv   = s;
	clkctl.ppiclkdiv = p;

	/* Program CSI PLL
	 *   pll_clk = RefClk * [(FBD + 1)/ (PRD + 1)] * [1 / (2^FRS)]
	 */
	for (frs = 0 ; frs < 4 ; frs++) {
		for (prd = 0 ; prd < 16 ; prd++) {
			for (fbd = 0 ; fbd < 512 ; fbd++) {
				unsigned long pll_clk_test;
				unsigned long error;

				pll_clk_test = pdata->refclk * (fbd + 1) /
					       (prd + 1) / (1 << frs);

				error = pll_clk - pll_clk_test;

				if (error < 0)
					error = -error;

				/* Save the configuration */
				if (error < best_error || best_error < 0) {
					best_error      = error;
					pllctl0.pll_fbd = fbd;
					pllctl0.pll_prd = prd;
					pllctl1.pll_frs = frs;
				}
			}
		}
	}

	tc358746a_write(sd, PLLCTL0, pllctl0._register);
	tc358746a_write(sd, PLLCTL1, pllctl1._register);
	tc358746a_write(sd, CLKCTL,  clkctl._register);

	/* Update pclk */
	tc358746a->pclk = pdata->refclk * (pllctl0.pll_fbd + 1) /
			  (pllctl0.pll_prd + 1) /
			  (1 << pllctl1.pll_frs) /
			  div[clkctl.sclkdiv];

	/* MCLK configuration */
	tc358746a_write(sd, MCLKCTL, 0x0201);

	return 0;
}

/* Loop through all delay value in order to find the working window. And then
 * set the delay value at its middle
 */
static int tc358746a_calibrate(struct v4l2_subdev *sd)
{
	struct tc358746a        *tc358746a = to_tc358746a(sd);
	int			 first_dly = 0;
	int			 state	   = 0;
	unsigned		 error_wait;
	unsigned		 dly;

	union phytimdly phytimdly = {
		.dsettle     = 0,
		.td_term_sel = 1,
		.tc_term_sel = 1,
	};


	dly = tc358746a->pdata->phytimdly;
	if (dly != 0) {
		/* Use provided delay value */
		phytimdly.dsettle = dly;
		tc358746a_write(sd, PHYTIMDLY, phytimdly._register);
		return 0;
	}

	error_wait = tc358746a->pdata->calibration_delay_ms;
	if (error_wait == 0) {
		/* Default to 30ms */
		error_wait = 30;
	}

	for (dly = 0 ; dly <= 0x7f ; dly++) {
		union csistatus csistatus;
		union physta    physta;
		int             error;

		phytimdly.dsettle = dly;

		tc358746a_write(sd, PHYTIMDLY, phytimdly._register);

		/* Clear previous error measure */
		tc358746a_write(sd, CSISTATUS, 0xffff);
		msleep(error_wait); /* In order to receive some packets */

		/* Read Status */
		tc358746a_read(sd, CSISTATUS, &csistatus._register);
		tc358746a_read(sd, PHYSTA, &physta._register);

		/*printk("BRIDGE CALIBRATION 0x62: %04x 0x64: %04x\n",
		       physta._register, csistatus._register);*/

		error = ((physta._register & 0x0055) != 0) ||
			((csistatus._register & 0x01BF) != 0);

		/* We hit the first possible value of the PHYTIMDLY window */
		if (!error && !state) {

			/* In fast calibration mode, assume the window is ~10
			 * dly large and set the value in the middle */
#ifdef CONFIG_TC358746A_FAST_CALIBRATION
			phytimdly.dsettle = dly + 4;
			tc358746a_write(sd, PHYTIMDLY, phytimdly._register);
			return 0;
#endif

			first_dly = dly;
			state = 1;
		}

		/* We hit the last possible value of the PHYTIMDLY window */
		if (error && state) {
			/* Position the PHYTIMDLY in the middle of the window */
			phytimdly.dsettle = (dly + first_dly) / 2;
			v4l2_info(sd,
				  "Measured PHYTIMDLY: %d\n",
				  phytimdly.dsettle);
			tc358746a_write(sd, PHYTIMDLY, phytimdly._register);
			return 0;
		}
	}

	v4l2_err(sd, "Could not find a correct PHYTIMDLY value\n");

	return -1;
}

static int tc358746a_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc358746a               *tc358746a = to_tc358746a(sd);
	struct tc358746a_platform_data *pdata = tc358746a->pdata;
	struct v4l2_subdev             *remote;
	struct v4l2_subdev_format       fmt;
	struct v4l2_dv_timings          timings;
	int                             ret;
	union confctl                   confctl;
	union datafmt                   datafmt;

	remote = tc358746a_get_remote_sd(sd);

	if (!remote)
		return -ENODEV;

	/* For stream off, we just need to propagate the command */
	if (enable == 0) {
		ret = v4l2_subdev_call(remote, video, s_stream, enable);

		if (ret < 0)
			v4l2_err(sd, "cannot call s_stream on remote subdev\n");

		if (pdata->set_power)
			pdata->set_power(TC358746A_POWER_OFF);

		return ret;
	}

	/* TC358746A needs to be configured before any CSI stream is enabled,
	 * thus we have to put a default configuration, activate the remote
	 * device and then auto-calibrate if needed.
	 */
	if (pdata->set_power)
		pdata->set_power(TC358746A_POWER_ON);

	/* Software reset */
	tc358746a_write(sd, SYSCTL, 0x0001);
	tc358746a_write(sd, SYSCTL, 0x0000);

	/* Ask remote subdev the pixel clock */
	ret = v4l2_subdev_call(remote, video, g_dv_timings, &timings);
	if (ret < 0) {
		v4l2_err(sd, "cannot call g_dv_timings on remote subdev\n");
		goto power_off;
	}
	tc358746a->pclk = timings.bt.pixelclock;

	ret = tc358746a_set_pclk(sd);
	if (ret < 0)
		goto power_off;

	/* PHYTimDly - csettle & dsettle
	 * Here we set it to its minimum, after activating the remote device, we
	 * will start the auto-calibration.
	 */
	tc358746a_write(sd, PHYTIMDLY, 0x8000);

	/* Here we explicitly specify the format, since in some cases, the
	 * remote subdev generates some embedded data at the beginning with
	 * non-consistant MIPI code.
	 */
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad   = 0;

	ret = v4l2_subdev_call(remote, pad, get_fmt, NULL, &fmt);
	if (ret < 0) {
		v4l2_err(sd, "cannot call get_fmt on remote subdev\n");
		goto power_off;
	}

	datafmt.udt_en = 1;

	switch (fmt.format.code) {
	case V4L2_MBUS_FMT_SBGGR10_1X10:
	case V4L2_MBUS_FMT_SGBRG10_1X10:
	case V4L2_MBUS_FMT_SGRBG10_1X10:
	case V4L2_MBUS_FMT_SRGGB10_1X10:
		datafmt.pdformat = 1;
		break;
	case V4L2_MBUS_FMT_SBGGR12_1X12:
	case V4L2_MBUS_FMT_SGBRG12_1X12:
	case V4L2_MBUS_FMT_SGRBG12_1X12:
	case V4L2_MBUS_FMT_SRGGB12_1X12:
		datafmt.pdformat = 2;
		break;
	default:
		v4l2_err(sd, "code not supported yet\n");
		ret = -EINVAL;
		goto power_off;
	}

	tc358746a_write(sd, DATAFMT, datafmt._register);

	/* ConfCtl */
	confctl.datalane  = pdata->lanes - 1,
	confctl.auto_incr = 1,
	confctl.pclkp     = 0,
	confctl.hsyncp    = 0,
	confctl.vsyncp    = 0,
	confctl.ppen      = 1,
	confctl.pdataf    = 0,
	confctl.bt656en   = 0,
	confctl.inte2n    = 0,
	confctl.trien     = 1,

	tc358746a_write(sd, CONFCTL, confctl._register);

	/* Finally we can call the stream on for the remote subdev */
	ret = v4l2_subdev_call(remote, video, s_stream, enable);
	if (ret < 0) {
		v4l2_err(sd, "cannot call s_stream on remote subdev\n");
		goto power_off;
	}
	/* We finish with a calibration in order to center the delay */
	ret = tc358746a_calibrate(sd);
	if (ret < 0)
		goto ecalibrate;

	return 0;

ecalibrate:
	v4l2_subdev_call(remote, video, s_stream, 0);
power_off:
	if (pdata->set_power)
		pdata->set_power(TC358746A_POWER_OFF);

	return ret;
}

static int tc358746a_g_dv_timings(struct v4l2_subdev *sd,
				  struct v4l2_dv_timings *timings)
{
	struct v4l2_subdev *remote = tc358746a_get_remote_sd(sd);

	if (!remote)
		return -ENODEV;

	/* Ask the remote directly */
	return v4l2_subdev_call(remote, video, g_dv_timings, timings);
}


static const struct v4l2_subdev_video_ops tc358746a_video_ops = {
	.s_stream     = tc358746a_s_stream,
	.g_dv_timings = tc358746a_g_dv_timings,
};

static const struct v4l2_subdev_ops tc358746a_ops = {
	.core  = &tc358746a_core_ops,
	.video = &tc358746a_video_ops,
	.pad   = &tc358746a_pad_ops,
};

static int tc358746a_detect_chip(struct v4l2_subdev *sd)
{
	struct tc358746a               *tc358746a = to_tc358746a(sd);
	struct tc358746a_platform_data *pdata = tc358746a->pdata;
	int                             ret = 0;
	u16                             chip_id = 0;

	if (pdata->set_power) {
		ret = pdata->set_power(TC358746A_POWER_ON);
		if (ret) {
			v4l2_err(sd, "Power on failed\n");
			return ret;
		}
	}

	ret = tc358746a_read(sd, CHIPID, &chip_id);

	if (pdata->set_power)
		pdata->set_power(TC358746A_POWER_OFF);

	if (ret) {
		v4l2_err(sd, "Couldn't read on I2C bus");
		return ret;
	}

	if (chip_id == TC358746A_CHIPID ||
	    chip_id == TC358746A_CHIPID_REV0) {
		v4l2_info(sd, "Found " DRIVER_NAME " chip\n");

		return 0;
	}

	v4l2_err(sd, "Error Chip ID = 0x%04x instead of 0x%04x\n",
		     chip_id, TC358746A_CHIPID);

	return -ENODEV;
}

static int tc358746a_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tc358746a               *tc358746a;
	struct tc358746a_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev             *sd;
	int                             ret = 0;

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

	tc358746a = kzalloc(sizeof(*tc358746a), GFP_KERNEL);

	if (!tc358746a) {
		dev_err(&client->dev, "alloc failed for data structure\n");
		return -ENOMEM;
	}

	tc358746a->pdata = pdata;

	sd = &tc358746a->sd;
	v4l2_i2c_subdev_init(sd, client, &tc358746a_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	tc358746a->pad[TC358746A_IN_PAD].flags = MEDIA_PAD_FL_SINK;
	tc358746a->pad[TC358746A_OUT_PAD].flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	ret = media_entity_init(&sd->entity,
				TC358746A_NUM_PADS,
				tc358746a->pad,
				0);

	if (ret < 0) {
		v4l_err(client, "failed to init media entity\n");
		goto emedia;
	}

	/* Check if the chip is preset */
	ret = tc358746a_detect_chip(sd);
	if (ret < 0)
		goto edetect;

	v4l2_info(sd, "TC358746A MIPI bridge successfully probed");

	return 0;

emedia:
edetect:
	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(tc358746a);

	return ret;
}

static int tc358746a_remove(struct i2c_client *client)
{
	struct v4l2_subdev             *sd = i2c_get_clientdata(client);
	struct tc358746a               *tc358746a = to_tc358746a(sd);
	struct tc358746a_platform_data *pdata = client->dev.platform_data;

	if (!pdata)
		return -EINVAL;

	if (pdata->set_power)
		pdata->set_power(TC358746A_POWER_OFF);

	media_entity_cleanup(&sd->entity);
	v4l2_device_unregister_subdev(sd);
	kfree(tc358746a);

	return 0;
}

static const struct i2c_device_id tc358746a_id[] = {
	{DRIVER_NAME, 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, tc358746a_id);

static struct i2c_driver tc358746a_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = DRIVER_NAME,
	},
	.probe    = tc358746a_probe,
	.remove   = tc358746a_remove,
	.id_table = tc358746a_id,
};

module_i2c_driver(tc358746a_driver);
