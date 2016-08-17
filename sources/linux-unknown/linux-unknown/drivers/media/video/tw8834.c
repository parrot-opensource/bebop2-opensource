/* tw8834.c
 *
 * Driver for techwell tw8834
 * It handles 2 inputs, composite and bt656 input
 * It handles one output, which is bt656
 *
 * Author : Julien BERAUD <julien.beraud@parrot.com>
 *
 * Date : 23/04/2014
 *
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <media/tw8834.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-event.h>

MODULE_AUTHOR("Julien BERAUD <julien.beraud@parrot.com>");
MODULE_DESCRIPTION("kernel driver for tw8834");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

enum {
	TW8834_IN_PAD = 0,
	TW8834_OUT_PAD,
	TW8834_NUM_PADS,
};

enum {
	TW8834_INPUT_CVBS = 0,
	TW8834_INPUT_DTV,
	TW8834_NUM_INPUTS,
};

static struct v4l2_dv_timings tw8834_ntsc_timings = V4L2_DV_BT_CEA_720X480I59_94;


static struct v4l2_dv_timings tw8834_pal_timings = V4L2_DV_BT_CEA_720X576I50;


#define TW8834_CVBS_WIDTH 720
#define TW8834_CVBS_PAL_HEIGHT 576
#define TW8834_CVBS_NTSC_HEIGHT 480

#define TW8834_IRQ		0x002
#define TW8834_IMASK		0x003
#define TW8834_STATUS		0x004
#define TW8834_STD		0x11C
#define TW8834_STDR		0x11D
#define TW8834_BT656_DEC_CTRL2	0x48
#define TW8834_CSTATUS		0x101

/* tw8834 data structure */
struct tw8834 {
	struct v4l2_subdev sd;
	struct i2c_client *i2c_client;
	struct media_pad pad[TW8834_NUM_PADS];
	struct v4l2_mbus_framefmt format[TW8834_NUM_PADS];
	int input;
	int is_streaming;
	int (*set_power)(int on);
};

static inline struct tw8834 *to_tw8834(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tw8834, sd);
}

/* Register maps */
struct reg_cfg {
	unsigned char reg;
	unsigned char val;
};

static struct reg_cfg tw8834_common_cfg[] = {
	{ 0xFF, 0x00 },
	{ 0x08, 0xC6 },
	{ 0x07, 0x08 },//Reserved bit in tw8834 doc but enable bt656 in tw8836 doc
	{ 0x09, 0x05 },//VD input clock polarity control:negative + reserved
	{ 0x41, 0xC0 },//Diff-dtv thresholds for DTV no use for cvbs
	{ 0x51, 0x00 },//
	{ 0x52, 0x02 },//
	{ 0x53, 0x01 },//Diff->unnecessary diff, apply dtv config to both.
	{ 0xDC, 0x00 },//pwm1 freq control
	{ 0xDD, 0x80 },//pwm1 duty ctrl
	{ 0xDE, 0x00 },//pw2 freq ctrl
	{ 0xDF, 0x80 },//pw2 duty ctrl
	{ 0xF9, 0x50 },//sspll freq
	{ 0xFC, 0x30 },//sspll
	{ 0xFF, 0x00 },//
	{ 0xFF, 0x01 },//
	{ 0x02, 0x48 },//Diff-> unapplyable to dtv cvbs input select
	{ 0x05, 0x09 },//anti-aliasing enable
	{ 0x08, 0x16 },//
	{ 0x0A, 0x10 },//
	{ 0x11, 0x64 },//
	{ 0x17, 0x80 },//
	{ 0x21, 0x22 },//
	{ 0x2F, 0xE4 },//LCS enabled
	{ 0x30, 0x00 },//undocumented
	{ 0x3D, 0x40 },
	{ 0xFF, 0x01 },
	{ 0xFF, 0x02 },
	{ 0x40, 0x11 },
	{ 0x41, 0x0A },
	{ 0x42, 0x05 },
	{ 0x43, 0x01 },
	{ 0x44, 0x64 },
	{ 0x45, 0xF4 },
	{ 0x47, 0x0A },
	{ 0x4C, 0x00 },
	{ 0x4D, 0x44 },
	{ 0x80, 0x00 },
	{ 0x8B, 0x44 },
	{ 0xB0, 0x30 },
	{ 0xFF, 0x02 },
};

static struct reg_cfg tw8834_cvbs_in_cfg[] = {
	{ 0xff, 0x00 },
	{ 0x06, 0x06 },//Diff
	{ 0x67, 0x00 },//Diff source of bt656 output
	{ 0x69, 0x02 },//Diff crop dtv
	{ 0x6A, 0x20 },//Diff crop dtv
	{ 0x6B, 0xF0 },//Diff crop dtv
	{ 0x6C, 0x20 },//Diff crop dtv
	{ 0x6D, 0xD0 },//Diff crop dtv
	{ 0xFF, 0x01 },//
	{ 0x0C, 0xDC },//black level is 7.5 IRE above the blank level
	{ 0x10, 0xD0 },//Diff-brightness
	{ 0x20, 0x30 },//Diff-clamping
	{ 0x24, 0x60 },//Diff-clamping
	{ 0x37, 0xBA },//Diff
	{ 0x3A, 0x00 },//Diff
	{ 0x3B, 0x0F },//Diff
	{ 0xFF, 0x01 },//
	{ 0xFF, 0x02 },//
	{ 0x48, 0x36 },//Added (not present for dtv)
	{ 0xFF, 0x02 },
};

static struct reg_cfg tw8834_dtv_in_cfg[] = {
	{ 0xFF, 0x00 },//
	{ 0x06, 0x26 },//Diff
	{ 0x67, 0x02 },//Diff source of bt656 output
	{ 0x69, 0x12 },//Diff crop dtv
	{ 0x6A, 0x4F },//Diff crop dtv
	{ 0x6B, 0xE0 },//Diff crop dtv
	{ 0x6C, 0xB4 },//Diff crop dtv
	{ 0x6D, 0x80 },//Diff crop dtv
	{ 0xFF, 0x01 },//
	{ 0x10, 0x00 },//Diff-brightness
	{ 0x20, 0x50 },//Diff-clamping
	{ 0x24, 0x3C },//Diff-clamping
	{ 0x37, 0xB9 },//Diff
	{ 0x3A, 0x88 },//Diff
	{ 0x3B, 0xCF },//Diff
	{ 0xFF, 0x01 },//
};

/* i2c transfer functions */
static s32 tw8834_i2c_read_byte(struct tw8834 *tw8834,
				u8 command)
{
	struct i2c_client *client = tw8834->i2c_client;
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	else
		return -EIO;
}

static s32 tw8834_i2c_write_byte(struct tw8834 *tw8834,
				 u8 command,
				 u8 value)
{
	struct i2c_client *client = tw8834->i2c_client;
	union i2c_smbus_data data;
	int err;
	int i;

	data.byte = value;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_xfer(client->adapter, client->addr,
				client->flags,
				I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
		if (!err)
			break;
	}
	if (err < 0)
		v4l_err(client, "error writing %02x, %02x, %02x\n",
				client->addr, command, value);
	return err;
}

static int tw8834_write_reg_array(struct tw8834 *tw8834,
				  struct reg_cfg *array, int nb)
{
	s32 ret;
	int i;
	struct v4l2_subdev *sd = &tw8834->sd;

	for(i=0;i<nb;i++) {
		ret = tw8834_i2c_write_byte(tw8834,
					    array[i].reg,
					    array[i].val);
		if(ret < 0) {
			v4l2_err(sd, "failed to write 0x%X to reg 0x%X\n",
				 array[i].val, array[i].reg);
			return ret;
		}
	}
	return 0;
}

static int tw8834_read_register(struct tw8834 *tw8834,
				u16 reg)
{
	int ret;
	u8 page = (reg >> 8);

	ret = tw8834_i2c_write_byte(tw8834, 0xFF, page);
	if(ret < 0)
		return ret;

	return (tw8834_i2c_read_byte(tw8834, (u8)(reg & 0xFF)));
}

static int tw8834_write_register(struct tw8834 *tw8834,
				 u16 reg,
				 u8 value)
{
	int ret;
	u8 page = (reg >> 8);

	ret = tw8834_i2c_write_byte(tw8834, 0xFF, page);
	if(ret < 0)
		return ret;

	ret = tw8834_i2c_write_byte(tw8834, (u8)(reg & 0xFF), value);

	return ret;
}

/* Helpers */
static struct v4l2_subdev *
tw8834_remote_subdev(struct tw8834 *tw8834, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_source(&tw8834->pad[TW8834_IN_PAD]);

	if (remote == NULL ||
	    media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

/* video operations */

static int tw8834_s_routing(struct v4l2_subdev *sd, u32 in, u32 out, u32 config)
{
	struct tw8834 *tw8834 = to_tw8834(sd);

	if(!tw8834 || in >= TW8834_NUM_INPUTS)
		return -EINVAL;

	tw8834->input = in;

	return 0;
}

static struct v4l2_mbus_framefmt *
__tw8834_get_fmt(struct tw8834 *tw8834,
		 struct v4l2_subdev_fh *fh,
		 unsigned int pad,
		 enum v4l2_subdev_format_whence which)
{
	if(pad > TW8834_NUM_PADS)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &tw8834->format[pad];
}

static int tw8834_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct tw8834 *tw8834 = to_tw8834(sd);
	struct v4l2_mbus_framefmt *format;

	format = __tw8834_get_fmt(tw8834, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int tw8834_set_in_fmt(struct tw8834 *tw8834,struct v4l2_subdev_fh *fh,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_mbus_framefmt *in_format;
	struct v4l2_mbus_framefmt *out_format;

	out_format = __tw8834_get_fmt(tw8834, fh, TW8834_OUT_PAD, V4L2_SUBDEV_FORMAT_ACTIVE);
	if(out_format == NULL)
		return -EINVAL;

	if(tw8834->input == TW8834_INPUT_CVBS) {
		v4l2_err(&tw8834->sd, "Cannot set input pad format for cvbs\n");
		return -EINVAL;
	}
	else if(tw8834->input == TW8834_INPUT_DTV) {
		in_format = __tw8834_get_fmt(tw8834, fh, TW8834_IN_PAD, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (in_format == NULL)
			return -EINVAL;

		if(fmt->width == 640 && fmt->height == 480) {
		/*Up to now, I have only been able to make it work in bt656
		because bt601 is not functionnal with the register config given
		by Intersil and I only had time to make VGA mode work, which is
		the mode we are willing to use the input sensor in (ref:mt9v117) */
			fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
		}
		else
			return -EINVAL;

		*in_format = *fmt;
		*out_format = *fmt;
		/*I have currently only been able to output 694 lines that have
		to be cropped with top=1, height=494*/
		out_format->height = 494;
	}

	return 0;
}

static int tw8834_set_out_fmt(struct tw8834 *tw8834,struct v4l2_subdev_fh *fh,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct v4l2_mbus_framefmt *out_format;

	out_format = __tw8834_get_fmt(tw8834, fh, TW8834_OUT_PAD, V4L2_SUBDEV_FORMAT_ACTIVE);
	if(out_format == NULL)
		return -EINVAL;

	if(tw8834->input == TW8834_INPUT_CVBS) {
		if(fmt->width == TW8834_CVBS_WIDTH &&
		   fmt->height == TW8834_CVBS_PAL_HEIGHT) {
			fmt->field = V4L2_FIELD_INTERLACED;
			fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
		}
		else if(fmt->width == TW8834_CVBS_WIDTH &&
			fmt->height == TW8834_CVBS_NTSC_HEIGHT) {
			fmt->field = V4L2_FIELD_INTERLACED;
			fmt->colorspace = V4L2_COLORSPACE_SMPTE170M;
		}
		else
			return -EINVAL;

		/*propagate to out pad */
		*out_format = *fmt;
	}
	else if(tw8834->input == TW8834_INPUT_DTV) {
		v4l2_err(&tw8834->sd, "cannot set out fmt for dtv\n");
		return -EINVAL;
	}

	return 0;
}

static int tw8834_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct tw8834 *tw8834 = to_tw8834(sd);
	int ret = 0;

	if(fmt->pad == TW8834_IN_PAD)
		ret = tw8834_set_in_fmt(tw8834, fh, &fmt->format);
	else
		ret = tw8834_set_out_fmt(tw8834, fh, &fmt->format);

	return ret;
}

static int tw8834_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct tw8834 *tw8834 = to_tw8834(sd);

	if (code->index)
		return -EINVAL;

	code->code = tw8834->format[code->pad].code;
	return 0;
}

static int tw8834_enable_cvbs(struct tw8834 *tw8834)
{
	tw8834_write_reg_array(tw8834,
			       tw8834_cvbs_in_cfg,
			       ARRAY_SIZE(tw8834_cvbs_in_cfg));
	return 0;
}

static int tw8834_enable_dtv(struct v4l2_subdev *sd, struct tw8834 *tw8834)
{
	struct v4l2_subdev *dtv_sd;
	int ret = 0;

	dtv_sd = tw8834_remote_subdev(tw8834, NULL);
	if(!sd) {
		v4l2_err(sd, "no remote subdev found\n");
		return -ENODEV;
	}
	ret = v4l2_subdev_call(dtv_sd, video, s_stream, 1);
	if(ret < 0) {
		v4l2_err(sd, "cannot start streaming on dtv subdev\n");
		return ret;
	}
	else {
		tw8834_write_reg_array(tw8834,
				       tw8834_dtv_in_cfg,
				       ARRAY_SIZE(tw8834_dtv_in_cfg));
	}
	return ret;
}

static int tw8834_disable_dtv(struct v4l2_subdev *sd, struct tw8834 *tw8834)
{
	struct v4l2_subdev *dtv_sd;
	int ret = 0;

	dtv_sd = tw8834_remote_subdev(tw8834, NULL);
	if(!sd) {
		v4l2_err(sd, "no remote subdev found\n");
		return -ENODEV;
	}
	ret = v4l2_subdev_call(dtv_sd, video, s_stream, 0);
	if(ret < 0) {
		v4l2_err(sd, "cannot stop streaming on dtv subdev\n");
		return ret;
	}
	return ret;
}

static int tw8834_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tw8834 *tw8834 = to_tw8834(sd);
	int ret = 0;

	if(enable) {
		if(tw8834->set_power)
			tw8834->set_power(TW8834_POWER_INPUT_ENABLE);
		if(tw8834->input == TW8834_INPUT_CVBS)
			ret = tw8834_enable_cvbs(tw8834);
		else
			ret = tw8834_enable_dtv(sd, tw8834);
	}
	else
	{
		if(tw8834->set_power)
			tw8834->set_power(TW8834_POWER_INPUT_DISABLE);
		if(tw8834->input == TW8834_INPUT_DTV)
			ret = tw8834_disable_dtv(sd, tw8834);
	}

	tw8834->is_streaming = !!enable;

	return ret;
}

static int tw8834_query_dv_timings(struct v4l2_subdev *sd,
				   struct v4l2_dv_timings *timings)
{
	struct tw8834 *tw8834 = to_tw8834(sd);
	unsigned char status;
	unsigned char freq_50_hz = 0;
	int lock_timeout = 20;

	while (--lock_timeout) {
		status = tw8834_read_register(tw8834,
					TW8834_CSTATUS);
		if (status & 0x80) {
			v4l2_info(sd, "Video loss\n");
			return -ENOLINK;
		}
		else if (((status & 0x68) == 0x68)) {
			v4l2_info(sd, "signal locked\n");
			break;
		}
		else {
			v4l2_info(sd, "wait for lock, status = 0x%x\n", status);
			msleep(50);
		}
	}

	if (!lock_timeout) {
		v4l2_info(sd, "timeout waiting for lock\n");
		return -ENOLINK;
	}

	freq_50_hz = (status & 0x1);

	if (freq_50_hz) {
		*timings = tw8834_pal_timings;
		/*pclk doubled for 8 bits transmission of YUYV
		  so horizontal timngs are doubled*/
		timings->bt.pixelclock = 2 * timings->bt.pixelclock;
		timings->bt.hfrontporch = 2 * timings->bt.hfrontporch;
		timings->bt.hsync = 2 * timings->bt.hsync;
		timings->bt.hbackporch = 2 * timings->bt.hbackporch;
	}
	else {
		*timings = tw8834_ntsc_timings;
		/*pclk doubled for 8 bits transmission of YUYV
		  so horizontal timings are doubled*/
		timings->bt.pixelclock = 2 * timings->bt.pixelclock;
		timings->bt.hfrontporch = 2 * timings->bt.hfrontporch;
		timings->bt.hsync = 2 * timings->bt.hsync;
		timings->bt.hbackporch = 2 * timings->bt.hbackporch;
	}

	return 0;
}

int tw8834_subscribe_event(struct v4l2_subdev *subdev,
			   struct v4l2_fh *fh,
			   struct v4l2_event_subscription *sub)
{
	return v4l2_src_change_event_subscribe(fh, sub);
}

int tw8834_unsubscribe_event(struct v4l2_subdev *subdev,
			     struct v4l2_fh *fh,
			     struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}
static const struct v4l2_subdev_core_ops tw8834_core_ops = {
	.subscribe_event = tw8834_subscribe_event,
	.unsubscribe_event = tw8834_unsubscribe_event,
};

static const struct v4l2_subdev_video_ops tw8834_video_ops = {
	.s_stream = tw8834_s_stream,
	.query_dv_timings = tw8834_query_dv_timings,
	.s_routing = tw8834_s_routing,
};

static const struct v4l2_subdev_pad_ops tw8834_pad_ops = {
	.get_fmt = tw8834_get_fmt,
	.set_fmt = tw8834_set_fmt,
	.enum_mbus_code = tw8834_enum_mbus_code,
};

static const struct v4l2_subdev_ops tw8834_ops = {
	.core = &tw8834_core_ops,
	.video = &tw8834_video_ops,
	.pad = &tw8834_pad_ops,
};

static irqreturn_t tw8834_isr(int irq, void *priv)
{
	struct tw8834 *tw8834 = priv;
	unsigned char status, tw8834_irq;
	struct video_device *vdev = tw8834->sd.devnode;
	static const struct v4l2_event event = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes =
		        V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	tw8834_irq = tw8834_read_register(tw8834, TW8834_IRQ);
	status = tw8834_read_register(tw8834, TW8834_CSTATUS);
	tw8834_write_register(tw8834, TW8834_IRQ, tw8834_irq);

	v4l2_info(&tw8834->sd,
		 "read status = 0x%X irq = 0x%X\n", status, tw8834_irq);

	if(vdev)
		v4l2_event_queue(vdev, &event);
	return IRQ_HANDLED;
}

static int tw8834_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tw8834 *tw8834;
	struct tw8834_platform_data *pdata = NULL;
	struct v4l2_subdev *sd;
	int ret = -ENODEV;
	struct v4l2_mbus_framefmt *format;
	struct reg_cfg reg;
	pdata = client->dev.platform_data;

	if(!pdata)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}
	v4l_dbg(1, debug, client, "detecting tw8834 client on address 0x%x\n",
			client->addr << 1);

	tw8834 = kzalloc(sizeof(*tw8834), GFP_KERNEL);
	if(!tw8834) {
		v4l_err(client, "alloc failed for data structure\n");
		ret = -ENOMEM;
		goto enomem;
	}

	tw8834->set_power = pdata->set_power;
	tw8834->i2c_client = client;
	tw8834->input = TW8834_INPUT_CVBS;

	format = &tw8834->format[TW8834_IN_PAD];
	format->width = 640;
	format->height = 480;
	format->field = V4L2_FIELD_NONE;
	format->code = V4L2_MBUS_FMT_UYVY8_2X8;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;

	format = &tw8834->format[TW8834_OUT_PAD];
	format->width = TW8834_CVBS_WIDTH;
	format->height = TW8834_CVBS_PAL_HEIGHT;
	format->field = V4L2_FIELD_INTERLACED;
	format->code = V4L2_MBUS_FMT_UYVY8_2X8;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;

	sd = &tw8834->sd;
	v4l2_i2c_subdev_init(sd, client, &tw8834_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	tw8834->pad[TW8834_IN_PAD].flags = MEDIA_PAD_FL_SINK;
	tw8834->pad[TW8834_OUT_PAD].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 2, tw8834->pad, 0);
	if(ret < 0) {
		v4l_err(client, "failed to init media entity\n");
		goto emedia;
	}

	/*see if chip is present */
	if(pdata->set_power) {
		pdata->set_power(TW8834_POWER_ON);
		msleep(500);
	}

	reg.reg = 0xFF;
	reg.val = 0x00;

	ret = tw8834_i2c_write_byte(tw8834, reg.reg, reg.val);

	if(ret < 0) {
		v4l_err(client, "impossible to write to tw8834\n");
		goto echipident;
	}

	reg.reg = 0x00;

	ret = tw8834_i2c_read_byte(tw8834, reg.reg);

	if(ret < 0)
	{
		v4l_err(client, "failed to read chip id\n");
		goto echipident;
	}

	reg.val = ret;

	if(reg.val != 0x34) {
		v4l_err(client, "read 0x%d in chip ident reg\n", reg.val);
		ret = -ENODEV;
		goto echipident;
	}

	if(client->irq > 0) {
		ret = request_threaded_irq(client->irq,NULL,
				tw8834_isr,
				IRQF_ONESHOT | IRQF_TRIGGER_LOW,
				"tw8834", tw8834);
		if(ret < 0) {
			v4l2_err(client, "failed to register irq\n");
			client->irq = -1;
		}
	}
	tw8834_write_reg_array(tw8834,
			       tw8834_common_cfg,
			       ARRAY_SIZE(tw8834_common_cfg));
	tw8834_enable_cvbs(tw8834);
	tw8834_write_register(tw8834, TW8834_IRQ, 0xFF);
	tw8834_write_register(tw8834, TW8834_STATUS, 0xFF);
	tw8834_write_register(tw8834, TW8834_IMASK, 0xEC);
	tw8834_write_register(tw8834, TW8834_BT656_DEC_CTRL2, 0x10);

	return 0;
echipident:
	if(pdata->set_power)
		pdata->set_power(TW8834_POWER_OFF);
	media_entity_cleanup(&sd->entity);
emedia:
	kfree(tw8834);
enomem:
ei2c:
	return ret;
}

static int tw8834_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw8834 *tw8834 = to_tw8834(sd);
	struct tw8834_platform_data *pdata = NULL;

	pdata = client->dev.platform_data;

	if(pdata->set_power)
		pdata->set_power(TW8834_POWER_OFF);

	if(client->irq > 0)
		free_irq(client->irq, tw8834);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	kfree(tw8834);
	return 0;
}

#ifdef CONFIG_PM
int tw8834_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw8834 *tw8834 = to_tw8834(sd);
	int is_streaming;

	if(tw8834->set_power)
		tw8834->set_power(TW8834_POWER_OFF);
	// Saving the state to restore at resume
	is_streaming = tw8834->is_streaming;

	// Stop if it was streaming
	if (is_streaming)
		tw8834_s_stream(sd, 0);

	tw8834->is_streaming = is_streaming;

	return 0;
}

int tw8834_resume(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw8834 *tw8834 = to_tw8834(sd);

	if(tw8834->set_power){
		tw8834->set_power(TW8834_POWER_ON);
		msleep(100);
	}

	tw8834_write_reg_array(tw8834,
			       tw8834_common_cfg,
			       ARRAY_SIZE(tw8834_common_cfg));
	tw8834_enable_cvbs(tw8834);
	tw8834_write_register(tw8834, TW8834_IRQ, 0xFF);
	tw8834_write_register(tw8834, TW8834_STATUS, 0xFF);
	tw8834_write_register(tw8834, TW8834_IMASK, 0xEC);
	tw8834_write_register(tw8834, TW8834_BT656_DEC_CTRL2, 0x10);

	if (tw8834->is_streaming)
		tw8834_s_stream(sd, 1);

	return 0;
}
#endif

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id tw8834_id[] = {
	{ "tw8834", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tw8834_id);

static struct i2c_driver tw8834_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tw8834",
	},
	.probe = tw8834_probe,
	.remove = tw8834_remove,
#ifdef CONFIG_PM
	.suspend = tw8834_suspend,
	.resume = tw8834_resume,
#endif
	.id_table = tw8834_id,
};

module_i2c_driver(tw8834_driver);
