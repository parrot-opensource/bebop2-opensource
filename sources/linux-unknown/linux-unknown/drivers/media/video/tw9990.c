/* tw9990.c
 *
 * Driver for techwell tw9990
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
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <media/tw9990.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-event.h>

MODULE_AUTHOR("Julien BERAUD <julien.beraud@parrot.com>");
MODULE_DESCRIPTION("kernel driver for tw9990");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

#define TW9990_NUM_INPUTS 2

#define TW9990_CVBS_WIDTH 720
#define TW9990_CVBS_PAL_HEIGHT 576
#define TW9990_CVBS_NTSC_HEIGHT 480

#define TW_BIT_SHIFT_L(val, shift) (val<<shift)
#define TW_MASK(bits) (TW_BIT_SHIFT_L(1, bits)-1)

/* registers */
#define TW9990_CHIP_STATUS 0x01
#define TW9990_IN_FORM 0x02
#define TW9990_OP_FORM 0x03
#define TW9990_CMN_MODE_CLAMP 0x0D
#define TW9990_AN_CTRL_2 0x1A
#define TW9990_STD_SEL 0x1C
#define TW9990_STD_RECOG 0x1D
#define TW9990_SYNCT 0x25
#define TW9990_MISC1_CTRL_1 0x2D
#define TW9990_SHORT_DIAG 0xAF
#define TW9990_INT2_ENABLE 0xB1
#define TW9990_INT2_RST 0xB4
#define TW9990_INT2_RAW_STATUS 0xB7
#define TW9990_INT2_STATUS 0xBA

enum {
	TW9990_IN_FORM_ID = 0,
	TW9990_OP_FORM_ID,
	TW9990_CMN_MODE_CLAMP_ID,
	TW9990_AN_CTRL_2_ID,
	TW9990_SYNCT_ID,
	TW9990_MISC1_CTRL_1_ID,
	TW9990_MAX_IDS
};

/* Register maps */
struct reg_cfg {
	unsigned char reg;
	unsigned char val;
};

/* tw9990 data structure */
struct tw9990 {
	struct v4l2_subdev sd;
	struct i2c_client *i2c_client;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct v4l2_subdev_frame_interval frame_interval;
	int is_streaming;
	int (*set_power)(int on);
	int irq;
	int differential_input;
	struct reg_cfg config[TW9990_MAX_IDS];
};

static inline struct tw9990 *to_tw9990(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tw9990, sd);
}

static struct v4l2_dv_timings tw9990_ntsc_timings = V4L2_DV_BT_CEA_720X480I59_94;


static struct v4l2_dv_timings tw9990_pal_timings = V4L2_DV_BT_CEA_720X576I50;

/* i2c transfer functions */
static s32 tw9990_i2c_read_byte(struct tw9990 *tw9990,
				u8 command)
{
	struct i2c_client *client = tw9990->i2c_client;
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	else
		return -EIO;
}

static s32 tw9990_i2c_write_byte(struct tw9990 *tw9990,
				 u8 command,
				 u8 value)
{
	struct i2c_client *client = tw9990->i2c_client;
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
		v4l2_err(client, "error writing %02x, %02x, %02x\n",
				client->addr, command, value);
	return err;
}

static int tw9990_write_reg_array(struct tw9990 *tw9990,
				  struct reg_cfg *array, int nb)
{
	s32 ret;
	int i;
	struct v4l2_subdev *sd = &tw9990->sd;

	for(i=0;i<nb;i++) {
		ret = tw9990_i2c_write_byte(tw9990,
					    array[i].reg,
					    array[i].val);
		if(ret < 0) {
			v4l2_err(sd, "failed to write 0x%X to reg 0x%X\n",
				 array[i].reg, array[i].val);
			return ret;
		}
	}
	return 0;
}

/* video operations */

static struct v4l2_mbus_framefmt *
__tw9990_get_fmt(struct tw9990 *tw9990,
		 struct v4l2_subdev_fh *fh,
		 unsigned int pad,
		 enum v4l2_subdev_format_whence which)
{
	if(pad != 0)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &tw9990->format;
}

static int tw9990_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct tw9990 *tw9990 = to_tw9990(sd);
	struct v4l2_mbus_framefmt *format;

	format = __tw9990_get_fmt(tw9990, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int tw9990_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct tw9990 *tw9990 = to_tw9990(sd);
	int ret = 0;
	struct v4l2_mbus_framefmt *format;

	format = __tw9990_get_fmt(tw9990, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	if(fmt->format.width != 720 ||
	   (fmt->format.height != 480 && fmt->format.height != 576))
		return -EINVAL;

	fmt->format.colorspace = V4L2_COLORSPACE_SMPTE170M;

	*format = fmt->format;
	return ret;
}

static int tw9990_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct tw9990 *tw9990 = to_tw9990(sd);

	if (code->index)
		return -EINVAL;

	code->code = tw9990->format.code;
	return 0;
}

static void tw9990_setup(struct tw9990 *tw9990,
		struct tw9990_platform_data *pdata)
{
	/* Input format configuration: by default on MUX0 */
	tw9990->config[TW9990_IN_FORM_ID].reg = TW9990_IN_FORM;
	tw9990->config[TW9990_IN_FORM_ID].val = (0x40 |
		((pdata->differential_input == TW9990_DIFFERENTIAL_ENABLED) ?
				0x80 : 0));

	/* Output format: configure ITU-656 data sequence format */
	tw9990->config[TW9990_OP_FORM_ID].reg = TW9990_OP_FORM;
	tw9990->config[TW9990_OP_FORM_ID].val = 0xA0;

	/* Common mode clamp: based on platform data only */
	tw9990->config[TW9990_CMN_MODE_CLAMP_ID].reg = TW9990_CMN_MODE_CLAMP;
	tw9990->config[TW9990_CMN_MODE_CLAMP_ID].val =
			pdata->common_mode_clamp;

	/* Analog control II: configure anti-alias and power saving
	 * for Y/C channels, based on platform data.
	 */
	tw9990->config[TW9990_AN_CTRL_2_ID].reg = TW9990_AN_CTRL_2;
	tw9990->config[TW9990_AN_CTRL_2_ID].val =
		(pdata->anti_aliasing == TW9990_ANTI_ALIAS_ENABLED) ?
				0x0A : 0;
	tw9990->config[TW9990_AN_CTRL_2_ID].val |=
		(pdata->power_saving == TW9990_POWER_SAVE_ENABLED) ?
				0x05 : 0;

	/* Sync amplitude (SYNCT) */
	tw9990->config[TW9990_SYNCT_ID].reg = TW9990_SYNCT;
	if (pdata->synct_mode == TW9990_SYNCT_FORCED)
		tw9990->config[TW9990_SYNCT_ID].val =
				pdata->sync_pulse_amplitude & 0x7F;
	else
		tw9990->config[TW9990_SYNCT_ID].val = 0xB8; /* default value */

	/* Miscellaneous control I: configure range limited for Luma and
	 * Chroma, output data range are limited to 16~235 for Y and
	 * 16~240 for Cb/Cr.
	 */
	tw9990->config[TW9990_MISC1_CTRL_1_ID].reg = TW9990_MISC1_CTRL_1;
	tw9990->config[TW9990_MISC1_CTRL_1_ID].val = 0x1C;
}

static int tw9990_enable(struct tw9990 *tw9990)
{
	return tw9990_write_reg_array(tw9990,
			       tw9990->config,
			       ARRAY_SIZE(tw9990->config));
}

static int tw9990_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tw9990 *tw9990 = to_tw9990(sd);
	int ret = 0;

	if (enable)
		ret = tw9990_enable(tw9990);

	tw9990->is_streaming = !!enable;

	return ret;
}

static int tw9990_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *fi)
{
	struct tw9990 *tw9990 = to_tw9990(sd);
	if (!fi)
		return -EINVAL;

	fi->interval.numerator = tw9990->frame_interval.interval.numerator;
	fi->interval.denominator = tw9990->frame_interval.interval.denominator;
	return 0;
}

static int tw9990_query_dv_timings(struct v4l2_subdev *sd,
				   struct v4l2_dv_timings *timings)
{
	struct tw9990 *tw9990 = to_tw9990(sd);
	unsigned char std_sel, std, status;

	std_sel = tw9990_i2c_read_byte(tw9990,
				       TW9990_STD_SEL);
	while((std_sel & 0x80) != 0) {
		v4l2_info(sd, "waiting for std detection\n");
		msleep(500);
		std_sel = tw9990_i2c_read_byte(tw9990,
				       TW9990_STD_SEL);
	}

	status = tw9990_i2c_read_byte(tw9990,
				      TW9990_CHIP_STATUS);
	if(status & 0x80) {
		v4l2_info(sd, "Video loss\n");
		return -ENOLINK;
	}

	std = (std_sel & 0x70) >> 4;

	switch(std) {
	case 0:
	case 3:
	case 4:
		*timings = tw9990_ntsc_timings;
		/*pclk doubled for 8 bits transmission of YUYV
		  so horizontal timings are doubled*/
		timings->bt.pixelclock = 2 * timings->bt.pixelclock;
		timings->bt.hfrontporch = 2 * timings->bt.hfrontporch;
		timings->bt.hsync = 2 * timings->bt.hsync;
		timings->bt.hbackporch = 2 * timings->bt.hbackporch;
		break;
	case 1:
	case 2:
	case 5:
	case 6:
		*timings = tw9990_pal_timings;
		/*pclk doubled for 8 bits transmission of YUYV
		  so horizontal timngs are doubled*/
		timings->bt.pixelclock = 2 * timings->bt.pixelclock;
		timings->bt.hfrontporch = 2 * timings->bt.hfrontporch;
		timings->bt.hsync = 2 * timings->bt.hsync;
		timings->bt.hbackporch = 2 * timings->bt.hbackporch;
		break;
	case 7:
		v4l2_info(sd, "no standard detected\n");
		memset(timings, 0, sizeof(*timings));
		break;
	default:
		v4l2_err(sd, "invalid value for register 0x%X:0x%X\n",
				      TW9990_STD_SEL, std_sel);
		return -EPIPE;
	}
	return 0;
}

int tw9990_s_routing(struct v4l2_subdev *sd, u32 in, u32 out, u32 config) {
	struct tw9990 *tw9990 = to_tw9990(sd);

	if(!tw9990 || in >= TW9990_NUM_INPUTS)
		return -EINVAL;
	/* Clean-up input selection (b11110011) */
	tw9990->config[TW9990_IN_FORM_ID].val &= ~(TW_MASK(2) << 2);

	/* Setup new input selection: based on tw9990 data-sheet
	 * In differential mode YSEL take only 0 or 2 respectively
	 * MUX0+/- or MUX2+/-
	 * In single ended mode YSEL take only 0 or 1 respectively
	 * MUX0 or MUX1... for MUX3 or MUX4, who knows ?
	 */
	if (tw9990->differential_input)
		tw9990->config[TW9990_IN_FORM_ID].val |= (in << 3);
	else
		tw9990->config[TW9990_IN_FORM_ID].val |= (in << 2);
	return 0;
}

int tw9990_subscribe_event(struct v4l2_subdev *subdev,
			   struct v4l2_fh *fh,
			   struct v4l2_event_subscription *sub)
{
	return v4l2_src_change_event_subscribe(fh, sub);
}

int tw9990_unsubscribe_event(struct v4l2_subdev *subdev,
			     struct v4l2_fh *fh,
			     struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static const struct v4l2_subdev_core_ops tw9990_core_ops = {
	.subscribe_event = tw9990_subscribe_event,
	.unsubscribe_event = tw9990_unsubscribe_event,
};

static const struct v4l2_subdev_video_ops tw9990_video_ops = {
	.s_stream = tw9990_s_stream,
	.g_frame_interval = tw9990_g_frame_interval,
	.query_dv_timings = tw9990_query_dv_timings,
	.s_routing = tw9990_s_routing,
};

static const struct v4l2_subdev_pad_ops tw9990_pad_ops = {
	.get_fmt = tw9990_get_fmt,
	.set_fmt = tw9990_set_fmt,
	.enum_mbus_code = tw9990_enum_mbus_code,
};

static const struct v4l2_subdev_ops tw9990_ops = {
	.core = &tw9990_core_ops,
	.video = &tw9990_video_ops,
	.pad = &tw9990_pad_ops,
};

static irqreturn_t tw9990_isr(int irq, void *priv)
{
	struct tw9990 *tw9990 = priv;
	unsigned char int2status;
	struct video_device *vdev = tw9990->sd.devnode;
	static const struct v4l2_event event = {
		.type = V4L2_EVENT_SOURCE_CHANGE,
		.u.src_change.changes =
		        V4L2_EVENT_SRC_CH_RESOLUTION,
	};

	int2status = tw9990_i2c_read_byte(tw9990,
					  TW9990_INT2_STATUS);

	v4l2_info(&tw9990->sd,
		 "read int2status = 0x%X\n", int2status);

	if(vdev)
		v4l2_event_queue(vdev, &event);
	tw9990_i2c_write_byte(tw9990, TW9990_STD_RECOG, 0xFF);
	tw9990_i2c_write_byte(tw9990, TW9990_INT2_RST, 0xFF);
	return IRQ_HANDLED;
}

static int tw9990_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct tw9990 *tw9990;
	struct tw9990_platform_data *pdata = NULL;
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
	v4l2_dbg(1, debug, client, "detecting tw9990 client on address 0x%x\n",
			client->addr << 1);

	tw9990 = kzalloc(sizeof(*tw9990), GFP_KERNEL);
	if(!tw9990) {
		ret = -ENOMEM;
		v4l2_err(client, "alloc failed for data structure\n");
		goto enomem;
	}

	tw9990->set_power = pdata->set_power;
	tw9990->differential_input = pdata->differential_input;
	tw9990->i2c_client = client;
	tw9990_setup(tw9990, pdata);

	format = &tw9990->format;
	format->width = TW9990_CVBS_WIDTH;
	format->height = TW9990_CVBS_PAL_HEIGHT;
	format->field = V4L2_FIELD_INTERLACED;
	format->code = V4L2_MBUS_FMT_UYVY8_2X8;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;

	tw9990->frame_interval.interval.numerator = 1;
	tw9990->frame_interval.interval.denominator = 30;

	sd = &tw9990->sd;
	v4l2_i2c_subdev_init(sd, client, &tw9990_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	tw9990->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&sd->entity, 1, &tw9990->pad, 0);
	if(ret < 0) {
		v4l2_err(client, "failed to init media entity\n");
		goto emedia;
	}

	/*see if chip is present */
	if(pdata->set_power) {
		pdata->set_power(TW9990_POWER_ON);
		msleep(500);
	}

	reg.reg = 0x00;

	ret = tw9990_i2c_read_byte(tw9990, reg.reg);
	/* not chip */
	if (ret < 0)
		goto echipident;

	reg.val = ret;

	if(reg.val != 0) {
		v4l2_err(client, "read 0x%d in chip ident reg\n", reg.val);
		ret = -ENODEV;
		goto echipident;
	}

	if(client->irq > 0) {
		ret = request_threaded_irq(client->irq,NULL,
				tw9990_isr,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"tw9990", tw9990);
		if(ret < 0) {
			v4l2_err(client, "failed to register irq\n");
			client->irq = -1;
		}
	}

	tw9990_i2c_write_byte(tw9990, TW9990_INT2_RST, 0xFF);
	tw9990_i2c_write_byte(tw9990, TW9990_INT2_ENABLE, 0x30);
	tw9990_i2c_write_byte(tw9990, TW9990_SHORT_DIAG, 0x0);
	tw9990_i2c_write_byte(tw9990, TW9990_STD_RECOG, 0xFF);
	tw9990_enable(tw9990);

	return 0;
echipident:
	media_entity_cleanup(&sd->entity);
emedia:
	kfree(tw9990);
enomem:
ei2c:
	return ret;
}

static int tw9990_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw9990 *tw9990 = to_tw9990(sd);
	struct tw9990_platform_data *pdata = NULL;

	pdata = client->dev.platform_data;

	if(tw9990->set_power)
		tw9990->set_power(TW9990_POWER_OFF);

	if(client->irq > 0)
		free_irq(client->irq, tw9990);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	kfree(tw9990);
	return 0;
}

#ifdef CONFIG_PM
int tw9990_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw9990 *tw9990 = to_tw9990(sd);

	// Saving the state to restore at resume
	int is_streaming = tw9990->is_streaming;

	// Stop if it was streaming
	if (is_streaming)
		tw9990_s_stream(sd, 0);

	tw9990->is_streaming = is_streaming;
	if(tw9990->set_power)
		tw9990->set_power(TW9990_POWER_OFF);

	return 0;
}

int tw9990_resume(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tw9990 *tw9990 = to_tw9990(sd);

	if(tw9990->set_power)
		tw9990->set_power(TW9990_POWER_ON);

	if (tw9990->is_streaming)
		tw9990_s_stream(sd, 1);
	return 0;
}
#endif

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id tw9990_id[] = {
	{ "tw9990", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tw9990_id);

static struct i2c_driver tw9990_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "tw9990",
	},
	.probe = tw9990_probe,
	.remove = tw9990_remove,
#ifdef CONFIG_PM
	.suspend = tw9990_suspend,
	.resume = tw9990_resume,
#endif
	.id_table = tw9990_id,
};

module_i2c_driver(tw9990_driver);
