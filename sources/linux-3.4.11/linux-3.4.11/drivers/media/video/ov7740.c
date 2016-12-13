/* ov7740.c
 *
 * Driver for Omnivision 7740 camera
 *
 * Author : Maxime JOURDAN <maxime.jourdan@parrot.com>
 *
 * Date : 10/10/2014
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
#include <media/ov7740.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/v4l2-dv-timings.h>
#include <media/v4l2-event.h>

#define VGA_WIDTH 640
#define VGA_HEIGHT 480
#define VGA_FPS 30

#define QVGA_WIDTH 320
#define QVGA_HEIGHT 240
#define QVGA_FPS 60

#define OV7740_ID_MSB 0x77
#define OV7740_ID_LSB 0x42

/* Registers */
#define OV7740_GAIN 0x00
#define OV7740_BGAIN 0x01
#define OV7740_RGAIN 0x02
#define OV7740_GGAIN 0x03
#define OV7740_AEC_MSB 0x0F
#define OV7740_AEC_LSB 0x10
#define OV7740_DIG_GAIN 0x15

MODULE_AUTHOR("Maxime JOURDAN <maxime.jourdan@parrot.com>");
MODULE_DESCRIPTION("Omnivision OV7740 VGA camera sensor driver");
MODULE_LICENSE("GPL");

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-2)");

#define SUB_ADDRESS        0x38
#define YAVG_CONFIG        0xE9

struct ov7740 {
	struct v4l2_subdev sd;
	struct i2c_client *i2c_client;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler       ctrl_handler;
	struct v4l2_ctrl              *hflip;
	struct v4l2_ctrl              *vflip;
	struct v4l2_ctrl              *expo_window;

	int is_streaming;
	int (*set_power)(int on);
};

static inline struct ov7740 *to_ov7740(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov7740, sd);
}

/* Register maps */
struct reg_cfg {
	unsigned char reg;
	unsigned char val;
};

// Omnivision Application Note defaults
static struct reg_cfg ov7740_cfg[] = {
	{ 0x11, 0x00 },
	{ 0x12, 0x20 },
	{ 0x17, 0x25 },
	{ 0x18, 0xA1 },
	{ 0x19, 0x03 },
	{ 0x1B, 0x89 },
	{ 0x22, 0x03 },
	{ 0x31, 0xA1 },
	{ 0x32, 0xF0 },
	{ 0x33, 0xC4 },
	{ 0x36, 0x3F },
	{ 0x04, 0x60 },
	{ 0x40, 0x7F },
	{ 0x41, 0x6A },
	{ 0x42, 0x29 },
	{ 0x44, 0xE5 },
	{ 0x45, 0x41 },
	{ 0x47, 0x42 },
	{ 0x48, 0x00 },
	{ 0x49, 0x61 },
	{ 0x64, 0x00 },
	{ 0x67, 0x88 },
	{ 0x68, 0x1A },
	{ 0x26, 0x72 },
	{ 0x53, 0x02 },
	{ 0x55, 0xC0 },
	{ 0x56, 0x55 },
	{ 0x57, 0xFF },
	{ 0x58, 0xFF },
	{ 0x59, 0xFF },
	{ 0x80, 0x00 },
	{ 0x81, 0x3F },
	{ 0x82, 0x32 },
	{ 0x83, 0x01 },
	{ 0x38, 0x11 },
	{ 0x84, 0x70 },
	{ 0x85, 0x00 },
	{ 0x86, 0x03 },
	{ 0x87, 0x01 },
	{ 0x88, 0x05 },
	{ 0x70, 0x00 },
	{ 0x71, 0x34 },
	{ 0x74, 0x28 },
	{ 0x75, 0x98 },
	{ 0x76, 0x00 },
	{ 0x77, 0x08 },
	{ 0x78, 0x01 },
	{ 0x79, 0xC2 },
	{ 0x14, 0x28 },
	{ 0x50, 0x97 },
	{ 0x51, 0x7E },
	{ 0x52, 0x00 },
	{ 0x20, 0x00 },
	{ 0x21, 0x23 },
	{ 0x56, 0xC3 },
	{ 0x57, 0xEB },
	{ 0x58, 0xEB },
	{ 0x59, 0xC3 },
	{ 0x27, 0x00 },
	{ 0x24, 0x88 },
	{ 0x89, 0x30 },
	{ 0x93, 0x38 },
	{ 0x95, 0x85 },
	{ 0x99, 0x33 },
	{ 0x24, 0x44 },
	{ 0x25, 0x33 },
	{ 0x14, 0x28 },
	{ 0x13, 0xBF },
	{ 0x52, 0x00 },
	{ 0x50, 0x97 },
	{ 0x51, 0x7E },
	{ 0x80, 0x7F },
};

static inline struct ov7740 *ctrl_to_ov7740(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ov7740, ctrl_handler);
}

/* i2c transfer functions */
static s32 ov7740_i2c_read_byte(struct ov7740 *ov7740,
				u8 command)
{
	struct i2c_client *client = ov7740->i2c_client;
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	else
		return -EIO;
}

static s32 ov7740_i2c_write_byte(struct ov7740 *ov7740,
				 u8 command,
				 u8 value)
{
	struct i2c_client *client = ov7740->i2c_client;
	union i2c_smbus_data data;
	int err;
	int i;

	data.byte = value;
	for (i = 0; i < 3; i++) {
		err = i2c_smbus_write_byte_data(client,
				command,
				value);
		if (!err)
			break;
	}
	if (err < 0)
		v4l2_err(client, "error writing %02x, %02x, %02x\n",
				client->addr, command, value);
	return err;
}

static int ov7740_write_reg_array(struct ov7740 *ov7740,
				  struct reg_cfg *array, int nb)
{
	s32 ret;
	int i;
	struct v4l2_subdev *sd = &ov7740->sd;

	for(i=0;i<nb;i++) {
		ret = ov7740_i2c_write_byte(ov7740,
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
__ov7740_get_fmt(struct ov7740 *ov7740,
		 struct v4l2_subdev_fh *fh,
		 unsigned int pad,
		 enum v4l2_subdev_format_whence which)
{
	if(pad != 0)
		return NULL;

	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &ov7740->format;
}

static int ov7740_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov7740 *ov7740 = to_ov7740(sd);
	struct v4l2_mbus_framefmt *format;

	format = __ov7740_get_fmt(ov7740, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int ov7740_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			  struct v4l2_subdev_format *fmt)
{
	struct ov7740 *ov7740 = to_ov7740(sd);
	int ret = 0;
	struct v4l2_mbus_framefmt *format;

	format = __ov7740_get_fmt(ov7740, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	// OV7740 only supports VGA or QVGA
	if(!(fmt->format.width == 640 && fmt->format.height == 480) &&
	   !(fmt->format.width == 320 && fmt->format.height == 240))
		return -EINVAL;

	*format = fmt->format;
	return ret;
}

static int ov7740_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov7740 *ov7740 = to_ov7740(sd);

	if (code->index)
		return -EINVAL;

	code->code = ov7740->format.code;
	return 0;
}

static int ov7740_apply_flip(struct ov7740 *ov7740) {
	u8 val = 0x12;

	if (ov7740->hflip->val) {
		val |= 0x40;
		ov7740_i2c_write_byte(ov7740, 0x16, 0x01);
	} else {
		ov7740_i2c_write_byte(ov7740, 0x16, 0x00);
	}

	if (ov7740->vflip->val)
		val |= 0x80;

	return ov7740_i2c_write_byte(ov7740, 0x0C, val);
}


static int set_sub_address(struct ov7740 *ov7740, u8 sub)
{
	u8 sub_address;
	int ret;

	ret = ov7740_i2c_read_byte(ov7740, SUB_ADDRESS);
	if (ret < 0) {
		return ret;
	}

	sub_address = ret & 0xff;

	sub_address &= 0xf0;
	sub_address |= sub & 0xf;

	return ov7740_i2c_write_byte(ov7740, SUB_ADDRESS, sub_address);
}

static int ov7740_apply_expo_window(struct ov7740 *ov7740)
{
	u16			*window	  = ov7740->expo_window->p_new.p_u16;
	u16                      x, y, w, h;
	u8                       reg;
	int			 ret;

	x = window[0];
	y = window[1];
	w = window[2];
	h = window[3];

	if ((x | y | w | h) == 0) {
		/* When the window is 0 we switch to automatic mode */
		ret = set_sub_address(ov7740, 4);
		if (ret < 0) {
			return ret;
		}

		return ov7740_i2c_write_byte(ov7740, YAVG_CONFIG, 0);
	}

	/* Build YAVG_CTRL register value */
	reg = 1 << 5;                    /* Manual mode */
	reg |= ((y >> 8) & 1) << 4;      /* YAGV YOFFSET MSB */
	reg |= ((x >> 8) & 3);           /* YAGV XOFFSET MSB */

	ret = set_sub_address(ov7740, 4);
	if (ret < 0) {
		return ret;
	}

	ret = ov7740_i2c_write_byte(ov7740, YAVG_CONFIG, reg);
	if (ret < 0) {
		return ret;
	}

	/* Set YAVG XOFFSET LSB */
	ret = set_sub_address(ov7740, 2);
	if (ret < 0) {
		return ret;
	}

	ret = ov7740_i2c_write_byte(ov7740, YAVG_CONFIG, x & 0xff);
	if (ret < 0) {
		return ret;
	}

	/* Set YAVG YOFFSET LSB */
	ret = set_sub_address(ov7740, 3);
	if (ret < 0) {
		return ret;
	}

	ret = ov7740_i2c_write_byte(ov7740, YAVG_CONFIG, y & 0xff);
	if (ret < 0) {
		return ret;
	}

	/* Set YAVG HSIZE */
	ret = set_sub_address(ov7740, 5);
	if (ret < 0) {
		return ret;
	}

	ret = ov7740_i2c_write_byte(ov7740, YAVG_CONFIG, (w / 8) & 0x7f);
	if (ret < 0) {
		return ret;
	}

	/* Set YAVG VSIZE */
	ret = set_sub_address(ov7740, 6);
	if (ret < 0) {
		return ret;
	}

	ret = ov7740_i2c_write_byte(ov7740, YAVG_CONFIG, (h / 4) & 0x7f);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int ov7740_enable(struct ov7740 *ov7740)
{
	int ret;

	// reset
	ret = ov7740_i2c_write_byte(ov7740, 0x12, 0x80);
	if(ret < 0)
		return ret;

	msleep(10);
	ov7740_write_reg_array(ov7740,
			       ov7740_cfg,
			       ARRAY_SIZE(ov7740_cfg));

	ov7740_apply_flip(ov7740);
	ov7740_apply_expo_window(ov7740);
	return 0;
}

static int ov7740_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov7740 *ov7740 = to_ov7740(sd);
	int ret = 0;

	if(enable)
		ret = ov7740_enable(ov7740);

	if(ret < 0)
		return ret;

	ov7740->is_streaming = !!enable;

	return ret;
}


#define V4L2_CID_OV7740_RGAIN       (V4L2_CID_CAMERA_CLASS_BASE + 0x100)
#define V4L2_CID_OV7740_GGAIN       (V4L2_CID_CAMERA_CLASS_BASE + 0x101)
#define V4L2_CID_OV7740_BGAIN       (V4L2_CID_CAMERA_CLASS_BASE + 0x102)
#define V4L2_CID_OV7740_EXPO_WINDOW (V4L2_CID_CAMERA_CLASS_BASE + 0x103)

static int ov7740_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov7740 *ov7740 = ctrl_to_ov7740(ctrl);

	/* If not streaming, just keep interval structures up-to-date */
	if (!ov7740->is_streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		return ov7740_apply_flip(ov7740);
	case V4L2_CID_OV7740_EXPO_WINDOW:
		return ov7740_apply_expo_window(ov7740);
	}

	return 0;
}

#define BIT_X(b, i) ((b >> i) & 0x01)

static int ov7740_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov7740 *ov7740 = ctrl_to_ov7740(ctrl);
	u8 aec_lsb, aec_msb;
	u8 again, dgain;

	/* If not streaming, just default value */
	if (!ov7740->is_streaming) {
		ctrl->val = ctrl->default_value;
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		again = ov7740_i2c_read_byte(ov7740, OV7740_GAIN);
		dgain = ov7740_i2c_read_byte(ov7740, OV7740_DIG_GAIN);
		ctrl->val = (BIT_X(dgain, 1) + 1) *
			    (BIT_X(dgain, 0) + 1) *
			    (BIT_X(again, 7) + 1) *
			    (BIT_X(again, 6) + 1) *
			    (BIT_X(again, 5) + 1) *
			    (BIT_X(again, 4) + 1) *
			    ((again & 0x0F) + 16);
		break;
	case V4L2_CID_OV7740_RGAIN:
		ctrl->val = ov7740_i2c_read_byte(ov7740, OV7740_RGAIN);
		break;
	case V4L2_CID_OV7740_GGAIN:
		ctrl->val = ov7740_i2c_read_byte(ov7740, OV7740_GGAIN);
		break;
	case V4L2_CID_OV7740_BGAIN:
		ctrl->val = ov7740_i2c_read_byte(ov7740, OV7740_BGAIN);
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		aec_msb = ov7740_i2c_read_byte(ov7740, OV7740_AEC_MSB);
		aec_lsb = ov7740_i2c_read_byte(ov7740, OV7740_AEC_LSB);
		ctrl->val = (aec_msb << 8) + aec_lsb;
		break;
	}

	return 0;
}

static const struct v4l2_subdev_video_ops ov7740_video_ops = {
	.s_stream = ov7740_s_stream,
};

static const struct v4l2_subdev_pad_ops ov7740_pad_ops = {
	.get_fmt = ov7740_get_fmt,
	.set_fmt = ov7740_set_fmt,
	.enum_mbus_code = ov7740_enum_mbus_code,
};

static const struct v4l2_subdev_ops ov7740_ops = {
	.video = &ov7740_video_ops,
	.pad = &ov7740_pad_ops,
};

static const struct v4l2_ctrl_ops ov7740_ctrl_ops = {
	.s_ctrl          = ov7740_s_ctrl,
	.g_volatile_ctrl = ov7740_g_volatile_ctrl,
};

static const struct v4l2_ctrl_config ov7740_ctrl_rgain = {
	.ops  = &ov7740_ctrl_ops,
	.id   = V4L2_CID_OV7740_RGAIN,
	.name = "Red gain",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 1,
	.max  = 255,
	.step = 1,
	.def  = 100,
};

static const struct v4l2_ctrl_config ov7740_ctrl_ggain = {
	.ops  = &ov7740_ctrl_ops,
	.id   = V4L2_CID_OV7740_GGAIN,
	.name = "Green gain",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 1,
	.max  = 255,
	.step = 1,
	.def  = 100,
};

static const struct v4l2_ctrl_config ov7740_ctrl_bgain = {
	.ops  = &ov7740_ctrl_ops,
	.id   = V4L2_CID_OV7740_BGAIN,
	.name = "Blue gain",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min  = 1,
	.max  = 255,
	.step = 1,
	.def  = 100,
};

static const struct v4l2_ctrl_config ov7740_ctrl_expo_window = {
	.ops  = &ov7740_ctrl_ops,
	.id   = V4L2_CID_OV7740_EXPO_WINDOW,
	.name = "Exposure Average Window: { x, y, width, height }",
	.type = V4L2_CTRL_TYPE_U16,
	.min  = 0,
	.max  = 0xffff,
	.step = 1,
	.def  = 0,
	.dims = { 4 },
};

static int ov7740_init_ctrls(struct ov7740* ov7740)
{
	struct v4l2_ctrl_handler *hdl = &ov7740->ctrl_handler;
	struct v4l2_ctrl         *ctrl;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 7);
	if (ret < 0) {
		v4l2_err(&ov7740->sd, "failed to init ctrl handler\n");
		goto einit;
	}

	ov7740->hflip = v4l2_ctrl_new_std(hdl,
					    &ov7740_ctrl_ops,
					    V4L2_CID_HFLIP,
					    0, 1, 1, 0);

	ov7740->vflip = v4l2_ctrl_new_std(hdl,
					    &ov7740_ctrl_ops,
					    V4L2_CID_VFLIP,
					    0, 1, 1, 0);

	/* Exposure window manual configuration */
	ov7740->expo_window = v4l2_ctrl_new_custom(hdl,
						   &ov7740_ctrl_expo_window,
						   NULL);

	ctrl = v4l2_ctrl_new_std(hdl, &ov7740_ctrl_ops, V4L2_CID_EXPOSURE_ABSOLUTE, 0, 0xFFFF, 1, 0);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_std(hdl, &ov7740_ctrl_ops, V4L2_CID_GAIN, 0, 1984, 1, 0);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_custom(hdl, &ov7740_ctrl_rgain, NULL);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_custom(hdl, &ov7740_ctrl_ggain, NULL);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_custom(hdl, &ov7740_ctrl_bgain, NULL);
	ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (hdl->error) {
		v4l2_err(&ov7740->sd, "failed to add new ctrls\n");
		ret = hdl->error;
		goto ectrl;
	}

	ov7740->sd.ctrl_handler = hdl;

	return 0;

ectrl:
	v4l2_ctrl_handler_free(hdl);
einit:
	return ret;
}

static int ov7740_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov7740 *ov7740;
	struct ov7740_platform_data *pdata = NULL;
	struct v4l2_subdev *sd;
	int ret = -ENODEV;
	struct v4l2_mbus_framefmt *format;
	pdata = client->dev.platform_data;

	if(!pdata)
		return -EINVAL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}

	ov7740 = kzalloc(sizeof(*ov7740), GFP_KERNEL);
	if(!ov7740) {
		ret = -ENOMEM;
		v4l2_err(client, "alloc failed for data structure\n");
		goto enomem;
	}

	ov7740->set_power = pdata->set_power;
	ov7740->i2c_client = client;

	format = &ov7740->format;
	format->width = VGA_WIDTH;
	format->height = VGA_HEIGHT;
	format->field = V4L2_FIELD_NONE;
	format->code = V4L2_MBUS_FMT_YUYV8_2X8;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;

	sd = &ov7740->sd;
	v4l2_i2c_subdev_init(sd, client, &ov7740_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov7740->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	/*see if chip is present */
	if (pdata->set_power) {
		pdata->set_power(1);
		msleep(20);
	}

	ret = ov7740_i2c_read_byte(ov7740, 0x0A);
	if (ret < 0 || ret != OV7740_ID_MSB) {
		v4l2_err(sd, "ID MSB is %02X instead of %02X\n", ret, OV7740_ID_MSB);
		goto echipident;
	}

	ret = ov7740_i2c_read_byte(ov7740, 0x0B);
	if (ret < 0 || ret != OV7740_ID_LSB) {
		v4l2_err(sd, "ID LSB is %02X instead of %02X\n", ret, OV7740_ID_LSB);
		goto echipident;
	}

	ret = ov7740_init_ctrls(ov7740);
	if (ret)
		goto echipident;

	// ov7740_enable(ov7740);
	ret = media_entity_init(&sd->entity, 1, &ov7740->pad, 0);
	if(ret < 0) {
		v4l2_err(sd, "failed to init media entity\n");
		goto emedia;
	}

	v4l2_info(sd, "Found ov7740 sensor\n");

	return 0;
echipident:
	v4l2_err(sd, "Failed to identify OV7740 camera\n");
emedia:
	kfree(ov7740);
	v4l2_device_unregister_subdev(sd);
enomem:
ei2c:
	return ret;
}

static int ov7740_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7740 *ov7740 = to_ov7740(sd);
	struct ov7740_platform_data *pdata = NULL;
	struct v4l2_ctrl_handler *hdl = &ov7740->ctrl_handler;

	pdata = client->dev.platform_data;

	if(ov7740->set_power)
		ov7740->set_power(0);

	if(client->irq > 0)
		free_irq(client->irq, ov7740);

	v4l2_ctrl_handler_free(hdl);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	kfree(ov7740);
	return 0;
}

#ifdef CONFIG_PM
int ov7740_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7740 *ov7740 = to_ov7740(sd);

	// Saving the state to restore at resume
	int is_streaming = ov7740->is_streaming;

	// Stop if it was streaming
	if (is_streaming)
		ov7740_s_stream(sd, 0);

	ov7740->is_streaming = is_streaming;
	if(ov7740->set_power)
		ov7740->set_power(0);

	return 0;
}

int ov7740_resume(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov7740 *ov7740 = to_ov7740(sd);

	if(ov7740->set_power)
		ov7740->set_power(1);

	if (ov7740->is_streaming)
		ov7740_s_stream(sd, 1);
	return 0;
}
#endif

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id ov7740_id[] = {
	{ "ov7740", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov7740_id);

static struct i2c_driver ov7740_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "ov7740",
	},
	.probe = ov7740_probe,
	.remove = ov7740_remove,
#ifdef CONFIG_PM
	.suspend = ov7740_suspend,
	.resume = ov7740_resume,
#endif
	.id_table = ov7740_id,
};

module_i2c_driver(ov7740_driver);
