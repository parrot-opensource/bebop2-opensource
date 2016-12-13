#include <linux/i2c.h>
#include "ov16825_common.h"
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

inline struct ov16825_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov16825_info, sd);
}

int sensor_write(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int                ret;
	u8                 data[3];
	struct i2c_msg     msg = {
		.addr  = client->addr,
		.flags = 0,
		.len   = sizeof(data),
		.buf   = data,
	};

	data[0] =  reg >> 8;
	data[1] =  reg & 0xff;
	data[2] =  val;

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0) {
		v4l2_err(sd, "I2C write failed [%02x->%04x: %d]\n",
			 val, reg, ret);
		return ret;
	}

	return 0;
}

int sensor_write16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	int ret;
	ret = sensor_write(sd, reg, (val >> 8) & 0xFF);
	if (!ret)
		ret = sensor_write(sd, reg +1, val & 0xFF);
	return ret;
}

int sensor_read(struct v4l2_subdev *sd, u16 reg, u8 *val)
{
	int			 ret;
	struct i2c_client	*client = v4l2_get_subdevdata(sd);
	u8			 regbuf[2];
	u8			 valbuf[1];

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
		v4l2_err(sd, "I2C read failed [%04x: %d]\n", reg, ret);
		return ret;
	}

	*val = valbuf[0];

	return 0;
}

/*
 * Write a list of register settings;
 */
int sensor_write_array(struct v4l2_subdev *sd,
                              struct regval_list *regs,
                              int array_size)
{
	int i=0;

	if(!regs)
		return -EINVAL;

	while(i<array_size) {
		int ret;
		ret = sensor_write(sd, regs->addr, regs->data);
		if (ret) {
			dev_err(sd->v4l2_dev->dev, "[@0x%x] <- 0x%x error %d\n",
			        regs->addr, regs->data, ret);
			return ret;
		}
		i++;
		regs++;
	}
	return 0;
}
