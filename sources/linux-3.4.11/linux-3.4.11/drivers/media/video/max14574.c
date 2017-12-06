/*
 * drivers/media/video/max14574.c - MAX14574 Lens controller driver
 *
 * Copyright (C) 2016 Parrot S.A.
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 */


#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <media/max14574.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>


/*
 * struct max14574
 *
 * @subdev:		V4L2 subdev
 * @pdata:		Lens platform data
 * @power_lock:		Protects power_count
 * @power_count:	Power reference count
 */
struct max14574 {
	struct v4l2_subdev             subdev;
	struct max14574_platform_data  *pdata;

	struct mutex                   power_lock;
	int                            power_count;

	/* Controls */
	struct v4l2_ctrl_handler       ctrls;

	struct v4l2_ctrl              *position;
	struct v4l2_ctrl              *get_status_reg;
	struct v4l2_ctrl              *get_m_status_reg;
};

#define to_max14574(sd) container_of(sd, struct max14574, subdev)

/* Return negative errno else a data byte received from the device. */
static int max14574_read(struct max14574 *max14574, u8 addr)
{
	struct i2c_client *client = v4l2_get_subdevdata(&max14574->subdev);
	int rval;

	rval = i2c_smbus_read_byte_data(client, addr);

	if (rval < 0)
		dev_warn(&client->dev,
				"Failed to read register 0x%02x.\n", addr);

	dev_dbg(&client->dev, "Read Addr:%02X Val:%02X %s\n", addr, rval,
		rval < 0 ? "fail" : "ok");

	return rval;
}

/* Return negative errno else zero on success */
static int max14574_write(struct max14574 *max14574, u8 addr, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&max14574->subdev);
	int rval;

	rval = i2c_smbus_write_byte_data(client, addr, val);

	if (rval < 0)
		dev_warn(&client->dev,
				"Failed to write register 0x%02x.\n", addr);

	dev_dbg(&client->dev, "Write Addr:%02X Val:%02X %s\n", addr, val,
		rval < 0 ? "fail" : "ok");

	return rval;
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

/* Read the FAIL register and print failure */
static int max14574_print_failure(struct max14574 *max14574)
{
	struct i2c_client *client = v4l2_get_subdevdata(&max14574->subdev);
	int status = 0x01;
	int rval = 0;

	/* Write CHK_FAIL bit for checking status */
	rval = max14574_write(max14574, CMND, CMND_CHK_FAIL);

	if (rval < 0)
	  return rval;

	/* Waiting the end of test */
	while ((status & CMND_CHK_FAIL) != 0)
		status = max14574_read(max14574, CMND);

	/* Get the result of the test by reading FAIL register */
	status = max14574_read(max14574, FAIL);

	if (status & FAIL_OP)
		dev_info(&client->dev, "Open connection detected between the outputs\n");

	if (status & FAIL_SH)
		dev_info(&client->dev, "Short-circuit connection detected between the outputs\n");

	if (status & FAIL_COM_FAIL)
		dev_info(&client->dev, "Fail connection detected on the C1 bump (COM)\n");

	if (status & FAIL_LL4_FAIL)
		dev_info(&client->dev, "Fail connection detected on the C5 bump (LL4)\n");

	return status;
}

/* Read the M_STATUS register and print the value of this register */
static int max14574_printk_Mstatus(struct max14574 *max14574)
{
	struct i2c_client *client = v4l2_get_subdevdata(&max14574->subdev);

	int status = max14574_read(max14574, M_STATUS);

	if (status & STATUS_LL_TH)
		dev_info(&client->dev, "Thermal shutdown event has occurred\n");

	if (status & STATUS_BST_FAIL)
		dev_info(&client->dev, "B5 bump is not able to reach the Vpeak voltage\n");

	/* Keep only bit4 (LL_TH) and bit2 (BST_FAIL), other bits are useless */
	status = status & 0b00010100;

	return status;
}

static int max14574_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max14574 *max14574 =
		container_of(ctrl->handler, struct max14574, ctrls);
	int value;
	int value_LSB;

	switch (ctrl->id) {

	case V4L2_CID_PRIVATE_MAX14574_CONTROL_POSITION:
		value_LSB = max14574_read(max14574, OIS_LSB);
		value_LSB = (value_LSB & 0b11000000) >> 6;
		value = max14574_read(max14574, LLV4);
		value = (value << 2) + value_LSB;

		if (value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;
	case V4L2_CID_PRIVATE_MAX14574_GET_INTERRUPT_STATUS:
		value = max14574_print_failure(max14574);

		if (value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;
	case V4L2_CID_PRIVATE_MAX14574_GET_MAIN_STATUS:
		value = max14574_printk_Mstatus(max14574);

		if (value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;
	}

	return 0;
}

/* Set the eight MSBs in LLV4 register and the two LSBs in OIS_LSB register */
static int max14574_set_voltage(struct max14574 *max14574)
{
	u16 *reg = max14574->position->p_new.p_u16;
	u16 Voltage = reg[0];
	int ret = 0;

	u8 Voltage_LSB;
	u8 OIS_LSB_value;
	u8 CMND_value;

	/* Check if the value is between 0 and 1023 (10bits) */
	if (Voltage > CHARGE_MAX_VOLT_MAX)
		Voltage = CHARGE_MAX_VOLT_MAX;
	else if (Voltage < CHARGE_MAX_VOLT_MIN)
		Voltage = CHARGE_MAX_VOLT_MIN;

	/* Obtain the two LSBs and the eight MSBs of the 10bits voltage */
	Voltage_LSB = (u8)(Voltage & 0x0003);
	OIS_LSB_value = Voltage_LSB<<6;
	Voltage = (u8)(Voltage>>2);

	CMND_value = CMND_UPD_OUT;

	/* Write values on registers */
	ret = max14574_write(max14574, OIS_LSB, OIS_LSB_value);
	if (ret < 0)
	  return ret;

	ret = max14574_write(max14574, LLV4, Voltage);
	if (ret < 0)
	  return ret;

	/* Update the value of output */
	ret = max14574_write(max14574, CMND, CMND_value);

	max14574->position->val = reg[0];
	max14574->position->cur.val = max14574->position->val;

	if (ret < 0)
	  return ret;
	else
	  return 0;
}

static int max14574_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max14574 *max14574 =
		container_of(ctrl->handler, struct max14574, ctrls);

	int ret;
	switch (ctrl->id) {
	case V4L2_CID_PRIVATE_MAX14574_CONTROL_POSITION:
		ret = max14574_set_voltage(max14574);
		return ret;
	}
	return 0;
}

static const struct v4l2_ctrl_ops max14574_ctrl_ops = {
	.g_volatile_ctrl = max14574_get_ctrl,
	.s_ctrl = max14574_set_ctrl,
};

/* Put device into know state. */
static int max14574_setup(struct max14574 *max14574)
{
	int ret;
	struct max14574_platform_data *pdata = max14574->pdata;
	struct i2c_client *client = v4l2_get_subdevdata(&max14574->subdev);
	u8 mode = pdata->mode;

	ret = max14574_write(max14574, USERMODE, mode);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in USERMODE Register/n");
		return ret;
	}
	/* Reset voltage's LSBs */
	ret = max14574_write(max14574, OIS_LSB, 0x00);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in OIS_LSB Register/n");
		return ret;
	}

	/* Reset voltage's MSBs */
	ret = max14574_write(max14574, LLV1, 0x00);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in LLV1 Register/n");
		return ret;
	}

	/* Reset voltage's MSBs */
	ret = max14574_write(max14574, LLV2, 0x00);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in LLV2 Register/n");
		return ret;
	}

	/* Reset voltage's MSBs */
	ret = max14574_write(max14574, LLV3, 0x00);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in LLV3 Register/n");
		return ret;
	}

	/* Reset voltage's MSBs */
	ret = max14574_write(max14574, LLV4, 0x00);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in LLV4 Register/n");
		return ret;
	}

	/* Set output value to zero */
	ret = max14574_write(max14574, CMND, CMND_UPD_OUT);
	if (ret < 0) {
		dev_info(&client->dev, "Writing Error in CMND Register/n");
		return ret;
	}

	return 0;
}

static int max14574_set_power(struct v4l2_subdev *sd, int on)
{
	struct max14574 *max14574 = to_max14574(sd);
	int ret = 0;

	mutex_lock(&max14574->power_lock);

	if (on) {
		ret = max14574_setup(max14574);
		if (ret < 0)
			goto done;
	}

	max14574->power_count += on ? 1 : -1;
	WARN_ON(max14574->power_count < 0);

done:
	mutex_unlock(&max14574->power_lock);
	return ret;
}

/* Power up the lens driver and Chip revision */
static int max14574_registered(struct v4l2_subdev *sd)
{
	struct max14574 *max14574 = to_max14574(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int rval;
	int version;

	rval = max14574_set_power(&max14574->subdev, 1);
	if (rval < 0)
		return rval;

	rval = max14574_read(max14574, M_STATUS);
	if (rval < 0) {
		dev_err(&client->dev,
				"MAX14574 not detected (model %d)\n", rval);
		rval = -ENODEV;
		goto power_off;
	}

	/* Get the revision value with M_STATUS[7;5] */
	version = (rval & 0b11100000) >> 5;
	dev_info(&client->dev, "MAX14574 Chip revision: %d\n", version);

power_off:
	if (rval < 0) {
		max14574_set_power(&max14574->subdev, 0);
		return rval;
	}

	return 0;
}

static int max14574_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return max14574_set_power(sd, 1);
}

static int max14574_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return max14574_set_power(sd, 0);
}


static const struct v4l2_subdev_internal_ops max14574_internal_ops = {
	.registered = max14574_registered,
	.open = max14574_open,
	.close = max14574_close,

};

static const struct v4l2_subdev_core_ops max14574_core_ops = {
	.s_power = max14574_set_power,
};

static const struct v4l2_subdev_ops max14574_ops = {
	.core = &max14574_core_ops,
};

/* -----------------------------------------------------------------------------
 *  I2C driver
 */
#define	max14574_suspend	NULL
#define	max14574_resume		NULL

static const struct v4l2_ctrl_config ctrl_private_position = {
	.id = V4L2_CID_PRIVATE_MAX14574_CONTROL_POSITION,
	.ops = &max14574_ctrl_ops,
	.name = "MAX14574 - Set/Get Position",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = CHARGE_MAX_VOLT_MIN,
	.max = CHARGE_MAX_VOLT_MAX,
	.step = 1,
	.def = CHARGE_MAX_VOLT_MIN,
	.flags = V4L2_CTRL_FLAG_VOLATILE,
};

static const struct v4l2_ctrl_config ctrl_private_get_status_register = {
	.id = V4L2_CID_PRIVATE_MAX14574_GET_INTERRUPT_STATUS,
	.ops = &max14574_ctrl_ops,
	.name = "MAX14574 - Status register",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0xff,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config ctrl_private_get_m_status_register = {
	.id = V4L2_CID_PRIVATE_MAX14574_GET_MAIN_STATUS,
	.ops = &max14574_ctrl_ops,
	.name = "MAX14574 - Main Status register",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0xff,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};


/*
 * max14574_init_controls - Create controls
 * @max14574: The lens controller
 *
 */
static int max14574_init_controls(struct max14574 *max14574)
{
	int ret = 0;
	ret = v4l2_ctrl_handler_init(&max14574->ctrls, 3);

	if (ret >= 0) {

		max14574->position =
		    v4l2_ctrl_new_custom(&max14574->ctrls,
					 &ctrl_private_position,
					 NULL);

		max14574->get_status_reg =
		    v4l2_ctrl_new_custom(&max14574->ctrls,
					 &ctrl_private_get_status_register,
					 NULL);

		max14574->get_m_status_reg =
		    v4l2_ctrl_new_custom(&max14574->ctrls,
					 &ctrl_private_get_m_status_register,
					 NULL);

		max14574->subdev.ctrl_handler = &max14574->ctrls;

		ret = max14574->ctrls.error;
	}

	return ret;

}

static int max14574_probe(struct i2c_client *client,
			 const struct i2c_device_id *devid)
{
	struct max14574 *max14574;
	struct max14574_platform_data *pdata = client->dev.platform_data;
	int ret;

	if (pdata == NULL) {
		dev_err(&client->dev, "Missing platform data\n");
		return -ENODEV;
	}

	max14574 = kzalloc(sizeof(*max14574), GFP_KERNEL);
	if (max14574 == NULL) {
		dev_err(&client->dev, "Alloc failed\n");
		return -ENOMEM;
	}

	max14574->pdata = pdata;

	if (pdata->voltage < CHARGE_MAX_VOLT_MIN)
		pdata->voltage = CHARGE_MAX_VOLT_MIN;

	if (pdata->voltage > CHARGE_MAX_VOLT_MAX) {
		dev_warn(&client->dev, "Charge voltage over 70V, overriding to 70V\n");
		pdata->voltage = CHARGE_MAX_VOLT_MAX;
	}

	v4l2_i2c_subdev_init(&max14574->subdev, client, &max14574_ops);
	max14574->subdev.internal_ops = &max14574_internal_ops;
	max14574->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = max14574_init_controls(max14574);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't init controls (%d)\n", ret);
		goto einitctrl;
	}

	ret = media_entity_init(&max14574->subdev.entity, 0, NULL, 0);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't init media entity (%d)\n", ret);
		goto emedia;
	}

	max14574->subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	mutex_init(&max14574->power_lock);
	ret = max14574_registered(&max14574->subdev);
	if (ret < 0)
		goto edetect;

	return 0;

edetect:
	max14574_set_power(&max14574->subdev, 0);
	media_entity_cleanup(&max14574->subdev.entity);
emedia:
	v4l2_ctrl_handler_free(&max14574->ctrls);
einitctrl:
	v4l2_device_unregister_subdev(&max14574->subdev);
	kfree(pdata);
	kfree(max14574);

	return ret;
}

static int __devexit max14574_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct max14574 *max14574 = to_max14574(subdev);

	if (!max14574)
	  return -EINVAL;

	v4l2_device_unregister_subdev(subdev);
	v4l2_ctrl_handler_free(&max14574->ctrls);
	media_entity_cleanup(&max14574->subdev.entity);
	mutex_destroy(&max14574->power_lock);
	kfree(max14574);

	return 0;
}

static const struct i2c_device_id max14574_id_table[] = {
	{ MAX14574_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max14574_id_table);

static const struct dev_pm_ops max14574_pm_ops = {
	.suspend = max14574_suspend,
	.resume = max14574_resume,
};

static struct i2c_driver max14574_i2c_driver = {
	.driver	= {
		.owner = THIS_MODULE,
		.name = MAX14574_NAME,
		.pm   = &max14574_pm_ops,
	},
	.probe	= max14574_probe,
	.remove	= __devexit_p(max14574_remove),
	.id_table = max14574_id_table,
};

module_i2c_driver(max14574_i2c_driver);

MODULE_AUTHOR("Julien Garnier <julien.garnier@sensefly.com>");
MODULE_DESCRIPTION("Lens driver for MAX14574");
MODULE_LICENSE("GPL");

