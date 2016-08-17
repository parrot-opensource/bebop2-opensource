/*
 * drivers/media/video/as3636a.c - AS3636 flash controller driver
 *
 * Copyright (C) 2014 Parrot S.A.
 *
 * Contact: Maxime Jourdan <maxime.jourdan@parrot.com>
 *
 * Inspired from AS3645A driver made by Laurent Pinchart
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
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <media/as3636.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define REG_IC_INFO                0x00
#define REG_IC_VERSION             0x01
#define REG_MODULE_INFO            0x02
#define REG_USER_NVM_1             0x03

#define REG_CONTROL                0x04
#define CONTROL_AFSTROBE           (1 << 1)
#define CONTROL_MST                (1 << 2)
#define CONTROL_RESET              (1 << 6)
#define CONTROL_STANDBY            (1 << 7)

#define REG_INTERRUPT_MASK         0x05

#define REG_INTERRUPT_STATUS       0x06
#define INTERRUPT_STATUS_ILF	   1
#define INTERRUPT_STATUS_CTF	   (1 << 1)
#define INTERRUPT_STATUS_BCF	   (1 << 2)
#define INTERRUPT_STATUS_TOF	   (1 << 3)
#define INTERRUPT_STATUS_OTPF	   (1 << 4)
#define INTERRUPT_STATUS_AFF	   (1 << 5)
#define INTERRUPT_STATUS_READYF    (1 << 6)
#define INTERRUPT_STATUS_SSTF	   (1 << 7)

#define REG_XENON_CONTROL          0x07
#define XENON_CHARGE               1
#define XENON_READY                (1 << 1)
#define XENON_STROBE               (1 << 2)
#define XENON_CAR                  (1 << 3)

#define REG_XENON_CAR_INTERVAL     0x08
#define REG_XENON_CHARGE_TIME      0x09
#define REG_XENON_SST_PULSE_LENGTH 0x0A
#define REG_LIFE_TIME_MSB          0x0B
#define REG_LIFE_TIME_LSB          0x0C
#define REG_XENON_CONFIG_A         0x0D
#define REG_XENON_CONFIG_B         0x0E
#define REG_XENON_CONFIG_C         0x0F
#define REG_LED_CURRENT            0x10
#define REG_LED_CONTROL            0x11
#define REG_LED_CONFIG             0x12
#define REG_PASSWORD               0x13
#define REG_USER_NVM_2             0x14
#define REG_USER_NVM_3             0x15
#define REG_USER_NVM_4             0x16
#define REG_USER_NVM_5             0x17

#define EEPROM_PASSWORD 0x8A

#define CHARGE_MAX_VOLT_MIN 285 // 28.5 V
#define CHARGE_MAX_VOLT_MAX 348
#define DCDC_PEAK_MIN 250
#define DCDC_PEAK_MAX 400

const u8 REG_IS_EEPROM[] = {0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 1, 1};

/*
 * struct as3636
 *
 * @subdev:		V4L2 subdev
 * @pdata:		Flash platform data
 * @power_lock:		Protects power_count
 * @power_count:	Power reference count
 * @led_mode:		V4L2 flash LED mode
 * @timeout:		Flash timeout in microseconds
 * @strobe_source:	Flash strobe source (software or external)
 */
struct as3636 {
	struct v4l2_subdev             subdev;
	struct as3636_platform_data   *pdata;

	struct mutex                   power_lock;
	int                            power_count;

	/* Controls */
	struct v4l2_ctrl_handler       ctrls;

	enum v4l2_flash_led_mode       led_mode;
	unsigned int                   timeout;
	enum v4l2_flash_strobe_source  strobe_source;
	struct v4l2_ctrl              *source;
	struct v4l2_ctrl              *set_reg;
	struct v4l2_ctrl              *get_control_reg;
	struct v4l2_ctrl              *get_interrupt_status_reg;
	struct v4l2_ctrl              *get_xenon_control_reg;
};

#define to_as3636(sd) container_of(sd, struct as3636, subdev)

/* Return negative errno else a data byte received from the device. */
static int as3636_read(struct as3636 *flash, u8 addr)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);
	int rval;

	rval = i2c_smbus_read_byte_data(client, addr);

	if (rval < 0)
		dev_warn(&client->dev, "Failed to read register 0x%02x.\n", addr);

	dev_dbg(&client->dev, "Read Addr:%02X Val:%02X %s\n", addr, rval,
		rval < 0 ? "fail" : "ok");

	return rval;
}

/* Return negative errno else zero on success */
static int as3636_write(struct as3636 *flash, u8 addr, u8 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);
	int rval;

	// Wait if the EEPROM is busy with a previous request.
	while((rval = as3636_read(flash, REG_PASSWORD)) > 0)
		usleep_range(2000, 2000);

	if(rval < 0)
		return rval;

	// To unlock EEPROM registers, we must first write a password value in the corresponding register
	if(REG_IS_EEPROM[addr])
	{
		rval = as3636_write(flash, REG_PASSWORD, EEPROM_PASSWORD);
		if(rval < 0)
			return rval;
	}

	rval = i2c_smbus_write_byte_data(client, addr, val);

	if (rval < 0)
		dev_warn(&client->dev, "Failed to write register 0x%02x.\n", addr);

	dev_dbg(&client->dev, "Write Addr:%02X Val:%02X %s\n", addr, val,
		rval < 0 ? "fail" : "ok");

	return rval;
}

/* -----------------------------------------------------------------------------
 * V4L2 controls
 */

static int as3636_read_interrupt(struct as3636 *flash)
{
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);
	int rval;

	/* NOTE: reading register clears interrupt status */
	rval = as3636_read(flash, REG_INTERRUPT_STATUS);
	if (rval < 0)
		return rval;

	if (rval & INTERRUPT_STATUS_ILF)
		dev_info(&client->dev, "Indicator LED fault: "
			"Short circuit or open loop\n");

	if (rval & INTERRUPT_STATUS_CTF)
		dev_info(&client->dev, "Xenon Charger Transformer fault\n");

	if (rval & INTERRUPT_STATUS_BCF)
		dev_info(&client->dev, "DCDC Boost converter "
			"(VOUTBOOST) fault\n");

	if (rval & INTERRUPT_STATUS_TOF)
		dev_info(&client->dev, "Xenon Charger Timeout fault\n");

	if (rval & INTERRUPT_STATUS_OTPF)
		dev_info(&client->dev, "Over Temperature protection fault\n");

	if (rval & INTERRUPT_STATUS_AFF)
		dev_info(&client->dev, "Autofocus LED (VLED) fault\n");

	if (rval & INTERRUPT_STATUS_SSTF)
		dev_info(&client->dev, "Xenon Strobe Self test fault\n");

	return rval;
}

static int as3636_flash_test(struct as3636 *flash) {
	int ret;

	ret = as3636_read(flash, REG_XENON_CONTROL);

	if(ret < 0)
		return ret;

	if(!(ret & XENON_READY))
		return -EINVAL;

	return as3636_write(flash, REG_XENON_CONTROL, ret | XENON_STROBE);
}

/**
 * AS3636 self-test and diagnosis in case there is a problem
 */
static int as3636_self_test(struct as3636 *flash) {
	int ret, rval;

	as3636_read_interrupt(flash);
	ret = as3636_write(flash, REG_LED_CONTROL, 0x08); // Enable AF LED to go into active mode.
	if (ret < 0)
		return ret;

	ret = as3636_write(flash, REG_LED_CONTROL, 0x00); // Disable AF LED.
	if (ret < 0)
		return ret;

	rval = as3636_read(flash, REG_CONTROL);

	ret = as3636_write(flash, REG_CONTROL, rval | CONTROL_MST);
	if (ret < 0)
		return ret;

	while (((rval = as3636_read(flash, REG_CONTROL)) & CONTROL_MST) == CONTROL_MST)
		msleep(20);

	if (rval < 0)
		return rval;

	rval = as3636_read_interrupt(flash);
	if (rval < 0)
		return rval;

	/*if (rval != INTERRUPT_STATUS_READYF)
		return -EINVAL;*/

	return 0;
}

static int as3636_is_flash_ready(struct as3636* flash) {
	int val = as3636_read(flash, REG_XENON_CONTROL);
	if(val < 0)
		return val;

	return (val & XENON_READY) ? 1 : 0;
}

static int as3636_get_fstrobe_mode(struct as3636 *flash) {
	int val = as3636_read(flash, REG_CONTROL);
	if (val < 0)
		return val;
	if (val & CONTROL_AFSTROBE)
		return V4L2_FLASH_STROBE_SOURCE_SOFTWARE;
	else
		return V4L2_FLASH_STROBE_SOURCE_EXTERNAL;
}

static int as3636_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct as3636 *flash =
		container_of(ctrl->handler, struct as3636, ctrls);
	int value;

	switch (ctrl->id) {

	case V4L2_CID_FLASH_STROBE_STATUS:
		value = as3636_read(flash, REG_XENON_CONTROL);
		if (value < 0)
			return value;

		ctrl->val = (value & XENON_STROBE) ? 1 : 0;
		ctrl->cur.val = ctrl->val;
		break;
	case V4L2_CID_FLASH_READY:
		value = as3636_is_flash_ready(flash);
		if(value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;
	case V4L2_CID_FLASH_STROBE_SOURCE:
		value = as3636_get_fstrobe_mode(flash);
		if(value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;
	case V4L2_CID_FLASH_CHARGE:
		value = as3636_read(flash, REG_XENON_CONTROL);
		if(value < 0)
			return value;

		ctrl->val = (value & XENON_CHARGE) ? 1 : 0;
		ctrl->cur.val = ctrl->val;
		break;

	case V4L2_CID_PRIVATE_AS3636_GET_CONTROL:
		value = as3636_read(flash, REG_CONTROL);
		if(value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;

	case V4L2_CID_PRIVATE_AS3636_GET_INTERRUPT_STATUS:
		value = as3636_read(flash, REG_INTERRUPT_STATUS);
		if(value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;

	case V4L2_CID_PRIVATE_AS3636_GET_XENON_CONTROL:
		value = as3636_read(flash, REG_XENON_CONTROL);
		if(value < 0)
			return value;

		ctrl->val = value;
		ctrl->cur.val = ctrl->val;
		break;
	}

	return 0;
}

static int as3636_start_charging(struct as3636* flash) {
	int val = as3636_read(flash, REG_XENON_CONTROL);
	if(val < 0)
		return val;

	// We're already fully charged
	if((val & XENON_READY) == XENON_READY)
		return 0;

	// Clear interrupts that prevent entering active mode
	as3636_read_interrupt(flash);

	return as3636_write(flash, REG_XENON_CONTROL, val | XENON_CHARGE);
}

static int as3636_stdby(struct as3636* flash, int enable) {
	int val = as3636_read(flash, REG_CONTROL);
	if (val < 0)
		return val;

	return as3636_write(flash, REG_CONTROL, val | CONTROL_STANDBY);
}

static int as3636_enable_led(struct as3636 *flash, int on) {
	if (on) {
		// Enable IND LED
		as3636_write(flash, REG_LED_CONFIG, 0x01);
		return as3636_write(flash, REG_LED_CONTROL, 0x01);
	}

	// Disable IND LED
	return as3636_write(flash, REG_LED_CONTROL, 0x00);
}

static int as3636_set_fstrobe_mode(struct as3636 *flash, int strobe_source) {
	int val = as3636_read(flash, REG_CONTROL);
	if (val < 0)
		return val;

	if (strobe_source == V4L2_FLASH_STROBE_SOURCE_EXTERNAL)
		val &= ~CONTROL_AFSTROBE;
	else
		val |= CONTROL_AFSTROBE;

	return as3636_write(flash, REG_CONTROL, val);
}

static int as3636_set_register(struct as3636 *flash)
{
	u8 *reg = flash->set_reg->p_new.p_u8;
	u8  addr = reg[0];
	u8 value = reg[1];

	if (addr != REG_CONTROL &&
	    addr != REG_INTERRUPT_STATUS &&
	    addr != REG_XENON_CONTROL )
		return -EINVAL;

	return as3636_write(flash, addr, value);
}


static int as3636_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct as3636 *flash =
		container_of(ctrl->handler, struct as3636, ctrls);
	struct i2c_client *client = v4l2_get_subdevdata(&flash->subdev);

	switch (ctrl->id) {
	case V4L2_CID_FLASH_STROBE:
		return as3636_flash_test(flash);
	case V4L2_CID_FLASH_CHARGE:
		ctrl->val = 0;
		return as3636_start_charging(flash);
	case V4L2_CID_FLASH_STROBE_SOURCE:
		return as3636_set_fstrobe_mode(flash, ctrl->val);
	case V4L2_CID_PRIVATE_AS3636_TEST:
		return as3636_self_test(flash);
	case V4L2_CID_PRIVATE_AS3636_STDBY:
		return as3636_stdby(flash, ctrl->val);
	case V4L2_CID_FLASH_LED_MODE:
		if (ctrl->val == V4L2_FLASH_LED_MODE_FLASH) {
			dev_warn(&client->dev, "Flash LED Mode not supported."
			      		       "Please use NONE or TORCH\n");
			return -EINVAL;
		}
		return as3636_enable_led(flash, ctrl->val);

	case V4L2_CID_PRIVATE_AS3636_SET_REGISTER:
		return as3636_set_register(flash);
	}

	return 0;
}

static inline void as3636_synchronize_ctrl(struct v4l2_ctrl *ctrl)
{
	v4l2_ctrl_lock(ctrl);
	ctrl->cur.val = ctrl->val;
	v4l2_ctrl_unlock(ctrl);
}

static const struct v4l2_ctrl_ops as3636_ctrl_ops = {
	.g_volatile_ctrl = as3636_get_ctrl,
	.s_ctrl = as3636_set_ctrl,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev core operations
 */

/* Put device into know state. */
static int as3636_setup(struct as3636 *flash)
{
	int ret;
	struct as3636_platform_data *pdata = flash->pdata;
	u8 switch_sel, charge_voltage;
	int val;

	ret = as3636_write(flash, REG_CONTROL, CONTROL_RESET);
	if (ret < 0)
		return ret;

	// Recommended 4ms wait after reset
	usleep_range(4000, 5000);

	ret = as3636_write(flash, REG_LED_CONTROL, 0x08); // Enable AF LED to go into active mode.
	if (ret < 0)
		return ret;

	switch_sel = (flash->pdata->switch_selection-375)/75;

	ret = as3636_write(flash, REG_XENON_CONFIG_B, switch_sel | 0x80);
	if (ret < 0)
		return ret;

	charge_voltage = (pdata->charge_max_voltage-CHARGE_MAX_VOLT_MIN);

	ret = as3636_write(flash, REG_XENON_CONFIG_A, charge_voltage);
	if (ret < 0)
		return ret;

	ret = as3636_read(flash, REG_XENON_CONTROL);

	if (ret < 0)
		return ret;

	ret = as3636_write(flash, REG_XENON_CONTROL,
	              ret | (pdata->auto_charge ? XENON_CAR : 0));

    if (ret < 0)
		return ret;

	ret = as3636_write(flash, REG_XENON_CONFIG_C,
	                   ((pdata->dcdc_peak-DCDC_PEAK_MIN)/50) << 6);

	if (ret < 0)
		return ret;

	ret = as3636_write(flash, REG_XENON_SST_PULSE_LENGTH, 0xFF); // In [us].
	if(ret < 0)
		return ret;

	ret = as3636_write(flash, REG_LED_CONTROL, 0x00); // Disable AF LED.
	if (ret < 0)
		return ret;

	ret = as3636_set_fstrobe_mode(flash, V4L2_FLASH_STROBE_SOURCE_SOFTWARE); // Set strobe mode to software by default
	if (ret < 0)
		return ret;

	val = as3636_get_fstrobe_mode(flash);
	flash->source->val = val;

	/* Synchronize control values */
	as3636_synchronize_ctrl(flash->source);

	return 0;
}

static int as3636_set_power(struct v4l2_subdev *sd, int on)
{
	struct as3636 *flash = to_as3636(sd);
	int ret = 0;

	mutex_lock(&flash->power_lock);

	if (on) {
		ret = as3636_setup(flash);
		if (ret < 0) {
			goto done;
		}
	}

	flash->power_count += on ? 1 : -1;
	WARN_ON(flash->power_count < 0);

done:
	mutex_unlock(&flash->power_lock);
	return ret;
}

static int as3636_registered(struct v4l2_subdev *sd)
{
	struct as3636 *flash = to_as3636(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int rval, man, model, version;

	/* Power up the flash driver and read manufacturer ID, model ID, RFU
	 * and version.
	 */
	rval = as3636_set_power(&flash->subdev, 1);
	if (rval < 0)
		return rval;

	rval = as3636_read(flash, REG_IC_INFO);
	if (rval < 0)
		goto power_off;

	man = rval >> 4;
	model = rval & 0x0F;

	/* Verify the chip model and version. */
	if (model != 0x02) {
		dev_err(&client->dev, "AS3636 not detected (model %d)\n", model);
		rval = -ENODEV;
		goto power_off;
	}

	rval = as3636_read(flash, REG_IC_VERSION);
	if (rval < 0)
		goto power_off;

	version = rval & 0x0F;

	dev_info(&client->dev, "AS3636 Version: %d\n", version);

power_off:
	if(rval < 0) {
		as3636_set_power(&flash->subdev, 0);
		return rval;
	}

	return 0;
}

static int as3636_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return as3636_set_power(sd, 1);
}

static int as3636_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return as3636_set_power(sd, 0);
}


static const struct v4l2_subdev_internal_ops as3636_internal_ops = {
	.registered = as3636_registered,
	.open = as3636_open,
	.close = as3636_close,

};

static const struct v4l2_subdev_core_ops as3636_core_ops = {
	.s_power = as3636_set_power,
};

static const struct v4l2_subdev_ops as3636_ops = {
	.core = &as3636_core_ops,
};


/* -----------------------------------------------------------------------------
 *  I2C driver
 */
#ifdef CONFIG_PM

static int as3636_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct as3636 *flash = to_as3636(subdev);
	int rval = 0;

	if (flash->power_count == 0)
		return 0;

	if (flash->pdata->set_power) {
		rval = flash->pdata->set_power(&flash->subdev, 0);
	}

	dev_dbg(&client->dev, "Suspend %s\n", rval < 0 ? "failed" : "ok");

	return rval;
}

static int as3636_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct as3636 *flash = to_as3636(subdev);
	int rval;

	if (flash->power_count == 0)
		return 0;

	if (flash->pdata->set_power) {
		rval = flash->pdata->set_power(&flash->subdev, 1);
		if (rval < 0)
			return rval;
	}

	rval = as3636_setup(flash);

	dev_dbg(&client->dev, "Resume %s\n", rval < 0 ? "fail" : "ok");

	return rval;
}

#else

#define as3636_suspend	NULL
#define as3636_resume	NULL

#endif /* CONFIG_PM */

/**
 * AS3636 interruption handler
 */
static irqreturn_t as3636_isr(int irq, void *priv)
{
	struct as3636 *as3636 = priv;
	int val;

	val = as3636_read_interrupt(as3636);

	if(val < 0)
		return IRQ_HANDLED;

	printk("%s:%d INT : %02X!\n", __func__, __LINE__, val);

	// Charge timeout : capacity is probably too big to charge in less
	// than 5s. Re-enable charging.
	if(val & INTERRUPT_STATUS_TOF)
		as3636_start_charging(as3636);

	return IRQ_HANDLED;
}

static const struct v4l2_ctrl_config ctrl_private_test = {
	.id = V4L2_CID_PRIVATE_AS3636_TEST,
	.ops = &as3636_ctrl_ops,
	.name = "AS3636 - SELFTEST",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config ctrl_private_fstrobe = {
	.id = V4L2_CID_PRIVATE_AS3636_STDBY,
	.ops = &as3636_ctrl_ops,
	.name = "AS3636 - Standby mode",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = false,
	.max = true,
	.step = 1,
	.def = false,
};

static const struct v4l2_ctrl_config ctrl_private_set_register = {
	.id = V4L2_CID_PRIVATE_AS3636_SET_REGISTER,
	.ops = &as3636_ctrl_ops,
	.name = "AS3636 - Set register",
	.type = V4L2_CTRL_TYPE_U8,
	.min = 0,
	.max = 0xff,
	.step = 1,
	.def = 0,
	.dims = { 2 },
	.flags = V4L2_CTRL_FLAG_WRITE_ONLY,
};

static const struct v4l2_ctrl_config ctrl_private_get_control_register = {
	.id = V4L2_CID_PRIVATE_AS3636_GET_CONTROL,
	.ops = &as3636_ctrl_ops,
	.name = "AS3636 - Control reg",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0xff,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config ctrl_private_get_inerrupt_status_register = {
	.id = V4L2_CID_PRIVATE_AS3636_GET_INTERRUPT_STATUS,
	.ops = &as3636_ctrl_ops,
	.name = "AS3636 - Interrupt Status reg",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0xff,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};

static const struct v4l2_ctrl_config ctrl_private_get_xenon_control_register = {
	.id = V4L2_CID_PRIVATE_AS3636_GET_XENON_CONTROL,
	.ops = &as3636_ctrl_ops,
	.name = "AS3636 - Xenon Control reg",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 0xff,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
};


/*
 * as3636_init_controls - Create controls
 * @flash: The flash
 *
 */
static int as3636_init_controls(struct as3636 *flash)
{
	struct v4l2_ctrl *ctrl;
	v4l2_ctrl_handler_init(&flash->ctrls, 10);

	v4l2_ctrl_new_std(&flash->ctrls, &as3636_ctrl_ops,
			  V4L2_CID_FLASH_STROBE, 0, 1, 1, 0);

	ctrl = v4l2_ctrl_new_std(&flash->ctrls, &as3636_ctrl_ops,
				 V4L2_CID_FLASH_CHARGE, 0, 1, 1, 0);

	if(ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	ctrl = v4l2_ctrl_new_std(&flash->ctrls, &as3636_ctrl_ops,
				 V4L2_CID_FLASH_STROBE_STATUS, 0, 1, 1, 0);

	if(ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY;

	ctrl = v4l2_ctrl_new_std(&flash->ctrls, &as3636_ctrl_ops,
				 V4L2_CID_FLASH_READY, 0, 1, 1, 0);

	if(ctrl != NULL)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	v4l2_ctrl_new_std_menu(&flash->ctrls, &as3636_ctrl_ops,
			       V4L2_CID_FLASH_LED_MODE, 2, ~7, V4L2_FLASH_LED_MODE_NONE);

	v4l2_ctrl_new_custom(&flash->ctrls, &ctrl_private_test, NULL);
	v4l2_ctrl_new_custom(&flash->ctrls, &ctrl_private_fstrobe, NULL);

	/* Flash Strobe */
	flash->source = v4l2_ctrl_new_std_menu(&flash->ctrls, &as3636_ctrl_ops,
					       V4L2_CID_FLASH_STROBE_SOURCE,
					       V4L2_FLASH_STROBE_SOURCE_EXTERNAL,
					       ~0x3,
					       V4L2_FLASH_STROBE_SOURCE_SOFTWARE);

	if(flash->source  != NULL)
		flash->source->flags |= V4L2_CTRL_FLAG_VOLATILE;


	/* SET / GET registers */

	flash->set_reg = v4l2_ctrl_new_custom(&flash->ctrls,
					      &ctrl_private_set_register,
					      NULL);

	flash->get_control_reg = v4l2_ctrl_new_custom(&flash->ctrls,
					      &ctrl_private_get_control_register,
					      NULL);

	flash->get_interrupt_status_reg = v4l2_ctrl_new_custom(&flash->ctrls,
							       &ctrl_private_get_inerrupt_status_register,
							       NULL);

	flash->get_xenon_control_reg = v4l2_ctrl_new_custom(&flash->ctrls,
							    &ctrl_private_get_xenon_control_register,
							    NULL);

	flash->subdev.ctrl_handler = &flash->ctrls;

	return flash->ctrls.error;
}

static int as3636_probe(struct i2c_client *client,
			 const struct i2c_device_id *devid)
{
	struct as3636 *flash;
	struct as3636_platform_data *pdata = client->dev.platform_data;
	int ret;

	if (pdata == NULL) {
		dev_err(&client->dev, "Missing platform data\n");
		return -ENODEV;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (flash == NULL) {
		dev_err(&client->dev, "Alloc failed\n");
		return -ENOMEM;
	}

	flash->pdata = pdata;

	if(pdata->switch_selection < 375)
		pdata->switch_selection = 375;

	if(pdata->switch_selection > 900) {
		dev_warn(&client->dev, "Switch selection over 900mA, overriding to 900mA\n");
		pdata->switch_selection = 900;
	}

	if(pdata->charge_max_voltage < CHARGE_MAX_VOLT_MIN)
		pdata->charge_max_voltage = CHARGE_MAX_VOLT_MIN;

	if(pdata->charge_max_voltage > CHARGE_MAX_VOLT_MAX) {
		dev_warn(&client->dev, "Charge voltage over 34.8V, overriding to 34.8V\n");
                pdata->switch_selection = CHARGE_MAX_VOLT_MAX;
        }

	if(pdata->auto_charge > 1)
		pdata->auto_charge = 0;

	if(pdata->dcdc_peak < DCDC_PEAK_MIN)
		pdata->dcdc_peak = DCDC_PEAK_MIN;

	if(pdata->dcdc_peak > DCDC_PEAK_MAX) {
		dev_warn(&client->dev,
		         "DCDC Boost Coil Peak current over 400mA, overriding to 400mA");
		pdata->dcdc_peak = DCDC_PEAK_MAX;
	}

	v4l2_i2c_subdev_init(&flash->subdev, client, &as3636_ops);
	flash->subdev.internal_ops = &as3636_internal_ops;
	flash->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	ret = as3636_init_controls(flash);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't init controls (%d)\n", ret);
		goto done;
	}

	ret = media_entity_init(&flash->subdev.entity, 0, NULL, 0);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't init media entity (%d)\n", ret);
		goto done;
	}

	flash->subdev.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_FLASH;

	mutex_init(&flash->power_lock);

	flash->led_mode = V4L2_FLASH_LED_MODE_NONE;

	flash->strobe_source = V4L2_FLASH_STROBE_SOURCE_EXTERNAL;

	if(client->irq > 0) {
		ret = request_threaded_irq(client->irq,NULL,
		                           as3636_isr,
		                           IRQF_TRIGGER_FALLING,
		                           "as3636", flash);
		if(ret < 0) {
			v4l2_err(client, "failed to register irq\n");
			client->irq = -1;
		}
	}

done:
	if (ret < 0) {
		v4l2_ctrl_handler_free(&flash->ctrls);
		kfree(flash);
	}

	return ret;
}

static int __devexit as3636_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct as3636 *flash = to_as3636(subdev);

	v4l2_device_unregister_subdev(subdev);
	v4l2_ctrl_handler_free(&flash->ctrls);
	media_entity_cleanup(&flash->subdev.entity);
	mutex_destroy(&flash->power_lock);
	kfree(flash);

	return 0;
}

static const struct i2c_device_id as3636_id_table[] = {
	{ AS3636_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, as3636_id_table);

static const struct dev_pm_ops as3636_pm_ops = {
	.suspend = as3636_suspend,
	.resume = as3636_resume,
};

static struct i2c_driver as3636_i2c_driver = {
	.driver	= {
		.owner = THIS_MODULE,
		.name = AS3636_NAME,
		.pm   = &as3636_pm_ops,
	},
	.probe	= as3636_probe,
	.remove	= __devexit_p(as3636_remove),
	.id_table = as3636_id_table,
};

module_i2c_driver(as3636_i2c_driver);

MODULE_AUTHOR("Maxime Jourdan <maxime.jourdan@parrot.com>");
MODULE_DESCRIPTION("Xenon Flash driver for AS3636");
MODULE_LICENSE("GPL");
