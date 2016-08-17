/*
 * adv7391.c
 *
 * Driver for Analog device's 7391
 *
 * Author : Nicolas Laclau <nicolas.laclau@parrot.com>
 *
 * Date : 22 Jan. 2015
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <media/adv7391.h>

struct adv7391 {
	struct i2c_client *client;
};

/* Register maps */
struct reg_cfg {
	u8 reg;
	u8 val;
};

/* ------------------------------------------------------------------------- */
static s32 adv7391_i2c_read_byte(struct i2c_client *client,
				 u8 command)
{
	union i2c_smbus_data data;

	if (!i2c_smbus_xfer(client->adapter, client->addr, client->flags,
			I2C_SMBUS_READ, command,
			I2C_SMBUS_BYTE_DATA, &data))
		return data.byte;
	else
		return -EIO;
}

static s32 adv7391_i2c_write_byte(struct i2c_client *client,
				  u8 command,
				  u8 value)
{
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
		dev_err(&client->dev, "error writing addr:%02x, reg:%02x,"
				" val:%02x\n",
				client->addr, command, value);
	return err;
}

static s32 adv7391_write_reg_array(struct i2c_client *client,
				   struct reg_cfg *array, int nb)
{
	s32 ret;
	int i;

	for(i=0;i<nb;i++) {
		ret = adv7391_i2c_write_byte(client,
					     array[i].reg,
					     array[i].val);
		if(ret < 0)
			return ret;
	}
	return 0;
}

static s32 adv7391_setup_default(struct adv7391 *adv) {

	struct adv7391_platform_data * pdata = adv->client->dev.platform_data;
	struct reg_cfg cfg[] = {
			{ADV7391_REG_SW_RESET, ADV7391_FLG_SW_RESET},
			{ADV7391_REG_PWR_MODE, pdata->dac_pwr},
			{ADV7391_REG_DAC_OUT_LVL, pdata->dac_gain},
			{ADV7391_REG_SD_MODE1, pdata->sd_mode1},
			{ADV7391_REG_SD_MODE2, pdata->sd_mode2},
			{ADV7391_REG_SD_MODE3, pdata->sd_mode3},
			{ADV7391_REG_SD_SCL_LSB, pdata->scale_lsb}
	};

	return adv7391_write_reg_array(adv->client, cfg, ARRAY_SIZE(cfg));
}

/* ------------------------------------------------------------------------- */
/* attributes operations */
static s32 adv7391_write_config(struct i2c_client *client, u8 reg,
				const char *buf, size_t count)
{
	s32 res;
	ulong val;

	/* force null terminated string */
	if (buf[count] != '\0')
		return -EINVAL;

	res = kstrtoul(buf, 0, &val);
	if (res < 0)
		return res;

	/* check range */
	if( (val < 0x0000) || (val > 0x00FF) )
		return -EINVAL;

	res = adv7391_i2c_write_byte(client, reg, (u8)(val & 0x00FF));
	if (res < 0)
		return res;

	return count;
}

static ssize_t adv7391_g_enable(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	int res = 0, ret = 0;

	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_PWR_MODE);
	if (res < 0)
		return res;

	if (res & ADV7391_FLG_DAC1_PWR)
		ret |= 1;
	if (res & ADV7391_FLG_DAC2_PWR)
		ret |= 2;
	if (res & ADV7391_FLG_DAC2_PWR)
		ret |= 4;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", ret);
}

static ssize_t adv7391_s_enable(struct device *device,
		struct device_attribute *attr,
		const  char *buf,
		size_t count)
{
	s32 res;
	ulong val;
	struct adv7391 *adv = dev_get_drvdata(device);

	/* force null terminated string */
	if (buf[count] != '\0')
		return -EINVAL;

	res = kstrtoul(buf, 0, &val);
	if (res < 0)
		return res;

	/* check range */
	if ((val < 0x0000) || (val > 0x00FF))
		return -EINVAL;

	/* get current register value */
	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_PWR_MODE);
	if (res < 0)
		return res;

	/* clear DAC1, DAC2 and DAC3 flags */
	res &= ~(ADV7391_FLG_DAC1_PWR |
			ADV7391_FLG_DAC2_PWR |
			ADV7391_FLG_DAC3_PWR);

	/* Set new value requested for DACs */
	if (val & 0x01)
		res |=  ADV7391_FLG_DAC1_PWR;
	if (val & 0x02)
		res |=  ADV7391_FLG_DAC2_PWR;
	if (val & 0x04)
		res |=  ADV7391_FLG_DAC3_PWR;

	res = adv7391_i2c_write_byte(adv->client, ADV7391_REG_PWR_MODE, (u8)(res & 0x00FF));
	if (res < 0)
		return res;

	return count;
}

static ssize_t adv7391_g_ctrl(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	int res;

	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_SD_MODE6);
	if (res < 0)
		return res;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", res);
}

static ssize_t adv7391_s_ctrl(struct device *device,
		struct device_attribute *attr,
		const  char *buf,
		size_t count)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	return adv7391_write_config(adv->client, ADV7391_REG_SD_MODE6,
			buf, count);
}

static ssize_t adv7391_g_brightness(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	int res;

	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_SD_BRIGHT);
	if (res < 0)
		return res;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", res);
}

static ssize_t adv7391_s_brightness(struct device *device,
		struct device_attribute *attr,
		const  char *buf,
		size_t count)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	return adv7391_write_config(adv->client, ADV7391_REG_SD_BRIGHT,
			buf, count);
}

static ssize_t adv7391_g_luma(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	int res;

	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_SD_Y_SCL);
	if (res < 0)
		return res;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", res);
}

static ssize_t adv7391_s_luma(struct device *device,
		struct device_attribute *attr,
		const  char *buf,
		size_t count)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	return adv7391_write_config(adv->client, ADV7391_REG_SD_Y_SCL,
			buf, count);
}

static ssize_t adv7391_g_cb_scl(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	int res;

	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_SD_CB_SCL);
	if (res < 0)
		return res;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", res);
}

static ssize_t adv7391_g_cr_scl(struct device *device,
		struct device_attribute *attr,
		char *buf)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	int res;

	res = adv7391_i2c_read_byte(adv->client, ADV7391_REG_SD_CR_SCL);
	if (res < 0)
		return res;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", res);
}

static ssize_t adv7391_s_cb_scl(struct device *device,
		struct device_attribute *attr,
		const  char *buf,
		size_t count)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	return adv7391_write_config(adv->client, ADV7391_REG_SD_CB_SCL,
			buf, count);
}

static ssize_t adv7391_s_cr_scl(struct device *device,
		struct device_attribute *attr,
		const  char *buf,
		size_t count)
{
	struct adv7391 *adv = dev_get_drvdata(device);
	return adv7391_write_config(adv->client, ADV7391_REG_SD_CR_SCL,
			buf, count);
}

static struct device_attribute adv7391_sysfs_attrs[] = {
	__ATTR(enable, S_IRUGO|S_IWUSR, adv7391_g_enable, adv7391_s_enable),
	__ATTR(control, S_IRUGO|S_IWUSR, adv7391_g_ctrl, adv7391_s_ctrl),
	__ATTR(brightness, S_IRUGO|S_IWUSR, adv7391_g_brightness, adv7391_s_brightness),
	__ATTR(luma, S_IRUGO|S_IWUSR, adv7391_g_luma, adv7391_s_luma),
	__ATTR(chroma_b, S_IRUGO|S_IWUSR, adv7391_g_cb_scl, adv7391_s_cb_scl),
	__ATTR(chroma_r, S_IRUGO|S_IWUSR, adv7391_g_cr_scl, adv7391_s_cr_scl),
};

/* ------------------------------------------------------------------------- */
static int __devinit adv7391_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct adv7391 *adv = NULL;
	int ret = 0;
	int i;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		ret = -EIO;
		goto ei2c;
	}

	/* allocate context memory */
	adv = kzalloc(sizeof(*adv), GFP_KERNEL);
	if (!adv) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto enomem;
	}

	adv->client = client;

	for (i = 0; i < ARRAY_SIZE(adv7391_sysfs_attrs); i++) {
		ret = device_create_file(&client->dev,
				         &adv7391_sysfs_attrs[i]);
		if (ret) {
			for (--i; i >= 0; i--)
				device_remove_file(&client->dev,
						&adv7391_sysfs_attrs[i]);
			goto efail;
		}
	}

	ret = adv7391_setup_default(adv);

	dev_info(&client->dev, "detected i2c-device: 0x%02x\n", client->addr);

	i2c_set_clientdata(client, adv);

	return ret;
efail:
	kfree(adv);
enomem:
ei2c:
	return ret;
}

static int __devexit adv7391_remove(struct i2c_client *client)
{
	struct adv7391 *adv = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < ARRAY_SIZE(adv7391_sysfs_attrs); i++)
		device_remove_file(&client->dev, &adv7391_sysfs_attrs[i]);

	kfree(adv);
	return 0;
}

static struct i2c_device_id adv7391_id[] = {
	{ ADV7391_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, adv7391_id);

static struct i2c_driver adv7391_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = ADV7391_NAME,
	},
	.id_table = adv7391_id,
	.probe = adv7391_probe,
	.remove = adv7391_remove,
};

module_i2c_driver(adv7391_driver);

MODULE_AUTHOR("Nicolas Laclau <nicolas.laclau@parrot.com>");
MODULE_DESCRIPTION("Analog device's 7391 driver");
MODULE_LICENSE("GPL");
