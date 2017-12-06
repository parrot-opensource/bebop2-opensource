#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/module.h>

#include <parrot/mfd/dib0700_mfd.h>

#define DRIVER_NAME "sicilia-irradiance"

static struct led_info ncp5623c_info[] = {
	{
		.name = "irradiance:blue",
		.default_trigger = "none",
	},
	{
		.name = "irradiance:green",
		.default_trigger = "none",
	},
	{
		.name = "irradiance:red",
		.default_trigger = "none",
	},
};

static struct led_info pca9633_info[] = {
	{
		.name = "irradiance:red",
		.default_trigger = "none",
	},
	{
		.name = "irradiance:green",
		.default_trigger = "none",
	},
	{
		.name = "irradiance:blue",
		.default_trigger = "none",
	},
};

static struct led_platform_data ncp5623c_data = {
	.num_leds = ARRAY_SIZE(ncp5623c_info),
	.leds = ncp5623c_info,
};

static struct led_platform_data pca9633_data = {
	.num_leds = ARRAY_SIZE(pca9633_info),
	.leds = pca9633_info,
};

static struct i2c_board_info leds_driver_board_info[] = {
	{
		I2C_BOARD_INFO("ncp5623c", 0x39),
		.platform_data = &ncp5623c_data,
	},
	{
		I2C_BOARD_INFO("pca9633", 0x62),
		.platform_data = &pca9633_data,
	}
};

#define TSL2591_SENSORS_NB 4
#define GPIO_GNSS_POWER_EN 1
#define GPIO_THERMAL_PWM   6
#define GPIO_FLASH         8

static struct i2c_board_info tsl2591_board_info = {
	I2C_BOARD_INFO("tsl2591", 0x29),
};

static struct i2c_client * leds_driver_client;
static struct i2c_client * tsl2591_client[TSL2591_SENSORS_NB];
static int dib0700_gpio_base = -1;

static int irradiance_probe(struct usb_interface *intf,
                            const struct usb_device_id *id)
{
	int ret = 0;
	struct i2c_adapter * adapter = NULL;
	int i, j;
	struct i2c_msg msg = {0};
	char temp = 0;
	unsigned gps_power_en_gpio;

	ret = dib0700_probe(intf, &dib0700_gpio_base);
	if (ret) {
		goto dib0700_probe_failed;
	}

	/* leds i2c driver */
	adapter = i2c_get_adapter(3);
	for (i = 0; i < ARRAY_SIZE(leds_driver_board_info); i++) {
		/* scan i2c bus */
		msg.addr = leds_driver_board_info[i].addr;
		msg.flags = 0;
		msg.len = 0;
		msg.buf = NULL;

		ret = i2c_transfer(adapter, &msg, 1);

		if (ret == 1)
			break;

		msg.flags = I2C_M_RD;
		msg.len = 1;
		msg.buf = &temp;
		ret = i2c_transfer(adapter, &msg, 1);

		if (ret == 1)
			break;
	}

	if (i >= ARRAY_SIZE(leds_driver_board_info)) {
		dev_err(&intf->dev, "no leds driver found\n");

		ret = -ENXIO;
		goto leds_driver_probe_failed;
	}

	leds_driver_client =
		i2c_new_device(adapter, &leds_driver_board_info[i]);
	if (leds_driver_client == NULL) {
		ret = -ENXIO;
		goto leds_driver_probe_failed;
	}

	/* ts2591 illuminance i2c driver */
	for (i = 0; i < TSL2591_SENSORS_NB; i++) {
		adapter = i2c_get_adapter(3+i);
		tsl2591_client[i] = i2c_new_device(adapter, &tsl2591_board_info);
		if (tsl2591_client[i] == NULL) {
			ret = -ENXIO;
			goto tsl2591_probe_failed;
		}
	}

	/* Export gps power enable gpio */
	gps_power_en_gpio = dib0700_gpio_base + GPIO_GNSS_POWER_EN;
	gpio_request_one(gps_power_en_gpio,
	                 GPIOF_OUT_INIT_HIGH,
	                 "gps_power_en");
	gpio_export(gps_power_en_gpio, 0);


	return 0;

tsl2591_probe_failed:
	for (j = 0; j < i; j++) {
		i2c_unregister_device(tsl2591_client[j]);
	}
	i2c_unregister_device(leds_driver_client);
leds_driver_probe_failed:
	dib0700_device_exit(intf);
dib0700_probe_failed:
	return ret;
}

static void irradiance_device_exit(struct usb_interface *intf)
{
	int i;

	for (i = 0; i < TSL2591_SENSORS_NB; i++) {
		i2c_unregister_device(tsl2591_client[i]);
	}

	i2c_unregister_device(leds_driver_client);
	if (dib0700_gpio_base != -1)
		gpio_free(dib0700_gpio_base + GPIO_GNSS_POWER_EN);
	dib0700_device_exit(intf);
}

#define USB_VID_PARROT			0x19cf
#define USB_PID_PARROT_HOOK_DEFAULT	0x5100

static struct usb_device_id irradiance_usb_id_table[] = {
	{ USB_DEVICE(USB_VID_DIBCOM, USB_PID_DIBCOM_HOOK_DEFAULT) },
	{ USB_DEVICE(USB_VID_PARROT, USB_PID_PARROT_HOOK_DEFAULT) },
	{ 0 }
};

MODULE_DEVICE_TABLE(usb, irradiance_usb_id_table);

static struct usb_driver irradiance_driver = {
	.name       = DRIVER_NAME,
	.probe      = irradiance_probe,
	.disconnect = irradiance_device_exit,
	.id_table   = irradiance_usb_id_table,
};

module_usb_driver(irradiance_driver);

MODULE_AUTHOR("Ronan CHAUVIN <ronan.chauvin@parrot.com>");
MODULE_DESCRIPTION("Sicilia irradiance module driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
