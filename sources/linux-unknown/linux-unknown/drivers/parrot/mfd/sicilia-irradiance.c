#include <linux/usb.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/leds.h>
#include <linux/module.h>

#include <parrot/mfd/dib0700_mfd.h>

#define DRIVER_NAME "sicilia-irradiance"

static struct led_info pca9633_leds[] = {
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

static struct led_platform_data pca9633_data = {
	.num_leds = ARRAY_SIZE(pca9633_leds),
	.leds = pca9633_leds,
};

static struct i2c_board_info pca9633_board_info = {
	I2C_BOARD_INFO("pca9633", 0x62),
	.platform_data = &pca9633_data,
};

#define TSL2591_SENSORS_NB 4
#define GPIO_GNSS_POWER_EN 1
#define GPIO_THERMAL_PWM   6
#define GPIO_FLASH         8

static struct i2c_board_info tsl2591_board_info = {
	I2C_BOARD_INFO("tsl2591", 0x29),
};

static struct i2c_client * pca9633_client;
static struct i2c_client * tsl2591_client[TSL2591_SENSORS_NB];

static int irradiance_probe(struct usb_interface *intf,
                            const struct usb_device_id *id)
{
	int ret = 0;
	struct i2c_adapter * adapter = NULL;
	int dib0700_gpio_base;
	int i, j;

	ret = dib0700_probe(intf, &dib0700_gpio_base);
	if (ret) {
		goto dib0700_probe_failed;
	}

	/* pca9633 leds i2c driver */
	adapter = i2c_get_adapter(3);
	pca9633_client = i2c_new_device(adapter, &pca9633_board_info);
	if (pca9633_client == NULL) {
		ret = -ENXIO;
		goto pca966_probe_failed;
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

	/* Power up gps */
	gpio_request_one(dib0700_gpio_base + GPIO_GNSS_POWER_EN,
	                 GPIOF_OUT_INIT_HIGH,
	                 "gps_power_en");

	return 0;

tsl2591_probe_failed:
	for (j = 0; j < i; j++) {
		i2c_unregister_device(tsl2591_client[j]);
	}
	i2c_unregister_device(pca9633_client);
pca966_probe_failed:
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

	i2c_unregister_device(pca9633_client);
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
