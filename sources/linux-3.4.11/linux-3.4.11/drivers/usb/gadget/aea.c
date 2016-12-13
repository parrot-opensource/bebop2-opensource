/*
 * eae.c -- Apple External Accessory driver
 *
 * Copyright (C) 2008 David Brownell
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/module.h>
#include "u_serial.h"

#define DRIVER_DESC		"Apple External Accessory iAP2"

/*-------------------------------------------------------------------------*/

/* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */

/* Thanks to NetChip Technologies for donating this product ID.
 * It's for devices with only this composite CDC configuration.
 */
#define CDC_VENDOR_NUM		0x19cf	/* Parrot SA */
#define CDC_PRODUCT_NUM		0xa4aa	/* CDC Composite: ECM + ACM */

/*-------------------------------------------------------------------------*/

#define USE_SERIAL_IAP	1 // define for f_serial.c

/*-------------------------------------------------------------------------*/

#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "u_serial.c"
#include "f_serial.c"
#include "aea_serial.c"

/*-------------------------------------------------------------------------*/

static char *app_protocol = "com.unknown.protocol";

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		0x00,
	.bDeviceSubClass =	0x00,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id can be overridden by module parameters.  */
	.idVendor =		cpu_to_le16(CDC_VENDOR_NUM),
	.idProduct =		cpu_to_le16(CDC_PRODUCT_NUM),
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = "Parrot",
	[STRING_PRODUCT_IDX].s = "Generic",
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static int __init aea_do_config(struct usb_configuration *c)
{
	int status;

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	status = gser_bind_config(c, 0);
	if (status < 0)
		return status;

	status = gser_aea_bind_config(c, 1, app_protocol);
	if (status < 0)
		return status;

	return 0;
}

static struct usb_configuration aea_config_driver = {
	.label			= "Apple External Accessory",
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

static int __init aea_bind(struct usb_composite_dev *cdev)
{
	struct usb_gadget *gadget = cdev->gadget;
	int status, gcnum;

	/**
	 * set up serial link layer
	 * 2 tty are created: one for iap communication,
	 * another for app communication.
	 */
	status = gserial_setup(cdev->gadget, 2);
	if (status < 0)
		return status;

	gcnum = usb_gadget_controller_number(gadget);
	device_desc.bcdDevice = cpu_to_le16(0x0300 | gcnum);

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;

	strings_dev[STRING_MANUFACTURER_IDX].id = status;
	device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;

	strings_dev[STRING_PRODUCT_IDX].id = status;
	device_desc.iProduct = status;

	/* register our configuration */
	status = usb_add_config(cdev, &aea_config_driver, aea_do_config);
	if (status < 0)
		goto fail;

	dev_info(&gadget->dev, "%s\n", DRIVER_DESC);
	return 0;

fail:
	gserial_cleanup();
	return status;
}

static int __exit aea_unbind(struct usb_composite_dev *cdev)
{
	gserial_cleanup();
	return 0;
}

static struct usb_composite_driver aea_driver = {
	.name		= "g_aea",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.max_speed	= USB_SPEED_HIGH,
	.unbind		= __exit_p(aea_unbind),
};

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");

/*-------------------------------------------------------------------------*/
module_param(app_protocol, charp, S_IRUSR);
MODULE_PARM_DESC(app_protocol, "Application protocol");

static int __init init(void)
{
	printk(KERN_DEBUG "app protocol:'%s'\n", app_protocol);
	return usb_composite_probe(&aea_driver, aea_bind);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&aea_driver);
}
module_exit(cleanup);
