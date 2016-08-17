/*
 * Blackberry USB Phone driver
 *
 * Copyright (C) 2008 Greg Kroah-Hartman <greg@kroah.com>
 * Copyright (C) 2011 Parrot <carrier.nicolas0@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 * {sigh}
 * Based on the moto-modem driver, provides a "dumb serial connection" to a
 * Blackberry with as much ttys as half the number of endpoints. Ip modem
 * typically corresponds to the third tty created.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>

#define DRIVER_NAME "blackberry"
#define VENDOR_ID 0x0fca

static struct usb_device_id id_table [] = {
	{USB_DEVICE_AND_INTERFACE_INFO(VENDOR_ID, 0x0001, 0xff, 1, 0xff)},
	{USB_DEVICE_AND_INTERFACE_INFO(VENDOR_ID, 0x0004, 0xff, 1, 0xff)},
	{USB_DEVICE_AND_INTERFACE_INFO(VENDOR_ID, 0x8001, 0xff, 1, 0xff)},
	{USB_DEVICE_AND_INTERFACE_INFO(VENDOR_ID, 0x8004, 0xff, 1, 0xff)},
	{USB_DEVICE_AND_INTERFACE_INFO(VENDOR_ID, 0x8007, 0xff, 1, 0xff)},
	{}     /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver bb_driver = {
	.name =          DRIVER_NAME,
	.probe =         usb_serial_probe,
	.disconnect =    usb_serial_disconnect,
	.id_table =      id_table,
	.no_dynamic_id = 1,
};

static int bb_calc_num_ports(struct usb_serial *serial)
{
	return serial->interface->cur_altsetting->desc.bNumEndpoints / 2;
}

static struct usb_serial_driver bb_serial_driver = {
	.driver = {
		.owner =  THIS_MODULE,
		.name =   DRIVER_NAME,
	},
	.id_table =       id_table,
	.usb_driver =     &bb_driver,
	.calc_num_ports = bb_calc_num_ports,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&bb_serial_driver, NULL
};

module_usb_serial_driver(bb_driver, serial_drivers);
MODULE_LICENSE("GPL");
