/*
 * aea_serial.c
 * based on f_serial.c - generic USB serial function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>

#include "u_serial.h"
#include "gadget_chips.h"


/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */

struct f_gser_aea {
	struct gserial port;
	struct usb_composite_dev *cdev;
	struct work_struct work;
	bool connected;
	bool sw_connected;
	u8 data_id;
	u8 port_num;
};

static inline struct f_gser_aea *func_to_gser_aea(struct usb_function *f)
{
	return container_of(f, struct f_gser_aea, port.func);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor gser_aea_interface_desc __initdata = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0xf0,
	.bInterfaceProtocol =	1,
	.bAlternateSetting =	1,
	/* .iInterface = DYNAMIC */
};

static struct usb_interface_descriptor gser_aea_interface_nop_desc __initdata = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	0,
	.bInterfaceClass =	USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass =	0xf0,
	.bInterfaceProtocol =	1,
	.bAlternateSetting =	0,
	/* .iInterface = DYNAMIC */
};


/* full speed support: */

static struct usb_endpoint_descriptor gser_aea_fs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor gser_aea_fs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *gser_aea_fs_function[] __initdata = {
	(struct usb_descriptor_header *) &gser_aea_interface_desc,
	(struct usb_descriptor_header *) &gser_aea_fs_in_desc,
	(struct usb_descriptor_header *) &gser_aea_fs_out_desc,
	(struct usb_descriptor_header *) &gser_aea_interface_nop_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor gser_aea_hs_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_endpoint_descriptor gser_aea_hs_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
};

static struct usb_descriptor_header *gser_aea_hs_function[] __initdata = {
	(struct usb_descriptor_header *) &gser_aea_interface_desc,
	(struct usb_descriptor_header *) &gser_aea_hs_in_desc,
	(struct usb_descriptor_header *) &gser_aea_hs_out_desc,
	(struct usb_descriptor_header *) &gser_aea_interface_nop_desc,
	NULL,
};

static struct usb_endpoint_descriptor gser_aea_ss_in_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor gser_aea_ss_out_desc __initdata = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor gser_aea_ss_bulk_comp_desc __initdata = {
	.bLength =              sizeof gser_aea_ss_bulk_comp_desc,
	.bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *gser_aea_ss_function[] __initdata = {
	(struct usb_descriptor_header *) &gser_aea_interface_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_in_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_out_desc,
	(struct usb_descriptor_header *) &gser_aea_ss_bulk_comp_desc,
	NULL,
};

/* string descriptors: */
static struct usb_string gser_aea_string_defs[] = {
	[0].s = NULL,
	[1].s = NULL,
	{  } /* end of list */
};

static struct usb_gadget_strings gser_aea_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		gser_aea_string_defs,
};

static struct usb_gadget_strings *gser_aea_strings[] = {
	&gser_aea_string_table,
	NULL,
};

/*-------------------------------------------------------------------------*/

static int match_tty_device(struct device *device, void *data)
{
	int match = 0;
	u8 tty_index = *(u8 *)data;
	char *tty_name = kasprintf(GFP_KERNEL, "ttyGS%d", tty_index);

	if (strncmp(device->kobj.name, tty_name, 6) == 0)
		match = 1;

	kfree(tty_name);
	return match;
}

static void gser_aea_work(struct work_struct *data)
{
	struct f_gser_aea *gser_aea = NULL;
	struct device *tty_device = NULL;
	char *connected[2] = { "TTY_STATE=CONNECTED", NULL };
	char *disconnected[2] = { "TTY_STATE=DISCONNECTED", NULL };
	char **uevent_envp = NULL;
	u8 tty_index = 1;

	gser_aea = container_of(data, struct f_gser_aea, work);

	if (gser_aea->sw_connected == gser_aea->connected)
		return;

	tty_device = device_find_child(&gser_aea->cdev->gadget->dev, &tty_index,
			&match_tty_device);
	if (!tty_device)
		return;

	/* notify userspace of tty connected */
	uevent_envp = gser_aea->connected ? connected : disconnected;
	kobject_uevent_env(&tty_device->kobj, KOBJ_CHANGE, uevent_envp);
	gser_aea->sw_connected = gser_aea->connected;
}

static void connect_tty_device(struct f_gser_aea *gser_aea,
			       struct usb_function *usb_function)
{
	printk(KERN_INFO "connect tty\n");
	gserial_connect(&gser_aea->port, gser_aea->port_num);
	gser_aea->connected = 1;
	schedule_work(&gser_aea->work);
}

static void disconnect_tty_device(struct f_gser_aea *gser_aea,
				  struct usb_function *usb_function)
{
	printk(KERN_INFO "disconnect tty\n");
	gserial_disconnect(&gser_aea->port);
	gser_aea->connected = 0;
	schedule_work(&gser_aea->work);
}

static int config_tty_device_ep(struct usb_function *usb_function,
				struct usb_composite_dev *cdev)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(usb_function);
	struct usb_ep *ep_in = gser_aea->port.in;
	struct usb_ep *ep_out = gser_aea->port.out;
	u8 tty_index = gser_aea->port_num;

	/* USB EndPoints already configured, nothing to do */
	if (ep_in->desc && ep_out->desc)
		return 0;

	DBG(cdev, "configure usb EndPoints for ttyGS%d\n", tty_index);

	if (config_ep_by_speed(cdev->gadget, usb_function, ep_in) ||
	    config_ep_by_speed(cdev->gadget, usb_function, ep_out)) {
		DBG(cdev, "Cannot configure usb EndPoints [ttyGS%d]\n",
		    tty_index);

		ep_in->desc = NULL;
		ep_out->desc = NULL;

		return -EINVAL;
	}

	return 0;
}

static int gser_aea_set_alt(struct usb_function *usb_function,
				 unsigned interface, unsigned alt)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(usb_function);
	struct usb_composite_dev *cdev = usb_function->config->cdev;

	DBG(cdev, "gser_aea_set_alt [interface=%u] [alt=%u]\n",
	    interface, alt);

	/* if alt == 0: apple device uses the dummy interface
	 *    so we disconnect ttyGS1
	 * if alt == 1: apple device uses the data interface
	 *    so we configure usb EP and connect ttyGS1 */
	if (alt == 1) {
		int ret = config_tty_device_ep(usb_function, cdev);
		if (ret != 0)
			return -EINVAL;

		connect_tty_device(gser_aea, usb_function);
	}
	else {
		if (gser_aea->port.in->driver_data)
			disconnect_tty_device(gser_aea, usb_function);
	}

	return 0;
}

static void gser_aea_disable(struct usb_function *usb_function)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(usb_function);
	disconnect_tty_device(gser_aea, usb_function);
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int __init
gser_aea_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_gser_aea	*gser_aea = func_to_gser_aea(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser_aea->data_id = status;
	gser_aea_interface_desc.bInterfaceNumber = status;
	gser_aea_interface_nop_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* data interface label */
	status = usb_string_id(c->cdev);
	if (status < 0)
		return status;
	gser_aea_string_defs[0].id = status;
	gser_aea_string_defs[1].id = status;
	gser_aea_interface_desc.iInterface = status;
	gser_aea_interface_nop_desc.iInterface = status;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &gser_aea_fs_in_desc);
	if (!ep)
		goto fail;
	gser_aea->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &gser_aea_fs_out_desc);
	if (!ep)
		goto fail;
	gser_aea->port.out = ep;
	gser_aea->cdev = cdev;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->descriptors = usb_copy_descriptors(gser_aea_fs_function);

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		gser_aea_hs_in_desc.bEndpointAddress =
				gser_aea_fs_in_desc.bEndpointAddress;
		gser_aea_hs_out_desc.bEndpointAddress =
				gser_aea_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(gser_aea_hs_function);
	}
	if (gadget_is_superspeed(c->cdev->gadget)) {
		gser_aea_ss_in_desc.bEndpointAddress =
			gser_aea_fs_in_desc.bEndpointAddress;
		gser_aea_ss_out_desc.bEndpointAddress =
			gser_aea_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->ss_descriptors = usb_copy_descriptors(gser_aea_ss_function);
		if (!f->ss_descriptors)
			goto fail;
	}

	INFO(cdev, "Apple External Accessory ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser_aea->port_num,
			gadget_is_superspeed(cdev->gadget) ? "super" :
			gadget_is_dualspeed(cdev->gadget) ? "dual" : "full",
			gser_aea->port.in->name, gser_aea->port.out->name);
	return 0;

fail:
	/* we might as well release our claims on endpoints */
	if (gser_aea->port.out)
		gser_aea->port.out->driver_data = NULL;
	if (gser_aea->port.in)
		gser_aea->port.in->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);
	return status;
}

static void
gser_aea_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct f_gser_aea *gser_aea = func_to_gser_aea(f);

	cancel_work_sync(&gser_aea->work);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	if (gadget_is_superspeed(c->cdev->gadget))
		usb_free_descriptors(f->ss_descriptors);
	usb_free_descriptors(f->descriptors);
	kfree(gser_aea);
}

/**
 * gser_aea_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * @protocol: protocol used
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int __init gser_aea_bind_config(struct usb_configuration *c, u8 port_num,
				const char *protocol)
{
	struct f_gser_aea	*gser_aea;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	gser_aea_string_defs[0].s = protocol;
	gser_aea_string_defs[1].s = protocol;

	/* maybe allocate device-global string ID */
	if (gser_aea_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_aea_string_defs[0].id = status;
	}

	if (gser_aea_string_defs[1].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		gser_aea_string_defs[1].id = status;
	}

	/* allocate and initialize one new instance */
	gser_aea = kzalloc(sizeof *gser_aea, GFP_KERNEL);
	if (!gser_aea)
		return -ENOMEM;

	gser_aea->port_num = port_num;
	gser_aea->port.func.name = "gser_aea";
	gser_aea->port.func.strings = gser_aea_strings;
	gser_aea->port.func.bind = gser_aea_bind;
	gser_aea->port.func.unbind = gser_aea_unbind;
	gser_aea->port.func.set_alt = gser_aea_set_alt;
	gser_aea->port.func.disable = gser_aea_disable;

	INIT_WORK(&gser_aea->work, gser_aea_work);

	status = usb_add_function(c, &gser_aea->port.func);
	if (status)
		kfree(gser_aea);

	return status;
}
