/**
 * linux/drivers/parrot/mfd/dib0700_mfd.h
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Ronan Chauvin <ronan.chauvin@parrot.com>
 * date:    14-Sep-2014
 *
 * This file is released under the GPL
 */

#ifndef _MFD_DIB0700_H
#define _MFD_DIB0700_H

#include "dib07x0.h"

extern int dib0700_probe(struct usb_interface *intf, int *gpio_base);
extern void dib0700_device_exit(struct usb_interface *intf);

#endif
