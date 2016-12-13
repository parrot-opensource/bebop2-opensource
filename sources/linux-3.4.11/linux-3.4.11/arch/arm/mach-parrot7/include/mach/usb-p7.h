/**
 * @file arch/arm/mach-parrot7/include/mach/usb-p7.h
 * @brief Parrot7 usb controller definitions
 *
 * Copyright (C) 2009 Parrot S.A.
 *
 * @author     olivier.bienvenu@parrot.com
 * @date       2009-04-17
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef __ARCH_ARM_PARROT_USB_P7_H
#define __ARCH_ARM_PARROT_USB_P7_H

#include <linux/pinctrl/consumer.h>
#include <linux/usb/chipidea.h>

#define USB_DRV_NAME "ci_hdrc"

enum p7_usb2_phy_modes {
	P7_USB2_PHY_NONE,
	P7_USB2_PHY_ULPI,
	P7_USB2_PHY_UTMI,
	P7_USB2_PHY_UTMI_WIDE,
	P7_USB2_PHY_SERIAL,
};

struct p7_usb2_platform_data {
	struct ci13xxx_udc_driver	ci_udc;
	unsigned int				phy_enable; // adress of the PHY enable register on the p7mu
	unsigned int				gpio_vbus;
	struct pinctrl				*pctl;
	unsigned int				chip_rev;
};

#endif /* __ARCH_ARM_PARROT_REGS_P7_H */
