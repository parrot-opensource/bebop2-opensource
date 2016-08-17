/*
 * Platform data for the chipidea USB dual role controller
 */

#ifndef __LINUX_USB_CHIPIDEA_H
#define __LINUX_USB_CHIPIDEA_H

#include <linux/platform_device.h>

enum ci_udc_operating_modes {
	CI_UDC_DR_HOST = 0,
	CI_UDC_DR_DEVICE,
	CI_UDC_DR_OTG,
	/*
	 * "static OTG": driver modules for host/device
	 * can be changed after linux boot
	 */
	CI_UDC_DR_DUAL_HOST, // dual_mode booting in host
	CI_UDC_DR_DUAL_DEVICE,// dual_mode booting in device

	CI_UDC_DR_DISABLE_HOST_WORKAROUND = 0x100, // Will do a reset if 10 "null" transfer are detected
};


struct ci13xxx;
struct ci13xxx_udc_driver {
	const char	*name;
	/* offset of the capability registers */
	uintptr_t	 capoffset;
	unsigned	 power_budget;
	unsigned long	 flags;
#define CI13XXX_REGS_SHARED		BIT(0)
#define CI13XXX_REQUIRE_TRANSCEIVER	BIT(1)
#define CI13XXX_PULLUP_ON_VBUS		BIT(2)
#define CI13XXX_DISABLE_STREAMING	BIT(3)

#define CI13XXX_CONTROLLER_RESET_EVENT		0
#define CI13XXX_CONTROLLER_STOPPED_EVENT	1
	enum	ci_udc_operating_modes    operating_mode;
	u8		workaround_reset_nb;
	void	(*notify_event) (struct ci13xxx *udc, unsigned event);
	int		(*init)(struct platform_device *, int (*ulpi_write)(struct platform_device *pdev, u8 addr, u8 val));
	void	(*exit)(struct platform_device *);
	void	(*set_vbus)(struct platform_device *, int value);
};

/* Default offset of capability registers */
#define DEF_CAPOFFSET		0x100

#endif
