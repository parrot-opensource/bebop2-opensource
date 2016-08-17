/**
 * linux/arch/arm/mach-parrot7/board-mpp.c
 *
 * Copyright (C) 2016 Parrot S.A.
 *
 *
 * This file is released under the GPL
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/i2c.h>
#include <linux/leds_pwm.h>
#include <linux/dma-mapping.h>
#include <linux/ramoops.h>
#include <linux/memblock.h>
#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/pgtable.h>
#include <asm/hardware/gic.h>
#include <asm/system_info.h>
#include <mach/p7.h>
#include <mach/irqs.h>
#include <mach/usb-p7.h>
#include <mfd/p7mu.h>
#include "board-common.h"
#include "common.h"
#include "pinctrl.h"
#include "usb.h"
#include "p7_temperature.h"
#include "nand.h"
#include "usb.h"

/* USB HUB support */
#include <../drivers/parrot/i2c/smsc-82514-usb-hub.h>

#include "board-common.h"
#include "common.h"
#include "p7_pwm.h"
#include "gpio.h"

#include "board-drone-common.h"

int get_mpp_revision(void)
{
	int board_rev = 0, i, err, val;
	static const int gpios[] = {138, 139, 140, 141};

	for (i = 0; i < ARRAY_SIZE(gpios); i++) {
		err = parrot_gpio_in_init(P7_GPIO_NR(gpios[i]),
				P7CTL_PUD_CFG(DOWN), "mpp rev");

		if (err == 0) {
			val = gpio_get_value(P7_GPIO_NR(gpios[i]));
			board_rev |= (val << i);
			gpio_free(P7_GPIO_NR(gpios[i]));
		}
	}

	return board_rev;
}

/*************
 * USB hub
 *************/

#if 0
static struct smsc82514_pdata ev_smsc82512_pdata = {
	.us_port   = DS_HIGH,
	.ds_port_1 = DS_HIGH,
	.ds_port_2 = DS_HIGH,
	.reset_pin = P7_GPIO_NR(HSIS_HWxx__HUB_RST),
};

static struct i2c_board_info smsc82512_info = {
	I2C_BOARD_INFO("smsc82512", 0x2c),
	.platform_data = &ev_smsc82512_pdata,
};
#endif

static void __init mpp_init_mach(void)
{
	/* Initialize ramoops */
	drone_common_init_ramoops();

	/* Init hardware revision independent stuff */
	p7_init_mach();

	/* Init GPIOs */
	p7_init_gpio(NULL, 0);

	/* Get PCB/HW revision and update HSIS */
	//ev_board_probe();

	/* Init debug UART */
	p7brd_init_uart(3, 0);

	/* Init NAND */
	p7brd_init_nand(0);

	/* Init I2C master
	 * I2CM-0:
	 *     P7MU
	 *     HDMI Input (with EDID EEPROM) not used anymore
	 * I2CM-2:
	 *     AKM8963 (magneto)
	 *     MS5607 (Pressure/Temperature)
	 *     HDMI Input (with EDID EEPROM) not used anymore
	 * I2CM-1:
	 *     MPU6050 (gyro/accel)
	 * XXX connect hub ?
	 *     USB HUB
	 */
	p7brd_init_i2cm(0, 100);
	p7brd_init_i2cm(1, 200);
	/* /!\ HDMI DDC which uses I2C protocol,
	 *     requires a clock lower or equal to
	 *     to 100kHz! */
	p7brd_init_i2cm(2, 100);

	/* Init BT */
	p7brd_init_uart(1, 1);
	p7brd_export_gpio(119, GPIOF_OUT_INIT_LOW, "bt-rst");

	/* Initialize P7MU */
	p7_gpio_interrupt_register(73);
	drone_common_init_p7mu(73, 0);

	/* Init USB */
	if (get_mpp_revision() == 0) {
		/* HW0 are buggy */
		if (parrot_force_usb_device) {
			p7brd_init_usb(0, -1, CI_UDC_DR_DEVICE);
		} else {
			void __iomem *sysctl = (void __iomem *)__MMIO_P2V(P7_SYS);
			u32 tmp;
			/* put usb 0 ulpi in HiZ on p7mu */
			p7mu_write16(0xa00, 2);
			p7mu_write16(0xb00, 6);

			/* gpio input on p7 */
			/* 0xa00000 + 0x1000 */
			tmp = readl(sysctl + 0x1000 + 99*4);
			tmp |= 1;
			tmp &= (3<<9);
			writel(tmp, sysctl + 0x1000 + 99*4);

			tmp = readl(sysctl + 0x1000 + 101*4);
			tmp |= 1;
			tmp &= (3<<9);
			writel(tmp, sysctl + 0x1000 + 101*4);

			p7brd_init_usb(1, -1, CI_UDC_DR_HOST);
		}
	} else {
		/* HW1 is ok */
		drone_common_init_usb(86 /* VBUS_TABLET_EN */, 87 /* USB_VBUS_TABLET */,
				      88 /* #OC_TABLET */, 1);
		/* Init EHCI 1 */
		p7brd_init_usb(1, -1, CI_UDC_DR_HOST);
	}

	/* Init USB Hub */
	gpio_request_one(179, GPIOF_OUT_INIT_LOW,
			 "RESET_USB_HUB");
#if 0
	parrot_init_i2c_slave(HUB_I2C_BUS, &smsc82512_info, "smsc 82512",
			      P7_I2C_NOIRQ);
#endif

	/* Init sensors */
#if 0
	drone_common_init_ak8963(AK8963_I2C_BUS, ev_hsis.magneto_int_p7);
	drone_common_init_inv_mpu6050(MPU6050_I2C_BUS, ev_hsis.gyro_int_p7,
				      FSYNC_GYRO_FILTER, ev_hsis.clkin_gyro);
	drone_common_init_ms5607(MS5607_I2C_BUS);
#endif

	/* Init FAN */
	drone_common_init_fan(85);

	p7brd_export_gpio(P7_GPIO_NR(124), GPIOF_OUT_INIT_LOW, "POWER_KEEP");
	p7brd_export_gpio(9, GPIOF_OUT_INIT_LOW, "RESET_WIFI");
	p7brd_export_gpio(P7_GPIO_NR(132), GPIOF_OUT_INIT_LOW, "ipod-rst");
	p7brd_export_i2c_hw_infos(2, 0x10, "2C", "ipod");

	/* End of initialization */
	pr_info("Mpp board : init done\n");
}

static void __init mpp_reserve_mem(void)
{
	p7_reserve_nand_mem();
	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);
	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_MPP, "Mpp board")
	.reserve        = &mpp_reserve_mem,
	.init_machine   = &mpp_init_mach,
P7_MACHINE_END
