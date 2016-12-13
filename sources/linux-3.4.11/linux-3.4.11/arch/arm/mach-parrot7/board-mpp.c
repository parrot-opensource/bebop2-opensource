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

/**********************
 * Magnetometer sensor
 **********************/

#if defined(CONFIG_AK8975) || defined(CONFIG_AK8975_MODULE)
#include <linux/platform_data/ak8975.h>

#define MPP_MAGN_NAME     "ak8963"
#define MPP_MAGN_I2C_BUS  (2)
#define MPP_MAGN_I2C_ADDR (0x0c)
#define MPP_MAGN_IRQ_GPIO (91)
#define MPP_MAGN_ORIENT   "0, 0, -1; 0, 1, 0; 1, 0, 0;"

static struct ak8975_platform_data mpp_magn_pdata = {
	.orientation = MPP_MAGN_ORIENT,
};

static struct i2c_board_info mpp_magn_info = {
	I2C_BOARD_INFO(MPP_MAGN_NAME, MPP_MAGN_I2C_ADDR),
	.platform_data = &mpp_magn_pdata,
	.irq = P7_GPIO_NR(MPP_MAGN_IRQ_GPIO),
};

static void __init mpp_init_magn(void)
{
	parrot_init_i2c_slave(MPP_MAGN_I2C_BUS, &mpp_magn_info, "Magnetometer",
			      P7_I2C_IRQ);
}
#else
#define mpp_init_magn()
#endif

/******************************
 * Inertial measurement sensor
 ******************************/

#if defined(CONFIG_INV_MPU6050_I2C) || defined(CONFIG_INV_MPU6050_I2C_MODULE)
#include <linux/platform_data/invensense_mpu6050.h>

#define MPP_IMU_NAME     "icm20608"
#define MPP_IMU_I2C_BUS  (1)
#define MPP_IMU_I2C_ADDR (0x68)
#define MPP_IMU_IRQ_GPIO (92)
#define MPP_IMU_ORIENT   { 0, 0, -1, -1, 0, 0, 0, 1, 0 }

static struct inv_mpu6050_platform_data mpp_imu_pdata = {
	.orientation = MPP_IMU_ORIENT
};

static struct i2c_board_info mpp_imu_info = {
	I2C_BOARD_INFO(MPP_IMU_NAME, MPP_IMU_I2C_ADDR),
	.platform_data = &mpp_imu_pdata,
	.irq = P7_GPIO_NR(MPP_IMU_IRQ_GPIO),
};

static void __init mpp_init_imu(void)
{
	parrot_init_i2c_slave(MPP_IMU_I2C_BUS, &mpp_imu_info, "IMU",
			      P7_I2C_IRQ);
}
#else
#define mpp_init_imu()
#endif

static void __init mpp_init_mach(void)
{
	/* Initialize ramoops */
	drone_common_init_ramoops();

	/* Init hardware revision independent stuff */
	p7_init_mach();

	/* Init GPIOs */
	p7_init_gpio(NULL, 0);

	/* Init debug UART */
	p7brd_init_uart(3, 0);

	/* Init BT UART */
	p7brd_init_uart(1, 1);

	/* Init NAND */
	p7brd_init_nand(0);

	/* Init I2C master
	 * I2CM-0:
	 *     P7MU
	 * I2CM-2:
	 *     AKM8963 (magneto)
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
		/* config pull up on GPIO 88 */
		p7_config_pin(88, P7CTL_PUD_CFG(UP));

		/* Init EHCI 1 */
		p7brd_init_usb(1, -1, CI_UDC_DR_HOST|CI_UDC_DR_DISABLE_HOST_WORKAROUND);
	}

	/* Init IMU, i.e. accelero + gyro */
	mpp_init_imu();

	/* Init magneto */
	mpp_init_magn();

	/* Init FAN */
	drone_common_init_fan(85);

	p7brd_export_gpio(P7_GPIO_NR(179), GPIOF_OUT_INIT_LOW, "RESET_USB_HUB");
	p7brd_export_gpio(P7_GPIO_NR(124), GPIOF_OUT_INIT_LOW, "POWER_KEEP");
	p7brd_export_gpio(P7_GPIO_NR(9), GPIOF_OUT_INIT_LOW, "RESET_WIFI");
	p7brd_export_gpio(P7_GPIO_NR(132), GPIOF_OUT_INIT_LOW, "ipod-rst");
	p7brd_export_i2c_hw_infos(2, 0x10, "2C", "ipod");

	/* End of initialization */
	pr_info("Mpp board : init done\n");
}

static void __init mpp_reserve_mem(void)
{
	drone_common_reserve_mem_ramoops();
	p7_reserve_nand_mem();
	p7_reserve_usb_mem(0);
	p7_reserve_usb_mem(1);
	p7_reserve_dmamem();
}

P7_MACHINE_START(PARROT_MPP, "Mpp board")
	.reserve        = &mpp_reserve_mem,
	.init_machine   = &mpp_init_mach,
P7_MACHINE_END
