/**
 * linux/arch/arm/mach-parrot7/board-drone-common.h - Parrot7 based boards
 *                                                    drone common interface
 *
 * Copyright (C) 2015 Parrot S.A.
 *
 * author:  Alexandre Dilly <alexandre.dilly@parrot.com>
 * date:    7-May-2015
 *
 * This file is released under the GPL
 */

#ifndef _DRONE_COMMON_BOARD_H
#define _DRONE_COMMON_BOARD_H

#include "pinctrl.h"
#include "avi.h"
#include <iio/platform_data/bldc_cypress.h>

/* Driver usage */
#if defined(CONFIG_VIDEO_MT9F002) || defined(CONFIG_VIDEO_MT9F002_MODULE)
#define DRIVER_VIDEO_MT9F002
#else
#undef DRIVER_VIDEO_MT9F002
#endif

#if defined(CONFIG_VIDEO_MT9V117) || defined(CONFIG_VIDEO_MT9V117_MODULE)
#define DRIVER_VIDEO_MT9V117
#else
#undef DRIVER_VIDEO_MT9V117
#endif

#if defined(CONFIG_PARROT_IIO_AK8975) ||\
	defined(CONFIG_PARROT_IIO_AK8975_MODULE) ||\
	defined(CONFIG_AK8975) ||\
	defined(CONFIG_AK8975_MODULE)
#define DRIVER_PARROT_IIO_AK8975
#else
#undef DRIVER_PARROT_IIO_AK8975
#endif

#if defined(CONFIG_PARROT_IIO_INV_MPU6050) ||\
	defined(CONFIG_PARROT_IIO_INV_MPU6050_MODULE) ||\
	defined(CONFIG_INV_MPU6050_IIO) ||\
	defined(CONFIG_INV_MPU6050_IIO_MODULE)
#define DRIVER_PARROT_IIO_INV_MPU6050
#else
#undef DRIVER_PARROT_IIO_INV_MPU6050
#endif

#if defined(CONFIG_PARROT_IIO_MS5607) ||\
	defined(CONFIG_PARROT_IIO_MS5607_MODULE) ||\
	defined(CONFIG_IIO_MS5607) ||\
	defined(CONFIG_IIO_MS5607_MODULE)
#define DRIVER_PARROT_IIO_MS5607
#else
#undef DRIVER_PARROT_IIO_MS5607
#endif

#if defined(CONFIG_IIO_PARROT_BLDC_CYPRESS) || \
	defined(CONFIG_IIO_PARROT_BLDC_CYPRESS_MODULE)
#define DRIVER_PARROT_IIO_BLDC_CYPRESS
#else
#undef DRIVER_PARROT_IIO_BLDC_CYPRESS
#endif


/* Commandline parser for FAN activation.
 * By default, drone_fan is command line arg.
 * Possible values are on (or 1) and off (or 0).
 */
int __init drone_common_set_fan(char *option);

/* Initialize P7MU.
 * If check_availability is set, presence of P7Mu on i2c-0 is checked and if it
 * is not detected, P7MU is probed on i2c-1.
 */
void __init drone_common_init_p7mu(int gpio_int, int check_availability);

/* Initialize SDHCI for eMMC.
 * Use default mapping when maps is NULL.
 */
void __init drone_common_init_sdhci(struct pinctrl_map *pins, int size);

/* Initialize Ultrasound TX (SPI).
 * RX part is initialized during P7MU init.
 */
void __init drone_common_init_us_tx(void);

/* Initialize cameras.
 *  -> drone_common_init_cam_h_mt9f002() for mt9f002 has horizontal camera
 *  -> drone_common_init_cam_v_mt9v117() for mt9v117 has vertical camera
 * The AVI must be initialized first with p7_init_avi().
 */
void __init drone_common_init_cam_h_mt9f002(int gpio_pwm, int gpio_en, union avi_cam_interface *cam_interface, struct pinctrl_map *cam_pins, size_t pin_cnt);
void __init drone_common_init_cam_v_mt9v117(int gpio_pwm, int gpio_en);

/* Initialize MEM2MEM.
 * Use default channels when pdata is NULL.
 */
void __init drone_common_init_m2m(struct avi_m2m_platform_data *pdata);

/* Initialize USB.
 */
void __init drone_common_init_usb(int gpio_on, int gpio_host_mode_3v3,
				  int gpio_usb0_oc, int is_host);

/* Initialize sensors.
 *  -> drone_common_init_ak8963() for AK8963 magnetometer
 *  -> drone_common_init_inv_mpu6050() for MPU6050 IMU
 *  -> drone_common_init_ms5607() fpr MS5607 Pressure/Temperature
 * When irq is equal to -1, NOIRQ is used for I2C init.
 * For MPU6050, filter_rate indicates the gyro FSYNC interrupt filter rate.
 */
void __init drone_common_init_ak8963(int i2c_bus, int irq);
void __init drone_common_init_inv_mpu6050(int i2c_bus, int irq,
					  int filter_rate, int clkin_pwm);
void __init drone_common_init_ms5607(int i2c_bus);

/* Initialize BLDC controller.
 */
void __init drone_common_init_bldc(int i2c_bus, int gpio_reset);
void __init drone_common_init_bldc_with_lut(int i2c_bus, int gpio_reset,
				struct bldc_cypress_platform_data *pdata);

/* Initialize FAN.
 * It uses value from command line (see drone_common_set_fan()).
 */
void __init drone_common_init_fan(int gpio);

/* RamOOPS.
 * Default size is 1M with 128k record size.
 * The function drone_common_reserve_mem_ramoops() must be called before
 * drone_common_reserve_mem().
 */
void __init drone_common_init_ramoops(void);
void __init drone_common_reserve_mem_ramoops(void);

/* Reserve persistent memory.
 * Use default AVI memory size when avi_size is equal to 0.
 */
void __init drone_common_reserve_mem(size_t avi_size);

/* Calculate board revision from a GPIO list: the first GPIO is the LSB and the
 * last GPIO in list is the MSB. A name can be passed for GPIO initialization.
 */
int __init drone_common_get_rev(int *gpios, int n_gpio, const char *name);

/* Export a GPIO.
 */
void __init drone_common_export_gpio(int gpio, int flags, const char *label);

/* Structure for GPIO exporting */
struct drone_common_gpio {
	int *gpio;
	unsigned int flags;
	char *label;
};

#define DRONE_COMMON_GPIO(_gpio, _flags, _label)\
	{\
		.gpio = _gpio,\
		.flags = _flags,\
		.label = _label,\
	}

/* Export GPIO list of type struct drone_common_gpio.
 * This list is ended with a record where gpio field is NULL.
 */
void __init drone_common_export_gpios(struct drone_common_gpio *gpios);

/* Structure for HSIS export to sysfs */
struct drone_common_hsis_sysfs_attr {
	struct kobj_attribute attr;
	int *value;
};

#define DRONE_COMMON_HSIS_SYSFS_ATTR(_name, _value)\
	{\
		.attr =\
		       __ATTR(_name, 0644, drone_common_hsis_sysfs_show,\
			      drone_common_hsis_sysfs_store),\
		.value = _value,\
	}

/* Common drone sysfs callbacks. */
ssize_t drone_common_hsis_sysfs_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf);
ssize_t drone_common_hsis_sysfs_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count);

/* Create a /sys/kernel/hsis entry and fill with a list of type struct
 * drone_common_hsis_sysfs_attr.
 * This list is ended with a record where value field is NULL.
 */
int __init drone_common_init_sysfs(struct drone_common_hsis_sysfs_attr *attrs);

#endif /* #ifdef _DRONE_COMMON_BOARD_H */
