/*
 * INVENSENSE MPU6050 Accelerometer driver
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Kishore Kadiyala <kishore.kadiyala@ti.com>
 * Contributor: Leed Aguilar <leed.aguilar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/input/mpu6050.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include "mpu6050x.h"

#define MPU6050_ACCELERO_DATA_DBG 0
#define MPU6050_GYRO_DATA_DBG 0
#define MPU6050_GPIO_DBG 0 // Enable gpio debugging (pulse during it handling)
#define MPU6050_GPIO_NUM 134 // Gpio number for gpio debugging

/*MPU6050 full scale range types conversion table */
/* Shift left the value to get in in the same scale as the most precise one */
static int mpu6050_accel_shift_table[4] = {
    0,
    1,
    2,
    3,
};
static int mpu6050_gyro_shift_table[4] = {
    0,
    1,
    2,
    3,
};
/* Range of accelero sensor */
#define MPU6050_ACCEL_RANGE_MAX (16384 * 16) // +16g @ 16384 LSB / g
#define MPU6050_GYRO_RANGE_MAX (131 * 2000) // 2000 °/s @ 131 LSB / °/s


static inline int mpu6050_acc_convert(int16_t val, int fsr)
{
    int newval = (int)val;
    newval <<= mpu6050_accel_shift_table[fsr >> 3];
    return newval;
}

static inline int mpu6050_gyro_convert(int16_t val, int fsr)
{
    int newval = (int)val;
    newval <<= mpu6050_gyro_shift_table[fsr >> 3];
    return newval;
}

/**
 * mpu6050_accel_set_standby - Put's the accel axis in standby or not
 * @mpu6050_data: accelerometer driver data
 * @enable: to put into standby mode or not
 */
static void mpu6050_accel_set_standby(struct mpu6050_data *data,
                                      uint8_t enable)
{
    uint8_t val = 0;

    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR, MPU6050_REG_PWR_MGMT_2,
                 1, &val, "Accel STBY");
    if (enable)
        val |= (MPU6050_STBY_XA |  MPU6050_STBY_YA |  MPU6050_STBY_ZA);
    else
        val &= ~(MPU6050_STBY_XA |  MPU6050_STBY_YA |  MPU6050_STBY_ZA);

    MPU6050_WRITE(data, MPU6050_CHIP_I2C_ADDR,
                  MPU6050_REG_PWR_MGMT_2, val, "Accel STBY");
}

/**
 * mpu6050_gyro_set_standby - Put's the gyro axis in standby or not
 * @mpu6050_gyro_data: gyroscope driver data
 * @enable: to put into standby mode or not
 */
static void mpu6050_gyro_set_standby(struct mpu6050_data *data,
                                     uint8_t enable)
{
    unsigned char val = 0;

    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR, MPU6050_REG_PWR_MGMT_2,
                 1, &val, "Gyro STBY");
    if (enable)
        val |= (MPU6050_STBY_XG |  MPU6050_STBY_YG |  MPU6050_STBY_ZG);
    else
        val &= ~(MPU6050_STBY_XG |  MPU6050_STBY_YG |  MPU6050_STBY_ZG);

    MPU6050_WRITE(data, MPU6050_CHIP_I2C_ADDR,
                  MPU6050_REG_PWR_MGMT_2, val, "Gyro STBY");
}

/**
 * mpu6050_gyro_reset - Reset's the signal path of gyroscope
 * @mpu6050_gyro_data: gyroscope driver data
 */
static int mpu6050_gyro_reset(struct mpu6050_data *data)
{
    MPU6050_WRITE(data, MPU6050_CHIP_I2C_ADDR,
                  MPU6050_REG_SIGNAL_PATH_RESET, MPU6050_GYRO_SP_RESET,
                  "Gyro SP-reset");
    mpu6050_gyro_set_standby(data, 0);
    return 0;
}

/**
 * mpu6050_accel_reset - Reset's the signal path of acceleromter
 * @mpu6050_data: accelerometer driver data
 */
static void mpu6050_accel_reset(struct mpu6050_data *data)
{
    MPU6050_WRITE(data, MPU6050_CHIP_I2C_ADDR,
                  MPU6050_REG_SIGNAL_PATH_RESET, MPU6050_ACCEL_SP_RESET,
                  "Accel SP-reset");
    mpu6050_accel_set_standby(data, 0);
}

static void mpu6050_accel_read_xyz(struct mpu6050_data *data)
{
    uint8_t fsr;
    int16_t datax, datay, dataz;
    int convx, convy, convz;
    int16_t buffer[3];

    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR,
                 MPU6050_REG_ACCEL_XOUT_H, 6, (uint8_t *)buffer, "Accel-x-y-z");

    datax = be16_to_cpu(buffer[0]);
    datay = be16_to_cpu(buffer[1]);
    dataz = be16_to_cpu(buffer[2]);

    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR,
                 MPU6050_REG_ACCEL_CONFIG, 1, &fsr, "full scale range");

    convx = mpu6050_acc_convert(datax, fsr);
    convy = mpu6050_acc_convert(datay, fsr);
    convz = mpu6050_acc_convert(dataz, fsr);

#if MPU6050_ACCELERO_DATA_DBG
    pr_info("Accelero: x = %d LSB, y = %d LSB, z = %d LSB\n\n",
            convx, convy, convz);
#endif

    input_report_abs(data->input_dev, ABS_X, convx);
    input_report_abs(data->input_dev, ABS_Y, convy);
    input_report_abs(data->input_dev, ABS_Z, convz);
}

void mpu6050_gyro_read_xyz(struct mpu6050_data *data)
{
    int16_t datax, datay, dataz;
    int convx, convy, convz;
    int16_t buffer[3];
    uint8_t fsr;

    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR, MPU6050_REG_GYRO_XOUT_H,
                 6, (uint8_t *)buffer, "gyro-x-y-z");

    datax = be16_to_cpu(buffer[0]);
    datay = be16_to_cpu(buffer[1]);
    dataz = be16_to_cpu(buffer[2]);

    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR,
                 MPU6050_REG_GYRO_CONFIG, 1, &fsr, "Gyro full scale range");

    convx = mpu6050_gyro_convert(datax, fsr);
    convy = mpu6050_gyro_convert(datay, fsr);
    convz = mpu6050_gyro_convert(dataz, fsr);

#if MPU6050_GYRO_DATA_DBG
    pr_info("Gyro: x = %d LSB, y = %d LSB, z = %d LSB\n\n",
            convx, convy, convz);
#endif
    input_report_abs(data->input_dev, ABS_RX, convx);
    input_report_abs(data->input_dev, ABS_RY, convy);
    input_report_abs(data->input_dev, ABS_RZ, convz);
}

static void mpu6050_read_values(struct mpu6050_data *data)
{
    uint8_t val;

#if MPU6050_GPIO_DBG
    gpio_set_value(MPU6050_GPIO_NUM, 1);
#endif

    /* Clear interrupts for requested mode */
    MPU6050_READ(data, MPU6050_CHIP_I2C_ADDR,
                 MPU6050_REG_INT_STATUS, 1, &val, "Clear Isr");

    mpu6050_accel_read_xyz(data);
    mpu6050_gyro_read_xyz(data);
    input_sync(data->input_dev);

#if MPU6050_GPIO_DBG
    gpio_set_value(MPU6050_GPIO_NUM, 0);
#endif

}

/**
 * mpu6050_accel_thread_irq - mpu6050 interrupt handler
 * @irq: device interrupt number
 * @dev_id: pointer to driver device data
 */
static irqreturn_t mpu6050_thread_irq(int irq, void *dev_id)
{
    struct mpu6050_data *data = dev_id;

    mpu6050_read_values(data);

    return IRQ_HANDLED;
}

struct mpu6050_data *mpu6050_accel_gyro_init(
    struct mpu6050_data *mpu_data)
{
    struct mpu6050_platform_data *pdata = mpu_data->pdata;
    struct input_dev *input_dev;
    int error;
    uint8_t val;

    /* Verify for IRQ line */
    if (!mpu_data->irq) {
        pr_err("%s: Accelerometer Irq not assigned\n", __func__);
        error = -EINVAL;
        goto err_out;
    }

    /* Input device allocation */
    input_dev = input_allocate_device();
    if (!input_dev) {
        error = -ENOMEM;
        goto err_free_mem;
    }

    mpu_data->input_dev = input_dev;
    /* Configure the init values from platform data */
    pdata->accel_fsr = 0;
    error = MPU6050_WRITE(mpu_data, MPU6050_CHIP_I2C_ADDR,
                          MPU6050_REG_ACCEL_CONFIG,
                          (pdata->accel_fsr << 3), "Init accel fsr");
    if (error) {
        dev_err(mpu_data->dev, "fail to configure accel FSR\n");
        goto err_free_input;
    }

    /* Configure the init values from platform data */
    pdata->gyro_fsr = 3; // 2000 deg/s
    MPU6050_WRITE(mpu_data, MPU6050_CHIP_I2C_ADDR,
                  MPU6050_REG_GYRO_CONFIG, (pdata->gyro_fsr << 3), "Init gyro fsr");
    if (error) {
        dev_err(mpu_data->dev, "fail to configure gyro FSR\n");
        goto err_free_input;
    }

    /* Configure the sample rate to 125Hz */
    // = 1000 / (1+val)
    // 200 -> 0x04 (4)  --> 1000 / (1+4)
    // 125 -> 0x07 (7)  --> 1000 / (1+7)
    // 100 -> 0x09 (9)  --> 1000 / (1+9)
    // 50  -> 0x13 (19) --> 1000 / (1+19)
    MPU6050_WRITE(mpu_data, MPU6050_CHIP_I2C_ADDR,
                  MPU6050_REG_SMPLRT_DIV, 0x09, "Init sample rate");
    if (error) {
        dev_err(mpu_data->dev, "fail to configure FSR\n");
        goto err_free_input;
    }

    /* Initialize the Input Subsystem configuration */
    input_dev->name = MPU6050_NAME;
    input_dev->id.bustype = mpu_data->bus_ops->bustype;

    __set_bit(EV_ABS, input_dev->evbit);

    input_set_abs_params(input_dev, ABS_X,
                         -MPU6050_ACCEL_RANGE_MAX,
                         MPU6050_ACCEL_RANGE_MAX, pdata->x_axis, 0);
    input_set_abs_params(input_dev, ABS_Y,
                         -MPU6050_ACCEL_RANGE_MAX,
                         MPU6050_ACCEL_RANGE_MAX, pdata->y_axis, 0);
    input_set_abs_params(input_dev, ABS_Z,
                         -MPU6050_ACCEL_RANGE_MAX,
                         MPU6050_ACCEL_RANGE_MAX, pdata->z_axis, 0);

    input_set_abs_params(input_dev, ABS_RX,
                         -MPU6050_GYRO_RANGE_MAX,
                         MPU6050_GYRO_RANGE_MAX, pdata->x_axis, 0);
    input_set_abs_params(input_dev, ABS_RY,
                         -MPU6050_GYRO_RANGE_MAX,
                         MPU6050_GYRO_RANGE_MAX, pdata->y_axis, 0);
    input_set_abs_params(input_dev, ABS_RZ,
                         -MPU6050_GYRO_RANGE_MAX,
                         MPU6050_GYRO_RANGE_MAX, pdata->z_axis, 0);

    input_set_drvdata(input_dev, mpu_data);

    /* Reset Accelerometer signal path */
    mpu6050_accel_reset(mpu_data);

    /* Reset Gyroscope signal path */
    mpu6050_gyro_reset(mpu_data);

    error = request_threaded_irq(mpu_data->irq, NULL,
                                 mpu6050_thread_irq,
                                 IRQF_TRIGGER_RISING | IRQF_SHARED,
                                 MPU6050_NAME, mpu_data);
    if (error) {
        dev_err(mpu_data->dev, "request_threaded_irq failed\n");
        goto err_free_input;
    }

    /* Register with the Input Subsystem */
    error = input_register_device(mpu_data->input_dev);
    if (error) {
        dev_err(mpu_data->dev, "Unable to register input device\n");
        goto err_free_irq;
    }

    /* Enable interrupt for requested mode */
    error = MPU6050_WRITE(mpu_data, MPU6050_CHIP_I2C_ADDR,
                          MPU6050_REG_INT_ENABLE, MPU6050_FF_INT, "Enable Intr");
    if (error) {
        dev_err(mpu_data->dev, "failed to enable interrupt\n");
        goto err_reg_device;
    }

    /* Clear interrupts for requested mode */
    error = MPU6050_READ(mpu_data, MPU6050_CHIP_I2C_ADDR,
                         MPU6050_REG_INT_STATUS, 1, &val, "Clear Isr");
    if (error) {
        dev_err(mpu_data->dev, "fail to clear ISR\n");
        goto err_reg_device;
    }

    return mpu_data;

err_reg_device:
    input_unregister_device(mpu_data->input_dev);
err_free_irq:
    free_irq(gpio_to_irq(mpu_data->irq), mpu_data);
err_free_input:
    input_free_device(mpu_data->input_dev);
err_free_mem:
err_out:
    return ERR_PTR(error);
}
EXPORT_SYMBOL(mpu6050_accel_gyro_init);

void mpu6050_accel_gyro_exit(struct mpu6050_data *data)
{
    mpu6050_accel_set_standby(data, 1);
    mpu6050_gyro_set_standby(data, 1);
    input_unregister_device(data->input_dev);
    free_irq(data->irq, data);
    input_free_device(data->input_dev);
    kfree(data);
}
EXPORT_SYMBOL(mpu6050_accel_gyro_exit);

MODULE_AUTHOR("Kishore Kadiyala <kishore.kadiyala@ti.com");
MODULE_DESCRIPTION("MPU6050 I2c Driver");
MODULE_LICENSE("GPL");
