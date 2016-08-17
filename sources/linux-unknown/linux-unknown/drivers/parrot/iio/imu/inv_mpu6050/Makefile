#
# Makefile for Invensense MPU6050 device.
#
ccflags-y += -I$(srctree)/drivers/parrot

obj-$(CONFIG_PARROT_IIO_INV_MPU6050) += inv-mpu6050.o
inv-mpu6050-objs := inv_mpu_core.o inv_mpu_ring.o inv_mpu_trigger.o
