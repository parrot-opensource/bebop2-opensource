#
# Makefile for the linux kernel.
#

obj-y					:= v2m.o
obj-$(CONFIG_ARCH_VEXPRESS_CA9X4)	+= ct-ca9x4.o
obj-$(CONFIG_SMP)			+= platsmp.o
obj-$(CONFIG_HOTPLUG_CPU)		+= hotplug.o

CFLAGS_p7.o += -I$(srctree)/drivers/parrot
CFLAGS_camera-sensor-common.o += -I$(srctree)/drivers/parrot
CFLAGS_lcd-monspecs.o += -I$(srctree)/drivers/parrot
obj-$(CONFIG_ARCH_VEXPRESS_P7FPGA) += p7.o camera-sensor-common.o lcd-monspecs.o
