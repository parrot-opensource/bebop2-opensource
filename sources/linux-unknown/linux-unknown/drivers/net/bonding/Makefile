#
# Makefile for the Ethernet Bonding driver
#

obj-$(CONFIG_BONDING) += bonding.o

bonding-objs := bond_main.o bond_3ad.o bond_alb.o bond_debugfs.o

ifeq ($(CONFIG_BONDING_SYSFS),y)
	bonding-objs += bond_sysfs.o
endif

proc-$(CONFIG_PROC_FS) += bond_procfs.o
bonding-objs += $(proc-y)

