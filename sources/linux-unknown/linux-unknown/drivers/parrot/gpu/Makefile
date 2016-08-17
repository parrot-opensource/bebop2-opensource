ccflags-y += -I$(srctree)/drivers/parrot/gpu/ump/include/ump
obj-$(CONFIG_GPU_PARROT7) += p7gpu-pm.o
obj-$(CONFIG_UMP_PARROT7) += p7ump.o

ifneq ($(CONFIG_MALI400_SUPPORT),y)
obj-$(CONFIG_MALI200) += mali200/ 
obj-$(CONFIG_UMP) += ump/
endif

ifneq ($(CONFIG_MALI400_LEGACY),y)
	obj-$(CONFIG_UMP) += ump/
	obj-$(CONFIG_MALI400) += mali400/
else
	obj-$(CONFIG_UMP) += ump_legacy/
	obj-$(CONFIG_MALI400) += mali400_legacy/
endif

obj-y += ion/
