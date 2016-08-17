#
# Parrot7 platforms Linux makefile
#

# Allow platform to access Parrot driver APIs
ccflags-y   += -I$(srctree)/drivers/parrot

# Add search path for sdhci-p7
ccflags-y   += -I$(srctree)/drivers/mmc

# Add search path for chipidea usb
ccflags-y += -I$(srctree)/drivers/usb

# Add search path for mali gpu
ifneq (,$(filter $(CONFIG_MALI200),y m))
ccflags-y += -I$(srctree)/drivers/parrot/gpu/mali200/linux
endif
ifneq (,$(filter $(CONFIG_MALI400),y m))
ccflags-y += -I$(srctree)/drivers/parrot/gpu/mali400/common
ccflags-y += -I$(srctree)/drivers/parrot/gpu/mali400/linux
ccflags-y += -I$(srctree)/drivers/parrot/gpu/mali400/include
endif

# Enable platform debug and verbose output if required.
ccflags-$(CONFIG_ARCH_PARROT7_DEBUG)    += -DDEBUG

# Common code base for all Parrot7 boards.
obj-y   += common.o clock.o devices.o iopin.o core.o mpmc.o
obj-$(CONFIG_COMMON_CLK) += clock_data.o
obj-y	+= p7-smc.o
ifeq ($(CONFIG_PM),y)
obj-y   += pm.o
obj-y   += sleep.o
endif

obj-$(CONFIG_ARCH_PARROT7_BOARD)            += board.o
obj-$(CONFIG_PINCTRL_PARROT7)               += pinctrl.o
obj-$(CONFIG_PINCTRL_PARROT7)               += pindesc-r1.o pindesc-r2.o
obj-$(CONFIG_PINCTRL_PARROT7)               += pindesc-r3.o
obj-$(if $(CONFIG_ION),y,n)                 += ion.o
obj-$(if $(CONFIG_GPIO_PARROT7),y,n)        += gpio.o
obj-$(if $(CONFIG_I2CM_PARROT7),y,n)        += i2cm.o
obj-$(if $(CONFIG_PLDS_I2CS),y,n)           += i2cs.o
obj-$(if $(CONFIG_SERIAL_PARROTX),y,n)      += uart.o
obj-$(if $(CONFIG_PWM_PARROT7),y,n)         += p7_pwm.o
obj-$(if $(CONFIG_MTD_NAND_CAST),y,n)       += nand.o
obj-$(if $(CONFIG_MMC_SDHCI_ACS3),y,n)      += sdhci.o
obj-$(if $(CONFIG_UIO_HX270),y,n)           += vdec.o
obj-$(if $(CONFIG_UIO_HX280),y,n)           += venc.o
obj-$(if $(CONFIG_DMA_PARROT7),y,n)         += dma.o
obj-$(if $(CONFIG_MFD_P7MU),y,n)            += p7mu.o
obj-$(if $(CONFIG_SPI_PARROT7),y,n)         += spi.o
obj-$(if $(CONFIG_MPEGTS_PARROT7),y,n)      += mpegts.o
obj-$(if $(CONFIG_USB_CHIPIDEA_HOST),y,n)         += usb.o
obj-$(if $(CONFIG_USB_GADGET),y,n)	    += usb.o
obj-$(if $(CONFIG_SND_AAI),y,n)             += aai.o
obj-$(if $(CONFIG_AVI),y,n)                 += avi.o
obj-y					    += lcd-monspecs.o
obj-$(if $(CONFIG_GPU_PARROT7),y,n)         += gpu-m200.o
obj-$(if $(CONFIG_GPU_PARROT7),y,n)         += gpu-m400.o
obj-$(if $(CONFIG_GPU_PARROT7),y,n)         += gpu.o
obj-$(if $(CONFIG_AXIMON_PARROT7),y,n)      += aximon.o
obj-$(if $(CONFIG_STMMAC_ETH),y,n)          += ether.o
obj-$(if $(CONFIG_P7_TEMPERATURE),y,n)            += p7_temperature.o
obj-$(if $(CONFIG_CAN_C_CAN_PLATFORM),y,n)        += c_can.o
obj-$(if $(CONFIG_CRYPTO_DEV_P7),y,n)        += crypto.o
ifneq ($(CONFIG_BACKLIGHT_PWM),)
ifneq ($(CONFIG_FB_AVI),)
	obj-y   += backlight.o
endif
endif
obj-$(if $(CONFIG_WILINK_PLATFORM_DATA),y,n) += wl18xx.o

obj-y   += board-common.o
obj-$(CONFIG_SYSFS) += board-sysfs.o

obj-$(CONFIG_MACH_PARROT_FC7100_FAMILY)    += fc7100-module.o
obj-$(CONFIG_MACH_PARROT_FC7100_FAMILY)    += fc7100-mezzs2-avi.o fc7100-module-lcd.o

#fc7000 module based
obj-$(CONFIG_MACH_PARROT_FC7100)           += board-fc7100-workbench.o board-fc7100-workbench-variant.o fc7100-mezzs.o
obj-$(CONFIG_MACH_PARROT_FC7100)           += board-fc7100-workbench-yoshi.o

obj-$(CONFIG_MACH_PARROT_ANTEC)            += board-fc7100-antec.o
obj-$(CONFIG_MACH_PARROT_SOCA)             += board-fc7100-soca.o
obj-$(CONFIG_MACH_PARROT_DTV)              += board-fc7100-dtv.o
obj-$(CONFIG_MACH_PARROT_RTV)              += board-fc7100-rtv.o
obj-$(CONFIG_MACH_PARROT_AGU)              += board-fc7100-agu.o
obj-$(CONFIG_MACH_PARROT_PORSCHE_RSE)      += board-fc7100-porsche-rse.o
obj-$(CONFIG_MACH_PARROT_DRSE)             += board-fc7100-drse.o
obj-$(CONFIG_MACH_PARROT_EMC)              += board-fc7100-emc.o
obj-$(CONFIG_MACH_PARROT_NAP)              += board-fc7100-nap.o
obj-$(CONFIG_MACH_PARROT_VOLVO_TRUCK)      += board-fc7100-volvo-truck.o
obj-$(CONFIG_MACH_PARROT_VTNA)		   += board-fc7100-vtna.o
obj-$(CONFIG_MACH_PARROT_BORGWARD)	   += board-fc7100-borgward.o
obj-$(CONFIG_MACH_PARROT_FC7100_TB)        += board-fc7100-testbench.o
obj-$(CONFIG_MACH_PARROT_MEZZ_DEV)         += board-fc7100-mezz-dev.o

obj-$(CONFIG_MACH_PARROT_DRONE_COMMON)     += board-drone-common.o
obj-$(CONFIG_MACH_PARROT_MYKONOS3)         += board-mykonos3.o
obj-$(CONFIG_MACH_PARROT_EVINRUDE)         += board-evinrude.o
obj-$(CONFIG_MACH_PARROT_SFX1)             += board-sfx1.o
obj-$(CONFIG_MACH_PARROT_SFX1B)            += board-sfx1b.o
obj-$(CONFIG_MACH_PARROT_BEBOP)            += board-bebop.o
obj-$(CONFIG_MACH_PARROT_RNB6)             += board-rnb6.o
obj-$(CONFIG_MACH_PARROT_PAROS)            += board-paros.o
obj-$(CONFIG_MACH_PARROT_SICILIA)          += board-sicilia.o
obj-$(CONFIG_MACH_PARROT_GALILEI)          += board-galilei.o
obj-$(CONFIG_MACH_PARROT_MPP)              += board-mpp.o

# P7Dev related
obj-$(CONFIG_MACH_PARROT_P7DEV)             += board-p7dev.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_NOR)        += board-p7dev-nor.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_FPGA)       += board-p7dev-fpga.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_RNB6)       += board-p7dev-rnb6.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_CAMERA)     += board-p7dev-camera.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_GALILEO2)   += board-p7dev-galileo2.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_SICILIA)    += board-p7dev-sicilia.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_HDMI)       += board-p7dev-hdmi.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_SPI)        += board-p7dev-spi.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_QUAD_NOR)   += board-p7dev-quad.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_SND_AAI)    += board-p7dev-aai.o
obj-$(CONFIG_ARCH_PARROT7_P7DEV_TUNER)      += board-p7dev-tuner.o
# Zebu platform specific
obj-$(CONFIG_MACH_PARROT_ZEBU)         += board-zebu.o

# Remove me once platform rework completed
#obj-$(CONFIG_SMP)                       += platsmp.o
#obj-$(CONFIG_HOTPLUG_CPU)               += hotplug.o
