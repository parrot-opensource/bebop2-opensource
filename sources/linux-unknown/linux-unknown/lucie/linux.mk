#############################################################
#
# linux
#
#############################################################

LINUX_DIR:=$(PACKAGE_DIR)/linux
LINUX_EXPORTED_HEADERS := \
	$(LINUX_DIR)/drivers/parrot/sound/p7_aai/aai_ioctl.h \
#	$(LINUX_DIR)/drivers/parrot/video/p6fb_ioctl.h \
#	$(LINUX_DIR)/drivers/parrot/char/dmamem_ioctl.h \
#	$(LINUX_DIR)/drivers/parrot/char/pwm/pwm_ioctl.h

ifndef LINUXCONF_MK
	$(error LINUXCONF_MK is not defined)
endif

include $(LINUXCONF_MK)

#XXX overide toolchain
LINUX_MAKE_ARGS= \
				 ARCH=$(ARCH) \
				 CROSS_COMPILE=/opt/arm-2012.03/bin/arm-none-linux-gnueabi- \
				 -C $(LINUX_DIR) \
				 INSTALL_MOD_PATH=$(TARGET_DIR) \
				 INSTALL_MOD_STRIP=1 \
				 O=$(LINUX_DIR_BUILD)
