
# Linux, p7 cpu
ifeq ("$(TARGET_OS)","linux")
ifeq ("$(TARGET_CPU)","p7")

ifndef BUILD_LINUX
$(error update alchemy to version 1.0.5 or more)
endif

ifndef TARGET_LINUX_CROSS
TARGET_LINUX_CROSS := /opt/arm-2012.03/bin/arm-none-linux-gnueabi-
endif

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := linux
LOCAL_CATEGORY_PATH := system

LINUX_EXPORTED_HEADERS := \
	$(LOCAL_PATH)/drivers/parrot/sound/p7_aai/aai_ioctl.h \
	$(LOCAL_PATH)/drivers/parrot/media/p7-mpegts_ioctl.h \
	$(LOCAL_PATH)/drivers/parrot/gpu/ion/p7-ion.h


# Linux configuration file
LINUX_DEFAULT_CONFIG_TARGET := p7_defconfig

include $(BUILD_LINUX)

include $(CLEAR_VARS)

LOCAL_MODULE := perf
LOCAL_CATEGORY_PATH := devel

include $(BUILD_LINUX)


endif
endif
