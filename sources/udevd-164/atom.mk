LOCAL_PATH := $(call my-dir)

################################################################################
# udev meta-package
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udev
LOCAL_DESCRIPTION := Meta-package which selects all udev components
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_REQUIRED_MODULES := libudev libudev-private udevd udevadm

include $(BUILD_META_PACKAGE)

################################################################################
# libudev
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libudev
LOCAL_DESCRIPTION := Client library for interacting with udev
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_CFLAGS := \
	-include "include/config.h"

LOCAL_CFLAGS += -Wno-unused-result
ifeq ("$(TARGET_CC_VERSION)","4.3.3")
LOCAL_CFLAGS += \
	-DSOCK_CLOEXEC=02000000
endif

LOCAL_EXPORT_C_INCLUDES := \
	$(LOCAL_PATH)/libudev/

LOCAL_SRC_FILES := \
	libudev/libudev.c \
	libudev/libudev-list.c \
	libudev/libudev-util.c \
	libudev/libudev-device.c \
	libudev/libudev-enumerate.c \
	libudev/libudev-monitor.c \
	libudev/libudev-queue.c \
	libudev/libudev-provider.c

include $(BUILD_SHARED_LIBRARY)

################################################################################
# libudev-private
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libudev-private
LOCAL_DESCRIPTION := Private library for udev executables
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_CFLAGS := \
	-Wno-unused-result \
	-include "include/config.h"

LOCAL_EXPORT_C_INCLUDES := \
	$(LOCAL_PATH)/libudev/

LOCAL_SRC_FILES := \
	libudev/libudev-ctrl.c \
	libudev/libudev-util-private.c \
	libudev/libudev-device-private.c \
	libudev/libudev-queue-private.c

LOCAL_LIBRARIES := libudev

include $(BUILD_SHARED_LIBRARY)

################################################################################
# udevd
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udevd
LOCAL_DESCRIPTION := Daemon monitoring kernel events concerning hardware
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_CFLAGS := \
	-Wno-unused-result \
	-include "include/config.h"

ifeq ("$(TARGET_CC_VERSION)","4.3.3")
LOCAL_CFLAGS += \
	-DSOCK_CLOEXEC=02000000 \
	-DIN_CLOEXEC=02000000
endif

LOCAL_SRC_FILES := \
	udev/udev-event.c \
	udev/udev-watch.c \
	udev/udev-node.c \
	udev/udev-rules.c \
	udev/udev-perms.c \
	\
	udev/udevd.c \
	udev/sd-daemon.c \
	udev/udevd_init.c \
	udev/udevadm-trigger.c

LOCAL_LIBRARIES := libudev libudev-private

LOCAL_COPY_FILES := \
	scripts/30-udev.rc:etc/boxinit.d/

include $(BUILD_EXECUTABLE)

################################################################################
# udevadm
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udevadm
LOCAL_DESCRIPTION := Configuration and monitoring for udev
LOCAL_CATEGORY_PATH := devman/udev
LOCAL_REQUIRED_MODULES := udevd

LOCAL_CFLAGS := \
	-include "include/config.h"

LOCAL_CFLAGS += -Wno-unused-result
ifeq ("$(TARGET_CC_VERSION)","4.3.3")
LOCAL_CFLAGS += \
	-DIN_CLOEXEC=02000000
endif

LOCAL_SRC_FILES := \
	udev/udev-event.c \
	udev/udev-watch.c \
	udev/udev-node.c \
	udev/udev-rules.c \
	udev/udev-perms.c \
	\
	udev/udevadm.c \
	udev/udevadm-info.c \
	udev/udevadm-control.c \
	udev/udevadm-test.c \
	udev/udevadm-monitor.c \
	udev/udevadm-settle.c \
	udev/udevadm-trigger.c

LOCAL_LIBRARIES := libudev libudev-private

include $(BUILD_EXECUTABLE)
