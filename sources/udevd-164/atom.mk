LOCAL_PATH := $(call my-dir)

################################################################################
# udev meta-package
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udev
LOCAL_DESCRIPTION := Meta-package which selects all udev components
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_DEPENDS_MODULES := \
	libudev \
	libudev-private \
	udevd \
	udevadm \
	udevd_init \

# TODO change the BUILD_CUSTOM to the definitive name of BUILD_META_PACKAGE
include $(BUILD_CUSTOM)

################################################################################
# libudev
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libudev
LOCAL_DESCRIPTION := Client library for interacting with udev
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_CFLAGS := \
	-include "include/config.h"

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
	libudev/libudev-queue.c

LOCAL_LIBRARIES := \
	libpac

include $(BUILD_SHARED_LIBRARY)

################################################################################
# libudev-private
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libudev-private
LOCAL_DESCRIPTION := Private library for udev executables
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_CFLAGS := \
	-include "include/config.h"
	
LOCAL_EXPORT_C_INCLUDES := \
	$(LOCAL_PATH)/libudev/

LOCAL_SRC_FILES := \
	libudev/libudev-ctrl.c \
	libudev/libudev-util-private.c \
	libudev/libudev-device-private.c \
	libudev/libudev-queue-private.c

LOCAL_LIBRARIES := \
	libpac \
	libudev

include $(BUILD_SHARED_LIBRARY)

################################################################################
# udevd
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udevd
LOCAL_DESCRIPTION := Daemon monitoring kernel events concerning hardware
LOCAL_CATEGORY_PATH := devman/udev
LOCAL_DEPENDS_MODULES := udevd_init

LOCAL_CFLAGS := \
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
	\
	udev/udevd.c \
	udev/sd-daemon.c

LOCAL_LIBRARIES := \
	libudev \
	libudev-private \
	libpac

LOCAL_COPY_FILES := \
	scripts/udevd.sh:sbin/

ifdef UDEVD_NATIVE_SCRIPT
LOCAL_COPY_FILES += \
	scripts/udevd_native.sh:sbin/
endif

include $(BUILD_EXECUTABLE)

################################################################################
# udevadm
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udevadm
LOCAL_DESCRIPTION := Configuration and monitoring for udev
LOCAL_CATEGORY_PATH := devman/udev
LOCAL_DEPENDS_MODULES := udevd

LOCAL_CFLAGS := \
	-include "include/config.h"

ifeq ("$(TARGET_CC_VERSION)","4.3.3")
LOCAL_CFLAGS += \
	-DIN_CLOEXEC=02000000
endif

LOCAL_SRC_FILES := \
	udev/udev-event.c \
	udev/udev-watch.c \
	udev/udev-node.c \
	udev/udev-rules.c \
	\
	udev/udevadm.c \
	udev/udevadm-info.c \
	udev/udevadm-control.c \
	udev/udevadm-test.c \
	udev/udevadm-monitor.c \
	udev/udevadm-settle.c \
	udev/udevadm-trigger.c

LOCAL_LIBRARIES := \
	libudev \
	libudev-private \
	libpac

include $(BUILD_EXECUTABLE)

################################################################################
# udevd_init
################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := udevd_init
LOCAL_DESCRIPTION := Launcher for the udevd daemon
LOCAL_CATEGORY_PATH := devman/udev

LOCAL_CFLAGS := \
	-include "include/config.h"

LOCAL_SRC_FILES := \
	udev/udevadm-settle.c \
	udev/udevadm-trigger.c \
	udev/udevd_init.c

LOCAL_LIBRARIES := \
	libudev \
	libudev-private \
	libpac

include $(BUILD_EXECUTABLE)

