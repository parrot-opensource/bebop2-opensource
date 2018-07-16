LOCAL_PATH := $(call my-dir)

###############################################################################
# mtpprobe
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := mtp-probe
LOCAL_DESCRIPTION := Helper used to detect mtp-devices, from libmtp
LOCAL_CATEGORY_PATH := multimedia/mtp

LOCAL_LIBRARIES := libusb_1_0 libmtp-sf

LOCAL_SRC_FILES := \
	util/mtp-probe.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/src/

LOCAL_DESTDIR := lib/udev

LOCAL_CFLAGS := -Wno-sign-compare

include $(BUILD_EXECUTABLE)

###############################################################################
# libMTP
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libmtp-sf
LOCAL_DESCRIPTION := libmtp
LOCAL_CATEGORY_PATH := multimedia/mtp

LOCAL_CONFIG_FILES := Config.in
$(call load-config)

LOCAL_LIBRARIES := libusb_1_0
LOCAL_CONDITIONAL_LIBRARIES := OPTIONAL:libcutils
LOCAL_CONDITIONAL_LIBRARIES += OPTIONAL:libulog

LOCAL_SRC_FILES := \
	src/util.c \
	src/playlist-spl.c \
	src/libusb1-glue.c \
	src/unicode.c \
	src/libmtp.c \
	src/ptp.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/src/ \

LOCAL_EXPORT_C_INCLUDES := \
	$(LOCAL_PATH)/src/

LOCAL_DESTDIR := usr/lib

LOCAL_CFLAGS := -Wno-sign-compare

include $(BUILD_SHARED_LIBRARY)

###############################################################################
# examples
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := mtp-detect
LOCAL_DESCRIPTION := libmtp example: detect devices
LOCAL_CATEGORY_PATH := multimedia/mtp

LOCAL_LIBRARIES := libmtp-sf

LOCAL_SRC_FILES := \
	examples/detect.c

LOCAL_C_INCLUDES := \
	$(LOCAL_PATH)/examples

LOCAL_CFLAGS := -Wno-sign-compare

include $(BUILD_EXECUTABLE)
