LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libdaemon
LOCAL_DESCRIPTION := Libdaemon library
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 0.14
LOCAL_AUTOTOOLS_ARCHIVE := libdaemon-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := libdaemon-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_CONFIGURE_ARGS += \
		ac_cv_func_setpgrp_void=yes \
		--disable-lynx

include $(BUILD_AUTOTOOLS)

