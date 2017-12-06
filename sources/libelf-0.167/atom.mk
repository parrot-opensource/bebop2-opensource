
LOCAL_PATH := $(call my-dir)

###############################################################################
# libelf
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libelf
LOCAL_DESCRIPTION := read, modify or create ELF files in an \
	architecture-independent way
LOCAL_CATEGORY_PATH := libs
LOCAL_LIBRARIES := zlib

LOCAL_AUTOTOOLS_VERSION := 0.167
LOCAL_AUTOTOOLS_ARCHIVE := elfutils-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := elfutils-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	SUBDIRS=libelf

LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	SUBDIRS=libelf

include $(BUILD_AUTOTOOLS)

