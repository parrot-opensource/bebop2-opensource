LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libnss-mdns
LOCAL_DESCRIPTION := Library nss for avahi mdns resolution
LOCAL_CATEGORY_PATH := libs

LOCAL_AUTOTOOLS_VERSION := 0.10
LOCAL_AUTOTOOLS_ARCHIVE := nss-mdns-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := nss-mdns-$(LOCAL_AUTOTOOLS_VERSION)

# Remove unused debug feature which might prevent building
LOCAL_ARCHIVE_PATCHES := remove_debug_trap.patch

LOCAL_EXPORT_LDLIBS := -lnss_dns

include $(BUILD_AUTOTOOLS)
