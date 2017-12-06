
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := iptables
LOCAL_DESCRIPTION := Administration tool for IP packet filtering and NAT.
LOCAL_CATEGORY_PATH := network

LOCAL_AUTOTOOLS_VERSION := 1.4.21
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.bz2
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	iptables-1.4.21.patch \
	iptables-cgroup.patch

ifeq ($(TARGET_LIBC),musl)
	LOCAL_AUTOTOOLS_PATCHES += iptables-1.4.21-musl.patch
endif

# Do some manual cleanup
define LOCAL_AUTOTOOLS_CMD_POST_CLEAN
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/bin/iptables-xml
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/sbin/ip6tables*
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/sbin/iptables*
	$(Q) rm -rf $(TARGET_OUT_STAGING)/usr/libexec/xtables
endef

include $(BUILD_AUTOTOOLS)

