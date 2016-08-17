LOCAL_PATH := $(call my-dir)

###############################################################################
# dnsmasq
###############################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := dnsmasq
LOCAL_DESCRIPTION := Lightweight DNS forwarder and DHCP server
LOCAL_CATEGORY_PATH := network

LOCAL_AUTOTOOLS_VERSION := 2.66
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_CONFIG_FILES := Config.in
$(call load-config)

DNSMASQ_COPTS += \
	-DHAVE_BROKEN_RTC \
	-DNO_TFTP \
	-DNO_IPV6 \
	-DNO_LARGEFILE

LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	$(AUTOTOOLS_CONFIGURE_ENV) \
	COPTS="$(DNSMASQ_COPTS)" \
	PREFIX="$(AUTOTOOLS_CONFIGURE_PREFIX)"

LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	PREFIX="$(AUTOTOOLS_CONFIGURE_PREFIX)"

ifeq ("$(TARGET_OS_FLAVOUR)","android")
DNSMASQ_COPTS += -D__ANDROID__
LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS += LDFLAGS+=" -lcutils"
endif

define LOCAL_AUTOTOOLS_CMD_CONFIGURE
	$(empty)
endef

ifeq ($(CONFIG_DNSMASQ_SUPPORT_ULOGWRAPPER),y)
LOCAL_AUTOTOOLS_PATCHES:= ulogwrapper.patch
endif

LOCAL_CLEAN_FILES := \
	$(TARGET_OUT_STAGING)/usr/sbin/dnsmasq \
	$(TARGET_OUT_STAGING)/usr/share/man/man8/dnsmasq.8

include $(BUILD_AUTOTOOLS)
