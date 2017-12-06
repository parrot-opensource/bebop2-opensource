
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := libusb
LOCAL_DESCRIPTION := Userspace library for accessing USB devices
LOCAL_CATEGORY_PATH := libs

LOCAL_EXPORT_LDLIBS := -lusb

LOCAL_AUTOTOOLS_VERSION := 0.1.12
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE)-$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	libusb-0.1.12-abort_and_monotonic.patch \
	libusb-0.1.12-bus_loc_init.patch \
	libusb-0.1.12-nocpp.patch \
	libusb-0.1.12-raptor.patch \
	libusb-0.1.12-types.patch

LOCAL_AUTOTOOLS_CONFIGURE_ENV := \
	ac_cv_header_regex_h=no

LOCAL_AUTOTOOLS_CONFIGURE_ARGS := \
	--disable-debug \
	--disable-build-docs

ifeq ("$(TARGET_LIBC)","bionic")

LOCAL_AUTOTOOLS_MAKE_BUILD_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	CROSS=$(TARGET_CROSS)

LOCAL_AUTOTOOLS_MAKE_INSTALL_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	DESTDIR=$(TARGET_OUT_STAGING)/usr \
	CROSS=$(TARGET_CROSS)

define LOCAL_AUTOTOOLS_CMD_POST_UNPACK
	$(Q) cp -af $(PRIVATE_PATH)/config.guess $(PRIVATE_SRC_DIR)/config.guess
	$(Q) cp -af $(PRIVATE_PATH)/config.sub $(PRIVATE_SRC_DIR)/config.sub
endef

define LOCAL_AUTOTOOLS_CMD_POST_CONFIGURE
	$(Q) cp -af $(PRIVATE_PATH)/Makefile_raptor $(PRIVATE_SRC_DIR)/Makefile
endef

define LOCAL_AUTOTOOLS_CMD_POST_CLEAN
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/lib/libusb.so
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/lib/libusb.a
	$(Q) rm -f $(TARGET_OUT_STAGING)/usr/include/usb.h
endef

endif

include $(BUILD_AUTOTOOLS)

