
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE := wireless_tools
LOCAL_DESCRIPTION := Wireless configuration tools
LOCAL_CATEGORY_PATH := network/wireless

LOCAL_AUTOTOOLS_VERSION := 29
LOCAL_AUTOTOOLS_ARCHIVE := $(LOCAL_MODULE).$(LOCAL_AUTOTOOLS_VERSION).tar.gz
LOCAL_AUTOTOOLS_SUBDIR := $(LOCAL_MODULE).$(LOCAL_AUTOTOOLS_VERSION)

LOCAL_AUTOTOOLS_PATCHES := \
	wireless_tools.29-android.patch wireless_tools.29-warningsfix.patch

LOCAL_AUTOTOOLS_MAKE_BUILD_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_BUILD_ARGS := \
	CC="$(CCACHE) $(TARGET_CC)"

LOCAL_AUTOTOOLS_MAKE_INSTALL_ENV := \
	$(AUTOTOOLS_CONFIGURE_ENV)

LOCAL_AUTOTOOLS_MAKE_INSTALL_ARGS := \
	CC="$(CCACHE) $(TARGET_CC)" \
	PREFIX="$(TARGET_OUT_STAGING)" \
	INSTALL_INC="$(TARGET_OUT_STAGING)/usr/include" \
	INSTALL_MAN="$(TARGET_OUT_STAGING)/usr/share/man"

# Skip the configure step. The empty macro call is to make sure the step does
# nothing while shill being defined to something.
define LOCAL_AUTOTOOLS_CMD_CONFIGURE
	$(empty)
endef

include $(BUILD_AUTOTOOLS)

